#include "bota_payload_utils/nodes/bota_payload_estimator_node.hpp"

#include <exception>

using bota::BotaControlReturnCode;
using bota_payload_utils::WrenchImuArray;
namespace idx = bota_payload_utils::wrench_imu;

namespace
{
constexpr int kThrottleMs = 2000;

// The driver already publishes the IMU in the wrench frame, so the values are
// forwarded unchanged.
WrenchImuArray toWrenchImuArray(const bota_driver::msg::BotaFrame &msg)
{
  WrenchImuArray a{};
  a[idx::FORCE_X]   = msg.wrench.wrench.force.x;
  a[idx::FORCE_Y]   = msg.wrench.wrench.force.y;
  a[idx::FORCE_Z]   = msg.wrench.wrench.force.z;
  a[idx::TORQUE_X]  = msg.wrench.wrench.torque.x;
  a[idx::TORQUE_Y]  = msg.wrench.wrench.torque.y;
  a[idx::TORQUE_Z]  = msg.wrench.wrench.torque.z;
  a[idx::LIN_ACC_X] = msg.imu.linear_acceleration.x;
  a[idx::LIN_ACC_Y] = msg.imu.linear_acceleration.y;
  a[idx::LIN_ACC_Z] = msg.imu.linear_acceleration.z;
  a[idx::ANG_VEL_X] = msg.imu.angular_velocity.x;
  a[idx::ANG_VEL_Y] = msg.imu.angular_velocity.y;
  a[idx::ANG_VEL_Z] = msg.imu.angular_velocity.z;
  return a;
}
}  // namespace

BotaPayloadEstimatorNode::BotaPayloadEstimatorNode(const std::string &node_name)
    : Node(node_name), bota_payload_estimator_node_name_(node_name)
{
  // Get mandatory config file parameter
  try
  {
    config_file_ = declare_parameter<std::string>("config_file");
    RCLCPP_INFO(get_logger(), "Using config file: '%s'", config_file_.c_str());
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(get_logger(),
                 "Required parameter 'config_file' was not provided. "
                 "Please set it in your launch file or via --ros-args -p config_file:=<path>");
    rclcpp::shutdown();
    return;
  }

  // Get mandatory input node name parameter
  try
  {
    input_node_name_ = declare_parameter<std::string>("input_node_name");
    RCLCPP_INFO(get_logger(), "Using input node name: '%s'", input_node_name_.c_str());
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(get_logger(),
                 "Required parameter 'input_node_name' was not provided. "
                 "Please set it in your launch file or via --ros-args -p input_node_name:=<node_name>");
    rclcpp::shutdown();
    return;
  }

  // Initialize the bota payload estimator with the user-provided config file
  try
  {
    bota_payload_estimator_ = std::make_unique<bota::BotaPayloadEstimator>(config_file_);
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(get_logger(),
                 "Failed to initialize BotaPayloadEstimator with config file '%s': %s",
                 config_file_.c_str(), e.what());
    rclcpp::shutdown();
    return;
  }

  // Set up the subscriber to the input node's bota_frame topic
  subscription_bota_frame_ = create_subscription<bota_driver::msg::BotaFrame>(
      input_node_name_ + "/bota_frame", 10,
      std::bind(&BotaPayloadEstimatorNode::input_bota_frame_callback, this, std::placeholders::_1));

  // Set up the publishers with prefixed topic names
  publisher_is_estimator_running_ =
      create_publisher<std_msgs::msg::Bool>(bota_payload_estimator_node_name_ + "/is_estimator_running", 10);
  publisher_is_estimation_successful_ =
      create_publisher<std_msgs::msg::Bool>(bota_payload_estimator_node_name_ + "/is_estimation_successful", 10);

  // Create a separate callback group for the service
  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Add start service with prefixed name and separate callback group
  start_payload_estimation_service_ = create_service<std_srvs::srv::Trigger>(
      bota_payload_estimator_node_name_ + "/start_payload_estimation",
      std::bind(&BotaPayloadEstimatorNode::start_payload_estimation_callback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(),
      service_callback_group_);

  RCLCPP_INFO(get_logger(), "BotaPayloadEstimatorNode initialized successfully");
}

BotaPayloadEstimatorNode::~BotaPayloadEstimatorNode()
{
  RCLCPP_INFO(get_logger(), "bota_payload_estimator_node shutting down");
}

void BotaPayloadEstimatorNode::input_bota_frame_callback(const bota_driver::msg::BotaFrame::SharedPtr msg)
{
  if (!bota_payload_estimator_)
  {
    return;
  }

  const WrenchImuArray input = toWrenchImuArray(*msg);

  std_msgs::msg::Bool is_running_msg;
  std_msgs::msg::Bool is_successful_msg;

  {
    std::lock_guard<std::mutex> lock(estimator_mutex_);

    // Time base: the stamp of the incoming topic (wrench header). update() needs strictly
    // increasing timestamps, so a repeated or older stamp is dropped.
    const double timestamp = rclcpp::Time(msg->wrench.header.stamp).seconds();
    if (timestamp <= last_stamp_)
    {
      RCLCPP_DEBUG(get_logger(), "Dropping sample with non-increasing stamp (%.9f <= %.9f)",
                   timestamp, last_stamp_);
      return;
    }
    last_stamp_ = timestamp;

    try
    {
      const BotaControlReturnCode rc = bota_payload_estimator_->update(timestamp, input);
      if (bota_payload_utils::isError(rc))
      {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kThrottleMs,
                              "Estimator returned %s", bota_payload_utils::toString(rc));
      }
      else if (rc != BotaControlReturnCode::OK)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kThrottleMs,
                             "Estimator returned %s", bota_payload_utils::toString(rc));
      }
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kThrottleMs,
                            "BotaPayloadEstimator::update() threw: %s", e.what());
      return;
    }

    is_running_msg.data = bota_payload_estimator_->isEstimatorRunning();
    is_successful_msg.data = bota_payload_estimator_->isEstimationSuccessful();
  }

  // Publish whether the estimator is running and whether the last estimation was successful
  publisher_is_estimator_running_->publish(is_running_msg);
  publisher_is_estimation_successful_->publish(is_successful_msg);
}

void BotaPayloadEstimatorNode::start_payload_estimation_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;  // Mark as unused

  if (!bota_payload_estimator_)
  {
    response->success = false;
    response->message = "Estimator not initialized";
    return;
  }

  try
  {
    std::lock_guard<std::mutex> lock(estimator_mutex_);

    // Check if estimator is already running
    if (bota_payload_estimator_->isEstimatorRunning())
    {
      RCLCPP_WARN(get_logger(), "Payload estimation is already running. Cannot start a new estimation.");
      response->success = false;
      response->message = "Payload estimation is already running";
      return;
    }

    // Call the start method
    if (bota_payload_estimator_->startPayloadEstimation())
    {
      RCLCPP_INFO(get_logger(), "Payload estimation started successfully");
      response->success = true;
      response->message = "Payload estimation started successfully";
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Starting payload estimation failed");
      response->success = false;
      response->message = "Starting payload estimation failed";
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to start payload estimation: %s", e.what());
    response->success = false;
    response->message = "Failed to start payload estimation: " + std::string(e.what());
  }
}

// Factory function to create node with dynamic name
std::shared_ptr<BotaPayloadEstimatorNode> create_bota_payload_estimator_node()
{
  // Create a temporary node to get the parameter
  auto temp_node = rclcpp::Node::make_shared("temp_bota_payload_estimator_node");

  std::string node_name;
  try
  {
    node_name = temp_node->declare_parameter<std::string>("node_name");
    RCLCPP_INFO(temp_node->get_logger(), "Using provided node name: '%s'", node_name.c_str());
  }
  catch (const std::exception &e)
  {
    // Use default value if parameter is not provided
    node_name = "bota_payload_estimator_node";
    RCLCPP_WARN(temp_node->get_logger(),
                "Parameter 'node_name' was not provided. Using default: '%s'",
                node_name.c_str());
  }

  // Create the actual node with the node name
  return std::make_shared<BotaPayloadEstimatorNode>(node_name);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = create_bota_payload_estimator_node();
  if (!node)
  {
    rclcpp::shutdown();
    return 1;
  }
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
