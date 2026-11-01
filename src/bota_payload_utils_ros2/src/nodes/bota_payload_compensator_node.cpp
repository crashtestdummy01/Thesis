#include "bota_payload_utils/nodes/bota_payload_compensator_node.hpp"

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

BotaPayloadCompensatorNode::BotaPayloadCompensatorNode(const std::string &node_name)
    : Node(node_name), bota_payload_compensator_node_name_(node_name)
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

  // Initialize the bota payload compensator with the user-provided config file
  try
  {
    bota_payload_compensator_ = std::make_unique<bota::BotaPayloadCompensator>(config_file_);
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(get_logger(),
                 "Failed to initialize BotaPayloadCompensator with config file '%s': %s",
                 config_file_.c_str(), e.what());
    rclcpp::shutdown();
    return;
  }

  // Set up the subscriber to the input node's bota_frame topic
  subscription_bota_frame_ = create_subscription<bota_driver::msg::BotaFrame>(
      input_node_name_ + "/bota_frame", 10,
      std::bind(&BotaPayloadCompensatorNode::input_bota_frame_callback, this, std::placeholders::_1));

  // Set up the publishers with prefixed topic names
  publisher_bota_wrench_ =
      create_publisher<geometry_msgs::msg::WrenchStamped>(bota_payload_compensator_node_name_ + "/wrench", 10);
  publisher_bota_imu_ =
      create_publisher<sensor_msgs::msg::Imu>(bota_payload_compensator_node_name_ + "/imu", 10);
  publisher_bota_frame_ =
      create_publisher<bota_driver::msg::BotaFrame>(bota_payload_compensator_node_name_ + "/bota_frame", 10);

  // Create a separate callback group for the service
  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Add tare service with prefixed name and separate callback group
  tare_service_ = create_service<std_srvs::srv::Trigger>(
      bota_payload_compensator_node_name_ + "/tare_compensated_wrench",
      std::bind(&BotaPayloadCompensatorNode::tare_callback, this,
                std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS().get_rmw_qos_profile(),
      service_callback_group_);

  RCLCPP_INFO(get_logger(), "BotaPayloadCompensatorNode initialized successfully");
  RCLCPP_INFO(get_logger(), "Waiting for first message to perform auto-tare...");
}

BotaPayloadCompensatorNode::~BotaPayloadCompensatorNode()
{
  RCLCPP_INFO(get_logger(), "bota_payload_compensator_node shutting down");
}

void BotaPayloadCompensatorNode::input_bota_frame_callback(const bota_driver::msg::BotaFrame::SharedPtr msg)
{
  if (!bota_payload_compensator_)
  {
    return;
  }

  WrenchImuArray input = toWrenchImuArray(*msg);
  WrenchImuArray compensated{};
  BotaControlReturnCode rc = BotaControlReturnCode::FatalError;

  {
    std::lock_guard<std::mutex> lock(compensator_mutex_);

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

    // Auto-tare on the first message. The tare is non-blocking: if the library does not
    // have enough data yet it is deferred and update() returns Stale (zeros) until the
    // tare window has been collected.
    if (!auto_tare_done_)
    {
      auto_tare_done_ = true;
      RCLCPP_INFO(get_logger(), "First message received, requesting auto-tare...");
      try
      {
        if (!bota_payload_compensator_->tareCompensatedWrench())
        {
          RCLCPP_ERROR(get_logger(), "Auto-tare request failed");
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(get_logger(), "Auto-tare exception: %s", e.what());
      }
    }

    try
    {
      rc = bota_payload_compensator_->update(timestamp, input, compensated);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kThrottleMs,
                            "BotaPayloadCompensator::update() threw: %s", e.what());
      return;
    }
  }

  if (bota_payload_utils::isOutputUsable(rc))
  {
    if (rc != BotaControlReturnCode::OK)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kThrottleMs,
                           "Compensator returned %s", bota_payload_utils::toString(rc));
    }
    publish_frame_data(*msg, compensated);
  }
  else if (bota_payload_utils::isError(rc))
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kThrottleMs,
                          "Compensator returned %s, output not published", bota_payload_utils::toString(rc));
  }
  else
  {
    RCLCPP_DEBUG(get_logger(), "Compensator output stale (e.g. tare in progress), skipping publish");
  }
}

void BotaPayloadCompensatorNode::publish_frame_data(const bota_driver::msg::BotaFrame &input,
                                                    const WrenchImuArray &compensated)
{
  // Outputs carry the stamp of the input sample they were computed from
  const auto &stamp = input.wrench.header.stamp;

  // Wrench (compensated)
  geometry_msgs::msg::WrenchStamped wrench_msg;
  wrench_msg.header.stamp = stamp;
  wrench_msg.header.frame_id = input_node_name_ + "_wrench";  // the same frame as the input
  wrench_msg.wrench.force.x = compensated[idx::FORCE_X];
  wrench_msg.wrench.force.y = compensated[idx::FORCE_Y];
  wrench_msg.wrench.force.z = compensated[idx::FORCE_Z];
  wrench_msg.wrench.torque.x = compensated[idx::TORQUE_X];
  wrench_msg.wrench.torque.y = compensated[idx::TORQUE_Y];
  wrench_msg.wrench.torque.z = compensated[idx::TORQUE_Z];
  publisher_bota_wrench_->publish(wrench_msg);

  // Imu: the compensator only modifies the wrench, so the input IMU is forwarded
  sensor_msgs::msg::Imu imu_msg = input.imu;
  imu_msg.header.stamp = stamp;
  publisher_bota_imu_->publish(imu_msg);

  // Full frame: status flags, temperature and timestamp are taken from the input
  bota_driver::msg::BotaFrame bf_msg = input;
  bf_msg.wrench = wrench_msg;
  bf_msg.imu = imu_msg;
  publisher_bota_frame_->publish(bf_msg);
}

void BotaPayloadCompensatorNode::tare_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;  // Mark as unused

  RCLCPP_INFO(get_logger(), "Manual tare service called");

  if (!bota_payload_compensator_)
  {
    response->success = false;
    response->message = "Compensator not initialized";
    return;
  }

  try
  {
    std::lock_guard<std::mutex> lock(compensator_mutex_);

    // Non-blocking: completes immediately if enough data is buffered, otherwise it is
    // deferred and completes after the next updates.
    if (bota_payload_compensator_->tareCompensatedWrench())
    {
      RCLCPP_INFO(get_logger(), "Manual tare accepted");
      response->success = true;
      response->message = "Tare accepted";
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Manual tare operation failed");
      response->success = false;
      response->message = "Tare operation failed";
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to tare sensor: %s", e.what());
    response->success = false;
    response->message = "Failed to tare sensor: " + std::string(e.what());
  }
}

// Factory function to create node with dynamic name
std::shared_ptr<BotaPayloadCompensatorNode> create_bota_payload_compensator_node()
{
  // Create a temporary node to get the parameter
  auto temp_node = rclcpp::Node::make_shared("temp_bota_payload_compensator_node");

  std::string node_name;
  try
  {
    node_name = temp_node->declare_parameter<std::string>("node_name");
    RCLCPP_INFO(temp_node->get_logger(), "Using provided node name: '%s'", node_name.c_str());
  }
  catch (const std::exception &e)
  {
    // Use default value if parameter is not provided
    node_name = "bota_payload_compensator_node";
    RCLCPP_WARN(temp_node->get_logger(),
                "Parameter 'node_name' was not provided. Using default: '%s'",
                node_name.c_str());
  }

  // Create the actual node with the node name
  return std::make_shared<BotaPayloadCompensatorNode>(node_name);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = create_bota_payload_compensator_node();
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
