#ifndef BOTA_PAYLOAD_ESTIMATOR_NODE_HPP
#define BOTA_PAYLOAD_ESTIMATOR_NODE_HPP

#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "bota_driver/msg/bota_frame.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"

#include "bota_payload_estimator/bota_payload_estimator.hpp"
#include "bota_payload_utils/wrench_imu.hpp"

class BotaPayloadEstimatorNode : public rclcpp::Node
{
public:
  explicit BotaPayloadEstimatorNode(const std::string &node_name);
  ~BotaPayloadEstimatorNode() override;

private:
  // Callback for incoming bota frame messages: feeds the estimator and publishes its status
  void input_bota_frame_callback(const bota_driver::msg::BotaFrame::SharedPtr msg);

  // Service callback
  void start_payload_estimation_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Parameters
  std::string config_file_;
  std::string input_node_name_;
  std::string bota_payload_estimator_node_name_;

  // Bota Payload Estimator object (guarded by estimator_mutex_: the subscription
  // and the start service run in different callback groups)
  std::unique_ptr<bota::BotaPayloadEstimator> bota_payload_estimator_;
  std::mutex estimator_mutex_;

  // Stamp (seconds) of the last sample fed to the estimator. The topic stamp is used as
  // time base; update() needs strictly increasing timestamps
  double last_stamp_{-std::numeric_limits<double>::infinity()};

  // ROS2 subscribers
  rclcpp::Subscription<bota_driver::msg::BotaFrame>::SharedPtr subscription_bota_frame_;

  // ROS2 publishers
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_is_estimator_running_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_is_estimation_successful_;

  // ROS2 service and its callback group
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_payload_estimation_service_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
};

#endif  // BOTA_PAYLOAD_ESTIMATOR_NODE_HPP