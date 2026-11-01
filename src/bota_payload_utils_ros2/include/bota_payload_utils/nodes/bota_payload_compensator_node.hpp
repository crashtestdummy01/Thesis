#ifndef BOTA_PAYLOAD_COMPENSATOR_NODE_HPP
#define BOTA_PAYLOAD_COMPENSATOR_NODE_HPP

#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "bota_driver/msg/bota_frame.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "bota_payload_compensator/bota_payload_compensator.hpp"
#include "bota_payload_utils/wrench_imu.hpp"

class BotaPayloadCompensatorNode : public rclcpp::Node
{
public:
  explicit BotaPayloadCompensatorNode(const std::string &node_name);
  ~BotaPayloadCompensatorNode() override;

private:
  // Callback for incoming bota frame messages: runs the compensator and publishes
  void input_bota_frame_callback(const bota_driver::msg::BotaFrame::SharedPtr msg);

  // Publishes the compensated wrench together with the (passed-through) input IMU
  void publish_frame_data(const bota_driver::msg::BotaFrame &input,
                          const bota_payload_utils::WrenchImuArray &compensated);

  // Service callback
  void tare_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Parameters
  std::string config_file_;
  std::string input_node_name_;
  std::string bota_payload_compensator_node_name_;

  // Bota Payload Compensator object (guarded by compensator_mutex_: the
  // subscription and the tare service run in different callback groups)
  std::unique_ptr<bota::BotaPayloadCompensator> bota_payload_compensator_;
  std::mutex compensator_mutex_;

  // Stamp (seconds) of the last sample fed to the compensator. The topic stamp is used as
  // time base; update() needs strictly increasing timestamps
  double last_stamp_{-std::numeric_limits<double>::infinity()};

  // Auto-tare on the first received message
  bool auto_tare_done_{false};

  // ROS2 subscribers
  rclcpp::Subscription<bota_driver::msg::BotaFrame>::SharedPtr subscription_bota_frame_;

  // ROS2 publishers
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_bota_wrench_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_bota_imu_;
  rclcpp::Publisher<bota_driver::msg::BotaFrame>::SharedPtr publisher_bota_frame_;

  // ROS2 service and its callback group
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tare_service_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
};

#endif  // BOTA_PAYLOAD_COMPENSATOR_NODE_HPP