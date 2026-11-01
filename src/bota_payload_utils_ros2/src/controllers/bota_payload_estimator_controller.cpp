// Copyright (c) 2025 Bota Systems AG
//
// Licensed under BSD3

#include <bota_payload_utils/controllers/bota_payload_estimator_controller.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>

namespace bota_payload_utils_controllers
{

  using bota::BotaControlReturnCode;
  using bota_payload_utils::WrenchImuArray;
  namespace idx = bota_payload_utils::wrench_imu;

  namespace
  {
    constexpr int kThrottleMs = 2000;
  }

  BotaPayloadEstimatorController::BotaPayloadEstimatorController()
  {
    output_states_.resize(NUM_OUTPUT_STATES, 0.0);
  }

  controller_interface::InterfaceConfiguration BotaPayloadEstimatorController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::NONE;
    return config;
  }

  controller_interface::InterfaceConfiguration BotaPayloadEstimatorController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // State interfaces : INPUT from input_block_name_
    config.names = {
        input_block_name_ + "/status",      // 0
        input_block_name_ + "/force.x",     // 1
        input_block_name_ + "/force.y",     // 2
        input_block_name_ + "/force.z",     // 3
        input_block_name_ + "/torque.x",    // 4
        input_block_name_ + "/torque.y",    // 5
        input_block_name_ + "/torque.z",    // 6
        input_block_name_ + "/lin_acc.x",   // 7
        input_block_name_ + "/lin_acc.y",   // 8
        input_block_name_ + "/lin_acc.z",   // 9
        input_block_name_ + "/ang_vel.x",   // 10
        input_block_name_ + "/ang_vel.y",   // 11
        input_block_name_ + "/ang_vel.z",   // 12
        input_block_name_ + "/temperature", // 13
        input_block_name_ + "/timestamp",   // 14
    };

    return config;
  }

  std::vector<hardware_interface::CommandInterface> BotaPayloadEstimatorController::on_export_reference_interfaces()
  {
    return {};
  }

  std::vector<hardware_interface::StateInterface> BotaPayloadEstimatorController::on_export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> states;

    auto logger = get_node()->get_logger();

    if (output_states_.size() != NUM_OUTPUT_STATES)
    {
      RCLCPP_ERROR(logger, "Output states vector has wrong size: %zu, expected %zu.",
                   output_states_.size(), NUM_OUTPUT_STATES);
      return states;
    }
    if (output_state_names_.size() != NUM_OUTPUT_STATES)
    {
      RCLCPP_ERROR(logger, "Output state names vector has wrong size: %zu, expected %zu.",
                   output_state_names_.size(), NUM_OUTPUT_STATES);
      return states;
    }

    states.reserve(NUM_OUTPUT_STATES);
    for (size_t i = 0; i < output_state_names_.size(); ++i)
    {
      // Exposed as <controller_name>/estimation_running, <controller_name>/estimation_successful
      states.emplace_back(get_node()->get_name(), output_state_names_[i], &output_states_[i]);
    }

    return states;
  }

  bool BotaPayloadEstimatorController::on_set_chained_mode(bool activate)
  {
    is_in_chained_mode_ = activate;
    RCLCPP_INFO(get_node()->get_logger(), "BotaPayloadEstimatorController: chained mode set to %s", activate ? "true" : "false");
    return true;
  }

  double BotaPayloadEstimatorController::readState(size_t index) const
  {
    return state_interfaces_[index].get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
  }

  WrenchImuArray BotaPayloadEstimatorController::readInput() const
  {
    // The IMU is already expressed in the wrench frame by the driver, so it is used as is.
    WrenchImuArray a{};
    a[idx::FORCE_X]   = readState(StateIndices::INPUT_FORCE_X);
    a[idx::FORCE_Y]   = readState(StateIndices::INPUT_FORCE_Y);
    a[idx::FORCE_Z]   = readState(StateIndices::INPUT_FORCE_Z);
    a[idx::TORQUE_X]  = readState(StateIndices::INPUT_TORQUE_X);
    a[idx::TORQUE_Y]  = readState(StateIndices::INPUT_TORQUE_Y);
    a[idx::TORQUE_Z]  = readState(StateIndices::INPUT_TORQUE_Z);
    a[idx::LIN_ACC_X] = readState(StateIndices::INPUT_LIN_ACC_X);
    a[idx::LIN_ACC_Y] = readState(StateIndices::INPUT_LIN_ACC_Y);
    a[idx::LIN_ACC_Z] = readState(StateIndices::INPUT_LIN_ACC_Z);
    a[idx::ANG_VEL_X] = readState(StateIndices::INPUT_ANG_VEL_X);
    a[idx::ANG_VEL_Y] = readState(StateIndices::INPUT_ANG_VEL_Y);
    a[idx::ANG_VEL_Z] = readState(StateIndices::INPUT_ANG_VEL_Z);
    return a;
  }

  controller_interface::return_type
  BotaPayloadEstimatorController::update_and_write_commands(
      const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
  {
    auto &clock = *get_node()->get_clock();
    auto logger = get_node()->get_logger();

    if (!bota_payload_estimator_)
    {
      return controller_interface::return_type::OK;
    }

    // Start request coming from the service thread
    if (start_requested_.exchange(false))
    {
      try
      {
        if (bota_payload_estimator_->startPayloadEstimation())
        {
          RCLCPP_INFO(logger, "Payload estimation started");
        }
        else
        {
          RCLCPP_ERROR(logger, "Starting payload estimation failed");
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(logger, "Starting payload estimation threw: %s", e.what());
      }
    }

    // Time base: the ros2_control update time. update() needs strictly increasing timestamps.
    const double timestamp = time.seconds();
    if (timestamp > last_time_)
    {
      last_time_ = timestamp;
      try
      {
        const BotaControlReturnCode rc = bota_payload_estimator_->update(timestamp, readInput());
        if (bota_payload_utils::isError(rc))
        {
          RCLCPP_ERROR_THROTTLE(logger, clock, kThrottleMs, "Estimator returned %s",
                                bota_payload_utils::toString(rc));
        }
        else if (rc != BotaControlReturnCode::OK)
        {
          RCLCPP_WARN_THROTTLE(logger, clock, kThrottleMs, "Estimator returned %s",
                               bota_payload_utils::toString(rc));
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR_THROTTLE(logger, clock, kThrottleMs, "BotaPayloadEstimator::update() threw: %s", e.what());
      }
    }

    // Update output states with estimation status
    const bool running = bota_payload_estimator_->isEstimatorRunning();
    estimator_running_ = running;
    output_states_[OutputStateIndices::ESTIMATION_RUNNING] = running ? 1.0 : 0.0;
    output_states_[OutputStateIndices::ESTIMATION_SUCCESSFUL] =
        bota_payload_estimator_->isEstimationSuccessful() ? 1.0 : 0.0;

    return controller_interface::return_type::OK;
  }

  controller_interface::return_type BotaPayloadEstimatorController::update_reference_from_subscribers(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    return controller_interface::return_type::OK;
  }

  CallbackReturn BotaPayloadEstimatorController::on_init()
  {
    RCLCPP_INFO(get_node()->get_logger(), "BotaPayloadEstimatorController: on_init starting...");

    auto node = get_node();

    if (!node->has_parameter("input_block_name"))
    {
      node->declare_parameter("input_block_name", input_block_name_);
    }
    if (!node->has_parameter("config_file"))
    {
      node->declare_parameter("config_file", config_file_);
    }

    try
    {
      input_block_name_ = node->get_parameter("input_block_name").as_string();
      RCLCPP_INFO(node->get_logger(), "Using input_block_name: %s", input_block_name_.c_str());
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(node->get_logger(), "Could not get input_block_name parameter, using default: %s", input_block_name_.c_str());
    }

    try
    {
      config_file_ = node->get_parameter("config_file").as_string();
      RCLCPP_INFO(node->get_logger(), "Using config_file: %s", config_file_.c_str());
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(node->get_logger(), "Could not get config_file parameter, using default: %s", config_file_.c_str());
    }

    output_state_names_ = {
        "estimation_running",
        "estimation_successful"};

    // Create the estimator object
    try
    {
      bota_payload_estimator_ = std::make_unique<bota::BotaPayloadEstimator>(config_file_);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(node->get_logger(), "Failed to create estimator: %s", e.what());
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "BotaPayloadEstimatorController: on_init completed.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn BotaPayloadEstimatorController::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Create the service server
    start_payload_estimation_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
        "~/start_payload_estimation",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
          startPayloadEstimationCallback(request, response);
        });

    RCLCPP_INFO(get_node()->get_logger(), "BotaPayloadEstimatorController: on_configure completed.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn BotaPayloadEstimatorController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    last_time_ = -std::numeric_limits<double>::infinity();
    start_requested_ = false;
    is_active_ = true;

    RCLCPP_INFO(get_node()->get_logger(), "BotaPayloadEstimatorController: on_activate completed.");
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  BotaPayloadEstimatorController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    is_active_ = false;
    start_requested_ = false;

    RCLCPP_INFO(get_node()->get_logger(), "BotaPayloadEstimatorController: on_deactivate completed.");
    return CallbackReturn::SUCCESS;
  }

  void BotaPayloadEstimatorController::startPayloadEstimationCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!is_active_)
    {
      response->success = false;
      response->message = "Controller is not active";
      RCLCPP_WARN(get_node()->get_logger(), "start_payload_estimation service called but the controller is not active");
      return;
    }

    // Safety check: prevent starting estimation if it's already running
    if (estimator_running_ || start_requested_)
    {
      response->success = false;
      response->message = "Payload estimation is already running. Cannot start estimation again.";
      RCLCPP_WARN(get_node()->get_logger(), "start_payload_estimation service called but estimation is already running");
      return;
    }

    // The estimator is started from the real-time loop on its next cycle
    start_requested_ = true;

    response->success = true;
    response->message = "Payload estimation start requested";
    RCLCPP_INFO(get_node()->get_logger(), "start_payload_estimation service called - estimation start requested");
  }

} // namespace bota_payload_utils_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(
    bota_payload_utils_controllers::BotaPayloadEstimatorController,
    controller_interface::ChainableControllerInterface)