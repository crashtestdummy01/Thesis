// Copyright (c) 2025 Bota Systems AG
//
// Licensed under BSD3

#include <bota_payload_utils/controllers/bota_payload_compensator_controller.hpp>

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

  BotaPayloadCompensatorController::BotaPayloadCompensatorController()
  {
    output_states_.resize(NUM_OUTPUT_STATES, 0.0);
  }

  controller_interface::InterfaceConfiguration BotaPayloadCompensatorController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::NONE;
    return config;
  }

  controller_interface::InterfaceConfiguration BotaPayloadCompensatorController::state_interface_configuration() const
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

  std::vector<hardware_interface::CommandInterface> BotaPayloadCompensatorController::on_export_reference_interfaces()
  {
    return {};
  }

  std::vector<hardware_interface::StateInterface> BotaPayloadCompensatorController::on_export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> states;

    auto logger = get_node()->get_logger();

    if (output_states_.size() != NUM_OUTPUT_STATES)
    {
      RCLCPP_ERROR(logger, "State outputs vector has wrong size: %zu, expected %zu.",
                   output_states_.size(), NUM_OUTPUT_STATES);
      return states;
    }
    if (output_state_names_.size() != NUM_OUTPUT_STATES)
    {
      RCLCPP_ERROR(logger, "State output names vector has wrong size: %zu, expected %zu.",
                   output_state_names_.size(), NUM_OUTPUT_STATES);
      return states;
    }

    states.reserve(NUM_OUTPUT_STATES);
    for (size_t i = 0; i < output_state_names_.size(); ++i)
    {
      states.emplace_back(get_node()->get_name(), output_state_names_[i], &output_states_[i]);
    }

    return states;
  }

  bool BotaPayloadCompensatorController::on_set_chained_mode(bool activate)
  {
    is_in_chained_mode_ = activate;
    RCLCPP_INFO(get_node()->get_logger(), "BotaPayloadCompensatorController: chained mode set to %s", activate ? "true" : "false");
    return true;
  }

  double BotaPayloadCompensatorController::readState(size_t index) const
  {
    return state_interfaces_[index].get_optional().value_or(std::numeric_limits<double>::quiet_NaN());
  }

  WrenchImuArray BotaPayloadCompensatorController::readInput() const
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
  BotaPayloadCompensatorController::update_and_write_commands(
      const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
  {
    auto &clock = *get_node()->get_clock();
    auto logger = get_node()->get_logger();

    if (!bota_payload_compensator_)
    {
      output_states_[OutputStateIndices::ALIVE] = 0.0;
      return controller_interface::return_type::OK;
    }

    // Watchdog: the input sensor timestamp must keep changing, otherwise alive drops to 0
    const double input_timestamp = readState(StateIndices::INPUT_TIMESTAMP);
    if (input_timestamp == last_input_timestamp_ || !std::isfinite(input_timestamp))
    {
      ++cycles_without_new_input_;
      if (stale_input_cycles_ > 0 && cycles_without_new_input_ >= stale_input_cycles_)
      {
        if (output_states_[OutputStateIndices::ALIVE] != 0.0)
        {
          RCLCPP_WARN_THROTTLE(logger, clock, kThrottleMs,
                               "Input timestamp unchanged for %ld cycles, alive set to 0",
                               static_cast<long>(cycles_without_new_input_));
        }
        output_states_[OutputStateIndices::ALIVE] = 0.0;
        return controller_interface::return_type::OK;
      }
    }
    else
    {
      last_input_timestamp_ = input_timestamp;
      cycles_without_new_input_ = 0;
    }

    // Time base: the ros2_control update time. update() needs strictly increasing timestamps.
    const double timestamp = time.seconds();
    if (timestamp <= last_time_)
    {
      return controller_interface::return_type::OK;
    }
    last_time_ = timestamp;

    WrenchImuArray input = readInput();
    WrenchImuArray compensated{};
    BotaControlReturnCode rc = BotaControlReturnCode::FatalError;

    try
    {
      rc = bota_payload_compensator_->update(timestamp, input, compensated);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR_THROTTLE(logger, clock, kThrottleMs, "BotaPayloadCompensator::update() threw: %s", e.what());
    }

    if (!bota_payload_utils::isOutputUsable(rc))
    {
      // Stale: tare is collecting its window (the library outputs zeros meanwhile).
      // Errors: do not forward anything. In both cases downstream sees alive = 0.
      std::fill(output_states_.begin(), output_states_.end(), 0.0);
      if (bota_payload_utils::isError(rc))
      {
        RCLCPP_ERROR_THROTTLE(logger, clock, kThrottleMs, "Compensator returned %s",
                              bota_payload_utils::toString(rc));
      }
      return controller_interface::return_type::OK;
    }

    if (rc != BotaControlReturnCode::OK)
    {
      RCLCPP_WARN_THROTTLE(logger, clock, kThrottleMs, "Compensator returned %s",
                           bota_payload_utils::toString(rc));
    }

    // Compensated wrench
    output_states_[OutputStateIndices::FORCE_X]  = compensated[idx::FORCE_X];
    output_states_[OutputStateIndices::FORCE_Y]  = compensated[idx::FORCE_Y];
    output_states_[OutputStateIndices::FORCE_Z]  = compensated[idx::FORCE_Z];
    output_states_[OutputStateIndices::TORQUE_X] = compensated[idx::TORQUE_X];
    output_states_[OutputStateIndices::TORQUE_Y] = compensated[idx::TORQUE_Y];
    output_states_[OutputStateIndices::TORQUE_Z] = compensated[idx::TORQUE_Z];

    // The compensator only modifies the wrench: IMU, status, temperature and timestamp are passed through
    output_states_[OutputStateIndices::STATUS]      = readState(StateIndices::INPUT_STATUS);
    output_states_[OutputStateIndices::LIN_ACC_X]   = input[idx::LIN_ACC_X];
    output_states_[OutputStateIndices::LIN_ACC_Y]   = input[idx::LIN_ACC_Y];
    output_states_[OutputStateIndices::LIN_ACC_Z]   = input[idx::LIN_ACC_Z];
    output_states_[OutputStateIndices::ANG_VEL_X]   = input[idx::ANG_VEL_X];
    output_states_[OutputStateIndices::ANG_VEL_Y]   = input[idx::ANG_VEL_Y];
    output_states_[OutputStateIndices::ANG_VEL_Z]   = input[idx::ANG_VEL_Z];
    output_states_[OutputStateIndices::TEMPERATURE] = readState(StateIndices::INPUT_TEMPERATURE);
    output_states_[OutputStateIndices::TIMESTAMP]   = input_timestamp;  // unchanged sensor timestamp

    output_states_[OutputStateIndices::ALIVE] = 1.0;

    return controller_interface::return_type::OK;
  }

  controller_interface::return_type BotaPayloadCompensatorController::update_reference_from_subscribers(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    return controller_interface::return_type::OK;
  }

  CallbackReturn BotaPayloadCompensatorController::on_init()
  {
    RCLCPP_INFO(get_node()->get_logger(), "BotaPayloadCompensatorController: on_init starting...");

    auto node = get_node();

    if (!node->has_parameter("input_block_name"))
    {
      node->declare_parameter("input_block_name", input_block_name_);
    }
    if (!node->has_parameter("config_file"))
    {
      node->declare_parameter("config_file", config_file_);
    }
    if (!node->has_parameter("tare_on_activation"))
    {
      node->declare_parameter("tare_on_activation", tare_on_activation_);
    }
    if (!node->has_parameter("stale_input_cycles"))
    {
      node->declare_parameter("stale_input_cycles", stale_input_cycles_);
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

    try
    {
      tare_on_activation_ = node->get_parameter("tare_on_activation").as_bool();
      RCLCPP_INFO(node->get_logger(), "Using tare_on_activation: %s",
                  tare_on_activation_ ? "true" : "false");
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(node->get_logger(), "Could not get tare_on_activation parameter, using default: %s",
                  tare_on_activation_ ? "true" : "false");
    }

    try
    {
      stale_input_cycles_ = node->get_parameter("stale_input_cycles").as_int();
      RCLCPP_INFO(node->get_logger(), "Using stale_input_cycles: %ld", static_cast<long>(stale_input_cycles_));
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(node->get_logger(), "Could not get stale_input_cycles parameter, using default: %ld",
                  static_cast<long>(stale_input_cycles_));
    }

    output_state_names_ = {
        "status",       // 0
        "force.x",      // 1
        "force.y",      // 2
        "force.z",      // 3
        "torque.x",     // 4
        "torque.y",     // 5
        "torque.z",     // 6
        "lin_acc.x",    // 7
        "lin_acc.y",    // 8
        "lin_acc.z",    // 9
        "ang_vel.x",    // 10
        "ang_vel.y",    // 11
        "ang_vel.z",    // 12
        "temperature",  // 13
        "timestamp",    // 14
        "alive",        // 15
    };

    RCLCPP_INFO(get_node()->get_logger(), "BotaPayloadCompensatorController: on_init completed.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn BotaPayloadCompensatorController::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(get_node()->get_logger(), "BotaPayloadCompensatorController: on_configure completed.");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn BotaPayloadCompensatorController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Re-read config_file parameter in case it changed
    try
    {
      config_file_ = get_node()->get_parameter("config_file").as_string();
      RCLCPP_INFO(get_node()->get_logger(), "Reading config_file on activation: %s", config_file_.c_str());
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN(get_node()->get_logger(), "Could not get config_file parameter on activation: %s", e.what());
    }

    // Create the compensator object with the (potentially updated) config file
    try
    {
      bota_payload_compensator_ = std::make_unique<bota::BotaPayloadCompensator>(config_file_);
      RCLCPP_INFO(get_node()->get_logger(), "Created BotaPayloadCompensator with config: %s", config_file_.c_str());
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to create compensator: %s", e.what());
      return CallbackReturn::ERROR;
    }

    // All outputs (including alive) stay 0 until the first usable compensated sample
    std::fill(output_states_.begin(), output_states_.end(), 0.0);
    last_time_ = -std::numeric_limits<double>::infinity();
    last_input_timestamp_ = std::numeric_limits<double>::quiet_NaN();
    cycles_without_new_input_ = 0;

    if (tare_on_activation_)
    {
      // Non-blocking: the library defers the tare until it has collected enough samples and
      // returns Stale (alive = 0) from update() until then.
      try
      {
        if (bota_payload_compensator_->tareCompensatedWrench())
        {
          RCLCPP_INFO(get_node()->get_logger(), "Tare requested on activation");
        }
        else
        {
          RCLCPP_ERROR(get_node()->get_logger(), "Tare request on activation failed");
        }
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Tare request on activation threw: %s", e.what());
      }
    }

    RCLCPP_INFO(get_node()->get_logger(), "BotaPayloadCompensatorController: on_activate completed.");
    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn
  BotaPayloadCompensatorController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Signal downstream controllers immediately before doing anything else
    output_states_[OutputStateIndices::ALIVE] = 0.0;

    // Destroy the compensator object to release resources
    bota_payload_compensator_.reset();
    RCLCPP_INFO(get_node()->get_logger(), "Destroyed BotaPayloadCompensator");

    RCLCPP_INFO(get_node()->get_logger(), "BotaPayloadCompensatorController: on_deactivate completed.");
    return CallbackReturn::SUCCESS;
  }

} // namespace bota_payload_utils_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(
    bota_payload_utils_controllers::BotaPayloadCompensatorController,
    controller_interface::ChainableControllerInterface)