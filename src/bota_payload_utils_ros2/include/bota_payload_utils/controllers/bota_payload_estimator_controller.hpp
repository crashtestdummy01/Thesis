// Copyright (c) 2025 Bota Systems AG
//
// Licensed under BSD3

#pragma once

#include <atomic>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/chainable_controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>  // For CallbackReturn
#include <rclcpp_lifecycle/state.hpp>                                    // For rclcpp_lifecycle::State
#include <std_srvs/srv/trigger.hpp>

#include "bota_payload_estimator/bota_payload_estimator.hpp"
#include "bota_payload_utils/wrench_imu.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace bota_payload_utils_controllers
{

    /**
     * Payload estimator controller
     *
     * The start service only sets a request flag; the estimator itself is only
     * touched from the real-time update loop.
     */
    class BotaPayloadEstimatorController : public controller_interface::ChainableControllerInterface
    {
    public:
        BotaPayloadEstimatorController();

        [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
            const override;
        [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
            const override;

        controller_interface::return_type update_and_write_commands(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        CallbackReturn on_init() override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        // Exports reference interfaces (the output of this controller for chaining)
        std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

        // Exports state interfaces (read-only data for monitoring or other controllers)
        std::vector<hardware_interface::StateInterface> on_export_state_interfaces() override;

        // Called when the controller enters/exits chained mode
        bool on_set_chained_mode(bool activate) override;

    protected:
        controller_interface::return_type update_reference_from_subscribers(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        static constexpr size_t NUM_OUTPUT_STATES = 2;

        // Indices of the claimed input state interfaces
        struct StateIndices {
            static constexpr size_t INPUT_STATUS = 0;
            static constexpr size_t INPUT_FORCE_X = 1;
            static constexpr size_t INPUT_FORCE_Y = 2;
            static constexpr size_t INPUT_FORCE_Z = 3;
            static constexpr size_t INPUT_TORQUE_X = 4;
            static constexpr size_t INPUT_TORQUE_Y = 5;
            static constexpr size_t INPUT_TORQUE_Z = 6;
            static constexpr size_t INPUT_LIN_ACC_X = 7;
            static constexpr size_t INPUT_LIN_ACC_Y = 8;
            static constexpr size_t INPUT_LIN_ACC_Z = 9;
            static constexpr size_t INPUT_ANG_VEL_X = 10;
            static constexpr size_t INPUT_ANG_VEL_Y = 11;
            static constexpr size_t INPUT_ANG_VEL_Z = 12;
            static constexpr size_t INPUT_TEMPERATURE = 13;
            static constexpr size_t INPUT_TIMESTAMP = 14;
        };

        // Indices of the exported output state interfaces
        struct OutputStateIndices {
            static constexpr size_t ESTIMATION_RUNNING    = 0;
            static constexpr size_t ESTIMATION_SUCCESSFUL = 1;
        };

        // Reads one claimed state interface (NaN if it has no value)
        double readState(size_t index) const;

        // Copies the current input state interfaces into the library's 12-element layout
        bota_payload_utils::WrenchImuArray readInput() const;

        // Service callback
        void startPayloadEstimationCallback(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        // Controller parameters from YAML
        std::string input_block_name_{"DEFAULT_INPUT_BLOCK"};
        std::string config_file_{"DEFAULT_CONFIG_FILE"};

        // Pointer to the BotaPayloadEstimator object
        std::unique_ptr<bota::BotaPayloadEstimator> bota_payload_estimator_;

        // Time (seconds) of the last update() call. The ros2_control update time is the time
        // base; update() needs strictly increasing timestamps
        double last_time_{-std::numeric_limits<double>::infinity()};

        // Hand-over between the service thread and the real-time loop
        std::atomic<bool> start_requested_{false};
        std::atomic<bool> estimator_running_{false};
        std::atomic<bool> is_active_{false};

        // Service server
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_payload_estimation_service_;

        bool is_in_chained_mode_ = false;

        std::vector<double> output_states_;
        std::vector<std::string> output_state_names_;
    };

} // namespace bota_payload_utils_controllers