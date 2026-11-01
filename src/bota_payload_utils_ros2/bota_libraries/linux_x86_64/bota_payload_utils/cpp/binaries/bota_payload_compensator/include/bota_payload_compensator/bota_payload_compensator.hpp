/**
 * @file bota_payload_compensator.hpp
 * @brief Public header file for the BotaPayloadCompensator class.
 * @brief More information about the BotaPayloadCompensator class and its functionality can be found
 * in the documentation. https://code.botasys.com/en/gen_a/layer1/payload_utils/payload_utils.html
 *
 * @copyright
 * Copyright (c) 2025 Botasys AG
 * SPDX-License-Identifier: BSL3
 * The BotaPayloadCompensator is part of the Bota Payload Utils solution and is licensed under the
 * BSL3 License. See the LICENSE file accompanying this library for more information.
 */

#ifndef BOTA_PAYLOAD_COMPENSATOR_HPP
#define BOTA_PAYLOAD_COMPENSATOR_HPP

#include <array>
#include <string>
#include <functional>
#include <memory>

#include "bota_control/bota_control_definitions.hpp"

// Platform-specific DLL export/import macros
// (BOTA_PAYLOAD_UTILS_STATIC is predefined in the headers of the static package)
#if defined(_WIN32) && !defined(BOTA_PAYLOAD_UTILS_STATIC)
#ifdef BOTA_PAYLOAD_COMPENSATOR_EXPORTS
#define BOTA_PAYLOAD_COMPENSATOR_API __declspec(dllexport)
#else
#define BOTA_PAYLOAD_COMPENSATOR_API __declspec(dllimport)
#endif
#else
#define BOTA_PAYLOAD_COMPENSATOR_API
#endif

namespace bota {

class BOTA_PAYLOAD_COMPENSATOR_API BotaPayloadCompensator {
 public:
  // Constructor
  // BotaPayloadCompensator();
  explicit BotaPayloadCompensator(const std::string &config_path);

  // Destructor
  ~BotaPayloadCompensator();

  // Bota Control block interface method
  BotaControlReturnCode update(double timestamp, std::array<double, 12> &wrench_imu_raw, std::array<double, 12> &wrench_imu_compensated);

  // Service methods
  bool tareCompensatedWrench();

 private:
  // Implementation details - forward declaration
  class BotaPayloadCompensatorImpl;                   ///< Implementation class
  std::unique_ptr<BotaPayloadCompensatorImpl> impl_;  ///< Pointer to implementation
};

}  // namespace bota

#endif  // BOTA_PAYLOAD_COMPENSATOR_HPP