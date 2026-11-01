// Copyright (c) 2025 Bota Systems AG
//
// Licensed under BSD3
//
// Helpers shared by the nodes and the controllers for the array-based
// BotaPayloadCompensator / BotaPayloadEstimator API.

#ifndef BOTA_PAYLOAD_UTILS_WRENCH_IMU_HPP
#define BOTA_PAYLOAD_UTILS_WRENCH_IMU_HPP

#include <array>
#include <cstddef>

#include "bota_control/bota_control_definitions.hpp"

namespace bota_payload_utils
{

// Layout of the 12-element array passed to update():
//   [ Fx Fy Fz | Tx Ty Tz | ax ay az | wx wy wz ]
// The IMU values must already be expressed in the wrench frame.
using WrenchImuArray = std::array<double, 12>;

namespace wrench_imu
{
constexpr std::size_t FORCE_X   = 0;
constexpr std::size_t FORCE_Y   = 1;
constexpr std::size_t FORCE_Z   = 2;
constexpr std::size_t TORQUE_X  = 3;
constexpr std::size_t TORQUE_Y  = 4;
constexpr std::size_t TORQUE_Z  = 5;
constexpr std::size_t LIN_ACC_X = 6;
constexpr std::size_t LIN_ACC_Y = 7;
constexpr std::size_t LIN_ACC_Z = 8;
constexpr std::size_t ANG_VEL_X = 9;
constexpr std::size_t ANG_VEL_Y = 10;
constexpr std::size_t ANG_VEL_Z = 11;
}  // namespace wrench_imu

// True when the block produced an output that can be forwarded downstream.
// Stale is excluded: the compensator returns it while a deferred tare is
// collecting its window (and outputs zeros meanwhile).
inline bool isOutputUsable(bota::BotaControlReturnCode rc) noexcept
{
  return rc == bota::BotaControlReturnCode::OK ||
         rc == bota::BotaControlReturnCode::Degraded ||
         rc == bota::BotaControlReturnCode::DeadlineMissed;
}

inline bool isError(bota::BotaControlReturnCode rc) noexcept
{
  return rc == bota::BotaControlReturnCode::NonFatalError ||
         rc == bota::BotaControlReturnCode::FatalError;
}

inline const char *toString(bota::BotaControlReturnCode rc) noexcept
{
  switch (rc)
  {
    case bota::BotaControlReturnCode::OK:             return "OK";
    case bota::BotaControlReturnCode::Stale:          return "Stale";
    case bota::BotaControlReturnCode::Degraded:       return "Degraded";
    case bota::BotaControlReturnCode::DeadlineMissed: return "DeadlineMissed";
    case bota::BotaControlReturnCode::NonFatalError:  return "NonFatalError";
    case bota::BotaControlReturnCode::FatalError:     return "FatalError";
  }
  return "Unknown";
}


}  // namespace bota_payload_utils

#endif  // BOTA_PAYLOAD_UTILS_WRENCH_IMU_HPP