#ifndef BOTA_DRIVER_CONFIG_HPP
#define BOTA_DRIVER_CONFIG_HPP

#include <cstdint>
#include <optional>
#include <string>

namespace bota {

// =============================================================================
// bota_driver_config.hpp
//
// Use this header to create a BotaDriverConfig object programmatically instead
// of loading a JSON file. The field names mirror the JSON config exactly.
//
// Quick-start:
//
//   driver_config = BotaDriverConfig{
//     .product_name                          = "BFT-SENS-SER-M8",
//     .serial_number                         = "SN000853",
//     .communication_interface_name          = "Bota_Binary_gen0",
//     .binary_communication_interface_params = { .com_port = "/dev/ttyUSB0" },
//     .gen0_sensor_operation_params          = { .sinc_length = 112 },
//   });
//
// Pairing rules (same as JSON). Set the one *_communication_interface_params field and the one
// *_sensor_operation_params field that your communication_interface_name calls for:
//
//   communication_interface_name   *_communication_interface_params   *_sensor_operation_params
//   ----------------------------   --------------------------------   -------------------------
//   "Bota_Binary_gen0"             binary                             gen0
//   "Bota_Binary"                  binary                             genA
//   "Bota_Socket"                  socket                             genA
//   "CANopen_over_EtherCAT_gen0"   coe                                gen0
//   "CANopen_over_EtherCAT"        coe                                genA
//   "Bota_CAN"                     can                                genA
//   "Bota_CAN_FD"                  can                                genA
//   "Bota_CAN_Robot"               can                                genA
//   "Bota_Dummy"                   dummy                              genA
//
//   The "Bota_CAN*" interfaces are built on SocketCAN and are available on Linux only.
//
// Mandatory vs optional fields:
//
//   Mandatory — must be set explicitly, no default:
//     binary_communication_interface_params.com_port
//     socket_communication_interface_params.sensor_ip_address
//     coe_communication_interface_params.network_interface
//     can_communication_interface_params.can_interface
//     can_communication_interface_params.device_id
//     gen0_sensor_operation_params.sinc_length
//     genA_sensor_operation_params.app_mode
//     genA_sensor_operation_params.app_submode
//
//   Optional — omit to use the default or keep the sensor value:
//     binary_communication_interface_params.baudrate  (default: 460800)
//     gen0_sensor_operation_params.wrench_offset      (nullopt = do not overwrite sensor value)
//     genA_sensor_operation_params.wrench_offset      (nullopt = do not overwrite sensor value)
//     genA_sensor_operation_params.temperature_coefficients (nullopt = do not overwrite sensor
//     value)
//
// =============================================================================

// -----------------------------------------------------------------------------
// Shared types
// -----------------------------------------------------------------------------

/**
 * @brief Force/torque offset applied to all readings. All components default to zero.
 *        Maps to JSON key "wrench_offset": { "fx", "fy", "fz", "tx", "ty", "tz" }.
 */
struct WrenchOffset {
  float fx = 0.f;
  float fy = 0.f;
  float fz = 0.f;
  float tx = 0.f;
  float ty = 0.f;
  float tz = 0.f;
};

/**
 * @brief Temperature compensation coefficients for all readings. All components default to zero.
 *        Maps to JSON key "temperature_coefficients": { "fx", "fy", "fz", "tx", "ty", "tz" }.
 */
struct TemperatureCoefficients {
  float fx = 0.f;
  float fy = 0.f;
  float fz = 0.f;
  float tx = 0.f;
  float ty = 0.f;
  float tz = 0.f;
};

// -----------------------------------------------------------------------------
// Communication interface params
// Populate exactly ONE of the structs below, matching
// your communication_interface_name.
// -----------------------------------------------------------------------------

/**
 * @brief Serial communication parameters.
 *        Use with: "Bota_Binary_gen0" or "Bota_Binary".
 *        Maps to JSON key "communication_interface_params".
 */
struct BotaBinaryCommunicationInterfaceConfig {
  std::string com_port;            ///< mandatory — e.g. "/dev/ttyUSB0" or "COM7"
  unsigned int baudrate = 460800;  ///< optional  — default: 460800
};

/**
 * @brief Ethernet/socket communication parameters.
 *        Use with: "Bota_Socket".
 *        Maps to JSON key "communication_interface_params".
 */
struct BotaSocketCommunicationInterfaceConfig {
  std::string sensor_ip_address;  ///< mandatory — e.g. "192.168.0.45"
};

/**
 * @brief EtherCAT communication parameters.
 *        Use with: "CANopen_over_EtherCAT_gen0" or "CANopen_over_EtherCAT".
 *        Maps to JSON key "communication_interface_params".
 */
struct CoeCommunicationInterfaceConfig {
  std::string network_interface;  ///< mandatory — e.g. "eth0" or "en0"
};

/**
 * @brief SocketCAN communication parameters.
 *        Use with: "Bota_CAN", "Bota_CAN_FD" or "Bota_CAN_Robot".
 *        Maps to JSON key "communication_interface_params".
 * @note  SocketCAN is Linux only; on macOS and Windows these interfaces are rejected.
 */
struct BotaCanCommunicationInterfaceConfig {
  std::string can_interface;  ///< mandatory — e.g. "can0" or "vcan0"
  /// mandatory — DeviceID field of the 29-bit CAN ID. For "Bota_CAN_Robot" it also
  /// selects the standard IDs of the stream (1 -> 0x600, 2 -> 0x610, ... 10 -> 0x690,
  /// 11 -> 0x700, ... 20 -> 0x790) and must therefore be 1..20.
  std::optional<uint8_t> device_id;
};

/**
 * @brief Dummy communication interface parameters.
 *        Use with: "Bota_Dummy".
 *        Maps to JSON key "communication_interface_params".
 */
struct BotaDummyCommunicationInterfaceConfig {
  // No parameters for the dummy interface
};

// -----------------------------------------------------------------------------
// Sensor operation params
// Populate exactly ONE of the two structs below:
//   - Gen0SensorOperationConfig  →  Gen 0 sensors (Bota_Binary_gen0, CANopen_over_EtherCAT_gen0)
//   - GenASensorOperationConfig  →  Gen A sensors (Bota_Binary, Bota_Socket, Bota_CAN,
//                                   Bota_CAN_Robot, CANopen_over_EtherCAT)
// -----------------------------------------------------------------------------

/**
 * @brief Sensor operation parameters for gen0 sensors.
 *        Use with: "Bota_Binary_gen0" or "CANopen_over_EtherCAT_gen0".
 *        Maps to JSON key "sensor_operation_params".
 */
struct Gen0SensorOperationConfig {
  std::optional<uint16_t> sinc_length;  ///< mandatory — valid range depends on interface
  std::optional<WrenchOffset>
      wrench_offset;  ///< optional  — nullopt = do not overwrite sensor value
};

/**
 * @brief Sensor operation parameters for genA sensors (One-series and above).
 *        Use with: "Bota_Binary", "Bota_Socket", or "CANopen_over_EtherCAT".
 *        Maps to JSON key "sensor_operation_params".
 */
struct GenASensorOperationConfig {
  std::optional<uint8_t> app_mode;     ///< mandatory — no default
  std::optional<uint8_t> app_submode;  ///< mandatory — no default

  /// optional — nullopt = keep the value already stored on the sensor.
  /// Set to overwrite — the driver will warn you when it does so.
  std::optional<WrenchOffset> wrench_offset;

  /// optional — nullopt = keep the value already stored on the sensor.
  /// Set to overwrite — the driver will warn you when it does so.
  std::optional<TemperatureCoefficients> temperature_coefficients;
};

// -----------------------------------------------------------------------------
// Driver operation params
// Optional — omit the field to use the defaults shown below.
// Maps to JSON key "driver_operation_params".
// -----------------------------------------------------------------------------

/**
 * @brief Controls driver-level behaviour (logging, daemon, metadata).
 *        All fields match the JSON key names exactly and carry the same defaults.
 */
struct DriverOperationConfig {
  bool runtime_verbosity = true;
  bool register_with_daemon = false;
  bool skip_metadata_reading = false;
};

// -----------------------------------------------------------------------------
// Top-level config  — the object you pass to BotaDriver(...)
// -----------------------------------------------------------------------------

/**
 * @brief Complete inline configuration for BotaDriver.
 *
 * Field names and nesting mirror the JSON config file so that anyone familiar
 * with the JSON can read or write this struct without extra documentation.
 *
 * Rules:
 *  - Set exactly one of the *_communication_interface_params fields.
 *  - Set exactly one of gen0_sensor_operation_params / genA_sensor_operation_params.
 *  - The chosen pair must be consistent with communication_interface_name (see table above).
 *  - driver_operation_params is optional; defaults match the JSON parser defaults.
 */
struct BotaDriverConfig {
  // ── Identity ────────────────────────────────────────────────────────────────
  std::string product_name;
  std::string serial_number;
  std::string communication_interface_name;
  std::optional<bool> is_mn1s;  ///< Leave empty if not applicable

  // ── communication_interface_params — set exactly one ────────────────────────
  std::optional<BotaBinaryCommunicationInterfaceConfig> binary_communication_interface_params;
  std::optional<BotaSocketCommunicationInterfaceConfig> socket_communication_interface_params;
  std::optional<CoeCommunicationInterfaceConfig> coe_communication_interface_params;
  std::optional<BotaCanCommunicationInterfaceConfig> can_communication_interface_params;
  std::optional<BotaDummyCommunicationInterfaceConfig> dummy_communication_interface_params;

  // ── sensor_operation_params — set exactly one ────────────────────────────────
  std::optional<Gen0SensorOperationConfig> gen0_sensor_operation_params;
  std::optional<GenASensorOperationConfig> genA_sensor_operation_params;

  // ── driver_operation_params — optional, has defaults ─────────────────────────
  DriverOperationConfig driver_operation_params{};
};

}  // namespace bota

#endif  // BOTA_DRIVER_CONFIG_HPP