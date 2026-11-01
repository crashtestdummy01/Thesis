#ifndef BOTA_CONTROL_DEFINITIONS_HPP
#define BOTA_CONTROL_DEFINITIONS_HPP

#include <cstdint>

namespace bota {

// Return code shared across all BOTA control blocks.
enum struct BotaControlReturnCode {
  // Normal operation - caller can proceed as usual
  OK,

  // Warning - caller can proceed but should monitor the situation and react accordingly
  Stale,     // The block did not produce an updated output; it will hold the last value.
  Degraded,  // The block produced an output but it is degraded in some way, e.g. a filter is not
             // fully warmed up yet.
  DeadlineMissed,  // The block did not produce an output within the expected time frame, but it may
                   // still produce an output in the future if the situation is transient.

  // Error - caller should react to the error, either try to recover or shutdown accordingly
  NonFatalError,  // Non-fatal error - the controller could recover by resetting the block.
  FatalError,     // Fatal error - the controller cannot continue without external intervention; the
                  // loop should stop.
};

// Runtime policy that governs how each condition is mapped to a BotaControlReturnCode.
// Select the policy that matches the real-time requirements of the application.
enum struct BotaControlBlockPolicy {
  BestEffort,  // Tolerant policy: only truly unrecoverable faults escalate to FatalError.
  SoftRt,      // Soft real-time: deadline misses are reported but not fatal.
  HardRt,   // Hard real-time: deadline misses and internal errors are fatal; stale/degraded outputs
            // are still reported as warnings.
  StrictRt  // Strictest policy: any non-OK condition becomes FatalError.
};

// Return codes used by Bota Control hardware drivers.
enum struct BotaControlHWReturnCode : uint8_t
{
    OK = 0,
    StaleBufferRead,  
    StaleBufferWrite,
    Timeout,           
    InvalidDriverState,
    InvalidDriverMode,
    InvalidCommand,
    FatalError
};

}  // namespace bota

#endif  // BOTA_CONTROL_DEFINITIONS_HPP