#ifndef BOTA_CONTROL_ERROR_HANDLING_HPP
#define BOTA_CONTROL_ERROR_HANDLING_HPP

#include"bota_control_definitions.hpp"

namespace bota {

// -------------------------------------------------------------------------
// Condition types used internally to categorise what went wrong in a block.
//
// Policy mapping table — maps (condition, policy) → BotaControlReturnCode:
//
//  ┌────────────────────┬─────────────────┬──────────────────┬────────────────┬────────────────┐
//  │ Condition          │ BestEffort      │ SoftRt           │ HardRt         │ StrictRt       │
//  ├────────────────────┼─────────────────┼──────────────────┼────────────────┼────────────────┤
//  │ DeadlineMissed     │ OK              │ DeadlineMissed   │ FatalError     │ FatalError     │
//  │ StaleInput         │ Stale           │ Stale            │ Stale          │ FatalError     │
//  │ DegradedOutput     │ Degraded        │ Degraded         │ Degraded       │ FatalError     │
//  │ InternalError      │ NonFatalError   │ NonFatalError    │ FatalError     │ FatalError     │
//  │ FatalInternal      │ FatalError      │ FatalError       │ FatalError     │ FatalError     │
//  └────────────────────┴─────────────────┴──────────────────┴────────────────┴────────────────┘
// ---------------------------------------------------------------------------
enum struct BotaCondition {
  DeadlineMissed,  ///< Block did not produce output within the expected timing window.
  StaleInput,      ///< Input data has not been refreshed within the expected window.
  DegradedOutput,  ///< Output is valid but of reduced quality (e.g. warm-up phase of a filter).
  InternalError,   ///< Recoverable internal fault; the block can be reset and re-used.
  FatalInternal,   ///< Unrecoverable internal fault; the block must be discarded.
};

// ---------------------------------------------------------------------------
// Policy mapping helper
// ---------------------------------------------------------------------------

// Map a detected condition to a BotaControlReturnCode using the configured policy.
// FatalInternal always returns FatalError regardless of policy.
inline BotaControlReturnCode applyPolicy(BotaControlBlockPolicy policy,
                                         BotaCondition condition) noexcept {
  if (condition == BotaCondition::FatalInternal) {
    return BotaControlReturnCode::FatalError;
  }

  switch (policy) {
    case BotaControlBlockPolicy::BestEffort:
      switch (condition) {
        case BotaCondition::DeadlineMissed:
          return BotaControlReturnCode::OK;
        case BotaCondition::StaleInput:
          return BotaControlReturnCode::Stale;
        case BotaCondition::DegradedOutput:
          return BotaControlReturnCode::Degraded;
        case BotaCondition::InternalError:
          return BotaControlReturnCode::NonFatalError;
        default:
          break;
      }
      break;
    case BotaControlBlockPolicy::SoftRt:
      switch (condition) {
        case BotaCondition::DeadlineMissed:
          return BotaControlReturnCode::DeadlineMissed;
        case BotaCondition::StaleInput:
          return BotaControlReturnCode::Stale;
        case BotaCondition::DegradedOutput:
          return BotaControlReturnCode::Degraded;
        case BotaCondition::InternalError:
          return BotaControlReturnCode::NonFatalError;
        default:
          break;
      }
      break;
    case BotaControlBlockPolicy::HardRt:
      switch (condition) {
        case BotaCondition::DeadlineMissed:
          return BotaControlReturnCode::FatalError;
        case BotaCondition::StaleInput:
          return BotaControlReturnCode::Stale;
        case BotaCondition::DegradedOutput:
          return BotaControlReturnCode::Degraded;
        case BotaCondition::InternalError:
          return BotaControlReturnCode::FatalError;
        default:
          break;
      }
      break;
    case BotaControlBlockPolicy::StrictRt:
      // All non-OK conditions escalate to FatalError under strict_rt.
      return BotaControlReturnCode::FatalError;
  }

  return BotaControlReturnCode::FatalError;  // unreachable
}

}  // namespace bota

#endif  // BOTA_CONTROL_DEFINITIONS_HPP