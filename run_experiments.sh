#!/usr/bin/env bash
set -uo pipefail

# Enable job control so the background logger can receive SIGINT and shut down
# cleanly (rather than inheriting the usual ignored SIGINT disposition).
set -m

SCRIPT_PATH="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/$(basename -- "${BASH_SOURCE[0]}")"
# The experiment paths below are relative to the directory you launch this script from.

NUM_EXPERIMENTS="${1:-5}"
if [[ ! "$NUM_EXPERIMENTS" =~ ^[1-9][0-9]*$ ]]; then
  echo "Usage: $0 [number_of_experiments]" >&2
  echo "Example: $0 5" >&2
  exit 2
fi

# If not already inside the humble Pixi environment, rerun this script there.
if [[ "${PIXI_ENVIRONMENT_NAME:-}" != "humble" ]]; then
  if ! command -v pixi >/dev/null 2>&1; then
    echo "Error: Pixi is not available on PATH, and the 'humble' environment is not active." >&2
    exit 1
  fi
  exec pixi run -e humble bash "$SCRIPT_PATH" "$@"
fi

# Seconds to let ROS 2 discovery/subscriptions settle before starting each trial.
LOGGER_STARTUP_DELAY="${LOGGER_STARTUP_DELAY:-2}"

logger_pid=""
stop_logger() {
  if [[ -n "$logger_pid" ]]; then
    echo "Stopping force logger (PID $logger_pid)..."
    kill -INT "$logger_pid" 2>/dev/null || true
    wait "$logger_pid" 2>/dev/null || true
    logger_pid=""
  fi
}

trap stop_logger EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

for ((experiment = 1; experiment <= NUM_EXPERIMENTS; experiment++)); do
  echo
  echo "========== Experiment $experiment / $NUM_EXPERIMENTS =========="

  python ./experiments/force_logger.py "$experiment" &
  logger_pid=$!
  echo "Started force logger for experiment $experiment (PID $logger_pid)."

  sleep "$LOGGER_STARTUP_DELAY"

  echo "Running pi_admittance_experiment.py..."
  if python ./python_nodes/pi_admittance_experiment.py; then
    experiment_status=0
  else
    experiment_status=$?
  fi

  stop_logger

  if (( experiment_status != 0 )); then
    echo "Experiment $experiment failed with exit code $experiment_status; stopping batch." >&2
    exit "$experiment_status"
  fi

  echo "Experiment $experiment completed."
done

echo "All $NUM_EXPERIMENTS experiments completed."
