#!/usr/bin/env bash

set -uo pipefail

usage() {
  echo "usage: $0 --read-only | --mutating | --glove-read-only" >&2
}

is_valid_hardware_selector() {
  local LC_ALL=C
  [[ -n "$1" && ! "$1" =~ [[:space:][:cntrl:]] ]]
}

is_valid_runner_command() {
  local LC_ALL=C
  [[ -n "$1" ]] &&
    [[ ! "$1" =~ [[:cntrl:]] ]] &&
    [[ ! "$1" =~ ^[[:space:]] ]] &&
    [[ ! "$1" =~ [[:space:]]$ ]]
}

if [[ "$#" -ne 1 ]]; then
  usage
  exit 2
fi

case "$1" in
  --read-only)
    executable_name="wuji_hand2_hardware_read_only"
    package_name="wuji_hand2_driver"
    runner_variable="WUJI_HARDWARE_RUNNER"
    ;;
  --mutating)
    executable_name="wuji_hand2_hardware_mutating"
    package_name="wuji_hand2_driver"
    runner_variable="WUJI_HARDWARE_RUNNER"
    ;;
  --glove-read-only)
    executable_name="wuji_glove_hardware_read_only"
    package_name="wuji_glove_driver"
    runner_variable="WUJI_GLOVE_HARDWARE_RUNNER"
    ;;
  *)
    usage
    exit 2
    ;;
esac

if [[ "$1" == "--glove-read-only" ]]; then
  if [[ "${WUJI_GLOVE_HARDWARE_TESTS:-}" != "1" ]]; then
    echo "Glove hardware tests are disabled (set WUJI_GLOVE_HARDWARE_TESTS=1 to opt in)" >&2
    exit 77
  fi
  glove_serial="${WUJI_GLOVE_HARDWARE_SERIAL:-}"
  glove_endpoint="${WUJI_GLOVE_HARDWARE_ENDPOINT:-}"
  glove_hand_side="${WUJI_GLOVE_HARDWARE_HAND_SIDE:-}"
  if [[ -n "$glove_serial" ]] && ! is_valid_hardware_selector "$glove_serial"; then
    echo "WUJI_GLOVE_HARDWARE_SERIAL must not contain whitespace or ASCII controls" >&2
    exit 2
  fi
  if [[ -n "$glove_endpoint" ]] && ! is_valid_hardware_selector "$glove_endpoint"; then
    echo "WUJI_GLOVE_HARDWARE_ENDPOINT must not contain whitespace or ASCII controls" >&2
    exit 2
  fi
  if [[ -n "$glove_hand_side" && "$glove_hand_side" != "left" && "$glove_hand_side" != "right" ]]; then
    echo "WUJI_GLOVE_HARDWARE_HAND_SIDE must be left, right, or empty" >&2
    exit 2
  fi
else
if [[ "${WUJI_HARDWARE_TESTS:-}" != "1" ]]; then
  echo "Hand 2 hardware tests are disabled (set WUJI_HARDWARE_TESTS=1 to opt in)" >&2
  exit 77
fi

hardware_serial="${WUJI_HARDWARE_SERIAL:-}"
hardware_endpoint="${WUJI_HARDWARE_ENDPOINT:-}"
if [[ -n "$hardware_serial" ]] && ! is_valid_hardware_selector "$hardware_serial"; then
  echo "WUJI_HARDWARE_SERIAL must not contain whitespace or ASCII controls" >&2
  exit 2
fi
if [[ -n "$hardware_endpoint" ]] && ! is_valid_hardware_selector "$hardware_endpoint"; then
  echo "WUJI_HARDWARE_ENDPOINT must not contain whitespace or ASCII controls" >&2
  exit 2
fi
if [[ -z "$hardware_serial" && -z "$hardware_endpoint" ]]; then
  echo "set WUJI_HARDWARE_SERIAL or WUJI_HARDWARE_ENDPOINT to select one device" >&2
  exit 2
fi

if [[ "$1" == "--mutating" ]]; then
  if [[ -z "$hardware_serial" || -n "$hardware_endpoint" ]]; then
    echo "mutating tests require only WUJI_HARDWARE_SERIAL" >&2
    exit 2
  fi
  if [[ "${WUJI_MUTATING_HARDWARE_TESTS:-}" != "1" ]]; then
    echo "mutating tests require WUJI_MUTATING_HARDWARE_TESTS=1" >&2
    exit 3
  fi
  if [[ "${WUJI_HARDWARE_SAFETY_ACK:-}" != "fixture-unloaded-operator-state-not-restored" ]]; then
    echo "mutating tests require WUJI_HARDWARE_SAFETY_ACK=fixture-unloaded-operator-state-not-restored" >&2
    exit 3
  fi
  unset WUJI_HARDWARE_ENDPOINT
fi
fi

if [[ "${!runner_variable+x}" == "x" ]]; then
  runner_command="${!runner_variable}"
  if ! is_valid_runner_command "$runner_command"; then
    echo "$runner_variable must be trimmed and contain no ASCII controls" >&2
    exit 2
  fi

  # WUJI_HARDWARE_RUNNER is an explicit test/operator command-string hook. A
  # shell is required by that contract; the value is never evaluated until all
  # safety gates above pass, and it receives no interpolated script data.
  exec /usr/bin/env bash -c -- "$runner_command"
fi

script_directory="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
public_root="$(cd -- "$script_directory/.." && pwd -P)"
candidate_paths=(
  "$PWD/build/$package_name/$executable_name"
  "$public_root/build/$package_name/$executable_name"
)
for candidate in "${candidate_paths[@]}"; do
  if [[ -f "$candidate" && -x "$candidate" ]]; then
    exec "$candidate"
  fi
done

echo "hardware test executable not found: $executable_name" >&2
echo "build $package_name with BUILD_TESTING=ON or set $runner_variable" >&2
exit 2
