#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
MODE="${1:---check}"
if [[ "$MODE" == --help ]]; then
  echo 'Usage: ./flash_esp32.sh --install-core | --check | --upload /dev/serial/by-id/ESP32'; exit 0
fi
if [[ "$MODE" != --check && "$MODE" != --install-core && "$MODE" != --upload ]]; then echo 'Unknown mode; use --help' >&2; exit 2; fi
if [[ "$MODE" == --upload && -z "${2:-}" ]]; then echo '--upload requires the ESP32 serial port' >&2; exit 2; fi
CLI=("${ARDUINO_CLI:-arduino-cli}")
if [[ -n "${ARDUINO_CONFIG_FILE:-}" ]]; then CLI+=(--config-file "$ARDUINO_CONFIG_FILE"); fi
command -v "${CLI[0]}" >/dev/null || { echo 'Install Arduino CLI: https://arduino.github.io/arduino-cli/'; exit 1; }
INDEX=https://espressif.github.io/arduino-esp32/package_esp32_index.json
if [[ "$MODE" == --install-core ]]; then
  "${CLI[@]}" core update-index --additional-urls "$INDEX"
  "${CLI[@]}" core install esp32:esp32@2.0.17 --additional-urls "$INDEX"
  exit 0
fi
if ! "${CLI[@]}" core list --format json | python3 -c 'import json,sys; p=json.load(sys.stdin).get("platforms",[]); sys.exit(0 if any(x.get("id")=="esp32:esp32" and x.get("installed_version")=="2.0.17" for x in p) else 1)'; then
  echo 'Install the tested ESP32 core first: ./flash_esp32.sh --install-core' >&2; exit 1
fi
BUILD_DIR="${ROVER_BUILD_DIR:-$HERE/firmware/build}"
mkdir -p "$BUILD_DIR"
"${CLI[@]}" compile --fqbn esp32:esp32:esp32 --output-dir "$BUILD_DIR" "$HERE/firmware/rover_esp32"
if [[ "$MODE" == --upload ]]; then
  "${CLI[@]}" upload --port "$2" --fqbn esp32:esp32:esp32 --input-dir "$BUILD_DIR" "$HERE/firmware/rover_esp32"
else
  echo 'Compilation passed. No firmware uploaded.'
fi
