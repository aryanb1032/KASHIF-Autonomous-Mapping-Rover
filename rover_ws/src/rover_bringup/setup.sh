#!/usr/bin/env bash
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
exec bash "$HERE/../../../setup_pi.sh" "$@"
