#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "$1" ]; then
    echo "Usage: $0 <ethernet_if> [--gui]"
    echo "Example: $0 eth0"
    echo "  --gui  optional, enable OpenCV debug windows"
    exit 1
fi

cd "${SCRIPT_DIR}/build"
cmake ..
make -j$(nproc)

echo "==> Running rc2025_run: $*"
"${SCRIPT_DIR}/build/rc2025_run" "$@"
