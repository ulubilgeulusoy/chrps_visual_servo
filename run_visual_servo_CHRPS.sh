#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "$0")" && pwd)"
BIN="${APP_DIR}/build/servoFrankaIBVS_CHRPS"
EMC="${APP_DIR}/config/eMc.yaml"   # update this if your YAML lives elsewhere

# Make sure ViSP libs are found
export LD_LIBRARY_PATH="$HOME/visp_install/lib:${LD_LIBRARY_PATH:-}"

if [[ ! -x "$BIN" ]]; then
  echo "Binary not found. Build first:"
  echo "  cd \"$APP_DIR\" && mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DVISP_DIR=$HOME/visp_install/lib/cmake/visp && make -j\$(nproc)"
  exit 1
fi

"$BIN" \
  --eMc "$EMC" \
  --ip 172.16.0.2 \
  --no-convergence-threshold \
  --adaptive-gain \
  --plot \
  --tag-size 0.05 \
  --desired-factor 9
