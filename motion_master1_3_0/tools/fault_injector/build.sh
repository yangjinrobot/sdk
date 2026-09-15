#!/bin/bash
# ============================================================================
# build.sh — 交叉编译 fault_inject 故障注入调试工具（x86 → ARM）
#
# 依赖：aarch64-linux-gnu-g++
#
# 用法：
#   ./build.sh           # 编译
#   ./build.sh clean     # 清理
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
THIRD_PARTY="$PROJECT_ROOT/third_party"
DR_LIB="$THIRD_PARTY/dr_lib/arm"

OUTPUT="$SCRIPT_DIR/fault_inject"
CXX="${CXX:-aarch64-linux-gnu-g++}"

# ---- clean ----
if [ "${1:-}" = "clean" ]; then
    rm -f "$OUTPUT"
    echo "[clean] removed $OUTPUT"
    exit 0
fi

# ---- check ----
if ! command -v "$CXX" &>/dev/null; then
    echo "[ERROR] cross-compiler $CXX not found. Install: apt install g++-aarch64-linux-gnu"
    exit 1
fi

if [ ! -f "$DR_LIB/drdds/libdrdds.so" ]; then
    echo "[ERROR] DDS lib not found: $DR_LIB/drdds/libdrdds.so"
    exit 1
fi

# ---- compile ----
echo "[build] cross-compiling for ARM with $CXX ..."

$CXX -std=c++17 -O2 -Wall \
    -I"$DR_LIB/drdds/include/dridl" \
    -I"$DR_LIB/drdds/include" \
    -I"$DR_LIB/drdds/" \
    -L"$DR_LIB/drdds/" \
    -L"$DR_LIB/drdds/lib/" \
    -L"$DR_LIB/log4cplus/lib/" \
    -Wl,-rpath-link,"$DR_LIB/drdds/lib" \
    -Wl,-rpath-link,"$DR_LIB/log4cplus/lib" \
    "$SCRIPT_DIR/fault_inject.cpp" \
    -o "$OUTPUT" \
    -ldrdds -lfastcdr -lfastrtps -lfoonathan_memory-0.7.3 \
    -lpthread -ldl

echo "[build] SUCCESS → $OUTPUT"
echo ""
echo "Usage examples:"
echo "  $OUTPUT pos_over_limit 0x7E0000    # leg joints fault → JointDamping"
echo "  $OUTPUT pos_over_limit 0x1FC00     # right arm fault → in-place protection"
echo "  $OUTPUT nan 0x4                    # waist_z NaN"
echo "  $OUTPUT clear all                  # clear all injected faults"
