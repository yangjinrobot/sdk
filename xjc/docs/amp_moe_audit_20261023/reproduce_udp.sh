#!/usr/bin/env bash
set -euo pipefail

# Loopback sockets and synthetic feedback only. No hardware interface is loaded.
audit_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "$audit_dir/../.." && pwd)"
audit_build_dir="$(mktemp -d /tmp/mm_moe_udp_audit.XXXXXX)"
trap 'rm -rf -- "$audit_build_dir"' EXIT
cd -- "$repo_dir"

g++ -std=c++17 -O2 \
  -I run_policy/cr1pro/include -I types -I utils -I state_machine \
  -I interface/user_command -I third_party/toml -I third_party/eigen \
  -I third_party/onnxruntime/x86/include \
  "$audit_dir/udp_repro.cpp" run_policy/cr1pro/src/udp_node.cpp \
  -L third_party/onnxruntime/x86/lib \
  -Wl,-rpath,"$repo_dir/third_party/onnxruntime/x86/lib" \
  -lonnxruntime -pthread -o "$audit_build_dir/udp_repro"
MOTION_MASTER_POLICY_DIR="$repo_dir/run_policy/cr1pro/policy" "$audit_build_dir/udp_repro"
