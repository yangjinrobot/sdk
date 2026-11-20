#!/usr/bin/env bash
# ============================================================
# 一键流程：清理 build -> cmake -> make -> make package
#           -> scp deb -> ssh 远端 sudo dpkg -i
# 用法:
#   ./deploy.sh                      # 直接运行，密码在下方配置区
#   REMOTE_PASS='xxx' ./deploy.sh    # 也可用环境变量覆盖
# ============================================================
set -Eeuo pipefail

# ==================== 配置区 ====================
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${PROJECT_DIR}/build"
BUILD_PLATFORM="arm"

REMOTE_USER="user"
REMOTE_HOST="10.21.33.103"
REMOTE_DIR="qxj_deb"

# 本机 sudo 密码（用于清理 build/）。留空则要求本机免密 sudo
LOCAL_SUDO_PASS="${LOCAL_SUDO_PASS:-1}"

# 远端 SSH 登录密码。留空则尝试 SSH 公钥免密
DEFAULT_REMOTE_PASS="'"
REMOTE_PASS="${REMOTE_PASS:-${DEFAULT_REMOTE_PASS}}"

# 远端 sudo 密码，留空则沿用 REMOTE_PASS
REMOTE_SUDO_PASS="${REMOTE_SUDO_PASS:-${REMOTE_PASS}}"

JOBS="$(nproc)"
SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o ConnectTimeout=10)
# ================================================

log() { printf '\n\033[1;32m==> %s\033[0m\n' "$*"; }
die() { printf '\033[1;31m[ERROR] %s\033[0m\n' "$*" >&2; exit 1; }
trap 'printf "\033[1;31m[ERROR] 第 %s 行执行失败，脚本中止\033[0m\n" "$LINENO" >&2' ERR

# ---------- 1. 清理 build ----------
local_sudo() {
    if [[ -n "${LOCAL_SUDO_PASS}" ]]; then
        printf '%s\n' "${LOCAL_SUDO_PASS}" | sudo -S -p '' -- "$@"
    else
        sudo -n -- "$@"
    fi
}

log "校验本机 sudo 权限"
local_sudo true || die "本机 sudo 失败，请检查配置区的 LOCAL_SUDO_PASS"

mkdir -p "$BUILD_DIR"
log "清理 build 目录: $BUILD_DIR"
# 等价于 cd build && sudo rm -rf *，同时覆盖隐藏文件
local_sudo find "$BUILD_DIR" -mindepth 1 -maxdepth 1 -exec rm -rf -- {} +
local_sudo chown -R "$(id -u):$(id -g)" "$BUILD_DIR"

# ---------- 2. cmake ----------
log "cmake 配置 (BUILD_PLATFORM=${BUILD_PLATFORM})"
cmake -S "$PROJECT_DIR" -B "$BUILD_DIR" -DBUILD_PLATFORM="${BUILD_PLATFORM}"

# ---------- 3. make ----------
log "make -j${JOBS}"
make -C "$BUILD_DIR" -j"${JOBS}"

# ---------- 4. make package ----------
log "make package"
make -C "$BUILD_DIR" package

# ---------- 5. 取最新生成的 deb ----------
# 注意：工程路径含空格/括号，不能用 awk 按空白切分，这里用数组 + -nt 比较
DEB_PATH=""
for f in "$BUILD_DIR"/*.deb; do
    [[ -e "$f" ]] || continue
    if [[ -z "$DEB_PATH" || "$f" -nt "$DEB_PATH" ]]; then
        DEB_PATH="$f"
    fi
done
[[ -n "${DEB_PATH}" && -f "${DEB_PATH}" ]] || die "未找到 .deb 产物，请检查 make package 输出"
DEB_NAME="$(basename "$DEB_PATH")"
log "打包产物: ${DEB_NAME}"

# ---------- 6. 远端认证方式 ----------
if [[ -z "${REMOTE_PASS}" ]]; then
    log "检测 ${REMOTE_USER}@${REMOTE_HOST} 是否可免密登录"
    if ssh "${SSH_OPTS[@]}" -o BatchMode=yes "${REMOTE_USER}@${REMOTE_HOST}" true >/dev/null 2>&1; then
        USE_PASS=0
    else
        command -v sshpass >/dev/null 2>&1 || die "无法免密登录且未安装 sshpass，请执行: sudo apt install sshpass，或配置 SSH 公钥"
        read -rsp "请输入 ${REMOTE_USER}@${REMOTE_HOST} 的密码: " REMOTE_PASS
        echo
        [[ -n "${REMOTE_PASS}" ]] || die "密码不能为空"
        USE_PASS=1
    fi
else
    command -v sshpass >/dev/null 2>&1 || die "已设置 REMOTE_PASS 但未安装 sshpass，请执行: sudo apt install sshpass"
    USE_PASS=1
fi

remote_ssh() {
    if (( USE_PASS )); then
        sshpass -p "${REMOTE_PASS}" ssh "${SSH_OPTS[@]}" "${REMOTE_USER}@${REMOTE_HOST}" "$@"
    else
        ssh "${SSH_OPTS[@]}" "${REMOTE_USER}@${REMOTE_HOST}" "$@"
    fi
}

remote_scp() {
    if (( USE_PASS )); then
        sshpass -p "${REMOTE_PASS}" scp "${SSH_OPTS[@]}" "$@"
    else
        scp "${SSH_OPTS[@]}" "$@"
    fi
}

# ---------- 7. scp 上传 ----------
log "创建远端目录 ~/${REMOTE_DIR}"
remote_ssh "mkdir -p ~/${REMOTE_DIR}"

log "上传 ${DEB_NAME} -> ${REMOTE_USER}@${REMOTE_HOST}:~/${REMOTE_DIR}/"
remote_scp "${DEB_PATH}" "${REMOTE_USER}@${REMOTE_HOST}:${REMOTE_DIR}/"

# ---------- 8. 远端 dpkg 安装 ----------
log "远端安装 ${DEB_NAME}"
# 远端 sudo 密码与 SSH 密码分开处理，密码特殊字符（如单引号）用 %q 安全转义
if [[ -n "${REMOTE_SUDO_PASS}" ]]; then
    SUDO_Q="$(printf '%q' "${REMOTE_SUDO_PASS}")"
    remote_ssh -t "cd ~/${REMOTE_DIR} && printf '%s\n' ${SUDO_Q} | sudo -S -p '' dpkg -i '${DEB_NAME}'"
else
    remote_ssh -t "cd ~/${REMOTE_DIR} && sudo -n dpkg -i '${DEB_NAME}'"
fi

log "检查远端安装状态"
remote_ssh "dpkg -s motion_master 2>/dev/null | head -n 5 || true"

log "全部完成"
