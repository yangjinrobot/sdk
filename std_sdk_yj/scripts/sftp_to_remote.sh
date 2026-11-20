#!/bin/sh
set -eu

binary=${1:?Usage: sftp_to_remote.sh PATH_TO_MOTION_MASTER}
target=${MOTION_MASTER_SSH_TARGET:?Set MOTION_MASTER_SSH_TARGET to an SSH host or alias}
remote_path=${MOTION_MASTER_SSH_PATH:?Set MOTION_MASTER_SSH_PATH to the destination file}

scp -- "$binary" "$target:$remote_path"
