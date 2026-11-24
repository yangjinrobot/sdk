#!/usr/bin/expect

set ip "10.21.41.1"
set username "user"
set passwd "'"
set port 44529
set timeout 60
set remote_bin "/opt/robot/share/motion_master/bin"
set remote_policy "/opt/robot/share/motion_master/policy"
set remote_scripts "/opt/robot/share/motion_master/scripts"
set remote_conf "/var/opt/robot/conf/motion_master"
set policy_root "run_policy/cr1/policy"

# 默认支持从项目根目录手动运行，CMake 会传入实际构建产物的路径。
set binary_path "build/motion_master"
if {$argc > 0} {
  set binary_path [lindex $argv 0]
}
set binary_path [file normalize $binary_path]
set build_cache [file join [file dirname $binary_path] CMakeCache.txt]
if {![file isfile $build_cache] || ![file isfile $binary_path]} {
  puts stderr "缺少编译产物或 CMakeCache.txt：$binary_path"
  exit 1
}
set cache_file [open $build_cache r]
set cache_text [read $cache_file]
close $cache_file
set expected_source "CMAKE_HOME_DIRECTORY:INTERNAL=[file normalize [pwd]]"
if {[lsearch -exact [split $cache_text "\n"] $expected_source] < 0} {
  puts stderr "CMakeCache.txt 指向其他源码目录，拒绝推送旧程序"
  exit 1
}

# 1. 开启SSH
spawn ./scripts/drsec_tool open_ssh 10.21.33.103
expect eof

# 2. 先更新策略，保留 arm 包使用的目录结构。
spawn scp -r -P $port $policy_root/amp $policy_root/rl $policy_root/mimic $username@$ip:$remote_policy/
expect {
  "密码：" { send "$passwd\r"; exp_continue }
  "pass"   { send "$passwd\r"; exp_continue }
  "yes/no" { send "yes\r"; exp_continue }
  timeout  { puts stderr "策略推送超时：$remote_policy"; exit 1 }
  eof      {}
}
set copy_status [wait]
if {[lindex $copy_status 2] != 0 || [lindex $copy_status 3] != 0} {
  puts stderr "策略推送失败：$remote_policy"
  exit 1
}

# 3. scp motion_master
spawn scp -P $port $binary_path $username@$ip:$remote_bin
expect {
  "密码：" { send "$passwd\r" }
  "pass"   { send "$passwd\r" }
  "yes/no" { send "yes\r"; exp_continue }
}
expect eof

# 4. scp run.sh
spawn scp -P $port scripts/run.sh $username@$ip:$remote_scripts/run.sh
expect {
  "密码：" { send "$passwd\r" }
  "pass"   { send "$passwd\r" }
  "yes/no" { send "yes\r"; exp_continue }
}
expect eof

# 5. SSH执行setcap
spawn ssh -p $port $username@$ip
expect {
  "密码：" { send "$passwd\r" }
  "pass"   { send "$passwd\r" }
  "yes/no" { send "yes\r"; exp_continue }
}
expect -re {[$#] }

send "sudo setcap cap_sys_nice,cap_net_raw,cap_net_admin+ep $remote_bin/motion_master\r"
expect {
  "password" { send "$passwd\r"; exp_continue }
  -re {[$#] } {}
}

send "exit\r"
expect eof

# 6. scp config.toml 到 /tmp/（目标目录仅 root 可写）
spawn scp -P $port conf/config.toml $username@$ip:/tmp/config.toml
expect {
  "密码：" { send "$passwd\r" }
  "pass"   { send "$passwd\r" }
  "yes/no" { send "yes\r"; exp_continue }
}
expect eof

# 7. SSH 里 sudo 落位到 $remote_conf/config.toml
spawn ssh -p $port $username@$ip
expect {
  "密码：" { send "$passwd\r" }
  "pass"   { send "$passwd\r" }
  "yes/no" { send "yes\r"; exp_continue }
}
expect -re {[$#] }

send "sudo cp /tmp/config.toml $remote_conf/config.toml && sudo rm -f /tmp/config.toml\r"
expect {
  "password" { send "$passwd\r"; exp_continue }
  -re {[$#] } {}
}

send "exit\r"
expect eof
