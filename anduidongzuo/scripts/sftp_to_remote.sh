#!/usr/bin/expect

set ip "10.21.41.1"
set username "user"
set passwd "'"
set port 44529
set timeout 60
set remote_bin "/opt/robot/share/motion_master/bin"
set remote_scripts "/opt/robot/share/motion_master/scripts"
set remote_conf "/var/opt/robot/conf/motion_master"

# 防止误把其他源码目录生成的旧 build/motion_master 推送到机器人。
set build_cache "build/CMakeCache.txt"
if {![file exists $build_cache] || ![file exists "build/motion_master"]} {
  puts stderr "缺少当前工程的 build 产物，请先在本目录重新配置并编译"
  exit 1
}
set cache_file [open $build_cache r]
set cache_text [read $cache_file]
close $cache_file
set expected_source "CMAKE_HOME_DIRECTORY:INTERNAL=[file normalize [pwd]]"
if {[string first $expected_source $cache_text] < 0} {
  puts stderr "build/CMakeCache.txt 指向其他源码目录，拒绝推送旧程序"
  exit 1
}

# 1. 开启SSH
spawn ./scripts/drsec_tool open_ssh 10.21.33.103
expect eof

# 2. scp motion_master
spawn scp -P $port build/motion_master $username@$ip:$remote_bin
expect {
  "密码：" { send "$passwd\r" }
  "pass"   { send "$passwd\r" }
  "yes/no" { send "yes\r"; exp_continue }
}
expect eof

# 3. 只部署保留的 RL 模型和备用 old 模型
spawn scp -P $port run_policy/cr1/policy/new.onnx $username@$ip:/opt/robot/share/motion_master/policy/new.onnx
expect {
  "密码：" { send "$passwd\r" }
  "pass"   { send "$passwd\r" }
  "yes/no" { send "yes\r"; exp_continue }
}
expect eof

spawn scp -P $port run_policy/cr1/policy/old.onnx $username@$ip:/opt/robot/share/motion_master/policy/old.onnx
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
