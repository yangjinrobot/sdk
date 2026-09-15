#!/usr/bin/expect

set ip "10.21.41.1"
set username "user"
set passwd "'"
set port 44529
set timeout 60
set remote_bin "/opt/robot/share/motion_master/bin"
set remote_scripts "/opt/robot/share/motion_master/scripts"
set remote_conf "/var/opt/robot/conf/motion_master"

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

# 3. scp ONNX 模型到机器人
spawn ssh -p $port $username@$ip "mkdir -p /opt/robot/share/motion_master/policy"
expect {
  "密码：" { send "$passwd\r" }
  "pass"   { send "$passwd\r" }
  "yes/no" { send "yes\r"; exp_continue }
}
expect eof

spawn scp -P $port run_policy/cr1/policy/policy_CR1_8.onnx $username@$ip:/opt/robot/share/motion_master/policy/policy_CR1_8.onnx
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
