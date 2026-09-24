# Robot AP 部署方案

> **强制操作约束：** Codex 只能检查网络状态、准备配置和提供命令，不得自行执行
> `nmcli connection up/down`、切换活动 Wi-Fi、启用/关闭 Robot AP 或修改网络模式。
> 所有会造成网络切换的终端命令只能由用户本人明确执行，避免 VS Code Remote SSH
> 和 Codex 会话中断。

## 已确认的网络方案

首版实机采用 **Robot AP**：RK3588 自建 Wi-Fi 热点，手机直接连接机器人，
不依赖家庭路由器或互联网。

```text
SSID              Creeper-Robot
RK3588 地址        192.168.50.1/24
WebSocket（开发） ws://192.168.50.1:8765
频段              2.4 GHz
NetworkManager     Creeper-Robot-AP
```

热点密码不得提交到 Git。

## 当前设备检查结果（2026-09-14）

- 无线接口为 `wlan0`；
- 驱动支持 AP 模式；
- NetworkManager 正在运行；
- 已安装 `hostapd` 和 `dnsmasq`；
- 当前 SSH 通过 `wlan0` 的 `Robotstar` 网络连接；
- 有线接口当前不可用。

因此设备具备建立 Robot AP 的条件，但不能在没有备用登录方式时直接远程切换，
否则当前 SSH 会立即断开。

## 第一次配置（只创建，不启动）

先确保机器人不在运动、保存所有文件，并准备以下任一恢复方式：

- HDMI 显示器和键盘；
- 串口终端；
- 已验证可用的有线网络。

然后执行：

```bash
sudo nmcli connection add \
  type wifi ifname wlan0 con-name Creeper-Robot-AP \
  autoconnect no ssid Creeper-Robot

sudo nmcli connection modify Creeper-Robot-AP \
  802-11-wireless.mode ap \
  802-11-wireless.band bg \
  802-11-wireless.channel 6 \
  ipv4.method shared \
  ipv4.addresses 192.168.50.1/24 \
  ipv6.method disabled \
  wifi-sec.key-mgmt wpa-psk

read -rsp "Robot AP password: " ROBOT_AP_PSK; echo
sudo nmcli connection modify Creeper-Robot-AP wifi-sec.psk "$ROBOT_AP_PSK"
unset ROBOT_AP_PSK
```

密码长度应为 8～63 个字符。上述方式不会把密码明文写入脚本、终端历史或 Git；
配置完成后也立即清除了当前 shell 变量。

检查配置但不要启动：

```bash
nmcli connection show Creeper-Robot-AP
```

## 首次启动

首次启动热点会断开当前 `Robotstar` Wi-Fi 和其上的 SSH：

```bash
sudo nmcli connection up Creeper-Robot-AP
```

随后让电脑或手机连接 `Creeper-Robot`，再使用：

```bash
ssh orangepi@192.168.50.1
```

热点验证完成前保留 `autoconnect no`。如果配置失败，可重启设备，使原来的
`Robotstar` 自动连接恢复。

## WebSocket 冒烟验证

在 RK3588 上：

```bash
cd /home/orangepi/unitree_actuator_sdk
python3 wechat_mini_program/rk3588_gateway/smoke_test/ws_smoke_server.py
```

连接到 `Creeper-Robot` 的另一台设备使用：

```text
ws://192.168.50.1:8765
```

该冒烟服务器不会访问电机、串口、GPIO 或机器人控制 FIFO。

## 必须验证的异常场景

- 手机能够保持连接“无互联网”的 Wi-Fi；
- 手机休眠、小程序进入后台后连接会按预期断开；
- 重新进入小程序能够重连；
- Robot AP 断开后 RK3588 的命令 watchdog 能独立归零；
- 网络线程异常不会阻塞机器人实时控制线程；
- 正式实机控制前完成软件急停和物理急停测试。

Robot AP 只解决近距离无线链路，不替代正式 Protocol V1、控制权、命令超时、
heartbeat 超时、鉴权或安全停机机制。
