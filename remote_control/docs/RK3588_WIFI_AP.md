# RK3588 调试热点

## 目标

使用 RK3588 内置 `wlan0` 发出 WPA2 热点：

```text
SSID: Creeper_Dog
RK3588: 192.168.50.1/24
SSH: orangepi@192.168.50.1
```

热点由 NetworkManager 管理，不属于 `robot_control`，也不参与电机控制。由于当前只有
一块 Wi-Fi 网卡，进入热点模式后 RK3588 不再连接 `Robotstar`，通常也无法访问互联网，
因此运行在 RK3588 上的 Codex 会离线。

## 一次性创建配置

下面的命令不会开启热点，也不会中断当前 SSH。密码会隐藏输入，不会写进命令历史：

```bash
cd /home/orangepi/unitree_actuator_sdk
sudo ./remote_control/network/setup_creeper_ap.sh
```

脚本同时停用旧的 `hostapd` 和独立 `dnsmasq` 服务，避免它们与 NetworkManager 冲突。
已保存的 `Robotstar`、`SQ` 和 `ZZPdesktop` Wi-Fi 配置不会被删除。

## 开启热点

只允许用户在 RK3588 的本机终端执行。不要通过 SSH 执行，因为切换后当前 SSH 和
Codex 会立刻断开：

```bash
sudo /home/orangepi/unitree_actuator_sdk/remote_control/network/start_creeper_ap.sh
```

电脑连接 `Creeper_Dog` 后验证：

```powershell
ping 192.168.50.1
Test-NetConnection 192.168.50.1 -Port 22
ssh orangepi@192.168.50.1
```

## 恢复联网模式

同样只在 RK3588 本机终端执行：

```bash
sudo /home/orangepi/unitree_actuator_sdk/remote_control/network/stop_creeper_ap.sh
```

脚本默认恢复 `Robotstar`。如需连接其他已保存网络，可在本机执行：

```bash
sudo nmcli connection up SQ
# 或
sudo nmcli connection up ZZPdesktop
```

## 查看状态（不会切换网络）

```bash
nmcli device status
nmcli connection show --active
ip -brief address show wlan0
```

电脑使用双网卡时，建议内置网卡负责互联网，USB 网卡连接 `Creeper_Dog`。如果 Windows
错误地把热点链路当作默认互联网出口，可提高 USB 网卡的接口跃点数；RK3588 仍然不会
因此自动获得互联网，若需热点和 Codex 联网同时工作，应给 RK3588 增加第二块网卡。
