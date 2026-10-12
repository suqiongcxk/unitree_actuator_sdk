# PC键盘多键遥控说明

## 为什么修改

旧的`keyboard_velocity_console.py`运行在香橙派SSH终端中。终端只传输字符和
操作系统产生的按键重复，不传输可靠的key-down/key-up状态。长按`W+A`时，
终端常常只继续重复最后按下的按键，另一个轴会被650 ms deadman清零。因此，
仅延长超时或给全部轴续期都会产生错误或“松开W仍继续前进”的风险。

同时，旧脚本用同一组`+/-`同时改变`vx`和`vy`，不能独立调整前进与横移
速度。V45仿真已经验证`vx=+-0.2 m/s`，横移需求范围为`0.2~0.3 m/s`，旧的
最低速度也不再合适。

新方案在连接物理键盘的Windows PC上直接读取每个键的按下状态，通过一条
持续SSH连接把完整的`vx/vy/yaw`发送给香橙派。香橙派桥接器只验证文本命令并
写入现有FIFO，不开放新端口，不访问电机，也不绕过`robot_control`的速度斜坡、
watchdog、关节限位或安全停机。

## 文件及职责

- `remote_control/keyboard/pc_keyboard_velocity_client.py`：复制到Windows PC运行，读取真实多键状态，
  维护独立速度档位，并启动持续SSH连接。
- `remote_control/keyboard/ssh_velocity_command_bridge.py`：在香橙派由SSH远程启动，验证命令并写入
  `/tmp/creeper_velocity_command.fifo`。FIFO不存在时拒绝启动；SSH EOF时请求
  平滑回零。
- `remote_control/keyboard/keyboard_velocity_console.py`：保留的SSH终端单键备用控制器。已修正为三个
  独立速度档位，但由于终端协议限制，不作为多键长按方案。
- `velocity_command_manager.{h,cpp}`：主程序内的限幅、V45加减速斜坡和500 ms
  watchdog。
- `remote_control/tests/test_keyboard_velocity_control.py`：组合键、同轴冲突、独立调速和桥接
  命令验证的纯软件测试。

## 速度和按键

| 轴 | 范围 | 默认值 | 步长 |
|---|---:|---:|---:|
| `vx` | `0.2~1.0 m/s` | `0.2 m/s` | `0.1` |
| `vy` | `0.2~0.3 m/s` | `0.2 m/s` | `0.1` |
| `yaw` | `0.5~1.0 rad/s` | `0.5 rad/s` | `0.1` |

| 按键 | 功能 |
|---|---|
| `W/S` | 前进/后退 |
| `A/D` | 左移/右移 |
| `Q/E` | 左转/右转 |
| `+/-` | 增加/减少`vx`档位 |
| `>/<`（句号/逗号键） | 增加/减少`vy`档位 |
| `]/[` | 增加/减少`yaw`档位 |
| 空格 | 平滑回零，按住时屏蔽运动键 |
| `X` | 请求统一安全停机并退出 |
| `Esc` | 平滑回零并退出客户端 |
| `Ctrl+C` | 请求统一安全停机并退出 |

`W+A`、`W+Q`、`W+A+Q`等组合会产生完整的多轴命令。同轴相反按键同时按下
时该轴为零，例如`W+S -> vx=0`。

## 首次准备：把PC客户端复制到Windows

在Windows PowerShell执行，目标路径可以自行更改：

```powershell
scp orangepi@192.168.31.100:/home/orangepi/unitree_actuator_sdk/remote_control/keyboard/pc_keyboard_velocity_client.py "$env:USERPROFILE\Desktop\pc_keyboard_velocity_client.py"
```

PC需要有Python 3和Windows自带的OpenSSH客户端。脚本只使用Python标准库，
不需要安装`keyboard`或`pynput`。如果使用机器人热点，把IP改为实际的
`192.168.50.1`；切换网络/AP只能由操作者手动完成。

## 使用步骤

### 终端1：在香橙派启动主控制程序

```bash
cd /home/orangepi/unitree_actuator_sdk

sudo ./build/robot_control \
  --log \
  --log-file /tmp/v45_keyboard_walk.csv \
  2>&1 | tee -i /tmp/v45_keyboard_walk.log
```

确认模型是`creeper_v45_model999_actor.onnx`，等待机械标定、开环站立和NN平滑
接管完成。先观察零命令站立；主程序创建FIFO以后才能连接键盘客户端。

### Windows PowerShell：启动PC客户端

```powershell
py "$env:USERPROFILE\Desktop\pc_keyboard_velocity_client.py" --host orangepi@192.168.31.100
```

SSH认证期间程序不会读取运动按键。远端桥接器确认FIFO存在并返回
`BRIDGE_READY`后，终端显示`Key capture is now active`，此时才开始控制。

也可以显式指定初始速度：

```powershell
py "$env:USERPROFILE\Desktop\pc_keyboard_velocity_client.py" `
  --host orangepi@192.168.31.100 `
  --vx 0.2 --vy 0.2 --yaw 0.5
```

首次实机测试从`vx=0.2 m/s`开始，不要立即提高到`1.0 m/s`。方向反转前先
松开按键并确认命令回零。V45此前在接近`0.9 m/s`时出现过`joint[9]`目标越过
实机机械上限；关节角保护必须保持启用。

## 失联与安全行为

1. PC客户端每100 ms刷新非零三维命令。
2. 所有运动键松开后立即发送`z`，速度由主程序按V45减速斜坡回零。
3. SSH正常结束时，桥接器在退出前发送`z`。
4. SSH异常中断、PC死机或桥接器无法写FIFO时，`robot_control`现有500 ms
   watchdog把目标速度回零。
5. `X`或`Ctrl+C`发送`s`，进入主程序统一锁存故障/阻尼停机路径。
6. PC客户端和桥接器均不能直接访问电机。

网络watchdog不能代替物理急停。实机测试仍应使用吊带保护并准备断电手段。

## 纯软件测试

在香橙派运行：

```bash
cd /home/orangepi/unitree_actuator_sdk

python3 remote_control/tests/test_keyboard_velocity_control.py
./build/test_velocity_command_manager
```

第一项不访问FIFO和硬件；第二项验证V45加减速、反向过零和命令watchdog。

## 备用SSH终端控制器

如果只需要单轴调试，仍可在第二个SSH终端运行：

```bash
cd /home/orangepi/unitree_actuator_sdk
python3 remote_control/keyboard/keyboard_velocity_console.py
```

它现在也使用独立档位和新的速度范围，但不能可靠识别真正的多键按下/松开，
因此不要用它验收组合运动。
