# RK3588 WebSocket 传输层冒烟测试

本目录只验证以下最小链路：

```text
RK3588 每秒主动上报状态 → WebSocket 客户端
WebSocket 客户端发送探针 → RK3588 返回 ACK
```

它不访问电机、串口、GPIO、控制 FIFO 或 `robot_control`，消息名均带
`transport_*_v0`，不是正式的 Protocol V1。

## 1. 在香橙派启动服务器

```bash
cd /home/orangepi/unitree_actuator_sdk
python3 wechat_mini_program/rk3588_gateway/smoke_test/ws_smoke_server.py
```

看到以下输出即表示正在监听：

```text
[启动] ws://0.0.0.0:8765
```

## 2. 在香橙派本机验证

另开一个 SSH 终端：

```bash
cd /home/orangepi/unitree_actuator_sdk
python3 wechat_mini_program/rk3588_gateway/smoke_test/ws_smoke_client.py
```

出现 `[PASS]` 证明服务端的主动上报、客户端发送和 ACK 三条路径都正常。

## 3. 从同一 Wi-Fi 下的电脑验证

将这个 `smoke_test` 文件夹复制到电脑并执行：

```bash
python ws_smoke_client.py --url ws://192.168.31.100:8765
```

手机微信小程序联调时，开发阶段同样连接：

```text
ws://192.168.31.100:8765
```

上面的地址适用于 RK3588 连接 `Robotstar` 路由器时。切换为 Robot AP 后使用：

```text
ws://192.168.50.1:8765
```

开发者工具中可能需要临时关闭合法域名校验。正式发布时应使用备案域名、TLS
证书和 `wss://`，具体以微信小程序当前平台要求为准。

## 4. 停止

服务器终端按 `Ctrl+C`。该程序没有启动机器人控制，因此不会发送任何电机指令。

## 户外使用建议

- 近距离：让机器人建立 Wi-Fi 热点，手机直连热点；不依赖家庭路由器。
- 简单方案：机器人连接手机热点或随身 4G/5G 路由器。
- 真正远程：机器人和手机均联网，通过带认证和 TLS 的公网服务中转。
- 完全没有 Wi-Fi、蜂窝网络或其他无线链路时，手机无法遥控机器人。
