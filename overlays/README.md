# 设备树 Overlay（可选 / 实验性）

此目录包含 RK3588 设备树 overlay，用于 **硬件 RS-485 模式**（TIOCSRS485）的实验性支持。

## 当前状态

**未部署。** 当前代码使用软件 GPIO 控制 RS-485 方向（FastGPIO + TIOCSERGETLSR），
不需要这些 overlay。

## 文件说明

| 文件 | 说明 |
|------|------|
| `rk3588-uart4-m2-rs485.dts` | 设备树源文件：将 UART4 RTS 映射到 GPIO1_D7 |
| `rk3588-uart4-m2-rs485.dtbo` | 编译后的 device tree blob |

## 使用场景

如果将来需要使用内核级的 RS-485 自动方向控制（`TIOCSRS485` ioctl），
可以部署此 overlay。

## 编译 & 部署

```bash
# 编译
dtc -@ -I dts -O dtb -o rk3588-uart4-m2-rs485.dtbo rk3588-uart4-m2-rs485.dts

# 安装
sudo cp rk3588-uart4-m2-rs485.dtbo /boot/dtb/rockchip/overlay/

# 启用（在 /boot/orangepiEnv.txt 中添加）
overlays=uart4-m2 uart4-m2-rs485
```

> ⚠ **注意：** Orange Pi 5 Pro 未引出 UART4 硬件 RTS 引脚（GPIO1_D4），
> 此 overlay 将 RTS 重映射到 GPIO1_D7。未经充分测试，存在风险。
