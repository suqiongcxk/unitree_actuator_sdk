#!/usr/bin/env bash
set -euo pipefail

readonly CONNECTION_NAME="Creeper_Dog_AP"

if [[ ${EUID} -ne 0 ]]; then
    echo "请使用 sudo 运行：sudo $0"
    exit 1
fi

if ! nmcli -t -f NAME connection show | grep -Fxq "${CONNECTION_NAME}"; then
    echo "错误：热点配置不存在，请先运行 setup_creeper_ap.sh。"
    exit 1
fi

echo "警告：即将把 wlan0 从当前 Wi-Fi 切换为 Creeper_Dog 热点。"
echo "当前 SSH/Codex 连接会断开；热点启用后请连接 Creeper_Dog，再访问 192.168.50.1。"
read -r -p "必须在 RK3588 本机终端确认，输入 START 继续：" CONFIRM
if [[ ${CONFIRM} != "START" ]]; then
    echo "已取消，网络未改变。"
    exit 0
fi

nmcli connection up "${CONNECTION_NAME}"

echo "Creeper_Dog 已启动。"
echo "SSH 地址：orangepi@192.168.50.1"
