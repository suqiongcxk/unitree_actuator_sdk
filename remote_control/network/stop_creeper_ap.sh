#!/usr/bin/env bash
set -euo pipefail

readonly CONNECTION_NAME="Creeper_Dog_AP"
readonly DEFAULT_CLIENT_WIFI="Robotstar"

if [[ ${EUID} -ne 0 ]]; then
    echo "请使用 sudo 运行：sudo $0"
    exit 1
fi

echo "即将关闭 Creeper_Dog，并尝试恢复 ${DEFAULT_CLIENT_WIFI}。"
echo "当前通过 192.168.50.1 建立的 SSH/Codex 连接会断开。"
read -r -p "必须在 RK3588 本机终端确认，输入 STOP 继续：" CONFIRM
if [[ ${CONFIRM} != "STOP" ]]; then
    echo "已取消，网络未改变。"
    exit 0
fi

nmcli connection down "${CONNECTION_NAME}" 2>/dev/null || true
nmcli connection up "${DEFAULT_CLIENT_WIFI}"

echo "已关闭热点并请求连接 ${DEFAULT_CLIENT_WIFI}。"
nmcli -f DEVICE,TYPE,STATE,CONNECTION device status
