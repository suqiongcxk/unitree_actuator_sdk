#!/usr/bin/env bash
set -euo pipefail

readonly CONNECTION_NAME="Creeper_Dog_AP"
readonly AP_SSID="Creeper_Dog"
readonly AP_ADDRESS="192.168.50.1/24"

if [[ ${EUID} -ne 0 ]]; then
    echo "请使用 sudo 运行：sudo $0"
    exit 1
fi

if ! command -v nmcli >/dev/null 2>&1; then
    echo "错误：系统中没有 nmcli。"
    exit 1
fi

read -r -s -p "请输入 Creeper_Dog 热点密码（至少 8 个字符）：" AP_PASSWORD
echo
if (( ${#AP_PASSWORD} < 8 || ${#AP_PASSWORD} > 63 )); then
    echo "错误：WPA2 密码长度必须为 8～63 个字符。"
    exit 1
fi

# 避免旧的 hostapd/dnsmasq 服务与 NetworkManager 的 AP/DHCP 冲突。
systemctl disable --now hostapd.service 2>/dev/null || true
systemctl disable --now dnsmasq.service 2>/dev/null || true

if nmcli -t -f NAME connection show | grep -Fxq "${CONNECTION_NAME}"; then
    nmcli connection delete "${CONNECTION_NAME}"
fi

nmcli connection add \
    type wifi \
    ifname wlan0 \
    con-name "${CONNECTION_NAME}" \
    autoconnect no \
    ssid "${AP_SSID}"

nmcli connection modify "${CONNECTION_NAME}" \
    802-11-wireless.mode ap \
    802-11-wireless.band bg \
    802-11-wireless.channel 6 \
    802-11-wireless.powersave 2 \
    802-11-wireless-security.key-mgmt wpa-psk \
    802-11-wireless-security.proto rsn \
    802-11-wireless-security.pairwise ccmp \
    802-11-wireless-security.psk "${AP_PASSWORD}" \
    ipv4.method shared \
    ipv4.addresses "${AP_ADDRESS}" \
    ipv6.method disabled \
    connection.autoconnect no

unset AP_PASSWORD

echo "热点配置已创建，但尚未启用，当前网络不会切换。"
echo "SSID: ${AP_SSID}"
echo "RK3588 地址: 192.168.50.1"
echo "需要开启时，请在 RK3588 本机终端运行："
echo "  sudo /home/orangepi/unitree_actuator_sdk/remote_control/network/start_creeper_ap.sh"
