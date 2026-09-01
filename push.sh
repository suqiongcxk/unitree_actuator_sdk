#!/bin/bash
# 一键 git add + commit + push
# 用法: ./push.sh "提交信息"
#       ./push.sh            (自动生成提交信息)

set -e

MESSAGE="${1:-auto commit $(date '+%Y-%m-%d %H:%M')}"

echo "=== 变更文件 ==="
git status -s

# if [ -z "$(git status -s)" ]; then
#     echo "工作区干净，无需提交"
#     exit 0
# fi

echo ""
echo "=== 提交信息: $MESSAGE ==="
read -p "确认提交并推送? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "已取消"
    exit 0
fi

git add -A
git commit -m "$MESSAGE"
git push origin main

echo ""
echo "=== 推送完成 ==="
git log --oneline -1
