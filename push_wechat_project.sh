#!/usr/bin/env bash
#
# 将微信小程序改动同时发布到：
#   1. 机器人工程 origin/CXK
#   2. 独立小程序仓库 wechat/CXK（仅包含 wechat_mini_program/）
#
# 用法：
#   ./push_wechat_project.sh
#   ./push_wechat_project.sh "完成登录页面"

set -Eeuo pipefail

readonly EXPECTED_BRANCH="CXK"
readonly SUBTREE_DIR="wechat_mini_program"
readonly ROBOT_REMOTE="origin"
readonly WECHAT_REMOTE="wechat"
readonly SCRIPT_PATH="push_wechat_project.sh"

stage="初始化"
on_error() {
    local exit_code=$?
    echo
    echo "[失败] 阶段：${stage}，退出码：${exit_code}" >&2
    echo "已经成功完成的 Git 操作不会自动回滚；修复问题后可直接重跑本脚本。" >&2
    exit "${exit_code}"
}
trap on_error ERR

usage() {
    cat <<'EOF'
用法：
  ./push_wechat_project.sh
  ./push_wechat_project.sh "本次提交说明"

不提供说明时，自动使用带时间的提交信息。
脚本只自动暂存 wechat_mini_program/ 和脚本自身，不会提交其他目录的改动。
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi
if (( $# > 1 )); then
    usage >&2
    exit 2
fi

stage="检查 Git 仓库"
repo_root="$(git rev-parse --show-toplevel)"
cd "${repo_root}"

current_branch="$(git branch --show-current)"
if [[ "${current_branch}" != "${EXPECTED_BRANCH}" ]]; then
    echo "错误：当前分支是 '${current_branch}'，必须先切换到 '${EXPECTED_BRANCH}'。" >&2
    exit 2
fi
if [[ ! -d "${SUBTREE_DIR}" ]]; then
    echo "错误：找不到 ${SUBTREE_DIR}/。" >&2
    exit 2
fi
if [[ -n "$(git diff --name-only --diff-filter=U)" ]]; then
    echo "错误：仓库中仍有未解决的合并冲突。" >&2
    exit 2
fi
if ! git diff --cached --quiet; then
    echo "错误：暂存区已有内容。为防止误提交，请先提交或取消暂存。" >&2
    exit 2
fi
for remote in "${ROBOT_REMOTE}" "${WECHAT_REMOTE}"; do
    if ! git remote get-url "${remote}" >/dev/null 2>&1; then
        echo "错误：缺少 Git remote '${remote}'。" >&2
        exit 2
    fi
done

stage="检查改动"
git diff --check -- "${SUBTREE_DIR}" "${SCRIPT_PATH}"

stage="提交小程序改动"
git add -A -- "${SUBTREE_DIR}" "${SCRIPT_PATH}"
if git diff --cached --quiet; then
    echo "[信息] 没有新的小程序改动需要提交，将继续同步现有提交。"
else
    commit_message="${1:-更新微信小程序 $(date '+%Y-%m-%d %H:%M:%S')}"
    git commit -m "${commit_message}"
fi

stage="推送机器人工程 ${ROBOT_REMOTE}/${EXPECTED_BRANCH}"
git push -u "${ROBOT_REMOTE}" "${EXPECTED_BRANCH}"
echo "[完成] 机器人工程已推送到 ${ROBOT_REMOTE}/${EXPECTED_BRANCH}"

stage="推送独立小程序仓库 ${WECHAT_REMOTE}/${EXPECTED_BRANCH}"
git subtree push \
    --prefix="${SUBTREE_DIR}" \
    "${WECHAT_REMOTE}" "${EXPECTED_BRANCH}"
echo "[完成] 小程序目录已推送到 ${WECHAT_REMOTE}/${EXPECTED_BRANCH}"

stage="完成"
echo
echo "[全部完成] 两个仓库的 ${EXPECTED_BRANCH} 均已同步。"
