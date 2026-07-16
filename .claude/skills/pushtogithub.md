---
name: pushtogithub
description: 自动执行 git add、commit、push 流程，将当前工程变更上传到 GitHub
---

# /pushtogithub

将当前工作区所有变更自动提交并推送到 GitHub 远程仓库。

## 执行步骤

### 1. 检查状态
```bash
git status
```
- 如果没有变更，报告"工作区干净，无需提交"并结束
- 如果有变更，列出变更文件清单给用户确认

### 2. 生成 commit message
- 根据 `git diff --stat` 总结变更内容，用中文写 commit message
- 格式: `<类型>: <简短描述>`
- 类型: 修复/新增/重构/文档/清理 等
- **将 message 展示给用户确认**（用 AskUserQuestion 给用户 yes/no 选择），用户也可以手动输入

### 3. 暂存并提交
```bash
git add -A
git commit -m "<用户确认的message>"
```
- 排除 `.claude/` 下的临时文件（.gitignore 已配置）
- 提交后显示 commit hash

### 4. 推送
```bash
git push origin main
```
- 推送前告知用户目标分支
- 如果推送失败（如认证问题），提示用户手动处理

## 安全规则
- 永远不要提交包含密码、token、密钥的文件
- 如果发现 `settings.local.json` 等配置文件被暂存，先 `git restore --staged` 移除
- `build/`、`lib/*.a`、`lib/*.so` 等构建产物由 .gitignore 保护
