#!/bin/bash

# ==============================================================================
# Git 仓库自动化更新、提交与标签管理脚本 (增强版)
# 版本：4.0.0
# 功能：
# 1. 检查当前 Git 状态，并显示当前分支
# 2. 列出所有本地分支，让用户选择要操作的分支
# 3. 如果工作区不干净，提供提交选项
# 4. 拉取远程仓库最新代码，使用 rebase 策略
# 5. 推送本地提交到用户选择的远程分支
# 6. 引导用户创建并推送新的 Git 标签
# ==============================================================================

echo "🚀 开始自动化 Git 流程..."

# 1. 检查当前目录是否为 Git 仓库
if ! git rev-parse --is-inside-work-tree > /dev/null 2>&1; then
    echo "❌ 错误：当前目录不是一个 Git 仓库。请在仓库根目录执行此脚本。"
    exit 1
fi

# 获取当前分支名
current_branch=$(git rev-parse --abbrev-ref HEAD)
echo "➡️ 当前分支：${current_branch}"

# 2. 选择要操作的分支
echo ""
echo "请选择要操作的分支："
branches=($(git branch --format="%(refname:short)"))
for i in "${!branches[@]}"; do
    echo "  $((i+1))) ${branches[$i]}"
done
read -p "请输入分支对应的数字（默认为 ${current_branch}）： " branch_choice

if [[ -z "$branch_choice" ]]; then
    target_branch=$current_branch
elif [[ "$branch_choice" -le ${#branches[@]} && "$branch_choice" -gt 0 ]]; then
    target_branch=${branches[$((branch_choice-1))]}
else
    echo "❌ 错误：无效的选项。操作取消。"
    exit 1
fi

echo "✅ 你选择的分支是：${target_branch}"
echo ""

# 3. 尝试切换到目标分支
if [[ "$target_branch" != "$current_branch" ]]; then
    echo "➡️ 正在尝试切换到分支：${target_branch}..."
    if ! git checkout "$target_branch" > /dev/null 2>&1; then
        echo "❌ 错误：切换分支失败。当前工作区不干净。"
        read -p "是否要提交当前分支'${current_branch}'的修改？ (y/n): " confirm_commit
        if [[ "$confirm_commit" == "y" ]]; then
            echo "➡️ 正在暂存所有文件变更..."
            git add .
            read -p "请输入提交信息： " commit_message
            if [[ -z "$commit_message" ]]; then
                echo "❌ 错误：提交信息不能为空。操作取消。"
                exit 1
            fi
            if ! git commit -m "$commit_message"; then
                echo "❌ 错误：提交失败，请手动检查。"
                exit 1
            fi
            echo "✅ 提交成功，再次尝试切换分支..."
            if ! git checkout "$target_branch" > /dev/null 2>&1; then
                echo "❌ 错误：再次切换分支失败，请手动解决问题。"
                exit 1
            fi
            echo "✅ 已成功切换到分支：${target_branch}。"
        else
            echo "✅ 用户选择不提交。请手动解决工作区问题后再次运行脚本。"
            exit 0
        fi
    else
        echo "✅ 已成功切换到分支：${target_branch}。"
    fi
fi

# 4. 拉取最新代码
echo "➡️ 正在拉取远程仓库最新代码..."
if ! git pull --rebase; then
    echo "❌ 错误：拉取最新代码失败。可能存在合并冲突，请手动解决后重试。"
    exit 1
fi
echo "✅ 代码已成功同步到最新。"

# 5. 检查并提交修改
echo "➡️ 正在暂存所有文件变更..."
git add .
if git diff --cached --quiet; then
    echo "✅ 没有新的修改需要提交，跳过提交步骤。"
else
    read -p "请输入提交信息（例如: 'feat: add new feature'）： " commit_message
    if [[ -z "$commit_message" ]]; then
        echo "❌ 错误：提交信息不能为空。操作取消。"
        exit 1
    fi
    if ! git commit -m "$commit_message"; then
        echo "❌ 错误：提交失败，请检查文件状态。"
        exit 1
    fi
    echo "✅ 提交成功。"

    # 6. 推送提交
    echo "➡️ 推送提交到远程仓库..."
    if ! git push origin "${target_branch}"; then
        echo "❌ 错误：推送失败。可能存在网络问题或远程仓库已更新，请再次运行脚本。"
        exit 1
    fi
    echo "✅ 代码已成功推送到远程分支'${target_branch}'。"
fi

---

### **标签管理部分**

echo ""
echo "----------------------------------------"
echo "🏷️ 开始处理 Git 标签..."
read -p "请输入新的版本标签（例如: 'v1.0.0' 或 'v1.1.0'），或直接回车跳过： " tag_name

if [[ -z "$tag_name" ]]; then
    echo "✅ 跳过标签创建，脚本执行完毕。"
    exit 0
fi

if git rev-parse --verify "refs/tags/${tag_name}" >/dev/null 2>&1; then
    echo "⚠️ 警告：标签 '$tag_name' 已存在。请使用一个新标签名。"
else
    git tag -a "$tag_name" -m "Release $tag_name"
    echo "✅ 本地标签 '$tag_name' 已创建。"
    git push origin "$tag_name"
    echo "✅ 标签 '$tag_name' 已推送到远程仓库。"
fi

echo ""
echo "🎉 脚本执行完毕。仓库已更新并上传标签。"