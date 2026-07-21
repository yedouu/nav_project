#!/bin/bash
# 训练结果自动上传脚本 (V2/V3 共用)
# 用法: source upload-results.sh && upload_results

upload_results() {
    echo ""
    echo "========================================="
    echo " 上传结果到 GitHub"
    echo "========================================="

    cd /root/nav_project

    LATEST_DIR=$(ls -td dqn_ros/training_results/results-2026-* 2>/dev/null | head -1)
    if [ -z "$LATEST_DIR" ]; then
        echo "[SKIP] 未找到训练结果目录"
        return 0
    fi

    if [ -f "$LATEST_DIR/policy_net.pth" ]; then
        mkdir -p dqn_ros/weights
        cp "$LATEST_DIR/policy_net.pth" "dqn_ros/weights/${TRAIN_VERSION}_policy_net.pth"
        echo "[OK] 权重 -> dqn_ros/weights/${TRAIN_VERSION}_policy_net.pth"
    else
        echo "[WARN] 未找到 policy_net.pth, 跳过权重"
    fi

    git config user.email "train-bot@auto"
    git config user.name "DQN-Training-Bot"

    REPO_URL=$(git remote get-url origin)
    if echo "$REPO_URL" | grep -q "^https://"; then
        AUTH_URL=$(echo "$REPO_URL" | sed "s|https://|https://$GITHUB_TOKEN@|")
    elif echo "$REPO_URL" | grep -q "^git@"; then
        REPO_PATH=$(echo "$REPO_URL" | sed 's|git@github.com:|github.com/|')
        AUTH_URL="https://$GITHUB_TOKEN@$REPO_PATH"
    fi
    git remote set-url origin "$AUTH_URL"

    git pull --rebase origin main 2>/dev/null || git pull --rebase origin master 2>/dev/null || true

    git add "$LATEST_DIR" dqn_ros/weights/ DQN_TRAINING.md 2>/dev/null || true

    if git diff --cached --quiet 2>/dev/null; then
        echo "[SKIP] 无新文件"
    else
        TIMESTAMP=$(date '+%Y-%m-%d %H:%M')
        SUMMARY=$(ls "$LATEST_DIR"/*.json 2>/dev/null | head -1 | xargs python3 -c \
            "import json,sys; d=json.load(open(sys.argv[1])); print(f'EP:{len(d[\"rewards\"])} Best:{max(d[\"rewards\"]):.0f}')" 2>/dev/null || echo "N/A")
        git commit -m "Auto ${TRAIN_VERSION} - $TIMESTAMP - $SUMMARY"
        git push origin HEAD
        echo "[OK] 已推送!"
    fi

    git remote set-url origin "$REPO_URL"
}

# 入口
TRAIN_VERSION=${1:-unknown}
UPLOAD=${UPLOAD_RESULTS:-false}
if [ "$UPLOAD" = "true" ]; then
    if [ -z "$GITHUB_TOKEN" ]; then
        echo "[SKIP] UPLOAD_RESULTS=true 但 GITHUB_TOKEN 未设置"
    else
        upload_results
    fi
else
    echo "[INFO] 跳过上传。开启: -e UPLOAD_RESULTS=true -e GITHUB_TOKEN=ghp_xxx"
fi
