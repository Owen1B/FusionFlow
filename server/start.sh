#!/bin/bash

SESSION=smartinfusion

# 检查会话是否存在
if tmux has-session -t $SESSION 2>/dev/null; then
    echo "发现已存在的会话，正在重启服务..."
    # 停止现有会话
    tmux kill-session -t $SESSION
else
    echo "未发现现有会话，正在启动新服务..."
fi

# 创建新的会话
tmux new-session -d -s $SESSION

# 创建前端构建窗口
tmux rename-window -t $SESSION:0 'frontend-build'
echo "正在构建前端..."
tmux send-keys -t $SESSION:0 'cd ~/smartinfusion/frontend && npm run build' C-m

# 等待前端构建完成
sleep 10

# 创建后端窗口并启动后端服务
tmux new-window -t $SESSION:1 -n 'backend'
tmux send-keys -t $SESSION:1 'cd ~/smartinfusion/backend && ./start.sh' C-m

# 创建前端开发窗口并启动前端服务
tmux new-window -t $SESSION:2 -n 'frontend-dev'
tmux send-keys -t $SESSION:2 'cd ~/smartinfusion/frontend && npm start' C-m

echo "服务已启动，使用 'tmux attach -t $SESSION' 查看服务状态。" 