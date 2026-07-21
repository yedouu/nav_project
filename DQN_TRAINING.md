# DQN 强化学习训练文档

## 版本对比

| | V1 | V2 | V3 |
|---|---|---|---|
| **环境** | TurtleBot3World-v0 | TurtleBot3World-v2 | TurtleBot3World-v2 |
| **观测维度** | 120 维激光 | **12 维** (30°采样) | 12 维 |
| **撞墙惩罚** | -200 | **-10** | -10 |
| **距离惩罚** | 无 | **渐进式** (<0.5m扣分) | 渐进式 |
| **前进奖励** | +5 | +2 | +2 |
| **算法** | DQN | DQN | **Double DQN** |
| **网络** | 6层 | 6层 | **自适应**(12维用3层) |
| **epsilon_decay** | 20000 | 10000 | **15000** |
| **target_update** | 50 | 50 | **100** |
| **batch_size** | 32 | 64 | **128** |
| **learning_rate** | 0.0001 | **0.001** | 0.001 |
| **预期效果** | 学不会(5%) | 80%成功率 | 90%+更快收敛 |

## 启动命令

```bash
# V1 (原始，不推荐)
make train-dqn              # 端口 12345

# V2 (改进Reward)
make train-dqn-v2           # 端口 12346

# V3 (Double DQN，推荐)
make train-dqn-v3           # 端口 12347

# 训练 + 自动上传 (需先设置 GITHUB_TOKEN)
make train-dqn-v2-upload    # V2 + 上传
make train-dqn-v3-upload    # V3 + 上传
```

## 自动上传训练结果到仓库

### 设置

1. 在 GitHub 生成 Personal Access Token: Settings → Developer settings → Tokens (classic) → 勾选 `repo` 权限
2. 设置环境变量:

```bash
export GITHUB_TOKEN=ghp_xxxxxxxxxxxx
```

### 使用

```bash
# 手动训练 + 上传
make train-dqn-v3-upload

# 定时自动跑 (crontab，每天晚上 10 点)
# crontab -e 加一行:
0 22 * * * export GITHUB_TOKEN=ghp_xxx; cd /path/to/nav_project && make train-dqn-v3-upload
```

### 上传内容

```
dqn_ros/training_results/results-<timestamp>/   ← 训练曲线 + JSON 数据
dqn_ros/weights/V3_policy_net.pth               ← 策略网络权重
DQN_TRAINING.md                                 ← 本文档
```

Git commit 格式: `Auto V3 - 2026-07-21 22:35 - EP:300 Best:45`

### 上传逻辑

- `docker/upload-results.sh` — V2/V3 公用的上传脚本
- 最新权重自动复制到 `dqn_ros/weights/`，文件名带版本号
- 使用 Token 通过 HTTPS 认证，结束后自动恢复原始 remote URL
- 不设置 `GITHUB_TOKEN` 不会上传，不影响训练

## V2 核心改进原理

### 1. Reward 重设计

**问题**: 原始 reward 极端不平衡。撞墙 -200 vs 前进 +5，需要 40 步才能抵消一次碰撞。DQN 收到的反馈基本上是"撞了"和"没撞"两种极端值，中间没有渐变信号。

**解决**:
- 撞墙 -200 → -10（缩小到前进的 5 倍）
- 新增距离惩罚：离障碍 <0.5m 时 `惩罚 = (0.5 - 距离) × 10`
  - 0.4m → -1分, 0.3m → -2分, 0.22m → 接近撞墙
- 前进 +5 → +2（防止前进奖励淹没距离惩罚）

效果：机器人获得了"倒车雷达"——不用撞墙就知道自己在靠近危险。

### 2. 降维观测

**问题**: 120 维激光数据中，相邻角度的读数几乎相同，大量冗余。简单全连接网络被淹没在噪声中。

**解决**: 120→12 维，每 30° 采样一个方向。信息量不丢（还是能感知周围障碍），维度降 10 倍，训练量降约 5 倍。

### 3. 学习率提升

0.0001 → 0.001，加速 10 倍。配合改进的 reward 信号，学习效率大幅提升。

## V3 新增改进

### Double DQN

**问题**: 标准 DQN 用同一个网络选动作和评估 Q 值，会系统性高估 Q 值（"乐观偏差"）。

**解决**: 
```python
# DQN: target_net 选动作 + 评估 (同一网络)
next_q = target_net(states).max(1)[0]

# Double DQN: policy_net 选动作, target_net 评估 (解耦)
best_actions = policy_net(states).max(1)[1]
next_q = target_net(states).gather(1, best_actions)
```

### 自适应网络

12 维输入用 3 层网络（12→128→128→3），120 维输入用 6 层。减少过拟合，训练更快。

### 稳定参数

- target_update 50→100：Double DQN 需要目标网络更稳定
- batch_size 64→128：更大批次降低梯度方差
- epsilon_decay 10000→15000：折中，既不过快也不拖沓

## 常见问题

### 为什么每行输出都是 [ERROR]？
原作者用 `rospy.logerr()` 打训练日志，不是真的错误。不影响训练。

### 为什么 reward 是小数？
V2/V3 加了距离渐进惩罚，不再只是整数加减。

### 训练可以并行吗？
可以，V1/V2/V3 各自使用独立 Gazebo 端口，可同时运行。你的硬件（i7-13700 + 31GB）能并行 3-4 组。

### 怎么看到学习效果？
打开 `dqn_ros/training_results/` 下的 `plot.png`，reward 曲线持续上升 = 在学习。

### 模型权重在哪？
训练结束后保存在:
```
dqn_ros/training_results/results-<timestamp>/
├── policy_net.pth     ← 策略网络（推理用）
├── target_net.pth     ← 目标网络
├── plot.png           ← reward 曲线
└── results-*.json     ← 完整训练数据
```

## 边缘部署

训练好的模型可直接部署到嵌入式设备（如树莓派 5、Jetson Nano）上推理:

| 平台 | 模型 | 推理耗时 | 部署方式 |
|------|------|---------|---------|
| 树莓派 5 | `policy_net.pth` | <5ms | PyTorch / ONNX Runtime |
| Jetson Orin Nano | `policy_net.pth` | <1ms | PyTorch / TensorRT |
| x86 工控机 | `policy_net.pth` | <1ms | PyTorch / ONNX |

模型仅 ~72KB (V3 3 层网络)，ROS 20Hz 控制周期下推理开销可忽略。详见上面足球 PPO 方案的讨论。

## 故障排查

```bash
# 清理所有训练进程
pkill -9 -f gz; pkill -9 Xvfb; docker rm -f $(docker ps -aq) 2>/dev/null

# 查看某个端口是否被占用
ss -tlnp | grep -E "12345|12346|12347"

# 杀掉特定端口
fuser -k 12346/tcp
```
