# from GDAM_env import ImplementEnv
# import tensorflow as tf
# import tflearn
# import numpy as np
# from GDAM_args import d_args

import time
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from GDAM_env import ImplementEnv
from GDAM_args import d_args


class Actor(nn.Module):
    def __init__(self, state_dim=23, action_dim=2):
        super(Actor, self).__init__()
        self.layer_1 = nn.Linear(state_dim, 800)
        self.layer_2 = nn.Linear(800, 600)
        self.layer_3 = nn.Linear(600, action_dim)
        self.tanh = nn.Tanh()

    def forward(self, s):
        s = F.relu(self.layer_1(s))
        s = F.relu(self.layer_2(s))
        a = self.tanh(self.layer_3(s))
        # 对应原代码中的 scaled_out = tf.multiply(out, [1, 1])
        scaled_a = a * torch.tensor([1, 1], device=s.device)
        return scaled_a, s  # 返回缩放后的动作和输入


class TD3(object):
    def __init__(self, state_dim, action_dim, device):
        self.actor = Actor(state_dim, action_dim).to(device)
        self.actor_target = Actor(state_dim, action_dim).to(device)
        self.actor_target.load_state_dict(self.actor.state_dict())
    def get_action(self, state):
        # 根据状态生成动作
        state = torch.Tensor(state.reshape(1, -1)).to(self.device)
        with torch.no_grad():
            action = self.actor(state).cpu().data.numpy().flatten()  # 转回numpy
        return action
    def load(self, directory, filename):
        self.actor.load_state_dict(
            torch.load(f"{directory}/{filename}_actor.pth", map_location=self.device)
        )

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
seed = 0
max_ep_timesteps = 500  # 单轮最大步数
file_name = "TD3_velodyne"  # 模型文件名
state_dim = 20

env = ImplementEnv(d_args)
time.sleep(5)

torch.manual_seed(seed)
np.random.seed(seed)

action_dim = 2

# 创建网络实例
network = TD3(state_dim, action_dim, device)

# 加载预训练模型
try:
    network.load(file_name, "./pytorch_models")  # 模型存储路径
except Exception as e:
    raise ValueError(f"Failed to load model: {e}")

# 测试循环
done = False
episode_timesteps = 0
state = env.reset()  # 重置环境获取初始状态

while True:
    # 获取动作
    action = network.get_action(np.array(state))

    # 动作转换：与参考代码保持一致的缩放逻辑
    # 线速度映射到[0,1]，角速度映射到[-1,1]（根据你的需求调整系数）
    a_in = [
        (action[0] + 1) / 4,  # 原代码中此处为/4，根据实际范围调整
        action[1] / 4         # 角速度缩放
    ]

    # 与环境交互
    next_state, toGoal = env.step(a_in)  # 适配你的环境step返回值

    # 检查是否达到最大步数
    episode_timesteps += 1
    if episode_timesteps >= max_ep_timesteps:
        done = True

    # 重置条件
    if done:
        print(f"Episode finished. Resetting environment...")
        state = env.reset()
        done = False
        episode_timesteps = 0
    else:
        # 更新状态（根据你的环境返回值构造新状态）
        # 原代码中状态拼接逻辑：s = np.append(s2, a[0]); s = np.append(s, toGoal)
        state = np.append(next_state, action[0])
        state = np.append(state, toGoal)
        # 确保状态维度始终为23（截断或补零，根据实际情况调整）
        state = state[:23]