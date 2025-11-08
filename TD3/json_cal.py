import json
import os
import numpy as np

# 结果文件所在目录
results_dir = "/home/sanshiqi/project/DRL-robot-navigation/TD3/test_results"  # 替换为你的结果文件目录

# 遍历目录下所有JSON结果文件
for filename in os.listdir(results_dir):
    if filename.endswith(".json"):
        file_path = os.path.join(results_dir, filename)
        print(f"\n处理文件：{filename}")
        
        # 读取文件数据
        with open(file_path, "r") as f:
            data = json.load(f)
        
        # 提取各项指标（根据实际字段调整）
        episodes = len(data)
        success = [d["success"] for d in data]  # 成功标志（布尔值）
        collision = [d["collision"] for d in data]  # 机器狗碰撞标志
        human_collision = [d["human_collision"] for d in data]  # 人类碰撞标志
        time_cost = [d["time_cost"] for d in data]  # 回合耗时
        path_length = [d["path_length"] for d in data]  # 轨迹长度
        linear_acc = [d["linear_acc_avg"] for d in data]  # 平均线加速度
        angular_acc = [d["angular_acc_avg"] for d in data]  # 平均角加速度
        linear_jerk = [d["linear_jerk_avg"] for d in data]  # 平均线加加速度
        angular_jerk = [d["angular_jerk_avg"] for d in data]  # 平均角加加速度
        
        # 计算平均值（数值型）和比例（布尔型）
        avg_time = np.mean(time_cost)
        avg_path = np.mean(path_length)
        avg_linear_acc = np.mean(linear_acc)
        avg_angular_acc = np.mean(angular_acc)
        avg_linear_jerk = np.mean(linear_jerk)
        avg_angular_jerk = np.mean(angular_jerk)
        
        success_rate = np.mean(success) * 100  # 转换为百分比
        collision_rate = np.mean(collision) * 100
        human_collision_rate = np.mean(human_collision) * 100
        
        # 打印每个文件的统计结果
        print(f"总回合数：{episodes}")
        print(f"成功率：{success_rate:.2f}%")
        print(f"机器狗碰撞率：{collision_rate:.2f}%")
        print(f"人类碰撞率：{human_collision_rate:.2f}%")
        print(f"平均运动时间：{avg_time:.2f}秒")
        print(f"平均轨迹长度：{avg_path:.2f}米")
        print(f"平均线加速度（平滑性）：{avg_linear_acc:.4f}")
        print(f"平均角加速度（平滑性）：{avg_angular_acc:.4f}")
        print(f"平均线加加速度（流畅性）：{avg_linear_jerk:.4f}")
        print(f"平均角加加速度（流畅性）：{avg_angular_jerk:.4f}")
        print("-" * 50)
