#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np

class JointController:
    def __init__(self):
        rospy.init_node('joint_controller')
        
        # 定义关节名称和初始位置
        self.joint_names = ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', 
                           'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
                           'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
                           'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint']
        
        # 创建发布者字典
        self.publishers = {}
        for name in self.joint_names:
            topic = f'/go2_gazebo/{name}_controller/command'
            self.publishers[name] = rospy.Publisher(topic, Float64, queue_size=10)
        
        # 订阅当前关节状态
        self.current_positions = {name: 0.0 for name in self.joint_names}
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        # 设置发布频率
        self.rate = rospy.Rate(100)  # 100Hz
    
    def joint_state_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                self.current_positions[name] = msg.position[i]
    
    def set_joint_positions(self, target_positions):
        """设置关节位置（位置控制）"""
        for name, pos in target_positions.items():
            if name in self.joint_names:
                self.publishers[name].publish(pos)
        self.rate.sleep()
    
    def set_joint_efforts(self, target_efforts):
        """设置关节力矩（力矩控制）"""
        for name, effort in target_efforts.items():
            if name in self.joint_names:
                self.publishers[name].publish(effort)
        self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = JointController()
        
        # 设置目标位置（弧度）
        target_positions = {
            'FL_hip_joint': 0.5,
            'FL_thigh_joint': 0.8,
            'FL_calf_joint': -1.5,
            # 其他关节...
        }
        
        # 持续发布指令直到节点关闭
        while not rospy.is_shutdown():
            controller.set_joint_positions(target_positions)
    except rospy.ROSInterruptException:
        pass
