import math
import os
import random
import subprocess
import time
from os import path

import json
from datetime import datetime
import numpy as np
import geometry_msgs
import rospy
import sensor_msgs.point_cloud2 as pc2
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped  # 添加ekf消息类型
from sensor_msgs.msg import PointCloud2
from squaternion import Quaternion
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import JointState  # 添加此行
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Bool
from std_msgs.msg import Empty  # 添加Empty消息类型
from std_srvs.srv import Empty as EmptyService  # 服务类型
from std_msgs.msg import Empty as EmptyMsg      # 消息类型
from std_msgs.msg import Header  # 添加此行
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist  # 用于轨迹点的定义

GOAL_REACHED_DIST = 0.3
COLLISION_DIST = 0.35  # 避障距离
CENTER_COLLISION_DIST = 0.5  # 避障距离
TIME_DELTA = 1.0


# Check if the random goal position is located on an obstacle and do not accept it if it is
def check_pos(x, y):
    goal_ok = True
    # 障碍物n判定范围变大，防止ｏｄｏｍ漂移导致目标点进入障碍物
    if -3.6 > x > -6.2 and 6.2 > y > 3.4:
        goal_ok = False

    if -1.1 > x > -2.9 and 4.7 > y > -0.4:
        goal_ok = False

    if -0.1 > x > -4.4 and 2.9 > y > 1.1:
        goal_ok = False

    if -1.0 > x > -4.4 and -2.1 > y > -4.4:
        goal_ok = False

    if -1.1 > x > -3.9 and -0.6 > y > -2.9:
        goal_ok = False

    if 4.4 > x > 0.6 and -1.6 > y > -3.4:
        goal_ok = False

    if 4.2 > x > 2.3 and 0.9 > y > -3.6:
        goal_ok = False

    if 6.2 > x > 3.6 and -3.1 > y > -4.4:
        goal_ok = False

    if 4.4 > x > 1.1 and 3.9 > y > 1.3:
        goal_ok = False

    if -2.8 > x > -7.2 and 0.3 > y > -1.7:
        goal_ok = False

    if x > 4.0 or x < -4.0 or y > 4.0 or y < -4.0:
        goal_ok = False

    return goal_ok


class GazeboEnv:
    """Superclass for all Gazebo environments."""

    def __init__(self, launchfile, environment_dim):
        self.environment_dim = environment_dim
        self.odom_x = 0
        self.odom_y = 0

        self.goal_x = 1
        self.goal_y = 0.0

        self.upper = 9.0
        self.lower = -9.0
        self.velodyne_data = np.ones(self.environment_dim) * 10
        self.last_odom = None

        self.set_self_state = ModelState()
        self.set_self_state.model_name = "go2"
        self.set_self_state.pose.position.x = 0.0
        self.set_self_state.pose.position.y = 0.0
        self.set_self_state.pose.position.z = 0.25
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 1.0

        # 存储gazebo中位置
        self.true_x = 0.0
        self.true_y = 0.0
        self.true_yaw = 0.0
        # 同时新增模型状态话题订阅
        self.model_state_sub = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.model_state_callback, queue_size=1
        )

        # self.gaps = [[-np.pi / 2 - 0.03, -np.pi / 2 + np.pi / self.environment_dim]]
        # for m in range(self.environment_dim - 1):
        #     self.gaps.append(
        #         [self.gaps[m][1], self.gaps[m][1] + np.pi / self.environment_dim]
        #     )
        # self.gaps[-1][-1] += 0.03
        self.gaps = []
        angle_step = 2 * np.pi / self.environment_dim
        for i in  range(self.environment_dim):
            start = -np.pi + i * angle_step
            end = start + angle_step
            self.gaps.append([start, end])

        port = "11311"
        subprocess.Popen(["roscore", "-p", port])

        print("Roscore launched!")

        # Launch the simulation with the given launchfile name
        rospy.init_node("gym", anonymous=True)
        if launchfile.startswith("/"):
            fullpath = launchfile
        else:
            fullpath = os.path.join(os.path.dirname(__file__), "assets", launchfile)
        if not path.exists(fullpath):
            raise IOError("File " + fullpath + " does not exist")

        subprocess.Popen(["roslaunch", "-p", port, fullpath])
        print("Gazebo launched!")

        # Set up the ROS publishers and subscribers
        self.vel_pub = rospy.Publisher("/go2_gazebo/cmd_vel", Twist, queue_size=1)
        self.set_state = rospy.Publisher(
            "gazebo/set_model_state", ModelState, queue_size=10
        )
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", EmptyService)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", EmptyService)
        self.reset_proxy = rospy.ServiceProxy("/gazebo/reset_world", EmptyService)
        self.publisher = rospy.Publisher("goal_point", MarkerArray, queue_size=3)
        self.publisher2 = rospy.Publisher("linear_velocity", MarkerArray, queue_size=1)
        self.publisher3 = rospy.Publisher("angular_velocity", MarkerArray, queue_size=1)
        self.velodyne = rospy.Subscriber(
            "/velodyne_points", PointCloud2, self.velodyne_callback, queue_size=1
        )
        self.odom = rospy.Subscriber(
            "odom", Odometry, self.odom_callback, queue_size=1
        )

        # 添加ekf订阅器
        # self.ekf_sub = rospy.Subscriber(
        #     "/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.odom_callback, queue_size=1
        # )
        # self.last_ekf_pose = PoseWithCovarianceStamped() # 用于存储最新的ekf位姿数据

        self.reset_count = 0
        self.target_count = 0
        self.collision_count = 0
        self.human_collision_dist = 0.25  # 人类碰撞距离阈值
        self.human_front_safety_dist = 1.5  # 人类前方安全距离阈值
        self.human_left_angle_range = np.radians(10)  # 人类左侧安全区角度范围（弧度）
        self.human_right_angle_range = np.radians(90)  # 人类右侧安全区角度范围（弧度）
        self.human_collision = False
        self.human_pos = None  # 初始化人类位置变量
        self.center_offset_x = -0.125
        self.center_offset_y = -0.075
        self.laser_offset_x = 0.125
        self.laser_offset_y = 0.0
        # 跟踪历史速度
        self.prev_linear_vel = 0.0  # 前一时刻线速度
        self.prev_angular_vel = 0.0  # 前一时刻角速度
        self.prev_linear_acc = 0.0  # 前一时刻线加速度
        self.prev_angular_acc = 0.0  # 前一时刻角加速度
        self.TIME_DELTA = TIME_DELTA  # 时间间隔（从常量导入）

        self.center_collision_dist = CENTER_COLLISION_DIST  # 最小避障距离（碰撞阈值）
        self.center_punish_dist = 1.5     # 距离惩罚阈值

        self.robot_collision_dist = COLLISION_DIST  # 最小避障距离（碰撞阈值）
        self.robot_punish_dist = 1     # 距离惩罚阈值

        self.human_trajectory = []  # 存储人类轨迹的列表

        # 测试指标存储
        self.test_metrics = []  # 每个元素为一个回合的指标字典
        self.current_episode_metrics = {}  # 当前回合的指标
        self.episode_start_time = 0.0  # 回合开始时间

        # 结果文件保存路径（当前目录下的results文件夹）
        self.results_dir = os.path.join(os.path.dirname(__file__), "test_results")
        os.makedirs(self.results_dir, exist_ok=True)  # 确保文件夹存在

        self.max_steps_per_episode = 200  # 每个episode的最大允许步数（超时阈值）
        self.current_step_count = 0  # 当前episode的已走步数
        self.episode_timeout = False  # 是否因超时结束的标志

        self.human_trajectory_pub = rospy.Publisher(
            "/human_trajectory",  # 人类轨迹话题名
            Marker,
            queue_size=10
        )

        self.center_marker_pub = rospy.Publisher(
            "/center_collision_markers",  # RViz中订阅的话题名
            MarkerArray, 
            queue_size=10
        )
        
        self.robot_marker_pub = rospy.Publisher(
            "/robot_collision_markers",  # RViz中订阅的话题名
            MarkerArray, 
            queue_size=10
        )

        self.human_marker_pub = rospy.Publisher(
            "/human_marker",
            Marker,
            queue_size=10
        )
        self.obstacle_cloud_pub = rospy.Publisher(
            "/obstacle_cloud",
            PointCloud2,
            queue_size=10
        )
        # 新增：发布带位置姿态的重置消息
        self.odom_reset_pub = rospy.Publisher(
            "/odometry_reset_pose",  # 新话题名
            Pose, 
            queue_size=1
        )
        # 新增：初始化重置信号发布器
        self.reset_pub = rospy.Publisher(
            '/go2_gazebo/reset',
            Empty,
            queue_size=10
        )
        # 新增：人类位置订阅器
        self.human_pos_sub = rospy.Subscriber(
            '/actor_pos', 
            PointStamped, 
            self.human_pos_callback)
        



        rospy.sleep(1.0)  # 等待发布器初始化


    def human_pos_callback(self, msg):
        self.human_pos = msg.point
        # 将当前位置添加到轨迹列表
        self.human_trajectory.append((msg.point.x, msg.point.y, msg.point.z))
        
        # 发布轨迹可视化标记
        trajectory_marker = Marker()
        trajectory_marker.header.frame_id = "odom"  # 与人类位置同坐标系
        trajectory_marker.header.stamp = rospy.Time.now()
        trajectory_marker.id = 1
        trajectory_marker.type = Marker.LINE_STRIP  # 线串类型，连接所有点
        trajectory_marker.action = Marker.ADD
        trajectory_marker.scale.x = 0.05  # 线宽
        trajectory_marker.color.r = 0.0
        trajectory_marker.color.g = 1.0
        trajectory_marker.color.b = 0.0  # 绿色轨迹
        trajectory_marker.color.a = 1.0
        
        # 填充轨迹点
        for (x, y, z) in self.human_trajectory:
            point = geometry_msgs.msg.Point()
            point.x = x
            point.y = y
            point.z = z  # 可设为0.1，避免与地面重叠
            trajectory_marker.points.append(point)
        
        self.human_trajectory_pub.publish(trajectory_marker)


    def get_human_real_position(self, odom_x, odom_y, angle):
        if self.human_pos is None:
            print("Human position not available.")
            return self.get_human_position(self.odom_x, self.odom_y, self.odom_yaw)
        else:
            # 计算理想跟随坐标（基于机器人当前位置和朝向）
            ideal_x, ideal_y = self.get_human_position(odom_x, odom_y, angle)
            
            # 获取实际订阅的跟随坐标
            actual_x, actual_y = self.human_pos.x, self.human_pos.y
            
            # 打印两种坐标进行对比（保留4位小数提高精度）
            # print(f"理想跟随坐标: x={ideal_x:.4f}, y={ideal_y:.4f}")
            # print(f"实际跟随坐标: x={actual_x:.4f}, y={actual_y:.4f}")
            # print(f"坐标误差: dx={actual_x-ideal_x:.4f}, dy={actual_y-ideal_y:.4f}")
            return self.human_pos.x, self.human_pos.y

    def get_human_position(self, odom_x, odom_y, angle):
        distance = 0.5  # 假设人类距离机器人为0.6米
        human_relative_angle =  angle - 3 * np.pi / 4
        human_x = odom_x + distance * np.cos(human_relative_angle)
        human_y = odom_y + distance * np.sin(human_relative_angle)
        return human_x, human_y
    
    def get_center_position(self, odom_x, odom_y, robot_angle):
        local_x = self.center_offset_x
        local_y = self.center_offset_y
        center_x = odom_x + local_x * math.cos(robot_angle) - local_y * math.sin(robot_angle)
        center_y = odom_y + local_x * math.sin(robot_angle) + local_y * math.cos(robot_angle)
        return center_x, center_y
    
    def publish_center_markers(self, center_x, center_y):
        """发布中心点及碰撞区域的可视化标记"""
        marker_array = MarkerArray()
        
        # 1. 中心点标记（红色小球，半径0.05m）
        center_marker = Marker()
        center_marker.header.frame_id = "odom"  # 与机器人位置同坐标系
        center_marker.header.stamp = rospy.Time.now()
        center_marker.id = 0  # 唯一ID
        center_marker.type = Marker.SPHERE
        center_marker.action = Marker.ADD
        center_marker.scale.x = 0.1  # 直径0.1m
        center_marker.scale.y = 0.1
        center_marker.scale.z = 0.1
        center_marker.color.r = 1.0  # 红色
        center_marker.color.g = 0.0
        center_marker.color.b = 0.0
        center_marker.color.a = 1.0  # 不透明
        center_marker.pose.position.x = center_x
        center_marker.pose.position.y = center_y
        center_marker.pose.position.z = 0.1  # 略微抬高，避免与地面重叠
        marker_array.markers.append(center_marker)
        
        # 2. 最小避障区域（红色半透明圆柱体，半径=碰撞阈值）
        collision_zone = Marker()
        collision_zone.header.frame_id = "odom"
        collision_zone.header.stamp = rospy.Time.now()
        collision_zone.id = 1
        collision_zone.type = Marker.CYLINDER
        collision_zone.action = Marker.ADD
        collision_zone.scale.x = 2 * self.center_collision_dist  # 直径=2×半径
        collision_zone.scale.y = 2 * self.center_collision_dist
        collision_zone.scale.z = 0.05  # 高度0.05m（薄圆盘）
        collision_zone.color.r = 1.0  # 红色
        collision_zone.color.g = 0.0
        collision_zone.color.b = 0.0
        collision_zone.color.a = 0.3  # 半透明
        collision_zone.pose.position.x = center_x
        collision_zone.pose.position.y = center_y
        collision_zone.pose.position.z = 0.025  # 贴地显示
        marker_array.markers.append(collision_zone)
        
        # 3. 距离惩罚区域（黄色半透明圆柱体，半径=惩罚阈值）
        punish_zone = Marker()
        punish_zone.header.frame_id = "odom"
        punish_zone.header.stamp = rospy.Time.now()
        punish_zone.id = 2
        punish_zone.type = Marker.CYLINDER
        punish_zone.action = Marker.ADD
        punish_zone.scale.x = 2 * self.center_punish_dist
        punish_zone.scale.y = 2 * self.center_punish_dist
        punish_zone.scale.z = 0.05
        punish_zone.color.r = 1.0  # 黄色
        punish_zone.color.g = 1.0
        punish_zone.color.b = 0.0
        punish_zone.color.a = 0.2  # 更透明reward
        punish_zone.pose.position.x = center_x
        punish_zone.pose.position.y = center_y
        punish_zone.pose.position.z = 0.025
        marker_array.markers.append(punish_zone)
        
        # 发布标记
        self.center_marker_pub.publish(marker_array)


    def publish_robot_markers(self, odom_x, odom_y):
        """发布以雷达为中心的碰撞区域可视化标记"""
        marker_array = MarkerArray()
        
        # 1. 雷达中心标记（蓝色小球，标识雷达位置）
        radar_center = Marker()
        radar_center.header.frame_id = "base_laser"  # 雷达坐标系（需与实际雷达frame一致）
        radar_center.header.stamp = rospy.Time.now()
        radar_center.id = 10  # 唯一ID，避免与其他标记冲突
        radar_center.type = Marker.SPHERE
        radar_center.action = Marker.ADD
        radar_center.scale.x = 0.1  # 直径0.1m
        radar_center.scale.y = 0.1
        radar_center.scale.z = 0.1
        radar_center.color.r = 0.0  # 蓝色
        radar_center.color.g = 0.0
        radar_center.color.b = 1.0
        radar_center.color.a = 1.0  # 不透明
        # 雷达中心在自身坐标系原点（0,0,0）
        radar_center.pose.position.x = odom_x
        radar_center.pose.position.y = odom_y
        radar_center.pose.position.z = 0.1
        marker_array.markers.append(radar_center)
        
        # 2. 碰撞危险区域（红色半透明圆柱体，半径=COLLISION_DIST）
        collision_zone = Marker()
        collision_zone.header.frame_id = "base_laser"
        collision_zone.header.stamp = rospy.Time.now()
        collision_zone.id = 11
        collision_zone.type = Marker.CYLINDER
        collision_zone.action = Marker.ADD
        collision_zone.scale.x = 2 * COLLISION_DIST  # 直径=2×半径
        collision_zone.scale.y = 2 * COLLISION_DIST
        collision_zone.scale.z = 0.1  # 高度0.1m（突出显示）
        collision_zone.color.r = 1.0  # 红色
        collision_zone.color.g = 0.0
        collision_zone.color.b = 0.0
        collision_zone.color.a = 0.3  # 半透明
        # 区域中心与雷达中心重合
        collision_zone.pose.position.x = odom_x
        collision_zone.pose.position.y = odom_y
        collision_zone.pose.position.z = 0.05  # 半高位置，避免贴地
        marker_array.markers.append(collision_zone)
        
        # 3. 预警区域（黄色半透明圆柱体，半径=1.5×碰撞半径，可选扩展）
        warning_zone = Marker()
        warning_zone.header.frame_id = "base_laser"
        warning_zone.header.stamp = rospy.Time.now()
        warning_zone.id = 12
        warning_zone.type = Marker.CYLINDER
        warning_zone.action = Marker.ADD
        warning_zone.scale.x = 2 * COLLISION_DIST * 1.5  # 预警范围稍大
        warning_zone.scale.y = 2 * COLLISION_DIST * 1.5
        warning_zone.scale.z = 0.05  # 比碰撞区域矮，区分层级
        warning_zone.color.r = 1.0  # 黄色
        warning_zone.color.g = 1.0
        warning_zone.color.b = 0.0
        warning_zone.color.a = 0.2  # 更透明
        warning_zone.pose.position.x = odom_x
        warning_zone.pose.position.y = odom_y
        warning_zone.pose.position.z = 0.025  # 贴地显示
        marker_array.markers.append(warning_zone)
        
        # 发布标记数组（需在类中定义publisher，如self.robot_marker_pub）
        self.robot_marker_pub.publish(marker_array)

    def trans_to_center(self, odom_x, odom_y, robot_angle):
        o_points = []  # 原始点云

        # 获取o点坐标
        center_x, center_y = self.get_center_position(odom_x, odom_y, robot_angle)

        # 从激光点云中提取所有点
        data = list(pc2.read_points(self.last_velodyne, skip_nans=True, field_names=("x", "y", "z")))

        for (x_laser, y_laser, z_laser) in data:
            if z_laser < 0.1:
                continue
            # 旋转：
            x_rot = x_laser * math.cos(robot_angle) - y_laser * math.sin(robot_angle)
            y_rot = x_laser * math.sin(robot_angle) + y_laser * math.cos(robot_angle)
            #　平移：先平移到base_link坐标系，再平移到全局坐标
            x_world = odom_x + x_rot + self.laser_offset_x
            y_world = odom_y + y_rot + self.laser_offset_y
            # 计算o点到激光点的距离
            x_rel_o = x_world - center_x
            y_rel_o = y_world - center_y
            dist_to_o = np.sqrt(x_rel_o**2 + y_rel_o**2)
            angle_to_o = np.arctan2(y_rel_o, x_rel_o)
            o_points.append((dist_to_o, angle_to_o, x_rel_o, y_rel_o, x_world, y_world))
        min_center_laser = min([point[0] for point in o_points]) if o_points else float('inf')
        if min_center_laser < CENTER_COLLISION_DIST:
            return True, True, min_center_laser
        return False, False, min_center_laser

    # def center_collision(self, min_center_laser):
    #     if min_center_laser < CENTER_COLLISION_DIST:
    #         return True, True, min_center_laser
    #     else:
    #         return False, False, min_center_laser


    # Read velodyne pointcloud and turn it into distance data, then select the minimum value for each angle
    # range as state representation
    def model_state_callback(self, msg):
        try:
            idx = msg.name.index("go2")
        except ValueError:
            print("Model not found")
            return
        true_position = msg.pose[idx].position
        self.true_x = true_position.x
        self.true_y = true_position.y
        true_orientaion = msg.pose[idx].orientation
        true_quat = Quaternion(
            w=true_orientaion.w,
            x=true_orientaion.x,
            y=true_orientaion.y,
            z=true_orientaion.z,
        )
        euler = true_quat.to_euler(degrees=False)
        self.true_yaw = euler[2]
        # print(f"Model true position updated: x={self.true_x:.2f}, y={self.true_y:.2f}, yaw={self.true_yaw:.2f}")


    def velodyne_callback(self, v):
        self.last_velodyne = v
        data = list(pc2.read_points(v, skip_nans=False, field_names=("x", "y", "z")))
        self.velodyne_data = np.ones(self.environment_dim) * 10
        ground_projection = []
        for i in range(len(data)):
            x, y, z = data[i]
            if 0.3 < z < 0.8:
                ground_projection.append([x, y, 0.0])
            # if data[i][2] > -0.2:
            if data[i][2] > 0.05:
                dot = data[i][0] * 1 + data[i][1] * 0
                mag1 = math.sqrt(math.pow(data[i][0], 2) + math.pow(data[i][1], 2))
                mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
                beta = math.acos(dot / (mag1 * mag2)) * np.sign(data[i][1])
                dist = math.sqrt(data[i][0] ** 2 + data[i][1] ** 2 + data[i][2] ** 2)

                for j in range(len(self.gaps)):
                    if self.gaps[j][0] <= beta < self.gaps[j][1]:
                        self.velodyne_data[j] = min(self.velodyne_data[j], dist)
                        break
        for (x, y, _) in ground_projection:
            dist_ground = math.sqrt(x ** 2 + y ** 2)
            beta = math.atan2(y, x)
            for j in range(len(self.gaps)):
                if self.gaps[j][0] <= beta < self.gaps[j][1]:
                    self.velodyne_data[j] = min(self.velodyne_data[j], dist_ground)
                    break
        

    def odom_callback(self, od_data):
        self.last_odom = od_data
        # 打印日志
        # print("Odometry updated: x={:.2f}, y={:.2f}".format(
        #     od_data.pose.pose.position.x, od_data.pose.pose.position.y))
        
    # def ekf_callback(self, msg):
    #     self.last_ekf_pose = msg
    #     # 提取位姿（路径：msg.pose.pose.position，而非msg.pose.pose.position）
    #     x = msg.pose.pose.position.x
    #     y = msg.pose.pose.position.y
    #     rospy.loginfo(f"EKF数据接收：x={x:.4f}, y={y:.4f}")  # 此时会正常打印日志

        

    # Perform an action and read a new state
    def step(self, action):
        target = False

        # Publish the robot action
        vel_cmd = Twist()
        vel_cmd.linear.x = action[0]
        vel_cmd.angular.z = action[1]
        self.vel_pub.publish(vel_cmd)
        self.publish_markers(action)

        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        # propagate state for TIME_DELTA seconds
        time.sleep(TIME_DELTA)

        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            pass
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")

        # read velodyne laser state
        done, collision, min_laser = self.observe_collision(self.velodyne_data)
        v_state = []
        v_state[:] = self.velodyne_data[:]
        laser_state = [v_state]

        # Calculate robot heading from odometry data
        self.odom_x = self.last_odom.pose.pose.position.x
        self.odom_y = self.last_odom.pose.pose.position.y
        quaternion = Quaternion(
            self.last_odom.pose.pose.orientation.w,
            self.last_odom.pose.pose.orientation.x,
            self.last_odom.pose.pose.orientation.y,
            self.last_odom.pose.pose.orientation.z,
        )
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)

        # Calculate distance to the goal from the robot
        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )
        # 新增：检测人类是否碰撞障碍物
        human_x, human_y, human_collision, min_human_dist = self.check_human_obstacle_collision(self.odom_x, self.odom_y, angle)
        human_angle = angle
        human_x, human_y = self.get_human_position(self.odom_x, self.odom_y, angle)
        self.publish_human_marker(human_x, human_y)
        if human_collision:
            done =True
            self.publish_obstacle_cloud(human_x, human_y)
        center_done, center_collision, min_center_laser = self.trans_to_center(self.odom_x, self.odom_y, angle)
        # if center_done:
        #     done = True
        center_x, center_y = self.get_center_position(self.odom_x, self.odom_y, robot_angle=angle)
        self.publish_center_markers(center_x, center_y)
        self.publish_robot_markers(self.odom_x, self.odom_y)

        # 新增里程计和真值对比 todo
        # print("True position: ", self.true_x, self.true_y)
        # print("Odom position: ", self.odom_x, self.odom_y)
        # if abs(self.true_x) < 0.01 and abs(self.true_y) < 0.01:
        #     print(f"[Step {self.current_step_count}] 警告：Gazebo真值未更新，暂不计算偏差。")
        #     distance_odom_to_model = -1.0
        # else:
        #     distance_odom_to_model = np.linalg.norm(
        #         [self.odom_x - self.true_x, self.odom_y - self.true_y]
        #     )
        #     if self.current_step_count % 1 == 0:  # 每1步打印一次
        #         print(f"[Step {self.current_step_count}] odom与模型位置误差: {distance_odom_to_model:.4f} m")
        


        # 新增：计算当前线速度和角速度（从action中获取，action[0]是线速度，action[1]是角速度）
        current_linear_vel = action[0]
        current_angular_vel = action[1]
        
        # 计算线加速度（速度变化率）和角加速度
        linear_acc = (current_linear_vel - self.prev_linear_vel) / self.TIME_DELTA
        angular_acc = (current_angular_vel - self.prev_angular_vel) / self.TIME_DELTA
        
        # 计算线加加速度（jerk，加速度变化率）和角加加速度
        linear_jerk = (linear_acc - self.prev_linear_acc) / self.TIME_DELTA
        angular_jerk = (angular_acc - self.prev_angular_acc) / self.TIME_DELTA
        
        # 更新历史状态（为下一时刻计算做准备）
        self.prev_linear_vel = current_linear_vel
        self.prev_angular_vel = current_angular_vel
        self.prev_linear_acc = linear_acc
        self.prev_angular_acc = angular_acc


        # Calculate the relative angle between the robots heading and heading toward the goal
        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        # Detect if the goal has been reached and give a large positive reward
        if distance < GOAL_REACHED_DIST:
            target = True
            done = True

        robot_state = [distance, theta, action[0], action[1]]
        state = np.append(laser_state, robot_state)
        # print("laser_state: ", laser_state, " robot_state: ", robot_state, "\n", "state: ", state)
        robot_x = self.odom_x
        robot_y = self.odom_y
        robot_angle = angle

        # 更新当前episode步数
        self.current_step_count += 1
        
        # 超时判定：若超过最大步数且未完成，则标记超时
        if self.current_step_count >= self.max_steps_per_episode and not done:
            self.episode_timeout = True
            done = True  # 强制结束当前回合
            print(f"超时：当前回合步数超过{self.max_steps_per_episode}步")
        
        # 更新回合指标（包含超时信息）
        self.current_episode_metrics["steps"] = self.current_step_count
        self.current_episode_metrics["timeout"] = self.episode_timeout


        # reward = self.get_reward_1(self,target, collision, human_collision, action, min_laser, min_human_dist, human_x, human_y, human_angle, robot_x, robot_y, robot_angle)
        reward = self.get_reward(target, collision, human_collision, 
                                 action, min_laser, min_center_laser,
                                 linear_acc, angular_acc, linear_jerk, angular_jerk)

        # 计算当前回合已耗时
        current_time = time.time()
        self.current_episode_metrics["time_cost"] = current_time - self.episode_start_time
        
        # 计算机器狗运动轨迹长度（累加每步位移）
        current_robot_pos = (self.odom_x, self.odom_y)
        dx = current_robot_pos[0] - self.prev_robot_pos[0]
        dy = current_robot_pos[1] - self.prev_robot_pos[1]
        self.current_episode_metrics["path_length"] += np.sqrt(dx**2 + dy**2)
        self.prev_robot_pos = current_robot_pos  # 更新上一位置
        
        # 新增：记录每步的加速度
        self.current_episode_metrics["linear_acc_list"].append(abs(linear_acc))
        self.current_episode_metrics["angular_acc_list"].append(abs(angular_acc))

        # 记录每步的加加速度（用于计算流畅性）
        self.current_episode_metrics["linear_jerk_list"].append(abs(linear_jerk))
        self.current_episode_metrics["angular_jerk_list"].append(abs(angular_jerk))
        
        # 回合结束时（done=True）计算最终指标
        if done:
            # 标记是否成功（到达目标且无碰撞）
            self.current_episode_metrics["success"] = target and not collision and not human_collision
            # 标记是否碰撞
            self.current_episode_metrics["collision"] = collision
            self.current_episode_metrics["human_collision"] = human_collision
            # 计算加速度平均值
            if self.current_episode_metrics["linear_acc_list"]:
                self.current_episode_metrics["linear_acc_avg"] = np.mean(self.current_episode_metrics["linear_acc_list"])
                self.current_episode_metrics["angular_acc_avg"] = np.mean(self.current_episode_metrics["angular_acc_list"])
            # 计算加加速度平均值（越小越流畅）
            if self.current_episode_metrics["linear_jerk_list"]:
                self.current_episode_metrics["linear_jerk_avg"] = np.mean(np.abs(self.current_episode_metrics["linear_jerk_list"]))
                self.current_episode_metrics["angular_jerk_avg"] = np.mean(np.abs(self.current_episode_metrics["angular_jerk_list"]))
            # 将当前回合指标存入总列表
            self.test_metrics.append(self.current_episode_metrics)
            # 每100个回合输出一次统计结果（可根据需要调整）
            if len(self.test_metrics) % 100 == 0:
                self.print_test_summary()        

        return state, reward, done, target
    
    def print_test_summary(self):
        """打印100次回合的指标汇总"""
        total = len(self.test_metrics)
        if total == 0:
            return
        
        # 计算无碰撞率（无任何碰撞的成功回合比例）
        success_count = sum(1 for m in self.test_metrics if m["success"])
        collision_free_rate = success_count / total * 100
        
        # 平均运动时间
        avg_time = np.mean([m["time_cost"] for m in self.test_metrics])
        
        # 平均轨迹长度（效率：越短越优，相同目标下）
        avg_path_length = np.mean([m["path_length"] for m in self.test_metrics])

        # 平均加速度（流畅性：越小越优）
        avg_linear_acc = np.mean([m["linear_acc_avg"] for m in self.test_metrics])
        avg_angular_acc = np.mean([m["angular_acc_avg"] for m in self.test_metrics])
        
        # 平均加加速度（流畅性：越小越优）
        avg_linear_jerk = np.mean([m["linear_jerk_avg"] for m in self.test_metrics])
        avg_angular_jerk = np.mean([m["angular_jerk_avg"] for m in self.test_metrics])
        
        # 打印汇总结果
        print("\n" + "="*50)
        print(f"测试汇总（{total}次回合）")
        print(f"无碰撞成功率：{collision_free_rate:.2f}%")
        print(f"平均运动时间：{avg_time:.2f}秒")
        print(f"平均轨迹长度：{avg_path_length:.2f}米")
        print(f"平均加速度（流畅性）：{avg_linear_acc:.4f}")
        print(f"平均角加速度（流畅性）：{avg_angular_acc:.4f}")
        print(f"平均线加加速度（流畅性）：{avg_linear_jerk:.4f}")
        print(f"平均角加加速度（流畅性）：{avg_angular_jerk:.4f}")
        print("="*50 + "\n")        
        # 新增：100次回合完成后保存结果文件
        if len(self.test_metrics) % 100 == 0:
            self.save_test_results()

    def save_test_results(self):
        """将100次回合的测试结果保存为JSON文件"""
        # 生成带时间戳的文件名（避免重复）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        file_path = os.path.join(self.results_dir, f"test_results_{timestamp}.json")
        
        # 转换numpy类型为Python原生类型（避免JSON序列化错误）
        serializable_metrics = []
        for metric in self.test_metrics:
            serializable = {
                "episode_id": metric["episode_id"],
                "collision": metric["collision"],
                "human_collision": metric["human_collision"],
                "success": metric["success"],
                "time_cost": float(metric["time_cost"]),  # 转换numpy float为Python float
                "path_length": float(metric["path_length"]),
                "linear_jerk_avg": float(metric["linear_jerk_avg"]),
                "angular_jerk_avg": float(metric["angular_jerk_avg"]),
                "linear_acc_avg": float(metric["linear_acc_avg"]),
                "angular_acc_avg": float(metric["angular_acc_avg"]),
                "steps": metric["steps"],
                "timeout": metric["timeout"]
            }
            serializable_metrics.append(serializable)
        
        # 写入JSON文件
        with open(file_path, "w") as f:
            json.dump(serializable_metrics, f, indent=4)  # indent=4：格式化输出，便于阅读
        
        print(f"\n测试结果已保存至：{file_path}")


    def publish_human_marker(self, human_x, human_y):
            """发布人类位置标记（蓝色球体，半径0.3m）"""
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.scale.x = 0.3  # 人类半径
            marker.scale.y = 0.3
            marker.scale.z = 1.5  # 人类高度（从脚底到头顶）
            marker.color.a = 0.8  # 透明度
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0  # 蓝色
            marker.pose.position.x = human_x
            marker.pose.position.y = human_y
            marker.pose.position.z = 0.75  # 球体中心在腰部高度
            self.human_marker_pub.publish(marker)

    def publish_obstacle_cloud(self, human_x, human_y):
        """发布人类周围的障碍物点云（红色，仅碰撞时发布）"""
        header = Header()
        header.frame_id = "odom"
        header.stamp = rospy.Time.now()
        
        cloud_msg = pc2.create_cloud_xyz32(
            header=header,
            points=[(x_world, y_world, 0.5) for x_world, y_world in self.obstacle_points]
        )
        self.obstacle_cloud_pub.publish(cloud_msg)


# ######################## reset ############################
 
    def reset(self):
        print("Resetting environment...")
        self.reset_count += 1

        if (self.reset_count % 10) == 0:
            print(f"Reset count: {self.reset_count}")

        
        # 重置前先暂停物理引擎
        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except rospy.ServiceException as e:
            print("/gazebo/pause_physics service call failed")

        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service("/gazebo/reset_world")
        try:
            self.reset_proxy()

        except rospy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")

        # 新增：发送重置信号到机器人控制系统
        reset_msg = Empty()
        self.reset_pub.publish(reset_msg)
        # rospy.loginfo("Sent reset command to robot controller")
        rospy.sleep(1.0)  # 等待机器人重置完成

        angle = np.random.uniform(-np.pi, np.pi)
        quaternion = Quaternion.from_euler(0.0, 0.0, angle)
        object_state = self.set_self_state

        x = 0
        y = 0
        position_ok = False
        while not position_ok:
            x = np.random.uniform(-4.5, 4.5)
            y = np.random.uniform(-4.5, 4.5)
            robot_position_ok = check_pos(x, y)
            # print(f"[Reset] Set robot position: x={x:.3f}, y={y:.3f}")
            human_x, human_y = self.get_human_position(x, y, angle)
            # human_x, human_y = self.get_human_real_position(x, y, angle)
            # print(f"[Reset] Set human position: x={human_x:.3f}, y={human_y:.3f}")
            human_position_ok = check_pos(human_x, human_y)     
            position_ok = robot_position_ok and human_position_ok 
            # print(f"[Reset] Robot position ok: {robot_position_ok}, Human position ok: {human_position_ok}")

        # print(f"[Reset] Set robot position: x={x:.3f}, y={y:.3f}")
        object_state.pose.position.x = x
        object_state.pose.position.y = y
        object_state.pose.position.z = 0.25
        object_state.pose.orientation.x = quaternion.x
        object_state.pose.orientation.y = quaternion.y
        object_state.pose.orientation.z = quaternion.z
        object_state.pose.orientation.w = quaternion.w
        self.set_state.publish(object_state)


        # 构造并发布Pose消息给里程计
        reset_pose = Pose()
        # 复制位置
        reset_pose.position.x = object_state.pose.position.x  # x
        reset_pose.position.y = object_state.pose.position.y  # y
        reset_pose.position.z = object_state.pose.position.z  # z
        # 复制姿态（四元数）
        reset_pose.orientation.x = object_state.pose.orientation.x
        reset_pose.orientation.y = object_state.pose.orientation.y
        reset_pose.orientation.z = object_state.pose.orientation.z
        reset_pose.orientation.w = object_state.pose.orientation.w
        self.odom_reset_pub.publish(reset_pose)  # 发布到新话题
        rospy.sleep(0.1)  # 等待里程计接收并处理        

        self.odom_x = object_state.pose.position.x
        self.odom_y = object_state.pose.position.y



        # set a random goal in empty space in environment
        self.change_goal()
        # randomly scatter boxes in the environment
        self.random_box()
        self.publish_markers([0.0, 0.0])

        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print("/gazebo/unpause_physics service call failed")

        time.sleep(TIME_DELTA)

        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print("/gazebo/pause_physics service call failed")
        v_state = []
        v_state[:] = self.velodyne_data[:]
        laser_state = [v_state]

        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )

        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y

        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))

        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle

        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        robot_state = [distance, theta, 0.0, 0.0]
        state = np.append(laser_state, robot_state)
        print("Environment reset completed.")
            
        # 重置超时相关计数
        self.current_step_count = 0  # 重置步数计数
        self.episode_timeout = False  # 重置超时标记
        # 初始化当前回合指标
        self.current_episode_metrics = {
            "episode_id": self.reset_count,  # 回合ID
            "collision": False,  # 是否碰撞
            "human_collision": False,  # 是否碰撞人类
            "success": False,  # 是否成功到达目标
            "time_cost": 0.0,  # 回合耗时
            "path_length": 0.0,  # 机器狗运动轨迹长度
            "linear_acc_avg": 0,  # 每步线加速度
            "angular_acc_avg": 0,  # 每步角加速度
            "linear_acc_list": [],  # 每步加加速度
            "angular_acc_list": [],  # 每步角加加速度
            "linear_jerk_avg": 0.0,  # 线加加速度平均值（衡量流畅性）
            "angular_jerk_avg": 0.0,  # 角加加速度平均值
            "linear_jerk_list": [],  # 每步线加加速度
            "angular_jerk_list": [],  # 每步角加加速度
            "steps": 0,  # 步数
            "timeout": False  # 超时标记
        }
        # 记录回合开始时间
        self.episode_start_time = time.time()
        # 重置轨迹长度计算（记录上一位置）
        self.prev_robot_pos = (self.odom_x, self.odom_y)
        return state
            

# ######################## reset ############################

    def change_goal(self):
        # Place a new goal and check if its location is not on one of the obstacles
        if self.upper < 9:
            self.upper += 0.004
        if self.lower > -9:
            self.lower -= 0.004

        goal_ok = False

        while not goal_ok:
            self.goal_x = self.odom_x + random.uniform(self.upper, self.lower)
            self.goal_y = self.odom_y + random.uniform(self.upper, self.lower)
            goal_ok = check_pos(self.goal_x, self.goal_y)

    def random_box(self):
        # Randomly change the location of the boxes in the environment on each reset to randomize the training
        # environment
        for i in range(2):
            name = "cardboard_box_" + str(i)

            x = 0
            y = 0
            box_ok = False
            while not box_ok:
                x = np.random.uniform(-6, 6)
                y = np.random.uniform(-6, 6)
                box_ok = check_pos(x, y)
                distance_to_robot = np.linalg.norm([x - self.odom_x, y - self.odom_y])
                distance_to_goal = np.linalg.norm([x - self.goal_x, y - self.goal_y])
                if distance_to_robot < 1.5 or distance_to_goal < 1.5:
                    box_ok = False
            box_state = ModelState()
            box_state.model_name = name
            box_state.pose.position.x = x
            box_state.pose.position.y = y
            box_state.pose.position.z = 0.0
            box_state.pose.orientation.x = 0.0
            box_state.pose.orientation.y = 0.0
            box_state.pose.orientation.z = 0.0
            box_state.pose.orientation.w = 1.0
            self.set_state.publish(box_state)

    def publish_markers(self, action):
        # Publish visual data in Rviz
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.goal_x
        marker.pose.position.y = self.goal_y
        marker.pose.position.z = 0

        markerArray.markers.append(marker)

        self.publisher.publish(markerArray)

        markerArray2 = MarkerArray()
        marker2 = Marker()
        marker2.header.frame_id = "odom"
        marker2.type = marker.CUBE
        marker2.action = marker.ADD
        marker2.scale.x = abs(action[0])
        marker2.scale.y = 0.1
        marker2.scale.z = 0.01
        marker2.color.a = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 0.0
        marker2.pose.orientation.w = 1.0
        marker2.pose.position.x = 5
        marker2.pose.position.y = 0
        marker2.pose.position.z = 0

        markerArray2.markers.append(marker2)
        self.publisher2.publish(markerArray2)

        markerArray3 = MarkerArray()
        marker3 = Marker()
        marker3.header.frame_id = "odom"
        marker3.type = marker.CUBE
        marker3.action = marker.ADD
        marker3.scale.x = abs(action[1])
        marker3.scale.y = 0.1
        marker3.scale.z = 0.01
        marker3.color.a = 1.0
        marker3.color.r = 1.0
        marker3.color.g = 0.0
        marker3.color.b = 0.0
        marker3.pose.orientation.w = 1.0
        marker3.pose.position.x = 5
        marker3.pose.position.y = 0.2
        marker3.pose.position.z = 0

        markerArray3.markers.append(marker3)
        self.publisher3.publish(markerArray3)

    # 检查人与障碍物之间的碰撞
    def check_human_obstacle_collision(self, odom_x, odom_y, angle):
        self.obstacle_points = []  # 存储碰撞点的列表
        human_x, human_y = self.get_human_position(odom_x, odom_y, angle)
        # human_x, human_y = self.get_human_real_position(odom_x, odom_y, angle)
        min_dist = float('inf')  # 初始化最小距离为无穷大
        # 从激光点云中提取所有点（x,y坐标）
        data = list(pc2.read_points(self.last_velodyne, skip_nans=True, field_names=("x", "y", "z")))
        # 检查是否有障碍物点距离人类小于避障半径
        for (x_laser, y_laser, z_laser) in data:
            # 过滤地面点（仅保留z>0.1的点，可根据人类高度调整）
            if z_laser < 0.1:  
                continue
            
            # 1. 旋转：转换为机器人base_link坐标系
            x_rot = x_laser * math.cos(angle) - y_laser * math.sin(angle)
            y_rot = x_laser * math.sin(angle) + y_laser * math.cos(angle)
            
            # 2. 平移：转换为世界坐标系（odom系）
            x_world = odom_x + x_rot
            y_world = odom_y + y_rot
            
            # 计算距离
            dx = x_world - human_x
            dy = y_world - human_y
            dist = math.sqrt(dx**2 + dy**2)
            if dist < min_dist:
                min_dist = dist
            
            if dist < self.human_collision_dist:
                print(f"obstacle x: {x_world:.4f}, y: {y_world:.4f}, dist: {dist:.4f}")
                print(f"human x: {human_x:.4f}, y: {human_y:.4f}")
                print(f"robot x: {odom_x:.4f}, y: {odom_y:.4f}")
                return human_x, human_y, True, dist  # 人类碰撞障碍物
        return human_x, human_y, False, min_dist  # 无碰撞

    # def get_human_front_obstacle(self, human_x, human_y, human_angle, robot_x, robot_y, robot_angle ,laser_data):
    #     min_front_dist = float('inf')
    #     max_score = 0
    #     front_angle = 0
    #     # 从激光点云中提取所有点（x,y坐标）
    #     data = list(pc2.read_points(laser_data, skip_nans=True, field_names=("x", "y", "z")))
    #     # 检查是否有障碍物点距离人类小于避障半径
    #     for (x_laser, y_laser, z_laser) in data:
    #         # 过滤地面点（仅保留z>0.1的点，可根据人类高度调整）
    #         if z_laser < 0.1:
    #             continue

    #         # 1.机器狗坐标系->世界坐标系
    #         # 先旋转
    #         x_robot_rot = x_laser * math.cos(robot_angle) - y_laser * math.sin(robot_angle)
    #         y_robot_rot = x_laser * math.sin(robot_angle) + y_laser * math.cos(robot_angle)
    #         x_world = robot_x + x_robot_rot  # 激光点在世界坐标系的x
    #         y_world = robot_y + y_robot_rot  # 激光点在世界坐标系的y

    #         # 计算距离
    #         dx = x_world - human_x
    #         dy = y_world - human_y
    #         dist = math.sqrt(dx**2 + dy**2)
    #         if dist > self.human_front_safety_dist:
    #             continue
    #         obstacle_angle = math.atan2(dy, dx) - human_angle
    #         if obstacle_angle > np.pi:
    #             obstacle_angle -= 2 * np.pi
    #         elif obstacle_angle < -np.pi:
    #             obstacle_angle += 2 * np.pi


    #         if (obstacle_angle < 0 and obstacle_angle > -self.human_left_angle_range) or \
    #         (obstacle_angle >= 0 and obstacle_angle < self.human_right_angle_range):
    #             if obstacle_angle < 0:  # 左侧障碍物
    #                 max_angle = self.human_left_angle_range
    #             else:  # 右侧障碍物
    #                 max_angle = self.human_right_angle_range
                
    #             dist_factor = 1 - dist / self.human_front_safety_dist
        
    #             # 5. 计算角度因子：角度越接近正前方，因子越大（1→0），且不小于0
    #             angle_factor = max(0, 1 - abs(obstacle_angle) / max_angle)
                
    #             # 6. 计算综合得分：距离因子×角度因子（越大越需要关注）
    #             score = dist_factor * angle_factor
    #             if score > max_score:
    #                 max_score = score
    #                 min_front_dist = dist
    #                 front_angle = obstacle_angle
    #                 print(f"front obstacle x: {x_world:.4f}, y: {y_world:.4f}, dist: {dist:.4f}, angle: {obstacle_angle:.4f}")
        
    #     return min_front_dist, front_angle, max_score

    @staticmethod
    def observe_collision(laser_data):
        # Detect a collision from laser data
        min_laser = min(laser_data)
        if min_laser < COLLISION_DIST:
            return True, True, min_laser
        return False, False, min_laser

  

    def get_reward(self, target, collision, human_collision, action, min_laser, min_center_laser, linear_acc, angular_acc, linear_jerk, angular_jerk):
        if target:
            self.target_count += 1
            print("***********Target reached! target count:",self.target_count, "***********")
            return 100.0
        elif collision:
            self.collision_count += 1
            print("***********Collision detected! collision count:", self.collision_count, "***********")
            return -100.0
        elif human_collision:
            self.collision_count += 1
            print("***********Human collision detected! collision count:", self.collision_count,"***********")
            return -100.0
        else:
            r3 = lambda x: 1 - x if x < 1 else 0.0
            r4 = lambda x: 1.5 - x if x < 1.5 else 0.0
            
            # 计算人机交互奖励
            # 运动平稳性惩罚项
            acc_weight = 0.1  # 加速度惩罚权重
            jerk_weight = 0.2  # 加加速度惩罚权重
            
            # 线加速度惩罚（平方项，鼓励小加速度）
            linear_acc_penalty = acc_weight * (linear_acc ** 2)
            # 角加速度惩罚
            angular_acc_penalty = acc_weight * (angular_acc ** 2)
            
            # 线加加速度惩罚（平方项，鼓励平滑的加速度变化）
            linear_jerk_penalty = jerk_weight * (linear_jerk ** 2)
            # 角加加速度惩罚
            angular_jerk_penalty = jerk_weight * (angular_jerk ** 2)
            # 总惩罚项
            total_penalty = linear_acc_penalty + angular_acc_penalty + linear_jerk_penalty + angular_jerk_penalty

            # 惩罚长时间转向
            # if abs(action[0]) < 0.05:
            #     static_count += 1
            # else:
            #     static_count = 0

            # 目前是没考虑前方安全区
            reward = action[0] / 0.4 - abs(action[1]) / 1.5 - r3(min_laser) / 4 
            - r4(min_center_laser) / 4 
            # - total_penalty

            print("reward:", reward)
            # print("---------------------------------------")
            return reward