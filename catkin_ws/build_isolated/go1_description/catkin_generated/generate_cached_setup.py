# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/noetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/noetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/actor_pos_plugin;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/z1_description;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/velodyne_simulator;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/velodyne_gazebo_plugins;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/velodyne_description;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/unitree_move_base;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/unitree_motor_ctrl;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/unitree_legged_real;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/unitree_legged_sdk;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/unitree_legged_control;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/unitree_guide;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/unitree_gazebo;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/unitree_controller;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/unitree_legged_msgs;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/odom;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/multi_robot_scenario;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/laikago_description;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/human_plugin;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/human_description;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/h1_description;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/go2w_description;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/go2_description;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/go1_description;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/b2w_description;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/b2_description;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/b1_description;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/aliengo_description;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/aliengoZ1_description;/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/a1_description;/opt/ros/noetic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/devel_isolated/go1_description/env.sh')

output_filename = '/home/sanshiqi/project/DRL-robot-navigation/catkin_ws/build_isolated/go1_description/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
