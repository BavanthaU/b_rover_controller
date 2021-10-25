import os
from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
 
  return LaunchDescription([
    Node(package='b_rover_controller', executable='robot_controller',
      output='screen'),
    Node(package='b_rover_controller', executable='robot_estimator',
      output='screen'),
  ])
