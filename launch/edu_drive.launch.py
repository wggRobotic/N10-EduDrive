import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
   
    package_path = FindPackageShare('n10_edu_drive_ros2')
    parameter_file = PathJoinSubstitution([
      package_path,
      'parameter',
      'edu_drive.yaml'
    ])

    edu_drive = Node(
      package='n10_edu_drive_ros2',
      executable='n10_edu_drive_ros2_node',
      name='n10_edu_drive_ros2_node',
      parameters=[parameter_file],
      namespace=EnvironmentVariable('EDU_ROBOT_NAMESPACE', default_value="eduard"),
      output='screen'
    )  
    
    return LaunchDescription([
        edu_drive
    ])
