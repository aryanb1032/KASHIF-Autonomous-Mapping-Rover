from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    xacro=PathJoinSubstitution([FindPackageShare('rover_description'),'urdf','rover.urdf.xacro'])
    desc=ParameterValue(Command(['xacro ',xacro]),value_type=str)
    return LaunchDescription([
      Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'robot_description':desc}]),
      Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui'),
      Node(package='rviz2',executable='rviz2',arguments=['-f','base_link','-d',PathJoinSubstitution([FindPackageShare('rover_description'),'rviz','rover.rviz'])])
    ])
