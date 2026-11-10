from pathlib import Path
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, Shutdown
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def description(sim):
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('explore', default_value='true'),
        DeclareLaunchArgument('config', default_value=str(Path(share('rover_bringup'))/'config/rover_config.yaml')),
        OpaqueFunction(function=lambda context: start(context, sim)),
    ])


def start(context, sim):
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    config = LaunchConfiguration('config').perform(context)
    robot = yaml.safe_load(Path(config).read_text())['base_bridge']['ros__parameters']
    model_share = Path(share('rover_description'))
    model = xacro.process_file(str(model_share/'urdf/rover.urdf.xacro'), mappings={
        'wheel_radius': str(robot['wheel_radius']), 'track_width': str(robot['track_width'])}).toxml()
    clock = {'use_sim_time': sim}
    actions = [Node(package='robot_state_publisher', executable='robot_state_publisher',
                    parameters=[clock, {'robot_description': model}], output='screen')]
    if sim:
        args = ['gz','sim','-r'] + ([] if gui else ['-s','--headless-rendering']) + [str(model_share/'urdf/empty.sdf')]
        actions += [
            ExecuteProcess(cmd=args,output='screen',on_exit=lambda event, ctx: [] if ctx.is_shutdown else [Shutdown(reason='Gazebo exited')]),
            Node(package='ros_gz_sim', executable='create', arguments=['-world','rover_world','-topic','robot_description','-name','mapping_rover','-z',str(robot['wheel_radius']+0.005)], output='screen'),
            Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=[
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/cmd_vel_safe@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/wheel/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'],
                 parameters=[{'qos_overrides./scan.publisher.reliability':'best_effort'}], output='screen')]
    else:
        actions += [Node(package='rover_bringup', executable='base_bridge', name='base_bridge',parameters=[config],output='screen'),
                    Node(package='sllidar_ros2',executable='sllidar_node',name='sllidar_node',parameters=[config],output='screen')]
    actions += [
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', parameters=[config,clock],output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(share('slam_toolbox'))/'launch/online_async_launch.py')),
                                 launch_arguments={'slam_params_file':config,'use_sim_time':str(sim).lower(),'autostart':'true'}.items()),
        Node(package='rover_bringup', executable='safety_gate', name='safety_gate',parameters=[clock,{'require_base_status':not sim}],output='screen'),

    ]
    if LaunchConfiguration('explore').perform(context).lower() == 'true':
        actions.append(Node(package='rover_bringup',executable='frontier_explorer',parameters=[clock],output='screen'))
    params = str(Path(share('rover_bringup'))/'config/nav2_params.yaml')
    servers = [('nav2_controller','controller_server',[('cmd_vel','cmd_vel_raw')]),
               ('nav2_planner','planner_server',[]),('nav2_smoother','smoother_server',[]),
               ('nav2_behaviors','behavior_server',[('cmd_vel','cmd_vel_raw')]),
               ('nav2_bt_navigator','bt_navigator',[]),
               ('nav2_velocity_smoother','velocity_smoother',[('cmd_vel','cmd_vel_raw'),('cmd_vel_smoothed','cmd_vel')])]
    for package, executable, remaps in servers:
        actions.append(Node(package=package,executable=executable,name=executable,
                            parameters=[params,clock],remappings=remaps,output='screen'))
    actions.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',
                        parameters=[clock,{'autostart':True,'node_names':[s[1] for s in servers]}],output='screen'))
    if gui:
        actions.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(model_share/'rviz/rover.rviz')],parameters=[clock],output='screen'))
    return actions
