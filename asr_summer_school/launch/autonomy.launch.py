"""Explicit Nav2 velocity routing through a disabled-by-default gate."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share=get_package_share_directory('asr_summer_school')
    sim=LaunchConfiguration('use_sim_time')
    sensors=LaunchConfiguration('sensors')
    params=LaunchConfiguration('nav_params')
    actions=[DeclareLaunchArgument('nav_params',default_value=os.path.join(share,'config','autonomy_nav2.yaml')),DeclareLaunchArgument('use_sim_time',default_value='false'),
             DeclareLaunchArgument('sensors',default_value='false'),
             DeclareLaunchArgument('duration',default_value='180.0'),
             DeclareLaunchArgument('max_radius',default_value='2.0')]
    for package,file in [('turtlebot3_bringup','robot.launch.py'),('turtlebot3_perception','camera.launch.py'),('asr_summer_school','apriltag_corrected.launch.py')]:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package),'launch',file)),launch_arguments={'use_sim_time':sim}.items(),condition=IfCondition(sensors)))
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch','slam_toolbox.launch.py')),launch_arguments={'use_sim_time':sim}.items(),condition=IfCondition(sensors)))
    servers=[('nav2_controller','controller_server'),('nav2_planner','planner_server'),
             ('nav2_smoother','smoother_server'),('nav2_behaviors','behavior_server'),
             ('nav2_bt_navigator','bt_navigator'),('nav2_waypoint_follower','waypoint_follower')]
    for package,name in servers:
        actions.append(Node(package=package,executable=name,name=name,output='screen',
                            parameters=[params,{'use_sim_time':sim},
                            {'default_nav_to_pose_bt_xml':os.path.join(share,'config','autonomy_to_pose.xml'),
                             'default_nav_through_poses_bt_xml':os.path.join(share,'config','autonomy_through_poses.xml')} if name=='bt_navigator' else {}],
                            remappings=[('cmd_vel','/autonomy/cmd_vel')]))
    actions.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
                        name='lifecycle_manager_autonomy',parameters=[{'use_sim_time':sim,'autostart':True,'node_names':[name for _,name in servers]}]))
    actions.append(Node(package='asr_summer_school',executable='motion_guard.py',output='screen',parameters=[{'use_sim_time':sim}]))
    actions.append(Node(package='asr_summer_school',executable='mission_orchestrator.py',output='screen',
                        parameters=[{'use_sim_time':sim,'duration':LaunchConfiguration('duration'),'max_radius':LaunchConfiguration('max_radius')}]))
    return LaunchDescription(actions)
