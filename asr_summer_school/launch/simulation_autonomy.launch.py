"""Gazebo Classic integration. No physical sensor drivers and no automatic arm."""
import os
import tempfile
import xml.etree.ElementTree as ET
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def setup(context):
    share=get_package_share_directory('asr_summer_school')
    tb=get_package_share_directory('turtlebot3_gazebo')
    gz=get_package_share_directory('gazebo_ros')
    # Normalize simulated camera to the same public topics as the real camera.
    tree=ET.parse(os.path.join(tb,'models','turtlebot3_burger','model.sdf'))
    camera=tree.find(".//sensor[@type='camera']")
    camera.find('camera/image/width').text='640'
    camera.find('camera/image/height').text='480'
    camera.find('update_rate').text='15'
    plugin=camera.find("plugin[@filename='libgazebo_ros_camera.so']")
    ros=plugin.find('ros')
    ET.SubElement(ros,'remapping').text='/color/image_raw:=/camera/color/image_raw'
    ET.SubElement(ros,'remapping').text='/color/camera_info:=/camera/color/camera_info'
    ET.SubElement(plugin,'camera_name').text='color'
    ET.SubElement(plugin,'frame_name').text='camera_rgb_optical_frame'
    model=tempfile.NamedTemporaryFile(suffix='.sdf',delete=False)
    model.close();tree.write(model.name)
    params=yaml.safe_load(open(os.path.join(get_package_share_directory('turtlebot3_perception'),'config','apriltag.yaml')))['camera']['apriltag']['ros__parameters']
    def include(folder,file,**kwargs):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(folder,'launch',file)),launch_arguments=kwargs.items())
    return [
        SetEnvironmentVariable('GAZEBO_MODEL_PATH',os.path.join(share,'models')+':'+os.path.join(tb,'models')+':/usr/share/gazebo-11/models:'+os.environ.get('GAZEBO_MODEL_PATH','')),
        include(gz,'gzserver.launch.py',world=LaunchConfiguration('world').perform(context)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gz,'launch','gzclient.launch.py')),condition=IfCondition(LaunchConfiguration('gui'))),
        include(tb,'robot_state_publisher.launch.py',use_sim_time='true'),
        Node(package='gazebo_ros',executable='spawn_entity.py',arguments=['-entity','burger','-file',model.name,'-z','0.01'],output='screen'),
        include(share,'slam_toolbox.launch.py',use_sim_time='true'),
        include(share,'autonomy.launch.py',use_sim_time='true',sensors='false',nav_params=os.path.join(share,'config','simulation_rpp.yaml'),duration=LaunchConfiguration('duration').perform(context),max_radius='5.0'),
        Node(package='apriltag_ros',executable='apriltag_node',namespace='camera',name='apriltag',parameters=[params,{'use_sim_time':True}],remappings=[('image_rect','/camera/color/image_raw'),('camera_info','/camera/color/camera_info')]),
        Node(package='asr_summer_school',executable='tag_landmarks.py',parameters=[{'use_sim_time':True}]),
        Node(package='rviz2',executable='rviz2',arguments=['-d',os.path.join(share,'config','mission.rviz')],parameters=[{'use_sim_time':True}],condition=IfCondition(LaunchConfiguration('rviz')))]


def generate_launch_description():
    share=get_package_share_directory('asr_summer_school')
    return LaunchDescription([DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('duration',default_value='180.0'),DeclareLaunchArgument('world',default_value=os.path.join(share,'worlds','smoke_tags.world')),SetEnvironmentVariable('TURTLEBOT3_MODEL','burger'),OpaqueFunction(function=setup)])
