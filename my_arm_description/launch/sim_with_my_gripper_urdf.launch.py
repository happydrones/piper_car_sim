#TODO  bug: 目前手臂无法在gazebo中正常显示，并且会遍历其他路径寻找urdf文件，怀疑是model参数传递的问题
#TODO  控制也仅仅作用在了rviz中
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess , RegisterEventHandler ,LogInfo
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
import xacro
import re
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue


from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    robot_name_in_model = 'piper'
    package_name = 'my_arm_description'
    pkg_Path = FindPackageShare(package_name)
    # model_urdf_name ='piper_no_gripper_description_gazebo.xacro'
    model_urdf_name ='test_gripper.urdf.xacro'

    # ====== parameters ======
    package_arg = DeclareLaunchArgument('urdf_package',
                                        description='The package where the robot description is located',
                                        default_value='my_arm_description') 
    start_gazebo_arg = DeclareLaunchArgument('gui',
                                            description='Start Gazebo simulation (gui:=true/false)',
                                            default_value='true')

    model_arg        = DeclareLaunchArgument('model',
                                            description='Absolute path to robot urdf file',
                                            default_value=PathJoinSubstitution([ 'urdf', model_urdf_name]))

    # Start Gazebo server
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')
    xacro_file = get_package_share_directory('my_arm_description') +'/urdf/'+ model_urdf_name
    # Robot State Publisher 
    robot_state_publisher = Node(package    ='robot_state_publisher',
                                 executable ='robot_state_publisher',
                                 name       ='robot_state_publisher',
                                 output     ='both',
                                 parameters =[{'robot_description': Command(['xacro ',xacro_file ])           
                                }])
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'arm_controller'],
        output='screen'
    )
    # push robot_description to factory and spawn robot in gazebo
    urdf_spawner_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=['-topic', '/robot_description', '-entity',robot_name_in_model , '-z', '0', '-unpause'],
        output='screen',
    )

# ========== 9) 分步式组装 ==========

    ld = LaunchDescription()
    # 先声明参数
    ld.add_action(package_arg)
    ld.add_action(start_gazebo_arg)
    ld.add_action(model_arg)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(load_joint_trajectory_controller)
    ld.add_action(urdf_spawner_node)
    ld.add_action(load_joint_state_controller)
    return ld