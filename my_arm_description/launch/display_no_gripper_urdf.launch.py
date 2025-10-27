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
    model_urdf_name ='piper_no_gripper_description_gazebo.xacro'

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
    rviz_config_arg = DeclareLaunchArgument('rvizconfig',
                                            description='Path to an RViz config file',
                                            default_value=PathJoinSubstitution([pkg_Path, 'rviz', 'rviz.rviz']))
                            
    use_ros2_control_arg = DeclareLaunchArgument('use_ros2_control', default_value='false')


    description_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': LaunchConfiguration('urdf_package'),
            'urdf_package_path': LaunchConfiguration('model')}.items()
    )
    '''
    empty_world_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
        launch_arguments={
            'gui': LaunchConfiguration('gui')
        }.items(),
    )

    robot_description = {
    'robot_description': ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )}




    # ====== Robot State Publisher（仿真时钟）======
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )
    """"TODO: 完善理解"""
    #从底层硬件接口（或仿真接口，如 Gazebo）中读取各关节的状态（角度、速度、力矩等），
    #并发布到 /joint_states 话题。
    #所以他的作用就是相当于从仿真环境获取机械臂的信息
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    # 路径执行控制器，也就是那个action？
    # 系统是如何知道有my_group_controller这个控制器的存在？
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'arm_controller'],
        output='screen'
        )

    """"TODO: 完善理解"""
    # ====== 启动 Gazebo Classic（gazebo_ros 的官方 launch）======
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py']
        ),
        launch_arguments={'verbose': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('start_gazebo'))
    )

    # Launch the robot, 通过robot_description话题进行模型内容获取从而在gazebo中生成模型
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model,  '-topic', 'robot_description'], 
        output='screen',
    )

    # ====== （可选）控制器加载：仅当 use_ros2_control=true 时执行 ======
    load_jsb = Node(  # joint_state_broadcaster
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ros2_control'))
    )
    load_arm = Node(  # 你的轨迹控制器/位置控制器，名字按你的 ros2_control 配置修改
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ros2_control'))
    )
    # 先等 spawn_entity_cmd 完成，再加载控制器（顺序更稳）
    after_spawn_load_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity_cmd,
            on_exit=[load_jsb],
        )
    )
    after_jsb_load_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=load_jsb,
            on_exit=[load_arm],
        )
    )

    # ====== RViz ======
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
    )

    # 用参数开关：如果没用 ros2_control，就起 JSP（或 JSP GUI）
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
        condition=UnlessCondition(LaunchConfiguration('use_ros2_control')),
        output='screen'
    )

    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=UnlessCondition(LaunchConfiguration('use_ros2_control')),
        output='screen'
    )

    # 用下面这两个估计是想控制好各个节点的启动顺序
    # 监听 spawn_entity_cmd，当其退出（完全启动）时，启动load_joint_state_controller？
    close_evt1 =  RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=spawn_entity_cmd,
                on_exit=[load_joint_state_controller],
            )
    )
    # 监听 load_joint_state_controller，当其退出（完全启动）时，启动load_joint_trajectory_controller？
    # moveit是怎么和gazebo这里提供的action连接起来的？？
    close_evt2 = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
    )
###########################LOG_PART_START##########################
    log_pkg_path =LogInfo(
    msg=[
        '\n==========================================================\n',
        '\n[DEBUG]  log_pkg_path (Substitution) is: ', 
        pkg_Path,
        '\n==========================================================\n'
    ])
    log_model_path = LogInfo(
        msg=[
            '==========================================================',
            '\n[DEBUG] The effective robot model path (Launch Arg) is: ', 
            model_config,
            '\n=========================================================='
    ])
    '''





# ========== 9) 分步式组装 ==========
    #TODO: 完善理解
    ld = LaunchDescription()

    # 先声明参数
    ld.add_action(package_arg)
    ld.add_action(start_gazebo_arg)
    ld.add_action(model_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(description_launch_py)
    return ld
    #TODO: 完善理解