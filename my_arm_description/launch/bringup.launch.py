import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command,PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction,RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    bringup_dir = get_package_share_directory('robot_bringup')
    world_path = PathJoinSubstitution([FindPackageShare("robot_bringup"), "world", "my_world.world"])
    urdf_dir = get_package_share_path('robot_bringup') / 'urdf' / 'fw_mid.xacro'

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz', default='false')


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
        # 传递参数
        launch_arguments=[('world', world_path), ('verbose', 'true')]
    )

    # declare_rviz_config_file_cmd = DeclareLaunchArgument(
    #     'rviz_config_file',
    #     default_value=os.path.join(bringup_dir, 'rviz', 'rviz2.rviz'),
    #     description='Full path to the RVIZ config file to use'
    # )

    start_gazebo_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', 'robot', 
                   '-x', '0.0',      # 设置 X 坐标
                   '-y', '0.0',      # 设置 Y 坐标
                   '-z', '0.2',      # 设置 Z 坐标 (高度)
                   '-Y', '0.0'      # 设置 Yaw 偏航角 (绕Z轴旋转, 1.57 弧度约等于 90度)
                   ],
                   output='screen')


    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', str(urdf_dir)]), value_type=str
            ),
        }],
        output='screen'
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', str(urdf_dir)]), value_type=str
            ),
        }],
        output='screen'
    )

    start_point_deal_cmd = Node(
        package='robot_bringup',
        executable='point_deal',
        name='point_deal',
        output='screen'
    )


    # ==================================================================
    # == 控制器启动 (修改后的核心部分) ==
    # ==================================================================

    # 1. 首先加载 joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 2. 等待 joint_state_broadcaster 成功启动后，再启动其他控制器
    #    这是通过事件处理机制实现的
    delay_other_controllers_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                # 这两个 spawner 会在 jsb 成功退出后才执行
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["steering_position_controller", "--controller-manager", "/controller_manager"],
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["wheel_velocity_controller", "--controller-manager", "/controller_manager"],
                ),
            ],
        )
    )

    # 运动学转换节点
    kinematics_node = Node(
        package='robot_bringup',
        executable='cmd_vel_to_joints_node.py',
        name='cmd_vel_to_joints_node',
        output='screen'
    )




    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        namespace='',
        executable='rviz2',
        arguments=['-d' + os.path.join(bringup_dir, 'rviz', 'rviz2.rviz')]
    )


    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(launch_gazebo)
    ld.add_action(start_point_deal_cmd)
    ld.add_action(start_gazebo_node)
    ld.add_action(declare_use_sim_time_cmd)
    # ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(delay_other_controllers_after_jsb)
    ld.add_action(kinematics_node)

    return ld

