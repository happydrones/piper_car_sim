from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.actions import ExecuteProcess
# 条件判断
from launch.actions import RegisterEventHandler
# 事件处理相关
from launch.conditions import IfCondition ,UnlessCondition
#日志打印
from launch.actions import LogInfo
from launch_ros.parameter_descriptions import ParameterValue  # 提供参数值包装
from launch.substitutions import Command          # 用于在 launch 中执行 xacro

def generate_launch_description():
    robot_name_in_model = 'piper_arm'
    package_name = 'my_arm_description'
    urdf_name = 'test_gripper.urdf'

    pkg_Path = FindPackageShare(package_name)
    # ====== 可调参数 ======
    package_arg = DeclareLaunchArgument('urdf_package',
                                        description='The package where the robot description is located',
                                        default_value='my_arm_description') 
    start_gazebo_arg = DeclareLaunchArgument('gui',
                                            description='Start Gazebo simulation (true/false)',
                                            default_value='true')
    # 模型传入参数配置
    model_arg        = DeclareLaunchArgument('model',
                                            description='Absolute path to robot urdf file',
                                            default_value=PathJoinSubstitution([pkg_Path, 'urdf', urdf_name]))
    # 是否加载 ros2_control 控制器（你的 URDF/控制器必须已正确配置，否则请保持 false）
    use_ros2_control_arg = DeclareLaunchArgument('use_ros2_control', default_value='false')
    
    robot_description = {'robot_description': ParameterValue(
    Command(['xacro ', LaunchConfiguration('model')]), value_type=str)}

    # 启动 gazebo 仿真
    # Start Gazebo server
    start_gazebo_cmd =  ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen')
    
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )

    # # Launch the robot, 通过robot_description话题进行模型内容获取从而在gazebo中生成模型
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, '-topic', 'robot_description'], 
        parameters=[robot_description],   # 关键：把同名参数也传给 spawner
        output='screen',
    )

    # ====== RViz ======
    rviz_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=PathJoinSubstitution([pkg_Path, 'rviz', 'rviz.rviz']),
        description='Path to an RViz config file'
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
    )



    # 路径执行控制器，也就是那个action？
    # 系统是如何知道有my_group_controller这个控制器的存在？
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'arm_controller'],
        output='screen'
        )



    #===========LOG_START=================
    #============LOG_END==================
    ld = LaunchDescription()
    ld.add_action(start_gazebo_cmd)
    # 先声明参数
    ld.add_action(start_gazebo_arg)
    ld.add_action(model_arg)
    ld.add_action(package_arg)

    # ===========LOG_START=================

    ld.add_action(use_ros2_control_arg)

    ld.add_action(rviz_arg)

    ld.add_action(rsp)

    ld.add_action(spawn_entity_cmd)


    return ld
    #TODO: 完善理解