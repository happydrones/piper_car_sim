#FIXME: 路径问题暂时解决了，后续还是希望使用package://方法获取meshes
#TODO :修改$GAZEBO_MODEL_PATH变量设置方法
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch_ros.parameter_descriptions import ParameterValue  # 提供参数值包装
from launch.substitutions import Command          # 用于在 launch 中执行 xacro
from launch.event_handlers import OnProcessExit
def generate_launch_description():
    robot_name_in_model = 'yunhai_gripper'
    package_name = 'my_arm_description'
    urdf_name = 'piper_with_my_gripper_description_gazebo.urdf.xacro'

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
    # ====== RViz ======
    rviz_arg         = DeclareLaunchArgument('rvizconfig',
                                            default_value=PathJoinSubstitution([pkg_Path, 'rviz', 'rviz.rviz']),
                                            description='Path to an RViz config file'
    )
    
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
        parameters=[robot_description, {'use_sim_time': True},{"publish_frequency":15.0}],
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
    action_load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'arm_controller'],
        output='screen'
        )
    close_evt1 =  RegisterEventHandler( 
            event_handler=OnProcessExit(
                target_action=spawn_entity_cmd,
                on_exit=[action_load_joint_state_controller],
            )
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(start_gazebo_cmd)
    # 先声明参数
    ld.add_action(start_gazebo_arg)
    ld.add_action(model_arg)
    ld.add_action(package_arg)
    ld.add_action(rviz_arg)
    ld.add_action(rviz)
    ld.add_action(rsp)
    # 把它添加进来
    ld.add_action(jsp)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(close_evt1)
    return ld