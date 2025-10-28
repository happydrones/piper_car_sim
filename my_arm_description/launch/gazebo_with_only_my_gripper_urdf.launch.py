from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution,Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# 条件判断
from launch.actions import RegisterEventHandler
# 事件处理相关
from launch.conditions import IfCondition ,UnlessCondition
#日志打印
from launch.actions import LogInfo
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    robot_name_in_model = 'piper'
    package_name = 'my_arm_description'
    pkg_Path = FindPackageShare(package_name)

    log_pkg_path =LogInfo(
        msg=[
            '==========================================================',
            '\n[DEBUG] The effective log_pkg_path (Substitution) is: ', 
            pkg_Path,
            '\n=========================================================='
        ]
    )

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
                                            default_value=PathJoinSubstitution([pkg_Path, 'urdf', 'piper_no_gripper_description.urdf'])
    )

    # 是否加载 ros2_control 控制器（你的 URDF/控制器必须已正确配置，否则请保持 false）
    use_ros2_control_arg = DeclareLaunchArgument('use_ros2_control', default_value='false')



    
    # 2. 定义 LogInfo Action 
    log_model_path = LogInfo(
        msg=[
            '==========================================================',
            '\n[DEBUG] The effective robot model path (Launch Arg) is: ', 
            LaunchConfiguration('model'),
            '\n=========================================================='
        ]
    )

    empty_world_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py']),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'pause': 'true',
        }.items(),
    )
    description_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
        launch_arguments={

            'urdf_package': LaunchConfiguration('urdf_package'),
            'urdf_package_path': LaunchConfiguration('model')}.items()
    )

    rviz_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=PathJoinSubstitution([pkg_Path, 'rviz', 'rviz.rviz']),
        description='Path to an RViz config file'
    )

    # push robot_description to factory and spawn robot in gazebo
    
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
    # # 先等 spawn_entity_cmd 完成，再加载控制器（顺序更稳）
    # after_spawn_load_jsb = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=spawn_entity_cmd,
    #         on_exit=[load_jsb],
    #     )
    # )
    # after_jsb_load_arm = RegisterEventHandler(
    #     OnProcessExit(
    #         target_action=load_jsb,
    #         on_exit=[load_arm],
    #     )
    # )

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

    # # 用下面这两个估计是想控制好各个节点的启动顺序
    # # 监听 spawn_entity_cmd，当其退出（完全启动）时，启动load_joint_state_controller？
    # close_evt1 =  RegisterEventHandler( 
    #         event_handler=OnProcessExit(
    #             target_action=spawn_entity_cmd,
    #             on_exit=[load_joint_state_controller],
    #         )
    # )
    # # 监听 load_joint_state_controller，当其退出（完全启动）时，启动load_joint_trajectory_controller？
    # # moveit是怎么和gazebo这里提供的action连接起来的？？
    # close_evt2 = RegisterEventHandler(
    #         event_handler=OnProcessExit(
    #             target_action=load_joint_state_controller,
    #             on_exit=[load_joint_trajectory_controller],
    #         )
    # )

# ========== 9) 分步式组装 ==========
    ld = LaunchDescription()

    # 先声明参数
    ld.add_action(start_gazebo_arg)
    ld.add_action(model_arg)
    ld.add_action(package_arg)
    ld.add_action(log_model_path)
    ld.add_action(use_ros2_control_arg)

    ld.add_action(rviz_arg)
    # 1) 先起 Gazebo（这样 /spawn_entity 服务才会存在）
    ld.add_action(empty_world_launch)
    ld.add_action(description_launch_py)


    # 3) 把机器人真正 spawn 进 Gazebo
    ld.add_action(spawn_entity_cmd)

    # 4) 若使用 ros2_control：spawn 成功后 -> 加载 jsb -> 再加载 arm_controller
    # ld.add_action(after_spawn_load_jsb)
    # ld.add_action(after_jsb_load_arm)

    # 5) 如果没用 ros2_control，就用 JSP/JSP GUI 人工发 /joint_states（UnlessCondition 控制）
    ld.add_action(jsp)
    ld.add_action(jsp_gui)

    # 6) 最后起 RViz
    # ld.add_action(rviz)

    return ld
