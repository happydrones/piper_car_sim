import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
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
    # 1. 获取包的 'share' 目录的【真实字符串路径】
    pkg_share_dir = get_package_share_directory(package_name)
    resource_path = os.environ.get('GAZEBO_RESOURCE_PATH', '')
    new_resource_path = f"{pkg_share_dir}{os.pathsep}{resource_path}"
    set_resource_path = SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', new_resource_path)
    # 3. (修复gzclient刷屏错误) 设置 GAZEBO_MODEL_PATH
    #    告诉 gzclient 在哪里可以找到 'Insert' 标签页的模型 (假设你把模型放在了 'models' 文件夹)
    pkg_models_path = os.path.join(pkg_share_dir, 'models')
    model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_model_path = f"{pkg_models_path}{os.pathsep}{model_path}"
    set_model_path = SetEnvironmentVariable('GAZEBO_MODEL_PATH', new_model_path)
    # ====== 可调参数 ======
    # 是否启动 Gazebo（默认 true）；只想看 RViz 可设为 false
    start_gazebo_arg = DeclareLaunchArgument('start_gazebo', default_value='true')
    # 是否加载 ros2_control 控制器（你的 URDF/控制器必须已正确配置，否则请保持 false）
    use_ros2_control_arg = DeclareLaunchArgument('use_ros2_control', default_value='false')

    # xacro/rviz 配置
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=PathJoinSubstitution([pkg_Path, 'urdf', 'piper_with_my_gripper_description_gazebo.urdf'])
    )
    # ******** 新增：打印 model 参数的值 ********
    # 1. 获取 model 参数的值 (Substitution 对象)
    model_config = LaunchConfiguration('model')
    
    # 2. 定义 LogInfo Action 
    log_model_path = LogInfo(
        msg=[
            '==========================================================',
            '\n[DEBUG] The effective robot model path (Launch Arg) is: ', 
            model_config,
            '\n=========================================================='
        ]
    )
    # *****************************************

    robot_description = {
    'robot_description': ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )
}

    rviz_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=PathJoinSubstitution([pkg_Path, 'rviz', 'rviz.rviz']),
        description='Path to an RViz config file'
    )
    gui_arg = DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'])


    # ====== Robot State Publisher（仿真时钟）======
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )
    """"TODO: 完善理解"""

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
    #TODO: 完善理解
    ld = LaunchDescription()

    # 先声明参数
    ld.add_action(start_gazebo_arg)
    ld.add_action(use_ros2_control_arg)
    ld.add_action(gui_arg)
    ld.add_action(model_arg)
    ld.add_action(rviz_arg)
    # --- 在启动 Gazebo 之前设置好环境变量 ---
    ld.add_action(set_resource_path)
    ld.add_action(set_model_path)
    # 1) 先起 Gazebo（这样 /spawn_entity 服务才会存在）
    ld.add_action(gazebo)

    # 2) 起 RSP（发布 /robot_description 到 TF，等待 joint_states）
    ld.add_action(rsp)

    # 3) 把机器人真正 spawn 进 Gazebo
    ld.add_action(spawn_entity_cmd)

    # 4) 若使用 ros2_control：spawn 成功后 -> 加载 jsb -> 再加载 arm_controller
    ld.add_action(after_spawn_load_jsb)
    ld.add_action(after_jsb_load_arm)

    # 5) 如果没用 ros2_control，就用 JSP/JSP GUI 人工发 /joint_states（UnlessCondition 控制）
    ld.add_action(jsp)
    ld.add_action(jsp_gui)

    # 6) 最后起 RViz
    ld.add_action(rviz)

    return ld
    #TODO: 完善理解