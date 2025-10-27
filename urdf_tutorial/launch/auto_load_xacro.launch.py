from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution,LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
   
   urdf_package ='urdf_tutorial'
   urdf_name = '08-macroed.urdf'
   urdf_package_arg  = DeclareLaunchArgument('urdf_package',
                                    description='The package where the robot description is located',
                                    default_value=urdf_package)
   model_name_arg =DeclareLaunchArgument('model_name',
                                    description='the name of the description that you want to show',
                                    default_value=PathJoinSubstitution(['urdf',urdf_name]))
   description_launch =IncludeLaunchDescription(
      PathJoinSubstitution([FindPackageShare('urdf_launch'),'launch','display.launch.py']),
      launch_arguments={
        'urdf_package':LaunchConfiguration('urdf_package'),
        'urdf_package_path':LaunchConfiguration('model_name')
      }.items()
   )
   ld = LaunchDescription()
   ld.add_action(urdf_package_arg)
   ld.add_action(model_name_arg)
   ld.add_action(description_launch)
   return ld