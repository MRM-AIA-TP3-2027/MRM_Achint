from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    pkg_path   = get_package_share_directory("robot")
    gazebo_ros = get_package_share_directory("gazebo_ros")

    xacro_file  = os.path.join(pkg_path, "urdf", "rover.urdf.xacro")
    robot_desc  = xacro.process_file(xacro_file).toxml()

    
    world_file  = os.path.join(pkg_path, "world", "rover_world.world")

     
    rviz_config = os.path.join(pkg_path, "rviz", "rover.rviz")
    rviz_args   = ["-d", rviz_config] if os.path.isfile(rviz_config) else []

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros, "launch", "gazebo.launch.py")
            ),
            launch_arguments={
                "world":   world_file,
                "verbose": "false",
            }.items(),
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_desc,
                "use_sim_time": True,
            }],
        ),

        
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "robot",
                "-topic",  "robot_description",
                "-x", "0", "-y", "0", "-z", "0.15",
            ],
            output="screen",
        ),
        Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="screen",
    arguments=rviz_args,
),

       
    ])
