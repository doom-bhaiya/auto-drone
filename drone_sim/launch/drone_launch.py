from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

from ament_index_python import get_package_prefix

pkg_share_path = os.pathsep + os.path.join(get_package_prefix("drone_sim"), 'share')
if 'GAZEBO_MODEL_PATH' in os.environ:
    os.environ['GAZEBO_MODEL_PATH'] += pkg_share_path
else:
    os.environ['GAZEBO_MODEL_PATH'] =  pkg_share_path


def generate_launch_description():

    ld = LaunchDescription()

    gazebo_ros_pkg_path = get_package_share_directory("gazebo_ros")
    launch_file_path = os.path.join(gazebo_ros_pkg_path, "launch", "gazebo.launch.py") 

    gazebo_ros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path)
    )

    urdf_spawner_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=['-topic', '/robot_description', '-entity', 'robot', '-z', '0.5', '-unpause'],
        output='screen',
    )

    bot_pkg_path = get_package_share_directory("drone_sim")
    bot_path = os.path.join(bot_pkg_path, "urdf", "drone.urdf")


    spawner_node =  Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=[
                            '-entity', 'my_robot',
                            '-file', bot_path,
                            '-z', '0.1',
                            '-robot_namespace', ''
                        ],
                        output='screen'
                    )


    ld.add_action(gazebo_ros_launch)
    ld.add_action(spawner_node)

    return ld




# from launch import LaunchDescription
# from launch_ros.actions import Node


# def generate_launch_description():
#     return LaunchDescription([
#         IncludeLaunchDescription(
#             PathJoinSubstitution([
#                 FindPackageShare('gazebo_ros'),
#                 'launch',
#                 'gazebo.launch.py']))
#             ])
        
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# import os


# def generate_launch_description():

#     gazebo_ros_pkg = get_package_share_directory('gazebo_ros')


#     return LaunchDescription([
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
#             )
#         ),
#         Node(
#             package='drone_sim',
#             executable='param_loader',
#             name='param_loader',
#             parameters=[{
#                 'param_file_path': PathJoinSubstitution([drone_sim_path, 'config', 'drone_params.txt'])
#             }]
#         )
#     ])
    
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess
# from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare

# def generate_launch_description():
#     drone_sim_path = FindPackageShare("drone_sim")

#     robot_description = Command([
#         'xacro ',
#         PathJoinSubstitution([drone_sim_path, 'urdf', 'drone.urdf'])
#     ])

#     return LaunchDescription([
#         # Set robot_description parameter
#         Node(
#             package="robot_state_publisher",
#             executable="robot_state_publisher",
#             name="robot_state_publisher",
#             parameters=[{'robot_description': robot_description}]
#         ),

#         # Launch Gazebo with a custom world
#         ExecuteProcess(
#             cmd=[
#                 'gazebo',
#                 '--verbose',
#                 PathJoinSubstitution([drone_sim_path, 'world', 'base_world.world']),
#                 '-s', 'libgazebo_ros_factory.so'
#             ],
#             output='screen'
#         ),

#         # Custom parameter loading node

#         # Spawn robot in Gazebo
#         Node(
#             package='gazebo_ros',
#             executable='spawn_entity.py',
#             arguments=[
#                 '-entity', 'my_robot',
#                 '-file', PathJoinSubstitution([drone_sim_path, 'urdf', 'drone.urdf']),
#                 '-z', '0.1',
#                 '-robot_namespace', ''
#             ],
#             output='screen'
#         )
#     ])





