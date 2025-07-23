from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


from ament_index_python import get_package_share_directory
import os
import xacro

def generate_launch_description():

    set_log_level = SetEnvironmentVariable('IGN_LOG_LEVEL', 'info')

    pkg_share_dir = get_package_share_directory('drone_sim')
    models_path = pkg_share_dir + "/meshes"
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    drone_sim_pkg_path = get_package_share_directory("drone_sim")
    sdf_path = os.path.join(drone_sim_pkg_path, "worlds", "empty.sdf")

    ld = LaunchDescription()

    gazebo_ros_pkg_path = get_package_share_directory("ros_gz_sim")

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_path, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [sdf_path], 'on_exit_shutdown': 'true'}.items()
    )


    bot_pkg_path = get_package_share_directory("drone_sim")
    bot_path = os.path.join(bot_pkg_path, "urdf", "drone.urdf")
    xacro_file_path = bot_path
    robot_description_config = xacro.process_file(xacro_file_path)
    robot_desc = robot_description_config.toxml()

    params = {"robot_description": robot_desc, "use_sim_time": use_sim_time}

    spawner_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            "-name","drone",
            "-allow_renaming","true",
            "-topic","/robot_description",
            "-x","0.0",
            "-y","0.0",
            "-z","0.2",
        ],
        output='screen',
        )

    spawner_delayed = TimerAction(
    period=3.0,  
    actions=[spawner_node]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params],
    )



    bridge_rule = [
        '/cmd_vel@ros_gz_interfaces/msg/Float32Array@ignition.msgs.Float_V',
        '/switchOn@std_msgs/msg/Bool@ignition.msgs.Boolean',
        '/DronePose@nav_msgs/msg/Odometry@ignition.msgs.Odometry'
    ]

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_float_bridge',
        output='screen',
        arguments = bridge_rule,
    )

    switch_on_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("drone_sim"),
                "launch",
                "enable_drone_launch.py"
            )
        )
    )

    switch_on_launch_delayed = TimerAction(
    period=4.0,  
    actions=[switch_on_launch]
    )

    ld.add_action(set_log_level)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(gzserver_cmd)
    ld.add_action(spawner_delayed)
    ld.add_action(bridge_node)
    ld.add_action(switch_on_launch_delayed)
    

    return ld

