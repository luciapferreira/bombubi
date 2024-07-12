import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    package_name='bombubi'

    bombubi = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','bombubi.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/cmd_vel')]
        )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    world_file = os.path.join(get_package_share_directory(package_name), 'worlds', 'bookstore.world')  
    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={
                        'world': world_file,
                        'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
                    }.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'bombubi'],
                        remappings=[('/gazebo/spawn_entity', '/spawn_entity')
        ])
    #                    output='screen')

    #diff_drive_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner",
    #    arguments=["diff_cont"],
    #)

    #joint_broad_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner",
    #    arguments=["joint_broad"],
    #)
    
    #slam = IncludeLaunchDescription(
    #            PythonLaunchDescriptionSource([os.path.join(
    #                get_package_share_directory(package_name),'launch','orbslam3.launch.py'
    #            )])
    #)

    # Launch them all!
    return LaunchDescription([
        bombubi,
        joystick,
        twist_mux,
        gazebo,
        spawn_entity,
        #diff_drive_spawner,
        #joint_broad_spawner
    ])
