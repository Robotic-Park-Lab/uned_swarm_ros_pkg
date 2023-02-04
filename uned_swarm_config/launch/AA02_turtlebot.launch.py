import os
import pathlib
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    config_package_dir = get_package_share_directory('uned_swarm_config')
    config_path = os.path.join(config_package_dir, 'resources', 'AA02_distance_formation_configuration_event.yaml')
    rviz_config_path = os.path.join(config_package_dir, 'rviz', 'test.rviz')
    robot_description = pathlib.Path(os.path.join(config_package_dir, 'resources', 'kheperaiv.urdf')).read_text()
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    webots = WebotsLauncher(
        world=os.path.join(config_package_dir, 'worlds', 'AA01_RoboticPark_3kh.wbt')
    )

    ros2_supervisor = Ros2SupervisorLauncher()

    swarm_node = Node(
        package='uned_crazyflie_driver',
        executable='swarm_driver',
        name='swarm',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'first_uri': 'radio://0/80/2M/E7E7E7E701'},
            {'n': 4},
            {'config': config_path}
        ]
    )

    turtlebot_node = Node(
        package='uned_swarm_driver',
        executable='turtlebot_driver',
        name='driver',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'id': 'turtlebot01'},
            {'config': config_path}
        ]
    )

    real_robot_task = Node(
        package='uned_kheperaiv_task',
        executable='distance_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='turtlebot01',
        parameters=[
            {"config_file": config_path},
            {"robot": 'turtlebot01'},
        ]
    )

    webots_khepera02_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='khepera02',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'khepera02',
                        'WEBOTS_ROBOT_NAME': 'khepera02'},
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ],
    )

    webots_khepera02_task = Node(
        package='uned_kheperaiv_task',
        executable='distance_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='khepera02',
        parameters=[
            {"config_file": config_path},
            {"robot": 'khepera02'},
        ]
    )

    webots_khepera03_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='khepera03',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'khepera03',
                        'WEBOTS_ROBOT_NAME': 'khepera03'},
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ],
    )

    webots_khepera03_task = Node(
        package='uned_kheperaiv_task',
        executable='distance_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='khepera03',
        parameters=[
            {"config_file": config_path},
            {"robot": 'khepera03'},
        ]
    )

    webots_khepera04_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='khepera04',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'khepera04',
                        'WEBOTS_ROBOT_NAME': 'khepera04'},
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ],
    )

    webots_khepera04_task = Node(
        package='uned_kheperaiv_task',
        executable='distance_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='khepera04',
        parameters=[
            {"config_file": config_path},
            {"robot": 'khepera04'},
        ]
    )

    rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='interface'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],

    )

    vicon_node = Node(
        package='uned_vicon_sim', 
        executable='vicon_webots',
        name='vicon_webots',
        output='screen',
        #arguments=['--ros-args', '--log-level', 'info'],
        parameters=[
            {'use_sim_time': use_sim_time},
            {"agents": 'khepera02, khepera03, khepera04'},
        ]
    )

    return LaunchDescription([
        webots,
        ros2_supervisor,
        swarm_node,
        turtlebot_node,
        real_robot_task,
        webots_khepera02_driver,
        webots_khepera02_task,
        webots_khepera03_driver,
        webots_khepera03_task,
        webots_khepera04_driver,
        webots_khepera04_task,
        rqt_node,
        rviz_node,
        vicon_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])