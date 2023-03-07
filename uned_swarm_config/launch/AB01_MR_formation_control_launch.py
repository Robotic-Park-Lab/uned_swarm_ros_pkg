import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    config_package_dir = get_package_share_directory('uned_swarm_config')
    dron_description = pathlib.Path(os.path.join(config_package_dir, 'resources', 'crazyflie.urdf')).read_text()
    config_path = os.path.join(config_package_dir, 'resources', 'AB01_formation_configuration.yaml')
    rviz_config_path = os.path.join(config_package_dir, 'rviz', 'AB01_formation.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    webots = WebotsLauncher(
        world=os.path.join(config_package_dir, 'worlds', 'AB01_RoboticPark_n10.wbt')
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
            {'use_sim_time': use_sim_time},
            {'n': 4},
            {'config': config_path}
        ]
    )

    dron02_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron02',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron02',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron02',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron03_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron03',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron03',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron03',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron04_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron04',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron04',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron04',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron05_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron05',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron05',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron05',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron06_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron06',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron06',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron06',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron07_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron07',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron07',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron07',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron08_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron08',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron08',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron08',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron09_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron09',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron09',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron09',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron10_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron10',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron10',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron10',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron11_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron11',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron11',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron11',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )
    
    dron12_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron12',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron12',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron12',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron13_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron13',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron13',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron13',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron14_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron14',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron14',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron14',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='interface',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    vicon_node = Node(
        package='uned_vicon_sim', 
        executable='vicon_webots',
        name='vicon_webots',
        output='screen',
        #arguments=['--ros-args', '--log-level', 'info'],
        parameters=[
            {'use_sim_time': use_sim_time},
            {"agents": 'dron05, dron06, dron07, dron08, dron09, dron10, dron11, dron12, dron13, dron14'},
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        webots,
        ros2_supervisor,
        swarm_node,
        dron05_driver,
        dron06_driver,
        dron07_driver,
        dron08_driver,
        dron09_driver,
        dron10_driver,
        dron11_driver,
        dron12_driver,
        dron13_driver,
        dron14_driver,
        # robot_state_publisher,
        rqt_node,
        vicon_node,
        rviz_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])