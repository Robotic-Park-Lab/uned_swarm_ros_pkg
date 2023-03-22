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
    config_path = os.path.join(config_package_dir, 'resources', 'AC04_formation_configuration.yaml')
    rviz_config_path = os.path.join(config_package_dir, 'rviz', 'AB01_formation.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    webots = WebotsLauncher(
        world=os.path.join(config_package_dir, 'worlds', 'AC04_RoboticPark_n32.wbt')
    )

    ros2_supervisor = Ros2SupervisorLauncher()

    dron01_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron01',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron01',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron01',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
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

    dron15_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron15',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron15',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron15',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron16_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron16',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron16',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron16',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron17_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron17',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron17',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron17',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron18_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron18',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron18',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron18',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron19_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron19',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron19',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron19',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron20_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron20',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron20',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron20',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron21_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron21',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron21',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron21',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron22_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron22',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron22',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron22',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron23_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron23',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron23',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron23',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron24_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron24',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron24',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron24',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron25_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron25',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron25',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron25',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron26_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron26',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron26',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron26',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron27_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron27',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron27',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron27',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron28_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron28',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron28',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron28',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron29_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron29',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron29',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron29',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron30_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron30',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron30',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron30',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron31_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron31',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron31',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron31',
                        'WEBOTS_ROBOT_CONFIG_FILE': config_path},
        parameters=[
            {'robot_description': dron_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron32_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='dron32',
        additional_env={'WEBOTS_ROBOT_NAME': 'dron32',
                        'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron32',
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
        dron01_driver,
        dron02_driver,
        dron03_driver,
        dron04_driver,
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
        dron15_driver,
        dron16_driver,
        dron17_driver,
        dron18_driver,
        dron19_driver,
        dron20_driver,
        dron21_driver,
        dron22_driver,
        dron23_driver,
        dron24_driver,
        dron25_driver,
        dron26_driver,
        dron27_driver,
        dron28_driver,
        dron29_driver,
        dron30_driver,
        dron31_driver,
        dron32_driver,
        # robot_state_publisher,
        # rqt_node,
        rviz_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])