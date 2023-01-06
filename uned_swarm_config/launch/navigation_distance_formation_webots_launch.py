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
    swarm_package_dir = get_package_share_directory('uned_swarm_config')
    dron_package_dir = get_package_share_directory('uned_crazyflie_webots')
    robot_package_dir = get_package_share_directory('uned_kheperaiv_webots')
    package_dir = get_package_share_directory('webots_ros2_turtlebot')
    default_rviz_config_path = os.path.join(swarm_package_dir, 'rviz', 'test.rviz')
    dron01_description = pathlib.Path(os.path.join(dron_package_dir, 'resource', 'MRS_dron01.urdf')).read_text()
    dron02_description = pathlib.Path(os.path.join(dron_package_dir, 'resource', 'MRS_dron02.urdf')).read_text()
    dron03_description = pathlib.Path(os.path.join(dron_package_dir, 'resource', 'MRS_dron03.urdf')).read_text()
    dron04_description = pathlib.Path(os.path.join(dron_package_dir, 'resource', 'MRS_dron04.urdf')).read_text()
    khepera_description = pathlib.Path(os.path.join(robot_package_dir, 'resource', 'kheperaiv.urdf')).read_text()
    robot_description = pathlib.Path(os.path.join(robot_package_dir, 'resource', 'turtlebot3burger.urdf')).read_text()
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yml')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    webots = WebotsLauncher(
        world=os.path.join(swarm_package_dir, 'worlds', 'RoboticPark_4cf_3kh_1tb.wbt')
    )

    mappings = [('/TurtleBot3Burger/gps', '/turtlebot01/driver/gps')]

    turtlebot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'turtlebot01',
                        'WEBOTS_ROBOT_NAME': 'turtlebot01'},
        parameters=[
            {'robot_description': robot_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings
    )

    ros2_supervisor = Ros2SupervisorLauncher()

    turtlebot_task = Node(
        package='uned_kheperaiv_task',
        executable='distance_based_formation_control',
        output='screen',
        name='formation_control',
        parameters=[
            {'use_sim_time': use_sim_time},
            {"config_file": 'path'},
            {"robot": 'turtlebot01'},
            {"agents": 'khepera01, khepera03'},
            {"distance": '0.4, 0.4'},
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

    dron01_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='driver',
        namespace='dron01',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron01',
                        'WEBOTS_ROBOT_NAME': 'dron01'},
        parameters=[
            {'robot_description': dron01_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron02_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='driver',
        namespace='dron02',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron02',
                        'WEBOTS_ROBOT_NAME': 'dron02'},
        parameters=[
            {'robot_description': dron02_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron03_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='driver',
        namespace='dron03',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron03',
                        'WEBOTS_ROBOT_NAME': 'dron03'},
        parameters=[
            {'robot_description': dron03_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    dron04_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='driver',
        namespace='dron04',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'dron04',
                        'WEBOTS_ROBOT_NAME': 'dron04'},
        parameters=[
            {'robot_description': dron04_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ]
    )

    swarm_driver = Node(
        package='uned_swarm_driver',
        executable='swarm_driver',
        output='screen',
        name='swarm_driver',
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robots': 'dron01, dron02, dron03, dron04, khepera01, khepera02, khepera03, turtlebot01'},
        ])

    robot01_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='driver',
        namespace='khepera01',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'khepera01',
                        'WEBOTS_ROBOT_NAME': 'khepera01'},
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[
            {'robot_description': khepera_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ],
    )

    robot01_task = Node(
        package='uned_kheperaiv_task',
        executable='distance_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='khepera01',
        remappings=[
            ('/khepera01/khepera02/local_pose', '/khepera02/local_pose'),
            ('/khepera01/khepera03/local_pose', '/khepera03/local_pose'),
            ('/khepera01/turtlebot01/local_pose', '/turtlebot01/local_pose'),
            ('/khepera01/dron01/local_pose', '/dron01/local_pose'),
            ('/khepera01/dron03/local_pose', '/dron03/local_pose'),
            ('/khepera01/swarm/status', '/swarm/status'),
            ('/khepera01/swarm/order', '/swarm/order')],
        parameters=[
            {'use_sim_time': use_sim_time},
            {"config_file": 'path'},
            {"robot": 'khepera01'},
            {"agents": 'khepera02, khepera03, turtlebot01, dron01, dron03'},
            {"distance": '0.6, 0.6, 0.4, 1.2, 1.2'},
        ]
    )

    robot02_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='driver',
        namespace='khepera02',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'khepera02',
                        'WEBOTS_ROBOT_NAME': 'khepera02'},
        parameters=[
            {'robot_description': khepera_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ],
    )

    robot02_task = Node(
        package='uned_kheperaiv_task',
        executable='distance_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='khepera02',
        remappings=[
            ('/khepera02/khepera01/local_pose', '/khepera01/local_pose'),
            ('/khepera02/khepera03/local_pose', '/khepera03/local_pose'),
            ('/khepera02/turtlebot01/local_pose', '/turtlebot01/local_pose'),
            ('/khepera02/dron01/local_pose', '/dron01/local_pose'),
            ('/khepera02/dron02/local_pose', '/dron02/local_pose'),
            ('/khepera02/swarm/status', '/swarm/status'),
            ('/khepera02/swarm/order', '/swarm/order')],
        parameters=[
            {'use_sim_time': use_sim_time},
            {"config_file": 'path'},
            {"robot": 'khepera02'},
            {"agents": 'khepera01, khepera03, dron01, dron02'},
            {"distance": '0.6, 0.6, 1.2, 1.2'},
        ]
    )

    robot03_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        name='driver',
        namespace='khepera03',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'khepera03',
                        'WEBOTS_ROBOT_NAME': 'khepera03'},
        parameters=[
            {'robot_description': khepera_description,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
        ],
    )

    robot03_task = Node(
        package='uned_kheperaiv_task',
        executable='distance_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='khepera03',
        remappings=[
            ('/khepera03/khepera02/local_pose', '/khepera02/local_pose'),
            ('/khepera03/khepera01/local_pose', '/khepera01/local_pose'),
            ('/khepera03/turtlebot01/local_pose', '/turtlebot01/local_pose'),
            ('/khepera03/dron04/local_pose', '/dron04/local_pose'),
            ('/khepera03/dron02/local_pose', '/dron02/local_pose'),
            ('/khepera03/swarm/status', '/swarm/status'),
            ('/khepera03/swarm/order', '/swarm/order')],
        parameters=[
            {'use_sim_time': use_sim_time},
            {"config_file": 'path'},
            {"robot": 'khepera03'},
            {"agents": 'khepera01, khepera02, turtlebot01, dron02, dron04'},
            {"distance": '0.6, 0.6, 0.4, 1.2, 1.2'},
        ]
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
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
        arguments=['-d', default_rviz_config_path],
        parameters=[
            {'use_sim_time': use_sim_time}
        ]

    )

    vicon_node = Node(
        package='uned_vicon_sim', 
        executable='vicon_webots',
        name='vicon_webots',
        output='screen',
        #arguments=['--ros-args', '--log-level', 'info'],
        parameters=[
            {'use_sim_time': use_sim_time},
            {"agents": 'turtlebot01, khepera01, khepera02, khepera03, dron01, dron02, dron03, dron04'},
        ]
    )


    return LaunchDescription([
        # joint_state_broadcaster_spawner,
        # diffdrive_controller_spawner,
        webots,
        ros2_supervisor,
        dron01_driver,
        dron02_driver,
        dron03_driver,
        dron04_driver,
        swarm_driver,
        robot01_driver,
        robot01_task,
        robot02_driver,
        robot02_task,
        robot03_driver,
        robot03_task,
        robot_state_publisher,
        turtlebot_driver,
        turtlebot_task,
        # footprint_publisher,
        rqt_node,
        rviz_node,
        # vicon_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])