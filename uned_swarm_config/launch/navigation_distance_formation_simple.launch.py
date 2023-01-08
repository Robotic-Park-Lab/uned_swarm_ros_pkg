import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    swarm_package_dir = get_package_share_directory('uned_swarm_config')
    dron_package_dir = get_package_share_directory('uned_crazyflie_webots')
    robot_package_dir = get_package_share_directory('uned_kheperaiv_webots')
    
    dron_config_path = os.path.join(swarm_package_dir, 'resources', 'crazyflie_n2_formation_distance.yaml')
    khepera_config_path = os.path.join(swarm_package_dir, 'resources', 'khepera_n3_formation_distance.yaml')
    turtlebot_config_path = os.path.join(swarm_package_dir, 'resources', 'turtlebot_n1_formation_distance.yaml')
    rviz_config_path = os.path.join(swarm_package_dir, 'rviz', 'distance_formation.rviz')

    hostname = '192.168.0.17'
    buffer_size = 200
    topic_namespace = 'vicon'

    swarm_node = Node(
        package='uned_crazyflie_driver',
        executable='swarm_driver',
        name='swarm',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'first_uri': 'radio://0/80/2M/E7E7E7E701'},
            {'n': 2},
            {'control_mode': 'HighLevel, HighLevel'},
            {'controller_type': 'EventBased, Continuous'},
            {'config': dron_config_path}
        ]
    )

    robot01_node = Node(
        package='uned_kheperaiv_driver',
        executable='kheperaIV_client_driver',
        name='driver',
        namespace='khepera01',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'agent_ip': '192.168.0.21'},
            {'port_number': 50000},
            {'id': 'khepera01'},
            {'config': khepera_config_path}
        ])

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
            {"config_file": 'path'},
            {"robot": 'khepera01'},
            {"agents": 'khepera02'},
            {"agents": 'khepera02, turtlebot01, dron01, dron02'},
            {"distance": '0.6, 0.4, 1.2, 1.2'},
        ]
    )

    robot02_node = Node(
        package='uned_kheperaiv_driver',
        executable='kheperaIV_client_driver',
        namespace='khepera02',
        name='driver',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'agent_ip': '192.168.0.22'},
            {'port_number': 50000},
            {'id': 'khepera02'},
            {'config': khepera_config_path}
        ])

    robot02_task = Node(
        package='uned_kheperaiv_task',
        executable='distance_based_formation_control',
        output='screen',
        name='formation_control',
        namespace='khepera02',
        remappings=[
            ('/khepera02/khepera01/local_pose', '/khepera01/local_pose'),
            ('/khepera02/turtlebot01/local_pose', '/turtlebot01/local_pose'),
            ('/khepera02/dron01/local_pose', '/dron01/local_pose'),
            ('/khepera02/dron02/local_pose', '/dron02/local_pose'),
            ('/khepera02/swarm/status', '/swarm/status'),
            ('/khepera02/swarm/order', '/swarm/order')],
        parameters=[
            {"config_file": 'path'},
            {"robot": 'khepera02'},
            {"agents": 'khepera01, turtlebo01, dron01, dron02'},
            {"distance": '0.6, 0.4, 1.2, 1.2'},
        ]
    )

    turtlebot01_node = Node(
        package='uned_swarm_driver',
        executable='turtlebot_driver',
        output='screen',
        name='driver',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'id': 'turtlebot01'},
            {'config': turtlebot_config_path}
        ],
    )

    turtlebot01_task = Node(
        package='uned_kheperaiv_task',
        executable='distance_based_formation_control',
        output='screen',
        name='formation_control',
        parameters=[
            {"config_file": 'path'},
            {"robot": 'turtlebot01'},
            {"agents": 'khepera01, khepera02'},
            {"distance": '0.4, 0.4'},
        ]
    )

    turtlebot_init_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'map'],
    )

    rqt_node = Node(
        package='rqt_gui',
        executable='rqt_gui',
        name='interface',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    vicon_node = Node(
        package='vicon_receiver',
        executable='vicon_client',
        name='vicon_node',
        parameters=[
            {'hostname': hostname, 'buffer_size': buffer_size, 'namespace': topic_namespace}
        ])


    return LaunchDescription([
        swarm_node,
        robot01_node,
        robot01_task,
        robot02_node,
        robot02_task,
        turtlebot01_node,
        turtlebot01_task,
        turtlebot_init_map,
        rqt_node,
        rviz_node,
        # vicon_node,
    ])