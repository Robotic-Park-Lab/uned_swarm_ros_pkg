import os
import pathlib
import launch
import yaml
from yaml.loader import SafeLoader
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.utils import controller_url_prefix

def generate_launch_description():
    general_package_dir = get_package_share_directory('uned_swarm_config')
    config_path = os.path.join(general_package_dir, 'resources', 'AC10_RoboticPark.yaml')
    rviz_config_path = os.path.join(general_package_dir, 'rviz', 'AC10_RoboticPark.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    dron_package_dir = get_package_share_directory('uned_crazyflie_webots')
    webots = WebotsLauncher(
        world=os.path.join(general_package_dir, 'worlds', 'AC10_RoboticPark.wbt')
    )
    ros2_supervisor = Ros2SupervisorLauncher()

    robot_node_list = []
    physical_crazyflie_list = ''
    physical_khepera_list = ''


    with open(config_path) as f:
        data = yaml.load(f, Loader=SafeLoader)
        for key, robot in data.items():
            print("###  "+robot['name']+"  ###")
            
            if robot['description'] == 'crazyflie':
                robot_description = pathlib.Path(os.path.join(general_package_dir, 'resources', 'crazyflie.urdf')).read_text()
            else:
                robot_description = pathlib.Path(os.path.join(general_package_dir, 'resources', 'kheperaiv.urdf')).read_text()
            
            individual_config_path = os.path.join(general_package_dir, 'resources', robot['config_path'])

            if not robot['type'] == 'physical':
                robot_node_list.append(Node(package='webots_ros2_driver', 
                                            executable='driver', 
                                            output='screen',
                                            name=robot['name'],
                                            additional_env={'WEBOTS_ROBOT_NAME': robot['name'],
                                                            'WEBOTS_CONTROLLER_URL': controller_url_prefix() + robot['name'],
                                                            'WEBOTS_ROBOT_CONFIG_FILE': individual_config_path,
                                                            'WEBOTS_ROBOT_ROLE': robot['type']},
                                            parameters=[{   'robot_description': robot_description,
                                                            'use_sim_time': use_sim_time,
                                                            'set_robot_state_publisher': True},
                                            ]
                                        )
                )
            if not robot['type'] == 'virtual' and robot['description'] == 'crazyflie':
                physical_crazyflie_list += ', '+robot['name']
            if not robot['type'] == 'virtual' and robot['description'] == 'khepera':
                physical_khepera_list += ', '+robot['name']
    
    print(physical_crazyflie_list)

    swarm_node = Node(
        package='uned_crazyflie_driver',
        executable='swarm_driver',
        name='swarm',
        output='screen',
        shell=True,
        emulate_tty=True,
        parameters=[
            {'enviroment': 'mrs'},
            {'config': config_path},
            {'robots': physical_crazyflie_list[2:]}
        ]
    )
    aux = physical_khepera_list.split(', ')
    for robot in aux:
        if not robot == '':
            individual_config_path = os.path.join(general_package_dir, 'resources', data[robot]['config_path'])
            robot_node_list.append(Node(package='uned_kheperaiv_driver',
                                        executable='kheperaIV_client_driver',
                                        name='driver',
                                        namespace=data[robot]['name'],
                                        output='screen',
                                        shell=True,
                                        emulate_tty=True,
                                        parameters=[{'id': data[robot]['name']},
                                                    {'config': individual_config_path}
                                        ]
                                    )
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
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
        arguments=['-d', rviz_config_path],
    )

    ros2_close = launch.actions.RegisterEventHandler(
                    event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                    )
                )
    
    ld = LaunchDescription()
    ld.add_action(webots)
    ld.add_action(ros2_supervisor)
    ld.add_action(rqt_node)
    ld.add_action(rviz_node)
    ld.add_action(swarm_node)
    for robot in robot_node_list:
        ld.add_action(robot)
        
    ld.add_action(ros2_close)

    return ld 
        
