import os
import pathlib
import launch
import yaml
from yaml.loader import SafeLoader
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix


def get_ros2_nodes(*args):
    general_package_dir = get_package_share_directory('uned_swarm_config')
    config_path = os.path.join(general_package_dir, 'resources', 'AD09c_RoboticPark.yaml')
    rviz_config_path = os.path.join(general_package_dir, 'rviz', 'AD09_RoboticPark.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    robot_node_list = []
    physical_crazyflie_list = ''
    physical_khepera_list = ''

    
    with open(config_path) as f:
        data = yaml.load(f, Loader=SafeLoader)
        for key, robot in data.items():
            print("###  "+robot['name']+"  ###")
            
            if not robot['type'] == 'virtual' and robot['description'] == 'crazyflie':
                physical_crazyflie_list += ', '+robot['name']
            if not robot['type'] == 'virtual' and robot['description'] == 'khepera':
                physical_khepera_list += ', '+robot['name']

    print(physical_crazyflie_list)
    robot_node_list.append(Node(
            package='uned_swarm_task',
            executable='centralized_formation_controller',
            name='formation_controller',
            output='screen',
            shell=True,
            emulate_tty=True,
            parameters=[
                {'config_file': config_path},
                {'use_sim_time': use_sim_time},
            ]
        )
    )

    robot_node_list.append(Node(
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
    )

    print(physical_khepera_list)
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

    robot_node_list.append(Node(
            package='rqt_gui',
            executable='rqt_gui',
            name='interface',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
        )
    )

    robot_node_list.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
            arguments=['-d', rviz_config_path],
        )
    )

    robot_node_list.append(Node(
            package='measure_process_ros2_pkg',
            executable='measure_process',
            name='benchmark',
            output='screen',
            parameters=[{
                'process_name' : 'webots-bin, driver, ros2, swarm_driver, rviz2, kheperaIV_clien, centralized_for',
                'process_period' : 0.5},
            ],
        )
    )

    return robot_node_list

def generate_launch_description():    

    return LaunchDescription([
    ] + get_ros2_nodes())