import os
import pathlib
import launch
import yaml
import random
from yaml.loader import SafeLoader
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch.actions import ExecuteProcess

def generate_launch_description():
    general_package_dir = get_package_share_directory('uned_swarm_config')
    config_path = os.path.join(general_package_dir, 'resources', 'AC30_RoboticPark_sim.yaml')
    rviz_config_path = os.path.join(general_package_dir, 'rviz', 'AC30_RoboticPark.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    crazyflie_model_dir = get_package_share_directory('tello_description')

    world_path = os.path.join(general_package_dir, 'worlds', 'Empty.world')
    
    gazebo = ExecuteProcess(cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', '--ros-args',
        ], output='screen'
    )

    robot_node_list = []
    physical_crazyflie_list = ''
    physical_khepera_list = ''
    j = 1
    with open(config_path) as f:
        data = yaml.load(f, Loader=SafeLoader)
        for key, robot in data.items():
            print("###  "+robot['name']+"  ###")
            
            individual_config_path = os.path.join(general_package_dir, 'resources', robot['config_path'])

            if not robot['type'] == 'physical' and robot['description'] == 'khepera':
                urdf_path = os.path.join(crazyflie_model_dir, 'urdf', robot['name']+'.urdf')
                pose = robot['pose'].split(', ')
                
                robot_node_list.append(Node(package='tello_gazebo', executable='inject_entity.py', output='screen',
                                            arguments=[urdf_path, pose[0], pose[1], '0.05', '0']),
                )
                robot_node_list.append(Node(package='uned_kheperaiv_task', 
                                            executable='gazebo_driver',
                                            name='driver',
                                            namespace=robot['name'],
                                            output='screen',
                                            parameters=[{   'use_sim_time' : use_sim_time,
                                                            'config_file' : individual_config_path,
                                                            'robot' : robot['name'],
                                                            'type' : robot['type']},
                                        ]),
                    )
                
                if not robot['type'] == 'virtual':
                    physical_khepera_list += ', '+robot['name']
            
            if not robot['type'] == 'physical' and robot['description'] == 'crazyflie':
                
                urdf_path = os.path.join(crazyflie_model_dir, 'urdf', robot['name']+'.urdf')
                pose = robot['pose'].split(', ')
                robot_node_list.append(Node(package='tello_gazebo', executable='inject_entity.py', output='screen',
                                            arguments=[urdf_path, pose[0], pose[1], '0', '0']),
                )
                robot_node_list.append(Node(package='uned_swarm_driver', 
                                            executable='tello_gazebo_driver',
                                            name='driver',
                                            namespace=robot['name'],
                                            output='screen',
                                            parameters=[{   'use_sim_time' : use_sim_time,
                                                            'config_file' : individual_config_path,
                                                            'robot' : robot['name'],
                                                            'type' : robot['type']},
                                        ]),
                )
                if not robot['type'] == 'virtual':
                    physical_crazyflie_list += ', '+robot['name']
                
                    

    # print(physical_crazyflie_list)

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

    cpu_measure = Node(
        package='measure_process_ros2_pkg',
        executable='measure_process',
        name='benchmark',
        output='screen',
        parameters=[{
             'process_name' : 'gzserver, ros2, gazebo_driver, swarm_driver, rviz2, kheperaIV_clien, gzclient, tello_gazebo_dr',
             'process_period' : 0.5},
        ],
    )
    
    ld = LaunchDescription()
    ld.add_action(gazebo)
    ld.add_action(rqt_node)
    ld.add_action(rviz_node)
    ld.add_action(swarm_node)
    ld.add_action(cpu_measure)
    for robot in robot_node_list:
        ld.add_action(robot)

    return ld 
        