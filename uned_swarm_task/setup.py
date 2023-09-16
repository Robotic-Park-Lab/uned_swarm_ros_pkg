from setuptools import setup

package_name = 'uned_swarm_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francisco José Mañas Álvarez',
    maintainer_email='fjmanas@dia.uned.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'centralized_formation_controller = uned_swarm_task.centralized_formation_controller:main',
            'system_identification = uned_swarm_task.system_identification:main',
            'open_loop_signal = uned_swarm_task.open_loop_signal:main',
            'formation_controller = uned_swarm_task.formation_controller:main',
            'turtlebot_distance_control = uned_swarm_task.turtlebot_distance_control:main',
        ],
    },
)
