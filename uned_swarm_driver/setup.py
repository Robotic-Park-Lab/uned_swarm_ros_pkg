from setuptools import setup

package_name = 'uned_swarm_driver'

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
    license='BSD-3-Clause license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swarm_driver = uned_swarm_driver.swarm_driver:main',
            'turtlebot_driver = uned_swarm_driver.turtlebot_driver:main',
            'tello_gazebo_driver = uned_swarm_driver.tello_gazebo_driver:main'
        ],
    },
)
