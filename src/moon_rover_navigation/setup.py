from setuptools import setup

package_name = 'moon_rover_navigation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/simulation.launch.py',
            'launch/planner.launch.py',
            'launch/controller.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/planners/astar.yaml',
            'config/planners/hybrid_astar.yaml',
            'config/planners/rrt_star.yaml',
            'config/controllers/dwa.yaml',
            'config/controllers/pure_pursuit.yaml',
            'config/terrain/lunar_costs.yaml',
        ]),
    ],
    install_scripts=['bin/planner_node', 'bin/controller_node'],
    zip_safe=True,
    maintainer='Lunar Rover Team',
    maintainer_email='lunar-rover@example.com',
    description='Lunar Rover Autonomous Navigation Framework',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node = moon_rover_navigation.planner_node:main',
            'controller_node = moon_rover_navigation.controller_node:main',
            'localization_node = moon_rover_navigation.localization_node:main',
            'map_server_node = moon_rover_navigation.map_server_node:main',
            'visualization_node = moon_rover_navigation.visualization_node:main',
        ],
    },
)