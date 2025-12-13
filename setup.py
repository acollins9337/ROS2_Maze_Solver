from setuptools import find_packages, setup

package_name = 'sv_maze_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/nav_maze.launch.py']),
    ],
    install_requires=['setuptools','numpy'],
    zip_safe=True,
    maintainer='MAINTAINER',
    maintainer_email='EMAIL',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'sign_id = sv_maze_nav.sign_id:main',
        'nav_maze = sv_maze_nav.nav_maze:main',
        'sensors = sv_maze_nav.sensors:main',
        'move_robot_client = sv_maze_nav.move_robot_client:main',
        ],
    },
)
