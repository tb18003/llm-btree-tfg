from setuptools import find_packages, setup

package_name = 'robot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch", ['launch/tb3_launch.py']),
        ('share/' + package_name + "/assets", ['assets/map.yaml', 'assets/map.pgm', 
                                               'assets/mapir_lab_world.model', 'assets/sancho_found.mp3',
                                               'assets/sancho_finished.mp3']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='tb18ms@gmail.com',
    description='A robot simulator (topics and services) with GUI',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "tts_sim_node = robot_sim.tts_sim_node:main",
            "whisper_sim_node = robot_sim.whisper_sim_node:main",
        ],
    },
)
