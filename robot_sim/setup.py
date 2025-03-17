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
        ('share/' + package_name + "/launch", ['launch/launch.py']),
        ('share/' + package_name + "/config", ['config/topics_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='tb18ms@gmail.com',
    description='TODO: Package description',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robot_sim_node = robot_sim.robot_sim_node:main"
        ],
    },
)
