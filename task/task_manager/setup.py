from setuptools import find_packages, setup

package_name = 'task_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch", ['launch/logger_launcher.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tonib',
    maintainer_email='68343004+tb18003@users.noreply.github.com',
    description='TODO: Package description',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_executor_node = task_manager.task_executor_node:main'
        ],
    },
)
