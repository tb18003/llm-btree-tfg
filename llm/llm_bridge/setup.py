from setuptools import find_packages, setup

package_name = 'llm_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py', 'launch/local_launch.py', 'launch/test_launch.py']),
        ('share/' + package_name + '/config', ['config/test_params.yaml', 'config/params.yaml', 'config/local_params.yaml', 'config/sys.prompt', 'config/sys.sancho.prompt','config/.env']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tonib',
    maintainer_email='68343004+tb18003@users.noreply.github.com',
    description='TODO: Package description',
    license='GPL3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "llm_service_node = llm_bridge.llm_bridge_service:main",
            "llm_bridge_node = llm_bridge.llm_bridge_node:main",
            "llm_bridge_test_node = llm_bridge.llm_bridge_test:main",
        ],
    },
)
