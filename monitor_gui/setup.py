from setuptools import find_packages, setup
from glob import glob

package_name = 'monitor_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/assets', ['assets/splash.png','assets/icon.png']),
        ('share/' + package_name + '/assets/move', glob('assets/move/*.svg')),
        ('share/' + package_name + '/assets/talk', glob('assets/talk/*.svg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tonib',
    maintainer_email='68343004+tb18003@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor_node = monitor_gui.monitor_node:main'
        ],
    },
)
