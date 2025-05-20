from setuptools import find_packages, setup

package_name = 'monitor_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('share/' + package_name + "/assets", ['splash.png']),
        #('share/' + package_name + "/assets/face-recognition", ['face-recognition/done.svg', 'face-recognition/failure.svg',
        #                                                        'face-recognition/runnning.svg', 'face-recognition/pending.svg']),
        #('share/' + package_name + "/assets/move", ['move/done.svg', 'move/failure.svg',
        #                                                        'move/runnning.svg', 'move/pending.svg']),
        #('share/' + package_name + "/assets/talk", ['talk/done.svg', 'talk/failure.svg',
        #                                                        'talk/runnning.svg', 'talk/pending.svg']),                                                            
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
