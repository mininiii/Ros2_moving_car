import os
from glob import glob
from setuptools import setup

package_name = 'moving_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name), glob('rviz/*.rviz')),
        (os.path.join('share', package_name), glob('meshes/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='didals0521',
    maintainer_email='didals0521@gmail.con',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = moving_car.state_publisher:main'
        ],
    },
)