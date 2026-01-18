from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pipeline_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'),
        glob('launch/*.py')),
        
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jesus-timaure',
    maintainer_email='jesus-timaure@todo.todo',
    description='Sensor fusion, localization and movement control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pseudo_dvl = pipeline_follower.pseudo_dvl:main',
            'imu_covariance_wrapper = pipeline_follower.imu_covariance_wrapper:main',
            'pseudo_depth = pipeline_follower.pseudo_depth:main',
            'landmark_simulator = pipeline_follower.landmark_simulator:main',
            'trajectory_gen = pipeline_follower.trajectory_gen:main',
            'path_follower = pipeline_follower.path_follower:main'
        ],
    },
)
