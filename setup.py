from setuptools import setup
import os

package_name = 'diffdrive'

data_files = []
data_files.append(
    ('share/ament_index/resource_index/packages', ['resource/' + package_name])
)
data_files.append(
    ('share/' + package_name, ['package.xml'])
)

data_files.append(
    (
        os.path.join('share', package_name, 'launch'), 
        [os.path.join('launch', 'pioneer_tracker_demo_launch.py')]
    )
)

data_files.append(
    (
        os.path.join('share', package_name, 'worlds'),
        [os.path.join('worlds', 'new_world.wbt')]
    )
)

data_files.append(
    (
        os.path.join('share', package_name, 'resource'), 
        [os.path.join('resource', 'pioneer.urdf')]
    )
)

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='imb537',
    maintainer_email='isinmertbalci@gmail.com',
    description='This package is a collection of files to drive '+
        'a differential drive robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_state_publisher = {}.robot_state_publisher:main'.format(package_name),
            'pioneer_driver = {}.pioneer_driver:main'.format(package_name),
            'trajectory_generator = {}.trajectory_generator:main'.format(package_name),
            'trajectory_tracker = {}.trajectory_tracker:main'.format(package_name), 
            'lyapunov_tracker = {}.lyapunov_tracker:main'.format(package_name)
        ],
    },
)
