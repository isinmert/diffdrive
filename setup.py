from setuptools import setup
import os
import glob

def save_folder_to_datafiles(folder_name:str, data_files_:list) -> None:
    """
    Save all the files under a given folder name into data files in the correct format.
    All the files will be copied by preserving the package structure.
    """
    for (dir, _, files) in os.walk(folder_name):
        SAVE_DIR = os.path.join('share', package_name, dir)
        SAVE_LIST = []
        for filename in files:
            if filename != package_name:
                SAVE_LIST.append(os.path.join(dir, filename))
        data_files_.append((SAVE_DIR, SAVE_LIST))
    return 

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
        [os.path.join('launch', 'pioneer_tracker_demo_launch.py'),
         os.path.join('launch', 'husky_launch.py')]
    )
)

data_files.append(
    (
        os.path.join('share', package_name, 'worlds'),
        [os.path.join('worlds', 'new_world.wbt'),
         os.path.join('worlds', 'gazebo_husky.world')]
    )
)

data_files.append(
    (
        os.path.join('share', package_name, 'resource'), 
        [os.path.join('resource', 'pioneer.urdf')]
    )
)

save_folder_to_datafiles('models', data_files)
save_folder_to_datafiles('protos', data_files)

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
            'feedback_linearize_tracker = {}.feedback_linearize_tracker:main'.format(package_name), 
            'lyapunov_tracker = {}.lyapunov_tracker:main'.format(package_name), 
            'husky_spawner = {}.husky_spawner:main'.format(package_name)
        ],
    },
)
