from setuptools import find_packages, setup
from glob import glob

package_name = 'miata_hw4'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/f23_robotics_1_launch.py']))

data_files.append(('share/' + package_name + '/worlds', [
    'worlds/f23_robotics_lab.wbt', 
]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/resource', [
    'resource/turtlebot_webots.urdf',
    'resource/ros2control.yml',
]))

data_files.extend([
    ('share/' + package_name + '/protos', glob('protos/*.proto')),
    ('share/' + package_name + '/images', glob('images/*.png')),
    ('share/' + package_name + '/worlds', glob('worlds/*.wbt')),
])

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='Anastasia Spencer, Lilly Eide, Jeongbin Son',
    maintainer_email='',
    description='hw4 cs460',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros'],    
        'console_scripts': ['miata_hw4 = miata_hw4.miata_hw4:main']
    },

)
