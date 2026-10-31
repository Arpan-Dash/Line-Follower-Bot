from setuptools import setup
from glob import glob

package_name = 'ros2_line_follower'

setup(
    name=package_name,
    version='1.0',
    packages=['ros2_line_follower_pkg'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/config', glob('config/*.rviz') + glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ahmed M. Abbas',
    maintainer_email='ahmed@example.com',
    description='Line follower robot in a beautiful indoor room — Gazebo Harmonic + ROS 2 Jazzy.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower = ros2_line_follower_pkg.line_follower:main',
            'error_plotter = ros2_line_follower_pkg.error_plotter:main',
        ],
    },
)
