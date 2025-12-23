from setuptools import setup
import glob

package_name = 'bag_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student Xu',
    maintainer_email='xuchunbobo520@gmail.com',
    description='Python rosbag2 recorder node with bag split and topic config support.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_recorder_node = bag_recorder.bag_recorder_node:main',
        ],
    },
)
