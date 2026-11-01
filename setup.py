from setuptools import find_packages, setup

package_name = 'parking_perception'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/project_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team A3',
    maintainer_email='xiaobu.wan@outlook.com',
    description='Autonomous parking with sign detection for F1TENTH - Team A3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'front_in_parking_node = parking_perception.nodes.front_in_parking_node:main',
            'sign_detection_node = parking_perception.nodes.sign_detection_node:main',
        ],
    },
)
