from setuptools import setup
import os
from glob import glob

package_name = 'ethical_robot_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='A simulation of a ground robot facing ethical dilemmas',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = ethical_robot_sim.robot_controller:main',
            'extended_robot_controller = ethical_robot_sim.extended_robot_controller:main',
            'auto_decision_maker = ethical_robot_sim.auto_decision_maker:main',
            'decision_engine = ethical_robot_sim.decision_engine:main',
            'perception_node = ethical_robot_sim.perception:main',
        ],
    },
)