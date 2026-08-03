from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'warehouse_fleet_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Warehouse Fleet System',
    maintainer_email='warehouse@industrial.com',
    description='Autonomous multi-robot warehouse fleet management system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_planner = warehouse_fleet_manager.mission_planner_node:main',
            'mission_validator = warehouse_fleet_manager.mission_validator_node:main',
            'mission_coordinator = warehouse_fleet_manager.mission_coordinator_node:main',
            'mission_executor = warehouse_fleet_manager.mission_executor_node:main',
            'battery_manager = warehouse_fleet_manager.battery_manager_node:main',
            'charging_manager = warehouse_fleet_manager.charging_manager_node:main',
            'fleet_dashboard = warehouse_fleet_manager.fleet_dashboard_node:main',
        ],
    },
)
