from setuptools import find_packages, setup

package_name = 'warehouse_inspection'

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
    description='Shelf inspection, detection, and anomaly tracking for warehouse robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inspection_detector = warehouse_inspection.inspection_detector_node:main',
            'inspection_tracker  = warehouse_inspection.inspection_tracker_node:main',
        ],
    },
)
