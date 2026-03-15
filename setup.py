from setuptools import setup

package_name = 'gps_map_viewer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 node that reads GPS from topic and displays it on OSM map',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'gps_map_node = gps_map_viewer.gps_map_node:main',
    ],
},
)
