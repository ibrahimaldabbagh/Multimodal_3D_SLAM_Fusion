from setuptools import setup

package_name = 'pressure_sensor_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aldabbag',
    maintainer_email='“eng.ibrahim.aldabbagh@gmail.com”',
    description='ROS2 Node for BMP390 Pressure Sensor on Jetson Orin Nano',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pressure_sensor_node= pressure_sensor_node.pressure_sensor_node:main',
        ],
    },
)
