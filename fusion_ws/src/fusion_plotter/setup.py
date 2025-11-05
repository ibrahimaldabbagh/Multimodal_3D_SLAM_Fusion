from setuptools import find_packages, setup

package_name = 'fusion_plotter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aldabbag',
    maintainer_email='eng.ibrahim.aldabbagh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'fusion_plotter_node = fusion_plotter.fusion_plotter_node:main',
            'rviz_visualizer_node = fusion_plotter.rviz_visualizer_node:main',
        ],
    },
)
