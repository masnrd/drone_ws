from setuptools import find_packages, setup
from pathlib import Path

package_name = 'sensor_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f"share/{package_name}/simmaps/", [str(p) for p in Path("simmaps").glob("*.simmap")])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rye',
    maintainer_email='ryantoh99@gmail.com',
    description='Package for sensor interfacing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = sensor_node.sensor_node:main'
        ],
    },
)
