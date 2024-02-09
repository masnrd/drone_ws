import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'full_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join("share", package_name, "launch"),
         glob(os.path.join("launch", "*launch.[pxy][yma]*")))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rye',
    maintainer_email='ryantoh99@gmail.com',
    description='Launches multiple drones at once for simulation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    }
)
