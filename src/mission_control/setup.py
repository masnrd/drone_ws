from setuptools import find_packages, setup

package_name = 'mission_control'

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
    maintainer='rye',
    maintainer_email='ryantoh99@gmail.com',
    description='Mission Control. Handles communications with the drone and provides the user interface.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_control_node = mission_control.mission_control_node:main'
        ],
    },
)
