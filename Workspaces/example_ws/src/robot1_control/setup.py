from setuptools import find_packages, setup

package_name = 'robot1_control'

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
    maintainer='robousr',
    maintainer_email='luisfdopapu@gmail.com',
    description='Robot 1 (XY plane) control package - RRR manipulator in horizontal plane',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          "controller_manager = robot1_control.controller_manager:main",
          "hardware_interface = robot1_control.hardware_interface:main",
          "manipulator_controller = robot1_control.manipulator_controller:main"
        ],
    },
)