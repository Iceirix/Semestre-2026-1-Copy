from setuptools import find_packages, setup
from glob import glob

package_name = 'robot2_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # IMPORTANTE: Instalar archivos URDF
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        # IMPORTANTE: Instalar archivos RViz
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robousr',
    maintainer_email='luisfdopapu@gmail.com',
    description='Robot 2 description package - RRR with rotating base',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)