import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'medusa'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/' + package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share/' + package_name, 'meshes'), glob(os.path.join('meshes', '*.*'))),
        (os.path.join('share/' + package_name, 'urdf'), glob(os.path.join('urdf', '*.*'))),
        (os.path.join('share/' + package_name, 'config'), glob(os.path.join('config', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juan',
    maintainer_email='juan_jose.moreno@uao.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'control = medusa.control:main',
        'mando = medusa.mando:main',
        'gui = medusa.gui:main',
        'esp32_imu_bridge = medusa.esp32_imu_bridge:main',
        'serial_IMU = medusa.serial_IMU:main',
        ],
    },
)
