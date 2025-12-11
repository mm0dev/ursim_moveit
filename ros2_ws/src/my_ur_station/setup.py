from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'my_ur_station'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='126759861+mm0dev@users.noreply.github.com',
    description='UR Station Package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_mover = my_ur_station.simple_mover:main',
            'moveit_mover = my_ur_station.moveit_mover:main',
            'stress_test = my_ur_station.stress_test:main',
            'unchained_stress_test = my_ur_station.unchained_stress_test:main',
            # --- THIS WAS MISSING ---
            'final_stable_test = my_ur_station.final_stable_test:main',
            'keyboard_teleop = my_ur_station.keyboard_teleop:main',
            'keyboard_teleop_servo = my_ur_station.keyboard_teleop_servo:main',
            'keyboard_teleop_direct = my_ur_station.keyboard_teleop_direct:main',
            'cartesian_teleop = my_ur_station.cartesian_teleop:main',
        ],
    },
)