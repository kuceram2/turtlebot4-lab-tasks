from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'my_tb_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guest',
    maintainer_email='skokankucera@seznam.cz',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_prefix_publisher = my_tb_nodes.tf_prefix_publisher:main',
            'scan_republisher = my_tb_nodes.scan_republisher:main',
            'nav_to_pose = my_tb_nodes.nav_to_pose:main',
            'nav_through_poses = my_tb_nodes.nav_through_poses:main',
            'relative_movements = my_tb_nodes.rleative_movements:main',
            'test_script = my_tb_nodes.test_script:main',
            'qr_commands = my_tb_nodes.qr_commands:main'
        ],
    },
)
