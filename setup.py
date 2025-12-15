from setuptools import setup
from glob import glob
import os

package_name = 'way_finder'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='barna154',
    maintainer_email='simonbaran154@gmail.com',
    description='Way finder launch files',
    license='GNU General Public License v3.0',
)