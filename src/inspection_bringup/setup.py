from setuptools import setup
import os
from glob import glob

package_name = 'inspection_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raquel',
    maintainer_email='raquel@example.com',
    description='Launch files for inspection robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={},
)

