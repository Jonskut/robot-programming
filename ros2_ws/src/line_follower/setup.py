from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'line_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        # Install package.xml
        ('share/ament_index/resource_index/packages', ['package.xml']),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonatan Tevaniemi',
    maintainer_email='jonatantevaniemi@outlook.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_follower_node = line_follower.line_follower_node:main',
        ],
    },
)
