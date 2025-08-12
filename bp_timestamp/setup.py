from setuptools import find_packages, setup
import os
from glob import glob
# When you want to use file that is outside source code folder , we need to speciify path
# in this setup for ROS build tool to recognize

package_name = 'bp_timestamp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='toenen',
    maintainer_email='toenen@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sub_timestamper = bp_timestamp.bp_stamp:main' 
        ],
    },
)
