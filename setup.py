from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'cibo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HarukiIsono',
    maintainer_email='haruki.isono861@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'top_camera_node = cibo.top_camera:main',
            'front_camera_node = cibo.front_camera:main',
            'chew_counter_node = cibo.chew_counter:main',
        ],
    },
)
