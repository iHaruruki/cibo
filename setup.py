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
            'body_face_node = cibo.body_face:main',
            'hand_node = cibo.hand:main',
            'camera_calibration_node = cibo.camera_calibration:main',
            'body_face_hand_node = cibo.body_face_hand:main',
        ],
    },
)
