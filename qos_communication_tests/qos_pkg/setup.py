from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qos_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dominik',
    maintainer_email='dominik.urbaniak@upc.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'a1_compressed_image_pub = qos_pkg.a1_compressed_image_pub:main',
            'a2_compressed_image_sub_pub = qos_pkg.a2_compressed_image_sub_pub:main',
            'a3_compressed_image_sub = qos_pkg.a3_compressed_image_sub:main',
            'image_a1_pub = qos_pkg.image_a1_pub:main',
            'image_a2_sub_pub = qos_pkg.image_a2_sub_pub:main',
            'rs_a2_sub_pub = qos_pkg.rs_a2_sub_pub:main',
            'image_a3_sub = qos_pkg.image_a3_sub:main',
            'poses_a1_pub = qos_pkg.poses_a1_pub:main',
            'poses_a2_sub_pub = qos_pkg.poses_a2_sub_pub:main',
            'poses_a3_sub = qos_pkg.poses_a3_sub:main',
            'poses_client = qos_pkg.poses_client:main',
            'poses_srv = qos_pkg.poses_srv:main',
        ],
    },
)
