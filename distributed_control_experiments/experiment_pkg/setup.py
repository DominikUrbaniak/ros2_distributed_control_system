from setuptools import find_packages, setup

package_name = 'experiment_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dom_ros',
    maintainer_email='dominik.urbaniak@upc.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'linear_motion = experiment_pkg.linear_motion:main',
            'linear_motion_pub = experiment_pkg.linear_motion_pub:main',
            'real_pose_real_push = experiment_pkg.real_pose_real_push:main',
            'a1_image_publisher = experiment_pkg.a1_image_publisher:main',
            'a1_compressed_image_publisher = experiment_pkg.a1_compressed_image_publisher:main',
            'a2_aruco_detection = experiment_pkg.a2_aruco_detection:main',
            'a2_compressed_aruco_detection = experiment_pkg.a2_compressed_aruco_detection:main',
            'a3_pose_subscriber = experiment_pkg.a3_pose_subscriber:main',
            'a3_accuracy_measurement = experiment_pkg.a3_accuracy_measurement:main',
            'a2_hand_detection = experiment_pkg.a2_hand_detection:main',
            'a3_hand_pose_subscriber = experiment_pkg.a3_hand_pose_subscriber:main',
            'real_hand_follow = experiment_pkg.real_hand_follow:main',
            'rc_pose_client = experiment_pkg.rc_pose_client:main',
            'rc_real_pose_real_push = experiment_pkg.rc_real_pose_real_push:main',
            'cad_match_real_pose_real_push = experiment_pkg.cad_match_real_pose_real_push:main',
            'cad_match_pose_client = experiment_pkg.cad_match_pose_client:main',
        ],
    },
)
