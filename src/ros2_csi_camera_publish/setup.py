from setuptools import setup
from glob import glob

package_name = 'ros2_csi_camera_publish'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['settings.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ANI717 (Animesh Bala Ani)',
    maintainer_email='animesh.ani@live.com',
    description='Package to Publish CSI Camera Data in ROS 2 Topic',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jetson = ros2_csi_camera_publish.jetson:main',
        ],
    },
)
