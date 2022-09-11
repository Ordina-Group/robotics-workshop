from setuptools import setup
from glob import glob

package_name = 'ros2_save_camera_image'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['settings.json']),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ANI717 (Animesh Bala Ani)',
    maintainer_email='animesh.ani@live.com',
    description='ROS2 Package to Save Camera Image',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'execute = ros2_save_camera_image.execute:main',
        ],
    },
)
