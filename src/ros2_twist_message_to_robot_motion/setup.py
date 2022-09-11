from setuptools import setup
from glob import glob

package_name = 'ros2_twist_message_to_robot_motion'

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
    description='Package to Run Robots Subscribed to Geometry Twist Message',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jetbot = ros2_twist_message_to_robot_motion.jetbot:main',
            'adafruit = ros2_twist_message_to_robot_motion.adafruit:main',
        ],
    },
)
