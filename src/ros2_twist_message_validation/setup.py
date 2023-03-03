from setuptools import setup
from glob import glob

package_name = 'ros2_twist_message_validation'

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
    maintainer='Louigimar Richardson',
    maintainer_email='louigimar.richardson@ordina.nl',
    description='Package to validate Twist Message for Robot Running',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'validate = ros2_twist_message_validate.validation:main',
        ],
    },
)