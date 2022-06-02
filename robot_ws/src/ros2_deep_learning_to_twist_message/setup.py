from setuptools import setup
from glob import glob

package_name = 'ros2_deep_learning_to_twist_message'

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
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ANI717 (Animesh Bala Ani)',
    maintainer_email='animesh.ani@live.com',
    description='Deep Learning Package to Publish Twist Message from Camera Image for Robot Motion',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'onnx = ros2_deep_learning_to_twist_message.onnx_model:main',
        ],
    },
)