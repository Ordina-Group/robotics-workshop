from glob import glob
from pathlib import Path
from setuptools import setup


def get_value_from_string(string: str, prefix: str, suffix: str):
    value = ""
    if string.startswith(prefix) and string.endswith(suffix):
        value = string[len(prefix):][:-len(suffix)]
    return value


with open(Path(__file__).parent / 'package.xml', 'r') as xml_file:
    for line in xml_file.readlines():
        if line.startswith('  <name>') and line.endswith('</name>\n'):
            package_name = get_value_from_string(line, '  <name>', '</name>\n')
        elif line.startswith('  <version>') and line.endswith('</version>\n'):
            version = get_value_from_string(line, '  <version>', '</version>\n')
        elif line.startswith('  <description>') and line.endswith('</description>\n'):
            description = get_value_from_string(line, '  <description>', '</description>\n')
        elif line.startswith('  <maintainer') and line.endswith('</maintainer>\n'):
            line = get_value_from_string(line, '  <maintainer email="', '</maintainer>\n')
            maintainer_email, maintainer = line.split('">')
        elif line.startswith('  <license>') and line.endswith('</license>\n'):
            project_license = get_value_from_string(line, '  <license>', '</license>\n')


setup(
    name=package_name,
    version=version,
    packages=[package_name],
    data_files=[
        (str(Path('share/ament_index/resource_index/packages')), [str(Path(f'resource/{package_name}'))]),
        (str(Path(f'share/{package_name}/config')), glob(str(Path('config/*')))),
        (str(Path(f'share/{package_name}/launch')), glob(str(Path('launch/*.launch.py')))),
        (str(Path(f'share/{package_name}')), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer=maintainer,
    maintainer_email=maintainer_email,
    description=description,
    license=project_license,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_livestream = camera_livestream.camera_livestream:main'
        ],
    },
)
