from glob import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'traffic_light_adapter_template'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['config.yaml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.xml'),
        ),
    ],
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn',
        'pydantic',
        'requests>=2.25',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='Bey Hao Yun',
    maintainer_email='garybey@intrinsic.ai',
    description='A template for an RMF Traffic Light fleet adapter',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_adapter=traffic_light_adapter_template.fleet_adapter:main',
        ],
    },
)
