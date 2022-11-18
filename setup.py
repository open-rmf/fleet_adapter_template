from setuptools import setup

package_name = 'temi_fleet_adapter_v2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name, ['configs/config.yaml']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yadunund',
    maintainer_email='yadunund@openrobotics.org',
    description='A template for an RMF fleet adapter',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'temi_fleet_adapter_v2=temi_fleet_adapter_v2.temi_fleet_adapter_v2:main'
        ],
    },
)
