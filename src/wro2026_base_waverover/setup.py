from glob import glob

from setuptools import setup

package_name = 'wro2026_base_waverover'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver = wro2026_base_waverover.driver_node:main',
            'test_motion = wro2026_base_waverover.test_motion:main',
            'check_serial = wro2026_base_waverover.check_serial:main',
        ],
    },
)
