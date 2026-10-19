from glob import glob

from setuptools import setup

package_name = 'wro2026_base_r550'

setup(
    name=package_name,
    version='0.1.0',
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
            'feedback_node = wro2026_base_r550.feedback_node:main',
            'base_node = wro2026_base_r550.base_node:main',
            'listen = wro2026_base_r550.listen:main',
            'teleop_node = wro2026_base_r550.teleop_node:main',
            'record_node = wro2026_base_r550.record_node:main',
            'soak_log_node = wro2026_base_r550.soak_log_node:main',
            'sensor_log_node = wro2026_base_r550.sensor_log_node:main',
        ],
    },
)
