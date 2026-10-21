from glob import glob

from setuptools import setup

package_name = 'wro2026_sim_r550'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/models/r550', glob('models/r550/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_base_node = wro2026_sim_r550.sim_base_node:main',
            'sim_sensors_node = wro2026_sim_r550.sim_sensors_node:main',
            'drive_test_node = wro2026_sim_r550.drive_test_node:main',
            'lap_node = wro2026_sim_r550.lap_node:main',
        ],
    },
)
