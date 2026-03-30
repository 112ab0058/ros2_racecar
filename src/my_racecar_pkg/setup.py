import os
from glob import glob
from setuptools import setup

package_name = 'my_racecar_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch 檔案設定 (記得加逗號！)
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))), 
        
        # Config 檔案設定
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_car = my_racecar_pkg.hello_car:main',
            'talker = my_racecar_pkg.talker:main',
            'listener = my_racecar_pkg.listener:main',
            'service = my_racecar_pkg.service:main',
            'client = my_racecar_pkg.client:main',
            'lidar_tf = my_racecar_pkg.lidar_tf:main',
            'lidar_sim = my_racecar_pkg.lidar_sim:main',
            'odom_pub = my_racecar_pkg.odom_pub:main',
            'static_turtle_tf2_broadcaster = my_racecar_pkg.static_turtle_tf2_broadcaster:main',
            'turtle_tf2_broadcaster = my_racecar_pkg.turtle_tf2_broadcaster:main',
            'turtle_tf2_listener = my_racecar_pkg.turtle_tf2_listener:main',
        ],
    }, # 這裡必須在 setup() 的大括號裡面！
)