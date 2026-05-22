from setuptools import find_packages, setup

package_name = 'wro2026_mapper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WRO 2026 Team',
    maintainer_email='team@wro2026.com',
    description='Independent WRO 2026 lawn-mower mapping tool for /odom and /scan based coverage runs.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wro2026_mapper = wro2026_mapper.wro2026_mapper:main',
        ],
    },
)
