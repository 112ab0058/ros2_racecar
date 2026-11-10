from setuptools import setup

package_name = 'wro2026_imitation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bc_extract = wro2026_imitation.extract:main',
            'bc_train = wro2026_imitation.train:main',
            'bc_node = wro2026_imitation.bc_node:main',
        ],
    },
)
