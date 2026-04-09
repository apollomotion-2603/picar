from setuptools import find_packages, setup

package_name = 'mpc_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/nmpc.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='apollomotion',
    maintainer_email='apollomotion@todo.todo',
    description='NMPC lane following — Kloeser 2020',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_node = mpc_pkg.mpc_node:main',
        ],
    },
)
