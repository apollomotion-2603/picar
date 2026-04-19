from setuptools import setup

package_name = 'race_cars'

setup(
    name=package_name,
    version='0.1.0',
    # Flat layout: all .py files sit directly in the package root.
    # List them explicitly as top-level modules so setuptools installs them.
    py_modules=[
        'racecar_nmpc_node',
        'acados_settings',
        'bicycle_model',
        'time2spatial',
        'plotFcn',
    ],
    data_files=[
        # ament resource index entry (required by colcon)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml (required by colcon)
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Realtime NMPC controller for autonomous racing (track-agnostic, kappa from camera)',
    license='BSD-2-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # top-level module (flat layout)
            'racecar_nmpc_node = racecar_nmpc_node:main',
        ],
    },
)


