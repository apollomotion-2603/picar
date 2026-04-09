from setuptools import find_packages, setup

package_name = 'ekf_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='apollomotion',
    maintainer_email='apollomotion@todo.todo',
    description='EKF sensor fusion for RC car',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ekf_node = ekf_pkg.ekf_node:main',
            'reset_node = ekf_pkg.reset_node:main', 
        ],
    },
)
