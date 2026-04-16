from setuptools import find_packages, setup

package_name = "perception_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/perception.yaml']),
    ],

    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="apollomotion",
    maintainer_email="phi@todo.todo",
    description="Perception pipeline for autonomous RC car",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "perception_node = perception_pkg.perception_node:main",
            "visualizer_node = perception_pkg.visualizer_node:main",
            "grid_viewer = perception_pkg.grid_viewer:main",
        ],
    },
)
