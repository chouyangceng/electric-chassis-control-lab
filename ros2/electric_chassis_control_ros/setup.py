from setuptools import setup

package_name = "electric_chassis_control_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/controller.yaml"]),
        ("share/" + package_name + "/launch", ["launch/controller.launch.py"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    entry_points={"console_scripts": ["controller_node = electric_chassis_control_ros.controller_node:main"]},
)
