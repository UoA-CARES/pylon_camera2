import os
from glob import glob

from setuptools import find_packages, setup

package_name = "pylon_camera2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools", "pypylon==4.0.0"],
    zip_safe=True,
    maintainer="anyone",
    maintainer_email="henryamwilliams@gmail.com",
    description="Pylon Camera ROS2 Node",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pylon_camera_node = pylon_camera2.pylon_camera_node:main",
        ],
    },
)
