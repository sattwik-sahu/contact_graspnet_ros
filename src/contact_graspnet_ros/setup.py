from glob import glob

from setuptools import find_packages, setup

package_name = "contact_graspnet_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "test.*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=[],
    zip_safe=True,
    maintainer="Contact-GraspNet ROS maintainers",
    maintainer_email="maintainer@example.com",
    description="ROS 2 wrapper for Contact-GraspNet (PyTorch port).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "graspnet_node = contact_graspnet_ros.node:main",
        ],
    },
)
