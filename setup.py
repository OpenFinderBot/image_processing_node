import os
from glob import glob

from setuptools import find_packages, setup

package_name = "image_processing_node"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Eduardo-Vilas-Boas",
    maintainer_email="eduardo.vilasboas.guerra@gmail.com",
    description="ROS 2 package for image processing operations including distortion removal and ROI cropping",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "image_processing_node = image_processing_node.image_processing_node:main",
        ],
    },
)
