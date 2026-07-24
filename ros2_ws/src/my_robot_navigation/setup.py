import os
from glob import glob

from setuptools import find_packages, setup


package_name = "my_robot_navigation"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            os.path.join("share", package_name),
            ["package.xml"],
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.xml"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="alvaro-medina",
    maintainer_email="alvaro.mediina2003@gmail.com",
    description="Navegación y orquestación del robot con Nav2",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "orchestrator = my_robot_navigation.orchestrator:main",
        ],
    },
)