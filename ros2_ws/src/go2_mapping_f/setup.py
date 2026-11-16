from glob import glob
import os

from setuptools import find_packages, setup


package_name = "go2_mapping_f"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fishbone",
    maintainer_email="fishbone@localhost",
    description="Experimental low-smear online mapping pipeline for the Go2.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "scan_stability_gate = go2_mapping_f.scan_stability_gate:main",
        ],
    },
)
