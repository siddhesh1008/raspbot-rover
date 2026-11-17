from glob import glob

from setuptools import find_packages, setup

package_name = "rover_hmi"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/web", glob("web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Browser control page for the rover.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hmi = rover_hmi.hmi_node:main",
        ],
    },
)
