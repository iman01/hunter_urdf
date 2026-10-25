from glob import glob

from setuptools import find_packages, setup

package_name = "hunter_control"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="Iman Esfandiyar",
    maintainer_email="iman.esfandiyar@pit.lukasiewicz.gov.pl",
    description="4WS drive controller, odometry and suspension visualiser for the PIAP HUNTeR replica",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "four_ws_controller = hunter_control.four_ws_controller:main",
            "suspension_linkage_publisher = hunter_control.suspension_linkage_publisher:main",
        ],
    },
)
