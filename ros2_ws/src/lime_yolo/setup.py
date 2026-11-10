from glob import glob

from setuptools import setup

package_name = "lime_yolo"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sakaki-an",
    description="YOLO object detection for the TurtleBot3 Lime camera",
    license="Apache-2.0",
    entry_points={"console_scripts": ["yolo_detector = lime_yolo.yolo_detector:main"]},
)
