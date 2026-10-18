from glob import glob
from setuptools import find_packages, setup


package_name = "piper_yolo"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Sut1313",
    maintainer_email="323442660+Sut1313@users.noreply.github.com",
    description="YOLOv8-seg ONNX perception bridge for Piper and GraspNet.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "yolo_bridge_node = piper_yolo.bootstrap:main",
            "real_yolo_pointcloud_node = piper_yolo.real_bootstrap:main",
        ],
    },
)
