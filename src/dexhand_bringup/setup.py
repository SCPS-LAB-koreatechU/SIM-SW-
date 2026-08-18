from glob import glob

from setuptools import find_packages, setup

package_name = "dexhand_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/tools", glob("tools/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="김종혁",
    maintainer_email="hexk0131@gmail.com",
    description="DexHand v2 4손가락 8서보 ROS 2 드라이버, 손끝 IK, 그립 프리셋",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hand_driver = dexhand_bringup.hand_driver_node:main",
            "fingertip_ik = dexhand_bringup.fingertip_ik_node:main",
            "grip_presets = dexhand_bringup.grip_preset_node:main",
            "joint_sliders = dexhand_bringup.joint_slider_node:main",
        ],
    },
)
