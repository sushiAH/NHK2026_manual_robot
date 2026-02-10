from setuptools import find_packages, setup
import os
from glob import glob

package_name = "manual_robot"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*_launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aratahorie",
    maintainer_email="aratahorie@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": ["pytest",],
    },
    # ノードの実行ファイルはここに書く
    entry_points={
        "console_scripts": [
            "subscribe_twist_node = manual_robot.subscribe_twist_node:main",
            "publish_twist_node = manual_robot.publish_twist_node:main",
            "publish_feedback_node = manual_robot.publish_feedback_node:main",
            "control_arm_hand_node = manual_robot.control_arm_hand_node:main",
            "control_spear_node = manual_robot.control_spear_node:main",
            "control_box_node = manual_robot.control_box_node:main",
        ],
    },
)
