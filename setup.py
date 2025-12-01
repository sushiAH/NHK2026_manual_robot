from setuptools import find_packages, setup
import os
from glob import glob

package_name = "manual_robot"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
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
        "test": [
            "pytest",
        ],
    },
    # ノードの実行ファイルはここに書く
    entry_points={
        "console_scripts": [
            "twist_subscriber = manual_robot.subscribe_twist:main",
            "twist_publisher = manual_robot.publish_twist:main",
            "feedback_publisher = manual_robot.publish_feedback:main",
        ],
    },
)
