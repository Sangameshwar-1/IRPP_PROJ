from setuptools import setup

package_name = "ros_humans_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/demo.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="scan-project",
    maintainer_email="user@example.com",
    description="ROS 2 human pose publisher and mover.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "publish_human_pose = ros_humans_ros2.publish_human_pose:main",
            "move_humans = ros_humans_ros2.move_humans:main",
        ],
    },
)
