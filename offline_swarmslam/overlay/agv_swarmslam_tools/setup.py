from glob import glob
from setuptools import setup

package_name = "agv_swarmslam_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="riyaa",
    maintainer_email="riyaa@example.com",
    description="Offline Swarm-SLAM launch and visualization tools for myAGV RGB-D bags.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "apriltag_loop_closure_publisher = agv_swarmslam_tools.apriltag_loop_closure_publisher:main",
            "viz_bridge = agv_swarmslam_tools.viz_bridge:main",
        ],
    },
)
