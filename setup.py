from setuptools import find_packages, setup

package_name = "composite_sensing"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "pyyaml"],
    zip_safe=True,
    maintainer="hound",
    maintainer_email="hound@todo.todo",
    description="Stereo camera + derived VSLAM (composite sensing).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "multicam_rail_replay = composite_sensing.multicam_rail_replay:main",
        ],
    },
)
