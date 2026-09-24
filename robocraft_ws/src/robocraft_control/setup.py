from setuptools import find_packages, setup

package_name = "robocraft_control"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Basheer Al-Tawil",
    maintainer_email="basheeraltaweel@gmail.com",
    description="RoboCraft cell commander, scenarios, vision and visualisation",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "scenario = robocraft_control.scenario_runner:main",
            "cell_visualizer = robocraft_control.cell_visualizer:main",
            "obstacle_detector = robocraft_control.obstacle_detector:main",
        ],
    },
)
