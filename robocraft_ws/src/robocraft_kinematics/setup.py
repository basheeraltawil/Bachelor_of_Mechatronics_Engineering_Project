from setuptools import find_packages, setup

package_name = "robocraft_kinematics"

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
    description="RoboCraft kinematics and trajectory library",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "workspace_analysis = robocraft_kinematics.workspace:main",
        ],
    },
)
