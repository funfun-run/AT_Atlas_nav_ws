from setuptools import find_packages, setup

package_name = "mission_manager"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="funfun",
    maintainer_email="1219921425@qq.com",
    description="Mission manager for task scheduling and nav action client",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_manager = mission_manager.mission_manager:main",
        ],
    },
)
