from setuptools import find_packages, setup

package_name = "ft_fz_udp_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Flying-Hand",
    maintainer_email="maintainer@example.com",
    description="UDP Fz to ROS2 WrenchStamped (ft_data) bridge",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "udp_fz_bridge = ft_fz_udp_bridge.udp_fz_bridge_node:main",
        ],
    },
)
