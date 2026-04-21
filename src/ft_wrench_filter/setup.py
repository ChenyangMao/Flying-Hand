from setuptools import find_packages, setup

package_name = "ft_wrench_filter"

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
    description="WrenchStamped low-pass filter node",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wrench_filter = ft_wrench_filter.wrench_filter_node:main",
        ],
    },
)

