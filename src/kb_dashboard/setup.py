from setuptools import setup

package_name = "kb_dashboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, package_name + ".generated"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/dashboard.launch.py"]),
    ],
    package_data={package_name: ["index.html"]},
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "dashboard = kb_dashboard.dashboard_node:main",
        ],
    },
)
