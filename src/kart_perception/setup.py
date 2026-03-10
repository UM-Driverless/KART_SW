from setuptools import setup


package_name = "kart_perception"


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/perception_test.launch.py",
                "launch/perception_3d.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rubenayla",
    maintainer_email="rubenayla@example.com",
    description="Perception nodes for image sources and cone detection.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "image_source = kart_perception.image_source_node:main",
            "yolo_detector = kart_perception.yolo_detector_node:main",
            "cone_marker_viz = kart_perception.cone_marker_viz_node:main",
            "cone_depth_localizer = kart_perception.cone_depth_localizer_node:main",
            "cone_marker_viz_3d = kart_perception.cone_marker_viz_3d_node:main",
            "steering_hud = kart_perception.steering_hud_node:main",
            "hud_viewer = kart_perception.hud_viewer_node:main",
        ],
    },
)
