from setuptools import setup

setup(
    name="pod_perception",
    version="0.1.0",
    packages=["pod_perception"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/pod_perception"]),
                ("share/pod_perception", ["package.xml"])],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["pod_perception = pod_perception.node:main"]},
)
