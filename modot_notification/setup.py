import os

from setuptools import find_packages, setup

package_name = "modot_notification"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mitsuhiro Sakamoto",
    maintainer_email="mitukou1109@gmail.com",
    description="Package for MODOT notification",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["sound_notifier = modot_notification.sound_notifier:main"],
    },
)
