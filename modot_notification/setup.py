import os

from setuptools import find_packages, setup

package_name = "modot_notification"


def data_files_from_directory(directory: str) -> list[tuple[str, list[str]]]:
    data_files = []
    for dirpath, _, filenames in os.walk(directory):
        if filenames:
            data_files.append(
                (
                    "share/" + package_name + "/" + dirpath,
                    [dirpath + "/" + filename for filename in filenames],
                )
            )
    return data_files


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ]
    + data_files_from_directory("resource/sound"),
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
