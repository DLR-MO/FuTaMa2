# SPDX-FileCopyrightText: 2024 German Aerospace Center <adrian.ricardezortigosa@dlr.de>
#
# SPDX-License-Identifier: MIT

from setuptools import setup
from glob import glob

package_name = "futama2_description"

setup(
    name=package_name,
    version="0.0.1",
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*.py")),
        ("share/" + package_name, glob("config/*.yaml")),
        ("share/" + package_name + "/urdf/", glob("urdf/*")),
        ("share/" + package_name + "/rviz/", glob("rviz/*")),
        ("share/" + package_name + "/meshes/collision/", glob("meshes/collision/*")),
        ("share/" + package_name + "/meshes/visual/", glob("meshes/visual/*")),
    ],
    install_requires=["setuptools"],
    py_modules=[],
    zip_safe=True,
    maintainer="Adrián Ricárdez Ortigosa",
    maintainer_email="adrian.ricardezortigosa@dlr.de",
    description="Description package for the FUTAMA2 robotn",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [],
    },
)
