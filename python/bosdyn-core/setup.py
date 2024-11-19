# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

with open("README.md", "r") as fh:
    long_description = fh.read()

import xml.etree.ElementTree as ET
tree = ET.parse("package.xml")
root = tree.getroot()
tag = root.find("name")
assert tag is not None
package_name = tag.text or ""
tag = root.find("version")
assert tag is not None
version = tag.text

import setuptools
setuptools.setup(
    name=package_name,
    version=version,
    author="Boston Dynamics",
    author_email="support@bostondynamics.com",
    description="Boston Dynamics API Core code and interfaces",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://dev.bostondynamics.com/",
    project_urls={
        "Documentation": "https://dev.bostondynamics.com/",
        "Source": "https://github.com/boston-dynamics/spot-sdk/",
    },
    packages=setuptools.find_packages('src'),
    package_dir={'': 'src'},
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name])
    ],
    classifiers=[
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "License :: Other/Proprietary License",
        "Operating System :: OS Independent",
    ],
)
