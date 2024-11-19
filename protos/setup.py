# Copyright (c) 2023 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

import os
import pathlib
import pkg_resources

import setuptools
import setuptools.dist
import setuptools._distutils.command.build


class BuildProtos(setuptools.Command):
    """Compiles protobufs into pb2.py files."""

    user_options = [("build-lib=", "b", "Directory to compile protobufs into")]

    def initialize_options(self):
        self.build_lib = None

    def finalize_options(self):
        if self.build_lib is None:
            build = self.distribution.get_command_obj("build")
            if self.distribution.packages is None:
                self.distribution.packages = []
            self.distribution.packages.extend(self.distribution.proto_packages)
            if self.distribution.package_dir is None:
                self.distribution.package_dir = {}
            for package in self.distribution.proto_packages:
                package_relpath = package.replace(".", os.path.sep)
                self.distribution.package_dir[package] = (
                    os.path.join(build.build_lib, package_relpath))
            self.build_lib = build.build_lib

    def get_source_files(self):
        source_files = []
        for package_name in self.distribution.proto_packages:
            package_base_path = package_name.replace(".", os.path.sep)
            if self.distribution.proto_package_dir is not None:
                if package_name in self.distribution.proto_package_dir:
                    package_root_path = self.distribution.proto_package_dir[package_name]
                else:
                    package_root_path = self.distribution.proto_package_dir[""]
            else:
                package_root_path = "."
            package_path = os.path.join(package_root_path, package_base_path)
            for filename in os.listdir(package_path):
                if filename.endswith(".proto"):
                    source_files.append(os.path.join(package_path, filename))
        return source_files

    def get_output_mapping(self):
        output_mapping = {}
        for f in self.get_source_files():
            output_mapping[f.replace(".proto", "_pb2.py")] = f
            output_mapping[f.replace(".proto", "_pb2_grpc.py")] = f
        return output_mapping

    def get_outputs(self):
        return list(self.get_output_mapping().keys())

    def run(self):
        for package_name in self.distribution.proto_packages:
            package_base_path = package_name.replace(".", os.path.sep)
            if self.distribution.proto_package_dir is not None:
                if package_name in self.distribution.proto_package_dir:
                    package_root_path = self.distribution.proto_package_dir[package_name]
                else:
                    package_root_path = self.distribution.proto_package_dir[""]
            else:
                package_root_path = "."
            output_path = os.path.join(self.build_lib, package_base_path)
            os.makedirs(output_path, exist_ok=True)
            with open(os.path.join(output_path, "__init__.py"), "w") as f:
                f.write('__path__ = __import__("pkgutil").extend_path(__path__, __name__)')

            from grpc_tools import protoc
            for filename in os.listdir(os.path.join(package_root_path, package_base_path)):
                if not filename.endswith(".proto"):
                    continue
                # the protoc.main discards the first argument, assuming it"s the program.
                args = ("_", os.path.join(package_base_path, filename), "--python_out=" + self.build_lib,
                        "--grpc_python_out=" + self.build_lib, "-I" + package_root_path,
                        "-I" + pkg_resources.resource_filename("grpc_tools", "_proto"))
                if self.verbose:
                    print("Building {}".format(filename))
                protoc.main(args)


class Build(setuptools._distutils.command.build.build):
    sub_commands = [("build_protos", None), *setuptools._distutils.command.build.build.sub_commands]


class ProtoDistribution(setuptools.dist.Distribution):

    proto_packages: list[str]
    proto_package_dir: dict

    def __init__(self, attrs) -> None:
        proto_packages = attrs.pop("proto_packages")
        proto_package_dir = attrs.pop("proto_package_dir", None)
        super().__init__(attrs)
        self.cmdclass["build_protos"] = BuildProtos
        self.cmdclass["build"] = Build
        self.proto_packages = proto_packages
        self.proto_package_dir = proto_package_dir

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

setuptools.setup(
    name=package_name,
    version=version,
    author="Boston Dynamics",
    author_email="support@bostondynamics.com",
    description="Boston Dynamics API definition of protobuf messages",
    distclass=ProtoDistribution,
    proto_packages=[root.replace(os.path.sep, ".") for root, _, _ in os.walk("bosdyn")],
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        ("share/ament_index/resource_index/packages", ["resource/" + package_name])
    ],
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://dev.bostondynamics.com/",
    project_urls={
        "Documentation": "https://dev.bostondynamics.com/",
        "Source": "https://github.com/boston-dynamics/spot-sdk/",
    },
    classifiers=[
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "License :: Other/Proprietary License",
        "Operating System :: OS Independent",
    ],
)
