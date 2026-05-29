# Copyright (c) 2022-2026 Littleton Robotics
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup
import os

script_dir = os.path.abspath(os.path.dirname(__file__))
ext_modules = [
    Pybind11Extension(
        "aruco_max",
        [
            os.path.join(script_dir, "wrapper.cpp"),
            os.path.join(script_dir, "metal_context.cpp")
        ],
        include_dirs=["/opt/homebrew/include/opencv4", script_dir],
        libraries=["opencv_core", "opencv_imgproc", "opencv_calib3d", "opencv_objdetect"],
        library_dirs=["/opt/homebrew/lib"],
        extra_link_args=["-framework", "Metal", "-framework", "Foundation"],
        extra_compile_args=["-O3", "-std=c++14", "-ObjC++"]
    ),
]

setup(
    name="aruco_max",
    ext_modules=ext_modules,
    cmdclass={"build_ext": build_ext},
)
