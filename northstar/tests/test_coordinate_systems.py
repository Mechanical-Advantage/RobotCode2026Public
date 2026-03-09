# Copyright (c) 2025-2026 Littleton Robotics
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

"""Tests for northstar/pipeline/coordinate_systems.py.

Verifies the OpenCV ↔ WPILib coordinate system conversions used by the
AprilTag pose pipeline.  Both functions are pure math with no hardware
dependency.
"""

import math
import sys
from pathlib import Path

import numpy
import pytest

# northstar uses local imports without a package prefix; add its root to the path.
sys.path.insert(0, str(Path(__file__).parent.parent))

from pipeline.coordinate_systems import openCvPoseToWpilib, wpilibTranslationToOpenCv
from wpimath.geometry import Translation3d


class TestOpenCvPoseToWpilib:
    """openCvPoseToWpilib(tvec, rvec) -> Pose3d"""

    def _make_tvec(self, x, y, z):
        """Create a 3×1 tvec as OpenCV returns it."""
        return numpy.array([[x], [y], [z]], dtype=numpy.float64)

    def _make_rvec(self, rx, ry, rz):
        return numpy.array([[rx], [ry], [rz]], dtype=numpy.float64)

    def test_zero_pose_produces_origin(self):
        pose = openCvPoseToWpilib(
            self._make_tvec(0, 0, 0), self._make_rvec(0, 0, 0)
        )
        assert abs(pose.X()) < 1e-9
        assert abs(pose.Y()) < 1e-9
        assert abs(pose.Z()) < 1e-9

    def test_translation_axis_mapping(self):
        """OpenCV Z → WPILib X, OpenCV X → WPILib -Y, OpenCV Y → WPILib -Z."""
        tvec = self._make_tvec(1.0, 2.0, 3.0)
        rvec = self._make_rvec(0, 0, 0)
        pose = openCvPoseToWpilib(tvec, rvec)

        assert abs(pose.X() - 3.0) < 1e-9, f"Expected X=3 (from OpenCV Z), got {pose.X()}"
        assert abs(pose.Y() - (-1.0)) < 1e-9, f"Expected Y=-1 (from -OpenCV X), got {pose.Y()}"
        assert abs(pose.Z() - (-2.0)) < 1e-9, f"Expected Z=-2 (from -OpenCV Y), got {pose.Z()}"

    def test_pure_z_translation(self):
        """Camera optical axis (OpenCV Z) maps to WPILib forward (X)."""
        tvec = self._make_tvec(0, 0, 5.0)
        rvec = self._make_rvec(0, 0, 0)
        pose = openCvPoseToWpilib(tvec, rvec)
        assert abs(pose.X() - 5.0) < 1e-9

    def test_rotation_angle_preserved(self):
        """Rotation vector magnitude (angle) must be preserved after axis remap."""
        angle = math.pi / 4
        # Rotation around OpenCV Z axis → WPILib X axis
        rvec = self._make_rvec(0, 0, angle)
        tvec = self._make_tvec(0, 0, 0)
        pose = openCvPoseToWpilib(tvec, rvec)
        # WPILib Rotation3d.angle is a property returning the rotation magnitude
        assert abs(pose.rotation().angle - angle) < 1e-9

    def test_negative_translation(self):
        tvec = self._make_tvec(-2.0, -3.0, -4.0)
        rvec = self._make_rvec(0, 0, 0)
        pose = openCvPoseToWpilib(tvec, rvec)
        assert abs(pose.X() - (-4.0)) < 1e-9
        assert abs(pose.Y() - 2.0) < 1e-9
        assert abs(pose.Z() - 3.0) < 1e-9


class TestWpilibTranslationToOpenCv:
    """wpilibTranslationToOpenCv(translation) -> [float, float, float]"""

    def test_zero_translation(self):
        result = wpilibTranslationToOpenCv(Translation3d(0, 0, 0))
        assert result == [0.0, 0.0, 0.0]

    def test_axis_mapping(self):
        """WPILib X → OpenCV Z, WPILib Y → OpenCV -X, WPILib Z → OpenCV -Y."""
        t = Translation3d(1.0, 2.0, 3.0)
        result = wpilibTranslationToOpenCv(t)
        # result = [-Y, -Z, X] = [-2, -3, 1]
        assert abs(result[0] - (-2.0)) < 1e-9, f"Expected cv_x=-2, got {result[0]}"
        assert abs(result[1] - (-3.0)) < 1e-9, f"Expected cv_y=-3, got {result[1]}"
        assert abs(result[2] - 1.0) < 1e-9, f"Expected cv_z=1, got {result[2]}"

    def test_returns_list_of_three(self):
        result = wpilibTranslationToOpenCv(Translation3d(1, 2, 3))
        assert isinstance(result, list)
        assert len(result) == 3

    def test_roundtrip_with_openCvPoseToWpilib(self):
        """Converting WPILib → OpenCV → WPILib should yield the original translation."""
        original = Translation3d(3.0, -1.0, 2.0)
        cv_coords = wpilibTranslationToOpenCv(original)

        # Pack into tvec (OpenCV format), zero rotation
        tvec = numpy.array([[cv_coords[0]], [cv_coords[1]], [cv_coords[2]]], dtype=numpy.float64)
        rvec = numpy.zeros((3, 1), dtype=numpy.float64)
        pose = openCvPoseToWpilib(tvec, rvec)

        assert abs(pose.X() - original.X()) < 1e-9
        assert abs(pose.Y() - original.Y()) < 1e-9
        assert abs(pose.Z() - original.Z()) < 1e-9
