# Copyright (c) 2025-2026 Littleton Robotics
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

"""Tests for northstar/vision_types.py dataclasses."""

import sys
from pathlib import Path

import numpy
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from vision_types import (
    CameraPoseObservation,
    FiducialImageObservation,
    FiducialPoseObservation,
    ObjDetectObservation,
    TagAngleObservation,
)
from wpimath.geometry import Pose3d, Rotation3d, Translation3d


def _dummy_corners():
    return numpy.zeros((4, 2), dtype=numpy.float64)


def _pose(x=0.0, y=0.0, z=0.0):
    return Pose3d(Translation3d(x, y, z), Rotation3d())


class TestFiducialImageObservation:
    def test_construction(self):
        corners = _dummy_corners()
        obs = FiducialImageObservation(tag_id=5, corners=corners)
        assert obs.tag_id == 5
        assert obs.corners is corners

    def test_frozen_prevents_mutation(self):
        obs = FiducialImageObservation(tag_id=1, corners=_dummy_corners())
        with pytest.raises((AttributeError, TypeError)):
            obs.tag_id = 99  # type: ignore[misc]


class TestFiducialPoseObservation:
    def test_construction(self):
        obs = FiducialPoseObservation(
            tag_id=3,
            pose_0=_pose(1, 0, 0),
            error_0=0.05,
            pose_1=_pose(2, 0, 0),
            error_1=0.10,
        )
        assert obs.tag_id == 3
        assert obs.error_0 < obs.error_1

    def test_frozen(self):
        obs = FiducialPoseObservation(
            tag_id=1, pose_0=_pose(), error_0=0.0, pose_1=_pose(), error_1=0.0
        )
        with pytest.raises((AttributeError, TypeError)):
            obs.error_0 = 99.0  # type: ignore[misc]


class TestCameraPoseObservation:
    def test_single_solution(self):
        obs = CameraPoseObservation(
            tag_ids=[1, 2], pose_0=_pose(1, 0, 0), error_0=0.1, pose_1=None, error_1=None
        )
        assert obs.pose_1 is None
        assert obs.error_1 is None
        assert len(obs.tag_ids) == 2

    def test_dual_solution(self):
        obs = CameraPoseObservation(
            tag_ids=[3], pose_0=_pose(1, 0, 0), error_0=0.1, pose_1=_pose(2, 0, 0), error_1=0.2
        )
        assert obs.pose_1 is not None
        assert obs.error_1 == 0.2


class TestTagAngleObservation:
    def test_construction(self):
        corners = _dummy_corners()
        obs = TagAngleObservation(tag_id=7, corners=corners, distance=3.5)
        assert obs.tag_id == 7
        assert obs.distance == 3.5

    def test_frozen(self):
        obs = TagAngleObservation(tag_id=1, corners=_dummy_corners(), distance=1.0)
        with pytest.raises((AttributeError, TypeError)):
            obs.distance = 99.0  # type: ignore[misc]


class TestObjDetectObservation:
    def test_construction(self):
        angles = numpy.zeros((4, 2), dtype=numpy.float64)
        pixels = numpy.zeros((4, 2), dtype=numpy.float64)
        obs = ObjDetectObservation(
            obj_class=0, confidence=0.95, corner_angles=angles, corner_pixels=pixels
        )
        assert obs.obj_class == 0
        assert obs.confidence == pytest.approx(0.95)

    def test_frozen(self):
        obs = ObjDetectObservation(
            obj_class=0,
            confidence=0.5,
            corner_angles=numpy.zeros((4, 2)),
            corner_pixels=numpy.zeros((4, 2)),
        )
        with pytest.raises((AttributeError, TypeError)):
            obs.confidence = 0.0  # type: ignore[misc]
