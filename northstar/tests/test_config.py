# Copyright (c) 2025-2026 Littleton Robotics
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

"""Tests for northstar/config/config.py dataclasses."""

import sys
from pathlib import Path

import numpy
import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from config.config import ConfigStore, LocalConfig, RemoteConfig


class TestLocalConfig:
    def test_defaults(self):
        cfg = LocalConfig()
        assert cfg.device_id == ""
        assert cfg.server_ip == ""
        assert cfg.apriltags_stream_port == 8000
        assert cfg.objdetect_stream_port == 8001
        assert cfg.capture_impl == ""
        assert cfg.obj_detect_model == ""
        assert cfg.obj_detect_max_fps == -1
        assert cfg.apriltags_enable is False
        assert cfg.objdetect_enable is True
        assert cfg.video_folder == ""
        assert cfg.has_calibration is False
        assert cfg.camera_matrix is None
        assert cfg.distortion_coefficients is None

    def test_custom_values(self):
        matrix = numpy.eye(3, dtype=numpy.float64)
        dist = numpy.zeros((1, 5), dtype=numpy.float64)
        cfg = LocalConfig(
            device_id="cam0",
            server_ip="10.63.28.2",
            apriltags_enable=True,
            has_calibration=True,
            camera_matrix=matrix,
            distortion_coefficients=dist,
        )
        assert cfg.device_id == "cam0"
        assert cfg.apriltags_enable is True
        assert cfg.has_calibration is True
        assert cfg.camera_matrix is matrix
        assert cfg.distortion_coefficients is dist

    def test_mutable(self):
        """LocalConfig is a plain dataclass — fields can be updated."""
        cfg = LocalConfig()
        cfg.device_id = "updated"
        assert cfg.device_id == "updated"


class TestRemoteConfig:
    def test_defaults(self):
        cfg = RemoteConfig()
        assert cfg.event_name == ""
        assert cfg.match_type == 0
        assert cfg.match_number == 0
        assert cfg.camera_id == ""
        assert cfg.camera_resolution_width == 0
        assert cfg.camera_resolution_height == 0
        assert cfg.camera_auto_exposure == 0
        assert cfg.camera_exposure == 0
        assert cfg.camera_gain == 0
        assert cfg.camera_denoise == 0
        assert cfg.fiducial_size_m == 0
        assert cfg.tag_layout is None
        assert cfg.is_recording is False
        assert cfg.timestamp == 0

    def test_custom_values(self):
        cfg = RemoteConfig(
            event_name="NHED",
            match_number=5,
            camera_resolution_width=1280,
            camera_resolution_height=720,
            fiducial_size_m=0.1651,
            is_recording=True,
        )
        assert cfg.event_name == "NHED"
        assert cfg.match_number == 5
        assert cfg.camera_resolution_width == 1280
        assert cfg.camera_resolution_height == 720
        assert abs(cfg.fiducial_size_m - 0.1651) < 1e-9
        assert cfg.is_recording is True


class TestConfigStore:
    def test_construction(self):
        local = LocalConfig(device_id="front")
        remote = RemoteConfig(event_name="TEST")
        store = ConfigStore(local_config=local, remote_config=remote)
        assert store.local_config.device_id == "front"
        assert store.remote_config.event_name == "TEST"

    def test_references_are_same_objects(self):
        local = LocalConfig()
        remote = RemoteConfig()
        store = ConfigStore(local_config=local, remote_config=remote)
        assert store.local_config is local
        assert store.remote_config is remote
