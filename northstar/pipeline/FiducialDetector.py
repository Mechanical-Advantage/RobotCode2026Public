# Copyright (c) 2022-2026 Littleton Robotics
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

from typing import List

import cv2
from config.config import ConfigStore
from vision_types import FiducialImageObservation


class FiducialDetector:
    def __init__(self) -> None:
        raise NotImplementedError

    def detect_fiducials(self, image: cv2.Mat, config_store: ConfigStore) -> List[FiducialImageObservation]:
        raise NotImplementedError


class ArucoFiducialDetector(FiducialDetector):
    def __init__(self, dictionary_id) -> None:
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self._aruco_params = cv2.aruco.DetectorParameters()

    def detect_fiducials(self, image: cv2.Mat, config_store: ConfigStore) -> List[FiducialImageObservation]:
        corners, ids, _ = cv2.aruco.detectMarkers(image, self._aruco_dict, parameters=self._aruco_params)
        if len(corners) == 0:
            return []
        return [FiducialImageObservation(id[0], corner) for id, corner in zip(ids, corners)]


class ArucoMaxFiducialDetector(FiducialDetector):
    def __init__(self, dictionary_id) -> None:
        import aruco_max
        self._detector = aruco_max.ArucoMaxDetector(cv2.aruco.DICT_APRILTAG_36h11)

    def detect_fiducials(self, image: cv2.Mat, config_store: ConfigStore) -> List[FiducialImageObservation]:
        if len(image.shape) > 2:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids = self._detector.detect(image)
        if len(corners) == 0:
            return []
        return [FiducialImageObservation(id, corner) for id, corner in zip(ids, corners)]
