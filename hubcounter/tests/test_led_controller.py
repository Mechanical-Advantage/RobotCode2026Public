"""Tests for hub_counter.led.controller.LEDController (using mock hardware)."""

import os

import pytest

# Ensure mock LED hardware is used regardless of platform
os.environ["MOCK_HARDWARE"] = "1"

from hub_counter.config.settings import ColorConfig, LEDConfig, ThresholdConfig
from hub_counter.led.controller import LEDController


@pytest.fixture
def controller():
    led_cfg = LEDConfig(led_count=6, brightness=128)
    thresh_cfg = ThresholdConfig(yellow=100, blue=360)
    color_cfg = ColorConfig()
    ctrl = LEDController(led_cfg, thresh_cfg, color_cfg)
    ctrl.start()
    yield ctrl
    ctrl.stop()


class TestLEDControllerLifecycle:
    def test_start_sets_active(self, controller):
        assert controller.is_active is True

    def test_stop_clears_active(self, controller):
        controller.stop()
        assert controller.is_active is False

    def test_double_start_is_idempotent(self):
        led_cfg = LEDConfig(led_count=4)
        ctrl = LEDController(led_cfg, ThresholdConfig(), ColorConfig())
        ctrl.start()
        ctrl.start()  # second call should be a no-op
        assert ctrl.is_active is True
        ctrl.stop()

    def test_double_stop_is_idempotent(self, controller):
        controller.stop()
        controller.stop()  # second call should be a no-op


class TestGetColorForCount:
    def test_below_yellow_threshold_returns_red(self, controller):
        color = controller.get_color_for_count(0)
        assert color == ColorConfig().red

    def test_at_yellow_threshold_returns_yellow(self, controller):
        color = controller.get_color_for_count(100)
        assert color == ColorConfig().yellow

    def test_between_thresholds_returns_yellow(self, controller):
        color = controller.get_color_for_count(200)
        assert color == ColorConfig().yellow

    def test_at_blue_threshold_returns_blue(self, controller):
        color = controller.get_color_for_count(360)
        assert color == ColorConfig().blue

    def test_above_blue_threshold_returns_blue(self, controller):
        color = controller.get_color_for_count(1000)
        assert color == ColorConfig().blue


class TestUpdateConfig:
    def test_update_threshold_changes_color_result(self, controller):
        # Lower blue threshold to 10 so count=15 → blue
        controller.update_config(threshold_config=ThresholdConfig(yellow=5, blue=10))
        assert controller.get_color_for_count(15) == ColorConfig().blue

    def test_update_color_config(self, controller):
        new_colors = ColorConfig(red=(1, 2, 3))
        controller.update_config(color_config=new_colors)
        assert controller.get_color_for_count(0) == (1, 2, 3)

    def test_update_none_args_is_noop(self, controller):
        controller.update_config()  # both None — should not raise


class TestSetSolid:
    def test_set_solid_does_not_raise(self, controller):
        controller.set_solid((255, 0, 0))  # should not raise

    def test_set_solid_when_inactive_is_noop(self):
        led_cfg = LEDConfig(led_count=4)
        ctrl = LEDController(led_cfg, ThresholdConfig(), ColorConfig())
        # Not started — calling set_solid should silently do nothing
        ctrl.set_solid((255, 0, 0))
