"""Tests for hub_counter.config.settings."""

import tempfile
from pathlib import Path

import pytest
import yaml

from hub_counter.config.settings import (
    ColorConfig,
    GPIOConfig,
    LEDConfig,
    NetworkConfig,
    Settings,
    ThresholdConfig,
    WebConfig,
    load_settings,
    save_settings,
)


class TestGPIOConfig:
    def test_defaults(self):
        cfg = GPIOConfig()
        assert cfg.channel_pins == [23, 24, 25, 16]
        assert cfg.active_low is True
        assert cfg.debounce_ms == 10

    def test_custom_values(self):
        cfg = GPIOConfig(channel_pins=[1, 2, 3, 4], active_low=False, debounce_ms=50)
        assert cfg.channel_pins == [1, 2, 3, 4]
        assert cfg.active_low is False
        assert cfg.debounce_ms == 50

    def test_debounce_min_boundary(self):
        cfg = GPIOConfig(debounce_ms=10)
        assert cfg.debounce_ms == 10

    def test_debounce_below_min_raises(self):
        with pytest.raises(Exception):
            GPIOConfig(debounce_ms=9)

    def test_debounce_above_max_raises(self):
        with pytest.raises(Exception):
            GPIOConfig(debounce_ms=501)


class TestLEDConfig:
    def test_defaults(self):
        cfg = LEDConfig()
        assert cfg.data_pin == 21
        assert cfg.led_count == 12
        assert cfg.brightness == 200

    def test_brightness_zero_allowed(self):
        cfg = LEDConfig(brightness=0)
        assert cfg.brightness == 0

    def test_led_count_min(self):
        cfg = LEDConfig(led_count=1)
        assert cfg.led_count == 1

    def test_led_count_below_min_raises(self):
        with pytest.raises(Exception):
            LEDConfig(led_count=0)


class TestThresholdConfig:
    def test_defaults(self):
        cfg = ThresholdConfig()
        assert cfg.yellow == 100
        assert cfg.blue == 360

    def test_custom(self):
        cfg = ThresholdConfig(yellow=50, blue=200)
        assert cfg.yellow == 50
        assert cfg.blue == 200

    def test_yellow_zero_raises(self):
        with pytest.raises(Exception):
            ThresholdConfig(yellow=0)


class TestNetworkConfig:
    def test_defaults(self):
        cfg = NetworkConfig()
        assert cfg.team_number == 6328
        assert cfg.robot_address == "10.63.28.2"

    def test_team_number_min(self):
        cfg = NetworkConfig(team_number=1)
        assert cfg.team_number == 1

    def test_team_number_max(self):
        cfg = NetworkConfig(team_number=9999)
        assert cfg.team_number == 9999

    def test_team_number_below_min_raises(self):
        with pytest.raises(Exception):
            NetworkConfig(team_number=0)

    def test_team_number_above_max_raises(self):
        with pytest.raises(Exception):
            NetworkConfig(team_number=10000)


class TestSettings:
    def test_defaults(self):
        s = Settings()
        assert isinstance(s.gpio, GPIOConfig)
        assert isinstance(s.led, LEDConfig)
        assert isinstance(s.thresholds, ThresholdConfig)
        assert isinstance(s.colors, ColorConfig)
        assert isinstance(s.network, NetworkConfig)
        assert isinstance(s.web, WebConfig)


class TestLoadSaveSettings:
    def test_load_nonexistent_returns_defaults(self, tmp_path):
        path = tmp_path / "missing.yaml"
        s = load_settings(path)
        assert s.network.team_number == 6328

    def test_save_and_reload(self, tmp_path):
        path = tmp_path / "config.yaml"
        s = Settings()
        s.network.team_number = 1234
        s.led.brightness = 100
        save_settings(s, path)

        loaded = load_settings(path)
        assert loaded.network.team_number == 1234
        assert loaded.led.brightness == 100

    def test_load_partial_yaml(self, tmp_path):
        path = tmp_path / "partial.yaml"
        path.write_text(yaml.dump({"network": {"team_number": 5555}}))
        s = load_settings(path)
        assert s.network.team_number == 5555
        # Unspecified sections should stay at defaults
        assert s.led.brightness == 200

    def test_save_creates_valid_yaml(self, tmp_path):
        path = tmp_path / "out.yaml"
        save_settings(Settings(), path)
        with open(path) as f:
            data = yaml.safe_load(f)
        assert "network" in data
        assert data["network"]["team_number"] == 6328
