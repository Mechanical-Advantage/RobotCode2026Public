"""Additional API tests covering edge cases and alternative code paths."""

import pytest
from fastapi.testclient import TestClient

from hub_counter.config.settings import Settings
from hub_counter.gpio.ball_counter import BallCounter
from hub_counter.web.routes.api import set_dependencies
from hub_counter.web.server import create_app


@pytest.fixture
def bare_client():
    """Test client with no dependencies set (all None)."""
    app = create_app()
    set_dependencies(
        ball_counter=None,
        nt_client=None,
        led_controller=None,
        settings=None,
        save_settings_fn=None,
    )
    with TestClient(app) as c:
        yield c


@pytest.fixture
def full_client():
    """Test client with all injectable dependencies."""
    app = create_app()
    ball_counter = BallCounter()
    settings = Settings()
    set_dependencies(
        ball_counter=ball_counter,
        nt_client=None,
        led_controller=None,
        settings=settings,
        save_settings_fn=lambda s: None,
    )
    with TestClient(app) as c:
        yield c, ball_counter, settings


# ── "No dependency" fallback paths ────────────────────────────────────────────

def test_get_counts_no_counter(bare_client):
    response = bare_client.get("/api/counts")
    assert response.status_code == 200
    assert response.json()["total"] == 0


def test_reset_counts_no_counter(bare_client):
    response = bare_client.post("/api/counts/reset")
    assert response.status_code == 200
    assert response.json()["success"] is False


def test_get_config_no_settings(bare_client):
    response = bare_client.get("/api/config")
    assert response.status_code == 200
    assert response.json() == {}


def test_update_config_no_settings(bare_client):
    response = bare_client.patch("/api/config", json={"team_number": 999})
    assert response.status_code == 200
    assert response.json()["success"] is False


def test_simulate_ball_no_counter(bare_client):
    response = bare_client.post("/api/test/simulate-ball/0")
    assert response.status_code == 200
    assert response.json()["success"] is False


def test_dismiss_external_no_app(bare_client):
    response = bare_client.post("/api/external/dismiss")
    assert response.status_code == 200
    assert response.json()["success"] is False


# ── Config update variations ──────────────────────────────────────────────────

def test_update_config_robot_address(full_client):
    c, _, settings = full_client
    response = c.patch("/api/config", json={"robot_address": "10.0.0.1"})
    assert response.status_code == 200
    assert response.json()["success"] is True
    assert settings.network.robot_address == "10.0.0.1"


def test_update_config_colors(full_client):
    c, _, settings = full_client
    response = c.patch(
        "/api/config",
        json={"colors": {"red": [10, 20, 30], "yellow": [40, 50, 60], "blue": [70, 80, 90]}},
    )
    assert response.status_code == 200
    assert response.json()["success"] is True
    assert settings.colors.red == (10, 20, 30)


def test_update_config_no_changes(full_client):
    """Patch with no fields → updated=False."""
    c, _, _ = full_client
    response = c.patch("/api/config", json={})
    assert response.status_code == 200
    assert response.json()["updated"] is False


# ── simulate-ball edge cases ───────────────────────────────────────────────────

def test_simulate_ball_out_of_range(full_client):
    c, _, _ = full_client
    response = c.post("/api/test/simulate-ball/5")
    assert response.status_code == 200
    assert response.json()["success"] is False


def test_simulate_multiple_channels(full_client):
    c, counter, _ = full_client
    for ch in range(4):
        response = c.post(f"/api/test/simulate-ball/{ch}")
        assert response.json()["success"] is True
    assert counter.counts.total == 4


# ── Status endpoint ────────────────────────────────────────────────────────────

def test_get_status_no_dependencies(bare_client):
    response = bare_client.get("/api/status")
    assert response.status_code == 200
    data = response.json()
    assert data["nt_connected"] is False
    assert data["led_active"] is False
    assert data["counts"]["total"] == 0
