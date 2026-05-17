"""Step 1 acceptance: defaults flow through, JSON overlay wins, env wins over JSON,
runtime set() wins over env, and RESTART_REQUIRED keys refuse runtime set()."""
import json

import pytest

from config.config_manager import ConfigManager


def test_default_lookup():
    cm = ConfigManager(user_config_path="/nonexistent/path.json", read_env=False)
    assert cm.get("LASERCUBE_DEFAULT_IP") == "169.254.40.83"
    assert cm.get("LASERCUBE_BUFFER_SIZE") == 6000
    assert cm.get("DETECTION_MIN_AREA") == 5


def test_unknown_key_raises_without_default():
    cm = ConfigManager(user_config_path="/nonexistent/path.json", read_env=False)
    with pytest.raises(KeyError):
        cm.get("NOT_A_SETTING")
    assert cm.get("NOT_A_SETTING", "fallback") == "fallback"


def test_json_overlay(tmp_path):
    cfg = tmp_path / "config.json"
    cfg.write_text(json.dumps({"LASERCUBE_DEFAULT_IP": "10.0.0.5",
                                "DETECTION_MIN_AREA": 99}))
    cm = ConfigManager(user_config_path=cfg, read_env=False)
    assert cm.get("LASERCUBE_DEFAULT_IP") == "10.0.0.5"
    assert cm.get("DETECTION_MIN_AREA") == 99


def test_env_overrides_json(tmp_path, monkeypatch):
    cfg = tmp_path / "config.json"
    cfg.write_text(json.dumps({"LASERCUBE_DEFAULT_IP": "10.0.0.5"}))
    monkeypatch.setenv("NMS_LASERCUBE_DEFAULT_IP", "192.168.99.99")
    cm = ConfigManager(user_config_path=cfg, read_env=True)
    assert cm.get("LASERCUBE_DEFAULT_IP") == "192.168.99.99"


def test_env_coerces_to_default_type(monkeypatch):
    monkeypatch.setenv("NMS_LASERCUBE_BUFFER_SIZE", "4096")
    monkeypatch.setenv("NMS_SAFETY_DEV_MODE", "true")
    cm = ConfigManager(user_config_path="/none", read_env=True)
    assert cm.get("LASERCUBE_BUFFER_SIZE") == 4096
    assert cm.get("SAFETY_DEV_MODE") is True


def test_runtime_set_for_mutable_setting():
    cm = ConfigManager(user_config_path="/none", read_env=False)
    cm.set("DETECTION_THRESHOLD", 42)
    assert cm.get("DETECTION_THRESHOLD") == 42


def test_runtime_set_refuses_restart_required():
    cm = ConfigManager(user_config_path="/none", read_env=False)
    with pytest.raises(ValueError):
        cm.set("LASERCUBE_DEFAULT_IP", "10.0.0.5")


def test_save_user_config_round_trip(tmp_path):
    cfg = tmp_path / "config.json"
    cm = ConfigManager(user_config_path=cfg, read_env=False)
    cm.save_user_config({"DETECTION_MIN_AREA": 7, "LASERCUBE_DEFAULT_IP": "10.0.0.99"})
    cm2 = ConfigManager(user_config_path=cfg, read_env=False)
    assert cm2.get("DETECTION_MIN_AREA") == 7
    assert cm2.get("LASERCUBE_DEFAULT_IP") == "10.0.0.99"


def test_is_runtime_mutable():
    cm = ConfigManager(user_config_path="/none", read_env=False)
    assert cm.is_runtime_mutable("DETECTION_THRESHOLD") is True
    assert cm.is_runtime_mutable("LASERCUBE_DEFAULT_IP") is False


def test_all_returns_snapshot():
    cm = ConfigManager(user_config_path="/none", read_env=False)
    snap = cm.all()
    assert "LASERCUBE_DEFAULT_IP" in snap
    assert "TARGETING_MODE" in snap
    assert snap["TARGETING_MODE"] == "gopro_only"


def test_unknown_json_keys_silently_ignored(tmp_path):
    cfg = tmp_path / "config.json"
    cfg.write_text(json.dumps({"NOT_A_SETTING": "x", "DETECTION_MIN_AREA": 11}))
    cm = ConfigManager(user_config_path=cfg, read_env=False)
    assert cm.get("DETECTION_MIN_AREA") == 11
    with pytest.raises(KeyError):
        cm.get("NOT_A_SETTING")
