"""Tests for the HMI web server, with a real HTTP round trip on a random local port. No ROS."""

import json
import urllib.error
import urllib.request

import pytest

from rover_hmi.server import HmiServer, clamp_command


@pytest.fixture
def server(tmp_path):
    page = tmp_path / "index.html"
    page.write_text("<h1>rover</h1>")
    received = []
    s = HmiServer(page, received.append, lambda: {"distance_m": 1.25}, host="127.0.0.1", port=0,
                  max_linear=0.5, max_angular=2.0)
    s.received = received
    s.start()
    yield s
    s.stop()


def request(server, path, body=None):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(f"http://127.0.0.1:{server.port}{path}", data=data, method="POST" if data is not None else "GET")
    with urllib.request.urlopen(req, timeout=3) as r:
        return r.status, r.read()


def test_serves_the_page(server):
    status, body = request(server, "/")
    assert status == 200 and b"rover" in body


def test_status_has_distance_and_limits(server):
    _, body = request(server, "/status")
    assert json.loads(body) == {"max_linear": 0.5, "max_angular": 2.0, "distance_m": 1.25}


def test_cmd_is_passed_on(server):
    request(server, "/cmd", {"vx": 0.2, "vy": -0.1, "wz": 1.0})
    assert server.received == [(0.2, -0.1, 1.0)]


def test_cmd_is_clamped(server):
    request(server, "/cmd", {"vx": 9, "vy": -9, "wz": 99})
    assert server.received == [(0.5, -0.5, 2.0)]


def test_stop(server):
    request(server, "/stop", {})
    assert server.received == [(0.0, 0.0, 0.0)]


def test_bad_json_is_rejected_and_not_passed_on(server):
    req = urllib.request.Request(f"http://127.0.0.1:{server.port}/cmd", data=b"not json", method="POST")
    with pytest.raises(urllib.error.HTTPError) as err:
        urllib.request.urlopen(req, timeout=3)
    assert err.value.code == 400
    assert server.received == []


@pytest.mark.parametrize("data, expected", [
    ({}, (0.0, 0.0, 0.0)),
    ({"vx": "abc", "wz": None}, (0.0, 0.0, 0.0)),
    ({"vx": float("nan")}, (0.0, 0.0, 0.0)),
    ({"vx": -0.3}, (-0.3, 0.0, 0.0)),
])
def test_clamp_command_handles_junk(data, expected):
    assert clamp_command(data, 0.6, 3.0) == expected
