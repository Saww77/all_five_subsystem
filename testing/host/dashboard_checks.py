"""Exercise dashboard REST endpoints through a real loopback MQTT broker."""

import json
import os
import queue
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import paho.mqtt.client as mqtt

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "dashboard"))
from app import Dashboard, handler_for
from protocol import decode_routes, decode_status, encode_routes


TOPIC = "test/se09/v3"
SAMPLE = bytes.fromhex("53 e9 2d 61 45 23 61 1a 1a b0 34 b9 01")


def until(read, check, timeout=6):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = read()
        if check(result):
            return result
        time.sleep(0.05)
    raise AssertionError("timed out waiting for " + str(check))


def port_available():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def accepts(port):
    try:
        with socket.create_connection(("127.0.0.1", port), 0.2):
            return True
    except OSError:
        return False


def client_for(port):
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.connect("127.0.0.1", port)
    client.loop_start()
    return client


def publish(client, suffix, payload, retain=False):
    info = client.publish(TOPIC + suffix, payload, qos=1, retain=retain)
    info.wait_for_publish(4)
    assert info.is_published()


def check(broker_path, conf, port):
    flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    broker = subprocess.Popen([broker_path, "-c", str(conf)], stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL, creationflags=flags)
    try:
        until(lambda: accepts(port), bool)
        dashboard = Dashboard("127.0.0.1", port, TOPIC)
        dashboard.client.loop_start()
        robot = client_for(port)
        commands = queue.Queue()
        robot.on_message = lambda _client, _data, message: commands.put(message)
        robot.subscribe([(TOPIC + "/command", 1), (TOPIC + "/routes/set", 1)])

        try:
            with ThreadingHTTPServer(("127.0.0.1", 0), handler_for(dashboard)) as server:
                thread = threading.Thread(target=server.serve_forever, daemon=True)
                thread.start()

                def request(path, data=None):
                    url = f"http://127.0.0.1:{server.server_port}" + path
                    req = urllib.request.Request(url)
                    if data is not None:
                        req.data = json.dumps(data).encode()
                        req.add_header("Content-Type", "application/json")
                    try:
                        with urllib.request.urlopen(req, timeout=3) as response:
                            return response.status, response.read()
                    except urllib.error.HTTPError as response:
                        return response.code, response.read()

                def status():
                    result, body = request("/api/status")
                    assert result == 200
                    return json.loads(body)

                try:
                    code, page = request("/")
                    assert code == 200 and b"Barcode actions" in page
                    assert request("/api/stop", {})[0] == 503
                    until(status, lambda value: value["broker"])
                    publish(robot, "/online", b"\x01", True)
                    publish(robot, "/routes/state", encode_routes({"A": 3, "Z": 4}), True)
                    for _ in range(2):
                        publish(robot, "/state", SAMPLE)
                        time.sleep(0.08)
                    fresh = until(status, lambda value: value["online"] and value["routes"])
                    assert fresh["status"] == decode_status(SAMPLE)
                    assert fresh["status"]["speed_mm_s"] == -109
                    ir_setup = (3 | (2 << 15)).to_bytes(13, "little")
                    publish(robot, "/state", ir_setup)
                    waiting_black = until(status, lambda value: value["status"]["phase"] == "IR setup")
                    assert "matte black and tap GP20" in waiting_black["status"]["setup"]
                    assert waiting_black["status"]["plan"] is None
                    assert fresh["routes"]["Z"] == 4
                    assert request("/api/routes", {"version": fresh["route_version"],
                                                    "routes": {"A": 7}})[0] == 400
                    assert request("/api/routes", {"version": "stale", "routes": {"A": 1}})[0] == 409
                    new_routes = dict(fresh["routes"], A=1, X=2)
                    code, _ = request("/api/routes", {"version": fresh["route_version"],
                                                      "routes": new_routes})
                    assert code == 202
                    update = commands.get(timeout=5)
                    assert update.topic == TOPIC + "/routes/set"
                    assert decode_routes(update.payload) == new_routes
                    assert status()["pending"] == update.payload.hex()
                    publish(robot, "/routes/state", update.payload, True)
                    until(status, lambda value: value["pending"] is None and value["routes"] == new_routes)

                    observer = client_for(port)
                    try:
                        retained = queue.Queue()
                        observer.on_message = lambda _client, _data, message: retained.put(message)
                        observer.subscribe(TOPIC + "/routes/set", qos=1)
                        after_reconnect = retained.get(timeout=5)
                        assert after_reconnect.retain and after_reconnect.payload == update.payload
                    finally:
                        observer.disconnect()
                        observer.loop_stop()

                    assert request("/api/stop", {})[0] == 202
                    stop = commands.get(timeout=5)
                    assert stop.payload == b"\x01" and not stop.retain and stop.topic.endswith("/command")
                    publish(robot, "/online", b"\x00", True)
                    until(status, lambda value: not value["online"])
                finally:
                    server.shutdown()
                    thread.join()
        finally:
            robot.disconnect()
            robot.loop_stop()
            dashboard.client.disconnect()
            dashboard.client.loop_stop()
    finally:
        broker.terminate()
        broker.wait(timeout=5)


if __name__ == "__main__":
    default = r"C:\Program Files\Mosquitto\mosquitto.exe" if os.name == "nt" else "mosquitto"
    target = os.environ.get("MOSQUITTO", default)
    executable = shutil.which(target)
    if executable is None:
        raise SystemExit("Set MOSQUITTO to the broker executable.")
    selected_port = port_available()
    with tempfile.TemporaryDirectory() as temp:
        config = Path(temp, "mosquitto.conf")
        config.write_text(f"listener {selected_port} 127.0.0.1\nallow_anonymous true\n", encoding="utf8")
        check(executable, config, selected_port)
    print("PASS dashboard HTTP, MQTT status, retained route update, car acknowledgment and stop")
