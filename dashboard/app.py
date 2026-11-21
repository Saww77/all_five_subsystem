"""Serve the car status and route configuration on localhost."""

import argparse
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import paho.mqtt.client as mqtt

from protocol import decode_routes, decode_status, encode_routes


class Dashboard:
    def __init__(self, broker, port, topic):
        self.topic = topic.rstrip("/")
        self.lock = threading.Lock()
        self.connected = False
        self.available = False
        self.last_update = 0.0
        self.status = None
        self.routes = None
        self.routes_version = None
        self.pending = None
        self.pending_at = 0.0
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        if "MQTT_USERNAME" in os.environ:
            self.client.username_pw_set(os.environ["MQTT_USERNAME"], os.environ.get("MQTT_PASSWORD"))
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.reconnect_delay_set(1, 15)
        self.client.connect_async(broker, port, 30)

    def on_connect(self, client, _data, _flags, reason, _properties):
        with self.lock:
            self.connected = not reason.is_failure
        if not reason.is_failure:
            client.subscribe([(self.topic + "/state", 0), (self.topic + "/online", 0),
                              (self.topic + "/routes/state", 1)])

    def on_disconnect(self, _client, _data, _flags, _reason, _properties):
        with self.lock:
            self.connected = False
            self.available = False

    def on_message(self, _client, _data, message):
        try:
            with self.lock:
                suffix = message.topic[len(self.topic):]
                if suffix == "/state":
                    self.status = decode_status(message.payload)
                    self.last_update = time.monotonic()
                elif suffix == "/online":
                    self.available = message.payload == b"\x01"
                    if not self.available:
                        self.last_update = 0.0
                elif suffix == "/routes/state":
                    self.routes = decode_routes(message.payload)
                    self.routes_version = message.payload.hex()
                    if self.pending == self.routes_version:
                        self.pending = None
        except ValueError:
            return

    def snapshot(self):
        with self.lock:
            if self.pending is not None and time.monotonic() - self.pending_at > 10:
                self.pending = None
            return {
                "broker": self.connected,
                "online": self.is_online(),
                "status": self.status,
                "routes": self.routes,
                "route_version": self.routes_version,
                "pending": self.pending,
            }

    def is_online(self):
        return self.connected and self.available and time.monotonic() - self.last_update <= 8

    def send_stop(self):
        with self.lock:
            if not self.is_online():
                return 503, {"error": "Wait for fresh telemetry."}
            result = self.client.publish(self.topic + "/command", b"\x01", qos=1)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            return 503, {"error": "Could not queue the stop request."}
        return 202, {"message": "Stop requested. Check the vehicle state for confirmation."}

    def save_routes(self, body):
        if not isinstance(body, dict):
            return 400, {"error": "Invalid route document."}
        try:
            payload = encode_routes(body.get("routes"))
        except ValueError as error:
            return 400, {"error": str(error)}
        with self.lock:
            if not self.is_online() or self.routes_version is None:
                return 503, {"error": "Wait for the car's current route table."}
            if body.get("version") != self.routes_version:
                return 409, {"error": "The car's route table changed. Reload the mapping and retry."}
            if payload.hex() == self.routes_version:
                return 200, {"message": "Mapping already active."}
            if self.pending is not None:
                return 409, {"error": "The car is still confirming the previous update."}
            result = self.client.publish(self.topic + "/routes/set", payload, qos=1, retain=True)
            if result.rc != mqtt.MQTT_ERR_SUCCESS:
                return 503, {"error": "Could not queue the mapping."}
            self.pending = payload.hex()
            self.pending_at = time.monotonic()
        return 202, {"message": "Waiting for the car to confirm the mapping."}


def handler_for(dashboard):
    page = Path(__file__).with_name("index.html").read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format, *_args):
            return

        def reply(self, status, media_type, body):
            self.send_response(status)
            self.send_header("Content-Type", media_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            self.wfile.write(body)

        def json_reply(self, status, data):
            self.reply(status, "application/json", json.dumps(data).encode())

        def do_GET(self):
            if self.path == "/":
                self.reply(200, "text/html; charset=utf-8", page)
            elif self.path == "/api/status":
                self.json_reply(200, dashboard.snapshot())
            else:
                self.json_reply(404, {"error": "Unknown endpoint."})

        def do_POST(self):
            if self.headers.get_content_type() != "application/json":
                self.json_reply(415, {"error": "Expected application/json."})
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if not 0 < size <= 2048:
                    raise ValueError
                body = json.loads(self.rfile.read(size))
            except (ValueError, UnicodeDecodeError):
                self.json_reply(400, {"error": "Invalid JSON payload."})
                return
            if self.path == "/api/routes":
                self.json_reply(*dashboard.save_routes(body))
            elif self.path == "/api/stop":
                self.json_reply(*dashboard.send_stop())
            else:
                self.json_reply(404, {"error": "Unknown endpoint."})

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broker", required=True)
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument("--topic", default="robot/se09-car/v3")
    parser.add_argument("--http-port", type=int, default=8080)
    options = parser.parse_args()

    dashboard = Dashboard(options.broker, options.mqtt_port, options.topic)
    dashboard.client.loop_start()
    try:
        with ThreadingHTTPServer(("127.0.0.1", options.http_port), handler_for(dashboard)) as server:
            print(f"Car dashboard running at http://127.0.0.1:{options.http_port}", flush=True)
            server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        dashboard.client.disconnect()
        dashboard.client.loop_stop()


if __name__ == "__main__":
    main()
