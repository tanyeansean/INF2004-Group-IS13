#!/usr/bin/env python3
"""
Robocar dashboard: buttons and gauges in the browser instead of typing
commands into telemetry_monitor.py.

    pip install "paho-mqtt>=2.0"
    python tools/dashboard.py              # then open http://localhost:8765
    python tools/dashboard.py --broker 192.168.43.100 --port 1883 --http 8765

This script is the bridge: it subscribes to the car's topics on the MQTT
broker, decodes every packed message with packets.py, and streams them to
the page (dashboard.html, same folder) as JSON. Buttons on the page come
back as POST requests, which it turns into the 1-byte commands and the
3-byte barcode map the car understands. Everything is logged to a CSV
(time, topic, hex, decoded text), the same format as the monitor.

No extra packages beyond paho-mqtt: the web side is the standard library.
"""
import argparse
import csv
import json
import os
import queue
import random
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import paho.mqtt.client as mqtt

import barcode_map as bm
import packets as pk

HERE = os.path.dirname(os.path.abspath(__file__))

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--broker", default="localhost", help="MQTT broker host (the laptop running mosquitto)")
ap.add_argument("--port", type=int, default=1883)
ap.add_argument("--root", default="robocar/p2-4", help="topic root, as in comms_config.h")
ap.add_argument("--http", type=int, default=8765, help="port the dashboard page is served on")
ap.add_argument("--log", default="dashboard_log.csv")
args = ap.parse_args()

# ---------------------------------------------------------------- state --
lock = threading.Lock()
clients = []                    # one queue per open browser tab (SSE)
state = {                       # latest of everything, sent to a new tab
    "broker": f"{args.broker}:{args.port}", "root": args.root, "mqtt": False,
    "status": None, "live": None, "odom": None, "heartbeat": None,
    "broker_map": None, "car_map": None, "summary": None, "summary_check": None,
    "ack": None, "map_ack": None, "lost": 0,
}
last_live_seq = None
enc = (pk.Unwrap(16), pk.Unwrap(16))
dist = pk.Unwrap(14, signed_start=False)
raw = None                      # laptop-side tally of the run, for the summary check
none_beats = 0

log_file = open(args.log, "a", newline="")
log = csv.writer(log_file)


def push(kind, data):
    """Send one message to every open tab."""
    msg = json.dumps({"kind": kind, "t": time.time(), "data": data})
    with lock:
        for q in clients:
            q.put(msg)


# ---------------------------------------------------------- raw tally --
def raw_reset():
    return {"barcodes": [], "unreadable": 0, "junction": 0, "unmapped": 0, "humps": 0,
            "peak": 0.0, "obstacles": 0, "obst_min": 255, "max_speed": 0.0, "dist": 0}


def raw_note(sub, payload):
    global raw
    if sub == "event/state":
        e = pk.unpack("ev_state", payload)
        if e["to"] in pk.RUNNING_STATES and e["from"] not in pk.RUNNING_STATES:
            raw = raw_reset()
        return
    if raw is None:
        return
    if sub == "event/barcode":
        letter, _ = bm.decode_letter_move(payload)
        if letter is None:
            raw["unreadable"] += 1
        else:
            raw["barcodes"].append(letter)
    elif sub == "event/junction":
        _, move = bm.decode_letter_move(payload)
        raw["unmapped" if move == "UNMAPPED" else "junction"] += 1
    elif sub == "event/hump":
        e = pk.scaled("ev_hump", payload)
        raw["humps"] += 1
        raw["peak"] = max(raw["peak"], e["peak"])
    elif sub == "event/obstacle":
        e = pk.unpack("ev_obstacle", payload)
        raw["obstacles"] += 1
        raw["obst_min"] = min(raw["obst_min"], e["dist"])
    elif sub == "telemetry/live":
        d = pk.scaled("live", payload)
        raw["max_speed"] = max(raw["max_speed"], abs(d["speed_l"]), abs(d["speed_r"]))
        raw["dist"] = int(d["dist"])


def summary_check(s, retained):
    """Compare the car's summary with the raw events this bridge saw."""
    if raw is None:
        return "stored on the broker from an earlier run" if retained else \
               "the run started before the dashboard did: nothing to compare"
    pairs = [("barcodes", int(s["n_barcode"]), len(raw["barcodes"]), 4),
             ("letters", "".join(pk.summary_letters(s)), "".join(raw["barcodes"][:pk.SUMMARY_LETTERS]), 0),
             ("unreadable", int(s["n_unreadable"]), raw["unreadable"], 3),
             ("junction moves", int(s["n_junction"]), raw["junction"], 4),
             ("unmapped", int(s["n_unmapped"]), raw["unmapped"], 3),
             ("humps", int(s["n_hump"]), raw["humps"], 3),
             ("highest hump cm", round(s["peak"], 1), round(raw["peak"], 1), 0),
             ("obstacles", int(s["n_obst"]), raw["obstacles"], 3),
             ("closest obstacle cm", int(s["obst_min"]), raw["obst_min"], 0),
             ("distance cm", int(s["dist"]), raw["dist"], 0),
             ("top speed cm/s", round(s["max_speed"], 1), round(raw["max_speed"], 1), 0)]
    slack = int(s["max_speed"] * 0.25) + 1          # live data is sampled 5x a second
    diffs = []
    for what, car, laptop, bits in pairs:
        if bits and car == (1 << bits) - 1 and laptop >= car:
            continue                                # saturated count
        if what == "distance cm" and 0 <= car - laptop <= slack:
            continue
        if car != laptop:
            diffs.append(f"{what}: car {car}, raw events {laptop}")
    return "matches the raw data" if not diffs else "DIFFERS: " + "; ".join(diffs)


# ------------------------------------------------------------- MQTT --
def on_connect(client, userdata, flags, reason_code, properties):
    client.subscribe(f"{args.root}/#", qos=1)
    state["mqtt"] = True
    push("mqtt", {"connected": True, "reason": str(reason_code)})


def on_disconnect(client, userdata, flags, reason_code, properties):
    state["mqtt"] = False
    push("mqtt", {"connected": False, "reason": str(reason_code)})


def on_message(client, userdata, msg):
    global last_live_seq, none_beats
    sub = msg.topic[len(args.root) + 1:]
    payload = msg.payload
    text = pk.describe(sub, payload)
    log.writerow([f"{time.time():.3f}", sub, payload.hex(" "), text])
    log_file.flush()
    try:
        raw_note(sub, payload)
    except ValueError:
        pass
    try:
        if sub == "status":
            last_live_seq = None
            for u in (*enc, dist):
                u.reset()
            d = pk.unpack("status", payload)
            state["status"] = {"online": bool(d["online"]), "version": d["version"], "text": text}
            push("status", state["status"])
        elif sub == "telemetry/live":
            d = pk.scaled("live", payload)
            seq = int(d["seq"])
            if last_live_seq is not None and seq != last_live_seq:
                state["lost"] += (seq - last_live_seq - 1) % 256
            last_live_seq = seq
            total = dist.update(int(d["dist"]))
            faults = int(d["faults"])
            state["live"] = {
                "state": pk.name(pk.STATES, int(d["state"])), "speed_l": d["speed_l"], "speed_r": d["speed_r"],
                "dist": total, "line_bits": int(d["line_bits"]), "line_pos": int(d["line_pos"]),
                "on_line": bool(d["on_line"]), "pitch": d["pitch"], "yaw_rate": int(d["yaw_rate"]),
                "motion": pk.name(pk.MOTIONS, int(d["motion"])), "peak": d["peak"],
                "faults": faults, "faults_text": pk.describe_faults(faults), "lost": state["lost"]}
            push("live", state["live"])
        elif sub == "telemetry/odometry":
            o = pk.unpack("odom", payload)
            state["odom"] = {"enc_l": enc[0].update(o["enc_l"]), "enc_r": enc[1].update(o["enc_r"])}
            push("odom", state["odom"])
        elif sub == "heartbeat":
            h = pk.unpack("heartbeat", payload)
            none_beats = 0 if h["map_held"] else none_beats + 1
            if h["map_held"]:
                state["car_map"] = bm.describe_map(pk.map_letters(h["map"]))
            elif none_beats >= 2:
                state["car_map"] = "none"
            state["heartbeat"] = {"rssi": -h["rssi"] if h["rssi"] else None, "uptime": h["uptime"],
                                  "reconnects": h["reconnects"], "pub_fail": h["pub_fail"],
                                  "cmd_drop": h["cmd_drop"], "ev_drop": h["ev_drop"],
                                  "car_map": state["car_map"]}
            push("heartbeat", state["heartbeat"])
        elif sub == "cmd/barcode_map":
            if not payload:
                state["broker_map"] = "none (cleared)"
            else:
                state["broker_map"] = bm.describe_map(bm.decode_map(payload))
            push("broker_map", {"map": state["broker_map"], "retained": msg.retain})
        elif sub == "cmd/barcode_map/ack":
            state["map_ack"] = text
            push("map_ack", {"text": text})
        elif sub == "cmd/ack":
            state["ack"] = text
            push("ack", {"text": text})
        elif sub == "run/summary":
            s = pk.scaled("summary", payload)
            check = summary_check(s, msg.retain)
            state["summary"] = {"text": text, "check": check, "letters": "".join(pk.summary_letters(s)),
                                "fields": {k: (round(v, 1) if isinstance(v, float) else int(v))
                                           for k, v in s.items()}}
            push("summary", state["summary"])
        elif sub.startswith("event/"):
            push("event", {"topic": sub, "text": text})
        elif sub == "cmd":
            push("sent", {"text": f"cmd: {text}"})
    except ValueError as e:
        push("event", {"topic": sub, "text": f"{payload.hex(' ')} ({e})"})


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                     client_id=f"robocar-dashboard-{random.randrange(1 << 16):04x}")
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message


# --------------------------------------------------------------- HTTP --
class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *a):      # keep the console quiet
        pass

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "dashboard.html"), "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/state":
            self._json(200, state)
        elif self.path == "/events":
            q = queue.Queue()
            with lock:
                clients.append(q)
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                self.wfile.write(f"data: {json.dumps({'kind': 'snapshot', 'data': state})}\n\n".encode())
                self.wfile.flush()
                while True:
                    try:
                        msg = q.get(timeout=15)
                        self.wfile.write(f"data: {msg}\n\n".encode())
                    except queue.Empty:
                        self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                with lock:
                    clients.remove(q)
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            body = {}
        if self.path == "/cmd":
            line = str(body.get("line", "")).strip()
            try:
                payload = pk.encode_command(line)
            except ValueError as e:
                self._json(400, {"error": str(e)})
                return
            note = ""
            if line.lower().split()[0] == "speed" and (payload[0] & 0x1F) > pk.SPEED_MAX_CMS:
                note = f"the car caps speed at {pk.SPEED_MAX_CMS} cm/s"
            client.publish(f"{args.root}/cmd", payload, qos=1)
            self._json(200, {"sent": line, "byte": payload.hex().upper(), "note": note})
        elif self.path == "/map":
            words = str(body.get("words", "")).split()
            try:
                letters = bm.parse_map_args(words)
            except (ValueError, TypeError) as e:
                self._json(400, {"error": str(e)})
                return
            payload = bm.encode_map(*letters)
            client.publish(f"{args.root}/cmd/barcode_map", payload, qos=1, retain=True)
            self._json(200, {"sent": bm.describe_map(letters), "bytes": payload.hex(" ").upper()})
        elif self.path == "/map/clear":
            client.publish(f"{args.root}/cmd/barcode_map", b"", qos=1, retain=True)
            self._json(200, {"sent": "stored map removed; the car keeps the map it has"})
        else:
            self._json(404, {"error": "not found"})


def main():
    client.connect_async(args.broker, args.port, keepalive=30)
    client.loop_start()
    httpd = ThreadingHTTPServer(("0.0.0.0", args.http), Handler)
    httpd.daemon_threads = True
    print(f"Dashboard: http://localhost:{args.http}  (broker {args.broker}:{args.port}, "
          f"topics {args.root}/#, log {args.log}). Ctrl+C to quit.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()


if __name__ == "__main__":
    main()
