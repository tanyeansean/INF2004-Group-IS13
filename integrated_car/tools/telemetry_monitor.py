#!/usr/bin/env python3
"""
Laptop-side telemetry monitor and command console for the robotic car.

    pip install "paho-mqtt>=2.0"
    python telemetry_monitor.py                  # broker on this laptop
    python telemetry_monitor.py --broker 1.2.3.4 --root robocar/p2-4

Shows live telemetry on one line, prints events as they arrive, and sends
whatever you type as a command (start, stop, estop, speed 25, reset_peak, ping).

Barcode map (the 4 letters the instructor announces):
    map F Z W X                           letters for LEFT RIGHT FORWARD U-TURN
    map A=forward B=left C=right D=uturn  letter=move pairs, any order
    map show                              what the broker stores and the car holds
    map clear                             remove the map stored on the broker
The map is sent as 3 bytes, QoS 1 and retained, so the broker hands it to the
car again whenever the car reconnects or restarts. The car answers on
cmd/barcode_map/ack, and its heartbeat says which map it holds: the monitor
prints [car map] whenever that changes.

Every received message is also appended to telemetry_log.csv for the report
(binary payloads as hex).
"""
import argparse
import csv
import json
import sys
import threading
import time

import paho.mqtt.client as mqtt

import barcode_map as bm

parser = argparse.ArgumentParser()
parser.add_argument("--broker", default="localhost")
parser.add_argument("--port", type=int, default=1883)
parser.add_argument("--root", default="robocar/p2-4")
parser.add_argument("--log", default="telemetry_log.csv")
args = parser.parse_args()

# Topics whose payload is packed binary (Change Requirement 1), not text.
BINARY_TOPICS = ("cmd/barcode_map", "cmd/barcode_map/ack",
                 "event/barcode", "event/junction")
ACK_WAIT_S = 3.0

MAP_HELP = ("map F Z W X  (letters for LEFT RIGHT FORWARD U-TURN) | "
            "map A=forward B=left C=right D=uturn | map show | map clear")

log_file = open(args.log, "a", newline="")
log = csv.writer(log_file)
last_live_seq = None
lost_live = 0
map_sent_at = None              # time the last map went out, until its ack
broker_map = None               # last payload seen on cmd/barcode_map (None = not seen)
car_map = None                  # "map" field of the latest heartbeat (None = no heartbeat yet)


def describe_hex_map(hexed):
    if not hexed:
        return "none - the car has no barcode map (send it with: map ...)"
    try:
        return bm.describe_map(bm.decode_map(bytes.fromhex(hexed)))
    except ValueError as e:
        return f"{hexed} ({e})"


def describe_binary(sub, payload, retained):
    if sub == "cmd/barcode_map":
        if not payload:
            return "empty (stored map removed)"
        where = "stored on broker" if retained else "sent"
        try:
            return f"{where}: {bm.describe_map(bm.decode_map(payload))}"
        except ValueError as e:
            return f"{where}: {payload.hex(' ')} - not a valid map ({e})"
    if sub == "cmd/barcode_map/ack":
        return bm.describe_ack(payload)
    return bm.describe_event(sub, payload)


def on_connect(client, userdata, flags, reason_code, properties):
    print(f"[monitor] connected to {args.broker}:{args.port} ({reason_code})")
    client.subscribe(f"{args.root}/#", qos=1)


def on_message(client, userdata, msg):
    global last_live_seq, lost_live, map_sent_at, broker_map, car_map
    sub = msg.topic[len(args.root) + 1:]

    if sub in BINARY_TOPICS:
        log.writerow([f"{time.time():.3f}", sub, msg.payload.hex(" ")])
        log_file.flush()
        print(f"\n[{sub}] {describe_binary(sub, msg.payload, msg.retain)}")
        if sub == "cmd/barcode_map":
            broker_map = msg.payload
        elif sub == "cmd/barcode_map/ack":
            map_sent_at = None
        return

    text = msg.payload.decode(errors="replace")
    log.writerow([f"{time.time():.3f}", sub, text])
    log_file.flush()

    if sub == "telemetry/live":
        try:
            d = json.loads(text)
        except json.JSONDecodeError:
            print(f"\n[bad json] {text}")
            return
        if last_live_seq is not None and d["seq"] > last_live_seq + 1:
            lost_live += d["seq"] - last_live_seq - 1
        last_live_seq = d["seq"]
        m, ln, imu = d["mot"], d["line"], d["imu"]
        sys.stdout.write(
            f"\r{d['state']:<13} v={m['vl']:5.1f}/{m['vr']:5.1f} "
            f"enc={m['el']}/{m['er']} dist={m['dist']:7.1f}cm "
            f"line={ln['bits']:03b} pos={ln['pos']:+d} "
            f"pitch={imu['pitch']:+5.1f} peak={imu['peak']:.2f}cm "
            f"lost={lost_live}   ")
        sys.stdout.flush()
    elif sub == "heartbeat":
        # Logged to CSV; only a change in the car's map is printed.
        try:
            held = json.loads(text).get("map")
        except (json.JSONDecodeError, AttributeError):
            return
        if held is not None and held != car_map:
            car_map = held
            print(f"\n[car map] {describe_hex_map(held)}")
    else:
        print(f"\n[{sub}] {text}")


def warn_if_no_ack(sent_at):
    if map_sent_at == sent_at:
        print(f"\n[map] no ack after {ACK_WAIT_S:.0f} s. If the car is offline it gets the "
              "map when it connects (watch for [car map]); if it is online, send the map again.")


def show_maps():
    if broker_map is None:
        stored = "nothing seen yet"
    elif not broker_map:
        stored = "none (cleared)"
    else:
        try:
            stored = bm.describe_map(bm.decode_map(broker_map))
        except ValueError as e:
            stored = f"{broker_map.hex(' ')} ({e})"
    held = "no heartbeat yet" if car_map is None else describe_hex_map(car_map)
    print(f"\n[map] stored on broker: {stored}\n[map] car holds:        {held}")


def send_map(client, words):
    global map_sent_at
    sub = [w.lower() for w in words]
    if sub in ([], ["help"]):
        print(f"\n[map] {MAP_HELP}")
        return
    if sub == ["show"]:
        show_maps()
        return
    if sub == ["clear"]:
        client.publish(f"{args.root}/cmd/barcode_map", b"", qos=1, retain=True)
        print("\n[map] stored map removed from the broker. The car keeps the map it has "
              "and answers 'payload not 3 bytes' - that is expected.")
        return
    try:
        letters = bm.parse_map_args(words)
    except (ValueError, TypeError) as e:
        print(f"\n[map] not sent: {e}")
        return
    payload = bm.encode_map(*letters)
    print(f"\n[map] {bm.describe_map(letters)}  ->  payload {payload.hex(' ').upper()}")
    sent_at = time.time()
    map_sent_at = sent_at
    client.publish(f"{args.root}/cmd/barcode_map", payload, qos=1, retain=True)
    timer = threading.Timer(ACK_WAIT_S, warn_if_no_ack, args=(sent_at,))
    timer.daemon = True
    timer.start()


print("Type a command and press Enter (start, stop, estop, speed 25, reset_peak, ping),\n"
      f"or a barcode map: {MAP_HELP}. Ctrl+C to quit.")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="robocar-monitor")
client.on_connect = on_connect
client.on_message = on_message
client.connect(args.broker, args.port, keepalive=30)
client.loop_start()
try:
    for line in sys.stdin:
        words = line.split()
        if not words:
            continue
        if words[0].lower() == "map":
            send_map(client, words[1:])
        else:
            client.publish(f"{args.root}/cmd", line.strip(), qos=1)
except KeyboardInterrupt:
    pass
finally:
    client.loop_stop()
    log_file.close()
