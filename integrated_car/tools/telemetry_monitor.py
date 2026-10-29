#!/usr/bin/env python3
"""
Laptop-side telemetry monitor and command console for the robotic car.

    pip install "paho-mqtt>=2.0"
    python telemetry_monitor.py                  # broker on this laptop
    python telemetry_monitor.py --broker 1.2.3.4 --root robocar/p2-4

Every payload is packed binary (Change Requirement 1, layouts in packets.py).
The monitor decodes them: live telemetry on one line, events as they arrive.

Commands you type are packed into 1 byte before they are sent:
    start | stop | estop | speed N (cm/s, the car caps it at 20) | reset_peak | ping

Barcode map (the 4 letters the instructor announces):
    map F Z W X                           letters for LEFT RIGHT FORWARD U-TURN
    map A=forward B=left C=right D=uturn  letter=move pairs, any order
    map show                              what the broker stores and the car holds
    map clear                             remove the map stored on the broker
The map is sent as 3 bytes, QoS 1 and retained, so the broker hands it to the
car again whenever the car reconnects or restarts. The car answers on
cmd/barcode_map/ack, and its heartbeat says which map it holds: the monitor
prints [car map] whenever that changes.

Every received message is also appended to telemetry_log.csv for the report:
time, topic, payload as hex, and the decoded meaning.
"""
import argparse
import csv
import random
import sys
import threading
import time

import paho.mqtt.client as mqtt

import barcode_map as bm
import packets as pk

parser = argparse.ArgumentParser()
parser.add_argument("--broker", default="localhost")
parser.add_argument("--port", type=int, default=1883)
parser.add_argument("--root", default="robocar/p2-4")
parser.add_argument("--log", default="telemetry_log.csv")
args = parser.parse_args()

ACK_WAIT_S = 3.0

MAP_HELP = ("map F Z W X  (letters for LEFT RIGHT FORWARD U-TURN) | "
            "map A=forward B=left C=right D=uturn | map show | map clear")
CMD_HELP = "start, stop, estop, speed N, reset_peak, ping"

log_file = open(args.log, "a", newline="")
log = csv.writer(log_file)
last_live_seq = None
lost_live = 0
map_sent_at = None              # time the last map went out, until its ack
broker_map = None               # last payload seen on cmd/barcode_map (None = not seen)
car_map = None                  # (held, 20 map bits) from the latest heartbeat
link = None                     # (reconnects, pub_fail, cmd_drop, ev_drop) last printed
rssi = None
enc = (pk.Unwrap(16), pk.Unwrap(16))
enc_now = (None, None)
dist = pk.Unwrap(14, signed_start=False)       # distance wraps every 163.84 m
none_beats = 0                  # heartbeats in a row saying "no map"


def describe_car_map(held, bits):
    if not held:
        return "none - the car has no barcode map (send it with: map ...)"
    return bm.describe_map(pk.map_letters(bits))


def on_connect(client, userdata, flags, reason_code, properties):
    print(f"[monitor] connected to {args.broker}:{args.port} ({reason_code})")
    client.subscribe(f"{args.root}/#", qos=1)


def show_live(payload):
    global last_live_seq, lost_live
    try:
        seq = pk.unpack("live", payload)["seq"]
        text = pk.describe_live(payload)
    except ValueError as e:
        print(f"\n[telemetry/live] {payload.hex(' ')} ({e})")
        return
    if last_live_seq is not None and seq != last_live_seq:   # same seq = duplicate
        lost_live += (seq - last_live_seq - 1) % 256
    last_live_seq = seq
    total = dist.update(pk.unpack("live", payload)["dist"])
    if total != pk.unpack("live", payload)["dist"]:
        text += f" (total {total / 100:.2f} m)"
    encs = "" if enc_now[0] is None else f" enc={enc_now[0]}/{enc_now[1]}"
    sig = "" if rssi is None else f" rssi=-{rssi}dBm"
    sys.stdout.write(f"\r{text}{encs}{sig} lost={lost_live}   ")
    sys.stdout.flush()


def on_heartbeat(payload):
    global car_map, link, rssi, none_beats
    try:
        h = pk.unpack("heartbeat", payload)
    except ValueError as e:
        print(f"\n[heartbeat] {payload.hex(' ')} ({e})")
        return
    rssi = h["rssi"]
    held = (h["map_held"], h["map"] if h["map_held"] else 0)
    # Right after a reboot the first heartbeat can beat the retained map, so
    # "no map" is only reported when two heartbeats in a row say so.
    none_beats = none_beats + 1 if not held[0] else 0
    if held != car_map and (held[0] or none_beats >= 2):
        car_map = held
        print(f"\n[car map] {describe_car_map(*held)}")
    counters = (h["reconnects"], h["pub_fail"], h["cmd_drop"], h["ev_drop"])
    if counters != link:
        if link is not None:
            print(f"\n[link] reconnects {counters[0]}, publish fails {counters[1]}, "
                  f"commands dropped {counters[2]}, events dropped {counters[3]}")
        link = counters


def on_message(client, userdata, msg):
    global map_sent_at, broker_map, enc_now, last_live_seq, rssi
    sub = msg.topic[len(args.root) + 1:]
    text = pk.describe(sub, msg.payload)
    log.writerow([f"{time.time():.3f}", sub, msg.payload.hex(" "), text])
    log_file.flush()

    if sub == "status":
        last_live_seq = None            # the car (re)connected or went away:
        for u in (*enc, dist):          # its counters may have restarted
            u.reset()
        enc_now, rssi = (None, None), None   # hide the old car's values until new ones arrive
        print(f"\n[{sub}] {text}")
    elif sub == "telemetry/live":
        show_live(msg.payload)
    elif sub == "telemetry/odometry":
        try:
            o = pk.unpack("odom", msg.payload)
            enc_now = (enc[0].update(o["enc_l"]), enc[1].update(o["enc_r"]))
        except ValueError:
            print(f"\n[telemetry/odometry] {text}")
    elif sub == "heartbeat":
        on_heartbeat(msg.payload)
    elif sub == "cmd/barcode_map":
        broker_map = msg.payload
        where = "stored on broker" if msg.retain else "sent"
        print(f"\n[{sub}] {where}: {text}")
    else:
        print(f"\n[{sub}] {text}")
        if sub == "cmd/barcode_map/ack":
            map_sent_at = None


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
    held = "no heartbeat yet" if car_map is None else describe_car_map(*car_map)
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


def send_command(client, line):
    try:
        payload = pk.encode_command(line)
    except ValueError as e:
        print(f"\n[cmd] not sent: {e}")
        return
    if line.lower().split()[0] == "speed" and (payload[0] & 0x1F) > pk.SPEED_MAX_CMS:
        print(f"\n[cmd] note: the car caps speed at {pk.SPEED_MAX_CMS} cm/s")
    client.publish(f"{args.root}/cmd", payload, qos=1)


print(f"Type a command and press Enter ({CMD_HELP}),\n"
      f"or a barcode map: {MAP_HELP}. Ctrl+C to quit.")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                     client_id=f"robocar-monitor-{random.randrange(1 << 16):04x}")
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
            send_command(client, line.strip())
except KeyboardInterrupt:
    pass
finally:
    client.loop_stop()
    log_file.close()
