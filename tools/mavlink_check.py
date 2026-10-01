# /// script
# dependencies = ["pymavlink"]
# ///
"""Act as a ground station against a running krabshell and fly one drone.

    uv run tools/mavlink_check.py              # UAV-4 in london-patrol
    uv run tools/mavlink_check.py --sysid 2 --listen 0.0.0.0:14550

Listens where krabshell sends telemetry (its --mavlink-gcs, default
127.0.0.1:14550), then: lists the drones it hears, uploads a small box
mission to one, reads it back, arms it, starts it, and prints each
MISSION_ITEM_REACHED until it has landed. Stop krabshell's other ground
stations first, or they will fight over the same drone.
"""
import argparse
import os
import sys
import time

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil  # noqa: E402
from pymavlink.dialects.v20 import common as mav  # noqa: E402

p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
p.add_argument("--listen", default="127.0.0.1:14550", help="where krabshell sends telemetry")
p.add_argument("--sysid", type=int, default=4, help="drone to fly (london-patrol: 4 is idle)")
p.add_argument("--box-m", type=float, default=150.0, help="side of the square to fly, metres")
p.add_argument("--alt", type=float, default=40.0, help="altitude, metres above home")
args = p.parse_args()

m = mavutil.mavlink_connection(f"udpin:{args.listen}", source_system=255, source_component=190, dialect="common")
T = args.sysid


def wait(type_, cond=lambda x: True, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        msg = m.recv_match(type=type_, blocking=True, timeout=0.5)
        if msg and msg.get_srcSystem() == T and cond(msg):
            return msg
    return None


def command(cmd, *params):
    params = list(params) + [0] * (7 - len(params))
    m.mav.command_long_send(T, 1, cmd, 0, *params)
    ack = wait("COMMAND_ACK", lambda a: a.command == cmd)
    return ack.result if ack else None


def status_texts():
    while (st := m.recv_match(type="STATUSTEXT", blocking=False)) is not None:
        print(f"  STATUSTEXT from {st.get_srcSystem()}: {st.text}")


print(f"listening on {args.listen} for krabshell heartbeats…")
seen = {}
end = time.time() + 4
while time.time() < end:
    hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
    if hb and hb.get_srcComponent() == 1:
        seen[hb.get_srcSystem()] = "armed" if hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED else "disarmed"
if not seen:
    sys.exit("no heartbeats: is krabshell running with a drone scenario, and is --mavlink-gcs pointed here?")
print("drones:", ", ".join(f"sysid {k} ({v})" for k, v in sorted(seen.items())))
if T not in seen:
    sys.exit(f"sysid {T} is not among them")

pos = wait("GLOBAL_POSITION_INT")
lat, lon = pos.lat / 1e7, pos.lon / 1e7
print(f"sysid {T} is at {lat:.6f}, {lon:.6f}, {pos.relative_alt / 1000:.1f} m")

# A square north-east of where the drone is, then home.
dlat = args.box_m / 111_320
dlon = args.box_m / (111_320 * max(0.1, abs(__import__("math").cos(__import__("math").radians(lat)))))
corners = [(lat + dlat, lon), (lat + dlat, lon + dlon), (lat, lon + dlon)]
items = [(mav.MAV_CMD_NAV_TAKEOFF, 0, 0, args.alt)]
items += [(mav.MAV_CMD_NAV_WAYPOINT, int(a * 1e7), int(o * 1e7), args.alt) for a, o in corners]
items += [(mav.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0)]

print(f"uploading {len(items)} items…")
m.mav.mission_count_send(T, 1, len(items), mav.MAV_MISSION_TYPE_MISSION)
for _ in items:
    req = wait("MISSION_REQUEST_INT")
    if req is None:
        sys.exit("no MISSION_REQUEST_INT: upload stalled")
    cmd, x, y, z = items[req.seq]
    m.mav.mission_item_int_send(T, 1, req.seq, mav.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, cmd, 0, 1,
                                0, 2.0, 0, float("nan"), x, y, z, mav.MAV_MISSION_TYPE_MISSION)
ack = wait("MISSION_ACK")
status_texts()
print("  MISSION_ACK:", mav.enums["MAV_MISSION_RESULT"][ack.type].name if ack else "none")

m.mav.mission_request_list_send(T, 1, mav.MAV_MISSION_TYPE_MISSION)
count = wait("MISSION_COUNT")
print(f"read back: {count.count if count else '?'} items")

for name, cmd, params in [("arm", mav.MAV_CMD_COMPONENT_ARM_DISARM, (1,)), ("mission start", mav.MAV_CMD_MISSION_START, (0, 0))]:
    result = command(cmd, *params)
    status_texts()
    print(f"{name}: {mav.enums['MAV_RESULT'][result].name if result is not None else 'no ack'}")

print("flying (the sim runs faster than real time unless you set a rate on the dashboard)…")
end = time.time() + 60
while time.time() < end:
    msg = m.recv_match(type=["MISSION_ITEM_REACHED", "HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=1)
    if not msg or msg.get_srcSystem() != T:
        continue
    if msg.get_type() == "MISSION_ITEM_REACHED":
        print(f"  reached item {msg.seq}")
    elif msg.get_type() == "STATUSTEXT":
        print(f"  STATUSTEXT: {msg.text}")
    elif msg.get_type() == "HEARTBEAT" and msg.custom_mode == 0 and not msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED:
        # custom_mode 0 is "disarmed" in krabshell's mode numbering.
        print("landed and disarmed")
        break
