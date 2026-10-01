#!/usr/bin/env python3
"""Print the Cursor-on-Target events krabshell sends, as a TAK client would see them.

    python3 tools/cot_listen.py                   # ATAK's SA multicast group
    python3 tools/cot_listen.py --unicast 7777    # if krabshell has --cot-dest 127.0.0.1:7777

No dependencies. One line per event: type, callsign, position, and remarks.
Ctrl-C prints a count by type.
"""
import argparse
import collections
import socket
import struct
import xml.etree.ElementTree as ET

p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
p.add_argument("--group", default="239.2.3.1", help="multicast group")
p.add_argument("--port", type=int, default=6969)
p.add_argument("--unicast", type=int, metavar="PORT", help="listen on 127.0.0.1:PORT instead of multicast")
p.add_argument("--only", help="comma-separated CoT types to show, e.g. a-u-G-U-C-I,a-u-A-M-H-Q")
args = p.parse_args()

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
if args.unicast:
    s.bind(("127.0.0.1", args.unicast))
    print(f"listening on 127.0.0.1:{args.unicast}")
else:
    s.bind(("", args.port))
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                 struct.pack("4sl", socket.inet_aton(args.group), socket.INADDR_ANY))
    print(f"listening on {args.group}:{args.port}", flush=True)

only = set(args.only.split(",")) if args.only else None
counts = collections.Counter()
try:
    while True:
        data, sender = s.recvfrom(65536)
        try:
            ev = ET.fromstring(data)
        except ET.ParseError as err:
            print(f"!! unparseable event from {sender}: {err}")
            continue
        t = ev.get("type")
        counts[t] += 1
        if only and t not in only:
            continue
        pt = ev.find("point")
        contact = ev.find("detail/contact")
        remarks = ev.findtext("detail/remarks") or ""
        callsign = contact.get("callsign") if contact is not None else ev.get("uid")
        print(f"{t:<13} {callsign:<22} {float(pt.get('lat')):.5f},{float(pt.get('lon')):.5f} "
              f"{float(pt.get('hae')):6.1f} m  {remarks}", flush=True)
except KeyboardInterrupt:
    print()
    for t, n in counts.most_common():
        print(f"{n:6d}  {t}")
