"""Raw UDP probe: are the robot's RSI packets reaching this PC at all?

Binds the config's port on all interfaces and prints every datagram's
source and first bytes. It does NOT reply, so the controller will report
RSIBad after its Timeout (0.4 s) - that's fine: seeing even one packet here
proves the network/firewall path works, and the problem is then on the
RSIPI side. Seeing nothing means the packets never arrive (firewall,
IP_NUMBER mismatch, wrong adapter, or RSI_ON never ran).

Usage: run this, then press Start on the pendant (past the HALT).
"""
import socket
import sys
import time
import xml.etree.ElementTree as ET

CONFIG = (sys.argv[1] if len(sys.argv) > 1
          else "controller/SensorInterface/RSI_EthernetConfig_Basic.xml")


def config_port(path):
    return int(ET.parse(path).getroot().find("CONFIG/PORT").text.strip())


if __name__ == "__main__":
    port = config_port(CONFIG)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", port))
    sock.settimeout(1.0)
    print(f"Listening on UDP *:{port} for 60 s - now press Start on the pendant.")
    deadline = time.monotonic() + 60
    count = 0
    while time.monotonic() < deadline:
        try:
            data, addr = sock.recvfrom(65535)
        except socket.timeout:
            continue
        count += 1
        if count <= 5:
            print(f"packet {count} from {addr[0]}:{addr[1]} ({len(data)} bytes): "
                  f"{data[:120].decode(errors='replace')}...")
    print(f"\n{count} packets received" if count else
          "\nNO packets received - check firewall / IP_NUMBER / adapter / that RSI_ON ran")
    sys.exit(0 if count else 1)
