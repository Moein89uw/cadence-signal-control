import xml.etree.ElementTree as ET

import pytest

from cadence.paths import R1_DIR, load_spec


@pytest.fixture(scope="session")
def spec():
    return load_spec()


@pytest.fixture(scope="session")
def net_root():
    return ET.parse(R1_DIR / "r1.net.xml").getroot()


@pytest.fixture(scope="session")
def net_lanes(net_root):
    """{edge_id: [lane lengths by index]} for normal (non-internal) edges."""
    out = {}
    for e in net_root.findall("edge"):
        if e.get("function") == "internal":
            continue
        lanes = sorted(e.findall("lane"), key=lambda l: int(l.get("index")))
        out[e.get("id")] = [float(l.get("length")) for l in lanes]
    return out


@pytest.fixture(scope="session")
def net_connections(net_root):
    """Set of (from_edge, to_edge, from_lane, to_lane) between normal edges."""
    cons = set()
    for c in net_root.findall("connection"):
        if c.get("from").startswith(":"):
            continue
        cons.add((c.get("from"), c.get("to"), int(c.get("fromLane")), int(c.get("toLane"))))
    return cons
