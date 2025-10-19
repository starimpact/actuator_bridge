#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate a ROS launch file containing one socketcan_bridge_node per CAN device/topic triple
from an actuator_bridge YAML config.

Default behavior: deduplicate by (can_device, rx_topic, tx_topic).
Optional --per-actuator: generate one node per actuator entry (may conflict if sharing device).
"""
import argparse
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    print("ERROR: PyYAML is required. Install with: pip install pyyaml", file=sys.stderr)
    sys.exit(2)


def load_config(path: Path):
    with path.open('r') as f:
        return yaml.safe_load(f)


def derive_topics(can_device: str, rx: str, tx: str):
    if not rx:
        rx = f"/{can_device}_rx"
    if not tx:
        tx = f"/{can_device}_tx"
    return rx, tx


def collect_triples(conf: dict, per_actuator: bool):
    buses = conf.get('buses', [])
    if not isinstance(buses, list):
        raise ValueError("'buses' must be a list in YAML")
    triples = []  # list of dicts: {can_device, rx, tx, name}
    seen = set()
    for group in buses:
        acts = group.get('actuators', []) if isinstance(group, dict) else []
        for a in acts:
            can_dev = a.get('can_device')
            rx = a.get('rx_topic')
            tx = a.get('tx_topic')
            if not can_dev:
                # allow group-level can_device fallback in future if needed
                continue
            rx, tx = derive_topics(can_dev, rx, tx)
            key = (can_dev, rx, tx)
            entry = {
                'can_device': can_dev,
                'rx': rx,
                'tx': tx,
                'name': a.get('name')
            }
            if per_actuator:
                triples.append(entry)
            else:
                if key in seen:
                    continue
                seen.add(key)
                triples.append(entry)
    return triples


def make_node_xml(idx: int, ent: dict) -> str:
    can_dev = ent['can_device']
    rx = ent['rx']
    tx = ent['tx']
    # Unique and readable node name
    base = f"socketcan_{can_dev}"
    name = f"{base}_{idx}"
    return (
        f"  <node pkg=\"socketcan_bridge\" type=\"socketcan_bridge_node\" name=\"{name}\" output=\"screen\">\n"
        f"    <param name=\"can_device\" value=\"{can_dev}\"/>\n"
        f"    <remap from=\"can_rx\" to=\"{rx}\"/>\n"
        f"    <remap from=\"can_tx\" to=\"{tx}\"/>\n"
        f"  </node>\n"
    )


def render_launch(triples):
    parts = ["<launch>\n"]
    for i, ent in enumerate(triples):
        parts.append(make_node_xml(i, ent))
        parts.append("\n")
    parts.append("</launch>\n")
    return ''.join(parts)


def main():
    ap = argparse.ArgumentParser(description="Generate socketcan_bridge launch from actuator YAML")
    ap.add_argument('-c', '--config', required=True, help='Path to YAML (actuator_bridge config)')
    ap.add_argument('-o', '--output', help='Path to write launch XML. If omitted, prints to stdout')
    ap.add_argument('--per-actuator', action='store_true', help='Generate one node per actuator (no dedup)')
    args = ap.parse_args()

    conf = load_config(Path(args.config))
    triples = collect_triples(conf, per_actuator=args.per_actuator)
    if not triples:
        print("WARNING: No socketcan triples discovered from YAML.", file=sys.stderr)
    xml = render_launch(triples)
    if args.output:
        Path(args.output).write_text(xml)
        print(f"Wrote launch to {args.output}")
    else:
        print(xml)


if __name__ == '__main__':
    main()
