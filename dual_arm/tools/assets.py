#!/usr/bin/env python3
"""Fetch/check official reference assets; no robot or simulation dependencies."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import urllib.request
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def digest(data):
    return hashlib.sha256(data).hexdigest()


def checked_path(root, relative):
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError(f"Asset path escapes cache: {relative}")
    return path


def verify_data(entry, data):
    if len(data) != entry['bytes'] or digest(data) != entry['sha256']:
        raise ValueError(f"Size/hash mismatch: {entry['path']}; preserve file and review source")


def verify(manifest, cache):
    for entry in manifest['assets']:
        verify_data(entry, checked_path(cache, entry['path']).read_bytes())
    print(f"Verified {len(manifest['assets'])} files against the manifest")


def fetch(manifest, cache):
    for entry in manifest['assets']:
        path = checked_path(cache, entry['path'])
        if path.exists():
            verify_data(entry, path.read_bytes())
            continue
        url = entry['url']
        if not url.startswith(('https://raw.githubusercontent.com/waveshareteam/roarm_ws/',
                               'https://files.waveshare.com/wiki/RoArm-M3/')):
            raise ValueError(f"Unexpected source: {url}")
        with urllib.request.urlopen(url, timeout=60) as response:
            data = response.read(entry['bytes'] + 1)
        verify_data(entry, data)
        path.parent.mkdir(parents=True, exist_ok=True)
        # Exclusive creation prevents overwriting a file another process supplied.
        with path.open('xb') as output:
            output.write(data)
        print(f"Fetched {entry['path']}")
    verify(manifest, cache)


def inspect(cache):
    package = cache / 'roarm_description'
    model = ET.parse(package / 'urdf/roarm_m3/roarm_m3.xacro').getroot()
    dependencies = set()
    for element in model.iter():
        filename = element.get('filename')
        if filename:
            relative = filename.removeprefix('file://').removeprefix('$(find roarm_description)/')
            path = checked_path(package, relative)
            if not path.is_file():
                raise ValueError(f"Unresolved model dependency: {filename}")
            dependencies.add(relative)
    # Parse all includes too; this is not Xacro expansion or simulator validation.
    for relative in dependencies:
        if not relative.endswith('.stl'):
            ET.parse(package / relative)
    joints = model.findall('joint')
    zero_limits = [j.get('name') for j in joints if j.get('type') == 'revolute'
                   and any(float(j.find('limit').get(k, '0')) <= 0 for k in ('effort', 'velocity'))]
    print(json.dumps({
        'links': len(model.findall('link')),
        'revolute_joints': sum(j.get('type') == 'revolute' for j in joints),
        'fixed_joints': sum(j.get('type') == 'fixed' for j in joints),
        'resolved_dependencies': sorted(dependencies),
        'joints_needing_effort_velocity_limits': zero_limits,
        'readiness': 'reference_only; not validated for simulation or physical control',
    }, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('fetch', 'verify', 'inspect'))
    parser.add_argument('--cache', type=Path, default=ROOT / 'assets/cache')
    args = parser.parse_args()
    manifest = json.loads((ROOT / 'assets/manifest.json').read_text())
    try:
        if args.action == 'fetch':
            fetch(manifest, args.cache)
        else:
            verify(manifest, args.cache)
            if args.action == 'inspect':
                inspect(args.cache)
    except (OSError, ValueError, ET.ParseError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
