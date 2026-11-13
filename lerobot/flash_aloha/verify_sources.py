#!/usr/bin/env python3
"""Hash-check local reference files against sources.json; never fetch or import them."""

import argparse
import hashlib
import json
from pathlib import Path


def verify_sources(manifest, roots):
    results = []
    for name, repository in manifest["repositories"].items():
        root = roots.get(name)
        if root is None:
            continue
        root = Path(root)
        for entry in repository["files"]:
            path = root / entry["path"]
            try:
                digest = hashlib.sha256(path.read_bytes()).hexdigest()
                status = "matched" if digest == entry["sha256"] else "mismatch"
            except FileNotFoundError:
                status = "missing"
            except OSError:
                status = "unreadable"
            results.append(dict(repository=name, path=entry["path"], status=status))
    return results


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--flash-root", type=Path)
    parser.add_argument("--roarm-root", type=Path)
    parser.add_argument("--sdk-root", type=Path)
    args = parser.parse_args(argv)
    roots = {name: root for name, root in (
        ("flash_aloha", args.flash_root), ("roarm", args.roarm_root), ("waveshare_sdk", args.sdk_root)
    ) if root is not None}
    if not roots:
        parser.error("Supply at least one source checkout root")
    manifest = json.loads((Path(__file__).parent / "sources.json").read_text())
    results = verify_sources(manifest, roots)
    print(json.dumps({"checked_files": len(results), "results": results}, indent=2))
    return 0 if results and all(row["status"] == "matched" for row in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
