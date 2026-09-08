#!/bin/sh
set -eu
roarm_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
roarm_cache=${ROARM_BUILD_CACHE:-"$HOME/.cache/roarm-m3-build"}
roarm_python="$roarm_root/tools/flash-env-arm64/bin/python"
roarm_cxx="$roarm_cache/arduino-data/packages/esp32/tools/esp-x32/2302/bin/xtensa-esp32-elf-g++"

"$roarm_cxx" -std=gnu++17 -fsyntax-only "$roarm_root/tests/feedback_safety_test.cpp"
"$roarm_python" - "$roarm_root" "$roarm_cache" <<'PY'
from pathlib import Path
import shutil, sys
source, cache = map(Path, sys.argv[1:])
shutil.copytree(source / 'src/RoArm-M3_safe', cache / 'RoArm-M3_safe', dirs_exist_ok=True)
PY

# All implementation is ordinary C++ in main.cpp. The .ino is empty, so no
# generated prototypes are needed. Skip the bundled Intel-only ctags helper;
# every implementation file still receives normal compiler/type checking.
PATH="$roarm_root/tools/flash-env-arm64/bin:$PATH" "$roarm_root/tools/arduino-cli" compile \
  --fqbn esp32:esp32:esp32:FlashMode=dio,FlashFreq=80,FlashSize=4M,PartitionScheme=default \
  --config-file "$roarm_cache/arduino-cli.yaml" \
  --build-path "$roarm_cache/output" \
  --build-property "tools.esptool_py.path=$roarm_root/tools" \
  --build-property tools.ctags.cmd.path=/usr/bin/true \
  "$roarm_cache/RoArm-M3_safe"

"$roarm_python" - "$roarm_root" "$roarm_cache" <<'PY'
from pathlib import Path
import hashlib, json, shutil, sys
root, cache = map(Path, sys.argv[1:])
build = cache / 'output'
assert (build / 'RoArm-M3_safe.ino.partitions.bin').read_bytes() == (
    root / 'firmware/official-0.84/RoArm-M3_example.ino.partitions.bin').read_bytes()
binary = build / 'RoArm-M3_safe.ino.bin'
assert binary.stat().st_size <= 0x140000, 'Application exceeds existing partition'
dest = root / 'firmware/0.84-s1'
dest.mkdir(exist_ok=True, parents=True)
shutil.copy2(binary, dest / binary.name)
manifest = json.loads((dest / 'manifest.json').read_text())
manifest.update(application_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                application_bytes=binary.stat().st_size)
(dest / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print('Built application:', dest / binary.name)
PY
