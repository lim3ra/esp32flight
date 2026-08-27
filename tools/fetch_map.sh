#!/bin/sh
# Recover assets/map/world.png (800x400) and world_small.png (490x245).
#
# These two are not in the repository and no other tool regenerates them:
# .gitignore lists them as "generated assets", but nothing generates them -
# upstream keeps them local and ships them only inside release binaries.
# A clean clone therefore builds an assets image with no world map, and the
# route/screensaver map view comes up black with no error anywhere.
#
# The Android APK of a release is a plain ZIP, and apkflight/assets/map is a
# symlink to the same assets/map the device image uses, so the APK carries
# the exact files - no SPIFFS image parsing needed.
set -e
cd "$(dirname "$0")/.."

tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

url=$(curl -sL https://api.github.com/repos/theqkash/esp32flight/releases/latest |
    python3 -c "import json,sys; print(next(a['browser_download_url'] for a in json.load(sys.stdin)['assets'] if a['name'].endswith('.apk')))")
echo "fetching $url"
curl -sL -o "$tmp/release.apk" "$url"

mkdir -p assets/map
python3 - "$tmp/release.apk" assets/map <<'EOF'
import sys, zipfile

z = zipfile.ZipFile(sys.argv[1])
found = 0
for name in z.namelist():
    base = name.rsplit("/", 1)[-1]
    if name.startswith("assets/appdata/map/") and base.endswith(".png"):
        with open(sys.argv[2] + "/" + base, "wb") as out:
            out.write(z.read(name))
        print("  %s (%d B)" % (base, z.getinfo(name).file_size))
        found += 1
if found == 0:
    sys.exit("no map assets inside the APK - has the staging layout changed?")
EOF

echo "world maps -> assets/map/"
