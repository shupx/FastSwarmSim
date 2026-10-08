#!/usr/bin/env bash
set -euo pipefail

# Resolve the release through the authenticated API instead of setup-ros's
# unauthenticated request, which can produce an empty version and a 404 URL.
source_package="${1:-ros2-apt-source}"
source /etc/os-release
source_codename="${UBUNTU_CODENAME:-${VERSION_CODENAME}}"
source_asset_url="$(gh api repos/ros-infrastructure/ros-apt-source/releases/latest \
  | /usr/bin/python3 -c '
import json
import sys
package, codename = sys.argv[1:]
assets = [asset for asset in json.load(sys.stdin)["assets"]
          if asset["name"].startswith(package + "_")
          and asset["name"].endswith("." + codename + "_all.deb")]
if len(assets) != 1:
    sys.exit(f"Expected one {package} release asset for {codename}, got {len(assets)}")
print(assets[0]["browser_download_url"])
' "$source_package" "$source_codename")"

source_deb="$(mktemp --suffix=.deb)"
trap 'rm -f "$source_deb"' EXIT
curl --fail --silent --show-error --location --retry 3 \
  --output "$source_deb" "$source_asset_url"
sudo apt-get update
sudo apt-get install --no-install-recommends --yes "$source_deb"
sudo apt-get update
