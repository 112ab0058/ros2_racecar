#!/usr/bin/env bash
# Orin: called by tools/real/deploy.cmd after the packages arrive in
# ~/racecar_ws/.deploy. Replaces the old copies in ~/racecar_ws/src (delete
# first, so files removed from the repo do not linger), clears their old
# build/install output (stale output caused StopIteration on 2026-10-16) and
# builds. Touches only the packages listed below.
set -eo pipefail

WS="$HOME/racecar_ws"
STAGE="$WS/.deploy"
PKGS=(wro2026_base_r550 wro2026_lidar_n10p wro2026_description wro2026_bringup wro2026_sim_r550)

for p in "${PKGS[@]}"; do
    if [ ! -d "$STAGE/src/$p" ]; then
        echo "missing $p in the upload, nothing changed" >&2
        exit 1
    fi
done

for p in "${PKGS[@]}"; do
    rm -rf "$WS/src/$p" "$WS/build/$p" "$WS/install/$p"
    mv "$STAGE/src/$p" "$WS/src/"
done
mkdir -p "$WS/tools"
cp "$STAGE"/tools/real/orin_*.sh "$STAGE"/tools/real/analyze_*.py "$WS/tools/"   # e.g. bash ~/racecar_ws/tools/orin_check.sh
rm -rf "$STAGE"

source /opt/ros/jazzy/setup.bash
cd "$WS"
colcon build --packages-select "${PKGS[@]}"

echo
echo "Built: ${PKGS[*]}"
echo "New terminals: source ~/racecar_ws/install/setup.bash"
