#!/usr/bin/env bash
# Orin: field results for the report (E1 localisation, E4 rule vs network). NO MOTORS:
# it only reads the bags and logs of orin_auto.sh runs.
#
#   bash ~/racecar_ws/tools/orin_field_report.sh [run folder ...]   # default: today's runs
#
# Writes ~/r550_map/report_<time>.txt (and prints it). Tape readings for E1: put
# <run>/tape.txt (four numbers, cm: left rear wheel forward, out; left front wheel
# forward, out) before running this. See tools/field_report.py.
set -o pipefail
source /opt/ros/jazzy/setup.bash
source "$HOME/racecar_ws/install/setup.bash"
OUT="$HOME/r550_map/report_$(date +%Y%m%d_%H%M%S).txt"
python3 "$HOME/racecar_ws/tools/field_report.py" "$@" 2>&1 | tee "$OUT"
echo "saved $OUT"
