#!/usr/bin/env bash
# Orin: train the imitation network from the recorded runs. NO MOTORS, no serial port:
# it only reads the bags and writes files.
#
#   bash ~/racecar_ws/tools/orin_train.sh [source all] [run folder ...]
#
# source: all (default) | hand (only Ray's driving with the drive page's keys / joystick)
#         | rule (only the corner rule's driving while 自動 was held)
# runs:   default = every ~/r550_map/*_auto folder with a bag/ (all orin_auto.sh runs)
# FRESH=1 extracts every run again (after a change to bc_extract); otherwise a run that
# already has dataset.npz is not read again.
#
# 1. bc_extract: each run -> <run>/dataset.npz (/scan_filled + executed command, 5 labels;
#    drops stand tests and the 1.5 s before each too-close stop)
# 2. bc_train: all datasets -> ~/r550_map/models/model_<time>.npz (kept) and a copy at
#    ~/r550_map/model.npz, which policy model (launcher 網路, orin_auto.sh model) drives
# Log: ~/r550_map/models/model_<time>.txt. The launcher page's 訓練 button runs this.
set -o pipefail

SOURCE="${1:-all}"
shift || true
case "$SOURCE" in all|hand|rule) ;; *) echo "source $SOURCE: must be all, hand or rule."; exit 1 ;; esac

source /opt/ros/jazzy/setup.bash
source "$HOME/racecar_ws/install/setup.bash"

MAP="$HOME/r550_map"
STAMP=$(date +%Y%m%d_%H%M%S)
mkdir -p "$MAP/models"
LOG="$MAP/models/model_$STAMP.txt"

if [ "$#" -gt 0 ]; then
    RUNS=("$@")
else
    RUNS=()
    for d in "$MAP"/*_auto; do [ -d "$d/bag" ] && RUNS+=("$d"); done
fi
if [ "${#RUNS[@]}" -eq 0 ]; then
    echo "no runs with a bag/ folder in $MAP"; exit 1
fi

TODO=()
for d in "${RUNS[@]}"; do
    if [ "${FRESH:-0}" = 1 ] || [ ! -f "$d/dataset.npz" ]; then TODO+=("$d"); fi
done
{
    echo "orin_train.sh $STAMP: source $SOURCE, ${#RUNS[@]} runs, ${#TODO[@]} to extract"
    if [ "${#TODO[@]}" -gt 0 ]; then
        echo "== 1. extract"
        ros2 run wro2026_imitation bc_extract "${TODO[@]}"
    fi
    DATA=()
    for d in "${RUNS[@]}"; do [ -f "$d/dataset.npz" ] && DATA+=("$d/dataset.npz"); done
    echo "== 2. train on ${#DATA[@]} datasets"
    if [ "${#DATA[@]}" -eq 0 ]; then echo "no usable data"; exit 1; fi
    ros2 run wro2026_imitation bc_train "${DATA[@]}" --source "$SOURCE" \
        -o "$MAP/models/model_$STAMP.npz" || exit 1
    cp "$MAP/models/model_$STAMP.npz" "$MAP/model.npz"
    echo "MODEL=$MAP/model.npz (copy of models/model_$STAMP.npz)"
} 2>&1 | tee "$LOG"
