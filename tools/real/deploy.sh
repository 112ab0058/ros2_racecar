#!/usr/bin/env bash
# Bash version of deploy.cmd (Git Bash / WSL / Claude Code). Run from anywhere:
#   bash tools/real/deploy.sh
# git pull, then send the 4 real-car packages + Orin scripts over one ssh and
# build them with tools/real/orin_build.sh. wro2026_sim is never sent.
set -eo pipefail
ORIN="${ORIN:-user2@192.168.0.15}"
cd "$(dirname "$0")/../.."

echo "[1/2] git pull"
git pull --ff-only

# The send step lives in its own file and is started only now, so a pull that
# changed it takes effect in this run (bash reads a running script lazily;
# 2026-10-19 the first deploy ran the old version and skipped a file).
exec bash tools/real/deploy_send.sh
