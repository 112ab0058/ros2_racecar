# AGENTS.md

## Project Role

This repository is the shared source of truth for the WRO-inspired 160 x 160 cm ROS 2 / Gazebo self-driving car project.

Use this file as persistent guidance for Codex. Chat history is not the project source of truth; checked-in files are.

## Workspaces

- Git repository: `~/ros2_racecar`
- Runtime ROS 2 workspace: `~/ros2_ws`
- Main package: `src/wro2026_sim`
- Official world: `src/wro2026_sim/worlds/wro2026_field.sdf`

Formal changes should be made in `~/ros2_racecar` first, then synced into the runtime workspace:

```bash
rsync -av --delete ~/ros2_racecar/src/wro2026_sim/ ~/ros2_ws/src/wro2026_sim/
```

Build and run from `~/ros2_ws`:

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select wro2026_sim
source install/setup.bash
ros2 launch wro2026_sim race.launch.py
```

Use GUI only when needed:

```bash
ros2 launch wro2026_sim race.launch.py gui:=true
```

## Hard Rules

- Do not use `git reset --hard`.
- Do not `git stash pop` old stashes unless the user explicitly asks.
- Do not change orange/blue turn lines from diagonal marks into horizontal/vertical cross marks.
- Do not enable parking divider collision in race mode.
- Do not start SLAM or Nav2 from `race.launch.py`.
- Do not jump to AI, Nav2, or parking while line following and obstacle avoidance are still under test.
- Do not enable 2 or 3 obstacles until the 1-obstacle case is proven stable with evidence.
- Do not rely on Codex or ChatGPT memory for project state. Update checked-in docs.

## Current Test Discipline

Every implementation/testing round must report evidence, not only conclusions.

Required evidence:

- `git status --short --branch`
- `git diff --stat`
- `git log -1 --oneline` if a commit was made
- `which python3`
- `python3 --version`
- `colcon build --symlink-install --packages-select wro2026_sim` summary
- ROS topic/probe evidence for `/clock`, `/scan`, `/camera/image_raw`, `/cmd_vel`, `/line_detection`, `/odom`, `/tf`
- behavior logs for relevant controller states
- final status: `Done`, `Partial`, or `Blocked`

## AI Synchronization Files

Update these files at the end of each meaningful work round:

- `docs/AI_SYNC.md`: compact project state for ChatGPT/GPT handoff.
- `docs/TEST_EVIDENCE.md`: test history and evidence summary.
- `docs/NEXT_PROMPT.md`: next task prompt to give Codex.
- `docs/CURRENT_STATE.md`: broader project status when behavior changes.

When handing off to ChatGPT/GPT, provide a concise summary that can be pasted directly into the GPT project.

