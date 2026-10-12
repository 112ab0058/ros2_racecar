---
name: project-sync
description: Use after ROS/Gazebo or real-car coding or testing work in this repository to update docs/STATUS.md and produce a ChatGPT-ready summary.
---

# Project Sync Skill

Use this skill after any meaningful coding, testing, tuning, or debugging round in the WRO 160 cm ROS 2 project (Gazebo simulation or the real car).

## Required Updates

1. Update `docs/STATUS.md` (the only "current state" document, one page, Traditional Chinese) when project state, milestone status, known issues, or the next step changes. Keep it to facts from checked-in docs or test output; write `UNKNOWN` when unsure.
2. Do not maintain `AI_SYNC.md`, `NEXT_PROMPT.md`, `CURRENT_STATE.md` or `TEST_EVIDENCE.md` any more: they are archived in `docs/archive/sim_phases/` and frozen (only the one-line archive header was added).
3. Put longer test evidence for a work session in a dated report (`docs/reports/<date>_<topic>.md`) and link it from `docs/STATUS.md` if it matters for the current state.
4. If a document is added, moved, or renamed, update `docs/README.md` (the index) and run `python3 tools/check_md_links.py`.

## Required Evidence In The Final Response

Always include:

- `git status --short --branch`
- `git diff --stat`
- `git log -1 --oneline` if a commit was made
- Python path and version
- build command and summary
- ROS topic/probe evidence
- behavior/log excerpts
- final status: `Done`, `Partial`, or `Blocked`

## Handoff Summary

End with a compact handoff summary that can be pasted into ChatGPT/GPT:

```text
Status: Done/Partial/Blocked
Commit: <hash or none>
Changed files:
- ...
Tests:
- ...
Key evidence:
- ...
Current issue:
- ...
Next prompt:
- ...
```

## Safety Rules

- Do not use `git reset --hard`.
- Do not `stash pop` old stashes without explicit user approval.
- Do not commit unrelated Partial tuning work when the task is only documentation sync.
- Do not rely on chat memory as project state; update checked-in docs instead.

