---
name: project-sync
description: Use after ROS/Gazebo coding or testing work in this repository to update handoff docs, evidence, next prompts, and ChatGPT-ready summaries.
---

# Project Sync Skill

Use this skill after any meaningful coding, testing, tuning, or debugging round in the WRO 160 cm ROS 2 / Gazebo project.

## Required Updates

1. Update `docs/AI_SYNC.md` when project state, git state, current task, known issues, or next action changes.
2. Update `docs/TEST_EVIDENCE.md` after every test run with meaningful evidence.
3. Update `docs/NEXT_PROMPT.md` with the next Codex task prompt.
4. Update `docs/CURRENT_STATE.md` if behavior, launch commands, world geometry, or controller strategy changed.

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

