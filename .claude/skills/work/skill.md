---
name: work
description: Work on tasks from .agents/tasks.md. Launch subagents that pick Ready tasks, update status, do the work, commit, and mark Done or Blocked.
argument-hint: [number of tasks | "list" | "add <description>"]
---

# Task Runner

Read `.agents/tasks.md` and `AGENTS.md` → Task Management for the full protocol.

## Actions

### `/work` or `/work <N>`
Launch subagents to work on tasks. Default: 1 task. If N > 1, launch N subagents in parallel on independent tasks.

For each task:
1. **Pick** the first available `- [ ]` (Ready) task. Each subagent picks a different one.
2. **Claim** it: edit `.agents/tasks.md`, move to `## In Progress` as `- [→] ...`.
3. **Do the work.** Read relevant `.agents/` docs first. Follow all project conventions.
4. **If blocked** (needs user input, hardware, unclear spec): move to `## Blocked` as `- [⏸] ... — reason: <specific question>`. Return the question to the user.
5. **If done**: commit changes, move to `## Done` as `- [x] ... (YYYY-MM-DD)`. Return a summary.

Use `isolation: "worktree"` for each subagent so parallel tasks don't conflict.

### `/work list`
Show the current state of `.agents/tasks.md` — how many ready, in progress, blocked, done.

### `/work add <description>`
Add a new task to the Ready section of `.agents/tasks.md`.
