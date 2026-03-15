#!/bin/bash
# Autopilot — runs Claude Code headless to pick and execute tasks from .agents/tasks.md
# Loops until no Ready tasks remain.
# Usage: ./scripts/autopilot.sh

set -euo pipefail
cd "$(dirname "$0")/.."

PROMPT='Read .agents/tasks.md. Pick the first "Ready" task (- [ ]).
Move it to "In Progress" (- [→]). Do the work. When done, move to "Done" (- [x] ... (YYYY-MM-DD)).
If blocked, move to "Blocked" (- [⏸]) with a reason. Commit changes.
If no Ready tasks exist, respond with exactly "NO_TASKS".'

while true; do
    echo "[$(date '+%H:%M:%S')] Running..."
    OUTPUT=$(claude -p "$PROMPT" --allowedTools "Edit,Read,Write,Bash,Grep,Glob,Agent" 2>&1)
    echo "$OUTPUT"

    if echo "$OUTPUT" | grep -q "NO_TASKS"; then
        echo "No more tasks. Done."
        break
    fi
done
