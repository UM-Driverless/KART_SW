#!/usr/bin/env python3
"""Move done content out of kart-brain's tasks.md into tasks/done-archive.md.

Adapted from kart-medulla's tasks/archive_done.py (itself adapted from partle's).
Differences from kart-medulla:
  * headings here are `## Ready` / `## In Progress` / `## Blocked` / `## Done`
    (kart-medulla used `## TODO` / `## Done`).
  * the `## Done` section holds a few OPEN `- [ ]` items as well as closed
    ones, so it cannot be moved wholesale -- open items go back onto the
    board, under `## Ready`.
  * closed items appear in three shapes here: `- [x]`, `- [x YYYY-MM-DD]`,
    and `- [YYYY-MM-DD]` (no x). All three are treated as closed.
"""
import re, sys

TASKS = "/Users/rubenayla/repos/hardware/kart-brain/tasks.md"
lines = open(TASKS).read().split("\n")

CLOSED_RE = re.compile(r"^- \[(x(\s+\d{4}-\d{2}-\d{2})?|\d{4}-\d{2}-\d{2})\]")
OPEN_RE = re.compile(r"^- \[ \]")


def block_at(src, i):
    """The bullet at src[i] plus its indented / interleaved-blank continuation."""
    block = [src[i]]
    j = i + 1
    while j < len(src):
        nxt = src[j]
        if nxt == "":
            if j + 1 < len(src) and re.match(r"^\s+\S", src[j + 1]):
                block.append(nxt)
                j += 1
                continue
            break
        if re.match(r"^\s+\S", nxt):
            block.append(nxt)
            j += 1
            continue
        break
    return block, j


# --- split the file at `## Done` -------------------------------------------
done_start = next(i for i, l in enumerate(lines) if l.strip() == "## Done")
# `## Done` section runs to the next `## ` heading or EOF.
done_end = len(lines)
for i in range(done_start + 1, len(lines)):
    if re.match(r"^## ", lines[i]):
        done_end = i
        break
before, done_section, after = lines[:done_start], lines[done_start + 1:done_end], lines[done_end:]

# --- from the Done section, separate closed bullets from open ones ----------
archived_from_done, reopened = [], []
i = 0
while i < len(done_section):
    l = done_section[i]
    if CLOSED_RE.match(l):
        blk, i = block_at(done_section, i)
        archived_from_done += blk
    elif OPEN_RE.match(l):
        blk, i = block_at(done_section, i)
        reopened += blk
    else:
        archived_from_done.append(l)
        i += 1

# --- which sections (## or ###) still have open work? -----------------------
# A `- [x]` under a heading that still holds `- [ ]` items is a finished STEP
# of a live task, not a finished task. Archiving it would strip the open step
# of the context that says what was already settled. So only sections with
# nothing open left are archived whole; otherwise closed bullets are pulled
# out individually only when they sit directly under a `##` heading
# (standalone), never when nested under a `###` cluster that still has open
# steps.
section_open = {}
heading = ""
for l in before:
    if re.match(r"^#{2,3} ", l):
        heading = l
        section_open.setdefault(heading, False)
    if OPEN_RE.match(l):
        section_open[heading] = True

has_closed = {}
heading = ""
for l in before:
    if re.match(r"^#{2,3} ", l):
        heading = l
        has_closed.setdefault(heading, False)
    if CLOSED_RE.match(l):
        has_closed[heading] = True

kept, archived_from_board = [], []
heading = ""
i = 0
while i < len(before):
    l = before[i]

    # A fully-closed `###` section travels whole -- heading, prose and all --
    # so the archive keeps the reasoning and the board is not left with an
    # orphan heading over nothing.
    if re.match(r"^### ", l) and not section_open.get(l, False) and has_closed.get(l, False):
        j = i + 1
        while j < len(before) and not re.match(r"^#{2,3} ", before[j]):
            j += 1
        archived_from_board.append((None, before[i:j]))
        i = j
        continue

    if re.match(r"^#{2,3} ", l):
        heading = l

    # A bullet directly under a `##` heading (`## Ready`) is a standalone
    # task and travels on its own. Inside a `###` cluster, a done step stays
    # put until the whole cluster closes -- otherwise the remaining open step
    # loses the record of what was already settled.
    standalone = heading.startswith("## ") and not heading.startswith("### ")
    if CLOSED_RE.match(l) and standalone:
        blk, i = block_at(before, i)
        archived_from_board.append((heading, blk))
        continue
    kept.append(l)
    i += 1

print(f"closed blocks pulled off the board (Ready/In Progress/Blocked): {len(archived_from_board)}")
print(f"lines archived from ## Done                                   : {len(archived_from_done)}")
n_reopened = sum(1 for l in reopened if OPEN_RE.match(l))
print(f"open items rescued from ## Done -> ## Ready                   : {n_reopened}")

if "--apply" not in sys.argv:
    print("\n(dry run -- pass --apply to write)")
    for h, b in archived_from_board:
        print(f"  [board] {(h or b[0])[:90]}")
    for b in reopened:
        if OPEN_RE.match(b):
            print(f"  [Done->Ready] {b[:90]}")
    sys.exit(0)

# --- write the archive -----------------------------------------------------
out = [
    "<!-- reference — read only when you need the history of a shipped item -->",
    "# Done archive — completed work items",
    "",
    "Closed items moved out of the root `tasks.md` on 2026-08-10, following the same convention as",
    "the partle and kart-medulla repos. Nothing here is actionable: the root board carries only live",
    "work, while the reasoning behind finished things stays findable.",
    "",
    "The board is `tasks.md` at the repo root — the only task board in this repo.",
    "",
    "## Closed items from the board",
    "",
]
last_heading = None
for h, b in archived_from_board:
    if h != last_heading:
        if h:
            out += ["", h.replace("## ", "### ").replace("#### ", "### "), ""]
        last_heading = h
    out += b + [""]

out += ["", "## Previously under the board's `## Done` heading", ""] + archived_from_done + [""]

with open("/Users/rubenayla/repos/hardware/kart-brain/tasks/done-archive.md", "w") as f:
    f.write("\n".join(out).rstrip() + "\n")

# --- rewrite the board -------------------------------------------------------
# Insert rescued open items back under `## Ready` (right after that heading),
# then keep `## Done` as an empty heading (matching kart-medulla's convention
# of leaving the heading in place).
board = []
inserted = False
for l in kept:
    board.append(l)
    if not inserted and l.strip() == "## Ready":
        inserted = True
        if any(OPEN_RE.match(x) for x in reopened):
            board.append("")
            board += reopened

board += after  # tail after ## Done section (none expected here, but keep for safety)

with open(TASKS, "w") as f:
    f.write("\n".join(board).rstrip() + "\n")
print("\nwritten.")
