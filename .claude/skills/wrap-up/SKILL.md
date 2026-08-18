---
name: wrap-up
description: >
  End-of-session wrap-up for the Servo-Calibrator repo: commit pending
  changes, push, open/merge the PR, update CLAUDE.md/README.md (and any
  other affected .md docs), and update Claude's persistent memory — doing
  only whichever of those steps isn't already done. Use when the user says
  "wrap up", "wrap this up", "wrap-up", or asks (in any phrasing) to
  commit/push/merge and update docs/memory together at the end of a work
  session in this repo.
---

# Wrap-up (Servo-Calibrator)

Closes out a work session in this repo. Five checks, each done only if not
already done — read current state before acting, don't blindly redo a step
that's already satisfied.

## 0. Figure out what's actually pending first

Run `git status` and `git diff`/`git log` before touching anything:
- Uncommitted changes (tracked, modified/staged)?
- New untracked files that belong in git (vs. `ServoDAQ/data/`, which is
  deliberately untracked — see CLAUDE.md's "ServoDAQ" section — never add
  that)?
- Current branch: on `master`, or already on a feature branch?
- Is the branch pushed / up to date with `origin`?
- Is there already an open PR for this branch (`gh pr view` / `gh pr list
  --head <branch>`)?
- Do CLAUDE.md/README.md (and any other relevant `.md`, e.g.
  `ServoDAQ/README.md`, `ServoDAQ/MOTOR_TYPES.md`) already reflect this
  session's changes?
- Does anything from this session belong in persistent memory (see step 5
  — most sessions don't add anything new here; don't force it)?

Only act on the ones that are actually outstanding.

## 1. Docs first, then commit

This repo's convention (see CLAUDE.md's own history section, and
`ServoDAQ/README.md`) is to narrate *why*, not just *what* — a dated
subsection describing the change, the reasoning, and any real bugs/results
found, in the same voice as the existing entries. Update CLAUDE.md and/or
README.md (and any other doc the change actually touches) to match that
before committing, so the commit that lands includes the doc update, not a
separate "oops, forgot the docs" follow-up. Skip this only if the session's
change is truly doc-free (e.g. a pure data/investigation run with nothing
new to record) or the docs already cover it.

## 2. Branch, per this repo's actual convention

Check git log (`git log --oneline --graph -20`) if unsure: this repo
merges feature work through a PR (`Merge pull request #N from
vishwam-aggarwal/<branch>`), branch names are short kebab-case topic
descriptions (e.g. `servodaq-decoupled-accuracy-and-labeling`). If sitting
on `master` with real changes to commit, branch first — don't commit
straight to `master`. Name the branch for what the session actually did.

## 3. Commit and push

Stage the intended files explicitly (never blind `git add -A` — this repo
has deliberately-untracked directories like `ServoDAQ/data/`). Write a
commit message in this repo's own style: short imperative summary line,
body explaining the *why* when it's not obvious, ending with

```
Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
```

Push with `-u` if the branch has no upstream yet.

## 4. Open/merge the PR

If no PR exists for the branch yet, open one with `gh pr create` — title
matching the branch's topic, body summarizing what changed and why
(mirroring the doc update from step 1), ending with

```
🤖 Generated with [Claude Code](https://claude.com/claude-code)
```

If a PR already exists and is ready (no unresolved review comments, no
merge conflicts), merge it (`gh pr merge --merge`, matching the "Merge
pull request" style already in this repo's history — not squash or
rebase) and confirm `master` is back up to date locally afterward
(`git checkout master && git pull`).

**Ask before merging** if it's not obvious the PR is meant to land
immediately (e.g. the user hasn't reviewed it, or there's an open
question) — merging is not reversible in the casual sense a branch push
is.

## 5. Persistent memory

Check `MEMORY.md` and the memory directory for anything this session
established that's genuinely durable and not already derivable from the
repo (git history, CLAUDE.md, code) — see the memory-writing rules in
this project's system context for what counts. Most wrap-ups add nothing
here; only write a memory file when the session produced a real
correction, preference, or ongoing-project fact worth recalling in a
*different* future session. Update an existing memory file instead of
duplicating one, and add/refresh its `MEMORY.md` index line.

## Reporting back

State plainly which of the five steps actually did something and which
were already satisfied (e.g. "docs were already current, nothing to add
to memory — committed, pushed, and merged PR #6"). Don't claim a step
happened if it was a no-op.
