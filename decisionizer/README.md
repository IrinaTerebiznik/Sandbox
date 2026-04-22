# Dev's Quest: Oracle

A gamified decision-support tool for engineers. It helps you make smarter, faster engineering decisions by scoring your situation and guiding you toward the right action — before you waste time going in circles.

## Why this exists

Engineers lose time to two failure modes: grinding alone too long when they should escalate, and context-switching into new tasks before their current block is truly resolved. The Oracle surfaces those moments with weighted scoring and gentle game mechanics (coins, lives, timers) to make the right call feel rewarding rather than like an admission of defeat.

## What it does

**Ask for Help?** — Weighs how lost you are, how long you've been stuck, whether you've checked code hygiene basics, and how urgency-critical the task is. Scores 1–5 on each axis, then puts you in one of three modes:

- `EXPLORER` — you're fine, keep going
- `PREPARATION` — document what you've tried; 5-minute countdown before auto-escalation
- `ESCALATION` — ask for help now; ignoring costs a life

**Open New Task?** — Evaluates whether context-switching is justified. Scores across four dimensions:

- *Clarity* — how well you understand the new task's requirements and can estimate the work
- *Blocking situation* — whether you've exhausted unblocking options on the current task (pinged others, waited >1hr) and whether placeholders would let you keep going without switching
- *Task duration* — shorter new tasks are safer to pick up mid-block
- *Sprint priority* — penalizes switching when your current task is blocking others or is sprint-critical

Outcomes: `GO AHEAD`, `THINK TWICE` (with specific gaps called out), or `NOT YET`.

**PR Checklist** — A comprehensive pre-push checklist organized into four categories:

- *Code Quality* — self-review, no debug statements, no commented-out code, no hardcoded secrets, clear naming
- *Testing* — existing tests pass, new tests written, happy path and edge cases manually tested
- *Branch & Commits* — up to date with base branch, no unintended files, descriptive commit messages
- *PR Description* — clear title, explains what and why, screenshots for UI changes, breaking changes documented, ticket linked, reviewers assigned

Tracks completion with a progress bar and celebrates at 100%.

## Game mechanics

| Resource | Earned by | Lost by |
|---|---|---|
| Efficiency Coins | Solving solo (+1), escalating when required (+2) | — |
| Lives | — | Staying stuck alone when Oracle says escalate |

Lives hitting zero triggers a Game Over and a nudge to talk to your Lead.

## Run

```bash
pip install streamlit
streamlit run app.py
```
