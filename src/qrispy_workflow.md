# QRISPY: Stage-Gated Development Workflow
version: 1.1 | based on Dexter Horthy's talk (MLOps.community)

> **Companion document**: For the research rationale behind these rules — harness
> engineering principles, ICM methodology, empirical evidence — see
> `harness_rationale.md` in this directory.

---

## Purpose

This document defines the operational workflow and constraints for AI agents performing software development tasks. Read this document in full before beginning any task. Do not proceed until you have internalized these rules.

---

## Core Principle

> **You are an execution engine, not a decision-maker.**

The engineer reviewing your output is responsible for all design decisions, quality judgments, and final approval. Your role is to produce accurate, reviewable artifacts at each stage — not to autonomously produce a finished product. Do not skip stages. Do not merge stages. Do not make undocumented assumptions.

---

## What These Rules Prevent

Every rule in this document exists because the opposite was tried and failed. This table maps rules to the specific failure modes they guard against.

| Rule | Failure mode it prevents |
|------|--------------------------|
| Engineer reads all code (gate after each slice) | Reviewing only plans — not code — led to months of shipped "slop" that required full rewrites |
| Max instructions per stage | A single prompt with 85+ instructions caused the LLM to silently skip critical steps |
| Design doc max 200 lines | 1000+ line plan documents diverged from actual implementation; review time was wasted on text that never became code |
| Research stage hides the goal | Telling the AI the goal during research produced subjective, opinion-laden analysis instead of objective fact-gathering |
| Vertical slicing (not horizontal) | Layer-by-layer plans (all DB, then all API, then all UI) produced untestable intermediate states |
| Stage gates with human approval | Without checkpoints, the AI "outsources the thinking" — it accelerates, but only the engineer can verify |

---

## The QRISPY Workflow

Each stage produces exactly one artifact and stops. Do not proceed to the next stage until the engineer explicitly approves the current artifact.

```
Questions → Research → Design → Structure → Plan → Work → PR
    Q          R         I        S          P      Y      →
```

---

### Before Stage Q — The Ticket

The engineer writes the ticket before handing it to the agent. The ticket is the single input that drives the entire workflow. A weak ticket produces unfocused questions, unbounded designs, and scope creep.

**Required fields:**

| Field | Purpose | Example |
|-------|---------|---------|
| **What** | The observable problem or desired behavior | "Agents drift apart when the target moves fast" |
| **Why** | Motivation — why this matters now | "Causes pair_valid_rate to drop below 40% in training" |
| **Scope boundary** | What is explicitly out of scope | "Don't change the reward function; only fix the controller response" |

**Optional but recommended:**

| Field | Purpose | Example |
|-------|---------|---------|
| **Affected modules** | Helps the agent load the right context in Stage R | "Likely in controller/ and target_controller/" |
| **Acceptance criteria** | How the engineer will know it's done | "pair_valid_rate stays above 60% with target speed 2.0 m/s" |
| **Scope threshold** | Which flow to use (Full / Light / Direct) | "Full QRISPY" |

If the scope threshold is not specified, the agent defaults to Full QRISPY.

**Ticket template:**
```
## Ticket: [short title]

**What**: [observable problem or desired behavior]
**Why**: [motivation — why this matters now]
**Scope boundary**: [what is explicitly NOT in scope]
**Affected modules**: [if known]
**Acceptance criteria**: [how to verify success]
**Flow**: [Full / Light / Direct]
```

The engineer hands this ticket to the agent with an explicit directive: `"Start Q"`, `"Light flow, start I"`, or `"Direct fix"`.

---

### Stage Q — Questions

**Goal:** Surface all ambiguities before any work begins.

**Instructions:**
- Read the task ticket in full.
- List every assumption you would need to make to proceed.
- List every decision point where you would need to choose between two or more reasonable approaches.
- Do not attempt to answer these questions yourself.
- Do not begin research.

**Output format:**
```
## Open Questions

### Assumptions requiring confirmation
- [question]

### Architectural decisions requiring human input
- [decision point with options]
```

**Constraint:** Maximum 20 questions. If you have more, prioritize by impact on implementation scope.

---

### Stage R — Research

**Goal:** Produce an objective map of the existing codebase.

**⚠ Critical rule: You must not know the implementation goal during this stage.**

The engineer will provide the codebase scope and ask you to document what exists. You will not be told what will be built. This is intentional. Knowing the goal causes you to filter and interpret facts to support a conclusion, which produces biased research.

**Instructions:**
- Document what exists: modules, interfaces, data models, conventions, patterns.
- Record facts only. No opinions, no suggestions, no "this could be improved."
- Flag inconsistencies or gaps you observe, without prescribing fixes.
- Do not infer what is missing based on what you expect to build.

**Output format:**
```
## Codebase Research Report

### Module inventory
[list with brief descriptions]

### Data models
[schemas and relationships]

### Conventions observed
[naming, error handling, testing patterns, etc.]

### Gaps and inconsistencies
[factual observations only]
```

**Constraint:** Facts only. Any sentence containing "should," "could," "might want to," or "would benefit from" is out of scope for this stage.

---

### Stage I — Design Document

**Goal:** Produce a concise design document for human review before any code is written.

**Instructions:**
- Now you may read the ticket.
- Using the research output and the ticket, draft a design document.
- The design document must describe: what will change, why, and how it fits the existing architecture.
- Do not write pseudocode. Do not write implementation details.
- Focus on interfaces, data flow, and component boundaries.

**Output format:**
```
## Design Document: [Feature Name]

### Problem statement
[1–3 sentences]

### Proposed approach
[narrative, not bullets]

### Key interfaces and data flow
[diagrams or structured description]

### What this does NOT include
[explicit scope boundaries]

### Open risks
[unresolved concerns for engineer review]
```

**Constraint:** Maximum 200 lines. If you need more, your scope is too large — flag this to the engineer.

---

### Stage S — Structure Outline

**Goal:** Define the file and function skeleton before writing logic.

**Instructions:**
- List every file that will be created or modified.
- For each file, list function signatures, class names, and exported interfaces — no implementations.
- Indicate which files are new vs. modified.
- This is the last checkpoint before code is written. The engineer must confirm this structure before you proceed.
- Use the reusable patterns in Appendix B as starting points where applicable.

**Output format:**
```
## Structure Outline

### New files
- path/to/file.ext
  - functionName(params): ReturnType
  - ClassName { methodName() }

### Modified files
- path/to/existing.ext
  - [existing] functionName — no change
  - [add] newFunction(params): ReturnType
  - [modify] existingFunction — [what changes]
```

**Constraint:** No implementation. No logic. Signatures and shapes only.

---

### Stage P — Plan

**Goal:** Produce a vertical, ordered implementation plan.

**Instructions:**
- Break the work into vertical slices. Each slice must be end-to-end testable on its own.
- A vertical slice = one complete user-facing behavior, from data layer through API through UI.
- Do not plan horizontal layers (e.g., "implement all DB changes, then all API changes").
- Each step must reference specific files and functions from the Structure outline.
- Steps must be ordered so that each step produces working, testable code.

**Output format:**
```
## Implementation Plan

### Slice 1: [behavior name]
- Step 1.1: [specific action] in [file].[function]
- Step 1.2: [specific action] in [file].[function]
- Test checkpoint: [what can be verified after this slice]

### Slice 2: [behavior name]
...
```

**Constraint:** Maximum 10 steps per slice. Maximum 5 slices per plan. If you need more, the scope must be split into separate tasks.

---

### Stage Y — Work (Implement)

**Goal:** Implement exactly what the approved plan specifies.

**Instructions:**
- Implement one slice at a time.
- After each slice, stop and report what was implemented and what the test checkpoint is.
- Do not implement the next slice until directed.
- Do not add features, refactors, or improvements not in the plan. If you notice something that should be changed, log it in a "Observations" section and continue.
- Follow every convention documented in the Research stage.

**Per-slice output:**
```
## Slice [N] Complete

### Files changed
- [file]: [what changed]

### Test checkpoint
[how to verify this slice works]

### Observations (do not act on these)
- [anything noticed that is out of scope]
```

**Constraint:** You must not proceed past a slice boundary without engineer confirmation.

---

### PR — Pull Request Description

**Goal:** Produce a reviewable PR description that lets the engineer read the diff intelligently.

**Instructions:**
- Summarize what was implemented, slice by slice.
- List every file changed and why.
- Identify any deviations from the approved plan and explain them.
- Include the test checkpoints from each slice as the verification guide.

**Constraint:** Do not summarize what the feature does for users. That is in the ticket. Focus on what changed in the code and why.

---

## Instruction Budget

Each stage above contains at most 10 instructions. This is intentional.

**You must not follow more than 30 active constraints simultaneously.**

The original research found that 85+ simultaneous instructions caused silent omissions. The safe operating range is under 40. Each stage in this document contributes ~10 instructions; the remaining budget accommodates mid-task instructions from the engineer. If the combined total exceeds 30, flag the conflict. Do not silently attempt to satisfy all constraints — this produces inconsistent output. Report: "Instruction budget exceeded. Please confirm priority."

---

## What You Must Never Do

| Behavior | Reason |
|---|---|
| Skip a stage because it seems obvious | Every stage exists to create a human checkpoint |
| Merge stages to save time | Reduces reviewability, not development time |
| Write code before the Structure is approved | Code written against an unapproved structure is throwaway work |
| Interpret ambiguity silently | Undocumented assumptions compound into large divergences |
| Add improvements not in scope | Scope creep is the primary source of rework |
| Plan horizontally (layer by layer) | Horizontal plans produce untestable intermediate states |
| Make research subjective | Biased research produces plans that rationalize rather than solve |

---

## Scope Threshold

Not every task requires all 7 stages. The engineer decides which flow applies when handing off the ticket. If unspecified, the agent defaults to full QRISPY.

**Full QRISPY** (all stages): New feature, architectural change, multi-file modification, anything touching >3 files or adding new interfaces.

**Light flow** (I → S → Y → PR): Bug fix with known root cause, config change, single-file modification where the design is obvious. Skip Q (no ambiguity), R (codebase already understood), P (single slice). Still requires Structure approval before coding.

**Direct fix** (Y only): Typo, comment update, import fix, <10 lines changed in one file. Still report what was done. Still get engineer review before merge.

---

## What the Engineer Is Responsible For

You are not responsible for these. If you find yourself making these decisions, stop and ask.

- Whether the approach is architecturally correct
- Whether the design tradeoffs are acceptable
- Whether the scope is right for this task
- Reading and approving all code before it is merged
- Final quality judgment

The engineer reads the code. You do not ship unreviewed output.

---

## Engineer–Agent Interaction Map

The engineer intervenes exactly **6 + N times** (N = number of slices).

```
Engineer                          Agent
   │                                │
   ├─ Hand off ticket ─────────────► │ [Stage Q] Generate question list
   │ ◄──────────── Return questions ┤
   ├─ Answer questions ─────────────► │ [Stage R] Codebase research
   │   (⚠ Do not reveal the goal)    │
   │ ◄───────────── Return research ┤
   ├─ Confirm research + reveal ────► │ [Stage I] Write design document
   │    ticket                        │
   │ ◄──────────────── Return design ┤
   ├─ Approve/revise design ────────► │ [Stage S] Write structure outline
   │ ◄─────────── Return structure  ┤
   ├─ Approve/revise structure ─────► │ [Stage P] Write implementation plan
   │ ◄────────────── Return plan    ┤
   ├─ Approve plan ─────────────────► │ [Stage Y] Implement Slice 1
   │ ◄──────────── Slice 1 done    ┤
   ├─ Code review + approve ────────► │ [Stage Y] Implement Slice 2
   │ ◄──────────── Slice 2 done    ┤
   ├─ Code review + approve ────────► │ [Stage PR] Write PR description
   │ ◄──────────────── Return PR    ┤
   └─ Final merge                     │
```

---

## Gate Details: What the Engineer Actually Does

Each gate is a hard stop. The agent waits. The engineer acts. Only then does work continue.

| Gate | Engineer reads | Judgment call | How to unblock |
|---|---|---|---|
| After Q | Question list | Any missing assumptions? Any wrong premises? | Answer each question, then → `"Start R"` |
| After R | Research report | Does it accurately understand the codebase? | Confirm report, reveal ticket → `"Start I"` |
| After I | Design document | Is the direction correct? Is the scope appropriate? | Send revisions or → `"Start S"` |
| After S | Structure outline | Does the file/function structure match existing conventions? | Revise or → `"Start P"` |
| After P | Implementation plan | Are slices vertical? Is the order correct? | Revise or → `"Start Y"` |
| After each Slice | **Actual code** | Read the code directly and judge quality | After code review → `"Next slice"` or `"Start PR"` |

### The most important gate: After each Slice

The gates from Q through P involve reviewing AI-generated text. The gate after each Slice is **the only point where the engineer reads actual code**.

If this gate is skipped:
- Unreviewed code accumulates.
- Review cost increases exponentially as slices pile up.
- The entire thing ends up being rewritten at the final PR.

**When the agent reports "Slice N complete," the engineer does not start the next slice until they have read the diff.**

---

## Summary Reference

```
Stage      Artifact              Max size     Human gate?   Scope
-------    -------------------   ----------   -----------   -----
Q          Question list         20 items     ✓ before R    Full only
R          Research report       unbounded    ✓ before I    Full only
I          Design document       200 lines    ✓ before S    Full, Light
S          Structure outline     —            ✓ before P    Full, Light
P          Implementation plan   10×5 steps   ✓ before Y    Full only
Y (×N)     Slice implementation  1 slice      ✓ per slice   All
PR         PR description        —            → merge       Full, Light
```

---

## Context Loading Per Stage

This project uses a layered documentation system. Load only what each stage needs — do not load all spec files or all CONTEXT.md files at once.

| Stage | What to load |
|-------|-------------|
| Q | Ticket + `CLAUDE.md` + `ARCHITECTURE.md` |
| R | Relevant module `CONTEXT.md` files + their key source files |
| I | Ticket + R output + relevant `*_spec.md` files |
| S | I output + existing code in affected files |
| P | S output only |
| Y | P + S outputs + module `CONTEXT.md` for each file being changed |
| PR | All slice outputs + ticket |

If unsure which specs or modules are relevant, ask the engineer before loading broadly.

For a complete catalog of all meta-files in this project (what each file is, where it goes, its token budget), see Appendix A below.

---

## Artifact Handoff Between Stages

### Within a single session

Artifacts stay in conversation. Each stage's output is visible to subsequent stages through the chat history. No files are written — the conversation *is* the handoff mechanism.

### Across session boundaries (`/save-qrispy-state`)

If a context window fills mid-workflow or a session must end before all stages are complete, the agent must persist artifacts to files so the next session can resume without loss.

**Invoke with `/save-qrispy-state`.** This is a Claude Code slash command (skill location: `.claude/commands/save-qrispy-state.md`).

**What it does:**

1. Creates `doc/active/tickets/<ticket-slug>/` if it doesn't exist
2. Writes each completed stage's artifact to a file:
   ```
   doc/active/tickets/<ticket-slug>/
   ├── ticket.md              # Original ticket (copied once)
   ├── q_questions.md         # Stage Q output
   ├── r_research.md          # Stage R output
   ├── i_design.md            # Stage I output
   ├── s_structure.md         # Stage S output
   ├── p_plan.md              # Stage P output
   ├── y_slice_1.md           # Slice 1 report
   ├── y_slice_2.md           # Slice 2 report (etc.)
   ��── state.json             # Current stage, approval status, next action
   ```
3. Updates `doc/active/progress.txt` with the current QRISPY stage
4. Updates `doc/active/feature_list.json` if any feature status changed

**`state.json` format:**
```json
{
  "ticket_slug": "fix-agent-drift",
  "current_stage": "S",
  "last_approved_stage": "I",
  "next_action": "Engineer reviews structure outline, then → Start P",
  "slices_completed": [],
  "slices_total": null,
  "created": "2026-03-30",
  "updated": "2026-03-30"
}
```

**Resuming in the next session:**

1. The agent reads `doc/active/tickets/<ticket-slug>/state.json`
2. Loads only the artifacts needed for the current stage (per the Context Loading table)
3. Resumes at the **next unapproved stage** — not at Q
4. If resuming mid-Y, reads `state.json` to determine which slices are complete

**When NOT to use:** If the ticket completes within a single session, no files are written. The ticket directory is only created when crossing a session boundary.

**Completion:** After the PR is merged, rename the ticket directory to append `-DONE` (e.g., `012-gimbal-teleop-keyboard/` → `012-gimbal-teleop-keyboard-DONE/`) on the user's approval.

**Why a skill (not automatic):** Like `/update-progress`, this requires judgment — the agent needs to determine the ticket slug, identify which artifacts are approved vs. in-progress, and write accurate state. A user-invoked skill preserves control. The engineer can also invoke it proactively before ending a session.

**Design note:** The end-of-session protocol in `CLAUDE.md` (`/update-progress`) handles *project-level* progress. `/save-qrispy-state` handles *ticket-level* QRISPY state. They are complementary — run both at session end if a ticket is in flight.

---
---

# Appendix A: Meta-File Catalog

There is no single standard — different communities (Anthropic, ICM, AGENTS.md spec, HumanLayer) define overlapping but distinct meta-files. Below is a unified catalog of all meta-files referenced across these sources, with our adopted subset marked.

## A.1 Universal Meta-Files (Cross-Community)

| File | Origin | Purpose | Where It Goes | Token Budget | Our Status |
|------|--------|---------|---------------|-------------|------------|
| **CLAUDE.md** | Claude Code | System prompt: project conventions, build commands, coding rules. Always loaded by Claude. | Repo root + nested per-directory | ~800 tokens ideal | **Exists** — comprehensive but monolithic |
| **AGENTS.md** | [agents.md spec](https://agents.md/) (Linux Foundation) | Agent-agnostic version of CLAUDE.md. Works across 25+ tools (Claude, Codex, Cursor, Copilot, etc.). Same purpose, broader compatibility. | Repo root + nested subdirs | Flexible | Not used — CLAUDE.md covers our needs |

## A.2 ICM Meta-Files (Jake Van Clief)

| File | Layer | Purpose | Where It Goes | Token Budget | Our Status |
|------|-------|---------|---------------|-------------|------------|
| **CONTEXT.md** (root) | L1 | Routing table: maps tasks to workspace folders. No reference material — only navigation. | Workspace root | ~300 tokens | **Adopt** |
| **CONTEXT.md** (per-folder) | L2 | Stage contract: specifies Inputs, Process, Outputs for a module. Must stay <80 lines. | Each sub-module folder | 200-500 tokens | **Adopt** |
| **questionnaire.md** | Setup | One-time config: brand, voice, preferences. Flat format, not per-run. | `setup/` | Varies | Not applicable |

## A.3 Harness Engineering Meta-Files (Anthropic / Community)

| File | Purpose | Where It Goes | Format | Our Status |
|------|---------|---------------|--------|------------|
| **feature_list.json** | Tracks implementation status across sessions. JSON chosen because agents are less likely to corrupt structured data vs. markdown. | Active work directory | JSON with id, name, status, notes fields | **Adopt** — in `doc/active/` |
| **progress.txt** | Running session handoff log. Each session appends what was done, what's next. | Active work directory | Plain text, append-only | **Adopt** — in `doc/active/` |
| **init.sh** | Bootstraps dev environment for new agent sessions. Runs setup + basic verification. | Project root | Shell script | Not needed — `./isaaclab.sh` covers this |
| **verify.sh** | Back-pressure: runs tests/checks at session end. | Project root | Shell script | **Investigate** |

## A.4 Structural Documentation Files

| File | Purpose | Where It Goes | Our Status |
|------|---------|---------------|------------|
| **ARCHITECTURE.md** | Module dependency map, data flow, integration boundaries. Scoped per project iteration. | `iris_ma6/` root | **Adopt** |
| **agent_docs/*.md** | Progressive disclosure: topic-specific guides loaded on demand. CLAUDE.md points to them. | `doc/` or `agent_docs/` | Partially exists — our `doc/*_spec.md` files serve this role |

## A.5 What We Adopt vs. Skip

**Adopted:**
1. `CONTEXT.md` (per-module) — 11+ files across all sub-modules
2. `ARCHITECTURE.md` — at `iris_ma6/ARCHITECTURE.md`
3. `feature_list.json` + `progress.txt` — in `doc/active/`
4. `iris_ma6/CLAUDE.md` — session workflow protocol with meta-file maintenance rules

**Skip (with rationale):**
- `AGENTS.md` — redundant with CLAUDE.md for a single-agent-tool project
- `questionnaire.md` — designed for content workflows, not code
- `init.sh` — `./isaaclab.sh` already handles environment setup
- Root-level `CONTEXT.md` (L1 router) — our project has a single workspace (iris_ma6), so the routing layer adds overhead without benefit. If we later have parallel workspaces, revisit.

---

# Appendix B: Reusable Development Patterns

These patterns were extracted from building all 11 iris_ma6 modules. They are reference templates that the engineer selects during QRISPY Stage S (Structure). The agent uses the selected pattern as a starting point for its structure outline — it does not autonomously apply patterns without engineer direction.

## B.1 Module Scaffold

**When**: Creating a new sub-module from scratch.
**Reuse**: 100% — every module follows this structure.

```
module_name/
├── __init__.py           # Exports: configs first, then classes, then presets
├── CONTEXT.md            # ICM Layer 2 contract (Purpose, Inputs, Outputs, Dependencies, Key Files, Spec)
├── module_name_cfg.py    # @configclass with field docs, sections, defaults
├── module_name.py        # Core implementation
└── tests/
    ├── __init__.py
    ├── run_tests.py      # AppLauncher-based standalone runner
    ├── README.md
    ├── test_result.txt   # Overwritten each run
    └── error_log.txt     # Appended each iteration
```

**`__init__.py` template**:
```python
"""module_name — one-line description.

Multi-line description of what this module provides.
"""

# Configurations
from .module_name_cfg import ModuleNameCfg

# Core classes
from .module_name import ModuleName

__all__ = [
    # Configurations
    "ModuleNameCfg",
    # Core classes
    "ModuleName",
]
```

**After creating**: Update `ARCHITECTURE.md` with the new module and its dependencies.

## B.2 Configuration Dataclass

**When**: Defining configurable parameters for a module or component.
**Reuse**: 95% — minor variations for simple vs. nested vs. per-agent configs.

```python
from __future__ import annotations
from dataclasses import dataclass, field
from isaaclab.utils import configclass

@configclass
class SubComponentCfg:
    """One-line description.

    Multi-line explanation of purpose and typical usage.
    """

    # ===== Section Name =====

    param_name: float = 1.0
    """Description. Units: [m/s]. Typical range: [0.5, 2.0]."""

    param_tuple: tuple[float, float, float] = (1.0, 1.0, 1.0)
    """Description [x, y, z] [units]."""

    enabled: bool = True
    """Whether this subsystem is active."""

@configclass
class MasterCfg:
    """Composite config aggregating sub-configs."""

    sub_component: SubComponentCfg = SubComponentCfg()
    """Sub-component configuration."""

    control_dt: float = 0.01
    """Control loop timestep [s]."""
```

**Conventions**:
- `@configclass` for IsaacLab integration, `@dataclass` for standalone
- Every field has an inline docstring with purpose, units, and range
- Group related fields under `# ===== SECTION =====` headers
- Use `field(default_factory=...)` for mutable defaults (lists, dicts)
- Nested composition: `sub: SubCfg = SubCfg()` for hierarchical configs

## B.3 Test Suite

**When**: Adding tests for a new or existing module.
**Reuse**: 90% — core structure is identical; test content varies.

**`run_tests.py` skeleton**:
```python
#!/usr/bin/env python3
"""Module Test Suite.

Usage:
    ./isaaclab.sh -p path/to/run_tests.py
    ./isaaclab.sh -p path/to/run_tests.py --test-verbose
"""
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Run [Module] test suite")
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(headless=True)
parser.add_argument("--test-verbose", action="store_true", help="Verbose output")
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# --- NOW import everything else ---
import sys
import torch
import traceback
from datetime import datetime

VERBOSE = args_cli.test_verbose

class TestResults:
    def __init__(self):
        self.passed, self.failed, self.errors = [], [], []

    def add_pass(self, name: str):
        self.passed.append(name)
        print(f"  ✓ {name}")

    def add_fail(self, name: str, error: str):
        self.failed.append((name, error))
        print(f"  ✗ {name}")
        for line in error.split("\n")[:5]:
            print(f"    {line}")

    def add_error(self, name: str, error: str):
        self.errors.append((name, error))
        print(f"  ERROR {name}\n    {error[:200]}")

    def print_summary(self) -> bool:
        total = len(self.passed) + len(self.failed) + len(self.errors)
        print(f"\n{'='*80}\nTEST SUMMARY\n{'='*80}")
        print(f"Total: {total}  Passed: {len(self.passed)}  Failed: {len(self.failed)}  Errors: {len(self.errors)}")
        if self.failed:
            print(f"\n{'-'*80}\nFAILED:")
            for name, err in self.failed:
                print(f"  {name}: {err[:200]}")
        print("=" * 80)
        return len(self.failed) == 0 and len(self.errors) == 0

def run_tests(results: TestResults, device: torch.device):
    print(f"\n{'='*80}\nTesting ModuleName\n{'='*80}")
    num_envs = 16

    # Test: Initialization
    try:
        cfg = ModuleCfg()
        module = Module(cfg, num_envs=num_envs, device=device)
        results.add_pass("Initialization")
    except Exception as e:
        results.add_fail("Initialization", traceback.format_exc())
        return  # Can't continue without init

    # Test: Core functionality (simple values first, then realistic)
    try:
        result = module.compute(simple_input)
        assert result.shape == expected_shape
        results.add_pass("Core computation — simple values")
    except Exception as e:
        results.add_fail("Core computation — simple values", traceback.format_exc())

def main():
    print(f"{'='*80}\n[MODULE] TEST SUITE\n{'='*80}")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Device: {device}  Time: {datetime.now():%Y-%m-%d %H:%M:%S}")
    results = TestResults()
    try:
        run_tests(results, device)
    except Exception as e:
        results.add_error("Suite-level", traceback.format_exc())
    sys.exit(0 if results.print_summary() else 1)

if __name__ == "__main__":
    main()
```

**Critical**: AppLauncher + arg parsing MUST come before all other imports. This is non-negotiable for Isaac Sim compatibility.

**Test methodology**: 2-step scheme — test with simple/known values first, then with realistic values.

## B.4 Specification Document

**When**: Defining what a module should do before or during implementation.
**Reuse**: 85% — section structure is standard; content depth varies by module.

```markdown
# [Module] Specification

**Module:** `module_name`
**Status:** [Draft v1 | Stable | Deprecated]
**Last Updated:** YYYY-MM-DD

---

## 1. Motivation and Design Philosophy
### 1.1 The Problem
### 1.2 Design Philosophy
### 1.3 Architecture Overview (ASCII diagram)

## 2. Mathematical Formulation
### 2.1 System Model (equations, notation)
### 2.2 [Algorithm/Component details]

## 3. Implementation Details
### 3.1 Configuration (Cfg dataclass fields, defaults, tuning)
### 3.2 [Major Class] (purpose, key methods, state)

## 4. Integration Points
### 4.1 Input Interface (tensor shapes, value ranges)
### 4.2 Output Interface (tensor shapes, value ranges)
### 4.3 Dependencies (upstream and downstream modules)
### 4.4 Calling Contract
- **Call frequency**: How many times per step each public method should be called
- **Mutating vs read-only**: Which methods advance internal state (buffers, counters, RNG)
- **Idempotency**: Which methods are safe to call multiple times at the same sim time
- **Lifecycle placement**: Which env lifecycle method (`_pre_physics_step`, `_get_observations`, etc.) should call each method
- **Stateful invariants**: Assumptions about calling order or exclusivity

## 5. Validation and Testing
### 5.1 Unit Tests (categories, criteria)
### 5.2 Integration Tests (with which modules)

## 6. Known Limitations

## 7. References
```

**Key principle**: Specs are authoritative — implementation conforms to spec, not the other way around. If the implementation diverges, update the spec first.

## B.5 Environment Integration

**When**: Adding a completed module to `iris_ma_env6_test.py`.
**Reuse**: 80% — the pattern is consistent but details are highly module-specific.

**Step-by-step checklist**:

1. **Config** (`iris_ma_env6_test_cfg.py`):
   ```python
   from isaaclab_tasks.direct.iris_ma6.module_name import ModuleNameCfg

   @configclass
   class IrisMA6TestEnvCfg(DirectMARLEnvCfg):
       module_name: ModuleNameCfg = ModuleNameCfg()
       """Module description."""
   ```

2. **Init** (`iris_ma_env6_test.py` — `__init__`):
   - Decide: per-agent instance or centralized?
   - Per-agent: loop over `cfg.possible_agents`, create one instance each
   - Centralized: single instance with `num_agents` parameter
   ```python
   # After super().__init__()
   self._module = ModuleName(
       cfg=cfg.module_name,
       num_envs=self.num_envs,
       device=self.device,
   )
   ```

3. **Per-step calls** — place in the appropriate lifecycle method:
   - `_pre_physics_step()`: action processing, safety filters
   - `_physics_step()`: force application
   - `_post_physics_step()`: state updates, sensor processing
   - `_get_observations()`: observation augmentation
   - `_get_rewards()`: reward terms
   - `_reset_idx()`: state resets

4. **After integrating**: Update `ARCHITECTURE.md` dependency graph and the module's `CONTEXT.md` if integration revealed new interfaces.

## B.6 Pattern Selection Guide

| Task | Pattern(s) to Use | QRISPY Stage | Scope |
|------|-------------------|-------------|-------|
| "Add a new module" | Scaffold → Config → Implementation → Test Suite → Spec → Integration | S (Structure), Y (Work) | Full |
| "Add tests to existing module" | Test Suite only | S + Y | Light |
| "Define a new feature before coding" | Spec → then Scaffold when ready | I (Design) | Full |
| "Wire an existing module into env" | Integration only | S + Y | Light |
| "Add parameters to existing module" | Config (update existing `*_cfg.py`) | Y | Direct fix |

## B.7 Session Progress Update (`/update-progress`)

**When**: End of a work session, or when the user asks to update progress.
**Reuse**: 100% — identical workflow every time.
**Skill location**: `.claude/commands/update-progress.md`

This is a Claude Code slash command that automates the end-of-session protocol
defined in `iris_ma6/CLAUDE.md`. Invoke with `/update-progress`.

**What it does**:
1. Reads `doc/active/progress.txt`, `doc/active/feature_list.json`, and `ARCHITECTURE.md`
2. Reviews `git diff` (staged + unstaged) to summarize what changed
3. Appends a dated entry to `progress.txt` (Done + Next sections)
4. Updates `feature_list.json` if any feature status changed
5. Updates `ARCHITECTURE.md` if module dependencies changed
6. Updates module `CONTEXT.md` files if interfaces changed

**Why a skill**: This workflow is mechanical and identical every session, but
requires reading multiple files and making coordinated edits. Encoding it as
a slash command ensures consistency and prevents the agent from forgetting
steps. It also avoids polluting `CLAUDE.md` with instructions that are only
needed at session boundaries.

**Design choice — skill vs. hook**: A hook (auto-triggered at session end)
was considered but rejected. Progress updates require judgment (which changes
are worth documenting, what the "Next" items should be) and sometimes the
user wants to skip the update. A user-invoked skill preserves human control.
