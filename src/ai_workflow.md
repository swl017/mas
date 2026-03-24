# Harness Engineering & Workspace Blueprints for AI-Backed Development

**Date**: 2026-03-18
**Status**: Implemented (Actions 1-3 done) / Research reference
**Purpose**: Investigate harness engineering and Jake Van Clief's 3-layer workspace blueprint, and identify how to apply them to our development workflow.

---

## 1. Harness Engineering

### 1.1 What It Is

Harness engineering is the discipline of designing the infrastructure, constraints, and feedback loops that wrap around AI coding agents. The core formula:

```
coding agent = AI model(s) + harness
```

The key insight: **the model is commodity** — Claude, GPT-4, and Gemini perform similarly. The harness determines whether agents succeed or fail. LangChain demonstrated this empirically: they improved from 52.8% to 66.5% accuracy on Terminal Bench 2.0 by changing *only the harness*, not the model.

As Mitchell Hashimoto put it: "anytime you find an agent makes a mistake, you take the time to engineer a solution such that the agent never makes that mistake again."

### 1.2 Three Pillars

| Pillar | Purpose | Examples |
|--------|---------|----------|
| **Context Engineering** | Provide the right information at the right time | CLAUDE.md, AGENTS.md, design docs, CI status |
| **Architectural Constraints** | Limit the solution space to improve reliability | Linters, pre-commit hooks, dependency layering, structural tests |
| **Entropy Management** | Prevent codebase degradation over time | "Garbage collection" agents for doc drift, dead code, circular deps |

The critical principle: **anything the agent can't access in-context doesn't exist** to that agent.

### 1.3 Configuration Surfaces

The harness exposes several configuration points:

1. **System Prompts (CLAUDE.md / AGENTS.md)**: Hand-crafted, concise (<60 lines ideally), not auto-generated. Contains only what the agent needs.

2. **Tools & MCP Servers**: Extend agent capabilities but avoid tool bloat — excessive tool descriptions push agents toward worse performance.

3. **Skills**: Progressive disclosure mechanism. Agents access detailed instructions only when needed, avoiding upfront context overload.

4. **Sub-Agents (Context Firewalls)**: Encapsulate tasks in isolated context windows. Supported by Chroma's research showing performance degradation at longer context lengths ("context rot").

5. **Hooks**: Scripts executed at lifecycle events — notifications, approvals, typechecks, build verification.

6. **Back-Pressure Mechanisms**: Context-efficient verification where success is silent and only failures surface (tests, typechecks, coverage).

### 1.4 Design Patterns

#### Two-Agent Pattern (Anthropic)
For long-running tasks spanning multiple context windows:
- **Initializer Agent**: First session that establishes environment — creates feature list (JSON), progress file, init script, git repo
- **Coding Agent**: Subsequent sessions that pick up where the last left off, working on one feature at a time

#### Incremental Work Decomposition
Rather than attempting complete implementation in one session, agents focus on single features sequentially. This addresses the agent's tendency to "do too much at once."

#### Progress Tracking Files
- **Feature list (JSON)**: Structured file with pass/fail status. JSON chosen because "the model is less likely to inappropriately change or overwrite JSON files compared to Markdown files."
- **claude-progress.txt**: Running log of completed work for context recovery across sessions.

#### Environment Hygiene
Each session must conclude with code ready for merging: no major bugs, well-documented, orderly. This eliminates cleanup work for subsequent sessions.

#### Session Start Protocol
1. Verify directory structure
2. Review git history
3. Consult feature list
4. Test basic functionality before starting new work

### 1.5 Implementation Maturity Levels

| Level | Scope | Setup Time | Key Elements |
|-------|-------|-----------|--------------|
| Basic | Individual dev | 1-2 hours | CLAUDE.md, linting hooks, tests |
| Team | 3-10 developers | 1-2 days | AGENTS.md, CI constraints, templates |
| Production | Organization | 1-2 weeks | Middleware, observability, monitoring |

### 1.6 What Works vs. What Doesn't

**Works:**
- Start simple; add configuration only after observing failures
- Test and iterate continuously; discard ineffective elements
- Distribute battle-tested configs team-wide
- Optimize for iteration speed over first-attempt success

**Doesn't Work:**
- Designing ideal harnesses preemptively
- Installing dozens of skills/servers "just in case"
- Running full test suites after every change
- Micro-optimizing tool access across sub-agents

---

## 2. Jake Van Clief's 3-Layer Workspace Blueprint (ICM)

### 2.1 What It Is

The Interpreted Context Methodology (ICM) is a filesystem-based approach to AI agent orchestration. It replaces framework-level orchestration with folder structure as the primary architecture. Based on 1970s Unix pipe-and-filter principles applied to modern AI context management.

Core philosophy: **"Give each process exactly the data it needs, and nothing else."** Irrelevant context dilutes agent attention quality.

### 2.2 The Layer Model

```
Layer 0: CLAUDE.md        (~800 tokens)  — DNS/system prompt, ALWAYS loaded
Layer 1: Root CONTEXT.md  (~300 tokens)  — Routing table, read ONCE per session
Layer 2: Folder CONTEXT.md (200-500 tok) — Per-task workspace instructions
Layer 3: Reference files   (500-3000 tok)— Stable resources, selectively loaded
Layer 4: Working artifacts (varies)      — Run-specific outputs and source material
```

**Token efficiency**: ~5,200 tokens per task vs. ~15,000+ with monolithic loading (65% reduction).

### 2.3 How Each Layer Works

**Layer 0 — CLAUDE.md (Always loaded)**
Functions as architectural DNS:
- Complete folder mapping
- ID system definitions
- Navigation rules for agents
- Stays lean — paid on every conversation

**Layer 1 — Root CONTEXT.md (Read once)**
Acts as a load balancer:
- Maps tasks to appropriate workspaces
- Directs traffic without doing actual work
- No reference material — only routing instructions

**Layer 2 — Per-Folder CONTEXT.md (Task-specific)**
Each workspace's contract specifying:
- **Inputs**: Exact file locations and sections to load
- **Process**: Step-by-step workflow with checkpoints
- **Outputs**: Artifact names, locations, and formats

**Layer 3 — Reference Files (Selective loading)**
Loaded only as specified by Layer 2:
- Design systems, voice rules, API specs
- Selective section loading: a 174-line file might only need ~80 lines loaded

**Layer 4 — Working Artifacts (Per-run)**
Run-specific outputs serving as handoff points between stages.

### 2.4 Five Design Principles

1. **Single-Purpose Stages**: Each folder handles one task. Research doesn't also write; writing doesn't also build.

2. **Plain Text Interface**: All communication through markdown. Any human with a text editor can inspect or modify any artifact.

3. **Layered Context Loading**: Agents retrieve only necessary context for the current stage.

4. **Edit Surfaces Between Stages**: Every intermediate output becomes editable before the next stage runs.

5. **Factory Configuration**: One-time setup for preferences and style. Each subsequent run uses identical config with different input.

### 2.5 Three Foundational Patterns

| Pattern | Rule | Rationale |
|---------|------|-----------|
| **Canonical Sources** | Every piece of information lives in exactly one location | Prevents drift when updates occur |
| **One-Way Dependencies** | References flow directionally (A -> B, never B -> A) | Bidirectional references create O(n^2) maintenance |
| **Selective Section Loading** | Load "these sections" not "this file" | Minimizes irrelevant context |

### 2.6 Example: Script Writing Task

```
Layer 0: CLAUDE.md                              ~800 tokens
Layer 1: Root CONTEXT.md → routes to script-lab  ~300 tokens
Layer 2: script-lab/CONTEXT.md                   ~300 tokens
Layer 3: voice-and-tone.md [voice rules only]   ~2,000 tokens
         hooks.md                                ~800 tokens
         template.md                             ~800 tokens
                                          Total: ~5,200 tokens
```

vs. loading everything: ~15,000+ tokens with degraded performance.

### 2.7 When ICM Works / Doesn't Work

**Good fit**: Sequential, reviewable, repeatable workflows — content production, research-to-analysis pipelines, reporting, training development.

**Poor fit**: Real-time agent collaboration, high-concurrency systems, automated branching logic requiring mid-pipeline decisions.

---

## 3. Meta-File Catalog

There is no single standard — different communities (Anthropic, ICM, AGENTS.md spec, HumanLayer) define overlapping but distinct meta-files. Below is a unified catalog of all meta-files referenced across these sources, with our adopted subset marked.

### 3.1 Universal Meta-Files (Cross-Community)

| File | Origin | Purpose | Where It Goes | Token Budget | Our Status |
|------|--------|---------|---------------|-------------|------------|
| **CLAUDE.md** | Claude Code | System prompt: project conventions, build commands, coding rules. Always loaded by Claude. | Repo root + nested per-directory | ~800 tokens ideal | **Exists** — comprehensive but monolithic |
| **AGENTS.md** | [agents.md spec](https://agents.md/) (Linux Foundation) | Agent-agnostic version of CLAUDE.md. Works across 25+ tools (Claude, Codex, Cursor, Copilot, etc.). Same purpose, broader compatibility. | Repo root + nested subdirs | Flexible | Not used — CLAUDE.md covers our needs |

### 3.2 ICM Meta-Files (Jake Van Clief)

| File | Layer | Purpose | Where It Goes | Token Budget |  Our Status |
|------|-------|---------|---------------|-------------|------------|
| **CONTEXT.md** (root) | L1 | Routing table: maps tasks to workspace folders. No reference material — only navigation. | Workspace root | ~300 tokens | **Adopt** |
| **CONTEXT.md** (per-folder) | L2 | Stage contract: specifies Inputs, Process, Outputs for a module. Must stay <80 lines. | Each sub-module folder | 200-500 tokens | **Adopt** |
| **questionnaire.md** | Setup | One-time config: brand, voice, preferences. Flat format, not per-run. | `setup/` | Varies | Not applicable |

### 3.3 Harness Engineering Meta-Files (Anthropic / Community)

| File | Purpose | Where It Goes | Format | Our Status |
|------|---------|---------------|--------|------------|
| **feature_list.json** | Tracks implementation status across sessions. JSON chosen because agents are less likely to corrupt structured data vs. markdown. | Active work directory | JSON with id, name, status, notes fields | **Adopt** — in `doc/active/` |
| **progress.txt** | Running session handoff log. Each session appends what was done, what's next. | Active work directory | Plain text, append-only | **Adopt** — in `doc/active/` |
| **init.sh** | Bootstraps dev environment for new agent sessions. Runs setup + basic verification. | Project root | Shell script | Not needed — `./isaaclab.sh` covers this |
| **verify.sh** | Back-pressure: runs tests/checks at session end. | Project root | Shell script | **Investigate** (Action 4) |

### 3.4 Structural Documentation Files

| File | Purpose | Where It Goes | Our Status |
|------|---------|---------------|------------|
| **ARCHITECTURE.md** | Module dependency map, data flow, integration boundaries. Scoped per project iteration. | `iris_ma6/` root | **Adopt** |
| **agent_docs/*.md** | Progressive disclosure: topic-specific guides loaded on demand. CLAUDE.md points to them. | `doc/` or `agent_docs/` | Partially exists — our `doc/*_spec.md` files serve this role |

### 3.5 What We Adopt vs. Skip

**Adopted (all implemented 2026-03-18):**
1. `CONTEXT.md` (per-module) — 11 files created across all sub-modules
2. `ARCHITECTURE.md` — created at `iris_ma6/ARCHITECTURE.md`
3. `feature_list.json` + `progress.txt` — created in `doc/active/`
4. `iris_ma6/CLAUDE.md` — session workflow protocol with meta-file maintenance rules

**Skip (with rationale):**
- `AGENTS.md` — redundant with CLAUDE.md for a single-agent-tool project
- `questionnaire.md` — designed for content workflows, not code
- `init.sh` — `./isaaclab.sh` already handles environment setup
- Root-level `CONTEXT.md` (L1 router) — our project has a single workspace (iris_ma6), so the routing layer adds overhead without benefit. If we later have parallel workspaces, revisit.

---

## 4. Application to Our Development Flow

### 4.1 Current State Assessment

| Element | Status | Equivalent Layer |
|---------|--------|-----------------|
| `/IsaacLab/CLAUDE.md` | Exists, comprehensive | Layer 0 (global) |
| `iris_ma6/CLAUDE.md` | **Created** — session workflow protocol | Layer 0 (local) |
| `iris_ma6/ARCHITECTURE.md` | **Created** — dependency graph + data flow | Layer 1 |
| `<module>/CONTEXT.md` | **Created** — 11 modules covered | Layer 2 |
| `doc/*_spec.md` | Exists (12 spec files) | Layer 3 |
| `doc/active/` | **Created** — feature_list.json + progress.txt | Session tracking |
| `doc/backlog/` | Exists | Future tasks / research |
| `tests/` directory | Exists per module | Back-pressure |
| Test runner pattern | Standardized in CLAUDE.md | Verification |

**Current module structure:**
```
iris_ma6/
├── ARCHITECTURE.md          # Module dependency graph
├── CLAUDE.md                # Session workflow protocol
├── asset/CONTEXT.md
├── bbox_raycaster_v2/CONTEXT.md
├── cbf_safety/CONTEXT.md
├── controller/CONTEXT.md
├── curriculum/CONTEXT.md
├── delay_system_v3/CONTEXT.md
├── doc/
│   ├── active/              # feature_list.json, progress.txt
│   ├── backlog/             # Future tasks, research (incl. this file)
│   ├── ai_workflow.md       # This file (rationale + patterns)
│   └── *_spec.md            # Authoritative specs
├── domain_randomization/CONTEXT.md
├── initial_states/CONTEXT.md
├── target_controller/CONTEXT.md
├── tests/
├── triangulation/CONTEXT.md
├── visualization/CONTEXT.md
└── iris_ma_env6_test.py     # Main environment
```

### 4.2 Gaps Identified

1. ~~**No per-module CONTEXT.md files**~~ — **Resolved**: 11 CONTEXT.md files created across all sub-modules.

2. ~~**No progress tracking across sessions**~~ — **Resolved**: `doc/active/feature_list.json` + `progress.txt` created. Spec docs define *what* to build; these track *where we are*.

3. ~~**No explicit dependency map**~~ — **Resolved**: `ARCHITECTURE.md` created with directed dependency graph and data flow.

4. **CLAUDE.md is monolithic** — **Partially resolved**: Nested `iris_ma6/CLAUDE.md` created for session workflow. Root CLAUDE.md remains comprehensive but functional. Revisit if token pressure becomes measurable.

5. **No hooks for back-pressure** — **Open**: Tests exist but aren't wired as automated verification gates. Needs investigation on aligning automated checks with user intent (see Action 4).

### 4.3 Recommended Actions

#### Action 1: Add Per-Module CONTEXT.md Files (ICM Layer 2) — DONE
Created 11 CONTEXT.md files across all sub-modules. Template used:

```markdown
# CBF Safety Module

## Purpose
Control Barrier Function safety layer for inter-agent collision avoidance.

## Inputs
- Agent positions and velocities from env state
- Safety margins from domain_randomization config

## Outputs
- Safe action corrections (penalty-based)
- Safety violation flags

## Dependencies
- controller/ (reads action outputs)
- domain_randomization/ (reads safety margin configs)

## Key Files
- cbf_layer.py: Core CBF computation
- cbf_cfg.py: Configuration dataclass
- tests/: Test suite (run via ./isaaclab.sh -p ...)

## Spec
- doc/safety_spec.md
```

**Priority**: High. This directly reduces Claude's exploration time from ~5 file reads to 1.

#### Action 2: Create ARCHITECTURE.md for iris_ma6 — DONE
Created `iris_ma6/ARCHITECTURE.md` with directed dependency graph, data flow diagram, and key data containers. Scoped to iris_ma6 since ~90% of development work happens here. Each iris_ma iteration should have its own ARCHITECTURE.md.

```
initial_states → env
domain_randomization → env
controller → env
delay_system_v3 → env
cbf_safety → controller
triangulation → delay_system_v3
bbox_raycaster_v2 → triangulation
visualization → env (all modules)
curriculum → env (reward scaling)
target_controller → env
```

**Priority**: High. Enforces one-way dependencies and helps Claude understand module boundaries without reading code.

#### Action 3: Implement Session Progress Tracking — DONE
Created `doc/active/` with `feature_list.json` (11 features tracked) and `progress.txt` (append-only session log). `doc/backlog/` remains reserved for low-urgency future tasks. Session workflow protocol in `iris_ma6/CLAUDE.md` instructs Claude to read these at session start and update at session end.

Originally proposed to hold:
- `feature_list.json` — structured tracking of implementation status
- `progress.txt` — running log for session handoff
- Per-task context files as needed

**Example feature_list.json:**
```json
{
  "features": [
    {"id": "cbf-v2", "name": "CBF safety layer v2", "status": "in_progress", "notes": "Penalty-based approach implemented, testing needed"},
    {"id": "curriculum", "name": "Curriculum learning", "status": "not_started", "notes": "Spec in doc/curriculum_spec.md"}
  ]
}
```

**Folder semantics:**
```
doc/
├── backlog/          # Low-urgency future tasks and research
├── active/           # Multi-session tracking for in-progress work
│   ├── feature_list.json
│   └── progress.txt
├── *_spec.md         # Authoritative specs (what to build)
└── ARCHITECTURE.md   # → lives at iris_ma6/ root, not here
```

**Priority**: Medium. Most valuable for long-running implementation tasks.

#### Action 4: Add Back-Pressure Hooks
Create a pre-commit or session-end verification script:

```bash
#!/bin/bash
# verify.sh — back-pressure mechanism
echo "Running module tests..."
./isaaclab.sh -p source/.../iris_ma6/tests/run_tests.py --headless 2>&1 | tail -5
echo "Checking imports..."
python -c "from isaaclab_tasks.direct.iris_ma6 import *" 2>&1
```

**Open question**: Some tests require user attendance to verify results (e.g., visual inspection, simulation behavior). Fully automated back-pressure works for deterministic checks (imports, shapes, value ranges) but not for subjective assessments. Worth investigating:
- How to write test assertions that truly capture user intent (not just "does it run")
- Which test categories can be fully automated vs. which need human-in-the-loop
- Whether structured test output (metrics, plots) can reduce the need for live observation

**Priority**: Low-Medium. Good to have, but needs further investigation on aligning automated checks with user intent.

#### Action 5: Restructure CLAUDE.md as Layered Router
Split the current monolithic CLAUDE.md into:
- **Layer 0** (root CLAUDE.md): Folder map, conventions, entry points only (~800 tokens)
- **Layer 3** (doc/ specs): Detailed implementation guidance (already exists)
- Move verbose test templates and patterns into a `doc/testing_guide.md` reference file

**Trade-off — token efficiency vs. reusability**: Moving the testing guide to a separate file saves ~tokens on every conversation, but raises the question: will Claude reliably find and load it when needed? Two options:
1. **Keep in CLAUDE.md** (current): Always loaded, always available. Token-expensive but zero risk of being missed. Good if testing guidance is needed in >50% of sessions.
2. **Move to doc/testing_guide.md with pointer**: Add a one-line reference in CLAUDE.md (e.g., `For test patterns, see doc/testing_guide.md`). Token-efficient but requires Claude to make an extra file read. Works well with CONTEXT.md files that explicitly list it as an input for test-related tasks.

Given that our CLAUDE.md testing section is heavily used and acts as a reusable template across all modules, **keeping it in CLAUDE.md is defensible** — the cost is tokens, but the benefit is guaranteed availability. If token pressure becomes a real problem, the pointer approach with CONTEXT.md routing is the fallback.

**Priority**: Low. Revisit only if CLAUDE.md token cost becomes a measurable bottleneck.

### 4.4 What NOT To Do

Per harness engineering best practices:
- Don't over-engineer upfront — add CONTEXT.md files only to modules being actively developed
- Don't add dozens of hooks "just in case"
- Don't restructure everything at once — iterate based on observed failures
- Don't duplicate information — each fact lives in exactly one file

---

## 5. Reusable Development Patterns (Agent Skills)

These patterns were extracted from building all 11 iris_ma6 modules. They represent repeatable workflows that the agent should follow when creating or modifying modules.

### 5.1 Module Scaffold

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

### 5.2 Configuration Dataclass

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

### 5.3 Test Suite

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

### 5.4 Specification Document

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

### 5.5 Environment Integration

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

### 5.6 Pattern Selection Guide

| Task | Pattern(s) to Use |
|------|-------------------|
| "Add a new module" | Scaffold → Config → Implementation → Test Suite → Spec → Integration |
| "Add tests to existing module" | Test Suite only |
| "Define a new feature before coding" | Spec → then Scaffold when ready |
| "Wire an existing module into env" | Integration only |
| "Add parameters to existing module" | Config (update existing `*_cfg.py`) |

### 5.7 Session Progress Update (`/update-progress`)

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

---

## 6. Key Takeaways

| Concept | Core Insight | Our Takeaway |
|---------|-------------|--------------|
| Harness Engineering | The model is commodity; the harness determines success | Invest in CLAUDE.md, CONTEXT.md, and test infrastructure over model selection |
| Context Firewalls | Sub-agents with isolated context prevent "context rot" | Use sub-agents for independent module work (cbf_safety, triangulation) |
| Layered Context Loading | Load only what's needed, when needed | Add per-module CONTEXT.md to reduce token waste |
| Canonical Sources | Each fact lives in one place | Ensure specs are authoritative; don't duplicate in CLAUDE.md |
| Back-Pressure | Silent success, loud failure | Wire tests as verification gates, not optional extras |
| Incremental Decomposition | One feature per session | Break multi-module tasks into per-module sessions |
| Reusable Patterns | Extract skills from repeated work | 5 patterns cover ~90% of module development tasks |

---

## Sources

- [Effective Harnesses for Long-Running Agents — Anthropic Engineering](https://www.anthropic.com/engineering/effective-harnesses-for-long-running-agents)
- [Skill Issue: Harness Engineering for Coding Agents — HumanLayer](https://www.humanlayer.dev/blog/skill-issue-harness-engineering-for-coding-agents)
- [Harness Engineering: The Complete Guide — NxCode](https://www.nxcode.io/resources/news/harness-engineering-complete-guide-ai-agent-codex-2026)
- [Content-Agent-Routing-Promptbase — Jake Van Clief (GitHub)](https://github.com/RinDig/Content-Agent-Routing-Promptbase)
- [Interpreted Context Methodology — Jake Van Clief (GitHub)](https://github.com/RinDig/Interpreted-Context-Methdology)
- [Agent Harness: Understanding Claude Code's Superpower Engine — Medium](https://medium.com/@fruitful2007/agent-harness-understanding-claude-codes-superpower-engine-85e35a7ec764)
- [2025 Was Agents. 2026 Is Agent Harnesses — Medium](https://aakashgupta.medium.com/2025-was-agents-2026-is-agent-harnesses-heres-why-that-changes-everything-073e9877655e)
