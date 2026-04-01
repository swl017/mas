# Harness Engineering & Workspace Blueprints — Rationale

**Date**: 2026-03-18
**Status**: Research reference
**Purpose**: Explains *why* the QRISPY workflow and meta-file system are designed the way they are.

> **Companion document**: For the operational workflow (what the agent must do
> step-by-step), see `qrispy_workflow.md` in this directory.

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

> **Note**: In our project, the Coding Agent's resumption is governed by the QRISPY
> workflow (see `qrispy_workflow.md`). The agent does not autonomously decide what
> to work on next — it reads `progress.txt`, then the engineer directs which QRISPY stage
> to enter. The Initializer/Coding distinction maps to our session start protocol in
> `CLAUDE.md`.

#### Incremental Work Decomposition
Rather than attempting complete implementation in one session, agents focus on single features sequentially. For the operational implementation of this principle, see QRISPY Stage P (vertical slicing) in `qrispy_workflow.md`.

#### Progress Tracking Files
- **Feature list (JSON)**: Structured file with pass/fail status. JSON chosen because "the model is less likely to inappropriately change or overwrite JSON files compared to Markdown files."
- **progress.txt**: Running log of completed work for context recovery across sessions.

#### Environment Hygiene and Session Protocol
Each session must conclude with code ready for merging. The session start/end protocol is defined in `iris_ma6/CLAUDE.md`.

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

## 3. Key Takeaways

| Concept | Core Insight | Our Takeaway |
|---------|-------------|--------------|
| Harness Engineering | The model is commodity; the harness determines success | Invest in CLAUDE.md, CONTEXT.md, and test infrastructure over model selection |
| Context Firewalls | Sub-agents with isolated context prevent "context rot" | Use sub-agents for independent module work (cbf_safety, triangulation) |
| Layered Context Loading | Load only what's needed, when needed | Add per-module CONTEXT.md to reduce token waste |
| Canonical Sources | Each fact lives in one place | Ensure specs are authoritative; don't duplicate in CLAUDE.md |
| Back-Pressure | Silent success, loud failure | Wire tests as verification gates, not optional extras |
| Incremental Decomposition | One feature per session | Break multi-module tasks into per-module sessions |
| Reusable Patterns | Extract skills from repeated work | 5 patterns cover ~90% of module development tasks |
| QRISPY Workflow | Stage gates prevent "outsourcing the thinking" | Full workflow for new features; light flow for small changes. See `qrispy_workflow.md` |

---

## Sources

- [Effective Harnesses for Long-Running Agents — Anthropic Engineering](https://www.anthropic.com/engineering/effective-harnesses-for-long-running-agents)
- [Skill Issue: Harness Engineering for Coding Agents — HumanLayer](https://www.humanlayer.dev/blog/skill-issue-harness-engineering-for-coding-agents)
- [Harness Engineering: The Complete Guide — NxCode](https://www.nxcode.io/resources/news/harness-engineering-complete-guide-ai-agent-codex-2026)
- [Content-Agent-Routing-Promptbase — Jake Van Clief (GitHub)](https://github.com/RinDig/Content-Agent-Routing-Promptbase)
- [Interpreted Context Methodology — Jake Van Clief (GitHub)](https://github.com/RinDig/Interpreted-Context-Methdology)
- [Agent Harness: Understanding Claude Code's Superpower Engine — Medium](https://medium.com/@fruitful2007/agent-harness-understanding-claude-codes-superpower-engine-85e35a7ec764)
- [2025 Was Agents. 2026 Is Agent Harnesses — Medium](https://aakashgupta.medium.com/2025-was-agents-2026-is-agent-harnesses-heres-why-that-changes-everything-073e9877655e)
