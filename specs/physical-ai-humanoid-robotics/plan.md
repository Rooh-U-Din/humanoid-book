# Implementation Plan: Physical AI & Humanoid Robotics Book

**Feature Branch**: `feat/book-content-spec`
**Created**: 2025-12-05
**Status**: Planned
**Input**: User description for plan generation.

## Technical Context

This plan outlines the architecture, structure, workflow, and quality validation for the "Physical AI & Humanoid Robotics" book. The book will be developed using a spec-first approach, leveraging Docusaurus v3+ for documentation and Spec-Kit Plus for content generation and validation. The goal is to produce a fully reproducible, open-source educational resource for building embodied intelligence.

### High-level Architecture Sketch (Repository Layout)

```
/ (Docusaurus v3 root)
├── docs/                          → Introductory static docs
├── src/
│   ├── pages/                     → Index, capstone landing, hardware chooser
│   ├── theme/                     → Custom components (ROS2 code tabs, Jetson pinout diagrams, etc.)
│   └── components/                → Reusable (VideoPlayer, IsaacSimViewer, LiveColab)
├── examples/                      → 20+ standalone, runnable ROS2/Isaac projects
│   ├── ros2/
│   ├── gazebo/
│   ├── isaac-sim/
│   └── capstone/
├── hardware-kit/                  → BOMs, wiring SVGs, 3D printable mounts
├── specs/                         → All .sp files (constitution, specify, toc, plan, chapters)
├── blog/                          → Optional release notes & community builds
├── static/img/                    → Covers, diagrams, photos of real setups
├── docusaurus.config.js           → GitHub Pages deployment + dark mode
└── .github/workflows/deploy.yml   → CI: lint → build → deploy
```

### Detailed Section Structure for Every Chapter

Every technical chapter will follow this exact MDX skeleton:

-   Hero banner with week number + hardware needed
-   Learning outcomes (bullet list)
-   Theory & Motivation (with diagrams)
-   Prerequisites checklist
-   Hands-on Lab (step-by-step numbered)
-   Complete runnable example (code tabs: .sp spec ↔ generated code ↔ launch file)
-   Verification checklist (what you should see)
-   Sim-to-Real notes (if applicable)
-   Further reading + video links
-   Quiz / reflection questions

### Research-While-Writing Workflow

-   **Research-concurrent**: No upfront 3-month literature review.
-   **Per chapter**: 3–5 primary sources (official docs, NVIDIA/ROS GitHub, 2023–2025 arXiv papers on VLA/humanoids).
-   **Citation style**: APA 7th (as mandated by /sp.constitution).
-   **Sources storage**: `/references/zotero-export.json` + PDFs in `/static/references`.
-   **New claims**: Every new major claim gets a `.sp.research-note` file before inclusion.

### Quality Validation Pipeline

**Automated (runs on every PR)**

-   `spec-kit-plus validate --all`
-   `markdownlint` + `docusaurus build` (must succeed <4 min)
-   Link checker (no broken URLs)
-   Lighthouse CI (≥95 performance/accessibility/SEO)
-   Example tests: `ros2 test examples/ros2/**`, `colcon test` in Docker

**Human + Acceptance Validation (success criteria from /sp.specify)**

-   One reader with fresh Ubuntu 22.04 + RTX 4070 Ti + $700 Jetson kit must complete capstone in ≤ 8 hours.
-   All 20+ examples run without modification.
-   Capstone simulation track runs on AWS g5.4xlarge spot instance (< $0.80/hour).
-   Zero missing diagrams or dead videos at release.

## Constitution Check

The plan adheres to the core principles and standards defined in the project constitution:

-   **Teach-by-doing**: Ensured by the "Hands-on Lab" and "Complete runnable example" sections in every chapter.
-   **Spec-first development**: Mandated by requiring `.sp` spec files for all chapters and examples.
-   **Full reproducibility**: Addressed by the detailed setup guides, example repositories, and the goal of cloning and running in <10 minutes.
-   **Open-source first**: Implicit in the GitHub Pages deployment, MIT license, and emphasis on community contributions.
-   **AI-augmented authorship**: Explicitly stated in the overall project description.

**Key Standards, Constraints, and Success Criteria** from the constitution are directly reflected and supported by the detailed planning of the repository structure, chapter templates, and quality validation strategy.

## Architectural Decisions

The following decisions are architecturally significant and require documentation in ADRs:

1.  **ROS 2 Distro Selection**: Choosing Iron for best Jetson + Isaac ROS support in 2025. This impacts compatibility, available packages, and future-proofing.
2.  **Isaac Sim Delivery Method**: Deciding between Local Omniverse Launcher, Docker, or cloud streaming. This affects reader setup complexity, hardware requirements, and accessibility.
3.  **LLM for VLA**: Selecting between local-only (Llama-3.1-8B-Instruct + OpenVLA) and hybrid OpenAI fallback. This influences cost, privacy, and performance.
4.  **Primary Humanoid Model**: Choice of Unitree G1 USD (open) vs Figure 01 (closed) vs custom 22-DoF. This impacts model availability, licensing, and complexity of examples.
5.  **Simulation Backend for Capstone**: Deciding between Isaac Sim only vs Gazebo + Isaac Sim bridge. This affects the scope of simulation environments and complexity of integration.

## Phased Execution

### Phase 1 – Foundation (Weeks 1–2 of writing)

-   Finish `/sp.constitution`, `/sp.specify`, `/sp.toc`, `/sp.plan` (done today).
-   Scaffold Docusaurus repo + GitHub Actions deploy.
-   Create chapter spec template + first 3 example skeletons.

### Phase 2 – Core Modules (Weeks 3–10)

-   One chapter per week in exact `/sp.toc` order.
-   Parallel research → spec → MDX → validation loop.

### Phase 3 – Capstone & Polish (Weeks 11–13)

-   Full capstone integration.
-   Sim-to-real chapter + real hardware videos.
-   Community review + bug-bash.

### Phase 4 – Synthesis & Launch

-   Write foreword + contributing guide.
-   Tag v1.0.0 and announce.
-   Permanent GitHub Pages URL + DOI via Zenodo.

This plan is now locked. All future work must stay inside these guardrails.
