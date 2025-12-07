---
id: 004
title: Review Physical AI & Humanoid Robotics Execution Plan
stage: plan
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-humanoid-robotics
branch: master
user: unknown
command: /sp.plan Physical AI & Humanoid Robotics – Technical Execution Plan

Create
1. High-level architecture sketch of the entire book repository (directory tree + data/flow between tools)
2. Detailed section structure for every chapter (MDX page layout, code tabs, admonitions, embedded live examples)
3. Research-while-writing workflow (concurrent research → spec → draft → validate loop)
4. Quality validation pipeline (automated + human)

Architecture sketch (repository layout)
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

Section structure template (every technical chapter follows this exact MDX skeleton)
- Hero banner with week number + hardware needed
- Learning outcomes (bullet list)
- Theory & Motivation (with diagrams)
- Prerequisites checklist
- Hands-on Lab (step-by-step numbered)
- Complete runnable example (code tabs: .sp spec ↔ generated code ↔ launch file)
- Verification checklist (what you should see)
- Sim-to-Real notes (if applicable)
- Further reading + video links
- Quiz / reflection questions

Research approach
- Research-concurrent: No upfront 3-month literature review
- For each chapter: 3–5 primary sources (official docs, NVIDIA/ROS GitHub, 2023–2025 arXiv papers on VLA/humanoids)
- Citation style: APA 7th (as mandated by /sp.constitution)
- Sources stored in /references/zotero-export.json + PDFs in /static/references
- Every new major claim gets a .sp.research-note file before inclusion

Decisions needing documentation
1. ROS 2 distro → Humble vs Iron vs Jazzy (choose Iron for best Jetson + Isaac ROS support in 2025)
2. Isaac Sim delivery → Local Omniverse Launcher vs Docker vs cloud streaming
3. LLM for VLA → Local-only (Llama-3.1-8B-Instruct + OpenVLA) vs hybrid OpenAI fallback
4. Primary humanoid model → Unitree G1 USD (open) vs Figure 01 (closed) vs custom 22-DoF
5. Simulation backend for capstone → Isaac Sim only vs Gazebo + Isaac Sim bridge

Testing & quality validation strategy
Automated (runs on every PR)
- spec-kit-plus validate --all
- markdownlint + docusaurus build (must succeed <4 min)
- Link checker (no broken URLs)
- Lighthouse CI (≥95 performance/accessibility/SEO)
- Example tests: ros2 test examples/ros2/**, colcon test in Docker

Human + acceptance validation (success criteria from /sp.specify)
- One reader with fresh Ubuntu 22.04 + RTX 4070 Ti + $700 Jetson kit must complete capstone in ≤ 8 hours
- All 20+ examples run without modification
- Capstone simulation track runs on AWS g5.4xlarge spot instance (< $0.80/hour)
- Zero missing diagrams or dead videos at release

Phased execution
Phase 1 – Foundation (Weeks 1–2 of writing)
  → Finish /sp.constitution, /sp.specify, /sp.toc, /sp.plan (done today)
  → Scaffold Docusaurus repo + GitHub Actions deploy
  → Create chapter spec template + first 3 example skeletons

Phase 2 – Core Modules (Weeks 3–10)
  → One chapter per week in exact /sp.toc order
  → Parallel research → spec → MDX → validation loop

Phase 3 – Capstone & Polish (Weeks 11–13)
  → Full capstone integration
  → Sim-to-real chapter + real hardware videos
  → Community review + bug-bash

Phase 4 – Synthesis & Launch
  → Write foreword + contributing guide
  → Tag v1.0.0 and announce
  → Permanent GitHub Pages URL + DOI via Zenodo

This plan is now locked. All future work must stay inside these guardrails.
labels: ["planning", "review"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - specs/physical-ai-humanoid-robotics/plan.md
tests:
  null
---

## Prompt

```text
/sp.plan Physical AI & Humanoid Robotics – Technical Execution Plan

Create
1. High-level architecture sketch of the entire book repository (directory tree + data/flow between tools)
2. Detailed section structure for every chapter (MDX page layout, code tabs, admonitions, embedded live examples)
3. Research-while-writing workflow (concurrent research → spec → draft → validate loop)
4. Quality validation pipeline (automated + human)

Architecture sketch (repository layout)
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

Section structure template (every technical chapter follows this exact MDX skeleton)
- Hero banner with week number + hardware needed
- Learning outcomes (bullet list)
- Theory & Motivation (with diagrams)
- Prerequisites checklist
- Hands-on Lab (step-by-step numbered)
- Complete runnable example (code tabs: .sp spec ↔ generated code ↔ launch file)
- Verification checklist (what you should see)
- Sim-to-Real notes (if applicable)
- Further reading + video links
- Quiz / reflection questions

Research approach
- Research-concurrent: No upfront 3-month literature review
- For each chapter: 3–5 primary sources (official docs, NVIDIA/ROS GitHub, 2023–2025 arXiv papers on VLA/humanoids)
- Citation style: APA 7th (as mandated by /sp.constitution)
- Sources stored in /references/zotero-export.json + PDFs in /static/references
- Every new major claim gets a .sp.research-note file before inclusion

Decisions needing documentation
1. ROS 2 distro → Humble vs Iron vs Jazzy (choose Iron for best Jetson + Isaac ROS support in 2025)
2. Isaac Sim delivery → Local Omniverse Launcher vs Docker vs cloud streaming
3. LLM for VLA → Local-only (Llama-3.1-8B-Instruct + OpenVLA) vs hybrid OpenAI fallback
4. Primary humanoid model → Unitree G1 USD (open) vs Figure 01 (closed) vs custom 22-DoF
5. Simulation backend for capstone → Isaac Sim only vs Gazebo + Isaac Sim bridge

Testing & quality validation strategy
Automated (runs on every PR)
- spec-kit-plus validate --all
- markdownlint + docusaurus build (must succeed <4 min)
- Link checker (no broken URLs)
- Lighthouse CI (≥95 performance/accessibility/SEO)
- Example tests: ros2 test examples/ros2/**, colcon test in Docker

Human + acceptance validation (success criteria from /sp.specify)
- One reader with fresh Ubuntu 22.04 + RTX 4070 Ti + $700 Jetson kit must complete capstone in ≤ 8 hours
- All 20+ examples run without modification
- Capstone simulation track runs on AWS g5.4xlarge spot instance (< $0.80/hour)
- Zero missing diagrams or dead videos at release

Phased execution
Phase 1 – Foundation (Weeks 1–2 of writing)
  → Finish /sp.constitution, /sp.specify, /sp.toc, /sp.plan (done today)
  → Scaffold Docusaurus repo + GitHub Actions deploy
  → Create chapter spec template + first 3 example skeletons

Phase 2 – Core Modules (Weeks 3–10)
  → One chapter per week in exact /sp.toc order
  → Parallel research → spec → MDX → validation loop

Phase 3 – Capstone & Polish (Weeks 11–13)
  → Full capstone integration
  → Sim-to-real chapter + real hardware videos
  → Community review + bug-bash

Phase 4 – Synthesis & Launch
  → Write foreword + contributing guide
  → Tag v1.0.0 and announce
  → Permanent GitHub Pages URL + DOI via Zenodo

This plan is now locked. All future work must stay inside these guardrails.
```

## Response snapshot

The implementation plan for the "Physical AI & Humanoid Robotics" book has been reviewed and confirmed. The plan details the high-level architecture, chapter structure, research workflow, quality validation strategy, and phased execution. It also identifies several key architectural decisions that warrant separate ADRs. The plan is locked, indicating no modifications were made during this re-invocation.

## Outcome

- ✅ Impact: Implementation plan reviewed and confirmed.
- 🧪 Tests: No automated tests for plan review.
- 📁 Files: No changes to plan file.
- 🔁 Next prompts: Pending further user instructions, specifically on whether to document the identified architectural decisions with ADRs.
- 🧠 Reflection: Successfully confirmed the existing implementation plan, and reiterated key architectural decisions.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): None
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): None
