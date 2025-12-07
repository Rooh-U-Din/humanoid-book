---
id: 001
title: Cross-artifact analysis attempt
stage: misc
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.analyze
labels: ["analysis", "prerequisites", "blocked"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
tests:
---

## Prompt

/sp.analyze

## Response snapshot

Analysis was blocked due to missing prerequisite artifacts. The feature directory `specs/master` does not exist, which means the specification workflow has not been initiated.

**Analysis Result:** BLOCKED - Missing Prerequisites
- No feature directory found
- Required files (spec.md, plan.md, tasks.md) do not exist
- User must run /sp.specify → /sp.plan → /sp.tasks before /sp.analyze can execute

**Recommended Actions:**
1. Run `/sp.specify` to create initial feature specification
2. Run `/sp.plan` to generate architectural plan
3. Run `/sp.tasks` to generate implementation tasks
4. Then run `/sp.analyze` to perform cross-artifact consistency analysis

## Outcome

- ✅ Impact: Identified missing prerequisites and provided clear guidance on required workflow steps
- 🧪 Tests: No tests run (analysis blocked)
- 📁 Files: 1 PHR created (this record)
- 🔁 Next prompts: User should run /sp.specify to start specification workflow
- 🧠 Reflection: Analysis command correctly validated prerequisites before attempting to load artifacts, preventing wasted token usage on non-existent files

## Evaluation notes (flywheel)

- Failure modes observed: Command invoked out of sequence before specification artifacts exist
- Graders run and results (PASS/FAIL): N/A - prerequisite check correctly blocked execution
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): User should follow the proper workflow sequence: specify → plan → tasks → analyze
