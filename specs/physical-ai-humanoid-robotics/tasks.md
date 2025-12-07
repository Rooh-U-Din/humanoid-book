# Tasks: Physical AI & Humanoid Robotics Book

**Input**: Design documents from `/specs/physical-ai-humanoid-robotics/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The feature specification does not explicitly request test-first development for every task, but success criteria imply runnable examples and passing tests for the overall project. Individual example tests are covered.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/
- Paths shown below assume Docusaurus project structure from plan.md

## Phase 1: Setup (Shared Infrastructure - Foundation)

**Purpose**: Project initialization, Docusaurus scaffolding, and core repository structure.

- [ ] T001 Create Docusaurus project scaffolding at `.`
- [ ] T002 Configure `docusaurus.config.js` for GitHub Pages deployment
- [ ] T003 [P] Create GitHub Actions workflow `.github/workflows/deploy.yml` for lint, build, and deploy
- [ ] T004 Create base `src/pages/index.js` for book landing page
- [ ] T005 [P] Create `docs/` directory for introductory static docs
- [ ] T006 Create `specs/` directory for all .sp files
- [ ] T007 Create `examples/` directory and subdirectories `ros2/`, `gazebo/`, `isaac-sim/`, `capstone/`
- [ ] T008 [P] Create `hardware-kit/` directory for BOMs, wiring SVGs, 3D printable mounts
- [ ] T009 [P] Create `blog/` directory for optional release notes
- [ ] T010 [P] Create `static/img/` directory for covers, diagrams, photos
- [ ] T011 Create `/references/` directory for Zotero export and PDFs
- [ ] T012 Create a generic chapter spec template `specs/chapter-template.sp`
- [ ] T013 Create first 3 example skeletons within `examples/`
- [ ] T014 [P] Implement markdownlint configuration at `.`
- [ ] T015 [P] Integrate link checker into CI workflow `.github/workflows/deploy.yml`
- [ ] T016 [P] Configure Lighthouse CI for performance/accessibility/SEO checks in `.github/workflows/deploy.yml`

---

## Phase 2: Foundational (Blocking Prerequisites - Chapter Content & Core Features)

**Purpose**: Core infrastructure and base components that MUST be complete before specific user story chapters can be fully implemented.

**⚠️ CRITICAL**: No detailed chapter content work can begin until this phase is complete.

- [ ] T017 Create `src/theme/` directory for custom Docusaurus components
- [ ] T018 Create `src/components/` directory for reusable components (VideoPlayer, IsaacSimViewer, LiveColab)
- [ ] T019 Develop Docusaurus MDX components for ROS2 code tabs in `src/theme/CodeTabs/ROS2CodeTab.js`
- [ ] T020 Develop Docusaurus MDX components for Jetson pinout diagrams in `src/theme/MDXComponents/JetsonPinout.js`
- [ ] T021 Develop reusable `VideoPlayer` component in `src/components/VideoPlayer.js`
- [ ] T022 Develop reusable `IsaacSimViewer` component in `src/components/IsaacSimViewer.js`
- [ ] T023 Develop reusable `LiveColab` component in `src/components/LiveColab.js`
- [ ] T024 Define standard MDX skeleton for technical chapters in `docs/chapter-template.mdx` (incorporating hero banner, outcomes, theory, lab, example, verification, sim-to-real, reading, quiz)

**Checkpoint**: Foundation ready - user story chapter content can now begin, building on shared components and structure.

---

## Phase 3: User Story 1 - Setting up the AI Lab (Priority: P1) 🎯 MVP

**Goal**: A reader can successfully set up their physical AI lab, choosing a budget track, and configure their workstation or cloud environment. Covers Chapter 02.

**Independent Test**: Reader successfully assembles and flashes Jetson Kit, sets up RTX workstation, or understands cloud fallback path and uses cost calculator.

### Implementation for User Story 1

- [ ] T025 [P] [US1] Create chapter spec `specs/02-hardware-lab-setup.sp` for Chapter 02
- [ ] T026 [US1] Draft MDX content for Chapter 02 in `docs/02-hardware-lab-setup.mdx` (Assemble $700 Economy Jetson Student Kit)
- [ ] T027 [US1] Draft MDX content for Chapter 02 in `docs/02-hardware-lab-setup.mdx` (Set up RTX workstation - Ubuntu 22.04 + drivers + Docker)
- [ ] T028 [P] [US1] Create wiring SVGs for Economy Jetson Kit in `hardware-kit/jetson-wiring.svg`
- [ ] T029 [P] [US1] Create flashing guide content `hardware-kit/jetpack-flashing-guide.md`
- [ ] T030 [US1] Draft MDX content for Chapter 02 in `docs/02-hardware-lab-setup.mdx` (Cloud fallback path + cost calculator)
- [ ] T031 [US1] Embed wiring SVGs and flashing guide into Chapter 02 MDX content
- [ ] T032 [US1] Validate `specs/02-hardware-lab-setup.sp` using `spec-kit-plus validate`

**Checkpoint**: Chapter 02 content is complete and validated, providing a working guide for lab setup.

---

## Phase 4: User Story 2 - Mastering ROS 2 Fundamentals (Priority: P1)

**Goal**: A reader can learn ROS 2 fundamentals, create robotic components, build packages, understand humanoid descriptions, and run teleop controllers. Covers Chapters 03 and 04.

**Independent Test**: Reader can create nodes, topics, services, actions with `rclpy`, build a ROS 2 package, understand URDF/Xacro, master launch files, bridge Python AI agents, and run a teleop controller on Jetson.

### Implementation for User Story 2

- [ ] T033 [P] [US2] Create chapter spec `specs/03-ros2-fundamentals.sp` for Chapter 03
- [ ] T034 [P] [US2] Create chapter spec `specs/04-ros2-advanced.sp` for Chapter 04
- [ ] T035 [US2] Draft MDX content for Chapter 03 in `docs/03-ros2-fundamentals.mdx` (Nodes, topics, services, actions with rclpy)
- [ ] T036 [US2] Create runnable example for ROS 2 package creation in `examples/ros2/first-package/`
- [ ] T037 [US2] Draft MDX content for Chapter 03 in `docs/03-ros2-fundamentals.mdx` (URDF/Xacro for humanoid description)
- [ ] T038 [US2] Draft MDX content for Chapter 04 in `docs/04-ros2-advanced.mdx` (Launch files and parameter servers)
- [ ] T039 [US2] Create runnable example for complex launch files in `examples/ros2/advanced-launch/`
- [ ] T040 [US2] Draft MDX content for Chapter 04 in `docs/04-ros2-advanced.mdx` (Bridge Python AI agents to low-level controllers)
- [ ] T041 [US2] Create runnable example for Python AI agent bridging in `examples/ros2/ai-bridge/`
- [ ] T042 [US2] Draft MDX content for Chapter 04 in `docs/04-ros2-advanced.mdx` (First teleop controller on Jetson)
- [ ] T043 [US2] Create runnable example for Jetson teleop controller in `examples/ros2/jetson-teleop/`
- [ ] T044 [US2] Validate `specs/03-ros2-fundamentals.sp` and `specs/04-ros2-advanced.sp` using `spec-kit-plus validate`

**Checkpoint**: Chapters 03 and 04 content complete, with runnable examples, covering ROS 2 fundamentals.

---

## Phase 5: User Story 6 - Integrating Vision-Language-Action Models (Priority: P1)

**Goal**: A reader can integrate Vision-Language-Action models to enable conversational control of the humanoid robot, translating voice commands into physical actions locally. Covers Chapter 09.

**Independent Test**: Reader successfully runs a Whisper → LLM → ROS 2 action pipeline locally to turn a voice command into navigation and manipulation goals.

### Implementation for User Story 6

- [ ] T045 [P] [US6] Create chapter spec `specs/09-vla-fundamentals.sp` for Chapter 09
- [ ] T046 [US6] Draft MDX content for Chapter 09 in `docs/09-vla-fundamentals.mdx` (Whisper → LLM → ROS 2 action pipeline)
- [ ] T047 [US6] Create runnable example for Whisper + LLM + ROS 2 action pipeline in `examples/vla/whisper-llm-ros2-action/`
- [ ] T048 [US6] Draft MDX content for Chapter 09 in `docs/09-vla-fundamentals.mdx` (Turn “Pick up the red cup” into navigation + manipulation goals)
- [ ] T049 [US6] Create runnable example for "Pick up the red cup" scenario in `examples/vla/red-cup-manipulation/`
- [ ] T050 [US6] Ensure local-only operation for VLA pipeline (no OpenAI API required after Week 9)
- [ ] T051 [US6] Validate `specs/09-vla-fundamentals.sp` using `spec-kit-plus validate`

**Checkpoint**: Chapter 09 content complete, with runnable examples, demonstrating local VLA integration.

---

## Phase 6: User Story 7 - Completing the Autonomous Humanoid Capstone (Priority: P1)

**Goal**: A reader can complete an end-to-end capstone project, demonstrating an autonomous conversational humanoid in simulation or real hardware. Covers Chapter 10.

**Independent Test**: Reader successfully clones and runs the complete capstone repository in under 10 minutes, achieving voice command to grasp functionality.

### Implementation for User Story 7

- [ ] T052 [P] [US7] Create chapter spec `specs/10-capstone-project.sp` for Chapter 10
- [ ] T053 [US7] Draft MDX content for Chapter 10 in `docs/10-capstone-project.mdx` (Full end-to-end project: voice command → path planning → object detection → grasp)
- [ ] T054 [US7] Create the core capstone project in `examples/capstone/autonomous-humanoid/`
- [ ] T055 [P] [US7] Develop simulation track for capstone (100% simulation) within `examples/capstone/autonomous-humanoid/`
- [ ] T056 [P] [US7] Develop real hardware track for capstone (Jetson + RealSense + optional Unitree Go2/G1) within `examples/capstone/autonomous-humanoid/`
- [ ] T057 [US7] Ensure capstone repo can be cloned and run in <10 minutes
- [ ] T058 [US7] Validate `specs/10-capstone-project.sp` using `spec-kit-plus validate`

**Checkpoint**: Chapter 10 content and capstone project complete, demonstrating autonomous conversational humanoid.

---

## Phase 7: User Story 3 - Simulating Humanoids (Priority: P2)

**Goal**: A reader can use simulation environments like Gazebo and NVIDIA Isaac Sim to build digital twins and generate synthetic data for humanoids. Covers Chapters 05 and 06.

**Independent Test**: Reader successfully builds a world in Gazebo with humanoid URDF, simulates sensors, and generates domain-randomized data in Isaac Sim.

### Implementation for User Story 3

- [ ] T059 [P] [US3] Create chapter spec `specs/05-gazebo-basics.sp` for Chapter 05
- [ ] T060 [P] [US3] Create chapter spec `specs/06-isaac-sim-intro.sp` for Chapter 06
- [ ] T061 [US3] Draft MDX content for Chapter 05 in `docs/05-gazebo-basics.mdx` (Build worlds and spawn humanoid URDFs)
- [ ] T062 [US3] Create runnable example for Gazebo world and humanoid spawning in `examples/gazebo/humanoid-world/`
- [ ] T063 [US3] Draft MDX content for Chapter 05 in `docs/05-gazebo-basics.mdx` (Simulate LiDAR, depth cameras, IMUs + Add plugins)
- [ ] T064 [US3] Create runnable example for Gazebo sensor simulation and plugins in `examples/gazebo/sensor-plugins/`
- [ ] T065 [US3] Draft MDX content for Chapter 06 in `docs/06-isaac-sim-intro.mdx` (Run Isaac Sim on RTX workstation)
- [ ] T066 [US3] Create runnable example for basic Isaac Sim setup in `examples/isaac-sim/basic-setup/`
- [ ] T067 [US3] Draft MDX content for Chapter 06 in `docs/06-isaac-sim-intro.mdx` (Import humanoid USD assets)
- [ ] T068 [US3] Create runnable example for importing USD assets in `examples/isaac-sim/import-usd/`
- [ ] T069 [US3] Draft MDX content for Chapter 06 in `docs/06-isaac-sim-intro.mdx` (Generate domain-randomized training data)
- [ ] T070 [US3] Create runnable example for synthetic data generation in `examples/isaac-sim/synthetic-data/`
- [ ] T071 [US3] Validate `specs/05-gazebo-basics.sp` and `specs/06-isaac-sim-intro.sp` using `spec-kit-plus validate`

**Checkpoint**: Chapters 05 and 06 content complete, with runnable examples, covering humanoid simulation.

---

## Phase 8: User Story 4 - Implementing Hardware-Accelerated Perception and Navigation (Priority: P2)

**Goal**: A reader can implement high-performance perception and navigation for bipedal robots using Isaac ROS and Nav2, deploying to real hardware. Covers Chapter 07.

**Independent Test**: Reader successfully runs VSLAM, AprilTag detection, and people detection at 30+ FPS on Jetson, implements Nav2 stack, and deploys perception pipeline to real RealSense + Jetson kit.

### Implementation for User Story 4

- [ ] T072 [P] [US4] Create chapter spec `specs/07-isaac-ros-perception.sp` for Chapter 07
- [ ] T073 [US4] Draft MDX content for Chapter 07 in `docs/07-isaac-ros-perception.mdx` (VSLAM, AprilTag, people detection at 30+ FPS on Jetson)
- [ ] T074 [US4] Create runnable example for Isaac ROS perception on Jetson in `examples/isaac-ros/jetson-perception/`
- [ ] T075 [US4] Draft MDX content for Chapter 07 in `docs/07-isaac-ros-perception.mdx` (Implement Nav2 stack for bipedal navigation)
- [ ] T076 [US4] Create runnable example for Nav2 bipedal navigation in `examples/isaac-ros/nav2-bipedal/`
- [ ] T077 [US4] Draft MDX content for Chapter 07 in `docs/07-isaac-ros-perception.mdx` (Deploy perception pipeline to real RealSense + Jetson kit)
- [ ] T078 [US4] Create runnable example for RealSense + Jetson deployment in `examples/isaac-ros/realsense-deployment/`
- [ ] T079 [US4] Validate `specs/07-isaac-ros-perception.sp` using `spec-kit-plus validate`

**Checkpoint**: Chapter 07 content complete, with runnable examples, covering hardware-accelerated perception and navigation.

---

## Phase 9: User Story 5 - Understanding Humanoid Kinematics and Locomotion (Priority: P2)

**Goal**: A reader can understand and implement advanced humanoid kinematics, balance control, and bipedal locomotion. Covers Chapter 08.

**Independent Test**: Reader successfully implements forward/inverse kinematics and runs a walking pattern generator.

### Implementation for User Story 5

- [ ] T080 [P] [US5] Create chapter spec `specs/08-humanoid-kinematics.sp` for Chapter 08
- [ ] T081 [US5] Draft MDX content for Chapter 08 in `docs/08-humanoid-kinematics.mdx` (Forward/inverse kinematics for 20+ DoF humanoids)
- [ ] T082 [US5] Create runnable example for humanoid kinematics in `examples/kinematics/humanoid-kinematics/`
- [ ] T083 [US5] Draft MDX content for Chapter 08 in `docs/08-humanoid-kinematics.mdx` (Zero-moment point (ZMP) and balance control)
- [ ] T084 [US5] Create runnable example for ZMP and balance control in `examples/kinematics/balance-control/`
- [ ] T085 [US5] Draft MDX content for Chapter 08 in `docs/08-humanoid-kinematics.mdx` (Walking pattern generators)
- [ ] T086 [US5] Create runnable example for walking pattern generators in `examples/kinematics/walking-patterns/`
- [ ] T087 [US5] Validate `specs/08-humanoid-kinematics.sp` using `spec-kit-plus validate`

**Checkpoint**: Chapter 08 content complete, with runnable examples, covering humanoid kinematics and locomotion.

---

## Phase 10: User Story 8 - Sim-to-Real Transfer (Priority: P3)

**Goal**: A reader can effectively transfer simulated models to real hardware, including best practices for domain randomization, latency, calibration, and safety. Covers Chapter 11.

**Independent Test**: Reader successfully applies domain randomization recipes and understands best practices for deploying models to a real humanoid.

### Implementation for User Story 8

- [ ] T088 [P] [US8] Create chapter spec `specs/11-sim-to-real.sp` for Chapter 11
- [ ] T089 [US8] Draft MDX content for Chapter 11 in `docs/11-sim-to-real.mdx` (Domain randomization recipes)
- [ ] T090 [US8] Create runnable example for domain randomization in `examples/sim-to-real/domain-randomization/`
- [ ] T091 [US8] Draft MDX content for Chapter 11 in `docs/11-sim-to-real.mdx` (Latency, calibration, and safety best practices)
- [ ] T092 [US8] Draft MDX content for Chapter 11 in `docs/11-sim-to-real.mdx` (Deploying capstone model to a real humanoid)
- [ ] T093 [US8] Create runnable example for real humanoid deployment in `examples/sim-to-real/real-hardware-deployment/`
- [ ] T094 [US8] Validate `specs/11-sim-to-real.sp` using `spec-kit-plus validate`

**Checkpoint**: Chapter 11 content complete, with runnable examples, covering sim-to-real transfer.

---

## Phase 11: User Story 9 - Contributing to the Book (Priority: P3)

**Goal**: A reader can understand how to contribute to the living book using the spec-first workflow. Covers Chapter 12.

**Independent Test**: Reader successfully adds a new chapter or example following the Spec-Kit Plus workflow.

### Implementation for User Story 9

- [ ] T095 [P] [US9] Create chapter spec `specs/12-contributing.sp` for Chapter 12
- [ ] T096 [US9] Draft MDX content for Chapter 12 in `docs/12-contributing.mdx` (Add new chapters, examples, or hardware tracks using Spec-Kit Plus)
- [ ] T097 [US9] Draft MDX content for Chapter 12 in `docs/12-contributing.mdx` (Full guide to fork → spec → PR → deployed book)
- [ ] T098 [US9] Validate `specs/12-contributing.sp` using `spec-kit-plus validate`

**Checkpoint**: Chapter 12 content complete, providing guidance for contributions.

---

## Phase 12: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements and cross-cutting tasks that affect multiple user stories or the overall book quality.

- [ ] T099 [P] Create `src/pages/capstone-landing.js` for capstone project overview
- [ ] T100 [P] Create `src/pages/hardware-chooser.js` for guiding readers on hardware track selection
- [ ] T101 [P] Draft MDX content for Chapter 01 in `docs/01-intro-why-physical-ai.mdx` (The Age of Embodied Intelligence)
- [ ] T102 [P] Create `specs/01-intro-why-physical-ai.sp` for Chapter 01
- [ ] T103 [P] Validate `specs/01-intro-why-physical-ai.sp`
- [ ] T104 [P] Create `static/img/cover-physical-ai.jpg` cover image as per book metadata
- [ ] T105 Create appendix content for A-hardware-boms in `docs/A-hardware-boms.mdx`
- [ ] T106 Create appendix content for B-jetson-flashing in `docs/B-jetson-flashing.mdx`
- [ ] T107 Create appendix content for C-docker-cheatsheet in `docs/C-docker-cheatsheet.mdx`
- [ ] T108 Create appendix content for D-troubleshooting in `docs/D-troubleshooting.mdx`
- [ ] T109 Create appendix content for E-glossary in `docs/E-glossary.mdx`
- [ ] T110 Review all chapters for citation style (APA 7th) and consistency with `/references/`
- [ ] T111 Conduct a comprehensive `spec-kit-plus validate --all` across the entire project
- [ ] T112 Run `markdownlint` across all `.mdx` and `.md` files
- [ ] T113 Perform a full `docusaurus build` and resolve any errors (<4 min target)
- [ ] T114 Execute link checker across entire built site
- [ ] T115 Run Lighthouse CI on deployed site and achieve ≥95 performance/accessibility/SEO
- [ ] T116 Run all `ros2 test examples/ros2/**` and `colcon test` in Docker for examples
- [ ] T117 Human acceptance validation: One reader completes capstone in ≤ 8 hours
- [ ] T118 Human acceptance validation: All 20+ examples run without modification
- [ ] T119 Human acceptance validation: Capstone simulation track runs on AWS g5.4xlarge spot instance (< $0.80/hour)
- [ ] T120 Human acceptance validation: Zero missing diagrams or dead videos at release
- [ ] T121 Write foreword for the book
- [ ] T122 Write `README.md` with one-click setup instructions
- [ ] T123 Tag v1.0.0 for initial release
- [ ] T124 Establish permanent GitHub Pages URL + DOI via Zenodo

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Setup (Phase 1)**: No dependencies - can start immediately.
-   **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories.
-   **User Stories (Phases 3-11)**: All depend on Foundational phase completion.
    -   User stories can then proceed in parallel (if staffed).
    -   Or sequentially in priority order (P1 → P2 → P3).
-   **Polish (Phase 12)**: Depends on all desired user stories being complete.

### User Story Dependencies

-   **User Story 1 (P1 - Setting up the AI Lab)**: Can start after Foundational (Phase 2). No dependencies on other stories for its core completion.
-   **User Story 2 (P1 - Mastering ROS 2 Fundamentals)**: Can start after Foundational (Phase 2). No dependencies on other stories for its core completion.
-   **User Story 6 (P1 - Integrating Vision-Language-Action Models)**: Can start after Foundational (Phase 2). Depends on prior knowledge from ROS 2 and possibly perception/kinematics.
-   **User Story 7 (P1 - Completing the Autonomous Humanoid Capstone)**: Can start after Foundational (Phase 2). Heavily depends on US1, US2, US3, US4, US5, US6 for a full E2E system. Can be built incrementally.
-   **User Story 3 (P2 - Simulating Humanoids)**: Can start after Foundational (Phase 2). No dependencies on other stories for its core completion.
-   **User Story 4 (P2 - Implementing Hardware-Accelerated Perception and Navigation)**: Can start after Foundational (Phase 2). Depends on US1 for hardware setup.
-   **User Story 5 (P2 - Understanding Humanoid Kinematics and Locomotion)**: Can start after Foundational (Phase 2). No direct story dependencies.
-   **User Story 8 (P3 - Sim-to-Real Transfer)**: Can start after Foundational (Phase 2). Depends on completion of a capstone project (US7) for content.
-   **User Story 9 (P3 - Contributing to the Book)**: Can start after Foundational (Phase 2). No direct story dependencies.

### Within Each User Story

-   Tests (if included) MUST be written and FAIL before implementation.
-   Models before services.
-   Services before endpoints.
-   Core implementation before integration.
-   Story complete before moving to next priority.

### Parallel Opportunities

-   All Setup tasks marked [P] can run in parallel.
-   All Foundational tasks marked [P] can run in parallel (within Phase 2).
-   Once Foundational phase completes, several user stories can start in parallel (if team capacity allows, e.g., US1, US2, US3, US5, US9 can be developed somewhat independently initially).
-   All tests for a user story marked [P] can run in parallel.
-   Different user stories can be worked on in parallel by different team members, especially those with fewer dependencies.

---

## Parallel Example: User Story 1

```bash
# Launch chapter spec creation and content drafting in parallel
Task: "Create chapter spec specs/02-hardware-lab-setup.sp for Chapter 02"
Task: "Draft MDX content for Chapter 02 in docs/02-hardware-lab-setup.mdx"

# Launch static asset creation in parallel
Task: "Create wiring SVGs for Economy Jetson Kit in hardware-kit/jetson-wiring.svg"
Task: "Create flashing guide content hardware-kit/jetpack-flashing-guide.md"
```

---

## Implementation Strategy

### MVP First (Prioritized User Stories)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3.  Complete Phase 3: User Story 1 (Setting up the AI Lab)
4.  Complete Phase 4: User Story 2 (Mastering ROS 2 Fundamentals)
5.  Complete Phase 5: User Story 6 (Integrating Vision-Language-Action Models)
6.  Complete Phase 6: User Story 7 (Completing the Autonomous Humanoid Capstone)
7.  **STOP and VALIDATE**: Test all P1 stories and the capstone independently.
8.  Deploy/demo if ready.

### Incremental Delivery

1.  Complete Setup + Foundational → Foundation ready.
2.  Add User Story 1 → Test independently → Deploy/Demo (Lab setup ready!).
3.  Add User Story 2 → Test independently → Deploy/Demo (ROS 2 ready!).
4.  Add User Story 6 → Test independently → Deploy/Demo (VLA ready!).
5.  Add User Story 7 → Test independently → Deploy/Demo (Capstone ready!).
6.  Then proceed with P2 and P3 stories incrementally.
7.  Each story adds value without breaking previous stories.

### Parallel Team Strategy

With multiple developers:

1.  Team completes Setup + Foundational together.
2.  Once Foundational is done:
    -   Developer A: User Story 1 (Lab Setup)
    -   Developer B: User Story 2 (ROS 2 Fundamentals)
    -   Developer C: User Story 3 (Simulating Humanoids)
    -   Developer D: User Story 5 (Humanoid Kinematics)
3.  Stories complete and integrate as dependencies are met.

---

## Notes

-   [P] tasks = different files, no dependencies.
-   [Story] label maps task to specific user story for traceability.
-   Each user story should be independently completable and testable.
-   Verify examples/tests run successfully.
-   Commit after each task or logical group.
-   Stop at any checkpoint to validate story independently.
-   Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence without clear rationale.
