---
id: 001
title: Create Specification for Physical AI & Humanoid Robotics Book
stage: spec
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-humanoid-robotics
branch: master
user: unknown
command: /sp.spcify /sp.toc Physical AI & Humanoid Robotics – From Digital Brain to Embodied Intelligence

Book metadata:
  title: "Physical AI & Humanoid Robotics: Building Embodied Intelligence with ROS 2, NVIDIA Isaac, and Vision-Language-Action Models"
  subtitle: "A 13-Week Capstone Course + Complete Lab Manual – From $700 Jetson Kit to Autonomous Conversational Humanoids"
  authors: "Your Name + Open Physical AI Community"
  license: MIT
  cover_image: /static/img/cover-physical-ai.jpg

Target reader journey:
- Week 0: Set up workstation + Economy Jetson Kit
- Weeks 1–13: Follow exactly one chapter per week
- End of Week 13: Fully working autonomous humanoid (simulated + optional real hardware deployment)

Table of Contents (exact 1-chapter-per-week cadence)

01-intro-why-physical-ai
  title: "Chapter 01 – The Age of Embodied Intelligence"
  weeks: 1–2
  outcomes:
    - Explain why humanoids are the ultimate general-purpose robots
    - Describe the three pillars: perception → cognition → action in the physical world
    - Choose your hardware track ($700 / $3k / premium)

02-hardware-lab-setup
  title: "Chapter 02 – Building Your Physical AI Lab (Three Budget Tracks)"
  outcomes:
    - Assemble the $700 Economy Jetson Student Kit (full wiring + flashing guide)
    - Set up the RTX workstation (Ubuntu 22.04 + drivers + Docker)
    - Cloud fallback path (AWS g5/g6 + cost calculator)

03-ros2-fundamentals
  title: "Chapter 03 – ROS 2: The Robotic Nervous System (Part 1)"
  weeks: 3–4
  outcomes:
    - Create nodes, topics, services, actions with rclpy
    - Build and launch your first ROS 2 package
    - Understand URDF/Xacro for humanoid description

04-ros2-advanced
  title: "Chapter 04 – ROS 2: Parameters, Launch Files, and Real-Time Control"
  week: 5
  outcomes:
    - Master complex launch files and parameter servers
    - Bridge Python AI agents to low-level controllers
    - Run your first teleop controller on Jetson

05-gazebo-basics
  title: "Chapter 05 – Digital Twins: Gazebo Simulation"
  weeks: 6–7
  outcomes:
    - Build worlds and spawn humanoid URDFs
    - Simulate LiDAR, depth cameras, IMUs
    - Add plugins for realistic physics and sensors

06-isaac-sim-intro
  title: "Chapter 06 – NVIDIA Isaac Sim: Photorealistic Simulation & Synthetic Data"
  weeks: 8–9
  outcomes:
    - Run Isaac Sim on RTX workstation
    - Import humanoid USD assets
    - Generate domain-randomized training data

07-isaac-ros-perception
  title: "Chapter 07 – Isaac ROS: Hardware-Accelerated Perception & Nav2"
  week: 10
  outcomes:
    - Run VSLAM, AprilTag detection, and people detection at 30+ FPS on Jetson
    - Implement Nav2 stack for bipedal navigation
    - Deploy perception pipeline to real RealSense + Jetson kit

08-humanoid-kinematics
  title: "Chapter 08 – Humanoid Kinematics, Balance, and Bipedal Locomotion"
  weeks: 11–12
  outcomes:
    - Forward/inverse kinematics for 20+ DoF humanoids
    - Zero-moment point (ZMP) and balance control
    - Walking pattern generators you can actually run

09-vla-fundamentals
  title: "Chapter 09 – Vision-Language-Action Models: From Voice to Robot Behavior"
  week: 13 (part 1)
  outcomes:
    - Whisper → LLM → ROS 2 action pipeline
    - Turn “Pick up the red cup” into a sequence of navigation + manipulation goals
    - Run everything locally (no OpenAI API required after Week 9)

10-capstone-project
  title: "Chapter 10 – Capstone: The Autonomous Conversational Humanoid"
  week: 13 (part 2)
  outcomes:
    - Full end-to-end project (voice command → path planning → object detection → grasp)
    - Two tracks: 100% simulation OR real Jetson + RealSense + (optional) Unitree Go2/G1
    - Complete repo you can clone and run in <10 minutes

11-sim-to-real
  title: "Chapter 11 – Sim-to-Real Transfer Cookbook"
  outcomes:
    - Domain randomization recipes that actually work
    - Latency, calibration, and safety best practices
    - Deploying your capstone model to a real humanoid

12-contributing
  title: "Chapter 12 – Contribute to This Living Book (Spec-First Workflow)"
  outcomes:
    - Add new chapters, examples, or hardware tracks using Spec-Kit Plus
    - Full guide to fork → spec → PR → deployed book in <1 hour

Appendices (reference, not weekly reading)
A-hardware-boms       → All three budget tracks with 2025 pricing and links
B-jetson-flashing     → Step-by-step JetPack 6.0 installation
C-docker-cheatsheet   → One-command ROS 2 + Isaac Sim environments
D-troubleshooting     → The 50 most common “it doesn’t work” issues and fixes
E-glossary            → ROS 2, Isaac, VLA terminology demystified

Success criteria for the final published book:
- Every chapter has its own /specs/chapter-XX.sp file
- At least 20 fully runnable GitHub repositories embedded as examples
- Capstone project works in <15 minutes on fresh Ubuntu 22.04 + RTX 4070 Ti + Jetson kit
- Entire book remains free, open-source, and deployable by anyone with the $700 kit
- Lighthouse score ≥ 95 on GitHub Pages deployment
labels: ["specification", "book-content"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - specs/physical-ai-humanoid-robotics/spec.md
tests:
  null
---

## Prompt

```text
/sp.spcify /sp.toc Physical AI & Humanoid Robotics – From Digital Brain to Embodied Intelligence

Book metadata:
  title: "Physical AI & Humanoid Robotics: Building Embodied Intelligence with ROS 2, NVIDIA Isaac, and Vision-Language-Action Models"
  subtitle: "A 13-Week Capstone Course + Complete Lab Manual – From $700 Jetson Kit to Autonomous Conversational Humanoids"
  authors: "Your Name + Open Physical AI Community"
  license: MIT
  cover_image: /static/img/cover-physical-ai.jpg"

Target reader journey:
- Week 0: Set up workstation + Economy Jetson Kit
- Weeks 1–13: Follow exactly one chapter per week
- End of Week 13: Fully working autonomous humanoid (simulated + optional real hardware deployment)

Table of Contents (exact 1-chapter-per-week cadence)

01-intro-why-physical-ai
  title: "Chapter 01 – The Age of Embodied Intelligence"
  weeks: 1–2
  outcomes:
    - Explain why humanoids are the ultimate general-purpose robots
    - Describe the three pillars: perception → cognition → action in the physical world
    - Choose your hardware track ($700 / $3k / premium)

02-hardware-lab-setup
  title: "Chapter 02 – Building Your Physical AI Lab (Three Budget Tracks)"
  outcomes:
    - Assemble the $700 Economy Jetson Student Kit (full wiring + flashing guide)
    - Set up the RTX workstation (Ubuntu 22.04 + drivers + Docker)
    - Cloud fallback path (AWS g5/g6 + cost calculator)

03-ros2-fundamentals
  title: "Chapter 03 – ROS 2: The Robotic Nervous System (Part 1)"
  weeks: 3–4
  outcomes:
    - Create nodes, topics, services, actions with rclpy
    - Build and launch your first ROS 2 package
    - Understand URDF/Xacro for humanoid description

04-ros2-advanced
  title: "Chapter 04 – ROS 2: Parameters, Launch Files, and Real-Time Control"
  week: 5
  outcomes:
    - Master complex launch files and parameter servers
    - Bridge Python AI agents to low-level controllers
    - Run your first teleop controller on Jetson

05-gazebo-basics
  title: "Chapter 05 – Digital Twins: Gazebo Simulation"
  weeks: 6–7
  outcomes:
    - Build worlds and spawn humanoid URDFs
    - Simulate LiDAR, depth cameras, IMUs
    - Add plugins for realistic physics and sensors

06-isaac-sim-intro
  title: "Chapter 06 – NVIDIA Isaac Sim: Photorealistic Simulation & Synthetic Data"
  weeks: 8–9
  outcomes:
    - Run Isaac Sim on RTX workstation
    - Import humanoid USD assets
    - Generate domain-randomized training data

07-isaac-ros-perception
  title: "Chapter 07 – Isaac ROS: Hardware-Accelerated Perception & Nav2"
  week: 10
  outcomes:
    - Run VSLAM, AprilTag detection, and people detection at 30+ FPS on Jetson
    - Implement Nav2 stack for bipedal navigation
    - Deploy perception pipeline to real RealSense + Jetson kit

08-humanoid-kinematics
  title: "Chapter 08 – Humanoid Kinematics, Balance, and Bipedal Locomotion"
  weeks: 11–12
  outcomes:
    - Forward/inverse kinematics for 20+ DoF humanoids
    - Zero-moment point (ZMP) and balance control
    - Walking pattern generators you can actually run

09-vla-fundamentals
  title: "Chapter 09 – Vision-Language-Action Models: From Voice to Robot Behavior"
  week: 13 (part 1)
  outcomes:
    - Whisper → LLM → ROS 2 action pipeline
    - Turn “Pick up the red cup” into a sequence of navigation + manipulation goals
    - Run everything locally (no OpenAI API required after Week 9)

10-capstone-project
  title: "Chapter 10 – Capstone: The Autonomous Conversational Humanoid"
  week: 13 (part 2)
  outcomes:
    - Full end-to-end project (voice command → path planning → object detection → grasp)
    - Two tracks: 100% simulation OR real Jetson + RealSense + (optional) Unitree Go2/G1
    - Complete repo you can clone and run in <10 minutes

11-sim-to-real
  title: "Chapter 11 – Sim-to-Real Transfer Cookbook"
  outcomes:
    - Domain randomization recipes that actually work
    - Latency, calibration, and safety best practices
    - Deploying your capstone model to a real humanoid

12-contributing
  title: "Chapter 12 – Contribute to This Living Book (Spec-First Workflow)"
  outcomes:
    - Add new chapters, examples, or hardware tracks using Spec-Kit Plus
    - Full guide to fork → spec → PR → deployed book in <1 hour

Appendices (reference, not weekly reading)
A-hardware-boms       → All three budget tracks with 2025 pricing and links
B-jetson-flashing     → Step-by-step JetPack 6.0 installation
C-docker-cheatsheet   → One-command ROS 2 + Isaac Sim environments
D-troubleshooting     → The 50 most common “it doesn’t work” issues and fixes
E-glossary            → ROS 2, Isaac, VLA terminology demystified

Success criteria for the final published book:
- Every chapter has its own /specs/chapter-XX.sp file
- At least 20 fully runnable GitHub repositories embedded as examples
- Capstone project works in <15 minutes on fresh Ubuntu 22.04 + RTX 4070 Ti + Jetson kit
- Entire book remains free, open-source, and deployable by anyone with the $700 kit
- Lighthouse score ≥ 95 on GitHub Pages deployment
```

## Response snapshot

The specification file `specs/physical-ai-humanoid-robotics/spec.md` has been successfully created. This document outlines the user scenarios, functional requirements, key entities, and measurable success criteria for the "Physical AI & Humanoid Robotics" book, based on the detailed input provided by the user. It structures the book's content, development process, and target reader journey into a formal specification.

## Outcome

- ✅ Impact: Specification for Physical AI & Humanoid Robotics Book created.
- 🧪 Tests: No automated tests for spec creation.
- 📁 Files: Created spec file.
- 🔁 Next prompts: Pending further user instructions or creation of other Spec-Kit Plus artifacts.
- 🧠 Reflection: Successfully created the feature specification based on user input, detailing user scenarios, requirements, and success criteria for the book.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): None
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): None
