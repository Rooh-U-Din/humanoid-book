# Feature Specification: Physical AI & Humanoid Robotics Docusaurus Book + Integrated RAG Chatbot

**Feature Branch**: `001-physical-ai-humanoid-book`
**Created**: 2025-12-05
**Status**: Draft
**Input**: User description: "Physical AI & Humanoid Robotics Docusaurus Book + Integrated RAG Chatbot - Produce a full Docusaurus-based textbook and technical guide for the course 'Physical AI & Humanoid Robotics,' including simulations (ROS 2, Gazebo, Unity, NVIDIA Isaac), Vision-Language-Action systems, conversational robotics, and a capstone humanoid robot project. The published book must integrate a functional RAG chatbot capable of answering questions strictly from book content or user-selected text."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Reader Accesses Book Content and Navigation (Priority: P1) 🎯 MVP

A robotics student visits the deployed Docusaurus book on GitHub Pages, navigates through chapters on ROS 2, Gazebo, Isaac Sim, and VLA systems, reads theory and code examples, and views Mermaid diagrams illustrating robot architectures and simulation pipelines.

**Why this priority**: This is the foundational value delivery - without readable, well-structured book content, nothing else matters. This establishes the core educational resource.

**Independent Test**: Can be fully tested by deploying Docusaurus to GitHub Pages with at least 3 sample chapters (e.g., ROS 2 Basics, Gazebo Simulation, VLA Overview) and verifying navigation, rendering, and diagram display work correctly on desktop and mobile.

**Acceptance Scenarios**:

1. **Given** the book is deployed to GitHub Pages, **When** a user navigates to the URL, **Then** the homepage loads with course overview, module structure, and navigation sidebar
2. **Given** the user is on the homepage, **When** they click on "Chapter 3: ROS 2 Workspace Setup", **Then** the chapter loads with theory, code blocks, and Mermaid diagrams
3. **Given** the user is reading a chapter, **When** they view a Mermaid diagram, **Then** the diagram renders correctly showing ROS 2 node architecture
4. **Given** the user is on mobile device, **When** they access any chapter, **Then** content is responsive and readable with working navigation menu

---

### User Story 2 - Reader Runs Code Examples Locally (Priority: P2)

A developer clones the book's GitHub repository, follows installation instructions for ROS 2 Humble on Ubuntu 22.04, sets up the workspace, and successfully runs example ROS 2 nodes, Gazebo simulations, and Isaac Sim scripts referenced in the book chapters.

**Why this priority**: Runnable code examples are critical for hands-on learning. This validates the book's technical accuracy and reproducibility claims.

**Independent Test**: Provide a setup script and 3 standalone code examples (ROS 2 publisher/subscriber, Gazebo world launch, Isaac Sim basic scene) that can be tested on a clean Ubuntu 22.04 VM following only the book's instructions.

**Acceptance Scenarios**:

1. **Given** a clean Ubuntu 22.04 installation, **When** user follows Chapter 2 ROS 2 installation guide, **Then** `ros2 --version` returns Humble release
2. **Given** ROS 2 is installed, **When** user runs `ros2 run examples talker_node`, **Then** node publishes messages visible via `ros2 topic echo`
3. **Given** Gazebo is installed per Chapter 4, **When** user runs `ros2 launch examples warehouse_sim.launch.py`, **Then** Gazebo opens with warehouse environment and robot model
4. **Given** Isaac Sim is installed per Chapter 6, **When** user runs `python examples/isaac_basic_scene.py`, **Then** Isaac Sim opens with simulated robot and camera feed

---

### User Story 3 - Reader Queries RAG Chatbot (Full-Book Mode) (Priority: P3)

A student reading the book encounters a concept they want clarified (e.g., "What is the difference between SLAM and VSLAM?"), opens the embedded chatbot interface, types the question, and receives an accurate answer generated from book content with citations to relevant chapter sections.

**Why this priority**: The RAG chatbot provides immediate, context-aware assistance without leaving the book interface, enhancing the learning experience significantly.

**Independent Test**: Deploy the book with chatbot UI embedded, send 5 predefined questions via the chatbot (e.g., "How do I install ROS 2?", "What hardware do I need for Isaac Sim?"), and verify responses cite correct chapter sections and contain accurate information from book content only.

**Acceptance Scenarios**:

1. **Given** the book is open in a browser, **When** user clicks the chatbot icon in the bottom-right corner, **Then** chatbot interface opens with input field
2. **Given** the chatbot is open, **When** user types "What is the difference between SLAM and VSLAM?" and presses Enter, **Then** chatbot responds within 5 seconds with answer citing Chapter 8
3. **Given** the chatbot response is displayed, **When** user clicks a chapter citation link, **Then** browser navigates to the cited chapter section
4. **Given** the chatbot has answered a question, **When** user asks a follow-up question ("Can you explain EKF-SLAM?"), **Then** chatbot maintains context and provides relevant answer from Chapter 8

---

### User Story 4 - Reader Uses Selection-Based RAG Mode (Priority: P4)

A reader highlights a specific paragraph about "Isaac Sim synthetic data generation" in Chapter 6, right-clicks to select "Ask chatbot about this selection", types a question ("How do I export this data to ROS 2?"), and receives an answer strictly based on the selected text and closely related book sections.

**Why this priority**: This advanced RAG feature allows targeted, context-specific queries that improve precision and reduce hallucination risk by constraining the retrieval scope.

**Independent Test**: Highlight a 200-word section on Isaac Sim in Chapter 6, use the selection-based query mode to ask 3 questions (e.g., "What file formats are supported?", "Can this run on Jetson?"), and verify answers reference only the selected section and immediate context.

**Acceptance Scenarios**:

1. **Given** user is reading Chapter 6, **When** they highlight text about "synthetic data generation", **Then** a context menu appears with "Ask about selection" option
2. **Given** user selects "Ask about selection", **When** chatbot opens, **Then** the selected text is shown in the chat context pane
3. **Given** the selected text is loaded, **When** user asks "What file formats are supported?", **Then** chatbot answers using only the selected text and retrieves related content from the same chapter section
4. **Given** user asks a question outside selection scope, **When** chatbot cannot answer from selection, **Then** chatbot responds "This information is not covered in the selected text. Would you like to search the full book?"

---

### User Story 5 - Instructor Deploys Full Capstone Project (Priority: P5)

An instructor teaching the Physical AI course follows Chapter 12's capstone project guide to set up the complete voice-command humanoid robot pipeline: Whisper speech recognition → LLM action planner → ROS 2 navigation → Isaac Sim training → Jetson Orin deployment, validates each stage works end-to-end, and demonstrates to students.

**Why this priority**: The capstone project is the culminating learning experience that integrates all course modules. While lower priority for initial book release, it's essential for course completeness.

**Independent Test**: Follow Chapter 12 step-by-step with recommended hardware (RTX 3080 workstation + Jetson Orin kit + RealSense camera), complete all 8 pipeline stages (voice → plan → navigate → manipulate → deploy), and verify the robot responds to voice commands in both simulation and edge deployment.

**Acceptance Scenarios**:

1. **Given** instructor has Chapter 12 hardware setup, **When** they run the Whisper integration test, **Then** voice command "Go to the kitchen" is transcribed correctly
2. **Given** Whisper output is working, **When** instructor runs LLM planner with sample command, **Then** planner generates valid ROS 2 navigation goal coordinates
3. **Given** LLM planner works, **When** instructor launches Isaac Sim training environment, **Then** robot navigates to goal using trained policy in simulation
4. **Given** simulation works, **When** instructor deploys ROS 2 stack to Jetson Orin, **Then** robot navigates to goal using onboard sensors (RealSense depth + IMU)
5. **Given** full pipeline is deployed, **When** instructor issues voice command "Pick up the red cube", **Then** robot navigates, detects cube via YOLO, and executes grasp trajectory

---

### User Story 6 - Backend API Developer Tests RAG Endpoints (Priority: P6)

A developer contributing to the RAG backend starts the FastAPI server locally, sends test queries to `/api/query` (full-book mode) and `/api/query-selection` (selection mode) endpoints via Postman, verifies embeddings are retrieved from Qdrant, responses are generated via OpenAI Agents, and query logs are stored in Neon Postgres.

**Why this priority**: Backend API reliability is crucial for chatbot functionality, but can be tested independently of the book frontend. Lower priority for initial user-facing value.

**Independent Test**: Deploy FastAPI backend locally with test Qdrant collection (50 sample book chunks), send 10 API requests with various queries, and verify: 200 OK responses, JSON contains `answer` and `citations` fields, Qdrant logs show vector searches, Postgres logs show query records.

**Acceptance Scenarios**:

1. **Given** FastAPI server is running on `localhost:8000`, **When** developer sends `POST /api/query` with `{"query": "What is ROS 2?"}`, **Then** response returns JSON with `answer` field containing book-based answer and `citations` array with chapter references
2. **Given** API responds to queries, **When** developer checks Qdrant dashboard, **Then** vector search logs show 5-10 chunks retrieved for each query
3. **Given** vector retrieval works, **When** developer queries Neon Postgres `query_logs` table, **Then** all queries are logged with timestamp, query text, and response metadata
4. **Given** selection mode endpoint exists, **When** developer sends `POST /api/query-selection` with `{"selected_text": "...", "query": "..."}`, **Then** response prioritizes selected text context over full-book search

---

### Edge Cases

- **What happens when a user asks a question completely outside the book scope** (e.g., "What is the capital of France?")? → Chatbot responds: "This question is outside the scope of the Physical AI & Humanoid Robotics book. I can only answer questions based on book content."

- **What happens when the RAG system retrieves no relevant chunks** (e.g., query uses terminology not in the book)? → Chatbot responds: "I couldn't find information about this topic in the book. Please try rephrasing your question or check the table of contents for related chapters."

- **What happens when a code example fails to run due to version mismatch** (e.g., ROS 2 Humble vs. Foxy)? → Book includes troubleshooting section with version compatibility table and common error solutions.

- **What happens when a user tries to run Isaac Sim examples without an RTX GPU?** → Chapter 6 includes hardware requirements section warning about GPU dependencies and suggests cloud alternatives (AWS g5 instances).

- **What happens when Qdrant Cloud free tier limit is reached** (100k vectors)? → Backend includes monitoring dashboard showing vector count, alerts when approaching 80% capacity, and documentation on pruning old embeddings.

- **What happens when a user highlights 5000 words of text for selection-based query?** → Chatbot truncates selection to max 1000 words and shows warning: "Selection too large (5000 words). Using first 1000 words. Please select a smaller section for more precise answers."

- **What happens when multiple users query the chatbot simultaneously?** → FastAPI backend handles async requests; if OpenAI rate limit is hit, backend returns 429 status with retry-after header.

- **What happens when book is updated with new chapters but embeddings are not regenerated?** → Deployment pipeline includes post-build hook that detects content changes, triggers embedding regeneration script, and updates Qdrant collection before deploying updated book.

## Requirements *(mandatory)*

### Functional Requirements

#### Book Content Requirements

- **FR-001**: Book MUST include minimum 12 chapters covering: (1) Course Overview, (2) ROS 2 Fundamentals, (3) ROS 2 Workspace Setup, (4) Gazebo Simulation, (5) Unity Visualization Basics, (6) NVIDIA Isaac Sim, (7) Isaac Sim Training Workflows, (8) VSLAM & Navigation, (9) Vision-Language-Action Systems, (10) Conversational Robotics, (11) Edge Deployment (Jetson Orin), (12) Capstone Humanoid Robot Project

- **FR-002**: Each chapter MUST include: (a) theory section explaining core concepts, (b) at least one architecture/workflow diagram in Mermaid format, (c) at least two runnable code examples with full context, (d) summary section with key takeaways

- **FR-003**: Book MUST include dedicated hardware requirements sections specifying: (a) Digital Twin Setup (workstation with RTX GPU, 32GB RAM, Ubuntu 22.04), (b) Edge AI Kit (Jetson Orin, RealSense D435i, IMU, USB microphone), (c) optional Robot Lab equipment (Unitree Go2/G1, OP3, or Hiwonder humanoid), (d) cloud-native alternative (AWS g5/g6 instance specifications)

- **FR-004**: All code examples MUST be tested and runnable on Ubuntu 22.04 with ROS 2 Humble and include: (a) prerequisite dependencies listed, (b) installation commands, (c) expected output/behavior described, (d) troubleshooting section for common errors

- **FR-005**: Book MUST include complete capstone project walkthrough (Chapter 12) covering: (a) voice input pipeline (Whisper integration), (b) LLM-based action planning, (c) ROS 2 navigation stack configuration, (d) Isaac Sim training environment setup, (e) perception pipeline (YOLO object detection + depth fusion), (f) manipulation planning (MoveIt integration), (g) Jetson Orin deployment procedure, (h) end-to-end testing protocol

#### RAG Chatbot Requirements

- **FR-006**: Book MUST embed a chatbot UI component accessible via persistent bottom-right icon on all pages that: (a) opens chat interface on click, (b) maintains chat history within session, (c) provides "Clear chat" option, (d) shows loading state during query processing

- **FR-007**: Chatbot MUST support full-book query mode where: (a) user types natural language question, (b) backend retrieves relevant chunks from Qdrant vector database, (c) OpenAI Agents/ChatKit generates answer using only retrieved book content, (d) response includes citations with clickable links to source chapters

- **FR-008**: Chatbot MUST support selection-based query mode where: (a) user highlights text in book, (b) context menu offers "Ask about selection" option, (c) chatbot interface opens with selected text shown in context pane, (d) queries prioritize selected text and immediate surrounding context, (e) chatbot warns if answer cannot be found in selection

- **FR-009**: Chatbot MUST enforce book-content-only constraint: (a) responses reference exclusively embedded book content, (b) chatbot declines questions outside book scope with polite message, (c) chatbot does not generate speculative or external knowledge

- **FR-010**: Backend API MUST provide RESTful endpoints: (a) `POST /api/query` for full-book queries accepting `{query: string}` and returning `{answer: string, citations: Citation[]}`, (b) `POST /api/query-selection` for selection-based queries accepting `{selected_text: string, query: string, chapter_context: string}` and returning same format, (c) `GET /api/health` for service monitoring

#### Data & Infrastructure Requirements

- **FR-011**: Book content MUST be chunked and embedded into Qdrant Cloud Free Tier with: (a) chunk size of 500-1000 tokens with 100-token overlap, (b) embeddings generated using OpenAI text-embedding-3-small model, (c) metadata including chapter ID, section title, and page URL stored with each vector

- **FR-012**: Neon Serverless Postgres MUST store: (a) query_logs table with columns: id, timestamp, query_text, response_text, mode (full-book or selection), latency_ms, user_session_id, (b) chat_sessions table with columns: id, session_id, created_at, last_activity, (c) embeddings_metadata table tracking embedding generation timestamps and chunk counts per chapter

- **FR-013**: FastAPI backend MUST implement: (a) async endpoint handlers to avoid blocking, (b) structured logging (JSON format) with request ID tracing, (c) error handling with appropriate HTTP status codes (400 for bad requests, 429 for rate limits, 500 for server errors), (d) CORS configuration allowing requests from GitHub Pages domain

#### Deployment Requirements

- **FR-014**: Docusaurus book MUST build successfully with zero errors and deploy to GitHub Pages via GitHub Actions workflow triggered on push to main branch

- **FR-015**: Backend FastAPI service MUST be deployable to: (a) local development environment via `uvicorn` with hot reload, (b) production environment (e.g., Render, Railway, or AWS Lambda) with environment variable configuration for API keys and database URLs

- **FR-016**: Repository MUST include: (a) README.md with one-click setup instructions for book and backend, (b) .env.example template listing all required environment variables (OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL), (c) DEPLOYMENT.md with step-by-step deployment guide for GitHub Pages, FastAPI hosting, Qdrant setup, and Neon configuration

### Key Entities

- **Chapter**: Represents a book chapter with attributes: id (unique identifier), title, order (sequence number), markdown_content (MDX source), url_path (deployed page URL), last_updated (timestamp)

- **BookChunk**: Represents an embedded text chunk with attributes: id, chapter_id (foreign key), chunk_index (position within chapter), text_content (raw text), embedding_vector (1536-dimensional float array for OpenAI embeddings), metadata (JSON with chapter title, section heading, url_fragment)

- **QueryLog**: Represents a chatbot query record with attributes: id, timestamp, query_text, selected_text (nullable, only for selection mode), response_answer, citations (JSON array of {chapter_id, section_title, url}), mode (enum: full-book | selection), latency_ms, error_message (nullable)

- **ChatSession**: Represents a user chat session with attributes: id, session_id (UUID), created_at, last_activity, message_count, user_agent (browser info for analytics)

- **CodeExample**: Represents a runnable code example with attributes: id, chapter_id, example_name, language (python | bash | yaml), code_content, prerequisites (array of package names), expected_output, file_path (location in /examples directory)

- **HardwareSpec**: Represents hardware requirement specifications with attributes: id, setup_type (enum: digital-twin | edge-kit | robot-lab | cloud), component_name, min_specs (JSON with CPU/GPU/RAM/storage), recommended_specs (JSON), vendor_links (array of URLs)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Docusaurus book builds and deploys to GitHub Pages with zero build errors and loads successfully on desktop and mobile browsers within 3 seconds

- **SC-002**: Book includes minimum 12 complete chapters (150-200 pages equivalent) with 100% coverage of required modules: ROS 2, Gazebo, Unity, Isaac Sim, VLA, Capstone

- **SC-003**: All code examples (minimum 30 across all chapters) run successfully on clean Ubuntu 22.04 + ROS 2 Humble installation following only book instructions, with 95%+ success rate in test runs

- **SC-004**: RAG chatbot responds to queries within 5 seconds (p95 latency) and provides answers with correct chapter citations for 90%+ of in-scope questions in test dataset (100 curated questions)

- **SC-005**: Selection-based chatbot mode correctly constrains answers to selected text context for 85%+ of test cases (50 curated selection-based queries)

- **SC-006**: Book achieves Lighthouse score ≥90 for performance, accessibility, best practices, and SEO on production GitHub Pages deployment

- **SC-007**: Backend API passes functional testing with 100% success rate for: (a) 20 full-book query tests, (b) 20 selection-based query tests, (c) error handling tests (invalid input, rate limiting, empty results)

- **SC-008**: All Mermaid diagrams (minimum 15 across chapters) render correctly without errors on supported browsers (Chrome, Firefox, Safari, Edge)

- **SC-009**: Repository README setup instructions enable a developer to run local book instance + backend API in under 30 minutes on Ubuntu 22.04

- **SC-010**: Zero broken links in deployed book verified by automated link checker in CI/CD pipeline

- **SC-011**: Capstone project (Chapter 12) can be completed end-to-end by instructor with recommended hardware setup, validated by at least one successful test run per pipeline stage

- **SC-012**: Book content passes technical accuracy review by at least one domain expert in robotics/ROS 2, with zero critical errors and fewer than 10 minor corrections needed
