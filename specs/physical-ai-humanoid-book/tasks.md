# Tasks: Physical AI & Humanoid Robotics Docusaurus Book + Integrated RAG Chatbot

**Input**: Design documents from `/specs/physical-ai-humanoid-book/`
**Prerequisites**: plan.md (complete), spec.md (complete), research.md (pending), data-model.md (pending), contracts/ (pending)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4, US5, US6)
- Include exact file paths in descriptions

## Path Conventions

- **Book Content**: `docs/` (MDX chapters)
- **Backend**: `backend/src/` (FastAPI API, models, services)
- **Frontend Components**: `src/components/` (React chatbot widget)
- **Code Examples**: `examples/` (ROS 2, Gazebo, Isaac Sim)
- **Specifications**: `specs/physical-ai-humanoid-book/` (artifacts)
- **CI/CD**: `.github/workflows/` (GitHub Actions)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Initialize Docusaurus 3 project in repository root with TypeScript preset
- [ ] T002 [P] Initialize FastAPI backend structure in backend/src/ with models/, services/, api/ directories
- [ ] T003 [P] Create .env.example in repository root with OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL placeholders
- [ ] T004 [P] Create backend/requirements.txt with fastapi, uvicorn, pydantic, sqlalchemy, qdrant-client, psycopg2-binary, openai dependencies
- [ ] T005 [P] Create package.json in repository root with Docusaurus 3 dependencies and Mermaid plugin
- [ ] T006 Setup Git repository structure with docs/, backend/, src/components/, examples/ directories
- [ ] T007 [P] Create .gitignore with .env, node_modules/, __pycache__/, .DS_Store, build/ entries
- [ ] T008 [P] Initialize backend/tests/ directory with contract/, integration/, unit/ subdirectories

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Database & Infrastructure Setup

- [ ] T009 Create Neon Postgres schema in backend/contracts/database-schema.sql with query_logs table (id, timestamp, query_text, response_text, mode, latency_ms, user_session_id, selected_text, citations_json)
- [ ] T010 [P] Add chat_sessions table to backend/contracts/database-schema.sql (id, session_id UUID, created_at, last_activity, message_count, user_agent)
- [ ] T011 [P] Add embeddings_metadata table to backend/contracts/database-schema.sql (id, chapter_id, chunk_count, last_generated, embedding_model, total_tokens)
- [ ] T012 Setup Qdrant Cloud Free Tier account and create collection "book-embeddings" with 1536 dimensions, Cosine similarity
- [ ] T013 Create Qdrant collection config in backend/contracts/qdrant-collection.json with vector dimensions, distance metric, metadata payload schema
- [ ] T014 Create Neon Serverless Postgres database and run schema migration from backend/contracts/database-schema.sql

### Backend Foundation

- [ ] T015 Implement Pydantic models in backend/src/models/query.py (QueryRequest, QueryResponse, Citation, SelectionQueryRequest)
- [ ] T016 [P] Implement SQLAlchemy models in backend/src/models/database.py (QueryLog, ChatSession, EmbeddingsMetadata)
- [ ] T017 [P] Implement Qdrant models in backend/src/models/embeddings.py (BookChunk, ChunkMetadata)
- [ ] T018 Setup FastAPI app in backend/src/main.py with CORS middleware (allow GitHub Pages domain), rate limiting middleware (10 queries/min per session_id), structured logging (JSON format with request_id)
- [ ] T019 [P] Create database connection manager in backend/src/services/database_service.py with SQLAlchemy async session handling
- [ ] T020 [P] Create Qdrant client wrapper in backend/src/services/qdrant_service.py with connection pooling and error handling
- [ ] T021 [P] Create OpenAI client wrapper in backend/src/services/openai_service.py with retry logic and rate limit handling

### Docusaurus Configuration

- [ ] T022 Configure Docusaurus in docusaurus.config.js with title "Physical AI & Humanoid Robotics", GitHub Pages baseUrl, custom theme configuration
- [ ] T023 [P] Enable Mermaid plugin in docusaurus.config.js with theme support (light/dark mode)
- [ ] T024 [P] Configure sidebar structure in sidebars.js with 12 chapter categories (Course Overview, ROS 2, Gazebo, Unity, Isaac Sim, VLA, Conversational, Capstone, Hardware)
- [ ] T025 [P] Create custom CSS theme in src/css/custom.css with responsive typography, code block styling, diagram containers
- [ ] T026 Setup navbar in docusaurus.config.js with GitHub repository link, chatbot toggle button

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Reader Accesses Book Content and Navigation (Priority: P1) 🎯 MVP

**Goal**: Deploy Docusaurus book with 3+ sample chapters, Mermaid diagrams, responsive navigation, and GitHub Pages hosting

**Independent Test**: Book loads at GitHub Pages URL, homepage displays course overview, navigation sidebar works, 3+ chapters render with Mermaid diagrams, mobile responsive layout confirmed on viewport <768px width, Lighthouse score ≥90 for performance

### Content Creation

- [ ] T027 [US1] Write Chapter 1 Course Overview in docs/intro.md with learning objectives, prerequisites (Python, Linux basics, basic robotics concepts), course structure, hardware requirements overview, expected outcomes
- [ ] T028 [US1] Write Chapter 2 ROS 2 Fundamentals in docs/modules/ros2/fundamentals.mdx with ROS 2 architecture explanation, nodes/topics/services concepts, comparison with ROS 1, Mermaid diagram showing node communication architecture
- [ ] T029 [US1] Write Chapter 4 Gazebo Simulation in docs/modules/gazebo/intro.mdx with Gazebo architecture, world files, robot models (URDF/SDF), sensor simulation, Mermaid diagram showing Gazebo component interaction

### Diagrams

- [ ] T030 [P] [US1] Create Mermaid diagram for ROS 2 node architecture in docs/modules/ros2/fundamentals.mdx (graph showing publisher → topic → subscriber, service client/server)
- [ ] T031 [P] [US1] Create Mermaid diagram for Gazebo simulation pipeline in docs/modules/gazebo/intro.mdx (flowchart: world file → physics engine → sensor simulation → ROS 2 interface)
- [ ] T032 [P] [US1] Create Mermaid diagram for course module dependencies in docs/intro.md (graph showing prerequisite relationships: ROS 2 → Gazebo → Isaac → VLA → Capstone)

### Deployment

- [ ] T033 [US1] Create GitHub Actions workflow in .github/workflows/deploy-book.yml with Node.js 18 setup, Docusaurus build, GitHub Pages deployment on push to main branch
- [ ] T034 [US1] Configure GitHub repository Settings → Pages to use GitHub Actions source
- [ ] T035 [US1] Run initial deployment and verify book loads at GitHub Pages URL
- [ ] T036 [US1] Run Lighthouse audit on deployed book and verify Performance/Accessibility/Best Practices/SEO scores ≥90

**Checkpoint**: At this point, User Story 1 should be fully functional - book accessible online with navigation and diagrams rendering correctly

---

## Phase 4: User Story 2 - Reader Runs Code Examples Locally (Priority: P2)

**Goal**: Provide 3+ ROS 2/Gazebo/Isaac Sim examples runnable on Ubuntu 22.04 with complete setup instructions

**Independent Test**: Spin up clean Ubuntu 22.04 VM, follow book installation instructions, successfully run talker/listener ROS 2 example, launch Gazebo warehouse simulation, execute Isaac Sim basic scene (if GPU available), verify ≥95% success rate across 5 test runs

### ROS 2 Examples

- [ ] T037 [P] [US2] Create ROS 2 talker node in examples/ros2/talker_listener/talker.py with publisher publishing String messages at 1Hz
- [ ] T038 [P] [US2] Create ROS 2 listener node in examples/ros2/talker_listener/listener.py with subscriber logging received messages
- [ ] T039 [US2] Create package.xml and setup.py in examples/ros2/talker_listener/ for ROS 2 package structure
- [ ] T040 [US2] Write README.md in examples/ros2/talker_listener/ with prerequisites (ros-humble-desktop), installation commands (rosdep install, colcon build), run instructions (ros2 run), expected output
- [ ] T041 [US2] Create ROS 2 service example in examples/ros2/add_two_ints/ with service server adding two integers and returning result

### Gazebo Examples

- [ ] T042 [P] [US2] Create Gazebo warehouse world file in examples/gazebo/worlds/warehouse.world with walls, obstacles, lighting configuration
- [ ] T043 [P] [US2] Create TurtleBot3 robot URDF in examples/gazebo/models/turtlebot3/model.urdf with differential drive, lidar sensor
- [ ] T044 [US2] Create Gazebo launch file in examples/gazebo/warehouse_sim.launch.py spawning warehouse world and TurtleBot3 robot
- [ ] T045 [US2] Write README.md in examples/gazebo/ with Gazebo 11 installation (apt install gazebo11), launch instructions (ros2 launch examples warehouse_sim.launch.py), troubleshooting section for common errors

### Isaac Sim Examples (Optional - GPU Required)

- [ ] T046 [P] [US2] Create Isaac Sim basic scene Python script in examples/isaac/basic_scene.py loading Carter robot, adding camera sensor, running simulation loop
- [ ] T047 [US2] Write README.md in examples/isaac/ with hardware requirements (RTX 2060+, 16GB RAM), Isaac Sim installation instructions, cloud alternative (AWS g5.xlarge), script execution steps

### Integration with Book

- [ ] T048 [US2] Update Chapter 2 ROS 2 Fundamentals in docs/modules/ros2/fundamentals.mdx to reference examples/ros2/talker_listener/ with setup and run instructions
- [ ] T049 [US2] Update Chapter 4 Gazebo Simulation in docs/modules/gazebo/intro.mdx to reference examples/gazebo/ with warehouse simulation walkthrough
- [ ] T050 [US2] Create Chapter 6 NVIDIA Isaac Sim in docs/modules/isaac/intro.mdx with Isaac Sim architecture, synthetic data generation, reference to examples/isaac/

### Testing

- [ ] T051 [US2] Test ROS 2 examples on clean Ubuntu 22.04 VM and document any missing dependencies or installation issues
- [ ] T052 [US2] Test Gazebo examples on clean Ubuntu 22.04 VM and validate robot spawns correctly and sensors publish data
- [ ] T053 [US2] Test Isaac Sim example on RTX-equipped workstation and verify scene loads and simulation runs without errors

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - readers can access book online and run code examples locally

---

## Phase 5: User Story 3 - Reader Queries RAG Chatbot (Full-Book Mode) (Priority: P3)

**Goal**: Embed chatbot in book that answers questions from full book content with citations, <5s latency, ≥90% accuracy

**Independent Test**: Deploy book with chatbot embedded, send 5 test queries ("What is ROS 2?", "How do I install Gazebo?", "What hardware do I need for Isaac Sim?", "Explain SLAM", "What is the capstone project?"), verify responses cite correct chapters, contain accurate information, and return within 5 seconds

### Embedding Pipeline

- [ ] T054 [P] [US3] Create embedding generation script in backend/scripts/generate_embeddings.py that reads all MDX files from docs/ directory
- [ ] T055 [US3] Implement chunking logic in backend/scripts/generate_embeddings.py with section-aware splitting (500-1000 tokens, 100-token overlap), preserving Markdown headers
- [ ] T056 [US3] Implement OpenAI embedding API calls in backend/scripts/generate_embeddings.py using text-embedding-3-small model (1536 dimensions)
- [ ] T057 [US3] Implement Qdrant upsert logic in backend/scripts/generate_embeddings.py with metadata (chapter_id, chapter_title, section_title, url_path)
- [ ] T058 [US3] Add CLI arguments to backend/scripts/generate_embeddings.py for --docs-dir, --collection-name, --batch-size (default 100)
- [ ] T059 [US3] Run embedding generation on 3 sample chapters and verify 100-200 chunks uploaded to Qdrant with correct metadata

### Backend API Implementation

- [ ] T060 [P] [US3] Implement retrieval service in backend/src/services/retrieval_service.py with query_similar_chunks(query: str, top_k: int = 5) function using Qdrant search
- [ ] T061 [P] [US3] Implement response service in backend/src/services/response_service.py with generate_answer(query: str, chunks: List[BookChunk]) function calling OpenAI Chat Completion API
- [ ] T062 [US3] Implement citation extraction logic in backend/src/services/response_service.py to parse chunk metadata and generate Citation objects with chapter_id, section_title, url
- [ ] T063 [US3] Create POST /api/query endpoint in backend/src/api/query_routes.py accepting QueryRequest (query: str, session_id: str), calling retrieval and response services, logging to Neon query_logs table
- [ ] T064 [US3] Add error handling to backend/src/api/query_routes.py for empty queries (400), rate limit exceeded (429), OpenAI errors (500), Qdrant errors (500)
- [ ] T065 [US3] Create GET /api/health endpoint in backend/src/api/health_routes.py checking Qdrant connection, Neon connection, OpenAI API availability
- [ ] T066 [US3] Add request validation middleware in backend/src/main.py to sanitize input (max query length 500 chars, trim whitespace, reject empty queries)

### Chatbot UI Component

- [ ] T067 [P] [US3] Create ChatbotWidget React component in src/components/ChatbotWidget/ChatbotWidget.tsx with open/close toggle, message list, input field, send button
- [ ] T068 [P] [US3] Implement chat state management in src/components/ChatbotWidget/ChatbotWidget.tsx using React hooks (useState for messages, useEffect for API calls)
- [ ] T069 [US3] Implement API client in src/components/ChatbotWidget/api.ts with sendQuery(query: string, sessionId: string) function calling POST /api/query
- [ ] T070 [US3] Add loading state UI in src/components/ChatbotWidget/ChatbotWidget.tsx with spinner and "Thinking..." message during API call
- [ ] T071 [US3] Implement citation rendering in src/components/ChatbotWidget/MessageBubble.tsx with clickable links to chapter sections
- [ ] T072 [US3] Add chatbot icon in src/components/ChatbotWidget/FloatingButton.tsx positioned bottom-right with z-index 1000, pulsing animation
- [ ] T073 [US3] Integrate ChatbotWidget in Docusaurus theme by adding to src/theme/Root.tsx wrapper component

### Backend Deployment

- [ ] T074 [US3] Create Render Web Service configuration in render.yaml with Python 3.11 runtime, uvicorn start command, environment variables from Render dashboard
- [ ] T075 [US3] Deploy FastAPI backend to Render Free Tier and verify /api/health endpoint returns 200 OK
- [ ] T076 [US3] Configure CORS in backend/src/main.py to allow requests from GitHub Pages domain (https://<username>.github.io)
- [ ] T077 [US3] Update chatbot API client in src/components/ChatbotWidget/api.ts with deployed Render backend URL

### Testing & Validation

- [ ] T078 [US3] Create test question dataset in backend/tests/data/test_questions.json with 100 curated questions covering all 12 chapters
- [ ] T079 [US3] Create accuracy test script in backend/tests/integration/test_rag_accuracy.py that sends all 100 questions and validates responses contain correct chapter citations
- [ ] T080 [US3] Run accuracy test and verify ≥90% pass rate (90+ questions answered correctly with valid citations)
- [ ] T081 [US3] Create latency test script in backend/tests/integration/test_query_latency.py that measures p50, p95, p99 latencies for 50 concurrent queries
- [ ] T082 [US3] Run latency test and verify p95 latency <5 seconds

**Checkpoint**: At this point, User Story 3 is complete - chatbot embedded in book, responding to queries with citations, meeting accuracy and latency targets

---

## Phase 6: User Story 4 - Reader Uses Selection-Based RAG Mode (Priority: P4)

**Goal**: Enable targeted queries on highlighted text with constrained retrieval scope, ≥85% accuracy on selection-based test queries

**Independent Test**: Highlight 200-word section on "Isaac Sim synthetic data generation" in Chapter 6, right-click "Ask about selection", ask 3 questions ("What file formats are supported?", "Can this run on Jetson?", "How do I export to ROS 2?"), verify answers reference selected text and immediate context only, not full book

### Selection Mode UI

- [ ] T083 [P] [US4] Create SelectionMode component in src/components/ChatbotWidget/SelectionMode.tsx with text selection listener using window.getSelection()
- [ ] T084 [P] [US4] Implement context menu in src/components/ChatbotWidget/ContextMenu.tsx with "Ask about selection" option, positioned at mouse cursor on right-click
- [ ] T085 [US4] Add selection text display in src/components/ChatbotWidget/SelectionContext.tsx showing highlighted text (truncated to 1000 chars if >1000 words) with "Clear selection" button
- [ ] T086 [US4] Integrate SelectionMode into ChatbotWidget in src/components/ChatbotWidget/ChatbotWidget.tsx with conditional rendering based on selection state
- [ ] T087 [US4] Add visual indicator in src/components/ChatbotWidget/ChatbotWidget.tsx showing "Selection Mode Active" badge when selection exists

### Backend Selection API

- [ ] T088 [P] [US4] Create POST /api/query-selection endpoint in backend/src/api/query_routes.py accepting SelectionQueryRequest (selected_text: str, query: str, chapter_context: str, session_id: str)
- [ ] T089 [US4] Implement selection-based retrieval in backend/src/services/retrieval_service.py with query_with_selection(query: str, selected_text: str, chapter_id: str, top_k: int = 3) prioritizing chunks from same chapter
- [ ] T090 [US4] Update response service in backend/src/services/response_service.py with generate_answer_from_selection(query: str, selected_text: str, retrieved_chunks: List[BookChunk]) including selected text in context
- [ ] T091 [US4] Add selection text validation in backend/src/api/query_routes.py to reject selections >5000 words (truncate to 1000 with warning) or empty selections (400 error)
- [ ] T092 [US4] Update query logging in backend/src/api/query_routes.py to store selected_text in query_logs table for selection mode queries

### Frontend Integration

- [ ] T093 [US4] Update API client in src/components/ChatbotWidget/api.ts with sendSelectionQuery(selectedText: string, query: string, chapterContext: string, sessionId: string) function
- [ ] T094 [US4] Add chapter context detection in src/components/ChatbotWidget/SelectionMode.tsx by reading Docusaurus page metadata to pass chapter_id to backend
- [ ] T095 [US4] Implement fallback behavior in src/components/ChatbotWidget/MessageBubble.tsx showing "Information not found in selection. Search full book?" with button to switch to full-book mode

### Testing

- [ ] T096 [US4] Create selection-based test dataset in backend/tests/data/selection_questions.json with 50 questions paired with specific text selections from chapters
- [ ] T097 [US4] Create selection accuracy test in backend/tests/integration/test_selection_accuracy.py validating responses are constrained to selection context
- [ ] T098 [US4] Run selection accuracy test and verify ≥85% pass rate (43+ questions answered correctly from selection only)

**Checkpoint**: At this point, User Story 4 is complete - selection-based mode works, constrains answers to highlighted text, meets 85% accuracy target

---

## Phase 7: User Story 5 - Instructor Deploys Full Capstone Project (Priority: P5)

**Goal**: Complete Chapter 12 capstone guide covering voice → LLM → navigation → Jetson deployment, validate end-to-end pipeline with recommended hardware

**Independent Test**: Follow Chapter 12 step-by-step with hardware setup (RTX 3080 workstation, Jetson Orin kit, RealSense D435i, USB mic), complete 8 pipeline stages, verify robot responds to voice command "Go to the kitchen" in both simulation and edge deployment

### Capstone Chapter Content

- [ ] T099 [US5] Write Chapter 12 Capstone Humanoid Voice Control in docs/modules/capstone/humanoid-voice-control.mdx with project overview, 8-stage pipeline architecture, Mermaid diagram showing data flow (voice → text → plan → goal → navigation → feedback)
- [ ] T100 [US5] Add Stage 1: Voice Input Pipeline section in Chapter 12 with Whisper model selection (base vs small), ROS 2 audio_common integration, microphone configuration, speech-to-text node implementation
- [ ] T101 [US5] Add Stage 2: LLM Action Planner section in Chapter 12 with OpenAI function calling setup, navigation goal extraction, natural language → coordinates mapping, error handling for ambiguous commands
- [ ] T102 [US5] Add Stage 3: ROS 2 Navigation Stack section in Chapter 12 with Nav2 configuration, costmap parameters, planner selection (DWB vs TEB), recovery behaviors
- [ ] T103 [US5] Add Stage 4: Isaac Sim Training section in Chapter 12 with environment setup, RL policy training (PPO), sim-to-real transfer techniques, domain randomization
- [ ] T104 [US5] Add Stage 5: Perception Pipeline section in Chapter 12 with YOLOv8 object detection, RealSense depth fusion, point cloud processing, object pose estimation
- [ ] T105 [US5] Add Stage 6: Manipulation Planning section in Chapter 12 with MoveIt configuration, inverse kinematics, grasp planning, trajectory execution
- [ ] T106 [US5] Add Stage 7: Jetson Orin Deployment section in Chapter 12 with Jetson setup (JetPack 5.x), ROS 2 cross-compilation, TensorRT optimization, power management
- [ ] T107 [US5] Add Stage 8: End-to-End Testing section in Chapter 12 with integration test protocol, success criteria checklist, debugging tips, video demonstration

### Capstone Code Examples

- [ ] T108 [P] [US5] Create Whisper ROS 2 integration in examples/capstone/whisper_ros.py with audio subscriber, Whisper API call, text publisher to /voice_command topic
- [ ] T109 [P] [US5] Create LLM action planner in examples/capstone/llm_planner.py with OpenAI function calling, navigation goal extraction, coordinate validation, goal publisher to /nav_goal topic
- [ ] T110 [P] [US5] Create Nav2 launch file in examples/capstone/navigation_stack.launch.py with TurtleBot3 configuration, costmap params, planner config, AMCL localization
- [ ] T111 [P] [US5] Create Isaac Sim training environment in examples/capstone/isaac_train_env.py with Carter robot, obstacle course, reward function, Isaac Gym integration
- [ ] T112 [P] [US5] Create YOLO object detection node in examples/capstone/yolo_detector.py with RealSense subscriber, YOLOv8 inference, bounding box publisher
- [ ] T113 [P] [US5] Create MoveIt grasp planner in examples/capstone/grasp_planner.py with object pose subscriber, grasp pose generation, trajectory execution
- [ ] T114 [P] [US5] Create Jetson deployment script in examples/capstone/deploy_to_jetson.sh with SSH setup, package installation, service configuration, autostart setup
- [ ] T115 [US5] Create end-to-end integration test in examples/capstone/test_full_pipeline.py launching all nodes, sending voice command, validating navigation and manipulation execution

### Hardware Documentation

- [ ] T116 [P] [US5] Write Digital Twin Setup guide in docs/hardware/digital-twin.mdx with RTX GPU requirements (3060+), Ubuntu 22.04 installation, CUDA/cuDNN setup, Isaac Sim installation
- [ ] T117 [P] [US5] Write Edge AI Kit guide in docs/hardware/edge-kit.mdx with Jetson Orin specs (16GB/32GB), RealSense D435i setup, IMU integration, microphone configuration
- [ ] T118 [P] [US5] Write Robot Lab guide in docs/hardware/robot-lab.mdx with Unitree G1 specs, OP3 humanoid specs, Hiwonder setup, vendor links, cost breakdown

### Testing

- [ ] T119 [US5] Test Whisper integration on workstation with USB microphone and verify voice command "Go to the kitchen" transcribed correctly
- [ ] T120 [US5] Test LLM planner with sample commands and verify navigation goals extracted correctly (coordinates, orientation)
- [ ] T121 [US5] Test Nav2 stack in Gazebo warehouse simulation and verify robot navigates to goal avoiding obstacles
- [ ] T122 [US5] Test Isaac Sim training environment and verify RL policy converges within 1M steps
- [ ] T123 [US5] Test YOLO detection with RealSense camera and verify objects detected with >80% confidence
- [ ] T124 [US5] Test MoveIt grasp execution in Isaac Sim and verify successful grasp rate >70%
- [ ] T125 [US5] Test Jetson deployment on Orin kit and verify all ROS 2 nodes run without errors
- [ ] T126 [US5] Run end-to-end integration test and verify robot completes full pipeline from voice command to object manipulation

**Checkpoint**: At this point, User Story 5 is complete - capstone project guide written, code examples provided, validated on recommended hardware

---

## Phase 8: User Story 6 - Backend API Developer Tests RAG Endpoints (Priority: P6)

**Goal**: Validate backend API endpoints independently via automated tests, verify 100% pass rate for contract, integration, and error handling tests

**Independent Test**: Deploy FastAPI backend locally with test Qdrant collection (50 sample chunks), run pytest suite, verify all tests pass, check Qdrant logs show vector searches, check Neon logs show query records

### Contract Tests

- [ ] T127 [P] [US6] Create contract test for POST /api/query in backend/tests/contract/test_query_endpoint.py validating request schema (query: str required, session_id: str optional), response schema (answer: str, citations: List[Citation]), HTTP 200 status
- [ ] T128 [P] [US6] Create contract test for POST /api/query-selection in backend/tests/contract/test_selection_endpoint.py validating request schema (selected_text: str, query: str, chapter_context: str), response schema matches /api/query
- [ ] T129 [P] [US6] Create contract test for GET /api/health in backend/tests/contract/test_health_endpoint.py validating response contains status: "ok", qdrant_connected: bool, neon_connected: bool, openai_available: bool
- [ ] T130 [P] [US6] Create error response contract tests in backend/tests/contract/test_error_responses.py for 400 (bad request), 429 (rate limit), 500 (server error) with standardized JSON format

### Integration Tests - Qdrant

- [ ] T131 [P] [US6] Create Qdrant integration test in backend/tests/integration/test_qdrant.py testing collection creation, vector upsert, similarity search, metadata retrieval
- [ ] T132 [US6] Create Qdrant search accuracy test in backend/tests/integration/test_qdrant.py verifying top-5 chunks for 10 test queries contain relevant content (manual validation)
- [ ] T133 [US6] Create Qdrant error handling test in backend/tests/integration/test_qdrant.py simulating connection failure, empty results, invalid collection name

### Integration Tests - Neon Postgres

- [ ] T134 [P] [US6] Create Neon integration test in backend/tests/integration/test_neon.py testing query_logs insert, chat_sessions upsert, embeddings_metadata update
- [ ] T135 [US6] Create query logging validation test in backend/tests/integration/test_neon.py verifying all API queries logged with timestamp, query_text, response_text, latency_ms
- [ ] T136 [US6] Create Neon error handling test in backend/tests/integration/test_neon.py simulating connection timeout, constraint violation, transaction rollback

### Integration Tests - End-to-End API

- [ ] T137 [US6] Create end-to-end API test in backend/tests/integration/test_api_endpoints.py sending 10 queries to POST /api/query, verifying 200 OK, valid JSON response, citations present
- [ ] T138 [US6] Create rate limiting test in backend/tests/integration/test_api_endpoints.py sending 15 queries in 1 minute from same session_id, verifying requests 11-15 return 429 status
- [ ] T139 [US6] Create selection mode integration test in backend/tests/integration/test_api_endpoints.py sending selection query, verifying response prioritizes selected text context

### Postman Collection

- [ ] T140 [P] [US6] Create Postman collection in backend/tests/postman/rag_api_collection.json with POST /api/query request examples (valid query, empty query, long query)
- [ ] T141 [P] [US6] Add POST /api/query-selection examples to Postman collection with sample selections from Chapter 6 Isaac Sim section
- [ ] T142 [P] [US6] Add GET /api/health request to Postman collection with success response example
- [ ] T143 [US6] Add environment variables to Postman collection (BACKEND_URL, SESSION_ID) with local and production configurations

### Test Execution

- [ ] T144 [US6] Run pytest suite in backend/tests/ and verify 100% pass rate (all contract, integration, unit tests)
- [ ] T145 [US6] Run Postman collection manually and verify all requests succeed with expected responses
- [ ] T146 [US6] Check Qdrant dashboard logs and verify vector searches logged for each test query
- [ ] T147 [US6] Query Neon query_logs table and verify all test queries recorded with correct metadata

**Checkpoint**: At this point, User Story 6 is complete - backend API fully tested, contract validated, integration confirmed, 100% test pass rate

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Complete remaining chapters, documentation, CI/CD, and final validations

### Remaining Chapters

- [ ] T148 [P] Write Chapter 3 ROS 2 Workspace Setup in docs/modules/ros2/workspace-setup.mdx with colcon build system, package structure, ament_cmake vs ament_python, overlays
- [ ] T149 [P] Write Chapter 5 Unity Visualization Basics in docs/modules/unity/ros-unity-integration.mdx with Unity ROS TCP Connector, data visualization, AR/VR integration
- [ ] T150 [P] Write Chapter 7 Isaac Sim Training Workflows in docs/modules/isaac/training.mdx with Isaac Gym setup, PPO training, domain randomization, tensorboard monitoring
- [ ] T151 [P] Write Chapter 8 VSLAM & Navigation in docs/modules/vla/slam.mdx with ORB-SLAM3, RTAB-Map, Nav2 integration, map server configuration
- [ ] T152 [P] Write Chapter 9 Vision-Language-Action Systems in docs/modules/vla/architectures.mdx with RT-1, RT-2, PaLM-E architectures, attention mechanisms, action tokenization
- [ ] T153 [P] Write Chapter 10 VLA Fine-Tuning in docs/modules/vla/fine-tuning.mdx with dataset collection, LoRA fine-tuning, behavioral cloning, RLHF
- [ ] T154 [P] Write Chapter 11 Conversational Robotics in docs/modules/conversational/whisper.mdx with speech recognition, dialogue management, LLM integration, context tracking
- [ ] T155 Write summary and key takeaways sections for all 12 chapters (docs/intro.md and docs/modules/*/*)

### Documentation

- [ ] T156 [P] Write README.md in repository root with project overview, features list, quick start instructions (<30 min setup), architecture diagram, technology stack
- [ ] T157 [P] Write CONTRIBUTING.md with spec-driven development workflow, chapter contribution guide, code example guidelines, testing requirements
- [ ] T158 [P] Write DEPLOYMENT.md with GitHub Pages deployment steps, Render backend deployment, Qdrant setup, Neon setup, environment variables configuration
- [ ] T159 [P] Create architecture diagram in docs/assets/architecture.png showing Docusaurus → GitHub Pages, Chatbot UI → FastAPI Backend → Qdrant/Neon, ROS 2 Examples
- [ ] T160 Write troubleshooting guide in docs/troubleshooting.md with common errors for ROS 2, Gazebo, Isaac Sim, chatbot API, embedding generation

### CI/CD Workflows

- [ ] T161 [P] Create link validation workflow in .github/workflows/link-checker.yml running on every push, checking all internal and external links in docs/
- [ ] T162 [P] Create embedding regeneration workflow in .github/workflows/regenerate-embeddings.yml triggered on docs/ changes, running backend/scripts/generate_embeddings.py, updating Qdrant
- [ ] T163 [P] Create backend test workflow in .github/workflows/backend-tests.yml running pytest on backend/tests/ with coverage report
- [ ] T164 Create Lighthouse CI workflow in .github/workflows/lighthouse.yml running audit on GitHub Pages deployment, failing if scores <90

### Final Validations

- [ ] T165 Run technical accuracy review by domain expert in robotics/ROS 2, collect feedback, address critical errors and minor corrections
- [ ] T166 Run full Lighthouse audit on production deployment and verify Performance/Accessibility/Best Practices/SEO all ≥90
- [ ] T167 Run link validation across all deployed pages and verify zero broken links
- [ ] T168 Run full RAG accuracy test (100 questions) and verify ≥90% pass rate
- [ ] T169 Run selection accuracy test (50 questions) and verify ≥85% pass rate
- [ ] T170 Verify all 30+ code examples run successfully on clean Ubuntu 22.04 VM following book instructions
- [ ] T171 Run book build locally and verify zero build errors or warnings
- [ ] T172 Check Qdrant vector count and verify <90k vectors (within free tier limit)
- [ ] T173 Check Neon database size and verify <400MB (within free tier limit)
- [ ] T174 Validate quickstart instructions by having new developer follow README.md and complete setup in <30 minutes

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-8)**: All depend on Foundational phase completion
  - User stories CAN proceed in parallel (if staffed)
  - OR sequentially in priority order (P1 → P2 → P3 → P4 → P5 → P6)
  - Recommended: Complete US1 first (MVP), validate deployment, then proceed to US2-6
- **Polish (Phase 9)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Integrates with US1 (references book chapters) but independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Requires US1 chapters for embedding content, but can use sample chapters for initial testing
- **User Story 4 (P4)**: Depends on US3 completion (builds on chatbot UI and backend API)
- **User Story 5 (P5)**: Can start after Foundational (Phase 2) - Adds new chapter content, independently testable
- **User Story 6 (P6)**: Depends on US3 completion (tests backend API from US3)

### Within Each User Story

**User Story 1 (Book Content)**:
- Content creation tasks (T027-T029) can run in parallel
- Diagram tasks (T030-T032) can run in parallel after chapters written
- Deployment tasks (T033-T036) sequential after content complete

**User Story 2 (Code Examples)**:
- ROS 2 example tasks (T037-T041) can run in parallel
- Gazebo example tasks (T042-T045) can run in parallel
- Isaac example tasks (T046-T047) can run in parallel
- Integration tasks (T048-T050) after examples complete
- Testing tasks (T051-T053) sequential after integration

**User Story 3 (RAG Chatbot)**:
- Embedding pipeline tasks (T054-T059) sequential (chunking depends on reading)
- Backend API tasks (T060-T066): retrieval and response services can be parallel, routes after services
- Chatbot UI tasks (T067-T073): components can be parallel, integration after components
- Deployment tasks (T074-T077) sequential
- Testing tasks (T078-T082) sequential after deployment

**User Story 4 (Selection Mode)**:
- Selection UI tasks (T083-T087) can run in parallel
- Backend API tasks (T088-T092) can run in parallel with UI tasks
- Frontend integration tasks (T093-T095) after UI and API complete
- Testing tasks (T096-T098) sequential after integration

**User Story 5 (Capstone)**:
- Chapter content tasks (T099-T107) sequential (sections build on each other)
- Code example tasks (T108-T115) can run in parallel
- Hardware docs (T116-T118) can run in parallel
- Testing tasks (T119-T126) sequential (pipeline stages tested in order)

**User Story 6 (API Testing)**:
- Contract test tasks (T127-T130) can run in parallel
- Qdrant integration tasks (T131-T133) sequential
- Neon integration tasks (T134-T136) sequential
- API integration tasks (T137-T139) sequential
- Postman collection tasks (T140-T143) can run in parallel
- Test execution tasks (T144-T147) sequential

**Phase 9 (Polish)**:
- Chapter writing tasks (T148-T155) can run in parallel
- Documentation tasks (T156-T160) can run in parallel
- CI/CD workflow tasks (T161-T164) can run in parallel
- Final validation tasks (T165-T174) sequential

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T002, T003, T004, T005, T007, T008)
- All Foundational database tasks can run in parallel (T010, T011)
- All Foundational model tasks can run in parallel (T016, T017, T020, T021)
- All Foundational Docusaurus config tasks can run in parallel (T023, T024, T025)
- Once Foundational phase completes, User Stories 1, 2, 5 can start in parallel (US1 book content, US2 code examples, US5 capstone)
- After US3 completes, US4 and US6 can start in parallel
- Within US1: Content tasks (T027-T029) and diagram tasks (T030-T032) can run in parallel
- Within US2: All example creation tasks for different tools (ROS 2, Gazebo, Isaac) can run in parallel
- Within US3: Backend services (T060, T061) and UI components (T067, T068) can run in parallel
- Within Phase 9: All chapter writing tasks (T148-T155), all documentation tasks (T156-T160), all CI/CD tasks (T161-T164) can run in parallel

---

## Parallel Example: User Story 3 (RAG Chatbot)

```bash
# Launch embedding pipeline development:
Task: "Create embedding generation script in backend/scripts/generate_embeddings.py"
Task: "Implement chunking logic in backend/scripts/generate_embeddings.py"

# Launch backend services in parallel:
Task: "Implement retrieval service in backend/src/services/retrieval_service.py"
Task: "Implement response service in backend/src/services/response_service.py"

# Launch chatbot UI components in parallel:
Task: "Create ChatbotWidget React component in src/components/ChatbotWidget/ChatbotWidget.tsx"
Task: "Implement chat state management in src/components/ChatbotWidget/ChatbotWidget.tsx"
Task: "Implement API client in src/components/ChatbotWidget/api.ts"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T008)
2. Complete Phase 2: Foundational (T009-T026) - CRITICAL BLOCKER
3. Complete Phase 3: User Story 1 (T027-T036)
4. **STOP and VALIDATE**: Test book deployment, verify navigation works, diagrams render, Lighthouse ≥90
5. Deploy to GitHub Pages and demo

**Estimated Timeline**: 2-3 weeks for MVP (book with 3 chapters deployed)

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready (1 week)
2. Add User Story 1 → Test independently → Deploy/Demo (MVP! - week 2-3)
3. Add User Story 2 → Test independently → Deploy/Demo (runnable examples - week 4)
4. Add User Story 3 → Test independently → Deploy/Demo (RAG chatbot - week 5-6)
5. Add User Story 4 → Test independently → Deploy/Demo (selection mode - week 7)
6. Add User Story 5 → Test independently → Deploy/Demo (capstone project - week 8-9)
7. Add User Story 6 → Test independently → Deploy/Demo (API testing - week 10)
8. Complete Phase 9: Polish → Final validation → Release (week 11-12)

**Estimated Total Timeline**: 10-12 weeks for full project with all 6 user stories

### Parallel Team Strategy

With 3 developers:

1. **Week 1**: Team completes Setup + Foundational together
2. **Week 2-3** (once Foundational done):
   - Developer A: User Story 1 (Book Content)
   - Developer B: User Story 2 (Code Examples)
   - Developer C: User Story 5 (Capstone - can proceed independently)
3. **Week 4-5**:
   - Developer A: User Story 3 (RAG Chatbot)
   - Developer B: Continue US2, assist US3 with testing
   - Developer C: Continue US5, write remaining chapters
4. **Week 6-7**:
   - Developer A: User Story 4 (Selection Mode)
   - Developer B: User Story 6 (API Testing)
   - Developer C: Phase 9 (Polish - chapters, docs)
5. **Week 8**: Team performs final validation together

**Estimated Timeline with Team**: 6-8 weeks for full project

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group (e.g., T027-T029 "Chapter content creation")
- Stop at any checkpoint to validate story independently before proceeding
- File paths are absolute from repository root (docs/, backend/src/, examples/)
- Foundational phase (Phase 2) is CRITICAL - no user story can start until complete
- MVP strategy: Complete US1 first, validate deployment, get feedback before proceeding
- Testing is embedded in user story phases (not separate phase) for faster feedback
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
