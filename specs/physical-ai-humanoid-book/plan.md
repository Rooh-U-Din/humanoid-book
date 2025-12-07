# Implementation Plan: Physical AI & Humanoid Robotics Docusaurus Book + Integrated RAG Chatbot

**Branch**: `001-physical-ai-humanoid-book` | **Date**: 2025-12-05 | **Spec**: [spec.md](./spec.md)

**Input**: Feature specification from `specs/physical-ai-humanoid-book/spec.md`

## Summary

Build a comprehensive educational platform combining a Docusaurus-based technical textbook on Physical AI & Humanoid Robotics with an intelligent RAG chatbot that answers questions strictly from book content. The system integrates 12+ chapters covering ROS 2, Gazebo, Unity, NVIDIA Isaac Sim, Vision-Language-Action systems, conversational robotics, and a capstone humanoid project, with all code examples tested and runnable. The RAG backend (FastAPI + Qdrant + Neon Postgres) supports both full-book search and selection-based queries, ensuring accurate, citation-backed responses for learners.

**Technical Approach**: Multi-component architecture with Docusaurus 3 frontend (GitHub Pages), FastAPI async backend (cloud-hosted), Qdrant Cloud vector database (embeddings), Neon Serverless Postgres (metadata/logs), OpenAI APIs (embeddings + chat completion), and ROS 2 Humble example code (Ubuntu 22.04). All components orchestrated via GitHub Actions CI/CD with automated embedding regeneration on content updates.

## Technical Context

**Language/Version**:
- Python 3.11+ (backend API, embedding pipeline, ROS 2 examples)
- Node.js 18+ LTS (Docusaurus build system)
- TypeScript 5.x (React chatbot component)
- ROS 2 Humble Hawksbill (robotics examples)

**Primary Dependencies**:
- **Frontend**: `@docusaurus/core@3.x`, `@docusaurus/preset-classic@3.x`, `@docusaurus/theme-mermaid@3.x`, `react@18.x`, `axios@1.x`
- **Backend**: `fastapi@0.115+`, `uvicorn[standard]@0.30+`, `pydantic@2.x`, `sqlalchemy@2.x`, `qdrant-client@1.x`, `psycopg2-binary@2.9+`, `openai@1.x`
- **Robotics**: `ros-humble-desktop`, `gazebo11`, `nvidia-isaac-sim` (optional, hardware-dependent)

**Storage**:
- Neon Serverless Postgres (query logs, session metadata, embedding metadata) - Free tier: 0.5GB
- Qdrant Cloud (vector embeddings for book content) - Free tier: 100k vectors, 1GB
- Git LFS (optional for large binary assets like Gazebo models)

**Testing**:
- Backend: `pytest@8.x`, `pytest-asyncio@0.23+`, `httpx@0.27+` (async client testing)
- Frontend: `@docusaurus/module-type-aliases`, Docusaurus built-in build validation
- Integration: Custom test suite for RAG query accuracy (100 curated questions)
- Code Examples: ROS 2 testing framework, manual validation on Ubuntu 22.04

**Target Platform**:
- Book: GitHub Pages (static hosting, CDN distribution)
- Backend API: Cloud platform with Python support (Render Free Tier, Railway, or AWS Lambda)
- Development: Ubuntu 22.04 LTS (ROS 2 Humble native target)
- Production: Cross-platform web (Chrome, Firefox, Safari, Edge - desktop + mobile)

**Project Type**: Hybrid Web Application + Documentation Site
- Multi-component architecture: Static site (Docusaurus) + RESTful backend (FastAPI) + Vector DB (Qdrant) + Relational DB (Neon Postgres)

**Performance Goals**:
- Book page load: <3s p95 (Lighthouse Performance ≥90)
- RAG query latency: <5s p95 (embedding retrieval + LLM generation)
- Code example success rate: ≥95% on clean Ubuntu 22.04 installation
- Embedding generation: <10 minutes for full book (100k+ tokens)
- Build time: <10 minutes (GitHub Actions)

**Constraints**:
- **Free-tier compatibility**: All infrastructure must work within free tier limits (Qdrant 100k vectors, Neon 0.5GB, GitHub Actions 2000 min/month)
- **No external knowledge**: RAG responses must cite book content only, no hallucination
- **Zero paid dependencies**: Readers must be able to run everything locally with free accounts
- **Offline code examples**: All ROS 2/Gazebo examples must run without internet after initial setup
- **Security**: No API keys in code, CORS restrictions, rate limiting (10 queries/min per session)

**Scale/Scope**:
- Book content: 12+ chapters, 150-200 pages equivalent, 30+ code examples
- Vector database: ~1000-2000 chunks (100k-150k tokens embedded)
- Diagrams: 15+ Mermaid diagrams (architecture, workflow, data flow)
- User stories: 6 acceptance scenarios (P1-P6 priority)
- Deployment targets: 1 static site + 1 backend API + 2 databases (vector + relational)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Accuracy and Reliability ✅ PASS

**Gate**: All content, code, and system outputs based strictly on verified information. No hallucinated or unverified data. RAG responses cite book content only.

**Compliance**:
- Technical accuracy review required before release (SC-012: domain expert validation)
- All code examples tested on Ubuntu 22.04 (FR-004, SC-003: 95%+ success rate)
- RAG system design enforces book-content-only constraint (FR-009: no external knowledge)
- Citation system links every response to source chapter sections (FR-007)
- Automated link validation in CI/CD (SC-010: zero broken links)

**Phase 1 Verification**: Data model includes `citations` field in QueryLog entity; API contract specifies `{answer: string, citations: Citation[]}` response format; embedding pipeline metadata includes chapter URLs for citation generation.

---

### II. Modular, Spec-Driven Development ✅ PASS

**Gate**: All features, chapters, and components require specifications before implementation. No code without a spec.

**Compliance**:
- This plan document serves as Phase 0 output (research.md to be generated)
- Phase 1 will produce: data-model.md, quickstart.md, contracts/ (OpenAPI specs)
- Phase 2 tasks.md will break down implementation into testable units
- Each chapter follows consistent structure: theory + diagrams + code + summary (FR-002)
- Repository includes /specs directory with feature-level specifications

**Phase 1 Verification**: All Phase 1 artifacts (data-model.md, contracts/api.openapi.yaml, contracts/database-schema.sql, quickstart.md) must exist before implementation begins.

---

### III. Maintainability and Clarity ✅ PASS

**Gate**: Code, documentation, and architecture prioritize readability for developers and technical readers. Simple, well-documented solutions preferred.

**Compliance**:
- Clean architecture for backend (separation of concerns: models/, services/, api/)
- README.md with one-click setup (<30 min to run locally, SC-009)
- CONTRIBUTING.md guide for adding chapters via spec-driven process
- Code examples include prerequisites, installation commands, expected output (FR-004)
- Consistent chapter structure reduces cognitive load (FR-002)
- Naming conventions enforced across stack (data models, API routes, database tables)

**Phase 1 Verification**: Data model uses clear entity names (Chapter, BookChunk, QueryLog, ChatSession, CodeExample, HardwareSpec); API contracts use RESTful conventions; architecture diagrams document component interactions.

---

### IV. End-to-End Consistency ✅ PASS

**Gate**: All components maintain consistent data models, naming conventions, error handling patterns, and documentation standards.

**Compliance**:
- Unified entity model: Chapter ID flows through Docusaurus frontmatter → BookChunk metadata → QueryLog citations → API responses
- Error handling pattern: HTTP status codes (400 bad request, 429 rate limit, 500 server error) with structured JSON responses (FR-013)
- Logging: Structured JSON logs with request ID tracing across backend services (FR-013)
- Documentation style: Consistent MDX frontmatter, admonition usage, code block formatting
- Database naming: snake_case tables (query_logs, chat_sessions), camelCase TypeScript, PascalCase Python classes

**Phase 1 Verification**: Contracts document error response schemas; data model shows relationships between Chapter, BookChunk, QueryLog entities; naming conventions documented in plan.

---

### V. Security and Privacy ✅ PASS

**Gate**: User queries, embeddings, and data handled with appropriate security. No exposure of secrets, PII, or vector embeddings. APIs implement authentication, input validation, rate limiting.

**Compliance**:
- Environment variable management: .env.example template, no hardcoded secrets (FR-016)
- Input validation: Pydantic models for all API requests (max query length, selection text limits)
- Rate limiting: 10 queries/min per session_id stored in ChatSession table (edge case: rate limit hit → 429 response)
- CORS configuration: Allow GitHub Pages domain only (FR-013)
- Query logging: Store query_text and response_text, no PII collection (QueryLog entity)
- Edge case handling: Selection >1000 words → truncate + warning (edge case documented)
- OpenAI API: No training on user data (OpenAI API policy compliance)

**Phase 1 Verification**: API contracts include rate limit headers; database schema includes session_id (UUID) without PII; security section in quickstart.md documents secret management; CORS middleware configuration in contracts.

---

**Constitution Gate Status: ALL PRINCIPLES PASS ✅**

No violations requiring justification in Complexity Tracking section.

---

## Project Structure

### Documentation (this feature)

```text
specs/physical-ai-humanoid-book/
├── spec.md              # Feature specification (COMPLETE)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output: Technology evaluation, best practices, trade-offs
├── data-model.md        # Phase 1 output: Entity definitions, relationships, schemas
├── quickstart.md        # Phase 1 output: Local setup guide, environment configuration
├── contracts/           # Phase 1 output: API and database contracts
│   ├── api.openapi.yaml         # OpenAPI 3.0 spec for FastAPI endpoints
│   ├── database-schema.sql      # Neon Postgres schema with migrations
│   ├── qdrant-collection.json   # Qdrant collection config (vector dimensions, distance metric)
│   └── error-responses.json     # Standardized error response formats
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Documentation Site (Docusaurus)
docs/                           # MDX chapter content
├── intro.md                    # Course overview, learning objectives
├── modules/                    # Main course content
│   ├── ros2/                   # ROS 2 chapters
│   │   ├── fundamentals.mdx
│   │   ├── workspace-setup.mdx
│   │   └── communication.mdx
│   ├── gazebo/                 # Gazebo simulation
│   │   ├── intro.mdx
│   │   └── worlds.mdx
│   ├── unity/                  # Unity visualization
│   │   └── ros-unity-integration.mdx
│   ├── isaac/                  # NVIDIA Isaac Sim
│   │   ├── intro.mdx
│   │   ├── training.mdx
│   │   └── synthetic-data.mdx
│   ├── vla/                    # Vision-Language-Action
│   │   ├── architectures.mdx
│   │   └── fine-tuning.mdx
│   ├── conversational/         # Conversational robotics
│   │   ├── whisper.mdx
│   │   └── llm-action-planning.mdx
│   └── capstone/               # Final project
│       └── humanoid-voice-control.mdx
└── hardware/                   # Hardware specifications
    ├── digital-twin.mdx
    ├── edge-kit.mdx
    └── robot-lab.mdx

backend/                        # FastAPI RAG system
├── src/
│   ├── models/                 # Pydantic, SQLAlchemy, Qdrant schemas
│   ├── services/               # Embedding, retrieval, response generation
│   └── api/                    # FastAPI routes
└── tests/

src/components/ChatbotWidget/   # Docusaurus custom chatbot UI

examples/                       # ROS 2, Gazebo, Isaac Sim code examples
├── ros2/
├── gazebo/
└── isaac/
```

**Structure Decision**: This project uses a **hybrid multi-component architecture** combining a static documentation site (Docusaurus), backend API service (FastAPI), and standalone code examples repository. This separation aligns with Constitution Principle III (Maintainability and Clarity) by keeping concerns separated while maintaining clear integration points.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**Status: No violations detected. This section intentionally left empty.**

All complexity has been justified within constitutional bounds. The multi-component architecture (Docusaurus + FastAPI + Qdrant + Neon) is necessary to satisfy functional requirements FR-006 through FR-013.

---

## Phases

### Phase 0: Research & Technology Evaluation

**Output**: `research.md`

**Research Topics**:
1. Docusaurus best practices for technical documentation
2. RAG chunking strategies for long-form content
3. OpenAI Agents vs. Direct API comparison
4. Qdrant collection optimization for book content
5. ROS 2 + Gazebo + Isaac Sim integration patterns
6. Jetson Orin deployment best practices
7. GitHub Actions for multi-component deployment

**Key Decisions to Document**:
- Chunking strategy: Section-aware with 500-1000 token limit, 100-token overlap
- RAG implementation: Direct OpenAI API (more control than Agents SDK)
- Qdrant config: Cosine similarity, single collection with chapter_id filtering
- Backend platform: Render Free Tier (documented cold start behavior)
- Example complexity: 3-tier ladder (Basic/Intermediate/Advanced)

---

### Phase 1: Foundation & Design

**Outputs**: `data-model.md`, `contracts/`, `quickstart.md`

**Tasks**:
1. Define 6 core entities: Chapter, BookChunk, QueryLog, ChatSession, CodeExample, HardwareSpec
2. Create OpenAPI spec for `/api/query` and `/api/query-selection` endpoints
3. Write Neon Postgres migration SQL with indexes
4. Design Qdrant collection schema (1536-dim vectors, metadata payload)
5. Document quickstart setup (<30 min target)

---

### Phase 2: Content Creation & Examples

**Outputs**: 12+ chapters in `docs/`, 30+ examples in `examples/`

**Chapter Structure** (per FR-002):
- Theory section (500-1000 words)
- Architecture diagrams (1-2 Mermaid)
- Code examples (2-3 per chapter)
- Summary with key takeaways

**Testing Protocol**: Validate 95%+ success rate on clean Ubuntu 22.04

---

### Phase 3: RAG Integration

**Outputs**: Functional RAG system with embedding pipeline, backend API, chatbot UI

**Components**:
1. Embedding pipeline: `backend/scripts/generate_embeddings.py`
2. Backend API: FastAPI routes with rate limiting, CORS, structured logging
3. Chatbot UI: React component with selection mode support
4. Integration testing: 100-question accuracy test (90%+ target)

---

### Phase 4: Deployment & CI/CD

**Outputs**: GitHub Actions workflows, `DEPLOYMENT.md`

**Workflows**:
1. Deploy book to GitHub Pages
2. Deploy backend to Render
3. Regenerate embeddings on `docs/` changes
4. Link validation CI check

---

## Key Architectural Decisions

### Decision 1: Direct OpenAI API (not Agents SDK)
**Rationale**: More control for book-content-only constraint, simpler debugging, no token overhead

### Decision 2: Section-Aware Chunking with Token Limits
**Rationale**: Preserves semantic coherence while ensuring optimal retrieval precision

### Decision 3: Render Free Tier for Backend
**Rationale**: Zero cost, simple deployment, acceptable cold start trade-off for educational project

### Decision 4: Progressive Example Complexity (3 Tiers)
**Rationale**: Balances accessibility (Tier 1: 100% reproducible) with advanced learning (Tier 3: RTX GPU)

### Decision 5: Client-Only Chatbot Component
**Rationale**: Preserves Docusaurus SSG, easy maintenance, fast code splitting

---

## Risk Analysis & Mitigation

### Risk 1: OpenAI API Rate Limits or Cost Overruns
**Mitigation**: 10 queries/min rate limiting, $20/month billing alert, response caching, kill switch

### Risk 2: Qdrant Free Tier Limit Exceeded (100k vectors)
**Mitigation**: Monitor at 90k threshold, increase chunk size if needed, pruning strategy documented

### Risk 3: Code Examples Fail on Fresh Ubuntu Install
**Mitigation**: Dependency pinning, Docker containers, weekly CI testing, troubleshooting wiki

---

## Success Criteria

- ✅ 12+ complete chapters with consistent structure
- ✅ 30+ code examples with ≥95% success rate
- ✅ RAG accuracy ≥90% (full-book), ≥85% (selection mode)
- ✅ Lighthouse score ≥90
- ✅ Build time <10 minutes
- ✅ Setup time <30 minutes
- ✅ Zero broken links (automated check)

---

**Plan Status**: COMPLETE ✅

**Next Action**: Run `/sp.tasks` to generate detailed implementation tasks from this plan.
