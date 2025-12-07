<!--
Sync Impact Report:
Version change: 1.0.0 -> 2.0.0
List of modified principles:
  - REMOVED: Teach-by-doing (replaced with Accuracy and Reliability)
  - REMOVED: Spec-first development (absorbed into broader Modular Development principle)
  - MODIFIED: Full reproducibility -> Enhanced with RAG chatbot requirements
  - REMOVED: Open-source first (absorbed into Key Standards)
  - REMOVED: AI-augmented authorship (absorbed into broader project context)
  - ADDED: Accuracy and Reliability (verified information only)
  - ADDED: Modular, Spec-Driven Development
  - ADDED: Maintainability and Clarity
  - ADDED: End-to-End Consistency (across all stack components)
  - ADDED: Security and Privacy (user queries and embeddings)
Added sections:
  - Key Standards (Book Creation) - expanded with technical audience requirements
  - Key Standards (RAG Chatbot) - new section for RAG-specific requirements
  - Content Constraints - new section for book structure requirements
  - Engineering Constraints - new section for architecture requirements
  - Success Criteria and Deliverables - expanded with RAG integration criteria
Removed sections:
  - Previous simplified Key Standards section
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending (needs RAG architecture)
  - .specify/templates/spec-template.md: ⚠ pending (needs book chapter templates)
  - .specify/templates/tasks-template.md: ⚠ pending (needs embedding pipeline tasks)
Follow-up TODOs:
  - Update plan-template.md to include RAG backend architecture patterns
  - Update spec-template.md to include book chapter specification format
  - Update tasks-template.md to include embedding/ingestion pipeline task examples
  - Create example ADRs for OpenAI vs other LLMs, Qdrant vs alternatives, Neon vs alternatives
-->
# AI/Spec-Driven Book + Integrated RAG Chatbot System Constitution

## Core Principles

### I. Accuracy and Reliability
All content, code, and system outputs MUST be based strictly on verified and reproducible information. No hallucinated or unverified data allowed. RAG responses MUST cite book content only.

**Rationale**: Trust is paramount for both educational content and AI-powered systems. Users must be able to verify every claim and reproduce every result.

### II. Modular, Spec-Driven Development
All development MUST follow Spec-Kit Plus workflows. Every feature, chapter, and component requires a specification before implementation. No code without a spec.

**Rationale**: Specifications ensure clarity, enable parallel development, provide documentation, and create reproducible workflows for both human and AI collaborators.

### III. Maintainability and Clarity
Code, documentation, and architecture MUST prioritize readability and maintainability for developers and technical readers. Prefer simple, well-documented solutions over clever optimizations.

**Rationale**: The project serves both as a learning resource and a production system. Clear code teaches better and maintains better.

### IV. End-to-End Consistency
All components (Docusaurus book, FastAPI backend, Neon Postgres, Qdrant vector DB, deployment pipelines) MUST maintain consistent data models, naming conventions, error handling patterns, and documentation standards.

**Rationale**: Consistency reduces cognitive load, enables faster debugging, and ensures components integrate seamlessly across the full stack.

### V. Security and Privacy
User queries, embeddings, and data MUST be handled with appropriate security measures. No exposure of secrets, PII, or vector embeddings. All APIs MUST implement proper authentication, input validation, and rate limiting.

**Rationale**: Educational systems handling user queries require robust security. Privacy violations destroy trust and violate user expectations.

## Key Standards (Book Creation)

- **Documentation Framework**: Docusaurus v3+ with MDX v3 support
- **Deployment Target**: GitHub Pages (automatic deployment via GitHub Actions)
- **Specification Tool**: Spec-Kit Plus (all features, chapters, and examples MUST have corresponding spec files)
- **Version Control**: Git + GitHub with meaningful commit messages and branch protection
- **Code Examples**: All code snippets MUST be runnable, copy-paste ready, and tested
- **Writing Level**: Intermediate–advanced technical audience (software engineers)
- **Writing Style**: Clear, technically precise, citation-friendly (clarity, correctness, consistency)
- **Structure**: Consistent chapter/section structure throughout the book
- **Language**: English (US spelling)
- **Formatting**: Markdown with MDX where needed; follow Docusaurus best practices (front matter, admonitions, code tabs, etc.)

## Key Standards (RAG Chatbot)

- **LLM Framework**: OpenAI Agents/ChatKit SDKs (for conversational interface)
- **Backend Framework**: FastAPI (RESTful API with async support)
- **Database**: Neon Serverless Postgres (for metadata, user sessions, query logs)
- **Vector Database**: Qdrant Cloud Free Tier (for embeddings and semantic search)
- **RAG Scope**: Chatbot MUST answer questions using book content ONLY (no external knowledge)
- **Selection Mode**: MUST support "Answer from selected text only" mode for targeted queries
- **Data Pipeline**: MUST include embedding generation, ingestion, retrieval, and response generation stages
- **API Standards**: MUST include structured logging, comprehensive error handling, and monitoring/observability
- **Reproducibility**: All components MUST be fully reproducible via provided setup guide
- **Documentation**: Architecture, API contracts, data schemas, deployment procedures MUST be documented

## Content Constraints

- **Book Length**: Minimum 8 chapters with consistent depth and coverage
- **Chapter Structure**: Each chapter MUST include:
  - Explanation section (concepts and theory)
  - Diagrams (architecture, flow, data models)
  - Code examples (runnable and tested)
  - Summary section (key takeaways)
- **End-to-End Project**: MUST include at least one complete project example integrating the RAG system from spec to deployment
- **Diagram Format**: All diagrams MUST use Mermaid or MDX-compatible formats (no external image dependencies)
- **Deployment Guide**: MUST include comprehensive deployment guide covering:
  - GitHub Pages setup and configuration
  - FastAPI backend deployment
  - Neon Postgres setup and schema migration
  - Qdrant Cloud configuration and indexing

## Engineering Constraints

- **Architecture**: Backend MUST follow clean architecture principles (separation of concerns, dependency inversion, testability)
- **Embedding Pipeline**: MUST run asynchronously and be horizontally scalable
- **Database Schema**: Postgres schema and Qdrant collections MUST be fully documented with migration scripts
- **Secrets Management**: Environment variables and secrets MUST be well-structured, never hard-coded, and documented in .env.example
- **Free Tier Compatibility**: All infrastructure MUST be compatible with free-tier limits:
  - Qdrant Cloud Free Tier: 1GB storage, 100k vectors
  - Neon Serverless Postgres: 0.5GB storage, shared compute
- **Repository Structure**: Entire project MUST live in a single public GitHub repository
- **Zero Paid Dependencies**: Readers MUST be able to build, run, and deploy the entire system using only free-tier services
- **Dependency Management**: All dependencies MUST be specified in package.json (frontend) and requirements.txt (backend) with version pinning where necessary
- **Build Performance**: GitHub Actions build MUST complete in under 10 minutes
- **Cross-Platform**: Final deployed book MUST be fully functional on mobile and desktop browsers
- **Zero Broken Links**: No broken links or failed builds allowed at any release (enforced via CI)

## Success Criteria and Deliverables

### Success Criteria

- **Book Deployment**: Docusaurus book builds cleanly with no errors and deploys successfully to GitHub Pages
- **RAG Integration**: RAG chatbot loads inside the book and answers questions accurately based exclusively on book content
- **Selection Mode**: "Answer from selected text only" mode works reliably for targeted queries
- **API Reliability**: All FastAPI endpoints pass functional testing with proper error handling
- **Content Quality**: All diagrams, code examples, and explanations are technically correct and consistent
- **Reproducibility**: Architecture is fully reproducible following only the provided installation instructions
- **Spec Compliance**: Entire project meets Spec-Kit Plus standards for clarity, accuracy, and maintainability
- **Chapter Completeness**: Every chapter has at least one complete Spec-Kit Plus specification file and corresponding generated output
- **Local Development**: Readers can run `npm install && npm start` locally and see the full book with working chatbot
- **End-to-End Examples**: At least one complete, runnable end-to-end example (from .sp → code → Docusaurus page → RAG chatbot integration)
- **Performance**: Final site scores ≥90 on Lighthouse (performance, accessibility, best practices, SEO)
- **Link Validation**: Zero dead links (checked automatically in CI/CD pipeline)

### Deliverables

#### Documentation
- **README.md**: One-click setup instructions for entire system (book + RAG backend)
- **CONTRIBUTING.md**: Clear guide for adding new chapters using spec-driven process
- **DEPLOYMENT.md**: Step-by-step deployment guide for GitHub Pages, FastAPI, Neon, and Qdrant

#### Configuration
- **docusaurus.config.js**: Fully configured for GitHub Pages deployment with chatbot integration
- **GitHub Actions Workflows**: Build, test, and deploy pipeline for both book and backend
- **.env.example**: Template for all required environment variables (API keys, DB URLs, etc.)

#### Specifications
- **/specs directory**: All Spec-Kit Plus constitution and chapter specification files
- **Architecture specs**: Data models, API contracts, embedding pipeline specifications

#### Source Code
- **/docs or /src/pages**: Docusaurus book content following conventions
- **/backend**: FastAPI application with clean architecture
  - **/backend/src/models**: Data models (Pydantic, SQLAlchemy, Qdrant schemas)
  - **/backend/src/services**: Business logic (embedding, retrieval, response generation)
  - **/backend/src/api**: FastAPI routes and endpoints
  - **/backend/tests**: Unit, integration, and contract tests
- **/frontend**: Chatbot UI component integrated with Docusaurus
- **/examples**: Working code examples referenced in book chapters

#### Data & Infrastructure
- **Database migrations**: SQL migration scripts for Neon Postgres schema
- **Qdrant collections**: Collection definitions and indexing configuration
- **Embedding pipeline**: Scripts for generating and ingesting book content embeddings

## Governance

This constitution defines the core principles and standards for the project. Amendments to this constitution require a documented proposal, review, and approval process.

### Versioning Policy
- **MAJOR**: Backward-incompatible changes to governance, principle removals, or core architecture redefinitions
- **MINOR**: New principles added, sections materially expanded, or new mandatory requirements
- **PATCH**: Clarifications, wording improvements, typo fixes, non-semantic refinements

### Compliance Review
Compliance with these principles MUST be verified during:
- Code reviews (architecture, security, maintainability)
- Automated CI checks (tests, linting, link validation, build success)
- Specification reviews (completeness, accuracy, consistency)
- Pre-release validation (deployment testing, integration testing)

### Amendment Procedure
1. Propose amendment with clear rationale and impact analysis
2. Update affected templates and documentation
3. Review with project stakeholders
4. Document in Sync Impact Report
5. Increment version according to semantic versioning rules

**Version**: 2.0.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05
