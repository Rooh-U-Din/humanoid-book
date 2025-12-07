---
id: 002
title: Constitution update RAG chatbot system
stage: constitution
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "major-update", "rag-system", "architecture"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
---

## Prompt

/sp.constitution

Project: AI/Spec-Driven Book + Integrated RAG Chatbot System

Core Principles:
- Accuracy and reliability based strictly on verified and reproducible information
- Modular, spec-driven development following Spec-Kit Plus workflows
- Maintainability and clarity for developers and technical readers
- End-to-end consistency across Docusaurus, RAG backend, database, and deployment stack
- Security and privacy in handling user queries and vector embeddings

Key Standards (Book Creation):
- Book must be written using Docusaurus and deployed to GitHub Pages
- All chapters, sections, and examples must follow a consistent structure and style
- Use citation-friendly writing (clarity, correctness, technical consistency)
- All code samples must be runnable, copy-paste ready, and tested
- Writing level: Intermediate–advanced technical audience (software engineers)

Key Standards (RAG Chatbot):
- Chatbot must use: OpenAI Agents/ChatKit SDKs, FastAPI backend, Neon Serverless Postgres, and Qdrant Cloud Free Tier
- RAG must answer questions using book content only
- Must support "Answer from selected text only" mode
- Data pipeline must include embedding, ingestion, retrieval, and response generation
- API must include structured logging, error handling, and monitoring
- All components must be fully reproducible with a provided setup guide

Content Constraints:
- Book length: Minimum 8 chapters
- Each chapter must include: explanation, diagrams, code examples, and a summary
- Include at least one end-to-end project example integrating the RAG system
- All diagrams must be generated using Mermaid or MDX-compatible formats
- Must include a full "Deployment Guide" for GitHub Pages + FastAPI + Neon + Qdrant

Engineering Constraints:
- Backend must follow clean architecture principles
- Embedding pipeline must run asynchronously and be scalable
- Database schema for Postgres + Qdrant must be documented
- Environment variables and secrets must be well-structured and never hard-coded
- All work must be compatible with free-tier limits of Qdrant & Neon

Success Criteria:
- Docusaurus book builds cleanly with no errors and deploys successfully to GitHub Pages
- RAG chatbot loads inside the book and answers questions accurately based on the book content
- "Selection-based answering" works reliably
- All API endpoints pass functional testing
- All diagrams, code, and examples are correct and consistent
- Architecture is reproducible using installation instructions
- Entire project meets Spec-Kit Plus standards for clarity, accuracy, and maintainability

## Response snapshot

Updated constitution from v1.0.0 to v2.0.0 with MAJOR version bump due to fundamental project scope change from "Learn AI-Native Software Engineering" book to "AI/Spec-Driven Book + Integrated RAG Chatbot System".

**Major Changes:**
1. Replaced 5 original principles with new set focused on RAG system requirements
2. Added separate "Key Standards" sections for Book Creation and RAG Chatbot
3. Added new "Content Constraints" and "Engineering Constraints" sections
4. Expanded "Success Criteria and Deliverables" with RAG-specific requirements
5. Added detailed technical stack requirements (FastAPI, Neon, Qdrant, OpenAI)

**New Core Principles:**
- I. Accuracy and Reliability (verified information only, RAG cites book content only)
- II. Modular, Spec-Driven Development (Spec-Kit Plus workflows mandatory)
- III. Maintainability and Clarity (readable code for learning and production)
- IV. End-to-End Consistency (across all stack components)
- V. Security and Privacy (proper handling of user queries and embeddings)

**Technical Stack Codified:**
- Frontend: Docusaurus v3+ with MDX v3, deployed to GitHub Pages
- Backend: FastAPI with clean architecture
- Database: Neon Serverless Postgres (free tier)
- Vector DB: Qdrant Cloud (free tier)
- LLM: OpenAI Agents/ChatKit SDKs

**Templates Requiring Updates:**
- plan-template.md (needs RAG backend architecture patterns)
- spec-template.md (needs book chapter specification format)
- tasks-template.md (needs embedding/ingestion pipeline task examples)

## Outcome

- ✅ Impact: Constitution completely redefined to support RAG chatbot system; provides clear technical constraints and success criteria for complex multi-component architecture
- 🧪 Tests: No tests run (constitution is governance document)
- 📁 Files: 1 modified (.specify/memory/constitution.md), 1 PHR created (this record)
- 🔁 Next prompts: User should run /sp.specify to create first feature specification using new constitution principles
- 🧠 Reflection: MAJOR version bump appropriate due to complete reorientation of project scope. Templates will need updates to align with RAG system architecture patterns. New principles provide strong foundation for both educational book quality and production RAG system requirements.

## Evaluation notes (flywheel)

- Failure modes observed: None - constitution successfully updated with comprehensive coverage of RAG system requirements
- Graders run and results (PASS/FAIL): N/A - governance document
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Create first feature spec for RAG system component (e.g., embedding pipeline, chatbot UI, or book chapter structure) to validate constitution principles are actionable and sufficient
