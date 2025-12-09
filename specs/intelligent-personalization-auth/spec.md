# Feature Specification: Intelligent Personalization & Authentication System

**Feature Branch**: `intelligent-personalization-auth`
**Created**: 2025-12-09
**Status**: Draft
**Input**: User description: "Add reusable intelligence using Claude Code Subagents and Agent Skills, Better Auth signup/signin with user background collection, and content personalization based on user expertise"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - New User Onboarding with Background Collection (Priority: P1)

A new user visits the Physical AI & Humanoid Robotics book and wants personalized content. They sign up using Better Auth, answer questions about their background (programming experience, robotics knowledge, goals), and the system remembers their preferences.

**Why this priority**: Authentication is foundational - all personalization features depend on knowing who the user is. Without auth, we cannot persist preferences or provide personalized experiences.

**Independent Test**: Can be fully tested by creating a new account, completing the background questionnaire, and verifying the data is persisted and retrievable on subsequent visits.

**Acceptance Scenarios**:

1. **Given** a new visitor on any book page, **When** they click "Sign Up", **Then** they see a Better Auth registration form with email/password options
2. **Given** a user completes registration, **When** they submit the form, **Then** they are presented with a background questionnaire (3-5 questions)
3. **Given** a user answers background questions, **When** they submit, **Then** their profile is created with expertise level and learning goals stored
4. **Given** a returning user, **When** they log in, **Then** their background preferences are loaded and applied to their session

---

### User Story 2 - Personalize Chapter Content Button (Priority: P2)

A logged-in user reading a chapter wants content adapted to their expertise level. They click a "Personalize Content" button, and the chapter is rewritten to match their background (e.g., simplified for beginners or more technical for experts).

**Why this priority**: This is the core value proposition - making the educational content more accessible based on user's background. Depends on authentication (P1) being in place.

**Independent Test**: Can be fully tested by logging in as a user with "beginner" profile, clicking "Personalize Content" on any chapter, and verifying the content is adapted with simpler explanations.

**Acceptance Scenarios**:

1. **Given** a logged-in user with expertise level "beginner" viewing a chapter, **When** they click "Personalize Content", **Then** the chapter is rewritten with simpler explanations, more context, and prerequisite concepts
2. **Given** a logged-in user with expertise level "expert" viewing a chapter, **When** they click "Personalize Content", **Then** the chapter skips basics and focuses on advanced details and edge cases
3. **Given** a personalized chapter, **When** the user clicks "Show Original", **Then** the original content is restored
4. **Given** a user without a profile (not logged in), **When** they click "Personalize Content", **Then** they are prompted to sign up/log in first

---

### User Story 3 - Claude Code Subagent Integration for In-Book Intelligence (Priority: P3)

Users can interact with specialized AI agents embedded in the book that understand their learning context. The agents use Claude Code Agent SDK patterns to provide contextual help, code explanations, and personalized guidance.

**Why this priority**: This extends the existing RAG chatbot with more sophisticated intelligence. It's an enhancement that depends on core functionality (P1, P2) being stable.

**Independent Test**: Can be fully tested by opening the chatbot, asking a context-specific question about the current chapter, and verifying the response uses the Claude Agent SDK patterns for structured reasoning.

**Acceptance Scenarios**:

1. **Given** a user on a ROS 2 chapter with the chatbot open, **When** they ask "Explain this code example for me", **Then** the subagent analyzes the code contextually and provides a personalized explanation based on user background
2. **Given** a user with "Python experience" in their profile, **When** they ask about C++ code, **Then** the agent provides Python comparisons and analogies
3. **Given** any chapter, **When** a user requests "Generate practice exercises", **Then** the agent creates exercises appropriate to their expertise level

---

### User Story 4 - Agent Skills for Reusable Intelligence (Priority: P4)

The system provides reusable "Agent Skills" that users can invoke for common tasks: code explanation, concept simplification, prerequisite identification, and practice problem generation.

**Why this priority**: Skills are an optimization layer on top of the subagent system (P3). They package common patterns for efficiency and consistency.

**Independent Test**: Can be fully tested by invoking the "Explain Code" skill on any code block and verifying it returns a structured explanation.

**Acceptance Scenarios**:

1. **Given** any code block in a chapter, **When** a user hovers and clicks "Explain", **Then** the "code-explainer" skill activates and provides a step-by-step breakdown
2. **Given** a technical concept with jargon, **When** a user selects text and clicks "Simplify", **Then** the "concept-simplifier" skill rewrites it at the user's level
3. **Given** a chapter, **When** a user clicks "What should I know first?", **Then** the "prerequisite-finder" skill lists required knowledge with links

---

### Edge Cases

- What happens when a user's session expires mid-personalization? (Re-authenticate without losing work)
- How does the system handle personalization for users who skip the background questionnaire? (Default to intermediate level with gentle prompts to complete profile)
- What happens if the LLM rate limit is exceeded during personalization? (Queue requests, show progress indicator, allow cancellation)
- How does personalization handle code blocks and diagrams? (Preserve technical accuracy, only adapt explanatory text)
- What if Better Auth provider is unavailable? (Graceful degradation with local session, prompt to complete auth later)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST integrate Better Auth for user authentication (email/password at minimum)
- **FR-002**: System MUST present a 3-5 question background questionnaire after initial registration
- **FR-003**: Users MUST be able to skip the questionnaire but receive prompts to complete it later
- **FR-004**: System MUST store user profiles with expertise level (beginner/intermediate/expert), programming languages known, and learning goals
- **FR-005**: System MUST display a "Personalize Content" button on every chapter page for logged-in users
- **FR-006**: System MUST use Gemini API (or configured LLM) to rewrite chapter content based on user profile
- **FR-007**: System MUST cache personalized content per chapter per user profile hash to avoid redundant LLM calls
- **FR-008**: System MUST allow users to toggle between personalized and original content
- **FR-009**: System MUST preserve all code blocks, diagrams, and technical accuracy during personalization
- **FR-010**: System MUST integrate Claude Code Agent SDK patterns for structured chatbot responses
- **FR-011**: System MUST provide at least 3 reusable Agent Skills: code-explainer, concept-simplifier, prerequisite-finder
- **FR-012**: System MUST rate-limit personalization requests per user (5 per minute default)

### Key Entities

- **User**: Authenticated user with email, password hash, session tokens, and profile data
- **UserProfile**: Expertise level, programming languages, learning goals, created_at, updated_at
- **PersonalizedContent**: Chapter ID, user profile hash, personalized HTML/markdown, cached_at, expires_at
- **AgentSkill**: Skill ID, name, description, prompt template, input schema, output schema
- **SkillInvocation**: User ID, skill ID, input, output, latency_ms, timestamp

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can complete signup and background questionnaire in under 3 minutes
- **SC-002**: Personalized content is generated within 10 seconds for cached profiles, 30 seconds for new
- **SC-003**: 80% of users who complete background questionnaire use personalization at least once
- **SC-004**: Agent Skills respond within 5 seconds for standard queries
- **SC-005**: System maintains 99% uptime for authentication services
- **SC-006**: Personalization accuracy: 90% of users report content matches their expected level (survey metric)
- **SC-007**: Zero authentication data leaks or security vulnerabilities in penetration testing

## Technical Context

### Existing Architecture

- **Frontend**: Docusaurus 3 with React components (TranslationToggle, ChatbotWidget already exist)
- **Backend**: FastAPI on Python 3.13 with async support
- **Database**: Neon Serverless Postgres (for metadata)
- **Vector DB**: Qdrant Cloud (for RAG chatbot)
- **Current Auth**: None (session-based anonymous IDs only)
- **LLM**: Google Gemini API (already configured for chatbot and translation)

### Integration Points

1. **Better Auth**: New authentication layer, likely using @better-auth/core package
2. **Claude Agent SDK**: For subagent patterns (optional enhancement, can start with simpler prompt engineering)
3. **Personalization Service**: New FastAPI service endpoint for content adaptation
4. **User Profile Storage**: New Postgres tables for users and profiles
5. **Skill Registry**: Configuration-driven skill definitions

### Out of Scope (for this feature)

- Social login (OAuth with Google, GitHub) - future enhancement
- Real-time collaborative features
- User-generated content or annotations
- Payment or subscription features
