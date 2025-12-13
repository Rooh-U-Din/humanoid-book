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

## Clarifications

### Session 2025-12-12

- Q: What password policy should be enforced for BetterAuth signup? → A: Basic (minimum 8 characters, no complexity requirements)
- Q: Is email verification required before users can access personalization features? → A: Required (must verify email before accessing personalization)
- Q: How should BetterAuth (TypeScript) integrate with FastAPI (Python) backend? → A: BetterAuth runs in Docusaurus (client-only), FastAPI validates session cookies directly

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

---

## Extended Feature: BetterAuth Authentication Integration

**Added**: 2025-12-12
**Reference**: https://www.better-auth.com/
**Status**: Draft (Appended Requirement)

### Overview

Integrate BetterAuth as the authentication framework for the Physical AI & Humanoid Robotics book platform. BetterAuth is a comprehensive TypeScript authentication framework that provides robust signup/signin flows, secure session management, and an extensible plugin ecosystem.

### User Scenarios & Testing (BetterAuth Extension)

#### User Story 5 - BetterAuth Email/Password Signup Flow (Priority: P1-A)

A new user creates an account using the BetterAuth email/password authentication system. The system validates credentials, creates a secure session, and transitions the user to the background questionnaire flow.

**Why this priority**: Core authentication is the foundation for all personalization features. BetterAuth provides enterprise-grade security with minimal configuration.

**Independent Test**: Can be fully tested by submitting the signup form with valid credentials and verifying a session is created, user record exists in the database, and the user is redirected to the questionnaire.

**Acceptance Scenarios**:

1. **Given** a new visitor on the signup page, **When** they enter email, password, and name, **Then** the BetterAuth `signUp.email` client method creates their account
2. **Given** a user submits the signup form, **When** credentials are valid, **Then** a session cookie is created and the user is automatically signed in (BetterAuth autoSignIn default)
3. **Given** a user with weak password, **When** they submit signup, **Then** the system rejects with clear validation feedback
4. **Given** an email already registered, **When** signup is attempted, **Then** the system displays an appropriate error without leaking user existence information

---

#### User Story 6 - BetterAuth Signin Flow (Priority: P1-B)

A returning user signs in using their email and password. The system validates credentials, creates/refreshes the session, and loads the user's profile and preferences.

**Why this priority**: Signin is essential for returning users to access their personalized content and continue their learning journey.

**Independent Test**: Can be fully tested by signing in with valid credentials and verifying session creation, profile data retrieval, and proper redirect to the last visited page or dashboard.

**Acceptance Scenarios**:

1. **Given** a registered user on the signin page, **When** they enter correct email/password, **Then** BetterAuth `signIn.email` authenticates them and creates a session
2. **Given** a user checks "Remember Me", **When** they sign in, **Then** the session persists beyond browser close (BetterAuth rememberMe option)
3. **Given** incorrect credentials, **When** signin is attempted, **Then** a generic "Invalid credentials" error is shown (no email enumeration)
4. **Given** a signed-in user, **When** they navigate to protected pages, **Then** the session is automatically validated via cookie

---

#### User Story 7 - Secure Session Handling (Priority: P1-C)

The system manages user sessions securely using BetterAuth's session management with cookie caching, automatic refresh, and proper expiration handling.

**Why this priority**: Session security directly impacts user data protection and system integrity.

**Independent Test**: Can be fully tested by creating a session, verifying cookie properties (httpOnly, secure, sameSite), checking automatic refresh behavior, and confirming proper session revocation on signout.

**Acceptance Scenarios**:

1. **Given** a new session is created, **When** the server responds, **Then** a signed session cookie is set with appropriate security flags
2. **Given** a session older than updateAge (default 1 day), **When** the user makes a request, **Then** the session is automatically refreshed
3. **Given** a session older than expiresIn (default 7 days), **When** the user makes a request, **Then** they are redirected to signin
4. **Given** a user signs out, **When** signout completes, **Then** the session is revoked server-side and cookie is cleared

---

#### User Story 8 - Structured Background Questions During Signup (Priority: P2-A)

After BetterAuth signup completes, the system presents new users with structured background questions to enable personalization. This integrates with BetterAuth's post-signup hooks.

**Why this priority**: Background collection is essential for personalization but depends on successful authentication first.

**Independent Test**: Can be fully tested by completing signup, verifying the questionnaire is presented, submitting answers, and confirming data is stored in the user profile.

**Acceptance Scenarios**:

1. **Given** a user completes BetterAuth signup, **When** `onSuccess` callback fires, **Then** the user is redirected to the background questionnaire page
2. **Given** a user on the questionnaire page, **When** they submit their responses, **Then** the profile data is stored and linked to their BetterAuth user ID
3. **Given** a user skips the questionnaire, **When** they access personalization features later, **Then** they are prompted to complete their profile
4. **Given** a returning user who never completed the questionnaire, **When** they sign in, **Then** they see a non-intrusive prompt to complete their profile

---

#### User Story 9 - Backend BetterAuth Integration (Priority: P1-D)

The FastAPI backend integrates with BetterAuth server utilities to validate sessions, manage users, and provide secure API endpoints.

**Why this priority**: Backend integration is essential for protecting API endpoints and managing user data securely.

**Independent Test**: Can be fully tested by making authenticated API requests and verifying session validation, user context extraction, and proper error responses for invalid sessions.

**Acceptance Scenarios**:

1. **Given** an authenticated request to a protected endpoint, **When** the server processes it, **Then** BetterAuth `auth.api.getSession()` validates the session from request headers/cookies
2. **Given** an unauthenticated request to a protected endpoint, **When** the server processes it, **Then** a 401 Unauthorized response is returned
3. **Given** a valid session, **When** the server needs user data, **Then** the user profile is retrievable via BetterAuth's user management APIs
4. **Given** a user requests account deletion, **When** processed, **Then** BetterAuth handles cascading session revocation

---

#### User Story 10 - Client-Side BetterAuth SDK Integration (Priority: P1-E)

The Docusaurus/React frontend integrates the BetterAuth client SDK to provide reactive authentication state, session management, and seamless auth flows.

**Why this priority**: Client integration provides the user-facing authentication experience and real-time auth state management.

**Independent Test**: Can be fully tested by verifying the `useSession` hook updates reactively, auth methods (`signUp.email`, `signIn.email`, `signOut`) function correctly, and protected routes redirect appropriately.

**Acceptance Scenarios**:

1. **Given** the application loads, **When** `useSession` hook initializes, **Then** it reflects the current authentication state (authenticated or not)
2. **Given** a user signs in successfully, **When** the auth state changes, **Then** all components using `useSession` update reactively
3. **Given** a protected page, **When** an unauthenticated user navigates to it, **Then** they are redirected to the signin page
4. **Given** an authenticated user, **When** they click signout, **Then** `authClient.signOut()` clears the session and updates UI state

---

#### User Story 11 - User Profile Handling Post-Authentication (Priority: P2-B)

Authenticated users can view and update their profile information, with changes persisted through BetterAuth's user management system and synchronized with the personalization profile.

**Why this priority**: Profile management enables users to maintain accurate information for personalization and account management.

**Independent Test**: Can be fully tested by loading the profile page, verifying current data display, making updates, and confirming persistence across sessions.

**Acceptance Scenarios**:

1. **Given** an authenticated user, **When** they access the profile page, **Then** their name, email, and expertise preferences are displayed
2. **Given** a user updates their profile, **When** they save changes, **Then** the updates are persisted and reflected immediately
3. **Given** a user updates their expertise level, **When** they next use personalization, **Then** content adapts to the new level
4. **Given** a user wants to change their password, **When** they submit the change request, **Then** BetterAuth handles secure password update with current password verification

---

### Edge Cases (BetterAuth Extension)

- What happens when BetterAuth service is unavailable? (Graceful degradation with cached session validation, queued auth operations)
- How does the system handle concurrent sessions across devices? (BetterAuth `listSessions` allows viewing and revoking active sessions)
- What if a user's session is revoked while they're actively using the site? (Real-time detection via periodic session validation, smooth re-authentication prompt)
- How are session cookies secured in different environments? (httpOnly, secure in production, sameSite=strict, proper domain scoping)
- What happens if cookie caching fails? (Fallback to database session validation, transparent to user)

### Functional Requirements (BetterAuth Extension)

- **FR-013**: System MUST integrate BetterAuth for all user authentication using the `@better-auth/core` package
- **FR-014**: System MUST implement email/password authentication via BetterAuth `signUp.email` and `signIn.email` methods
- **FR-015**: System MUST use BetterAuth's session management with secure cookie-based tokens
- **FR-016**: System MUST configure session expiration (expiresIn: 7 days) and refresh interval (updateAge: 1 day)
- **FR-017**: System MUST enable cookie caching with the "compact" encoding strategy for optimal performance
- **FR-018**: System MUST implement BetterAuth client SDK with `useSession` hook for reactive auth state in React components
- **FR-019**: Backend MUST validate sessions using `auth.api.getSession()` for all protected API endpoints
- **FR-020**: System MUST use BetterAuth's `onSuccess` callback to trigger the background questionnaire flow after signup
- **FR-021**: System MUST allow users to view and manage active sessions via BetterAuth's `listSessions` functionality
- **FR-022**: System MUST implement secure signout that revokes sessions both client-side and server-side
- **FR-023**: System MUST store user profile data linked to BetterAuth user IDs for seamless integration with personalization
- **FR-024**: System MUST handle authentication errors gracefully without leaking user existence information
- **FR-025**: System MUST enforce password policy: minimum 8 characters, no complexity requirements
- **FR-026**: System MUST require email verification before granting access to personalization features
- **FR-027**: BetterAuth MUST run client-side in Docusaurus; FastAPI MUST validate session cookies directly using shared secret

### Key Entities (BetterAuth Extension)

- **BetterAuthUser**: BetterAuth-managed user record with id, email, name, emailVerified, image, createdAt, updatedAt
- **BetterAuthSession**: Session record with id, token, userId, expiresAt, ipAddress, userAgent, createdAt, updatedAt
- **UserProfileLink**: Maps BetterAuth user ID to application-specific UserProfile entity for personalization data
- **SessionActivity**: Optional audit log of session events (created, refreshed, revoked) for security monitoring

### Success Criteria (BetterAuth Extension)

- **SC-008**: Users can complete BetterAuth signup (email/password) in under 30 seconds
- **SC-009**: Session validation latency remains under 50ms with cookie caching enabled
- **SC-010**: 100% of protected API endpoints correctly reject unauthenticated requests
- **SC-011**: Session refresh operates transparently without user-visible interruption
- **SC-012**: Signout fully revokes sessions with no residual authenticated state
- **SC-013**: Password reset flow completes in under 5 user interactions
- **SC-014**: System passes OWASP authentication security checklist (no credential enumeration, secure session handling, proper cookie flags)

### Compatibility with Existing Features

This BetterAuth integration is designed to seamlessly complement the existing personalization and agent intelligence features defined in this specification:

1. **Authentication Foundation**: BetterAuth provides the identity layer that FR-001 through FR-003 require for user recognition
2. **Profile Continuity**: BetterAuth user IDs link to UserProfile entities (FR-004) for persistent expertise tracking
3. **Personalization Trigger**: Authenticated sessions enable the "Personalize Content" button (FR-005) by providing user context
4. **Agent Context**: Claude Code Subagents (FR-010, FR-011) receive user profile data for context-aware responses
5. **Caching Alignment**: BetterAuth session caching works alongside content caching (FR-007) for optimized performance
