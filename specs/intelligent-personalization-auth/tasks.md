# Tasks: Intelligent Personalization & Authentication

**Input**: Design documents from `/specs/intelligent-personalization-auth/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, quickstart.md

**Tests**: NOT included (no explicit test request in feature specification)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/src/` (FastAPI)
- **Frontend**: `src/` (Docusaurus/React)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and dependency installation

- [ ] T001 Install backend dependencies (bcrypt, pyjwt) in backend/requirements.txt
- [ ] T002 [P] Install frontend dependencies (better-auth, @better-auth/react) via npm
- [ ] T003 [P] Add JWT environment variables to backend/.env (JWT_SECRET_KEY, JWT_ALGORITHM, JWT_EXPIRATION_HOURS)
- [ ] T004 [P] Create auth client configuration in src/lib/auth-client.ts

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**CRITICAL**: No user story work can begin until this phase is complete

- [ ] T005 Add User and UserProfile SQLAlchemy models to backend/src/models/database.py
- [ ] T006 Create database migration script in backend/src/migrations/001_auth_personalization.py
- [ ] T007 Run database migration against Neon Postgres
- [ ] T008 [P] Create Pydantic models for auth in backend/src/models/auth.py
- [ ] T009 [P] Create Pydantic models for personalization in backend/src/models/personalization.py
- [ ] T010 Implement JWT validation middleware in backend/src/services/auth_service.py
- [ ] T011 Register new API routes in backend/src/main.py

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - New User Onboarding with Background Collection (Priority: P1) MVP

**Goal**: Users can sign up, sign in, and complete a background questionnaire to set their expertise profile

**Independent Test**: Create a new account, complete the background questionnaire, log out, log back in, and verify profile data persists

### Implementation for User Story 1

#### Backend (Auth & Profile)

- [ ] T012 [US1] Implement signup endpoint (POST /api/auth/signup) in backend/src/api/auth_routes.py
- [ ] T013 [US1] Implement signin endpoint (POST /api/auth/signin) in backend/src/api/auth_routes.py
- [ ] T014 [US1] Implement signout endpoint (POST /api/auth/signout) in backend/src/api/auth_routes.py
- [ ] T015 [US1] Implement get current user endpoint (GET /api/auth/me) in backend/src/api/auth_routes.py
- [ ] T016 [US1] Implement profile_service.py with CRUD operations in backend/src/services/profile_service.py
- [ ] T017 [US1] Implement get profile endpoint (GET /api/profile) in backend/src/api/profile_routes.py
- [ ] T018 [US1] Implement update profile endpoint (PUT /api/profile) in backend/src/api/profile_routes.py
- [ ] T019 [US1] Implement questionnaire submission endpoint (POST /api/profile/questionnaire) in backend/src/api/profile_routes.py

#### Frontend (Auth Components)

- [ ] T020 [P] [US1] Create AuthProvider context in src/components/Auth/AuthProvider.tsx
- [ ] T021 [P] [US1] Create SignInModal component in src/components/Auth/SignInModal.tsx
- [ ] T022 [P] [US1] Create SignUpModal component in src/components/Auth/SignUpModal.tsx
- [ ] T023 [US1] Create BackgroundQuestionnaire component in src/components/Auth/BackgroundQuestionnaire.tsx
- [ ] T024 [US1] Create UserProfileBadge component (shows logged-in status) in src/components/Auth/UserProfileBadge.tsx
- [ ] T025 [US1] Wrap app with AuthProvider in src/theme/Root.tsx
- [ ] T026 [US1] Add sign in/sign up buttons to navbar in src/theme/NavbarItem/AuthNavbarItem.tsx

**Checkpoint**: Users can sign up, complete questionnaire, sign in, and see their profile. Test independently before proceeding.

---

## Phase 4: User Story 2 - Personalize Chapter Content Button (Priority: P2)

**Goal**: Logged-in users can click a button to get chapter content adapted to their expertise level

**Independent Test**: Log in as beginner user, click "Personalize Content" on any chapter, verify content is simplified with more explanations

### Implementation for User Story 2

#### Backend (Personalization Service)

- [ ] T027 [US2] Implement personalization_service.py with Gemini integration in backend/src/services/personalization_service.py
- [ ] T028 [US2] Implement profile hash generation for caching in backend/src/services/personalization_service.py
- [ ] T029 [US2] Add PersonalizedContent SQLAlchemy model to backend/src/models/database.py
- [ ] T030 [US2] Implement personalize chapter endpoint (POST /api/personalize/chapter) in backend/src/api/personalization_routes.py
- [ ] T031 [US2] Implement get cached personalization endpoint (GET /api/personalize/chapter/:id) in backend/src/api/personalization_routes.py
- [ ] T032 [US2] Add rate limiting (5 req/min) to personalization endpoints in backend/src/api/personalization_routes.py

#### Frontend (Personalization Components)

- [ ] T033 [P] [US2] Create PersonalizeButton component in src/components/Personalization/PersonalizeButton.tsx
- [ ] T034 [P] [US2] Create PersonalizedContent wrapper component in src/components/Personalization/PersonalizedContent.tsx
- [ ] T035 [US2] Create toggle between original/personalized content in src/components/Personalization/ContentToggle.tsx
- [ ] T036 [US2] Add PersonalizeButton to DocPage layout in src/theme/DocItem/Layout/index.tsx
- [ ] T037 [US2] Add loading state and progress indicator for personalization in src/components/Personalization/PersonalizeButton.tsx
- [ ] T038 [US2] Handle unauthenticated users (prompt to sign in) in src/components/Personalization/PersonalizeButton.tsx

**Checkpoint**: Users can personalize chapter content. Beginner gets simpler explanations, expert gets advanced details. Test independently.

---

## Phase 5: User Story 3 - Claude Code Subagent Integration (Priority: P3)

**Goal**: Users can interact with AI agents in the chatbot that provide contextual, personalized help

**Independent Test**: Open chatbot on a ROS 2 chapter, ask "Explain this code for me", verify response uses user's profile for personalization

### Implementation for User Story 3

#### Backend (Enhanced Chatbot)

- [ ] T039 [US3] Extend chatbot service to include user profile context in backend/src/services/chatbot_service.py
- [ ] T040 [US3] Add personalized prompt templates based on expertise level in backend/src/services/chatbot_service.py
- [ ] T041 [US3] Implement chapter context injection for code explanations in backend/src/services/chatbot_service.py
- [ ] T042 [US3] Add Python comparison prompts for users with Python background in backend/src/services/chatbot_service.py

#### Frontend (Chatbot Enhancement)

- [ ] T043 [US3] Update ChatbotWidget to pass user profile to API in src/components/ChatbotWidget/ChatbotWidget.tsx
- [ ] T044 [US3] Add "Explain this code" quick action to chatbot in src/components/ChatbotWidget/QuickActions.tsx
- [ ] T045 [US3] Add "Generate practice exercises" quick action in src/components/ChatbotWidget/QuickActions.tsx

**Checkpoint**: Chatbot provides personalized responses based on user expertise. Test independently.

---

## Phase 6: User Story 4 - Agent Skills for Reusable Intelligence (Priority: P4)

**Goal**: Users can invoke specialized AI skills (code-explainer, concept-simplifier, prerequisite-finder) from the UI

**Independent Test**: Select any code block, click "Explain", verify the code-explainer skill returns a step-by-step breakdown

### Implementation for User Story 4

#### Backend (Skills Service)

- [ ] T046 [US4] Create skills registry with prompt templates in backend/src/services/skills_service.py
- [ ] T047 [US4] Implement code-explainer skill in backend/src/services/skills_service.py
- [ ] T048 [US4] Implement concept-simplifier skill in backend/src/services/skills_service.py
- [ ] T049 [US4] Implement prerequisite-finder skill in backend/src/services/skills_service.py
- [ ] T050 [US4] Add SkillInvocation logging to database in backend/src/services/skills_service.py
- [ ] T051 [US4] Implement invoke skill endpoint (POST /api/skills/invoke) in backend/src/api/personalization_routes.py
- [ ] T052 [US4] Implement list skills endpoint (GET /api/skills) in backend/src/api/personalization_routes.py

#### Frontend (Skills UI)

- [ ] T053 [P] [US4] Create SkillsPanel component in src/components/AgentSkills/SkillsPanel.tsx
- [ ] T054 [P] [US4] Create CodeExplainer overlay for code blocks in src/components/AgentSkills/CodeExplainer.tsx
- [ ] T055 [US4] Create ConceptSimplifier for text selection in src/components/AgentSkills/ConceptSimplifier.tsx
- [ ] T056 [US4] Create PrerequisiteFinder button for chapters in src/components/AgentSkills/PrerequisiteFinder.tsx
- [ ] T057 [US4] Add hover actions to code blocks (Explain button) in src/theme/CodeBlock/index.tsx
- [ ] T058 [US4] Add "What should I know first?" button to chapter header in src/theme/DocItem/Layout/index.tsx

**Checkpoint**: All three skills are functional and accessible from the UI. Test independently.

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T059 [P] Add error handling for Gemini API rate limits across all personalization features
- [ ] T060 [P] Add graceful degradation when Better Auth is unavailable
- [ ] T061 [P] Handle session expiry during personalization (re-authenticate without losing work)
- [ ] T062 Add default "intermediate" expertise for users who skip questionnaire
- [ ] T063 [P] Add CSS styles for all new components in src/components/Auth/Auth.module.css
- [ ] T064 [P] Add CSS styles for personalization components in src/components/Personalization/Personalization.module.css
- [ ] T065 [P] Add CSS styles for skills components in src/components/AgentSkills/AgentSkills.module.css
- [ ] T066 Validate all endpoints against quickstart.md verification steps
- [ ] T067 Update CORS configuration to allow frontend origin

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational - Authentication is required for all other stories
- **User Story 2 (Phase 4)**: Depends on User Story 1 (needs user profile for personalization)
- **User Story 3 (Phase 5)**: Depends on User Story 1 (needs user profile for context)
- **User Story 4 (Phase 6)**: Depends on User Story 1 (needs user profile for expertise level)
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Required first - all other stories depend on authentication
- **User Story 2 (P2)**: Can start after US1 complete - needs profile data
- **User Story 3 (P3)**: Can start after US1 complete - independent of US2
- **User Story 4 (P4)**: Can start after US1 complete - independent of US2, US3

### Within Each User Story

- Backend models before services
- Backend services before API routes
- API routes before frontend integration
- Core implementation before UX polish

### Parallel Opportunities

**Phase 1 (Setup)**:
```
T002, T003, T004 can run in parallel (different files)
```

**Phase 2 (Foundational)**:
```
T008, T009 can run in parallel (different Pydantic model files)
```

**User Story 1 (Phase 3)**:
```
T020, T021, T022 can run in parallel (different React components)
```

**User Story 2 (Phase 4)**:
```
T033, T034 can run in parallel (different React components)
```

**User Story 4 (Phase 6)**:
```
T053, T054 can run in parallel (different React components)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test signup, signin, questionnaire, profile independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational -> Foundation ready
2. Add User Story 1 -> Test independently -> Deploy/Demo (MVP!)
3. Add User Story 2 -> Test independently -> Deploy/Demo (Personalization!)
4. Add User Story 3 -> Test independently -> Deploy/Demo (Smart Chatbot!)
5. Add User Story 4 -> Test independently -> Deploy/Demo (Skills!)

---

## Summary

| Phase | Description | Task Count |
|-------|-------------|------------|
| Phase 1 | Setup | 4 |
| Phase 2 | Foundational | 7 |
| Phase 3 | User Story 1 (Auth + Profile) | 15 |
| Phase 4 | User Story 2 (Personalization) | 12 |
| Phase 5 | User Story 3 (Subagent Integration) | 7 |
| Phase 6 | User Story 4 (Agent Skills) | 13 |
| Phase 7 | Polish | 9 |
| **Total** | | **67** |

**Parallel Opportunities**: 14 tasks marked with [P]
**MVP Scope**: Phases 1-3 (26 tasks)
**Suggested First Increment**: User Story 1 completes a full auth + profile flow

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
