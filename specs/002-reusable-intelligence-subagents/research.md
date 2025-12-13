# Research: Reusable Intelligence with Subagents

**Feature**: 002-reusable-intelligence-subagents
**Date**: 2025-12-13
**Status**: Complete

## Research Questions Resolved

### RQ-001: How to implement a skill registry with dynamic loading?

**Decision**: Use a Python dictionary-based registry with class-based skill implementations following a common interface (abstract base class).

**Rationale**:
- Simple and Pythonic approach that works with FastAPI's dependency injection
- Skills can be registered at startup without runtime dynamic loading (no restart required for pre-registered skills)
- Extensible via configuration file for enabling/disabling skills
- Aligns with existing service pattern in codebase (gemini_service, response_service, etc.)

**Alternatives Considered**:
1. Plugin-based loading (importlib.import_module): Overkill for current scale, adds complexity
2. Database-driven skill registry: Unnecessary for 5-10 skills, adds latency
3. Decorator-based auto-registration: Less explicit, harder to debug

### RQ-002: How to route requests to appropriate subagents?

**Decision**: Implement an AgentOrchestrator service that receives skill invocation requests and delegates to the appropriate skill handler based on skill_id.

**Rationale**:
- Single entry point for all skill invocations simplifies authentication/logging
- Orchestrator owns context building (chapter, user profile, selected text)
- Each skill handler is a focused class with single responsibility
- Matches existing pattern where routes delegate to services

**Alternatives Considered**:
1. Direct skill endpoint per skill type: Proliferates endpoints, harder to maintain auth
2. Message queue with workers: Over-engineered for synchronous web requests
3. GraphQL with resolvers: Different paradigm than existing REST API

### RQ-003: How to pass context to skills (chapter, selection, user profile)?

**Decision**: Create an `AgentContext` dataclass that aggregates all contextual information and is passed to every skill invocation.

**Rationale**:
- Single source of truth for context
- Easy to extend with new context fields
- Skills declare which context they need
- Testable - can mock context easily

**Context Structure**:
```python
@dataclass
class AgentContext:
    user_id: str
    user_email: str
    chapter_id: Optional[str]
    chapter_title: Optional[str]
    selected_text: Optional[str]
    user_profile: Optional[UserProfileData]
    trace_id: str
    timestamp: datetime
```

### RQ-004: How to implement logging and tracing?

**Decision**: Use structured logging with unique trace IDs for each skill invocation. Store invocations in `skill_invocations` table (already exists).

**Rationale**:
- Trace ID enables correlation across distributed logs
- Database storage enables analytics and debugging
- Structured logging works with existing Python logging
- `skill_invocations` table already has required schema

**Implementation**:
- Generate UUID trace_id at orchestrator level
- Log: timestamp, user_id, skill_id, input_hash, duration_ms, status, trace_id
- Include trace_id in response for client debugging
- Error logs include full stack trace (server-only)

### RQ-005: What AI service to use for skill execution?

**Decision**: Use existing GeminiService with skill-specific prompts. Each skill has a prompt template optimized for its task.

**Rationale**:
- Gemini already integrated and working (gemini_service.py)
- Free tier sufficient for expected usage
- Prompt engineering per skill provides best results
- No additional API dependencies

**Prompt Strategy Per Skill**:
| Skill | Prompt Focus |
|-------|-------------|
| explain | Code analysis + plain language explanation |
| translate | Urdu translation preserving technical terms |
| debug | Error analysis + fix suggestions |
| navigate | Related content discovery from book structure |
| personalize | Adapt explanation based on expertise level |

### RQ-006: How to handle skill versioning?

**Decision**: Use semantic versioning strings stored in skill metadata. Orchestrator validates compatibility before execution.

**Rationale**:
- Simple version strings (1.0.0) understood by all stakeholders
- Backward compatibility via major version matching
- Version included in skill_invocations for debugging
- No runtime version negotiation (server decides)

### RQ-007: Frontend skill invocation UX pattern?

**Decision**: Create a SkillPanel component that appears on text selection or via toolbar button. Uses existing AuthenticatedChatbot pattern for auth gating.

**Rationale**:
- Consistent with existing chatbot UI pattern
- Selection-based invocation natural for code explanation
- Toolbar provides discovery of available skills
- Reuses AuthModal for unauthenticated users

## Technical Decisions Summary

| Decision | Choice | Key Reason |
|----------|--------|------------|
| TD-001 | Dictionary-based skill registry | Simple, explicit, testable |
| TD-002 | AgentOrchestrator service | Single entry point, centralized auth/logging |
| TD-003 | AgentContext dataclass | Clean context passing, extensible |
| TD-004 | UUID trace IDs + DB logging | Debugging, analytics |
| TD-005 | GeminiService with skill prompts | Already integrated, free tier |
| TD-006 | Semantic versioning strings | Simple, understood |
| TD-007 | SkillPanel component | Consistent UX, auth-gated |

## Integration Points

### With Existing Systems

1. **Authentication (001-betterauth)**:
   - Use `get_session` dependency for all skill routes
   - Access `user_id` from session for logging
   - Access `UserProfile` for personalization skill

2. **Chatbot API**:
   - Extend `/api/skills/*` namespace alongside `/api/query`
   - Reuse `GeminiService` for AI responses
   - Follow same response patterns

3. **Translation Service**:
   - Existing `translation_service.py` can be wrapped as a skill
   - Provides Urdu translation capability

4. **Personalization**:
   - Existing `personalization_service.py` provides profile-based adaptation
   - Wrap as skill with simpler interface

## Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                    Frontend (React)                      │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐  │
│  │ SkillPanel  │  │ SkillButton │  │SkillResultView  │  │
│  └──────┬──────┘  └──────┬──────┘  └────────┬────────┘  │
└─────────┼────────────────┼──────────────────┼───────────┘
          │                │                  │
          ▼                ▼                  ▼
┌─────────────────────────────────────────────────────────┐
│                  API Layer (FastAPI)                     │
│  ┌──────────────────────────────────────────────────┐   │
│  │              /api/skills/* routes                 │   │
│  │  POST /invoke  GET /list  GET /history           │   │
│  └──────────────────────┬───────────────────────────┘   │
└─────────────────────────┼───────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────┐
│              AgentOrchestrator Service                   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │
│  │ SkillRegistry│  │ContextBuilder│  │ InvocationLog│   │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘   │
└─────────┼────────────────┼──────────────────┼───────────┘
          │                │                  │
          ▼                ▼                  ▼
┌─────────────────────────────────────────────────────────┐
│                   Skill Handlers                         │
│  ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ │
│  │Explain │ │Translate│ │ Debug  │ │Navigate│ │Personal│ │
│  └────┬───┘ └────┬───┘ └────┬───┘ └────┬───┘ └────┬───┘ │
└───────┼──────────┼──────────┼──────────┼──────────┼─────┘
        │          │          │          │          │
        ▼          ▼          ▼          ▼          ▼
┌─────────────────────────────────────────────────────────┐
│                  GeminiService (LLM)                     │
└─────────────────────────────────────────────────────────┘
```

## Next Steps

1. Generate data-model.md with entity definitions
2. Generate API contracts (OpenAPI)
3. Generate quickstart.md
4. Update plan.md with Phase 1 artifacts
