# Research: Intelligent Personalization & Authentication

**Feature**: intelligent-personalization-auth
**Date**: 2025-12-09
**Status**: Complete

## Research Areas

### 1. Better Auth Integration

**Question**: How to integrate Better Auth with Docusaurus React frontend?

**Research Findings**:

Better Auth is a TypeScript-first authentication library that works well with React applications.

**Decision**: Use `better-auth` with `@better-auth/react` for client-side auth
**Rationale**:
- Framework-agnostic, works with any React app including Docusaurus
- Built-in session management with cookie-based tokens
- TypeScript support out of the box
- MIT licensed (free tier compatible)

**Implementation Pattern**:
```typescript
// Client setup - src/lib/auth-client.ts
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: "http://localhost:8000", // FastAPI backend
});

// Usage in components
import { authClient } from "../lib/auth-client";

// Sign up
await authClient.signUp.email({
  email,
  password,
  name,
});

// Sign in
await authClient.signIn.email({
  email,
  password,
});
```

**Alternatives Considered**:
- **NextAuth/Auth.js**: Requires Next.js, not suitable for Docusaurus
- **Firebase Auth**: External dependency, adds complexity
- **Custom JWT**: More work, reinventing the wheel
- **Supabase Auth**: External service, overkill for this use case

---

### 2. Backend Authentication Strategy

**Question**: How should the FastAPI backend handle auth tokens from Better Auth?

**Research Findings**:

Better Auth uses JWT tokens stored in cookies. The backend needs to:
1. Validate JWT tokens on protected routes
2. Store user data in Postgres
3. Manage sessions

**Decision**: Stateless JWT validation with user table sync
**Rationale**:
- No session table needed in Postgres (Better Auth handles sessions)
- User table stores profile data only
- JWT validation is fast and stateless

**Implementation Pattern**:
```python
# backend/src/services/auth_service.py
from fastapi import HTTPException, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
import jwt

security = HTTPBearer()

async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security)
) -> dict:
    token = credentials.credentials
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=["HS256"])
        return payload
    except jwt.InvalidTokenError:
        raise HTTPException(status_code=401, detail="Invalid token")
```

**Alternatives Considered**:
- **Session-based auth**: Requires session storage, more state management
- **OAuth proxy**: Adds latency, external dependency

---

### 3. User Profile Storage

**Question**: What's the best schema for storing user expertise profiles?

**Research Findings**:

User profiles need to store:
- Expertise level (enum: beginner/intermediate/expert)
- Known programming languages (array)
- Learning goals (text)
- Questionnaire responses (JSON)

**Decision**: Single UserProfile table with JSON column for flexible questionnaire data
**Rationale**:
- Simple schema, easy to extend
- JSON allows flexible questionnaire evolution
- Enum for expertise level enables fast filtering

**Schema Design**:
```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    name VARCHAR(255),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE TABLE user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    expertise_level VARCHAR(20) DEFAULT 'intermediate',
    programming_languages TEXT[],
    learning_goals TEXT,
    questionnaire_responses JSONB,
    profile_completed BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX idx_user_profiles_user_id ON user_profiles(user_id);
```

**Alternatives Considered**:
- **Separate tables per question**: Too normalized, complex queries
- **Redis for profiles**: Overkill, Postgres is sufficient
- **Local storage only**: No cross-device persistence

---

### 4. Content Personalization Strategy

**Question**: How to efficiently personalize chapter content using Gemini API?

**Research Findings**:

Options for personalization:
1. **Full chapter rewrite**: Send entire chapter, get adapted version
2. **Paragraph-by-paragraph**: More granular but many API calls
3. **Template-based**: Pre-generate multiple versions

**Decision**: Full chapter rewrite with aggressive caching
**Rationale**:
- Single API call per chapter per profile type
- Cache by hash(chapter_id + profile_hash)
- Profile hash = MD5(expertise_level + languages)
- ~20 chapters × 3 expertise levels = 60 cached versions max

**Implementation Pattern**:
```python
# backend/src/services/personalization_service.py
import hashlib
from services.gemini_service import get_gemini_service

class PersonalizationService:
    def __init__(self):
        self._cache: Dict[str, str] = {}
        self.gemini = get_gemini_service()

    def get_profile_hash(self, profile: UserProfile) -> str:
        key = f"{profile.expertise_level}:{','.join(profile.programming_languages)}"
        return hashlib.md5(key.encode()).hexdigest()[:8]

    async def personalize_chapter(
        self,
        chapter_id: str,
        content: str,
        profile: UserProfile
    ) -> str:
        cache_key = f"{chapter_id}:{self.get_profile_hash(profile)}"

        if cache_key in self._cache:
            return self._cache[cache_key]

        prompt = self._build_personalization_prompt(content, profile)
        personalized = await self.gemini.generate(prompt)

        self._cache[cache_key] = personalized
        return personalized
```

**Prompt Template**:
```
You are adapting educational content about robotics for a specific reader.

READER PROFILE:
- Expertise level: {expertise_level}
- Known languages: {programming_languages}
- Learning goals: {learning_goals}

INSTRUCTIONS:
- For "beginner": Add more context, explain jargon, include prerequisites
- For "intermediate": Balance explanation with depth
- For "expert": Skip basics, focus on advanced details and edge cases
- If reader knows Python but content has C++, add Python analogies
- PRESERVE ALL CODE BLOCKS EXACTLY AS-IS (do not modify code)
- PRESERVE ALL DIAGRAMS AND IMAGES
- Maintain technical accuracy

ORIGINAL CONTENT:
{chapter_content}

PERSONALIZED CONTENT:
```

**Alternatives Considered**:
- **Multiple pre-generated versions**: Storage-heavy, less personalized
- **Real-time streaming**: More complex, marginal UX benefit
- **Hybrid approach**: Too complex for initial implementation

---

### 5. Agent Skills Architecture

**Question**: How to implement reusable AI skills for code explanation, simplification, etc.?

**Research Findings**:

Options:
1. **Claude Agent SDK**: Full agent framework with tools
2. **Prompt templates**: Simpler, Gemini-based
3. **Langchain**: Heavier, more dependencies

**Decision**: Prompt templates with skill registry
**Rationale**:
- Already have Gemini integration
- Skills are essentially specialized prompts
- Registry pattern allows easy extension
- Upgrade path to Claude SDK exists

**Skill Registry Design**:
```python
# backend/src/services/skills_service.py
from dataclasses import dataclass
from typing import Dict, Callable

@dataclass
class Skill:
    id: str
    name: str
    description: str
    prompt_template: str

SKILLS: Dict[str, Skill] = {
    "code-explainer": Skill(
        id="code-explainer",
        name="Explain Code",
        description="Step-by-step breakdown of code",
        prompt_template="""
Explain this code step by step for a {expertise_level} developer.
If they know {languages}, use analogies from those languages.

CODE:
```
{code}
```

EXPLANATION:
"""
    ),
    "concept-simplifier": Skill(
        id="concept-simplifier",
        name="Simplify Concept",
        description="Simplify technical jargon",
        prompt_template="""
Simplify this technical concept for a {expertise_level} reader:

CONCEPT:
{text}

SIMPLIFIED:
"""
    ),
    "prerequisite-finder": Skill(
        id="prerequisite-finder",
        name="Find Prerequisites",
        description="List required knowledge",
        prompt_template="""
What should a reader know before understanding this content?
Reader is at {expertise_level} level.

CONTENT:
{text}

PREREQUISITES:
1.
"""
    ),
}
```

**Alternatives Considered**:
- **Claude Agent SDK**: More powerful but adds dependency
- **Custom tool framework**: Reinventing the wheel
- **Per-skill microservices**: Overkill for 3-5 skills

---

## Research Summary

| Topic | Decision | Confidence |
|-------|----------|------------|
| Auth Library | Better Auth | High |
| Backend Auth | Stateless JWT | High |
| Profile Storage | Postgres with JSON | High |
| Personalization | Full chapter + cache | Medium |
| Agent Skills | Prompt templates | High |

## Open Questions

1. **Rate limiting**: Should we use database-based rate limiting or in-memory?
   - **Answer**: Start with in-memory, upgrade if needed

2. **Profile migration**: What if questionnaire changes?
   - **Answer**: JSONB allows schema evolution, add versioning field

3. **Caching persistence**: Memory cache lost on restart?
   - **Answer**: Acceptable for MVP, can add Redis later

## Next Steps

1. Create data-model.md with final schema
2. Create API contracts (OpenAPI spec)
3. Create quickstart.md for development setup
