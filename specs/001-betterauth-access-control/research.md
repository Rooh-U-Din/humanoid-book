# Research: BetterAuth Strict Access Control

**Branch**: `001-betterauth-access-control`
**Date**: 2025-12-13
**Purpose**: Resolve technical unknowns and document integration patterns

## Executive Summary

BetterAuth is a framework-agnostic TypeScript authentication library that provides email/password authentication with built-in session management. Key findings:

1. **Client-Side SDK**: Uses `createAuthClient` from `better-auth/react` for React/Docusaurus integration
2. **Server Integration**: BetterAuth is primarily a Node.js library; FastAPI integration requires custom session validation
3. **Session Management**: Cookie-based with configurable expiration (7 days default) and refresh intervals (1 day default)

---

## Research Topics

### 1. BetterAuth + Docusaurus Integration

**Decision**: Use BetterAuth client SDK (`better-auth/react`) in Docusaurus React components

**Rationale**:
- BetterAuth provides `createAuthClient` specifically for React
- `useSession` hook provides reactive session state
- `signUp.email` and `signIn.email` methods handle auth flows
- Docusaurus is React-based, so client SDK integrates naturally

**Alternatives Considered**:
- NextAuth.js: More opinionated, designed for Next.js
- Auth0: External service, adds cost and complexity
- Custom JWT: More work, less battle-tested

**Implementation Pattern**:
```typescript
// src/lib/auth-client.ts
import { createAuthClient } from "better-auth/react"

export const authClient = createAuthClient({
  baseURL: process.env.BETTER_AUTH_URL // Backend URL
})

export const { signIn, signUp, signOut, useSession } = authClient
```

---

### 2. BetterAuth + FastAPI Backend Integration

**Decision**: Implement custom session validation in FastAPI that validates BetterAuth session cookies

**Rationale**:
- BetterAuth is a TypeScript/Node.js library - no native Python SDK
- BetterAuth uses signed cookies with predictable structure
- FastAPI can validate cookies using shared secret (BETTER_AUTH_SECRET)
- Session data stored in database accessible to both systems

**Alternatives Considered**:
- Node.js middleware proxy: Adds latency, complexity
- Separate auth service: Overkill for this use case
- JWT-only approach: Loses BetterAuth session management benefits

**Implementation Pattern**:
```python
# Backend validates BetterAuth session cookies
# Option 1: Decode cookie with shared secret
# Option 2: Call BetterAuth /api/auth/session endpoint
# Option 3: Share database and validate session directly

# Recommended: Option 3 - Direct database validation
from sqlalchemy import select
from models import Session, User

async def get_current_user(session_token: str, db: Session):
    session = await db.execute(
        select(Session).where(Session.token == session_token)
    )
    if not session or session.expires_at < datetime.utcnow():
        raise HTTPException(401, "Invalid session")
    return session.user
```

---

### 3. Email/Password Authentication Configuration

**Decision**: Use BetterAuth emailAndPassword with min 8 character password policy

**Rationale**:
- FR-008 requires minimum 8 characters
- BetterAuth supports `minPasswordLength` configuration
- Auto sign-in after signup improves UX (FR-004)
- Generic error messages prevent credential enumeration (FR-009)

**Configuration**:
```typescript
import { betterAuth } from "better-auth"

export const auth = betterAuth({
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,
    maxPasswordLength: 128,
    autoSignIn: true, // Auto login after signup
  }
})
```

---

### 4. Session Persistence and Cookie Configuration

**Decision**: Use 7-day session expiration with 1-day refresh, cookie caching enabled

**Rationale**:
- Spec assumption: "Standard session expiration of 7 days with 1-day refresh window"
- Cookie caching reduces database load
- Compact encoding strategy for smaller cookies

**Configuration**:
```typescript
export const auth = betterAuth({
  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days in seconds
    updateAge: 60 * 60 * 24, // Refresh every 1 day
    cookieCache: {
      enabled: true,
      maxAge: 5 * 60, // 5 minute cache
      strategy: "compact"
    }
  }
})
```

---

### 5. Content Protection Strategy (FR-001)

**Decision**: Client-side route guards + server-side API protection

**Rationale**:
- Docusaurus generates static content - cannot fully protect at build time
- Client-side guards provide UX (redirect to login)
- Server-side API protection is the security boundary
- Protected content fetched via authenticated API calls

**Implementation Pattern**:
```tsx
// Client-side protection wrapper
function ProtectedContent({ children }) {
  const { data: session, isPending } = useSession()

  if (isPending) return <Loading />
  if (!session) return <LoginPrompt />
  return children
}

// Wrap doc content
<ProtectedContent>
  <DocContent />
</ProtectedContent>
```

---

### 6. Database Schema for BetterAuth

**Decision**: Use BetterAuth's default schema with PostgreSQL (Neon)

**Rationale**:
- BetterAuth expects specific tables: user, session, account, verification
- Constitution specifies Neon Serverless Postgres
- Schema aligns with existing project database

**BetterAuth Required Tables**:
- `user`: id, name, email, emailVerified, image, createdAt, updatedAt
- `session`: id, userId, token, expiresAt, ipAddress, userAgent, createdAt, updatedAt
- `account`: id, userId, providerId, accountId, accessToken, refreshToken, etc.
- `verification`: id, identifier, value, expiresAt, createdAt, updatedAt

---

### 7. Background Questionnaire Flow (FR-018-020)

**Decision**: Post-signup callback triggers questionnaire modal

**Rationale**:
- BetterAuth supports `onSuccess` callback after signup
- Modal approach doesn't require page navigation
- Skip option allows content access (FR-019)
- Responses stored in UserProfile table

**Implementation Pattern**:
```typescript
await authClient.signUp.email({
  email, password, name
}, {
  onSuccess: (ctx) => {
    // Show questionnaire modal
    showQuestionnaireModal()
  }
})
```

---

## Technical Decisions Summary

| Topic | Decision | Confidence |
|-------|----------|------------|
| Client SDK | better-auth/react | High |
| Backend Integration | Custom FastAPI session validator | High |
| Password Policy | Min 8 chars, no complexity | High |
| Session Duration | 7 days expiry, 1 day refresh | High |
| Content Protection | Client guards + API auth | High |
| Database | Neon Postgres with BetterAuth schema | High |
| Questionnaire | Post-signup modal with skip | High |

---

## Dependencies

### Frontend (Docusaurus)
- `better-auth` - Core library
- `@better-auth/react` - React hooks and components

### Backend (FastAPI)
- `bcrypt` - Password hashing (if custom validation)
- `python-jose` - JWT handling (if cookie decoding needed)
- Existing: SQLAlchemy, Pydantic, FastAPI

---

## Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| BetterAuth Node.js only | Medium | Custom FastAPI session validator using shared DB |
| Cookie validation complexity | Low | Document cookie structure, test thoroughly |
| Content protection bypass | Medium | Server-side API is true security boundary |
| Session synchronization | Low | Single database source of truth |

---

## Open Questions (Resolved)

1. ~~How does FastAPI validate BetterAuth sessions?~~ → Direct database session lookup
2. ~~What cookie encoding does BetterAuth use?~~ → Compact strategy (signed, not encrypted)
3. ~~How to protect static Docusaurus content?~~ → Client-side guards + API protection

---

## References

- BetterAuth Documentation: https://www.better-auth.com/
- BetterAuth GitHub: https://github.com/better-auth/better-auth
- Constitution: `.specify/memory/constitution.md`
- Feature Spec: `specs/001-betterauth-access-control/spec.md`
