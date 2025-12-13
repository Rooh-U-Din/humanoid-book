# Quickstart: Intelligent Personalization & Authentication

**Feature**: intelligent-personalization-auth
**Date**: 2025-12-09 (Updated: 2025-12-12)

## Updates (2025-12-12)
- BetterAuth runs client-side, FastAPI validates cookies directly (FR-027)
- Added BETTER_AUTH_SECRET for cookie validation
- Added email verification setup (FR-026)
- Password policy: min 8 chars, no complexity (FR-025)

## Prerequisites

- Node.js 18+
- Python 3.13
- PostgreSQL (Neon account or local)
- Gemini API key (existing)
- SMTP server (for email verification)

## Quick Setup

### 1. Install Frontend Dependencies

```bash
cd D:/vsCode/CLI/hackathon/my_book

# Install Better Auth
npm install better-auth @better-auth/react
```

### 2. Install Backend Dependencies

```bash
cd backend

# Add new dependencies to requirements.txt
pip install bcrypt pyjwt

# Or with specific Python
"C:\Users\Fida\AppData\Local\Programs\Python\Python313\python.exe" -m pip install bcrypt pyjwt
```

### 3. Database Migration

Run the migration script against Neon Postgres:

```bash
# Set your Neon connection string
export DATABASE_URL="postgresql://user:pass@host/db"

# Run migration
psql $DATABASE_URL -f specs/intelligent-personalization-auth/migration.sql
```

Or use the Python script:

```python
# backend/src/migrations/001_auth_personalization.py
from sqlalchemy import create_engine, text
import os

engine = create_engine(os.getenv("DATABASE_URL"))

migration = """
-- Users table
CREATE TABLE IF NOT EXISTS users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    name VARCHAR(255),
    password_hash VARCHAR(255) NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_users_email ON users(email);

-- User profiles table
CREATE TABLE IF NOT EXISTS user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID UNIQUE REFERENCES users(id) ON DELETE CASCADE,
    expertise_level VARCHAR(20) DEFAULT 'intermediate',
    programming_languages TEXT[] DEFAULT '{}',
    learning_goals TEXT,
    questionnaire_responses JSONB,
    profile_completed BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Personalized content cache
CREATE TABLE IF NOT EXISTS personalized_content (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chapter_id VARCHAR(100) NOT NULL,
    user_id UUID REFERENCES users(id) ON DELETE SET NULL,
    profile_hash VARCHAR(16) NOT NULL,
    personalized_content TEXT NOT NULL,
    is_cached BOOLEAN DEFAULT TRUE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    expires_at TIMESTAMP WITH TIME ZONE
);

CREATE INDEX IF NOT EXISTS idx_personalized_chapter_hash
    ON personalized_content(chapter_id, profile_hash);
"""

with engine.connect() as conn:
    conn.execute(text(migration))
    conn.commit()
    print("Migration completed successfully!")
```

### 4. Environment Variables

Add to `backend/.env`:

```env
# Existing
GEMINI_API_KEY=your-key-here
DATABASE_URL=postgresql://...

# New for BetterAuth (Updated 2025-12-12)
BETTER_AUTH_SECRET=your-secret-key-here-min-32-chars
BETTER_AUTH_URL=http://localhost:3000
SESSION_EXPIRES_IN=604800   # 7 days in seconds
SESSION_UPDATE_AGE=86400    # 1 day in seconds

# Email verification (FR-026)
EMAIL_VERIFICATION_REQUIRED=true
SMTP_HOST=smtp.example.com
SMTP_PORT=587
SMTP_USER=your-smtp-user
SMTP_PASSWORD=your-smtp-password
SMTP_FROM=noreply@example.com
```

Generate a secure secret:
```python
import secrets
print(secrets.token_hex(32))
```

**Important**: The `BETTER_AUTH_SECRET` must be the same on both frontend (BetterAuth client) and backend (FastAPI cookie validation) to properly validate session cookies.

### 5. Start Development Servers

**Backend**:
```bash
cd backend/src
python -m uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

**Frontend**:
```bash
npm run start -- --port 3000
```

## Verification Steps

### 1. Check Database Tables

```sql
-- Connect to Neon and verify
SELECT table_name FROM information_schema.tables
WHERE table_schema = 'public'
AND table_name IN ('users', 'user_profiles', 'personalized_content');
```

### 2. Test Auth Endpoints

```bash
# Sign up
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"testpass123","name":"Test User"}'

# Sign in
curl -X POST http://localhost:8000/api/auth/signin \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"testpass123"}'
```

### 3. Test Profile Endpoints

```bash
# Get profile (with token from signin)
curl http://localhost:8000/api/profile \
  -H "Authorization: Bearer YOUR_JWT_TOKEN"

# Update profile
curl -X PUT http://localhost:8000/api/profile \
  -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"expertise_level":"beginner","programming_languages":["Python"]}'
```

### 4. Test Personalization

```bash
curl -X POST http://localhost:8000/api/personalize/chapter \
  -H "Authorization: Bearer YOUR_JWT_TOKEN" \
  -H "Content-Type: application/json" \
  -d '{"chapter_id":"ros2-intro","content":"ROS 2 is a robot operating system..."}'
```

## File Structure After Setup

```
backend/
├── src/
│   ├── models/
│   │   ├── database.py     # Updated with new tables
│   │   ├── auth.py         # NEW
│   │   └── personalization.py # NEW
│   ├── services/
│   │   ├── auth_service.py # NEW
│   │   ├── profile_service.py # NEW
│   │   └── personalization_service.py # NEW
│   └── api/
│       ├── auth_routes.py  # NEW
│       ├── profile_routes.py # NEW
│       └── personalization_routes.py # NEW

src/
├── lib/
│   └── auth-client.ts      # NEW - Better Auth client
├── components/
│   ├── Auth/               # NEW
│   │   ├── AuthProvider.tsx
│   │   ├── SignInModal.tsx
│   │   └── SignUpModal.tsx
│   └── Personalization/    # NEW
│       └── PersonalizeButton.tsx
```

## Common Issues

### 1. JWT Secret Not Set
```
Error: JWT_SECRET_KEY not configured
Solution: Add JWT_SECRET_KEY to .env (min 32 characters)
```

### 2. Database Connection Failed
```
Error: Connection refused
Solution: Check DATABASE_URL in .env, ensure Neon is accessible
```

### 3. Better Auth Import Error
```
Error: Cannot find module 'better-auth/react'
Solution: npm install better-auth @better-auth/react
```

### 4. CORS Issues
```
Error: CORS policy blocked
Solution: Ensure FastAPI CORS middleware allows localhost:3000
```

## BetterAuth Client Setup (Updated 2025-12-12)

Create `src/lib/auth-client.ts`:

```typescript
import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:3000",
  session: {
    expiresIn: 7 * 24 * 60 * 60, // 7 days
    updateAge: 24 * 60 * 60,     // 1 day
  },
  emailAndPassword: {
    enabled: true,
    minPasswordLength: 8,  // FR-025
    requireEmailVerification: true,  // FR-026
  },
});

// Export hooks
export const { useSession, signIn, signUp, signOut } = authClient;
```

## FastAPI Session Validation (Updated 2025-12-12)

Create `backend/src/services/session_validator.py`:

```python
import hmac
import base64
import json
import os
from datetime import datetime
from fastapi import HTTPException, Request, Depends

class BetterAuthSessionValidator:
    def __init__(self):
        self.secret = os.getenv("BETTER_AUTH_SECRET", "").encode()
        if len(self.secret) < 32:
            raise ValueError("BETTER_AUTH_SECRET must be at least 32 characters")

    async def validate_cookie(self, request: Request) -> dict:
        cookie = request.cookies.get("better-auth.session")
        if not cookie:
            raise HTTPException(status_code=401, detail="No session cookie")

        try:
            payload_b64, signature_b64 = cookie.rsplit('.', 1)
            payload_bytes = base64.urlsafe_b64decode(payload_b64 + '==')
            signature = base64.urlsafe_b64decode(signature_b64 + '==')

            expected = hmac.new(self.secret, payload_bytes, 'sha256').digest()
            if not hmac.compare_digest(signature, expected):
                raise HTTPException(status_code=401, detail="Invalid session signature")

            session = json.loads(payload_bytes)

            if datetime.fromisoformat(session['expiresAt'].replace('Z', '+00:00')) < datetime.now():
                raise HTTPException(status_code=401, detail="Session expired")

            return session
        except HTTPException:
            raise
        except Exception as e:
            raise HTTPException(status_code=401, detail=f"Invalid session: {e}")

# Dependency for protected routes
validator = BetterAuthSessionValidator()

async def get_session(request: Request) -> dict:
    return await validator.validate_cookie(request)

async def require_verified_email(session: dict = Depends(get_session)) -> dict:
    """Dependency that requires email verification (FR-026)"""
    if not session.get('user', {}).get('emailVerified'):
        raise HTTPException(
            status_code=403,
            detail="Email verification required for this feature"
        )
    return session
```

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks
2. Implement Phase 1 (BetterAuth Setup) first
3. Implement Phase 2 (Session & Cookie Validation)
4. Implement Phase 3 (Email Verification)
5. Implement Phase 4 (User Profiles)
6. Implement Phase 5 (Personalization)
7. Implement Phase 6 (Agent Skills)
