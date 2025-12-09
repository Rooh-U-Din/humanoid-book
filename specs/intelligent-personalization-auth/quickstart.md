# Quickstart: Intelligent Personalization & Authentication

**Feature**: intelligent-personalization-auth
**Date**: 2025-12-09

## Prerequisites

- Node.js 18+
- Python 3.13
- PostgreSQL (Neon account or local)
- Gemini API key (existing)

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

# New for auth
JWT_SECRET_KEY=your-secret-key-here-min-32-chars
JWT_ALGORITHM=HS256
JWT_EXPIRATION_HOURS=24
```

Generate a secure secret:
```python
import secrets
print(secrets.token_hex(32))
```

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

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks
2. Implement Phase 1 (Authentication) first
3. Test signup/signin flow
4. Implement Phase 2 (Profiles)
5. Implement Phase 3 (Personalization)
