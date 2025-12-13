# Quickstart: BetterAuth Strict Access Control

**Branch**: `001-betterauth-access-control`
**Date**: 2025-12-13
**Time to Complete**: ~30 minutes

---

## Prerequisites

- Node.js 18+ installed
- Python 3.11+ installed
- Neon Postgres database (or local PostgreSQL)
- Access to the repository

---

## 1. Environment Setup

### 1.1 Clone and Install Dependencies

```bash
# Frontend (Docusaurus)
npm install better-auth @better-auth/react

# Backend (FastAPI)
cd backend
pip install bcrypt python-jose[cryptography]
```

### 1.2 Environment Variables

Create/update `.env` files:

**Frontend (.env or .env.local):**
```env
# BetterAuth Configuration
BETTER_AUTH_URL=http://localhost:8000
BETTER_AUTH_SECRET=your-secret-key-min-32-chars

# API URL
VITE_API_URL=http://localhost:8000
```

**Backend (.env):**
```env
# BetterAuth Configuration
BETTER_AUTH_SECRET=your-secret-key-min-32-chars  # Must match frontend

# Database
DATABASE_URL=postgresql://user:password@host:5432/database

# Session Configuration
SESSION_EXPIRES_IN=604800  # 7 days in seconds
SESSION_UPDATE_AGE=86400   # 1 day in seconds
```

---

## 2. Database Setup

### 2.1 Run Migration

```bash
# Using psql
psql $DATABASE_URL -f backend/src/migrations/001_betterauth_access_control.sql

# Or using Python script
cd backend
python -m src.migrations.run_migrations
```

### 2.2 Verify Tables

```sql
-- Check tables exist
SELECT table_name FROM information_schema.tables
WHERE table_schema = 'public'
AND table_name IN ('user', 'session', 'account', 'verification', 'user_profile');
```

---

## 3. Frontend Configuration

### 3.1 Create Auth Client

Create `src/lib/auth-client.ts`:

```typescript
import { createAuthClient } from "better-auth/react"

export const authClient = createAuthClient({
  baseURL: process.env.BETTER_AUTH_URL || "http://localhost:8000"
})

export const {
  signIn,
  signUp,
  signOut,
  useSession
} = authClient
```

### 3.2 Create Auth Provider

Create `src/components/Auth/AuthProvider.tsx`:

```tsx
import React, { createContext, useContext } from 'react'
import { useSession } from '@/lib/auth-client'

const AuthContext = createContext(null)

export function AuthProvider({ children }) {
  const session = useSession()

  return (
    <AuthContext.Provider value={session}>
      {children}
    </AuthContext.Provider>
  )
}

export const useAuth = () => useContext(AuthContext)
```

### 3.3 Wrap App with Provider

Update `src/theme/Root.tsx`:

```tsx
import { AuthProvider } from '@/components/Auth/AuthProvider'

export default function Root({ children }) {
  return (
    <AuthProvider>
      {children}
    </AuthProvider>
  )
}
```

---

## 4. Backend Configuration

### 4.1 Create Session Validator

Create `backend/src/services/session_validator.py`:

```python
from datetime import datetime
from fastapi import HTTPException, Depends, Cookie
from sqlalchemy.orm import Session
from typing import Optional

from ..models.database import get_db, User, SessionModel

async def get_current_user(
    session_token: Optional[str] = Cookie(None, alias="better-auth.session_token"),
    db: Session = Depends(get_db)
) -> User:
    """Validate BetterAuth session and return current user."""
    if not session_token:
        raise HTTPException(status_code=401, detail="Not authenticated")

    # Find session in database
    session = db.query(SessionModel).filter(
        SessionModel.token == session_token,
        SessionModel.expires_at > datetime.utcnow()
    ).first()

    if not session:
        raise HTTPException(status_code=401, detail="Invalid or expired session")

    user = db.query(User).filter(User.id == session.user_id).first()
    if not user:
        raise HTTPException(status_code=401, detail="User not found")

    return user

# Optional: Require authenticated user
def require_auth(current_user: User = Depends(get_current_user)):
    return current_user
```

### 4.2 Create Auth Routes

Create `backend/src/api/auth_routes.py`:

```python
from fastapi import APIRouter, Depends, HTTPException, Response
from sqlalchemy.orm import Session
import bcrypt
import secrets

from ..models.database import get_db, User, SessionModel
from ..models.auth import SignUpRequest, SignInRequest, AuthResponse
from ..services.session_validator import get_current_user

router = APIRouter(prefix="/api/auth", tags=["auth"])

@router.post("/signup", response_model=AuthResponse)
async def signup(request: SignUpRequest, response: Response, db: Session = Depends(get_db)):
    # Check if email exists
    existing = db.query(User).filter(User.email == request.email.lower()).first()
    if existing:
        raise HTTPException(status_code=400, detail="Invalid request")  # Generic for security

    # Hash password
    password_hash = bcrypt.hashpw(request.password.encode(), bcrypt.gensalt())

    # Create user
    user = User(
        email=request.email.lower(),
        name=request.name,
        password_hash=password_hash.decode()
    )
    db.add(user)
    db.commit()
    db.refresh(user)

    # Create session
    session = create_session(user.id, db)

    # Set cookie
    response.set_cookie(
        key="better-auth.session_token",
        value=session.token,
        httponly=True,
        secure=True,  # Set to False for local dev
        samesite="lax",
        max_age=604800  # 7 days
    )

    return {"user": user, "session": session}

@router.post("/signin")
async def signin(request: SignInRequest, response: Response, db: Session = Depends(get_db)):
    # Find user
    user = db.query(User).filter(User.email == request.email.lower()).first()

    # Verify password (constant time comparison)
    if not user or not bcrypt.checkpw(request.password.encode(), user.password_hash.encode()):
        raise HTTPException(status_code=401, detail="Invalid credentials")

    # Create session
    session = create_session(user.id, db)

    # Set cookie
    response.set_cookie(
        key="better-auth.session_token",
        value=session.token,
        httponly=True,
        secure=True,
        samesite="lax",
        max_age=604800
    )

    return {"user": user, "session": session}

@router.post("/signout")
async def signout(response: Response, current_user: User = Depends(get_current_user)):
    # Clear cookie
    response.delete_cookie("better-auth.session_token")
    return {"message": "Successfully signed out"}

@router.get("/session")
async def get_session(current_user: User = Depends(get_current_user)):
    return {"user": current_user}

def create_session(user_id: str, db: Session) -> SessionModel:
    """Create a new session for user."""
    from datetime import datetime, timedelta

    session = SessionModel(
        user_id=user_id,
        token=secrets.token_urlsafe(32),
        expires_at=datetime.utcnow() + timedelta(days=7)
    )
    db.add(session)
    db.commit()
    db.refresh(session)
    return session
```

### 4.3 Register Routes

Update `backend/src/main.py`:

```python
from .api.auth_routes import router as auth_router
from .api.profile_routes import router as profile_router

app.include_router(auth_router)
app.include_router(profile_router)
```

---

## 5. Content Protection

### 5.1 Create Protected Content Wrapper

Create `src/components/Auth/ProtectedContent.tsx`:

```tsx
import React from 'react'
import { useSession } from '@/lib/auth-client'
import { LoginPrompt } from './LoginPrompt'
import { Loading } from './Loading'

export function ProtectedContent({ children }) {
  const { data: session, isPending } = useSession()

  if (isPending) {
    return <Loading />
  }

  if (!session) {
    return <LoginPrompt />
  }

  return <>{children}</>
}
```

### 5.2 Wrap Doc Content

Update `src/theme/DocItem/Content/index.tsx`:

```tsx
import { ProtectedContent } from '@/components/Auth/ProtectedContent'

export default function DocItemContent({ children }) {
  return (
    <ProtectedContent>
      <article>
        {children}
      </article>
    </ProtectedContent>
  )
}
```

---

## 6. Verification

### 6.1 Start Services

```bash
# Terminal 1: Backend
cd backend
uvicorn src.main:app --reload --port 8000

# Terminal 2: Frontend
npm start
```

### 6.2 Test Authentication Flow

1. **Visit any chapter page** → Should see login prompt
2. **Click "Sign Up"** → Fill form with email/password
3. **Submit signup** → Should be logged in, see content
4. **Refresh page** → Should remain logged in
5. **Click "Sign Out"** → Should see login prompt again

### 6.3 Test API Endpoints

```bash
# Sign up
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"password123","name":"Test User"}' \
  -c cookies.txt

# Get session (with cookie)
curl http://localhost:8000/api/auth/session -b cookies.txt

# Sign out
curl -X POST http://localhost:8000/api/auth/signout -b cookies.txt
```

---

## Troubleshooting

### Common Issues

| Issue | Solution |
|-------|----------|
| "Not authenticated" on refresh | Check cookie settings (secure, samesite) |
| Session not persisting | Verify DATABASE_URL and session table |
| CORS errors | Ensure backend allows frontend origin |
| Password validation fails | Check min 8 characters requirement |

### Debug Mode

Enable debug logging:

```python
# Backend
import logging
logging.basicConfig(level=logging.DEBUG)
```

```typescript
// Frontend - check session state
const { data, error, isPending } = useSession()
console.log({ data, error, isPending })
```

---

## Next Steps

After completing this quickstart:

1. Run `/sp.tasks` to generate implementation tasks
2. Implement background questionnaire (FR-018)
3. Add email verification flow
4. Connect to personalization system (FR-016, FR-017)
