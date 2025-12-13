# Data Model: BetterAuth Strict Access Control

**Branch**: `001-betterauth-access-control`
**Date**: 2025-12-13
**Source**: spec.md Key Entities + BetterAuth schema requirements

---

## Entity Relationship Diagram

```mermaid
erDiagram
    User ||--o{ Session : has
    User ||--o| UserProfile : has
    User ||--o{ Account : has
    User ||--o{ Verification : has

    User {
        string id PK "UUID"
        string email UK "unique, required"
        string name "display name"
        string password_hash "bcrypt hashed"
        boolean email_verified "default false"
        string image "profile image URL"
        datetime created_at
        datetime updated_at
    }

    Session {
        string id PK "UUID"
        string user_id FK "references User"
        string token UK "session token"
        datetime expires_at
        string ip_address "client IP"
        string user_agent "browser info"
        datetime created_at
        datetime updated_at
    }

    Account {
        string id PK "UUID"
        string user_id FK "references User"
        string provider_id "e.g., email, google"
        string account_id "provider-specific ID"
        string access_token "OAuth access token"
        string refresh_token "OAuth refresh token"
        datetime access_token_expires_at
        datetime created_at
        datetime updated_at
    }

    Verification {
        string id PK "UUID"
        string identifier "email address"
        string value "verification token"
        datetime expires_at
        datetime created_at
        datetime updated_at
    }

    UserProfile {
        string id PK "UUID"
        string user_id FK UK "references User, unique"
        string expertise_level "beginner|intermediate|expert"
        json programming_languages "array of languages"
        json learning_goals "array of goals"
        json questionnaire_responses "full responses"
        boolean questionnaire_completed "default false"
        datetime created_at
        datetime updated_at
    }
```

---

## Entity Definitions

### User (BetterAuth Core)

Primary entity representing authenticated users.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PK | Unique identifier |
| email | string(255) | UK, NOT NULL | User email address |
| name | string(255) | NOT NULL | Display name |
| password_hash | string(255) | NULL | Bcrypt hashed password (NULL for OAuth) |
| email_verified | boolean | DEFAULT false | Email verification status |
| image | string(500) | NULL | Profile image URL |
| created_at | timestamp | NOT NULL | Account creation time |
| updated_at | timestamp | NOT NULL | Last update time |

**Validation Rules**:
- Email must be valid format (RFC 5322)
- Email must be unique (case-insensitive)
- Password minimum 8 characters before hashing (FR-008)

**State Transitions**:
- `created` → User record exists, email_verified=false
- `verified` → email_verified=true, can access content
- `deleted` → Soft delete with anonymization

---

### Session (BetterAuth Core)

Represents active authentication sessions.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PK | Unique identifier |
| user_id | UUID | FK(User), NOT NULL | Reference to user |
| token | string(255) | UK, NOT NULL | Session token (cookie value) |
| expires_at | timestamp | NOT NULL | Session expiration time |
| ip_address | string(45) | NULL | Client IP (IPv4/IPv6) |
| user_agent | string(500) | NULL | Browser user agent |
| created_at | timestamp | NOT NULL | Session creation time |
| updated_at | timestamp | NOT NULL | Last activity time |

**Validation Rules**:
- Token must be cryptographically random (min 32 bytes)
- expires_at must be in future at creation
- Default expiration: 7 days from creation

**State Transitions**:
- `active` → expires_at > now, valid for auth
- `refreshed` → expires_at extended, updated_at changed
- `expired` → expires_at < now, requires re-auth
- `revoked` → Explicitly invalidated (signout)

---

### Account (BetterAuth Core)

Stores authentication provider links (email, OAuth).

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PK | Unique identifier |
| user_id | UUID | FK(User), NOT NULL | Reference to user |
| provider_id | string(50) | NOT NULL | Provider name (email, google, etc.) |
| account_id | string(255) | NOT NULL | Provider-specific user ID |
| access_token | text | NULL | OAuth access token |
| refresh_token | text | NULL | OAuth refresh token |
| access_token_expires_at | timestamp | NULL | Token expiration |
| created_at | timestamp | NOT NULL | Link creation time |
| updated_at | timestamp | NOT NULL | Last update time |

**Validation Rules**:
- Unique constraint on (user_id, provider_id)
- For email provider: account_id = user email

---

### Verification (BetterAuth Core)

Stores email verification and password reset tokens.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PK | Unique identifier |
| identifier | string(255) | NOT NULL | Email address |
| value | string(255) | NOT NULL | Verification token |
| expires_at | timestamp | NOT NULL | Token expiration |
| created_at | timestamp | NOT NULL | Token creation time |
| updated_at | timestamp | NOT NULL | Last update time |

**Validation Rules**:
- Token must be cryptographically random
- Default expiration: 24 hours for email verification
- Default expiration: 1 hour for password reset

---

### UserProfile (Application-Specific)

Stores user preferences and questionnaire responses.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PK | Unique identifier |
| user_id | UUID | FK(User), UK, NOT NULL | Reference to user (1:1) |
| expertise_level | enum | NULL | beginner, intermediate, expert |
| programming_languages | jsonb | DEFAULT '[]' | Array of known languages |
| learning_goals | jsonb | DEFAULT '[]' | Array of learning objectives |
| questionnaire_responses | jsonb | DEFAULT '{}' | Full questionnaire data |
| questionnaire_completed | boolean | DEFAULT false | Completion status |
| created_at | timestamp | NOT NULL | Profile creation time |
| updated_at | timestamp | NOT NULL | Last update time |

**Validation Rules**:
- One profile per user (unique user_id)
- expertise_level enum: 'beginner' | 'intermediate' | 'expert'
- Arrays validated as JSON arrays

---

## Indexes

```sql
-- User indexes
CREATE UNIQUE INDEX idx_user_email ON "user"(LOWER(email));
CREATE INDEX idx_user_created_at ON "user"(created_at);

-- Session indexes
CREATE UNIQUE INDEX idx_session_token ON session(token);
CREATE INDEX idx_session_user_id ON session(user_id);
CREATE INDEX idx_session_expires_at ON session(expires_at);

-- Account indexes
CREATE UNIQUE INDEX idx_account_user_provider ON account(user_id, provider_id);
CREATE INDEX idx_account_provider_account ON account(provider_id, account_id);

-- Verification indexes
CREATE INDEX idx_verification_identifier ON verification(identifier);
CREATE INDEX idx_verification_expires_at ON verification(expires_at);

-- UserProfile indexes
CREATE UNIQUE INDEX idx_user_profile_user_id ON user_profile(user_id);
```

---

## Migration Script

```sql
-- Migration: 001_betterauth_access_control
-- Created: 2025-12-13
-- Description: BetterAuth schema + UserProfile for access control

-- Enable UUID extension
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- User table (BetterAuth core)
CREATE TABLE IF NOT EXISTS "user" (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    email VARCHAR(255) NOT NULL,
    name VARCHAR(255) NOT NULL,
    password_hash VARCHAR(255),
    email_verified BOOLEAN DEFAULT FALSE,
    image VARCHAR(500),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE UNIQUE INDEX IF NOT EXISTS idx_user_email ON "user"(LOWER(email));

-- Session table (BetterAuth core)
CREATE TABLE IF NOT EXISTS session (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
    token VARCHAR(255) NOT NULL UNIQUE,
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    ip_address VARCHAR(45),
    user_agent VARCHAR(500),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_session_user_id ON session(user_id);
CREATE INDEX IF NOT EXISTS idx_session_expires_at ON session(expires_at);

-- Account table (BetterAuth core)
CREATE TABLE IF NOT EXISTS account (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID NOT NULL REFERENCES "user"(id) ON DELETE CASCADE,
    provider_id VARCHAR(50) NOT NULL,
    account_id VARCHAR(255) NOT NULL,
    access_token TEXT,
    refresh_token TEXT,
    access_token_expires_at TIMESTAMP WITH TIME ZONE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    UNIQUE(user_id, provider_id)
);

CREATE INDEX IF NOT EXISTS idx_account_provider_account ON account(provider_id, account_id);

-- Verification table (BetterAuth core)
CREATE TABLE IF NOT EXISTS verification (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    identifier VARCHAR(255) NOT NULL,
    value VARCHAR(255) NOT NULL,
    expires_at TIMESTAMP WITH TIME ZONE NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_verification_identifier ON verification(identifier);
CREATE INDEX IF NOT EXISTS idx_verification_expires_at ON verification(expires_at);

-- UserProfile table (Application-specific)
CREATE TABLE IF NOT EXISTS user_profile (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    user_id UUID NOT NULL UNIQUE REFERENCES "user"(id) ON DELETE CASCADE,
    expertise_level VARCHAR(20) CHECK (expertise_level IN ('beginner', 'intermediate', 'expert')),
    programming_languages JSONB DEFAULT '[]'::jsonb,
    learning_goals JSONB DEFAULT '[]'::jsonb,
    questionnaire_responses JSONB DEFAULT '{}'::jsonb,
    questionnaire_completed BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Trigger for updated_at
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = NOW();
    RETURN NEW;
END;
$$ language 'plpgsql';

CREATE TRIGGER update_user_updated_at BEFORE UPDATE ON "user"
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_session_updated_at BEFORE UPDATE ON session
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_account_updated_at BEFORE UPDATE ON account
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_verification_updated_at BEFORE UPDATE ON verification
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_user_profile_updated_at BEFORE UPDATE ON user_profile
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
```

---

## Pydantic Models (Backend)

```python
from datetime import datetime
from typing import Optional, List
from pydantic import BaseModel, EmailStr, Field
from uuid import UUID

class UserBase(BaseModel):
    email: EmailStr
    name: str

class UserCreate(UserBase):
    password: str = Field(..., min_length=8, max_length=128)

class UserResponse(UserBase):
    id: UUID
    email_verified: bool
    image: Optional[str] = None
    created_at: datetime

class SessionInfo(BaseModel):
    id: UUID
    user_id: UUID
    expires_at: datetime
    ip_address: Optional[str] = None

class UserProfileBase(BaseModel):
    expertise_level: Optional[str] = None
    programming_languages: List[str] = []
    learning_goals: List[str] = []

class UserProfileCreate(UserProfileBase):
    questionnaire_responses: dict = {}

class UserProfileResponse(UserProfileBase):
    id: UUID
    user_id: UUID
    questionnaire_completed: bool
    created_at: datetime
```

---

## Notes

- BetterAuth tables (user, session, account, verification) follow BetterAuth's expected schema
- UserProfile is application-specific, linked 1:1 with User
- All timestamps use timezone-aware format (TIMESTAMP WITH TIME ZONE)
- UUID used for all primary keys for security and distribution
- JSONB used for flexible questionnaire storage
- Cascade deletes ensure referential integrity
