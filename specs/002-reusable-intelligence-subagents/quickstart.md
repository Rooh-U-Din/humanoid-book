# Quickstart: Reusable Intelligence with Subagents

**Feature**: 002-reusable-intelligence-subagents
**Estimated Setup Time**: ~45 minutes
**Prerequisites**: Feature 001 (BetterAuth) completed

## Overview

This quickstart guides you through implementing the Reusable Intelligence system with 5 AI-powered skills:
1. **Explain** - Code explanation
2. **Translate** - Urdu translation
3. **Debug** - Error troubleshooting
4. **Navigate** - Related content discovery
5. **Personalize** - Profile-based adaptation

## Prerequisites

Before starting, ensure you have:

- [x] Python 3.11+ installed
- [x] Node.js 18+ installed
- [x] Feature 001 (BetterAuth) completed and working
- [x] Gemini API key configured in `.env`
- [x] Backend server running locally or deployed

## Quick Start (5 minutes)

### 1. Run Database Migration

```bash
cd backend
python -m src.migrations.003_add_subagent_fields
```

### 2. Verify Skill API

```bash
# Start backend if not running
uvicorn src.main:app --reload --port 8000

# Test skills list (requires auth cookie)
curl -X GET http://localhost:8000/api/skills \
  -H "Cookie: better-auth.session=YOUR_SESSION"
```

### 3. Invoke a Skill

```bash
curl -X POST http://localhost:8000/api/skills/invoke \
  -H "Content-Type: application/json" \
  -H "Cookie: better-auth.session=YOUR_SESSION" \
  -d '{
    "skill_id": "explain",
    "input": {
      "selected_text": "def hello():\n    print(\"Hello, World!\")"
    }
  }'
```

## Step-by-Step Implementation

### Phase 1: Backend Skill Infrastructure (~20 min)

#### 1.1 Create Skill Base Classes

Create `backend/src/services/skills/base.py`:

```python
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Dict, Any, List
from datetime import datetime
import uuid

@dataclass
class AgentContext:
    """Context passed to all skill invocations"""
    user_id: str
    user_email: str
    chapter_id: Optional[str] = None
    chapter_title: Optional[str] = None
    selected_text: Optional[str] = None
    user_profile: Optional[Dict[str, Any]] = None
    trace_id: str = None
    timestamp: datetime = None

    def __post_init__(self):
        if self.trace_id is None:
            self.trace_id = str(uuid.uuid4())
        if self.timestamp is None:
            self.timestamp = datetime.utcnow()

@dataclass
class SkillResult:
    """Result from skill execution"""
    content: str
    citations: Optional[List[Dict]] = None
    suggestions: Optional[List[Dict]] = None

class BaseSkill(ABC):
    """Abstract base class for all skills"""

    @property
    @abstractmethod
    def skill_id(self) -> str:
        """Unique identifier for this skill"""
        pass

    @property
    @abstractmethod
    def name(self) -> str:
        """Human-readable name"""
        pass

    @property
    @abstractmethod
    def description(self) -> str:
        """What this skill does"""
        pass

    @property
    def version(self) -> str:
        return "1.0.0"

    @abstractmethod
    async def execute(
        self,
        input_data: Dict[str, Any],
        context: AgentContext
    ) -> SkillResult:
        """Execute the skill with given input and context"""
        pass

    def validate_input(self, input_data: Dict[str, Any]) -> bool:
        """Validate input against skill's requirements"""
        return True
```

#### 1.2 Create Skill Registry

Create `backend/src/services/skills/registry.py`:

```python
from typing import Dict, Optional, List
from .base import BaseSkill

class SkillRegistry:
    """Central registry for all available skills"""

    def __init__(self):
        self._skills: Dict[str, BaseSkill] = {}

    def register(self, skill: BaseSkill) -> None:
        """Register a skill"""
        self._skills[skill.skill_id] = skill

    def get(self, skill_id: str) -> Optional[BaseSkill]:
        """Get skill by ID"""
        return self._skills.get(skill_id)

    def list_all(self) -> List[BaseSkill]:
        """List all registered skills"""
        return list(self._skills.values())

    def is_registered(self, skill_id: str) -> bool:
        """Check if skill exists"""
        return skill_id in self._skills

# Global registry instance
skill_registry = SkillRegistry()
```

#### 1.3 Implement Individual Skills

Create `backend/src/services/skills/explain_skill.py`:

```python
from typing import Dict, Any
from .base import BaseSkill, AgentContext, SkillResult
from services.gemini_service import get_gemini_service

class ExplainSkill(BaseSkill):
    """Explains code in plain language"""

    @property
    def skill_id(self) -> str:
        return "explain"

    @property
    def name(self) -> str:
        return "Code Explanation"

    @property
    def description(self) -> str:
        return "Analyzes code and generates plain-language explanations"

    async def execute(
        self,
        input_data: Dict[str, Any],
        context: AgentContext
    ) -> SkillResult:
        selected_text = input_data.get("selected_text", "")

        prompt = f"""You are explaining code to a learner studying Physical AI and Robotics.

Code to explain:
```
{selected_text}
```

Provide a clear, beginner-friendly explanation of:
1. What this code does
2. How it works step by step
3. Why it's useful in robotics/AI context

Keep the explanation concise but thorough."""

        gemini = get_gemini_service()
        response, latency = gemini.generate_answer(
            query="Explain this code",
            context_chunks=[selected_text],
            selected_text=None
        )

        return SkillResult(
            content=response,
            citations=None,
            suggestions=[{
                "type": "skill",
                "skill_id": "debug",
                "label": "Debug this code",
                "description": "Find potential issues"
            }]
        )
```

(Similar implementations for translate_skill.py, debug_skill.py, navigate_skill.py, personalize_skill.py)

#### 1.4 Create Agent Orchestrator

Create `backend/src/services/agent_orchestrator.py`:

```python
from typing import Dict, Any
from datetime import datetime
import uuid
from services.skills.registry import skill_registry
from services.skills.base import AgentContext, SkillResult
from models.database import SkillInvocation
from services.database_service import get_db

class AgentOrchestrator:
    """Orchestrates skill invocations"""

    async def invoke_skill(
        self,
        skill_id: str,
        input_data: Dict[str, Any],
        user_id: str,
        user_email: str,
        chapter_id: str = None,
        user_profile: Dict = None
    ) -> Dict[str, Any]:
        """Invoke a skill and log the invocation"""

        trace_id = str(uuid.uuid4())
        start_time = datetime.utcnow()

        # Get skill from registry
        skill = skill_registry.get(skill_id)
        if not skill:
            raise ValueError(f"Unknown skill: {skill_id}")

        # Build context
        context = AgentContext(
            user_id=user_id,
            user_email=user_email,
            chapter_id=chapter_id,
            selected_text=input_data.get("selected_text"),
            user_profile=user_profile,
            trace_id=trace_id,
            timestamp=start_time
        )

        # Execute skill
        try:
            result = await skill.execute(input_data, context)
            status = "success"
            error_message = None
        except Exception as e:
            result = SkillResult(content=f"Error: {str(e)}")
            status = "error"
            error_message = str(e)

        # Calculate latency
        latency_ms = int((datetime.utcnow() - start_time).total_seconds() * 1000)

        # Log invocation
        self._log_invocation(
            trace_id=trace_id,
            user_id=user_id,
            skill_id=skill_id,
            input_text=input_data.get("selected_text", "")[:1000],
            output_text=result.content[:5000],
            status=status,
            latency_ms=latency_ms,
            context={"chapter_id": chapter_id},
            error_message=error_message
        )

        return {
            "skill_id": skill_id,
            "trace_id": trace_id,
            "result": {
                "content": result.content,
                "citations": result.citations,
                "suggestions": result.suggestions
            },
            "latency_ms": latency_ms,
            "timestamp": datetime.utcnow().isoformat()
        }

    def _log_invocation(self, **kwargs):
        """Log skill invocation to database"""
        # Implementation using SkillInvocation model
        pass

# Global instance
orchestrator = AgentOrchestrator()
```

### Phase 2: API Routes (~10 min)

Create `backend/src/api/skills_routes.py`:

```python
from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from typing import Optional, Dict, Any, List
from services.agent_orchestrator import orchestrator
from services.skills.registry import skill_registry
from services.session_validator import get_session

router = APIRouter(prefix="/api/skills", tags=["skills"])

class SkillInvokeRequest(BaseModel):
    skill_id: str
    input: Dict[str, Any]

@router.post("/invoke")
async def invoke_skill(
    request: SkillInvokeRequest,
    session: dict = Depends(get_session)
):
    """Invoke a skill"""
    user = session.get("user", {})

    result = await orchestrator.invoke_skill(
        skill_id=request.skill_id,
        input_data=request.input,
        user_id=user.get("id"),
        user_email=user.get("email")
    )

    return result

@router.get("")
async def list_skills(session: dict = Depends(get_session)):
    """List available skills"""
    skills = skill_registry.list_all()
    return {
        "skills": [
            {
                "id": s.skill_id,
                "name": s.name,
                "description": s.description,
                "version": s.version
            }
            for s in skills
        ]
    }
```

### Phase 3: Frontend Integration (~15 min)

#### 3.1 Create Skills API Client

Create `src/components/Skills/api.ts`:

```typescript
const API_BASE_URL = 'https://backend-book-production.up.railway.app';

export interface SkillInvokeRequest {
  skill_id: 'explain' | 'translate' | 'debug' | 'navigate' | 'personalize';
  input: {
    selected_text?: string;
    code_or_error?: string;
    chapter_id?: string;
    topic?: string;
  };
}

export interface SkillResult {
  content: string;
  citations?: Array<{ chapter_id: string; chapter_title: string; url: string }>;
  suggestions?: Array<{ type: string; skill_id: string; label: string }>;
}

export interface SkillInvokeResponse {
  skill_id: string;
  trace_id: string;
  result: SkillResult;
  latency_ms: number;
  timestamp: string;
}

export async function invokeSkill(request: SkillInvokeRequest): Promise<SkillInvokeResponse> {
  const response = await fetch(`${API_BASE_URL}/api/skills/invoke`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    credentials: 'include',
    body: JSON.stringify(request)
  });

  if (response.status === 401) {
    throw new Error('Authentication required');
  }

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.detail || 'Skill invocation failed');
  }

  return response.json();
}
```

#### 3.2 Create SkillPanel Component

Create `src/components/Skills/SkillPanel.tsx`:

```tsx
import React, { useState } from 'react';
import { useAuth } from '../Auth';
import { invokeSkill, SkillInvokeResponse } from './api';

interface SkillPanelProps {
  selectedText: string;
  chapterId?: string;
  onClose: () => void;
}

export function SkillPanel({ selectedText, chapterId, onClose }: SkillPanelProps) {
  const { isAuthenticated } = useAuth();
  const [result, setResult] = useState<SkillInvokeResponse | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleSkillClick = async (skillId: string) => {
    setLoading(true);
    setError(null);

    try {
      const response = await invokeSkill({
        skill_id: skillId as any,
        input: { selected_text: selectedText, chapter_id: chapterId }
      });
      setResult(response);
    } catch (e) {
      setError(e.message);
    } finally {
      setLoading(false);
    }
  };

  if (!isAuthenticated) {
    return <div>Please sign in to use skills</div>;
  }

  return (
    <div className="skill-panel">
      <div className="skill-buttons">
        <button onClick={() => handleSkillClick('explain')}>Explain</button>
        <button onClick={() => handleSkillClick('translate')}>Translate</button>
        <button onClick={() => handleSkillClick('debug')}>Debug</button>
      </div>

      {loading && <div>Processing...</div>}
      {error && <div className="error">{error}</div>}
      {result && (
        <div className="skill-result">
          <p>{result.result.content}</p>
          <small>Trace: {result.trace_id} | {result.latency_ms}ms</small>
        </div>
      )}
    </div>
  );
}
```

## Verification Checklist

- [ ] Database migration runs without errors
- [ ] `GET /api/skills` returns list of 5 skills
- [ ] `POST /api/skills/invoke` works for "explain" skill
- [ ] Unauthenticated requests return 401
- [ ] Skill invocations are logged in database
- [ ] Frontend SkillPanel renders and invokes skills
- [ ] Trace IDs appear in responses

## Common Issues

### "Skill not found" Error

Ensure skills are registered in `backend/src/main.py`:

```python
from services.skills.registry import skill_registry
from services.skills.explain_skill import ExplainSkill
# ... import other skills

skill_registry.register(ExplainSkill())
# ... register other skills
```

### 401 Unauthorized

Check that BetterAuth cookie is being sent:
- `credentials: 'include'` in fetch
- CORS allows credentials from frontend origin

### Slow Response Times

- First request may be slow due to Gemini cold start
- Check Gemini API quota limits
- Consider adding response caching for repeated queries

## Next Steps

1. Run `/sp.tasks` to generate implementation tasks
2. Implement remaining skills (translate, debug, navigate, personalize)
3. Add frontend text selection integration
4. Deploy to Railway
