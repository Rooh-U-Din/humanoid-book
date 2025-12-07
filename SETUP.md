# Development Setup Guide

Complete guide to setting up the Physical AI & Humanoid Robotics book for local development.

## Prerequisites

- **OS**: Windows 10/11, macOS, or Linux (Ubuntu 22.04 recommended)
- **Node.js**: 18+ ([download](https://nodejs.org))
- **Python**: 3.11+ ([download](https://python.org))
- **Git**: Latest version
- **Code Editor**: VS Code recommended

## Quick Start (5 minutes)

```bash
# Clone repository
git clone https://github.com/YOUR_USERNAME/my_book.git
cd my_book

# Install frontend dependencies
npm install

# Create Python virtual environment
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# macOS/Linux:
source venv/bin/activate

# Install backend dependencies
pip install -r backend/requirements.txt

# Copy environment template
cp .env.example .env
# Edit .env and add your API keys

# Start frontend (terminal 1)
npm start

# Start backend (terminal 2)
cd backend/src
python main.py
```

Open http://localhost:3000 - the book should load!

## Detailed Setup

### 1. Frontend Setup (Docusaurus)

```bash
# Install dependencies
npm install

# Start development server
npm start
# Opens http://localhost:3000

# Build for production
npm run build

# Serve production build locally
npm run serve

# Type checking
npm run typecheck
```

**Project structure:**
```
my_book/
├── docs/              # Book chapters (MDX/MD)
├── src/
│   ├── components/    # React components
│   │   └── ChatbotWidget/  # Chatbot UI
│   ├── css/           # Global styles
│   └── theme/         # Theme overrides
├── static/            # Static assets
├── docusaurus.config.ts
├── sidebars.ts
└── package.json
```

### 2. Backend Setup (FastAPI)

```bash
# Create virtual environment
python -m venv venv

# Activate
# Windows:
venv\Scripts\activate
# macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install -r backend/requirements.txt

# Run backend
cd backend/src
python main.py
# Runs on http://localhost:8000

# API docs
# http://localhost:8000/api/docs (Swagger)
# http://localhost:8000/api/redoc (ReDoc)
```

**Project structure:**
```
backend/
├── src/
│   ├── api/           # FastAPI routes
│   ├── models/        # Pydantic models
│   ├── services/      # Business logic
│   └── main.py        # FastAPI app
├── scripts/
│   └── generate_embeddings.py
├── contracts/         # Database schemas
├── tests/             # Unit/integration tests
└── requirements.txt
```

### 3. Environment Variables

Create `.env` file in project root:

```bash
# Google Gemini API
GEMINI_API_KEY=your-gemini-api-key-here

# Qdrant Vector Database
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=your-qdrant-key

# Neon Postgres
NEON_DATABASE_URL=postgresql://user:password@ep-xxx.neon.tech/dbname?sslmode=require

# Application
ENVIRONMENT=development
LOG_LEVEL=INFO
PORT=8000
```

**Get API keys:**
- Gemini: https://ai.google.dev
- Qdrant: https://cloud.qdrant.io
- Neon: https://neon.tech

### 4. Database Setup

**Postgres (Neon):**
```bash
# Connect to Neon database
psql postgresql://user:password@ep-xxx.neon.tech/dbname?sslmode=require

# Run schema migration
\i backend/contracts/database-schema.sql

# Verify tables created
\dt
# Should show: query_logs, chat_sessions, embeddings_metadata
```

**Qdrant (Cloud):**
- Collection auto-created by embedding script
- Or create manually via Qdrant dashboard
- Use `backend/contracts/qdrant-collection.json` for config

### 5. Generate Embeddings

```bash
# Ensure backend dependencies installed
pip install -r backend/requirements.txt

# Navigate to backend
cd backend

# Run embedding script
python scripts/generate_embeddings.py --docs-dir ../docs

# Arguments:
# --docs-dir: Path to docs directory (default: ../../docs)
# --collection-name: Qdrant collection (default: book-embeddings)
# --batch-size: Batch upload size (default: 100)
```

**Expected output:**
```
==========================================================
Book Embedding Generation Script
==========================================================
✓ Collection 'book-embeddings' already exists

Processing: intro.md
  Created 5 chunks
  Uploading 5 points to Qdrant...
✓ Completed: intro.md (5 chunks)

Processing: modules/ros2/fundamentals.mdx
  Created 18 chunks
  Uploading 18 points to Qdrant...
✓ Completed: fundamentals.mdx (18 chunks)

==========================================================
SUMMARY
==========================================================
Total files found:     5
Files processed:       5
Total chunks created:  50
Collection:            book-embeddings
==========================================================
✓ Embedding generation complete!
```

## Development Workflow

### Adding New Chapters

1. **Create MDX file** in `docs/`:
   ```mdx
   ---
   sidebar_position: 3
   ---

   # Chapter Title

   Content here...
   ```

2. **Update sidebar** in `sidebars.ts`:
   ```typescript
   items: [
     'intro',
     'modules/new-chapter',  // Add here
   ]
   ```

3. **Regenerate embeddings**:
   ```bash
   python backend/scripts/generate_embeddings.py --docs-dir ../docs
   ```

4. **Test locally**:
   - Frontend: http://localhost:3000
   - Ask chatbot about new chapter

### Modifying Chatbot

**Frontend changes:**
```bash
# Edit React components
src/components/ChatbotWidget/

# Changes auto-reload (hot reload)
# Test at http://localhost:3000
```

**Backend changes:**
```bash
# Edit Python files
backend/src/

# Restart backend manually:
# Ctrl+C, then: python main.py
```

**API changes:**
```bash
# Edit routes
backend/src/api/query_routes.py

# Update models
backend/src/models/query.py

# Test with Swagger UI
http://localhost:8000/api/docs
```

## Testing

### Frontend Tests

```bash
# Type checking
npm run typecheck

# Build test
npm run build
```

### Backend Tests

```bash
# Install test dependencies (included in requirements.txt)
pip install pytest pytest-asyncio

# Run tests
cd backend
pytest

# Run with coverage
pytest --cov=src
```

### Manual Testing

**Chatbot:**
1. Open http://localhost:3000
2. Click chatbot button
3. Test queries:
   - "What is ROS 2?"
   - "How do I install Gazebo?"
   - "Explain the talker/listener example"

**Backend API:**
```bash
# Health check
curl http://localhost:8000/api/health

# Query test
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"query":"What is ROS 2?","session_id":"test-123"}'
```

## Common Issues

### Frontend won't start

**Error**: `npm ERR! code ELIFECYCLE`

**Fix**:
```bash
rm -rf node_modules package-lock.json
npm install
```

### Backend import errors

**Error**: `ModuleNotFoundError: No module named 'services'`

**Fix**:
```bash
# Make sure you're running from backend/src/
cd backend/src
python main.py

# Or use PYTHONPATH:
export PYTHONPATH="${PYTHONPATH}:backend/src"
python backend/src/main.py
```

### Chatbot not responding

**Check**:
1. Backend running? http://localhost:8000/api/health
2. Embeddings generated? Check Qdrant dashboard
3. API keys set? Check `.env` file
4. CORS issue? Check browser console

### Embeddings generation fails

**Error**: `google.api_core.exceptions.ResourceExhausted`

**Fix**: Rate limit exceeded, add delays:
```python
# In generate_embeddings.py
time.sleep(2)  # Increase from 1 to 2 seconds
```

## IDE Setup (VS Code)

**Recommended extensions:**
```json
{
  "recommendations": [
    "ms-python.python",
    "ms-python.vscode-pylance",
    "dbaeumer.vscode-eslint",
    "esbenp.prettier-vscode",
    "bradlc.vscode-tailwindcss"
  ]
}
```

**Settings** (`.vscode/settings.json`):
```json
{
  "python.defaultInterpreterPath": "./venv/bin/python",
  "python.linting.enabled": true,
  "python.linting.pylintEnabled": true,
  "editor.formatOnSave": true,
  "editor.codeActionsOnSave": {
    "source.organizeImports": true
  }
}
```

## Next Steps

- Read [DEPLOYMENT.md](DEPLOYMENT.md) for production deployment
- Check [examples/](examples/) for runnable code
- See [docs/](docs/) for book content
- Visit [backend/README.md](backend/README.md) for API details

---

Happy developing! 🚀
