# Deployment Guide

Complete guide to deploying the Physical AI & Humanoid Robotics book with RAG chatbot.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                     User's Browser                          │
└───────────────┬─────────────────────────────────────────────┘
                │
                ├─────────────────────────────────────────────┐
                │                                             │
        ┌───────▼────────┐                           ┌────────▼───────┐
        │  GitHub Pages  │                           │ FastAPI Backend│
        │  (Docusaurus)  │                           │  (Render.com)  │
        │                │                           │                │
        │  • Book content│──── API Calls ───────────▶│  • /api/query  │
        │  • Chatbot UI  │                           │  • /api/health │
        └────────────────┘                           └────────┬───────┘
                                                              │
                                         ┌────────────────────┴────────────────┐
                                         │                                     │
                                  ┌──────▼──────┐                     ┌───────▼──────┐
                                  │ Qdrant Cloud│                     │ Neon Postgres│
                                  │  (Vectors)  │                     │ (Query Logs) │
                                  └─────────────┘                     └──────────────┘
```

## Prerequisites

- **GitHub Account** (free)
- **Render Account** (free tier: https://render.com)
- **Qdrant Cloud Account** (free tier: https://qdrant.tech)
- **Neon Account** (free tier: https://neon.tech)
- **Google Gemini API Key** (free tier: https://ai.google.dev)

## Step 1: Setup External Services

### 1.1 Google Gemini API

1. Go to https://ai.google.dev
2. Click "Get API Key in Google AI Studio"
3. Create new API key
4. Copy the key (starts with `AIza...`)

### 1.2 Qdrant Cloud (Vector Database)

1. Sign up at https://cloud.qdrant.io
2. Create a new cluster:
   - **Plan**: Free tier
   - **Region**: Choose closest to you
3. Get credentials:
   - **URL**: `https://xxx-xxx.qdrant.io`
   - **API Key**: From cluster dashboard
4. Collection will be auto-created by embedding script

### 1.3 Neon Postgres (Metadata Database)

1. Sign up at https://neon.tech
2. Create a new project:
   - **Name**: physical-ai-robotics-db
   - **Region**: Choose closest to Render region
3. Get connection string:
   - Dashboard → Connection Details → Copy connection string
   - Format: `postgresql://user:password@ep-xxx.neon.tech/dbname?sslmode=require`
4. Run schema migration:
   ```bash
   psql <CONNECTION_STRING> < backend/contracts/database-schema.sql
   ```

## Step 2: Generate Embeddings

Before deploying, generate embeddings for your book content:

```bash
# Setup environment variables locally
export GEMINI_API_KEY="your-gemini-key"
export QDRANT_URL="https://xxx.qdrant.io"
export QDRANT_API_KEY="your-qdrant-key"

# Navigate to backend
cd backend

# Install dependencies (if not already done)
pip install -r requirements.txt

# Generate embeddings
python scripts/generate_embeddings.py --docs-dir ../docs

# Expected output:
# Processing: intro.md
#   Created 5 chunks
#   Uploading 5 points to Qdrant...
# ✓ Completed: intro.md (5 chunks)
# ...
# Total chunks created: 50-100 (depends on content)
```

**Important**: This needs to be done before the backend is useful!

## Step 3: Deploy Backend to Render

### 3.1 Push to GitHub

```bash
# Initialize git (if not already done)
git init
git add .
git commit -m "Initial commit: Physical AI book + RAG chatbot"

# Create GitHub repository at github.com/new
# Then push:
git remote add origin https://github.com/YOUR_USERNAME/my_book.git
git branch -M main
git push -u origin main
```

### 3.2 Deploy to Render

1. Go to https://dashboard.render.com
2. Click "New +" → "Web Service"
3. Connect your GitHub repository
4. Configuration:
   - **Name**: physical-ai-rag-backend
   - **Region**: Oregon (or closest)
   - **Branch**: main
   - **Root Directory**: (leave blank)
   - **Build Command**: `pip install -r backend/requirements.txt`
   - **Start Command**: `uvicorn backend.src.main:app --host 0.0.0.0 --port $PORT`
   - **Plan**: Free

5. **Environment Variables** (click "Advanced" → "Add Environment Variable"):
   ```
   PYTHON_VERSION=3.11.0
   GEMINI_API_KEY=your-gemini-api-key
   QDRANT_URL=https://xxx.qdrant.io
   QDRANT_API_KEY=your-qdrant-key
   NEON_DATABASE_URL=postgresql://...
   ENVIRONMENT=production
   LOG_LEVEL=info
   ```

6. Click "Create Web Service"
7. Wait for deployment (~5 minutes)
8. **Copy the URL**: `https://physical-ai-rag-backend.onrender.com`

### 3.3 Test Backend

```bash
# Health check
curl https://YOUR_BACKEND_URL.onrender.com/api/health

# Should return:
# {"status":"healthy","services":{"qdrant":"connected","gemini_api":"available"}}

# Test query
curl -X POST https://YOUR_BACKEND_URL.onrender.com/api/query \
  -H "Content-Type: application/json" \
  -d '{"query":"What is ROS 2?","session_id":"test"}'
```

## Step 4: Deploy Frontend to GitHub Pages

### 4.1 Update API URL

Edit `src/components/ChatbotWidget/api.ts`:

```typescript
const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://YOUR_BACKEND_URL.onrender.com'  // ← Update this!
  : 'http://localhost:8000';
```

### 4.2 Update Docusaurus Config

Edit `docusaurus.config.ts`:

```typescript
url: 'https://YOUR_USERNAME.github.io',
baseUrl: '/my_book/',
organizationName: 'YOUR_USERNAME',
projectName: 'my_book',
```

### 4.3 Enable GitHub Pages

1. Go to your GitHub repository
2. Settings → Pages
3. **Source**: GitHub Actions (not "Deploy from a branch")
4. Save

### 4.4 Trigger Deployment

```bash
# Commit changes
git add .
git commit -m "Configure production deployment"
git push origin main
```

This will automatically:
1. Trigger GitHub Actions workflow
2. Build Docusaurus site
3. Deploy to GitHub Pages
4. Available at: `https://YOUR_USERNAME.github.io/my_book/`

### 4.5 Verify Deployment

1. Open `https://YOUR_USERNAME.github.io/my_book/`
2. Click the chatbot button (bottom-right)
3. Ask: "What is ROS 2?"
4. Should receive answer with citations!

## Step 5: Monitoring & Maintenance

### Backend Monitoring (Render)

- **Logs**: Render Dashboard → Your Service → Logs
- **Metrics**: Dashboard shows CPU, memory, request count
- **Health Check**: Automated health checks every 5 minutes

### Frontend Monitoring (GitHub Pages)

- **Build Status**: GitHub repo → Actions tab
- **Analytics**: GitHub → Insights → Traffic

### Updating Content

**Add new chapters:**
1. Add `.mdx` files to `docs/`
2. Regenerate embeddings:
   ```bash
   python backend/scripts/generate_embeddings.py --docs-dir ../docs
   ```
3. Push to GitHub (frontend auto-deploys)

**Update chatbot:**
1. Edit React components in `src/components/ChatbotWidget/`
2. Push to GitHub (auto-deploys)

**Update backend:**
1. Edit Python files in `backend/src/`
2. Push to GitHub
3. Render auto-deploys from main branch

## Troubleshooting

### Chatbot shows "Failed to get response"

**Check backend health:**
```bash
curl https://YOUR_BACKEND_URL.onrender.com/api/health
```

**Common causes:**
- Backend spun down (free tier) - first request takes 30s
- Qdrant/Neon credentials incorrect
- No embeddings generated

### Citations not working

- Ensure embeddings have correct `url_path` metadata
- Check that chapter IDs match Docusaurus routes

### Slow chatbot responses

- **First request after idle**: 30+ seconds (Render free tier spin-up)
- **Subsequent requests**: 2-5 seconds
- **Upgrade Render plan** for always-on service

### Embeddings generation fails

- **Rate limits**: Add `time.sleep(1)` between API calls
- **Quota exceeded**: Check Gemini API dashboard
- **Qdrant connection**: Verify URL and API key

## Cost Breakdown

### Free Tier (Current Setup)

- **GitHub Pages**: Free (unlimited for public repos)
- **Render**: Free (512MB RAM, spins down after 15min idle)
- **Qdrant Cloud**: Free (1GB storage, 100K operations/month)
- **Neon Postgres**: Free (0.5GB storage, 1 compute unit)
- **Gemini API**: Free (60 requests/min, 1M tokens/day)

**Total**: $0/month ✅

### Production Upgrade (Recommended)

- **Render Standard**: $7/month (always on, 512MB RAM)
- **Qdrant Cloud**: $25/month (4GB storage)
- **Neon Pro**: $19/month (10GB storage)
- **Gemini API**: Still free for most use cases

**Total**: ~$51/month

## Security Notes

1. **Never commit `.env`** - Already in `.gitignore`
2. **Use Render environment variables** for secrets
3. **CORS configured** to only allow your GitHub Pages domain
4. **Rate limiting** enabled (10 queries/min per session)

## Next Steps

- Add user authentication (optional)
- Implement selection-based RAG mode
- Add conversation history
- Deploy to custom domain
- Set up monitoring/alerts
- Add A/B testing for prompts

---

**Deployed!** Your Physical AI & Humanoid Robotics book with RAG chatbot is now live! 🎉
