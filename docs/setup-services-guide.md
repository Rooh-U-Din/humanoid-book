# External Services Setup Guide

Step-by-step guide to set up Qdrant Cloud and Neon Postgres for your RAG chatbot.

## Current Status

✅ **Google Gemini API**: Configured
- API Key: `AIzaSyBjAbQjayvYHt9b3Es3w-9nsXXRrRtmQpw`
- Status: Ready to use

⏳ **Qdrant Cloud**: Not yet configured
⏳ **Neon Postgres**: Not yet configured

---

## Step 1: Setup Qdrant Cloud (Vector Database)

### 1.1 Create Account

1. Go to: https://cloud.qdrant.io
2. Click **"Sign Up"** or **"Get Started"**
3. Sign up with:
   - GitHub account (recommended)
   - Google account
   - Or email/password

### 1.2 Create Cluster

1. After login, click **"Create Cluster"**
2. Configure cluster:
   ```
   Cluster Name:     physical-ai-book-vectors
   Cloud Provider:   AWS (or preferred)
   Region:          us-east-1 (or closest to you)
   Plan:            Free Tier (1 GB storage, 100K operations/month)
   ```

3. Click **"Create Cluster"**
4. Wait 2-3 minutes for cluster provisioning

### 1.3 Get Credentials

1. Click on your cluster name
2. In the cluster dashboard, find:
   - **Cluster URL**: `https://xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx.us-east-1-0.aws.cloud.qdrant.io:6333`
   - **API Key**: Click "API Keys" → "Create API Key" → Copy the key

3. **Save these credentials** - you'll need them for `.env`:
   ```
   QDRANT_URL=https://your-cluster-id.region.aws.cloud.qdrant.io:6333
   QDRANT_API_KEY=your-qdrant-api-key-here
   ```

### 1.4 Verify Connection (Optional)

You can test the connection using their web interface:
1. Go to cluster dashboard
2. Click "Collections" tab
3. Should show empty list (collection will be created by embedding script)

---

## Step 2: Setup Neon Postgres (Metadata Database)

### 2.1 Create Account

1. Go to: https://neon.tech
2. Click **"Sign Up"**
3. Sign up with:
   - GitHub account (recommended)
   - Google account
   - Or email/password

### 2.2 Create Project

1. After login, you'll see "Create your first project"
2. Configure project:
   ```
   Project Name:     physical-ai-book-db
   Postgres Version: 16 (latest)
   Region:          US East (Ohio) or closest to you
   Compute Size:    0.25 vCPU, 1 GB RAM (Free Tier)
   ```

3. Click **"Create Project"**
4. Project provisions in ~30 seconds

### 2.3 Get Connection String

1. On the project dashboard, you'll see "Connection Details"
2. Select **"Connection string"** tab
3. Copy the connection string:
   ```
   postgresql://[user]:[password]@[endpoint].neon.tech/[database]?sslmode=require
   ```

   Example:
   ```
   postgresql://myuser:AbCd1234EfGh@ep-cool-field-12345678.us-east-1.aws.neon.tech/mydb?sslmode=require
   ```

4. **Save this connection string** for `.env`:
   ```
   NEON_DATABASE_URL=postgresql://[your-connection-string]
   ```

### 2.4 Create Database Schema

1. In Neon dashboard, click **"SQL Editor"**
2. Or use `psql` locally:
   ```bash
   psql "postgresql://[your-connection-string]"
   ```

3. Run the schema migration:
   ```sql
   -- Copy and paste the contents of backend/contracts/database-schema.sql
   -- Or run from command line:
   ```

   **From command line:**
   ```bash
   psql "postgresql://[your-connection-string]" < backend/contracts/database-schema.sql
   ```

4. Verify tables created:
   ```sql
   \dt
   -- Should show: query_logs, chat_sessions, embeddings_metadata
   ```

---

## Step 3: Update Environment Variables

### 3.1 Update `.env` File

Open `D:\vsCode\CLI\hackathon\my_book\.env` and add all credentials:

```bash
# Google Gemini API Configuration
GEMINI_API_KEY=AIzaSyBjAbQjayvYHt9b3Es3w-9nsXXRrRtmQpw

# Qdrant Vector Database Configuration
QDRANT_URL=https://[your-cluster-id].[region].aws.cloud.qdrant.io:6333
QDRANT_API_KEY=[your-qdrant-api-key]

# Neon Serverless Postgres Configuration
NEON_DATABASE_URL=postgresql://[user]:[password]@[endpoint].neon.tech/[database]?sslmode=require

# Application Configuration
ENVIRONMENT=development
LOG_LEVEL=INFO
PORT=8000
```

### 3.2 Verify Configuration

```bash
# Check that all required variables are set
cat .env | grep -E "(GEMINI|QDRANT|NEON)"
```

---

## Step 4: Test Connections

### 4.1 Test Backend Startup

```bash
cd backend/src
python main.py
```

**Expected output:**
```
INFO:     Started server process [XXXX]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
```

### 4.2 Test Health Endpoint

In a new terminal:
```bash
curl http://localhost:8000/api/health
```

**Expected response:**
```json
{
  "status": "healthy",
  "timestamp": "2024-12-06T...",
  "services": {
    "qdrant": "connected",
    "postgres": "connected",
    "gemini_api": "available"
  }
}
```

If you see errors, check the troubleshooting section below.

---

## Troubleshooting

### Qdrant Connection Issues

**Error**: `ConnectionError: Could not connect to Qdrant`

**Solutions:**
1. Verify cluster is running (check Qdrant dashboard)
2. Check URL format includes `https://` and port `:6333`
3. Verify API key is correct (no extra spaces)
4. Check firewall/network settings

**Test connection:**
```bash
curl -X GET "https://[your-cluster-url]:6333/collections" \
  -H "api-key: [your-api-key]"
```

### Neon Connection Issues

**Error**: `psycopg2.OperationalError: could not connect to server`

**Solutions:**
1. Verify connection string format (must include `?sslmode=require`)
2. Check project is active (not suspended)
3. Verify username/password are correct
4. Check IP allowlist in Neon dashboard (default allows all IPs)

**Test connection:**
```bash
psql "postgresql://[your-connection-string]" -c "SELECT version();"
```

### Gemini API Issues

**Error**: `google.api_core.exceptions.InvalidArgument: API key not valid`

**Solutions:**
1. Verify API key is correct
2. Check API is enabled at https://ai.google.dev
3. Check quota limits (free tier: 60 requests/min)

**Test API:**
```bash
curl -X POST "https://generativelanguage.googleapis.com/v1beta/models/gemini-pro:generateContent?key=YOUR_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{"contents":[{"parts":[{"text":"Hello"}]}]}'
```

---

## Next Steps

After all services are configured and tested:

1. **Generate Embeddings**:
   ```bash
   cd backend
   python scripts/generate_embeddings.py --docs-dir ../docs
   ```

2. **Start Full Application**:
   ```bash
   # Terminal 1: Backend
   cd backend/src
   python main.py

   # Terminal 2: Frontend
   npm start
   ```

3. **Test Chatbot**:
   - Open http://localhost:3000
   - Click chatbot button
   - Ask: "What is ROS 2?"

---

## Cost Monitoring

### Free Tier Limits

**Qdrant Cloud (Free):**
- Storage: 1 GB
- Operations: 100,000/month
- Expected usage: ~10-50 MB for book (well within limits)

**Neon Postgres (Free):**
- Storage: 0.5 GB
- Compute: 100 hours/month
- Expected usage: ~1-5 MB for metadata (well within limits)

**Gemini API (Free):**
- Requests: 60/minute
- Tokens: 1 million/day
- Expected usage: ~10-100 requests/day for development

### When to Upgrade

Consider upgrading if:
- Qdrant: Storage >1 GB (unlikely for this project)
- Neon: Storage >0.5 GB or need always-on compute
- Gemini: Need >60 requests/min or >1M tokens/day

---

## Summary Checklist

Before proceeding to embeddings generation:

- [ ] Qdrant cluster created and running
- [ ] Qdrant URL and API key saved to `.env`
- [ ] Neon project created and database provisioned
- [ ] Database schema applied (3 tables created)
- [ ] Neon connection string saved to `.env`
- [ ] Gemini API key configured in `.env`
- [ ] Backend starts without errors
- [ ] Health endpoint returns "healthy" status
- [ ] All three services show "connected" in health check

Once all items are checked, you're ready to generate embeddings!
