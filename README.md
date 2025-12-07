# Physical AI & Humanoid Robotics

> An interactive technical textbook with AI-powered chatbot for learning robotics, simulation, and Vision-Language-Action systems.

[![Deploy Book](https://github.com/YOUR_USERNAME/my_book/actions/workflows/deploy-book.yml/badge.svg)](https://github.com/YOUR_USERNAME/my_book/actions/workflows/deploy-book.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

## Overview

This project is a comprehensive educational platform combining:
- **Interactive Docusaurus Book**: 12+ chapters covering ROS 2, Gazebo, Unity, NVIDIA Isaac Sim, VLA systems, and humanoid robotics
- **RAG-Powered Chatbot**: AI assistant using Retrieval-Augmented Generation for accurate, citation-backed answers
- **Runnable Code Examples**: Ready-to-use ROS 2 packages and Gazebo simulations
- **Cloud Deployment**: Serverless architecture with free-tier hosting

## Features

### Book Content
- **Mermaid Diagrams**: Architecture visualizations, message flows, state machines
- **Code Examples**: Python, C++, YAML, SDF with syntax highlighting
- **Hands-on Projects**: Talker/listener, service clients, Gazebo worlds
- **Capstone Project**: Voice-controlled humanoid robot

### RAG Chatbot
- **Full-Book Mode**: Ask questions about any topic in the book
- **Citation Links**: Jump directly to relevant sections
- **Session Persistence**: Conversation history across page reloads
- **Rate Limiting**: 10 queries/minute per session
- **Dark Mode Support**: Adapts to Docusaurus theme

### Technical Stack
- **Frontend**: Docusaurus 3, React, TypeScript, CSS Modules
- **Backend**: FastAPI, Python 3.11, Async/Await
- **AI/ML**: Google Gemini (embeddings + generation)
- **Databases**: Qdrant (vectors), Neon Postgres (metadata)
- **Deployment**: GitHub Pages (frontend), Render (backend)

## Quick Start

### Prerequisites
- Node.js 18+
- Python 3.11+
- Git
- 16 GB RAM (32 GB recommended)

### Installation (5 minutes)

```bash
# Clone repository
git clone https://github.com/YOUR_USERNAME/my_book.git
cd my_book

# Install frontend
npm install

# Setup Python environment
python -m venv venv
venv\Scripts\activate  # Windows
# source venv/bin/activate  # macOS/Linux

# Install backend
pip install -r backend/requirements.txt

# Configure environment
cp .env.example .env
# Edit .env and add your API keys:
# - GEMINI_API_KEY (https://ai.google.dev)
# - QDRANT_URL + QDRANT_API_KEY (https://cloud.qdrant.io)
# - NEON_DATABASE_URL (https://neon.tech)
```

### Run Locally

**Terminal 1 (Frontend):**
```bash
npm start
# Opens http://localhost:3000
```

**Terminal 2 (Backend):**
```bash
cd backend/src
python main.py
# Runs on http://localhost:8000
```

**Terminal 3 (Generate Embeddings):**
```bash
cd backend
python scripts/generate_embeddings.py --docs-dir ../docs
# Creates 50-100 chunks from book content
```

Open http://localhost:3000, click the chatbot button, and ask: *"What is ROS 2?"*

## Documentation

- **[SETUP.md](SETUP.md)**: Complete development setup guide
- **[DEPLOYMENT.md](DEPLOYMENT.md)**: Production deployment to GitHub Pages + Render
- **[Backend API](backend/README.md)**: FastAPI routes and models
- **[Chatbot Widget](src/components/ChatbotWidget/README.md)**: React component docs

## Project Structure

```
my_book/
├── docs/                      # Book chapters (MDX/MD)
│   ├── intro.md               # Course overview
│   └── modules/
│       ├── ros2/              # ROS 2 tutorials
│       ├── gazebo/            # Gazebo simulation
│       ├── unity/             # Unity robotics
│       ├── isaac/             # NVIDIA Isaac Sim
│       ├── vla/               # Vision-Language-Action
│       ├── conversational/    # Conversational AI
│       └── capstone/          # Final project
│
├── src/                       # Frontend React components
│   ├── components/
│   │   └── ChatbotWidget/     # RAG chatbot UI
│   ├── css/                   # Global styles
│   └── theme/                 # Docusaurus theme overrides
│
├── backend/                   # FastAPI backend
│   ├── src/
│   │   ├── api/               # Query + health routes
│   │   ├── models/            # Pydantic + SQLAlchemy models
│   │   └── services/          # Business logic
│   ├── scripts/
│   │   └── generate_embeddings.py  # Embedding generation
│   └── contracts/             # Database schemas
│
├── examples/                  # Runnable code
│   ├── ros2/                  # ROS 2 packages
│   └── gazebo/                # Simulation worlds
│
├── .github/workflows/         # CI/CD
│   └── deploy-book.yml        # GitHub Pages deployment
│
├── docusaurus.config.ts       # Docusaurus configuration
├── render.yaml                # Render.com deployment
└── package.json               # Frontend dependencies
```

## Technology Stack

### Frontend
- **Docusaurus 3.5.2**: Static site generator with MDX support
- **React 18**: UI components and hooks
- **TypeScript**: Type-safe development
- **Mermaid**: Diagram rendering
- **Axios**: HTTP client for API calls

### Backend
- **FastAPI 0.109+**: Async web framework
- **Pydantic**: Data validation and serialization
- **SQLAlchemy**: Async ORM for Postgres
- **Google Generative AI**: Gemini embeddings (768-dim) + chat

### Infrastructure
- **Qdrant Cloud**: Vector database (Cosine similarity)
- **Neon**: Serverless Postgres
- **Render**: Backend hosting (free tier)
- **GitHub Pages**: Static site hosting

## Usage Examples

### Ask Questions

**Full-Book Mode:**
```
User: What is ROS 2?
Bot: ROS 2 (Robot Operating System 2) is an open-source robotics middleware...
     📖 ROS 2 Fundamentals (95%)
     📖 Course Overview (87%)
```

**Follow-up Questions:**
```
User: How do I create a publisher?
Bot: To create a publisher in ROS 2, use the create_publisher() method...
     [Code example with citation to ROS 2 Fundamentals]
```

### Run Examples

**ROS 2 Talker/Listener:**
```bash
cd examples/ros2/talker_listener

# Terminal 1
ros2 run talker_listener talker

# Terminal 2
ros2 run talker_listener listener
```

**Gazebo Warehouse:**
```bash
cd examples/gazebo
ros2 launch warehouse_sim.launch.py
```

## Deployment

### Free Tier (Recommended for Demo)
- **Cost**: $0/month
- **Services**: GitHub Pages + Render Free + Qdrant Free + Neon Free + Gemini Free
- **Limitations**: Backend spins down after 15min idle (30s cold start)

### Production Upgrade
- **Cost**: ~$51/month
- **Services**: Render Standard ($7) + Qdrant Cloud ($25) + Neon Pro ($19)
- **Benefits**: Always-on backend, 4 GB vector storage, 10 GB database

See [DEPLOYMENT.md](DEPLOYMENT.md) for complete instructions.

## API Endpoints

### Health Check
```bash
GET /api/health

Response:
{
  "status": "healthy",
  "services": {
    "qdrant": "connected",
    "gemini_api": "available"
  }
}
```

### Query Chatbot
```bash
POST /api/query
Content-Type: application/json

{
  "query": "What is ROS 2?",
  "session_id": "uuid-here"
}

Response:
{
  "answer": "ROS 2 is an open-source...",
  "citations": [
    {
      "chapter_id": "ros2-fundamentals",
      "chapter_title": "ROS 2 Fundamentals",
      "section_title": "Introduction",
      "url": "/docs/modules/ros2/fundamentals#introduction",
      "relevance_score": 0.95
    }
  ],
  "mode": "full-book",
  "latency_ms": 2341,
  "session_id": "uuid-here"
}
```

See [backend/README.md](backend/README.md) for complete API documentation.

## Development Workflow

### Adding New Chapters

1. **Create MDX file**:
   ```bash
   # Create docs/modules/new-topic/intro.mdx
   ---
   sidebar_position: 1
   ---

   # New Topic

   Content here...
   ```

2. **Update sidebar** in `sidebars.ts`:
   ```typescript
   {
     type: 'category',
     label: 'New Topic',
     items: ['modules/new-topic/intro']
   }
   ```

3. **Regenerate embeddings**:
   ```bash
   cd backend
   python scripts/generate_embeddings.py --docs-dir ../docs
   ```

4. **Test**:
   - Frontend: http://localhost:3000
   - Ask chatbot about new chapter

### Modifying Chatbot

**Frontend changes** (React components):
```bash
# Edit src/components/ChatbotWidget/ChatbotWidget.tsx
# Changes auto-reload via hot reload
```

**Backend changes** (Python services):
```bash
# Edit backend/src/services/response_service.py
# Restart backend: Ctrl+C, then: python main.py
```

## Testing

### Frontend
```bash
npm run typecheck    # Type checking
npm run build        # Production build
npm run serve        # Serve build locally
```

### Backend
```bash
cd backend
pytest               # Run all tests
pytest --cov=src     # With coverage
```

### Manual Testing
```bash
# Health check
curl http://localhost:8000/api/health

# Query test
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"query":"What is ROS 2?","session_id":"test-123"}'
```

## Troubleshooting

### Chatbot not responding
1. Check backend health: http://localhost:8000/api/health
2. Verify embeddings generated: Check Qdrant dashboard
3. Check API keys in `.env` file
4. Check browser console for CORS errors

### Build failures
```bash
# Clear caches
rm -rf node_modules package-lock.json
npm install

# Clear Docusaurus cache
npm run clear
npm start
```

### Python import errors
```bash
# Ensure running from backend/src/
cd backend/src
python main.py

# Or set PYTHONPATH:
export PYTHONPATH="${PYTHONPATH}:backend/src"
python backend/src/main.py
```

See [SETUP.md](SETUP.md) for complete troubleshooting guide.

## Contributing

Contributions are welcome! Please follow these guidelines:

1. **Fork** the repository
2. **Create a feature branch**: `git checkout -b feature/amazing-feature`
3. **Commit changes**: `git commit -m "Add amazing feature"`
4. **Push to branch**: `git push origin feature/amazing-feature`
5. **Open a Pull Request**

### Code Standards
- Frontend: ESLint + Prettier
- Backend: Black + Pylint
- Type checking: TypeScript (frontend), Pydantic (backend)
- Tests: 80%+ coverage for new features

See [CONTRIBUTING.md](CONTRIBUTING.md) for detailed guidelines.

## Roadmap

- [x] ROS 2 fundamentals chapter
- [x] Gazebo simulation chapter
- [x] RAG chatbot integration
- [x] Code examples (talker/listener, Gazebo worlds)
- [x] Deployment to GitHub Pages + Render
- [ ] Unity robotics chapter
- [ ] NVIDIA Isaac Sim chapter
- [ ] VLA systems chapter
- [ ] Conversational robotics chapter
- [ ] Capstone: Humanoid voice control project
- [ ] Selection-based RAG mode
- [ ] User authentication
- [ ] Conversation history
- [ ] Multimodal examples (images, videos)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- **ROS 2**: Open Robotics Foundation
- **Gazebo**: Open Source Robotics Foundation
- **Google Gemini**: For embeddings and chat completion
- **Qdrant**: Vector database infrastructure
- **Neon**: Serverless Postgres hosting
- **Render**: Backend deployment platform
- **Docusaurus**: Meta Open Source

## Support

- **Documentation**: [SETUP.md](SETUP.md) | [DEPLOYMENT.md](DEPLOYMENT.md)
- **Issues**: [GitHub Issues](https://github.com/YOUR_USERNAME/my_book/issues)
- **Discussions**: [GitHub Discussions](https://github.com/YOUR_USERNAME/my_book/discussions)

---

**Built with Spec-Driven Development** | Generated with Claude Code

