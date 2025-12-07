## RAG Chatbot Widget

This directory contains the React components for the AI-powered chatbot widget.

### Components

- **ChatbotWidget.tsx**: Main chat interface with message list and input
- **MessageBubble.tsx**: Individual message rendering with citations
- **FloatingButton.tsx**: Bottom-right floating button to open chat
- **api.ts**: API client for backend communication
- **ChatbotWidget.module.css**: Component styles

### Features

- ✅ Real-time chat interface
- ✅ Citation links to book sections
- ✅ Loading states with typing animation
- ✅ Error handling
- ✅ Session persistence (localStorage)
- ✅ Mobile responsive
- ✅ Dark mode support (Docusaurus theme)

### Backend API

The chatbot communicates with the FastAPI backend:

- **POST /api/query**: Full-book mode queries
- **POST /api/query-selection**: Selection-based queries (future)
- **GET /api/health**: Health check

### Configuration

Update the API URL in `api.ts`:

```typescript
const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-backend.onrender.com'  // Update this!
  : 'http://localhost:8000';
```

### Integration

The chatbot is integrated via `src/theme/Root.tsx`, which wraps the entire Docusaurus site.

### Styling

The component uses CSS modules for scoped styling. The styles automatically adapt to Docusaurus light/dark themes using CSS variables:

- `--ifm-background-color`
- `--ifm-font-color-base`
- `--ifm-color-primary`
- `--ifm-color-emphasis-*`

### Session Management

User sessions are tracked via a unique session ID stored in localStorage. This enables:
- Rate limiting (10 queries/min per session)
- Analytics (future)
- Conversation history (future)
