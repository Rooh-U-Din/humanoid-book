/**
 * API Client for RAG Chatbot
 *
 * Handles communication with the FastAPI backend.
 */

export interface QueryRequest {
  query: string;
  session_id: string;
}

export interface Citation {
  chapter_id: string;
  chapter_title: string;
  section_title: string;
  url: string;
  relevance_score?: number;
}

export interface QueryResponse {
  answer: string;
  citations: Citation[];
  mode: 'full-book' | 'selection';
  latency_ms: number;
  session_id: string;
  timestamp: string;
}

// Backend API URL - Railway production backend
const API_BASE_URL = 'https://backend-book-production-4d5a.up.railway.app';

// Demo mode responses when API is unavailable
const DEMO_RESPONSES: Record<string, string> = {
  'ros': 'ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides tools, libraries, and conventions to simplify creating complex robot behaviors. Key features include:\n\n• **Nodes**: Independent processes that perform computation\n• **Topics**: Named buses for message passing\n• **Services**: Request/response communication\n• **Actions**: Long-running tasks with feedback\n\nROS 2 improves on ROS 1 with better real-time support, security, and multi-robot capabilities.',
  'gazebo': 'Gazebo is a powerful 3D robot simulator that integrates with ROS 2. It provides:\n\n• **Physics simulation**: Accurate dynamics using ODE, Bullet, or DART\n• **Sensor simulation**: Cameras, LiDAR, IMU, GPS\n• **World building**: Create custom environments\n• **Plugin system**: Extend functionality\n\nGazebo is essential for testing robot algorithms before deploying on real hardware.',
  'isaac': 'NVIDIA Isaac is a platform for AI-powered robotics:\n\n• **Isaac Sim**: GPU-accelerated simulation with photorealistic rendering\n• **Isaac ROS**: Hardware-accelerated ROS 2 packages\n• **Isaac Gym**: Reinforcement learning for robotics\n\nIsaac leverages NVIDIA GPUs for faster training and inference.',
  'vla': 'Vision-Language-Action (VLA) models combine:\n\n• **Vision**: Understanding images/video\n• **Language**: Following natural language instructions\n• **Action**: Generating robot control commands\n\nExamples include RT-1, RT-2, and OpenVLA. These enable robots to follow commands like "pick up the red cup" by understanding both visual context and language.',
  'default': 'I can help you learn about Physical AI and Humanoid Robotics! Try asking about:\n\n• **ROS 2** - Robot Operating System\n• **Gazebo** - Robot simulation\n• **NVIDIA Isaac** - AI robotics platform\n• **VLA models** - Vision-Language-Action\n• **Unity Robotics** - Game engine for robots'
};

function getDemoResponse(query: string): string {
  const q = query.toLowerCase();
  if (q.includes('ros')) return DEMO_RESPONSES['ros'];
  if (q.includes('gazebo') || q.includes('simulation')) return DEMO_RESPONSES['gazebo'];
  if (q.includes('isaac') || q.includes('nvidia')) return DEMO_RESPONSES['isaac'];
  if (q.includes('vla') || q.includes('vision') || q.includes('language')) return DEMO_RESPONSES['vla'];
  return DEMO_RESPONSES['default'];
}

/**
 * Authentication error class for 401 responses
 */
export class AuthenticationError extends Error {
  constructor(message: string = 'Authentication required') {
    super(message);
    this.name = 'AuthenticationError';
  }
}

/**
 * Send a query to the chatbot (requires authentication)
 */
export async function sendQuery(
  query: string,
  sessionId: string
): Promise<QueryResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'X-Session-ID': sessionId,
      },
      credentials: 'include', // Send cookies for authentication
      body: JSON.stringify({
        query,
        session_id: sessionId,
      }),
    });

    // Handle authentication errors
    if (response.status === 401) {
      throw new AuthenticationError('Please sign in to use the AI assistant');
    }

    if (!response.ok) {
      const error = await response.json().catch(() => ({ detail: 'Unknown error' }));
      const detail = error.detail || 'Failed to get response from chatbot';

      // Fall back to demo mode for quota/rate limit errors
      if (response.status === 429 || detail.includes('quota') || detail.includes('rate limit')) {
        return {
          answer: getDemoResponse(query),
          citations: [],
          mode: 'full-book',
          latency_ms: 50,
          session_id: sessionId,
          timestamp: new Date().toISOString(),
        };
      }
      if (response.status === 503 || detail.includes('unavailable')) {
        throw new Error('The AI service is currently unavailable. Please try again later.');
      }

      throw new Error(detail);
    }

    return response.json();
  } catch (error) {
    // Re-throw authentication errors
    if (error instanceof AuthenticationError) {
      throw error;
    }
    // Fall back to demo mode if backend is unreachable
    if (error instanceof TypeError && error.message.includes('fetch')) {
      return {
        answer: getDemoResponse(query),
        citations: [],
        mode: 'full-book',
        latency_ms: 50,
        session_id: sessionId,
        timestamp: new Date().toISOString(),
      };
    }
    throw error;
  }
}

/**
 * Send a selection-based query (requires authentication)
 */
export async function sendSelectionQuery(
  selectedText: string,
  query: string,
  chapterContext: string,
  sessionId: string
): Promise<QueryResponse> {
  const response = await fetch(`${API_BASE_URL}/api/query-selection`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'X-Session-ID': sessionId,
    },
    credentials: 'include', // Send cookies for authentication
    body: JSON.stringify({
      selected_text: selectedText,
      query,
      chapter_context: chapterContext,
      session_id: sessionId,
    }),
  });

  // Handle authentication errors
  if (response.status === 401) {
    throw new AuthenticationError('Please sign in to use the AI assistant');
  }

  if (!response.ok) {
    const error = await response.json().catch(() => ({ detail: 'Unknown error' }));
    const detail = error.detail || 'Failed to get response from chatbot';

    // Handle specific error types with user-friendly messages
    if (response.status === 429 || detail.includes('quota') || detail.includes('rate limit')) {
      throw new Error('The AI service is temporarily unavailable due to high usage. Please try again in a few minutes.');
    }
    if (response.status === 503 || detail.includes('unavailable')) {
      throw new Error('The AI service is currently unavailable. Please try again later.');
    }

    throw new Error(detail);
  }

  return response.json();
}

/**
 * Check backend health
 */
export async function healthCheck(): Promise<{
  status: string;
  services: Record<string, string>;
}> {
  const response = await fetch(`${API_BASE_URL}/api/health`);

  if (!response.ok) {
    throw new Error('Health check failed');
  }

  return response.json();
}
