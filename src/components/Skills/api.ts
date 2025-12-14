/**
 * Skills API Client
 *
 * Provides functions for invoking skills and managing skill history.
 * All requests include credentials for BetterAuth cookie authentication.
 */

// Backend API URL - Uses production URL when not on localhost
const API_BASE_URL = typeof window !== 'undefined' && window.location.hostname !== 'localhost'
  ? 'https://backend-book-production-1279.up.railway.app'
  : 'http://localhost:8000';

/**
 * Error thrown when authentication is required or session is invalid.
 */
export class AuthenticationError extends Error {
  constructor(message: string = 'Authentication required') {
    super(message);
    this.name = 'AuthenticationError';
  }
}

/**
 * Error thrown when a skill is not found or disabled.
 */
export class SkillNotFoundError extends Error {
  constructor(skillId: string) {
    super(`Skill '${skillId}' not found or disabled`);
    this.name = 'SkillNotFoundError';
  }
}

/**
 * Error thrown when rate limit is exceeded.
 */
export class RateLimitError extends Error {
  retryAfter?: number;

  constructor(message: string = 'Rate limit exceeded', retryAfter?: number) {
    super(message);
    this.name = 'RateLimitError';
    this.retryAfter = retryAfter;
  }
}

// Type definitions based on OpenAPI contract

export interface SkillInput {
  selected_text?: string;
  code_or_error?: string;
  chapter_id?: string;
  topic?: string;
  preserve_terms?: string[];
}

export interface SkillInvokeRequest {
  skill_id: string;
  input: SkillInput;
}

export interface Citation {
  chapter_id?: string;
  chapter_title?: string;
  section_title?: string;
  url?: string;
}

export interface Suggestion {
  type: 'skill' | 'chapter' | 'topic';
  skill_id?: string;
  label: string;
  description?: string;
}

export interface SkillResult {
  content: string;
  citations?: Citation[];
  suggestions?: Suggestion[];
}

export interface SkillInvokeResponse {
  skill_id: string;
  trace_id: string;
  result: SkillResult;
  latency_ms: number;
  timestamp: string;
}

export interface SkillInfo {
  id: string;
  name: string;
  description: string;
  version: string;
  enabled: boolean;
  input_schema?: {
    required: string[];
    optional: string[];
  };
}

export interface SkillListResponse {
  skills: SkillInfo[];
}

export interface SkillInvocationSummary {
  trace_id: string;
  skill_id: string;
  status: 'success' | 'error' | 'timeout' | 'rate_limited';
  latency_ms?: number;
  created_at: string;
}

export interface SkillHistoryResponse {
  invocations: SkillInvocationSummary[];
  total: number;
  offset: number;
  limit: number;
}

export interface SkillInvocationDetail extends SkillInvocationSummary {
  input_preview?: string;
  output_preview?: string;
  context?: Record<string, unknown>;
}

/**
 * Handle API response and throw appropriate errors.
 */
async function handleResponse<T>(response: Response): Promise<T> {
  if (response.status === 401) {
    throw new AuthenticationError('Please sign in to use AI skills');
  }

  if (response.status === 404) {
    const data = await response.json();
    throw new SkillNotFoundError(data.detail || 'Skill not found');
  }

  if (response.status === 429) {
    const data = await response.json();
    throw new RateLimitError(data.detail, data.retry_after);
  }

  if (!response.ok) {
    const data = await response.json().catch(() => ({ detail: 'Unknown error' }));
    throw new Error(data.detail || `Request failed with status ${response.status}`);
  }

  return response.json();
}

/**
 * Invoke a skill with the given input.
 *
 * @param skillId - The skill to invoke (e.g., 'explain', 'translate')
 * @param input - Skill-specific input parameters
 * @returns The skill execution result
 * @throws AuthenticationError if not authenticated
 * @throws SkillNotFoundError if skill doesn't exist
 * @throws RateLimitError if rate limited
 */
export async function invokeSkill(
  skillId: string,
  input: SkillInput
): Promise<SkillInvokeResponse> {
  const response = await fetch(`${API_BASE_URL}/api/skills/invoke`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    credentials: 'include', // Send cookies for authentication
    body: JSON.stringify({
      skill_id: skillId,
      input,
    } as SkillInvokeRequest),
  });

  return handleResponse<SkillInvokeResponse>(response);
}

/**
 * List all available (enabled) skills.
 *
 * @returns List of skill metadata
 * @throws AuthenticationError if not authenticated
 */
export async function listSkills(): Promise<SkillInfo[]> {
  const response = await fetch(`${API_BASE_URL}/api/skills`, {
    method: 'GET',
    headers: {
      'Content-Type': 'application/json',
    },
    credentials: 'include', // Send cookies for authentication
  });

  const data = await handleResponse<SkillListResponse>(response);
  return data.skills;
}

/**
 * Get the user's skill invocation history.
 *
 * @param options - Pagination and filter options
 * @returns Paginated list of invocations
 * @throws AuthenticationError if not authenticated
 */
export async function getHistory(options?: {
  limit?: number;
  offset?: number;
  skill_id?: string;
}): Promise<SkillHistoryResponse> {
  const params = new URLSearchParams();
  if (options?.limit) params.set('limit', options.limit.toString());
  if (options?.offset) params.set('offset', options.offset.toString());
  if (options?.skill_id) params.set('skill_id', options.skill_id);

  const url = `${API_BASE_URL}/api/skills/history${params.toString() ? '?' + params.toString() : ''}`;

  const response = await fetch(url, {
    method: 'GET',
    headers: {
      'Content-Type': 'application/json',
    },
    credentials: 'include', // Send cookies for authentication
  });

  return handleResponse<SkillHistoryResponse>(response);
}

/**
 * Get details of a specific skill invocation.
 *
 * @param traceId - The trace ID of the invocation
 * @returns Invocation details
 * @throws AuthenticationError if not authenticated
 */
export async function getInvocationDetail(traceId: string): Promise<SkillInvocationDetail> {
  const response = await fetch(`${API_BASE_URL}/api/skills/${traceId}`, {
    method: 'GET',
    headers: {
      'Content-Type': 'application/json',
    },
    credentials: 'include', // Send cookies for authentication
  });

  return handleResponse<SkillInvocationDetail>(response);
}
