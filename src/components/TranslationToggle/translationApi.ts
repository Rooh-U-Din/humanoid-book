/**
 * Translation API Client
 *
 * Handles communication with the translation backend.
 */

// Backend API URL
const API_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://your-backend.onrender.com'
  : 'http://localhost:8000';

export interface TranslateRequest {
  chapterId: string;
  targetLanguage: 'urdu' | 'english';
  content: string;
}

export interface TranslateResponse {
  chapterId: string;
  urduContent: string;
  cached: boolean;
  translatedAt: string;
  latencyMs: number;
}

export interface PreferenceResponse {
  chapterId: string;
  language: 'urdu' | 'english';
  updatedAt: string | null;
}

/**
 * Translate chapter content to Urdu
 */
export async function translateChapter(
  chapterId: string,
  content: string,
  sessionId: string
): Promise<TranslateResponse> {
  const response = await fetch(`${API_BASE_URL}/api/chapter/translate`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'X-Session-ID': sessionId,
    },
    body: JSON.stringify({
      chapterId,
      targetLanguage: 'urdu',
      content,
    }),
  });

  if (!response.ok) {
    const error = await response.json().catch(() => ({ detail: 'Translation failed' }));
    throw new Error(error.detail || 'Failed to translate chapter');
  }

  return response.json();
}

/**
 * Set user's language preference for a chapter
 */
export async function setLanguagePreference(
  chapterId: string,
  language: 'urdu' | 'english',
  sessionId: string
): Promise<PreferenceResponse> {
  const response = await fetch(`${API_BASE_URL}/api/chapter/preference`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'X-Session-ID': sessionId,
    },
    body: JSON.stringify({
      chapterId,
      language,
    }),
  });

  if (!response.ok) {
    const error = await response.json().catch(() => ({ detail: 'Failed to set preference' }));
    throw new Error(error.detail || 'Failed to set language preference');
  }

  return response.json();
}

/**
 * Get user's language preference for a chapter
 */
export async function getLanguagePreference(
  chapterId: string,
  sessionId: string
): Promise<PreferenceResponse> {
  const response = await fetch(`${API_BASE_URL}/api/chapter/preference/${chapterId}`, {
    method: 'GET',
    headers: {
      'X-Session-ID': sessionId,
    },
  });

  if (!response.ok) {
    // Return default preference if fetch fails
    return {
      chapterId,
      language: 'english',
      updatedAt: null,
    };
  }

  return response.json();
}
