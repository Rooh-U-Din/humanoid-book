/**
 * Personalization API client
 */

import { authClient } from '../../lib/auth-client';

// Backend API URL - Railway production backend
const API_BASE_URL = 'https://backend-book-production-4d5a.up.railway.app';

export interface PersonalizeRequest {
  chapter_id: string;
  original_content: string;
}

export interface PersonalizeResponse {
  chapter_id: string;
  personalized_content: string;
  is_cached: boolean;
  profile_hash: string;
}

export interface PersonalizationStatus {
  chapter_id: string;
  is_personalized: boolean;
  profile_hash: string | null;
  cached_at: string | null;
}

/**
 * Personalize chapter content based on user profile
 */
export async function personalizeContent(
  chapterId: string,
  originalContent: string
): Promise<PersonalizeResponse> {
  const token = authClient.getToken();
  if (!token) {
    throw new Error('Not authenticated');
  }

  const response = await fetch(`${API_BASE_URL}/api/personalize`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      'Authorization': `Bearer ${token}`,
    },
    body: JSON.stringify({
      chapter_id: chapterId,
      original_content: originalContent,
    }),
  });

  if (!response.ok) {
    const error = await response.json().catch(() => ({ detail: 'Personalization failed' }));
    throw new Error(error.detail || 'Personalization failed');
  }

  return response.json();
}

/**
 * Check if personalized content exists for a chapter
 */
export async function getPersonalizationStatus(
  chapterId: string
): Promise<PersonalizationStatus> {
  const token = authClient.getToken();
  if (!token) {
    throw new Error('Not authenticated');
  }

  const response = await fetch(`${API_BASE_URL}/api/personalize/status/${encodeURIComponent(chapterId)}`, {
    headers: {
      'Authorization': `Bearer ${token}`,
    },
  });

  if (!response.ok) {
    const error = await response.json().catch(() => ({ detail: 'Failed to get status' }));
    throw new Error(error.detail || 'Failed to get status');
  }

  return response.json();
}

/**
 * Clear cached personalized content for a chapter
 */
export async function clearPersonalizationCache(
  chapterId: string
): Promise<void> {
  const token = authClient.getToken();
  if (!token) {
    throw new Error('Not authenticated');
  }

  const response = await fetch(`${API_BASE_URL}/api/personalize/cache/${encodeURIComponent(chapterId)}`, {
    method: 'DELETE',
    headers: {
      'Authorization': `Bearer ${token}`,
    },
  });

  if (!response.ok) {
    const error = await response.json().catch(() => ({ detail: 'Failed to clear cache' }));
    throw new Error(error.detail || 'Failed to clear cache');
  }
}
