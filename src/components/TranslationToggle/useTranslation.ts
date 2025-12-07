/**
 * useTranslation Hook
 *
 * Manages translation state for chapter content.
 */

import { useState, useEffect, useCallback, useRef } from 'react';
import {
  translateChapter,
  setLanguagePreference,
  getLanguagePreference,
  type TranslateResponse,
} from './translationApi';

export type Language = 'urdu' | 'english';

export interface TranslationState {
  currentLanguage: Language;
  isLoading: boolean;
  error: string | null;
  translatedContent: string | null;
  cached: boolean;
}

export interface UseTranslationResult extends TranslationState {
  toggleLanguage: () => Promise<void>;
  setLanguage: (lang: Language) => Promise<void>;
  getDisplayContent: (originalContent: string) => string;
}

// Local storage keys
const STORAGE_PREFIX = 'chapter_translation_';
const SESSION_KEY = 'translation_session_id';

/**
 * Get or create session ID for translation preferences
 */
function getSessionId(): string {
  if (typeof window === 'undefined') {
    return 'ssr_session';
  }

  let sessionId = localStorage.getItem(SESSION_KEY);
  if (!sessionId) {
    sessionId = `trans_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    localStorage.setItem(SESSION_KEY, sessionId);
  }
  return sessionId;
}

/**
 * Get cached translation from localStorage
 */
function getCachedTranslation(chapterId: string): string | null {
  if (typeof window === 'undefined') return null;

  try {
    const cached = localStorage.getItem(`${STORAGE_PREFIX}${chapterId}_urdu`);
    return cached;
  } catch {
    return null;
  }
}

/**
 * Cache translation in localStorage
 */
function cacheTranslation(chapterId: string, content: string): void {
  if (typeof window === 'undefined') return;

  try {
    localStorage.setItem(`${STORAGE_PREFIX}${chapterId}_urdu`, content);
  } catch {
    // Storage full or unavailable
    console.warn('Failed to cache translation');
  }
}

/**
 * Get saved language preference from localStorage
 */
function getSavedPreference(chapterId: string): Language {
  if (typeof window === 'undefined') return 'english';

  try {
    const pref = localStorage.getItem(`${STORAGE_PREFIX}${chapterId}_lang`);
    return (pref === 'urdu' || pref === 'english') ? pref : 'english';
  } catch {
    return 'english';
  }
}

/**
 * Save language preference to localStorage
 */
function savePreference(chapterId: string, language: Language): void {
  if (typeof window === 'undefined') return;

  try {
    localStorage.setItem(`${STORAGE_PREFIX}${chapterId}_lang`, language);
  } catch {
    console.warn('Failed to save language preference');
  }
}

/**
 * Hook for managing chapter translation
 */
export function useTranslation(chapterId: string): UseTranslationResult {
  const [state, setState] = useState<TranslationState>({
    currentLanguage: 'english',
    isLoading: false,
    error: null,
    translatedContent: null,
    cached: false,
  });

  const sessionId = useRef<string>('');
  const originalContentRef = useRef<string>('');

  // Initialize on mount
  useEffect(() => {
    sessionId.current = getSessionId();

    // Load saved preference
    const savedLang = getSavedPreference(chapterId);
    const cachedTranslation = getCachedTranslation(chapterId);

    setState(prev => ({
      ...prev,
      currentLanguage: savedLang,
      translatedContent: cachedTranslation,
      cached: !!cachedTranslation,
    }));
  }, [chapterId]);

  /**
   * Perform translation
   */
  const performTranslation = useCallback(async (content: string): Promise<void> => {
    if (!content || content.length < 10) {
      setState(prev => ({ ...prev, error: 'Content too short to translate' }));
      return;
    }

    setState(prev => ({ ...prev, isLoading: true, error: null }));

    try {
      // Check local cache first
      const cached = getCachedTranslation(chapterId);
      if (cached) {
        setState(prev => ({
          ...prev,
          isLoading: false,
          translatedContent: cached,
          cached: true,
        }));
        return;
      }

      // Call API
      const response = await translateChapter(
        chapterId,
        content,
        sessionId.current
      );

      // Cache the result
      cacheTranslation(chapterId, response.urduContent);

      setState(prev => ({
        ...prev,
        isLoading: false,
        translatedContent: response.urduContent,
        cached: response.cached,
      }));
    } catch (error) {
      setState(prev => ({
        ...prev,
        isLoading: false,
        error: error instanceof Error ? error.message : 'Translation failed',
      }));
    }
  }, [chapterId]);

  /**
   * Set language and translate if needed
   */
  const setLanguage = useCallback(async (lang: Language): Promise<void> => {
    if (lang === state.currentLanguage) return;

    // Save preference
    savePreference(chapterId, lang);

    // Update state
    setState(prev => ({
      ...prev,
      currentLanguage: lang,
      error: null,
    }));

    // If switching to Urdu and no translation, fetch it
    if (lang === 'urdu' && !state.translatedContent && originalContentRef.current) {
      await performTranslation(originalContentRef.current);
    }

    // Try to sync with backend (non-blocking)
    setLanguagePreference(chapterId, lang, sessionId.current).catch(() => {
      // Ignore backend sync errors
    });
  }, [chapterId, state.currentLanguage, state.translatedContent, performTranslation]);

  /**
   * Toggle between English and Urdu
   */
  const toggleLanguage = useCallback(async (): Promise<void> => {
    const newLang: Language = state.currentLanguage === 'english' ? 'urdu' : 'english';
    await setLanguage(newLang);
  }, [state.currentLanguage, setLanguage]);

  /**
   * Get content to display based on current language
   */
  const getDisplayContent = useCallback((originalContent: string): string => {
    // Store original content for later translation
    originalContentRef.current = originalContent;

    if (state.currentLanguage === 'urdu' && state.translatedContent) {
      return state.translatedContent;
    }

    return originalContent;
  }, [state.currentLanguage, state.translatedContent]);

  return {
    ...state,
    toggleLanguage,
    setLanguage,
    getDisplayContent,
  };
}

export default useTranslation;
