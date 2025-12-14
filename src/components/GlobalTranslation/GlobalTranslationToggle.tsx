/**
 * Global Translation Toggle Component
 *
 * Automatically appears on every doc page.
 * Uses React state only - NO direct DOM manipulation.
 */

import React, { useState, useCallback } from 'react';
import { useLocation } from '@docusaurus/router';
import styles from './GlobalTranslation.module.css';

type Language = 'english' | 'urdu';

// API Configuration - Railway backend
const API_BASE_URL = 'https://backend-book-production-1279.up.railway.app';

// Storage keys
const STORAGE_PREFIX = 'global_translation_';
const SESSION_KEY = 'translation_session_id';

function getSessionId(): string {
  if (typeof window === 'undefined') return 'ssr';
  let sessionId = localStorage.getItem(SESSION_KEY);
  if (!sessionId) {
    sessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    localStorage.setItem(SESSION_KEY, sessionId);
  }
  return sessionId;
}

function getChapterIdFromPath(pathname: string): string {
  const parts = pathname.replace(/^\/docs\/?/, '').replace(/\/$/, '').split('/');
  return parts.filter(p => p && p !== 'modules').join('-') || 'intro';
}

function getCachedTranslation(chapterId: string): string | null {
  if (typeof window === 'undefined') return null;
  try {
    return localStorage.getItem(`${STORAGE_PREFIX}${chapterId}_urdu`);
  } catch {
    return null;
  }
}

function cacheTranslation(chapterId: string, content: string): void {
  if (typeof window === 'undefined') return;
  try {
    localStorage.setItem(`${STORAGE_PREFIX}${chapterId}_urdu`, content);
  } catch {
    console.warn('Failed to cache translation');
  }
}

async function translateContent(
  chapterId: string,
  content: string,
  sessionId: string
): Promise<string> {
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
    throw new Error(error.detail || 'Translation failed');
  }

  const data = await response.json();
  return data.urduContent;
}

export default function GlobalTranslationToggle(): JSX.Element {
  const location = useLocation();
  const [language, setLanguage] = useState<Language>('english');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [showTranslation, setShowTranslation] = useState(false);

  const chapterId = getChapterIdFromPath(location.pathname);

  const handleToggle = useCallback(async () => {
    if (isLoading) return;

    if (language === 'urdu') {
      // Switch back to English
      setLanguage('english');
      setShowTranslation(false);
      return;
    }

    // Check cache first
    const cached = getCachedTranslation(chapterId);
    if (cached) {
      setTranslatedContent(cached);
      setLanguage('urdu');
      setShowTranslation(true);
      return;
    }

    // Get page title/description for translation context
    const pageTitle = document.title || chapterId;
    const metaDesc = document.querySelector('meta[name="description"]')?.getAttribute('content') || '';
    const contentToTranslate = `${pageTitle}\n\n${metaDesc}`;

    if (contentToTranslate.length < 10) {
      setError('Not enough content to translate');
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const sessionId = getSessionId();
      const translated = await translateContent(chapterId, contentToTranslate, sessionId);

      // Cache and apply
      cacheTranslation(chapterId, translated);
      setTranslatedContent(translated);
      setLanguage('urdu');
      setShowTranslation(true);
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Translation failed';
      if (errorMsg.includes('429') || errorMsg.includes('quota') || errorMsg.includes('limit')) {
        setError('Translation limit reached. Please wait 1 minute and try again.');
      } else {
        setError(errorMsg);
      }
    } finally {
      setIsLoading(false);
    }
  }, [chapterId, language, isLoading]);

  const handleCloseTranslation = useCallback(() => {
    setShowTranslation(false);
    setLanguage('english');
  }, []);

  return (
    <>
      <div className={styles.translationBar}>
        <div className={styles.translationContainer}>
          <button
            className={`${styles.toggleButton} ${language === 'urdu' ? styles.urduActive : ''}`}
            onClick={handleToggle}
            disabled={isLoading}
            aria-label={language === 'english' ? 'Translate to Urdu' : 'Show Original English'}
          >
            {isLoading ? (
              <>
                <span className={styles.spinner}></span>
                <span>Translating...</span>
              </>
            ) : language === 'english' ? (
              <>
                <span className={styles.langIcon}>اردو</span>
                <span>Translate to Urdu</span>
              </>
            ) : (
              <>
                <span className={styles.langIcon}>EN</span>
                <span>Show Original</span>
              </>
            )}
          </button>

          {language === 'urdu' && !isLoading && (
            <span className={styles.activeBadge}>
              Urdu Translation Active
            </span>
          )}
        </div>

        {error && (
          <div className={styles.errorBar}>
            <span>{error}</span>
            <button className={styles.retryBtn} onClick={handleToggle}>
              Retry
            </button>
          </div>
        )}
      </div>

      {/* Translation overlay - rendered via React, no DOM manipulation */}
      {showTranslation && translatedContent && (
        <div className={styles.translationOverlay}>
          <div className={styles.translationPanel}>
            <div className={styles.panelHeader}>
              <h3>اردو ترجمہ (Urdu Translation)</h3>
              <button
                className={styles.closeButton}
                onClick={handleCloseTranslation}
                aria-label="Close translation"
              >
                ✕
              </button>
            </div>
            <div className={styles.panelContent} dir="rtl">
              {translatedContent.split('\n').map((paragraph, index) => (
                <p key={index}>{paragraph}</p>
              ))}
            </div>
          </div>
        </div>
      )}
    </>
  );
}
