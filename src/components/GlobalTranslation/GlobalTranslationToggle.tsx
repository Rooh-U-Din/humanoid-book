/**
 * Global Translation Toggle Component
 *
 * Automatically appears on every doc page.
 * Handles translation state, API calls, and content switching.
 */

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { useLocation } from '@docusaurus/router';
import styles from './GlobalTranslation.module.css';

type Language = 'english' | 'urdu';

// API Configuration - Railway backend
const API_BASE_URL = 'https://backend-book-production-4d5a.up.railway.app';

// Storage keys
const STORAGE_PREFIX = 'global_translation_';
const SESSION_KEY = 'translation_session_id';

// Technical terms to preserve (not translate)
const PRESERVE_TERMS = [
  'ROS 2', 'ROS2', 'Gazebo', 'NVIDIA', 'Isaac', 'Unity', 'Python', 'C++',
  'Ubuntu', 'Linux', 'Windows', 'macOS', 'Docker', 'Git', 'GitHub',
  'API', 'SDK', 'CLI', 'GUI', 'IDE', 'JSON', 'YAML', 'XML', 'HTML', 'CSS',
  'HTTP', 'HTTPS', 'REST', 'GraphQL', 'WebSocket', 'TCP', 'UDP', 'IP',
  'GPU', 'CPU', 'RAM', 'SSD', 'CUDA', 'TensorRT', 'PyTorch', 'TensorFlow',
  'SLAM', 'LIDAR', 'IMU', 'GPS', 'URDF', 'SDF', 'XACRO',
  'OpenCV', 'PCL', 'MoveIt', 'Nav2', 'rclpy', 'rclcpp',
  'Humble', 'Iron', 'Jazzy', 'Noetic', 'Melodic',
];

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
  // Convert path like /docs/modules/ros2/fundamentals to ros2-fundamentals
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

function getSavedLanguage(chapterId: string): Language {
  if (typeof window === 'undefined') return 'english';
  try {
    const saved = localStorage.getItem(`${STORAGE_PREFIX}${chapterId}_lang`);
    return saved === 'urdu' ? 'urdu' : 'english';
  } catch {
    return 'english';
  }
}

function saveLanguage(chapterId: string, lang: Language): void {
  if (typeof window === 'undefined') return;
  try {
    localStorage.setItem(`${STORAGE_PREFIX}${chapterId}_lang`, lang);
  } catch {
    console.warn('Failed to save language preference');
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
  const [isTranslated, setIsTranslated] = useState(false);

  const contentRef = useRef<HTMLElement | null>(null);
  const originalContentRef = useRef<string>('');
  const chapterId = getChapterIdFromPath(location.pathname);

  // Find and store reference to the main content
  useEffect(() => {
    if (typeof window === 'undefined') return;

    // Wait for DOM to be ready, then find the article content
    const findContent = () => {
      // Try multiple selectors for Docusaurus content
      const selectors = [
        'article.markdown',
        '.markdown',
        '[class*="docItemContainer"] article',
        '.theme-doc-markdown',
        'main article',
      ];

      for (const selector of selectors) {
        const element = document.querySelector(selector);
        if (element && element.textContent && element.textContent.length > 100) {
          contentRef.current = element as HTMLElement;
          if (!originalContentRef.current) {
            originalContentRef.current = element.innerHTML;
          }
          return true;
        }
      }
      return false;
    };

    // Try immediately
    if (!findContent()) {
      // Retry after a short delay if not found
      const timer = setTimeout(findContent, 500);
      return () => clearTimeout(timer);
    }

    // Load saved preference
    const savedLang = getSavedLanguage(chapterId);
    const cachedTranslation = getCachedTranslation(chapterId);

    if (savedLang === 'urdu' && cachedTranslation) {
      setTranslatedContent(cachedTranslation);
      setLanguage('urdu');
    }
  }, [chapterId, location.pathname]);

  // Apply translation when language changes
  useEffect(() => {
    if (!contentRef.current) return;

    if (language === 'urdu' && translatedContent) {
      // Store original if not already stored
      if (!originalContentRef.current) {
        originalContentRef.current = contentRef.current.innerHTML;
      }

      // Apply translated content
      contentRef.current.innerHTML = `
        <div class="${styles.urduContent}" dir="rtl">
          ${translatedContent.split('\n').map(p => `<p>${p}</p>`).join('')}
        </div>
      `;
      setIsTranslated(true);
    } else if (language === 'english' && originalContentRef.current && isTranslated) {
      // Restore original content
      contentRef.current.innerHTML = originalContentRef.current;
      setIsTranslated(false);
    }
  }, [language, translatedContent, isTranslated]);

  const handleToggle = useCallback(async () => {
    if (isLoading) return;

    if (language === 'urdu') {
      // Switch back to English
      setLanguage('english');
      saveLanguage(chapterId, 'english');
      return;
    }

    // Check cache first
    const cached = getCachedTranslation(chapterId);
    if (cached) {
      setTranslatedContent(cached);
      setLanguage('urdu');
      saveLanguage(chapterId, 'urdu');
      return;
    }

    // Get content to translate
    if (!contentRef.current) {
      setError('Content not found');
      return;
    }

    const textContent = contentRef.current.innerText;
    if (textContent.length < 50) {
      setError('Not enough content to translate');
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const sessionId = getSessionId();
      const translated = await translateContent(chapterId, textContent, sessionId);

      // Cache and apply
      cacheTranslation(chapterId, translated);
      setTranslatedContent(translated);
      setLanguage('urdu');
      saveLanguage(chapterId, 'urdu');
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'Translation failed';
      // Show friendly message for quota exceeded
      if (errorMsg.includes('429') || errorMsg.includes('quota') || errorMsg.includes('limit')) {
        setError('Translation limit reached. Please wait 1 minute and try again.');
      } else {
        setError(errorMsg);
      }
    } finally {
      setIsLoading(false);
    }
  }, [chapterId, language, isLoading]);

  return (
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
              <span>Show Original English</span>
            </>
          )}
        </button>

        {language === 'urdu' && !isLoading && (
          <span className={styles.activeBadge}>
            Viewing Urdu Translation
          </span>
        )}
      </div>

      {error && (
        <div className={styles.errorBar}>
          <span>⚠️ {error}</span>
          <button className={styles.retryBtn} onClick={handleToggle}>
            Retry
          </button>
        </div>
      )}
    </div>
  );
}
