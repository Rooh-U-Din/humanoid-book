/**
 * ChapterWithTranslation Component
 *
 * Wrapper component that adds translation toggle to chapter content.
 * Use this in MDX files to enable Urdu translation.
 */

import React, { useState, useCallback, useEffect, useRef } from 'react';
import styles from './TranslationToggle.module.css';
import { translateChapter } from './translationApi';

export type Language = 'urdu' | 'english';

interface ChapterWithTranslationProps {
  chapterId: string;
  children: React.ReactNode;
}

// Storage keys
const STORAGE_PREFIX = 'chapter_translation_';
const SESSION_KEY = 'translation_session_id';

function getSessionId(): string {
  if (typeof window === 'undefined') return 'ssr';
  let sessionId = localStorage.getItem(SESSION_KEY);
  if (!sessionId) {
    sessionId = `trans_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    localStorage.setItem(SESSION_KEY, sessionId);
  }
  return sessionId;
}

export default function ChapterWithTranslation({
  chapterId,
  children,
}: ChapterWithTranslationProps): JSX.Element {
  const [language, setLanguage] = useState<Language>('english');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const contentRef = useRef<HTMLDivElement>(null);
  const originalHtmlRef = useRef<string>('');

  // Load saved preference and cached translation
  useEffect(() => {
    if (typeof window === 'undefined') return;

    const savedLang = localStorage.getItem(`${STORAGE_PREFIX}${chapterId}_lang`);
    if (savedLang === 'urdu') {
      const cached = localStorage.getItem(`${STORAGE_PREFIX}${chapterId}_urdu`);
      if (cached) {
        setTranslatedContent(cached);
        setLanguage('urdu');
      }
    }
  }, [chapterId]);

  // Store original HTML when component mounts
  useEffect(() => {
    if (contentRef.current && !originalHtmlRef.current) {
      originalHtmlRef.current = contentRef.current.innerHTML;
    }
  }, [children]);

  const handleTranslate = useCallback(async () => {
    if (isLoading) return;

    if (language === 'urdu') {
      // Switch back to English
      setLanguage('english');
      localStorage.setItem(`${STORAGE_PREFIX}${chapterId}_lang`, 'english');
      return;
    }

    // Check cache first
    const cached = localStorage.getItem(`${STORAGE_PREFIX}${chapterId}_urdu`);
    if (cached) {
      setTranslatedContent(cached);
      setLanguage('urdu');
      localStorage.setItem(`${STORAGE_PREFIX}${chapterId}_lang`, 'urdu');
      return;
    }

    // Get text content from the chapter
    const textContent = contentRef.current?.innerText || '';
    if (textContent.length < 50) {
      setError('Not enough content to translate');
      return;
    }

    setIsLoading(true);
    setError(null);

    try {
      const sessionId = getSessionId();
      const response = await translateChapter(chapterId, textContent, sessionId);

      // Cache the translation
      localStorage.setItem(`${STORAGE_PREFIX}${chapterId}_urdu`, response.urduContent);
      localStorage.setItem(`${STORAGE_PREFIX}${chapterId}_lang`, 'urdu');

      setTranslatedContent(response.urduContent);
      setLanguage('urdu');
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Translation failed');
    } finally {
      setIsLoading(false);
    }
  }, [chapterId, language, isLoading]);

  return (
    <div className={styles.chapterWrapper}>
      {/* Translation Toggle Button */}
      <div className={styles.translationContainer}>
        <button
          className={`${styles.toggleButton} ${language === 'urdu' ? styles.urduActive : ''}`}
          onClick={handleTranslate}
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
              <span className={styles.icon}>اردو</span>
              <span>Translate to Urdu</span>
            </>
          ) : (
            <>
              <span className={styles.icon}>EN</span>
              <span>Show Original English</span>
            </>
          )}
        </button>

        {error && (
          <div className={styles.error}>
            <span>⚠️ {error}</span>
            <button className={styles.retryButton} onClick={handleTranslate}>
              Retry
            </button>
          </div>
        )}

        {language === 'urdu' && translatedContent && (
          <div className={styles.badge}>
            <span>🌐</span>
            <span>Viewing Urdu Translation</span>
          </div>
        )}
      </div>

      {/* Chapter Content */}
      {language === 'urdu' && translatedContent ? (
        <div
          className={styles.urduContent}
          style={{
            direction: 'rtl',
            textAlign: 'right',
            fontFamily: "'Noto Nastaliq Urdu', serif",
            lineHeight: 2.2,
            fontSize: '1.1em',
          }}
        >
          {translatedContent.split('\n').map((paragraph, idx) => (
            <p key={idx} style={{ marginBottom: '1em' }}>
              {paragraph}
            </p>
          ))}
        </div>
      ) : (
        <div ref={contentRef}>{children}</div>
      )}
    </div>
  );
}
