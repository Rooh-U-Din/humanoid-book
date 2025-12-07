/**
 * TranslationToggle Component
 *
 * Toggle button for switching between English and Urdu translations.
 */

import React, { useCallback, useState, useEffect } from 'react';
import styles from './TranslationToggle.module.css';

export type Language = 'urdu' | 'english';

interface TranslationToggleProps {
  chapterId: string;
  onTranslate: (content: string) => Promise<string>;
  originalContent: string;
  onContentChange: (content: string, language: Language) => void;
}

// Storage keys
const STORAGE_PREFIX = 'chapter_translation_';

export default function TranslationToggle({
  chapterId,
  onTranslate,
  originalContent,
  onContentChange,
}: TranslationToggleProps): JSX.Element {
  const [language, setLanguage] = useState<Language>('english');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);

  // Load saved preference and cached translation on mount
  useEffect(() => {
    if (typeof window === 'undefined') return;

    // Load saved language preference
    const savedLang = localStorage.getItem(`${STORAGE_PREFIX}${chapterId}_lang`);
    if (savedLang === 'urdu' || savedLang === 'english') {
      setLanguage(savedLang);
    }

    // Load cached translation
    const cached = localStorage.getItem(`${STORAGE_PREFIX}${chapterId}_urdu`);
    if (cached) {
      setTranslatedContent(cached);
    }
  }, [chapterId]);

  // Apply content when language or translation changes
  useEffect(() => {
    if (language === 'urdu' && translatedContent) {
      onContentChange(translatedContent, 'urdu');
    } else if (language === 'english') {
      onContentChange(originalContent, 'english');
    }
  }, [language, translatedContent, originalContent, onContentChange]);

  const handleToggle = useCallback(async () => {
    if (isLoading) return;

    const newLang: Language = language === 'english' ? 'urdu' : 'english';

    // Save preference
    if (typeof window !== 'undefined') {
      localStorage.setItem(`${STORAGE_PREFIX}${chapterId}_lang`, newLang);
    }

    if (newLang === 'urdu') {
      // Check cache first
      if (translatedContent) {
        setLanguage('urdu');
        return;
      }

      // Translate
      setIsLoading(true);
      setError(null);

      try {
        const translated = await onTranslate(originalContent);

        // Cache the translation
        if (typeof window !== 'undefined') {
          localStorage.setItem(`${STORAGE_PREFIX}${chapterId}_urdu`, translated);
        }

        setTranslatedContent(translated);
        setLanguage('urdu');
      } catch (err) {
        setError(err instanceof Error ? err.message : 'Translation failed');
      } finally {
        setIsLoading(false);
      }
    } else {
      setLanguage('english');
    }
  }, [language, isLoading, translatedContent, originalContent, chapterId, onTranslate]);

  return (
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
          <span>⚠️</span>
          <span>{error}</span>
          <button
            className={styles.retryButton}
            onClick={handleToggle}
          >
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
  );
}
