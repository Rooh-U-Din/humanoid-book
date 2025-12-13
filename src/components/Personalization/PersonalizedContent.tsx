/**
 * Personalized Content Wrapper
 * Wraps chapter content and provides personalization toggle
 * Uses React refs only - NO direct DOM manipulation
 */

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { useAuth } from '../Auth/AuthContext';
import { PersonalizeButton } from './PersonalizeButton';
import { personalizeContent } from './personalizationApi';
import styles from './PersonalizedContent.module.css';

interface PersonalizedContentProps {
  chapterId: string;
  children: React.ReactNode;
}

export function PersonalizedContent({ chapterId, children }: PersonalizedContentProps) {
  const { isAuthenticated, isEmailVerified, isProfileCompleted } = useAuth();
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const [personalizedHtml, setPersonalizedHtml] = useState<string | null>(null);
  const [error, setError] = useState<string | null>(null);
  const contentRef = useRef<HTMLDivElement>(null);

  // Extract original content from React ref (not document.querySelector)
  const getOriginalContent = useCallback(() => {
    if (contentRef.current) {
      return contentRef.current.innerText || '';
    }
    return '';
  }, []);

  const handleToggle = async () => {
    if (isPersonalized) {
      // Switch back to original
      setIsPersonalized(false);
      return;
    }

    // Personalize content
    setIsLoading(true);
    setError(null);

    try {
      const originalContent = getOriginalContent();
      if (!originalContent || originalContent.length < 50) {
        setError('Not enough content to personalize');
        setIsLoading(false);
        return;
      }
      const response = await personalizeContent(chapterId, originalContent);
      setPersonalizedHtml(response.personalized_content);
      setIsPersonalized(true);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Personalization failed');
      console.error('Personalization error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  // Reset when chapter changes
  useEffect(() => {
    setIsPersonalized(false);
    setPersonalizedHtml(null);
    setError(null);
  }, [chapterId]);

  const showToggle = isAuthenticated && isEmailVerified && isProfileCompleted;

  return (
    <div className={styles.container}>
      {showToggle && (
        <div className={styles.toolbar}>
          <PersonalizeButton
            isPersonalized={isPersonalized}
            isLoading={isLoading}
            onToggle={handleToggle}
          />
          {isPersonalized && (
            <span className={styles.badge}>
              Content adapted to your level
            </span>
          )}
        </div>
      )}

      {error && (
        <div className={styles.error}>
          {error}
        </div>
      )}

      {isPersonalized && personalizedHtml ? (
        <div
          className={`${styles.personalizedContent} markdown`}
          dangerouslySetInnerHTML={{ __html: personalizedHtml }}
        />
      ) : (
        <div ref={contentRef} className="markdown">
          {children}
        </div>
      )}
    </div>
  );
}
