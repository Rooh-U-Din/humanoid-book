/**
 * SkillPanel Component
 *
 * A panel that appears when text is selected, providing AI-powered skills
 * like code explanation, translation, debugging, etc.
 *
 * Features:
 * - Text selection detection
 * - Skill buttons (Explain, Translate, Debug, etc.)
 * - Authentication gating with login prompt
 * - Loading states and error handling
 * - Result display with suggestions
 */

import React, { useState, useCallback, useEffect } from 'react';
import { useAuth, AuthModal } from '../Auth';
import { useSkill } from '../../hooks/useSkill';
import { SkillResult } from './SkillResult';
import { Suggestion, SkillInput } from './api';
import styles from './SkillPanel.module.css';

interface SkillPanelProps {
  /** The selected text to process */
  selectedText: string;
  /** Current chapter ID for context */
  chapterId?: string;
  /** Callback when panel is closed */
  onClose: () => void;
  /** Position of the panel */
  position?: { top: number; left: number };
}

/**
 * Login prompt shown when user is not authenticated.
 */
function SkillLoginPrompt({ onSignIn }: { onSignIn: () => void }) {
  return (
    <div className={styles.loginPrompt}>
      <div className={styles.loginIcon}>🔒</div>
      <p className={styles.loginText}>Sign in to use AI skills</p>
      <button className={styles.signInButton} onClick={onSignIn}>
        Sign In
      </button>
    </div>
  );
}

/**
 * Loading spinner component.
 */
function LoadingSpinner() {
  return (
    <div className={styles.loadingContainer}>
      <div className={styles.spinner} />
      <span className={styles.loadingText}>Processing...</span>
    </div>
  );
}

/**
 * Error display component.
 */
function ErrorDisplay({ message, onRetry }: { message: string; onRetry?: () => void }) {
  return (
    <div className={styles.errorContainer}>
      <span className={styles.errorIcon}>⚠️</span>
      <p className={styles.errorText}>{message}</p>
      {onRetry && (
        <button className={styles.retryButton} onClick={onRetry}>
          Try Again
        </button>
      )}
    </div>
  );
}

/**
 * SkillPanel provides AI-powered text analysis tools.
 *
 * Appears when text is selected and allows users to invoke skills
 * like explanation, translation, and debugging.
 */
export function SkillPanel({
  selectedText,
  chapterId,
  onClose,
  position,
}: SkillPanelProps): JSX.Element {
  const { isAuthenticated, isLoading: authLoading } = useAuth();
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [lastInvocation, setLastInvocation] = useState<{
    skillId: string;
    input: SkillInput;
  } | null>(null);

  const {
    invoke,
    result,
    error,
    isLoading,
    isAuthenticated: skillAuthOk,
    reset,
  } = useSkill();

  // Handle skill invocation
  const handleSkillClick = useCallback(async (skillId: string) => {
    const input: SkillInput = {
      selected_text: selectedText,
      chapter_id: chapterId,
    };
    setLastInvocation({ skillId, input });
    await invoke(skillId, input);
  }, [selectedText, chapterId, invoke]);

  // Handle suggestion click (invoke another skill)
  const handleSuggestionClick = useCallback(async (suggestion: Suggestion) => {
    if (suggestion.type === 'skill' && suggestion.skill_id) {
      await handleSkillClick(suggestion.skill_id);
    }
  }, [handleSkillClick]);

  // Handle retry
  const handleRetry = useCallback(() => {
    if (lastInvocation) {
      invoke(lastInvocation.skillId, lastInvocation.input);
    }
  }, [lastInvocation, invoke]);

  // Handle sign in
  const handleSignIn = () => setShowAuthModal(true);
  const handleAuthSuccess = () => setShowAuthModal(false);

  // Reset on close
  useEffect(() => {
    return () => reset();
  }, [reset]);

  // Panel style with position
  const panelStyle = position
    ? { top: position.top, left: position.left }
    : undefined;

  return (
    <>
      <div className={styles.skillPanel} style={panelStyle}>
        {/* Header */}
        <div className={styles.header}>
          <span className={styles.headerTitle}>AI Skills</span>
          <button
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close skill panel"
          >
            ✕
          </button>
        </div>

        {/* Content area */}
        <div className={styles.content}>
          {/* Loading auth state */}
          {authLoading ? (
            <LoadingSpinner />
          ) : !isAuthenticated ? (
            /* Not authenticated - show login prompt */
            <SkillLoginPrompt onSignIn={handleSignIn} />
          ) : isLoading ? (
            /* Skill is executing */
            <LoadingSpinner />
          ) : error ? (
            /* Error state */
            <ErrorDisplay
              message={error.message}
              onRetry={skillAuthOk ? handleRetry : undefined}
            />
          ) : result ? (
            /* Show result */
            <SkillResult
              response={result}
              onSuggestionClick={handleSuggestionClick}
              showTraceId={true}
            />
          ) : (
            /* Show skill buttons */
            <div className={styles.skillButtons}>
              <p className={styles.selectedPreview}>
                {selectedText.length > 100
                  ? selectedText.slice(0, 100) + '...'
                  : selectedText}
              </p>
              <div className={styles.buttonGrid}>
                <button
                  className={styles.skillButton}
                  onClick={() => handleSkillClick('explain')}
                  title="Get a plain-language explanation of this code"
                >
                  <span className={styles.skillIcon}>💡</span>
                  <span className={styles.skillLabel}>Explain</span>
                </button>
                <button
                  className={styles.skillButton}
                  onClick={() => handleSkillClick('translate')}
                  title="Translate to Urdu"
                >
                  <span className={styles.skillIcon}>🌐</span>
                  <span className={styles.skillLabel}>Translate</span>
                </button>
                <button
                  className={styles.skillButton}
                  onClick={() => handleSkillClick('debug')}
                  title="Get help debugging this code"
                >
                  <span className={styles.skillIcon}>🔍</span>
                  <span className={styles.skillLabel}>Debug</span>
                </button>
                <button
                  className={styles.skillButton}
                  onClick={() => handleSkillClick('navigate')}
                  title="Find related content"
                >
                  <span className={styles.skillIcon}>🔗</span>
                  <span className={styles.skillLabel}>Related</span>
                </button>
              </div>
              <button className={styles.clearButton} onClick={reset}>
                Clear Selection
              </button>
            </div>
          )}
        </div>
      </div>

      {/* Auth Modal */}
      <AuthModal
        isOpen={showAuthModal}
        onClose={() => setShowAuthModal(false)}
        onSuccess={handleAuthSuccess}
        defaultTab="signin"
      />
    </>
  );
}

export default SkillPanel;
