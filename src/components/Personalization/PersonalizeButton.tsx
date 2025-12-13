/**
 * Personalize Button Component
 * Toggle button to personalize/depersonalize chapter content
 */

import React, { useState } from 'react';
import { useAuth } from '../Auth/AuthContext';
import styles from './PersonalizeButton.module.css';

interface PersonalizeButtonProps {
  isPersonalized: boolean;
  isLoading: boolean;
  onToggle: () => void;
  disabled?: boolean;
}

export function PersonalizeButton({
  isPersonalized,
  isLoading,
  onToggle,
  disabled = false,
}: PersonalizeButtonProps) {
  const { isAuthenticated, isEmailVerified, isProfileCompleted } = useAuth();

  // Don't show if not authenticated, email not verified, or profile not completed (FR-026)
  if (!isAuthenticated || !isEmailVerified || !isProfileCompleted) {
    return null;
  }

  return (
    <button
      className={`${styles.button} ${isPersonalized ? styles.active : ''}`}
      onClick={onToggle}
      disabled={disabled || isLoading}
      title={isPersonalized ? 'Show original content' : 'Personalize for your level'}
    >
      {isLoading ? (
        <>
          <span className={styles.spinner} />
          <span>Personalizing...</span>
        </>
      ) : isPersonalized ? (
        <>
          <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M9 12l2 2 4-4" />
            <circle cx="12" cy="12" r="10" />
          </svg>
          <span>Personalized</span>
        </>
      ) : (
        <>
          <svg className={styles.icon} viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M12 2L2 7l10 5 10-5-10-5zM2 17l10 5 10-5M2 12l10 5 10-5" />
          </svg>
          <span>Personalize</span>
        </>
      )}
    </button>
  );
}
