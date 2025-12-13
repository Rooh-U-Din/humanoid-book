/**
 * Complete Profile Prompt Component (FR-003)
 * Shows a prompt to users who haven't completed their profile
 */

import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import { QuestionnaireModal } from './QuestionnaireModal';
import styles from './CompleteProfilePrompt.module.css';

interface CompleteProfilePromptProps {
  /** Message to show explaining why profile completion is needed */
  message?: string;
  /** Whether to show as a banner or inline */
  variant?: 'banner' | 'inline';
}

export function CompleteProfilePrompt({
  message = 'Complete your profile to get personalized content tailored to your experience level.',
  variant = 'banner'
}: CompleteProfilePromptProps) {
  const { isAuthenticated, isEmailVerified, isProfileCompleted } = useAuth();
  const [showQuestionnaire, setShowQuestionnaire] = useState(false);

  // Don't show if not authenticated, not verified, or already completed
  if (!isAuthenticated || !isEmailVerified || isProfileCompleted) {
    return null;
  }

  const containerClass = variant === 'banner' ? styles.banner : styles.inline;

  return (
    <>
      <div className={containerClass}>
        <div className={styles.icon}>
          <svg viewBox="0 0 24 24" width="20" height="20" fill="currentColor">
            <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-2 15l-5-5 1.41-1.41L10 14.17l7.59-7.59L19 8l-9 9z"/>
          </svg>
        </div>
        <div className={styles.content}>
          <p className={styles.message}>{message}</p>
          <p className={styles.subtitle}>Takes less than 2 minutes</p>
        </div>
        <button
          onClick={() => setShowQuestionnaire(true)}
          className={styles.button}
        >
          Complete Profile
        </button>
      </div>

      <QuestionnaireModal
        isOpen={showQuestionnaire}
        onClose={() => setShowQuestionnaire(false)}
      />
    </>
  );
}

export default CompleteProfilePrompt;
