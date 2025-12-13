/**
 * Verify Email Prompt Component (FR-026)
 * Shows a prompt to unverified users to verify their email
 */

import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import styles from './VerifyEmailPrompt.module.css';

interface VerifyEmailPromptProps {
  /** Message to show explaining why verification is needed */
  message?: string;
  /** Whether to show as a banner or inline */
  variant?: 'banner' | 'inline';
}

export function VerifyEmailPrompt({
  message = 'Please verify your email to access personalization features.',
  variant = 'banner'
}: VerifyEmailPromptProps) {
  const { isAuthenticated, isEmailVerified, user, resendVerification } = useAuth();
  const [isLoading, setIsLoading] = useState(false);
  const [feedback, setFeedback] = useState<{ type: 'success' | 'error'; message: string } | null>(null);

  // Don't show if not authenticated or already verified
  if (!isAuthenticated || isEmailVerified) {
    return null;
  }

  const handleResend = async () => {
    setIsLoading(true);
    setFeedback(null);

    try {
      await resendVerification();
      setFeedback({
        type: 'success',
        message: 'Verification email sent! Please check your inbox.',
      });
    } catch (error) {
      setFeedback({
        type: 'error',
        message: error instanceof Error ? error.message : 'Failed to send verification email.',
      });
    } finally {
      setIsLoading(false);
    }
  };

  const containerClass = variant === 'banner' ? styles.banner : styles.inline;

  return (
    <div className={containerClass}>
      <div className={styles.icon}>
        <svg viewBox="0 0 24 24" width="20" height="20" fill="currentColor">
          <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm1 15h-2v-2h2v2zm0-4h-2V7h2v6z"/>
        </svg>
      </div>
      <div className={styles.content}>
        <p className={styles.message}>{message}</p>
        {user?.email && (
          <p className={styles.email}>
            We sent a verification link to <strong>{user.email}</strong>
          </p>
        )}
        {feedback && (
          <p className={feedback.type === 'success' ? styles.success : styles.error}>
            {feedback.message}
          </p>
        )}
      </div>
      <button
        onClick={handleResend}
        disabled={isLoading}
        className={styles.button}
      >
        {isLoading ? 'Sending...' : 'Resend Email'}
      </button>
    </div>
  );
}

export default VerifyEmailPrompt;
