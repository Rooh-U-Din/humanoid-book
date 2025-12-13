/**
 * Login Prompt Component (T038)
 * Displayed when unauthenticated users try to access protected content
 * Provides options to sign up or sign in
 */

import React, { useState } from 'react';
import { AuthModal } from './AuthModal';
import styles from './LoginPrompt.module.css';

interface LoginPromptProps {
  /** Custom title for the prompt */
  title?: string;
  /** Custom description message */
  description?: string;
  /** Callback when user successfully authenticates */
  onAuthenticated?: () => void;
}

export function LoginPrompt({
  title = 'Sign in to access this content',
  description = 'Create a free account or sign in to unlock all chapters, modules, and interactive features.',
  onAuthenticated,
}: LoginPromptProps) {
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [defaultTab, setDefaultTab] = useState<'signin' | 'signup'>('signin');

  const handleSignIn = () => {
    setDefaultTab('signin');
    setShowAuthModal(true);
  };

  const handleSignUp = () => {
    setDefaultTab('signup');
    setShowAuthModal(true);
  };

  const handleSuccess = () => {
    setShowAuthModal(false);
    onAuthenticated?.();
  };

  return (
    <>
      <div className={styles.container}>
        <div className={styles.icon}>🔒</div>
        <h2 className={styles.title}>{title}</h2>
        <p className={styles.description}>{description}</p>

        <div className={styles.actions}>
          <button className={styles.primaryButton} onClick={handleSignUp}>
            Create Free Account
          </button>
          <button className={styles.secondaryButton} onClick={handleSignIn}>
            Sign In
          </button>
        </div>

        <div className={styles.features}>
          <p className={styles.featuresTitle}>What you'll get access to:</p>
          <div className={styles.featuresList}>
            <div className={styles.feature}>
              <span className={styles.featureIcon}>✓</span>
              <span>All book chapters</span>
            </div>
            <div className={styles.feature}>
              <span className={styles.featureIcon}>✓</span>
              <span>Interactive examples</span>
            </div>
            <div className={styles.feature}>
              <span className={styles.featureIcon}>✓</span>
              <span>Personalized content</span>
            </div>
            <div className={styles.feature}>
              <span className={styles.featureIcon}>✓</span>
              <span>Progress tracking</span>
            </div>
          </div>
        </div>
      </div>

      <AuthModal
        isOpen={showAuthModal}
        onClose={() => setShowAuthModal(false)}
        onSuccess={handleSuccess}
        defaultTab={defaultTab}
      />
    </>
  );
}

/**
 * Loading state component (T039)
 * Shown while checking authentication status
 */
export function AuthLoading() {
  return (
    <div className={styles.loadingOverlay}>
      <div className={styles.spinner} />
      <p className={styles.loadingText}>Checking authentication...</p>
    </div>
  );
}
