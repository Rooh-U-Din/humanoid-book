/**
 * Auth Modal Component
 * Handles Sign In and Sign Up with tabs
 * Shows questionnaire after successful signup (FR-020)
 */

import React, { useState } from 'react';
import { authClient } from '../auth-client';
import { QuestionnaireModal } from './QuestionnaireModal';
import styles from './AuthModal.module.css';

interface AuthModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSuccess?: () => void;
  defaultTab?: 'signin' | 'signup';
}

export function AuthModal({ isOpen, onClose, onSuccess, defaultTab = 'signin' }: AuthModalProps) {
  const [activeTab, setActiveTab] = useState<'signin' | 'signup'>(defaultTab);
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [name, setName] = useState('');
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [showQuestionnaire, setShowQuestionnaire] = useState(false);

  if (!isOpen && !showQuestionnaire) return null;

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setIsLoading(true);

    try {
      if (activeTab === 'signup') {
        await authClient.signUp({ email, password, name: name || undefined });
        // Show questionnaire after successful signup (FR-020)
        setShowQuestionnaire(true);
      } else {
        await authClient.signIn({ email, password });
        onSuccess?.();
        onClose();
      }

      // Clear form
      setEmail('');
      setPassword('');
      setName('');
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Authentication failed');
    } finally {
      setIsLoading(false);
    }
  };

  const handleQuestionnaireClose = () => {
    setShowQuestionnaire(false);
    onSuccess?.();
    onClose();
  };

  const handleOverlayClick = (e: React.MouseEvent) => {
    if (e.target === e.currentTarget) {
      onClose();
    }
  };

  return (
    <div className={styles.overlay} onClick={handleOverlayClick}>
      <div className={styles.modal}>
        <button className={styles.closeButton} onClick={onClose}>
          &times;
        </button>

        <div className={styles.tabs}>
          <button
            className={`${styles.tab} ${activeTab === 'signin' ? styles.activeTab : ''}`}
            onClick={() => setActiveTab('signin')}
          >
            Sign In
          </button>
          <button
            className={`${styles.tab} ${activeTab === 'signup' ? styles.activeTab : ''}`}
            onClick={() => setActiveTab('signup')}
          >
            Sign Up
          </button>
        </div>

        <form onSubmit={handleSubmit} className={styles.form}>
          {activeTab === 'signup' && (
            <div className={styles.field}>
              <label htmlFor="name">Name (optional)</label>
              <input
                id="name"
                type="text"
                value={name}
                onChange={(e) => setName(e.target.value)}
                placeholder="Your name"
              />
            </div>
          )}

          <div className={styles.field}>
            <label htmlFor="email">Email</label>
            <input
              id="email"
              type="email"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              placeholder="you@example.com"
              required
            />
          </div>

          <div className={styles.field}>
            <label htmlFor="password">Password</label>
            <input
              id="password"
              type="password"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              placeholder={activeTab === 'signup' ? 'Min 8 characters' : 'Your password'}
              minLength={activeTab === 'signup' ? 8 : undefined}
              required
            />
          </div>

          {error && <div className={styles.error}>{error}</div>}

          <button type="submit" className={styles.submitButton} disabled={isLoading}>
            {isLoading ? 'Please wait...' : activeTab === 'signin' ? 'Sign In' : 'Sign Up'}
          </button>
        </form>

        <p className={styles.switchText}>
          {activeTab === 'signin' ? (
            <>
              Don't have an account?{' '}
              <button className={styles.linkButton} onClick={() => setActiveTab('signup')}>
                Sign up
              </button>
            </>
          ) : (
            <>
              Already have an account?{' '}
              <button className={styles.linkButton} onClick={() => setActiveTab('signin')}>
                Sign in
              </button>
            </>
          )}
        </p>
      </div>

      {/* Questionnaire shown after successful signup (FR-020) */}
      <QuestionnaireModal
        isOpen={showQuestionnaire}
        onClose={handleQuestionnaireClose}
      />
    </div>
  );
}
