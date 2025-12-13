/**
 * Email Verification Page (FR-026)
 * Handles email verification token validation
 */

import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../components/Auth/AuthContext';
import styles from './verify-email.module.css';

export default function VerifyEmailPage(): JSX.Element {
  const { verifyEmail, isAuthenticated, isEmailVerified, refreshUser } = useAuth();
  const [status, setStatus] = useState<'loading' | 'success' | 'error' | 'no-token'>('loading');
  const [message, setMessage] = useState('');

  useEffect(() => {
    const verifyToken = async () => {
      // Get token from URL query params
      const params = new URLSearchParams(window.location.search);
      const token = params.get('token');

      if (!token) {
        setStatus('no-token');
        setMessage('No verification token provided. Please check your email for the verification link.');
        return;
      }

      try {
        const result = await verifyEmail(token);
        setStatus('success');
        setMessage(result.message || 'Email verified successfully!');
        // Refresh user data to get updated email_verified status
        await refreshUser();
      } catch (error) {
        setStatus('error');
        setMessage(error instanceof Error ? error.message : 'Failed to verify email. The token may be invalid or expired.');
      }
    };

    verifyToken();
  }, [verifyEmail, refreshUser]);

  return (
    <Layout title="Verify Email" description="Verify your email address">
      <main className={styles.container}>
        <div className={styles.card}>
          {status === 'loading' && (
            <>
              <div className={styles.spinner} />
              <h1>Verifying your email...</h1>
              <p>Please wait while we verify your email address.</p>
            </>
          )}

          {status === 'success' && (
            <>
              <div className={styles.icon}>✓</div>
              <h1>Email Verified!</h1>
              <p>{message}</p>
              <p>You can now access all personalization features.</p>
              <a href="/docs/intro" className={styles.button}>
                Start Learning
              </a>
            </>
          )}

          {status === 'error' && (
            <>
              <div className={styles.iconError}>✕</div>
              <h1>Verification Failed</h1>
              <p>{message}</p>
              {isAuthenticated && !isEmailVerified && (
                <ResendVerificationButton />
              )}
              <a href="/" className={styles.linkButton}>
                Return to Home
              </a>
            </>
          )}

          {status === 'no-token' && (
            <>
              <div className={styles.iconWarning}>!</div>
              <h1>No Token Found</h1>
              <p>{message}</p>
              {isAuthenticated && !isEmailVerified && (
                <ResendVerificationButton />
              )}
              <a href="/" className={styles.linkButton}>
                Return to Home
              </a>
            </>
          )}
        </div>
      </main>
    </Layout>
  );
}

function ResendVerificationButton() {
  const { resendVerification } = useAuth();
  const [isLoading, setIsLoading] = useState(false);
  const [resendMessage, setResendMessage] = useState('');

  const handleResend = async () => {
    setIsLoading(true);
    try {
      const result = await resendVerification();
      setResendMessage(result.message || 'Verification email sent! Please check your inbox.');
    } catch (error) {
      setResendMessage(error instanceof Error ? error.message : 'Failed to resend verification email.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.resendSection}>
      <button
        onClick={handleResend}
        disabled={isLoading}
        className={styles.button}
      >
        {isLoading ? 'Sending...' : 'Resend Verification Email'}
      </button>
      {resendMessage && <p className={styles.resendMessage}>{resendMessage}</p>}
    </div>
  );
}
