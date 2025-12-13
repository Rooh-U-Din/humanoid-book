/**
 * User Profile Page
 * Displays and allows editing of user profile information
 */

import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../components/Auth/AuthContext';
import { authClient } from '../components/auth-client';
import { QuestionnaireModal } from '../components/Auth/QuestionnaireModal';
import { VerifyEmailPrompt } from '../components/Auth/VerifyEmailPrompt';
import styles from './profile.module.css';

export default function ProfilePage(): JSX.Element {
  const { user, profile, isAuthenticated, isEmailVerified, isLoading, refreshProfile } = useAuth();
  const [showQuestionnaire, setShowQuestionnaire] = useState(false);
  const [isRefreshing, setIsRefreshing] = useState(false);

  // Redirect to home if not authenticated (client-side only)
  useEffect(() => {
    if (!isLoading && !isAuthenticated && typeof window !== 'undefined') {
      window.location.href = '/';
    }
  }, [isLoading, isAuthenticated]);

  const handleRefreshProfile = async () => {
    setIsRefreshing(true);
    await refreshProfile();
    setIsRefreshing(false);
  };

  if (isLoading) {
    return (
      <Layout title="Profile" description="Your user profile">
        <main className={styles.container}>
          <div className={styles.loading}>Loading...</div>
        </main>
      </Layout>
    );
  }

  if (!isAuthenticated) {
    return (
      <Layout title="Profile" description="Your user profile">
        <main className={styles.container}>
          <div className={styles.card}>
            <h1>Sign In Required</h1>
            <p>Please sign in to view your profile.</p>
            <a href="/" className={styles.button}>Go to Home</a>
          </div>
        </main>
      </Layout>
    );
  }

  return (
    <Layout title="Profile" description="Your user profile">
      <main className={styles.container}>
        <div className={styles.header}>
          <h1>Your Profile</h1>
          <p>Manage your account and learning preferences</p>
        </div>

        {!isEmailVerified && (
          <VerifyEmailPrompt
            message="Verify your email to access personalization features."
            variant="banner"
          />
        )}

        <div className={styles.grid}>
          {/* Account Info Card */}
          <div className={styles.card}>
            <h2>Account Information</h2>
            <div className={styles.infoRow}>
              <span className={styles.label}>Email</span>
              <span className={styles.value}>{user?.email}</span>
            </div>
            <div className={styles.infoRow}>
              <span className={styles.label}>Name</span>
              <span className={styles.value}>{user?.name || 'Not set'}</span>
            </div>
            <div className={styles.infoRow}>
              <span className={styles.label}>Email Verified</span>
              <span className={`${styles.value} ${isEmailVerified ? styles.verified : styles.unverified}`}>
                {isEmailVerified ? 'Yes' : 'No'}
              </span>
            </div>
            <div className={styles.infoRow}>
              <span className={styles.label}>Member Since</span>
              <span className={styles.value}>
                {user?.created_at ? new Date(user.created_at).toLocaleDateString() : 'Unknown'}
              </span>
            </div>
          </div>

          {/* Learning Profile Card */}
          <div className={styles.card}>
            <div className={styles.cardHeader}>
              <h2>Learning Profile</h2>
              <button
                onClick={() => setShowQuestionnaire(true)}
                className={styles.editButton}
              >
                {profile?.profile_completed ? 'Edit' : 'Complete'}
              </button>
            </div>

            {profile?.profile_completed ? (
              <>
                <div className={styles.infoRow}>
                  <span className={styles.label}>Expertise Level</span>
                  <span className={`${styles.value} ${styles.badge}`}>
                    {profile.expertise_level}
                  </span>
                </div>
                <div className={styles.infoRow}>
                  <span className={styles.label}>Programming Languages</span>
                  <span className={styles.value}>
                    {profile.programming_languages?.length > 0
                      ? profile.programming_languages.join(', ')
                      : 'Not specified'}
                  </span>
                </div>
                {profile.learning_goals && (
                  <div className={styles.infoRow}>
                    <span className={styles.label}>Learning Goals</span>
                    <span className={styles.value}>{profile.learning_goals}</span>
                  </div>
                )}
                <div className={styles.infoRow}>
                  <span className={styles.label}>Last Updated</span>
                  <span className={styles.value}>
                    {profile.updated_at ? new Date(profile.updated_at).toLocaleDateString() : 'Unknown'}
                  </span>
                </div>
              </>
            ) : (
              <div className={styles.incomplete}>
                <p>Complete your learning profile to get personalized content.</p>
                <button
                  onClick={() => setShowQuestionnaire(true)}
                  className={styles.button}
                >
                  Complete Profile
                </button>
              </div>
            )}
          </div>

          {/* Features Card */}
          <div className={styles.card}>
            <h2>Personalization Features</h2>
            <div className={styles.featureList}>
              <div className={`${styles.feature} ${isEmailVerified && profile?.profile_completed ? styles.active : styles.inactive}`}>
                <span className={styles.featureIcon}>
                  {isEmailVerified && profile?.profile_completed ? '✓' : '○'}
                </span>
                <div>
                  <span className={styles.featureName}>Content Personalization</span>
                  <span className={styles.featureDesc}>
                    {isEmailVerified && profile?.profile_completed
                      ? 'Available - Click "Personalize" on any chapter'
                      : 'Complete profile and verify email to unlock'}
                  </span>
                </div>
              </div>
              <div className={`${styles.feature} ${isEmailVerified ? styles.active : styles.inactive}`}>
                <span className={styles.featureIcon}>
                  {isEmailVerified ? '✓' : '○'}
                </span>
                <div>
                  <span className={styles.featureName}>AI Chatbot</span>
                  <span className={styles.featureDesc}>
                    {isEmailVerified
                      ? 'Available - Use the chat widget'
                      : 'Verify email to unlock'}
                  </span>
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* Actions */}
        <div className={styles.actions}>
          <button
            onClick={handleRefreshProfile}
            disabled={isRefreshing}
            className={styles.secondaryButton}
          >
            {isRefreshing ? 'Refreshing...' : 'Refresh Profile'}
          </button>
          <a href="/docs/intro" className={styles.button}>
            Start Learning
          </a>
        </div>
      </main>

      <QuestionnaireModal
        isOpen={showQuestionnaire}
        onClose={() => {
          setShowQuestionnaire(false);
          handleRefreshProfile();
        }}
      />
    </Layout>
  );
}
