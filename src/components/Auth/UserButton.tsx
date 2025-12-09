/**
 * User Button Component
 * Shows sign in button when logged out, user menu when logged in
 */

import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from './AuthContext';
import { AuthModal } from './AuthModal';
import { QuestionnaireModal } from './QuestionnaireModal';
import styles from './UserButton.module.css';

export function UserButton() {
  const { user, profile, isAuthenticated, isProfileCompleted, signOut } = useAuth();
  const [showAuthModal, setShowAuthModal] = useState(false);
  const [showQuestionnaire, setShowQuestionnaire] = useState(false);
  const [showDropdown, setShowDropdown] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  // Close dropdown when clicking outside
  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setShowDropdown(false);
      }
    }

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  // Show questionnaire after successful signup
  const handleAuthSuccess = () => {
    if (!isProfileCompleted) {
      setShowQuestionnaire(true);
    }
  };

  if (!isAuthenticated) {
    return (
      <>
        <button className={styles.signInButton} onClick={() => setShowAuthModal(true)}>
          Sign In
        </button>

        <AuthModal
          isOpen={showAuthModal}
          onClose={() => setShowAuthModal(false)}
          onSuccess={handleAuthSuccess}
        />

        <QuestionnaireModal
          isOpen={showQuestionnaire}
          onClose={() => setShowQuestionnaire(false)}
        />
      </>
    );
  }

  const displayName = user?.name || user?.email?.split('@')[0] || 'User';
  const initials = displayName.charAt(0).toUpperCase();

  return (
    <>
      <div className={styles.container} ref={dropdownRef}>
        <button
          className={styles.userButton}
          onClick={() => setShowDropdown(!showDropdown)}
          aria-expanded={showDropdown}
        >
          <div className={styles.avatar}>{initials}</div>
          <span className={styles.userName}>{displayName}</span>
          <svg
            className={`${styles.chevron} ${showDropdown ? styles.chevronUp : ''}`}
            width="12"
            height="12"
            viewBox="0 0 12 12"
          >
            <path d="M2 4l4 4 4-4" fill="none" stroke="currentColor" strokeWidth="2" />
          </svg>
        </button>

        {showDropdown && (
          <div className={styles.dropdown}>
            <div className={styles.userInfo}>
              <div className={styles.userEmail}>{user?.email}</div>
              {profile && (
                <>
                  <div className={styles.profileBadge}>
                    <span className={`${styles.levelBadge} ${styles[profile.expertise_level]}`}>
                      {profile.expertise_level}
                    </span>
                  </div>
                  {profile.programming_languages && profile.programming_languages.length > 0 && (
                    <div className={styles.languages}>
                      {profile.programming_languages.slice(0, 3).map(lang => (
                        <span key={lang} className={styles.langTag}>{lang}</span>
                      ))}
                      {profile.programming_languages.length > 3 && (
                        <span className={styles.langTag}>+{profile.programming_languages.length - 3}</span>
                      )}
                    </div>
                  )}
                </>
              )}
            </div>

            <div className={styles.divider} />

            {!isProfileCompleted ? (
              <button
                className={styles.dropdownItem}
                onClick={() => {
                  setShowDropdown(false);
                  setShowQuestionnaire(true);
                }}
              >
                Complete Profile
              </button>
            ) : (
              <button
                className={styles.dropdownItem}
                onClick={() => {
                  setShowDropdown(false);
                  setShowQuestionnaire(true);
                }}
              >
                Edit Profile
              </button>
            )}

            <button
              className={styles.dropdownItem}
              onClick={async () => {
                setShowDropdown(false);
                await signOut();
              }}
            >
              Sign Out
            </button>
          </div>
        )}
      </div>

      <QuestionnaireModal
        isOpen={showQuestionnaire}
        onClose={() => setShowQuestionnaire(false)}
      />
    </>
  );
}
