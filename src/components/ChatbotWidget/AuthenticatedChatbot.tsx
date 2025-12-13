/**
 * Authenticated Chatbot Wrapper
 *
 * Conditionally renders chatbot UI based on authentication state.
 * Shows login prompt for unauthenticated users.
 */

import React, { useState } from 'react';
import { useAuth, AuthModal } from '../Auth';
import ChatbotWidget from './ChatbotWidget';
import FloatingButton from './FloatingButton';
import styles from './ChatbotWidget.module.css';

/**
 * Login prompt shown inside the chatbot container when user is not authenticated
 */
function ChatbotLoginPrompt({ onSignIn }: { onSignIn: () => void }) {
  return (
    <div className={styles.loginPromptContainer}>
      <div className={styles.loginPromptIcon}>🔒</div>
      <h3 className={styles.loginPromptTitle}>Sign in to use AI Assistant</h3>
      <p className={styles.loginPromptDescription}>
        Create a free account or sign in to chat with our AI assistant about
        Physical AI, ROS 2, Gazebo, and robotics.
      </p>
      <button className={styles.loginPromptButton} onClick={onSignIn}>
        Sign In to Continue
      </button>
    </div>
  );
}

interface AuthenticatedChatbotProps {
  /** Whether the chatbot panel is open */
  isOpen: boolean;
  /** Callback to close the chatbot */
  onClose: () => void;
  /** Callback to open the chatbot */
  onOpen: () => void;
}

/**
 * AuthenticatedChatbot wraps the chatbot with authentication gating.
 *
 * - Authenticated users: Full chatbot functionality
 * - Unauthenticated users: Login prompt with sign-in button
 */
export default function AuthenticatedChatbot({
  isOpen,
  onClose,
  onOpen,
}: AuthenticatedChatbotProps): JSX.Element {
  const { isAuthenticated, isLoading } = useAuth();
  const [showAuthModal, setShowAuthModal] = useState(false);

  // Show floating button for all users (authenticated or not)
  // The authentication check happens when they try to open the chat

  const handleFloatingButtonClick = () => {
    if (isAuthenticated) {
      onOpen();
    } else {
      // Open chatbot container with login prompt
      onOpen();
    }
  };

  const handleSignIn = () => {
    setShowAuthModal(true);
  };

  const handleAuthSuccess = () => {
    setShowAuthModal(false);
    // Chatbot will now show full functionality since user is authenticated
  };

  // If chat is not open, just show the floating button
  if (!isOpen) {
    return (
      <>
        <FloatingButton isOpen={false} onClick={handleFloatingButtonClick} />
        <AuthModal
          isOpen={showAuthModal}
          onClose={() => setShowAuthModal(false)}
          onSuccess={handleAuthSuccess}
          defaultTab="signin"
        />
      </>
    );
  }

  // Chat is open - show either login prompt or full chatbot
  return (
    <>
      {isLoading ? (
        // Loading state while checking authentication
        <div className={styles.chatbotContainer}>
          <div className={styles.chatbotHeader}>
            <div className={styles.headerTitle}>
              <span className={styles.headerIcon}>🤖</span>
              <span>AI Course Assistant</span>
            </div>
            <button className={styles.closeButton} onClick={onClose} aria-label="Close chatbot">
              ✕
            </button>
          </div>
          <div className={styles.loadingContainer}>
            <div className={styles.loadingSpinner} />
            <p>Checking authentication...</p>
          </div>
        </div>
      ) : isAuthenticated ? (
        // Authenticated - show full chatbot
        <ChatbotWidget isOpen={true} onClose={onClose} />
      ) : (
        // Not authenticated - show login prompt inside chatbot container
        <div className={styles.chatbotContainer}>
          <div className={styles.chatbotHeader}>
            <div className={styles.headerTitle}>
              <span className={styles.headerIcon}>🤖</span>
              <span>AI Course Assistant</span>
            </div>
            <button className={styles.closeButton} onClick={onClose} aria-label="Close chatbot">
              ✕
            </button>
          </div>
          <ChatbotLoginPrompt onSignIn={handleSignIn} />
        </div>
      )}

      <AuthModal
        isOpen={showAuthModal}
        onClose={() => setShowAuthModal(false)}
        onSuccess={handleAuthSuccess}
        defaultTab="signin"
      />
    </>
  );
}
