/**
 * Floating Chatbot Button
 *
 * Button that opens the chatbot widget
 */

import React from 'react';
import styles from './ChatbotWidget.module.css';

interface FloatingButtonProps {
  onClick: () => void;
  isOpen: boolean;
}

export default function FloatingButton({ onClick, isOpen }: FloatingButtonProps): JSX.Element {
  if (isOpen) {
    return null; // Hide button when chat is open
  }

  return (
    <button
      className={styles.floatingButton}
      onClick={onClick}
      aria-label="Open AI assistant"
      title="Ask the AI assistant"
    >
      <span className={styles.buttonIcon}>🤖</span>
      <span className={styles.buttonPulse}></span>
    </button>
  );
}
