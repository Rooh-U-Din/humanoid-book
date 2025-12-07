/**
 * Chatbot Widget Component
 *
 * Main chatbot interface with message list and input
 */

import React, { useState, useEffect, useRef } from 'react';
import MessageBubble, { type Message } from './MessageBubble';
import { sendQuery, healthCheck } from './api';
import styles from './ChatbotWidget.module.css';

// Generate a session ID (stored in localStorage)
// This function is only called client-side
function getSessionId(): string {
  if (typeof window === 'undefined') {
    // SSR: return a temporary ID
    return `session_ssr_${Math.random().toString(36).substr(2, 9)}`;
  }
  let sessionId = localStorage.getItem('chatbot_session_id');
  if (!sessionId) {
    sessionId = `session_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
    localStorage.setItem('chatbot_session_id', sessionId);
  }
  return sessionId;
}

interface ChatbotWidgetProps {
  isOpen: boolean;
  onClose: () => void;
}

export default function ChatbotWidget({ isOpen, onClose }: ChatbotWidgetProps): JSX.Element | null {
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [backendStatus, setBackendStatus] = useState<'checking' | 'healthy' | 'degraded' | 'offline'>('checking');
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const sessionId = useRef<string | null>(null);

  // Initialize session ID on client-side only
  useEffect(() => {
    if (!sessionId.current) {
      sessionId.current = getSessionId();
    }
  }, []);

  // Check backend health on open
  useEffect(() => {
    if (isOpen && backendStatus === 'checking') {
      healthCheck()
        .then((result) => {
          setBackendStatus(result.status === 'healthy' ? 'healthy' : 'degraded');
        })
        .catch(() => {
          setBackendStatus('offline');
        });
    }
  }, [isOpen, backendStatus]);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Welcome message based on backend status
  useEffect(() => {
    if (messages.length === 0 && isOpen && backendStatus !== 'checking') {
      let welcomeContent = 'Hi! I\'m your AI assistant for the Physical AI & Humanoid Robotics course. Ask me anything about ROS 2, Gazebo, robotics, or any topic from the book!';

      if (backendStatus === 'offline') {
        welcomeContent = '⚠️ The AI backend is currently offline. Please make sure the backend server is running at localhost:8000.\n\nYou can browse the course content in the meantime!';
      } else if (backendStatus === 'degraded') {
        welcomeContent = '⚡ Some AI services may be limited. I\'ll do my best to help you with your questions about Physical AI & Humanoid Robotics!';
      }

      setMessages([{
        id: 'welcome',
        role: 'assistant',
        content: welcomeContent,
        timestamp: new Date(),
      }]);
    }
  }, [isOpen, messages.length, backendStatus]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!inputValue.trim() || isLoading) {
      return;
    }

    const userMessage: Message = {
      id: `user_${Date.now()}`,
      role: 'user',
      content: inputValue.trim(),
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      const response = await sendQuery(userMessage.content, sessionId.current || 'anonymous');

      const assistantMessage: Message = {
        id: `assistant_${Date.now()}`,
        role: 'assistant',
        content: response.answer,
        citations: response.citations,
        timestamp: new Date(response.timestamp),
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to get response');

      const errorMessage: Message = {
        id: `error_${Date.now()}`,
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again or rephrase your question.',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  if (!isOpen) {
    return null;
  }

  return (
    <div className={styles.chatbotContainer}>
      {/* Header */}
      <div className={styles.chatbotHeader}>
        <div className={styles.headerTitle}>
          <span className={styles.headerIcon}>🤖</span>
          <span>AI Course Assistant</span>
          <span
            className={styles.statusIndicator}
            style={{
              backgroundColor: backendStatus === 'healthy' ? '#4ade80' :
                              backendStatus === 'degraded' ? '#fbbf24' :
                              backendStatus === 'offline' ? '#f87171' : '#9ca3af',
            }}
            title={`Status: ${backendStatus}`}
          />
        </div>
        <button
          className={styles.closeButton}
          onClick={onClose}
          aria-label="Close chatbot"
        >
          ✕
        </button>
      </div>

      {/* Messages */}
      <div className={styles.messagesContainer}>
        {messages.map(message => (
          <MessageBubble key={message.id} message={message} />
        ))}

        {/* Loading indicator */}
        {isLoading && (
          <div className={styles.loadingIndicator}>
            <div className={styles.typingDots}>
              <span></span>
              <span></span>
              <span></span>
            </div>
            <span className={styles.loadingText}>Thinking...</span>
          </div>
        )}

        {/* Error message */}
        {error && (
          <div className={styles.errorMessage}>
            ⚠️ {error}
          </div>
        )}

        <div ref={messagesEndRef} />
      </div>

      {/* Input */}
      <form className={styles.inputForm} onSubmit={handleSubmit}>
        <input
          type="text"
          className={styles.input}
          placeholder={backendStatus === 'offline' ? 'Backend offline...' : 'Ask about ROS 2, Gazebo, robotics...'}
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          disabled={isLoading || backendStatus === 'offline'}
          maxLength={500}
        />
        <button
          type="submit"
          className={styles.sendButton}
          disabled={isLoading || !inputValue.trim() || backendStatus === 'offline'}
          aria-label="Send message"
        >
          {isLoading ? '⏳' : '📤'}
        </button>
      </form>

      {/* Footer */}
      <div className={styles.chatbotFooter}>
        Powered by Gemini AI • <a href="/docs/intro" target="_blank">Course Info</a>
      </div>
    </div>
  );
}
