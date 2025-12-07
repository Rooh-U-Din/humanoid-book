/**
 * Message Bubble Component
 *
 * Renders individual messages with citations
 */

import React from 'react';
import type { Citation } from './api';
import styles from './ChatbotWidget.module.css';

export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
  timestamp: Date;
}

interface MessageBubbleProps {
  message: Message;
}

export default function MessageBubble({ message }: MessageBubbleProps): JSX.Element {
  const isUser = message.role === 'user';

  return (
    <div className={`${styles.messageBubble} ${isUser ? styles.userMessage : styles.assistantMessage}`}>
      <div className={styles.messageContent}>
        {message.content}
      </div>

      {/* Citations */}
      {!isUser && message.citations && message.citations.length > 0 && (
        <div className={styles.citations}>
          <div className={styles.citationsLabel}>Sources:</div>
          {message.citations.map((citation, idx) => (
            <a
              key={idx}
              href={citation.url}
              className={styles.citation}
              title={`${citation.chapter_title} - ${citation.section_title}`}
            >
              <span className={styles.citationIcon}>📖</span>
              {citation.chapter_title}
              {citation.relevance_score && (
                <span className={styles.relevanceScore}>
                  ({Math.round(citation.relevance_score * 100)}%)
                </span>
              )}
            </a>
          ))}
        </div>
      )}

      <div className={styles.messageTime}>
        {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
      </div>
    </div>
  );
}
