/**
 * SkillResult Component
 *
 * Displays the result of a skill invocation with content,
 * citations, suggestions, and trace ID for debugging.
 */

import React from 'react';
import { SkillInvokeResponse, Suggestion } from './api';
import styles from './SkillResult.module.css';

interface SkillResultProps {
  response: SkillInvokeResponse;
  onSuggestionClick?: (suggestion: Suggestion) => void;
  showTraceId?: boolean;
}

/**
 * Renders skill execution results with formatted content and metadata.
 */
export function SkillResult({
  response,
  onSuggestionClick,
  showTraceId = false,
}: SkillResultProps): JSX.Element {
  const { result, trace_id, latency_ms, skill_id } = response;

  return (
    <div className={styles.skillResult}>
      {/* Main content */}
      <div className={styles.content}>
        <div
          className={styles.contentText}
          dangerouslySetInnerHTML={{
            __html: formatContent(result.content),
          }}
        />
      </div>

      {/* Citations */}
      {result.citations && result.citations.length > 0 && (
        <div className={styles.citations}>
          <h4 className={styles.sectionTitle}>Related Content</h4>
          <ul className={styles.citationList}>
            {result.citations.map((citation, index) => (
              <li key={index} className={styles.citation}>
                {citation.url ? (
                  <a href={citation.url} className={styles.citationLink}>
                    {citation.chapter_title || citation.section_title || citation.chapter_id}
                  </a>
                ) : (
                  <span>{citation.chapter_title || citation.section_title || citation.chapter_id}</span>
                )}
              </li>
            ))}
          </ul>
        </div>
      )}

      {/* Suggestions */}
      {result.suggestions && result.suggestions.length > 0 && (
        <div className={styles.suggestions}>
          <h4 className={styles.sectionTitle}>Next Steps</h4>
          <div className={styles.suggestionList}>
            {result.suggestions.map((suggestion, index) => (
              <button
                key={index}
                className={styles.suggestionButton}
                onClick={() => onSuggestionClick?.(suggestion)}
                title={suggestion.description}
              >
                {getSuggestionIcon(suggestion.type)}
                <span>{suggestion.label}</span>
              </button>
            ))}
          </div>
        </div>
      )}

      {/* Metadata footer */}
      <div className={styles.metadata}>
        <span className={styles.skillBadge}>{skill_id}</span>
        <span className={styles.latency}>{latency_ms}ms</span>
        {showTraceId && (
          <span className={styles.traceId} title="Trace ID for debugging">
            {trace_id.slice(0, 8)}...
          </span>
        )}
      </div>
    </div>
  );
}

/**
 * Format markdown-like content to HTML.
 * Basic formatting for bold, code blocks, and line breaks.
 */
function formatContent(content: string): string {
  return content
    // Code blocks
    .replace(/```(\w*)\n([\s\S]*?)```/g, '<pre><code class="language-$1">$2</code></pre>')
    // Inline code
    .replace(/`([^`]+)`/g, '<code>$1</code>')
    // Bold
    .replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>')
    // Italic
    .replace(/\*([^*]+)\*/g, '<em>$1</em>')
    // Line breaks
    .replace(/\n/g, '<br />');
}

/**
 * Get icon for suggestion type.
 */
function getSuggestionIcon(type: string): string {
  switch (type) {
    case 'skill':
      return '🔧';
    case 'chapter':
      return '📖';
    case 'topic':
      return '🏷️';
    default:
      return '→';
  }
}

export default SkillResult;
