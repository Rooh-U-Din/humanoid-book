/**
 * TextSelectionProvider Component
 *
 * Detects text selection within documentation pages and displays
 * the SkillPanel when text is selected.
 *
 * Usage:
 * ```tsx
 * <TextSelectionProvider chapterId="ros2-basics">
 *   <DocItem {...props} />
 * </TextSelectionProvider>
 * ```
 */

import React, { useState, useEffect, useCallback, useRef } from 'react';
import { SkillPanel } from './SkillPanel';

interface TextSelectionProviderProps {
  /** The current chapter ID for context */
  chapterId?: string;
  /** Children to wrap (typically the doc content) */
  children: React.ReactNode;
}

interface SelectionState {
  text: string;
  position: { top: number; left: number };
}

/**
 * Minimum characters required to show the skill panel.
 */
const MIN_SELECTION_LENGTH = 10;

/**
 * Delay before showing panel (prevents flickering on quick selections).
 */
const SELECTION_DELAY_MS = 300;

/**
 * TextSelectionProvider wraps content and provides text selection detection.
 *
 * When text is selected, a SkillPanel appears near the selection,
 * allowing users to invoke AI skills on the selected content.
 */
export function TextSelectionProvider({
  chapterId,
  children,
}: TextSelectionProviderProps): JSX.Element {
  const [selection, setSelection] = useState<SelectionState | null>(null);
  const [showPanel, setShowPanel] = useState(false);
  const containerRef = useRef<HTMLDivElement>(null);
  const timeoutRef = useRef<NodeJS.Timeout | null>(null);

  /**
   * Get the position for the skill panel based on selection.
   */
  const getSelectionPosition = useCallback((): { top: number; left: number } | null => {
    const windowSelection = window.getSelection();
    if (!windowSelection || windowSelection.rangeCount === 0) return null;

    const range = windowSelection.getRangeAt(0);
    const rect = range.getBoundingClientRect();

    // Position panel below and slightly to the right of selection
    return {
      top: rect.bottom + window.scrollY + 8,
      left: Math.min(
        rect.left + window.scrollX,
        window.innerWidth - 380 // Ensure panel doesn't overflow viewport
      ),
    };
  }, []);

  /**
   * Handle text selection changes.
   */
  const handleSelectionChange = useCallback(() => {
    // Clear any pending timeout
    if (timeoutRef.current) {
      clearTimeout(timeoutRef.current);
    }

    const windowSelection = window.getSelection();
    const selectedText = windowSelection?.toString().trim() || '';

    // Check if selection is within our container
    if (
      containerRef.current &&
      windowSelection?.anchorNode &&
      !containerRef.current.contains(windowSelection.anchorNode)
    ) {
      // Selection is outside our container
      return;
    }

    if (selectedText.length < MIN_SELECTION_LENGTH) {
      // Selection too short - hide panel
      setShowPanel(false);
      setSelection(null);
      return;
    }

    // Delay showing panel to prevent flickering
    timeoutRef.current = setTimeout(() => {
      const position = getSelectionPosition();
      if (position) {
        setSelection({
          text: selectedText,
          position,
        });
        setShowPanel(true);
      }
    }, SELECTION_DELAY_MS);
  }, [getSelectionPosition]);

  /**
   * Handle click outside to close panel.
   */
  const handleClickOutside = useCallback((event: MouseEvent) => {
    const target = event.target as HTMLElement;

    // Don't close if clicking within the skill panel
    if (target.closest('[data-skill-panel]')) {
      return;
    }

    // Don't close if clicking within a selection
    const windowSelection = window.getSelection();
    if (windowSelection && windowSelection.toString().trim().length >= MIN_SELECTION_LENGTH) {
      return;
    }

    // Close panel
    setShowPanel(false);
    setSelection(null);
  }, []);

  /**
   * Handle panel close.
   */
  const handleClosePanel = useCallback(() => {
    setShowPanel(false);
    setSelection(null);

    // Clear text selection
    window.getSelection()?.removeAllRanges();
  }, []);

  // Set up selection change listener
  useEffect(() => {
    document.addEventListener('selectionchange', handleSelectionChange);
    document.addEventListener('mousedown', handleClickOutside);

    return () => {
      document.removeEventListener('selectionchange', handleSelectionChange);
      document.removeEventListener('mousedown', handleClickOutside);

      if (timeoutRef.current) {
        clearTimeout(timeoutRef.current);
      }
    };
  }, [handleSelectionChange, handleClickOutside]);

  return (
    <div ref={containerRef} data-text-selection-provider>
      {children}

      {showPanel && selection && (
        <div data-skill-panel>
          <SkillPanel
            selectedText={selection.text}
            chapterId={chapterId}
            position={selection.position}
            onClose={handleClosePanel}
          />
        </div>
      )}
    </div>
  );
}

export default TextSelectionProvider;
