/**
 * Root Component - Docusaurus Theme Wrapper
 *
 * This component wraps the entire Docusaurus site and adds the chatbot widget.
 * See: https://docusaurus.io/docs/swizzling#wrapper-your-site-with-root
 */

import React, { useState } from 'react';
import ChatbotWidget from '../components/ChatbotWidget/ChatbotWidget';
import FloatingButton from '../components/ChatbotWidget/FloatingButton';

export default function Root({ children }): JSX.Element {
  const [isChatOpen, setIsChatOpen] = useState(false);

  return (
    <>
      {children}

      {/* Chatbot UI */}
      <ChatbotWidget
        isOpen={isChatOpen}
        onClose={() => setIsChatOpen(false)}
      />

      <FloatingButton
        isOpen={isChatOpen}
        onClick={() => setIsChatOpen(true)}
      />
    </>
  );
}
