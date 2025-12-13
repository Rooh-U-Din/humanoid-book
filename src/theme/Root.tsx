/**
 * Root Component - Docusaurus Theme Wrapper
 *
 * This component wraps the entire Docusaurus site and adds:
 * - AuthProvider for user authentication
 * - Authenticated chatbot widget (requires login)
 * See: https://docusaurus.io/docs/swizzling#wrapper-your-site-with-root
 */

import React, { useState } from 'react';
import { AuthProvider } from '../components/Auth';
import AuthenticatedChatbot from '../components/ChatbotWidget/AuthenticatedChatbot';

export default function Root({ children }): JSX.Element {
  const [isChatOpen, setIsChatOpen] = useState(false);

  return (
    <AuthProvider>
      {children}

      {/* Authenticated Chatbot - requires sign-in to use */}
      <AuthenticatedChatbot
        isOpen={isChatOpen}
        onClose={() => setIsChatOpen(false)}
        onOpen={() => setIsChatOpen(true)}
      />
    </AuthProvider>
  );
}
