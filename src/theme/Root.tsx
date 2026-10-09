/**
 * Root Component - Docusaurus Theme Wrapper
 *
 * This component wraps the entire Docusaurus site and adds:
 * - AuthProvider for user authentication
 * - Authenticated chatbot widget (requires login)
 * See: https://docusaurus.io/docs/swizzling#wrapper-your-site-with-root
 */

import React, { useState, Component, ErrorInfo, ReactNode } from 'react';
import { AuthProvider } from '../components/Auth';
import AuthenticatedChatbot from '../components/ChatbotWidget/AuthenticatedChatbot';

interface ErrorBoundaryProps {
  children: ReactNode;
  fallback?: ReactNode;
}

interface ErrorBoundaryState {
  hasError: boolean;
  error: Error | null;
}

class RootErrorBoundary extends Component<ErrorBoundaryProps, ErrorBoundaryState> {
  constructor(props: ErrorBoundaryProps) {
    super(props);
    this.state = { hasError: false, error: null };
  }

  static getDerivedStateFromError(error: Error): ErrorBoundaryState {
    return { hasError: true, error };
  }

  componentDidCatch(error: Error, errorInfo: ErrorInfo) {
    console.error('RootErrorBoundary caught error:', error, errorInfo);
  }

  render() {
    if (this.state.hasError) {
      if (this.props.fallback) {
        return this.props.fallback;
      }
      return (
        <div style={{ padding: '2rem', textAlign: 'center', fontFamily: 'sans-serif' }}>
          <h2>Something went wrong loading this component.</h2>
          <pre style={{ color: 'red', textAlign: 'left', maxWidth: '800px', margin: '1rem auto', overflow: 'auto' }}>
            {this.state.error?.toString()}
          </pre>
          <button
            onClick={() => this.setState({ hasError: false, error: null })}
            style={{ padding: '0.5rem 1rem', cursor: 'pointer', borderRadius: '4px', border: '1px solid #ccc' }}
          >
            Try Again
          </button>
        </div>
      );
    }
    return this.props.children;
  }
}

export default function Root({ children }: { children: ReactNode }): JSX.Element {
  const [isChatOpen, setIsChatOpen] = useState(false);

  return (
    <RootErrorBoundary>
      <AuthProvider>
        {children}

        {/* Authenticated Chatbot - requires sign-in to use */}
        <RootErrorBoundary fallback={null}>
          <AuthenticatedChatbot
            isOpen={isChatOpen}
            onClose={() => setIsChatOpen(false)}
            onOpen={() => setIsChatOpen(true)}
          />
        </RootErrorBoundary>
      </AuthProvider>
    </RootErrorBoundary>
  );
}
