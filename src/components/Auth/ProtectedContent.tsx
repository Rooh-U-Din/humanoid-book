/**
 * Protected Content Wrapper (T040, T041)
 * Gates content behind authentication
 * Shows LoginPrompt for unauthenticated users
 * Shows Loading state while checking auth
 */

import React, { ReactNode } from 'react';
import { useAuth } from './AuthContext';
import { LoginPrompt, AuthLoading } from './LoginPrompt';

interface ProtectedContentProps {
  /** Content to show when authenticated */
  children: ReactNode;
  /** Custom title for login prompt */
  loginTitle?: string;
  /** Custom description for login prompt */
  loginDescription?: string;
  /** Fallback component to show instead of LoginPrompt */
  fallback?: ReactNode;
  /** Whether to require email verification (default: false) */
  requireEmailVerification?: boolean;
}

/**
 * ProtectedContent wraps any content that should only be visible to authenticated users.
 *
 * Usage:
 * ```tsx
 * <ProtectedContent>
 *   <p>This content is only visible to logged-in users</p>
 * </ProtectedContent>
 * ```
 */
export function ProtectedContent({
  children,
  loginTitle,
  loginDescription,
  fallback,
  requireEmailVerification = false,
}: ProtectedContentProps) {
  const { isAuthenticated, isEmailVerified, isLoading } = useAuth();

  // Show loading state while checking authentication
  if (isLoading) {
    return <AuthLoading />;
  }

  // Show login prompt if not authenticated
  if (!isAuthenticated) {
    if (fallback) {
      return <>{fallback}</>;
    }
    return (
      <LoginPrompt
        title={loginTitle}
        description={loginDescription}
      />
    );
  }

  // Optionally require email verification
  if (requireEmailVerification && !isEmailVerified) {
    return (
      <LoginPrompt
        title="Please verify your email"
        description="Check your inbox for a verification link to access this content."
      />
    );
  }

  // User is authenticated, show the protected content
  return <>{children}</>;
}
