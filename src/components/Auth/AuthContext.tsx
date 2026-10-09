/**
 * Auth Context Provider
 * Provides authentication state throughout the app
 */

import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { authClient, User, UserProfile } from '../auth-client';

interface AuthContextType {
  user: User | null;
  profile: UserProfile | null;
  isAuthenticated: boolean;
  isEmailVerified: boolean;
  isProfileCompleted: boolean;
  isLoading: boolean;
  signOut: () => Promise<void>;
  refreshProfile: () => Promise<void>;
  refreshUser: () => Promise<void>;
  verifyEmail: (token: string) => Promise<{ message: string; email_verified: boolean }>;
  resendVerification: () => Promise<{ message: string }>;
}

const AuthContext = createContext<AuthContextType | null>(null);

export function AuthProvider({ children }: { children: ReactNode }) {
  // Start with null to avoid hydration mismatch (server has no localStorage)
  const [user, setUser] = useState<User | null>(null);
  const [profile, setProfile] = useState<UserProfile | null>(null);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    // Load initial state from authClient after hydration (client-side only)
    setUser(authClient.getUser());
    setProfile(authClient.getProfile());

    // Subscribe to auth changes
    const unsubscribe = authClient.subscribe(() => {
      setUser(authClient.getUser());
      setProfile(authClient.getProfile());
    });

    // Initial profile load if authenticated
    const initAuth = async () => {
      try {
        if (authClient.isAuthenticated()) {
          await authClient.loadProfile();
        }
      } catch (err) {
        console.warn('Failed to load user profile on init:', err);
      } finally {
        setIsLoading(false);
      }
    };

    initAuth();

    return unsubscribe;
  }, []);

  const signOut = async () => {
    await authClient.signOut();
  };

  const refreshProfile = async () => {
    await authClient.loadProfile();
  };

  const refreshUser = async () => {
    await authClient.loadCurrentUser();
  };

  const verifyEmail = async (token: string) => {
    return authClient.verifyEmail(token);
  };

  const resendVerification = async () => {
    return authClient.resendVerification();
  };

  return (
    <AuthContext.Provider
      value={{
        user,
        profile,
        isAuthenticated: !!user,
        isEmailVerified: !!user?.email_verified,
        isProfileCompleted: !!profile?.profile_completed,
        isLoading,
        signOut,
        refreshProfile,
        refreshUser,
        verifyEmail,
        resendVerification,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
}

const defaultAuthContext: AuthContextType = {
  user: null,
  profile: null,
  isAuthenticated: false,
  isEmailVerified: false,
  isProfileCompleted: false,
  isLoading: false,
  signOut: async () => {},
  refreshProfile: async () => {},
  refreshUser: async () => {},
  verifyEmail: async () => ({ message: '', email_verified: false }),
  resendVerification: async () => ({ message: '' }),
};

export function useAuth(): AuthContextType {
  const context = useContext(AuthContext);
  if (!context) {
    return defaultAuthContext;
  }
  return context;
}
