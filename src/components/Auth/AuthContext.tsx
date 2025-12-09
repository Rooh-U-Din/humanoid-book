/**
 * Auth Context Provider
 * Provides authentication state throughout the app
 */

import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { authClient, User, UserProfile } from '../../lib/auth-client';

interface AuthContextType {
  user: User | null;
  profile: UserProfile | null;
  isAuthenticated: boolean;
  isProfileCompleted: boolean;
  isLoading: boolean;
  signOut: () => Promise<void>;
  refreshProfile: () => Promise<void>;
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
      if (authClient.isAuthenticated()) {
        await authClient.loadProfile();
      }
      setIsLoading(false);
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

  return (
    <AuthContext.Provider
      value={{
        user,
        profile,
        isAuthenticated: !!user,
        isProfileCompleted: !!profile?.profile_completed,
        isLoading,
        signOut,
        refreshProfile,
      }}
    >
      {children}
    </AuthContext.Provider>
  );
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}
