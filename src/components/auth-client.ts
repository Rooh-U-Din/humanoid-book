/**
 * Authentication client for the Physical AI Book
 * Handles JWT-based auth with the FastAPI backend
 */

// API Configuration - Uses environment variable or defaults to localhost
const API_BASE_URL = typeof window !== 'undefined' && window.location.hostname !== 'localhost'
  ? 'https://backend-book-production-1279.up.railway.app'
  : 'http://localhost:8000';

// Types
export interface User {
  id: string;
  email: string;
  name: string | null;
  email_verified: boolean;
  created_at: string;
}

export interface AuthResponse {
  user_id: string;
  email: string;
  name: string | null;
  token: string;
  profile_completed: boolean;
  email_verified?: boolean;
}

export interface UserProfile {
  user_id: string;
  expertise_level: 'beginner' | 'intermediate' | 'expert';
  programming_languages: string[];
  learning_goals: string | null;
  profile_completed: boolean;
  updated_at: string;
}

export interface SignUpData {
  email: string;
  password: string;
  name?: string;
}

export interface SignInData {
  email: string;
  password: string;
}

export interface QuestionnaireData {
  expertise_level: 'beginner' | 'intermediate' | 'expert';
  programming_languages: string[];
  learning_goals?: string;
  programming_experience_years?: number;
  robotics_experience?: 'none' | 'hobbyist' | 'professional';
  primary_interest?: 'simulation' | 'hardware' | 'ai' | 'all';
  preferred_learning_style?: 'examples' | 'theory' | 'projects';
  time_commitment?: 'casual' | 'moderate' | 'intensive';
}

// Storage keys
const TOKEN_KEY = 'auth_token';
const USER_KEY = 'auth_user';
const PROFILE_KEY = 'auth_profile';

// Auth client class
class AuthClient {
  private token: string | null = null;
  private user: User | null = null;
  private profile: UserProfile | null = null;
  private listeners: Set<() => void> = new Set();

  constructor() {
    // Load from localStorage on init
    if (typeof window !== 'undefined') {
      this.token = localStorage.getItem(TOKEN_KEY);
      const userJson = localStorage.getItem(USER_KEY);
      const profileJson = localStorage.getItem(PROFILE_KEY);

      if (userJson) {
        try {
          this.user = JSON.parse(userJson);
        } catch (e) {
          console.error('Failed to parse user from storage');
        }
      }

      if (profileJson) {
        try {
          this.profile = JSON.parse(profileJson);
        } catch (e) {
          console.error('Failed to parse profile from storage');
        }
      }
    }
  }

  private notifyListeners() {
    this.listeners.forEach(listener => listener());
  }

  subscribe(listener: () => void): () => void {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  private async request<T>(
    endpoint: string,
    options: RequestInit = {}
  ): Promise<T> {
    const headers: Record<string, string> = {
      'Content-Type': 'application/json',
      ...((options.headers as Record<string, string>) || {}),
    };

    if (this.token) {
      headers['Authorization'] = `Bearer ${this.token}`;
    }

    const response = await fetch(`${API_BASE_URL}${endpoint}`, {
      ...options,
      headers,
    });

    if (!response.ok) {
      const error = await response.json().catch(() => ({ detail: 'Request failed' }));
      throw new Error(error.detail || 'Request failed');
    }

    return response.json();
  }

  async signUp(data: SignUpData): Promise<AuthResponse> {
    const response = await this.request<AuthResponse>('/api/auth/signup', {
      method: 'POST',
      body: JSON.stringify(data),
    });

    this.token = response.token;
    this.user = {
      id: response.user_id,
      email: response.email,
      name: response.name,
      email_verified: response.email_verified ?? false,
      created_at: new Date().toISOString(),
    };

    // Save to localStorage
    localStorage.setItem(TOKEN_KEY, this.token);
    localStorage.setItem(USER_KEY, JSON.stringify(this.user));

    this.notifyListeners();
    return response;
  }

  async signIn(data: SignInData): Promise<AuthResponse> {
    const response = await this.request<AuthResponse>('/api/auth/signin', {
      method: 'POST',
      body: JSON.stringify(data),
    });

    this.token = response.token;
    this.user = {
      id: response.user_id,
      email: response.email,
      name: response.name,
      email_verified: response.email_verified ?? false,
      created_at: new Date().toISOString(),
    };

    // Save to localStorage
    localStorage.setItem(TOKEN_KEY, this.token);
    localStorage.setItem(USER_KEY, JSON.stringify(this.user));

    // Load profile if completed
    if (response.profile_completed) {
      await this.loadProfile();
    }

    this.notifyListeners();
    return response;
  }

  async signOut(): Promise<void> {
    try {
      await this.request('/api/auth/signout', { method: 'POST' });
    } catch (e) {
      // Ignore errors on signout
    }

    this.token = null;
    this.user = null;
    this.profile = null;

    localStorage.removeItem(TOKEN_KEY);
    localStorage.removeItem(USER_KEY);
    localStorage.removeItem(PROFILE_KEY);

    this.notifyListeners();
  }

  async loadProfile(): Promise<UserProfile | null> {
    if (!this.token) return null;

    try {
      this.profile = await this.request<UserProfile>('/api/profile');
      localStorage.setItem(PROFILE_KEY, JSON.stringify(this.profile));
      this.notifyListeners();
      return this.profile;
    } catch (e) {
      console.error('Failed to load profile:', e);
      return null;
    }
  }

  async updateProfile(data: Partial<QuestionnaireData>): Promise<UserProfile> {
    const response = await this.request<UserProfile>('/api/profile', {
      method: 'PUT',
      body: JSON.stringify(data),
    });

    this.profile = response;
    localStorage.setItem(PROFILE_KEY, JSON.stringify(this.profile));
    this.notifyListeners();
    return response;
  }

  async submitQuestionnaire(data: QuestionnaireData): Promise<UserProfile> {
    const response = await this.request<UserProfile>('/api/profile/questionnaire', {
      method: 'POST',
      body: JSON.stringify(data),
    });

    this.profile = response;
    localStorage.setItem(PROFILE_KEY, JSON.stringify(this.profile));
    this.notifyListeners();
    return response;
  }

  // Getters
  getToken(): string | null {
    return this.token;
  }

  getUser(): User | null {
    return this.user;
  }

  getProfile(): UserProfile | null {
    return this.profile;
  }

  isAuthenticated(): boolean {
    return !!this.token && !!this.user;
  }

  isProfileCompleted(): boolean {
    return !!this.profile?.profile_completed;
  }

  isEmailVerified(): boolean {
    return !!this.user?.email_verified;
  }

  // Email verification methods (FR-026)
  async verifyEmail(token: string): Promise<{ message: string; email_verified: boolean }> {
    const response = await this.request<{ message: string; email_verified: boolean }>(
      '/api/auth/verify-email',
      {
        method: 'POST',
        body: JSON.stringify({ token }),
      }
    );

    // Update user's email_verified status
    if (this.user && response.email_verified) {
      this.user.email_verified = true;
      localStorage.setItem(USER_KEY, JSON.stringify(this.user));
      this.notifyListeners();
    }

    return response;
  }

  async resendVerification(): Promise<{ message: string }> {
    return this.request<{ message: string }>('/api/auth/resend-verification', {
      method: 'POST',
    });
  }

  // Load current user info from /api/auth/me
  async loadCurrentUser(): Promise<User | null> {
    if (!this.token) return null;

    try {
      const userData = await this.request<{
        id: string;
        email: string;
        name: string | null;
        email_verified: boolean;
        created_at: string;
      }>('/api/auth/me');

      this.user = {
        id: userData.id,
        email: userData.email,
        name: userData.name,
        email_verified: userData.email_verified,
        created_at: userData.created_at,
      };
      localStorage.setItem(USER_KEY, JSON.stringify(this.user));
      this.notifyListeners();
      return this.user;
    } catch (e) {
      console.error('Failed to load current user:', e);
      return null;
    }
  }

  // Password change (T069)
  async changePassword(currentPassword: string, newPassword: string): Promise<{ message: string }> {
    return this.request<{ message: string }>('/api/auth/change-password', {
      method: 'POST',
      body: JSON.stringify({
        current_password: currentPassword,
        new_password: newPassword,
      }),
    });
  }
}

// Export singleton instance
export const authClient = new AuthClient();
