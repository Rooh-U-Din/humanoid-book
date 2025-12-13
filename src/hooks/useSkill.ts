/**
 * useSkill Hook
 *
 * Manages skill invocation state including loading, error handling,
 * and result storage.
 */

import { useState, useCallback } from 'react';
import {
  invokeSkill,
  SkillInput,
  SkillInvokeResponse,
  AuthenticationError,
  SkillNotFoundError,
  RateLimitError,
} from '../components/Skills/api';

export type SkillStatus = 'idle' | 'loading' | 'success' | 'error';

export interface UseSkillState {
  status: SkillStatus;
  result: SkillInvokeResponse | null;
  error: Error | null;
  isLoading: boolean;
  isAuthenticated: boolean;
}

export interface UseSkillActions {
  invoke: (skillId: string, input: SkillInput) => Promise<SkillInvokeResponse | null>;
  reset: () => void;
}

export type UseSkillReturn = UseSkillState & UseSkillActions;

/**
 * Hook for managing skill invocation state.
 *
 * @example
 * ```tsx
 * const { invoke, result, isLoading, error, isAuthenticated } = useSkill();
 *
 * const handleExplain = async () => {
 *   await invoke('explain', { selected_text: code });
 * };
 *
 * if (!isAuthenticated) {
 *   return <LoginPrompt />;
 * }
 *
 * if (isLoading) {
 *   return <Spinner />;
 * }
 *
 * if (result) {
 *   return <SkillResult result={result} />;
 * }
 * ```
 */
export function useSkill(): UseSkillReturn {
  const [status, setStatus] = useState<SkillStatus>('idle');
  const [result, setResult] = useState<SkillInvokeResponse | null>(null);
  const [error, setError] = useState<Error | null>(null);
  const [isAuthenticated, setIsAuthenticated] = useState(true);

  const invoke = useCallback(async (
    skillId: string,
    input: SkillInput
  ): Promise<SkillInvokeResponse | null> => {
    setStatus('loading');
    setError(null);
    setResult(null);

    try {
      const response = await invokeSkill(skillId, input);
      setResult(response);
      setStatus('success');
      setIsAuthenticated(true);
      return response;
    } catch (err) {
      setStatus('error');

      if (err instanceof AuthenticationError) {
        setIsAuthenticated(false);
        setError(err);
      } else if (err instanceof SkillNotFoundError) {
        setError(err);
      } else if (err instanceof RateLimitError) {
        setError(new Error(`Too many requests. Please wait ${err.retryAfter || 60} seconds.`));
      } else if (err instanceof Error) {
        setError(err);
      } else {
        setError(new Error('An unexpected error occurred'));
      }

      return null;
    }
  }, []);

  const reset = useCallback(() => {
    setStatus('idle');
    setResult(null);
    setError(null);
  }, []);

  return {
    status,
    result,
    error,
    isLoading: status === 'loading',
    isAuthenticated,
    invoke,
    reset,
  };
}

export default useSkill;
