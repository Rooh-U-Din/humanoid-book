/**
 * Skills Component Exports
 *
 * Provides AI-powered text analysis skills including code explanation,
 * translation, debugging assistance, and content navigation.
 */

// Components
export { SkillPanel, default as SkillPanelDefault } from './SkillPanel';
export { SkillResult, default as SkillResultDefault } from './SkillResult';
export { TextSelectionProvider, default as TextSelectionProviderDefault } from './TextSelectionProvider';

// API
export {
  invokeSkill,
  listSkills,
  getHistory,
  getInvocationDetail,
  AuthenticationError,
  SkillNotFoundError,
  RateLimitError,
} from './api';

// Types
export type {
  SkillInput,
  SkillInvokeRequest,
  SkillInvokeResponse,
  SkillResult as SkillResultType,
  SkillInfo,
  SkillListResponse,
  SkillInvocationSummary,
  SkillHistoryResponse,
  SkillInvocationDetail,
  Citation,
  Suggestion,
} from './api';
