/**
 * Custom DocItem Wrapper
 *
 * Automatically injects:
 * - Authentication gating for all documentation content (US4 - Content Blocking)
 * - Urdu translation toggle into ALL documentation pages
 * - Email verification prompt for unverified users
 * - Complete profile prompt for users without completed profiles
 * - AI Skills panel on text selection (US1 - Code Explanation)
 */

import React from 'react';
import DocItem from '@theme-original/DocItem';
import type DocItemType from '@theme/DocItem';
import type { WrapperProps } from '@docusaurus/types';
import { useLocation } from '@docusaurus/router';
import GlobalTranslationToggle from '@site/src/components/GlobalTranslation/GlobalTranslationToggle';
import { VerifyEmailPrompt, CompleteProfilePrompt, ProtectedContent } from '@site/src/components/Auth';
import { TextSelectionProvider } from '@site/src/components/Skills';

type Props = WrapperProps<typeof DocItemType>;

/**
 * Extract chapter ID from the current URL path.
 * e.g., /docs/ros2-basics/intro -> ros2-basics
 */
function useChapterId(): string | undefined {
  const location = useLocation();
  const pathParts = location.pathname.split('/').filter(Boolean);
  // Typically: ['docs', 'chapter-name', 'page-name']
  if (pathParts.length >= 2 && pathParts[0] === 'docs') {
    return pathParts[1];
  }
  return undefined;
}

export default function DocItemWrapper(props: Props): JSX.Element {
  const chapterId = useChapterId();

  return (
    <ProtectedContent
      loginTitle="Sign in to access this chapter"
      loginDescription="Create a free account or sign in to unlock all chapters, modules, and interactive features of the Physical AI Book."
    >
      <GlobalTranslationToggle />
      <VerifyEmailPrompt variant="banner" />
      <CompleteProfilePrompt variant="banner" />
      <TextSelectionProvider chapterId={chapterId}>
        <DocItem {...props} />
      </TextSelectionProvider>
    </ProtectedContent>
  );
}
