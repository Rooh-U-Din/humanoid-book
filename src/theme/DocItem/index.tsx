/**
 * Custom DocItem Wrapper
 *
 * Automatically injects:
 * - Authentication gating for all documentation content (US4 - Content Blocking)
 * - Urdu translation toggle into ALL documentation pages
 * - Email verification prompt for unverified users
 * - Complete profile prompt for users without completed profiles
 */

import React from 'react';
import DocItem from '@theme-original/DocItem';
import type DocItemType from '@theme/DocItem';
import type { WrapperProps } from '@docusaurus/types';
import GlobalTranslationToggle from '@site/src/components/GlobalTranslation/GlobalTranslationToggle';
import { VerifyEmailPrompt, CompleteProfilePrompt, ProtectedContent } from '@site/src/components/Auth';

type Props = WrapperProps<typeof DocItemType>;

export default function DocItemWrapper(props: Props): JSX.Element {
  return (
    <ProtectedContent
      loginTitle="Sign in to access this chapter"
      loginDescription="Create a free account or sign in to unlock all chapters, modules, and interactive features of the Physical AI Book."
    >
      <GlobalTranslationToggle />
      <VerifyEmailPrompt variant="banner" />
      <CompleteProfilePrompt variant="banner" />
      <DocItem {...props} />
    </ProtectedContent>
  );
}
