/**
 * Custom DocItem Wrapper
 *
 * Automatically injects the Urdu translation toggle into ALL documentation pages.
 * This wraps Docusaurus's default DocItem component.
 */

import React from 'react';
import DocItem from '@theme-original/DocItem';
import type DocItemType from '@theme/DocItem';
import type { WrapperProps } from '@docusaurus/types';
import GlobalTranslationToggle from '@site/src/components/GlobalTranslation/GlobalTranslationToggle';

type Props = WrapperProps<typeof DocItemType>;

export default function DocItemWrapper(props: Props): JSX.Element {
  return (
    <>
      <GlobalTranslationToggle />
      <DocItem {...props} />
    </>
  );
}
