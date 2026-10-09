import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const isVercel = process.env.VERCEL === '1';
const isGitHubActions = process.env.GITHUB_ACTIONS === 'true';

// Determine site URL:
// 1. Explicit SITE_URL environment variable (e.g. custom domain)
// 2. Vercel production/preview URL
// 3. GitHub Pages URL fallback
const siteUrl = process.env.SITE_URL
  ? process.env.SITE_URL
  : isVercel
    ? process.env.VERCEL_PROJECT_PRODUCTION_URL
      ? `https://${process.env.VERCEL_PROJECT_PRODUCTION_URL}`
      : process.env.VERCEL_URL
        ? `https://${process.env.VERCEL_URL}`
        : 'https://humanoid-book.vercel.app'
    : 'https://rooh-u-din.github.io';

// Base URL:
// 1. Explicit BASE_URL environment variable
// 2. GitHub Actions (GitHub Pages subpath) -> '/humanoid-book/'
// 3. Vercel / local development -> '/'
const baseUrl = process.env.BASE_URL || (isGitHubActions ? '/humanoid-book/' : '/');

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Learn AI-Native Software Engineering for Humanoid Robots',
  favicon: 'img/logo.png',

  // Set the production url of your site here
  url: siteUrl,
  // Set the /<baseUrl>/ pathname under which your site is served
  baseUrl: baseUrl,

  customFields: {
    backendUrl: process.env.DOCUSAURUS_BACKEND_URL || 'https://backend-book-vtha.onrender.com',
  },

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Rooh-U-Din', // Usually your GitHub org/user name.
  projectName: 'humanoid-book', // Usually your repo name.
  trailingSlash: false,
  deploymentBranch: 'gh-pages',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  markdown: {
    mermaid: true,
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Rooh-U-Din/humanoid-book/tree/main/',
        },
        blog: false, // Disable blog for this project
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themes: ['@docusaurus/theme-mermaid'],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/logo.png',
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo.png',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Course',
        },
        {
          href: 'https://github.com/Rooh-U-Din/humanoid-book',
          label: 'GitHub',
          position: 'right',
        },
        {
          type: 'custom-userButton',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Documentation',
          items: [
            {
              label: 'Getting Started',
              to: '/docs/intro',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/Rooh-U-Din/humanoid-book',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book Project. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml', 'cpp'],
    },
    mermaid: {
      theme: {light: 'neutral', dark: 'dark'},
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
