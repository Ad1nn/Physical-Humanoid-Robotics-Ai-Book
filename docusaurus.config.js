// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Digital Brain to Physical Body',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://Ad1nn.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/Physical-Humanoid-Robotics-Ai-Book/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'Ad1nn', // Usually your GitHub org/user name.
  projectName: 'Physical-Humanoid-Robotics-Ai-Book', // Usually your repo name.
  staticDirectories: ['static_new'],

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          path: 'docs', // This is important to point to the correct folder
          routeBasePath: '/', // Serve docs from the root
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Ad1nn/Physical-Humanoid-Robotics-Ai-Book/tree/main/', // Point to the actual repo
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  plugins: [
    [
      '@docusaurus/plugin-google-analytics',
      {
        trackingID: 'UA-XXXXX-Y', // Replace with your actual Google Analytics tracking ID
        anonymizeIP: true, // Should IPs be anonymized?
      },
    ],
    function (context, options) {
      return {
        name: 'webpack-configuration',
        configureWebpack(config, isServer, utils) {
          return {
            resolve: {
              alias: {
                '@site': context.siteDir,
                '@components': `${context.siteDir}/src/components`,
                '@theme': `${context.siteDir}/src/theme`,
              },
            },
          };
        },
      };
    },
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Physical AI & Humanoid Robotics Logo',
          src: 'img/book-logo.png',
          href: '/', // Link logo to the homepage
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'bookSidebar',
            position: 'left',
            label: 'Documentation', // Changed from "Module 1: ROS 2"
            to: '/', // Link to the root of the documentation
          },
          {
            href: 'https://github.com/Ad1nn/Physical-Humanoid-Robotics-Ai-Book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      colorMode: {
        defaultMode: 'dark',
        disableSwitch: false,
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Module 1: ROS 2',
                to: '/module-01-ros2', // Path relative to routeBasePath
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/docusaurus',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/docusaurus',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/Ad1nn/Physical-Humanoid-Robotics-Ai-Book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Ad1nn, Inc. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        defaultShowLineNumbers: true, // Enable line numbers by default
      },
    }),
};

export default config;