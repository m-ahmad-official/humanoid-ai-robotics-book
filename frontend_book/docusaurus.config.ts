import { themes as prismThemes } from "prism-react-renderer";
import type { Config } from "@docusaurus/types";
import type * as Preset from "@docusaurus/preset-classic";
const remarkMath = require("remark-math");
const rehypeKatex = require("rehype-katex");

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: "Humanoid Robotics with ROS 2",
  tagline: "Educational Guide for AI Students",
  favicon: "img/favicon.ico",

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: "https://humanoid-ai-robotics-book-1.vercel.app/",
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: "/",

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: "frontend_book", // Usually your GitHub org/user name.
  projectName: "humanoid-ai-robotics-book-1", // Usually your repo name.

  onBrokenLinks: "throw",

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: "en",
    locales: ["en"],
  },

  presets: [
    [
      "classic",
      {
        docs: {
          sidebarPath: "./sidebars.ts",
          remarkPlugins: [remarkMath],
          rehypePlugins: [rehypeKatex],
        },
        // blog: {
        //   showReadingTime: true,
        //   feedOptions: {
        //     type: ["rss", "atom"],
        //     xslt: true,
        //   },
        //   // Please change this to your repo.
        //   // Remove this to remove the "edit this page" links.
        //   editUrl:
        //     "https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/",
        //   // Useful options to enforce blogging best practices
        //   onInlineTags: "warn",
        //   onInlineAuthors: "warn",
        //   onUntruncatedBlogPosts: "warn",
        // },
        theme: {
          customCss: "./src/css/custom.css",
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: "img/docusaurus-social-card.jpg",
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: "Humanoid Robotics",
      logo: {
        alt: "Humanoid Robotics Logo",
        src: "img/logo.svg",
      },
      items: [
        {
          type: "docSidebar",
          sidebarId: "tutorialSidebar",
          position: "left",
          label: "Tutorial",
        },
        {
          type: "docSidebar",
          sidebarId: "module1Sidebar",
          position: "left",
          label: "Module 1: ROS 2",
        },
        {
          type: "docSidebar",
          sidebarId: "module2Sidebar",
          position: "left",
          label: "Module 2: Digital Twins",
        },
        {
          type: "docSidebar",
          sidebarId: "module3Sidebar",
          position: "left",
          label: "Module 3: AI-Robot Brain",
        },
        {
          type: "docSidebar",
          sidebarId: "module4Sidebar",
          position: "left",
          label: "Module 4: VLA Systems",
        },
        // { to: "/blog", label: "Blog", position: "left" },
        {
          href: "https://github.com/m-ahmad-official/humanoid-ai-robotics-book.git",
          label: "GitHub",
          position: "right",
        },
      ],
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Modules",
          items: [
            {
              label: "Module 1: ROS 2",
              to: "/docs/module-1/intro-to-ros2",
            },
            {
              label: "Module 2: Digital Twins",
              to: "/docs/module-2/intro-to-digital-twins",
            },
            {
              label: "Module 3: AI-Robot Brain",
              to: "/docs/module-3/intro-to-ai-robot-brain",
            },
            {
              label: "Module 4: VLA Systems",
              to: "/docs/module-4/intro-to-vla",
            },
          ],
        },
        {
          title: "Community",
          items: [
            {
              label: "Stack Overflow",
              href: "https://stackoverflow.com/questions/tagged/docusaurus",
            },
            {
              label: "Discord",
              href: "https://discordapp.com/invite/docusaurus",
            },
            {
              label: "X",
              href: "https://x.com/docusaurus",
            },
          ],
        },
        {
          title: "More",
          items: [
            // {
            //   label: "Blog",
            //   to: "/blog",
            // },
            {
              label: "GitHub",
              href: "https://github.com/m-ahmad-official/humanoid-ai-robotics-book.git",
            },
            {
              label: "LinkedIn",
              href: "https://www.linkedin.com/in/muhammad-ahmed-a06357347/",
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Humanoid Robotics Guide. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
