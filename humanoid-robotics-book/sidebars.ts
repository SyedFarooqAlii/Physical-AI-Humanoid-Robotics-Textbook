import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manual sidebar for the Physical AI & Humanoid Robotics book
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module-1-ros/intro',
        'module-1-ros/chapters/communication-architecture',
        'module-1-ros/chapters/distributed-communication-patterns',
        'module-1-ros/chapters/real-time-performance-optimization',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin Systems',
      items: [
        'module-2-digital-twin/intro',
        'module-2-digital-twin/chapters/simulation-environment-architecture',
        'module-2-digital-twin/chapters/simulation-fidelity-optimization',
        'module-2-digital-twin/chapters/sim-to-real-transfer-methodologies',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain',
      items: [
        'module-3-ai-brain/intro',
        'module-3-ai-brain/chapters/perception-planning-control-architecture',
        'module-3-ai-brain/chapters/cognitive-architecture-patterns',
        'module-3-ai-brain/chapters/adaptive-learning-algorithms',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: VLA Integration',
      items: [
        'module-4-vla/intro',
        'module-4-vla/chapters/multimodal-integration-architecture',
        'module-4-vla/chapters/multimodal-fusion-techniques',
        'module-4-vla/chapters/context-aware-interaction-models',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Capstone Integration',
      items: [
        'capstone/intro',
        'capstone/capstone-integration-spec',
        'capstone/capstone-demonstration-scenarios',
        'capstone/capstone-validation-testing',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Technical Resources',
      items: [
        'contracts/ros2-interfaces',
        'data-models/foundational-data-models',
        'simulation/simulation-environment-templates',
        'hardware/hardware-configuration-guidelines',
        'validation/validation-testing-framework',
        'citation/citation-reference-management',
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Quickstart & Setup',
      items: [
        'quickstart/quickstart',
        'quickstart/project-implementation-summary',
        'quickstart/final-quality-assurance-review',
        'quickstart/quickstart-validation-report',
      ],
      collapsed: true,
    },
  ],
};

export default sidebars;
