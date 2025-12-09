import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// Minimal sidebar that only includes documents Docusaurus can actually find
const sidebars: SidebarsConfig = {
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
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin Systems',
      items: [
        'module-2-digital-twin/intro',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain',
      items: [
        'module-3-ai-brain/intro',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: VLA Integration',
      items: [
        'module-4-vla/intro',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Capstone Integration',
      items: [
        'capstone/intro',
        'contracts/ros2-interfaces',
      ],
      collapsed: false,
    },
  ],
};

export default sidebars;