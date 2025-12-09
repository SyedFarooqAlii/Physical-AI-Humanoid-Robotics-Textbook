// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros/index',
        'module-1-ros/chapter-1-1',
        'module-1-ros/chapter-1-2',
        'module-1-ros/chapter-1-3'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-digital-twin/index',
        'module-2-digital-twin/chapter-2-1',
        'module-2-digital-twin/chapter-2-2',
        'module-2-digital-twin/chapter-2-3'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3-ai-brain/index',
        'module-3-ai-brain/chapter-3-1',
        'module-3-ai-brain/chapter-3-2',
        'module-3-ai-brain/chapter-3-3'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vla/index',
        'module-4-vla/chapter-4-1',
        'module-4-vla/chapter-4-2',
        'module-4-vla/chapter-4-3'
      ],
    },
    {
      type: 'category',
      label: 'Capstone: The Autonomous Humanoid',
      items: [
        'capstone/autonomous-humanoid',
      ],
    },
  ],
};

module.exports = sidebars;