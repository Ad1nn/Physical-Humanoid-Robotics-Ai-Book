/**
 * Creating a sidebar enables you to:
 * - Create an ordered group of docs
 * - Render a sidebar in the docs side navigation
 * - Learn more: https://docusaurus.io/docs/sidebar
 */

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  bookSidebar: [
    'intro',
    'prerequisites',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      link: {
        type: 'doc',
        id: 'module-01-ros2/index',
      },
      items: [
        'module-01-ros2/chapter1.1',
        'module-01-ros2/chapter1.2',
        'module-01-ros2/chapter1.3',
        'module-01-ros2/project',
        'module-01-ros2/assessment',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      link: {
        type: 'doc',
        id: 'module-02-simulation/index',
      },
      items: [
        'module-02-simulation/chapter-01-gazebo',
        'module-02-simulation/chapter-02-unity',
        'module-02-simulation/chapter-03-sensors',
        'module-02-simulation/project',
        'module-02-simulation/assessment',
      ],
    }, // Add this comma to fix the parsing error
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      link: {
        type: 'doc',
        id: 'module-03-isaac/index',
      },
      items: [
        'module-03-isaac/chapter-01-isaac-sim',
        'module-03-isaac/chapter-02-isaac-ros-vslam',
        'module-03-isaac/chapter-03-nav2-planning',
        'module-03-isaac/project',
        'module-03-isaac/assessment',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      link: {
        type: 'doc',
        id: 'module-04-vla/index', // Placeholder index.md for Module 4
      },
      items: [
        'module-04-vla/chapter-01-whisper',
        'module-04-vla/chapter-02-llm-planning',
        'module-04-vla/chapter-03-humanoid-integration',
        'module-04-vla/assessment',
      ],
    },
    'final-capstone/index',
  ],
};

export default sidebars;
