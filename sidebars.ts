import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Course Overview',
      collapsed: false,
      items: ['intro'],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      collapsed: true,
      items: [
        'modules/ros2/fundamentals',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo + Unity)',
      collapsed: true,
      items: [
        'modules/gazebo/intro',
        'modules/unity/intro',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      collapsed: true,
      items: [
        'modules/isaac/intro',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      collapsed: true,
      items: [
        'modules/vla/intro',
        'modules/conversational/intro',
      ],
    },
    {
      type: 'category',
      label: 'Capstone: Autonomous Humanoid Robot',
      collapsed: true,
      items: [
        'modules/capstone/humanoid-voice-control',
      ],
    },
  ],
};

export default sidebars;
