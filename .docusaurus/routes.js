import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/ai-robotics-textbook/docs',
    component: ComponentCreator('/ai-robotics-textbook/docs', 'd74'),
    routes: [
      {
        path: '/ai-robotics-textbook/docs',
        component: ComponentCreator('/ai-robotics-textbook/docs', 'fdd'),
        routes: [
          {
            path: '/ai-robotics-textbook/docs',
            component: ComponentCreator('/ai-robotics-textbook/docs', 'af6'),
            routes: [
              {
                path: '/ai-robotics-textbook/docs/accessibility',
                component: ComponentCreator('/ai-robotics-textbook/docs/accessibility', '1bd'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/assets/diagrams/isaac-sim-architecture',
                component: ComponentCreator('/ai-robotics-textbook/docs/assets/diagrams/isaac-sim-architecture', '756'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/assets/diagrams/ros2_architecture',
                component: ComponentCreator('/ai-robotics-textbook/docs/assets/diagrams/ros2_architecture', 'fc3'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/assets/diagrams/ros2-launch-system',
                component: ComponentCreator('/ai-robotics-textbook/docs/assets/diagrams/ros2-launch-system', '9e2'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/assets/diagrams/ros2-node-communication',
                component: ComponentCreator('/ai-robotics-textbook/docs/assets/diagrams/ros2-node-communication', '741'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/assets/diagrams/vla-system-architecture',
                component: ComponentCreator('/ai-robotics-textbook/docs/assets/diagrams/vla-system-architecture', 'eea'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/assets/diagrams/vslam-pipeline',
                component: ComponentCreator('/ai-robotics-textbook/docs/assets/diagrams/vslam-pipeline', 'e44'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/assets/references/isaac-ros-references',
                component: ComponentCreator('/ai-robotics-textbook/docs/assets/references/isaac-ros-references', 'bcf'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/assets/references/ros2-academic-references',
                component: ComponentCreator('/ai-robotics-textbook/docs/assets/references/ros2-academic-references', '97b'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/assets/references/ros2-documentation-references',
                component: ComponentCreator('/ai-robotics-textbook/docs/assets/references/ros2-documentation-references', '79f'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/assets/references/vla-references',
                component: ComponentCreator('/ai-robotics-textbook/docs/assets/references/vla-references', 'cd8'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/capstone-project',
                component: ComponentCreator('/ai-robotics-textbook/docs/capstone-project', '1d6'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/citation-standards',
                component: ComponentCreator('/ai-robotics-textbook/docs/citation-standards', '89a'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/code-examples/ai',
                component: ComponentCreator('/ai-robotics-textbook/docs/code-examples/ai', 'c3f'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/code-examples/ros2',
                component: ComponentCreator('/ai-robotics-textbook/docs/code-examples/ros2', '786'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/code-examples/simulation',
                component: ComponentCreator('/ai-robotics-textbook/docs/code-examples/simulation', 'ce9'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/code-examples/vla',
                component: ComponentCreator('/ai-robotics-textbook/docs/code-examples/vla', '588'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/ethical-considerations',
                component: ComponentCreator('/ai-robotics-textbook/docs/ethical-considerations', '8ed'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/glossary',
                component: ComponentCreator('/ai-robotics-textbook/docs/glossary', '58e'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/intro',
                component: ComponentCreator('/ai-robotics-textbook/docs/intro', '5aa'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-1-robotic-nervous-system/',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-1-robotic-nervous-system/', '2ac'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-1-robotic-nervous-system/chapter-1',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-1-robotic-nervous-system/chapter-1', '4c3'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-1-robotic-nervous-system/chapter-2',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-1-robotic-nervous-system/chapter-2', '853'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-1-robotic-nervous-system/chapter-3',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-1-robotic-nervous-system/chapter-3', '49f'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-1-robotic-nervous-system/chapter-4',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-1-robotic-nervous-system/chapter-4', 'e6c'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-1-robotic-nervous-system/chapter-5',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-1-robotic-nervous-system/chapter-5', '4a6'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-1-robotic-nervous-system/exercises',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-1-robotic-nervous-system/exercises', 'e2f'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-1-robotic-nervous-system/learning-objectives',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-1-robotic-nervous-system/learning-objectives', 'cc0'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-1-robotic-nervous-system/mini-project',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-1-robotic-nervous-system/mini-project', '200'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-1-robotic-nervous-system/ros2-practical-exercises',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-1-robotic-nervous-system/ros2-practical-exercises', 'd0d'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-2-digital-twin/',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-2-digital-twin/', '1d3'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-2-digital-twin/chapter-3',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-2-digital-twin/chapter-3', 'b57'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-2-digital-twin/chapter-4',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-2-digital-twin/chapter-4', '125'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-2-digital-twin/exercises',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-2-digital-twin/exercises', '354'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-2-digital-twin/learning-objectives',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-2-digital-twin/learning-objectives', '681'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-2-digital-twin/mini-project',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-2-digital-twin/mini-project', '7a4'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-3-ai-robot-brain/',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-3-ai-robot-brain/', 'a16'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-3-ai-robot-brain/assets/performance_evaluation_metrics',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-3-ai-robot-brain/assets/performance_evaluation_metrics', 'c88'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-3-ai-robot-brain/chapter-5',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-3-ai-robot-brain/chapter-5', '5e3'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-3-ai-robot-brain/chapter-6',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-3-ai-robot-brain/chapter-6', 'd6e'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-3-ai-robot-brain/exercises',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-3-ai-robot-brain/exercises', 'c1a'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-3-ai-robot-brain/learning-objectives',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-3-ai-robot-brain/learning-objectives', 'c50'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-3-ai-robot-brain/mini-project',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-3-ai-robot-brain/mini-project', '0c1'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-4-vla/',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-4-vla/', 'ff5'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-4-vla/capstone-project',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-4-vla/capstone-project', '04c'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-4-vla/chapter-7',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-4-vla/chapter-7', '88b'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-4-vla/chapter-8',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-4-vla/chapter-8', '19b'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-4-vla/exercises',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-4-vla/exercises', '66d'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/module-4-vla/learning-objectives',
                component: ComponentCreator('/ai-robotics-textbook/docs/module-4-vla/learning-objectives', 'b05'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/modules',
                component: ComponentCreator('/ai-robotics-textbook/docs/modules', '177'),
                exact: true
              },
              {
                path: '/ai-robotics-textbook/docs/references',
                component: ComponentCreator('/ai-robotics-textbook/docs/references', 'd90'),
                exact: true,
                sidebar: "textbookSidebar"
              },
              {
                path: '/ai-robotics-textbook/docs/review-workflow',
                component: ComponentCreator('/ai-robotics-textbook/docs/review-workflow', 'fc3'),
                exact: true
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/ai-robotics-textbook/',
    component: ComponentCreator('/ai-robotics-textbook/', 'b95'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
