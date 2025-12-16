import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';
import styles from './index.module.css';

// SVG Icons as inline components
const RobotIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor" className="w-16 h-16">
    <path d="M12 2a3 3 0 0 0-3 3v1H7a3 3 0 0 0-3 3v10a3 3 0 0 0 3 3h10a3 3 0 0 0 3-3V9a3 3 0 0 0-3-3h-2V5a3 3 0 0 0-3-3zm0 2c.55 0 1 .45 1 1s-.45 1-1 1-1-.45-1-1 .45-1 1-1zm-2 8a2 2 0 1 1 4 0 2 2 0 0 1-4 0zm-3 5a1 1 0 0 1 1-1h10a1 1 0 0 1 1 1v2a1 1 0 0 1-1 1H8a1 1 0 0 1-1-1v-2z" />
  </svg>
);

const UnityIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor" className="w-16 h-16">
    <path d="M12 2L2 7l10 5 10-5-10-5zM2 17l10 5 10-5M2 12l10 5 10-5" />
  </svg>
);

const BrainIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor" className="w-16 h-16">
    <path d="M12 4.5a7.5 7.5 0 0 0-7.5 7.5c0 2.25 1.05 4.2 2.7 5.4V21h9v-3.6c1.65-1.2 2.7-3.15 2.7-5.4a7.5 7.5 0 0 0-7.5-7.5zm0 2a5.5 5.5 0 0 1 5.5 5.5c0 1.85-.85 3.5-2.2 4.5H8.7A5.48 5.48 0 0 1 6.5 12c0-3.05 2.45-5.5 5.5-5.5z" />
  </svg>
);

const VisionIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor" className="w-16 h-16">
    <path d="M12 4.5a7.5 7.5 0 0 1 7.5 7.5c0 2.25-1.05 4.2-2.7 5.4V21h-9v-3.6c-1.65-1.2-2.7-3.15-2.7-5.4a7.5 7.5 0 0 1 7.5-7.5zM12 15a3 3 0 1 0 0-6 3 3 0 0 0 0 6z" />
  </svg>
);

const CodeIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor" className="w-16 h-16">
    <path d="M8.25 4.5a3.75 3.75 0 1 1 7.5 0v8.25a3.75 3.75 0 1 1-7.5 0V4.5z" />
    <path d="M6 10.5a.75.75 0 0 1 .75.75v1.5a5.25 5.25 0 1 0 10.5 0v-1.5a.75.75 0 0 1 1.5 0v1.5a6.751 6.751 0 0 1-6 6.709v2.291h3a.75.75 0 0 1 0 1.5h-7.5a.75.75 0 0 1 0-1.5h3v-2.291a6.751 6.751 0 0 1-6-6.709v-1.5A.75.75 0 0 1 6 10.5z" />
  </svg>
);

const ProjectIcon = () => (
  <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor" className="w-16 h-16">
    <path d="M11.47 3.84a.75.75 0 0 1 1.06 0l8.69 8.69a.75.75 0 1 0 1.06-1.06l-8.689-8.69a2.25 2.25 0 0 0-3.182 0l-8.69 8.69a.75.75 0 0 0 1.061 1.06l8.69-8.69z" />
    <path d="m12 5.432 8.159 8.159c.03.03.06.058.091.086v6.284c0 1.657-1.296 2.998-2.986 2.998H4.928c-1.69 0-2.986-1.342-2.986-2.999V13.59l8.16-8.158z" />
  </svg>
);

// Hero Section Component
function HeroSection() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className="text--center">
          <Heading as="h1" className={clsx('hero__title', styles.mainTitle)}>
            {siteConfig.title}
          </Heading>
          <p className={clsx('hero__subtitle', styles.mainSubtitle)}>
            {siteConfig.tagline}
          </p>
          <div className={styles.buttons}>
            <Link className="button button--secondary button--lg" to="/docs/intro">
              Get Started
            </Link>
            <Link className="button button--outline button--lg" to="/docs/modules">
              Browse Modules
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

// Modules Overview Section Component
function ModulesOverview() {
  const modules = [
    {
      title: 'Module 1: Robotic Nervous System (ROS2)',
      icon: <RobotIcon />,
      description: 'Learn about ROS2 fundamentals, nodes, topics, services, and building distributed robotic systems.',
      to: '/docs/module-1-ros2'
    },
    {
      title: 'Module 2: Digital Twin & Simulation (Unity/Gazebo)',
      icon: <UnityIcon />,
      description: 'Create realistic simulations and digital twins using Unity and Gazebo for robot testing and development.',
      to: '/docs/module-2-digital-twin'
    },
    {
      title: 'Module 3: AI Robot Brain (NVIDIA Isaac)',
      icon: <BrainIcon />,
      description: 'Develop intelligent robot behaviors using NVIDIA Isaac SDK and advanced AI algorithms.',
      to: '/docs/module-3-ai-robot-brain'
    },
    {
      title: 'Module 4: Vision-Language-Action (VLA)',
      icon: <VisionIcon />,
      description: 'Integrate computer vision, natural language processing, and robotic action for advanced human-robot interaction.',
      to: '/docs/module-4-vla'
    }
  ];

  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <div className="text--center padding-bottom--lg">
          <Heading as="h2" className={styles.sectionTitle}>
            Learning Modules
          </Heading>
          <p className={styles.sectionSubtitle}>
            Four comprehensive modules covering essential aspects of humanoid robotics
          </p>
        </div>
        <div className="row">
          {modules.map((module, index) => (
            <div key={index} className="col col--3 margin-bottom--lg">
              <div className={clsx('card', styles.moduleCard)}>
                <div className="card__header text--center">
                  <div className={styles.iconContainer}>
                    {module.icon}
                  </div>
                  <Heading as="h3">{module.title}</Heading>
                </div>
                <div className="card__body">
                  <p>{module.description}</p>
                </div>
                <div className="card__footer text--center">
                  <Link className="button button--primary button--block" to={module.to}>
                    Learn More
                  </Link>
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// Capstone Project Section Component
function CapstoneProject() {
  return (
    <section className={styles.capstoneSection}>
      <div className="container">
        <div className="row">
          <div className="col col--6">
            <div className={styles.capstoneContent}>
              <Heading as="h2" className={styles.sectionTitle}>
                Capstone Project
              </Heading>
              <p className={styles.sectionSubtitle}>
                Humanoid Robotics Integration Challenge
              </p>
              <p>
                Apply everything you've learned by building an integrated humanoid robot system that combines all four modules.
                You'll develop a complete robot that can perceive its environment, understand natural language commands,
                plan actions, and execute complex manipulation tasks.
              </p>
              <ul className={styles.capstoneFeatures}>
                <li>Full-body humanoid robot control</li>
                <li>Vision-language-action integration</li>
                <li>Real-world task execution</li>
                <li>Advanced AI decision making</li>
              </ul>
              <Link className="button button--primary button--lg" to="/docs/capstone-project">
                Start Capstone Project
              </Link>
            </div>
          </div>
          <div className="col col--6 text--center">
            <div className={styles.capstoneImage}>
              <ProjectIcon />
              <p className={styles.imageCaption}>Humanoid Robot Integration</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

// Code Examples Section Component
function CodeExamples() {
  const examples = [
    {
      title: 'ROS2 Examples',
      description: 'Sample ROS2 packages and nodes',
      to: '/code-examples/ros2'
    },
    {
      title: 'Simulation Scripts',
      description: 'Unity and Gazebo simulation examples',
      to: '/code-examples/simulation'
    },
    {
      title: 'AI Models',
      description: 'NVIDIA Isaac AI implementations',
      to: '/code-examples/ai'
    },
    {
      title: 'VLA Integrations',
      description: 'Vision-Language-Action examples',
      to: '/code-examples/vla'
    }
  ];

  return (
    <section className={styles.codeExamplesSection}>
      <div className="container">
        <div className="text--center padding-bottom--lg">
          <Heading as="h2" className={styles.sectionTitle}>
            Code Examples
          </Heading>
          <p className={styles.sectionSubtitle}>
            Practical code samples for each module
          </p>
        </div>
        <div className="row">
          {examples.map((example, index) => (
            <div key={index} className="col col--3 margin-bottom--lg">
              <div className={clsx('card', styles.exampleCard)}>
                <div className="card__header text--center">
                  <CodeIcon />
                  <Heading as="h3">{example.title}</Heading>
                </div>
                <div className="card__body">
                  <p>{example.description}</p>
                </div>
                <div className="card__footer text--center">
                  <Link className="button button--secondary button--sm" to={example.to}>
                    View Examples
                  </Link>
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// Footer Section Component (using Docusaurus theme footer)
function CustomFooter() {
  return null; // Using theme footer instead
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Textbook - Comprehensive guide to building intelligent humanoid robots">
      <HeroSection />
      <main>
        <ModulesOverview />
        <CapstoneProject />
        <CodeExamples />
      </main>
    </Layout>
  );
}
