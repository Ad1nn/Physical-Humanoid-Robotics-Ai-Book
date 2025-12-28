import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { FaRobot, FaMicrochip, FaBookOpen, FaProjectDiagram } from 'react-icons/fa';
import styles from './index.module.css';

const modules = [
  {
    title: 'Module 1: The Robotic Nervous System',
    icon: <FaRobot className={styles.moduleIcon} />,
    description: 'An introduction to ROS 2, the communication backbone of modern robotics. Learn about nodes, topics, services, and building a basic robotic system.',
    link: '/module-01-ros2',
  },
  {
    title: 'Module 2: The Digital Twin',
    icon: <FaMicrochip className={styles.moduleIcon} />,
    description: 'Explore the world of simulation with Gazebo and Isaac Sim. Understand how to create digital replicas of robots and environments for testing and development.',
    link: '/module-02-simulation',
  },
  {
    title: 'Module 3: The AI Brain',
    icon: <FaBookOpen className={styles.moduleIcon} />,
    description: 'Dive into the core of robotic intelligence. This module covers perception, SLAM, and navigation using the Nav2 stack in Isaac Sim.',
    link: '/module-03-isaac',
  },
  {
    title: 'Module 4: Vision-Language-Action',
    icon: <FaProjectDiagram className={styles.moduleIcon} />,
    description: 'Discover how to connect large language models to robotic actions. Implement systems that can understand and respond to natural language commands.',
    link: '/module-04-vla',
  },
];

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className={styles.heroGrid}></div>
      <div className="container">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1 className={styles.hero__title}>{siteConfig.title}</h1>
            <p className={styles.hero__subtitle}>{siteConfig.tagline}</p>
            <p className={styles.heroDescription}>
              A technical book on AI systems operating in the physical world —
              covering ROS 2, simulation, perception, planning, and
              vision-language-action systems for humanoid robots.
            </p>
            <div className={styles.heroImage}>
                          <img src="/img/humanoid-robot.svg" alt="Humanoid Robot" />
            </div>
            <div className={styles.buttons}>
              <Link
                className="button button--primary button--lg"
                to="/intro"
              >
                Start Reading
              </Link>
              <Link
                className="button button--secondary button--lg"
                to="/module-01-ros2"
              >
                Browse Modules
              </Link>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function ModulesSection() {
  return (
    <section className={styles.modules}>
      <div className="container">
        <h2>Course Modules</h2>
        <div className="row">
          {modules.map((module, idx) => (
            <div className="col col--6" key={idx}>
              <div className={styles.moduleCard}>
                <div className="card">
                  <div className="card__header">
                    <h3>
                      {module.icon}
                      {module.title}
                    </h3>
                  </div>
                  <div className="card__body">
                    <p>{module.description}</p>
                  </div>
                  <div className="card__footer">
                    <Link
                      className="button button--secondary button--block"
                      to={module.link}
                    >
                      Explore Module
                    </Link>
                  </div>
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CourseInfoSection() {
  return (
    <div className={styles.courseInfo}>
      <div className="container">
        <div className="row">
          <div className="col col--6">
            <h3>Prerequisites</h3>
            <p>
              This book is designed for intermediate-level developers with a solid understanding of Python and basic familiarity with linear algebra and calculus.
            </p>
          </div>
          <div className="col col--6">
            <h3>Hands-On Projects</h3>
            <p>
              Each module includes practical, hands-on projects that allow you to apply the concepts you've learned in a real-world context.
            </p>
          </div>
        </div>
      </div>
    </div>
  );
}

function TechStackSection() {
  return (
    <div className={styles.techStack}>
      <div className="container">
        <h2>Technology Stack</h2>
        <div className="row">
          <div className="col">
            <div className={styles.techBadge}>ROS 2</div>
          </div>
          <div className="col">
            <div className={styles.techBadge}>Gazebo</div>
          </div>
          <div className="col">
            <div className={styles.techBadge}>Isaac Sim</div>
          </div>
          <div className="col">
            <div className={styles.techBadge}>PyTorch</div>
          </div>
          <div className="col">
            <div className={styles.techBadge}>Docker</div>
          </div>
        </div>
      </div>
    </div>
  );
}

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="A technical book on AI systems operating in the physical world."
    >
      <HomepageHeader />
      <main>
        <ModulesSection />
        <CourseInfoSection />
        <TechStackSection />
      </main>
    </Layout>
  );
}
