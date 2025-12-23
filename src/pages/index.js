import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <p className={styles.heroDescription}>
          Learn to build autonomous humanoid robots using ROS 2, simulation, and AI.
        </p>
        <div className={clsx('hero__img', styles.heroImage)}>
            <img src="/img/humanoid-robot.png" alt="Humanoid Robot" />
        </div>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="#modules">
            Get Started
          </Link>
        </div>
      </div>
    </header>
  );
}

function HomepageModules() {
  const modules = [
    {
      title: 'Module 1: The Robotic Nervous System (ROS 2)',
      icon: '🤖',
      description: 'Understand the foundational middleware for robot control and communication.',
      link: '/module-01-ros2',
    },
    {
      title: 'Module 2: The Digital Twin (Gazebo & Unity)',
      icon: '🎮',
      description: 'Create high-fidelity virtual replicas of robots and environments for simulation.',
      link: '/module-02-simulation',
    },
    {
      title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      icon: '🧠',
      description: 'Leverage advanced simulation platforms for synthetic data generation and reinforcement learning.',
      link: '/module-03-isaac',
    },
    {
      title: 'Module 4: Vision-Language-Action (VLA)',
      icon: '💬',
      description: 'Integrate state-of-the-art AI models for cognitive planning and autonomous decision-making.',
      link: '/module-04-vla',
    },
  ];

  return (
    <section id="modules" className={styles.modules}>
      <div className="container">
        <div className="row">
          {modules.map((module, idx) => (
            <div key={idx} className={clsx('col col--3', styles.moduleCard)}>
              <Link to={module.link}>
                <div className="card">
                  <div className="card__header">
                    <h3>{module.icon} {module.title}</h3>
                  </div>
                  <div className="card__body">
                    <p>{module.description}</p>
                  </div>
                  <div className="card__footer">
                    <button className="button button--primary button--block">Start Module</button>
                  </div>
                </div>
              </Link>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function HomepageCourseInfo() {
  return (
    <section className={styles.courseInfo}>
      <div className="container">
        <h2>Course at a Glance</h2>
        <div className="row">
          <div className="col col--4">
            <h3>Duration</h3>
            <p>8-12 weeks (self-paced)</p>
          </div>
          <div className="col col--4">
            <h3>Format</h3>
            <p>4 modules, 12 chapters, hands-on projects</p>
          </div>
          <div className="col col--4">
            <h3>Prerequisites</h3>
            <p>Python, AI/ML basics, Linux terminal</p>
          </div>
        </div>
      </div>
    </section>
  );
}

function HomepageTechStack() {
  const tech = [
    { name: 'ROS 2 Humble', icon: '🤖' },
    { name: 'Python 3.10+', icon: '🐍' },
    { name: 'Gazebo', icon: '⚙️' },
    { name: 'Unity', icon: '✨' },
    { name: 'NVIDIA Isaac Sim', icon: '🚀' },
  ];
  return (
    <section className={styles.techStack}>
      <div className="container">
        <h2>What You'll Use</h2>
        <div className="row">
          {tech.map((item, idx) => (
            <div key={idx} className={clsx('col col--2', styles.techBadge)}>
              {item.icon} {item.name}
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}


function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageModules />
        <HomepageCourseInfo />
        <HomepageTechStack />
      </main>
    </Layout>
  );
}

export default Home;