import React from 'react';
import styles from './styles.module.css';

const LearningObjectives = ({ children }) => {
  return (
    <div className={styles.learningObjectives}>
      <h3>Learning Objectives</h3>
      {children}
    </div>
  );
};

export default LearningObjectives;
