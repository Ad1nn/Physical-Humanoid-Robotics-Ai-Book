import React from 'react';
import styles from './styles.module.css';

const ProgressBar = ({ progress }) => {
  return (
    <div className={styles.progressBarContainer}>
      <div className={styles.progressBar} style={{ width: `${progress}%` }}>
        {progress > 0 && `${progress}%`}
      </div>
    </div>
  );
};

export default ProgressBar;
