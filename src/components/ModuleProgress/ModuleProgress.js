import React, { useState, useEffect } from 'react';
import ProgressBar from '../ProgressBar/ProgressBar';
import progressData from '@site/src/data/progress.json';

const ModuleProgress = ({ module }) => {
  const [progress, setProgress] = useState(0);

  useEffect(() => {
    if (progressData[module]) {
      setProgress(progressData[module]);
    }
  }, [module]);

  return (
    <div>
      <h4>Module Progress</h4>
      <ProgressBar progress={progress} />
    </div>
  );
};

export default ModuleProgress;
