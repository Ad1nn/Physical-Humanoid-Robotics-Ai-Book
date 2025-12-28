import React, { useEffect, useState } from 'react';

const HeavyContent = () => {
  const [data, setData] = useState(null);

  useEffect(() => {
    // Simulate fetching heavy data or rendering a complex component
    const timer = setTimeout(() => {
      setData("I'm heavy content loaded after a delay!");
    }, 2000); // Simulate 2 seconds loading time

    return () => clearTimeout(timer);
  }, []);

  return (
    <div style={{ padding: '20px', border: '1px solid #ccc', marginTop: '20px' }}>
      <h4>Heavy Content Component</h4>
      {data ? <p>{data}</p> : <p>Loading heavy content...</p>}
    </div>
  );
};

export default HeavyContent;
