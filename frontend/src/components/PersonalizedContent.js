// frontend/src/components/PersonalizedContent.js
import React from 'react';
import { useAuth } from '@site/src/context/AuthContext';

const PersonalizedContent = ({ children }) => {
  const { user, isAuthenticated } = useAuth();

  // Extract content based on tags
  const getContentByLevel = (level) => {
    const childrenArray = React.Children.toArray(children);
    const targetChild = childrenArray.find(
      (child) => child.props && child.props.mdxType === level
    );
    return targetChild ? targetChild.props.children : null;
  };

  // Determine the user's personalization level
  // Default to 'Intermediate' if not authenticated or profile not set
  let personalizationLevel = 'Intermediate';
  if (isAuthenticated && user && user.profile && user.profile.python_level) {
    // This assumes 'python_level' is a good proxy for overall personalization level
    // In a real app, this logic would be more sophisticated, possibly mapping levels
    personalizationLevel = user.profile.python_level;
  }

  // Render the appropriate content
  switch (personalizationLevel) {
    case 'Beginner':
      return getContentByLevel('Beginner');
    case 'Advanced':
      return getContentByLevel('Advanced');
    case 'Intermediate': // Default
    default:
      return getContentByLevel('Intermediate');
  }
};

// Sub-components for clarity in MDX
export const Beginner = ({ children }) => <>{children}</>;
export const Intermediate = ({ children }) => <>{children}</>;
export const Advanced = ({ children }) => <>{children}</>;

export default PersonalizedContent;