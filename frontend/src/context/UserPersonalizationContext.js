// frontend/src/context/UserPersonalizationContext.js
import React, { createContext, useContext, useState, useEffect } from 'react';
// import BetterAuth from '../services/auth'; // Assuming auth service will provide user
// import profileService from '../services/profile'; // Assuming profile service will provide personalization data

const UserPersonalizationContext = createContext(null);

export const UserPersonalizationProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [personalizationLevel, setPersonalizationLevel] = useState('intermediate'); // Default level

  // This effect would fetch user and profile data
  useEffect(() => {
    const fetchUserData = async () => {
      // In a real application, you would fetch the logged-in user
      // and their profile/preferences to determine the personalizationLevel.
      // For now, we'll simulate this.

      // const currentUser = await BetterAuth.getCurrentUser();
      // setUser(currentUser);

      // if (currentUser) {
      //   const userProfile = await profileService.getProfile(currentUser.id);
      //   if (userProfile && userProfile.python_level) {
      //     // Example: Map python_level to personalization level
      //     if (userProfile.python_level === 'beginner') {
      //       setPersonalizationLevel('beginner');
      //     } else if (userProfile.python_level === 'advanced') {
      //       setPersonalizationLevel('advanced');
      //     } else {
      //       setPersonalizationLevel('intermediate');
      //     }
      //   }
      // }
    };
    fetchUserData();
  }, []); // Run once on component mount

  const value = {
    user,
    setUser,
    personalizationLevel,
    setPersonalizationLevel,
  };

  return (
    <UserPersonalizationContext.Provider value={value}>
      {children}
    </UserPersonalizationContext.Provider>
  );
};

export const useUserPersonalization = () => {
  const context = useContext(UserPersonalizationContext);
  if (context === undefined) {
    throw new Error('useUserPersonalization must be used within a UserPersonalizationProvider');
  }
  return context;
};
