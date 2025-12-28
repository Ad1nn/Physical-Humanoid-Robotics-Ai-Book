import React, { createContext, useContext, useState, useEffect } from 'react';
import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

const AuthContext = createContext();

export const AuthProvider = ({ children }) => {
  console.log('AuthContext: AuthProvider rendering...');
  const [user, setUser] = useState(null);
  const [isAuthenticated, setIsAuthenticated] = useState(false);

  useEffect(() => {
    if (ExecutionEnvironment.canUseDOM) {
      console.log('AuthContext: Initializing user from localStorage (client-side)');
      const storedUser = localStorage.getItem('user');
      const storedIsAuthenticated = localStorage.getItem('isAuthenticated');

      try {
        if (storedUser) {
          setUser(JSON.parse(storedUser));
        }
        setIsAuthenticated(storedIsAuthenticated === 'true');
      } catch (e) {
        console.error("AuthContext: Error parsing stored user from localStorage on initialization", e);
        localStorage.removeItem('user'); // Clear invalid data
        localStorage.removeItem('isAuthenticated');
        setUser(null);
        setIsAuthenticated(false);
      }
    }
  }, []); // Run only once on mount

  useEffect(() => {
    if (ExecutionEnvironment.canUseDOM) {
      console.log('AuthContext: useEffect running. user:', user, 'isAuthenticated:', isAuthenticated);
      // Sync localStorage when user or isAuthenticated changes
      if (user) {
        localStorage.setItem('user', JSON.stringify(user));
      } else {
        localStorage.removeItem('user');
      }
      localStorage.setItem('isAuthenticated', isAuthenticated.toString());
    }
  }, [user, isAuthenticated]);

  const login = (userData) => {
    console.log('AuthContext: login called with userData:', userData);
    setUser(userData);
    setIsAuthenticated(true);
  };

  const logout = () => {
    console.log('AuthContext: logout called');
    setUser(null);
    setIsAuthenticated(false);
  };

  return (
    <AuthContext.Provider value={{ user, isAuthenticated, login, logout }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  return useContext(AuthContext);
};
