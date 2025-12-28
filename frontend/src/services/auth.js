// frontend/src/services/auth.js

const API_BASE_URL = 'http://localhost:3001/api'; // Assuming backend runs on 3001

const BetterAuth = {
  init: () => {
    console.log('Better Auth service initialized.');
  },

  signUp: async (email, password) => {
    try {
      const response = await fetch(`${API_BASE_URL}/auth/signup`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.message || 'Signup failed.');
      }

      console.log('User signed up successfully:', data);
      return { success: true, userId: data.userId };
    } catch (error) {
      console.error('Signup failed:', error);
      return { success: false, error: error.message };
    }
  },

  // This will be used for actual login in a later task
  signIn: async (email, password) => {
    try {
      const response = await fetch(`${API_BASE_URL}/auth/signin`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.message || 'Sign-in failed.');
      }

      console.log('User signed in successfully:', data);
      // Store user info (e.g., in local storage, context, or Redux)
      return { success: true, user: data.user };
    } catch (error) {
      console.error('Sign-in failed:', error);
      return { success: false, error: error.message };
    }
  },

  logout: async () => {
    try {
      console.log('User logged out.');
      await new Promise(resolve => setTimeout(resolve, 500));
      return { success: true };
    } catch (error) {
      console.error('Logout failed:', error);
      return { success: false, error: error.message };
    }
  },

  getCurrentUser: async () => {
    try {
      console.log('Fetching current user.');
      await new Promise(resolve => setTimeout(resolve, 300));
      const user = null; 
      return user;
    } catch (error) {
      console.error('Failed to get current user:', error);
      return null;
    }
  },
};

export default BetterAuth;
