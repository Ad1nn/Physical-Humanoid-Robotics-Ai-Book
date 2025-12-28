// frontend/src/services/profile.js

const API_BASE_URL = 'http://localhost:3001/api'; // Assuming backend runs on 3001

const ProfileService = {
  // Fetch user profile
  getProfile: async (userId) => {
    try {
      // For now, userId is passed explicitly. In a real app, it would be from auth token.
      // Assuming a GET endpoint for profile (not yet implemented on backend)
      const response = await fetch(`${API_BASE_URL}/profile/user/${userId}`);
      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.message || 'Failed to fetch profile.');
      }
      return data.profile;
    } catch (error) {
      console.error('Error fetching profile:', error);
      return null;
    }
  },

  // Update user profile
  updateProfile: async (userId, profileData) => {
    try {
      const response = await fetch(`${API_BASE_URL}/profile/update-profile`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ userId, ...profileData }),
      });
      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.message || 'Failed to update profile.');
      }
      return { success: true, message: data.message };
    } catch (error) {
      console.error('Error updating profile:', error);
      return { success: false, error: error.message };
    }
  },

  // Fetch user preferences (assuming a separate endpoint or part of profile)
  getPreferences: async (userId) => {
    try {
      // Assuming a GET endpoint for preferences (not yet implemented on backend)
      const response = await fetch(`${API_BASE_URL}/profile/preferences/${userId}`);
      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.message || 'Failed to fetch preferences.');
      }
      return data.preferences;
    } catch (error) {
      console.error('Error fetching preferences:', error);
      return null;
    }
  },

  // Update user preferences
  updatePreferences: async (userId, preferencesData) => {
    try {
      // Assuming a PUT endpoint for preferences (not yet implemented on backend)
      const response = await fetch(`${API_BASE_URL}/profile/update-preferences`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ userId, ...preferencesData }),
      });
      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.message || 'Failed to update preferences.');
      }
      return { success: true, message: data.message };
    } catch (error) {
      console.error('Error updating preferences:', error);
      return { success: false, error: error.message };
    }
  },

  // Submit questionnaire (already implemented backend POST /api/profile/questionnaire)
  submitQuestionnaire: async (userId, questionnaireData) => {
    try {
      const response = await fetch(`${API_BASE_URL}/profile/questionnaire`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ userId, ...questionnaireData }),
      });
      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.message || 'Failed to submit questionnaire.');
      }
      return { success: true, message: data.message };
    } catch (error) {
      console.error('Error submitting questionnaire:', error);
      return { success: false, error: error.message };
    }
  },

  // Delete user account
  deleteAccount: async (userId) => {
    try {
      const response = await fetch(`${API_BASE_URL}/profile/delete-account`, {
        method: 'DELETE',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ userId }),
      });
      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.message || 'Failed to delete account.');
      }
      return { success: true, message: data.message };
    } catch (error) {
      console.error('Error deleting account:', error);
      return { success: false, error: error.message };
    }
  },
};

export default ProfileService;
