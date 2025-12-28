// frontend/src/pages/Profile.js
import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../../frontend/src/context/AuthContext';
import { useHistory } from '@docusaurus/router'; // Assuming docusaurus router is available
import axios from 'axios';

const Profile = () => {
  const { user, isAuthenticated, login, logout } = useAuth();
  const history = useHistory();
  const [profileData, setProfileData] = useState(null);
  const [loading, setLoading] = useState(true);
  const [saving, setSaving] = useState(false);
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');
  const [profileFetched, setProfileFetched] = useState(false); // New state variable

  useEffect(() => {
    console.log('Profile Page - useEffect triggered');
    console.log('User from useAuth():', user);
    console.log('IsAuthenticated from useAuth():', isAuthenticated);

    if (!isAuthenticated) {
      console.log('Profile: Not authenticated, redirecting to /login');
      history.push('/Physical-Humanoid-Robotics-Ai-Book/login'); // Use correct Docusaurus base URL
      return;
    }

    const fetchProfile = async () => {
      console.log('Profile: fetchProfile called');
      try {
        setLoading(true);
        const url = `http://localhost:3001/api/users/profile/${user.id}`;
        console.log('Profile: fetching profile from URL:', url);
        // Assuming a GET endpoint for user profile by user ID
        const response = await axios.get(url);
        console.log('Profile: fetchProfile success, data:', response.data);
        setProfileData(response.data.profile);
      } catch (err) {
        console.error('Fetch profile error:', err);
        console.log('Error response:', err.response);
        if (err.response && err.response.status === 404) {
          setProfileData(null);
        } else {
          setError(err.response?.data?.message || 'Failed to fetch profile.');
        }
      } finally {
        setLoading(false);
        setProfileFetched(true); // Set profileFetched to true after fetching
        console.log('Profile: setLoading(false) in finally');
      }
    };

    if (user && user.id && !profileFetched) { // Only call if not already fetched
      fetchProfile();
    } else if (!user || !user.id) {
      console.log('Profile: user or user.id not available, not calling fetchProfile yet.');
    }
  }, [isAuthenticated, user, history, profileFetched]); // Add profileFetched to dependencies

  const handleCreateProfile = async () => {
    setError('');
    setSuccess('');
    setSaving(true);

    try {
      const response = await axios.post(`http://localhost:3001/api/users/user-profile`, { userId: user.id });
      if (response.data && response.data.profile) {
        setSuccess('Profile created successfully!');
        setProfileData(response.data.profile);
      } else {
        setError(response.data.message || 'Failed to create profile.');
      }
    } catch (err) {
      setError(err.response?.data?.message || 'An error occurred while creating profile.');
      console.error('Create profile error:', err);
    } finally {
      setSaving(false);
    }
  };

  const handleChange = (e) => {
    const { name, value } = e.target;
    setProfileData((prevData) => ({ ...prevData, [name]: value }));
  };

  const handleUpdateProfile = async (e) => {
    e.preventDefault();
    setError('');
    setSuccess('');
    setSaving(true);

    try {
      const response = await axios.put(`http://localhost:3001/api/users/profile/${user.id}`, profileData);
      if (response.data && response.data.profile) {
        setSuccess('Profile updated successfully!');
        login({ ...user, profile: response.data.profile }); // Update user context
      } else {
        setError(response.data.message || 'Failed to update profile.');
      }
    } catch (err) {
      setError(err.response?.data?.message || 'An error occurred while updating profile.');
      console.error('Update profile error:', err);
    } finally {
      setSaving(false);
    }
  };

  const handleDeleteAccount = async () => {
    if (window.confirm('Are you sure you want to delete your account? This action cannot be undone.')) {
      try {
        setLoading(true);
        await axios.delete(`http://localhost:3001/api/users/${user.id}`);
        logout(); // Log out user after deletion
        history.push('/Physical-Humanoid-Robotics-Ai-Book/'); // Redirect to homepage
      } catch (err) {
        setError(err.response?.data?.message || 'Failed to delete account.');
        console.error('Delete account error:', err);
      } finally {
        setLoading(false);
      }
    }
  };

  console.log('Rendering Profile component with profileData:', profileData);

  console.log('Checking loading condition: loading =', loading);
  if (loading) {
    return (
      <Layout title="Profile" description="User Profile Management">
        <div className="container margin-vert--lg">
          <h1>Loading Profile...</h1>
        </div>
      </Layout>
    );
  }

  console.log('Checking isAuthenticated condition: isAuthenticated =', isAuthenticated);
  if (!isAuthenticated) {
    return (
      <Layout title="Profile" description="User Profile Management">
        <div className="container margin-vert--lg">
          <h1>Please log in to view your profile.</h1>
        </div>
      </Layout>
    );
  }

  console.log('Checking profileData condition: profileData =', profileData);
  if (profileData === null) {
    return (
      <Layout title="Profile" description="User Profile Management">
        <div className="container margin-vert--lg">
          <h1>No Profile Found</h1>
          <p>You don't have a profile yet. Create one now!</p>
          <button
            onClick={handleCreateProfile}
            className="button button--primary"
            disabled={saving}
          >
            {saving ? 'Creating...' : 'Create Profile'}
          </button>
          {error && <div className="alert alert--danger margin-top--lg">{error}</div>}
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Profile" description="User Profile Management">
      <div className="container margin-vert--lg">
        <h1>Your Profile</h1>
        {error && <div className="alert alert--danger">{error}</div>}
        {success && <div className="alert alert--success">{success}</div>}
        <form onSubmit={handleUpdateProfile}>
          <div className="margin-bottom--md">
            <label htmlFor="email">Email</label>
            <input
              type="email"
              id="email"
              className="form-control"
              value={user.email}
              disabled
            />
          </div>
          <div className="margin-bottom--md">
            <label htmlFor="first_name">First Name</label>
            <input
              type="text"
              id="first_name"
              name="first_name"
              className="form-control"
              value={profileData.first_name || ''}
              onChange={handleChange}
            />
          </div>
          <div className="margin-bottom--md">
            <label htmlFor="last_name">Last Name</label>
            <input
              type="text"
              id="last_name"
              name="last_name"
              className="form-control"
              value={profileData.last_name || ''}
              onChange={handleChange}
            />
          </div>
          <div className="margin-bottom--md">
            <label htmlFor="bio">Bio</label>
            <textarea
              id="bio"
              name="bio"
              className="form-control"
              value={profileData.bio || ''}
              onChange={handleChange}
            />
          </div>
          <button
            type="submit"
            className="button button--primary"
            disabled={saving}
          >
            {saving ? 'Saving...' : 'Save Profile'}
          </button>
        </form>
        <hr className="margin-vert--lg" />
        <div className="text--center">
          <button
            onClick={handleDeleteAccount}
            className="button button--danger"
            disabled={loading}
          >
            Delete Account
          </button>
        </div>
      </div>
    </Layout>
  );
};

export default Profile;