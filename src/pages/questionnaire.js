// frontend/src/pages/Questionnaire.js
import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../../frontend/src/context/AuthContext';
import { useHistory } from '@docusaurus/router'; // Assuming docusaurus router is available

const Questionnaire = () => {
  const { user, isAuthenticated } = useAuth();
  const history = useHistory();
  const [formData, setFormData] = useState({
    python_level: '',
    ros_knowledge: '',
    ai_ml_background: '',
    simulation_experience: '',
    linux_comfort: '',
    hardware_experience: '',
    gpu_availability: '',
    learning_goal: '',
  });
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  // Redirect if not authenticated or if user profile already exists
  // (Assuming a way to check if profile already exists in AuthContext user object)
  // useEffect(() => {
  //   if (!isAuthenticated || (user && user.profileCompleted)) {
  //     history.push('/');
  //   }
  // }, [isAuthenticated, user, history]);

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData((prevData) => ({ ...prevData, [name]: value }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    if (!isAuthenticated || !user) {
      setError('You must be logged in to complete the questionnaire.');
      setLoading(false);
      return;
    }

    try {
      const response = await fetch('http://localhost:3001/api/users/user-profile', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          // Assuming an Authorization header is needed, e.g., with a token from Better Auth
          // 'Authorization': `Bearer ${user.token}`
        },
        body: JSON.stringify({ userId: user.id, ...formData }), // Assuming user.id is available
      });

      const data = await response.json();

      if (response.ok) {
        // Optionally update user context with profile data
        // login({ ...user, profileCompleted: true, profile: data.profile });
        history.push('/'); // Redirect to homepage
      } else {
        setError(data.message || 'Failed to save profile information.');
      }
    } catch (err) {
      setError('An error occurred while saving your profile.');
      console.error(err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Background Questionnaire" description="Help us personalize your learning experience.">
      <div className="container margin-vert--lg">
        <h1>Background Questionnaire</h1>
        <p>Please tell us a bit about your background so we can personalize the content for you.</p>
        <form onSubmit={handleSubmit}>
          {/* Python Level */}
          <div className="margin-bottom--md">
            <label htmlFor="python_level">What is your Python programming level?</label>
            <select name="python_level" id="python_level" className="form-control" value={formData.python_level} onChange={handleChange} required>
              <option value="">Select an option</option>
              <option value="Beginner">Beginner</option>
              <option value="Intermediate">Intermediate</option>
              <option value="Advanced">Advanced</option>
            </select>
          </div>

          {/* ROS Knowledge */}
          <div className="margin-bottom--md">
            <label htmlFor="ros_knowledge">How familiar are you with ROS (Robot Operating System)?</label>
            <select name="ros_knowledge" id="ros_knowledge" className="form-control" value={formData.ros_knowledge} onChange={handleChange} required>
              <option value="">Select an option</option>
              <option value="None">None</option>
              <option value="Basic">Basic</option>
              <option value="Intermediate">Intermediate</option>
              <option value="Advanced">Advanced</option>
            </select>
          </div>

          {/* AI/ML Background */}
          <div className="margin-bottom--md">
            <label htmlFor="ai_ml_background">Do you have a background in AI/Machine Learning?</label>
            <select name="ai_ml_background" id="ai_ml_background" className="form-control" value={formData.ai_ml_background} onChange={handleChange} required>
              <option value="">Select an option</option>
              <option value="None">None</option>
              <option value="Basic Concepts">Basic Concepts</option>
              <option value="Implemented Models">Implemented Models</option>
              <option value="Research/Expert">Research/Expert</option>
            </select>
          </div>

          {/* Simulation Experience */}
          <div className="margin-bottom--md">
            <label htmlFor="simulation_experience">What is your experience with robot simulation software (e.g., Gazebo, Unity)?</label>
            <select name="simulation_experience" id="simulation_experience" className="form-control" value={formData.simulation_experience} onChange={handleChange} required>
              <option value="">Select an option</option>
              <option value="None">None</option>
              <option value="Basic Use">Basic Use</option>
              <option value="Developed Simulations">Developed Simulations</option>
              <option value="Expert/Custom Simulations">Expert/Custom Simulations</option>
            </select>
          </div>

          {/* Linux Comfort */}
          <div className="margin-bottom--md">
            <label htmlFor="linux_comfort">How comfortable are you working in a Linux environment?</label>
            <select name="linux_comfort" id="linux_comfort" className="form-control" value={formData.linux_comfort} onChange={handleChange} required>
              <option value="">Select an option</option>
              <option value="Beginner (Basic Commands)">Beginner (Basic Commands)</option>
              <option value="Intermediate (Scripting/Admin)">Intermediate (Scripting/Admin)</option>
              <option value="Advanced (Kernel/System Dev)">Advanced (Kernel/System Dev)</option>
            </select>
          </div>

          {/* Hardware Experience */}
          <div className="margin-bottom--md">
            <label htmlFor="hardware_experience">What is your experience with robotics hardware?</label>
            <select name="hardware_experience" id="hardware_experience" className="form-control" value={formData.hardware_experience} onChange={handleChange} required>
              <option value="">Select an option</option>
              <option value="None">None</option>
              <option value="Basic (Kits/Assembly)">Basic (Kits/Assembly)</option>
              <option value="Intermediate (Custom Builds/Integration)">Intermediate (Custom Builds/Integration)</option>
              <option value="Advanced (PCB Design/Firmware)">Advanced (PCB Design/Firmware)</option>
            </select>
          </div>

          {/* GPU Availability */}
          <div className="margin-bottom--md">
            <label htmlFor="gpu_availability">Do you have access to an NVIDIA GPU (e.g., RTX series, Jetson)?</label>
            <select name="gpu_availability" id="gpu_availability" className="form-control" value={formData.gpu_availability} onChange={handleChange} required>
              <option value="">Select an option</option>
              <option value="None">None</option>
              <option value="Yes, Entry-Level">Yes, Entry-Level</option>
              <option value="Yes, Mid-Range">Yes, Mid-Range</option>
              <option value="Yes, High-End">Yes, High-End</option>
            </select>
          </div>

          {/* Learning Goal */}
          <div className="margin-bottom--md">
            <label htmlFor="learning_goal">What is your primary learning goal from this book?</label>
            <textarea name="learning_goal" id="learning_goal" className="form-control" value={formData.learning_goal} onChange={handleChange} rows="3" required></textarea>
          </div>

          {error && <div className="alert alert--danger">{error}</div>}
          <button type="submit" className="button button--primary button--block" disabled={loading}>
            {loading ? 'Saving...' : 'Submit Questionnaire'}
          </button>
        </form>
      </div>
    </Layout>
  );
};

export default Questionnaire;