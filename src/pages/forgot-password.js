// frontend/src/pages/ForgotPassword.js
import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import axios from 'axios';

const ForgotPassword = () => {
  const [email, setEmail] = useState('');
  const [message, setMessage] = useState('');
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);
  const history = useHistory();

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');
    setMessage('');
    setLoading(true);

    try {
      // Assuming Better Auth has a /auth/forgot-password endpoint
      const response = await axios.post('http://localhost:3001/api/auth/forgot-password', { email });

      if (response.status === 200) {
        setMessage('If an account with that email exists, a password reset link has been sent.');
      } else {
        setError(response.data.message || 'Failed to initiate password reset.');
      }
    } catch (err) {
      setError(err.response?.data?.message || 'An error occurred. Please try again.');
      console.error('Forgot password error:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <Layout title="Forgot Password" description="Reset your password.">
      <div className="container margin-vert--lg">
        <h1>Forgot Password</h1>
        <p>Enter your email address and we'll send you a link to reset your password.</p>
        <form onSubmit={handleSubmit}>
          <div className="margin-bottom--md">
            <label htmlFor="email">Email</label>
            <input
              type="email"
              id="email"
              className="form-control"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
            />
          </div>
          {message && <div className="alert alert--success">{message}</div>}
          {error && <div className="alert alert--danger">{error}</div>}
          <button type="submit" className="button button--primary button--block" disabled={loading}>
            {loading ? 'Sending...' : 'Send Reset Link'}
          </button>
        </form>
        <div className="margin-top--md">
          Remember your password? <a href="/login">Log In</a>
        </div>
      </div>
    </Layout>
  );
};

export default ForgotPassword;
