import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';

export default function Signup() {
  const history = useHistory();

  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [termsAccepted, setTermsAccepted] = useState(false);
  const [error, setError] = useState('');

  const handleSignup = async (e) => {
    e.preventDefault();
    setError('');

    if (password !== confirmPassword) {
      setError('Passwords do not match');
      return;
    }

    if (!termsAccepted) {
      setError('You must accept the terms of service');
      return;
    }

    try {
      const response = await fetch('http://localhost:3001/api/auth/signup', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json', // Important
        },
        body: JSON.stringify({ email, password }),
      });

      // Read raw text to avoid "<" parse error
      const text = await response.text();
      let data;
      try {
        data = JSON.parse(text);
      } catch {
        console.error('Server returned non-JSON response:', text);
        setError('Backend did not return JSON');
        return;
      }

      if (!response.ok) {
        setError(data.message || 'Signup failed');
        return;
      }

      console.log('Signup success:', data);
      alert('Signup successful!');
      history.push('/Physical-Humanoid-Robotics-Ai-Book/login'); // Redirect to login page

    } catch (err) {
      console.error(err);
      setError('Could not reach backend. Is it running?');
    }
  };

  return (
    <Layout title="Sign Up">
      <div className="container margin-vert--lg" style={{ maxWidth: 400 }}>
        <h1>Sign Up</h1>

        {error && <div className="alert alert--danger">{error}</div>}

        <form onSubmit={handleSignup}>
          <div className="margin-bottom--md">
            <label>Email</label>
            <input
              type="email"
              className="form-control"
              value={email}
              onChange={(e) => setEmail(e.target.value)}
              required
            />
          </div>

          <div className="margin-bottom--md">
            <label>Password</label>
            <input
              type="password"
              className="form-control"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
            />
          </div>

          <div className="margin-bottom--md">
            <label>Confirm Password</label>
            <input
              type="password"
              className="form-control"
              value={confirmPassword}
              onChange={(e) => setConfirmPassword(e.target.value)}
              required
            />
          </div>

          <div className="margin-bottom--md">
            <input
              type="checkbox"
              checked={termsAccepted}
              onChange={(e) => setTermsAccepted(e.target.checked)}
            />{' '}
            I accept the{' '}
            <a href="/docs/terms-of-service" target="_blank" rel="noopener noreferrer">
              Terms of Service
            </a>
          </div>

          <button
            type="submit"
            className="button button--primary button--block"
          >
            Sign Up
          </button>
        </form>
      </div>
    </Layout>
  );
}
