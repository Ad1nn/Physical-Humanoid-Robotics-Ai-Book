// frontend/src/pages/Login.js
import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../../frontend/src/context/AuthContext';
import { useHistory } from '@docusaurus/router'; // Assuming docusaurus router is available

const Login = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const { login } = useAuth();
  const history = useHistory();

  const handleLogin = async (e) => {
    e.preventDefault();
    setError('');

    try {
      // Replace with actual Better Auth API call
      const response = await fetch('http://localhost:3001/api/auth/login', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ email, password }),
      });

      const data = await response.json();
      console.log('Login: data received from backend:', data);

      if (response.ok) {
        // Assuming Better Auth returns some user data on successful login
        login(data); // HERE IS THE FIX
        history.push('/Physical-Humanoid-Robotics-Ai-Book/'); // Redirect to homepage or user's dashboard
      } else {
        setError(data.message || 'Login failed');
      }
    } catch (err) {
      setError('An error occurred during login');
      console.error(err);
    }
  };

  return (
    <Layout title="Log In" description="Log in to your account.">
      <div className="container margin-vert--lg">
        <h1>Log In</h1>
        <form onSubmit={handleLogin}>
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
          <div className="margin-bottom--md">
            <label htmlFor="password">Password</label>
            <input
              type="password"
              id="password"
              className="form-control"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
            />
          </div>
          {error && <div className="alert alert--danger">{error}</div>}
          <button type="submit" className="button button--primary button--block">
            Log In
          </button>
        </form>
        <div className="margin-top--lg text--center">
          <p>Or log in with:</p>
          <div className="button-group button-group--block">
            <button className="button button--outline button--secondary margin-right--sm">
              Login with Google
            </button>
            <button className="button button--outline button--secondary">
              Login with GitHub
            </button>
          </div>
        </div>
        <div className="margin-top--md">
          Don't have an account? <a href="/signup">Sign Up</a>
        </div>
        <div className="margin-top--md">
          <a href="/forgot-password">Forgot password?</a>
        </div>
      </div>
    </Layout>
  );
};

export default Login;
