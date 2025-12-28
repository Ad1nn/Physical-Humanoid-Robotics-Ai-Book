// frontend/tests/unit/services/auth.test.js
import BetterAuth from '../../../src/services/auth';

// Mock fetch API
global.fetch = jest.fn(() =>
  Promise.resolve({
    ok: true,
    json: () => Promise.resolve({ message: 'Success', userId: 'test-user-id' }),
  })
);

describe('BetterAuth Service', () => {
  beforeEach(() => {
    fetch.mockClear();
  });

  it('should call signup API successfully', async () => {
    const response = await BetterAuth.signUp('test@example.com', 'password123');
    expect(response.success).toBe(true);
    expect(fetch).toHaveBeenCalledTimes(1);
    expect(fetch).toHaveBeenCalledWith(
      'http://localhost:3001/api/auth/signup',
      expect.objectContaining({
        method: 'POST',
        body: JSON.stringify({ email: 'test@example.com', password: 'password123' }),
      })
    );
  });

  it('should handle signup API error', async () => {
    fetch.mockImplementationOnce(() =>
      Promise.resolve({
        ok: false,
        json: () => Promise.resolve({ message: 'User already exists' }),
      })
    );
    const response = await BetterAuth.signUp('existing@example.com', 'password123');
    expect(response.success).toBe(false);
    expect(response.error).toBe('User already exists');
  });

  // Add tests for signIn, logout, getCurrentUser
});
