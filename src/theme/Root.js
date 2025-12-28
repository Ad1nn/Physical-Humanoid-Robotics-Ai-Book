import React from 'react';
import { AuthProvider } from '../../frontend/src/context/AuthContext';
// import { BetterAuthProvider } from 'better-auth/react'; // This is a placeholder

// Placeholder for BetterAuthProvider until we have the real one
const BetterAuthProvider = ({ children }) => {
    return <>{children}</>;
};


export default function Root({children}) {
  return (
    <BetterAuthProvider>
      <AuthProvider>
        {children}
      </AuthProvider>
    </BetterAuthProvider>
  );
}
