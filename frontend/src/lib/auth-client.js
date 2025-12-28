import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: "http://localhost:3000/api/auth", // Base URL of your backend auth API
});

export const { signIn, signUp, useSession } = authClient;
