import { test, expect } from '@playwright/test';

test.describe('Navigation', () => {
  test('should have correct title on homepage', async ({ page }) => {
    await page.goto('/');
    await expect(page).toHaveTitle('Physical AI & Humanoid Robotics');
  });

  test('should navigate to Module 1 via sidebar', async ({ page }) => {
    await page.goto('/');
    // Click on the sidebar link for Module 1
    await page.getByRole('link', { name: 'Module 1: The Robotic Nervous System (ROS 2)' }).click();
    // Expect the URL to change to the module's index page
    await expect(page).toHaveURL(/.*\/module-01-ros2$/);
    // Expect the heading on the page to be visible
    await expect(page.getByRole('heading', { name: 'Module 1: The Robotic Nervous System (ROS 2)' })).toBeVisible();
  });

  // Add more navigation tests here as needed
});