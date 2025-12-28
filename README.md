# Physical AI & Humanoid Robotics: From Digital Brain to Physical Body

This repository hosts the content and code examples for the "Physical AI & Humanoid Robotics: From Digital Brain to Physical Body" book. The book is built using Docusaurus, a modern static website generator.

## Project Structure

*   `docs/`: Contains all the book content, organized by modules and chapters.
*   `code-examples/`: Houses all runnable code examples, organized by module and chapter.
*   `specs/`: Stores the Spec-Driven Development (SDD) artifacts (specifications, plans, tasks) for the book's features.
*   `static/`: Contains static assets like images and generic code snippets.

## Docusaurus Development Setup

To get the Docusaurus site running locally, follow these steps:

1.  **Clone the Repository**:
    ```bash
    git clone https://github.com/Ad1nn/Physical-Humanoid-Robotics-Ai-Book.git
    cd Physical-Humanoid-Robotics-Ai-Book
    ```

2.  **Install Node.js Dependencies**:
    Docusaurus requires Node.js. Ensure you have Node.js 18.x+ installed.
    ```bash
    npm install
    ```

3.  **Start the Development Server**:
    ```bash
    npm start
    ```
    This will start a local development server and open your browser to `http://localhost:3000`. The site will automatically reload as you make changes.

4.  **Build the Static Site**:
    To generate the static HTML, CSS, and JavaScript files for deployment:
    ```bash
    npm run build
    ```
    The generated files will be placed in the `build/` directory.

## New Features: Personalization, Authentication, and Translation

This project includes several new features designed to enhance the learning experience:

*   **Authentication**: Users can create accounts and sign in using email/password or OAuth (Google, GitHub). This allows for a personalized experience and the ability to save progress.
*   **Personalization**: The content complexity and presentation adapt to the user's declared background and expertise. This is designed to provide a tailored learning experience for users of all skill levels.
*   **Translation**: The platform will provide high-quality, real-time translation to support a global audience, starting with Urdu.

These features are designed to be non-intrusive and optional. Users can still access the book's content without creating an account.

## ROS 2 Development Environment

For running the ROS 2 code examples and projects, a dedicated development environment is recommended. Please refer to the [Prerequisites & Setup Guide](/prerequisites) in the book documentation for detailed instructions on setting up your ROS 2 environment, preferably using Docker.

## Contributing

Contributions are welcome! Please refer to the contribution guidelines within the book's documentation for more details.

## License

This project is licensed under the MIT License for code and CC BY 4.0 for documentation.

## Image Optimization (WebP)

To ensure optimal performance and faster page load times, images used in the book should be optimized and converted to the WebP format.

*   **Why WebP?**: WebP is a modern image format that provides superior lossless and lossy compression for images on the web, resulting in smaller file sizes and faster loading.
*   **Optimized Assets Directory**: Optimized WebP images should be placed in the `static_new/img/` directory. The Docusaurus configuration is set up to prioritize serving images from this directory over the older `static/img/` directory.
*   **Optimization Process**:
    1.  **Convert Images**: Use image optimization tools (e.g., `cwebp` command-line tool, online converters, or image editing software) to convert your `.png`, `.jpg`, etc., images to `.webp` format.
    2.  **Place in `static_new`**: After conversion, place the `.webp` files in the appropriate subdirectory within `static_new/img/`.
    3.  **Update References**: When referencing images in your markdown or MDX files, ensure you link to the `.webp` version in the `static_new` directory (e.g., `/img/your-image.webp`).

This approach ensures that users with WebP-compatible browsers will receive the optimized images, significantly improving their browsing experience.

## Accessibility (WCAG 2.1 AA Compliance)

We are committed to making this book accessible to all users, including those with disabilities. To ensure a high standard of accessibility, we aim for WCAG 2.1 AA compliance.

*   **WCAG 2.1 AA**: These guidelines define how to make web content more accessible to people with disabilities, covering a wide range of recommendations for making web content more perceivable, operable, understandable, and robust.
*   **Tools for Auditing**:
    *   **Lighthouse (built into Chrome DevTools)**: Provides automated accessibility audits.
    *   **axe DevTools (browser extension)**: Offers intelligent automated accessibility testing.
    *   **Manual Testing**: Crucial for aspects not covered by automated tools, including keyboard navigation, screen reader compatibility, and logical content order.
*   **Best Practices**:
    *   Use semantic HTML elements correctly.
    *   Provide meaningful `alt` text for all images.
    *   Ensure sufficient color contrast.
    *   Implement clear and consistent navigation.
    *   Support keyboard navigation for all interactive elements.

By integrating accessibility checks throughout the development process, we can create an inclusive learning experience for everyone.

## Mobile Responsiveness

A high-quality mobile experience is crucial for our users. We strive to ensure that the website is fully responsive and provides an optimal viewing and interaction experience across various devices and screen sizes, from mobile phones to tablets and desktops.

*   **Responsive Design Principles**: The Docusaurus theme and custom components are developed with responsive design principles, using flexible grids, fluid images, and media queries to adapt layouts.
*   **Testing and Verification**:
    *   **Browser Developer Tools**: Use browser-built-in developer tools (e.g., Chrome DevTools device mode) to simulate different screen sizes and devices.
    *   **Real Device Testing**: Whenever possible, test on actual physical devices to account for touch interactions and device-specific rendering.
    *   **Playwright E2E Tests**: Our end-to-end tests are configured to run across various viewports, ensuring critical user flows and UI elements function correctly and appear as intended on different screen sizes. This helps catch regressions early in the development cycle.
    *   **Visual Regression Testing**: Consider integrating visual regression testing to automatically detect unintended UI changes across breakpoints.

## Mobile Responsiveness

A high-quality mobile experience is crucial for our users. We strive to ensure that the website is fully responsive and provides an optimal viewing and interaction experience across various devices and screen sizes, from mobile phones to tablets and desktops.

*   **Responsive Design Principles**: The Docusaurus theme and custom components are developed with responsive design principles, using flexible grids, fluid images, and media queries to adapt layouts.
*   **Testing and Verification**:
    *   **Browser Developer Tools**: Use browser-built-in developer tools (e.g., Chrome DevTools device mode) to simulate different screen sizes and devices.
    *   **Real Device Testing**: Whenever possible, test on actual physical devices to account for touch interactions and device-specific rendering.
    *   **Playwright E2E Tests**: Our end-to-end tests are configured to run across various viewports, ensuring critical user flows and UI elements function correctly and appear as intended on different screen sizes. This helps catch regressions early in the development cycle.
    *   **Visual Regression Testing**: Consider integrating visual regression testing to automatically detect unintended UI changes across breakpoints.

Consistent verification of mobile responsiveness ensures that all users have an excellent experience, regardless of their device.

## Performance Optimization (Lighthouse)

Achieving a high-performance website is critical for user engagement and search engine optimization. We target a Lighthouse performance score of **90 or above** to ensure a fast and smooth user experience.

*   **Lighthouse Audits**: Use Lighthouse (built into Chrome DevTools) to regularly audit the website's performance. Focus on metrics like First Contentful Paint (FCP), Largest Contentful Paint (LCP), Total Blocking Time (TBT), and Cumulative Layout Shift (CLS).
*   **Key Optimization Strategies**:
    *   **Image Optimization**: (Covered in "Image Optimization (WebP)" section) Ensure all images are optimized and served in efficient formats like WebP.
    *   **Lazy Loading**: Implement lazy loading for off-screen images, videos, and heavy components to reduce initial page load time.
    *   **Code Splitting**: Docusaurus inherently uses code splitting, but further optimizations can be made for custom components or large JavaScript bundles.
    *   **Minification and Compression**: Docusaurus handles JavaScript, CSS, and HTML minification during the build process. Ensure server-side compression (Gzip/Brotli) is enabled for deployed assets.
    *   **Reduce Render-Blocking Resources**: Minimize the impact of CSS and JavaScript on the critical rendering path.
    *   **Caching Strategies**: Leverage browser caching and CDN caching for static assets.
*   **Docusaurus Optimizations**:
    *   **Route Preloading**: Docusaurus preloads assets for linked pages when a link is hovered over or becomes visible, improving navigation speed.
    *   **Static Site Generation**: By generating static HTML, Docusaurus delivers fast initial loads as content doesn't need to be fetched dynamically.

Continuous monitoring and optimization are essential to maintain high performance and provide an excellent user experience.
