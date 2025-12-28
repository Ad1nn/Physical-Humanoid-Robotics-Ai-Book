# Quickstart: Content Personalization

This guide explains how to use the new personalization features in your MDX content.

## Personalization Component

To create personalized content, use the `<PersonalizedContent>` component. This component will automatically show the correct content based on the user's personalization level.

### Example

```jsx
import PersonalizedContent from '@site/src/components/PersonalizedContent';

<PersonalizedContent>
  <Beginner>
    This is the content for beginners. It includes more detailed explanations and simpler code examples.
  </Beginner>
  <Intermediate>
    This is the default content for intermediate users. It assumes some prior knowledge.
  </Intermediate>
  <Advanced>
    This is the content for advanced users. It is more condensed and focuses on advanced topics.
  </Advanced>
</PersonalizedContent>
```

## How it Works

The `<PersonalizedContent>` component reads the user's personalization level from the React context. It then renders the content of the corresponding tag (`<Beginner>`, `<Intermediate>`, or `<Advanced>`).

If a user is not logged in, or has not completed the background questionnaire, the `<Intermediate>` content will be shown by default.
