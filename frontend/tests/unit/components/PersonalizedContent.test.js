// frontend/tests/unit/components/PersonalizedContent.test.js
import React from 'react';
import { render, screen } from '@testing-library/react';
import { PersonalizedContent, Beginner, Intermediate, Advanced } from '../../../src/components/PersonalizedContent';
import { UserPersonalizationProvider } from '../../../src/context/UserPersonalizationContext';

describe('PersonalizedContent', () => {
  it('renders intermediate content by default', () => {
    render(
      <UserPersonalizationProvider>
        <PersonalizedContent>
          <Beginner>Beginner Content</Beginner>
          <Intermediate>Intermediate Content</Intermediate>
          <Advanced>Advanced Content</Advanced>
        </PersonalizedContent>
      </UserPersonalizationProvider>
    );
    expect(screen.getByText('Intermediate Content')).toBeInTheDocument();
  });

  // Add more tests for different personalization levels
});
