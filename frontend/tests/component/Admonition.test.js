import React from 'react';
import { render } from '@testing-library/react';
import Admonition from '@site/src/theme/Admonition';

describe('Admonition', () => {
  it('renders a tip admonition', () => {
    const { getByText } = render(
      <Admonition type="tip" title="Tip">
        This is a tip.
      </Admonition>
    );
    expect(getByText('Tip')).toBeInTheDocument();
    expect(getByText('This is a tip.')).toBeInTheDocument();
  });

  it('renders a note admonition', () => {
    const { getByText } = render(
      <Admonition type="note" title="Note">
        This is a note.
      </Admonition>
    );
    expect(getByText('Note')).toBeInTheDocument();
    expect(getByText('This is a note.')).toBeInTheDocument();
  });
});
