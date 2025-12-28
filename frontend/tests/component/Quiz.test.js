import React from 'react';
import { render, fireEvent } from '@testing-library/react';
import Quiz from '@site/src/components/Quiz/Quiz';

const questions = [
  {
    question: 'What is the capital of France?',
    options: [
      { text: 'London', isCorrect: false },
      { text: 'Paris', isCorrect: true },
      { text: 'Berlin', isCorrect: false },
      { text: 'Madrid', isCorrect: false },
    ],
  },
  {
    question: 'What is 2 + 2?',
    options: [
      { text: '3', isCorrect: false },
      { text: '4', isCorrect: true },
      { text: '5', isCorrect: false },
      { text: '6', isCorrect: false },
    ],
  },
];

describe('Quiz', () => {
  it('renders the first question', () => {
    const { getByText, getByTestId } = render(<Quiz questions={questions} />);
    expect(getByTestId('question-count')).toHaveTextContent('Question 1/2');
    expect(getByText('What is the capital of France?')).toBeInTheDocument();
  });

  it('moves to the next question after answering', () => {
    const { getByText, getByTestId } = render(<Quiz questions={questions} />);
    fireEvent.click(getByText('Paris'));
    expect(getByTestId('question-count')).toHaveTextContent('Question 2/2');
    expect(getByText('What is 2 + 2?')).toBeInTheDocument();
  });

  it('shows the score after answering all questions', () => {
    const { getByText } = render(<Quiz questions={questions} />);
    fireEvent.click(getByText('Paris'));
    fireEvent.click(getByText('4'));
    expect(getByText('You scored 2 out of 2')).toBeInTheDocument();
  });
});
