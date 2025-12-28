import React, { useState } from 'react';
import styles from './styles.module.css';

const Quiz = ({ questions }) => {
  const [currentQuestion, setCurrentQuestion] = useState(0);
  const [selectedAnswer, setSelectedAnswer] = useState(null);
  const [showScore, setShowScore] = useState(false);
  const [score, setScore] = useState(0);

  const handleAnswerOptionClick = (isCorrect) => {
    if (isCorrect) {
      setScore(score + 1);
    }

    const nextQuestion = currentQuestion + 1;
    if (nextQuestion < questions.length) {
      setCurrentQuestion(nextQuestion);
    } else {
      setShowScore(true);
    }
  };

  return (
    <div className={styles.quiz}>
      {showScore ? (
        <div className={styles.scoreSection}>
          You scored {score} out of {questions.length}
        </div>
      ) : (
        <>
          <div className={styles.questionSection}>
            <div className={styles.questionCount} data-testid="question-count">
              <span>Question {currentQuestion + 1}</span>/{questions.length}
            </div>
            <div className={styles.questionText}>
              {questions[currentQuestion].question}
            </div>
          </div>
          <div className={styles.answerSection}>
            {questions[currentQuestion].options.map((option, index) => (
              <button
                key={index}
                onClick={() =>
                  handleAnswerOptionClick(option.isCorrect)
                }
              >
                {option.text}
              </button>
            ))}
          </div>
        </>
      )}
    </div>
  );
};

export default Quiz;
