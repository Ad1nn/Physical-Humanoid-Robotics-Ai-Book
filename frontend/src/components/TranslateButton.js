// frontend/src/components/TranslateButton.js
import React, { useState } from 'react';
import { useAuth } from '@site/src/context/AuthContext';
import axios from 'axios';

const TranslateButton = ({ chapterId, originalContent, onTranslate }) => {
  const { isAuthenticated } = useAuth();
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const handleTranslate = async () => {
    if (!isAuthenticated) {
      setError('You must be logged in to use the translation feature.');
      return;
    }

    setLoading(true);
    setError('');

    try {
      const response = await axios.post('/api/translate', {
        text: originalContent,
        targetLanguage: 'ur', // As per spec, translate to Urdu
        chapterId: chapterId,
      });

      if (response.data && response.data.translation) {
        onTranslate(response.data.translation);
      } else {
        setError('Translation API returned no content.');
      }
    } catch (err) {
      setError(err.response?.data?.message || 'Failed to translate content.');
      console.error('Translation error:', err);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="margin-bottom--md">
      {error && <div className="alert alert--danger">{error}</div>}
      <button
        className="button button--primary"
        onClick={handleTranslate}
        disabled={loading || !isAuthenticated}
      >
        {loading ? 'Translating...' : 'Translate to Urdu'}
      </button>
      {!isAuthenticated && (
        <span className="margin-left--sm text--warning"> (Login to enable)</span>
      )}
    </div>
  );
};

export default TranslateButton;
