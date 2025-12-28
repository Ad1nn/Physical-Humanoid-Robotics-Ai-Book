// frontend/src/components/TranslationToggle.js
import React, { useState } from 'react';
import { useUserPersonalization } from '../context/UserPersonalizationContext';

function TranslationToggle() {
  const { personalizationLevel, setPersonalizationLevel } = useUserPersonalization();
  const [selectedLanguage, setSelectedLanguage] = useState('en'); // Default to English

  // In a real application, the user's language preference would be loaded
  // from their user_preferences and updated in the context.
  // For now, we'll just use a local state and update context.

  const handleLanguageChange = (e) => {
    const newLanguage = e.target.value;
    setSelectedLanguage(newLanguage);
    // Update personalization context with new language preference
    // This will eventually trigger content re-rendering in the selected language
    // For now, simply update the context.
    // setPersonalizationLevel will need to be updated to handle language too
    console.log(`Language changed to: ${newLanguage}`);
    // Example: send update to backend
  };

  return (
    <div className="translation-toggle">
      <label htmlFor="language-select" className="margin-right--sm">Language:</label>
      <select id="language-select" value={selectedLanguage} onChange={handleLanguageChange} className="select--sm">
        <option value="en">English</option>
        <option value="ur">Urdu</option>
        {/* Add more languages as needed */}
      </select>
    </div>
  );
}

export default TranslationToggle;
