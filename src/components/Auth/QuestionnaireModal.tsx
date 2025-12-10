/**
 * Background Questionnaire Modal
 * Collects user expertise and learning preferences
 */

import React, { useState, useEffect } from 'react';
import { authClient, QuestionnaireData } from '../auth-client';
import styles from './QuestionnaireModal.module.css';

interface QuestionnaireModalProps {
  isOpen: boolean;
  onClose: () => void;
}

export function QuestionnaireModal({ isOpen, onClose }: QuestionnaireModalProps) {
  const [step, setStep] = useState(1);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');

  // Form state
  const [expertiseLevel, setExpertiseLevel] = useState<'beginner' | 'intermediate' | 'expert'>('intermediate');
  const [programmingLanguages, setProgrammingLanguages] = useState<string[]>([]);
  const [learningGoals, setLearningGoals] = useState('');
  const [experienceYears, setExperienceYears] = useState<number | undefined>();
  const [roboticsExperience, setRoboticsExperience] = useState<'none' | 'hobbyist' | 'professional'>('none');
  const [primaryInterest, setPrimaryInterest] = useState<'simulation' | 'hardware' | 'ai' | 'all'>('all');
  const [learningStyle, setLearningStyle] = useState<'examples' | 'theory' | 'projects'>('examples');

  // Pre-populate with existing profile data when editing
  useEffect(() => {
    if (isOpen) {
      const profile = authClient.getProfile();
      if (profile) {
        setExpertiseLevel(profile.expertise_level || 'intermediate');
        setProgrammingLanguages(profile.programming_languages || []);
        setLearningGoals(profile.learning_goals || '');
      }
      setStep(1); // Reset to first step when opening
    }
  }, [isOpen]);

  if (!isOpen) return null;

  const languages = ['Python', 'C++', 'JavaScript', 'TypeScript', 'Rust', 'Go', 'Java', 'C#'];

  const toggleLanguage = (lang: string) => {
    if (programmingLanguages.includes(lang)) {
      setProgrammingLanguages(programmingLanguages.filter(l => l !== lang));
    } else {
      setProgrammingLanguages([...programmingLanguages, lang]);
    }
  };

  const handleSubmit = async () => {
    setError('');
    setIsLoading(true);

    try {
      const data: QuestionnaireData = {
        expertise_level: expertiseLevel,
        programming_languages: programmingLanguages,
        learning_goals: learningGoals || undefined,
        programming_experience_years: experienceYears,
        robotics_experience: roboticsExperience,
        primary_interest: primaryInterest,
        preferred_learning_style: learningStyle,
      };

      await authClient.submitQuestionnaire(data);
      onClose();
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to save profile');
    } finally {
      setIsLoading(false);
    }
  };

  const handleSkip = () => {
    onClose();
  };

  const renderStep1 = () => (
    <>
      <h3 className={styles.stepTitle}>What's your experience level?</h3>

      <div className={styles.optionGrid}>
        {[
          { value: 'beginner', label: 'Beginner', desc: 'New to programming or robotics' },
          { value: 'intermediate', label: 'Intermediate', desc: 'Some experience with coding and/or robotics' },
          { value: 'expert', label: 'Expert', desc: 'Professional or advanced hobbyist' },
        ].map(opt => (
          <button
            key={opt.value}
            className={`${styles.optionCard} ${expertiseLevel === opt.value ? styles.selected : ''}`}
            onClick={() => setExpertiseLevel(opt.value as typeof expertiseLevel)}
          >
            <span className={styles.optionLabel}>{opt.label}</span>
            <span className={styles.optionDesc}>{opt.desc}</span>
          </button>
        ))}
      </div>

      <div className={styles.field}>
        <label>Years of programming experience</label>
        <input
          type="number"
          min="0"
          max="50"
          value={experienceYears || ''}
          onChange={(e) => setExperienceYears(e.target.value ? parseInt(e.target.value) : undefined)}
          placeholder="e.g., 3"
        />
      </div>
    </>
  );

  const renderStep2 = () => (
    <>
      <h3 className={styles.stepTitle}>Which programming languages do you know?</h3>

      <div className={styles.languageGrid}>
        {languages.map(lang => (
          <button
            key={lang}
            className={`${styles.languageChip} ${programmingLanguages.includes(lang) ? styles.selected : ''}`}
            onClick={() => toggleLanguage(lang)}
          >
            {lang}
          </button>
        ))}
      </div>

      <div className={styles.field}>
        <label>Robotics experience</label>
        <select
          value={roboticsExperience}
          onChange={(e) => setRoboticsExperience(e.target.value as typeof roboticsExperience)}
        >
          <option value="none">No robotics experience</option>
          <option value="hobbyist">Hobbyist / DIY projects</option>
          <option value="professional">Professional / Research</option>
        </select>
      </div>
    </>
  );

  const renderStep3 = () => (
    <>
      <h3 className={styles.stepTitle}>What are your learning goals?</h3>

      <div className={styles.field}>
        <label>Primary interest</label>
        <div className={styles.radioGroup}>
          {[
            { value: 'simulation', label: 'Simulation & Virtual Robots' },
            { value: 'hardware', label: 'Physical Hardware & Actuators' },
            { value: 'ai', label: 'AI & Machine Learning for Robotics' },
            { value: 'all', label: 'All of the above' },
          ].map(opt => (
            <label key={opt.value} className={styles.radioLabel}>
              <input
                type="radio"
                name="interest"
                value={opt.value}
                checked={primaryInterest === opt.value}
                onChange={() => setPrimaryInterest(opt.value as typeof primaryInterest)}
              />
              {opt.label}
            </label>
          ))}
        </div>
      </div>

      <div className={styles.field}>
        <label>Preferred learning style</label>
        <div className={styles.radioGroup}>
          {[
            { value: 'examples', label: 'Code examples & hands-on' },
            { value: 'theory', label: 'Theory & concepts first' },
            { value: 'projects', label: 'Project-based learning' },
          ].map(opt => (
            <label key={opt.value} className={styles.radioLabel}>
              <input
                type="radio"
                name="style"
                value={opt.value}
                checked={learningStyle === opt.value}
                onChange={() => setLearningStyle(opt.value as typeof learningStyle)}
              />
              {opt.label}
            </label>
          ))}
        </div>
      </div>

      <div className={styles.field}>
        <label>Your learning goals (optional)</label>
        <textarea
          value={learningGoals}
          onChange={(e) => setLearningGoals(e.target.value)}
          placeholder="e.g., Build autonomous robots using ROS 2, understand humanoid locomotion..."
          rows={3}
        />
      </div>
    </>
  );

  const isEditing = authClient.isProfileCompleted();

  return (
    <div className={styles.overlay}>
      <div className={styles.modal}>
        <div className={styles.header}>
          <h2>{isEditing ? 'Edit Your Profile' : 'Personalize Your Experience'}</h2>
          <p>{isEditing ? 'Update your learning preferences' : 'Help us tailor the content to your background'}</p>
        </div>

        <div className={styles.progress}>
          {[1, 2, 3].map(s => (
            <div
              key={s}
              className={`${styles.progressStep} ${s <= step ? styles.active : ''}`}
            />
          ))}
        </div>

        <div className={styles.content}>
          {step === 1 && renderStep1()}
          {step === 2 && renderStep2()}
          {step === 3 && renderStep3()}
        </div>

        {error && <div className={styles.error}>{error}</div>}

        <div className={styles.actions}>
          {step > 1 && (
            <button
              className={styles.backButton}
              onClick={() => setStep(step - 1)}
              disabled={isLoading}
            >
              Back
            </button>
          )}

          <button className={styles.skipButton} onClick={handleSkip} disabled={isLoading}>
            {isEditing ? 'Cancel' : 'Skip for now'}
          </button>

          {step < 3 ? (
            <button
              className={styles.nextButton}
              onClick={() => setStep(step + 1)}
            >
              Next
            </button>
          ) : (
            <button
              className={styles.submitButton}
              onClick={handleSubmit}
              disabled={isLoading}
            >
              {isLoading ? 'Saving...' : (isEditing ? 'Save Changes' : 'Complete')}
            </button>
          )}
        </div>
      </div>
    </div>
  );
}
