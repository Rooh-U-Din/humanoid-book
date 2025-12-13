/**
 * Profile Edit Form Component (T068)
 * Inline form for editing user profile information
 */

import React, { useState, useEffect } from 'react';
import { useAuth } from '../Auth/AuthContext';
import { authClient } from '../auth-client';
import styles from './ProfileEditForm.module.css';

interface ProfileEditFormProps {
  onSave?: () => void;
  onCancel?: () => void;
}

export function ProfileEditForm({ onSave, onCancel }: ProfileEditFormProps) {
  const { user, profile, refreshProfile } = useAuth();

  // Form state
  const [name, setName] = useState('');
  const [expertiseLevel, setExpertiseLevel] = useState<'beginner' | 'intermediate' | 'expert'>('intermediate');
  const [programmingLanguages, setProgrammingLanguages] = useState<string[]>([]);
  const [learningGoals, setLearningGoals] = useState('');

  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');
  const [success, setSuccess] = useState('');

  const availableLanguages = ['Python', 'C++', 'JavaScript', 'TypeScript', 'Rust', 'Go', 'Java', 'C#'];

  // Initialize form with current values
  useEffect(() => {
    if (user) {
      setName(user.name || '');
    }
    if (profile) {
      setExpertiseLevel(profile.expertise_level || 'intermediate');
      setProgrammingLanguages(profile.programming_languages || []);
      setLearningGoals(profile.learning_goals || '');
    }
  }, [user, profile]);

  const toggleLanguage = (lang: string) => {
    if (programmingLanguages.includes(lang)) {
      setProgrammingLanguages(programmingLanguages.filter(l => l !== lang));
    } else {
      setProgrammingLanguages([...programmingLanguages, lang]);
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setSuccess('');
    setIsLoading(true);

    try {
      await authClient.updateProfile({
        expertise_level: expertiseLevel,
        programming_languages: programmingLanguages,
        learning_goals: learningGoals || undefined,
      });

      await refreshProfile();
      setSuccess('Profile updated successfully!');
      onSave?.();
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to update profile');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <form onSubmit={handleSubmit} className={styles.form}>
      <h3 className={styles.title}>Edit Learning Profile</h3>

      {/* Expertise Level */}
      <div className={styles.field}>
        <label className={styles.label}>Experience Level</label>
        <div className={styles.radioGroup}>
          {[
            { value: 'beginner', label: 'Beginner', desc: 'New to programming or robotics' },
            { value: 'intermediate', label: 'Intermediate', desc: 'Some experience' },
            { value: 'expert', label: 'Expert', desc: 'Professional level' },
          ].map(opt => (
            <label
              key={opt.value}
              className={`${styles.radioOption} ${expertiseLevel === opt.value ? styles.selected : ''}`}
            >
              <input
                type="radio"
                name="expertise"
                value={opt.value}
                checked={expertiseLevel === opt.value}
                onChange={() => setExpertiseLevel(opt.value as typeof expertiseLevel)}
              />
              <span className={styles.radioLabel}>{opt.label}</span>
              <span className={styles.radioDesc}>{opt.desc}</span>
            </label>
          ))}
        </div>
      </div>

      {/* Programming Languages */}
      <div className={styles.field}>
        <label className={styles.label}>Programming Languages</label>
        <div className={styles.chipGroup}>
          {availableLanguages.map(lang => (
            <button
              key={lang}
              type="button"
              className={`${styles.chip} ${programmingLanguages.includes(lang) ? styles.selected : ''}`}
              onClick={() => toggleLanguage(lang)}
            >
              {lang}
            </button>
          ))}
        </div>
      </div>

      {/* Learning Goals */}
      <div className={styles.field}>
        <label className={styles.label} htmlFor="learningGoals">
          Learning Goals (optional)
        </label>
        <textarea
          id="learningGoals"
          value={learningGoals}
          onChange={(e) => setLearningGoals(e.target.value)}
          placeholder="e.g., Build autonomous robots using ROS 2..."
          rows={3}
          className={styles.textarea}
        />
      </div>

      {/* Status Messages */}
      {error && <div className={styles.error}>{error}</div>}
      {success && <div className={styles.success}>{success}</div>}

      {/* Actions */}
      <div className={styles.actions}>
        {onCancel && (
          <button
            type="button"
            onClick={onCancel}
            className={styles.cancelButton}
            disabled={isLoading}
          >
            Cancel
          </button>
        )}
        <button
          type="submit"
          className={styles.submitButton}
          disabled={isLoading}
        >
          {isLoading ? 'Saving...' : 'Save Changes'}
        </button>
      </div>
    </form>
  );
}

export default ProfileEditForm;
