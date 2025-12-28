import React from 'react';
import clsx from 'clsx';
import styles from './styles.module.css';
import { FaLightbulb, FaPencilAlt, FaInfoCircle, FaExclamationTriangle, FaExclamationCircle } from 'react-icons/fa';

const iconMapping = {
  tip: FaLightbulb,
  note: FaPencilAlt,
  info: FaInfoCircle,
  warning: FaExclamationTriangle,
  danger: FaExclamationCircle,
};

const Admonition = ({ type, title, children }) => {
  const Icon = iconMapping[type];
  return (
    <div
      className={clsx('admonition', styles.admonition, {
        [styles.admonitionTip]: type === 'tip',
        [styles.admonitionNote]: type === 'note',
        [styles.admonitionInfo]: type === 'info',
        [styles.admonitionWarning]: type === 'warning',
        [styles.admonitionDanger]: type === 'danger',
      })}
    >
      <div className={styles.admonitionHeading}>
        {Icon && <Icon className={styles.admonitionIcon} />}
        {title && <div className={styles.admonitionTitle}>{title}</div>}
      </div>
      <div className={styles.admonitionContent}>{children}</div>
    </div>
  );
};

export default Admonition;
