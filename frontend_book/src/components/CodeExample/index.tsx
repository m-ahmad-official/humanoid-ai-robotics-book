import React from 'react';
import clsx from 'clsx';
import styles from './CodeExample.module.css';

type CodeExampleProps = {
  title: string;
  code: string;
  language?: string;
  description?: string;
};

export default function CodeExample({
  title,
  code,
  language = 'python',
  description,
}: CodeExampleProps): JSX.Element {
  return (
    <div className={styles.codeExample}>
      <h4>{title}</h4>
      {description && <p className={styles.description}>{description}</p>}
      <div className={styles.codeBlock}>
        <pre>
          <code className={`language-${language}`}>
            {code}
          </code>
        </pre>
      </div>
    </div>
  );
}