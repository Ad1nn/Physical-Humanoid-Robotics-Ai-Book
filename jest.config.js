module.exports = {
  testEnvironment: 'jsdom',
  roots: ['<rootDir>/frontend/tests/component'],
  moduleNameMapper: {
    '^@site/(.*)$': '<rootDir>/$1',
    '\\.css$': '<rootDir>/__mocks__/fileMock.js',
    '^@theme-original/(.*)$': '<rootDir>/__mocks__/fileMock.js',
    '^@docusaurus/(.*)$': '<rootDir>/__mocks__/fileMock.js',
  },
  setupFilesAfterEnv: ['<rootDir>/setupTests.js'],
};
