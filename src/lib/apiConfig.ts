/**
 * Safe API configuration for Docusaurus frontend.
 * Guards against `ReferenceError: process is not defined` in browser runtimes.
 */
function getApiBaseUrl(): string {
  try {
    if (typeof process !== 'undefined' && process?.env?.DOCUSAURUS_BACKEND_URL) {
      return process.env.DOCUSAURUS_BACKEND_URL;
    }
  } catch {
    // In browser, process is not defined
  }
  return 'https://backend-book-vtha.onrender.com';
}

export const API_BASE_URL: string = getApiBaseUrl();
