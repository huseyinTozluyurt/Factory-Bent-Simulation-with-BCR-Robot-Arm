import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// The site reuses the project's own docs folder (docs/images and docs/reports),
// so screenshots and PDF reports are never copied twice.
export default defineConfig({
  plugins: [react()],
  base: './',
  publicDir: '../docs',
  server: { port: 5173, open: true },
})
