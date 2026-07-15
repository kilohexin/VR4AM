import basicSsl from '@vitejs/plugin-basic-ssl';
import {defineConfig} from 'vite';

export default defineConfig({
  plugins: [basicSsl()],
  server: {
    https: {},
    proxy: {
      '/ws': {
        target: 'ws://127.0.0.1:8000',
        ws: true,
      },
    },
  },
});
