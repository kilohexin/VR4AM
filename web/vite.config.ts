import basicSsl from '@vitejs/plugin-basic-ssl';
import {defineConfig,loadEnv} from 'vite';

export default defineConfig(({mode})=>{
 const port=loadEnv(mode,'.','VITE_').VITE_TELEOP_PROXY_PORT||'8000';
 if(!['8000','8769'].includes(port))throw new Error('unsupported_teleop_proxy_port');
 return {
  plugins: [basicSsl()],
  server: {
    https: {},
    fs: {allow: ['..']},
    proxy: {
      '/ws': {
        target: `ws://127.0.0.1:${port}`,
        ws: true,
      },
    },
  },
 };});
