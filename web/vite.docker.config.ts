import basicSsl from '@vitejs/plugin-basic-ssl';
import {defineConfig} from 'vite';

// Serve the same built app and WebSocket as the desktop Docker endpoint.
// No environment override: this entry cannot proxy the real-arm port 8769.
export default defineConfig({
  plugins:[basicSsl()],
  server:{
    host:'0.0.0.0',port:5173,strictPort:true,https:{},
    proxy:{'/':{target:'http://127.0.0.1:8773',ws:true}},
  },
});
