import basicSsl from '@vitejs/plugin-basic-ssl';
import {defineConfig} from 'vite';

// Separate headset entry for the bounded, independently admitted Docker test.
// No environment override and no proxy to the real-arm or old experience port.
export default defineConfig({
  plugins:[basicSsl()],
  server:{
    host:'0.0.0.0',port:8444,strictPort:true,https:{},
    proxy:{'/':{target:'http://127.0.0.1:8774',ws:true}},
  },
});
