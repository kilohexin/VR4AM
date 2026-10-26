import {afterEach,it,expect,vi} from 'vitest';
import config from '../vite.config';
afterEach(()=>vi.unstubAllEnvs());
function resolve(){return (config as Function)({mode:'development',command:'serve'});}
it('keeps the legacy backend by default and explicitly proxies finite control through HTTPS',()=>{
 vi.stubEnv('VITE_TELEOP_PROXY_PORT','');expect(resolve().server.proxy['/ws'].target).toBe('ws://127.0.0.1:8000');
 vi.stubEnv('VITE_TELEOP_PROXY_PORT','8769');expect(resolve().server.proxy['/ws'].target).toBe('ws://127.0.0.1:8769');
});
it('rejects arbitrary hosts or unexpected proxy ports',()=>{
 vi.stubEnv('VITE_TELEOP_PROXY_PORT','172.16.2.162:80');expect(resolve).toThrow('unsupported_teleop_proxy_port');
});
