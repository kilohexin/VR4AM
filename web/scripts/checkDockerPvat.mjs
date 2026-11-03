import {build} from 'esbuild';
import {resolve} from 'node:path';
const result=await build({entryPoints:[resolve('scripts/checkDockerPvat.ts')],bundle:true,platform:'node',format:'esm',write:false});
try{await import('data:text/javascript;base64,'+Buffer.from(result.outputFiles[0].text).toString('base64'));}
catch(error){console.error(error.message);process.exitCode=1;}
