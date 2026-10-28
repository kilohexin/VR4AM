import {build} from 'esbuild';
import {mkdtemp,rm,rmdir} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import {join} from 'node:path';
import {fileURLToPath} from 'node:url';
import {spawn} from 'node:child_process';

// Compile the current product input modules, not a historical bundled driver.
const directory=await mkdtemp(join(tmpdir(),'vr4arm-docker-xr-'));
const outfile=join(directory,'check.mjs');
try{
  await build({entryPoints:[fileURLToPath(new URL('./checkDockerXR.ts',import.meta.url))],
    bundle:true,platform:'node',format:'esm',outfile});
  const code=await new Promise((resolve,reject)=>{
    const child=spawn(process.execPath,[outfile,...process.argv.slice(2)],{stdio:'inherit'});
    child.on('error',reject);child.on('exit',code=>resolve(code??1));
  });
  process.exitCode=code;
}finally{await rm(outfile,{force:true});await rmdir(directory);}
