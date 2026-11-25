const path = require('node:path');
const web = path.join(__dirname, '..', '..', '..', 'web');
const esbuild = require(path.join(web, 'node_modules', 'esbuild'));

esbuild.build({
  entryPoints: [path.join(__dirname, 'model-view.js')],
  outfile: path.join(__dirname, 'model-view.bundle.js'),
  bundle: true,
  format: 'iife',
  globalName: 'LM3WebModel',
  minify: true,
  legalComments: 'inline',
  nodePaths: [path.join(web, 'node_modules')],
  define: {'import.meta.env.BASE_URL': JSON.stringify('./')},
  loader: {'.glb': 'binary'},
}).catch(() => { process.exitCode = 1; });
