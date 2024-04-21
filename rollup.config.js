import { nodeResolve } from '@rollup/plugin-node-resolve';

export default {
   input: 'build/libserial/avrdude-worker.js',
   output: {
     dir: '.',
     format: 'cjs'
   },
   plugins: [nodeResolve()]
};
