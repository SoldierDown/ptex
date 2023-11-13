#!/usr/bin/env python
import os
input = 'ab.ptx'
output = 'mixt.ptx'
if os.path.exists('ptx_out/{}'.format(output)):
    os.system('rm -rf ptx_out/{}'.format(output))
os.system('cp ptx_in/{} ptx_out/{}'.format(input, output))
os.system('make all')
os.system('./build/Linux-4.4.0-x86_64-optimize/src/tests/ptxtransfer  ./ptx_in/{} ./ptx_out/{}'.format(input, output))