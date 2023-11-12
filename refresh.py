#!/usr/bin/env python
import os
os.system('rm -rf ptx_files/all_white.ptx')
os.system('git checkout ptx_files/*')
os.system('rm -rf ptx_files/mask*')
os.system('cp ptx_files/all_black.ptx ptx_files/all_white.ptx')
os.system('make all')
# os.system('./build/Linux-4.4.0-x86_64-optimize/src/tests/wtest_plane')
os.system('./build/Linux-4.4.0-x86_64-optimize/src/tests/ptxtransfer  ./ptx_files/all_black.ptx ./ptx_files/all_white.ptx')