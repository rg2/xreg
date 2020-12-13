# MIT License
#
# Copyright (c) 2020 Robert Grupp
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import sys
import os
import os.path
import shutil
import glob
import subprocess as sp

kDEBUG = False

def dbg_print(s):
    if kDEBUG:
        print(s)

def get_lib_deps(exe_path):
    ldd_lines = sp.check_output('ldd {}'.format(exe_path), shell=True).decode().splitlines()
    
    deps = []

    for l in ldd_lines:
        toks = l.split('=>')
        if len(toks) > 1:
            if toks[1].strip() == 'not found':
                raise ValueError('dep. not found for: {}'.format(toks[0].strip()))

            lib_path = toks[1][:toks[1].find('(0x')].strip()

            if lib_path:
                deps.append(lib_path)

    return deps

def update_rpath(exe_path):
    sp.check_call('patchelf --force-rpath --set-rpath \'$ORIGIN/../lib\' {}'.format(exe_path), shell=True)

if __name__ == '__main__':
    xreg_install_prefix = sys.argv[1]
    xreg_dist_prefix    = sys.argv[2]

    dst_bin_path = '{}/bin'.format(xreg_dist_prefix)
    dst_lib_path = '{}/lib'.format(xreg_dist_prefix)

    os.mkdir(dst_bin_path)
    os.mkdir(dst_lib_path)
    
    ffmpeg_src_path = '{}/bin/ffmpeg'.format(xreg_install_prefix)

    if os.path.exists(ffmpeg_src_path):
        shutil.copy(ffmpeg_src_path, dst_bin_path)

    xreg_exes = glob.glob('{}/bin/xreg-*'.format(xreg_install_prefix))

    for exe in xreg_exes:
        dbg_print('copying exe: {}'.format(exe))
        shutil.copy(exe, dst_bin_path)
        
        dbg_print('  checking deps...')
        for dep_src_path in get_lib_deps(exe):
            lib_name = os.path.basename(dep_src_path)

            dbg_print('    {}'.format(lib_name))
            if not os.path.exists('{}/{}'.format(dst_lib_path, lib_name)):
                dbg_print('      copying...')
                shutil.copy(dep_src_path, dst_lib_path)

                update_rpath('{}/{}'.format(dst_lib_path, lib_name))
        
        dbg_print('  updating RPATH...')
        update_rpath('{}/{}'.format(dst_bin_path, os.path.basename(exe)))

