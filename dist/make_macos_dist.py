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
import queue
import subprocess as sp

def get_lib_deps(exe_path):
    # skip first line - just prints out the name of the exe
    otool_lines = sp.check_output('otool -L {}'.format(exe_path), shell=True).decode().splitlines()[1:]
    
    deps = []

    for l in otool_lines:
        l = l.strip()
        
        if l and (not l.startswith('/System')) and (not l.startswith('/usr/lib')):
            lib_path = l[:l.find('(')].strip()
            if lib_path:
                deps.append(lib_path)

    return deps

def get_lib_file_name(s):
    dyn_prefix = None
    new_s = s
    
    for p in ['@rpath/', '@executable_path/', '@load_path']:
        if s.startswith(p):
            new_s = s[len(p):]
            dyn_prefix = p
            break
    
    return (new_s, dyn_prefix)

def find_lib(search_roots, lib_name):
    # if the lib name is an absolute path then the file will exist:
    if os.path.exists(lib_name):
        return lib_name
    else:
        # otherwise try and find it
        for sr in search_roots:
            locs = sp.check_output('find {} -name "{}\"'.format(sr, lib_name), shell=True).decode().splitlines()
        
            if locs:
                return locs[0].strip()
    # if we have not returned yet, then we could not find the library!
    raise FileNotFoundError(lib_name)

def update_rpath(exe_path, prev_rpath):
    sp.check_call('install_name_tool -rpath {} @executable_path/../lib {}'.format(prev_rpath, exe_path), shell=True)

def update_lib_names_in_file(file_path):
    for cur_dep_name in get_lib_deps(file_path):
        # only change entries that are not using rpath
        if not cur_dep_name.startswith('@rpath/'):
            lib_name = cur_dep_name
            if os.path.exists(cur_dep_name):
                # name is an absolute path, get the library name
                lib_name = os.path.basename(cur_dep_name)

            # change the name to use rpath
            sp.check_call('install_name_tool -change {} @rpath/{} {}'.format(
                                cur_dep_name, lib_name, file_path), shell=True)

def update_dynlib_id(lib_path):
    sp.check_call('install_name_tool -id @rpath/{} {}'.format(os.path.basename(lib_path), lib_path), shell=True)

if __name__ == '__main__':
    if len(sys.argv) < 3:
        sys.stderr.write('Usage: {} <Cur. xReg Install> <Bundle (new) xReg Install> [<dylib search path 1> [... <dylib search path N> ]]\n'.format(os.path.basename(sys.argv[0])))
        sys.stderr.flush()
        sys.exit(1)

    xreg_install_prefix = sys.argv[1]
    xreg_dist_prefix    = sys.argv[2]

    lib_search_roots = sys.argv[3:]
    lib_search_roots.append(xreg_install_prefix)
    lib_search_roots = [os.path.abspath(os.path.realpath(p)) for p in lib_search_roots]

    # use the entries in DYLD_LIBRARY_PATH when available
    if 'DYLD_LIBRARY_PATH' in os.environ:
        for dyld_path in os.environ['DYLD_LIBRARY_PATH'].split(':'):
            if dyld_path:
                lib_search_roots.append(dyld_path)

    dst_bin_path = '{}/bin'.format(xreg_dist_prefix)
    dst_lib_path = '{}/lib'.format(xreg_dist_prefix)

    os.mkdir(dst_bin_path)
    os.mkdir(dst_lib_path)
    
    ffmpeg_src_path = '{}/bin/ffmpeg'.format(xreg_install_prefix)

    if os.path.exists(ffmpeg_src_path):
        shutil.copy(ffmpeg_src_path, dst_bin_path)

    xreg_exes = glob.glob('{}/bin/xreg-*'.format(xreg_install_prefix))

    files_to_check = queue.Queue()

    # copy the exes to the bundle location and initialize the queue to fixup
    # library names/paths with the exes
    for exe in xreg_exes:
        shutil.copy(exe, dst_bin_path)
        
        exe_new_path = '{}/{}'.format(dst_bin_path, os.path.basename(exe))

        update_rpath(exe_new_path, '{}/lib'.format(xreg_install_prefix))
        update_lib_names_in_file(exe_new_path)

        files_to_check.put(exe_new_path)

    all_lib_names = []

    while not files_to_check.empty():
        cur_item = files_to_check.get()
        
        print('finding dependencies of {}...'.format(cur_item))

        # for dependency of the current item
        for dep_src_path in get_lib_deps(cur_item):
            # extract the library filename and also any dynamic prefix
            (lib_file_name, dyn_prefix) = get_lib_file_name(dep_src_path)
            
            lib_src_path = None
            # if the lib name is an absolute path then the file will exist:
            if os.path.exists(lib_file_name):
                lib_src_path = lib_file_name
                lib_file_name = os.path.basename(lib_file_name)

            # this is where the library will be located in the distributed bundle
            lib_new_path = '{}/{}'.format(dst_lib_path, lib_file_name)

            # if the library has not been copied over yet, then copy it and then
            # add it to the queue, so we can check for its dependencies later
            if not os.path.exists(lib_new_path):
                if lib_src_path is None:
                    # Need to try and find the library
                    lib_src_path = find_lib(lib_search_roots, lib_file_name)

                shutil.copy(lib_src_path, lib_new_path)
                files_to_check.put(lib_new_path)
                
                # change the ID of this dynlib to be @rpath/filename
                update_dynlib_id(lib_new_path)

                # change all deps referenced in this library to be @rpath refs
                update_lib_names_in_file(lib_new_path)

                all_lib_names.append(lib_file_name)

