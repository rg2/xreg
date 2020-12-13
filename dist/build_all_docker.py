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

import os.path
import sys

import subprocess as sp

def build_dev_base(os_name, os_version, xreg_git_dir):
    sp.check_call('docker build -t xreg-dev-base-{0}-{1} --build-arg os_name={0} --build-arg os_version={1} -f {2}/docker/Dockerfile.{0}_dev_base {2}'.format(os_name, os_version, xreg_git_dir), shell=True)

def build_deps(os_name, os_version, xreg_git_dir):
    sp.check_call('docker build -t xreg-deps-{0}-{1} --build-arg os_name={0} --build-arg os_version={1} -f {2}/docker/Dockerfile.xreg-deps {2}'.format(os_name, os_version, xreg_git_dir), shell=True)

def build_xreg(os_name, os_version, xreg_git_dir):
    sp.check_call('docker build -t xreg-{0}-{1} --build-arg os_name={0} --build-arg os_version={1} -f {2}/docker/Dockerfile.xreg {2}'.format(os_name, os_version, xreg_git_dir), shell=True)

def build_xreg_dist(os_name, os_version, xreg_git_dir, xreg_dist_str):
    dist_img_name = 'xreg-dist-{0}-{1}'.format(os_name, os_version)

    sp.check_call('docker build -t {4} --build-arg os_name={0} --build-arg os_version={1} --build-arg xreg_dist_str={3} -f {2}/docker/Dockerfile.xreg-dist-bin {2}'.format(os_name, os_version, xreg_git_dir, xreg_dist_str, dist_img_name), shell=True)

    cont_id = sp.check_output('docker create {}'.format(dist_img_name), shell=True).decode().strip()
    
    dist_file_name = '{}-{}-{}.tar.gz'.format(xreg_dist_str, os_name, os_version)

    sp.check_output('docker cp {}:/{} .'.format(cont_id, dist_file_name), shell=True)
    
    sp.check_call('docker rm {}'.format(cont_id), shell=True)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.stderr.write('Usage: {} <xreg src dir> [<xreg ver string>]\n'.format(os.path.basename(sys.argv[0])))
        sys.stderr.flush()
        sys.exit(1)
    
    xreg_src_dir = sys.argv[1]

    xreg_dist_prefix = 'xreg'
    
    if len(sys.argv) > 2:
        xreg_dist_prefix = '{}-{}'.format(xreg_dist_prefix, sys.argv[2])

    # os name --> list of os version
    configs_to_build = { 'ubuntu' : [ '16.04', '18.04', '20.04' ], 'centos' : [ '7' ] }
    
    for os_name, os_versions in configs_to_build.items():
        for os_ver in os_versions:
            build_dev_base(os_name, os_ver, xreg_src_dir)
            build_deps(os_name, os_ver, xreg_src_dir)
            build_xreg(os_name, os_ver, xreg_src_dir)

            build_xreg_dist(os_name, os_ver, xreg_src_dir, xreg_dist_prefix)


