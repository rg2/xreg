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

# Functional testing by running the example commands from the wiki. Success or
# failure is determined by manual inspection of the output results/data.

import sys
import os.path
import os
import platform
import urllib.request
import shutil
import time
import zipfile

import subprocess as sp

def download_file(url, dst_file_name=None, use_existing=False):
    print('Downloading: {} ...'.format(url))

    web_obj = urllib.request.urlopen(url)

    if dst_file_name is not None:
        dst_file_path = dst_file_name
    else:
        dst_file_path = os.path.basename(url)
    
    if use_existing and os.path.exists(dst_file_path):
        print('  Using already downloaded file: {}'.format(dst_file_path))
    else:
        with open(dst_file_path, 'wb') as file_out:
            shutil.copyfileobj(web_obj, file_out)
            file_out.flush()
        
        print('  Saved to: {}'.format(dst_file_path))

def extract_zip(zip_path, dst_dir_path=None):
    print('Extracting contents of {}...'.format(zip_path))

    if dst_dir_path and not os.path.exists(dst_dir_path):
        os.makedirs(dst_dir_path)

    with zipfile.ZipFile(zip_path, 'r') as zf:
        zf.extractall(path=dst_dir_path)

def run_cmd(cmd_str):
    print('Running command: {}'.format(cmd_str))
    
    sp.check_call(cmd_str, shell=True)

def wait_prompt(msg_str):
    input('{}. Press the <enter> or <return> key to continue...'.format(msg_str))

def macos_check_for_dyld_vars():
    if platform.system() == 'Darwin':
        # On MacOS variables starting with DYLD_ (like DYLD_LIBRARY_PATH) are removed before spawning
        # subprocesses. If a user relies on these variables to properly load the libraries needed by
        # xReg, then the subproccess calls will fail. This section attempts to warn the user of this.
        # This can be avoided by using xReg executables with properly encoded library paths, such as
        # those created by dist/make_macos_dist.py, or by statically linking everything.

        first_msg_printed = False

        for k in os.environ:
            if k.startswith('DYLD_'):
                if not first_msg_printed:
                    print('WARNING: YOU HAVE DYLD_* VARIABLES SET. IF THESE ARE NEEDED TO RUN XREG, '
                          'THEY WILL NOT BE PROPAGATED TO THE SUBPROCCESSES AND THE CALLS WILL FAIL '
                          'AS A RESULT OF APPLE SYSTEM INTEGRITY PROTECTION. TRY BUILDING A '
                          'DISTRIBUTABLE VERSION OF XREG (SEE dist/make_macos_dist.py) AND PASS THAT '
                          'BIN DIRECTORY TO THIS SCRIPT.')
                    first_msg_printed = True
                print('-->  {}'.format(k))

        if first_msg_printed:
            # let the warning message stay visible for a few seconds
            time.sleep(5)

if __name__ == '__main__':
    if ('--help' in sys.argv) or ('-h' in sys.argv):
        print('TODO: help message')
        sys.exit(0)
    elif len(sys.argv) > 1:
        xreg_bin_dir = sys.argv[1]
        print('using custom xReg bin directory: {}'.format(xreg_bin_dir))
        
        if not os.path.isdir(xreg_bin_dir):
            print('ERROR: passed bin directory is not a directory!')
            sys.exit(1)

        print('updating PATH...')
        os.environ['PATH'] = '{}:{}'.format(xreg_bin_dir, os.environ['PATH'])
    else:
        print('using existing PATH to resolve xReg executables...')

    macos_check_for_dyld_vars()

    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-DICOM-Conversion
    if False:
        print('DICOM conversion/resampling...')
        
        download_file('https://services.cancerimagingarchive.net/services/v4/TCIA/query/getImage?SeriesInstanceUID=61.7.167248355135476067044532759811631626828', 'ABD_LYMPH_001.zip', True)
       
        extract_zip('ABD_LYMPH_001.zip', 'ABD_LYMPH_001')

        run_cmd('xreg-convert-dicom ABD_LYMPH_001 lymph.nii.gz --one')
       
        wait_prompt('Check lymph.nii.gz in a visualization tool such as 3D Slicer')

    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Volume-Cropping
    if False:
        print('Volume cropping...')
        download_file('https://github.com/rg2/xreg/wiki/walkthrough/volume_cropping/R.acsv')

        run_cmd('xreg-crop-vol lymph.nii.gz R.acsv pelvis.nii.gz')

        wait_prompt('Check pelvis.nii.gz in a visualization tool such as 3D Slicer')
    
    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Mesh-Creation
    if False:
        print('Mesh creation using intensity threshold...')

        run_cmd('xreg-create-mesh -i --lower 150 pelvis.nii.gz pelvis_thresh_sur.ply')
        
        run_cmd('xreg-show-mesh pelvis_thresh_sur.ply')

    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Mesh-Creation
    if False:
        print('Mesh creation using segmentation...')

        download_file('https://raw.githubusercontent.com/wiki/rg2/xreg/walkthrough/bone_segmentation/pelvis_seg.nii.gz')

        run_cmd('xreg-create-mesh pelvis_seg.nii.gz pelvis_sur.ply 1')

        run_cmd('xreg-show-mesh pelvis_sur.ply')

    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-PAO-Planning
    if False:
        print('PAO fragment creation...')

        download_file('https://github.com/rg2/xreg/wiki/walkthrough/pao_planning/pelvis_app_lands.fcsv')
        download_file('https://github.com/rg2/xreg/wiki/walkthrough/pao_planning/pao_cut_lands.fcsv')

        run_cmd('xreg-pao-create-frag pelvis_seg.nii.gz pelvis_app_lands.fcsv pao_cut_lands.fcsv left - pao_cuts_seg.nii.gz pao_cut_defs.h5')

        run_cmd('xreg-pao-draw-bones pao_cuts_seg.nii.gz pelvis_app_lands.fcsv left --cut-defs pao_cut_defs.h5')

    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-PAO-Planning
    if False:
        print('PAO random fragment creation...')

        run_cmd('xreg-pao-create-frag pelvis_seg.nii.gz pelvis_app_lands.fcsv pao_cut_defs.h5 left - pao_cuts_seg_batch pao_cut_defs_batch --batch 5 --add-noise-planes --use-normal-dist-plane-noise')
        
        run_cmd('xreg-pao-draw-bones pao_cuts_seg_batch_000.nii.gz pelvis_app_lands.fcsv left --cut-defs pao_cut_defs_batch_000.h5')
    
    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Simulated-PAO-Adjustments
    if False:
        print('Simulated PAO adjustments/movements...')

        run_cmd('xreg-pao-sample-frag-moves pao_cuts_seg.nii.gz pelvis_app_lands.fcsv left 10 test_pao --frag-rot-mean-x 10 --frag-rot-std-x 10 --frag-rot-mean-y 3 --frag-rot-std-y 3 --frag-rot-mean-z 12.5 --frag-rot-std-z 12.5 --trans-mean-x 2.5 --trans-std-x 5 --trans-mean-y -2 --trans-std-y 2 --trans-mean-z 2 --trans-std-z 3 --femur-rot-mean-x 10 --femur-rot-std-x 10 --femur-rot-mean-y 0 --femur-rot-std-y 5 --femur-rot-mean-z 0 --femur-rot-std-z 2 --morph-open-size 5 --uniform-sampling')

        run_cmd('xreg-pao-draw-bones pao_cuts_seg.nii.gz pelvis_app_lands.fcsv left --femur-frag-xform test_pao_frag_0.h5 --femur-only-xform test_pao_femur_0.h5')

    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Volumetric-Modeling-of-PAO-Adjustments
    if False:
        print('PAO volume modeling of adjusted fragment...')

        run_cmd('xreg-pao-create-repo-vol pelvis.nii.gz pao_cuts_seg.nii.gz pelvis_app_lands.fcsv left test_pao_frag_0.h5 test_pao_femur_0.h5 pao_vol.nii.gz')
        
        wait_prompt('Check pao_vol.nii.gz in a visualization tool such as 3D Slicer')

    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Volumetric-Modeling-of-Screws-and-Wires-for-PAO
    if False:
        print('Volume modeling of K-wires...')

        download_file('https://raw.githubusercontent.com/wiki/rg2/xreg/walkthrough/volumetric_modeling_screws_kwire/pelvis_left_insertion_labels.nii.gz')

        run_cmd('xreg-pao-add-screw-kwires-to-vol --super-sample 2 pao_vol.nii.gz pao_cuts_seg.nii.gz pelvis_app_lands.fcsv left test_pao_frag_0.h5 pelvis_left_insertion_labels.nii.gz pao_vol_with_kwire.nii.gz --p-two 1 --p-wire 1')

        wait_prompt('Check pao_vol_with_kwire.nii.gz in a visualization tool such as 3D Slicer')
    
    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Volumetric-Modeling-of-Screws-and-Wires-for-PAO
    if False:
        print('Volume modeling of screws...')

        run_cmd('xreg-pao-add-screw-kwires-to-vol --super-sample 2 pao_vol.nii.gz pao_cuts_seg.nii.gz pelvis_app_lands.fcsv left test_pao_frag_0.h5 pelvis_left_insertion_labels.nii.gz pao_vol_with_screws.nii.gz --p-two 0 --p-wire 0')

        wait_prompt('Check pao_vol_with_screws.nii.gz in a visualization tool such as 3D Slicer')

    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Simulated-Fluoroscopy
    if False:
        print('Simulated fluoroscopy...')

        run_cmd('xreg-pao-create-synth-fluoro pao_vol_with_kwire.nii.gz pelvis_app_lands.fcsv left 5 example1_1_pd example1_1_pose')

        run_cmd('xreg-draw-xray-scene -i example1_1_pd_003.h5')

        run_cmd('xreg-create-mesh -i --lower 300 pao_vol_with_kwire.nii.gz pao_w_kwire_mesh.h5')

        run_cmd('xreg-draw-xray-scene -i example1_1_pd_003.h5 pao_w_kwire_mesh.h5 example1_1_pose_003.h5')

        run_cmd('xreg-remap-tile-proj-data example1_1_pd_003.h5 example1_1_pd_003.png -d 0.25 -b 1')
        
        wait_prompt('Check example1_1_pd_003.png in an image viewer')
    
