# MIT License
#
# Copyright (c) 2020-2022 Robert Grupp
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
#
# NOTE: As this script will launch GUI tools to help determine test results,
# the script must be run in an environment with a sufficient display. Some X11
# forwarding configurations may NOT work.

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

def has_xdg_open():
    try:
        return bool(sp.check_output('which xdg-open', shell=True).decode().strip())
    except:
        return False

def view_image(img_path):
    if platform.system() == 'Darwin':
        sp.check_call('open {}'.format(img_path), shell=True)
    elif platform.system() == 'Windows':
        sp.check_call(img_path, shell=True)
    elif (platform.system() == 'Linux') and has_xdg_open():
        sp.check_call('xdg-open {}'.format(img_path), shell=True)
    
    wait_prompt('Check {} in an image viewer'.format(img_path))

def view_movie(movie_path):
    if platform.system() == 'Darwin':
        sp.check_call('open {}'.format(movie_path), shell=True)
    elif platform.system() == 'Windows':
        sp.check_call(movie_path, shell=True)
    elif (platform.system() == 'Linux') and has_xdg_open():
        sp.check_call('xdg-open {}'.format(movie_path), shell=True)

    wait_prompt('Inspect {} in a video player'.format(movie_path))

def view_vol(vol_path, slicer_path):
    if slicer_path:
        sp.check_call('{} --python-code \"slicer.util.loadVolume(\'{}\')\"'.format(slicer_path, vol_path), shell=True)
    else:
        wait_prompt('Check {} in a visualization tool such as 3D Slicer'.format(vol_path))

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
    mac_slicer_default_paths = ['/Applications/Slicer.app/Contents/MacOS/Slicer']
    win_slicer_default_paths = ['%LOCALAPPDATA%\\NA-MIC\\{}\\Slicer.exe'.format(sn) for sn in \
            ['Slicer 5.0.3', 'Slicer 5.0.2', 'Slicer 4.11.20210226']] \
            + ['C:\\Program Files\\Slicer 4.10.2\\Slicer.exe']
    linux_slicer_default_paths = ['{}/Slicer'.format(sd) for sd in \
               ['$HOME/Slicer-5.0.3-linux-amd64',
                '$HOME/Slicer-5.0.2-linux-amd64',
                '$HOME/Slicer-4.11.20210226-linux-amd64',
                '$HOME/Slicer-4.10.2-linux-amd64']]
    
    if ('--help' in sys.argv) or ('-h' in sys.argv):
        print('This script exists for semi-automatic functional testing by executing the commands '
              'found on the xReg wiki walkthrough and having the user manually verify the outputs.\n'
              'Usage: python {} [<xReg bin directory>]\n'
              'xReg binaries will be found via the PATH environment variable when an xReg bin '
              'directory is not provided.\n'
              'When available on the system, 3D Slicer is automatically invoked to visualize volumes.'
              'Depending on the operating system, 3D Slicer is assumed to reside in one of the '
              'following locations:\n'
              '  *   MacOS: {}\n'
              '  * Windows: {}\n'
              '  *   Linux: {}'.format(
                  os.path.basename(sys.argv[0]),
                  ' , '.join(mac_slicer_default_paths),
                  ' , '.join(win_slicer_default_paths),
                  ' , '.join(linux_slicer_default_paths)))
        sys.exit(0)
    elif len(sys.argv) > 1:
        xreg_bin_dir = sys.argv[1]
        print('using custom xReg bin directory: {}'.format(xreg_bin_dir))
        
        if not os.path.isdir(xreg_bin_dir):
            print('ERROR: passed bin directory is not a directory!')
            sys.exit(1)

        print('updating PATH...')
        path_list_sep = ';' if platform.system() == 'Windows' else ':'
        os.environ['PATH'] = '{}{}{}'.format(xreg_bin_dir, path_list_sep, os.environ['PATH'])
    else:
        print('using existing PATH to resolve xReg executables...')

    macos_check_for_dyld_vars()

    # Try and find 3D Slicer in order to display volume files
    slicer_path = None

    if platform.system() == 'Darwin':
        for p in mac_slicer_default_paths:
            if os.path.exists(p):
                slicer_path = p
                break
    elif platform.system() == 'Windows':
        for p in win_slicer_default_paths:
            pp = os.path.expandvars(p)
            if os.path.exists(pp):
                # enclose the path in quotes to handle any spaces
                slicer_path = '\"{}\"'.format(pp)
                break
    elif platform.system() == 'Linux':
        for p in linux_slicer_default_paths:
            linux_slicer_default_path = os.path.expandvars(p)
        
            if os.path.exists(linux_slicer_default_path):
                slicer_path = linux_slicer_default_path
                break

    if slicer_path:
        print('Found Slicer at: {} -- will use for volume visualization!'.format(slicer_path))
    else:
        print('Slicer not found! Useer must manually open a tool for volume visualization!')

    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-DICOM-Conversion
    if True:
        print('DICOM conversion/resampling...')
        
        download_file('https://services.cancerimagingarchive.net/services/v4/TCIA/query/getImage?SeriesInstanceUID=61.7.167248355135476067044532759811631626828', 'ABD_LYMPH_001.zip', True)
       
        extract_zip('ABD_LYMPH_001.zip', 'ABD_LYMPH_001')

        run_cmd('xreg-convert-dicom-vols ABD_LYMPH_001 lymph.nii.gz --one')
       
        view_vol('lymph.nii.gz', slicer_path)

    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Volume-Cropping
    if True:
        print('Volume cropping...')
        download_file('https://github.com/rg2/xreg/wiki/walkthrough/volume_cropping/R.acsv')

        run_cmd('xreg-crop-vol lymph.nii.gz R.acsv pelvis.nii.gz')

        view_vol('pelvis.nii.gz', slicer_path)
    
    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Mesh-Creation
    if True:
        print('Mesh creation using intensity threshold...')

        run_cmd('xreg-create-mesh -i --lower 150 pelvis.nii.gz pelvis_thresh_sur.ply')
        
        run_cmd('xreg-show-mesh pelvis_thresh_sur.ply')

    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Mesh-Creation
    if True:
        print('Mesh creation using segmentation...')

        download_file('https://raw.githubusercontent.com/wiki/rg2/xreg/walkthrough/bone_segmentation/pelvis_seg.nii.gz')

        run_cmd('xreg-create-mesh pelvis_seg.nii.gz pelvis_sur.ply 1')

        run_cmd('xreg-show-mesh pelvis_sur.ply')

    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-PAO-Planning
    if True:
        print('PAO fragment creation...')

        download_file('https://github.com/rg2/xreg/wiki/walkthrough/pao_planning/pelvis_app_lands.fcsv')
        download_file('https://github.com/rg2/xreg/wiki/walkthrough/pao_planning/pao_cut_lands.fcsv')

        run_cmd('xreg-pao-create-frag pelvis_seg.nii.gz pelvis_app_lands.fcsv pao_cut_lands.fcsv left - pao_cuts_seg.nii.gz pao_cut_defs.h5')

        run_cmd('xreg-pao-draw-bones pao_cuts_seg.nii.gz pelvis_app_lands.fcsv left --cut-defs pao_cut_defs.h5')

    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-PAO-Planning
    if True:
        print('PAO random fragment creation...')

        run_cmd('xreg-pao-create-frag pelvis_seg.nii.gz pelvis_app_lands.fcsv pao_cut_defs.h5 left - pao_cuts_seg_batch pao_cut_defs_batch --batch 5 --add-noise-planes --use-normal-dist-plane-noise')
        
        run_cmd('xreg-pao-draw-bones pao_cuts_seg_batch_000.nii.gz pelvis_app_lands.fcsv left --cut-defs pao_cut_defs_batch_000.h5')
    
    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Simulated-PAO-Adjustments
    if True:
        print('Simulated PAO adjustments/movements...')

        run_cmd('xreg-pao-sample-frag-moves pao_cuts_seg.nii.gz pelvis_app_lands.fcsv left 10 test_pao --frag-rot-mean-x 10 --frag-rot-std-x 10 --frag-rot-mean-y 3 --frag-rot-std-y 3 --frag-rot-mean-z 12.5 --frag-rot-std-z 12.5 --trans-mean-x 2.5 --trans-std-x 5 --trans-mean-y -2 --trans-std-y 2 --trans-mean-z 2 --trans-std-z 3 --femur-rot-mean-x 10 --femur-rot-std-x 10 --femur-rot-mean-y 0 --femur-rot-std-y 5 --femur-rot-mean-z 0 --femur-rot-std-z 2 --morph-open-size 5 --uniform-sampling')

        run_cmd('xreg-pao-draw-bones pao_cuts_seg.nii.gz pelvis_app_lands.fcsv left --femur-frag-xform test_pao_frag_0.h5 --femur-only-xform test_pao_femur_0.h5')

    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Volumetric-Modeling-of-PAO-Adjustments
    if True:
        print('PAO volume modeling of adjusted fragment...')

        run_cmd('xreg-pao-create-repo-vol pelvis.nii.gz pao_cuts_seg.nii.gz pelvis_app_lands.fcsv left test_pao_frag_0.h5 test_pao_femur_0.h5 pao_vol.nii.gz')
        
        view_vol('pao_vol.nii.gz', slicer_path)

    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Volumetric-Modeling-of-Screws-and-Wires-for-PAO
    if True:
        print('Volume modeling of K-wires...')

        download_file('https://raw.githubusercontent.com/wiki/rg2/xreg/walkthrough/volumetric_modeling_screws_kwire/pelvis_left_insertion_labels.nii.gz')

        run_cmd('xreg-pao-add-screw-kwires-to-vol --super-sample 2 pao_vol.nii.gz pao_cuts_seg.nii.gz pelvis_app_lands.fcsv left test_pao_frag_0.h5 pelvis_left_insertion_labels.nii.gz pao_vol_with_kwire.nii.gz --p-two 1 --p-wire 1')

        view_vol('pao_vol_with_kwire.nii.gz', slicer_path)
    
    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Volumetric-Modeling-of-Screws-and-Wires-for-PAO
    if True:
        print('Volume modeling of screws...')

        run_cmd('xreg-pao-add-screw-kwires-to-vol --super-sample 2 pao_vol.nii.gz pao_cuts_seg.nii.gz pelvis_app_lands.fcsv left test_pao_frag_0.h5 pelvis_left_insertion_labels.nii.gz pao_vol_with_screws.nii.gz --p-two 0 --p-wire 0')

        view_vol('pao_vol_with_screws.nii.gz', slicer_path)

    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Simulated-Fluoroscopy
    if True:
        print('Simulated fluoroscopy...')

        run_cmd('xreg-pao-create-synth-fluoro pao_vol_with_kwire.nii.gz pelvis_app_lands.fcsv left 5 example1_1_pd example1_1_pose')

        run_cmd('xreg-draw-xray-scene -i example1_1_pd_003.h5')

        run_cmd('xreg-create-mesh -i --lower 300 pao_vol_with_kwire.nii.gz pao_w_kwire_mesh.h5')

        run_cmd('xreg-draw-xray-scene -i example1_1_pd_003.h5 pao_w_kwire_mesh.h5 example1_1_pose_003.h5')

        run_cmd('xreg-remap-tile-proj-data example1_1_pd_003.h5 example1_1_pd_003.png -d 0.25 -b 1')
       
        view_image('example1_1_pd_003.png')
   
    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Point-Cloud-to-Surface-Registration
    if True:
        print('3D/3D ICP registration...')

        download_file('https://github.com/rg2/xreg/wiki/walkthrough/point_cloud_to_surface_regi/pelvis_preop_lands.fcsv')
        download_file('https://github.com/rg2/xreg/wiki/walkthrough/point_cloud_to_surface_regi/pelvis_intraop_lands.fcsv')
        download_file('https://github.com/rg2/xreg/wiki/walkthrough/point_cloud_to_surface_regi/pelvis_intraop_point_cloud.fcsv')
        
        wait_prompt('The next visualization will show the pelvis and intraop. point cloud BEFORE registration.')
        run_cmd('xreg-pao-draw-bones --no-frag --no-femurs --pelvis-mesh pelvis_sur.ply pelvis_seg.nii.gz pelvis_app_lands.fcsv left --other-pts pelvis_intraop_point_cloud.fcsv')

        run_cmd('xreg-sur-regi pelvis_sur.ply pelvis_intraop_point_cloud.fcsv pelvis_regi.h5 --mesh-lands pelvis_preop_lands.fcsv --pts-lands pelvis_intraop_lands.fcsv')
        
        run_cmd('xreg-xform-fcsv pelvis_intraop_point_cloud.fcsv pelvis_regi.h5 pelvis_intraop_point_cloud_regi.fcsv')
        
        wait_prompt('The next visualization will show the pelvis and intraop. point cloud AFTER registration.')
        run_cmd('xreg-pao-draw-bones --no-frag --no-femurs --pelvis-mesh pelvis_sur.ply pelvis_seg.nii.gz pelvis_app_lands.fcsv left --other-pts pelvis_intraop_point_cloud_regi.fcsv')
    
    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Single-View-Pelvis-Registration
    if True:
        print('2D/3D registration - single object, single view...')
  
        download_file('https://raw.githubusercontent.com/wiki/rg2/xreg/walkthrough/simulated_fluoroscopy/example1_1_pd_003.h5', 'example1_1_pd_003_for_regi.h5')
        download_file('https://github.com/rg2/xreg/wiki/walkthrough/single_view_pelvis_regi/pelvis_regi_2d_3d_lands.fcsv')
        download_file('https://github.com/rg2/xreg/wiki/walkthrough/single_view_pelvis_regi/example1_1_pd_003_proj_0_lands.fcsv')

        run_cmd('xreg-proj-data-extract-nii example1_1_pd_003_for_regi.h5 example1_1_pd_003_proj 0')
       
        run_cmd('xreg-add-lands-to-proj-data example1_1_pd_003_for_regi.h5 example1_1_pd_003_proj_0_lands.fcsv')

        run_cmd('xreg-remap-tile-proj-data example1_1_pd_003_for_regi.h5 -o -p 0 -d 0.25 example1_1_pd_003_proj_0_w_lands.png')
        
        view_image('example1_1_pd_003_proj_0_w_lands.png')

        run_cmd('xreg-hip-surg-pelvis-single-view-regi-2d-3d pelvis.nii.gz pelvis_regi_2d_3d_lands.fcsv example1_1_pd_003_for_regi.h5 regi_pose_example1_1_pd_003_proj0.h5 regi_debug_example1_1_pd_003_proj0_w_seg.h5 -s pelvis_seg.nii.gz')
        
        run_cmd('xreg-draw-xray-scene example1_1_pd_003_for_regi.h5 -p 0 pelvis_sur.ply regi_pose_example1_1_pd_003_proj0.h5 -i --bg-color 1 1 1 --mesh-color-bone')

        run_cmd('xreg-regi2d3d-replay regi_debug_example1_1_pd_003_proj0_w_seg.h5 --video-fps 10 --proj-ds 0.5')

        view_movie('edges.mp4')
        view_movie('mov.mp4')

    #################################################################
    # https://github.com/rg2/xreg/wiki/Walkthrough%3A-Multiple-View-PAO-Fragment-Registration
    if True:
        print('2D/3D registration - multi. objects, multi. views...')

        run_cmd('xreg-pao-frag-mult-view-regi-2d-3d pelvis.nii.gz pao_cuts_seg.nii.gz left pelvis_app_lands.fcsv pelvis_regi_2d_3d_lands.fcsv example1_1_pd_003_for_regi.h5 regi_pose_pelvis.h5 regi_pose_femur.h5 regi_pose_frag.h5 rel_pose_femur.h5 rel_pose_frag.h5 multi_obj_multi_view_debug.h5')

        run_cmd('xreg-pao-draw-bones pao_cuts_seg.nii.gz pelvis_app_lands.fcsv left --femur-frag-xform rel_pose_frag.h5 --femur-only-xform rel_pose_femur.h5 --femur-not-rel-to-frag --cam-view ap --bg-color 1 1 1')

        run_cmd('xreg-regi2d3d-replay --video-fps 10 --proj-ds 0.5 multi_obj_multi_view_debug.h5')
        
        view_movie('edges.mp4')
        view_movie('mov.mp4')

    #################################################################
    # Based off: https://github.com/rg2/xreg/wiki/Example%3A-TCIA-Hip-Radiograph-Rigid-Registration
    if True:
        print('Workflow for 2D/3D registration to real pelvis radiograph...')
        
        print('3D DICOM conversion/resampling...')
        
        download_file('https://services.cancerimagingarchive.net/services/v4/TCIA/query/getImage?SeriesInstanceUID=1.3.6.1.4.1.14519.5.2.1.1706.4016.124291161306415775701317569638', 'TCGA-G2-A3VY_ct.zip', True)
       
        extract_zip('TCGA-G2-A3VY_ct.zip', 'TCGA-G2-A3VY_ct_dcm')

        run_cmd('xreg-convert-dicom-vols --one TCGA-G2-A3VY_ct_dcm TCGA-G2-A3VY_ct.nii.gz')
       
        view_vol('TCGA-G2-A3VY_ct.nii.gz', slicer_path)
        
        download_file('https://raw.githubusercontent.com/wiki/rg2/xreg/examples/tcia_hip_radiograph_rigid_2d_3d/TCGA-G2-A3VY_ct_full_pelvis-label.nii.gz', use_existing=True)

        download_file('https://github.com/rg2/xreg/wiki/examples/tcia_hip_radiograph_rigid_2d_3d/pelvis_3d_lands.fcsv', use_existing=True)

        print('Creating 3D mesh of pelvis...')
        run_cmd('xreg-create-mesh TCGA-G2-A3VY_ct_full_pelvis-label.nii.gz pelvis.ply 1 2')

        print('2D DICOM conversion...')

        download_file('https://services.cancerimagingarchive.net/services/v4/TCIA/query/getImage?SeriesInstanceUID=1.3.6.1.4.1.14519.5.2.1.1706.4016.146872675804132000774592060313', 'TCGA-G2-A3VY_radiographs_1.zip', True)
        
        extract_zip('TCGA-G2-A3VY_radiographs_1.zip', 'TCGA-G2-A3VY_radiographs')

        download_file('https://github.com/rg2/xreg/wiki/examples/tcia_hip_radiograph_rigid_2d_3d/2-c73e.fcsv', use_existing=True)

        run_cmd('xreg-convert-dicom-radiograph TCGA-G2-A3VY_radiographs/2-c73e4bdf6c7d19772d00acd2891965fa.dcm 2-c73e_pd.h5 2-c73e.fcsv')

        run_cmd('xreg-remap-tile-proj-data -d 0.25 -o 2-c73e_pd.h5 2-c73e_remap.png')

        view_image('2-c73e_remap.png')

        run_cmd('xreg-draw-xray-scene -i -l 2-c73e_pd.h5 pelvis.ply - pelvis_3d_lands.fcsv -')

        print('2D/3D registration of pelvis..')

        run_cmd('xreg-hip-surg-pelvis-single-view-regi-2d-3d TCGA-G2-A3VY_ct.nii.gz pelvis_3d_lands.fcsv 2-c73e_pd.h5 2-c73e_pelvis_regi.h5 2-c73e_regi_debug.h5 --no-log-remap -s TCGA-G2-A3VY_ct_full_pelvis-label.nii.gz')
        
        run_cmd('xreg-draw-xray-scene -i -l 2-c73e_pd.h5 pelvis.ply 2-c73e_pelvis_regi.h5 pelvis_3d_lands.fcsv 2-c73e_pelvis_regi.h5')

        run_cmd('xreg-regi2d3d-replay --proj-ds 0.5 --video-fps 10 2-c73e_regi_debug.h5')
        
        view_movie('edges.mp4')
        view_movie('mov.mp4')

