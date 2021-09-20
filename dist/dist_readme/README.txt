            
            +------------------------------------------------------+
            | xReg: Modeling and Registration Software for Surgery |
            |                                                      |
            |                  Executable Programs                 |
            |                     v2021.09.19.0                    |
            |                                                      |
            |              Robert Grupp (grupp@jhu.edu)            |
            +------------------------------------------------------+

See the following pages for more information about the library and tools:
  * https://github.com/rg2/xreg
  * https://github.com/rg2/xreg/releases
  * https://github.com/rg2/xreg/wiki
  * https://github.com/rg2/xreg/wiki#walkthrough

The walkthrough link above contains demonstrations (with data) on example usages of these tools.

Included in this release is a "bin" directory which contains the xReg executable files. Each xReg
program name is prefixed with "xreg-" (e.g. "xreg-convert-dicom-vols"). The help message of each
program, obtained by passing the "--help" or "-h" flags, documents the program's interface and
the available options.

The "bin" directory of the Windows release also includes a multitude of ".dll" files, which are
shared libraries of the xReg dependencies and are necessary for running the programs. Also
included on Windows is a "setup-xreg-vars.bat" batch script, which sets the PATH variable to
include the bin directory, so that the xReg program names may be entered into the command console
without having to type the full path.

A lib directory is also included in the Mac and Linux releases. This directory contains the
Mac/Linux versions of the xReg dependencies' shared libraries and are necessary for executing the
programs. Care was taken to setup RPATHs on each binary and the programs should run as provided
without having to introduce or modify any PATH/LD_LIBRARY_PATH/DYLD_LIBRARY_PATH environment
variables (see the "dist" directory in the git repository). Having said that, you probably want
to add the bin directory to your PATH when working with these tools directly in a terminal.

Enjoy!

This software is provided under the terms of the MIT license and a copy of the license may be
found in the "LICENSE" file.

If you have found this software useful in your work, we kindly ask that you cite the most
appropriate references listed below. If you are unsure which work to cite, please see this wiki
page for guidance: https://github.com/rg2/xreg/wiki/Licensing-and-Attribution.


    Grupp, Robert B., et al. "Pose estimation of periacetabular osteotomy fragments with intraoperative X-ray navigation." IEEE Transactions on Biomedical Engineering 67.2 (2019): 441-452.
    ----------------------------------------------------------------------
    @article{grupp2019pose,
      title={Pose estimation of periacetabular osteotomy fragments with intraoperative {X}-ray navigation},
      author={Grupp, Robert B and Hegeman, Rachel A and Murphy, Ryan J and Alexander, Clayton P and Otake, Yoshito and McArthur, Benjamin A and Armand, Mehran and Taylor, Russell H},
      journal={IEEE Transactions on Biomedical Engineering},
      volume={67},
      number={2},
      pages={441--452},
      year={2019},
      publisher={IEEE}
    }


    Grupp, Robert B., Mehran Armand, and Russell H. Taylor. "Patch-based image similarity for intraoperative 2D/3D pelvis registration during periacetabular osteotomy." OR 2.0 Context-Aware Operating Theaters, Computer Assisted Robotic Endoscopy, Clinical Image-Based Procedures, and Skin Image Analysis. Springer, Cham, 2018. 153-163.
    ----------------------------------------------------------------------
    @incollection{grupp2018patch,
      title={Patch-based image similarity for intraoperative {2D}/{3D} pelvis registration during periacetabular osteotomy},
      author={Grupp, Robert B and Armand, Mehran and Taylor, Russell H},
      booktitle={OR 2.0 Context-Aware Operating Theaters, Computer Assisted Robotic Endoscopy, Clinical Image-Based Procedures, and Skin Image Analysis},
      pages={153--163},
      year={2018},
      publisher={Springer}
    }


    Grupp, Robert B., et al. "Automatic annotation of hip anatomy in fluoroscopy for robust and efficient 2D/3D registration." International Journal of Computer Assisted Radiology and Surgery (2020): 1-11.
    ----------------------------------------------------------------------
    @article{grupp2020automatic,
      title={Automatic annotation of hip anatomy in fluoroscopy for robust and efficient {2D}/{3D} registration},
      author={Grupp, Robert B and Unberath, Mathias and Gao, Cong and Hegeman, Rachel A and Murphy, Ryan J and Alexander, Clayton P and Otake, Yoshito and McArthur, Benjamin A and Armand, Mehran and Taylor, Russell H},
      journal={International Journal of Computer Assisted Radiology and Surgery},
      pages={1--11},
      publisher={Springer}
    }

    
    Grupp, Robert, et al. "Fast and automatic periacetabular osteotomy fragment pose estimation using intraoperatively implanted fiducials and single-view fluoroscopy." Physics in Medicine & Biology (2020).
    ----------------------------------------------------------------------
    @article{grupp2020fast,
      title={Fast and automatic periacetabular osteotomy fragment pose estimation using intraoperatively implanted fiducials and single-view fluoroscopy},
      author={Grupp, Robert and Murphy, Ryan and Hegeman, Rachel and Alexander, Clayton and Unberath, Mathias and Otake, Yoshito and McArthur, Benjamin and Armand, Mehran and Taylor, Russell H},
      journal={Physics in Medicine \& Biology},
      year={2020},
      publisher={IOP Publishing}
    }

    
    Grupp, R., et al. "Pelvis surface estimation from partial CT for computer-aided pelvic osteotomies." Orthopaedic Proceedings. Vol. 98. No. SUPP_5. The British Editorial Society of Bone & Joint Surgery, 2016.
    ----------------------------------------------------------------------
    @inproceedings{grupp2016pelvis,
      title={Pelvis surface estimation from partial {CT} for computer-aided pelvic osteotomies},
      author={Grupp, R and Otake, Y and Murphy, R and Parvizi, J and Armand, M and Taylor, R},
      booktitle={Orthopaedic Proceedings},
      volume={98},
      number={SUPP\_5},
      pages={55--55},
      year={2016},
      organization={The British Editorial Society of Bone \& Joint Surgery}
    }

    
    Grupp, Robert B., et al. "Smooth extrapolation of unknown anatomy via statistical shape models." Medical Imaging 2015: Image-Guided Procedures, Robotic Interventions, and Modeling. Vol. 9415. International Society for Optics and Photonics, 2015.
    ----------------------------------------------------------------------
    @inproceedings{grupp2015smooth,
      title={Smooth extrapolation of unknown anatomy via statistical shape models},
      author={Grupp, Robert B and Chiang, H and Otake, Yoshito and Murphy, Ryan J and Gordon, Chad R and Armand, Mehran and Taylor, Russell H},
      booktitle={Medical Imaging 2015: Image-Guided Procedures, Robotic Interventions, and Modeling},
      volume={9415},
      pages={941524},
      year={2015},
      organization={International Society for Optics and Photonics}
    }

