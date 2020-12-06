# xReg: Modeling and Registration Software for Surgery
## Introduction
This repository contains library routines and stand-alone programs for various modeling and registration tasks relating to surgery, with a particular focus on 2D/3D registration for intraoperative X-ray navigation.
Although xReg was developed from a practical approach and provides an out-of-the-box capability for intraoperative fluoroscopic navigation, it is also very much compatible with research-oriented goals.
In addition to accelerating the development of new registration components through the extendable interfaces for ray casting, image similarity computation and iterative optimization, xReg is also useful as an initial baseline for evaluating the neural network approaches *du jour*.

Much of the functionality provided by xReg is also useful for other computer-assisted surgery tasks.
A comprehensive listing of the features of the library are provided below, along with a list of the executable programs that are also included in this repository.
Please visit the [wiki](https://github.com/rg2/xreg/wiki) for descriptions on the use of the library and executable programs.
Basic usage of the executables is provided via a walkthrough/tutorial [here](https://github.com/rg2/xreg/wiki#walkthrough).
Please submit an [issue](https://github.com/rg2/xreg/issues) for any problems, feature requests, or suggestions.

This software was originally created while conducting research with [Russell Taylor](http://www.cs.jhu.edu/~rht), [Mehran Armand](https://bigss.lcsr.jhu.edu), and [Mathias Unberath](https://mathiasunberath.github.io/) within the [Laboratory for Computational Sensing and Robotics](https://lcsr.jhu.edu) at [Johns Hopkins University](https://www.jhu.edu).
The current repository represents a rewrite/refactor of the original, internal, version of this software, with a focus on minimizing compile times and being reusable to novice developers and researchers.

## General Capabilities
A listing of the high-level capabilities of the xReg library is provided below.
Stand-alone programs provide implementations of common, but very specific, use-cases, such as 2D/3D registration of a pelvis model to a single fluoroscopic view.
These programs also serve as usage examples for the xReg library.

A detailed listing of the features and programs, along with links to the appropriate source files is provided in the [Features Index](FeaturesIndex.md).

### Library Features
* Registration:
  * Efficient ray casters for 2D/3D registration and visualization
  * Image similarity metrics for driving 2D/3D registrations
  * Various optimization strategies for 2D/3D intensity-based registration
  * Pipeline for chaining together 2D/3D registrations
  * Perspective-n-Point (PnP) solvers (paired point 2D/3D)
  * Point-based 3D/3D registration methods
* Mesh Processing
* Image/Volume Processing
* Numerical Optimization
* Geometric Primitives and Spatial Data Structures
* Spatial Transformation Utilities
* Visualization
* Basic Math Utilities
* General/Common Utilities (e.g. string parsing, filesystem objects)
* Hip Surgery (e.g. planning, modeling, simulation)

## Planned Work
Although the following capabilities currently only exist in an internal version of the xReg software, they will be incorporated into this repository at a future date:
* Executable for running a multiple-view/multiple-resolution 2D/3D registration pipeline defined using a configuration file
* Intraoperative reconstruction of PAO bone fragments
* Utilities for creation and manipulation of statistical shape models
* Shape completion from partial shapes and statistical models
* More point cloud manipulation utilities
* Python bindings, conda integration
* And more...

## Dependencies
* C++ 11 compatible compiler
  * Tested with various flavors of gcc and Apple clang, and Visual Studio 2019
* External libraries (compatible versions are listed):
  * [Intel Threading Building Blocks (TBB)](https://github.com/oneapi-src/oneTBB) (20170919oss, v2020.3)
  * [Boost](https://www.boost.org) (header only) (1.74)
  * [Eigen3](http://eigen.tuxfamily.org) (3.3.4)
  * [fmt](https://fmt.dev) (5.3.0)
  * [NLOpt](https://github.com/stevengj/nlopt) (2.5.0)
  * [ITK](https://itk.org) (5.1.1)
  * [VTK](https://vtk.org) (8.2.0)
  * [OpenCV](https://opencv.org) (3.4.12)
  * [ViennaCL](http://viennacl.sourceforge.net) (1.7.1)
  * Highly recomended for GPU acceleration: OpenCL (1.x)
    * Only needed at runtime on Windows and Linux and is typically provided with your graphics drivers or CUDA SDK
    * Included with MacOS
  * Optional: [ffmpeg](https://ffmpeg.org) is used for writing videos when it is found in the system path. The OpenCV video writer is used if ffmpeg is not available.

## Building
A standard CMake configure/generate process is used.
It is recommended to generate Ninja build files for fast compilation. 
Example scripts for building all dependencies and the xReg repository are provided for [POSIX systems](example_build_script) (e.g. MacOS and Linux) and [Windows](example_build_script_win.cmd).
The [docker](docker) directory demonstrates how Docker may be used to build the software.

## License and Attribution
The software is available for use under the [MIT License](LICENSE).

If you have found this software useful in your work, we kindly ask that you cite the most appropriate references below:
```
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
```
```
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
```
```
Grupp, Robert B., et al. "Automatic annotation of hip anatomy in fluoroscopy for robust and efficient 2D/3D registration." International Journal of Computer Assisted Radiology and Surgery (2020): 1-11.
----------------------------------------------------------------------
@article{grupp2020automatic,
  title={Automatic annotation of hip anatomy in fluoroscopy for robust and efficient {2D}/{3D} registration},
  author={Grupp, Robert B and Unberath, Mathias and Gao, Cong and Hegeman, Rachel A and Murphy, Ryan J and Alexander, Clayton P and Otake, Yoshito and McArthur, Benjamin A and Armand, Mehran and Taylor, Russell H},
  journal={International Journal of Computer Assisted Radiology and Surgery},
  pages={1--11},
  publisher={Springer}
}
```
```
Grupp, Robert, et al. "Fast and automatic periacetabular osteotomy fragment pose estimation using intraoperatively implanted fiducials and single-view fluoroscopy." Physics in Medicine & Biology (2020).
----------------------------------------------------------------------
@article{grupp2020fast,
  title={Fast and automatic periacetabular osteotomy fragment pose estimation using intraoperatively implanted fiducials and single-view fluoroscopy},
  author={Grupp, Robert and Murphy, Ryan and Hegeman, Rachel and Alexander, Clayton and Unberath, Mathias and Otake, Yoshito and McArthur, Benjamin and Armand, Mehran and Taylor, Russell H},
  journal={Physics in Medicine \& Biology},
  year={2020},
  publisher={IOP Publishing}
}
```
```
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
```
```
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
```
