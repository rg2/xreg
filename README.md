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

This software was originally created while conducting research with [Russell Taylor](http://www.cs.jhu.edu/~rht), [Mehran Armand](https://bigss.lcsr.jhu.edu), and [Mathias Unberath](https://mathiasunberath.github.io/) within the [Laboratory for Computational Sensing and Robotics](https://lcsr.jhu.edu) at [Johns Hopkins University](https://www.jhu.edu) and with Yoshito Otake of the [Imaging-based Computational Biomedicine Laboratory](http://icb-lab.naist.jp) at [NAIST](http://www.naist.jp).
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

## Building
A standard CMake configure/generate process is used.
It is recommended to generate Ninja build files for fast compilation. 
Example scripts for building all dependencies and the xReg repository are provided for POSIX systems (e.g. MacOS and Linux) [here](example_build_script) and [here](example_build_script_2), and also for [Windows](example_build_script_win.cmd).
The [docker](docker) directory demonstrates how Docker may be used to build the software.

### Dependencies
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
  * Optional: [ffmpeg](https://ffmpeg.org) is used for writing videos when it is found in the system path. When ffmpeg is not found, the following fallbacks are used:
    * MacOS: a video writer using AVFoundation,
    * Windows, Linux, something else: a writer using OpenCV.

## Testing
Functional testing is available in the form of a [python script](tests/wiki_cmds.py) that runs the commands found on the [wiki walkthrough](https://github.com/rg2/xreg/wiki#walkthrough).
Results of the commands need to be examined by the user and determined to be successful or failures.
All necessary data is downloaded automatically.
When the appropriate visualization programs are available on the system, the script also automatically loads the output data for inspection.

## License and Attribution
The software is available for use under the [MIT License](LICENSE).

If you have found this software useful in your work, we kindly ask that you cite the most appropriate references listed [here](https://github.com/rg2/xreg/wiki/Licensing-and-Attribution).
