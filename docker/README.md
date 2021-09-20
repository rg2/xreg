# Docker Builds

## Introduction
The files in this directory may be used to create Docker images with builds of the xReg software.
Ubuntu and CentOS base images are supported.
The releases of exectuable programs for CentOS 7 and Ubuntu 16.04, 18.04, 20.04 were created using this pipeline. 

The build argument `os_name` is used to choose either Ubuntu or CentOS (e.g. passing `--build-arg os_name=ubuntu` or `--build-arg os_name=centos` to `docker build`).
The operating system version is specified using the `os_version` build argument (e.g. passing `--build-arg os_version=7` or `--build-arg os_version=16.04` to `docker build`).
When these arguments are not provided, Ubuntu 16.04 is used by default.

## GitHub Packages
The images containing all xReg dependencies are available as GitHub packages [here](https://github.com/rg2?tab=packages&repo_name=xreg).
Although these are primarily intended to be used as part of a continuous integration pipeline, others may find them useful for developing or building xReg.
More images may be made available as needed/requested.

## List of Files
* `Dockerfile.centos_dev_base`
  * Installs packages needed for xReg build on *CentOS* and also builds a recent version of CMake
* `Dockerfile.ubuntu_dev_base`
  * Installs packages needed for xReg build on *Ubuntu* and also builds a recent version of CMake
* `Dockerfile.xreg-deps`
  * Creates an image with all of the necessary dependencies needed to build xReg. e.g. builds `ninja`, downloads `ffmpeg`, downloads `boost`, builds `VTK`, builds `ITK`, etc.
* `Dockerfile.xreg`
  * Builds xReg library and executable programs
* `Dockerfile.xreg-dist-bin`
  * Extracts the xReg executables for distribution. This bundles the executables and shared libraries together in an archive.

## Example Commands
A copy/pastable list of shell commands is provided in [`example_commands`](example_commands).

Specific commands are also listed below:

NOTE: all of these commands assume that the xReg repository contents are located in `~/xreg-git`.

### Ubuntu 16.04 Build

1. `docker build -t xreg-dev-base-ubuntu-16.04 --build-arg os_name=ubuntu --build-arg os_version=16.04 -f ~/xreg-git/docker/Dockerfile.ubuntu_dev_base .`
2. `docker build -t xreg-deps-ubuntu-16.04 --build-arg os_name=ubuntu --build-arg os_version=16.04 -f ~/xreg-git/docker/Dockerfile.xreg-deps .`
3. `docker build -t xreg-ubuntu-16.04 --build-arg os_name=ubuntu --build-arg os_version=16.04 -f ~/xreg-git/docker/Dockerfile.xreg ~/xreg-git`
4. `docker build -t xreg-dist-ubuntu-16.04 --build-arg os_name=ubuntu --build-arg os_version=16.04 -f ~/xreg-git/docker/Dockerfile.xreg-dist-bin ~/xreg-git`
    * Copy the package from the image:
      1. `docker create xreg-dist-ubuntu-16.04` 
      2. `docker cp <container ID>:/xreg-ubuntu-16.04.tar.gz .`

### CentOS 7 Build

Basically the steps above with "ubuntu" replaced with "centos" and "16.04" replaced with "7".

1. `docker build -t xreg-dev-base-centos-7 --build-arg os_name=centos --build-arg os_version=7 -f ~/xreg-git/docker/Dockerfile.centos_dev_base .`
2. `docker build -t xreg-deps-centos-7 --build-arg os_name=centos --build-arg os_version=7 -f ~/xreg-git/docker/Dockerfile.xreg-deps .`
3. `docker build -t xreg-centos-7 --build-arg os_name=centos --build-arg os_version=7 -f ~/xreg-git/docker/Dockerfile.xreg ~/xreg-git`
4. `docker build -t xreg-dist-centos-7 --build-arg os_name=centos --build-arg os_version=7 -f ~/xreg-git/docker/Dockerfile.xreg-dist-bin ~/xreg-git`
    * Copy the package from the image:
      1. `docker create xreg-dist-centos-7` 
      2. `docker cp <container ID>:/xreg-centos-7.tar.gz .`
