REM Example script for building dependencies and xReg on Windows.
REM This is not pretty, but can be used in a pinch and serves well to illustrate
REM the settings used for achieving a successful build.
REM This should be run from a command prompt loaded with the appropriate environment
REM for building Visual Studio solutions (e.g. x64 Native Tools Command Prompt).

REM Leaving echo on is useful for debugging continuous integration issues.
REM Disable the following line to enable printing the commands:
@ECHO OFF

setlocal

SET "NEED_TO_DOWNLOAD=true"
SET "NEED_TO_BUILD_THIRD_PARTY=true"
SET "NEED_TO_START_XREG=true"

REM Use the drive where this script currently resides
SET CUR_DRIVE="%~d0"
ECHO CUR_DRIVE = %CUR_DRIVE%

REM Build everything in a temporary directory, e.g. C:\tmp.
REM Change this to the directory of your choice
SET "BUILD_ROOT=%CUR_DRIVE%\tmp"
ECHO BUILD_ROOT = %BUILD_ROOT%

REM Everything will be installed here using the standard
REM bin, include, lib layout
SET "INSTALL_ROOT=%CUR_DRIVE%\usr\local"
ECHO INSTALL_ROOT = %INSTALL_ROOT%

REM Use forward slashes for paths given to CMake, this converts the
REM backslashes (\) to forward (/)
SET "INSTALL_ROOT_CMAKE=%INSTALL_ROOT:\=/%"

REM Here is where you can specify a custom install version of CMake rather than the version available
REM through the VS installer (3.17.20032601-MSVC_2) as the VS version resulted in
REM the call to find_package() for ITK in xReg to fail. I do not know the reason,
REM but using an official version downloaded from cmake.org works.
REM If you want to use this, make sure that NEED_TO_DOWNLOAD_CMAKE is set to false later in this script.
REM SET "PATH=C:\cmake-3.19.2-win64-x64\bin;%PATH%"

REM SET "BUILD_CONFIG=Debug"
SET "BUILD_CONFIG=Release"

REM Set this to ON to build libraries as shared/dynamic.
REM Building shared libraries currently causes a linker error when building xReg.
REM I believe this is due to xReg's unconventional use of the HDF5 libraries built
REM for ITK. In the future, we should look at building HDF5 separately for use in xReg.
SET "BUILD_SHARED=OFF"

REM Root directory of the xReg source code. Assumed to be located in the same
REM directory that this script was invoked from.
SET XREG_SOURCE_DIR="%~dp0"

REM create the temporary dir if necessary
IF EXIST %BUILD_ROOT% (
ECHO %BUILD_ROOT% exists
) ELSE (
ECHO Creating %BUILD_ROOT%
MKDIR %BUILD_ROOT% || EXIT /b
)

cd %BUILD_ROOT% || EXIT /b

MKDIR %INSTALL_ROOT%\include
MKDIR %INSTALL_ROOT%\bin

REM TODO: detect when CMake is present or not
SET "NEED_TO_DOWNLOAD_CMAKE=true"

if %NEED_TO_DOWNLOAD_CMAKE% == true (
REM Update the PATH so cmake commands can be run
SET "PATH=%BUILD_ROOT%\cmake-3.22.2-windows-x86_64\bin;%PATH%"
)

REM Ninja may be installed during VS installation - it greatly speeds up the builds
SET "CMAKE_GENERATOR_ARG="

REM TODO: detect when ninja is present or not
SET "NEED_TO_DOWNLOAD_NINJA=true"

if %NEED_TO_DOWNLOAD_NINJA% == true (
REM Update the PATH so that cmake can find the downloaded version of ninja
SET "PATH=%BUILD_ROOT%\ninja-bin;%PATH%"
SET "CMAKE_GENERATOR_ARG=-G Ninja -DCMAKE_MAKE_PROGRAM:PATH=%BUILD_ROOT%\ninja-bin\ninja.exe"
)

if %NEED_TO_DOWNLOAD% == true (

REM Download CMake when needed
if %NEED_TO_DOWNLOAD_CMAKE% == true (
ECHO Downloading CMake
curl -L -O -J https://github.com/Kitware/CMake/releases/download/v3.22.2/cmake-3.22.2-windows-x86_64.zip || EXIT /b

ECHO Extracting CMake
tar -xf cmake-3.22.2-windows-x86_64.zip || EXIT /b
)

REM Download Ninja when needed
if %NEED_TO_DOWNLOAD_NINJA% == true (
ECHO Downloading Ninja
curl -L -O -J https://github.com/ninja-build/ninja/releases/download/v1.10.2/ninja-win.zip || EXIT /b

ECHO Extracting Ninja
tar -xf ninja-win.zip || EXIT /b

IF EXIST %BUILD_ROOT%\ninja-bin (
ECHO %BUILD_ROOT%\ninja-bin exists
) ELSE (
ECHO Creating %BUILD_ROOT%\ninja-bin
MKDIR %BUILD_ROOT%\ninja-bin || EXIT /b
)

MOVE ninja.exe %BUILD_ROOT%\ninja-bin || EXIT /b
)

ECHO Downloading ffmpeg
curl -L -O -J https://github.com/GyanD/codexffmpeg/releases/download/4.3.1-2020-11-19/ffmpeg-4.3.1-2020-11-19-full_build.zip || EXIT /b

ECHO Extracing ffmpeg
tar -xf ffmpeg-4.3.1-2020-11-19-full_build.zip || EXIT /b

ECHO Downloading TBB
curl -L -O -J https://github.com/oneapi-src/oneTBB/releases/download/v2020.3/tbb-2020.3-win.zip || EXIT /b

ECHO Extracting TBB
tar -xf tbb-2020.3-win.zip || EXIT /b

ECHO Downloading boost
curl -L -O -J https://boostorg.jfrog.io/artifactory/main/release/1.74.0/source/boost_1_74_0.zip || EXIT /b

ECHO Extracting boost
tar -xf boost_1_74_0.zip || EXIT /b

ECHO Downloading Eigen
curl -L -O -J https://gitlab.com/libeigen/eigen/-/archive/3.3.4/eigen-3.3.4.zip || EXIT /b

ECHO Extracting Eigen
tar -xf eigen-3.3.4.zip || EXIT /b

ECHO Downloading ViennaCL
curl -L -O -J https://github.com/viennacl/viennacl-dev/archive/release-1.7.1.zip || EXIT /b

REM This command will complete but return with errors, so we are not checking the return code.
REM Several of the CUDA backend files fail to extract. However, as we do not use the CUDA
REM backend and may proceed. Using the standard file explorer right-click extract functionality
REM works fine. This can be investigated in the future

ECHO Extracting ViennaCL
tar -xf viennacl-dev-release-1.7.1.zip

ECHO Downloading fmt
curl -L -O -J https://github.com/fmtlib/fmt/archive/5.3.0.zip || EXIT /b

ECHO Extracting fmt
tar -xf fmt-5.3.0.zip || EXIT /b

ECHO Downloading nlopt
curl -L -O -J https://github.com/stevengj/nlopt/archive/v2.5.0.zip || EXIT /b

REM This command also returns with errors, similar to the case described above for Vienna CL.
REM In this case a PNG file could not be extracted.
ECHO Extracting nlopt
tar -xf nlopt-2.5.0.zip

ECHO Downloading VTK
curl -L -O -J https://www.vtk.org/files/release/8.2/VTK-8.2.0.zip || EXIT /b

ECHO Extracting VTK
tar -xf VTK-8.2.0.zip || EXIT /b

ECHO Downloading ITK
curl -L -O -J https://github.com/InsightSoftwareConsortium/ITK/releases/download/v5.2.1/InsightToolkit-5.2.1.zip || EXIT /b

ECHO Extracting ITK
tar -xf InsightToolkit-5.2.1.zip || EXIT /b

ECHO Downloading OpenCV
curl -L -O -J https://github.com/opencv/opencv/archive/3.4.12.zip || EXIT /b

ECHO Extracting OpenCV
tar -xf opencv-3.4.12.zip || EXIT /b

)

if %NEED_TO_BUILD_THIRD_PARTY% == true (

ECHO Installing ffpeg
MOVE ffmpeg-4.3.1-2020-11-19-full_build\bin\ffmpeg.exe %INSTALL_ROOT%\bin || EXIT /b

ECHO Installing TBB (1/2)
MOVE tbb %INSTALL_ROOT%\tbb || EXIT /b

ECHO Installing TBB (2/2)
COPY %INSTALL_ROOT%\tbb\bin\intel64\vc14\tbb.dll %INSTALL_ROOT%\bin || EXIT /b

ECHO Installing boost
MOVE boost_1_74_0\boost %INSTALL_ROOT%\include\boost || EXIT /b

ECHO Installing Eigen
MOVE eigen-3.3.4\Eigen %INSTALL_ROOT%\include\Eigen || EXIT /b
MOVE eigen-3.3.4\unsupported %INSTALL_ROOT%\include\unsupported || EXIT /b
MOVE eigen-3.3.4\signature_of_eigen3_matrix_library %INSTALL_ROOT%\include\signature_of_eigen3_matrix_library || EXIT /b

ECHO Installing ViennaCL
MOVE viennacl-dev-release-1.7.1\viennacl %INSTALL_ROOT%\include\viennacl || EXIT /b

ECHO Building fmt, setting up...
cd fmt-5.3.0 || EXIT /b

mkdir build || EXIT /b

cd build || EXIT /b

REM Note that we are opting to always link fmt statically -
REM it is small and should not cause executable sizes to balloon.

ECHO fmt CMake configuring
cmake %CMAKE_GENERATOR_ARG% .. ^
    -DCMAKE_INSTALL_PREFIX:PATH=%INSTALL_ROOT_CMAKE% ^
    -DCMAKE_CXX_STANDARD:STRING="11" ^
    -DCMAKE_BUILD_TYPE:STRING=%BUILD_CONFIG% ^
    -DBUILD_SHARED_LIBS:BOOL=OFF ^
    -DFMT_USE_CPP11:BOOL=ON ^
    -DFMT_TEST:BOOL=OFF ^
    -DFMT_INSTALL:BOOL=ON ^
    -DFMT_DOC:BOOL=OFF || EXIT /b

ECHO fmt building
cmake --build . --config %BUILD_CONFIG% || EXIT /b

ECHO Installing fmt
cmake --install . || EXIT /b

cd ..\.. || EXIT /b

ECHO Buidling nlopt, setting up...
cd nlopt-2.5.0 || EXIT /b

mkdir build || EXIT /b

cd build || EXIT /b

REM For some reason this command fails with the following message:
REM "The filename, directory name, or volume label syntax is incorrect."
REM However, the CMake configuration succeeds and the project may still be build.

ECHO nlopt CMake configuring (1/2)
cmake %CMAKE_GENERATOR_ARG% .. ^
    -DCMAKE_INSTALL_PREFIX:PATH=%INSTALL_ROOT_CMAKE% ^
    -DCMAKE_CXX_STANDARD:STRING="11" ^
    -DCMAKE_BUILD_TYPE:STRING=%BUILD_CONFIG% ^
    -DBUILD_SHARED_LIBS:BOOL=%BUILD_SHARED% ^
    -DNLOPT_CXX:BOOL=OFF ^
    -DNLOPT_PYTHON:BOOL=OFF ^
    -DNLOPT_OCTAVE:BOOL=OFF ^
    -DNLOPT_MATLAB:BOOL=OFF ^
    -DNLOPT_GUILE:BOOL=OFF ^
    -DNLOPT_SWIG:BOOL=OFF ^
    -DNLOPT_LINK_PYTHON:BOOL=OFF

REM The nlopt initial configure processs overrides the value passed for CMAKE_INSTALL_PREFIX.
REM Let's delete the existing cache variable and set it again.
ECHO nlopt CMake configuring (2/2)
cmake -UCMAKE_INSTALL_PREFIX -DCMAKE_INSTALL_PREFIX:PATH=%INSTALL_ROOT_CMAKE% . || EXIT /b

ECHO nlopt building
cmake --build . --config %BUILD_CONFIG% || EXIT /b

ECHO Install nlopt
cmake --install . || EXIT /b

cd ..\.. || EXIT /b

ECHO Building ITK, setting up...
cd InsightToolkit-5.2.1 || EXIT /b

mkdir build || EXIT /b

cd build || EXIT /b

ECHO ITK CMake configuring
cmake %CMAKE_GENERATOR_ARG% .. ^
      -DCMAKE_INSTALL_PREFIX:PATH=%INSTALL_ROOT_CMAKE% ^
      -DCMAKE_CXX_STANDARD:STRING="11" ^
      -DCMAKE_BUILD_TYPE:STRING=%BUILD_CONFIG% ^
      -DBUILD_SHARED_LIBS:BOOL=%BUILD_SHARED% ^
      -DBUILD_TESTING:BOOL=OFF ^
      -DBUILD_EXAMPLES:BOOL=OFF ^
      -DITK_USE_GPU:BOOL=OFF ^
      -DModule_ITKReview:BOOL=ON ^
      -DModule_LabelErodeDilate:BOOL=ON || EXIT /b

ECHO ITK building
cmake --build . --config %BUILD_CONFIG% || EXIT /b

ECHO Install ITK
cmake --install . || EXIT /b

cd ..\.. || EXIT /b

ECHO Building VTK, setting up...
cd VTK-8.2.0 || EXIT /b

mkdir build || EXIT /b

cd build || EXIT /b

ECHO VTK CMake configuring
cmake %CMAKE_GENERATOR_ARG% .. ^
    -DCMAKE_INSTALL_PREFIX:PATH=%INSTALL_ROOT_CMAKE% ^
    -DCMAKE_CXX_STANDARD:STRING="11" ^
    -DCMAKE_BUILD_TYPE:STRING=%BUILD_CONFIG% ^
    -DBUILD_SHARED_LIBS:BOOL=%BUILD_SHARED% ^
    -DVTK_Group_Imaging:BOOL=ON ^
    -DVTK_Group_Views:BOOL=ON ^
    -DBUILD_TESTING:BOOL=OFF || EXIT /b

ECHO VTK building
cmake --build . --config %BUILD_CONFIG% || EXIT /b

ECHO Install VTK
cmake --install . || EXIT /b

cd ..\.. || EXIT /b

ECHO Building OpenCV, setting up...
cd opencv-3.4.12 || EXIT /b

mkdir build || EXIT /b

cd build || EXIT /b

EHCO OpenCV CMake configuring
cmake %CMAKE_GENERATOR_ARG% .. ^
        -DCMAKE_INSTALL_PREFIX:PATH=%INSTALL_ROOT_CMAKE% ^
        -DCMAKE_CXX_STANDARD:STRING="11" ^
        -DCMAKE_BUILD_TYPE:STRING=%BUILD_CONFIG% ^
        -DBUILD_SHARED_LIBS:BOOL=%BUILD_SHARED% ^
        -DBUILD_TESTS:BOOL=OFF ^
        -DBUILD_EXAMPLES:BOOL=OFF ^
        -DBUILD_DOCS:BOOL=OFF ^
        -DBUILD_WITH_DEBUG_INFO:BOOL=OFF ^
		-DBUILD_WITH_STATIC_CRT:BOOL=OFF ^
        -DWITH_TBB:BOOL=OFF ^
        -DBUILD_TBB:BOOL=OFF ^
        -DWITH_IPP:BOOL=OFF ^
        -DWITH_VTK:BOOL=OFF ^
        -DWITH_CUBLAS:BOOL=OFF ^
        -DWITH_CUDA:BOOL=OFF ^
        -DWITH_CUFFT:BOOL=OFF ^
        -DWITH_OPENCL:BOOL=OFF ^
        -DBUILD_opencv_python2:BOOL=OFF ^
        -DBUILD_opencv_python3:BOOL=OFF || EXIT /b

ECHO OpenCV building
cmake --build . --config %BUILD_CONFIG% || EXIT /b

ECHO Install OpenCV
cmake --install . || EXIT /b

REM COPY %INSTALL_ROOT%\x64\vc16\bin\*.dll %INSTALL_ROOT%\bin

cd ..\.. || EXIT /b

)

if %NEED_TO_START_XREG% == true (

ECHO Building xReg, setting up...
mkdir xreg_build || EXIT /b

cd xreg_build || EXIT /b

ECHO xReg CMake configuring
cmake %CMAKE_GENERATOR_ARG% ^
        -DCMAKE_PREFIX_PATH:PATH=%INSTALL_ROOT_CMAKE% ^
        -DCMAKE_INSTALL_PREFIX:PATH=%INSTALL_ROOT_CMAKE% ^
        -DCMAKE_CXX_STANDARD:STRING="11" ^
        -DCMAKE_BUILD_TYPE:STRING=%BUILD_CONFIG% ^
        -DBUILD_SHARED_LIBS:BOOL=%BUILD_SHARED% ^
        -DXREG_INCLUDE_GIT_HASH_IN_VER_STR:BOOL=ON ^
        %XREG_SOURCE_DIR% || EXIT /b

ECHO xReg building
cmake --build . --config %BUILD_CONFIG% || EXIT /b

ECHO Install xReg
cmake --install . || EXIT /b

cd .. || EXIT /b

)
