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

REM Build everything in a temporary directory, e.g. C:\tmp.
REM Change this to the directory of your choice
SET "BUILD_ROOT=C:\tmp"

REM Everything will be installed here using the standard
REM bin, include, lib layout
SET "INSTALL_ROOT=C:\usr\local"

REM Use forward slashes for paths given to CMake, this converts the
REM backslashes (\) to forward (/)
SET "INSTALL_ROOT_CMAKE=%INSTALL_ROOT:\=/%"

REM Using a custom install version of CMake rather than the version available
REM through the VS installer (3.17.20032601-MSVC_2) as the VS version resulted in
REM the call to find_package() for ITK in xReg to fail. I do not know the reason,
REM but using an official version downloaded from cmake.org works.
SET "PATH=C:\cmake-3.19.2-win64-x64\bin;%PATH%"

REM Ninja may be installed during VS installation - it greatly speeds up the builds
SET "CMAKE_GENERATOR_ARG=-G Ninja"
REM SET "CMAKE_GENERATOR_ARG="

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

if %NEED_TO_DOWNLOAD% == true (

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
curl -L -O -J https://github.com/InsightSoftwareConsortium/ITK/releases/download/v5.1.1/InsightToolkit-5.1.1.zip || EXIT /b

ECHO Extracting ITK
tar -xf InsightToolkit-5.1.1.zip || EXIT /b

ECHO Downloading OpenCV
curl -L -O -J https://github.com/opencv/opencv/archive/3.4.12.zip || EXIT /b

ECHO Extracting OpenCV
tar -xf opencv-3.4.12.zip || EXIT /b

)

if %NEED_TO_BUILD_THIRD_PARTY% == true (

MOVE ffmpeg-4.3.1-2020-11-19-full_build\bin\ffmpeg.exe %INSTALL_ROOT%\bin || EXIT /b

MOVE tbb %INSTALL_ROOT%\tbb || EXIT /b

COPY %INSTALL_ROOT%\tbb\bin\intel64\vc14\tbb.dll %INSTALL_ROOT%\bin || EXIT /b

MOVE boost_1_74_0\boost %INSTALL_ROOT%\include\boost || EXIT /b

MOVE eigen-3.3.4\Eigen %INSTALL_ROOT%\include\Eigen || EXIT /b
MOVE eigen-3.3.4\unsupported %INSTALL_ROOT%\include\unsupported || EXIT /b
MOVE eigen-3.3.4\signature_of_eigen3_matrix_library %INSTALL_ROOT%\include\signature_of_eigen3_matrix_library || EXIT /b

MOVE viennacl-dev-release-1.7.1\viennacl %INSTALL_ROOT%\include\viennacl || EXIT /b

cd fmt-5.3.0 || EXIT /b

mkdir build || EXIT /b

cd build || EXIT /b

REM Note that we are opting to always link fmt statically -
REM it is small and should not cause executable sizes to balloon.

cmake %CMAKE_GENERATOR_ARG% .. ^
    -DCMAKE_INSTALL_PREFIX:PATH=%INSTALL_ROOT_CMAKE% ^
    -DCMAKE_CXX_STANDARD:STRING="11" ^
    -DCMAKE_BUILD_TYPE:STRING=%BUILD_CONFIG% ^
    -DBUILD_SHARED_LIBS:BOOL=OFF ^
    -DFMT_USE_CPP11:BOOL=ON ^
    -DFMT_TEST:BOOL=OFF ^
    -DFMT_INSTALL:BOOL=ON ^
    -DFMT_DOC:BOOL=OFF || EXIT /b

cmake --build . --config %BUILD_CONFIG% || EXIT /b

cmake --install . || EXIT /b

cd ..\.. || EXIT /b

cd nlopt-2.5.0 || EXIT /b

mkdir build || EXIT /b

cd build || EXIT /b

REM For some reason this command fails with the following message:
REM "The filename, directory name, or volume label syntax is incorrect."
REM However, the CMake configuration succeeds and the project may still be build.

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
cmake -UCMAKE_INSTALL_PREFIX -DCMAKE_INSTALL_PREFIX:PATH=%INSTALL_ROOT_CMAKE% . || EXIT /b

cmake --build . --config %BUILD_CONFIG% || EXIT /b

cmake --install . || EXIT /b

cd ..\.. || EXIT /b

cd VTK-8.2.0 || EXIT /b

mkdir build || EXIT /b

cd build || EXIT /b

cmake %CMAKE_GENERATOR_ARG% .. ^
    -DCMAKE_INSTALL_PREFIX:PATH=%INSTALL_ROOT_CMAKE% ^
    -DCMAKE_CXX_STANDARD:STRING="11" ^
    -DCMAKE_BUILD_TYPE:STRING=%BUILD_CONFIG% ^
    -DBUILD_SHARED_LIBS:BOOL=%BUILD_SHARED% ^
    -DVTK_Group_Imaging:BOOL=ON ^
    -DVTK_Group_Views:BOOL=ON ^
    -DBUILD_TESTING:BOOL=OFF || EXIT /b

cmake --build . --config %BUILD_CONFIG% || EXIT /b

cmake --install . || EXIT /b

cd ..\.. || EXIT /b

cd InsightToolkit-5.1.1 || EXIT /b

mkdir build || EXIT /b

cd build || EXIT /b

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

cmake --build . --config %BUILD_CONFIG% || EXIT /b

cmake --install . || EXIT /b

cd ..\.. || EXIT /b

cd opencv-3.4.12 || EXIT /b

mkdir build || EXIT /b

cd build || EXIT /b

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

cmake --build . --config %BUILD_CONFIG% || EXIT /b

cmake --install . || EXIT /b

REM COPY %INSTALL_ROOT%\x64\vc16\bin\*.dll %INSTALL_ROOT%\bin

cd ..\.. || EXIT /b

)

if %NEED_TO_START_XREG% == true (

mkdir xreg_build || EXIT /b

cd xreg_build || EXIT /b

cmake %CMAKE_GENERATOR_ARG% ^
        -DCMAKE_PREFIX_PATH:PATH=%INSTALL_ROOT_CMAKE% ^
        -DCMAKE_INSTALL_PREFIX:PATH=%INSTALL_ROOT_CMAKE% ^
        -DCMAKE_CXX_STANDARD:STRING="11" ^
        -DCMAKE_BUILD_TYPE:STRING=%BUILD_CONFIG% ^
        -DBUILD_SHARED_LIBS:BOOL=%BUILD_SHARED% ^
        %XREG_SOURCE_DIR% || EXIT /b

cmake --build . --config %BUILD_CONFIG% || EXIT /b

cmake --install . || EXIT /b

cd .. || EXIT /b

)
