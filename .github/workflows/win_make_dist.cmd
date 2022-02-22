REM Create a .zip of the compiled binaries

REM Use the drive where this script currently resides
SET CUR_DRIVE="%~d0"
ECHO CUR_DRIVE = %CUR_DRIVE%

REM Everything will be installed here using the standard
REM bin, include, lib layout
SET "INSTALL_ROOT=%CUR_DRIVE%\usr\local"
ECHO INSTALL_ROOT = %INSTALL_ROOT%

SET "XREG_NAME=xreg-win64"
MKDIR %XREG_NAME% || EXIT /b

XCOPY /E %INSTALL_ROOT%\bin %XREG_NAME%\bin\

COPY dist\dist_readme\README.txt %XREG_NAME%

COPY LICENSE %XREG_NAME%

REM GitHub runners have 7zip installed, so we will use that to create the .zip
7z a -tzip %XREG_NAME%.zip %XREG_NAME%
