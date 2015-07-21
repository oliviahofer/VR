@echo off

set CMAKE_CONFIGURATION_FILE=%1
if '%1'=='' (
	echo ERROR: No cmake configuration file specified.
	echo Usage: "%~nx0" cmake_configuration_file [path_to_build_tree [path_to_source_tree]]
	echo The cmake-configuration-file typically contains some set cache commands.
	echo The path-to-build-tree doesn't have to exist.
	echo If no paths are given, the path to the cmake-configuration-file is used.
	pause
	exit /B 1
)
REM echo CMAKE_CONFIGURATION_FILE is %1

set PATH_TO_BUILD_TREE=%2
if '%2'=='' (
	set PATH_TO_BUILD_TREE="%~dp1build_%~n1"
)
REM echo PATH_TO_BUILD_TREE is '%PATH_TO_BUILD_TREE%'

set PATH_TO_SOURCE_TREE=%3
if '%3'=='' (
	set PATH_TO_SOURCE_TREE="%~dp1"
)
REM echo PATH_TO_SOURCE_TREE is '%PATH_TO_SOURCE_TREE%'

set SUPERFLUOUS_ARGS=%4
if NOT '%SUPERFLUOUS_ARGS%'=='' (
	echo WARNING: superfluous arguments '%SUPERFLUOUS_ARGS%' ignored.
	pause
)


mkdir %PATH_TO_BUILD_TREE% 2> NUL
chdir /D %PATH_TO_BUILD_TREE%
if ERRORLEVEL 1 (
	echo ERROR: Change to '%PATH_TO_BUILD_TREE%' failed!
	pause
	exit /B 2
)


echo on
cmake -C %CMAKE_CONFIGURATION_FILE% %PATH_TO_SOURCE_TREE%
@echo off
if ERRORLEVEL 1 (
	echo ERROR: Cmake failed!
	pause
	exit /B 3
)


for %%i in (*.sln) do set SLN_FILE="%%~dpnxi"
REM echo SLN_FILE is %SLN_FILE%
if NOT DEFINED SLN_FILE (
	goto NO_SHORTCUT
)
set TARGET=%SLN_FILE%
set SHORTCUT=%~dp1start_vs_%~n1.lnk
set PWS=powershell.exe -ExecutionPolicy Bypass -NoLogo -NonInteractive -NoProfile
%PWS% -Command "$ws=New-Object -ComObject WScript.Shell; $sc=$ws.CreateShortcut('%SHORTCUT%'); $sc.TargetPath='%TARGET%'; $sc.Description='Open project in Visual Studio'; $sc.Save()"
if ERRORLEVEL 1 (
	echo ERROR: Creation of the shortcut to the "Visual Studio Solution (sln)" file failed!
	pause
	exit /B 4
)
:NO_SHORTCUT

