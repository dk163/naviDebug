@echo off

rem === set value only visible local
SETLOCAL

pushd .

set bat_dir=%~dp0
set project_dir=%bat_dir%
set build_dir=%project_dir%\_build
set obj_out=%build_dir%\obj
set libs_out=%build_dir%\libs

if exist %build_dir% rmdir /s /q %build_dir%
mkdir %build_dir%

cd /d %build_dir%

Title build-%cpu%-release
:: ************ no release log version ************
cmd /c %NDK_ROOT%\ndk-build.cmd APP_ABI="armeabi-v7a" NDK_PROJECT_PATH=%project_dir% APP_BUILD_SCRIPT=%project_dir%\Android.mk -B NDK_OUT=%obj_out% NDK_LIBS_OUT=%libs_out% 

rem================ over ================
popd

ENDLOCAL
