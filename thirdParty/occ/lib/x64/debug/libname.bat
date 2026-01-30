@echo off
setlocal enabledelayedexpansion

set "folder_path=.\"
set "output_file=libname.txt"

cd "%folder_path%"

rem Clear or create the output file
type nul > "%output_file%"

rem Loop through each file in the directory
for %%f in (*.lib) do (
    echo %%f >> "%output_file%"
)

echo File names have been listed in %output_file%
pause