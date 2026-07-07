@echo off
setlocal

rem define the starting directory. Use "." for the current directory.
rem set "src_dir=."

for %%f in ("*.zip") do (
    echo "unpacking: %%~nxf"
    tar -xf %%~nxf
)
endlocal