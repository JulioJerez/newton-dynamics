@echo off
setlocal

rem Define the starting directory. Use "." for the current directory.
set "src_dir=."
set "dst_dir=xxx/"

echo xxxxxxxxxxxxx
echo "%dst_dir%"
echo zzzzzzzzzzzzz

for /R "%src_dir%" %%f in (*.zip) do (
    echo "processing %%f"
    rem fbxToMesh.exe %%f
    tar -xf %%f -C "%dst_dir%"
)
pause
endlocal