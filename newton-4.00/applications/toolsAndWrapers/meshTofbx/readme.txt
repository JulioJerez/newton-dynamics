This tool converts BVH and Newton proprietary NDM files to Autodesk FBX format.

Unfortunately, the Autodesk FBX SDK is free to use, but it is not redistributable. 
Therefore, to build this tool, you must download and install the FBX SDK directly from Autodesk.

At the time of this build, the SDK can be downloaded from:

https://www.autodesk.com/developer-network/platform-technologies/fbx-sdk-2020-0

After downloading and installing the SDK, you must create a user environment variable pointing to the SDK installation folder:

FBX_SDK=[sdk_path]

For example:

# FBX_SDK=C:\Program Files\Autodesk\FBX\FBX SDK\2020.0.1\
FBX_SDK=C:\Program Files\Autodesk\FBX\FBX SDK\2020.3.2\

Building the Tool
Open CMake and select the source folder.
Select a build folder.
Click Configure and then Build.
Open the generated Visual Studio solution.
Compile the tool in Visual Studio.