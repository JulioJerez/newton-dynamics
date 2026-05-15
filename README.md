![emscripten logo](https://github.com/JulioJerez/newton-dynamics/blob/dliw/newton-dynamics/newtonLogo/newtonLogo.png)

Newton Dynamics is a realistic, cross-platform rigid body physics simulation library designed for realtime interactive applications and game engines. 
It offers easy integration, combining high performance with accuracy and stability. 
Backed by active development and a permissive license, Newton Dynamics is well suited for everything from scientific simulations to large-scale game engine development.

* Links to older repository can be found at: <https://github.com/MADEAPPS/newton-dynamics>
* Links to **demos**, **tutorial**, **FAQ**, etc: <https://github.com/MADEAPPS/newton-dynamics/wiki>
* youtube channel at: <https://www.youtube.com/@NewtonDynamics-j2j/videos>

* note: <http://newtondynamics.com> is discontinue


Compiling Newton
================

## Windows
Cmake generate varius Visual Studio solutions for: build sdk, demos, wrappers and ulities 

* Newton Core and packages: [sdk/projects/](sdk/projects/)
* assets tool: [applications/ndNewAsset/](applications/ndNewAsset/)
* demo sandbox: [applications/ndSandbox/](applications/ndSandbox/)

## OSX
Cmake XCode for SDK, 

* Newton Core and packages: [sdk/projects/](sdk/projects/)

note: at this time since demos are build for opengl, there is not backend graphics for apple metal. 

## Linux
There are Unix makefiles in the project folders mentioned above, but it is *highly* recommended to use CMake instead.

Newton Core does not have any third party dependencies.

To build the demo sandbox the following packages need to be installed:
* OpenGL
* glfw3
* glatter

All of these should be available on any major Linux distribution (with associated `-dev` or `-devel` packages).

Alternatively they can be found in the folder [sdk/thirdParty/](sdk/thirdParty/) and built from source.


License
=======
Newton Dynamics is licensed under the zlib open source license, with little if any practical difference between them.

See `LICENSE` for the full content of the licenses.

Authors
=======
Newton Dynamics is a project of Julio Jerez and Alain Suero. See `AUTHORS` for a full list of contributors.
