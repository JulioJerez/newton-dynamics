![emscripten logo](https://raw.githubusercontent.com/MADEAPPS/newton-dynamics/master/applications/newtonLogo/newtonLogo.png)

Newton dynamics is a realistic, cross-platform physics simulation library. It can easily be integrated into games and game engines and provides top of its class performance and simulation stability.
Ongoing development and a permissive license makes Newton Dynamics a good choice for all kinds of projects from scientific projects to game engines.

* Links to older repository can be found at: <https://github.com/MADEAPPS/newton-dynamics>
* Links to **demos**, **tutorial**, **FAQ**, etc: <https://github.com/MADEAPPS/newton-dynamics/wiki>

* note: <http://newtondynamics.com> is discontinue


Compiling Newton
================

## Windows
Cmake generate solutions Project Visual Studio, build sdk, demos, wrapes and ulities 

* Newton Core and packages: [sdk/projects/](sdk/projects/)
* demo sandbox: [applications/demosSandbox/projects/](applications/demosSandbox/projects/)

## OSX
Cmake XCode for SDK, 

* Newton Core and packages: [sdk/projects/](sdk/projects/)

note: at the time since demos ar build for opengl, they is not backend graphich for apple metal. 

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
