![emscripten logo](https://github.com/JulioJerez/newton-dynamics/blob/dliw/newton-dynamics/newtonLogo/newtonLogo.png)

Newton Dynamics is a realistic, cross-platform rigidbodies physics simulation library. 
It integrates easily into games and game engines or realtime any interactive simulation application, delivering a balance between top performance along with high accuracy and stability.
With ongoing development and a permissive license, Newton Dynamics is an excellent choice for a wide range of projects, from scientific simulations to full-scale game engines.

* Links to older repository can be found at: <https://github.com/MADEAPPS/newton-dynamics>
* Links to **demos**, **tutorial**, **FAQ**, etc: <https://github.com/MADEAPPS/newton-dynamics/wiki>
* youtube channel at: <https://www.youtube.com/@NewtonDynamics-j2j/videos>

* note: <http://newtondynamics.com> is discontinue


Compiling Newton
================

## Windows
Cmake generate varius Visual Studio solutions for: build sdk, demos, wrappers and ulities 

* Newton Core and packages: [sdk/projects/](sdk/projects/)
* demo sandbox: [applications/demosSandbox/projects/](applications/demosSandbox/projects/)

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
