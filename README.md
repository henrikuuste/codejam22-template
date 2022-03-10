# CodeJam22 project
> In theory, theory and practice are the same.
> In practice, they are not.
>
> -- <cite>Falsely attributed to many people :)</cite>

Project template for CodeJam22 applications.

The core of this project is a C++ library that can be used in a ROS node.

## Build instructions

### Developer build
```sh
mkdir build && cd build
cmake .. -G "Ninja Multi-Config"
cmake --build . --config [Release | Debug | RelWithDebInfo]
ctest -VV -C [Release | Debug | RelWithDebInfo]
```

### Package deployment
```sh
conan create . codejam22/latest -b missing
```

## References
### CMake
* https://cmake.org/cmake/help/latest/index.html
* https://cmake.org/cmake/help/latest/manual/ctest.1.html
* https://github.com/onqtam/awesome-cmake
* https://github.com/cpp-best-practices/project_options

### Conan
* https://github.com/conan-io/cmake-conan
* https://docs.conan.io/en/latest/cheatsheet.html
* https://docs.conan.io/en/latest/reference/conanfile.html
* https://docs.conan.io/en/latest/reference/build_helpers/cmake.html

### Docker
* https://code.visualstudio.com/docs/remote/containers
* https://code.visualstudio.com/docs/remote/devcontainerjson-reference
* https://github.com/deluan/zsh-in-docker

### Libs
* https://github.com/gabime/spdlog
* https://github.com/docopt/docopt.cpp
* https://github.com/catchorg/Catch2
* https://github.com/eranpeer/FakeIt

## TODO

### Build system setup
- [x] Docker development container
- [x] Library for core features
- [x] Unit tests for library
- [x] Basic demo app for development
- [x] Deploy to package management
  - Integrate with ROS node later
- [x] Static analysis
- [ ] Version management
- [x] Compiler and OS agnostic
- [x] Separate development and user builds
- [x] Documentation

### ROS setup
- [ ] Package and node
- [ ] Link with library
- [ ] Tests
- [ ] Launch setup
- [ ] RViz visualization setup

### Path planner interface
- [ ] Core planner interface
  - [ ] Return a path
- [ ] Environment data (aka map) interface
  - [ ] Read from external file(s)
  - [ ] Tile based
- [ ] Composability
  - [ ] Planning
  - [ ] Cost
  - [ ] Environment layers

### Test cases
