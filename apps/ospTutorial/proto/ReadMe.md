# Howto install/build/use protobuf

We use protobuf to send binary data of (feature) layers.

## Windows

Installing protobuf on Windows with CMake / VisualStudio has [not a good documentation](https://github.com/protocolbuffers/protobuf/tree/master/cmake). Here's a better attempt:

### Installing protobuf with CMake / VS

* clone protobuf v3.12.1
* mkdir protobuf/cmake/build
* run configure in cmake
* set `protobuf_BUILD_SHARED_LIBS` to `ON` or `True`
* set CMAKE_INSTALL_PREFIX to something meaningful
* run generate in cmake
* open visual studio and compile in **Release** mode
* build the `INSTALL` target

Protobuf actually creates a CMake target (`protobuf::libprotobuf`), such that we can just use `target_link_libraries` on our side to link against it. This is very convenient for us.

### Using protobuf in the test app

* when running cmake, set `Protobuf_DIR` to your `${CMAKE_INSTALL_PREFIX}` from above
* add `${CMAKE_INSTALL_PREFIX}/bin` to your `PATH`

## Not Windows

This is probably a lot simpler, and the [original protobuf documentation](https://github.com/protocolbuffers/protobuf/tree/master/src) should cover your case. If not, please expand this section.
