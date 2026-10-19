# modeled_c

A C++17 implementation of the inverse-kinematics and object-tracking
primitives in `py/`. The Python files are reference implementations and are
not modified by the C++ build.

## Configure and build

From the project root:

```sh
cmake -S . -B build
cmake --build build
```

The build produces the `modeled_c` shared library. Define the public API in
`src/main.hpp` and its implementation in `src/main.cpp`; CMake compiles those
alongside the IK and object-tracking implementations.

The C++ APIs are declared in [`src/ik.hpp`](src/ik.hpp) and
[`src/object_tracking.hpp`](src/object_tracking.hpp). `IK::solve` returns
base, shoulder, and elbow angles in radians. `ObjectTracker` supports greedy
one-to-one matching with either smoothed constant velocity or Kalman-filtered
tracks.

The project uses CMake's `target_compile_features` to require C++17 and
disables compiler-specific C++ extensions.
