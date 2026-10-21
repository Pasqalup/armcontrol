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

## JNI bridge

The optional `modeled_jni` target wraps the existing C API in
`src/jni/modeled_jni.cpp` and does not change the `modeled_c` library or its
C ABI. Java loads it with `System.loadLibrary("modeled_jni")`.

The JNI class contract is
`org.firstinspires.ftc.teamcode.modeled.NativePipeline`; a reference source
file lives in `jni/java/`. Configuration arrays are laid out as follows:

- `ik`: `[l1, l2]`
- `tracking`: `[maxDistance, velocityAlpha, radiusWeight, maxSpeed]`
- `trackingInts`: `[maxMissedFrames, confirmationHits, useKalman]`
- `pipeline`: `[lookaheadTime, trueRadius, armBaseHeight]`
- `camera`: `[px,py,pz, fx,fy,fz, ux,uy,uz, focalLength, principalX, principalY]`

`update(double[] detections, double timestamp, double[] out)` accepts flat
`x,y,radius` triples and fills `out` with `[errorCode, theta1, theta2, theta3]`.
A non-zero `errorCode` describes the pipeline result; JNI-level failures throw
Java exceptions.

Build and smoke-test the JNI library on the host with JDK 11 or newer:

```sh
build-jni.bat
```

This configures `build/host-jni`, builds `modeled_jni.dll`, compiles the
reference Java class plus `jni/test/SmokeTest.java`, and runs the smoke test.

Build Android libraries with the NDK (GNU Make must be on `PATH`):

```sh
set ANDROID_NDK_HOME=C:\path\to\ndk
build-android.bat
```

The script builds `armeabi-v7a`, `arm64-v8a`, and `x86_64` using the CMake
presets in `CMakePresets.json`, then copies `libmodeled_jni.so` into
`dist/jniLibs/<abi>/`. Copy that directory into the Android project's
`src/main/jniLibs/` and add the reference Java class to the TeamCode sources.
