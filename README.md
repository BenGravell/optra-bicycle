# Optra-bicycle

Optimization of trajectories for bicycle systems.

Provides the following trajectory optimization solvers:

- iterative Linear Quadratic Regulator (iLQR)

## Development

It is critical to use the `gcc` compiler and the `MinGW Makefiles` generator so that the correct debug symbols are generated that VSCode can use.

The `gcc` compiler is specified in the conan profile. The generator, `Ninja` or `MinGW Makefiles`, is specified in the cmake CLI command.

## C++ build (Ninja)

### Release build

```pwsh
conan install . --build=missing -of=build/conan --settings=build_type=Release

cmake -B build/release -S . -G "Ninja" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="build/conan/conan_toolchain.cmake"

cmake --build build/release --config Release
```

#### Use additional compiler flags for potential speedup on your particular machine

```pwsh
conan install . --build=missing -of=build/conan --settings=build_type=Release

cmake -B build/release -S . -G "Ninja" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="build/conan/conan_toolchain.cmake" -DCMAKE_CXX_FLAGS="-march=native -ffast-math" -DCMAKE_C_FLAGS="-march=native -ffast-math"

cmake --build build/release --config Release
```

#### Link Time Optimization

Use LTO for further optimization at link time yielding maximum performance of the compiled executable.
Relies on LLVM / LLD.

```pwsh
conan install . --build=missing -of=build/conan --settings=build_type=Release

cmake -B build/release -S . -G "Ninja" `
    -DCMAKE_BUILD_TYPE=Release `
    -DCMAKE_TOOLCHAIN_FILE="build/conan/conan_toolchain.cmake" `
    -DCMAKE_CXX_FLAGS="-march=native -ffast-math -flto=auto -fuse-ld=""C:/Program Files/LLVM/bin/lld.exe""" `
    -DCMAKE_C_FLAGS="-march=native -ffast-math -flto=auto -fuse-ld=""C:/Program Files/LLVM/bin/lld.exe""" `
    -DCMAKE_EXE_LINKER_FLAGS="-fuse-ld=""C:/Program Files/LLVM/bin/lld.exe""" `
    -DCMAKE_SHARED_LINKER_FLAGS="-fuse-ld=""C:/Program Files/LLVM/bin/lld.exe"""

cmake --build build/release --config Release
```

If you encounter errors during linking, you may need to install LLVM and point to the correct path to lld.exe in the command.

If you encounter errors related to glog, those are currently an unsolved known issue. The only resolution is to skip LTO.

### Debug build

```pwsh
conan install . --build=missing -of=build/conan --settings=build_type=Debug

cmake -B build/debug -S . -G "Ninja" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE="build/conan/conan_toolchain.cmake"

cmake --build build/debug --config Debug
```

### RelWithDebInfo build

NOTE this uses the `release` build folder and will overwrite it!!!

TODO don't do that!

```pwsh
conan install . --build=missing -of=build/conan --settings=build_type=RelWithDebInfo

cmake -B build/release -S . -G "Ninja" -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_TOOLCHAIN_FILE="build/conan/conan_toolchain.cmake" -DCMAKE_CXX_FLAGS="-march=native -ffast-math -g" -DCMAKE_C_FLAGS="-march=native -ffast-math -g"

cmake --build build/release --config Release
```

## C++ build (MinGW)

### Release build

```pwsh
conan install . --build=missing -of=build/conan --settings=build_type=Release

cmake -B build/release -S . -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="build/conan/conan_toolchain.cmake"

cmake --build build/release --config Release -j8
```

### Debug build

```pwsh
conan install . --build=missing -of=build/conan --settings=build_type=Debug

cmake -B build/debug -S . -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE="build/conan/conan_toolchain.cmake" -DCMAKE_CXX_FLAGS="-g -O0" -DCMAKE_C_FLAGS="-g -O0"

cmake --build build/debug --config Debug
```

## Run

```pwsh
build/release/optra_bicycle.exe

build/debug/optra_bicycle.exe
```
