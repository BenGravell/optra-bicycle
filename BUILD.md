# BUILD

## Release

```bash
conan install . --build=missing -of=build/conan --settings=build_type=Release
cmake -B build/release -S . -G "Ninja" -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE="build/conan/conan_toolchain.cmake" -DCMAKE_CXX_FLAGS="-march=native -ffast-math -flto=auto" -DCMAKE_C_FLAGS="-march=native -ffast-math -flto=auto"
cmake --build build/release --config Release
```

## Debug

```bash
conan install . --build=missing -of=build/conan --settings=build_type=Debug
cmake -B build/debug -S . -G "Ninja" -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE="build/conan/conan_toolchain.cmake"
cmake --build build/debug --config Debug
```

## Run

```pwsh
build/release/optra_bicycle

build/debug/optra_bicycle
```
