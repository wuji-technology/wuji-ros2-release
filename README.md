# wuji_sdk_vendor

`wuji_sdk_vendor` installs the Wuji SDK C header, shared library, SDK license, and imported CMake target for ROS2 packages.

## Build Behavior

The package supports x86_64 and aarch64 Linux targets. By default, CMake selects the matching archive from a pinned Wuji SDK GitHub Release, checks its digest and ELF architecture, and checks that the library exports `wuji_version`.

Use an extracted SDK C release for an offline or pre-fetched build:

```bash
colcon build \
  --packages-select wuji_sdk_vendor \
  --cmake-args -DWUJI_SDK_ROOT=<sdk-root>
```

The SDK root must contain:

```text
<sdk-root>/
├── include/wuji_sdk.h
├── lib/libwuji_sdk_c.so
└── LICENSE
```

## CMake Consumer

After installation, downstream CMake projects can use:

```cmake
find_package(wuji_sdk_vendor REQUIRED)
target_link_libraries(my_target PRIVATE wuji_sdk_vendor::wuji_sdk_c)
```

The installed shared library keeps its Wuji SDK license. The ROS package itself uses the repository MIT license.

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).
