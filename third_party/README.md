# third_party

Optional external dependencies for the legacy `ray_voxel` demo. The main
`embedded_voxel_mapper` and Python bindings do **not** require anything from
this directory.

To enable the `ray_voxel_legacy` build target, vendor these two header-only
libraries here:

```bash
mkdir -p third_party/nlohmann third_party
# nlohmann/json (single-header)
curl -L -o third_party/nlohmann/json.hpp \
    https://github.com/nlohmann/json/releases/latest/download/json.hpp
# stb_image.h (single-header)
curl -L -o third_party/stb_image.h \
    https://raw.githubusercontent.com/nothings/stb/master/stb_image.h
```

Then either:

```bash
make legacy            # via Makefile
# or
cmake -B build . && cmake --build build   # CMake auto-detects these headers
```

If the headers are missing, both build systems silently skip the legacy
target and continue with the core `embedded_voxel_mapper`.

## Licenses

- `nlohmann/json` — MIT License (https://github.com/nlohmann/json)
- `stb` — Public Domain / MIT License (https://github.com/nothings/stb)