# Vendored OBJ dependencies

`tiny_obj_loader.h` is a snapshot of tinyobjloader's `release`
branch, retrieved on 2026-10-19. Its license is in `LICENSE` and in the header.

- Upstream: https://github.com/tinyobjloader/tinyobjloader
- Source: https://raw.githubusercontent.com/tinyobjloader/tinyobjloader/release/tiny_obj_loader.h

Two small local fixes apply to the standard stream-loading path used here:

1. With Earcut enabled, also use Earcut for quads. The upstream shortcut picks
   the shorter diagonal, which can leave a concave quad with overlapping triangles.
2. Load all filenames on a single `mtllib` line, rather than stopping after the
   first successfully opened library.

Both cases have regression coverage in `tests/objLoaderTests.cpp`.

The wrapper enables `TINYOBJLOADER_USE_MAPBOX_EARCUT`. The unmodified Earcut
header and its ISC license are vendored in `lib/earcut`.

- Upstream: https://github.com/mapbox/earcut.hpp
- Source: https://raw.githubusercontent.com/mapbox/earcut.hpp/master/include/mapbox/earcut.hpp

The implementation macros are defined only in `src/objLoader.cpp`. Public
renderer headers do not expose either dependency. No network download is
needed to build the project.
