# QuickHull bundled for Choreonoid

This directory holds a subset of the upstream QuickHull implementation by
Antti Kuukka, extracted so that Choreonoid can compute tight bounding boxes
of Body models without pulling in an external convex-hull dependency.

- Upstream repository: <https://github.com/akuukka/quickhull>
- Upstream license: 100% Public Domain (see `LICENSE` in this directory)

Only the files strictly required to build QuickHull as a C++ library are
kept here. The upstream repository additionally ships a `Tests/` directory,
a `main.cpp`, a `README.md`, and a `CMakeLists.txt`; none of these are
retained.

## Files retained from upstream

- `QuickHull.hpp`
- `QuickHull.cpp`
- `ConvexHull.hpp`
- `HalfEdgeMesh.hpp`
- `MathUtils.hpp`
- `Structs/Mesh.hpp`
- `Structs/Plane.hpp`
- `Structs/Pool.hpp`
- `Structs/Ray.hpp`
- `Structs/Vector3.hpp`
- `Structs/VertexDataSource.hpp`

## Files added for Choreonoid

- `CMakeLists.txt` — builds `QuickHull.cpp` as a static library `quickhull`
  with position-independent code (so it can be linked into Choreonoid's
  shared libraries) and hidden symbol visibility (so its classes are not
  re-exported through the ABI of any consumer shared library).
- `LICENSE` — records the upstream public-domain statement, since the
  upstream repository does not ship a standalone license file.
- `CHOREONOID_NOTES.md` — this file.

## How to update to a newer upstream

1. `git clone https://github.com/akuukka/quickhull.git /tmp/qh` (or
   check out the tag/commit you want).
2. Replace the retained files listed above with the fresh copies from
   `/tmp/qh` and `/tmp/qh/Structs`. Do **not** copy `Tests/`, `main.cpp`,
   `README.md`, or the upstream `CMakeLists.txt`.
3. If upstream now ships a standalone license file, replace the local
   `LICENSE` with it. Otherwise leave `LICENSE` as-is - it captures the
   public-domain statement from the upstream README and source headers.
4. Rebuild Choreonoid. The API used by
   `src/Body/BodyBoundingBox.cpp` is:
   ```
   quickhull::QuickHull<double>::getConvexHull(
       const std::vector<quickhull::Vector3<double>>&, bool, bool);
   ```
   and `ConvexHull::getVertexBuffer()`. If either of these change upstream,
   update `BodyBoundingBox.cpp` accordingly.
