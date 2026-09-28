# Exodus

Mesh unsubdivide, material combiner, and optimizer. With 3dsmax plugin and standalone written in C++

## Standalone Use

```
exodus scene.fbx --info
exodus scene.fbx -o out/dir [options]
```

`--info` lists meshes, materials, the maps bound to each and the UDIM tiles they use.

| option | |
|---|---|
| `--size N` / `WxH` | atlas size; default is the smallest power of two that fits |
| `--padding N` | minimum gap between islands in atlas pixels (default 8) |
| `--scale <material-glob>=S` | S = 1, 0.5, 0.25 … (2× box downsample per step); 1 is bit-exact |
| `--exclude <material-glob>` | drop a material's faces |
| `--tiles LIST` | pack only these UDIM tiles, e.g. `1001-1004,1010`; faces in other tiles are left out |
| `--map <channel>:<glob>=<file>` | bind or replace a map (for maps the FBX does not reference) |
| `--kind <channel>=normal-gl\|normal-dx` | rotate tangent-space normals with their islands |
| `--fill <channel>[:<glob>]=r,g,b[,a]` | value for islands whose material has no map in that channel |
| `--no-rotate` | keep islands upright |
| `--unsubdivide N` | reverse N Catmull-Clark levels (runs before decimation): exact where possible, Blender's un-subdivide otherwise |
| `--quad-dominant` | with `--unsubdivide`: approximate by grouping quads into blocks instead (mostly quads, reduces more) |
| `--unsubdivide-exact` | refuse meshes that are not a subdivision result instead of approximating them |
| `--no-symmetry` | with `--unsubdivide`: let the approximation break the symmetry of mirror-symmetric meshes |
| `--cage` | with `--unsubdivide`: recover the control cage positions by least squares instead of keeping the smoothed ones |
| `--subdivide N` | apply N Catmull-Clark levels to positions, UVs and normals (utility; `--subdivide 1 --unsubdivide 1 --cage` is an exact round trip) |
| `--tris N` | decimate the whole scene to about N triangles |
| `--decimate R` | decimate to fraction R of the triangles (e.g. 0.25) |
| `--max-error E` | decimation error limit relative to model size (0.01 = 1%); alone it decimates as far as E allows |
| `--no-decimate <material-glob>` | leave a material at full detail (it still counts toward `--tris`) |
| `--lock-border` | keep open mesh borders fixed |
| `--symmetric-decimate` | decimate mirror-symmetric meshes symmetrically: one half is simplified, the other rebuilt as its mirror image |
| `--normal-weight W` / `--uv-weight W` | decimation attribute weights (defaults 0.5 / `1/sqrt(mean UV triangle area)`) |


## Build

```
cmake -S . -B build
cmake --build build --config Release
```
