# Boids
Simple simulation of swarms. Written in rust, and compatible with web assembly.

# Build

If you use nix, `shell.nix` will create a build environment.

```
nix-shell
```
Otherwise install `wasm-pack` and it's dependencies from you package manager.

Build the package.
```
wasm-pack build --release --scope <package scope>
```
This will create a `pkg` directory with the compiled package.


Publish for use in other projects
```
wasm-pack publish --access=public
```
