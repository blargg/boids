let
 moz_overlay = import (builtins.fetchTarball https://github.com/mozilla/nixpkgs-mozilla/archive/master.tar.gz);
 nixpkgs = import <nixpkgs> { overlays = [ moz_overlay ]; };
 rust = (nixpkgs.rustChannelOf {
   date = "2020-09-10";
   channel = "nightly";
 }).rust;
in
  with nixpkgs;
  mkShell {
    buildInputs = [
      cargo
      wasm-pack
      nodejs
      (rust.override {
        targets = ["wasm32-unknown-unknown"];
      })
    ];
  }
