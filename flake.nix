{
  description = "Classes for manipulation planning";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    flake-parts.follows = "gepetto/flake-parts";
    nixpkgs.follows = "gepetto/nixpkgs";
    nix-ros-overlay.follows = "gepetto/nix-ros-overlay";
    systems.follows = "gepetto/systems";
    treefmt-nix.follows = "gepetto/treefmt-nix";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = import inputs.systems;
      imports = [
        inputs.gepetto.flakeModule
        {
          gepetto-pkgs.overlays = [
            (final: prev: {
              hpp-core = prev.hpp-core.overrideAttrs (super: {
                patches = super.patches ++ [
                  (final.fetchpatch {
                    url = "https://github.com/humanoid-path-planner/hpp-core/pull/391.patch";
                    hash = "sha256-oqbeUr+Y88hhI8BmCWGVT1ZvI05+YW4TEiEeymaFQ/E=";
                  })
                ];
              });
            })
          ];
        }
      ];
      perSystem =
        {
          lib,
          pkgs,
          self',
          ...
        }:
        {
          packages = {
            default = self'.packages.hpp-manipulation;
            hpp-manipulation = pkgs.hpp-manipulation.overrideAttrs {
              patches = [ ];
              src = lib.fileset.toSource {
                root = ./.;
                fileset = lib.fileset.unions [
                  ./CMakeLists.txt
                  ./doc
                  ./include
                  ./package.xml
                  ./plugins
                  ./src
                  ./tests
                ];
              };
            };
          };
        };
    };
}
