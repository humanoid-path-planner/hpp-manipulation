{
  description = "Classes for manipulation planning";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    hpp-core = {
      url = "github:humanoid-path-planner/hpp-core";
      inputs.gepetto.follows = "gepetto";
    };
  };

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        overlays = [ inputs.hpp-core.overlays.flakoboros ];
        overrideAttrs.hpp-manipulation = {
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
      }
    );
}
