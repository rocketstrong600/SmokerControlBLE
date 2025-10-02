{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, utils}:
    utils.lib.eachDefaultSystem(system:
      let
        pkgs = import nixpkgs { inherit system; };
        python-with-pip = pkgs.python3.withPackages (p: with p; [pip]);
        in {
          devShell = with pkgs; mkShell {
            buildInputs = [ platformio esptool python-with-pip clang clang-tools ];
            shellHook = ''
              export PLATFORMIO_CORE_DIR=$PWD/.platformio
              export PIP_TARGET=$PWD/.platformio/packages/
              exec fish
            '';
          };
        }
    );
}
