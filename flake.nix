{
  inputs = {
    utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "nixpkgs/nixos-24.11";
    rust-overlay.url = "github:oxalica/rust-overlay/snapshot/2024-08-01";
    rust-overlay.inputs.nixpkgs.follows = "nixpkgs";
  };
  outputs = { nixpkgs, utils, rust-overlay, ... }: utils.lib.eachDefaultSystem (system:
    let
      overlays = [ (import rust-overlay) ];
      pkgs = import nixpkgs { inherit system overlays; };
    in
    {
      formatter = pkgs.nixpkgs-fmt;
      devShell = pkgs.mkShell {
        buildInputs = with pkgs; [
          pkgs.pkg-config
          openssl
          flip-link
          (rust-bin.nightly."2022-08-29".default.override {
            extensions = [
              "clippy"
              "rust-src"
              #"rust-analyzer"
            ];
            targets = [
              "thumbv6m-none-eabi"
              "thumbv8m.main-none-eabihf"
              "riscv32imac-unknown-none-elf"
              "x86_64-unknown-linux-gnu"
            ];
          })
        ];
      };
    }
  );
}
