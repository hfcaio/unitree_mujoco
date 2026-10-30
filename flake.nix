{
  description = "unitree_mujoco dev shell (C++ simulator + Python simulator)";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
  inputs.flake-utils.url = "github:numtide/flake-utils";

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachSystem [ "x86_64-linux" ] (system:
      let
        pkgs = import nixpkgs { inherit system; };

        # Prebuilt MuJoCo release (matches the version in the readme). It ships
        # include/, lib/ and simulate/, which simulate/CMakeLists.txt expects at
        # simulate/mujoco.
        mujoco-release = pkgs.stdenv.mkDerivation rec {
          pname = "mujoco-release";
          version = "3.3.6";
          src = pkgs.fetchurl {
            url = "https://github.com/google-deepmind/mujoco/releases/download/${version}/mujoco-${version}-linux-x86_64.tar.gz";
            hash = "sha256-BJIEFykBr60lEHA4Wmut9G15Xr5HQD0JP4RpVX7uq1o=";
          };
          nativeBuildInputs = [ pkgs.autoPatchelfHook ];
          buildInputs = [ pkgs.stdenv.cc.cc.lib ];
          installPhase = ''
            mkdir -p $out
            cp -r include lib simulate $out/
          '';
        };

        # nixpkgs ships 2.0.2, which lacks unitree/dds_wrapper used by simulate/src
        unitree-sdk2 = pkgs.unitree-sdk2.overrideAttrs (_: {
          version = "unstable-2026-10";
          src = pkgs.fetchFromGitHub {
            owner = "unitreerobotics";
            repo = "unitree_sdk2";
            rev = "d1abf001220d3e5f59e00a0c4aaa595e00089a4a";
            hash = "sha256-teiFLeVeelhZNl3Ot10MYRq2ZbMlPW5E1gkZ0plkkY8=";
          };
        });

        # unitree_sdk2py pins cyclonedds==0.10.2, whose binding needs a 0.10.x C lib.
        # 0.10.5: 0.10.2 aborts (fortify snprintf overflow) when given an interface.
        cyclonedds = pkgs.cyclonedds.overrideAttrs (_: {
          version = "0.10.5";
          patches = [ ];
          postInstall = ''
            sed -i 's|''${prefix}//|/|' $out/lib/pkgconfig/CycloneDDS.pc
          '';
          src = pkgs.fetchFromGitHub {
            owner = "eclipse-cyclonedds";
            repo = "cyclonedds";
            rev = "0.10.5";
            hash = "sha256-MQVUZ7PkxauoPpfxlhhAtsKztMe9tcZOpOzshuz/eb8=";
          };
        });

        # Python binding 0.10.2 to match; patch one private CPython call removed in 3.13
        cyclonedds-python = ps: ps.buildPythonPackage rec {
          pname = "cyclonedds";
          version = "0.10.2";
          pyproject = true;
          src = pkgs.fetchFromGitHub {
            owner = "eclipse-cyclonedds";
            repo = "cyclonedds-python";
            tag = version;
            hash = "sha256-EnvNWIYDviMPANanUqI7Uk8lQBjXcN9DVb9OZlAelrM=";
          };
          postPatch = ''
            substituteInPlace clayer/pysertype.c --replace-fail _Py_IsFinalizing Py_IsFinalizing
          '';
          build-system = [ ps.setuptools ps.wheel ];
          buildInputs = [ cyclonedds ];
          dependencies = [ ps.rich-click ];
          env.CYCLONEDDS_HOME = "${cyclonedds}";
          env.NIX_CFLAGS_COMPILE = "-Wno-error=discarded-qualifiers";
          pythonImportsCheck = [ "cyclonedds.domain" ];
        };

        # Not on PyPI
        unitree-sdk2py = ps: ps.buildPythonPackage {
          pname = "unitree_sdk2py";
          version = "1.0.1-unstable-2026-09-21";
          pyproject = true;
          src = pkgs.fetchFromGitHub {
            owner = "unitreerobotics";
            repo = "unitree_sdk2_python";
            rev = "814556d15970dd2ecf1c9984e845ca02ab07e206";
            hash = "sha256-B21qXh2x7cFr3cZvYQqGoEZ1xpYDPBlnH6dMnhZ72X8=";
          };
          build-system = [ ps.setuptools ];
          dependencies = [ (cyclonedds-python ps) ps.numpy ps.opencv4 ];
          pythonRemoveDeps = [ "opencv-python" ]; # provided by opencv4
          pythonImportsCheck = [ "unitree_sdk2py.core.channel" ];
        };

        # 3.13: cyclonedds 0.10.2's IDL classes break under 3.14's lazy annotations
        python = pkgs.python313.withPackages (ps: [ ps.mujoco ps.pygame ps.numpy ps.opencv4 (unitree-sdk2py ps) ]);
      in
      {
        devShells.default = pkgs.mkShell {
          packages = with pkgs; [
            # C++
            cmake
            gnumake
            pkg-config
            gcc
            gdb
            boost
            yaml-cpp
            spdlog
            fmt
            eigen
            glfw
            unitree-sdk2
            mujoco-release

            # Python (mujoco, pygame, opencv, unitree_sdk2py)
            python
            cyclonedds
          ];

          # GL / windowing libs loaded at runtime by glfw and the mujoco viewer
          LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath (with pkgs; [
            libGL
            libx11
            libxcursor
            libxi
            libxinerama
            libxrandr
            libxkbcommon
            wayland
            stdenv.cc.cc.lib
            zlib
            glfw
            mujoco-release
          ]);

          CYCLONEDDS_HOME = "${cyclonedds}";

          # same DDS domain/interface as simulate/config.yaml (domain_id 1,
          # interface "lo"), so `ros2 topic list` in this shell sees the sim.
          # `lo` has no MULTICAST flag by default: `sudo ip link set lo multicast on`
          ROS_DOMAIN_ID = "1";
          RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp";
          CYCLONEDDS_URI = ''
            <CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="true"/></Interfaces><AllowMulticast>true</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer address="localhost"/></Peers></Discovery></Domain></CycloneDDS>
          '';
          CMAKE_PREFIX_PATH = "${unitree-sdk2}/lib/cmake";

          shellHook = ''
            root=$PWD

            # simulate/mujoco -> prebuilt release (gitignored)
            if [ -d "$root/simulate" ] && [ "$(readlink "$root/simulate/mujoco")" != "${mujoco-release}" ]; then
              ln -sfn ${mujoco-release} "$root/simulate/mujoco"
            fi
          '';
        };
      });
}
