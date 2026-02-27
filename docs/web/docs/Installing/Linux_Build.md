---
title: Linux Build
---

This document describes how to install SUMO on Linux from sources. If
you don't want to **extend** SUMO, but merely **use** it, you might want
to [download one of our pre-built binary
packages](index.md) instead.

To be able to run SUMO on Linux, just follow these steps:

1.  Install all of the required tools and libraries
2.  Get the source code
3.  Build the SUMO binaries

For ubuntu this boils down to

```
sudo apt-get install git
git clone --recursive https://github.com/eclipse-sumo/sumo
cd sumo
export SUMO_HOME="$PWD"
sudo apt-get install $(cat build_config/build_req_deb.txt build_config/tools_req_deb.txt)
cmake -B build .
cmake --build build -j$(nproc)
```

!!! note
    Do not build in an active Anaconda environment! The libraries have sometimes different versions than Anaconda which will break the build or the executables.

Each of these steps is described in more detail and with possible
alternatives below.

## Installing required tools and libraries

- For the build infrastructure you will need cmake together with a moderately
  recent g++ (7 will do) or clang++ (or any other C++17 enabled compiler).
- It is possible to build SUMO with older compilers too (at least gcc 4.8) but
  it is not recommended since some features will be disabled.
- The library Xerces-C is always needed. To use
  [sumo-gui](../sumo-gui.md) you also need Fox Toolkit in version
  1.6.x. It is highly recommended to also install Proj to have support
  for geo-conversion and referencing. Another common requirement is
  network import from shapefile (arcgis). This requires the GDAL
  library. To compile you will need the devel versions of all packages.
  For openSUSE this means installing libxerces-c-devel, libproj-devel,
  libgdal-devel, and fox16-devel. For ubuntu the call is above.
  The installation of swig, python3-dev and the jdk enables also the build of libsumo
  while eigen3 is necessary for the overheadwire model.
  There are some outdated [platform specific
  and manual build instructions for the
  libraries](Linux_Build_Libraries.md)
- Optionally you may want to add
  - ccache (to speed up builds)
  - ffmpeg-devel (for video output),
  - libOpenSceneGraph-devel (for the experimental 3D GUI),
  - gtest (for [unit testing](../Developer/Unit_Tests.md), do not use 1.13 or later)
  - gettext (for internationalization)
  - texttest, xvfb  and tkdiff (for the acceptance tests, include python3-gobject for the texttest GUI)
  - flake, astyle and autopep for style checking
  - see also further dependencies [for GUI testing](../Developer/GUI_Testing.md)

The package names above are for openSUSE, for ubuntu the call to get the most important optional libraries and tools is:

```
sudo apt-get install ccache libavformat-dev libswscale-dev libopenscenegraph-dev python3-pip python3-build
sudo apt-get install libgtest-dev gettext tkdiff xvfb flake8 astyle python3-autopep8 python3-gi-cairo gir1.2-gtk-3.0
sudo apt-get install python3-pyproj python3-rtree python3-pandas python3-pulp python3-ezdxf
python3 -m pip install texttest
```

For the Python tools there are some more requirements depending on which tools you want to use. If you want to install
everything using pip do `python3 -m pip install -r tools/requirements.txt -r tools/req_dev.txt`.

### Parquet support

see https://arrow.apache.org/install/
```
wget https://packages.apache.org/artifactory/arrow/$(lsb_release --id --short | tr 'A-Z' 'a-z')/apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb
sudo apt install -y -V ./apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb
sudo apt update
sudo apt install -y -V libarrow-dev libparquet-dev
```

## Getting the source code

For the correct setting of SUMO_HOME you have to remember the correct
path, where you build your SUMO, the SUMO build path. This path is shown
with pwd at the end of getting the source code. If you want to develop
actively on sumo we strongly recommend to use the git repository. Please
build the tarball version if you need for some reason a specific version
of sumo.

### repository checkout (recommended)

The following commands should be issued:

```
git clone --recursive https://github.com/eclipse-sumo/sumo
cd sumo
git fetch origin refs/replace/*:refs/replace/*
pwd
```

The additional fetch of the replacements is necessary to get a full
local project history.

### release version or nightly tarball

Download
[sumo-src-{{Version}}.tar.gz](https://sumo.dlr.de/releases/{{Version}}/sumo-src-{{Version}}.tar.gz) or <https://sumo.dlr.de/daily/sumo-src-git.tar.gz>

```
tar xzf sumo-src-<version>.tar.gz
cd sumo-<version>/
pwd
```

## Definition of SUMO_HOME

Before compiling is advisable to
define the environment variable SUMO_HOME. SUMO_HOME must be set to
the SUMO build path from the previous step. Assuming that you placed
SUMO in the folder "*/home/<user\>/sumo-<version\>*", if you want to
define only for the current session, type in the console

```
export SUMO_HOME="/home/<user>/sumo-<version>"
```

If you want to define for all sessions (i.e. for every time that you run
your Linux distribution), go to your HOME folder, and find one of the
next three files (depending of your Linux distribution):
**.bash_profile**, **.bash_login** or **.profile** (Note that these
files can be hidden). Then edit the file, add the line from above at the
end and restart your session.

You can check that SUMO_HOME was successfully set if you type

```
echo $SUMO_HOME
```

and console shows "/home/<user\>/sumo-<version\>"

## Installing Python packages for the tools

Calling the tools from netedit requires a list of Python packages to generate templates during compilation.
Many of them might be available with the package manager of your distribution and most of the time we prefer
to use those. For ubuntu this currently means, you should first do

```
sudo apt-get install python3-pyproj python3-rtree python3-pandas flake8 python3-autopep8 python3-pulp python3-ezdxf
```

and then install the remaining parts using pip:

```
python3 -m pip install -r tools/requirements.txt -r tools/req_dev.txt
```

The pip installation will ensure that all libraries are there, so it is safe to skip the first `apt-get` step.
If you need information about the minimum required versions of the packages read them directly
from the [requirements.txt](https://github.com/eclipse-sumo/sumo/blob/main/tools/requirements.txt). Be aware that
the minimum versions in the requirements file just reflect our current test server setup, so you might also get away with earlier versions.

If you want to reproduce our test server setup exactly, then use the versions in
[req_test_server.txt](https://github.com/eclipse-sumo/sumo/blob/main/tools/req_test_server.txt)
Unfortunately some old pip packages are incomplete, so this requires `sudo apt install libspatialindex-dev`.

## Building the SUMO binaries with cmake

To build with cmake version 3.5 or higher is required.

Create a build folder for cmake (in the SUMO root folder)
and configure SUMO with the full set of available options like GDAL and
OpenSceneGraph support (if the libraries are installed):

```
cmake -B build .
```

to build the debug version just use

```
cmake -D CMAKE_BUILD_TYPE=Debug -B build .
```

!!! note
    On some platforms the required cmake executable is called *cmake3*.

Other useful cmake configuration options:

- `-D PROFILING=ON` enable profiling instrumentation for gprof (gcc build only)
- `-D COVERAGE=ON` enable coverage instrumentation for lcov (gcc build only)
- `-D CHECK_OPTIONAL_LIBS=OFF` disable all optional libraries (only
  include EPL compatible licensed code)
- `-D CMAKE_BUILD_TYPE=RelWithDebInfo` enable debug symbols for
  debugging the release build or using a different profiler
- `-D PROJ_LIBRARY=` disable PROJ
- `-D FOX_CONFIG=` disable FOX toolkit (GUI and multithreading)
- `-D PYTHON_EXECUTABLE=/usr/bin/python3` select a different python version (also for libsumo / libtraci)
- `-D MVN_EXECUTABLE=` disable maven packaging (especially useful if you have no network connection)
- `-D ENABLE_CS_BINDINGS=ON` enable C# bindings when compiling libsumo / libtraci

After this is finished, run

```
cmake --build build -j $(nproc)
```

The `nproc` command gives you the number of logical cores on your
computer, so that make will start parallel build jobs which makes the
build a lot faster. If `nproc` is not available on your system, insert a
fixed number here or leave the option out. You may also try

```
cmake --build build -j $(grep -c ^processor /proc/cpuinfo)
```

## Building with clang

If you want to use a different compiler (just for the fun of it or
because it has additional features) you can enable it at configure time.
Our current clang configuration for additional static code checking
enables the following CXXFLAGS:

```
-stdlib=libstdc++ -fsanitize=undefined,address,integer -fno-omit-frame-pointer -fsanitize-blacklist=$SUMO_HOME/build_config/clang_sanitize_blacklist.txt
```

You may of course leave out all the sanitizer-checks you don't want but
the stdlib option has to be set. The blacklist filters out a known bug
in the cstdlib. For details see the clang documentation.

The current CMake configuration already contains this for the debug
build, so for building with CMake and clang just change to your repository base dir and use

```
CXX=clang++ cmake -D CMAKE_BUILD_TYPE=Debug -B build .
cmake --build build -j $(nproc)
```

The clang-debug-build will detect memory leaks (among other things)
If the errors are reported with cryptic hexadecimal numbers as

```
Indirect leak of 72 byte(s) in 1 object(s) allocated from:
    #0 0xa4ee2d  (.../sumo/bin/netconvertD+0xa4ee2d)
```

set the following environment variable to point to the llvm-symbolizer executable:
`export ASAN_SYMBOLIZER_PATH=/usr/bin/llvm-symbolizer` before running the executable.

Please note that the undefined behavior checker is very sensitive and will report
some false negatives also in system libraries. It is recommended to use
`export UBSAN_OPTIONS=suppressions=$SUMO_HOME/build_config/clang_ubsan_suppressions.txt`
before calling the executable.

## Building a minimal feature set

Many of the project features are optional. To test a minimal setup, the following build configuration can be used:

```
cmake -DFOX_CONFIG= -DPROJ_LIBRARY= -DCHECK_OPTIONAL_LIBS=OFF -DFMI=OFF -B build.
```

## Installing the SUMO binaries

This (optional) step will copy the SUMO binaries to another path, so
that you can delete all source and intermediate files afterwards. If you
do not want (or need) to do that, you can simply skip this step and run
SUMO from the bin subfolder (bin/sumo-gui and bin/sumo).

If you want to install the SUMO binaries into your system, run
```
sudo cmake --install build
```

You have to adjust your SUMO_HOME variable to the install dir (usually
/usr/local/share/sumo)
```
export SUMO_HOME=/usr/local/share/sumo
```

## Uninstalling

CMake provides no `make uninstall` so if you ever want to uninstall, run
```
sudo xargs rm < install_manifest.txt
```
from the same folder you ran `make install`. This will leave some empty
directories, so if you want to remove them as well, double check that
$SUMO_HOME points to the right directory (see above) and run
```
sudo rm -r $SUMO_HOME
```

## Building Python wheels for sumolib, traci, libsumo and eclipse-sumo

If you want to distribute sumolib, traci, libsumo or the whole SUMO package
as wheels, you can build those directly from the source tree. Please be aware
that nightly builds of those packages are also available on
https://sumo.dlr.de/wheels/ (built on the SUMO servers) and
https://sumo.dlr.de/ciwheels/ (built on GitHub CI, has usually more platforms).

If you still want to build them all yourself, maybe because you need a custom configuration,
the easiest way is to call tools/build_config/build_wheels.sh but be aware that this
will invoke [cibuildwheel](https://cibuildwheel.pypa.io/) which in turn fires up docker
and may take some time and space to build for multiple python versions.

If you only need a single wheel the pattern is usually:
```
tools/build_config/version.py --pep440 build_config/pyproject/sumolib.toml pyproject.toml
python -m build --wheel
```
where the first line patches the pyproject.toml with the current version
(you can replace sumolib here by traci, libsumo etc.)
and saves it in the SUMO root dir and the second call runs the build.

You will need a recent version of pip (>=22) and [python-build](https://build.pypa.io/)
for this to work. If you cannot build in an isolated environment
(maybe because you do not have internet access) you will also need scikit-build-core
(for sumo-data, eclipse-sumo and libsumo) and/or hatchling (sumolib and traci).
The traci and the sumolib wheel are platform and Python version independent,
sumo-data and eclipse-sumo depend on the platform but not on the Python version
while libsumo depends on the exact platform and Python you built it with.

## (Frequent) Rebuilds

If you did a repository clone you can simply update it by doing `git pull`
from inside the SUMO_HOME folder. Then change to the build directory and run
`make -j $(nproc)` again.

If your underlying system changed (updated libraries) or you experience other
build problems please try a clean build first by removing the build directory (or at
least the CMakeCache.txt) and running cmake and make again before reporting a bug.

If you find yourself building very often after minor changes, consider installing
ccache and run cmake again. It will be picked up automatically and can dramatically
improve build speed.

## How to build JuPedSim and then build SUMO with JuPedSim

In this section, you will learn how to build the latest version of the pedestrian simulator JuPedSim and how to compile SUMO with this latest version of JuPedSim. First of all, clone the JuPedSim repository:

``` bash
git clone https://github.com/PedestrianDynamics/jupedsim
```
Note that this will clone the full repository, including the latest version of JuPedSim. **We strongly recommend to build the latest release of JuPedSim (not the master branch), which is officially supported by SUMO.** You can consult the [JuPedSim build procedure](https://github.com/PedestrianDynamics/jupedsim#readme); hereafter we propose a similar procedure. First check which is the [latest release](https://github.com/PedestrianDynamics/jupedsim/releases) then in the cloned directory checkout to the latest release and do a regular cmake build. For example, for JuPedSim release v1.3.1, you would need to type:

``` bash
cd jupedsim
git checkout v1.3.1
cmake -B build .
cmake --build build
sudo cmake --install build
```

Now you should make sure GEOS is installed (`sudo apt-get install libgeos-dev`) and
continue with the [standard build procedure above](#building_the_sumo_binaries_with_cmake).

### Tweaking the JuPedSim build

If you do not want to install jupedsim into your system, you can specify an alternative install directory like this:

``` bash
cmake -B build -DCMAKE_INSTALL_PREFIX=$PWD/../jupedsim-install .
cmake --build build
cmake --install build
```

This will install jupedsim in the directory `jupedsim-install` right beside the checkout. If you installed jupedsim
into the system or in a `jupedsim-install` directory beside sumo, the standard cmake call of SUMO will find it
automatically.

Please be aware that if you want to install sumo into the system, you also need to install jupedsim into the system.
To tweak or debug the jupedsim build you can also change the configuration to Debug (with `-DCMAKE_BUILD_TYPE=Debug`)
and also enable multithreading (with `-j4`) as usual with CMake. If you have different jupedsim versions or choose a
different install path, you can notify CMake where JuPedSim is installed by setting `JUPEDSIM_CUSTOMDIR` when calling CMake.

For further remarks on the use of JuPedSim inside SUMO, please consult [the documentation on the model](../Simulation/Pedestrians.md#model_jupedsim).

## Troubleshooting

### Problems with the socket subsystem

Problem:

```
recv ./foreign/tcpip/libtcpip.a(socket.o) (symbol belongs to implicit dependency /usr/lib/libsocket.so.1)
```

Solution:
<https://lists.danga.com/pipermail/memcached/2005-September/001611.html>

### ld cannot find an existing library (Fedora-23)

Problem:

```
/usr/bin/ld: cannot find -lfreetype
ls -lah /usr/lib64/libfreetype*
 lrwxrwxrwx. 1 root root   21 Jul 28 15:54 /usr/lib64/libfreetype.so.6 -> libfreetype.so.6.12.0
 lrwxr-xr-x. 1 root root 689K Jul 28 15:54 /usr/lib64/libfreetype.so.6.12.0
```

Solution: Install the dev package; for fedora:

```
sudo yum install freetype-devel
```

For details see
[stackoverflow](https://stackoverflow.com/questions/335928/ld-cannot-find-an-existing-library)
discussion.

### ld cannot find certain functions in an existing external library

Make sure you don't have an environment like anaconda installed which modifies your library search path.

### Additional notes for Cygwin users

At the moment GUI building is still troublesome. It depends whether you
want to use the X-Server or native Windows GUI. We tried native Windows
GUI only and had to change the installed libFOX-1.4.la such that it
contains

```
dependency_libs=' -lgdi32 -lglaux -ldl -lcomctl32 -lwsock32 -lwinspool -lmpr
-lpthread -lpng /usr/lib/libtiff.la /usr/lib/libjpeg.la -lz -lbz2 -lopengl32 -lglu32'
```

Your mileage may vary.
