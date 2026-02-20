<a href="https://sumo.dlr.de/docs"><p align="center"><img width=50% src="https://raw.githubusercontent.com/eclipse/sumo/main/docs/web/docs/images/sumo-logo.svg"></p></a>

Eclipse SUMO - Simulation of Urban MObility
===========================================
[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.18406080.svg)](https://doi.org/10.5281/zenodo.18406080)
[![Windows](https://github.com/eclipse-sumo/sumo/actions/workflows/windows.yml/badge.svg)](https://github.com/eclipse-sumo/sumo/actions/workflows/windows.yml)
[![Linux](https://github.com/eclipse-sumo/sumo/actions/workflows/linux.yml/badge.svg)](https://github.com/eclipse-sumo/sumo/actions/workflows/linux.yml)
[![macOS](https://github.com/eclipse-sumo/sumo/actions/workflows/macos.yml/badge.svg)](https://github.com/eclipse-sumo/sumo/actions/workflows/macos.yml)
[![sonarcloud security](https://sonarcloud.io/api/project_badges/measure?project=org.eclipse.sumo&metric=security_rating)](https://sonarcloud.io/summary/overall?id=org.eclipse.sumo)
[![Translation status](https://hosted.weblate.org/widgets/eclipse-sumo/-/svg-badge.svg)](https://hosted.weblate.org/engage/eclipse-sumo/)
![Repo Size](https://img.shields.io/github/repo-size/eclipse/sumo.svg)

What is SUMO
------------

["Simulation of Urban MObility" (SUMO)](https://sumo.dlr.de/) is an open source,
highly portable, microscopic traffic simulation package designed to handle
large road networks and different modes of transport.

<p align="center"><img width=70% src="https://raw.githubusercontent.com/eclipse/sumo/main/docs/web/docs/images/multiple-screenshots.png"></p>

It is mainly developed by employees of the [Institute of Transportation Systems
at the German Aerospace Center](https://www.dlr.de/ts/en/).


Where to get it
---------------

You can download SUMO via our [downloads site](https://sumo.dlr.de/docs/Downloads.html).

As the program is still under development (and is being extended continuously), we advice you to
use the latest sources from our GitHub repository. Using a command line client,
execute the following command:

```
git clone --recursive https://github.com/eclipse-sumo/sumo
```

Contact
-------

To stay informed, we have a mailing list for SUMO, which
[you can subscribe](https://dev.eclipse.org/mailman/listinfo/sumo-user) to.
Messages to the list can be sent to sumo-user@eclipse.org.
SUMO announcements will be made through the sumo-announce@eclipse.org list;
[you can subscribe](https://dev.eclipse.org/mailman/listinfo/sumo-announce) to it as well.
For further contact information, have a look at [this page](https://sumo.dlr.de/docs/Contact.html).


Build and Installation
----------------------

For Windows we provide pre-compiled binaries and CMake files to generate Visual Studio projects.
If you want to develop under Windows, please also clone the dependent libraries using:

```
git clone --recursive https://github.com/DLR-TS/SUMOLibraries
```

If you're using Linux, you should have a look whether your distribution already contains sumo.
There is also a [ppa for ubuntu users](https://launchpad.net/~sumo) and an
[open build service instance](https://build.opensuse.org/project/show/science:dlr).
If you want to build SUMO yourself, the steps for ubuntu are:

```
cd <SUMO_DIR> # please insert the correct directory name here
export SUMO_HOME="$PWD"
sudo apt-get install $(cat build_config/build_req_deb.txt build_config/tools_req_deb.txt)
cmake -B build .
cmake --build build -j$(nproc)
```

For [detailed build instructions, have a look at our Documentation](https://sumo.dlr.de/docs/Developer/Main.html#build_instructions).


Getting started
---------------

To get started with SUMO, take a look at the docs/tutorial and examples directories,
which contain some example networks with routing data and configuration files.
There is also user documentation provided in the docs/ directory and on the
homepage.

Documentation
---------------

- The main documentation is at [sumo.dlr.de/docs](https://sumo.dlr.de/docs). Note that this tracks the [development version](https://sumo.dlr.de/docs/FAQ.html#why_does_sumo_not_behave_as_documented_in_this_wiki).
- A mirror of the main documentation is at [eclipse.dev/sumo/docs/](https://eclipse.dev/sumo/docs/).
- An offline version of the documentation is part of every release and can be accessed via `docs/userdoc/index.html`.

Improving SUMO
--------------

Please use the [GitHub issue tracking tool](https://github.com/eclipse-sumo/sumo/issues) for bugs and requests,
or file them to the sumo-user@eclipse.org list. Before
filing a bug, please consider to check with a current repository checkout
whether the problem has already been fixed.

We welcome patches, pull requests and other contributions! For details see [our contribution guidelines](CONTRIBUTING.md).

We use [Weblate for translating SUMO](https://hosted.weblate.org/projects/eclipse-sumo/). If you
want to add translation strings or a language, see [our contribution guidelines](CONTRIBUTING.md#translating) and
[this page](https://sumo.dlr.de/docs/Developer/Translating.html) for more information.


License
-------

SUMO is licensed under the [Eclipse Public License Version 2](https://eclipse.org/legal/epl-v20.html).
The licenses of the different libraries and supplementary code information are in the
subdirectories and in the [Documentation](https://sumo.dlr.de/docs/Libraries_Licenses.html).
