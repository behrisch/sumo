---
title: Libsumo
---

# Libsumo

The main way to interact with a running simulation is
[TraCI](TraCI.md) which gives the complete flexibility of doing
cross-platform, cross-language, and networked interaction with
[sumo](sumo.md) acting as a server. One major drawback is the
communication overhead due to the protocol and the socket communication.
To have a more efficient coupling without the need for socket
communication, the TraCI API is provided as a C++ library with the
following properties:

- C++ interface based on static functions and a few simple wrapper
  classes for results which can be linked directly to the client code
- Function signatures similar to [TraCI](TraCI.md)
- Pre-built language bindings for Java and Python (using
  [SWIG](https://www.swig.org/))
- Support for other programming languages via
  [SWIG](https://www.swig.org/)

# Limitations

The following things currently do not work (or work differently than with the TraCI Python client):

- running with [sumo-gui](sumo-gui.md) does not work on Windows and is still highly experimental on other platforms
- subscriptions that require additional arguments (except for *vehicle.getLeader*)
- stricter type checking
  - the TraCI client sometimes accepts any iterable object where Libsumo wants a list
  - TraCI client may accept any object where Libsumo needs a boolean value
  - TraCI automatically converts every parameter into a string if a string is needed, Libsumo does not
- using traci.init or traci.connect is not possible (you always need to use traci.start / libsumo.start)
- with traci every TraCIException will generate a message on stderr, Libsumo does not generate this message
- libsumo by itself cannot be used to [connect multiple clients to the simulation](TraCI/Interfacing_TraCI_from_Python.md#controlling_the_same_simulation_from_multiple_clients) (though connecting normal TraCI clients to a libsumo instance is possible)
- running parallel instances of libsumo requires the [multiprocessing module (in python)](https://docs.python.org/3/library/multiprocessing.html)

To avoid the limitations with respect to GUI, multi-clients support, you can also use [libtraci](Libtraci.md). This is a C++ traci client library which is fully API-compatible with libsumo.

# Building and Installing it

The binary windows release already contains the readily compiled libsumo
for C++ and Java. For Python you can install it via `pip install libsumo`.
Only if your platform or language is not supported follow the steps below.

It currently requires cmake and swig being installed together with the
developer packages for Python (and Java if needed), for Windows see
[Windows CMake](Installing/Windows_Build.md#manual_cmake_configuration). You
need to (re-)compile sumo yourself under Windows following the remarks
above, under Linux see [Installing/Linux_Build](Installing/Linux_Build.md)
(it is probably just a matter of calling cmake and
make again if you previously did a build without swig).
For the python bindings you will get a libsumo.py and a
_libsumo.so (or .pyd on Windows). If you place them somewhere on your
python path you should be able to use them as described below.
If you want to enable the experimental C# support, make sure that
you have `ENABLE_CS_BINDINGS` set in your cmake configuration.

!!! note
    Make sure to add `"/your/path/to/sumo/tools"` to the `PYTHONPATH` environment variable.

# Using libsumo

If you want to use the (experimental) GUI then you need to have `sumo-gui`
in your start command instead of `sumo` or define the environment variable
`LIBSUMO_GUI`.

## Python

Make sure you have libsumo installed (`pip install libsumo`).

```py
import libsumo
libsumo.start(["sumo", "-c", "test.sumocfg"])
libsumo.simulationStep()
```

Existing traci scripts can be reused (subject to the [limitations](#limitations) mentioned above) by calling

```py
import libsumo as traci
```

In case you have a lot of scripts you can also set the environment
variable `LIBSUMO_AS_TRACI` to a non empty value which will trigger the
import as above.


## C++

Please note the extra `#define` for enabling GUI code which is not needed if you do not or cannot use the GUI (Windows).

### Example Code (test.cpp)

The example assumes you have the named sumocfg file in your current working directory and the SUMO `bin` directory included
in the `PATH` environmental variable. Otherwise you need to provide the full file path.

```cpp
#include <iostream>
#define HAVE_LIBSUMOGUI  // if you are on Windows or have libsumo compiled yourself without GUI you should remove this line
#include <libsumo/libsumo.h>

using namespace libsumo;

int main(int argc, char* argv[]) {
    Simulation::start({"sumo", "-c", "test.sumocfg"});
    for (int i = 0; i < 5; i++) {
        Simulation::step();
    }
    Simulation::close();
}
```

### compiling on Linux (make sure SUMO_HOME is set and sumo has been built)

```
g++ -o test -std=c++11 -I$SUMO_HOME/src test.cpp -L$SUMO_HOME/bin -lsumocpp
```

### running on Linux

```
LD_LIBRARY_PATH=$SUMO_HOME/bin ./test
```

## Java

You might want to use the available [Maven package](Developer/Maven.md).

### Example Code (Test.java)

```java
import org.eclipse.sumo.libsumo.Simulation;
import org.eclipse.sumo.libsumo.StringVector;

public class Test {
    public static void main(String[] args) {
        if (System.getProperty("os.name").startsWith("Windows")) {
            System.loadLibrary("iconv-2");
            System.loadLibrary("intl-8");
            System.loadLibrary("libcrypto-3-x64");
            System.loadLibrary("libssl-3-x64");
            System.loadLibrary("proj_9");
        }
        System.loadLibrary("libsumojni");
        Simulation.start(new StringVector(new String[] {"sumo", "-c", "test.sumocfg"}));
        for (int i = 0; i < 5; i++) {
            Simulation.step();
        }
        Simulation.close();
    }
}
```

Please note that starting with SUMO 1.16.0 it seems to be necessary to preload some libraries
on Windows, see https://github.com/eclipse-sumo/sumo/issues/12605.

### compiling on Linux (make sure SUMO_HOME is set and sumo has been built)

```
javac -cp $SUMO_HOME/bin/libsumo-{{Version}}-SNAPSHOT.jar Test.java
```

### running on Linux

```
java -Djava.library.path=$SUMO_HOME/bin -cp $SUMO_HOME/bin/libsumo-{{Version}}-SNAPSHOT.jar:. Test
```

### casting subscription results

Please be aware that casting subscription results is not straightforward with Java.
You have to use the `cast` function as below. If the cast is not successful it will not throw an exception
but return a null pointer.

```
TraCIResults ssRes = Simulation.getSubscriptionResults();
for (Map.Entry<Integer, TraCIResult> entry : ssRes.entrySet()) {
    TraCIResult sR = entry.getValue();
    TraCIStringList vehIDs = TraCIStringList.cast(sR);
    for (String vehID : vehIDs.getValue()) {
        System.out.println("Subscription Departed vehicles: " + vehID);
    }
}
```
## Matlab

Please install the [Python package](#python). You can then use all commands inside your Matlab scripts
just as in Python by adding the `py.` prefix.

```
py.libsumo.start(["sumo", "-c", "test.sumocfg"])
py.libsumo.simulationStep()
```

## C#

### Example Code (Program.cs)
Make sure you have added the swig generated C# bindings to your solution, as well as the compiled libsumocs.dll if not present in your Sumo installation folder.

```c#
using Eclipse.Sumo.Libsumo;

internal class Program 
{
    static void Main(String[] args) {
        Simulation.start(new StringVector(new String[] {"sumo", "-c", "test.sumocfg"}));
        for (int i = 0; i < 5; i++) {
            Simulation.step();
        }
        Simulation.close();
    }
}
```