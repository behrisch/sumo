---
title: Emissions
---

# Introduction

SUMO includes the following emission models:

- [HBEFA v2.1-based](../Models/Emissions/HBEFA-based.md): A
  continuous reformulation of the [HBEFA](https://www.hbefa.net/) v2.1
  emissions data base (open source);
- [HBEFA v3.1-based](../Models/Emissions/HBEFA3-based.md): A
  continuous reformulation of the [HBEFA](https://www.hbefa.net/) v3.1
  emissions data base (open source);
- [HBEFA v4.2-based](../Models/Emissions/HBEFA4-based.md): A
  continuous reformulation of the [HBEFA](https://www.hbefa.net/) v4.2
  emissions data base (open source);
- [PHEMlight](../Models/Emissions/PHEMlight.md), a derivation of
  the original
  [PHEM](https://www.itna.tugraz.at/en/research/areas/em/simulation/phem.html)
  emission model (model is open source, but full data sets are only commercially available);
- [PHEMlight5](../Models/Emissions/PHEMlight5.md), the V5 version of PHEMlight
  supporting deterioration emission model (model is open source, but full data sets are only commercially available);
- [Electric Vehicle
  Model](../Models/Electric.md#emission_output): an
  electricity-consumption model by [Kurczveil, T., López, P.A.,
  Schnieder](../Models/Electric.md#publications).
- [MMP Electric Vehicle Model](../Models/MMPEVEM.md): an
  electricity-consumption model by [Kevin Badalian from Teaching and Research
  Area Mechatronics in Mobile Propulsion (MMP), RWTH Aachen
  University](../Models/MMPEVEM.md#publications).

Literature on the Models and their implementation can be found at [the
DLR electronic library
(https://elib.dlr.de/89398/)](https://elib.dlr.de/89398/).

# Using Models

All models implement different vehicle emission classes. These classes
can be assigned to vehicles by using the vehicle type attribute
`emissionClass`. If the model has different classes, the definition
has the form `emissionClass="<model>/<class>"` e.g. `HBEFA3/PC_G_EU4`.
If the model has only one class (for instance the Electric Vehicle Model),
the `<class>` can be omitted in the input and will show up as `default`
in the output. There is one special model `Zero` which does not
generate emissions or energy consumption at all.

Available emission classes
can be found within the emission model descriptions
([HBEFA v2.1-based](../Models/Emissions/HBEFA-based.md),
[HBEFA v3.1-based](../Models/Emissions/HBEFA3-based.md),
[HBEFA v4.2-based](../Models/Emissions/HBEFA4-based.md),
[PHEMlight](../Models/Emissions/PHEMlight.md),
[PHEMlight5](../Models/Emissions/PHEMlight5.md)). The current default
model is `HBEFA3/PC_G_EU4` (a gasoline powered Euro norm 4 passenger car
modeled using the HBEFA3 based model).

# Pollutants / Measurements covered by models

!!! caution
    Please note that the unit of fuel-related outputs changed with SUMO 1.14.0 from liters to milligram.
	For the old behavior use the option **--emissions.volumetric-fuel**.

<table class="tg">
  <tr>
    <th class="tg-uzvj" rowspan="2">model</th>
    <th class="tg-7btt" colspan="7">pollutant / measurement</th>
  </tr>
  <tr>
    <td class="tg-7btt">CO<sub>2</sub></td>
    <td class="tg-7btt">CO</td>
    <td class="tg-7btt">HC</td>
    <td class="tg-7btt">NO<sub>x</sub></td>
    <td class="tg-7btt">PM<sub>x</sub></td>
    <td class="tg-7btt">fuel consumption</td>
    <td class="tg-uzvj">electricity consumption</td>
  </tr>
  <tr>
    <td class="tg-0pky">HBEFA v2.1-based<br><code>emissionClass="HBEFA/..."</code></td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">-</td>
  </tr>
  <tr>
    <td class="tg-0pky">HBEFA v3.1-based<br><code>emissionClass="HBEFA3/..."</code></td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">-</td>
  </tr>
  <tr>
    <td class="tg-0pky">HBEFA v4.2-based<br><code>emissionClass="HBEFA4/..."</code></td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
  </tr>
  <tr>
    <td class="tg-0pky">PHEMlight<br><code>emissionClass="PHEMlight/..."</code></td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">(x)</td>
  </tr>
  <tr>
    <td class="tg-0pky">PHEMlight<br><code>emissionClass="PHEMlight5/..."</code></td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">x</td>
    <td class="tg-c3ow">(x)</td>
  </tr>
  <tr>
    <td class="tg-0pky">Electric Vehicle Model<br><code>emissionClass="Energy"</code></td>
    <td class="tg-c3ow">-</td>
    <td class="tg-c3ow">-</td>
    <td class="tg-c3ow">-</td>
    <td class="tg-c3ow">-</td>
    <td class="tg-c3ow">-</td>
    <td class="tg-c3ow">-</td>
    <td class="tg-c3ow">x</td>
  </tr>
  <tr>
    <td class="tg-lboi">No Emissions<br><code>emissionClass="Zero"</code></td>
    <td class="tg-c3ow">-</td>
    <td class="tg-c3ow">-</td>
    <td class="tg-c3ow">-</td>
    <td class="tg-c3ow">-</td>
    <td class="tg-c3ow">-</td>
    <td class="tg-c3ow">-</td>
    <td class="tg-9wq8">-</td>
  </tr>
</table>

PHEMlight and PHEMlight5 generate electricity consumption values only if the
data files for battery powered or hybrid vehicles are available (which are not part of
the free data set).

# Outputs

Pollutants emitted by the simulated vehicles can be visualised using
[sumo-gui](../sumo-gui.md) or be written into output files, both by
[sumo](../sumo.md) and [sumo-gui](../sumo-gui.md). The
following output can be used:

- [trip information](../Simulation/Output/TripInfo.md): In
  combination with the emissions device, the tripinfo output contains
  the sum of all pollutants emitted / fuel consumed during a vehicle's
  journey. To record emissions for all trips add the options **--tripinfo-output** and **--device.emissions.probability 1.0**.
  Alternatively, you can configure emission recording for selected
  vehicles using [generic
  parameters](../Definition_of_Vehicles,_Vehicle_Types,_and_Routes.md#devices).
- [edgelane
  emissions](../Simulation/Output/Lane-_or_Edge-based_Emissions_Measures.md):
  These output files contain the pollutants emitted at an edge / a
  lane, aggregated over a variable time span
- [emission-output](../Simulation/Output/EmissionOutput.md):
  Unaggregated emission values for every vehicle and time step
- [Color vehicles by
  emissions](../sumo-gui.md#vehicle_visualisation_settings): This
  can be used to show the emissions for each vehicle during each
  simulation step
- [Color lanes/edges by
  emissions](../sumo-gui.md#edgelane_visualisation_settings):
  This can be used to show the emissions for all vehicles on a lane
  during each simulation step (or for the whole edge in
  [Simulation/Meso](../Simulation/Meso.md)).

# Standing vehicles

A vehicle can have either a foreseeable stop on its route or stop at a junction or in a jam.
In both cases it will still produce emissions / consume energy as long as the motor is running.
For planned stops SUMO will switch off the engine immediately if the duration is longer than 300s
(configurable via the vehicle type [generic parameter](../Simulation/GenericParameters.md) `<param key="shutOffStopDuration" value="300"/>` with a value in seconds).

The automated start/stop also for unplanned stops is not enabled by default but can be set using
the vehicle type [generic parameter](../Simulation/GenericParameters.md) `<param key="shutOffAutoDuration" value="20"/>`.
This expects a value in seconds and will switch off the engine automatically if the vehicle does not move for longer than the given time.

# Coasting vehicles

A rolling (combustion driven) vehicle which does not accelerate actively (coasting) will not consume
fuel because the engine gets switch off automatically in this case. All PHEMlight and HBEFA models use
a threshold depending on the current speed, acceleration and slope to determine whether the vehicle is
in this regime and set all emission values to zero. The value depends of course also on the
characteristics of the vehicle such as mass and front area. For details see the code of
PollutantsInterface::Helper::getCoastingDecel.

# Further Interfaces

The tool [traceExporter.py](../Tools/TraceExporter.md) converts
SUMO's [fcd-output](../Simulation/Output/FCDOutput.md) into files
that can be directly read by the
[PHEM](https://www.itna.tugraz.at/en/research/areas/em/)
application. A [tutorial on generating trace files (including PHEM input
files)](../Tutorials/Trace_File_Generation.md) using this tool is
available.

<div style="border:1px solid #909090; min-height: 35px;" align="right">
<span style="float: right; margin-top: -5px;"><a href="https://web.archive.org/web/20191005024529/https:/ec.europa.eu/research/fp7/index_en.cfm"><img src="../images/FP7-small.gif" alt="Seventh Framework Programme"></a>
<a href="http://web.archive.org/web/20240722054138/https://verkehrsforschung.dlr.de/en/projects/colombo"><img src="../images/COLOMBO-small.png" alt="COLOMBO project"></a></span>
<span style="">This part of SUMO was developed, reworked, or extended within the project
<a href="http://web.archive.org/web/20240722054138/https://verkehrsforschung.dlr.de/en/projects/colombo">"COLOMBO"</a>, co-funded by the European Commission within the <a href="https://web.archive.org/web/20191005024529/https:/ec.europa.eu/research/fp7/index_en.cfm">Seventh Framework Programme</a>.</span></div>

<div style="border:1px solid #909090; min-height: 35px;" align="right">
<span style="float: right; margin-top: -5px;"><a href="https://web.archive.org/web/20191005024529/https:/ec.europa.eu/research/fp7/index_en.cfm"><img src="../images/FP7-small.gif" alt="Seventh Framework Programme"></a>
<a href="https://web.archive.org/web/20180309093847/https://amitran.eu/"><img src="../images/AMITRAN-small.png" alt="AMITRAN project"></a></span>
<span style="">This part of SUMO was developed, reworked, or extended within the project
<a href="https://web.archive.org/web/20180309093847/https://amitran.eu/">"AMITRAN"</a>, co-funded by the European Commission within the <a href="https://web.archive.org/web/20191005024529/https:/ec.europa.eu/research/fp7/index_en.cfm">Seventh Framework Programme</a>.</span></div>
