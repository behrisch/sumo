---
title: Calibrator
---

# Calibrators

These trigger-type objects may be specified within an {{AdditionalFile}} and allow the
dynamic adaption of traffic flows, speeds and vehicle parameters (vTypes). The syntax for such an
object is: `<calibrator id="<ID>" lane="<LANE_ID>" output="<OUTPUT_FILE>"/>`. They can be used to modify simulation
scenario based on induction loop measurements. They can also be used to model location-base changes in driving behavior.

A calibrator will remove vehicles in excess of the specified flow and it will insert new vehicles (of the specified type)
if the normal traffic demand of the simulation does not meet the
specified number of `vehsPerHour`. Furthermore, the speed on the edge will be
adjusted to the specified `speed` similar to the workings of a [variable speed
sign](../Simulation/Variable_Speed_Signs.md).

Calibrators will also remove vehicles if the traffic on their lane is jammed beyond what
would be expected given the specified flow and speed. This ensures that
invalid jams do not grow upstream past a calibrator. This behavior can be controlled with attribute 'jamThreshold' (see below).

If no target flow is given, the provided type information will instead be used to modify the type of vehicles that are passing the calibrator.

```xml
<additional>
  <vType id="t0" speedDev="0.1" speedFactor="1.2" sigma="0"/>
  <route id="c1" edges="beg middle end rend"/>

  <calibrator id="calibtest_edge" edge="beg" pos="0" output="detector.xml">
    <flow begin="0"    end="1800" route="c1" vehsPerHour="2500" speed="27.8" type="t0" departPos="free" departSpeed="max"/>
    <flow begin="1800" end="3600" route="c1" vehsPerHour="2500" speed="15.0" type="t0" departPos="free" departSpeed="max"/>
  </calibrator>

  <calibrator id="calibtest_lane" edge="middle_1" pos="0" output="detector.xml">
    <flow begin="0"    end="1800" route="c1" vehsPerHour="600" speed="27.8" type="t0" departPos="free" departSpeed="max"/>
    <flow begin="1800" end="3600" route="c1" vehsPerHour="800" speed="15.0" type="t0" departPos="free" departSpeed="max"/>
  </calibrator>

</additional>
```

The following attributes/elements are used within the calibrator
element:

| Attribute Name | Value Type    | Description                                                                                                     |
| -------------- | ------------- | --------------------------------------------------------------------------------------------------------------- |
| **id**         | id (string)   | The id of the calibrator                                                                                        |
| edge           | id (string)   | The id of an edge for measuring and calibrating flow. (Either *edge* or *lane* must be specified)               |
| lane           | id (string)   | The id of a lane for measuring and calibrating flow (Either *edge* or *lane* must be specified)                 |
| **pos**        | float         | The position of the calibrator on the specified lane (currently ignored, see [\[1\]](https://github.com/eclipse-sumo/sumo/issues/1331))  |
| period (alias freq) | float    | The time interval between calibration attempts. default is step-length. Setting a high value limits the maximum achievable flow  |
| routeProbe     | id (string)   | The id of the [routeProbe](../Simulation/Output/RouteProbe.md) element from which to determine the route distribution for generated vehicles.|
| jamThreshold    | float | A threshold value to detect and clear unexpected jamming if the mean edge speed drops below FLOAT * speedLimit. Range [0, 1]. Default: 0.5 (0.8 in meso)|
| output         | file (string) | The output file for writing calibrator information or *NULL*                                                    |
| vTypes         | string        | space separated list of vehicle type ids to consider (for counting/removal/type-modification), "" means all; default "".          |

The `flow` elements which are defined as children of the calibrator definition
follow the general [format of flow
definitions](../Definition_of_Vehicles,_Vehicle_Types,_and_Routes.md#repeated_vehicles_flows).
As the only difference, either the attribute `vehsPerHour` or `speed` or `type` (or any combination of these) must be
given.

By default edge calibrators will use `departLane="free"` whereas lane calibrators will use `departLane="x"`
where *x* is the lane index of the calibrator lane. All calibrators
default to `departSpeed="max"`.

## Decision to insert new vehicles

The interval (*begin*, *end*) defines the time in which calibration
takes place. The length of the interval also defines the aggregation
period for comparing observed and desired flows. The goal of the
calibration is to ensure that the correct number of vehicles are
inserted by the end of the interval. At the same time, the space-time
structure of existing traffic should be preserved as much as possible.

There is a trade-off between using shorter or longer calibration
intervals.

- With shorter consecutive intervals it may happen that vehicles will
  be removed first and later inserted because they did not arrive with
  uniform density
- With longer intervals it may happen that vehicles are inserted very
  lated because the calibrator tries to wait for existing vehicles
  that might still appear

The *period* attribute defines how often a check for inserting vehicles
takes place. This value defaults to the simulation step-length. Larger
values conserve computation time but may also lead to a tighter
clustering of inserted vehicles.

The algorithm for deciding when exactly to insert (and remove) vehicles
is described in [Erdmann, Jakob (2012) Online-Kalibrierung einer
Mikroskopischen Verkehrssimulation. ViMOS 2012, 29.11.2012, Dresden,
Deutschland.](https://elib.dlr.de/79428/)

## Routes of generated Vehicles

Whenever the measured flow in a given interval is lower than the
specified flow, new vehicles are inserted. If the `routeProbe` attribute is
specified, a route is sampled from the distribution of the named [route
probe detector](../Simulation/Output/RouteProbe.md). Otherwise the `route`
attribute of the flow is used. Note, that this value may also specify
the name of a route distribution.

## Calibrating only Flow

If attribute 'speed' is omitted from the `<flow>` definition, the calibrator will only affect flow by removing or insertion vehicles:

```xml
<additional>
  <vType id="t0" speedDev="0.1" speedFactor="1.2" sigma="0"/>
  <route id="c1" edges="beg middle end rend"/>

  <calibrator id="calibtest_edge" edge="beg" pos="0" output="detector.xml">
    <flow begin="0"    end="1800" route="c1" vehsPerHour="2500" type="t0"/>
    <flow begin="1800" end="3600" route="c1" vehsPerHour="2500" type="t0"/>
  </calibrator>
</additional>
```

## Calibrating only Speed

If only attribute 'speed' is given in the `<flow>` definition, the calibrator acts similar to a [variableSpeedSign](Variable_Speed_Signs.md):

```xml
<additional>
  <calibrator id="calibtest_edge" edge="beg" pos="0" output="detector.xml">
    <flow begin="0"    end="1800" speed="10"/>
    <flow begin="1800" end="3600" speed="20"/>
  </calibrator>
</additional>
```

## Calibrating vehicle types
When a calibrator flow is defined without attribute `vehsPerHour` but with attribute `type`, this defines a type-calibrator.
This type of calibrator will modify the types of all passing vehicles (or all vehicles that match the `vTypes` attribute of the calibrator).
The normal behavior is to replace the type of the passing vehicles with the type set in the flow element.

!!! note
    When calibrating types, the 'route' attribute can be omitted from the flow definition.

!!! caution
    The type modification happens when the vehicle enters the calibrator edge regardless of the configured calibrator position.

!!! caution
    Type calibration does currently not work with the mesoscopic model.

### Type-dependent mapping

If the traffic consists of multiple vehicle types (i.e. passenger cars and trucks) it may be desirable to either

- modify only some of the observed types
- perform a dependent mapping: carType -> carType2, truckType -> truckType2

Both can be accomplished by using the `vTypes` attribute of the calibrator to make it apply to a subset of types only.
For a dependent mapping, multiple calibrators (each with a different `vTypes` attribute) may be defined as in the following example:

```xml
<additional>
  ...
  <calibrator id="forCars" edge="E1" pos="0" vTypes="myCarType">
    <flow begin="0" end="1800" type="myCarType2"/>
  </calibrator>
  <calibrator id="forTrucks" edge="E1" pos="0" vTypes="myTruckType">
    <flow begin="0" end="1800" type="myTruckType2"/>
  </calibrator>
</additional>
```

If there are many different vehicle types in the simulation, creating one calibrator per vType may be cumbersome to define.
In this case it may be better to define a mapping between type distributions as described below.

### Mapping between vTypeDistributions

A special behavior is activated if the following conditions are met:
- the `type` in the flow element references a `vTypeDistribution`
- the passing vehicle was defined with a type drawn from a `vTypeDistribution`
- both vTypeDistributions have the same number of member types
In this case, the new type of the passing vehicle will be mapped to a specific type in the vType distribution:
- the index of the actual vehicle type in the original vTypeDistribution will be computed
- the type with that index in the new vTypeDistribution will be used as the new vehicle type

Example route-file input:

```xml
<vTypeDistribution id="dist1">
  <vType id="car" probability="70"/>
  <vType id="truck" maxSpeed="10" probability="30" vClass="truck"/>
</vTypeDistribution>
```
Example additional-file input:
```xml
<vTypeDistribution id="bad_weather">
  <vType id="car2" speedFactor="0.8" decel="3"/>
  <vType id="truck2" decel="2" tau="1.5" vClass="truck"/>
</vTypeDistribution>

<calibrator id="c1" lane="middle_0" pos="0" output="detector.xml">
  <flow begin="900"    end="1800" route="r1" type="bad_weather"/>
</calibrator>
```
In this example, all cars will be mapped to slower cars (type 'car' to 'car2') and all trucks will be mapped to trucks that keep larger distances.

### Calibrating on junctions

You can define a type calibration for a whole junction in the following way:

```xml
<additional>
  <calibrator id="cars" node="J1" pos="0">
    <flow begin="0" end="1800" type="myNewCarType"/>
  </calibrator>
  <calibrator id="carsUndone" node="J2" pos="0" local="true">
    <flow begin="0" end="1800" type="myNewCarType2"/>
  </calibrator>
</additional>
```

This will change the vehicle type once the vehicle enters one of the edges which approach the junction.
The second variant above with the `local` attribute will also undo the changes (revert to the original type)
once the vehicle has left any edge outgoing from the junction.
As with the general type calibration the `pos` attribute is being ignored.


# Building a scenario without knowledge of routes, based on flow measurements

!!! note
    A simpler and more robust approach is provided by the [routeSampler](../Tools/Turns.md#routesamplerpy) tool.

Due to their ability of adapting higher as as well as lower flows to a
specified value, calibrators may be used to adapt (almost) arbitrary
traffic demand to a given set of measurements. One strategy for building
a scenario from measurements is therefore, to [generated random
traffic](../Tools/Trip.md#randomtripspy) and use Calibrators in
conjunction with [route probe
detectors](../Simulation/Output/RouteProbe.md).

Each edge where measurements are given should receive a calibrator and a
route probe detector. As soon as the first vehicle has passed the route
probe detector, the calibrator will be able to use that vehicles route.
For the calibrator to be able to function before the first vehicle, it
needs a *fall back* route which just needs to consist of a single edge
(i.e. the edge on which the calibrator is placed).

Example {{AdditionalFile}}:

```xml
<additional>
   <vType id="t0" speedDev="0.1"/>
   <routeProbe id="cali_edge1_probe" edge="edge1" period="60" file="output.xml"/>
   <route id="cali1_fallback" edges="edge1"/>
   <calibrator id="cali_edge1" lane="edge1_0" pos="0" output="detector.xml" period="60" routeProbe="cali_edge1_probe">
      <flow begin="0"    end="1800" route="cal1_fallback" vehsPerHour="2500" speed="27.8" type="t0" departPos="free" departSpeed="max"/>
      <flow begin="1800" end="3600" route="cal1_fallback" vehsPerHour="2500" speed="15.0" type="t0" departPos="free" departSpeed="max"/>
   </calibrator>
</additional>
```

Running the simulation with the random demand as well as these `<calibrator>` and `<routeProbe>`
definitions will achieve a simulation in which traffic matches the
specified flows at each calibrator edge. However, the realism of traffic
flow behind (or between) calibrators depends on the fit between random
routes and real-world routes. The importance of this fit increases with
the size and complexity of the network between calibrator edges.
