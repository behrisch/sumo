---
title: GenericParameters
---

Generic parameters allow an arbitrary mapping of string keys to string
values. They can be used for user-defined data storage but some
key/value pairs also affect the simulation.

The following objects support the definition of custom parameters in
their XML definitions:

- Edge
- Lane
- Person
- Vehicle
- Stop
- VehicleType
- PoI
- Polygon
- Route
- TrafficLight
- ChargingStation
- ParkingArea
- Calibrator
- Rerouter
- VariableSpeedSign

Parameters support the following functionality

- editing in [netedit](../Netedit/editModesCommon.md#generic_parameters)
- Supplying additional information to the user (via
  [sumo-gui](../sumo-gui.md) [dialogs](../sumo-gui.md#plotting_object_properties), [text annotations](../sumo-gui.md#textual_annotations) or when inspecting XML files)
- reading and writing [via TraCI](../TraCI/GenericParameters.md).
- retaining custom information when [saving and loading simulation state](SaveAndLoad.md)

Some parameters are *special* and affect (or reflect) simulation behavior in the following ways:

- [setting up devices on a per-vehicle basis](../Definition_of_Vehicles,_Vehicle_Types,_and_Routes.md#devices)
- customizing the functionality of [actuated traffic lights](../Simulation/Traffic_Lights.md#parameters)
- configurating the [visualization](Railways.md#trains) of trains
- configuring vehicle types for use with the [electric vehicle model](../Models/Electric.md)
- tracking the [public transport 'tripId' that may change at stops](../Definition_of_Vehicles,_Vehicle_Types,_and_Routes.md#stops_and_waypoints).
- setting [transient junction model parameters](../Definition_of_Vehicles,_Vehicle_Types,_and_Routes.md#transient_parameters)
- customizing [parking search traffic (cruising for parking)](Rerouter.md#rerouting_to_an_alternative_parking_area) and [charging station search](Stationfinder.md#charging_station_target_function)
- customizing emission model [PHEMlight5 parameters](../Models/Emissions/PHEMlight5.md#dynamic_modification_of_parameters)
- customizing emission behavior with respect to [engine-shutoff](../Models/Emissions.md#standing_vehicles)
- configuring [parameters of carFollowModel and laneChangeModel  via TraCI](../TraCI/Change_Vehicle_State.md#supported_lanechangemodel_parameters)
- retrieving [parameters of carFollowModel and laneChangeModel via TraCI](../TraCI/Vehicle_Value_Retrieval.md#supported_lanechangemodel_parameters)
- configuring [device parameters via TraCI](../TraCI/Change_Vehicle_State.md#supported_device_parameters)
- retrieving [device parameters via TraCI](../TraCI/Vehicle_Value_Retrieval.md#supported_device_parameters) (some devices have read-only parameters)


Parameters are always defined as child elements of the respective
object:

```xml
<vehicle id="v0" route="route0" depart="0">
  <param key="answer to everything" value="42"/>
</vehicle>
```
