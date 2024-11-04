---
title: elementsAdditional
---

# Additional elements

Additionals* are elements which do not belong to the network, but may be used to influence the simulation or generate specific outputs. Additionals are defined in an {{AdditionalFile}}.

## Loading and Saving

Additionals can be loaded and saved via the File menu:

![](../images/GNELoadAdditionals.png)
Menu for loading additionals

## Create
Additionals are created in the *Additionals* editing mode (shortcut key: a). Once this mode is selected, the first step is to choose what kind of additional should be created in the comboBox "additional element". Once selected and if required, the parameters of the additional can be changed. Finally, with a click over a Lane/junction/edge (If the additional should be set over an element of the network), or over an empty area (if the item is independent of the network) the additional will be created.

![](../images/GNEFrame.png)
Frame to insert additionals

## Move and references

Additionals can be moved, but the freedom of movement depends of their relation with the network. Additionals that must be located over a lane can only move along the lane, and additionals that are located on a map can be moved in any direction. The movement also depends if the item is locked or not (symbolized by a lock icon on the item). The movement can be blocked during the creation of the element. The corresponding parameter can be changed within the *inspector* mode. Certain additionals have a length contingent to the length of the lane. To create a bus stop by choosing a reference point, which marks the initial
position of this additional element, three types of references for the length can be selected. E.g. for a bus stop with the length 20 in the point 50 of the lane it will be the following:

- Reference left will create a new bus stop with startPos = 30 and endPos = 50.
- Reference right will create a new bus stop with startPos = 50 and endPos = 70.
- Reference center will create a new bus stop with startPos = 40 and endPos = 60.

![](../images/GNEMove.png)
Movement of different additionals


![](../images/GNELock.png)
Additional locked and unlocked

Some additional types cannot be moved, therefore show a different icon:

![](../images/GNENotMove.png)
Example of additional that cannot be moved

## Parameters

There are two types of parameters for each additional: User-defined-parameters and **netedit**-parameters. The first mentioned parameters can be of type *int*, *float*, *bool*,*string*, or *list* and each has a default value. In the case of type *list* the user can add or remove values using the *add* or *remove* *row* buttons. In the case of the characteristic parameters of **netedit**, this is the option to block the movement of an element, and in the case of the elements have a length, the user is allowed to change the length and reference.

![](../images/GNEParameterlist.png)
Adding a list of bus stop lines

![](../images/GNEReferences.png)
Adding additional with different references

## Additional hierarchies

Certain additionals need another previous additionals already placed in the network to be created (For example, a *detEntry* or *detExit* needs a previous *detectorE3* placed in the network). These child additionals are inserted in the same way as an Additional, but the insertion of an additional child is
different. Before the insertion of an *additional child* in the map, either the ID of the *additional parent* parent must be selected in the list of IDs placed on the left frame or additional parent must be clicked in the view. In the list the IDs of the additional parents only appear when they can be parents of the additional child (Referring to the given example: if the user wants to insert a *detEntry*, than only IDs of *detectorE3* will appear in the list on the left frame). Additional parents and their childs are graphically connected by a yellow line.

![](../images/GNEAdditionalSet.png)
Insertion of an additional Set

Additional parent can be changed during inspection of an additional child, either writing the Id of the new parent in section "Netedit attributes" or clicking over "Set new Parent" button, and then clicking over new parent.

## Help dialogs

With the help buttons placed in the left frame, users can obtain information about additional and editor parameters (Full name, type and description).

![](../images/GNEHelpParameters.png)
Help window for parameters

![](../images/GNEHelpReferences.png)
Help window for editor parameter

## Additional types

This section describes the different types of additional objects which are supported

### Stopping Places

Stopping places are sections of lanes, in which vehicles can stop during a certain time defined by the user.

#### Bus stop

[Bus stops](../Simulation/Public_Transport.md#bus_stops) are positions of a
lane in which vehicles ("busses") stop for a pre-given time. Every Bus stop has an unique ID assigned automatically by
**netedit**, a length and a list of bus lines defined by the user.

![](../images/GNEBusStop.png)
Bus stop

#### Container stops

[Container stops](../Specification/Logistics.md#container_stops) are similar to BusStops, but they are oriented towards [logistics simulation](../Specification/Logistics.md).

![](../images/GNEContainerStop.png)
Container stop

#### Charging station

[Charging stations](../Models/Electric.md#charging_stations) define a surface over a lane in which the vehicles equipped with a battery are charged. Charging stations own an unique-ID generated by **netedit**, a length, a charging power defined in W, a charging efficiency, a switch for enable or disable charge in transit, and a charge delay.

![](../images/GNEChargingStation.png)
Charging station

#### Parking Areas

A [ParkingArea](../Simulation/ParkingArea.md)-element describes an area for parking outside the road network (either road-side parking or car parks).

<img src="../images/GNEParkingArea.png" style="width:600px;">
Parking Area

### Detectors

Detectors are additionals which save information about vehicles that passed over a certain position on the lane.

#### Induction Loops Detectors (E1)

A [E1 detector](../Simulation/Output/Induction_Loops_Detectors_(E1).md) owns a ID parameter generated by **netedit**, a position at a certain lane, a *freq* attribute which describes the
period over which collected values shall be aggregated, a list of *VTypes* that will be detected, and a *file* attribute which tells the
simulation to which file the detector shall write his results to.

![](../images/GNEE1.png)
Detector E1

#### Lane Area Detectors (E2)

Most of the [E2 detectors](../Simulation/Output/Lanearea_Detectors_(E2).md)' attributes have the same meaning as for [E1 induction loops](../Simulation/Output/Induction_Loops_Detectors_(E1).md), including automatic ID and position at a certain lane. As a real detector has a certain length, "length" must be supplied as a further parameter. When placed in netedit, the detector will be extended by the given length in the upstream direction. When selecting 'e2MultiLaneDetector', two or more sequential lanes must selected on which to place the detector.

![](../images/GNEE2.png)
Detector E2

#### Multi-Entry Multi-Exit Detectors (E3)

A [Detector E3](../Simulation/Output/Multi-Entry-Exit_Detectors_(E3).md) is an AdditionalSet with the same attributes as [Induction Loop Detector E1](../Simulation/Output/Induction_Loops_Detectors_(E1).md). The difference is that detectors E3 have as childs the Entry/Exit detectors.

![](../images/GNEE3.png)
Detector E3

#### DetEntry/DetExit

Childs of an AdditionalSet [Multi-Entry Multi-Exit Detectors E3](../Simulation/Output/Multi-Entry-Exit_Detectors_(E3).md). These additionals have only the attributes ID of a lane in which it is
placed and positioned over a lane.

![](../images/GNEEntry.png)
Detector entry

![](../images/GNEExit.png)
Detector exit

#### Instant Induction Loops Detectors (E1Instant)

A [E1 detector](../Simulation/Output/Induction_Loops_Detectors_(E1).md) owns a ID parameter generated by **netedit**, a position at a certain lane, a *freq* attribute which describes the period over which collected values shall be aggregated, a list of *VTypes* that will be detected and a *file* attribute which tells the simulation to which file the detector shall write his results to.

![](../images/GNEE1Instant.png)
Detector E1 Instant

### Route Probe

[RouteProbe detectors](../Simulation/Output/RouteProbe.md) are meant to determine the route distribution for all vehicles that passed an edge in a given interval. Their real-world equivalent would be a police stop with questionnaire or a look into the database of navigation providers such as TomTom.

![](../images/GNERouteProbe.png)
Route Probe

### Calibrator

A [calibrator](../Simulation/Calibrator.md) generates a flow of vehicles during a certain time, and allows dynamic adaption of traffic flows and speeds. A calibrator can be used to modify a simulation scenario based on induction loop measurements. It will remove vehicles in excess of the specified flow and will insert new vehicles if the normal traffic demand of the simulation does not reach the specified number of vehsPerHour. Furthermore, the defined edge speed will be adjusted to the specified speed similar of the operation of a variable speed sign. Calibrators will also remove vehicles if the traffic on their lane is jammed beyond the specified flow and speed. This ensures that invalid jams do not grow upstream past a calibrator. A right-click over the calibrator icon allows opening the values editor with the menu option 'Open calibrator Dialog'.

![](../images/GNECalibrator.png)
Calibrator

![](../images/GNECalibratorDialog.png)
Editing calibrator's values

### Rerouter

A [rerouter](../Simulation/Rerouter.md) changes the route of a vehicle as soon as the vehicle moves onto a specified edge.

![](../images/GNERerouter.png)
Rerouter is placed off the net.

A right-click over the rerouter icon allows opening the values editor with the menu option 'Open rerouter Dialog'. Rerouter's values are divided in intervals, and every interval contains a list of [closing streets](../Simulation/Rerouter.md#closing_a_street), [closing lanes](../Simulation/Rerouter.md#closing_a_lane), [assignations of new destinations](../Simulation/Rerouter.md#assigning_a_new_destination) and [assignations of new routes](../Simulation/Rerouter.md#assigning_a_new_route):

![](../images/GNERerouterDialog1.png)
open rerouter dialog. One click over '+' button open a dialog for adding a new interval.

![](../images/GNERerouterDialog2.png)
In interval dialog can be specified the four types of actions, as well as the begin and end of interval.

![](../images/GNERerouterDialog3.png)
If values of actions are invalid, a warning icon appears in every row. This can be applicable in begin and end of interval.

![](../images/GNERerouterDialog4.png)
A click over begin or end of interval opens the Rerouter interval dialog. In the same way, a click over "x" button removes the interval.

### Vaporizer

[vaporizers](../Simulation/Vaporizer.md) remove all vehicles as soon as they move onto a specified edge.

![](../images/GNEVaporizer.png)
Vaporizer placed over edge. Its placed always at the begins of edge.

!!! caution
    Vaporizers are deprecated

### Variable Speed Signs

A [Variable Speed Signal](../Simulation/Variable_Speed_Signs.md) modifies the speed of a set of lanes during a certain time defined by user. A right-click over the Variable Speed Signal icon allows opening the values editor with the menu option 'Open variableSpeedSign Dialog'.

![](../images/GNEVariableSpeedSignal.png)
Variable Speed Sign

![](../images/GNEVariableSpeedSignalDialog.png)
Variable Speed Signal Sign
