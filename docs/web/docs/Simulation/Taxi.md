---
title: Taxi
---

# Introduction
Since version 1.5.0 SUMO supports simulation of demand responsive transport (DRT)
via the taxi device. This allows a fleet of taxis to service customer requests
based on a configurable dispatch algorithm.

!!! note
    While the taxi capabilities are under development, their status can be checked via Issue #6418.

# Equipping Vehicles
A vehicle can be equipped with an Taxi device to make it part of the taxi fleet.
To attach a Taxi device to a vehicle, the [standard device-equipment
procedures](../Definition_of_Vehicles,_Vehicle_Types,_and_Routes.md#devices) can be applied using `<device name>=taxi`.

For instance, a single vehicle can configured as taxi as in the following minimal example:

```xml
    <vehicle id="v0" route="route0" depart="0" line="taxi">
        <param key="has.taxi.device" value="true"/>
    </vehicle>
```

The following table gives the full list of possible sumo options for the taxi device (all option names have to be prepended with "--device.taxi." when set from the command line):

| Parameter                        | Type             | Range                                                        | Default          | Description                                                                         |
| -------------------------------- | ---------------- | ------------------------------------------------------------ | ---------------- | ----------------------------------------------------------------------------------- |
| dispatch-algorithm               | enum             | {greedy; greedyClosest; greedyShared; routeExtension; traci} | greedy | The dispatch algorithm                                     |
| dispatch-algorithm.output        | string           |                                                              | -                | Write information from the dispatch algorithm to a file                             |
| dispatch-algorithm.params        | string           |                                                              | -                | Provide absLossThreshold and relLossThreshold thresholds for greedyShared dispatch algorithm as KEY1:VALUE1[,KEY2:VALUE] |
| dispatch-period                  | float (s)        |                                                              | 60               | The period between successive calls to the dispatcher                               |
| idle-algorithm                   | enum             | {stop; randomCircling; taxistand}                            | stop     | The behavior of idle taxis                                                          |
| idle-algorithm.output            | string           |                                                              | -                | Write information from the idling algorithm to file                                 |



# Taxi Requests

## Direct Ride Hailing
A person can be defined as taxi customer with the following definition:

```xml
    <person id="p0" depart="0.00">
        <ride from="B2C2" to="A0B0" lines="taxi"/>
    </person>
```

## Intermodal Routing
A person can also use a taxi by including it as a [personTrip](../Specification/Persons.md#persontrips) mode:

```xml
    <person id="p0" depart="0.00">
        <personTrip from="B2C2" to="A0B0" modes="taxi"/>
    </person>
```

Whenever a person enters a taxi during the intermodal route search, a time penalty is applied to account for the expected time loss from waiting for the taxi and embarking. The default value is set 300s and can be configure with option **--persontrip.taxi.waiting-time**. This prevents rapid switching between travel modes.

## Groups of Persons
Multiple persons can travel together from the same origin to the same destination as a group using attribute `group` (if the taxi has sufficient capacity):

```xml
    <person id="p0" depart="0.00">
        <ride from="B2C2" to="A0B0" lines="taxi" group="g0"/>
    </person>
    <person id="p1" depart="0.00">
        <ride from="B2C2" to="A0B0" lines="taxi" group="g0"/>
    </person>
```

Starting with version SUMO 1.27.0 this grouping also works if reservations are received after a taxi has alrady been dispatched. Persons in the same group will be permitted to join the ride up to the capacity of the vehicle.

## Prebooking
Direct ride hailing can be regarded as a spontaneous booking. If the taxi is to be requested before the person is at the pick-up location, prebooking is necessary. This is used by specifying an `earliestPickupTime` and a `reservationTime` in the `ride`. The `reservationTime` specifies the time at which the reservation is created and made available to the dispatch. It may also be less than the person's depart. The `earliestPickupTime` specifies the time at which the person is ready to be picked up. Depending on the algorithm used, this may allow a better dispatch to be made. Especially in combination with a custom dispatch algorithm via [TraCi](#traci), the requirements of different use cases can be addressed.

### Application:
- The file with the persons must be loaded as an additional file.
- At least the `earliestPickupTime` must be specified. The `reservationTime` has the start time of the simulation as the default value.
- It is necessary to define the attribute `from` in the `ride`.
- If there was previously a `walk` in which the `arrivalPos` attribute was defined, then the `fromPos` attribute must also be defined in the `ride` with the same value.

### Example:
```xml
    <person id="p0" depart="100.00">
        <ride from="B2C2" to="A0B0" lines="taxi">
            <param key="earliestPickupTime" value="100.00"/>
            <param key="reservationTime" value="50.00"/>
        </ride>
    </person>
    <person id="p1" depart="0.00">
        <walk from="B2C2" to="B1C1"/>
        <ride from="B1C1" to="A1B1" lines="taxi">
            <param key="earliestPickupTime" value="30.00"/>
        </ride>
    </person>
```

### Behavior:
- If a person arrives at the departure point earlier than the `earliestPickupTime`, the person boards as soon as the taxi arrives.
- If the taxi is on time and the person does not arrive at the departure point within 3 minutes of the `earliestPickupTime`, the reservation is canceled and the taxi serves the next reservation.
- If the taxi arrives later and the person does not arrive at the departure point within 3 minutes of the taxi's arrival time, the reservation is canceled.
- If the greedy or greedyShared scheduling algorithms are used, the reservations are processed in the order of the `earliestPickupTime`. A combination of spontaneous bookings and prebookings can lead to side effects. It is recommended to use a custom dispo algorithm via TraCi.

### Notes:
- Prebookings for merged reservations and group reservations are not yet supported.
- The current status can be tracked in [Ticket 11429](https://github.com/eclipse-sumo/sumo/issues/11429)

# Multiple Taxi Fleets

By default, there is only a single taxi fleet using line attribute 'taxi' and taxi customers use attribute `lines="taxi"` for their rides.
It is permitted to define the line attribute for taxi with the prefix 'taxi:' and an arbitrary suffix (i.e. "taxi:fleetA").
Likewise, it is permitted to define the lines attribute for rides with the prefix 'taxi:' and a suffix.
When this is done, the following rules are applied when assigning taxis to customers:

- a taxi with line 'taxi:X' may only pick up customers with matching ride attribute lines="taxi:X" (for any value of X)
- a customer with lines="taxi" may use any taxi regardless of taxi fleet suffix
- a taxi with line 'taxi' may be pick up any customer regardless of the ride fleet suffix

# Dispatch Algorithms
The dispatch algorithm assigns taxis to waiting customers. The algorithm is
selected using option **--device.taxi.dispatch-algorithm ALGONAME**. The following
algorithms are available

- greedy: Assigns taxis to customers in the order of their reservations. The
  closest taxi (in terms of travel time) is assigned. If the reservation date is
  too far in the future, the customer is postponed.

- greedyClosest: For each available taxi, the closest customer (in terms of
  travel time) is assigned. If the reservation date is too far in the future,
  the customer is postponed.

- greedyShared: like 'greedy' but tries to pick up another passenger while delivering the first passenger to its destination. Parameters **absLossThreshold** and **relLossThreshold** to configure acceptable detours can be supplied using **--device.taxi.dispatch-algorithm.params KEY1:VALUE1[,KEY2:VALUE]**.

- routeExtension: like greedy but can pick up any passenger along the route and also extend the original route (within personCapacity limit).

- traci: Dispatch is deferred to [traci control](#traci). The algorithm only keeps track of pending reservations

!!! note
    User-contributed dispatch algorithms are welcome.

## Dispatch Options

The period in which the dispatch algorithm runs can be controlled with option **--device.taxi.dispatch-period**. The default is 60s.

The option **--device.taxi.dispatch-algorithm.params KEY1:VALUE1[,KEY2:VALUE]** supports key:value-pair `routingMode:0` which makes taxis use empty-network traveltimes when estimating pickup duration (or loaded traveltimes when available). By default taxis will use [current traveltimes (smoothed)](../Demand/Automatic_Routing.md#edge_weights) instead.

## Swapping tasks after dispatch

Taxis can be configured with `<param key="device.taxi.swapGroup" value="<GROUPNAME>"/>` in their `vType` or `<vehicle>`-definition (setting `<GROUPNAME>` to an arbitrary non-empty string).
If a taxi sets this parameter, it can receive tasks from other taxis with the same `swapGroup` value upon entering the idle state. This is doen if the other taxi has not yet picked up any passengers and the idle taxi is closer to the pickup location.

## Algorithm-Output

Option **--device.taxi.dispatch-algorithm.output FILE** is can be set to receive extra outputs from algorithms (i.e. for ride sharing metrics).

# Taxi Behavior

By default, taxis will remain in the simulation until all persons have left. To make them leave the simulation at an earlier time, the end time can be defined using a generic parameter in their ```vType``` or ```vehicle```-definition:

```xml
    <vType id="taxi" vClass="taxi">
        <param key="has.taxi.device" value="true"/>
        <param key="device.taxi.end" value="3600"/>
    </vType>
```

## Idle Behavior
By default, vehicles will leave the simulation after reaching the end of their final edge. To avoid this, taxis have an idling behavior configurable with option **--device.taxi.idle-algorithm**:

- "stop" (default): stop at the current location (off-road) after delivering the last customer of the current service request.
- "randomCircling": continue driving to random edges until the next request is received. (caution: taxi might get stuck in a cul-de-sac if the network has those)
- "taxistand": drive to a taxi stand and wait there for the next customer / dispatch. Defining the set of taxi stands and the strategy of choosing among them is described below

!!! note
    When using "randomCircling", the default value for parameter "device.taxi.end" is 8 hours after vehicle departure.

### Defining taxi stands

When using idle-algorithm **taxistand**, the following inputs must be provided:

- each taxi stand must be defined as a [parkingArea](ParkingArea.md)
- the list of parkingAreas that may be used for a particular taxi or taxi fleet must be defined as a `<rerouter>`-element according to the [description for parking search simulation](Rerouter.md#rerouting_to_an_alternative_parking_area).
- the taxi must define the parameter `device.taxi.stands-rerouter` either as a child element of the `<vehicle>` or its `<vType>` and declare the rerouter id.

By default, an idle taxi will pick the first parkingarea in the list of alternatives (`<parkingAreaReroute` entries in the `<rerouter>`).
If the [generic parameter](GenericParameters.md) `<param key="parking.ignoreDest" value="1"/>` is set in the vehicle or vType, then the "best" according to the rerouting strategy is used (i.e. the stand closest to the current vehicle location).

The strategy for choosing among the alternative taxi stands follows the description for [parking search simulation](Rerouter.md#determining_the_alternative_parking_area) (i.e. with respect to prior knowledge of remaining capacity).


Example declarations for the rerouter and the taxi vType that references it:

```
<rerouter id="rr0" edges="B0C0 E2D2" vTypes="taxi">
        <interval begin="0" end="1:0:0:0">
            <parkingAreaReroute id="pa_0"/>
            <parkingAreaReroute id="pa_1"/>
        </interval>
</rerouter>
```

!!! note
    To avoid warnings, the `edges` attribute of the rerouter should match the edges which contain taxi stands.

```
<vType id="taxi" vClass="taxi">
        <param key="has.taxi.device" value="true"/>
        <param key="device.taxi.stands-rerouter" value="rr0"/>
</vType>
```

### Taxis stands for rail taxis

When defining taxi stands for rail taxis, overtaking on a rail track is not possible and thus parking spaces will automatically be filled beginning at the downstream end.
If the first taxi in the queue of waiting taxis is dispatched, all idling taxis behind it will then automatically advance in the queue to fill up that spot.
If another vehicle approaches the end of the queue and cannot park (due to lack of free capacity or havin a different destination), the whole queue ahead will exit the parkingArea to allow the rear vehcle to pass.

## Customer Stops

Taxis will stop to pick-up and drop-off customers. The 'actType' attribute of a stop indicates the purpose ('pickup' / 'dropOff') as well as the ids of the customers and their reservation id. Stop attributes can be configured using [generic parameters]() `<vType>` or `<vehicle>` definition of the taxi:

```xml
    <vType id="taxi" vClass="taxi">
        <param key="has.taxi.device" value="true"/>
        <param key="device.taxi.pickUpDuration" value="0"/>
        <param key="device.taxi.dropOffDuration" value="60"/>
        <param key="device.taxi.parking" value="false"/>
    </vType>
```

- duration for pick-up stop can be configured with vType/vehicle param "device.taxi.pickupDuration" (default "0")
- duration for drop-off stop can be configured with vType/vehicle param "device.taxi.dropOffDuration" (default "60")

By default, vehicle stops will have attribute `parking="true"` which means that the taxi will not block a driving lane. This can be changed by setting
param "device.taxi.parking" to "false".

# TraCI
To couple an external dispatch algorithm to SUMO, the following [TraCI](../TraCI/index.md) functions are provided:

!!! note
    To make use of these functions, the option **--device.taxi.dispatch-algorithm traci** must be set

- traci.person.getTaxiReservations(reservationState)
- traci.vehicle.getTaxiFleet(taxiState)
- traci.vehicle.dispatchTaxi(vehID, reservations)

This set of API calls can be used to simplify writing custom dispatch algorithms by letting sumo:

- manage existing reservations
- manage the taxi fleet
- dispatch a taxi to service one or more reservations by giving a list of reservation ids (vehicle routing and stopping is then automatic).

## getTaxiReservations

Returns a list of of Reservation objects that have the following attributes

- id
- persons
- group
- state
- fromEdge
- toEdge
- arrivalPos
- departPos
- depart
- reservationTime
- state (positive value, see below)

When calling `traci.person.getTaxiReservations(reservationState)` the following arguments for reservationState are supported:

- 0: return all reservations regardless of state
- 1: return only new reservations
- 2: return reservations already retrieved
- 4: return reservations that have been assigned to a taxi
- 8: return reservations that have been picked up

Combinations of these values are also supported. For example sending a value of 3 (= 1 + 2) will return all reservations of both states 1 and 2.

## getTaxiFleet

A taxi can be in any of the following states:

- 0 (empty) : taxi is idle
- 1 (pickup):  taxi is en-route to pick up a customer
- 2 (occupied): taxi has customer on board and is driving to drop-off
- 3 (pickup + occupied): taxi has customer on board but will pick up more customers

when calling `traci.vehicle.getTaxiFleet(taxiState)` the following arguments for taxiState are supported:

- -1: (return all taxis regardless of state)
- 0: return only empty taxis
- 1: return taxis in state 1 and 3
- 2: return taxis in state 2 and 3
- 3: return taxis in state 3

## dispatchTaxi

If a taxi is empty, the following dispatch calls are supported

- dispatchTaxi(vehID, [reservationID]): pickup and drop-off persons belonging to the given reservation ID
- If more than one reservation ID is given, each individual reservation ID must occur exactly twice in the list for complete pickup and drop-off. The first occurrence of an ID denotes pick-up and the second occurrence denotes drop-off.

Example 1:  dispatchTaxi(vehID, [a]) means: pick up and drop off a.

Example 2:  dispatchTaxi(vehID, [a, a, b, c, b, c]) means: pick up and drop off a, then pick up b and c and then drop off b and c.

If a taxi is not in state empty the following re-dispatch calls are supported

- new reservations have no overlap with previous reservation: append new reservations to the previous reservations
- new reservations include all previous unique reservation ids exactly twice: reset current route and stops and treat as complete new dispatch. If one of the persons of the earlier reservation is already picked up, ignore the first occurrence of the reservation in the reservation list
- new reservations mentions include all previous unique reservation ids once or twice, all customers that are mentioned once are already picked up: reset current route and stops, use the single-occurrence ids as as drop-of

## Controlling Idle Taxis

The default idle algorithm ("stop") aims to ensure that the taxi doesn't exit the simulation (which were to happen if it ever drives past the final edge of it's route). To this end, it makes the taxi stop where it is or uses an existing stop on the route of the taxi and sets it to `triggered="person"` (which effectively makes the taxi wait indefinitely until the next dispatch).

In oder to send an idle taxi to another destination, the following things need to happend in order:

1. The taxi needs to extend it's route (i.e. with `vehicle.changeTarget` or `vehicle.setRoute`)
2. The current stop must be aborted with `vehicle.resume`
3. A new stop must be added (i.e. with `vehicle.setStop`.

# Outputs

The Taxi device generates output within a tripinfo-output file in the following
form:

```xml
    <tripinfo id="trip_0" ... >
        <taxi customers="5" occupiedDistance="6748.77" occupiedTime="595.00"/>
    </tripinfo>
```

## Parameter Retrieval
The following parameters can be retrieved via `traci.vehicle.getParameter` and written via **--fcd-output.params**.
It is also possible to color vehicles in [SUMO-GUI 'by param (numerical)'](../sumo-gui.md#vehicle_visualisation_settings) by setting these keys.

- device.taxi.state: returns integer value (see #gettaxifleet)
- device.taxi.customers: total number of customers served
- device.taxi.occupiedDistance: total distance driven in m with customer on board
- device.taxi.occupiedTime: total time driven in s with customers on board
- device.taxi.currentCustomers: space-separated list of persons that are to be picked up or already on board
