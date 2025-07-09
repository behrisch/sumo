---
title: Why Vehicles are teleporting
---

When running a simulation, one may encounter the following warning:

```
Warning: Teleporting vehicle '...'; waited too long, lane='...', time=....
```

What does it mean?

# Reasons

The following circumstances may force the simulation to "teleport" a
vehicle:

- the vehicle stood too long in front of an intersection (message:
  "*...'; waited too long, lane='...*")
- the vehicle has [collided](Safety.md#collisions) with his leader (message: "*...';
  collision, lane='...*")
- the vehicle is performing a [jump](../Definition_of_Vehicles%2C_Vehicle_Types%2C_and_Routes.md#jumps)

## Waiting too long, aka Grid-locks

In the case a vehicle is standing at the first position in front of an
intersection (or all the vehicles before it have a scheduled stop), SUMO counts the number of steps the vehicle's velocity
stays below 0.1m/s. These steps are the "waiting time". In the case the
vehicle moves with a larger speed, this counter is reset. In the case
the vehicle waited longer than a certain threshold value (default 300
seconds), the vehicle is assumed to be in grid-lock and teleported onto
the next free edge on its route.
Note that for
vehicles which have a stop as part of their route, the time spent
stopping is not counted towards their waiting time.

If there is enough space on the subsequent edge of
the vehicle's route, the vehicle will be directly assigned (teleported) to the respective
available space. Vehicle insertion will use depart method "free" (anywhere on the lane with the least traffic)
and attempt to insert the vehicle with its maximum allowed speed.
The insertion space must allow for all necessary safety gaps of the vehicle itself and its follower vehicle, though the speed may be reduced if it helps with insertion.

If the vehicle cannot be inserted immediately,it will be held outside the road network in a special 'teleporting-buffer' and (virtually) traverse the next edge with the average speed of that edge (minimum  m/s). After this virtual travel time has passed, the next attempt at re-inserting the vehicle into the network is made one edge further along its route. This procedure repeats until the vehicle has been re-inserted or the end of the route is reached.
In the latter case, the vehicle is removed from the simulation.
While the vehicle is in the teleporting buffer it remains invisible.

The threshold value can be configure
using the option **--time-to-teleport** {{DT_INT}} which sets the time in seconds.

!!! note
    Setting **--time-to-teleport** to a negative value disables teleporting due to gridlock.

There are different reasons why a vehicle cannot continue with its
route. Every time a vehicle teleports due to grid-lock one of the
following reasons is given:

- **wrong lane**: The vehicle is stuck on a lane which has no
  connection to the next edge on its route.
  - **highway** special case of *wrong lane* where the lane had a speed limit above threshold **--time-to-teleport.highways.min-speed** (default 69km/h)
- **yield** The vehicle is stuck on a low-priority road and did not
  find a gap in the prioritized traffic
- **jam** The vehicle is stuck on a priority road and there is no
  space on the next edge.
- **blocked** The vehicle was standing behind a [stopped](../Definition_of_Vehicles%2C_Vehicle_Types%2C_and_Routes.md#stops_and_waypoints) vehicle
- **disconnected** The current and next edge of the vehicle are not connected (for the vClass of the vehicle)
- **bidi** The current lane is usable in both directions which means the vehicle may be blocked by oncoming traffic
- **railSignal** The vehicle was waiting at a red rail signal with a circular dependency ([deadlock](Railways.md#deadlocks))


Related options are

- **--max-num-teleport**
- **--time-to-teleport.highways**: teleport earlier when stuck on the wrong lane of a road with speed above 19.167 m/s
- **--time-to-teleport.highways.min-speed**: configure threshold for above option
- **--time-to-teleport.disconnected**: teleport earlier when the route is disconnected
- **--time-to-teleport.bidi**: teleport earlier when on a bidi-edge (as this is more prone to dead-lock)
- **--time-to-teleport.remove-constraint**: resolve a constraint-based [railsignal-deadlock](Railways.md#deadlocks) by removing a constraint
- **--time-to-teleport.railsignal-deadlock**: teleport earlier when stopped due to a [railsignal-deadlock](Railways.md#deadlocks)
- **--time-to-teleport.ride**: teleports [persons that are waiting for a ride](../Specification/Persons.md#riding) rather than vehicles.
- **--time-to-teleport.remove**: remove teleporting vehicles directly

Unfortunately, grid-locks are rather common in congested simulation
scenarios. You can solve this only by [improving traffic flow, either by
correcting junction priorities, traffic light timings or the traffic
demand (route
files)](../FAQ.md#the_simulation_has_lots_of_jamsdeadlocks_what_can_i_do).

Also, besides plain grid-locks, the imperfection of the lane-change
model sometimes leads to a situation where two vehicles try to get to
the other lane, and each vehicle is blocking the other one. The
simulation behaves as described earlier. There are two common causes of
this:

- [Motorway weaving sections (combined on- and off-ramps). This can be
  solved with additional
  connections](../Simulation/Motorways.md#combined_on-off-ramps)
- Vehicles inserted on the wrong lane close to an intersection with
  turning lanes. This is best solved by setting the vehicle attribute `departLane="best"`
  to ensure that vehicles are inserted on the correct lane for
  following their route.

## Collisions

By default, SUMO uses a collision-free model. However, due to bugs,
network problems or deliberate configuration,
[collisions may occur](../Simulation/Safety.md#collisions). The default behavior of SUMO is to immediately teleport the rear vehicle onto the next edge of its route (or remove it, when already on its final edge). This behavior can be [configured to avoid or delay teleporting](../Simulation/Safety.md#collisions).

To avoid collisions, observe the simulation
[sumo-gui](../sumo-gui.md) at the location and time of the
collision. Check if some of the [things that may cause a collision are present
in your network](../Simulation/Safety.md#deliberately_causing_collisions).

# What is happening while a vehicle teleports

A teleported vehicle is removed from the network. It is then moved along
its route, but no longer being on the street. It is reinserted into the
network as soon as this becomes possible. While being teleported, the
vehicle is moved along its route with the average speed of the edge it
was removed from or - later - it is currently "passing". The vehicle is
reinserted into the network if there is enough place to be placed on a
lane which allows to continue its drive. Consequently a vehicle may
teleport multiple times within one simulation.

!!! note
    Outputs react differently to teleporting vehicles. While [meanData-output](Output/Lane-_or_Edge-based_Traffic_Measures.md) counts teleports explicitly, [fcd-output](Output/FCDOutput.md) does not record vehicles while they are teleporting.
