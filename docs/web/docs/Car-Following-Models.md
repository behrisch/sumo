---
title: Car-Following-Models
---

# Car-Following Models

This overview page collects information regarding car-following models
in [sumo](sumo.md). It is still a stub and needs be expanded.

## Links

- [Publications on car-following models](Publications.md#publications_using_sumo)
- [How to implement a new car-following model](Developer/How_To/Car-Following_Model.md)
- [More implementation notes on car-following models](Developer/Implementation_Notes/Vehicle_Models.md)

## Model Parameters

Most car-following models have their own [set of parameters](Definition_of_Vehicles,_Vehicle_Types,_and_Routes.md#car-following_model_parameters).
Because some models have a common ancestry they share some parameters.
These are described below.

### Reaction time

By default, drivers may react to their surroundings in every simulation step (i.e. by changing their speed or changing lanes).
Thus, their time to react, is equal to the value set by sumo-option **--step-length** (default 1s).
To change the frequency of decision making and thereby delay reaction, the parameter [actionStepLength](#actionsteplength) may be set.

### actionStepLength

This parameter is used to decouple the simulation step length (option **--step-length**)
from the frequency of driver decision making. It can be set either in
the `<vType>`-definition as attribute *actionStepLength* or by setting a default
value for all vTypes using option **--default.action-step-length** {{DT_FLOAT}}.

By default, the action step length is equal to the simulation step
length which works well for the default step length of 1s. When
performing sub-second simulation by setting a lower step-length value,
it may be useful to maintain a higher action step length in order to
model reaction times and also in order to reduce computational demand
and thus speed up the simulation.

!!! note
    The action step length works similar to a reaction time (vehicle will not react immediately to changes in their environment) but it also differs from a "true" reaction time because whenever a vehicle has its action step it reacts to the state in the previous simulation step rather than to the state that was seen in their previous action step. Thus the Perception-Reaction loop is less frequent but still as fast as the simulation step length.

### tau

This parameter is intended to model a drivers desired minimum time headway (in
seconds). It is used by all models. Drivers attempt to maintain a
minimum time gap of tau between the rear bumper of their leader and
their own (front-bumper + minGap) to assure the possibility to brake in
time when their leader starts braking.

The use of minGap with respect to the desired (minimum) time headway varies by model. I.e. the Krauss model selects a speed that ensures minGap can always be maintained whereas other models may not do this.

As explained [above](#actionsteplength), the reaction time is defined by the simulation step length and the **actionStepLength** parameter. fundamentally limited by the simulation step size ([sumo](sumo.md) option **--step-length** {{DT_FLOAT}}).

By setting a value of tau that is higher than the reaction time, the driving safety can be increased since vehicles will use large gaps and have enough time to react. Conversely, setting a value of tau that is lower than the reaction time leads to unsafe driving. In the simulation this can manifest as  high deceleration or even collisions.

### decel, apparentDecel, emergencyDecel

By default, carFollowModels will adapt their driving speed to limit necessary braking to a maximum configured by the **decel** attribute. For the default *Krauss* model this is a hard limit whereas for other models such as *IDM* the decel bound is less strict. The default decel value is 4.5 (in units of m/s^2) for passenger cars and takes a different [default for other vehicle classes](Vehicle_Type_Parameter_Defaults.md)

The "safe" velocity for every simulation step is computed by the configured carFollowModel based on the leading vehicle as well as the right-of-way-rules. To ensure safe driving under various circumstances, the maximum braking capability of the leader vehicle is also taken into account. This value is taken from the **apparentDecel** attribute of the leader vehicle (which defaults to the same value as it's **decel** attribute).

If for some reasons, reaching the safe velocity requires braking beyond the desired deceleration, the vehicle may do so up to a hard limit configured by the attribute **emergencyDecel**. The default emergency deceleration is 9 for passenger cars and takes a different [default for other vehicle classes](Vehicle_Type_Parameter_Defaults.md). The global default can be changed with option **--default.emergencydecel**.

An emergency braking warning is triggered if the braking strength exceeds a threshold value configured by **----emergencydecel.warning-threshold**. By default this threshold is set to *1* which triggers a warning when braking with 100% of **emergencyDecel**. The warnings can be disabled by setting the threshold to a value > 1.
