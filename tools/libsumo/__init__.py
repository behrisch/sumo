# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
# Copyright (C) 2018-2025 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    __init__.py
# @author  Benjamin Striner
# @author  Michael Behrisch
# @date    2018-06-05

import os
import sys
import warnings
SUMO_WHEEL_HOME = None
try:
    import sumo
    if not hasattr(sumo, "SUMO_HOME"):  # maybe it is not the correct module
        raise ImportError()
    SUMO_WHEEL_HOME = sumo.SUMO_HOME
    if "SUMO_HOME" not in os.environ:
        os.environ["SUMO_HOME"] = SUMO_WHEEL_HOME
except ImportError:
    if "SUMO_HOME" not in os.environ:
        warnings.warn("SUMO_HOME is not set and the eclipse-sumo wheel is not installed!")
if hasattr(sys, "_is_gil_enabled") and not sys._is_gil_enabled():
    warnings.warn("This package has not been validated in free-threaded Python (no-GIL mode).")
if hasattr(os, "add_dll_directory"):
    # since Python 3.8 the DLL search path has to be set explicitly see https://bugs.python.org/issue43173
    if SUMO_WHEEL_HOME and os.path.exists(os.path.join(SUMO_WHEEL_HOME, "bin", "zlib.dll")):
        os.add_dll_directory(os.path.abspath(os.path.join(SUMO_WHEEL_HOME, "bin")))
    elif "SUMO_HOME" in os.environ and os.path.exists(os.path.join(os.environ["SUMO_HOME"], "bin", "zlib.dll")):
        os.add_dll_directory(os.path.join(os.environ["SUMO_HOME"], "bin"))
    elif os.path.exists(os.path.join(os.path.dirname(__file__), "..", "..", "bin", "zlib.dll")):
        os.add_dll_directory(os.path.join(os.path.dirname(__file__), "..", "..", "bin"))

from traci import connection, constants, exceptions, _vehicle, _person, _trafficlight, _simulation  # noqa
from traci.step import StepManager, StepListener  # noqa
from .libsumo import vehicle, simulation, person, trafficlight, edge  # noqa
from .libsumo import TraCIStage, TraCINextStopData, TraCIReservation, TraCILogic, TraCIPhase, TraCIException  # noqa
from .libsumo import TraCICollision, TraCISignalConstraint  # noqa
from ._libsumo import TraCILogic_phases_get, TraCILogic_phases_set, TraCILogic_swiginit, new_TraCILogic  # noqa
from .libsumo import *  # noqa

DOMAINS = [
    busstop,  # noqa
    calibrator,  # noqa
    chargingstation,  # noqa
    edge,
    gui,  # noqa
    inductionloop,  # noqa
    junction,  # noqa
    lane,  # noqa
    lanearea,  # noqa
    meandata,  # noqa
    multientryexit,  # noqa
    overheadwire,  # noqa
    parkingarea,  # noqa
    person,
    poi,  # noqa
    polygon,  # noqa
    rerouter,  # noqa
    route,  # noqa
    routeprobe,  # noqa
    simulation,
    trafficlight,
    variablespeedsign,  # noqa
    vehicle,
    vehicletype,  # noqa
]

_stepManager = StepManager()


def wrapAsClassMethod(func, module):
    def wrapper(*args, **kwargs):
        return func(module, *args, **kwargs)
    return wrapper


TraCIStage.__attr_repr__ = _simulation.Stage.__attr_repr__
TraCIStage.__repr__ = _simulation.Stage.__repr__
TraCIStage.toXML = _simulation.Stage.toXML
TraCICollision.__attr_repr__ = _simulation.Collision.__attr_repr__
TraCICollision.__repr__ = _simulation.Collision.__repr__

TraCINextStopData.__attr_repr__ = _vehicle.StopData.__attr_repr__
TraCINextStopData.__repr__ = _vehicle.StopData.__repr__

TraCIReservation.__attr_repr__ = _person.Reservation.__attr_repr__
TraCIReservation.__repr__ = _person.Reservation.__repr__


def set_phases(self, phases):
    new_phases = [TraCIPhase(p.duration, p.state, p.minDur, p.maxDur, p.next, p.name) for p in phases]
    TraCILogic_phases_set(self, new_phases)


def TraCILogic__init__(self, *args, **kwargs):
    # Extract known keyword arguments or set to None if not provided
    programID = kwargs.get('programID', args[0] if len(args) > 0 else None)
    type_ = kwargs.get('type',  args[1] if len(args) > 1 else None)
    currentPhaseIndex = kwargs.get('currentPhaseIndex',  args[2] if len(args) > 2 else None)
    phases = kwargs.get('phases',  args[3] if len(args) > 3 else None)
    # subParameter = kwargs.get('subParameter',  args[4] if len(args) > 4 else None)

    # Update phases if provided
    if phases:
        new_phases = [TraCIPhase(p.duration, p.state, p.minDur, p.maxDur, p.next, p.name) for p in phases]
        phases = new_phases

    # Rebuild args including the extracted keyword arguments
    args = (programID, type_, currentPhaseIndex, phases)

    # Initialize with the original function
    TraCILogic_swiginit(self, new_TraCILogic(*args))


# Override methods and properties
TraCILogic.__init__ = TraCILogic__init__
TraCILogic.phases = property(TraCILogic_phases_get, set_phases)
TraCILogic.getPhases = _trafficlight.Logic.getPhases
TraCILogic.__repr__ = _trafficlight.Logic.__repr__
TraCIPhase.__repr__ = _trafficlight.Phase.__repr__
TraCISignalConstraint.__repr__ = _trafficlight.Constraint.__repr__


exceptions.TraCIException = TraCIException
simulation.Stage = TraCIStage
vehicle.StopData = TraCINextStopData
person.Reservation = TraCIReservation
trafficlight.Phase = TraCIPhase
trafficlight.Logic = TraCILogic
vehicle.addFull = vehicle.add
vehicle.addLegacy = wrapAsClassMethod(_vehicle.VehicleDomain.addLegacy, vehicle)
vehicle.couldChangeLane = wrapAsClassMethod(_vehicle.VehicleDomain.couldChangeLane, vehicle)
vehicle.wantsAndCouldChangeLane = wrapAsClassMethod(_vehicle.VehicleDomain.wantsAndCouldChangeLane, vehicle)
vehicle.isStopped = wrapAsClassMethod(_vehicle.VehicleDomain.isStopped, vehicle)
vehicle.isStoppedParking = wrapAsClassMethod(_vehicle.VehicleDomain.isStoppedParking, vehicle)
vehicle.isStoppedTriggered = wrapAsClassMethod(_vehicle.VehicleDomain.isStoppedTriggered, vehicle)
vehicle.isAtBusStop = wrapAsClassMethod(_vehicle.VehicleDomain.isAtBusStop, vehicle)
vehicle.isAtContainerStop = wrapAsClassMethod(_vehicle.VehicleDomain.isAtContainerStop, vehicle)
vehicle.setBusStop = wrapAsClassMethod(_vehicle.VehicleDomain.setBusStop, vehicle)
vehicle.setContainerStop = wrapAsClassMethod(_vehicle.VehicleDomain.setContainerStop, vehicle)
vehicle.setParkingAreaStop = wrapAsClassMethod(_vehicle.VehicleDomain.setParkingAreaStop, vehicle)
vehicle.setChargingStationStop = wrapAsClassMethod(_vehicle.VehicleDomain.setChargingStationStop, vehicle)
vehicle.getRightFollowers = wrapAsClassMethod(_vehicle.VehicleDomain.getRightFollowers, vehicle)
vehicle.getRightLeaders = wrapAsClassMethod(_vehicle.VehicleDomain.getRightLeaders, vehicle)
vehicle.getLeftFollowers = wrapAsClassMethod(_vehicle.VehicleDomain.getLeftFollowers, vehicle)
vehicle.getLeftLeaders = wrapAsClassMethod(_vehicle.VehicleDomain.getLeftLeaders, vehicle)
vehicle.getLaneChangeStatePretty = wrapAsClassMethod(_vehicle.VehicleDomain.getLaneChangeStatePretty, vehicle)
vehicle._legacyGetLeader = True
person.removeStages = wrapAsClassMethod(_person.PersonDomain.removeStages, person)
_trafficlight.TraCIException = TraCIException
trafficlight.setLinkState = wrapAsClassMethod(_trafficlight.TrafficLightDomain.setLinkState, trafficlight)
trafficlight.getNemaPhaseCalls = wrapAsClassMethod(_trafficlight.TrafficLightDomain.getNemaPhaseCalls, trafficlight)
trafficlight.setNemaSplits = wrapAsClassMethod(_trafficlight.TrafficLightDomain.setNemaSplits, trafficlight)
trafficlight.setNemaOffset = wrapAsClassMethod(_trafficlight.TrafficLightDomain.setNemaOffset, trafficlight)


def addStepListener(listener):
    return _stepManager.addStepListener(listener)


def removeStepListener(listenerID):
    return _stepManager.removeStepListener(listenerID)


def isLibsumo():
    return True


def isLibtraci():
    return False


def init(port):
    print("Warning! To make your code usable with traci and libsumo, please use traci.start instead of traci.init.")


hasGUI = simulation.hasGUI
load = simulation.load
isLoaded = simulation.isLoaded
getVersion = simulation.getVersion
executeMove = simulation.executeMove

_libsumo_step = simulation.step


def simulationStep(time=0.):
    _libsumo_step(time)
    _stepManager.manageStepListeners(time)


simulation.step = simulationStep


def close():
    simulation.close()
    _stepManager.close()


def start(cmd, port=None, numRetries=constants.DEFAULT_NUM_RETRIES, label="default", verbose=False,
          traceFile=None, traceGetters=True, stdout=None, doSwitch=True):
    if port is not None:
        print("Warning! To make your code usable with traci and libsumo, do not set an explicit port.")
    version = simulation.start(cmd)
    if traceFile is not None:
        if _stepManager.startTracing(traceFile, traceGetters, DOMAINS):
            # simulationStep shows up as simulation.step
            global _libsumo_step
            _libsumo_step = _stepManager._addTracing(_libsumo_step, "simulation")
        _stepManager._traceFile.write("import traci\n")
        _stepManager.write("start", repr(cmd))
    return version


def setLegacyGetLeader(enabled):
    vehicle._legacyGetLeader = enabled
