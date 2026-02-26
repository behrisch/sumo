#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
# Copyright (C) 2008-2026 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    runner.py
# @author  Michael Behrisch
# @author  Daniel Krajzewicz
# @author  Jakob Erdmann
# @date    2011-03-04


from __future__ import print_function
from __future__ import absolute_import
import os
import sys

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
import traci  # noqa
import sumolib  # noqa


def checkVehicleStates():
    print("time", traci.simulation.getTime())
    print("#loaded", traci.simulation.getLoadedNumber())
    print("loaded", traci.simulation.getLoadedIDList())
    print("#departed", traci.simulation.getDepartedNumber())
    print("departed", traci.simulation.getDepartedIDList())
    print("#arrived", traci.simulation.getArrivedNumber())
    print("arrived", traci.simulation.getArrivedIDList())
    print("#parkstart", traci.simulation.getParkingStartingVehiclesNumber())
    print("parkstart", traci.simulation.getParkingStartingVehiclesIDList())
    print("#parkend", traci.simulation.getParkingEndingVehiclesNumber())
    print("parkend", traci.simulation.getParkingEndingVehiclesIDList())
    print("#stopstart", traci.simulation.getStopStartingVehiclesNumber())
    print("stopstart", traci.simulation.getStopStartingVehiclesIDList())
    print("#stopend", traci.simulation.getStopEndingVehiclesNumber())
    print("stopend", traci.simulation.getStopEndingVehiclesIDList())
    print("#colliding", traci.simulation.getCollidingVehiclesNumber())
    print("colliding", traci.simulation.getCollidingVehiclesIDList())
    print("#emergencystopping", traci.simulation.getEmergencyStoppingVehiclesNumber())
    print("emergencystopping", traci.simulation.getEmergencyStoppingVehiclesIDList())
    print("min#expected", traci.simulation.getMinExpectedNumber())
    print("#teleportStart", traci.simulation.getStartingTeleportNumber())
    print("teleportStart", traci.simulation.getStartingTeleportIDList())
    print("#teleportEnd", traci.simulation.getEndingTeleportNumber())
    print("teleportEnd", traci.simulation.getEndingTeleportIDList())


def ppStages(comment, stages):
    print("%s\n  %s\n" % (comment, "\n  ".join(map(str, stages))))


print("loaded?", traci.isLoaded())
version = traci.start([sumolib.checkBinary('sumo'), "-c", "sumo.sumocfg",
                       "--ignore-route-errors",
                       "--end", "42",
                       "--log", "log.txt"])
print("version at start", version)
print("version", traci.getVersion())
print("loaded?", traci.isLoaded())
print("endTime", traci.simulation.getEndTime())
print("net file", traci.simulation.getOption("net-file"))

traci.simulation.subscribe(
    [traci.constants.VAR_LOADED_VEHICLES_IDS, traci.constants.VAR_DEPARTED_VEHICLES_IDS])
print(traci.simulation.getSubscriptionResults())
for step in range(6):
    print("step", step)
    traci.simulationStep()
    print("pendingVehicles", traci.simulation.getPendingVehicles())
    print(traci.simulation.getSubscriptionResults())
checkVehicleStates()
print("deltaT", traci.simulation.getDeltaT())
print("boundary", traci.simulation.getNetBoundary())
print("convertRoad2D", traci.simulation.convert2D("o", 0.))
print("convertRoad3D", traci.simulation.convert3D("o", 0.))
print("convertRoadGeo", traci.simulation.convert2D("o", 0., toGeo=True))
print("convertRoadGeoAlt", traci.simulation.convert3D("o", 0., toGeo=True))
print("convertRoad2D length", traci.simulation.convert2D("2o", 8000.))
print("convert2DGeo", traci.simulation.convertGeo(488.65, 501.65))
print("convertGeo2D", traci.simulation.convertGeo(12, 48, True))
print("convert2DRoad", traci.simulation.convertRoad(488.65, 501.65))
print("convertGeoRoad", traci.simulation.convertRoad(12, 48.1, True))
print("convertGeoRoadBus", traci.simulation.convertRoad(12, 48.1, True, "bus"))
traci.lane.setDisallowed("o_0", ["bus"])
print("convertGeoRoadBusDisallowed", traci.simulation.convertRoad(12, 48.1, True, "bus"))

pos1 = (488.65, 501.65)
pos2 = (498.65, 501.65)
print("distance2D", traci.simulation.getDistance2D(pos1[0], pos1[1], pos2[0], pos2[1]))
pos1geo = traci.simulation.convertGeo(*pos1)
pos2geo = traci.simulation.convertGeo(*pos2)
print("distance2Dgeo", pos1geo, pos2geo,
      traci.simulation.getDistance2D(pos1geo[0], pos1geo[1],
                                     pos2geo[0], pos2geo[1], isGeo=True))

print("drivingDistance2D", traci.simulation.getDistance2D(
    488.65, 501.65, 498.65, 501.65, isDriving=True))


print("distanceRoad", traci.simulation.getDistanceRoad("o", 0., "2o", 0.))
print("drivingDistanceRoad", traci.simulation.getDistanceRoad(
    "o", 0., "2o", 0., isDriving=True))
print("clearing pending")
traci.simulation.clearPending()
print("save simstate")
traci.simulation.saveState("state.xml")
try:
    traci.simulation.setParameter("foo", "foo.bla", "42")
except traci.TraCIException:
    pass
traci.simulation.setParameter("", "bar", "42")
print("getParameter (generic)", traci.simulation.getParameter("", "bar"))

try:
    print("getParameter", traci.simulation.getParameter("foo", "foo.bla"))
except traci.TraCIException:
    pass
try:
    print("getParameter", traci.simulation.getParameter("cs1", "chargingStation.bla"))
except traci.TraCIException:
    pass
try:
    print("getParameter", traci.simulation.getParameter("foo", "chargingStation.totalEnergyCharged"))
except traci.TraCIException:
    pass
print("getParameter charginStation.totalEnergyCharged",
      traci.simulation.getParameter("cs1", "chargingStation.totalEnergyCharged"))
print("getParameter chargingStation.name", traci.simulation.getParameter("cs1", "chargingStation.name"))
print("getParameter chargingStation.lane", traci.simulation.getParameter("cs1", "chargingStation.lane"))
print("getParameter chargingStation.key1", traci.simulation.getParameter("cs1", "chargingStation.key1"))

print("getParameter parkingArea.capacity", traci.simulation.getParameter("pa1", "parkingArea.capacity"))
print("getParameter parkingArea.occupancy", traci.simulation.getParameter("pa1", "parkingArea.occupancy"))
print("getParameter parkingArea.name", traci.simulation.getParameter("pa1", "parkingArea.name"))
print("getParameter parkingArea.lane", traci.simulation.getParameter("pa1", "parkingArea.lane"))
print("getParameter parkingArea.key3", traci.simulation.getParameter("pa1", "parkingArea.key3"))

print("getParameter busStop.name", traci.simulation.getParameter("bs", "busStop.name"))
print("getParameter busStop.lane", traci.simulation.getParameter("bs", "busStop.lane"))
print("getParameter busStop.key2", traci.simulation.getParameter("bs", "busStop.key2"))

print("getParameter stats.vehicles.loaded", traci.simulation.getParameter("", "stats.vehicles.loaded"))
print("getParameter stats.vehicles.inserted", traci.simulation.getParameter("", "stats.vehicles.inserted"))
print("getParameter stats.vehicles.running", traci.simulation.getParameter("", "stats.vehicles.running"))
print("getParameter stats.vehicles.waiting", traci.simulation.getParameter("", "stats.vehicles.waiting"))
print("getParameter stats.teleports.total", traci.simulation.getParameter("", "stats.teleports.total"))
print("getParameter stats.teleports.jam", traci.simulation.getParameter("", "stats.teleports.jam"))
print("getParameter stats.teleports.yield", traci.simulation.getParameter("", "stats.teleports.yield"))
print("getParameter stats.teleports.wrongLane", traci.simulation.getParameter("", "stats.teleports.wrongLane"))
print("getParameter stats.safety.collisions", traci.simulation.getParameter("", "stats.safety.collisions"))
print("getParameter stats.safety.emergencyStops", traci.simulation.getParameter("", "stats.safety.emergencyStops"))
print("getParameter stats.safety.emergencyBraking", traci.simulation.getParameter("", "stats.safety.emergencyBraking"))
print("getParameter stats.persons.loaded", traci.simulation.getParameter("", "stats.persons.loaded"))
print("getParameter stats.persons.running", traci.simulation.getParameter("", "stats.persons.running"))
print("getParameter stats.persons.jammed", traci.simulation.getParameter("", "stats.persons.jammed"))
print("getParameter stats.personTeleports.total", traci.simulation.getParameter("", "stats.personTeleports.total"))
print("getParameter stats.personTeleports.abortWait", traci.simulation.getParameter("", "stats.personTeleports.abortWait"))  # noqa
print("getParameter stats.personTeleports.wrongDest", traci.simulation.getParameter("", "stats.personTeleports.wrongDest"))  # noqa

print("getParameter buildConfig.ARROW_SO_VERSION", traci.simulation.getParameter("", "buildConfig.ARROW_SO_VERSION"))
print("getParameter buildConfig.JPS_VERSION", traci.simulation.getParameter("", "buildConfig.JPS_VERSION"))
print("getParameter buildConfig.ENABLED", traci.simulation.getParameter("", "buildConfig.ENABLED"))

try:
    print("getBusStopWaiting", traci.simulation.getBusStopWaiting("foo"))
except traci.TraCIException:
    pass

print("getBusStopIDList", traci.simulation.getBusStopIDList())
print("getBusStopWaiting", traci.simulation.getBusStopWaiting("bs"))
print("getBusStopWaitingIDList", traci.simulation.getBusStopWaitingIDList("bs"))

try:
    print("findRoute", traci.simulation.findRoute("foo", "fup"))
except traci.TraCIException:
    pass
print("findRoute", traci.simulation.findRoute("o", "2o"))
print("findRoute with routing mode", traci.simulation.findRoute(
    "o", "2o", routingMode=traci.constants.ROUTING_MODE_AGGREGATED))
try:
    print("findRoute", traci.simulation.findRoute("footpath", "footpath2", "DEFAULT_VEHTYPE"))
except traci.TraCIException:
    pass

try:
    print("findIntermodalRoute", traci.simulation.findIntermodalRoute("foo", "fup"))
except traci.TraCIException:
    pass
print("findIntermodalRoute", traci.simulation.findIntermodalRoute(
    "footpath", "footpath2", "bicycle", vType="DEFAULT_BIKETYPE"))
ppStages("findIntermodalRoute (walk)", traci.simulation.findIntermodalRoute("o", "2o"))
ppStages("findIntermodalRoute (bike)", traci.simulation.findIntermodalRoute("o", "2o", modes="bicycle"))
ppStages("findIntermodalRoute (car)", traci.simulation.findIntermodalRoute("o", "2o", modes="car"))
ppStages("findIntermodalRoute (bike,car,public)",
         traci.simulation.findIntermodalRoute("o", "2o", modes="car bicycle public"))

try:
    print("findIntermodalRoute", traci.simulation.findIntermodalRoute("o", "2o", departPos=1e5))
except traci.TraCIException:
    pass

traci.vehicle.setSpeedMode("emergencyStopper", 0)
traci.vehicle.setSpeed("emergencyStopper", 100)
try:
    traci.simulation.subscribeContext("",
                                      traci.constants.CMD_GET_VEHICLE_VARIABLE, 42,
                                      [traci.constants.VAR_POSITION])
    print("contextSubscriptions:", traci.simulation.getAllContextSubscriptionResults())
except traci.TraCIException:
    pass

traci.simulation.setScale(1.5)
print("scale", traci.simulation.getScale())

for step in range(12):
    print("step", step)
    traci.simulationStep()
    if traci.simulation.getCollidingVehiclesNumber() > 0:
        print("detected collision")
        checkVehicleStates()
    if traci.simulation.getEmergencyStoppingVehiclesNumber() > 0:
        print("detected emergency stop")
        checkVehicleStates()
    print(traci.simulation.getSubscriptionResults())

print("check whether GUI is present", traci.hasGUI())
traci.simulation.writeMessage("custom log message")
traci.simulationStep()
print("loaded?", traci.isLoaded())
traci.close()
print("loaded?", traci.isLoaded())
