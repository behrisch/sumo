# -*- coding: utf-8 -*-
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
# Copyright (C) 2011-2026 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    _vehicle.py
# @author  Michael Behrisch
# @author  Lena Kalleske
# @author  Mario Krumnow
# @author  Lena Kalleske
# @author  Jakob Erdmann
# @author  Laura Bieker
# @author  Daniel Krajzewicz
# @author  Leonhard Luecken
# @author  Mirko Barthauer
# @date    2011-03-09

from __future__ import absolute_import
import warnings
from ._vehicletype import VTypeDomain
from . import constants as tc
from .exceptions import TraCIException, deprecated, alias_param
from ._lane import _readLinks


_legacyGetLeader = True


class StopData(object):

    def __init__(self,
                 lane="",
                 startPos=-1,
                 endPos=-1,
                 stoppingPlaceID="",
                 stopFlags=0,
                 duration=-1,
                 until=-1,
                 intendedArrival=-1,
                 arrival=-1,
                 depart=-1,
                 split="",
                 join="",
                 actType="",
                 tripId="",
                 line="",
                 speed=0):
        self.lane = lane
        self.startPos = startPos
        self.endPos = endPos
        self.stoppingPlaceID = stoppingPlaceID
        self.stopFlags = stopFlags
        self.duration = duration
        self.until = until
        self.intendedArrival = intendedArrival
        self.arrival = arrival
        self.depart = depart
        self.split = split
        self.join = join
        self.actType = actType
        self.tripId = tripId
        self.line = line
        self.speed = speed

    def __attr_repr__(self, attrname, default=""):
        if getattr(self, attrname) == default:
            return ""
        else:
            val = getattr(self, attrname)
            if val == tc.INVALID_DOUBLE_VALUE:
                val = "INVALID"
            return "%s=%s" % (attrname, val)

    def __repr__(self):
        return "StopData(%s)" % ', '.join([v for v in [
            self.__attr_repr__("lane"),
            self.__attr_repr__("startPos"),
            self.__attr_repr__("endPos"),
            self.__attr_repr__("stoppingPlaceID"),
            self.__attr_repr__("stopFlags"),
            self.__attr_repr__("duration", tc.INVALID_DOUBLE_VALUE),
            self.__attr_repr__("until", tc.INVALID_DOUBLE_VALUE),
            self.__attr_repr__("intendedArrival", tc.INVALID_DOUBLE_VALUE),
            self.__attr_repr__("arrival", tc.INVALID_DOUBLE_VALUE),
            self.__attr_repr__("depart", tc.INVALID_DOUBLE_VALUE),
            self.__attr_repr__("split"),
            self.__attr_repr__("join"),
            self.__attr_repr__("actType"),
            self.__attr_repr__("tripId"),
            self.__attr_repr__("line"),
            self.__attr_repr__("speed", 0),
        ] if v != ""])


def _readStopData(result):
    result.read("!iB")  # numCompounds, TYPE_INT
    numStops = result.read("!i")[0]
    nextStops = []
    for _ in range(numStops):
        lane = result.readTypedString()
        endPos = result.readTypedDouble()
        stoppingPlaceID = result.readTypedString()
        stopFlags = result.readTypedInt()
        duration = result.readTypedDouble()
        until = result.readTypedDouble()
        startPos = result.readTypedDouble()
        intendedArrival = result.readTypedDouble()
        arrival = result.readTypedDouble()
        depart = result.readTypedDouble()
        split = result.readTypedString()
        join = result.readTypedString()
        actType = result.readTypedString()
        tripId = result.readTypedString()
        line = result.readTypedString()
        speed = result.readTypedDouble()
        nextStops.append(StopData(lane, startPos, endPos, stoppingPlaceID,
                                  stopFlags, duration, until, intendedArrival, arrival, depart, split, join,
                                  actType, tripId, line, speed))
    return tuple(nextStops)


def _readBestLanes(result):
    result.read("!iB")
    nbLanes = result.read("!i")[0]  # Length
    lanes = []
    for _ in range(nbLanes):
        laneID = result.readTypedString()
        length, occupation, offset = result.read("!BdBdBb")[1::2]
        allowsContinuation = bool(result.read("!BB")[1])
        numNextLanes = result.read("!Bi")[1]
        nextLanes = [result.readString() for __ in range(numNextLanes)]
        lanes.append((laneID, length, occupation, offset, allowsContinuation, tuple(nextLanes)))
    return tuple(lanes)


def _readLeader(result):
    assert result.read("!i")[0] == 2  # compound size
    vehicleID = result.readTypedString()
    dist = result.readTypedDouble()
    if vehicleID == "" and _legacyGetLeader:
        return None
    return vehicleID, dist


def _readFollower(result):
    # note: merge this with _readLeader once the default of _legacyGetLeader has
    # been changed to False
    assert result.read("!i")[0] == 2  # compound size
    vehicleID = result.readTypedString()
    dist = result.readTypedDouble()
    return vehicleID, dist


def _readNeighbors(result):
    """ result has structure:
    byte(TYPE_COMPOUND) | length(neighList) | Per list entry: string(vehID) | double(dist)
    """
    num = result.readInt()  # length of the vehicle list
    neighs = []
    for _ in range(num):
        vehID = result.readString()
        dist = result.readDouble()
        neighs.append((vehID, dist))
    return tuple(neighs)


def _readNextTLS(result):
    result.read("!iB")  # numCompounds, TYPE_INT
    numTLS = result.read("!i")[0]
    nextTLS = []
    for _ in range(numTLS):
        result.read("!B")
        tlsID = result.readString()
        tlsIndex, dist, state = result.read("!BiBdBB")[1::2]
        nextTLS.append((tlsID, tlsIndex, dist, chr(state)))
    return tuple(nextTLS)


def _readNextStops(result):
    result.read("!iB")  # numCompounds, TYPE_INT
    numStops = result.read("!i")[0]
    nextStop = []
    for _ in range(numStops):
        result.read("!B")
        lane = result.readString()
        result.read("!B")
        endPos = result.readDouble()
        result.read("!B")
        stoppingPlaceID = result.readString()
        result.read("!B")
        stopFlags = result.readInt()
        result.read("!B")
        duration = result.readDouble()
        result.read("!B")
        until = result.readDouble()
        nextStop.append((lane, endPos, stoppingPlaceID, stopFlags, duration, until))
    return tuple(nextStop)


def _readJunctionFoes(result):
    result.read("!Bi")
    nbJunctionFoes = result.readInt()
    junctionFoes = []
    for _ in range(nbJunctionFoes):
        result.read("!B")
        foeId = result.readString()
        result.read("!B")
        egoDist = result.readDouble()
        result.read("!B")
        foeDist = result.readDouble()
        result.read("!B")
        egoExitDist = result.readDouble()
        result.read("!B")
        foeExitDist = result.readDouble()
        result.read("!B")
        egoLane = result.readString()
        result.read("!B")
        foeLane = result.readString()
        result.read("!B")
        egoResponse = bool(result.read("!B")[0])
        result.read("!B")
        foeResponse = bool(result.read("!B")[0])
        junctionFoes.append((foeId, egoDist, foeDist, egoExitDist, foeExitDist,
                             egoLane, foeLane, egoResponse, foeResponse))
    return tuple(junctionFoes)


_RETURN_VALUE_FUNC = {tc.VAR_ROUTE_VALID: lambda result: bool(result.read("!i")[0]),
                      tc.VAR_BEST_LANES: _readBestLanes,
                      tc.VAR_LEADER: _readLeader,
                      tc.VAR_FOLLOWER: _readFollower,
                      tc.VAR_NEIGHBORS: _readNeighbors,
                      tc.VAR_NEXT_TLS: _readNextTLS,
                      tc.VAR_NEXT_STOPS: _readNextStops,
                      tc.VAR_NEXT_LINKS: _readLinks,
                      tc.VAR_NEXT_STOPS2: _readStopData,
                      tc.VAR_FOES: _readJunctionFoes,
                      # ignore num compounds and type int
                      tc.CMD_CHANGELANE: lambda result: result.read("!iBiBi")[2::2]}


class VehicleDomain(VTypeDomain):
    # imported for backwards compatibility
    STOP_DEFAULT = tc.STOP_DEFAULT
    STOP_PARKING = tc.STOP_PARKING
    STOP_TRIGGERED = tc.STOP_TRIGGERED
    STOP_CONTAINER_TRIGGERED = tc.STOP_CONTAINER_TRIGGERED
    STOP_BUS_STOP = tc.STOP_BUS_STOP
    STOP_CONTAINER_STOP = tc.STOP_CONTAINER_STOP
    STOP_CHARGING_STATION = tc.STOP_CHARGING_STATION
    STOP_PARKING_AREA = tc.STOP_PARKING_AREA
    DEPART_TRIGGERED = tc.DEPARTFLAG_TRIGGERED
    DEPART_CONTAINER_TRIGGERED = tc.DEPARTFLAG_CONTAINER_TRIGGERED
    DEPART_NOW = tc.DEPARTFLAG_NOW
    DEPART_SPEED_RANDOM = tc.DEPARTFLAG_SPEED_RANDOM
    DEPART_SPEED_MAX = tc.DEPARTFLAG_SPEED_MAX
    DEPART_LANE_RANDOM = tc.DEPARTFLAG_LANE_RANDOM
    DEPART_LANE_FREE = tc.DEPARTFLAG_LANE_FREE
    DEPART_LANE_ALLOWED_FREE = tc.DEPARTFLAG_LANE_ALLOWED_FREE
    DEPART_LANE_BEST_FREE = tc.DEPARTFLAG_LANE_BEST_FREE
    DEPART_LANE_FIRST_ALLOWED = tc.DEPARTFLAG_LANE_FIRST_ALLOWED

    def __init__(self):
        VTypeDomain.__init__(self, "vehicle", tc.CMD_GET_VEHICLE_VARIABLE, tc.CMD_SET_VEHICLE_VARIABLE,
                             tc.CMD_SUBSCRIBE_VEHICLE_VARIABLE, tc.RESPONSE_SUBSCRIBE_VEHICLE_VARIABLE,
                             tc.CMD_SUBSCRIBE_VEHICLE_CONTEXT, tc.RESPONSE_SUBSCRIBE_VEHICLE_CONTEXT,
                             _RETURN_VALUE_FUNC, subscriptionDefault=(tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION))

    def getSpeed(self, vehID):
        """getSpeed(string) -> double

        Returns the (longitudinal) speed in m/s of the named vehicle within the last step.
        """
        return self._getUniversal(tc.VAR_SPEED, vehID)

    def getLateralSpeed(self, vehID):
        """getLateralSpeed(string) -> double

        Returns the lateral speed in m/s of the named vehicle within the last step.
        """
        return self._getUniversal(tc.VAR_SPEED_LAT, vehID)

    def getAcceleration(self, vehID):
        """getAcceleration(string) -> double

        Returns the acceleration in m/s^2 of the named vehicle within the last step.
        """
        return self._getUniversal(tc.VAR_ACCELERATION, vehID)

    def getSpeedWithoutTraCI(self, vehID):
        """getSpeedWithoutTraCI(string) -> double
        Returns the speed that the vehicle would drive if no speed-influencing
        command such as setSpeed or slowDown was given.
        """
        return self._getUniversal(tc.VAR_SPEED_WITHOUT_TRACI, vehID)

    def getPosition(self, vehID):
        """getPosition(string) -> (double, double)

        Returns the position of the named vehicle within the last step [m,m].
        """
        return self._getUniversal(tc.VAR_POSITION, vehID)

    def getPosition3D(self, vehID):
        """getPosition3D(string) -> (double, double, double)

        Returns the position of the named vehicle within the last step [m,m,m].
        """
        return self._getUniversal(tc.VAR_POSITION3D, vehID)

    def getAngle(self, vehID):
        """getAngle(string) -> double

        Returns the angle in degrees of the named vehicle within the last step.
        """
        return self._getUniversal(tc.VAR_ANGLE, vehID)

    def getRoadID(self, vehID):
        """getRoadID(string) -> string

        Returns the id of the edge the named vehicle was at within the last step.
        """
        return self._getUniversal(tc.VAR_ROAD_ID, vehID)

    def getDeparture(self, vehID):
        """getDeparture(string) -> double

        Returns the actual departure time in seconds
        """
        return self._getUniversal(tc.VAR_DEPARTURE, vehID)

    def getDepartDelay(self, vehID):
        """getDepartDelay(string) -> double

        Returns the delay between intended and actual departure in seconds
        """
        return self._getUniversal(tc.VAR_DEPART_DELAY, vehID)

    def getLaneID(self, vehID):
        """getLaneID(string) -> string

        Returns the id of the lane the named vehicle was at within the last step.
        """
        return self._getUniversal(tc.VAR_LANE_ID, vehID)

    def getLaneIndex(self, vehID):
        """getLaneIndex(string) -> integer

        Returns the index of the lane the named vehicle was at within the last step.
        """
        return self._getUniversal(tc.VAR_LANE_INDEX, vehID)

    def getSegmentID(self, vehID):
        """getSegmentID(string) -> string

        Returns the id of the segment the named vehicle was at within the last step (mesosim).
        """
        return self._getUniversal(tc.VAR_SEGMENT_ID, vehID)

    def getSegmentIndex(self, vehID):
        """getSegmentIndex(string) -> integer

        Returns the index of the segment the named vehicle was at within the last step (mesosim).
        """
        return self._getUniversal(tc.VAR_SEGMENT_INDEX, vehID)

    def getTypeID(self, vehID):
        """getTypeID(string) -> string

        Returns the id of the type of the named vehicle.
        """
        return self._getUniversal(tc.VAR_TYPE, vehID)

    def getRouteID(self, vehID):
        """getRouteID(string) -> string

        Returns the id of the route of the named vehicle.
        """
        return self._getUniversal(tc.VAR_ROUTE_ID, vehID)

    def getRouteIndex(self, vehID):
        """getRouteIndex(string) -> int

        Returns the index of the current edge within the vehicles route or -1 if the
        vehicle has not yet departed
        """
        return self._getUniversal(tc.VAR_ROUTE_INDEX, vehID)

    def getRoute(self, vehID):
        """getRoute(string) -> tuple(string)

        Returns the ids of the edges the vehicle's route is made of.
        """
        return self._getUniversal(tc.VAR_EDGES, vehID)

    def getLanePosition(self, vehID):
        """getLanePosition(string) -> double

        The position of the vehicle along the lane measured in m.
        """
        return self._getUniversal(tc.VAR_LANEPOSITION, vehID)

    def getCO2Emission(self, vehID):
        """getCO2Emission(string) -> double

        Returns the CO2 emission in mg/s for the last time step.
        Multiply by the step length to get the value for one step.
        """
        return self._getUniversal(tc.VAR_CO2EMISSION, vehID)

    def getCOEmission(self, vehID):
        """getCOEmission(string) -> double

        Returns the CO emission in mg/s for the last time step.
        Multiply by the step length to get the value for one step.
        """
        return self._getUniversal(tc.VAR_COEMISSION, vehID)

    def getHCEmission(self, vehID):
        """getHCEmission(string) -> double

        Returns the HC emission in mg/s for the last time step.
        Multiply by the step length to get the value for one step.
        """
        return self._getUniversal(tc.VAR_HCEMISSION, vehID)

    def getPMxEmission(self, vehID):
        """getPMxEmission(string) -> double

        Returns the particular matter emission in mg/s for the last time step.
        Multiply by the step length to get the value for one step.
        """
        return self._getUniversal(tc.VAR_PMXEMISSION, vehID)

    def getNOxEmission(self, vehID):
        """getNOxEmission(string) -> double

        Returns the NOx emission in mg/s for the last time step.
        Multiply by the step length to get the value for one step.
        """
        return self._getUniversal(tc.VAR_NOXEMISSION, vehID)

    def getFuelConsumption(self, vehID):
        """getFuelConsumption(string) -> double

        Returns the fuel consumption in mg/s for the last time step.
        Multiply by the step length to get the value for one step.
        """
        return self._getUniversal(tc.VAR_FUELCONSUMPTION, vehID)

    def getNoiseEmission(self, vehID):
        """getNoiseEmission(string) -> double

        Returns the noise emission in db for the last time step.
        """
        return self._getUniversal(tc.VAR_NOISEEMISSION, vehID)

    def getElectricityConsumption(self, vehID):
        """getElectricityConsumption(string) -> double

        Returns the electricity consumption in Wh/s for the last time step.
        Multiply by the step length to get the value for one step.
        """
        return self._getUniversal(tc.VAR_ELECTRICITYCONSUMPTION, vehID)

    def getPersonNumber(self, vehID):
        """getPersonNumber(string) -> integer
        Returns the total number of persons which includes those defined
        using attribute 'personNumber' as well as <person>-objects who are riding in
        this vehicle.
        """
        return self._getUniversal(tc.VAR_PERSON_NUMBER, vehID)

    def getPersonIDList(self, vehID):
        """getPersonIDList(string) -> tuple(string)
        Returns the tuple of persons who are riding in this vehicle.
        """
        return self._getUniversal(tc.LAST_STEP_PERSON_ID_LIST, vehID)

    def getAdaptedTraveltime(self, vehID, time, edgeID):
        """getAdaptedTraveltime(string, double, string) -> double

        Returns the information about the travel time of edge "edgeID" valid
        for the given time from the vehicle's internal edge weights
        container (see setAdaptedTraveltime).
        If there is no individual travel time set, INVALID_DOUBLE_VALUE is returned.
        """
        return self._getUniversal(tc.VAR_EDGE_TRAVELTIME, vehID, "tds", 2, time, edgeID)

    def getEffort(self, vehID, time, edgeID):
        """getEffort(string, double, string) -> double

        Returns the information about the effort needed for edge "edgeID" valid
        for the given time from the vehicle's internal effort
        container (see setEffort).
        If there is no individual travel time set, INVALID_DOUBLE_VALUE is returned.
        """
        return self._getUniversal(tc.VAR_EDGE_EFFORT, vehID, "tds", 2, time, edgeID)

    def isRouteValid(self, vehID):
        """isRouteValid(string) -> bool
        Returns whether the current vehicle route is connected for the vehicle
        class of the given vehicle.
        """
        return self._getUniversal(tc.VAR_ROUTE_VALID, vehID)

    def getSignals(self, vehID):
        """getSignals(string) -> integer

        Returns an integer encoding the state of a vehicle's signals.
        """
        return self._getUniversal(tc.VAR_SIGNALS, vehID)

    def getLateralLanePosition(self, vehID):
        """getLateralLanePosition(string) -> double

        Returns the lateral position of the vehicle on its current lane measured in m.
        """
        return self._getUniversal(tc.VAR_LANEPOSITION_LAT, vehID)

    def getAllowedSpeed(self, vehID):
        """getAllowedSpeed(string) -> double

        Returns the maximum allowed speed on the current lane regarding speed factor in m/s for this vehicle.
        """
        return self._getUniversal(tc.VAR_ALLOWED_SPEED, vehID)

    def getWaitingTime(self, vehID):
        """getWaitingTime(string) -> double
        The waiting time of a vehicle is defined as the time (in seconds) spent with a
        speed below 0.1m/s since the last time it was faster than 0.1m/s.
        (basically, the waiting time of a vehicle is reset to 0 every time it moves).
        A vehicle that is stopping intentionally with a <stop> does not accumulate waiting time.
        """
        return self._getUniversal(tc.VAR_WAITING_TIME, vehID)

    def getAccumulatedWaitingTime(self, vehID):
        """getAccumulatedWaitingTime(string) -> double
        The accumulated waiting time of a vehicle collects the vehicle's waiting time
        over a certain time interval (interval length is set per option '--waiting-time-memory')
        """
        return self._getUniversal(tc.VAR_ACCUMULATED_WAITING_TIME, vehID)

    def getLaneChangeMode(self, vehID):
        """getLaneChangeMode(string) -> integer

        Gets the vehicle's lane change mode as a bitset.
        """
        return self._getUniversal(tc.VAR_LANECHANGE_MODE, vehID)

    def getSpeedMode(self, vehID):
        """getSpeedMode(string) -> int
        The speed mode of a vehicle
        """
        return self._getUniversal(tc.VAR_SPEEDSETMODE, vehID)

    def getSlope(self, vehID):
        """getSlope(string) -> double
        The slope at the current position of the vehicle in degrees
        """
        return self._getUniversal(tc.VAR_SLOPE, vehID)

    def getLine(self, vehID):
        """getLine(string) -> string

        Returns the line information of this vehicle.
        """
        return self._getUniversal(tc.VAR_LINE, vehID)

    def getVia(self, vehID):
        """getVia(string) -> tuple(string)

        Returns the ids of via edges for this vehicle
        """
        return self._getUniversal(tc.VAR_VIA, vehID)

    def getLastActionTime(self, vehID):
        """getLastActionTime(string) -> double

        Returns the time in s of last action point for this vehicle.
        """
        return self._getUniversal(tc.VAR_LASTACTIONTIME, vehID)

    def getBestLanes(self, vehID):
        """getBestLanes(string) -> tuple(data)
        where data is a tuple of (laneID, length, occupation, offset, allowsContinuation, tuple(nextLanes))

        For each lane of the current edge a data tuple is returned where the
        entries have the following meaning:
        - laneID: the id of that lane on the current edge
        - the length that can be driven without lane change (measured from the start of that lane)
        - the occupation on the future lanes (brutto vehicle lengths)
        - the offset of that lane from the lane that would be strategically
          preferred (this is the lane that requires the least future lane
          changes or a lane that needs to be used for stopping)
        - whether that lane allows continuing the route (for at least one more edge)
        - the sequence of lanes that would be driven starting at laneID if no
          lane change were to take place
        """
        return self._getUniversal(tc.VAR_BEST_LANES, vehID)

    def getLeader(self, vehID, dist=100.):
        """getLeader(string, double) -> (string, double)

        Return the leading vehicle id together with the distance. The distance
        is measured from the front + minGap to the back of the leader, so it does not include the
        minGap of the vehicle.
        The dist parameter defines the minimum lookahead, 0 calculates a lookahead from the brake gap.
        Note that the returned leader may be further away than the given dist and that the vehicle
        will only look on its current best lanes and not look beyond the end of its final route edge.

        In the case where no leader is found, the function returns 'None'.
        This special case is deprecated. The future behavior is to return the
        pair ("", -1) when no leader is found.
        The function 'traci.setLegacyGetLeader(bool) can be used to switch
        between both behaviors.
        """
        return self._getUniversal(tc.VAR_LEADER, vehID, "d", dist)

    def getFollower(self, vehID, dist=0.):
        """getFollower(string, double) -> (string, double)

        Return the following vehicle id together with the distance. The distance
        is measured from the front + minGap of the follower to the back of vehID, so it does not include the
        minGap of the follower.
        The dist parameter defines the minimum lookback, 0 calculates the
        lookback distance from the braking distance at 4.5m/s^2 at 2*roadSpeedLimit.
        Due to junctions and lane merges, there may be multiple followers.
        In this case, the "critical" follower is returned. This is the follower
        where the value of (getSecureGap - gap) is maximal.
        Note that the returned follower may be further away than the given dist.
        """
        return self._getUniversal(tc.VAR_FOLLOWER, vehID, "d", dist)

    def getRightFollowers(self, vehID, blockingOnly=False):
        """ getRightFollowers(string, bool) -> tuple(tuple(string, double))
        Convenience method, see getNeighbors()
        """
        if blockingOnly:
            mode = 5
        else:
            mode = 1
        return self.getNeighbors(vehID, mode)

    def getRightLeaders(self, vehID, blockingOnly=False):
        """ getRightLeaders(string, bool) -> tuple(tuple(string, double))
        Convenience method, see getNeighbors()
        """
        if blockingOnly:
            mode = 7
        else:
            mode = 3
        return self.getNeighbors(vehID, mode)

    def getLeftFollowers(self, vehID, blockingOnly=False):
        """ getLeftFollowers(string, bool) -> tuple(tuple(string, double))
        Convenience method, see getNeighbors()
        """
        if blockingOnly:
            mode = 4
        else:
            mode = 0
        return self.getNeighbors(vehID, mode)

    def getLeftLeaders(self, vehID, blockingOnly=False):
        """ getLeftLeaders(string, bool) -> tuple(tuple(string, double))
        Convenience method, see getNeighbors()
        """
        if blockingOnly:
            mode = 6
        else:
            mode = 2
        return self.getNeighbors(vehID, mode)

    def getNeighbors(self, vehID, mode):
        """ getNeighbors(string, byte) -> tuple(tuple(string, double))

        The parameter mode is a bitset (UBYTE), specifying the following:
        bit 1: query lateral direction (left:0, right:1)
        bit 2: query longitudinal direction (followers:0, leaders:1)
        bit 3: blocking (return all:0, return only blockers:1)

        The returned tuple contains pairs (ID, dist) for all lane change relevant neighboring leaders, resp. followers,
        along with their longitudinal distance to the ego vehicle (egoFront - egoMinGap to leaderBack, resp.
        followerFront - followerMinGap to egoBack. The value can be negative for overlapping neighs).
        For the non-sublane case, the lists will contain at most one entry.

        Note: The exact set of blockers in case blocking==1 is not determined for the sublane model,
        but either all neighboring vehicles are returned (in case LCA_BLOCKED) or
        none is returned (in case !LCA_BLOCKED).
        """
        return self._getUniversal(tc.VAR_NEIGHBORS, vehID, "B", mode)

    def getFollowSpeed(self, vehID, speed, gap, leaderSpeed, leaderMaxDecel, leaderID=""):
        """getFollowSpeed(string, double, double, double, double, string) -> double
        Return the follow speed computed by the carFollowModel of vehID
        """
        return self._getUniversal(tc.VAR_FOLLOW_SPEED, vehID, "tdddds", 5,
                                  speed, gap, leaderSpeed, leaderMaxDecel, leaderID)

    def getSecureGap(self, vehID, speed, leaderSpeed, leaderMaxDecel, leaderID=""):
        """getSecureGap(string, double, double, double, string) -> double
        Return the secure gap computed by the carFollowModel of vehID
        """
        return self._getUniversal(tc.VAR_SECURE_GAP, vehID, "tddds", 4,
                                  speed, leaderSpeed, leaderMaxDecel, leaderID)

    def getStopSpeed(self, vehID, speed, gap):
        """getStopSpeed(string, double, double) -> double
        Return the speed for stopping at gap computed by the carFollowModel of vehID
        """
        return self._getUniversal(tc.VAR_STOP_SPEED, vehID, "tdd", 2, speed, gap)

    def getStopDelay(self, vehID):
        """getStopDelay(string) -> double
        Returns the expected depart delay at the next stop (if that stop defines the
        until-attribute) in seconds. Returns -1 if the next stop is not applicable
        """
        return self._getUniversal(tc.VAR_STOP_DELAY, vehID)

    def getStopArrivalDelay(self, vehID):
        """getStopArrivalDelay(string) -> double
        Returns the expected arrival delay at the next stop (if that stop defines the
        arrival-attribute) in seconds. The returned value may be negative to
        indicate early arrival.  Returns INVALID_DOUBLE if the next stop is not applicable
        """
        return self._getUniversal(tc.VAR_STOP_ARRIVALDELAY, vehID)

    def getTimeLoss(self, vehID):
        """getTimeLoss(string) -> double
        Returns the time loss since departure
        """
        return self._getUniversal(tc.VAR_TIMELOSS, vehID)

    def getNextTLS(self, vehID):
        """getNextTLS(string) -> tuple(tuple(string, int, double, string))

        Return tuple of upcoming traffic lights [(tlsID, tlsIndex, distance, state), ...]
        """
        return self._getUniversal(tc.VAR_NEXT_TLS, vehID)

    @alias_param("dist", "distance")
    def getJunctionFoes(self, vehID, dist=0.):
        """getJunctionFoes(string, double) -> complex

        Return tuple of junction foes [(foeId, egoDist, foeDist, egoExitDist, foeExitDist,
        egoLane, foeLane, egoResponse, foeResponse), ...] within the given distance to the given vehicle.
        """
        return self._getUniversal(tc.VAR_FOES, vehID, "d", dist)

    @deprecated()
    def getNextStops(self, vehID):
        """getNextStops(string) -> tuple(tuple(string, double, string, int, double, double))

        Return tuple of upcoming stops ((lane, endPos, stoppingPlaceID, stopFlags, duration, until), ...)
        where integer stopFlag is defined as:
               1 * stopped +
               2 * parking +
               4 * personTriggered +
               8 * containerTriggered +
              16 * isBusStop +
              32 * isContainerStop +
              64 * chargingStation +
             128 * parkingarea
        with each of these flags defined as 0 or 1.
        """
        return self._getUniversal(tc.VAR_NEXT_STOPS, vehID)

    def getNextLinks(self, vehID):
        """getNextLinks(string) -> tuple(tuple(string, string, bool, bool, bool, string, string, double))

        Return tuple of upcoming links along the route ((lane, via, priority, opened, foe,
         state, direction, length), ...)
        """
        return self._getUniversal(tc.VAR_NEXT_LINKS, vehID)

    def getStops(self, vehID, limit=0):
        """getStops(string, int) -> tuple(StopData)

        Return a tuple of StopData object. The flags are the same as for setStop and
        replaceStop (and different from getNextStops(!) for backward compatibility):
               1 * parking +
               2 * personTriggered +
               4 * containerTriggered +
               8 * isBusStop +
              16 * isContainerStop +
              32 * chargingStation +
              64 * parkingarea
        with each of these flags defined as 0 or 1.

        The optional argument limit can be used to limit the returned stops to
        the next INT number (i.e. limit=1 if only the next stop is required).
        Setting a negative limit returns up to 'limit' previous stops (or fewer
        if the vehicle stopped fewer times previously)
        """
        return self._getUniversal(tc.VAR_NEXT_STOPS2, vehID, "i", limit)

    def subscribeLeader(self, vehID, dist=0., begin=0, end=2**31 - 1):
        """subscribeLeader(string, double, double, double) -> None

        Subscribe for the leading vehicle id together with the distance.
        The dist parameter defines the maximum lookahead, 0 calculates a lookahead from the brake gap.
        """
        self.subscribe(vehID, (tc.VAR_LEADER,), begin, end, {tc.VAR_LEADER: ("d", dist)})

    def getDrivingDistance(self, vehID, edgeID, pos, laneIndex=0):
        """getDrivingDistance(string, string, double, integer) -> double

        For an edge along the remaining route of vehID, return the distance from the current vehicle position
        to the given edge and position along the vehicles route.
        Otherwise, return INVALID_DOUBLE_VALUE
        """
        return self._getUniversal(tc.DISTANCE_REQUEST, vehID, "tru", 2,
                                  (edgeID, pos, laneIndex), tc.REQUEST_DRIVINGDIST)

    def getDrivingDistance2D(self, vehID, x, y):
        """getDrivingDistance2D(string, double, double) -> integer

        Return the distance to the given network position along the vehicles route.
        """
        return self._getUniversal(tc.DISTANCE_REQUEST, vehID, "tou", 2, (x, y), tc.REQUEST_DRIVINGDIST)

    def getDistance(self, vehID):
        """getDistance(string) -> double

        Returns the distance to the starting point like an odometer.
        """
        return self._getUniversal(tc.VAR_DISTANCE, vehID)

    def getReferenceDistance(self, vehID):
        """getReferenceDistance(string) -> double
        Returns the distance along the linear reference system in which the current edge takes part (i.e. kilometrage/mile markers)
        """
        return self._getUniversal(tc.VAR_REFERENCE_DISTANCE, vehID)

    def getStopParameter(self, vehID, nextStopIndex, param, customParam=False):
        """getStopParameter(string, int, string) -> string
        Gets the value of the given parameter for the stop at the given index
        Negative indices permit access to past stops.
        Supported params correspond to all legal stop xml-attributes
        If customParam is set to True, the user defined stop parameter with the
        specified param name will be returned instead (or "" if undefined)
        """
        return self._getUniversal(tc.VAR_STOP_PARAMETER, vehID, "tisb", 3, nextStopIndex, param, customParam)

    def getStopState(self, vehID):
        """getStopState(string) -> integer

        Returns information in regard to stopping:
        The returned integer is defined as 1 * stopped + 2 * parking
        + 4 * personTriggered + 8 * containerTriggered + 16 * isBusStop
        + 32 * isContainerStop
        with each of these flags defined as 0 or 1
        """
        return self._getUniversal(tc.VAR_STOPSTATE, vehID)

    def isStopped(self, vehID):
        """isStopped(string) -> bool
        Return whether the vehicle is stopped
        """
        return (self.getStopState(vehID) & 1) == 1

    def isStoppedParking(self, vehID):
        """isStoppedParking(string) -> bool
        Return whether the vehicle is parking (implies stopped)
        """
        return (self.getStopState(vehID) & 2) == 2

    def isStoppedTriggered(self, vehID):
        """isStoppedTriggered(string) -> bool
        Return whether the vehicle is stopped and waiting for a person or container
        """
        return (self.getStopState(vehID) & 12) > 0

    def isAtBusStop(self, vehID):
        """isAtBusStop(string) -> bool
        Return whether the vehicle is stopped at a bus stop
        """
        return (self.getStopState(vehID) & 16) == 16

    def isAtContainerStop(self, vehID):
        """isAtContainerStop(string) -> bool
        Return whether the vehicle is stopped at a container stop
        """
        return (self.getStopState(vehID) & 32) == 32

    def getLaneChangeState(self, vehID, direction):
        """getLaneChangeState(string, int) -> (int, int)
        Return the lane change state for the vehicle. The first value returns
        the state as computed by the lane change model and the second value
        returns the state after incorporation TraCI requests.
        See getLaneChangeStatePretty for an interpretation of the integer/bitset
        results
        """
        return self._getUniversal(tc.CMD_CHANGELANE, vehID, "i", direction)

    def getLaneChangeStatePretty(self, vehID, direction):
        """getLaneChangeStatePretty(string, int) -> ([string, ...], [string, ...])
        Return the lane change state for the vehicle as two lists of string
        constants. The first tuple returns the state as computed by the lane change
        model and the second tuple returns the state after incorporation TraCI requests.
        """
        constants = {
            0: 'stay',
            1: 'left',
            2: 'right',
            3: 'strategic',
            4: 'cooperative',
            5: 'speedGain',
            6: 'keepRight',
            7: 'TraCI',
            8: 'urgent',
            9: 'blocked by left leader',
            10: 'blocked by left follower',
            11: 'blocked by right leader',
            12: 'blocked by right follower',
            13: 'overlapping',
            14: 'insufficient space',
            15: 'sublane',
        }

        def prettifyBitstring(intval):
            return tuple([v for k, v in constants.items() if (intval & 2**k)])

        state, stateTraCI = self.getLaneChangeState(vehID, direction)
        return prettifyBitstring(state), prettifyBitstring(stateTraCI)

    def couldChangeLane(self, vehID, direction, state=None):
        """couldChangeLane(string, int) -> bool
        Return whether the vehicle could change lanes in the specified direction.
        This reflects the state after the last try to change lanes.
        If you want to execute changeLane as a result of the evaluation of this function
        it is not guaranteed to work because vehicle movements occur first.
        """
        if state is None:
            state, stateTraCI = self.getLaneChangeState(vehID, direction)
            if self.wantsAndCouldChangeLane(vehID, direction, stateTraCI):
                # vehicle changed in the last step. state is no longer applicable
                return False
        return state != tc.LCA_UNKNOWN and (state & tc.LCA_BLOCKED == 0)

    def wantsAndCouldChangeLane(self, vehID, direction, state=None):
        """wantsAndCouldChangeLane(string, int) -> bool
        Return whether the vehicle wants to and could change lanes in the specified direction
        This reflects the state after the last try to change lanes.
        If you want to execute changeLane as a result of the evaluation of this function
        it is not guaranteed to work because vehicle movements occur first.
        """
        if state is None:
            state, stateTraCI = self.getLaneChangeState(vehID, direction)
            if self.wantsAndCouldChangeLane(vehID, direction, stateTraCI):
                # vehicle changed in the last step. state is no longer applicable
                return False
        if state & tc.LCA_BLOCKED == 0:
            if direction == -1:
                return state & tc.LCA_RIGHT != 0
            if direction == 1:
                return state & tc.LCA_LEFT != 0
        return False

    def getRoutingMode(self, vehID):
        """getRoutingMode(string)
        returns the current routing mode:
        tc.ROUTING_MODE_DEFAULT    : use weight storages and fall-back to edge speeds (default)
        tc.ROUTING_MODE_AGGREGATED : use global smoothed travel times from device.rerouting
        """
        return self._getUniversal(tc.VAR_ROUTING_MODE, vehID)

    @alias_param("taxiState", "flag")
    def getTaxiFleet(self, taxiState=0):
        """getTaxiFleet(int) -> tuple(string)
        Return the tuple of all taxis with the given taxiState:
        0 : empty
        1 : pickup
        2 : occupied
        """
        return self._getUniversal(tc.VAR_TAXI_FLEET, "", "i", taxiState)

    def getLoadedIDList(self):
        """getLoadedIDList() -> tuple(string)
        returns all loaded vehicles that have not yet left the simulation
        """
        return self._getUniversal(tc.VAR_LOADED_LIST, "")

    def getTeleportingIDList(self):
        """getTeleportingIDList() -> tuple(string)
        returns all teleporting or jumping vehicles
        """
        return self._getUniversal(tc.VAR_TELEPORTING_LIST, "")

    def rerouteParkingArea(self, vehID, parkingAreaID):
        """rerouteParkingArea(string, string)

        Changes the next parking area in parkingAreaID, updates the vehicle route,
        and preserve consistency in case of passengers/containers on board.
        """
        self._setCmd(tc.CMD_REROUTE_TO_PARKING, vehID, "ts", 1, parkingAreaID)

    def setStop(self, vehID, edgeID, pos=1., laneIndex=0, duration=tc.INVALID_DOUBLE_VALUE,
                flags=tc.STOP_DEFAULT, startPos=tc.INVALID_DOUBLE_VALUE, until=tc.INVALID_DOUBLE_VALUE):
        """setStop(string, string, double, integer, double, integer, double, double) -> None

        Adds or modifies a stop with the given parameters. The duration and the until attribute are
        in seconds.
        """
        if type(duration) is int and duration >= 1000 and duration % 1000 == 0:
            warnings.warn("API change now handles duration as floating point seconds", stacklevel=2)
        self._setCmd(tc.CMD_STOP, vehID, "tsdbdbdd", 7, edgeID, pos, laneIndex, duration, flags, startPos, until)

    def setBusStop(self, vehID, stopID, duration=tc.INVALID_DOUBLE_VALUE,
                   until=tc.INVALID_DOUBLE_VALUE, flags=tc.STOP_DEFAULT):
        """setBusStop(string, string, double, double, integer) -> None

        Adds or modifies a bus stop with the given parameters. The duration and the until attribute are
        in seconds.
        """
        self.setStop(vehID, stopID, duration=duration, until=until, flags=flags | tc.STOP_BUS_STOP)

    def setContainerStop(self, vehID, stopID, duration=tc.INVALID_DOUBLE_VALUE,
                         until=tc.INVALID_DOUBLE_VALUE, flags=tc.STOP_DEFAULT):
        """setContainerStop(string, string, double, double, integer) -> None

        Adds or modifies a container stop with the given parameters. The duration and the until attribute are
        in seconds.
        """
        self.setStop(vehID, stopID, duration=duration, until=until, flags=flags | tc.STOP_CONTAINER_STOP)

    def setChargingStationStop(self, vehID, stopID, duration=tc.INVALID_DOUBLE_VALUE,
                               until=tc.INVALID_DOUBLE_VALUE, flags=tc.STOP_DEFAULT):
        """setChargingStationStop(string, string, double, double, integer) -> None

        Adds or modifies a stop at a chargingStation with the given parameters. The duration and the until attribute are
        in seconds.
        """
        self.setStop(vehID, stopID, duration=duration, until=until, flags=flags | tc.STOP_CHARGING_STATION)

    def setParkingAreaStop(self, vehID, stopID, duration=tc.INVALID_DOUBLE_VALUE,
                           until=tc.INVALID_DOUBLE_VALUE, flags=tc.STOP_PARKING):
        """setParkingAreaStop(string, string, double, double, integer) -> None

        Adds or modifies a stop at a parkingArea with the given parameters. The duration and the until attribute are
        in seconds.
        """
        self.setStop(vehID, stopID, duration=duration, until=until, flags=flags | tc.STOP_PARKING_AREA)

    def replaceStop(self, vehID, nextStopIndex, edgeID, pos=1., laneIndex=0, duration=tc.INVALID_DOUBLE_VALUE,
                    flags=tc.STOP_DEFAULT, startPos=tc.INVALID_DOUBLE_VALUE,
                    until=tc.INVALID_DOUBLE_VALUE, teleport=0):
        """replaceStop(string, int, string, double, integer, double, integer, double, double) -> None

        Replaces stop at the given index (within the list of all stops) with a new stop.
        Automatically modifies the route if the replacement stop is at another location.
        For edgeID a stopping place id may be given if the flag marks this
        stop as stopping on busStop, parkingArea, containerStop etc.
        If edgeID is "", the stop at the given index will be removed without
        replacement and the route will not be modified (unless setting
        teleport=2 which will trigger rerouting between the prior and next stop)
        If teleport is set to 1, the route to the replacement stop will be
        disconnected (forcing a teleport).
        If stopIndex is 0 the gap will be between the current
        edge and the new stop. Otherwise the gap will be between the stop edge for
        nextStopIndex - 1 and the new stop.
        """
        self._setCmd(tc.CMD_REPLACE_STOP, vehID, "tsdbdiddib", 9, edgeID, pos,
                     laneIndex, duration, flags, startPos, until, nextStopIndex, teleport)

    def insertStop(self, vehID, nextStopIndex, edgeID, pos=1., laneIndex=0, duration=tc.INVALID_DOUBLE_VALUE,
                   flags=tc.STOP_DEFAULT, startPos=tc.INVALID_DOUBLE_VALUE,
                   until=tc.INVALID_DOUBLE_VALUE, teleport=0):
        """insertStop(string, int, string, double, integer, double, integer, double, double) -> None

        Insert stop at the given index (within the list of all existing stops).
        Automatically modifies the route if the new stop is not along the route between the preceeding
        and succeeding stops (or start / end).
        For edgeID a stopping place id may be given if the flag marks this
        stop as stopping on busStop, parkingArea, containerStop etc.
        If teleport is set to 1, the route to the new stop will be
        disconnected (forcing a teleport).
        If stopIndex is 0 the gap will be between the current
        edge and the new stop. Otherwise the gap will be between the stop edge for
        nextStopIndex - 1 and the new stop.
        """
        self._setCmd(tc.CMD_INSERT_STOP, vehID, "tsdbdiddib", 9, edgeID, pos,
                     laneIndex, duration, flags, startPos, until, nextStopIndex, teleport)

    def setStopParameter(self, vehID, nextStopIndex, param, value, customParam=False):
        """setStopParameter(string, int, string, string) -> None
        Sets the value of the given parameter for the (upcoming) stop at the
        given index (within the list of all stops).
        Supported params correspond to (almost) all legal stop xml-attributes
        and their value semantics
        If customParam is set to True, the user defined stop parameter with the
        specified param name will be set instead
        """
        self._setCmd(tc.VAR_STOP_PARAMETER, vehID, "tissb", 4, nextStopIndex, param, value, customParam)

    def resume(self, vehID):
        """resume(string) -> None

        Resumes the vehicle from the current stop (throws an error if the vehicle is not stopped).
        """
        self._setCmd(tc.CMD_RESUME, vehID, "t", 0)

    def changeLane(self, vehID, laneIndex, duration):
        """changeLane(string, int, double) -> None
        Forces a lane change to the lane with the given index; The lane change
        will be attempted for the given duration (in s) and if it succeeds,
        the vehicle will stay on that lane for the remaining duration.
        """
        if type(duration) is int and duration >= 1000:
            warnings.warn("API change now handles duration as floating point seconds", stacklevel=2)
        self._setCmd(tc.CMD_CHANGELANE, vehID, "tbd", 2, laneIndex, duration)

    def changeLaneRelative(self, vehID, indexOffset, duration):
        """changeLaneRelative(string, int, double) -> None

        Forces a relative lane change; if successful,
        the lane will be chosen for the given amount of time (in s).
        The indexOffset specifies the target lane relative to the vehicles current lane
        """
        if type(duration) is int and duration >= 1000:
            warnings.warn("API change now handles duration as floating point seconds", stacklevel=2)
        self._setCmd(tc.CMD_CHANGELANE, vehID, "tbdb", 3, indexOffset, duration, 1)

    def changeSublane(self, vehID, latDist):
        """changeSublane(string, double) -> None
        Forces a lateral change by the given amount (negative values indicate changing to the right, positive
        to the left). This will override any other lane change motivations but conform to
        safety-constraints as configured by laneChangeMode.
        """
        self._setCmd(tc.CMD_CHANGESUBLANE, vehID, "d", latDist)

    def slowDown(self, vehID, speed, duration):
        """slowDown(string, double, double) -> None

        Changes the speed smoothly to the given value over the given amount
        of time in seconds (can also be used to increase speed).
        """
        if type(duration) is int and duration >= 1000:
            warnings.warn("API change now handles duration as floating point seconds", stacklevel=2)
        self._setCmd(tc.CMD_SLOWDOWN, vehID, "tdd", 2, speed, duration)

    def openGap(self, vehID, newTimeHeadway, newSpaceHeadway, duration, changeRate, maxDecel=-1, referenceVehID=None):
        """openGap(string, double, double, double, double, double, string) -> None

        Changes the vehicle's desired time headway (cf-parameter tau) smoothly to the given new value
        using the given change rate. Similarly, the given space headway is applied gradually
        to achieve a minimal spatial gap.
        The vehicle is commanded to keep the increased headway for
        the given duration once its target value is attained. The maximal value for the
        deceleration can be given to prevent harsh braking due to the change of tau. If maxDecel=-1,
        the limit determined by the CF model is used.
        A vehicle ID for a reference vehicle can optionally be given, otherwise, the gap is created with
        respect to the current leader on the ego vehicle's current lane.
        Note that this does only affect the following behavior regarding the current leader and does
        not influence the gap acceptance during lane change, etc.
        """
        if type(duration) is int and duration >= 1000:
            warnings.warn("API change now handles duration as floating point seconds", stacklevel=2)
        if referenceVehID is None:
            self._setCmd(tc.CMD_OPENGAP, vehID, "tddddd", 5,
                         newTimeHeadway, newSpaceHeadway, duration, changeRate, maxDecel)
        else:
            self._setCmd(tc.CMD_OPENGAP, vehID, "tddddds", 6,
                         newTimeHeadway, newSpaceHeadway, duration, changeRate, maxDecel, referenceVehID)

    def deactivateGapControl(self, vehID):
        """deactivateGapControl(string) -> None

        Deactivate the vehicle's gap control
        """
        self.openGap(vehID, -1, -1, -1, -1)

    def requestToC(self, vehID, leadTime):
        """ requestToC(string, double) -> None

        Interface for triggering a transition of control for a vehicle equipped with a ToC device.
        """
        self.setParameter(vehID, "device.toc.requestToC", str(leadTime))

    def changeTarget(self, vehID, edgeID):
        """changeTarget(string, string) -> None

        The vehicle's destination edge is set to the given edge id. The route is rebuilt.
        """
        self._setCmd(tc.CMD_CHANGETARGET, vehID, "s", edgeID)

    def setType(self, vehID, typeID):
        """setType(string, string) -> None

        Sets the id of the type for the named vehicle.
        """
        self._setCmd(tc.VAR_TYPE, vehID, "s", typeID)

    def setRouteID(self, vehID, routeID):
        """setRouteID(string, string) -> None

        Changes the vehicles route to the route with the given id.
        """
        self._setCmd(tc.VAR_ROUTE_ID, vehID, "s", routeID)

    def setRoute(self, vehID, edgeList):
        """
        setRoute(string, list) ->  None

        changes the vehicle route to given edges list.
        The first edge in the list has to be the one that the vehicle is at at the moment.

        example usage:
        setRoute('1', ['1', '2', '4', '6', '7'])

        this changes route for vehicle id 1 to edges 1-2-4-6-7
        """
        if isinstance(edgeList, str):
            edgeList = [edgeList]
        self._setCmd(tc.VAR_ROUTE, vehID, "l", edgeList)

    def setLateralLanePosition(self, vehID, posLat):
        """setLateralLanePosition(string, double) -> None

        Sets the lateral vehicle position relative to the center line of the
        lane in m (negative values are to the right in right-hand networks).
        The vehicle may adapt this position in the same step unless this is
        disabled via setLaneChangeMode.
        """
        self._setCmd(tc.VAR_LANEPOSITION_LAT, vehID, "d", posLat)

    def updateBestLanes(self, vehID):
        """ updateBestLanes(string) -> None
        Triggers an update of the vehicle's bestLanes (structure determining the lane preferences used by LC models)
        It may be called after modifying the vClass for instance.
        """
        self._setCmd(tc.VAR_UPDATE_BESTLANES, vehID)

    def setAdaptedTraveltime(self, vehID, edgeID, time=None, begTime=None, endTime=None):
        """setAdaptedTraveltime(string, string, double, double, double) -> None
        Inserts the information about the travel time of edge "edgeID" valid
        from begin time to end time into the vehicle's internal edge weights
        container.
        If the time is not specified, any previously set values for that edge
        are removed.
        If begTime or endTime are not specified the value is set for the whole
        simulation duration.
        """
        if not isinstance(edgeID, str) and isinstance(begTime, str):
            # legacy handling
            warnings.warn(
                "Parameter order has changed for setAdaptedTraveltime(). Attempting legacy ordering. " +
                "Please update your code.", stacklevel=2)
            return self.setAdaptedTraveltime(vehID, begTime, endTime, edgeID, time)
        if time is None:
            # reset
            self._setCmd(tc.VAR_EDGE_TRAVELTIME, vehID, "ts", 1, edgeID)
        elif begTime is None:
            # set value for the whole simulation
            self._setCmd(tc.VAR_EDGE_TRAVELTIME, vehID, "tsd", 2, edgeID, time)
        else:
            self._setCmd(tc.VAR_EDGE_TRAVELTIME, vehID, "tddsd", 4, begTime, endTime, edgeID, time)

    def setEffort(self, vehID, edgeID, effort=None, begTime=None, endTime=None):
        """setEffort(string, string, double, double, double) -> None
        Inserts the information about the effort of edge "edgeID" valid from
        begin time to end time into the vehicle's internal edge weights
        container.
        If the time is not specified, any previously set values for that edge
        are removed.
        If begTime or endTime are not specified the value is set for the whole
        simulation duration.
        """
        if not isinstance(edgeID, str) and isinstance(begTime, str):
            # legacy handling
            warnings.warn(
                "Parameter order has changed for setEffort(). Attempting legacy ordering. Please update your code.",
                stacklevel=2)
            return self.setEffort(vehID, begTime, endTime, edgeID, effort)
        if effort is None:
            # reset
            self._setCmd(tc.VAR_EDGE_EFFORT, vehID, "ts", 1, edgeID)
        elif begTime is None:
            # set value for the whole simulation
            self._setCmd(tc.VAR_EDGE_EFFORT, vehID, "tsd", 2, edgeID, effort)
        else:
            self._setCmd(tc.VAR_EDGE_EFFORT, vehID, "tddsd", 4, begTime, endTime, edgeID, effort)

    LAST_TRAVEL_TIME_UPDATE = -1

    def setRoutingMode(self, vehID, routingMode):
        """setRoutingMode(string, int) -> None
        Sets the current routing mode:
        tc.ROUTING_MODE_DEFAULT    : use weight storages and fall-back to edge speeds (default)
        tc.ROUTING_MODE_AGGREGATED : use global smoothed travel times from device.rerouting
        tc.ROUTING_MODE_AGGREGATED_CUSTOM : use weight storages and fall-back to smoothed travel times
        """
        self._setCmd(tc.VAR_ROUTING_MODE, vehID, "i", routingMode)

    def rerouteTraveltime(self, vehID, currentTravelTimes=True):
        """rerouteTraveltime(string, bool) -> None
        Reroutes a vehicle.
        If currentTravelTimes is True (default) and the routing mode is still ROUTING_MODE_DEFAULT
        then the ROUTING_MODE_AGGREGATED_CUSTOM gets activated temporarily
        and used for rerouting. The various functions and options for
        customizing travel times are described at https://sumo.dlr.de/wiki/Simulation/Routing

        When rerouteTraveltime has been called once with an aggregated routing mode,
        edge weight storage and update gets activated which might slow down the simulation.
        """
        if currentTravelTimes:
            routingMode = self.getRoutingMode(vehID)
            if routingMode == tc.ROUTING_MODE_DEFAULT:
                self.setRoutingMode(vehID, tc.ROUTING_MODE_AGGREGATED_CUSTOM)
        self._setCmd(tc.CMD_REROUTE_TRAVELTIME, vehID, "t", 0)
        if currentTravelTimes and routingMode == tc.ROUTING_MODE_DEFAULT:
            self.setRoutingMode(vehID, routingMode)

    def rerouteEffort(self, vehID):
        """rerouteEffort(string) -> None
        Reroutes a vehicle according to the effort values.
        """
        self._setCmd(tc.CMD_REROUTE_EFFORT, vehID, "t", 0)

    def setSignals(self, vehID, signals):
        """setSignals(string, integer) -> None

        Sets an integer encoding the state of the vehicle's signals.
        """
        self._setCmd(tc.VAR_SIGNALS, vehID, "i", signals)

    def moveTo(self, vehID, laneID, pos, reason=tc.MOVE_AUTOMATIC):
        """moveTo(string, string, double, integer) -> None

        Move a vehicle to a new position along its current route.
        """
        self._setCmd(tc.VAR_MOVE_TO, vehID, "tsdi", 3, laneID, pos, reason)

    def setSpeed(self, vehID, speed):
        """setSpeed(string, double) -> None

        Sets the speed in m/s for the named vehicle within the last step.
        Calling with speed=-1 hands the vehicle control back to SUMO.
        """
        self._setCmd(tc.VAR_SPEED, vehID, "d", speed)

    def setAcceleration(self, vehID, acceleration, duration):
        """setAcceleration(string, double, double) -> None

        Sets the acceleration in m/s^2 for the named vehicle and the given duration.
        """
        self._setCmd(tc.VAR_ACCELERATION, vehID, "tdd", 2, acceleration, duration)

    def setPreviousSpeed(self, vehID, speed, acceleration=tc.INVALID_DOUBLE_VALUE):
        """setPreviousSpeed(string, double, double) -> None

        Sets the previous speed in m/s for the named vehicle wich will be used for
        calculations in the current step. Optionally, the acceleration for the
        previous step (in m/s^2) can be set as well.
        """
        self._setCmd(tc.VAR_PREV_SPEED, vehID, "tdd", 2, speed, acceleration)

    def setLine(self, vehID, line):
        """setLine(string, string) -> None

        Sets the line information for this vehicle.
        """
        self._setCmd(tc.VAR_LINE, vehID, "s", line)

    def setVia(self, vehID, edgeList):
        """
        setVia(string, list) ->  None

        changes the via edges to the given edges list (to be used during
        subsequent rerouting calls).

        Note: a single edgeId as argument is allowed as shorthand for a list of length 1
        """
        if isinstance(edgeList, str):
            edgeList = [edgeList]
        self._setCmd(tc.VAR_VIA, vehID, "l", edgeList)

    def highlight(self, vehID, color=(255, 0, 0, 255), size=-1, alphaMax=-1, duration=-1, type=0):
        """ highlight(string, color, float, ubyte, float, ubyte) -> None
            Adds a circle of the given color tracking the vehicle.
            If a positive size [in m] is given the size of the highlight is chosen accordingly,
            otherwise the length of the vehicle is used as reference.
            If alphaMax and duration are positive, the circle fades in and out within the given duration,
            otherwise it permanently follows the vehicle.
        """
        if type > 255:
            raise TraCIException("vehicle.highlight(): maximal value for type is 255")
        if alphaMax > 255:
            raise TraCIException("vehicle.highlight(): maximal value for alphaMax is 255")
        if alphaMax <= 0 and duration > 0:
            raise TraCIException("vehicle.highlight(): duration>0 requires alphaMax>0")
        if alphaMax > 0 and duration <= 0:
            raise TraCIException("vehicle.highlight(): alphaMax>0 requires duration>0")

        if alphaMax > 0:
            self._setCmd(tc.VAR_HIGHLIGHT, vehID, "tcdBdB", 5, color, size, alphaMax, duration, type)
        else:
            self._setCmd(tc.VAR_HIGHLIGHT, vehID, "tcd", 2, color, size)

    @alias_param("laneChangeMode", "lcm")
    def setLaneChangeMode(self, vehID, laneChangeMode):
        """setLaneChangeMode(string, integer) -> None

        Sets the vehicle's lane change mode as a bitset.
        """
        self._setCmd(tc.VAR_LANECHANGE_MODE, vehID, "i", laneChangeMode)

    @alias_param("speedMode", "sm")
    def setSpeedMode(self, vehID, speedMode):
        """setSpeedMode(string, integer) -> None

        Sets the vehicle's speed mode as a bitset.
        """
        self._setCmd(tc.VAR_SPEEDSETMODE, vehID, "i", speedMode)

    def addLegacy(self, vehID, routeID, depart=tc.DEPARTFLAG_NOW, pos=0, speed=0,
                  lane=tc.DEPARTFLAG_LANE_FIRST_ALLOWED, typeID="DEFAULT_VEHTYPE"):
        """
        Add a new vehicle (old style)
        """
        if depart == tc.DEPARTFLAG_NOW:
            depart = "now"
        elif depart == tc.DEPARTFLAG_TRIGGERED:
            depart = "triggered"
        else:
            depart = str(depart)
        if pos < 0:
            print("Invalid departure position.")
            return
        if lane == tc.DEPARTFLAG_LANE_FIRST_ALLOWED:
            lane = "first"
        elif lane == tc.DEPARTFLAG_LANE_FREE:
            lane = "free"
        else:
            lane = str(lane)
        self.addFull(vehID, routeID, typeID, depart, lane, str(pos), str(speed))

    def add(self, vehID, routeID, typeID="DEFAULT_VEHTYPE", depart="now",
            departLane="first", departPos="base", departSpeed="0",
            arrivalLane="current", arrivalPos="max", arrivalSpeed="current",
            fromTaz="", toTaz="", line="", personCapacity=0, personNumber=0):
        """
        Add a new vehicle (new style with all possible parameters)
        If routeID is "", the vehicle will be inserted on a random network edge
        if route consists of two disconnected edges, the vehicle will be treated
        like a <trip> and use the fastest route between the two edges.
        """
        if depart is None:
            # legacy compatibility
            depart = str(self._connection.simulation.getTime())
        self._setCmd(tc.ADD_FULL, vehID, "t" + (12 * "s") + "ii", 14,
                     routeID, typeID, depart, departLane, departPos, departSpeed,
                     arrivalLane, arrivalPos, arrivalSpeed, fromTaz, toTaz, line, personCapacity, personNumber)

    addFull = add

    def dispatchTaxi(self, vehID, reservations):
        """dispatchTaxi(string, list(string)) -> None
        dispatches the taxi with the given id to service the given reservations.
        If only a single reservation is given, this implies pickup and drop-off
        If multiple reservations are given, each reservation id must occur twice
        (once for pickup and once for drop-off) and the list encodes ride
        sharing of passengers (in pickup and drop-off order)
        """
        if isinstance(reservations, str):
            reservations = [reservations]
        self._setCmd(tc.CMD_TAXI_DISPATCH, vehID, "l", reservations)

    def remove(self, vehID, reason=tc.REMOVE_VAPORIZED):
        '''Remove vehicle with the given ID for the give reason.
           Reasons are defined in module constants and start with REMOVE_'''
        self._setCmd(tc.REMOVE, vehID, "b", reason)

    @alias_param("laneIndex", "lane")
    def moveToXY(self, vehID, edgeID, laneIndex, x, y, angle=tc.INVALID_DOUBLE_VALUE, keepRoute=1, matchThreshold=100):
        '''Place vehicle at the given x,y coordinates and force its angle to
        the given value (for drawing).
        If the angle is set to INVALID_DOUBLE_VALUE, the vehicle assumes the
        natural angle of the edge on which it is driving.
        If keepRoute is set to 1, the closest position
        within the existing route is taken. If keepRoute is set to 0, the vehicle may move to
        any edge in the network but its route then only consists of that edge.
        If keepRoute is set to 2 the vehicle has all the freedom of keepRoute=0
        but in addition to that may even move outside the road network.
        edgeID and lane are optional placement hints to resolve ambiguities.
        The command fails if no suitable target position is found within the
        distance given by matchThreshold.
        '''
        self._setCmd(tc.MOVE_TO_XY, vehID, "tsidddbd", 7, edgeID, laneIndex, x, y, angle, keepRoute, matchThreshold)

    def addSubscriptionFilterLanes(self, lanes, noOpposite=False, downstreamDist=None, upstreamDist=None):
        """addSubscriptionFilterLanes(list(integer), bool, double, double) -> None

        Adds a lane-filter to the last modified vehicle context subscription (call it just after subscribing).
        lanes is a list of relative lane indices (-1 -> right neighboring lane of the ego, 0 -> ego lane, etc.)
        noOpposite specifies whether vehicles on opposite direction lanes shall be returned
        downstreamDist and upstreamDist specify the range of the search for surrounding vehicles along the road net.
        """
        self._connection._addSubscriptionFilter(tc.FILTER_TYPE_LANES, lanes)
        if noOpposite:
            self.addSubscriptionFilterNoOpposite()
        if downstreamDist is not None:
            self.addSubscriptionFilterDownstreamDistance(downstreamDist)
        if upstreamDist is not None:
            self.addSubscriptionFilterUpstreamDistance(upstreamDist)

    def addSubscriptionFilterNoOpposite(self):
        """addSubscriptionFilterNoOpposite() -> None

        Omits vehicles on other edges than the ego's for the last modified vehicle context subscription
        (call it just after subscribing).
        """
        self._connection._addSubscriptionFilter(tc.FILTER_TYPE_NOOPPOSITE)

    def addSubscriptionFilterDownstreamDistance(self, dist):
        """addSubscriptionFilterDownstreamDist(float) -> None

        Sets the downstream distance along the network for vehicles to be returned by the last modified
        vehicle context subscription (call it just after subscribing).
        """
        self._connection._addSubscriptionFilter(tc.FILTER_TYPE_DOWNSTREAM_DIST, dist)

    def addSubscriptionFilterUpstreamDistance(self, dist):
        """addSubscriptionFilterUpstreamDist(float) -> None

        Sets the upstream distance along the network for vehicles to be returned by the last modified
        vehicle context subscription (call it just after subscribing).
        """
        self._connection._addSubscriptionFilter(tc.FILTER_TYPE_UPSTREAM_DIST, dist)

    def addSubscriptionFilterCFManeuver(self, downstreamDist=None, upstreamDist=None):
        """addSubscriptionFilterCFManeuver() -> None

        Restricts vehicles returned by the last modified vehicle context subscription to leader and follower of the ego.
        downstreamDist and upstreamDist specify the range of the search for leader and follower along the road net.
        """
        self.addSubscriptionFilterLeadFollow([0])
        if downstreamDist is not None:
            self.addSubscriptionFilterDownstreamDistance(downstreamDist)
        if upstreamDist is not None:
            self.addSubscriptionFilterUpstreamDistance(upstreamDist)

    def addSubscriptionFilterLCManeuver(self, direction=None, noOpposite=False, downstreamDist=None, upstreamDist=None):
        """addSubscriptionFilterLCManeuver(int) -> None

        Restricts vehicles returned by the last modified vehicle context subscription to neighbor and ego-lane leader
        and follower of the ego.
        direction - lane change direction (in {-1=right, 1=left})
        noOpposite specifies whether vehicles on opposite direction lanes shall be returned
        downstreamDist and upstreamDist specify the range of the search for leader and follower along the road net.
        Combine with: distance filters; vClass/vType filter.
        """
        if direction is None:
            # Using default: both directions
            lanes = [-1, 0, 1]
        elif not (direction == -1 or direction == 1):
            warnings.warn("Ignoring lane change subscription filter " +
                          "with non-neighboring lane offset direction=%s." % direction)
            return
        else:
            lanes = [0, direction]
        self.addSubscriptionFilterLeadFollow(lanes)
        if noOpposite:
            self.addSubscriptionFilterNoOpposite()
        if downstreamDist is not None:
            self.addSubscriptionFilterDownstreamDistance(downstreamDist)
        if upstreamDist is not None:
            self.addSubscriptionFilterUpstreamDistance(upstreamDist)

    def addSubscriptionFilterLeadFollow(self, lanes):
        """addSubscriptionFilterLCManeuver(lanes) -> None

        Restricts vehicles returned by the last modified vehicle context subscription to neighbor and ego-lane leader
        and follower of the ego.
        Combine with: lanes-filter to restrict to one direction; distance filters; vClass/vType filter.
        """
        self._connection._addSubscriptionFilter(tc.FILTER_TYPE_LEAD_FOLLOW)
        self._connection._addSubscriptionFilter(tc.FILTER_TYPE_LANES, lanes)

    def addSubscriptionFilterTurn(self, downstreamDist=None, foeDistToJunction=None):
        """addSubscriptionFilterTurn(double, double) -> None

        Restricts vehicles returned by the last modified vehicle context subscription to foes on upcoming junctions
        """
        self._connection._addSubscriptionFilter(tc.FILTER_TYPE_TURN, foeDistToJunction)
        if downstreamDist is not None:
            self.addSubscriptionFilterDownstreamDistance(downstreamDist)

    def addSubscriptionFilterVClass(self, vClasses):
        """addSubscriptionFilterVClass(list(String)) -> None

        Restricts vehicles returned by the last modified vehicle context subscription to vehicles of the given classes
        """
        self._connection._addSubscriptionFilter(tc.FILTER_TYPE_VCLASS, vClasses)

    def addSubscriptionFilterVType(self, vTypes):
        """addSubscriptionFilterVType(list(String)) -> None

        Restricts vehicles returned by the last modified vehicle context subscription to vehicles of the given types
        """
        self._connection._addSubscriptionFilter(tc.FILTER_TYPE_VTYPE, vTypes)

    def addSubscriptionFilterFieldOfVision(self, openingAngle):
        """addSubscriptionFilterFieldOfVision(float) -> None

        Restricts vehicles returned by the last modified vehicle context subscription
        to vehicles within field of vision with given opening angle
        """
        self._connection._addSubscriptionFilter(tc.FILTER_TYPE_FIELD_OF_VISION, openingAngle)

    def addSubscriptionFilterLateralDistance(self, lateralDist, downstreamDist=None, upstreamDist=None):
        """addSubscriptionFilterLateralDist(double, double, double) -> None

        Adds a lateral distance filter to the last modified vehicle context subscription
        (call it just after subscribing).
        downstreamDist and upstreamDist specify the longitudinal range of the search
        for surrounding vehicles along the ego vehicle's route.
        """
        self._connection._addSubscriptionFilter(tc.FILTER_TYPE_LATERAL_DIST, lateralDist)
        if downstreamDist is not None:
            self.addSubscriptionFilterDownstreamDistance(downstreamDist)
        if upstreamDist is not None:
            self.addSubscriptionFilterUpstreamDistance(upstreamDist)
