/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2017-2026 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    Vehicle.cpp
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Mirko Barthauer
/// @date    30.05.2012
///
// C++ TraCI client API implementation
/****************************************************************************/
#include <config.h>
#include <sstream>

#define LIBTRACI 1
#include <libsumo/StorageHelper.h>
#include <libsumo/Vehicle.h>
#include "Domain.h"

namespace libtraci {

typedef Domain<libsumo::CMD_GET_VEHICLE_VARIABLE, libsumo::CMD_SET_VEHICLE_VARIABLE> Dom;


// ===========================================================================
// static member definitions
// ===========================================================================
std::vector<std::string>
Vehicle::getIDList() {
    return Dom::getStringVector(libsumo::TRACI_ID_LIST, "");
}


int
Vehicle::getIDCount() {
    return Dom::getInt(libsumo::ID_COUNT, "");
}


LIBTRACI_SUBSCRIPTION_IMPLEMENTATION(Vehicle, VEHICLE)
LIBTRACI_PARAMETER_IMPLEMENTATION(Vehicle, VEHICLE)

double
Vehicle::getSpeed(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_SPEED, vehID);
}

double
Vehicle::getLateralSpeed(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_SPEED_LAT, vehID);
}

double
Vehicle::getAcceleration(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_ACCELERATION, vehID);
}


double
Vehicle::getSpeedWithoutTraCI(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_SPEED_WITHOUT_TRACI, vehID);
}


libsumo::TraCIPosition
Vehicle::getPosition(const std::string& vehID, const bool includeZ) {
    return includeZ ? getPosition3D(vehID) : Dom::getPos(libsumo::VAR_POSITION, vehID);
}


libsumo::TraCIPosition
Vehicle::getPosition3D(const std::string& vehID) {
    return Dom::getPos3D(libsumo::VAR_POSITION3D, vehID);
}


double
Vehicle::getAngle(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_ANGLE, vehID);
}


double
Vehicle::getSlope(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_SLOPE, vehID);
}


std::string
Vehicle::getRoadID(const std::string& vehID) {
    return Dom::getString(libsumo::VAR_ROAD_ID, vehID);
}


double
Vehicle::getDeparture(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_DEPARTURE, vehID);
}


double
Vehicle::getDepartDelay(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_DEPART_DELAY, vehID);
}


std::string
Vehicle::getLaneID(const std::string& vehID) {
    return Dom::getString(libsumo::VAR_LANE_ID, vehID);
}


int
Vehicle::getLaneIndex(const std::string& vehID) {
    return Dom::getInt(libsumo::VAR_LANE_INDEX, vehID);
}


std::string
Vehicle::getSegmentID(const std::string& vehID) {
    return Dom::getString(libsumo::VAR_SEGMENT_ID, vehID);
}


int
Vehicle::getSegmentIndex(const std::string& vehID) {
    return Dom::getInt(libsumo::VAR_SEGMENT_INDEX, vehID);
}


std::string
Vehicle::getTypeID(const std::string& vehID) {
    return Dom::getString(libsumo::VAR_TYPE, vehID);
}


std::string
Vehicle::getRouteID(const std::string& vehID) {
    return Dom::getString(libsumo::VAR_ROUTE_ID, vehID);
}


int
Vehicle::getRouteIndex(const std::string& vehID) {
    return Dom::getInt(libsumo::VAR_ROUTE_INDEX, vehID);
}


libsumo::TraCIColor
Vehicle::getColor(const std::string& vehID) {
    return Dom::getCol(libsumo::VAR_COLOR, vehID);
}

double
Vehicle::getLanePosition(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_LANEPOSITION, vehID);
}

double
Vehicle::getLateralLanePosition(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_LANEPOSITION_LAT, vehID);
}

double
Vehicle::getCO2Emission(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_CO2EMISSION, vehID);
}

double
Vehicle::getCOEmission(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_COEMISSION, vehID);
}

double
Vehicle::getHCEmission(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_HCEMISSION, vehID);
}

double
Vehicle::getPMxEmission(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_PMXEMISSION, vehID);
}

double
Vehicle::getNOxEmission(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_NOXEMISSION, vehID);
}

double
Vehicle::getFuelConsumption(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_FUELCONSUMPTION, vehID);
}

double
Vehicle::getNoiseEmission(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_NOISEEMISSION, vehID);
}

double
Vehicle::getElectricityConsumption(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_ELECTRICITYCONSUMPTION, vehID);
}

int
Vehicle::getPersonNumber(const std::string& vehID) {
    return Dom::getInt(libsumo::VAR_PERSON_NUMBER, vehID);
}

int
Vehicle::getPersonCapacity(const std::string& vehID) {
    return Dom::getInt(libsumo::VAR_PERSON_CAPACITY, vehID);
}


double
Vehicle::getBoardingDuration(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_BOARDING_DURATION, vehID);
}


double
Vehicle::getImpatience(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_IMPATIENCE, vehID);
}


std::vector<std::string>
Vehicle::getPersonIDList(const std::string& vehID) {
    return Dom::getStringVector(libsumo::LAST_STEP_PERSON_ID_LIST, vehID);
}

std::pair<std::string, double>
Vehicle::getLeader(const std::string& vehID, double dist) {
    tcpip::Storage content;
    StoHelp::writeTypedDouble(content, dist);
    std::unique_lock<std::mutex> lock{ libtraci::Connection::getActive().getMutex() };
    tcpip::Storage& ret = Dom::get(libsumo::VAR_LEADER, vehID, &content);
    ret.readInt(); // components
    ret.readUnsignedByte();
    const std::string leaderID = ret.readString();
    ret.readUnsignedByte();
    const double gap = ret.readDouble();
    return std::make_pair(leaderID, gap);
}


std::pair<std::string, double>
Vehicle::getFollower(const std::string& vehID, double dist) {
    tcpip::Storage content;
    StoHelp::writeTypedDouble(content, dist);
    std::unique_lock<std::mutex> lock{ libtraci::Connection::getActive().getMutex() };
    tcpip::Storage& ret = Dom::get(libsumo::VAR_FOLLOWER, vehID, &content);
    ret.readInt(); // components
    const std::string leaderID = StoHelp::readTypedString(ret);
    return std::make_pair(leaderID, StoHelp::readTypedDouble(ret));
}


std::vector<libsumo::TraCIJunctionFoe>
Vehicle::getJunctionFoes(const std::string& vehID, double dist) {
    std::vector<libsumo::TraCIJunctionFoe> result;
    tcpip::Storage content;
    StoHelp::writeTypedDouble(content, dist);
    std::unique_lock<std::mutex> lock{ libtraci::Connection::getActive().getMutex() };
    tcpip::Storage& ret = Dom::get(libsumo::VAR_FOES, vehID, &content);
    ret.readInt(); // compound size
    StoHelp::readJunctionFoeVector(ret, result);
    return result;
}


double
Vehicle::getWaitingTime(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_WAITING_TIME, vehID);
}


double
Vehicle::getAccumulatedWaitingTime(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_ACCUMULATED_WAITING_TIME, vehID);
}


double
Vehicle::getAdaptedTraveltime(const std::string& vehID, double time, const std::string& edgeID) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 2);
    StoHelp::writeTypedDouble(content, time);
    StoHelp::writeTypedString(content, edgeID);
    return Dom::getDouble(libsumo::VAR_EDGE_TRAVELTIME, vehID, &content);
}


double
Vehicle::getEffort(const std::string& vehID, double time, const std::string& edgeID) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 2);
    StoHelp::writeTypedDouble(content, time);
    StoHelp::writeTypedString(content, edgeID);
    return Dom::getDouble(libsumo::VAR_EDGE_EFFORT, vehID, &content);
}


bool
Vehicle::isRouteValid(const std::string& vehID) {
    return Dom::getInt(libsumo::VAR_ROUTE_VALID, vehID) != 0;
}


std::vector<std::string>
Vehicle::getRoute(const std::string& vehID) {
    return Dom::getStringVector(libsumo::VAR_EDGES, vehID);
}


int
Vehicle::getSignals(const std::string& vehID) {
    return Dom::getInt(libsumo::VAR_SIGNALS, vehID);
}


std::vector<libsumo::TraCIBestLanesData>
Vehicle::getBestLanes(const std::string& vehID) {
    std::unique_lock<std::mutex> lock{ libtraci::Connection::getActive().getMutex() };
    std::vector<libsumo::TraCIBestLanesData> result;
    tcpip::Storage& ret = Dom::get(libsumo::VAR_BEST_LANES, vehID);
    ret.readInt();
    StoHelp::readBestLanesVector(ret, result);
    return result;
}


std::vector<libsumo::TraCINextTLSData>
Vehicle::getNextTLS(const std::string& vehID) {
    std::unique_lock<std::mutex> lock{ libtraci::Connection::getActive().getMutex() };
    std::vector<libsumo::TraCINextTLSData> result;
    tcpip::Storage& ret = Dom::get(libsumo::VAR_NEXT_TLS, vehID);
    ret.readInt(); // components
    StoHelp::readTLSDataVector(ret, result);
    return result;
}

std::vector<libsumo::TraCINextStopData>
Vehicle::getNextStops(const std::string& vehID) {
    return getStops(vehID, 0);
}

std::vector<libsumo::TraCIConnection>
Vehicle::getNextLinks(const std::string& vehID) {
    std::unique_lock<std::mutex> lock{ libtraci::Connection::getActive().getMutex() };
    std::vector<libsumo::TraCIConnection> result;
    tcpip::Storage& ret = Dom::get(libsumo::VAR_NEXT_LINKS, vehID);
    ret.readInt(); // components
    // number of items
    const int linkNo = StoHelp::readTypedInt(ret);
    for (int i = 0; i < linkNo; ++i) {
        libsumo::TraCIConnection con;
        StoHelp::readConnection(ret, con);
        result.emplace_back(con);
    }
    return result;
}

std::vector<libsumo::TraCINextStopData>
Vehicle::getStops(const std::string& vehID, int limit) {
    std::vector<libsumo::TraCINextStopData> result;
    tcpip::Storage content;
    StoHelp::writeTypedInt(content, limit);
    std::unique_lock<std::mutex> lock{ libtraci::Connection::getActive().getMutex() };
    tcpip::Storage& ret = Dom::get(libsumo::VAR_NEXT_STOPS2, vehID, &content);
    ret.readInt(); // components
    StoHelp::readStopVector(ret, result);
    return result;
}

std::string
Vehicle::getStopParameter(const std::string& vehID, int nextStopIndex, const std::string& param, bool customParam) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 3);
    StoHelp::writeTypedInt(content, nextStopIndex);
    StoHelp::writeTypedString(content, param);
    StoHelp::writeTypedByte(content, customParam);
    return Dom::getString(libsumo::VAR_STOP_PARAMETER, vehID, &content);
}

int
Vehicle::getStopState(const std::string& vehID) {
    return Dom::getInt(libsumo::VAR_STOPSTATE, vehID);
}


double
Vehicle::getDistance(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_DISTANCE, vehID);
}


double
Vehicle::getDrivingDistance(const std::string& vehID, const std::string& edgeID, double pos, int laneIndex) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 2);
    content.writeUnsignedByte(libsumo::POSITION_ROADMAP);
    content.writeString(edgeID);
    content.writeDouble(pos);
    content.writeUnsignedByte(laneIndex);
    content.writeUnsignedByte(libsumo::REQUEST_DRIVINGDIST);
    return Dom::getDouble(libsumo::DISTANCE_REQUEST, vehID, &content);
}


double
Vehicle::getDrivingDistance2D(const std::string& vehID, double x, double y) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 2);
    content.writeUnsignedByte(libsumo::POSITION_2D);
    content.writeDouble(x);
    content.writeDouble(y);
    content.writeUnsignedByte(libsumo::REQUEST_DRIVINGDIST);
    return Dom::getDouble(libsumo::DISTANCE_REQUEST, vehID, &content);
}


double
Vehicle::getReferenceDistance(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_REFERENCE_DISTANCE, vehID);
}


double
Vehicle::getAllowedSpeed(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_ALLOWED_SPEED, vehID);
}


double
Vehicle::getSpeedFactor(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_SPEED_FACTOR, vehID);
}


int
Vehicle::getSpeedMode(const std::string& vehID) {
    return Dom::getInt(libsumo::VAR_SPEEDSETMODE, vehID);
}


int
Vehicle::getLaneChangeMode(const std::string& vehID) {
    return Dom::getInt(libsumo::VAR_LANECHANGE_MODE, vehID);
}


int
Vehicle::getRoutingMode(const std::string& vehID) {
    return Dom::getInt(libsumo::VAR_ROUTING_MODE, vehID);
}


std::string
Vehicle::getLine(const std::string& vehID) {
    return Dom::getString(libsumo::VAR_LINE, vehID);
}



std::vector<std::string>
Vehicle::getVia(const std::string& vehID) {
    return Dom::getStringVector(libsumo::VAR_VIA, vehID);
}


std::pair<int, int>
Vehicle::getLaneChangeState(const std::string& vehID, int direction) {
    tcpip::Storage content;
    StoHelp::writeTypedInt(content, direction);
    std::unique_lock<std::mutex> lock{ libtraci::Connection::getActive().getMutex() };
    tcpip::Storage& ret = Dom::get(libsumo::CMD_CHANGELANE, vehID, &content);
    ret.readInt(); // components
    const int stateWithoutTraCI = StoHelp::readTypedInt(ret);
    const int state = StoHelp::readTypedInt(ret);
    return std::make_pair(stateWithoutTraCI, state);
}


std::vector<std::pair<std::string, double> >
Vehicle::getNeighbors(const std::string& vehID, const int mode) {
    std::vector<std::pair<std::string, double> > neighs;
    tcpip::Storage content;
    content.writeUnsignedByte(libsumo::TYPE_UBYTE);
    content.writeUnsignedByte(mode);
    std::unique_lock<std::mutex> lock{ libtraci::Connection::getActive().getMutex() };
    tcpip::Storage& ret = Dom::get(libsumo::VAR_NEIGHBORS, vehID, &content);
    const int items = ret.readInt(); // components
    for (int i = 0; i < items; i++) {
        const std::string neighID = ret.readString();
        neighs.emplace_back(neighID, ret.readDouble());
    }
    return neighs;
}


double
Vehicle::getFollowSpeed(const std::string& vehID, double speed, double gap, double leaderSpeed, double leaderMaxDecel, const std::string& leaderID) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 5);
    StoHelp::writeTypedDouble(content, speed);
    StoHelp::writeTypedDouble(content, gap);
    StoHelp::writeTypedDouble(content, leaderSpeed);
    StoHelp::writeTypedDouble(content, leaderMaxDecel);
    StoHelp::writeTypedString(content, leaderID);
    return Dom::getDouble(libsumo::VAR_FOLLOW_SPEED, vehID, &content);
}


double
Vehicle::getSecureGap(const std::string& vehID, double speed, double leaderSpeed, double leaderMaxDecel, const std::string& leaderID) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 4);
    StoHelp::writeTypedDouble(content, speed);
    StoHelp::writeTypedDouble(content, leaderSpeed);
    StoHelp::writeTypedDouble(content, leaderMaxDecel);
    StoHelp::writeTypedString(content, leaderID);
    return Dom::getDouble(libsumo::VAR_SECURE_GAP, vehID, &content);
}


double
Vehicle::getStopSpeed(const std::string& vehID, const double speed, double gap) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 2);
    StoHelp::writeTypedDouble(content, speed);
    StoHelp::writeTypedDouble(content, gap);
    return Dom::getDouble(libsumo::VAR_STOP_SPEED, vehID, &content);
}

double
Vehicle::getStopDelay(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_STOP_DELAY, vehID);
}

double
Vehicle::getStopArrivalDelay(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_STOP_ARRIVALDELAY, vehID);
}

double
Vehicle::getTimeLoss(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_TIMELOSS, vehID);
}

std::vector<std::string>
Vehicle::getTaxiFleet(int taxiState) {
    tcpip::Storage content;
    StoHelp::writeTypedInt(content, taxiState);
    return Dom::getStringVector(libsumo::VAR_TAXI_FLEET, "", &content);
}

std::vector<std::string>
Vehicle::getLoadedIDList() {
    return Dom::getStringVector(libsumo::VAR_LOADED_LIST, "");
}

std::vector<std::string>
Vehicle::getTeleportingIDList() {
    return Dom::getStringVector(libsumo::VAR_TELEPORTING_LIST, "");
}

std::string
Vehicle::getEmissionClass(const std::string& vehID) {
    return Dom::getString(libsumo::VAR_EMISSIONCLASS, vehID);
}

std::string
Vehicle::getShapeClass(const std::string& vehID) {
    return Dom::getString(libsumo::VAR_SHAPECLASS, vehID);
}


double
Vehicle::getLength(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_LENGTH, vehID);
}


double
Vehicle::getAccel(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_ACCEL, vehID);
}


double
Vehicle::getDecel(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_DECEL, vehID);
}


double Vehicle::getEmergencyDecel(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_EMERGENCY_DECEL, vehID);
}


double Vehicle::getApparentDecel(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_APPARENT_DECEL, vehID);
}


double Vehicle::getActionStepLength(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_ACTIONSTEPLENGTH, vehID);
}


double Vehicle::getLastActionTime(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_LASTACTIONTIME, vehID);
}


double
Vehicle::getTau(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_TAU, vehID);
}


double
Vehicle::getImperfection(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_IMPERFECTION, vehID);
}


double
Vehicle::getSpeedDeviation(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_SPEED_DEVIATION, vehID);
}


std::string
Vehicle::getVehicleClass(const std::string& vehID) {
    return Dom::getString(libsumo::VAR_VEHICLECLASS, vehID);
}


double
Vehicle::getMinGap(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_MINGAP, vehID);
}


double
Vehicle::getMinGapLat(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_MINGAP_LAT, vehID);
}


double
Vehicle::getMaxSpeed(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_MAXSPEED, vehID);
}


double
Vehicle::getMaxSpeedLat(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_MAXSPEED_LAT, vehID);
}


std::string
Vehicle::getLateralAlignment(const std::string& vehID) {
    return Dom::getString(libsumo::VAR_LATALIGNMENT, vehID);
}


double
Vehicle::getWidth(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_WIDTH, vehID);
}


double
Vehicle::getHeight(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_HEIGHT, vehID);
}


double
Vehicle::getMass(const std::string& vehID) {
    return Dom::getDouble(libsumo::VAR_MASS, vehID);
}


void
Vehicle::setStop(const std::string& vehID,
                 const std::string& edgeID,
                 double pos,
                 int laneIndex,
                 double duration,
                 int flags,
                 double startPos,
                 double until) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 7);
    StoHelp::writeTypedString(content, edgeID);
    StoHelp::writeTypedDouble(content, pos);
    StoHelp::writeTypedByte(content, laneIndex);
    StoHelp::writeTypedDouble(content, duration);
    StoHelp::writeTypedByte(content, flags);
    StoHelp::writeTypedDouble(content, startPos);
    StoHelp::writeTypedDouble(content, until);
    Dom::set(libsumo::CMD_STOP, vehID, &content);
}


void
Vehicle::replaceStop(const std::string& vehID,
                     int nextStopIndex,
                     const std::string& edgeID,
                     double pos,
                     int laneIndex,
                     double duration,
                     int flags,
                     double startPos,
                     double until,
                     int teleport) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 9);
    StoHelp::writeTypedString(content, edgeID);
    StoHelp::writeTypedDouble(content, pos);
    StoHelp::writeTypedByte(content, laneIndex);
    StoHelp::writeTypedDouble(content, duration);
    StoHelp::writeTypedInt(content, flags);
    StoHelp::writeTypedDouble(content, startPos);
    StoHelp::writeTypedDouble(content, until);
    StoHelp::writeTypedInt(content, nextStopIndex);
    StoHelp::writeTypedByte(content, teleport);
    Dom::set(libsumo::CMD_REPLACE_STOP, vehID, &content);
}


void
Vehicle::insertStop(const std::string& vehID,
                    int nextStopIndex,
                    const std::string& edgeID,
                    double pos,
                    int laneIndex,
                    double duration,
                    int flags,
                    double startPos,
                    double until,
                    int teleport) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 9);
    StoHelp::writeTypedString(content, edgeID);
    StoHelp::writeTypedDouble(content, pos);
    StoHelp::writeTypedByte(content, laneIndex);
    StoHelp::writeTypedDouble(content, duration);
    StoHelp::writeTypedInt(content, flags);
    StoHelp::writeTypedDouble(content, startPos);
    StoHelp::writeTypedDouble(content, until);
    StoHelp::writeTypedInt(content, nextStopIndex);
    StoHelp::writeTypedByte(content, teleport);
    Dom::set(libsumo::CMD_INSERT_STOP, vehID, &content);
}


void
Vehicle::setStopParameter(const std::string& vehID, int nextStopIndex,
                          const std::string& param, const std::string& value,
                          bool customParam) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 4);
    StoHelp::writeTypedInt(content, nextStopIndex);
    StoHelp::writeTypedString(content, param);
    StoHelp::writeTypedString(content, value);
    StoHelp::writeTypedByte(content, customParam);
    Dom::set(libsumo::VAR_STOP_PARAMETER, vehID, &content);
}


void
Vehicle::rerouteParkingArea(const std::string& vehID, const std::string& parkingAreaID) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 1);
    StoHelp::writeTypedString(content, parkingAreaID);
    Dom::set(libsumo::CMD_REROUTE_TO_PARKING, vehID, &content);
}


void
Vehicle::resume(const std::string& vehID) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 0);
    Dom::set(libsumo::CMD_RESUME, vehID, &content);
}


void
Vehicle::changeTarget(const std::string& vehID, const std::string& edgeID) {
    Dom::setString(libsumo::CMD_CHANGETARGET, vehID, edgeID);
}


void
Vehicle::changeLane(const std::string& vehID, int laneIndex, double duration) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 2);
    StoHelp::writeTypedByte(content, laneIndex);
    StoHelp::writeTypedDouble(content, duration);
    Dom::set(libsumo::CMD_CHANGELANE, vehID, &content);
}

void
Vehicle::changeLaneRelative(const std::string& vehID, int indexOffset, double duration) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 3);
    StoHelp::writeTypedByte(content, indexOffset);
    StoHelp::writeTypedDouble(content, duration);
    StoHelp::writeTypedByte(content, 1);
    Dom::set(libsumo::CMD_CHANGELANE, vehID, &content);
}


void
Vehicle::changeSublane(const std::string& vehID, double latDist) {
    Dom::setDouble(libsumo::CMD_CHANGESUBLANE, vehID, latDist);
}


void
Vehicle::add(const std::string& vehID,
             const std::string& routeID,
             const std::string& typeID,
             const std::string& depart,
             const std::string& departLane,
             const std::string& departPos,
             const std::string& departSpeed,
             const std::string& arrivalLane,
             const std::string& arrivalPos,
             const std::string& arrivalSpeed,
             const std::string& fromTaz,
             const std::string& toTaz,
             const std::string& line,
             int personCapacity,
             int personNumber) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 14);
    StoHelp::writeTypedString(content, routeID);
    StoHelp::writeTypedString(content, typeID);
    StoHelp::writeTypedString(content, depart);
    StoHelp::writeTypedString(content, departLane);
    StoHelp::writeTypedString(content, departPos);
    StoHelp::writeTypedString(content, departSpeed);

    StoHelp::writeTypedString(content, arrivalLane);
    StoHelp::writeTypedString(content, arrivalPos);
    StoHelp::writeTypedString(content, arrivalSpeed);

    StoHelp::writeTypedString(content, fromTaz);
    StoHelp::writeTypedString(content, toTaz);
    StoHelp::writeTypedString(content, line);

    StoHelp::writeTypedInt(content, personCapacity);
    StoHelp::writeTypedInt(content, personNumber);

    Dom::set(libsumo::ADD_FULL, vehID, &content);
}


void
Vehicle::moveToXY(const std::string& vehID, const std::string& edgeID, const int laneIndex,
                  const double x, const double y, double angle, const int keepRoute, double matchThreshold) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 7);
    StoHelp::writeTypedString(content, edgeID);
    StoHelp::writeTypedInt(content, laneIndex);
    StoHelp::writeTypedDouble(content, x);
    StoHelp::writeTypedDouble(content, y);
    StoHelp::writeTypedDouble(content, angle);
    StoHelp::writeTypedByte(content, keepRoute);
    StoHelp::writeTypedDouble(content, matchThreshold);
    Dom::set(libsumo::MOVE_TO_XY, vehID, &content);
}

void
Vehicle::slowDown(const std::string& vehID, double speed, double duration) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 2);
    StoHelp::writeTypedDouble(content, speed);
    StoHelp::writeTypedDouble(content, duration);
    Dom::set(libsumo::CMD_SLOWDOWN, vehID, &content);
}

void
Vehicle::openGap(const std::string& vehID, double newTimeHeadway, double newSpaceHeadway, double duration, double changeRate, double maxDecel, const std::string& referenceVehID) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, referenceVehID != "" ? 6 : 5);
    StoHelp::writeTypedDouble(content, newTimeHeadway);
    StoHelp::writeTypedDouble(content, newSpaceHeadway);
    StoHelp::writeTypedDouble(content, duration);
    StoHelp::writeTypedDouble(content, changeRate);
    StoHelp::writeTypedDouble(content, maxDecel);
    if (referenceVehID != "") {
        StoHelp::writeTypedString(content, referenceVehID);
    }
    Dom::set(libsumo::CMD_OPENGAP, vehID, &content);
}

void
Vehicle::deactivateGapControl(const std::string& vehID) {
    openGap(vehID, -1, -1, -1, -1);
}

void
Vehicle::requestToC(const std::string& vehID, double leadTime) {
    std::ostringstream oss;
    oss.setf(std::ios::fixed, std::ios::floatfield);
    oss << std::setprecision(2);
    oss << leadTime;
    setParameter(vehID, "device.toc.requestToC", oss.str());
}

void
Vehicle::setSpeed(const std::string& vehID, double speed) {
    Dom::setDouble(libsumo::VAR_SPEED, vehID, speed);
}

void
Vehicle::setAcceleration(const std::string& vehID, double acceleration, double duration) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 2);
    StoHelp::writeTypedDouble(content, acceleration);
    StoHelp::writeTypedDouble(content, duration);
    Dom::set(libsumo::VAR_ACCELERATION, vehID, &content);
}

void
Vehicle::setPreviousSpeed(const std::string& vehID, double prevSpeed, double prevAcceleration) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 2);
    StoHelp::writeTypedDouble(content, prevSpeed);
    StoHelp::writeTypedDouble(content, prevAcceleration);
    Dom::set(libsumo::VAR_PREV_SPEED, vehID, &content);
}

void
Vehicle::setSpeedMode(const std::string& vehID, int speedMode) {
    Dom::setInt(libsumo::VAR_SPEEDSETMODE, vehID, speedMode);
}

void
Vehicle::setLaneChangeMode(const std::string& vehID, int laneChangeMode) {
    Dom::setInt(libsumo::VAR_LANECHANGE_MODE, vehID, laneChangeMode);
}

void
Vehicle::setRoutingMode(const std::string& vehID, int routingMode) {
    Dom::setInt(libsumo::VAR_ROUTING_MODE, vehID, routingMode);
}

void
Vehicle::setType(const std::string& vehID, const std::string& typeID) {
    Dom::setString(libsumo::VAR_TYPE, vehID, typeID);
}

void
Vehicle::setRouteID(const std::string& vehID, const std::string& routeID) {
    Dom::setString(libsumo::VAR_ROUTE_ID, vehID, routeID);
}

void
Vehicle::setRoute(const std::string& vehID, const std::string& edgeID) {
    setRoute(vehID, std::vector<std::string>({edgeID}));
}

void
Vehicle::setRoute(const std::string& vehID, const std::vector<std::string>& edgeIDs) {
    Dom::setStringVector(libsumo::VAR_ROUTE, vehID, edgeIDs);
}

void
Vehicle::setLateralLanePosition(const std::string& vehID, double posLat) {
    Dom::setDouble(libsumo::VAR_LANEPOSITION_LAT, vehID, posLat);
}

void
Vehicle::updateBestLanes(const std::string& vehID) {
    tcpip::Storage content;
    Dom::set(libsumo::VAR_UPDATE_BESTLANES, vehID, &content);
}


void
Vehicle::setAdaptedTraveltime(const std::string& vehID, const std::string& edgeID,
                              double time, double begSeconds, double endSeconds) {
    tcpip::Storage content;
    if (time == libsumo::INVALID_DOUBLE_VALUE) {
        // reset
        StoHelp::writeCompound(content, 1);
        StoHelp::writeTypedString(content, edgeID);
    } else if (begSeconds == libsumo::INVALID_DOUBLE_VALUE) {
        // set value for the whole simulation
        StoHelp::writeCompound(content, 2);
        StoHelp::writeTypedString(content, edgeID);
        StoHelp::writeTypedDouble(content, time);
    } else {
        StoHelp::writeCompound(content, 4);
        StoHelp::writeTypedDouble(content, begSeconds);
        StoHelp::writeTypedDouble(content, endSeconds);
        StoHelp::writeTypedString(content, edgeID);
        StoHelp::writeTypedDouble(content, time);
    }
    Dom::set(libsumo::VAR_EDGE_TRAVELTIME, vehID, &content);
}


void
Vehicle::setEffort(const std::string& vehID, const std::string& edgeID,
                   double effort, double begSeconds, double endSeconds) {
    tcpip::Storage content;
    if (effort == libsumo::INVALID_DOUBLE_VALUE) {
        // reset
        StoHelp::writeCompound(content, 1);
        StoHelp::writeTypedString(content, edgeID);
    } else if (begSeconds == libsumo::INVALID_DOUBLE_VALUE) {
        // set value for the whole simulation
        StoHelp::writeCompound(content, 2);
        StoHelp::writeTypedString(content, edgeID);
        StoHelp::writeTypedDouble(content, effort);
    } else {
        StoHelp::writeCompound(content, 4);
        StoHelp::writeTypedDouble(content, begSeconds);
        StoHelp::writeTypedDouble(content, endSeconds);
        StoHelp::writeTypedString(content, edgeID);
        StoHelp::writeTypedDouble(content, effort);
    }
    Dom::set(libsumo::VAR_EDGE_EFFORT, vehID, &content);
}


void
Vehicle::rerouteTraveltime(const std::string& vehID, const bool /* currentTravelTimes */) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 0);
    Dom::set(libsumo::CMD_REROUTE_TRAVELTIME, vehID, &content);
}


void
Vehicle::rerouteEffort(const std::string& vehID) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 0);
    Dom::set(libsumo::CMD_REROUTE_EFFORT, vehID, &content);
}


void
Vehicle::setSignals(const std::string& vehID, int signals) {
    Dom::setInt(libsumo::VAR_SIGNALS, vehID, signals);
}


void
Vehicle::moveTo(const std::string& vehID, const std::string& laneID, double pos, int reason) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, 3);
    StoHelp::writeTypedString(content, laneID);
    StoHelp::writeTypedDouble(content, pos);
    StoHelp::writeTypedInt(content, reason);
    Dom::set(libsumo::VAR_MOVE_TO, vehID, &content);
}


void
Vehicle::setActionStepLength(const std::string& vehID, double actionStepLength, bool resetActionOffset) {
    //if (actionStepLength < 0) {
    //    raise TraCIException("Invalid value for actionStepLength. Given value must be non-negative.")
    //{
    // Use negative value to indicate resetActionOffset == False
    if (!resetActionOffset) {
        actionStepLength *= -1;
    }
    Dom::setDouble(libsumo::VAR_ACTIONSTEPLENGTH, vehID, actionStepLength);
}


void
Vehicle::remove(const std::string& vehID, char reason) {
    tcpip::Storage content;
    content.writeUnsignedByte(libsumo::TYPE_BYTE);
    content.writeUnsignedByte(reason);
    Dom::set(libsumo::REMOVE, vehID, &content);
}


void
Vehicle::setColor(const std::string& vehID, const libsumo::TraCIColor& color) {
    Dom::setCol(libsumo::VAR_COLOR, vehID, color);
}


void
Vehicle::setSpeedFactor(const std::string& vehID, double factor) {
    Dom::setDouble(libsumo::VAR_SPEED_FACTOR, vehID, factor);
}


void
Vehicle::setLine(const std::string& vehID, const std::string& line) {
    Dom::setString(libsumo::VAR_LINE, vehID, line);
}


void
Vehicle::setVia(const std::string& vehID, const std::vector<std::string>& edgeList) {
    Dom::setStringVector(libsumo::VAR_VIA, vehID, edgeList);
}


void
Vehicle::setLength(const std::string& vehID, double length) {
    Dom::setDouble(libsumo::VAR_LENGTH, vehID, length);
}


void
Vehicle::setMaxSpeed(const std::string& vehID, double speed) {
    Dom::setDouble(libsumo::VAR_MAXSPEED, vehID, speed);
}


void
Vehicle::setVehicleClass(const std::string& vehID, const std::string& clazz) {
    Dom::setString(libsumo::VAR_VEHICLECLASS, vehID, clazz);
}


void
Vehicle::setShapeClass(const std::string& vehID, const std::string& clazz) {
    Dom::setString(libsumo::VAR_SHAPECLASS, vehID, clazz);
}


void
Vehicle::setEmissionClass(const std::string& vehID, const std::string& clazz) {
    Dom::setString(libsumo::VAR_EMISSIONCLASS, vehID, clazz);
}


void
Vehicle::setWidth(const std::string& vehID, double width) {
    Dom::setDouble(libsumo::VAR_WIDTH, vehID, width);
}


void
Vehicle::setHeight(const std::string& vehID, double height) {
    Dom::setDouble(libsumo::VAR_HEIGHT, vehID, height);
}


void
Vehicle::setMass(const std::string& vehID, double mass) {
    Dom::setDouble(libsumo::VAR_MASS, vehID, mass);
}


void
Vehicle::setMinGap(const std::string& vehID, double minGap) {
    Dom::setDouble(libsumo::VAR_MINGAP, vehID, minGap);
}


void
Vehicle::setAccel(const std::string& vehID, double accel) {
    Dom::setDouble(libsumo::VAR_ACCEL, vehID, accel);
}


void
Vehicle::setDecel(const std::string& vehID, double decel) {
    Dom::setDouble(libsumo::VAR_DECEL, vehID, decel);
}


void
Vehicle::setEmergencyDecel(const std::string& vehID, double decel) {
    Dom::setDouble(libsumo::VAR_EMERGENCY_DECEL, vehID, decel);
}


void
Vehicle::setApparentDecel(const std::string& vehID, double decel) {
    Dom::setDouble(libsumo::VAR_APPARENT_DECEL, vehID, decel);
}


void
Vehicle::setImperfection(const std::string& vehID, double imperfection) {
    Dom::setDouble(libsumo::VAR_IMPERFECTION, vehID, imperfection);
}


void
Vehicle::setTau(const std::string& vehID, double tau) {
    Dom::setDouble(libsumo::VAR_TAU, vehID, tau);
}


void
Vehicle::setMinGapLat(const std::string& vehID, double minGapLat) {
    Dom::setDouble(libsumo::VAR_MINGAP_LAT, vehID, minGapLat);
}


void
Vehicle::setMaxSpeedLat(const std::string& vehID, double speed) {
    Dom::setDouble(libsumo::VAR_MAXSPEED_LAT, vehID, speed);
}


void
Vehicle::setLateralAlignment(const std::string& vehID, const std::string& latAlignment) {
    Dom::setString(libsumo::VAR_LATALIGNMENT, vehID, latAlignment);
}


void
Vehicle::setImpatience(const std::string& vehID, double impatience) {
    Dom::setDouble(libsumo::VAR_IMPATIENCE, vehID, impatience);
}

void
Vehicle::setBoardingDuration(const std::string& vehID, double boardingDuration) {
    Dom::setDouble(libsumo::VAR_BOARDING_DURATION, vehID, boardingDuration);
}


void
Vehicle::highlight(const std::string& vehID, const libsumo::TraCIColor& col, double size, const int alphaMax, const double duration, const int type) {
    tcpip::Storage content;
    StoHelp::writeCompound(content, alphaMax > 0 ? 5 : 2);
    content.writeUnsignedByte(libsumo::TYPE_COLOR);
    content.writeUnsignedByte(col.r);
    content.writeUnsignedByte(col.g);
    content.writeUnsignedByte(col.b);
    content.writeUnsignedByte(col.a);
    StoHelp::writeTypedDouble(content, size);
    if (alphaMax > 0) {
        content.writeUnsignedByte(libsumo::TYPE_UBYTE);
        content.writeUnsignedByte(alphaMax);
        StoHelp::writeTypedDouble(content, duration);
        content.writeUnsignedByte(libsumo::TYPE_UBYTE);
        content.writeUnsignedByte(type);
    }
    Dom::set(libsumo::VAR_HIGHLIGHT, vehID, &content);
}

void
Vehicle::dispatchTaxi(const std::string& vehID,  const std::vector<std::string>& reservations) {
    Dom::setStringVector(libsumo::CMD_TAXI_DISPATCH, vehID, reservations);
}


void
Vehicle::subscribeLeader(const std::string& vehID, double dist, double begin, double end) {
    subscribe(vehID, std::vector<int>({ libsumo::VAR_LEADER }), begin, end,
    libsumo::TraCIResults({ {libsumo::VAR_LEADER, std::make_shared<libsumo::TraCIDouble>(dist)} }));
}


void
Vehicle::addSubscriptionFilterLanes(const std::vector<int>& lanes, bool noOpposite, double downstreamDist, double upstreamDist) {
    tcpip::Storage content;
    content.writeUnsignedByte((int)lanes.size());
    for (int lane : lanes) {
        content.writeUnsignedByte(lane < 0 ? lane + 256 : lane);
    }
    libtraci::Connection::getActive().addFilter(libsumo::FILTER_TYPE_LANES, &content);
    if (noOpposite) {
        addSubscriptionFilterNoOpposite();
    }
    if (downstreamDist != libsumo::INVALID_DOUBLE_VALUE) {
        addSubscriptionFilterDownstreamDistance(downstreamDist);
    }
    if (upstreamDist != libsumo::INVALID_DOUBLE_VALUE) {
        addSubscriptionFilterUpstreamDistance(upstreamDist);
    }
}


void
Vehicle::addSubscriptionFilterNoOpposite() {
    libtraci::Connection::getActive().addFilter(libsumo::FILTER_TYPE_NOOPPOSITE);
}


void
Vehicle::addSubscriptionFilterDownstreamDistance(double dist) {
    tcpip::Storage content;
    StoHelp::writeTypedDouble(content, dist);
    libtraci::Connection::getActive().addFilter(libsumo::FILTER_TYPE_DOWNSTREAM_DIST, &content);
}


void
Vehicle::addSubscriptionFilterUpstreamDistance(double dist) {
    tcpip::Storage content;
    StoHelp::writeTypedDouble(content, dist);
    libtraci::Connection::getActive().addFilter(libsumo::FILTER_TYPE_UPSTREAM_DIST, &content);
}


void
Vehicle::addSubscriptionFilterCFManeuver(double downstreamDist, double upstreamDist) {
    addSubscriptionFilterLeadFollow(std::vector<int>(1));
    if (downstreamDist != libsumo::INVALID_DOUBLE_VALUE) {
        addSubscriptionFilterDownstreamDistance(downstreamDist);
    }
    if (upstreamDist != libsumo::INVALID_DOUBLE_VALUE) {
        addSubscriptionFilterUpstreamDistance(upstreamDist);
    }
}


void
Vehicle::addSubscriptionFilterLCManeuver(int direction, bool noOpposite, double downstreamDist, double upstreamDist) {
    if (direction == libsumo::INVALID_INT_VALUE) {
        addSubscriptionFilterLeadFollow({ -1, 0, 1 });
    } else if (direction != -1 && direction != 1) {
        // warnings.warn("Ignoring lane change subscription filter with non-neighboring lane offset direction=%s." % direction)
        return;
    } else {
        addSubscriptionFilterLeadFollow({ 0, direction });
    }
    if (noOpposite) {
        addSubscriptionFilterNoOpposite();
    }
    if (downstreamDist != libsumo::INVALID_DOUBLE_VALUE) {
        addSubscriptionFilterDownstreamDistance(downstreamDist);
    }
    if (upstreamDist != libsumo::INVALID_DOUBLE_VALUE) {
        addSubscriptionFilterUpstreamDistance(upstreamDist);
    }
}


void
Vehicle::addSubscriptionFilterLeadFollow(const std::vector<int>& lanes) {
    libtraci::Connection::getActive().addFilter(libsumo::FILTER_TYPE_LEAD_FOLLOW);
    addSubscriptionFilterLanes(lanes);
}


void
Vehicle::addSubscriptionFilterTurn(double downstreamDist, double foeDistToJunction) {
    tcpip::Storage content;
    StoHelp::writeTypedDouble(content, foeDistToJunction);
    libtraci::Connection::getActive().addFilter(libsumo::FILTER_TYPE_TURN, &content);
    if (downstreamDist != libsumo::INVALID_DOUBLE_VALUE) {
        addSubscriptionFilterDownstreamDistance(downstreamDist);
    }
}


void
Vehicle::addSubscriptionFilterVClass(const std::vector<std::string>& vClasses) {
    tcpip::Storage content;
    StoHelp::writeTypedStringList(content, vClasses);
    libtraci::Connection::getActive().addFilter(libsumo::FILTER_TYPE_VCLASS, &content);
}


void
Vehicle::addSubscriptionFilterVType(const std::vector<std::string>& vTypes) {
    tcpip::Storage content;
    StoHelp::writeTypedStringList(content, vTypes);
    libtraci::Connection::getActive().addFilter(libsumo::FILTER_TYPE_VTYPE, &content);
}


void
Vehicle::addSubscriptionFilterFieldOfVision(double openingAngle) {
    tcpip::Storage content;
    StoHelp::writeTypedDouble(content, openingAngle);
    libtraci::Connection::getActive().addFilter(libsumo::FILTER_TYPE_FIELD_OF_VISION, &content);
}


void
Vehicle::addSubscriptionFilterLateralDistance(double lateralDist, double downstreamDist, double upstreamDist) {
    tcpip::Storage content;
    StoHelp::writeTypedDouble(content, lateralDist);
    libtraci::Connection::getActive().addFilter(libsumo::FILTER_TYPE_LATERAL_DIST, &content);
    if (downstreamDist != libsumo::INVALID_DOUBLE_VALUE) {
        addSubscriptionFilterDownstreamDistance(downstreamDist);
    }
    if (upstreamDist != libsumo::INVALID_DOUBLE_VALUE) {
        addSubscriptionFilterUpstreamDistance(upstreamDist);
    }
}


}


/****************************************************************************/
