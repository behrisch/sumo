/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2017-2024 German Aerospace Center (DLR) and others.
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
/// @file    ParkingArea.cpp
/// @author  Angelo Banse
/// @author  Mirko Barthauer
/// @date    10.11.2020
///
// C++ TraCI client API implementation
/****************************************************************************/
#include <config.h>

#define LIBTRACI 1
#include <libsumo/TraCIConstants.h>
#include <libsumo/ParkingArea.h>
#include "Connection.h"
#include "Domain.h"


namespace libtraci {

typedef Domain<libsumo::CMD_GET_PARKINGAREA_VARIABLE, libsumo::CMD_SET_PARKINGAREA_VARIABLE> Dom;


// ===========================================================================
// static member definitions
// ===========================================================================
std::vector<std::string>
ParkingArea::getIDList() {
    return Dom::getStringVector(libsumo::TRACI_ID_LIST, "");
}

int
ParkingArea::getIDCount() {
    return Dom::getInt(libsumo::ID_COUNT, "");
}


std::string
ParkingArea::getLaneID(const std::string& stopID) {
    return Dom::getString(libsumo::VAR_LANE_ID, stopID);
}

double
ParkingArea::getStartPos(const std::string& stopID) {
    return Dom::getDouble(libsumo::VAR_POSITION, stopID);
}

double
ParkingArea::getEndPos(const std::string& stopID) {
    return Dom::getDouble(libsumo::VAR_LANEPOSITION, stopID);
}

std::string
ParkingArea::getName(const std::string& stopID) {
    return Dom::getString(libsumo::VAR_NAME, stopID);
}

int
ParkingArea::getVehicleCount(const std::string& stopID) {
    return Dom::getInt(libsumo::VAR_STOP_STARTING_VEHICLES_NUMBER, stopID);
}

std::vector<std::string>
ParkingArea::getVehicleIDs(const std::string& stopID) {
    return Dom::getStringVector(libsumo::VAR_STOP_STARTING_VEHICLES_IDS, stopID);
}

std::vector<std::string>
ParkingArea::getAcceptedBadges(const std::string& stopID) {
    return Dom::getStringVector(libsumo::VAR_ACCESS_BADGE, stopID);
}

void
ParkingArea::setAcceptedBadges(const std::string& stopID, const std::vector<std::string>& badges) {
    Dom::setStringVector(libsumo::VAR_ACCESS_BADGE, stopID, badges);
}

LIBTRACI_SUBSCRIPTION_IMPLEMENTATION(ParkingArea, PARKINGAREA)
LIBTRACI_PARAMETER_IMPLEMENTATION(ParkingArea, PARKINGAREA)


}


/****************************************************************************/
