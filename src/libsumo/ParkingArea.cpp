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
/// @author  Jakob Erdmann
/// @author  Mirko Barthauer
/// @date    16.03.2020
///
// C++ TraCI client API implementation
/****************************************************************************/
#include <config.h>

#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSStoppingPlace.h>
#include <microsim/MSParkingArea.h>
#include <libsumo/TraCIConstants.h>
#include "Helper.h"
#include "ParkingArea.h"


namespace libsumo {
// ===========================================================================
// static member initializations
// ===========================================================================
SubscriptionResults ParkingArea::mySubscriptionResults;
ContextSubscriptionResults ParkingArea::myContextSubscriptionResults;


// ===========================================================================
// static member definitions
// ===========================================================================
std::vector<std::string>
ParkingArea::getIDList() {
    std::vector<std::string> ids;
    for (auto& item : MSNet::getInstance()->getStoppingPlaces(SUMO_TAG_PARKING_AREA)) {
        ids.push_back(item.first);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

int
ParkingArea::getIDCount() {
    return (int)getIDList().size();
}


std::string
ParkingArea::getLaneID(const std::string& stopID) {
    return getParkingArea(stopID)->getLane().getID();
}

double
ParkingArea::getStartPos(const std::string& stopID) {
    return getParkingArea(stopID)->getBeginLanePosition();
}

double
ParkingArea::getEndPos(const std::string& stopID) {
    return getParkingArea(stopID)->getEndLanePosition();
}

std::string
ParkingArea::getName(const std::string& stopID) {
    return getParkingArea(stopID)->getMyName();
}

int
ParkingArea::getVehicleCount(const std::string& stopID) {
    return (int)getParkingArea(stopID)->getStoppedVehicles().size();
}

std::vector<std::string>
ParkingArea::getVehicleIDs(const std::string& stopID) {
    std::vector<std::string> result;
    for (const SUMOVehicle* veh : getParkingArea(stopID)->getStoppedVehicles()) {
        result.push_back(veh->getID());
    }
    return result;
}

std::vector<std::string>
ParkingArea::getAcceptedBadges(const std::string& stopID) {
    return dynamic_cast<MSParkingArea*>(getParkingArea(stopID))->getAcceptedBadges();
}


void
ParkingArea::setAcceptedBadges(const std::string& stopID, const std::vector<std::string>& badges) {
    dynamic_cast<MSParkingArea*>(getParkingArea(stopID))->setAcceptedBadges(badges);
}


std::string
ParkingArea::getParameter(const std::string& stopID, const std::string& param) {
    return getParkingArea(stopID)->getParameter(param, "");
}


LIBSUMO_GET_PARAMETER_WITH_KEY_IMPLEMENTATION(ParkingArea)


void
ParkingArea::setParameter(const std::string& stopID, const std::string& key, const std::string& value) {
    getParkingArea(stopID)->setParameter(key, value);
}


LIBSUMO_SUBSCRIPTION_IMPLEMENTATION(ParkingArea, PARKINGAREA)


MSStoppingPlace*
ParkingArea::getParkingArea(const std::string& id) {
    return Helper::getStoppingPlace(id, SUMO_TAG_PARKING_AREA);
}


std::shared_ptr<VariableWrapper>
ParkingArea::makeWrapper() {
    return std::make_shared<Helper::SubscriptionWrapper>(handleVariable, mySubscriptionResults, myContextSubscriptionResults);
}


bool
ParkingArea::handleVariable(const std::string& objID, const int variable, VariableWrapper* wrapper, tcpip::Storage* paramData) {
    switch (variable) {
        case TRACI_ID_LIST:
            return wrapper->wrapStringList(objID, variable, getIDList());
        case ID_COUNT:
            return wrapper->wrapInt(objID, variable, getIDCount());
        case VAR_LANE_ID:
            return wrapper->wrapString(objID, variable, getLaneID(objID));
        case VAR_POSITION:
            return wrapper->wrapDouble(objID, variable, getStartPos(objID));
        case VAR_LANEPOSITION:
            return wrapper->wrapDouble(objID, variable, getEndPos(objID));
        case VAR_NAME:
            return wrapper->wrapString(objID, variable, getName(objID));
        case VAR_ACCESS_BADGE:
            return wrapper->wrapStringList(objID, variable, getAcceptedBadges(objID));
        case VAR_STOP_STARTING_VEHICLES_NUMBER:
            return wrapper->wrapInt(objID, variable, getVehicleCount(objID));
        case VAR_STOP_STARTING_VEHICLES_IDS:
            return wrapper->wrapStringList(objID, variable, getVehicleIDs(objID));
        case libsumo::VAR_PARAMETER:
            paramData->readUnsignedByte();
            return wrapper->wrapString(objID, variable, getParameter(objID, paramData->readString()));
        case libsumo::VAR_PARAMETER_WITH_KEY:
            paramData->readUnsignedByte();
            return wrapper->wrapStringPair(objID, variable, getParameterWithKey(objID, paramData->readString()));
        default:
            return false;
    }
}

}


/****************************************************************************/
