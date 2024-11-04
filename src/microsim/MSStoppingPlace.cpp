/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2005-2024 German Aerospace Center (DLR) and others.
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
/// @file    MSStoppingPlace.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Johannes Rummel
/// @date    Mon, 13.12.2005
///
// A lane area vehicles can halt at
/****************************************************************************/
#include <config.h>

#include <cassert>
#include <map>
#include <utils/vehicle/SUMOVehicle.h>
#include <utils/geom/Position.h>
#include <utils/common/RGBColor.h>
#include <microsim/transportables/MSTransportable.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSVehicleType.h>
#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSStop.h>
#include "MSStoppingPlace.h"

// ===========================================================================
// method definitions
// ===========================================================================
MSStoppingPlace::MSStoppingPlace(const std::string& id,
                                 SumoXMLTag element,
                                 const std::vector<std::string>& lines,
                                 MSLane& lane,
                                 double begPos, double endPos, const std::string name,
                                 int capacity,
                                 double parkingLength,
                                 const RGBColor& color) :
    Named(id),
    myElement(element),
    myLines(lines), myLane(lane),
    myBegPos(begPos), myEndPos(endPos),
    myLastFreePos(endPos),
    myLastParking(nullptr),
    myName(name),
    myTransportableCapacity(capacity),
    myParkingFactor(parkingLength <= 0 ? 1 : (endPos - begPos) / parkingLength),
    myColor(color),
    // see MSVehicleControl defContainerType
    myTransportableDepth(element == SUMO_TAG_CONTAINER_STOP ? SUMO_const_waitingContainerDepth : SUMO_const_waitingPersonDepth) {
    computeLastFreePos();
    for (int i = 0; i < capacity; i++) {
        myWaitingSpots.insert(i);
    }
}


MSStoppingPlace::~MSStoppingPlace() {}


const MSLane&
MSStoppingPlace::getLane() const {
    return myLane;
}


double
MSStoppingPlace::getBeginLanePosition() const {
    return myBegPos;
}


double
MSStoppingPlace::getEndLanePosition() const {
    return myEndPos;
}

Position
MSStoppingPlace::getCenterPos() const {
    return myLane.getShape().positionAtOffset(myLane.interpolateLanePosToGeometryPos((myBegPos + myEndPos) / 2),
            myLane.getWidth() / 2 + 0.5);
}


void
MSStoppingPlace::enter(SUMOVehicle* veh, bool parking) {
    double beg = veh->getPositionOnLane() + veh->getVehicleType().getMinGap() * (parking ? myParkingFactor : 1);
    double end = beg - veh->getVehicleType().getLengthWithGap() * (parking ? myParkingFactor : 1);
    myEndPositions[veh] = std::make_pair(beg, end);
    computeLastFreePos();
}


double
MSStoppingPlace::getLastFreePos(const SUMOVehicle& forVehicle, double /*brakePos*/) const {
    if (getStoppedVehicleNumber() > 0) {
        const double vehGap = forVehicle.getVehicleType().getMinGap();
        double pos = myLastFreePos - vehGap;
        if (myParkingFactor < 1 && myLastParking != nullptr && forVehicle.hasStops() && (forVehicle.getStops().front().pars.parking == ParkingType::ONROAD)
                && myLastParking->remainingStopDuration() < forVehicle.getStops().front().getMinDuration(SIMSTEP)) {
            // stop far back enough so that the previous parking vehicle can leave (even if this vehicle fits, it will
            // be a blocker because it stops on the road)
            pos = MIN2(pos, myLastParking->getPositionOnLane() - myLastParking->getLength() - vehGap - NUMERICAL_EPS);
        }
        if (forVehicle.getLane() == &myLane && forVehicle.getPositionOnLane() < myEndPos && forVehicle.getPositionOnLane() > myBegPos && forVehicle.getSpeed() <= SUMO_const_haltingSpeed) {
            return forVehicle.getPositionOnLane();
        }
        if (!fits(pos, forVehicle)) {
            // try to find a place ahead of the waiting vehicles
            const double vehLength = forVehicle.getVehicleType().getLength() * myParkingFactor;
            std::vector<std::pair<double, std::pair<double, const SUMOVehicle*> > > spaces;
            for (auto it : myEndPositions) {
                spaces.push_back(std::make_pair(it.second.first, std::make_pair(it.second.second, it.first)));
            }
            // sorted from myEndPos towars myBegPos
            std::sort(spaces.begin(), spaces.end());
            std::reverse(spaces.begin(), spaces.end());
            double prev = myEndPos;
            for (auto it : spaces) {
                //if (forVehicle.isSelected()) {
                //    std::cout << SIMTIME << " fitPosFor " << forVehicle.getID() << " l=" << vehLength << " prev=" << prev << " vehBeg=" << it.first << " vehEnd=" << it.second.first << " found=" << (prev - it.first >= vehLength) << "\n";
                //}
                if (prev - it.first + NUMERICAL_EPS >= vehLength && (
                            it.second.second->isParking()
                            || it.second.second->remainingStopDuration() > TIME2STEPS(10))) {
                    return prev;
                }
                prev = it.second.first - vehGap;
            }
            if (myParkingFactor < 1 && myLastParking != nullptr) {
                // stop far back enough so that the previous vehicle can leave
                pos = MIN2(pos, myLastParking->getPositionOnLane() - myLastParking->getLength() - vehGap - NUMERICAL_EPS);
            }
        }
        return pos;
    }
    return myLastFreePos;
}

bool
MSStoppingPlace::fits(double pos, const SUMOVehicle& veh) const {
    // always fit at the default position or if at least half the vehicle length
    // is within the stop range (debatable)
    return pos + POSITION_EPS >= myEndPos || (pos - myBegPos >= veh.getVehicleType().getLength() * myParkingFactor / 2);
}

double
MSStoppingPlace::getWaitingPositionOnLane(MSTransportable* t) const {
    auto it = myWaitingTransportables.find(t);
    const double waitingWidth = myElement == SUMO_TAG_CONTAINER_STOP
                                ? SUMO_const_waitingContainerWidth
                                : SUMO_const_waitingPersonWidth;
    if (it != myWaitingTransportables.end() && it->second >= 0) {
        return myEndPos - (0.5 + (it->second) % getTransportablesAbreast()) * waitingWidth;
    } else {
        return (myEndPos + myBegPos) / 2;
    }
}


int
MSStoppingPlace::getTransportablesAbreast(double length, SumoXMLTag element) {
    return MAX2(1, (int)floor(length / (element == SUMO_TAG_CONTAINER_STOP
                                        ? SUMO_const_waitingContainerWidth
                                        : SUMO_const_waitingPersonWidth)));
}

int
MSStoppingPlace::getTransportablesAbreast() const {
    return getTransportablesAbreast(myEndPos - myBegPos, myElement);
}

Position
MSStoppingPlace::getWaitPosition(MSTransportable* t) const {
    double lanePos = getWaitingPositionOnLane(t);
    int row = 0;
    auto it = myWaitingTransportables.find(t);
    if (it != myWaitingTransportables.end()) {
        if (it->second >= 0) {
            row = int(it->second / getTransportablesAbreast());
        } else {
            // invalid position, draw outside bounds
            row = 1 + myTransportableCapacity / getTransportablesAbreast();
        }
    }
    const double lefthandSign = (MSGlobals::gLefthand ? -1 : 1);
    return myLane.getShape().positionAtOffset(myLane.interpolateLanePosToGeometryPos(lanePos),
            lefthandSign * (myLane.getWidth() / 2 + row * myTransportableDepth));
}


double
MSStoppingPlace::getStoppingPosition(const SUMOVehicle* veh) const {
    auto i = myEndPositions.find(veh);
    if (i != myEndPositions.end()) {
        return i->second.second;
    } else {
        return getLastFreePos(*veh);
    }
}

std::vector<const MSTransportable*>
MSStoppingPlace::getTransportables() const {
    std::vector<const MSTransportable*> result;
    for (auto item : myWaitingTransportables) {
        result.push_back(item.first);
    }
    return result;
}

bool
MSStoppingPlace::hasSpaceForTransportable() const {
    return myWaitingSpots.size() > 0;
}

bool
MSStoppingPlace::addTransportable(const MSTransportable* p) {
    int spot = -1;
    if (!hasSpaceForTransportable()) {
        return false;
    }
    spot = *myWaitingSpots.begin();
    myWaitingSpots.erase(myWaitingSpots.begin());
    myWaitingTransportables[p] = spot;
    return true;
}


void
MSStoppingPlace::removeTransportable(const MSTransportable* p) {
    auto i = myWaitingTransportables.find(p);
    if (i != myWaitingTransportables.end()) {
        if (i->second >= 0) {
            myWaitingSpots.insert(i->second);
        }
        myWaitingTransportables.erase(i);
    }
}


void
MSStoppingPlace::leaveFrom(SUMOVehicle* what) {
    assert(myEndPositions.find(what) != myEndPositions.end());
    myEndPositions.erase(myEndPositions.find(what));
    computeLastFreePos();
}


void
MSStoppingPlace::computeLastFreePos() {
    myLastFreePos = myEndPos;
    myLastParking = nullptr;
    for (auto item : myEndPositions) {
        // vehicle might be stopped beyond myEndPos
        if (myLastFreePos >= item.second.second || myLastFreePos == myEndPos) {
            myLastFreePos = item.second.second;
            if (item.first->isStoppedParking()) {
                myLastParking = item.first;
            }
        }
    }
}


double
MSStoppingPlace::getAccessPos(const MSEdge* edge, SumoRNG* rng) const {
    if (edge == &myLane.getEdge()) {
        return (myBegPos + myEndPos) / 2.;
    }
    for (const auto& access : myAccessPos) {
        if (edge == &access.lane->getEdge()) {
            if (rng == nullptr || access.startPos == access.endPos) {
                return access.endPos;
            }
            return RandHelper::rand(access.startPos, access.endPos, rng);
        }
    }
    return -1.;
}


const MSStoppingPlace::Access*
MSStoppingPlace::getAccess(const MSEdge* edge) const {
    for (const auto& access : myAccessPos) {
        if (edge == &access.lane->getEdge()) {
            return &access;
        }
    }
    return nullptr;
}


const std::string&
MSStoppingPlace::getMyName() const {
    return myName;
}


const RGBColor&
MSStoppingPlace::getColor() const {
    return myColor;
}


bool
MSStoppingPlace::addAccess(MSLane* const lane, const double startPos, const double endPos, double length, const MSStoppingPlace::AccessExit exit) {
    // prevent multiple accesses on the same lane
    for (const auto& access : myAccessPos) {
        if (lane == access.lane) {
            return false;
        }
    }
    if (length < 0.) {
        const Position accPos = lane->geometryPositionAtOffset((startPos + endPos) / 2.);
        const Position stopPos = myLane.geometryPositionAtOffset((myBegPos + myEndPos) / 2.);
        length = accPos.distanceTo(stopPos);
    }
    myAccessPos.push_back({lane, startPos, endPos, length, exit});
    return true;
}


std::vector<const SUMOVehicle*>
MSStoppingPlace::getStoppedVehicles() const {
    std::vector<const SUMOVehicle*> result;
    for (auto item : myEndPositions) {
        result.push_back(item.first);
    }
    return result;
}


void
MSStoppingPlace::getWaitingPersonIDs(std::vector<std::string>& into) const {
    for (auto item : myWaitingTransportables) {
        into.push_back(item.first->getID());
    }
    std::sort(into.begin(), into.end());
}


void
MSStoppingPlace::clearState() {
    myEndPositions.clear();
    myWaitingTransportables.clear();
    computeLastFreePos();
}


/****************************************************************************/
