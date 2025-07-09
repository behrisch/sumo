/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2001-2025 German Aerospace Center (DLR) and others.
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
/// @file    MSLane.cpp
/// @author  Christian Roessel
/// @author  Jakob Erdmann
/// @author  Daniel Krajzewicz
/// @author  Tino Morenz
/// @author  Axel Wegener
/// @author  Michael Behrisch
/// @author  Christoph Sommer
/// @author  Mario Krumnow
/// @author  Leonhard Luecken
/// @author  Mirko Barthauer
/// @date    Mon, 05 Mar 2001
///
// Representation of a lane in the micro simulation
/****************************************************************************/
#include <config.h>

#include <cmath>
#include <bitset>
#include <iostream>
#include <cassert>
#include <functional>
#include <algorithm>
#include <iterator>
#include <exception>
#include <climits>
#include <set>
#include <utils/common/UtilExceptions.h>
#include <utils/common/StdDefs.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/ToString.h>
#ifdef HAVE_FOX
#include <utils/common/ScopedLocker.h>
#endif
#include <utils/options/OptionsCont.h>
#include <utils/emissions/HelpersHarmonoise.h>
#include <utils/geom/GeomHelper.h>
#include <libsumo/TraCIConstants.h>
#include <microsim/transportables/MSPModel.h>
#include <microsim/transportables/MSTransportableControl.h>
#include <microsim/traffic_lights/MSRailSignal.h>
#include <microsim/traffic_lights/MSRailSignalControl.h>
#include <microsim/traffic_lights/MSDriveWay.h>
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <microsim/devices/MSDevice_Taxi.h>
#include <mesosim/MELoop.h>
#include "MSNet.h"
#include "MSVehicleType.h"
#include "MSEdge.h"
#include "MSEdgeControl.h"
#include "MSJunction.h"
#include "MSLogicJunction.h"
#include "MSLink.h"
#include "MSLane.h"
#include "MSVehicleTransfer.h"
#include "MSGlobals.h"
#include "MSVehicleControl.h"
#include "MSInsertionControl.h"
#include "MSVehicleControl.h"
#include "MSLeaderInfo.h"
#include "MSVehicle.h"
#include "MSStop.h"

//#define DEBUG_INSERTION
//#define DEBUG_PLAN_MOVE
//#define DEBUG_EXEC_MOVE
//#define DEBUG_CONTEXT
//#define DEBUG_PARTIALS
//#define DEBUG_OPPOSITE
//#define DEBUG_VEHICLE_CONTAINER
//#define DEBUG_COLLISIONS
//#define DEBUG_JUNCTION_COLLISIONS
//#define DEBUG_PEDESTRIAN_COLLISIONS
//#define DEBUG_LANE_SORTER
//#define DEBUG_NO_CONNECTION
//#define DEBUG_SURROUNDING
//#define DEBUG_EXTRAPOLATE_DEPARTPOS
//#define DEBUG_ITERATOR

//#define DEBUG_COND (false)
//#define DEBUG_COND (true)
//#define DEBUG_COND (getID() == "undefined")
#define DEBUG_COND (isSelected())
//#define DEBUG_COND2(obj) ((obj != 0 && (obj)->getID() == "disabled"))
#define DEBUG_COND2(obj) ((obj != nullptr && (obj)->isSelected()))
//#define DEBUG_COND (getID() == "ego")
//#define DEBUG_COND2(obj) ((obj != 0 && (obj)->getID() == "ego"))
//#define DEBUG_COND2(obj) (true)


// ===========================================================================
// static member definitions
// ===========================================================================
MSLane::DictType MSLane::myDict;
MSLane::CollisionAction MSLane::myCollisionAction(MSLane::COLLISION_ACTION_TELEPORT);
MSLane::CollisionAction MSLane::myIntermodalCollisionAction(MSLane::COLLISION_ACTION_WARN);
bool MSLane::myCheckJunctionCollisions(false);
double MSLane::myCheckJunctionCollisionMinGap(0);
SUMOTime MSLane::myCollisionStopTime(0);
SUMOTime MSLane::myIntermodalCollisionStopTime(0);
double MSLane::myCollisionMinGapFactor(1.0);
bool MSLane::myExtrapolateSubstepDepart(false);
std::vector<SumoRNG> MSLane::myRNGs;


// ===========================================================================
// internal class method definitions
// ===========================================================================
void
MSLane::StoringVisitor::add(const MSLane* const l) const {
    switch (myDomain) {
        case libsumo::CMD_GET_VEHICLE_VARIABLE: {
            for (const MSVehicle* veh : l->getVehiclesSecure()) {
                if (myShape.distance2D(veh->getPosition()) <= myRange) {
                    myObjects.insert(veh);
                }
            }
            for (const MSBaseVehicle* veh : l->getParkingVehicles()) {
                if (myShape.distance2D(veh->getPosition()) <= myRange) {
                    myObjects.insert(veh);
                }
            }
            l->releaseVehicles();
        }
        break;
        case libsumo::CMD_GET_PERSON_VARIABLE: {
            l->getVehiclesSecure();
            std::vector<MSTransportable*> persons = l->getEdge().getSortedPersons(MSNet::getInstance()->getCurrentTimeStep(), true);
            for (auto p : persons) {
                if (myShape.distance2D(p->getPosition()) <= myRange) {
                    myObjects.insert(p);
                }
            }
            l->releaseVehicles();
        }
        break;
        case libsumo::CMD_GET_EDGE_VARIABLE: {
            if (myShape.size() != 1 || l->getShape().distance2D(myShape[0]) <= myRange) {
                myObjects.insert(&l->getEdge());
            }
        }
        break;
        case libsumo::CMD_GET_LANE_VARIABLE: {
            if (myShape.size() != 1 || l->getShape().distance2D(myShape[0]) <= myRange) {
                myObjects.insert(l);
            }
        }
        break;
        default:
            break;

    }
}


MSLane::AnyVehicleIterator&
MSLane::AnyVehicleIterator::operator++() {
    if (nextIsMyVehicles()) {
        if (myI1 != myI1End) {
            myI1 += myDirection;
        } else if (myI3 != myI3End) {
            myI3 += myDirection;
        }
        // else: already at end
    } else {
        myI2 += myDirection;
    }
    //if (DEBUG_COND2(myLane)) std::cout << SIMTIME << "          AnyVehicleIterator::operator++ lane=" << myLane->getID() << " myI1=" << myI1 << " myI2=" << myI2 << "\n";
    return *this;
}


const MSVehicle*
MSLane::AnyVehicleIterator::operator*() {
    if (nextIsMyVehicles()) {
        if (myI1 != myI1End) {
            return myLane->myVehicles[myI1];
        } else if (myI3 != myI3End) {
            return myLane->myTmpVehicles[myI3];
        } else {
            assert(myI2 == myI2End);
            return nullptr;
        }
    } else {
        return myLane->myPartialVehicles[myI2];
    }
}


bool
MSLane::AnyVehicleIterator::nextIsMyVehicles() const {
#ifdef DEBUG_ITERATOR
    if (DEBUG_COND2(myLane)) std::cout << SIMTIME << "          AnyVehicleIterator::nextIsMyVehicles lane=" << myLane->getID()
                                           << " myI1=" << myI1
                                           << " myI1End=" << myI1End
                                           << " myI2=" << myI2
                                           << " myI2End=" << myI2End
                                           << " myI3=" << myI3
                                           << " myI3End=" << myI3End
                                           << "\n";
#endif
    if (myI1 == myI1End && myI3 == myI3End) {
        if (myI2 != myI2End) {
            return false;
        } else {
            return true; // @note. must be caught
        }
    } else {
        if (myI2 == myI2End) {
            return true;
        } else {
            MSVehicle* cand = myI1 == myI1End ? myLane->myTmpVehicles[myI3] : myLane->myVehicles[myI1];
#ifdef DEBUG_ITERATOR
            if (DEBUG_COND2(myLane)) std::cout << "              "
                                                   << " veh1=" << cand->getID()
                                                   << " isTmp=" << (myI1 == myI1End)
                                                   << " veh2=" << myLane->myPartialVehicles[myI2]->getID()
                                                   << " pos1=" << cand->getPositionOnLane(myLane)
                                                   << " pos2=" << myLane->myPartialVehicles[myI2]->getPositionOnLane(myLane)
                                                   << "\n";
#endif
            if (cand->getPositionOnLane() < myLane->myPartialVehicles[myI2]->getPositionOnLane(myLane)) {
                return myDownstream;
            } else {
                return !myDownstream;
            }
        }
    }
}


// ===========================================================================
// member method definitions
// ===========================================================================
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4355) // mask warning about "this" in initializers
#endif
MSLane::MSLane(const std::string& id, double maxSpeed, double friction, double length, MSEdge* const edge,
               int numericalID, const PositionVector& shape, double width,
               SVCPermissions permissions,
               SVCPermissions changeLeft, SVCPermissions changeRight,
               int index, bool isRampAccel,
               const std::string& type,
               const PositionVector& outlineShape) :
    Named(id),
    myNumericalID(numericalID), myShape(shape), myIndex(index),
    myVehicles(), myLength(length), myWidth(width),
    myEdge(edge), myMaxSpeed(maxSpeed),
    myFrictionCoefficient(friction),
    mySpeedByVSS(false),
    mySpeedByTraCI(false),
    myPermissions(permissions),
    myChangeLeft(changeLeft),
    myChangeRight(changeRight),
    myOriginalPermissions(permissions),
    myLogicalPredecessorLane(nullptr),
    myCanonicalPredecessorLane(nullptr),
    myCanonicalSuccessorLane(nullptr),
    myBruttoVehicleLengthSum(0), myNettoVehicleLengthSum(0),
    myBruttoVehicleLengthSumToRemove(0), myNettoVehicleLengthSumToRemove(0),
    myRecalculateBruttoSum(false),
    myLeaderInfo(width, nullptr, 0.),
    myFollowerInfo(width, nullptr, 0.),
    myLeaderInfoTime(SUMOTime_MIN),
    myFollowerInfoTime(SUMOTime_MIN),
    myLengthGeometryFactor(MAX2(POSITION_EPS, myShape.length()) / myLength), // factor should not be 0
    myIsRampAccel(isRampAccel),
    myLaneType(type),
    myRightSideOnEdge(0), // initialized in MSEdge::initialize
    myRightmostSublane(0),
    myNeedsCollisionCheck(false),
    myOpposite(nullptr),
    myBidiLane(nullptr),
#ifdef HAVE_FOX
    mySimulationTask(*this, 0),
#endif
    myStopWatch(3) {
    // initialized in MSEdge::initialize
    initRestrictions();// may be reloaded again from initialized in MSEdge::closeBuilding
    assert(myRNGs.size() > 0);
    myRNGIndex = numericalID % myRNGs.size();
    if (outlineShape.size() > 0) {
        myOutlineShape = new PositionVector(outlineShape);
    }
}
#ifdef _MSC_VER
#pragma warning(pop)
#endif


MSLane::~MSLane() {
    for (MSLink* const l : myLinks) {
        delete l;
    }
    delete myOutlineShape;
}


void
MSLane::initRestrictions() {
    // simplify unit testing without MSNet instance
    myRestrictions = MSGlobals::gUnitTests ? nullptr : MSNet::getInstance()->getRestrictions(myEdge->getEdgeType());
}


void
MSLane::checkBufferType() {
    if (MSGlobals::gNumSimThreads <= 1) {
        myVehBuffer.unsetCondition();
//    } else {
// this is an idea for better memory locality, lanes with nearby numerical ids get the same rng and thus the same thread
// first tests show no visible effect though
//        myRNGIndex = myNumericalID * myRNGs.size() / dictSize();
    }
}


void
MSLane::addLink(MSLink* link) {
    myLinks.push_back(link);
}


void
MSLane::setOpposite(MSLane* oppositeLane) {
    myOpposite = oppositeLane;
    if (myOpposite != nullptr && getLength() > myOpposite->getLength()) {
        WRITE_WARNINGF(TL("Unequal lengths of neigh lane '%' and lane '%' (% != %)."), getID(), myOpposite->getID(), getLength(), myOpposite->getLength());
    }
}

void
MSLane::setBidiLane(MSLane* bidiLane) {
    myBidiLane = bidiLane;
    if (myBidiLane != nullptr && getLength() > myBidiLane->getLength()) {
        if (isNormal() || MSGlobals::gUsingInternalLanes) {
            WRITE_WARNINGF(TL("Unequal lengths of bidi lane '%' and lane '%' (% != %)."), getID(), myBidiLane->getID(), getLength(), myBidiLane->getLength());
        }
    }
}



// ------ interaction with MSMoveReminder ------
void
MSLane::addMoveReminder(MSMoveReminder* rem, bool addToVehicles) {
    myMoveReminders.push_back(rem);
    if (addToVehicles) {
        for (MSVehicle* const veh : myVehicles) {
            veh->addReminder(rem);
        }
    }
    // XXX: Here, the partial occupators are ignored!? Refs. #3255
}


void
MSLane::removeMoveReminder(MSMoveReminder* rem) {
    auto it = std::find(myMoveReminders.begin(), myMoveReminders.end(), rem);
    if (it != myMoveReminders.end()) {
        myMoveReminders.erase(it);
        for (MSVehicle* const veh : myVehicles) {
            veh->removeReminder(rem);
        }
    }
}


double
MSLane::setPartialOccupation(MSVehicle* v) {
    // multithreading: there are concurrent writes to myNeedsCollisionCheck but all of them should set it to true
    myNeedsCollisionCheck = true; // always check
#ifdef DEBUG_PARTIALS
    if (DEBUG_COND2(v)) {
        std::cout << SIMTIME << " setPartialOccupation. lane=" << getID() << " veh=" << v->getID() << "\n";
    }
#endif
    // XXX update occupancy here?
#ifdef HAVE_FOX
    ScopedLocker<> lock(myPartialOccupatorMutex, MSGlobals::gNumSimThreads > 1);
#endif
    //assert(std::find(myPartialVehicles.begin(), myPartialVehicles.end(), v) == myPartialVehicles.end());
    myPartialVehicles.push_back(v);
    return myLength;
}


void
MSLane::resetPartialOccupation(MSVehicle* v) {
#ifdef HAVE_FOX
    ScopedLocker<> lock(myPartialOccupatorMutex, MSGlobals::gNumSimThreads > 1);
#endif
#ifdef DEBUG_PARTIALS
    if (DEBUG_COND2(v)) {
        std::cout << SIMTIME << " resetPartialOccupation. lane=" << getID() << " veh=" << v->getID() << "\n";
    }
#endif
    for (VehCont::iterator i = myPartialVehicles.begin(); i != myPartialVehicles.end(); ++i) {
        if (v == *i) {
            myPartialVehicles.erase(i);
            // XXX update occupancy here?
            //std::cout << "    removed from myPartialVehicles\n";
            return;
        }
    }
    // bluelight eqipped vehicle can teleport onto the intersection without using a connection
    assert(false || MSGlobals::gClearState || v->getLaneChangeModel().hasBlueLight());
}


void
MSLane::setManeuverReservation(MSVehicle* v) {
#ifdef DEBUG_CONTEXT
    if (DEBUG_COND2(v)) {
        std::cout << SIMTIME << " setManeuverReservation. lane=" << getID() << " veh=" << v->getID() << "\n";
    }
#endif
    myManeuverReservations.push_back(v);
}


void
MSLane::resetManeuverReservation(MSVehicle* v) {
#ifdef DEBUG_CONTEXT
    if (DEBUG_COND2(v)) {
        std::cout << SIMTIME << " resetManeuverReservation(): lane=" << getID() << " veh=" << v->getID() << "\n";
    }
#endif
    for (VehCont::iterator i = myManeuverReservations.begin(); i != myManeuverReservations.end(); ++i) {
        if (v == *i) {
            myManeuverReservations.erase(i);
            return;
        }
    }
    assert(false);
}


// ------ Vehicle emission ------
void
MSLane::incorporateVehicle(MSVehicle* veh, double pos, double speed, double posLat, const MSLane::VehCont::iterator& at, MSMoveReminder::Notification notification) {
    myNeedsCollisionCheck = true;
    assert(pos <= myLength);
    bool wasInactive = myVehicles.size() == 0;
    veh->enterLaneAtInsertion(this, pos, speed, posLat, notification);
    if (at == myVehicles.end()) {
        // vehicle will be the first on the lane
        myVehicles.push_back(veh);
    } else {
        myVehicles.insert(at, veh);
    }
    myBruttoVehicleLengthSum += veh->getVehicleType().getLengthWithGap();
    myNettoVehicleLengthSum += veh->getVehicleType().getLength();
    myEdge->markDelayed();
    if (wasInactive) {
        MSNet::getInstance()->getEdgeControl().gotActive(this);
    }
    if (getBidiLane() != nullptr && (!isRailway(veh->getVClass()) || (getPermissions() & ~SVC_RAIL_CLASSES) != 0)) {
        // railways don't need to "see" each other when moving in opposite directions on the same track (efficiency)
        getBidiLane()->setPartialOccupation(veh);
    }
}


bool
MSLane::lastInsertion(MSVehicle& veh, double mspeed, double posLat, bool patchSpeed) {
    double pos = getLength() - POSITION_EPS;
    MSVehicle* leader = getLastAnyVehicle();
    // back position of leader relative to this lane
    double leaderBack;
    if (leader == nullptr) {
        /// look for a leaders on consecutive lanes
        veh.setTentativeLaneAndPosition(this, pos, posLat);
        veh.updateBestLanes(false, this);
        std::pair<MSVehicle* const, double> leaderInfo = getLeader(&veh, pos, veh.getBestLanesContinuation(), veh.getCarFollowModel().brakeGap(mspeed));
        leader = leaderInfo.first;
        leaderBack = pos + leaderInfo.second + veh.getVehicleType().getMinGap();
    } else {
        leaderBack = leader->getBackPositionOnLane(this);
        //std::cout << " leaderPos=" << leader->getPositionOnLane(this) << " leaderBack=" << leader->getBackPositionOnLane(this) << " leaderLane=" << leader->getLane()->getID() << "\n";
    }
    if (leader == nullptr) {
        // insert at the end of this lane
        return isInsertionSuccess(&veh, mspeed, pos, posLat, patchSpeed, MSMoveReminder::NOTIFICATION_DEPARTED);
    } else {
        // try to insert behind the leader
        const double frontGapNeeded = veh.getCarFollowModel().getSecureGap(&veh, leader, mspeed, leader->getSpeed(), leader->getCarFollowModel().getMaxDecel()) + veh.getVehicleType().getMinGap() + POSITION_EPS;
        if (leaderBack >= frontGapNeeded) {
            pos = MIN2(pos, leaderBack - frontGapNeeded);
            bool result = isInsertionSuccess(&veh, mspeed, pos, posLat, patchSpeed, MSMoveReminder::NOTIFICATION_DEPARTED);
            //if (!result) std::cout << " insertLast failed for " << veh.getID() << " pos=" << pos << " leaderBack=" << leaderBack << " frontGapNeeded=" << frontGapNeeded << "\n";
            return result;
        }
        //std::cout << " insertLast failed for " << veh.getID() << " pos=" << pos << " leaderBack=" << leaderBack << " frontGapNeeded=" << frontGapNeeded << "\n";
    }
    return false;
}


bool
MSLane::freeInsertion(MSVehicle& veh, double mspeed, double posLat,
                      MSMoveReminder::Notification notification) {
    // try to insert teleporting vehicles fully on this lane
    double maxPos = myLength;
    if (veh.hasStops() && veh.getNextStop().edge == veh.getCurrentRouteEdge()) {
        maxPos = MAX2(0.0, veh.getNextStop().getEndPos(veh));
    }
    const double minPos = (notification == MSMoveReminder::NOTIFICATION_TELEPORT ?
                           MIN2(maxPos, veh.getVehicleType().getLength()) : 0);
    veh.setTentativeLaneAndPosition(this, minPos, 0);
    if (myVehicles.size() == 0) {
        // ensure sufficient gap to followers on predecessor lanes
        const double backOffset = minPos - veh.getVehicleType().getLength();
        const double missingRearGap = getMissingRearGap(&veh, backOffset, mspeed);
        if (missingRearGap > 0) {
            if (minPos + missingRearGap <= maxPos) {
                // @note. The rear gap is tailored to mspeed. If it changes due
                // to a leader vehicle (on subsequent lanes) insertion will
                // still fail. Under the right combination of acceleration and
                // deceleration values there might be another insertion
                // positions that would be successful be we do not look for it.
                //std::cout << SIMTIME << " freeInsertion lane=" << getID() << " veh=" << veh.getID() << " unclear @(340)\n";
                return isInsertionSuccess(&veh, mspeed, minPos + missingRearGap, posLat, true, notification);
            }
            return false;
        } else {
            return isInsertionSuccess(&veh, mspeed, minPos, posLat, true, notification);
        }

    } else {
        // check whether the vehicle can be put behind the last one if there is such
        const MSVehicle* const leader = myVehicles.back(); // @todo reproduction of bogus old behavior. see #1961
        const double leaderPos = leader->getBackPositionOnLane(this);
        const double speed = leader->getSpeed();
        const double frontGapNeeded = veh.getCarFollowModel().getSecureGap(&veh, leader, speed, leader->getSpeed(), leader->getCarFollowModel().getMaxDecel()) + veh.getVehicleType().getMinGap();
        if (leaderPos >= frontGapNeeded) {
            const double tspeed = MIN2(veh.getCarFollowModel().insertionFollowSpeed(&veh, mspeed, frontGapNeeded, leader->getSpeed(), leader->getCarFollowModel().getMaxDecel(), leader), mspeed);
            // check whether we can insert our vehicle behind the last vehicle on the lane
            if (isInsertionSuccess(&veh, tspeed, minPos, posLat, true, notification)) {
                //std::cout << SIMTIME << " freeInsertion lane=" << getID() << " veh=" << veh.getID() << " pos=" << minPos<< " speed=" << speed  << " tspeed=" << tspeed << " frontGapNeeded=" << frontGapNeeded << " lead=" << leader->getID() << " lPos=" << leaderPos << "\n   vehsOnLane=" << toString(myVehicles) << " @(358)\n";
                return true;
            }
        }
    }
    // go through the lane, look for free positions (starting after the last vehicle)
    MSLane::VehCont::iterator predIt = myVehicles.begin();
    while (predIt != myVehicles.end()) {
        // get leader (may be zero) and follower
        // @todo compute secure position in regard to sublane-model
        const MSVehicle* leader = predIt != myVehicles.end() - 1 ? *(predIt + 1) : nullptr;
        if (leader == nullptr && myPartialVehicles.size() > 0) {
            leader = myPartialVehicles.front();
        }
        const MSVehicle* follower = *predIt;

        // patch speed if allowed
        double speed = mspeed;
        if (leader != nullptr) {
            speed = MIN2(leader->getSpeed(), mspeed);
        }

        // compute the space needed to not collide with leader
        double frontMax = maxPos;
        if (leader != nullptr) {
            double leaderRearPos = leader->getBackPositionOnLane(this);
            double frontGapNeeded = veh.getCarFollowModel().getSecureGap(&veh, leader, speed, leader->getSpeed(), leader->getCarFollowModel().getMaxDecel()) + veh.getVehicleType().getMinGap();
            frontMax = MIN2(maxPos, leaderRearPos - frontGapNeeded);
        }
        // compute the space needed to not let the follower collide
        const double followPos = follower->getPositionOnLane() + follower->getVehicleType().getMinGap();
        const double backGapNeeded = follower->getCarFollowModel().getSecureGap(follower, &veh, follower->getSpeed(), veh.getSpeed(), veh.getCarFollowModel().getMaxDecel());
        const double backMin = followPos + backGapNeeded + veh.getVehicleType().getLength();

        // check whether there is enough room (given some extra space for rounding errors)
        if (frontMax > minPos && backMin + POSITION_EPS < frontMax) {
            // try to insert vehicle (should be always ok)
            if (isInsertionSuccess(&veh, speed, backMin + POSITION_EPS, posLat, true, notification)) {
                //std::cout << SIMTIME << " freeInsertion lane=" << getID() << " veh=" << veh.getID() << " @(393)\n";
                return true;
            }
        }
        ++predIt;
    }
    // first check at lane's begin
    //std::cout << SIMTIME << " freeInsertion lane=" << getID() << " veh=" << veh.getID() << " fail final\n";
    return false;
}


double
MSLane::getDepartSpeed(const MSVehicle& veh, bool& patchSpeed) {
    double speed = 0;
    const SUMOVehicleParameter& pars = veh.getParameter();
    switch (pars.departSpeedProcedure) {
        case DepartSpeedDefinition::GIVEN:
            speed = pars.departSpeed;
            patchSpeed = false;
            break;
        case DepartSpeedDefinition::RANDOM:
            speed = roundDecimal(RandHelper::rand(getVehicleMaxSpeed(&veh)), gPrecisionRandom);
            patchSpeed = true;
            break;
        case DepartSpeedDefinition::MAX:
            speed = getVehicleMaxSpeed(&veh);
            patchSpeed = true;
            break;
        case DepartSpeedDefinition::DESIRED:
            speed = getVehicleMaxSpeed(&veh);
            patchSpeed = false;
            break;
        case DepartSpeedDefinition::LIMIT:
            speed = getVehicleMaxSpeed(&veh) / veh.getChosenSpeedFactor();
            patchSpeed = false;
            break;
        case DepartSpeedDefinition::LAST: {
            MSVehicle* last = getLastAnyVehicle();
            speed = getVehicleMaxSpeed(&veh);
            if (last != nullptr) {
                speed = MIN2(speed, last->getSpeed());
                patchSpeed = false;
            }
            break;
        }
        case DepartSpeedDefinition::AVG: {
            speed = MIN2(getVehicleMaxSpeed(&veh), getMeanSpeed());
            if (getLastAnyVehicle() != nullptr) {
                patchSpeed = false;
            }
            break;
        }
        case DepartSpeedDefinition::DEFAULT:
        default:
            // speed = 0 was set before
            patchSpeed = false; // @todo check
            break;
    }
    return speed;
}


double
MSLane::getDepartPosLat(const MSVehicle& veh) {
    const SUMOVehicleParameter& pars = veh.getParameter();
    switch (pars.departPosLatProcedure) {
        case DepartPosLatDefinition::GIVEN:
            return pars.departPosLat;
        case DepartPosLatDefinition::RIGHT:
            return -getWidth() * 0.5 + veh.getVehicleType().getWidth() * 0.5;
        case DepartPosLatDefinition::LEFT:
            return getWidth() * 0.5 - veh.getVehicleType().getWidth() * 0.5;
        case DepartPosLatDefinition::RANDOM: {
            const double raw = RandHelper::rand(getWidth() - veh.getVehicleType().getWidth()) - getWidth() * 0.5 + veh.getVehicleType().getWidth() * 0.5;
            return roundDecimal(raw, gPrecisionRandom);
        }
        case DepartPosLatDefinition::CENTER:
        case DepartPosLatDefinition::DEFAULT:
        // @note:
        // case DepartPosLatDefinition::FREE
        // case DepartPosLatDefinition::RANDOM_FREE
        // are not handled here because they involve multiple insertion attempts
        default:
            return 0;
    }
}


bool
MSLane::insertVehicle(MSVehicle& veh) {
    double pos = 0;
    bool patchSpeed = true; // whether the speed shall be adapted to infrastructure/traffic in front
    const SUMOVehicleParameter& pars = veh.getParameter();
    double speed = getDepartSpeed(veh, patchSpeed);
    double posLat = getDepartPosLat(veh);

    // determine the position
    switch (pars.departPosProcedure) {
        case DepartPosDefinition::GIVEN:
            pos = pars.departPos;
            if (pos < 0.) {
                pos += myLength;
            }
            break;
        case DepartPosDefinition::RANDOM:
            pos = roundDecimal(RandHelper::rand(getLength()), gPrecisionRandom);
            break;
        case DepartPosDefinition::RANDOM_FREE: {
            for (int i = 0; i < 10; i++) {
                // we will try some random positions ...
                pos = RandHelper::rand(getLength());
                posLat = getDepartPosLat(veh); // could be random as well
                if (isInsertionSuccess(&veh, speed, pos, posLat, patchSpeed, MSMoveReminder::NOTIFICATION_DEPARTED)) {
                    MSNet::getInstance()->getInsertionControl().retractDescheduleDeparture(&veh);
                    return true;
                }
            }
            // ... and if that doesn't work, we put the vehicle to the free position
            bool success = freeInsertion(veh, speed, posLat);
            if (success) {
                MSNet::getInstance()->getInsertionControl().retractDescheduleDeparture(&veh);
            }
            return success;
        }
        case DepartPosDefinition::FREE:
            return freeInsertion(veh, speed, posLat);
        case DepartPosDefinition::LAST:
            return lastInsertion(veh, speed, posLat, patchSpeed);
        case DepartPosDefinition::STOP:
            if (veh.hasStops() && veh.getNextStop().edge == veh.getCurrentRouteEdge()) {
                // getLastFreePos of stopping place could return negative position to avoid blocking the stop
                pos = MAX2(0.0, veh.getNextStop().getEndPos(veh));
                break;
            }
            FALLTHROUGH;
        case DepartPosDefinition::BASE:
        case DepartPosDefinition::DEFAULT:
        case DepartPosDefinition::SPLIT_FRONT:
        default:
            if (pars.departProcedure == DepartDefinition::SPLIT) {
                pos = getLength();
                // find the vehicle from which we are splitting off (should only be a single lane to check)
                AnyVehicleIterator end = anyVehiclesEnd();
                for (AnyVehicleIterator it = anyVehiclesBegin(); it != end; ++it) {
                    const MSVehicle* cand = *it;
                    if (cand->isStopped() && cand->getNextStopParameter()->split == veh.getID()) {
                        if (pars.departPosProcedure == DepartPosDefinition::SPLIT_FRONT) {
                            pos = cand->getPositionOnLane() + cand->getVehicleType().getMinGap() + veh.getLength();
                        } else {
                            pos = cand->getBackPositionOnLane() - veh.getVehicleType().getMinGap();
                        }
                        break;
                    }
                }
            } else {
                pos = veh.basePos(myEdge);
            }
            break;
    }
    // determine the lateral position for special cases
    if (MSGlobals::gLateralResolution > 0) {
        switch (pars.departPosLatProcedure) {
            case DepartPosLatDefinition::RANDOM_FREE: {
                for (int i = 0; i < 10; i++) {
                    // we will try some random positions ...
                    posLat = RandHelper::rand(getWidth()) - getWidth() * 0.5;
                    if (isInsertionSuccess(&veh, speed, pos, posLat, patchSpeed, MSMoveReminder::NOTIFICATION_DEPARTED)) {
                        return true;
                    }
                }
                FALLTHROUGH;
            }
            // no break! continue with DepartPosLatDefinition::FREE
            case DepartPosLatDefinition::FREE: {
                // systematically test all positions until a free lateral position is found
                double posLatMin = -getWidth() * 0.5 + veh.getVehicleType().getWidth() * 0.5;
                double posLatMax = getWidth() * 0.5 - veh.getVehicleType().getWidth() * 0.5;
                for (posLat = posLatMin; posLat < posLatMax; posLat += MSGlobals::gLateralResolution) {
                    if (isInsertionSuccess(&veh, speed, pos, posLat, patchSpeed, MSMoveReminder::NOTIFICATION_DEPARTED)) {
                        return true;
                    }
                }
                return false;
            }
            default:
                break;
        }
    }
    // try to insert
    const bool success = isInsertionSuccess(&veh, speed, pos, posLat, patchSpeed, MSMoveReminder::NOTIFICATION_DEPARTED);
#ifdef DEBUG_EXTRAPOLATE_DEPARTPOS
    if (DEBUG_COND2(&veh)) {
        std::cout << SIMTIME << " veh=" << veh.getID() << " success=" << success << " extrapolate=" << myExtrapolateSubstepDepart << " delay=" << veh.getDepartDelay() << " speed=" << speed << "\n";
    }
#endif
    if (success && myExtrapolateSubstepDepart && veh.getDepartDelay() > 0) {
        SUMOTime relevantDelay = MIN2(DELTA_T, veh.getDepartDelay());
        // try to compensate sub-step depart delay by moving the vehicle forward
        speed = veh.getSpeed(); // may have been adapted in isInsertionSuccess
        double dist = speed * STEPS2TIME(relevantDelay);
        std::pair<MSVehicle* const, double> leaderInfo = getLeader(&veh, pos, veh.getBestLanesContinuation());
        if (leaderInfo.first != nullptr) {
            MSVehicle* leader = leaderInfo.first;
            const double frontGapNeeded = veh.getCarFollowModel().getSecureGap(&veh, leader, speed, leader->getSpeed(),
                                          leader->getCarFollowModel().getMaxDecel());
            dist = MIN2(dist, leaderInfo.second - frontGapNeeded);
        }
        if (dist > 0) {
            veh.executeFractionalMove(dist);
        }
    }
    return success;
}


bool
MSLane::checkFailure(const MSVehicle* aVehicle, double& speed, double& dist, const double nspeed, const bool patchSpeed, const std::string errorMsg, InsertionCheck check) const {
    if (nspeed < speed) {
        if (patchSpeed) {
            speed = MIN2(nspeed, speed);
            dist = aVehicle->getCarFollowModel().brakeGap(speed) + aVehicle->getVehicleType().getMinGap();
        } else if (speed > 0) {
            if ((aVehicle->getInsertionChecks() & (int)check) == 0) {
                return false;
            }
            if (MSGlobals::gEmergencyInsert) {
                // Check whether vehicle can stop at the given distance when applying emergency braking
                double emergencyBrakeGap = 0.5 * speed * speed / aVehicle->getCarFollowModel().getEmergencyDecel();
                if (emergencyBrakeGap <= dist) {
                    // Vehicle may stop in time with emergency deceleration
                    // stil, emit a warning
                    WRITE_WARNINGF(TL("Vehicle '%' is inserted in emergency situation."), aVehicle->getID());
                    return false;
                }
            }

            if (errorMsg != "") {
                WRITE_ERRORF(TL("Vehicle '%' will not be able to depart using the given velocity (%)!"), aVehicle->getID(), errorMsg);
                MSNet::getInstance()->getInsertionControl().descheduleDeparture(aVehicle);
            }
            return true;
        }
    }
    return false;
}


bool
MSLane::isInsertionSuccess(MSVehicle* aVehicle,
                           double speed, double pos, double posLat, bool patchSpeed,
                           MSMoveReminder::Notification notification) {
    int insertionChecks = aVehicle->getInsertionChecks();
    if (pos < 0 || pos > myLength) {
        // we may not start there
        WRITE_WARNINGF(TL("Invalid departPos % given for vehicle '%'. Inserting at lane end instead."),
                       pos, aVehicle->getID());
        pos = myLength;
    }

#ifdef DEBUG_INSERTION
    if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
        std::cout << "\nIS_INSERTION_SUCCESS\n"
                  << SIMTIME  << " lane=" << getID()
                  << " veh '" << aVehicle->getID()
                  << " bestLanes=" << toString(aVehicle->getBestLanesContinuation(this))
                  << " pos=" << pos
                  << " speed=" << speed
                  << " patchSpeed=" << patchSpeed
                  << "'\n";
    }
#endif

    aVehicle->setTentativeLaneAndPosition(this, pos, posLat);
    aVehicle->updateBestLanes(false, this);
    const MSCFModel& cfModel = aVehicle->getCarFollowModel();
    const std::vector<MSLane*>& bestLaneConts = aVehicle->getBestLanesContinuation(this);
    std::vector<MSLane*>::const_iterator ri = bestLaneConts.begin();
    double seen = getLength() - pos; // == distance from insertion position until the end of the currentLane
    double dist = cfModel.brakeGap(speed) + aVehicle->getVehicleType().getMinGap();
    const bool isRail = aVehicle->isRail();
    if (isRail && insertionChecks != (int)InsertionCheck::NONE
            && aVehicle->getParameter().departProcedure != DepartDefinition::SPLIT
            && MSRailSignalControl::isSignalized(aVehicle->getVClass())) {
        const MSDriveWay* dw = MSDriveWay::getDepartureDriveway(aVehicle);
        MSEdgeVector occupied;
        if (dw->foeDriveWayOccupied(false, aVehicle, occupied)) {
            setParameter("insertionBlocked:" + aVehicle->getID(), dw->getID());
#ifdef DEBUG_INSERTION
            if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
                std::cout << "   foe of driveway " + dw->getID() + " has occupied edges " + toString(occupied) << "\n";
            }
#endif
            return false;
        }
    }
    // do not insert if the bidirectional edge is occupied
    if (getBidiLane() != nullptr && isRail && getBidiLane()->getVehicleNumberWithPartials() > 0) {
        if ((insertionChecks & (int)InsertionCheck::BIDI) != 0) {
#ifdef DEBUG_INSERTION
            if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
                std::cout << "   bidi-lane occupied\n";
            }
#endif
            return false;
        }
    }
    MSLink* firstRailSignal = nullptr;
    double firstRailSignalDist = -1;

    // before looping through the continuation lanes, check if a stop is scheduled on this lane
    // (the code is duplicated in the loop)
    if (aVehicle->hasStops()) {
        const MSStop& nextStop = aVehicle->getNextStop();
        if (nextStop.lane == this) {
            std::stringstream msg;
            double distToStop, safeSpeed;
            if (nextStop.pars.speed > 0) {
                msg << "scheduled waypoint on lane '" << myID << "' too close";
                distToStop = MAX2(0.0, nextStop.pars.startPos - pos);
                safeSpeed = cfModel.freeSpeed(aVehicle, speed, distToStop, nextStop.pars.speed, true, MSCFModel::CalcReason::FUTURE);
            } else {
                msg << "scheduled stop on lane '" << myID << "' too close";
                distToStop = nextStop.pars.endPos - pos;
                safeSpeed = cfModel.stopSpeed(aVehicle, speed, distToStop, MSCFModel::CalcReason::FUTURE);
            }
            if (checkFailure(aVehicle, speed, dist, MAX2(0.0, safeSpeed), patchSpeed, msg.str(), InsertionCheck::STOP)) {
                // we may not drive with the given velocity - we cannot stop at the stop
                return false;
            }
        }
    }

    const MSRoute& r = aVehicle->getRoute();
    MSRouteIterator ce = r.begin();
    int nRouteSuccs = 1;
    MSLane* currentLane = this;
    MSLane* nextLane = this;
    SUMOTime arrivalTime = MSNet::getInstance()->getCurrentTimeStep() + TIME2STEPS(seen / MAX2(speed, SUMO_const_haltingSpeed));
    while ((seen < dist || (isRail && firstRailSignal == nullptr)) && ri != bestLaneConts.end()) {
        // get the next link used...
        std::vector<MSLink*>::const_iterator link = succLinkSec(*aVehicle, nRouteSuccs, *currentLane, bestLaneConts);
        if (currentLane->isLinkEnd(link)) {
            if (&currentLane->getEdge() == r.getLastEdge()) {
                // reached the end of the route
                if (aVehicle->getParameter().arrivalSpeedProcedure == ArrivalSpeedDefinition::GIVEN) {
                    const double remaining = seen + aVehicle->getArrivalPos() - currentLane->getLength();
                    const double nspeed = cfModel.freeSpeed(aVehicle, speed, remaining, aVehicle->getParameter().arrivalSpeed, true, MSCFModel::CalcReason::FUTURE);
                    if (checkFailure(aVehicle, speed, dist, nspeed,
                                     patchSpeed, "arrival speed too low", InsertionCheck::ARRIVAL_SPEED)) {
                        // we may not drive with the given velocity - we cannot match the specified arrival speed
                        return false;
                    }
                }
            } else {
                // lane does not continue
                if (checkFailure(aVehicle, speed, dist, cfModel.insertionStopSpeed(aVehicle, speed, seen),
                                 patchSpeed, "junction '" + currentLane->getEdge().getToJunction()->getID() + "' too close", InsertionCheck::JUNCTION)) {
                    // we may not drive with the given velocity - we cannot stop at the junction
                    return false;
                }
            }
            break;
        }
        if (isRail && firstRailSignal == nullptr) {
            std::string constraintInfo;
            bool isInsertionOrder;
            if (MSRailSignal::hasInsertionConstraint(*link, aVehicle, constraintInfo, isInsertionOrder)) {
                setParameter((isInsertionOrder ? "insertionOrder" : "insertionConstraint:")
                             + aVehicle->getID(), constraintInfo);
#ifdef DEBUG_INSERTION
                if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
                    std::cout << " insertion constraint at link " << (*link)->getDescription() << " not cleared \n";
                }
#endif
                return false;
            }
        }

        // might also by a regular traffic_light instead of a rail_signal
        if (firstRailSignal == nullptr && (*link)->getTLLogic() != nullptr) {
            firstRailSignal = *link;
            firstRailSignalDist = seen;
        }
        if (!(*link)->opened(arrivalTime, speed, speed, aVehicle->getVehicleType().getLength(), aVehicle->getImpatience(),
                             cfModel.getMaxDecel(), 0, posLat, nullptr, false, aVehicle)
                || !(*link)->havePriority()) {
            // have to stop at junction
            std::string errorMsg = "";
            const LinkState state = (*link)->getState();
            if (state == LINKSTATE_MINOR
                    || state == LINKSTATE_EQUAL
                    || state == LINKSTATE_STOP
                    || state == LINKSTATE_ALLWAY_STOP) {
                // no sense in trying later
                errorMsg = "unpriorised junction too close";
            } else if ((*link)->getTLLogic() != nullptr && !(*link)->getTLLogic()->getsMajorGreen((*link)->getTLIndex())) {
                // traffic light never turns 'G'?
                errorMsg = "tlLogic '" + (*link)->getTLLogic()->getID() + "' link " + toString((*link)->getTLIndex()) + " never switches to 'G'";
            }
            const double laneStopOffset = MAX2(getVehicleStopOffset(aVehicle),
                                               aVehicle->getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_STOPLINE_CROSSING_GAP, MSPModel::SAFETY_GAP) - (*link)->getDistToFoePedCrossing());
            const double remaining = seen - laneStopOffset;
            auto dsp = aVehicle->getParameter().departSpeedProcedure;
            const bool patchSpeedSpecial = patchSpeed || dsp == DepartSpeedDefinition::DESIRED || dsp == DepartSpeedDefinition::LIMIT;
            // patchSpeed depends on the presence of vehicles for these procedures. We never want to abort them here
            if (dsp == DepartSpeedDefinition::LAST || dsp == DepartSpeedDefinition::AVG) {
                errorMsg = "";
            }
            if (checkFailure(aVehicle, speed, dist, cfModel.insertionStopSpeed(aVehicle, speed, remaining),
                             patchSpeedSpecial, errorMsg, InsertionCheck::JUNCTION)) {
                // we may not drive with the given velocity - we cannot stop at the junction in time
#ifdef DEBUG_INSERTION
                if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
                    std::cout << SIMTIME << " isInsertionSuccess lane=" << getID()
                              << " veh=" << aVehicle->getID()
                              << " patchSpeed=" << patchSpeed
                              << " speed=" << speed
                              << " remaining=" << remaining
                              << " leader=" << currentLane->getLastVehicleInformation(aVehicle, 0, 0).toString()
                              << " last=" << Named::getIDSecure(getLastAnyVehicle())
                              << " meanSpeed=" << getMeanSpeed()
                              << " failed (@926)!\n";
                }
#endif
                return false;
            }
#ifdef DEBUG_INSERTION
            if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
                std::cout << "trying insertion before minor link: "
                          << "insertion speed = " << speed << " dist=" << dist
                          << "\n";
            }
#endif
            break;
        } else if (nextLane->isInternal()) {
            double tmp = 0;
            bool dummyReq = true;
#ifdef DEBUG_INSERTION
            if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
                std::cout << "checking linkLeader for lane '" << nextLane->getID() << "'\n";
                gDebugFlag1 = true;
            }
#endif
            aVehicle->checkLinkLeader(*link, nextLane, seen, nullptr, speed, tmp, tmp, dummyReq);
#ifdef DEBUG_INSERTION
            gDebugFlag1 = false;
#endif
        }
        // get the next used lane (including internal)
        nextLane = (*link)->getViaLaneOrLane();
        // check how next lane affects the journey
        if (nextLane != nullptr) {

            // do not insert if the bidirectional edge is occupied before a railSignal has been encountered
            if (firstRailSignal == nullptr && nextLane->getBidiLane() != nullptr && nextLane->getBidiLane()->getVehicleNumberWithPartials() > 0) {
                if ((insertionChecks & (int)InsertionCheck::ONCOMING_TRAIN) != 0) {
#ifdef DEBUG_INSERTION
                    if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
                        std::cout << " nextLane=" << nextLane->getID() << " occupiedBidi\n";
                    }
#endif
                    return false;
                }
            }

            // check if there are stops on the next lane that should be regarded
            // (this block is duplicated before the loop to deal with the insertion lane)
            if (aVehicle->hasStops()) {
                const MSStop& nextStop = aVehicle->getNextStop();
                if (nextStop.lane == nextLane) {
                    std::stringstream msg;
                    msg << "scheduled stop on lane '" << nextStop.lane->getID() << "' too close";
                    const double distToStop = seen + nextStop.pars.endPos;
                    if (checkFailure(aVehicle, speed, dist, cfModel.insertionStopSpeed(aVehicle, speed, distToStop),
                                     patchSpeed, msg.str(), InsertionCheck::STOP)) {
                        // we may not drive with the given velocity - we cannot stop at the stop
                        return false;
                    }
                }
            }

            // check leader on next lane
            MSLeaderInfo leaders = nextLane->getLastVehicleInformation(aVehicle, 0, 0);
            if (leaders.hasVehicles()) {
                const double nspeed = nextLane->safeInsertionSpeed(aVehicle, seen, leaders, speed);
#ifdef DEBUG_INSERTION
                if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
                    std::cout << SIMTIME << " leader on lane '" << nextLane->getID() << "': " << leaders.toString() << " nspeed=" << nspeed << "\n";
                }
#endif
                if (nspeed == INVALID_SPEED || checkFailure(aVehicle, speed, dist, nspeed, patchSpeed, "", InsertionCheck::LEADER_GAP)) {
                    // we may not drive with the given velocity - we crash into the leader
#ifdef DEBUG_INSERTION
                    if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
                        std::cout << " isInsertionSuccess lane=" << getID()
                                  << " veh=" << aVehicle->getID()
                                  << " pos=" << pos
                                  << " posLat=" << posLat
                                  << " patchSpeed=" << patchSpeed
                                  << " speed=" << speed
                                  << " nspeed=" << nspeed
                                  << " nextLane=" << nextLane->getID()
                                  << " lead=" << leaders.toString()
                                  << " failed (@641)!\n";
                    }
#endif
                    return false;
                }
            }
            if (!nextLane->checkForPedestrians(aVehicle, speed, dist, -seen, patchSpeed)) {
                return false;
            }
            // check next lane's maximum velocity
            const double nspeed = cfModel.freeSpeed(aVehicle, speed, seen, nextLane->getVehicleMaxSpeed(aVehicle), true, MSCFModel::CalcReason::FUTURE);
            if (nspeed < speed) {
                if (patchSpeed || aVehicle->getParameter().departSpeedProcedure != DepartSpeedDefinition::GIVEN) {
                    speed = nspeed;
                    dist = cfModel.brakeGap(speed) + aVehicle->getVehicleType().getMinGap();
                } else {
                    if ((insertionChecks & (int)InsertionCheck::SPEED_LIMIT) != 0) {
                        if (!MSGlobals::gCheckRoutes) {
                            WRITE_WARNINGF(TL("Vehicle '%' is inserted too fast and will violate the speed limit on a lane '%'."),
                                           aVehicle->getID(), nextLane->getID());
                        } else {
                            // we may not drive with the given velocity - we would be too fast on the next lane
                            WRITE_ERRORF(TL("Vehicle '%' will not be able to depart using the given velocity (slow lane ahead)!"), aVehicle->getID());
                            MSNet::getInstance()->getInsertionControl().descheduleDeparture(aVehicle);
                            return false;
                        }
                    }
                }
            }
            // check traffic on next junction
            // we cannot use (*link)->opened because a vehicle without priority
            // may already be comitted to blocking the link and unable to stop
            const SUMOTime leaveTime = (*link)->getLeaveTime(arrivalTime, speed, speed, aVehicle->getVehicleType().getLength());
            if ((*link)->hasApproachingFoe(arrivalTime, leaveTime, speed, cfModel.getMaxDecel())) {
                if (checkFailure(aVehicle, speed, dist, cfModel.insertionStopSpeed(aVehicle, speed, seen), patchSpeed, "", InsertionCheck::JUNCTION)) {
                    // we may not drive with the given velocity - we crash at the junction
                    return false;
                }
            }
            arrivalTime += TIME2STEPS(nextLane->getLength() / MAX2(speed, NUMERICAL_EPS));
            seen += nextLane->getLength();
            currentLane = nextLane;
            if ((*link)->getViaLane() == nullptr) {
                nRouteSuccs++;
                ++ce;
                ++ri;
            }
        }
    }

    // get the pointer to the vehicle next in front of the given position
    MSLeaderInfo leaders = getLastVehicleInformation(aVehicle, 0, pos);
    //if (aVehicle->getID() == "disabled") std::cout << " leaders=" << leaders.toString() << "\n";
    const double nspeed = safeInsertionSpeed(aVehicle, -pos, leaders, speed);
    if (nspeed == INVALID_SPEED || checkFailure(aVehicle, speed, dist, nspeed, patchSpeed, "", InsertionCheck::LEADER_GAP)) {
        // we may not drive with the given velocity - we crash into the leader
#ifdef DEBUG_INSERTION
        if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
            std::cout << SIMTIME << " isInsertionSuccess lane=" << getID()
                      << " veh=" << aVehicle->getID()
                      << " pos=" << pos
                      << " posLat=" << posLat
                      << " patchSpeed=" << patchSpeed
                      << " speed=" << speed
                      << " nspeed=" << nspeed
                      << " nextLane=" << nextLane->getID()
                      << " leaders=" << leaders.toString()
                      << " failed (@700)!\n";
        }
#endif
        return false;
    }
#ifdef DEBUG_INSERTION
    if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
        std::cout << SIMTIME << " speed = " << speed << " nspeed = " << nspeed << std::endl;
    }
#endif

    const MSLeaderDistanceInfo& followers = getFollowersOnConsecutive(aVehicle, aVehicle->getBackPositionOnLane(), false);
    for (int i = 0; i < followers.numSublanes(); ++i) {
        const MSVehicle* follower = followers[i].first;
        if (follower != nullptr) {
            const double backGapNeeded = follower->getCarFollowModel().getSecureGap(follower, aVehicle, follower->getSpeed(), speed, cfModel.getMaxDecel());
            if (followers[i].second < backGapNeeded
                    && ((insertionChecks & (int)InsertionCheck::FOLLOWER_GAP) != 0
                        || (followers[i].second < 0 && (insertionChecks & (int)InsertionCheck::COLLISION) != 0))) {
                // too close to the follower on this lane
#ifdef DEBUG_INSERTION
                if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
                    std::cout << SIMTIME << " isInsertionSuccess lane=" << getID()
                              << " veh=" << aVehicle->getID()
                              << " pos=" << pos
                              << " posLat=" << posLat
                              << " speed=" << speed
                              << " nspeed=" << nspeed
                              << " follower=" << follower->getID()
                              << " backGapNeeded=" << backGapNeeded
                              << " gap=" << followers[i].second
                              << " failure (@719)!\n";
                }
#endif
                return false;
            }
        }
    }

    if (!checkForPedestrians(aVehicle, speed, dist, pos, patchSpeed)) {
        return false;
    }

    MSLane* shadowLane = aVehicle->getLaneChangeModel().getShadowLane(this);
#ifdef DEBUG_INSERTION
    if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
        std::cout << "    shadowLane=" << Named::getIDSecure(shadowLane) << "\n";
    }
#endif
    if (shadowLane != nullptr) {
        const MSLeaderDistanceInfo& shadowFollowers = shadowLane->getFollowersOnConsecutive(aVehicle, aVehicle->getBackPositionOnLane(), false);
        for (int i = 0; i < shadowFollowers.numSublanes(); ++i) {
            const MSVehicle* follower = shadowFollowers[i].first;
            if (follower != nullptr) {
                const double backGapNeeded = follower->getCarFollowModel().getSecureGap(follower, aVehicle, follower->getSpeed(), speed, cfModel.getMaxDecel());
                if (shadowFollowers[i].second < backGapNeeded
                        && ((insertionChecks & (int)InsertionCheck::FOLLOWER_GAP) != 0
                            || (shadowFollowers[i].second < 0 && (insertionChecks & (int)InsertionCheck::COLLISION) != 0))) {
                    // too close to the follower on this lane
#ifdef DEBUG_INSERTION
                    if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
                        std::cout << SIMTIME
                                  << " isInsertionSuccess shadowlane=" << shadowLane->getID()
                                  << " veh=" << aVehicle->getID()
                                  << " pos=" << pos
                                  << " posLat=" << posLat
                                  << " speed=" << speed
                                  << " nspeed=" << nspeed
                                  << " follower=" << follower->getID()
                                  << " backGapNeeded=" << backGapNeeded
                                  << " gap=" << shadowFollowers[i].second
                                  << " failure (@812)!\n";
                    }
#endif
                    return false;
                }
            }
        }
        const MSLeaderInfo& ahead = shadowLane->getLastVehicleInformation(nullptr, 0, aVehicle->getPositionOnLane(), false);
        for (int i = 0; i < ahead.numSublanes(); ++i) {
            const MSVehicle* veh = ahead[i];
            if (veh != nullptr) {
                const double gap = veh->getBackPositionOnLane(shadowLane) - aVehicle->getPositionOnLane() - aVehicle->getVehicleType().getMinGap();
                const double gapNeeded = aVehicle->getCarFollowModel().getSecureGap(aVehicle, veh, speed, veh->getSpeed(), veh->getCarFollowModel().getMaxDecel());
                if (gap <  gapNeeded
                        && ((insertionChecks & (int)InsertionCheck::LEADER_GAP) != 0
                            || (gap < 0 && (insertionChecks & (int)InsertionCheck::COLLISION) != 0))) {
                    // too close to the shadow leader
#ifdef DEBUG_INSERTION
                    if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
                        std::cout << SIMTIME
                                  << " isInsertionSuccess shadowlane=" << shadowLane->getID()
                                  << " veh=" << aVehicle->getID()
                                  << " pos=" << pos
                                  << " posLat=" << posLat
                                  << " speed=" << speed
                                  << " nspeed=" << nspeed
                                  << " leader=" << veh->getID()
                                  << " gapNeeded=" << gapNeeded
                                  << " gap=" << gap
                                  << " failure (@842)!\n";
                    }
#endif
                    return false;
                }
            }
        }
    }
    if (followers.numFreeSublanes() > 0) {
        // check approaching vehicles to prevent rear-end collisions
        const double backOffset = pos - aVehicle->getVehicleType().getLength();
        const double missingRearGap = getMissingRearGap(aVehicle, backOffset, speed);
        if (missingRearGap > 0
                && (insertionChecks & (int)InsertionCheck::LEADER_GAP) != 0) {
            // too close to a follower
#ifdef DEBUG_INSERTION
            if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
                std::cout << SIMTIME
                          << " isInsertionSuccess lane=" << getID()
                          << " veh=" << aVehicle->getID()
                          << " pos=" << pos
                          << " posLat=" << posLat
                          << " speed=" << speed
                          << " nspeed=" << nspeed
                          << " missingRearGap=" << missingRearGap
                          << " failure (@728)!\n";
            }
#endif
            return false;
        }
    }
    if (insertionChecks == (int)InsertionCheck::NONE) {
        speed = MAX2(0.0, speed);
    }
    // may got negative while adaptation
    if (speed < 0) {
#ifdef DEBUG_INSERTION
        if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
            std::cout << SIMTIME
                      << " isInsertionSuccess lane=" << getID()
                      << " veh=" << aVehicle->getID()
                      << " pos=" << pos
                      << " posLat=" << posLat
                      << " speed=" << speed
                      << " nspeed=" << nspeed
                      << " failed (@733)!\n";
        }
#endif
        return false;
    }
    const int bestLaneOffset = aVehicle->getBestLaneOffset();
    const double extraReservation = aVehicle->getLaneChangeModel().getExtraReservation(bestLaneOffset);
    if (extraReservation > 0) {
        std::stringstream msg;
        msg << "too many lane changes required on lane '" << myID << "'";
        // we need to take into acount one extra actionStep of delay due to #3665
        double distToStop = aVehicle->getBestLaneDist() - pos - extraReservation - speed * aVehicle->getActionStepLengthSecs();
        if (distToStop >= 0) {
            double stopSpeed = cfModel.stopSpeed(aVehicle, speed, distToStop, MSCFModel::CalcReason::FUTURE);
#ifdef DEBUG_INSERTION
            if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
                std::cout << "\nIS_INSERTION_SUCCESS\n"
                          << SIMTIME << " veh=" << aVehicle->getID() << " bestLaneOffset=" << bestLaneOffset << " bestLaneDist=" << aVehicle->getBestLaneDist() << " extraReservation=" << extraReservation
                          << " distToStop=" << distToStop << " v=" << speed << " v2=" << stopSpeed << "\n";
            }
#endif
            if (checkFailure(aVehicle, speed, distToStop, MAX2(0.0, stopSpeed),
                             patchSpeed, msg.str(), InsertionCheck::LANECHANGE)) {
                // we may not drive with the given velocity - we cannot reserve enough space for lane changing
                return false;
            }
        }
    }
    // enter
    incorporateVehicle(aVehicle, pos, speed, posLat, find_if(myVehicles.begin(), myVehicles.end(), [&](MSVehicle * const v) {
        return v->getPositionOnLane() >= pos;
    }), notification);
#ifdef DEBUG_INSERTION
    if (DEBUG_COND2(aVehicle) || DEBUG_COND) {
        std::cout << SIMTIME
                  << " isInsertionSuccess lane=" << getID()
                  << " veh=" << aVehicle->getID()
                  << " pos=" << pos
                  << " posLat=" << posLat
                  << " speed=" << speed
                  << " nspeed=" << nspeed
                  << "\n myVehicles=" << toString(myVehicles)
                  << " myPartial=" << toString(myPartialVehicles)
                  << " myManeuverReservations=" << toString(myManeuverReservations)
                  << "\n leaders=" << leaders.toString()
                  << "\n success!\n";
    }
#endif
    if (isRail) {
        unsetParameter("insertionConstraint:" + aVehicle->getID());
        unsetParameter("insertionOrder:" + aVehicle->getID());
        unsetParameter("insertionBlocked:" + aVehicle->getID());
        // rail_signal (not traffic_light) requires approach information for
        // switching correctly at the start of the next simulation step
        if (firstRailSignal != nullptr && firstRailSignal->getJunction()->getType() == SumoXMLNodeType::RAIL_SIGNAL) {
            aVehicle->registerInsertionApproach(firstRailSignal, firstRailSignalDist);
        }
    }
    return true;
}


void
MSLane::forceVehicleInsertion(MSVehicle* veh, double pos, MSMoveReminder::Notification notification, double posLat) {
    veh->updateBestLanes(true, this);
    bool dummy;
    const double speed = veh->hasDeparted() ? veh->getSpeed() : getDepartSpeed(*veh, dummy);
    incorporateVehicle(veh, pos, speed, posLat, find_if(myVehicles.begin(), myVehicles.end(), [&](MSVehicle * const v) {
        return v->getPositionOnLane() >= pos;
    }), notification);
}


double
MSLane::safeInsertionSpeed(const MSVehicle* veh, double seen, const MSLeaderInfo& leaders, double speed) {
    double nspeed = speed;
#ifdef DEBUG_INSERTION
    if (DEBUG_COND2(veh)) {
        std::cout << SIMTIME << " safeInsertionSpeed veh=" << veh->getID() << " speed=" << speed << "\n";
    }
#endif
    for (int i = 0; i < leaders.numSublanes(); ++i) {
        const MSVehicle* leader = leaders[i];
        if (leader != nullptr) {
            double gap = leader->getBackPositionOnLane(this) + seen - veh->getVehicleType().getMinGap();
            if (leader->getLane() == getBidiLane()) {
                // use distance to front position and account for movement
                gap -= (leader->getLength() + leader->getBrakeGap(true));
            }
            if (gap < 0) {
#ifdef DEBUG_INSERTION
                if (DEBUG_COND2(veh)) {
                    std::cout << "    leader=" << leader->getID() << " bPos=" << leader->getBackPositionOnLane(this) << " gap=" << gap << "\n";
                }
#endif
                if ((veh->getInsertionChecks() & (int)InsertionCheck::COLLISION) != 0) {
                    return INVALID_SPEED;
                } else {
                    return 0;
                }
            }
            nspeed = MIN2(nspeed,
                          veh->getCarFollowModel().insertionFollowSpeed(veh, speed, gap, leader->getSpeed(), leader->getCarFollowModel().getMaxDecel(), leader));
#ifdef DEBUG_INSERTION
            if (DEBUG_COND2(veh)) {
                std::cout << "    leader=" << leader->getID() << " bPos=" << leader->getBackPositionOnLane(this) << " gap=" << gap << " nspeed=" << nspeed << "\n";
            }
#endif
        }
    }
    return nspeed;
}


// ------ Handling vehicles lapping into lanes ------
const MSLeaderInfo
MSLane::getLastVehicleInformation(const MSVehicle* ego, double latOffset, double minPos, bool allowCached) const {
    if (myLeaderInfoTime < MSNet::getInstance()->getCurrentTimeStep() || ego != nullptr || minPos > 0 || !allowCached) {
        MSLeaderInfo leaderTmp(myWidth, ego, latOffset);
        AnyVehicleIterator last = anyVehiclesBegin();
        int freeSublanes = 1; // number of sublanes for which no leader was found
        //if (ego->getID() == "disabled" && SIMTIME == 58) {
        //    std::cout << "DEBUG\n";
        //}
        const MSVehicle* veh = *last;
        while (freeSublanes > 0 && veh != nullptr) {
#ifdef DEBUG_PLAN_MOVE
            if (DEBUG_COND2(ego) || DEBUG_COND) {
                gDebugFlag1 = true;
                std::cout << "      getLastVehicleInformation lane=" << getID() << " minPos=" << minPos << " veh=" << veh->getID() << " pos=" << veh->getPositionOnLane(this)  << "\n";
            }
#endif
            if (veh != ego && MAX2(0.0, veh->getPositionOnLane(this)) >= minPos) {
                const double vehLatOffset = veh->getLatOffset(this);
                freeSublanes = leaderTmp.addLeader(veh, true, vehLatOffset);
#ifdef DEBUG_PLAN_MOVE
                if (DEBUG_COND2(ego) || DEBUG_COND) {
                    std::cout << "         latOffset=" << vehLatOffset << " newLeaders=" << leaderTmp.toString() << "\n";
                }
#endif
            }
            veh = *(++last);
        }
        if (ego == nullptr && minPos == 0) {
#ifdef HAVE_FOX
            ScopedLocker<> lock(myLeaderInfoMutex, MSGlobals::gNumSimThreads > 1);
#endif
            // update cached value
            myLeaderInfo = leaderTmp;
            myLeaderInfoTime = MSNet::getInstance()->getCurrentTimeStep();
        }
#ifdef DEBUG_PLAN_MOVE
        //if (DEBUG_COND2(ego)) std::cout << SIMTIME
        //    << " getLastVehicleInformation lane=" << getID()
        //        << " ego=" << Named::getIDSecure(ego)
        //        << "\n"
        //        << "    vehicles=" << toString(myVehicles)
        //        << "    partials=" << toString(myPartialVehicles)
        //        << "\n"
        //        << "    result=" << leaderTmp.toString()
        //        << "    cached=" << myLeaderInfo.toString()
        //        << "    myLeaderInfoTime=" << myLeaderInfoTime
        //        << "\n";
        gDebugFlag1 = false;
#endif
        return leaderTmp;
    }
    return myLeaderInfo;
}


const MSLeaderInfo
MSLane::getFirstVehicleInformation(const MSVehicle* ego, double latOffset, bool onlyFrontOnLane, double maxPos, bool allowCached) const {
#ifdef HAVE_FOX
    ScopedLocker<> lock(myFollowerInfoMutex, MSGlobals::gNumSimThreads > 1);
#endif
    if (myFollowerInfoTime < MSNet::getInstance()->getCurrentTimeStep() || ego != nullptr || maxPos < myLength || !allowCached || onlyFrontOnLane) {
        // XXX separate cache for onlyFrontOnLane = true
        MSLeaderInfo followerTmp(myWidth, ego, latOffset);
        AnyVehicleIterator first = anyVehiclesUpstreamBegin();
        int freeSublanes = 1; // number of sublanes for which no leader was found
        const MSVehicle* veh = *first;
        while (freeSublanes > 0 && veh != nullptr) {
#ifdef DEBUG_PLAN_MOVE
            if (DEBUG_COND2(ego)) {
                std::cout << "       veh=" << veh->getID() << " pos=" << veh->getPositionOnLane(this) << " maxPos=" << maxPos << "\n";
            }
#endif
            if (veh != ego && veh->getPositionOnLane(this) <= maxPos
                    && (!onlyFrontOnLane || veh->isFrontOnLane(this))) {
                //const double vehLatOffset = veh->getLane()->getRightSideOnEdge() - getRightSideOnEdge();
                const double vehLatOffset = veh->getLatOffset(this);
#ifdef DEBUG_PLAN_MOVE
                if (DEBUG_COND2(ego)) {
                    std::cout << "          veh=" << veh->getID() << " latOffset=" << vehLatOffset << "\n";
                }
#endif
                freeSublanes = followerTmp.addLeader(veh, true, vehLatOffset);
            }
            veh = *(++first);
        }
        if (ego == nullptr && maxPos == std::numeric_limits<double>::max()) {
            // update cached value
            myFollowerInfo = followerTmp;
            myFollowerInfoTime = MSNet::getInstance()->getCurrentTimeStep();
        }
#ifdef DEBUG_PLAN_MOVE
        //if (DEBUG_COND2(ego)) std::cout << SIMTIME
        //    << " getFirstVehicleInformation lane=" << getID()
        //        << " ego=" << Named::getIDSecure(ego)
        //        << "\n"
        //        << "    vehicles=" << toString(myVehicles)
        //        << "    partials=" << toString(myPartialVehicles)
        //        << "\n"
        //        << "    result=" << followerTmp.toString()
        //        //<< "    cached=" << myFollowerInfo.toString()
        //        << "    myLeaderInfoTime=" << myLeaderInfoTime
        //        << "\n";
#endif
        return followerTmp;
    }
    return myFollowerInfo;
}


// ------  ------
void
MSLane::planMovements(SUMOTime t) {
    assert(myVehicles.size() != 0);
    double cumulatedVehLength = 0.;
    MSLeaderInfo leaders(myWidth);

    // iterate over myVehicles, myPartialVehicles, and myManeuverReservations merge-sort style
    VehCont::reverse_iterator veh = myVehicles.rbegin();
    VehCont::reverse_iterator vehPart = myPartialVehicles.rbegin();
    VehCont::reverse_iterator vehRes = myManeuverReservations.rbegin();
#ifdef DEBUG_PLAN_MOVE
    if (DEBUG_COND) std::cout
                << "\n"
                << SIMTIME
                << " planMovements() lane=" << getID()
                << "\n    vehicles=" << toString(myVehicles)
                << "\n    partials=" << toString(myPartialVehicles)
                << "\n    reservations=" << toString(myManeuverReservations)
                << "\n";
#endif
    assert(MSGlobals::gLateralResolution || myManeuverReservations.size() == 0);
    for (; veh != myVehicles.rend(); ++veh) {
#ifdef DEBUG_PLAN_MOVE
        if (DEBUG_COND2((*veh))) {
            std::cout << "   plan move for: " << (*veh)->getID();
        }
#endif
        updateLeaderInfo(*veh, vehPart, vehRes, leaders); // 36ns with 8 threads, 9ns with 1
#ifdef DEBUG_PLAN_MOVE
        if (DEBUG_COND2((*veh))) {
            std::cout << " leaders=" << leaders.toString() << "\n";
        }
#endif
        (*veh)->planMove(t, leaders, cumulatedVehLength); // 4800ns with 8 threads, 3100 with 1
        cumulatedVehLength += (*veh)->getVehicleType().getLengthWithGap();
        leaders.addLeader(*veh, false, 0);
    }
}


void
MSLane::setJunctionApproaches() const {
    for (MSVehicle* const veh : myVehicles) {
        veh->setApproachingForAllLinks();
    }
}


void
MSLane::updateLeaderInfo(const MSVehicle* veh, VehCont::reverse_iterator& vehPart, VehCont::reverse_iterator& vehRes, MSLeaderInfo& ahead) const {
    bool morePartialVehsAhead = vehPart != myPartialVehicles.rend();
    bool moreReservationsAhead = vehRes != myManeuverReservations.rend();
    bool nextToConsiderIsPartial;

    // Determine relevant leaders for veh
    while (moreReservationsAhead || morePartialVehsAhead) {
        if ((!moreReservationsAhead || (*vehRes)->getPositionOnLane(this) <= veh->getPositionOnLane())
                && (!morePartialVehsAhead || (*vehPart)->getPositionOnLane(this) <= veh->getPositionOnLane())) {
            // All relevant downstream vehicles have been collected.
            break;
        }

        // Check whether next farthest relevant vehicle downstream is a partial vehicle or a maneuver reservation
        if (moreReservationsAhead && !morePartialVehsAhead) {
            nextToConsiderIsPartial = false;
        } else if (morePartialVehsAhead && !moreReservationsAhead) {
            nextToConsiderIsPartial = true;
        } else {
            assert(morePartialVehsAhead && moreReservationsAhead);
            // Add farthest downstream vehicle first
            nextToConsiderIsPartial = (*vehPart)->getPositionOnLane(this) > (*vehRes)->getPositionOnLane(this);
        }
        // Add appropriate leader information
        if (nextToConsiderIsPartial) {
            const double latOffset = (*vehPart)->getLatOffset(this);
#ifdef DEBUG_PLAN_MOVE
            if (DEBUG_COND) {
                std::cout << "        partial ahead: " << (*vehPart)->getID() << " latOffset=" << latOffset << "\n";
            }
#endif
            if (!(MSGlobals::gLaneChangeDuration > 0 && (*vehPart)->getLaneChangeModel().isOpposite()
                    && !(*vehPart)->getLaneChangeModel().isChangingLanes())) {
                ahead.addLeader(*vehPart, false, latOffset);
            }
            ++vehPart;
            morePartialVehsAhead = vehPart != myPartialVehicles.rend();
        } else {
            const double latOffset = (*vehRes)->getLatOffset(this);
#ifdef DEBUG_PLAN_MOVE
            if (DEBUG_COND) {
                std::cout << "    reservation ahead: " << (*vehRes)->getID() << " latOffset=" << latOffset << "\n";
            }
#endif
            ahead.addLeader(*vehRes, false, latOffset);
            ++vehRes;
            moreReservationsAhead = vehRes != myManeuverReservations.rend();
        }
    }
}


void
MSLane::detectCollisions(SUMOTime timestep, const std::string& stage) {
    myNeedsCollisionCheck = false;
#ifdef DEBUG_COLLISIONS
    if (DEBUG_COND) {
        std::vector<const MSVehicle*> all;
        for (AnyVehicleIterator last = anyVehiclesBegin(); last != anyVehiclesEnd(); ++last) {
            all.push_back(*last);
        }
        std::cout << SIMTIME << " detectCollisions stage=" << stage << " lane=" << getID() << ":\n"
                  << "   vehs=" << toString(myVehicles) << "\n"
                  << "   part=" << toString(myPartialVehicles) << "\n"
                  << "   all=" << toString(all) << "\n"
                  << "\n";
    }
#endif

    if (myCollisionAction == COLLISION_ACTION_NONE) {
        return;
    }

    std::set<const MSVehicle*, ComparatorNumericalIdLess> toRemove;
    std::set<const MSVehicle*, ComparatorNumericalIdLess> toTeleport;
    if (mustCheckJunctionCollisions()) {
        myNeedsCollisionCheck = true; // always check
#ifdef DEBUG_JUNCTION_COLLISIONS
        if (DEBUG_COND) {
            std::cout << SIMTIME << " detect junction Collisions stage=" << stage << " lane=" << getID() << ":\n"
                      << "   vehs=" << toString(myVehicles) << "\n"
                      << "   part=" << toString(myPartialVehicles) << "\n"
                      << "\n";
        }
#endif
        assert(myLinks.size() == 1);
        const std::vector<const MSLane*>& foeLanes = myLinks.front()->getFoeLanes();
        // save the iterator, it might get modified, see #8842
        MSLane::AnyVehicleIterator end = anyVehiclesEnd();
        for (AnyVehicleIterator veh = anyVehiclesBegin(); veh != end; ++veh) {
            const MSVehicle* const collider = *veh;
            //std::cout << "   collider " << collider->getID() << "\n";
            PositionVector colliderBoundary = collider->getBoundingBox(myCheckJunctionCollisionMinGap);
            for (const MSLane* const foeLane : foeLanes) {
#ifdef DEBUG_JUNCTION_COLLISIONS
                if (DEBUG_COND) {
                    std::cout << "     foeLane " << foeLane->getID()
                              << " foeVehs=" << toString(foeLane->myVehicles)
                              << " foePart=" << toString(foeLane->myPartialVehicles) << "\n";
                }
#endif
                MSLane::AnyVehicleIterator foeEnd = foeLane->anyVehiclesEnd();
                for (MSLane::AnyVehicleIterator it_veh = foeLane->anyVehiclesBegin(); it_veh != foeEnd; ++it_veh) {
                    const MSVehicle* const victim = *it_veh;
                    if (victim == collider) {
                        // may happen if the vehicles lane and shadow lane are siblings
                        continue;
                    }
#ifdef DEBUG_JUNCTION_COLLISIONS
                    if (DEBUG_COND && DEBUG_COND2(collider)) {
                        std::cout << SIMTIME << " foe=" << victim->getID()
                                  << " bound=" << colliderBoundary << " foeBound=" << victim->getBoundingBox()
                                  << " overlaps=" << colliderBoundary.overlapsWith(victim->getBoundingBox())
                                  << " poly=" << collider->getBoundingPoly()
                                  << " foePoly=" << victim->getBoundingPoly()
                                  << " overlaps2=" << collider->getBoundingPoly().overlapsWith(victim->getBoundingPoly())
                                  << "\n";
                    }
#endif
                    if (MSGlobals::gIgnoreJunctionBlocker < std::numeric_limits<SUMOTime>::max()) {
                        if (collider->getWaitingTime() >= MSGlobals::gIgnoreJunctionBlocker
                                || victim->getWaitingTime() >= MSGlobals::gIgnoreJunctionBlocker) {
                            // ignored vehicles should not tigger collision
                            continue;
                        }
                    }

                    if (colliderBoundary.overlapsWith(victim->getBoundingBox())) {
                        // make a detailed check
                        PositionVector boundingPoly = collider->getBoundingPoly();
                        if (collider->getBoundingPoly(myCheckJunctionCollisionMinGap).overlapsWith(victim->getBoundingPoly())) {
                            // junction leader is the victim (collider must still be on junction)
                            assert(isInternal());
                            if (victim->getLane()->isInternal() && victim->isLeader(myLinks.front(), collider, -1)) {
                                foeLane->handleCollisionBetween(timestep, stage, victim, collider, -1, 0, toRemove, toTeleport);
                            } else {
                                handleCollisionBetween(timestep, stage, collider, victim, -1, 0, toRemove, toTeleport);
                            }
                        }
                    }
                }
                detectPedestrianJunctionCollision(collider, colliderBoundary, foeLane, timestep, stage, toRemove, toTeleport);
            }
            if (myLinks.front()->getWalkingAreaFoe() != nullptr) {
                detectPedestrianJunctionCollision(collider, colliderBoundary, myLinks.front()->getWalkingAreaFoe(), timestep, stage, toRemove, toTeleport);
            }
            if (myLinks.front()->getWalkingAreaFoeExit() != nullptr) {
                detectPedestrianJunctionCollision(collider, colliderBoundary, myLinks.front()->getWalkingAreaFoeExit(), timestep, stage, toRemove, toTeleport);
            }
        }
    }


    if (myIntermodalCollisionAction != COLLISION_ACTION_NONE && myEdge->getPersons().size() > 0 && hasPedestrians()) {
#ifdef DEBUG_PEDESTRIAN_COLLISIONS
        if (DEBUG_COND) {
            std::cout << SIMTIME << " detect pedestrian collisions stage=" << stage << " lane=" << getID() << "\n";
        }
#endif
        AnyVehicleIterator v_end = anyVehiclesEnd();
        for (AnyVehicleIterator it_v = anyVehiclesBegin(); it_v != v_end; ++it_v) {
            const MSVehicle* v = *it_v;
            double back = v->getBackPositionOnLane(this);
            const double length = v->getVehicleType().getLength();
            const double right = v->getRightSideOnEdge(this) - getRightSideOnEdge();
            if (v->getLane() == getBidiLane()) {
                // use the front position for checking
                back -= length;
            }
            PersonDist leader = nextBlocking(back, right, right + v->getVehicleType().getWidth());
#ifdef DEBUG_PEDESTRIAN_COLLISIONS
            if (DEBUG_COND && DEBUG_COND2(v)) {
                std::cout << SIMTIME << " back=" << back << " right=" << right << " person=" << Named::getIDSecure(leader.first)
                          << " dist=" << leader.second << " jammed=" << (leader.first == nullptr ? false : leader.first->isJammed()) << "\n";
            }
#endif
            if (leader.first != 0 && leader.second < length && !leader.first->isJammed()) {
                if (v->getVehicleType().getGuiShape() == SUMOVehicleShape::AIRCRAFT) {
                    // aircraft wings and body are above walking level
                    continue;
                }
                const double gap = leader.second - length;
                handleIntermodalCollisionBetween(timestep, stage, v, leader.first, gap, "sharedLane", toRemove, toTeleport);
            }
        }
    }

    if (myVehicles.size() == 0) {
        return;
    }
    if (!MSGlobals::gSublane) {
        // no sublanes
        VehCont::reverse_iterator lastVeh = myVehicles.rend() - 1;
        for (VehCont::reverse_iterator pred = myVehicles.rbegin(); pred != lastVeh; ++pred) {
            VehCont::reverse_iterator veh = pred + 1;
            detectCollisionBetween(timestep, stage, *veh, *pred, toRemove, toTeleport);
        }
        if (myPartialVehicles.size() > 0) {
            detectCollisionBetween(timestep, stage, *lastVeh, myPartialVehicles.front(), toRemove, toTeleport);
        }
        if (getBidiLane() != nullptr) {
            // bidirectional railway
            MSLane* bidiLane = getBidiLane();
            if (bidiLane->getVehicleNumberWithPartials() > 0) {
                for (auto veh = myVehicles.begin(); veh != myVehicles.end(); ++veh) {
                    double high = (*veh)->getPositionOnLane(this);
                    double low = (*veh)->getBackPositionOnLane(this);
                    if (stage == MSNet::STAGE_MOVEMENTS) {
                        // use previous back position to catch trains that
                        // "jump" through each other
                        low -= SPEED2DIST((*veh)->getSpeed());
                    }
                    for (AnyVehicleIterator veh2 = bidiLane->anyVehiclesBegin(); veh2 != bidiLane->anyVehiclesEnd(); ++veh2) {
                        // self-collisions might legitemately occur when a long train loops back on itself
                        if (*veh == *veh2 && !(*veh)->isRail()) {
                            continue;
                        }
                        if ((*veh)->getLane() == (*veh2)->getLane() ||
                                (*veh)->getLane() == (*veh2)->getBackLane() ||
                                (*veh)->getBackLane() == (*veh2)->getLane()) {
                            // vehicles are not in a bidi relation
                            continue;
                        }
                        double low2 = myLength - (*veh2)->getPositionOnLane(bidiLane);
                        double high2 = myLength - (*veh2)->getBackPositionOnLane(bidiLane);
                        if (stage == MSNet::STAGE_MOVEMENTS) {
                            // use previous back position to catch trains that
                            // "jump" through each other
                            high2 += SPEED2DIST((*veh2)->getSpeed());
                        }
                        if (!(high < low2 || high2 < low)) {
#ifdef DEBUG_COLLISIONS
                            if (DEBUG_COND) {
                                std::cout << SIMTIME << " bidi-collision veh=" << (*veh)->getID() << " bidiVeh=" << (*veh2)->getID()
                                          << " vehFurther=" << toString((*veh)->getFurtherLanes())
                                          << " high=" << high << " low=" << low << " high2=" << high2 << " low2=" << low2 << "\n";
                            }
#endif
                            // the faster vehicle is at fault
                            MSVehicle* collider = const_cast<MSVehicle*>(*veh);
                            MSVehicle* victim = const_cast<MSVehicle*>(*veh2);
                            if (collider->getSpeed() < victim->getSpeed()) {
                                std::swap(victim, collider);
                            }
                            handleCollisionBetween(timestep, stage, collider, victim, -1, 0, toRemove, toTeleport);
                        }
                    }
                }
            }
        }
    } else {
        // in the sublane-case it is insufficient to check the vehicles ordered
        // by their front position as there might be more than 2 vehicles next to each
        // other on the same lane
        // instead, a moving-window approach is used where all vehicles that
        // overlap in the longitudinal direction receive pairwise checks
        // XXX for efficiency, all lanes of an edge should be checked together
        // (lanechanger-style)

        // XXX quick hack: check each in myVehicles against all others
        for (AnyVehicleIterator veh = anyVehiclesBegin(); veh != anyVehiclesEnd(); ++veh) {
            MSVehicle* follow = (MSVehicle*)*veh;
            for (AnyVehicleIterator veh2 = anyVehiclesBegin(); veh2 != anyVehiclesEnd(); ++veh2) {
                MSVehicle* lead = (MSVehicle*)*veh2;
                if (lead == follow) {
                    continue;
                }
                if (lead->getPositionOnLane(this) < follow->getPositionOnLane(this)) {
                    continue;
                }
                if (detectCollisionBetween(timestep, stage, follow, lead, toRemove, toTeleport)) {
                    // XXX what about collisions with multiple leaders at once?
                    break;
                }
            }
        }
    }


    for (std::set<const MSVehicle*, ComparatorNumericalIdLess>::iterator it = toRemove.begin(); it != toRemove.end(); ++it) {
        MSVehicle* veh = const_cast<MSVehicle*>(*it);
        MSLane* vehLane = veh->getMutableLane();
        vehLane->removeVehicle(veh, MSMoveReminder::NOTIFICATION_TELEPORT, false);
        if (toTeleport.count(veh) > 0) {
            MSVehicleTransfer::getInstance()->add(timestep, veh);
        } else {
            veh->onRemovalFromNet(MSMoveReminder::NOTIFICATION_VAPORIZED_COLLISION);
            MSNet::getInstance()->getVehicleControl().scheduleVehicleRemoval(veh);
        }
    }
}


void
MSLane::detectPedestrianJunctionCollision(const MSVehicle* collider, const PositionVector& colliderBoundary, const MSLane* foeLane,
        SUMOTime timestep, const std::string& stage,
        std::set<const MSVehicle*, ComparatorNumericalIdLess>& toRemove,
        std::set<const MSVehicle*, ComparatorNumericalIdLess>& toTeleport) {
    if (myIntermodalCollisionAction != COLLISION_ACTION_NONE && foeLane->getEdge().getPersons().size() > 0 && foeLane->hasPedestrians()) {
#ifdef DEBUG_PEDESTRIAN_COLLISIONS
        if (DEBUG_COND) {
            std::cout << SIMTIME << " detect pedestrian junction collisions stage=" << stage << " lane=" << getID() << " foeLane=" << foeLane->getID() << "\n";
        }
#endif
        const std::vector<MSTransportable*>& persons = foeLane->getEdge().getSortedPersons(timestep);
        for (std::vector<MSTransportable*>::const_iterator it_p = persons.begin(); it_p != persons.end(); ++it_p) {
#ifdef DEBUG_PEDESTRIAN_COLLISIONS
            if (DEBUG_COND) {
                std::cout << "    collider=" << collider->getID()
                          << " ped=" << (*it_p)->getID()
                          << " jammed=" << (*it_p)->isJammed()
                          << " colliderBoundary=" << colliderBoundary
                          << " pedBoundary=" << (*it_p)->getBoundingBox()
                          << "\n";
            }
#endif
            if ((*it_p)->isJammed()) {
                continue;
            }
            if (colliderBoundary.overlapsWith((*it_p)->getBoundingBox())
                    && collider->getBoundingPoly().overlapsWith((*it_p)->getBoundingBox())) {
                std::string collisionType = "junctionPedestrian";
                if (foeLane->getEdge().isCrossing()) {
                    collisionType = "crossing";
                } else if (foeLane->getEdge().isWalkingArea()) {
                    collisionType = "walkingarea";
                }
                handleIntermodalCollisionBetween(timestep, stage, collider, *it_p, 0, collisionType, toRemove, toTeleport);
            }
        }
    }
}


bool
MSLane::detectCollisionBetween(SUMOTime timestep, const std::string& stage, MSVehicle* collider, MSVehicle* victim,
                               std::set<const MSVehicle*, ComparatorNumericalIdLess>& toRemove,
                               std::set<const MSVehicle*, ComparatorNumericalIdLess>& toTeleport) const {
    if (myCollisionAction == COLLISION_ACTION_TELEPORT && ((victim->hasInfluencer() && victim->getInfluencer().isRemoteAffected(timestep)) ||
            (collider->hasInfluencer() && collider->getInfluencer().isRemoteAffected(timestep)))) {
        return false;
    }

    // No self-collisions! (This is assumed to be ensured at caller side)
    if (collider == victim) {
        return false;
    }

    const bool colliderOpposite = collider->getLaneChangeModel().isOpposite() || collider->isBidiOn(this);
    const bool victimOpposite = victim->getLaneChangeModel().isOpposite() || victim->isBidiOn(this);
    const bool bothOpposite = victimOpposite && colliderOpposite;
    if (bothOpposite) {
        std::swap(victim, collider);
    }
    const double colliderPos = colliderOpposite && !bothOpposite ? collider->getBackPositionOnLane(this) : collider->getPositionOnLane(this);
    const double minGapFactor = myCollisionMinGapFactor >= 0 ? myCollisionMinGapFactor : collider->getCarFollowModel().getCollisionMinGapFactor();
    double victimBack = victimOpposite && !bothOpposite ? victim->getPositionOnLane(this) : victim->getBackPositionOnLane(this);
    if (victim->getLateralOverlap() > 0 || collider->getLateralOverlap() > 0) {
        if (&collider->getLane()->getEdge() == myEdge && collider->getLane()->getLength() > getLength()) {
            // interpret victim position on the longer lane
            victimBack *= collider->getLane()->getLength() / getLength();
        }
    }
    double gap = victimBack - colliderPos - minGapFactor * collider->getVehicleType().getMinGap();
    if (bothOpposite) {
        gap = colliderPos - victimBack - minGapFactor * collider->getVehicleType().getMinGap();
    } else if (colliderOpposite) {
        // vehicles are back to back so (frontal) minGap doesn't apply
        gap += minGapFactor * collider->getVehicleType().getMinGap();
    }
#ifdef DEBUG_COLLISIONS
    if (DEBUG_COND && (DEBUG_COND2(collider) || DEBUG_COND2(victim))) {
        std::cout << SIMTIME
                  << " thisLane=" << getID()
                  << " collider=" << collider->getID()
                  << " victim=" << victim->getID()
                  << " colOpposite=" << colliderOpposite
                  << " vicOpposite=" << victimOpposite
                  << " colLane=" << collider->getLane()->getID()
                  << " vicLane=" << victim->getLane()->getID()
                  << " colPos=" << colliderPos
                  << " vicBack=" << victimBack
                  << " colLat=" << collider->getCenterOnEdge(this)
                  << " vicLat=" << victim->getCenterOnEdge(this)
                  << " minGap=" << collider->getVehicleType().getMinGap()
                  << " minGapFactor=" << minGapFactor
                  << " gap=" << gap
                  << "\n";
    }
#endif
    if (victimOpposite && gap < -(collider->getLength() + victim->getLength())) {
        // already past each other
        return false;
    }
    if (gap < -NUMERICAL_EPS) {
        double latGap = 0;
        if (MSGlobals::gSublane) {
            latGap = (fabs(victim->getCenterOnEdge(this) - collider->getCenterOnEdge(this))
                      - 0.5 * fabs(victim->getVehicleType().getWidth() + collider->getVehicleType().getWidth()));
            if (latGap + NUMERICAL_EPS > 0) {
                return false;
            }
            // account for ambiguous gap computation related to partial
            // occupation of lanes with different lengths
            if (isInternal() && getEdge().getNumLanes() > 1 && victim->getLane() != collider->getLane()) {
                double gapDelta = 0;
                const MSVehicle* otherLaneVeh = collider->getLane() == this ? victim : collider;
                if (otherLaneVeh->getLaneChangeModel().getShadowLane() == this) {
                    gapDelta = getLength() - otherLaneVeh->getLane()->getLength();
                } else {
                    for (const MSLane* cand : otherLaneVeh->getFurtherLanes()) {
                        if (&cand->getEdge() == &getEdge()) {
                            gapDelta = getLength() - cand->getLength();
                            break;
                        }
                    }
                }
                if (gap + gapDelta >= 0) {
                    return false;
                }
            }
        }
        if (MSGlobals::gLaneChangeDuration > DELTA_T
                && collider->getLaneChangeModel().isChangingLanes()
                && victim->getLaneChangeModel().isChangingLanes()
                && victim->getLane() != this) {
            // synchroneous lane change maneuver
            return false;
        }
#ifdef DEBUG_COLLISIONS
        if (DEBUG_COND && (DEBUG_COND2(collider) || DEBUG_COND2(victim))) {
            std::cout << SIMTIME << " detectedCollision gap=" << gap << " latGap=" << latGap << "\n";
        }
#endif
        handleCollisionBetween(timestep, stage, collider, victim, gap, latGap, toRemove, toTeleport);
        return true;
    }
    return false;
}


void
MSLane::handleCollisionBetween(SUMOTime timestep, const std::string& stage, const MSVehicle* collider, const MSVehicle* victim,
                               double gap, double latGap, std::set<const MSVehicle*, ComparatorNumericalIdLess>& toRemove,
                               std::set<const MSVehicle*, ComparatorNumericalIdLess>& toTeleport) const {
    if (collider->ignoreCollision() || victim->ignoreCollision()) {
        return;
    }
    std::string collisionType;
    std::string collisionText;
    if (isFrontalCollision(collider, victim)) {
        collisionType = "frontal";
        collisionText = TL("frontal collision");
    } else if (stage == MSNet::STAGE_LANECHANGE) {
        collisionType = "side";
        collisionText = TL("side collision");
    } else if (isInternal()) {
        collisionType = "junction";
        collisionText = TL("junction collision");
    } else {
        collisionType = "collision";
        collisionText = TL("collision");
    }

    // in frontal collisions the opposite vehicle is the collider
    if (victim->getLaneChangeModel().isOpposite() && !collider->getLaneChangeModel().isOpposite()) {
        std::swap(collider, victim);
    }
    std::string prefix = TLF("Vehicle '%'; % with vehicle '%", collider->getID(), collisionText, victim->getID());
    if (myCollisionStopTime > 0) {
        if (collider->collisionStopTime() >= 0 && victim->collisionStopTime() >= 0) {
            return;
        }
        std::string dummyError;
        SUMOVehicleParameter::Stop stop;
        stop.duration = myCollisionStopTime;
        stop.parametersSet |= STOP_DURATION_SET;
        const double collisionAngle = RAD2DEG(fabs(GeomHelper::angleDiff(victim->getAngle(), collider->getAngle())));
        // determine new speeds from collision angle (@todo account for vehicle mass)
        double victimSpeed = victim->getSpeed();
        double colliderSpeed = collider->getSpeed();
        // double victimOrigSpeed = victim->getSpeed();
        // double colliderOrigSpeed = collider->getSpeed();
        if (collisionAngle < 45) {
            // rear-end collisions
            colliderSpeed = MIN2(colliderSpeed, victimSpeed);
        } else if (collisionAngle < 135) {
            // side collision
            colliderSpeed /= 2;
            victimSpeed /= 2;
        } else {
            // frontal collision
            colliderSpeed = 0;
            victimSpeed = 0;
        }
        const double victimStopPos = MIN2(victim->getLane()->getLength(),
                                          victim->getPositionOnLane() + victim->getCarFollowModel().brakeGap(victimSpeed, victim->getCarFollowModel().getEmergencyDecel(), 0));
        if (victim->collisionStopTime() < 0) {
            stop.collision = true;
            stop.lane = victim->getLane()->getID();
            // @todo: push victim forward?
            stop.startPos = victimStopPos;
            stop.endPos = stop.startPos;
            stop.parametersSet |= STOP_START_SET | STOP_END_SET;
            ((MSBaseVehicle*)victim)->addStop(stop, dummyError, 0);
        }
        if (collider->collisionStopTime() < 0) {
            stop.collision = true;
            stop.lane = collider->getLane()->getID();
            stop.startPos = MIN2(collider->getPositionOnLane() + collider->getCarFollowModel().brakeGap(colliderSpeed, collider->getCarFollowModel().getEmergencyDecel(), 0),
                                 MAX3(0.0, victimStopPos - 0.75 * victim->getVehicleType().getLength(),
                                      collider->getPositionOnLane() - SPEED2DIST(collider->getSpeed())));
            stop.endPos = stop.startPos;
            stop.parametersSet |= STOP_START_SET | STOP_END_SET;
            ((MSBaseVehicle*)collider)->addStop(stop, dummyError, 0);
        }
        //std::cout << " collisionAngle=" << collisionAngle
        //    << "\n    vPos=" << victim->getPositionOnLane()   << " vStop=" << victimStopPos  << " vSpeed=" << victimOrigSpeed     << " vSpeed2=" << victimSpeed   << " vSpeed3=" << victim->getSpeed()
        //    << "\n    cPos=" << collider->getPositionOnLane() << " cStop=" << stop.startPos  << " cSpeed=" << colliderOrigSpeed   << " cSpeed2=" << colliderSpeed << " cSpeed3=" << collider->getSpeed()
        //    << "\n";
    } else {
        switch (myCollisionAction) {
            case COLLISION_ACTION_WARN:
                break;
            case COLLISION_ACTION_TELEPORT:
                prefix = TLF("Teleporting vehicle '%'; % with vehicle '%", collider->getID(), collisionText, victim->getID());
                toRemove.insert(collider);
                toTeleport.insert(collider);
                break;
            case COLLISION_ACTION_REMOVE: {
                prefix = TLF("Removing % participants: vehicle '%', vehicle '%", collisionText, collider->getID(), victim->getID());
                bool removeCollider = true;
                bool removeVictim = true;
                removeVictim = !(victim->hasInfluencer() && victim->getInfluencer()->isRemoteAffected(timestep));
                removeCollider = !(collider->hasInfluencer() && collider->getInfluencer()->isRemoteAffected(timestep));
                if (removeVictim) {
                    toRemove.insert(victim);
                }
                if (removeCollider) {
                    toRemove.insert(collider);
                }
                if (!removeVictim) {
                    if (!removeCollider) {
                        prefix = TLF("Keeping remote-controlled % participants: vehicle '%', vehicle '%", collisionText, collider->getID(), victim->getID());
                    } else {
                        prefix = TLF("Removing % participant: vehicle '%', keeping remote-controlled vehicle '%", collisionText, collider->getID(), victim->getID());
                    }
                } else if (!removeCollider) {
                    prefix = TLF("Keeping remote-controlled % participant: vehicle '%', removing vehicle '%", collisionText, collider->getID(), victim->getID());
                }
                break;
            }
            default:
                break;
        }
    }
    const bool newCollision = MSNet::getInstance()->registerCollision(collider, victim, collisionType, this, collider->getPositionOnLane(this));
    if (newCollision) {
        WRITE_WARNINGF(prefix + "', lane='%', gap=%%, time=%, stage=%.",
                       getID(), toString(gap), (MSGlobals::gSublane ? TL(", latGap=") + toString(latGap) : ""),
                       time2string(timestep), stage);
        MSNet::getInstance()->informVehicleStateListener(victim, MSNet::VehicleState::COLLISION);
        MSNet::getInstance()->informVehicleStateListener(collider, MSNet::VehicleState::COLLISION);
        MSNet::getInstance()->getVehicleControl().countCollision(myCollisionAction == COLLISION_ACTION_TELEPORT);
    }
#ifdef DEBUG_COLLISIONS
    if (DEBUG_COND2(collider)) {
        toRemove.erase(collider);
        toTeleport.erase(collider);
    }
    if (DEBUG_COND2(victim)) {
        toRemove.erase(victim);
        toTeleport.erase(victim);
    }
#endif
}


void
MSLane::handleIntermodalCollisionBetween(SUMOTime timestep, const std::string& stage, const MSVehicle* collider, const MSTransportable* victim,
        double gap, const std::string& collisionType,
        std::set<const MSVehicle*, ComparatorNumericalIdLess>& toRemove,
        std::set<const MSVehicle*, ComparatorNumericalIdLess>& toTeleport) const {
    if (collider->ignoreCollision()) {
        return;
    }
    std::string prefix = TLF("Vehicle '%'", collider->getID());
    if (myIntermodalCollisionStopTime > 0) {
        if (collider->collisionStopTime() >= 0) {
            return;
        }
        std::string dummyError;
        SUMOVehicleParameter::Stop stop;
        stop.duration = myIntermodalCollisionStopTime;
        stop.parametersSet |= STOP_DURATION_SET;
        // determine new speeds from collision angle (@todo account for vehicle mass)
        double colliderSpeed = collider->getSpeed();
        const double victimStopPos = victim->getEdgePos();
        // double victimOrigSpeed = victim->getSpeed();
        // double colliderOrigSpeed = collider->getSpeed();
        if (collider->collisionStopTime() < 0) {
            stop.collision = true;
            stop.lane = collider->getLane()->getID();
            stop.startPos = MIN2(collider->getPositionOnLane() + collider->getCarFollowModel().brakeGap(colliderSpeed, collider->getCarFollowModel().getEmergencyDecel(), 0),
                                 MAX3(0.0, victimStopPos - 0.75 * victim->getVehicleType().getLength(),
                                      collider->getPositionOnLane() - SPEED2DIST(collider->getSpeed())));
            stop.endPos = stop.startPos;
            stop.parametersSet |= STOP_START_SET | STOP_END_SET;
            ((MSBaseVehicle*)collider)->addStop(stop, dummyError, 0);
        }
    } else {
        switch (myIntermodalCollisionAction) {
            case COLLISION_ACTION_WARN:
                break;
            case COLLISION_ACTION_TELEPORT:
                prefix = TLF("Teleporting vehicle '%' after", collider->getID());
                toRemove.insert(collider);
                toTeleport.insert(collider);
                break;
            case COLLISION_ACTION_REMOVE: {
                prefix = TLF("Removing vehicle '%' after", collider->getID());
                bool removeCollider = true;
                removeCollider = !(collider->hasInfluencer() && collider->getInfluencer()->isRemoteAffected(timestep));
                if (!removeCollider) {
                    prefix = TLF("Keeping remote-controlled vehicle '%' after", collider->getID());
                } else {
                    toRemove.insert(collider);
                }
                break;
            }
            default:
                break;
        }
    }
    const bool newCollision = MSNet::getInstance()->registerCollision(collider, victim, collisionType, this, victim->getEdgePos());
    if (newCollision) {
        if (gap != 0) {
            WRITE_WARNING(prefix + TLF(" collision with person '%', lane='%', gap=%, time=%, stage=%.",
                                       victim->getID(), getID(), gap, time2string(timestep), stage));
        } else {
            WRITE_WARNING(prefix + TLF(" collision with person '%', lane='%', time=%, stage=%.",
                                       victim->getID(), getID(), time2string(timestep), stage));
        }
        MSNet::getInstance()->informVehicleStateListener(collider, MSNet::VehicleState::COLLISION);
        MSNet::getInstance()->getVehicleControl().countCollision(myIntermodalCollisionAction == COLLISION_ACTION_TELEPORT);
    }
#ifdef DEBUG_COLLISIONS
    if (DEBUG_COND2(collider)) {
        toRemove.erase(collider);
        toTeleport.erase(collider);
    }
#endif
}


bool
MSLane::isFrontalCollision(const MSVehicle* collider, const MSVehicle* victim) {
    if (collider->getLaneChangeModel().isOpposite() != victim->getLaneChangeModel().isOpposite()) {
        return true;
    } else {
        const MSEdge* victimBidi = victim->getLane()->getEdge().getBidiEdge();
        if (&collider->getLane()->getEdge() == victimBidi) {
            return true;
        } else {
            for (MSLane* further : collider->getFurtherLanes()) {
                if (&further->getEdge() == victimBidi) {
                    return true;
                }
            }
        }
    }
    return false;
}

void
MSLane::executeMovements(const SUMOTime t) {
    // multithreading: there are concurrent writes to myNeedsCollisionCheck but all of them should set it to true
    myNeedsCollisionCheck = true;
    MSLane* bidi = getBidiLane();
    if (bidi != nullptr && bidi->getVehicleNumber() == 0) {
        MSNet::getInstance()->getEdgeControl().checkCollisionForInactive(bidi);
    }
    MSVehicle* firstNotStopped = nullptr;
    // iterate over vehicles in reverse so that move reminders will be called in the correct order
    for (VehCont::reverse_iterator i = myVehicles.rbegin(); i != myVehicles.rend();) {
        MSVehicle* veh = *i;
        // length is needed later when the vehicle may not exist anymore
        const double length = veh->getVehicleType().getLengthWithGap();
        const double nettoLength = veh->getVehicleType().getLength();
        const bool moved = veh->executeMove();
        MSLane* const target = veh->getMutableLane();
        if (veh->hasArrived()) {
            // vehicle has reached its arrival position
#ifdef DEBUG_EXEC_MOVE
            if DEBUG_COND2(veh) {
                std::cout << SIMTIME << " veh " << veh->getID() << " has arrived." << std::endl;
            }
#endif
            veh->onRemovalFromNet(MSMoveReminder::NOTIFICATION_ARRIVED);
            MSNet::getInstance()->getVehicleControl().scheduleVehicleRemoval(veh);
        } else if (target != nullptr && moved) {
            if (target->getEdge().isVaporizing()) {
                // vehicle has reached a vaporizing edge
                veh->onRemovalFromNet(MSMoveReminder::NOTIFICATION_VAPORIZED_VAPORIZER);
                MSNet::getInstance()->getVehicleControl().scheduleVehicleRemoval(veh);
            } else {
                // vehicle has entered a new lane (leaveLane and workOnMoveReminders were already called in MSVehicle::executeMove)
                target->myVehBuffer.push_back(veh);
                MSNet::getInstance()->getEdgeControl().needsVehicleIntegration(target);
                if (MSGlobals::gSublane && veh->getLaneChangeModel().getShadowLane() != nullptr) {
                    // trigger sorting of partial vehicles as their order may have changed (lane might not be active and only contain partial vehicles)
                    MSNet::getInstance()->getEdgeControl().needsVehicleIntegration(veh->getLaneChangeModel().getShadowLane());
                }
            }
        } else if (veh->isParking()) {
            // vehicle started to park
            MSVehicleTransfer::getInstance()->add(t, veh);
            myParkingVehicles.insert(veh);
        } else if (veh->brokeDown()) {
            veh->resumeFromStopping();
            WRITE_WARNINGF(TL("Removing vehicle '%' after breaking down, lane='%', time=%."),
                           veh->getID(), veh->getLane()->getID(), time2string(t));
            veh->onRemovalFromNet(MSMoveReminder::NOTIFICATION_VAPORIZED_BREAKDOWN);
            MSNet::getInstance()->getVehicleControl().scheduleVehicleRemoval(veh);
        } else if (veh->isJumping()) {
            // vehicle jumps to next route edge
            MSVehicleTransfer::getInstance()->add(t, veh);
        } else if (veh->getPositionOnLane() > getLength()) {
            // for any reasons the vehicle is beyond its lane...
            // this should never happen because it is handled in MSVehicle::executeMove
            assert(false);
            WRITE_WARNINGF(TL("Teleporting vehicle '%'; beyond end of lane, target lane='%', time=%."),
                           veh->getID(), getID(), time2string(t));
            MSNet::getInstance()->getVehicleControl().countCollision(true);
            MSVehicleTransfer::getInstance()->add(t, veh);

        } else if (veh->collisionStopTime() == 0) {
            veh->resumeFromStopping();
            if (getCollisionAction() == COLLISION_ACTION_REMOVE) {
                WRITE_WARNINGF(TL("Removing vehicle '%' after earlier collision, lane='%', time=%."),
                               veh->getID(), veh->getLane()->getID(), time2string(t));
                veh->onRemovalFromNet(MSMoveReminder::NOTIFICATION_VAPORIZED_COLLISION);
                MSNet::getInstance()->getVehicleControl().scheduleVehicleRemoval(veh);
            } else if (getCollisionAction() == COLLISION_ACTION_TELEPORT) {
                WRITE_WARNINGF(TL("Teleporting vehicle '%' after earlier collision, lane='%', time=%."),
                               veh->getID(), veh->getLane()->getID(), time2string(t));
                MSVehicleTransfer::getInstance()->add(t, veh);
            } else {
                if (firstNotStopped == nullptr && !(*i)->isStopped() && (*i)->getLane() == this) {
                    firstNotStopped = *i;
                }
                ++i;
                continue;
            }
        } else {
            if (firstNotStopped == nullptr && !(*i)->isStopped() && (*i)->getLane() == this) {
                firstNotStopped = *i;
            }
            ++i;
            continue;
        }
        myBruttoVehicleLengthSumToRemove += length;
        myNettoVehicleLengthSumToRemove += nettoLength;
        ++i;
        i = VehCont::reverse_iterator(myVehicles.erase(i.base()));
    }
    if (firstNotStopped != nullptr) {
        const SUMOTime ttt = firstNotStopped->getVehicleType().getParameter().getTimeToTeleport(MSGlobals::gTimeToGridlock);
        const SUMOTime tttb = firstNotStopped->getVehicleType().getParameter().getTimeToTeleportBidi(MSGlobals::gTimeToTeleportBidi);
        if (ttt > 0 || MSGlobals::gTimeToGridlockHighways > 0 || MSGlobals::gTimeToTeleportDisconnected >= 0 || tttb > 0 || MSGlobals::gTimeToTeleportRSDeadlock > 0) {
            const bool wrongLane = !appropriate(firstNotStopped);
            const bool disconnected = (MSGlobals::gTimeToTeleportDisconnected >= 0
                                       && firstNotStopped->succEdge(1) != nullptr
                                       && firstNotStopped->getEdge()->allowedLanes(*firstNotStopped->succEdge(1), firstNotStopped->getVClass()) == nullptr);

            const bool r1 = ttt > 0 && firstNotStopped->getWaitingTime() > ttt && !disconnected
                            // never teleport a taxi on the last edge of it's route (where it would exit the simulation)
                            && (firstNotStopped->getDevice(typeid(MSDevice_Taxi)) == nullptr || firstNotStopped->getRoutePosition() < (firstNotStopped->getRoute().size() - 1));
            const bool r2 = !r1 && MSGlobals::gTimeToGridlockHighways > 0
                            && firstNotStopped->getWaitingTime() > MSGlobals::gTimeToGridlockHighways
                            && getSpeedLimit() > MSGlobals::gGridlockHighwaysSpeed && wrongLane
                            && !disconnected;
            const bool r3 = disconnected && firstNotStopped->getWaitingTime() > MSGlobals::gTimeToTeleportDisconnected;
            const bool r4 = !r1 && !r2 && !r3 && tttb > 0
                            && firstNotStopped->getWaitingTime() > tttb && getBidiLane() && !disconnected;
            const bool r5 = MSGlobals::gTimeToTeleportRSDeadlock > 0 && MSRailSignalControl::hasInstance() && !r1 && !r2 && !r3 && !r4
                            && firstNotStopped->getWaitingTime() > MSGlobals::gTimeToTeleportRSDeadlock && MSRailSignalControl::getInstance().haveDeadlock(firstNotStopped);
            if (r1 || r2 || r3 || r4 || r5) {
                const std::vector<MSLink*>::const_iterator link = succLinkSec(*firstNotStopped, 1, *this, firstNotStopped->getBestLanesContinuation());
                const bool minorLink = !wrongLane && (link != myLinks.end()) && !((*link)->havePriority());
                std::string reason = (wrongLane ? " (wrong lane" : (minorLink ? " (yield" : " (jam"));
                myBruttoVehicleLengthSumToRemove += firstNotStopped->getVehicleType().getLengthWithGap();
                myNettoVehicleLengthSumToRemove += firstNotStopped->getVehicleType().getLength();
                if (firstNotStopped == myVehicles.back()) {
                    myVehicles.pop_back();
                } else {
                    myVehicles.erase(std::find(myVehicles.begin(), myVehicles.end(), firstNotStopped));
                    reason = " (blocked";
                }
                WRITE_WARNINGF("Teleporting vehicle '%'; waited too long" + reason
                               + (r2 ? ", highway" : "")
                               + (r3 ? ", disconnected" : "")
                               + (r4 ? ", bidi" : "")
                               + (r5 ? ", railSignal" : "")
                               + "), lane='%', time=%.", firstNotStopped->getID(), getID(), time2string(t));
                if (wrongLane) {
                    MSNet::getInstance()->getVehicleControl().registerTeleportWrongLane();
                } else if (minorLink) {
                    MSNet::getInstance()->getVehicleControl().registerTeleportYield();
                } else {
                    MSNet::getInstance()->getVehicleControl().registerTeleportJam();
                }
                if (MSGlobals::gRemoveGridlocked) {
                    firstNotStopped->onRemovalFromNet(MSMoveReminder::NOTIFICATION_TELEPORT_ARRIVED);
                    MSNet::getInstance()->getVehicleControl().scheduleVehicleRemoval(firstNotStopped);
                } else {
                    MSVehicleTransfer::getInstance()->add(t, firstNotStopped);
                }
            }
        }
    }
    if (MSGlobals::gSublane) {
        // trigger sorting of vehicles as their order may have changed
        MSNet::getInstance()->getEdgeControl().needsVehicleIntegration(this);
    }
}


void
MSLane::markRecalculateBruttoSum() {
    myRecalculateBruttoSum = true;
}


void
MSLane::updateLengthSum() {
    myBruttoVehicleLengthSum -= myBruttoVehicleLengthSumToRemove;
    myNettoVehicleLengthSum -= myNettoVehicleLengthSumToRemove;
    myBruttoVehicleLengthSumToRemove = 0;
    myNettoVehicleLengthSumToRemove = 0;
    if (myVehicles.empty()) {
        // avoid numerical instability
        myBruttoVehicleLengthSum = 0;
        myNettoVehicleLengthSum = 0;
    } else if (myRecalculateBruttoSum) {
        myBruttoVehicleLengthSum = 0;
        for (VehCont::const_iterator i = myVehicles.begin(); i != myVehicles.end(); ++i) {
            myBruttoVehicleLengthSum += (*i)->getVehicleType().getLengthWithGap();
        }
        myRecalculateBruttoSum = false;
    }
}


void
MSLane::changeLanes(const SUMOTime t) {
    myEdge->changeLanes(t);
}


const MSEdge*
MSLane::getNextNormal() const {
    return myEdge->getNormalSuccessor();
}


const MSLane*
MSLane::getFirstInternalInConnection(double& offset) const {
    if (!this->isInternal()) {
        return nullptr;
    }
    offset = 0.;
    const MSLane* firstInternal = this;
    MSLane* pred = getCanonicalPredecessorLane();
    while (pred != nullptr && pred->isInternal()) {
        firstInternal = pred;
        offset += pred->getLength();
        pred = firstInternal->getCanonicalPredecessorLane();
    }
    return firstInternal;
}


// ------ Static (sic!) container methods  ------
bool
MSLane::dictionary(const std::string& id, MSLane* ptr) {
    const DictType::iterator it = myDict.lower_bound(id);
    if (it == myDict.end() || it->first != id) {
        // id not in myDict
        myDict.emplace_hint(it, id, ptr);
        return true;
    }
    return false;
}


MSLane*
MSLane::dictionary(const std::string& id) {
    const DictType::iterator it = myDict.find(id);
    if (it == myDict.end()) {
        // id not in myDict
        return nullptr;
    }
    return it->second;
}


void
MSLane::clear() {
    for (DictType::iterator i = myDict.begin(); i != myDict.end(); ++i) {
        delete (*i).second;
    }
    myDict.clear();
}


void
MSLane::insertIDs(std::vector<std::string>& into) {
    for (DictType::iterator i = myDict.begin(); i != myDict.end(); ++i) {
        into.push_back((*i).first);
    }
}


template<class RTREE> void
MSLane::fill(RTREE& into) {
    for (DictType::iterator i = myDict.begin(); i != myDict.end(); ++i) {
        MSLane* l = (*i).second;
        Boundary b = l->getShape().getBoxBoundary();
        b.grow(3.);
        const float cmin[2] = {(float) b.xmin(), (float) b.ymin()};
        const float cmax[2] = {(float) b.xmax(), (float) b.ymax()};
        into.Insert(cmin, cmax, l);
    }
}

template void MSLane::fill<NamedRTree>(NamedRTree& into);
template void MSLane::fill<LANE_RTREE_QUAL>(LANE_RTREE_QUAL& into);

// ------   ------
bool
MSLane::appropriate(const MSVehicle* veh) const {
    if (veh->getLaneChangeModel().isOpposite()) {
        return false;
    }
    if (myEdge->isInternal()) {
        return true;
    }
    if (veh->succEdge(1) == nullptr) {
        assert((int)veh->getBestLanes().size() > veh->getLaneIndex());
        if (veh->getBestLanes()[veh->getLaneIndex()].bestLaneOffset == 0) {
            return true;
        } else {
            return false;
        }
    }
    std::vector<MSLink*>::const_iterator link = succLinkSec(*veh, 1, *this, veh->getBestLanesContinuation());
    return (link != myLinks.end());
}


void
MSLane::integrateNewVehicles() {
    myNeedsCollisionCheck = true;
    std::vector<MSVehicle*>& buffered = myVehBuffer.getContainer();
    sort(buffered.begin(), buffered.end(), vehicle_position_sorter(this));
    for (MSVehicle* const veh : buffered) {
        assert(veh->getLane() == this);
        myVehicles.insert(myVehicles.begin(), veh);
        myBruttoVehicleLengthSum += veh->getVehicleType().getLengthWithGap();
        myNettoVehicleLengthSum += veh->getVehicleType().getLength();
        //if (true) std::cout << SIMTIME << " integrateNewVehicle lane=" << getID() << " veh=" << veh->getID() << " (on lane " << veh->getLane()->getID() << ") into lane=" << getID() << " myBrutto=" << myBruttoVehicleLengthSum << "\n";
        myEdge->markDelayed();
    }
    buffered.clear();
    myVehBuffer.unlock();
    //std::cout << SIMTIME << " integrateNewVehicle lane=" << getID() << " myVehicles1=" << toString(myVehicles);
    if (MSGlobals::gLateralResolution > 0 || myOpposite != nullptr) {
        sort(myVehicles.begin(), myVehicles.end(), vehicle_natural_position_sorter(this));
    }
    sortPartialVehicles();
#ifdef DEBUG_VEHICLE_CONTAINER
    if (DEBUG_COND) std::cout << SIMTIME << " integrateNewVehicle lane=" << getID()
                                  << " vehicles=" << toString(myVehicles) << " partials=" << toString(myPartialVehicles) << "\n";
#endif
}


void
MSLane::sortPartialVehicles() {
    if (myPartialVehicles.size() > 1) {
        sort(myPartialVehicles.begin(), myPartialVehicles.end(), vehicle_natural_position_sorter(this));
    }
}


void
MSLane::sortManeuverReservations() {
    if (myManeuverReservations.size() > 1) {
#ifdef DEBUG_CONTEXT
        if (DEBUG_COND) {
            std::cout << "sortManeuverReservations on lane " << getID()
                      << "\nBefore sort: " << toString(myManeuverReservations) << std::endl;
        }
#endif
        sort(myManeuverReservations.begin(), myManeuverReservations.end(), vehicle_natural_position_sorter(this));
#ifdef DEBUG_CONTEXT
        if (DEBUG_COND) {
            std::cout << "After sort: " << toString(myManeuverReservations) << std::endl;
        }
#endif
    }
}


bool
MSLane::isInternal() const {
    return myEdge->isInternal();
}


bool
MSLane::isNormal() const {
    return myEdge->isNormal();
}


bool
MSLane::isCrossing() const {
    return myEdge->isCrossing();
}

bool
MSLane::isWalkingArea() const {
    return myEdge->isWalkingArea();
}


MSVehicle*
MSLane::getLastFullVehicle() const {
    if (myVehicles.size() == 0) {
        return nullptr;
    }
    return myVehicles.front();
}


MSVehicle*
MSLane::getFirstFullVehicle() const {
    if (myVehicles.size() == 0) {
        return nullptr;
    }
    return myVehicles.back();
}


MSVehicle*
MSLane::getLastAnyVehicle() const {
    // all vehicles in myVehicles should have positions smaller or equal to
    // those in myPartialVehicles (unless we're on a bidi-lane)
    if (myVehicles.size() > 0) {
        if (myBidiLane != nullptr && myPartialVehicles.size() > 0) {
            if (myVehicles.front()->getPositionOnLane() > myPartialVehicles.front()->getPositionOnLane(this)) {
                return myPartialVehicles.front();
            }
        }
        return myVehicles.front();
    }
    if (myPartialVehicles.size() > 0) {
        return myPartialVehicles.front();
    }
    return nullptr;
}


MSVehicle*
MSLane::getFirstAnyVehicle() const {
    MSVehicle* result = nullptr;
    if (myVehicles.size() > 0) {
        result = myVehicles.back();
    }
    if (myPartialVehicles.size() > 0
            && (result == nullptr || result->getPositionOnLane(this) < myPartialVehicles.back()->getPositionOnLane(this))) {
        result = myPartialVehicles.back();
    }
    return result;
}


std::vector<MSLink*>::const_iterator
MSLane::succLinkSec(const SUMOVehicle& veh, int nRouteSuccs,
                    const MSLane& succLinkSource, const std::vector<MSLane*>& conts) {
    const MSEdge* nRouteEdge = veh.succEdge(nRouteSuccs);
    // check whether the vehicle tried to look beyond its route
    if (nRouteEdge == nullptr) {
        // return end (no succeeding link) if so
        return succLinkSource.myLinks.end();
    }
    // if we are on an internal lane there should only be one link and it must be allowed
    if (succLinkSource.isInternal()) {
        assert(succLinkSource.myLinks.size() == 1);
        // could have been disallowed dynamically with a rerouter or via TraCI
        // assert(succLinkSource.myLinks[0]->getLane()->allowsVehicleClass(veh.getVehicleType().getVehicleClass()));
        return succLinkSource.myLinks.begin();
    }
    // a link may be used if
    //  1) there is a destination lane ((*link)->getLane()!=0)
    //  2) the destination lane belongs to the next edge in route ((*link)->getLane()->myEdge == nRouteEdge)
    //  3) the destination lane allows the vehicle's class ((*link)->getLane()->allowsVehicleClass(veh.getVehicleClass()))

    // there should be a link which leads to the next desired lane our route in "conts" (built in "getBestLanes")
    // "conts" stores the best continuations of our current lane
    // we should never return an arbitrary link since this may cause collisions

    if (nRouteSuccs < (int)conts.size()) {
        // we go through the links in our list and return the matching one
        for (std::vector<MSLink*>::const_iterator link = succLinkSource.myLinks.begin(); link != succLinkSource.myLinks.end(); ++link) {
            if ((*link)->getLane() != nullptr && (*link)->getLane()->myEdge == nRouteEdge
                    && (*link)->getLane()->allowsVehicleClass(veh.getVClass())
                    && ((*link)->getViaLane() == nullptr || (*link)->getViaLane()->allowsVehicleClass(veh.getVClass()))) {
                // we should use the link if it connects us to the best lane
                if ((*link)->getLane() == conts[nRouteSuccs]) {
                    return link;
                }
            }
        }
    } else {
        // the source lane is a dead end (no continuations exist)
        return succLinkSource.myLinks.end();
    }
    // the only case where this should happen is for a disconnected route (deliberately ignored)
#ifdef DEBUG_NO_CONNECTION
    // the "'" around the ids are missing intentionally in the message below because it slows messaging down, resulting in test timeouts
    WRITE_WARNING("Could not find connection between lane " + succLinkSource.getID() + " and lane " + conts[nRouteSuccs]->getID() +
                  " for vehicle " + veh.getID() + ", time=" + time2string(MSNet::getInstance()->getCurrentTimeStep()) + ".");
#endif
    return succLinkSource.myLinks.end();
}


const MSLink*
MSLane::getLinkTo(const MSLane* const target) const {
    const bool internal = target->isInternal();
    for (const MSLink* const l : myLinks) {
        if ((internal && l->getViaLane() == target) || (!internal && l->getLane() == target)) {
            return l;
        }
    }
    return nullptr;
}


const MSLane*
MSLane::getInternalFollowingLane(const MSLane* const target) const {
    for (const MSLink* const l : myLinks) {
        if (l->getLane() == target) {
            return l->getViaLane();
        }
    }
    return nullptr;
}


const MSLink*
MSLane::getEntryLink() const {
    if (!isInternal()) {
        return nullptr;
    }
    const MSLane* internal = this;
    const MSLane* lane = this->getCanonicalPredecessorLane();
    assert(lane != nullptr);
    while (lane->isInternal()) {
        internal = lane;
        lane = lane->getCanonicalPredecessorLane();
        assert(lane != nullptr);
    }
    return lane->getLinkTo(internal);
}


void
MSLane::setMaxSpeed(double val, bool byVSS, bool byTraCI, double jamThreshold) {
    myMaxSpeed = val;
    mySpeedByVSS = byVSS;
    mySpeedByTraCI = byTraCI;
    myEdge->recalcCache();
    if (MSGlobals::gUseMesoSim) {
        MESegment* first = MSGlobals::gMesoNet->getSegmentForEdge(*myEdge);
        while (first != nullptr) {
            first->setSpeed(val, SIMSTEP, jamThreshold, myIndex);
            first = first->getNextSegment();
        }
    }
}


void
MSLane::setFrictionCoefficient(double val) {
    myFrictionCoefficient = val;
    myEdge->recalcCache();
}


void
MSLane::setLength(double val) {
    myLength = val;
    myEdge->recalcCache();
}


void
MSLane::swapAfterLaneChange(SUMOTime) {
    //if (getID() == "disabled_lane") std::cout << SIMTIME << " swapAfterLaneChange lane=" << getID() << " myVehicles=" << toString(myVehicles) << " myTmpVehicles=" << toString(myTmpVehicles) << "\n";
    myVehicles = myTmpVehicles;
    myTmpVehicles.clear();
    // this needs to be done after finishing lane-changing for all lanes on the
    // current edge (MSLaneChanger::updateLanes())
    sortPartialVehicles();
    if (MSGlobals::gSublane && getOpposite() != nullptr) {
        getOpposite()->sortPartialVehicles();
    }
    if (myBidiLane != nullptr) {
        myBidiLane->sortPartialVehicles();
    }
}


MSVehicle*
MSLane::removeVehicle(MSVehicle* remVehicle, MSMoveReminder::Notification notification, bool notify) {
    assert(remVehicle->getLane() == this);
    for (MSLane::VehCont::iterator it = myVehicles.begin(); it < myVehicles.end(); it++) {
        if (remVehicle == *it) {
            if (notify) {
                remVehicle->leaveLane(notification);
            }
            myVehicles.erase(it);
            myBruttoVehicleLengthSum -= remVehicle->getVehicleType().getLengthWithGap();
            myNettoVehicleLengthSum -= remVehicle->getVehicleType().getLength();
            break;
        }
    }
    return remVehicle;
}


MSLane*
MSLane::getParallelLane(int offset, bool includeOpposite) const {
    return myEdge->parallelLane(this, offset, includeOpposite);
}


void
MSLane::addIncomingLane(MSLane* lane, MSLink* viaLink) {
    IncomingLaneInfo ili;
    ili.lane = lane;
    ili.viaLink = viaLink;
    ili.length = lane->getLength();
    myIncomingLanes.push_back(ili);
}


void
MSLane::addApproachingLane(MSLane* lane, bool warnMultiCon) {
    MSEdge* approachingEdge = &lane->getEdge();
    if (myApproachingLanes.find(approachingEdge) == myApproachingLanes.end()) {
        myApproachingLanes[approachingEdge] = std::vector<MSLane*>();
    } else if (!approachingEdge->isInternal() && warnMultiCon) {
        // whenever a normal edge connects twice, there is a corresponding
        // internal edge wich connects twice, one warning is sufficient
        WRITE_WARNINGF(TL("Lane '%' is approached multiple times from edge '%'. This may cause collisions."),
                       getID(), approachingEdge->getID());
    }
    myApproachingLanes[approachingEdge].push_back(lane);
}


bool
MSLane::isApproachedFrom(MSEdge* const edge, MSLane* const lane) {
    std::map<MSEdge*, std::vector<MSLane*> >::const_iterator i = myApproachingLanes.find(edge);
    if (i == myApproachingLanes.end()) {
        return false;
    }
    const std::vector<MSLane*>& lanes = (*i).second;
    return std::find(lanes.begin(), lanes.end(), lane) != lanes.end();
}


double MSLane::getMissingRearGap(const MSVehicle* leader, double backOffset, double leaderSpeed) const {
    // this follows the same logic as getFollowerOnConsecutive. we do a tree
    // search and check for the vehicle with the largest missing rear gap within
    // relevant range
    double result = 0;
    const double leaderDecel = leader->getCarFollowModel().getMaxDecel();
    CLeaderDist followerInfo = getFollowersOnConsecutive(leader, backOffset, false)[0];
    const MSVehicle* v = followerInfo.first;
    if (v != nullptr) {
        result = v->getCarFollowModel().getSecureGap(v, leader, v->getSpeed(), leaderSpeed, leaderDecel) - followerInfo.second;
    }
    return result;
}


double
MSLane::getMaximumBrakeDist() const {
    const MSVehicleControl& vc = MSNet::getInstance()->getVehicleControl();
    const double maxSpeed = getSpeedLimit() * vc.getMaxSpeedFactor();
    // NOTE: For the euler update this is an upper bound on the actual braking distance (see ticket #860)
    // impose a hard bound due to visibility / common sense to avoid unnecessary computation if there are strange vehicles in the fleet
    const double minDecel = isRailway(myPermissions) ? vc.getMinDecelerationRail() : vc.getMinDeceleration();
    return MIN2(maxSpeed * maxSpeed * 0.5 / minDecel,
                myPermissions == SVC_SHIP ? 10000.0 : 1000.0);
}


std::pair<MSVehicle* const, double>
MSLane::getLeader(const MSVehicle* veh, const double vehPos, const std::vector<MSLane*>& bestLaneConts, double dist, bool checkTmpVehicles) const {
    // get the leading vehicle for (shadow) veh
    // XXX this only works as long as all lanes of an edge have equal length
#ifdef DEBUG_CONTEXT
    if (DEBUG_COND2(veh)) {
        std::cout << "   getLeader lane=" << getID() << " ego=" << veh->getID() << " vehs=" << toString(myVehicles) << " tmpVehs=" << toString(myTmpVehicles) << "\n";
    }
#endif
    if (checkTmpVehicles) {
        for (VehCont::const_iterator last = myTmpVehicles.begin(); last != myTmpVehicles.end(); ++last) {
            // XXX refactor leaderInfo to use a const vehicle all the way through the call hierarchy
            MSVehicle* pred = (MSVehicle*)*last;
            if (pred == veh) {
                continue;
            }
#ifdef DEBUG_CONTEXT
            if (DEBUG_COND2(veh)) {
                std::cout << std::setprecision(gPrecision) << "   getLeader lane=" << getID() << " ego=" << veh->getID() << " egoPos=" << vehPos << " pred=" << pred->getID() << " predPos=" << pred->getPositionOnLane() << "\n";
            }
#endif
            if (pred->getPositionOnLane() >= vehPos) {
                return std::pair<MSVehicle* const, double>(pred, pred->getBackPositionOnLane(this) - veh->getVehicleType().getMinGap() - vehPos);
            }
        }
    } else {
        for (AnyVehicleIterator last = anyVehiclesBegin(); last != anyVehiclesEnd(); ++last) {
            // XXX refactor leaderInfo to use a const vehicle all the way through the call hierarchy
            MSVehicle* pred = (MSVehicle*)*last;
            if (pred == veh) {
                continue;
            }
#ifdef DEBUG_CONTEXT
            if (DEBUG_COND2(veh)) {
                std::cout << "   getLeader lane=" << getID() << " ego=" << veh->getID() << " egoPos=" << vehPos
                          << " pred=" << pred->getID() << " predPos=" << pred->getPositionOnLane(this) << " predBack=" << pred->getBackPositionOnLane(this) << "\n";
            }
#endif
            if (pred->getPositionOnLane(this) >= vehPos) {
                if (MSGlobals::gLaneChangeDuration > 0
                        && pred->getLaneChangeModel().isOpposite()
                        && !pred->getLaneChangeModel().isChangingLanes()
                        && pred->getLaneChangeModel().getShadowLane() == this) {
                    // skip non-overlapping shadow
                    continue;
                }
                return std::pair<MSVehicle* const, double>(pred, pred->getBackPositionOnLane(this) - veh->getVehicleType().getMinGap() - vehPos);
            }
        }
    }
    // XXX from here on the code mirrors MSLaneChanger::getRealLeader
    if (bestLaneConts.size() > 0) {
        double seen = getLength() - vehPos;
        double speed = veh->getSpeed();
        if (dist < 0) {
            dist = veh->getCarFollowModel().brakeGap(speed) + veh->getVehicleType().getMinGap();
        }
#ifdef DEBUG_CONTEXT
        if (DEBUG_COND2(veh)) {
            std::cout << "   getLeader lane=" << getID() << " seen=" << seen << " dist=" << dist << "\n";
        }
#endif
        if (seen > dist) {
            return std::pair<MSVehicle* const, double>(static_cast<MSVehicle*>(nullptr), -1);
        }
        return getLeaderOnConsecutive(dist, seen, speed, *veh, bestLaneConts);
    } else {
        return std::make_pair(static_cast<MSVehicle*>(nullptr), -1);
    }
}


std::pair<MSVehicle* const, double>
MSLane::getLeaderOnConsecutive(double dist, double seen, double speed, const MSVehicle& veh,
                               const std::vector<MSLane*>& bestLaneConts, bool considerCrossingFoes) const {
#ifdef DEBUG_CONTEXT
    if (DEBUG_COND2(&veh)) {
        std::cout << "   getLeaderOnConsecutive lane=" << getID() << " ego=" << veh.getID() << " seen=" << seen << " dist=" << dist << " conts=" << toString(bestLaneConts) << "\n";
    }
#endif
    if (seen > dist && !isInternal()) {
        return std::make_pair(static_cast<MSVehicle*>(nullptr), -1);
    }
    int view = 1;
    // loop over following lanes
    if (myPartialVehicles.size() > 0) {
        // XXX
        MSVehicle* pred = myPartialVehicles.front();
        const double gap = seen - (getLength() - pred->getBackPositionOnLane(this)) - veh.getVehicleType().getMinGap();
#ifdef DEBUG_CONTEXT
        if (DEBUG_COND2(&veh)) {
            std::cout << "    predGap=" << gap << " partials=" << toString(myPartialVehicles) << "\n";
        }
#endif
        // make sure pred is really a leader and not doing continous lane-changing behind ego
        if (gap > 0) {
            return std::pair<MSVehicle* const, double>(pred, gap);
        }
    }
#ifdef DEBUG_CONTEXT
    if (DEBUG_COND2(&veh)) {
        gDebugFlag1 = true;
    }
#endif
    const MSLane* nextLane = this;
    do {
        nextLane->getVehiclesSecure(); // lock against running sim when called from GUI for time gap coloring
        // get the next link used
        std::vector<MSLink*>::const_iterator link = succLinkSec(veh, view, *nextLane, bestLaneConts);
        if (nextLane->isLinkEnd(link) && view < veh.getRoute().size() - veh.getRoutePosition()) {
            const MSEdge* nextEdge = *(veh.getCurrentRouteEdge() + view);
            if (nextEdge->getNumLanes() == 1) {
                // lanes are unambiguous on the next route edge, continue beyond bestLaneConts
                for (link = nextLane->getLinkCont().begin(); link < nextLane->getLinkCont().end(); link++) {
                    if ((*link)->getLane() == nextEdge->getLanes().front()) {
                        break;
                    }
                }
            }
        }
        if (nextLane->isLinkEnd(link)) {
#ifdef DEBUG_CONTEXT
            if (DEBUG_COND2(&veh)) {
                std::cout << "    cannot continue after nextLane=" << nextLane->getID() << "\n";
            }
#endif
            nextLane->releaseVehicles();
            break;
        }
        // check for link leaders
        const bool laneChanging = veh.getLane() != this;
        const MSLink::LinkLeaders linkLeaders = (*link)->getLeaderInfo(&veh, seen);
        nextLane->releaseVehicles();
        if (linkLeaders.size() > 0) {
            std::pair<MSVehicle*, double> result;
            double shortestGap = std::numeric_limits<double>::max();
            for (auto ll : linkLeaders) {
                double gap = ll.vehAndGap.second;
                MSVehicle* lVeh = ll.vehAndGap.first;
                if (lVeh != nullptr) {
                    // leader is a vehicle, not a pedestrian
                    gap += lVeh->getCarFollowModel().brakeGap(lVeh->getSpeed(), lVeh->getCarFollowModel().getMaxDecel(), 0);
                }
#ifdef DEBUG_CONTEXT
                if (DEBUG_COND2(&veh)) {
                    std::cout << "      linkLeader candidate " << Named::getIDSecure(lVeh)
                              << " isLeader=" << veh.isLeader(*link, lVeh, ll.vehAndGap.second)
                              << " gap=" << ll.vehAndGap.second
                              << " gap+brakeing=" << gap
                              << "\n";
                }
#endif
                // skip vehicles which do not share the outgoing edge (to get only real leader vehicles in TraCI #13842)
                if (!considerCrossingFoes && !ll.sameTarget()) {
                    continue;
                }
                // in the context of lane-changing, all candidates are leaders
                if (lVeh != nullptr && !laneChanging && !veh.isLeader(*link, lVeh, ll.vehAndGap.second)) {
                    continue;
                }
                if (gap < shortestGap) {
                    shortestGap = gap;
                    if (ll.vehAndGap.second < 0 && !MSGlobals::gComputeLC) {
                        // can always continue up to the stop line or crossing point
                        // @todo: figure out whether this should also impact lane changing
                        ll.vehAndGap.second = MAX2(seen - nextLane->getLength(), ll.distToCrossing);
                    }
                    result = ll.vehAndGap;
                }
            }
            if (shortestGap != std::numeric_limits<double>::max()) {
#ifdef DEBUG_CONTEXT
                if (DEBUG_COND2(&veh)) {
                    std::cout << "    found linkLeader after nextLane=" << nextLane->getID() << "\n";
                    gDebugFlag1 = false;
                }
#endif
                return result;
            }
        }
        bool nextInternal = (*link)->getViaLane() != nullptr;
        nextLane = (*link)->getViaLaneOrLane();
        if (nextLane == nullptr) {
            break;
        }
        nextLane->getVehiclesSecure(); // lock against running sim when called from GUI for time gap coloring
        MSVehicle* leader = nextLane->getLastAnyVehicle();
        if (leader != nullptr) {
#ifdef DEBUG_CONTEXT
            if (DEBUG_COND2(&veh)) {
                std::cout << "    found leader " << leader->getID() << " on nextLane=" << nextLane->getID() << "\n";
            }
#endif
            const double leaderDist = seen + leader->getBackPositionOnLane(nextLane) - veh.getVehicleType().getMinGap();
            nextLane->releaseVehicles();
            return std::make_pair(leader, leaderDist);
        }
        nextLane->releaseVehicles();
        if (nextLane->getVehicleMaxSpeed(&veh) < speed) {
            dist = veh.getCarFollowModel().brakeGap(nextLane->getVehicleMaxSpeed(&veh));
        }
        seen += nextLane->getLength();
        if (!nextInternal) {
            view++;
        }
    } while (seen <= dist || nextLane->isInternal());
#ifdef DEBUG_CONTEXT
    gDebugFlag1 = false;
#endif
    return std::make_pair(static_cast<MSVehicle*>(nullptr), -1);
}


std::pair<MSVehicle* const, double>
MSLane::getCriticalLeader(double dist, double seen, double speed, const MSVehicle& veh) const {
#ifdef DEBUG_CONTEXT
    if (DEBUG_COND2(&veh)) {
        std::cout << SIMTIME << " getCriticalLeader. lane=" << getID() << " veh=" << veh.getID() << "\n";
    }
#endif
    const std::vector<MSLane*>& bestLaneConts = veh.getBestLanesContinuation(this);
    std::pair<MSVehicle*, double> result = std::make_pair(static_cast<MSVehicle*>(nullptr), -1);
    double safeSpeed = std::numeric_limits<double>::max();
    int view = 1;
    // loop over following lanes
    // @note: we don't check the partial occupator for this lane since it was
    // already checked in MSLaneChanger::getRealLeader()
    const MSLane* nextLane = this;
    SUMOTime arrivalTime = MSNet::getInstance()->getCurrentTimeStep() + TIME2STEPS(seen / MAX2(speed, NUMERICAL_EPS));
    do {
        // get the next link used
        std::vector<MSLink*>::const_iterator link = succLinkSec(veh, view, *nextLane, bestLaneConts);
        if (nextLane->isLinkEnd(link) || !(*link)->opened(arrivalTime, speed, speed, veh.getVehicleType().getLength(),
                veh.getImpatience(), veh.getCarFollowModel().getMaxDecel(), 0, veh.getLateralPositionOnLane(), nullptr, false, &veh) || (*link)->haveRed()) {
            return result;
        }
        // check for link leaders
#ifdef DEBUG_CONTEXT
        if (DEBUG_COND2(&veh)) {
            gDebugFlag1 = true;    // See MSLink::getLeaderInfo
        }
#endif
        const MSLink::LinkLeaders linkLeaders = (*link)->getLeaderInfo(&veh, seen);
#ifdef DEBUG_CONTEXT
        if (DEBUG_COND2(&veh)) {
            gDebugFlag1 = false;    // See MSLink::getLeaderInfo
        }
#endif
        for (MSLink::LinkLeaders::const_iterator it = linkLeaders.begin(); it != linkLeaders.end(); ++it) {
            const MSVehicle* leader = (*it).vehAndGap.first;
            if (leader != nullptr && leader != result.first) {
                // XXX ignoring pedestrians here!
                // XXX ignoring the fact that the link leader may alread by following us
                // XXX ignoring the fact that we may drive up to the crossing point
                double tmpSpeed = safeSpeed;
                veh.adaptToJunctionLeader((*it).vehAndGap, seen, nullptr, nextLane, tmpSpeed, tmpSpeed, (*it).distToCrossing);
#ifdef DEBUG_CONTEXT
                if (DEBUG_COND2(&veh)) {
                    std::cout << "    linkLeader=" << leader->getID() << " gap=" << result.second << " tmpSpeed=" << tmpSpeed << " safeSpeed=" << safeSpeed << "\n";
                }
#endif
                if (tmpSpeed < safeSpeed) {
                    safeSpeed = tmpSpeed;
                    result = (*it).vehAndGap;
                }
            }
        }
        bool nextInternal = (*link)->getViaLane() != nullptr;
        nextLane = (*link)->getViaLaneOrLane();
        if (nextLane == nullptr) {
            break;
        }
        MSVehicle* leader = nextLane->getLastAnyVehicle();
        if (leader != nullptr && leader != result.first) {
            const double gap = seen + leader->getBackPositionOnLane(nextLane) - veh.getVehicleType().getMinGap();
            const double tmpSpeed = veh.getCarFollowModel().insertionFollowSpeed(&veh, speed, gap, leader->getSpeed(), leader->getCarFollowModel().getMaxDecel(), leader);
            if (tmpSpeed < safeSpeed) {
                safeSpeed = tmpSpeed;
                result = std::make_pair(leader, gap);
            }
        }
        if (nextLane->getVehicleMaxSpeed(&veh) < speed) {
            dist = veh.getCarFollowModel().brakeGap(nextLane->getVehicleMaxSpeed(&veh));
        }
        seen += nextLane->getLength();
        if (seen <= dist) {
            // delaying the update of arrivalTime and making it conditional to avoid possible integer overflows
            arrivalTime += TIME2STEPS(nextLane->getLength() / MAX2(speed, NUMERICAL_EPS));
        }
        if (!nextInternal) {
            view++;
        }
    } while (seen <= dist || nextLane->isInternal());
    return result;
}


MSLane*
MSLane::getLogicalPredecessorLane() const {
    if (myLogicalPredecessorLane == nullptr) {
        MSEdgeVector pred = myEdge->getPredecessors();
        // get only those edges which connect to this lane
        for (MSEdgeVector::iterator i = pred.begin(); i != pred.end();) {
            std::vector<IncomingLaneInfo>::const_iterator j = find_if(myIncomingLanes.begin(), myIncomingLanes.end(), edge_finder(*i));
            if (j == myIncomingLanes.end()) {
                i = pred.erase(i);
            } else {
                ++i;
            }
        }
        // get the lane with the "straightest" connection
        if (pred.size() != 0) {
            std::sort(pred.begin(), pred.end(), by_connections_to_sorter(&getEdge()));
            MSEdge* best = *pred.begin();
            std::vector<IncomingLaneInfo>::const_iterator j = find_if(myIncomingLanes.begin(), myIncomingLanes.end(), edge_finder(best));
            myLogicalPredecessorLane = j->lane;
        }
    }
    return myLogicalPredecessorLane;
}


const MSLane*
MSLane::getNormalPredecessorLane() const {
    if (isInternal()) {
        return getLogicalPredecessorLane()->getNormalPredecessorLane();
    } else {
        return this;
    }
}


const MSLane*
MSLane::getNormalSuccessorLane() const {
    if (isInternal()) {
        return getCanonicalSuccessorLane()->getNormalSuccessorLane();
    } else {
        return this;
    }
}


MSLane*
MSLane::getLogicalPredecessorLane(const MSEdge& fromEdge) const {
    for (const IncomingLaneInfo& cand : myIncomingLanes) {
        if (&(cand.lane->getEdge()) == &fromEdge) {
            return cand.lane;
        }
    }
    return nullptr;
}


MSLane*
MSLane::getCanonicalPredecessorLane() const {
    if (myCanonicalPredecessorLane != nullptr) {
        return myCanonicalPredecessorLane;
    }
    if (myIncomingLanes.empty()) {
        return nullptr;
    }
    // myCanonicalPredecessorLane has not yet been determined and there exist incoming lanes
    // get the lane with the priorized (or if this does not apply the "straightest") connection
    const auto bestLane = std::min_element(myIncomingLanes.begin(), myIncomingLanes.end(), incoming_lane_priority_sorter(this));
    {
#ifdef HAVE_FOX
        ScopedLocker<> lock(myLeaderInfoMutex, MSGlobals::gNumSimThreads > 1);
#endif
        myCanonicalPredecessorLane = bestLane->lane;
    }
#ifdef DEBUG_LANE_SORTER
    std::cout << "\nBest predecessor lane for lane '" << myID << "': '" << myCanonicalPredecessorLane->getID() << "'" << std::endl;
#endif
    return myCanonicalPredecessorLane;
}


MSLane*
MSLane::getCanonicalSuccessorLane() const {
    if (myCanonicalSuccessorLane != nullptr) {
        return myCanonicalSuccessorLane;
    }
    if (myLinks.empty()) {
        return nullptr;
    }
    // myCanonicalSuccessorLane has not yet been determined and there exist outgoing links
    std::vector<MSLink*> candidateLinks = myLinks;
    // get the lane with the priorized (or if this does not apply the "straightest") connection
    std::sort(candidateLinks.begin(), candidateLinks.end(), outgoing_lane_priority_sorter(this));
    MSLane* best = (*candidateLinks.begin())->getViaLaneOrLane();
#ifdef DEBUG_LANE_SORTER
    std::cout << "\nBest successor lane for lane '" << myID << "': '" << best->getID() << "'" << std::endl;
#endif
    myCanonicalSuccessorLane = best;
    return myCanonicalSuccessorLane;
}


LinkState
MSLane::getIncomingLinkState() const {
    const MSLane* const pred = getLogicalPredecessorLane();
    if (pred == nullptr) {
        return LINKSTATE_DEADEND;
    } else {
        return pred->getLinkTo(this)->getState();
    }
}


const std::vector<std::pair<const MSLane*, const MSEdge*> >
MSLane::getOutgoingViaLanes() const {
    std::vector<std::pair<const MSLane*, const MSEdge*> > result;
    for (const MSLink* link : myLinks) {
        assert(link->getLane() != nullptr);
        result.push_back(std::make_pair(link->getLane(), link->getViaLane() == nullptr ? nullptr : &link->getViaLane()->getEdge()));
    }
    return result;
}

std::vector<const MSLane*>
MSLane::getNormalIncomingLanes() const {
    std::vector<const MSLane*> result = {};
    for (std::map<MSEdge*, std::vector<MSLane*> >::const_iterator it = myApproachingLanes.begin(); it != myApproachingLanes.end(); ++it) {
        for (std::vector<MSLane*>::const_iterator it_lane = (*it).second.begin(); it_lane != (*it).second.end(); ++it_lane) {
            if (!((*it_lane)->isInternal())) {
                result.push_back(*it_lane);
            }
        }
    }
    return result;
}


void
MSLane::leftByLaneChange(MSVehicle* v) {
    myBruttoVehicleLengthSum -= v->getVehicleType().getLengthWithGap();
    myNettoVehicleLengthSum -= v->getVehicleType().getLength();
}


void
MSLane::enteredByLaneChange(MSVehicle* v) {
    myBruttoVehicleLengthSum += v->getVehicleType().getLengthWithGap();
    myNettoVehicleLengthSum += v->getVehicleType().getLength();
}


int
MSLane::getCrossingIndex() const {
    for (std::vector<MSLink*>::const_iterator i = myLinks.begin(); i != myLinks.end(); ++i) {
        if ((*i)->getLane()->getEdge().isCrossing()) {
            return (int)(i - myLinks.begin());
        }
    }
    return -1;
}

// ------------ Current state retrieval
double
MSLane::getFractionalVehicleLength(bool brutto) const {
    double sum = 0;
    if (myPartialVehicles.size() > 0) {
        const MSLane* bidi = getBidiLane();
        for (MSVehicle* cand : myPartialVehicles) {
            if (MSGlobals::gSublane && cand->getLaneChangeModel().getShadowLane() == this) {
                continue;
            }
            if (cand->getLane() == bidi) {
                sum += (brutto ? cand->getVehicleType().getLengthWithGap() : cand->getVehicleType().getLength());
            } else {
                sum += myLength - cand->getBackPositionOnLane(this);
            }
        }
    }
    return sum;
}

double
MSLane::getBruttoOccupancy() const {
    getVehiclesSecure();
    double fractions = getFractionalVehicleLength(true);
    if (myVehicles.size() != 0) {
        MSVehicle* lastVeh = myVehicles.front();
        if (lastVeh->getPositionOnLane() < lastVeh->getVehicleType().getLength()) {
            fractions -= (lastVeh->getVehicleType().getLength() - lastVeh->getPositionOnLane());
        }
    }
    releaseVehicles();
    return MIN2(1., (myBruttoVehicleLengthSum + fractions) / myLength);
}


double
MSLane::getNettoOccupancy() const {
    getVehiclesSecure();
    double fractions = getFractionalVehicleLength(false);
    if (myVehicles.size() != 0) {
        MSVehicle* lastVeh = myVehicles.front();
        if (lastVeh->getPositionOnLane() < lastVeh->getVehicleType().getLength()) {
            fractions -= (lastVeh->getVehicleType().getLength() - lastVeh->getPositionOnLane());
        }
    }
    releaseVehicles();
    return (myNettoVehicleLengthSum + fractions) / myLength;
}


double
MSLane::getWaitingSeconds() const {
    if (myVehicles.size() == 0) {
        return 0;
    }
    double wtime = 0;
    for (VehCont::const_iterator i = myVehicles.begin(); i != myVehicles.end(); ++i) {
        wtime += (*i)->getWaitingSeconds();
    }
    return wtime;
}


double
MSLane::getMeanSpeed() const {
    if (myVehicles.size() == 0) {
        return myMaxSpeed;
    }
    double v = 0;
    int numVehs = 0;
    for (const MSVehicle* const veh : getVehiclesSecure()) {
        if (!veh->isStopped() || !myEdge->hasLaneChanger()) {
            v += veh->getSpeed();
            numVehs++;
        }
    }
    releaseVehicles();
    if (numVehs == 0) {
        return myMaxSpeed;
    }
    return v / numVehs;
}


double
MSLane::getMeanSpeedBike() const {
    // @note: redundant code with getMeanSpeed to avoid extra checks in a function that is called very often
    if (myVehicles.size() == 0) {
        return myMaxSpeed;
    }
    double v = 0;
    int numBikes = 0;
    for (MSVehicle* veh : getVehiclesSecure()) {
        if (veh->getVClass() == SVC_BICYCLE) {
            v += veh->getSpeed();
            numBikes++;
        }
    }
    double ret;
    if (numBikes > 0) {
        ret = v / (double) myVehicles.size();
    } else {
        ret = myMaxSpeed;
    }
    releaseVehicles();
    return ret;
}


double
MSLane::getHarmonoise_NoiseEmissions() const {
    double ret = 0;
    const MSLane::VehCont& vehs = getVehiclesSecure();
    if (vehs.size() == 0) {
        releaseVehicles();
        return 0;
    }
    for (MSLane::VehCont::const_iterator i = vehs.begin(); i != vehs.end(); ++i) {
        double sv = (*i)->getHarmonoise_NoiseEmissions();
        ret += (double) pow(10., (sv / 10.));
    }
    releaseVehicles();
    return HelpersHarmonoise::sum(ret);
}


int
MSLane::vehicle_position_sorter::operator()(MSVehicle* v1, MSVehicle* v2) const {
    const double pos1 = v1->getBackPositionOnLane(myLane);
    const double pos2 = v2->getBackPositionOnLane(myLane);
    if (pos1 != pos2) {
        return pos1 > pos2;
    } else {
        return v1->getNumericalID() > v2->getNumericalID();
    }
}


int
MSLane::vehicle_natural_position_sorter::operator()(MSVehicle* v1, MSVehicle* v2) const {
    const double pos1 = v1->getBackPositionOnLane(myLane);
    const double pos2 = v2->getBackPositionOnLane(myLane);
    if (pos1 != pos2) {
        return pos1 < pos2;
    } else {
        return v1->getLateralPositionOnLane() < v2->getLateralPositionOnLane();
    }
}


MSLane::by_connections_to_sorter::by_connections_to_sorter(const MSEdge* const e) :
    myEdge(e),
    myLaneDir(e->getLanes()[0]->getShape().angleAt2D(0)) {
}


int
MSLane::by_connections_to_sorter::operator()(const MSEdge* const e1, const MSEdge* const e2) const {
//    std::cout << "\nby_connections_to_sorter()";

    const std::vector<MSLane*>* ae1 = e1->allowedLanes(*myEdge);
    const std::vector<MSLane*>* ae2 = e2->allowedLanes(*myEdge);
    double s1 = 0;
    if (ae1 != nullptr && ae1->size() != 0) {
//        std::cout << "\nsize 1 = " << ae1->size()
//        << " anglediff 1 = " << fabs(GeomHelper::angleDiff((*ae1)[0]->getShape().angleAt2D(0), myLaneDir)) / M_PI / 2.
//        << "\nallowed lanes: ";
//        for (std::vector<MSLane*>::const_iterator j = ae1->begin(); j != ae1->end(); ++j){
//            std::cout << "\n" << (*j)->getID();
//        }
        s1 = (double) ae1->size() + fabs(GeomHelper::angleDiff((*ae1)[0]->getShape().angleAt2D(0), myLaneDir)) / M_PI / 2.;
    }
    double s2 = 0;
    if (ae2 != nullptr && ae2->size() != 0) {
//        std::cout << "\nsize 2 = " << ae2->size()
//        << " anglediff 2 = " << fabs(GeomHelper::angleDiff((*ae2)[0]->getShape().angleAt2D(0), myLaneDir)) / M_PI / 2.
//        << "\nallowed lanes: ";
//        for (std::vector<MSLane*>::const_iterator j = ae2->begin(); j != ae2->end(); ++j){
//            std::cout << "\n" << (*j)->getID();
//        }
        s2 = (double) ae2->size() + fabs(GeomHelper::angleDiff((*ae2)[0]->getShape().angleAt2D(0), myLaneDir)) / M_PI / 2.;
    }

//    std::cout << "\ne1 = " << e1->getID() << " e2 = " << e2->getID()
//            << "\ns1 = " << s1 << " s2 = " << s2
//            << std::endl;

    return s1 < s2;
}


MSLane::incoming_lane_priority_sorter::incoming_lane_priority_sorter(const MSLane* const targetLane) :
    myLane(targetLane),
    myLaneDir(targetLane->getShape().angleAt2D(0)) {}

int
MSLane::incoming_lane_priority_sorter::operator()(const IncomingLaneInfo& laneInfo1, const IncomingLaneInfo& laneInfo2) const {
    const MSLane* noninternal1 = laneInfo1.lane;
    while (noninternal1->isInternal()) {
        assert(noninternal1->getIncomingLanes().size() == 1);
        noninternal1 = noninternal1->getIncomingLanes()[0].lane;
    }
    MSLane* noninternal2 = laneInfo2.lane;
    while (noninternal2->isInternal()) {
        assert(noninternal2->getIncomingLanes().size() == 1);
        noninternal2 = noninternal2->getIncomingLanes()[0].lane;
    }

    const MSLink* link1 = noninternal1->getLinkTo(myLane);
    const MSLink* link2 = noninternal2->getLinkTo(myLane);

#ifdef DEBUG_LANE_SORTER
    std::cout << "\nincoming_lane_priority sorter()\n"
              << "noninternal predecessor for lane '" << laneInfo1.lane->getID()
              << "': '" << noninternal1->getID() << "'\n"
              << "noninternal predecessor for lane '" << laneInfo2.lane->getID()
              << "': '" << noninternal2->getID() << "'\n";
#endif

    assert(laneInfo1.lane->isInternal() || link1 == laneInfo1.viaLink);
    assert(link1 != 0);
    assert(link2 != 0);

    // check priority between links
    bool priorized1 = true;
    bool priorized2 = true;

#ifdef DEBUG_LANE_SORTER
    std::cout << "FoeLinks of '" << noninternal1->getID() << "'" << std::endl;
#endif
    for (const MSLink* const foeLink : link1->getFoeLinks()) {
#ifdef DEBUG_LANE_SORTER
        std::cout << foeLink->getLaneBefore()->getID() << std::endl;
#endif
        if (foeLink == link2) {
            priorized1 = false;
            break;
        }
    }

#ifdef DEBUG_LANE_SORTER
    std::cout << "FoeLinks of '" << noninternal2->getID() << "'" << std::endl;
#endif
    for (const MSLink* const foeLink : link2->getFoeLinks()) {
#ifdef DEBUG_LANE_SORTER
        std::cout << foeLink->getLaneBefore()->getID() << std::endl;
#endif
        // either link1 is priorized, or it should not appear in link2's foes
        if (foeLink == link1) {
            priorized2 = false;
            break;
        }
    }
    // if one link is subordinate, the other must be priorized (except for
    // traffic lights where mutual response is permitted to handle stuck-on-red
    // situation)
    if (priorized1 != priorized2) {
        return priorized1;
    }

    // both are priorized, compare angle difference
    double d1 = fabs(GeomHelper::angleDiff(noninternal1->getShape().angleAt2D(0), myLaneDir));
    double d2 = fabs(GeomHelper::angleDiff(noninternal2->getShape().angleAt2D(0), myLaneDir));

    return d2 > d1;
}



MSLane::outgoing_lane_priority_sorter::outgoing_lane_priority_sorter(const MSLane* const sourceLane) :
    myLaneDir(sourceLane->getShape().angleAt2D(0)) {}

int
MSLane::outgoing_lane_priority_sorter::operator()(const MSLink* link1, const MSLink* link2) const {
    const MSLane* target1 = link1->getLane();
    const MSLane* target2 = link2->getLane();
    if (target2 == nullptr) {
        return true;
    }
    if (target1 == nullptr) {
        return false;
    }

#ifdef DEBUG_LANE_SORTER
    std::cout << "\noutgoing_lane_priority sorter()\n"
              << "noninternal successors for lane '" << myLane->getID()
              << "': '" << target1->getID() << "' and "
              << "'" << target2->getID() << "'\n";
#endif

    // priority of targets
    int priority1 = target1->getEdge().getPriority();
    int priority2 = target2->getEdge().getPriority();

    if (priority1 != priority2) {
        return priority1 > priority2;
    }

    // if priority of targets coincides, use angle difference

    // both are priorized, compare angle difference
    double d1 = fabs(GeomHelper::angleDiff(target1->getShape().angleAt2D(0), myLaneDir));
    double d2 = fabs(GeomHelper::angleDiff(target2->getShape().angleAt2D(0), myLaneDir));

    return d2 > d1;
}

void
MSLane::addParking(MSBaseVehicle* veh) {
    myParkingVehicles.insert(veh);
}


void
MSLane::removeParking(MSBaseVehicle* veh) {
    myParkingVehicles.erase(veh);
}

bool
MSLane::hasApproaching() const {
    for (const MSLink* link : myLinks) {
        if (link->getApproaching().size() > 0) {
            return true;
        }
    }
    return false;
}

void
MSLane::saveState(OutputDevice& out) {
    const bool toRailJunction = myLinks.size() > 0 && (
                                    myEdge->getToJunction()->getType() == SumoXMLNodeType::RAIL_SIGNAL
                                    || myEdge->getToJunction()->getType() == SumoXMLNodeType::RAIL_CROSSING);
    const bool hasVehicles = myVehicles.size() > 0;
    if (hasVehicles || (toRailJunction && hasApproaching())) {
        out.openTag(SUMO_TAG_LANE);
        out.writeAttr(SUMO_ATTR_ID, getID());
        if (hasVehicles) {
            out.openTag(SUMO_TAG_VIEWSETTINGS_VEHICLES);
            out.writeAttr(SUMO_ATTR_VALUE, myVehicles);
            out.closeTag();
        }
        if (toRailJunction) {
            for (const MSLink* link : myLinks) {
                if (link->getApproaching().size() > 0) {
                    out.openTag(SUMO_TAG_LINK);
                    out.writeAttr(SUMO_ATTR_TO, link->getViaLaneOrLane()->getID());
                    for (auto item : link->getApproaching()) {
                        out.openTag(SUMO_TAG_APPROACHING);
                        out.writeAttr(SUMO_ATTR_ID, item.first->getID());
                        out.writeAttr(SUMO_ATTR_ARRIVALTIME, item.second.arrivalTime);
                        out.writeAttr(SUMO_ATTR_ARRIVALSPEED, item.second.arrivalSpeed);
                        out.writeAttr(SUMO_ATTR_DEPARTSPEED, item.second.leaveSpeed);
                        out.writeAttr(SUMO_ATTR_REQUEST, item.second.willPass);
                        out.writeAttr(SUMO_ATTR_ARRIVALSPEEDBRAKING, item.second.arrivalSpeedBraking);
                        out.writeAttr(SUMO_ATTR_WAITINGTIME, item.second.waitingTime);
                        out.writeAttr(SUMO_ATTR_DISTANCE, item.second.dist);
                        if (item.second.latOffset != 0) {
                            out.writeAttr(SUMO_ATTR_POSITION_LAT, item.second.latOffset);
                        }
                        out.closeTag();
                    }
                    out.closeTag();
                }
            }
        }
        out.closeTag();
    }
}

void
MSLane::clearState() {
    myVehicles.clear();
    myParkingVehicles.clear();
    myPartialVehicles.clear();
    myManeuverReservations.clear();
    myBruttoVehicleLengthSum = 0;
    myNettoVehicleLengthSum = 0;
    myBruttoVehicleLengthSumToRemove = 0;
    myNettoVehicleLengthSumToRemove = 0;
    myLeaderInfoTime = SUMOTime_MIN;
    myFollowerInfoTime = SUMOTime_MIN;
    for (MSLink* link : myLinks) {
        link->clearState();
    }
}

void
MSLane::loadState(const std::vector<SUMOVehicle*>& vehs) {
    for (SUMOVehicle* veh : vehs) {
        MSVehicle* v = dynamic_cast<MSVehicle*>(veh);
        v->updateBestLanes(false, this);
        // incorporateVehicle resets the lastActionTime (which has just been loaded from state) so we must restore it
        const SUMOTime lastActionTime = v->getLastActionTime();
        incorporateVehicle(v, v->getPositionOnLane(), v->getSpeed(), v->getLateralPositionOnLane(), myVehicles.end(),
                           MSMoveReminder::NOTIFICATION_LOAD_STATE);
        v->resetActionOffset(lastActionTime - MSNet::getInstance()->getCurrentTimeStep());
        v->processNextStop(v->getSpeed());
    }
}


double
MSLane::getVehicleStopOffset(const MSVehicle* veh) const {
    if (!myLaneStopOffset.isDefined()) {
        return 0;
    }
    if ((myLaneStopOffset.getPermissions() & veh->getVClass()) != 0) {
        return myLaneStopOffset.getOffset();
    } else {
        return 0;
    }
}


const StopOffset&
MSLane::getLaneStopOffsets() const {
    return myLaneStopOffset;
}


void
MSLane::setLaneStopOffset(const StopOffset& stopOffset) {
    myLaneStopOffset = stopOffset;
}


MSLeaderDistanceInfo
MSLane::getFollowersOnConsecutive(const MSVehicle* ego, double backOffset,
                                  bool allSublanes, double searchDist, MinorLinkMode mLinkMode) const {
    assert(ego != 0);
    // get the follower vehicle on the lane to change to
    const double egoPos = backOffset + ego->getVehicleType().getLength();
    const double egoLatDist = ego->getLane()->getRightSideOnEdge() - getRightSideOnEdge();
    const bool getOppositeLeaders = ((ego->getLaneChangeModel().isOpposite() && ego->getLane() == this)
                                     || (!ego->getLaneChangeModel().isOpposite() && &ego->getLane()->getEdge() != &getEdge()));
#ifdef DEBUG_CONTEXT
    if (DEBUG_COND2(ego)) {
        std::cout << SIMTIME << " getFollowers lane=" << getID() << " ego=" << ego->getID()
                  << " backOffset=" << backOffset << " pos=" << egoPos
                  << " allSub=" << allSublanes << " searchDist=" << searchDist << " ignoreMinor=" << mLinkMode
                  << " egoLatDist=" << egoLatDist
                  << " getOppositeLeaders=" << getOppositeLeaders
                  << "\n";
    }
#endif
    MSCriticalFollowerDistanceInfo result(myWidth, allSublanes ? nullptr : ego, allSublanes ? 0 : egoLatDist, getOppositeLeaders);
    if (MSGlobals::gLateralResolution > 0 && egoLatDist == 0) {
        // check whether ego is outside lane bounds far enough so that another vehicle might
        // be between itself and the first "actual" sublane
        // shift the offset so that we "see" this vehicle
        if (ego->getLeftSideOnLane() < -MSGlobals::gLateralResolution) {
            result.setSublaneOffset(int(-ego->getLeftSideOnLane() / MSGlobals::gLateralResolution));
        } else if (ego->getRightSideOnLane() > getWidth() + MSGlobals::gLateralResolution) {
            result.setSublaneOffset(-int((ego->getRightSideOnLane() - getWidth()) / MSGlobals::gLateralResolution));
        }
#ifdef DEBUG_CONTEXT
        if (DEBUG_COND2(ego)) {
            std::cout << SIMTIME << " getFollowers lane=" << getID() << " ego=" << ego->getID()
                      << " egoPosLat=" << ego->getLateralPositionOnLane()
                      << " egoLatDist=" << ego->getLane()->getRightSideOnEdge() - getRightSideOnEdge()
                      << " extraOffset=" << result.getSublaneOffset()
                      << "\n";
        }
#endif
    }
    /// XXX iterate in reverse and abort when there are no more freeSublanes
    for (AnyVehicleIterator last = anyVehiclesBegin(); last != anyVehiclesEnd(); ++last) {
        const MSVehicle* veh = *last;
#ifdef DEBUG_CONTEXT
        if (DEBUG_COND2(ego)) {
            std::cout << "  veh=" << veh->getID() << " lane=" << veh->getLane()->getID() << " pos=" << veh->getPositionOnLane(this) << "\n";
        }
#endif
        if (veh != ego && veh->getPositionOnLane(this) < egoPos) {
            //const double latOffset = veh->getLane()->getRightSideOnEdge() - getRightSideOnEdge();
            const double latOffset = veh->getLatOffset(this);
            double dist = backOffset - veh->getPositionOnLane(this) - veh->getVehicleType().getMinGap();
            if (veh->isBidiOn(this)) {
                dist -= veh->getLength();
            }
            result.addFollower(veh, ego, dist, latOffset);
#ifdef DEBUG_CONTEXT
            if (DEBUG_COND2(ego)) {
                std::cout << "  (1) added veh=" << veh->getID() << " latOffset=" << latOffset << " result=" << result.toString() << "\n";
            }
#endif
        }
    }
#ifdef DEBUG_CONTEXT
    if (DEBUG_COND2(ego)) {
        std::cout << "  result.numFreeSublanes=" << result.numFreeSublanes() << "\n";
    }
#endif
    if (result.numFreeSublanes() > 0) {
        // do a tree search among all follower lanes and check for the most
        // important vehicle (the one requiring the largest reargap)
        // to get a safe bound on the necessary search depth, we need to consider the maximum speed and minimum
        // deceleration of potential follower vehicles
        if (searchDist == -1) {
            searchDist = getMaximumBrakeDist() - backOffset;
#ifdef DEBUG_CONTEXT
            if (DEBUG_COND2(ego)) {
                std::cout << "   computed searchDist=" << searchDist << "\n";
            }
#endif
        }
        std::set<const MSEdge*> egoFurther;
        for (MSLane* further : ego->getFurtherLanes()) {
            egoFurther.insert(&further->getEdge());
        }
        if (ego->getPositionOnLane() < ego->getVehicleType().getLength() && egoFurther.size() == 0
                && ego->getLane()->getLogicalPredecessorLane() != nullptr) {
            // on insertion
            egoFurther.insert(&ego->getLane()->getLogicalPredecessorLane()->getEdge());
        }

        // avoid loops
        std::set<const MSLane*> visited(myEdge->getLanes().begin(), myEdge->getLanes().end());
        if (myEdge->getBidiEdge() != nullptr) {
            visited.insert(myEdge->getBidiEdge()->getLanes().begin(), myEdge->getBidiEdge()->getLanes().end());
        }
        std::vector<MSLane::IncomingLaneInfo> newFound;
        std::vector<MSLane::IncomingLaneInfo> toExamine = myIncomingLanes;
        while (toExamine.size() != 0) {
            for (std::vector<MSLane::IncomingLaneInfo>::iterator it = toExamine.begin(); it != toExamine.end(); ++it) {
                MSLane* next = (*it).lane;
                searchDist = MAX2(searchDist, next->getMaximumBrakeDist() - backOffset);
                MSLeaderInfo first = next->getFirstVehicleInformation(nullptr, 0, false, std::numeric_limits<double>::max(), false);
                MSLeaderInfo firstFront = next->getFirstVehicleInformation(nullptr, 0, true);
#ifdef DEBUG_CONTEXT
                if (DEBUG_COND2(ego)) {
                    std::cout << "   next=" << next->getID() << " seen=" << (*it).length << " first=" << first.toString() << " firstFront=" << firstFront.toString() << " backOffset=" << backOffset << "\n";
                    gDebugFlag1 = true; // for calling getLeaderInfo
                }
#endif
                if (backOffset + (*it).length - next->getLength() < 0
                        && egoFurther.count(&next->getEdge()) != 0
                   )  {
                    // check for junction foes that would interfere with lane changing
                    // @note: we are passing the back of ego as its front position so
                    //        we need to add this back to the returned gap
                    const MSLink::LinkLeaders linkLeaders = (*it).viaLink->getLeaderInfo(ego, -backOffset);
                    for (const auto& ll : linkLeaders) {
                        if (ll.vehAndGap.first != nullptr) {
                            const bool bidiFoe = (*it).viaLink->getLane() == ll.vehAndGap.first->getLane()->getNormalPredecessorLane()->getBidiLane();
                            const bool egoIsLeader = !bidiFoe && ll.vehAndGap.first->isLeader((*it).viaLink, ego, ll.vehAndGap.second);
                            // if ego is leader the returned gap still assumes that ego follows the leader
                            // if the foe vehicle follows ego we need to deduce that gap
                            const double gap = (egoIsLeader
                                                ? -ll.vehAndGap.second - ll.vehAndGap.first->getVehicleType().getLengthWithGap() - ego->getVehicleType().getMinGap()
                                                : ll.vehAndGap.second + ego->getVehicleType().getLength());
                            result.addFollower(ll.vehAndGap.first, ego, gap);
#ifdef DEBUG_CONTEXT
                            if (DEBUG_COND2(ego)) {
                                std::cout << SIMTIME << " ego=" << ego->getID() << "    link=" << (*it).viaLink->getViaLaneOrLane()->getID()
                                          << " (3) added veh=" << Named::getIDSecure(ll.vehAndGap.first)
                                          << " gap=" << ll.vehAndGap.second << " dtC=" << ll.distToCrossing
                                          << " bidiFoe=" << bidiFoe
                                          << " egoIsLeader=" << egoIsLeader << " gap2=" << gap
                                          << "\n";
                            }
#endif
                        }
                    }
                }
#ifdef DEBUG_CONTEXT
                if (DEBUG_COND2(ego)) {
                    gDebugFlag1 = false;
                }
#endif

                for (int i = 0; i < first.numSublanes(); ++i) {
                    const MSVehicle* v = first[i] == ego ? firstFront[i] : first[i];
                    double agap = 0;

                    if (v != nullptr && v != ego) {
                        if (!v->isFrontOnLane(next)) {
                            // the front of v is already on divergent trajectory from the ego vehicle
                            // for which this method is called (in the context of MSLaneChanger).
                            // Therefore, technically v is not a follower but only an obstruction and
                            // the gap is not between the front of v and the back of ego
                            // but rather between the flank of v and the back of ego.
                            agap = (*it).length - next->getLength() + backOffset;
                            if (MSGlobals::gUsingInternalLanes) {
                                // ego should have left the intersection still occupied by v
                                agap -= v->getVehicleType().getMinGap();
                            }
#ifdef DEBUG_CONTEXT
                            if (DEBUG_COND2(ego)) {
                                std::cout << "    agap1=" << agap << "\n";
                            }
#endif
                            const bool differentEdge = &v->getLane()->getEdge() != &ego->getLane()->getEdge();
                            if (agap > 0 && differentEdge) {
                                // Only if ego overlaps we treat v as if it were a real follower
                                // Otherwise we ignore it and look for another follower
                                if (!getOppositeLeaders) {
                                    // even if the vehicle is not a real
                                    // follower, it still forms a real
                                    // obstruction in opposite direction driving
                                    v = firstFront[i];
                                    if (v != nullptr && v != ego) {
                                        agap = (*it).length - v->getPositionOnLane() + backOffset - v->getVehicleType().getMinGap();
                                    } else {
                                        v = nullptr;
                                    }
                                }
                            } else if (differentEdge && result.hasVehicle(v)) {
                                // ignore this vehicle as it was already seen on another lane
                                agap = 0;
                            }
                        } else {
                            if (next->getBidiLane() != nullptr && v->isBidiOn(next)) {
                                agap = v->getPositionOnLane() + backOffset - v->getVehicleType().getLengthWithGap();
                            } else {
                                agap = (*it).length - v->getPositionOnLane() + backOffset - v->getVehicleType().getMinGap();
                            }
                            if (!(*it).viaLink->havePriority() && egoFurther.count(&(*it).lane->getEdge()) == 0
                                    && ego->isOnRoad() // during insertion, this can lead to collisions because ego's further lanes are not set (see #3053)
                                    && !ego->getLaneChangeModel().isOpposite()
                                    && v->getSpeed() < SUMO_const_haltingSpeed
                               ) {
                                // if v is stopped on a minor side road it should not block lane changing
                                agap = MAX2(agap, 0.0);
                            }
                        }
                        result.addFollower(v, ego, agap, 0, i);
#ifdef DEBUG_CONTEXT
                        if (DEBUG_COND2(ego)) {
                            std::cout << " (2) added veh=" << Named::getIDSecure(v) << " agap=" << agap << " next=" << next->getID() << " result=" << result.toString() << "\n";
                        }
#endif
                    }
                }
                if ((*it).length < searchDist) {
                    const std::vector<MSLane::IncomingLaneInfo>& followers = next->getIncomingLanes();
                    for (std::vector<MSLane::IncomingLaneInfo>::const_iterator j = followers.begin(); j != followers.end(); ++j) {
                        if (visited.find((*j).lane) == visited.end() && (((*j).viaLink->havePriority() && !(*j).viaLink->isTurnaround())
                                || mLinkMode == MinorLinkMode::FOLLOW_ALWAYS
                                || (mLinkMode == MinorLinkMode::FOLLOW_ONCOMING && (*j).viaLink->getDirection() == LinkDirection::STRAIGHT))) {
                            visited.insert((*j).lane);
                            MSLane::IncomingLaneInfo ili;
                            ili.lane = (*j).lane;
                            ili.length = (*j).length + (*it).length;
                            ili.viaLink = (*j).viaLink;
                            newFound.push_back(ili);
                        }
                    }
                }
            }
            toExamine.clear();
            swap(newFound, toExamine);
        }
        //return result;

    }
    return result;
}


void
MSLane::getLeadersOnConsecutive(double dist, double seen, double speed, const MSVehicle* ego,
                                const std::vector<MSLane*>& bestLaneConts, MSLeaderDistanceInfo& result,
                                bool oppositeDirection) const {
    if (seen > dist && !(isInternal() && MSGlobals::gComputeLC)) {
        return;
    }
    // check partial vehicles (they might be on a different route and thus not
    // found when iterating along bestLaneConts)
    for (VehCont::const_iterator it = myPartialVehicles.begin(); it != myPartialVehicles.end(); ++it) {
        MSVehicle* veh = *it;
        if (!veh->isFrontOnLane(this)) {
            result.addLeader(veh, seen, veh->getLatOffset(this));
        } else {
            break;
        }
    }
#ifdef DEBUG_CONTEXT
    if (DEBUG_COND2(ego)) {
        gDebugFlag1 = true;
    }
#endif
    const MSLane* nextLane = this;
    int view = 1;
    // loop over following lanes
    while ((seen < dist || nextLane->isInternal()) && result.numFreeSublanes() > 0) {
        // get the next link used
        bool nextInternal = false;
        if (oppositeDirection) {
            if (view >= (int)bestLaneConts.size()) {
                break;
            }
            nextLane = bestLaneConts[view];
        } else {
            std::vector<MSLink*>::const_iterator link = succLinkSec(*ego, view, *nextLane, bestLaneConts);
            if (nextLane->isLinkEnd(link)) {
                break;
            }
            // check for link leaders
            const MSLink::LinkLeaders linkLeaders = (*link)->getLeaderInfo(ego, seen);
            if (linkLeaders.size() > 0) {
                const MSLink::LinkLeader ll = linkLeaders[0];
                MSVehicle* veh = ll.vehAndGap.first;
                // in the context of lane changing all junction leader candidates must be respected
                if (veh != 0 && (ego->isLeader(*link, veh, ll.vehAndGap.second)
                                 || (MSGlobals::gComputeLC
                                     && veh->getPosition().distanceTo2D(ego->getPosition()) - veh->getVehicleType().getMinGap() - ego->getVehicleType().getLength()
                                     < veh->getCarFollowModel().brakeGap(veh->getSpeed())))) {
#ifdef DEBUG_CONTEXT
                    if (DEBUG_COND2(ego)) {
                        std::cout << "   linkleader=" << veh->getID() << " gap=" << ll.vehAndGap.second << " leaderOffset=" << ll.latOffset << " flags=" << ll.llFlags << "\n";
                    }
#endif
                    if (ll.sameTarget() || ll.sameSource()) {
                        result.addLeader(veh, ll.vehAndGap.second, ll.latOffset);
                    } else {
                        // add link leader to all sublanes and return
                        for (int i = 0; i < result.numSublanes(); ++i) {
                            result.addLeader(veh, ll.vehAndGap.second, 0, i);
                        }
                    }
#ifdef DEBUG_CONTEXT
                    gDebugFlag1 = false;
#endif
                    return;
                } // XXX else, deal with pedestrians
            }
            nextInternal = (*link)->getViaLane() != nullptr;
            nextLane = (*link)->getViaLaneOrLane();
            if (nextLane == nullptr) {
                break;
            }
        }

        MSLeaderInfo leaders = nextLane->getLastVehicleInformation(nullptr, 0, 0, false);
#ifdef DEBUG_CONTEXT
        if (DEBUG_COND2(ego)) {
            std::cout << SIMTIME << " getLeadersOnConsecutive lane=" << getID() << " nextLane=" << nextLane->getID() << " leaders=" << leaders.toString() << "\n";
        }
#endif
        // @todo check alignment issues if the lane width changes
        const int iMax = MIN2(leaders.numSublanes(), result.numSublanes());
        for (int i = 0; i < iMax; ++i) {
            const MSVehicle* veh = leaders[i];
            if (veh != nullptr) {
#ifdef DEBUG_CONTEXT
                if (DEBUG_COND2(ego)) std::cout << "   lead=" << veh->getID()
                                                    << " seen=" << seen
                                                    << " minGap=" << ego->getVehicleType().getMinGap()
                                                    << " backPos=" << veh->getBackPositionOnLane(nextLane)
                                                    << " gap=" << seen - ego->getVehicleType().getMinGap() + veh->getBackPositionOnLane(nextLane)
                                                    << "\n";
#endif
                result.addLeader(veh, seen - ego->getVehicleType().getMinGap() + veh->getBackPositionOnLane(nextLane), 0, i);
            }
        }

        if (nextLane->getVehicleMaxSpeed(ego) < speed) {
            dist = ego->getCarFollowModel().brakeGap(nextLane->getVehicleMaxSpeed(ego));
        }
        seen += nextLane->getLength();
        if (!nextInternal) {
            view++;
        }
    }
#ifdef DEBUG_CONTEXT
    gDebugFlag1 = false;
#endif
}


void
MSLane::addLeaders(const MSVehicle* vehicle, double vehPos, MSLeaderDistanceInfo& result, bool opposite) {
    // if there are vehicles on the target lane with the same position as ego,
    // they may not have been added to 'ahead' yet
#ifdef DEBUG_SURROUNDING
    if (DEBUG_COND || DEBUG_COND2(vehicle)) {
        std::cout << " addLeaders lane=" << getID() << " veh=" << vehicle->getID() << " vehPos=" << vehPos << " opposite=" << opposite << "\n";
    }
#endif
    const MSLeaderInfo& aheadSamePos = getLastVehicleInformation(nullptr, 0, vehPos, false);
    for (int i = 0; i < aheadSamePos.numSublanes(); ++i) {
        const MSVehicle* veh = aheadSamePos[i];
        if (veh != nullptr && veh != vehicle) {
            const double gap = veh->getBackPositionOnLane(this) - vehPos - vehicle->getVehicleType().getMinGap();
#ifdef DEBUG_SURROUNDING
            if (DEBUG_COND || DEBUG_COND2(vehicle)) {
                std::cout << " further lead=" << veh->getID() << " leadBack=" << veh->getBackPositionOnLane(this) << " gap=" << gap << "\n";
            }
#endif
            result.addLeader(veh, gap, 0, i);
        }
    }

    if (result.numFreeSublanes() > 0) {
        double seen = vehicle->getLane()->getLength() - vehPos;
        double speed = vehicle->getSpeed();
        // leader vehicle could be link leader on the next junction
        double dist = MAX2(vehicle->getCarFollowModel().brakeGap(speed), 10.0) + vehicle->getVehicleType().getMinGap();
        if (getBidiLane() != nullptr) {
            dist = MAX2(dist, myMaxSpeed * 20);
        }
        // check for link leaders when on internal
        if (seen > dist && !(isInternal() && MSGlobals::gComputeLC)) {
#ifdef DEBUG_SURROUNDING
            if (DEBUG_COND || DEBUG_COND2(vehicle)) {
                std::cout << " aborting forward search. dist=" << dist << " seen=" << seen << "\n";
            }
#endif
            return;
        }
#ifdef DEBUG_SURROUNDING
        if (DEBUG_COND || DEBUG_COND2(vehicle)) {
            std::cout << " add consecutive before=" << result.toString() << " seen=" << seen << " dist=" << dist;
        }
#endif
        if (opposite) {
            const std::vector<MSLane*> bestLaneConts = vehicle->getUpstreamOppositeLanes();
#ifdef DEBUG_SURROUNDING
            if (DEBUG_COND || DEBUG_COND2(vehicle)) {
                std::cout << " upstreamOpposite=" << toString(bestLaneConts);
            }
#endif
            getLeadersOnConsecutive(dist, seen, speed, vehicle, bestLaneConts, result, opposite);
        } else {
            const std::vector<MSLane*>& bestLaneConts = vehicle->getBestLanesContinuation(this);
            getLeadersOnConsecutive(dist, seen, speed, vehicle, bestLaneConts, result);
        }
#ifdef DEBUG_SURROUNDING
        if (DEBUG_COND || DEBUG_COND2(vehicle)) {
            std::cout << " after=" << result.toString() << "\n";
        }
#endif
    }
}


MSVehicle*
MSLane::getPartialBehind(const MSVehicle* ego) const {
    for (VehCont::const_reverse_iterator i = myPartialVehicles.rbegin(); i != myPartialVehicles.rend(); ++i) {
        MSVehicle* veh = *i;
        if (veh->isFrontOnLane(this)
                && veh != ego
                && veh->getPositionOnLane() <= ego->getPositionOnLane()) {
#ifdef DEBUG_CONTEXT
            if (DEBUG_COND2(ego)) {
                std::cout << SIMTIME << " getPartialBehind lane=" << getID() << " ego=" << ego->getID() << " found=" << veh->getID() << "\n";
            }
#endif
            return veh;
        }
    }
#ifdef DEBUG_CONTEXT
    if (DEBUG_COND2(ego)) {
        std::cout << SIMTIME << " getPartialBehind lane=" << getID() << " ego=" << ego->getID() << " nothing found. partials=" << toString(myPartialVehicles) << "\n";
    }
#endif
    return nullptr;
}

MSLeaderInfo
MSLane::getPartialBeyond() const {
    MSLeaderInfo result(myWidth);
    for (VehCont::const_iterator it = myPartialVehicles.begin(); it != myPartialVehicles.end(); ++it) {
        MSVehicle* veh = *it;
        if (!veh->isFrontOnLane(this)) {
            result.addLeader(veh, false, veh->getLatOffset(this));
        } else {
            break;
        }
    }
    return result;
}


std::set<MSVehicle*>
MSLane::getSurroundingVehicles(double startPos, double downstreamDist, double upstreamDist, std::shared_ptr<LaneCoverageInfo> checkedLanes) const {
    assert(checkedLanes != nullptr);
    if (checkedLanes->find(this) != checkedLanes->end()) {
#ifdef DEBUG_SURROUNDING
        std::cout << "Skipping previously scanned lane: " << getID() << std::endl;
#endif
        return std::set<MSVehicle*>();
    } else {
        // Add this lane's coverage to the lane coverage info
        (*checkedLanes)[this] = std::make_pair(MAX2(0.0, startPos - upstreamDist), MIN2(startPos + downstreamDist, getLength()));
    }
#ifdef DEBUG_SURROUNDING
    std::cout << "Scanning on lane " << myID << "(downstr. " << downstreamDist << ", upstr. " << upstreamDist << ", startPos " << startPos << "): " << std::endl;
#endif
    std::set<MSVehicle*> foundVehicles = getVehiclesInRange(MAX2(0., startPos - upstreamDist), MIN2(myLength, startPos + downstreamDist));
    if (startPos < upstreamDist) {
        // scan incoming lanes
        for (const IncomingLaneInfo& incomingInfo : getIncomingLanes()) {
            MSLane* incoming = incomingInfo.lane;
#ifdef DEBUG_SURROUNDING
            std::cout << "Checking on incoming: " << incoming->getID() << std::endl;
            if (checkedLanes->find(incoming) != checkedLanes->end()) {
                std::cout << "Skipping previous: " << incoming->getID() << std::endl;
            }
#endif
            std::set<MSVehicle*> newVehs = incoming->getSurroundingVehicles(incoming->getLength(), 0.0, upstreamDist - startPos, checkedLanes);
            foundVehicles.insert(newVehs.begin(), newVehs.end());
        }
    }

    if (getLength() < startPos + downstreamDist) {
        // scan successive lanes
        const std::vector<MSLink*>& lc = getLinkCont();
        for (MSLink* l : lc) {
#ifdef DEBUG_SURROUNDING
            std::cout << "Checking on outgoing: " << l->getViaLaneOrLane()->getID() << std::endl;
#endif
            std::set<MSVehicle*> newVehs = l->getViaLaneOrLane()->getSurroundingVehicles(0.0, downstreamDist - (myLength - startPos), upstreamDist, checkedLanes);
            foundVehicles.insert(newVehs.begin(), newVehs.end());
        }
    }
#ifdef DEBUG_SURROUNDING
    std::cout << "On lane (2) " << myID << ": \nFound vehicles: " << std::endl;
    for (MSVehicle* v : foundVehicles) {
        std::cout << v->getID() << " pos = " << v->getPositionOnLane() << std::endl;
    }
#endif
    return foundVehicles;
}


std::set<MSVehicle*>
MSLane::getVehiclesInRange(const double a, const double b) const {
    std::set<MSVehicle*> res;
    const VehCont& vehs = getVehiclesSecure();

    if (!vehs.empty()) {
        for (MSVehicle* const veh : vehs) {
            if (veh->getPositionOnLane() >= a) {
                if (veh->getBackPositionOnLane() > b) {
                    break;
                }
                res.insert(veh);
            }
        }
    }
    releaseVehicles();
    return res;
}


std::vector<const MSJunction*>
MSLane::getUpcomingJunctions(double pos, double range, const std::vector<MSLane*>& contLanes) const {
    // set of upcoming junctions and the corresponding conflict links
    std::vector<const MSJunction*> junctions;
    for (auto l : getUpcomingLinks(pos, range, contLanes)) {
        junctions.insert(junctions.end(), l->getJunction());
    }
    return junctions;
}


std::vector<const MSLink*>
MSLane::getUpcomingLinks(double pos, double range, const std::vector<MSLane*>& contLanes) const {
#ifdef DEBUG_SURROUNDING
    std::cout << "getUpcoming links on lane '" << getID() << "' with pos=" << pos
              << " range=" << range << std::endl;
#endif
    // set of upcoming junctions and the corresponding conflict links
    std::vector<const MSLink*> links;

    // Currently scanned lane
    const MSLane* lane = this;

    // continuation lanes for the vehicle
    std::vector<MSLane*>::const_iterator contLanesIt = contLanes.begin();
    // scanned distance so far
    double dist = 0.0;
    // link to be crossed by the vehicle
    const MSLink* link = nullptr;
    if (lane->isInternal()) {
        assert(*contLanesIt == nullptr); // is called with vehicle's bestLane structure
        link = lane->getEntryLink();
        links.insert(links.end(), link);
        dist += link->getInternalLengthsAfter();
        // next non-internal lane behind junction
        lane = link->getLane();
        pos = 0.0;
        assert(*(contLanesIt + 1) == lane);
    }
    while (++contLanesIt != contLanes.end()) {
        assert(!lane->isInternal());
        dist += lane->getLength() - pos;
        pos = 0.;
#ifdef DEBUG_SURROUNDING
        std::cout << "Distance until end of lane '" << lane->getID() << "' is " << dist << "." << std::endl;
#endif
        if (dist > range) {
            break;
        }
        link = lane->getLinkTo(*contLanesIt);
        if (link != nullptr) {
            links.insert(links.end(), link);
        }
        lane = *contLanesIt;
    }
    return links;
}


MSLane*
MSLane::getOpposite() const {
    return myOpposite;
}


MSLane*
MSLane::getParallelOpposite() const {
    return myEdge->getLanes().back()->getOpposite();
}


double
MSLane::getOppositePos(double pos) const {
    return MAX2(0., myLength - pos);
}

std::pair<MSVehicle* const, double>
MSLane::getFollower(const MSVehicle* ego, double egoPos, double dist, MinorLinkMode mLinkMode) const {
    for (AnyVehicleIterator first = anyVehiclesUpstreamBegin(); first != anyVehiclesUpstreamEnd(); ++first) {
        // XXX refactor leaderInfo to use a const vehicle all the way through the call hierarchy
        MSVehicle* pred = (MSVehicle*)*first;
#ifdef DEBUG_CONTEXT
        if (DEBUG_COND2(ego)) {
            std::cout << "   getFollower lane=" << getID() << " egoPos=" << egoPos << " pred=" << pred->getID() << " predPos=" << pred->getPositionOnLane(this) << "\n";
        }
#endif
        if (pred != ego && pred->getPositionOnLane(this) < egoPos) {
            return std::pair<MSVehicle* const, double>(pred, egoPos - pred->getPositionOnLane(this) - ego->getVehicleType().getLength() - pred->getVehicleType().getMinGap());
        }
    }
    const double backOffset = egoPos - ego->getVehicleType().getLength();
    if (dist > 0 && backOffset > dist) {
        return std::make_pair(nullptr, -1);
    }
    const MSLeaderDistanceInfo followers = getFollowersOnConsecutive(ego, backOffset, true,  dist, mLinkMode);
    CLeaderDist result = followers.getClosest();
    return std::make_pair(const_cast<MSVehicle*>(result.first), result.second);
}

std::pair<MSVehicle* const, double>
MSLane::getOppositeLeader(const MSVehicle* ego, double dist, bool oppositeDir, MinorLinkMode mLinkMode) const {
#ifdef DEBUG_OPPOSITE
    if (DEBUG_COND2(ego)) std::cout << SIMTIME << " getOppositeLeader lane=" << getID()
                                        << " ego=" << ego->getID()
                                        << " pos=" << ego->getPositionOnLane()
                                        << " posOnOpposite=" << getOppositePos(ego->getPositionOnLane())
                                        << " dist=" << dist
                                        << " oppositeDir=" << oppositeDir
                                        << "\n";
#endif
    if (!oppositeDir) {
        return getLeader(ego, getOppositePos(ego->getPositionOnLane()), ego->getBestLanesContinuation(this));
    } else {
        const double egoLength = ego->getVehicleType().getLength();
        const double egoPos = ego->getLaneChangeModel().isOpposite() ? ego->getPositionOnLane() : getOppositePos(ego->getPositionOnLane());
        std::pair<MSVehicle* const, double> result = getFollower(ego, egoPos + egoLength, dist, mLinkMode);
        if (result.first != nullptr) {
            result.second -= ego->getVehicleType().getMinGap();
            if (result.first->getLaneChangeModel().isOpposite()) {
                result.second -= result.first->getVehicleType().getLength();
            }
        }
        return result;
    }
}


std::pair<MSVehicle* const, double>
MSLane::getOppositeFollower(const MSVehicle* ego) const {
#ifdef DEBUG_OPPOSITE
    if (DEBUG_COND2(ego)) std::cout << SIMTIME << " getOppositeFollower lane=" << getID()
                                        << " ego=" << ego->getID()
                                        << " backPos=" << ego->getBackPositionOnLane()
                                        << " posOnOpposite=" << getOppositePos(ego->getBackPositionOnLane())
                                        << "\n";
#endif
    if (ego->getLaneChangeModel().isOpposite()) {
        std::pair<MSVehicle* const, double> result = getFollower(ego, getOppositePos(ego->getPositionOnLane()), -1, MinorLinkMode::FOLLOW_NEVER);
        return result;
    } else {
        double vehPos = getOppositePos(ego->getPositionOnLane() - ego->getVehicleType().getLength());
        std::pair<MSVehicle*, double> result = getLeader(ego, vehPos, std::vector<MSLane*>());
        double dist = getMaximumBrakeDist() + getOppositePos(ego->getPositionOnLane() - getLength());
        MSLane* next = const_cast<MSLane*>(this);
        while (result.first == nullptr && dist > 0) {
            // cannot call getLeadersOnConsecutive because succLinkSec doesn't
            // uses the vehicle's route and doesn't work on the opposite side
            vehPos -= next->getLength();
            next = next->getCanonicalSuccessorLane();
            if (next == nullptr) {
                break;
            }
            dist -= next->getLength();
            result = next->getLeader(ego, vehPos, std::vector<MSLane*>());
        }
        if (result.first != nullptr) {
            if (result.first->getLaneChangeModel().isOpposite()) {
                result.second -= result.first->getVehicleType().getLength();
            } else {
                if (result.second > POSITION_EPS) {
                    // follower can be safely ignored since it is going the other way
                    return std::make_pair(static_cast<MSVehicle*>(nullptr), -1);
                }
            }
        }
        return result;
    }
}

void
MSLane::initCollisionAction(const OptionsCont& oc, const std::string& option, CollisionAction& myAction) {
    const std::string action = oc.getString(option);
    if (action == "none") {
        myAction = COLLISION_ACTION_NONE;
    } else if (action == "warn") {
        myAction = COLLISION_ACTION_WARN;
    } else if (action == "teleport") {
        myAction = COLLISION_ACTION_TELEPORT;
    } else if (action == "remove") {
        myAction = COLLISION_ACTION_REMOVE;
    } else {
        WRITE_ERROR(TLF("Invalid % '%'.", option, action));
    }
}

void
MSLane::initCollisionOptions(const OptionsCont& oc) {
    initCollisionAction(oc, "collision.action", myCollisionAction);
    initCollisionAction(oc, "intermodal-collision.action", myIntermodalCollisionAction);
    myCheckJunctionCollisions = oc.getBool("collision.check-junctions");
    myCheckJunctionCollisionMinGap = oc.getFloat("collision.check-junctions.mingap");
    myCollisionStopTime = string2time(oc.getString("collision.stoptime"));
    myIntermodalCollisionStopTime = string2time(oc.getString("intermodal-collision.stoptime"));
    myCollisionMinGapFactor = oc.getFloat("collision.mingap-factor");
    myExtrapolateSubstepDepart = oc.getBool("extrapolate-departpos");
}


void
MSLane::setPermissions(SVCPermissions permissions, long long transientID) {
    if (transientID == CHANGE_PERMISSIONS_PERMANENT) {
        myPermissions = permissions;
        myOriginalPermissions = permissions;
    } else {
        myPermissionChanges[transientID] = permissions;
        resetPermissions(CHANGE_PERMISSIONS_PERMANENT);
    }
}


void
MSLane::resetPermissions(long long transientID) {
    myPermissionChanges.erase(transientID);
    if (myPermissionChanges.empty()) {
        myPermissions = myOriginalPermissions;
    } else {
        // combine all permission changes
        myPermissions = SVCAll;
        for (const auto& item : myPermissionChanges) {
            myPermissions &= item.second;
        }
    }
}


bool
MSLane::hadPermissionChanges() const {
    return !myPermissionChanges.empty();
}


void
MSLane::setChangeLeft(SVCPermissions permissions) {
    myChangeLeft = permissions;
}


void
MSLane::setChangeRight(SVCPermissions permissions) {
    myChangeRight = permissions;
}


bool
MSLane::hasPedestrians() const {
    MSNet* const net = MSNet::getInstance();
    return net->hasPersons() && net->getPersonControl().getMovementModel()->hasPedestrians(this);
}


PersonDist
MSLane::nextBlocking(double minPos, double minRight, double maxLeft, double stopTime, bool bidi) const {
    return MSNet::getInstance()->getPersonControl().getMovementModel()->nextBlocking(this, minPos, minRight, maxLeft, stopTime, bidi);
}


bool
MSLane::checkForPedestrians(const MSVehicle* aVehicle, double& speed, double& dist,  double pos, bool patchSpeed) const {
    if (getEdge().getPersons().size() > 0 && hasPedestrians()) {
#ifdef DEBUG_INSERTION
        if (DEBUG_COND2(aVehicle)) {
            std::cout << SIMTIME << " check for pedestrians on lane=" << getID() << " pos=" << pos << "\n";
        }
#endif
        PersonDist leader = nextBlocking(pos - aVehicle->getVehicleType().getLength(),
                                         aVehicle->getRightSideOnLane(), aVehicle->getRightSideOnLane() + aVehicle->getVehicleType().getWidth(), ceil(speed / aVehicle->getCarFollowModel().getMaxDecel()));
        if (leader.first != 0) {
            const double gap = leader.second - aVehicle->getVehicleType().getLengthWithGap();
            const double stopSpeed = aVehicle->getCarFollowModel().stopSpeed(aVehicle, speed, gap, MSCFModel::CalcReason::FUTURE);
            if ((gap < 0 && (aVehicle->getInsertionChecks() & ((int)InsertionCheck::COLLISION | (int)InsertionCheck::PEDESTRIAN)) != 0)
                    || checkFailure(aVehicle, speed, dist, stopSpeed, patchSpeed, "", InsertionCheck::PEDESTRIAN)) {
                // we may not drive with the given velocity - we crash into the pedestrian
#ifdef DEBUG_INSERTION
                if (DEBUG_COND2(aVehicle)) std::cout << SIMTIME
                                                         << " isInsertionSuccess lane=" << getID()
                                                         << " veh=" << aVehicle->getID()
                                                         << " pos=" << pos
                                                         << " posLat=" << aVehicle->getLateralPositionOnLane()
                                                         << " patchSpeed=" << patchSpeed
                                                         << " speed=" << speed
                                                         << " stopSpeed=" << stopSpeed
                                                         << " pedestrianLeader=" << leader.first->getID()
                                                         << " failed (@796)!\n";
#endif
                return false;
            }
        }
    }
    double backLength = aVehicle->getLength() - pos;
    if (backLength > 0 && MSNet::getInstance()->hasPersons()) {
        // look upstream for pedestrian crossings
        const MSLane* prev = getLogicalPredecessorLane();
        const MSLane* cur = this;
        while (backLength > 0 && prev != nullptr) {
            const MSLink* link = prev->getLinkTo(cur);
            if (link->hasFoeCrossing()) {
                for (const MSLane* foe : link->getFoeLanes()) {
                    if (foe->isCrossing() && (foe->hasPedestrians() ||
                                              (foe->getIncomingLanes()[0].viaLink->getApproachingPersons() != nullptr
                                               && foe->getIncomingLanes()[0].viaLink->getApproachingPersons()->size() > 0))) {
#ifdef DEBUG_INSERTION
                        if (DEBUG_COND2(aVehicle)) std::cout << SIMTIME
                                                                 << " isInsertionSuccess lane=" << getID()
                                                                 << " veh=" << aVehicle->getID()
                                                                 << " pos=" << pos
                                                                 << " backCrossing=" << foe->getID()
                                                                 << " peds=" << joinNamedToString(foe->getEdge().getPersons(), " ")
                                                                 << " approaching=" << foe->getIncomingLanes()[0].viaLink->getApproachingPersons()->size()
                                                                 << " failed (@4550)!\n";
#endif
                        return false;
                    }
                }
            }
            backLength -= prev->getLength();
            cur = prev;
            prev = prev->getLogicalPredecessorLane();
        }
    }
    return true;
}


void
MSLane::initRNGs(const OptionsCont& oc) {
    myRNGs.clear();
    const int numRNGs = oc.getInt("thread-rngs");
    const bool random = oc.getBool("random");
    int seed = oc.getInt("seed");
    myRNGs.reserve(numRNGs); // this is needed for stable pointers on debugging
    for (int i = 0; i < numRNGs; i++) {
        myRNGs.push_back(SumoRNG("lanes_" + toString(i)));
        RandHelper::initRand(&myRNGs.back(), random, seed++);
    }
}

void
MSLane::saveRNGStates(OutputDevice& out) {
    for (int i = 0; i < getNumRNGs(); i++) {
        out.openTag(SUMO_TAG_RNGLANE);
        out.writeAttr(SUMO_ATTR_INDEX, i);
        out.writeAttr(SUMO_ATTR_STATE, RandHelper::saveState(&myRNGs[i]));
        out.closeTag();
    }
}

void
MSLane::loadRNGState(int index, const std::string& state) {
    if (index >= getNumRNGs()) {
        throw ProcessError(TLF("State was saved with more than % threads. Change the number of threads or do not load RNG state", toString(getNumRNGs())));
    }
    RandHelper::loadState(state, &myRNGs[index]);
}


MSLane*
MSLane::getBidiLane() const {
    return myBidiLane;
}


bool
MSLane::mustCheckJunctionCollisions() const {
    return myCheckJunctionCollisions && myEdge->isInternal() && (
               myLinks.front()->getFoeLanes().size() > 0
               || myLinks.front()->getWalkingAreaFoe() != nullptr
               || myLinks.front()->getWalkingAreaFoeExit() != nullptr);
}


double
MSLane::getSpaceTillLastStanding(const MSVehicle* ego, bool& foundStopped) const {
    /// @todo if ego isn't on this lane, we could use a cached value
    double lengths = 0;
    for (const MSVehicle* last : myVehicles) {
        if (last->getSpeed() < SUMO_const_haltingSpeed && !last->getLane()->getEdge().isRoundabout()
                && last != ego
                // @todo recheck
                && last->isFrontOnLane(this)) {
            foundStopped = true;
            const double lastBrakeGap = last->getCarFollowModel().brakeGap(last->getSpeed());
            const double ret = last->getBackPositionOnLane() + lastBrakeGap - lengths;
            return ret;
        }
        if (MSGlobals::gSublane && ego->getVehicleType().getWidth() + last->getVehicleType().getWidth() < getWidth()) {
            lengths += last->getVehicleType().getLengthWithGap() * (last->getVehicleType().getWidth() + last->getVehicleType().getMinGapLat()) / getWidth();
        } else {
            lengths += last->getVehicleType().getLengthWithGap();
        }
    }
    return getLength() - lengths;
}


bool
MSLane::allowsVehicleClass(SUMOVehicleClass vclass, int routingMode) const {
    return (((routingMode & libsumo::ROUTING_MODE_IGNORE_TRANSIENT_PERMISSIONS) ? myOriginalPermissions : myPermissions) & vclass) == vclass;
}

/****************************************************************************/
