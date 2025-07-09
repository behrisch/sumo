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
/// @file    MSDriveWay.cpp
/// @author  Jakob Erdmann
/// @date    December 2021
///
// A sequende of rail tracks (lanes) that may be used as a "set route" (Fahrstraße)
/****************************************************************************/
#include <config.h>
#include <cassert>
#include <utility>

#include <utils/xml/SUMOSAXAttributes.h>
#include <utils/common/StringUtils.h>
#include <microsim/MSStop.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLink.h>
#include <microsim/MSNet.h>
#include <microsim/MSVehicleControl.h>
#include <microsim/MSJunctionLogic.h>
#include <mesosim/MELoop.h>
#include "MSRailSignal.h"
#include "MSDriveWay.h"
#include "MSRailSignalControl.h"

#define DRIVEWAY_SANITY_CHECK
//#define SUBDRIVEWAY_WARN_NOCONFLICT

//#define DEBUG_BUILD_DRIVEWAY
//#define DEBUG_BUILD_SUBDRIVEWAY
//#define DEBUG_ADD_FOES
//#define DEBUG_BUILD_SIDINGS
//#define DEBUG_DRIVEWAY_BUILDROUTE
//#define DEBUG_CHECK_FLANKS
//#define DEBUG_SIGNALSTATE_PRIORITY
//#define DEBUG_SIGNALSTATE
//#define DEBUG_FIND_PROTECTION
//#define DEBUG_MOVEREMINDER
//#define DEBUG_MATCH

#define DEBUG_HELPER(obj) ((obj) != nullptr && (obj)->isSelected())
//#define DEBUG_HELPER(obj) ((obj)->getID() == "")
//#define DEBUG_HELPER(obj) (true)

//#define DEBUG_COND_DW (dw->myNumericalID == 5)
#define DEBUG_COND_DW (false)

// ===========================================================================
// static value definitions
// ===========================================================================
int MSDriveWay::myGlobalDriveWayIndex(0);
std::set<const MSEdge*> MSDriveWay::myBlockLengthWarnings;
bool MSDriveWay::myWriteVehicles(false);
std::map<const MSLink*, std::vector<MSDriveWay*> > MSDriveWay::mySwitchDriveWays;
std::map<const MSEdge*, std::vector<MSDriveWay*> > MSDriveWay::myReversalDriveWays;
std::map<const MSEdge*, std::vector<MSDriveWay*>, ComparatorNumericalIdLess> MSDriveWay::myDepartureDriveways;
std::map<const MSJunction*, int> MSDriveWay::myDepartDrivewayIndex;
std::map<const MSEdge*, std::vector<MSDriveWay*> > MSDriveWay::myDepartureDrivewaysEnds;
std::map<const MSEdge*, std::vector<MSDriveWay*>, ComparatorNumericalIdLess> MSDriveWay::myEndingDriveways;
std::map<ConstMSEdgeVector, MSDriveWay*> MSDriveWay::myDriveWayRouteLookup;
std::map<std::string, MSDriveWay*> MSDriveWay::myDriveWayLookup;

// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDriveWay::init() {
    myWriteVehicles = OptionsCont::getOptions().isSet("railsignal-vehicle-output");
}

// ===========================================================================
// MSDriveWay method definitions
// ===========================================================================


MSDriveWay::MSDriveWay(const MSLink* origin, const std::string& id, bool temporary) :
    MSMoveReminder("DriveWay_" + (temporary ? "tmp" : id)),
    Named(id),
    myNumericalID(temporary ? -1 : myGlobalDriveWayIndex++),
    myOrigin(origin),
    myActive(nullptr),
    myCoreSize(0),
    myForwardEdgeCount(0),
    myFoundSignal(false),
    myFoundJump(false),
    myTerminateRoute(false),
    myAbortedBuild(false),
    myBidiEnded(false),
    myIsSubDriveway(false)
{}


MSDriveWay::~MSDriveWay() {
    for (const MSDriveWay* sub : mySubDriveWays) {
        delete sub;
    }
    mySubDriveWays.clear();
}

void
MSDriveWay::cleanup() {
    myGlobalDriveWayIndex = 0;
    myBlockLengthWarnings.clear();
    myWriteVehicles = false;

    for (auto item : myDepartureDriveways) {
        for (MSDriveWay* dw : item.second) {
            delete dw;
        }
    }
    MSDriveWay::mySwitchDriveWays.clear();
    MSDriveWay::myReversalDriveWays.clear();
    MSDriveWay::myDepartureDriveways.clear();
    MSDriveWay::myDepartDrivewayIndex.clear();
    MSDriveWay::myDepartureDrivewaysEnds.clear();
    MSDriveWay::myEndingDriveways.clear();
}

void
MSDriveWay::clearState() {
    for (auto item : myEndingDriveways) {
        for (MSDriveWay* dw : item.second) {
            dw->myTrains.clear();
        }
    }
}


bool
MSDriveWay::notifyEnter(SUMOTrafficObject& veh, Notification reason, const MSLane* enteredLane) {
#ifdef DEBUG_MOVEREMINDER
    std::cout << SIMTIME << " notifyEnter " << getDescription() << " veh=" << veh.getID() << " lane=" << (MSGlobals::gUseMesoSim ? veh.getEdge()->getID() : Named::getIDSecure(enteredLane)) << " reason=" << reason << "\n";
#endif
    if (veh.isVehicle() && (enteredLane == myLane || (MSGlobals::gUseMesoSim && veh.getEdge() == &myLane->getEdge()))
            && (reason == NOTIFICATION_DEPARTED || reason == NOTIFICATION_JUNCTION || reason == NOTIFICATION_PARKING)) {
        SUMOVehicle& sveh = dynamic_cast<SUMOVehicle&>(veh);
        MSRouteIterator firstIt = std::find(sveh.getCurrentRouteEdge(), sveh.getRoute().end(), myLane->getNextNormal());
        if (match(firstIt, sveh.getRoute().end())) {
            if (myTrains.count(&sveh) == 0) {
                enterDriveWay(sveh, reason);
            }
            return true;
        }
    } else if (reason == NOTIFICATION_REROUTE) {
        assert(veh.isVehicle());
        SUMOVehicle& sveh = dynamic_cast<SUMOVehicle&>(veh);
        assert(myTrains.count(&sveh) == 0);
        int movedPast = matchesPastRoute(sveh);
        // vehicle must still be one the drivway
        if (movedPast >= 0 && movedPast < myForwardEdgeCount) {
            enterDriveWay(sveh, reason);
            return true;
        }
    }
    return false;
}


bool
MSDriveWay::notifyLeave(SUMOTrafficObject& veh, double /*lastPos*/, Notification reason, const MSLane* enteredLane) {
    UNUSED_PARAMETER(enteredLane);
#ifdef DEBUG_MOVEREMINDER
    std::cout << SIMTIME << " notifyLeave " << getDescription() << " veh=" << veh.getID() << " lane=" << Named::getIDSecure(enteredLane) << " reason=" << toString(reason) << "\n";
#endif
    if (veh.isVehicle()) {
        // leaving network with departure, teleport etc
        if (reason != MSMoveReminder::NOTIFICATION_JUNCTION && reason != MSMoveReminder::NOTIFICATION_SEGMENT) {
            myTrains.erase(&dynamic_cast<SUMOVehicle&>(veh));
            if (myWriteVehicles) {
                myVehicleEvents.push_back(VehicleEvent(SIMSTEP, false, veh.getID(), reason));
            }
            return false;
        } else if (MSGlobals::gUseMesoSim && reason != MSMoveReminder::NOTIFICATION_SEGMENT) {
            // notifyLeave is called before moving the route iterator
            const MSLane* leftLane = (*(dynamic_cast<SUMOVehicle&>(veh).getCurrentRouteEdge()))->getLanes().front();
            return notifyLeaveBack(veh, reason, leftLane);
        } else {
            return true;
        }
    } else {
        return false;
    }
}


bool
MSDriveWay::notifyLeaveBack(SUMOTrafficObject& veh, Notification reason, const MSLane* leftLane) {
#ifdef DEBUG_MOVEREMINDER
    std::cout << SIMTIME << " notifyLeaveBack " << getDescription() << " veh=" << veh.getID() << " lane=" << Named::getIDSecure(leftLane) << " reason=" << toString(reason) << "\n";
#endif
    if (veh.isVehicle()) {
        if (leftLane == myForward.back() && (veh.getBackLane() != leftLane->getBidiLane() || MSGlobals::gUseMesoSim)) {
            myTrains.erase(&dynamic_cast<SUMOVehicle&>(veh));
            if (myWriteVehicles) {
                myVehicleEvents.push_back(VehicleEvent(SIMSTEP, false, veh.getID(), reason));
            }
            return false;
        } else {
            return true;
        }
    } else {
        return false;
    }
}


bool
MSDriveWay::notifyReroute(SUMOTrafficObject& veh) {
#ifdef DEBUG_MOVEREMINDER
    std::cout << SIMTIME << " notifyReroute " << getDescription() << " veh=" << veh.getID() << "\n";
#endif
    assert(veh.isVehicle());
    SUMOVehicle* sveh = dynamic_cast<SUMOVehicle*>(&veh);
    assert(myTrains.count(sveh) != 0);
    if (matchesPastRoute(*sveh) >= 0) {
        //std::cout << SIMTIME << " notifyReroute " << getDescription() << " veh=" << veh.getID() << " valid\n";
        return true;
    }
    // no match found, remove
    myTrains.erase(sveh);
    if (myWriteVehicles) {
        myVehicleEvents.push_back(VehicleEvent(SIMSTEP, false, veh.getID(), NOTIFICATION_REROUTE));
    }
    //std::cout << SIMTIME << " notifyReroute " << getDescription() << " veh=" << veh.getID() << " invalid\n";
    return false;
}


int
MSDriveWay::matchesPastRoute(SUMOVehicle& sveh) const {
    // look backwards along the route to find the driveway lane
    const ConstMSEdgeVector& routeEdges = sveh.getRoute().getEdges();
    for (int i = sveh.getRoutePosition(); i >= 0; i--) {
        if (routeEdges[i] == myLane->getNextNormal()) {
            MSRouteIterator firstIt = routeEdges.begin() + i;
            if (match(firstIt, sveh.getRoute().end())) {
                // driveway is still valid after rerouting
                //std::cout << SIMTIME << " notifyReroute " << getDescription() << " veh=" << veh.getID() << " valid\n";
                return sveh.getRoutePosition() - i;
            }
            break;
        }
    }
    return -1;
}


void
MSDriveWay::enterDriveWay(SUMOVehicle& sveh, Notification reason) {
    myTrains.insert(&sveh);
    if (myOrigin != nullptr) {
        MSRailSignalControl::getInstance().notifyApproach(myOrigin);
    }
    for (const MSDriveWay* foe : myFoes) {
        if (foe->myOrigin != nullptr) {
            MSRailSignalControl::getInstance().notifyApproach(foe->myOrigin);
        }
    }
    if (myWriteVehicles) {
        myVehicleEvents.push_back(VehicleEvent(SIMSTEP, true, sveh.getID(), reason));
    }
}

bool
MSDriveWay::reserve(const Approaching& closest, MSEdgeVector& occupied) {
    if (foeDriveWayOccupied(true, closest.first, occupied)) {
        return false;
    }
    for (MSLink* foeLink : myConflictLinks) {
        if (hasLinkConflict(closest, foeLink)) {
#ifdef DEBUG_SIGNALSTATE
            if (gDebugFlag4 || DEBUG_HELPER(closest.first)) {
                std::cout << getID() << " linkConflict with " << getTLLinkID(foeLink) << "\n";
            }
#endif
            return false;
        }
    }
    myActive = closest.first;
    return true;
}


bool
MSDriveWay::hasLinkConflict(const Approaching& veh, const MSLink* foeLink) const {
#ifdef DEBUG_SIGNALSTATE_PRIORITY
    if (gDebugFlag4) {
        std::cout << "   checkLinkConflict foeLink=" << getTLLinkID(foeLink) << " ego=" << Named::getIDSecure(veh.first) << "\n";
    }
#endif
    if (foeLink->getApproaching().size() > 0) {
        Approaching foe = foeLink->getClosest();
#ifdef DEBUG_SIGNALSTATE_PRIORITY
        if (gDebugFlag4) {
            std::cout << "     approaching foe=" << foe.first->getID() << "\n";
        }
#endif
        if (foe.first == veh.first) {
            return false;
        }
        const MSTrafficLightLogic* foeTLL = foeLink->getTLLogic();
        assert(foeTLL != nullptr);
        const MSRailSignal* constFoeRS = dynamic_cast<const MSRailSignal*>(foeTLL);
        MSRailSignal* foeRS = const_cast<MSRailSignal*>(constFoeRS);
        if (foeRS != nullptr) {
            const MSDriveWay& foeDriveWay = foeRS->retrieveDriveWayForVeh(foeLink->getTLIndex(), foe.first);
            MSEdgeVector occupied;
            if (foeDriveWay.foeDriveWayOccupied(false, foe.first, occupied) ||
                    !foeRS->constraintsAllow(foe.first) ||
                    !overlap(foeDriveWay) ||
                    !isFoeOrSubFoe(&foeDriveWay) ||
                    canUseSiding(veh.first, &foeDriveWay).first) {
#ifdef DEBUG_SIGNALSTATE_PRIORITY
                if (gDebugFlag4) {
                    if (foeDriveWay.foeDriveWayOccupied(false, foe.first, occupied)) {
                        std::cout << "     foe blocked\n";
                    } else if (!foeRS->constraintsAllow(foe.first)) {
                        std::cout << "     foe constrained\n";
                    } else if (!overlap(foeDriveWay)) {
                        std::cout << "     no overlap\n";
                    } else if (!isFoeOrSubFoe(&foeDriveWay)) {
                        std::cout << "     foeDW=" << foeDriveWay.getID() << " is not a foe to " << getID() << "\n";
                    } else if (canUseSiding(veh.first, &foeDriveWay).first) {
                        std::cout << "     use siding\n";
                    }
                }
#endif
                return false;
            }
#ifdef DEBUG_SIGNALSTATE_PRIORITY
            if (gDebugFlag4) {
                std::cout
                        << "  aSB=" << veh.second.arrivalSpeedBraking << " foeASB=" << foe.second.arrivalSpeedBraking
                        << "  aT=" << veh.second.arrivalTime << " foeAT=" << foe.second.arrivalTime
                        << "  aS=" << veh.first->getSpeed() << " foeS=" << foe.first->getSpeed()
                        << "  aD=" << veh.second.dist << " foeD=" << foe.second.dist
                        << "  aW=" << veh.first->getWaitingTime() << " foeW=" << foe.first->getWaitingTime()
                        << "  aN=" << veh.first->getNumericalID() << " foeN=" << foe.first->getNumericalID()
                        << "\n";
            }
#endif
            const bool yield = mustYield(veh, foe);
            if (MSRailSignal::storeVehicles()) {
                MSRailSignal::rivalVehicles().push_back(foe.first);
                if (yield) {
                    MSRailSignal::priorityVehicles().push_back(foe.first);
                }
            }
            return yield;
        }
    }
    return false;
}


bool
MSDriveWay::isFoeOrSubFoe(const MSDriveWay* foe) const {
    if (std::find(myFoes.begin(), myFoes.end(), foe) != myFoes.end()) {
        return true;
    }
    for (const MSDriveWay* sub : foe->mySubDriveWays) {
        if (isFoeOrSubFoe(sub)) {
            return true;
        }
    }
    return false;
}


bool
MSDriveWay::mustYield(const Approaching& veh, const Approaching& foe) {
    if (foe.second.arrivalSpeedBraking == veh.second.arrivalSpeedBraking) {
        if (foe.second.arrivalTime == veh.second.arrivalTime) {
            if (foe.first->getSpeed() == veh.first->getSpeed()) {
                if (foe.second.dist == veh.second.dist) {
                    if (foe.first->getWaitingTime() == veh.first->getWaitingTime()) {
                        return foe.first->getNumericalID() < veh.first->getNumericalID();
                    } else {
                        return foe.first->getWaitingTime() > veh.first->getWaitingTime();
                    }
                } else {
                    return foe.second.dist < veh.second.dist;
                }
            } else {
                return foe.first->getSpeed() > veh.first->getSpeed();
            }
        } else {
            return foe.second.arrivalTime < veh.second.arrivalTime;
        }
    } else {
        return foe.second.arrivalSpeedBraking > veh.second.arrivalSpeedBraking;
    }
}


bool
MSDriveWay::conflictLaneOccupied(bool store, const SUMOVehicle* ego) const {
    for (const MSLane* lane : myConflictLanes) {
        if (!lane->isEmpty()) {
            std::string joinVehicle = "";
            if (ego != nullptr && !MSGlobals::gUseMesoSim) {
                const SUMOVehicleParameter::Stop* stop = ego->getNextStopParameter();
                if (stop != nullptr) {
                    joinVehicle = stop->join;
                }
            }
#ifdef DEBUG_SIGNALSTATE
            if (gDebugFlag4) {
                std::cout << SIMTIME << " conflictLane " << lane->getID() << " occupied ego=" << Named::getIDSecure(ego) << " vehNumber=" << lane->getVehicleNumber() << "\n";
                if (joinVehicle != "") {
                    std::cout << "  joinVehicle=" << joinVehicle << " occupant=" << toString(lane->getVehiclesSecure()) << "\n";
                    lane->releaseVehicles();
                }
            }
#endif
            if (lane->getVehicleNumberWithPartials() == 1) {
                MSVehicle* foe = lane->getLastAnyVehicle();
                if (joinVehicle != "") {
                    if (foe->getID() == joinVehicle && foe->isStopped()) {
#ifdef DEBUG_SIGNALSTATE
                        if (gDebugFlag4) {
                            std::cout << "    ignore join-target '" << joinVehicle << "\n";
                        }
#endif
                        continue;
                    }
                }
                if (ego != nullptr) {
                    if (foe == ego && std::find(myForward.begin(), myForward.end(), lane) == myForward.end()) {
#ifdef DEBUG_SIGNALSTATE
                        if (gDebugFlag4) {
                            std::cout << "    ignore ego as oncoming '" << ego->getID() << "\n";
                        }
#endif
                        continue;
                    }
                    if (foe->isStopped() && foe->getNextStopParameter()->join == ego->getID()) {
#ifdef DEBUG_SIGNALSTATE
                        if (gDebugFlag4) {
                            std::cout << "    ignore " << foe->getID() << " for which ego is join-target\n";
                        }
#endif
                        continue;
                    }
                }
            }
            if (MSRailSignal::storeVehicles() && store) {
                MSRailSignal::blockingVehicles().push_back(lane->getLastAnyVehicle());
            }
            return true;
        }
    }
    return false;
}


bool
MSDriveWay::foeDriveWayApproached() const {
    for (const MSDriveWay* foeDW : myFoes) {
        if (foeDW->myOrigin != nullptr && foeDW->myOrigin->getApproaching().size() > 0) {
#ifdef DEBUG_SIGNALSTATE
            if (gDebugFlag4) {
                std::cout << SIMTIME << " foeLink=" << foeDW->myOrigin->getDescription() << " approachedBy=" << foeDW->myOrigin->getApproaching().begin()->first->getID() << "\n";
            }
#endif
            return true;
        }
    }
    return false;
}


bool
MSDriveWay::foeDriveWayOccupied(bool store, const SUMOVehicle* ego, MSEdgeVector& occupied) const {
    for (const MSDriveWay* foeDW : myFoes) {
        if (!foeDW->myTrains.empty()) {
#ifdef DEBUG_SIGNALSTATE
            if (gDebugFlag4 || DEBUG_COND_DW || DEBUG_HELPER(ego)) {
                std::cout << SIMTIME << " " << getID() << " foeDriveWay " << foeDW->getID() << " occupied ego=" << Named::getIDSecure(ego) << " foeVeh=" << toString(foeDW->myTrains) << "\n";
            }
#endif
            if (foeDW->myTrains.size() == 1) {
                SUMOVehicle* foe = *foeDW->myTrains.begin();
                if (foe == ego) {
#ifdef DEBUG_SIGNALSTATE
                    if (gDebugFlag4 || DEBUG_HELPER(ego)) {
                        std::cout << "    ignore ego as foe '" << Named::getIDSecure(ego) << "\n";
                    }
#endif
                    continue;
                }
                if (hasJoin(ego, foe)) {
                    continue;
                }
            }
            std::pair<bool, const MSDriveWay*> useSiding = canUseSiding(ego, foeDW);
#ifdef DEBUG_SIGNALSTATE
            if (gDebugFlag4 || DEBUG_COND_DW || DEBUG_HELPER(ego)) {
                auto it = mySidings.find(foeDW);
                int numSidings = 0;
                if (it != mySidings.end()) {
                    numSidings = it->second.size();
                }
                std::cout << "  useSiding=" << useSiding.first << " sidingFoe=" << Named::getIDSecure(useSiding.second) << " numSidings=" << numSidings << "\n";
            }
#endif
            if (useSiding.first) {
                continue;
            } else {
                if (MSRailSignal::storeVehicles() && store) {
                    for (SUMOVehicle* foe : foeDW->myTrains) {
                        MSRailSignal::blockingVehicles().push_back(foe);
                    }
                    MSRailSignal::blockingDriveWays().push_back(foeDW);
                }
                for (const SUMOVehicle* foe : foeDW->myTrains) {
                    occupied.push_back(const_cast<MSEdge*>(foe->getEdge()));
                    MSEdge* bidi = const_cast<MSEdge*>(foe->getEdge()->getBidiEdge());
                    if (bidi != nullptr) {
                        occupied.push_back(bidi);
                    }
                    /// @todo: if foe occupies more than one edge we should add all of them to the occupied vector
                }
                if (ego != nullptr && MSGlobals::gTimeToTeleportRSDeadlock > 0
                        && (ego->getWaitingTime() > ego->getVehicleType().getCarFollowModel().getStartupDelay() || !ego->isOnRoad())) {
                    // if there is an occupied siding, it becomes part of the waitRelation
                    SUMOVehicle* foe = *(useSiding.second == nullptr ? foeDW : useSiding.second)->myTrains.begin();
                    const MSRailSignal* rs = myOrigin != nullptr ? dynamic_cast<const MSRailSignal*>(myOrigin->getTLLogic()) : nullptr;
                    MSRailSignalControl::getInstance().addWaitRelation(ego, rs, foe);
                }
                return true;
            }
        } else if (foeDW != this && isDepartDriveway() && !foeDW->isDepartDriveway()) {
            if (foeDW->myOrigin->getApproaching().size() > 0) {
                Approaching foeA = foeDW->myOrigin->getClosest();
                const SUMOVehicle* foe = foeA.first;
                if (foeA.second.dist < foe->getBrakeGap(true)) {
                    MSRouteIterator firstIt = std::find(foe->getCurrentRouteEdge(), foe->getRoute().end(), foeDW->myRoute.front());
                    if (firstIt != foe->getRoute().end()) {
                        if (foeDW->match(firstIt, foe->getRoute().end())) {
                            bool useSiding = canUseSiding(ego, foeDW).first;
#ifdef DEBUG_SIGNALSTATE
                            if (gDebugFlag4 || DEBUG_COND_DW || DEBUG_HELPER(ego)) {
                                std::cout << SIMTIME << " " << getID() << " blocked by " << foeDW->getID() << " (approached by " << foe->getID() << ") useSiding=" << useSiding << "\n";
                            }
#endif
                            if (useSiding) {
                                //std::cout << SIMTIME << " " << getID() << " ego=" << ego->getID() << " foeDW=" << foeDW->getID() << " myFoes=" << toString(myFoes) << "\n";
                                continue;
                            } else {
                                return true;
                            }
                        }
                    }
                }
            }
        }
    }
    for (const std::set<const MSDriveWay*>& dlFoes : myDeadlocks) {
        bool allOccupied = true;
        for (const MSDriveWay* dlFoe : dlFoes) {
            if (dlFoe->myTrains.empty()) {
                allOccupied = false;
                //std::cout << SIMTIME << " " << getID() << " ego=" << Named::getIDSecure(ego) << "  deadlockCheck clear " << dlFoe->getID() << "\n";
                break;
            }
        }
        if (allOccupied) {
#ifdef DEBUG_SIGNALSTATE
            if (gDebugFlag4 || DEBUG_COND_DW || DEBUG_HELPER(ego)) {
                std::cout << SIMTIME << " " << getID() << " ego=" << Named::getIDSecure(ego) << " deadlockCheck " << joinNamedToString(dlFoes, " ") << "\n";
            }
#endif
            for (const MSDriveWay* dlFoe : dlFoes) {
                MSRailSignal::blockingDriveWays().push_back(dlFoe);
            }
            return true;
        }
    }
    return false;
}


bool
MSDriveWay::hasJoin(const SUMOVehicle* ego, const SUMOVehicle* foe) {
    if (ego != nullptr && !MSGlobals::gUseMesoSim) {
        std::string joinVehicle = "";
        const SUMOVehicleParameter::Stop* stop = ego->getNextStopParameter();
        if (stop != nullptr) {
            joinVehicle = stop->join;
        }
        if (joinVehicle == "" && !ego->hasDeparted() && ego->getStops().size() > 1) {
            // check one more stop
            auto it = ego->getStops().begin();
            std::advance(it, 1);
            joinVehicle = it->pars.join;
        }
        if (joinVehicle != "") {
#ifdef DEBUG_SIGNALSTATE
            if (gDebugFlag4 || DEBUG_COND_DW) {
                std::cout << "  joinVehicle=" << joinVehicle << "\n";
            }
#endif
            if (foe->getID() == joinVehicle && foe->isStopped()) {
#ifdef DEBUG_SIGNALSTATE
                if (gDebugFlag4 || DEBUG_COND_DW) {
                    std::cout << "    ignore join-target '" << joinVehicle << "\n";
                }
#endif
                return true;
            }
        }

        if (foe->isStopped() && foe->getNextStopParameter()->join == ego->getID()) {
#ifdef DEBUG_SIGNALSTATE
            if (gDebugFlag4 || DEBUG_COND_DW) {
                std::cout << "    ignore " << foe->getID() << " for which ego is join-target\n";
            }
#endif
            return true;
        }
    }
    return false;
}


std::pair<bool, const MSDriveWay*>
MSDriveWay::canUseSiding(const SUMOVehicle* ego, const MSDriveWay* foe, bool recurse) const {
    auto it = mySidings.find(foe);
    if (it != mySidings.end()) {
        for (auto siding : it->second) {
            // assume siding is usuable when computing state for unapproached signal (ego == nullptr)
            if (ego == nullptr || siding.length >= ego->getLength()) {
                // if the siding is already "reserved" by another vehicle we cannot use it here
                const MSEdge* sidingEnd = myRoute[siding.end];
                for (MSDriveWay* sidingApproach : myEndingDriveways[sidingEnd]) {
                    if (!sidingApproach->myTrains.empty()) {
                        // possibly the foe vehicle can use the other part of the siding
                        if (recurse) {
                            const SUMOVehicle* foeVeh = nullptr;
                            if (!foe->myTrains.empty()) {
                                foeVeh = *foe->myTrains.begin();
                            } else if (foe->myOrigin != nullptr && foe->myOrigin->getApproaching().size() > 0) {
                                foeVeh = foe->myOrigin->getClosest().first;
                            }
                            if (foeVeh == nullptr) {
                                WRITE_WARNINGF("Invalid call to canUseSiding dw=% foe=% ego=% time=%", getID(), foe->getID(), Named::getIDSecure(ego), time2string(SIMSTEP));
                                continue;
                            }
                            if (foe->canUseSiding(foeVeh, this, false).first) {
                                continue;
                            }
                        }
                        // possibly the foe vehicle
                        // @todo: in principle it might still be possible to continue if vehicle that approaches the siding can safely leave the situation
#ifdef DEBUG_SIGNALSTATE
                        if (gDebugFlag4 || DEBUG_COND_DW || DEBUG_HELPER(ego)) {
                            std::cout << SIMTIME << " " << getID() << " ego=" << Named::getIDSecure(ego) << " foe=" << foe->getID()
                                      << " foeVeh=" << toString(foe->myTrains)
                                      << " sidingEnd=" << sidingEnd->getID() << " sidingApproach=" << sidingApproach->getID() << " approaching=" << toString(sidingApproach->myTrains) << "\n";
                        }
#endif
                        return std::make_pair(false, sidingApproach);
                    }
                }
                //std::cout << SIMTIME << " " << getID() << " ego=" << Named::getIDSecure(ego) << " foe=" << foe->getID()
                //    << " foeVeh=" << toString(foe->myTrains)
                //    << " sidingEnd=" << sidingEnd->getID() << "usable\n";
                return std::make_pair(true, nullptr);
            }
        }
    }
    return std::make_pair(false, nullptr);
}

bool
MSDriveWay::overlap(const MSDriveWay& other) const {
    for (int i = 0; i < myCoreSize; i++) {
        for (int j = 0; j < other.myCoreSize; j++) {
            const MSEdge* edge = myRoute[i];
            const MSEdge* edge2 = other.myRoute[j];
            if (edge->getToJunction() == edge2->getToJunction()
                    || edge->getToJunction() == edge2->getFromJunction()) {
                // XXX might be rail_crossing with parallel tracks
                return true;
            }
        }
    }
    return false;
}


bool
MSDriveWay::flankConflict(const MSDriveWay& other) const {
    for (const MSLane* lane : myForward) {
        for (const MSLane* lane2 : other.myForward) {
            if (lane == lane2) {
                return true;
            }
        }
        for (const MSLane* lane2 : other.myBidi) {
            if (lane == lane2) {
                if (bidiBlockedBy(other)) {
                    // it's only a deadlock if both trains block symmetrically
                    return true;
                }
            }
        }
        for (const MSLane* lane2 : other.myBidiExtended) {
            if (lane == lane2) {
                if (bidiBlockedBy(other)) {
                    // it's only a deadlock if both trains block symmetrically
                    return true;
                }
            }
        }
    }
    return false;
}


bool
MSDriveWay::crossingConflict(const MSDriveWay& other) const {
    for (const MSLane* lane : myForward) {
        for (const MSLane* lane2 : other.myForward) {
            if (lane->isNormal() && lane2->isNormal() && lane->getEdge().getToJunction() == lane2->getEdge().getToJunction()) {
                return true;
            }
        }
    }
    return false;
}


bool
MSDriveWay::bidiBlockedBy(const MSDriveWay& other) const {
    for (const MSLane* lane : myBidi) {
        for (const MSLane* lane2 : other.myForward) {
            if (lane == lane2) {
                return true;
            }
        }
    }
    for (const MSLane* lane : myBidiExtended) {
        for (const MSLane* lane2 : other.myForward) {
            if (lane == lane2) {
                if (overlap(other)) {
                    return true;
                }
            }
        }
    }
    return false;
}


bool
MSDriveWay::bidiBlockedByEnd(const MSDriveWay& other) const {
    const MSLane* end = other.myForward.back();
    for (const MSLane* lane : myBidi) {
        if (lane == end) {
            return true;
        }
    }
    for (const MSLane* lane : myBidiExtended) {
        if (lane == end) {
            if (overlap(other)) {
                return true;
            }
        }
    }
    return false;
}

bool
MSDriveWay::forwardRouteConflict(std::set<const MSEdge*> forward, const MSDriveWay& other, bool secondCheck) {
    int i = 0;
    for (const MSEdge* edge2 : other.myRoute) {
        if (i == other.myCoreSize) {
            return false;
        }
        i++;
        if (edge2 == myForward.front()->getNextNormal() && !secondCheck) {
            // foe should not pass from behind through our own forward section
            return false;
        }
        if (forward.count(edge2->getBidiEdge()) != 0) {
            return true;
        }
    }
    return false;
}

void
MSDriveWay::writeBlocks(OutputDevice& od) const {
    od.openTag(myIsSubDriveway ? SUMO_TAG_SUBDRIVEWAY : SUMO_TAG_DRIVEWAY);
    od.writeAttr(SUMO_ATTR_ID, myID);
    od.writeAttr(SUMO_ATTR_VEHICLE, myFirstVehicle);
    od.writeAttr(SUMO_ATTR_EDGES, toString(myRoute));
    if (myCoreSize != (int)myRoute.size()) {
        od.writeAttr("core", myCoreSize);
    }
    od.openTag("forward");
    od.writeAttr(SUMO_ATTR_LANES, toString(myForward));
    od.closeTag();
    if (!myIsSubDriveway) {
        od.openTag("bidi");
        od.writeAttr(SUMO_ATTR_LANES, toString(myBidi));
        if (myBidiExtended.size() > 0) {
            od.lf();
            od << "                   ";
            od.writeAttr("deadlockCheck", toString(myBidiExtended));
        }
        od.closeTag();
        od.openTag("flank");
        od.writeAttr(SUMO_ATTR_LANES, toString(myFlank));
        od.closeTag();

        od.openTag("conflictLinks");

        std::vector<std::string> signals;
        for (MSLink* link : myConflictLinks) {
            signals.push_back(getTLLinkID(link));
        }
        od.writeAttr("signals", joinToStringSorting(signals, " "));
        od.closeTag();

        std::vector<std::string> foes;
        for (MSDriveWay* dw : myFoes) {
            foes.push_back(dw->myID);
        }
        if (foes.size() > 0) {
            od.openTag("foes");
            od.writeAttr("driveWays", joinToStringSorting(foes, " "));
            od.closeTag();
        }
        for (auto item : mySidings) {
            od.openTag("sidings");
            od.writeAttr("foe", item.first->getID());
            for (auto siding : item.second) {
                od.openTag("siding");
                od.writeAttr("start", myRoute[siding.start]->getID());
                od.writeAttr("end", myRoute[siding.end]->getID());
                od.writeAttr("length", siding.length);
                od.closeTag();
            }
            od.closeTag();
        }
        for (auto item : myDeadlocks) {
            od.openTag("deadlock");
            od.writeAttr("foes", joinNamedToStringSorting(item, " "));
            od.closeTag();
        }
    }
    od.closeTag(); // driveWay

    for (const MSDriveWay* sub : mySubDriveWays) {
        sub->writeBlocks(od);
    }
#ifdef DRIVEWAY_SANITY_CHECK
    std::set<MSDriveWay*> uFoes(myFoes.begin(), myFoes.end());
    if (uFoes.size() != myFoes.size()) {
        WRITE_WARNINGF("Duplicate foes in driveway '%'", getID());

    }
#endif
}


void
MSDriveWay::writeBlockVehicles(OutputDevice& od) const {
    od.openTag(myIsSubDriveway ? "subDriveWay" : "driveWay");
    od.writeAttr(SUMO_ATTR_ID, myID);
    for (const VehicleEvent& ve : myVehicleEvents) {
        od.openTag(ve.isEntry ? "entry" : "exit");
        od.writeAttr(SUMO_ATTR_ID, ve.id);
        od.writeAttr(SUMO_ATTR_TIME, time2string(ve.time));
        od.writeAttr("reason", Notifications.getString(ve.reason));
        od.closeTag(); // event
    }
    od.closeTag(); // driveWay

    for (const MSDriveWay* sub : mySubDriveWays) {
        sub->writeBlockVehicles(od);
    }
}


void
MSDriveWay::buildRoute(const MSLink* origin,
                       MSRouteIterator next, MSRouteIterator end,
                       LaneVisitedMap& visited,
                       std::set<MSLink*>& flankSwitches) {
    double length = 0;
    bool seekForwardSignal = true;
    bool seekBidiSwitch = true;
    bool foundUnsafeSwitch = false;
    MSLane* toLane = origin ? origin->getViaLaneOrLane() : (*next)->getLanes()[0];
    const std::string warnID = origin ? "rail signal " + getClickableTLLinkID(origin) : "insertion lane '" + toLane->getID() + "'";
#ifdef DEBUG_DRIVEWAY_BUILDROUTE
    gDebugFlag4 = DEBUG_COND_DW;
    if (gDebugFlag4) std::cout << "buildRoute origin=" << warnID << " vehRoute=" << toString(ConstMSEdgeVector(next, end))
                                   << " visited=" << formatVisitedMap(visited) << "\n";
#endif
    while ((seekForwardSignal || seekBidiSwitch)) {
        if (length > MSGlobals::gMaxRailSignalBlockLength) {
            // typical block length in germany on main lines is 3-5km on branch lines up to 7km
            // special branches that are used by one train exclusively could also be up to 20km in length
            // minimum block size in germany is 37.5m (LZB)
            // larger countries (USA, Russia) might see blocks beyond 20km)
            if (myRoute.size() == 0 || myBlockLengthWarnings.count(myRoute.front()) == 0) {
                WRITE_WARNINGF("Block after % exceeds maximum length (stopped searching after edge '%' (length=%m).",
                               warnID, toLane->getEdge().getID(), length);
                myBlockLengthWarnings.insert(myRoute.front());
            }
            myAbortedBuild = true;
            // length exceeded
#ifdef DEBUG_DRIVEWAY_BUILDROUTE
            if (gDebugFlag4) {
                std::cout << " abort: length=" << length << "\n";
            }
#endif
            return;
        }
#ifdef DEBUG_DRIVEWAY_BUILDROUTE
        if (gDebugFlag4) {
            std::cout << "   toLane=" << toLane->getID() << " visited=" << formatVisitedMap(visited) << "\n";
        }
#endif
        const MSEdge* current = &toLane->getEdge();
        if (current->isNormal()) {
            myRoute.push_back(current);
            if (next != end) {
                next++;
            }
        }
        appendMapIndex(visited, toLane);
        length += toLane->getLength();
        MSLane* bidi = toLane->getBidiLane();
        if (seekForwardSignal) {
            if (!foundUnsafeSwitch) {
                myForward.push_back(toLane);
                if (toLane->isNormal()) {
                    myForwardEdgeCount++;
                }
                if (myForward.size() == 1) {
                    myLane = toLane;
                    if (MSGlobals::gUseMesoSim) {
                        MESegment* s = MSGlobals::gMesoNet->getSegmentForEdge(myLane->getEdge());
                        s->addDetector(this, myLane->getIndex());
                    } else {
                        toLane->addMoveReminder(this, false);
                    }
                }
            }
        } else if (bidi == nullptr) {
            if (toLane->isInternal() && toLane->getIncomingLanes().front().viaLink->isTurnaround()) {
#ifdef DEBUG_DRIVEWAY_BUILDROUTE
                if (gDebugFlag4) {
                    std::cout << "      continue bidiSearch beyond turnaround\n";
                }
#endif
            } else {
                seekBidiSwitch = false;
#ifdef DEBUG_DRIVEWAY_BUILDROUTE
                if (gDebugFlag4) {
                    std::cout << "      noBidi, abort search for bidiSwitch\n";
                }
#endif
            }
        }
        if (bidi != nullptr) {
            if (foundUnsafeSwitch) {
                myBidiExtended.push_back(bidi);
            } else {
                myBidi.push_back(bidi);
            }
            if (!seekForwardSignal) {
                // look for switch that could protect from oncoming vehicles
                for (const auto& ili : bidi->getIncomingLanes()) {
                    if (ili.viaLink->getDirection() == LinkDirection::TURN) {
                        continue;
                    }
                    for (const MSLink* const link : ili.lane->getLinkCont()) {
                        if (link->getDirection() == LinkDirection::TURN) {
                            continue;
                        }
                        if (link->getViaLaneOrLane() != bidi) {
                            myCoreSize = (int)myRoute.size();
#ifdef DEBUG_DRIVEWAY_BUILDROUTE
                            if (gDebugFlag4) {
                                const MSEdge* const bidiNext = bidi->getNextNormal();
                                std::cout << "      found unsafe switch " << ili.viaLink->getDescription() << " (used=" << bidiNext->getID() << ")\n";
                            }
#endif
                            // trains along our route beyond this switch might create deadlock
                            foundUnsafeSwitch = true;
                            // the switch itself must still be guarded to ensure safety
                            for (const auto& ili2 : bidi->getIncomingLanes()) {
                                if (ili2.viaLink->getDirection() != LinkDirection::TURN) {
                                    flankSwitches.insert(ili.viaLink);
                                }
                            }
                        }
                    }
                }
            }
        }
        const std::vector<MSLink*>& links = toLane->getLinkCont();
        toLane = nullptr;
        for (const MSLink* const link : links) {
            if ((next != end && &link->getLane()->getEdge() == *next)
                    && isRailwayOrShared(link->getViaLaneOrLane()->getPermissions())) {
                toLane = link->getViaLaneOrLane();
                if (link->getTLLogic() != nullptr && link->getTLIndex() >= 0) {
                    if (link == origin) {
                        WRITE_WARNINGF(TL("Found circular block after % (% edges, length %)"), warnID, toString(myRoute.size()), toString(length));
                        //std::cout << getClickableTLLinkID(origin) << " circularBlock2=" << toString(myRoute) << "\n";
                        myAbortedBuild = true;
#ifdef DEBUG_DRIVEWAY_BUILDROUTE
                        if (gDebugFlag4) {
                            std::cout << " abort: found circle\n";
                        }
#endif
                        return;
                    }
                    seekForwardSignal = false;
                    myFoundSignal = true;
                    seekBidiSwitch = bidi != nullptr;
#ifdef DEBUG_DRIVEWAY_BUILDROUTE
                    if (gDebugFlag4) {
                        std::cout << "      found forwardSignal " << link->getTLLogic()->getID() << " seekBidiSwitch=" << seekBidiSwitch << "\n";
                    }
#endif
                }
                //if (links.size() > 1 && !foundUnsafeSwitch) {
                if (isSwitch(link)) {
                    // switch on driveway
                    //std::cout << "mySwitchDriveWays " << getID() << " link=" << link->getDescription() << "\n";
                    mySwitchDriveWays[link].push_back(this);
                }
                if (link->getLane()->getBidiLane() != nullptr && &link->getLane()->getEdge() == current->getBidiEdge()) {
                    // reversal on driveway
                    myReversalDriveWays[current].push_back(this);
                    myReversals.push_back(current);
                }
                break;
            }
        }
        if (toLane == nullptr) {
            if (next != end) {
                // no connection found, jump to next route edge
                toLane = (*next)->getLanes()[0];
#ifdef DEBUG_DRIVEWAY_BUILDROUTE
                if (gDebugFlag4) {
                    std::cout << "      abort: turn-around or jump\n";
                }
#endif
                myFoundJump = true;
                return;
            } else {
#ifdef DEBUG_DRIVEWAY_BUILDROUTE
                if (gDebugFlag4) {
                    std::cout << "      abort: no next lane available\n";
                }
#endif
                myTerminateRoute = true;
                return;
            }
        }
    }
    myBidiEnded = !seekBidiSwitch;
#ifdef DEBUG_DRIVEWAY_BUILDROUTE
    if (gDebugFlag4) {
        std::cout << " normalEnd myBidiEnded=" << myBidiEnded << "\n";
    }
#endif
}


bool
MSDriveWay::isSwitch(const MSLink* link) {
    for (const MSLink* other : link->getLaneBefore()->getNormalPredecessorLane()->getLinkCont()) {
        if (other->getLane() != link->getLane() && !other->isTurnaround()) {
            return true;
        }
    }
    for (auto ili : link->getLane()->getIncomingLanes()) {
        if (ili.viaLink != link && !ili.viaLink->isTurnaround()) {
            return true;
        }
    }
    const MSLane* bidi = link->getLane()->getBidiLane();
    if (bidi != nullptr) {
        for (const MSLink* other : bidi->getLinkCont()) {
            if (other->getLane() != link->getLaneBefore()->getNormalPredecessorLane()->getBidiLane() && !other->isTurnaround()) {
                return true;
            }
        }
    }
    return false;
}


void
MSDriveWay::checkFlanks(const MSLink* originLink, const std::vector<const MSLane*>& lanes, const LaneVisitedMap& visited, bool allFoes, std::set<MSLink*>& flankSwitches) const {
#ifdef DEBUG_CHECK_FLANKS
    std::cout << " checkFlanks lanes=" << toString(lanes) << " allFoes=" << allFoes << "\n";
#endif
    const MSLink* reverseOriginLink = originLink != nullptr && originLink->getLane()->getBidiLane() != nullptr && originLink->getLaneBefore()->getBidiLane() != nullptr
                                      ? originLink->getLane()->getBidiLane()->getLinkTo(originLink->getLaneBefore()->getBidiLane())
                                      : nullptr;
    //std::cout << "   originLink=" << originLink->getDescription() << "\n";
    if (reverseOriginLink != nullptr) {
        reverseOriginLink = reverseOriginLink->getCorrespondingExitLink();
        //std::cout << "   reverseOriginLink=" << reverseOriginLink->getDescription() << "\n";
    }
    for (int i = 0; i < (int)lanes.size(); i++) {
        const MSLane* lane = lanes[i];
        const MSLane* prev = i > 0 ? lanes[i - 1] : nullptr;
        const MSLane* next = i + 1 < (int)lanes.size() ? lanes[i + 1] : nullptr;
        if (lane->isInternal()) {
            continue;
        }
        for (auto ili : lane->getIncomingLanes()) {
            if (ili.viaLink == originLink
                    || ili.viaLink == reverseOriginLink
                    || ili.viaLink->getDirection() == LinkDirection::TURN
                    || ili.viaLink->getDirection() == LinkDirection::TURN_LEFTHAND) {
                continue;
            }
            if (ili.lane != prev && ili.lane != next) {
#ifdef DEBUG_CHECK_FLANKS
                std::cout << " add flankSwitch junction=" << ili.viaLink->getJunction()->getID() << " index=" << ili.viaLink->getIndex() << " iLane=" << ili.lane->getID() << " prev=" << Named::getIDSecure(prev) <<  " targetLane=" << lane->getID() << " next=" << Named::getIDSecure(next) << "\n";
#endif
                flankSwitches.insert(ili.viaLink);
            } else if (allFoes) {
                // link is part of the driveway, find foes that cross the driveway without entering
                checkCrossingFlanks(ili.viaLink, visited, flankSwitches);
            }
        }
    }
}


void
MSDriveWay::checkCrossingFlanks(MSLink* dwLink, const LaneVisitedMap& visited, std::set<MSLink*>& flankSwitches) const {
#ifdef DEBUG_CHECK_FLANKS
    std::cout << "  checkCrossingFlanks  dwLink=" << dwLink->getDescription() << " visited=" << formatVisitedMap(visited) << "\n";
#endif
    const MSJunction* junction = dwLink->getJunction();
    if (junction == nullptr) {
        return; // unregulated junction;
    }
    const MSJunctionLogic* logic = junction->getLogic();
    if (logic == nullptr) {
        return; // unregulated junction;
    }
    for (const MSEdge* in : junction->getIncoming()) {
        if (in->isInternal()) {
            continue;
        }
        for (MSLane* inLane : in->getLanes()) {
            const MSLane* inBidi = inLane->getBidiLane();
            if (isRailwayOrShared(inLane->getPermissions()) && visited.count(inLane) == 0 && (inBidi == nullptr || visited.count(inBidi) == 0)) {
                for (MSLink* link : inLane->getLinkCont()) {
                    if (link->getIndex() >= 0 && logic->getFoesFor(dwLink->getIndex()).test(link->getIndex())
                            && visited.count(link->getLane()) == 0) {
#ifdef DEBUG_CHECK_FLANKS
                        std::cout << " add crossing flankSwitch junction=" << junction->getID() << " index=" << link->getIndex() << "\n";
#endif
                        if (link->getViaLane() == nullptr) {
                            flankSwitches.insert(link);
                        } else {
                            flankSwitches.insert(link->getViaLane()->getLinkCont().front());
                        }
                    }
                }
            }
        }
    }
}

void
MSDriveWay::findFlankProtection(MSLink* link, MSLink* origLink, std::vector<const MSLane*>& flank) {
#ifdef DEBUG_CHECK_FLANKS
    std::cout << "  findFlankProtection link=" << link->getDescription() << " origLink=" << origLink->getDescription() << "\n";
#endif
    if (link->getCorrespondingEntryLink()->getTLLogic() != nullptr && link->getJunction()->getType() == SumoXMLNodeType::RAIL_SIGNAL) {
        MSLink* entry = const_cast<MSLink*>(link->getCorrespondingEntryLink());
        // guarded by signal
#ifdef DEBUG_CHECK_FLANKS
        std::cout << "   flank guarded by " << entry->getTLLogic()->getID() << "\n";
#endif
        // @note, technically it's enough to collect links from foe driveways
        // but this also adds "unused" conflict links which may aid comprehension
        myConflictLinks.push_back(entry);
        addFoes(entry);
    } else {
        const MSLane* lane = link->getLaneBefore();
        std::vector<MSLink*> predLinks;
        for (auto ili : lane->getIncomingLanes()) {
            if (!ili.viaLink->isTurnaround()) {
                predLinks.push_back(ili.viaLink);
            }
        }
        if (predLinks.size() > 1) {
            // this is a switch
#ifdef DEBUG_ADD_FOES
            std::cout << "    predecessors of " << link->getDescription() << " isSwitch\n";
#endif
            for (MSLink* pred : predLinks) {
                addSwitchFoes(pred);
            }
        } else if (predLinks.size() == 1) {
            if (isSwitch(link)) {
                addSwitchFoes(link);
            } else {
                // continue upstream via single predecessor
                findFlankProtection(predLinks.front(), origLink, flank);
            }
        }
        // check for insertions
        if (myDepartureDriveways.count(&lane->getEdge()) != 0) {
            for (MSDriveWay* foe : myDepartureDriveways[&lane->getEdge()]) {
                if (flankConflict(*foe) || crossingConflict(*foe)) {
#ifdef DEBUG_ADD_FOES
                    std::cout << "  foe " << foe->getID() << " departs on flank=" << lane->getID() << "\n";
#endif
                    myFoes.push_back(foe);
                } else {
#ifdef DEBUG_ADD_FOES
                    std::cout << "  cand foe " << foe->getID() << " departs on flank=" << lane->getID() << " rejected\n";
#endif
                }
            }
        }
    }
}


void
MSDriveWay::addSwitchFoes(MSLink* link) {
    auto it = mySwitchDriveWays.find(link);
    if (it != mySwitchDriveWays.end()) {
#ifdef DEBUG_ADD_FOES
        std::cout << "   driveway " << myID << " addSwitchFoes for link " << link->getDescription() << "\n";
#endif
        for (MSDriveWay* foe : it->second) {
            if (foe != this && (flankConflict(*foe) || foe->flankConflict(*this) || crossingConflict(*foe) || foe->crossingConflict(*this))) {
#ifdef DEBUG_ADD_FOES
                std::cout << "   foe=" << foe->myID
                          << " fc1=" << flankConflict(*foe) << " fc2=" << foe->flankConflict(*this)
                          << " cc1=" << crossingConflict(*foe) << " cc2=" << foe->crossingConflict(*this) << "\n";
#endif
                myFoes.push_back(foe);
            } else {
#ifdef DEBUG_ADD_FOES
                std::cout << "   cand=" << foe->myID << "\n";
#endif
            }
        }
    }
}


MSDriveWay*
MSDriveWay::buildDriveWay(const std::string& id, const MSLink* link, MSRouteIterator first, MSRouteIterator end) {
    // collect lanes and links that are relevant for setting this signal for the current driveWay
    // For each driveway we collect
    //   - conflictLanes (signal must be red if any conflict lane is occupied)
    //   - conflictLinks (signal must be red if any conflict link is approached by a vehicle
    //      - that cannot break in time (arrivalSpeedBraking > 0)
    //      - approached by a vehicle with higher switching priority (see #3941)
    // These objects are construct in steps:
    //
    // forwardBlock
    // - search forward recursive from outgoing lane until controlled railSignal link found
    //   -> add all found lanes to conflictLanes
    //
    // bidiBlock (if any forwardBlock edge has bidi edge)
    // - search bidi backward recursive until first switch
    //   - from switch search backward recursive all other incoming until controlled rail signal link
    //     -> add final links to conflictLinks
    //
    // flanks
    // - search backward recursive from flanking switches
    //   until controlled railSignal link or protecting switch is found
    //   -> add all found lanes to conflictLanes
    //   -> add final links to conflictLinks
    MSDriveWay* dw = new MSDriveWay(link, id);
    LaneVisitedMap visited;
    std::vector<const MSLane*> before;
    MSLane* fromBidi = nullptr;
    if (link != nullptr) {
        appendMapIndex(visited, link->getLaneBefore());
        fromBidi = link->getLaneBefore()->getBidiLane();
    }
    std::set<MSLink*> flankSwitches; // list of switches that threaten the driveway and for which protection must be found

    if (fromBidi != nullptr) {
        before.push_back(fromBidi);
    }
    dw->buildRoute(link, first, end, visited, flankSwitches);
    dw->myCoreSize = (int)dw->myRoute.size();
    dw->checkFlanks(link, dw->myForward, visited, true, flankSwitches);
    dw->checkFlanks(link, dw->myBidi, visited, false, flankSwitches);
    dw->checkFlanks(link, before, visited, true, flankSwitches);
    for (MSLink* fsLink : flankSwitches) {
#ifdef DEBUG_ADD_FOES
        if (DEBUG_COND_DW) {
            std::cout << " fsLink=" << fsLink->getDescription() << "\n";
        }
#endif
        dw->findFlankProtection(fsLink, fsLink, dw->myFlank);
    }
    std::set<MSLink*> flankSwitchesBidiExtended;
    dw->checkFlanks(link, dw->myBidiExtended, visited, false, flankSwitchesBidiExtended);
    for (MSLink* const flink : flankSwitchesBidiExtended) {
#ifdef DEBUG_ADD_FOES
        if (DEBUG_COND_DW) {
            std::cout << " fsLinkExtended=" << flink->getDescription() << "\n";
        }
#endif
        dw->findFlankProtection(flink, flink, dw->myBidiExtended);
    }
    MSRailSignal* rs = link ? const_cast<MSRailSignal*>(static_cast<const MSRailSignal*>(link->getTLLogic())) : nullptr;
    const bool movingBlock = (rs && rs->isMovingBlock()) || (!rs && OptionsCont::getOptions().getBool("railsignal-moving-block"));
#ifdef DEBUG_BUILD_DRIVEWAY
    if (DEBUG_COND_DW) {
        std::cout << SIMTIME << " buildDriveWay " << dw->myID << " link=" << (link == nullptr ? "NULL" : link->getDescription())
                  << "\n    route=" << toString(dw->myRoute)
                  << "\n    forward=" << toString(dw->myForward)
                  << "\n    bidi=" << toString(dw->myBidi)
                  << "\n    bidiEx=" << toString(dw->myBidiExtended)
                  << "\n    flank=" << toString(dw->myFlank)
                  << "\n    flankSwitch=" << MSRailSignal::describeLinks(std::vector<MSLink*>(flankSwitches.begin(), flankSwitches.end()))
                  << "\n    coreSize=" << dw->myCoreSize
                  << "\n";
    }
#endif
    if (!rs || !rs->isMovingBlock()) {
        dw->myConflictLanes.insert(dw->myConflictLanes.end(), dw->myForward.begin(), dw->myForward.end());
    }
    dw->myConflictLanes.insert(dw->myConflictLanes.end(), dw->myBidi.begin(), dw->myBidi.end());
    dw->myConflictLanes.insert(dw->myConflictLanes.end(), dw->myFlank.begin(), dw->myFlank.end());
    dw->addBidiFoes(rs, false);
    dw->addBidiFoes(rs, true);
    // add driveways that start on the same signal / lane
    dw->addParallelFoes(link, *first);
    // add driveways that reverse along this driveways route
    dw->addReversalFoes(movingBlock);
    // make foes unique and symmetrical
    std::set<MSDriveWay*, ComparatorNumericalIdLess> uniqueFoes(dw->myFoes.begin(), dw->myFoes.end());
    std::set<MSLink*> uniqueCLink(dw->myConflictLinks.begin(), dw->myConflictLinks.end());
    dw->myFoes.clear();
    const MSEdge* lastEdge = &dw->myForward.back()->getEdge();
    for (MSDriveWay* foe : uniqueFoes) {
        const MSEdge* foeLastEdge = &foe->myForward.back()->getEdge();
        const bool sameLast = foeLastEdge == lastEdge;
        if (sameLast && !movingBlock) {
            dw->myFoes.push_back(foe);
            if (foe != dw) {
                foe->myFoes.push_back(dw);
            }
        } else {
            if (foe->bidiBlockedByEnd(*dw)) {
#ifdef DEBUG_ADD_FOES
                if (DEBUG_COND_DW) {
                    std::cout << " setting " << dw->getID() << " as foe of " << foe->getID() << "\n";
                }
#endif
                foe->myFoes.push_back(dw);
                foe->addSidings(dw);
            } else {
                dw->buildSubFoe(foe, movingBlock);
            }
            if (foe != dw) { // check for movingBlock
                if (dw->bidiBlockedByEnd(*foe)) {
#ifdef DEBUG_ADD_FOES
                    if (DEBUG_COND_DW) {
                        std::cout << " addFoeCheckSiding " << foe->getID() << "\n";
                    }
#endif
                    dw->myFoes.push_back(foe);
                    dw->addSidings(foe);
                } else  {
                    foe->buildSubFoe(dw, movingBlock);
                }
            }
        }
        if (link) {
            foe->addConflictLink(link);
        }
        // ignore links that have the same start junction
        if (foe->myRoute.front()->getFromJunction() != dw->myRoute.front()->getFromJunction()) {
            for (auto ili : foe->myForward.front()->getIncomingLanes()) {
                if (ili.viaLink->getTLLogic() != nullptr) {
                    // ignore links that originate on myBidi
                    const MSLane* origin = ili.viaLink->getLaneBefore();
                    if (std::find(dw->myBidi.begin(), dw->myBidi.end(), origin) == dw->myBidi.end()) {
                        uniqueCLink.insert(ili.viaLink);
                    }
                }
            }
        }
    }
    dw->myConflictLinks.clear();
    dw->myConflictLinks.insert(dw->myConflictLinks.begin(), uniqueCLink.begin(), uniqueCLink.end());
    myEndingDriveways[lastEdge].push_back(dw);
    if (!movingBlock) {
        // every driveway is it's own foe (also all driveways that depart in the same block)
        for (MSDriveWay* sameEnd : myEndingDriveways[lastEdge]) {
            if (uniqueFoes.count(sameEnd) == 0) {
                dw->myFoes.push_back(sameEnd);
                if (sameEnd != dw) {
                    sameEnd->myFoes.push_back(dw);
                }
            }
        }
    }
#ifdef DEBUG_BUILD_DRIVEWAY
    if (DEBUG_COND_DW) {
        std::cout << dw->myID << " finalFoes " << toString(dw->myFoes) << "\n";
    }
#endif
    return dw;
}

std::string
MSDriveWay::getTLLinkID(const MSLink* link) {
    return link->getTLLogic()->getID() + "_" + toString(link->getTLIndex());
}

std::string
MSDriveWay::getJunctionLinkID(const MSLink* link) {
    return link->getJunction()->getID() + "_" + toString(link->getIndex());
}

std::string
MSDriveWay::getClickableTLLinkID(const MSLink* link) {
    return "junction '" +  link->getTLLogic()->getID() + "', link " + toString(link->getTLIndex());
}

std::string
MSDriveWay::formatVisitedMap(const LaneVisitedMap& visited) {
    UNUSED_PARAMETER(visited);
    /*
    std::vector<const MSLane*> lanes(visited.size(), nullptr);
    for (auto item : visited) {
        lanes[item.second] = item.first;
    }
    for (auto it = lanes.begin(); it != lanes.end();) {
        if (*it == nullptr) {
            it = lanes.erase(it);
        } else {
            it++;
        }
    }
    return toString(lanes);
    */
    return "dummy";
}


void
MSDriveWay::appendMapIndex(LaneVisitedMap& map, const MSLane* lane) {
    // avoid undefined behavior from evaluation order
    const int tmp = (int)map.size();
    map[lane] = tmp;
}

bool
MSDriveWay::match(MSRouteIterator firstIt, MSRouteIterator endIt) const {
    // @todo optimize: it is sufficient to check for specific edges (after each switch)
    auto itRoute = firstIt;
    auto itDwRoute = myRoute.begin();
    bool match = true;
    while (itRoute != endIt && itDwRoute != myRoute.end()) {
        if (*itRoute != *itDwRoute) {
            match = false;
#ifdef DEBUG_MATCH
            std::cout << "  check dw=" << getID() << " match failed at vehEdge=" << (*itRoute)->getID() << " dwEdge=" << (*itDwRoute)->getID() << "\n";
#endif
            break;
        }
        itRoute++;
        itDwRoute++;
    }
    // if the vehicle arrives before the end of this driveway,
    // we'd rather build a new driveway to avoid superfluous restrictions
    if (match && itDwRoute == myRoute.end()
            && (itRoute == endIt || myAbortedBuild || myBidiEnded || myFoundJump || myIsSubDriveway)) {
        //std::cout << "  using dw=" << "\n";
        if (itRoute != endIt) {
            // check whether the current route requires an extended driveway
            const MSEdge* next = *itRoute;
            const MSEdge* prev = myRoute.back();
            if (myFoundJump && prev->getBidiEdge() != next && prev->getBidiEdge() != nullptr
                    && prev->isConnectedTo(*next, (SUMOVehicleClass)(SVC_RAIL_CLASSES & prev->getPermissions()))) {
#ifdef DEBUG_MATCH
                std::cout << "  check dw=" << getID() << " prev=" << prev->getID() << " next=" << next->getID() << "\n";
#endif
                return false;
            }
            if (!myFoundJump && prev->getBidiEdge() == next && prev == &myForward.back()->getEdge()) {
                assert(myIsSubDriveway || myBidiEnded);
                // must not leave driveway via reversal
#ifdef DEBUG_MATCH
                std::cout << getID() << " back=" << myForward.back()->getID() << " noMatch route " << toString(ConstMSEdgeVector(firstIt, endIt)) << "\n";
#endif
                return false;
            }
        }
        return true;
    }
    return false;
}

void
MSDriveWay::addFoes(const MSLink* link) {
#ifdef DEBUG_ADD_FOES
    std::cout << "driveway " << myID << " addFoes for link " << link->getDescription() << "\n";
#endif
    const MSRailSignal* rs = dynamic_cast<const MSRailSignal*>(link->getTLLogic());
    if (rs != nullptr) {
        for (MSDriveWay* foe : rs->retrieveDriveWays(link->getTLIndex())) {
#ifdef DEBUG_ADD_FOES
            std::cout << "  cand foe=" << foe->myID << " fc1=" << flankConflict(*foe) << " fc2=" << foe->flankConflict(*this) << " cc1=" << crossingConflict(*foe) << " cc2=" <<  foe->crossingConflict(*this) << "\n";
#endif
            if (foe != this && (flankConflict(*foe) || foe->flankConflict(*this) || crossingConflict(*foe) || foe->crossingConflict(*this))) {
#ifdef DEBUG_ADD_FOES
                std::cout << "   foe=" << foe->myID << "\n";
#endif
                myFoes.push_back(foe);
            }
        }
    }
}


void
MSDriveWay::addBidiFoes(const MSRailSignal* ownSignal, bool extended) {
#ifdef DEBUG_ADD_FOES
    std::cout << "driveway " << myID << " addBidiFoes extended=" << extended << "\n";
#endif
    const std::vector<const MSLane*>& bidiLanes = extended ? myBidiExtended : myBidi;
    for (const MSLane* bidi : bidiLanes) {
        for (auto ili : bidi->getIncomingLanes()) {
            const MSRailSignal* rs = dynamic_cast<const MSRailSignal*>(ili.viaLink->getTLLogic());
            if (rs != nullptr && rs != ownSignal &&
                    std::find(bidiLanes.begin(), bidiLanes.end(), ili.lane) != bidiLanes.end()) {
                addFoes(ili.viaLink);
            }
        }
        const MSEdge* bidiEdge = &bidi->getEdge();
        if (myDepartureDriveways.count(bidiEdge) != 0) {
            for (MSDriveWay* foe : myDepartureDriveways[bidiEdge]) {
                if (flankConflict(*foe)) {
#ifdef DEBUG_ADD_FOES
                    std::cout << "  foe " << foe->getID() << " departs on bidi=" << bidiEdge->getID() << "\n";
#endif
                    myFoes.push_back(foe);
                } else {
#ifdef DEBUG_ADD_FOES
                    std::cout << "  cand foe " << foe->getID() << " departs on bidi=" << bidiEdge->getID() << " rejected\n";
#endif
                }
            }
        }
        if (myDepartureDrivewaysEnds.count(bidiEdge) != 0) {
            for (MSDriveWay* foe : myDepartureDrivewaysEnds[bidiEdge]) {
                if (flankConflict(*foe)) {
#ifdef DEBUG_ADD_FOES
                    std::cout << "  foe " << foe->getID() << " ends on bidi=" << bidiEdge->getID() << "\n";
#endif
                    myFoes.push_back(foe);
                } else {
#ifdef DEBUG_ADD_FOES
                    std::cout << "  cand foe " << foe->getID() << " ends on bidi=" << bidiEdge->getID() << " rejected\n";
#endif
                }
            }
        }
    }
}


void
MSDriveWay::addParallelFoes(const MSLink* link, const MSEdge* first) {
#ifdef DEBUG_ADD_FOES
    std::cout << "driveway " << myID << " addParallelFoes\n";
#endif
    if (link) {
        addFoes(link);
    } else {
        auto it = myDepartureDriveways.find(first);
        if (it != myDepartureDriveways.end()) {
            for (MSDriveWay* foe : it->second) {
#ifdef DEBUG_ADD_FOES
                std::cout << "  foe " << foe->getID() << " departs on first=" << first->getID() << "\n";
#endif
                myFoes.push_back(foe);
            }
        }
    }
}


void
MSDriveWay::addReversalFoes(bool movingBlock) {
#ifdef DEBUG_ADD_FOES
    std::cout << "driveway " << myID << " addReversalFoes\n";
#endif
    std::set<const MSEdge*> forward;
    for (const MSLane* lane : myForward) {
        if (lane->isNormal()) {
            forward.insert(&lane->getEdge());
        }
    }
    int i = 0;
    for (const MSEdge* e : myRoute) {
        if (forward.count(e) != 0 && !movingBlock) {
            // reversals in our own forward can be ignored because each driveway
            // is automatically a foe of itself by default
            continue;
        }
        if (i == myCoreSize) {
            break;
        }
        i++;
        auto it = myReversalDriveWays.find(e);
        if (it != myReversalDriveWays.end()) {
            for (MSDriveWay* foe : it->second) {
                // check whether the foe reverses into our own forward section
                // (it might reverse again or disappear via arrival)
#ifdef DEBUG_ADD_FOES
                //std::cout << "  candidate foe " << foe->getID() << " reverses on edge=" << e->getID() << " forward=" << joinNamedToString(forward, " ") << " foeRoute=" << toString(foe->myRoute) << "\n";
#endif
                if (forwardRouteConflict(forward, *foe)) {
                    std::set<const MSEdge*> foeForward;
                    for (const MSLane* lane : foe->myForward) {
                        if (lane->isNormal()) {
                            foeForward.insert(&lane->getEdge());
                            if (lane->getBidiLane() != nullptr) {
                                foeForward.insert(lane->getEdge().getBidiEdge());
                            }
                        }
                    }
#ifdef DEBUG_ADD_FOES
                    std::cout << "  reversal cand=" << foe->getID() << " foeForward " << toString(foeForward) << "\n";
#endif
                    if (foe->forwardRouteConflict(foeForward, *this, true)) {
#ifdef DEBUG_ADD_FOES
                        std::cout << "  foe " << foe->getID() << " reverses on edge=" << e->getID() << "\n";
#endif
                        myFoes.push_back(foe);
                    }
                } else if (movingBlock && foe == this) {
#ifdef DEBUG_ADD_FOES
                    std::cout << "  dw " << getID() << " reverses on forward edge=" << e->getID() << " (movingBlock)\n";
#endif
                    myFoes.push_back(foe);
                }
            }
        }
    }
}


bool
MSDriveWay::buildSubFoe(MSDriveWay* foe, bool movingBlock) {
    // Subdriveways (Teilfahrstraße) model the resolution of a driving conflict
    // before a vehicle has left the driveway. This is possible when the driveway diverges from the foe
    // driveway at an earlier point (switch or crossing).
    //
    // We already know that the last edge of this driveway doesn't impact the foe (unless the driveway ends within the block).
    // Remove further edges from the end of the driveway (myForward) until the point of conflict is found.
    //
    // For movingBlock the logic is changed:
    // We remove the conflict-free part as before but then keep removing the the conflict part until the
    // another non-conconflit part is found
    if (myForward.size() < foe->myForward.size() &&
            myForward == std::vector<const MSLane*>(foe->myForward.begin(), foe->myForward.begin() + myForward.size())) {
#ifdef DEBUG_BUILD_SUBDRIVEWAY
        std::cout << SIMTIME << " buildSubFoe dw=" << getID() << " is subpart of foe=" << foe->getID() << "\n";
#endif
        foe->myFoes.push_back(this);
        return true;
    }
    int subLast = (int)myForward.size() - 2;
#ifdef DEBUG_BUILD_SUBDRIVEWAY
    if (subLast < 0) {
        std::cout << "  " << getID() << " cannot build subDriveWay for foe " << foe->getID() << " because myForward has only a single lane\n";
    }
#endif
    bool foundConflict = false;
    while (subLast >= 0) {
        const MSLane* lane = myForward[subLast];
        MSDriveWay tmp(myOrigin, "tmp", true);
        tmp.myForward.push_back(lane);
#ifdef DEBUG_BUILD_SUBDRIVEWAY
        std::cout << "  subLast=" << subLast << " lane=" << lane->getID() << " fc=" << tmp.flankConflict(*foe) << " cc=" << tmp.crossingConflict(*foe)
                  << " bc=" << (std::find(foe->myBidi.begin(), foe->myBidi.end(), lane) != foe->myBidi.end()) << "\n";
#endif
        const bool bidiConflict = std::find(foe->myBidi.begin(), foe->myBidi.end(), lane) != foe->myBidi.end();
        if (tmp.flankConflict(*foe) || tmp.crossingConflict(*foe) || bidiConflict) {
            foundConflict = true;
            if (!movingBlock || bidiConflict) {
                break;
            }
        } else if (foundConflict) {
            break;
        }
        subLast--;
    }
    if (subLast < 0) {
        if (foe->myTerminateRoute) {
            if (bidiBlockedByEnd(*foe) && bidiBlockedBy(*this) && foe->forwardEndOnRoute(this)) {
                foe->myFoes.push_back(this);
                // foe will get the sidings
                addSidings(foe, true);
            }
#ifdef DEBUG_BUILD_SUBDRIVEWAY
            std::cout << SIMTIME << " buildSubFoe dw=" << getID() << " foe=" << foe->getID() << " terminates\n";
#endif
        } else if (myTerminateRoute && myBidi.size() <= myForward.size()) {
            foe->myFoes.push_back(this);
#ifdef DEBUG_BUILD_SUBDRIVEWAY
            std::cout << SIMTIME << " buildSubFoe dw=" << getID() << " terminates, foe=" << foe->getID() << "\n";
#endif
            return true;
        } else if (foe->myReversals.size() % 2 == 1) {
#ifdef DEBUG_BUILD_SUBDRIVEWAY
            std::cout << SIMTIME << " buildSubFoe dw=" << getID() << " foe=" << foe->getID() << " has " << foe->myReversals.size() << " reversals\n";
#endif
        } else {
#ifdef DEBUG_BUILD_SUBDRIVEWAY
            std::cout << SIMTIME << " buildSubFoe dw=" << getID() << " foe=" << foe->getID() << " failed\n";
#endif
#ifdef SUBDRIVEWAY_WARN_NOCONFLICT
            WRITE_WARNINGF("No point of conflict found between driveway '%' and driveway '%' when creating sub-driveway", getID(), foe->getID());
#endif
        }
        return false;
    }
    int subSize = subLast + 1;
    for (MSDriveWay* cand : mySubDriveWays) {
        if ((int)cand->myForward.size() == subSize) {
            // can re-use existing sub-driveway
            foe->myFoes.push_back(cand);
            cand->myFoes.push_back(foe);
#ifdef DEBUG_BUILD_SUBDRIVEWAY
            std::cout << SIMTIME << " buildSubFoe dw=" << getID() << " foe=" << foe->getID() << " useExisting=" << cand->getID() << "\n";
#endif
            return true;
        }
    }
    std::vector<const MSLane*> forward(myForward.begin(), myForward.begin() + subSize);
    std::vector<const MSEdge*> route;
    for (const MSLane* lane : forward) {
        if (lane->isNormal()) {
            route.push_back(&lane->getEdge());
        }
    }
    if (route.empty()) {
#ifdef DEBUG_BUILD_SUBDRIVEWAY
        std::cout << SIMTIME << " abort subFoe dw=" << getID() << " foe=" << foe->getID() << " empty subRoute\n";
#endif
        return false;
    }
    if (myRoute.size() > route.size()) {
        // route continues. make sure the subDriveway does not end with a reversal
        const MSEdge* lastNormal = route.back();
        const MSEdge* nextNormal = myRoute[route.size()];
        if (lastNormal->getBidiEdge() == nextNormal) {
#ifdef DEBUG_BUILD_SUBDRIVEWAY
            std::cout << SIMTIME << " abort subFoe dw=" << getID() << " foe=" << foe->getID()
                      << " lastNormal=" << lastNormal->getID() << " nextNormal=" << nextNormal->getID() << " endWithReversal\n";
#endif
            return false;
        }
    }
    MSDriveWay* sub = new MSDriveWay(myOrigin, getID() + "." + toString(mySubDriveWays.size()));
    sub->myLane = myLane;
    sub->myIsSubDriveway = true;
    sub->myForward = forward;
    sub->myRoute = route;
    sub->myCoreSize = (int)sub->myRoute.size();
    myLane->addMoveReminder(sub, false);

    // copy trains that are currently on this driveway (and associated entry events)
    for (SUMOVehicle* veh : myTrains) {
        if (std::find(sub->myRoute.begin(), sub->myRoute.end(), veh->getEdge()) != sub->myRoute.end()) {
            sub->myTrains.insert(veh);
            dynamic_cast<MSBaseVehicle*>(veh)->addReminder(sub);
            for (const VehicleEvent& ve : myVehicleEvents) {
                if (ve.id == veh->getID()) {
                    sub->myVehicleEvents.push_back(ve);
                }
            }
        }
    }

    foe->myFoes.push_back(sub);
    sub->myFoes.push_back(foe);
    mySubDriveWays.push_back(sub);
#ifdef DEBUG_BUILD_SUBDRIVEWAY
    std::cout << SIMTIME << " buildSubFoe dw=" << getID() << " foe=" << foe->getID() << " sub=" << sub->getID() << " route=" << toString(sub->myRoute) << "\n";
#endif
    return true;
}

void
MSDriveWay::addSidings(MSDriveWay* foe, bool addToFoe) {
    const MSEdge* foeEndBidi = foe->myForward.back()->getEdge().getBidiEdge();
    int forwardNormals = 0;
    for (auto lane : foe->myForward) {
        if (lane->isNormal()) {
            forwardNormals++;
        }
    }
    if (forwardNormals == (int)foe->myRoute.size()) {
#ifdef DEBUG_BUILD_SIDINGS
        std::cout << "checkSiding " << getID() << " foe=" << foe->getID() << " forwardNormals=" << forwardNormals << " frSize=" << foe->myRoute.size() <<  " aborted\n";
#endif
        return;
    }
    auto foeSearchBeg = foe->myRoute.begin() + forwardNormals;
    auto foeSearchEnd = foe->myRoute.end();
    if (foeEndBidi == nullptr) {
        throw ProcessError("checkSiding " + getID() + " foe=" + foe->getID() + " noBidi\n");
    }
    int i;
    std::vector<int> start;
    std::vector<double> length;
    for (i = 0; i < (int)myRoute.size(); i++) {
        if (myRoute[i] == foeEndBidi) {
            break;
        }
    }
    if (i == (int)myRoute.size()) {
        throw ProcessError("checkSiding " + getID() + " foe=" + foe->getID() + " foeEndBidi=" + foeEndBidi->getID() + " not on route\n");
    }
    const MSEdge* next = myRoute[i];
#ifdef DEBUG_BUILD_SIDINGS
    std::cout << "checkSiding " << getID() << " foe=" << foe->getID() << " i=" << i << " next=" << next->getID() << " forwardNormals=" << forwardNormals << " frSize=" << foe->myRoute.size() << " foeSearchBeg=" << (*foeSearchBeg)->getID() << "\n";
#endif
    i--;
    for (; i >= 0; i--) {
        const MSEdge* cur = myRoute[i];
        if (hasRS(cur, next)) {
            if (std::find(foeSearchBeg, foeSearchEnd, cur->getBidiEdge()) == foeSearchEnd) {
                start.push_back(i);
                length.push_back(0);
            }
        }
        if (!start.empty()) {
            auto itFind = std::find(foeSearchBeg, foeSearchEnd, cur->getBidiEdge());
            if (itFind != foeSearchEnd) {
#ifdef DEBUG_BUILD_SIDINGS
                std::cout << "endSiding " << getID() << " foe=" << foe->getID() << " i=" << i << " curBidi=" << Named::getIDSecure(cur->getBidiEdge()) << " length=" << toString(length) << "\n";
#endif
                const int firstIndex = i + 1;
                if (addToFoe) {
                    auto& foeSidings = foe->mySidings[this];
                    // indices must be mapped onto foe route;
                    const MSEdge* first = myRoute[firstIndex];
                    auto itFirst = std::find(foe->myRoute.begin(), foe->myRoute.end(), first);
                    if (itFirst != foe->myRoute.end()) {
                        for (int j = 0; j < (int)length.size(); j++) {
                            const MSEdge* last = myRoute[start[j]];
                            auto itLast = std::find(itFirst, foe->myRoute.end(), last);
                            if (itLast != foe->myRoute.end()) {
                                foeSidings.insert(foeSidings.begin(), Siding((int)(itFirst - foe->myRoute.begin()), (int)(itLast - foe->myRoute.begin()), length[j]));
                            }
                        }
                    }
                } else {
                    auto& foeSidings = mySidings[foe];
                    for (int j = 0; j < (int)length.size(); j++) {
                        foeSidings.insert(foeSidings.begin(), Siding(firstIndex, start[j], length[j]));
                    }
                }
                start.clear();
                length.clear();
                foeSearchBeg = itFind;
            } else {
                for (int j = 0; j < (int)length.size(); j++) {
                    length[j] += cur->getLength();
                }
            }
        }
        next = cur;
    }
}


bool
MSDriveWay::hasRS(const MSEdge* cur, const MSEdge* next) {
    if (cur->getToJunction()->getType() == SumoXMLNodeType::RAIL_SIGNAL) {
        // check if there is a controlled link between cur and next
        for (auto lane : cur->getLanes()) {
            for (const MSLink* link : lane->getLinkCont()) {
                if (&link->getLane()->getEdge() == next && link->getTLLogic() != nullptr) {
                    return true;
                }
            }
        }
    }
    return false;
}


bool
MSDriveWay::forwardEndOnRoute(const MSDriveWay* foe) const {
    const MSEdge* foeForwardEnd = &foe->myForward.back()->getNormalPredecessorLane()->getEdge();
    return std::find(myRoute.begin(), myRoute.end(), foeForwardEnd) != myRoute.end();
}

void
MSDriveWay::addConflictLink(const MSLink* link) {
    if (link->getTLLogic() != nullptr) {
        // ignore links that originate on myBidi
        // and also links from the same junction as my own link
        const MSLane* origin = link->getLaneBefore();
        if (std::find(myBidi.begin(), myBidi.end(), origin) == myBidi.end()) {
            if (link->getJunction() != myRoute.front()->getFromJunction()) {
                if (std::find(myConflictLinks.begin(), myConflictLinks.end(), link) == myConflictLinks.end()) {
                    myConflictLinks.push_back(const_cast<MSLink*>(link));
                }
            }
        }
    }
}

void
MSDriveWay::addDWDeadlock(const std::vector<const MSDriveWay*>& deadlockFoes) {
    std::set<const MSDriveWay*> filtered;
    for (const MSDriveWay* foe : deadlockFoes) {
        if (std::find(myFoes.begin(), myFoes.end(), foe) == myFoes.end()) {
            filtered.insert(foe);
        }
    }
    if (std::find(myDeadlocks.begin(), myDeadlocks.end(), filtered) == myDeadlocks.end()) {
        myDeadlocks.push_back(filtered);
        //std::cout << getID() << " deadlockFoes=" << toString(deadlockFoes) << "\n";
    }
}

const MSDriveWay*
MSDriveWay::getDepartureDriveway(const SUMOVehicle* veh, bool init) {
    const MSEdge* edge = init ? veh->getRoute().getEdges()[veh->getDepartEdge()] : veh->getEdge();
    if (edge->getFromJunction()->getType() == SumoXMLNodeType::RAIL_SIGNAL) {
        for (const MSLane* lane : edge->getLanes()) {
            for (auto ili : lane->getIncomingLanes()) {
                const MSLink* entry = ili.viaLink->getCorrespondingEntryLink();
                const MSRailSignal* rs = dynamic_cast<const MSRailSignal*>(entry->getTLLogic());
                if (rs != nullptr) {
                    const MSDriveWay* dw = &const_cast<MSRailSignal*>(rs)->retrieveDriveWayForVeh(entry->getTLIndex(), veh);
                    if (&dw->myForward.front()->getEdge() == edge) {
                        return dw;
                    }
                }
            }
        }
    }
    for (MSDriveWay* dw : myDepartureDriveways[edge]) {
        auto matchStart = init ? veh->getRoute().begin() + veh->getDepartEdge() : veh->getCurrentRouteEdge();
        if (dw->match(matchStart, veh->getRoute().end())) {
            return dw;
        }
    }
    const std::string id = edge->getFromJunction()->getID() + ".d" + toString(myDepartDrivewayIndex[edge->getFromJunction()]++);
    MSDriveWay* dw = buildDriveWay(id, nullptr, veh->getCurrentRouteEdge(), veh->getRoute().end());
    myDepartureDriveways[edge].push_back(dw);
    myDepartureDrivewaysEnds[&dw->myForward.back()->getEdge()].push_back(dw);
    dw->setVehicle(veh->getID());
    return dw;
}


void
MSDriveWay::writeDepatureBlocks(OutputDevice& od, bool writeVehicles) {
    for (auto item  : myDepartureDriveways) {
        const MSEdge* edge = item.first;
        if (item.second.size() > 0) {
            od.openTag("departJunction");
            od.writeAttr(SUMO_ATTR_ID, edge->getFromJunction()->getID());
            for (const MSDriveWay* dw : item.second) {
                if (writeVehicles) {
                    dw->writeBlockVehicles(od);
                } else {
                    dw->writeBlocks(od);
                }
            }
            od.closeTag(); // departJunction
        }
    }
}

void
MSDriveWay::saveState(OutputDevice& out) {
    // all driveways are in myEndingDriveways which makes it convenient
    for (auto item : myEndingDriveways) {
        for (MSDriveWay* dw : item.second) {
            dw->_saveState(out);
            for (MSDriveWay* sub : dw->mySubDriveWays) {
                sub->_saveState(out);
            }
        }
    }
}

void
MSDriveWay::_saveState(OutputDevice& out) const {
    if (!myTrains.empty() || haveSubTrains()) {
        out.openTag(myIsSubDriveway ? SUMO_TAG_SUBDRIVEWAY : SUMO_TAG_DRIVEWAY);
        out.writeAttr(SUMO_ATTR_ID, getID());
        out.writeAttr(SUMO_ATTR_EDGES, toString(myRoute));
        if (!myTrains.empty()) {
            out.writeAttr(SUMO_ATTR_VEHICLES, toString(myTrains));
        }
        out.closeTag();
    }
}


bool
MSDriveWay::haveSubTrains() const {
    for (MSDriveWay* sub : mySubDriveWays) {
        if (!sub->myTrains.empty()) {
            return true;
        }
    }
    return false;
}

void
MSDriveWay::loadState(const SUMOSAXAttributes& attrs, int tag) {
    if ((int)myDriveWayRouteLookup.size() < myGlobalDriveWayIndex) {
        for (auto item : myEndingDriveways) {
            for (MSDriveWay* dw : item.second) {
                myDriveWayRouteLookup[dw->myRoute] = dw;
            }
        }
    }
    MSVehicleControl& c = MSNet::getInstance()->getVehicleControl();
    bool ok;
    const std::string id  = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    const std::string edges = attrs.get<std::string>(SUMO_ATTR_EDGES, id.c_str(), ok);
    ConstMSEdgeVector route;
    if (attrs.hasAttribute(SUMO_ATTR_EDGES)) {
        MSEdge::parseEdgesList(edges, route, id);
    }
    MSDriveWay* dw = nullptr;
    if (tag == SUMO_TAG_DRIVEWAY) {
        auto it = myDriveWayRouteLookup.find(route);
        if (it == myDriveWayRouteLookup.end()) {
            //WRITE_WARNING(TLF("Unknown driveWay '%' with route '%'", id, edges));
            //return;
            throw ProcessError(TLF("Unknown driveWay '%' with route '%'", id, edges));
        }
        dw = it->second;
        myDriveWayLookup[id] = dw;
    } else {
        std::string parentID = id.substr(0, id.rfind('.'));
        auto it = myDriveWayLookup.find(parentID);
        if (it == myDriveWayLookup.end()) {
            //WRITE_WARNING(TLF("Unknown parent driveway '%' for subDriveWay '%'", parentID, id));
            //return;
            throw ProcessError(TLF("Unknown parent driveway '%' for subDriveWay '%'", parentID, id));
        }
        MSDriveWay* parent = it->second;
        for (MSDriveWay* sub : parent->mySubDriveWays) {
            if (sub->myRoute == route) {
                dw = sub;
                break;
            }
        }
        if (dw == nullptr) {
            // missing subdriveways can be ignored. They may have been created
            // as foes for driveways that are not relevant at state loading time
            return;
        }
    }
    const std::string vehicles = attrs.getOpt<std::string>(SUMO_ATTR_VEHICLES, id.c_str(), ok, "");
    for (const std::string& vehID : StringTokenizer(vehicles).getVector()) {
        MSBaseVehicle* veh = dynamic_cast<MSBaseVehicle*>(c.getVehicle(vehID));
        if (veh == nullptr) {
            throw ProcessError(TLF("Unknown vehicle '%' in driveway '%'", vehID, id));
        }
        dw->myTrains.insert(veh);
        veh->addReminder(dw);
    }
}

const MSDriveWay*
MSDriveWay::retrieveDepartDriveWay(const MSEdge* edge, const std::string& id) {
    for (MSDriveWay* dw : myDepartureDriveways[edge]) {
        if (dw->getID() == id) {
            return dw;
        }
    }
    return nullptr;
}


bool
MSDriveWay::hasTrain(SUMOVehicle* veh) const {
    return myTrains.count(veh) != 0;
}

/****************************************************************************/
