/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2001-2024 German Aerospace Center (DLR) and others.
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
/// @file    MSEdge.cpp
/// @author  Christian Roessel
/// @author  Jakob Erdmann
/// @author  Christoph Sommer
/// @author  Daniel Krajzewicz
/// @author  Laura Bieker
/// @author  Michael Behrisch
/// @author  Sascha Krieg
/// @date    Tue, 06 Mar 2001
///
// A road/street connecting two junctions
/****************************************************************************/
#include <config.h>

#include <algorithm>
#include <iostream>
#include <cassert>
#ifdef HAVE_FOX
#include <utils/common/ScopedLocker.h>
#endif
#include <utils/common/StringTokenizer.h>
#include <utils/options/OptionsCont.h>
#include <microsim/devices/MSRoutingEngine.h>
#include <mesosim/MELoop.h>
#include <mesosim/MESegment.h>
#include <mesosim/MEVehicle.h>
#include "MSInsertionControl.h"
#include "MSJunction.h"
#include "MSLane.h"
#include "MSLaneChanger.h"
#include "MSLaneChangerSublane.h"
#include "MSLink.h"
#include "MSGlobals.h"
#include "MSNet.h"
#include "MSVehicle.h"
#include "MSLeaderInfo.h"
#include <microsim/transportables/MSTransportable.h>
#include "MSEdgeWeightsStorage.h"
#include "MSEdge.h"

#define BEST_LANE_LOOKAHEAD 3000.0

// ===========================================================================
// static member definitions
// ===========================================================================
MSEdge::DictType MSEdge::myDict;
MSEdgeVector MSEdge::myEdges;
SVCPermissions MSEdge::myMesoIgnoredVClasses(0);


// ===========================================================================
// member method definitions
// ===========================================================================
MSEdge::MSEdge(const std::string& id, int numericalID,
               const SumoXMLEdgeFunc function,
               const std::string& streetName,
               const std::string& edgeType,
               int priority,
               double distance) :
    Named(id), myNumericalID(numericalID), myLanes(nullptr),
    myLaneChanger(nullptr), myFunction(function), myVaporizationRequests(0),
    myLastFailedInsertionTime(-1),
    myFromJunction(nullptr), myToJunction(nullptr),
    myHaveTransientPermissions(false),
    myOtherTazConnector(nullptr),
    myStreetName(streetName),
    myEdgeType(edgeType),
    myPriority(priority),
    myDistance(distance),
    myWidth(0.),
    myLength(0.),
    myEmptyTraveltime(0.),
    myTimePenalty(0.),
    myAmDelayed(false),
    myAmRoundabout(false),
    myAmFringe(true),
    myBidiEdge(nullptr)
{ }


MSEdge::~MSEdge() {
    delete myLaneChanger;
    delete myReversedRoutingEdge;
    delete myRailwayRoutingEdge;
}


void
MSEdge::initialize(const std::vector<MSLane*>* lanes) {
    assert(lanes != 0);
    myLanes = std::shared_ptr<const std::vector<MSLane*> >(lanes);
    if (myFunction == SumoXMLEdgeFunc::CONNECTOR) {
        myCombinedPermissions = SVCAll;
    }
    for (MSLane* const lane : *lanes) {
        lane->setRightSideOnEdge(myWidth, (int)mySublaneSides.size());
        MSLeaderInfo ahead(lane->getWidth());
        for (int j = 0; j < ahead.numSublanes(); ++j) {
            mySublaneSides.push_back(myWidth + j * MSGlobals::gLateralResolution);
        }
        myWidth += lane->getWidth();
    }
}


void MSEdge::recalcCache() {
    if (myLanes->empty()) {
        return;
    }
    myLength = myLanes->front()->getLength();
    myEmptyTraveltime = myLength / MAX2(getSpeedLimit(), NUMERICAL_EPS);
    if (isNormal() && (MSGlobals::gUseMesoSim || MSGlobals::gTLSPenalty > 0)) {
        SUMOTime minorPenalty = 0;
        bool haveTLSPenalty = MSGlobals::gTLSPenalty > 0;
        if (MSGlobals::gUseMesoSim) {
            const MESegment::MesoEdgeType& edgeType = MSNet::getInstance()->getMesoType(getEdgeType());
            minorPenalty = edgeType.minorPenalty;
            haveTLSPenalty = edgeType.tlsPenalty > 0;
        }
        if (haveTLSPenalty || minorPenalty > 0) {
            // add tls penalties to the minimum travel time
            SUMOTime minPenalty = -1;
            for (const MSLane* const l : *myLanes) {
                for (const MSLink* const link : l->getLinkCont()) {
                    if (link->getLane()->isWalkingArea() && link->getLaneBefore()->isNormal()) {
                        continue;
                    }
                    SUMOTime linkPenalty = link->isTLSControlled() ? link->getMesoTLSPenalty() : (link->havePriority() ? 0 : minorPenalty);
                    if (minPenalty == -1) {
                        minPenalty = linkPenalty;
                    } else {
                        minPenalty = MIN2(minPenalty, linkPenalty);
                    }
                }
            }
            if (minPenalty > 0) {
                myEmptyTraveltime += STEPS2TIME(minPenalty);
                myTimePenalty = STEPS2TIME(minPenalty);
            }
        }
    } else if (isCrossing() && MSGlobals::gTLSPenalty > 0) {
        // penalties are recorded for the entering link
        for (const auto& ili : myLanes->front()->getIncomingLanes()) {
            double penalty = STEPS2TIME(ili.viaLink->getMesoTLSPenalty());
            if (!ili.viaLink->haveOffPriority()) {
                penalty = MAX2(penalty, MSGlobals::gMinorPenalty);
            }
            if (penalty > 0) {
                myEmptyTraveltime += penalty;
                myTimePenalty = penalty;
            }
        }
    } else if (isInternal() && MSGlobals::gUsingInternalLanes) {
        const MSLink* link = myLanes->front()->getIncomingLanes()[0].viaLink;
        if (!link->isTLSControlled() && !link->havePriority()) {
            if (link->isTurnaround()) {
                myEmptyTraveltime += MSGlobals::gTurnaroundPenalty;
                myTimePenalty = MSGlobals::gTurnaroundPenalty;
            } else {
                myEmptyTraveltime += MSGlobals::gMinorPenalty;
                myTimePenalty = MSGlobals::gMinorPenalty;
            }
        }
    }
}


void
MSEdge::resetTAZ(MSJunction* junction) {
    mySuccessors.clear();
    myPredecessors.clear();
    for (const MSEdge* edge : junction->getIncoming()) {
        if (!edge->isInternal()) {
            MSEdgeVector& succ = const_cast<MSEdgeVector&>(edge->mySuccessors);
            MSConstEdgePairVector& succVia = const_cast<MSConstEdgePairVector&>(edge->myViaSuccessors);
            MSEdgeVector& pred = const_cast<MSEdgeVector&>(edge->myPredecessors);
            auto it = std::find(succ.begin(), succ.end(), this);
            auto it2 = std::find(succVia.begin(), succVia.end(), std::make_pair(const_cast<const MSEdge*>(this), (const MSEdge*)nullptr));
            auto it3 = std::find(pred.begin(), pred.end(), this);
            if (it != succ.end()) {
                succ.erase(it);
                succVia.erase(it2);
            }
            if (it3 != pred.end()) {
                pred.erase(it3);
            }
        }
    }
}

void
MSEdge::closeBuilding() {
    for (MSLane* const lane : *myLanes) {
        for (MSLink* const link : lane->getLinkCont()) {
            link->initParallelLinks();
            MSLane* const toL = link->getLane();
            MSLane* const viaL = link->getViaLane();
            if (toL != nullptr) {
                MSEdge& to = toL->getEdge();
                if (std::find(mySuccessors.begin(), mySuccessors.end(), &to) == mySuccessors.end()) {
                    mySuccessors.push_back(&to);
                    myViaSuccessors.push_back(std::make_pair(&to, (viaL == nullptr ? nullptr : &viaL->getEdge())));
                }
                if (std::find(to.myPredecessors.begin(), to.myPredecessors.end(), this) == to.myPredecessors.end()) {
                    to.myPredecessors.push_back(this);
                }
                if (link->getDirection() != LinkDirection::TURN) {
                    myAmFringe = false;
                }
            }
            if (viaL != nullptr) {
                MSEdge& to = viaL->getEdge();
                if (std::find(to.myPredecessors.begin(), to.myPredecessors.end(), this) == to.myPredecessors.end()) {
                    to.myPredecessors.push_back(this);
                }
            }
        }
        lane->checkBufferType();
    }
    std::sort(mySuccessors.begin(), mySuccessors.end(), by_id_sorter());
    rebuildAllowedLanes(true);
    recalcCache();

    // extend lookup table for sublane model after all edges are read
    if (myLanes->back()->getOpposite() != nullptr) {
        MSLane* opposite = myLanes->back()->getOpposite();
        MSLeaderInfo ahead(opposite->getWidth());
        for (int j = 0; j < ahead.numSublanes(); ++j) {
            mySublaneSides.push_back(myWidth + j * MSGlobals::gLateralResolution);
        }
    }
}


void
MSEdge::updateMesoType() {
    assert(MSGlobals::gUseMesoSim);
    if (!myLanes->empty()) {
        MSGlobals::gMesoNet->updateSegmentsForEdge(*this);
    }
}


void
MSEdge::buildLaneChanger() {
    if (!myLanes->empty()) {
        const bool allowChanging = allowsLaneChanging();
        if (MSGlobals::gLateralResolution > 0) {
            // may always initiate sublane-change
            if (!isInternal() || MSGlobals::gUsingInternalLanes) {
                myLaneChanger = new MSLaneChangerSublane(myLanes.get(), allowChanging);
            }
        } else {
            if (MSGlobals::gLaneChangeDuration > 0) {
                myLaneChanger = new MSLaneChanger(myLanes.get(), allowChanging);
            } else if (myLanes->size() > 1 || canChangeToOpposite()) {
                myLaneChanger = new MSLaneChanger(myLanes.get(), allowChanging);
            }
        }
    }
}


bool
MSEdge::allowsLaneChanging() const {
    if (isInternal() && MSGlobals::gUsingInternalLanes) {
        // allow changing only if all links leading to this internal lane have priority
        // or they are controlled by a traffic light
        for (const MSLane* const lane : *myLanes) {
            const MSLink* const link = lane->getLogicalPredecessorLane()->getLinkTo(lane);
            assert(link != nullptr);
            const LinkState state = link->getState();
            if ((state == LINKSTATE_MINOR && lane->getBidiLane() == nullptr)
                    || state == LINKSTATE_EQUAL
                    || state == LINKSTATE_STOP
                    || state == LINKSTATE_ALLWAY_STOP
                    || state == LINKSTATE_DEADEND) {
                return false;
            }
        }
    }
    return true;
}


void
MSEdge::addToAllowed(const SVCPermissions permissions, std::shared_ptr<const std::vector<MSLane*> > allowedLanes, AllowedLanesCont& laneCont) const {
    if (!allowedLanes->empty()) {
        // recheck whether we had this list to save memory
        for (auto& allowed : laneCont) {
            if (*allowed.second == *allowedLanes) {
                allowed.first |= permissions;
                return;
            }
        }
        laneCont.push_back(std::make_pair(permissions, allowedLanes));
    }
}


SVCPermissions
MSEdge::getMesoPermissions(SVCPermissions p, SVCPermissions ignoreIgnored) {
    SVCPermissions ignored = myMesoIgnoredVClasses & ~ignoreIgnored;
    return (p | ignored) == ignored ? 0 : p;
}


void
MSEdge::rebuildAllowedLanes(const bool onInit) {
    // rebuild myMinimumPermissions and myCombinedPermissions
    myMinimumPermissions = SVCAll;
    myCombinedPermissions = 0;
    bool lanesChangedPermission = false;
    for (MSLane* const lane : *myLanes) {
        // same dedicated lanes are ignored in meso to avoid capacity errors.
        // Here we have to make sure that vehicles which are set to depart on
        // such lanes trigger an error.
        SVCPermissions allow = getMesoPermissions(lane->getPermissions(), SVC_PEDESTRIAN);
        myMinimumPermissions &= allow;
        myCombinedPermissions |= allow;
        lanesChangedPermission |= lane->hadPermissionChanges();
    }
    if (!onInit && !myHaveTransientPermissions && lanesChangedPermission) {
        myHaveTransientPermissions = true;
        // backup original structures when first needed
        myOrigAllowed = myAllowed;
        myOrigAllowedTargets = myAllowedTargets;
        myOrigClassesViaSuccessorMap = myClassesViaSuccessorMap;
    }
    // rebuild myAllowed
    myAllowed.clear();
    if (myCombinedPermissions != myMinimumPermissions) {
        myAllowed.push_back(std::make_pair(SVC_IGNORING, myLanes));
        for (SVCPermissions vclass = SVC_PRIVATE; vclass <= SUMOVehicleClass_MAX; vclass *= 2) {
            if ((myCombinedPermissions & vclass) == vclass) {
                std::shared_ptr<std::vector<MSLane*> > allowedLanes = std::make_shared<std::vector<MSLane*> >();
                for (MSLane* const lane : *myLanes) {
                    if (lane->allowsVehicleClass((SUMOVehicleClass)vclass)) {
                        allowedLanes->push_back(lane);
                    }
                }
                addToAllowed(vclass, allowedLanes, myAllowed);
            }
        }
    }
    if (onInit) {
        myOriginalMinimumPermissions = myMinimumPermissions;
        myOriginalCombinedPermissions = myCombinedPermissions;
    } else {
        rebuildAllowedTargets(false);
        for (MSEdge* pred : myPredecessors) {
            if (myHaveTransientPermissions && !pred->myHaveTransientPermissions) {
                pred->myOrigAllowed = pred->myAllowed;
                pred->myOrigAllowedTargets = pred->myAllowedTargets;
                pred->myOrigClassesViaSuccessorMap = pred->myClassesViaSuccessorMap;
                pred->myHaveTransientPermissions = true;
            }
            pred->rebuildAllowedTargets(false);
        }
        if (MSGlobals::gUseMesoSim) {
            for (MESegment* s = MSGlobals::gMesoNet->getSegmentForEdge(*this); s != nullptr; s = s->getNextSegment()) {
                s->updatePermissions();
            }
        }
    }
}


void
MSEdge::rebuildAllowedTargets(const bool updateVehicles) {
    myAllowedTargets.clear();
    for (const MSEdge* target : mySuccessors) {
        bool universalMap = true; // whether the mapping for SVC_IGNORING is also valid for all vehicle classes
        std::shared_ptr<std::vector<MSLane*> > allLanes = std::make_shared<std::vector<MSLane*> >();
        // compute the mapping for SVC_IGNORING
        for (MSLane* const lane : *myLanes) {
            SVCPermissions combinedTargetPermissions = 0;
            for (const MSLink* const link : lane->getLinkCont()) {
                if (&link->getLane()->getEdge() == target) {
                    allLanes->push_back(lane);
                    combinedTargetPermissions |= link->getLane()->getPermissions();
                    if (link->getViaLane() != nullptr &&
                            ((lane->getPermissions() & link->getLane()->getPermissions()) != link->getViaLane()->getPermissions())) {
                        // custom connection permissions
                        universalMap = false;
                    }
                }
            }
            if (combinedTargetPermissions == 0 || (lane->getPermissions() & combinedTargetPermissions) != lane->getPermissions()) {
                universalMap = false;
            }
        }
        if (universalMap) {
            if (myAllowed.empty()) {
                // we have no lane specific permissions
                myAllowedTargets[target].push_back(std::make_pair(myMinimumPermissions, myLanes));
            } else {
                for (const auto& i : myAllowed) {
                    addToAllowed(i.first, i.second, myAllowedTargets[target]);
                }
            }
        } else {
            addToAllowed(SVC_IGNORING, allLanes, myAllowedTargets[target]);
            // compute the vclass specific mapping
            for (SVCPermissions vclass = SVC_PRIVATE; vclass <= SUMOVehicleClass_MAX; vclass *= 2) {
                if ((myCombinedPermissions & vclass) == vclass) {
                    std::shared_ptr<std::vector<MSLane*> > allowedLanes = std::make_shared<std::vector<MSLane*> >();
                    for (MSLane* const lane : *myLanes) {
                        if (lane->allowsVehicleClass((SUMOVehicleClass)vclass)) {
                            for (const MSLink* const link : lane->getLinkCont()) {
                                if (link->getLane()->allowsVehicleClass((SUMOVehicleClass)vclass) && &link->getLane()->getEdge() == target && (link->getViaLane() == nullptr || link->getViaLane()->allowsVehicleClass((SUMOVehicleClass)vclass))) {
                                    allowedLanes->push_back(lane);
                                }
                            }
                        }
                    }
                    addToAllowed(vclass, allowedLanes, myAllowedTargets[target]);
                }
            }
        }
    }
    if (updateVehicles) {
        for (const MSLane* const lane : *myLanes) {
            const MSLane::VehCont& vehs = lane->getVehiclesSecure();
            for (MSVehicle* veh : vehs) {
                veh->updateBestLanes(true);
            }
            lane->releaseVehicles();
        }
    }
    myClassesSuccessorMap.clear();
}


// ------------ Access to the edge's lanes
MSLane*
MSEdge::leftLane(const MSLane* const lane) const {
    return parallelLane(lane, 1);
}


MSLane*
MSEdge::rightLane(const MSLane* const lane) const {
    return parallelLane(lane, -1);
}


MSLane*
MSEdge::parallelLane(const MSLane* const lane, int offset, bool includeOpposite) const {
    const int resultIndex = lane->getIndex() + offset;
    if (resultIndex >= getNumLanes() && includeOpposite) {
        const MSEdge* opposite = getOppositeEdge();
        if (opposite != nullptr && resultIndex < getNumLanes() + opposite->getNumLanes()) {
            return opposite->getLanes()[opposite->getNumLanes() + getNumLanes() - resultIndex - 1];
        }
        return nullptr;
    } else if (resultIndex >= (int)myLanes->size() || resultIndex < 0) {
        return nullptr;
    } else {
        return (*myLanes)[resultIndex];
    }
}


const std::vector<MSLane*>*
MSEdge::allowedLanes(const MSEdge& destination, SUMOVehicleClass vclass, bool ignoreTransientPermissions) const {
    const auto& targets = ignoreTransientPermissions && myHaveTransientPermissions ? myOrigAllowedTargets : myAllowedTargets;
    AllowedLanesByTarget::const_iterator i = targets.find(&destination);
    if (i != targets.end()) {
        for (const auto& allowed : i->second) {
            if ((allowed.first & vclass) == vclass) {
                return allowed.second.get();
            }
        }
    }
    return nullptr;
}


const std::vector<MSLane*>*
MSEdge::allowedLanes(SUMOVehicleClass vclass) const {
    if ((myMinimumPermissions & vclass) == vclass) {
        return myLanes.get();
    } else {
        if ((myCombinedPermissions & vclass) == vclass) {
            for (const auto& allowed : myAllowed) {
                if ((allowed.first & vclass) == vclass) {
                    return allowed.second.get();
                }
            }
        }
        return nullptr;
    }
}


// ------------
SUMOTime
MSEdge::incVaporization(SUMOTime) {
    ++myVaporizationRequests;
    return 0;
}


SUMOTime
MSEdge::decVaporization(SUMOTime) {
    --myVaporizationRequests;
    return 0;
}


MSLane*
MSEdge::getFreeLane(const std::vector<MSLane*>* allowed, const SUMOVehicleClass vclass, double departPos) const {
    if (allowed == nullptr) {
        allowed = allowedLanes(vclass);
    }
    MSLane* res = nullptr;
    if (allowed != nullptr) {
        double largestGap = 0;
        MSLane* resByGap = nullptr;
        double leastOccupancy = std::numeric_limits<double>::max();
        for (std::vector<MSLane*>::const_iterator i = allowed->begin(); i != allowed->end(); ++i) {
            const double occupancy = (*i)->getBruttoOccupancy();
            if (occupancy < leastOccupancy) {
                res = (*i);
                leastOccupancy = occupancy;
            }
            const MSVehicle* last = (*i)->getLastFullVehicle();
            const double lastGap = (last != nullptr ? last->getPositionOnLane() : myLength) - departPos;
            if (lastGap > largestGap) {
                largestGap = lastGap;
                resByGap = (*i);
            }
        }
        if (resByGap != nullptr) {
            //if (res != resByGap) std::cout << SIMTIME << " edge=" << getID() << " departPos=" << departPos << " res=" << Named::getIDSecure(res) << " resByGap=" << Named::getIDSecure(resByGap) << " largestGap=" << largestGap << "\n";
            res = resByGap;
        }
    }
    return res;
}


MSLane*
MSEdge::getProbableLane(const std::vector<MSLane*>* allowed, const SUMOVehicleClass vclass, double departPos, double maxSpeed) const {
    if (allowed == nullptr) {
        allowed = allowedLanes(vclass);
    }
    MSLane* res = nullptr;
    if (allowed != nullptr) {
        double largestGap = 0;
        double largestSpeed = 0;
        MSLane* resByGap = nullptr;
        double leastOccupancy = std::numeric_limits<double>::max();
        int aIndex = 0;
        for (std::vector<MSLane*>::const_iterator i = allowed->begin(); i != allowed->end(); ++i, aIndex++) {
            const double occupancy = (*i)->getBruttoOccupancy();
            if (occupancy < leastOccupancy) {
                res = (*i);
                leastOccupancy = occupancy;
            }
            const MSVehicle* last = (*i)->getLastFullVehicle();
            double lastGap = (last != nullptr ? last->getPositionOnLane() : myLength) - departPos;
            // never insert to the left of a vehicle with a larger speedFactor
            if (lastGap > largestGap && maxSpeed >= largestSpeed) {
                largestGap = lastGap;
                resByGap = (*i);
            }
            if (last != nullptr) {
                largestSpeed = MAX2(largestSpeed, getVehicleMaxSpeed(last));
            }
        }
        if (resByGap != nullptr) {
            //if (res != resByGap) std::cout << SIMTIME << " edge=" << getID() << " departPos=" << departPos << " res=" << Named::getIDSecure(res) << " resByGap=" << Named::getIDSecure(resByGap) << " largestGap=" << largestGap << "\n";
            res = resByGap;
        }
    }
    return res;
}


double
MSEdge::getDepartPosBound(const MSVehicle& veh, bool upper) const {
    const SUMOVehicleParameter& pars = veh.getParameter();
    double pos = getLength();
    // determine the position
    switch (pars.departPosProcedure) {
        case DepartPosDefinition::GIVEN:
            pos = pars.departPos;
            if (pos < 0.) {
                pos += myLength;
            }
            break;
        case DepartPosDefinition::RANDOM:
            // could be any position on the edge
            break;
        case DepartPosDefinition::RANDOM_FREE:
            // could be any position on the edge due to multiple random attempts
            break;
        case DepartPosDefinition::FREE:
            // many candidate positions, upper bound could be computed exactly
            // with much effort
            break;
        case DepartPosDefinition::LAST:
            if (upper) {
                for (std::vector<MSLane*>::const_iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
                    MSVehicle* last = (*i)->getLastFullVehicle();
                    if (last != nullptr) {
                        pos = MIN2(pos, last->getPositionOnLane());
                    }
                }
            } else {
                pos = 0;
            }
            break;
        case DepartPosDefinition::BASE:
        case DepartPosDefinition::DEFAULT:
            if (!upper) {
                pos = 0;
            }
            break;
        default:
            pos = MIN2(pos, veh.getVehicleType().getLength());
            break;
    }
    return pos;
}

MSLane*
MSEdge::getDepartLaneMeso(SUMOVehicle& veh) const {
    if (veh.getParameter().departLaneProcedure == DepartLaneDefinition::GIVEN) {
        if ((int) myLanes->size() <= veh.getParameter().departLane || !(*myLanes)[veh.getParameter().departLane]->allowsVehicleClass(veh.getVehicleType().getVehicleClass())) {
            return nullptr;
        }
        return (*myLanes)[veh.getParameter().departLane];
    }
    return (*myLanes)[0];
}

MSLane*
MSEdge::getDepartLane(MSVehicle& veh) const {
    switch (veh.getParameter().departLaneProcedure) {
        case DepartLaneDefinition::GIVEN:
            if ((int) myLanes->size() <= veh.getParameter().departLane || !(*myLanes)[veh.getParameter().departLane]->allowsVehicleClass(veh.getVehicleType().getVehicleClass())) {
                return nullptr;
            }
            return (*myLanes)[veh.getParameter().departLane];
        case DepartLaneDefinition::RANDOM:
            return RandHelper::getRandomFrom(*allowedLanes(veh.getVehicleType().getVehicleClass()));
        case DepartLaneDefinition::FREE:
            return getFreeLane(nullptr, veh.getVehicleType().getVehicleClass(), getDepartPosBound(veh, false));
        case DepartLaneDefinition::ALLOWED_FREE:
            if (veh.getRoute().size() == 1) {
                return getFreeLane(nullptr, veh.getVehicleType().getVehicleClass(), getDepartPosBound(veh, false));
            } else {
                return getFreeLane(allowedLanes(**(veh.getRoute().begin() + 1), veh.getVehicleType().getVehicleClass()), veh.getVehicleType().getVehicleClass(), getDepartPosBound(veh, false));
            }
        case DepartLaneDefinition::BEST_FREE:
        case DepartLaneDefinition::BEST_PROB: {
            veh.updateBestLanes(false, myLanes->front());
            const std::vector<MSVehicle::LaneQ>& bl = veh.getBestLanes();
            double bestLength = -1;
            for (std::vector<MSVehicle::LaneQ>::const_iterator i = bl.begin(); i != bl.end(); ++i) {
                if ((*i).length > bestLength) {
                    bestLength = (*i).length;
                }
            }
            // beyond a certain length, all lanes are suitable
            // however, we still need to check departPos to avoid unsuitable insertion
            // (this is only possible in some cases)
            double departPos = 0;
            if (bestLength > BEST_LANE_LOOKAHEAD) {
                departPos = getDepartPosBound(veh);
                bestLength = MIN2(bestLength - departPos, BEST_LANE_LOOKAHEAD);
            }
            std::vector<MSLane*>* bestLanes = new std::vector<MSLane*>();
            for (std::vector<MSVehicle::LaneQ>::const_iterator i = bl.begin(); i != bl.end(); ++i) {
                if (((*i).length - departPos) >= bestLength) {
                    bestLanes->push_back((*i).lane);
                }
            }
            MSLane* ret = nullptr;
            if (veh.getParameter().departLaneProcedure == DepartLaneDefinition::BEST_FREE) {
                ret = getFreeLane(bestLanes, veh.getVehicleType().getVehicleClass(), getDepartPosBound(veh, false));
            } else {
                ret = getProbableLane(bestLanes, veh.getVehicleType().getVehicleClass(), getDepartPosBound(veh, false), getVehicleMaxSpeed(&veh));
            }
            delete bestLanes;
            return ret;
        }
        case DepartLaneDefinition::DEFAULT:
        case DepartLaneDefinition::FIRST_ALLOWED:
            return getFirstAllowed(veh.getVehicleType().getVehicleClass());
        default:
            break;
    }
    if (!(*myLanes)[0]->allowsVehicleClass(veh.getVehicleType().getVehicleClass())) {
        return nullptr;
    }
    return (*myLanes)[0];
}


MSLane*
MSEdge::getFirstAllowed(SUMOVehicleClass vClass, bool defaultFirst) const {
    for (std::vector<MSLane*>::const_iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
        if ((*i)->allowsVehicleClass(vClass)) {
            return *i;
        }
    }
    return defaultFirst && !myLanes->empty() ? myLanes->front() : nullptr;
}


bool
MSEdge::validateDepartSpeed(SUMOVehicle& v) const {
    const SUMOVehicleParameter& pars = v.getParameter();
    const MSVehicleType& type = v.getVehicleType();
    if (pars.departSpeedProcedure == DepartSpeedDefinition::GIVEN) {
        // departSpeed could have been rounded down in the output
        double vMax = getVehicleMaxSpeed(&v) + SPEED_EPS;
        if (pars.departSpeed > vMax) {
            // check departLane (getVehicleMaxSpeed checks lane 0)
            MSLane* departLane = MSGlobals::gMesoNet ? getDepartLaneMeso(v) : getDepartLane(dynamic_cast<MSVehicle&>(v));
            if (departLane != nullptr) {
                vMax = departLane->getVehicleMaxSpeed(&v);
                if (pars.wasSet(VEHPARS_SPEEDFACTOR_SET)) {
                    // speedFactor could have been rounded down in the output
                    vMax *= (1 + SPEED_EPS);
                }
                // additive term must come after multiplication!
                vMax += SPEED_EPS;
                if (pars.departSpeed > vMax) {
                    const std::vector<double>& speedFactorParams = type.getSpeedFactor().getParameter();
                    if (speedFactorParams[1] > 0.) {
                        v.setChosenSpeedFactor(type.computeChosenSpeedDeviation(nullptr, pars.departSpeed / MIN2(getSpeedLimit(), type.getDesiredMaxSpeed() - SPEED_EPS)));
                        if (v.getChosenSpeedFactor() > speedFactorParams[0] + 2 * speedFactorParams[1]) {
                            // only warn for significant deviation
                            WRITE_WARNINGF(TL("Choosing new speed factor % for vehicle '%' to match departure speed % (max %)."),
                                           toString(v.getChosenSpeedFactor()), pars.id, pars.departSpeed, vMax);
                        }
                    } else {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}


bool
MSEdge::insertVehicle(SUMOVehicle& v, SUMOTime time, const bool checkOnly, const bool forceCheck) const {
    // when vaporizing, no vehicles are inserted, but checking needs to be successful to trigger removal
    if (isVaporizing() || isTazConnector()
            || v.getRouteValidity(true, checkOnly) != MSBaseVehicle::ROUTE_VALID) {
        return checkOnly;
    }
    const SUMOVehicleParameter& pars = v.getParameter();
    if (!validateDepartSpeed(v)) {
        const std::string errorMsg = "Departure speed for vehicle '" + pars.id + "' is too high for the departure edge '" + getID() + "'.";
        if (MSGlobals::gCheckRoutes) {
            throw ProcessError(errorMsg);
        } else {
            WRITE_WARNING(errorMsg);
        }
    }
    if (MSGlobals::gUseMesoSim) {
        if (!forceCheck && myLastFailedInsertionTime == time) {
            return false;
        }
        double pos = 0.0;
        switch (pars.departPosProcedure) {
            case DepartPosDefinition::GIVEN:
                if (pars.departPos >= 0.) {
                    pos = pars.departPos;
                } else {
                    pos = pars.departPos + getLength();
                }
                if (pos < 0 || pos > getLength()) {
                    WRITE_WARNING("Invalid departPos " + toString(pos) + " given for vehicle '" +
                                  v.getID() + "'. Inserting at lane end instead.");
                    pos = getLength();
                }
                break;
            case DepartPosDefinition::RANDOM:
            case DepartPosDefinition::RANDOM_FREE:
                pos = RandHelper::rand(getLength());
                break;
            default:
                break;
        }
        bool result = false;
        MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this, pos);
        MEVehicle* veh = static_cast<MEVehicle*>(&v);
        int qIdx;
        if (pars.departPosProcedure == DepartPosDefinition::FREE) {
            while (segment != nullptr && !result) {
                if (checkOnly) {
                    result = segment->hasSpaceFor(veh, time, qIdx, true) == time;
                } else {
                    result = segment->initialise(veh, time);
                }
                segment = segment->getNextSegment();
            }
        } else {
            if (checkOnly) {
                result = segment->hasSpaceFor(veh, time, qIdx, true) == time;
            } else {
                result = segment->initialise(veh, time);
            }
        }
        return result;
    }
    if (checkOnly) {
        switch (v.getParameter().departLaneProcedure) {
            case DepartLaneDefinition::GIVEN:
            case DepartLaneDefinition::DEFAULT:
            case DepartLaneDefinition::FIRST_ALLOWED: {
                MSLane* insertionLane = getDepartLane(static_cast<MSVehicle&>(v));
                if (insertionLane == nullptr) {
                    WRITE_WARNING("could not insert vehicle '" + v.getID() + "' on any lane of edge '" + getID() + "', time=" + time2string(MSNet::getInstance()->getCurrentTimeStep()));
                    return false;
                }
                const double occupancy = insertionLane->getBruttoOccupancy();
                return (occupancy == 0 || occupancy * myLength + v.getVehicleType().getLengthWithGap() <= myLength ||
                        v.getParameter().departProcedure == DepartDefinition::SPLIT);
            }
            default:
                for (std::vector<MSLane*>::const_iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
                    const double occupancy = (*i)->getBruttoOccupancy();
                    if (occupancy == 0 || occupancy * myLength + v.getVehicleType().getLengthWithGap() <= myLength ||
                            v.getParameter().departProcedure == DepartDefinition::SPLIT) {
                        return true;
                    }
                }
        }
        return false;
    }
    MSLane* insertionLane = getDepartLane(static_cast<MSVehicle&>(v));
    if (insertionLane == nullptr) {
        return false;
    }

    if (!forceCheck) {
        if (myLastFailedInsertionTime == time) {
            if (myFailedInsertionMemory.count(insertionLane->getIndex())) {
                // A vehicle was already rejected for the proposed insertionLane in this timestep
                return false;
            }
        } else {
            // last rejection occurred in a previous timestep, clear cache
            myFailedInsertionMemory.clear();
        }
    }

    bool success = insertionLane->insertVehicle(static_cast<MSVehicle&>(v));

    if (!success) {
        // constraints may enforce explicit re-ordering so we need to try other vehicles after failure
        if (!insertionLane->hasParameter("insertionOrder" + v.getID())) {
            myFailedInsertionMemory.insert(insertionLane->getIndex());
        }
    }
    return success;
}


void
MSEdge::changeLanes(SUMOTime t) const {
    if (myLaneChanger != nullptr) {
        myLaneChanger->laneChange(t);
    }
}


const MSEdge*
MSEdge::getInternalFollowingEdge(const MSEdge* followerAfterInternal, SUMOVehicleClass vClass) const {
    //@todo to be optimized
    for (const MSLane* const l : *myLanes) {
        for (const MSLink* const link : l->getLinkCont()) {
            if (&link->getLane()->getEdge() == followerAfterInternal) {
                if (link->getViaLane() != nullptr) {
                    if (link->getViaLane()->allowsVehicleClass(vClass)) {
                        return &link->getViaLane()->getEdge();
                    } else {
                        continue;
                    }
                } else {
                    return nullptr; // network without internal links
                }
            }
        }
    }
    return nullptr;
}


double
MSEdge::getInternalFollowingLengthTo(const MSEdge* followerAfterInternal, SUMOVehicleClass vClass) const {
    assert(followerAfterInternal != 0);
    assert(!followerAfterInternal->isInternal());
    double dist = 0.;
    const MSEdge* edge = getInternalFollowingEdge(followerAfterInternal, vClass);
    // Take into account non-internal lengths until next non-internal edge
    while (edge != nullptr && edge->isInternal()) {
        dist += edge->getLength();
        edge = edge->getInternalFollowingEdge(followerAfterInternal, vClass);
    }
    return dist;
}


const MSEdge*
MSEdge::getNormalBefore() const {
    const MSEdge* result = this;
    while (result->isInternal() && MSGlobals::gUsingInternalLanes) {
        assert(result->getPredecessors().size() == 1);
        result = result->getPredecessors().front();
    }
    return result;
}

const MSEdge*
MSEdge::getNormalSuccessor() const {
    const MSEdge* result = this;
    while (result->isInternal()) {
        assert(result->getSuccessors().size() == 1);
        result = result->getSuccessors().front();
    }
    return result;
}

double
MSEdge::getMeanSpeed() const {
    double v = 0;
    double totalNumVehs = 0;
    if (MSGlobals::gUseMesoSim) {
        for (MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this); segment != nullptr; segment = segment->getNextSegment()) {
            const int numVehs = segment->getCarNumber();
            if (numVehs > 0) {
                v += numVehs * segment->getMeanSpeed();
                totalNumVehs += numVehs;
            }
        }
        if (totalNumVehs == 0) {
            return getLength() / myEmptyTraveltime; // may include tls-penalty
        }
    } else {
        for (const MSLane* const lane : *myLanes) {
            int numVehs = lane->getVehicleNumber();
            if (numVehs == 0) {
                // take speed limit but with lowest possible weight
                numVehs = 1;
            }
            v += numVehs * lane->getMeanSpeed();
            totalNumVehs += numVehs;
        }
        if (myBidiEdge != nullptr) {
            for (const MSLane* const lane : myBidiEdge->getLanes()) {
                if (lane->getVehicleNumber() > 0) {
                    // do not route across edges which are already occupied in reverse direction
                    return 0;
                }
            }
        }
        if (totalNumVehs == 0) {
            return getSpeedLimit();
        }
    }
    return v / totalNumVehs;
}


double
MSEdge::getMeanFriction() const {
    double f = 0.;
    for (const MSLane* const lane : *myLanes) {
        f += lane->getFrictionCoefficient();
    }
    if (!myLanes->empty()) {
        return f / (double)myLanes->size();
    }
    return 1.;
}


double
MSEdge::getMeanSpeedBike() const {
    if (MSGlobals::gUseMesoSim) {
        // no separate bicycle speeds in meso
        return getMeanSpeed();
    }
    double v = 0;
    double totalNumVehs = 0;
    for (const MSLane* const lane : *myLanes) {
        const int numVehs = lane->getVehicleNumber();
        v += numVehs * lane->getMeanSpeedBike();
        totalNumVehs += numVehs;
    }
    if (totalNumVehs == 0) {
        return getSpeedLimit();
    }
    return v / totalNumVehs;
}


double
MSEdge::getCurrentTravelTime(double minSpeed) const {
    assert(minSpeed > 0);
    if (!myAmDelayed) {
        return myEmptyTraveltime;
    }
    return getLength() / MAX2(minSpeed, getMeanSpeed());
}


double
MSEdge::getRoutingSpeed() const {
    return MSRoutingEngine::getAssumedSpeed(this, nullptr);
}


bool
MSEdge::dictionary(const std::string& id, MSEdge* ptr) {
    const DictType::iterator it = myDict.lower_bound(id);
    if (it == myDict.end() || it->first != id) {
        // id not in myDict
        myDict.emplace_hint(it, id, ptr);
        while (ptr->getNumericalID() >= (int)myEdges.size()) {
            myEdges.push_back(nullptr);
        }
        myEdges[ptr->getNumericalID()] = ptr;
        return true;
    }
    return false;
}


MSEdge*
MSEdge::dictionary(const std::string& id) {
    const DictType::iterator it = myDict.find(id);
    if (it == myDict.end()) {
        return nullptr;
    }
    return it->second;
}


MSEdge*
MSEdge::dictionaryHint(const std::string& id, const int startIdx) {
    // this method is mainly useful when parsing connections from the net.xml which are sorted by "from" id
    if (myEdges[startIdx] != nullptr && myEdges[startIdx]->getID() == id) {
        return myEdges[startIdx];
    }
    if (startIdx + 1 < (int)myEdges.size() && myEdges[startIdx + 1] != nullptr && myEdges[startIdx + 1]->getID() == id) {
        return myEdges[startIdx + 1];
    }
    return dictionary(id);
}


const MSEdgeVector&
MSEdge::getAllEdges() {
    return myEdges;
}


void
MSEdge::clear() {
    for (DictType::iterator i = myDict.begin(); i != myDict.end(); ++i) {
        delete (*i).second;
    }
    myDict.clear();
    myEdges.clear();
}


void
MSEdge::insertIDs(std::vector<std::string>& into) {
    for (DictType::iterator i = myDict.begin(); i != myDict.end(); ++i) {
        into.push_back((*i).first);
    }
}


void
MSEdge::parseEdgesList(const std::string& desc, ConstMSEdgeVector& into,
                       const std::string& rid) {
    StringTokenizer st(desc);
    parseEdgesList(st.getVector(), into, rid);
}


void
MSEdge::parseEdgesList(const std::vector<std::string>& desc, ConstMSEdgeVector& into,
                       const std::string& rid) {
    for (std::vector<std::string>::const_iterator i = desc.begin(); i != desc.end(); ++i) {
        const MSEdge* edge = MSEdge::dictionary(*i);
        // check whether the edge exists
        if (edge == nullptr) {
            throw ProcessError("The edge '" + *i + "' within the route " + rid + " is not known."
                               + "\n The route can not be build.");
        }
        into.push_back(edge);
    }
}


double
MSEdge::getDistanceTo(const MSEdge* other, const bool doBoundaryEstimate) const {
    assert(this != other);
    if (doBoundaryEstimate) {
        return myBoundary.distanceTo2D(other->myBoundary);
    }
    if (isTazConnector()) {
        if (other->isTazConnector()) {
            return myBoundary.distanceTo2D(other->myBoundary);
        }
        return myBoundary.distanceTo2D(other->getLanes()[0]->getShape()[0]);
    }
    if (other->isTazConnector()) {
        return other->myBoundary.distanceTo2D(getLanes()[0]->getShape()[-1]);
    }
    return getLanes()[0]->getShape()[-1].distanceTo2D(other->getLanes()[0]->getShape()[0]);
}


const Position
MSEdge::getStopPosition(const SUMOVehicleParameter::Stop& stop) {
    return MSLane::dictionary(stop.lane)->geometryPositionAtOffset((stop.endPos + stop.startPos) / 2.);
}


double
MSEdge::getSpeedLimit() const {
    // @note lanes might have different maximum speeds in theory
    return myLanes->empty() ? 1 : getLanes()[0]->getSpeedLimit();
}


double
MSEdge::getLengthGeometryFactor() const {
    return myLanes->empty() ? 1 : getLanes()[0]->getLengthGeometryFactor();
}

double
MSEdge::getVehicleMaxSpeed(const SUMOTrafficObject* const veh) const {
    // @note lanes might have different maximum speeds in theory
    return myLanes->empty() ? 1 : getLanes()[0]->getVehicleMaxSpeed(veh);
}


void
MSEdge::setMaxSpeed(double val, double jamThreshold) {
    assert(val >= 0);
    if (myLanes != nullptr) {
        for (std::vector<MSLane*>::const_iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
            (*i)->setMaxSpeed(val, false, false, jamThreshold);
        }
    }
}


void
MSEdge::addTransportable(MSTransportable* t) const {
    if (t->isPerson()) {
        myPersons.insert(t);
    } else {
        myContainers.insert(t);
    }
}

void
MSEdge::removeTransportable(MSTransportable* t) const {
    std::set<MSTransportable*, ComparatorNumericalIdLess>& tc = t->isPerson() ? myPersons : myContainers;
    auto it = tc.find(t);
    if (it != tc.end()) {
        tc.erase(it);
    }
}

std::vector<MSTransportable*>
MSEdge::getSortedPersons(SUMOTime timestep, bool includeRiding) const {
    std::vector<MSTransportable*> result(myPersons.begin(), myPersons.end());
    if (includeRiding) {
        for (std::vector<MSLane*>::const_iterator i = myLanes->begin(); i != myLanes->end(); ++i) {
            const MSLane::VehCont& vehs = (*i)->getVehiclesSecure();
            for (MSLane::VehCont::const_iterator j = vehs.begin(); j != vehs.end(); ++j) {
                const std::vector<MSTransportable*>& persons = (*j)->getPersons();
                result.insert(result.end(), persons.begin(), persons.end());
            }
            (*i)->releaseVehicles();
        }
    }
    sort(result.begin(), result.end(), transportable_by_position_sorter(timestep));
    return result;
}


std::vector<MSTransportable*>
MSEdge::getSortedContainers(SUMOTime timestep, bool /* includeRiding */) const {
    std::vector<MSTransportable*> result(myContainers.begin(), myContainers.end());
    sort(result.begin(), result.end(), transportable_by_position_sorter(timestep));
    return result;
}


int
MSEdge::transportable_by_position_sorter::operator()(const MSTransportable* const c1, const MSTransportable* const c2) const {
    const double pos1 = c1->getCurrentStage()->getEdgePos(myTime);
    const double pos2 = c2->getCurrentStage()->getEdgePos(myTime);
    if (pos1 != pos2) {
        return pos1 < pos2;
    }
    return c1->getID() < c2->getID();
}


void
MSEdge::addSuccessor(MSEdge* edge, const MSEdge* via) {
    mySuccessors.push_back(edge);
    myViaSuccessors.push_back(std::make_pair(edge, via));
    if (isTazConnector() && edge->getFromJunction() != nullptr) {
        myBoundary.add(edge->getFromJunction()->getPosition());
    }

    edge->myPredecessors.push_back(this);
    if (edge->isTazConnector() && getToJunction() != nullptr) {
        edge->myBoundary.add(getToJunction()->getPosition());
    }
}


const MSEdgeVector&
MSEdge::getSuccessors(SUMOVehicleClass vClass) const {
    if (vClass == SVC_IGNORING || !MSNet::getInstance()->hasPermissions() || myFunction == SumoXMLEdgeFunc::CONNECTOR) {
        return mySuccessors;
    }
#ifdef HAVE_FOX
    ScopedLocker<> lock(mySuccessorMutex, MSGlobals::gNumThreads > 1);
#endif
    std::map<SUMOVehicleClass, MSEdgeVector>::iterator i = myClassesSuccessorMap.find(vClass);
    if (i == myClassesSuccessorMap.end()) {
        // instantiate vector
        myClassesSuccessorMap[vClass];
        i = myClassesSuccessorMap.find(vClass);
        // this vClass is requested for the first time. rebuild all successors
        for (MSEdgeVector::const_iterator it = mySuccessors.begin(); it != mySuccessors.end(); ++it) {
            if ((*it)->isTazConnector()) {
                i->second.push_back(*it);
            } else {
                const std::vector<MSLane*>* allowed = allowedLanes(**it, vClass);
                if (allowed != nullptr && allowed->size() > 0) {
                    i->second.push_back(*it);
                }
            }
        }
    }
    // can use cached value
    return i->second;
}


const MSConstEdgePairVector&
MSEdge::getViaSuccessors(SUMOVehicleClass vClass, bool ignoreTransientPermissions) const {
    if (vClass == SVC_IGNORING || !MSNet::getInstance()->hasPermissions() || myFunction == SumoXMLEdgeFunc::CONNECTOR) {
        return myViaSuccessors;
    }
#ifdef HAVE_FOX
    ScopedLocker<> lock(mySuccessorMutex, MSGlobals::gNumThreads > 1);
#endif
    auto& viaMap = ignoreTransientPermissions && myHaveTransientPermissions ? myOrigClassesViaSuccessorMap : myClassesViaSuccessorMap;
    auto i = viaMap.find(vClass);
    if (i != viaMap.end()) {
        // can use cached value
        return i->second;
    }
    // instantiate vector
    MSConstEdgePairVector& result = viaMap[vClass];
    // this vClass is requested for the first time. rebuild all successors
    for (const auto& viaPair : myViaSuccessors) {
        if (viaPair.first->isTazConnector()) {
            result.push_back(viaPair);
        } else {
            const std::vector<MSLane*>* allowed = allowedLanes(*viaPair.first, vClass, ignoreTransientPermissions);
            if (allowed != nullptr && allowed->size() > 0) {
                result.push_back(viaPair);
            }
        }
    }
    return result;
}


void
MSEdge::setJunctions(MSJunction* from, MSJunction* to) {
    myFromJunction = from;
    myToJunction = to;
    if (!isTazConnector()) {
        myBoundary.add(from->getPosition());
        myBoundary.add(to->getPosition());
    }
}


bool
MSEdge::canChangeToOpposite() const {
    return (!myLanes->empty() && myLanes->back()->getOpposite() != nullptr &&
            // do not change on curved internal lanes
            (!isInternal()
             || (MSGlobals::gUsingInternalLanes
                 && myLanes->back()->getIncomingLanes()[0].viaLink->getDirection() == LinkDirection::STRAIGHT)));
}


const MSEdge*
MSEdge::getOppositeEdge() const {
    if (!myLanes->empty() && myLanes->back()->getOpposite() != nullptr) {
        return &(myLanes->back()->getOpposite()->getEdge());
    } else {
        return nullptr;
    }
}


bool
MSEdge::hasMinorLink() const {
    for (const MSLane* const l : *myLanes) {
        for (const MSLink* const link : l->getLinkCont()) {
            if (!link->havePriority()) {
                return true;
            }
        }
    }
    return false;
}

bool
MSEdge::hasChangeProhibitions(SUMOVehicleClass svc, int index) const {
    if (myLanes->size() == 1) {
        return false;
    }
    for (const MSLane* const l : *myLanes) {
        if (l->getIndex() <= index && !l->allowsChangingRight(svc) && l->getIndex() > 0) {
            return true;
        } else if (l->getIndex() >= index && !l->allowsChangingLeft(svc) && l->getIndex() < (int)(myLanes->size() - 1)) {
            return true;
        }
    }
    return false;
}

void
MSEdge::checkAndRegisterBiDirEdge(const std::string& bidiID) {
    if (bidiID != "") {
        myBidiEdge = dictionary(bidiID);
        if (myBidiEdge == nullptr) {
            WRITE_ERRORF(TL("Bidi-edge '%' does not exist"), bidiID);
        }
        setBidiLanes();
        return;
    }
    if (getFunction() != SumoXMLEdgeFunc::NORMAL) {
        return;
    }
    // legacy networks (no bidi attribute)
    ConstMSEdgeVector candidates = myToJunction->getOutgoing();
    for (ConstMSEdgeVector::const_iterator it = candidates.begin(); it != candidates.end(); it++) {
        if ((*it)->getToJunction() == myFromJunction) { //reverse edge
            if (myBidiEdge != nullptr && isSuperposable(*it)) {
                WRITE_WARNINGF(TL("Ambiguous superposable edges between junction '%' and '%'."), myToJunction->getID(), myFromJunction->getID());
                break;
            }
            if (isSuperposable(*it)) {
                myBidiEdge = *it;
                setBidiLanes();
            }
        }
    }
}


void
MSEdge::setBidiLanes() {
    assert(myBidiEdge != nullptr);
    if (getNumLanes() == 1 && myBidiEdge->getNumLanes() == 1) {
        // the other way round is set when this method runs for the bidiEdge
        getLanes()[0]->setBidiLane(myBidiEdge->getLanes()[0]);
    } else {
        // find lanes with matching reversed shapes
        int numBidiLanes = 0;
        for (MSLane* l1 : *myLanes) {
            for (MSLane* l2 : *myBidiEdge->myLanes) {
                if (l1->getShape().reverse().almostSame(l2->getShape(), POSITION_EPS * 2)) {
                    l1->setBidiLane(l2);
                    numBidiLanes++;
                }
            }
        }
        // warn only once for each pair
        if (numBidiLanes == 0 && getNumericalID() < myBidiEdge->getNumericalID()) {
            WRITE_WARNINGF(TL("Edge '%s' and bidi edge '%s' have no matching bidi lanes"), getID(), myBidiEdge->getID());
        }
    }
}


bool
MSEdge::isSuperposable(const MSEdge* other) {
    if (other == nullptr || other->getLanes().size() != myLanes->size()) {
        return false;
    }
    std::vector<MSLane*>::const_iterator it1 = myLanes->begin();
    std::vector<MSLane*>::const_reverse_iterator it2 = other->getLanes().rbegin();
    do {
        if ((*it1)->getShape().reverse() != (*it2)->getShape()) {
            return false;
        }
        it1++;
        it2++;
    } while (it1 != myLanes->end());

    return true;
}


void
MSEdge::addWaiting(SUMOVehicle* vehicle) const {
#ifdef HAVE_FOX
    ScopedLocker<> lock(myWaitingMutex, MSGlobals::gNumSimThreads > 1);
#endif
    myWaiting.push_back(vehicle);
}


void
MSEdge::removeWaiting(const SUMOVehicle* vehicle) const {
#ifdef HAVE_FOX
    ScopedLocker<> lock(myWaitingMutex, MSGlobals::gNumSimThreads > 1);
#endif
    std::vector<SUMOVehicle*>::iterator it = std::find(myWaiting.begin(), myWaiting.end(), vehicle);
    if (it != myWaiting.end()) {
        myWaiting.erase(it);
    }
}


SUMOVehicle*
MSEdge::getWaitingVehicle(MSTransportable* transportable, const double position) const {
#ifdef HAVE_FOX
    ScopedLocker<> lock(myWaitingMutex, MSGlobals::gNumSimThreads > 1);
#endif
    for (SUMOVehicle* const vehicle : myWaiting) {
        if (transportable->isWaitingFor(vehicle)) {
            if (vehicle->isStoppedInRange(position, MSGlobals::gStopTolerance) ||
                    (!vehicle->hasDeparted() &&
                     (vehicle->getParameter().departProcedure == DepartDefinition::TRIGGERED ||
                      vehicle->getParameter().departProcedure == DepartDefinition::CONTAINER_TRIGGERED))) {
                return vehicle;
            }
            if (!vehicle->isLineStop(position) && vehicle->allowsBoarding(transportable)) {
                WRITE_WARNING((transportable->isPerson() ? "Person '" : "Container '")
                              + transportable->getID() + "' at edge '" + getID() + "' position " + toString(position) + " cannot use waiting vehicle '"
                              + vehicle->getID() + "' at position " + toString(vehicle->getPositionOnLane()) + " because it is too far away.");
            }
        }
    }
    return nullptr;
}

std::vector<const SUMOVehicle*>
MSEdge::getVehicles() const {
    std::vector<const SUMOVehicle*> result;
    if (MSGlobals::gUseMesoSim) {
        for (MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this); segment != nullptr; segment = segment->getNextSegment()) {
            std::vector<const MEVehicle*> segmentVehs = segment->getVehicles();
            result.insert(result.end(), segmentVehs.begin(), segmentVehs.end());
        }
    } else {
        for (MSLane* lane : getLanes()) {
            for (auto veh : lane->getVehiclesSecure()) {
                result.push_back(veh);
            }
            lane->releaseVehicles();
        }
    }
    return result;
}

int
MSEdge::getNumDrivingLanes() const {
    int result = 0;
    SVCPermissions filter = SVCAll;
    if ((myCombinedPermissions & ~(SVC_PEDESTRIAN | SVC_WHEELCHAIR)) != 0) {
        filter = ~(SVC_PEDESTRIAN | SVC_WHEELCHAIR);
    } else if ((myCombinedPermissions & (SVC_PEDESTRIAN | SVC_WHEELCHAIR)) != 0) {
        // filter out green verge
        filter = (SVC_PEDESTRIAN | SVC_WHEELCHAIR);
    }
    for (const MSLane* const l : *myLanes) {
        if ((l->getPermissions() & filter) != 0) {
            result++;
        }
    }
    return result;
}

int
MSEdge::getVehicleNumber() const {
    return (int)getVehicles().size();
}


bool
MSEdge::isEmpty() const {
    /// more efficient than retrieving vehicle number
    if (MSGlobals::gUseMesoSim) {
        for (MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this); segment != nullptr; segment = segment->getNextSegment()) {
            if (segment->getCarNumber() > 0) {
                return false;
            }
        }
    } else {
        for (MSLane* lane : getLanes()) {
            if (lane->getVehicleNumber() > 0) {
                return false;
            }
        }
    }
    return true;
}


double
MSEdge::getWaitingSeconds() const {
    double wtime = 0;
    if (MSGlobals::gUseMesoSim) {
        for (MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this); segment != nullptr; segment = segment->getNextSegment()) {
            wtime += segment->getWaitingSeconds();
        }
    } else {
        for (MSLane* lane : getLanes()) {
            wtime += lane->getWaitingSeconds();
        }
    }
    return wtime;
}


double
MSEdge::getOccupancy() const {
    if (myLanes->size() == 0) {
        return 0;
    }
    if (MSGlobals::gUseMesoSim) {
        /// @note MESegment only tracks brutto occupancy so we compute this from sratch
        double sum = 0;
        for (const SUMOVehicle* veh : getVehicles()) {
            sum += dynamic_cast<const MEVehicle*>(veh)->getVehicleType().getLength();
        }
        return sum / (myLength * (double)myLanes->size());
    } else {
        double sum = 0;
        for (auto lane : getLanes()) {
            sum += lane->getNettoOccupancy();
        }
        return sum / (double)myLanes->size();
    }
}


double
MSEdge::getFlow() const {
    if (myLanes->size() == 0) {
        return 0;
    }
    double flow = 0;
    for (MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this); segment != nullptr; segment = segment->getNextSegment()) {
        flow += (double) segment->getCarNumber() * segment->getMeanSpeed();
    }
    return 3600 * flow / (*myLanes)[0]->getLength();
}


double
MSEdge::getBruttoOccupancy() const {
    if (myLanes->size() == 0) {
        return 0;
    }
    double occ = 0;
    for (MESegment* segment = MSGlobals::gMesoNet->getSegmentForEdge(*this); segment != nullptr; segment = segment->getNextSegment()) {
        occ += segment->getBruttoOccupancy();
    }
    return occ / (*myLanes)[0]->getLength() / (double)(myLanes->size());
}

double
MSEdge::getTravelTimeAggregated(const MSEdge* const edge, const SUMOVehicle* const veh, double /*time*/) {
    return edge->getLength() / MIN2(MSRoutingEngine::getAssumedSpeed(edge, veh), veh->getMaxSpeed());
}


void
MSEdge::inferEdgeType() {
    // @note must be called after closeBuilding() to ensure successors and
    // predecessors are set
    if (isInternal() && myEdgeType == "") {
        const std::string typeBefore = getNormalBefore()->getEdgeType();
        if (typeBefore != "") {
            const std::string typeAfter = getNormalSuccessor()->getEdgeType();
            if (typeBefore == typeAfter) {
                myEdgeType = typeBefore;
            } else if (typeAfter != "") {
                MSNet* net = MSNet::getInstance();
                auto resBefore = net->getRestrictions(typeBefore);
                auto resAfter = net->getRestrictions(typeAfter);
                if (resBefore != nullptr && resAfter != nullptr) {
                    // create new restrictions for this type-combination
                    myEdgeType = typeBefore + "|" + typeAfter;
                    if (net->getRestrictions(myEdgeType) == nullptr) {
                        for (const auto& item : *resBefore) {
                            const SUMOVehicleClass svc = item.first;
                            const double speed = item.second;
                            const auto it = (*resAfter).find(svc);
                            if (it != (*resAfter).end()) {
                                const double speed2 = it->second;
                                const double newSpeed = (MSNet::getInstance()->hasJunctionHigherSpeeds()
                                                         ? MAX2(speed, speed2) : (speed + speed2) / 2);
                                net->addRestriction(myEdgeType, svc, newSpeed);
                            }
                        }
                    }
                }
            }
        }
    }
}


double
MSEdge::getDistanceAt(double pos) const {
    // negative values of myDistances indicate descending kilometrage
    return fabs(myDistance + pos);
}


bool
MSEdge::hasTransientPermissions() const {
    return myHaveTransientPermissions;
}


void
MSEdge::clearState() {
    myPersons.clear();
    myContainers.clear();
    myWaiting.clear();
}

/****************************************************************************/
