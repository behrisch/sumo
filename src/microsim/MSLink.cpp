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
/// @file    MSLink.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Laura Bieker
/// @date    Sept 2002
///
// A connection between lanes
/****************************************************************************/
#include <config.h>

#include <iostream>
#include <algorithm>
#include <limits>
#include <utils/iodevices/OutputDevice.h>
#include <utils/common/RandHelper.h>
#include <utils/common/StringTokenizer.h>
#include "MSNet.h"
#include "MSJunction.h"
#include "MSJunctionLogic.h"
#include "MSLink.h"
#include "MSLane.h"
#include <microsim/transportables/MSPerson.h>
#include <microsim/transportables/MSTransportableControl.h>
#include "MSEdge.h"
#include "MSGlobals.h"
#include "MSVehicle.h"
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <microsim/transportables/MSPModel.h>

//#define MSLink_DEBUG_CROSSING_POINTS
//#define MSLink_DEBUG_CROSSING_POINTS_DETAILS
//#define MSLink_DEBUG_OPENED
//#define DEBUG_APPROACHING
//#define DEBUG_ZIPPER
//#define DEBUG_WALKINGAREA
//#define DEBUG_COND (myLane->getID()=="43[0]_0" && myLaneBefore->getID()==":33_0_0")
//#define DEBUG_COND (myLane->getID()=="end_0")
//#define DEBUG_COND (true)
#define DEBUG_COND2(obj) (obj->isSelected())
//#define DEBUG_COND2(obj) (obj->getID() == "train2")
//#define DEBUG_COND_ZIPPER (gDebugFlag1)
//#define DEBUG_COND_ZIPPER (true)
#define DEBUG_COND_ZIPPER (ego->isSelected())

// ===========================================================================
// static member variables
// ===========================================================================

#define INVALID_TIME -1000

// the default safety gap when passing before oncoming pedestrians
#define JM_CROSSING_GAP_DEFAULT 10

// minimim width between sibling lanes to qualify as non-overlapping
#define DIVERGENCE_MIN_WIDTH 2.5

const SUMOTime MSLink::myLookaheadTime = TIME2STEPS(1);
// additional caution is needed when approaching a zipper link
const SUMOTime MSLink::myLookaheadTimeZipper = TIME2STEPS(4);
std::set<std::pair<MSLink*, MSLink*> > MSLink::myRecheck;
const double MSLink::NO_INTERSECTION(10000);

// ===========================================================================
// ConflictInfo member method definitions
// ===========================================================================

double
MSLink::ConflictInfo::getFoeLengthBehindCrossing(const MSLink* foeExitLink) const {
    if (flag == CONFLICT_DUMMY_MERGE) {
        return 0;
    } else if (foeConflictIndex >= 0) {
        return foeExitLink->myConflicts[foeConflictIndex].lengthBehindCrossing;
    } else {
        return -NO_INTERSECTION;
    }
}

double
MSLink::ConflictInfo::getFoeConflictSize(const MSLink* foeExitLink) const {
    if (foeConflictIndex >= 0) {
        return foeExitLink->myConflicts[foeConflictIndex].conflictSize;
    } else {
        return 0;
    }
}

double
MSLink::ConflictInfo::getLengthBehindCrossing(const MSLink* exitLink) const {
    if (flag == CONFLICT_STOP_AT_INTERNAL_JUNCTION) {
        return exitLink->getInternalLaneBefore()->getLength();
    } else {
        return lengthBehindCrossing;
    }
}

// ===========================================================================
// member method definitions
// ===========================================================================
MSLink::MSLink(MSLane* predLane, MSLane* succLane, MSLane* via, LinkDirection dir, LinkState state,
               double length, double foeVisibilityDistance, bool keepClear,
               MSTrafficLightLogic* logic, int tlIndex,
               bool indirect) :
    myLane(succLane),
    myLaneBefore(predLane),
    myApproachingPersons(nullptr),
    myIndex(-1),
    myTLIndex(tlIndex),
    myLogic(logic),
    myState(state),
    myLastGreenState(LINKSTATE_TL_GREEN_MINOR),
    myOffState(state),
    myLastStateChange(SUMOTime_MIN / 2), // a large negative value, but avoid overflows when subtracting
    myDirection(dir),
    myLength(length),
    myFoeVisibilityDistance(foeVisibilityDistance),
    myDistToFoePedCrossing(std::numeric_limits<double>::max()),
    myHasFoes(false),
    myAmCont(false),
    myAmContOff(false),
    myKeepClear(keepClear),
    myInternalLane(via),
    myInternalLaneBefore(nullptr),
    myMesoTLSPenalty(0),
    myGreenFraction(1),
    myLateralShift(0),
    myOffFoeLinks(nullptr),
    myWalkingAreaFoe(nullptr),
    myWalkingAreaFoeExit(nullptr),
    myHavePedestrianCrossingFoe(false),
    myParallelRight(nullptr),
    myParallelLeft(nullptr),
    myAmIndirect(indirect),
    myRadius(std::numeric_limits<double>::max()),
    myPermissions(myLaneBefore->getPermissions() & myLane->getPermissions() & (via == nullptr ? SVCAll : via->getPermissions())),
    myJunction(nullptr) {

    if (MSGlobals::gLateralResolution > 0) {
        // detect lateral shift from lane geometries
        //std::cout << "DEBUG link=" << myLaneBefore->getID() << "->" << getViaLaneOrLane()->getID() << " hasInternal=" << MSNet::getInstance()->hasInternalLinks() << " shapeBefore=" << myLaneBefore->getShape().back() << " shapeFront=" << getViaLaneOrLane()->getShape().front() << "\n";
        if ((myInternalLane != nullptr || predLane->isInternal())
                && myLaneBefore->getShape().back() != getViaLaneOrLane()->getShape().front()) {
            PositionVector from = myLaneBefore->getShape();
            const PositionVector& to = getViaLaneOrLane()->getShape();
            const double dist = from.back().distanceTo2D(to.front());
            // figure out direction of shift
            try {
                from.move2side(dist);
            } catch (InvalidArgument&) {
            }
            myLateralShift = (from.back().distanceTo2D(to.front()) < dist) ? dist : -dist;
            if (MSGlobals::gLefthand) {
                myLateralShift *= -1;
            }
            //std::cout << " lateral shift link=" << myLaneBefore->getID() << "->" << getViaLaneOrLane()->getID() << " dist=" << dist << " shift=" << myLateralShift << "\n";
        }
    }
}


MSLink::~MSLink() {
    delete myOffFoeLinks;
    delete myApproachingPersons;
}


void
MSLink::addCustomConflict(const MSLane* from, const MSLane* to, double startPos, double endPos) {
    myCustomConflicts.push_back(CustomConflict(from, to, startPos, endPos));
}

const MSLink::CustomConflict*
MSLink::getCustomConflict(const MSLane* foeLane) const {
    if (myCustomConflicts.size() > 0) {
        const MSLane* foeFrom = foeLane->getNormalPredecessorLane();
        const MSLane* foeTo = foeLane->getNormalSuccessorLane();
        for (const CustomConflict& cc : myCustomConflicts) {
            if (cc.from == foeFrom && cc.to == foeTo) {
                return &cc;
            }
        }

    }
    return nullptr;
}

void
MSLink::setRequestInformation(int index, bool hasFoes, bool isCont,
                              const std::vector<MSLink*>& foeLinks,
                              const std::vector<MSLane*>& foeLanes,
                              MSLane* internalLaneBefore) {
//#ifdef MSLink_DEBUG_CROSSING_POINTS
//    std::cout << " setRequestInformation() for junction " << getViaLaneOrLane()->getEdge().getFromJunction()->getID()
//            << "\nInternalLanes = " << toString(getViaLaneOrLane()->getEdge().getFromJunction()->getInternalLanes())
//            << std::endl;
//#endif
    myIndex = index;
    myHasFoes = hasFoes;
    myAmCont = isCont;
    myFoeLinks = foeLinks;
    for (MSLane* foeLane : foeLanes) {
        // cannot assign vector due to const-ness
        myFoeLanes.push_back(foeLane);
    }
    myJunction = const_cast<MSJunction*>(myLane->getEdge().getFromJunction()); // junctionGraph is initialized after the whole network is loaded
    myAmContOff = isCont && myLogic != nullptr && internalLaneBefore == nullptr && checkContOff();
    myInternalLaneBefore = internalLaneBefore;
    MSLane* lane = nullptr;
    if (internalLaneBefore != nullptr) {
        // this is an exit link. compute crossing points with all foeLanes
        lane = internalLaneBefore;
        //} else if (myLane->getEdge().isCrossing()) {
        //    // this is the link to a pedestrian crossing. compute crossing points with all foeLanes
        //    // @note not currently used by pedestrians
        //    lane = myLane;
    }
    const MSLink* entryLink = getCorrespondingEntryLink();
    if (entryLink->getOffState() == LinkState::LINKSTATE_ALLWAY_STOP && entryLink->getTLLogic() != nullptr) {
        // TLS has "normal" right of way rules but all conflicting links are foes when switching TLS off
        // (unless it's an internal junction link which should ignore all foes and should be ignored by all foes
        myOffFoeLinks = new std::vector<MSLink*>();
        if (isEntryLink()) {
            for (MSLane* foeLane : foeLanes) {
                assert(foeLane->isInternal() || foeLane->isCrossing());
                MSLink* viaLink = foeLane->getIncomingLanes().front().viaLink;
                if (viaLink->getLaneBefore()->isNormal()) {
                    myOffFoeLinks->push_back(viaLink);
                }
            }
        }
    }
#ifdef MSLink_DEBUG_CROSSING_POINTS
    std::cout << "link " << myIndex << " to " << getViaLaneOrLane()->getID() << " internalLaneBefore=" << (lane == 0 ? "NULL" : lane->getID()) << " has foes: " << toString(foeLanes) << "\n";
#endif
    if (lane != nullptr) {
        const bool beforeInternalJunction = lane->getLinkCont()[0]->getViaLaneOrLane()->getEdge().isInternal();
        if (lane->getIncomingLanes().size() != 1) {
            throw ProcessError("Internal lane '" + lane->getID() + "' has " + toString(lane->getIncomingLanes().size()) + " predecessors");
        }
        const MSLink* junctionEntryLink = lane->getEntryLink();
        const bool isSecondPart = isExitLinkAfterInternalJunction();
        // compute crossing points
        for (const MSLane* foeLane : myFoeLanes) {
            const CustomConflict* cc = junctionEntryLink != nullptr ? junctionEntryLink->getCustomConflict(foeLane) : nullptr;
            if (cc != nullptr) {
                // handle custom conflict definition
                double startPos = cc->startPos;
                const double conflictSize = cc->endPos - cc->startPos;
                if (isSecondPart) {
                    startPos -= junctionEntryLink->getViaLane()->getLength();
                }
                // the foe connection may be split at an internal
                // junction, we need to figure out whether the current
                // foeLane is the intended target for the custom conflict
                // There are two possibilities:
                // a) We have no custom conflict for the reverse pair of connections
                //    -> just check whether lane and foeLane intersect
                // b) We have a "reverse" custom conflict
                //    -> check whether it covers the foeLane
                const CustomConflict* rcc = foeLane->getEntryLink()->getCustomConflict(lane);
                bool haveIntersection = false;
                if (rcc == nullptr) {
                    // a)
                    haveIntersection = lane->getShape().intersectsAtLengths2D(foeLane->getShape()).size() > 0;
                } else {
                    // b)
                    const bool foeIsSecondPart = foeLane->getLogicalPredecessorLane()->isInternal();
                    double foeStartPos = rcc->startPos;
                    const double foeConflictSize = rcc->endPos - rcc->startPos;
                    if (foeIsSecondPart) {
                        foeStartPos -= foeLane->getLogicalPredecessorLane()->getLength();
                    }
                    const double foeEndPos = foeStartPos + foeConflictSize;
                    haveIntersection = ((foeStartPos > 0 && foeStartPos < foeLane->getLength())
                                        || (foeEndPos > 0 && foeEndPos < foeLane->getLength()));
                }
                if (haveIntersection) {
                    myConflicts.push_back(ConflictInfo(lane->getLength() - startPos, conflictSize));
                } else {
                    myConflicts.push_back(ConflictInfo(-NO_INTERSECTION, 0));
                }
#ifdef MSLink_DEBUG_CROSSING_POINTS
                std::cout << " " << lane->getID() << " custom conflict with " << foeLane->getID() << " customReverse=" << (rcc != nullptr)
                          << " haveIntersection=" << haveIntersection
                          << " startPos=" << startPos << " conflictSize=" << conflictSize
                          << " lbc=" << myConflicts.back().lengthBehindCrossing
                          << "\n";
#endif
                continue;
            }
            myHavePedestrianCrossingFoe = myHavePedestrianCrossingFoe || foeLane->getEdge().isCrossing();
            const bool sameTarget = myLane == foeLane->getLinkCont()[0]->getLane();
            if (sameTarget && !beforeInternalJunction && !contIntersect(lane, foeLane)) {
                //if (myLane == foeLane->getLinkCont()[0]->getLane()) {
                // this foeLane has the same target and merges at the end (lane exits the junction)
                const double minDist = MIN2(DIVERGENCE_MIN_WIDTH, 0.5 * (lane->getWidth() + foeLane->getWidth()));
                if (lane->getShape().back().distanceTo2D(foeLane->getShape().back()) >= minDist) {
                    // account for lateral shift by the entry links
                    if (foeLane->getEntryLink()->isIndirect()) {
                        myConflicts.push_back(ConflictInfo(-NO_INTERSECTION, 0)); // dummy value, never used
#ifdef MSLink_DEBUG_CROSSING_POINTS
                        std::cout << " " << lane->getID() << " dummy merge with indirect" << foeLane->getID() << "\n";
#endif
                    } else {
                        myConflicts.push_back(ConflictInfo(0, foeLane->getWidth(), CONFLICT_DUMMY_MERGE)); // dummy value, never used
#ifdef MSLink_DEBUG_CROSSING_POINTS
                        std::cout << " " << lane->getID() << " dummy merge with " << foeLane->getID() << "\n";
#endif
                    }
                } else {
                    const double distAfterDivergence = computeDistToDivergence(lane, foeLane, minDist, false);
                    const double lbcLane = lane->interpolateGeometryPosToLanePos(distAfterDivergence);
                    myConflicts.push_back(ConflictInfo(lbcLane, foeLane->getWidth()));
#ifdef MSLink_DEBUG_CROSSING_POINTS
                    std::cout
                            << " " << lane->getID()
                            << " merges with " << foeLane->getID()
                            << " nextLane " << lane->getLinkCont()[0]->getViaLaneOrLane()->getID()
                            << " dist1=" << myConflicts.back().lengthBehindCrossing
                            << "\n";
#endif
                }
            } else {
                std::vector<double> intersections1 = lane->getShape().intersectsAtLengths2D(foeLane->getShape());
#ifdef MSLink_DEBUG_CROSSING_POINTS_DETAILS
                std::cout << "    intersections1=" << toString(intersections1) << "\n";
#endif
                bool haveIntersection = true;
                if (intersections1.size() == 0) {
                    intersections1.push_back(-NO_INTERSECTION); // disregard this foe (using maxdouble leads to nasty problems down the line)
                    haveIntersection = false;
                } else if (intersections1.size() > 1) {
                    std::sort(intersections1.begin(), intersections1.end());
                }
                std::vector<double> intersections2 = foeLane->getShape().intersectsAtLengths2D(lane->getShape());
#ifdef MSLink_DEBUG_CROSSING_POINTS_DETAILS
                std::cout << "    intersections2=" << toString(intersections2) << "\n";
#endif
                if (intersections2.size() == 0) {
                    intersections2.push_back(0);
                } else if (intersections2.size() > 1) {
                    std::sort(intersections2.begin(), intersections2.end());
                }
                double conflictSize = foeLane->getWidth();
                ConflictFlag flag = CONFLICT_NO_INTERSECTION;
                if (haveIntersection) {
                    flag = CONFLICT_DEFAULT;
                    const double angle1 = GeomHelper::naviDegree(lane->getShape().rotationAtOffset(intersections1.back()));
                    const double angle2 = GeomHelper::naviDegree(foeLane->getShape().rotationAtOffset(intersections2.back()));
                    const double angleDiff = GeomHelper::getMinAngleDiff(angle1, angle2);
                    //const double angleDiff = MIN2(GeomHelper::getMinAngleDiff(angle1, angle2),
                    //                              GeomHelper::getMinAngleDiff(angle1, angle2 + 180));
                    const double widthFactor = 1 / MAX2(sin(DEG2RAD(angleDiff)), 0.2) * 2 - 1;
                    //std::cout << "  intersection of " << lane->getID() << " with " << foeLane->getID() << " angle1=" << angle1 << " angle2=" << angle2 << " angleDiff=" << angleDiff << " widthFactor=" << widthFactor << "\n";
                    conflictSize *= widthFactor;
                    conflictSize = MIN2(conflictSize, lane->getLength());
                    // lane width affects the crossing point
                    intersections1.back() -= conflictSize / 2;
                    // ensure non-negative offset for weird geometries
                    intersections1.back() = MAX2(0.0, intersections1.back());

                    // also length/geometry factor. (XXX: Why subtract width/2 *before* converting geometric position to lane pos? refs #3031)
                    intersections1.back() = lane->interpolateGeometryPosToLanePos(intersections1.back());

                    if (internalLaneBefore->getLogicalPredecessorLane()->getEdge().isInternal() && !foeLane->getEdge().isCrossing())  {
                        flag = CONFLICT_STOP_AT_INTERNAL_JUNCTION;
                    }

                    if (foeLane->getEdge().isCrossing()) {
                        const_cast<MSLink*>(getCorrespondingEntryLink())->updateDistToFoePedCrossing(intersections1.back());
                    };
                }

                myConflicts.push_back(ConflictInfo(
                                          lane->getLength() - intersections1.back(),
                                          conflictSize, flag));

#ifdef MSLink_DEBUG_CROSSING_POINTS
                std::cout
                        << "  intersection of " << lane->getID()
                        << " totalLength=" << lane->getLength()
                        << " with " << foeLane->getID()
                        << " totalLength=" << foeLane->getLength()
                        << " dist1=" << myConflicts.back().lengthBehindCrossing
                        << " widthFactor=" << myConflicts.back().conflictSize / foeLane->getWidth()
                        << "\n";
#endif
            }
        }
        // check for overlap with internal lanes from the same source lane
        const MSLane* pred = lane->getLogicalPredecessorLane();
        // to avoid overlap with vehicles that came from pred (especially when pred has endOffset > 0)
        // we add all other internal lanes from pred as foeLanes
        for (const MSLink* const link : pred->getLinkCont()) {
            const MSLane* const sibling = link->getViaLane();
            if (sibling != lane && sibling != nullptr) {
                const double minDist = MIN2(DIVERGENCE_MIN_WIDTH, 0.5 * (lane->getWidth() + sibling->getWidth()));
                if (lane->getShape().front().distanceTo2D(sibling->getShape().front()) >= minDist) {
                    // account for lateral shift by the entry links
                    continue;
                }
                const double distToDivergence = computeDistToDivergence(lane, sibling, minDist, true);
                double lbcLane;
                if (lane->getLength() == sibling->getLength() && &lane->getEdge() == &sibling->getEdge()) {
                    // for parallel lanes, avoid inconsistency in distance estimation (#10988)
                    // between forward distance (getLeaderInfo)
                    // and backward distance used in lane-changing (getFollowersOnConsecutive)
                    lbcLane = lane->getLength() - distToDivergence;
                } else {
                    lbcLane = MAX2(0.0, lane->getLength() - lane->interpolateGeometryPosToLanePos(distToDivergence));
                }
                ConflictInfo ci = ConflictInfo(lbcLane, sibling->getWidth());
                auto it = std::find(myFoeLanes.begin(), myFoeLanes.end(), sibling);
                if (it != myFoeLanes.end()) {
                    // avoid duplicate foeLane
                    const int replacedIndex = (int)(it - myFoeLanes.begin());
                    myConflicts[replacedIndex] = ci;
                } else {
                    myConflicts.push_back(ci);
                    myFoeLanes.push_back(sibling);
                }
#ifdef MSLink_DEBUG_CROSSING_POINTS
                std::cout << " adding same-origin foe" << sibling->getID()
                          << " dist1=" << myConflicts.back().lengthBehindCrossing
                          << "\n";
#endif
            }
        }
        // init points for the symmetrical conflict
        // for each pair of conflicting lanes, the link that gets second, sets the pointers
        for (int i = 0; i < (int)myFoeLanes.size(); i++) {
            const MSLane* foeLane = myFoeLanes[i];
            MSLink* foeExitLink = foeLane->getLinkCont()[0];
            int foundIndex = -1;
            for (int i2 = 0; i2 < (int)foeExitLink->myFoeLanes.size(); i2++) {
                if (foeExitLink->myFoeLanes[i2] == lane) {
                    myConflicts[i].foeConflictIndex = i2;
                    foeExitLink->myConflicts[i2].foeConflictIndex = i;
                    myRecheck.erase({foeExitLink, this});
                    foundIndex = i2;
                    break;
                }
            }
#ifdef MSLink_DEBUG_CROSSING_POINTS
            std::cout << lane->getID() << " foeLane=" << foeLane->getID() << " index=" << i << " foundIndex=" << foundIndex << "\n";
#endif
            if (foundIndex < 0) {
                if (myConflicts[i].flag != CONFLICT_NO_INTERSECTION) {
                    myRecheck.insert({this, foeExitLink});
                }
            }
        }
    }
    if (MSGlobals::gLateralResolution > 0) {
        // check for links with the same origin lane and the same destination edge
        const MSEdge* myTarget = &myLane->getEdge();
        // save foes for entry links
        for (MSLink* const it : myLaneBefore->getLinkCont()) {
            const MSEdge* target = &(it->getLane()->getEdge());
            if (it == this) {
                continue;
            }
            if (target == myTarget) {
                mySublaneFoeLinks.push_back(it);
#ifdef MSLink_DEBUG_CROSSING_POINTS
                std::cout << "  sublaneFoeLink (same target): " << it->getViaLaneOrLane()->getID() << "\n";
#endif
            } else if (myDirection != LinkDirection::STRAIGHT && it->getDirection() == LinkDirection::STRAIGHT) {
                // potential turn conflict
                mySublaneFoeLinks2.push_back(it);
#ifdef MSLink_DEBUG_CROSSING_POINTS
                std::cout << "  sublaneFoeLink2 (other target: " << it->getViaLaneOrLane()->getID() << "\n";
#endif
            }
        }
        // save foes for exit links
        if (fromInternalLane()) {
            //std::cout << " setRequestInformation link=" << getViaLaneOrLane()->getID() << " before=" << myLaneBefore->getID() << " before2=" << myLaneBefore->getIncomingLanes().front().lane->getID() << "\n";
            for (const MSLink* const link : myLaneBefore->getIncomingLanes().front().lane->getLinkCont()) {
                if (link->getViaLane() != myInternalLaneBefore && &link->getLane()->getEdge() == myTarget) {
                    //std::cout << " add sublaneFoe=" << (*it)->getViaLane()->getID() << "\n";
                    mySublaneFoeLanes.push_back(link->getViaLane());
                }
            }
        }
    }
    if (myInternalLaneBefore != nullptr
            && myDirection != LinkDirection::STRAIGHT
            // for right turns, the curvature helps rather than restricts the linkLeader check
            && (
                (!MSGlobals::gLefthand && myDirection != LinkDirection::RIGHT)
                || (MSGlobals::gLefthand && myDirection != LinkDirection::LEFT))) {
        const double angle = fabs(GeomHelper::angleDiff(
                                      myLaneBefore->getNormalPredecessorLane()->getShape().angleAt2D(-2),
                                      myLane->getShape().angleAt2D(0)));
        if (angle > 0) {
            double length = myInternalLaneBefore->getShape().length2D();
            if (myInternalLaneBefore->getIncomingLanes().size() == 1 &&
                    myInternalLaneBefore->getIncomingLanes()[0].lane->isInternal()) {
                length += myInternalLaneBefore->getIncomingLanes()[0].lane->getShape().length2D();
            } else if (myInternalLane != nullptr) {
                length += myInternalLane->getShape().length2D();
            }
            myRadius = length / angle;
            //std::cout << getDescription() << " a=" << RAD2DEG(angle) << " l=" << length << " r=" << myRadius << "\n";
        }
    }
}


void
MSLink::recheckSetRequestInformation() {
    for (auto item : myRecheck) {
#ifdef MSLink_DEBUG_CROSSING_POINTS
        std::cout << " recheck l1=" << item.first->getDescription() << " l2=" << item.second->getDescription() << "\n";
#endif
        MSLink* link = item.first;
        MSLink* foeExitLink = item.second;
        const MSLane* lane = link->getInternalLaneBefore();
        const MSLane* foeLane = foeExitLink->getInternalLaneBefore();
        int conflictIndex = -1;
        for (int i = 0; i < (int)link->myFoeLanes.size(); i++) {
            if (link->myFoeLanes[i] == foeLane) {
                conflictIndex = i;
                break;
            }
        }
        if (conflictIndex == -1) {
            WRITE_WARNING("Could not recheck ConflictInfo for " + link->getDescription() + " and " + foeExitLink->getDescription() + "\n");
            continue;
        }
        ConflictInfo& ci = link->myConflicts[conflictIndex];
        std::vector<double> intersections1 = foeLane->getShape().intersectsAtLengths2D(lane->getShape());
        if (intersections1.size() == 0) {
#ifdef MSLink_DEBUG_CROSSING_POINTS
            std::cout << "    no intersection\n";
#endif
            continue;
        }
        const double widthFactor = ci.conflictSize / foeLane->getWidth();
        const double conflictSize2 = lane->getWidth() * widthFactor;
        std::sort(intersections1.begin(), intersections1.end());
        intersections1.back() -= conflictSize2 / 2;
        intersections1.back() = MAX2(0.0, intersections1.back());
        ci.foeConflictIndex = (int)foeExitLink->myConflicts.size();
        foeExitLink->myConflicts.push_back(ConflictInfo(foeLane->getLength() - intersections1.back(), conflictSize2));
#ifdef MSLink_DEBUG_CROSSING_POINTS
        std::cout << "    ci=" << conflictIndex << " wf=" << widthFactor << " flag=" << ci.flag << " flbc=" << foeExitLink->myConflicts.back().lengthBehindCrossing << "\n";
#endif
    }
    myRecheck.clear();
}

double
MSLink::computeDistToDivergence(const MSLane* lane, const MSLane* sibling, double minDist, bool sameSource) const {
    double lbcSibling = 0;
    double lbcLane = 0;

    PositionVector l = lane->getShape();
    PositionVector s = sibling->getShape();
    double length = l.length2D();
    double sibLength = s.length2D();
    if (!sameSource) {
        l = l.reverse();
        s = s.reverse();
    } else if (sibling->getEntryLink()->myAmIndirect) {
        // ignore final waiting position since it may be quite close to the lane
        // shape but the waiting position is perpendicular (so the minDist
        // requirement is not necessary
        lbcSibling += s[-1].distanceTo2D(s[-2]);
        s.pop_back();
    } else if (lane->getEntryLink()->myAmIndirect) {
        // ignore final waiting position since it may be quite close to the lane
        // shape but the waiting position is perpendicular (so the minDist
        // requirement is not necessary
        lbcLane += l[-1].distanceTo2D(l[-2]);
        l.pop_back();
    }

#ifdef MSLink_DEBUG_CROSSING_POINTS_DETAILS
    std::cout << "   sameSource=" << sameSource << " minDist=" << minDist << " backDist=" << l.back().distanceTo2D(s.back()) << "\n";
#endif
    if (l.back().distanceTo2D(s.back()) > minDist) {
        // compute the final divergence point
        // this position serves two purposes:
        // 1) once the foe vehicle back (on sibling) has passed this point, we can safely ignore it
        // 2) both vehicles are put into a cf-relationship while before the point.
        //    Since the actual crossing point is at the start of the junction,
        //    we want to make sure that both vehicles have the same distance to the crossing point and thus follow each other naturally
        std::vector<double> distances = l.distances(s);
#ifdef MSLink_DEBUG_CROSSING_POINTS
        std::cout << "   distances=" << toString(distances) << "\n";
#endif
        assert(distances.size() == l.size() + s.size());
        if (distances.back() > minDist && distances[l.size() - 1] > minDist) {
            // do a pairwise check between lane and sibling to make because we do not know which of them bends more
            for (int j = (int)s.size() - 2; j >= 0; j--) {
                const int i = j + (int)l.size();
                const double segLength = s[j].distanceTo2D(s[j + 1]);
                if (distances[i] > minDist) {
                    lbcSibling += segLength;
                } else {
                    // assume no sharp bends and just interpolate the last segment
                    lbcSibling += segLength - (minDist - distances[i]) * segLength / (distances[i + 1] - distances[i]);
                    break;
                }
            }
            for (int i = (int)l.size() - 2; i >= 0; i--) {
                const double segLength = l[i].distanceTo2D(l[i + 1]);
                if (distances[i] > minDist) {
                    lbcLane += segLength;
                } else {
                    // assume no sharp bends and just interpolate the last segment
                    lbcLane += segLength - (minDist - distances[i]) * segLength / (distances[i + 1] - distances[i]);
                    break;
                }
            }
        }
        assert(lbcSibling >= -NUMERICAL_EPS);
        assert(lbcLane >= -NUMERICAL_EPS);
    }
    const double distToDivergence1 = sibling->getLength() - lbcSibling;
    const double distToDivergence2 = lane->getLength() - lbcLane;
    const double distToDivergence = MIN3(
                                        MAX2(distToDivergence1, distToDivergence2),
                                        sibLength, length);
#ifdef MSLink_DEBUG_CROSSING_POINTS
    std::cout << "   distToDivergence=" << distToDivergence
              << " distTD1=" << distToDivergence1
              << " distTD2=" << distToDivergence2
              << " length=" << length
              << " sibLength=" << sibLength
              << "\n";
#endif
    return distToDivergence;
}


bool
MSLink::contIntersect(const MSLane* lane, const MSLane* foe) {
    if (foe->getLinkCont()[0]->getViaLane() != nullptr) {
        std::vector<double> intersections = lane->getShape().intersectsAtLengths2D(foe->getShape());
        return intersections.size() > 0;
    }
    return false;
}


void
MSLink::setApproaching(const SUMOVehicle* approaching, const SUMOTime arrivalTime, const double arrivalSpeed, const double leaveSpeed,
                       const bool setRequest, const double arrivalSpeedBraking, const SUMOTime waitingTime, double dist, double latOffset) {
    const SUMOTime leaveTime = getLeaveTime(arrivalTime, arrivalSpeed, leaveSpeed, approaching->getVehicleType().getLength());
#ifdef DEBUG_APPROACHING
    if (DEBUG_COND2(approaching)) {
        std::cout << SIMTIME << " Link '" << (myLaneBefore == 0 ? "NULL" : myLaneBefore->getID()) << "'->'" << (myLane == 0 ? "NULL" : myLane->getID()) << "' Adding approaching vehicle '" << approaching->getID() << "'\nCurrently registered vehicles:" << std::endl;
        for (auto i = myApproachingVehicles.begin(); i != myApproachingVehicles.end(); ++i) {
            std::cout << "'" << i->first->getID() << "'" << std::endl;
        }
    }
#endif
    myApproachingVehicles.emplace(approaching,
                                  ApproachingVehicleInformation(arrivalTime, leaveTime, arrivalSpeed, leaveSpeed, setRequest,
                                          arrivalSpeedBraking, waitingTime, dist, approaching->getSpeed(), latOffset));
}


void
MSLink::setApproaching(const SUMOVehicle* approaching, ApproachingVehicleInformation ai) {

#ifdef DEBUG_APPROACHING
    if (DEBUG_COND2(approaching)) {
        std::cout << SIMTIME << " Link '" << (myLaneBefore == 0 ? "NULL" : myLaneBefore->getID()) << "'->'" << (myLane == 0 ? "NULL" : myLane->getID()) << "' Adding approaching vehicle '" << approaching->getID() << "'\nCurrently registered vehicles:" << std::endl;
        for (auto i = myApproachingVehicles.begin(); i != myApproachingVehicles.end(); ++i) {
            std::cout << "'" << i->first->getID() << "'" << std::endl;
        }
    }
#endif
    myApproachingVehicles.emplace(approaching, ai);
}

void
MSLink::setApproachingPerson(const MSPerson* approaching, const SUMOTime arrivalTime, const SUMOTime leaveTime) {
    if (myApproachingPersons == nullptr) {
        myApproachingPersons = new PersonApproachInfos();
    }
    myApproachingPersons->emplace(approaching, ApproachingPersonInformation(arrivalTime, leaveTime));
}

void
MSLink::removeApproaching(const SUMOVehicle* veh) {

#ifdef DEBUG_APPROACHING
    if (DEBUG_COND2(veh)) {
        std::cout << SIMTIME << " Link '" << (myLaneBefore == 0 ? "NULL" : myLaneBefore->getID()) << "'->'" << (myLane == 0 ? "NULL" : myLane->getID()) << std::endl;
        std::cout << "' Removing approaching vehicle '" << veh->getID() << "'\nCurrently registered vehicles:" << std::endl;
        for (auto i = myApproachingVehicles.begin(); i != myApproachingVehicles.end(); ++i) {
            std::cout << "'" << i->first->getID() << "'" << std::endl;
        }
    }
#endif
    myApproachingVehicles.erase(veh);
}


void
MSLink::removeApproachingPerson(const MSPerson* person) {
    if (myApproachingPersons == nullptr) {
        WRITE_WARNINGF("Person '%' entered crossing lane '%' without registering approach, time=%", person->getID(), myLane->getID(), time2string(SIMSTEP));
        return;
    }
#ifdef DEBUG_APPROACHING
    if (DEBUG_COND2(person)) {
        std::cout << SIMTIME << " Link '" << (myLaneBefore == 0 ? "NULL" : myLaneBefore->getID()) << "'->'" << (myLane == 0 ? "NULL" : myLane->getID()) << std::endl;
        std::cout << "' Removing approaching person '" << person->getID() << "'\nCurrently registered persons:" << std::endl;
        for (auto i = myApproachingPersons->begin(); i != myApproachingPersons->end(); ++i) {
            std::cout << "'" << i->first->getID() << "'" << std::endl;
        }
    }
#endif
    myApproachingPersons->erase(person);
}


MSLink::ApproachingVehicleInformation
MSLink::getApproaching(const SUMOVehicle* veh) const {
    auto i = myApproachingVehicles.find(veh);
    if (i != myApproachingVehicles.end()) {
        return i->second;
    } else {
        return ApproachingVehicleInformation(INVALID_TIME, INVALID_TIME, 0, 0, false, 0, 0, 0, 0, 0);
    }
}


void
MSLink::clearState() {
    myApproachingVehicles.clear();
}


SUMOTime
MSLink::getLeaveTime(const SUMOTime arrivalTime, const double arrivalSpeed,
                     const double leaveSpeed, const double vehicleLength) const {
    return arrivalTime == SUMOTime_MAX ? SUMOTime_MAX : arrivalTime + TIME2STEPS((getLength() + vehicleLength) / MAX2(0.5 * (arrivalSpeed + leaveSpeed), NUMERICAL_EPS));
}


bool
MSLink::opened(SUMOTime arrivalTime, double arrivalSpeed, double leaveSpeed, double vehicleLength,
               double impatience, double decel, SUMOTime waitingTime, double posLat,
               BlockingFoes* collectFoes, bool ignoreRed, const SUMOTrafficObject* ego, double dist) const {
#ifdef MSLink_DEBUG_OPENED
    if (gDebugFlag1) {
        std::cout << SIMTIME << "  opened? link=" << getDescription() << " red=" << haveRed() << " cont=" << isCont() << " numFoeLinks=" << myFoeLinks.size() << " havePrio=" << havePriority() << " lastWasContMajorGreen=" << lastWasContState(LINKSTATE_TL_GREEN_MAJOR) << "\n";
    }
#endif
    if (haveRed() && !ignoreRed) {
        return false;
    }
    if (isCont() && MSGlobals::gUsingInternalLanes) {
        return true;
    }
    const SUMOTime leaveTime = getLeaveTime(arrivalTime, arrivalSpeed, leaveSpeed, vehicleLength);
    if (MSGlobals::gLateralResolution > 0) {
        // check for foes on the same lane with the same target edge
        for (const MSLink* foeLink : mySublaneFoeLinks) {
            assert(myLane != foeLink->getLane());
            for (const auto& it : foeLink->myApproachingVehicles) {
                const SUMOVehicle* foe = it.first;
                if (
                    // there only is a conflict if the paths cross
                    ((posLat < foe->getLateralPositionOnLane() + it.second.latOffset && myLane->getIndex() > foeLink->myLane->getIndex())
                     || (posLat > foe->getLateralPositionOnLane() + it.second.latOffset && myLane->getIndex() < foeLink->myLane->getIndex()))
                    // the vehicle that arrives later must yield
                    && (arrivalTime > it.second.arrivalTime
                        // if both vehicles arrive at the same time, the one
                        // to the left must yield
                        || (arrivalTime == it.second.arrivalTime && posLat > foe->getLateralPositionOnLane()))) {
                    if (blockedByFoe(foe, it.second, arrivalTime, leaveTime, arrivalSpeed, leaveSpeed, false,
                                     impatience, decel, waitingTime, ego)) {
#ifdef MSLink_DEBUG_OPENED
                        if (gDebugFlag1) {
                            std::cout << SIMTIME << " blocked by " << foe->getID() << " arrival=" << arrivalTime << " foeArrival=" << it.second.arrivalTime << "\n";
                        }
#endif
                        if (collectFoes == nullptr) {
#ifdef MSLink_DEBUG_OPENED
                            if (gDebugFlag1) {
                                std::cout << " link=" << getViaLaneOrLane()->getID() << " blocked by sublaneFoe=" << foe->getID() << " foeLink=" << foeLink->getViaLaneOrLane()->getID() << " posLat=" << posLat << "\n";
                            }
#endif
                            return false;
                        } else {
                            collectFoes->push_back(it.first);
                        }
                    }
                }
            }
        }
        // check for foes on the same lane with a different target edge
        // (straight movers take precedence if the paths cross)
        const int lhSign = MSGlobals::gLefthand ? -1 : 1;
        for (const MSLink* foeLink : mySublaneFoeLinks2) {
            assert(myDirection != LinkDirection::STRAIGHT);
            for (const auto& it : foeLink->myApproachingVehicles) {
                const SUMOVehicle* foe = it.first;
                // there only is a conflict if the paths cross
                // and if the vehicles are not currently in a car-following relationship
                const double egoWidth = ego == nullptr ? 1.8 : ego->getVehicleType().getWidth();
                if (!lateralOverlap(posLat, egoWidth, foe->getLateralPositionOnLane() + it.second.latOffset, foe->getVehicleType().getWidth())
                        && (((myDirection == LinkDirection::RIGHT || myDirection == LinkDirection::PARTRIGHT)
                             && (posLat * lhSign > (foe->getLateralPositionOnLane() + it.second.latOffset) * lhSign))
                            || ((myDirection == LinkDirection::LEFT || myDirection == LinkDirection::PARTLEFT)
                                && (posLat * lhSign < (foe->getLateralPositionOnLane() + it.second.latOffset) * lhSign)))) {
                    if (blockedByFoe(foe, it.second, arrivalTime, leaveTime, arrivalSpeed, leaveSpeed, false,
                                     impatience, decel, waitingTime, ego)) {
#ifdef MSLink_DEBUG_OPENED
                        if (gDebugFlag1) {
                            std::cout << SIMTIME << " blocked by sublane foe " << foe->getID() << " arrival=" << arrivalTime << " foeArrival=" << it.second.arrivalTime << "\n";
                        }
#endif
                        if (collectFoes == nullptr) {
#ifdef MSLink_DEBUG_OPENED
                            if (gDebugFlag1) {
                                std::cout << " link=" << getViaLaneOrLane()->getID() << " blocked by sublaneFoe2=" << foe->getID() << " foeLink=" << foeLink->getViaLaneOrLane()->getID() << " posLat=" << posLat << "\n";
                            }
#endif
                            return false;
                        } else {
                            collectFoes->push_back(it.first);
                        }
                    }
                }
            }
        }
    }
    if ((havePriority() || lastWasContState(LINKSTATE_TL_GREEN_MAJOR)) && myState != LINKSTATE_ZIPPER) {
        // priority usually means the link is open but there are exceptions:
        // zipper still needs to collect foes
        // sublane model could have detected a conflict
        return collectFoes == nullptr || collectFoes->size() == 0;
    }
    if (myState == LINKSTATE_ALLWAY_STOP && waitingTime < TIME2STEPS(ego == nullptr ? TS : ego->getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_ALLWAYSTOP_WAIT, TS))) {
        return false;
    } else if (myState == LINKSTATE_STOP && waitingTime < TIME2STEPS(ego == nullptr ? TS : ego->getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_STOPSIGN_WAIT, TS))) {
        return false;
    }

    const std::vector<MSLink*>& foeLinks = (myOffFoeLinks == nullptr || getCorrespondingEntryLink()->getState() != LINKSTATE_ALLWAY_STOP) ? myFoeLinks : *myOffFoeLinks;
#ifdef MSLink_DEBUG_OPENED
    if (gDebugFlag1) {
        std::cout << SIMTIME << " opened link=" << getViaLaneOrLane()->getID() << " foeLinks=" << foeLinks.size() << "\n";
    }
#endif

    if (MSGlobals::gUseMesoSim && impatience == 1) {
        return true;
    }
    const bool lastWasContRed = lastWasContState(LINKSTATE_TL_RED);
    for (const MSLink* const link : foeLinks) {
        if (MSGlobals::gUseMesoSim) {
            if (link->haveRed()) {
                continue;
            }
        }
#ifdef MSLink_DEBUG_OPENED
        if (gDebugFlag1) {
            std::cout << SIMTIME << "   foeLink=" << link->getViaLaneOrLane()->getID() << " numApproaching=" << link->getApproaching().size() << "\n";
            if (link->getLane()->isCrossing()) {
                std::cout << SIMTIME << "     approachingPersons=" << (link->myApproachingPersons == nullptr ? "NULL" : toString(link->myApproachingPersons->size())) << "\n";
            }
        }
#endif
        if (link->blockedAtTime(arrivalTime, leaveTime, arrivalSpeed, leaveSpeed, myLane == link->getLane(),
                                impatience, decel, waitingTime, collectFoes, ego, lastWasContRed, dist)) {
            return false;
        }
    }
    if (collectFoes != nullptr && collectFoes->size() > 0) {
        return false;
    }
    return true;
}


bool
MSLink::blockedAtTime(SUMOTime arrivalTime, SUMOTime leaveTime, double arrivalSpeed, double leaveSpeed,
                      bool sameTargetLane, double impatience, double decel, SUMOTime waitingTime,
                      BlockingFoes* collectFoes, const SUMOTrafficObject* ego, bool lastWasContRed, double dist) const {
    for (const auto& it : myApproachingVehicles) {
#ifdef MSLink_DEBUG_OPENED
        if (gDebugFlag1) {
            if (ego != nullptr
                    && ego->getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_IGNORE_FOE_SPEED, 0) >= it.second.speed
                    && ego->getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_IGNORE_FOE_PROB, 0) > 0) {
                std::stringstream stream; // to reduce output interleaving from different threads
                stream << SIMTIME << " " << myApproachingVehicles.size() << "   foe link=" << getViaLaneOrLane()->getID()
                       << " foeVeh=" << it.first->getID() << " (below ignore speed)"
                       << " ignoreFoeProb=" << ego->getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_IGNORE_FOE_PROB, 0)
                       << "\n";
                std::cout << stream.str();
            }
        }
#endif
        if (it.first != ego
                && (ego == nullptr
                    || ego->getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_IGNORE_FOE_PROB, 0) == 0
                    || ego->getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_IGNORE_FOE_SPEED, 0) < it.second.speed
                    || ego->getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_IGNORE_FOE_PROB, 0) < RandHelper::rand(ego->getRNG()))
                && !ignoreFoe(ego, it.first)
                && (!lastWasContRed || it.first->getSpeed() > SUMO_const_haltingSpeed)
                && blockedByFoe(it.first, it.second, arrivalTime, leaveTime, arrivalSpeed, leaveSpeed, sameTargetLane,
                                impatience, decel, waitingTime, ego)) {
            if (collectFoes == nullptr) {
                return true;
            } else {
                collectFoes->push_back(it.first);
            }
        }
    }
    if (myApproachingPersons != nullptr && !haveRed()) {
        for (const auto& it : *myApproachingPersons) {
            if ((ego == nullptr
                    || ego->getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_IGNORE_FOE_PROB, 0) == 0
                    || ego->getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_IGNORE_FOE_SPEED, 0) < it.first->getSpeed()
                    || ego->getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_IGNORE_FOE_PROB, 0) < RandHelper::rand(ego->getRNG()))
                    && !ignoreFoe(ego, it.first)
                    && !((arrivalTime > it.second.leavingTime) || (leaveTime < it.second.arrivalTime))) {
                // check whether braking is feasible (ego might have started to accelerate already)
                const auto& cfm = ego->getVehicleType().getCarFollowModel();
#ifdef MSLink_DEBUG_OPENED
                if (gDebugFlag1) {
                    std::cout << SIMTIME << ": " << ego->getID() << " conflict with person " << it.first->getID() << " aTime=" << arrivalTime << " foeATime=" << it.second.arrivalTime << " dist=" << dist << " bGap=" << cfm.brakeGap(ego->getSpeed(), cfm.getMaxDecel(), 0) << "\n";
                }
#endif
                if (dist > cfm.brakeGap(ego->getSpeed(), cfm.getMaxDecel(), 0)) {
#ifdef MSLink_DEBUG_OPENED
                    if (gDebugFlag1) {
                        std::cout << SIMTIME << ": " << ego->getID() << " blocked by person " << it.first->getID() << "\n";
                    }
#endif
                    if (collectFoes == nullptr) {
                        return true;
                    } else {
                        collectFoes->push_back(it.first);
                    }
                }
            }
        }
    }
    return false;
}


bool
MSLink::blockedByFoe(const SUMOVehicle* veh, const ApproachingVehicleInformation& avi,
                     SUMOTime arrivalTime, SUMOTime leaveTime, double arrivalSpeed, double leaveSpeed,
                     bool sameTargetLane, double impatience, double decel, SUMOTime waitingTime,
                     const SUMOTrafficObject* ego) const {
#ifdef MSLink_DEBUG_OPENED
    if (gDebugFlag1) {
        std::stringstream stream; // to reduce output interleaving from different threads
        stream << "    link=" << getDescription()
               << " foeVeh=" << veh->getID()
               << " req=" << avi.willPass
               << " aT=" << avi.arrivalTime
               << " lT=" << avi.leavingTime
               << "\n";
        std::cout << stream.str();
    }
#endif
    if (!avi.willPass) {
        return false;
    }
    if (myState == LINKSTATE_ALLWAY_STOP) {
        assert(waitingTime > 0);
#ifdef MSLink_DEBUG_OPENED
        if (gDebugFlag1) {
            std::stringstream stream; // to reduce output interleaving from different threads
            stream << "    foeDist=" << avi.dist
                   << " foeBGap=" << veh->getBrakeGap(false)
                   << " foeWait=" << avi.waitingTime
                   << " wait=" << waitingTime
                   << "\n";
            std::cout << stream.str();
        }
#endif
        // when using actionSteps, the foe waiting time may be outdated
        const SUMOTime actionDelta = SIMSTEP - veh->getLastActionTime();
        if (waitingTime > avi.waitingTime + actionDelta) {
            return false;
        }
        if (waitingTime == (avi.waitingTime + actionDelta) && arrivalTime < avi.arrivalTime + actionDelta) {
            return false;
        }
    }
    SUMOTime foeArrivalTime = avi.arrivalTime;
    double foeArrivalSpeedBraking = avi.arrivalSpeedBraking;
    if (impatience > 0 && arrivalTime < avi.arrivalTime) {
#ifdef MSLink_DEBUG_OPENED
        gDebugFlag6 = ((ego == nullptr || ego->isSelected()) && (veh == nullptr || veh->isSelected()));
#endif
        const SUMOTime fatb = computeFoeArrivalTimeBraking(arrivalTime, veh, avi.arrivalTime, impatience, avi.dist, foeArrivalSpeedBraking);
        foeArrivalTime = (SUMOTime)((1. - impatience) * (double)avi.arrivalTime + impatience * (double)fatb);
#ifdef MSLink_DEBUG_OPENED
        if (gDebugFlag6) {
            std::cout << SIMTIME << " link=" << getDescription() << " ego=" << ego->getID() << " foe=" << veh->getID()
                      << " at=" << STEPS2TIME(arrivalTime)
                      << " fat=" << STEPS2TIME(avi.arrivalTime)
                      << " fatb=" << STEPS2TIME(fatb)
                      << " fat2=" << STEPS2TIME(foeArrivalTime)
                      << "\n";
        }
#endif
    }


    const SUMOTime lookAhead = (myState == LINKSTATE_ZIPPER
                                ? myLookaheadTimeZipper
                                : (ego == nullptr
                                   ? myLookaheadTime
                                   : TIME2STEPS(ego->getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_TIMEGAP_MINOR, STEPS2TIME(myLookaheadTime)))));
    //if (ego != 0) std::cout << SIMTIME << " ego=" << ego->getID() << " jmTimegapMinor=" << ego->getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_TIMEGAP_MINOR, -1) << " lookAhead=" << lookAhead << "\n";
#ifdef MSLink_DEBUG_OPENED
    if (gDebugFlag1 || gDebugFlag6) {
        std::stringstream stream; // to reduce output interleaving from different threads
        stream << "       imp=" << impatience << " fAT2=" << foeArrivalTime << " fASb=" << foeArrivalSpeedBraking << " lA=" << lookAhead << " egoAT=" << arrivalTime << " egoLT=" << leaveTime << " egoLS=" << leaveSpeed << "\n";
        std::cout << stream.str();
    }
#endif
    if (avi.leavingTime < arrivalTime) {
        // ego wants to be follower
        if (sameTargetLane && (arrivalTime - avi.leavingTime < lookAhead
                               || unsafeMergeSpeeds(avi.leaveSpeed, arrivalSpeed,
                                       veh->getVehicleType().getCarFollowModel().getMaxDecel(), decel))) {
#ifdef MSLink_DEBUG_OPENED
            if (gDebugFlag1 || gDebugFlag6) {
                std::cout << "      blocked (cannot follow)\n";
            }
#endif
            return true;
        }
    } else if (foeArrivalTime > leaveTime + lookAhead) {
        // ego wants to be leader.
        if (sameTargetLane && unsafeMergeSpeeds(leaveSpeed, foeArrivalSpeedBraking,
                                                decel, veh->getVehicleType().getCarFollowModel().getMaxDecel())) {
#ifdef MSLink_DEBUG_OPENED
            if (gDebugFlag1 || gDebugFlag6) {
                std::cout << "      blocked (cannot lead)\n";
            }
#endif
            return true;
        }
    } else {
        // even without considering safeHeadwayTime there is already a conflict
#ifdef MSLink_DEBUG_OPENED
        if (gDebugFlag1 || gDebugFlag6) {
            std::cout << "      blocked (hard conflict)\n";
        }
#endif
        return true;
    }
    return false;
}


SUMOTime
MSLink::computeFoeArrivalTimeBraking(SUMOTime arrivalTime, const SUMOVehicle* foe, SUMOTime foeArrivalTime, double impatience, double dist, double& fasb) {
    // a: distance saved when foe brakes from arrivalTime to foeArrivalTime
    // b: distance driven past foeArrivalTime
    // m: permitted decceleration
    // d: total deceleration until foeArrivalTime
    // dist2: distance of foe at arrivalTime
    // actual arrivalTime must fall on a simulation step
    if (arrivalTime - arrivalTime % DELTA_T == foeArrivalTime - foeArrivalTime % DELTA_T) {
        // foe enters the junction in the same step
        return foeArrivalTime;
    }
    if (arrivalTime % DELTA_T > 0) {
        arrivalTime = arrivalTime - (arrivalTime % DELTA_T) + DELTA_T;
    }
    //arrivalTime += DELTA_T - arrivalTime % DELTA_T;
    const double m = foe->getVehicleType().getCarFollowModel().getMaxDecel() * impatience;
    const double dt = STEPS2TIME(foeArrivalTime - arrivalTime);
    const double d = dt * m;
    const double a = dt * d / 2;
    const double v = dist / STEPS2TIME(foeArrivalTime - SIMSTEP + DELTA_T);
    const double dist2 = dist - v * STEPS2TIME(arrivalTime - SIMSTEP);
    if (0.5 * v * v / m <= dist2) {
        if (gDebugFlag6) {
            std::cout << "   dist=" << dist << " dist2=" << dist2 << " at=" << STEPS2TIME(arrivalTime) << " m=" << m << " d=" << d << " a=" << a << " canBrakeToStop\n";
        }
        fasb = 0;
        return foeArrivalTime + TIME2STEPS(30);
    }
    // a = b (foe reaches the original distance to the stop line)
    // x: time driven past foeArrivalTime
    // v: foe speed without braking
    // v2: average foe speed after foeArrivalTime (braking continues for time x)
    // v2 = (v - d - x * m / 2)
    // b = v2 * x
    // solving for x gives:
    const double x = (sqrt(4 * (v - d) * (v - d) - 8 * m * a) * -0.5 - d + v) / m;

#ifdef MSLink_DEBUG_OPENED
    const double x2 = (sqrt(4 * (v - d) * (v - d) - 8 * m * a) * 0.5 - d + v) / m;
    if (gDebugFlag6 || std::isnan(x)) {
        std::cout << SIMTIME << "   dist=" << dist << " dist2=" << dist2  << " at=" << STEPS2TIME(arrivalTime) << " m=" << m << " d=" << d << " v=" << v << " a=" << a << " x=" << x << " x2=" << x2 << "\n";
    }
#endif
    fasb = v - (dt + x) * m;
    return foeArrivalTime + TIME2STEPS(x);
}


bool
MSLink::hasApproachingFoe(SUMOTime arrivalTime, SUMOTime leaveTime, double speed, double decel) const {
    for (const MSLink* const link : myFoeLinks) {
        if (link->blockedAtTime(arrivalTime, leaveTime, speed, speed, myLane == link->getLane(), 0, decel, 0)) {
            return true;
        }
    }
    for (const MSLane* const lane : myFoeLanes) {
        if (lane->getVehicleNumberWithPartials() > 0) {
            return true;
        }
    }
    return false;
}


std::pair<const SUMOVehicle*, const MSLink*>
MSLink::getFirstApproachingFoe(const MSLink* wrapAround) const {
    double closetDist = std::numeric_limits<double>::max();
    const SUMOVehicle* closest = nullptr;
    const MSLink* foeLink = nullptr;
    for (MSLink* link : myFoeLinks) {
        for (const auto& it : link->myApproachingVehicles) {
            //std::cout << " link=" << getDescription() << " foeLink_in=" << link->getLaneBefore()->getID() << " wrapAround=" << wrapAround->getDescription() << "\n";
            if (link->getLaneBefore() == wrapAround->getLaneBefore()) {
                return std::make_pair(nullptr, wrapAround);
            } else if (it.second.dist < closetDist) {
                closetDist = it.second.dist;
                if (it.second.willPass) {
                    closest = it.first;
                    foeLink = link;
                }
            }
        }
    }
    return std::make_pair(closest, foeLink);
}


void
MSLink::setTLState(LinkState state, SUMOTime t) {
    if (myState != state) {
        myLastStateChange = t;
    }
    myState = state;
    if (haveGreen()) {
        myLastGreenState = myState;
    }
}


void
MSLink::setTLLogic(const MSTrafficLightLogic* logic) {
    myLogic = logic;
}


bool
MSLink::isCont() const {
    // when a traffic light is switched off minor roads have their cont status revoked
    return (myState == LINKSTATE_TL_OFF_BLINKING || myState == LINKSTATE_ALLWAY_STOP) ? myAmContOff : myAmCont;
}


bool
MSLink::lastWasContMajor() const {
    if (myInternalLane == nullptr || myAmCont) {
        return false;
    } else {
        MSLane* pred = myInternalLane->getLogicalPredecessorLane();
        if (!pred->getEdge().isInternal()) {
            return false;
        } else {
            const MSLane* const pred2 = pred->getLogicalPredecessorLane();
            assert(pred2 != nullptr);
            const MSLink* const predLink = pred2->getLinkTo(pred);
            assert(predLink != nullptr);
            if (predLink->havePriority()) {
                return true;
            }
            if (myHavePedestrianCrossingFoe) {
                return predLink->getLastGreenState() == LINKSTATE_TL_GREEN_MAJOR;
            } else {
                return predLink->haveYellow();
            }
        }
    }
}


bool
MSLink::lastWasContState(LinkState linkState) const {
    if (myInternalLane == nullptr || myAmCont || myHavePedestrianCrossingFoe) {
        return false;
    } else {
        MSLane* pred = myInternalLane->getLogicalPredecessorLane();
        if (!pred->getEdge().isInternal()) {
            return false;
        } else {
            const MSLane* const pred2 = pred->getLogicalPredecessorLane();
            assert(pred2 != nullptr);
            const MSLink* const predLink = pred2->getLinkTo(pred);
            assert(predLink != nullptr);
            return predLink->getState() == linkState;
        }
    }
}


void
MSLink::writeApproaching(OutputDevice& od, const std::string fromLaneID) const {
    if (myApproachingVehicles.size() > 0) {
        od.openTag("link");
        od.writeAttr(SUMO_ATTR_FROM, fromLaneID);
        const std::string via = getViaLane() == nullptr ? "" : getViaLane()->getID();
        od.writeAttr(SUMO_ATTR_VIA, via);
        od.writeAttr(SUMO_ATTR_TO, getLane() == nullptr ? "" : getLane()->getID());
        std::vector<std::pair<SUMOTime, const SUMOVehicle*> > toSort; // stabilize output
        for (auto it : myApproachingVehicles) {
            toSort.push_back(std::make_pair(it.second.arrivalTime, it.first));
        }
        std::sort(toSort.begin(), toSort.end());
        for (std::vector<std::pair<SUMOTime, const SUMOVehicle*> >::const_iterator it = toSort.begin(); it != toSort.end(); ++it) {
            od.openTag("approaching");
            const ApproachingVehicleInformation& avi = myApproachingVehicles.find(it->second)->second;
            od.writeAttr(SUMO_ATTR_ID, it->second->getID());
            od.writeAttr(SUMO_ATTR_IMPATIENCE, it->second->getImpatience());
            od.writeAttr("arrivalTime", time2string(avi.arrivalTime));
            od.writeAttr("leaveTime", time2string(avi.leavingTime));
            od.writeAttr("arrivalSpeed", toString(avi.arrivalSpeed));
            od.writeAttr("arrivalSpeedBraking", toString(avi.arrivalSpeedBraking));
            od.writeAttr("leaveSpeed", toString(avi.leaveSpeed));
            od.writeAttr("willPass", toString(avi.willPass));
            od.closeTag();
        }
        od.closeTag();
    }
}


double
MSLink::getInternalLengthsAfter() const {
    double len = 0.;
    MSLane* lane = myInternalLane;

    while (lane != nullptr && lane->isInternal()) {
        len += lane->getLength();
        lane = lane->getLinkCont()[0]->getViaLane();
    }
    return len;
}

double
MSLink::getInternalLengthsBefore() const {
    double len = 0.;
    const MSLane* lane = myInternalLane;

    while (lane != nullptr && lane->isInternal()) {
        len += lane->getLength();
        if (lane->getIncomingLanes().size() == 1) {
            lane = lane->getIncomingLanes()[0].lane;
        } else {
            break;
        }
    }
    return len;
}


double
MSLink::getLengthsBeforeCrossing(const MSLane* foeLane) const {
    MSLane* via = myInternalLane;
    double totalDist = 0.;
    bool foundCrossing = false;
    while (via != nullptr) {
        MSLink* link = via->getLinkCont()[0];
        double dist = link->getLengthBeforeCrossing(foeLane);
        if (dist != INVALID_DOUBLE) {
            // found conflicting lane
            totalDist += dist;
            foundCrossing = true;
            break;
        } else {
            totalDist += via->getLength();
            via = link->getViaLane();
        }
    }
    if (foundCrossing) {
        return totalDist;
    } else {
        return INVALID_DOUBLE;
    }
}


double
MSLink::getLengthBeforeCrossing(const MSLane* foeLane) const {
    int foe_ix;
    for (foe_ix = 0; foe_ix != (int)myFoeLanes.size(); ++foe_ix) {
        if (myFoeLanes[foe_ix] == foeLane) {
            break;
        }
    }
    if (foe_ix == (int)myFoeLanes.size()) {
        // no conflict with the given lane, indicate by returning -1
#ifdef MSLink_DEBUG_CROSSING_POINTS
        std::cout << "No crossing of lanes '" << foeLane->getID() << "' and '" << myInternalLaneBefore->getID() << "'" << std::endl;
#endif
        return INVALID_DOUBLE;
    } else {
        // found conflicting lane index
        double dist = myInternalLaneBefore->getLength() - myConflicts[foe_ix].getLengthBehindCrossing(this);
        if (dist == -10000.) {
            // this is the value in myConflicts, if the relation allows intersection but none is present for the actual geometry.
            return INVALID_DOUBLE;
        }
#ifdef MSLink_DEBUG_CROSSING_POINTS
        std::cout << "Crossing of lanes '" << myInternalLaneBefore->getID() << "' and '" << foeLane->getID()
                  << "' at distance " << dist << " (approach along '"
                  <<  myInternalLaneBefore->getEntryLink()->getLaneBefore()->getID() << "')" << std::endl;
#endif
        return dist;
    }
}


bool
MSLink::isEntryLink() const {
    if (MSGlobals::gUsingInternalLanes) {
        return myInternalLane != nullptr && myInternalLaneBefore == nullptr;
    } else {
        return false;
    }
}

bool
MSLink::isConflictEntryLink() const {
    // either a non-cont entry link or the link after a cont-link
    return !myAmCont && (isEntryLink() || (myInternalLaneBefore != nullptr && myInternalLane != nullptr));
}

bool
MSLink::isExitLink() const {
    if (MSGlobals::gUsingInternalLanes) {
        return myInternalLaneBefore != nullptr && myInternalLane == nullptr;
    } else {
        return false;
    }
}

bool
MSLink::isExitLinkAfterInternalJunction() const {
    if (MSGlobals::gUsingInternalLanes) {
        return (getInternalLaneBefore() != nullptr
                && myInternalLaneBefore->getIncomingLanes().size() == 1
                && myInternalLaneBefore->getIncomingLanes().front().viaLink->isInternalJunctionLink());
    } else {
        return false;
    }
}


const MSLink*
MSLink::getCorrespondingExitLink() const {
    MSLane* lane = myInternalLane;
    const MSLink* link = this;
    while (lane != nullptr) {
        link = lane->getLinkCont()[0];
        lane = link->getViaLane();
    }
    return link;
}


const MSLink*
MSLink::getCorrespondingEntryLink() const {
    const MSLink* link = this;
    while (link->myLaneBefore->isInternal()) {
        assert(myLaneBefore->getIncomingLanes().size() == 1);
        link = link->myLaneBefore->getIncomingLanes().front().viaLink;
    }
    return link;
}


bool
MSLink::isInternalJunctionLink() const {
    return getInternalLaneBefore() != nullptr && myInternalLane != nullptr;
}


const MSLink::LinkLeaders
MSLink::getLeaderInfo(const MSVehicle* ego, double dist, std::vector<const MSPerson*>* collectBlockers, bool isShadowLink) const {
    LinkLeaders result;
    // this link needs to start at an internal lane (either an exit link or between two internal lanes)
    // or it must be queried by the pedestrian model (ego == 0)
    if (ego != nullptr && (!fromInternalLane() || ego->getLaneChangeModel().isOpposite())) {
        // ignore link leaders
        return result;
    }
    //gDebugFlag1 = true;
    if (gDebugFlag1) {
        std::cout << SIMTIME << " getLeaderInfo link=" << getDescription() << " dist=" << dist << " isShadowLink=" << isShadowLink << "\n";
    }
    if (MSGlobals::gComputeLC && ego != nullptr && ego->getLane()->isNormal()) {
        const MSLink* junctionEntry = getLaneBefore()->getEntryLink();
        if (junctionEntry->haveRed() && !ego->ignoreRed(junctionEntry, true)
                // check oncoming on bidiLane during laneChanging
                && (!MSGlobals::gComputeLC || junctionEntry->getLaneBefore()->getBidiLane() == nullptr)) {
            if (gDebugFlag1) {
                std::cout << "   ignore linkLeaders beyond red light\n";
            }
            return result;
        }
    }
    // this is an exit link
    const double extraGap = ego != nullptr ? ego->getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_EXTRA_GAP, 0) : 0;
    for (int i = 0; i < (int)myFoeLanes.size(); ++i) {
        const MSLane* foeLane = myFoeLanes[i];
        const MSLink* foeExitLink = foeLane->getLinkCont()[0];
        // distance from the querying vehicle to the crossing point with foeLane
        double distToCrossing = dist - myConflicts[i].getLengthBehindCrossing(this);
        const double foeDistToCrossing = foeLane->getLength() - myConflicts[i].getFoeLengthBehindCrossing(foeExitLink);
        const bool sameTarget = (myLane == foeExitLink->getLane()) && !isInternalJunctionLink() && !foeExitLink->isInternalJunctionLink();
        const bool sameSource = (myInternalLaneBefore != nullptr && myInternalLaneBefore->getLogicalPredecessorLane() == foeLane->getLogicalPredecessorLane());
        const double crossingWidth = (sameTarget || sameSource) ? 0 : myConflicts[i].conflictSize;
        const double foeCrossingWidth = (sameTarget || sameSource) ? 0 : myConflicts[i].getFoeConflictSize(foeExitLink);
        // special treatment of contLane foe only applies if this lane is not a contLane or contLane follower itself
        const bool contLane = (foeExitLink->getViaLaneOrLane()->getEdge().isInternal() && !(
                                   isInternalJunctionLink() || isExitLinkAfterInternalJunction()));
        if (gDebugFlag1) {
            std::cout << " distToCrossing=" << distToCrossing << " foeLane=" << foeLane->getID() << " cWidth=" << crossingWidth
                      << " ijl=" << isInternalJunctionLink() << " sT=" << sameTarget << " sS=" << sameSource
                      << " lbc=" << myConflicts[i].getLengthBehindCrossing(this)
                      << " flbc=" << myConflicts[i].getFoeLengthBehindCrossing(foeExitLink)
                      << " cw=" << crossingWidth
                      << " fcw=" << foeCrossingWidth
                      << " contLane=" << contLane
                      << " state=" << toString(myState)
                      << " foeState=" << toString(foeExitLink->getState())
                      << "\n";
        }
        if (distToCrossing + crossingWidth < 0 && !sameTarget
                && (ego == nullptr || !MSGlobals::gComputeLC || distToCrossing + crossingWidth + ego->getVehicleType().getLength() < 0)) {
            continue; // vehicle is behind the crossing point, continue with next foe lane
        }
        bool ignoreGreenCont = false;
        bool foeIndirect = false;
        if (contLane) {
            const MSLink* entry = getLaneBefore()->getEntryLink();
            const MSLink* foeEntry = foeLane->getEntryLink();
            foeIndirect = foeEntry->myAmIndirect;
            if (entry != nullptr && entry->haveGreen()
                    && foeEntry != nullptr && foeEntry->haveGreen()
                    && entry->myLaneBefore != foeEntry->myLaneBefore)  {
                // ignore vehicles before an internaljunction as long as they are still in green minor mode
                ignoreGreenCont = true;
            }
        }
        if (foeIndirect && distToCrossing >= NO_INTERSECTION) {
            if (gDebugFlag1) {
                std::cout << " ignore:noIntersection\n";
            }
            continue;
        }
        // it is not sufficient to return the last vehicle on the foeLane because ego might be its leader
        // therefore we return all vehicles on the lane
        //
        // special care must be taken for continuation lanes. (next lane is also internal)
        // vehicles on cont. lanes or on internal lanes with the same target as this link can not be ignored
        // and should block (gap = -1) unless they are part of an indirect turn
        MSLane::AnyVehicleIterator end = foeLane->anyVehiclesEnd();
        for (MSLane::AnyVehicleIterator it_veh = foeLane->anyVehiclesBegin(); it_veh != end; ++it_veh) {
            MSVehicle* leader = (MSVehicle*)*it_veh;
            const double leaderBack = leader->getBackPositionOnLane(foeLane) - extraGap;
            const double leaderBackDist = foeDistToCrossing - leaderBack;
            const double l2 = ego != nullptr ? ego->getLength() + 2 : 0; // add some slack to account for further meeting-angle effects
            const double sagitta = ego != nullptr && myRadius != std::numeric_limits<double>::max() ? myRadius - sqrt(myRadius * myRadius - 0.25 * l2 * l2) : 0;
            const bool pastTheCrossingPoint = leaderBackDist + foeCrossingWidth + sagitta < 0;
            const bool enteredTheCrossingPoint = leaderBackDist < leader->getVehicleType().getLength();
            const bool foeIsBicycleTurn = (leader->getVehicleType().getVehicleClass() == SVC_BICYCLE
                                           && foeLane->getIncomingLanes().front().viaLink->getDirection() == LinkDirection::LEFT);
            const bool ignoreIndirectBicycleTurn = pastTheCrossingPoint && foeIsBicycleTurn;
            const bool cannotIgnore = ((contLane && !ignoreIndirectBicycleTurn) || sameTarget || sameSource) && ego != nullptr;
            const bool inTheWay = ((((!pastTheCrossingPoint && distToCrossing > 0) || (sameTarget && distToCrossing > leaderBackDist - leader->getLength()))
                                    && enteredTheCrossingPoint
                                    && (!foeExitLink->isInternalJunctionLink() || foeIsBicycleTurn))
                                   || foeExitLink->getLaneBefore()->getNormalPredecessorLane() == myLane->getBidiLane());
            const bool isOpposite = leader->getLaneChangeModel().isOpposite();
            const auto avi = foeExitLink->getApproaching(leader);
            // if leader is not found, assume that it performed a lane change in the last step
            const bool willPass = avi.willPass || (avi.arrivalTime == INVALID_TIME && sameTarget);
            if (gDebugFlag1) {
                std::cout << " candidate leader=" << leader->getID()
                          << " cannotIgnore=" << cannotIgnore
                          << " fdtc=" << foeDistToCrossing
                          << " lb=" << leaderBack
                          << " lbd=" << leaderBackDist
                          << " fcwidth=" << foeCrossingWidth
                          << " r=" << myRadius
                          << " sagitta=" << sagitta
                          << " foePastCP=" << pastTheCrossingPoint
                          << " foeEnteredCP=" << enteredTheCrossingPoint
                          << " inTheWay=" << inTheWay
                          << " willPass=" << willPass
                          << " isFrontOnLane=" << leader->isFrontOnLane(foeLane)
                          << " ignoreGreenCont=" << ignoreGreenCont
                          << " foeIndirect=" << foeIndirect
                          << " foeBikeTurn=" << foeIsBicycleTurn
                          << " isOpposite=" << isOpposite << "\n";
            }
            if (leader == ego) {
                continue;
            }
            // ignore greenCont foe vehicles that are not in the way
            if (!inTheWay && ignoreGreenCont) {
                if (gDebugFlag1) {
                    std::cout << "   ignoreGreenCont\n";
                }
                continue;
            }
            // after entering the conflict area, ignore foe vehicles that are not in the way
            if ((!MSGlobals::gComputeLC || (ego != nullptr && ego->getLane() == foeLane) || MSGlobals::gSublane)
                    && distToCrossing < -POSITION_EPS && !inTheWay
                    && (ego == nullptr || !MSGlobals::gComputeLC || distToCrossing < -ego->getVehicleType().getLength())) {
                if (gDebugFlag1) {
                    std::cout << "   ego entered conflict area\n";
                }
                continue;
            }
            if (!MSGlobals::gComputeLC
                    && sameSource
                    && &ego->getLane()->getEdge() == &myInternalLaneBefore->getEdge()
                    && leaderBack + leader->getLength() < ego->getPositionOnLane() - ego->getLength()) {
                // ego is already on the junction and clearly ahead of foe
                if (gDebugFlag1) {
                    std::cout << "   ego ahead of same-source foe\n";
                }
                continue;
            }

            // ignore foe vehicles that will not pass
            if ((!cannotIgnore || leader->isStopped() || sameTarget)
                    && !willPass
                    && leader->isFrontOnLane(foeLane)
                    && !isOpposite
                    && !inTheWay
                    // willPass is false if the vehicle is already on the stopping edge
                    && !leader->willStop()) {
                if (gDebugFlag1) {
                    std::cout << "   foe will not pass\n";
                }
                continue;
            }
            if (leader->isBidiOn(foeLane)) {
                // conflict resolved via forward lane of the foe
                continue;
            }
            // check whether foe is blocked and might need to change before leaving the junction
            const bool foeStrategicBlocked = (leader->getLaneChangeModel().isStrategicBlocked() &&
                                              leader->getCarFollowModel().brakeGap(leader->getSpeed()) <= foeLane->getLength() - leaderBack);
            const bool sameInternalEdge = &myInternalLaneBefore->getEdge() == &foeExitLink->getInternalLaneBefore()->getEdge();

            const bool foeLaneIsBidi = myInternalLaneBefore->getBidiLane() == foeLane;
            if (MSGlobals::gSublane && ego != nullptr && (sameSource || sameTarget || foeLaneIsBidi)
                    && (!foeStrategicBlocked || sameInternalEdge)) {
                if (ego->getLane() == leader->getLane()) {
                    continue;
                }
                // ignore vehicles if not in conflict sublane-wise
                const double egoLatOffset = isShadowLink ? ego->getLatOffset(ego->getLaneChangeModel().getShadowLane()) : 0;
                const double posLat = ego->getLateralPositionOnLane() + egoLatOffset;
                double posLatLeader = leader->getLateralPositionOnLane() + leader->getLatOffset(foeLane);
                if (foeLaneIsBidi) {
                    // leader is oncoming
                    posLatLeader = foeLane->getWidth() - posLatLeader;
                }
                const double latGap = (fabs(posLat - posLatLeader)
                                       - 0.5 * (ego->getVehicleType().getWidth() + leader->getVehicleType().getWidth()));
                const double maneuverDist = leader->getLaneChangeModel().getManeuverDist() * (posLat < posLatLeader ? -1 : 1);
                if (gDebugFlag1) {
                    std::cout << " checkIgnore sublaneFoe lane=" << myInternalLaneBefore->getID()
                              << " sameSource=" << sameSource
                              << " sameTarget=" << sameTarget
                              << " foeLaneIsBidi=" << foeLaneIsBidi
                              << " foeLane=" << foeLane->getID()
                              << " leader=" << leader->getID()
                              << " egoLane=" << ego->getLane()->getID()
                              << " leaderLane=" << leader->getLane()->getID()
                              << " egoLat=" << posLat
                              << " egoLatOffset=" << egoLatOffset
                              << " leaderLat=" << posLatLeader
                              << " leaderLatOffset=" << leader->getLatOffset(foeLane)
                              << " latGap=" << latGap
                              << " maneuverDist=" << maneuverDist
                              << " computeLC=" << MSGlobals::gComputeLC
                              << " egoMaxSpeedLat=" << ego->getVehicleType().getMaxSpeedLat()
                              << "\n";
                }
                if (latGap > 0 && (latGap > maneuverDist || !sameTarget || !MSGlobals::gComputeLC)
                        // do not perform sublane changes that interfere with the leader vehicle
                        && (!MSGlobals::gComputeLC || latGap > ego->getVehicleType().getMaxSpeedLat())) {
                    const MSLink* foeEntryLink = foeLane->getIncomingLanes().front().viaLink;
                    if (sameSource) {
                        // for lanes from the same edge, higer index implies a
                        // connection further to the left
                        const bool leaderFromRight = (myIndex > foeEntryLink->getIndex());
                        if ((posLat > posLatLeader) == leaderFromRight) {
                            // ignore speed since lanes diverge
                            if (gDebugFlag1) {
                                std::cout << "   ignored (same source) leaderFromRight=" << leaderFromRight << "\n";
                            }
                            continue;
                        }
                    } else if (sameTarget) {
                        // for lanes from different edges we cannot rely on the
                        // index due to wrap-around issues
                        if (myDirection != foeEntryLink->getDirection()) {
                            bool leaderFromRight = foeEntryLink->getDirection() < myDirection;
                            // leader vehicle should not move towards ego
                            if (MSGlobals::gLefthand) {
                                leaderFromRight = !leaderFromRight;
                            }
                            if ((posLat > posLatLeader) == leaderFromRight
                                    // leader should keep lateral position or move away from ego
                                    && (leader->getLaneChangeModel().getSpeedLat() == 0
                                        || leaderFromRight == (leader->getLaneChangeModel().getSpeedLat() < latGap))
                                    && (ego->getLaneChangeModel().getSpeedLat() == 0
                                        || leaderFromRight == (ego->getLaneChangeModel().getSpeedLat() > latGap))) {
                                if (gDebugFlag1) {
                                    std::cout << "   ignored (different source) leaderFromRight=" << leaderFromRight << "\n";
                                }
                                continue;
                            }
                        } else {
                            // XXX figure out relative direction somehow
                        }
                    } else {
                        if (gDebugFlag1) {
                            std::cout << "   ignored oncoming bidi leader\n";
                        }
                        continue;
                    }
                }
            }
            if (leader->getWaitingTime() < MSGlobals::gIgnoreJunctionBlocker) {
                // compute distance between vehicles on the superimposition of both lanes
                // where the crossing point is the common point
                double gap;
                bool fromLeft = true;
                if (ego == nullptr) {
                    // request from pedestrian model. return distance between leaderBack and crossing point
                    //std::cout << "   foeLane=" << foeLane->getID() << " leaderBack=" << leaderBack << " foeDistToCrossing=" << foeDistToCrossing << " foeLength=" << foeLane->getLength() << " foebehind=" << myConflicts[i].second << " dist=" << dist << " behind=" << myConflicts[i].first << "\n";
                    gap = leaderBackDist;
                    // distToCrossing should not take into account the with of the foe lane
                    // (which was subtracted in setRequestInformation)
                    // Instead, the width of the foe vehicle is used directly by the caller.
                    distToCrossing += myConflicts[i].conflictSize / 2;
                    if (gap + foeCrossingWidth < 0) {
                        // leader is completely past the crossing point
                        // or there is no crossing point
                        continue; // next vehicle
                    }
                    // we need to determine whether the vehicle passes the
                    // crossing from the left or the right (heuristic)
                    fromLeft = foeDistToCrossing > 0.5 * foeLane->getLength();
                } else if ((contLane && !sameSource && !ignoreIndirectBicycleTurn) || isOpposite) {
                    gap = -std::numeric_limits<double>::max(); // always break for vehicles which are on a continuation lane or for opposite-direction vehicles
                } else {
                    if (pastTheCrossingPoint && !sameTarget) {
                        // leader is completely past the crossing point
                        // or there is no crossing point
                        if (gDebugFlag1) {
                            std::cout << " foePastCP ignored\n";
                        }
                        continue;
                    }
                    double leaderBackDist2 = leaderBackDist;
                    if (sameTarget && leaderBackDist2 < 0) {
                        const double mismatch = myConflicts[i].getFoeLengthBehindCrossing(foeExitLink) - myConflicts[i].getLengthBehindCrossing(this);
                        if (mismatch > 0) {
                            leaderBackDist2 += mismatch;
                        }
                    }
                    if (gDebugFlag1) {
                        std::cout << " distToCrossing=" << distToCrossing << " leaderBack=" << leaderBack
                                  << " backDist=" << leaderBackDist
                                  << " backDist2=" << leaderBackDist2
                                  << " blockedStrategic=" << leader->getLaneChangeModel().isStrategicBlocked()
                                  << "\n";
                    }
                    gap = distToCrossing - ego->getVehicleType().getMinGap() - leaderBackDist2 - foeCrossingWidth;
                }
                // if the foe is already moving off the intersection, we may
                // advance up to the crossing point unless we have the same target or same source
                // (for sameSource, the crossing point indicates the point of divergence)
                const bool stopAsap = ((leader->isFrontOnLane(foeLane) ? cannotIgnore : (sameTarget || sameSource))
                    || (ego != nullptr && ego->getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_ADVANCE, 1.0) == 0.0));
                if (gDebugFlag1) {
                    std::cout << " leader=" << leader->getID() << " contLane=" << contLane << " cannotIgnore=" << cannotIgnore << " stopAsap=" << stopAsap << " gap=" << gap << "\n";
                }
                if (ignoreFoe(ego, leader)) {
                    continue;
                }
                const int llFlags = ((fromLeft ? LL_FROM_LEFT : 0) |
                                     (inTheWay ? LL_IN_THE_WAY : 0) |
                                     (sameSource ? LL_SAME_SOURCE : 0) |
                                     (sameTarget ? LL_SAME_TARGET : 0));
                result.emplace_back(leader, gap, stopAsap ? -1 : distToCrossing, llFlags, leader->getLatOffset(foeLane));
            }

        }
        if (ego != nullptr && MSNet::getInstance()->hasPersons()) {
            // check for crossing pedestrians (keep driving if already on top of the crossing
            const double distToPeds = distToCrossing - ego->getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_STOPLINE_CROSSING_GAP, MSPModel::SAFETY_GAP);
            const double vehWidth = ego->getVehicleType().getWidth() + MSPModel::SAFETY_GAP; // + configurable safety gap
            /// @todo consider lateral position (depending on whether the crossing is encountered on the way in or out)
            // @check lefthand?!
            const bool wayIn = myConflicts[i].lengthBehindCrossing < myLaneBefore->getLength() * 0.5;
            const double vehSideOffset = (foeDistToCrossing + myLaneBefore->getWidth() * 0.5 - vehWidth * 0.5
                                          + ego->getLateralPositionOnLane() * (wayIn ? -1 : 1));
            // can access the movement model here since we already checked for existing persons above
            if (distToPeds >= -MSPModel::SAFETY_GAP && MSNet::getInstance()->getPersonControl().getMovementModel()->blockedAtDist(ego, foeLane, vehSideOffset, vehWidth,
                    ego->getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_CROSSING_GAP, JM_CROSSING_GAP_DEFAULT),
                    collectBlockers)) {
                result.emplace_back(nullptr, -1, distToPeds);
            } else if (foeLane->isCrossing() && ego->getLane()->isInternal() && ego->getLane()->getEdge().getToJunction() == myJunction) {
                const MSLink* crossingLink = foeLane->getIncomingLanes()[0].viaLink;
                if (distToCrossing > 0 && crossingLink->havePriority() && crossingLink->myApproachingPersons != nullptr) {
                    // a person might step on the crossing at any moment, since ego
                    // is already on the junction, the opened() check is not done anymore
                    const double timeToEnterCrossing = distToCrossing / MAX2(ego->getSpeed(), 1.0);
                    for (const auto& item : (*crossingLink->myApproachingPersons)) {
                        if (!ignoreFoe(ego, item.first) && timeToEnterCrossing > STEPS2TIME(item.second.arrivalTime - SIMSTEP)) {
                            if (gDebugFlag1) {
                                std::cout << SIMTIME << ": " << ego->getID() << " breaking for approaching person " << item.first->getID()
                                          //<< " dtc=" << distToCrossing << " ttc=" << distToCrossing / MAX2(ego->getSpeed(), 1.0) << " foeAT=" << item.second.arrivalTime << " foeTTC=" << STEPS2TIME(item.second.arrivalTime - SIMSTEP)
                                          << "\n";
                            }
                            result.emplace_back(nullptr, -1, distToPeds);
                            break;
                            //} else {
                            //    if (gDebugFlag1) {
                            //        std::cout << SIMTIME << ": " << ego->getID() << " notBreaking for approaching person " << item.first->getID()
                            //            << " dtc=" << distToCrossing << " ttc=" << distToCrossing / MAX2(ego->getSpeed(), 1.0) << " foeAT=" << item.second.arrivalTime << " foeTTC=" << STEPS2TIME(item.second.arrivalTime - SIMSTEP)
                            //            << "\n";
                            //    }
                        }
                    }
                }
            }
        }
    }

    //std::cout << SIMTIME << " ego=" << Named::getIDSecure(ego) << " link=" << getViaLaneOrLane()->getID() << " myWalkingAreaFoe=" << Named::getIDSecure(myWalkingAreaFoe) << "\n";
    if (ego != nullptr) {
        checkWalkingAreaFoe(ego, myWalkingAreaFoe, collectBlockers, result);
        checkWalkingAreaFoe(ego, myWalkingAreaFoeExit, collectBlockers, result);
    }

    if (MSGlobals::gLateralResolution > 0 && ego != nullptr && !isShadowLink) {
        // check for foes on the same edge
        for (std::vector<MSLane*>::const_iterator it = mySublaneFoeLanes.begin(); it != mySublaneFoeLanes.end(); ++it) {
            const MSLane* foeLane = *it;
            MSLane::AnyVehicleIterator end = foeLane->anyVehiclesEnd();
            for (MSLane::AnyVehicleIterator it_veh = foeLane->anyVehiclesBegin(); it_veh != end; ++it_veh) {
                MSVehicle* leader = (MSVehicle*)*it_veh;
                if (leader == ego) {
                    continue;
                }
                if (leader->getLane()->isNormal()) {
                    // leader is past the conflict point
                    continue;
                }
                const double maxLength = MAX2(myInternalLaneBefore->getLength(), foeLane->getLength());
                const double gap = dist - maxLength - ego->getVehicleType().getMinGap() + leader->getBackPositionOnLane(foeLane) - extraGap;
                if (gap < -(ego->getVehicleType().getMinGap() + leader->getLength())) {
                    // ego is ahead of leader
                    continue;
                }
                const double posLat = ego->getLateralPositionOnLane();
                const double posLatLeader = leader->getLateralPositionOnLane() + leader->getLatOffset(foeLane);
                if (gDebugFlag1) {
                    std::cout << " sublaneFoe lane=" << myInternalLaneBefore->getID()
                              << " foeLane=" << foeLane->getID()
                              << " leader=" << leader->getID()
                              << " egoLane=" << ego->getLane()->getID()
                              << " leaderLane=" << leader->getLane()->getID()
                              << " gap=" << gap
                              << " egoLat=" << posLat
                              << " leaderLat=" << posLatLeader
                              << " leaderLatOffset=" << leader->getLatOffset(foeLane)
                              << " egoIndex=" << myInternalLaneBefore->getIndex()
                              << " foeIndex=" << foeLane->getIndex()
                              << " dist=" << dist
                              << " leaderBack=" << leader->getBackPositionOnLane(foeLane)
                              << "\n";
                }
                // there only is a conflict if the paths cross
                if ((posLat < posLatLeader && myInternalLaneBefore->getIndex() > foeLane->getIndex())
                        || (posLat > posLatLeader && myInternalLaneBefore->getIndex() < foeLane->getIndex())) {
                    if (gDebugFlag1) {
                        std::cout << SIMTIME << " blocked by " << leader->getID() << " (sublane split) foeLane=" << foeLane->getID() << "\n";
                    }
                    if (ignoreFoe(ego, leader)) {
                        continue;
                    }
                    result.emplace_back(leader, gap, -1);
                }
            }
        }
    }
    return result;
}


void
MSLink::checkWalkingAreaFoe(const MSVehicle* ego, const MSLane* foeLane, std::vector<const MSPerson*>* collectBlockers, LinkLeaders& result) const {
    if (foeLane != nullptr && foeLane->getEdge().getPersons().size() > 0) {
        // pedestrians may be on an arbitrary path across this
        // walkingarea. make sure to keep enough distance.
        // This is a simple but conservative solution that could be improved
        // by ignoring pedestrians that are "obviously" not on a collision course
        double distToPeds = std::numeric_limits<double>::max();
        assert(myInternalLaneBefore != nullptr);
        PositionVector egoPath = myInternalLaneBefore->getShape();
        if (ego->getLateralPositionOnLane() != 0) {
            egoPath.move2side((MSGlobals::gLefthand ? 1 : -1) * ego->getLateralPositionOnLane());
        }
        for (MSTransportable* t : foeLane->getEdge().getPersons()) {
            MSPerson* p = static_cast<MSPerson*>(t);
            double dist = ego->getPosition().distanceTo2D(p->getPosition()) - p->getVehicleType().getLength();
            const bool inFront = isInFront(ego, egoPath, p->getPosition()) || isInFront(ego, egoPath, getFuturePosition(p));
            if (inFront) {
                dist -= ego->getVehicleType().getMinGap();
            }
#ifdef DEBUG_WALKINGAREA
            if (ego->isSelected()) {
                std::cout << SIMTIME << " veh=" << ego->getID() << " ped=" << p->getID()
                          << " pos=" << ego->getPosition() << " pedPos=" << p->getPosition()
                          << " futurePedPos=" << getFuturePosition(p)
                          << " rawDist=" << ego->getPosition().distanceTo2D(p->getPosition())
                          << " inFront=" << inFront
                          << " dist=" << dist << "\n";
            }
#endif
            if (dist < ego->getVehicleType().getWidth() / 2 || inFront) {
                if (inFront) {
                    const double oncomingFactor = isOnComingPed(ego, p);
                    if (oncomingFactor > 0) {
                        // account for pedestrian movement while closing in
                        const double timeToStop = sqrt(dist) / 2;
                        const double pedDist = p->getMaxSpeed() * MAX2(timeToStop, TS) * oncomingFactor;
                        dist = MAX2(0.0, dist - pedDist);
#ifdef DEBUG_WALKINGAREA
                        if (ego->isSelected()) {
                            std::cout << "    timeToStop=" << timeToStop << " pedDist=" << pedDist << " factor=" << oncomingFactor << " dist2=" << dist << "\n";
                        }
#endif
                    }
                }
                if (ignoreFoe(ego, p)) {
                    continue;
                }
                distToPeds = MIN2(distToPeds, dist);
                if (collectBlockers != nullptr) {
                    collectBlockers->push_back(p);
                }
            }
        }
        if (distToPeds != std::numeric_limits<double>::max()) {
            // leave extra space in front
            result.emplace_back(nullptr, -1, distToPeds);
        }
    }
}

bool
MSLink::isInFront(const MSVehicle* ego, const PositionVector& egoPath, const Position& pPos) const {
    const double pedAngle = ego->getPosition().angleTo2D(pPos);
    const double angleDiff = fabs(GeomHelper::angleDiff(ego->getAngle(), pedAngle));
#ifdef DEBUG_WALKINGAREA
    if (ego->isSelected()) {
        std::cout << " angleDiff=" << RAD2DEG(angleDiff) << "\n";
    }
#endif
    if (angleDiff < DEG2RAD(75)) {
        return egoPath.distance2D(pPos) < ego->getVehicleType().getWidth() + MSPModel::SAFETY_GAP;
    }
    return false;
}


double
MSLink::isOnComingPed(const MSVehicle* ego, const MSPerson* p) const {
    const double pedToEgoAngle = p->getPosition().angleTo2D(ego->getPosition());
    const double angleDiff = fabs(GeomHelper::angleDiff(p->getAngle(), pedToEgoAngle));
#ifdef DEBUG_WALKINGAREA
    if (ego->isSelected()) {
        std::cout << " ped-angleDiff=" << RAD2DEG(angleDiff) << " res=" << cos(angleDiff) << "\n";
    }
#endif
    if (angleDiff <= DEG2RAD(90)) {
        ;
        return cos(angleDiff);
    } else {
        return 0;
    }
}


Position
MSLink::getFuturePosition(const MSPerson* p, double timeHorizon) const {
    const double a = p->getAngle();
    const double dist = timeHorizon * p->getMaxSpeed();

    const Position offset(cos(a) * dist, sin(a) * dist);
    return p->getPosition() + offset;
}


MSLink*
MSLink::getParallelLink(int direction) const {
    if (direction == -1) {
        return myParallelRight;
    } else if (direction == 1) {
        return myParallelLeft;
    } else {
        assert(false || myLane->getOpposite() != nullptr);
        return nullptr;
    }
}

MSLink*
MSLink::getOppositeDirectionLink() const {
    if (myLane->getOpposite() != nullptr && myLaneBefore->getOpposite() != nullptr) {
        for (MSLink* cand : myLane->getOpposite()->getLinkCont()) {
            if (cand->getLane() == myLaneBefore->getOpposite()) {
                return cand;
            }
        }
    }
    return nullptr;
}


MSLink*
MSLink::computeParallelLink(int direction) {
    const MSLane* const before = getLaneBefore()->getParallelLane(direction, false);
    const MSLane* const after = getLane()->getParallelLane(direction, false);
    if (before != nullptr && after != nullptr) {
        for (MSLink* const link : before->getLinkCont()) {
            if (link->getLane() == after) {
                return link;
            }
        }
    }
    return nullptr;
}


double
MSLink::getZipperSpeed(const MSVehicle* ego, const double dist, double vSafe,
                       SUMOTime arrivalTime,
                       const BlockingFoes* foes) const {
    if (myFoeLinks.size() == 0) {
        // link should have LINKSTATE_MAJOR in this case
        assert(false);
        return vSafe;
    } else if (myFoeLinks.size() > 1) {
        throw ProcessError("Zipper junctions with more than two conflicting lanes are not supported (at junction '"
                           + myJunction->getID() + "')");
    }
    const double brakeGap = ego->getCarFollowModel().brakeGap(ego->getSpeed(), ego->getCarFollowModel().getMaxDecel(), TS);
    if (dist > MAX2(myFoeVisibilityDistance, brakeGap)) {
#ifdef DEBUG_ZIPPER
        const SUMOTime now = MSNet::getInstance()->getCurrentTimeStep();
        DEBUGOUT(DEBUG_COND_ZIPPER, SIMTIME << " getZipperSpeed ego=" << ego->getID()
                 << " dist=" << dist << " bGap=" << brakeGap << " ignoring foes (arrival in " << STEPS2TIME(arrivalTime - now) << ")\n")
#endif
        return vSafe;
    }
#ifdef DEBUG_ZIPPER
    DEBUGOUT(DEBUG_COND_ZIPPER, SIMTIME << " getZipperSpeed ego=" << ego->getID()
             << " egoAT=" << arrivalTime
             << " dist=" << dist
             << " brakeGap=" << brakeGap
             << " vSafe=" << vSafe
             << " numFoes=" << foes->size()
             << "\n")
#endif
    MSLink* foeLink = myFoeLinks[0];
    for (const auto& item : *foes) {
        if (!item->isVehicle()) {
            continue;
        }
        const MSVehicle* foe = dynamic_cast<const MSVehicle*>(item);
        assert(foe != 0);
        const ApproachingVehicleInformation& avi = foeLink->getApproaching(foe);
        const double foeDist = (foe->isActive() ? avi.dist : MAX2(0.0, avi.dist -
                                STEPS2TIME(MSNet::getInstance()->getCurrentTimeStep() - foe->getLastActionTime()) * avi.speed));

        if (    // ignore vehicles that arrive after us (unless they are ahead and we could easily brake for them)
            ((avi.arrivalTime > arrivalTime) && !couldBrakeForLeader(dist, foeDist, ego, foe)) ||
            // also ignore vehicles that are behind us and are able to brake for us
            couldBrakeForLeader(foeDist, dist, foe, ego) ||
            // resolve ties by lane index
            (avi.arrivalTime == arrivalTime && foeDist == dist && ego->getLane()->getIndex() < foe->getLane()->getIndex())) {
#ifdef DEBUG_ZIPPER
            if (DEBUG_COND_ZIPPER) std::cout
                        << "    ignoring foe=" << foe->getID()
                        << " foeAT=" << avi.arrivalTime
                        << " foeDist=" << avi.dist
                        << " foeDist2=" << foeDist
                        << " foeSpeed=" << avi.speed
                        << " egoSpeed=" << ego->getSpeed()
                        << " deltaDist=" << foeDist - dist
                        << " delteSpeed=" << avi.speed - foe->getCarFollowModel().getMaxDecel() - ego->getSpeed()
                        << " egoCouldBrake=" << couldBrakeForLeader(dist, foeDist, ego, foe)
                        << " foeCouldBrake=" << couldBrakeForLeader(foeDist, dist, foe, ego)
                        << "\n";
#endif
            continue;
        }
        // the idea behind speed adaption is three-fold:
        // 1) ego needs to be in a car-following relationship with foe eventually
        //    thus, the ego speed should be equal to the follow speed once the foe enters
        //    the zipper junction
        // 2) ego vehicle needs to put a certain distance beteen himself and foe (safeGap)
        //    achieving this distance can be spread over time but computing
        //    safeGap is subject to estimation errors of future speeds
        // 3) deceleration can be spread out over the time until true
        //    car-following happens, at the start of speed adaptions, smaller
        //    decelerations should be sufficient

        // we cannot trust avi.arrivalSpeed if the foe has leader vehicles that are accelerating
        // lets try to extrapolate
        const double uMax = foe->getLane()->getVehicleMaxSpeed(foe);
        const double uAccel = foe->getCarFollowModel().estimateSpeedAfterDistance(foeDist, avi.speed, foe->getCarFollowModel().getMaxAccel());
        const double uEnd = MIN2(uMax, uAccel);
        const double uAvg = (avi.speed + uEnd) / 2;
        const double tf0 = foeDist / MAX2(NUMERICAL_EPS, uAvg);
        const double tf = MAX2(1.0, ceil((tf0) / TS) * TS);

        const double vMax = ego->getLane()->getVehicleMaxSpeed(ego);
        const double vAccel = ego->getCarFollowModel().estimateSpeedAfterDistance(dist, ego->getSpeed(), ego->getCarFollowModel().getMaxAccel());
        const double vDecel = ego->getCarFollowModel().estimateSpeedAfterDistance(dist, ego->getSpeed(), ego->getCarFollowModel().getMaxDecel());
        const double vEnd = MIN3(vMax, vAccel, MAX2(uEnd, vDecel));
        const double vAvg = (ego->getSpeed() + vEnd) / 2;
        const double te0 = dist / MAX2(NUMERICAL_EPS, vAvg);
        const double te = MAX2(1.0, ceil((te0) / TS) * TS);

        const double gap = dist - foe->getVehicleType().getLength() - ego->getVehicleType().getMinGap() - foeDist;
        const double safeGap = ego->getCarFollowModel().getSecureGap(ego, foe, vEnd, uEnd, foe->getCarFollowModel().getMaxDecel());
        // round t to next step size
        // increase gap to safeGap by the time foe reaches link
        // gap + u*t - (t * v + a * t^2 / 2) = safeGap
        const double deltaGap = gap + tf * uAvg - safeGap - vAvg * tf;
        const double a = 2 * deltaGap / (tf * tf);
        const double vSafeGap = ego->getSpeed() + ACCEL2SPEED(a);
        const double vFollow = ego->getCarFollowModel().followSpeed(
                                   ego, ego->getSpeed(), gap, avi.speed, foe->getCarFollowModel().getMaxDecel(), foe);

        // scale behavior based on ego time to link (te)
        const double w = MIN2(1.0, te / 10);
        const double maxDecel = w * ego->getCarFollowModel().getMaxDecel() + (1 - w) * ego->getCarFollowModel().getEmergencyDecel();
        const double vZipper = MAX3(vFollow, ego->getSpeed() - ACCEL2SPEED(maxDecel), w * vSafeGap + (1 - w) * vFollow);

        vSafe = MIN2(vSafe, vZipper);
#ifdef DEBUG_ZIPPER
        if (DEBUG_COND_ZIPPER) std::cout << "    adapting to foe=" << foe->getID()
                                             << " foeDist=" << foeDist
                                             << " foeSpeed=" << avi.speed
                                             << " foeAS=" << avi.arrivalSpeed
                                             << " egoSpeed=" << ego->getSpeed()
                                             << " uMax=" << uMax
                                             << " uAccel=" << uAccel
                                             << " uEnd=" << uEnd
                                             << " uAvg=" << uAvg
                                             << " gap=" << gap
                                             << " safeGap=" << safeGap
                                             << "\n      "
                                             << " tf=" << tf
                                             << " te=" << te
                                             << " dg=" << deltaGap
                                             << " aSafeGap=" << a
                                             << " vMax=" << vMax
                                             << " vAccel=" << vAccel
                                             << " vDecel=" << vDecel
                                             << " vEnd=" << vEnd
                                             << " vSafeGap=" << vSafeGap
                                             << " vFollow=" << vFollow
                                             << " w=" << w
                                             << " maxDecel=" << maxDecel
                                             << " vZipper=" << vZipper
                                             << " vSafe=" << vSafe
                                             << "\n";
#endif
    }
    return vSafe;
}


bool
MSLink::couldBrakeForLeader(double followDist, double leaderDist, const MSVehicle* follow, const MSVehicle* leader) {
    return (// leader is ahead of follower
               followDist > leaderDist &&
               // and follower could brake for 1 s to stay behind leader
               followDist - leaderDist > follow->getSpeed() - follow->getCarFollowModel().getMaxDecel() - leader->getSpeed());
}


void
MSLink::initParallelLinks() {
    myParallelRight = computeParallelLink(-1);
    myParallelLeft = computeParallelLink(1);
}

bool
MSLink::checkContOff() const {
    // check whether this link gets to keep its cont status switching the tls off
    // @note: this could also be pre-computed in netconvert
    // we check whether there is any major link from this edge
    for (const MSLane* cand : myLaneBefore->getEdge().getLanes()) {
        for (const MSLink* link : cand->getLinkCont()) {
            if (link->getOffState() == LINKSTATE_TL_OFF_NOSIGNAL) {
                return true;
            }
        }
    }
    return false;
}

bool
MSLink::lateralOverlap(double posLat, double width, double posLat2, double width2) {
    return fabs(posLat2 - posLat) < (width + width2) / 2;
}

std::string
MSLink::getDescription() const {
    return myLaneBefore->getID() + "->" + getViaLaneOrLane()->getID();
}


bool
MSLink::ignoreFoe(const SUMOTrafficObject* ego, const SUMOTrafficObject* foe) {
    if (ego == nullptr || !ego->getParameter().wasSet(VEHPARS_JUNCTIONMODEL_PARAMS_SET)) {
        return false;
    }
    const SUMOVehicleParameter& param = ego->getParameter();
    for (const std::string& typeID : StringTokenizer(param.getParameter(toString(SUMO_ATTR_JM_IGNORE_TYPES), "")).getVector()) {
        if (typeID == foe->getVehicleType().getID()) {
            return true;
        }
    }
    for (const std::string& id : StringTokenizer(param.getParameter(toString(SUMO_ATTR_JM_IGNORE_IDS), "")).getVector()) {
        if (id == foe->getID()) {
            return true;
        }
    }
    return false;
}


void
MSLink::updateDistToFoePedCrossing(double dist) {
    myDistToFoePedCrossing = MIN2(myDistToFoePedCrossing, dist);
}


std::pair<const SUMOVehicle* const, const MSLink::ApproachingVehicleInformation>
MSLink::getClosest() const {
    assert(getApproaching().size() > 0);
    double minDist = std::numeric_limits<double>::max();
    auto closestIt = getApproaching().begin();
    for (auto apprIt = getApproaching().begin(); apprIt != getApproaching().end(); apprIt++) {
        if (apprIt->second.dist < minDist) {
            minDist = apprIt->second.dist;
            closestIt = apprIt;
        }
    }
    // maybe a parallel link has a closer vehicle
    /*
    for (MSLink* link2 : link->getLaneBefore()->getLinkCont()) {
        if (link2 != link) {
            for (auto apprIt2 = link2->getApproaching().begin(); apprIt2 != link2->getApproaching().end(); apprIt2++) {
                if (apprIt2->second.dist < minDist) {
                    minDist = apprIt2->second.dist;
                    closestIt = apprIt2;
                }
            }
        }
    }
    */
    return *closestIt;
}


/****************************************************************************/
