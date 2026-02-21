/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2001-2026 German Aerospace Center (DLR) and others.
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
/// @file    NBNode.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @date    Tue, 20 Nov 2001
///
// The representation of a single node
/****************************************************************************/
#include <config.h>

#include <string>
#include <map>
#include <cassert>
#include <algorithm>
#include <vector>
#include <deque>
#include <set>
#include <cmath>
#include <iterator>
#include <utils/common/UtilExceptions.h>
#include <utils/common/StringUtils.h>
#include <utils/options/OptionsCont.h>
#include <utils/geom/GeomHelper.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/StdDefs.h>
#include <utils/common/ToString.h>
#include <utils/geom/GeoConvHelper.h>
#include <utils/iodevices/OutputDevice.h>
#include <iomanip>
#include "NBNode.h"
#include "NBAlgorithms.h"
#include "NBNodeCont.h"
#include "NBNodeShapeComputer.h"
#include "NBEdgeCont.h"
#include "NBTypeCont.h"
#include "NBHelpers.h"
#include "NBDistrict.h"
#include "NBContHelper.h"
#include "NBRequest.h"
#include "NBOwnTLDef.h"
#include "NBLoadedSUMOTLDef.h"
#include "NBTrafficLightLogicCont.h"
#include "NBTrafficLightDefinition.h"

// allow to extend a crossing across multiple edges
#define EXTEND_CROSSING_ANGLE_THRESHOLD 35.0 // degrees
// create intermediate walking areas if either of the following thresholds is exceeded
#define SPLIT_CROSSING_WIDTH_THRESHOLD 1.5 // meters
#define SPLIT_CROSSING_ANGLE_THRESHOLD 5 // degrees

// minimum length for a weaving section at a combined on-off ramp
#define MIN_WEAVE_LENGTH 20.0

//#define DEBUG_CONNECTION_GUESSING
//#define DEBUG_SMOOTH_GEOM
//#define DEBUG_PED_STRUCTURES
//#define DEBUG_EDGE_SORTING
//#define DEBUG_CROSSING_OUTLINE
//#define DEBUGCOND true
#define DEBUG_NODE_ID "C"
#define DEBUGCOND (getID() == DEBUG_NODE_ID)
#define DEBUGCOND2(obj) ((obj != 0 && (obj)->getID() == DEBUG_NODE_ID))
#ifdef DEBUG_PED_STRUCTURES
#define DEBUGCOUT(cond, msg) DEBUGOUT(cond, msg)
#else
#define DEBUGCOUT(cond, msg)
#endif

// ===========================================================================
// static members
// ===========================================================================
const int NBNode::FORWARD(1);
const int NBNode::BACKWARD(-1);
const double NBNode::UNSPECIFIED_RADIUS = -1;
const int NBNode::AVOID_WIDE_LEFT_TURN(1);
const int NBNode::AVOID_WIDE_RIGHT_TURN(2);
const int NBNode::FOUR_CONTROL_POINTS(4);
const int NBNode::AVOID_INTERSECTING_LEFT_TURNS(8);
const int NBNode::SCURVE_IGNORE(16);
const int NBNode::INDIRECT_LEFT(32);

SVCPermissions NBNode::myHaveRailSignalClasses;
SVCPermissions NBNode::myPermitUnsignalizedClasses;

// ===========================================================================
// method definitions
// ===========================================================================
/* -------------------------------------------------------------------------
 * NBNode::ApproachingDivider-methods
 * ----------------------------------------------------------------------- */
NBNode::ApproachingDivider::ApproachingDivider(
    const EdgeVector& approaching, NBEdge* currentOutgoing) :
    myApproaching(approaching),
    myCurrentOutgoing(currentOutgoing),
    myNumStraight(0),
    myIsBikeEdge(currentOutgoing->getPermissions() == SVC_BICYCLE) {
    // collect lanes which are expliclity targeted
    std::set<int> approachedLanes;
    bool hasIncomingBusLane = false;
    for (const NBEdge* const approachingEdge : myApproaching) {
        for (const NBEdge::Connection& con : approachingEdge->getConnections()) {
            if (con.toEdge == myCurrentOutgoing) {
                approachedLanes.insert(con.toLane);
            }
        }
        myDirections.push_back(approachingEdge->getToNode()->getDirection(approachingEdge, currentOutgoing));
        if (myDirections.back() == LinkDirection::STRAIGHT) {
            myNumStraight++;
        }
        hasIncomingBusLane |= (approachingEdge->getSpecialLane(SVC_BUS) != -1);
    }
    // compute the indices of lanes that should be targeted (excluding pedestrian
    // lanes that will be connected from walkingAreas and forbidden lanes)
    // if the lane is targeted by an explicitly set connection we need
    // to make it available anyway
    for (int i = 0; i < currentOutgoing->getNumLanes(); ++i) {
        const SVCPermissions lp = currentOutgoing->getPermissions(i);
        if ((lp == SVC_PEDESTRIAN
                // don't consider bicycle lanes as targets unless the target
                // edge is exclusively for bicycles
                || (lp == SVC_BICYCLE && !myIsBikeEdge)
                || (lp == SVC_BUS && hasIncomingBusLane)
                || isForbidden(lp))
                && approachedLanes.count(i) == 0) {
            continue;
        }
        myAvailableLanes.push_back(i);
    }
}


NBNode::ApproachingDivider::~ApproachingDivider() {}


void
NBNode::ApproachingDivider::execute(const int src, const int dest) {
    assert((int)myApproaching.size() > src);
    // get the origin edge
    NBEdge* incomingEdge = myApproaching[src];
    if (incomingEdge->getStep() == NBEdge::EdgeBuildingStep::LANES2LANES_DONE || incomingEdge->getStep() == NBEdge::EdgeBuildingStep::LANES2LANES_USER) {
        return;
    }
    if (myAvailableLanes.size() == 0) {
        return;
    }
    std::vector<int> approachingLanes = incomingEdge->getConnectionLanes(myCurrentOutgoing, myIsBikeEdge || incomingEdge->getPermissions() == SVC_BICYCLE);
    if (approachingLanes.size() == 0) {
        return;
    }
#ifdef DEBUG_CONNECTION_GUESSING
    if (DEBUGCOND2(incomingEdge->getToNode())) {
        std::cout << "Bre:ex src=" << src << " dest=" << dest << " in=" << incomingEdge->getID() << " apLanes=" << toString(approachingLanes) << "\n";
    }

#endif
    int numConnections = (int)approachingLanes.size();
    double factor = 1;
    const bool rightOnRed = incomingEdge->getToNode()->getType() == SumoXMLNodeType::TRAFFIC_LIGHT_RIGHT_ON_RED;
    if (myNumStraight == 1 && myDirections[src] == LinkDirection::STRAIGHT && (
                // we do not want to destroy ramp-like assignments where the
                // on-connection-per-lane rule avoids conflicts
                // - at a traffic light the phases are seperated so there is no conflict anyway
                (incomingEdge->getToNode()->isTLControlled() && !rightOnRed)
                // - there are no incoming edges to the right
                || src == 0
                // - a minor straight road is likely in conflict anyway
                || (incomingEdge->getJunctionPriority(incomingEdge->getToNode()) == NBEdge::MINOR_ROAD && !rightOnRed))) {
        numConnections = (int)myAvailableLanes.size();
        factor = (double)approachingLanes.size() / (double)numConnections;
        if (factor > 0.5) {
            factor = 1;
        }
    }
    std::deque<int>* approachedLanes = spread(numConnections, dest);
    assert(approachedLanes->size() <= myAvailableLanes.size());
    // set lanes
    const int maxFrom = (int)approachingLanes.size() - 1;
    for (int i = 0; i < (int)approachedLanes->size(); i++) {
        // distribute i evenly on approaching lanes in case we are building more
        // connections than there are lanes
        int fromLane = approachingLanes[MIN2((int)(i * factor), maxFrom)];
        int approached = myAvailableLanes[(*approachedLanes)[i]];
        incomingEdge->setConnection(fromLane, myCurrentOutgoing, approached, NBEdge::Lane2LaneInfoType::COMPUTED);
    }
    delete approachedLanes;
}


std::deque<int>*
NBNode::ApproachingDivider::spread(int numLanes, int dest) const {
    std::deque<int>* ret = new std::deque<int>();
    // when only one lane is approached, we check, whether the double-value
    //  is assigned more to the left or right lane
    if (numLanes == 1) {
        ret->push_back(dest);
        return ret;
    }

    const int numOutgoingLanes = (int)myAvailableLanes.size();
    //
    ret->push_back(dest);
    int noSet = 1;
    int roffset = 1;
    int loffset = 1;
    while (noSet < numLanes) {
        // It may be possible, that there are not enough lanes the source
        //  lanes may be divided on
        //  In this case, they remain unset
        //  !!! this is only a hack. It is possible, that this yields in
        //   uncommon divisions
        if (numOutgoingLanes == noSet) {
            return ret;
        }

        // as due to the conversion of double->uint the numbers will be lower
        //  than they should be, we try to append to the left side first
        //
        // check whether the left boundary of the approached street has
        //  been overridden; if so, move all lanes to the right
        if (dest + loffset >= numOutgoingLanes) {
            loffset -= 1;
            roffset += 1;
            for (int i = 0; i < (int)ret->size(); i++) {
                (*ret)[i] = (*ret)[i] - 1;
            }
        }
        // append the next lane to the left of all edges
        //  increase the position (destination edge)
        ret->push_back(dest + loffset);
        noSet++;
        loffset += 1;

        // as above
        if (numOutgoingLanes == noSet) {
            return ret;
        }

        // now we try to append the next lane to the right side, when needed
        if (noSet < numLanes) {
            // check whether the right boundary of the approached street has
            //  been overridden; if so, move all lanes to the right
            if (dest < roffset) {
                loffset += 1;
                roffset -= 1;
                for (int i = 0; i < (int)ret->size(); i++) {
                    (*ret)[i] = (*ret)[i] + 1;
                }
            }
            ret->push_front(dest - roffset);
            noSet++;
            roffset += 1;
        }
    }
    return ret;
}


/* -------------------------------------------------------------------------
 * NBNode::Crossing-methods
 * ----------------------------------------------------------------------- */
NBNode::Crossing::Crossing(const NBNode* _node, const EdgeVector& _edges, double _width, bool _priority, int _customTLIndex, int _customTLIndex2, const PositionVector& _customShape) :
    Parameterised(),
    node(_node),
    edges(_edges),
    customWidth(_width),
    width(_width),
    priority(_priority),
    customShape(_customShape),
    tlLinkIndex(_customTLIndex),
    tlLinkIndex2(_customTLIndex2),
    customTLIndex(_customTLIndex),
    customTLIndex2(_customTLIndex2),
    valid(true) {
}


/* -------------------------------------------------------------------------
 * NBNode-methods
 * ----------------------------------------------------------------------- */
NBNode::NBNode(const std::string& id, const Position& position,
               SumoXMLNodeType type) :
    Named(StringUtils::convertUmlaute(id)),
    myPosition(position),
    myType(type),
    myDistrict(nullptr),
    myHaveCustomPoly(false),
    myRequest(nullptr),
    myRadius(UNSPECIFIED_RADIUS),
    myKeepClear(OptionsCont::getOptions().getBool("default.junctions.keep-clear")),
    myRightOfWay(SUMOXMLDefinitions::RightOfWayValues.get(OptionsCont::getOptions().getString("default.right-of-way"))),
    myFringeType(FringeType::DEFAULT),
    myRoundaboutType(RoundaboutType::DEFAULT),
    myDiscardAllCrossings(false),
    myCrossingsLoadedFromSumoNet(0),
    myDisplacementError(0),
    myIsBentPriority(false),
    myTypeWasGuessed(false) {
    if (!SUMOXMLDefinitions::isValidNetID(myID)) {
        throw ProcessError(TLF("Invalid node id '%'.", myID));
    }
    if (myPosition.isNAN()) {
        throw ProcessError(TLF("Invalid position '%' for node '%'", myPosition, myID));
    }
}


NBNode::NBNode(const std::string& id, const Position& position, NBDistrict* district) :
    Named(StringUtils::convertUmlaute(id)),
    myPosition(position),
    myType(district == nullptr ? SumoXMLNodeType::UNKNOWN : SumoXMLNodeType::DISTRICT),
    myDistrict(district),
    myHaveCustomPoly(false),
    myRequest(nullptr),
    myRadius(UNSPECIFIED_RADIUS),
    myKeepClear(OptionsCont::getOptions().getBool("default.junctions.keep-clear")),
    myRightOfWay(SUMOXMLDefinitions::RightOfWayValues.get(OptionsCont::getOptions().getString("default.right-of-way"))),
    myFringeType(FringeType::DEFAULT),
    myRoundaboutType(RoundaboutType::DEFAULT),
    myDiscardAllCrossings(false),
    myCrossingsLoadedFromSumoNet(0),
    myDisplacementError(0),
    myIsBentPriority(false),
    myTypeWasGuessed(false) {
    if (!SUMOXMLDefinitions::isValidNetID(myID)) {
        throw ProcessError(TLF("Invalid node id '%'.", myID));
    }
    if (myPosition.isNAN()) {
        throw ProcessError(TLF("Invalid position '%' for node '%'", myPosition, myID));
    }
}


NBNode::~NBNode() {
    delete myRequest;
}


void
NBNode::reinit(const Position& position, SumoXMLNodeType type,
               bool updateEdgeGeometries) {
    myPosition = position;
    if (myPosition.isNAN()) {
        throw ProcessError(TLF("Invalid position '%' for node '%'", myPosition, myID));
    }
    // patch type
    myType = type;
    if (!isTrafficLight(myType)) {
        removeTrafficLights();
    }
    if (updateEdgeGeometries) {
        for (EdgeVector::iterator i = myIncomingEdges.begin(); i != myIncomingEdges.end(); i++) {
            PositionVector geom = (*i)->getGeometry();
            geom[-1] = myPosition;
            (*i)->setGeometry(geom);
        }
        for (EdgeVector::iterator i = myOutgoingEdges.begin(); i != myOutgoingEdges.end(); i++) {
            PositionVector geom = (*i)->getGeometry();
            geom[0] = myPosition;
            (*i)->setGeometry(geom);
        }
    }
}



// -----------  Applying offset
void
NBNode::reshiftPosition(double xoff, double yoff) {
    myPosition.add(xoff, yoff, 0);
    myPoly.add(xoff, yoff, 0);
    for (auto& wacs : myWalkingAreaCustomShapes) {
        wacs.shape.add(xoff, yoff, 0);
    }
    for (auto& c : myCrossings) {
        c->customShape.add(xoff, yoff, 0);
    }
}


void
NBNode::roundGeometry() {
    myPosition.round(gPrecision);
    if (myHaveCustomPoly) {
        myPoly.round(gPrecision);
    }
    for (auto& wacs : myWalkingAreaCustomShapes) {
        wacs.shape.round(gPrecision);
    }
    for (auto& c : myCrossings) {
        c->customShape.round(gPrecision);
    }
}


void
NBNode::mirrorX() {
    myPosition.mul(1, -1);
    myPoly.mirrorX();
    // mirror pre-computed geometry of crossings and walkingareas
    for (auto& c : myCrossings) {
        c->customShape.mirrorX();
        c->shape.mirrorX();
    }
    for (auto& wa : myWalkingAreas) {
        wa.shape.mirrorX();
    }
    for (auto& wacs : myWalkingAreaCustomShapes) {
        wacs.shape.mirrorX();
    }
}


// -----------  Methods for dealing with assigned traffic lights
void
NBNode::addTrafficLight(NBTrafficLightDefinition* tlDef) {
    myTrafficLights.insert(tlDef);
    // rail signals receive a temporary traffic light in order to set connection tl-linkIndex
    if (!isTrafficLight(myType) && myType != SumoXMLNodeType::RAIL_SIGNAL && myType != SumoXMLNodeType::RAIL_CROSSING) {
        myType = SumoXMLNodeType::TRAFFIC_LIGHT;
    }
}


void
NBNode::removeTrafficLight(NBTrafficLightDefinition* tlDef) {
    tlDef->removeNode(this);
    myTrafficLights.erase(tlDef);
}


void
NBNode::removeTrafficLights(bool setAsPriority) {
    std::set<NBTrafficLightDefinition*> trafficLights = myTrafficLights; // make a copy because we will modify the original
    for (std::set<NBTrafficLightDefinition*>::const_iterator i = trafficLights.begin(); i != trafficLights.end(); ++i) {
        removeTrafficLight(*i);
    }
    if (setAsPriority) {
        myType = myRequest != nullptr ? SumoXMLNodeType::PRIORITY : (
                     myType == SumoXMLNodeType::TRAFFIC_LIGHT_NOJUNCTION ? SumoXMLNodeType::NOJUNCTION : SumoXMLNodeType::DEAD_END);
    }
}

bool
NBNode::hadSignal() const {
    for (NBEdge* e : getIncomingEdges()) {
        if (e->getSignalPosition() != Position::INVALID) {
            return true;
        }
    }
    return false;
}


void
NBNode::invalidateTLS(NBTrafficLightLogicCont& tlCont, bool addedConnections, bool removedConnections) {
    if (isTLControlled()) {
        std::set<NBTrafficLightDefinition*> oldDefs(myTrafficLights);
        for (std::set<NBTrafficLightDefinition*>::iterator it = oldDefs.begin(); it != oldDefs.end(); ++it) {
            NBTrafficLightDefinition* orig = *it;
            if (dynamic_cast<NBLoadedSUMOTLDef*>(orig) != nullptr) {
                dynamic_cast<NBLoadedSUMOTLDef*>(orig)->registerModifications(addedConnections, removedConnections);
            } else if (dynamic_cast<NBOwnTLDef*>(orig) == nullptr) {
                NBTrafficLightDefinition* newDef = new NBOwnTLDef(orig->getID(), orig->getOffset(), orig->getType());
                const std::vector<NBNode*>& nodes = orig->getNodes();
                while (!nodes.empty()) {
                    newDef->addNode(nodes.front());
                    nodes.front()->removeTrafficLight(orig);
                }
                tlCont.removeFully(orig->getID());
                tlCont.insert(newDef);
            }
        }
    }
}


void
NBNode::shiftTLConnectionLaneIndex(NBEdge* edge, int offset, int threshold) {
    for (std::set<NBTrafficLightDefinition*>::iterator it = myTrafficLights.begin(); it != myTrafficLights.end(); ++it) {
        (*it)->shiftTLConnectionLaneIndex(edge, offset, threshold);
    }
}

// ----------- Prunning the input
int
NBNode::removeSelfLoops(NBDistrictCont& dc, NBEdgeCont& ec, NBTrafficLightLogicCont& tc) {
    int ret = 0;
    int pos = 0;
    EdgeVector::const_iterator j = myIncomingEdges.begin();
    while (j != myIncomingEdges.end()) {
        // skip edges which are only incoming and not outgoing
        if (find(myOutgoingEdges.begin(), myOutgoingEdges.end(), *j) == myOutgoingEdges.end()) {
            ++j;
            ++pos;
            continue;
        }
        // an edge with both its origin and destination being the current
        //  node should be removed
        NBEdge* dummy = *j;
        WRITE_WARNINGF(TL(" Removing self-looping edge '%'"), dummy->getID());
        // get the list of incoming edges connected to the self-loop
        EdgeVector incomingConnected = dummy->getIncomingEdges();
        // get the list of outgoing edges connected to the self-loop
        EdgeVector outgoingConnected = dummy->getConnectedEdges();
        // let the self-loop remap its connections
        dummy->remapConnections(incomingConnected);
        remapRemoved(tc, dummy, incomingConnected, outgoingConnected);
        // delete the self-loop
        ec.erase(dc, dummy);
        j = myIncomingEdges.begin() + pos;
        ++ret;
    }
    return ret;
}


// -----------
void
NBNode::addIncomingEdge(NBEdge* edge) {
    assert(edge != 0);
    if (find(myIncomingEdges.begin(), myIncomingEdges.end(), edge) == myIncomingEdges.end()) {
        myIncomingEdges.push_back(edge);
        myAllEdges.push_back(edge);
    }
}


void
NBNode::addOutgoingEdge(NBEdge* edge) {
    assert(edge != 0);
    if (find(myOutgoingEdges.begin(), myOutgoingEdges.end(), edge) == myOutgoingEdges.end()) {
        myOutgoingEdges.push_back(edge);
        myAllEdges.push_back(edge);
    }
}


bool
NBNode::isSimpleContinuation(bool checkLaneNumbers, bool checkWidth) const {
    // one in, one out->continuation
    if (myIncomingEdges.size() == 1 && myOutgoingEdges.size() == 1) {
        NBEdge* in = myIncomingEdges.front();
        NBEdge* out = myOutgoingEdges.front();
        // both must have the same number of lanes
        return ((!checkLaneNumbers || in->getNumLanes() == out->getNumLanes())
                && (!checkWidth || in->getTotalWidth() == out->getTotalWidth()));
    }
    // two in and two out and both in reverse direction
    if (myIncomingEdges.size() == 2 && myOutgoingEdges.size() == 2) {
        for (EdgeVector::const_iterator i = myIncomingEdges.begin(); i != myIncomingEdges.end(); i++) {
            NBEdge* in = *i;
            EdgeVector::const_iterator opposite = find_if(myOutgoingEdges.begin(), myOutgoingEdges.end(), NBContHelper::opposite_finder(in));
            // must have an opposite edge
            if (opposite == myOutgoingEdges.end()) {
                return false;
            }
            // both must have the same number of lanes
            NBContHelper::nextCW(myOutgoingEdges, opposite);
            if (checkLaneNumbers && in->getNumLanes() != (*opposite)->getNumLanes()) {
                return false;
            }
            if (checkWidth && in->getTotalWidth() != (*opposite)->getTotalWidth()) {
                return false;
            }
        }
        return true;
    }
    // nope
    return false;
}


PositionVector
NBNode::computeSmoothShape(const PositionVector& begShape,
                           const PositionVector& endShape,
                           int numPoints,
                           bool isTurnaround,
                           double extrapolateBeg,
                           double extrapolateEnd,
                           NBNode* recordError,
                           int shapeFlag) const {

    bool ok = true;
    if ((shapeFlag & INDIRECT_LEFT) != 0) {
        return indirectLeftShape(begShape, endShape, numPoints);
    }
    PositionVector init = bezierControlPoints(begShape, endShape, isTurnaround, extrapolateBeg, extrapolateEnd, ok, recordError, DEG2RAD(5), shapeFlag);
#ifdef DEBUG_SMOOTH_GEOM
    if (DEBUGCOND) {
        std::cout << "computeSmoothShape node " << getID() << " begShape=" << begShape << " endShape=" << endShape << " init=" << init << " shapeFlag=" << shapeFlag << "\n";
    }
#endif
    if (init.size() == 0) {
        PositionVector ret;
        ret.push_back(begShape.back());
        ret.push_back(endShape.front());
        return ret;
    } else {
        return init.bezier(numPoints).smoothedZFront();
    }
}

PositionVector
NBNode::bezierControlPoints(
    const PositionVector& begShape,
    const PositionVector& endShape,
    bool isTurnaround,
    double extrapolateBeg,
    double extrapolateEnd,
    bool& ok,
    NBNode* recordError,
    double straightThresh,
    int shapeFlag) {

    const Position beg = begShape.back();
    const Position end = endShape.front();
    const double dist = beg.distanceTo2D(end);
    PositionVector init;
    if (dist < POSITION_EPS || beg.distanceTo2D(begShape[-2]) < POSITION_EPS || end.distanceTo2D(endShape[1]) < POSITION_EPS) {
#ifdef DEBUG_SMOOTH_GEOM
        if (DEBUGCOND2(recordError)) std::cout << "   bezierControlPoints failed beg=" << beg << " end=" << end
                                                   << " dist=" << dist
                                                   << " distBegLast=" << beg.distanceTo2D(begShape[-2])
                                                   << " distEndFirst=" << end.distanceTo2D(endShape[1])
                                                   << "\n";
#endif
        // typically, this node a is a simpleContinuation. see also #2539
        return init;
    } else {
        init.push_back(beg);
        if (isTurnaround) {
            // turnarounds:
            //  - end of incoming lane
            //  - position between incoming/outgoing end/begin shifted by the distance orthogonally
            //  - begin of outgoing lane
            Position center = PositionVector::positionAtOffset2D(beg, end, beg.distanceTo2D(end) / (double) 2.);
            center.sub(beg.y() - end.y(), end.x() - beg.x());
            init.push_back(center);
        } else {
            const double EXT = 100;
            const double angle = GeomHelper::angleDiff(begShape.angleAt2D(-2), endShape.angleAt2D(0));
            PositionVector endShapeBegLine(endShape[0], endShape[1]);
            PositionVector begShapeEndLineRev(begShape[-1], begShape[-2]);
            endShapeBegLine.extrapolate2D(EXT, true);
            begShapeEndLineRev.extrapolate2D(EXT, true);
#ifdef DEBUG_SMOOTH_GEOM
            if (DEBUGCOND2(recordError)) std::cout
                        << "   endShapeBegLine=" << endShapeBegLine
                        << " begShapeEndLineRev=" << begShapeEndLineRev
                        << " angle=" << RAD2DEG(angle) << "\n";
#endif
            if (fabs(angle) < M_PI / 4.) {
                // very low angle: could be an s-shape or a straight line
                const double displacementAngle = GeomHelper::angleDiff(begShape.angleAt2D(-2), beg.angleTo2D(end));
                const double bendDeg = RAD2DEG(fabs(displacementAngle - angle));
                const double halfDistance = dist / 2;
                if (fabs(displacementAngle) <= straightThresh && fabs(angle) <= straightThresh) {
#ifdef DEBUG_SMOOTH_GEOM
                    if (DEBUGCOND2(recordError)) std::cout << "   bezierControlPoints identified straight line beg=" << beg << " end=" << end
                                                               << " angle=" << RAD2DEG(angle) << " displacementAngle=" << RAD2DEG(displacementAngle) << "\n";
#endif
                    return PositionVector();
                } else if (bendDeg > 22.5 && pow(bendDeg / 45, 2) / dist > 0.13) {
                    // do not allow s-curves with extreme bends
                    // (a linear dependency is to restrictive at low displacementAngles and too permisive at high angles)
#ifdef DEBUG_SMOOTH_GEOM
                    if (DEBUGCOND2(recordError)) std::cout << "   bezierControlPoints found extreme s-curve, falling back to straight line beg=" << beg << " end=" << end
                                                               << " angle=" << RAD2DEG(angle) << " displacementAngle=" << RAD2DEG(displacementAngle)
                                                               << " dist=" << dist << " bendDeg=" << bendDeg << " bd2=" << pow(bendDeg / 45, 2)
                                                               << " displacementError=" << sin(displacementAngle) * dist
                                                               << " begShape=" << begShape << " endShape=" << endShape << "\n";
#endif
                    ok = false;
                    if (recordError != nullptr && (shapeFlag & SCURVE_IGNORE) == 0) {
                        recordError->myDisplacementError = MAX2(recordError->myDisplacementError, (double)fabs(sin(displacementAngle) * dist));
                    }
                    return PositionVector();
                } else {
                    const double endLength = begShape[-2].distanceTo2D(begShape[-1]);
                    const double off1 = endLength + MIN2(extrapolateBeg, halfDistance);
                    init.push_back(PositionVector::positionAtOffset2D(begShapeEndLineRev[1], begShapeEndLineRev[0], off1));
                    const double off2 = EXT - MIN2(extrapolateEnd, halfDistance);
                    init.push_back(PositionVector::positionAtOffset2D(endShapeBegLine[0], endShapeBegLine[1], off2));
#ifdef DEBUG_SMOOTH_GEOM
                    if (DEBUGCOND2(recordError)) std::cout << "   bezierControlPoints found s-curve beg=" << beg << " end=" << end
                                                               << " angle=" << RAD2DEG(angle) << " displacementAngle=" << RAD2DEG(displacementAngle)
                                                               << " halfDistance=" << halfDistance << "\n";
#endif
                }
            } else {
                // turning
                //  - end of incoming lane
                //  - intersection of the extrapolated lanes
                //  - begin of outgoing lane
                // attention: if there is no intersection, use a straight line
                Position intersect = endShapeBegLine.intersectionPosition2D(begShapeEndLineRev);
                if (intersect == Position::INVALID) {
#ifdef DEBUG_SMOOTH_GEOM
                    if (DEBUGCOND2(recordError)) {
                        std::cout << "   bezierControlPoints failed beg=" << beg << " end=" << end << " intersect=" << intersect
                                  << " endShapeBegLine=" << endShapeBegLine
                                  << " begShapeEndLineRev=" << begShapeEndLineRev
                                  << "\n";
                    }
#endif
                    ok = false;
                    if (recordError != nullptr && (shapeFlag & SCURVE_IGNORE) == 0) {
                        // it's unclear if this error can be solved via stretching the intersection.
                        recordError->myDisplacementError = MAX2(recordError->myDisplacementError, (double)1.0);
                    }
                    return PositionVector();
                }
                const double begOffset = begShapeEndLineRev.nearest_offset_to_point2D(intersect);
                const double endOffset = endShapeBegLine.nearest_offset_to_point2D(intersect);
                /*
                if ((shapeFlag & FOUR_CONTROL_POINTS) == 0 && (begOffset >= EXT || endOffset >= EXT)) {
                    // intersection point lies within begShape / endShape so we cannot use it
                    if (dist < 2) {
                        return PositionVector();
                    }
                    shapeFlag |= FOUR_CONTROL_POINTS;
                    extrapolateBeg = MIN2(10.0, dist / 2);
                    extrapolateEnd = extrapolateBeg;
                }
                */
                const double minControlLength = MIN2((double)1.0, dist / 2);
                const double distBeg = intersect.distanceTo2D(beg);
                const double distEnd = intersect.distanceTo2D(end);
                const bool lengthenBeg = distBeg <= minControlLength;
                const bool lengthenEnd = distEnd <= minControlLength;
#ifdef DEBUG_SMOOTH_GEOM
                if (DEBUGCOND2(recordError)) std::cout
                            << "   beg=" << beg << " end=" << end << " intersect=" << intersect
                            << " distBeg=" << distBeg << " distEnd=" << distEnd
                            << " begOffset=" << begOffset << " endOffset=" << endOffset
                            << " lEnd=" << lengthenEnd << " lBeg=" << lengthenBeg
                            << "\n";
#endif
                if (lengthenBeg && lengthenEnd) {
#ifdef DEBUG_SMOOTH_GEOM
                    if (DEBUGCOND2(recordError)) {
                        std::cout << "   bezierControlPoints failed\n";
                    }
#endif
                    if (recordError != nullptr && (shapeFlag & SCURVE_IGNORE) == 0) {
                        // This should be fixable with minor stretching
                        recordError->myDisplacementError = MAX2(recordError->myDisplacementError, (double)1.0);
                    }
                    ok = false;
                    return PositionVector();
                } else if ((shapeFlag & FOUR_CONTROL_POINTS)) {
                    init.push_back(begShapeEndLineRev.positionAtOffset2D(EXT - extrapolateBeg));
                    init.push_back(endShapeBegLine.positionAtOffset2D(EXT - extrapolateEnd));
                } else if (lengthenBeg || lengthenEnd) {
                    init.push_back(begShapeEndLineRev.positionAtOffset2D(EXT - minControlLength));
                    init.push_back(endShapeBegLine.positionAtOffset2D(EXT - minControlLength));
                } else if ((shapeFlag & AVOID_WIDE_LEFT_TURN) != 0
                           // there are two reasons for enabling special geometry rules:
                           // 1) sharp edge angles which could cause overshoot
                           // 2) junction geometries with a large displacement between opposite left turns
                           //    which would cause the default geometry to overlap
                           && ((shapeFlag & AVOID_INTERSECTING_LEFT_TURNS) != 0
                               || (angle > DEG2RAD(95) && (distBeg > 20 || distEnd > 20)))) {
                    //std::cout << "   bezierControlPoints intersect=" << intersect << " dist=" << dist << " distBeg=" << distBeg <<  " distEnd=" << distEnd << " angle=" << RAD2DEG(angle) << " flag=" << shapeFlag << "\n";
                    const double factor = ((shapeFlag & AVOID_INTERSECTING_LEFT_TURNS) == 0 ? 1
                                           : MIN2(0.6, 16 / dist));
                    init.push_back(begShapeEndLineRev.positionAtOffset2D(EXT - MIN2(distBeg * factor / 1.2, dist * factor / 1.8)));
                    init.push_back(endShapeBegLine.positionAtOffset2D(EXT - MIN2(distEnd * factor / 1.2, dist * factor / 1.8)));
                } else if ((shapeFlag & AVOID_WIDE_RIGHT_TURN) != 0 && angle < DEG2RAD(-95) && (distBeg > 20 || distEnd > 20)) {
                    //std::cout << "   bezierControlPoints intersect=" << intersect << " distBeg=" << distBeg <<  " distEnd=" << distEnd << "\n";
                    init.push_back(begShapeEndLineRev.positionAtOffset2D(EXT - MIN2(distBeg / 1.4, dist / 2)));
                    init.push_back(endShapeBegLine.positionAtOffset2D(EXT - MIN2(distEnd / 1.4, dist / 2)));
                } else {
                    double z;
                    const double z1 = begShapeEndLineRev.positionAtOffset2D(begOffset).z();
                    const double z2 = endShapeBegLine.positionAtOffset2D(endOffset).z();
                    const double z3 = 0.5 * (beg.z() + end.z());
                    // if z1 and z2 are on the same side in regard to z3 then we
                    // can use their avarage. Otherwise, the intersection in 3D
                    // is not good and we are better of using z3
                    if ((z1 <= z3 && z2 <= z3) || (z1 >= z3 && z2 >= z3)) {
                        z = 0.5 * (z1 + z2);
                    } else {
                        z = z3;
                    }
                    intersect.set(intersect.x(), intersect.y(), z);
                    init.push_back(intersect);
                }
            }
        }
        init.push_back(end);
    }
    return init;
}

PositionVector
NBNode::indirectLeftShape(const PositionVector& begShape, const PositionVector& endShape, int numPoints) const {
    UNUSED_PARAMETER(numPoints);
    PositionVector result;
    result.push_back(begShape.back());
    //const double angle = GeomHelper::angleDiff(begShape.angleAt2D(-2), endShape.angleAt2D(0));
    PositionVector endShapeBegLine(endShape[0], endShape[1]);
    PositionVector begShapeEndLineRev(begShape[-1], begShape[-2]);
    endShapeBegLine.extrapolate2D(100, true);
    begShapeEndLineRev.extrapolate2D(100, true);
    Position intersect = endShapeBegLine.intersectionPosition2D(begShapeEndLineRev);
    if (intersect == Position::INVALID) {
        WRITE_WARNINGF(TL("Could not compute indirect left turn shape at node '%'"), getID());
    } else {
        Position dir = intersect;
        dir.sub(endShape[0]);
        dir.norm2D();
        const double radius = myRadius == NBNode::UNSPECIFIED_RADIUS ? OptionsCont::getOptions().getFloat("default.junctions.radius") : myRadius;
        dir.mul(radius);
        result.push_back(intersect + dir);
    }
    result.push_back(endShape.front());
    return result;
}

PositionVector
NBNode::computeInternalLaneShape(const NBEdge* fromE, const NBEdge::Connection& con, int numPoints, NBNode* recordError, int shapeFlag) const {
    if (con.fromLane >= fromE->getNumLanes()) {
        throw ProcessError(TLF("Connection '%' starts at a non-existant lane.", con.getDescription(fromE)));
    }
    if (con.toLane >= con.toEdge->getNumLanes()) {
        throw ProcessError(TLF("Connection '%' targets a non-existant lane.", con.getDescription(fromE)));
    }
    PositionVector fromShape = fromE->getLaneShape(con.fromLane);
    PositionVector toShape = con.toEdge->getLaneShape(con.toLane);
    PositionVector ret;
    bool useCustomShape = con.customShape.size() > 0;
    if (useCustomShape) {
        // ensure that the shape starts and ends at the intersection boundary
        PositionVector startBorder = fromE->getNodeBorder(this);
        if (startBorder.size() == 0) {
            startBorder = fromShape.getOrthogonal(fromShape.back(), 1, true);
        }
        PositionVector tmp = NBEdge::startShapeAt(con.customShape, this, startBorder);
        if (tmp.size() < 2) {
            WRITE_WARNINGF(TL("Could not use custom shape for connection %."), con.getDescription(fromE));
            useCustomShape = false;
        } else {
            if (tmp.length2D() > con.customShape.length2D() + POSITION_EPS) {
                // shape was lengthened at the start, make sure it attaches at the center of the lane
                tmp[0] = fromShape.back();
            } else if (recordError != nullptr) {
                const double offset = tmp[0].distanceTo2D(fromShape.back());
                if (offset > fromE->getLaneWidth(con.fromLane) / 2) {
                    WRITE_WARNINGF(TL("Custom shape has distance % to incoming lane for connection %."), offset, con.getDescription(fromE));
                }
            }
            PositionVector endBorder = con.toEdge->getNodeBorder(this);
            if (endBorder.size() == 0) {
                endBorder = toShape.getOrthogonal(toShape.front(), 1, false);
            }
            ret = NBEdge::startShapeAt(tmp.reverse(), this, endBorder).reverse();
            if (ret.size() < 2) {
                WRITE_WARNINGF(TL("Could not use custom shape for connection %."), con.getDescription(fromE));
                useCustomShape = false;
            } else if (ret.length2D() > tmp.length2D() + POSITION_EPS) {
                // shape was lengthened at the end, make sure it attaches at the center of the lane
                ret[-1] = toShape.front();
            } else if (recordError != nullptr) {
                const double offset = ret[-1].distanceTo2D(toShape.front());
                if (offset > con.toEdge->getLaneWidth(con.toLane) / 2) {
                    WRITE_WARNINGF(TL("Custom shape has distance % to outgoing lane for connection %."), offset, con.getDescription(fromE));
                }
            }
        }
    }
    if (!useCustomShape) {
        displaceShapeAtWidthChange(fromE, con, fromShape, toShape);
        double extrapolateBeg = 5. * fromE->getNumLanes();
        double extrapolateEnd = 5. * con.toEdge->getNumLanes();
        LinkDirection dir = getDirection(fromE, con.toEdge);
        if (dir == LinkDirection::LEFT || dir == LinkDirection::TURN) {
            shapeFlag += AVOID_WIDE_LEFT_TURN;
        }
        if (con.indirectLeft) {
            shapeFlag += INDIRECT_LEFT;
        }
#ifdef DEBUG_SMOOTH_GEOM
        if (DEBUGCOND) {
            std::cout << "computeInternalLaneShape node " << getID() << " fromE=" << fromE->getID() << " toE=" << con.toEdge->getID() << "\n";
        }
#endif
        ret = computeSmoothShape(fromShape, toShape,
                                 numPoints, fromE->getTurnDestination() == con.toEdge,
                                 extrapolateBeg, extrapolateEnd, recordError, shapeFlag);
    }
    const NBEdge::Lane& lane = fromE->getLaneStruct(con.fromLane);
    if (lane.endOffset > 0) {
        PositionVector beg = lane.shape.getSubpart(lane.shape.length() - lane.endOffset, lane.shape.length());
        beg.append(ret);
        ret = beg;
    }
    if (con.toEdge->isBidiRail() && con.toEdge->getTurnDestination(true)->getEndOffset() > 0) {
        PositionVector end = toShape.getSubpart(0, con.toEdge->getTurnDestination(true)->getEndOffset());
        ret.append(end);
    }
    return ret;
}


bool
NBNode::isConstantWidthTransition() const {
    return (myIncomingEdges.size() == 1
            && myOutgoingEdges.size() == 1
            && myIncomingEdges[0]->getNumLanes() != myOutgoingEdges[0]->getNumLanes()
            && myIncomingEdges[0]->getTotalWidth() == myOutgoingEdges[0]->getTotalWidth());
}

void
NBNode::displaceShapeAtWidthChange(const NBEdge* from, const NBEdge::Connection& con,
                                   PositionVector& fromShape, PositionVector& toShape) const {
    if (isConstantWidthTransition()) {
        // displace shapes
        NBEdge* in = myIncomingEdges[0];
        NBEdge* out = myOutgoingEdges[0];
        double outCenter = out->getLaneWidth(con.toLane) / 2;
        for (int i = 0; i < con.toLane; ++i) {
            outCenter += out->getLaneWidth(i);
        }
        double inCenter = in->getLaneWidth(con.fromLane) / 2;
        for (int i = 0; i < con.fromLane; ++i) {
            inCenter += in->getLaneWidth(i);
        }
        //std::cout << "displaceShapeAtWidthChange inCenter=" << inCenter << " outCenter=" << outCenter << "\n";
        try {
            if (in->getNumLanes() > out->getNumLanes()) {
                // shift toShape so the internal lane ends straight at the displaced entry point
                toShape.move2side(outCenter - inCenter);
            } else {
                // shift fromShape so the internal lane starts straight at the displaced exit point
                fromShape.move2side(inCenter - outCenter);

            }
        } catch (InvalidArgument&) { }
    } else {
        SVCPermissions fromP = from->getPermissions(con.fromLane);
        SVCPermissions toP = con.toEdge->getPermissions(con.toLane);
        if ((fromP & toP) == SVC_BICYCLE && (fromP | toP) != SVC_BICYCLE) {
            double shift = (from->getLaneWidth(con.fromLane) - con.toEdge->getLaneWidth(con.toLane)) / 2;
            if (toP == SVC_BICYCLE) {
                // let connection to dedicated bicycle lane start on the right side of a mixed lane for straight an right-going connections
                // (on the left side for left turns)
                // XXX indirect left turns should also start on the right side
                LinkDirection dir = getDirection(from, con.toEdge);
                if ((dir == LinkDirection::LEFT) || (dir == LinkDirection::PARTLEFT) || (dir == LinkDirection::TURN)) {
                    fromShape.move2side(-shift);
                } else {
                    fromShape.move2side(shift);
                }
            } else if (fromP == SVC_BICYCLE) {
                // let connection from dedicated bicycle end on the right side of a mixed lane
                toShape.move2side(-shift);
            }
        }
    }
}

bool
NBNode::needsCont(const NBEdge* fromE, const NBEdge* otherFromE,
                  const NBEdge::Connection& c, const NBEdge::Connection& otherC, bool checkOnlyTLS) const {
    const NBEdge* toE = c.toEdge;
    const NBEdge* otherToE = otherC.toEdge;

    if (!checkOnlyTLS) {
        if (myType == SumoXMLNodeType::RIGHT_BEFORE_LEFT
                || myType == SumoXMLNodeType::LEFT_BEFORE_RIGHT
                || myType == SumoXMLNodeType::ALLWAY_STOP
                || myType == SumoXMLNodeType::ZIPPER) {
            return false;
        }
        LinkDirection d1 = getDirection(fromE, toE);
        const bool thisRight = (d1 == LinkDirection::RIGHT || d1 == LinkDirection::PARTRIGHT);
        const bool rightTurnConflict = (thisRight &&
                                        NBNode::rightTurnConflict(fromE, toE, c.fromLane, otherFromE, otherToE, otherC.fromLane));
        if (thisRight && !rightTurnConflict) {
            return false;
        }
        if (myRequest && myRequest->indirectLeftTurnConflict(fromE, c, otherFromE, otherC, false)) {
            return true;
        }
        if (!(foes(otherFromE, otherToE, fromE, toE) || myRequest == nullptr || rightTurnConflict)) {
            // if they do not cross, no waiting place is needed
            return false;
        }
        LinkDirection d2 = getDirection(otherFromE, otherToE);
        if (d2 == LinkDirection::TURN) {
            return false;
        }
        if (fromE == otherFromE && !thisRight) {
            // ignore same edge links except for right-turns
            return false;
        }
        if (thisRight && d2 != LinkDirection::STRAIGHT) {
            return false;
        }
    }
    if (c.tlID != "") {
        assert(myTrafficLights.size() > 0 || myType == SumoXMLNodeType::RAIL_CROSSING || myType == SumoXMLNodeType::RAIL_SIGNAL);
        for (std::set<NBTrafficLightDefinition*>::const_iterator it = myTrafficLights.begin(); it != myTrafficLights.end(); ++it) {
            if ((*it)->needsCont(fromE, toE, otherFromE, otherToE)) {
                return true;
            }
        }
        return false;
    }
    if (fromE->getJunctionPriority(this) > 0 && otherFromE->getJunctionPriority(this) > 0) {
        return mustBrake(fromE, toE, c.fromLane, c.toLane, false);
    }
    return false;
}

bool
NBNode::tlsStrandedConflict(const NBEdge* from, const NBEdge::Connection& c,
                            const NBEdge* foeFrom, const NBEdge::Connection& foe) const {
    return (foe.haveVia && isTLControlled() && c.tlLinkIndex >= 0 && foe.tlLinkIndex >= 0
            && !foeFrom->isTurningDirectionAt(foe.toEdge)
            && foes(from, c.toEdge, foeFrom, foe.toEdge)
            && !needsCont(foeFrom, from, foe, c, true));
}


void
NBNode::removeJoinedTrafficLights() {
    std::set<NBTrafficLightDefinition*> trafficLights = myTrafficLights; // make a copy because we will modify the original
    for (std::set<NBTrafficLightDefinition*>::const_iterator i = trafficLights.begin(); i != trafficLights.end(); ++i) {
        // if this is the only controlled node we keep the tlDef as it is to generate a warning later
        if ((*i)->getNodes().size() > 1) {
            myTrafficLights.erase(*i);
            (*i)->removeNode(this);
            (*i)->setParticipantsInformation();
            (*i)->setTLControllingInformation();
        }
    }
}


void
NBNode::computeLogic(const NBEdgeCont& ec) {
    delete myRequest; // possibly recomputation step
    myRequest = nullptr;
    if (myIncomingEdges.size() == 0 || myOutgoingEdges.size() == 0) {
        // no logic if nothing happens here
        myType = SumoXMLNodeType::DEAD_END;
        removeJoinedTrafficLights();
        return;
    }
    // compute the logic if necessary or split the junction
    if (myType != SumoXMLNodeType::NOJUNCTION && myType != SumoXMLNodeType::DISTRICT && myType != SumoXMLNodeType::TRAFFIC_LIGHT_NOJUNCTION) {
        // build the request
        myRequest = new NBRequest(ec, this, myAllEdges, myIncomingEdges, myOutgoingEdges, myBlockedConnections);
        // check whether it is not too large
        int numConnections = numNormalConnections();
        if (numConnections >= SUMO_MAX_CONNECTIONS) {
            // yep -> make it untcontrolled, warn
            delete myRequest;
            myRequest = nullptr;
            if (myType == SumoXMLNodeType::TRAFFIC_LIGHT) {
                myType = SumoXMLNodeType::TRAFFIC_LIGHT_NOJUNCTION;
            } else {
                myType = SumoXMLNodeType::NOJUNCTION;
            }
            WRITE_WARNINGF(TL("Junction '%' is too complicated (% connections, max %); will be set to %."),
                           getID(), numConnections, SUMO_MAX_CONNECTIONS, toString(myType));
        } else if (numConnections == 0) {
            delete myRequest;
            myRequest = nullptr;
            myType = SumoXMLNodeType::DEAD_END;
            removeJoinedTrafficLights();
        } else {
            myRequest->buildBitfieldLogic();
        }
    }
}


void
NBNode::computeLogic2(bool checkLaneFoes) {
    if (myRequest != nullptr) {
        myRequest->computeLogic(checkLaneFoes);
    }
}

void
NBNode::computeKeepClear() {
    if (hasConflict()) {
        if (!myKeepClear) {
            for (NBEdge* incoming : myIncomingEdges) {
                std::vector<NBEdge::Connection>& connections = incoming->getConnections();
                for (NBEdge::Connection& c : connections) {
                    c.keepClear = KEEPCLEAR_FALSE;
                }
            }
        } else if (geometryLike() && myCrossings.size() == 0 && !isTLControlled()) {
            int linkIndex = 0;
            for (NBEdge* incoming : myIncomingEdges) {
                std::vector<NBEdge::Connection>& connections = incoming->getConnections();
                for (NBEdge::Connection& c : connections) {
                    if (c.keepClear == KEEPCLEAR_UNSPECIFIED && myRequest->hasConflictAtLink(linkIndex)) {
                        const LinkState linkState = getLinkState(incoming, c.toEdge, c.fromLane, c.toLane, c.mayDefinitelyPass, c.tlID);
                        if (linkState == LINKSTATE_MAJOR) {
                            c.keepClear = KEEPCLEAR_FALSE;
                        }
                    }
                }
                linkIndex++;
            }
        }
    }
}


bool
NBNode::writeLogic(OutputDevice& into) const {
    if (myRequest) {
        myRequest->writeLogic(into);
        return true;
    }
    return false;
}


const std::string
NBNode::getFoes(int linkIndex) const {
    if (myRequest == nullptr) {
        return "";
    } else {
        return myRequest->getFoes(linkIndex);
    }
}


const std::string
NBNode::getResponse(int linkIndex) const {
    if (myRequest == nullptr) {
        return "";
    } else {
        return myRequest->getResponse(linkIndex);
    }
}

bool
NBNode::hasConflict() const {
    if (myRequest == nullptr) {
        return false;
    } else {
        return myRequest->hasConflict();
    }
}


bool
NBNode::hasConflict(const NBEdge* e) const {
    if (myRequest == nullptr) {
        return false;
    }
    for (const auto& con : e->getConnections()) {
        const int index = getConnectionIndex(e, con);
        if (myRequest->hasConflictAtLink(index)) {
            return true;
        }
    }
    return false;
}


void
NBNode::updateSurroundingGeometry() {
    NBTurningDirectionsComputer::computeTurnDirectionsForNode(this, false);
    sortEdges(false);
    computeNodeShape(-1);
    for (NBEdge* edge : myAllEdges) {
        edge->computeEdgeShape();
    }
}

void
NBNode::computeNodeShape(double mismatchThreshold) {
    if (myHaveCustomPoly) {
        return;
    }
    if (myIncomingEdges.size() == 0 && myOutgoingEdges.size() == 0) {
        // may be an intermediate step during network editing
        myPoly.clear();
        myPoly.push_back(myPosition);
        return;
    }
    if (OptionsCont::getOptions().getFloat("default.junctions.radius") < 0) {
        // skip shape computation by option
        return;
    }
    try {
        NBNodeShapeComputer computer(*this);
        myPoly = computer.compute(OptionsCont::getOptions().getBool("junctions.minimal-shape"));
        if (myRadius == UNSPECIFIED_RADIUS && !OptionsCont::getOptions().isDefault("default.junctions.radius")) {
            myRadius = computer.getRadius();
        }
        if (myPoly.size() > 0) {
            PositionVector tmp = myPoly;
            tmp.push_back_noDoublePos(tmp[0]); // need closed shape
            if (mismatchThreshold >= 0
                    && !tmp.around(myPosition)
                    && tmp.distance2D(myPosition) > mismatchThreshold) {
                WRITE_WARNINGF(TL("Shape for junction '%' has distance % to its given position."), myID, tmp.distance2D(myPosition));
            }
        }
    } catch (InvalidArgument&) {
        WRITE_WARNINGF(TL("For junction '%': could not compute shape."), myID);
        // make sure our shape is not empty because our XML schema forbids empty attributes
        myPoly.clear();
        myPoly.push_back(myPosition);
    }
}


void
NBNode::computeLanes2Lanes() {
    // special case a):
    //  one in, one out, the outgoing has more lanes
    if (myIncomingEdges.size() == 1 && myOutgoingEdges.size() == 1) {
        NBEdge* in = myIncomingEdges[0];
        NBEdge* out = myOutgoingEdges[0];
        // check if it's not the turnaround
        if (in->getTurnDestination() == out) {
            // will be added later or not...
            return;
        }
        int inOffset, inEnd, outOffset, outEnd, addedLanes;
        getReduction(out, in, outOffset, outEnd, inOffset, inEnd, addedLanes);
        if (in->getStep() <= NBEdge::EdgeBuildingStep::LANES2EDGES
                && addedLanes > 0
                && in->isConnectedTo(out)) {
            const int addedRight = addedLanesRight(out, addedLanes);
            const int addedLeft = addedLanes - addedRight;
#ifdef DEBUG_CONNECTION_GUESSING
            if (DEBUGCOND) {
                std::cout << "l2l node=" << getID() << " specialCase a. addedRight=" << addedRight << " addedLeft=" << addedLeft << " inOff=" << inOffset << " outOff=" << outOffset << " inEnd=" << inEnd << " outEnd=" << outEnd << "\n";
            }
#endif
            // "straight" connections
            for (int i = inOffset; i < inEnd; ++i) {
                in->setConnection(i, out, i - inOffset + outOffset + addedRight, NBEdge::Lane2LaneInfoType::COMPUTED);
            }
            // connect extra lane on the right
            for (int i = 0; i < addedRight; ++i) {
                in->setConnection(inOffset, out, outOffset + i, NBEdge::Lane2LaneInfoType::COMPUTED);
            }
            // connect extra lane on the left
            const int inLeftMost = inEnd - 1;;
            const int outOffset2 = outOffset + addedRight + inEnd - inOffset;
            for (int i = 0; i < addedLeft; ++i) {
                in->setConnection(inLeftMost, out, outOffset2 + i, NBEdge::Lane2LaneInfoType::COMPUTED);
            }
            if (out->getSpecialLane(SVC_BICYCLE) >= 0) {
                recheckVClassConnections(out);
            }
            return;
        }
    }
    // special case b):
    //  two in, one out, the outgoing has the same number of lanes as the sum of the incoming
    //  --> highway on-ramp
    if (myIncomingEdges.size() == 2 && myOutgoingEdges.size() == 1) {
        NBEdge* const out = myOutgoingEdges[0];
        NBEdge* in1 = myIncomingEdges[0];
        NBEdge* in2 = myIncomingEdges[1];
        const int outOffset = MAX2(0, out->getFirstNonPedestrianNonBicycleLaneIndex(FORWARD, true));
        int in1Offset = MAX2(0, in1->getFirstNonPedestrianNonBicycleLaneIndex(FORWARD, true));
        int in2Offset = MAX2(0, in2->getFirstNonPedestrianNonBicycleLaneIndex(FORWARD, true));
        if (in1->getNumLanes() + in2->getNumLanes() - in1Offset - in2Offset == out->getNumLanes() - outOffset
                && (in1->getStep() <= NBEdge::EdgeBuildingStep::LANES2EDGES)
                && (in2->getStep() <= NBEdge::EdgeBuildingStep::LANES2EDGES)
                && in1 != out
                && in2 != out
                && in1->isConnectedTo(out)
                && in2->isConnectedTo(out)
                && in1->getSpecialLane(SVC_BICYCLE) == -1
                && in2->getSpecialLane(SVC_BICYCLE) == -1
                && out->getSpecialLane(SVC_BICYCLE) == -1
                && in1->getSpecialLane(SVC_TRAM) == -1
                && in2->getSpecialLane(SVC_TRAM) == -1
                && out->getSpecialLane(SVC_TRAM) == -1
                && isLongEnough(out, MIN_WEAVE_LENGTH)) {
#ifdef DEBUG_CONNECTION_GUESSING
            if (DEBUGCOND) {
                std::cout << "l2l node=" << getID() << " specialCase b\n";
            }
#endif
            // for internal: check which one is the rightmost
            double a1 = in1->getAngleAtNode(this);
            double a2 = in2->getAngleAtNode(this);
            double ccw = GeomHelper::getCCWAngleDiff(a1, a2);
            double cw = GeomHelper::getCWAngleDiff(a1, a2);
            if (ccw > cw) {
                std::swap(in1, in2);
                std::swap(in1Offset, in2Offset);
            }
            in1->addLane2LaneConnections(in1Offset, out, outOffset, in1->getNumLanes() - in1Offset, NBEdge::Lane2LaneInfoType::COMPUTED, true);
            in2->addLane2LaneConnections(in2Offset, out, in1->getNumLanes() + outOffset - in1Offset, in2->getNumLanes() - in2Offset, NBEdge::Lane2LaneInfoType::COMPUTED, true);
            if (out->getSpecialLane(SVC_BICYCLE) >= 0) {
                recheckVClassConnections(out);
            }
            return;
        }
    }
    // special case c):
    //  one in, two out, the incoming has the same number of lanes or only 1 lane less than the sum of the outgoing lanes
    //  --> highway off-ramp
    if (myIncomingEdges.size() == 1 && myOutgoingEdges.size() == 2) {
        NBEdge* in = myIncomingEdges[0];
        NBEdge* out1 = myOutgoingEdges[0];
        NBEdge* out2 = myOutgoingEdges[1];
        const int inOffset = MAX2(0, in->getFirstNonPedestrianNonBicycleLaneIndex(FORWARD, true));
        int out1Offset = MAX2(0, out1->getFirstNonPedestrianNonBicycleLaneIndex(FORWARD, true));
        int out2Offset = MAX2(0, out2->getFirstNonPedestrianNonBicycleLaneIndex(FORWARD, true));
        const int deltaLaneSum = (out2->getNumLanes() + out1->getNumLanes() - out1Offset - out2Offset) - (in->getNumLanes() - inOffset);
        if ((deltaLaneSum == 0 || (deltaLaneSum == 1 && in->getPermissionVariants(inOffset, in->getNumLanes()).size() == 1))
                && (in->getStep() <= NBEdge::EdgeBuildingStep::LANES2EDGES)
                && in != out1
                && in != out2
                && in->isConnectedTo(out1)
                && in->isConnectedTo(out2)
                && !in->isTurningDirectionAt(out1)
                && !in->isTurningDirectionAt(out2)
           ) {
#ifdef DEBUG_CONNECTION_GUESSING
            if (DEBUGCOND) {
                std::cout << "l2l node=" << getID() << " specialCase c\n";
            }
#endif
            // for internal: check which one is the rightmost
            if (NBContHelper::relative_outgoing_edge_sorter(in)(out2, out1)) {
                std::swap(out1, out2);
                std::swap(out1Offset, out2Offset);
            }
            in->addLane2LaneConnections(inOffset, out1, out1Offset, out1->getNumLanes() - out1Offset, NBEdge::Lane2LaneInfoType::COMPUTED, true);
            in->addLane2LaneConnections(out1->getNumLanes() + inOffset - out1Offset - deltaLaneSum, out2, out2Offset, out2->getNumLanes() - out2Offset, NBEdge::Lane2LaneInfoType::COMPUTED, false);
            if (in->getSpecialLane(SVC_BICYCLE) >= 0) {
                recheckVClassConnections(out1);
                recheckVClassConnections(out2);
            }
            return;
        }
    }
    // special case d):
    //  one in, one out, the outgoing has one lane less and node has type 'zipper'
    if (myIncomingEdges.size() == 1 && myOutgoingEdges.size() == 1 && myType == SumoXMLNodeType::ZIPPER) {
        NBEdge* in = myIncomingEdges[0];
        NBEdge* out = myOutgoingEdges[0];
        // check if it's not the turnaround
        if (in->getTurnDestination() == out) {
            // will be added later or not...
            return;
        }
#ifdef DEBUG_CONNECTION_GUESSING
        if (DEBUGCOND) {
            std::cout << "l2l node=" << getID() << " specialCase d\n";
        }
#endif
        const int inOffset = MAX2(0, in->getFirstNonPedestrianLaneIndex(FORWARD, true));
        const int outOffset = MAX2(0, out->getFirstNonPedestrianLaneIndex(FORWARD, true));
        if (in->getStep() <= NBEdge::EdgeBuildingStep::LANES2EDGES
                && in->getNumLanes() - inOffset == out->getNumLanes() - outOffset + 1
                && in != out
                && in->isConnectedTo(out)) {
            for (int i = inOffset; i < in->getNumLanes(); ++i) {
                in->setConnection(i, out, MIN2(outOffset + i, out->getNumLanes() - 1), NBEdge::Lane2LaneInfoType::COMPUTED, true);
            }
            return;
        }
    }
    // special case f):
    //  one in, one out, out has reduced or same number of lanes
    if (myIncomingEdges.size() == 1 && myOutgoingEdges.size() == 1) {
        NBEdge* in = myIncomingEdges[0];
        NBEdge* out = myOutgoingEdges[0];
        // check if it's not the turnaround
        if (in->getTurnDestination() == out) {
            // will be added later or not...
            return;
        }
        int inOffset, inEnd, outOffset, outEnd, reduction;
        getReduction(in, out, inOffset, inEnd, outOffset, outEnd, reduction);
        if (in->getStep() <= NBEdge::EdgeBuildingStep::LANES2EDGES
                && reduction >= 0
                && in != out
                && in->isConnectedTo(out)) {
#ifdef DEBUG_CONNECTION_GUESSING
            if (DEBUGCOND) {
                std::cout << "l2l node=" << getID() << " specialCase f inOff=" << inOffset << " outOff=" << outOffset << " inEnd=" << inEnd << " outEnd=" << outEnd << " reduction=" << reduction << "\n";
            }
#endif
            // in case of reduced lane number, let the rightmost lanes end
            inOffset += reduction;
            for (int i = outOffset; i < outEnd; ++i) {
                in->setConnection(i + inOffset - outOffset, out, i, NBEdge::Lane2LaneInfoType::COMPUTED);
            }
            //std::cout << " special case f at node=" << getID() << " inOffset=" << inOffset << " outOffset=" << outOffset << "\n";
            recheckVClassConnections(out);
            return;
        }
    }

    // go through this node's outgoing edges
    //  for every outgoing edge, compute the distribution of the node's
    //  incoming edges on this edge when approaching this edge
    // the incoming edges' steps will then also be marked as LANE2LANE_RECHECK...
    EdgeVector approaching;
    for (NBEdge* currentOutgoing : myOutgoingEdges) {
        // get the information about edges that do approach this edge
        getEdgesThatApproach(currentOutgoing, approaching);
        const int numApproaching = (int)approaching.size();
        if (numApproaching != 0) {
            ApproachingDivider divider(approaching, currentOutgoing);
            Bresenham::compute(&divider, numApproaching, divider.numAvailableLanes());
        }
#ifdef DEBUG_CONNECTION_GUESSING
        if (DEBUGCOND) {
            std::cout << "l2l node=" << getID() << " outgoing=" << currentOutgoing->getID() << " bresenham:\n";
            for (NBEdge* e : myIncomingEdges) {
                const std::vector<NBEdge::Connection>& elv = e->getConnections();
                for (std::vector<NBEdge::Connection>::const_iterator k = elv.begin(); k != elv.end(); ++k) {
                    std::cout << "  " << e->getID() << "_" << (*k).fromLane << " -> " << Named::getIDSecure((*k).toEdge) << "_" << (*k).toLane << "\n";
                }
            }
        }
#endif
        recheckVClassConnections(currentOutgoing);

        // in case of lane change restrictions on the outgoing edge, ensure that
        // all its lanes can be reached from each connected incoming edge
        bool targetProhibitsChange = false;
        for (int i = 0; i < currentOutgoing->getNumLanes(); i++) {
            const NBEdge::Lane& lane = currentOutgoing->getLanes()[i];
            if ((lane.changeLeft != SVCAll && lane.changeLeft != SVC_IGNORING && i + 1 < currentOutgoing->getNumLanes())
                    || (lane.changeRight != SVCAll && lane.changeRight != SVC_IGNORING && i > 0)) {
                targetProhibitsChange = true;
                break;
            }
        }
        if (targetProhibitsChange) {
            //std::cout << " node=" << getID() << " outgoing=" << currentOutgoing->getID() << " targetProhibitsChange\n";
            for (NBEdge* incoming : myIncomingEdges) {
                if (incoming->getStep() < NBEdge::EdgeBuildingStep::LANES2LANES_DONE) {
                    std::map<int, int> outToIn;
                    for (const NBEdge::Connection& c : incoming->getConnections()) {
                        if (c.toEdge == currentOutgoing) {
                            outToIn[c.toLane] = c.fromLane;
                        }
                    }
                    for (int toLane = 0; toLane < currentOutgoing->getNumLanes(); toLane++) {
                        if (outToIn.count(toLane) == 0) {
                            bool added = false;
                            // find incoming lane for neighboring outgoing
                            for (int i = 0; i < toLane; i++) {
                                if (outToIn.count(i) != 0) {
#ifdef DEBUG_CONNECTION_GUESSING
                                    if (DEBUGCOND) {
                                        std::cout << "l2l node=" << getID() << " from=" << incoming->getID() << " to " << currentOutgoing->getLaneID(toLane) << " (changeProhibited, secondTarget)\n";
                                    }
#endif
                                    incoming->setConnection(outToIn[i], currentOutgoing, toLane, NBEdge::Lane2LaneInfoType::COMPUTED);
                                    added = true;
                                    break;
                                }
                            }
                            if (!added) {
                                for (int i = toLane; i < currentOutgoing->getNumLanes(); i++) {
                                    if (outToIn.count(i) != 0) {
#ifdef DEBUG_CONNECTION_GUESSING
                                        if (DEBUGCOND) {
                                            std::cout << "l2l node=" << getID() << " from=" << incoming->getID() << " to " << currentOutgoing->getLaneID(toLane) << " (changeProhibited, newTarget)\n";
                                        }
#endif
                                        incoming->setConnection(outToIn[i], currentOutgoing, toLane, NBEdge::Lane2LaneInfoType::COMPUTED);
                                        added = true;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    // special case e): rail_crossing
    // there should only be straight connections here
    if (myType == SumoXMLNodeType::RAIL_CROSSING) {
        for (EdgeVector::const_iterator i = myIncomingEdges.begin(); i != myIncomingEdges.end(); i++) {
            const std::vector<NBEdge::Connection> cons = (*i)->getConnections();
            for (std::vector<NBEdge::Connection>::const_iterator k = cons.begin(); k != cons.end(); ++k) {
                if (getDirection(*i, (*k).toEdge) == LinkDirection::TURN) {
                    (*i)->removeFromConnections((*k).toEdge);
                }
            }
        }
    }

    // ... but we may have the case that there are no outgoing edges
    //  In this case, we have to mark the incoming edges as being in state
    //   LANE2LANE( not RECHECK) by hand
    if (myOutgoingEdges.size() == 0) {
        for (NBEdge* incoming : myIncomingEdges) {
            incoming->markAsInLane2LaneState();
        }
    }

#ifdef DEBUG_CONNECTION_GUESSING
    if (DEBUGCOND) {
        std::cout << "final connections at " << getID() << "\n";
        for (NBEdge* e : myIncomingEdges) {
            const std::vector<NBEdge::Connection>& elv = e->getConnections();
            for (std::vector<NBEdge::Connection>::const_iterator k = elv.begin(); k != elv.end(); ++k) {
                std::cout << "  " << e->getID() << "_" << (*k).fromLane << " -> " << Named::getIDSecure((*k).toEdge) << "_" << (*k).toLane << "\n";
            }
        }
    }
#endif
}

void
NBNode::recheckVClassConnections(NBEdge* currentOutgoing) {
    // ensure that all modes have a connection if possible
    for (NBEdge* incoming : myIncomingEdges) {
        if (incoming->getConnectionLanes(currentOutgoing).size() > 0 && incoming->getStep() <= NBEdge::EdgeBuildingStep::LANES2LANES_DONE) {
            // no connections are needed for pedestrians during this step
            // no satisfaction is possible if the outgoing edge disallows
            SVCPermissions unsatisfied = incoming->getPermissions() & currentOutgoing->getPermissions() & ~SVC_PEDESTRIAN;
            //std::cout << "initial unsatisfied modes from edge=" << incoming->getID() << " toEdge=" << currentOutgoing->getID() << " deadModes=" << getVehicleClassNames(unsatisfied) << "\n";
            const std::vector<NBEdge::Connection>& elv = incoming->getConnections();
            for (std::vector<NBEdge::Connection>::const_iterator k = elv.begin(); k != elv.end(); ++k) {
                const NBEdge::Connection& c = *k;
                if (c.toEdge == currentOutgoing && c.toLane >= 0) {
                    const SVCPermissions satisfied = (incoming->getPermissions(c.fromLane) & c.toEdge->getPermissions(c.toLane));
                    //std::cout << "  from=" << incoming->getID() << "_" << c.fromLane << " to=" << c.toEdge->getID() << "_" << c.toLane << " satisfied=" << getVehicleClassNames(satisfied) << "\n";
                    unsatisfied &= ~satisfied;
                }
            }
            if (unsatisfied != 0) {
#ifdef DEBUG_CONNECTION_GUESSING
                if (DEBUGCOND) {
                    std::cout << " unsatisfied modes from edge=" << incoming->getID() << " toEdge=" << currentOutgoing->getID() << " deadModes=" << getVehicleClassNames(unsatisfied) << "\n";
                }
#endif
                int fromLane = 0;
                // first attempt: try to use a dedicated fromLane
                while (unsatisfied != 0 && fromLane < incoming->getNumLanes()) {
                    if (incoming->getPermissions(fromLane) == unsatisfied) {
                        unsatisfied = findToLaneForPermissions(currentOutgoing, fromLane, incoming, unsatisfied);
                    }
                    fromLane++;
                }
                // second attempt: try to re-use a fromLane that already connects to currentOutgoing
                // (because we don't wont to create extra turn lanes)
                fromLane = 0;
                while (unsatisfied != 0 && fromLane < incoming->getNumLanes()) {
                    if ((incoming->getPermissions(fromLane) & unsatisfied) != 0
                            && incoming->getConnectionsFromLane(fromLane, currentOutgoing, -1).size() > 0) {
                        unsatisfied = findToLaneForPermissions(currentOutgoing, fromLane, incoming, unsatisfied);
                    }
                    fromLane++;
                }
                // third attempt: use any possible fromLane
                fromLane = 0;
                while (unsatisfied != 0 && fromLane < incoming->getNumLanes()) {
                    if ((incoming->getPermissions(fromLane) & unsatisfied) != 0) {
                        unsatisfied = findToLaneForPermissions(currentOutgoing, fromLane, incoming, unsatisfied);
                    }
                    fromLane++;
                }
#ifdef DEBUG_CONNECTION_GUESSING
                if (DEBUGCOND) {
                    if (unsatisfied != 0) {
                        std::cout << "     still unsatisfied modes from edge=" << incoming->getID() << " toEdge=" << currentOutgoing->getID() << " deadModes=" << getVehicleClassNames(unsatisfied) << "\n";
                    }
                }
#endif
            }
        }
        // prevent dead-end bus and bicycle lanes (they were excluded by the ApproachingDivider)
        // and the bus/bicycle class might already be satisfied by other lanes
        recheckSpecialConnections(incoming, currentOutgoing, SVC_BUS);
        recheckSpecialConnections(incoming, currentOutgoing, SVC_BICYCLE);
    }
}


void
NBNode::recheckSpecialConnections(NBEdge* incoming, NBEdge* currentOutgoing, SVCPermissions svcSpecial) {
    // assume that left-turns and turn-arounds are better satisfied from lanes to the left
    const int specialTarget = currentOutgoing->getSpecialLane(svcSpecial);
    const LinkDirection dir = getDirection(incoming, currentOutgoing);
    if (incoming->getStep() <= NBEdge::EdgeBuildingStep::LANES2LANES_DONE
            && ((specialTarget >= 0 && dir != LinkDirection::TURN)
                || dir == LinkDirection::RIGHT || dir == LinkDirection::PARTRIGHT || dir == LinkDirection::STRAIGHT)) {
        bool builtConnection = false;
        for (int i = 0; i < (int)incoming->getNumLanes(); i++) {
            if (incoming->getPermissions(i) == svcSpecial
                    && incoming->getConnectionsFromLane(i, currentOutgoing).size() == 0) {
                // find a dedicated bike lane as target
                if (specialTarget >= 0) {
                    incoming->setConnection(i, currentOutgoing, specialTarget, NBEdge::Lane2LaneInfoType::COMPUTED);
#ifdef DEBUG_CONNECTION_GUESSING
                    if (DEBUGCOND) {
                        std::cout << "  extra " << getVehicleClassNames(svcSpecial) << " connection from=" << incoming->getLaneID(i) << " (dedicated) to=" << currentOutgoing->getLaneID(specialTarget)  << "\n";
                    }
#endif
                    builtConnection = true;
                } else {
                    // do not create turns that create a conflict with neighboring lanes
                    if (avoidConfict(incoming, currentOutgoing, svcSpecial, dir, i)) {
                        continue;
                    }
                    // use any lane that allows the special class
                    for (int i2 = 0; i2 < (int)currentOutgoing->getNumLanes(); i2++) {
                        if ((currentOutgoing->getPermissions(i2) & svcSpecial) != 0) {
                            // possibly a double-connection
                            const bool allowDouble = (incoming->getPermissions(i) == svcSpecial
                                                      && (dir == LinkDirection::RIGHT || dir == LinkDirection::PARTRIGHT || dir == LinkDirection::STRAIGHT));
                            incoming->setConnection(i, currentOutgoing, i2, NBEdge::Lane2LaneInfoType::COMPUTED, allowDouble);
#ifdef DEBUG_CONNECTION_GUESSING
                            if (DEBUGCOND) {
                                std::cout << "  extra " << getVehicleClassNames(svcSpecial) << " connection from=" << incoming->getLaneID(i) << " to=" << currentOutgoing->getLaneID(i2)  << "\n";
                            }
#endif
                            builtConnection = true;
                            break;
                        }
                    }
                }
            }
        }
        if (!builtConnection && specialTarget >= 0
                && incoming->getConnectionsFromLane(-1, currentOutgoing, specialTarget).size() == 0) {
            // find origin lane that allows bicycles
            int start = 0;
            int end = incoming->getNumLanes();
            int inc = 1;
            if (dir == LinkDirection::TURN || dir == LinkDirection::LEFT || dir == LinkDirection::PARTLEFT) {
                std::swap(start, end);
                inc = -1;
            }
            for (int i = start; i < end; i += inc) {
                if ((incoming->getPermissions(i) & svcSpecial) != 0) {
                    incoming->setConnection(i, currentOutgoing, specialTarget, NBEdge::Lane2LaneInfoType::COMPUTED);
#ifdef DEBUG_CONNECTION_GUESSING
                    if (DEBUGCOND) {
                        std::cout << "  extra " << getVehicleClassNames(svcSpecial) << " connection from=" << incoming->getLaneID(i) << " (final) to=" << currentOutgoing->getLaneID(specialTarget)  << "\n";
                    }
#endif
                    break;
                }
            }
        }
    }
}


bool
NBNode::avoidConfict(NBEdge* incoming, NBEdge* currentOutgoing, SVCPermissions svcSpecial, LinkDirection dir, int i) {
    for (const auto& c : incoming->getConnections()) {
        if (incoming->getPermissions(c.fromLane) == svcSpecial && c.toEdge == currentOutgoing) {
            return true;
        }
    }
    if (dir == LinkDirection::RIGHT || dir == LinkDirection::PARTRIGHT) {
        for (const auto& c : incoming->getConnections()) {
            if (c.fromLane < i && (c.toEdge != currentOutgoing || incoming->getPermissions(c.fromLane) == svcSpecial)) {
                return true;
            }
        }
    } else if (dir == LinkDirection::RIGHT || dir == LinkDirection::PARTRIGHT) {
        for (const auto& c : incoming->getConnections()) {
            if (c.fromLane > i && (c.toEdge != currentOutgoing || incoming->getPermissions(c.fromLane) == svcSpecial)) {
                return true;
            }
        }
    } else if (svcSpecial != SVC_BICYCLE && dir == LinkDirection::STRAIGHT) {
        for (const auto& c : incoming->getConnections()) {
            const LinkDirection dir2 = getDirection(incoming, c.toEdge);
            if (c.fromLane < i && (dir2 == LinkDirection::LEFT || dir2 == LinkDirection::PARTLEFT)) {
                return true;
            } else if (c.fromLane > i && (dir2 == LinkDirection::RIGHT || dir2 == LinkDirection::PARTRIGHT)) {
                return true;
            }
        }
    }
    return false;
}


void
NBNode::getReduction(const NBEdge* in, const NBEdge* out, int& inOffset, int& inEnd, int& outOffset, int& outEnd, int& reduction) const {
    inOffset = MAX2(0, in->getFirstNonPedestrianNonBicycleLaneIndex(FORWARD, true));
    outOffset = MAX2(0, out->getFirstNonPedestrianNonBicycleLaneIndex(FORWARD, true));
    inEnd = in->getFirstNonPedestrianLaneIndex(BACKWARD, true) + 1;
    outEnd = out->getFirstNonPedestrianLaneIndex(BACKWARD, true) + 1;
    reduction = (inEnd - inOffset) - (outEnd - outOffset);
}


SVCPermissions
NBNode::findToLaneForPermissions(NBEdge* currentOutgoing, int fromLane, NBEdge* incoming, SVCPermissions unsatisfied) {
    for (int toLane = 0; toLane < currentOutgoing->getNumLanes(); ++toLane) {
        const SVCPermissions satisfied = incoming->getPermissions(fromLane) & currentOutgoing->getPermissions(toLane) & unsatisfied;
        if (satisfied != 0 && !incoming->getLaneStruct(fromLane).connectionsDone) {
            if (incoming->hasConnectionTo(currentOutgoing, toLane)
                    && unsatisfied == SVC_TRAM
                    && incoming->getPermissions(fromLane) == currentOutgoing->getPermissions(toLane)) {
                // avoid double tram connection by shifting an existing connection
                for (auto con : incoming->getConnections()) {
                    if (con.toEdge == currentOutgoing && con.toLane == toLane) {
#ifdef DEBUG_CONNECTION_GUESSING
                        if (DEBUGCOND) {
                            std::cout << "  shifting connection from=" << con.fromLane << " to=" << currentOutgoing->getID() << "_" << toLane << ": newFromLane=" << fromLane << " satisfies=" << getVehicleClassNames(satisfied) << "\n";
                        }
#endif
                        incoming->getConnectionRef(con.fromLane, con.toEdge, toLane).fromLane = fromLane;
                        unsatisfied &= ~satisfied;
                        break;
                    }
                }
            } else {
                // other modes (i.e. bus) can fix lane permissions NBPTLineCont::fixPermissions but do not wish to create parallel tram tracks here
                bool mayUseSameDestination = unsatisfied == SVC_TRAM || (unsatisfied & SVC_PASSENGER) != 0;
                incoming->setConnection((int)fromLane, currentOutgoing, toLane, NBEdge::Lane2LaneInfoType::COMPUTED, mayUseSameDestination);
#ifdef DEBUG_CONNECTION_GUESSING
                if (DEBUGCOND) {
                    std::cout << "  new connection from=" << fromLane << " to=" << currentOutgoing->getID() << "_" << toLane << " satisfies=" << getVehicleClassNames(satisfied) << "\n";
                }
#endif
                unsatisfied &= ~satisfied;
            }
        }
    }
    return unsatisfied;
}


int
NBNode::addedLanesRight(NBEdge* out, int addedLanes) const {
    if (out->isOffRamp()) {
        return addedLanes;
    }
    NBNode* to = out->getToNode();
    // check whether a right lane ends
    if (to->getIncomingEdges().size() == 1
            && to->getOutgoingEdges().size() == 1) {
        int inOffset, inEnd, outOffset, outEnd, reduction;
        to->getReduction(out, to->getOutgoingEdges()[0], inOffset, inEnd, outOffset, outEnd, reduction);

        if (reduction > 0) {
            return reduction;
        }
    }
    // check for the presence of right and left turns at the next intersection
    int outLanesRight = 0;
    int outLanesLeft = 0;
    int outLanesStraight = 0;
    for (NBEdge* succ : to->getOutgoingEdges()) {
        if (out->isConnectedTo(succ)) {
            const int outOffset = MAX2(0, succ->getFirstNonPedestrianNonBicycleLaneIndex(FORWARD, true));
            const int usableLanes = succ->getNumLanes() - outOffset;
            LinkDirection dir = to->getDirection(out, succ);
            if (dir == LinkDirection::STRAIGHT) {
                outLanesStraight += usableLanes;
            } else if (dir == LinkDirection::RIGHT || dir == LinkDirection::PARTRIGHT) {
                outLanesRight += usableLanes;
            } else {
                outLanesLeft += usableLanes;
            }
        }
    }
    const int outOffset = MAX2(0, out->getFirstNonPedestrianNonBicycleLaneIndex(FORWARD, true));
    const int outEnd = out->getFirstNonPedestrianLaneIndex(BACKWARD, true) + 1;
    const int usableLanes = outEnd - outOffset;
    int addedTurnLanes = MIN3(
                             addedLanes,
                             MAX2(0, usableLanes - outLanesStraight),
                             outLanesRight + outLanesLeft);
#ifdef DEBUG_CONNECTION_GUESSING
    if (DEBUGCOND) {
        std::cout << "out=" << out->getID() << " usableLanes=" << usableLanes << " addedTurnLanes=" << addedTurnLanes << " addedLanes=" << addedLanes << " outLanesStraight=" << outLanesStraight << " outLanesLeft=" << outLanesLeft << " outLanesRight=" << outLanesRight << "\n";
    }
#endif
    if (outLanesLeft == 0) {
        return addedTurnLanes;
    } else {
        return MIN2(addedTurnLanes / 2, outLanesRight);
    }
}


bool
NBNode::isLongEnough(NBEdge* out, double minLength) {
    double seen = out->getLoadedLength();
    while (seen < minLength) {
        // advance along trivial continuations
        if (out->getToNode()->getOutgoingEdges().size() != 1
                || out->getToNode()->getIncomingEdges().size() != 1) {
            return false;
        } else {
            out = out->getToNode()->getOutgoingEdges()[0];
            seen += out->getLoadedLength();
        }
    }
    return true;
}


void
NBNode::getEdgesThatApproach(NBEdge* currentOutgoing, EdgeVector& approaching) {
    // get the position of the node to get the approaching nodes of
    EdgeVector::const_iterator i = std::find(myAllEdges.begin(),
                                   myAllEdges.end(), currentOutgoing);
    // get the first possible approaching edge
    NBContHelper::nextCW(myAllEdges, i);
    // go through the list of edges clockwise and add the edges
    approaching.clear();
    for (; *i != currentOutgoing;) {
        // check only incoming edges
        if ((*i)->getToNode() == this && (*i)->getTurnDestination() != currentOutgoing) {
            std::vector<int> connLanes = (*i)->getConnectionLanes(currentOutgoing);
            if (connLanes.size() != 0) {
                approaching.push_back(*i);
            }
        }
        NBContHelper::nextCW(myAllEdges, i);
    }
}


void
NBNode::replaceOutgoing(NBEdge* which, NBEdge* by, int laneOff) {
    // replace the edge in the list of outgoing nodes
    EdgeVector::iterator i = std::find(myOutgoingEdges.begin(), myOutgoingEdges.end(), which);
    if (i != myOutgoingEdges.end()) {
        (*i) = by;
        i = std::find(myAllEdges.begin(), myAllEdges.end(), which);
        (*i) = by;
    }
    // replace the edge in connections of incoming edges
    for (i = myIncomingEdges.begin(); i != myIncomingEdges.end(); ++i) {
        (*i)->replaceInConnections(which, by, laneOff);
    }
    // replace within the connetion prohibition dependencies
    replaceInConnectionProhibitions(which, by, 0, laneOff);
}


void
NBNode::replaceOutgoing(const EdgeVector& which, NBEdge* by) {
    // replace edges
    int laneOff = 0;
    for (EdgeVector::const_iterator i = which.begin(); i != which.end(); i++) {
        replaceOutgoing(*i, by, laneOff);
        laneOff += (*i)->getNumLanes();
    }
    // removed double occurrences
    removeDoubleEdges();
    // check whether this node belongs to a district and the edges
    //  must here be also remapped
    if (myDistrict != nullptr) {
        myDistrict->replaceOutgoing(which, by);
    }
}


void
NBNode::replaceIncoming(NBEdge* which, NBEdge* by, int laneOff) {
    // replace the edge in the list of incoming nodes
    EdgeVector::iterator i = std::find(myIncomingEdges.begin(), myIncomingEdges.end(), which);
    if (i != myIncomingEdges.end()) {
        (*i) = by;
        i = std::find(myAllEdges.begin(), myAllEdges.end(), which);
        (*i) = by;
    }
    // replace within the connetion prohibition dependencies
    replaceInConnectionProhibitions(which, by, laneOff, 0);
}


void
NBNode::replaceIncoming(const EdgeVector& which, NBEdge* by) {
    // replace edges
    int laneOff = 0;
    for (EdgeVector::const_iterator i = which.begin(); i != which.end(); i++) {
        replaceIncoming(*i, by, laneOff);
        laneOff += (*i)->getNumLanes();
    }
    // removed double occurrences
    removeDoubleEdges();
    // check whether this node belongs to a district and the edges
    //  must here be also remapped
    if (myDistrict != nullptr) {
        myDistrict->replaceIncoming(which, by);
    }
}



void
NBNode::replaceInConnectionProhibitions(NBEdge* which, NBEdge* by,
                                        int whichLaneOff, int byLaneOff) {
    // replace in keys
    NBConnectionProhibits::iterator j = myBlockedConnections.begin();
    while (j != myBlockedConnections.end()) {
        bool changed = false;
        NBConnection c = (*j).first;
        if (c.replaceFrom(which, whichLaneOff, by, byLaneOff)) {
            changed = true;
        }
        if (c.replaceTo(which, whichLaneOff, by, byLaneOff)) {
            changed = true;
        }
        if (changed) {
            myBlockedConnections[c] = (*j).second;
            myBlockedConnections.erase(j);
            j = myBlockedConnections.begin();
        } else {
            j++;
        }
    }
    // replace in values
    for (j = myBlockedConnections.begin(); j != myBlockedConnections.end(); j++) {
        NBConnectionVector& prohibiting = (*j).second;
        for (NBConnectionVector::iterator k = prohibiting.begin(); k != prohibiting.end(); k++) {
            NBConnection& sprohibiting = *k;
            sprohibiting.replaceFrom(which, whichLaneOff, by, byLaneOff);
            sprohibiting.replaceTo(which, whichLaneOff, by, byLaneOff);
        }
    }
}



void
NBNode::removeDoubleEdges() {
    // check incoming
    for (int i = 0; myIncomingEdges.size() > 0 && i < (int)myIncomingEdges.size() - 1; i++) {
        int j = i + 1;
        while (j < (int)myIncomingEdges.size()) {
            if (myIncomingEdges[i] == myIncomingEdges[j]) {
                myIncomingEdges.erase(myIncomingEdges.begin() + j);
            } else {
                j++;
            }
        }
    }
    // check outgoing
    for (int i = 0; myOutgoingEdges.size() > 0 && i < (int)myOutgoingEdges.size() - 1; i++) {
        int j = i + 1;
        while (j < (int)myOutgoingEdges.size()) {
            if (myOutgoingEdges[i] == myOutgoingEdges[j]) {
                myOutgoingEdges.erase(myOutgoingEdges.begin() + j);
            } else {
                j++;
            }
        }
    }
    // check all
    for (int i = 0; myAllEdges.size() > 0 && i < (int)myAllEdges.size() - 1; i++) {
        int j = i + 1;
        while (j < (int)myAllEdges.size()) {
            if (myAllEdges[i] == myAllEdges[j]) {
                myAllEdges.erase(myAllEdges.begin() + j);
            } else {
                j++;
            }
        }
    }
}


bool
NBNode::hasIncoming(const NBEdge* const e) const {
    return std::find(myIncomingEdges.begin(), myIncomingEdges.end(), e) != myIncomingEdges.end();
}


bool
NBNode::hasOutgoing(const NBEdge* const e) const {
    return std::find(myOutgoingEdges.begin(), myOutgoingEdges.end(), e) != myOutgoingEdges.end();
}


NBEdge*
NBNode::getOppositeIncoming(NBEdge* e) const {
    EdgeVector edges = myIncomingEdges;
    if (find(edges.begin(), edges.end(), e) != edges.end()) {
        edges.erase(find(edges.begin(), edges.end(), e));
    }
    if (edges.size() == 0) {
        return nullptr;
    }
    if (e->getToNode() == this) {
        sort(edges.begin(), edges.end(), NBContHelper::edge_opposite_direction_sorter(e, this, false));
    } else {
        sort(edges.begin(), edges.end(), NBContHelper::edge_similar_direction_sorter(e));
    }
    return edges[0];
}


void
NBNode::addSortedLinkFoes(const NBConnection& mayDrive,
                          const NBConnection& mustStop) {
    if (mayDrive.getFrom() == nullptr ||
            mayDrive.getTo() == nullptr ||
            mustStop.getFrom() == nullptr ||
            mustStop.getTo() == nullptr) {

        WRITE_WARNING(TL("Something went wrong during the building of a connection..."));
        return; // !!! mark to recompute connections
    }
    NBConnectionVector conn = myBlockedConnections[mustStop];
    conn.push_back(mayDrive);
    myBlockedConnections[mustStop] = conn;
}


NBEdge*
NBNode::getPossiblySplittedIncoming(const std::string& edgeid) {
    int size = (int) edgeid.length();
    for (EdgeVector::iterator i = myIncomingEdges.begin(); i != myIncomingEdges.end(); i++) {
        std::string id = (*i)->getID();
        if (id.substr(0, size) == edgeid) {
            return *i;
        }
    }
    return nullptr;
}


NBEdge*
NBNode::getPossiblySplittedOutgoing(const std::string& edgeid) {
    int size = (int) edgeid.length();
    for (EdgeVector::iterator i = myOutgoingEdges.begin(); i != myOutgoingEdges.end(); i++) {
        std::string id = (*i)->getID();
        if (id.substr(0, size) == edgeid) {
            return *i;
        }
    }
    return nullptr;
}


void
NBNode::removeEdge(NBEdge* edge, bool removeFromConnections) {
    EdgeVector::iterator i = std::find(myAllEdges.begin(), myAllEdges.end(), edge);
    if (i != myAllEdges.end()) {
        myAllEdges.erase(i);
        i = std::find(myOutgoingEdges.begin(), myOutgoingEdges.end(), edge);
        if (i != myOutgoingEdges.end()) {
            myOutgoingEdges.erase(i);
            // potential self-loop
            i = std::find(myIncomingEdges.begin(), myIncomingEdges.end(), edge);
            if (i != myIncomingEdges.end()) {
                myIncomingEdges.erase(i);
            }
        } else {
            i = std::find(myIncomingEdges.begin(), myIncomingEdges.end(), edge);
            if (i != myIncomingEdges.end()) {
                myIncomingEdges.erase(i);
            } else {
                // edge must have been either incoming or outgoing
                assert(false);
            }
        }
        if (removeFromConnections) {
            for (i = myAllEdges.begin(); i != myAllEdges.end(); ++i) {
                (*i)->removeFromConnections(edge);
            }
        }
        // invalidate controlled connections for loaded traffic light plans
        const bool incoming = edge->getToNode() == this;
        for (NBTrafficLightDefinition* const tld : myTrafficLights) {
            tld->replaceRemoved(edge, -1, nullptr, -1, incoming);
        }
    }
}


Position
NBNode::getEmptyDir() const {
    Position pos(0, 0);
    for (const NBEdge* const in : myIncomingEdges) {
        Position toAdd = in->getFromNode()->getPosition();
        toAdd.sub(myPosition);
        toAdd.norm2D();
        pos.add(toAdd);
    }
    for (const NBEdge* const out : myOutgoingEdges) {
        Position toAdd = out->getToNode()->getPosition();
        toAdd.sub(myPosition);
        toAdd.norm2D();
        pos.add(toAdd);
    }
    pos.mul(-1. / (double)(myIncomingEdges.size() + myOutgoingEdges.size()));
    if (pos.x() == 0. && pos.y() == 0.) {
        pos = Position(1, 0);
    }
    pos.norm2D();
    return pos;
}



void
NBNode::invalidateIncomingConnections(bool reallowSetting) {
    for (EdgeVector::const_iterator i = myIncomingEdges.begin(); i != myIncomingEdges.end(); i++) {
        (*i)->invalidateConnections(reallowSetting);
    }
}


void
NBNode::invalidateOutgoingConnections(bool reallowSetting) {
    for (EdgeVector::const_iterator i = myOutgoingEdges.begin(); i != myOutgoingEdges.end(); i++) {
        (*i)->invalidateConnections(reallowSetting);
    }
}


bool
NBNode::mustBrake(const NBEdge* const from, const NBEdge* const to, int fromLane, int toLane, bool includePedCrossings) const {
    // unregulated->does not need to brake
    if (myRequest == nullptr) {
        return false;
    }
    // vehicles which do not have a following lane must always decelerate to the end
    if (to == nullptr) {
        return true;
    }
    // maybe we need to brake due to entering a bidi-edge
    if (to->isBidiEdge() && !from->isBidiEdge()) {
        return true;
    }
    // check whether any other connection on this node prohibits this connection
    return myRequest->mustBrake(from, to, fromLane, toLane, includePedCrossings);
}

bool
NBNode::mustBrakeForCrossing(const NBEdge* const from, const NBEdge* const to, const NBNode::Crossing& crossing) const {
    return NBRequest::mustBrakeForCrossing(this, from, to, crossing);
}

bool
NBNode::brakeForCrossingOnExit(const NBEdge* to, LinkDirection dir, bool indirect) const {
    // code is called for connections exiting after an internal junction.
    // If the connection is turning we do not check for crossing priority anymore.
    if (dir == LinkDirection::STRAIGHT && !indirect) {
        return false;
    }
    for (auto& c : myCrossings) {
        if (std::find(c->edges.begin(), c->edges.end(), to) != c->edges.end()) {
            return true;
        }
    }
    return false;
}


bool
NBNode::rightTurnConflict(const NBEdge* from, const NBEdge* to, int fromLane,
                          const NBEdge* prohibitorFrom, const NBEdge* prohibitorTo, int prohibitorFromLane) {
    if (from != prohibitorFrom) {
        return false;
    }
    if (from->isTurningDirectionAt(to)
            || prohibitorFrom->isTurningDirectionAt(prohibitorTo)) {
        // XXX should warn if there are any non-turning connections left of this
        return false;
    }
    // conflict if to is between prohibitorTo and from when going clockwise
    if (to->getStartAngle() == prohibitorTo->getStartAngle()) {
        // reduce rounding errors
        return false;
    }
    const LinkDirection d1 = from->getToNode()->getDirection(from, to);
    // must be a right turn to qualify as rightTurnConflict
    if (d1 == LinkDirection::STRAIGHT) {
        // no conflict for straight going connections
        // XXX actually this should check the main direction (which could also
        // be a turn)
        return false;
    } else {
        const LinkDirection d2 = prohibitorFrom->getToNode()->getDirection(prohibitorFrom, prohibitorTo);
        /* std::cout
            << "from=" << from->getID() << " to=" << to->getID() << " fromLane=" << fromLane
            << " pFrom=" << prohibitorFrom->getID() << " pTo=" << prohibitorTo->getID() << " pFromLane=" << prohibitorFromLane
            << " d1=" << toString(d1) << " d2=" << toString(d2)
            << "\n"; */
        bool flip = false;
        if (d1 == LinkDirection::LEFT || d1 == LinkDirection::PARTLEFT) {
            // check for leftTurnConflicht
            flip = !flip;
            if (d2 == LinkDirection::RIGHT || d2 == LinkDirection::PARTRIGHT) {
                // assume that the left-turning bicycle goes straight at first
                // and thus gets precedence over a right turning vehicle
                return false;
            }
        }
        if ((!flip && fromLane <= prohibitorFromLane) ||
                (flip && fromLane >= prohibitorFromLane)) {
            return false;
        }
        const double toAngleAtNode = fmod(to->getStartAngle() + 180, (double)360.0);
        const double prohibitorToAngleAtNode = fmod(prohibitorTo->getStartAngle() + 180, (double)360.0);
        return (flip != (GeomHelper::getCWAngleDiff(from->getEndAngle(), toAngleAtNode) <
                         GeomHelper::getCWAngleDiff(from->getEndAngle(), prohibitorToAngleAtNode)));
    }
}

bool
NBNode::mergeConflictYields(const NBEdge* from, int fromLane, int fromLaneFoe, NBEdge* to, int toLane) const {
    if (myRequest == nullptr) {
        return false;
    }
    const NBEdge::Connection& con = from->getConnection(fromLane, to, toLane);
    const NBEdge::Connection& prohibitorCon = from->getConnection(fromLaneFoe, to, toLane);
    return myRequest->mergeConflict(from, con, from, prohibitorCon, false);
}


bool
NBNode::mergeConflict(const NBEdge* from, const NBEdge::Connection& con,
                      const NBEdge* prohibitorFrom,  const NBEdge::Connection& prohibitorCon, bool foes) const {
    if (myRequest == nullptr) {
        return false;
    }
    return myRequest->mergeConflict(from, con, prohibitorFrom, prohibitorCon, foes);
}

bool
NBNode::bidiConflict(const NBEdge* from, const NBEdge::Connection& con,
                     const NBEdge* prohibitorFrom,  const NBEdge::Connection& prohibitorCon, bool foes) const {
    if (myRequest == nullptr) {
        return false;
    }
    return myRequest->bidiConflict(from, con, prohibitorFrom, prohibitorCon, foes);
}

bool
NBNode::turnFoes(const NBEdge* from, const NBEdge* to, int fromLane,
                 const NBEdge* from2, const NBEdge* to2, int fromLane2,
                 bool lefthand) const {
    UNUSED_PARAMETER(lefthand);
    if (from != from2 || to == to2 || fromLane == fromLane2) {
        return false;
    }
    if (from->isTurningDirectionAt(to)
            || from2->isTurningDirectionAt(to2)) {
        // XXX should warn if there are any non-turning connections left of this
        return false;
    }
    bool result = false;
    EdgeVector::const_iterator it = std::find(myAllEdges.begin(), myAllEdges.end(), from);
    if (fromLane < fromLane2) {
        // conflict if 'to' comes before 'to2' going clockwise starting at 'from'
        while (*it != to2) {
            if (*it == to) {
                result = true;
            }
            NBContHelper::nextCW(myAllEdges, it);
        }
    } else {
        // conflict if 'to' comes before 'to2' going counter-clockwise starting at 'from'
        while (*it != to2) {
            if (*it == to) {
                result = true;
            }
            NBContHelper::nextCCW(myAllEdges, it);
        }
    }
    /*
    if (result) {
        std::cout << "turnFoes node=" << getID()
        << " from=" << from->getLaneID(fromLane)
        << " to=" << to->getID()
        << " from2=" << from2->getLaneID(fromLane2)
        << " to2=" << to2->getID()
        << "\n";
    }
    */
    return result;
}


bool
NBNode::isLeftMover(const NBEdge* const from, const NBEdge* const to) const {
    // when the junction has only one incoming edge, there are no
    //  problems caused by left blockings
    if (myIncomingEdges.size() == 1 || myOutgoingEdges.size() == 1) {
        return false;
    }
    double fromAngle = from->getAngleAtNode(this);
    double toAngle = to->getAngleAtNode(this);
    double cw = GeomHelper::getCWAngleDiff(fromAngle, toAngle);
    double ccw = GeomHelper::getCCWAngleDiff(fromAngle, toAngle);
    std::vector<NBEdge*>::const_iterator i = std::find(myAllEdges.begin(), myAllEdges.end(), from);
    do {
        NBContHelper::nextCW(myAllEdges, i);
    } while ((!hasOutgoing(*i) || from->isTurningDirectionAt(*i)) && *i != from);
    return cw < ccw && (*i) == to && myOutgoingEdges.size() > 2;
}


bool
NBNode::forbids(const NBEdge* const possProhibitorFrom, const NBEdge* const possProhibitorTo,
                const NBEdge* const possProhibitedFrom, const NBEdge* const possProhibitedTo,
                bool regardNonSignalisedLowerPriority) const {
    return myRequest != nullptr && myRequest->forbids(possProhibitorFrom, possProhibitorTo,
            possProhibitedFrom, possProhibitedTo,
            regardNonSignalisedLowerPriority);
}


bool
NBNode::foes(const NBEdge* const from1, const NBEdge* const to1,
             const NBEdge* const from2, const NBEdge* const to2) const {
    return myRequest != nullptr && myRequest->foes(from1, to1, from2, to2);
}


void
NBNode::remapRemoved(NBTrafficLightLogicCont& tc,
                     NBEdge* removed, const EdgeVector& incoming,
                     const EdgeVector& outgoing) {
    assert(find(incoming.begin(), incoming.end(), removed) == incoming.end());
    bool changed = true;
    while (changed) {
        changed = false;
        NBConnectionProhibits blockedConnectionsTmp = myBlockedConnections;
        NBConnectionProhibits blockedConnectionsNew;
        // remap in connections
        for (NBConnectionProhibits::iterator i = blockedConnectionsTmp.begin(); i != blockedConnectionsTmp.end(); i++) {
            const NBConnection& blocker = (*i).first;
            const NBConnectionVector& blocked = (*i).second;
            // check the blocked connections first
            // check whether any of the blocked must be changed
            bool blockedChanged = false;
            NBConnectionVector newBlocked;
            NBConnectionVector::const_iterator j;
            for (j = blocked.begin(); j != blocked.end(); j++) {
                const NBConnection& sblocked = *j;
                if (sblocked.getFrom() == removed || sblocked.getTo() == removed) {
                    blockedChanged = true;
                }
            }
            // adapt changes if so
            for (j = blocked.begin(); blockedChanged && j != blocked.end(); j++) {
                const NBConnection& sblocked = *j;
                if (sblocked.getFrom() == removed && sblocked.getTo() == removed) {
                    /*                    for(EdgeVector::const_iterator k=incoming.begin(); k!=incoming.end(); k++) {
                    !!!                        newBlocked.push_back(NBConnection(*k, *k));
                                        }*/
                } else if (sblocked.getFrom() == removed) {
                    assert(sblocked.getTo() != removed);
                    for (EdgeVector::const_iterator k = incoming.begin(); k != incoming.end(); k++) {
                        newBlocked.push_back(NBConnection(*k, sblocked.getTo()));
                    }
                } else if (sblocked.getTo() == removed) {
                    assert(sblocked.getFrom() != removed);
                    for (EdgeVector::const_iterator k = outgoing.begin(); k != outgoing.end(); k++) {
                        newBlocked.push_back(NBConnection(sblocked.getFrom(), *k));
                    }
                } else {
                    newBlocked.push_back(NBConnection(sblocked.getFrom(), sblocked.getTo()));
                }
            }
            if (blockedChanged) {
                blockedConnectionsNew[blocker] = newBlocked;
                changed = true;
            }
            // if the blocked were kept
            else {
                if (blocker.getFrom() == removed && blocker.getTo() == removed) {
                    changed = true;
                    /*                    for(EdgeVector::const_iterator k=incoming.begin(); k!=incoming.end(); k++) {
                    !!!                        blockedConnectionsNew[NBConnection(*k, *k)] = blocked;
                                        }*/
                } else if (blocker.getFrom() == removed) {
                    assert(blocker.getTo() != removed);
                    changed = true;
                    for (EdgeVector::const_iterator k = incoming.begin(); k != incoming.end(); k++) {
                        blockedConnectionsNew[NBConnection(*k, blocker.getTo())] = blocked;
                    }
                } else if (blocker.getTo() == removed) {
                    assert(blocker.getFrom() != removed);
                    changed = true;
                    for (EdgeVector::const_iterator k = outgoing.begin(); k != outgoing.end(); k++) {
                        blockedConnectionsNew[NBConnection(blocker.getFrom(), *k)] = blocked;
                    }
                } else {
                    blockedConnectionsNew[blocker] = blocked;
                }
            }
        }
        myBlockedConnections = blockedConnectionsNew;
    }
    // remap in traffic lights
    tc.remapRemoved(removed, incoming, outgoing);
}


NBEdge*
NBNode::getNextCompatibleOutgoing(const NBEdge* incoming, SVCPermissions vehPerm, EdgeVector::const_iterator itOut, bool clockwise) const {
    EdgeVector::const_iterator i = itOut;
    while (*i != incoming) {
        if (clockwise) {
            NBContHelper::nextCW(myAllEdges, i);
        } else {
            NBContHelper::nextCCW(myAllEdges, i);
        }
        if ((*i)->getFromNode() != this) {
            // only look for outgoing edges
            // @note we use myAllEdges to stop at the incoming edge
            continue;
        }
        if (incoming->isTurningDirectionAt(*i)) {
            return nullptr;
        }
        if ((vehPerm & (*i)->getPermissions()) != 0 || vehPerm == 0) {
            return *i;
        }
    }
    return nullptr;
}


bool
NBNode::isStraighter(const NBEdge* const incoming, const double angle, const SVCPermissions vehPerm, const int modeLanes, const NBEdge* const candidate) const {
    if (candidate != nullptr) {
        const double candAngle = NBHelpers::normRelAngle(incoming->getAngleAtNode(this), candidate->getAngleAtNode(this));
        // they are too similar it does not matter
        if (fabs(angle - candAngle) < 5.) {
            return false;
        }
        // the other edge is at least 5 degree straighter
        if (fabs(candAngle) < fabs(angle) - 5.) {
            return true;
        }
        if (fabs(angle) < fabs(candAngle) - 5.) {
            return false;
        }
        if (fabs(candAngle) < 44.) {
            // the lane count for the same modes is larger
            const int candModeLanes = candidate->getNumLanesThatAllow(vehPerm);
            if (candModeLanes > modeLanes) {
                return true;
            }
            if (candModeLanes < modeLanes) {
                return false;
            }
            // we would create a left turn
            if (candAngle < 0 && angle > 0) {
                return true;
            }
            if (angle < 0 && candAngle > 0) {
                return false;
            }
        }
    }
    return false;
}

EdgeVector
NBNode::getPassengerEdges(bool incoming) const {
    EdgeVector result;
    for (NBEdge* e : (incoming ? myIncomingEdges : myOutgoingEdges)) {
        if ((e->getPermissions() & SVC_PASSENGER) != 0) {
            result.push_back(e);
        }
    }
    return result;
}

LinkDirection
NBNode::getDirection(const NBEdge* const incoming, const NBEdge* const outgoing, bool leftHand) const {
    // ok, no connection at all -> dead end
    if (outgoing == nullptr) {
        return LinkDirection::NODIR;
    }
    assert(incoming->getToNode() == this);
    assert(outgoing->getFromNode() == this);
    if (incoming->getJunctionPriority(this) == NBEdge::JunctionPriority::ROUNDABOUT && outgoing->getJunctionPriority(this) == NBEdge::JunctionPriority::ROUNDABOUT) {
        return LinkDirection::STRAIGHT;
    }
    // turning direction
    if (incoming->isTurningDirectionAt(outgoing)) {
        if (isExplicitRailNoBidi(incoming, outgoing)) {
            return LinkDirection::STRAIGHT;
        }
        return leftHand ? LinkDirection::TURN_LEFTHAND : LinkDirection::TURN;
    }
    // get the angle between incoming/outgoing at the junction
    const double angle = NBHelpers::normRelAngle(incoming->getAngleAtNode(this), outgoing->getAngleAtNode(this));
    // ok, should be a straight connection
    EdgeVector::const_iterator itOut = std::find(myAllEdges.begin(), myAllEdges.end(), outgoing);
    SVCPermissions vehPerm = incoming->getPermissions() & outgoing->getPermissions();
    if (vehPerm != SVC_PEDESTRIAN) {
        vehPerm &= ~SVC_PEDESTRIAN;
    }
    const int modeLanes = outgoing->getNumLanesThatAllow(vehPerm);
    if (fabs(angle) < 44.) {
        if (fabs(angle) > 6.) {
            if (isStraighter(incoming, angle, vehPerm, modeLanes, getNextCompatibleOutgoing(incoming, vehPerm, itOut, true))) {
                return angle > 0 ? LinkDirection::PARTRIGHT : LinkDirection::PARTLEFT;
            }
            if (isStraighter(incoming, angle, vehPerm, modeLanes, getNextCompatibleOutgoing(incoming, vehPerm, itOut, false))) {
                return angle > 0 ? LinkDirection::PARTRIGHT : LinkDirection::PARTLEFT;
            }
        }
        if (angle > 0 && incoming->getJunctionPriority(this) == NBEdge::JunctionPriority::ROUNDABOUT) {
            return angle > 15 ? LinkDirection::RIGHT : LinkDirection::PARTRIGHT;
        }
        return LinkDirection::STRAIGHT;
    }

    if (angle > 0) {
        // check whether any other edge goes further to the right
        if (angle > 90 + NUMERICAL_EPS) {
            return LinkDirection::RIGHT;
        }
        NBEdge* outCW = getNextCompatibleOutgoing(incoming, vehPerm, itOut, !leftHand);
        if (outCW != nullptr) {
            return LinkDirection::PARTRIGHT;
        } else {
            return LinkDirection::RIGHT;
        }
    } else {
        // check whether any other edge goes further to the left
        if (angle < -170 && incoming->getGeometry().reverse() == outgoing->getGeometry()) {
            if (isExplicitRailNoBidi(incoming, outgoing)) {
                return LinkDirection::STRAIGHT;
            }
            return leftHand ? LinkDirection::TURN_LEFTHAND : LinkDirection::TURN;
        } else if (angle < -(90 + NUMERICAL_EPS)) {
            return LinkDirection::LEFT;
        }
        NBEdge* outCCW = getNextCompatibleOutgoing(incoming, vehPerm, itOut, leftHand);
        if (outCCW != nullptr) {
            return LinkDirection::PARTLEFT;
        } else {
            return LinkDirection::LEFT;
        }
    }
}


bool
NBNode::isExplicitRailNoBidi(const NBEdge* incoming, const NBEdge* outgoing) {
    // assume explicit connections at sharp turn-arounds are either for reversal or due to a geometry glitch
    // (but should not have been guessed)
    // @note this function is also called from NBAlgorithms when there aren't any connections ready
    return (incoming->getStep() >= NBEdge::EdgeBuildingStep::LANES2LANES_RECHECK
            && isRailway(incoming->getPermissions())
            && isRailway(outgoing->getPermissions())
            && incoming->getBidiEdge() != outgoing);
}


LinkState
NBNode::getLinkState(const NBEdge* incoming, const NBEdge* outgoing, int fromLane, int toLane,
                     bool mayDefinitelyPass, const std::string& tlID) const {
    if (myType == SumoXMLNodeType::RAIL_CROSSING && isRailway(incoming->getPermissions())) {
        return LINKSTATE_MAJOR; // the trains must run on time
    }
    if (tlID != "") {
        if (getRightOfWay() == RightOfWay::ALLWAYSTOP) {
            return LINKSTATE_ALLWAY_STOP;
        }
        return mustBrake(incoming, outgoing, fromLane, toLane, true) ? LINKSTATE_TL_OFF_BLINKING : LINKSTATE_TL_OFF_NOSIGNAL;
    }
    if (outgoing == nullptr) { // always off
        return LINKSTATE_TL_OFF_NOSIGNAL;
    }
    if ((myType == SumoXMLNodeType::RIGHT_BEFORE_LEFT || myType == SumoXMLNodeType::LEFT_BEFORE_RIGHT)
            && mustBrake(incoming, outgoing, fromLane, toLane, true)) {
        return LINKSTATE_EQUAL; // all the same
    }
    if (myType == SumoXMLNodeType::ALLWAY_STOP) {
        return LINKSTATE_ALLWAY_STOP; // all drive, first one to arrive may drive first
    }
    if (myType == SumoXMLNodeType::ZIPPER && zipperConflict(incoming, outgoing, fromLane, toLane)) {
        return LINKSTATE_ZIPPER;
    }
    if (!mayDefinitelyPass
            && mustBrake(incoming, outgoing, fromLane, toLane, true)
            // legacy mode
            && (!incoming->isInsideTLS() || getDirection(incoming, outgoing) != LinkDirection::STRAIGHT)
            // avoid linkstate minor at pure railway nodes
            && (!NBNodeTypeComputer::isRailwayNode(this) || unsignalizedOperation())) {
        return myType == SumoXMLNodeType::PRIORITY_STOP && incoming->getJunctionPriority(this) == NBEdge::JunctionPriority::MINOR_ROAD ? LINKSTATE_STOP : LINKSTATE_MINOR; // minor road
    }
    // traffic lights are not regarded here
    return LINKSTATE_MAJOR;
}


bool
NBNode::zipperConflict(const NBEdge* incoming, const NBEdge* outgoing, int fromLane, int toLane) const {
    if (mustBrake(incoming, outgoing, fromLane, toLane, false)) {
        // there should be another connection with the same target (not just some intersecting trajectories)
        for (const NBEdge* in : getIncomingEdges()) {
            for (const NBEdge::Connection& c : in->getConnections()) {
                if ((in != incoming || c.fromLane != fromLane) && c.toEdge == outgoing && c.toLane == toLane) {
                    return true;
                }
            }
        }
    }
    return false;
}


bool
NBNode::unsignalizedOperation() const {
    SVCPermissions railClasses = 0;
    for (NBEdge* e : myIncomingEdges) {
        railClasses |= (e->getPermissions() & SVC_RAIL_CLASSES);
    }
    assert(railClasses != 0);
    return ((railClasses & myPermitUnsignalizedClasses) == railClasses
            && (railClasses & myHaveRailSignalClasses) == 0);
}


void
NBNode::initRailSignalClasses(const NBNodeCont& nc) {
    myPermitUnsignalizedClasses = parseVehicleClasses(OptionsCont::getOptions().getStringVector("railway.signal.permit-unsignalized"));
    myHaveRailSignalClasses = 0;
    for (auto it : nc) {
        const NBNode* n = it.second;
        if (n->getType() == SumoXMLNodeType::RAIL_SIGNAL) {
            for (const NBEdge* in : n->getIncomingEdges()) {
                myHaveRailSignalClasses |= in->getPermissions();
            }
        }
    }
}


bool
NBNode::checkIsRemovable() const {
    std::string reason;
    return checkIsRemovableReporting(reason);
}

bool
NBNode::checkIsRemovableReporting(std::string& reason) const {
    if (getEdges().empty()) {
        return true;
    }
    // check whether this node is included in a traffic light or crossing
    if (myTrafficLights.size() != 0) {
        reason = "TLS";
        return false;
    }
    if (myType == SumoXMLNodeType::RAIL_SIGNAL) {
        reason = "rail_signal";
        return false;
    }
    if (myCrossings.size() != 0) {
        reason = "crossing";
        return false;
    }
    EdgeVector::const_iterator i;
    // one in, one out -> just a geometry ...
    if (myOutgoingEdges.size() == 1 && myIncomingEdges.size() == 1) {
        // ... if types match ...
        if (!myIncomingEdges[0]->expandableBy(myOutgoingEdges[0], reason)) {
            reason = "edges incompatible: " + reason;
            return false;
        }
        if (myIncomingEdges[0]->getTurnDestination(true) == myOutgoingEdges[0]) {
            reason = "turnaround";
            return false;
        }
        return true;
    }
    // two in, two out -> may be something else
    if (myOutgoingEdges.size() == 2 && myIncomingEdges.size() == 2) {
        // check whether the origin nodes of the incoming edges differ
        std::set<NBNode*> origSet;
        for (i = myIncomingEdges.begin(); i != myIncomingEdges.end(); i++) {
            origSet.insert((*i)->getFromNode());
        }
        if (origSet.size() < 2) {
            // overlapping case
            if (myIncomingEdges[0]->getGeometry() == myIncomingEdges[1]->getGeometry() &&
                    myOutgoingEdges[0]->getGeometry() == myOutgoingEdges[1]->getGeometry()) {
                return ((myIncomingEdges[0]->expandableBy(myOutgoingEdges[0], reason) &&
                         myIncomingEdges[1]->expandableBy(myOutgoingEdges[1], reason))
                        || (myIncomingEdges[0]->expandableBy(myOutgoingEdges[1], reason) &&
                            myIncomingEdges[1]->expandableBy(myOutgoingEdges[0], reason)));
            }
        }
        // check whether this node is an intermediate node of
        //  a two-directional street
        for (i = myIncomingEdges.begin(); i != myIncomingEdges.end(); i++) {
            // each of the edges must have an opposite direction edge
            NBEdge* opposite = (*i)->getTurnDestination(true);
            if (opposite != nullptr) {
                // the other outgoing edges must be the continuation of the current
                NBEdge* continuation = opposite == myOutgoingEdges.front() ? myOutgoingEdges.back() : myOutgoingEdges.front();
                // check whether the types allow joining
                if (!(*i)->expandableBy(continuation, reason)) {
                    reason = "edges incompatible: " + reason;
                    return false;
                }
            } else {
                // ok, at least one outgoing edge is not an opposite
                //  of an incoming one
                reason = "not opposites";
                return false;
            }
        }
        return true;
    }
    // ok, a real node
    reason = "intersection";
    return false;
}


std::vector<std::pair<NBEdge*, NBEdge*> >
NBNode::getEdgesToJoin() const {
    assert(checkIsRemovable());
    std::vector<std::pair<NBEdge*, NBEdge*> > ret;
    // one in, one out-case
    if (myOutgoingEdges.size() == 1 && myIncomingEdges.size() == 1) {
        ret.push_back(std::make_pair(myIncomingEdges[0], myOutgoingEdges[0]));
        return ret;
    }
    if (myIncomingEdges.size() == 2 && myOutgoingEdges.size() == 2) {
        // two in, two out-case
        if (myIncomingEdges[0]->getGeometry() == myIncomingEdges[1]->getGeometry() &&
                myOutgoingEdges[0]->getGeometry() == myOutgoingEdges[1]->getGeometry()) {
            // overlapping edges
            std::string reason;
            if (myIncomingEdges[0]->expandableBy(myOutgoingEdges[0], reason)) {
                ret.push_back(std::make_pair(myIncomingEdges[0], myOutgoingEdges[0]));
                ret.push_back(std::make_pair(myIncomingEdges[1], myOutgoingEdges[1]));
            } else {
                ret.push_back(std::make_pair(myIncomingEdges[0], myOutgoingEdges[1]));
                ret.push_back(std::make_pair(myIncomingEdges[1], myOutgoingEdges[0]));
            }
            return ret;
        }
    }
    for (EdgeVector::const_iterator i = myIncomingEdges.begin(); i != myIncomingEdges.end(); i++) {
        // join with the edge that is not a turning direction
        NBEdge* opposite = (*i)->getTurnDestination(true);
        assert(opposite != 0);
        NBEdge* continuation = opposite == myOutgoingEdges.front() ? myOutgoingEdges.back() : myOutgoingEdges.front();
        ret.push_back(std::pair<NBEdge*, NBEdge*>(*i, continuation));
    }
    return ret;
}


const PositionVector&
NBNode::getShape() const {
    return myPoly;
}


void
NBNode::setCustomShape(const PositionVector& shape) {
    myPoly = shape;
    myHaveCustomPoly = (myPoly.size() > 1);
    if (myHaveCustomPoly) {
        for (EdgeVector::iterator i = myAllEdges.begin(); i != myAllEdges.end(); i++) {
            (*i)->resetNodeBorder(this);
        }
    }
}


NBEdge*
NBNode::getConnectionTo(NBNode* n) const {
    for (NBEdge* e : myOutgoingEdges) {
        if (e->getToNode() == n && e->getPermissions() != 0) {
            return e;
        }
    }
    return nullptr;
}


bool
NBNode::isNearDistrict() const {
    if (isDistrict()) {
        return false;
    }
    for (const NBEdge* const t : getEdges()) {
        const NBNode* const other = t->getToNode() == this ? t->getFromNode() : t->getToNode();
        for (const NBEdge* const k : other->getEdges()) {
            if (k->getFromNode()->isDistrict() || k->getToNode()->isDistrict()) {
                return true;
            }
        }
    }
    return false;
}


bool
NBNode::isDistrict() const {
    return myType == SumoXMLNodeType::DISTRICT;
}


int
NBNode::guessCrossings() {
#ifdef DEBUG_PED_STRUCTURES
    gDebugFlag1 = DEBUGCOND;
#endif
    int numGuessed = 0;
    if (myCrossings.size() > 0 || myDiscardAllCrossings) {
        // user supplied crossings, do not guess
        return numGuessed;
    }
    DEBUGCOUT(gDebugFlag1, "guess crossings for " << getID() << "\n")
    EdgeVector allEdges = getEdgesSortedByAngleAtNodeCenter();
    // check for pedestrial lanes going clockwise around the node
    std::vector<std::pair<NBEdge*, bool> > normalizedLanes;
    for (EdgeVector::const_iterator it = allEdges.begin(); it != allEdges.end(); ++it) {
        NBEdge* edge = *it;
        const std::vector<NBEdge::Lane>& lanes = edge->getLanes();
        if (edge->getFromNode() == this) {
            for (std::vector<NBEdge::Lane>::const_reverse_iterator it_l = lanes.rbegin(); it_l != lanes.rend(); ++it_l) {
                normalizedLanes.push_back(std::make_pair(edge, ((*it_l).permissions & SVC_PEDESTRIAN) != 0));
            }
        } else {
            for (std::vector<NBEdge::Lane>::const_iterator it_l = lanes.begin(); it_l != lanes.end(); ++it_l) {
                normalizedLanes.push_back(std::make_pair(edge, ((*it_l).permissions & SVC_PEDESTRIAN) != 0));
            }
        }
    }
    // do we even have a pedestrian lane?
    int firstSidewalk = -1;
    for (int i = 0; i < (int)normalizedLanes.size(); ++i) {
        if (normalizedLanes[i].second) {
            firstSidewalk = i;
            break;
        }
    }
    int hadCandidates = 0;
    std::vector<int> connectedCandidates; // number of crossings that were built for each connected candidate
    if (firstSidewalk != -1) {
        // rotate lanes to ensure that the first one allows pedestrians
        std::vector<std::pair<NBEdge*, bool> > tmp;
        copy(normalizedLanes.begin() + firstSidewalk, normalizedLanes.end(), std::back_inserter(tmp));
        copy(normalizedLanes.begin(), normalizedLanes.begin() + firstSidewalk, std::back_inserter(tmp));
        normalizedLanes = tmp;
        // find candidates
        EdgeVector candidates;
        for (int i = 0; i < (int)normalizedLanes.size(); ++i) {
            NBEdge* edge = normalizedLanes[i].first;
            const bool allowsPed = normalizedLanes[i].second;
            DEBUGCOUT(gDebugFlag1, "  cands=" << toString(candidates) << "  edge=" << edge->getID() << " allowsPed=" << allowsPed << "\n")
            if (!allowsPed && (candidates.size() == 0 || candidates.back() != edge)) {
                candidates.push_back(edge);
            } else if (allowsPed) {
                if (candidates.size() > 0) {
                    if (hadCandidates > 0 || forbidsPedestriansAfter(normalizedLanes, i)) {
                        hadCandidates++;
                        const int n = checkCrossing(candidates);
                        numGuessed += n;
                        if (n > 0) {
                            connectedCandidates.push_back(n);
                        }
                    }
                    candidates.clear();
                }
            }
        }
        if (hadCandidates > 0 && candidates.size() > 0) {
            // avoid wrapping around to the same sidewalk
            hadCandidates++;
            const int n = checkCrossing(candidates);
            numGuessed += n;
            if (n > 0) {
                connectedCandidates.push_back(n);
            }
        }
    }
    // Avoid duplicate crossing between the same pair of walkingareas
    DEBUGCOUT(gDebugFlag1, "  hadCandidates=" << hadCandidates << "  connectedCandidates=" << toString(connectedCandidates) << "\n")
    if (hadCandidates == 2 && connectedCandidates.size() == 2) {
        // One or both of them might be split: remove the one with less splits
        if (connectedCandidates.back() <= connectedCandidates.front()) {
            numGuessed -= connectedCandidates.back();
            myCrossings.erase(myCrossings.end() - connectedCandidates.back(), myCrossings.end());
        } else {
            numGuessed -= connectedCandidates.front();
            myCrossings.erase(myCrossings.begin(), myCrossings.begin() + connectedCandidates.front());
        }
    }
    std::sort(myCrossings.begin(), myCrossings.end(), NBNodesEdgesSorter::crossing_by_junction_angle_sorter(this, myAllEdges));
#ifdef DEBUG_PED_STRUCTURES
    if (gDebugFlag1) {
        std::cout << "guessedCrossings:\n";
        for (auto& crossing : myCrossings) {
            std::cout << "  edges=" << toString(crossing->edges) << "\n";
        }
    }
#endif
    if (numGuessed > 0 && isSimpleContinuation(true, true)) {
        // avoid narrow node shape when there is a crossing
        computeNodeShape(-1);
        for (NBEdge* e : myAllEdges) {
            e->computeEdgeShape();
        }
    }
    return numGuessed;
}


int
NBNode::checkCrossing(EdgeVector candidates, bool checkOnly) {
    DEBUGCOUT(gDebugFlag1, "checkCrossing candidates=" << toString(candidates) << "\n")
    if (candidates.size() == 0) {
        DEBUGCOUT(gDebugFlag1, "no crossing added (numCandidates=" << candidates.size() << ")\n")
        return 0;
    } else {
        // check whether the edges may be part of a common crossing due to having similar angle
        double prevAngle = -100000; // dummy
        for (int i = 0; i < (int)candidates.size(); ++i) {
            NBEdge* edge = candidates[i];
            double angle = edge->getCrossingAngle(this);
            // edges should be sorted by angle but this only holds true approximately
            if (i > 0 && fabs(NBHelpers::relAngle(angle, prevAngle)) > EXTEND_CROSSING_ANGLE_THRESHOLD) {
                DEBUGCOUT(gDebugFlag1, "no crossing added (found angle difference of " << fabs(NBHelpers::relAngle(angle, prevAngle)) << " at i=" << i << "\n")
                return 0;
            }
            if (!checkOnly && !isTLControlled() && myType != SumoXMLNodeType::RAIL_CROSSING && edge->getSpeed() > OptionsCont::getOptions().getFloat("crossings.guess.speed-threshold")) {
                DEBUGCOUT(gDebugFlag1, "no crossing added (uncontrolled, edge with speed > " << edge->getSpeed() << ")\n")
                return 0;
            }
            prevAngle = angle;
        }
        if (candidates.size() == 1 || getType() == SumoXMLNodeType::RAIL_CROSSING) {
            if (!checkOnly) {
                addCrossing(candidates, NBEdge::UNSPECIFIED_WIDTH, isTLControlled()
                            || (isRoundabout() && OptionsCont::getOptions().getBool("crossings.guess.roundabout-priority")));
                DEBUGCOUT(gDebugFlag1, "adding crossing: " << toString(candidates) << "\n")
            }
            return 1;
        } else {
            // check for intermediate walking areas
            prevAngle = -100000; // dummy
            for (EdgeVector::iterator it = candidates.begin(); it != candidates.end(); ++it) {
                double angle = (*it)->getCrossingAngle(this);
                if (it != candidates.begin()) {
                    NBEdge* prev = *(it - 1);
                    NBEdge* curr = *it;
                    Position prevPos, currPos;
                    int laneI;
                    // compute distance between candiate edges
                    double intermediateWidth = 0;
                    if (prev->getToNode() == this) {
                        laneI = prev->getNumLanes() - 1;
                        prevPos = prev->getLanes()[laneI].shape[-1];
                    } else {
                        laneI = 0;
                        prevPos = prev->getLanes()[laneI].shape[0];
                    }
                    intermediateWidth -= 0.5 * prev->getLaneWidth(laneI);
                    if (curr->getFromNode() == this) {
                        laneI = curr->getNumLanes() - 1;
                        currPos = curr->getLanes()[laneI].shape[0];
                    } else {
                        laneI = 0;
                        currPos = curr->getLanes()[laneI].shape[-1];
                    }
                    intermediateWidth -= 0.5 * curr->getLaneWidth(laneI);
                    intermediateWidth += currPos.distanceTo2D(prevPos);
                    DEBUGCOUT(gDebugFlag1, " prevAngle=" << prevAngle << " angle=" << angle << " intermediateWidth=" << intermediateWidth << "\n")
                    if (fabs(NBHelpers::relAngle(prevAngle, angle)) > SPLIT_CROSSING_ANGLE_THRESHOLD
                            || (intermediateWidth > SPLIT_CROSSING_WIDTH_THRESHOLD)) {
                        return checkCrossing(EdgeVector(candidates.begin(), it), checkOnly)
                               + checkCrossing(EdgeVector(it, candidates.end()), checkOnly);
                    }
                }
                prevAngle = angle;
            }
            if (!checkOnly) {
                addCrossing(candidates, NBEdge::UNSPECIFIED_WIDTH, isTLControlled()
                            || (isRoundabout() && OptionsCont::getOptions().getBool("crossings.guess.roundabout-priority")));
                DEBUGCOUT(gDebugFlag1, "adding crossing: " << toString(candidates) << "\n")
            }
            return 1;
        }
    }
}


bool
NBNode::checkCrossingDuplicated(EdgeVector edges) {
    // sort edge vector
    std::sort(edges.begin(), edges.end());
    // iterate over crossing to find a crossing with the same edges
    for (auto& crossing : myCrossings) {
        // sort edges of crossing before compare
        EdgeVector edgesOfCrossing = crossing->edges;
        std::sort(edgesOfCrossing.begin(), edgesOfCrossing.end());
        if (edgesOfCrossing == edges) {
            return true;
        }
    }
    return false;
}


bool
NBNode::forbidsPedestriansAfter(std::vector<std::pair<NBEdge*, bool> > normalizedLanes, int startIndex) {
    for (int i = startIndex; i < (int)normalizedLanes.size(); ++i) {
        if (!normalizedLanes[i].second) {
            return true;
        }
    }
    return false;
}


void
NBNode::buildCrossingsAndWalkingAreas() {
    buildCrossings();
    buildWalkingAreas(OptionsCont::getOptions().getInt("junctions.corner-detail"),
                      OptionsCont::getOptions().getFloat("walkingareas.join-dist"));
    buildCrossingOutlines();
    // ensure that all crossings are properly connected
    bool recheck = myCrossings.size() > 0;
    while (recheck) {
        recheck = false;
        std::set<std::string> waIDs;
        int numSidewalks = 0;
        for (WalkingArea& wa : myWalkingAreas) {
            waIDs.insert(wa.id);
            numSidewalks += (int)(wa.prevSidewalks.size() + wa.nextSidewalks.size());
        }
        if (numSidewalks < 2) {
            // all crossings are invalid if there are fewer than 2 sidewalks involved
            waIDs.clear();
        }
        for (auto& crossing : myCrossings) {
            if (waIDs.count(crossing->prevWalkingArea) == 0 || waIDs.count(crossing->nextWalkingArea) == 0 || !crossing->valid) {
                if (crossing->valid) {
                    WRITE_WARNINGF(TL("Discarding invalid crossing '%' at junction '%' with edges [%] (no walkingarea found)."),
                                   crossing->id, getID(), toString(crossing->edges));
                    recheck = true;
                }
                for (auto waIt = myWalkingAreas.begin(); waIt != myWalkingAreas.end();) {
                    WalkingArea& wa = *waIt;
                    std::vector<std::string>::iterator it_nc = std::find(wa.nextCrossings.begin(), wa.nextCrossings.end(), crossing->id);
                    if (it_nc != wa.nextCrossings.end()) {
                        wa.nextCrossings.erase(it_nc);
                    }
                    if (wa.prevSidewalks.size() + wa.nextSidewalks.size() + wa.nextCrossings.size() + wa.prevCrossings.size() < 2) {
                        waIt = myWalkingAreas.erase(waIt);
                        recheck = true;
                    } else {
                        waIt++;
                    }
                }
                crossing->valid = false;
                crossing->prevWalkingArea = "";
                crossing->nextWalkingArea = "";
            }
        }
    }
}


std::vector<NBNode::Crossing*>
NBNode::getCrossings() const {
    std::vector<Crossing*> result;
    for (auto& c : myCrossings) {
        if (c->valid) {
            result.push_back(c.get());
        }
    }
    //if (myCrossings.size() > 0) {
    //    std::cout << "valid crossings at " << getID() << "\n";
    //    for (std::vector<NBNode::Crossing*>::const_iterator it = result.begin(); it != result.end(); ++it) {
    //        std::cout << "  " << toString((*it)->edges) << "\n";
    //    }
    //}
    return result;
}


void
NBNode::discardAllCrossings(bool rejectAll) {
    myCrossings.clear();
    // also discard all further crossings
    if (rejectAll) {
        myDiscardAllCrossings = true;
    }
}


void
NBNode::discardWalkingareas() {
    myWalkingAreas.clear();
}


double
NBNode::buildInnerEdges() {
    // myDisplacementError is computed during this operation. reset first
    myDisplacementError = 0.;
    // build inner edges for vehicle movements across the junction
    int noInternalNoSplits = 0;
    for (const NBEdge* const edge : myIncomingEdges) {
        for (const NBEdge::Connection& con : edge->getConnections()) {
            if (con.toEdge == nullptr) {
                continue;
            }
            noInternalNoSplits++;
        }
    }
    int lno = 0;
    int splitNo = 0;
    double maxCrossingSeconds = 0.;
    for (NBEdge* const edge : myIncomingEdges) {
        maxCrossingSeconds = MAX2(maxCrossingSeconds, edge->buildInnerEdges(*this, noInternalNoSplits, lno, splitNo));
    }
    return maxCrossingSeconds;
}


int
NBNode::buildCrossings() {
#ifdef DEBUG_PED_STRUCTURES
    gDebugFlag1 = DEBUGCOND;
#endif
    DEBUGCOUT(gDebugFlag1, "build crossings for " << getID() << ":\n")
    if (myDiscardAllCrossings) {
        myCrossings.clear();
    }
    int index = 0;
    const double defaultWidth = OptionsCont::getOptions().getFloat("default.crossing-width");
    for (auto& c : myCrossings) {
        c->valid = true;
        if (!isTLControlled()) {
            c->tlID = ""; // reset for Netedit, set via setCrossingTLIndices()
        }
        c->id = ":" + getID() + "_c" + toString(index++);
        c->width = (c->customWidth == NBEdge::UNSPECIFIED_WIDTH) ? defaultWidth : c->customWidth;
        // reset fields, so repeated computation (Netedit) will successfully perform the checks
        // in buildWalkingAreas (split crossings) and buildInnerEdges (sanity check)
        c->nextWalkingArea = "";
        c->prevWalkingArea = "";
        EdgeVector& edges = c->edges;
        DEBUGCOUT(gDebugFlag1, "  crossing=" << c->id << " edges=" << toString(edges))
        // sorting the edges in the right way is imperative. We want to sort
        // them by getAngleAtNodeToCenter() but need to be extra carefull to avoid wrapping around 0 somewhere in between
        std::sort(edges.begin(), edges.end(), NBContHelper::edge_by_angle_to_nodeShapeCentroid_sorter(this));
        DEBUGCOUT(gDebugFlag1, " sortedEdges=" << toString(edges) << "\n")
        // rotate the edges so that the largest relative angle difference comes at the end
        std::vector<double> rawAngleDiffs;
        double maxAngleDiff = 0;
        int maxAngleDiffIndex = 0; // index before maxDist
        for (int i = 0; i < (int) edges.size(); i++) {
            double diff = NBHelpers::relAngle(edges[i]->getAngleAtNodeToCenter(this),
                                              edges[(i + 1) % edges.size()]->getAngleAtNodeToCenter(this));
            if (diff < 0) {
                diff += 360;
            }
            const double rawDiff = NBHelpers::relAngle(
                                       edges[i]->getAngleAtNodeNormalized(this),
                                       edges[(i + 1) % edges.size()]->getAngleAtNodeNormalized(this));
            rawAngleDiffs.push_back(fabs(rawDiff));

            DEBUGCOUT(gDebugFlag1, "   i=" << i << " a1=" << edges[i]->getAngleAtNodeToCenter(this) << " a2=" << edges[(i + 1) % edges.size()]->getAngleAtNodeToCenter(this) << " diff=" << diff << "\n")
            if (diff > maxAngleDiff) {
                maxAngleDiff = diff;
                maxAngleDiffIndex = i;
            }
        }
        if (maxAngleDiff > 2 && maxAngleDiff < 360 - 2) {
            // if the angle differences is too small, we better not rotate
            std::rotate(edges.begin(), edges.begin() + (maxAngleDiffIndex + 1) % edges.size(), edges.end());
            DEBUGCOUT(gDebugFlag1, " rotatedEdges=" << toString(edges))
        }
        bool diagonalCrossing = false;
        std::sort(rawAngleDiffs.begin(), rawAngleDiffs.end());
        if (rawAngleDiffs.size() >= 2 && rawAngleDiffs[rawAngleDiffs.size() - 2] > 30) {
            diagonalCrossing = true;
#ifdef DEBUG_PED_STRUCTURES
            if (gDebugFlag1) {
                std::cout << " detected pedScramble " << c->id << " edges=" << toString(edges) << " rawDiffs=" << toString(rawAngleDiffs) << "\n";
                for (auto e : edges) {
                    std::cout << "  e=" << e->getID()
                              << " aC=" << e->getAngleAtNodeToCenter(this)
                              << " a=" << e->getAngleAtNode(this)
                              << " aN=" << e->getAngleAtNodeNormalized(this)
                              << "\n";
                }
            }
#endif
        }
        // reverse to get them in CCW order (walking direction around the node)
        std::reverse(edges.begin(), edges.end());
        // compute shape
        c->shape.clear();
        const int begDir = (edges.front()->getFromNode() == this ? FORWARD : BACKWARD);
        const int endDir = (edges.back()->getToNode() == this ? FORWARD : BACKWARD);
        int firstNonPedLane = edges.front()->getFirstNonPedestrianLaneIndex(begDir);
        int lastNonPedLane = edges.back()->getFirstNonPedestrianLaneIndex(endDir);
        DEBUGCOUT(gDebugFlag1, " finalEdges=" << toString(edges) << " firstNonPedLane=" << firstNonPedLane << " lastNonPedLane=" << lastNonPedLane << "\n")
        if (firstNonPedLane < 0 || lastNonPedLane < 0) {
            // invalid crossing
            WRITE_WARNINGF(TL("Discarding invalid crossing '%' at junction '%' with edges [%] (no vehicle lanes to cross)."), c->id, getID(), toString(c->edges));
            c->valid = false;
            // compute surrogate shape to make it visible in netedit
            firstNonPedLane = begDir == FORWARD ? 0 : edges.front()->getNumLanes() - 1;
            lastNonPedLane = endDir == FORWARD ? 0 : edges.back()->getNumLanes() - 1;
        }
        if (c->customShape.size() != 0) {
            c->shape = c->customShape;
        } else {
            NBEdge::Lane crossingBeg = edges.front()->getLanes()[firstNonPedLane];
            NBEdge::Lane crossingEnd = edges.back()->getLanes()[lastNonPedLane];
            crossingBeg.width = (crossingBeg.width == NBEdge::UNSPECIFIED_WIDTH ? SUMO_const_laneWidth : crossingBeg.width);
            crossingEnd.width = (crossingEnd.width == NBEdge::UNSPECIFIED_WIDTH ? SUMO_const_laneWidth : crossingEnd.width);
            crossingBeg.shape.move2side(begDir * crossingBeg.width / 2);
            crossingEnd.shape.move2side(endDir * crossingEnd.width / 2);
            double offset = c->width / 2;
            patchOffset_pathAcrossStreet(offset);
            crossingBeg.shape.extrapolate(offset);
            crossingEnd.shape.extrapolate(offset);
            // check if after all changes shape are NAN (in these case, discard)
            if (crossingBeg.shape.isNAN() || crossingEnd.shape.isNAN()) {
                WRITE_WARNINGF(TL("Discarding invalid crossing '%' at junction '%' with edges [%] (invalid shape)."), c->id, getID(), toString(c->edges));
                c->valid = false;
            } else {
                c->shape.push_back(crossingBeg.shape[begDir == FORWARD ? 0 : -1]);
                c->shape.push_back(crossingEnd.shape[endDir == FORWARD ? -1 : 0]);
            }
            if (diagonalCrossing) {
                c->shape.move2side(-c->width);
            }
        }
    }
    return index;
}


void
NBNode::patchOffset_pathAcrossStreet(double& offset) {
    if (myCrossings.size() == 1 && myAllEdges.size() >= 3) {
        EdgeVector nonPedIncoming;
        EdgeVector nonPedOutgoing;
        EdgeVector pedIncoming;
        EdgeVector pedOutgoing;
        for (NBEdge* e : getIncomingEdges()) {
            if (e->getPermissions() != SVC_PEDESTRIAN) {
                nonPedIncoming.push_back(e);
            } else {
                pedIncoming.push_back(e);
            }
        }
        for (NBEdge* e : getOutgoingEdges()) {
            if (e->getPermissions() != SVC_PEDESTRIAN) {
                nonPedOutgoing.push_back(e);
            } else {
                pedOutgoing.push_back(e);
            }
        }
        if (geometryLike(nonPedIncoming, nonPedOutgoing) && (pedIncoming.size() > 0 || pedOutgoing.size() > 0)) {
            double maxAngle = 0;
            const NBEdge* in = nonPedIncoming.front();
            const NBEdge* out = nonPedOutgoing.front();
            if (nonPedIncoming.size() == 1) {
                maxAngle = fabs(NBHelpers::relAngle(in->getAngleAtNode(this), out->getAngleAtNode(this)));
            } else {
                for (const NBEdge* const in2 : nonPedIncoming) {
                    double minAngle = 180;
                    for (const NBEdge* const out2 : nonPedOutgoing) {
                        double angle = fabs(NBHelpers::relAngle(in2->getAngleAtNode(this), out2->getAngleAtNode(this)));
                        if (angle < minAngle) {
                            minAngle = angle;
                            in = in2;
                            out = out2;
                        }
                    }
                    maxAngle = MAX2(maxAngle, minAngle);
                }
            }
            // changing the offset only handles the simple case where the road stays straight
            if (maxAngle < 15) {
                const int inLane = in->getFirstNonPedestrianLaneIndex(FORWARD);
                const int outLane = out->getFirstNonPedestrianLaneIndex(FORWARD);
                if (inLane >= 0 && outLane >= 0) {
                    const Position& p0 = in->getLaneShape(inLane).back();
                    const Position& p1 = out->getLaneShape(outLane).front();
                    PositionVector road;
                    road.push_back(p0);
                    road.push_back(p1);
                    Position mid = (p0 + p1) / 2;
                    double maxPathDist = 0;
                    for (NBEdge* e : pedIncoming) {
                        const Position roadPos = road.positionAtOffset2D(road.nearest_offset_to_point2D(e->getLaneShape(0).back()));
                        maxPathDist = MAX2(maxPathDist, mid.distanceTo2D(roadPos));
                    }
                    for (NBEdge* e : pedOutgoing) {
                        const Position roadPos = road.positionAtOffset2D(road.nearest_offset_to_point2D(e->getLaneShape(0).front()));
                        maxPathDist = MAX2(maxPathDist, mid.distanceTo2D(roadPos));
                    }
                    // if the junction is stretched, the crossing should stay close to the paths
                    if (maxPathDist < MAX2(myCrossings.front()->width, 4.0)) {
                        offset = p0.distanceTo2D(p1) / 2;
                    }
                }
            }
        }
    }
}


void
NBNode::buildWalkingAreas(int cornerDetail, double joinMinDist) {
#ifdef DEBUG_PED_STRUCTURES
    gDebugFlag1 = DEBUGCOND;
#endif
    int index = 0;
    myWalkingAreas.clear();
    DEBUGCOUT(gDebugFlag1, "build walkingAreas for " << getID() << ":\n")
    if (myAllEdges.size() == 0) {
        return;
    }
    EdgeVector allEdges = getEdgesSortedByAngleAtNodeCenter();
    // shapes are all pointing away from the intersection
    std::vector<std::pair<NBEdge*, NBEdge::Lane> > normalizedLanes;
    for (EdgeVector::const_iterator it = allEdges.begin(); it != allEdges.end(); ++it) {
        NBEdge* edge = *it;
        const std::vector<NBEdge::Lane>& lanes = edge->getLanes();
        std::vector<NBEdge::Lane> tmp;
        bool hadSidewalk = false;
        bool hadNonSidewalk = false;
        for (int i = 0; i < (int)lanes.size(); i++) {
            NBEdge::Lane l = lanes[i];
            const bool sidewalk = (l.permissions & SVC_PEDESTRIAN) != 0;
            if (sidewalk) {
                if (hadSidewalk && hadNonSidewalk) {
                    if (edge->getFromNode() == this) {
                        WRITE_WARNINGF(TL("Ignoring additional sidewalk lane % on edge '%' for walkingareas."),
                                       i, edge->getID());
                    }
                    continue;
                }
                hadSidewalk = true;
            } else {
                hadNonSidewalk = true;
            }
            tmp.push_back(l);
        }
        if (edge->getFromNode() == this) {
            std::reverse(tmp.begin(), tmp.end());
        } else {
            for (NBEdge::Lane& l : tmp) {
                l.shape = l.shape.reverse();
            }
        }
        for (NBEdge::Lane& l : tmp) {
            l.shape = l.shape.getSubpartByIndex(0, 2);
            l.width = (l.width == NBEdge::UNSPECIFIED_WIDTH ? SUMO_const_laneWidth : l.width);
            normalizedLanes.push_back(std::make_pair(edge, l));
        }
    }
    //if (gDebugFlag1) std::cout << "  normalizedLanes=" << normalizedLanes.size() << "\n";
    // collect [start,count[ indices in normalizedLanes that belong to a walkingArea
    std::vector<std::pair<int, int> > waIndices;
    int start = -1;
    NBEdge* prevEdge = normalizedLanes.back().first;
    for (int i = 0; i < (int)normalizedLanes.size(); ++i) {
        NBEdge* edge = normalizedLanes[i].first;
        NBEdge::Lane& l = normalizedLanes[i].second;
        if (start == -1) {
            if ((l.permissions & SVC_PEDESTRIAN) != 0) {
                start = i;
            }
        } else {
            if ((l.permissions & SVC_PEDESTRIAN) == 0
                    || crossingBetween(edge, prevEdge)
                    || alreadyConnectedPaths(edge, prevEdge, joinMinDist)
                    || crossesFringe(edge, prevEdge)
               ) {
                waIndices.push_back(std::make_pair(start, i - start));
                if ((l.permissions & SVC_PEDESTRIAN) != 0) {
                    start = i;
                } else {
                    start = -1;
                }

            }
        }
        DEBUGCOUT(gDebugFlag1, "     i=" << i << " edge=" << edge->getID() << " start=" << start << " ped=" << ((l.permissions & SVC_PEDESTRIAN) != 0)
                  << " waI=" << waIndices.size() << " crossingBetween=" << crossingBetween(edge, prevEdge) << "\n")
        prevEdge = edge;
    }
    // deal with wrap-around issues
    if (start != - 1) {
        const int waNumLanes = (int)normalizedLanes.size() - start;
        if (waIndices.size() == 0) {
            waIndices.push_back(std::make_pair(start, waNumLanes));
            DEBUGCOUT(gDebugFlag1, "  single wa, end at wrap-around\n")
        } else {
            if (waIndices.front().first == 0) {
                NBEdge* edge = normalizedLanes.front().first;
                if (crossingBetween(edge, normalizedLanes.back().first)
                        || crossesFringe(edge, normalizedLanes.back().first)) {
                    // do not wrap-around (see above)
                    waIndices.push_back(std::make_pair(start, waNumLanes));
                    DEBUGCOUT(gDebugFlag1, "  do not wrap around\n")
                } else {
                    // first walkingArea wraps around
                    waIndices.front().first = start;
                    waIndices.front().second = waNumLanes + waIndices.front().second;
                    DEBUGCOUT(gDebugFlag1, "  wrapping around\n")
                }
            } else {
                // last walkingArea ends at the wrap-around
                waIndices.push_back(std::make_pair(start, waNumLanes));
                DEBUGCOUT(gDebugFlag1, "  end at wrap-around\n")
            }
        }
    }
#ifdef DEBUG_PED_STRUCTURES
    if (gDebugFlag1) {
        std::cout << "  normalizedLanes=" << normalizedLanes.size() << " waIndices:\n";
        for (int i = 0; i < (int)waIndices.size(); ++i) {
            std::cout << "   " << waIndices[i].first << ", " << waIndices[i].second << "\n";
        }
    }
#endif
    // build walking areas connected to a sidewalk
    for (int i = 0; i < (int)waIndices.size(); ++i) {
        const bool buildExtensions = waIndices[i].second != (int)normalizedLanes.size();
        int startIdx = waIndices[i].first;
        const int prev = startIdx > 0 ? startIdx - 1 : (int)normalizedLanes.size() - 1;
        const int count = waIndices[i].second;
        const int end = (startIdx + count) % normalizedLanes.size();
        int lastIdx = (startIdx + count - 1) % normalizedLanes.size();

        WalkingArea wa(":" + getID() + "_w" + toString(index++), 1);
        DEBUGCOUT(gDebugFlag1, "build walkingArea " << wa.id << " start=" << startIdx << " end=" << end << " count=" << count << " prev=" << prev << ":\n")
        double endCrossingWidth = 0;
        double startCrossingWidth = 0;
        PositionVector endCrossingShape;
        PositionVector startCrossingShape;
        // check for connected crossings
        bool connectsCrossing = false;
        bool crossingNearSidewalk = false;
        int numCrossings = 0;
        std::vector<Position> connectedPoints;
        for (auto c : getCrossings()) {
            DEBUGCOUT(gDebugFlag1, "  crossing=" << c->id << " sortedEdges=" << toString(c->edges) << "\n")
            if (c->edges.back() == normalizedLanes[end].first
                    && (normalizedLanes[end].second.permissions & SVC_PEDESTRIAN) == 0) {
                // crossing ends
                if (c->nextWalkingArea != "") {
                    WRITE_WARNINGF(TL("Invalid pedestrian topology at junction '%'; crossing '%' targets '%' and '%'."),
                                   getID(), c->id, c->nextWalkingArea, wa.id);
                    c->valid = false;
                }
                c->nextWalkingArea = wa.id;
                wa.prevCrossings.push_back(c->id);
                if ((int)c->edges.size() < wa.minPrevCrossingEdges) {
                    // if there are multiple crossings, use the shape of the one that crosses fewer edges
                    endCrossingWidth = c->width;
                    endCrossingShape = c->shape;
                    wa.width = MAX2(wa.width, endCrossingWidth);
                    connectsCrossing = true;
                    connectedPoints.push_back(c->shape[-1]);
                    wa.minPrevCrossingEdges = (int)c->edges.size();
                    numCrossings++;
                    if (normalizedLanes[lastIdx].second.shape[0].distanceTo2D(connectedPoints.back()) < endCrossingWidth) {
                        crossingNearSidewalk = true;
                        DEBUGCOUT(gDebugFlag1, "    nearSidewalk\n")
                    }
                }
                DEBUGCOUT(gDebugFlag1, "    crossing " << c->id << " ends\n")
            }
            if (c->edges.front() == normalizedLanes[prev].first
                    && (normalizedLanes[prev].second.permissions & SVC_PEDESTRIAN) == 0) {
                // crossing starts
                if (c->prevWalkingArea != "") {
                    WRITE_WARNINGF(TL("Invalid pedestrian topology at junction '%'; crossing '%' is targeted by '%' and '%'."),
                                   getID(), c->id, c->prevWalkingArea, wa.id);
                    c->valid = false;
                }
                if (c->valid && std::find(wa.prevCrossings.begin(), wa.prevCrossings.end(), c->id) != wa.prevCrossings.end()) {
                    WRITE_WARNINGF(TL("Invalid pedestrian topology at junction '%'; crossing '%' starts and ends at walkingarea '%'."),
                                   getID(), c->id, wa.id);
                    c->valid = false;
                }
                c->prevWalkingArea = wa.id;
                wa.nextCrossings.push_back(c->id);
                if ((int)c->edges.size() < wa.minNextCrossingEdges) {
                    // if there are multiple crossings, use the shape of the one that crosses fewer edges
                    startCrossingWidth = c->width;
                    startCrossingShape = c->shape;
                    wa.width = MAX2(wa.width, startCrossingWidth);
                    connectsCrossing = true;
                    connectedPoints.push_back(c->shape[0]);
                    wa.minNextCrossingEdges = (int)c->edges.size();
                    numCrossings++;
                    if (normalizedLanes[startIdx].second.shape[0].distanceTo2D(connectedPoints.back()) < startCrossingWidth) {
                        crossingNearSidewalk = true;
                        DEBUGCOUT(gDebugFlag1, "    nearSidewalk\n")
                    }
                }
                DEBUGCOUT(gDebugFlag1, "    crossing " << c->id << " starts\n")
            }
            DEBUGCOUT(gDebugFlag1, "  check connections to crossing " << c->id
                      << " cFront=" << c->edges.front()->getID() << " cBack=" << c->edges.back()->getID()
                      << " wEnd=" << normalizedLanes[end].first->getID() << " wStart=" << normalizedLanes[startIdx].first->getID()
                      << " wStartPrev=" << normalizedLanes[prev].first->getID()
                      << "\n")
        }
        if (count < 2 && !connectsCrossing) {
            // not relevant for walking
            DEBUGCOUT(gDebugFlag1, "    not relevant for walking: count=" << count << " connectsCrossing=" << connectsCrossing << "\n")
            continue;
        }
        // build shape and connections
        std::set<const NBEdge*, ComparatorIdLess>& connected = wa.refEdges;
        for (int j = 0; j < count; ++j) {
            const int nlI = (startIdx + j) % normalizedLanes.size();
            NBEdge* edge = normalizedLanes[nlI].first;
            NBEdge::Lane l = normalizedLanes[nlI].second;
            wa.width = MAX2(wa.width, l.width);
            if (connected.count(edge) == 0) {
                if (edge->getFromNode() == this) {
                    wa.nextSidewalks.push_back(edge->getSidewalkID());
                    connectedPoints.push_back(edge->getLaneShape(0)[0]);
                } else {
                    wa.prevSidewalks.push_back(edge->getSidewalkID());
                    connectedPoints.push_back(edge->getLaneShape(0)[-1]);
                }
                DEBUGCOUT(gDebugFlag1, "    connectedEdge=" << edge->getID() << " connectedPoint=" << connectedPoints.back() << "\n")
                connected.insert(edge);
            }
            l.shape.move2side(-l.width / 2);
            wa.shape.push_back_noDoublePos(l.shape[0]);
            l.shape.move2side(l.width);
            wa.shape.push_back(l.shape[0]);
        }
        if (buildExtensions) {
            // extension at starting crossing
            if (startCrossingShape.size() > 0) {
                startCrossingShape.move2side(startCrossingWidth / 2);
                wa.shape.push_front_noDoublePos(startCrossingShape[0]); // right corner
                startCrossingShape.move2side(-startCrossingWidth);
                wa.shape.push_front_noDoublePos(startCrossingShape[0]); // left corner goes first
                DEBUGCOUT(gDebugFlag1, "  extension at startCrossingShape=" << endCrossingShape << " waShape=" << wa.shape << "\n")
            }
            // extension at ending crossing
            if (endCrossingShape.size() > 0) {
                endCrossingShape.move2side(endCrossingWidth / 2);
                wa.shape.push_back_noDoublePos(endCrossingShape[-1]);
                endCrossingShape.move2side(-endCrossingWidth);
                wa.shape.push_back_noDoublePos(endCrossingShape[-1]);
                DEBUGCOUT(gDebugFlag1, "  extension at endCrossingShape=" << endCrossingShape << " waShape=" << wa.shape << "\n")
            }
        }
        if (connected.size() == 2 && !connectsCrossing && wa.nextSidewalks.size() == 1 && wa.prevSidewalks.size() == 1
                && normalizedLanes.size() == 2) {
            // do not build a walkingArea since a normal connection exists
            const NBEdge* e1 = *connected.begin();
            const NBEdge* e2 = *(++connected.begin());
            if (e1->hasConnectionTo(e2, 0, 0) || e2->hasConnectionTo(e1, 0, 0)) {
                DEBUGCOUT(gDebugFlag1, "    not building a walkingarea since normal connections exist\n")
                continue;
            }
        }
        if (count == (int)normalizedLanes.size()) {
            // junction is covered by the whole walkingarea
            wa.shape = myPoly;
            // increase walking width if the walkingare is wider than a single lane
            for (const NBEdge* in : myIncomingEdges) {
                for (const NBEdge* out : myOutgoingEdges) {
                    if (in->getFromNode() == out->getToNode() && in->getInnerGeometry().reverse() == out->getInnerGeometry()
                            && (in->getPermissions() & SVC_PEDESTRIAN)
                            && (out->getPermissions() & SVC_PEDESTRIAN)) {
                        // doesn't catch all cases but probably most
                        wa.width = MAX2(wa.width, in->getTotalWidth() + out->getTotalWidth());
                    }
                }
            }
        } else if (cornerDetail > 0) {
            // build smooth inner curve (optional)
            int smoothEnd = end;
            int smoothPrev = prev;
            // extend to green verge
            if (endCrossingWidth > 0 && normalizedLanes[smoothEnd].second.permissions == 0) {
                smoothEnd = (smoothEnd + 1) % normalizedLanes.size();
            }
            if (startCrossingWidth > 0 && normalizedLanes[smoothPrev].second.permissions == 0) {
                if (smoothPrev == 0) {
                    smoothPrev = (int)normalizedLanes.size() - 1;
                } else {
                    smoothPrev--;
                }
            }
            PositionVector begShape = normalizedLanes[smoothEnd].second.shape;
            begShape = begShape.reverse();
            double shiftBegExtra = 0;
            double shiftEndExtra = 0;
            if (lastIdx == startIdx) {
                lastIdx = (startIdx + 1) % normalizedLanes.size();
                DEBUGCOUT(gDebugFlag1, "    new lastIdx=" << lastIdx << " startEdge=" << normalizedLanes[startIdx].first->getID() << " lastEdge=" << normalizedLanes[lastIdx].first->getID() << "\n")
                if (normalizedLanes[startIdx].first == normalizedLanes[lastIdx].first) {
                    lastIdx = startIdx;
                    startIdx--;
                    if (startIdx < 0) {
                        startIdx = (int)normalizedLanes.size() - 1;
                    }
                    DEBUGCOUT(gDebugFlag1, "    new startIdx=" << startIdx << " startEdge=" << normalizedLanes[startIdx].first->getID() << " lastEdge=" << normalizedLanes[lastIdx].first->getID() << "\n")
                    shiftEndExtra += OptionsCont::getOptions().getFloat("default.sidewalk-width");
                } else {
                    shiftBegExtra += OptionsCont::getOptions().getFloat("default.sidewalk-width");
                }
            }
            PositionVector begShapeOuter = normalizedLanes[lastIdx].second.shape;
            begShapeOuter = begShapeOuter.reverse();
            //begShape.extrapolate(endCrossingWidth);
            begShape.move2side(normalizedLanes[smoothEnd].second.width / 2);
            begShapeOuter.move2side(normalizedLanes[lastIdx].second.width / 2 + shiftBegExtra);
            PositionVector endShape = normalizedLanes[smoothPrev].second.shape;
            PositionVector endShapeOuter = normalizedLanes[startIdx].second.shape;;
            endShape.move2side(normalizedLanes[smoothPrev].second.width / 2);
            endShapeOuter.move2side(normalizedLanes[startIdx].second.width / 2 + shiftEndExtra);
            //endShape.extrapolate(startCrossingWidth);
            PositionVector curve;
            if (count != (int)normalizedLanes.size() || count == 2) {
                const double angle = GeomHelper::angleDiff(begShape.angleAt2D(-2), endShape.angleAt2D(0));
                if (count == 1 && angle > 0 && crossingNearSidewalk && numCrossings < 2) {
                    // do not build smooth shape for an unconnected left turn
                    // (the walkingArea would get bigger without a reason to
                    // walk there)
                } else if ((normalizedLanes[smoothEnd].first->getPermissions() & normalizedLanes[smoothPrev].first->getPermissions() &
                            ~(SVC_PEDESTRIAN | SVC_RAIL_CLASSES)) != 0) {
                    DEBUGCOUT(gDebugFlag1, "   traffic curve\n")
                    curve = computeSmoothShape(begShape, endShape, cornerDetail + 2, false, 25, 25, gDebugFlag1 ? this : nullptr);
                    if (curve.length2D() - begShape.back().distanceTo2D(endShape.front()) > 5) {
                        DEBUGCOUT(gDebugFlag1, "   reduceBulge directLength=" << begShape.back().distanceTo2D(endShape.front())
                                  << " curveLength=" << curve.length2D()
                                  << " delta=" << curve.length2D() - begShape.back().distanceTo2D(endShape.front())
                                  << "\n")
                        curve = computeSmoothShape(begShape, endShape, cornerDetail + 2, false, 25, 25, nullptr, AVOID_WIDE_LEFT_TURN | AVOID_INTERSECTING_LEFT_TURNS);
                    }
                } else {
                    DEBUGCOUT(gDebugFlag1, "   nonTraffic curve\n")
                    const double extend = MIN2(10.0, begShape.back().distanceTo2D(endShape.front()) / 2);
                    curve = computeSmoothShape(begShape, endShape, cornerDetail + 2, false, extend, extend, nullptr, FOUR_CONTROL_POINTS);
                }
                if (curve.size() > 2) {
                    curve.erase(curve.begin());
                    curve.pop_back();
                    if (endCrossingWidth > 0) {
                        wa.shape.pop_back();
                    }
                    if (startCrossingWidth > 0) {
                        wa.shape.erase(wa.shape.begin());
                    }
                    if (count == (int)normalizedLanes.size()) {
                        curve = curve.reverse();
                    }
                    wa.shape.append(curve, 0);
                }
                DEBUGCOUT(gDebugFlag1, " end=" << smoothEnd << " prev=" << smoothPrev
                          << " endCrossingWidth=" << endCrossingWidth << " startCrossingWidth=" << startCrossingWidth
                          << "  begShape=" << begShape << " endShape=" << endShape << " smooth curve=" << curve
                          << "  begShapeOuter=" << begShapeOuter << " endShapeOuter=" << endShapeOuter
                          << "  waShape=" << wa.shape
                          << "\n")
            }
            if (curve.size() > 2 && (count == 2 || (count == 1 && numCrossings > 0))) {
                const double innerDist = begShape.back().distanceTo2D(endShape[0]);
                const double outerDist = begShapeOuter.back().distanceTo2D(endShapeOuter[0]);
                DEBUGCOUT(gDebugFlag1, " innerDist=" << innerDist << " outerDist=" << outerDist << "\n")
                if (outerDist > innerDist) {
                    // we also need a rounded outer curve (unless we have only a single walkingarea)
                    const double extend = MIN2(10.0, begShapeOuter.back().distanceTo2D(endShapeOuter.front()) / 2);
                    curve = computeSmoothShape(begShapeOuter, endShapeOuter, cornerDetail + 2, false, extend, extend, nullptr);
                    if (curve.length2D() - begShapeOuter.back().distanceTo2D(endShapeOuter.front()) > 5) {
                        DEBUGCOUT(gDebugFlag1, "   reduceBulge directLength=" << begShapeOuter.back().distanceTo2D(endShapeOuter.front())
                                  << " curveLength=" << curve.length2D()
                                  << " delta=" << curve.length2D() - begShapeOuter.back().distanceTo2D(endShapeOuter.front())
                                  << "\n")
                        curve = computeSmoothShape(begShapeOuter, endShapeOuter, cornerDetail + 2, false, 25, 25, nullptr, AVOID_WIDE_LEFT_TURN | AVOID_INTERSECTING_LEFT_TURNS);
                    }
                    curve = curve.reverse();
                    // keep the points in case of extraShift
                    if (shiftBegExtra != 0) {
                        curve.push_front_noDoublePos(wa.shape[1]);
                        curve.push_back_noDoublePos(wa.shape[2]);
                    } else if (shiftEndExtra != 0) {
                        curve.push_back_noDoublePos(wa.shape[1]);
                        curve.push_back_noDoublePos(wa.shape[2]);
                    }
                    DEBUGCOUT(gDebugFlag1, " outerCurveRaw=" << curve << " wa1=" << wa.shape[1] << " wa2=" << wa.shape[2] << "\n")
                    wa.shape.erase(wa.shape.begin() + 1, wa.shape.begin() + 3);
                    wa.shape.insert(wa.shape.begin() + 1, curve.begin(), curve.end());
                    DEBUGCOUT(gDebugFlag1, " outerCurve=" << curve << "\n")
                }
            }
        }
        // apply custom shapes
        if (myWalkingAreaCustomShapes.size() > 0) {
            for (auto wacs : myWalkingAreaCustomShapes) {
                // every edge in wasc.edges must be part of connected
                if ((wacs.shape.size() != 0 || wacs.width != NBEdge::UNSPECIFIED_WIDTH) && includes(connected, wacs.edges)) {
                    if (wacs.shape.size() != 0) {
                        wa.shape = wacs.shape;
                    }
                    if (wacs.width != NBEdge::UNSPECIFIED_WIDTH) {
                        wa.width = wacs.width;
                    }
                    wa.hasCustomShape = true;
                }
            }
        }
        // determine length (average of all possible connections)
        double lengthSum = 0;
        int combinations = 0;
        for (std::vector<Position>::const_iterator it1 = connectedPoints.begin(); it1 != connectedPoints.end(); ++it1) {
            for (std::vector<Position>::const_iterator it2 = connectedPoints.begin(); it2 != connectedPoints.end(); ++it2) {
                const Position& p1 = *it1;
                const Position& p2 = *it2;
                if (p1 != p2) {
                    lengthSum += p1.distanceTo2D(p2);
                    combinations += 1;
                }
            }
        }
        DEBUGCOUT(gDebugFlag1, "  combinations=" << combinations << " connectedPoints=" << connectedPoints << "\n")
        wa.length = POSITION_EPS;
        if (combinations > 0) {
            wa.length = MAX2(POSITION_EPS, lengthSum / combinations);
        }
        myWalkingAreas.push_back(wa);
    }
    // build walkingAreas between split crossings
    std::vector<Crossing*> validCrossings = getCrossings();
    for (std::vector<Crossing*>::iterator it = validCrossings.begin(); it != validCrossings.end(); ++it) {
        Crossing& prev = **it;
        Crossing& next = (it !=  validCrossings.begin() ? **(it - 1) :** (validCrossings.end() - 1));
        DEBUGCOUT(gDebugFlag1, "  checkIntermediate: prev=" << prev.id << " next=" << next.id << " prev.nextWA=" << prev.nextWalkingArea << " next.prevWA=" << next.prevWalkingArea << "\n")
        if (prev.nextWalkingArea == "") {
            if (next.prevWalkingArea != "" || &prev == &next) {
                WRITE_WARNINGF(TL("Invalid pedestrian topology: crossing '%' across [%] has no target."), prev.id, toString(prev.edges));
                prev.valid = false;
                continue;
            }
            WalkingArea wa(":" + getID() + "_w" + toString(index++), prev.width);
            prev.nextWalkingArea = wa.id;
            wa.nextCrossings.push_back(next.id);
            next.prevWalkingArea = wa.id;
            // back of previous crossing
            PositionVector tmp = prev.shape;
            tmp.move2side(-prev.width / 2);
            wa.shape.push_back(tmp[-1]);
            tmp.move2side(prev.width);
            wa.shape.push_back(tmp[-1]);
            // front of next crossing
            tmp = next.shape;
            tmp.move2side(prev.width / 2);
            wa.shape.push_back(tmp[0]);
            tmp.move2side(-prev.width);
            wa.shape.push_back(tmp[0]);
            wa.refEdges.insert(prev.edges.begin(), prev.edges.end());
            wa.refEdges.insert(next.edges.begin(), next.edges.end());
            // apply custom shapes
            if (myWalkingAreaCustomShapes.size() > 0) {
                for (auto wacs : myWalkingAreaCustomShapes) {
                    // every edge in wacs.edges must be part of crossed
                    if (wacs.shape.size() != 0 && wacs.edges.size() > 1 && includes(wa.refEdges, wacs.edges)) {
                        wa.shape = wacs.shape;
                        wa.hasCustomShape = true;
                    }
                }
            }
            // length (special case)
            wa.length = MAX2(POSITION_EPS, prev.shape.back().distanceTo2D(next.shape.front()));
            myWalkingAreas.push_back(wa);
            DEBUGCOUT(gDebugFlag1, "     build wa=" << wa.id << "\n")
        }
    }
}


void
NBNode::buildCrossingOutlines() {
#ifdef DEBUG_CROSSING_OUTLINE
    if (myCrossings.size() > 0) {
        std::cerr << "<add>\n";
    }
#endif
    std::map<std::string, PositionVector> waShapes;
    for (auto wa : myWalkingAreas) {
        waShapes[wa.id] = wa.shape;
    }
    for (auto c : getCrossings()) {
        PositionVector wa1 = waShapes[c->prevWalkingArea];
        PositionVector wa2 = waShapes[c->nextWalkingArea];
        if (wa1.empty() || wa2.empty()) {
            continue;
        }
        wa1.closePolygon();
        wa2.closePolygon();
        PositionVector side1 = c->shape;
        PositionVector side2 = c->shape.reverse();
        side1.move2side(c->width / 2);
        side2.move2side(c->width / 2);
        PositionVector side1default = side1;
        PositionVector side2default = side2;
        side1.extrapolate(POSITION_EPS);
        side2.extrapolate(c->width);
        side1 = cutAtShapes(side1, wa1, wa2, side1default);
        side2 = cutAtShapes(side2, wa1, wa2, side2default);
        PositionVector side1ex = side1;
        PositionVector side2ex = side2;
        side1ex.extrapolate(POSITION_EPS);
        side2ex.extrapolate(side2 == side2default ? c->width / 2 : POSITION_EPS);
        PositionVector side3 = cutAtShapes(wa2, side1ex, side2ex, PositionVector());
        PositionVector side4 = cutAtShapes(wa1, side1ex, side2ex, PositionVector());
        c->outlineShape = side1;
        c->outlineShape.append(side3, POSITION_EPS);
        c->outlineShape.append(side2, POSITION_EPS);
        c->outlineShape.append(side4, POSITION_EPS);
        c->outlineShape.removeDoublePoints();
        if (c->outlineShape.back().almostSame(c->outlineShape.front())) {
            c->outlineShape.pop_back();
        }
        // DEBUG
#ifdef DEBUG_CROSSING_OUTLINE
        std::cout << "  side1=" << side1 << "\n  side2=" << side2 << "\n  side3=" << side3 << "\n  side4=" << side4 << "\n";
        std::cerr << "<poly id=\"" << c->id << "\" shape=\"" << c->outlineShape << "\" color=\"blue\" lineWidth=\"0.2\" layer=\"100\"/>\n";
#endif
    }
#ifdef DEBUG_CROSSING_OUTLINE
    if (myCrossings.size() > 0) {
        std::cerr << "</add>\n";
    }
#endif
}


PositionVector
NBNode::cutAtShapes(const PositionVector& cut, const PositionVector& border1, const PositionVector& border2, const PositionVector& def) {
    std::vector<double> is1 = cut.intersectsAtLengths2D(border1);
    std::vector<double> is2 = cut.intersectsAtLengths2D(border2);
#ifdef DEBUG_CROSSING_OUTLINE
    std::cout << "is1=" << is1 << " is2=" << is2 << " cut=" << cut << " border1=" << border1 << " border2=" << border2 << "\n";
#endif
    if (is1.size() == 0 && border1.size() == 2) {
        const double d1 = cut.distance2D(border1.front());
        const double d2 = cut.distance2D(border1.back());
        Position closer = d1 < d2 ? border1.front() : border1.back();
        double nOp = cut.nearest_offset_to_point2D(closer, false);
#ifdef DEBUG_CROSSING_OUTLINE
        std::cout << " closer=" << closer << " nOp=" << nOp << "\n";
#endif
        if (nOp <= 2 * POSITION_EPS && cut.back().distanceTo2D(closer) <= 2 * POSITION_EPS) {
            is1.push_back(cut.length2D());
        } else {
            is1.push_back(nOp);
        }
    }
    if (is2.size() == 0 && border2.size() == 2) {
        const double d1 = cut.distance2D(border2.front());
        const double d2 = cut.distance2D(border2.back());
        Position closer = d1 < d2 ? border2.front() : border2.back();
        double nOp = cut.nearest_offset_to_point2D(closer, false);
        if (nOp <= 2 * POSITION_EPS && cut.back().distanceTo2D(closer) <= 2 * POSITION_EPS) {
            is2.push_back(cut.length2D());
        } else {
            is2.push_back(nOp);
        }
    }
    if (is1.size() > 0 && is2.size() > 0) {
        double of1 = VectorHelper<double>::maxValue(is1);
        double of2 = VectorHelper<double>::minValue(is2);
#ifdef DEBUG_CROSSING_OUTLINE
        std::cout << " of1=" << of1 << " of2=" << of2 << "\n";
#endif
        if (of1 > of2) {
            of1 = VectorHelper<double>::maxValue(is2);
            of2 = VectorHelper<double>::minValue(is1);
#ifdef DEBUG_CROSSING_OUTLINE
            std::cout << " of1=" << of1 << " of2=" << of2 << "\n";
#endif
        }
        if (of1 > of2) {
            of2 = VectorHelper<double>::maxValue(is1);
            of1 = VectorHelper<double>::minValue(is2);
#ifdef DEBUG_CROSSING_OUTLINE
            std::cout << " of1=" << of1 << " of2=" << of2 << "\n";
#endif
        }
        assert(of1 <= of2);
        return cut.getSubpart(of1, of2);
    } else {
        return def;
    }
}


bool
NBNode::includes(const std::set<const NBEdge*, ComparatorIdLess>& super,
                 const std::set<const NBEdge*, ComparatorIdLess>& sub) {
    // for some reason std::include does not work reliably
    for (const NBEdge* e : sub) {
        if (super.count(const_cast<NBEdge*>(e)) == 0) {
            return false;
        }
    }
    return true;
}


bool
NBNode::crossingBetween(const NBEdge* e1, const NBEdge* e2) const {
    if (e1 == e2) {
        return false;
    }
    if (myAllEdges.size() > 3) {
        // pedestrian scramble
        return false;
    }
    for (auto c : getCrossings()) {
        const EdgeVector& edges = c->edges;
        EdgeVector::const_iterator it1 = std::find(edges.begin(), edges.end(), e1);
        EdgeVector::const_iterator it2 = std::find(edges.begin(), edges.end(), e2);
        if (it1 != edges.end() && it2 != edges.end()) {
            return true;
        }
    }
    return false;
}


bool
NBNode::alreadyConnectedPaths(const NBEdge* e1, const NBEdge* e2, double dist) const {
    if (e1 == e2) {
        return false;
    }
    if (e1->getPermissions() != SVC_PEDESTRIAN
            || e2->getPermissions() != SVC_PEDESTRIAN) {
        // no paths
        return false;
    }
    if (e1->getFinalLength() > dist &&
            e2->getFinalLength() > dist) {
        // too long
        return false;
    }
    NBNode* other1 = e1->getFromNode() == this ? e1->getToNode() : e1->getFromNode();
    NBNode* other2 = e2->getFromNode() == this ? e2->getToNode() : e2->getFromNode();
    return other1 == other2;
}


bool
NBNode::crossesFringe(const NBEdge* e1, const NBEdge* e2) const {
    return myFringeType != FringeType::DEFAULT
           && myIncomingEdges.size() == 1 && myOutgoingEdges.size() == 1
           && (e1->isTurningDirectionAt(e2) || e2->isTurningDirectionAt(e1));
}


EdgeVector
NBNode::edgesBetween(const NBEdge* e1, const NBEdge* e2) const {
    EdgeVector result;
    EdgeVector::const_iterator it = std::find(myAllEdges.begin(), myAllEdges.end(), e1);
    assert(it != myAllEdges.end());
    NBContHelper::nextCW(myAllEdges, it);
    EdgeVector::const_iterator it_end = std::find(myAllEdges.begin(), myAllEdges.end(), e2);
    assert(it_end != myAllEdges.end());
    while (it != it_end) {
        result.push_back(*it);
        NBContHelper::nextCW(myAllEdges, it);
    }
    return result;
}


void
NBNode::addWalkingAreaShape(EdgeVector edges, const PositionVector& shape, double width) {
    WalkingAreaCustomShape wacs;
    wacs.edges.insert(edges.begin(), edges.end());
    wacs.shape = shape;
    wacs.width = width;
    myWalkingAreaCustomShapes.push_back(wacs);
}


bool
NBNode::geometryLike() const {
    return geometryLike(myIncomingEdges, myOutgoingEdges);
}

bool
NBNode::geometryLike(const EdgeVector& incoming, const EdgeVector& outgoing) {
    if (incoming.size() == 1 && outgoing.size() == 1) {
        return true;
    }
    if (incoming.size() == 2 && outgoing.size() == 2) {
        // check whether the incoming and outgoing edges are pairwise (near) parallel and
        // thus the only cross-connections could be turn-arounds
        NBEdge* in0 = incoming[0];
        NBEdge* in1 = incoming[1];
        NBEdge* out0 = outgoing[0];
        NBEdge* out1 = outgoing[1];
        if ((in0->isTurningDirectionAt(out0) || in0->isTurningDirectionAt(out1))
                && (in1->isTurningDirectionAt(out0) || in1->isTurningDirectionAt(out1))) {
            return true;
        }
        if (in0->getGeometry() == in1->getGeometry() && out0->getGeometry() == out1->getGeometry()) {
            // overlapping edges
            return true;
        }
        for (EdgeVector::const_iterator it = incoming.begin(); it != incoming.end(); ++it) {
            NBEdge* inEdge = *it;
            double angle0 = fabs(NBHelpers::relAngle(inEdge->getAngleAtNode(inEdge->getToNode()), out0->getAngleAtNode(out0->getFromNode())));
            double angle1 = fabs(NBHelpers::relAngle(inEdge->getAngleAtNode(inEdge->getToNode()), out1->getAngleAtNode(out1->getFromNode())));
            if (MAX2(angle0, angle1) <= 160) {
                // neither of the outgoing edges is parallel to inEdge
                return false;
            }
        }
        return true;
    }
    return false;
}

void
NBNode::setRoundabout() {
    if (myType == SumoXMLNodeType::RIGHT_BEFORE_LEFT || myType == SumoXMLNodeType::LEFT_BEFORE_RIGHT) {
        myType = SumoXMLNodeType::PRIORITY;
    }
}

bool
NBNode::isRoundabout() const {
    for (NBEdge* out : myOutgoingEdges) {
        if (out->getJunctionPriority(this) == NBEdge::JunctionPriority::ROUNDABOUT) {
            return true;
        }
    }
    return false;
}

NBNode::Crossing*
NBNode::addCrossing(EdgeVector edges, double width, bool priority, int tlIndex, int tlIndex2,
                    const PositionVector& customShape, bool fromSumoNet, const Parameterised* params) {
    Crossing* c = new Crossing(this, edges, width, priority, tlIndex, tlIndex2, customShape);
    if (params != nullptr) {
        c->updateParameters(params->getParametersMap());
    }
    myCrossings.push_back(std::unique_ptr<Crossing>(c));
    if (fromSumoNet) {
        myCrossingsLoadedFromSumoNet += 1;
    }
    return c;
}


void
NBNode::removeCrossing(const EdgeVector& edges) {
    EdgeSet edgeSet(edges.begin(), edges.end());
    for (auto it = myCrossings.begin(); it != myCrossings.end();) {
        EdgeSet edgeSet2((*it)->edges.begin(), (*it)->edges.end());
        if (edgeSet == edgeSet2) {
            it = myCrossings.erase(it);
        } else {
            ++it;
        }
    }
}


NBNode::Crossing*
NBNode::getCrossing(const std::string& id) const {
    for (auto& c : myCrossings) {
        if (c->id == id) {
            return c.get();
        }
    }
    throw ProcessError(TLF("Request for unknown crossing '%'", id));
}


NBNode::Crossing*
NBNode::getCrossing(const EdgeVector& edges, bool hardFail) const {
    const EdgeSet edgeSet(edges.begin(), edges.end());
    for (auto& crossing : myCrossings) {
        const EdgeSet edgeSet2(crossing->edges.begin(), crossing->edges.end());
        if (edgeSet == edgeSet2) {
            return crossing.get();
        }
    }
    if (!hardFail) {
        return nullptr;
    }
    throw ProcessError(TL("Request for unknown crossing for the given Edges"));
}


NBNode::WalkingArea&
NBNode::getWalkingArea(const std::string& id) {
    for (auto& walkingArea : myWalkingAreas) {
        if (walkingArea.id == id) {
            return walkingArea;
        }
    }
    // not found, maybe we need to rebuild
    updateSurroundingGeometry();
    sortEdges(true);
    buildCrossingsAndWalkingAreas();
    for (auto& walkingArea : myWalkingAreas) {
        if (walkingArea.id == id) {
            return walkingArea;
        }
    }
    if (myWalkingAreas.size() > 0) {
        // don't crash
        WRITE_WARNINGF("Could not retrieve walkingarea '%' (edge ordering changed after recompute).", id);
        return myWalkingAreas.front();
    }
    throw ProcessError(TLF("Request for unknown walkingarea '%'.", id));
}


bool
NBNode::setCrossingTLIndices(const std::string& tlID, int startIndex, bool ignoreCustom) {
    bool usedCustom = false;
    for (auto c : getCrossings()) {
        c->tlLinkIndex = startIndex++;
        c->tlID = tlID;
        if (c->customTLIndex != -1 && !ignoreCustom) {
            usedCustom |= (c->tlLinkIndex != c->customTLIndex);
            c->tlLinkIndex = c->customTLIndex;
        }
        if (c->customTLIndex2 != -1 && !ignoreCustom) {
            usedCustom = true;
            c->tlLinkIndex2 = c->customTLIndex2;
        }
    }
    return usedCustom;
}


int
NBNode::numNormalConnections() const {
    if (myRequest == nullptr) {
        // could be an uncontrolled type
        int result = 0;
        for (const NBEdge* const edge : myIncomingEdges) {
            result += (int)edge->getConnections().size();
        }
        return result;
    } else {
        return myRequest->getSizes().second;
    }
}


int
NBNode::getConnectionIndex(const NBEdge* from, const NBEdge::Connection& con) const {
    int result = 0;
    for (const NBEdge* const e : myIncomingEdges) {
        for (const NBEdge::Connection& cand : e->getConnections()) {
            if (e == from && cand.fromLane == con.fromLane && cand.toLane == con.toLane && cand.toEdge == con.toEdge) {
                return result;
            }
            result++;
        }
    }
    return -1;
}


Position
NBNode::getCenter() const {
    /* Conceptually, the center point would be identical with myPosition.
    * However, if the shape is influenced by custom geometry endpoints of the adjoining edges,
    * myPosition may fall outside the shape. In this case it is better to use
    * the center of the shape
    **/
    PositionVector tmp = myPoly;
    tmp.closePolygon();
    //std::cout << getID() << " around=" << tmp.around(myPosition) << " dist=" << tmp.distance2D(myPosition) << "\n";
    if (tmp.size() < 3 || tmp.around(myPosition) || tmp.distance2D(myPosition) < POSITION_EPS) {
        return myPosition;
    }
    return myPoly.getPolygonCenter();
}


EdgeVector
NBNode::getEdgesSortedByAngleAtNodeCenter() const {
    EdgeVector result = myAllEdges;
#ifdef DEBUG_PED_STRUCTURES
    if (gDebugFlag1) {
        std::cout << "  angles:\n";
        for (EdgeVector::const_iterator it = result.begin(); it != result.end(); ++it) {
            std::cout << "    edge=" << (*it)->getID() << " edgeAngle=" << (*it)->getAngleAtNode(this) << " angleToShape=" << (*it)->getAngleAtNodeToCenter(this) << "\n";
        }
        std::cout << "  allEdges before: " << toString(result) << "\n";
    }
#endif
    sort(result.begin(), result.end(), NBContHelper::edge_by_angle_to_nodeShapeCentroid_sorter(this));
    // let the first edge in myAllEdges remain the first
    DEBUGCOUT(gDebugFlag1, "  allEdges sorted: " << toString(result) << "\n")
    rotate(result.begin(), std::find(result.begin(), result.end(), *myAllEdges.begin()), result.end());
    DEBUGCOUT(gDebugFlag1, "  allEdges rotated: " << toString(result) << "\n")
    return result;
}


void
NBNode::avoidOverlap() {
    // simple case: edges with LaneSpreadFunction::CENTER and a (possible) turndirection at the same node
    bool haveModifications = false;
    for (EdgeVector::iterator it = myIncomingEdges.begin(); it != myIncomingEdges.end(); it++) {
        NBEdge* edge = *it;
        NBEdge* turnDest = edge->getTurnDestination(true);
        if (turnDest != nullptr) {
            haveModifications |= edge->shiftPositionAtNode(this, turnDest);
            haveModifications |= turnDest->shiftPositionAtNode(this, edge);
        }
    }
    if (haveModifications) {
        NBTurningDirectionsComputer::computeTurnDirectionsForNode(this, false);
    }
    // @todo: edges in the same direction with sharp angles starting/ending at the same position
}


bool
NBNode::isTrafficLight(SumoXMLNodeType type) {
    return type == SumoXMLNodeType::TRAFFIC_LIGHT
           || type == SumoXMLNodeType::TRAFFIC_LIGHT_NOJUNCTION
           || type == SumoXMLNodeType::TRAFFIC_LIGHT_RIGHT_ON_RED;
}


bool
NBNode::extraConflict(int index, int foeIndex) const {
    for (NBTrafficLightDefinition* def : myTrafficLights) {
        if (def->extraConflict(index, foeIndex)) {
            return true;
        }
    }
    return false;
}


void
NBNode::sortEdges(bool useNodeShape) {
    if (myAllEdges.size() == 0) {
        return;
    }
    EdgeVector allEdgesOriginal = myAllEdges;
    EdgeVector& allEdges = myAllEdges;
    EdgeVector& incoming = myIncomingEdges;
    EdgeVector& outgoing = myOutgoingEdges;

    // sort the edges by angle (this is the canonical sorting)
    std::sort(allEdges.begin(), allEdges.end(), NBNodesEdgesSorter::edge_by_junction_angle_sorter(this));
    std::sort(incoming.begin(), incoming.end(), NBNodesEdgesSorter::edge_by_junction_angle_sorter(this));
    std::sort(outgoing.begin(), outgoing.end(), NBNodesEdgesSorter::edge_by_junction_angle_sorter(this));
    std::vector<NBEdge*>::iterator j;
    for (j = allEdges.begin(); j != allEdges.end() - 1 && j != allEdges.end(); ++j) {
        NBNodesEdgesSorter::swapWhenReversed(this, j, j + 1);
    }
    if (allEdges.size() > 1 && j != allEdges.end()) {
        NBNodesEdgesSorter::swapWhenReversed(this, allEdges.end() - 1, allEdges.begin());
    }

    // sort again using additional geometry information
    NBEdge* firstOfAll = allEdges.front();
    NBEdge* firstOfIncoming = incoming.size() > 0 ? incoming.front() : 0;
    NBEdge* firstOfOutgoing = outgoing.size() > 0 ? outgoing.front() : 0;
    // sort by the angle between the node shape center and the point where the edge meets the node shape
    std::sort(allEdges.begin(), allEdges.end(), NBContHelper::edge_by_angle_to_nodeShapeCentroid_sorter(this));
    std::sort(incoming.begin(), incoming.end(), NBContHelper::edge_by_angle_to_nodeShapeCentroid_sorter(this));
    std::sort(outgoing.begin(), outgoing.end(), NBContHelper::edge_by_angle_to_nodeShapeCentroid_sorter(this));
    // let the first edge remain the first
    rotate(allEdges.begin(), std::find(allEdges.begin(), allEdges.end(), firstOfAll), allEdges.end());
    if (firstOfIncoming != nullptr) {
        rotate(incoming.begin(), std::find(incoming.begin(), incoming.end(), firstOfIncoming), incoming.end());
    }
    if (firstOfOutgoing != nullptr) {
        rotate(outgoing.begin(), std::find(outgoing.begin(), outgoing.end(), firstOfOutgoing), outgoing.end());
    }
#ifdef DEBUG_EDGE_SORTING
    if (DEBUGCOND) {
        std::cout << "sortedEdges (useNodeShape=" << useNodeShape << "):\n";
        for (NBEdge* e : allEdges) {
            std::cout << "  " << e->getID()
                      << " angleToCenter=" << e->getAngleAtNodeToCenter(this)
                      << " junctionAngle=" << e->getAngleAtNode(this) << "\n";
        }
    }
#endif

    // fixing some pathological all edges orderings
    // if every of the edges a,b,c has a turning edge a',b',c' the all edges ordering should be a,a',b,b',c,c'
    if (incoming.size() == outgoing.size() && incoming.front() == allEdges.front()) {
        std::vector<NBEdge*>::const_iterator in, out;
        std::vector<NBEdge*> allTmp;
        for (in = incoming.begin(), out = outgoing.begin(); in != incoming.end(); ++in, ++out) {
            if ((*in)->isTurningDirectionAt(*out)) {
                allTmp.push_back(*in);
                allTmp.push_back(*out);
            } else {
                break;
            }
        }
        if (allTmp.size() == allEdges.size()) {
            allEdges = allTmp;
        }
    }
    // sort the crossings
    std::sort(myCrossings.begin(), myCrossings.end(), NBNodesEdgesSorter::crossing_by_junction_angle_sorter(this, allEdges));
    //if (crossings.size() > 0) {
    //    std::cout << " crossings at " << getID() << "\n";
    //    for (std::vector<NBNode::Crossing*>::iterator it = crossings.begin(); it != crossings.end(); ++it) {
    //        std::cout << "  " << toString((*it)->edges) << "\n";
    //    }
    //}

    if (useNodeShape && myAllEdges != allEdgesOriginal) {
        // sorting order changed after node shape was computed.
        computeNodeShape(-1);
        for (NBEdge* e : myAllEdges) {
            e->computeEdgeShape();
        }
    }
}

std::vector<std::pair<Position, std::string> >
NBNode::getEndPoints() const {
    // using a set would be nicer but we want to have some slack in position identification
    std::vector<std::pair<Position, std::string> >result;
    for (NBEdge* e : myAllEdges) {
        Position pos = this == e->getFromNode() ? e->getGeometry().front() : e->getGeometry().back();
        const std::string origID = e->getParameter(this == e->getFromNode() ? "origFrom" : "origTo");
        bool unique = true;
        for (const auto& pair : result) {
            if (pos.almostSame(pair.first) || (origID != "" && pair.second == origID)) {
                unique = false;
                break;
            }
        }
        if (unique) {
            result.push_back(std::make_pair(pos, origID));
        }
    }
    return result;
}


/****************************************************************************/
