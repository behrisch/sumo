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
/// @file    NLHandler.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Clemens Honomichl
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @author  Felix Brack
/// @date    Mon, 9 Jul 2001
///
// The XML-Handler for network loading
/****************************************************************************/
#include <config.h>

#include <string>
#include "NLHandler.h"
#include "NLEdgeControlBuilder.h"
#include "NLJunctionControlBuilder.h"
#include "NLDetectorBuilder.h"
#include "NLTriggerBuilder.h"
#include <utils/xml/SUMOXMLDefinitions.h>
#include <utils/xml/SUMOSAXHandler.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/SUMOTime.h>
#include <utils/common/StringUtils.h>
#include <utils/common/StringTokenizer.h>
#include <utils/common/RGBColor.h>
#include <utils/geom/GeomConvHelper.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSLane.h>
#include <microsim/MSJunction.h>
#include <microsim/MSJunctionLogic.h>
#include <microsim/MSStoppingPlace.h>
#include <microsim/traffic_lights/MSTrafficLightLogic.h>
#include <microsim/traffic_lights/MSRailSignal.h>
#include <microsim/traffic_lights/MSRailSignalControl.h>
#include <microsim/traffic_lights/MSRailSignalConstraint.h>
#include <mesosim/MESegment.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/common/UtilExceptions.h>
#include <utils/geom/GeoConvHelper.h>
#include <utils/shapes/ShapeContainer.h>
#include <utils/shapes/Shape.h>


// ===========================================================================
// method definitions
// ===========================================================================
NLHandler::NLHandler(const std::string& file, MSNet& net,
                     NLDetectorBuilder& detBuilder,
                     NLTriggerBuilder& triggerBuilder,
                     NLEdgeControlBuilder& edgeBuilder,
                     NLJunctionControlBuilder& junctionBuilder) :
    MSRouteHandler(file, true),
    myNet(net), myActionBuilder(net),
    myCurrentIsInternalToSkip(false),
    myDetectorBuilder(detBuilder), myTriggerBuilder(triggerBuilder),
    myEdgeControlBuilder(edgeBuilder), myJunctionControlBuilder(junctionBuilder),
    myAmParsingTLLogicOrJunction(false), myCurrentIsBroken(false),
    myHaveWarnedAboutInvalidTLType(false),
    myHaveSeenInternalEdge(false),
    myHaveJunctionHigherSpeeds(false),
    myHaveSeenDefaultLength(false),
    myHaveSeenNeighs(false),
    myHaveSeenAdditionalSpeedRestrictions(false),
    myHaveSeenTLSParams(false),
    myNetworkVersion(0, 0),
    myNetIsLoaded(false) {
}


NLHandler::~NLHandler() {}


void
NLHandler::myStartElement(int element,
                          const SUMOSAXAttributes& attrs) {
    try {
        switch (element) {
            case SUMO_TAG_NET: {
                bool ok;
                MSGlobals::gLefthand = attrs.getOpt<bool>(SUMO_ATTR_LEFTHAND, nullptr, ok, false);
                myHaveJunctionHigherSpeeds = attrs.getOpt<bool>(SUMO_ATTR_HIGHER_SPEED, nullptr, ok, false);
                myNetworkVersion = StringUtils::toVersion(attrs.get<std::string>(SUMO_ATTR_VERSION, nullptr, ok, false));
                break;
            }
            case SUMO_TAG_EDGE:
                beginEdgeParsing(attrs);
                break;
            case SUMO_TAG_LANE:
                addLane(attrs);
                break;
            case SUMO_TAG_NEIGH:
                if (!myCurrentIsInternalToSkip) {
                    myEdgeControlBuilder.addNeigh(attrs.getString(SUMO_ATTR_LANE));
                }
                myHaveSeenNeighs = true;
                break;
            case SUMO_TAG_JUNCTION:
                openJunction(attrs);
                initJunctionLogic(attrs);
                break;
            case SUMO_TAG_PHASE:
                addPhase(attrs);
                break;
            case SUMO_TAG_CONDITION:
                addCondition(attrs);
                break;
            case SUMO_TAG_ASSIGNMENT:
                addAssignment(attrs);
                break;
            case SUMO_TAG_FUNCTION:
                addFunction(attrs);
                break;
            case SUMO_TAG_CONNECTION:
                addConnection(attrs);
                break;
            case SUMO_TAG_CONFLICT:
                addConflict(attrs);
                break;
            case SUMO_TAG_TLLOGIC:
                initTrafficLightLogic(attrs);
                break;
            case SUMO_TAG_REQUEST:
                addRequest(attrs);
                break;
            case SUMO_TAG_WAUT:
                openWAUT(attrs);
                break;
            case SUMO_TAG_WAUT_SWITCH:
                addWAUTSwitch(attrs);
                break;
            case SUMO_TAG_WAUT_JUNCTION:
                addWAUTJunction(attrs);
                break;
            case SUMO_TAG_E1DETECTOR:
            case SUMO_TAG_INDUCTION_LOOP:
                addE1Detector(attrs);
                break;
            case SUMO_TAG_E2DETECTOR:
            case SUMO_TAG_LANE_AREA_DETECTOR:
                addE2Detector(attrs);
                break;
            case SUMO_TAG_E3DETECTOR:
            case SUMO_TAG_ENTRY_EXIT_DETECTOR:
                beginE3Detector(attrs);
                break;
            case SUMO_TAG_DET_ENTRY:
                addE3Entry(attrs);
                break;
            case SUMO_TAG_DET_EXIT:
                addE3Exit(attrs);
                break;
            case SUMO_TAG_INSTANT_INDUCTION_LOOP:
                addInstantE1Detector(attrs);
                break;
            case SUMO_TAG_VSS:
                myTriggerBuilder.parseAndBuildLaneSpeedTrigger(myNet, attrs, getFileName());
                break;
            case SUMO_TAG_CALIBRATOR:
                myTriggerBuilder.parseAndBuildCalibrator(myNet, attrs, getFileName());
                break;
            case SUMO_TAG_REROUTER:
                myTriggerBuilder.parseAndBuildRerouter(myNet, attrs);
                break;
            case SUMO_TAG_BUS_STOP:
            case SUMO_TAG_TRAIN_STOP:
            case SUMO_TAG_CONTAINER_STOP:
                myTriggerBuilder.parseAndBuildStoppingPlace(myNet, attrs, (SumoXMLTag)element);
                myLastParameterised.push_back(myTriggerBuilder.getCurrentStop());
                break;
            case SUMO_TAG_PARKING_SPACE:
                myTriggerBuilder.parseAndAddLotEntry(attrs);
                break;
            case SUMO_TAG_PARKING_AREA:
                myTriggerBuilder.parseAndBeginParkingArea(myNet, attrs);
                myLastParameterised.push_back(myTriggerBuilder.getCurrentStop());
                break;
            case SUMO_TAG_ACCESS:
                myTriggerBuilder.addAccess(myNet, attrs);
                break;
            case SUMO_TAG_CHARGING_STATION:
                myTriggerBuilder.parseAndBuildChargingStation(myNet, attrs);
                myLastParameterised.push_back(myTriggerBuilder.getCurrentStop());
                break;
            case SUMO_TAG_OVERHEAD_WIRE_SEGMENT:
                myTriggerBuilder.parseAndBuildOverheadWireSegment(myNet, attrs);
                break;
            case SUMO_TAG_OVERHEAD_WIRE_SECTION:
                myTriggerBuilder.parseAndBuildOverheadWireSection(myNet, attrs);
                break;
            case SUMO_TAG_TRACTION_SUBSTATION:
                myTriggerBuilder.parseAndBuildTractionSubstation(myNet, attrs);
                break;
            case SUMO_TAG_OVERHEAD_WIRE_CLAMP:
                myTriggerBuilder.parseAndBuildOverheadWireClamp(myNet, attrs);
                break;
            case SUMO_TAG_VTYPEPROBE:
                addVTypeProbeDetector(attrs);
                break;
            case SUMO_TAG_ROUTEPROBE:
                addRouteProbeDetector(attrs);
                break;
            case SUMO_TAG_MEANDATA_EDGE:
                addEdgeLaneMeanData(attrs, SUMO_TAG_MEANDATA_EDGE);
                break;
            case SUMO_TAG_MEANDATA_LANE:
                addEdgeLaneMeanData(attrs, SUMO_TAG_MEANDATA_LANE);
                break;
            case SUMO_TAG_TIMEDEVENT:
                myActionBuilder.addAction(attrs, getFileName());
                break;
            case SUMO_TAG_VAPORIZER:
                myTriggerBuilder.buildVaporizer(attrs);
                break;
            case SUMO_TAG_LOCATION:
                setLocation(attrs);
                break;
            case SUMO_TAG_TAZ:
                addDistrict(attrs);
                break;
            case SUMO_TAG_TAZSOURCE:
                addDistrictEdge(attrs, true);
                break;
            case SUMO_TAG_TAZSINK:
                addDistrictEdge(attrs, false);
                break;
            case SUMO_TAG_ROUNDABOUT:
                addRoundabout(attrs);
                break;
            case SUMO_TAG_TYPE: {
                bool ok = true;
                myCurrentTypeID = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
                break;
            }
            case SUMO_TAG_RESTRICTION: {
                bool ok = true;
                const SUMOVehicleClass svc = getVehicleClassID(attrs.get<std::string>(SUMO_ATTR_VCLASS, myCurrentTypeID.c_str(), ok));
                const double speed = attrs.get<double>(SUMO_ATTR_SPEED, myCurrentTypeID.c_str(), ok);
                if (ok) {
                    myNet.addRestriction(myCurrentTypeID, svc, speed);
                }
                if (myNetIsLoaded) {
                    myHaveSeenAdditionalSpeedRestrictions = true;
                }
                break;
            }
            case SUMO_TAG_PREFERENCE: {
                bool ok = true;
                const std::string routingType = attrs.get<std::string>(SUMO_ATTR_ROUTINGTYPE, nullptr, ok);
                const double prio = attrs.get<double>(SUMO_ATTR_PRIORITY, routingType.c_str(), ok);
                if (prio <= 0) {
                    throw InvalidArgument("In preference for routingType '" + routingType + "', priority must be positve");
                }
                if (attrs.hasAttribute(SUMO_ATTR_VCLASSES)) {
                    StringTokenizer st(attrs.get<std::string>(SUMO_ATTR_VCLASSES, routingType.c_str(), ok));
                    for (std::string className : st.getVector()) {
                        myNet.addPreference(routingType, getVehicleClassID(className), prio);
                    }
                } else if (!attrs.hasAttribute(SUMO_ATTR_VTYPES)) {
                    // general preferenze applying to all types and vClasses
                    myNet.addPreference(routingType, "", prio);
                }
                if (attrs.hasAttribute(SUMO_ATTR_VTYPES)) {
                    StringTokenizer st(attrs.get<std::string>(SUMO_ATTR_VTYPES, routingType.c_str(), ok));
                    for (std::string typeName : st.getVector()) {
                        myNet.addPreference(routingType, typeName, prio);
                    }
                }
                break;
            }
            case SUMO_TAG_STOPOFFSET: {
                bool ok = true;
                const StopOffset stopOffset(attrs, ok);
                if (!ok) {
                    WRITE_ERROR(myEdgeControlBuilder.reportCurrentEdgeOrLane());
                } else {
                    myEdgeControlBuilder.addStopOffsets(stopOffset);
                }
                break;
            }
            case SUMO_TAG_RAILSIGNAL_CONSTRAINTS: {
                bool ok = true;
                const std::string signalID = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
                if (!MSNet::getInstance()->getTLSControl().knows(signalID)) {
                    throw InvalidArgument("Rail signal '" + signalID + "' in railSignalConstraints is not known");
                }
                myConstrainedSignal = dynamic_cast<MSRailSignal*>(MSNet::getInstance()->getTLSControl().get(signalID).getDefault());
                if (myConstrainedSignal == nullptr) {
                    throw InvalidArgument("Traffic light '" + signalID + "' is not a rail signal");
                }
                break;
            }
            case SUMO_TAG_PREDECESSOR: // intended fall-through
            case SUMO_TAG_FOE_INSERTION: // intended fall-through
            case SUMO_TAG_INSERTION_PREDECESSOR: // intended fall-through
            case SUMO_TAG_INSERTION_ORDER: // intended fall-through
            case SUMO_TAG_BIDI_PREDECESSOR:
                myLastParameterised.push_back(addPredecessorConstraint(element, attrs, myConstrainedSignal));
                break;
            case SUMO_TAG_DEADLOCK:
                addDeadlock(attrs);
                break;
            default:
                break;
        }
    } catch (InvalidArgument& e) {
        myCurrentIsBroken = true;
        WRITE_ERROR(e.what());
    }
    MSRouteHandler::myStartElement(element, attrs);
    if (element == SUMO_TAG_PARAM && !myCurrentIsBroken) {
        addParam(attrs);
    }
}


void
NLHandler::myEndElement(int element) {
    switch (element) {
        case SUMO_TAG_EDGE:
            closeEdge();
            break;
        case SUMO_TAG_LANE:
            myEdgeControlBuilder.closeLane();
            if (!myCurrentIsInternalToSkip && !myCurrentIsBroken) {
                myLastParameterised.pop_back();
            }
            break;
        case SUMO_TAG_JUNCTION:
            if (!myCurrentIsBroken) {
                try {
                    myJunctionControlBuilder.closeJunction(getFileName());
                } catch (InvalidArgument& e) {
                    WRITE_ERROR(e.what());
                }
            }
            myAmParsingTLLogicOrJunction = false;
            break;
        case SUMO_TAG_TLLOGIC:
            if (!myCurrentIsBroken) {
                try {
                    myJunctionControlBuilder.closeTrafficLightLogic(getFileName());
                } catch (InvalidArgument& e) {
                    for (MSPhaseDefinition* const phase : myJunctionControlBuilder.getLoadedPhases()) {
                        delete phase;
                    }
                    WRITE_ERROR(e.what());
                }
            }
            myAmParsingTLLogicOrJunction = false;
            break;
        case SUMO_TAG_FUNCTION:
            closeFunction();
            break;
        case SUMO_TAG_WAUT:
            closeWAUT();
            break;
        case SUMO_TAG_RAILSIGNAL_CONSTRAINTS:
            myConstrainedSignal = nullptr;
            break;
        case SUMO_TAG_E1DETECTOR:
        case SUMO_TAG_INDUCTION_LOOP:
        case SUMO_TAG_INSTANT_INDUCTION_LOOP:
        case SUMO_TAG_E2DETECTOR:
        case SUMO_TAG_LANE_AREA_DETECTOR:
            if (!myCurrentIsBroken) {
                myLastParameterised.pop_back();
            }
            break;
        case SUMO_TAG_E3DETECTOR:
        case SUMO_TAG_ENTRY_EXIT_DETECTOR:
            endE3Detector();
            if (!myCurrentIsBroken) {
                myLastParameterised.pop_back();
            }
            break;
        case SUMO_TAG_PARKING_AREA:
            myTriggerBuilder.updateParkingAreaDefaultCapacity();
            myTriggerBuilder.endParkingArea();
            myLastParameterised.pop_back();
            break;
        case SUMO_TAG_BUS_STOP:
        case SUMO_TAG_TRAIN_STOP:
        case SUMO_TAG_CONTAINER_STOP:
        case SUMO_TAG_CHARGING_STATION:
            myTriggerBuilder.endStoppingPlace();
            myLastParameterised.pop_back();
            break;
        case SUMO_TAG_PREDECESSOR: // intended fall-through
        case SUMO_TAG_FOE_INSERTION: // intended fall-through
        case SUMO_TAG_INSERTION_PREDECESSOR: // intended fall-through
        case SUMO_TAG_INSERTION_ORDER: // intended fall-through
        case SUMO_TAG_BIDI_PREDECESSOR:
            myLastParameterised.pop_back();
            break;
        case SUMO_TAG_NET:
            // build junction graph
            for (JunctionGraph::iterator it = myJunctionGraph.begin(); it != myJunctionGraph.end(); ++it) {
                MSEdge* edge = MSEdge::dictionary(it->first);
                MSJunction* from = myJunctionControlBuilder.retrieve(it->second.first);
                MSJunction* to = myJunctionControlBuilder.retrieve(it->second.second);
                if (from == nullptr) {
                    WRITE_ERRORF(TL("Unknown from-node '%' for edge '%'."), it->second.first, it->first);
                    return;
                }
                if (to == nullptr) {
                    WRITE_ERRORF(TL("Unknown to-node '%' for edge '%'."), it->second.second, it->first);
                    return;
                }
                if (edge != nullptr) {
                    edge->setJunctions(from, to);
                    from->addOutgoing(edge);
                    to->addIncoming(edge);
                }
            }
            myNetIsLoaded = true;
            break;
        default:
            break;
    }
    MSRouteHandler::myEndElement(element);
}



// ---- the root/edge - element
void
NLHandler::beginEdgeParsing(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    myCurrentIsBroken = false;
    // get the id, report an error if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    if (!ok) {
        myCurrentIsBroken = true;
        return;
    }
    // parse the function
    const SumoXMLEdgeFunc func = attrs.getOpt<SumoXMLEdgeFunc>(SUMO_ATTR_FUNCTION, id.c_str(), ok, SumoXMLEdgeFunc::NORMAL);
    if (!ok) {
        myCurrentIsBroken = true;
        return;
    }
    // omit internal edges if not wished
    if (id[0] == ':') {
        myHaveSeenInternalEdge = true;
        if (!MSGlobals::gUsingInternalLanes && (func == SumoXMLEdgeFunc::CROSSING || func == SumoXMLEdgeFunc::WALKINGAREA)) {
            myCurrentIsInternalToSkip = true;
            return;
        }
        std::string junctionID = SUMOXMLDefinitions::getJunctionIDFromInternalEdge(id);
        myJunctionGraph[id] = std::make_pair(junctionID, junctionID);
    } else {
        myHaveSeenDefaultLength |= !attrs.hasAttribute(SUMO_ATTR_LENGTH);
        myJunctionGraph[id] = std::make_pair(
                                  attrs.get<std::string>(SUMO_ATTR_FROM, id.c_str(), ok),
                                  attrs.get<std::string>(SUMO_ATTR_TO, id.c_str(), ok));
    }
    if (!ok) {
        myCurrentIsBroken = true;
        return;
    }
    myCurrentIsInternalToSkip = false;
    // get the street name
    const std::string streetName = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), ok, "");
    // get the edge type
    const std::string edgeType = attrs.getOpt<std::string>(SUMO_ATTR_TYPE, id.c_str(), ok, "");
    const std::string routingType = attrs.getOpt<std::string>(SUMO_ATTR_ROUTINGTYPE, id.c_str(), ok, "");
    // get the edge priority (only for visualization)
    const int priority = attrs.getOpt<int>(SUMO_ATTR_PRIORITY, id.c_str(), ok, -1); // default taken from netbuild/NBFrame option 'default.priority'
    // get the bidi-edge
    const std::string bidi = attrs.getOpt<std::string>(SUMO_ATTR_BIDI, id.c_str(), ok, "");
    // get the kilometrage/mileage (for visualization and output)
    const double distance = attrs.getOpt<double>(SUMO_ATTR_DISTANCE, id.c_str(), ok, 0);

    if (!ok) {
        myCurrentIsBroken = true;
        return;
    }
    //
    try {
        myEdgeControlBuilder.beginEdgeParsing(id, func, streetName, edgeType, routingType, priority, bidi, distance);
    } catch (InvalidArgument& e) {
        WRITE_ERROR(e.what());
        myCurrentIsBroken = true;
    }

    if (func == SumoXMLEdgeFunc::CROSSING) {
        //get the crossingEdges attribute (to implement the other side of the road pushbutton)
        const std::string crossingEdges = attrs.getOpt<std::string>(SUMO_ATTR_CROSSING_EDGES, id.c_str(), ok, "");
        if (!crossingEdges.empty()) {
            std::vector<std::string> crossingEdgesVector;
            StringTokenizer edges(crossingEdges);
            while (edges.hasNext()) {
                crossingEdgesVector.push_back(edges.next());
            }
            myEdgeControlBuilder.addCrossingEdges(crossingEdgesVector);
        }
    }
    myLastEdgeParameters.clearParameter();
    myLastParameterised.push_back(&myLastEdgeParameters);
}


void
NLHandler::closeEdge() {
    myLastParameterised.clear();
    // omit internal edges if not wished and broken edges
    if (myCurrentIsInternalToSkip || myCurrentIsBroken) {
        return;
    }
    try {
        MSEdge* e = myEdgeControlBuilder.closeEdge();
        MSEdge::dictionary(e->getID(), e);
        e->updateParameters(myLastEdgeParameters.getParametersMap());
    } catch (InvalidArgument& e) {
        WRITE_ERROR(e.what());
    }
}


//             ---- the root/edge/lanes/lane - element
void
NLHandler::addLane(const SUMOSAXAttributes& attrs) {
    // omit internal edges if not wished and broken edges
    if (myCurrentIsInternalToSkip || myCurrentIsBroken) {
        return;
    }
    bool ok = true;
    // get the id, report an error if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    if (!ok) {
        myCurrentIsBroken = true;
        return;
    }
    const double maxSpeed = attrs.get<double>(SUMO_ATTR_SPEED, id.c_str(), ok);
    const double friction = attrs.getOpt<double>(SUMO_ATTR_FRICTION, id.c_str(), ok, (double)(1.), false);
    const double length = attrs.get<double>(SUMO_ATTR_LENGTH, id.c_str(), ok);
    const std::string allow = attrs.getOpt<std::string>(SUMO_ATTR_ALLOW, id.c_str(), ok, "", false);
    const std::string disallow = attrs.getOpt<std::string>(SUMO_ATTR_DISALLOW, id.c_str(), ok, "");
    const std::string changeLeftS = attrs.getOpt<std::string>(SUMO_ATTR_CHANGE_LEFT, id.c_str(), ok, "");
    const std::string changeRightS = attrs.getOpt<std::string>(SUMO_ATTR_CHANGE_RIGHT, id.c_str(), ok, "");
    const double width = attrs.getOpt<double>(SUMO_ATTR_WIDTH, id.c_str(), ok, SUMO_const_laneWidth);
    const PositionVector shape = attrs.get<PositionVector>(SUMO_ATTR_SHAPE, id.c_str(), ok);
    const PositionVector outlineShape = attrs.getOpt<PositionVector>(SUMO_ATTR_OUTLINESHAPE, id.c_str(), ok, PositionVector());
    const int index = attrs.get<int>(SUMO_ATTR_INDEX, id.c_str(), ok);
    const bool isRampAccel = attrs.getOpt<bool>(SUMO_ATTR_ACCELERATION, id.c_str(), ok, false);
    const std::string type = attrs.getOpt<std::string>(SUMO_ATTR_TYPE, id.c_str(), ok, "");
    if (shape.size() < 2) {
        WRITE_ERRORF(TL("Shape of lane '%' is broken.\n Can not build according edge."), id);
        myCurrentIsBroken = true;
        return;
    }
    const SVCPermissions permissions = parseVehicleClasses(allow, disallow, myNetworkVersion);
    SVCPermissions changeLeft = parseVehicleClasses(changeLeftS, "", myNetworkVersion);
    SVCPermissions changeRight = parseVehicleClasses(changeRightS, "", myNetworkVersion);
    if (MSGlobals::gLefthand) {
        // internally, changeLeft always checks for the higher lane index
        // even though the higher lane index is to the right in a left-hand network
        std::swap(changeLeft, changeRight);
    }
    if (permissions != SVCAll || changeLeft != SVCAll || changeRight != SVCAll) {
        myNet.setPermissionsFound();
    }
    myCurrentIsBroken |= !ok;
    if (!myCurrentIsBroken) {
        try {
            MSLane* lane = myEdgeControlBuilder.addLane(id, maxSpeed, friction, length, shape, width, permissions, changeLeft, changeRight, index, isRampAccel, type, outlineShape);
            // insert the lane into the lane-dictionary, checking
            if (!MSLane::dictionary(id, lane)) {
                delete lane;
                WRITE_ERRORF(TL("Another lane with the id '%' exists."), id);
                myCurrentIsBroken = true;
                myLastParameterised.push_back(nullptr);
            } else {
                myLastParameterised.push_back(lane);
            }
        } catch (InvalidArgument& e) {
            WRITE_ERROR(e.what());
        }
    }
}


// ---- the root/junction - element
void
NLHandler::openJunction(const SUMOSAXAttributes& attrs) {
    myCurrentIsBroken = false;
    bool ok = true;
    // get the id, report an error if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    if (!ok) {
        myCurrentIsBroken = true;
        return;
    }
    PositionVector shape;
    if (attrs.hasAttribute(SUMO_ATTR_SHAPE)) {
        // inner junctions have no shape
        shape = attrs.getOpt<PositionVector>(SUMO_ATTR_SHAPE, id.c_str(), ok, PositionVector());
        if (shape.size() > 2) {
            shape.closePolygon();
        }
    }
    double x = attrs.get<double>(SUMO_ATTR_X, id.c_str(), ok);
    double y = attrs.get<double>(SUMO_ATTR_Y, id.c_str(), ok);
    double z = attrs.getOpt<double>(SUMO_ATTR_Z, id.c_str(), ok, 0);
    const SumoXMLNodeType type = attrs.get<SumoXMLNodeType>(SUMO_ATTR_TYPE, id.c_str(), ok);
    std::string key = attrs.getOpt<std::string>(SUMO_ATTR_KEY, id.c_str(), ok, "");
    std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), ok, "");
    // incoming lanes
    std::vector<MSLane*> incomingLanes;
    parseLanes(id, attrs.getStringSecure(SUMO_ATTR_INCLANES, ""), incomingLanes, ok);
    // internal lanes
    std::vector<MSLane*> internalLanes;
    if (MSGlobals::gUsingInternalLanes) {
        parseLanes(id, attrs.getStringSecure(SUMO_ATTR_INTLANES, ""), internalLanes, ok);
    }
    if (!ok) {
        myCurrentIsBroken = true;
    } else {
        try {
            myJunctionControlBuilder.openJunction(id, key, type, Position(x, y, z), shape, incomingLanes, internalLanes, name);
        } catch (InvalidArgument& e) {
            WRITE_ERROR(e.what() + std::string("\n Can not build according junction."));
            myCurrentIsBroken = true;
        }
    }
}


void
NLHandler::parseLanes(const std::string& junctionID,
                      const std::string& def, std::vector<MSLane*>& into, bool& ok) {
    StringTokenizer st(def, " ");
    while (ok && st.hasNext()) {
        std::string laneID = st.next();
        MSLane* lane = MSLane::dictionary(laneID);
        if (!MSGlobals::gUsingInternalLanes && laneID[0] == ':') {
            continue;
        }
        if (lane == nullptr) {
            WRITE_ERRORF(TL("An unknown lane ('%') was tried to be set as incoming to junction '%'."), laneID, junctionID);
            ok = false;
            continue;
        }
        into.push_back(lane);
    }
}
// ----

void
NLHandler::addParam(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    const std::string key = attrs.get<std::string>(SUMO_ATTR_KEY, nullptr, ok);
    // circumventing empty string test
    const std::string val = attrs.hasAttribute(SUMO_ATTR_VALUE) ? attrs.getString(SUMO_ATTR_VALUE) : "";
    if (myLastParameterised.size() > 0 && myLastParameterised.back() != nullptr) {
        myLastParameterised.back()->setParameter(key, val);
    }
    // set
    if (ok && myAmParsingTLLogicOrJunction) {
        assert(key != "");
        myJunctionControlBuilder.addParam(key, val);
        if (myNetIsLoaded) {
            myHaveSeenTLSParams = true;
        }
    }
}


void
NLHandler::openWAUT(const SUMOSAXAttributes& attrs) {
    myCurrentIsBroken = false;
    bool ok = true;
    // get the id, report an error if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    if (!ok) {
        myCurrentIsBroken = true;
        return;
    }
    SUMOTime refTime = attrs.getOptSUMOTimeReporting(SUMO_ATTR_REF_TIME, id.c_str(), ok, 0);
    SUMOTime period = attrs.getOptSUMOTimeReporting(SUMO_ATTR_PERIOD, id.c_str(), ok, 0);
    std::string startProg = attrs.get<std::string>(SUMO_ATTR_START_PROG, id.c_str(), ok);
    if (!ok) {
        myCurrentIsBroken = true;
    }
    if (!myCurrentIsBroken) {
        myCurrentWAUTID = id;
        try {
            myJunctionControlBuilder.getTLLogicControlToUse().addWAUT(refTime, id, startProg, period);
        } catch (InvalidArgument& e) {
            WRITE_ERROR(e.what());
            myCurrentIsBroken = true;
        }
    }
}


void
NLHandler::addWAUTSwitch(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    SUMOTime t = attrs.getSUMOTimeReporting(SUMO_ATTR_TIME, myCurrentWAUTID.c_str(), ok);
    std::string to = attrs.get<std::string>(SUMO_ATTR_TO, myCurrentWAUTID.c_str(), ok);
    if (!ok) {
        myCurrentIsBroken = true;
    }
    if (!myCurrentIsBroken) {
        try {
            myJunctionControlBuilder.getTLLogicControlToUse().addWAUTSwitch(myCurrentWAUTID, t, to);
        } catch (InvalidArgument& e) {
            WRITE_ERROR(e.what());
            myCurrentIsBroken = true;
        }
    }
}


void
NLHandler::addWAUTJunction(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    std::string wautID = attrs.get<std::string>(SUMO_ATTR_WAUT_ID, nullptr, ok);
    std::string junctionID = attrs.get<std::string>(SUMO_ATTR_JUNCTION_ID, nullptr, ok);
    std::string procedure = attrs.getOpt<std::string>(SUMO_ATTR_PROCEDURE, nullptr, ok, "");
    bool synchron = attrs.getOpt<bool>(SUMO_ATTR_SYNCHRON, nullptr, ok, false);
    if (!ok) {
        myCurrentIsBroken = true;
    }
    try {
        if (!myCurrentIsBroken) {
            myJunctionControlBuilder.getTLLogicControlToUse().addWAUTJunction(wautID, junctionID, procedure, synchron);
        }
    } catch (InvalidArgument& e) {
        WRITE_ERROR(e.what());
        myCurrentIsBroken = true;
    }
}


void
NLHandler::addRequest(const SUMOSAXAttributes& attrs) {
    if (myCurrentIsBroken) {
        return;
    }
    bool ok = true;
    int request = attrs.get<int>(SUMO_ATTR_INDEX, nullptr, ok);
    bool cont = false;
    cont = attrs.getOpt<bool>(SUMO_ATTR_CONT, nullptr, ok, false);
    std::string response = attrs.get<std::string>(SUMO_ATTR_RESPONSE, nullptr, ok);
    std::string foes = attrs.get<std::string>(SUMO_ATTR_FOES, nullptr, ok);
    if (!ok) {
        return;
    }
    // store received information
    if (request >= 0 && response.length() > 0) {
        try {
            myJunctionControlBuilder.addLogicItem(request, response, foes, cont);
        } catch (InvalidArgument& e) {
            WRITE_ERROR(e.what());
        }
    }
}


void
NLHandler::initJunctionLogic(const SUMOSAXAttributes& attrs) {
    if (myCurrentIsBroken) {
        return;
    }
    myAmParsingTLLogicOrJunction = true;
    bool ok = true;
    // we either a have a junction or a legacy network with ROWLogic
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    if (ok) {
        myJunctionControlBuilder.initJunctionLogic(id);
    }
}


void
NLHandler::initTrafficLightLogic(const SUMOSAXAttributes& attrs) {
    myCurrentIsBroken = false;
    myAmParsingTLLogicOrJunction = true;
    bool ok = true;
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    std::string programID = attrs.getOpt<std::string>(SUMO_ATTR_PROGRAMID, id.c_str(), ok, "<unknown>");
    TrafficLightType type = TrafficLightType::STATIC;
    std::string typeS;
    if (myJunctionControlBuilder.getTLLogicControlToUse().get(id, programID) == nullptr) {
        // SUMO_ATTR_TYPE is not needed when only modifying the offset of an
        // existing program
        typeS = attrs.get<std::string>(SUMO_ATTR_TYPE, id.c_str(), ok);
        if (!ok) {
            myCurrentIsBroken = true;
            return;
        }
        if (SUMOXMLDefinitions::TrafficLightTypes.hasString(typeS)) {
            type = SUMOXMLDefinitions::TrafficLightTypes.get(typeS);
        } else {
            WRITE_ERRORF(TL("Traffic light '%' has unknown type '%'."), id, typeS);
        }
        if (MSGlobals::gUseMesoSim && (type == TrafficLightType::ACTUATED || type == TrafficLightType::NEMA)) {
            if (!myHaveWarnedAboutInvalidTLType) {
                WRITE_WARNINGF(TL("Traffic light type '%' cannot be used in mesoscopic simulation. Using '%' as fallback."), toString(type), toString(TrafficLightType::STATIC));
                myHaveWarnedAboutInvalidTLType = true;
            }
            type = TrafficLightType::STATIC;
        }
    }
    SUMOTime offset = attrs.getOptOffsetReporting(SUMO_ATTR_OFFSET, id.c_str(), ok, 0);
    if (offset == SUMOTime_MAX) {
        offset = string2time(OptionsCont::getOptions().getString("begin"));
    }
    if (!ok) {
        myCurrentIsBroken = true;
        return;
    }
    myJunctionControlBuilder.initTrafficLightLogic(id, programID, type, offset);
}


void
NLHandler::addPhase(const SUMOSAXAttributes& attrs) {
    // try to get the phase definition
    bool ok = true;
    const std::string& id = myJunctionControlBuilder.getActiveKey();
    const SUMOTime tDefault = MSPhaseDefinition::UNSPECIFIED_DURATION;

    const SUMOTime duration = attrs.getSUMOTimeReporting(SUMO_ATTR_DURATION, myJunctionControlBuilder.getActiveKey().c_str(), ok);
    const std::string state = attrs.get<std::string>(SUMO_ATTR_STATE, nullptr, ok);
    if (duration == 0) {
        WRITE_ERROR("Duration of phase " + toString(myJunctionControlBuilder.getLoadedPhases().size())
                    + " for tlLogic '" + myJunctionControlBuilder.getActiveKey()
                    + "' program '" + myJunctionControlBuilder.getActiveSubKey() + "' is zero.");
        return;
    }
    if (!ok) {
        return;
    }
    MSPhaseDefinition* phase = new MSPhaseDefinition(duration, state);

    // if the traffic light is an actuated traffic light, try to get
    //  the minimum and maximum durations
    phase->minDuration = attrs.getOptSUMOTimeReporting(SUMO_ATTR_MINDURATION, id.c_str(), ok, duration);
    // if minDur is set but not maxDur, assume high maxDur (avoid using the absolute max in case we do some arithmetic later)
    SUMOTime defaultMaxDur = attrs.hasAttribute(SUMO_ATTR_MINDURATION) ? std::numeric_limits<int>::max() : duration;
    phase->maxDuration = attrs.getOptSUMOTimeReporting(SUMO_ATTR_MAXDURATION, id.c_str(), ok, defaultMaxDur);
    phase->earliestEnd = attrs.getOptSUMOTimeReporting(SUMO_ATTR_EARLIEST_END, id.c_str(), ok, tDefault);
    phase->latestEnd = attrs.getOptSUMOTimeReporting(SUMO_ATTR_LATEST_END, id.c_str(), ok, tDefault);
    phase->nextPhases = attrs.getOpt<std::vector<int> >(SUMO_ATTR_NEXT, id.c_str(), ok);
    phase->earlyTarget = attrs.getOpt<std::string>(SUMO_ATTR_EARLY_TARGET, id.c_str(), ok);
    phase->finalTarget = attrs.getOpt<std::string>(SUMO_ATTR_FINAL_TARGET, id.c_str(), ok);
    phase->name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), ok);

    phase->vehext = attrs.getOptSUMOTimeReporting(SUMO_ATTR_VEHICLEEXTENSION, id.c_str(), ok, tDefault);
    phase->yellow = attrs.getOptSUMOTimeReporting(SUMO_ATTR_YELLOW, id.c_str(), ok, tDefault);
    phase->red = attrs.getOptSUMOTimeReporting(SUMO_ATTR_RED, id.c_str(), ok, tDefault);

    if (attrs.hasAttribute(SUMO_ATTR_TYPE)) {
        //SOTL attributes
        //If the type attribute is not present, the parsed phase is of type "undefined" (MSPhaseDefinition constructor),
        //in this way SOTL traffic light logic can recognize the phase as unsuitable or decides other
        //behaviors. See SOTL traffic light logic implementations.
        std::string phaseTypeString;
        try {
            phaseTypeString = attrs.get<std::string>(SUMO_ATTR_TYPE, id.c_str(), ok, false);
        } catch (EmptyData&) {
            MsgHandler::getWarningInstance()->inform("Empty type definition. Assuming phase type as SUMOSOTL_TagAttrDefinitions::SOTL_ATTL_TYPE_TRANSIENT");
            phase->myTransientNotDecisional = false;
        }
        if (phaseTypeString.find("decisional") != std::string::npos) {
            phase->myTransientNotDecisional = false;
        } else if (phaseTypeString.find("transient") != std::string::npos) {
            phase->myTransientNotDecisional = true;
        } else {
            MsgHandler::getWarningInstance()->inform("SOTL_ATTL_TYPE_DECISIONAL nor SOTL_ATTL_TYPE_TRANSIENT. Assuming phase type as SUMOSOTL_TagAttrDefinitions::SOTL_ATTL_TYPE_TRANSIENT");
            phase->myTransientNotDecisional = false;
        }
        phase->myCommit = (phaseTypeString.find("commit") != std::string::npos);

        if (phaseTypeString.find("target") != std::string::npos) {
            std::string delimiter(" ,;");
            //Phase declared as target, getting targetLanes attribute
            try {
                phase->myTargetLaneSet = StringTokenizer(attrs.getStringSecure(SUMO_ATTR_TARGETLANE, ""), " ,;", true).getVector();
            } catch (EmptyData&) {
                MsgHandler::getErrorInstance()->inform("Missing targetLane definition for the target phase.");
                delete phase;
                return;
            }
        }
    }

    if (phase->maxDuration < phase->minDuration) {
        WRITE_WARNINGF(TL("maxDur % should not be smaller than minDir % in phase of tlLogic %"), phase->maxDuration, phase->minDuration, id);
        phase->maxDuration = phase->duration;
    }

    phase->myLastSwitch = string2time(OptionsCont::getOptions().getString("begin")) - 1; // SUMOTime-option
    myJunctionControlBuilder.addPhase(phase);
}


void
NLHandler::addCondition(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    const std::string value = attrs.get<std::string>(SUMO_ATTR_VALUE, id.c_str(), ok);
    if (!myJunctionControlBuilder.addCondition(id, value)) {
        WRITE_ERRORF(TL("Duplicate condition '%' in tlLogic '%'"), id, myJunctionControlBuilder.getActiveKey());
    }
}


void
NLHandler::addAssignment(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    const std::string check = attrs.get<std::string>(SUMO_ATTR_CHECK, nullptr, ok);
    const std::string value = attrs.get<std::string>(SUMO_ATTR_VALUE, id.c_str(), ok);
    myJunctionControlBuilder.addAssignment(id, check, value);
}


void
NLHandler::addFunction(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    const int nArgs = attrs.get<int>(SUMO_ATTR_NARGS, nullptr, ok);
    myJunctionControlBuilder.addFunction(id, nArgs);
}

void
NLHandler::closeFunction() {
    myJunctionControlBuilder.closeFunction();
}

void
NLHandler::addE1Detector(const SUMOSAXAttributes& attrs) {
    myCurrentIsBroken = false;
    bool ok = true;
    // get the id, report an error if not given or empty...
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    if (!ok) {
        myCurrentIsBroken = true;
        return;
    }
    const SUMOTime period = attrs.getOptPeriod(id.c_str(), ok, SUMOTime_MAX_PERIOD);
    const double position = attrs.get<double>(SUMO_ATTR_POSITION, id.c_str(), ok);
    const double length = attrs.getOpt<double>(SUMO_ATTR_LENGTH, id.c_str(), ok, 0);
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), ok, false);
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), ok, "");
    const std::string vTypes = attrs.getOpt<std::string>(SUMO_ATTR_VTYPES, id.c_str(), ok, "");
    const std::string nextEdges = attrs.getOpt<std::string>(SUMO_ATTR_NEXT_EDGES, id.c_str(), ok, "");
    const std::string lane = attrs.get<std::string>(SUMO_ATTR_LANE, id.c_str(), ok);
    const std::string file = attrs.get<std::string>(SUMO_ATTR_FILE, id.c_str(), ok);
    const std::string detectPersonsString = attrs.getOpt<std::string>(SUMO_ATTR_DETECT_PERSONS, id.c_str(), ok, "");
    const int detectPersons = parseDetectPersons(detectPersonsString, id, ok);
    if (!ok) {
        myCurrentIsBroken = true;
        return;
    }
    try {
        Parameterised* det = myDetectorBuilder.buildInductLoop(id, lane, position, length, period,
                             FileHelpers::checkForRelativity(file, getFileName()),
                             friendlyPos, name, vTypes, nextEdges, detectPersons);
        myLastParameterised.push_back(det);
    } catch (InvalidArgument& e) {
        myCurrentIsBroken = true;
        WRITE_ERROR(e.what());
    } catch (IOError& e) {
        myCurrentIsBroken = true;
        WRITE_ERROR(e.what());
    }
}


void
NLHandler::addInstantE1Detector(const SUMOSAXAttributes& attrs) {
    myCurrentIsBroken = false;
    bool ok = true;
    // get the id, report an error if not given or empty...
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    if (!ok) {
        myCurrentIsBroken = true;
        return;
    }
    const double position = attrs.get<double>(SUMO_ATTR_POSITION, id.c_str(), ok);
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), ok, false);
    const std::string lane = attrs.get<std::string>(SUMO_ATTR_LANE, id.c_str(), ok);
    const std::string file = attrs.get<std::string>(SUMO_ATTR_FILE, id.c_str(), ok);
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), ok, "");
    const std::string vTypes = attrs.getOpt<std::string>(SUMO_ATTR_VTYPES, id.c_str(), ok, "");
    const std::string nextEdges = attrs.getOpt<std::string>(SUMO_ATTR_NEXT_EDGES, id.c_str(), ok, "");
    const std::string detectPersonsString = attrs.getOpt<std::string>(SUMO_ATTR_DETECT_PERSONS, id.c_str(), ok, "");
    const int detectPersons = parseDetectPersons(detectPersonsString, id, ok);
    if (!ok) {
        myCurrentIsBroken = true;
        return;
    }
    try {
        Parameterised* det = myDetectorBuilder.buildInstantInductLoop(id, lane, position, FileHelpers::checkForRelativity(file, getFileName()), friendlyPos, name, vTypes, nextEdges, detectPersons);
        myLastParameterised.push_back(det);
    } catch (InvalidArgument& e) {
        WRITE_ERROR(e.what());
    } catch (IOError& e) {
        WRITE_ERROR(e.what());
    }
    myCurrentIsBroken = true;
}


void
NLHandler::addVTypeProbeDetector(const SUMOSAXAttributes& attrs) {
    WRITE_WARNING(TL("VTypeProbes are deprecated. Use fcd-output devices (assigned to the vType) instead."));
    bool ok = true;
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    SUMOTime period = attrs.getOptPeriod(id.c_str(), ok, SUMOTime_MAX_PERIOD);
    std::string type = attrs.getStringSecure(SUMO_ATTR_TYPE, "");
    std::string file = attrs.get<std::string>(SUMO_ATTR_FILE, id.c_str(), ok);
    if (!ok) {
        return;
    }
    try {
        myDetectorBuilder.buildVTypeProbe(id, type, period, FileHelpers::checkForRelativity(file, getFileName()));
    } catch (InvalidArgument& e) {
        WRITE_ERROR(e.what());
    } catch (IOError& e) {
        WRITE_ERROR(e.what());
    }
}


void
NLHandler::addRouteProbeDetector(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    SUMOTime period = attrs.getOptPeriod(id.c_str(), ok, SUMOTime_MAX_PERIOD);
    SUMOTime begin = attrs.getOptSUMOTimeReporting(SUMO_ATTR_BEGIN, id.c_str(), ok, -1);
    std::string edge = attrs.get<std::string>(SUMO_ATTR_EDGE, id.c_str(), ok);
    std::string file = attrs.get<std::string>(SUMO_ATTR_FILE, id.c_str(), ok);
    const std::string vTypes = attrs.getOpt<std::string>(SUMO_ATTR_VTYPES, id.c_str(), ok, "");
    if (!ok) {
        return;
    }
    try {
        myDetectorBuilder.buildRouteProbe(id, edge, period, begin,
                                          FileHelpers::checkForRelativity(file, getFileName()), vTypes);
    } catch (InvalidArgument& e) {
        WRITE_ERROR(e.what());
    } catch (IOError& e) {
        WRITE_ERROR(e.what());
    }
}



void
NLHandler::addE2Detector(const SUMOSAXAttributes& attrs) {
    myCurrentIsBroken = false;
    // check whether this is a detector connected to a tls and optionally to a link
    bool ok = true;
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    const std::string lsaid = attrs.getOpt<std::string>(SUMO_ATTR_TLID, id.c_str(), ok, "");
    const std::string toLane = attrs.getOpt<std::string>(SUMO_ATTR_TO, id.c_str(), ok, "");
    const SUMOTime haltingTimeThreshold = attrs.getOptSUMOTimeReporting(SUMO_ATTR_HALTING_TIME_THRESHOLD, id.c_str(), ok, TIME2STEPS(1));
    const double haltingSpeedThreshold = attrs.getOpt<double>(SUMO_ATTR_HALTING_SPEED_THRESHOLD, id.c_str(), ok, 5.0f / 3.6f);
    const double jamDistThreshold = attrs.getOpt<double>(SUMO_ATTR_JAM_DIST_THRESHOLD, id.c_str(), ok, 10.0f);
    double position = attrs.getOpt<double>(SUMO_ATTR_POSITION, id.c_str(), ok, std::numeric_limits<double>::max());
    const double length = attrs.getOpt<double>(SUMO_ATTR_LENGTH, id.c_str(), ok, std::numeric_limits<double>::max());
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), ok, false);
    const bool showDetector = attrs.getOpt<bool>(SUMO_ATTR_SHOW_DETECTOR, id.c_str(), ok, true);
    const std::string contStr = attrs.getOpt<std::string>(SUMO_ATTR_CONT, id.c_str(), ok, "");
    if (contStr != "") {
        WRITE_WARNINGF(TL("Ignoring deprecated argument 'cont' for E2 detector '%'"), id);
    }
    std::string lane = attrs.getOpt<std::string>(SUMO_ATTR_LANE, id.c_str(), ok, "");
    const std::string file = attrs.get<std::string>(SUMO_ATTR_FILE, id.c_str(), ok);
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), ok, "");
    const std::string vTypes = attrs.getOpt<std::string>(SUMO_ATTR_VTYPES, id.c_str(), ok, "");
    const std::string nextEdges = attrs.getOpt<std::string>(SUMO_ATTR_NEXT_EDGES, id.c_str(), ok, "");

    double endPosition = attrs.getOpt<double>(SUMO_ATTR_ENDPOS, id.c_str(), ok, std::numeric_limits<double>::max());
    const std::string lanes = attrs.getOpt<std::string>(SUMO_ATTR_LANES, id.c_str(), ok, ""); // lanes has priority to lane
    const std::string detectPersonsString = attrs.getOpt<std::string>(SUMO_ATTR_DETECT_PERSONS, id.c_str(), ok, "");
    const int detectPersons = parseDetectPersons(detectPersonsString, id, ok);
    if (!ok) {
        myCurrentIsBroken = true;
        return;
    }

    bool lanesGiven = lanes != "";
    bool laneGiven = lane != "";
    if (!(lanesGiven || laneGiven)) {
        // in absence of any lane-specification assume specification by id
        WRITE_WARNING(TL("Trying to specify detector's lane by the given id since the argument 'lane' is missing."))
        lane = id;
        laneGiven = true;
    }
    bool lengthGiven = length != std::numeric_limits<double>::max();
    bool posGiven = position != std::numeric_limits<double>::max();
    bool endPosGiven = endPosition != std::numeric_limits<double>::max();
    bool lsaGiven = lsaid != "";
    bool toLaneGiven = toLane != "";

    MSLane* clane = nullptr;
    std::vector<MSLane*> clanes;
    if (lanesGiven) {
        // If lanes is given, endPos and startPos are required. lane, and length are ignored
        std::string seps = " ,\t\n";
        StringTokenizer st = StringTokenizer(lanes, seps, true);
//        std::cout << "Parsing lanes..." << std::endl;
        while (st.hasNext()) {
            std::string nextLaneID = st.next();
//            std::cout << "Next: " << nextLaneID << std::endl;
            if (nextLaneID.find_first_of(seps) != nextLaneID.npos) {
                continue;
            }
            clane = myDetectorBuilder.getLaneChecking(nextLaneID, SUMO_TAG_E2DETECTOR, id);
            clanes.push_back(clane);
        }
        if (clanes.size() == 0) {
            throw InvalidArgument("Malformed argument 'lanes' for E2Detector '" + id + "'.\nSpecify 'lanes' as a sequence of lane-IDs separated by whitespace or comma (',')");
        }
        if (laneGiven) {
            WRITE_WARNING("Ignoring argument 'lane' for E2Detector '" + id + "' since argument 'lanes' was given.\n"
                          "Usage combinations for positional specification: [lane, pos, length], [lane, endPos, length], or [lanes, pos, endPos]");
        }
        if (lengthGiven) {
            WRITE_WARNING("Ignoring argument 'length' for E2Detector '" + id + "' since argument 'lanes' was given.\n"
                          "Usage combinations for positional specification: [lane, pos, length], [lane, endPos, length], or [lanes, pos, endPos]");
        }
        if (!posGiven) {
            // assuming start pos == lane start
            position = 0;
            WRITE_WARNINGF(TL("Missing argument 'pos' for E2Detector '%'. Assuming detector start == lane start of lane '%'."), id, clanes[0]->getID());
        }
        if (!endPosGiven) {
            // assuming end pos == lane end
            endPosition = clanes[clanes.size() - 1]->getLength();
            WRITE_WARNINGF(TL("Missing argument 'endPos' for E2Detector '%'. Assuming detector end == lane end of lane '%'."), id, clanes[clanes.size() - 1]->getID());
        }

    } else {
        if (!laneGiven) {
            std::stringstream ss;
            ss << "Missing argument 'lane' for E2Detector '" << id << "'."
               << "\nUsage combinations for positional specification: [lane, pos, length], [lane, endPos, length], or [lanes, pos, endPos]";
            throw InvalidArgument(ss.str());
        }
        clane = myDetectorBuilder.getLaneChecking(lane, SUMO_TAG_E2DETECTOR, id);

        if (posGiven) {
            // start pos is given
            if (endPosGiven && lengthGiven) {
                std::stringstream ss;
                ss << "Ignoring argument 'endPos' for E2Detector '" << id << "' since argument 'pos' was given."
                   << "\nUsage combinations for positional specification: [lane, pos, length], [lane, endPos, length], or [lanes, pos, endPos]";
                WRITE_WARNING(ss.str());
                endPosition = std::numeric_limits<double>::max();
            }
            if (!lengthGiven && !endPosGiven) {
                std::stringstream ss;
                ss << "Missing arguments 'length'/'endPos' for E2Detector '" << id << "'. Assuming detector end == lane end of lane '" << lane << "'.";
                WRITE_WARNING(ss.str());
                endPosition = clane->getLength();
            }
        } else if (endPosGiven) {
            // endPos is given, pos is not given
            if (!lengthGiven) {
                std::stringstream ss;
                ss << "Missing arguments 'length'/'pos' for E2Detector '" << id << "'. Assuming detector start == lane start of lane '" << lane << "'.";
                WRITE_WARNING(ss.str());
            }
        } else {
            std::stringstream ss;
            if (lengthGiven && fabs(length - clane->getLength()) > NUMERICAL_EPS) {
                ss << "Incomplete positional specification for E2Detector '" << id << "'."
                   << "\nUsage combinations for positional specification: [lane, pos, length], [lane, endPos, length], or [lanes, pos, endPos]";
                throw InvalidArgument(ss.str());
            }
            endPosition = clane->getLength();
            position = 0;
            ss << "Missing arguments 'pos'/'endPos' for E2Detector '" << id << "'. Assuming that the detector covers the whole lane '" << lane << "'.";
            WRITE_WARNING(ss.str());
        }
    }

    // Period

    SUMOTime period;
    if (!lsaGiven) {
        period = attrs.getOptPeriod(id.c_str(), ok, SUMOTime_MAX_PERIOD);
        if (!ok) {
            myCurrentIsBroken = true;
            return;
        }
    } else {
        period = attrs.getPeriod(id.c_str(), ok, false);
    }

    // TLS
    MSTLLogicControl::TLSLogicVariants* tlls = nullptr;
    if (lsaGiven) {
        tlls = &myJunctionControlBuilder.getTLLogic(lsaid);
        if (tlls->getActive() == nullptr) {
            throw InvalidArgument("The detector '" + id + "' refers to an unknown lsa '" + lsaid + "'.");
        }
        if (period != -1) {
            WRITE_WARNINGF(TL("Ignoring argument 'period' for E2Detector '%' since argument 'tl' was given."), id);
            period = -1;
        }
    }

    // Link
    MSLane* cToLane = nullptr;
    if (toLaneGiven) {
        cToLane = myDetectorBuilder.getLaneChecking(toLane, SUMO_TAG_E2DETECTOR, id);
    }

    // File
    std::string filename;
    try {
        filename = FileHelpers::checkForRelativity(file, getFileName());
    } catch (IOError& e) {
        WRITE_ERROR(e.what());
    }

    Parameterised* det;
    // Build detector
    if (lanesGiven) {
        // specification by a lane sequence
        det = myDetectorBuilder.buildE2Detector(id, clanes, position, endPosition, filename, period,
                                                haltingTimeThreshold, haltingSpeedThreshold, jamDistThreshold,
                                                name, vTypes, nextEdges, detectPersons, friendlyPos, showDetector,
                                                tlls, cToLane);
    } else {
        // specification by start or end lane
        det = myDetectorBuilder.buildE2Detector(id, clane, position, endPosition, length, filename, period,
                                                haltingTimeThreshold, haltingSpeedThreshold, jamDistThreshold,
                                                name, vTypes, nextEdges, detectPersons, friendlyPos, showDetector,
                                                tlls, cToLane);
    }
    myLastParameterised.push_back(det);
}


void
NLHandler::beginE3Detector(const SUMOSAXAttributes& attrs) {
    myCurrentIsBroken = false;
    bool ok = true;
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    const SUMOTime period = attrs.getOptPeriod(id.c_str(), ok, SUMOTime_MAX_PERIOD);
    const SUMOTime haltingTimeThreshold = attrs.getOptSUMOTimeReporting(SUMO_ATTR_HALTING_TIME_THRESHOLD, id.c_str(), ok, TIME2STEPS(1));
    const double haltingSpeedThreshold = attrs.getOpt<double>(SUMO_ATTR_HALTING_SPEED_THRESHOLD, id.c_str(), ok, 5.0f / 3.6f);
    const std::string file = attrs.get<std::string>(SUMO_ATTR_FILE, id.c_str(), ok);
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), ok, "");
    const std::string vTypes = attrs.getOpt<std::string>(SUMO_ATTR_VTYPES, id.c_str(), ok, "");
    const std::string nextEdges = attrs.getOpt<std::string>(SUMO_ATTR_NEXT_EDGES, id.c_str(), ok, "");
    const bool openEntry = attrs.getOpt<bool>(SUMO_ATTR_OPEN_ENTRY, id.c_str(), ok, false);
    const bool expectArrival = attrs.getOpt<bool>(SUMO_ATTR_EXPECT_ARRIVAL, id.c_str(), ok, false);
    const std::string detectPersonsString = attrs.getOpt<std::string>(SUMO_ATTR_DETECT_PERSONS, id.c_str(), ok, "");
    const int detectPersons = parseDetectPersons(detectPersonsString, id, ok);
    if (!ok) {
        myCurrentIsBroken = true;
        return;
    }
    try {
        Parameterised* det = myDetectorBuilder.beginE3Detector(id,
                             FileHelpers::checkForRelativity(file, getFileName()),
                             period, haltingSpeedThreshold, haltingTimeThreshold, name, vTypes, nextEdges, detectPersons, openEntry, expectArrival);
        myLastParameterised.push_back(det);
    } catch (InvalidArgument& e) {
        myCurrentIsBroken = true;
        WRITE_ERROR(e.what());
    } catch (IOError& e) {
        myCurrentIsBroken = true;
        WRITE_ERROR(e.what());
    }
}


void
NLHandler::addE3Entry(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    const double position = attrs.get<double>(SUMO_ATTR_POSITION, myDetectorBuilder.getCurrentE3ID().c_str(), ok);
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, myDetectorBuilder.getCurrentE3ID().c_str(), ok, false);
    const std::string lane = attrs.get<std::string>(SUMO_ATTR_LANE, myDetectorBuilder.getCurrentE3ID().c_str(), ok);
    if (!ok) {
        return;
    }
    myDetectorBuilder.addE3Entry(lane, position, friendlyPos);
}


void
NLHandler::addE3Exit(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    const double position = attrs.get<double>(SUMO_ATTR_POSITION, myDetectorBuilder.getCurrentE3ID().c_str(), ok);
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, myDetectorBuilder.getCurrentE3ID().c_str(), ok, false);
    const std::string lane = attrs.get<std::string>(SUMO_ATTR_LANE, myDetectorBuilder.getCurrentE3ID().c_str(), ok);
    if (!ok) {
        return;
    }
    myDetectorBuilder.addE3Exit(lane, position, friendlyPos);
}


void
NLHandler::addEdgeLaneMeanData(const SUMOSAXAttributes& attrs, int objecttype) {
    bool ok = true;
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    const double maxTravelTime = attrs.getOpt<double>(SUMO_ATTR_MAX_TRAVELTIME, id.c_str(), ok, 100000);
    const double minSamples = attrs.getOpt<double>(SUMO_ATTR_MIN_SAMPLES, id.c_str(), ok, 0);
    const double haltingSpeedThreshold = attrs.getOpt<double>(SUMO_ATTR_HALTING_SPEED_THRESHOLD, id.c_str(), ok, POSITION_EPS);
    const std::string excludeEmpty = attrs.getOpt<std::string>(SUMO_ATTR_EXCLUDE_EMPTY, id.c_str(), ok, "false");
    const bool withInternal = attrs.getOpt<bool>(SUMO_ATTR_WITH_INTERNAL, id.c_str(), ok, false);
    const bool trackVehicles = attrs.getOpt<bool>(SUMO_ATTR_TRACK_VEHICLES, id.c_str(), ok, false);
    const std::string detectPersonsString = attrs.getOpt<std::string>(SUMO_ATTR_DETECT_PERSONS, id.c_str(), ok, "");
    const std::string file = attrs.get<std::string>(SUMO_ATTR_FILE, id.c_str(), ok);
    const std::string type = attrs.getOpt<std::string>(SUMO_ATTR_TYPE, id.c_str(), ok, "performance");
    std::string vtypes = attrs.getOpt<std::string>(SUMO_ATTR_VTYPES, id.c_str(), ok, "");
    const std::string writeAttributes = attrs.getOpt<std::string>(SUMO_ATTR_WRITE_ATTRIBUTES, id.c_str(), ok, "");
    const SUMOTime period = attrs.getOptPeriod(id.c_str(), ok, -1);
    const SUMOTime begin = attrs.getOptSUMOTimeReporting(SUMO_ATTR_BEGIN, id.c_str(), ok, string2time(OptionsCont::getOptions().getString("begin")));
    const SUMOTime end = attrs.getOptSUMOTimeReporting(SUMO_ATTR_END, id.c_str(), ok, string2time(OptionsCont::getOptions().getString("end")));
    std::vector<std::string> edgeIDs = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_EDGES, id.c_str(), ok);
    const std::string edgesFile = attrs.getOpt<std::string>(SUMO_ATTR_EDGESFILE, id.c_str(), ok, "");
    const std::string aggregateStr = attrs.getOpt<std::string>(SUMO_ATTR_AGGREGATE, id.c_str(), ok, "false");
    AggregateType aggregate = AggregateType::NO;
    if (aggregateStr == "taz") {
        aggregate = AggregateType::TAZ;
    } else if (StringUtils::toBool(aggregateStr)) {
        aggregate = AggregateType::YES;
    }
    if (!ok) {
        return;
    }
    int detectPersons = 0;
    for (std::string mode : StringTokenizer(detectPersonsString).getVector()) {
        if (SUMOXMLDefinitions::PersonModeValues.hasString(mode)) {
            detectPersons |= (int)SUMOXMLDefinitions::PersonModeValues.get(mode);
        } else {
            WRITE_ERRORF(TL("Invalid person mode '%' in edgeData definition '%'"), mode, id);
            return;
        }
    }
    if (edgesFile != "") {
        std::ifstream strm(edgesFile.c_str());
        if (!strm.good()) {
            throw ProcessError("Could not load names of edges for edgeData definition '" + id + "' from '" + edgesFile + "'.");
        }
        while (strm.good()) {
            std::string name;
            strm >> name;
            // maybe we're loading an edge-selection
            if (StringUtils::startsWith(name, "edge:")) {
                edgeIDs.push_back(name.substr(5));
            } else if (name != "") {
                edgeIDs.push_back(name);
            }
        }
    }
    std::vector<MSEdge*> edges;
    for (const std::string& edgeID : edgeIDs) {
        MSEdge* edge = MSEdge::dictionary(edgeID);
        if (edge == nullptr) {
            WRITE_ERRORF(TL("Unknown edge '%' in edgeData definition '%'"), edgeID, id);
            return;
        }
        edges.push_back(edge);
    }
    bool useLanes = objecttype == SUMO_TAG_MEANDATA_LANE;
    if (useLanes && MSGlobals::gUseMesoSim && !OptionsCont::getOptions().getBool("meso-lane-queue")) {
        WRITE_WARNINGF(TL("LaneData '%' requested for mesoscopic simulation but --meso-lane-queue is not active. Falling back to edgeData."), id);
        useLanes = false;
    }
    try {
        myDetectorBuilder.createEdgeLaneMeanData(id, period, begin, end,
                type, useLanes, excludeEmpty, withInternal, trackVehicles, detectPersons,
                maxTravelTime, minSamples, haltingSpeedThreshold, vtypes, writeAttributes, edges, aggregate,
                FileHelpers::checkForRelativity(file, getFileName()));
    } catch (InvalidArgument& e) {
        WRITE_ERROR(e.what());
    } catch (IOError& e) {
        WRITE_ERROR(e.what());
    }
}


void
NLHandler::addConnection(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    const std::string fromID = attrs.get<std::string>(SUMO_ATTR_FROM, nullptr, ok);
    const std::string toID = attrs.get<std::string>(SUMO_ATTR_TO, nullptr, ok);
    if (!MSGlobals::gUsingInternalLanes && (fromID[0] == ':' || toID[0] == ':')) {
        std::string tlID = attrs.getOpt<std::string>(SUMO_ATTR_TLID, nullptr, ok, "");
        if (tlID != "") {
            int tlLinkIdx = attrs.get<int>(SUMO_ATTR_TLLINKINDEX, nullptr, ok);
            myJunctionControlBuilder.getTLLogic(tlID).ignoreLinkIndex(tlLinkIdx);
        }
        return;
    }

    myCurrentLink = nullptr;
    try {
        const int fromLaneIdx = attrs.get<int>(SUMO_ATTR_FROM_LANE, nullptr, ok);
        const int toLaneIdx = attrs.get<int>(SUMO_ATTR_TO_LANE, nullptr, ok);
        LinkDirection dir = parseLinkDir(attrs.get<std::string>(SUMO_ATTR_DIR, nullptr, ok));
        LinkState state = parseLinkState(attrs.get<std::string>(SUMO_ATTR_STATE, nullptr, ok));
        const double foeVisibilityDistance = attrs.getOpt<double>(SUMO_ATTR_VISIBILITY_DISTANCE, nullptr, ok, state == LINKSTATE_ZIPPER ? 100 : 4.5);
        const bool keepClear = attrs.getOpt<bool>(SUMO_ATTR_KEEP_CLEAR, nullptr, ok, true);
        const bool indirect = attrs.getOpt<bool>(SUMO_ATTR_INDIRECT, nullptr, ok, false);
        std::string tlID = attrs.getOpt<std::string>(SUMO_ATTR_TLID, nullptr, ok, "");
        std::string viaID = attrs.getOpt<std::string>(SUMO_ATTR_VIA, nullptr, ok, "");

        MSEdge* from = MSEdge::dictionaryHint(fromID, myPreviousEdgeIdx);
        if (from == nullptr) {
            WRITE_ERRORF(TL("Unknown from-edge '%' in connection."), fromID);
            return;
        }
        myPreviousEdgeIdx = from->getNumericalID();
        MSEdge* to = MSEdge::dictionary(toID);
        if (to == nullptr) {
            WRITE_ERRORF(TL("Unknown to-edge '%' in connection."), toID);
            return;
        }
        if (fromLaneIdx < 0 || fromLaneIdx >= (int)from->getLanes().size() ||
                toLaneIdx < 0 || toLaneIdx >= (int)to->getLanes().size()) {
            WRITE_ERRORF(TL("Invalid lane index in connection from '%' to '%'."), from->getID(), to->getID());
            return;
        }
        MSLane* fromLane = from->getLanes()[fromLaneIdx];
        MSLane* toLane = to->getLanes()[toLaneIdx];
        assert(fromLane);
        assert(toLane);

        MSTrafficLightLogic* logic = nullptr;
        int tlLinkIdx = -1;
        if (tlID != "") {
            tlLinkIdx = attrs.get<int>(SUMO_ATTR_TLLINKINDEX, nullptr, ok);
            // make sure that the index is in range
            logic = myJunctionControlBuilder.getTLLogic(tlID).getActive();
            if ((tlLinkIdx < 0 || tlLinkIdx >= (int)logic->getCurrentPhaseDef().getState().size())
                    && logic->getLogicType() != TrafficLightType::RAIL_SIGNAL
                    && logic->getLogicType() != TrafficLightType::RAIL_CROSSING) {
                WRITE_ERROR("Invalid " + toString(SUMO_ATTR_TLLINKINDEX) + " '" + toString(tlLinkIdx) +
                            "' in connection controlled by '" + tlID + "'");
                return;
            }
            if (!ok) {
                return;
            }
        }
        double length;
        // build the link
        MSLane* via = nullptr;
        if (viaID != "" && MSGlobals::gUsingInternalLanes) {
            via = MSLane::dictionary(viaID);
            if (via == nullptr) {
                WRITE_ERROR("An unknown lane ('" + viaID +
                            "') should be set as a via-lane for lane '" + toLane->getID() + "'.");
                return;
            }
            length = via->getLength();
        } else if (toLane->isCrossing()) {
            length = toLane->getLength();
        } else {
            length = fromLane->getShape()[-1].distanceTo(toLane->getShape()[0]);
        }
        myCurrentLink = new MSLink(fromLane, toLane, via, dir, state, length, foeVisibilityDistance, keepClear, logic, tlLinkIdx, indirect);
        if (via != nullptr) {
            via->addIncomingLane(fromLane, myCurrentLink);
        } else {
            toLane->addIncomingLane(fromLane, myCurrentLink);
        }
        toLane->addApproachingLane(fromLane, myNetworkVersion < MMVersion(0, 25));

        // if a traffic light is responsible for it, inform the traffic light
        // check whether this link is controlled by a traffic light
        // we can not reuse logic here because it might be an inactive one
        if (tlID != "") {
            myJunctionControlBuilder.getTLLogic(tlID).addLink(myCurrentLink, fromLane, tlLinkIdx);
        }
        // add the link
        fromLane->addLink(myCurrentLink);

    } catch (InvalidArgument& e) {
        WRITE_ERROR(e.what());
    }
}


void
NLHandler::addConflict(const SUMOSAXAttributes& attrs) {
    if (myCurrentLink == nullptr) {
        throw InvalidArgument(toString(SUMO_TAG_CONFLICT) + " must occur within a " + toString(SUMO_TAG_CONNECTION) + " element");
    }
    if (!MSGlobals::gUsingInternalLanes) {
        return;
    }
    bool ok = true;
    const std::string fromID = attrs.get<std::string>(SUMO_ATTR_FROM, nullptr, ok);
    const std::string toID = attrs.get<std::string>(SUMO_ATTR_TO, nullptr, ok);
    const int fromLaneIdx = attrs.get<int>(SUMO_ATTR_FROM_LANE, nullptr, ok);
    const int toLaneIdx = attrs.get<int>(SUMO_ATTR_TO_LANE, nullptr, ok);
    double startPos = attrs.get<double>(SUMO_ATTR_STARTPOS, nullptr, ok);
    double endPos = attrs.get<double>(SUMO_ATTR_ENDPOS, nullptr, ok);
    MSEdge* from = MSEdge::dictionary(fromID);
    if (from == nullptr) {
        WRITE_ERRORF(TL("Unknown from-edge '%' in conflict."), fromID);
        return;
    }
    MSEdge* to = MSEdge::dictionary(toID);
    if (to == nullptr) {
        WRITE_ERRORF(TL("Unknown to-edge '%' in conflict."), toID);
        return;
    }
    if (fromLaneIdx < 0 || fromLaneIdx >= (int)from->getLanes().size() ||
            toLaneIdx < 0 || toLaneIdx >= (int)to->getLanes().size()) {
        WRITE_ERRORF(TL("Invalid lane index in conflict with '%' to '%'."), from->getID(), to->getID());
        return;
    }
    MSLane* fromLane = from->getLanes()[fromLaneIdx];
    MSLane* toLane = to->getLanes()[toLaneIdx];
    assert(fromLane);
    assert(toLane);
    myCurrentLink->addCustomConflict(fromLane, toLane, startPos, endPos);
}


LinkDirection
NLHandler::parseLinkDir(const std::string& dir) {
    if (SUMOXMLDefinitions::LinkDirections.hasString(dir)) {
        return SUMOXMLDefinitions::LinkDirections.get(dir);
    } else {
        throw InvalidArgument("Unrecognised link direction '" + dir + "'.");
    }
}


LinkState
NLHandler::parseLinkState(const std::string& state) {
    if (SUMOXMLDefinitions::LinkStates.hasString(state)) {
        return SUMOXMLDefinitions::LinkStates.get(state);
    } else {
        if (state == "t") { // legacy networks
            // WRITE_WARNING(TL("Obsolete link state 't'. Use 'o' instead"));
            return LINKSTATE_TL_OFF_BLINKING;
        } else {
            throw InvalidArgument("Unrecognised link state '" + state + "'.");
        }
    }
}


// ----------------------------------
void
NLHandler::setLocation(const SUMOSAXAttributes& attrs) {
    if (myNetIsLoaded) {
        //WRITE_WARNING(TL("POIs and Polygons should be loaded using option --po-files"))
        return;
    }
    bool ok = true;
    PositionVector s = attrs.get<PositionVector>(SUMO_ATTR_NET_OFFSET, nullptr, ok);
    Boundary convBoundary = attrs.get<Boundary>(SUMO_ATTR_CONV_BOUNDARY, nullptr, ok);
    Boundary origBoundary = attrs.get<Boundary>(SUMO_ATTR_ORIG_BOUNDARY, nullptr, ok);
    std::string proj = attrs.get<std::string>(SUMO_ATTR_ORIG_PROJ, nullptr, ok);
    if (ok) {
        Position networkOffset = s[0];
        GeoConvHelper::init(proj, networkOffset, origBoundary, convBoundary);
        if (OptionsCont::getOptions().getBool("fcd-output.geo") && !GeoConvHelper::getFinal().usingGeoProjection()) {
            WRITE_WARNING(TL("no valid geo projection loaded from network. fcd-output.geo will not work"));
        }
    }
}


void
NLHandler::addDistrict(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    myCurrentIsBroken = false;
    // get the id, report an error if not given or empty...
    myCurrentDistrictID = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
    if (!ok) {
        myCurrentIsBroken = true;
        return;
    }
    try {
        const std::string sinkID = myCurrentDistrictID + "-sink";
        const std::string sourceID = myCurrentDistrictID + "-source";

        MSEdge* sink = MSEdge::dictionary(sinkID);
        if (sink == nullptr) {
            sink = myEdgeControlBuilder.buildEdge(sinkID, SumoXMLEdgeFunc::CONNECTOR, "", "", "", -1, 0);
            MSEdge::dictionary(sinkID, sink);
            sink->initialize(new std::vector<MSLane*>());
        } else {
            if (OptionsCont::getOptions().getBool("junction-taz")
                    && myNet.getJunctionControl().get(myCurrentDistrictID) != nullptr) {
                // overwrite junction taz
                sink->resetTAZ(myNet.getJunctionControl().get(myCurrentDistrictID));
                WRITE_WARNINGF(TL("Replacing junction-taz '%' with loaded TAZ."), myCurrentDistrictID);
            } else {
                throw InvalidArgument("Another edge with the id '" + sinkID + "' exists.");
            }
        }
        MSEdge* source = MSEdge::dictionary(sourceID);
        if (source == nullptr) {
            source = myEdgeControlBuilder.buildEdge(sourceID, SumoXMLEdgeFunc::CONNECTOR, "", "", "", -1, 0);
            MSEdge::dictionary(sourceID, source);
            source->initialize(new std::vector<MSLane*>());
        } else {
            if (OptionsCont::getOptions().getBool("junction-taz")
                    && myNet.getJunctionControl().get(myCurrentDistrictID) != nullptr) {
                // overwrite junction taz
                source->resetTAZ(myNet.getJunctionControl().get(myCurrentDistrictID));
            } else {
                throw InvalidArgument("Another edge with the id '" + sourceID + "' exists.");
            }
        }
        sink->setOtherTazConnector(source);
        source->setOtherTazConnector(sink);
        const std::vector<std::string>& desc = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_EDGES, myCurrentDistrictID.c_str(), ok);
        for (const std::string& eID : desc) {
            MSEdge* edge = MSEdge::dictionary(eID);
            // check whether the edge exists
            if (edge == nullptr) {
                throw InvalidArgument("The edge '" + eID + "' within district '" + myCurrentDistrictID + "' is not known.");
            }
            source->addSuccessor(edge);
            edge->addSuccessor(sink);
        }
        source->setParameter("taz", myCurrentDistrictID);
        sink->setParameter("taz", myCurrentDistrictID);
        RGBColor color = attrs.getOpt<RGBColor>(SUMO_ATTR_COLOR, myCurrentDistrictID.c_str(), ok, RGBColor::parseColor("1.0,.33,.33"));
        const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, myCurrentDistrictID.c_str(), ok, "");
        source->setParameter("tazColor", toString(color));
        sink->setParameter("tazColor", toString(color));

        if (attrs.hasAttribute(SUMO_ATTR_SHAPE)) {
            PositionVector shape = attrs.get<PositionVector>(SUMO_ATTR_SHAPE, myCurrentDistrictID.c_str(), ok);
            const bool fill = attrs.getOpt<bool>(SUMO_ATTR_FILL, myCurrentDistrictID.c_str(), ok, false);
            if (shape.size() != 0) {
                if (!myNet.getShapeContainer().addPolygon(myCurrentDistrictID, "taz", color, 0, 0, "", shape, false, fill, 1.0, false, name)) {
                    WRITE_WARNINGF(TL("Skipping visualization of taz '%', polygon already exists."), myCurrentDistrictID);
                } else {
                    myLastParameterised.push_back(myNet.getShapeContainer().getPolygons().get(myCurrentDistrictID));
                    myCurrentIsBroken = false;
                }
            }
        }
    } catch (InvalidArgument& e) {
        WRITE_ERROR(e.what());
        myCurrentIsBroken = true;
    }
}


void
NLHandler::addDistrictEdge(const SUMOSAXAttributes& attrs, bool isSource) {
    if (myCurrentIsBroken) {
        // earlier error
        return;
    }
    bool ok = true;
    std::string id = attrs.get<std::string>(SUMO_ATTR_ID, myCurrentDistrictID.c_str(), ok);
    MSEdge* succ = MSEdge::dictionary(id);
    if (succ != nullptr) {
        // connect edge
        if (isSource) {
            MSEdge::dictionary(myCurrentDistrictID + "-source")->addSuccessor(succ);
        } else {
            succ->addSuccessor(MSEdge::dictionary(myCurrentDistrictID + "-sink"));
        }
    } else {
        WRITE_ERRORF(TL("At district '%': succeeding edge '%' does not exist."), myCurrentDistrictID, id);
    }
}


void
NLHandler::addRoundabout(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    const std::vector<std::string>& edgeIDs = attrs.get<std::vector<std::string> >(SUMO_ATTR_EDGES, nullptr, ok);
    if (ok) {
        for (const std::string& eID : edgeIDs) {
            MSEdge* edge = MSEdge::dictionary(eID);
            if (edge == nullptr) {
                WRITE_ERRORF(TL("Unknown edge '%' in roundabout"), eID);
            } else {
                edge->markAsRoundabout();
            }
        }
    }
}


void
NLHandler::addDeadlock(const SUMOSAXAttributes& attrs) {
    bool ok = true;
    std::vector<std::string> signalIDs = attrs.get<std::vector<std::string>>(SUMO_ATTR_SIGNALS, nullptr, ok);
    std::vector<const MSRailSignal*> signals;
    for (const std::string& id : signalIDs) {
        const MSTrafficLightLogic* tll = myJunctionControlBuilder.getTLLogicControlToUse().getActive(id);
        const MSRailSignal* rs = dynamic_cast<const MSRailSignal*>(tll);
        if (rs != nullptr) {
            signals.push_back(rs);
        } else {
            throw InvalidArgument("Rail signal '" + toString(id) + "' in " + toString(SUMO_TAG_DEADLOCK) + " is not known");
        }
    }
    MSRailSignalControl::getInstance().addDeadlockCheck(signals);
}

// ----------------------------------
void
NLHandler::endE3Detector() {
    try {
        myDetectorBuilder.endE3Detector();
    } catch (InvalidArgument& e) {
        WRITE_ERROR(e.what());
    }
}


void
NLHandler::closeWAUT() {
    if (!myCurrentIsBroken) {
        try {
            myJunctionControlBuilder.getTLLogicControlToUse().closeWAUT(myCurrentWAUTID);
        } catch (InvalidArgument& e) {
            WRITE_ERROR(e.what());
            myCurrentIsBroken = true;
        }
    }
    myCurrentWAUTID = "";
}


Position
NLShapeHandler::getLanePos(const std::string& poiID, const std::string& laneID, double lanePos, bool friendlyPos, double lanePosLat) {
    MSLane* lane = MSLane::dictionary(laneID);
    if (lane == nullptr) {
        WRITE_ERRORF(TL("Lane '%' to place poi '%' on is not known."), laneID, poiID);
        return Position::INVALID;
    }
    if (lanePos < 0) {
        lanePos = lane->getLength() + lanePos;
    }
    if ((lanePos < 0) && friendlyPos) {
        lanePos = 0;
    }
    if ((lanePos > lane->getLength()) && friendlyPos) {
        lanePos = lane->getLength();
    }
    if (lanePos < 0 || lanePos > lane->getLength()) {
        WRITE_WARNINGF(TL("lane position % for poi '%' is not valid."), toString(lanePos), poiID);
    }
    return lane->geometryPositionAtOffset(lanePos, -lanePosLat);
}


Parameterised*
NLHandler::addPredecessorConstraint(int element, const SUMOSAXAttributes& attrs, MSRailSignal* rs) {
    if (rs == nullptr) {
        throw InvalidArgument("Rail signal '" + toString((SumoXMLTag)element) + "' constraint must occur within a railSignalConstraints element");
    }
    bool ok = true;
    const std::string tripId = attrs.get<std::string>(SUMO_ATTR_TRIP_ID, nullptr, ok);
    const std::string signalID = attrs.get<std::string>(SUMO_ATTR_TLID, nullptr, ok);
    const std::string foesString = attrs.get<std::string>(SUMO_ATTR_FOES, nullptr, ok);
    const std::vector<std::string> foes = StringTokenizer(foesString).getVector();
    const int limit = attrs.getOpt<int>(SUMO_ATTR_LIMIT, nullptr, ok, (int)foes.size());
    const bool active = attrs.getOpt<bool>(SUMO_ATTR_ACTIVE, nullptr, ok, true);

    if (!MSNet::getInstance()->getTLSControl().knows(signalID)) {
        throw InvalidArgument("Rail signal '" + signalID + "' in railSignalConstraints is not known");
    }
    MSRailSignal* signal = dynamic_cast<MSRailSignal*>(MSNet::getInstance()->getTLSControl().get(signalID).getDefault());
    if (signal == nullptr) {
        throw InvalidArgument("Traffic light '" + signalID + "' is not a rail signal");
    }
    MSRailSignalConstraint::ConstraintType type;
    switch (element) {
        case SUMO_TAG_PREDECESSOR:
            type = MSRailSignalConstraint::ConstraintType::PREDECESSOR;
            break;
        case SUMO_TAG_INSERTION_PREDECESSOR:
            type = MSRailSignalConstraint::ConstraintType::INSERTION_PREDECESSOR;
            break;
        case SUMO_TAG_FOE_INSERTION:
            type = MSRailSignalConstraint::ConstraintType::FOE_INSERTION;
            break;
        case SUMO_TAG_INSERTION_ORDER:
            type = MSRailSignalConstraint::ConstraintType::INSERTION_ORDER;
            break;
        case SUMO_TAG_BIDI_PREDECESSOR:
            type = MSRailSignalConstraint::ConstraintType::BIDI_PREDECESSOR;
            break;
        default:
            throw InvalidArgument("Unsupported rail signal constraint '" + toString((SumoXMLTag)element) + "'");
    }
    Parameterised* result = nullptr;
    if (ok) {
        for (const std::string& foe : foes) {
            MSRailSignalConstraint* c = new MSRailSignalConstraint_Predecessor(type, signal, foe, limit, active);
            rs->addConstraint(tripId, c);
            // XXX if there are multiple foes, only one constraint will receive the parameters
            result = c;
        }
    }
    return result;
}


int
NLHandler::parseDetectPersons(const std::string& detectPersonsString, const std::string& id, bool& ok) {
    int detectPersons = 0;
    for (std::string mode : StringTokenizer(detectPersonsString).getVector()) {
        if (SUMOXMLDefinitions::PersonModeValues.hasString(mode)) {
            detectPersons |= (int)SUMOXMLDefinitions::PersonModeValues.get(mode);
        } else {
            WRITE_ERRORF(TL("Invalid person mode '%' in E1 detector definition '%'"), mode, id);
            ok = false;
            return 0;
        }
    }
    return detectPersons;
}


/****************************************************************************/
