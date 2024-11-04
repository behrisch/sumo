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
/// @file    AdditionalHandler.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Jun 2021
///
// The XML-Handler for additionals loading
/****************************************************************************/
#include <config.h>

#include <utils/common/MsgHandler.h>
#include <utils/xml/SUMOSAXHandler.h>
#include <utils/shapes/Shape.h>
#include <utils/shapes/PointOfInterest.h>
#include <utils/options/OptionsCont.h>
#include <utils/vehicle/SUMOVehicleParserHelper.h>
#include <utils/xml/SUMOXMLDefinitions.h>

#include "AdditionalHandler.h"


// ===========================================================================
// method definitions
// ===========================================================================

AdditionalHandler::AdditionalHandler() {}


AdditionalHandler::~AdditionalHandler() {}


bool
AdditionalHandler::beginParseAttributes(SumoXMLTag tag, const SUMOSAXAttributes& attrs) {
    // open SUMOBaseOBject
    myCommonXMLStructure.openSUMOBaseOBject();
    // check tag
    try {
        switch (tag) {
            // Stopping Places
            case SUMO_TAG_BUS_STOP:
                parseBusStopAttributes(attrs);
                break;
            case SUMO_TAG_TRAIN_STOP:
                parseTrainStopAttributes(attrs);
                break;
            case SUMO_TAG_ACCESS:
                parseAccessAttributes(attrs);
                break;
            case SUMO_TAG_CONTAINER_STOP:
                parseContainerStopAttributes(attrs);
                break;
            case SUMO_TAG_CHARGING_STATION:
                parseChargingStationAttributes(attrs);
                break;
            case SUMO_TAG_PARKING_AREA:
                parseParkingAreaAttributes(attrs);
                break;
            case SUMO_TAG_PARKING_SPACE:
                parseParkingSpaceAttributes(attrs);
                break;
            // Detectors
            case SUMO_TAG_E1DETECTOR:
            case SUMO_TAG_INDUCTION_LOOP:
                parseE1Attributes(attrs);
                break;
            case SUMO_TAG_E2DETECTOR:
            case SUMO_TAG_LANE_AREA_DETECTOR:
                parseE2Attributes(attrs);
                break;
            case SUMO_TAG_E3DETECTOR:
            case SUMO_TAG_ENTRY_EXIT_DETECTOR:
                parseE3Attributes(attrs);
                break;
            case SUMO_TAG_DET_ENTRY:
                parseEntryAttributes(attrs);
                break;
            case SUMO_TAG_DET_EXIT:
                parseExitAttributes(attrs);
                break;
            case SUMO_TAG_INSTANT_INDUCTION_LOOP:
                parseE1InstantAttributes(attrs);
                break;
            // TAZs
            case SUMO_TAG_TAZ:
                parseTAZAttributes(attrs);
                break;
            case SUMO_TAG_TAZSOURCE:
                parseTAZSourceAttributes(attrs);
                break;
            case SUMO_TAG_TAZSINK:
                parseTAZSinkAttributes(attrs);
                break;
            // Variable Speed Sign
            case SUMO_TAG_VSS:
                parseVariableSpeedSignAttributes(attrs);
                break;
            case SUMO_TAG_STEP:
                parseVariableSpeedSignStepAttributes(attrs);
                break;
            // Calibrator
            case SUMO_TAG_CALIBRATOR:
            case GNE_TAG_CALIBRATOR_LANE:
                parseCalibratorAttributes(attrs);
                break;
            // flow (calibrator)
            case SUMO_TAG_FLOW:
                parseCalibratorFlowAttributes(attrs);
                break;
            // Rerouter
            case SUMO_TAG_REROUTER:
                parseRerouterAttributes(attrs);
                break;
            case SUMO_TAG_INTERVAL:
                parseRerouterIntervalAttributes(attrs);
                break;
            case SUMO_TAG_CLOSING_LANE_REROUTE:
                parseClosingLaneRerouteAttributes(attrs);
                break;
            case SUMO_TAG_CLOSING_REROUTE:
                parseClosingRerouteAttributes(attrs);
                break;
            case SUMO_TAG_DEST_PROB_REROUTE:
                parseDestProbRerouteAttributes(attrs);
                break;
            case SUMO_TAG_PARKING_AREA_REROUTE:
                parseParkingAreaRerouteAttributes(attrs);
                break;
            case SUMO_TAG_ROUTE_PROB_REROUTE:
                parseRouteProbRerouteAttributes(attrs);
                break;
            // Route probe
            case SUMO_TAG_ROUTEPROBE:
                parseRouteProbeAttributes(attrs);
                break;
            // Vaporizer (deprecated)
            case SUMO_TAG_VAPORIZER:
                parseVaporizerAttributes(attrs);
                break;
            // wires
            case SUMO_TAG_TRACTION_SUBSTATION:
                parseTractionSubstation(attrs);
                break;
            case SUMO_TAG_OVERHEAD_WIRE_CLAMP:
                parseOverheadWireClamp(attrs);
                break;
            case SUMO_TAG_OVERHEAD_WIRE_SECTION:
                parseOverheadWire(attrs);
                break;
            // Poly
            case SUMO_TAG_POLY:
                parsePolyAttributes(attrs);
                break;
            case SUMO_TAG_POI:
                parsePOIAttributes(attrs);
                break;
            // JuPedSim
            case GNE_TAG_JPS_WALKABLEAREA:
                parseJpsWalkableAreaAttributes(attrs);
                break;
            case GNE_TAG_JPS_OBSTACLE:
                parseJpsObstacleAttributes(attrs);
                break;
            // parameters
            case SUMO_TAG_PARAM:
                parseParameters(attrs);
                break;
            default:
                // tag cannot be parsed in AdditionalHandler
                return false;
                break;
        }
    } catch (InvalidArgument& e) {
        writeError(e.what());
    }
    return true;
}


void
AdditionalHandler::endParseAttributes() {
    // get last inserted object
    CommonXMLStructure::SumoBaseObject* obj = myCommonXMLStructure.getCurrentSumoBaseObject();
    // close SUMOBaseOBject
    myCommonXMLStructure.closeSUMOBaseOBject();
    // check tag
    switch (obj->getTag()) {
        // Stopping Places
        case SUMO_TAG_BUS_STOP:
        case SUMO_TAG_TRAIN_STOP:
        case SUMO_TAG_CONTAINER_STOP:
        case SUMO_TAG_CHARGING_STATION:
        case SUMO_TAG_PARKING_AREA:
        // detectors
        case SUMO_TAG_E1DETECTOR:
        case SUMO_TAG_INDUCTION_LOOP:
        case SUMO_TAG_E2DETECTOR:
        case SUMO_TAG_LANE_AREA_DETECTOR:
        case SUMO_TAG_E3DETECTOR:
        case SUMO_TAG_ENTRY_EXIT_DETECTOR:
        case SUMO_TAG_INSTANT_INDUCTION_LOOP:
        // TAZs
        case SUMO_TAG_TAZ:
        // Variable Speed Sign
        case SUMO_TAG_VSS:
        // Calibrator
        case SUMO_TAG_CALIBRATOR:
        case GNE_TAG_CALIBRATOR_LANE:
        // Rerouter
        case SUMO_TAG_REROUTER:
        // Route probe
        case SUMO_TAG_ROUTEPROBE:
        // Vaporizer (deprecated)
        case SUMO_TAG_VAPORIZER:
        // wires
        case SUMO_TAG_TRACTION_SUBSTATION:
        case SUMO_TAG_OVERHEAD_WIRE_SECTION:
        case SUMO_TAG_OVERHEAD_WIRE_CLAMP:
        // Shapes
        case SUMO_TAG_POLY:
        case SUMO_TAG_POI:
        // JuPedSim
        case GNE_TAG_JPS_WALKABLEAREA:
        case GNE_TAG_JPS_OBSTACLE:
            // parse object and all their childrens
            parseSumoBaseObject(obj);
            // delete object (and all of their childrens)
            delete obj;
            break;
        default:
            break;
    }
}


void
AdditionalHandler::parseSumoBaseObject(CommonXMLStructure::SumoBaseObject* obj) {
    // switch tag
    switch (obj->getTag()) {
        // Stopping Places
        case SUMO_TAG_BUS_STOP:
            buildBusStop(obj,
                         obj->getStringAttribute(SUMO_ATTR_ID),
                         obj->getStringAttribute(SUMO_ATTR_LANE),
                         obj->getDoubleAttribute(SUMO_ATTR_STARTPOS),
                         obj->getDoubleAttribute(SUMO_ATTR_ENDPOS),
                         obj->getStringAttribute(SUMO_ATTR_NAME),
                         obj->getStringListAttribute(SUMO_ATTR_LINES),
                         obj->getIntAttribute(SUMO_ATTR_PERSON_CAPACITY),
                         obj->getDoubleAttribute(SUMO_ATTR_PARKING_LENGTH),
                         obj->getColorAttribute(SUMO_ATTR_COLOR),
                         obj->getBoolAttribute(SUMO_ATTR_FRIENDLY_POS),
                         obj->getParameters());
            break;
        case SUMO_TAG_TRAIN_STOP:
            buildTrainStop(obj,
                           obj->getStringAttribute(SUMO_ATTR_ID),
                           obj->getStringAttribute(SUMO_ATTR_LANE),
                           obj->getDoubleAttribute(SUMO_ATTR_STARTPOS),
                           obj->getDoubleAttribute(SUMO_ATTR_ENDPOS),
                           obj->getStringAttribute(SUMO_ATTR_NAME),
                           obj->getStringListAttribute(SUMO_ATTR_LINES),
                           obj->getIntAttribute(SUMO_ATTR_PERSON_CAPACITY),
                           obj->getDoubleAttribute(SUMO_ATTR_PARKING_LENGTH),
                           obj->getColorAttribute(SUMO_ATTR_COLOR),
                           obj->getBoolAttribute(SUMO_ATTR_FRIENDLY_POS),
                           obj->getParameters());
            break;
        case SUMO_TAG_ACCESS:
            buildAccess(obj,
                        obj->getStringAttribute(SUMO_ATTR_LANE),
                        obj->getStringAttribute(SUMO_ATTR_POSITION),
                        obj->getDoubleAttribute(SUMO_ATTR_LENGTH),
                        obj->getBoolAttribute(SUMO_ATTR_FRIENDLY_POS),
                        obj->getParameters());
            break;
        case SUMO_TAG_CONTAINER_STOP:
            buildContainerStop(obj,
                               obj->getStringAttribute(SUMO_ATTR_ID),
                               obj->getStringAttribute(SUMO_ATTR_LANE),
                               obj->getDoubleAttribute(SUMO_ATTR_STARTPOS),
                               obj->getDoubleAttribute(SUMO_ATTR_ENDPOS),
                               obj->getStringAttribute(SUMO_ATTR_NAME),
                               obj->getStringListAttribute(SUMO_ATTR_LINES),
                               obj->getIntAttribute(SUMO_ATTR_CONTAINER_CAPACITY),
                               obj->getDoubleAttribute(SUMO_ATTR_PARKING_LENGTH),
                               obj->getColorAttribute(SUMO_ATTR_COLOR),
                               obj->getBoolAttribute(SUMO_ATTR_FRIENDLY_POS),
                               obj->getParameters());
            break;
        case SUMO_TAG_CHARGING_STATION:
            buildChargingStation(obj,
                                 obj->getStringAttribute(SUMO_ATTR_ID),
                                 obj->getStringAttribute(SUMO_ATTR_LANE),
                                 obj->getDoubleAttribute(SUMO_ATTR_STARTPOS),
                                 obj->getDoubleAttribute(SUMO_ATTR_ENDPOS),
                                 obj->getStringAttribute(SUMO_ATTR_NAME),
                                 obj->getDoubleAttribute(SUMO_ATTR_CHARGINGPOWER),
                                 obj->getDoubleAttribute(SUMO_ATTR_EFFICIENCY),
                                 obj->getBoolAttribute(SUMO_ATTR_CHARGEINTRANSIT),
                                 obj->getTimeAttribute(SUMO_ATTR_CHARGEDELAY),
                                 obj->getStringAttribute(SUMO_ATTR_CHARGETYPE),
                                 obj->getTimeAttribute(SUMO_ATTR_WAITINGTIME),
                                 obj->getBoolAttribute(SUMO_ATTR_FRIENDLY_POS),
                                 obj->getStringAttribute(SUMO_ATTR_PARKING_AREA),
                                 obj->getParameters());
            break;
        case SUMO_TAG_PARKING_AREA:
            buildParkingArea(obj,
                             obj->getStringAttribute(SUMO_ATTR_ID),
                             obj->getStringAttribute(SUMO_ATTR_LANE),
                             obj->getDoubleAttribute(SUMO_ATTR_STARTPOS),
                             obj->getDoubleAttribute(SUMO_ATTR_ENDPOS),
                             obj->getStringAttribute(SUMO_ATTR_DEPARTPOS),
                             obj->getStringAttribute(SUMO_ATTR_NAME),
                             obj->getStringListAttribute(SUMO_ATTR_ACCEPTED_BADGES),
                             obj->getBoolAttribute(SUMO_ATTR_FRIENDLY_POS),
                             obj->getIntAttribute(SUMO_ATTR_ROADSIDE_CAPACITY),
                             obj->getBoolAttribute(SUMO_ATTR_ONROAD),
                             obj->getDoubleAttribute(SUMO_ATTR_WIDTH),
                             obj->getDoubleAttribute(SUMO_ATTR_LENGTH),
                             obj->getDoubleAttribute(SUMO_ATTR_ANGLE),
                             obj->getBoolAttribute(SUMO_ATTR_LEFTHAND),
                             obj->getParameters());
            break;
        case SUMO_TAG_PARKING_SPACE:
            buildParkingSpace(obj,
                              obj->getDoubleAttribute(SUMO_ATTR_X),
                              obj->getDoubleAttribute(SUMO_ATTR_Y),
                              obj->getDoubleAttribute(SUMO_ATTR_Z),
                              obj->getStringAttribute(SUMO_ATTR_NAME),
                              obj->getStringAttribute(SUMO_ATTR_WIDTH),
                              obj->getStringAttribute(SUMO_ATTR_LENGTH),
                              obj->getStringAttribute(SUMO_ATTR_ANGLE),
                              obj->getDoubleAttribute(SUMO_ATTR_SLOPE),
                              obj->getParameters());
            break;
        // Detectors
        case SUMO_TAG_E1DETECTOR:
        case SUMO_TAG_INDUCTION_LOOP:
            // build E1
            buildE1Detector(obj,
                            obj->getStringAttribute(SUMO_ATTR_ID),
                            obj->getStringAttribute(SUMO_ATTR_LANE),
                            obj->getDoubleAttribute(SUMO_ATTR_POSITION),
                            obj->getPeriodAttribute(),
                            obj->getStringAttribute(SUMO_ATTR_FILE),
                            obj->getStringListAttribute(SUMO_ATTR_VTYPES),
                            obj->getStringListAttribute(SUMO_ATTR_NEXT_EDGES),
                            obj->getStringAttribute(SUMO_ATTR_DETECT_PERSONS),
                            obj->getStringAttribute(SUMO_ATTR_NAME),
                            obj->getBoolAttribute(SUMO_ATTR_FRIENDLY_POS),
                            obj->getParameters());
            break;
        case SUMO_TAG_E2DETECTOR:
        case SUMO_TAG_LANE_AREA_DETECTOR:
            if (obj->hasStringAttribute(SUMO_ATTR_LANE)) {
                buildSingleLaneDetectorE2(obj,
                                          obj->getStringAttribute(SUMO_ATTR_ID),
                                          obj->getStringAttribute(SUMO_ATTR_LANE),
                                          obj->getDoubleAttribute(SUMO_ATTR_POSITION),
                                          obj->getDoubleAttribute(SUMO_ATTR_LENGTH),
                                          obj->getPeriodAttribute(),
                                          obj->getStringAttribute(SUMO_ATTR_TLID),
                                          obj->getStringAttribute(SUMO_ATTR_FILE),
                                          obj->getStringListAttribute(SUMO_ATTR_VTYPES),
                                          obj->getStringListAttribute(SUMO_ATTR_NEXT_EDGES),
                                          obj->getStringAttribute(SUMO_ATTR_DETECT_PERSONS),
                                          obj->getStringAttribute(SUMO_ATTR_NAME),
                                          obj->getTimeAttribute(SUMO_ATTR_HALTING_TIME_THRESHOLD),
                                          obj->getDoubleAttribute(SUMO_ATTR_HALTING_SPEED_THRESHOLD),
                                          obj->getDoubleAttribute(SUMO_ATTR_JAM_DIST_THRESHOLD),
                                          obj->getBoolAttribute(SUMO_ATTR_FRIENDLY_POS),
                                          obj->getBoolAttribute(SUMO_ATTR_SHOW_DETECTOR),
                                          obj->getParameters());
            } else {
                buildMultiLaneDetectorE2(obj,
                                         obj->getStringAttribute(SUMO_ATTR_ID),
                                         obj->getStringListAttribute(SUMO_ATTR_LANES),
                                         obj->getDoubleAttribute(SUMO_ATTR_POSITION),
                                         obj->getDoubleAttribute(SUMO_ATTR_ENDPOS),
                                         obj->getPeriodAttribute(),
                                         obj->getStringAttribute(SUMO_ATTR_TLID),
                                         obj->getStringAttribute(SUMO_ATTR_FILE),
                                         obj->getStringListAttribute(SUMO_ATTR_VTYPES),
                                         obj->getStringListAttribute(SUMO_ATTR_NEXT_EDGES),
                                         obj->getStringAttribute(SUMO_ATTR_DETECT_PERSONS),
                                         obj->getStringAttribute(SUMO_ATTR_NAME),
                                         obj->getTimeAttribute(SUMO_ATTR_HALTING_TIME_THRESHOLD),
                                         obj->getDoubleAttribute(SUMO_ATTR_HALTING_SPEED_THRESHOLD),
                                         obj->getDoubleAttribute(SUMO_ATTR_JAM_DIST_THRESHOLD),
                                         obj->getBoolAttribute(SUMO_ATTR_FRIENDLY_POS),
                                         obj->getBoolAttribute(SUMO_ATTR_SHOW_DETECTOR),
                                         obj->getParameters());
            }
            break;
        case SUMO_TAG_E3DETECTOR:
        case SUMO_TAG_ENTRY_EXIT_DETECTOR:
            buildDetectorE3(obj,
                            obj->getStringAttribute(SUMO_ATTR_ID),
                            obj->getPositionAttribute(SUMO_ATTR_POSITION),
                            obj->getPeriodAttribute(),
                            obj->getStringAttribute(SUMO_ATTR_FILE),
                            obj->getStringListAttribute(SUMO_ATTR_VTYPES),
                            obj->getStringListAttribute(SUMO_ATTR_NEXT_EDGES),
                            obj->getStringAttribute(SUMO_ATTR_DETECT_PERSONS),
                            obj->getStringAttribute(SUMO_ATTR_NAME),
                            obj->getTimeAttribute(SUMO_ATTR_HALTING_TIME_THRESHOLD),
                            obj->getDoubleAttribute(SUMO_ATTR_HALTING_SPEED_THRESHOLD),
                            obj->getBoolAttribute(SUMO_ATTR_OPEN_ENTRY),
                            obj->getBoolAttribute(SUMO_ATTR_EXPECT_ARRIVAL),
                            obj->getParameters());
            break;
        case SUMO_TAG_DET_ENTRY:
            buildDetectorEntry(obj,
                               obj->getStringAttribute(SUMO_ATTR_LANE),
                               obj->getDoubleAttribute(SUMO_ATTR_POSITION),
                               obj->getBoolAttribute(SUMO_ATTR_FRIENDLY_POS),
                               obj->getParameters());
            break;
        case SUMO_TAG_DET_EXIT:
            buildDetectorExit(obj,
                              obj->getStringAttribute(SUMO_ATTR_LANE),
                              obj->getDoubleAttribute(SUMO_ATTR_POSITION),
                              obj->getBoolAttribute(SUMO_ATTR_FRIENDLY_POS),
                              obj->getParameters());
            break;
        case SUMO_TAG_INSTANT_INDUCTION_LOOP:
            buildDetectorE1Instant(obj,
                                   obj->getStringAttribute(SUMO_ATTR_ID),
                                   obj->getStringAttribute(SUMO_ATTR_LANE),
                                   obj->getDoubleAttribute(SUMO_ATTR_POSITION),
                                   obj->getStringAttribute(SUMO_ATTR_FILE),
                                   obj->getStringListAttribute(SUMO_ATTR_VTYPES),
                                   obj->getStringListAttribute(SUMO_ATTR_NEXT_EDGES),
                                   obj->getStringAttribute(SUMO_ATTR_DETECT_PERSONS),
                                   obj->getStringAttribute(SUMO_ATTR_NAME),
                                   obj->getBoolAttribute(SUMO_ATTR_FRIENDLY_POS),
                                   obj->getParameters());
            break;
        // TAZs
        case SUMO_TAG_TAZ:
            buildTAZ(obj,
                     obj->getStringAttribute(SUMO_ATTR_ID),
                     obj->getPositionVectorAttribute(SUMO_ATTR_SHAPE),
                     obj->getPositionAttribute(SUMO_ATTR_CENTER),
                     obj->getBoolAttribute(SUMO_ATTR_FILL),
                     obj->getColorAttribute(SUMO_ATTR_COLOR),
                     obj->getStringListAttribute(SUMO_ATTR_EDGES),
                     obj->getStringAttribute(SUMO_ATTR_NAME),
                     obj->getParameters());
            break;
        case SUMO_TAG_TAZSOURCE:
            buildTAZSource(obj,
                           obj->getStringAttribute(SUMO_ATTR_ID),
                           obj->getDoubleAttribute(SUMO_ATTR_WEIGHT));
            break;
        case SUMO_TAG_TAZSINK:
            buildTAZSink(obj,
                         obj->getStringAttribute(SUMO_ATTR_ID),
                         obj->getDoubleAttribute(SUMO_ATTR_WEIGHT));
            break;
        // Variable Speed Sign
        case SUMO_TAG_VSS:
            buildVariableSpeedSign(obj,
                                   obj->getStringAttribute(SUMO_ATTR_ID),
                                   obj->getPositionAttribute(SUMO_ATTR_POSITION),
                                   obj->getStringListAttribute(SUMO_ATTR_LANES),
                                   obj->getStringAttribute(SUMO_ATTR_NAME),
                                   obj->getStringListAttribute(SUMO_ATTR_VTYPES),
                                   obj->getParameters());
            break;
        case SUMO_TAG_STEP:
            buildVariableSpeedSignStep(obj,
                                       obj->getTimeAttribute(SUMO_ATTR_TIME),
                                       obj->getStringAttribute(SUMO_ATTR_SPEED));
            break;
        // Calibrator
        case SUMO_TAG_CALIBRATOR:
            buildEdgeCalibrator(obj,
                                obj->getStringAttribute(SUMO_ATTR_ID),
                                obj->getStringAttribute(SUMO_ATTR_EDGE),
                                obj->getDoubleAttribute(SUMO_ATTR_POSITION),
                                obj->getStringAttribute(SUMO_ATTR_NAME),
                                obj->getStringAttribute(SUMO_ATTR_OUTPUT),
                                obj->getPeriodAttribute(),
                                obj->getStringAttribute(SUMO_ATTR_ROUTEPROBE),
                                obj->getDoubleAttribute(SUMO_ATTR_JAM_DIST_THRESHOLD),
                                obj->getStringListAttribute(SUMO_ATTR_VTYPES),
                                obj->getParameters());
            break;
        case GNE_TAG_CALIBRATOR_LANE:
            buildLaneCalibrator(obj,
                                obj->getStringAttribute(SUMO_ATTR_ID),
                                obj->getStringAttribute(SUMO_ATTR_LANE),
                                obj->getDoubleAttribute(SUMO_ATTR_POSITION),
                                obj->getStringAttribute(SUMO_ATTR_NAME),
                                obj->getStringAttribute(SUMO_ATTR_OUTPUT),
                                obj->getPeriodAttribute(),
                                obj->getStringAttribute(SUMO_ATTR_ROUTEPROBE),
                                obj->getDoubleAttribute(SUMO_ATTR_JAM_DIST_THRESHOLD),
                                obj->getStringListAttribute(SUMO_ATTR_VTYPES),
                                obj->getParameters());
            break;
        case SUMO_TAG_FLOW:
            buildCalibratorFlow(obj,
                                obj->getVehicleParameter());
            break;
        // Rerouter
        case SUMO_TAG_REROUTER:
            buildRerouter(obj,
                          obj->getStringAttribute(SUMO_ATTR_ID),
                          obj->getPositionAttribute(SUMO_ATTR_POSITION),
                          obj->getStringListAttribute(SUMO_ATTR_EDGES),
                          obj->getDoubleAttribute(SUMO_ATTR_PROB),
                          obj->getStringAttribute(SUMO_ATTR_NAME),
                          obj->getBoolAttribute(SUMO_ATTR_OFF),
                          obj->getBoolAttribute(SUMO_ATTR_OPTIONAL),
                          obj->getTimeAttribute(SUMO_ATTR_HALTING_TIME_THRESHOLD),
                          obj->getStringListAttribute(SUMO_ATTR_VTYPES),
                          obj->getParameters());
            break;
        case SUMO_TAG_CLOSING_LANE_REROUTE:
            buildClosingLaneReroute(obj,
                                    obj->getStringAttribute(SUMO_ATTR_ID),
                                    parseVehicleClasses(obj->getStringAttribute(SUMO_ATTR_ALLOW), obj->getStringAttribute(SUMO_ATTR_DISALLOW)));
            break;
        case SUMO_TAG_CLOSING_REROUTE:
            buildClosingReroute(obj,
                                obj->getStringAttribute(SUMO_ATTR_ID),
                                parseVehicleClasses(obj->getStringAttribute(SUMO_ATTR_ALLOW), obj->getStringAttribute(SUMO_ATTR_DISALLOW)));
            break;
        case SUMO_TAG_DEST_PROB_REROUTE:
            buildDestProbReroute(obj,
                                 obj->getStringAttribute(SUMO_ATTR_ID),
                                 obj->getDoubleAttribute(SUMO_ATTR_PROB));
            break;
        case SUMO_TAG_PARKING_AREA_REROUTE:
            buildParkingAreaReroute(obj,
                                    obj->getStringAttribute(SUMO_ATTR_ID),
                                    obj->getDoubleAttribute(SUMO_ATTR_PROB),
                                    obj->getBoolAttribute(SUMO_ATTR_VISIBLE));
            break;
        case SUMO_TAG_ROUTE_PROB_REROUTE:
            buildRouteProbReroute(obj,
                                  obj->getStringAttribute(SUMO_ATTR_ID),
                                  obj->getDoubleAttribute(SUMO_ATTR_PROB));
            break;
        case SUMO_TAG_INTERVAL:
            // check if is VSS or a REROUTER interval
            if (obj->getParentSumoBaseObject()->getTag() == SUMO_TAG_REROUTER) {
                buildRerouterInterval(obj,
                                      obj->getTimeAttribute(SUMO_ATTR_BEGIN),
                                      obj->getTimeAttribute(SUMO_ATTR_END));
            } else {
                buildVariableSpeedSignStep(obj,
                                           obj->getTimeAttribute(SUMO_ATTR_TIME),
                                           obj->getStringAttribute(SUMO_ATTR_SPEED));
            }
            break;
        // Route probe
        case SUMO_TAG_ROUTEPROBE:
            buildRouteProbe(obj,
                            obj->getStringAttribute(SUMO_ATTR_ID),
                            obj->getStringAttribute(SUMO_ATTR_EDGE),
                            obj->getPeriodAttribute(),
                            obj->getStringAttribute(SUMO_ATTR_NAME),
                            obj->getStringAttribute(SUMO_ATTR_FILE),
                            obj->getTimeAttribute(SUMO_ATTR_BEGIN),
                            obj->getStringListAttribute(SUMO_ATTR_VTYPES),
                            obj->getParameters());
            break;
        // Vaporizer (deprecated)
        case SUMO_TAG_VAPORIZER:
            buildVaporizer(obj,
                           obj->getStringAttribute(SUMO_ATTR_ID),
                           obj->getTimeAttribute(SUMO_ATTR_BEGIN),
                           obj->getTimeAttribute(SUMO_ATTR_END),
                           obj->getStringAttribute(SUMO_ATTR_NAME),
                           obj->getParameters());
            break;
        // wire elements
        case SUMO_TAG_TRACTION_SUBSTATION:
            buildTractionSubstation(obj,
                                    obj->getStringAttribute(SUMO_ATTR_ID),
                                    obj->getPositionAttribute(SUMO_ATTR_POSITION),
                                    obj->getDoubleAttribute(SUMO_ATTR_VOLTAGE),
                                    obj->getDoubleAttribute(SUMO_ATTR_CURRENTLIMIT),
                                    obj->getParameters());
            break;
        case SUMO_TAG_OVERHEAD_WIRE_SECTION:
            buildOverheadWire(obj,
                              obj->getStringAttribute(SUMO_ATTR_ID),
                              obj->getStringAttribute(SUMO_ATTR_SUBSTATIONID),
                              obj->getStringListAttribute(SUMO_ATTR_LANES),
                              obj->getDoubleAttribute(SUMO_ATTR_STARTPOS),
                              obj->getDoubleAttribute(SUMO_ATTR_ENDPOS),
                              obj->getBoolAttribute(SUMO_ATTR_FRIENDLY_POS),
                              obj->getStringListAttribute(SUMO_ATTR_OVERHEAD_WIRE_FORBIDDEN),
                              obj->getParameters());
            break;
        case SUMO_TAG_OVERHEAD_WIRE_CLAMP:
            buildOverheadWireClamp(obj,
                                   obj->getStringAttribute(SUMO_ATTR_ID),
                                   obj->getStringAttribute(SUMO_ATTR_OVERHEAD_WIRECLAMP_START),
                                   obj->getStringAttribute(SUMO_ATTR_OVERHEAD_WIRECLAMP_LANESTART),
                                   obj->getStringAttribute(SUMO_ATTR_OVERHEAD_WIRECLAMP_END),
                                   obj->getStringAttribute(SUMO_ATTR_OVERHEAD_WIRECLAMP_LANEEND),
                                   obj->getParameters());
            break;
        // Polygon
        case SUMO_TAG_POLY:
            buildPolygon(obj,
                         obj->getStringAttribute(SUMO_ATTR_ID),
                         obj->getStringAttribute(SUMO_ATTR_TYPE),
                         obj->getColorAttribute(SUMO_ATTR_COLOR),
                         obj->getDoubleAttribute(SUMO_ATTR_LAYER),
                         obj->getDoubleAttribute(SUMO_ATTR_ANGLE),
                         obj->getStringAttribute(SUMO_ATTR_IMGFILE),
                         obj->getBoolAttribute(SUMO_ATTR_RELATIVEPATH),
                         obj->getPositionVectorAttribute(SUMO_ATTR_SHAPE),
                         obj->getBoolAttribute(SUMO_ATTR_GEO),
                         obj->getBoolAttribute(SUMO_ATTR_FILL),
                         obj->getDoubleAttribute(SUMO_ATTR_LINEWIDTH),
                         obj->getStringAttribute(SUMO_ATTR_NAME),
                         obj->getParameters());
            break;
        // POI
        case SUMO_TAG_POI:
            // check if we want to create a POI, POILane or POIGEO
            if (obj->hasDoubleAttribute(SUMO_ATTR_X)) {
                // build POI over view
                buildPOI(obj,
                         obj->getStringAttribute(SUMO_ATTR_ID),
                         obj->getStringAttribute(SUMO_ATTR_TYPE),
                         obj->getColorAttribute(SUMO_ATTR_COLOR),
                         obj->getDoubleAttribute(SUMO_ATTR_X),
                         obj->getDoubleAttribute(SUMO_ATTR_Y),
                         obj->getStringAttribute(SUMO_ATTR_ICON),
                         obj->getDoubleAttribute(SUMO_ATTR_LAYER),
                         obj->getDoubleAttribute(SUMO_ATTR_ANGLE),
                         obj->getStringAttribute(SUMO_ATTR_IMGFILE),
                         obj->getBoolAttribute(SUMO_ATTR_RELATIVEPATH),
                         obj->getDoubleAttribute(SUMO_ATTR_WIDTH),
                         obj->getDoubleAttribute(SUMO_ATTR_HEIGHT),
                         obj->getStringAttribute(SUMO_ATTR_NAME),
                         obj->getParameters());
            } else if (obj->hasStringAttribute(SUMO_ATTR_LANE)) {
                // build POI over Lane
                buildPOILane(obj,
                             obj->getStringAttribute(SUMO_ATTR_ID),
                             obj->getStringAttribute(SUMO_ATTR_TYPE),
                             obj->getColorAttribute(SUMO_ATTR_COLOR),
                             obj->getStringAttribute(SUMO_ATTR_LANE),
                             obj->getDoubleAttribute(SUMO_ATTR_POSITION),
                             obj->getBoolAttribute(SUMO_ATTR_FRIENDLY_POS),
                             obj->getDoubleAttribute(SUMO_ATTR_POSITION_LAT),
                             obj->getStringAttribute(SUMO_ATTR_ICON),
                             obj->getDoubleAttribute(SUMO_ATTR_LAYER),
                             obj->getDoubleAttribute(SUMO_ATTR_ANGLE),
                             obj->getStringAttribute(SUMO_ATTR_IMGFILE),
                             obj->getBoolAttribute(SUMO_ATTR_RELATIVEPATH),
                             obj->getDoubleAttribute(SUMO_ATTR_WIDTH),
                             obj->getDoubleAttribute(SUMO_ATTR_HEIGHT),
                             obj->getStringAttribute(SUMO_ATTR_NAME),
                             obj->getParameters());
            } else {
                // build POIGEO over view
                buildPOIGeo(obj,
                            obj->getStringAttribute(SUMO_ATTR_ID),
                            obj->getStringAttribute(SUMO_ATTR_TYPE),
                            obj->getColorAttribute(SUMO_ATTR_COLOR),
                            obj->getDoubleAttribute(SUMO_ATTR_LON),
                            obj->getDoubleAttribute(SUMO_ATTR_LAT),
                            obj->getStringAttribute(SUMO_ATTR_ICON),
                            obj->getDoubleAttribute(SUMO_ATTR_LAYER),
                            obj->getDoubleAttribute(SUMO_ATTR_ANGLE),
                            obj->getStringAttribute(SUMO_ATTR_IMGFILE),
                            obj->getBoolAttribute(SUMO_ATTR_RELATIVEPATH),
                            obj->getDoubleAttribute(SUMO_ATTR_WIDTH),
                            obj->getDoubleAttribute(SUMO_ATTR_HEIGHT),
                            obj->getStringAttribute(SUMO_ATTR_NAME),
                            obj->getParameters());
            }
            break;
        // Jps WalkableArea
        case GNE_TAG_JPS_WALKABLEAREA:
            buildJpsWalkableArea(obj,
                                 obj->getStringAttribute(SUMO_ATTR_ID),
                                 obj->getPositionVectorAttribute(SUMO_ATTR_SHAPE),
                                 obj->getBoolAttribute(SUMO_ATTR_GEO),
                                 obj->getStringAttribute(SUMO_ATTR_NAME),
                                 obj->getParameters());
            break;
        // Jps Obstacle
        case GNE_TAG_JPS_OBSTACLE:
            buildJpsObstacle(obj,
                             obj->getStringAttribute(SUMO_ATTR_ID),
                             obj->getPositionVectorAttribute(SUMO_ATTR_SHAPE),
                             obj->getBoolAttribute(SUMO_ATTR_GEO),
                             obj->getStringAttribute(SUMO_ATTR_NAME),
                             obj->getParameters());
            break;
        default:
            break;
    }
    // now iterate over childrens
    for (const auto& child : obj->getSumoBaseObjectChildren()) {
        // call this function recursively
        parseSumoBaseObject(child);
    }
}


bool
AdditionalHandler::isErrorCreatingElement() const {
    return myErrorCreatingElement;
}


void
AdditionalHandler::writeError(const std::string& error) {
    WRITE_ERROR(error);
    myErrorCreatingElement = true;
}


void
AdditionalHandler::parseBusStopAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const std::string laneId = attrs.get<std::string>(SUMO_ATTR_LANE, id.c_str(), parsedOk);
    // optional attributes
    const double startPos = attrs.getOpt<double>(SUMO_ATTR_STARTPOS, id.c_str(), parsedOk, INVALID_DOUBLE);
    const double endPos = attrs.getOpt<double>(SUMO_ATTR_ENDPOS, id.c_str(), parsedOk, INVALID_DOUBLE);
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), parsedOk, "");
    const std::vector<std::string> lines = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_LINES, id.c_str(), parsedOk, std::vector<std::string>());
    const int personCapacity = attrs.getOpt<int>(SUMO_ATTR_PERSON_CAPACITY, id.c_str(), parsedOk, 6);
    const double parkingLength = attrs.getOpt<double>(SUMO_ATTR_PARKING_LENGTH, id.c_str(), parsedOk, 0);
    const RGBColor color = attrs.getOpt<RGBColor>(SUMO_ATTR_COLOR, id.c_str(), parsedOk, RGBColor::INVISIBLE);
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), parsedOk, false);
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_BUS_STOP);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_LANE, laneId);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_STARTPOS, startPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_ENDPOS, endPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_LINES, lines);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addIntAttribute(SUMO_ATTR_PERSON_CAPACITY, personCapacity);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_PARKING_LENGTH, parkingLength);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addColorAttribute(SUMO_ATTR_COLOR, color);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_FRIENDLY_POS, friendlyPos);
    }
}


void
AdditionalHandler::parseTrainStopAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const std::string laneId = attrs.get<std::string>(SUMO_ATTR_LANE, id.c_str(), parsedOk);
    // optional attributes
    const double startPos = attrs.getOpt<double>(SUMO_ATTR_STARTPOS, id.c_str(), parsedOk, INVALID_DOUBLE);
    const double endPos = attrs.getOpt<double>(SUMO_ATTR_ENDPOS, id.c_str(), parsedOk, INVALID_DOUBLE);
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), parsedOk, "");
    const std::vector<std::string> lines = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_LINES, id.c_str(), parsedOk, std::vector<std::string>());
    const int personCapacity = attrs.getOpt<int>(SUMO_ATTR_PERSON_CAPACITY, id.c_str(), parsedOk, 6);
    const double parkingLength = attrs.getOpt<double>(SUMO_ATTR_PARKING_LENGTH, id.c_str(), parsedOk, 0);
    const RGBColor color = attrs.getOpt<RGBColor>(SUMO_ATTR_COLOR, id.c_str(), parsedOk, RGBColor::INVISIBLE);
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), parsedOk, false);
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_TRAIN_STOP);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_LANE, laneId);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_STARTPOS, startPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_ENDPOS, endPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_LINES, lines);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addIntAttribute(SUMO_ATTR_PERSON_CAPACITY, personCapacity);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_PARKING_LENGTH, parkingLength);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addColorAttribute(SUMO_ATTR_COLOR, color);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_FRIENDLY_POS, friendlyPos);
    }
}


void
AdditionalHandler::parseAccessAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string laneId = attrs.get<std::string>(SUMO_ATTR_LANE, "", parsedOk);
    const std::string position = attrs.get<std::string>(SUMO_ATTR_POSITION, "", parsedOk);
    // optional attributes
    const double length = attrs.getOpt<double>(SUMO_ATTR_LENGTH, "", parsedOk, -1.00); /* in future updates, INVALID_DOUBLE */
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, "", parsedOk, false);
    // check parent
    checkParent(SUMO_TAG_ACCESS, {SUMO_TAG_BUS_STOP, SUMO_TAG_TRAIN_STOP}, parsedOk);
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_ACCESS);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_LANE, laneId);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_POSITION, position);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_LENGTH, length);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_FRIENDLY_POS, friendlyPos);
    }
}


void
AdditionalHandler::parseContainerStopAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const std::string laneId = attrs.get<std::string>(SUMO_ATTR_LANE, id.c_str(), parsedOk);
    // optional attributes
    const double startPos = attrs.getOpt<double>(SUMO_ATTR_STARTPOS, id.c_str(), parsedOk, INVALID_DOUBLE);
    const double endPos = attrs.getOpt<double>(SUMO_ATTR_ENDPOS, id.c_str(), parsedOk, INVALID_DOUBLE);
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), parsedOk, "");
    const std::vector<std::string> lines = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_LINES, id.c_str(), parsedOk, std::vector<std::string>());
    const int containerCapacity = attrs.getOpt<int>(SUMO_ATTR_CONTAINER_CAPACITY, id.c_str(), parsedOk, 6);
    const double parkingLength = attrs.getOpt<double>(SUMO_ATTR_PARKING_LENGTH, id.c_str(), parsedOk, 0);
    const RGBColor color = attrs.getOpt<RGBColor>(SUMO_ATTR_COLOR, id.c_str(), parsedOk, RGBColor::INVISIBLE);
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), parsedOk, false);
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_CONTAINER_STOP);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_LANE, laneId);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_STARTPOS, startPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_ENDPOS, endPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_LINES, lines);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addIntAttribute(SUMO_ATTR_CONTAINER_CAPACITY, containerCapacity);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_PARKING_LENGTH, parkingLength);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addColorAttribute(SUMO_ATTR_COLOR, color);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_FRIENDLY_POS, friendlyPos);
    }
}


void
AdditionalHandler::parseChargingStationAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const std::string laneId = attrs.get<std::string>(SUMO_ATTR_LANE, id.c_str(), parsedOk);
    // optional attributes
    const double startPos = attrs.getOpt<double>(SUMO_ATTR_STARTPOS, id.c_str(), parsedOk, INVALID_DOUBLE);
    const double endPos = attrs.getOpt<double>(SUMO_ATTR_ENDPOS, id.c_str(), parsedOk, INVALID_DOUBLE);
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), parsedOk, "");
    const std::vector<std::string> lines = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_LINES, id.c_str(), parsedOk, std::vector<std::string>());
    const double chargingPower = attrs.getOpt<double>(SUMO_ATTR_CHARGINGPOWER, id.c_str(), parsedOk, 22000);
    const double efficiency = attrs.getOpt<double>(SUMO_ATTR_EFFICIENCY, id.c_str(), parsedOk, 0.95);
    const bool chargeInTransit = attrs.getOpt<bool>(SUMO_ATTR_CHARGEINTRANSIT, id.c_str(), parsedOk, 0);
    const SUMOTime chargeDelay = attrs.getOptSUMOTimeReporting(SUMO_ATTR_CHARGEDELAY, id.c_str(), parsedOk, 0);
    const std::string chargeType = attrs.getOpt<std::string>(SUMO_ATTR_CHARGETYPE, id.c_str(), parsedOk, "normal");
    const SUMOTime waitingTime = attrs.getOptSUMOTimeReporting(SUMO_ATTR_WAITINGTIME, id.c_str(), parsedOk, TIME2STEPS(900));
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), parsedOk, false);
    const std::string parkingAreaID = attrs.getOpt<std::string>(SUMO_ATTR_PARKING_AREA, id.c_str(), parsedOk, "");

    // check charge type
    if ((chargeType != "normal") && (chargeType != "electric") && (chargeType != "fuel")) {
        writeError(TLF("Invalid charge type '%' defined in chargingStation '%'.", chargeType, id));
        parsedOk = false;
    }
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_CHARGING_STATION);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_LANE, laneId);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_STARTPOS, startPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_ENDPOS, endPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_LINES, lines);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_CHARGINGPOWER, chargingPower);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_EFFICIENCY, efficiency);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_CHARGEINTRANSIT, chargeInTransit);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_CHARGEDELAY, chargeDelay);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_CHARGETYPE, chargeType);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_WAITINGTIME, waitingTime);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_FRIENDLY_POS, friendlyPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_PARKING_AREA, parkingAreaID);
    }
}


void
AdditionalHandler::parseParkingAreaAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const std::string laneId = attrs.get<std::string>(SUMO_ATTR_LANE, id.c_str(), parsedOk);
    // optional attributes
    const double startPos = attrs.getOpt<double>(SUMO_ATTR_STARTPOS, id.c_str(), parsedOk, INVALID_DOUBLE);
    const double endPos = attrs.getOpt<double>(SUMO_ATTR_ENDPOS, id.c_str(), parsedOk, INVALID_DOUBLE);
    const std::string departPos = attrs.getOpt<std::string>(SUMO_ATTR_DEPARTPOS, id.c_str(), parsedOk, "");
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), parsedOk, "");
    const std::vector<std::string> badges = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_ACCEPTED_BADGES, id.c_str(), parsedOk, std::vector<std::string>());
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), parsedOk, false);
    const int roadSideCapacity = attrs.getOpt<int>(SUMO_ATTR_ROADSIDE_CAPACITY, id.c_str(), parsedOk, 0);
    const bool onRoad = attrs.getOpt<bool>(SUMO_ATTR_ONROAD, id.c_str(), parsedOk, false);
    const double width = attrs.getOpt<double>(SUMO_ATTR_WIDTH, id.c_str(), parsedOk, 0);
    const double length = attrs.getOpt<double>(SUMO_ATTR_LENGTH, id.c_str(), parsedOk, 0);
    const double angle = attrs.getOpt<double>(SUMO_ATTR_ANGLE, id.c_str(), parsedOk, 0);
    const bool lefthand = attrs.getOpt<bool>(SUMO_ATTR_LEFTHAND, id.c_str(), parsedOk, false);

    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_PARKING_AREA);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_LANE, laneId);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_STARTPOS, startPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_ENDPOS, endPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_DEPARTPOS, departPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_ACCEPTED_BADGES, badges);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_FRIENDLY_POS, friendlyPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addIntAttribute(SUMO_ATTR_ROADSIDE_CAPACITY, roadSideCapacity);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_ONROAD, onRoad);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_WIDTH, width);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_LENGTH, length);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_ANGLE, angle);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_LEFTHAND, lefthand);
    }
}


void
AdditionalHandler::parseParkingSpaceAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const double x = attrs.get<double>(SUMO_ATTR_X, "", parsedOk);
    const double y = attrs.get<double>(SUMO_ATTR_Y, "", parsedOk);
    // optional attributes
    const double z = attrs.getOpt<double>(SUMO_ATTR_Z, "", parsedOk, 0);
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, "", parsedOk, "");
    const std::string width = attrs.getOpt<std::string>(SUMO_ATTR_WIDTH, "", parsedOk, "");
    const std::string length = attrs.getOpt<std::string>(SUMO_ATTR_LENGTH, "", parsedOk, "");
    const std::string angle = attrs.getOpt<std::string>(SUMO_ATTR_ANGLE, "", parsedOk, "");
    const double slope = attrs.getOpt<double>(SUMO_ATTR_SLOPE, "", parsedOk, 0);
    // check parent
    checkParent(SUMO_TAG_PARKING_SPACE, {SUMO_TAG_PARKING_AREA}, parsedOk);
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_PARKING_SPACE);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_X, x);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_Y, y);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_Z, z);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_WIDTH, width);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_LENGTH, length);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ANGLE, angle);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_SLOPE, slope);
    }
}


void
AdditionalHandler::parseE1Attributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const std::string laneId = attrs.get<std::string>(SUMO_ATTR_LANE, id.c_str(), parsedOk);
    const double position = attrs.get<double>(SUMO_ATTR_POSITION, id.c_str(), parsedOk);
    const SUMOTime period = attrs.getOptPeriod(id.c_str(), parsedOk, SUMOTime_MAX_PERIOD);
    const std::string file = attrs.get<std::string>(SUMO_ATTR_FILE, id.c_str(), parsedOk);
    // optional attributes
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), parsedOk, "");
    const std::vector<std::string> vehicleTypes = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_VTYPES, id.c_str(), parsedOk, std::vector<std::string>());
    const std::vector<std::string> nextEdges = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_NEXT_EDGES, id.c_str(), parsedOk, std::vector<std::string>());
    const std::string detectPersons = attrs.getOpt<std::string>(SUMO_ATTR_DETECT_PERSONS, "", parsedOk);
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), parsedOk, false);
    // continue if flag is ok
    if (parsedOk && checkDetectPersons(SUMO_TAG_E1DETECTOR, id, detectPersons)) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_E1DETECTOR);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_LANE, laneId);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_POSITION, position);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_PERIOD, period);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_FILE, file);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_VTYPES, vehicleTypes);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_NEXT_EDGES, nextEdges);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_DETECT_PERSONS, detectPersons);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_FRIENDLY_POS, friendlyPos);
    }
}


void
AdditionalHandler::parseE2Attributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    const int positionDef = attrs.hasAttribute(SUMO_ATTR_POSITION) ? 1 : 0;
    const int endPosDef = attrs.hasAttribute(SUMO_ATTR_ENDPOS) ? 1 : 0;
    const int lengthDef = attrs.hasAttribute(SUMO_ATTR_LENGTH) ? 1 : 0;
    // check attributes
    if (attrs.hasAttribute(SUMO_ATTR_LANE) && ((positionDef + endPosDef + lengthDef) > 2)) {
        writeError(TL("'pos', 'endPos' and 'length' cannot be defined together in a single lane area detector."));
    } else if (attrs.hasAttribute(SUMO_ATTR_LANE) && ((positionDef + endPosDef + lengthDef) < 2)) {
        writeError(TL("A single lane area detector requires two parameters of those 'pos', 'endPos' and 'length'."));
    } else {
        // needed attributes
        const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
        const std::string file = attrs.get<std::string>(SUMO_ATTR_FILE, id.c_str(), parsedOk);
        // special attributes
        const std::string laneId = attrs.getOpt<std::string>(SUMO_ATTR_LANE, id.c_str(), parsedOk, "");
        const std::vector<std::string> laneIds = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_LANES, id.c_str(), parsedOk, std::vector<std::string>());
        const double position = attrs.getOpt<double>(SUMO_ATTR_POSITION, id.c_str(), parsedOk, 0);
        const double endPos = attrs.getOpt<double>(SUMO_ATTR_ENDPOS, id.c_str(), parsedOk, 0);
        const double length = attrs.getOpt<double>(SUMO_ATTR_LENGTH, id.c_str(), parsedOk, 0);
        // optional attributes
        const SUMOTime period = attrs.getOptPeriod(id.c_str(), parsedOk, SUMOTime_MAX_PERIOD);
        const std::string trafficLight = attrs.getOpt<std::string>(SUMO_ATTR_TLID, id.c_str(), parsedOk, "");
        const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), parsedOk, "");
        const SUMOTime haltingTimeThreshold = attrs.getOptSUMOTimeReporting(SUMO_ATTR_HALTING_TIME_THRESHOLD, id.c_str(), parsedOk, TIME2STEPS(1));
        const double haltingSpeedThreshold = attrs.getOpt<double>(SUMO_ATTR_HALTING_SPEED_THRESHOLD, id.c_str(), parsedOk, 1.39);
        const double jamDistThreshold = attrs.getOpt<double>(SUMO_ATTR_JAM_DIST_THRESHOLD, id.c_str(), parsedOk, 10);
        const std::vector<std::string> vehicleTypes = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_VTYPES, id.c_str(), parsedOk, std::vector<std::string>());
        const std::vector<std::string> nextEdges = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_NEXT_EDGES, id.c_str(), parsedOk, std::vector<std::string>());
        const std::string detectPersons = attrs.getOpt<std::string>(SUMO_ATTR_DETECT_PERSONS, "", parsedOk);
        const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), parsedOk, false);
        const bool show = attrs.getOpt<bool>(SUMO_ATTR_SHOW_DETECTOR, id.c_str(), parsedOk, true);
        // continue if flag is ok
        if (parsedOk && checkDetectPersons(SUMO_TAG_E2DETECTOR, id, detectPersons)) {
            // set tag
            myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_E2DETECTOR);
            // add attributes depending of Lane/Lanes
            if (attrs.hasAttribute(SUMO_ATTR_LANE)) {
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_LANE, laneId);
                if (positionDef == 0) {
                    myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_POSITION, endPos - length);
                    myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_LENGTH, length);
                } else if (endPosDef == 0) {
                    myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_POSITION, position);
                    myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_LENGTH, length);
                } else if (lengthDef == 0) {
                    myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_POSITION, position);
                    myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_LENGTH, endPos - position);
                }
            } else {
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_LANES, laneIds);
                myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_POSITION, position);
                myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_ENDPOS, endPos);
            }
            // add all attributes
            myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_PERIOD, period);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_TLID, trafficLight);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_FILE, file);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_VTYPES, vehicleTypes);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_NEXT_EDGES, nextEdges);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_DETECT_PERSONS, detectPersons);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_HALTING_TIME_THRESHOLD, haltingTimeThreshold);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_HALTING_SPEED_THRESHOLD, haltingSpeedThreshold);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_JAM_DIST_THRESHOLD, jamDistThreshold);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_FRIENDLY_POS, friendlyPos);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_SHOW_DETECTOR, show);
        }
    }
}


void
AdditionalHandler::parseE3Attributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const std::string file = attrs.get<std::string>(SUMO_ATTR_FILE, id.c_str(), parsedOk);
    const SUMOTime period = attrs.getOptPeriod(id.c_str(), parsedOk, SUMOTime_MAX_PERIOD);
    // optional attributes
    const Position pos = attrs.getOpt<Position>(SUMO_ATTR_POSITION, id.c_str(), parsedOk, Position());
    const std::vector<std::string> vehicleTypes = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_VTYPES, id.c_str(), parsedOk, std::vector<std::string>());
    const std::vector<std::string> nextEdges = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_NEXT_EDGES, id.c_str(), parsedOk, std::vector<std::string>());
    const std::string detectPersons = attrs.getOpt<std::string>(SUMO_ATTR_DETECT_PERSONS, "", parsedOk);
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), parsedOk, "");
    const SUMOTime haltingTimeThreshold = attrs.getOptSUMOTimeReporting(SUMO_ATTR_HALTING_TIME_THRESHOLD, id.c_str(), parsedOk, TIME2STEPS(1));
    const double haltingSpeedThreshold = attrs.getOpt<double>(SUMO_ATTR_HALTING_SPEED_THRESHOLD, id.c_str(), parsedOk, 1.39);
    const bool openEntry = attrs.getOpt<bool>(SUMO_ATTR_OPEN_ENTRY, id.c_str(), parsedOk, false);
    const bool expectedArrival = attrs.getOpt<bool>(SUMO_ATTR_EXPECT_ARRIVAL, id.c_str(), parsedOk, false);
    // continue if flag is ok
    if (parsedOk && checkDetectPersons(SUMO_TAG_E3DETECTOR, id, detectPersons)) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_E3DETECTOR);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_FILE, file);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_PERIOD, period);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addPositionAttribute(SUMO_ATTR_POSITION, pos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_VTYPES, vehicleTypes);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_NEXT_EDGES, nextEdges);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_DETECT_PERSONS, detectPersons);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_HALTING_TIME_THRESHOLD, haltingTimeThreshold);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_HALTING_SPEED_THRESHOLD, haltingSpeedThreshold);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_OPEN_ENTRY, openEntry);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_EXPECT_ARRIVAL, expectedArrival);
    }
}


void
AdditionalHandler::parseEntryAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string laneId = attrs.get<std::string>(SUMO_ATTR_LANE, "", parsedOk);
    const double position = attrs.get<double>(SUMO_ATTR_POSITION, "", parsedOk);
    // optional attributes
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, "", parsedOk, false);
    // check parent
    checkParent(SUMO_TAG_DET_ENTRY, {SUMO_TAG_E3DETECTOR}, parsedOk);
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_DET_ENTRY);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_LANE, laneId);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_POSITION, position);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_FRIENDLY_POS, friendlyPos);
    }
}


void
AdditionalHandler::parseExitAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string laneId = attrs.get<std::string>(SUMO_ATTR_LANE, "", parsedOk);
    const double position = attrs.get<double>(SUMO_ATTR_POSITION, "", parsedOk);
    // optional attributes
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, "", parsedOk, false);
    // check parent
    checkParent(SUMO_TAG_DET_EXIT, {SUMO_TAG_E3DETECTOR}, parsedOk);
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_DET_EXIT);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_LANE, laneId);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_POSITION, position);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_FRIENDLY_POS, friendlyPos);
    }
}


void
AdditionalHandler::parseE1InstantAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const std::string laneId = attrs.get<std::string>(SUMO_ATTR_LANE, id.c_str(), parsedOk);
    const double position = attrs.get<double>(SUMO_ATTR_POSITION, id.c_str(), parsedOk);
    const std::string file = attrs.get<std::string>(SUMO_ATTR_FILE, id.c_str(), parsedOk);
    // optional attributes
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), parsedOk, "");
    const std::vector<std::string> vehicleTypes = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_VTYPES, id.c_str(), parsedOk, std::vector<std::string>());
    const std::vector<std::string> nextEdges = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_NEXT_EDGES, id.c_str(), parsedOk, std::vector<std::string>());
    const std::string detectPersons = attrs.getOpt<std::string>(SUMO_ATTR_DETECT_PERSONS, "", parsedOk);
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), parsedOk, false);
    // continue if flag is ok
    if (parsedOk && checkDetectPersons(SUMO_TAG_INSTANT_INDUCTION_LOOP, id, detectPersons)) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_INSTANT_INDUCTION_LOOP);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_LANE, laneId);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_POSITION, position);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_FILE, file);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_VTYPES, vehicleTypes);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_NEXT_EDGES, nextEdges);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_DETECT_PERSONS, detectPersons);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_FRIENDLY_POS, friendlyPos);
    }
}


void
AdditionalHandler::parseTAZAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    // optional attributes
    const PositionVector shape = attrs.getOpt<PositionVector>(SUMO_ATTR_SHAPE, id.c_str(), parsedOk, PositionVector());
    const Position center = attrs.getOpt<Position>(SUMO_ATTR_CENTER, id.c_str(), parsedOk, shape.size() > 0 ? shape.getCentroid() : Position::INVALID);
    const bool fill = attrs.getOpt<bool>(SUMO_ATTR_FILL, id.c_str(), parsedOk, false);
    const std::vector<std::string> edges = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_EDGES, id.c_str(), parsedOk, std::vector<std::string>());
    const RGBColor color = attrs.getOpt<RGBColor>(SUMO_ATTR_COLOR, id.c_str(), parsedOk, RGBColor::RED);
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), parsedOk, "");
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_TAZ);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addPositionVectorAttribute(SUMO_ATTR_SHAPE, shape);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addPositionAttribute(SUMO_ATTR_CENTER, center);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_FILL, fill);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_EDGES, edges);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addColorAttribute(SUMO_ATTR_COLOR, color);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
    }
}


void
AdditionalHandler::parseTAZSourceAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string edgeID = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const double weight = attrs.get<double>(SUMO_ATTR_WEIGHT, edgeID.c_str(), parsedOk);
    // check parent
    checkParent(SUMO_TAG_TAZSOURCE, {SUMO_TAG_TAZ}, parsedOk);
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_TAZSOURCE);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, edgeID);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_WEIGHT, weight);
    }
}


void
AdditionalHandler::parseTAZSinkAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string edgeID = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const double weight = attrs.get<double>(SUMO_ATTR_WEIGHT, edgeID.c_str(), parsedOk);
    // check parent
    checkParent(SUMO_TAG_TAZSINK, {SUMO_TAG_TAZ}, parsedOk);
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_TAZSINK);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, edgeID);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_WEIGHT, weight);
    }
}


void
AdditionalHandler::parseVariableSpeedSignAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const std::vector<std::string> lanes = attrs.get<std::vector<std::string> >(SUMO_ATTR_LANES, id.c_str(), parsedOk);
    // optional attributes
    const Position pos = attrs.getOpt<Position>(SUMO_ATTR_POSITION, id.c_str(), parsedOk, Position());
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), parsedOk, "");
    const std::vector<std::string> vehicleTypes = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_VTYPES, id.c_str(), parsedOk, std::vector<std::string>());
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_VSS);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_LANES, lanes);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addPositionAttribute(SUMO_ATTR_POSITION, pos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_VTYPES, vehicleTypes);
    }
}


void
AdditionalHandler::parseVariableSpeedSignStepAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const SUMOTime time = attrs.getSUMOTimeReporting(SUMO_ATTR_TIME, "", parsedOk);
    // optional attributes
    const std::string speed = attrs.getOpt<std::string>(SUMO_ATTR_SPEED, "", parsedOk, "");
    // check parent
    checkParent(SUMO_TAG_STEP, {SUMO_TAG_VSS}, parsedOk);
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_STEP);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_TIME, time);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_SPEED, speed);
    }
}


void
AdditionalHandler::parseCalibratorAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // check that frecuency and trafficLight aren't defined together
    if ((attrs.hasAttribute(SUMO_ATTR_EDGE) && attrs.hasAttribute(SUMO_ATTR_LANE)) ||
            (!attrs.hasAttribute(SUMO_ATTR_EDGE) && !attrs.hasAttribute(SUMO_ATTR_LANE))) {
        writeError(TL("Calibrators need either an edge or a lane"));
        parsedOk = false;
    }
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const double pos = attrs.get<double>(SUMO_ATTR_POSITION, id.c_str(), parsedOk);
    // special attributes
    const std::string edge = attrs.getOpt<std::string>(SUMO_ATTR_EDGE, id.c_str(), parsedOk, "");
    const std::string lane = attrs.getOpt<std::string>(SUMO_ATTR_LANE, id.c_str(), parsedOk, "");
    // optional attributes
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), parsedOk, "");
    const SUMOTime period = attrs.getOptPeriod(id.c_str(), parsedOk, DELTA_T);
    const std::string routeProbe = attrs.getOpt<std::string>(SUMO_ATTR_ROUTEPROBE, id.c_str(), parsedOk, "");
    const double jamThreshold = attrs.getOpt<double>(SUMO_ATTR_JAM_DIST_THRESHOLD, id.c_str(), parsedOk, 0.5);
    const std::string output = attrs.getOpt<std::string>(SUMO_ATTR_OUTPUT, id.c_str(), parsedOk, "");
    const std::vector<std::string> vehicleTypes = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_VTYPES, id.c_str(), parsedOk, std::vector<std::string>());
    // continue if flag is ok
    if (parsedOk) {
        // set tag depending of edge/lane
        if (attrs.hasAttribute(SUMO_ATTR_EDGE)) {
            myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_CALIBRATOR);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_EDGE, edge);
        } else {
            myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(GNE_TAG_CALIBRATOR_LANE);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_LANE, lane);
        }
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_POSITION, pos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_PERIOD, period);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ROUTEPROBE, routeProbe);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_JAM_DIST_THRESHOLD, jamThreshold);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_OUTPUT, output);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_VTYPES, vehicleTypes);
    }
}


void
AdditionalHandler::parseCalibratorFlowAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // check parent
    if (myCommonXMLStructure.getCurrentSumoBaseObject()->getParentSumoBaseObject() &&
            myCommonXMLStructure.getCurrentSumoBaseObject()->getParentSumoBaseObject()->getTag() != SUMO_TAG_ROOTFILE) {
        // check that frecuency and trafficLight aren't defined together
        if (!attrs.hasAttribute(SUMO_ATTR_TYPE) && !attrs.hasAttribute(SUMO_ATTR_VEHSPERHOUR) && !attrs.hasAttribute(SUMO_ATTR_SPEED)) {
            writeError(TL("CalibratorFlows need either the attribute vehsPerHour or speed or type (or any combination of these)"));
        }
        // first parse flow
        SUMOVehicleParameter* flowParameter = SUMOVehicleParserHelper::parseVehicleAttributes(SUMO_TAG_FLOW, attrs, false, true, true);
        if (flowParameter) {
            // set VPH and speed
            if (attrs.hasAttribute(SUMO_ATTR_VEHSPERHOUR)) {
                flowParameter->repetitionOffset = TIME2STEPS(3600. / attrs.get<double>(SUMO_ATTR_VEHSPERHOUR, "", parsedOk));
                flowParameter->parametersSet |= VEHPARS_VPH_SET;
            }
            if (attrs.hasAttribute(SUMO_ATTR_SPEED)) {
                flowParameter->calibratorSpeed = attrs.get<double>(SUMO_ATTR_SPEED, "", parsedOk);
                flowParameter->parametersSet |= VEHPARS_CALIBRATORSPEED_SET;
            }
            // set begin and end
            flowParameter->depart = attrs.getSUMOTimeReporting(SUMO_ATTR_BEGIN, "", parsedOk);
            flowParameter->repetitionEnd = attrs.getSUMOTimeReporting(SUMO_ATTR_END, "", parsedOk);
            if (parsedOk) {
                // set tag
                myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_FLOW);
                // set vehicle parameters
                myCommonXMLStructure.getCurrentSumoBaseObject()->setVehicleParameter(flowParameter);
                // delete flow parameter (because in XMLStructure we have a copy)
                delete flowParameter;
            }
        }
    }
}


void
AdditionalHandler::parseRerouterAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const std::vector<std::string> edges = attrs.get<std::vector<std::string> >(SUMO_ATTR_EDGES, id.c_str(), parsedOk);
    // optional attributes
    const Position pos = attrs.getOpt<Position>(SUMO_ATTR_POSITION, id.c_str(), parsedOk, Position::INVALID);
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), parsedOk, "");
    const double probability = attrs.getOpt<double>(SUMO_ATTR_PROB, id.c_str(), parsedOk, 1);
    SUMOTime timeThreshold = attrs.getOptSUMOTimeReporting(SUMO_ATTR_HALTING_TIME_THRESHOLD, id.c_str(), parsedOk, 0);
    const std::vector<std::string> vehicleTypes = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_VTYPES, id.c_str(), parsedOk, std::vector<std::string>());
    const bool off = attrs.getOpt<bool>(SUMO_ATTR_OFF, id.c_str(), parsedOk, false);
    const bool optional = attrs.getOpt<bool>(SUMO_ATTR_OPTIONAL, id.c_str(), parsedOk, false);
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_REROUTER);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_EDGES, edges);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addPositionAttribute(SUMO_ATTR_POSITION, pos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_PROB, probability);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_HALTING_TIME_THRESHOLD, timeThreshold);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_VTYPES, vehicleTypes);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_OFF, off);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_OPTIONAL, optional);
    }
}


void
AdditionalHandler::parseRerouterIntervalAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const SUMOTime begin = attrs.getSUMOTimeReporting(SUMO_ATTR_BEGIN, "", parsedOk);
    const SUMOTime end = attrs.getSUMOTimeReporting(SUMO_ATTR_END, "", parsedOk);
    // check parent
    checkParent(SUMO_TAG_INTERVAL, {SUMO_TAG_REROUTER}, parsedOk);
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_INTERVAL);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_BEGIN, begin);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_END, end);
    }
}


void
AdditionalHandler::parseClosingLaneRerouteAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string laneID = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    // optional attributes
    const std::string disallow = attrs.getOpt<std::string>(SUMO_ATTR_DISALLOW, "", parsedOk, "");
    const std::string allow = attrs.getOpt<std::string>(SUMO_ATTR_ALLOW, "", parsedOk, !disallow.size() ? "authority" : "");
    // check parent
    checkParent(SUMO_TAG_CLOSING_LANE_REROUTE, {SUMO_TAG_INTERVAL}, parsedOk);
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_CLOSING_LANE_REROUTE);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, laneID);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ALLOW, allow);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_DISALLOW, disallow);
    }
}


void
AdditionalHandler::parseClosingRerouteAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string edgeID = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    // optional attributes
    const std::string disallow = attrs.getOpt<std::string>(SUMO_ATTR_DISALLOW, "", parsedOk, "");
    const std::string allow = attrs.getOpt<std::string>(SUMO_ATTR_ALLOW, "", parsedOk, !disallow.size() ? "authority" : "");
    // check parent
    checkParent(SUMO_TAG_CLOSING_REROUTE, {SUMO_TAG_INTERVAL}, parsedOk);
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_CLOSING_REROUTE);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, edgeID);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ALLOW, allow);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_DISALLOW, disallow);
    }
}


void
AdditionalHandler::parseDestProbRerouteAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string edgeID = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const double probability = attrs.getOpt<double>(SUMO_ATTR_PROB, "", parsedOk, 1);
    // check parent
    checkParent(SUMO_TAG_DEST_PROB_REROUTE, {SUMO_TAG_INTERVAL}, parsedOk);
    // continue if flag is ok
    if (parsedOk) {
        if (probability < 0) {
            writeError(TLF("Probability of % must be equal or greater than 0", toString(SUMO_TAG_DEST_PROB_REROUTE)));
        } else {
            // set tag
            myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_DEST_PROB_REROUTE);
            // add all attributes
            myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, edgeID);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_PROB, probability);
        }
    }
}


void
AdditionalHandler::parseParkingAreaRerouteAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string parkingAreaID = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const double probability = attrs.getOpt<double>(SUMO_ATTR_PROB, "", parsedOk, 1);
    // optional attributes
    const bool visible = attrs.getOpt<bool>(SUMO_ATTR_VISIBLE, "", parsedOk, false);
    // check parent
    checkParent(SUMO_TAG_PARKING_AREA_REROUTE, {SUMO_TAG_INTERVAL}, parsedOk);
    // continue if flag is ok
    if (parsedOk) {
        if (probability < 0) {
            writeError(TLF("Probability of % must be equal or greater than 0", toString(SUMO_TAG_PARKING_AREA_REROUTE)));
        } else {
            // set tag
            myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_PARKING_AREA_REROUTE);
            // add all attributes
            myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, parkingAreaID);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_PROB, probability);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_VISIBLE, visible);
        }
    }
}


void
AdditionalHandler::parseRouteProbRerouteAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string routeID = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const double probability = attrs.getOpt<double>(SUMO_ATTR_PROB, "", parsedOk, 1);
    // check parent
    checkParent(SUMO_TAG_ROUTE_PROB_REROUTE, {SUMO_TAG_INTERVAL}, parsedOk);
    // continue if flag is ok
    if (parsedOk) {
        if (probability < 0) {
            writeError(TLF("Probability of % must be equal or greater than 0", toString(SUMO_TAG_ROUTE_PROB_REROUTE)));
        } else {
            // set tag
            myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_ROUTE_PROB_REROUTE);
            // add all attributes
            myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, routeID);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_PROB, probability);
        }
    }
}


void
AdditionalHandler::parseRouteProbeAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const std::string edge = attrs.get<std::string>(SUMO_ATTR_EDGE, id.c_str(), parsedOk);
    const std::string file = attrs.get<std::string>(SUMO_ATTR_FILE, id.c_str(), parsedOk);
    const SUMOTime period = attrs.getOptPeriod(id.c_str(), parsedOk, SUMOTime_MAX_PERIOD);
    const SUMOTime begin = attrs.getOptSUMOTimeReporting(SUMO_ATTR_BEGIN, id.c_str(), parsedOk, -1);
    // optional attributes
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), parsedOk, "");
    const std::vector<std::string> vehicleTypes = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_VTYPES, id.c_str(), parsedOk, std::vector<std::string>());
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_ROUTEPROBE);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_EDGE, edge);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_FILE, file);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_PERIOD, period);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_BEGIN, begin);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_VTYPES, vehicleTypes);
    }
}


void
AdditionalHandler::parseVaporizerAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string edgeID = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    SUMOTime begin = attrs.getSUMOTimeReporting(SUMO_ATTR_BEGIN, nullptr, parsedOk);
    SUMOTime end = attrs.getSUMOTimeReporting(SUMO_ATTR_END, nullptr, parsedOk);
    // optional attributes
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, edgeID.c_str(), parsedOk, "");
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_VAPORIZER);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, edgeID);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_BEGIN, begin);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_END, end);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
    }
}


void
AdditionalHandler::parseTractionSubstation(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    // optional attributes
    const Position pos = attrs.getOpt<Position>(SUMO_ATTR_POSITION, id.c_str(), parsedOk, Position::INVALID);
    const double voltage = attrs.getOpt<double>(SUMO_ATTR_VOLTAGE, id.c_str(), parsedOk, 600);
    const double currentLimit = attrs.getOpt<double>(SUMO_ATTR_CURRENTLIMIT, id.c_str(), parsedOk, 400);
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_TRACTION_SUBSTATION);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addPositionAttribute(SUMO_ATTR_POSITION, pos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_VOLTAGE, voltage);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_CURRENTLIMIT, currentLimit);
    }
}


void
AdditionalHandler::parseOverheadWire(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const std::string substationID = attrs.get<std::string>(SUMO_ATTR_SUBSTATIONID, id.c_str(), parsedOk);
    const std::vector<std::string> laneIDs = attrs.get<std::vector<std::string> >(SUMO_ATTR_LANES, id.c_str(), parsedOk);
    // optional attributes
    const double startPos = attrs.getOpt<double>(SUMO_ATTR_STARTPOS, id.c_str(), parsedOk, 0);
    const double endPos = attrs.getOpt<double>(SUMO_ATTR_ENDPOS, id.c_str(), parsedOk, INVALID_DOUBLE);
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), parsedOk, false);
    const std::vector<std::string> forbiddenInnerLanes = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_OVERHEAD_WIRE_FORBIDDEN, "", parsedOk);
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_OVERHEAD_WIRE_SECTION);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_SUBSTATIONID, substationID);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_LANES, laneIDs);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_STARTPOS, startPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_ENDPOS, endPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_FRIENDLY_POS, friendlyPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_OVERHEAD_WIRE_FORBIDDEN, forbiddenInnerLanes);
    }
}


void
AdditionalHandler::parseOverheadWireClamp(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const std::string substationId = attrs.get<std::string>(SUMO_ATTR_SUBSTATIONID, id.c_str(), parsedOk);
    const std::string wireClampStart = attrs.get<std::string>(SUMO_ATTR_OVERHEAD_WIRECLAMP_START, id.c_str(), parsedOk);
    const std::string wireClampLaneStart = attrs.get<std::string>(SUMO_ATTR_OVERHEAD_WIRECLAMP_LANESTART, id.c_str(), parsedOk);
    const std::string wireClampEnd = attrs.get<std::string>(SUMO_ATTR_OVERHEAD_WIRECLAMP_END, id.c_str(), parsedOk);
    const std::string wireClampLaneEnd = attrs.get<std::string>(SUMO_ATTR_OVERHEAD_WIRECLAMP_LANEEND, id.c_str(), parsedOk);
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_OVERHEAD_WIRE_CLAMP);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_SUBSTATIONID, substationId);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_OVERHEAD_WIRECLAMP_START, wireClampStart);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_OVERHEAD_WIRECLAMP_LANESTART, wireClampLaneStart);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_OVERHEAD_WIRECLAMP_END, wireClampEnd);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_OVERHEAD_WIRECLAMP_LANEEND, wireClampLaneEnd);
    }
}


void
AdditionalHandler::parsePolyAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const PositionVector shapeStr = attrs.get<PositionVector>(SUMO_ATTR_SHAPE, id.c_str(), parsedOk);
    // optional attributes
    const RGBColor color = attrs.getOpt<RGBColor>(SUMO_ATTR_COLOR, id.c_str(), parsedOk, RGBColor::RED);
    const bool geo = attrs.getOpt<bool>(SUMO_ATTR_GEO, id.c_str(), parsedOk, false);
    const bool fill = attrs.getOpt<bool>(SUMO_ATTR_FILL, id.c_str(), parsedOk, false);
    const double lineWidth = attrs.getOpt<double>(SUMO_ATTR_LINEWIDTH, id.c_str(), parsedOk, Shape::DEFAULT_LINEWIDTH);
    const double layer = attrs.getOpt<double>(SUMO_ATTR_LAYER, id.c_str(), parsedOk, Shape::DEFAULT_LAYER);
    const std::string type = attrs.getOpt<std::string>(SUMO_ATTR_TYPE, id.c_str(), parsedOk, Shape::DEFAULT_TYPE);
    const std::string imgFile = attrs.getOpt<std::string>(SUMO_ATTR_IMGFILE, id.c_str(), parsedOk, Shape::DEFAULT_IMG_FILE);
    const double angle = attrs.getOpt<double>(SUMO_ATTR_ANGLE, id.c_str(), parsedOk, Shape::DEFAULT_ANGLE);
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), parsedOk, "");
    const bool relativePath = attrs.getOpt<bool>(SUMO_ATTR_RELATIVEPATH, id.c_str(), parsedOk, Shape::DEFAULT_RELATIVEPATH);
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_POLY);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addPositionVectorAttribute(SUMO_ATTR_SHAPE, shapeStr);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addColorAttribute(SUMO_ATTR_COLOR, color);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_GEO, geo);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_FILL, fill);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_LINEWIDTH, lineWidth);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_LAYER, layer);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_TYPE, type);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_IMGFILE, imgFile);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_ANGLE, angle);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_RELATIVEPATH, relativePath);
    }
}


void
AdditionalHandler::parsePOIAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // check that x and y are defined together
    if ((attrs.hasAttribute(SUMO_ATTR_X) && !attrs.hasAttribute(SUMO_ATTR_Y)) ||
            (!attrs.hasAttribute(SUMO_ATTR_X) && attrs.hasAttribute(SUMO_ATTR_Y))) {
        writeError(TL("X and Y must be be defined together in POIs"));
        parsedOk = false;
    }
    // check that lane and pos are defined together
    if ((attrs.hasAttribute(SUMO_ATTR_LANE) && !attrs.hasAttribute(SUMO_ATTR_POSITION)) ||
            (!attrs.hasAttribute(SUMO_ATTR_LANE) && attrs.hasAttribute(SUMO_ATTR_POSITION))) {
        writeError(TL("lane and position must be defined together in POIs"));
        parsedOk = false;
    }
    // check that lon and lat are defined together
    if ((attrs.hasAttribute(SUMO_ATTR_LON) && !attrs.hasAttribute(SUMO_ATTR_LAT)) ||
            (!attrs.hasAttribute(SUMO_ATTR_LON) && attrs.hasAttribute(SUMO_ATTR_LAT))) {
        writeError(TL("lon and lat must be be defined together in POIs"));
        parsedOk = false;
    }
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    // special attributes
    const double x = attrs.getOpt<double>(SUMO_ATTR_X, id.c_str(), parsedOk, 0);
    const double y = attrs.getOpt<double>(SUMO_ATTR_Y, id.c_str(), parsedOk, 0);
    const std::string lane = attrs.getOpt<std::string>(SUMO_ATTR_LANE, "", parsedOk, "");
    const double pos = attrs.getOpt<double>(SUMO_ATTR_POSITION, id.c_str(), parsedOk, 0);
    const bool friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, id.c_str(), parsedOk, false);
    const double posLat = attrs.getOpt<double>(SUMO_ATTR_POSITION_LAT, id.c_str(), parsedOk, 0);
    const double lon = attrs.getOpt<double>(SUMO_ATTR_LON, id.c_str(), parsedOk, 0);
    const double lat = attrs.getOpt<double>(SUMO_ATTR_LAT, id.c_str(), parsedOk, 0);
    // optional attributes
    const RGBColor color = attrs.getOpt<RGBColor>(SUMO_ATTR_COLOR, id.c_str(), parsedOk, RGBColor::RED);
    const std::string type = attrs.getOpt<std::string>(SUMO_ATTR_TYPE, "", parsedOk, Shape::DEFAULT_TYPE);
    std::string icon = attrs.getOpt<std::string>(SUMO_ATTR_ICON, "", parsedOk, SUMOXMLDefinitions::POIIcons.getString(POIIcon::NONE));
    const double layer = attrs.getOpt<double>(SUMO_ATTR_LAYER, id.c_str(), parsedOk, Shape::DEFAULT_LAYER_POI);
    const std::string imgFile = attrs.getOpt<std::string>(SUMO_ATTR_IMGFILE, "", parsedOk, Shape::DEFAULT_IMG_FILE);
    const double width = attrs.getOpt<double>(SUMO_ATTR_WIDTH, id.c_str(), parsedOk, Shape::DEFAULT_IMG_WIDTH);
    const double height = attrs.getOpt<double>(SUMO_ATTR_HEIGHT, id.c_str(), parsedOk, Shape::DEFAULT_IMG_HEIGHT);
    const double angle = attrs.getOpt<double>(SUMO_ATTR_ANGLE, id.c_str(), parsedOk, Shape::DEFAULT_ANGLE);
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), parsedOk, "");
    const bool relativePath = attrs.getOpt<bool>(SUMO_ATTR_RELATIVEPATH, id.c_str(), parsedOk, Shape::DEFAULT_RELATIVEPATH);
    // check icon
    if (!SUMOXMLDefinitions::POIIcons.hasString(icon)) {
        WRITE_WARNING(TLF("Invalid icon % for POI '%', using default", icon, id));
        icon = "none";
    }
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_POI);
        // add attributes depending of Lane/Lanes
        if (attrs.hasAttribute(SUMO_ATTR_X) && attrs.hasAttribute(SUMO_ATTR_Y)) {
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_X, x);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_Y, y);
        } else if (attrs.hasAttribute(SUMO_ATTR_LANE) && attrs.hasAttribute(SUMO_ATTR_POSITION)) {
            myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_LANE, lane);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_POSITION, pos);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_FRIENDLY_POS, friendlyPos);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_POSITION_LAT, posLat);
        } else {
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_LON, lon);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_LAT, lat);
        }
        // add rest attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addColorAttribute(SUMO_ATTR_COLOR, color);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_TYPE, type);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ICON, icon);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_LAYER, layer);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_IMGFILE, imgFile);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_WIDTH, width);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_HEIGHT, height);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_ANGLE, angle);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addBoolAttribute(SUMO_ATTR_RELATIVEPATH, relativePath);
    }
}


void
AdditionalHandler::parseJpsWalkableAreaAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const PositionVector shapeStr = attrs.get<PositionVector>(SUMO_ATTR_SHAPE, id.c_str(), parsedOk);
    // optional attributes
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), parsedOk, "");
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(GNE_TAG_JPS_WALKABLEAREA);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addPositionVectorAttribute(SUMO_ATTR_SHAPE, shapeStr);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
    }
}


void
AdditionalHandler::parseJpsObstacleAttributes(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    const PositionVector shapeStr = attrs.get<PositionVector>(SUMO_ATTR_SHAPE, id.c_str(), parsedOk);
    // optional attributes
    const std::string name = attrs.getOpt<std::string>(SUMO_ATTR_NAME, id.c_str(), parsedOk, "");
    // continue if flag is ok
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(GNE_TAG_JPS_OBSTACLE);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addPositionVectorAttribute(SUMO_ATTR_SHAPE, shapeStr);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_NAME, name);
    }
}


void
AdditionalHandler::parseParameters(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // get key
    const std::string key = attrs.get<std::string>(SUMO_ATTR_KEY, nullptr, parsedOk);
    // get SumoBaseObject parent
    CommonXMLStructure::SumoBaseObject* SumoBaseObjectParent = myCommonXMLStructure.getCurrentSumoBaseObject()->getParentSumoBaseObject();
    // check parent
    if (SumoBaseObjectParent == nullptr) {
        writeError(TL("Parameters must be defined within an object."));
    } else if (SumoBaseObjectParent->getTag() == SUMO_TAG_ROOTFILE) {
        writeError(TL("Parameters cannot be defined in the additional file's root."));
    } else if (SumoBaseObjectParent->getTag() == SUMO_TAG_PARAM) {
        writeError(TL("Parameters cannot be defined within another parameter."));
    } else if (parsedOk) {
        // get tag str
        const std::string parentTagStr = toString(SumoBaseObjectParent->getTag());
        // circumventing empty string value
        const std::string value = attrs.hasAttribute(SUMO_ATTR_VALUE) ? attrs.getString(SUMO_ATTR_VALUE) : "";
        // show warnings if values are invalid
        if (key.empty()) {
            WRITE_WARNINGF(TL("Error parsing key from % generic parameter. Key cannot be empty."), parentTagStr);
        } else if (!SUMOXMLDefinitions::isValidParameterKey(key)) {
            WRITE_WARNINGF(TL("Error parsing key from % generic parameter. Key contains invalid characters."), parentTagStr);
        } else {
            WRITE_DEBUG("Inserting generic parameter '" + key + "|" + value + "' into " + parentTagStr);
            // insert parameter in SumoBaseObjectParent
            SumoBaseObjectParent->addParameter(key, value);
        }
    }
}


void
AdditionalHandler::checkParent(const SumoXMLTag currentTag, const std::vector<SumoXMLTag>& parentTags, bool& ok) {
    // check that parent SUMOBaseObject's tag is the parentTag
    CommonXMLStructure::SumoBaseObject* const parent = myCommonXMLStructure.getCurrentSumoBaseObject()->getParentSumoBaseObject();
    if ((parent != nullptr) &&
            (parentTags.size() > 0) &&
            (std::find(parentTags.begin(), parentTags.end(), parent->getTag()) == parentTags.end())) {
        const std::string id = parent->hasStringAttribute(SUMO_ATTR_ID) ? ", id: '" + parent->getStringAttribute(SUMO_ATTR_ID) + "'" : "";
        if (id.empty()) {
            writeError(TLF("'%' must be defined within the definition of a '%'.", toString(currentTag), toString(parentTags.front())));
        } else {
            writeError(TLF("'%' must be defined within the definition of a '%' (found % '%').", toString(currentTag), toString(parentTags.front()), toString(parent->getTag()), id));
        }
        ok = false;
    }
}


bool
AdditionalHandler::checkDetectPersons(const SumoXMLTag currentTag, const std::string& id, const std::string& detectPersons) {
    if (detectPersons.empty() || SUMOXMLDefinitions::PersonModeValues.hasString(detectPersons)) {
        return true;
    } else {
        writeError(TLF("Attribute '%' defined in % with id '%' doesn't have a valid value (given '%').", toString(SUMO_ATTR_DETECT_PERSONS), toString(currentTag), id, detectPersons));
        return false;
    }
}

/****************************************************************************/
