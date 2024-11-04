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
/// @file    RouteHandler.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Jun 2021
///
// The XML-Handler for route elements loading
/****************************************************************************/
#include <config.h>

#include <utils/common/MsgHandler.h>
#include <utils/common/RGBColor.h>
#include <utils/common/SUMOVehicleClass.h>
#include <utils/options/OptionsCont.h>
#include <utils/shapes/Shape.h>
#include <utils/vehicle/SUMOVehicleParserHelper.h>
#include <utils/xml/SUMOSAXHandler.h>
#include <utils/xml/XMLSubSys.h>
#include <utils/xml/NamespaceIDs.h>

#include "RouteHandler.h"


// ===========================================================================
// method definitions
// ===========================================================================

RouteHandler::RouteHandler(const std::string& filename, const bool hardFail) :
    myFilename(filename),
    myHardFail(hardFail),
    myFlowBeginDefault(string2time(OptionsCont::getOptions().getString("begin"))),
    myFlowEndDefault(string2time(OptionsCont::getOptions().getString("end"))) {
}


RouteHandler::~RouteHandler() {}


bool
RouteHandler::beginParseAttributes(SumoXMLTag tag, const SUMOSAXAttributes& attrs) {
    // open SUMOBaseOBject
    myCommonXMLStructure.openSUMOBaseOBject();
    // check tag
    try {
        switch (tag) {
            // vTypes
            case SUMO_TAG_VTYPE:
                parseVType(attrs);
                break;
            case SUMO_TAG_VTYPE_DISTRIBUTION:
                parseVTypeDistribution(attrs);
                break;
            // routes
            case SUMO_TAG_ROUTE:
                parseRoute(attrs);
                break;
            case SUMO_TAG_ROUTE_DISTRIBUTION:
                parseRouteDistribution(attrs);
                break;
            // vehicles
            case SUMO_TAG_TRIP:
                parseTrip(attrs);
                break;
            case SUMO_TAG_VEHICLE:
                parseVehicle(attrs);
                break;
            // flows
            case SUMO_TAG_FLOW:
                parseFlow(attrs);
                break;
            // stop
            case SUMO_TAG_STOP:
                parseStop(attrs);
                break;
            // persons
            case SUMO_TAG_PERSON:
                parsePerson(attrs);
                break;
            case SUMO_TAG_PERSONFLOW:
                parsePersonFlow(attrs);
                break;
            // person plans
            case SUMO_TAG_PERSONTRIP:
                parsePersonTrip(attrs);
                break;
            case SUMO_TAG_RIDE:
                parseRide(attrs);
                break;
            case SUMO_TAG_WALK:
                parseWalk(attrs);
                break;
            // container
            case SUMO_TAG_CONTAINER:
                parseContainer(attrs);
                break;
            case SUMO_TAG_CONTAINERFLOW:
                parseContainerFlow(attrs);
                break;
            // container plans
            case SUMO_TAG_TRANSPORT:
                parseTransport(attrs);
                break;
            case SUMO_TAG_TRANSHIP:
                parseTranship(attrs);
                break;
            // parameters
            case SUMO_TAG_PARAM:
                parseParameters(attrs);
                break;
            // other
            case SUMO_TAG_INTERVAL: {
                parseInterval(attrs);
                break;
            }
            default:
                // nested CFM attributes
                return parseNestedCFM(tag, attrs);
        }
    } catch (InvalidArgument& e) {
        WRITE_ERROR(e.what());
    }
    return true;
}


void
RouteHandler::endParseAttributes() {
    // get last inserted object
    CommonXMLStructure::SumoBaseObject* obj = myCommonXMLStructure.getCurrentSumoBaseObject();
    // close SUMOBaseOBject
    myCommonXMLStructure.closeSUMOBaseOBject();
    // check tag
    if (obj) {
        switch (obj->getTag()) {
            // specia case for route (because can be embedded)
            case SUMO_TAG_ROUTE:
                // only parse non-embedded and without distributionsroutes
                if ((obj->getStringAttribute(SUMO_ATTR_ID).size() > 0) &&
                        obj->getParentSumoBaseObject() &&
                        (obj->getParentSumoBaseObject()->getTag() != SUMO_TAG_ROUTE_DISTRIBUTION)) {
                    // parse route and all their childrens
                    parseSumoBaseObject(obj);
                    // delete object (and all of their childrens)
                    delete obj;
                }
                break;
            // demand elements
            case SUMO_TAG_VTYPE:
                // only parse vTypes without distributions
                if (obj->getParentSumoBaseObject() &&
                        (obj->getParentSumoBaseObject()->getTag() != SUMO_TAG_VTYPE_DISTRIBUTION)) {
                    // parse vType and all their childrens
                    parseSumoBaseObject(obj);
                    // delete object (and all of their childrens)
                    delete obj;
                }
                break;
            case SUMO_TAG_VTYPE_DISTRIBUTION:
            case SUMO_TAG_ROUTE_DISTRIBUTION:
            case SUMO_TAG_TRIP:
            case SUMO_TAG_VEHICLE:
            case SUMO_TAG_FLOW:
            case SUMO_TAG_PERSON:
            case SUMO_TAG_PERSONFLOW:
            case SUMO_TAG_CONTAINER:
            case SUMO_TAG_CONTAINERFLOW:
                // parse object and all their childrens
                parseSumoBaseObject(obj);
                // delete object (and all of their childrens)
                delete obj;
                break;
            default:
                break;
        }
    }
}


void
RouteHandler::parseSumoBaseObject(CommonXMLStructure::SumoBaseObject* obj) {
    // switch tag
    switch (obj->getTag()) {
        // vTypes
        case SUMO_TAG_VTYPE:
            buildVType(obj,
                       obj->getVehicleTypeParameter());
            break;
        case SUMO_TAG_VTYPE_DISTRIBUTION:
            buildVTypeDistribution(obj,
                                   obj->getStringAttribute(SUMO_ATTR_ID),
                                   obj->getIntAttribute(SUMO_ATTR_DETERMINISTIC),
                                   obj->getStringListAttribute(SUMO_ATTR_VTYPES),
                                   obj->getDoubleListAttribute(SUMO_ATTR_PROBS));
            break;
        // route
        case SUMO_TAG_ROUTE:
            if (obj->getStringAttribute(SUMO_ATTR_ID).empty()) {
                buildEmbeddedRoute(obj,
                                   obj->getStringListAttribute(SUMO_ATTR_EDGES),
                                   obj->getColorAttribute(SUMO_ATTR_COLOR),
                                   obj->getIntAttribute(SUMO_ATTR_REPEAT),
                                   obj->getTimeAttribute(SUMO_ATTR_CYCLETIME),
                                   obj->getParameters());
            } else {
                buildRoute(obj,
                           obj->getStringAttribute(SUMO_ATTR_ID),
                           obj->getVClass(),
                           obj->getStringListAttribute(SUMO_ATTR_EDGES),
                           obj->getColorAttribute(SUMO_ATTR_COLOR),
                           obj->getIntAttribute(SUMO_ATTR_REPEAT),
                           obj->getTimeAttribute(SUMO_ATTR_CYCLETIME),
                           obj->getDoubleAttribute(SUMO_ATTR_PROB),
                           obj->getParameters());
            }
            break;
        case SUMO_TAG_ROUTE_DISTRIBUTION:
            buildRouteDistribution(obj,
                                   obj->getStringAttribute(SUMO_ATTR_ID),
                                   obj->getStringListAttribute(SUMO_ATTR_ROUTES),
                                   obj->getDoubleListAttribute(SUMO_ATTR_PROBS));
            break;
        // vehicles
        case SUMO_TAG_TRIP:
            if (obj->hasStringAttribute(SUMO_ATTR_FROM_JUNCTION) &&
                       obj->hasStringAttribute(SUMO_ATTR_TO_JUNCTION)) {
                // build trip with from-to junctions
                buildTripJunctions(obj,
                                   obj->getVehicleParameter(),
                                   obj->getStringAttribute(SUMO_ATTR_FROM_JUNCTION),
                                   obj->getStringAttribute(SUMO_ATTR_TO_JUNCTION));
            } else if (obj->hasStringAttribute(SUMO_ATTR_FROM_TAZ) &&
                       obj->hasStringAttribute(SUMO_ATTR_TO_TAZ)) {
                // build trip with from-to TAZs
                buildTripTAZs(obj,
                              obj->getVehicleParameter(),
                              obj->getStringAttribute(SUMO_ATTR_FROM_TAZ),
                              obj->getStringAttribute(SUMO_ATTR_TO_TAZ));
            } else {
                // build trip with from-to edges
                buildTrip(obj,
                          obj->getVehicleParameter(),
                          obj->hasStringAttribute(SUMO_ATTR_FROM)? obj->getStringAttribute(SUMO_ATTR_FROM) : "",
                          obj->hasStringAttribute(SUMO_ATTR_TO)? obj->getStringAttribute(SUMO_ATTR_TO) : "");
            }
            break;
        case SUMO_TAG_VEHICLE:
            if (obj->hasStringAttribute(SUMO_ATTR_ROUTE)) {
                buildVehicleOverRoute(obj,
                                      obj->getVehicleParameter());
            }
            break;
        // flows
        case SUMO_TAG_FLOW:
            if (obj->hasStringAttribute(SUMO_ATTR_ROUTE)) {
                // build flow over route
                buildFlowOverRoute(obj,
                                   obj->getVehicleParameter());
            } else if (obj->hasStringAttribute(SUMO_ATTR_FROM_JUNCTION) &&
                       obj->hasStringAttribute(SUMO_ATTR_TO_JUNCTION)) {
                // build flow with from-to junctions
                buildFlowJunctions(obj,
                                   obj->getVehicleParameter(),
                                   obj->getStringAttribute(SUMO_ATTR_FROM_JUNCTION),
                                   obj->getStringAttribute(SUMO_ATTR_TO_JUNCTION));
            } else if (obj->hasStringAttribute(SUMO_ATTR_FROM_TAZ) &&
                       obj->hasStringAttribute(SUMO_ATTR_TO_TAZ)) {
                // build flow with from-to TAZs
                buildFlowTAZs(obj,
                              obj->getVehicleParameter(),
                              obj->getStringAttribute(SUMO_ATTR_FROM_TAZ),
                              obj->getStringAttribute(SUMO_ATTR_TO_TAZ));
            } else {
                // build flow with from-to edges
                buildFlow(obj,
                    obj->getVehicleParameter(),
                    obj->hasStringAttribute(SUMO_ATTR_FROM)? obj->getStringAttribute(SUMO_ATTR_FROM) : "",
                    obj->hasStringAttribute(SUMO_ATTR_TO)? obj->getStringAttribute(SUMO_ATTR_TO) : "");
            }
            break;
        // persons
        case SUMO_TAG_PERSON:
            buildPerson(obj,
                        obj->getVehicleParameter());
            break;
        case SUMO_TAG_PERSONFLOW:
            buildPersonFlow(obj,
                            obj->getVehicleParameter());
            break;
        // person plans
        case SUMO_TAG_PERSONTRIP:
            buildPersonTrip(obj,
                            obj->getPlanParameters(),
                            obj->getDoubleAttribute(SUMO_ATTR_ARRIVALPOS),
                            obj->getStringListAttribute(SUMO_ATTR_VTYPES),
                            obj->getStringListAttribute(SUMO_ATTR_MODES),
                            obj->getStringListAttribute(SUMO_ATTR_LINES),
                            obj->getDoubleAttribute(SUMO_ATTR_WALKFACTOR),
                            obj->getStringAttribute(SUMO_ATTR_GROUP));
            break;
        case SUMO_TAG_RIDE:
            buildRide(obj,
                      obj->getPlanParameters(),
                      obj->getDoubleAttribute(SUMO_ATTR_ARRIVALPOS),
                      obj->getStringListAttribute(SUMO_ATTR_LINES),
                      obj->getStringAttribute(SUMO_ATTR_GROUP));
            break;
        case SUMO_TAG_WALK:
            buildWalk(obj,
                      obj->getPlanParameters(),
                      obj->getDoubleAttribute(SUMO_ATTR_ARRIVALPOS),
                      obj->getDoubleAttribute(SUMO_ATTR_SPEED),
                      obj->getTimeAttribute(SUMO_ATTR_DURATION));
            break;
        // container
        case SUMO_TAG_CONTAINER:
            buildContainer(obj,
                           obj->getVehicleParameter());
            break;
        case SUMO_TAG_CONTAINERFLOW:
            buildContainerFlow(obj,
                               obj->getVehicleParameter());
            break;
        // container plans
        case SUMO_TAG_TRANSPORT:
            buildTransport(obj,
                           obj->getPlanParameters(),
                           obj->getDoubleAttribute(SUMO_ATTR_ARRIVALPOS),
                           obj->getStringListAttribute(SUMO_ATTR_LINES),
                           obj->getStringAttribute(SUMO_ATTR_GROUP));
            break;
        case SUMO_TAG_TRANSHIP:
            buildTranship(obj,
                          obj->getPlanParameters(),
                          obj->getDoubleAttribute(SUMO_ATTR_ARRIVALPOS),
                          obj->getDoubleAttribute(SUMO_ATTR_DEPARTPOS),
                          obj->getDoubleAttribute(SUMO_ATTR_SPEED),
                          obj->getTimeAttribute(SUMO_ATTR_DURATION));
            break;
        // stopss
        case SUMO_TAG_STOP:
            buildStop(obj,
                      obj->getPlanParameters(),
                      obj->getStopParameter());
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
RouteHandler::isErrorCreatingElement() const {
    return myErrorCreatingElement;
}


void
RouteHandler::writeError(const std::string& error) {
    WRITE_ERROR(error);
    myErrorCreatingElement = true;
}


void
RouteHandler::writeErrorInvalidID(const SumoXMLTag tag, const std::string& id) {
    WRITE_ERRORF(TL("Could not build % with ID '%' in netedit; ID contains invalid characters."), toString(tag), id);
    myErrorCreatingElement = true;
}


void
RouteHandler::writeErrorInvalidDistribution(const SumoXMLTag tag, const std::string& id) {
    WRITE_ERRORF(TL("Could not build % with ID '%' in netedit; Distinct number of distribution values and probabilities."), toString(tag), id);
    myErrorCreatingElement = true;
}


void
RouteHandler::parseVType(const SUMOSAXAttributes& attrs) {
    // parse vehicleType
    SUMOVTypeParameter* vehicleTypeParameter = SUMOVehicleParserHelper::beginVTypeParsing(attrs, myHardFail, myFilename);
    if (vehicleTypeParameter) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_VTYPE);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->setVehicleTypeParameter(vehicleTypeParameter);
        // delete vehicleType parameter (because in XMLStructure we have a copy)
        delete vehicleTypeParameter;
    }
}


void
RouteHandler::parseVTypeDistribution(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    // optional attributes
    const int deterministic = attrs.getOpt<int>(SUMO_ATTR_DETERMINISTIC, id.c_str(), parsedOk, -1);
    const std::vector<std::string> vTypes = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_VTYPES, id.c_str(), parsedOk);
    const std::vector<double> probabilities = attrs.getOpt<std::vector<double> >(SUMO_ATTR_PROBS, id.c_str(), parsedOk);
    if (parsedOk) {
        if (!SUMOXMLDefinitions::isValidVehicleID(id)) {
            writeErrorInvalidID(SUMO_TAG_VTYPE_DISTRIBUTION, id);
        } else if (vTypes.size() != probabilities.size()) {
            writeErrorInvalidDistribution(SUMO_TAG_VTYPE_DISTRIBUTION, id);
        } else {
            // set tag
            myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_VTYPE_DISTRIBUTION);
            // add all attributes
            myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addIntAttribute(SUMO_ATTR_DETERMINISTIC, deterministic);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_VTYPES, vTypes);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleListAttribute(SUMO_ATTR_PROBS, probabilities);
        }
    }
}


void
RouteHandler::parseRoute(const SUMOSAXAttributes& attrs) {
    // get embedded route flag
    const bool embeddedRoute = isEmbeddedRoute(attrs);
    // first check if this is an embedded route
    if ((embeddedRoute && attrs.hasAttribute(SUMO_ATTR_ID)) || (!embeddedRoute && !attrs.hasAttribute(SUMO_ATTR_ID))) {
        writeError(TL("a route must be defined either within a vehicle/flow or with an ID attribute"));
    } else {
        // declare Ok Flag
        bool parsedOk = true;
        // special case for ID
        const std::string id = attrs.getOpt<std::string>(SUMO_ATTR_ID, "", parsedOk, "");
        // needed attributes
        const std::vector<std::string> edges = attrs.get<std::vector<std::string> >(SUMO_ATTR_EDGES, id.c_str(), parsedOk);
        // optional attributes
        SUMOVehicleClass vClass = SUMOVehicleParserHelper::parseVehicleClass(attrs, id);
        const RGBColor color = attrs.getOpt<RGBColor>(SUMO_ATTR_COLOR, id.c_str(), parsedOk, RGBColor::INVISIBLE);
        const int repeat = attrs.getOpt<int>(SUMO_ATTR_REPEAT, id.c_str(), parsedOk, 0);
        const SUMOTime cycleTime = attrs.getOptSUMOTimeReporting(SUMO_ATTR_CYCLETIME, id.c_str(), parsedOk, 0);
        const double probability = attrs.getOpt<double>(SUMO_ATTR_PROB, id.c_str(), parsedOk, 0);
        if (parsedOk) {
            if (!id.empty() && !SUMOXMLDefinitions::isValidVehicleID(id)) {
                writeErrorInvalidID(SUMO_TAG_ROUTE, id);
            } else if (cycleTime < 0) {
                writeError(TLF("cycleTime of % must be equal or greater than 0", toString(SUMO_TAG_DEST_PROB_REROUTE)));
            } else {
                // set tag
                myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_ROUTE);
                // add all attributes
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
                myCommonXMLStructure.getCurrentSumoBaseObject()->setVClass(vClass);
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_EDGES, edges);
                myCommonXMLStructure.getCurrentSumoBaseObject()->addColorAttribute(SUMO_ATTR_COLOR, color);
                myCommonXMLStructure.getCurrentSumoBaseObject()->addIntAttribute(SUMO_ATTR_REPEAT, repeat);
                myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_CYCLETIME, cycleTime);
                myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_PROB, probability);
            }
        }
    }
}


void
RouteHandler::parseRouteDistribution(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // needed attributes
    const std::string id = attrs.get<std::string>(SUMO_ATTR_ID, "", parsedOk);
    // optional attributes
    const std::vector<std::string> routes = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_ROUTES, id.c_str(), parsedOk);
    const std::vector<double> probabilities = attrs.getOpt<std::vector<double> >(SUMO_ATTR_PROBS, id.c_str(), parsedOk);
    if (parsedOk) {
        if (!SUMOXMLDefinitions::isValidVehicleID(id)) {
            writeErrorInvalidID(SUMO_TAG_ROUTE_DISTRIBUTION, id);
        } else if (routes.size() != probabilities.size()) {
            writeErrorInvalidDistribution(SUMO_TAG_ROUTE_DISTRIBUTION, id);
        } else {
            // set tag
            myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_ROUTE_DISTRIBUTION);
            // add all attributes
            myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ID, id);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_ROUTES, routes);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleListAttribute(SUMO_ATTR_PROBS, probabilities);
        }
    }
}


void
RouteHandler::parseTrip(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // parse vehicle
    SUMOVehicleParameter* tripParameter = SUMOVehicleParserHelper::parseVehicleAttributes(SUMO_TAG_TRIP, attrs, myHardFail);
    if (tripParameter) {
        // check from/to edge/junction
        if ((attrs.hasAttribute(SUMO_ATTR_FROM) + attrs.hasAttribute(SUMO_ATTR_FROM_JUNCTION) + attrs.hasAttribute(SUMO_ATTR_FROM_TAZ)) > 1) {
            writeError(TL("Attributes 'from', 'fromJunction' and 'fromTaz' cannot be defined together"));
        } else if ((attrs.hasAttribute(SUMO_ATTR_TO) + attrs.hasAttribute(SUMO_ATTR_TO_JUNCTION) + attrs.hasAttribute(SUMO_ATTR_TO_TAZ)) > 1) {
            writeError(TL("Attributes 'to', 'toJunction' and 'toTaz' cannot be defined together"));
        } else if (attrs.hasAttribute(SUMO_ATTR_FROM_JUNCTION) && attrs.hasAttribute(SUMO_ATTR_TO_JUNCTION)) {
            // from-to attributes
            const std::string fromJunction = attrs.get<std::string>(SUMO_ATTR_FROM_JUNCTION, tripParameter->id.c_str(), parsedOk);
            const std::string toJunction = attrs.get<std::string>(SUMO_ATTR_TO_JUNCTION, tripParameter->id.c_str(), parsedOk);
            if (parsedOk) {
                // set tag
                myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_TRIP);
                // set vehicle parameters
                myCommonXMLStructure.getCurrentSumoBaseObject()->setVehicleParameter(tripParameter);
                // add other attributes
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_FROM_JUNCTION, fromJunction);
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_TO_JUNCTION, toJunction);
            }
        } else if (attrs.hasAttribute(SUMO_ATTR_FROM_TAZ) && attrs.hasAttribute(SUMO_ATTR_TO_TAZ)) {
            // from-to attributes
            const std::string fromJunction = attrs.get<std::string>(SUMO_ATTR_FROM_TAZ, tripParameter->id.c_str(), parsedOk);
            const std::string toJunction = attrs.get<std::string>(SUMO_ATTR_TO_TAZ, tripParameter->id.c_str(), parsedOk);
            if (parsedOk) {
                // set tag
                myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_TRIP);
                // set vehicle parameters
                myCommonXMLStructure.getCurrentSumoBaseObject()->setVehicleParameter(tripParameter);
                // add other attributes
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_FROM_TAZ, fromJunction);
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_TO_TAZ, toJunction);
            }
        } else {
            // from-to attributes
            const std::string from = attrs.getOpt<std::string>(SUMO_ATTR_FROM, tripParameter->id.c_str(), parsedOk, "");
            const std::string to = attrs.getOpt<std::string>(SUMO_ATTR_TO, tripParameter->id.c_str(), parsedOk, "");
            // optional attributes
            const std::vector<std::string> via = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_VIA, tripParameter->id.c_str(), parsedOk);
            if (parsedOk) {
                // set tag
                myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_TRIP);
                // set vehicle parameters
                myCommonXMLStructure.getCurrentSumoBaseObject()->setVehicleParameter(tripParameter);
                // add other attributes
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_FROM, from);
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_TO, to);
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_VIA, via);
            }
        }
        // delete trip parameter (because in XMLStructure we have a copy)
        delete tripParameter;
    }
}


void
RouteHandler::parseVehicle(const SUMOSAXAttributes& attrs) {
    // first parse vehicle
    SUMOVehicleParameter* vehicleParameter = SUMOVehicleParserHelper::parseVehicleAttributes(SUMO_TAG_VEHICLE, attrs, myHardFail);
    if (vehicleParameter) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_VEHICLE);
        // set vehicle parameters
        myCommonXMLStructure.getCurrentSumoBaseObject()->setVehicleParameter(vehicleParameter);
        // delete vehicle parameter (because in XMLStructure we have a copy)
        delete vehicleParameter;
    }
}


void
RouteHandler::parseFlow(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // first parse flow
    SUMOVehicleParameter* flowParameter = SUMOVehicleParserHelper::parseFlowAttributes(SUMO_TAG_FLOW, attrs, myHardFail, true, myFlowBeginDefault, myFlowEndDefault);
    if (flowParameter) {
        // set vehicle parameters
        myCommonXMLStructure.getCurrentSumoBaseObject()->setVehicleParameter(flowParameter);
        // check from/to edge/junction
        if ((attrs.hasAttribute(SUMO_ATTR_FROM) + attrs.hasAttribute(SUMO_ATTR_FROM_JUNCTION) + attrs.hasAttribute(SUMO_ATTR_FROM_TAZ)) > 1) {
            writeError(TL("Attributes 'from', 'fromJunction' and 'fromTaz' cannot be defined together"));
        } else if ((attrs.hasAttribute(SUMO_ATTR_TO) + attrs.hasAttribute(SUMO_ATTR_TO_JUNCTION) + attrs.hasAttribute(SUMO_ATTR_TO_TAZ)) > 1) {
            writeError(TL("Attributes 'to', 'toJunction' and 'toTaz' cannot be defined together"));
        } else if (attrs.hasAttribute(SUMO_ATTR_FROM) && attrs.hasAttribute(SUMO_ATTR_TO)) {
            // from-to attributes
            const std::string from = attrs.get<std::string>(SUMO_ATTR_FROM, flowParameter->id.c_str(), parsedOk);
            const std::string to = attrs.get<std::string>(SUMO_ATTR_TO, flowParameter->id.c_str(), parsedOk);
            // optional attributes
            const std::vector<std::string> via = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_VIA, flowParameter->id.c_str(), parsedOk);
            if (parsedOk) {
                // set tag
                myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_FLOW);
                // add other attributes
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_FROM, from);
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_TO, to);
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_VIA, via);
            }
        } else if (attrs.hasAttribute(SUMO_ATTR_FROM_JUNCTION) && attrs.hasAttribute(SUMO_ATTR_TO_JUNCTION)) {
            // from-to attributes
            const std::string fromJunction = attrs.get<std::string>(SUMO_ATTR_FROM_JUNCTION, flowParameter->id.c_str(), parsedOk);
            const std::string toJunction = attrs.get<std::string>(SUMO_ATTR_TO_JUNCTION, flowParameter->id.c_str(), parsedOk);
            if (parsedOk) {
                // set tag
                myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_FLOW);
                // add other attributes
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_FROM_JUNCTION, fromJunction);
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_TO_JUNCTION, toJunction);
            }
        } else if (attrs.hasAttribute(SUMO_ATTR_FROM_TAZ) && attrs.hasAttribute(SUMO_ATTR_TO_TAZ)) {
            // from-to attributes
            const std::string fromJunction = attrs.get<std::string>(SUMO_ATTR_FROM_TAZ, flowParameter->id.c_str(), parsedOk);
            const std::string toJunction = attrs.get<std::string>(SUMO_ATTR_TO_TAZ, flowParameter->id.c_str(), parsedOk);
            if (parsedOk) {
                // set tag
                myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_FLOW);
                // add other attributes
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_FROM_TAZ, fromJunction);
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_TO_TAZ, toJunction);
            }
        } else if (attrs.hasAttribute(SUMO_ATTR_ROUTE)) {
            // from-to attributes
            const std::string route = attrs.get<std::string>(SUMO_ATTR_ROUTE, flowParameter->id.c_str(), parsedOk);
            if (parsedOk) {
                // set tag
                myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_FLOW);
                // add other attributes
                myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_ROUTE, route);
            }
        } else {
            // set tag
            myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_FLOW);
        }
        // delete flow parameter (because in XMLStructure we have a copy)
        delete flowParameter;
    }
}


void
RouteHandler::parseStop(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // declare stop
    SUMOVehicleParameter::Stop stop;
    // plan parameters
    const auto planParameters = CommonXMLStructure::PlanParameters(myCommonXMLStructure.getCurrentSumoBaseObject(), attrs, parsedOk);
    // get parents
    std::vector<SumoXMLTag> stopParents;
    stopParents.insert(stopParents.end(), NamespaceIDs::vehicles.begin(), NamespaceIDs::vehicles.end());
    stopParents.insert(stopParents.end(), NamespaceIDs::routes.begin(), NamespaceIDs::routes.end());
    stopParents.insert(stopParents.end(), NamespaceIDs::persons.begin(), NamespaceIDs::persons.end());
    stopParents.insert(stopParents.end(), NamespaceIDs::containers.begin(), NamespaceIDs::containers.end());
    //  check parents
    checkParent(SUMO_TAG_STOP, stopParents, parsedOk);
    // parse stop
    if (parsedOk && parseStopParameters(stop, attrs)) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_STOP);
        // add stop attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->setPlanParameters(planParameters);
        myCommonXMLStructure.getCurrentSumoBaseObject()->setStopParameter(stop);
    }
}


void
RouteHandler::parsePerson(const SUMOSAXAttributes& attrs) {
    // first parse vehicle
    SUMOVehicleParameter* personParameter = SUMOVehicleParserHelper::parseVehicleAttributes(SUMO_TAG_PERSON, attrs, myHardFail);
    if (personParameter) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_PERSON);
        // set vehicle parameter
        myCommonXMLStructure.getCurrentSumoBaseObject()->setVehicleParameter(personParameter);
        // delete person parameter (because in XMLStructure we have a copy)
        delete personParameter;
    }
}


void
RouteHandler::parsePersonFlow(const SUMOSAXAttributes& attrs) {
    // first parse flow
    SUMOVehicleParameter* personFlowParameter = SUMOVehicleParserHelper::parseFlowAttributes(SUMO_TAG_PERSONFLOW, attrs, myHardFail, true, myFlowBeginDefault, myFlowEndDefault);
    if (personFlowParameter) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_PERSONFLOW);
        // set vehicle parameter
        myCommonXMLStructure.getCurrentSumoBaseObject()->setVehicleParameter(personFlowParameter);
        // delete person flow parameter (because in XMLStructure we have a copy)
        delete personFlowParameter;
    }
}


void
RouteHandler::parsePersonTrip(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // plan parameters
    const auto planParameters = CommonXMLStructure::PlanParameters(myCommonXMLStructure.getCurrentSumoBaseObject(), attrs, parsedOk);
    // optional attributes
    const std::vector<std::string> via = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_VIA, "", parsedOk);
    const std::vector<std::string> vTypes = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_VTYPES, "", parsedOk);
    const std::vector<std::string> lines = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_LINES, "", parsedOk);
    std::vector<std::string> modes = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_MODES, "", parsedOk);
    const double departPos = attrs.getOpt<double>(SUMO_ATTR_DEPARTPOS, "", parsedOk, -1);
    const double arrivalPos = attrs.getOpt<double>(SUMO_ATTR_ARRIVALPOS, "", parsedOk, -1);
    const double walkFactor = attrs.getOpt<double>(SUMO_ATTR_WALKFACTOR, "", parsedOk, 0);
    const std::string group = attrs.getOpt<std::string>(SUMO_ATTR_GROUP, "", parsedOk, "");
    // check modes
    SVCPermissions dummyModeSet;
    std::string dummyError;
    if (!SUMOVehicleParameter::parsePersonModes(toString(modes), toString(SUMO_TAG_PERSONTRIP), "", dummyModeSet, dummyError)) {
        WRITE_WARNING(dummyError);
        modes.clear();
    }
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_PERSONTRIP);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->setPlanParameters(planParameters);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_VTYPES, vTypes);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_MODES, modes);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_LINES, lines);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_DEPARTPOS, departPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_ARRIVALPOS, arrivalPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_WALKFACTOR, walkFactor);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_GROUP, group);
    }
}


void
RouteHandler::parseWalk(const SUMOSAXAttributes& attrs) {
    if (attrs.hasAttribute(SUMO_ATTR_SPEED) && attrs.hasAttribute(SUMO_ATTR_DURATION)) {
        WRITE_ERROR(TL("Speed and duration attributes cannot be defined together in walks"));
    } else {
        // declare Ok Flag
        bool parsedOk = true;
        // plan parameters
        const auto planParameters = CommonXMLStructure::PlanParameters(myCommonXMLStructure.getCurrentSumoBaseObject(), attrs, parsedOk);
        // optional attributes
        const double departPos = attrs.getOpt<double>(SUMO_ATTR_DEPARTPOS, "", parsedOk, -1);
        const double arrivalPos = attrs.getOpt<double>(SUMO_ATTR_ARRIVALPOS, "", parsedOk, -1);
        const double speed = attrs.getOpt<double>(SUMO_ATTR_SPEED, "", parsedOk, 1.39);
        const SUMOTime duration = attrs.getOptSUMOTimeReporting(SUMO_ATTR_DURATION, "", parsedOk, 0);
        if (parsedOk) {
            // set tag
            myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_WALK);
            // add all attributes
            myCommonXMLStructure.getCurrentSumoBaseObject()->setPlanParameters(planParameters);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_DEPARTPOS, departPos);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_ARRIVALPOS, arrivalPos);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_SPEED, speed);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_DURATION, duration);
        }
    }
}


void
RouteHandler::parseRide(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // plan parameters
    const auto planParameters = CommonXMLStructure::PlanParameters(myCommonXMLStructure.getCurrentSumoBaseObject(), attrs, parsedOk);
    // optional attributes
    const std::vector<std::string> lines = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_LINES, "", parsedOk);
    const double arrivalPos = attrs.getOpt<double>(SUMO_ATTR_ARRIVALPOS, "", parsedOk, -1);
    const std::string group = attrs.getOpt<std::string>(SUMO_ATTR_GROUP, "", parsedOk, "");
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_RIDE);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->setPlanParameters(planParameters);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_LINES, lines);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_ARRIVALPOS, arrivalPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_GROUP, group);
    }
}


void
RouteHandler::parseContainer(const SUMOSAXAttributes& attrs) {
    // first parse container
    SUMOVehicleParameter* containerParameter = SUMOVehicleParserHelper::parseVehicleAttributes(SUMO_TAG_CONTAINER, attrs, myHardFail);
    if (containerParameter) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_CONTAINER);
        // set vehicle parameter
        myCommonXMLStructure.getCurrentSumoBaseObject()->setVehicleParameter(containerParameter);
        // delete container parameter (because in XMLStructure we have a copy)
        delete containerParameter;
    }
}


void
RouteHandler::parseContainerFlow(const SUMOSAXAttributes& attrs) {
    // first parse flow
    SUMOVehicleParameter* containerFlowParameter = SUMOVehicleParserHelper::parseFlowAttributes(SUMO_TAG_CONTAINERFLOW, attrs, myHardFail, true, myFlowBeginDefault, myFlowEndDefault);
    if (containerFlowParameter) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_CONTAINERFLOW);
        // set vehicle parameter
        myCommonXMLStructure.getCurrentSumoBaseObject()->setVehicleParameter(containerFlowParameter);
        // delete container flow parameter (because in XMLStructure we have a copy)
        delete containerFlowParameter;
    }
}


void
RouteHandler::parseTransport(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // plan parameters
    const auto planParameters = CommonXMLStructure::PlanParameters(myCommonXMLStructure.getCurrentSumoBaseObject(), attrs, parsedOk);
    // optional attributes
    const std::vector<std::string> lines = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_LINES, "", parsedOk);
    const double arrivalPos = attrs.getOpt<double>(SUMO_ATTR_ARRIVALPOS, "", parsedOk, -1);
    const std::string group = attrs.getOpt<std::string>(SUMO_ATTR_GROUP, "", parsedOk, "");
    if (parsedOk) {
        // set tag
        myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_TRANSPORT);
        // add all attributes
        myCommonXMLStructure.getCurrentSumoBaseObject()->setPlanParameters(planParameters);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringListAttribute(SUMO_ATTR_LINES, lines);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_ARRIVALPOS, arrivalPos);
        myCommonXMLStructure.getCurrentSumoBaseObject()->addStringAttribute(SUMO_ATTR_GROUP, group);
    }
}


void
RouteHandler::parseTranship(const SUMOSAXAttributes& attrs) {
    if (attrs.hasAttribute(SUMO_ATTR_SPEED) && attrs.hasAttribute(SUMO_ATTR_DURATION)) {
        WRITE_ERROR(TL("Speed and duration attributes cannot be defined together in walks"));
    } else {
        // declare Ok Flag
        bool parsedOk = true;
        // plan parameters
        const auto planParameters = CommonXMLStructure::PlanParameters(myCommonXMLStructure.getCurrentSumoBaseObject(), attrs, parsedOk);
        // optional attributes
        const double arrivalPos = attrs.getOpt<double>(SUMO_ATTR_ARRIVALPOS, "", parsedOk, -1);
        const double departPos = attrs.getOpt<double>(SUMO_ATTR_DEPARTPOS, "", parsedOk, -1);
        const double speed = attrs.getOpt<double>(SUMO_ATTR_SPEED, "", parsedOk, 1.39);
        const SUMOTime duration = attrs.getOptSUMOTimeReporting(SUMO_ATTR_DURATION, "", parsedOk, 0);
        if (parsedOk) {
            // set tag
            myCommonXMLStructure.getCurrentSumoBaseObject()->setTag(SUMO_TAG_TRANSHIP);
            // add all attributes
            myCommonXMLStructure.getCurrentSumoBaseObject()->setPlanParameters(planParameters);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_ARRIVALPOS, arrivalPos);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_DEPARTPOS, departPos);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addDoubleAttribute(SUMO_ATTR_SPEED, speed);
            myCommonXMLStructure.getCurrentSumoBaseObject()->addTimeAttribute(SUMO_ATTR_DURATION, duration);
        }
    }
}


void
RouteHandler::parseInterval(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // just parse begin and end default
    myFlowBeginDefault = attrs.getSUMOTimeReporting(SUMO_ATTR_BEGIN, nullptr, parsedOk);
    myFlowEndDefault = attrs.getSUMOTimeReporting(SUMO_ATTR_END, nullptr, parsedOk);
}


void
RouteHandler::parseParameters(const SUMOSAXAttributes& attrs) {
    // declare Ok Flag
    bool parsedOk = true;
    // get key
    const std::string key = attrs.get<std::string>(SUMO_ATTR_KEY, nullptr, parsedOk);
    // get SumoBaseObject parent
    CommonXMLStructure::SumoBaseObject* SumoBaseObjectParent = myCommonXMLStructure.getCurrentSumoBaseObject()->getParentSumoBaseObject();
    // check parent
    if (SumoBaseObjectParent == nullptr) {
        writeError(TL("Parameters must be defined within an object"));
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
            WRITE_WARNINGF(TL("Error parsing key from % generic parameter. Key cannot be empty"), parentTagStr);
        } else if (!SUMOXMLDefinitions::isValidParameterKey(key)) {
            WRITE_WARNINGF(TL("Error parsing key from % generic parameter. Key contains invalid characters"), parentTagStr);
        } else {
            WRITE_DEBUG("Inserting generic parameter '" + key + "|" + value + "' into " + parentTagStr);
            // insert parameter in SumoBaseObjectParent
            SumoBaseObjectParent->addParameter(key, value);
        }
    }
}


bool
RouteHandler::parseNestedCFM(const SumoXMLTag tag, const SUMOSAXAttributes& attrs) {
    // get vehicle type Base object
    const auto vTypeObject = myCommonXMLStructure.getCurrentSumoBaseObject()->getParentSumoBaseObject();
    // parse embedded car following model information
    if (vTypeObject && (vTypeObject->getTag() == SUMO_TAG_VTYPE)) {
        WRITE_WARNINGF(TL("Defining car-following parameters in a nested element is deprecated in vType '%', use attributes instead!"), vTypeObject->getStringAttribute(SUMO_ATTR_ID));
        // get vType to modify it
        auto vType = vTypeObject->getVehicleTypeParameter();
        // parse nested CFM attributes
        if (SUMOVehicleParserHelper::parseCFMParams(&vType, tag, attrs, true)) {
            vTypeObject->setVehicleTypeParameter(&vType);
            return true;
        } else if (myHardFail) {
            throw ProcessError(TL("Invalid parsing embedded VType"));
        } else {
            writeError(TL("Invalid parsing embedded VType"));
        }
    }
    return false;
}


bool
RouteHandler::parseStopParameters(SUMOVehicleParameter::Stop& stop, const SUMOSAXAttributes& attrs) {
    // check stop parameters
    if (attrs.hasAttribute(SUMO_ATTR_ARRIVAL)) {
        stop.parametersSet |= STOP_ARRIVAL_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_DURATION)) {
        stop.parametersSet |= STOP_DURATION_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_UNTIL)) {
        stop.parametersSet |= STOP_UNTIL_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_STARTED)) {
        stop.parametersSet |= STOP_STARTED_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_ENDED)) {
        stop.parametersSet |= STOP_ENDED_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_EXTENSION)) {
        stop.parametersSet |= STOP_EXTENSION_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_ENDPOS)) {
        stop.parametersSet |= STOP_END_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_STARTPOS)) {
        stop.parametersSet |= STOP_START_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_POSITION_LAT)) {
        stop.parametersSet |= STOP_POSLAT_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_TRIGGERED)) {
        stop.parametersSet |= STOP_TRIGGER_SET;
    }
    // legacy attribute
    if (attrs.hasAttribute(SUMO_ATTR_CONTAINER_TRIGGERED)) {
        stop.parametersSet |= STOP_TRIGGER_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_PARKING)) {
        stop.parametersSet |= STOP_PARKING_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_EXPECTED)) {
        stop.parametersSet |= STOP_EXPECTED_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_PERMITTED)) {
        stop.parametersSet |= STOP_PERMITTED_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_EXPECTED_CONTAINERS)) {
        stop.parametersSet |= STOP_EXPECTED_CONTAINERS_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_TRIP_ID)) {
        stop.parametersSet |= STOP_TRIP_ID_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_SPLIT)) {
        stop.parametersSet |= STOP_SPLIT_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_JOIN)) {
        stop.parametersSet |= STOP_JOIN_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_LINE)) {
        stop.parametersSet |= STOP_LINE_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_SPEED)) {
        stop.parametersSet |= STOP_SPEED_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_ONDEMAND)) {
        stop.parametersSet |= STOP_ONDEMAND_SET;
    }
    if (attrs.hasAttribute(SUMO_ATTR_JUMP)) {
        stop.parametersSet |= STOP_JUMP_SET;
    }
    // get parameters
    bool ok = true;
    // edge/lane
    stop.edge = attrs.getOpt<std::string>(SUMO_ATTR_EDGE, nullptr, ok, "");
    stop.lane = attrs.getOpt<std::string>(SUMO_ATTR_LANE, nullptr, ok, stop.busstop);
    // check errors
    if (!stop.edge.empty() && !stop.lane.empty()) {
        writeError(TL("A stop must be defined either with an edge or with an lane, not both"));
        return false;
    }
    // stopping places
    stop.busstop = attrs.getOpt<std::string>(SUMO_ATTR_BUS_STOP, nullptr, ok, "");
    if (stop.busstop.empty()) {
        stop.busstop = attrs.getOpt<std::string>(SUMO_ATTR_TRAIN_STOP, nullptr, ok, stop.busstop);
    }
    stop.chargingStation = attrs.getOpt<std::string>(SUMO_ATTR_CHARGING_STATION, nullptr, ok, "");
    stop.overheadWireSegment = attrs.getOpt<std::string>(SUMO_ATTR_OVERHEAD_WIRE_SEGMENT, nullptr, ok, "");
    stop.containerstop = attrs.getOpt<std::string>(SUMO_ATTR_CONTAINER_STOP, nullptr, ok, "");
    stop.parkingarea = attrs.getOpt<std::string>(SUMO_ATTR_PARKING_AREA, nullptr, ok, "");
    //check stopping places
    const int numStoppingPlaces = !stop.busstop.empty() + !stop.chargingStation.empty() + !stop.overheadWireSegment.empty() +
                                  !stop.containerstop.empty() + !stop.parkingarea.empty();
    if (numStoppingPlaces > 1) {
        writeError(TL("A stop must be defined only in a StoppingPlace"));
        return false;
    } else if ((numStoppingPlaces == 0) && stop.edge.empty() && stop.lane.empty()) {
        writeError(TL("A stop must be defined in an edge, a lane, or in a StoppingPlace"));
        return false;
    }
    // declare error suffix
    std::string errorSuffix;
    if (stop.busstop != "") {
        errorSuffix = " at '" + stop.busstop + "'" + errorSuffix;
    } else if (stop.chargingStation != "") {
        errorSuffix = " at '" + stop.chargingStation + "'" + errorSuffix;
    } else if (stop.overheadWireSegment != "") {
        errorSuffix = " at '" + stop.overheadWireSegment + "'" + errorSuffix;
    } else if (stop.containerstop != "") {
        errorSuffix = " at '" + stop.containerstop + "'" + errorSuffix;
    } else if (stop.parkingarea != "") {
        errorSuffix = " at '" + stop.parkingarea + "'" + errorSuffix;
    } else if (stop.edge != "") {
        errorSuffix = " at '" + stop.edge + "'" + errorSuffix;
    } else {
        errorSuffix = " on lane '" + stop.lane + "'" + errorSuffix;
    }
    // speed for counting as stopped
    stop.speed = attrs.getOpt<double>(SUMO_ATTR_SPEED, nullptr, ok, 0);
    if (stop.speed < 0) {
        writeError("Speed cannot be negative for stop" + errorSuffix);
        return false;
    }
    // get the standing duration
    bool expectTrigger = !attrs.hasAttribute(SUMO_ATTR_DURATION) && !attrs.hasAttribute(SUMO_ATTR_UNTIL) && !attrs.hasAttribute(SUMO_ATTR_SPEED);
    std::vector<std::string> triggers = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_TRIGGERED, nullptr, ok);
    // legacy
    if (attrs.getOpt<bool>(SUMO_ATTR_CONTAINER_TRIGGERED, nullptr, ok, false)) {
        triggers.push_back(toString(SUMO_TAG_CONTAINER));
    };
    SUMOVehicleParameter::parseStopTriggers(triggers, expectTrigger, stop);
    stop.startPos = attrs.getOpt<double>(SUMO_ATTR_STARTPOS, nullptr, ok, 0);
    stop.endPos = attrs.getOpt<double>(SUMO_ATTR_ENDPOS, nullptr, ok, 0);
    stop.friendlyPos = attrs.getOpt<bool>(SUMO_ATTR_FRIENDLY_POS, nullptr, ok, false);
    stop.arrival = attrs.getOptSUMOTimeReporting(SUMO_ATTR_ARRIVAL, nullptr, ok, -1);
    stop.duration = attrs.getOptSUMOTimeReporting(SUMO_ATTR_DURATION, nullptr, ok, -1);
    stop.until = attrs.getOptSUMOTimeReporting(SUMO_ATTR_UNTIL, nullptr, ok, -1);
    if (!expectTrigger && (!ok || (stop.duration < 0 && stop.until < 0 && stop.speed == 0))) {
        writeError("Invalid duration or end time is given for a stop" + errorSuffix);
        return false;
    }
    stop.extension = attrs.getOptSUMOTimeReporting(SUMO_ATTR_EXTENSION, nullptr, ok, -1);
    const bool defaultParking = (stop.triggered || stop.containerTriggered || stop.parkingarea != "");
    stop.parking = attrs.getOpt<ParkingType>(SUMO_ATTR_PARKING, nullptr, ok, defaultParking ? ParkingType::OFFROAD : ParkingType::ONROAD);
    if ((stop.parkingarea != "") && (stop.parking == ParkingType::ONROAD)) {
        WRITE_WARNING("Stop at parkingarea overrides attribute 'parking' for stop" + errorSuffix);
        stop.parking = ParkingType::OFFROAD;
    }
    if (!ok) {
        writeError("Invalid bool for 'triggered', 'containerTriggered' or 'parking' for stop" + errorSuffix);
        return false;
    }
    // expected persons
    const std::vector<std::string>& expected = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_EXPECTED, nullptr, ok);
    stop.awaitedPersons.insert(expected.begin(), expected.end());
    if (stop.awaitedPersons.size() > 0 && (stop.parametersSet & STOP_TRIGGER_SET) == 0) {
        stop.triggered = true;
        if ((stop.parametersSet & STOP_PARKING_SET) == 0) {
            stop.parking = ParkingType::OFFROAD;
        }
    }
    // permitted transportables
    const std::vector<std::string>& permitted = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_PERMITTED, nullptr, ok);
    stop.permitted.insert(permitted.begin(), permitted.end());
    // expected containers
    const std::vector<std::string>& expectedContainers = attrs.getOpt<std::vector<std::string> >(SUMO_ATTR_EXPECTED_CONTAINERS, nullptr, ok);
    stop.awaitedContainers.insert(expectedContainers.begin(), expectedContainers.end());
    if (stop.awaitedContainers.size() > 0 && (stop.parametersSet & STOP_CONTAINER_TRIGGER_SET) == 0) {
        stop.containerTriggered = true;
        if ((stop.parametersSet & STOP_PARKING_SET) == 0) {
            stop.parking = ParkingType::OFFROAD;
        }
    }
    // public transport trip id
    stop.tripId = attrs.getOpt<std::string>(SUMO_ATTR_TRIP_ID, nullptr, ok, "");
    stop.split = attrs.getOpt<std::string>(SUMO_ATTR_SPLIT, nullptr, ok, "");
    stop.join = attrs.getOpt<std::string>(SUMO_ATTR_JOIN, nullptr, ok, "");
    stop.line = attrs.getOpt<std::string>(SUMO_ATTR_LINE, nullptr, ok, "");
    // index
    const std::string idx = attrs.getOpt<std::string>(SUMO_ATTR_INDEX, nullptr, ok, "end");
    if (idx == "end") {
        stop.index = STOP_INDEX_END;
    } else if (idx == "fit") {
        stop.index = STOP_INDEX_FIT;
    } else {
        stop.index = attrs.get<int>(SUMO_ATTR_INDEX, nullptr, ok);
        if (!ok || stop.index < 0) {
            writeError("Invalid 'index' for stop" + errorSuffix);
            return false;
        }
    }
    stop.started = attrs.getOptSUMOTimeReporting(SUMO_ATTR_STARTED, nullptr, ok, -1);
    stop.ended = attrs.getOptSUMOTimeReporting(SUMO_ATTR_ENDED, nullptr, ok, -1);
    stop.posLat = attrs.getOpt<double>(SUMO_ATTR_POSITION_LAT, nullptr, ok, INVALID_DOUBLE);
    stop.actType = attrs.getOpt<std::string>(SUMO_ATTR_ACTTYPE, nullptr, ok, "");
    stop.onDemand = attrs.getOpt<bool>(SUMO_ATTR_ONDEMAND, nullptr, ok, false);
    stop.jump = attrs.getOptSUMOTimeReporting(SUMO_ATTR_JUMP, nullptr, ok, -1);
    return true;
}


bool
RouteHandler::isEmbeddedRoute(const SUMOSAXAttributes& attrs) const {
    // check conditions
    if (attrs.hasAttribute(SUMO_ATTR_ID)) {
        return false;
    } else if (myCommonXMLStructure.getCurrentSumoBaseObject()->getParentSumoBaseObject() == nullptr) {
        return false;
    } else if (myCommonXMLStructure.getCurrentSumoBaseObject()->getParentSumoBaseObject()->hasStringAttribute(SUMO_ATTR_ROUTE)) {
        return false;
    } else if (myCommonXMLStructure.getCurrentSumoBaseObject()->getParentSumoBaseObject()->getTag() == SUMO_TAG_FLOW) {
        return true;
    } else if (myCommonXMLStructure.getCurrentSumoBaseObject()->getParentSumoBaseObject()->getTag() == SUMO_TAG_VEHICLE) {
        return true;
    } else {
        return false;
    }
}


void
RouteHandler::checkParent(const SumoXMLTag currentTag, const std::vector<SumoXMLTag>& parentTags, bool& ok) {
    // check that parent SUMOBaseObject's tag is the parentTag
    const CommonXMLStructure::SumoBaseObject* parent = myCommonXMLStructure.getCurrentSumoBaseObject()->getParentSumoBaseObject();
    // set parent string
    std::string parentStrings;
    for (const auto& tag : parentTags) {
        if (tag == parentTags.back()) {
            parentStrings.append(toString(tag));
        } else {
            parentStrings.append(toString(tag) + ", ");
        }
    }
    if ((parent != nullptr) &&
            (parentTags.size() > 0) &&
            (std::find(parentTags.begin(), parentTags.end(), parent->getTag()) == parentTags.end())) {
        const std::string id = parent->hasStringAttribute(SUMO_ATTR_ID) ? ", id: '" + parent->getStringAttribute(SUMO_ATTR_ID) + "'" : "";
        writeError("'" + toString(currentTag) + "' must be defined within the definition of a '" + parentStrings + "' (found '" + toString(parent->getTag()) + "'" + id + ").");
        ok = false;
    }
}

/****************************************************************************/
