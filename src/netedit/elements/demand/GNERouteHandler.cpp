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
/// @file    GNERouteHandler.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Jan 2019
///
// Builds demand objects for netedit
/****************************************************************************/
#include <netedit/GNENet.h>
#include <netedit/GNEUndoList.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNEViewParent.h>
#include <netedit/changes/GNEChange_DemandElement.h>
#include <netedit/frames/demand/GNEVehicleFrame.h>
#include <netedit/frames/common/GNEInspectorFrame.h>
#include <netedit/elements/additional/GNETAZ.h>
#include <utils/common/StringTokenizer.h>

#include "GNEContainer.h"
#include "GNEPerson.h"
#include "GNEPersonTrip.h"
#include "GNERide.h"
#include "GNERoute.h"
#include "GNERouteDistribution.h"
#include "GNERouteHandler.h"
#include "GNEStop.h"
#include "GNETranship.h"
#include "GNETransport.h"
#include "GNEVehicle.h"
#include "GNEVType.h"
#include "GNEVTypeDistribution.h"
#include "GNEWalk.h"
#include "GNEStopPlan.h"

// ===========================================================================
// member method definitions
// ===========================================================================

GNERouteHandler::GNERouteHandler(const std::string& file, GNENet* net, const bool allowUndoRedo, const bool overwrite) :
    RouteHandler(file, false),
    myNet(net),
    myPlanObject(new CommonXMLStructure::SumoBaseObject(nullptr)),
    myAllowUndoRedo(allowUndoRedo),
    myOverwrite(overwrite) {
}


GNERouteHandler::~GNERouteHandler() {
    delete myPlanObject;
}


void
GNERouteHandler::buildVType(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const SUMOVTypeParameter& vTypeParameter) {
    // check if loaded type is a default type
    if (DEFAULT_VTYPES.count(vTypeParameter.id) > 0) {
        // overwrite default vehicle type
        GNEVType::overwriteVType(myNet->getAttributeCarriers()->retrieveDemandElement(SUMO_TAG_VTYPE, vTypeParameter.id, false), vTypeParameter, myNet->getViewNet()->getUndoList());
    } else if (!checkDuplicatedDemandElement(SUMO_TAG_VTYPE, vTypeParameter.id)) {
        writeError(TLF("There is another % with the same ID='%'.", toString(SUMO_TAG_VTYPE), vTypeParameter.id));
    } else {
        // create vType/pType using myCurrentVType
        GNEDemandElement* vType = new GNEVType(myNet, vTypeParameter);
        // check if add this vType to a distribution
        GNEDemandElement* vTypeDistribution = nullptr;
        if (sumoBaseObject->getParentSumoBaseObject() && sumoBaseObject->getParentSumoBaseObject()->getTag() == SUMO_TAG_VTYPE_DISTRIBUTION) {
            vTypeDistribution = myNet->getAttributeCarriers()->retrieveDemandElement(SUMO_TAG_VTYPE_DISTRIBUTION, sumoBaseObject->getParentSumoBaseObject()->getStringAttribute(SUMO_ATTR_ID), false);
        }
        if (myAllowUndoRedo) {
            myNet->getViewNet()->getUndoList()->begin(vType, TL("add ") + vType->getTagStr() + " '" + vTypeParameter.id + "'");
            overwriteDemandElement();
            myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(vType, true), true);
            if (vTypeDistribution) {
                vTypeDistribution->addDistributionKey(vType, vType->getAttributeDouble(SUMO_ATTR_PROB), myNet->getViewNet()->getUndoList());
            }
            myNet->getViewNet()->getUndoList()->end();
        } else {
            myNet->getAttributeCarriers()->insertDemandElement(vType);
            if (vTypeDistribution) {
                vTypeDistribution->addDistributionKey(vType, vType->getAttributeDouble(SUMO_ATTR_PROB));
            }
            vType->incRef("buildVType");
        }
    }
}


void
GNERouteHandler::buildVTypeDistribution(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const int deterministic,
                                        const std::vector<std::string>& vTypeIDs, const std::vector<double>& probabilities) {
    // declare vector with vType and their probabilities
    std::vector<const GNEDemandElement*> vTypes;
    // first check conditions
    if (!checkDuplicatedDemandElement(SUMO_TAG_VTYPE_DISTRIBUTION, id)) {
        writeError(TLF("There is another % with the same ID='%'.", toString(SUMO_TAG_VTYPE)));
    } else if (getDistributionElements(sumoBaseObject, SUMO_TAG_VTYPE, vTypeIDs, probabilities, vTypes)) {
        // create distributions
        GNEVTypeDistribution* vTypeDistribution = new GNEVTypeDistribution(myNet, id, deterministic);
        if (myAllowUndoRedo) {
            myNet->getViewNet()->getUndoList()->begin(vTypeDistribution, TL("add ") + vTypeDistribution->getTagStr() + " '" + id + "'");
            overwriteDemandElement();
            myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(vTypeDistribution, true), true);
            // add all distributions
            for (int i = 0; i < (int)vTypes.size(); i++) {
                vTypeDistribution->addDistributionKey(vTypes.at(i), probabilities.at(i), myNet->getViewNet()->getUndoList());
            }
            myNet->getViewNet()->getUndoList()->end();
        } else {
            myNet->getAttributeCarriers()->insertDemandElement(vTypeDistribution);
            // add all distributions directly
            for (int i = 0; i < (int)vTypes.size(); i++) {
                vTypeDistribution->addDistributionKey(vTypes.at(i), probabilities.at(i));
            }
            vTypeDistribution->incRef("buildVTypeDistribution");
        }
    }
}


void
GNERouteHandler::buildRoute(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, SUMOVehicleClass vClass,
                            const std::vector<std::string>& edgeIDs, const RGBColor& color, const int repeat, const SUMOTime cycleTime,
                            const double probability, const Parameterised::Map& routeParameters) {
    // parse edges
    const auto edges = parseEdges(SUMO_TAG_ROUTE, edgeIDs);
    // check conditions
    if (!checkDuplicatedDemandElement(SUMO_TAG_ROUTE, id)) {
        writeError(TLF("There is another % with the same ID='%'.", toString(SUMO_TAG_ROUTE), id));
    } else if (edges.size() > 0) {
        // create GNERoute
        GNEDemandElement* route = new GNERoute(myNet, id, vClass, edges, color, repeat, cycleTime, routeParameters);
        // check if add this route to a distribution
        GNEDemandElement* routeDistribution = nullptr;
        if (sumoBaseObject && sumoBaseObject->getParentSumoBaseObject() && sumoBaseObject->getParentSumoBaseObject()->getTag() == SUMO_TAG_ROUTE_DISTRIBUTION) {
            routeDistribution = myNet->getAttributeCarriers()->retrieveDemandElement(SUMO_TAG_ROUTE_DISTRIBUTION, sumoBaseObject->getParentSumoBaseObject()->getStringAttribute(SUMO_ATTR_ID), false);
        }
        if (myAllowUndoRedo) {
            myNet->getViewNet()->getUndoList()->begin(route, TL("add ") + route->getTagStr() + " '" + id + "'");
            overwriteDemandElement();
            myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(route, true), true);
            if (routeDistribution) {
                routeDistribution->addDistributionKey(route, probability, myNet->getViewNet()->getUndoList());
            }
            myNet->getViewNet()->getUndoList()->end();
        } else {
            myNet->getAttributeCarriers()->insertDemandElement(route);
            for (const auto& edge : edges) {
                edge->addChildElement(route);
            }
            if (routeDistribution) {
                routeDistribution->addDistributionKey(route, probability);
            }
            route->incRef("buildRoute");
        }
    }
}


void
GNERouteHandler::buildEmbeddedRoute(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::vector<std::string>& edgeIDs,
                                    const RGBColor& color, const int repeat, const SUMOTime cycleTime, const Parameterised::Map& routeParameters) {
    // first create vehicle/flow
    const SUMOVehicleParameter& vehicleParameters = sumoBaseObject->getParentSumoBaseObject()->getVehicleParameter();
    const SumoXMLTag vehicleTag = (sumoBaseObject->getParentSumoBaseObject()->getTag() == SUMO_TAG_VEHICLE) ? GNE_TAG_VEHICLE_WITHROUTE :
                                  (sumoBaseObject->getParentSumoBaseObject()->getTag() == SUMO_TAG_FLOW) ? GNE_TAG_FLOW_WITHROUTE :
                                  sumoBaseObject->getParentSumoBaseObject()->getTag();
    // parse route edges
    const auto edges = parseEdges(SUMO_TAG_ROUTE, edgeIDs);
    // check if ID is duplicated
    if ((edges.size() > 0) && !isVehicleIdDuplicated(vehicleParameters.id)) {
        // obtain  type
        GNEDemandElement* type = getType(vehicleParameters.vtypeid);
        if (type == nullptr) {
            writeError(TLF("Invalid vehicle type '%' used in % '%'.", vehicleParameters.vtypeid, toString(vehicleParameters.tag), vehicleParameters.id));
        } else if (vehicleParameters.wasSet(VEHPARS_DEPARTLANE_SET) && (vehicleParameters.departLaneProcedure == DepartLaneDefinition::GIVEN) && ((int)edges.front()->getLanes().size() < vehicleParameters.departLane)) {
            writeError(TLF("Invalid % used in % '%'. % is greater than number of lanes", toString(SUMO_ATTR_DEPARTLANE), toString(vehicleParameters.tag), vehicleParameters.id, toString(vehicleParameters.departLane)));
        } else if (vehicleParameters.wasSet(VEHPARS_DEPARTSPEED_SET) && (vehicleParameters.departSpeedProcedure == DepartSpeedDefinition::GIVEN) && (type->getAttributeDouble(SUMO_ATTR_MAXSPEED) < vehicleParameters.departSpeed)) {
            writeError(TLF("Invalid % used in % '%'. % is greater than type %", toString(SUMO_ATTR_DEPARTSPEED), toString(vehicleParameters.tag), vehicleParameters.id, toString(vehicleParameters.departSpeed), toString(SUMO_ATTR_MAXSPEED)));
        } else {
            // create vehicle using vehicleParameters
            GNEDemandElement* vehicle = new GNEVehicle(vehicleTag, myNet, type, vehicleParameters);
            // create embedded route
            GNEDemandElement* route = new GNERoute(myNet, vehicle, edges, color, repeat, cycleTime, routeParameters);
            if (myAllowUndoRedo) {
                myNet->getViewNet()->getUndoList()->begin(route, TL("add ") + route->getTagStr() + " in '" + vehicle->getID() + "'");
                overwriteDemandElement();
                myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(vehicle, true), true);
                myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(route, true), true);
                myNet->getViewNet()->getUndoList()->end();
            } else {
                myNet->getAttributeCarriers()->insertDemandElement(vehicle);
                myNet->getAttributeCarriers()->insertDemandElement(route);
                type->addChildElement(vehicle);
                vehicle->addChildElement(route);
                for (const auto& edge : edges) {
                    edge->addChildElement(route);
                }
                vehicle->incRef("buildEmbeddedRoute");
                route->incRef("buildEmbeddedRoute");
            }
        }
    }
}


void
GNERouteHandler::buildRouteDistribution(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id,
                                        const std::vector<std::string>& routeIDs, const std::vector<double>& probabilities) {
    // declare vector with route and their probabilities
    std::vector<const GNEDemandElement*> routes;
    // first check conditions
    if (!checkDuplicatedDemandElement(SUMO_TAG_ROUTE_DISTRIBUTION, id)) {
        writeError(TLF("There is another % with the same ID='%'.", toString(SUMO_TAG_ROUTE)));
    } else if (getDistributionElements(sumoBaseObject, SUMO_TAG_ROUTE, routeIDs, probabilities, routes)) {
        // create distributions
        GNERouteDistribution* routeDistribution = new GNERouteDistribution(myNet, id);
        if (myAllowUndoRedo) {
            myNet->getViewNet()->getUndoList()->begin(routeDistribution, TL("add ") + routeDistribution->getTagStr() + " '" + id + "'");
            overwriteDemandElement();
            myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(routeDistribution, true), true);
            // add all distributions
            for (int i = 0; i < (int)routes.size(); i++) {
                routeDistribution->addDistributionKey(routes.at(i), probabilities.at(i), myNet->getViewNet()->getUndoList());
            }
            myNet->getViewNet()->getUndoList()->end();
        } else {
            myNet->getAttributeCarriers()->insertDemandElement(routeDistribution);
            // add all distributions directly
            for (int i = 0; i < (int)routes.size(); i++) {
                routeDistribution->addDistributionKey(routes.at(i), probabilities.at(i));
            }
            routeDistribution->incRef("buildRouteDistribution");
        }
    }
}


void
GNERouteHandler::buildVehicleOverRoute(const CommonXMLStructure::SumoBaseObject* /*sumoBaseObject*/, const SUMOVehicleParameter& vehicleParameters) {
    // first check if ID is duplicated
    if (!isVehicleIdDuplicated(vehicleParameters.id)) {
        // obtain routes and vtypes
        GNEDemandElement* type = getType(vehicleParameters.vtypeid);
        GNEDemandElement* route = myNet->getAttributeCarriers()->retrieveDemandElement(SUMO_TAG_ROUTE, vehicleParameters.routeid, false);
        if (type == nullptr) {
            writeError(TLF("Invalid vehicle type '%' used in % '%'.", vehicleParameters.vtypeid, toString(vehicleParameters.tag), vehicleParameters.id));
        } else if (route == nullptr) {
            writeError(TLF("Invalid route '%' used in % '%'.", vehicleParameters.routeid, toString(vehicleParameters.tag), vehicleParameters.id));
        } else if (vehicleParameters.wasSet(VEHPARS_DEPARTLANE_SET) && (vehicleParameters.departLaneProcedure == DepartLaneDefinition::GIVEN) && ((int)route->getParentEdges().front()->getLanes().size() < vehicleParameters.departLane)) {
            writeError(TLF("Invalid % used in % '%'. % is greater than number of lanes", toString(SUMO_ATTR_DEPARTLANE), toString(vehicleParameters.tag), vehicleParameters.id, toString(vehicleParameters.departLane)));
        } else if (vehicleParameters.wasSet(VEHPARS_DEPARTSPEED_SET) && (vehicleParameters.departSpeedProcedure == DepartSpeedDefinition::GIVEN) && (type->getAttributeDouble(SUMO_ATTR_MAXSPEED) < vehicleParameters.departSpeed)) {
            writeError(TLF("Invalid % used in % '%'. % is greater than type %", toString(SUMO_ATTR_DEPARTSPEED), toString(vehicleParameters.tag), vehicleParameters.id, toString(vehicleParameters.departSpeed), toString(SUMO_ATTR_MAXSPEED)));
        } else {
            // create vehicle using vehicleParameters
            GNEDemandElement* vehicle = new GNEVehicle(SUMO_TAG_VEHICLE, myNet, type, route, vehicleParameters);
            if (myAllowUndoRedo) {
                myNet->getViewNet()->getUndoList()->begin(vehicle, TL("add ") + vehicle->getTagStr() + " '" + vehicleParameters.id + "'");
                overwriteDemandElement();
                myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(vehicle, true), true);
                myNet->getViewNet()->getUndoList()->end();
            } else {
                myNet->getAttributeCarriers()->insertDemandElement(vehicle);
                // set vehicle as child of type and Route
                type->addChildElement(vehicle);
                route->addChildElement(vehicle);
                vehicle->incRef("buildVehicleOverRoute");
            }
        }
    }
}


void
GNERouteHandler::buildFlowOverRoute(const CommonXMLStructure::SumoBaseObject* /*sumoBaseObject*/, const SUMOVehicleParameter& vehicleParameters) {
    // first check if ID is duplicated
    if (!isVehicleIdDuplicated(vehicleParameters.id)) {
        // obtain routes and vtypes
        GNEDemandElement* type = getType(vehicleParameters.vtypeid);
        GNEDemandElement* route = myNet->getAttributeCarriers()->retrieveDemandElement(SUMO_TAG_ROUTE, vehicleParameters.routeid, false);
        if (type == nullptr) {
            writeError(TLF("Invalid vehicle type '%' used in % '%'.", vehicleParameters.vtypeid, toString(vehicleParameters.tag), vehicleParameters.id));
        } else if (route == nullptr) {
            writeError(TLF("Invalid route '%' used in % '%'.", vehicleParameters.routeid, toString(vehicleParameters.tag), vehicleParameters.id));
        } else if (vehicleParameters.wasSet(VEHPARS_DEPARTLANE_SET) && (vehicleParameters.departLaneProcedure == DepartLaneDefinition::GIVEN) && ((int)route->getParentEdges().front()->getLanes().size() < vehicleParameters.departLane)) {
            writeError(TLF("Invalid % used in % '%'. % is greater than number of lanes", toString(SUMO_ATTR_DEPARTLANE), toString(vehicleParameters.tag), vehicleParameters.id, toString(vehicleParameters.departLane)));
        } else if (vehicleParameters.wasSet(VEHPARS_DEPARTSPEED_SET) && (vehicleParameters.departSpeedProcedure == DepartSpeedDefinition::GIVEN) && (type->getAttributeDouble(SUMO_ATTR_MAXSPEED) < vehicleParameters.departSpeed)) {
            writeError(TLF("Invalid % used in % '%'. % is greater than type %", toString(SUMO_ATTR_DEPARTSPEED), toString(vehicleParameters.tag), vehicleParameters.id, toString(vehicleParameters.departSpeed), toString(SUMO_ATTR_MAXSPEED)));
        } else {
            // create flow or trips using vehicleParameters
            GNEDemandElement* flow = new GNEVehicle(GNE_TAG_FLOW_ROUTE, myNet, type, route, vehicleParameters);
            if (myAllowUndoRedo) {
                myNet->getViewNet()->getUndoList()->begin(flow, TL("add ") + flow->getTagStr() + " '" + vehicleParameters.id + "'");
                overwriteDemandElement();
                myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(flow, true), true);
                myNet->getViewNet()->getUndoList()->end();
            } else {
                myNet->getAttributeCarriers()->insertDemandElement(flow);
                // set flow as child of type and Route
                type->addChildElement(flow);
                route->addChildElement(flow);
                flow->incRef("buildFlowOverRoute");
            }
        }
    }
}


void
GNERouteHandler::buildTrip(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const SUMOVehicleParameter& vehicleParameters,
                           const std::string& fromEdgeID, const std::string& toEdgeID) {
    // parse edges
    const auto fromEdge = parseEdge(SUMO_TAG_TRIP, fromEdgeID, sumoBaseObject, true);
    const auto toEdge = parseEdge(SUMO_TAG_TRIP, toEdgeID, sumoBaseObject, false);
    // set via attribute
    if (sumoBaseObject && sumoBaseObject->hasStringListAttribute(SUMO_ATTR_VIA)) {
        vehicleParameters.via = sumoBaseObject->getStringListAttribute(SUMO_ATTR_VIA);
    }
    // check if exist another vehicle with the same ID (note: Vehicles, Flows and Trips share namespace)
    if (fromEdge && toEdge && !isVehicleIdDuplicated(vehicleParameters.id) && isViaAttributeValid(vehicleParameters.via)) {
        // obtain  type
        GNEDemandElement* type = getType(vehicleParameters.vtypeid);
        if (type == nullptr) {
            writeError(TLF("Invalid vehicle type '%' used in % '%'.", vehicleParameters.vtypeid, toString(vehicleParameters.tag), vehicleParameters.id));
        } else if (vehicleParameters.wasSet(VEHPARS_DEPARTLANE_SET) && ((vehicleParameters.departLaneProcedure == DepartLaneDefinition::GIVEN)) && ((int)fromEdge->getLanes().size() < vehicleParameters.departLane)) {
            writeError(TLF("Invalid % used in % '%'. % is greater than number of lanes", toString(SUMO_ATTR_DEPARTLANE), toString(vehicleParameters.tag), vehicleParameters.id, toString(vehicleParameters.departLane)));
        } else if (vehicleParameters.wasSet(VEHPARS_DEPARTSPEED_SET) && (vehicleParameters.departSpeedProcedure == DepartSpeedDefinition::GIVEN) && (type->getAttributeDouble(SUMO_ATTR_MAXSPEED) < vehicleParameters.departSpeed)) {
            writeError(TLF("Invalid % used in % '%'. % is greater than type %", toString(SUMO_ATTR_DEPARTSPEED), toString(vehicleParameters.tag), vehicleParameters.id, toString(vehicleParameters.departSpeed), toString(SUMO_ATTR_MAXSPEED)));
        } else {
            // create trip or flow using tripParameters
            GNEDemandElement* trip = new GNEVehicle(SUMO_TAG_TRIP, myNet, type, fromEdge, toEdge, vehicleParameters);
            if (myAllowUndoRedo) {
                myNet->getViewNet()->getUndoList()->begin(trip, TL("add ") + trip->getTagStr() + " '" + vehicleParameters.id + "'");
                overwriteDemandElement();
                myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(trip, true), true);
                myNet->getViewNet()->getUndoList()->end();
            } else {
                myNet->getAttributeCarriers()->insertDemandElement(trip);
                // set vehicle as child of type
                type->addChildElement(trip);
                trip->incRef("buildTrip");
                // add reference in all edges
                fromEdge->addChildElement(trip);
                toEdge->addChildElement(trip);
            }
        }
    }
}


void
GNERouteHandler::buildTripJunctions(const CommonXMLStructure::SumoBaseObject* /*sumoBaseObject*/, const SUMOVehicleParameter& vehicleParameters,
                                    const std::string& fromJunctionID, const std::string& toJunctionID) {
    // parse junctions
    const auto fromJunction = parseJunction(SUMO_TAG_TRIP, fromJunctionID);
    const auto toJunction = parseJunction(SUMO_TAG_TRIP, toJunctionID);
    // check if exist another vehicle with the same ID (note: Vehicles, Flows and Trips share namespace)
    if (fromJunction && toJunction && !isVehicleIdDuplicated(vehicleParameters.id)) {
        // obtain  type
        GNEDemandElement* type = getType(vehicleParameters.vtypeid);
        if (type == nullptr) {
            writeError(TLF("Invalid vehicle type '%' used in % '%'.", vehicleParameters.vtypeid, toString(vehicleParameters.tag), vehicleParameters.id));
        } else if (vehicleParameters.wasSet(VEHPARS_DEPARTLANE_SET) && ((vehicleParameters.departLaneProcedure == DepartLaneDefinition::GIVEN)) && (vehicleParameters.departLane > 0)) {
            writeError(TLF("Invalid % used in % '%'. % is greater than number of lanes", toString(SUMO_ATTR_DEPARTLANE), toString(vehicleParameters.tag), vehicleParameters.id, toString(vehicleParameters.departLane)));
        } else if (vehicleParameters.wasSet(VEHPARS_DEPARTSPEED_SET) && (vehicleParameters.departSpeedProcedure == DepartSpeedDefinition::GIVEN) && (type->getAttributeDouble(SUMO_ATTR_MAXSPEED) < vehicleParameters.departSpeed)) {
            writeError(TLF("Invalid % used in % '%'. % is greater than type %", toString(SUMO_ATTR_DEPARTSPEED), toString(vehicleParameters.tag), vehicleParameters.id, toString(vehicleParameters.departSpeed), toString(SUMO_ATTR_MAXSPEED)));
        } else {
            // create trip using vehicleParameters
            GNEDemandElement* flow = new GNEVehicle(GNE_TAG_TRIP_JUNCTIONS, myNet, type, fromJunction, toJunction, vehicleParameters);
            if (myAllowUndoRedo) {
                myNet->getViewNet()->getUndoList()->begin(flow, TL("add ") + flow->getTagStr() + " '" + vehicleParameters.id + "'");
                overwriteDemandElement();
                myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(flow, true), true);
                myNet->getViewNet()->getUndoList()->end();
            } else {
                myNet->getAttributeCarriers()->insertDemandElement(flow);
                // set vehicle as child of type
                type->addChildElement(flow);
                flow->incRef("buildFlow");
                // add reference in all junctions
                fromJunction->addChildElement(flow);
                toJunction->addChildElement(flow);
            }
        }
    }
}


void
GNERouteHandler::buildTripTAZs(const CommonXMLStructure::SumoBaseObject* /*sumoBaseObject*/, const SUMOVehicleParameter& vehicleParameters,
                               const std::string& fromTAZID, const std::string& toTAZID) {
    // parse TAZs
    const auto fromTAZ = parseTAZ(SUMO_TAG_TRIP, fromTAZID);
    const auto toTAZ = parseTAZ(SUMO_TAG_TRIP, toTAZID);
    // check if exist another vehicle with the same ID (note: Vehicles, Flows and Trips share namespace)
    if (fromTAZ && toTAZ && !isVehicleIdDuplicated(vehicleParameters.id)) {
        // obtain  type
        GNEDemandElement* type = getType(vehicleParameters.vtypeid);
        if (type == nullptr) {
            writeError(TLF("Invalid vehicle type '%' used in % '%'.", vehicleParameters.vtypeid, toString(vehicleParameters.tag), vehicleParameters.id));
        } else if (vehicleParameters.wasSet(VEHPARS_DEPARTLANE_SET) && ((vehicleParameters.departLaneProcedure == DepartLaneDefinition::GIVEN)) && (vehicleParameters.departLane > 0)) {
            writeError(TLF("Invalid % used in % '%'. % is greater than number of lanes", toString(SUMO_ATTR_DEPARTLANE), toString(vehicleParameters.tag), vehicleParameters.id, toString(vehicleParameters.departLane)));
        } else if (vehicleParameters.wasSet(VEHPARS_DEPARTSPEED_SET) && (vehicleParameters.departSpeedProcedure == DepartSpeedDefinition::GIVEN) && (type->getAttributeDouble(SUMO_ATTR_MAXSPEED) < vehicleParameters.departSpeed)) {
            writeError(TLF("Invalid % used in % '%'. % is greater than type %", toString(SUMO_ATTR_DEPARTSPEED), toString(vehicleParameters.tag), vehicleParameters.id, toString(vehicleParameters.departSpeed), toString(SUMO_ATTR_MAXSPEED)));
        } else {
            // create trip using vehicleParameters
            GNEDemandElement* flow = new GNEVehicle(GNE_TAG_TRIP_TAZS, myNet, type, fromTAZ, toTAZ, vehicleParameters);
            if (myAllowUndoRedo) {
                myNet->getViewNet()->getUndoList()->begin(flow, TL("add ") + flow->getTagStr() + " '" + vehicleParameters.id + "'");
                overwriteDemandElement();
                myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(flow, true), true);
                myNet->getViewNet()->getUndoList()->end();
            } else {
                myNet->getAttributeCarriers()->insertDemandElement(flow);
                // set vehicle as child of type
                type->addChildElement(flow);
                flow->incRef("buildFlow");
                // add reference in all TAZs
                fromTAZ->addChildElement(flow);
                toTAZ->addChildElement(flow);
            }
        }
    }
}


void
GNERouteHandler::buildFlow(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const SUMOVehicleParameter& vehicleParameters,
                           const std::string& fromEdgeID, const std::string& toEdgeID) {
    // parse edges
    const auto fromEdge = parseEdge(SUMO_TAG_FLOW, fromEdgeID, sumoBaseObject, true);
    const auto toEdge = parseEdge(SUMO_TAG_FLOW, toEdgeID, sumoBaseObject, false);
    // set via attribute
    if (sumoBaseObject && sumoBaseObject->hasStringListAttribute(SUMO_ATTR_VIA)) {
        vehicleParameters.via = sumoBaseObject->getStringListAttribute(SUMO_ATTR_VIA);
    }
    // check if exist another vehicle with the same ID (note: Vehicles, Flows and Trips share namespace)
    if (fromEdge && toEdge && !isVehicleIdDuplicated(vehicleParameters.id) && isViaAttributeValid(vehicleParameters.via)) {
        // obtain  type
        GNEDemandElement* type = getType(vehicleParameters.vtypeid);
        if (type == nullptr) {
            writeError(TLF("Invalid vehicle type '%' used in % '%'.", vehicleParameters.vtypeid, toString(vehicleParameters.tag), vehicleParameters.id));
        } else if (vehicleParameters.wasSet(VEHPARS_DEPARTLANE_SET) && (vehicleParameters.departLaneProcedure == DepartLaneDefinition::GIVEN) && ((int)fromEdge->getLanes().size() < vehicleParameters.departLane)) {
            writeError(TLF("Invalid % used in % '%'. % is greater than number of lanes", toString(SUMO_ATTR_DEPARTLANE), toString(vehicleParameters.tag), vehicleParameters.id, toString(vehicleParameters.departLane)));
        } else if (vehicleParameters.wasSet(VEHPARS_DEPARTSPEED_SET) && (vehicleParameters.departSpeedProcedure == DepartSpeedDefinition::GIVEN) && (type->getAttributeDouble(SUMO_ATTR_MAXSPEED) < vehicleParameters.departSpeed)) {
            writeError(TLF("Invalid % used in % '%'. % is greater than type %", toString(SUMO_ATTR_DEPARTSPEED), toString(vehicleParameters.tag), vehicleParameters.id, toString(vehicleParameters.departSpeed), toString(SUMO_ATTR_MAXSPEED)));
        } else {
            // create trip or flow using tripParameters
            GNEDemandElement* flow = new GNEVehicle(SUMO_TAG_FLOW, myNet, type, fromEdge, toEdge, vehicleParameters);
            if (myAllowUndoRedo) {
                myNet->getViewNet()->getUndoList()->begin(flow, TL("add ") + flow->getTagStr() + " '" + vehicleParameters.id + "'");
                overwriteDemandElement();
                myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(flow, true), true);
                myNet->getViewNet()->getUndoList()->end();
            } else {
                myNet->getAttributeCarriers()->insertDemandElement(flow);
                // set vehicle as child of type
                type->addChildElement(flow);
                flow->incRef("buildFlow");
                // add reference in all edges
                fromEdge->addChildElement(flow);
                toEdge->addChildElement(flow);
            }
        }
    }
}


void
GNERouteHandler::buildFlowJunctions(const CommonXMLStructure::SumoBaseObject* /*sumoBaseObject*/, const SUMOVehicleParameter& vehicleParameters,
                                    const std::string& fromJunctionID, const std::string& toJunctionID) {
    // parse junctions
    const auto fromJunction = parseJunction(SUMO_TAG_TRIP, fromJunctionID);
    const auto toJunction = parseJunction(SUMO_TAG_TRIP, toJunctionID);
    // check if exist another vehicle with the same ID (note: Vehicles, Flows and Trips share namespace)
    if (fromJunction && toJunction && !isVehicleIdDuplicated(vehicleParameters.id)) {
        // obtain  type
        GNEDemandElement* type = getType(vehicleParameters.vtypeid);
        if (type == nullptr) {
            writeError(TLF("Invalid vehicle type '%' used in % '%'.", vehicleParameters.vtypeid, toString(vehicleParameters.tag), vehicleParameters.id));
        } else if (vehicleParameters.wasSet(VEHPARS_DEPARTLANE_SET) && ((vehicleParameters.departLaneProcedure == DepartLaneDefinition::GIVEN)) && (vehicleParameters.departLane > 0)) {
            writeError(TLF("Invalid % used in % '%'. % is greater than number of lanes", toString(SUMO_ATTR_DEPARTLANE), toString(vehicleParameters.tag), vehicleParameters.id, toString(vehicleParameters.departLane)));
        } else if (vehicleParameters.wasSet(VEHPARS_DEPARTSPEED_SET) && (vehicleParameters.departSpeedProcedure == DepartSpeedDefinition::GIVEN) && (type->getAttributeDouble(SUMO_ATTR_MAXSPEED) < vehicleParameters.departSpeed)) {
            writeError(TLF("Invalid % used in % '%'. % is greater than type %", toString(SUMO_ATTR_DEPARTSPEED), toString(vehicleParameters.tag), vehicleParameters.id, toString(vehicleParameters.departSpeed), toString(SUMO_ATTR_MAXSPEED)));
        } else {
            // create flow using vehicleParameters
            GNEDemandElement* flow = new GNEVehicle(GNE_TAG_FLOW_JUNCTIONS, myNet, type, fromJunction, toJunction, vehicleParameters);
            if (myAllowUndoRedo) {
                myNet->getViewNet()->getUndoList()->begin(flow, TL("add ") + flow->getTagStr() + " '" + vehicleParameters.id + "'");
                overwriteDemandElement();
                myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(flow, true), true);
                myNet->getViewNet()->getUndoList()->end();
            } else {
                myNet->getAttributeCarriers()->insertDemandElement(flow);
                // set vehicle as child of type
                type->addChildElement(flow);
                flow->incRef("buildFlow");
                // add reference in all junctions
                fromJunction->addChildElement(flow);
                toJunction->addChildElement(flow);
            }
        }
    }
}


void
GNERouteHandler::buildFlowTAZs(const CommonXMLStructure::SumoBaseObject* /*sumoBaseObject*/, const SUMOVehicleParameter& vehicleParameters,
                               const std::string& fromTAZID, const std::string& toTAZID) {
    // parse TAZs
    const auto fromTAZ = parseTAZ(SUMO_TAG_TRIP, fromTAZID);
    const auto toTAZ = parseTAZ(SUMO_TAG_TRIP, toTAZID);
    // check if exist another vehicle with the same ID (note: Vehicles, Flows and Trips share namespace)
    if (fromTAZ && toTAZ && !isVehicleIdDuplicated(vehicleParameters.id)) {
        // obtain  type
        GNEDemandElement* type = getType(vehicleParameters.vtypeid);
        if (type == nullptr) {
            writeError(TLF("Invalid vehicle type '%' used in % '%'.", vehicleParameters.vtypeid, toString(vehicleParameters.tag), vehicleParameters.id));
        } else if (vehicleParameters.wasSet(VEHPARS_DEPARTLANE_SET) && ((vehicleParameters.departLaneProcedure == DepartLaneDefinition::GIVEN)) && (vehicleParameters.departLane > 0)) {
            writeError(TLF("Invalid % used in % '%'. % is greater than number of lanes", toString(SUMO_ATTR_DEPARTLANE), toString(vehicleParameters.tag), vehicleParameters.id, toString(vehicleParameters.departLane)));
        } else if (vehicleParameters.wasSet(VEHPARS_DEPARTSPEED_SET) && (vehicleParameters.departSpeedProcedure == DepartSpeedDefinition::GIVEN) && (type->getAttributeDouble(SUMO_ATTR_MAXSPEED) < vehicleParameters.departSpeed)) {
            writeError(TLF("Invalid % used in % '%'. % is greater than type %", toString(SUMO_ATTR_DEPARTSPEED), toString(vehicleParameters.tag), vehicleParameters.id, toString(vehicleParameters.departSpeed), toString(SUMO_ATTR_MAXSPEED)));
        } else {
            // create flow using vehicleParameters
            GNEDemandElement* flow = new GNEVehicle(GNE_TAG_FLOW_TAZS, myNet, type, fromTAZ, toTAZ, vehicleParameters);
            if (myAllowUndoRedo) {
                myNet->getViewNet()->getUndoList()->begin(flow, TL("add ") + flow->getTagStr() + " '" + vehicleParameters.id + "'");
                overwriteDemandElement();
                myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(flow, true), true);
                myNet->getViewNet()->getUndoList()->end();
            } else {
                myNet->getAttributeCarriers()->insertDemandElement(flow);
                // set vehicle as child of type
                type->addChildElement(flow);
                flow->incRef("buildFlow");
                // add reference in all TAZs
                fromTAZ->addChildElement(flow);
                toTAZ->addChildElement(flow);
            }
        }
    }
}


void
GNERouteHandler::buildPerson(const CommonXMLStructure::SumoBaseObject* /*sumoBaseObject*/, const SUMOVehicleParameter& personParameters) {
    // first check if ID is duplicated
    if (!isPersonIdDuplicated(personParameters.id)) {
        // obtain  type
        GNEDemandElement* type = getType(personParameters.vtypeid);
        if (type == nullptr) {
            writeError(TLF("Invalid vehicle type '%' used in % '%'.", personParameters.vtypeid, toString(personParameters.tag), personParameters.id));
        } else {
            // create person using personParameters
            GNEDemandElement* person = new GNEPerson(SUMO_TAG_PERSON, myNet, type, personParameters);
            if (myAllowUndoRedo) {
                myNet->getViewNet()->getUndoList()->begin(person, TL("add ") + person->getTagStr() + " '" + personParameters.id + "'");
                overwriteDemandElement();
                myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(person, true), true);
                myNet->getViewNet()->getUndoList()->end();
            } else {
                myNet->getAttributeCarriers()->insertDemandElement(person);
                // set person as child of type
                type->addChildElement(person);
                person->incRef("buildPerson");
            }
        }
    }
}


void
GNERouteHandler::buildPersonFlow(const CommonXMLStructure::SumoBaseObject* /*sumoBaseObject*/, const SUMOVehicleParameter& personFlowParameters) {
    // first check if ID is duplicated
    if (!isPersonIdDuplicated(personFlowParameters.id)) {
        // obtain  type
        GNEDemandElement* type = getType(personFlowParameters.vtypeid);
        if (type == nullptr) {
            writeError(TLF("Invalid vehicle type '%' used in % '%'.", personFlowParameters.vtypeid, toString(personFlowParameters.tag), personFlowParameters.id));
        } else {
            // create personFlow using personFlowParameters
            GNEDemandElement* personFlow = new GNEPerson(SUMO_TAG_PERSONFLOW, myNet, type, personFlowParameters);
            if (myAllowUndoRedo) {
                myNet->getViewNet()->getUndoList()->begin(personFlow, TL("add ") + personFlow->getTagStr() + " '" + personFlowParameters.id + "'");
                overwriteDemandElement();
                myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(personFlow, true), true);
                myNet->getViewNet()->getUndoList()->end();
            } else {
                myNet->getAttributeCarriers()->insertDemandElement(personFlow);
                // set personFlow as child of type
                type->addChildElement(personFlow);
                personFlow->incRef("buildPersonFlow");
            }
        }
    }
}


void
GNERouteHandler::buildPersonTrip(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const CommonXMLStructure::PlanParameters& planParameters,
                                 const double arrivalPos, const std::vector<std::string>& types, const std::vector<std::string>& modes,
                                 const std::vector<std::string>& lines, const double walkFactor, const std::string& group) {
    // get values
    GNEDemandElement* personParent = getPersonParent(sumoBaseObject);
    const auto tagIcon = GNEDemandElementPlan::getPersonTripTagIcon(planParameters);
    GNEPlanParents planParents = GNEPlanParents(planParameters, myNet->getAttributeCarriers());
    // check conditions
    if (personParent == nullptr) {
        WRITE_ERROR(TL("invalid person parent"));
    } else if (tagIcon.first == SUMO_TAG_NOTHING) {
        WRITE_ERROR(TL("invalid combination for personTrip"));
    } else if (planParents.checkIntegrity(tagIcon.first, personParent, planParameters)) {
        // build person trip
        GNEDemandElement* personTrip = new GNEPersonTrip(myNet, tagIcon.first, tagIcon.second, personParent, planParents,
                arrivalPos, types, modes, lines, walkFactor, group);
        // continue depending of undo.redo
        if (myAllowUndoRedo) {
            myNet->getViewNet()->getUndoList()->begin(personTrip, TLF("add % in '%'", personTrip->getTagStr(), personParent->getID()));
            overwriteDemandElement();
            myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(personTrip, true), true);
            myNet->getViewNet()->getUndoList()->end();
        } else {
            myNet->getAttributeCarriers()->insertDemandElement(personTrip);
            // set child references
            personParent->addChildElement(personTrip);
            planParents.addChildElements(personTrip);
            personTrip->incRef("buildPersonTrip");
        }
    }
}


void
GNERouteHandler::buildWalk(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const CommonXMLStructure::PlanParameters& planParameters,
                           const double arrivalPos, const double speed, const SUMOTime duration) {
    // get values
    GNEDemandElement* personParent = getPersonParent(sumoBaseObject);
    const auto tagIcon = GNEDemandElementPlan::getWalkTagIcon(planParameters);
    GNEPlanParents planParents = GNEPlanParents(planParameters, myNet->getAttributeCarriers());
    // check conditions
    if (personParent == nullptr) {
        WRITE_ERROR(TL("invalid person parent"));
    } else if (tagIcon.first == SUMO_TAG_NOTHING) {
        WRITE_ERROR(TL("invalid combination for personTrip"));
    } else if (speed < 0) {
        writeErrorInvalidNegativeValue(SUMO_TAG_WALK, SUMO_ATTR_SPEED);
    } else if (duration < 0) {
        writeErrorInvalidNegativeValue(SUMO_TAG_WALK, SUMO_ATTR_DURATION);
    } else if (planParents.checkIntegrity(tagIcon.first, personParent, planParameters)) {
        // build person trip
        GNEDemandElement* walk = new GNEWalk(myNet, tagIcon.first, tagIcon.second, personParent, planParents, arrivalPos, speed, duration);
        // continue depending of undo.redo
        if (myAllowUndoRedo) {
            myNet->getViewNet()->getUndoList()->begin(walk, TLF("add % in '%'", walk->getTagStr(), personParent->getID()));
            overwriteDemandElement();
            myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(walk, true), true);
            myNet->getViewNet()->getUndoList()->end();
        } else {
            myNet->getAttributeCarriers()->insertDemandElement(walk);
            // set child references
            personParent->addChildElement(walk);
            planParents.addChildElements(walk);
            walk->incRef("buildWalk");
        }
    }
}


void
GNERouteHandler::buildRide(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const CommonXMLStructure::PlanParameters& planParameters,
                           const double arrivalPos, const std::vector<std::string>& lines, const std::string& group) {
    // get values
    GNEDemandElement* personParent = getPersonParent(sumoBaseObject);
    const auto tagIcon = GNEDemandElementPlan::getRideTagIcon(planParameters);
    GNEPlanParents planParents = GNEPlanParents(planParameters, myNet->getAttributeCarriers());
    // check conditions
    if (personParent == nullptr) {
        writeError(TL("invalid person parent"));
    } else if (tagIcon.first == SUMO_TAG_NOTHING) {
        writeError(TL("invalid combination for ride"));
    } else if (planParents.checkIntegrity(tagIcon.first, personParent, planParameters)) {
        // build ride
        GNEDemandElement* ride = new GNERide(myNet, tagIcon.first, tagIcon.second, personParent, planParents, arrivalPos, lines, group);
        // continue depending of undo-redo
        if (myAllowUndoRedo) {
            myNet->getViewNet()->getUndoList()->begin(ride, TLF("add % in '%'", ride->getTagStr(), personParent->getID()));
            overwriteDemandElement();
            myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(ride, true), true);
            myNet->getViewNet()->getUndoList()->end();
        } else {
            myNet->getAttributeCarriers()->insertDemandElement(ride);
            // set child references
            personParent->addChildElement(ride);
            planParents.addChildElements(ride);
            ride->incRef("buildRide");
        }
    }
}


void
GNERouteHandler::buildContainer(const CommonXMLStructure::SumoBaseObject* /*sumoBaseObject*/, const SUMOVehicleParameter& containerParameters) {
    // first check if ID is duplicated
    if (!isContainerIdDuplicated(containerParameters.id)) {
        // obtain  type
        GNEDemandElement* type = getType(containerParameters.vtypeid);
        if (type == nullptr) {
            writeError(TLF("Invalid vehicle type '%' used in % '%'.", containerParameters.vtypeid, toString(containerParameters.tag), containerParameters.id));
        } else {
            // create container using containerParameters
            GNEDemandElement* container = new GNEContainer(SUMO_TAG_CONTAINER, myNet, type, containerParameters);
            if (myAllowUndoRedo) {
                myNet->getViewNet()->getUndoList()->begin(container, TL("add ") + container->getTagStr() + " '" + container->getID() + "'");
                overwriteDemandElement();
                myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(container, true), true);
                myNet->getViewNet()->getUndoList()->end();
            } else {
                myNet->getAttributeCarriers()->insertDemandElement(container);
                // set container as child of type
                type->addChildElement(container);
                container->incRef("buildContainer");
            }
        }
    }
}


void
GNERouteHandler::buildContainerFlow(const CommonXMLStructure::SumoBaseObject* /*sumoBaseObject*/, const SUMOVehicleParameter& containerFlowParameters) {
    // first check if ID is duplicated
    if (!isContainerIdDuplicated(containerFlowParameters.id)) {
        // obtain  type
        GNEDemandElement* type = getType(containerFlowParameters.vtypeid);
        if (type == nullptr) {
            writeError(TLF("Invalid vehicle type '%' used in % '%'.", containerFlowParameters.vtypeid, toString(containerFlowParameters.tag), containerFlowParameters.id));
        } else {
            // create containerFlow using containerFlowParameters
            GNEDemandElement* containerFlow = new GNEContainer(SUMO_TAG_CONTAINERFLOW, myNet, type, containerFlowParameters);
            if (myAllowUndoRedo) {
                myNet->getViewNet()->getUndoList()->begin(containerFlow, TL("add ") + containerFlow->getTagStr() + " '" + containerFlow->getID() + "'");
                overwriteDemandElement();
                myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(containerFlow, true), true);
                myNet->getViewNet()->getUndoList()->end();
            } else {
                myNet->getAttributeCarriers()->insertDemandElement(containerFlow);
                // set containerFlow as child of type
                type->addChildElement(containerFlow);
                containerFlow->incRef("buildContainerFlow");
            }
        }
    }
}


void
GNERouteHandler::buildTransport(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const CommonXMLStructure::PlanParameters& planParameters,
                                const double arrivalPos, const std::vector<std::string>& lines, const std::string& group) {
    // get values
    GNEDemandElement* containerParent = getContainerParent(sumoBaseObject);
    const auto tagIcon = GNEDemandElementPlan::getTransportTagIcon(planParameters);
    GNEPlanParents planParents = GNEPlanParents(planParameters, myNet->getAttributeCarriers());
    // check conditions
    if (containerParent == nullptr) {
        WRITE_ERROR(TL("invalid container parent"));
    } else if (tagIcon.first == SUMO_TAG_NOTHING) {
        WRITE_ERROR(TL("invalid combination for personTrip"));
    } else if (planParents.checkIntegrity(tagIcon.first, containerParent, planParameters)) {
        // build transport
        GNEDemandElement* transport = new GNETransport(myNet, tagIcon.first, tagIcon.second, containerParent, planParents, arrivalPos, lines, group);
        // continue depending of undo-redo
        if (myAllowUndoRedo) {
            myNet->getViewNet()->getUndoList()->begin(transport, TLF("add % in '%'", transport->getTagStr(), containerParent->getID()));
            overwriteDemandElement();
            myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(transport, true), true);
            myNet->getViewNet()->getUndoList()->end();
        } else {
            myNet->getAttributeCarriers()->insertDemandElement(transport);
            // set child references
            containerParent->addChildElement(transport);
            planParents.addChildElements(transport);
            transport->incRef("buildTransport");
        }
    }
}


void
GNERouteHandler::buildTranship(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const CommonXMLStructure::PlanParameters& planParameters,
                               const double arrivalPosition, const double departPosition, const double speed, const SUMOTime duration) {
    // get values
    GNEDemandElement* containerParent = getContainerParent(sumoBaseObject);
    const auto tagIcon = GNEDemandElementPlan::getTranshipTagIcon(planParameters);
    GNEPlanParents planParents = GNEPlanParents(planParameters, myNet->getAttributeCarriers());
    // check conditions
    if (containerParent == nullptr) {
        WRITE_ERROR(TL("invalid container parent"));
    } else if (tagIcon.first == SUMO_TAG_NOTHING) {
        WRITE_ERROR(TL("invalid combination for personTrip"));
    } else if (speed < 0) {
        writeErrorInvalidNegativeValue(SUMO_TAG_TRANSHIP, SUMO_ATTR_SPEED);
    } else if (duration < 0) {
        writeErrorInvalidNegativeValue(SUMO_TAG_TRANSHIP, SUMO_ATTR_DURATION);
    } else if (planParents.checkIntegrity(tagIcon.first, containerParent, planParameters)) {
        // build tranship
        GNEDemandElement* tranship = new GNETranship(myNet, tagIcon.first, tagIcon.second, containerParent, planParents,
                arrivalPosition, departPosition, speed, duration);
        // continue depending of undo-redo
        if (myAllowUndoRedo) {
            myNet->getViewNet()->getUndoList()->begin(tranship, TLF("add % in '%'", tranship->getTagStr(), containerParent->getID()));
            overwriteDemandElement();
            myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(tranship, true), true);
            myNet->getViewNet()->getUndoList()->end();
        } else {
            myNet->getAttributeCarriers()->insertDemandElement(tranship);
            // set child references
            containerParent->addChildElement(tranship);
            planParents.addChildElements(tranship);
            tranship->incRef("buildTranship");
        }
    }
}


void
GNERouteHandler::buildPersonStop(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const CommonXMLStructure::PlanParameters& planParameters,
                                 const double endPos, const SUMOTime duration, const SUMOTime until,
                                 const std::string& actType, const bool friendlyPos, const int parameterSet) {
    // get values
    GNEDemandElement* personParent = getPersonParent(sumoBaseObject);
    const auto tagIcon = GNEDemandElementPlan::getPersonStopTagIcon(planParameters);
    GNEPlanParents planParents = GNEPlanParents(planParameters, myNet->getAttributeCarriers());
    // check conditions
    if (personParent == nullptr) {
        WRITE_ERROR(TL("invalid person parent"));
    } else if (tagIcon.first == SUMO_TAG_NOTHING) {
        WRITE_ERROR(TL("invalid combination for person stop"));
    } else if (planParents.checkIntegrity(tagIcon.first, personParent, planParameters)) {
        // build person stop
        GNEDemandElement* stopPlan = new GNEStopPlan(myNet, tagIcon.first, tagIcon.second, personParent, planParents,
                endPos, duration, until, actType, friendlyPos, parameterSet);
        // continue depending of undo-redo
        if (myAllowUndoRedo) {
            myNet->getViewNet()->getUndoList()->begin(stopPlan, TLF("add % in '%'", stopPlan->getTagStr(), personParent->getID()));
            overwriteDemandElement();
            myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(stopPlan, true), true);
            myNet->getViewNet()->getUndoList()->end();
        } else {
            myNet->getAttributeCarriers()->insertDemandElement(stopPlan);
            // set child references
            personParent->addChildElement(stopPlan);
            planParents.addChildElements(stopPlan);
            stopPlan->incRef("buildPersonStop");
        }
    }
}


void
GNERouteHandler::buildContainerStop(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const CommonXMLStructure::PlanParameters& planParameters,
                                    const double endPos, const SUMOTime duration,
                                    const SUMOTime until, const std::string& actType, const bool friendlyPos, const int parameterSet) {
    // get values
    GNEDemandElement* containerParent = getContainerParent(sumoBaseObject);
    const auto tagIcon = GNEDemandElementPlan::getContainerStopTagIcon(planParameters);
    GNEPlanParents planParents = GNEPlanParents(planParameters, myNet->getAttributeCarriers());
    // check conditions
    if (containerParent == nullptr) {
        WRITE_ERROR(TL("invalid container parent"));
    } else if (tagIcon.first == SUMO_TAG_NOTHING) {
        WRITE_ERROR(TL("invalid combination for containerStop"));
    } else if (planParents.checkIntegrity(tagIcon.first, containerParent, planParameters)) {
        // build container stop
        GNEDemandElement* stopPlan = new GNEStopPlan(myNet, tagIcon.first, tagIcon.second, containerParent, planParents,
                endPos, duration, until, actType, friendlyPos, parameterSet);
        // continue depending of undo-redo
        if (myAllowUndoRedo) {
            myNet->getViewNet()->getUndoList()->begin(stopPlan, TLF("add % in '%'", stopPlan->getTagStr(), containerParent->getID()));
            overwriteDemandElement();
            myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(stopPlan, true), true);
            myNet->getViewNet()->getUndoList()->end();
        } else {
            myNet->getAttributeCarriers()->insertDemandElement(stopPlan);
            // set child references
            containerParent->addChildElement(stopPlan);
            planParents.addChildElements(stopPlan);
            stopPlan->incRef("buildContainerStop");
        }
    }
}


void
GNERouteHandler::buildStop(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const CommonXMLStructure::PlanParameters& planParameters,
                           const SUMOVehicleParameter::Stop& stopParameters) {
    // get obj parent
    const auto objParent = sumoBaseObject->getParentSumoBaseObject();
    // continue depending of objParent
    if (objParent == nullptr) {
        WRITE_ERROR(TL("Stops needs a parent"));
    } else if ((objParent->getTag() == SUMO_TAG_PERSON) || (objParent->getTag() == SUMO_TAG_PERSONFLOW)) {
        buildPersonStop(sumoBaseObject, planParameters, stopParameters.endPos,
                        stopParameters.duration, stopParameters.until, stopParameters.actType, stopParameters.friendlyPos, stopParameters.parametersSet);
    } else if ((objParent->getTag() == SUMO_TAG_CONTAINER) || (objParent->getTag() == SUMO_TAG_CONTAINERFLOW)) {
        buildContainerStop(sumoBaseObject, planParameters, stopParameters.endPos,
                           stopParameters.duration, stopParameters.until, stopParameters.actType, stopParameters.friendlyPos, stopParameters.parametersSet);
    } else {
        // get vehicle tag
        SumoXMLTag vehicleTag = objParent->getTag();
        if (vehicleTag == SUMO_TAG_VEHICLE) {
            // check if vehicle is placed over route or with embedded route
            if (!objParent->hasStringAttribute(SUMO_ATTR_ROUTE)) {
                vehicleTag = GNE_TAG_VEHICLE_WITHROUTE;
            }
        } else if (vehicleTag == SUMO_TAG_FLOW) {
            if (objParent->hasStringAttribute(SUMO_ATTR_ROUTE)) {
                vehicleTag = GNE_TAG_FLOW_ROUTE;
            } else if (objParent->hasStringAttribute(SUMO_ATTR_FROM) && objParent->hasStringAttribute(SUMO_ATTR_TO)) {
                vehicleTag = SUMO_TAG_FLOW;
            } else {
                vehicleTag = GNE_TAG_FLOW_WITHROUTE;
            }
        }
        // get stop parent
        GNEDemandElement* stopParent = myNet->getAttributeCarriers()->retrieveDemandElement(vehicleTag, objParent->getStringAttribute(SUMO_ATTR_ID), false);
        // check if stopParent exist
        if (stopParent) {
            // flag for waypoint (is like a stop, but with extra attribute speed)
            bool waypoint = false;
            // avoid waypoints for person and containers
            if (!stopParent->getTagProperty().isPerson() && !stopParent->getTagProperty().isContainer()) {
                waypoint = (sumoBaseObject->getStopParameter().parametersSet & STOP_SPEED_SET) || (sumoBaseObject->getStopParameter().speed > 0);
            }
            // declare pointers to parent elements
            GNEAdditional* stoppingPlace = nullptr;
            GNELane* lane = nullptr;
            GNEEdge* edge = nullptr;
            // declare stopTagType
            SumoXMLTag stopTagType = SUMO_TAG_NOTHING;
            bool validParentDemandElement = true;
            // check conditions
            if (stopParameters.busstop.size() > 0) {
                stoppingPlace = myNet->getAttributeCarriers()->retrieveAdditional(SUMO_TAG_BUS_STOP, stopParameters.busstop, false);
                stopTagType = waypoint ? GNE_TAG_WAYPOINT_BUSSTOP : GNE_TAG_STOP_BUSSTOP;
                // check if is a train stop
                if (stoppingPlace == nullptr) {
                    stoppingPlace = myNet->getAttributeCarriers()->retrieveAdditional(SUMO_TAG_TRAIN_STOP, stopParameters.busstop, false);
                    stopTagType = waypoint ? GNE_TAG_WAYPOINT_TRAINSTOP : GNE_TAG_STOP_TRAINSTOP;
                }
                // containers cannot stops in busStops
                if (stopParent->getTagProperty().isContainer()) {
                    writeError(TL("Containers don't support stops at busStops or trainStops"));
                    validParentDemandElement = false;
                }
            } else if (stopParameters.containerstop.size() > 0) {
                stoppingPlace = myNet->getAttributeCarriers()->retrieveAdditional(SUMO_TAG_CONTAINER_STOP, stopParameters.containerstop, false);
                stopTagType = waypoint ? GNE_TAG_WAYPOINT_CONTAINERSTOP : GNE_TAG_STOP_CONTAINERSTOP;
                // persons cannot stops in containerStops
                if (stopParent->getTagProperty().isPerson()) {
                    writeError(TL("Persons don't support stops at containerStops"));
                    validParentDemandElement = false;
                }
            } else if (stopParameters.chargingStation.size() > 0) {
                stoppingPlace = myNet->getAttributeCarriers()->retrieveAdditional(SUMO_TAG_CHARGING_STATION, stopParameters.chargingStation, false);
                stopTagType = waypoint ? GNE_TAG_WAYPOINT_CHARGINGSTATION : GNE_TAG_STOP_CHARGINGSTATION;
                // check person and containers
                if (stopParent->getTagProperty().isPerson()) {
                    writeError(TL("Persons don't support stops at chargingStations"));
                    validParentDemandElement = false;
                } else if (stopParent->getTagProperty().isContainer()) {
                    writeError(TL("Containers don't support stops at chargingStations"));
                    validParentDemandElement = false;
                }
            } else if (stopParameters.parkingarea.size() > 0) {
                stoppingPlace = myNet->getAttributeCarriers()->retrieveAdditional(SUMO_TAG_PARKING_AREA, stopParameters.parkingarea, false);
                stopTagType = waypoint ? GNE_TAG_WAYPOINT_PARKINGAREA : GNE_TAG_STOP_PARKINGAREA;
                // check person and containers
                if (stopParent->getTagProperty().isPerson()) {
                    writeError(TL("Persons don't support stops at parkingAreas"));
                    validParentDemandElement = false;
                } else if (stopParent->getTagProperty().isContainer()) {
                    writeError(TL("Containers don't support stops at parkingAreas"));
                    validParentDemandElement = false;
                }
            } else if (stopParameters.lane.size() > 0) {
                lane = myNet->getAttributeCarriers()->retrieveLane(stopParameters.lane, false);
                stopTagType = waypoint ? GNE_TAG_WAYPOINT_LANE : GNE_TAG_STOP_LANE;
            } else if (stopParameters.edge.size() > 0) {
                edge = myNet->getAttributeCarriers()->retrieveEdge(stopParameters.edge, false);
                // check vehicles
                if (stopParent->getTagProperty().isVehicle()) {
                    writeError(TL("vehicles don't support stops at edges"));
                    validParentDemandElement = false;
                }
            }
            // overwrite lane with edge parent if we're handling a personStop
            if (lane && (stopParent->getTagProperty().isPerson() || stopParent->getTagProperty().isContainer())) {
                edge = lane->getParentEdge();
                lane = nullptr;
            }
            // first check that parent is valid
            if (validParentDemandElement) {
                // check if values are correct
                if (stoppingPlace && lane && edge) {
                    writeError(TL("A stop must be defined either over a stoppingPlace, a edge or a lane"));
                } else if (!stoppingPlace && !lane && !edge) {
                    writeError(TL("A stop requires only a stoppingPlace, edge or lane"));
                } else if (stoppingPlace) {
                    // create stop using stopParameters and stoppingPlace
                    GNEDemandElement* stop = nullptr;
                    if (stopParent->getTagProperty().isPerson()) {
                        if (stoppingPlace->getTagProperty().getTag() == SUMO_TAG_BUS_STOP) {
                            stop = new GNEStop(GNE_TAG_STOPPERSON_BUSSTOP, myNet, stopParent, stoppingPlace, stopParameters);
                        } else {
                            stop = new GNEStop(GNE_TAG_STOPPERSON_TRAINSTOP, myNet, stopParent, stoppingPlace, stopParameters);
                        }
                    } else if (stopParent->getTagProperty().isContainer()) {
                        stop = new GNEStop(GNE_TAG_STOPCONTAINER_CONTAINERSTOP, myNet, stopParent, stoppingPlace, stopParameters);
                    } else {
                        stop = new GNEStop(stopTagType, myNet, stopParent, stoppingPlace, stopParameters);
                    }
                    // add it depending of undoDemandElements
                    if (myAllowUndoRedo) {
                        myNet->getViewNet()->getUndoList()->begin(stop, TL("add ") + stop->getTagStr() + " in '" + stopParent->getID() + "'");
                        overwriteDemandElement();
                        myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(stop, true), true);
                        myNet->getViewNet()->getUndoList()->end();
                    } else {
                        myNet->getAttributeCarriers()->insertDemandElement(stop);
                        stoppingPlace->addChildElement(stop);
                        stopParent->addChildElement(stop);
                        stop->incRef("buildStoppingPlaceStop");
                    }
                } else if (lane) {
                    // create stop using stopParameters and lane (only for vehicles)
                    GNEDemandElement* stop = new GNEStop(stopTagType, myNet, stopParent, lane, stopParameters);
                    // add it depending of undoDemandElements
                    if (myAllowUndoRedo) {
                        myNet->getViewNet()->getUndoList()->begin(stop, TL("add ") + stop->getTagStr() + " in '" + stopParent->getID() + "'");
                        overwriteDemandElement();
                        myNet->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(stop, true), true);
                        myNet->getViewNet()->getUndoList()->end();
                    } else {
                        myNet->getAttributeCarriers()->insertDemandElement(stop);
                        lane->addChildElement(stop);
                        stopParent->addChildElement(stop);
                        stop->incRef("buildLaneStop");
                    }
                }
            }
        }
    }
}


bool
GNERouteHandler::buildPersonPlan(const GNEDemandElement* planTemplate, GNEDemandElement* personParent,
                                 GNEAttributesCreator* personPlanAttributes, GNEPlanCreator* planCreator,
                                 const bool centerAfterCreation) {
    // first check if person is valid
    if (personParent == nullptr) {
        return false;
    }
    // clear and set person object
    myPlanObject->clear();
    myPlanObject->setTag(personParent->getTagProperty().getTag());
    myPlanObject->addStringAttribute(SUMO_ATTR_ID, personParent->getID());
    // declare personPlan object
    CommonXMLStructure::SumoBaseObject* personPlanObject = new CommonXMLStructure::SumoBaseObject(myPlanObject);
    // get person plan attributes
    personPlanAttributes->getAttributesAndValues(personPlanObject, true);
    // get attributes
    const std::vector<std::string> types = personPlanObject->hasStringListAttribute(SUMO_ATTR_VTYPES) ? personPlanObject->getStringListAttribute(SUMO_ATTR_VTYPES) :
                                           personPlanObject->hasStringAttribute(SUMO_ATTR_VTYPES) ? GNEAttributeCarrier::parse<std::vector<std::string> >(personPlanObject->getStringAttribute(SUMO_ATTR_VTYPES)) :
                                           std::vector<std::string>();
    const std::vector<std::string> modes = personPlanObject->hasStringListAttribute(SUMO_ATTR_MODES) ? personPlanObject->getStringListAttribute(SUMO_ATTR_MODES) :
                                           personPlanObject->hasStringAttribute(SUMO_ATTR_MODES) ? GNEAttributeCarrier::parse<std::vector<std::string> >(personPlanObject->getStringAttribute(SUMO_ATTR_MODES)) :
                                           std::vector<std::string>();
    const std::vector<std::string> lines = personPlanObject->hasStringListAttribute(SUMO_ATTR_LINES) ? personPlanObject->getStringListAttribute(SUMO_ATTR_LINES) :
                                           personPlanObject->hasStringAttribute(SUMO_ATTR_LINES) ? GNEAttributeCarrier::parse<std::vector<std::string> >(personPlanObject->getStringAttribute(SUMO_ATTR_LINES)) :
                                           std::vector<std::string>();
    const double arrivalPos = personPlanObject->hasDoubleAttribute(SUMO_ATTR_ARRIVALPOS) ? personPlanObject->getDoubleAttribute(SUMO_ATTR_ARRIVALPOS) :
                              personPlanObject->hasStringAttribute(SUMO_ATTR_ARRIVALPOS) ? GNEAttributeCarrier::parse<double>(personPlanObject->getStringAttribute(SUMO_ATTR_ARRIVALPOS)) :
                              -1;
    const double endPos = personPlanObject->hasDoubleAttribute(SUMO_ATTR_ENDPOS) ? personPlanObject->getDoubleAttribute(SUMO_ATTR_ENDPOS) :
                          personPlanObject->hasStringAttribute(SUMO_ATTR_ENDPOS) ? GNEAttributeCarrier::parse<double>(personPlanObject->getStringAttribute(SUMO_ATTR_ENDPOS)) :
                          planCreator->getClickedPositionOverLane();
    const SUMOTime duration = personPlanObject->hasTimeAttribute(SUMO_ATTR_DURATION) ? personPlanObject->getTimeAttribute(SUMO_ATTR_DURATION) :
                              personPlanObject->hasStringAttribute(SUMO_ATTR_DURATION) ? GNEAttributeCarrier::parse<SUMOTime>(personPlanObject->getStringAttribute(SUMO_ATTR_DURATION)) :
                              0;
    const SUMOTime until = personPlanObject->hasTimeAttribute(SUMO_ATTR_UNTIL) ? personPlanObject->getTimeAttribute(SUMO_ATTR_UNTIL) :
                           personPlanObject->hasStringAttribute(SUMO_ATTR_UNTIL) ? GNEAttributeCarrier::parse<SUMOTime>(personPlanObject->getStringAttribute(SUMO_ATTR_UNTIL)) :
                           0;
    const std::string actType = personPlanObject->hasStringAttribute(SUMO_ATTR_ACTTYPE) ? personPlanObject->getStringAttribute(SUMO_ATTR_ACTTYPE) : "";
    const bool friendlyPos = personPlanObject->hasBoolAttribute(SUMO_ATTR_FRIENDLY_POS) ? personPlanObject->getBoolAttribute(SUMO_ATTR_FRIENDLY_POS) :
                             personPlanObject->hasStringAttribute(SUMO_ATTR_FRIENDLY_POS) ? GNEAttributeCarrier::parse<bool>(personPlanObject->getStringAttribute(SUMO_ATTR_FRIENDLY_POS)) :
                             false;
    const double walkFactor = personPlanObject->hasDoubleAttribute(SUMO_ATTR_WALKFACTOR) ? personPlanObject->getDoubleAttribute(SUMO_ATTR_WALKFACTOR) : 0;
    const std::string group = personPlanObject->hasStringAttribute(SUMO_ATTR_GROUP) ? personPlanObject->getStringAttribute(SUMO_ATTR_GROUP) : "";
    const double speed = personPlanObject->hasDoubleAttribute(SUMO_ATTR_SPEED) ? personPlanObject->getDoubleAttribute(SUMO_ATTR_SPEED) : 0;
    // build depending of plan type
    if (planTemplate->getTagProperty().isPlanWalk()) {
        buildWalk(personPlanObject, planCreator->getPlanParameteres(), arrivalPos, speed, duration);
    } else if (planTemplate->getTagProperty().isPlanPersonTrip()) {
        buildPersonTrip(personPlanObject, planCreator->getPlanParameteres(), arrivalPos, types, modes, lines, walkFactor, group);
    } else if (planTemplate->getTagProperty().isPlanRide()) {
        buildRide(personPlanObject, planCreator->getPlanParameteres(), arrivalPos, lines, group);
    } else if (planTemplate->getTagProperty().isPlanStopPerson()) {
        // set specific stop parameters
        int parameterSet = 0;
        if (personPlanObject->hasTimeAttribute(SUMO_ATTR_DURATION)) {
            parameterSet |= STOP_DURATION_SET;
        }
        if (personPlanObject->hasTimeAttribute(SUMO_ATTR_UNTIL)) {
            parameterSet |= STOP_UNTIL_SET;
        }
        buildPersonStop(personPlanObject, planCreator->getPlanParameteres(), endPos, duration, until, actType, friendlyPos, parameterSet);
    }
    // get person
    const auto person = myNet->getAttributeCarriers()->retrieveDemandElement(personPlanObject->getParentSumoBaseObject()->getTag(),
                        personPlanObject->getParentSumoBaseObject()->getStringAttribute(SUMO_ATTR_ID), false);
    if (person) {
        // center view after creation
        if (centerAfterCreation && !myNet->getViewNet()->getVisibleBoundary().around(person->getPositionInView())) {
            myNet->getViewNet()->centerTo(person->getPositionInView(), false);
        }
    }
    delete personPlanObject;
    return true;
}


bool
GNERouteHandler::buildContainerPlan(const GNEDemandElement* planTemplate, GNEDemandElement* containerParent,
                                    GNEAttributesCreator* containerPlanAttributes, GNEPlanCreator* planCreator,
                                    const bool centerAfterCreation) {
    // first check if container is valid
    if (containerParent == nullptr) {
        return false;
    }
    // clear and set container object
    myPlanObject->clear();
    myPlanObject->setTag(containerParent->getTagProperty().getTag());
    myPlanObject->addStringAttribute(SUMO_ATTR_ID, containerParent->getID());
    // declare containerPlan object
    CommonXMLStructure::SumoBaseObject* containerPlanObject = new CommonXMLStructure::SumoBaseObject(myPlanObject);
    // get container plan attributes
    containerPlanAttributes->getAttributesAndValues(containerPlanObject, true);
    // get attributes
    const double speed = containerPlanObject->hasDoubleAttribute(SUMO_ATTR_SPEED) ? containerPlanObject->getDoubleAttribute(SUMO_ATTR_SPEED) :
                         containerPlanObject->hasStringAttribute(SUMO_ATTR_SPEED) ? GNEAttributeCarrier::parse<double>(containerPlanObject->getStringAttribute(SUMO_ATTR_SPEED)) :
                         0;
    const std::vector<std::string> lines = containerPlanObject->hasStringListAttribute(SUMO_ATTR_LINES) ? containerPlanObject->getStringListAttribute(SUMO_ATTR_LINES) :
                                           containerPlanObject->hasStringAttribute(SUMO_ATTR_LINES) ? GNEAttributeCarrier::parse<std::vector<std::string> >(containerPlanObject->getStringAttribute(SUMO_ATTR_LINES)) :
                                           std::vector<std::string>();
    const double departPos = containerPlanObject->hasDoubleAttribute(SUMO_ATTR_DEPARTPOS) ? containerPlanObject->getDoubleAttribute(SUMO_ATTR_DEPARTPOS) :
                             containerPlanObject->hasStringAttribute(SUMO_ATTR_DEPARTPOS) ? GNEAttributeCarrier::parse<double>(containerPlanObject->getStringAttribute(SUMO_ATTR_DEPARTPOS)) :
                             -1;
    const double arrivalPos = containerPlanObject->hasDoubleAttribute(SUMO_ATTR_ARRIVALPOS) ? containerPlanObject->getDoubleAttribute(SUMO_ATTR_ARRIVALPOS) :
                              containerPlanObject->hasStringAttribute(SUMO_ATTR_ARRIVALPOS) ? GNEAttributeCarrier::parse<double>(containerPlanObject->getStringAttribute(SUMO_ATTR_ARRIVALPOS)) :
                              -1;
    const double endPos = containerPlanObject->hasDoubleAttribute(SUMO_ATTR_ENDPOS) ? containerPlanObject->getDoubleAttribute(SUMO_ATTR_ENDPOS) :
                          containerPlanObject->hasStringAttribute(SUMO_ATTR_ENDPOS) ? GNEAttributeCarrier::parse<double>(containerPlanObject->getStringAttribute(SUMO_ATTR_ENDPOS)) :
                          planCreator->getClickedPositionOverLane();
    const SUMOTime duration = containerPlanObject->hasTimeAttribute(SUMO_ATTR_DURATION) ? containerPlanObject->getTimeAttribute(SUMO_ATTR_DURATION) :
                              containerPlanObject->hasStringAttribute(SUMO_ATTR_DURATION) ? GNEAttributeCarrier::parse<SUMOTime>(containerPlanObject->getStringAttribute(SUMO_ATTR_DURATION)) :
                              0;
    const SUMOTime until = containerPlanObject->hasTimeAttribute(SUMO_ATTR_UNTIL) ? containerPlanObject->getTimeAttribute(SUMO_ATTR_UNTIL) :
                           containerPlanObject->hasStringAttribute(SUMO_ATTR_UNTIL) ? GNEAttributeCarrier::parse<SUMOTime>(containerPlanObject->getStringAttribute(SUMO_ATTR_UNTIL)) :
                           0;
    const std::string actType = containerPlanObject->hasStringAttribute(SUMO_ATTR_ACTTYPE) ? containerPlanObject->getStringAttribute(SUMO_ATTR_ACTTYPE) : "";
    const bool friendlyPos = containerPlanObject->hasBoolAttribute(SUMO_ATTR_FRIENDLY_POS) ? containerPlanObject->getBoolAttribute(SUMO_ATTR_FRIENDLY_POS) :
                             containerPlanObject->hasStringAttribute(SUMO_ATTR_FRIENDLY_POS) ? GNEAttributeCarrier::parse<bool>(containerPlanObject->getStringAttribute(SUMO_ATTR_FRIENDLY_POS)) :
                             false;
    const std::string group = containerPlanObject->hasStringAttribute(SUMO_ATTR_GROUP) ? containerPlanObject->getStringAttribute(SUMO_ATTR_GROUP) : "";
    // build depending of plan type
    if (planTemplate->getTagProperty().isPlanTranship()) {
        buildTranship(containerPlanObject, planCreator->getPlanParameteres(), arrivalPos, departPos, speed, duration);
    } else if (planTemplate->getTagProperty().isPlanTransport()) {
        buildTransport(containerPlanObject, planCreator->getPlanParameteres(), arrivalPos, lines, group);
    } else if (planTemplate->getTagProperty().isPlanStopContainer()) {
        // set stops specific parameters
        int parameterSet = 0;
        if (containerPlanObject->hasTimeAttribute(SUMO_ATTR_DURATION)) {
            parameterSet |= STOP_DURATION_SET;
        }
        if (containerPlanObject->hasTimeAttribute(SUMO_ATTR_UNTIL)) {
            parameterSet |= STOP_UNTIL_SET;
        }
        buildContainerStop(containerPlanObject, planCreator->getPlanParameteres(), endPos, duration, until, actType, friendlyPos, parameterSet);
    }
    // get container
    const auto container = myNet->getAttributeCarriers()->retrieveDemandElement(containerPlanObject->getParentSumoBaseObject()->getTag(),
                           containerPlanObject->getParentSumoBaseObject()->getStringAttribute(SUMO_ATTR_ID), false);
    if (container) {
        // center view after creation
        if (centerAfterCreation && !myNet->getViewNet()->getVisibleBoundary().around(container->getPositionInView())) {
            myNet->getViewNet()->centerTo(container->getPositionInView(), false);
        }
    }
    delete containerPlanObject;
    return true;
}


void
GNERouteHandler::duplicatePlan(const GNEDemandElement* originalPlan, GNEDemandElement* newParent) {
    const auto& tagProperty = originalPlan->getTagProperty();
    // clear and set container object
    myPlanObject->clear();
    myPlanObject->setTag(newParent->getTagProperty().getTag());
    myPlanObject->addStringAttribute(SUMO_ATTR_ID, newParent->getID());
    // declare personPlan object for adding all attributes
    CommonXMLStructure::SumoBaseObject* planObject = new CommonXMLStructure::SumoBaseObject(myPlanObject);
    planObject->setTag(tagProperty.getTag());
    // declare parameters
    CommonXMLStructure::PlanParameters planParameters;
    // from-to elements
    if (tagProperty.planFromEdge()) {
        planParameters.fromEdge = originalPlan->getAttribute(SUMO_ATTR_FROM);
    }
    if (tagProperty.planToEdge()) {
        planParameters.toEdge = originalPlan->getAttribute(SUMO_ATTR_TO);
    }
    if (tagProperty.planFromJunction()) {
        planParameters.fromJunction = originalPlan->getAttribute(SUMO_ATTR_FROM_JUNCTION);
    }
    if (tagProperty.planToJunction()) {
        planParameters.toJunction = originalPlan->getAttribute(SUMO_ATTR_TO_JUNCTION);
    }
    if (tagProperty.planFromTAZ()) {
        planParameters.fromTAZ = originalPlan->getAttribute(SUMO_ATTR_FROM_TAZ);
    }
    if (tagProperty.planToTAZ()) {
        planParameters.toTAZ = originalPlan->getAttribute(SUMO_ATTR_TO_TAZ);
    }
    if (tagProperty.planFromBusStop()) {
        planParameters.fromBusStop = originalPlan->getAttribute(GNE_ATTR_FROM_BUSSTOP);
    }
    if (tagProperty.planToBusStop()) {
        planParameters.toBusStop = originalPlan->getAttribute(SUMO_ATTR_BUS_STOP);
    }
    if (tagProperty.planFromTrainStop()) {
        planParameters.fromTrainStop = originalPlan->getAttribute(GNE_ATTR_FROM_TRAINSTOP);
    }
    if (tagProperty.planToTrainStop()) {
        planParameters.toTrainStop = originalPlan->getAttribute(SUMO_ATTR_TRAIN_STOP);
    }
    if (tagProperty.planFromContainerStop()) {
        planParameters.fromContainerStop = originalPlan->getAttribute(GNE_ATTR_FROM_CONTAINERSTOP);
    }
    if (tagProperty.planToContainerStop()) {
        planParameters.toContainerStop = originalPlan->getAttribute(SUMO_ATTR_CONTAINER_STOP);
    }
    // single elements
    if (tagProperty.planEdge()) {
        planParameters.toEdge = originalPlan->getAttribute(SUMO_ATTR_EDGE);
    }
    if (tagProperty.planBusStop()) {
        planParameters.toBusStop = originalPlan->getAttribute(SUMO_ATTR_BUS_STOP);
    }
    if (tagProperty.planTrainStop()) {
        planParameters.toTrainStop = originalPlan->getAttribute(SUMO_ATTR_TRAIN_STOP);
    }
    if (tagProperty.planContainerStop()) {
        planParameters.toContainerStop = originalPlan->getAttribute(SUMO_ATTR_CONTAINER_STOP);
    }
    // route
    if (tagProperty.planRoute()) {
        planParameters.toRoute = originalPlan->getAttribute(SUMO_ATTR_ROUTE);
    }
    // path
    if (tagProperty.planConsecutiveEdges()) {
        planParameters.consecutiveEdges = GNEAttributeCarrier::parse<std::vector<std::string> >(originalPlan->getAttribute(SUMO_ATTR_EDGES));
    }
    // other elements
    planObject->addTimeAttribute(SUMO_ATTR_DURATION, 60);
    planObject->addTimeAttribute(SUMO_ATTR_UNTIL, 0);
    planObject->addDoubleAttribute(SUMO_ATTR_DEPARTPOS, 0);
    planObject->addDoubleAttribute(SUMO_ATTR_ARRIVALPOS, -1);
    planObject->addDoubleAttribute(SUMO_ATTR_ENDPOS, 0);
    planObject->addDoubleAttribute(SUMO_ATTR_SPEED, 1.39);
    planObject->addBoolAttribute(SUMO_ATTR_FRIENDLY_POS, false);
    // add rest of attributes
    for (const auto& attrProperty : tagProperty) {
        if (!planObject->hasStringAttribute(attrProperty.getAttr())) {
            if (attrProperty.isFloat()) {
                if (!originalPlan->getAttribute(attrProperty.getAttr()).empty()) {
                    planObject->addDoubleAttribute(attrProperty.getAttr(), originalPlan->getAttributeDouble(attrProperty.getAttr()));
                }
            } else if (attrProperty.isSUMOTime()) {
                if (!originalPlan->getAttribute(attrProperty.getAttr()).empty()) {
                    planObject->addTimeAttribute(attrProperty.getAttr(), GNEAttributeCarrier::parse<SUMOTime>(originalPlan->getAttribute(attrProperty.getAttr())));
                }
            } else if (attrProperty.isBool()) {
                planObject->addBoolAttribute(attrProperty.getAttr(), GNEAttributeCarrier::parse<bool>(originalPlan->getAttribute(attrProperty.getAttr())));
            } else if (attrProperty.isList()) {
                planObject->addStringListAttribute(attrProperty.getAttr(), GNEAttributeCarrier::parse<std::vector<std::string> >(originalPlan->getAttribute(attrProperty.getAttr())));
            } else {
                planObject->addStringAttribute(attrProperty.getAttr(), originalPlan->getAttribute(attrProperty.getAttr()));
            }
        }
    }
    // create plan
    if (tagProperty.isPlanPersonTrip()) {
        buildPersonTrip(planObject, planParameters,
                        planObject->getDoubleAttribute(SUMO_ATTR_ARRIVALPOS),
                        planObject->getStringListAttribute(SUMO_ATTR_VTYPES),
                        planObject->getStringListAttribute(SUMO_ATTR_MODES),
                        planObject->getStringListAttribute(SUMO_ATTR_LINES),
                        planObject->getDoubleAttribute(SUMO_ATTR_WALKFACTOR),
                        planObject->getStringAttribute(SUMO_ATTR_GROUP));
    } else if (tagProperty.isPlanWalk()) {
        buildWalk(planObject, planParameters,
                  planObject->getDoubleAttribute(SUMO_ATTR_ARRIVALPOS),
                  planObject->getDoubleAttribute(SUMO_ATTR_SPEED),
                  planObject->getTimeAttribute(SUMO_ATTR_DURATION));
    } else if (tagProperty.isPlanRide()) {
        buildRide(planObject, planParameters,
                  planObject->getDoubleAttribute(SUMO_ATTR_ARRIVALPOS),
                  planObject->getStringListAttribute(SUMO_ATTR_LINES),
                  planObject->getStringAttribute(SUMO_ATTR_GROUP));
    } else if (tagProperty.isPlanStopPerson()) {
        // set parameters
        int parameterSet = 0;
        if (planObject->hasTimeAttribute(SUMO_ATTR_DURATION)) {
            parameterSet |= STOP_DURATION_SET;
        }
        if (planObject->hasTimeAttribute(SUMO_ATTR_UNTIL)) {
            parameterSet |= STOP_UNTIL_SET;
        }
        buildPersonStop(planObject, planParameters,
                        planObject->getDoubleAttribute(SUMO_ATTR_ENDPOS),
                        planObject->getTimeAttribute(SUMO_ATTR_DURATION),
                        planObject->getTimeAttribute(SUMO_ATTR_UNTIL),
                        planObject->getStringAttribute(SUMO_ATTR_ACTTYPE),
                        planObject->getBoolAttribute(SUMO_ATTR_FRIENDLY_POS),
                        parameterSet);
    } else if (tagProperty.isPlanTransport()) {
        buildTransport(planObject, planParameters,
                       planObject->getDoubleAttribute(SUMO_ATTR_ARRIVALPOS),
                       planObject->getStringListAttribute(SUMO_ATTR_LINES),
                       planObject->getStringAttribute(SUMO_ATTR_GROUP));
    } else if (tagProperty.isPlanTranship()) {
        buildTranship(planObject, planParameters,
                      planObject->getDoubleAttribute(SUMO_ATTR_ARRIVALPOS),
                      planObject->getDoubleAttribute(SUMO_ATTR_DEPARTPOS),
                      planObject->getDoubleAttribute(SUMO_ATTR_SPEED),
                      planObject->getTimeAttribute(SUMO_ATTR_DURATION));
    } else if (tagProperty.isPlanStopContainer()) {
        // set parameters
        int parameterSet = 0;
        if (planObject->hasTimeAttribute(SUMO_ATTR_DURATION)) {
            parameterSet |= STOP_DURATION_SET;
        }
        if (planObject->hasTimeAttribute(SUMO_ATTR_UNTIL)) {
            parameterSet |= STOP_UNTIL_SET;
        }
        buildContainerStop(planObject, planParameters,
                           planObject->getDoubleAttribute(SUMO_ATTR_ENDPOS),
                           planObject->getTimeAttribute(SUMO_ATTR_DURATION),
                           planObject->getTimeAttribute(SUMO_ATTR_UNTIL),
                           planObject->getStringAttribute(SUMO_ATTR_ACTTYPE),
                           planObject->getBoolAttribute(SUMO_ATTR_FRIENDLY_POS),
                           parameterSet);
    } else {
        throw ProcessError("Invalid plan for duplicating");
    }
}


bool
GNERouteHandler::isVehicleIdDuplicated(const std::string& id) {
    // declare vehicle tags vector
    const std::vector<SumoXMLTag> vehicleTags = {
        SUMO_TAG_VEHICLE, GNE_TAG_FLOW_ROUTE,
        SUMO_TAG_TRIP, SUMO_TAG_FLOW,                       // over edges
        SUMO_TAG_VEHICLE, GNE_TAG_FLOW_ROUTE,               // over route
        GNE_TAG_VEHICLE_WITHROUTE, GNE_TAG_FLOW_WITHROUTE,  // over embedded routes
        GNE_TAG_TRIP_TAZS, GNE_TAG_FLOW_TAZS,               // over TAZs
        GNE_TAG_TRIP_JUNCTIONS, GNE_TAG_FLOW_JUNCTIONS      // over junctions
    };
    for (const auto& vehicleTag : vehicleTags) {
        if (!checkDuplicatedDemandElement(vehicleTag, id)) {
            writeError(TLF("There is another % with the same ID='%'.", toString(vehicleTag), id));
            return true;
        }
    }
    return false;
}


bool
GNERouteHandler::isViaAttributeValid(const std::vector<std::string>& via) {
    for (const auto& edgeID : via) {
        if (myNet->getAttributeCarriers()->retrieveEdge(edgeID, false) == nullptr) {
            writeError(TLF("Via edge '%' doesn't exist.", edgeID));
            return false;
        }
    }
    return true;
}


bool
GNERouteHandler::isPersonIdDuplicated(const std::string& id) {
    const std::vector<SumoXMLTag> personTags = std::vector<SumoXMLTag>({SUMO_TAG_PERSON, SUMO_TAG_PERSONFLOW});
    for (const auto& personTag : personTags) {
        if (!checkDuplicatedDemandElement(personTag, id)) {
            writeError(TLF("There is another % with the same ID='%'.", toString(personTag), id));
            return true;
        }
    }
    return false;
}


bool
GNERouteHandler::isContainerIdDuplicated(const std::string& id) {
    const std::vector<SumoXMLTag> containerTags = std::vector<SumoXMLTag>({SUMO_TAG_CONTAINER, SUMO_TAG_CONTAINERFLOW});
    for (const auto& containerTag : containerTags) {
        if (!checkDuplicatedDemandElement(containerTag, id)) {
            writeError(TLF("There is another % with the same ID='%'.", toString(containerTag), id));
            return true;
        }
    }
    return false;
}


void
GNERouteHandler::transformToVehicle(GNEVehicle* originalVehicle, bool createEmbeddedRoute) {
    // get pointer to net
    GNENet* net = originalVehicle->getNet();
    // check if transform after creation
    const bool inspectAfterTransform = net->getViewNet()->isAttributeCarrierInspected(originalVehicle);
    // declare route handler
    GNERouteHandler routeHandler("", net, true, false);
    // make a copy of the vehicle parameters
    SUMOVehicleParameter vehicleParameters = *originalVehicle;
    // obtain vClass
    const auto vClass = originalVehicle->getVClass();
    // set "yellow" as original route color
    RGBColor routeColor = RGBColor::YELLOW;
    // declare edges
    std::vector<GNEEdge*> routeEdges;
    // obtain edges depending of tag
    if (originalVehicle->getTagProperty().vehicleRoute()) {
        // get route edges
        routeEdges = originalVehicle->getParentDemandElements().at(1)->getParentEdges();
        // get original route color
        routeColor = originalVehicle->getParentDemandElements().back()->getColor();
    } else if (originalVehicle->getTagProperty().vehicleRouteEmbedded()) {
        // get embedded route edges
        routeEdges = originalVehicle->getChildDemandElements().front()->getParentEdges();
    } else if (originalVehicle->getTagProperty().vehicleEdges()) {
        // calculate path using from-via-to edges
        routeEdges = originalVehicle->getNet()->getPathManager()->getPathCalculator()->calculateDijkstraPath(originalVehicle->getVClass(), originalVehicle->getParentEdges());
    }
    // declare edge IDs
    std::vector<std::string> edgeIDs;
    for (const auto& edge : routeEdges) {
        edgeIDs.push_back(edge->getID());
    }
    // only continue if edges are valid
    if (routeEdges.empty()) {
        // declare header
        const std::string header = "Problem transforming to vehicle";
        // declare message
        const std::string message = "Vehicle cannot be transformed. Invalid number of edges";
        // write warning
        WRITE_DEBUG("Opened FXMessageBox " + header);
        // open message box
        FXMessageBox::warning(originalVehicle->getNet()->getViewNet()->getApp(), MBOX_OK, header.c_str(), "%s", message.c_str());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox " + header);
    } else {
        // begin undo-redo operation
        net->getViewNet()->getUndoList()->begin(originalVehicle, "transform " + originalVehicle->getTagStr() + " to " + toString(SUMO_TAG_VEHICLE));
        // first delete vehicle
        net->deleteDemandElement(originalVehicle, net->getViewNet()->getUndoList());
        // check if new vehicle must have an embedded route
        if (createEmbeddedRoute) {
            // change tag in vehicle parameters
            vehicleParameters.tag = GNE_TAG_VEHICLE_WITHROUTE;
            // create a flow with embebbed routes
            CommonXMLStructure::SumoBaseObject* vehicleBaseOBject = new CommonXMLStructure::SumoBaseObject(nullptr);
            CommonXMLStructure::SumoBaseObject* routeBaseOBject = new CommonXMLStructure::SumoBaseObject(vehicleBaseOBject);
            // fill parameters
            vehicleBaseOBject->setTag(SUMO_TAG_VEHICLE);
            vehicleBaseOBject->addStringAttribute(SUMO_ATTR_ID, vehicleParameters.id);
            vehicleBaseOBject->setVehicleParameter(&vehicleParameters);
            // build embedded route
            routeHandler.buildEmbeddedRoute(routeBaseOBject, edgeIDs, RGBColor::INVISIBLE, false, 0, {});
            // delete vehicle base object
            delete vehicleBaseOBject;
        } else {
            // change tag in vehicle parameters
            vehicleParameters.tag = SUMO_TAG_VEHICLE;
            // generate route ID
            const std::string routeID = net->getAttributeCarriers()->generateDemandElementID(SUMO_TAG_ROUTE);
            // build route
            routeHandler.buildRoute(nullptr, routeID, vClass, edgeIDs, routeColor, false, 0, 1.0, {});
            // set route ID in vehicle parameters
            vehicleParameters.routeid = routeID;
            // create vehicle
            routeHandler.buildVehicleOverRoute(nullptr, vehicleParameters);
        }
        // end undo-redo operation
        net->getViewNet()->getUndoList()->end();
        // check if inspect
        if (inspectAfterTransform) {
            // get created element
            auto transformedVehicle = net->getAttributeCarriers()->retrieveDemandElement(vehicleParameters.tag, vehicleParameters.id);
            // inspect it
            net->getViewNet()->getViewParent()->getInspectorFrame()->inspectSingleElement(transformedVehicle);
        }
    }
}


void
GNERouteHandler::transformToRouteFlow(GNEVehicle* originalVehicle, bool createEmbeddedRoute) {
    // get pointer to net
    GNENet* net = originalVehicle->getNet();
    // check if transform after creation
    const bool inspectAfterTransform = net->getViewNet()->isAttributeCarrierInspected(originalVehicle);
    // declare route handler
    GNERouteHandler routeHandler("", net, true, false);
    // obtain vehicle parameters
    SUMOVehicleParameter vehicleParameters = *originalVehicle;
    // obtain vClass
    const auto vClass = originalVehicle->getVClass();
    // set "yellow" as original route color
    RGBColor routeColor = RGBColor::YELLOW;
    // declare edges
    std::vector<GNEEdge*> routeEdges;
    // obtain edges depending of tag
    if (originalVehicle->getTagProperty().vehicleRoute()) {
        // get route edges
        routeEdges = originalVehicle->getParentDemandElements().back()->getParentEdges();
        // get original route color
        routeColor = originalVehicle->getParentDemandElements().back()->getColor();
    } else if (originalVehicle->getTagProperty().vehicleRouteEmbedded()) {
        // get embedded route edges
        routeEdges = originalVehicle->getChildDemandElements().front()->getParentEdges();
    } else if (originalVehicle->getTagProperty().vehicleEdges()) {
        // calculate path using from-via-to edges
        routeEdges = originalVehicle->getNet()->getPathManager()->getPathCalculator()->calculateDijkstraPath(originalVehicle->getVClass(), originalVehicle->getParentEdges());
    }
    // declare edge IDs
    std::vector<std::string> edgeIDs;
    for (const auto& edge : routeEdges) {
        edgeIDs.push_back(edge->getID());
    }
    // only continue if edges are valid
    if (routeEdges.empty()) {
        // declare header
        const std::string header = "Problem transforming to vehicle";
        // declare message
        const std::string message = "Vehicle cannot be transformed. Invalid number of edges";
        // write warning
        WRITE_DEBUG("Opened FXMessageBox " + header);
        // open message box
        FXMessageBox::warning(originalVehicle->getNet()->getViewNet()->getApp(), MBOX_OK, header.c_str(), "%s", message.c_str());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox " + header);
    } else {
        // begin undo-redo operation
        net->getViewNet()->getUndoList()->begin(originalVehicle, "transform " + originalVehicle->getTagStr() + " to " + toString(SUMO_TAG_VEHICLE));
        // first delete vehicle
        net->deleteDemandElement(originalVehicle, net->getViewNet()->getUndoList());
        // change depart
        if (!GNEAttributeCarrier::getTagProperty(vehicleParameters.tag).isFlow()) {
            // get template flow
            const auto templateFlow = net->getViewNet()->getViewParent()->getVehicleFrame()->getVehicleTagSelector()->getTemplateAC(GNE_TAG_FLOW_ROUTE);
            // set flow parameters
            vehicleParameters.repetitionEnd = vehicleParameters.depart + string2time("3600");
            vehicleParameters.repetitionNumber = GNEAttributeCarrier::parse<int>(templateFlow->getAttribute(SUMO_ATTR_NUMBER));
            vehicleParameters.repetitionOffset = string2time(templateFlow->getAttribute(SUMO_ATTR_PERIOD));
            vehicleParameters.repetitionProbability = GNEAttributeCarrier::parse<double>(templateFlow->getAttribute(SUMO_ATTR_PROB));
            // by default, number and end enabled
            vehicleParameters.parametersSet = GNEAttributeCarrier::parse<int>(templateFlow->getAttribute(GNE_ATTR_FLOWPARAMETERS));
        }
        // check if new vehicle must have an embedded route
        if (createEmbeddedRoute) {
            // change tag in vehicle parameters
            vehicleParameters.tag = GNE_TAG_FLOW_WITHROUTE;
            // create a flow with embebbed routes
            CommonXMLStructure::SumoBaseObject* vehicleBaseOBject = new CommonXMLStructure::SumoBaseObject(nullptr);
            CommonXMLStructure::SumoBaseObject* routeBaseOBject = new CommonXMLStructure::SumoBaseObject(vehicleBaseOBject);
            // fill parameters
            vehicleBaseOBject->setTag(SUMO_TAG_FLOW);
            vehicleBaseOBject->addStringAttribute(SUMO_ATTR_ID, vehicleParameters.id);
            vehicleBaseOBject->setVehicleParameter(&vehicleParameters);
            // build embedded route
            routeHandler.buildEmbeddedRoute(routeBaseOBject, edgeIDs, RGBColor::INVISIBLE, false, 0, {});
            // delete vehicle base object
            delete vehicleBaseOBject;
        } else {
            // change tag in vehicle parameters
            vehicleParameters.tag = GNE_TAG_FLOW_ROUTE;
            // generate a new route id
            const std::string routeID = net->getAttributeCarriers()->generateDemandElementID(SUMO_TAG_ROUTE);
            // build route
            routeHandler.buildRoute(nullptr, routeID, vClass, edgeIDs, routeColor, false, 0, 1.0, {});
            // set route ID in vehicle parameters
            vehicleParameters.routeid = routeID;
            // create vehicle
            routeHandler.buildFlowOverRoute(nullptr, vehicleParameters);
        }
        // end undo-redo operation
        net->getViewNet()->getUndoList()->end();
        // check if inspect
        if (inspectAfterTransform) {
            // get created element
            auto transformedVehicle = net->getAttributeCarriers()->retrieveDemandElement(vehicleParameters.tag, vehicleParameters.id);
            // inspect it
            net->getViewNet()->getViewParent()->getInspectorFrame()->inspectSingleElement(transformedVehicle);
        }
    }
}


void
GNERouteHandler::transformToTrip(GNEVehicle* originalVehicle) {
    // get pointer to net
    GNENet* net = originalVehicle->getNet();
    // check if transform after creation
    const bool inspectAfterTransform = net->getViewNet()->isAttributeCarrierInspected(originalVehicle);
    // declare route handler
    GNERouteHandler routeHandler("", net, true, false);
    // obtain vehicle parameters
    SUMOVehicleParameter vehicleParameters = *originalVehicle;
    // get route
    GNEDemandElement* route = nullptr;
    // declare edges
    std::vector<GNEEdge*> edges;
    // obtain edges depending of tag
    if (originalVehicle->getTagProperty().vehicleRoute()) {
        // set route
        route = originalVehicle->getParentDemandElements().back();
        // get route edges
        edges = route->getParentEdges();
    } else if (originalVehicle->getTagProperty().vehicleRouteEmbedded()) {
        // get embedded route edges
        edges = originalVehicle->getChildDemandElements().front()->getParentEdges();
    } else if (originalVehicle->getTagProperty().vehicleEdges()) {
        // just take parent edges (from and to)
        edges = originalVehicle->getParentEdges();
    }
    // only continue if edges are valid
    if (edges.size() < 2) {
        // declare header
        const std::string header = "Problem transforming to vehicle";
        // declare message
        const std::string message = "Vehicle cannot be transformed. Invalid number of edges";
        // write warning
        WRITE_DEBUG("Opened FXMessageBox " + header);
        // open message box
        FXMessageBox::warning(originalVehicle->getNet()->getViewNet()->getApp(), MBOX_OK, header.c_str(), "%s", message.c_str());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox " + header);
    } else {
        // begin undo-redo operation
        net->getViewNet()->getUndoList()->begin(originalVehicle, "transform " + originalVehicle->getTagStr() + " to " + toString(SUMO_TAG_TRIP));
        // first delete vehicle
        net->deleteDemandElement(originalVehicle, net->getViewNet()->getUndoList());
        // check if route has to be deleted
        if (route && route->getChildDemandElements().empty()) {
            net->deleteDemandElement(route, net->getViewNet()->getUndoList());
        }
        // change tag in vehicle parameters
        vehicleParameters.tag = SUMO_TAG_TRIP;
        // create trip
        routeHandler.buildTrip(nullptr, vehicleParameters, edges.front()->getID(), edges.back()->getID());
        // end undo-redo operation
        net->getViewNet()->getUndoList()->end();
        // check if inspect
        if (inspectAfterTransform) {
            // get created element
            auto transformedVehicle = net->getAttributeCarriers()->retrieveDemandElement(vehicleParameters.tag, vehicleParameters.id);
            // inspect it
            net->getViewNet()->getViewParent()->getInspectorFrame()->inspectSingleElement(transformedVehicle);
        }
    }
}


void
GNERouteHandler::transformToFlow(GNEVehicle* originalVehicle) {
    // get pointer to net
    GNENet* net = originalVehicle->getNet();
    // check if transform after creation
    const bool inspectAfterTransform = net->getViewNet()->isAttributeCarrierInspected(originalVehicle);
    // declare route handler
    GNERouteHandler routeHandler("", net, true, false);
    // obtain vehicle parameters
    SUMOVehicleParameter vehicleParameters = *originalVehicle;
    // declare route
    GNEDemandElement* route = nullptr;
    // declare edges
    std::vector<GNEEdge*> edges;
    // obtain edges depending of tag
    if (originalVehicle->getTagProperty().vehicleRoute()) {
        // set route
        route = originalVehicle->getParentDemandElements().back();
        // get route edges
        edges = route->getParentEdges();
    } else if (originalVehicle->getTagProperty().vehicleRouteEmbedded()) {
        // get embedded route edges
        edges = originalVehicle->getChildDemandElements().front()->getParentEdges();
    } else if (originalVehicle->getTagProperty().vehicleEdges()) {
        // just take parent edges (from and to)
        edges = originalVehicle->getParentEdges();
    }
    // only continue if edges are valid
    if (edges.empty()) {
        // declare header
        const std::string header = "Problem transforming to vehicle";
        // declare message
        const std::string message = "Vehicle cannot be transformed. Invalid number of edges";
        // write warning
        WRITE_DEBUG("Opened FXMessageBox " + header);
        // open message box
        FXMessageBox::warning(originalVehicle->getNet()->getViewNet()->getApp(), MBOX_OK, header.c_str(), "%s", message.c_str());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox " + header);
    } else {
        // begin undo-redo operation
        net->getViewNet()->getUndoList()->begin(originalVehicle, "transform " + originalVehicle->getTagStr() + " to " + toString(SUMO_TAG_VEHICLE));
        // first delete vehicle
        net->deleteDemandElement(originalVehicle, net->getViewNet()->getUndoList());
        // check if route has to be deleted
        if (route && route->getChildDemandElements().empty()) {
            net->deleteDemandElement(route, net->getViewNet()->getUndoList());
        }
        // change depart
        if (!GNEAttributeCarrier::getTagProperty(vehicleParameters.tag).isFlow()) {
            // get template flow
            const auto templateFlow = net->getViewNet()->getViewParent()->getVehicleFrame()->getVehicleTagSelector()->getTemplateAC(GNE_TAG_FLOW_ROUTE);
            // set flow parameters
            vehicleParameters.repetitionEnd = vehicleParameters.depart + string2time("3600");
            vehicleParameters.repetitionNumber = GNEAttributeCarrier::parse<int>(templateFlow->getAttribute(SUMO_ATTR_NUMBER));
            vehicleParameters.repetitionOffset = string2time(templateFlow->getAttribute(SUMO_ATTR_PERIOD));
            vehicleParameters.repetitionProbability = GNEAttributeCarrier::parse<double>(templateFlow->getAttribute(SUMO_ATTR_PROB));
            // by default, number and end enabled
            vehicleParameters.parametersSet = GNEAttributeCarrier::parse<int>(templateFlow->getAttribute(GNE_ATTR_FLOWPARAMETERS));
        }
        // change tag in vehicle parameters
        vehicleParameters.tag = SUMO_TAG_FLOW;
        // create flow
        routeHandler.buildFlow(nullptr, vehicleParameters, edges.front()->getID(), edges.back()->getID());
        // end undo-redo operation
        net->getViewNet()->getUndoList()->end();
        // check if inspect
        if (inspectAfterTransform) {
            // get created element
            auto transformedVehicle = net->getAttributeCarriers()->retrieveDemandElement(vehicleParameters.tag, vehicleParameters.id);
            // inspect it
            net->getViewNet()->getViewParent()->getInspectorFrame()->inspectSingleElement(transformedVehicle);
        }
    }
}


void
GNERouteHandler::transformToTripJunctions(GNEVehicle* originalVehicle) {
    // only continue if number of junctions are valid
    if (originalVehicle->getParentJunctions().empty()) {
        // declare header
        const std::string header = "Problem transforming to trip over junctions";
        // declare message
        const std::string message = "Vehicle cannot be transformed. Invalid number of junctions";
        // write warning
        WRITE_DEBUG("Opened FXMessageBox " + header);
        // open message box
        FXMessageBox::warning(originalVehicle->getNet()->getViewNet()->getApp(), MBOX_OK, header.c_str(), "%s", message.c_str());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox " + header);
    } else {
        // get pointer to net
        GNENet* net = originalVehicle->getNet();
        // check if transform after creation
        const bool inspectAfterTransform = net->getViewNet()->isAttributeCarrierInspected(originalVehicle);
        // declare route handler
        GNERouteHandler routeHandler("", net, true, false);
        // obtain vehicle parameters
        SUMOVehicleParameter vehicleParameters = *originalVehicle;
        // begin undo-redo operation
        net->getViewNet()->getUndoList()->begin(originalVehicle, "transform " + originalVehicle->getTagStr() + " to " + toString(GNE_TAG_TRIP_JUNCTIONS));
        // first delete vehicle
        net->deleteDemandElement(originalVehicle, net->getViewNet()->getUndoList());
        // change tag in vehicle parameters
        vehicleParameters.tag = GNE_TAG_TRIP_JUNCTIONS;
        // create trip
        routeHandler.buildTripJunctions(nullptr, vehicleParameters, originalVehicle->getParentJunctions().front()->getID(), originalVehicle->getParentJunctions().back()->getID());
        // end undo-redo operation
        net->getViewNet()->getUndoList()->end();
        // check if inspect
        if (inspectAfterTransform) {
            // get created element
            auto transformedVehicle = net->getAttributeCarriers()->retrieveDemandElement(vehicleParameters.tag, vehicleParameters.id);
            // inspect it
            net->getViewNet()->getViewParent()->getInspectorFrame()->inspectSingleElement(transformedVehicle);
        }
    }
}


void
GNERouteHandler::transformToFlowJunctions(GNEVehicle* originalVehicle) {
    // only continue if number of junctions are valid
    if (originalVehicle->getParentJunctions().empty()) {
        // declare header
        const std::string header = "Problem transforming to flow over junctions";
        // declare message
        const std::string message = "Vehicle cannot be transformed. Invalid number of junctions";
        // write warning
        WRITE_DEBUG("Opened FXMessageBox " + header);
        // open message box
        FXMessageBox::warning(originalVehicle->getNet()->getViewNet()->getApp(), MBOX_OK, header.c_str(), "%s", message.c_str());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox " + header);
    } else {
        // get pointer to net
        GNENet* net = originalVehicle->getNet();
        // check if transform after creation
        const bool inspectAfterTransform = net->getViewNet()->isAttributeCarrierInspected(originalVehicle);
        // declare route handler
        GNERouteHandler routeHandler("", net, true, false);
        // obtain vehicle parameters
        SUMOVehicleParameter vehicleParameters = *originalVehicle;
        // begin undo-redo operation
        net->getViewNet()->getUndoList()->begin(originalVehicle, "transform " + originalVehicle->getTagStr() + " to " + toString(GNE_TAG_FLOW_JUNCTIONS));
        // first delete vehicle
        net->deleteDemandElement(originalVehicle, net->getViewNet()->getUndoList());
        // get template flow
        const auto templateFlow = net->getViewNet()->getViewParent()->getVehicleFrame()->getVehicleTagSelector()->getTemplateAC(GNE_TAG_FLOW_JUNCTIONS);
        // set flow parameters
        vehicleParameters.repetitionEnd = vehicleParameters.depart + string2time("3600");
        vehicleParameters.repetitionNumber = GNEAttributeCarrier::parse<int>(templateFlow->getAttribute(SUMO_ATTR_NUMBER));
        vehicleParameters.repetitionOffset = string2time(templateFlow->getAttribute(SUMO_ATTR_PERIOD));
        vehicleParameters.repetitionProbability = GNEAttributeCarrier::parse<double>(templateFlow->getAttribute(SUMO_ATTR_PROB));
        // by default, number and end enabled
        vehicleParameters.parametersSet = GNEAttributeCarrier::parse<int>(templateFlow->getAttribute(GNE_ATTR_FLOWPARAMETERS));
        // change tag in vehicle parameters
        vehicleParameters.tag = GNE_TAG_FLOW_JUNCTIONS;
        // create flow
        routeHandler.buildFlowJunctions(nullptr, vehicleParameters, originalVehicle->getParentJunctions().front()->getID(), originalVehicle->getParentJunctions().back()->getID());
        // end undo-redo operation
        net->getViewNet()->getUndoList()->end();
        // check if inspect
        if (inspectAfterTransform) {
            // get created element
            auto transformedVehicle = net->getAttributeCarriers()->retrieveDemandElement(vehicleParameters.tag, vehicleParameters.id);
            // inspect it
            net->getViewNet()->getViewParent()->getInspectorFrame()->inspectSingleElement(transformedVehicle);
        }
    }
}


void
GNERouteHandler::transformToTripTAZs(GNEVehicle* originalVehicle) {
    // only continue if number of junctions are valid
    if (originalVehicle->getParentAdditionals().empty()) {
        // declare header
        const std::string header = "Problem transforming to trip over TAZs";
        // declare message
        const std::string message = "Vehicle cannot be transformed. Invalid number of TAZs";
        // write warning
        WRITE_DEBUG("Opened FXMessageBox " + header);
        // open message box
        FXMessageBox::warning(originalVehicle->getNet()->getViewNet()->getApp(), MBOX_OK, header.c_str(), "%s", message.c_str());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox " + header);
    } else {
        // get pointer to net
        GNENet* net = originalVehicle->getNet();
        // check if transform after creation
        const bool inspectAfterTransform = net->getViewNet()->isAttributeCarrierInspected(originalVehicle);
        // declare route handler
        GNERouteHandler routeHandler("", net, true, false);
        // obtain vehicle parameters
        SUMOVehicleParameter vehicleParameters = *originalVehicle;
        // begin undo-redo operation
        net->getViewNet()->getUndoList()->begin(originalVehicle, "transform " + originalVehicle->getTagStr() + " to " + toString(GNE_TAG_TRIP_TAZS));
        // first delete vehicle
        net->deleteDemandElement(originalVehicle, net->getViewNet()->getUndoList());
        // change tag in vehicle parameters
        vehicleParameters.tag = GNE_TAG_TRIP_TAZS;
        // create trip
        routeHandler.buildTripTAZs(nullptr, vehicleParameters, originalVehicle->getParentAdditionals().front()->getID(), originalVehicle->getParentAdditionals().back()->getID());
        // end undo-redo operation
        net->getViewNet()->getUndoList()->end();
        // check if inspect
        if (inspectAfterTransform) {
            // get created element
            auto transformedVehicle = net->getAttributeCarriers()->retrieveDemandElement(vehicleParameters.tag, vehicleParameters.id);
            // inspect it
            net->getViewNet()->getViewParent()->getInspectorFrame()->inspectSingleElement(transformedVehicle);
        }
    }
}


void
GNERouteHandler::transformToFlowTAZs(GNEVehicle* originalVehicle) {
    // only continue if number of junctions are valid
    if (originalVehicle->getParentAdditionals().empty()) {
        // declare header
        const std::string header = "Problem transforming to flow over TAZs";
        // declare message
        const std::string message = "Vehicle cannot be transformed. Invalid number of TAZs";
        // write warning
        WRITE_DEBUG("Opened FXMessageBox " + header);
        // open message box
        FXMessageBox::warning(originalVehicle->getNet()->getViewNet()->getApp(), MBOX_OK, header.c_str(), "%s", message.c_str());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox " + header);
    } else {
        // get pointer to net
        GNENet* net = originalVehicle->getNet();
        // check if transform after creation
        const bool inspectAfterTransform = net->getViewNet()->isAttributeCarrierInspected(originalVehicle);
        // declare route handler
        GNERouteHandler routeHandler("", net, true, false);
        // obtain vehicle parameters
        SUMOVehicleParameter vehicleParameters = *originalVehicle;
        // begin undo-redo operation
        net->getViewNet()->getUndoList()->begin(originalVehicle, "transform " + originalVehicle->getTagStr() + " to " + toString(GNE_TAG_FLOW_TAZS));
        // first delete vehicle
        net->deleteDemandElement(originalVehicle, net->getViewNet()->getUndoList());
        // get template flow
        const auto templateFlow = net->getViewNet()->getViewParent()->getVehicleFrame()->getVehicleTagSelector()->getTemplateAC(GNE_TAG_FLOW_TAZS);
        // set flow parameters
        vehicleParameters.repetitionEnd = vehicleParameters.depart + string2time("3600");
        vehicleParameters.repetitionNumber = GNEAttributeCarrier::parse<int>(templateFlow->getAttribute(SUMO_ATTR_NUMBER));
        vehicleParameters.repetitionOffset = string2time(templateFlow->getAttribute(SUMO_ATTR_PERIOD));
        vehicleParameters.repetitionProbability = GNEAttributeCarrier::parse<double>(templateFlow->getAttribute(SUMO_ATTR_PROB));
        // by default, number and end enabled
        vehicleParameters.parametersSet = GNEAttributeCarrier::parse<int>(templateFlow->getAttribute(GNE_ATTR_FLOWPARAMETERS));
        // change tag in vehicle parameters
        vehicleParameters.tag = GNE_TAG_FLOW_TAZS;
        // create flow
        routeHandler.buildFlowTAZs(nullptr, vehicleParameters, originalVehicle->getParentAdditionals().front()->getID(), originalVehicle->getParentAdditionals().back()->getID());
        // end undo-redo operation
        net->getViewNet()->getUndoList()->end();
        // check if inspect
        if (inspectAfterTransform) {
            // get created element
            auto transformedVehicle = net->getAttributeCarriers()->retrieveDemandElement(vehicleParameters.tag, vehicleParameters.id);
            // inspect it
            net->getViewNet()->getViewParent()->getInspectorFrame()->inspectSingleElement(transformedVehicle);
        }
    }
}


void
GNERouteHandler::transformToPerson(GNEPerson* originalPerson) {
    // get pointer to net
    GNENet* net = originalPerson->getNet();
    // check if transform after creation
    const bool inspectAfterTransform = net->getViewNet()->isAttributeCarrierInspected(originalPerson);
    // declare route handler
    GNERouteHandler routeHandler("", net, true, false);
    // obtain person parameters
    SUMOVehicleParameter personParameters = *originalPerson;
    // save ID
    const auto ID = personParameters.id;
    // set dummy ID
    personParameters.id = "%dummyID%";
    // begin undo-redo operation
    net->getViewNet()->getUndoList()->begin(originalPerson, "transform " + originalPerson->getTagStr() + " to " + toString(SUMO_TAG_PERSON));
    // create personFlow and get it
    routeHandler.buildPerson(nullptr, personParameters);
    auto newPerson = net->getAttributeCarriers()->retrieveDemandElement(SUMO_TAG_PERSON, "%dummyID%");
    // duplicate plans in new person
    for (const auto& personPlan : originalPerson->getChildDemandElements()) {
        routeHandler.duplicatePlan(personPlan, newPerson);
    }
    // delete original person plan
    net->deleteDemandElement(originalPerson, net->getViewNet()->getUndoList());
    // restore ID of new person plan
    newPerson->setAttribute(SUMO_ATTR_ID, ID, net->getViewNet()->getUndoList());
    // finish undoList
    net->getViewNet()->getUndoList()->end();
    // check if inspect
    if (inspectAfterTransform) {
        net->getViewNet()->getViewParent()->getInspectorFrame()->inspectSingleElement(newPerson);
    }
}


void
GNERouteHandler::transformToPersonFlow(GNEPerson* originalPerson) {
    // get pointer to net
    GNENet* net = originalPerson->getNet();
    // check if transform after creation
    const bool inspectAfterTransform = net->getViewNet()->isAttributeCarrierInspected(originalPerson);
    // declare route handler
    GNERouteHandler routeHandler("", net, true, false);
    // obtain person parameters
    SUMOVehicleParameter personParameters = *originalPerson;
    // get person plans
    const auto personPlans = originalPerson->getChildDemandElements();
    // save ID
    const auto ID = personParameters.id;
    // set dummy ID
    personParameters.id = "%dummyID%";
    // begin undo-redo operation
    net->getViewNet()->getUndoList()->begin(originalPerson, "transform " + originalPerson->getTagStr() + " to " + toString(SUMO_TAG_PERSONFLOW));
    // create personFlow and get it
    routeHandler.buildPersonFlow(nullptr, personParameters);
    auto newPerson = net->getAttributeCarriers()->retrieveDemandElement(SUMO_TAG_PERSONFLOW, "%dummyID%");
    // move all person plans to new person
    for (const auto& personPlan : personPlans) {
        routeHandler.duplicatePlan(personPlan, newPerson);
    }
    // delete original person plan
    net->deleteDemandElement(originalPerson, net->getViewNet()->getUndoList());
    // restore ID of new person plan
    newPerson->setAttribute(SUMO_ATTR_ID, ID, net->getViewNet()->getUndoList());
    // enable attributes
    newPerson->enableAttribute(SUMO_ATTR_END, net->getViewNet()->getUndoList());
    newPerson->enableAttribute(SUMO_ATTR_PERSONSPERHOUR, net->getViewNet()->getUndoList());
    // finish undoList
    net->getViewNet()->getUndoList()->end();
    // check if inspect
    if (inspectAfterTransform) {
        net->getViewNet()->getViewParent()->getInspectorFrame()->inspectSingleElement(newPerson);
    }
}


void
GNERouteHandler::transformToContainer(GNEContainer* originalContainer) {
    // get pointer to net
    GNENet* net = originalContainer->getNet();
    // check if transform after creation
    const bool inspectAfterTransform = net->getViewNet()->isAttributeCarrierInspected(originalContainer);
    // declare route handler
    GNERouteHandler routeHandler("", net, true, false);
    // obtain container parameters
    SUMOVehicleParameter containerParameters = *originalContainer;
    // get container plans
    const auto containerPlans = originalContainer->getChildDemandElements();
    // save ID
    const auto ID = containerParameters.id;
    // set dummy ID
    containerParameters.id = "%dummyID%";
    // begin undo-redo operation
    net->getViewNet()->getUndoList()->begin(originalContainer, "transform " + originalContainer->getTagStr() + " to " + toString(SUMO_TAG_CONTAINER));
    // create containerFlow
    routeHandler.buildContainer(nullptr, containerParameters);
    // move all container plans to new container
    for (const auto& containerPlan : containerPlans) {
        containerPlan->setAttribute(GNE_ATTR_PARENT, "%dummyID%", net->getViewNet()->getUndoList());
    }
    // delete original container plan
    net->deleteDemandElement(originalContainer, net->getViewNet()->getUndoList());
    // restore ID of new container plan
    auto newContainer = net->getAttributeCarriers()->retrieveDemandElement(SUMO_TAG_CONTAINER, "%dummyID%");
    newContainer->setAttribute(SUMO_ATTR_ID, ID, net->getViewNet()->getUndoList());
    // finish undoList
    net->getViewNet()->getUndoList()->end();
    // check if inspect
    if (inspectAfterTransform) {
        net->getViewNet()->getViewParent()->getInspectorFrame()->inspectSingleElement(newContainer);
    }
}


void
GNERouteHandler::transformToContainerFlow(GNEContainer* originalContainer) {
    // get pointer to net
    GNENet* net = originalContainer->getNet();
    // check if transform after creation
    const bool inspectAfterTransform = net->getViewNet()->isAttributeCarrierInspected(originalContainer);
    // declare route handler
    GNERouteHandler routeHandler("", net, true, false);
    // obtain container parameters
    SUMOVehicleParameter containerParameters = *originalContainer;
    // get container plans
    const auto containerPlans = originalContainer->getChildDemandElements();
    // save ID
    const auto ID = containerParameters.id;
    // set dummy ID
    containerParameters.id = "%dummyID%";
    // begin undo-redo operation
    net->getViewNet()->getUndoList()->begin(originalContainer, "transform " + originalContainer->getTagStr() + " to " + toString(SUMO_TAG_CONTAINERFLOW));
    // create containerFlow
    routeHandler.buildContainerFlow(nullptr, containerParameters);
    // move all container plans to new container
    for (const auto& containerPlan : containerPlans) {
        containerPlan->setAttribute(GNE_ATTR_PARENT, "%dummyID%", net->getViewNet()->getUndoList());
    }
    // delete original container plan
    net->deleteDemandElement(originalContainer, net->getViewNet()->getUndoList());
    // restore ID of new container plan
    auto newContainer = net->getAttributeCarriers()->retrieveDemandElement(SUMO_TAG_CONTAINERFLOW, "%dummyID%");
    newContainer->setAttribute(SUMO_ATTR_ID, ID, net->getViewNet()->getUndoList());
    // enable attributes
    newContainer->enableAttribute(SUMO_ATTR_END, net->getViewNet()->getUndoList());
    newContainer->enableAttribute(SUMO_ATTR_CONTAINERSPERHOUR, net->getViewNet()->getUndoList());
    // finish undoList
    net->getViewNet()->getUndoList()->end();
    // check if inspect
    if (inspectAfterTransform) {
        net->getViewNet()->getViewParent()->getInspectorFrame()->inspectSingleElement(newContainer);
    }
}


bool
GNERouteHandler::canReverse(const GNEDemandElement* element) {
    // continue depending of element
    if (element->getTagProperty().getTag() == SUMO_TAG_ROUTE) {
        return canReverse(element->getNet(), SVC_PEDESTRIAN, element->getParentEdges());
    } else if (element->getTagProperty().vehicleRoute()) {
        return canReverse(element->getNet(), element->getVClass(), element->getParentDemandElements().at(1)->getParentEdges());
    } else if (element->getTagProperty().vehicleRouteEmbedded()) {
        return canReverse(element->getNet(), element->getVClass(), element->getChildDemandElements().front()->getParentEdges());
    } else if (element->getTagProperty().vehicleEdges()) {
        return canReverse(element->getNet(), element->getVClass(), element->getParentEdges());
    } else if (element->getTagProperty().vehicleJunctions()) {
        return (element->getNet()->getPathManager()->getPathCalculator()->calculateDijkstraPath(element->getVClass(),
                element->getParentJunctions().back(), element->getParentJunctions().front()).size() > 0);
    } else if (element->getTagProperty().vehicleTAZs()) {
        return true;
    } else {
        return false;
    }
}


bool
GNERouteHandler::canReverse(GNENet* net, SUMOVehicleClass vClass, const std::vector<GNEEdge*>& edges) {
    if (edges.empty()) {
        return false;
    } else {
        // obtain opposite edges
        std::vector<GNEEdge*> reverseEdges;
        for (const auto& edge : edges) {
            const auto oppositeEdges = edge->getOppositeEdges();
            // stop if there isn't opposite edges for the current edge
            if (oppositeEdges.empty()) {
                return false;
            } else {
                reverseEdges.push_back(oppositeEdges.front());
            }
        }
        // reverse edges
        std::reverse(reverseEdges.begin(), reverseEdges.end());
        // now check if exist a path
        return (net->getPathManager()->getPathCalculator()->calculateDijkstraPath(vClass, edges).size() > 0);
    }
}


void
GNERouteHandler::reverse(GNEDemandElement* element) {
    // get undo list
    auto undoList = element->getNet()->getViewNet()->getUndoList();
    // continue depending of element
    if (element->getTagProperty().vehicleRoute()) {
        // reverse parent route
        reverse(element->getParentDemandElements().at(1));
    } else if (element->getTagProperty().vehicleRouteEmbedded()) {
        // reverse embedded route
        reverse(element->getChildDemandElements().front());
    } else if (element->getTagProperty().vehicleJunctions()) {
        // get from to junctions
        const auto fromJunction = element->getAttribute(SUMO_ATTR_FROM_JUNCTION);
        const auto toJunction = element->getAttribute(SUMO_ATTR_TO_JUNCTION);
        // swap both attributes
        element->setAttribute(SUMO_ATTR_FROM_JUNCTION, toJunction, undoList);
        element->setAttribute(SUMO_ATTR_TO_JUNCTION, fromJunction, undoList);
    } else if (element->getTagProperty().vehicleTAZs()) {
        // get from to TAZs
        const auto fromTAZ = element->getAttribute(SUMO_ATTR_FROM_TAZ);
        const auto toTAZ = element->getAttribute(SUMO_ATTR_TO_TAZ);
        // swap both attributes
        element->setAttribute(SUMO_ATTR_FROM_TAZ, toTAZ, undoList);
        element->setAttribute(SUMO_ATTR_TO_TAZ, fromTAZ, undoList);
    } else {
        // extract and reverse opposite edges
        std::vector<GNEEdge*> oppositeEdges;
        for (const auto& edge : element->getParentEdges()) {
            oppositeEdges.push_back(edge->getOppositeEdges().front());
        }
        std::reverse(oppositeEdges.begin(), oppositeEdges.end());
        if (element->isRoute()) {
            element->setAttribute(SUMO_ATTR_EDGES, GNEAttributeCarrier::parseIDs(oppositeEdges), undoList);
        } else {
            // set from and to
            element->setAttribute(SUMO_ATTR_FROM, oppositeEdges.front()->getID(), undoList);
            element->setAttribute(SUMO_ATTR_TO, oppositeEdges.back()->getID(), undoList);
            // check if add via attribute
            oppositeEdges.erase(oppositeEdges.begin());
            oppositeEdges.pop_back();
            if (oppositeEdges.size() > 0) {
                element->setAttribute(SUMO_ATTR_VIA, GNEAttributeCarrier::parseIDs(oppositeEdges), undoList);
            }
        }
    }
}


void
GNERouteHandler::addReverse(GNEDemandElement* element) {
    GNEDemandElement* elementCopy = nullptr;
    if (element->getTagProperty().getTag() == SUMO_TAG_ROUTE) {
        // make a copy of the route and reverse
        elementCopy = GNERoute::copyRoute(dynamic_cast<GNERoute*>(element));
    } else if (element->getTagProperty().isVehicle()) {
        // make a copy of the vehicle
        elementCopy = GNEVehicle::copyVehicle(dynamic_cast<GNEVehicle*>(element));
    }
    // reverse copied element
    reverse(elementCopy);
}

// ===========================================================================
// protected
// ===========================================================================

GNEJunction*
GNERouteHandler::parseJunction(const SumoXMLTag tag, const std::string& junctionID) {
    GNEJunction* junction = myNet->getAttributeCarriers()->retrieveJunction(junctionID, false);
    // empty junctions aren't allowed. If junction is empty, write error, clear junctions and stop
    if (junction == nullptr) {
        writeError(TLF("Could not build % in netedit", toString(tag)) + std::string("; ") + TLF("% doesn't exist.", toString(SUMO_TAG_JUNCTION)));
    }
    return junction;
}


GNEAdditional*
GNERouteHandler::parseTAZ(const SumoXMLTag tag, const std::string& TAZID) {
    GNEAdditional* TAZ = myNet->getAttributeCarriers()->retrieveAdditional(SUMO_TAG_TAZ, TAZID, false);
    // empty TAZs aren't allowed. If TAZ is empty, write error, clear TAZs and stop
    if (TAZ == nullptr) {
        writeError(TLF("Could not build % in netedit", toString(tag)) + std::string("; ") + TLF("% doesn't exist.", toString(SUMO_TAG_TAZ)));
    }
    return TAZ;
}


GNEEdge*
GNERouteHandler::parseEdge(const SumoXMLTag tag, const std::string& edgeID,
                           const CommonXMLStructure::SumoBaseObject* sumoBaseObject,
                           const bool firstEdge) {
    GNEEdge* edge = nullptr;
    if (edgeID.empty()) {
        if (sumoBaseObject->getSumoBaseObjectChildren().size() > 0) {
            const auto frontTag = sumoBaseObject->getSumoBaseObjectChildren().front()->getTag();
            const auto backTag = sumoBaseObject->getSumoBaseObjectChildren().back()->getTag();
            if (firstEdge && ((frontTag == SUMO_TAG_STOP) || (frontTag == SUMO_TAG_TRAIN_STOP) ||
                (frontTag == SUMO_TAG_CONTAINER_STOP) || (frontTag == SUMO_TAG_CHARGING_STATION) ||
                (frontTag == SUMO_TAG_PARKING_AREA))) {
                edge = parseStopEdge(sumoBaseObject->getSumoBaseObjectChildren().front());
            } else if (!firstEdge && ((backTag == SUMO_TAG_STOP) || (backTag == SUMO_TAG_TRAIN_STOP) ||
                (backTag == SUMO_TAG_CONTAINER_STOP) || (backTag == SUMO_TAG_CHARGING_STATION) ||
                (backTag == SUMO_TAG_PARKING_AREA))) {
                edge = parseStopEdge(sumoBaseObject->getSumoBaseObjectChildren().back());
            }
        }
    } else {
        edge = myNet->getAttributeCarriers()->retrieveEdge(edgeID, false);
    }
    // write info if edge doesn't exist
    if (edge == nullptr) {
        writeError(TLF("Could not build % in netedit", toString(tag)) + std::string("; ") + TLF("% doesn't exist.", toString(SUMO_TAG_EDGE)));
    }
    return edge;
}


GNEEdge*
GNERouteHandler::parseStopEdge(const CommonXMLStructure::SumoBaseObject* sumoBaseObject) const {
    if (sumoBaseObject->hasStringAttribute(SUMO_ATTR_EDGE)) {
        return myNet->getAttributeCarriers()->retrieveEdge(sumoBaseObject->getStringAttribute(SUMO_ATTR_EDGE), false);
    } else if (sumoBaseObject->hasStringAttribute(SUMO_ATTR_LANE)) {
        return parseEdgeFromLaneID(sumoBaseObject->getStringAttribute(SUMO_ATTR_LANE));
    } else if (sumoBaseObject->hasStringAttribute(SUMO_ATTR_BUS_STOP)) {
        const auto busStop = myNet->getAttributeCarriers()->retrieveAdditional(SUMO_TAG_BUS_STOP, sumoBaseObject->getStringAttribute(SUMO_ATTR_BUS_STOP), false);
        const auto trainStop = myNet->getAttributeCarriers()->retrieveAdditional(SUMO_TAG_TRAIN_STOP, sumoBaseObject->getStringAttribute(SUMO_ATTR_BUS_STOP), false);
        if (busStop != nullptr) {
            return busStop->getParentLanes().front()->getParentEdge();
        } else if (trainStop != nullptr) {
            return trainStop->getParentLanes().front()->getParentEdge();
        } else {
            return nullptr;
        }
    } else if (sumoBaseObject->hasStringAttribute(SUMO_ATTR_TRAIN_STOP)) {
        const auto busStop = myNet->getAttributeCarriers()->retrieveAdditional(SUMO_TAG_BUS_STOP, sumoBaseObject->getStringAttribute(SUMO_ATTR_TRAIN_STOP), false);
        const auto trainStop = myNet->getAttributeCarriers()->retrieveAdditional(SUMO_TAG_TRAIN_STOP, sumoBaseObject->getStringAttribute(SUMO_ATTR_TRAIN_STOP), false);
        if (busStop != nullptr) {
            return busStop->getParentLanes().front()->getParentEdge();
        } else if (trainStop != nullptr) {
            return trainStop->getParentLanes().front()->getParentEdge();
        } else {
            return nullptr;
        }
    } else if (sumoBaseObject->hasStringAttribute(SUMO_ATTR_CONTAINER_STOP)) {
        const auto containerStop = myNet->getAttributeCarriers()->retrieveAdditional(SUMO_TAG_CONTAINER_STOP, sumoBaseObject->getStringAttribute(SUMO_ATTR_CONTAINER_STOP), false);
        if (containerStop != nullptr) {
            return containerStop->getParentLanes().front()->getParentEdge();
        } else {
            return nullptr;
        }

    } else if (sumoBaseObject->hasStringAttribute(SUMO_ATTR_CHARGING_STATION)) {
        const auto containerStop = myNet->getAttributeCarriers()->retrieveAdditional(SUMO_TAG_CHARGING_STATION, sumoBaseObject->getStringAttribute(SUMO_ATTR_CHARGING_STATION), false);
        if (containerStop != nullptr) {
            return containerStop->getParentLanes().front()->getParentEdge();
        } else {
            return nullptr;
        }

    } else if (sumoBaseObject->hasStringAttribute(SUMO_ATTR_PARKING_AREA)) {
        const auto parkingArea = myNet->getAttributeCarriers()->retrieveAdditional(SUMO_TAG_PARKING_AREA, sumoBaseObject->getStringAttribute(SUMO_ATTR_PARKING_AREA), false);
        if (parkingArea != nullptr) {
            return parkingArea->getParentLanes().front()->getParentEdge();
        } else {
            return nullptr;
        }
    } else {
        return nullptr;
    }
}


GNEEdge*
GNERouteHandler::parseEdgeFromLaneID(const std::string &laneID) const {
    std::string edgeID = laneID;
    for (int i = ((int)laneID.size() - 1); (i >= 0) && (laneID[i+1] != '_'); i--) {
        edgeID.pop_back();
    }
    return myNet->getAttributeCarriers()->retrieveEdge(edgeID, false);
}


std::vector<GNEEdge*>
GNERouteHandler::parseEdges(const SumoXMLTag tag, const std::vector<std::string>& edgeIDs) {
    std::vector<GNEEdge*> edges;
    for (const auto& edgeID : edgeIDs) {
        GNEEdge* edge = myNet->getAttributeCarriers()->retrieveEdge(edgeID, false);
        // empty edges aren't allowed. If edge is empty, write error, clear edges and stop
        if (edge == nullptr) {
            writeError(TLF("Could not build % in netedit", toString(tag)) + std::string("; ") + TLF("% doesn't exist.", toString(SUMO_TAG_EDGE)));
            edges.clear();
            return edges;
        } else {
            edges.push_back(edge);
        }
    }
    return edges;
}


GNEDemandElement*
GNERouteHandler::getType(const std::string& id) const {
    GNEDemandElement* type = myNet->getAttributeCarriers()->retrieveDemandElement(SUMO_TAG_VTYPE, id, false);
    if (type == nullptr) {
        return myNet->getAttributeCarriers()->retrieveDemandElement(SUMO_TAG_VTYPE_DISTRIBUTION, id, false);
    } else {
        return type;
    }
}


GNEDemandElement*
GNERouteHandler::getPersonParent(const CommonXMLStructure::SumoBaseObject* sumoBaseObject) const {
    // check that sumoBaseObject has parent
    if (sumoBaseObject->getParentSumoBaseObject() == nullptr) {
        return nullptr;
    }
    if ((sumoBaseObject->getParentSumoBaseObject()->getTag() != SUMO_TAG_PERSON) &&
            (sumoBaseObject->getParentSumoBaseObject()->getTag() != SUMO_TAG_PERSONFLOW)) {
        return nullptr;
    }
    // try it with person
    GNEDemandElement* personParent = myNet->getAttributeCarriers()->retrieveDemandElement(SUMO_TAG_PERSON, sumoBaseObject->getParentSumoBaseObject()->getStringAttribute(SUMO_ATTR_ID), false);
    // if empty, try it with personFlow
    if (personParent == nullptr) {
        return myNet->getAttributeCarriers()->retrieveDemandElement(SUMO_TAG_PERSONFLOW, sumoBaseObject->getParentSumoBaseObject()->getStringAttribute(SUMO_ATTR_ID), false);
    } else {
        return personParent;
    }
}


GNEDemandElement*
GNERouteHandler::getContainerParent(const CommonXMLStructure::SumoBaseObject* sumoBaseObject) const {
    // check that sumoBaseObject has parent
    if (sumoBaseObject->getParentSumoBaseObject() == nullptr) {
        return nullptr;
    }
    if ((sumoBaseObject->getParentSumoBaseObject()->getTag() != SUMO_TAG_CONTAINER) &&
            (sumoBaseObject->getParentSumoBaseObject()->getTag() != SUMO_TAG_CONTAINERFLOW)) {
        return nullptr;
    }
    // try it with container
    GNEDemandElement* containerParent = myNet->getAttributeCarriers()->retrieveDemandElement(SUMO_TAG_CONTAINER, sumoBaseObject->getParentSumoBaseObject()->getStringAttribute(SUMO_ATTR_ID), false);
    // if empty, try it with containerFlow
    if (containerParent == nullptr) {
        return myNet->getAttributeCarriers()->retrieveDemandElement(SUMO_TAG_CONTAINERFLOW, sumoBaseObject->getParentSumoBaseObject()->getStringAttribute(SUMO_ATTR_ID), false);
    } else {
        return containerParent;
    }
}


bool
GNERouteHandler::getDistributionElements(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, SumoXMLTag distributionElementTag,
        const std::vector<std::string>& distributionElementIDs, const std::vector<double>& probabilities,
        std::vector<const GNEDemandElement*>& elements) {
    // get distribution tag and ID
    std::string distributionTag = toString(sumoBaseObject->getTag());
    std::string distributionID = sumoBaseObject->getStringAttribute(SUMO_ATTR_ID);
    // first parse vType IDs
    for (const auto& distributionElementID : distributionElementIDs) {
        auto distributionElement = myNet->getAttributeCarriers()->retrieveDemandElement(distributionElementTag, distributionElementID, false);
        if (distributionElement) {
            elements.push_back(distributionElement);
        } else {
            writeError(TLF("% with id '%' doesn't exist in % '%'", toString(distributionElementTag), distributionElementID, distributionTag, distributionID));
            return false;

        }
    }
    // check probabilities
    for (const auto& probability : probabilities) {
        if (probability < 0) {
            writeError(TLF("invalid probability % in % '%'", toString(probability), distributionTag, distributionID));
            return false;
        }
    }
    // check that number of elements and probabilities is the same
    if (elements.size() != probabilities.size()) {
        writeError(TLF("Invalid type distribution probabilities in % '%'. Must have the same number of elements", distributionTag, distributionID));
        return false;
    } else {
        return true;
    }
}


bool
GNERouteHandler::checkDuplicatedDemandElement(const SumoXMLTag tag, const std::string& id) {
    // retrieve demand element
    auto demandElement = myNet->getAttributeCarriers()->retrieveDemandElement(tag, id, false);
    // if demand exist, check if overwrite (delete)
    if (demandElement) {
        if (!myAllowUndoRedo) {
            // only overwrite if allow undo-redo
            return false;
        } else if (myOverwrite) {
            // update demand to overwrite
            myDemandToOverwrite = demandElement;
            return true;
        } else {
            // duplicated demand
            return false;
        }
    } else {
        // demand with these id doesn't exist, then all ok
        return true;
    }
}


void
GNERouteHandler::overwriteDemandElement() {
    if (myDemandToOverwrite) {
        // remove element
        myNet->deleteDemandElement(myDemandToOverwrite, myNet->getViewNet()->getUndoList());
        // reset pointer
        myDemandToOverwrite = nullptr;
    }
}


void
GNERouteHandler::writeErrorInvalidNegativeValue(const SumoXMLTag tag, const SumoXMLAttr attribute) {
    writeError(TLF("Could not build % in netedit", toString(tag)) + std::string("; ") + TLF("Attribute % cannot be negative.", toString(attribute)));
}

/****************************************************************************/
