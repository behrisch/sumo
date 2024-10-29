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
/// @file    GNEVehicle.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Jan 2019
///
// Representation of vehicles in netedit
/****************************************************************************/
#include <microsim/devices/MSDevice_BTreceiver.h>
#include <netedit/GNENet.h>
#include <netedit/GNEUndoList.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNEViewParent.h>
#include <netedit/changes/GNEChange_Attribute.h>
#include <netedit/changes/GNEChange_DemandElement.h>
#include <netedit/frames/common/GNEMoveFrame.h>
#include <netedit/frames/demand/GNETypeFrame.h>
#include <utils/gui/div/GLHelper.h>
#include <utils/gui/div/GUIBaseVehicleHelper.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/xml/NamespaceIDs.h>

#include "GNEVehicle.h"
#include "GNERoute.h"

// ===========================================================================
// FOX callback mapping
// ===========================================================================
FXDEFMAP(GNEVehicle::GNESingleVehiclePopupMenu) GNESingleVehiclePopupMenuMap[] = {
    FXMAPFUNCS(SEL_COMMAND,  MID_GNE_VEHICLE_TRANSFORM_TRIP, MID_GNE_VEHICLE_TRANSFORM_FLOW_TAZS, GNEVehicle::GNESingleVehiclePopupMenu::onCmdTransform),
};

FXDEFMAP(GNEVehicle::GNESelectedVehiclesPopupMenu) GNESelectedVehiclesPopupMenuMap[] = {
    FXMAPFUNCS(SEL_COMMAND,  MID_GNE_VEHICLE_TRANSFORM_TRIP, MID_GNE_VEHICLE_TRANSFORM_FLOW_TAZS, GNEVehicle::GNESelectedVehiclesPopupMenu::onCmdTransform),
};

// Object implementation
FXIMPLEMENT(GNEVehicle::GNESingleVehiclePopupMenu,      GUIGLObjectPopupMenu,   GNESingleVehiclePopupMenuMap,       ARRAYNUMBER(GNESingleVehiclePopupMenuMap))
FXIMPLEMENT(GNEVehicle::GNESelectedVehiclesPopupMenu,   GUIGLObjectPopupMenu,   GNESelectedVehiclesPopupMenuMap,    ARRAYNUMBER(GNESelectedVehiclesPopupMenuMap))

// ===========================================================================
// static definitions
// ===========================================================================
const double GNEVehicle::myArrivalPositionDiameter = SUMO_const_halfLaneWidth * 0.5;

// ===========================================================================
// GNEVehicle::GNESingleVehiclePopupMenu
// ===========================================================================

GNEVehicle::GNESingleVehiclePopupMenu::GNESingleVehiclePopupMenu(GNEVehicle* vehicle, GUIMainWindow& app, GUISUMOAbstractView& parent) :
    GUIGLObjectPopupMenu(app, parent, *vehicle),
    myVehicle(vehicle) {
    // build header
    myVehicle->buildPopupHeader(this, app);
    // build menu command for center button and copy cursor position to clipboard
    myVehicle->buildCenterPopupEntry(this);
    myVehicle->buildPositionCopyEntry(this, app);
    // build menu commands for names
    GUIDesigns::buildFXMenuCommand(this, ("Copy " + myVehicle->getTagStr() + " name to clipboard").c_str(), nullptr, this, MID_COPY_NAME);
    GUIDesigns::buildFXMenuCommand(this, ("Copy " + myVehicle->getTagStr() + " typed name to clipboard").c_str(), nullptr, this, MID_COPY_TYPED_NAME);
    new FXMenuSeparator(this);
    // build selection and show parameters menu
    myVehicle->getNet()->getViewNet()->buildSelectionACPopupEntry(this, myVehicle);
    myVehicle->buildShowParamsPopupEntry(this);
    // route length
    vehicle->buildMenuCommandRouteLength(this);
    // add transform functions only in demand mode
    if (myVehicle->getNet()->getViewNet()->getEditModes().isCurrentSupermodeDemand()) {
        // add reverse
        vehicle->buildMenuAddReverse(this);
        // continue depending of type
        if (myVehicle->getTagProperty().vehicleJunctions()) {
            // create menu pane for transform operations
            FXMenuPane* transformOperation = new FXMenuPane(this);
            this->insertMenuPaneChild(transformOperation);
            new FXMenuCascade(this, TL("transform to"), nullptr, transformOperation);
            // Create menu commands for all transform
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("Trip (over junctions)"),
                                           GUIIconSubSys::getIcon(GUIIcon::TRIP_JUNCTIONS), this, MID_GNE_VEHICLE_TRANSFORM_TRIP_JUNCTIONS,
                                           (myVehicle->getTagProperty().getTag() == GNE_TAG_TRIP_JUNCTIONS));
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("Flow (over junctions)"),
                                           GUIIconSubSys::getIcon(GUIIcon::FLOW_JUNCTIONS), this, MID_GNE_VEHICLE_TRANSFORM_FLOW_JUNCTIONS,
                                           (myVehicle->getTagProperty().getTag() == GNE_TAG_FLOW_JUNCTIONS));
        } else if (myVehicle->getTagProperty().vehicleTAZs()) {
            // create menu pane for transform operations
            FXMenuPane* transformOperation = new FXMenuPane(this);
            this->insertMenuPaneChild(transformOperation);
            new FXMenuCascade(this, TL("transform to"), nullptr, transformOperation);
            // Create menu commands for all transform
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("Trip (over TAZs)"),
                                           GUIIconSubSys::getIcon(GUIIcon::TRIP_TAZS), this, MID_GNE_VEHICLE_TRANSFORM_TRIP_TAZS,
                                           (myVehicle->getTagProperty().getTag() == GNE_TAG_TRIP_TAZS));
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("Flow (over TAZs)"),
                                           GUIIconSubSys::getIcon(GUIIcon::FLOW_TAZS), this, MID_GNE_VEHICLE_TRANSFORM_FLOW_TAZS,
                                           (myVehicle->getTagProperty().getTag() == GNE_TAG_FLOW_TAZS));
        } else {
            // create menu pane for transform operations
            FXMenuPane* transformOperation = new FXMenuPane(this);
            this->insertMenuPaneChild(transformOperation);
            new FXMenuCascade(this, TL("transform to"), nullptr, transformOperation);
            // Create menu commands for all transform
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("Vehicle"),
                                           GUIIconSubSys::getIcon(GUIIcon::VEHICLE), this, MID_GNE_VEHICLE_TRANSFORM_VEHICLE,
                                           (myVehicle->getTagProperty().getTag() == SUMO_TAG_VEHICLE));
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("Vehicle (embedded route)"),
                                           GUIIconSubSys::getIcon(GUIIcon::VEHICLE), this, MID_GNE_VEHICLE_TRANSFORM_VEHICLE_EMBEDDED,
                                           (myVehicle->getTagProperty().getTag() == GNE_TAG_VEHICLE_WITHROUTE));
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("RouteFlow"),
                                           GUIIconSubSys::getIcon(GUIIcon::ROUTEFLOW), this, MID_GNE_VEHICLE_TRANSFORM_ROUTEFLOW,
                                           (myVehicle->getTagProperty().getTag() == GNE_TAG_FLOW_ROUTE));
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("RouteFlow (embedded route)"),
                                           GUIIconSubSys::getIcon(GUIIcon::ROUTEFLOW), this, MID_GNE_VEHICLE_TRANSFORM_FLOW_EMBEDDED,
                                           (myVehicle->getTagProperty().getTag() == GNE_TAG_FLOW_WITHROUTE));
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("Trip"),
                                           GUIIconSubSys::getIcon(GUIIcon::TRIP), this, MID_GNE_VEHICLE_TRANSFORM_TRIP,
                                           (myVehicle->getTagProperty().getTag() == SUMO_TAG_TRIP));
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("Flow"),
                                           GUIIconSubSys::getIcon(GUIIcon::FLOW), this, MID_GNE_VEHICLE_TRANSFORM_FLOW,
                                           (myVehicle->getTagProperty().getTag() == SUMO_TAG_FLOW));
        }
    }
}


GNEVehicle::GNESingleVehiclePopupMenu::~GNESingleVehiclePopupMenu() {}


long
GNEVehicle::GNESingleVehiclePopupMenu::onCmdTransform(FXObject*, FXSelector sel, void*) {
    switch (FXSELID(sel)) {
        case MID_GNE_VEHICLE_TRANSFORM_TRIP:
            GNERouteHandler::transformToTrip(myVehicle);
            break;
        case MID_GNE_VEHICLE_TRANSFORM_FLOW:
            GNERouteHandler::transformToFlow(myVehicle);
            break;
        case MID_GNE_VEHICLE_TRANSFORM_VEHICLE:
            GNERouteHandler::transformToVehicle(myVehicle, false);
            break;
        case MID_GNE_VEHICLE_TRANSFORM_ROUTEFLOW:
            GNERouteHandler::transformToRouteFlow(myVehicle, false);
            break;
        case MID_GNE_VEHICLE_TRANSFORM_VEHICLE_EMBEDDED:
            GNERouteHandler::transformToVehicle(myVehicle, true);
            break;
        case MID_GNE_VEHICLE_TRANSFORM_FLOW_EMBEDDED:
            GNERouteHandler::transformToRouteFlow(myVehicle, true);
            break;
        case MID_GNE_VEHICLE_TRANSFORM_TRIP_JUNCTIONS:
            GNERouteHandler::transformToTripJunctions(myVehicle);
            break;
        case MID_GNE_VEHICLE_TRANSFORM_FLOW_JUNCTIONS:
            GNERouteHandler::transformToFlowJunctions(myVehicle);
            break;
        case MID_GNE_VEHICLE_TRANSFORM_TRIP_TAZS:
            GNERouteHandler::transformToTripTAZs(myVehicle);
            break;
        case MID_GNE_VEHICLE_TRANSFORM_FLOW_TAZS:
            GNERouteHandler::transformToFlowTAZs(myVehicle);
            break;
        default:
            break;
    }
    return 1;
}

// ===========================================================================
// GNEVehicle::GNESelectedVehiclesPopupMenu
// ===========================================================================

GNEVehicle::GNESelectedVehiclesPopupMenu::GNESelectedVehiclesPopupMenu(GNEVehicle* vehicle, const std::vector<GNEVehicle*>& selectedVehicle, GUIMainWindow& app, GUISUMOAbstractView& parent) :
    GUIGLObjectPopupMenu(app, parent, *vehicle),
    mySelectedVehicles(selectedVehicle),
    myVehicleTag(vehicle->getTagProperty().getTag()) {
    // build header
    vehicle->buildPopupHeader(this, app);
    // build menu command for center button and copy cursor position to clipboard
    vehicle->buildCenterPopupEntry(this);
    vehicle->buildPositionCopyEntry(this, app);
    // build menu commands for names
    GUIDesigns::buildFXMenuCommand(this, ("Copy " + vehicle->getTagStr() + " name to clipboard").c_str(), nullptr, this, MID_COPY_NAME);
    GUIDesigns::buildFXMenuCommand(this, ("Copy " + vehicle->getTagStr() + " typed name to clipboard").c_str(), nullptr, this, MID_COPY_TYPED_NAME);
    new FXMenuSeparator(this);
    // build selection and show parameters menu
    vehicle->getNet()->getViewNet()->buildSelectionACPopupEntry(this, vehicle);
    vehicle->buildShowParamsPopupEntry(this);
    // route length
    vehicle->buildMenuCommandRouteLength(this);
    // add transform functions only in demand mode
    if (vehicle->getNet()->getViewNet()->getEditModes().isCurrentSupermodeDemand()) {
        // add reverse
        vehicle->buildMenuAddReverse(this);
        // continue depending of type
        if (vehicle->getTagProperty().vehicleJunctions()) {
            // create menu pane for transform operations
            FXMenuPane* transformOperation = new FXMenuPane(this);
            this->insertMenuPaneChild(transformOperation);
            new FXMenuCascade(this, TL("transform selected to"), nullptr, transformOperation);
            // Create menu commands for restricted transforms
            myRestrictedMenuCommands[GUIDesigns::buildFXMenuCommand(transformOperation,
                                     TLF("Trips (over junctions) (only %)", toString(GNE_TAG_TRIP_JUNCTIONS)),
                                     GUIIconSubSys::getIcon(GUIIcon::TRIP_JUNCTIONS), this, MID_GNE_VEHICLE_TRANSFORM_TRIP_JUNCTIONS)] = GNE_TAG_TRIP_JUNCTIONS;
            myRestrictedMenuCommands[GUIDesigns::buildFXMenuCommand(transformOperation,
                                     TLF("Flows (over junctions) (only %)", toString(GNE_TAG_FLOW_JUNCTIONS)),
                                     GUIIconSubSys::getIcon(GUIIcon::FLOW_JUNCTIONS), this, MID_GNE_VEHICLE_TRANSFORM_FLOW_JUNCTIONS)] = GNE_TAG_FLOW_JUNCTIONS;
            // create separator
            new FXMenuSeparator(transformOperation);
            // Create menu commands for all transform
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("Trips (over junctions)"),
                                           GUIIconSubSys::getIcon(GUIIcon::TRIP_JUNCTIONS), this, MID_GNE_VEHICLE_TRANSFORM_TRIP_JUNCTIONS);
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("Flows (over junctions)"),
                                           GUIIconSubSys::getIcon(GUIIcon::FLOW_JUNCTIONS), this, MID_GNE_VEHICLE_TRANSFORM_FLOW_JUNCTIONS);
        } else if (vehicle->getTagProperty().vehicleTAZs()) {
            // create menu pane for transform operations
            FXMenuPane* transformOperation = new FXMenuPane(this);
            this->insertMenuPaneChild(transformOperation);
            new FXMenuCascade(this, TL("transform selected to"), nullptr, transformOperation);
            // Create menu commands for all transform
            myRestrictedMenuCommands[GUIDesigns::buildFXMenuCommand(transformOperation,
                                     TLF("Trips (over TAZs) (only %)", toString(GNE_TAG_TRIP_TAZS)),
                                     GUIIconSubSys::getIcon(GUIIcon::TRIP_TAZS), this, MID_GNE_VEHICLE_TRANSFORM_TRIP_TAZS)] = GNE_TAG_TRIP_TAZS;
            myRestrictedMenuCommands[GUIDesigns::buildFXMenuCommand(transformOperation,
                                     TLF("Flows (over TAZs) (only %)", toString(GNE_TAG_FLOW_TAZS)),
                                     GUIIconSubSys::getIcon(GUIIcon::FLOW_TAZS), this, MID_GNE_VEHICLE_TRANSFORM_FLOW_TAZS)] = GNE_TAG_FLOW_TAZS;
            // create separator
            new FXMenuSeparator(transformOperation);
            // Create menu commands for all transform
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("Trips (over TAZs)"),
                                           GUIIconSubSys::getIcon(GUIIcon::TRIP_TAZS), this, MID_GNE_VEHICLE_TRANSFORM_TRIP_TAZS);
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("Flows (over TAZs)"),
                                           GUIIconSubSys::getIcon(GUIIcon::FLOW_TAZS), this, MID_GNE_VEHICLE_TRANSFORM_FLOW_TAZS);
        } else {
            // create menu pane for transform operations
            FXMenuPane* transformOperation = new FXMenuPane(this);
            this->insertMenuPaneChild(transformOperation);
            new FXMenuCascade(this, TL("transform selected to"), nullptr, transformOperation);
            // Create menu commands for all transform
            myRestrictedMenuCommands[GUIDesigns::buildFXMenuCommand(transformOperation,
                                     TLF("Vehicles (only %)", toString(SUMO_TAG_VEHICLE)),
                                     GUIIconSubSys::getIcon(GUIIcon::VEHICLE), this, MID_GNE_VEHICLE_TRANSFORM_VEHICLE)] = SUMO_TAG_VEHICLE;
            myRestrictedMenuCommands[GUIDesigns::buildFXMenuCommand(transformOperation,
                                     TLF("Vehicles (embedded route) (only %)", toString(GNE_TAG_VEHICLE_WITHROUTE)),
                                     GUIIconSubSys::getIcon(GUIIcon::VEHICLE), this, MID_GNE_VEHICLE_TRANSFORM_VEHICLE_EMBEDDED)] = GNE_TAG_VEHICLE_WITHROUTE;
            myRestrictedMenuCommands[GUIDesigns::buildFXMenuCommand(transformOperation,
                                     TLF("RouteFlows (only %)", toString(GNE_TAG_FLOW_ROUTE)),
                                     GUIIconSubSys::getIcon(GUIIcon::ROUTEFLOW), this, MID_GNE_VEHICLE_TRANSFORM_ROUTEFLOW)] = GNE_TAG_FLOW_ROUTE;
            myRestrictedMenuCommands[GUIDesigns::buildFXMenuCommand(transformOperation,
                                     TLF("RouteFlows (embedded route) (only %)", toString(GNE_TAG_FLOW_WITHROUTE)),
                                     GUIIconSubSys::getIcon(GUIIcon::ROUTEFLOW), this, MID_GNE_VEHICLE_TRANSFORM_FLOW_EMBEDDED)] = GNE_TAG_FLOW_WITHROUTE;
            myRestrictedMenuCommands[GUIDesigns::buildFXMenuCommand(transformOperation,
                                     TLF("Trips (only %)", toString(SUMO_TAG_TRIP)),
                                     GUIIconSubSys::getIcon(GUIIcon::TRIP), this, MID_GNE_VEHICLE_TRANSFORM_TRIP)] = SUMO_TAG_TRIP;
            myRestrictedMenuCommands[GUIDesigns::buildFXMenuCommand(transformOperation,
                                     TLF("Flows (only %)", toString(SUMO_TAG_FLOW)),
                                     GUIIconSubSys::getIcon(GUIIcon::FLOW), this, MID_GNE_VEHICLE_TRANSFORM_FLOW)] = SUMO_TAG_FLOW;
            // create separator
            new FXMenuSeparator(transformOperation);
            // Create menu commands for all transform
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("Vehicles"),
                                           GUIIconSubSys::getIcon(GUIIcon::VEHICLE), this, MID_GNE_VEHICLE_TRANSFORM_VEHICLE);
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("Vehicles (embedded route)"),
                                           GUIIconSubSys::getIcon(GUIIcon::VEHICLE), this, MID_GNE_VEHICLE_TRANSFORM_VEHICLE_EMBEDDED);
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("RouteFlows"),
                                           GUIIconSubSys::getIcon(GUIIcon::ROUTEFLOW), this, MID_GNE_VEHICLE_TRANSFORM_ROUTEFLOW);
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("RouteFlows (embedded route)"),
                                           GUIIconSubSys::getIcon(GUIIcon::ROUTEFLOW), this, MID_GNE_VEHICLE_TRANSFORM_FLOW_EMBEDDED);
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("Trips"),
                                           GUIIconSubSys::getIcon(GUIIcon::TRIP), this, MID_GNE_VEHICLE_TRANSFORM_TRIP);
            GUIDesigns::buildFXMenuCommand(transformOperation,
                                           TL("Flows"),
                                           GUIIconSubSys::getIcon(GUIIcon::FLOW), this, MID_GNE_VEHICLE_TRANSFORM_FLOW);
        }
    }
}


GNEVehicle::GNESelectedVehiclesPopupMenu::~GNESelectedVehiclesPopupMenu() {}


long
GNEVehicle::GNESelectedVehiclesPopupMenu::onCmdTransform(FXObject* obj, FXSelector sel, void*) {
    // iterate over all selected vehicles
    for (const auto& vehicle : mySelectedVehicles) {
        switch (FXSELID(sel)) {
            case MID_GNE_VEHICLE_TRANSFORM_TRIP:
                if (!vehicle->getTagProperty().vehicleJunctions() && !vehicle->getTagProperty().vehicleTAZs()) {
                    if (myRestrictedMenuCommands.count(obj) > 0) {
                        if (vehicle->getTagProperty().getTag() == myRestrictedMenuCommands.at(obj)) {
                            GNERouteHandler::transformToTrip(vehicle);
                        }
                    } else {
                        GNERouteHandler::transformToTrip(vehicle);
                    }
                }
                break;
            case MID_GNE_VEHICLE_TRANSFORM_FLOW:
                if (!vehicle->getTagProperty().vehicleJunctions() && !vehicle->getTagProperty().vehicleTAZs()) {
                    if (myRestrictedMenuCommands.count(obj) > 0) {
                        if (vehicle->getTagProperty().getTag() == myRestrictedMenuCommands.at(obj)) {
                            GNERouteHandler::transformToFlow(vehicle);
                        }
                    } else {
                        GNERouteHandler::transformToFlow(vehicle);
                    }
                }
                break;
            case MID_GNE_VEHICLE_TRANSFORM_VEHICLE:
                if (!vehicle->getTagProperty().vehicleJunctions() && !vehicle->getTagProperty().vehicleTAZs()) {
                    if (myRestrictedMenuCommands.count(obj) > 0) {
                        if (vehicle->getTagProperty().getTag() == myRestrictedMenuCommands.at(obj)) {
                            GNERouteHandler::transformToVehicle(vehicle, false);
                        }
                    } else {
                        GNERouteHandler::transformToVehicle(vehicle, false);
                    }
                }
                break;
            case MID_GNE_VEHICLE_TRANSFORM_ROUTEFLOW:
                if (!vehicle->getTagProperty().vehicleJunctions() && !vehicle->getTagProperty().vehicleTAZs()) {
                    if (myRestrictedMenuCommands.count(obj) > 0) {
                        if (vehicle->getTagProperty().getTag() == myRestrictedMenuCommands.at(obj)) {
                            GNERouteHandler::transformToRouteFlow(vehicle, false);
                        }
                    } else {
                        GNERouteHandler::transformToRouteFlow(vehicle, false);
                    }
                }
                break;
            case MID_GNE_VEHICLE_TRANSFORM_VEHICLE_EMBEDDED:
                if (!vehicle->getTagProperty().vehicleJunctions() && !vehicle->getTagProperty().vehicleTAZs()) {
                    if (myRestrictedMenuCommands.count(obj) > 0) {
                        if (vehicle->getTagProperty().getTag() == myRestrictedMenuCommands.at(obj)) {
                            GNERouteHandler::transformToVehicle(vehicle, true);
                        }
                    } else {
                        GNERouteHandler::transformToVehicle(vehicle, true);
                    }
                }
                break;
            case MID_GNE_VEHICLE_TRANSFORM_FLOW_EMBEDDED:
                if (!vehicle->getTagProperty().vehicleJunctions() && !vehicle->getTagProperty().vehicleTAZs()) {
                    if (myRestrictedMenuCommands.count(obj) > 0) {
                        if (vehicle->getTagProperty().getTag() == myRestrictedMenuCommands.at(obj)) {
                            GNERouteHandler::transformToRouteFlow(vehicle, true);
                        }
                    } else {
                        GNERouteHandler::transformToRouteFlow(vehicle, true);
                    }
                }
                break;
            case MID_GNE_VEHICLE_TRANSFORM_TRIP_JUNCTIONS:
                if (vehicle->getTagProperty().vehicleJunctions()) {
                    if (myRestrictedMenuCommands.count(obj) > 0) {
                        if (vehicle->getTagProperty().getTag() == myRestrictedMenuCommands.at(obj)) {
                            GNERouteHandler::transformToTripJunctions(vehicle);
                        }
                    } else {
                        GNERouteHandler::transformToTripJunctions(vehicle);
                    }
                }
                break;
            case MID_GNE_VEHICLE_TRANSFORM_FLOW_JUNCTIONS:
                if (vehicle->getTagProperty().vehicleJunctions()) {
                    if (myRestrictedMenuCommands.count(obj) > 0) {
                        if (vehicle->getTagProperty().getTag() == myRestrictedMenuCommands.at(obj)) {
                            GNERouteHandler::transformToFlowJunctions(vehicle);
                        }
                    } else {
                        GNERouteHandler::transformToFlowJunctions(vehicle);
                    }
                }
                break;
            case MID_GNE_VEHICLE_TRANSFORM_TRIP_TAZS:
                if (vehicle->getTagProperty().vehicleTAZs()) {
                    if (myRestrictedMenuCommands.count(obj) > 0) {
                        if (vehicle->getTagProperty().getTag() == myRestrictedMenuCommands.at(obj)) {
                            GNERouteHandler::transformToTripTAZs(vehicle);
                        }
                    } else {
                        GNERouteHandler::transformToTripTAZs(vehicle);
                    }
                }
                break;
            case MID_GNE_VEHICLE_TRANSFORM_FLOW_TAZS:
                if (vehicle->getTagProperty().vehicleTAZs()) {
                    if (myRestrictedMenuCommands.count(obj) > 0) {
                        if (vehicle->getTagProperty().getTag() == myRestrictedMenuCommands.at(obj)) {
                            GNERouteHandler::transformToFlowTAZs(vehicle);
                        }
                    } else {
                        GNERouteHandler::transformToFlowTAZs(vehicle);
                    }
                }
                break;
            default:
                break;
        }
    }
    return 1;
}

// ===========================================================================
// member method definitions
// ===========================================================================

GNEVehicle::GNEVehicle(SumoXMLTag tag, GNENet* net) :
    GNEDemandElement("", net, GLO_VEHICLE, tag, GUIIconSubSys::getIcon(GUIIcon::VEHICLE),
                     GNEPathManager::PathElement::Options::DEMAND_ELEMENT, {}, {}, {}, {}, {}, {}),
GNEDemandElementFlow(this) {
    // reset default values
    resetDefaultValues();
    // set end and vehPerHours as default flow values
    toggleAttribute(SUMO_ATTR_END, true);
    toggleAttribute(SUMO_ATTR_VEHSPERHOUR, true);
}


GNEVehicle::GNEVehicle(SumoXMLTag tag, GNENet* net, const std::string& vehicleID, GNEDemandElement* vehicleType, GNEDemandElement* route) :
    GNEDemandElement(vehicleID, net, (tag == GNE_TAG_FLOW_ROUTE) ? GLO_ROUTEFLOW : GLO_VEHICLE, tag,
                     (tag == GNE_TAG_FLOW_ROUTE) ? GUIIconSubSys::getIcon(GUIIcon::ROUTEFLOW) : GUIIconSubSys::getIcon(GUIIcon::VEHICLE),
                     GNEPathManager::PathElement::Options::DEMAND_ELEMENT, {}, {}, {}, {}, {vehicleType, route}, {}),
GNEDemandElementFlow(this) {
    // SUMOVehicleParameter ID has to be set manually
    id = vehicleID;
    // set manually vtypeID (needed for saving)
    vtypeid = vehicleType->getID();
}


GNEVehicle::GNEVehicle(SumoXMLTag tag, GNENet* net, GNEDemandElement* vehicleType, GNEDemandElement* route, const SUMOVehicleParameter& vehicleParameters) :
    GNEDemandElement(vehicleParameters.id, net, (tag == GNE_TAG_FLOW_ROUTE) ? GLO_ROUTEFLOW : GLO_VEHICLE, tag,
                     (tag == GNE_TAG_FLOW_ROUTE) ? GUIIconSubSys::getIcon(GUIIcon::ROUTEFLOW) : GUIIconSubSys::getIcon(GUIIcon::VEHICLE),
                     GNEPathManager::PathElement::Options::DEMAND_ELEMENT, {}, {}, {}, {}, {vehicleType, route}, {}),
GNEDemandElementFlow(this, vehicleParameters) {
    // SUMOVehicleParameter ID has to be set manually
    id = vehicleParameters.id;
    // set manually vtypeID (needed for saving)
    vtypeid = vehicleType->getID();
}


GNEVehicle::GNEVehicle(SumoXMLTag tag, GNENet* net, GNEDemandElement* vehicleType, const SUMOVehicleParameter& vehicleParameters) :
    GNEDemandElement(vehicleParameters.id, net, (tag == GNE_TAG_VEHICLE_WITHROUTE) ? GLO_VEHICLE : GLO_ROUTEFLOW, tag,
                     (tag == GNE_TAG_FLOW_ROUTE) ? GUIIconSubSys::getIcon(GUIIcon::ROUTEFLOW) : GUIIconSubSys::getIcon(GUIIcon::VEHICLE),
                     GNEPathManager::PathElement::Options::DEMAND_ELEMENT, {}, {}, {}, {}, {vehicleType}, {}),
GNEDemandElementFlow(this, vehicleParameters) {
    // SUMOVehicleParameter ID has to be set manually
    id = vehicleParameters.id;
    // reset routeid
    routeid.clear();
    // set manually vtypeID (needed for saving)
    vtypeid = vehicleType->getID();
}


GNEVehicle::GNEVehicle(SumoXMLTag tag, GNENet* net, const std::string& vehicleID, GNEDemandElement* vehicleType,
                       GNEEdge* fromEdge, GNEEdge* toEdge) :
    GNEDemandElement(vehicleID, net, (tag == SUMO_TAG_FLOW) ? GLO_FLOW : GLO_TRIP, tag,
                     (tag == SUMO_TAG_FLOW) ? GUIIconSubSys::getIcon(GUIIcon::FLOW) : GUIIconSubSys::getIcon(GUIIcon::TRIP),
                     GNEPathManager::PathElement::Options::DEMAND_ELEMENT, {}, {fromEdge, toEdge}, {}, {}, {vehicleType}, {}),
GNEDemandElementFlow(this) {
}


GNEVehicle::GNEVehicle(SumoXMLTag tag, GNENet* net, GNEDemandElement* vehicleType, GNEEdge* fromEdge, GNEEdge* toEdge,
                       const SUMOVehicleParameter& vehicleParameters) :
    GNEDemandElement(vehicleParameters.id, net, (tag == SUMO_TAG_FLOW) ? GLO_FLOW : GLO_TRIP, tag,
                     (tag == SUMO_TAG_FLOW) ? GUIIconSubSys::getIcon(GUIIcon::FLOW) : GUIIconSubSys::getIcon(GUIIcon::TRIP),
                     GNEPathManager::PathElement::Options::DEMAND_ELEMENT, {}, {fromEdge, toEdge}, {}, {}, {vehicleType}, {}),
GNEDemandElementFlow(this, vehicleParameters) {
}


GNEVehicle::GNEVehicle(SumoXMLTag tag, GNENet* net, const std::string& vehicleID, GNEDemandElement* vehicleType, GNEJunction* fromJunction, GNEJunction* toJunction) :
    GNEDemandElement(vehicleID, net, (tag == GNE_TAG_FLOW_JUNCTIONS) ? GLO_FLOW : GLO_TRIP, tag,
                     (tag == GNE_TAG_FLOW_JUNCTIONS) ? GUIIconSubSys::getIcon(GUIIcon::FLOW_JUNCTIONS) : GUIIconSubSys::getIcon(GUIIcon::TRIP_JUNCTIONS),
                     GNEPathManager::PathElement::Options::DEMAND_ELEMENT, {
    fromJunction, toJunction
}, {}, {}, {}, {vehicleType}, {}),
GNEDemandElementFlow(this) {
}


GNEVehicle::GNEVehicle(SumoXMLTag tag, GNENet* net, GNEDemandElement* vehicleType, GNEJunction* fromJunction, GNEJunction* toJunction, const SUMOVehicleParameter& vehicleParameters) :
    GNEDemandElement(vehicleParameters.id, net, (tag == GNE_TAG_FLOW_JUNCTIONS) ? GLO_FLOW : GLO_TRIP, tag,
                     (tag == GNE_TAG_FLOW_JUNCTIONS) ? GUIIconSubSys::getIcon(GUIIcon::FLOW_JUNCTIONS) : GUIIconSubSys::getIcon(GUIIcon::TRIP_JUNCTIONS),
                     GNEPathManager::PathElement::Options::DEMAND_ELEMENT, {
    fromJunction, toJunction
}, {}, {}, {}, {vehicleType}, {}),
GNEDemandElementFlow(this, vehicleParameters) {
}


GNEVehicle::GNEVehicle(SumoXMLTag tag, GNENet* net, GNEDemandElement* vehicleType, GNEAdditional* fromTAZ, GNEAdditional* toTAZ, const SUMOVehicleParameter& vehicleParameters) :
    GNEDemandElement(vehicleParameters.id, net, (tag == GNE_TAG_FLOW_TAZS) ? GLO_FLOW : GLO_TRIP, tag,
                     (tag == GNE_TAG_FLOW_TAZS) ? GUIIconSubSys::getIcon(GUIIcon::FLOW_TAZS) : GUIIconSubSys::getIcon(GUIIcon::TRIP_TAZS),
                     GNEPathManager::PathElement::Options::DEMAND_ELEMENT, {}, {}, {}, {fromTAZ, toTAZ}, {vehicleType}, {}),
GNEDemandElementFlow(this, vehicleParameters) {
    // mark taz parameters as set
    parametersSet |= VEHPARS_FROM_TAZ_SET;
    parametersSet |= VEHPARS_TO_TAZ_SET;
    fromTaz = fromTAZ->getID();
    toTaz = toTAZ->getID();
}


GNEVehicle::~GNEVehicle() {}


GNEMoveOperation*
GNEVehicle::getMoveOperation() {
    // get first and last lanes
    const GNELane* firstLane = getFirstPathLane();
    const GNELane* lastLane = getLastPathLane();
    // check both lanes
    if (firstLane && lastLane) {
        // get depart and arrival positions (doubles)
        const double departPosDouble = getAttributeDouble(SUMO_ATTR_DEPARTPOS);
        const double arrivalPosDouble = (getAttributeDouble(SUMO_ATTR_ARRIVALPOS) < 0) ? lastLane->getLaneShape().length2D() : getAttributeDouble(SUMO_ATTR_ARRIVALPOS);
        // obtain diameter
        const double diameter = getAttributeDouble(SUMO_ATTR_WIDTH) > getAttributeDouble(SUMO_ATTR_LENGTH) ? getAttributeDouble(SUMO_ATTR_WIDTH) : getAttributeDouble(SUMO_ATTR_LENGTH);
        // return move operation depending if we're editing departPos or arrivalPos
        if (myNet->getViewNet()->getPositionInformation().distanceSquaredTo2D(getAttributePosition(GNE_ATTR_PLAN_GEOMETRY_STARTPOS)) < (diameter * diameter)) {
            return new GNEMoveOperation(this, firstLane, departPosDouble, lastLane, INVALID_DOUBLE,
                                        myNet->getViewNet()->getViewParent()->getMoveFrame()->getCommonModeOptions()->getAllowChangeLane(),
                                        GNEMoveOperation::OperationType::TWO_LANES_MOVEFIRST);
        } else if (myNet->getViewNet()->getPositionInformation().distanceSquaredTo2D(getAttributePosition(GNE_ATTR_PLAN_GEOMETRY_ENDPOS)) < (myArrivalPositionDiameter * myArrivalPositionDiameter)) {
            return new GNEMoveOperation(this, firstLane, INVALID_DOUBLE, lastLane, arrivalPosDouble,
                                        myNet->getViewNet()->getViewParent()->getMoveFrame()->getCommonModeOptions()->getAllowChangeLane(),
                                        GNEMoveOperation::OperationType::TWO_LANES_MOVESECOND);
        }
    }
    // nothing to move
    return nullptr;
}


void
GNEVehicle::writeDemandElement(OutputDevice& device) const {
    // attribute VType must not be written if is DEFAULT_VTYPE_ID
    if (getTypeParent()->getID() == DEFAULT_VTYPE_ID) {
        // unset VType parameter
        parametersSet &= ~VEHPARS_VTYPE_SET;
        // write vehicle attributes (VType will not be written)
        write(device, OptionsCont::getOptions(), myTagProperty.getXMLTag());
        // set VType parameter again
        parametersSet |= VEHPARS_VTYPE_SET;
    } else {
        // write vehicle attributes, including type/distribution
        write(device, OptionsCont::getOptions(), myTagProperty.getXMLTag(), getParentDemandElements().front()->getID());
    }
    // write specific attribute depending of tag property
    if (myTagProperty.vehicleRoute()) {
        // write route
        device.writeAttr(SUMO_ATTR_ROUTE, getRouteParent()->getID());
    }
    // write from, to and edge vias
    if (myTagProperty.vehicleEdges()) {
        // write manually from/to edges (it correspond to front and back parent edges)
        device.writeAttr(SUMO_ATTR_FROM, getParentEdges().front()->getID());
        device.writeAttr(SUMO_ATTR_TO, getParentEdges().back()->getID());
        // only write via if there isn't empty
        if (via.size() > 0) {
            device.writeAttr(SUMO_ATTR_VIA, via);
        }
    }
    // write from and to junctions
    if (myTagProperty.vehicleJunctions()) {
        // write manually from/to junctions (it correspond to front and back parent junctions)
        device.writeAttr(SUMO_ATTR_FROM_JUNCTION, getParentJunctions().front()->getID());
        device.writeAttr(SUMO_ATTR_TO_JUNCTION, getParentJunctions().back()->getID());
    }
    // write flow attributes
    writeFlowAttributes(this, device);
    // write parameters
    writeParams(device);
    // write route elements associated to this vehicle (except for calibrator FLows)
    if ((getChildDemandElements().size() > 0) && !myTagProperty.isCalibrator()) {
        if (getChildDemandElements().front()->getTagProperty().getTag() == GNE_TAG_ROUTE_EMBEDDED) {
            // write embedded route
            getChildDemandElements().front()->writeDemandElement(device);
            // write stops
            for (const auto& demandElement : getChildDemandElements()) {
                if (demandElement->getTagProperty().isVehicleStop()) {
                    demandElement->writeDemandElement(device);
                }
            }
        } else {
            for (const auto& route : getChildDemandElements()) {
                route->writeDemandElement(device);
            }
        }
    }
    // close vehicle tag
    device.closeTag();
}


GNEDemandElement::Problem
GNEVehicle::isDemandElementValid() const {
    // check conditions
    if (myTagProperty.vehicleTAZs()) {
        // vehicles and flows over tazs are always valid
        return Problem::OK;
    } else if (myTagProperty.vehicleEdges()) {
        // check vehicles and flows paths
        if (getParentEdges().front() == getParentEdges().back()) {
            return Problem::OK;
        } else if (myNet->getPathManager()->isPathValid(this)) {
            return Problem::OK;
        } else {
            return Problem::INVALID_PATH;
        }
    } else if (myTagProperty.vehicleJunctions()) {
        // check vehicles and flows paths
        if (getParentJunctions().front() == getParentJunctions().back()) {
            return Problem::OK;
        } else if (myNet->getPathManager()->isPathValid(this)) {
            return Problem::OK;
        } else {
            return Problem::INVALID_PATH;
        }
    } else if (myTagProperty.vehicleRoute()) {
        // check if exist a valid path using route parent edges
        if (myNet->getPathManager()->getPathCalculator()->calculateDijkstraPath(getTypeParent()->getVClass(), getRouteParent()->getParentEdges()).size() > 0) {
            return Problem::OK;
        } else {
            return Problem::INVALID_PATH;
        }
    } else if (myTagProperty.vehicleRouteEmbedded()) {
        // check if exist a valid path using route child edges
        if (myNet->getPathManager()->getPathCalculator()->calculateDijkstraPath(getTypeParent()->getVClass(), getChildDemandElements().at(0)->getParentEdges()).size() > 0) {
            return Problem::OK;
        } else {
            return Problem::INVALID_PATH;
        }
    } else {
        return Problem::INVALID_ELEMENT;
    }
}


std::string
GNEVehicle::getDemandElementProblem() const {
    // only trips or flows can have problems
    if (myTagProperty.vehicleEdges()) {
        // check if exist at least a connection between every edge
        for (int i = 1; i < (int)getParentEdges().size(); i++) {
            if (myNet->getPathManager()->getPathCalculator()->consecutiveEdgesConnected(getTypeParent()->getVClass(), getParentEdges().at((int)i - 1), getParentEdges().at(i)) == false) {
                return ("There is no valid path between edges '" + getParentEdges().at((int)i - 1)->getID() + "' and '" + getParentEdges().at(i)->getID() + "'");
            }
        }
        // if there are connections between all edges, then all is ok
        return "";
    } else if (myTagProperty.vehicleJunctions()) {
        return ("No path between junction '" + getParentJunctions().front()->getID() + "' and '" + getParentJunctions().back()->getID() + "'");
    } else if (myTagProperty.vehicleRoute()) {
        // get route parent edges
        const std::vector<GNEEdge*>& routeEdges = getRouteParent()->getParentEdges();
        // check if exist at least a connection between every edge
        for (int i = 1; i < (int)routeEdges.size(); i++) {
            if (myNet->getPathManager()->getPathCalculator()->consecutiveEdgesConnected(getTypeParent()->getVClass(), routeEdges.at((int)i - 1), routeEdges.at(i)) == false) {
                return ("There is no valid path between route edges '" + routeEdges.at((int)i - 1)->getID() + "' and '" + routeEdges.at(i)->getID() + "'");
            }
        }
        // if there are connections between all edges, then all is ok
        return "";
    } else if (myTagProperty.vehicleRouteEmbedded()) {
        // get route parent edges
        const std::vector<GNEEdge*>& routeEdges = getChildDemandElements().at(0)->getParentEdges();
        // check if exist at least a connection between every edge
        for (int i = 1; i < (int)routeEdges.size(); i++) {
            if (myNet->getPathManager()->getPathCalculator()->consecutiveEdgesConnected(getTypeParent()->getVClass(), routeEdges.at((int)i - 1), routeEdges.at(i)) == false) {
                return ("There is no valid path between embedded route edges '" + routeEdges.at((int)i - 1)->getID() + "' and '" + routeEdges.at(i)->getID() + "'");
            }
        }
        // if there are connections between all edges, then all is ok
        return "";
    } else {
        return "";
    }
}


void
GNEVehicle::fixDemandElementProblem() {
    //
}


SUMOVehicleClass
GNEVehicle::getVClass() const {
    return getParentDemandElements().front()->getVClass();
}


const RGBColor&
GNEVehicle::getColor() const {
    return color;
}


void
GNEVehicle::updateGeometry() {
    if (getParentJunctions().size() > 0) {
        // calculate rotation between both junctions
        const Position posA = getParentJunctions().front()->getPositionInView();
        const Position posB = getParentJunctions().back()->getPositionInView();
        const double rot = ((double)atan2((posB.x() - posA.x()), (posA.y() - posB.y())) * (double) -180.0 / (double)M_PI);
        // update Geometry
        myDemandElementGeometry.updateSinglePosGeometry(getParentJunctions().front()->getPositionInView(), rot);
    } else if (getParentAdditionals().size() > 0) {
        // calculate rotation between both TAZs
        const Position posA = getParentAdditionals().front()->getAttribute(SUMO_ATTR_CENTER).empty() ?
                              getParentAdditionals().front()->getAttributePosition(GNE_ATTR_TAZ_CENTROID) :
                              getParentAdditionals().front()->getAttributePosition(SUMO_ATTR_CENTER);
        const Position posB = getParentAdditionals().back()->getAttribute(SUMO_ATTR_CENTER).empty() ?
                              getParentAdditionals().back()->getAttributePosition(GNE_ATTR_TAZ_CENTROID) :
                              getParentAdditionals().back()->getAttributePosition(SUMO_ATTR_CENTER);
        const double rot = ((double)atan2((posB.x() - posA.x()), (posA.y() - posB.y())) * (double) -180.0 / (double)M_PI);
        // update Geometry
        myDemandElementGeometry.updateSinglePosGeometry(posA, rot);
    } else {
        // get first path lane
        const GNELane* firstPathLane = getFirstPathLane();
        // check path lane
        if (firstPathLane) {
            // declare departPos
            double posOverLane = 0;
            if (wasSet(VEHPARS_DEPARTPOS_SET) && (departPosProcedure == DepartPosDefinition::GIVEN)) {
                posOverLane = departPos;
            }
            // update Geometry
            myDemandElementGeometry.updateGeometry(firstPathLane->getLaneShape(), posOverLane, myMoveElementLateralOffset);
            // compute route embedded associated with this vehicle
            for (const auto& demandElement : getChildDemandElements()) {
                if (demandElement->getTagProperty().getTag() == GNE_TAG_ROUTE_EMBEDDED) {
                    demandElement->computePathElement();
                }
                demandElement->updateGeometry();
            }
        }
    }
}


Position
GNEVehicle::getPositionInView() const {
    return myDemandElementGeometry.getShape().front();
}


bool
GNEVehicle::checkDrawRelatedContour() const {
    // get edit modes
    const auto& editModes = myNet->getViewNet()->getEditModes();
    // check if we're editing a type
    if (editModes.isCurrentSupermodeDemand() && (editModes.demandEditMode == DemandEditMode::DEMAND_TYPE) &&
            (myNet->getViewNet()->getViewParent()->getTypeFrame()->getTypeSelector()->getCurrentType() == getParentDemandElements().front())) {
        return true;
    } else {
        return false;
    }
}


GUIGLObjectPopupMenu*
GNEVehicle::getPopUpMenu(GUIMainWindow& app, GUISUMOAbstractView& parent) {
    if (isAttributeCarrierSelected()) {
        // obtain all selected vehicles
        const auto selectedDemandElements = myNet->getAttributeCarriers()->getSelectedDemandElements();
        std::vector<GNEVehicle*> selectedVehicles;
        selectedVehicles.reserve(selectedDemandElements.size());
        for (const auto& selectedDemandElement : selectedDemandElements) {
            if (selectedDemandElement->getTagProperty().isVehicle()) {
                selectedVehicles.push_back(dynamic_cast<GNEVehicle*>(selectedDemandElement));
            }
        }
        // return a GNESelectedVehiclesPopupMenu
        return new GNESelectedVehiclesPopupMenu(this, selectedVehicles, app, parent);
    } else {
        // return a GNESingleVehiclePopupMenu
        return new GNESingleVehiclePopupMenu(this, app, parent);
    }
}


std::string
GNEVehicle::getParentName() const {
    if (myTagProperty.vehicleRoute()) {
        return getRouteParent()->getID();
    } else if (myTagProperty.vehicleEdges()) {
        return getParentEdges().front()->getID();
    } else if (myTagProperty.vehicleJunctions()) {
        return getParentJunctions().front()->getID();
    } else if (myTagProperty.vehicleTAZs()) {
        return getParentAdditionals().front()->getID();
    } else {
        throw ProcessError(TL("Invalid vehicle tag"));
    }
}


double
GNEVehicle::getExaggeration(const GUIVisualizationSettings& s) const {
    return s.vehicleSize.getExaggeration(s, this);
}


Boundary
GNEVehicle::getCenteringBoundary() const {
    Boundary vehicleBoundary;
    vehicleBoundary.add(myDemandElementGeometry.getShape().front());
    vehicleBoundary.grow(20);
    return vehicleBoundary;
}


void
GNEVehicle::splitEdgeGeometry(const double /*splitPosition*/, const GNENetworkElement* /*originalElement*/, const GNENetworkElement* /*newElement*/, GNEUndoList* /*undoList*/) {
    // geometry of this element cannot be splitted
}


void
GNEVehicle::drawGL(const GUIVisualizationSettings& s) const {
    // only drawn in super mode demand
    if (myNet->getViewNet()->getNetworkViewOptions().showDemandElements() &&
            myNet->getViewNet()->getDataViewOptions().showDemandElements() &&
            myNet->getViewNet()->getDemandViewOptions().showNonInspectedDemandElements(this)) {
        // declare common attributes
        const bool drawSpreadVehicles = (myNet->getViewNet()->getNetworkViewOptions().drawSpreadVehicles() || myNet->getViewNet()->getDemandViewOptions().drawSpreadVehicles());
        const double exaggeration = getExaggeration(s);
        const double width = getTypeParent()->getAttributeDouble(SUMO_ATTR_WIDTH);
        const double length = getTypeParent()->getAttributeDouble(SUMO_ATTR_LENGTH);
        const double vehicleSizeSquared = (width * width) * (length * length) * (exaggeration * exaggeration);
        // obtain Position an rotation (depending of draw spread vehicles)
        if ((!drawSpreadVehicles || (mySpreadGeometry.getShape().size() > 0)) && (myDemandElementGeometry.getShape().size() > 0)) {
            const Position vehiclePosition = drawSpreadVehicles ? mySpreadGeometry.getShape().front() : myDemandElementGeometry.getShape().front();
            const double vehicleRotation = drawSpreadVehicles ? mySpreadGeometry.getShapeRotations().front() : myDemandElementGeometry.getShapeRotations().front();
            // check that position is valid
            if (vehiclePosition == Position::INVALID) {
                return;
            }
            // get detail level
            const auto d = s.getDetailLevel(exaggeration);
            // draw geometry only if we'rent in drawForObjectUnderCursor mode
            if (s.checkDrawVehicle(d, isAttributeCarrierSelected())) {
                // first check if if mouse is enough near to this vehicle to draw it
                if (s.drawForRectangleSelection && (myNet->getViewNet()->getPositionInformation().distanceSquaredTo2D(vehiclePosition) >= (vehicleSizeSquared + 2))) {
                    // push draw matrix
                    GLHelper::pushMatrix();
                    // Start with the drawing of the area translating matrix to origin
                    myNet->getViewNet()->drawTranslateFrontAttributeCarrier(this, getType());
                    // translate to drawing position
                    glTranslated(vehiclePosition.x(), vehiclePosition.y(), 0);
                    glRotated(vehicleRotation, 0, 0, -1);
                    // extra translation needed to draw vehicle over edge (to avoid selecting problems)
                    glTranslated(0, (-1) * length * exaggeration, 0);
                    GLHelper::drawBoxLine(Position(0, 1), 0, 2, 1);
                    // Pop last matrix
                    GLHelper::popMatrix();
                } else {
                    SUMOVehicleShape shape = getVehicleShapeID(getTypeParent()->getAttribute(SUMO_ATTR_GUISHAPE));
                    // push draw matrix
                    GLHelper::pushMatrix();
                    // Start with the drawing of the area translating matrix to origin
                    myNet->getViewNet()->drawTranslateFrontAttributeCarrier(this, getType());
                    // translate to drawing position
                    glTranslated(vehiclePosition.x(), vehiclePosition.y(), 0);
                    glRotated(vehicleRotation, 0, 0, -1);
                    // extra translation needed to draw vehicle over edge (to avoid selecting problems)
                    glTranslated(0, (-1) * length * exaggeration, 0);
                    // set lane color
                    GLHelper::setColor(getDrawingColor(s));
                    double upscaleLength = exaggeration;
                    if ((exaggeration > 1) && (length > 5)) {
                        // reduce the length/width ratio because this is not useful at high zoom
                        upscaleLength = MAX2(1.0, upscaleLength * (5 + sqrt(length - 5)) / length);
                    }
                    glScaled(exaggeration, upscaleLength, 1);
                    // check if we're drawing in selecting mode
                    if (s.drawForRectangleSelection) {
                        // draw vehicle as a box and don't draw the rest of details
                        GUIBaseVehicleHelper::drawAction_drawVehicleAsBoxPlus(width, length);
                    } else {
                        // draw the vehicle depending of detail level
                        if (d <= GUIVisualizationSettings::Detail::VehiclePoly) {
                            GUIBaseVehicleHelper::drawAction_drawVehicleAsPoly(s, shape, width, length);
                        } else if (d <= GUIVisualizationSettings::Detail::VehicleBox) {
                            GUIBaseVehicleHelper::drawAction_drawVehicleAsBoxPlus(width, length);
                        } else if (d <= GUIVisualizationSettings::Detail::VehicleTriangle) {
                            GUIBaseVehicleHelper::drawAction_drawVehicleAsTrianglePlus(width, length);
                        }
                        // check if min gap has to be drawn
                        if (s.drawMinGap) {
                            const double minGap = -1 * getTypeParent()->getAttributeDouble(SUMO_ATTR_MINGAP);
                            glColor3d(0., 1., 0.);
                            glBegin(GL_LINES);
                            glVertex2d(0., 0);
                            glVertex2d(0., minGap);
                            glVertex2d(-.5, minGap);
                            glVertex2d(.5, minGap);
                            glEnd();
                        }
                        // drawing name at GLO_MAX fails unless translating z
                        glTranslated(0, MIN2(length / 2, double(5)), -getType());
                        glScaled(1 / exaggeration, 1 / upscaleLength, 1);
                        glRotated(vehicleRotation, 0, 0, -1);
                        drawName(Position(0, 0), s.scale, getTypeParent()->getAttribute(SUMO_ATTR_GUISHAPE) == "pedestrian" ? s.personName : s.vehicleName, s.angle);
                        // draw line
                        if (s.vehicleName.show(this) && line != "") {
                            glTranslated(0, 0.6 * s.vehicleName.scaledSize(s.scale), 0);
                            GLHelper::drawTextSettings(s.vehicleName, "line:" + line, Position(0, 0), s.scale, s.angle);
                        }
                    }
                    // pop draw matrix
                    GLHelper::popMatrix();
                    // draw line between junctions if path isn't valid
                    if ((getParentJunctions().size() > 0) && !myNet->getPathManager()->isPathValid(this)) {
                        drawJunctionLine(this);
                    }
                    // draw stack label
                    if ((myStackedLabelNumber > 0) && !drawSpreadVehicles) {
                        drawStackLabel(myStackedLabelNumber, "Vehicle", vehiclePosition, vehicleRotation, width, length, exaggeration);
                    }
                    // draw flow label
                    if (myTagProperty.isFlow()) {
                        drawFlowLabel(vehiclePosition, vehicleRotation, width, length, exaggeration);
                    }
                }
                // draw lock icon
                GNEViewNetHelper::LockIcon::drawLockIcon(d, this, getType(), vehiclePosition, exaggeration);
                // draw dotted contour
                myVehicleContour.drawDottedContours(s, d, this, s.dottedContourSettings.segmentWidth, true);
            }
            // draw squared shape
            myVehicleContour.calculateContourRectangleShape(s, d, this, vehiclePosition, length * 0.5, width * 0.5, getType(), length * -0.5, 0, vehicleRotation, exaggeration);
        }
    }
}


void
GNEVehicle::computePathElement() {
    // calculate path (only for flows and trips)
    if (myTagProperty.vehicleJunctions()) {
        // calculate path
        myNet->getPathManager()->calculatePath(this, getVClass(), getParentJunctions().front(), getParentJunctions().back());
    } else if (myTagProperty.vehicleEdges()) {
        // save edges in wich this vehicle has to stop
        std::vector<GNEEdge*> edgeStops;
        // iterate over child demand elements
        for (const auto& demandElement : getChildDemandElements()) {
            // extract lanes
            if (demandElement->getTagProperty().isVehicleStop()) {
                GNEEdge* edgeStop = nullptr;
                if (demandElement->getParentAdditionals().size() > 0) {
                    edgeStop = demandElement->getParentAdditionals().front()->getParentLanes().front()->getParentEdge();
                } else {
                    edgeStop = demandElement->getParentLanes().front()->getParentEdge();
                }
                if (edgeStop) {
                    // avoid double edge stops
                    if (stops.empty()) {
                        edgeStops.push_back(edgeStop);
                    } else if (edgeStops.back() != edgeStop) {
                        edgeStops.push_back(edgeStop);
                    }
                }
            }
        }
        // declare edge vector
        std::vector<GNEEdge*> edgePath;
        // get first and last lanes
        const auto firstLane = getFirstPathLane();
        const auto lastLane = getLastPathLane();
        // check first and last lanes
        if (firstLane && lastLane) {
            // add first lane
            edgePath.push_back(firstLane->getParentEdge());
            // give more priority to stops instead via
            if (edgeStops.size() > 0) {
                // add stops only if they're accesibles
                for (const auto& edgeStop : edgeStops) {
                    // check if exist a valid path that includes the last edge
                    auto edgePathStop = edgePath;
                    edgePathStop.push_back(edgeStop);
                    edgePathStop.push_back(lastLane->getParentEdge());
                    auto path = myNet->getPathManager()->getPathCalculator()->calculateDijkstraPath(getVClass(), edgePathStop);
                    if (path.size() > 0) {
                        edgePath.push_back(edgeStop);
                    }
                }
            } else {
                // add via lanes
                for (const auto& edgeViaID : via) {
                    const auto edgeVia = myNet->getAttributeCarriers()->retrieveEdge(edgeViaID, false);
                    if (edgeVia) {
                        // check if exist a valid path that includes the last edge
                        auto edgePathStop = edgePath;
                        edgePathStop.push_back(edgeVia);
                        edgePathStop.push_back(lastLane->getParentEdge());
                        if (myNet->getPathManager()->getPathCalculator()->calculateDijkstraPath(getVClass(), edgePathStop).size() > 0) {
                            edgePath.push_back(edgeVia);
                        }
                    }
                }
            }
            // add last lane
            edgePath.push_back(lastLane->getParentEdge());
            // calculate path
            myNet->getPathManager()->calculatePath(this, getVClass(), edgePath);
        }
    }
    // update geometry
    updateGeometry();
}


void
GNEVehicle::drawLanePartialGL(const GUIVisualizationSettings& s, const GNEPathManager::Segment* segment, const double offsetFront) const {
    // conditions for draw always in network mode
    const bool drawInNetworkMode = myNet->getViewNet()->getEditModes().isCurrentSupermodeNetwork() &&
                                   myNet->getViewNet()->getNetworkViewOptions().showDemandElements() &&
                                   myNet->getViewNet()->getDemandViewOptions().showAllTrips();
    // conditions for draw always in demand mode
    const bool drawInDemandMode = myNet->getViewNet()->getEditModes().isCurrentSupermodeDemand() &&
                                  myNet->getViewNet()->getDemandViewOptions().showAllTrips();
    // conditions for draw if is selected
    const bool isSelected = myNet->getViewNet()->getEditModes().isCurrentSupermodeDemand() &&
                            isAttributeCarrierSelected();
    // conditions for draw if is inspected
    const bool isInspected = myNet->getViewNet()->getEditModes().isCurrentSupermodeDemand() &&
                             myNet->getViewNet()->isAttributeCarrierInspected(this);
    // check drawing conditions
    if (segment->getLane() && !s.drawForRectangleSelection && (drawInNetworkMode || drawInDemandMode || isSelected || isInspected) &&
            myNet->getPathManager()->getPathDraw()->checkDrawPathGeometry(s, segment->getLane(), myTagProperty.getTag())) {
        // get detail level
        const auto d = s.getDetailLevel(1);
        // calculate width
        const double width = s.vehicleSize.getExaggeration(s, segment->getLane()) * s.widthSettings.tripWidth;
        // calculate startPos
        const double geometryDepartPos = (getParentJunctions().size() > 0) ? 0 : getAttributeDouble(SUMO_ATTR_DEPARTPOS) + getTypeParent()->getAttributeDouble(SUMO_ATTR_LENGTH);
        // get endPos
        const double geometryEndPos = (getParentJunctions().size() > 0) ? segment->getLane()->getLaneGeometry().getShape().length2D() : getAttributeDouble(SUMO_ATTR_ARRIVALPOS);
        // declare path geometry
        GUIGeometry vehicleGeometry;
        // update pathGeometry depending of first and last segment
        if (segment->isFirstSegment() && segment->isLastSegment()) {
            vehicleGeometry.updateGeometry(segment->getLane()->getLaneGeometry().getShape(),
                                           geometryDepartPos,
                                           Position::INVALID,
                                           geometryEndPos,
                                           Position::INVALID);
        } else if (segment->isFirstSegment()) {
            vehicleGeometry.updateGeometry(segment->getLane()->getLaneGeometry().getShape(),
                                           geometryDepartPos,
                                           Position::INVALID,
                                           -1,
                                           Position::INVALID);
        } else if (segment->isLastSegment()) {
            vehicleGeometry.updateGeometry(segment->getLane()->getLaneGeometry().getShape(),
                                           -1,
                                           Position::INVALID,
                                           geometryEndPos,
                                           Position::INVALID);
        } else {
            vehicleGeometry = segment->getLane()->getLaneGeometry();
        }
        // draw geometry only if we'rent in drawForObjectUnderCursor mode
        if (s.checkDrawVehicle(d, isAttributeCarrierSelected())) {
            // obtain color
            const RGBColor pathColor = drawUsingSelectColor() ? s.colorSettings.selectedVehicleColor : s.colorSettings.vehicleTripColor;
            // Add a draw matrix
            GLHelper::pushMatrix();
            // Start with the drawing of the area translating matrix to origin
            glTranslated(0, 0, getType() + offsetFront);
            // Set color
            GLHelper::setColor(pathColor);
            // draw geometry
            GUIGeometry::drawGeometry(d, vehicleGeometry, width);
            // Pop last matrix
            GLHelper::popMatrix();
            // check if we have to draw a red line to the next segment (if next segment isnt' a junction
            if (segment->getNextLane()) {
                // push draw matrix
                GLHelper::pushMatrix();
                // Start with the drawing of the area translating matrix to origin
                myNet->getViewNet()->drawTranslateFrontAttributeCarrier(this, getType());
                // Set red color
                GLHelper::setColor(RGBColor::RED);
                // get firstPosition (last position of current lane shape)
                const Position& firstPosition = segment->getLane()->getLaneShape().back();
                // get lastPosition (first position of next lane shape)
                const Position& arrivalPosition = segment->getNextLane()->getLaneShape().front();
                // draw box line
                GLHelper::drawBoxLine(arrivalPosition,
                                      RAD2DEG(firstPosition.angleTo2D(arrivalPosition)) - 90,
                                      firstPosition.distanceTo2D(arrivalPosition), .05);
                // pop draw matrix
                GLHelper::popMatrix();
            }
            // check if this is the last segment
            if (segment->isLastSegment() && (getParentJunctions().size() == 0)) {
                // get geometryEndPos
                const Position geometryEndPosition = getAttributePosition(GNE_ATTR_PLAN_GEOMETRY_ENDPOS);
                // check if endPos can be drawn
                if (!s.drawForRectangleSelection || (myNet->getViewNet()->getPositionInformation().distanceSquaredTo2D(geometryEndPosition) <= ((myArrivalPositionDiameter * myArrivalPositionDiameter) + 2))) {
                    // push draw matrix
                    GLHelper::pushMatrix();
                    // Start with the drawing of the area translating matrix to origin
                    myNet->getViewNet()->drawTranslateFrontAttributeCarrier(this, getType());
                    // translate to geometryEndPos
                    glTranslated(geometryEndPosition.x(), geometryEndPosition.y(), 0);
                    // Set person plan color
                    GLHelper::setColor(pathColor);
                    // resolution of drawn circle depending of the zoom (To improve smoothness)
                    GLHelper::drawFilledCircleDetailled(d, myArrivalPositionDiameter);
                    // pop draw matrix
                    GLHelper::popMatrix();
                }
            }
            // Draw name if isn't being drawn for selecting
            drawName(getCenteringBoundary().getCenter(), s.scale, s.addName);
            // draw dotted contour
            segment->getContour()->drawDottedContours(s, d, this, s.dottedContourSettings.segmentWidth, true);
        }
        // calculate contour and draw dotted geometry
        if (segment->isFirstSegment() || segment->isLastSegment()) {
            segment->getContour()->calculateContourExtrudedShape(s, d, this, vehicleGeometry.getShape(), getType(), width, 1, segment->isFirstSegment(), segment->isLastSegment(), 0);
        } else {
            segment->getContour()->calculateContourExtrudedShape(s, d, this, segment->getLane()->getLaneShape(), getType(), width, 1, segment->isFirstSegment(), segment->isLastSegment(), 0);
        }
    }
}


void
GNEVehicle::drawJunctionPartialGL(const GUIVisualizationSettings& s, const GNEPathManager::Segment* segment, const double offsetFront) const {
    // conditions for draw always in network mode
    const bool drawInNetworkMode = myNet->getViewNet()->getEditModes().isCurrentSupermodeNetwork() &&
                                   myNet->getViewNet()->getNetworkViewOptions().showDemandElements() &&
                                   myNet->getViewNet()->getDemandViewOptions().showAllTrips();
    // conditions for draw always in demand mode
    const bool drawInDemandMode = myNet->getViewNet()->getEditModes().isCurrentSupermodeDemand() &&
                                  myNet->getViewNet()->getDemandViewOptions().showAllTrips();
    // conditions for draw if is selected
    const bool isSelected = myNet->getViewNet()->getEditModes().isCurrentSupermodeDemand() &&
                            isAttributeCarrierSelected();
    // conditions for draw if is inspected
    const bool isInspected = myNet->getViewNet()->getEditModes().isCurrentSupermodeDemand() &&
                             myNet->getViewNet()->isAttributeCarrierInspected(this);
    // check drawing conditions
    if (segment->getJunction() && !s.drawForRectangleSelection && (drawInNetworkMode || drawInDemandMode || isSelected || isInspected) &&
            myNet->getPathManager()->getPathDraw()->checkDrawPathGeometry(s, segment, myTagProperty.getTag())) {
        // get detail level
        const auto d = s.getDetailLevel(1);
        // calculate width
        const double width = s.vehicleSize.getExaggeration(s, segment->getPreviousLane()) * s.widthSettings.tripWidth;
        // draw geometry only if we'rent in drawForObjectUnderCursor mode
        if (s.checkDrawVehicle(d, isAttributeCarrierSelected())) {
            // Add a draw matrix
            GLHelper::pushMatrix();
            // Start with the drawing of the area translating matrix to origin
            glTranslated(0, 0, getType() + offsetFront);
            // Set color of the base
            if (drawUsingSelectColor()) {
                GLHelper::setColor(s.colorSettings.selectedVehicleColor);
            } else {
                GLHelper::setColor(s.colorSettings.vehicleTripColor);
            }
            // continue depending if we're in the middle of two lanes or in the begin/end of a junction route
            if (segment->getPreviousLane() && segment->getNextLane()) {
                // draw lane2lane
                GUIGeometry::drawGeometry(d, segment->getPreviousLane()->getLane2laneConnections().getLane2laneGeometry(segment->getNextLane()), width);
            } else if (segment->getPreviousLane() && myTagProperty.vehicleJunctions()) {
                // draw line between center of junction and last lane shape
                GLHelper::drawBoxLines({segment->getPreviousLane()->getLaneShape().back(), getParentJunctions().back()->getPositionInView()}, width);
            } else if (segment->getNextLane() && myTagProperty.vehicleJunctions()) {
                // draw line between center of junction and first lane shape
                GLHelper::drawBoxLines({getParentJunctions().front()->getPositionInView(), segment->getNextLane()->getLaneShape().front()}, width);
            }
            // Pop last matrix
            GLHelper::popMatrix();
            // draw dotted contour
            segment->getContour()->drawDottedContours(s, d, this, s.dottedContourSettings.segmentWidth, true);
        }
        // continue depending if we're in the middle of two lanes or in the begin/end of a junction route
        if (segment->getPreviousLane() && segment->getNextLane()) {
            // calculate contour and draw dotted geometry
            segment->getContour()->calculateContourExtrudedShape(s, d, this, segment->getPreviousLane()->getLane2laneConnections().getLane2laneGeometry(segment->getNextLane()).getShape(),
                    getType(), 0, width, 1, false, false);
        } else if (segment->getPreviousLane() && myTagProperty.vehicleJunctions()) {
            segment->getContour()->calculateContourExtrudedShape(s, d, this, {segment->getPreviousLane()->getLaneShape().back(), getParentJunctions().back()->getPositionInView()},
                    getType(), 0, width, 1, true, true);
        } else if (segment->getNextLane() && myTagProperty.vehicleJunctions()) {
            segment->getContour()->calculateContourExtrudedShape(s, d, this, {getParentJunctions().front()->getPositionInView(), segment->getNextLane()->getLaneShape().front()},
                    getType(), 0, width, 1, true, true);
        }
    }
}


GNELane*
GNEVehicle::getFirstPathLane() const {
    // declare first edge
    GNEEdge* firstEdge = nullptr;
    // continue depending of tags
    if (myTagProperty.vehicleRoute()) {
        // check departEdge
        if ((departEdge > 0) && (departEdge < (int)getRouteParent()->getParentEdges().size())) {
            // use departEdge
            firstEdge = getRouteParent()->getParentEdges().at(departEdge);
        } else {
            // use first route edge
            firstEdge = getRouteParent()->getParentEdges().front();
        }
    } else if (myTagProperty.vehicleRouteEmbedded()) {
        // check if embedded route exist (due during loading embedded route doesn't exist)
        if (getChildDemandElements().empty()) {
            return nullptr;
        }
        // check departEdge
        if ((departEdge > 0) && (departEdge < (int)getChildDemandElements().front()->getParentEdges().size())) {
            // use depart edge
            firstEdge = getChildDemandElements().front()->getParentEdges().at(departEdge);
        } else if (getChildDemandElements().front()->getParentEdges().size() > 0) {
            firstEdge = getChildDemandElements().front()->getParentEdges().front();
        } else if (getChildDemandElements().front()->getParentLanes().size() > 0) {
            firstEdge = getChildDemandElements().front()->getParentLanes().front()->getParentEdge();
        } else {
            return nullptr;
        }
    } else if (getParentEdges().size() > 0) {
        // use first parent edge
        firstEdge = getParentEdges().front();
    } else {
        // defined over junctions
        return nullptr;
    }
    // get departLane index
    const int departLaneIndex = (int)getAttributeDouble(SUMO_ATTR_DEPARTLANE);
    // check departLane index
    if ((departLaneIndex >= 0) && (departLaneIndex < (int)firstEdge->getLanes().size())) {
        return firstEdge->getLanes().at(departLaneIndex);
    } else {
        // get first allowed VClass
        return firstEdge->getLaneByAllowedVClass(getVClass());
    }
}


GNELane*
GNEVehicle::getLastPathLane() const {
    // declare last edge
    GNEEdge* lastEdge = nullptr;
    // continue depending of tags
    if (myTagProperty.vehicleRoute()) {
        // check arrivalEdge
        if ((arrivalEdge > 0) && (arrivalEdge < (int)getRouteParent()->getParentEdges().size())) {
            // use arrival edge
            lastEdge = getRouteParent()->getParentEdges().at(arrivalEdge);
        } else {
            // use last route edge
            lastEdge = getRouteParent()->getParentEdges().back();
        }
    } else if (myTagProperty.vehicleRouteEmbedded()) {
        // check if embedded route exist (due during loading embedded route doesn't exist)
        if (getChildDemandElements().empty()) {
            return nullptr;
        }
        // check arrivalEdge
        if ((arrivalEdge > 0) && (arrivalEdge < (int)getChildDemandElements().front()->getParentEdges().size())) {
            // use arrival edge
            lastEdge = getChildDemandElements().front()->getParentEdges().at(arrivalEdge);
        } else if (getChildDemandElements().front()->getParentEdges().size() > 0) {
            // use last route edge
            lastEdge = getChildDemandElements().front()->getParentEdges().back();
        } else if (getChildDemandElements().front()->getParentLanes().size() > 0) {
            // use lane
            lastEdge = getChildDemandElements().front()->getParentLanes().back()->getParentEdge();
        } else {
            return nullptr;
        }
    } else if (getParentEdges().size() > 0) {
        // use last parent edge
        lastEdge = getParentEdges().back();
    } else {
        // defined over junctions
        return nullptr;
    }
    // get arrivalLane index
    const int arrivalLaneIndex = (int)getAttributeDouble(SUMO_ATTR_ARRIVALLANE);
    // check arrivalLane index
    if ((arrivalLaneIndex >= 0) && (arrivalLaneIndex < (int)lastEdge->getLanes().size())) {
        return lastEdge->getLanes().at(arrivalLaneIndex);
    } else {
        // get last allowed VClass
        return lastEdge->getLaneByAllowedVClass(getVClass());
    }
}


std::string
GNEVehicle::getAttribute(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_ID:
            return getMicrosimID();
        case SUMO_ATTR_TYPE:
            return vtypeid;
        case SUMO_ATTR_COLOR:
            if (wasSet(VEHPARS_COLOR_SET)) {
                return toString(color);
            } else {
                return myTagProperty.getDefaultValue(SUMO_ATTR_COLOR);
            }
        case SUMO_ATTR_DEPARTLANE:
            if (wasSet(VEHPARS_DEPARTLANE_SET)) {
                return getDepartLane();
            } else {
                return myTagProperty.getDefaultValue(SUMO_ATTR_DEPARTLANE);
            }
        case SUMO_ATTR_DEPARTPOS:
            if (wasSet(VEHPARS_DEPARTPOS_SET)) {
                return getDepartPos();
            } else {
                return myTagProperty.getDefaultValue(SUMO_ATTR_DEPARTPOS);
            }
        case SUMO_ATTR_DEPARTSPEED:
            if (wasSet(VEHPARS_DEPARTSPEED_SET)) {
                return getDepartSpeed();
            } else {
                return myTagProperty.getDefaultValue(SUMO_ATTR_DEPARTSPEED);
            }
        case SUMO_ATTR_ARRIVALLANE:
            if (wasSet(VEHPARS_ARRIVALLANE_SET)) {
                return getArrivalLane();
            } else {
                return myTagProperty.getDefaultValue(SUMO_ATTR_ARRIVALLANE);
            }
        case SUMO_ATTR_ARRIVALPOS:
            if (wasSet(VEHPARS_ARRIVALPOS_SET)) {
                return getArrivalPos();
            } else {
                return myTagProperty.getDefaultValue(SUMO_ATTR_ARRIVALPOS);
            }
        case SUMO_ATTR_ARRIVALSPEED:
            if (wasSet(VEHPARS_ARRIVALSPEED_SET)) {
                return getArrivalSpeed();
            } else {
                return myTagProperty.getDefaultValue(SUMO_ATTR_ARRIVALSPEED);
            }
        case SUMO_ATTR_LINE:
            if (wasSet(VEHPARS_LINE_SET)) {
                return line;
            } else {
                return myTagProperty.getDefaultValue(SUMO_ATTR_LINE);
            }
        case SUMO_ATTR_PERSON_NUMBER:
            if (wasSet(VEHPARS_PERSON_NUMBER_SET)) {
                return toString(personNumber);
            } else {
                return myTagProperty.getDefaultValue(SUMO_ATTR_PERSON_NUMBER);
            }
        case SUMO_ATTR_CONTAINER_NUMBER:
            if (wasSet(VEHPARS_CONTAINER_NUMBER_SET)) {
                return toString(containerNumber);
            } else {
                return myTagProperty.getDefaultValue(SUMO_ATTR_CONTAINER_NUMBER);
            }
        case SUMO_ATTR_REROUTE:
            if (wasSet(VEHPARS_CONTAINER_NUMBER_SET)) {
                return "true";
            } else {
                return "false";
            }
        case SUMO_ATTR_DEPARTPOS_LAT:
            if (wasSet(VEHPARS_DEPARTPOSLAT_SET)) {
                return getDepartPosLat();
            } else {
                return myTagProperty.getDefaultValue(SUMO_ATTR_DEPARTPOS_LAT);
            }
        case SUMO_ATTR_ARRIVALPOS_LAT:
            if (wasSet(VEHPARS_ARRIVALPOSLAT_SET)) {
                return getArrivalPosLat();
            } else {
                return myTagProperty.getDefaultValue(SUMO_ATTR_ARRIVALPOS_LAT);
            }
        case SUMO_ATTR_INSERTIONCHECKS:
            return getInsertionChecks();
        // Specific of vehicles over routes
        case SUMO_ATTR_ROUTE:
            if (getParentDemandElements().size() == 2) {
                return getRouteParent()->getID();
            } else {
                return "";
            }
        // Specific of from-to edge
        case SUMO_ATTR_FROM:
            return getParentEdges().front()->getID();
        case SUMO_ATTR_TO:
            return getParentEdges().back()->getID();
        case SUMO_ATTR_VIA:
            return toString(via);
        case SUMO_ATTR_DEPARTEDGE:
            if (departEdge == -1) {
                return "";
            } else {
                return toString(departEdge);
            }
        case SUMO_ATTR_ARRIVALEDGE:
            if (arrivalEdge == -1) {
                return "";
            } else {
                return toString(arrivalEdge);
            }
        // Specific of from-to junctions
        case SUMO_ATTR_FROM_JUNCTION:
            return getParentJunctions().front()->getID();
        case SUMO_ATTR_TO_JUNCTION:
            return getParentJunctions().back()->getID();
        // Specific of from-to tazs
        case SUMO_ATTR_FROM_TAZ:
            return getParentAdditionals().front()->getID();
        case SUMO_ATTR_TO_TAZ:
            return getParentAdditionals().back()->getID();
        // other
        case GNE_ATTR_SELECTED:
            return toString(isAttributeCarrierSelected());
        case GNE_ATTR_PARAMETERS:
            return getParametersStr();
        case GNE_ATTR_FLOWPARAMETERS:
            return toString(parametersSet);
        default:
            return getFlowAttribute(key);
    }
}


double
GNEVehicle::getAttributeDouble(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_DEPARTLANE:
            if (wasSet(VEHPARS_DEPARTLANE_SET) && (departLaneProcedure == DepartLaneDefinition::GIVEN)) {
                return departLane;
            } else {
                return -1;
            }
        case SUMO_ATTR_DEPARTPOS:
            // only return departPos it if is given
            if (departPosProcedure == DepartPosDefinition::GIVEN) {
                return departPos;
            } else {
                return 0;
            }
        case SUMO_ATTR_ARRIVALLANE:
            if (wasSet(VEHPARS_ARRIVALLANE_SET) && (arrivalLaneProcedure == ArrivalLaneDefinition::GIVEN)) {
                return arrivalLane;
            } else {
                return -1;
            }
        case SUMO_ATTR_ARRIVALPOS:
            // only return departPos it if is given
            if (arrivalPosProcedure == ArrivalPosDefinition::GIVEN) {
                return arrivalPos;
            } else {
                return -1;
            }
        case SUMO_ATTR_WIDTH:
        case SUMO_ATTR_LENGTH:
        case SUMO_ATTR_MINGAP:
            return getTypeParent()->getAttributeDouble(key);
        default:
            return getFlowAttributeDouble(key);
    }
}


Position
GNEVehicle::getAttributePosition(SumoXMLAttr key) const {
    switch (key) {
        case GNE_ATTR_PLAN_GEOMETRY_STARTPOS: {
            // check if this vehicle was defined over junctions
            if (getParentJunctions().size() > 0) {
                return getParentJunctions().front()->getPositionInView();
            } else {
                // get first path lane shape
                const PositionVector& laneShape = getFirstPathLane()->getLaneShape();
                // check arrivalPosProcedure
                if (departPosProcedure == DepartPosDefinition::GIVEN) {
                    if (departPos < 0) {
                        return laneShape.front();
                    } else if (departPos > laneShape.length2D()) {
                        return laneShape.back();
                    } else {
                        return laneShape.positionAtOffset2D(departPos);
                    }
                } else {
                    return laneShape.front();
                }
            }
        }
        case GNE_ATTR_PLAN_GEOMETRY_ENDPOS: {
            // check if this vehicle was defined over junctions
            if (getParentJunctions().size() > 0) {
                return getParentJunctions().back()->getPositionInView();
            } else {
                // get last path lane shape
                const PositionVector& laneShape = getLastPathLane()->getLaneShape();
                // check arrivalPosProcedure
                if (arrivalPosProcedure == ArrivalPosDefinition::GIVEN) {
                    if (arrivalPos < 0) {
                        return laneShape.front();
                    } else if (arrivalPos > laneShape.length2D()) {
                        return laneShape.back();
                    } else {
                        return laneShape.positionAtOffset2D(arrivalPos);
                    }
                } else {
                    return laneShape.back();
                }
            }
        }
        default:
            throw InvalidArgument(getTagStr() + " doesn't have a double attribute of type '" + toString(key) + "'");
    }
}


void
GNEVehicle::setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) {
    if (value == getAttribute(key)) {
        return; //avoid needless changes, later logic relies on the fact that attributes have changed
    }
    switch (key) {
        case SUMO_ATTR_ID:
        case SUMO_ATTR_TYPE:
        case SUMO_ATTR_COLOR:
        case SUMO_ATTR_DEPARTLANE:
        case SUMO_ATTR_DEPARTPOS:
        case SUMO_ATTR_DEPARTSPEED:
        case SUMO_ATTR_ARRIVALLANE:
        case SUMO_ATTR_ARRIVALPOS:
        case SUMO_ATTR_ARRIVALSPEED:
        case SUMO_ATTR_LINE:
        case SUMO_ATTR_PERSON_NUMBER:
        case SUMO_ATTR_CONTAINER_NUMBER:
        case SUMO_ATTR_REROUTE:
        case SUMO_ATTR_DEPARTPOS_LAT:
        case SUMO_ATTR_ARRIVALPOS_LAT:
        case SUMO_ATTR_INSERTIONCHECKS:
        // Specific of vehicles over routes
        case SUMO_ATTR_ROUTE:
        // Specific of from-to edges
        case SUMO_ATTR_FROM:
        case SUMO_ATTR_TO:
        case SUMO_ATTR_VIA:
        case SUMO_ATTR_DEPARTEDGE:
        case SUMO_ATTR_ARRIVALEDGE:
        // Specific of from-to junctions
        case SUMO_ATTR_FROM_JUNCTION:
        case SUMO_ATTR_TO_JUNCTION:
        // Specific of from-to taz
        case SUMO_ATTR_FROM_TAZ:
        case SUMO_ATTR_TO_TAZ:
        // other
        case GNE_ATTR_PARAMETERS:
        case GNE_ATTR_SELECTED:
            GNEChange_Attribute::changeAttribute(this, key, value, undoList);
            break;
        default:
            setFlowAttribute(this, key, value, undoList);
            break;
    }
}


bool
GNEVehicle::isValid(SumoXMLAttr key, const std::string& value) {
    // get ACs
    const auto ACs = myNet->getAttributeCarriers();
    // declare string error
    std::string error;
    switch (key) {
        case SUMO_ATTR_ID:
            return isValidDemandElementID(NamespaceIDs::vehicles, value);
        case SUMO_ATTR_TYPE:
            return (myNet->getAttributeCarriers()->retrieveDemandElements(NamespaceIDs::types, value, false) != nullptr);
        case SUMO_ATTR_COLOR:
            return canParse<RGBColor>(value);
        case SUMO_ATTR_DEPARTLANE: {
            int dummyDepartLane;
            DepartLaneDefinition dummyDepartLaneProcedure;
            parseDepartLane(value, myTagProperty.getTagStr(), id, dummyDepartLane, dummyDepartLaneProcedure, error);
            // if error is empty, check if depart lane is correct
            if (error.empty()) {
                if (dummyDepartLaneProcedure != DepartLaneDefinition::GIVEN) {
                    return true;
                } else if (isTemplate()) {
                    return true;
                } else if (getParentJunctions().size() > 0) {
                    return (dummyDepartLane == 0);
                } else if (getParentAdditionals().size() > 0) {
                    return (dummyDepartLane == 0);
                } else {
                    return dummyDepartLane < (int)getFirstPathLane()->getParentEdge()->getLanes().size();
                }
            } else {
                return false;
            }
        }
        case SUMO_ATTR_DEPARTPOS: {
            double dummyDepartPos;
            DepartPosDefinition dummyDepartPosProcedure;
            parseDepartPos(value, myTagProperty.getTagStr(), id, dummyDepartPos, dummyDepartPosProcedure, error);
            // if error is empty, given value is valid
            return error.empty();
        }
        case SUMO_ATTR_DEPARTSPEED: {
            double dummyDepartSpeed;
            DepartSpeedDefinition dummyDepartSpeedProcedure;
            parseDepartSpeed(value, myTagProperty.getTagStr(), id, dummyDepartSpeed, dummyDepartSpeedProcedure, error);
            // if error is empty, check if depart speed is correct
            if (error.empty()) {
                if (dummyDepartSpeedProcedure != DepartSpeedDefinition::GIVEN) {
                    return true;
                } else if (isTemplate()) {
                    return true;
                } else {
                    return (dummyDepartSpeed <= getTypeParent()->getAttributeDouble(SUMO_ATTR_MAXSPEED));
                }
            } else {
                return false;
            }
        }
        case SUMO_ATTR_ARRIVALLANE: {
            int dummyArrivalLane;
            ArrivalLaneDefinition dummyArrivalLaneProcedure;
            parseArrivalLane(value, myTagProperty.getTagStr(), id, dummyArrivalLane, dummyArrivalLaneProcedure, error);
            // if error is empty, given value is valid
            if (error.empty()) {
                if (dummyArrivalLaneProcedure != ArrivalLaneDefinition::GIVEN) {
                    return true;
                } else if (isTemplate()) {
                    return true;
                } else if (getParentJunctions().size() > 0) {
                    return (dummyArrivalLane == 0);
                } else if (getParentAdditionals().size() > 0) {
                    return (dummyArrivalLane == 0);
                } else {
                    return dummyArrivalLane < (int)getLastPathLane()->getParentEdge()->getLanes().size();
                }
            } else {
                return false;
            }
        }
        case SUMO_ATTR_ARRIVALPOS: {
            double dummyArrivalPos;
            ArrivalPosDefinition dummyArrivalPosProcedure;
            parseArrivalPos(value, myTagProperty.getTagStr(), id, dummyArrivalPos, dummyArrivalPosProcedure, error);
            // if error is empty, given value is valid
            return error.empty();
        }
        case SUMO_ATTR_ARRIVALSPEED: {
            double dummyArrivalSpeed;
            ArrivalSpeedDefinition dummyArrivalSpeedProcedure;
            parseArrivalSpeed(value, myTagProperty.getTagStr(), id, dummyArrivalSpeed, dummyArrivalSpeedProcedure, error);
            // if error is empty, given value is valid
            return error.empty();
        }
        case SUMO_ATTR_LINE:
            return true;
        case SUMO_ATTR_PERSON_NUMBER:
            return canParse<int>(value) && parse<int>(value) >= 0;
        case SUMO_ATTR_CONTAINER_NUMBER:
            return canParse<int>(value) && parse<int>(value) >= 0;
        case SUMO_ATTR_REROUTE:
            return true;    // check
        case SUMO_ATTR_DEPARTPOS_LAT: {
            double dummyDepartPosLat;
            DepartPosLatDefinition dummyDepartPosLatProcedure;
            parseDepartPosLat(value, myTagProperty.getTagStr(), id, dummyDepartPosLat, dummyDepartPosLatProcedure, error);
            // if error is empty, given value is valid
            return error.empty();
        }
        case SUMO_ATTR_ARRIVALPOS_LAT: {
            double dummyArrivalPosLat;
            ArrivalPosLatDefinition dummyArrivalPosLatProcedure;
            parseArrivalPosLat(value, myTagProperty.getTagStr(), id, dummyArrivalPosLat, dummyArrivalPosLatProcedure, error);
            // if error is empty, given value is valid
            return error.empty();
        }
        case SUMO_ATTR_INSERTIONCHECKS:
            return areInsertionChecksValid(value);
        // Specific of vehicles over routes
        case SUMO_ATTR_ROUTE:
            if (getParentDemandElements().size() == 2) {
                return SUMOXMLDefinitions::isValidVehicleID(value) && (ACs->retrieveDemandElement(SUMO_TAG_ROUTE, value, false) != nullptr);
            } else {
                return true;
            }
        // Specific of from-to edges
        case SUMO_ATTR_FROM:
        case SUMO_ATTR_TO:
            return (ACs->retrieveEdge(value, false) != nullptr);
        case SUMO_ATTR_DEPARTEDGE:
        case SUMO_ATTR_ARRIVALEDGE: {
            if (value.empty()) {
                return true;
            } else if (canParse<int>(value)) {
                // get index
                const int index = parse<int>(value);
                // check conditions
                if (index < 0) {
                    return false;
                } else if (myTagProperty.vehicleRoute()) {
                    // check parent route
                    return (index < (int)getRouteParent()->getParentEdges().size());
                } else {
                    // check embedded route
                    return (index < (int)getChildDemandElements().front()->getParentEdges().size());
                }
            } else {
                return false;
            }
        }
        case SUMO_ATTR_VIA:
            if (value.empty()) {
                return true;
            } else {
                return canParse<std::vector<GNEEdge*> >(myNet, value, false);
            }
        // Specific of from-to junctions
        case SUMO_ATTR_FROM_JUNCTION:
        case SUMO_ATTR_TO_JUNCTION:
            return (ACs->retrieveJunction(value, false) != nullptr);
        // Specific of from-to taz
        case SUMO_ATTR_FROM_TAZ:
        case SUMO_ATTR_TO_TAZ:
            return (ACs->retrieveAdditional(SUMO_TAG_TAZ, value, false) != nullptr);
        // other
        case GNE_ATTR_SELECTED:
            return canParse<bool>(value);
        case GNE_ATTR_PARAMETERS:
            return Parameterised::areParametersValid(value);
        default:
            return isValidFlowAttribute(this, key, value);
    }
}


void
GNEVehicle::enableAttribute(SumoXMLAttr key, GNEUndoList* undoList) {
    enableFlowAttribute(this, key, undoList);
}


void
GNEVehicle::disableAttribute(SumoXMLAttr key, GNEUndoList* undoList) {
    disableFlowAttribute(this, key, undoList);
}


bool
GNEVehicle::isAttributeEnabled(SumoXMLAttr key) const {
    return isFlowAttributeEnabled(key);
}


std::string
GNEVehicle::getPopUpID() const {
    return getTagStr();
}


std::string
GNEVehicle::getHierarchyName() const {
    // special case for Trips and flow
    if (myTagProperty.vehicleEdges()) {
        // check if we're inspecting a Edge
        if (!myNet->getViewNet()->getInspectedAttributeCarriers().empty() &&
                myNet->getViewNet()->getInspectedAttributeCarriers().front()->getTagProperty().getTag() == SUMO_TAG_EDGE) {
            // check if edge correspond to a "from", "to" or "via" edge
            if (myNet->getViewNet()->isAttributeCarrierInspected(getParentEdges().front())) {
                return getTagStr() + ": " + getAttribute(SUMO_ATTR_ID) + " (from)";
            } else if (myNet->getViewNet()->isAttributeCarrierInspected(getParentEdges().front())) {
                return getTagStr() + ": " + getAttribute(SUMO_ATTR_ID) + " (to)";
            } else {
                // iterate over via
                for (const auto& i : via) {
                    if (i == myNet->getViewNet()->getInspectedAttributeCarriers().front()->getID()) {
                        return getTagStr() + ": " + getAttribute(SUMO_ATTR_ID) + " (via)";
                    }
                }
            }
        }
    }
    return getTagStr() + ": " + getAttribute(SUMO_ATTR_ID);
}


const Parameterised::Map&
GNEVehicle::getACParametersMap() const {
    return getParametersMap();
}


GNEDemandElement*
GNEVehicle::copyVehicle(const GNEVehicle* originalVehicle) {
    // get net and undoList
    const auto net = originalVehicle->getNet();
    auto undoList = net->getViewNet()->getUndoList();
    // declare new route, vehicle and embedded route
    GNERoute* newRoute = nullptr;
    GNEVehicle* newVehicle = nullptr;
    GNERoute* newEmbeddedRoute = nullptr;
    // generate new vehicle ID
    const std::string newRouteID = net->getAttributeCarriers()->generateDemandElementID(SUMO_TAG_ROUTE);
    const std::string newVehicleID = net->getAttributeCarriers()->generateDemandElementID(originalVehicle->getTagProperty().getTag());
    // extract vehicle parameters and update ID
    auto newVehicleParameters = originalVehicle->getSUMOVehicleParameter();
    newVehicleParameters.id = newVehicleID;
    // create vehicle using vehicleParameters
    if (originalVehicle->getTagProperty().vehicleRoute()) {
        newRoute = new GNERoute(net, newRouteID, originalVehicle->getParentDemandElements().at(1));
        newVehicle = new GNEVehicle(originalVehicle->getTagProperty().getTag(), net,
                                    originalVehicle->getParentDemandElements().at(0), newRoute,
                                    newVehicleParameters);
    } else if (originalVehicle->getTagProperty().vehicleRouteEmbedded()) {
        newVehicle = new GNEVehicle(originalVehicle->getTagProperty().getTag(), net,
                                    originalVehicle->getParentDemandElements().at(0),
                                    newVehicleParameters);
        newEmbeddedRoute = new GNERoute(net, newVehicle, originalVehicle->getChildDemandElements().front());
    } else if (originalVehicle->getTagProperty().vehicleEdges()) {
        newVehicle = new GNEVehicle(originalVehicle->getTagProperty().getTag(), net,
                                    originalVehicle->getParentDemandElements().at(0),
                                    originalVehicle->getParentEdges().front(),
                                    originalVehicle->getParentEdges().back(),
                                    newVehicleParameters);
    } else if (originalVehicle->getTagProperty().vehicleJunctions()) {
        newVehicle = new GNEVehicle(originalVehicle->getTagProperty().getTag(), net,
                                    originalVehicle->getParentDemandElements().at(0),
                                    originalVehicle->getParentJunctions().front(),
                                    originalVehicle->getParentJunctions().back(),
                                    newVehicleParameters);
    } else if (originalVehicle->getTagProperty().vehicleTAZs()) {
        newVehicle = new GNEVehicle(originalVehicle->getTagProperty().getTag(), net,
                                    originalVehicle->getParentDemandElements().at(0),
                                    originalVehicle->getParentAdditionals().front(),
                                    originalVehicle->getParentAdditionals().back(),
                                    newVehicleParameters);
    }
    // add new vehicle
    undoList->begin(originalVehicle, TLF("copy % '%'", newVehicle->getTagStr(), newVehicleID));
    if (newRoute) {
        net->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(newRoute, true), true);
    }
    undoList->add(new GNEChange_DemandElement(newVehicle, true), true);
    if (newEmbeddedRoute) {
        net->getViewNet()->getUndoList()->add(new GNEChange_DemandElement(newEmbeddedRoute, true), true);
    }
    undoList->end();
    return newVehicle;
}

// ===========================================================================
// protected
// ===========================================================================

RGBColor
GNEVehicle::getDrawingColor(const GUIVisualizationSettings& s) const {
    // change color
    if (drawUsingSelectColor()) {
        return s.colorSettings.selectedVehicleColor;
    } else {
        return getColorByScheme(s.vehicleColorer, this);
    }
}


const SUMOVehicleParameter&
GNEVehicle::getSUMOVehicleParameter() const {
    return *this;
}

// ===========================================================================
// private
// ===========================================================================

void
GNEVehicle::setAttribute(SumoXMLAttr key, const std::string& value) {
    // declare string error
    std::string error;
    // flag to upate stack label
    bool updateSpreadStackGeometry = false;
    switch (key) {
        case SUMO_ATTR_ID:
            // update microsimID
            setDemandElementID(value);
            // set manually vehicle ID (needed for saving)
            id = value;
            break;
        case SUMO_ATTR_TYPE:
            if (getID().size() > 0) {
                if (myNet->getAttributeCarriers()->retrieveDemandElement(SUMO_TAG_VTYPE, value, false) != nullptr) {
                    replaceDemandElementParent(SUMO_TAG_VTYPE, value, 0);
                } else {
                    replaceDemandElementParent(SUMO_TAG_VTYPE_DISTRIBUTION, value, 0);
                }
                // set manually vtypeID (needed for saving)
                vtypeid = value;
            }
            break;
        case SUMO_ATTR_COLOR:
            if (!value.empty() && (value != myTagProperty.getDefaultValue(key))) {
                color = parse<RGBColor>(value);
                // mark parameter as set
                parametersSet |= VEHPARS_COLOR_SET;
            } else {
                // set default value
                color = parse<RGBColor>(myTagProperty.getDefaultValue(key));
                // unset parameter
                parametersSet &= ~VEHPARS_COLOR_SET;
            }
            break;
        case SUMO_ATTR_DEPARTLANE:
            if (!value.empty() && (value != myTagProperty.getDefaultValue(key))) {
                parseDepartLane(value, myTagProperty.getTagStr(), id, departLane, departLaneProcedure, error);
                // mark parameter as set
                parametersSet |= VEHPARS_DEPARTLANE_SET;
            } else {
                // set default value
                parseDepartLane(myTagProperty.getDefaultValue(key), myTagProperty.getTagStr(), id, departLane, departLaneProcedure, error);
                // unset parameter
                parametersSet &= ~VEHPARS_DEPARTLANE_SET;
            }
            break;
        case SUMO_ATTR_DEPARTPOS:
            if (value == toString(INVALID_DOUBLE)) {
                parseDepartPos(value, myTagProperty.getTagStr(), id, departPos, departPosProcedure, error);
                // mark parameter as set
                parametersSet |= VEHPARS_DEPARTPOS_SET;
            } else  if (!value.empty() && (value != myTagProperty.getDefaultValue(key))) {
                parseDepartPos(value, myTagProperty.getTagStr(), id, departPos, departPosProcedure, error);
                // mark parameter as set
                parametersSet |= VEHPARS_DEPARTPOS_SET;
            } else {
                // set default value
                parseDepartPos(myTagProperty.getDefaultValue(key), myTagProperty.getTagStr(), id, departPos, departPosProcedure, error);
                // unset parameter
                parametersSet &= ~VEHPARS_DEPARTPOS_SET;
            }
            if (getID().size() > 0) {
                updateGeometry();
                updateSpreadStackGeometry = true;
            }
            break;
        case SUMO_ATTR_DEPARTSPEED:
            if (!value.empty() && (value != myTagProperty.getDefaultValue(key))) {
                parseDepartSpeed(value, myTagProperty.getTagStr(), id, departSpeed, departSpeedProcedure, error);
                // mark parameter as set
                parametersSet |= VEHPARS_DEPARTSPEED_SET;
            } else {
                // set default value
                parseDepartSpeed(myTagProperty.getDefaultValue(key), myTagProperty.getTagStr(), id, departSpeed, departSpeedProcedure, error);
                // unset parameter
                parametersSet &= ~VEHPARS_DEPARTSPEED_SET;
            }
            break;
        case SUMO_ATTR_ARRIVALLANE:
            if (!value.empty() && (value != myTagProperty.getDefaultValue(key))) {
                parseArrivalLane(value, myTagProperty.getTagStr(), id, arrivalLane, arrivalLaneProcedure, error);
                // mark parameter as set
                parametersSet |= VEHPARS_ARRIVALLANE_SET;
            } else {
                // set default value
                parseArrivalLane(myTagProperty.getDefaultValue(key), myTagProperty.getTagStr(), id, arrivalLane, arrivalLaneProcedure, error);
                // unset parameter
                parametersSet &= ~VEHPARS_ARRIVALLANE_SET;
            }
            break;
        case SUMO_ATTR_ARRIVALPOS:
            if (value == toString(INVALID_DOUBLE)) {
                parseArrivalPos("max", myTagProperty.getTagStr(), id, arrivalPos, arrivalPosProcedure, error);
                // mark parameter as set
                parametersSet |= VEHPARS_ARRIVALPOS_SET;
            } else if (!value.empty() && (value != myTagProperty.getDefaultValue(key))) {
                parseArrivalPos(value, myTagProperty.getTagStr(), id, arrivalPos, arrivalPosProcedure, error);
                // mark parameter as set
                parametersSet |= VEHPARS_ARRIVALPOS_SET;
            } else {
                // set default value
                parseArrivalPos(myTagProperty.getDefaultValue(key), myTagProperty.getTagStr(), id, arrivalPos, arrivalPosProcedure, error);
                // unset parameter
                parametersSet &= ~VEHPARS_ARRIVALPOS_SET;
            }
            if (getID().size() > 0) {
                updateGeometry();
                updateSpreadStackGeometry = true;
            }
            break;
        case SUMO_ATTR_ARRIVALSPEED:
            if (!value.empty() && (value != myTagProperty.getDefaultValue(key))) {
                parseArrivalSpeed(value, myTagProperty.getTagStr(), id, arrivalSpeed, arrivalSpeedProcedure, error);
                // mark parameter as set
                parametersSet |= VEHPARS_ARRIVALSPEED_SET;
            } else {
                // set default value
                parseArrivalSpeed(myTagProperty.getDefaultValue(key), myTagProperty.getTagStr(), id, arrivalSpeed, arrivalSpeedProcedure, error);
                // unset parameter
                parametersSet &= ~VEHPARS_ARRIVALSPEED_SET;
            }
            break;
        case SUMO_ATTR_LINE:
            if (!value.empty() && (value != myTagProperty.getDefaultValue(key))) {
                line = value;
                // mark parameter as set
                parametersSet |= VEHPARS_LINE_SET;
            } else {
                // set default value
                line = myTagProperty.getDefaultValue(key);
                // unset parameter
                parametersSet &= ~VEHPARS_LINE_SET;
            }
            break;
        case SUMO_ATTR_PERSON_NUMBER:
            if (!value.empty() && (value != myTagProperty.getDefaultValue(key))) {
                personNumber = parse<int>(value);
                // mark parameter as set
                parametersSet |= VEHPARS_PERSON_NUMBER_SET;
            } else {
                // set default value
                personNumber = parse<int>(myTagProperty.getDefaultValue(key));
                // unset parameter
                parametersSet &= ~VEHPARS_PERSON_NUMBER_SET;
            }
            break;
        case SUMO_ATTR_CONTAINER_NUMBER:
            if (!value.empty() && (value != myTagProperty.getDefaultValue(key))) {
                containerNumber = parse<int>(value);
                // mark parameter as set
                parametersSet |= VEHPARS_CONTAINER_NUMBER_SET;
            } else {
                // set default value
                containerNumber = parse<int>(myTagProperty.getDefaultValue(key));
                // unset parameter
                parametersSet &= ~VEHPARS_CONTAINER_NUMBER_SET;
            }
            break;
        case SUMO_ATTR_REROUTE:
            if (!value.empty() && (value != myTagProperty.getDefaultValue(key))) {
                // mark parameter as set
                parametersSet |= VEHPARS_ROUTE_SET;
            } else {
                // unset parameter
                parametersSet &= ~VEHPARS_ROUTE_SET;
            }
            break;
        case SUMO_ATTR_DEPARTPOS_LAT:
            if (!value.empty() && (value != myTagProperty.getDefaultValue(key))) {
                parseDepartPosLat(value, myTagProperty.getTagStr(), id, departPosLat, departPosLatProcedure, error);
                // mark parameter as set
                parametersSet |= VEHPARS_DEPARTPOSLAT_SET;
            } else {
                // set default value
                parseDepartPosLat(myTagProperty.getDefaultValue(key), myTagProperty.getTagStr(), id, departPosLat, departPosLatProcedure, error);
                // unset parameter
                parametersSet &= ~VEHPARS_DEPARTPOSLAT_SET;
            }
            break;
        case SUMO_ATTR_ARRIVALPOS_LAT:
            if (!value.empty() && (value != myTagProperty.getDefaultValue(key))) {
                parseArrivalPosLat(value, myTagProperty.getTagStr(), id, arrivalPosLat, arrivalPosLatProcedure, error);
                // mark parameter as set
                parametersSet |= VEHPARS_ARRIVALPOSLAT_SET;
            } else {
                // set default value
                parseArrivalPosLat(myTagProperty.getDefaultValue(key), myTagProperty.getTagStr(), id, arrivalPosLat, arrivalPosLatProcedure, error);
                // unset parameter
                parametersSet &= ~VEHPARS_ARRIVALPOSLAT_SET;
            }
            parseArrivalPosLat(value, myTagProperty.getTagStr(), id, arrivalPosLat, arrivalPosLatProcedure, error);
            break;
        case SUMO_ATTR_INSERTIONCHECKS:
            insertionChecks = parseInsertionChecks(value);
            break;
        // Specific of vehicles over routes
        case SUMO_ATTR_ROUTE:
            if (getParentDemandElements().size() == 2) {
                replaceDemandElementParent(SUMO_TAG_ROUTE, value, 1);
            }
            updateGeometry();
            updateSpreadStackGeometry = true;
            break;
        // Specific of from-to edges
        case SUMO_ATTR_FROM: {
            // change first edge
            replaceFirstParentEdge(value);
            // compute vehicle
            computePathElement();
            updateSpreadStackGeometry = true;
            break;
        }
        case SUMO_ATTR_TO: {
            // change last edge
            replaceLastParentEdge(value);
            // compute vehicle
            computePathElement();
            updateSpreadStackGeometry = true;
            break;
        }
        case SUMO_ATTR_VIA: {
            if (!value.empty()) {
                // set new via edges
                via = parse< std::vector<std::string> >(value);
                // mark parameter as set
                parametersSet |= VEHPARS_VIA_SET;
            } else {
                // clear via
                via.clear();
                // unset parameter
                parametersSet &= ~VEHPARS_VIA_SET;
            }
            // compute vehicle
            computePathElement();
            updateSpreadStackGeometry = true;
            break;
        }
        case SUMO_ATTR_DEPARTEDGE: {
            // update depart edge
            if (value.empty()) {
                // unset parameter
                parametersSet &= ~VEHPARS_DEPARTEDGE_SET;
                departEdge = -1;
                departEdgeProcedure = RouteIndexDefinition::DEFAULT;
            } else {
                // mark parameter as set
                parametersSet |= VEHPARS_DEPARTEDGE_SET;
                departEdge = parse<int>(value);
                departEdgeProcedure = RouteIndexDefinition::GIVEN;
            }
            // compute vehicle
            if (getID().size() > 0) {
                computePathElement();
                updateSpreadStackGeometry = true;
            }
            break;
        }
        case SUMO_ATTR_ARRIVALEDGE: {
            // update arrival edge
            if (value.empty()) {
                // unset parameter
                parametersSet &= ~VEHPARS_ARRIVALEDGE_SET;
                arrivalEdge = -1;
                arrivalEdgeProcedure = RouteIndexDefinition::DEFAULT;
            } else {
                // mark parameter as set
                parametersSet |= VEHPARS_ARRIVALEDGE_SET;
                arrivalEdge = parse<int>(value);
                arrivalEdgeProcedure = RouteIndexDefinition::GIVEN;
            }
            if (getID().size() > 0) {
                // compute vehicle
                computePathElement();
                updateSpreadStackGeometry = true;
            }
            break;
        }
        // Specific of from-to junctions
        case SUMO_ATTR_FROM_JUNCTION: {
            // change first junction
            replaceFirstParentJunction(value);
            // compute vehicle
            computePathElement();
            updateSpreadStackGeometry = true;
            break;
        }
        case SUMO_ATTR_TO_JUNCTION: {
            // change last junction
            replaceLastParentJunction(value);
            // compute vehicle
            computePathElement();
            updateSpreadStackGeometry = true;
            break;
        }
        // Specific of from-to TAZs
        case SUMO_ATTR_FROM_TAZ: {
            // change first additional
            replaceFirstParentAdditional(SUMO_TAG_TAZ, value);
            // set taz manually
            fromTaz = value;
            // compute vehicle
            computePathElement();
            updateSpreadStackGeometry = true;
            break;
        }
        case SUMO_ATTR_TO_TAZ: {
            // change last additional
            replaceLastParentAdditional(SUMO_TAG_TAZ, value);
            // set taz manually
            toTaz = value;
            // compute vehicle
            computePathElement();
            updateSpreadStackGeometry = true;
            break;
        }
        // other
        case GNE_ATTR_SELECTED:
            if (parse<bool>(value)) {
                selectAttributeCarrier();
            } else {
                unselectAttributeCarrier();
            }
            break;
        case GNE_ATTR_PARAMETERS:
            setParametersStr(value);
            break;
        default:
            setFlowAttribute(this, key, value);
            break;
    }
    // check if stack label has to be updated
    if (updateSpreadStackGeometry) {
        if (myTagProperty.vehicleEdges()) {
            getParentEdges().front()->updateVehicleStackLabels();
            getParentEdges().front()->updateVehicleSpreadGeometries();
        } else if (myTagProperty.vehicleRoute()) {
            getRouteParent()->getParentEdges().front()->updateVehicleStackLabels();
            getRouteParent()->getParentEdges().front()->updateVehicleSpreadGeometries();
        } else if (myTagProperty.vehicleRouteEmbedded()) {
            getChildDemandElements().front()->getParentEdges().front()->updateVehicleStackLabels();
            getChildDemandElements().front()->getParentEdges().front()->updateVehicleSpreadGeometries();
        }
    }
}


void
GNEVehicle::toggleAttribute(SumoXMLAttr key, const bool value) {
    // toggle flow attributes
    toggleFlowAttribute(key, value);
}


void
GNEVehicle::setMoveShape(const GNEMoveResult& moveResult) {
    if ((moveResult.newFirstPos != INVALID_DOUBLE) &&
            (moveResult.operationType == GNEMoveOperation::OperationType::TWO_LANES_MOVEFIRST)) {
        // change depart
        departPosProcedure = DepartPosDefinition::GIVEN;
        parametersSet |= VEHPARS_DEPARTPOS_SET;
        departPos = moveResult.newFirstPos;
    }
    if ((moveResult.operationType == GNEMoveOperation::OperationType::ONE_LANE_MOVESECOND) ||
            (moveResult.operationType == GNEMoveOperation::OperationType::TWO_LANES_MOVESECOND)) {
        // change arrival
        arrivalPosProcedure = ArrivalPosDefinition::GIVEN;
        parametersSet |= VEHPARS_ARRIVALPOS_SET;
        arrivalPos = moveResult.newFirstPos;
    }
    // set lateral offset
    myMoveElementLateralOffset = moveResult.firstLaneOffset;
    // update geometry
    updateGeometry();
}


void
GNEVehicle::commitMoveShape(const GNEMoveResult& moveResult, GNEUndoList* undoList) {
    // reset lateral offset
    myMoveElementLateralOffset = 0;
    // check value
    if (moveResult.newFirstPos != INVALID_DOUBLE) {
        // continue depending if we're moving first or last position
        if (moveResult.operationType == GNEMoveOperation::OperationType::TWO_LANES_MOVEFIRST) {
            // begin change attribute
            undoList->begin(this, TLF("departPos of %", getTagStr()));
            // now set departPos
            setAttribute(SUMO_ATTR_DEPARTPOS, toString(moveResult.newFirstPos), undoList);
            // check if depart lane has to be changed
            if (moveResult.newFirstLane) {
                // set new depart lane
                setAttribute(SUMO_ATTR_DEPARTLANE, toString(moveResult.newFirstLane->getIndex()), undoList);
            }
        } else {
            // begin change attribute
            undoList->begin(this, TLF("arrivalPos of %", getTagStr()));
            // now set arrivalPos
            setAttribute(SUMO_ATTR_ARRIVALPOS, toString(moveResult.newFirstPos), undoList);
            // check if arrival lane has to be changed
            if (moveResult.newFirstLane) {
                // set new arrival lane
                setAttribute(SUMO_ATTR_ARRIVALLANE, toString(moveResult.newFirstLane->getIndex()), undoList);
            }
        }
    }
    // end change attribute
    undoList->end();
}

/****************************************************************************/
