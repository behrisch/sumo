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
/// @file    GNEJunction.cpp
/// @author  Jakob Erdmann
/// @date    Feb 2011
///
// A class for visualizing and editing junctions in netedit (adapted from
// GUIJunctionWrapper)
/****************************************************************************/
#include <config.h>

#include <netbuild/NBAlgorithms.h>
#include <netbuild/NBLoadedSUMOTLDef.h>
#include <netbuild/NBNetBuilder.h>
#include <netbuild/NBOwnTLDef.h>
#include <netedit/frames/common/GNEDeleteFrame.h>
#include <netedit/frames/network/GNETLSEditorFrame.h>
#include <netedit/frames/network/GNECreateEdgeFrame.h>
#include <netedit/elements/demand/GNEPlanParents.h>
#include <netedit/frames/demand/GNEVehicleFrame.h>
#include <netedit/frames/demand/GNEPersonFrame.h>
#include <netedit/frames/demand/GNEPersonPlanFrame.h>
#include <netedit/frames/demand/GNEContainerFrame.h>
#include <netedit/frames/demand/GNEContainerPlanFrame.h>
#include <netedit/GNENet.h>
#include <netedit/GNEUndoList.h>
#include <netedit/GNEViewNet.h>
#include <netedit/changes/GNEChange_Attribute.h>
#include <netedit/changes/GNEChange_Connection.h>
#include <netedit/changes/GNEChange_TLS.h>
#include <utils/gui/div/GLHelper.h>
#include <utils/gui/globjects/GLIncludes.h>
#include <utils/gui/globjects/GUIGLObjectPopupMenu.h>
#include <utils/gui/images/GUITextureSubSys.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/options/OptionsCont.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/gui/div/GUIGlobalViewObjectsHandler.h>
#include <netedit/GNEViewParent.h>
#include <netedit/frames/network/GNECreateEdgeFrame.h>

#include "GNEConnection.h"
#include "GNEJunction.h"
#include "GNECrossing.h"
#include "GNEWalkingArea.h"
#include "GNEInternalLane.h"


// ===========================================================================
// method definitions
// ===========================================================================

GNEJunction::GNEJunction(GNENet* net, NBNode* nbn, bool loaded) :
    GNENetworkElement(net, nbn->getID(), GLO_JUNCTION, SUMO_TAG_JUNCTION,
                      GUIIconSubSys::getIcon(GUIIcon::JUNCTION), {}, {}, {}, {}, {}, {}),
                                myNBNode(nbn),
                                myDrawingToggle(new int),
                                myLogicStatus(loaded ? FEATURE_LOADED : FEATURE_GUESSED),
                                myHasValidLogic(loaded),
myTesselation(nbn->getID(), "", RGBColor::MAGENTA, nbn->getShape(), false, true, 0) {
    // update centering boundary without updating grid
    updateCenteringBoundary(false);
}


GNEJunction::~GNEJunction() {
    // delete drawing toggle
    delete myDrawingToggle;
    // delete all GNECrossing
    for (const auto& crossing : myGNECrossings) {
        crossing->decRef();
        if (crossing->unreferenced()) {
            // check if remove it from Attribute Carriers
            if (myNet->getAttributeCarriers()->getCrossings().count(crossing) > 0) {
                myNet->getAttributeCarriers()->deleteCrossing(crossing);
            }
            // show extra information for tests
            WRITE_DEBUG("Deleting unreferenced " + crossing->getTagStr() + " '" + crossing->getID() + "' in GNEJunction destructor");
            delete crossing;
        }
    }
    // delete all GNEWalkingArea
    for (const auto& walkingArea : myGNEWalkingAreas) {
        walkingArea->decRef();
        if (walkingArea->unreferenced()) {
            // check if remove it from Attribute Carriers
            if (myNet->getAttributeCarriers()->getWalkingAreas().count(walkingArea) > 0) {
                myNet->getAttributeCarriers()->deleteWalkingArea(walkingArea);
            }
            // show extra information for tests
            WRITE_DEBUG("Deleting unreferenced " + walkingArea->getTagStr() + " '" + walkingArea->getID() + "' in GNEJunction destructor");
            delete walkingArea;
        }
    }
    if (myAmResponsible) {
        // show extra information for tests
        WRITE_DEBUG("Deleting NBNode of '" + getID() + "' in GNEJunction destructor");
        delete myNBNode;
    }
}


const PositionVector&
GNEJunction::getJunctionShape() const {
    return myNBNode->getShape();
}


void
GNEJunction::updateGeometry() {
    updateGeometryAfterNetbuild(true);
    // trigger rebuilding tesselation
    myExaggeration = 2;
}


void
GNEJunction::updateGeometryAfterNetbuild(bool rebuildNBNodeCrossings) {
    // rebuild crossings
    rebuildGNECrossings(rebuildNBNodeCrossings);
    // clear walking areas
    clearWalkingAreas();
    // clear missing connections
    checkMissingConnections();
}


Position
GNEJunction::getPositionInView() const {
    return myNBNode->getPosition();
}


bool
GNEJunction::checkDrawFromContour() const {
    // get modes and viewParent (for code legibility)
    const auto& modes = myNet->getViewNet()->getEditModes();
    const auto& viewParent = myNet->getViewNet()->getViewParent();
    // continue depending of current status
    if (myNet->getViewNet()->getInspectedAttributeCarriers().size() == 1) {
        // get inspected element
        const auto inspectedAC = myNet->getViewNet()->getInspectedAttributeCarriers().front();
        // check if starts in this junction
        if (inspectedAC->hasAttribute(SUMO_ATTR_FROM_JUNCTION) && (inspectedAC->getAttribute(SUMO_ATTR_FROM_JUNCTION) == getID())) {
            return true;
        }
    } else if (modes.isCurrentSupermodeNetwork()) {
        if (modes.networkEditMode == NetworkEditMode::NETWORK_CREATE_EDGE) {
            if (viewParent->getCreateEdgeFrame()->getJunctionSource()) {
                return viewParent->getCreateEdgeFrame()->getJunctionSource() == this;
            } else {
                return myNet->getViewNet()->getViewObjectsSelector().getJunctionFront() == this;
            }
        }
    } else if (modes.isCurrentSupermodeDemand()) {
        // get current GNEPlanCreator
        GNEPlanCreator* planCreator = nullptr;
        if (modes.demandEditMode == DemandEditMode::DEMAND_PERSON) {
            planCreator = viewParent->getPersonFrame()->getPlanCreator();
        } else if (modes.demandEditMode == DemandEditMode::DEMAND_PERSONPLAN) {
            planCreator = viewParent->getPersonPlanFrame()->getPlanCreator();
        } else if (modes.demandEditMode == DemandEditMode::DEMAND_CONTAINER) {
            planCreator = viewParent->getContainerFrame()->getPlanCreator();
        } else if (modes.demandEditMode == DemandEditMode::DEMAND_CONTAINERPLAN) {
            planCreator = viewParent->getContainerPlanFrame()->getPlanCreator();
        }
        // continue depending of planCreator
        if (planCreator) {
            if (planCreator->getPlanParameteres().fromJunction == getID()) {
                return true;
            }
        } else if (modes.demandEditMode == DemandEditMode::DEMAND_VEHICLE) {
            const auto& selectedJunctions = viewParent->getVehicleFrame()->getPathCreator()->getSelectedJunctions();
            // check if this is the first selected junction
            if ((selectedJunctions.size() > 0) && (selectedJunctions.front() == this)) {
                return true;
            }
        }
    }
    // nothing to draw
    return false;
}


bool
GNEJunction::checkDrawToContour() const {
    // get modes and viewParent (for code legibility)
    const auto& modes = myNet->getViewNet()->getEditModes();
    const auto& viewParent = myNet->getViewNet()->getViewParent();
    // continue depending of current status
    if (myNet->getViewNet()->getInspectedAttributeCarriers().size() == 1) {
        // get inspected element
        const auto inspectedAC = myNet->getViewNet()->getInspectedAttributeCarriers().front();
        // check if ends in this junction
        if (inspectedAC->getTagProperty().vehicleJunctions() && (inspectedAC->getAttribute(SUMO_ATTR_TO_JUNCTION) == getID())) {
            return true;
        }
    } else if (modes.isCurrentSupermodeNetwork()) {
        if (modes.networkEditMode == NetworkEditMode::NETWORK_CREATE_EDGE) {
            if (viewParent->getCreateEdgeFrame()->getJunctionSource() &&
                    (viewParent->getCreateEdgeFrame()->getJunctionSource() != this)) {
                return myNet->getViewNet()->getViewObjectsSelector().getJunctionFront() == this;
            }
        } else if (modes.networkEditMode == NetworkEditMode::NETWORK_MOVE) {
            // check if we're moving a junction
            const auto movedJunction = dynamic_cast<GNEJunction*>(myNet->getViewNet()->getMoveSingleElementValues().getMovedElement());
            if (movedJunction && (movedJunction != this)) {
                // continue depending of junction shape
                if (myNBNode->getShape().area() < 4) {
                    // calculate distance between both centers
                    const double junctionBubbleRadius = myNet->getViewNet()->getVisualisationSettings().neteditSizeSettings.junctionBubbleRadius;
                    const double radiusTo = getExaggeration(myNet->getViewNet()->getVisualisationSettings()) * junctionBubbleRadius;
                    if (myNBNode->getPosition().distanceSquaredTo2D(movedJunction->getPositionInView()) < (radiusTo * radiusTo)) {
                        // add it in the list of merging junction (first the moved junction)
                        gViewObjectsHandler.addMergingJunctions(movedJunction);
                        gViewObjectsHandler.addMergingJunctions(this);
                        return true;
                    }
                } else if (myNBNode->getShape().around(movedJunction->getNBNode()->getPosition())) {
                    // add it in the list of merging junction (first the moved junction)
                    gViewObjectsHandler.addMergingJunctions(movedJunction);
                    gViewObjectsHandler.addMergingJunctions(this);
                    return true;
                }
            }
        }
    } else if (modes.isCurrentSupermodeDemand()) {
        // get current GNEPlanCreator
        GNEPlanCreator* planCreator = nullptr;
        if (modes.demandEditMode == DemandEditMode::DEMAND_PERSON) {
            planCreator = viewParent->getPersonFrame()->getPlanCreator();
        } else if (modes.demandEditMode == DemandEditMode::DEMAND_PERSONPLAN) {
            planCreator = viewParent->getPersonPlanFrame()->getPlanCreator();
        } else if (modes.demandEditMode == DemandEditMode::DEMAND_CONTAINER) {
            planCreator = viewParent->getContainerFrame()->getPlanCreator();
        } else if (modes.demandEditMode == DemandEditMode::DEMAND_CONTAINERPLAN) {
            planCreator = viewParent->getContainerPlanFrame()->getPlanCreator();
        }
        // continue depending of planCreator
        if (planCreator) {
            if (planCreator->getPlanParameteres().toJunction == getID()) {
                return true;
            }
        } else if (modes.demandEditMode == DemandEditMode::DEMAND_VEHICLE) {
            const auto& selectedJunctions = viewParent->getVehicleFrame()->getPathCreator()->getSelectedJunctions();
            // check if this is the first selected junction
            if ((selectedJunctions.size() > 1) && (selectedJunctions.back() == this)) {
                return true;
            }
        }
    }
    // nothing to draw
    return false;
}


bool
GNEJunction::checkDrawRelatedContour() const {
    return false;
}


bool
GNEJunction::checkDrawOverContour() const {
    // get modes and viewParent (for code legibility)
    const auto& modes = myNet->getViewNet()->getEditModes();
    const auto& viewParent = myNet->getViewNet()->getViewParent();
    const auto& viewObjectsSelector = myNet->getViewNet()->getViewObjectsSelector();
    if (viewObjectsSelector.getJunctionFront() != this) {
        return false;
    } else {
        if (modes.isCurrentSupermodeDemand()) {
            // get current plan selector
            GNEPlanSelector* planSelector = nullptr;
            if (modes.demandEditMode == DemandEditMode::DEMAND_PERSON) {
                planSelector = viewParent->getPersonFrame()->getPlanSelector();
            } else if (modes.demandEditMode == DemandEditMode::DEMAND_PERSONPLAN) {
                planSelector = viewParent->getPersonPlanFrame()->getPlanSelector();
            } else if (modes.demandEditMode == DemandEditMode::DEMAND_CONTAINER) {
                planSelector = viewParent->getContainerFrame()->getPlanSelector();
            } else if (modes.demandEditMode == DemandEditMode::DEMAND_CONTAINERPLAN) {
                planSelector = viewParent->getContainerPlanFrame()->getPlanSelector();
            }
            // continue depending of plan selector
            if (planSelector && planSelector->markJunctions()) {
                return (viewObjectsSelector.getAttributeCarrierFront() == viewObjectsSelector.getJunctionFront());
            } else if (modes.demandEditMode == DemandEditMode::DEMAND_VEHICLE) {
                // get current vehicle template
                const auto& vehicleTemplate = viewParent->getVehicleFrame()->getVehicleTagSelector()->getCurrentTemplateAC();
                // check if vehicle can be placed over from-to TAZs
                if (vehicleTemplate && vehicleTemplate->getTagProperty().vehicleJunctions()) {
                    return (viewObjectsSelector.getAttributeCarrierFront() == viewObjectsSelector.getJunctionFront());
                }
            }
        }
        return false;
    }
}


bool
GNEJunction::checkDrawDeleteContour() const {
    // get edit modes
    const auto& editModes = myNet->getViewNet()->getEditModes();
    // check if we're in delete mode
    if (editModes.isCurrentSupermodeNetwork() && (editModes.networkEditMode == NetworkEditMode::NETWORK_DELETE)) {
        return myNet->getViewNet()->checkOverLockedElement(this, mySelected);
    } else {
        return false;
    }
}


bool
GNEJunction::checkDrawSelectContour() const {
    // get edit modes
    const auto& editModes = myNet->getViewNet()->getEditModes();
    // check if we're in select mode
    if (editModes.isCurrentSupermodeNetwork() && (editModes.networkEditMode == NetworkEditMode::NETWORK_SELECT)) {
        return myNet->getViewNet()->checkOverLockedElement(this, mySelected);
    } else {
        return false;
    }
}


bool
GNEJunction::checkDrawMoveContour() const {
    // get edit modes
    const auto& editModes = myNet->getViewNet()->getEditModes();
    // check if we're in move mode
    if (!myNet->getViewNet()->isCurrentlyMovingElements() && editModes.isCurrentSupermodeNetwork() &&
            (editModes.networkEditMode == NetworkEditMode::NETWORK_MOVE) && myNet->getViewNet()->checkOverLockedElement(this, mySelected)) {
        // check if we're editing this network element
        const GNENetworkElement* editedNetworkElement = myNet->getViewNet()->getEditNetworkElementShapes().getEditedNetworkElement();
        if (editedNetworkElement) {
            return editedNetworkElement == this;
        } else {
            // only move the first element
            return myNet->getViewNet()->getViewObjectsSelector().getGUIGlObjectFront() == this;
        }
    } else {
        return false;
    }
}


GNEMoveOperation*
GNEJunction::getMoveOperation() {
    // edit depending if shape is being edited
    if (isShapeEdited()) {
        // calculate move shape operation
        return calculateMoveShapeOperation(this, myNBNode->getShape(), false);
    } else {
        // return move junction position
        return new GNEMoveOperation(this, myNBNode->getPosition());
    }
}


void
GNEJunction::removeGeometryPoint(const Position clickedPosition, GNEUndoList* undoList) {
    // edit depending if shape is being edited
    if (isShapeEdited()) {
        // get original shape
        PositionVector shape = myNBNode->getShape();
        // check shape size
        if (shape.size() > 2) {
            // obtain index
            int index = shape.indexOfClosest(clickedPosition);
            // get snap radius
            const double snap_radius = myNet->getViewNet()->getVisualisationSettings().neteditSizeSettings.junctionGeometryPointRadius;
            // check if we have to create a new index
            if ((index != -1) && shape[index].distanceSquaredTo2D(clickedPosition) < (snap_radius * snap_radius)) {
                // remove geometry point
                shape.erase(shape.begin() + index);
                // commit new shape
                undoList->begin(this, "remove geometry point of " + getTagStr());
                GNEChange_Attribute::changeAttribute(this, SUMO_ATTR_SHAPE, toString(shape), undoList, true);
                undoList->end();
            }
        }
    }
}


void
GNEJunction::rebuildGNECrossings(bool rebuildNBNodeCrossings) {
    // rebuild GNECrossings only if create crossings and walkingAreas in net is enabled
    if (myNet->getNetBuilder()->haveNetworkCrossings()) {
        if (rebuildNBNodeCrossings) {
            // build new NBNode::Crossings and walking areas
            mirrorXLeftHand();
            myNBNode->buildCrossingsAndWalkingAreas();
            mirrorXLeftHand();
        }
        // create a vector to keep retrieved and created crossings
        std::vector<GNECrossing*> retrievedCrossings;
        // iterate over NBNode::Crossings of GNEJunction
        for (const auto& crossing : myNBNode->getCrossingsIncludingInvalid()) {
            // retrieve existent GNECrossing, or create it
            GNECrossing* retrievedGNECrossing = retrieveGNECrossing(crossing.get());
            retrievedCrossings.push_back(retrievedGNECrossing);
            // check if previously this GNECrossings exists, and if true, remove it from myGNECrossings and insert in tree again
            std::vector<GNECrossing*>::iterator retrievedExists = std::find(myGNECrossings.begin(), myGNECrossings.end(), retrievedGNECrossing);
            if (retrievedExists != myGNECrossings.end()) {
                myGNECrossings.erase(retrievedExists);
                // update geometry of retrieved crossing
                retrievedGNECrossing->updateGeometry();
                // update boundary
                retrievedGNECrossing->updateCenteringBoundary(false);
            } else {
                // include reference to created GNECrossing
                retrievedGNECrossing->incRef();
            }
        }
        // delete non retrieved GNECrossings (we don't need to extract if from Tree two times)
        for (const auto& crossing : myGNECrossings) {
            crossing->decRef();
            // check if crossing is selected
            if (crossing->isAttributeCarrierSelected()) {
                crossing->unselectAttributeCarrier();
            }
            // remove it from inspected ACS
            myNet->getViewNet()->removeFromAttributeCarrierInspected(crossing);
            // remove it from net
            myNet->removeGLObjectFromGrid(crossing);
            // remove it from attributeCarriers
            myNet->getAttributeCarriers()->deleteCrossing(crossing);
            if (crossing->unreferenced()) {
                // show extra information for tests
                WRITE_DEBUG("Deleting unreferenced " + crossing->getTagStr() + " in rebuildGNECrossings()");
                delete crossing;
            }
        }
        // copy retrieved (existent and created) GNECrossings to myGNECrossings
        myGNECrossings = retrievedCrossings;
    }
}


void
GNEJunction::mirrorXLeftHand() {
    if (OptionsCont::getOptions().getBool("lefthand")) {
        myNBNode->mirrorX();
        for (NBEdge* e : myNBNode->getEdges()) {
            e->mirrorX();

        }
    }
}


void
GNEJunction::buildTLSOperations(GUISUMOAbstractView& parent, GUIGLObjectPopupMenu* ret, const int numSelectedJunctions) {
    // create menu pane for edge operations
    FXMenuPane* TLSOperations = new FXMenuPane(ret);
    ret->insertMenuPaneChild(TLSOperations);
    new FXMenuCascade(ret, TL("TLS operations"), GUIIconSubSys::getIcon(GUIIcon::MODETLS), TLSOperations);
    // create menu commands for all TLS operations
    FXMenuCommand* mcAddTLS = GUIDesigns::buildFXMenuCommand(TLSOperations, TL("Add TLS"), nullptr, &parent, MID_GNE_JUNCTION_ADDTLS);
    FXMenuCommand* mcAddJoinedTLS = GUIDesigns::buildFXMenuCommand(TLSOperations, TL("Add joined TLS"), nullptr, &parent, MID_GNE_JUNCTION_ADDJOINTLS);
    // check if disable create TLS
    if (myNBNode->getControllingTLS().size() > 0) {
        mcAddTLS->disable();
        mcAddJoinedTLS->disable();
    } else {
        mcAddTLS->enable();
        // check if add joined TLS
        if (isAttributeCarrierSelected() && (numSelectedJunctions > 1)) {
            mcAddJoinedTLS->enable();
        } else {
            mcAddJoinedTLS->disable();
        }
    }
}


GUIGLObjectPopupMenu*
GNEJunction::getPopUpMenu(GUIMainWindow& app, GUISUMOAbstractView& parent) {
    if (myShapeEdited) {
        return getShapeEditedPopUpMenu(app, parent, myNBNode->getShape());
    } else {
        GUIGLObjectPopupMenu* ret = new GUIGLObjectPopupMenu(app, parent, *this);
        // build common commands
        buildPopupHeader(ret, app);
        buildCenterPopupEntry(ret);
        buildNameCopyPopupEntry(ret);
        // build selection and show parameters menu
        myNet->getViewNet()->buildSelectionACPopupEntry(ret, this);
        buildShowParamsPopupEntry(ret);
        buildPositionCopyEntry(ret, app);
        // add separator
        new FXMenuSeparator(ret);
        // check if we're in supermode network
        if (myNet->getViewNet()->getEditModes().isCurrentSupermodeNetwork()) {
            const int numSelectedJunctions = myNet->getAttributeCarriers()->getNumberOfSelectedJunctions();
            const int numEndpoints = (int)myNBNode->getEndPoints().size();
            // check if we're handling a selection
            bool handlingSelection = isAttributeCarrierSelected() && (numSelectedJunctions > 1);
            // check if menu commands has to be disabled
            const bool invalidMode = (myNet->getViewNet()->getEditModes().networkEditMode == NetworkEditMode::NETWORK_CONNECT) ||
                                     (myNet->getViewNet()->getEditModes().networkEditMode == NetworkEditMode::NETWORK_TLS) ||
                                     (myNet->getViewNet()->getEditModes().networkEditMode == NetworkEditMode::NETWORK_CREATE_EDGE);
            // build TLS operation
            if (!invalidMode) {
                buildTLSOperations(parent, ret, numSelectedJunctions);
            }
            // create menu commands
            GUIDesigns::buildFXMenuCommand(ret, TL("Reset edge endpoints"), nullptr, &parent, MID_GNE_JUNCTION_RESET_EDGE_ENDPOINTS);
            FXMenuCommand* mcCustomShape = GUIDesigns::buildFXMenuCommand(ret, TL("Set custom junction shape"), nullptr, &parent, MID_GNE_JUNCTION_EDIT_SHAPE);
            FXMenuCommand* mcResetCustomShape = GUIDesigns::buildFXMenuCommand(ret, TL("Reset junction shape"), nullptr, &parent, MID_GNE_JUNCTION_RESET_SHAPE);
            FXMenuCommand* mcReplaceByGeometryPoint = GUIDesigns::buildFXMenuCommand(ret, TL("Replace junction by geometry point"), nullptr, &parent, MID_GNE_JUNCTION_REPLACE);
            FXMenuCommand* mcSplitJunction = GUIDesigns::buildFXMenuCommand(ret, TLF("Split junction (% end points)", numEndpoints), nullptr, &parent, MID_GNE_JUNCTION_SPLIT);
            FXMenuCommand* mcSplitJunctionAndReconnect = GUIDesigns::buildFXMenuCommand(ret, TL("Split junction and reconnect"), nullptr, &parent, MID_GNE_JUNCTION_SPLIT_RECONNECT);
            // check if is a roundabout
            if (myNBNode->isRoundabout()) {
                GUIDesigns::buildFXMenuCommand(ret, TL("Select roundabout"), nullptr, &parent, MID_GNE_JUNCTION_SELECT_ROUNDABOUT);
            } else {
                // get radius
                const double radius = (myNBNode->getRadius() == NBNode::UNSPECIFIED_RADIUS) ? OptionsCont::getOptions().getFloat("default.junctions.radius") : myNBNode->getRadius();
                const std::string menuEntryInfo = TLF("Convert to roundabout (using junction attribute radius %)", toString(radius));
                FXMenuCommand* mcRoundabout = GUIDesigns::buildFXMenuCommand(ret, menuEntryInfo.c_str(), nullptr, &parent, MID_GNE_JUNCTION_CONVERT_ROUNDABOUT);
                // check if disable depending of number of edges
                if ((getChildEdges().size() < 2) ||
                        ((myGNEIncomingEdges.size() == 1) && (myGNEOutgoingEdges.size() == 1) && (myGNEIncomingEdges[0]->getFromJunction() == myGNEOutgoingEdges[0]->getToJunction()))) {
                    mcRoundabout->disable();
                }
            }
            // check multijunctions
            const std::string multi = ((numSelectedJunctions > 1) && isAttributeCarrierSelected()) ? TLF(" of % junctions", numSelectedJunctions) : "";
            FXMenuCommand* mcClearConnections = GUIDesigns::buildFXMenuCommand(ret, TL("Clear connections") + multi, nullptr, &parent, MID_GNE_JUNCTION_CLEAR_CONNECTIONS);
            FXMenuCommand* mcResetConnections = GUIDesigns::buildFXMenuCommand(ret, TL("Reset connections") + multi, nullptr, &parent, MID_GNE_JUNCTION_RESET_CONNECTIONS);
            // check if current mode  is correct
            if (invalidMode) {
                mcCustomShape->disable();
                mcClearConnections->disable();
                mcResetConnections->disable();
            }
            // check if we're handling a selection
            if (handlingSelection) {
                mcResetCustomShape->setText(TL("Reset junction shapes"));
            }
            // disable mcClearConnections if junction hasn't connections
            if (getGNEConnections().empty()) {
                mcClearConnections->disable();
            }
            // disable mcResetCustomShape if junction doesn't have a custom shape
            if (myNBNode->getShape().size() == 0) {
                mcResetCustomShape->disable();
            }
            // checkIsRemovable requires turnarounds to be computed. This is ugly
            if ((myNBNode->getIncomingEdges().size() == 2) && (myNBNode->getOutgoingEdges().size() == 2)) {
                NBTurningDirectionsComputer::computeTurnDirectionsForNode(myNBNode, false);
            }
            std::string reason = TL("wrong edit mode");
            if (invalidMode || !myNBNode->checkIsRemovableReporting(reason)) {
                mcReplaceByGeometryPoint->setText(mcReplaceByGeometryPoint->getText() + " (" + reason.c_str() + ")");
                mcReplaceByGeometryPoint->disable();
            }
            // check if disable split junctions
            if (numEndpoints == 1) {
                mcSplitJunction->disable();
                mcSplitJunctionAndReconnect->disable();
            }
        }
        return ret;
    }
}


double
GNEJunction::getExaggeration(const GUIVisualizationSettings& s) const {
    return s.junctionSize.getExaggeration(s, this, 4);
}


Boundary
GNEJunction::getCenteringBoundary() const {
    return myJunctionBoundary;
}


void
GNEJunction::updateCenteringBoundary(const bool updateGrid) {
    // Remove object from grid
    if (updateGrid) {
        myNet->removeGLObjectFromGrid(this);
    }
    // calculate boundary using a radius bigger than geometry point
    myJunctionBoundary = Boundary(myNBNode->getPosition().x() - 1, myNBNode->getPosition().y() - 1,
                                  myNBNode->getPosition().x() + 1, myNBNode->getPosition().y() + 1);
    myJunctionBoundary.grow(10);
    // add shape
    if (myNBNode->getShape().size() > 0) {
        myJunctionBoundary.add(myNBNode->getShape().getBoxBoundary());
        myJunctionBoundary.grow(5);
    }
    // add boundaries of all connections, walking areas and crossings
    for (const auto& edge : myGNEIncomingEdges) {
        for (const auto& connection : edge->getGNEConnections()) {
            const auto boundary = connection->getCenteringBoundary();
            if (boundary.isInitialised()) {
                myJunctionBoundary.add(boundary);
            }
        }
    }
    for (const auto& crossing : myGNECrossings) {
        const auto boundary = crossing->getCenteringBoundary();
        if (boundary.isInitialised()) {
            myJunctionBoundary.add(boundary);
        }
    }
    for (const auto& walkingArea : myGNEWalkingAreas) {
        const auto boundary = walkingArea->getCenteringBoundary();
        if (boundary.isInitialised()) {
            myJunctionBoundary.add(boundary);
        }
    }

    // add object into grid
    if (updateGrid) {
        // if junction has at least one edge, then don't add in grid (because uses the edge's grid)
        if (myGNEIncomingEdges.size() + myGNEOutgoingEdges.size() == 0) {
            myNet->addGLObjectIntoGrid(this);
        }
    }
    // trigger rebuilding tesselation
    myExaggeration = 2;
}


void
GNEJunction::drawGL(const GUIVisualizationSettings& s) const {
    // first check drawing toggle and boundary selection
    if ((*myDrawingToggle != myNet->getViewNet()->getDrawingToggle()) && checkDrawingBoundarySelection()) {
        // draw boundaries
        if (inGrid()) {
            GLHelper::drawBoundary(s, getCenteringBoundary());
        }
        // get junction exaggeration
        const double junctionExaggeration = getExaggeration(s);
        // only continue if exaggeration is greater than 0
        if (junctionExaggeration > 0) {
            // get detail level
            const auto d = s.getDetailLevel(junctionExaggeration);
            // get shape area
            const double junctionShapeArea = myNBNode->getShape().area();
            // check if draw junction as shape
            const bool drawBubble = drawAsBubble(s, junctionShapeArea);
            // draw geometry only if we'rent in drawForObjectUnderCursor mode
            if (!s.drawForViewObjectsHandler) {
                // push layer matrix
                GLHelper::pushMatrix();
                // translate to front
                myNet->getViewNet()->drawTranslateFrontAttributeCarrier(this, GLO_JUNCTION);
                // draw junction as shape
                drawJunctionAsShape(s, d, junctionExaggeration);
                // draw junction as bubble
                if (drawBubble) {
                    drawJunctionAsBubble(s, d, junctionExaggeration);
                }
                // draw TLS
                drawTLSIcon(s, d);
                // draw elevation
                drawElevation(s, d);
                // pop layer Matrix
                GLHelper::popMatrix();
                // draw lock icon
                GNEViewNetHelper::LockIcon::drawLockIcon(d, this, getType(), getPositionInView(), 1);
                // draw junction name
                drawJunctionName(s);
                // draw dotted contour depending if we're editing the custom shape
                const GNENetworkElement* editedNetworkElement = myNet->getViewNet()->getEditNetworkElementShapes().getEditedNetworkElement();
                if (editedNetworkElement && (editedNetworkElement == this)) {
                    // draw dotted contour geometry points
                    myNetworkElementContour.drawDottedContourGeometryPoints(s, d, this, myNBNode->getShape(), s.neteditSizeSettings.junctionGeometryPointRadius,
                            junctionExaggeration, s.dottedContourSettings.segmentWidthSmall);
                } else {
                    // draw dotted contour for shape
                    if (junctionShapeArea >= 4) {
                        myNetworkElementContour.drawDottedContours(s, d, this, s.dottedContourSettings.segmentWidth, true);
                    }
                    // draw dotted contour for bubble
                    if (drawBubble) {
                        myCircleContour.drawDottedContours(s, d, this, s.dottedContourSettings.segmentWidth, true);
                    }
                }
            }
            // calculate junction contour (always before children)
            calculateJunctioncontour(s, d, junctionExaggeration, drawBubble);
            // draw Junction childs
            drawJunctionChildren(s, d);
        }
        // update drawing toggle
        *myDrawingToggle = myNet->getViewNet()->getDrawingToggle();
    }
}


void
GNEJunction::deleteGLObject() {
    // Check if edge can be deleted
    if (GNEDeleteFrame::SubordinatedElements(this).checkElements(myNet->getViewNet()->getViewParent()->getDeleteFrame()->getProtectElements())) {
        myNet->deleteJunction(this, myNet->getViewNet()->getUndoList());
    }
}


void
GNEJunction::updateGLObject() {
    updateGeometry();
}


NBNode*
GNEJunction::getNBNode() const {
    return myNBNode;
}


std::vector<GNEJunction*>
GNEJunction::getJunctionNeighbours() const {
    // use set to avoid duplicates junctions
    std::set<GNEJunction*> junctions;
    for (const auto& incomingEdge : myGNEIncomingEdges) {
        junctions.insert(incomingEdge->getFromJunction());
    }
    for (const auto& outgoingEdge : myGNEOutgoingEdges) {
        junctions.insert(outgoingEdge->getToJunction());
    }
    return std::vector<GNEJunction*>(junctions.begin(), junctions.end());
}


void
GNEJunction::addIncomingGNEEdge(GNEEdge* edge) {
    // Check if incoming edge was already inserted
    std::vector<GNEEdge*>::iterator i = std::find(myGNEIncomingEdges.begin(), myGNEIncomingEdges.end(), edge);
    if (i != myGNEIncomingEdges.end()) {
        throw InvalidArgument("Incoming " + toString(SUMO_TAG_EDGE) + " with ID '" + edge->getID() + "' was already inserted into " + getTagStr() + " with ID " + getID() + "'");
    } else {
        // Add edge into containers
        myGNEIncomingEdges.push_back(edge);
    }
}



void
GNEJunction::addOutgoingGNEEdge(GNEEdge* edge) {
    // Check if outgoing edge was already inserted
    std::vector<GNEEdge*>::iterator i = std::find(myGNEOutgoingEdges.begin(), myGNEOutgoingEdges.end(), edge);
    if (i != myGNEOutgoingEdges.end()) {
        throw InvalidArgument("Outgoing " + toString(SUMO_TAG_EDGE) + " with ID '" + edge->getID() + "' was already inserted into " + getTagStr() + " with ID " + getID() + "'");
    } else {
        // Add edge into containers
        myGNEOutgoingEdges.push_back(edge);
    }
}


void
GNEJunction::removeIncomingGNEEdge(GNEEdge* edge) {
    // Check if incoming edge was already inserted
    std::vector<GNEEdge*>::iterator i = std::find(myGNEIncomingEdges.begin(), myGNEIncomingEdges.end(), edge);
    if (i == myGNEIncomingEdges.end()) {
        throw InvalidArgument("Incoming " + toString(SUMO_TAG_EDGE) + " with ID '" + edge->getID() + "' doesn't found into " + getTagStr() + " with ID " + getID() + "'");
    } else {
        // remove edge from containers
        myGNEIncomingEdges.erase(i);
    }
}


void
GNEJunction::removeOutgoingGNEEdge(GNEEdge* edge) {
    // Check if outgoing edge was already inserted
    std::vector<GNEEdge*>::iterator i = std::find(myGNEOutgoingEdges.begin(), myGNEOutgoingEdges.end(), edge);
    if (i == myGNEOutgoingEdges.end()) {
        throw InvalidArgument("Outgoing " + toString(SUMO_TAG_EDGE) + " with ID '" + edge->getID() + "' doesn't found into " + getTagStr() + " with ID " + getID() + "'");
    } else {
        // remove edge from containers
        myGNEOutgoingEdges.erase(i);
    }
}


const std::vector<GNEEdge*>&
GNEJunction::getGNEIncomingEdges() const {
    return myGNEIncomingEdges;
}


const std::vector<GNEEdge*>&
GNEJunction::getGNEOutgoingEdges() const {
    return myGNEOutgoingEdges;
}


const std::vector<GNECrossing*>&
GNEJunction::getGNECrossings() const {
    return myGNECrossings;
}


const std::vector<GNEWalkingArea*>&
GNEJunction::getGNEWalkingAreas() const {
    return myGNEWalkingAreas;
}


std::vector<GNEConnection*>
GNEJunction::getGNEConnections() const {
    std::vector<GNEConnection*> connections;
    for (const auto& incomingEdge : myGNEIncomingEdges) {
        for (const auto& connection : incomingEdge->getGNEConnections()) {
            connections.push_back(connection);
        }
    }
    return connections;
}


void
GNEJunction::markAsCreateEdgeSource() {
    myAmCreateEdgeSource = true;
}


void
GNEJunction::unMarkAsCreateEdgeSource() {
    myAmCreateEdgeSource = false;
}


void
GNEJunction::selectTLS(bool selected) {
    myAmTLSSelected = selected;
}


void
GNEJunction::invalidateShape() {
    if (!myNBNode->hasCustomShape()) {
        if (myNBNode->myPoly.size() > 0) {
            // write GL Debug
            WRITE_GLDEBUG("<-- Invalidating shape of junction '" + getID() + "' -->");
            // clear poly
            myNBNode->myPoly.clear();
            // update centering boundary
            updateCenteringBoundary(true);
        }
        myNet->requireRecompute();
    }
}


void
GNEJunction::setLogicValid(bool valid, GNEUndoList* undoList, const std::string& status) {
    myHasValidLogic = valid;
    if (!valid) {
        assert(undoList != 0);
        assert(undoList->hasCommandGroup());
        NBTurningDirectionsComputer::computeTurnDirectionsForNode(myNBNode, false);
        EdgeVector incoming = myNBNode->getIncomingEdges();
        for (EdgeVector::iterator it = incoming.begin(); it != incoming.end(); it++) {
            GNEEdge* srcEdge = myNet->getAttributeCarriers()->retrieveEdge((*it)->getID());
            removeConnectionsFrom(srcEdge, undoList, false); // false, because the whole tls will be invalidated at the end
            GNEChange_Attribute::changeAttribute(srcEdge, GNE_ATTR_MODIFICATION_STATUS, status, undoList, true);
        }
        GNEChange_Attribute::changeAttribute(this, GNE_ATTR_MODIFICATION_STATUS, status, undoList, true);
        invalidateTLS(undoList);
    } else {
        // logic valed, then rebuild GNECrossings to adapt it to the new logic
        // (but don't rebuild the crossings in NBNode because they are already finished)
        rebuildGNECrossings(false);
    }
}


void
GNEJunction::removeConnectionsFrom(GNEEdge* edge, GNEUndoList* undoList, bool updateTLS, int lane) {
    NBEdge* srcNBE = edge->getNBEdge();
    NBEdge* turnEdge = srcNBE->getTurnDestination();
    // Make a copy of connections
    std::vector<NBEdge::Connection> connections = srcNBE->getConnections();
    // delete in reverse so that undoing will add connections in the original order
    for (std::vector<NBEdge::Connection>::reverse_iterator con_it = connections.rbegin(); con_it != connections.rend(); con_it++) {
        if (lane >= 0 && (*con_it).fromLane != lane) {
            continue;
        }
        bool hasTurn = con_it->toEdge == turnEdge;
        undoList->add(new GNEChange_Connection(edge, *con_it, false, false), true);
        // needs to come after GNEChange_Connection
        // XXX bug: this code path will not be used on a redo!
        if (hasTurn) {
            myNet->addExplicitTurnaround(srcNBE->getID());
        }
    }
    if (updateTLS) {
        std::vector<NBConnection> removeConnections;
        for (NBEdge::Connection con : connections) {
            removeConnections.push_back(NBConnection(srcNBE, con.fromLane, con.toEdge, con.toLane));
        }
        removeTLSConnections(removeConnections, undoList);
    }
}


void
GNEJunction::removeConnectionsTo(GNEEdge* edge, GNEUndoList* undoList, bool updateTLS, int lane) {
    NBEdge* destNBE = edge->getNBEdge();
    std::vector<NBConnection> removeConnections;
    for (NBEdge* srcNBE : myNBNode->getIncomingEdges()) {
        GNEEdge* srcEdge = myNet->getAttributeCarriers()->retrieveEdge(srcNBE->getID());
        std::vector<NBEdge::Connection> connections = srcNBE->getConnections();
        for (std::vector<NBEdge::Connection>::reverse_iterator con_it = connections.rbegin(); con_it != connections.rend(); con_it++) {
            if ((*con_it).toEdge == destNBE) {
                if (lane >= 0 && (*con_it).toLane != lane) {
                    continue;
                }
                bool hasTurn = srcNBE->getTurnDestination() == destNBE;
                undoList->add(new GNEChange_Connection(srcEdge, *con_it, false, false), true);
                // needs to come after GNEChange_Connection
                // XXX bug: this code path will not be used on a redo!
                if (hasTurn) {
                    myNet->addExplicitTurnaround(srcNBE->getID());
                }
                removeConnections.push_back(NBConnection(srcNBE, (*con_it).fromLane, destNBE, (*con_it).toLane));
            }
        }
    }
    if (updateTLS) {
        removeTLSConnections(removeConnections, undoList);
    }
}


void
GNEJunction::removeTLSConnections(std::vector<NBConnection>& connections, GNEUndoList* undoList) {
    if (connections.size() > 0) {
        const std::set<NBTrafficLightDefinition*> coypOfTls = myNBNode->getControllingTLS(); // make a copy!
        for (const auto& TLS : coypOfTls) {
            NBLoadedSUMOTLDef* tlDef = dynamic_cast<NBLoadedSUMOTLDef*>(TLS);
            // guessed TLS (NBOwnTLDef) do not need to be updated
            if (tlDef != nullptr) {
                std::string newID = tlDef->getID();
                // create replacement before deleting the original because deletion will mess up saving original nodes
                NBLoadedSUMOTLDef* replacementDef = new NBLoadedSUMOTLDef(*tlDef, *tlDef->getLogic());
                for (NBConnection& con : connections) {
                    replacementDef->removeConnection(con);
                }
                undoList->add(new GNEChange_TLS(this, tlDef, false), true);
                undoList->add(new GNEChange_TLS(this, replacementDef, true, false, newID), true);
                // the removed traffic light may have controlled more than one junction. These too have become invalid now
                const std::vector<NBNode*> copyOfNodes = tlDef->getNodes(); // make a copy!
                for (const auto& node : copyOfNodes) {
                    GNEJunction* sharing = myNet->getAttributeCarriers()->retrieveJunction(node->getID());
                    undoList->add(new GNEChange_TLS(sharing, tlDef, false), true);
                    undoList->add(new GNEChange_TLS(sharing, replacementDef, true, false, newID), true);
                }
            }
        }
    }
}


void
GNEJunction::replaceIncomingConnections(GNEEdge* which, GNEEdge* by, GNEUndoList* undoList) {
    // remap connections of the edge
    assert(which->getLanes().size() == by->getLanes().size());
    std::vector<NBEdge::Connection> connections = which->getNBEdge()->getConnections();
    for (NBEdge::Connection& c : connections) {
        undoList->add(new GNEChange_Connection(which, c, false, false), true);
        undoList->add(new GNEChange_Connection(by, c, false, true), true);
    }
    // also remap tls connections
    const std::set<NBTrafficLightDefinition*> coypOfTls = myNBNode->getControllingTLS(); // make a copy!
    for (const auto& TLS : coypOfTls) {
        NBLoadedSUMOTLDef* tlDef = dynamic_cast<NBLoadedSUMOTLDef*>(TLS);
        // guessed TLS (NBOwnTLDef) do not need to be updated
        if (tlDef != nullptr) {
            std::string newID = tlDef->getID();
            // create replacement before deleting the original because deletion will mess up saving original nodes
            NBLoadedSUMOTLDef* replacementDef = new NBLoadedSUMOTLDef(*tlDef, *tlDef->getLogic());
            for (int i = 0; i < (int)which->getLanes().size(); ++i) {
                replacementDef->replaceRemoved(which->getNBEdge(), i, by->getNBEdge(), i, true);
            }
            undoList->add(new GNEChange_TLS(this, tlDef, false), true);
            undoList->add(new GNEChange_TLS(this, replacementDef, true, false, newID), true);
            // the removed traffic light may have controlled more than one junction. These too have become invalid now
            const std::vector<NBNode*> copyOfNodes = tlDef->getNodes(); // make a copy!
            for (const auto& node : copyOfNodes) {
                GNEJunction* sharing = myNet->getAttributeCarriers()->retrieveJunction(node->getID());
                undoList->add(new GNEChange_TLS(sharing, tlDef, false), true);
                undoList->add(new GNEChange_TLS(sharing, replacementDef, true, false, newID), true);
            }
        }
    }
}


void
GNEJunction::markAsModified(GNEUndoList* undoList) {
    EdgeVector incoming = myNBNode->getIncomingEdges();
    for (EdgeVector::iterator it = incoming.begin(); it != incoming.end(); it++) {
        NBEdge* srcNBE = *it;
        GNEEdge* srcEdge = myNet->getAttributeCarriers()->retrieveEdge(srcNBE->getID());
        GNEChange_Attribute::changeAttribute(srcEdge, GNE_ATTR_MODIFICATION_STATUS, FEATURE_MODIFIED, undoList, true);
    }
}


void
GNEJunction::invalidateTLS(GNEUndoList* undoList, const NBConnection& deletedConnection, const NBConnection& addedConnection) {
    assert(undoList->hasCommandGroup());
    // NBLoadedSUMOTLDef becomes invalid, replace with NBOwnTLDef which will be dynamically recomputed
    const std::set<NBTrafficLightDefinition*> coypOfTls = myNBNode->getControllingTLS(); // make a copy!
    for (const auto& TLS : coypOfTls) {
        NBLoadedSUMOTLDef* tlDef = dynamic_cast<NBLoadedSUMOTLDef*>(TLS);
        if (tlDef != nullptr) {
            // the removed traffic light may have controlled more than one junction. These too have become invalid now
            const std::vector<NBNode*> copyOfNodes = tlDef->getNodes(); // make a copy!
            if (myGNECrossings.size() == 0 && getNBNode()->getCrossings().size() != 0) {
                // crossings were not computed yet. We need them as netedit elements to manage tlIndex resetting
                myNet->getNetBuilder()->setHaveNetworkCrossings(true);
                rebuildGNECrossings();
                for (const auto& node : copyOfNodes) {
                    GNEJunction* sharing = myNet->getAttributeCarriers()->retrieveJunction(node->getID());
                    if (sharing != this) {
                        sharing->rebuildGNECrossings();
                    }
                }
            }
            NBTrafficLightDefinition* replacementDef = nullptr;
            std::string newID = tlDef->getID(); // + "_reguessed"; // changes due to reguessing will be visible in diff
            if (deletedConnection != NBConnection::InvalidConnection) {
                // create replacement before deleting the original because deletion will mess up saving original nodes
                NBLoadedSUMOTLDef* repl = new NBLoadedSUMOTLDef(*tlDef, *tlDef->getLogic());
                repl->removeConnection(deletedConnection);
                replacementDef = repl;
            } else if (addedConnection != NBConnection::InvalidConnection) {
                if (addedConnection.getTLIndex() == NBConnection::InvalidTlIndex) {
                    // custom tl indices of crossings might become invalid upon recomputation so we must save them
                    // however, they could remain valid so we register a change but keep them at their old value
                    for (const auto& crossing : myGNECrossings) {
                        const std::string oldValue = crossing->getAttribute(SUMO_ATTR_TLLINKINDEX);
                        GNEChange_Attribute::changeAttribute(crossing, SUMO_ATTR_TLLINKINDEX, toString(NBConnection::InvalidTlIndex), undoList, true);
                        GNEChange_Attribute::changeAttribute(crossing, SUMO_ATTR_TLLINKINDEX, oldValue, undoList, true);
                        const std::string oldValue2 = crossing->getAttribute(SUMO_ATTR_TLLINKINDEX2);
                        GNEChange_Attribute::changeAttribute(crossing, SUMO_ATTR_TLLINKINDEX2, toString(NBConnection::InvalidTlIndex), undoList, true);
                        GNEChange_Attribute::changeAttribute(crossing, SUMO_ATTR_TLLINKINDEX2, oldValue2, undoList, true);
                    }
                }
                NBLoadedSUMOTLDef* repl = new NBLoadedSUMOTLDef(*tlDef, *tlDef->getLogic());
                repl->addConnection(addedConnection.getFrom(), addedConnection.getTo(),
                                    addedConnection.getFromLane(), addedConnection.getToLane(), addedConnection.getTLIndex(), addedConnection.getTLIndex2());
                replacementDef = repl;
            } else {
                // recompute crossing indices along with everything else
                for (const auto& crossing : myGNECrossings) {
                    GNEChange_Attribute::changeAttribute(crossing, SUMO_ATTR_TLLINKINDEX, toString(NBConnection::InvalidTlIndex), undoList, true);
                    GNEChange_Attribute::changeAttribute(crossing, SUMO_ATTR_TLLINKINDEX2, toString(NBConnection::InvalidTlIndex), undoList, true);
                }
                replacementDef = new NBOwnTLDef(newID, tlDef->getOffset(), tlDef->getType());
                replacementDef->setProgramID(tlDef->getProgramID());
            }
            undoList->add(new GNEChange_TLS(this, tlDef, false), true);
            undoList->add(new GNEChange_TLS(this, replacementDef, true, false, newID), true);
            // reset nodes of joint tls
            for (const auto& node : copyOfNodes) {
                GNEJunction* sharing = myNet->getAttributeCarriers()->retrieveJunction(node->getID());
                if (sharing != this) {
                    if (deletedConnection == NBConnection::InvalidConnection && addedConnection == NBConnection::InvalidConnection) {
                        // recompute crossing indices for shared
                        // (they won't do this on subsequent call to invalidateTLS if they received an NBOwnTLDef)
                        for (const auto& crossing : sharing->getGNECrossings()) {
                            GNEChange_Attribute::changeAttribute(crossing, SUMO_ATTR_TLLINKINDEX, toString(NBConnection::InvalidTlIndex), undoList, true);
                            GNEChange_Attribute::changeAttribute(crossing, SUMO_ATTR_TLLINKINDEX2, toString(NBConnection::InvalidTlIndex), undoList, true);
                        }
                    }
                    undoList->add(new GNEChange_TLS(sharing, tlDef, false), true);
                    undoList->add(new GNEChange_TLS(sharing, replacementDef, true, false, newID), true);
                }
            }
        }
    }
}

void
GNEJunction::removeEdgeFromCrossings(GNEEdge* edge, GNEUndoList* undoList) {
    // obtain a copy of GNECrossing of junctions
    const auto copyOfGNECrossings = myGNECrossings;
    // iterate over copy of GNECrossings
    for (const auto& crossing : copyOfGNECrossings) {
        // obtain the set of edges vinculated with the crossing (due it works as ID)
        EdgeSet edgeSet(crossing->getCrossingEdges().begin(), crossing->getCrossingEdges().end());
        // If this edge is part of the set of edges of crossing
        if (edgeSet.count(edge->getNBEdge()) == 1) {
            // delete crossing if this is their last edge
            if ((crossing->getCrossingEdges().size() == 1) && (crossing->getCrossingEdges().front() == edge->getNBEdge())) {
                myNet->deleteCrossing(crossing, undoList);
            } else {
                // remove this edge of the edge's attribute of crossing (note: This can invalidate the crossing)
                std::vector<std::string> edges = GNEAttributeCarrier::parse<std::vector<std::string>>(crossing->getAttribute(SUMO_ATTR_EDGES));
                edges.erase(std::find(edges.begin(), edges.end(), edge->getID()));
                crossing->setAttribute(SUMO_ATTR_EDGES, joinToString(edges, " "), undoList);
            }
        }
    }
}


bool
GNEJunction::isLogicValid() {
    return myHasValidLogic;
}


GNECrossing*
GNEJunction::retrieveGNECrossing(NBNode::Crossing* NBNodeCrossing, bool createIfNoExist) {
    // iterate over all crossing
    for (const auto& crossing : myGNECrossings) {
        // if found, return it
        if (crossing->getCrossingEdges() == NBNodeCrossing->edges) {
            return crossing;
        }
    }
    if (createIfNoExist) {
        // create new GNECrossing
        GNECrossing* createdGNECrossing = new GNECrossing(this, NBNodeCrossing->edges);
        // show extra information for tests
        WRITE_DEBUG("Created " + createdGNECrossing->getTagStr() + " '" + createdGNECrossing->getID() + "' in retrieveGNECrossing()");
        // update geometry after creating
        createdGNECrossing->updateGeometry();
        // add it in Network
        myNet->addGLObjectIntoGrid(createdGNECrossing);
        // add it in attributeCarriers
        myNet->getAttributeCarriers()->insertCrossing(createdGNECrossing);
        return createdGNECrossing;
    } else {
        return nullptr;
    }
}


GNEWalkingArea*
GNEJunction::retrieveGNEWalkingArea(const std::string& NBNodeWalkingAreaID, bool createIfNoExist) {
    // iterate over all walkingArea
    for (const auto& walkingArea : myGNEWalkingAreas) {
        // if found, return it
        if (walkingArea->getID() == NBNodeWalkingAreaID) {
            return walkingArea;
        }
    }
    if (createIfNoExist) {
        // create new GNEWalkingArea
        GNEWalkingArea* createdGNEWalkingArea = new GNEWalkingArea(this, NBNodeWalkingAreaID);
        // show extra information for tests
        WRITE_DEBUG("Created " + createdGNEWalkingArea->getTagStr() + " '" + createdGNEWalkingArea->getID() + "' in retrieveGNEWalkingArea()");
        // update geometry after creating
        createdGNEWalkingArea->updateGeometry();
        // add it in Network
        myNet->addGLObjectIntoGrid(createdGNEWalkingArea);
        // add it in attributeCarriers
        myNet->getAttributeCarriers()->insertWalkingArea(createdGNEWalkingArea);
        return createdGNEWalkingArea;
    } else {
        return nullptr;
    }
}


void
GNEJunction::markConnectionsDeprecated(bool includingNeighbours) {
    // only it's needed to mark the connections of incoming edges
    for (const auto& i : myGNEIncomingEdges) {
        for (const auto& j : i->getGNEConnections()) {
            j->markConnectionGeometryDeprecated();
        }
        if (includingNeighbours) {
            i->getFromJunction()->markConnectionsDeprecated(false);
        }
    }
}


void
GNEJunction::setJunctionType(const std::string& value, GNEUndoList* undoList) {
    undoList->begin(this, "change " + getTagStr() + " type");
    if (NBNode::isTrafficLight(SUMOXMLDefinitions::NodeTypes.get(value))) {
        if (getNBNode()->isTLControlled() &&
                // if switching changing from or to traffic_light_right_on_red we need to remove the old plan
                (getNBNode()->getType() == SumoXMLNodeType::TRAFFIC_LIGHT_RIGHT_ON_RED
                 || SUMOXMLDefinitions::NodeTypes.get(value) == SumoXMLNodeType::TRAFFIC_LIGHT_RIGHT_ON_RED)
           ) {
            // make a copy because we will modify the original
            const std::set<NBTrafficLightDefinition*> copyOfTls = myNBNode->getControllingTLS();
            for (const auto& TLS : copyOfTls) {
                undoList->add(new GNEChange_TLS(this, TLS, false), true);
            }
        }
        if (!getNBNode()->isTLControlled()) {
            // create new traffic light
            undoList->add(new GNEChange_TLS(this, nullptr, true), true);
        }
    } else if (getNBNode()->isTLControlled()) {
        // delete old traffic light
        // make a copy because we will modify the original
        const std::set<NBTrafficLightDefinition*> copyOfTls = myNBNode->getControllingTLS();
        for (const auto& TLS : copyOfTls) {
            undoList->add(new GNEChange_TLS(this, TLS, false, false), true);
            const std::vector<NBNode*> copyOfNodes = TLS->getNodes(); // make a copy!
            for (const auto& node : copyOfNodes) {
                GNEJunction* sharing = myNet->getAttributeCarriers()->retrieveJunction(node->getID());
                sharing->invalidateTLS(undoList);
            }
        }
    }
    // must be the final step, otherwise we do not know which traffic lights to remove via GNEChange_TLS
    GNEChange_Attribute::changeAttribute(this, SUMO_ATTR_TYPE, value, undoList, true);
    for (const auto& crossing : myGNECrossings) {
        GNEChange_Attribute::changeAttribute(crossing, SUMO_ATTR_TLLINKINDEX, "-1", undoList, true);
        GNEChange_Attribute::changeAttribute(crossing, SUMO_ATTR_TLLINKINDEX2, "-1", undoList, true);
    }
    undoList->end();
}


void
GNEJunction::clearWalkingAreas() {
    // delete non retrieved GNEWalkingAreas (we don't need to extract if from Tree two times)
    for (const auto& walkingArea : myGNEWalkingAreas) {
        walkingArea->decRef();
        // check if walkingArea is selected
        if (walkingArea->isAttributeCarrierSelected()) {
            walkingArea->unselectAttributeCarrier();
        }
        // remove it from inspected ACS
        myNet->getViewNet()->removeFromAttributeCarrierInspected(walkingArea);
        // remove it from net
        myNet->removeGLObjectFromGrid(walkingArea);
        // remove it from attributeCarriers
        myNet->getAttributeCarriers()->deleteWalkingArea(walkingArea);
        if (walkingArea->unreferenced()) {
            // show extra information for tests
            WRITE_DEBUG("Deleting unreferenced " + walkingArea->getTagStr() + " in rebuildGNEWalkingAreas()");
            delete walkingArea;
        }
    }
    myGNEWalkingAreas.clear();
}


void
GNEJunction::rebuildGNEWalkingAreas() {
    // first clear GNEWalkingAreas
    clearWalkingAreas();
    // iterate over NBNode::WalkingAreas of GNEJunction
    for (const auto& walkingArea : myNBNode->getWalkingAreas()) {
        // retrieve existent GNEWalkingArea, or create it
        GNEWalkingArea* retrievedGNEWalkingArea = retrieveGNEWalkingArea(walkingArea.id, true);
        // include reference to created GNEWalkingArea
        retrievedGNEWalkingArea->incRef();
        // update geometry of retrieved walkingArea
        retrievedGNEWalkingArea->updateGeometry();
        // update boundary
        retrievedGNEWalkingArea->updateCenteringBoundary(false);
        // add in walkingAreas
        myGNEWalkingAreas.push_back(retrievedGNEWalkingArea);
    }
}



void
GNEJunction::addInternalLane(const GNEInternalLane* internalLane) {
    if (std::find(myInternalLanes.begin(), myInternalLanes.end(), internalLane) != myInternalLanes.end()) {
        throw ProcessError(internalLane->getTagStr() + " with ID='" + internalLane->getID() + "' already exist");
    } else {
        myInternalLanes.push_back(internalLane);
    }
}


void
GNEJunction::removeInternalLane(const GNEInternalLane* internalLane) {
    const auto finder = std::find(myInternalLanes.begin(), myInternalLanes.end(), internalLane);
    if (finder == myInternalLanes.end()) {
        throw ProcessError(internalLane->getTagStr() + " with ID='" + internalLane->getID() + "' wasn't previously inserted");
    } else {
        myInternalLanes.erase(finder);
    }
}


std::string
GNEJunction::getAttribute(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_ID:
            return getMicrosimID();
        case SUMO_ATTR_POSITION:
            return toString(myNBNode->getPosition());
        case SUMO_ATTR_TYPE:
            return toString(myNBNode->getType());
        case GNE_ATTR_MODIFICATION_STATUS:
            return myLogicStatus;
        case SUMO_ATTR_SHAPE:
            return toString(myNBNode->getShape());
        case SUMO_ATTR_RADIUS:
            if (myNBNode->getRadius() < 0) {
                return "default";
            } else {
                return toString(myNBNode->getRadius());
            }
        case SUMO_ATTR_TLTYPE:
            if (isAttributeEnabled(SUMO_ATTR_TLTYPE)) {
                // @todo this causes problems if the node were to have multiple programs of different type (plausible)
                return toString((*myNBNode->getControllingTLS().begin())->getType());
            } else {
                return "No TLS";
            }
        case SUMO_ATTR_TLLAYOUT:
            if (isAttributeEnabled(SUMO_ATTR_TLLAYOUT)) {
                return toString((*myNBNode->getControllingTLS().begin())->getLayout());
            } else {
                return "No TLS";
            }
        case SUMO_ATTR_TLID:
            if (isAttributeEnabled(SUMO_ATTR_TLID)) {
                return toString((*myNBNode->getControllingTLS().begin())->getID());
            } else {
                return "No TLS";
            }
        case GNE_ATTR_IS_ROUNDABOUT:
            return myNBNode->isRoundabout() ? True : False;
        case SUMO_ATTR_KEEP_CLEAR:
            // keep clear is only used as a convenience feature in plain xml
            // input. When saving to .net.xml the status is saved only for the connections
            // to show the correct state we must check all connections
            for (const auto& i : myGNEIncomingEdges) {
                for (const auto& j : i->getGNEConnections()) {
                    if (j->getNBEdgeConnection().keepClear) {
                        return True;
                    }
                }
            }
            return False;
        case SUMO_ATTR_RIGHT_OF_WAY:
            return SUMOXMLDefinitions::RightOfWayValues.getString(myNBNode->getRightOfWay());
        case SUMO_ATTR_FRINGE:
            return SUMOXMLDefinitions::FringeTypeValues.getString(myNBNode->getFringeType());
        case SUMO_ATTR_NAME:
            return myNBNode->getName();
        case GNE_ATTR_SELECTED:
            return toString(isAttributeCarrierSelected());
        case GNE_ATTR_PARAMETERS:
            return myNBNode->getParametersStr();
        default:
            throw InvalidArgument(getTagStr() + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


PositionVector
GNEJunction::getAttributePositionVector(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_SHAPE:
            return myNBNode->getShape();
        default:
            throw InvalidArgument(getTagStr() + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNEJunction::setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) {
    if (value == getAttribute(key)) {
        return; //avoid needless changes, later logic relies on the fact that attributes have changed
    }
    switch (key) {
        case SUMO_ATTR_ID:
        case GNE_ATTR_MODIFICATION_STATUS:
        case SUMO_ATTR_SHAPE:
        case SUMO_ATTR_RADIUS:
        case SUMO_ATTR_RIGHT_OF_WAY:
        case SUMO_ATTR_FRINGE:
        case SUMO_ATTR_NAME:
        case GNE_ATTR_SELECTED:
        case GNE_ATTR_PARAMETERS:
            GNEChange_Attribute::changeAttribute(this, key, value, undoList, true);
            break;
        case SUMO_ATTR_POSITION: {
            // change Keep Clear attribute in all connections
            undoList->begin(this, TL("change junction position"));
            // obtain NBNode position
            const Position orig = myNBNode->getPosition();
            // change junction position
            GNEChange_Attribute::changeAttribute(this, key, value, undoList, true);
            // calculate delta using new position
            const Position delta = myNBNode->getPosition() - orig;
            // set new position of adjacent edges
            for (const auto& edge : myGNEIncomingEdges) {
                const Position newEnd = edge->getNBEdge()->getGeometry().back() + delta;
                GNEChange_Attribute::changeAttribute(edge, GNE_ATTR_SHAPE_END, toString(newEnd), undoList, true);
            }
            for (const auto& edge : myGNEOutgoingEdges) {
                const Position newStart = edge->getNBEdge()->getGeometry().front() + delta;
                GNEChange_Attribute::changeAttribute(edge, GNE_ATTR_SHAPE_START, toString(newStart), undoList, true);
            }
            undoList->end();
            break;
        }
        case SUMO_ATTR_KEEP_CLEAR:
            // change Keep Clear attribute in all connections
            undoList->begin(this, TL("change keepClear for whole junction"));
            for (const auto& incomingEdge : myGNEIncomingEdges) {
                for (const auto& junction : incomingEdge->getGNEConnections()) {
                    GNEChange_Attribute::changeAttribute(junction, key, value, undoList, true);
                }
            }
            undoList->end();
            break;
        case SUMO_ATTR_TYPE: {
            // set junction type
            setJunctionType(value, undoList);
            break;
        }
        case SUMO_ATTR_TLTYPE: {
            undoList->begin(this, "change " + getTagStr() + " tl-type");
            // make a copy because we will modify the original
            const std::set<NBTrafficLightDefinition*> copyOfTls = myNBNode->getControllingTLS();
            for (const auto& TLS : copyOfTls) {
                NBLoadedSUMOTLDef* oldLoaded = dynamic_cast<NBLoadedSUMOTLDef*>(TLS);
                if (oldLoaded != nullptr) {
                    NBTrafficLightDefinition* newDef = nullptr;
                    if (value == toString(TrafficLightType::NEMA) || oldLoaded->getType() == TrafficLightType::NEMA) {
                        // rebuild the program because the old and new ones are incompatible
                        newDef = new NBOwnTLDef(oldLoaded->getID(), oldLoaded->getOffset(), TrafficLightType::NEMA);
                        newDef->setProgramID(oldLoaded->getProgramID());
                    } else {
                        NBLoadedSUMOTLDef* newLDef = new NBLoadedSUMOTLDef(*oldLoaded, *oldLoaded->getLogic());
                        newLDef->guessMinMaxDuration(); // minDur and maxDur are never written for a static tls
                        newDef = newLDef;
                    }
                    std::vector<NBNode*> nodes = TLS->getNodes();
                    for (const auto& node : nodes) {
                        GNEJunction* junction = myNet->getAttributeCarriers()->retrieveJunction(node->getID());
                        undoList->add(new GNEChange_TLS(junction, TLS, false), true);
                        undoList->add(new GNEChange_TLS(junction, newDef, true), true);
                    }
                }
            }
            GNEChange_Attribute::changeAttribute(this, key, value, undoList, true);
            undoList->end();
            break;
        }
        case SUMO_ATTR_TLLAYOUT: {
            undoList->begin(this, "change " + getTagStr() + " tlLayout");
            const std::set<NBTrafficLightDefinition*> copyOfTls = myNBNode->getControllingTLS();
            for (const auto& oldTLS : copyOfTls) {
                std::vector<NBNode*> copyOfNodes = oldTLS->getNodes();
                NBOwnTLDef* newTLS = new NBOwnTLDef(oldTLS->getID(), oldTLS->getOffset(), oldTLS->getType());
                newTLS->setLayout(SUMOXMLDefinitions::TrafficLightLayouts.get(value));
                newTLS->setProgramID(oldTLS->getProgramID());
                for (const auto& node : copyOfNodes) {
                    GNEJunction* oldJunction = myNet->getAttributeCarriers()->retrieveJunction(node->getID());
                    undoList->add(new GNEChange_TLS(oldJunction, oldTLS, false), true);
                }
                for (const auto& node : copyOfNodes) {
                    GNEJunction* oldJunction = myNet->getAttributeCarriers()->retrieveJunction(node->getID());
                    undoList->add(new GNEChange_TLS(oldJunction, newTLS, true), true);
                }
            }
            undoList->end();
            break;
        }
        case SUMO_ATTR_TLID: {
            undoList->begin(this, "change " + toString(SUMO_TAG_TRAFFIC_LIGHT) + " id");
            const std::set<NBTrafficLightDefinition*> copyOfTls = myNBNode->getControllingTLS();
            assert(copyOfTls.size() > 0);
            NBTrafficLightDefinition* currentTLS = *copyOfTls.begin();
            NBTrafficLightDefinition* currentTLSCopy = nullptr;
            const bool currentIsSingle = currentTLS->getNodes().size() == 1;
            const bool currentIsLoaded = dynamic_cast<NBLoadedSUMOTLDef*>(currentTLS) != nullptr;
            if (currentIsLoaded) {
                currentTLSCopy = new NBLoadedSUMOTLDef(*currentTLS,
                                                       *dynamic_cast<NBLoadedSUMOTLDef*>(currentTLS)->getLogic());
            }
            // remove from previous tls
            for (const auto& TLS : copyOfTls) {
                undoList->add(new GNEChange_TLS(this, TLS, false), true);
            }
            NBTrafficLightLogicCont& tlCont = myNet->getTLLogicCont();
            // programs to which the current node shall be added
            const std::map<std::string, NBTrafficLightDefinition*> programs = tlCont.getPrograms(value);
            if (programs.size() > 0) {
                for (const auto& TLSProgram : programs) {
                    NBTrafficLightDefinition* oldTLS = TLSProgram.second;
                    if (dynamic_cast<NBOwnTLDef*>(oldTLS) != nullptr) {
                        undoList->add(new GNEChange_TLS(this, oldTLS, true), true);
                    } else {
                        // delete and re-create the definition because the loaded phases are now invalid
                        if (dynamic_cast<NBLoadedSUMOTLDef*>(oldTLS) != nullptr &&
                                dynamic_cast<NBLoadedSUMOTLDef*>(oldTLS)->usingSignalGroups()) {
                            // keep the old program and add all-red state for the added links
                            NBLoadedSUMOTLDef* newTLSJoined = new NBLoadedSUMOTLDef(*oldTLS, *dynamic_cast<NBLoadedSUMOTLDef*>(oldTLS)->getLogic());
                            newTLSJoined->joinLogic(currentTLSCopy);
                            undoList->add(new GNEChange_TLS(this, newTLSJoined, true, true), true);
                        } else {
                            undoList->add(new GNEChange_TLS(this, nullptr, true, false, value), true);
                        }
                        NBTrafficLightDefinition* newTLS = *myNBNode->getControllingTLS().begin();
                        // switch from old to new definition
                        std::vector<NBNode*> copyOfNodes = oldTLS->getNodes();
                        for (const auto& node : copyOfNodes) {
                            GNEJunction* oldJunction = myNet->getAttributeCarriers()->retrieveJunction(node->getID());
                            undoList->add(new GNEChange_TLS(oldJunction, oldTLS, false), true);
                            undoList->add(new GNEChange_TLS(oldJunction, newTLS, true), true);
                        }
                    }
                }
            } else {
                if (currentIsSingle && currentIsLoaded) {
                    // rename the traffic light but keep everything else
                    NBTrafficLightLogic* renamedLogic = dynamic_cast<NBLoadedSUMOTLDef*>(currentTLSCopy)->getLogic();
                    renamedLogic->setID(value);
                    NBLoadedSUMOTLDef* renamedTLS = new NBLoadedSUMOTLDef(*currentTLSCopy, *renamedLogic);
                    renamedTLS->setID(value);
                    undoList->add(new GNEChange_TLS(this, renamedTLS, true, true), true);
                } else {
                    // create new traffic light
                    undoList->add(new GNEChange_TLS(this, nullptr, true, false, value), true);
                }
            }
            delete currentTLSCopy;
            undoList->end();
            break;
        }
        default:
            throw InvalidArgument(getTagStr() + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


bool
GNEJunction::isValid(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            return SUMOXMLDefinitions::isValidNetID(value) && (myNet->getAttributeCarriers()->retrieveJunction(value, false) == nullptr);
        case SUMO_ATTR_TYPE:
            return SUMOXMLDefinitions::NodeTypes.hasString(value);
        case SUMO_ATTR_POSITION:
            return canParse<Position>(value);
        case SUMO_ATTR_SHAPE:
            // empty shapes are allowed
            return canParse<PositionVector>(value);
        case SUMO_ATTR_RADIUS:
            if (value.empty() || (value == "default")) {
                return true;
            } else {
                return canParse<double>(value) && ((parse<double>(value) >= 0) || (parse<double>(value) == -1));
            }
        case SUMO_ATTR_TLTYPE:
            return myNBNode->isTLControlled() && SUMOXMLDefinitions::TrafficLightTypes.hasString(value);
        case SUMO_ATTR_TLLAYOUT:
            return myNBNode->isTLControlled() && SUMOXMLDefinitions::TrafficLightLayouts.hasString(value);
        case SUMO_ATTR_TLID:
            return myNBNode->isTLControlled() && (value != "");
        case SUMO_ATTR_KEEP_CLEAR:
            return canParse<bool>(value);
        case SUMO_ATTR_RIGHT_OF_WAY:
            return SUMOXMLDefinitions::RightOfWayValues.hasString(value);
        case SUMO_ATTR_FRINGE:
            return SUMOXMLDefinitions::FringeTypeValues.hasString(value);
        case SUMO_ATTR_NAME:
            return true;
        case GNE_ATTR_SELECTED:
            return canParse<bool>(value);
        case GNE_ATTR_PARAMETERS:
            return Parameterised::areParametersValid(value);
        default:
            throw InvalidArgument(getTagStr() + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


bool
GNEJunction::isAttributeEnabled(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_TLTYPE:
        case SUMO_ATTR_TLLAYOUT:
        case SUMO_ATTR_TLID:
            return myNBNode->isTLControlled();
        case SUMO_ATTR_KEEP_CLEAR: {
            // check if at least there is an incoming connection
            for (const auto& incomingEdge : myGNEIncomingEdges) {
                if (incomingEdge->getGNEConnections().size() > 0) {
                    return true;
                }
            }
            return false;
        }
        case GNE_ATTR_IS_ROUNDABOUT:
            return false;
        default:
            return true;
    }
}


bool
GNEJunction::isAttributeComputed(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_SHAPE:
            return !myNBNode->hasCustomShape();
        default:
            return false;
    }
}


const Parameterised::Map&
GNEJunction::getACParametersMap() const {
    return myNBNode->getParametersMap();
}


void
GNEJunction::setResponsible(bool newVal) {
    myAmResponsible = newVal;
}

// ===========================================================================
// private
// ===========================================================================

bool
GNEJunction::drawAsBubble(const GUIVisualizationSettings& s, const double junctionShapeArea) const {
    const auto& editModes = myNet->getViewNet()->getEditModes();
    // check conditions
    if (junctionShapeArea < 4) {
        // force draw if this junction is a candidate
        if (mySourceCandidate || myTargetCandidate || mySpecialCandidate ||
                myPossibleCandidate || myConflictedCandidate) {
            return true;
        }
        // force draw if we're in person/container plan mode
        if (editModes.isCurrentSupermodeDemand() &&
                ((editModes.demandEditMode == DemandEditMode::DEMAND_PERSON) ||
                 (editModes.demandEditMode == DemandEditMode::DEMAND_PERSONPLAN) ||
                 (editModes.demandEditMode == DemandEditMode::DEMAND_CONTAINER) ||
                 (editModes.demandEditMode == DemandEditMode::DEMAND_CONTAINERPLAN))) {
            return true;
        }
        // force draw if we're inspecting a vehicle that start or ends in a junction
        if (myNet->getViewNet()->getInspectedAttributeCarriers().size() == 1) {
            // get inspected element
            const auto inspectedAC = myNet->getViewNet()->getInspectedAttributeCarriers().front();
            // check if starts or ends in this junction
            if ((inspectedAC->hasAttribute(SUMO_ATTR_FROM_JUNCTION) && (inspectedAC->getAttribute(SUMO_ATTR_FROM_JUNCTION) == getID())) ||
                    (inspectedAC->hasAttribute(SUMO_ATTR_TO_JUNCTION) && (inspectedAC->getAttribute(SUMO_ATTR_TO_JUNCTION) == getID()))) {
                return true;
            }
        }
    }
    if (!s.drawJunctionShape) {
        // don't draw bubble if it was disabled in GUIVisualizationSettings
        return false;
    }
    if (myNet->getViewNet()->showJunctionAsBubbles()) {
        // force draw bubbles if we enabled option in checkbox of viewNet
        return true;
    }
    if (junctionShapeArea >= 4) {
        // don't draw if shape area is greater than 4
        return false;
    }
    if (!editModes.isCurrentSupermodeNetwork()) {
        // only draw bubbles in network mode
        return false;
    }
    return true;
}


void
GNEJunction::drawJunctionAsBubble(const GUIVisualizationSettings& s, const GUIVisualizationSettings::Detail d,
                                  const double exaggeration) const {
    // calculate bubble radius
    const double bubbleRadius = s.neteditSizeSettings.junctionBubbleRadius * exaggeration;
    // set bubble color
    const RGBColor bubbleColor = setColor(s, true);
    // push matrix
    GLHelper::pushMatrix();
    // set color
    GLHelper::setColor(bubbleColor);
    // move matrix junction center
    glTranslated(myNBNode->getPosition().x(), myNBNode->getPosition().y(), 1.5);
    // draw filled circle
    GLHelper::drawFilledCircleDetailled(d, bubbleRadius);
    // pop matrix
    GLHelper::popMatrix();
}


void
GNEJunction::drawJunctionAsShape(const GUIVisualizationSettings& s, const GUIVisualizationSettings::Detail d, const double exaggeration) const {
    // first check drawing conditions
    if (s.drawJunctionShape && (myNBNode->getShape().size() > 0)) {
        // set shape color
        const RGBColor junctionShapeColor = setColor(s, false);
        // set color
        GLHelper::setColor(junctionShapeColor);
        // adjust shape to exaggeration (check)
        if ((exaggeration > 1 || myExaggeration > 1) && exaggeration != myExaggeration) {
            myExaggeration = exaggeration;
            myTesselation.setShape(myNBNode->getShape());
            myTesselation.getShapeRef().closePolygon();
            myTesselation.getShapeRef().scaleRelative(exaggeration);
            myTesselation.myTesselation.clear();
        }
        // check if draw tesselation or or polygon
        if (d <= GUIVisualizationSettings::Detail::DrawPolygonTesselation) {
            // draw shape with high detail
            myTesselation.drawTesselation(myTesselation.getShape());
        } else {
            // draw shape
            GLHelper::drawFilledPoly(myNBNode->getShape(), true);
        }
        // draw shape points only in Network supermode
        if (myShapeEdited && myNet->getViewNet()->getEditModes().isCurrentSupermodeNetwork() &&
                s.drawMovingGeometryPoint(exaggeration, s.neteditSizeSettings.junctionGeometryPointRadius)) {
            // set color
            const RGBColor darkerColor = junctionShapeColor.changedBrightness(-32);
            // calculate geometry
            GUIGeometry junctionGeometry;
            // obtain junction Shape
            PositionVector junctionOpenShape = myNBNode->getShape();
            // adjust shape to exaggeration
            if (exaggeration > 1) {
                junctionOpenShape.scaleRelative(exaggeration);
            }
            // update geometry
            junctionGeometry.updateGeometry(junctionOpenShape);
            // set color
            GLHelper::setColor(darkerColor);
            // draw shape
            GUIGeometry::drawGeometry(d, junctionGeometry, s.neteditSizeSettings.junctionGeometryPointRadius * 0.5);
            // draw geometry points
            GUIGeometry::drawGeometryPoints(d, junctionOpenShape, darkerColor,
                                            s.neteditSizeSettings.junctionGeometryPointRadius, exaggeration,
                                            myNet->getViewNet()->getNetworkViewOptions().editingElevation());
        }
    }
}


void
GNEJunction::drawTLSIcon(const GUIVisualizationSettings& s, const GUIVisualizationSettings::Detail d) const {
    // draw TLS icon if isn't being drawn for selecting
    if ((d <= GUIVisualizationSettings::Detail::TLSIcon) && myNBNode->isTLControlled() &&
            (myNet->getViewNet()->getEditModes().networkEditMode == NetworkEditMode::NETWORK_TLS) &&
            !myAmTLSSelected && !s.drawForRectangleSelection) {
        GLHelper::pushMatrix();
        const Position pos = myNBNode->getPosition();
        glTranslated(pos.x(), pos.y(), 2.2);
        glColor3d(1, 1, 1);
        const double halfWidth = 32 / s.scale;
        const double halfHeight = 64 / s.scale;
        GUITexturesHelper::drawTexturedBox(GUITextureSubSys::getTexture(GUITexture::TLS), -halfWidth, -halfHeight, halfWidth, halfHeight);
        GLHelper::popMatrix();
    }
}


void
GNEJunction::drawElevation(const GUIVisualizationSettings& s, const GUIVisualizationSettings::Detail d) const {
    // check if draw elevation
    if ((d <= GUIVisualizationSettings::Detail::Text) && myNet->getViewNet()->getNetworkViewOptions().editingElevation()) {
        GLHelper::pushMatrix();
        // Translate to center of junction
        glTranslated(myNBNode->getPosition().x(), myNBNode->getPosition().y(), 0.1);
        // draw Z value
        GLHelper::drawText(toString(myNBNode->getPosition().z()), Position(), GLO_MAX - 5, s.junctionID.scaledSize(s.scale), s.junctionID.color);
        GLHelper::popMatrix();
    }
}


void
GNEJunction::drawJunctionName(const GUIVisualizationSettings& s) const {
    drawName(myNBNode->getPosition(), s.scale, s.junctionID);
    if (s.junctionName.show(this) && myNBNode->getName() != "") {
        GLHelper::drawTextSettings(s.junctionName, myNBNode->getName(), myNBNode->getPosition(), s.scale, s.angle);
    }
}


void
GNEJunction::drawJunctionChildren(const GUIVisualizationSettings& s, const GUIVisualizationSettings::Detail d) const {
    // check if draw junction elements
    if (s.drawForViewObjectsHandler || (d <= GUIVisualizationSettings::Detail::JunctionElement)) {
        // draw crossings
        for (const auto& crossing : myGNECrossings) {
            crossing->drawGL(s);
        }
        // draw walking areas
        for (const auto& walkingArea : myGNEWalkingAreas) {
            walkingArea->drawGL(s);
        }
        // draw internalLanes
        for (const auto& internalLanes : myInternalLanes) {
            internalLanes->drawGL(s);
        }
        // draw connections
        for (const auto& incomingEdge : myGNEIncomingEdges) {
            for (const auto& connection : incomingEdge->getGNEConnections()) {
                connection->drawGL(s);
            }
        }
        // draw child demand elements
        for (const auto& demandElement : getChildDemandElements()) {
            demandElement->drawGL(s);
        }
        // draw child demand elements
        for (const auto& demandElement : getChildDemandElements()) {
            demandElement->drawGL(s);
        }
        // draw path additional elements
        myNet->getPathManager()->drawJunctionPathElements(s, this);
    }
}


void
GNEJunction::calculateJunctioncontour(const GUIVisualizationSettings& s, const GUIVisualizationSettings::Detail d,
                                      const double exaggeration, const bool drawBubble) const {
    // if we're selecting using a boundary, first don't calculate contour bt check if edge boundary is within selection boundary
    if (gViewObjectsHandler.getSelectionBoundary().isInitialised() && gViewObjectsHandler.getSelectionBoundary().contains(myJunctionBoundary)) {
        // simply add object in ViewObjectsHandler with full boundary
        gViewObjectsHandler.addElementUnderCursor(this, getType(), false, true);
    } else {
        // always calculate for shape
        myNetworkElementContour.calculateContourClosedShape(s, d, this, myNBNode->getShape(), getType(), exaggeration);
        // check if calculate contour for bubble
        if (drawBubble) {
            myCircleContour.calculateContourCircleShape(s, d, this, myNBNode->getPosition(), s.neteditSizeSettings.junctionBubbleRadius, getType(), exaggeration);
        }
        // check geometry points if we're editing shape
        if (myShapeEdited) {
            myNetworkElementContour.calculateContourAllGeometryPoints(s, d, this, myNBNode->getShape(), getType(), s.neteditSizeSettings.junctionGeometryPointRadius,
                    exaggeration, true);
        }
    }
}


void
GNEJunction::setAttribute(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_KEEP_CLEAR: {
            throw InvalidArgument(toString(key) + " cannot be edited");
        }
        case SUMO_ATTR_ID: {
            myNet->getAttributeCarriers()->updateJunctionID(this, value);
            break;
        }
        case SUMO_ATTR_TYPE: {
            SumoXMLNodeType type = SUMOXMLDefinitions::NodeTypes.get(value);
            if (myNBNode->getType() == SumoXMLNodeType::PRIORITY
                    && (type == SumoXMLNodeType::RIGHT_BEFORE_LEFT || type == SumoXMLNodeType::LEFT_BEFORE_RIGHT)) {
                myNet->getNetBuilder()->getEdgeCont().removeRoundabout(myNBNode);
            }
            myNBNode->reinit(myNBNode->getPosition(), type);
            break;
        }
        case SUMO_ATTR_POSITION: {
            // set new position in NBNode updating edge boundaries
            moveJunctionGeometry(parse<Position>(value), true);
            // mark this connections and all of the junction's Neighbours as deprecated
            markConnectionsDeprecated(true);
            // update centering boundary and grid
            updateCenteringBoundary(true);
            break;
        }
        case GNE_ATTR_MODIFICATION_STATUS:
            if (myLogicStatus == FEATURE_GUESSED && value != FEATURE_GUESSED) {
                // clear guessed connections. previous connections will be restored
                myNBNode->invalidateIncomingConnections();
                // Clear GNEConnections of incoming edges
                for (const auto& i : myGNEIncomingEdges) {
                    i->clearGNEConnections();
                }
            }
            myLogicStatus = value;
            break;
        case SUMO_ATTR_SHAPE: {
            // set new shape (without updating grid)
            myNBNode->setCustomShape(parse<PositionVector>(value));
            // mark this connections and all of the junction's neighbors as deprecated
            markConnectionsDeprecated(true);
            // update centering boundary and grid
            updateCenteringBoundary(true);
            break;
        }
        case SUMO_ATTR_RADIUS: {
            if (value.empty() || (value == "default")) {
                myNBNode->setRadius(-1);
            } else {
                myNBNode->setRadius(parse<double>(value));
            }
            break;
        }
        case SUMO_ATTR_TLTYPE: {
            // we need to make a copy of controlling TLS (because original will be updated)
            const std::set<NBTrafficLightDefinition*> copyOfTls = myNBNode->getControllingTLS();
            for (const auto& TLS : copyOfTls) {
                TLS->setType(SUMOXMLDefinitions::TrafficLightTypes.get(value));
            }
            break;
        }
        case SUMO_ATTR_TLLAYOUT:
            // should not be triggered (handled via GNEChange_TLS)
            break;
        case SUMO_ATTR_RIGHT_OF_WAY:
            myNBNode->setRightOfWay(SUMOXMLDefinitions::RightOfWayValues.get(value));
            break;
        case SUMO_ATTR_FRINGE:
            myNBNode->setFringeType(SUMOXMLDefinitions::FringeTypeValues.get(value));
            break;
        case SUMO_ATTR_NAME:
            myNBNode->setName(value);
            break;
        case GNE_ATTR_SELECTED:
            if (parse<bool>(value)) {
                selectAttributeCarrier();
            } else {
                unselectAttributeCarrier();
            }
            break;
        case GNE_ATTR_PARAMETERS:
            myNBNode->setParametersStr(value);
            break;
        default:
            throw InvalidArgument(getTagStr() + " doesn't have an attribute of type '" + toString(key) + "'");
    }
    // invalidate path calculator
    myNet->getPathManager()->getPathCalculator()->invalidatePathCalculator();
}


void
GNEJunction::setMoveShape(const GNEMoveResult& moveResult) {
    // clear contour
    myNetworkElementContour.clearContour();
    // set new position in NBNode without updating grid
    if (isShapeEdited()) {
        // set new shape
        myNBNode->setCustomShape(moveResult.shapeToUpdate);
    } else if (moveResult.shapeToUpdate.size() > 0) {
        // obtain NBNode position
        const Position orig = myNBNode->getPosition();
        // move geometry
        moveJunctionGeometry(moveResult.shapeToUpdate.front(), false);
        // set new position of adjacent edges depending if we're moving a selection
        for (const auto& NBEdge : getNBNode()->getEdges()) {
            myNet->getAttributeCarriers()->retrieveEdge(NBEdge->getID())->updateJunctionPosition(this, orig);
        }
    }
    updateGeometry();
}


void
GNEJunction::commitMoveShape(const GNEMoveResult& moveResult, GNEUndoList* undoList) {
    // make sure that newShape isn't empty
    if (moveResult.shapeToUpdate.size() > 0) {
        // check if we're editing a shape
        if (isShapeEdited()) {
            // commit new shape
            undoList->begin(this, "moving " + toString(SUMO_ATTR_SHAPE) + " of " + getTagStr());
            setAttribute(SUMO_ATTR_SHAPE, toString(moveResult.shapeToUpdate), undoList);
            undoList->end();
        } else if (myNBNode->hasCustomShape()) {
            // commit new shape
            undoList->begin(this, "moving custom " + toString(SUMO_ATTR_SHAPE) + " of " + getTagStr());
            setAttribute(SUMO_ATTR_POSITION, toString(moveResult.shapeToUpdate.front()), undoList);
            // calculate offset and apply to custom shape
            const auto customShapeOffset = moveResult.shapeToUpdate.front() - myNBNode->getCenter();
            const auto customShapeMoved = myNBNode->getShape().added(customShapeOffset);
            setAttribute(SUMO_ATTR_SHAPE, toString(customShapeMoved), undoList);
            undoList->end();
        } else {
            setAttribute(SUMO_ATTR_POSITION, toString(moveResult.shapeToUpdate.front()), undoList);
        }
        // check merge junctions
        myNet->getViewNet()->checkMergeJunctions();
    }
}


double
GNEJunction::getColorValue(const GUIVisualizationSettings& /* s */, int activeScheme) const {
    switch (activeScheme) {
        case 0:
            if (myColorForMissingConnections) {
                return 3;
            } else {
                return 0;
            }
        case 1:
            return isAttributeCarrierSelected();
        case 2:
            switch (myNBNode->getType()) {
                case SumoXMLNodeType::TRAFFIC_LIGHT:
                    return 0;
                case SumoXMLNodeType::TRAFFIC_LIGHT_NOJUNCTION:
                    return 1;
                case SumoXMLNodeType::PRIORITY:
                    return 2;
                case SumoXMLNodeType::PRIORITY_STOP:
                    return 3;
                case SumoXMLNodeType::RIGHT_BEFORE_LEFT:
                    return 4;
                case SumoXMLNodeType::ALLWAY_STOP:
                    return 5;
                case SumoXMLNodeType::DISTRICT:
                    return 6;
                case SumoXMLNodeType::NOJUNCTION:
                    return 7;
                case SumoXMLNodeType::DEAD_END:
                case SumoXMLNodeType::DEAD_END_DEPRECATED:
                    return 8;
                case SumoXMLNodeType::UNKNOWN:
                    return 8; // may happen before first network computation
                case SumoXMLNodeType::INTERNAL:
                    assert(false);
                    return 8;
                case SumoXMLNodeType::RAIL_SIGNAL:
                    return 9;
                case SumoXMLNodeType::ZIPPER:
                    return 10;
                case SumoXMLNodeType::TRAFFIC_LIGHT_RIGHT_ON_RED:
                    return 11;
                case SumoXMLNodeType::RAIL_CROSSING:
                    return 12;
                case SumoXMLNodeType::LEFT_BEFORE_RIGHT:
                    return 13;
                default:
                    assert(false);
                    return 0;
            }
        case 3:
            return myNBNode->getPosition().z();
        default:
            assert(false);
            return 0;
    }
}

void
GNEJunction::checkMissingConnections() {
    for (auto edge : myGNEIncomingEdges) {
        if (edge->getGNEConnections().size() > 0) {
            myColorForMissingConnections = false;
            return;
        }
    }
    // no connections. Use normal color for border edges and cul-de-sac
    if (myGNEIncomingEdges.size() == 0 || myGNEOutgoingEdges.size() == 0) {
        myColorForMissingConnections = false;
        return;
    } else if (myGNEIncomingEdges.size() == 1 && myGNEOutgoingEdges.size() == 1) {
        NBEdge* in = myGNEIncomingEdges[0]->getNBEdge();
        NBEdge* out = myGNEOutgoingEdges[0]->getNBEdge();
        if (in->isTurningDirectionAt(out)) {
            myColorForMissingConnections = false;
            return;
        }
    }
    myColorForMissingConnections = true;
}


void
GNEJunction::moveJunctionGeometry(const Position& pos, const bool updateEdgeBoundaries) {
    // reinit NBNode
    myNBNode->reinit(pos, myNBNode->getType());
    // declare three sets with all affected GNEJunctions, GNEEdges and GNEConnections
    std::set<GNEJunction*> affectedJunctions;
    std::set<GNEEdge*> affectedEdges;
    // Iterate over GNEEdges
    for (const auto& edge : getChildEdges()) {
        // Add source and destination junctions
        affectedJunctions.insert(edge->getFromJunction());
        affectedJunctions.insert(edge->getToJunction());
        // Obtain neighbors of Junction source
        for (const auto& junctionSourceEdge : edge->getFromJunction()->getChildEdges()) {
            affectedEdges.insert(junctionSourceEdge);
        }
        // Obtain neighbors of Junction destination
        for (const auto& junctionDestinationEdge : edge->getToJunction()->getChildEdges()) {
            affectedEdges.insert(junctionDestinationEdge);
        }
    }
    // reset walking areas of affected edges
    for (const auto& affectedJunction : affectedJunctions) {
        affectedJunction->clearWalkingAreas();
    }
    // Iterate over affected Edges
    for (const auto& affectedEdge : affectedEdges) {
        // update edge boundaries
        if (updateEdgeBoundaries) {
            affectedEdge->updateCenteringBoundary(true);
        }
        // Update edge geometry
        affectedEdge->updateGeometry();
    }
}


RGBColor
GNEJunction::setColor(const GUIVisualizationSettings& s, bool bubble) const {
    // get active scheme
    const int scheme = s.junctionColorer.getActive();
    // first check if we're editing shape
    if (myShapeEdited) {
        return s.colorSettings.editShapeColor;
    }
    // set default color
    RGBColor color = s.junctionColorer.getScheme().getColor(getColorValue(s, scheme));
    // set special bubble color
    if (bubble && (scheme == 0) && !myColorForMissingConnections) {
        color = s.junctionColorer.getScheme().getColor(1);
    }
    // override with special colors (unless the color scheme is based on selection)
    if (drawUsingSelectColor() && scheme != 1) {
        color = s.colorSettings.selectionColor;
    }
    // overwrite color if we're in data mode
    if (myNet->getViewNet()->getEditModes().isCurrentSupermodeData()) {
        color = s.junctionColorer.getScheme().getColor(6);
    }
    // special color for source candidate junction
    if (mySourceCandidate) {
        color = s.candidateColorSettings.source;
    }
    // special color for target candidate junction
    if (myTargetCandidate) {
        color = s.candidateColorSettings.target;
    }
    // special color for special candidate junction
    if (mySpecialCandidate) {
        color = s.candidateColorSettings.special;
    }
    // special color for possible candidate junction
    if (myPossibleCandidate) {
        color = s.candidateColorSettings.possible;
    }
    // special color for conflicted candidate junction
    if (myConflictedCandidate) {
        color = s.candidateColorSettings.conflict;
    }
    // return color
    return color;
}


void
GNEJunction::addTrafficLight(NBTrafficLightDefinition* tlDef, bool forceInsert) {
    NBTrafficLightLogicCont& tlCont = myNet->getTLLogicCont();
    tlCont.insert(tlDef, forceInsert); // may return false for tlDef which controls multiple junctions
    tlDef->addNode(myNBNode);
}


void
GNEJunction::removeTrafficLight(NBTrafficLightDefinition* tlDef) {
    NBTrafficLightLogicCont& tlCont = myNet->getTLLogicCont();
    if (tlDef->getNodes().size() == 1) {
        tlCont.extract(tlDef);
    }
    myNBNode->removeTrafficLight(tlDef);
}


/****************************************************************************/
