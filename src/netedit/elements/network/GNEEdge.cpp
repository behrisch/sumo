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
/// @file    GNEEdge.cpp
/// @author  Jakob Erdmann
/// @date    Feb 2011
///
// A road/street connecting two junctions (netedit-version, adapted from GUIEdge)
// Basically a container for an NBEdge with drawing and editing capabilities
/****************************************************************************/

#include <netedit/changes/GNEChange_Attribute.h>
#include <netedit/changes/GNEChange_Lane.h>
#include <netedit/elements/additional/GNERouteProbe.h>
#include <netedit/elements/demand/GNEPlanParents.h>
#include <netedit/elements/demand/GNERoute.h>
#include <netedit/elements/moving/GNEMoveElementEdge.h>
#include <netedit/frames/common/GNEDeleteFrame.h>
#include <netedit/frames/common/GNEInspectorFrame.h>
#include <netedit/frames/common/GNEMoveFrame.h>
#include <netedit/frames/data/GNEEdgeDataFrame.h>
#include <netedit/frames/data/GNEEdgeRelDataFrame.h>
#include <netedit/frames/demand/GNEContainerFrame.h>
#include <netedit/frames/demand/GNEContainerPlanFrame.h>
#include <netedit/frames/demand/GNEPersonFrame.h>
#include <netedit/frames/demand/GNEPersonPlanFrame.h>
#include <netedit/frames/demand/GNEVehicleFrame.h>
#include <netedit/frames/GNEElementTree.h>
#include <netedit/frames/GNEViewObjectSelector.h>
#include <netedit/frames/network/GNEAdditionalFrame.h>
#include <netedit/GNENet.h>
#include <netedit/GNETagPropertiesDatabase.h>
#include <netedit/GNEUndoList.h>
#include <netedit/GNEViewParent.h>
#include <utils/gui/div/GLHelper.h>
#include <utils/options/OptionsCont.h>

#include "GNEConnection.h"
#include "GNECrossing.h"
#include "GNEEdge.h"
#include "GNEEdgeType.h"
#include "GNELaneType.h"
#include "GNEEdgeTemplate.h"
#include "GNELaneTemplate.h"

// ===========================================================================
// defines
// ===========================================================================

//#define DEBUG_SMOOTH_GEOM
//#define DEBUGCOND(obj) (true)
#define VEHICLE_GAP 1
#define ENDPOINT_TOLERANCE 2

// ===========================================================================
// static
// ===========================================================================

const double GNEEdge::SNAP_RADIUS = SUMO_const_halfLaneWidth;
const double GNEEdge::SNAP_RADIUS_SQUARED = (SUMO_const_halfLaneWidth* SUMO_const_halfLaneWidth);

// ===========================================================================
// members methods
// ===========================================================================

GNEEdge::GNEEdge(GNENet* net, NBEdge* nbe, bool wasSplit, bool loaded):
    GNENetworkElement(net, nbe->getID(), SUMO_TAG_EDGE),
    myNBEdge(nbe),
    myAmResponsible(false),
    myWasSplit(wasSplit),
    myConnectionStatus(loaded ? FEATURE_LOADED : FEATURE_GUESSED),
    myMoveElementEdge(new GNEMoveElementEdge(this)),
    myUpdateGeometry(true) {
    // set parents
    setParents<GNEJunction*>({
        net->getAttributeCarriers()->retrieveJunction(nbe->getFromNode()->getID()),
        net->getAttributeCarriers()->retrieveJunction(nbe->getToNode()->getID())
    });
    // Create lanes
    for (int i = 0; i < myNBEdge->getNumLanes(); i++) {
        auto lane = new GNELane(this, i);
        lane->incRef("GNEEdge::GNEEdge");
        addChildElement(lane);
    }
    // update Lane geometries
    for (const auto& lane : getChildLanes()) {
        lane->updateGeometry();
    }
    // update centering boundary without updating grid
    updateCenteringBoundary(false);
}


GNEEdge::~GNEEdge() {
    // Delete references to this edge in lanes
    for (const auto& lane : getChildLanes()) {
        lane->decRef("GNEEdge::~GNEEdge");
        if (lane->unreferenced()) {
            // check if remove it from Attribute Carriers
            if (myNet->getAttributeCarriers()->getLanes().count(lane) > 0) {
                myNet->getAttributeCarriers()->deleteLane(lane);
            }
            delete lane;
        }
    }
    // delete references to this edge in connections
    for (const auto& connection : myGNEConnections) {
        connection->decRef("GNEEdge::~GNEEdge");
        if (connection->unreferenced()) {
            // check if remove it from Attribute Carriers
            if (myNet->getAttributeCarriers()->getConnections().count(connection) > 0) {
                myNet->getAttributeCarriers()->deleteConnection(connection);
            }
            delete connection;
        }
    }
    if (myAmResponsible) {
        delete myNBEdge;
    }
}


GNEMoveElement*
GNEEdge::getMoveElement() const {
    return myMoveElementEdge;
}


Parameterised*
GNEEdge::getParameters() {
    return myNBEdge;
}


const Parameterised*
GNEEdge::getParameters() const {
    return myNBEdge;
}


bool
GNEEdge::isNetworkElementValid() const {
    if (getFromJunction()->getNBNode()->getPosition() != getToJunction()->getNBNode()->getPosition()) {
        return true;
    } else {
        return false;
    }
}


std::string
GNEEdge::getNetworkElementProblem() const {
    return TLF("Parent junctions are in the same position: %, %",
               toString(getFromJunction()->getNBNode()->getPosition().x()),
               toString(getFromJunction()->getNBNode()->getPosition().y()));
}


void
GNEEdge::updateGeometry() {
    // first check if myUpdateGeometry flag is enabled
    if (myUpdateGeometry) {
        // Update geometry of lanes
        for (const auto& lane : getChildLanes()) {
            lane->updateGeometry();
        }
        // Update geometry of connections
        for (const auto& connection : myGNEConnections) {
            connection->updateGeometry();
        }
        // Update geometry of additionals children vinculated to this edge
        for (const auto& childAdditional : getChildAdditionals()) {
            childAdditional->updateGeometry();
        }
        // Update geometry of additionals demand elements vinculated to this edge
        for (const auto& childDemandElement : getChildDemandElements()) {
            childDemandElement->updateGeometry();
        }
        // Update geometry of additionals generic datas vinculated to this edge
        for (const auto& childGenericData : getChildGenericDatas()) {
            childGenericData->updateGeometry();
        }
        // compute geometry of path elements elements vinculated with this edge (depending of showDemandElements)
        if (myNet->getViewNet() && myNet->getViewNet()->getNetworkViewOptions().showDemandElements()) {
            for (const auto& childAdditional : getChildAdditionals()) {
                childAdditional->computePathElement();
            }
            for (const auto& childDemandElement : getChildDemandElements()) {
                childDemandElement->computePathElement();
            }
            for (const auto& childGenericData : getChildGenericDatas()) {
                childGenericData->computePathElement();
            }
        }
    }
    // update vehicle geometry
    updateVehicleSpreadGeometries();
    // update stack labels
    updateVehicleStackLabels();
    updatePersonStackLabels();
    updateContainerStackLabels();
}


Position
GNEEdge::getPositionInView() const {
    return getChildLanes().front()->getPositionInView();
}


bool
GNEEdge::checkDrawFromContour() const {
    // get modes and viewParent (for code legibility)
    const auto& modes = myNet->getViewNet()->getEditModes();
    const auto& viewParent = myNet->getViewParent();
    const auto& inspectedElements = myNet->getViewNet()->getInspectedElements();
    // continue depending of current status
    if (inspectedElements.isInspectingSingleElement()) {
        // check if starts in this edge
        if (inspectedElements.getFirstAC()->getTagProperty()->vehicleEdges() &&
                inspectedElements.getFirstAC()->hasAttribute(SUMO_ATTR_FROM) &&
                (inspectedElements.getFirstAC()->getAttribute(SUMO_ATTR_FROM) == getID())) {
            return true;
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
            if (planCreator->getPlanParameteres().fromEdge == getID()) {
                return true;
            } else if ((planCreator->getPlanParameteres().consecutiveEdges.size() > 0) &&
                       (planCreator->getPlanParameteres().consecutiveEdges.front() == getID())) {
                return true;
            }
        } else if (modes.demandEditMode == DemandEditMode::DEMAND_VEHICLE) {
            const auto& selectedEdges = viewParent->getVehicleFrame()->getPathCreator()->getSelectedEdges();
            // check if this is the first selected edge
            if ((selectedEdges.size() > 0) && (selectedEdges.front() == this)) {
                return true;
            }
        }
    } else if (modes.isCurrentSupermodeData()) {
        // get TAZRelDataFrame
        const auto& edgeRelDataFrame = viewParent->getEdgeRelDataFrame();
        if (edgeRelDataFrame->shown()) {
            // check first Edge
            if (edgeRelDataFrame->getPathCreator()->getSelectedEdges().empty()) {
                return false;
            } else {
                return edgeRelDataFrame->getPathCreator()->getSelectedEdges().front() == this;
            }
        }
    }
    // nothing to draw
    return false;
}


bool
GNEEdge::checkDrawToContour() const {
    // get modes and viewParent (for code legibility)
    const auto& modes = myNet->getViewNet()->getEditModes();
    const auto& viewParent = myNet->getViewParent();
    const auto& inspectedElements = myNet->getViewNet()->getInspectedElements();
    // continue depending of current status
    if (inspectedElements.isInspectingSingleElement()) {
        // check if starts in this edge
        if (inspectedElements.getFirstAC()->getTagProperty()->vehicleEdges() &&
                inspectedElements.getFirstAC()->hasAttribute(SUMO_ATTR_TO) &&
                (inspectedElements.getFirstAC()->getAttribute(SUMO_ATTR_TO) == getID())) {
            return true;
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
            if (planCreator->getPlanParameteres().toEdge == getID()) {
                return true;
            } else if ((planCreator->getPlanParameteres().consecutiveEdges.size() > 1) &&
                       (planCreator->getPlanParameteres().consecutiveEdges.back() == getID())) {
                return true;
            }
        } else if (modes.demandEditMode == DemandEditMode::DEMAND_VEHICLE) {
            const auto& selectedEdges = viewParent->getVehicleFrame()->getPathCreator()->getSelectedEdges();
            // check if this is the last selected edge
            if ((selectedEdges.size() > 1) && (selectedEdges.back() == this)) {
                return true;
            }
        }
    } else if (modes.isCurrentSupermodeData()) {
        // get TAZRelDataFrame
        const auto& edgeRelDataFrame = viewParent->getEdgeRelDataFrame();
        if (edgeRelDataFrame->shown()) {
            // check last Edge
            if (edgeRelDataFrame->getPathCreator()->getSelectedEdges().empty()) {
                return false;
            } else {
                return edgeRelDataFrame->getPathCreator()->getSelectedEdges().back() == this;
            }
        }
    }
    // nothing to draw
    return false;
}


bool
GNEEdge::checkDrawRelatedContour() const {
    const auto& inspectedElements = myNet->getViewNet()->getInspectedElements();
    // continue depending of inspected elements
    if (inspectedElements.isInspectingSingleElement() &&
            inspectedElements.getFirstAC()->getTagProperty()->getTag() == SUMO_TAG_TAZ) {
        // check if one of the sourceSink child is placed in this edge
        for (const auto& sourceSink : getChildTAZSourceSinks()) {
            if (sourceSink->getParentAdditionals().front() == inspectedElements.getFirstAC()) {
                return true;
            }
        }
    }
    if (myNet->getViewNet()->getPopup()) {
        return myNet->getViewNet()->getPopup()->getGLObject() == this;
    }
    return false;
}


bool
GNEEdge::checkDrawOverContour() const {
    // get modes and viewParent (for code legibility)
    const auto& modes = myNet->getViewNet()->getEditModes();
    const auto& viewParent = myNet->getViewParent();
    const auto& viewObjectsSelector = myNet->getViewNet()->getViewObjectsSelector();
    // check if we're selecting edges in additional mode
    if (modes.isCurrentSupermodeNetwork() && (modes.networkEditMode == NetworkEditMode::NETWORK_ADDITIONAL)) {
        if (viewParent->getAdditionalFrame()->getViewObjetsSelector()->isNetworkElementSelected(this)) {
            return true;
        } else if (viewParent->getAdditionalFrame()->getViewObjetsSelector()->getTag() == myTagProperty->getTag()) {
            return viewObjectsSelector.getEdgeFront() == this;
        } else {
            return false;
        }
    }
    // check if this is the edge under cursor
    if (viewObjectsSelector.getEdgeFront() != this) {
        return false;
    } else {
        // continue depending of modes
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
            if (planSelector && planSelector->markEdges()) {
                return (viewObjectsSelector.getAttributeCarrierFront()->getTagProperty()->getTag() == SUMO_TAG_LANE);
            } else if (modes.demandEditMode == DemandEditMode::DEMAND_VEHICLE) {
                // get current vehicle template
                const auto& vehicleTemplate = viewParent->getVehicleFrame()->getVehicleTagSelector()->getCurrentTemplateAC();
                // check if vehicle can be placed over from-to TAZs
                if (vehicleTemplate && vehicleTemplate->getTagProperty()->vehicleEdges()) {
                    return (viewObjectsSelector.getAttributeCarrierFront()->getTagProperty()->getTag() == SUMO_TAG_LANE);
                }
            }
        } else if (modes.isCurrentSupermodeData()) {
            // get frames
            const auto& edgeDataFrame = viewParent->getEdgeDataFrame();
            const auto& edgeRelDataFrame = viewParent->getEdgeRelDataFrame();
            if (edgeDataFrame->shown()) {
                return true;
            } else if (edgeRelDataFrame->shown()) {
                // check edges
                if (edgeRelDataFrame->getPathCreator()->getSelectedEdges().empty()) {
                    return true;
                } else if (edgeRelDataFrame->getPathCreator()->getSelectedEdges().size() == 2) {
                    return false;
                } else if (edgeRelDataFrame->getPathCreator()->getSelectedEdges().front() == this) {
                    return false;
                } else if (edgeRelDataFrame->getPathCreator()->getSelectedEdges().back() == this) {
                    return false;
                } else {
                    return true;
                }
            }

        }
        // nothing to draw
        return false;
    }
}


bool
GNEEdge::checkDrawDeleteContour() const {
    // first check if we're selecting edges or lanes
    if (myNet->getViewNet()->checkSelectEdges()) {
        // get edit modes
        const auto& editModes = myNet->getViewNet()->getEditModes();
        // check if we're in delete mode
        if (editModes.isCurrentSupermodeNetwork() && (editModes.networkEditMode == NetworkEditMode::NETWORK_DELETE)) {
            // check lanes
            for (const auto& lane : getChildLanes()) {
                if (myNet->getViewNet()->checkOverLockedElement(lane, mySelected) &&
                        (myNet->getViewNet()->getViewObjectsSelector().getGUIGlObjectFront() == lane)) {
                    return true;
                }
            }
            // check edge
            if (myNet->getViewNet()->checkOverLockedElement(this, mySelected) &&
                    (myNet->getViewNet()->getViewObjectsSelector().getGUIGlObjectFront() == this)) {
                return true;
            }
            // nothing to draw
            return false;
        } else {
            return false;
        }
    } else {
        return false;
    }
}


bool
GNEEdge::checkDrawDeleteContourSmall() const {
    // get edit modes
    const auto& editModes = myNet->getViewNet()->getEditModes();
    // check if we're in delete mode
    if (editModes.isCurrentSupermodeNetwork() && (editModes.networkEditMode == NetworkEditMode::NETWORK_DELETE)) {
        const auto junction = myNet->getViewNet()->getViewObjectsSelector().getJunctionFront();
        if (junction == myNet->getViewNet()->getViewObjectsSelector().getAttributeCarrierFront()) {
            return ((getFromJunction() == junction) || (getToJunction() == junction));
        }
    }
    return false;
}


bool
GNEEdge::checkDrawSelectContour() const {
    // first check if we're selecting edges or lanes
    if (myNet->getViewNet()->checkSelectEdges()) {
        // get edit modes
        const auto& editModes = myNet->getViewNet()->getEditModes();
        // check if we're in select mode
        if (editModes.isCurrentSupermodeNetwork() && (editModes.networkEditMode == NetworkEditMode::NETWORK_SELECT)) {
            // check lanes
            for (const auto& lane : getChildLanes()) {
                if (myNet->getViewNet()->checkOverLockedElement(lane, mySelected) &&
                        (myNet->getViewNet()->getViewObjectsSelector().getGUIGlObjectFront() == lane)) {
                    return true;
                }
            }
            // check edge
            if (myNet->getViewNet()->checkOverLockedElement(this, mySelected) &&
                    (myNet->getViewNet()->getViewObjectsSelector().getGUIGlObjectFront() == this)) {
                return true;
            }
            // nothing to draw
            return false;
        } else {
            return false;
        }
    } else {
        return false;
    }
}


bool
GNEEdge::checkDrawMoveContour() const {
    // get edit modes
    const auto& editModes = myNet->getViewNet()->getEditModes();
    // get move element edge
    const auto moveElementEdge = dynamic_cast<GNEMoveElementEdge*>(myNet->getViewNet()->getMoveSingleElementValues().getMovedElement());
    // check if we're in move mode
    if ((!myNet->getViewNet()->isCurrentlyMovingElements() || (moveElementEdge && (moveElementEdge->getEdge() == this))) && editModes.isCurrentSupermodeNetwork() &&
            (editModes.networkEditMode == NetworkEditMode::NETWORK_MOVE)) {
        // check if we're editing this network element
        const GNENetworkElement* editedNetworkElement = myNet->getViewNet()->getEditNetworkElementShapes().getEditedNetworkElement();
        if (editedNetworkElement) {
            if (editedNetworkElement == this) {
                return true;
            } else {
                // check lanes
                for (const auto& lane : getChildLanes()) {
                    if (editedNetworkElement == lane) {
                        return true;
                    }
                }
            }
        } else {
            // check lanes
            for (const auto& lane : getChildLanes()) {
                if (myNet->getViewNet()->checkOverLockedElement(lane, mySelected) &&
                        (myNet->getViewNet()->getViewObjectsSelector().getGUIGlObjectFront() == lane)) {
                    return true;
                }
            }
            // check edge
            if (myNet->getViewNet()->checkOverLockedElement(this, mySelected) &&
                    (myNet->getViewNet()->getViewObjectsSelector().getGUIGlObjectFront() == this)) {
                return true;
            }
        }
        // nothing to draw
        return false;
    } else {
        return false;
    }
}


bool
GNEEdge::hasCustomEndPoints() const {
    if (myNBEdge->getGeometry().front().distanceSquaredTo2D(getFromJunction()->getNBNode()->getPosition()) > ENDPOINT_TOLERANCE) {
        return true;
    } else if (myNBEdge->getGeometry().back().distanceSquaredTo2D(getToJunction()->getNBNode()->getPosition()) > ENDPOINT_TOLERANCE) {
        return true;
    } else {
        return false;
    }
}


bool
GNEEdge::clickedOverShapeStart(const Position& pos) const {
    // get geometry point radius
    const double geometryPointRadius = getGeometryPointRadius();
    if (myNBEdge->getGeometry().front().distanceSquaredTo2D(getFromJunction()->getNBNode()->getPosition()) > ENDPOINT_TOLERANCE) {
        return (myNBEdge->getGeometry().front().distanceSquaredTo2D(pos) < (geometryPointRadius * geometryPointRadius));
    } else {
        return false;
    }
}


bool
GNEEdge::clickedOverShapeEnd(const Position& pos) const {
    // get geometry point radius
    const double geometryPointRadius = getGeometryPointRadius();
    if (myNBEdge->getGeometry().back().distanceSquaredTo2D(getToJunction()->getNBNode()->getPosition()) > ENDPOINT_TOLERANCE) {
        return (myNBEdge->getGeometry().back().distanceSquaredTo2D(pos) < (geometryPointRadius * geometryPointRadius));
    } else {
        return false;
    }
}


bool
GNEEdge::clickedOverGeometryPoint(const Position& pos) const {
    // get geometry point radius
    const auto geometryPointRadius = getGeometryPointRadius();
    // first check inner geometry
    const PositionVector innenShape = myNBEdge->getInnerGeometry();
    // iterate over geometry point
    for (const auto& geometryPoint : innenShape) {
        if (geometryPoint.distanceSquaredTo2D(pos) < (geometryPointRadius * geometryPointRadius)) {
            return true;
        }
    }
    // check start and end shapes
    if (clickedOverShapeStart(pos) || clickedOverShapeEnd(pos)) {
        return true;
    } else {
        return false;
    }
}


void
GNEEdge::updateJunctionPosition(GNEJunction* junction, const Position& origPos) {
    Position delta = junction->getNBNode()->getPosition() - origPos;
    PositionVector geom = myNBEdge->getGeometry();
    // geometry endpoint need not equal junction position hence we modify it with delta
    if (junction == getFromJunction()) {
        geom[0].add(delta);
    } else {
        geom[-1].add(delta);
    }
    setGeometry(geom, false);
}


double
GNEEdge::getExaggeration(const GUIVisualizationSettings& s) const {
    return s.addSize.getExaggeration(s, this);
}


Boundary
GNEEdge::getCenteringBoundary() const {
    return myEdgeBoundary;
}


void
GNEEdge::updateCenteringBoundary(const bool updateGrid) {
    // Remove object from net
    if (updateGrid) {
        myNet->removeGLObjectFromGrid(this);
    }
    // first add edge boundary
    myEdgeBoundary = myNBEdge->getGeometry().getBoxBoundary();
    // add lane boundaries
    for (const auto& lane : getChildLanes()) {
        const auto laneBoundary = lane->getCenteringBoundary();
        if (laneBoundary.isInitialised()) {
            myEdgeBoundary.add(laneBoundary);
            // add additional and demand boundaries
            for (const auto& additional : lane->getChildAdditionals()) {
                const auto additionalBoundary = additional->getCenteringBoundary();
                if (additionalBoundary.isInitialised()) {
                    myEdgeBoundary.add(additional->getCenteringBoundary());
                }
            }
        }
    }
    // add additional and demand boundaries
    for (const auto& additional : getChildAdditionals()) {
        const auto additionalBoundary = additional->getCenteringBoundary();
        if (additionalBoundary.isInitialised()) {
            myEdgeBoundary.add(additionalBoundary);
        }
    }
    // add junction positions
    myEdgeBoundary.add(getFromJunction()->getCenteringBoundary());
    myEdgeBoundary.add(getToJunction()->getCenteringBoundary());
    // grow boundary
    myEdgeBoundary.grow(5);
    // add object into net
    if (updateGrid) {
        myNet->addGLObjectIntoGrid(this);
    }
}


const std::string
GNEEdge::getOptionalName() const {
    return myNBEdge->getStreetName();
}


GUIGLObjectPopupMenu*
GNEEdge::getPopUpMenu(GUIMainWindow& app, GUISUMOAbstractView& parent) {
    // if we call this function, that's mean that we're clicked over a edge geometry point, then
    // open the popup dialog of the lane[0] (back)
    return getChildLanes().back()->getPopUpMenu(app, parent);
}


std::vector<GNEEdge*>
GNEEdge::getOppositeEdges() const {
    return myNet->getAttributeCarriers()->retrieveEdges(getToJunction(), getFromJunction());
}


void
GNEEdge::drawGL(const GUIVisualizationSettings& s) const {
    // check drawing boundary selection and size boundary
    if (checkDrawingBoundarySelection() && s.checkDrawEdge(myEdgeBoundary)) {
        // draw boundary
        GLHelper::drawBoundary(s, getCenteringBoundary());
        // get detail level from the first lane
        const auto d = getChildLanes().front()->getDrawingConstants()->getDetail();
        // calculate layer
        double layer = GLO_EDGE;
        if (myDrawInFront) {
            layer = GLO_FRONTELEMENT;
        } else if (getChildLanes().front()->getLaneShape().length2D() <= (s.neteditSizeSettings.junctionBubbleRadius * 2)) {
            layer = GLO_JUNCTION + 1.8;
        }
        // check if draw details
        if (!s.drawForViewObjectsHandler) {
            // draw geometry points
            drawEdgeGeometryPoints(s, d, layer);
            // draw edge shape (a red line only visible if lane shape is strange)
            drawEdgeShape(s, d, layer);
            // draw edge stopOffset
            drawLaneStopOffset(s, d, layer);
            // draw edge name
            drawEdgeName(s);
            // draw lock icon
            GNEViewNetHelper::LockIcon::drawLockIcon(d, this, getType(), getPositionInView(), 1);
            // draw dotted contour
            myNetworkElementContour.drawDottedContours(s, d, this, s.dottedContourSettings.segmentWidth, true);
        }
        // calculate edge contour (always before children)
        calculateEdgeContour(s, d, layer);
        // draw lanes
        for (const auto& lane : getChildLanes()) {
            lane->drawGL(s);
        }
        // draw junctions
        getFromJunction()->drawGL(s);
        getToJunction()->drawGL(s);
        // draw childrens
        drawChildrens(s);
    }
}


void
GNEEdge::deleteGLObject() {
    // Check if edge can be deleted
    if (GNEDeleteFrame::SubordinatedElements(this).checkElements(myNet->getViewParent()->getDeleteFrame()->getProtectElements())) {
        myNet->deleteEdge(this, myNet->getUndoList(), false);
    }
}


void
GNEEdge::updateGLObject() {
    updateGeometry();
}


NBEdge*
GNEEdge::getNBEdge() const {
    return myNBEdge;
}


Position
GNEEdge::getSplitPos(const Position& clickPos) {
    // get geometry point radius
    const double geometryPointRadius = getGeometryPointRadius();
    const PositionVector& geom = myNBEdge->getGeometry();
    int index = geom.indexOfClosest(clickPos, true);
    if (geom[index].distanceSquaredTo2D(clickPos) < (geometryPointRadius * geometryPointRadius)) {
        // split at existing geometry point
        return myNet->getViewNet()->snapToActiveGrid(geom[index]);
    } else {
        // split straight between the next two points
        return myNet->getViewNet()->snapToActiveGrid(geom.positionAtOffset(geom.nearest_offset_to_point2D(clickPos)));
    }
}


void
GNEEdge::editEndpoint(Position pos, GNEUndoList* undoList) {
    // get geometry point radius
    const double geometryPointRadius = getGeometryPointRadius();
    if ((myNBEdge->getGeometry().front().distanceSquaredTo2D(getFromJunction()->getNBNode()->getPosition()) > ENDPOINT_TOLERANCE) &&
            (myNBEdge->getGeometry().front().distanceSquaredTo2D(pos) < (geometryPointRadius * geometryPointRadius))) {
        undoList->begin(this, "remove endpoint");
        setAttribute(GNE_ATTR_SHAPE_START, "", undoList);
        undoList->end();
    } else if ((myNBEdge->getGeometry().back().distanceSquaredTo2D(getToJunction()->getNBNode()->getPosition()) > ENDPOINT_TOLERANCE) &&
               (myNBEdge->getGeometry().back().distanceSquaredTo2D(pos) < (geometryPointRadius * geometryPointRadius))) {
        undoList->begin(this, "remove endpoint");
        setAttribute(GNE_ATTR_SHAPE_END, "", undoList);
        undoList->end();
    } else {
        // we need to create new Start/End position over Edge shape, not over clicked position
        double offset = myNBEdge->getGeometry().nearest_offset_to_point2D(myNet->getViewNet()->snapToActiveGrid(pos), true);
        if (offset != GeomHelper::INVALID_OFFSET) {
            PositionVector geom = myNBEdge->getGeometry();
            // calculate position over edge shape relative to clicked position
            Position newPos = geom.positionAtOffset2D(offset);
            // snap new position to grid
            newPos = myNet->getViewNet()->snapToActiveGrid(newPos);
            undoList->begin(this, "set endpoint");
            const int index = geom.indexOfClosest(pos, true);
            const Position destPos = getToJunction()->getNBNode()->getPosition();
            const Position sourcePos = getFromJunction()->getNBNode()->getPosition();
            if (pos.distanceTo2D(destPos) < pos.distanceTo2D(sourcePos)) {
                // check if snap to existing geometrypoint
                if (geom[index].distanceSquaredTo2D(pos) < (geometryPointRadius * geometryPointRadius)) {
                    newPos = geom[index];
                    // remove existent geometry point to avoid double points
                    myMoveElementEdge->removeGeometryPoint(newPos, undoList);
                }
                setAttribute(GNE_ATTR_SHAPE_END, toString(newPos), undoList);
                getToJunction()->invalidateShape();
            } else {
                // check if snap to existing geometry point
                if (geom[index].distanceSquaredTo2D(pos) < (geometryPointRadius * geometryPointRadius)) {
                    newPos = geom[index];
                    // remove existent geometry point to avoid double points
                    myMoveElementEdge->removeGeometryPoint(newPos, undoList);
                }
                setAttribute(GNE_ATTR_SHAPE_START, toString(newPos), undoList);
                getFromJunction()->invalidateShape();
            }
            undoList->end();
        }
    }
}


void
GNEEdge::resetEndpoint(const Position& pos, GNEUndoList* undoList) {
    Position destPos = getToJunction()->getNBNode()->getPosition();
    Position sourcePos = getFromJunction()->getNBNode()->getPosition();
    if (pos.distanceTo2D(destPos) < pos.distanceTo2D(sourcePos)) {
        setAttribute(GNE_ATTR_SHAPE_END, toString(destPos), undoList);
        getToJunction()->invalidateShape();
    } else {
        setAttribute(GNE_ATTR_SHAPE_START, toString(sourcePos), undoList);
        getFromJunction()->invalidateShape();
    }
}


void
GNEEdge::resetBothEndpoint(GNEUndoList* undoList) {
    // reset shape start
    setAttribute(GNE_ATTR_SHAPE_END, "", undoList);
    getToJunction()->invalidateShape();
    // reset shape end
    setAttribute(GNE_ATTR_SHAPE_START, "", undoList);
    getFromJunction()->invalidateShape();
}

void
GNEEdge::setGeometry(PositionVector geom, bool inner) {
    // set new geometry
    const bool lefthand = OptionsCont::getOptions().getBool("lefthand");
    if (lefthand) {
        geom.mirrorX();
        myNBEdge->mirrorX();
    }
    myNBEdge->setGeometry(geom, inner);
    if (lefthand) {
        myNBEdge->mirrorX();
    }
    // update geometry
    updateGeometry();
    // invalidate junction source shape
    getFromJunction()->invalidateShape();
    // iterate over first parent junction edges and update geometry
    for (const auto& edge : getFromJunction()->getGNEIncomingEdges()) {
        edge->updateGeometry();
    }
    for (const auto& edge : getFromJunction()->getGNEOutgoingEdges()) {
        edge->updateGeometry();
    }
    // invalidate junction destination shape
    getToJunction()->invalidateShape();
    // iterate over second parent junction edges and update geometry
    for (const auto& edge : getToJunction()->getGNEIncomingEdges()) {
        edge->updateGeometry();
    }
    for (const auto& edge : getToJunction()->getGNEOutgoingEdges()) {
        edge->updateGeometry();
    }
}


const Position
GNEEdge::getFrontUpShapePosition() const {
    PositionVector laneShape = getChildLanes().front()->getLaneShape();
    laneShape.move2side(getChildLanes().front()->getParentEdge()->getNBEdge()->getLaneWidth(getChildLanes().front()->getIndex()) / 2);
    return laneShape.front();
}


const Position
GNEEdge::getFrontDownShapePosition() const {
    PositionVector laneShape = getChildLanes().back()->getLaneShape();
    laneShape.move2side(-1 * getChildLanes().back()->getParentEdge()->getNBEdge()->getLaneWidth(getChildLanes().back()->getIndex()) / 2);
    return laneShape.front();
}


const Position
GNEEdge::getBackUpShapePosition() const {
    PositionVector laneShape = getChildLanes().front()->getLaneShape();
    laneShape.move2side(getChildLanes().front()->getParentEdge()->getNBEdge()->getLaneWidth(getChildLanes().front()->getIndex()) / 2);
    return laneShape.back();
}


const Position
GNEEdge::getBackDownShapePosition() const {
    PositionVector laneShape = getChildLanes().back()->getLaneShape();
    laneShape.move2side(-1 * getChildLanes().back()->getParentEdge()->getNBEdge()->getLaneWidth(getChildLanes().back()->getIndex()) / 2);
    return laneShape.back();
}

void
GNEEdge::remakeGNEConnections(bool junctionsReady) {
    // create new and removed unused GNEConnections
    const std::vector<NBEdge::Connection>& connections = myNBEdge->getConnections();
    // create a vector to keep retrieved and created connections
    std::vector<GNEConnection*> retrievedConnections;
    // iterate over NBEdge::Connections of GNEEdge
    for (const auto& connection : connections) {
        // retrieve existent GNEConnection, or create it
        GNEConnection* retrievedGNEConnection = retrieveGNEConnection(connection.fromLane, connection.toEdge, connection.toLane);
        if (junctionsReady) {
            retrievedGNEConnection->updateLinkState();
        }
        retrievedConnections.push_back(retrievedGNEConnection);
        // check if previously this GNEConnections exists, and if true, remove it from myGNEConnections
        std::vector<GNEConnection*>::iterator retrievedExists = std::find(myGNEConnections.begin(), myGNEConnections.end(), retrievedGNEConnection);
        if (retrievedExists != myGNEConnections.end()) {
            myGNEConnections.erase(retrievedExists);
        } else {
            // include reference to created GNEConnection
            retrievedGNEConnection->incRef("GNEEdge::remakeGNEConnections");
        }
        // mark it as deprecated
        retrievedGNEConnection->markConnectionGeometryDeprecated();
    }
    // delete non retrieved GNEConnections
    for (const auto& connection : myGNEConnections) {
        // decrease reference
        connection->decRef();
        // remove it from network
        myNet->removeGLObjectFromGrid(connection);
        // and from AttributeCarriers
        if (myNet->getAttributeCarriers()->getConnections().count(connection) > 0) {
            myNet->getAttributeCarriers()->deleteConnection(connection);
        }
        // delete GNEConnection if is unreferenced
        if (connection->unreferenced()) {
            delete connection;
        }
    }
    // copy retrieved (existent and created) GNECrossings to myGNEConnections
    myGNEConnections = retrievedConnections;
}


void
GNEEdge::clearGNEConnections() {
    // Drop all existents connections that aren't referenced anymore
    for (const auto& connection : myGNEConnections) {
        // check if connection is selected
        if (connection->isAttributeCarrierSelected()) {
            connection->unselectAttributeCarrier();
        }
        // Dec reference of connection
        connection->decRef("GNEEdge::clearGNEConnections");
        // remove it from network
        myNet->removeGLObjectFromGrid(connection);
        // and from AttributeCarriers
        if (myNet->getAttributeCarriers()->getConnections().count(connection) > 0) {
            myNet->getAttributeCarriers()->deleteConnection(connection);
        }
        // Delete GNEConnectionToErase if is unreferenced
        if (connection->unreferenced()) {
            delete connection;
        }
    }
    myGNEConnections.clear();
}


int
GNEEdge::getRouteProbeRelativePosition(GNERouteProbe* routeProbe) const {
    std::vector<GNEAdditional*> routeProbes;
    for (const auto& additional : getChildAdditionals()) {
        if (additional->getTagProperty()->getTag() == routeProbe->getTagProperty()->getTag()) {
            routeProbes.push_back(additional);
        }
    }
    // return index of routeProbe in routeProbes vector
    auto it = std::find(routeProbes.begin(), routeProbes.end(), routeProbe);
    if (it == routeProbes.end()) {
        return -1;
    } else {
        return (int)(it - routeProbes.begin());
    }
}


std::vector<GNECrossing*>
GNEEdge::getGNECrossings() {
    std::vector<GNECrossing*> crossings;
    for (auto i : getFromJunction()->getGNECrossings()) {
        if (i->checkEdgeBelong(this)) {
            crossings.push_back(i);
        }
    }
    for (auto i : getToJunction()->getGNECrossings()) {
        if (i->checkEdgeBelong(this)) {
            crossings.push_back(i);
        }
    }
    return crossings;
}


void
GNEEdge::copyTemplate(const GNEEdgeTemplate* edgeTemplate, GNEUndoList* undoList) {
    // copy edge-specific attributes
    for (const auto& attProperty : myTagProperty->getAttributeProperties()) {
        if (attProperty->isCopyable() && isValid(attProperty->getAttr(), edgeTemplate->getAttribute(attProperty->getAttr()))) {
            setAttribute(attProperty->getAttr(), edgeTemplate->getAttribute(attProperty->getAttr()), undoList);
        }
    }
    // also copy parameters
    setAttribute(GNE_ATTR_PARAMETERS, edgeTemplate->getAttribute(GNE_ATTR_PARAMETERS), undoList);
    // copy lane attributes as well
    for (int i = 0; i < (int)getChildLanes().size(); i++) {
        for (const auto& attProperty : edgeTemplate->getLaneTemplates().at(i)->getTagProperty()->getAttributeProperties()) {
            if (attProperty->isCopyable() && getChildLanes()[i]->isValid(attProperty->getAttr(), edgeTemplate->getLaneTemplates().at(i)->getAttribute(attProperty->getAttr()))) {
                getChildLanes()[i]->setAttribute(attProperty->getAttr(), edgeTemplate->getLaneTemplates().at(i)->getAttribute(attProperty->getAttr()),  undoList);
            }
        }
        // also copy parameters
        getChildLanes()[i]->setAttribute(GNE_ATTR_PARAMETERS, edgeTemplate->getLaneTemplates().at(i)->getAttribute(GNE_ATTR_PARAMETERS), undoList);
    }
}


void
GNEEdge::copyEdgeType(const GNEEdgeType* edgeType, GNEUndoList* undoList) {
    const auto tagPropertiesDatabase = myNet->getTagPropertiesDatabase();
    // get tag properties
    const auto edgeProperties = tagPropertiesDatabase->getTagProperty(SUMO_TAG_EDGE, true);
    const auto laneProperties = tagPropertiesDatabase->getTagProperty(SUMO_TAG_LANE, true);
    const auto edgeTypeProperties = tagPropertiesDatabase->getTagProperty(SUMO_TAG_TYPE, true);
    const auto laneTypeProperties = tagPropertiesDatabase->getTagProperty(SUMO_TAG_LANETYPE, true);
    // set type (only for info)
    setAttribute(SUMO_ATTR_TYPE, edgeType->getAttribute(SUMO_ATTR_ID), undoList);
    // copy attributes
    for (const auto& attrProperty : edgeTypeProperties->getAttributeProperties()) {
        if (attrProperty->isCopyable() && edgeProperties->hasAttribute(attrProperty->getAttr())) {
            setAttribute(attrProperty->getAttr(), edgeType->getAttribute(attrProperty->getAttr()), undoList);
        }
    }
    setAttribute(GNE_ATTR_PARAMETERS, edgeType->getAttribute(GNE_ATTR_PARAMETERS), undoList);
    // copy lane attributes as well
    for (int i = 0; i < (int)getChildLanes().size(); i++) {
        for (const auto& attrProperty : laneTypeProperties->getAttributeProperties()) {
            if (attrProperty->isCopyable() && laneProperties->hasAttribute(attrProperty->getAttr()) &&
                    (edgeType->getLaneTypes().at(i)->getAttribute(attrProperty->getAttr()) != laneTypeProperties->getAttributeProperties(attrProperty->getAttr())->getDefaultStringValue())) {
                getChildLanes()[i]->setAttribute(attrProperty->getAttr(), edgeType->getLaneTypes().at(i)->getAttribute(attrProperty->getAttr()), undoList);
            }
        }
        if (edgeType->getLaneTypes().at(i)->getAttribute(GNE_ATTR_PARAMETERS).size() > 0) {
            getChildLanes()[i]->setAttribute(GNE_ATTR_PARAMETERS, edgeType->getLaneTypes().at(i)->getAttribute(GNE_ATTR_PARAMETERS), undoList);
        }
    }
}


std::set<GUIGlID>
GNEEdge::getLaneGlIDs() const {
    std::set<GUIGlID> result;
    for (const auto glID : getChildLanes()) {
        result.insert(glID->getGlID());
    }
    return result;
}


const std::vector<GNEConnection*>&
GNEEdge::getGNEConnections() const {
    return myGNEConnections;
}


bool
GNEEdge::wasSplit() {
    return myWasSplit;
}


std::string
GNEEdge::getAttribute(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_ID:
            return getMicrosimID();
        case SUMO_ATTR_FROM:
        case SUMO_ATTR_FROM_JUNCTION:
            return getFromJunction()->getID();
        case SUMO_ATTR_TO:
        case SUMO_ATTR_TO_JUNCTION:
            return getToJunction()->getID();
        case SUMO_ATTR_NUMLANES:
            return toString(myNBEdge->getNumLanes());
        case SUMO_ATTR_PRIORITY:
            return toString(myNBEdge->getPriority());
        case SUMO_ATTR_LENGTH:
            return toString(myNBEdge->getFinalLength());
        case SUMO_ATTR_TYPE:
            return myNBEdge->getTypeID();
        case SUMO_ATTR_ROUTINGTYPE:
            return myNBEdge->getRoutingType();
        case SUMO_ATTR_SHAPE:
            return toString(myNBEdge->getInnerGeometry());
        case SUMO_ATTR_SPREADTYPE:
            return SUMOXMLDefinitions::LaneSpreadFunctions.getString(myNBEdge->getLaneSpreadFunction());
        case SUMO_ATTR_NAME:
            return myNBEdge->getStreetName();
        case SUMO_ATTR_ALLOW:
            return (getVehicleClassNames(myNBEdge->getPermissions()) + (myNBEdge->hasLaneSpecificPermissions() ? " (combined!)" : ""));
        case SUMO_ATTR_DISALLOW: {
            return (getVehicleClassNames(invertPermissions(myNBEdge->getPermissions())) + (myNBEdge->hasLaneSpecificPermissions() ? " (combined!)" : ""));
        }
        case SUMO_ATTR_SPEED:
            if (myNBEdge->hasLaneSpecificSpeed()) {
                return "lane specific";
            } else {
                return toString(myNBEdge->getSpeed());
            }
        case SUMO_ATTR_FRICTION:
            if (myNBEdge->hasLaneSpecificFriction()) {
                return "lane specific";
            } else {
                return toString(myNBEdge->getFriction());
            }
        case SUMO_ATTR_WIDTH:
            if (myNBEdge->hasLaneSpecificWidth()) {
                return "lane specific";
            } else if (myNBEdge->getLaneWidth() == NBEdge::UNSPECIFIED_WIDTH) {
                return "default";
            } else {
                return toString(myNBEdge->getLaneWidth());
            }
        case SUMO_ATTR_ENDOFFSET:
            if (myNBEdge->hasLaneSpecificEndOffset()) {
                return "lane specific";
            } else {
                return toString(myNBEdge->getEndOffset());
            }
        case SUMO_ATTR_DISTANCE:
            return toString(myNBEdge->getDistance());
        case GNE_ATTR_MODIFICATION_STATUS:
            return myConnectionStatus;
        case GNE_ATTR_SHAPE_START:
            if (getParentJunctions().empty()) {
                return "";
            } else if (myNBEdge->getGeometry().front().distanceSquaredTo2D(getFromJunction()->getNBNode()->getPosition()) <= ENDPOINT_TOLERANCE) {
                return "";
            } else {
                return toString(myNBEdge->getGeometry().front());
            }
        case GNE_ATTR_SHAPE_END:
            if (getParentJunctions().empty()) {
                return "";
            } else if (myNBEdge->getGeometry().back().distanceSquaredTo2D(getToJunction()->getNBNode()->getPosition()) <= ENDPOINT_TOLERANCE) {
                return "";
            } else {
                return toString(myNBEdge->getGeometry().back());
            }
        case GNE_ATTR_BIDIR:
            return toString(myNBEdge->getBidiEdge() != nullptr);
        case GNE_ATTR_STOPOFFSET:
            return toString(myNBEdge->myEdgeStopOffset.getOffset());
        case GNE_ATTR_STOPOEXCEPTION:
            if (myNBEdge->myEdgeStopOffset.isDefined()) {
                return toString(myNBEdge->myEdgeStopOffset.getExceptions());
            } else {
                return "";
            }
        case GNE_ATTR_IS_ROUNDABOUT:
            return myNBEdge->getFromNode()->isRoundabout() && myNBEdge->getToNode()->isRoundabout() ? TRUE_STR : FALSE_STR;
        default:
            return getCommonAttribute(key);
    }
}


double
GNEEdge::getAttributeDouble(SumoXMLAttr key) const {
    return getCommonAttributeDouble(key);
}


Position
GNEEdge::getAttributePosition(SumoXMLAttr key) const {
    return getCommonAttributePosition(key);
}


PositionVector
GNEEdge::getAttributePositionVector(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_SHAPE:
            return myNBEdge->getInnerGeometry();
        default:
            return getCommonAttributePositionVector(key);
    }
}


std::string
GNEEdge::getAttributeForSelection(SumoXMLAttr key) const {
    std::string result = getAttribute(key);
    if ((key == SUMO_ATTR_ALLOW || key == SUMO_ATTR_DISALLOW) && result.find("all") != std::string::npos) {
        result += " " + getVehicleClassNames(SVCAll, true);
    }
    return result;
}


void
GNEEdge::setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) {
    // get template editor
    GNEInspectorFrame::TemplateEditor* templateEditor = myNet->getViewParent()->getInspectorFrame()->getTemplateEditor();
    // check if we have to update template
    const bool updateTemplate = templateEditor->getEdgeTemplate() ? (templateEditor->getEdgeTemplate()->getID() == getID()) : false;
    switch (key) {
        case SUMO_ATTR_WIDTH:
        case SUMO_ATTR_ENDOFFSET:
        case SUMO_ATTR_SPEED:
        case SUMO_ATTR_FRICTION:
        case SUMO_ATTR_ALLOW:
        case SUMO_ATTR_DISALLOW: {
            undoList->begin(this, "change " + getTagStr() + " attribute");
            const std::string origValue = getChildLanes().at(0)->getAttribute(key); // will have intermediate value of "lane specific"
            // lane specific attributes need to be changed via lanes to allow undo
            for (auto it : getChildLanes()) {
                it->setAttribute(key, value, undoList);
            }
            // ensure that the edge value is also changed. Actually this sets the lane attributes again but it does not matter
            GNEChange_Attribute::changeAttribute(this, key, value, origValue, undoList);
            undoList->end();
            break;
        }
        case SUMO_ATTR_FROM: {
            if (value != getAttribute(key)) {
                undoList->begin(this, "change  " + getTagStr() + "  attribute");
                // Remove edge from crossings of junction source
                removeEdgeFromCrossings(getFromJunction(), undoList);
                // continue changing from junction
                GNEJunction* originalFirstParentJunction = getFromJunction();
                getFromJunction()->setLogicValid(false, undoList);
                GNEChange_Attribute::changeAttribute(this, key, value, undoList);
                getFromJunction()->setLogicValid(false, undoList);
                myNet->getAttributeCarriers()->retrieveJunction(value)->setLogicValid(false, undoList);
                setAttribute(GNE_ATTR_SHAPE_START, toString(getFromJunction()->getNBNode()->getPosition()), undoList);
                getFromJunction()->invalidateShape();
                undoList->end();
                // update geometries of all implicated junctions
                originalFirstParentJunction->updateGeometry();
                getFromJunction()->updateGeometry();
                getToJunction()->updateGeometry();
            }
            break;
        }
        case SUMO_ATTR_TO: {
            if (value != getAttribute(key)) {
                undoList->begin(this, "change  " + getTagStr() + "  attribute");
                // Remove edge from crossings of junction destination
                removeEdgeFromCrossings(getToJunction(), undoList);
                // continue changing destination junction
                GNEJunction* originalSecondParentJunction = getToJunction();
                getToJunction()->setLogicValid(false, undoList);
                GNEChange_Attribute::changeAttribute(this, key, value, undoList);
                getToJunction()->setLogicValid(false, undoList);
                myNet->getAttributeCarriers()->retrieveJunction(value)->setLogicValid(false, undoList);
                setAttribute(GNE_ATTR_SHAPE_END, toString(getToJunction()->getNBNode()->getPosition()), undoList);
                getToJunction()->invalidateShape();
                undoList->end();
                // update geometries of all implicated junctions
                originalSecondParentJunction->updateGeometry();
                getToJunction()->updateGeometry();
                getFromJunction()->updateGeometry();
            }
            break;
        }
        case SUMO_ATTR_ID:
        case SUMO_ATTR_PRIORITY:
        case SUMO_ATTR_LENGTH:
        case SUMO_ATTR_TYPE:
        case SUMO_ATTR_ROUTINGTYPE:
        case SUMO_ATTR_SPREADTYPE:
        case SUMO_ATTR_DISTANCE:
        case GNE_ATTR_MODIFICATION_STATUS:
        case GNE_ATTR_STOPOEXCEPTION:
            GNEChange_Attribute::changeAttribute(this, key, value, undoList);
            break;
        case GNE_ATTR_STOPOFFSET:
            // special case for stop offset, because affects to stopOffsetExceptions (#15297)
            if (canParse<double>(value) && (parse<double>(value) == 0)) {
                GNEChange_Attribute::changeAttribute(this, GNE_ATTR_STOPOEXCEPTION, "", undoList);
            }
            GNEChange_Attribute::changeAttribute(this, key, value, undoList);
            break;
        case GNE_ATTR_SHAPE_START:
        case GNE_ATTR_SHAPE_END: {
            // due to ENDPOINT_TOLERANCE, force change
            GNEChange_Attribute::changeAttribute(this, key, value, undoList, true);
            break;
        }
        case SUMO_ATTR_NAME:
            // user cares about street names. Make sure they appear in the output
            OptionsCont::getOptions().resetWritable();
            OptionsCont::getOptions().set("output.street-names", "true");
            GNEChange_Attribute::changeAttribute(this, key, value, undoList);
            break;
        case SUMO_ATTR_NUMLANES:
            if (value != getAttribute(key)) {
                // set num lanes
                setNumLanes(parse<int>(value), undoList);
            }
            break;
        case GNE_ATTR_BIDIR:
            undoList->begin(this, "change  " + getTagStr() + "  attribute");
            GNEChange_Attribute::changeAttribute(this, key, value, undoList);
            if (myNBEdge->getTurnDestination(true) != nullptr) {
                GNEEdge* bidi = myNet->getAttributeCarriers()->retrieveEdge(myNBEdge->getTurnDestination(true)->getID());
                GNEChange_Attribute::changeAttribute(bidi, key, value, undoList);
                if (myNBEdge->getGeometry() != bidi->getNBEdge()->getGeometry().reverse()
                        && myNBEdge->getGeometry().size() == 2
                        && bidi->getNBEdge()->getGeometry().size() == 2
                        && myNBEdge->getBidiEdge() == nullptr) {
                    // NBEdge::avoidOverlap was already active so we need to reset the
                    // geometry to its default
                    resetBothEndpoint(undoList);
                    bidi->resetBothEndpoint(undoList);
                }
            }
            undoList->end();
            break;
        case SUMO_ATTR_SHAPE:
            // @note: assumes value of inner geometry!
            // actually the geometry is already updated (incrementally
            // during mouse movement). We set the restore point to the end
            // of the last change-set
            GNEChange_Attribute::changeAttribute(this, key, value, undoList);
            break;
        default:
            setCommonAttribute(key, value, undoList);
            break;
    }
    // update template
    if (updateTemplate) {
        templateEditor->setEdgeTemplate(this);
    }
}


bool
GNEEdge::isValid(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            return SUMOXMLDefinitions::isValidNetID(value) && (myNet->getAttributeCarriers()->retrieveEdge(value, false) == nullptr);
        case SUMO_ATTR_FROM: {
            // check that is a valid ID and is different of ID of junction destination
            if (value == getFromJunction()->getID()) {
                return true;
            } else if (SUMOXMLDefinitions::isValidNetID(value) && (value != getToJunction()->getID())) {
                return (myNet->getAttributeCarriers()->retrieveJunction(value, false) != nullptr);
            } else {
                return false;
            }
        }
        case SUMO_ATTR_TO: {
            // check that is a valid ID and is different of ID of junction Source
            if (value == getToJunction()->getID()) {
                return true;
            } else if (SUMOXMLDefinitions::isValidNetID(value) && (value != getFromJunction()->getID())) {
                return (myNet->getAttributeCarriers()->retrieveJunction(value, false) != nullptr);
            } else {
                return false;
            }
        }
        case SUMO_ATTR_SPEED:
            return canParse<double>(value) && (parse<double>(value) > 0);
        case SUMO_ATTR_FRICTION:
            return canParse<double>(value) && (parse<double>(value) > 0);
        case SUMO_ATTR_NUMLANES:
            return canParse<int>(value) && (parse<double>(value) > 0);
        case SUMO_ATTR_PRIORITY:
            return canParse<int>(value);
        case SUMO_ATTR_LENGTH:
            if (value.empty()) {
                return true;
            } else {
                return canParse<double>(value) && ((parse<double>(value) > 0) || (parse<double>(value) == NBEdge::UNSPECIFIED_LOADED_LENGTH));
            }
        case SUMO_ATTR_ALLOW:
        case SUMO_ATTR_DISALLOW:
            return canParseVehicleClasses(value);
        case SUMO_ATTR_TYPE:
        case SUMO_ATTR_ROUTINGTYPE:
            return true;
        case SUMO_ATTR_SHAPE:
            // empty shapes are allowed
            return canParse<PositionVector>(value);
        case SUMO_ATTR_SPREADTYPE:
            return SUMOXMLDefinitions::LaneSpreadFunctions.hasString(value);
        case SUMO_ATTR_NAME:
            return true;
        case SUMO_ATTR_WIDTH:
            if (value.empty() || (value == "default")) {
                return true;
            } else {
                return canParse<double>(value) && ((parse<double>(value) >= -1) || (parse<double>(value) == NBEdge::UNSPECIFIED_WIDTH));
            }
        case SUMO_ATTR_ENDOFFSET:
            return canParse<double>(value) && parse<double>(value) >= 0 && parse<double>(value) < myNBEdge->getLoadedLength();
        case SUMO_ATTR_DISTANCE:
            if (value.empty()) {
                return true;
            } else {
                return canParse<double>(value);
            }
        case GNE_ATTR_SHAPE_START: {
            if (value.empty()) {
                return true;
            } else if (canParse<Position>(value)) {
                Position shapeStart = parse<Position>(value);
                return (shapeStart != myNBEdge->getGeometry()[-1]);
            } else {
                return false;
            }
        }
        case GNE_ATTR_SHAPE_END: {
            if (value.empty()) {
                return true;
            } else if (canParse<Position>(value)) {
                Position shapeEnd = parse<Position>(value);
                return (shapeEnd != myNBEdge->getGeometry()[0]);
            } else {
                return false;
            }
        }
        case GNE_ATTR_BIDIR:
            return canParse<bool>(value) && (!parse<bool>(value) || myNBEdge->isBidiEdge(true));
        case GNE_ATTR_STOPOFFSET:
            if (value.empty()) {
                return true;
            } else {
                return canParse<double>(value) && (parse<double>(value) >= 0);
            }
        case GNE_ATTR_STOPOEXCEPTION:
            return canParseVehicleClasses(value);
        default:
            return isCommonAttributeValid(key, value);
    }
}


bool
GNEEdge::isAttributeEnabled(SumoXMLAttr key) const {
    switch (key) {
        case GNE_ATTR_BIDIR:
            return myNBEdge->isBidiEdge(true);
        case GNE_ATTR_STOPOEXCEPTION:
            return myNBEdge->myEdgeStopOffset.getOffset() > 0;
        case GNE_ATTR_IS_ROUNDABOUT:
            return false;
        default:
            return true;
    }
}


bool
GNEEdge::isAttributeComputed(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_LENGTH:
            return !myNBEdge->hasLoadedLength();
        case SUMO_ATTR_WIDTH:
            if (myNBEdge->hasLaneSpecificWidth()) {
                return false;
            } else {
                return (myNBEdge->getLaneWidth() == NBEdge::UNSPECIFIED_WIDTH);
            }
        default:
            return false;
    }
}


void
GNEEdge::setResponsible(bool newVal) {
    myAmResponsible = newVal;
}


GNELane*
GNEEdge::getLaneByAllowedVClass(const SUMOVehicleClass vClass) const {
    // iterate over all NBEdge lanes
    for (int i = 0; i < (int)myNBEdge->getLanes().size(); i++) {
        // if given VClass is in permissions, return lane
        if (myNBEdge->getLanes().at(i).permissions & vClass) {
            // return GNELane
            return getChildLanes().at(i);
        }
    }
    // return first lane
    return getChildLanes().front();
}


GNELane*
GNEEdge::getLaneByDisallowedVClass(const SUMOVehicleClass vClass) const {
    // iterate over all NBEdge lanes
    for (int i = 0; i < (int)myNBEdge->getLanes().size(); i++) {
        // if given VClass isn't in permissions, return lane
        if (~(myNBEdge->getLanes().at(i).permissions) & vClass) {
            // return GNELane
            return getChildLanes().at(i);
        }
    }
    // return first lane
    return getChildLanes().front();
}


void
GNEEdge::updateVehicleSpreadGeometries() {
    // get lane vehicles map
    const std::map<const GNELane*, std::vector<GNEDemandElement*> > laneVehiclesMap = getVehiclesOverEdgeMap();
    // iterate over every lane
    for (const auto& laneVehicle : laneVehiclesMap) {
        // obtain total length
        double totalLength = 0;
        for (const auto& vehicle : laneVehicle.second) {
            totalLength += vehicle->getAttributeDouble(SUMO_ATTR_LENGTH) + VEHICLE_GAP;
        }
        // calculate multiplier for vehicle positions
        double multiplier = 1;
        const double laneShapeLength = laneVehicle.first->getLaneShape().length();
        if (laneShapeLength == 0) {
            multiplier = 0;
        } else if (totalLength > laneShapeLength) {
            multiplier = (laneShapeLength / totalLength);
        }
        // declare current length
        double length = 0;
        // iterate over vehicles to calculate position and rotations
        for (const auto& vehicle : laneVehicle.second) {
            vehicle->updateDemandElementSpreadGeometry(laneVehicle.first, length * multiplier);
            // update length
            length += vehicle->getAttributeDouble(SUMO_ATTR_LENGTH) + VEHICLE_GAP;
        }
    }
}


void
GNEEdge::updateVehicleStackLabels() {
    // get lane vehicles map
    const std::map<const GNELane*, std::vector<GNEDemandElement*> > laneVehiclesMap = getVehiclesOverEdgeMap();
    // iterate over laneVehiclesMap and obtain a vector with
    for (const auto& vehicleMap : laneVehiclesMap) {
        // declare map for sort vehicles using their departpos+length position (StackPosition)
        std::vector<std::pair<GNEEdge::StackPosition, GNEDemandElement*> > departPosVehicles;
        // declare vector of stack demand elements
        std::vector<GNEEdge::StackDemandElements> stackedVehicles;
        // iterate over vehicles
        for (const auto& vehicle : vehicleMap.second) {
            // get vehicle's depart pos and length
            const double departPos = vehicle->getAttributeDouble(SUMO_ATTR_DEPARTPOS);
            const double length = vehicle->getAttributeDouble(SUMO_ATTR_LENGTH);
            double posOverLane = vehicle->getAttributeDouble(SUMO_ATTR_DEPARTPOS);
            // check if we have to adapt posOverLane
            if (posOverLane < 0) {
                posOverLane += vehicleMap.first->getLaneShape().length();
            }
            // make a stack position using departPos and length
            departPosVehicles.push_back(std::make_pair(StackPosition(departPos, length), vehicle));
            // update depart element geometry
            vehicle->updateDemandElementGeometry(vehicleMap.first, posOverLane);
            // reset vehicle stack label
            vehicle->updateDemandElementStackLabel(0);
        }

        // sort departPosVehicles
        std::sort(departPosVehicles.begin(), departPosVehicles.end());
        // iterate over departPosVehicles
        for (const auto& departPosVehicle : departPosVehicles) {
            // obtain stack position and vehicle
            const GNEEdge::StackPosition& vehicleStackPosition = departPosVehicle.first;
            GNEDemandElement* vehicle = departPosVehicle.second;
            // if stackedVehicles is empty, add a new StackDemandElements
            if (stackedVehicles.empty()) {
                stackedVehicles.push_back(GNEEdge::StackDemandElements(vehicleStackPosition, vehicle));
            } else if (areStackPositionOverlapped(vehicleStackPosition, stackedVehicles.back().getStackPosition())) {
                // add new vehicle to last inserted stackDemandElements
                stackedVehicles[stackedVehicles.size() - 1].addDemandElements(vehicle);
            } else {
                // No overlapping, then add a new StackDemandElements
                stackedVehicles.push_back(GNEEdge::StackDemandElements(vehicleStackPosition, vehicle));
            }
        }
        // iterate over stackedVehicles
        for (const auto& vehicle : stackedVehicles) {
            // only update vehicles with one or more stack
            if (vehicle.getDemandElements().size() > 1) {
                // set stack labels
                vehicle.getDemandElements().front()->updateDemandElementStackLabel((int)vehicle.getDemandElements().size());
            }
        }
    }
}


void
GNEEdge::updatePersonStackLabels() {
    // get lane persons map
    const std::map<const GNELane*, std::vector<GNEDemandElement*> > lanePersonsMap = getPersonsOverEdgeMap();
    // iterate over lanePersonsMap and obtain a vector with
    for (const auto& personMap : lanePersonsMap) {
        // declare map for sort persons using their departpos+length position (StackPosition)
        std::vector<std::pair<GNEEdge::StackPosition, GNEDemandElement*> > departPosPersons;
        // declare vector of stack demand elements
        std::vector<GNEEdge::StackDemandElements> stackedPersons;
        // iterate over persons
        for (const auto& person : personMap.second) {
            // get person's depart pos and length
            const double departPos = person->getAttributeDouble(SUMO_ATTR_DEPARTPOS);
            // make a stack position using departPos and length
            departPosPersons.push_back(std::make_pair(StackPosition(departPos, 1.8), person));
            // update depart element geometry
            person->updateDemandElementGeometry(personMap.first, departPos);
            // reset person stack label
            person->updateDemandElementStackLabel(0);
        }

        // sort departPosPersons
        std::sort(departPosPersons.begin(), departPosPersons.end());
        // iterate over departPosPersons
        for (const auto& departPosPerson : departPosPersons) {
            // obtain stack position and person
            const GNEEdge::StackPosition& personStackPosition = departPosPerson.first;
            GNEDemandElement* person = departPosPerson.second;
            // if stackedPersons is empty, add a new StackDemandElements
            if (stackedPersons.empty()) {
                stackedPersons.push_back(GNEEdge::StackDemandElements(personStackPosition, person));
            } else if (areStackPositionOverlapped(personStackPosition, stackedPersons.back().getStackPosition())) {
                // add new person to last inserted stackDemandElements
                stackedPersons[stackedPersons.size() - 1].addDemandElements(person);
            } else {
                // No overlapping, then add a new StackDemandElements
                stackedPersons.push_back(GNEEdge::StackDemandElements(personStackPosition, person));
            }
        }
        // iterate over stackedPersons
        for (const auto& person : stackedPersons) {
            // only update persons with one or more stack
            if (person.getDemandElements().size() > 1) {
                // set stack labels
                person.getDemandElements().front()->updateDemandElementStackLabel((int)person.getDemandElements().size());
            }
        }
    }
}


void
GNEEdge::updateContainerStackLabels() {
    // get lane containers map
    const std::map<const GNELane*, std::vector<GNEDemandElement*> > laneContainersMap = getContainersOverEdgeMap();
    // iterate over laneContainersMap and obtain a vector with
    for (const auto& containerMap : laneContainersMap) {
        // declare map for sort containers using their departpos+length position (StackPosition)
        std::vector<std::pair<GNEEdge::StackPosition, GNEDemandElement*> > departPosContainers;
        // declare vector of stack demand elements
        std::vector<GNEEdge::StackDemandElements> stackedContainers;
        // iterate over containers
        for (const auto& container : containerMap.second) {
            // get container's depart pos and length
            const double departPos = container->getAttributeDouble(SUMO_ATTR_DEPARTPOS);
            // make a stack position using departPos and length
            departPosContainers.push_back(std::make_pair(StackPosition(departPos, 1.8), container));
            // update depart element geometry
            container->updateDemandElementGeometry(containerMap.first, departPos);
            // reset container stack label
            container->updateDemandElementStackLabel(0);
        }

        // sort departPosContainers
        std::sort(departPosContainers.begin(), departPosContainers.end());
        // iterate over departPosContainers
        for (const auto& departPosContainer : departPosContainers) {
            // obtain stack position and container
            const GNEEdge::StackPosition& containerStackPosition = departPosContainer.first;
            GNEDemandElement* container = departPosContainer.second;
            // if stackedContainers is empty, add a new StackDemandElements
            if (stackedContainers.empty()) {
                stackedContainers.push_back(GNEEdge::StackDemandElements(containerStackPosition, container));
            } else if (areStackPositionOverlapped(containerStackPosition, stackedContainers.back().getStackPosition())) {
                // add new container to last inserted stackDemandElements
                stackedContainers[stackedContainers.size() - 1].addDemandElements(container);
            } else {
                // No overlapping, then add a new StackDemandElements
                stackedContainers.push_back(GNEEdge::StackDemandElements(containerStackPosition, container));
            }
        }
        // iterate over stackedContainers
        for (const auto& container : stackedContainers) {
            // only update containers with one or more stack
            if (container.getDemandElements().size() > 1) {
                // set stack labels
                container.getDemandElements().front()->updateDemandElementStackLabel((int)container.getDemandElements().size());
            }
        }
    }
}


bool
GNEEdge::isConvexAngle() const {
    // calculate angle between both junction positions
    double edgeAngle = RAD2DEG(getFromJunction()->getPositionInView().angleTo2D(getToJunction()->getPositionInView()));
    // adjust to 360 degrees
    while (edgeAngle < 0) {
        edgeAngle += 360;
    }
    // fmod round towards zero which is not want we want for negative numbers
    edgeAngle = fmod(edgeAngle, 360);
    // check angle
    return edgeAngle >= 0 && edgeAngle < 180;
}


bool
GNEEdge::hasPredecessors() const {
    // get incoming edges
    const auto incomingEdges = getFromJunction()->getGNEIncomingEdges();
    // iterate over connections
    for (const auto& incomingEdge : incomingEdges) {
        for (const auto& connection : incomingEdge->getGNEConnections()) {
            if (connection->getEdgeTo() == this) {
                return true;
            }
        }
    }
    return false;
}


bool
GNEEdge::hasSuccessors() const {
    return (myGNEConnections.size() > 0);
}


GNEEdge*
GNEEdge::getReverseEdge() const {
    for (const auto& outgoingEdge : getParentJunctions().back()->getGNEOutgoingEdges()) {
        if (outgoingEdge->getToJunction() == getFromJunction()) {
            return outgoingEdge;
        }
    }
    return nullptr;
}

// ===========================================================================
// private
// ===========================================================================

GNEEdge::StackPosition::StackPosition(const double departPos, const double length) :
    pair(departPos, departPos + length) {
}


double
GNEEdge::StackPosition::beginPosition() const {
    return first;
}


double
GNEEdge::StackPosition::endPosition() const {
    return second;
}


GNEEdge::StackDemandElements::StackDemandElements(const StackPosition stackedPosition, GNEDemandElement* demandElement) :
    pair(stackedPosition, {
    demandElement
}) {
}


void
GNEEdge::StackDemandElements::addDemandElements(GNEDemandElement* demandElement) {
    second.push_back(demandElement);
}


const GNEEdge::StackPosition&
GNEEdge::StackDemandElements::getStackPosition() const {
    return first;
}


const std::vector<GNEDemandElement*>&
GNEEdge::StackDemandElements::getDemandElements() const {
    return second;
}


void
GNEEdge::setAttribute(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            myNet->getAttributeCarriers()->updateEdgeID(this, value);
            // enable save demand elements if there are stops
            for (const auto& stop : getChildDemandElements()) {
                if (stop->getTagProperty()->isVehicleStop()) {
                    myNet->getSavingStatus()->requireSaveDemandElements();
                }
            }
            // also for lanes
            for (const auto& lane : getChildLanes()) {
                for (const auto& stop : lane->getChildDemandElements()) {
                    if (stop->getTagProperty()->isVehicleStop()) {
                        myNet->getSavingStatus()->requireSaveDemandElements();
                    }
                }
            }
            break;
        case SUMO_ATTR_FROM:
            myNet->changeEdgeEndpoints(this, value, getToJunction()->getID());
            // update this edge of list of outgoings edges of the old first parent junction
            getFromJunction()->removeOutgoingGNEEdge(this);
            // update first parent junction
            updateFirstParentJunction(value);
            // update this edge of list of outgoings edges of the new first parent junction
            getFromJunction()->addOutgoingGNEEdge(this);
            // update centering boundary and grid
            updateCenteringBoundary(true);
            break;
        case SUMO_ATTR_TO:
            myNet->changeEdgeEndpoints(this, getFromJunction()->getID(), value);
            // update this edge of list of incomings edges of the old second parent junction
            getToJunction()->removeIncomingGNEEdge(this);
            // update second parent junction
            updateSecondParentJunction(value);
            // update this edge of list of incomings edges of the new second parent junction
            getToJunction()->addIncomingGNEEdge(this);
            // update centering boundary and grid
            updateCenteringBoundary(true);
            break;
        case SUMO_ATTR_NUMLANES:
            throw InvalidArgument("GNEEdge::setAttribute (private) called for attr SUMO_ATTR_NUMLANES. This should never happen");
        case SUMO_ATTR_PRIORITY:
            myNBEdge->myPriority = parse<int>(value);
            break;
        case SUMO_ATTR_LENGTH:
            if (value.empty()) {
                myNBEdge->setLoadedLength(NBEdge::UNSPECIFIED_LOADED_LENGTH);
            } else {
                myNBEdge->setLoadedLength(parse<double>(value));
            }
            break;
        case SUMO_ATTR_TYPE:
            myNBEdge->myType = value;
            break;
        case SUMO_ATTR_ROUTINGTYPE:
            myNBEdge->myRoutingType = value;
            break;
        case SUMO_ATTR_SHAPE:
            // set new geometry
            setGeometry(parse<PositionVector>(value), true);
            // update centering boundary and grid
            updateCenteringBoundary(true);
            break;
        case SUMO_ATTR_SPREADTYPE:
            myNBEdge->setLaneSpreadFunction(SUMOXMLDefinitions::LaneSpreadFunctions.get(value));
            break;
        case SUMO_ATTR_NAME:
            myNBEdge->setStreetName(value);
            break;
        case SUMO_ATTR_SPEED:
            myNBEdge->setSpeed(-1, parse<double>(value));
            break;
        case SUMO_ATTR_FRICTION:
            myNBEdge->setFriction(-1, parse<double>(value));
            break;
        case SUMO_ATTR_WIDTH:
            if (value.empty() || (value == "default")) {
                myNBEdge->setLaneWidth(-1, NBEdge::UNSPECIFIED_WIDTH);
            } else {
                myNBEdge->setLaneWidth(-1, parse<double>(value));
            }
            break;
        case SUMO_ATTR_ENDOFFSET:
            myNBEdge->setEndOffset(-1, parse<double>(value));
            break;
        case SUMO_ATTR_ALLOW:
            break;  // no edge value
        case SUMO_ATTR_DISALLOW:
            break; // no edge value
        case SUMO_ATTR_DISTANCE:
            if (value.empty()) {
                myNBEdge->setDistance(0.0);
            } else {
                myNBEdge->setDistance(parse<double>(value));
            }
            break;
        case GNE_ATTR_MODIFICATION_STATUS:
            myConnectionStatus = value;
            if (value == FEATURE_GUESSED) {
                myNBEdge->invalidateConnections(true);
                clearGNEConnections();
            } else if (value != FEATURE_GUESSED) {
                myNBEdge->declareConnectionsAsLoaded();
            }
            break;
        case GNE_ATTR_SHAPE_START: {
            // get geometry of NBEdge, remove FIRST element with the new value (or with the Junction Source position) and set it back to edge
            Position newShapeStart;
            if (value == "") {
                newShapeStart = getFromJunction()->getNBNode()->getPosition();
            } else {
                newShapeStart = parse<Position>(value);
            }
            // set shape start position
            setShapeStartPos(newShapeStart);
            // update centering boundary and grid
            updateCenteringBoundary(true);
            break;
        }
        case GNE_ATTR_SHAPE_END: {
            // get geometry of NBEdge, remove LAST element with the new value (or with the Junction Destination position) and set it back to edge
            Position newShapeEnd;
            if (value == "") {
                newShapeEnd = getToJunction()->getNBNode()->getPosition();
            } else {
                newShapeEnd = parse<Position>(value);
            }
            // set shape end position
            setShapeEndPos(newShapeEnd);
            // update centering boundary and grid
            updateCenteringBoundary(true);
            break;
        }
        case GNE_ATTR_BIDIR:
            myNBEdge->setBidi(parse<bool>(value));
            break;
        case GNE_ATTR_STOPOFFSET:
            if (value.empty()) {
                myNBEdge->myEdgeStopOffset.setOffset(0);
            } else {
                myNBEdge->myEdgeStopOffset.setOffset(parse<double>(value));
            }
            break;
        case GNE_ATTR_STOPOEXCEPTION:
            myNBEdge->myEdgeStopOffset.setExceptions(value);
            break;
        default:
            setCommonAttribute(key, value);
            break;
    }
    // get template editor
    GNEInspectorFrame::TemplateEditor* templateEditor = myNet->getViewParent()->getInspectorFrame()->getTemplateEditor();
    // check if update template (except for modification status)
    if (templateEditor->getEdgeTemplate() && (templateEditor->getEdgeTemplate()->getID() == getID()) &&
            (key != GNE_ATTR_MODIFICATION_STATUS)) {
        myNet->getViewParent()->getInspectorFrame()->getTemplateEditor()->updateEdgeTemplate();
    }
    // invalidate demand path calculator
    myNet->getDemandPathManager()->getPathCalculator()->invalidatePathCalculator();
}


void
GNEEdge::setNumLanes(int numLanes, GNEUndoList* undoList) {
    // begin undo list
    undoList->begin(this, "change number of " + toString(SUMO_TAG_LANE) +  "s");
    // invalidate logic of source/destination edges
    getFromJunction()->setLogicValid(false, undoList);
    getToJunction()->setLogicValid(false, undoList);
    // disable update geometry (see #6336)
    myUpdateGeometry = false;
    // remove edge from grid
    myNet->removeGLObjectFromGrid(this);
    // save old number of lanes
    const int oldNumLanes = (int)getChildLanes().size();
    // get opposite ID
    const auto oppositeID = getChildLanes().back()->getAttribute(GNE_ATTR_OPPOSITE);
    if (oppositeID != "") {
        // we'll have a different leftmost lane after adding/removing lanes
        GNEChange_Attribute::changeAttribute(getChildLanes().back(), GNE_ATTR_OPPOSITE, "", undoList);
    }
    for (int i = oldNumLanes; i < numLanes; i++) {
        // since the GNELane does not exist yet, it cannot have yet been referenced so we only pass a zero-pointer
        undoList->add(new GNEChange_Lane(this, myNBEdge->getLaneStruct(oldNumLanes - 1)), true);
    }
    for (int i = (oldNumLanes - 1); i > (numLanes - 1); i--) {
        myNet->deleteLane(getChildLanes().at(i), undoList, false);
    }
    if (oppositeID != "") {
        GNEChange_Attribute::changeAttribute(getChildLanes().back(), GNE_ATTR_OPPOSITE, oppositeID, undoList);
    }
    // enable updateGeometry again
    myUpdateGeometry = true;
    // update geometry of entire edge
    updateGeometry();
    // end undo list
    undoList->end();
    // update centering boundary (without updating RTREE)
    updateCenteringBoundary(false);
    // insert edge in grid again
    myNet->addGLObjectIntoGrid(this);
}


void
GNEEdge::updateFirstParentJunction(const std::string& value) {
    auto newJunction = myNet->getAttributeCarriers()->retrieveJunction(value);
    GNEHierarchicalElement::updateParent(this, 0, newJunction);
}


void
GNEEdge::updateSecondParentJunction(const std::string& value) {
    auto newJunction = myNet->getAttributeCarriers()->retrieveJunction(value);
    GNEHierarchicalElement::updateParent(this, 1, newJunction);
}


void
GNEEdge::addLane(GNELane* lane, const NBEdge::Lane& laneAttrs, bool recomputeConnections) {
    const int index = lane ? lane->getIndex() : myNBEdge->getNumLanes();
    // the laneStruct must be created first to ensure we have some geometry
    // unless the connections are fully recomputed, existing indices must be shifted
    myNBEdge->addLane(index, true, recomputeConnections, !recomputeConnections);
    if (lane) {
        // restore a previously deleted lane
        GNEHierarchicalContainerChildren<GNELane*> newParentLanes = getChildLanes();
        newParentLanes.insert(newParentLanes.begin() + index, lane);
        while (getChildLanes().size() > 0) {
            removeChild(this, getChildLanes().front());
        }
        for (const auto newParentLane : newParentLanes) {
            insertChild(this, newParentLane);
        }
    } else {
        // create a new lane by copying leftmost lane
        lane = new GNELane(this, index);
        getHierarchicalElement()->addChildElement(lane);
    }
    lane->incRef("GNEEdge::addLane");
    // add in attributeCarriers
    myNet->getAttributeCarriers()->insertLane(lane);
    // check if lane is selected
    if (lane->isAttributeCarrierSelected()) {
        lane->selectAttributeCarrier();
    }
    // we copy all attributes except shape since this is recomputed from edge shape
    myNBEdge->setSpeed(lane->getIndex(), laneAttrs.speed);
    myNBEdge->setFriction(lane->getIndex(), laneAttrs.friction);
    myNBEdge->setPermissions(laneAttrs.permissions, lane->getIndex());
    myNBEdge->setPreferredVehicleClass(laneAttrs.preferred, lane->getIndex());
    myNBEdge->setEndOffset(lane->getIndex(), laneAttrs.endOffset);
    myNBEdge->setLaneWidth(lane->getIndex(), laneAttrs.width);
    // update indices
    for (int i = 0; i < (int)getChildLanes().size(); ++i) {
        getChildLanes()[i]->setIndex(i);
    }
    /* while technically correct, this looks ugly
    getFromJunction()->invalidateShape();
    getToJunction()->invalidateShape();
    */
    // Remake connections for this edge and all edges that target this lane
    remakeGNEConnections();
    // remake connections of all edges of junction source and destination
    for (const auto& fromEdge : getFromJunction()->getChildEdges()) {
        fromEdge->remakeGNEConnections();
    }
    // remake connections of all edges of junction source and destination
    for (const auto& toEdge : getToJunction()->getChildEdges()) {
        toEdge->remakeGNEConnections();
    }
    // Update geometry with the new lane
    updateGeometry();
    // update boundary and grid
    updateCenteringBoundary(myUpdateGeometry);
}


void
GNEEdge::removeLane(GNELane* lane, bool recomputeConnections) {
    if (getChildLanes().size() == 0) {
        throw ProcessError("Should not remove the last " + toString(SUMO_TAG_LANE) + " from an " + getTagStr());
    }
    if (lane == nullptr) {
        lane = getChildLanes().back();
    }
    // check if lane is selected
    if (lane->isAttributeCarrierSelected()) {
        lane->unselectAttributeCarrier();
    }
    // before removing, check that lane isn't being inspected
    myNet->getViewNet()->getInspectedElements().uninspectAC(lane);
    myNet->getViewParent()->getInspectorFrame()->getHierarchicalElementTree()->removeCurrentEditedAttributeCarrier(lane);
    // Delete lane of edge's container
    // unless the connections are fully recomputed, existing indices must be shifted
    myNBEdge->deleteLane(lane->getIndex(), recomputeConnections, !recomputeConnections);
    lane->decRef("GNEEdge::removeLane");
    // delete lane from edge
    GNEHierarchicalElement::removeChild(this, lane);
    // remove from attributeCarriers
    myNet->getAttributeCarriers()->deleteLane(lane);
    // Delete lane if is unreferenced
    if (lane->unreferenced()) {
        delete lane;
    }
    // update indices
    for (int i = 0; i < (int)getChildLanes().size(); ++i) {
        getChildLanes()[i]->setIndex(i);
    }
    /* while technically correct, this looks ugly
    getFromJunction()->invalidateShape();
    getToJunction()->invalidateShape();
    */
    // Remake connections of this edge
    remakeGNEConnections();
    // remake connections of all edges of junction source and destination
    for (const auto& fromEdge : getFromJunction()->getChildEdges()) {
        fromEdge->remakeGNEConnections();
    }
    // remake connections of all edges of junction source and destination
    for (const auto& toEdge : getToJunction()->getChildEdges()) {
        toEdge->remakeGNEConnections();
    }
    // Update element
    updateGeometry();
    // update boundary and grid
    updateCenteringBoundary(myUpdateGeometry);
}


void
GNEEdge::addConnection(NBEdge::Connection nbCon, bool selectAfterCreation) {
    // If a new connection was successfully created
    if (myNBEdge->setConnection(nbCon.fromLane, nbCon.toEdge, nbCon.toLane, NBEdge::Lane2LaneInfoType::USER, true, nbCon.mayDefinitelyPass,
                                nbCon.keepClear, nbCon.contPos, nbCon.visibility,
                                nbCon.speed, nbCon.friction, nbCon.customLength, nbCon.customShape, nbCon.uncontrolled)) {
        // Create or retrieve existent GNEConnection
        GNEConnection* con = retrieveGNEConnection(nbCon.fromLane, nbCon.toEdge, nbCon.toLane);
        // add it to GNEConnection container
        myGNEConnections.push_back(con);
        // Add reference
        myGNEConnections.back()->incRef("GNEEdge::addConnection");
        // select GNEConnection if needed
        if (selectAfterCreation) {
            con->selectAttributeCarrier();
        }
        // update geometry
        con->updateGeometry();
    }
    // actually we only do this to force a redraw
    updateGeometry();
}


void
GNEEdge::removeConnection(NBEdge::Connection nbCon) {
    // check if is a explicit turnaround
    if (nbCon.toEdge == myNBEdge->getTurnDestination()) {
        myNet->removeExplicitTurnaround(getID());
    }
    // remove NBEdge::connection from NBEdge
    myNBEdge->removeFromConnections(nbCon);
    // remove their associated GNEConnection
    GNEConnection* connection = retrieveGNEConnection(nbCon.fromLane, nbCon.toEdge, nbCon.toLane, false);
    if (connection != nullptr) {
        // before removing, check that the connection isn't being inspected
        myNet->getViewNet()->getInspectedElements().uninspectAC(connection);
        myNet->getViewParent()->getInspectorFrame()->getHierarchicalElementTree()->removeCurrentEditedAttributeCarrier(connection);
        connection->decRef("GNEEdge::removeConnection");
        myGNEConnections.erase(std::find(myGNEConnections.begin(), myGNEConnections.end(), connection));
        // check if connection is selected
        if (connection->isAttributeCarrierSelected()) {
            connection->unselectAttributeCarrier();
        }
        // remove it from network
        myNet->removeGLObjectFromGrid(connection);
        // check if remove it from Attribute Carriers
        if (myNet->getAttributeCarriers()->getConnections().count(connection) > 0) {
            myNet->getAttributeCarriers()->deleteConnection(connection);
        }
        if (connection->unreferenced()) {
            // actually we only do this to force a redraw
            updateGeometry();
        }
    }
}


GNEConnection*
GNEEdge::retrieveGNEConnection(int fromLane, NBEdge* to, int toLane, bool createIfNoExist) {
    for (const auto& connection : myGNEConnections) {
        if ((connection->getFromLaneIndex() == fromLane) && (connection->getEdgeTo()->getNBEdge() == to) && (connection->getToLaneIndex() == toLane)) {
            return connection;
        }
    }
    if (createIfNoExist) {
        // create new connection. Will be added to the rTree on first geometry computation
        GNEConnection* connection = new GNEConnection(getChildLanes()[fromLane], myNet->getAttributeCarriers()->retrieveEdge(to->getID())->getChildLanes()[toLane]);
        // add it into network
        myNet->addGLObjectIntoGrid(connection);
        // add it in attributeCarriers
        myNet->getAttributeCarriers()->insertConnection(connection);
        return connection;
    } else {
        return nullptr;
    }
}


void
GNEEdge::setEdgeID(const std::string& newID) {
    setNetworkElementID(newID);
    for (const auto& lane : getChildLanes()) {
        lane->setNetworkElementID(getNBEdge()->getLaneID(lane->getIndex()));
    }
}


bool
GNEEdge::hasRestrictedLane(SUMOVehicleClass vclass) const {
    for (const auto& lane : getChildLanes()) {
        if (lane->isRestricted(vclass)) {
            return true;
        }
    }
    return false;
}


void
GNEEdge::removeEdgeFromCrossings(GNEJunction* junction, GNEUndoList* undoList) {
    // Remove all crossings that contain this edge in parameter "edges"
    for (const auto& crossing : junction->getGNECrossings()) {
        if (crossing->checkEdgeBelong(this)) {
            myNet->deleteCrossing(crossing, undoList);
        }
    }
}


void
GNEEdge::straightenElevation(GNEUndoList* undoList) {
    PositionVector modifiedShape = myNBEdge->getGeometry().interpolateZ(
                                       myNBEdge->getFromNode()->getPosition().z(),
                                       myNBEdge->getToNode()->getPosition().z());
    PositionVector innerShape(modifiedShape.begin() + 1, modifiedShape.end() - 1);
    setAttribute(SUMO_ATTR_SHAPE, toString(innerShape), undoList);
}


PositionVector
GNEEdge::smoothShape(const PositionVector& old, bool forElevation) {
    const auto& neteditOptions = OptionsCont::getOptions();
    // distinguish 3 cases:
    // a) if the edge has exactly 3 or 4 points, use these as control points
    // b) if the edge has more than 4 points, use the first 2 and the last 2 as control points
    // c) if the edge is straight and both nodes are geometry-like nodes, use geometry of the continuation edges as control points
    PositionVector init;
#ifdef DEBUG_SMOOTH_GEOM
    if (DEBUGCOND(this)) std::cout << getID()
                                       << " forElevation=" << forElevation
                                       << " fromGeometryLike=" << myNBEdge->getFromNode()->geometryLike()
                                       << " toGeometryLike=" << myNBEdge->getToNode()->geometryLike()
                                       << " smoothShape old=" << old << "\n";
#endif
    if (old.size() == 3 || old.size() == 4) {
        init = old;
    } else if (old.size() > 4 && !forElevation) {
        // for elevation, the initial segments are not useful
        init.push_back(old[0]);
        init.push_back(old[1]);
        init.push_back(old[-2]);
        init.push_back(old[-1]);
    } else if (myNBEdge->getFromNode()->geometryLike() && myNBEdge->getToNode()->geometryLike()) {
        PositionVector begShape;
        PositionVector endShape;
        const EdgeVector& incoming = myNBEdge->getFromNode()->getIncomingEdges();
        const EdgeVector& outgoing = myNBEdge->getToNode()->getOutgoingEdges();
        if (incoming.size() == 1) {
            begShape = incoming[0]->getGeometry();
        } else {
            assert(incoming.size() == 2);
            begShape = myNBEdge->isTurningDirectionAt(incoming[0]) ? incoming[1]->getGeometry() : incoming[0]->getGeometry();
        }
        if (outgoing.size() == 1) {
            endShape = outgoing[0]->getGeometry();
        } else {
            assert(outgoing.size() == 2);
            endShape = myNBEdge->isTurningDirectionAt(outgoing[0]) ? outgoing[1]->getGeometry() : outgoing[0]->getGeometry();
        }
        const double dist = MIN2(old.length2D(), MAX2(old.length2D() / 8, fabs(old[0].z() - old[-1].z()) * OptionsCont::getOptions().getFloat("geometry.max-grade") / 3));
        if (forElevation) {
            // initialize control point elevation for smooth continuation
            init.push_back(old[0]);
            init.push_back(old.positionAtOffset2D(dist));
            init.push_back(old.positionAtOffset2D(old.length2D() - dist));
            init.push_back(old[-1]);
            double begZ = begShape.positionAtOffset2D(MAX2(0.0, begShape.length2D() - dist)).z();
            double endZ = endShape.positionAtOffset2D(MIN2(begShape.length2D(), dist)).z();
            // continue incline
            init[1].setz(2 * init[0].z() - begZ);
            init[2].setz(2 * init[-1].z() - endZ);
        } else {
            bool ok = true;
            const double straightThresh = DEG2RAD(neteditOptions.getFloat("opendrive-output.straight-threshold"));
            init = NBNode::bezierControlPoints(begShape, endShape, false, dist, dist, ok, nullptr, straightThresh);
        }
#ifdef DEBUG_SMOOTH_GEOM
        if (DEBUGCOND(this)) {
            std::cout << "   begShape=" << begShape << " endShape=" << endShape << " forElevation=" << forElevation << " dist=" << dist << " ok=" << ok << " init=" << init << "\n";
        }
#endif
    }
    if (init.size() == 0) {
        return PositionVector::EMPTY;
    } else {
        const int numPoints = MAX2(neteditOptions.getInt("junctions.internal-link-detail"),
                                   int(old.length2D() / neteditOptions.getFloat("opendrive.curve-resolution")));
        return init.bezier(numPoints);
    }
}


void
GNEEdge::smooth(GNEUndoList* undoList) {
    PositionVector modifiedShape = smoothShape(myNBEdge->getGeometry(), false);
    if (modifiedShape.size() < 2) {
        WRITE_WARNINGF(TL("Could not compute smooth shape for edge '%'"), getID());
    } else {
        PositionVector innerShape(modifiedShape.begin() + 1, modifiedShape.end() - 1);
        setAttribute(SUMO_ATTR_SHAPE, toString(innerShape), undoList);
    }
}


void
GNEEdge::smoothElevation(GNEUndoList* undoList) {
    PositionVector elevationBase;
    for (const Position& pos : myNBEdge->getGeometry()) {
        if (elevationBase.size() == 0 || elevationBase[-1].z() != pos.z()) {
            elevationBase.push_back(pos);
        }
    }
    PositionVector elevation = smoothShape(elevationBase, true);
    if (elevation.size() <= 2) {
        WRITE_WARNINGF(TL("Could not compute smooth elevation for edge '%'"), getID());
    } else {
        PositionVector modifiedShape = myNBEdge->getGeometry();
        if (modifiedShape.size() < 5) {
            modifiedShape = modifiedShape.resample(OptionsCont::getOptions().getFloat("opendrive.curve-resolution"), false);
        }
        const double scale = elevation.length2D() / modifiedShape.length2D();
        //std::cout << "   elevation=" << elevation << "\n mod1=" << modifiedShape << " scale=" << scale << "\n";
        double seen = 0;
        for (int i = 1; i < (int)modifiedShape.size(); ++i) {
            seen += modifiedShape[i - 1].distanceTo2D(modifiedShape[i]);
            modifiedShape[i].setz(elevation.positionAtOffset2D(seen * scale).z());
        }
        //std::cout << "   mod2=" << modifiedShape << "\n";
        PositionVector innerShape(modifiedShape.begin() + 1, modifiedShape.end() - 1);
        setAttribute(SUMO_ATTR_SHAPE, toString(innerShape), undoList);
    }
}


void
GNEEdge::setShapeStartPos(const Position& pos) {
    // remove start position and add it the new position
    PositionVector geom = myNBEdge->getGeometry();
    geom.pop_front();
    geom.push_front(pos);
    // restore modified shape
    setGeometry(geom, false);
}


void
GNEEdge::setShapeEndPos(const Position& pos) {
    // remove end position and add it the new position
    PositionVector geom = myNBEdge->getGeometry();
    geom.pop_back();
    geom.push_back(pos);
    // restore modified shape
    setGeometry(geom, false);
}


const std::map<const GNELane*, std::vector<GNEDemandElement*> >
GNEEdge::getVehiclesOverEdgeMap() const {
    // declare vehicles over edge vector
    std::vector<GNEDemandElement*> vehiclesOverEdge;
    // declare solution map
    std::map<const GNELane*, std::vector<GNEDemandElement*> > vehiclesOverEdgeMap;
    // declare a set of vehicles (to avoid duplicates)
    std::set<std::pair<double, GNEDemandElement*> > vehicles;
    // first obtain all vehicles of this edge
    for (const auto& edgeChild : getChildDemandElements()) {
        if (((edgeChild->getTagProperty()->getTag() == SUMO_TAG_TRIP) || (edgeChild->getTagProperty()->getTag() == SUMO_TAG_FLOW)) &&
                (edgeChild->getParentEdges().front() == this)) {
            vehicles.insert(std::make_pair(edgeChild->getAttributeDouble(SUMO_ATTR_DEPART), edgeChild));
            vehicles.insert(std::make_pair(edgeChild->getAttributeDouble(SUMO_ATTR_DEPART), edgeChild));
        } else if ((edgeChild->getTagProperty()->getTag() == SUMO_TAG_ROUTE) && (edgeChild->getParentEdges().front() == this)) {
            for (const auto& routeChild : edgeChild->getChildDemandElements()) {
                if (routeChild->getTagProperty()->vehicleRoute()) {
                    vehicles.insert(std::make_pair(routeChild->getAttributeDouble(SUMO_ATTR_DEPART), routeChild));
                }
            }
        } else if ((edgeChild->getTagProperty()->getTag() == GNE_TAG_ROUTE_EMBEDDED) && (edgeChild->getParentEdges().front() == this) && (edgeChild->getParentDemandElements().size() > 0)) {
            vehicles.insert(std::make_pair(edgeChild->getParentDemandElements().front()->getAttributeDouble(SUMO_ATTR_DEPART), edgeChild->getParentDemandElements().front()));
        }
    }
    // reserve
    vehiclesOverEdge.reserve(vehicles.size());
    // iterate over vehicles
    for (const auto& vehicle : vehicles) {
        // add it over vehiclesOverEdge;
        vehiclesOverEdge.push_back(vehicle.second);
    }
    // now split vehicles by lanes
    for (const auto& vehicle : vehiclesOverEdge) {
        const GNELane* vehicleLane = vehicle->getFirstPathLane();
        if (vehicleLane) {
            vehiclesOverEdgeMap[vehicleLane].push_back(vehicle);
        }
    }
    return vehiclesOverEdgeMap;
}


const std::map<const GNELane*, std::vector<GNEDemandElement*> >
GNEEdge::getPersonsOverEdgeMap() const {
    // declare persons over edge vector
    std::vector<GNEDemandElement*> personsOverEdge;
    // declare solution map
    std::map<const GNELane*, std::vector<GNEDemandElement*> > personsOverEdgeMap;
    // declare a set of persons (to avoid duplicates)
    std::set<std::pair<double, GNEDemandElement*> > persons;
    // first obtain all persons of this edge
    for (const auto& edgeChild : getChildDemandElements()) {
        if (edgeChild->getTagProperty()->isPlanPerson() && (edgeChild->getParentDemandElements().size() > 0)) {
            persons.insert(std::make_pair(edgeChild->getParentDemandElements().front()->getAttributeDouble(SUMO_ATTR_DEPARTPOS), edgeChild->getParentDemandElements().front()));
        }
    }
    // reserve
    personsOverEdge.reserve(persons.size());
    // iterate over persons
    for (const auto& person : persons) {
        // add it over personsOverEdge;
        personsOverEdge.push_back(person.second);
    }
    // now split persons by lanes
    for (const auto& person : personsOverEdge) {
        const GNELane* personLane = person->getFirstPathLane();
        if (personLane) {
            personsOverEdgeMap[personLane].push_back(person);
        }
    }
    return personsOverEdgeMap;
}



const std::map<const GNELane*, std::vector<GNEDemandElement*> >
GNEEdge::getContainersOverEdgeMap() const {
    // declare containers over edge vector
    std::vector<GNEDemandElement*> containersOverEdge;
    // declare solution map
    std::map<const GNELane*, std::vector<GNEDemandElement*> > containersOverEdgeMap;
    // declare a set of containers (to avoid duplicates)
    std::set<std::pair<double, GNEDemandElement*> > containers;
    // first obtain all containers of this edge
    for (const auto& edgeChild : getChildDemandElements()) {
        if (edgeChild->getTagProperty()->isPlanContainer() && (edgeChild->getParentDemandElements().size() > 0)) {
            containers.insert(std::make_pair(edgeChild->getParentDemandElements().front()->getAttributeDouble(SUMO_ATTR_DEPARTPOS), edgeChild->getParentDemandElements().front()));
        }
    }
    // reserve
    containersOverEdge.reserve(containers.size());
    // iterate over containers
    for (const auto& container : containers) {
        // add it over containersOverEdge;
        containersOverEdge.push_back(container.second);
    }
    // now split containers by lanes
    for (const auto& container : containersOverEdge) {
        const GNELane* containerLane = container->getFirstPathLane();
        if (containerLane) {
            containersOverEdgeMap[containerLane].push_back(container);
        }
    }
    return containersOverEdgeMap;
}


void
GNEEdge::drawEdgeGeometryPoints(const GUIVisualizationSettings& s, const GUIVisualizationSettings::Detail d, const double layer) const {
    // first check conditions
    if (myNet->getViewNet()->getEditModes().isCurrentSupermodeNetwork() && (d <= GUIVisualizationSettings::Detail::GeometryPoint)) {
        // check if draw geometry points
        const bool bigGeometryPoints = drawBigGeometryPoints();
        // Obtain exaggeration of the draw
        const double exaggeration = getExaggeration(s);
        // get geometry point radius
        const double geometryPointRadius = getGeometryPointRadius();
        // obtain geometry point color
        RGBColor geometryPointColor = s.junctionColorer.getSchemes()[0].getColor(2);
        if (drawUsingSelectColor() && s.laneColorer.getActive() != 1) {
            // override with special colors (unless the color scheme is based on selection)
            geometryPointColor = s.colorSettings.selectedEdgeColor.changedBrightness(-20);
        }
        // draw geometry points except initial and final
        for (int i = 1; i < (int)myNBEdge->getGeometry().size() - 1; i++) {
            // obtain geometry point
            const auto geometryPointPos = myNBEdge->getGeometry()[i];
            // push geometry point drawing matrix
            GLHelper::pushMatrix();
            // set color
            GLHelper::setColor(geometryPointColor);
            // move geometry point geometryPointPos
            glTranslated(geometryPointPos.x(), geometryPointPos.y(), bigGeometryPoints ? GLO_GEOMETRYPOINT : GLO_LANE + 1);
            // draw filled circle (resolution of drawn circle depending of the zoom, to improve smoothness)
            GLHelper::drawFilledCircleDetailled(d, geometryPointRadius);
            // draw elevation or special symbols (Start, End and Block)
            if ((d <= GUIVisualizationSettings::Detail::Text) && myNet->getViewNet()->getNetworkViewOptions().editingElevation()) {
                // Translate to top
                glTranslated(0, 0, 0.2);
                // draw Z value
                GLHelper::drawText(toString(geometryPointPos.z()), Position(), 0, s.edgeValue.scaledSize(s.scale) / 2, s.edgeValue.color);
            }
            // pop geometry point drawing matrix
            GLHelper::popMatrix();
        }
        // draw start and end points
        if (bigGeometryPoints) {
            drawStartGeometryPoint(s, d, geometryPointRadius, layer, exaggeration);
            drawEndGeometryPoint(s, d, geometryPointRadius, layer, exaggeration);
        }
        // draw dotted contour geometry points
        myNetworkElementContour.drawDottedContourGeometryPoints(s, d, this, myNBEdge->getGeometry(), geometryPointRadius,
                exaggeration, s.dottedContourSettings.segmentWidthSmall);
    }
}


void
GNEEdge::drawStartGeometryPoint(const GUIVisualizationSettings& s, const GUIVisualizationSettings::Detail d,
                                const double geometryPointRadius, const double layer, const double exaggeration) const {
    // check detail level
    if (d <= GUIVisualizationSettings::Detail::GeometryPoint) {
        // get first geometry point
        const auto& startGeometryPointPos = myNBEdge->getGeometry().front();
        // get flags
        const bool startPosEdited = (startGeometryPointPos != getParentJunctions().front()->getPositionInView());
        const bool forceDraw = myNet->getViewParent()->getMoveFrame()->getNetworkMoveOptions()->getForceDrawGeometryPoints();
        // check drawing conditions
        if (startPosEdited || forceDraw) {
            // calculate angle betwen first and second geometry point
            const double angle = RAD2DEG(startGeometryPointPos.angleTo2D(myNBEdge->getGeometry()[1])) * -1;
            // get selected geometry points
            const auto selectedGeometryPoints = gViewObjectsHandler.getSelectedGeometryPoints(this);
            // override with special colors (unless the color scheme is based on selection)
            if (drawUsingSelectColor() && s.laneColorer.getActive() != 1) {
                GLHelper::setColor(s.colorSettings.selectedEdgeColor.changedBrightness(-20));
            } else {
                GLHelper::setColor(s.junctionColorer.getSchemes()[0].getColor(2));
            }
            // push drawing matrix
            GLHelper::pushMatrix();
            // move to point position
            glTranslated(startGeometryPointPos.x(), startGeometryPointPos.y(), GLO_GEOMETRYPOINT);
            // resolution of drawn circle depending of detail
            GLHelper::drawFilledCircleDetailled(d, geometryPointRadius, angle + 90, angle + 270);
            // pop drawing matrix
            GLHelper::popMatrix();
            // draw a "s" over last point depending of detail level
            if (d <= GUIVisualizationSettings::Detail::Text) {
                // push drawing matrix
                GLHelper::pushMatrix();
                // move top
                glTranslated(0, 0, GLO_GEOMETRYPOINT + 0.1);
                // draw S
                GLHelper::drawText("S", startGeometryPointPos, 0, geometryPointRadius, RGBColor(0, 50, 255));
                // pop drawing matrix
                GLHelper::popMatrix();
                // check if draw line between junctions
                if ((selectedGeometryPoints.size() > 0) && (selectedGeometryPoints.front() == 0)) {
                    // set base color
                    GLHelper::setColor(RGBColor::ORANGE);
                    // push drawing matrix
                    GLHelper::pushMatrix();
                    // draw line between geometry point and from junction
                    const PositionVector lineA = {startGeometryPointPos, getFromJunction()->getNBNode()->getPosition()};
                    GLHelper::drawBoxLine(lineA[1], RAD2DEG(lineA[0].angleTo2D(lineA[1])) - 90, lineA[0].distanceTo2D(lineA[1]), .1);
                    // draw line between begin point of last lane shape and the first edge shape point
                    const PositionVector lineB = {startGeometryPointPos, myNBEdge->getLanes().back().shape.front()};
                    GLHelper::drawBoxLine(lineB[1], RAD2DEG(lineB[0].angleTo2D(lineB[1])) - 90, lineB[0].distanceTo2D(lineB[1]), .1);
                    // pop drawing matrix
                    GLHelper::popMatrix();
                }
            }
            // draw dotted contour geometry points
            myNetworkElementContour.calculateContourFirstGeometryPoint(s, d, this, myNBEdge->getInnerGeometry(), layer,
                    geometryPointRadius, exaggeration);
        }
    }
}


void
GNEEdge::drawEndGeometryPoint(const GUIVisualizationSettings& s, const GUIVisualizationSettings::Detail d,
                              const double geometryPointRadius, const double layer, const double exaggeration) const {
    // check detail level
    if (d <= GUIVisualizationSettings::Detail::GeometryPoint) {
        // get first geometry point
        const auto& geometryPointPos = myNBEdge->getGeometry().back();
        // get flags
        const bool endPosEdited = (geometryPointPos != getParentJunctions().back()->getPositionInView());
        const bool forceDraw = myNet->getViewParent()->getMoveFrame()->getNetworkMoveOptions()->getForceDrawGeometryPoints();
        // check drawing conditions
        if (endPosEdited || forceDraw) {
            // calculate angle last and previous geometry point
            const double angle = RAD2DEG(geometryPointPos.angleTo2D(myNBEdge->getGeometry()[-2])) * -1;
            // get selected geometry points
            const auto selectedGeometryPoints = gViewObjectsHandler.getSelectedGeometryPoints(this);
            // override with special colors (unless the color scheme is based on selection)
            if (drawUsingSelectColor() && s.laneColorer.getActive() != 1) {
                GLHelper::setColor(s.colorSettings.selectedEdgeColor.changedBrightness(-20));
            } else {
                GLHelper::setColor(s.junctionColorer.getSchemes()[0].getColor(2));
            }
            // push drawing matrix
            GLHelper::pushMatrix();
            // move to point position
            glTranslated(geometryPointPos.x(), geometryPointPos.y(), GLO_GEOMETRYPOINT);
            // resolution of drawn circle depending of detail
            GLHelper::drawFilledCircleDetailled(d, geometryPointRadius, angle + 90, angle + 270);
            // pop drawing matrix
            GLHelper::popMatrix();
            // draw a "s" over last point depending of detail level
            if (d <= GUIVisualizationSettings::Detail::Text) {
                // push drawing matrix
                GLHelper::pushMatrix();
                // move top
                glTranslated(0, 0, GLO_GEOMETRYPOINT + 0.1);
                // draw S
                GLHelper::drawText("E", geometryPointPos, 0, geometryPointRadius, RGBColor(0, 50, 255));
                // pop drawing matrix
                GLHelper::popMatrix();
                // check if draw line between junctions
                if ((selectedGeometryPoints.size() > 0) && (selectedGeometryPoints.back() == ((int)myNBEdge->getGeometry().size() - 1))) {
                    // set base color
                    GLHelper::setColor(RGBColor::ORANGE);
                    // push drawing matrix
                    GLHelper::pushMatrix();
                    // draw line between geometry point and from junction
                    const PositionVector lineA = {geometryPointPos, getToJunction()->getNBNode()->getPosition()};
                    GLHelper::drawBoxLine(lineA[1], RAD2DEG(lineA[0].angleTo2D(lineA[1])) - 90, lineA[0].distanceTo2D(lineA[1]), .1);
                    // draw line between begin point of last lane shape and the first edge shape point
                    const PositionVector lineB = {geometryPointPos, myNBEdge->getLanes().back().shape.back()};
                    GLHelper::drawBoxLine(lineB[1], RAD2DEG(lineB[0].angleTo2D(lineB[1])) - 90, lineB[0].distanceTo2D(lineB[1]), .1);
                    // pop drawing matrix
                    GLHelper::popMatrix();
                }
            }
            // draw dotted contour geometry points
            myNetworkElementContour.calculateContourFirstGeometryPoint(s, d, this, myNBEdge->getInnerGeometry(), layer,
                    geometryPointRadius, exaggeration);
        }
    }
}


void
GNEEdge::drawEdgeName(const GUIVisualizationSettings& s) const {
    // draw the name and/or the street name
    const bool drawStreetName = s.streetName.show(this) && (myNBEdge->getStreetName() != "");
    const bool spreadSuperposed = s.spreadSuperposed && myNBEdge->getBidiEdge() != nullptr;
    // check conditions
    if (s.edgeName.show(this) || drawStreetName || s.edgeValue.show(this)) {
        // get first and last lanes
        const GNELane* firstLane = getChildLanes()[0];
        const GNELane* lastLane = getChildLanes()[getChildLanes().size() - 1];
        // calculate draw position
        Position drawPosition = firstLane->getLaneShape().positionAtOffset(firstLane->getLaneShape().length() / (double) 2.);
        drawPosition.add(lastLane->getLaneShape().positionAtOffset(lastLane->getLaneShape().length() / (double) 2.));
        drawPosition.mul(.5);
        if (spreadSuperposed) {
            // move name to the right of the edge and towards its beginning
            const double dist = 0.6 * s.edgeName.scaledSize(s.scale);
            const double shiftA = firstLane->getLaneShape().rotationAtOffset(firstLane->getLaneShape().length() / (double) 2.) - DEG2RAD(135);
            const Position shift(dist * cos(shiftA), dist * sin(shiftA));
            drawPosition.add(shift);
        }
        // calculate drawing angle
        double drawAngle = firstLane->getLaneShape().rotationDegreeAtOffset(firstLane->getLaneShape().length() / (double) 2.);
        drawAngle += 90;
        // avoid draw inverted text
        if (drawAngle > 90 && drawAngle < 270) {
            drawAngle -= 180;
        }
        // draw edge name
        if (s.edgeName.show(this)) {
            drawName(drawPosition, s.scale, s.edgeName, drawAngle);
        }
        // draw street name
        if (drawStreetName) {
            GLHelper::drawTextSettings(s.streetName, myNBEdge->getStreetName(), drawPosition, s.scale, drawAngle);
        }
        // draw edge values
        if (s.edgeValue.show(this)) {
            // get current scheme
            const int activeScheme = s.laneColorer.getActive();
            // calculate value depending of active scheme
            std::string value;
            if (activeScheme == 12) {
                // edge param, could be non-numerical
                value = getNBEdge()->getParameter(s.edgeParam, "");
            } else if (activeScheme == 13) {
                // lane param, could be non-numerical
                value = getNBEdge()->getLaneStruct(lastLane->getIndex()).getParameter(s.laneParam, "");
            } else {
                // use numerical value value of leftmost lane to hopefully avoid sidewalks, bikelanes etc
                const double doubleValue = lastLane->getColorValue(s, activeScheme);
                const RGBColor color = s.laneColorer.getScheme().getColor(doubleValue);
                value = color.alpha() == 0 ? "" : toString(doubleValue);
            }
            // check if value is empty
            if (value != "") {
                GLHelper::drawTextSettings(s.edgeValue, value, drawPosition, s.scale, drawAngle);
            }
        }
    }
}


void
GNEEdge::drawLaneStopOffset(const GUIVisualizationSettings& s, const GUIVisualizationSettings::Detail d, const double layer) const {
    // draw geometry only if we'rent in drawForObjectUnderCursor mode
    if (d <= GUIVisualizationSettings::Detail::LaneDetails) {
        // Push stopOffset matrix
        GLHelper::pushMatrix();
        // translate to front (note: Special case)
        drawInLayer(layer + 1);
        if (myNBEdge->myEdgeStopOffset.isDefined() && (myNBEdge->myEdgeStopOffset.getPermissions() & SVC_PASSENGER) != 0) {
            for (const auto& lane : getChildLanes()) {
                lane->drawLaneStopOffset(s);
            }
        }
        // Push stopOffset matrix
        GLHelper::popMatrix();
    }
}


void
GNEEdge::drawChildrens(const GUIVisualizationSettings& s) const {
    // draw child additional
    for (const auto& additional : getChildAdditionals()) {
        if (!additional->getTagProperty()->isListedElement()) {
            additional->drawGL(s);
        }
    }
    // draw person stops
    if (myNet->getViewNet()->getNetworkViewOptions().showDemandElements() && myNet->getViewNet()->getDataViewOptions().showDemandElements()) {
        for (const auto& stopEdge : getChildDemandElements()) {
            if ((stopEdge->getTagProperty()->getTag() == GNE_TAG_STOPPERSON_EDGE) || (stopEdge->getTagProperty()->getTag() == GNE_TAG_STOPCONTAINER_EDGE)) {
                stopEdge->drawGL(s);
            }
        }
    }
    // draw vehicles
    const std::map<const GNELane*, std::vector<GNEDemandElement*> > vehiclesMap = getVehiclesOverEdgeMap();
    for (const auto& vehicleMap : vehiclesMap) {
        for (const auto& vehicle : vehicleMap.second) {
            vehicle->drawGL(s);
        }
    }
    // draw TAZ elements
    drawTAZElements(s);
}


void
GNEEdge::calculateEdgeContour(const GUIVisualizationSettings& s, const GUIVisualizationSettings::Detail d, const double layer) const {
    // if we're selecting using a boundary, first don't calculate contour bt check if edge boundary is within selection boundary
    if (gViewObjectsHandler.selectingUsingRectangle() && gViewObjectsHandler.getSelectionTriangle().isBoundaryFullWithin(myEdgeBoundary)) {
        // simply add object in ViewObjectsHandler with full boundary
        gViewObjectsHandler.selectObject(this, layer, false, nullptr);
    } else {
        // get geometry point radius
        const auto geometryPointRadius = getGeometryPointRadius();
        // check if edit extrems
        const bool forceDrawExtrems = myNet->getViewParent()->getMoveFrame()->getNetworkMoveOptions()->getForceDrawGeometryPoints();
        const bool firstExtrem = forceDrawExtrems || (myNBEdge->getGeometry().front() != getParentJunctions().front()->getPositionInView());
        const bool lastExtrem = forceDrawExtrems || (myNBEdge->getGeometry().back() != getParentJunctions().back()->getPositionInView());
        // check if we're in move mode
        const bool moveMode = (myNet->getViewNet()->getEditModes().networkEditMode == NetworkEditMode::NETWORK_MOVE);
        // calculate contour
        myNetworkElementContour.calculateContourEdge(s, d, this, this, layer, true, true);
        // calculate edge geometry points
        myNetworkElementContour.calculateContourEdgeGeometryPoints(s, d, this, geometryPointRadius, moveMode, firstExtrem, lastExtrem);
    }
}


void
GNEEdge::drawTAZElements(const GUIVisualizationSettings& s) const {
    // first check if draw TAZ Elements is enabled
    if (myNet->getViewNet()->getNetworkViewOptions().showTAZElements()) {
        if (getChildTAZSourceSinks().size() > 0) {
            // check if TAZ Source/sink is selected
            bool selected = false;
            for (const auto& TAZSourceSink : getChildTAZSourceSinks()) {
                if (TAZSourceSink->isAttributeCarrierSelected()) {
                    selected = true;
                }
            }
            // iterate over lanes
            for (const auto& lane : getChildLanes()) {
                // Push layer matrix
                GLHelper::pushMatrix();
                // translate to front (note: Special case)
                if (myDrawInFront) {
                    glTranslated(0, 0, GLO_FRONTELEMENT);
                } else if (lane->getLaneShape().length2D() <= (s.neteditSizeSettings.junctionBubbleRadius * 2)) {
                    drawInLayer(GLO_JUNCTION + 0.5);
                } else {
                    drawInLayer(GLO_LANE);
                }
                // move to front
                glTranslated(0, 0, 0.1);
                // set color
                if (selected) {
                    GLHelper::setColor(RGBColor::BLUE);
                } else {
                    GLHelper::setColor(RGBColor::CYAN);
                }
                // draw as box lines
                GUIGeometry::drawGeometry(GUIVisualizationSettings::Detail::Level0, lane->getLaneGeometry(),
                                          lane->getDrawingConstants()->getDrawingWidth());
                // Pop layer matrix
                GLHelper::popMatrix();
            }
            /*
                // check if curently we're inspecting a TAZ Source/Sink
                for (const auto& TAZSourceSink : TAZSourceSinks) {
                    if (myNet->getViewNet()->isAttributeCarrierInspected(TAZSourceSink)) {
                        drawDottedContourEdge(s, GUIDottedGeometry::DottedContourType::INSPECT, this, true, true);
                    } else if (TAZSourceSink == markAC) {
                        drawDottedContourEdge(s, GUIDottedGeometry::DottedContourType::FRONT, this, true, true);
                    }
                }
            */
        }
    }
}


void
GNEEdge::drawEdgeShape(const GUIVisualizationSettings& s, const GUIVisualizationSettings::Detail d, const double layer) const {
    // avoid draw for railways
    if ((d <= GUIVisualizationSettings::Detail::LaneDetails) && !getChildLanes().front()->getDrawingConstants()->drawAsRailway() &&
            (gViewObjectsHandler.markedFirstGeometryPoint == this)) {
        // push draw matrix
        GLHelper::pushMatrix();
        // translate to front depending of big points
        if (drawBigGeometryPoints()) {
            glTranslated(0, 0, GLO_GEOMETRYPOINT - 1);
        } else {
            glTranslated(0, 0, layer);
        }
        // obtain color
        RGBColor geometryPointColor = s.junctionColorer.getSchemes()[0].getColor(2);
        if (drawUsingSelectColor() && s.laneColorer.getActive() != 1) {
            // override with special colors (unless the color scheme is based on selection)
            geometryPointColor = s.colorSettings.selectedEdgeColor.changedBrightness(-20);
        }
        // set color
        GLHelper::setColor(geometryPointColor);
        // iterate over NBEdge geometry
        for (int i = 1; i < (int)myNBEdge->getGeometry().size(); i++) {
            // calculate line between previous and current geometry point
            PositionVector line = {myNBEdge->getGeometry()[i - 1], myNBEdge->getGeometry()[i]};
            line.move2side(0.2);
            // draw box line
            GLHelper::drawBoxLine(line[1], RAD2DEG(line[0].angleTo2D(line[1])) - 90, line[0].distanceTo2D(line[1]), .1);
        }
        // pop draw matrix
        GLHelper::popMatrix();
    }
}


bool
GNEEdge::drawBigGeometryPoints() const {
    // get edit modes
    const auto& editModes = myNet->getViewNet()->getEditModes();
    // continue depending of conditions
    if (!editModes.isCurrentSupermodeNetwork()) {
        return false;
    } else if (editModes.networkEditMode == NetworkEditMode::NETWORK_MOVE) {
        return true;
    } else if ((editModes.networkEditMode == NetworkEditMode::NETWORK_DELETE) &&
               (myNet->getViewParent()->getDeleteFrame()->getDeleteOptions()->deleteOnlyGeometryPoints())) {
        return true;
    } else {
        return false;
    }
}


bool
GNEEdge::areStackPositionOverlapped(const GNEEdge::StackPosition& vehicleA, const GNEEdge::StackPosition& vehicleB) const {
    if ((vehicleA.beginPosition() == vehicleB.beginPosition()) && (vehicleA.endPosition() == vehicleB.endPosition())) {
        return true;
    } else if ((vehicleA.beginPosition() < vehicleB.beginPosition()) && (vehicleA.endPosition() > vehicleB.endPosition())) {
        return true;
    } else if ((vehicleA.beginPosition() < vehicleB.beginPosition()) && (vehicleA.endPosition() > vehicleB.beginPosition())) {
        return true;
    } else if ((vehicleA.beginPosition() < vehicleB.endPosition()) && (vehicleA.endPosition() > vehicleB.endPosition())) {
        return true;
    } else {
        return false;
    }
}


double
GNEEdge::getGeometryPointRadius() const {
    return drawBigGeometryPoints() ? SNAP_RADIUS * MIN2(1.0, myNet->getViewNet()->getVisualisationSettings().laneWidthExaggeration) : 0.5;
}

/****************************************************************************/
