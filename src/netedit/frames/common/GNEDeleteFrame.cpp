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
/// @file    GNEDeleteFrame.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Dec 2016
///
// The Widget for remove network-elements
/****************************************************************************/
#include <config.h>

#include <netedit/GNENet.h>
#include <netedit/GNEUndoList.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNEViewParent.h>
#include <netedit/GNEApplicationWindow.h>
#include <netedit/elements/additional/GNEPoly.h>
#include <netedit/elements/additional/GNETAZ.h>
#include <netedit/elements/network/GNEConnection.h>
#include <netedit/elements/network/GNECrossing.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/foxtools/MFXMenuHeader.h>

#include "GNEDeleteFrame.h"

// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNEDeleteFrame::DeleteOptions) DeleteOptionsMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SET_ATTRIBUTE,  GNEDeleteFrame::DeleteOptions::onCmdSetOption),
};

FXDEFMAP(GNEDeleteFrame::ProtectElements) ProtectElementsMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_PROTECT_ALL,    GNEDeleteFrame::ProtectElements::onCmdProtectAll),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_PROTECT_ALL,    GNEDeleteFrame::ProtectElements::onUpdProtectAll),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_UNPROTECT_ALL,  GNEDeleteFrame::ProtectElements::onCmdUnprotectAll),
    FXMAPFUNC(SEL_UPDATE,   MID_GNE_UNPROTECT_ALL,  GNEDeleteFrame::ProtectElements::onUpdUnprotectAll),
};

// Object implementation
FXIMPLEMENT(GNEDeleteFrame::DeleteOptions,      MFXGroupBoxModule, DeleteOptionsMap,    ARRAYNUMBER(DeleteOptionsMap))
FXIMPLEMENT(GNEDeleteFrame::ProtectElements,    MFXGroupBoxModule, ProtectElementsMap,  ARRAYNUMBER(ProtectElementsMap))

// ---------------------------------------------------------------------------
// GNEDeleteFrame::DeleteOptions - methods
// ---------------------------------------------------------------------------

GNEDeleteFrame::DeleteOptions::DeleteOptions(GNEDeleteFrame* deleteFrameParent) :
    MFXGroupBoxModule(deleteFrameParent, TL("Options")),
    myDeleteFrameParent(deleteFrameParent) {
    // Create checkbox for enable/disable delete only geomtery point(by default, disabled)
    myDeleteOnlyGeometryPoints = new FXCheckButton(getCollapsableFrame(), TL("Delete geometry points"), this, MID_GNE_SET_ATTRIBUTE, GUIDesignCheckButton);
    myDeleteOnlyGeometryPoints->setCheck(FALSE);
}


GNEDeleteFrame::DeleteOptions::~DeleteOptions() {}


bool
GNEDeleteFrame::DeleteOptions::deleteOnlyGeometryPoints() const {
    return (myDeleteOnlyGeometryPoints->getCheck() == TRUE);
}


long
GNEDeleteFrame::DeleteOptions::onCmdSetOption(FXObject*, FXSelector, void*) {
    myDeleteFrameParent->getViewNet()->update();
    return 1;
}

// ---------------------------------------------------------------------------
// GNEDeleteFrame::SubordinatedElements - methods
// ---------------------------------------------------------------------------

GNEDeleteFrame::SubordinatedElements::SubordinatedElements(const GNEJunction* junction) :
    SubordinatedElements(junction, junction->getNet()->getViewNet(), junction) {
    // add the number of subodinated elements of child edges
    for (const auto& edge : junction->getChildEdges()) {
        addValuesFromSubordinatedElements(this, edge);
    }
}


GNEDeleteFrame::SubordinatedElements::SubordinatedElements(const GNEEdge* edge) :
    SubordinatedElements(edge, edge->getNet()->getViewNet(), edge) {
    // add the number of subodinated elements of child lanes
    for (const auto& lane : edge->getLanes()) {
        addValuesFromSubordinatedElements(this, lane);
    }
}


GNEDeleteFrame::SubordinatedElements::SubordinatedElements(const GNELane* lane) :
    SubordinatedElements(lane, lane->getNet()->getViewNet(), lane) {
}


GNEDeleteFrame::SubordinatedElements::SubordinatedElements(const GNEAdditional* additional) :
    SubordinatedElements(additional, additional->getNet()->getViewNet()) {
}


GNEDeleteFrame::SubordinatedElements::SubordinatedElements(const GNEDemandElement* demandElement) :
    SubordinatedElements(demandElement, demandElement->getNet()->getViewNet()) {
}


GNEDeleteFrame::SubordinatedElements::SubordinatedElements(const GNEGenericData* genericData) :
    SubordinatedElements(genericData, genericData->getNet()->getViewNet()) {
}


GNEDeleteFrame::SubordinatedElements::~SubordinatedElements() {}


bool
GNEDeleteFrame::SubordinatedElements::checkElements(const ProtectElements* protectElements) {
    // check every parent/child
    if ((myAdditionalParents > 0) && protectElements->protectAdditionals()) {
        openWarningDialog("additional", myAdditionalParents, false);
    } else if ((myAdditionalChilds > 0) && protectElements->protectAdditionals()) {
        openWarningDialog("additional", myAdditionalChilds, true);
    } else if ((myDemandElementParents > 0) && protectElements->protectDemandElements()) {
        openWarningDialog("demand", myDemandElementParents, false);
    } else if ((myDemandElementChilds > 0) && protectElements->protectDemandElements()) {
        openWarningDialog("demand", myDemandElementChilds, true);
    } else if ((myGenericDataParents > 0) && protectElements->protectGenericDatas()) {
        openWarningDialog("data", myGenericDataParents, false);
    } else if ((myGenericDataChilds > 0) && protectElements->protectGenericDatas()) {
        openWarningDialog("data", myGenericDataChilds, true);
    } else {
        // all checks ok, then return true, to remove element
        return true;
    }
    return false;
}


GNEDeleteFrame::SubordinatedElements::SubordinatedElements(const GNEAttributeCarrier* attributeCarrier, GNEViewNet* viewNet) :
    myAttributeCarrier(attributeCarrier),
    myViewNet(viewNet),
    myAdditionalParents(0),
    myAdditionalChilds(0),
    myDemandElementParents(0),
    myDemandElementChilds(0),
    myGenericDataParents(0),
    myGenericDataChilds(0) {
}


GNEDeleteFrame::SubordinatedElements::SubordinatedElements(const GNEAttributeCarrier* attributeCarrier, GNEViewNet* viewNet,
        const GNEHierarchicalElement* hierarchicalElement) :
    myAttributeCarrier(attributeCarrier),
    myViewNet(viewNet),
    myAdditionalParents(hierarchicalElement->getParentAdditionals().size()),
    myAdditionalChilds(hierarchicalElement->getChildAdditionals().size()),
    myDemandElementParents(hierarchicalElement->getParentDemandElements().size()),
    myDemandElementChilds(hierarchicalElement->getChildDemandElements().size()),
    myGenericDataParents(hierarchicalElement->getParentGenericDatas().size()),
    myGenericDataChilds(hierarchicalElement->getChildGenericDatas().size()) {
    // add the number of subodinated elements of additionals, demand elements and generic datas
    for (const auto& additionalParent : hierarchicalElement->getParentAdditionals()) {
        addValuesFromSubordinatedElements(this, additionalParent);
    }
    for (const auto& demandParent : hierarchicalElement->getParentDemandElements()) {
        addValuesFromSubordinatedElements(this, demandParent);
    }
    for (const auto& genericDataParent : hierarchicalElement->getParentGenericDatas()) {
        addValuesFromSubordinatedElements(this, genericDataParent);
    }
    for (const auto& additionalChild : hierarchicalElement->getChildAdditionals()) {
        addValuesFromSubordinatedElements(this, additionalChild);
    }
    for (const auto& demandChild : hierarchicalElement->getChildDemandElements()) {
        addValuesFromSubordinatedElements(this, demandChild);
    }
    for (const auto& genericDataChild : hierarchicalElement->getChildGenericDatas()) {
        addValuesFromSubordinatedElements(this, genericDataChild);
    }
}


void
GNEDeleteFrame::SubordinatedElements::addValuesFromSubordinatedElements(SubordinatedElements* originalSE, const SubordinatedElements& newSE) {
    originalSE->myAdditionalParents += newSE.myAdditionalParents;
    originalSE->myAdditionalChilds += newSE.myAdditionalChilds;
    originalSE->myDemandElementParents += newSE.myDemandElementParents;
    originalSE->myDemandElementChilds += newSE.myDemandElementChilds;
    originalSE->myGenericDataParents += newSE.myGenericDataParents;
    originalSE->myGenericDataChilds += newSE.myGenericDataChilds;
}


void
GNEDeleteFrame::SubordinatedElements::openWarningDialog(const std::string& type, const size_t number, const bool isChild) {
    // declare plural depending of "number"
    const std::string plural = (number > 1) ? "s" : "";
    // declare header
    const std::string header = "Problem deleting " + myAttributeCarrier->getTagProperty().getTagStr() + " '" + myAttributeCarrier->getID() + "'";
    // declare message
    std::string msg;
    // set message depending of isChild
    if (isChild) {
        msg = myAttributeCarrier->getTagProperty().getTagStr() + " '" + myAttributeCarrier->getID() +
              "' cannot be deleted because it has " + toString(number) + " " + type + " element" + plural + ".\n" +
              "To delete it, uncheck 'protect " + type + " elements'.";
    } else {
        msg = myAttributeCarrier->getTagProperty().getTagStr() + " '" + myAttributeCarrier->getID() +
              "' cannot be deleted because it is part of " + toString(number) + " " + type + " element" + plural + ".\n" +
              "To delete it, uncheck 'protect " + type + " elements'.";
    }
    // write warning
    WRITE_DEBUG("Opened FXMessageBox " + header);
    // open message box
    FXMessageBox::warning(myViewNet->getApp(), MBOX_OK, header.c_str(), "%s", msg.c_str());
    // write warning if netedit is running in testing mode
    WRITE_DEBUG("Closed FXMessageBox " + header);
}

// ---------------------------------------------------------------------------
// GNEDeleteFrame::ProtectElements - methods
// ---------------------------------------------------------------------------

GNEDeleteFrame::ProtectElements::ProtectElements(GNEDeleteFrame* deleteFrameParent) :
    MFXGroupBoxModule(deleteFrameParent, TL("Protect Elements")) {
    // Create "Protect all" Button
    GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Protect all"), "", TL("Protect all elements"), nullptr, this, MID_GNE_PROTECT_ALL, GUIDesignButton);
    // Create "Unprotect all" Button
    GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Unprotect all"), "", TL("Unprotect all elements"), nullptr, this, MID_GNE_UNPROTECT_ALL, GUIDesignButton);
    // Create checkbox for enable/disable delete only geomtery point(by default, disabled)
    myProtectAdditionals = new FXCheckButton(getCollapsableFrame(), TL("Protect additional elements"), deleteFrameParent, MID_GNE_SET_ATTRIBUTE, GUIDesignCheckButton);
    myProtectAdditionals->setCheck(TRUE);
    // Create checkbox for enable/disable delete only geomtery point(by default, disabled)
    myProtectTAZs = new FXCheckButton(getCollapsableFrame(), TL("Protect TAZ elements"), deleteFrameParent, MID_GNE_SET_ATTRIBUTE, GUIDesignCheckButton);
    myProtectTAZs->setCheck(TRUE);
    // Create checkbox for enable/disable delete only geomtery point(by default, disabled)
    myProtectDemandElements = new FXCheckButton(getCollapsableFrame(), TL("Protect demand elements"), deleteFrameParent, MID_GNE_SET_ATTRIBUTE, GUIDesignCheckButton);
    myProtectDemandElements->setCheck(TRUE);
    // Create checkbox for enable/disable delete only geomtery point(by default, disabled)
    myProtectGenericDatas = new FXCheckButton(getCollapsableFrame(), TL("Protect data elements"), deleteFrameParent, MID_GNE_SET_ATTRIBUTE, GUIDesignCheckButton);
    myProtectGenericDatas->setCheck(TRUE);
}


GNEDeleteFrame::ProtectElements::~ProtectElements() {}


bool
GNEDeleteFrame::ProtectElements::protectAdditionals() const {
    return (myProtectAdditionals->getCheck() == TRUE);
}


bool
GNEDeleteFrame::ProtectElements::protectTAZs() const {
    return (myProtectTAZs->getCheck() == TRUE);
}


bool
GNEDeleteFrame::ProtectElements::protectDemandElements() const {
    return (myProtectDemandElements->getCheck() == TRUE);
}


bool
GNEDeleteFrame::ProtectElements::protectGenericDatas() const {
    return (myProtectGenericDatas->getCheck() == TRUE);
}


long
GNEDeleteFrame::ProtectElements::onCmdProtectAll(FXObject*, FXSelector, void*) {
    myProtectAdditionals->setCheck(TRUE);
    myProtectTAZs->setCheck(TRUE);
    myProtectDemandElements->setCheck(TRUE);
    myProtectGenericDatas->setCheck(TRUE);
    return 1;
}


long
GNEDeleteFrame::ProtectElements::onCmdUnprotectAll(FXObject*, FXSelector, void*) {
    myProtectAdditionals->setCheck(FALSE);
    myProtectTAZs->setCheck(FALSE);
    myProtectDemandElements->setCheck(FALSE);
    myProtectGenericDatas->setCheck(FALSE);
    return 1;
}


long
GNEDeleteFrame::ProtectElements::onUpdProtectAll(FXObject* sender, FXSelector, void*) {
    if (myProtectAdditionals->getCheck() && myProtectTAZs->getCheck() &&
            myProtectDemandElements->getCheck() && myProtectGenericDatas->getCheck()) {
        return sender->handle(this, FXSEL(SEL_COMMAND, ID_DISABLE), nullptr);
    } else {
        return sender->handle(this, FXSEL(SEL_COMMAND, ID_ENABLE), nullptr);
    }
}


long
GNEDeleteFrame::ProtectElements::onUpdUnprotectAll(FXObject* sender, FXSelector, void*) {
    if (!myProtectAdditionals->getCheck() && !myProtectTAZs->getCheck() &&
            !myProtectDemandElements->getCheck() && !myProtectGenericDatas->getCheck()) {
        return sender->handle(this, FXSEL(SEL_COMMAND, ID_DISABLE), nullptr);
    } else {
        return sender->handle(this, FXSEL(SEL_COMMAND, ID_ENABLE), nullptr);
    }
}

// ===========================================================================
// method definitions
// ===========================================================================

GNEDeleteFrame::GNEDeleteFrame(GNEViewParent* viewParent, GNEViewNet* viewNet) :
    GNEFrame(viewParent, viewNet, TL("Delete")) {
    // create delete options module
    myDeleteOptions = new DeleteOptions(this);
    // create protect elements module
    myProtectElements = new ProtectElements(this);
}


GNEDeleteFrame::~GNEDeleteFrame() {
}


void
GNEDeleteFrame::show() {
    GNEFrame::show();
}


void
GNEDeleteFrame::hide() {
    GNEFrame::hide();
}


void
GNEDeleteFrame::removeSelectedAttributeCarriers() {
    // get attribute carriers
    const auto& attributeCarriers = myViewNet->getNet()->getAttributeCarriers();
    // first check if there is additional to remove
    if (selectedACsToDelete()) {
        // remove all selected attribute carriers using the following parent-child sequence
        myViewNet->getUndoList()->begin(GUIIcon::MODEDELETE, TL("remove selected items"));
        // disable update geometry
        myViewNet->getNet()->disableUpdateGeometry();
        // delete selected attribute carriers depending of current supermode
        if (myViewNet->getEditModes().isCurrentSupermodeNetwork()) {
            //junctions
            const auto selectedJunctions = attributeCarriers->getSelectedJunctions();
            for (const auto& selectedJunction : selectedJunctions) {
                myViewNet->getNet()->deleteJunction(selectedJunction, myViewNet->getUndoList());
            }
            // edges
            const auto selectedEdges = attributeCarriers->getSelectedEdges();
            for (const auto& selectedEdge : selectedEdges) {
                myViewNet->getNet()->deleteEdge(selectedEdge, myViewNet->getUndoList(), false);
            }
            // lanes
            const auto selectedLanes = attributeCarriers->getSelectedLanes();
            for (const auto& selectedLane : selectedLanes) {
                myViewNet->getNet()->deleteLane(selectedLane, myViewNet->getUndoList(), false);
            }
            // connections
            const auto selectedConnections = attributeCarriers->getSelectedConnections();
            for (const auto& selectedConnection : selectedConnections) {
                myViewNet->getNet()->deleteConnection(selectedConnection, myViewNet->getUndoList());
            }
            // crossings
            const auto selectedCrossings = attributeCarriers->getSelectedCrossings();
            for (const auto& selectedCrossing : selectedCrossings) {
                myViewNet->getNet()->deleteCrossing(selectedCrossing, myViewNet->getUndoList());
            }
            // additionals (including Polygons, POIs, TAZs and Wires)
            while (attributeCarriers->getNumberOfSelectedAdditionals() > 0) {
                myViewNet->getNet()->deleteAdditional(attributeCarriers->getSelectedAdditionals().front(), myViewNet->getUndoList());
            }
        } else if (myViewNet->getEditModes().isCurrentSupermodeDemand()) {
            // demand elements
            while (attributeCarriers->getNumberOfSelectedDemandElements() > 0) {
                myViewNet->getNet()->deleteDemandElement(attributeCarriers->getSelectedDemandElements().front(), myViewNet->getUndoList());
            }
        } else if (myViewNet->getEditModes().isCurrentSupermodeData()) {
            // generic datas
            auto selectedGenericDatas = attributeCarriers->getSelectedGenericDatas();
            for (const auto& selectedGenericData : selectedGenericDatas) {
                myViewNet->getNet()->deleteGenericData(selectedGenericData, myViewNet->getUndoList());
            }
        }
        // enable update geometry
        myViewNet->getNet()->enableUpdateGeometry();
        // finish deletion
        myViewNet->getUndoList()->end();
    }
}


void
GNEDeleteFrame::removeAttributeCarrier(const GNEViewNetHelper::ViewObjectsSelector& viewObjects) {
    // disable update geometry
    myViewNet->getNet()->disableUpdateGeometry();
    // first check if there more than one clicked GL object under cursor
    if (viewObjects.getGLObjects().size() > 1) {
        std::vector<GUIGlObject*> filteredGLObjects;
        // filter objects
        for (const auto& glObject : viewObjects.getGLObjects()) {
            if (glObject->isGLObjectLocked()) {
                continue;
            }
            filteredGLObjects.push_back(glObject);
        }
        // now filter elements based on the first element
        filteredGLObjects = GNEViewNetHelper::filterElementsByLayer(filteredGLObjects);
        // after filter, check if there is more than one element
        if (filteredGLObjects.size() > 1) {
            // use Cursor dialog
            myViewNet->openDeleteDialogAtCursor(filteredGLObjects);
        } else if (filteredGLObjects.size() > 0) {
            filteredGLObjects.front()->deleteGLObject();
        }
    } else if ((viewObjects.getGLObjects().size() > 0) &&
               !viewObjects.getGLObjects().front()->isGLObjectLocked()) {
        viewObjects.getGLObjects().front()->deleteGLObject();
    }
    // enable update geometry
    myViewNet->getNet()->enableUpdateGeometry();
}


bool
GNEDeleteFrame::removeGeometryPoint(const GNEViewNetHelper::ViewObjectsSelector& viewObjects) {
    // get clicked position
    const Position clickedPosition = myViewNet->getPositionInformation();
    // filter elements with geometry points
    for (const auto& AC : viewObjects.getAttributeCarriers()) {
        if (AC->getTagProperty().getTag() == SUMO_TAG_EDGE) {
            viewObjects.getEdgeFront()->removeGeometryPoint(clickedPosition, myViewNet->getUndoList());
            return true;
        } else if (AC->getTagProperty().getTag() == SUMO_TAG_POLY) {
            viewObjects.getPolyFront()->removeGeometryPoint(clickedPosition, myViewNet->getUndoList());
            return true;
        } else if (AC->getTagProperty().getTag() == SUMO_TAG_TAZ) {
            viewObjects.getTAZFront()->removeGeometryPoint(clickedPosition, myViewNet->getUndoList());
            return true;
        }
    }
    return false;
}


GNEDeleteFrame::DeleteOptions*
GNEDeleteFrame::getDeleteOptions() const {
    return myDeleteOptions;
}


GNEDeleteFrame::ProtectElements*
GNEDeleteFrame::getProtectElements() const {
    return myProtectElements;
}

// ---------------------------------------------------------------------------
// GNEAdditionalFrame - protected methods
// ---------------------------------------------------------------------------

bool
GNEDeleteFrame::selectedACsToDelete() const {
    // invert selection of elements depending of current supermode
    if (myViewNet->getEditModes().isCurrentSupermodeNetwork()) {
        // iterate over junctions
        for (const auto& junction : myViewNet->getNet()->getAttributeCarriers()->getJunctions()) {
            if (junction.second->isAttributeCarrierSelected()) {
                return true;
            }
            // since we iterate over all junctions, it's only necessary to iterate over incoming edges
            for (const auto& edge : junction.second->getGNEIncomingEdges()) {
                if (edge->isAttributeCarrierSelected()) {
                    return true;
                }
                // check lanes
                for (const auto& lane : edge->getLanes()) {
                    if (lane->isAttributeCarrierSelected()) {
                        return true;
                    }
                }
                // check connections
                for (const auto& connection : edge->getGNEConnections()) {
                    if (connection->isAttributeCarrierSelected()) {
                        return true;
                    }
                }
            }
            // check crossings
            for (const auto& crossing : junction.second->getGNECrossings()) {
                if (crossing->isAttributeCarrierSelected()) {
                    return true;
                }
            }
        }
        // check additionals
        for (const auto& additionalTag : myViewNet->getNet()->getAttributeCarriers()->getAdditionals()) {
            for (const auto& additional : additionalTag.second) {
                if (additional.second->isAttributeCarrierSelected()) {
                    return true;
                }
            }
        }
    } else if (myViewNet->getEditModes().isCurrentSupermodeDemand()) {
        // check demand elements
        for (const auto& demandElementTag : myViewNet->getNet()->getAttributeCarriers()->getDemandElements()) {
            for (const auto& demandElement : demandElementTag.second) {
                if (demandElement.second->isAttributeCarrierSelected()) {
                    return true;
                }
            }
        }
    } else if (myViewNet->getEditModes().isCurrentSupermodeData()) {
        // iterate over all generic datas
        for (const auto& genericDataTag : myViewNet->getNet()->getAttributeCarriers()->getGenericDatas()) {
            for (const auto& genericData : genericDataTag.second) {
                if (genericData.second->isAttributeCarrierSelected()) {
                    return true;
                }
            }
        }
    }
    return false;
}

/****************************************************************************/
