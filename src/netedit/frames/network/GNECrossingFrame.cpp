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
/// @file    GNECrossingFrame.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Oct 2016
///
// The Widget for add Crossing elements
/****************************************************************************/
#include <config.h>

#include <utils/foxtools/MFXDynamicLabel.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/div/GUIDesigns.h>
#include <netedit/changes/GNEChange_Crossing.h>
#include <netedit/elements/network/GNECrossing.h>
#include <netedit/frames/network/GNECreateEdgeFrame.h>
#include <netedit/GNENet.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNEViewParent.h>
#include <netedit/GNEUndoList.h>

#include "GNECrossingFrame.h"


// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNECrossingFrame::EdgesSelector) EdgesSelectorMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_USESELECTED,        GNECrossingFrame::EdgesSelector::onCmdUseSelectedEdges),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_CLEARSELECTION,     GNECrossingFrame::EdgesSelector::onCmdClearSelection)
};

FXDEFMAP(GNECrossingFrame::CrossingParameters) CrossingParametersMap[] = {
    FXMAPFUNC(SEL_COMMAND, MID_GNE_SET_ATTRIBUTE,   GNECrossingFrame::CrossingParameters::onCmdSetAttribute),
    FXMAPFUNC(SEL_COMMAND, MID_HELP,                GNECrossingFrame::CrossingParameters::onCmdHelp),
};

FXDEFMAP(GNECrossingFrame::CreateCrossing) CreateCrossingMap[] = {
    FXMAPFUNC(SEL_COMMAND, MID_GNE_CREATE,  GNECrossingFrame::CreateCrossing::onCmdCreateCrossing),
};

// Object implementation
FXIMPLEMENT(GNECrossingFrame::EdgesSelector,        MFXGroupBoxModule,     EdgesSelectorMap,       ARRAYNUMBER(EdgesSelectorMap))
FXIMPLEMENT(GNECrossingFrame::CrossingParameters,   MFXGroupBoxModule,     CrossingParametersMap,  ARRAYNUMBER(CrossingParametersMap))
FXIMPLEMENT(GNECrossingFrame::CreateCrossing,       MFXGroupBoxModule,     CreateCrossingMap,      ARRAYNUMBER(CreateCrossingMap))


// ===========================================================================
// method definitions
// ===========================================================================

// ---------------------------------------------------------------------------
// GNECrossingFrame::CurrentJunction - methods
// ---------------------------------------------------------------------------

GNECrossingFrame::JunctionInformation::JunctionInformation(GNECrossingFrame* crossingFrameParent) :
    MFXGroupBoxModule(crossingFrameParent, TL("Junction")) {
    // Create frame for junction ID
    FXHorizontalFrame* junctionIDFrame = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    // create label
    new FXLabel(junctionIDFrame, "", nullptr, GUIDesignLabelThickedFixed(100));
    // create text field and disable it
    myTextFieldJunctionID = new FXTextField(junctionIDFrame, GUIDesignTextFieldNCol, this, MID_GNE_SELECT, GUIDesignTextField);
    myTextFieldJunctionID->disable();
}


GNECrossingFrame::JunctionInformation::~JunctionInformation() {}


void
GNECrossingFrame::JunctionInformation::updateCurrentJunctionLabel(const std::string& junctionID) {
    if (junctionID.empty()) {
        myTextFieldJunctionID->setText("");
    } else {
        myTextFieldJunctionID->setText(junctionID.c_str());
    }
}

// ---------------------------------------------------------------------------
// GNECrossingFrame::EdgesSelector - methods
// ---------------------------------------------------------------------------

GNECrossingFrame::EdgesSelector::EdgesSelector(GNECrossingFrame* crossingFrameParent) :
    MFXGroupBoxModule(crossingFrameParent, TL("selection of edges")),
    myCrossingFrameParent(crossingFrameParent),
    myCurrentJunction(nullptr) {

    // Create button for selected edges
    myUseSelectedEdges = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Use selected edges"), "", "", nullptr, this, MID_GNE_USESELECTED, GUIDesignButton);

    // Create button for clear selection
    myClearEdgesSelection = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Clear edges"), "", "", nullptr, this, MID_GNE_CLEARSELECTION, GUIDesignButton);
}


GNECrossingFrame::EdgesSelector::~EdgesSelector() {}


GNEJunction*
GNECrossingFrame::EdgesSelector::getCurrentJunction() const {
    return myCurrentJunction;
}


void
GNECrossingFrame::EdgesSelector::enableEdgeSelector(GNEJunction* currentJunction) {
    // restore color of all lanes of edge candidates
    restoreEdgeColors();
    // Set current junction
    myCurrentJunction = currentJunction;
    // Update view net to show the new colors
    myCrossingFrameParent->getViewNet()->updateViewNet();
    // check if use selected eges must be enabled
    myUseSelectedEdges->disable();
    for (const auto& edge : myCurrentJunction->getChildEdges()) {
        if (edge->isAttributeCarrierSelected()) {
            myUseSelectedEdges->enable();
        }
    }
    // Enable rest of elements
    myClearEdgesSelection->enable();
}


void
GNECrossingFrame::EdgesSelector::disableEdgeSelector() {
    // disable current junction
    myCurrentJunction = nullptr;
    // disable all elements of the EdgesSelector
    myUseSelectedEdges->disable();
    myClearEdgesSelection->disable();
    // Disable crossing parameters
    myCrossingFrameParent->myCrossingParameters->disableCrossingParameters();
    // Update view net to show the new colors
    myCrossingFrameParent->getViewNet()->updateViewNet();
}


void
GNECrossingFrame::EdgesSelector::restoreEdgeColors() {
    if (myCurrentJunction != nullptr) {
        // restore color of all lanes of edge candidates
        for (const auto& edge : myCurrentJunction->getChildEdges()) {
            edge->resetCandidateFlags();
        }
        // Update view net to show the new colors
        myCrossingFrameParent->getViewNet()->updateViewNet();
        myCurrentJunction = nullptr;
    }
}


long
GNECrossingFrame::EdgesSelector::onCmdUseSelectedEdges(FXObject*, FXSelector, void*) {
    myCrossingFrameParent->myCrossingParameters->useSelectedEdges(myCurrentJunction);
    return 1;
}


long
GNECrossingFrame::EdgesSelector::onCmdClearSelection(FXObject*, FXSelector, void*) {
    myCrossingFrameParent->myCrossingParameters->clearEdges();
    return 1;
}

// ---------------------------------------------------------------------------
// GNECrossingFrame::GNENeteditAttributes- methods
// ---------------------------------------------------------------------------

GNECrossingFrame::CrossingParameters::CrossingParameters(GNECrossingFrame* crossingFrameParent) :
    MFXGroupBoxModule(crossingFrameParent, TL("Crossing parameters")),
    myCrossingFrameParent(crossingFrameParent),
    myCrossingTemplate(nullptr),
    myCurrentParametersValid(true) {
    // createcrossing template
    myCrossingTemplate = new GNECrossing(crossingFrameParent->getViewNet()->getNet());
    FXHorizontalFrame* crossingParameter = nullptr;
    // create label and string textField for edges
    crossingParameter = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    myCrossingEdgesLabel = new FXLabel(crossingParameter, toString(SUMO_ATTR_EDGES).c_str(), nullptr, GUIDesignLabelThickedFixed(100));
    myCrossingEdges = new FXTextField(crossingParameter, GUIDesignTextFieldNCol, this, MID_GNE_SET_ATTRIBUTE, GUIDesignTextField);
    myCrossingEdgesLabel->disable();
    myCrossingEdges->disable();
    // create label and checkbox for Priority
    crossingParameter = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    myCrossingPriorityLabel = new FXLabel(crossingParameter, toString(SUMO_ATTR_PRIORITY).c_str(), nullptr, GUIDesignLabelThickedFixed(100));
    myCrossingPriorityCheckButton = new FXCheckButton(crossingParameter, "", this, MID_GNE_SET_ATTRIBUTE, GUIDesignCheckButton);
    myCrossingPriorityLabel->disable();
    myCrossingPriorityCheckButton->disable();
    // create label and textfield for width
    crossingParameter = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    myCrossingWidthLabel = new FXLabel(crossingParameter, toString(SUMO_ATTR_WIDTH).c_str(), nullptr, GUIDesignLabelThickedFixed(100));
    myCrossingWidth = new FXTextField(crossingParameter, GUIDesignTextFieldNCol, this, MID_GNE_SET_ATTRIBUTE, GUIDesignTextField);
    myCrossingWidthLabel->disable();
    myCrossingWidth->disable();
    // Create help button
    myHelpCrossingAttribute = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Help"), "", "", nullptr, this, MID_HELP, GUIDesignButtonRectangular);
    myHelpCrossingAttribute->disable();
}


GNECrossingFrame::CrossingParameters::~CrossingParameters() {
    delete myCrossingTemplate;
}


void
GNECrossingFrame::CrossingParameters::enableCrossingParameters(bool hasTLS) {
    // obtain Tag Values
    const auto& tagProperties = GNEAttributeCarrier::getTagProperty(SUMO_TAG_CROSSING);
    // Enable all elements of the crossing frames
    myCrossingEdgesLabel->enable();
    myCrossingEdges->enable();
    myCrossingPriorityLabel->enable();
    // only enable priority check button if junction's crossing doesn't have TLS
    if (hasTLS) {
        myCrossingPriorityCheckButton->disable();
    } else {
        myCrossingPriorityCheckButton->enable();
    }
    myCrossingWidthLabel->enable();
    myCrossingWidth->enable();
    myHelpCrossingAttribute->enable();
    // set values of parameters
    onCmdSetAttribute(nullptr, 0, nullptr);
    // Crossings placed in junctinos with TLS always has priority
    if (hasTLS) {
        myCrossingPriorityCheckButton->setCheck(TRUE);
    } else {
        myCrossingPriorityCheckButton->setCheck(GNEAttributeCarrier::parse<bool>(tagProperties.getDefaultValue(SUMO_ATTR_PRIORITY)));
    }
    myCrossingWidth->setText(tagProperties.getDefaultValue(SUMO_ATTR_WIDTH).c_str());
    myCrossingWidth->setTextColor(FXRGB(0, 0, 0));
}


void
GNECrossingFrame::CrossingParameters::disableCrossingParameters() {
    // clear all values of parameters
    myCrossingEdges->setText("");
    myCrossingPriorityCheckButton->setCheck(false);
    myCrossingPriorityCheckButton->setText("false");
    myCrossingWidth->setText("");
    // Disable all elements of the crossing frames
    myCrossingEdgesLabel->disable();
    myCrossingEdges->disable();
    myCrossingPriorityLabel->disable();
    myCrossingPriorityCheckButton->disable();
    myCrossingWidthLabel->disable();
    myCrossingWidth->disable();
    myHelpCrossingAttribute->disable();
    myCrossingFrameParent->myCreateCrossing->setCreateCrossingButton(false);
}


bool
GNECrossingFrame::CrossingParameters::isCrossingParametersEnabled() const {
    return myCrossingEdgesLabel->isEnabled();
}


void
GNECrossingFrame::CrossingParameters::markEdge(GNEEdge* edge) {
    GNEJunction* currentJunction = myCrossingFrameParent->myEdgeSelector->getCurrentJunction();
    if (currentJunction != nullptr) {
        // Check if edge belongs to junction's edge
        if (std::find(currentJunction->getChildEdges().begin(), currentJunction->getChildEdges().end(), edge) != currentJunction->getChildEdges().end()) {
            // Update text field with the new edge
            std::vector<std::string> crossingEdges = GNEAttributeCarrier::parse<std::vector<std::string> > (myCrossingEdges->getText().text());
            // Check if new edge must be added or removed
            std::vector<std::string>::iterator itFinder = std::find(crossingEdges.begin(), crossingEdges.end(), edge->getID());
            if (itFinder == crossingEdges.end()) {
                crossingEdges.push_back(edge->getID());
            } else {
                crossingEdges.erase(itFinder);
            }
            myCrossingEdges->setText(joinToString(crossingEdges, " ").c_str());
        }
        // Update colors and attributes
        onCmdSetAttribute(nullptr, 0, nullptr);
    }
}


void
GNECrossingFrame::CrossingParameters::clearEdges() {
    myCrossingEdges->setText("");
    // Update colors and attributes
    onCmdSetAttribute(nullptr, 0, nullptr);
}


void
GNECrossingFrame::CrossingParameters::useSelectedEdges(GNEJunction* parentJunction) {
    std::vector<std::string> crossingEdges;
    for (const auto& edge : parentJunction->getChildEdges()) {
        if (edge->isAttributeCarrierSelected()) {
            crossingEdges.push_back(edge->getID());
        }
    }
    myCrossingEdges->setText(joinToString(crossingEdges, " ").c_str());
    // Update colors and attributes
    onCmdSetAttribute(nullptr, 0, nullptr);
}


std::vector<NBEdge*>
GNECrossingFrame::CrossingParameters::getCrossingEdges() const {
    std::vector<NBEdge*> NBEdgeVector;
    // Iterate over myCurrentSelectedEdges
    for (const auto& edge : myCurrentSelectedEdges) {
        NBEdgeVector.push_back(edge->getNBEdge());
    }
    return NBEdgeVector;
}


bool
GNECrossingFrame::CrossingParameters::getCrossingPriority() const {
    if (myCrossingPriorityCheckButton->getCheck()) {
        return true;
    } else {
        return false;
    }
}


bool
GNECrossingFrame::CrossingParameters::isCurrentParametersValid() const {
    return myCurrentParametersValid;
}


double
GNECrossingFrame::CrossingParameters::getCrossingWidth() const {
    return GNEAttributeCarrier::parse<double>(myCrossingWidth->getText().text());
}


long
GNECrossingFrame::CrossingParameters::onCmdSetAttribute(FXObject*, FXSelector, void*) {
    myCurrentParametersValid = true;
    // get string vector with the edges
    const auto& crossingEdgeIDs = GNEAttributeCarrier::parse<std::vector<std::string> > (myCrossingEdges->getText().text());
    // Clear selected edges
    myCurrentSelectedEdges.clear();
    // iterate over vector of edge IDs
    for (const auto& crossingEdgeID : crossingEdgeIDs) {
        GNEEdge* edge = myCrossingFrameParent->getViewNet()->getNet()->getAttributeCarriers()->retrieveEdge(crossingEdgeID, false);
        GNEJunction* currentJunction = myCrossingFrameParent->myEdgeSelector->getCurrentJunction();
        // Check that edge exists and belongs to Junction
        if (edge == nullptr) {
            myCurrentParametersValid = false;
        } else if (std::find(currentJunction->getChildEdges().begin(), currentJunction->getChildEdges().end(), edge) == currentJunction->getChildEdges().end()) {
            myCurrentParametersValid = false;
        } else {
            // select or unselected edge
            auto itFinder = std::find(myCurrentSelectedEdges.begin(), myCurrentSelectedEdges.end(), edge);
            if (itFinder == myCurrentSelectedEdges.end()) {
                myCurrentSelectedEdges.push_back(edge);
            } else {
                myCurrentSelectedEdges.erase(itFinder);
            }
        }
    }
    // change color of textfield dependig of myCurrentParametersValid
    if (myCurrentParametersValid) {
        myCrossingEdges->setTextColor(FXRGB(0, 0, 0));
        myCrossingEdges->killFocus();
    } else {
        myCrossingEdges->setTextColor(FXRGB(255, 0, 0));
        myCurrentParametersValid = false;
    }
    // Update edge colors
    if (myCurrentSelectedEdges.empty()) {
        for (const auto& edge : myCrossingFrameParent->myEdgeSelector->getCurrentJunction()->getChildEdges()) {
            // restore colors
            edge->resetCandidateFlags();
            // mark all edges as possible candidate
            edge->setPossibleCandidate(true);
        }
    } else {
        EdgeVector selected;
        for (GNEEdge* e : myCurrentSelectedEdges) {
            selected.push_back(e->getNBEdge());
        }
        NBNode* node = myCrossingFrameParent->myEdgeSelector->getCurrentJunction()->getNBNode();
        for (const auto& edge : myCrossingFrameParent->myEdgeSelector->getCurrentJunction()->getChildEdges()) {
            // restore colors
            edge->resetCandidateFlags();
            // set selected or candidate color
            if (std::find(myCurrentSelectedEdges.begin(), myCurrentSelectedEdges.end(), edge) != myCurrentSelectedEdges.end()) {
                edge->setTargetCandidate(true);
            } else {
                EdgeVector newCandidates = selected;;
                newCandidates.push_back(edge->getNBEdge());
                if (node->checkCrossing(newCandidates, true) == 0) {
                    edge->setInvalidCandidate(true);
                } else {
                    edge->setPossibleCandidate(true);
                }
            }
        }
    }
    // Update view net
    myCrossingFrameParent->getViewNet()->updateViewNet();
    // Check that at least there are one selected edge
    if (crossingEdgeIDs.empty()) {
        myCurrentParametersValid = false;
    }
    // change label of crossing priority
    if (myCrossingPriorityCheckButton->getCheck()) {
        myCrossingPriorityCheckButton->setText("true");
    } else {
        myCrossingPriorityCheckButton->setText("false");
    }

    // Check width
    if (GNEAttributeCarrier::canParse<double>(myCrossingWidth->getText().text()) &&
            GNEAttributeCarrier::parse<double>(myCrossingWidth->getText().text()) > 0) {
        myCrossingWidth->setTextColor(FXRGB(0, 0, 0));
        myCrossingWidth->killFocus();
    } else {
        myCrossingWidth->setTextColor(FXRGB(255, 0, 0));
        myCurrentParametersValid = false;
    }

    // Enable or disable create crossing button depending of the current parameters
    myCrossingFrameParent->myCreateCrossing->setCreateCrossingButton(myCurrentParametersValid);
    return 0;
}


long
GNECrossingFrame::CrossingParameters::onCmdHelp(FXObject*, FXSelector, void*) {
    myCrossingFrameParent->openHelpAttributesDialog(myCrossingTemplate);
    return 1;
}

// ---------------------------------------------------------------------------
// GNECrossingFrame::CreateCrossing - methods
// ---------------------------------------------------------------------------

GNECrossingFrame::CreateCrossing::CreateCrossing(GNECrossingFrame* crossingFrameParent) :
    MFXGroupBoxModule(crossingFrameParent, TL("Create")),
    myCrossingFrameParent(crossingFrameParent) {
    // Create groupbox for create crossings
    myCreateCrossingButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Create crossing"), "", "", 0, this, MID_GNE_CREATE, GUIDesignButton);
    myCreateCrossingButton->disable();
}


GNECrossingFrame::CreateCrossing::~CreateCrossing() {}


long
GNECrossingFrame::CreateCrossing::onCmdCreateCrossing(FXObject*, FXSelector, void*) {
    // First check that current parameters are valid
    if (myCrossingFrameParent->myCrossingParameters->isCurrentParametersValid()) {
        // iterate over junction's crossing to find duplicated crossings
        if (myCrossingFrameParent->myEdgeSelector->getCurrentJunction()->getNBNode()->checkCrossingDuplicated(myCrossingFrameParent->myCrossingParameters->getCrossingEdges()) == false) {
            // create new crossing
            myCrossingFrameParent->myViewNet->getUndoList()->add(new GNEChange_Crossing(myCrossingFrameParent->myEdgeSelector->getCurrentJunction(),
                    myCrossingFrameParent->myCrossingParameters->getCrossingEdges(),
                    myCrossingFrameParent->myCrossingParameters->getCrossingWidth(),
                    myCrossingFrameParent->myCrossingParameters->getCrossingPriority(),
                    -1, -1,
                    PositionVector::EMPTY,
                    false, true), true);
            // clear selected edges
            myCrossingFrameParent->myEdgeSelector->onCmdClearSelection(0, 0, 0);
            // update default create edge option
            myCrossingFrameParent->getViewNet()->getViewParent()->getCreateEdgeFrame()->getEdgeTypeSelector()->enableCheckBoxDisablePedestrians();
        } else {
            WRITE_WARNING(TL("There is already another crossing with the same edges in the junction; Duplicated crossing aren't allowed."));
        }
    }
    return 1;
}


void
GNECrossingFrame::CreateCrossing::setCreateCrossingButton(bool value) {
    if (value) {
        myCreateCrossingButton->enable();
    } else {
        myCreateCrossingButton->disable();
    }
}

// ---------------------------------------------------------------------------
// GNECrossingFrame::Legend - methods
// ---------------------------------------------------------------------------

GNECrossingFrame::Information::Information(GNECrossingFrame* crossingFrameParent) :
    MFXGroupBoxModule(crossingFrameParent, TL("Information")) {

    // create label
    new MFXDynamicLabel(getCollapsableFrame(), (std::string("- ") + TL("Click over junction to mark candidate edges.") + std::string("\n- ") + TL("Click over candidate edges for selecting.")).c_str(), 0, GUIDesignLabelFrameInformation);
    // candidate
    FXLabel* colorCandidateLabel = new FXLabel(getCollapsableFrame(), TL(" Candidate"), 0, GUIDesignLabel(JUSTIFY_LEFT));
    colorCandidateLabel->setBackColor(MFXUtils::getFXColor(crossingFrameParent->getViewNet()->getVisualisationSettings().candidateColorSettings.possible));
    colorCandidateLabel->setTextColor(MFXUtils::getFXColor(RGBColor::WHITE));
    // selected
    FXLabel* colorSelectedLabel = new FXLabel(getCollapsableFrame(), TL(" Selected"), 0, GUIDesignLabel(JUSTIFY_LEFT));
    colorSelectedLabel->setBackColor(MFXUtils::getFXColor(crossingFrameParent->getViewNet()->getVisualisationSettings().candidateColorSettings.target));
    // invalid
    FXLabel* colorInvalidLabel = new FXLabel(getCollapsableFrame(), TL(" Invalid"), 0, GUIDesignLabel(JUSTIFY_LEFT));
    colorInvalidLabel->setBackColor(MFXUtils::getFXColor(crossingFrameParent->getViewNet()->getVisualisationSettings().candidateColorSettings.invalid));
    colorInvalidLabel->setTextColor(MFXUtils::getFXColor(RGBColor::WHITE));
}


GNECrossingFrame::Information::~Information() {}

// ---------------------------------------------------------------------------
// GNECrossingFrame - methods
// ---------------------------------------------------------------------------

GNECrossingFrame::GNECrossingFrame(GNEViewParent* viewParent, GNEViewNet* viewNet) :
    GNEFrame(viewParent, viewNet, TL("Crossings")) {
    // create JunctionInformation module
    myJunctionInformation = new JunctionInformation(this);

    // Create edge Selector module
    myEdgeSelector = new EdgesSelector(this);

    // Create CrossingParameters module
    myCrossingParameters = new CrossingParameters(this);

    // create CreateCrossing module
    myCreateCrossing = new CreateCrossing(this);

    // create information module
    myInformation = new Information(this);

    // disable edge selector
    myEdgeSelector->disableEdgeSelector();
}


GNECrossingFrame::~GNECrossingFrame() {
}


void
GNECrossingFrame::hide() {
    // restore color of all lanes of edge candidates
    myEdgeSelector->restoreEdgeColors();
    // hide frame
    GNEFrame::hide();
}


void
GNECrossingFrame::addCrossing(const GNEViewNetHelper::ViewObjectsSelector& viewObjects) {
    // If current element is a junction
    if (viewObjects.getJunctionFront()) {
        // change label
        myJunctionInformation->updateCurrentJunctionLabel(viewObjects.getJunctionFront()->getID());
        // Enable edge selector and crossing parameters
        myEdgeSelector->enableEdgeSelector(viewObjects.getJunctionFront());
        myCrossingParameters->enableCrossingParameters(viewObjects.getJunctionFront()->getNBNode()->isTLControlled());
        // clears selected edges
        myCrossingParameters->clearEdges();
    } else if (viewObjects.getEdgeFront()) {
        // check if mark edge
        if (!viewObjects.getEdgeFront()->isInvalidCandidate()) {
            myCrossingParameters->markEdge(viewObjects.getEdgeFront());
        }
    } else {
        // set default label
        myJunctionInformation->updateCurrentJunctionLabel("");
        // restore  color of all lanes of edge candidates
        myEdgeSelector->restoreEdgeColors();
        // Disable edge selector
        myEdgeSelector->disableEdgeSelector();
    }
    // always update view after an operation
    myViewNet->updateViewNet();
}


void
GNECrossingFrame::createCrossingHotkey() {
    if (myEdgeSelector->getCurrentJunction()) {
        // simply call onCmdCreateCrossing of CreateCrossing module
        myCreateCrossing->onCmdCreateCrossing(0, 0, 0);
    }
}


void
GNECrossingFrame::clearEdgesHotkey() {
    if (myCrossingParameters->getCrossingEdges().size() > 0) {
        myCrossingParameters->clearEdges();
    } else if (myEdgeSelector->getCurrentJunction()) {
        myEdgeSelector->restoreEdgeColors();
        myEdgeSelector->disableEdgeSelector();
    }
}

/****************************************************************************/
