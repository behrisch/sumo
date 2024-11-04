/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2011-2024 German Aerospace Center (DLR) and others.
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
/// @file    GNEConnectorFrame.cpp
/// @author  Jakob Erdmann
/// @date    May 2011
///
// The Widget for modifying lane-to-lane connections
/****************************************************************************/
#include <config.h>

#include <utils/foxtools/MFXDynamicLabel.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/div/GUIDesigns.h>
#include <netedit/changes/GNEChange_Connection.h>
#include <netedit/GNEViewParent.h>
#include <netedit/GNEUndoList.h>
#include <netedit/GNENet.h>
#include <netedit/GNEViewNet.h>
#include <netedit/elements/network/GNEConnection.h>
#include <netedit/frames/network/GNEConnectorFrame.h>
#include <netedit/frames/common/GNESelectorFrame.h>


// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNEConnectorFrame::ConnectionModifications) ConnectionModificationsMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_CANCEL,     GNEConnectorFrame::ConnectionModifications::onCmdCancelModifications),
    FXMAPFUNC(SEL_COMMAND,  MID_OK,         GNEConnectorFrame::ConnectionModifications::onCmdSaveModifications),
};

FXDEFMAP(GNEConnectorFrame::ConnectionOperations) ConnectionOperationsMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_CHOOSEN_CLEAR,                          GNEConnectorFrame::ConnectionOperations::onCmdClearSelectedConnections),
    FXMAPFUNC(SEL_COMMAND,  MID_CHOOSEN_RESET,                          GNEConnectorFrame::ConnectionOperations::onCmdResetSelectedConnections),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_CONNECTORFRAME_SELECTDEADENDS,      GNEConnectorFrame::ConnectionOperations::onCmdSelectDeadEnds),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_CONNECTORFRAME_SELECTDEADSTARTS,    GNEConnectorFrame::ConnectionOperations::onCmdSelectDeadStarts),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_CONNECTORFRAME_SELECTCONFLICTS,     GNEConnectorFrame::ConnectionOperations::onCmdSelectConflicts),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_CONNECTORFRAME_SELECTPASS,          GNEConnectorFrame::ConnectionOperations::onCmdSelectPass),
};

// Object implementation
FXIMPLEMENT(GNEConnectorFrame::ConnectionModifications, MFXGroupBoxModule, ConnectionModificationsMap, ARRAYNUMBER(ConnectionModificationsMap))
FXIMPLEMENT(GNEConnectorFrame::ConnectionOperations,    MFXGroupBoxModule, ConnectionOperationsMap,    ARRAYNUMBER(ConnectionOperationsMap))


// ===========================================================================
// method definitions
// ===========================================================================

// ---------------------------------------------------------------------------
// GNEConnectorFrame::CurrentLane - methods
// ---------------------------------------------------------------------------

GNEConnectorFrame::CurrentLane::CurrentLane(GNEConnectorFrame* connectorFrameParent) :
    MFXGroupBoxModule(connectorFrameParent, TL("Lane")) {
    // create lane label
    myCurrentLaneLabel = new FXLabel(getCollapsableFrame(), TL("No lane selected"), 0, GUIDesignLabel(JUSTIFY_LEFT));
}


GNEConnectorFrame::CurrentLane::~CurrentLane() {}


void
GNEConnectorFrame::CurrentLane::updateCurrentLaneLabel(const std::string& laneID) {
    if (laneID.empty()) {
        myCurrentLaneLabel->setText(TL("No lane selected"));
    } else {
        myCurrentLaneLabel->setText((std::string(TL("Current Lane: ")) + laneID).c_str());
    }
}

// ---------------------------------------------------------------------------
// GNEConnectorFrame::ConnectionModifications - methods
// ---------------------------------------------------------------------------

GNEConnectorFrame::ConnectionModifications::ConnectionModifications(GNEConnectorFrame* connectorFrameParent) :
    MFXGroupBoxModule(connectorFrameParent, TL("Modifications")),
    myConnectorFrameParent(connectorFrameParent) {

    // Create "Cancel" button
    myCancelButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Cancel"), "", TL("Discard connection modifications (Esc)"),
                     GUIIconSubSys::getIcon(GUIIcon::CANCEL), this, MID_CANCEL, GUIDesignButton);
    // Create "OK" button
    mySaveButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("OK"), "", TL("Save connection modifications (Enter)"),
                   GUIIconSubSys::getIcon(GUIIcon::ACCEPT), this, MID_OK, GUIDesignButton);

    // Create checkbox for protect routes
    myProtectRoutesCheckBox = new FXCheckButton(getCollapsableFrame(), TL("Protect routes"), this, MID_GNE_SET_ATTRIBUTE, GUIDesignCheckButton);
}


GNEConnectorFrame::ConnectionModifications::~ConnectionModifications() {}


long
GNEConnectorFrame::ConnectionModifications::onCmdCancelModifications(FXObject*, FXSelector, void*) {
    if (myConnectorFrameParent->myCurrentEditedLane != 0) {
        myConnectorFrameParent->getViewNet()->getUndoList()->abortAllChangeGroups();
        if (myConnectorFrameParent->myNumChanges) {
            myConnectorFrameParent->getViewNet()->setStatusBarText(TL("Changes reverted"));
        }
        myConnectorFrameParent->cleanup();
        myConnectorFrameParent->getViewNet()->updateViewNet();
    }
    return 1;
}


long
GNEConnectorFrame::ConnectionModifications::onCmdSaveModifications(FXObject*, FXSelector, void*) {
    if (myConnectorFrameParent->myCurrentEditedLane != 0) {
        // check if routes has to be protected
        if (myProtectRoutesCheckBox->isEnabled() && (myProtectRoutesCheckBox->getCheck() == TRUE)) {
            for (const auto& demandElement : myConnectorFrameParent->myCurrentEditedLane->getParentEdge()->getChildDemandElements()) {
                if (demandElement->isDemandElementValid() != GNEDemandElement::Problem::OK) {
                    FXMessageBox::warning(getApp(), MBOX_OK,
                                          TL("Error saving connection operations"), "%s",
                                          (TLF("Connection edition cannot be saved because route '%' is broken.", demandElement->getID()).c_str()));
                    return 1;
                }
            }
        }
        // finish route editing
        myConnectorFrameParent->getViewNet()->getUndoList()->end();
        if (myConnectorFrameParent->myNumChanges) {
            myConnectorFrameParent->getViewNet()->setStatusBarText(TL("Changes accepted"));
        }
        myConnectorFrameParent->cleanup();
        // mark network for recomputing
        myConnectorFrameParent->getViewNet()->getNet()->requireRecompute();
    }
    return 1;
}

// ---------------------------------------------------------------------------
// GNEConnectorFrame::ConnectionOperations - methods
// ---------------------------------------------------------------------------

GNEConnectorFrame::ConnectionOperations::ConnectionOperations(GNEConnectorFrame* connectorFrameParent) :
    MFXGroupBoxModule(connectorFrameParent, TL("Operations")),
    myConnectorFrameParent(connectorFrameParent) {

    // Create "Select Dead Ends" button
    mySelectDeadEndsButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Select Dead Ends"), "", TL("Selects all lanes that have no outgoing connection (clears previous selection)"),
                             0, this, MID_GNE_CONNECTORFRAME_SELECTDEADENDS, GUIDesignButton);
    // Create "Select Dead Starts" button
    mySelectDeadStartsButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Select Dead Starts"), "", TL("Selects all lanes that have no incoming connection (clears previous selection)"),
                               0, this, MID_GNE_CONNECTORFRAME_SELECTDEADSTARTS, GUIDesignButton);
    // Create "Select Conflicts" button
    mySelectConflictsButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Select Conflicts"), "", TL("Selects all lanes with more than one incoming connection from the same edge (clears previous selection)"),
                              0, this, MID_GNE_CONNECTORFRAME_SELECTCONFLICTS, GUIDesignButton);
    // Create "Select Edges which may always pass" button
    mySelectPassingButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Select Passing"), "", TL("Selects all lanes with a connection that has the 'pass' attribute set"),
                            0, this, MID_GNE_CONNECTORFRAME_SELECTPASS, GUIDesignButton);
    // Create "Clear Selected" button
    myClearSelectedButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Clear Selected"), "", TL("Clears all connections of all selected objects"),
                            0, this, MID_CHOOSEN_CLEAR, GUIDesignButton);
    // Create "Reset Selected" button
    myResetSelectedButton = GUIDesigns::buildFXButton(getCollapsableFrame(), TL("Reset Selected"), "", TL("Recomputes connections at all selected junctions"),
                            0, this, MID_CHOOSEN_RESET, GUIDesignButton);
}


GNEConnectorFrame::ConnectionOperations::~ConnectionOperations() {}


long
GNEConnectorFrame::ConnectionOperations::onCmdSelectDeadEnds(FXObject*, FXSelector, void*) {
    // select all lanes that have no successor lane
    std::vector<GNEAttributeCarrier*> deadEnds;
    // every edge knows its outgoing connections so we can look at each edge in isolation
    for (const auto& edge : myConnectorFrameParent->getViewNet()->getNet()->getAttributeCarriers()->getEdges()) {
        for (const auto& lane : edge.second->getLanes()) {
            if (edge.second->getNBEdge()->getConnectionsFromLane(lane->getIndex()).size() == 0) {
                deadEnds.push_back(lane);
            }
        }
    }
    myConnectorFrameParent->getViewNet()->getViewParent()->getSelectorFrame()->handleIDs(deadEnds, GNESelectorFrame::ModificationMode::Operation::REPLACE);
    return 1;
}


long
GNEConnectorFrame::ConnectionOperations::onCmdSelectDeadStarts(FXObject*, FXSelector, void*) {
    // select all lanes that have no predecessor lane
    std::set<GNEAttributeCarrier*> deadStarts;
    GNENet* net = myConnectorFrameParent->getViewNet()->getNet();
    // every edge knows only its outgoing connections so we look at whole junctions
    for (const auto& junction : myConnectorFrameParent->getViewNet()->getNet()->getAttributeCarriers()->getJunctions()) {
        // first collect all outgoing lanes
        for (const auto& outgoingEdge : junction.second->getGNEOutgoingEdges()) {
            for (const auto& lane : outgoingEdge->getLanes()) {
                deadStarts.insert(lane);
            }
        }
        // then remove all approached lanes
        for (const auto& incomingEdge : junction.second->getGNEIncomingEdges()) {
            for (const auto& connection : incomingEdge->getNBEdge()->getConnections()) {
                deadStarts.erase(net->getAttributeCarriers()->retrieveEdge(connection.toEdge->getID())->getLanes()[connection.toLane]);
            }
        }
    }
    std::vector<GNEAttributeCarrier*> selectObjects(deadStarts.begin(), deadStarts.end());
    myConnectorFrameParent->getViewNet()->getViewParent()->getSelectorFrame()->handleIDs(selectObjects, GNESelectorFrame::ModificationMode::Operation::REPLACE);
    return 1;
}


long
GNEConnectorFrame::ConnectionOperations::onCmdSelectConflicts(FXObject*, FXSelector, void*) {
    std::vector<GNEAttributeCarrier*> conflicts;
    // conflicts happen per edge so we can look at each edge in isolation
    for (const auto& edge : myConnectorFrameParent->getViewNet()->getNet()->getAttributeCarriers()->getEdges()) {
        const EdgeVector destinations = edge.second->getNBEdge()->getConnectedEdges();
        for (const auto& destination : destinations) {
            GNEEdge* dest = myConnectorFrameParent->getViewNet()->getNet()->getAttributeCarriers()->retrieveEdge(destination->getID());
            for (const auto& lane : dest->getLanes()) {
                const bool isConflicted = count_if(edge.second->getNBEdge()->getConnections().begin(), edge.second->getNBEdge()->getConnections().end(),
                                                   NBEdge::connections_toedgelane_finder(destination, (int)lane->getIndex(), -1)) > 1;
                if (isConflicted) {
                    conflicts.push_back(lane);
                }
            }
        }

    }
    myConnectorFrameParent->getViewNet()->getViewParent()->getSelectorFrame()->handleIDs(conflicts, GNESelectorFrame::ModificationMode::Operation::REPLACE);
    return 1;
}


long
GNEConnectorFrame::ConnectionOperations::onCmdSelectPass(FXObject*, FXSelector, void*) {
    std::vector<GNEAttributeCarrier*> pass;
    for (const auto& edge : myConnectorFrameParent->getViewNet()->getNet()->getAttributeCarriers()->getEdges()) {
        for (const auto& connection : edge.second->getNBEdge()->getConnections()) {
            if (connection.mayDefinitelyPass) {
                pass.push_back(edge.second->getLanes()[connection.fromLane]);
            }
        }
    }
    myConnectorFrameParent->getViewNet()->getViewParent()->getSelectorFrame()->handleIDs(pass, GNESelectorFrame::ModificationMode::Operation::REPLACE);
    return 1;
}


long
GNEConnectorFrame::ConnectionOperations::onCmdClearSelectedConnections(FXObject*, FXSelector, void*) {
    myConnectorFrameParent->myConnectionModifications->onCmdCancelModifications(0, 0, 0);
    myConnectorFrameParent->getViewNet()->getUndoList()->begin(GUIIcon::CONNECTION, TL("clear connections from selected lanes, edges and junctions"));
    // clear junction's connection
    const auto selectedJunctions = myConnectorFrameParent->getViewNet()->getNet()->getAttributeCarriers()->getSelectedJunctions();
    for (const auto& junction : selectedJunctions) {
        junction->setLogicValid(false, myConnectorFrameParent->getViewNet()->getUndoList()); // clear connections
        junction->setLogicValid(false, myConnectorFrameParent->getViewNet()->getUndoList(), GNEAttributeCarrier::FEATURE_MODIFIED); // prevent re-guessing
    }
    // clear edge's connection
    const auto selectedEdges = myConnectorFrameParent->getViewNet()->getNet()->getAttributeCarriers()->getSelectedEdges();
    for (const auto& edge : selectedEdges) {
        for (const auto& lane : edge->getLanes()) {
            myConnectorFrameParent->removeConnections(lane);
        }
    }
    // clear lane's connection
    const auto selectedLanes = myConnectorFrameParent->getViewNet()->getNet()->getAttributeCarriers()->getSelectedLanes();
    for (const auto& lane : selectedLanes) {
        myConnectorFrameParent->removeConnections(lane);
    }
    myConnectorFrameParent->getViewNet()->getUndoList()->end();
    return 1;
}


long
GNEConnectorFrame::ConnectionOperations::onCmdResetSelectedConnections(FXObject*, FXSelector, void*) {
    myConnectorFrameParent->myConnectionModifications->onCmdCancelModifications(0, 0, 0);
    myConnectorFrameParent->getViewNet()->getUndoList()->begin(GUIIcon::CONNECTION, TL("reset connections from selected lanes"));
    const auto selectedJunctions = myConnectorFrameParent->getViewNet()->getNet()->getAttributeCarriers()->getSelectedJunctions();
    for (const auto& junction : selectedJunctions) {
        junction->setLogicValid(false, myConnectorFrameParent->getViewNet()->getUndoList());
    }
    myConnectorFrameParent->getViewNet()->getUndoList()->end();
    if (selectedJunctions.size() > 0) {
        auto viewNet = myConnectorFrameParent->getViewNet();
        viewNet->getNet()->requireRecompute();
        viewNet->getNet()->computeNetwork(viewNet->getViewParent()->getGNEAppWindows());
    }
    return 1;
}

// ---------------------------------------------------------------------------
// GNEConnectorFrame::ConnectionSelection - methods
// ---------------------------------------------------------------------------

GNEConnectorFrame::ConnectionSelection::ConnectionSelection(GNEConnectorFrame* connectorFrameParent) :
    MFXGroupBoxModule(connectorFrameParent, TL("Selection")) {
    // create label
    new MFXDynamicLabel(getCollapsableFrame(), (std::string("- ") + TL("Hold <SHIFT> while clicking to create unyielding connections (pass=true).")).c_str(), 0, GUIDesignLabelFrameInformation);
    new MFXDynamicLabel(getCollapsableFrame(), (std::string("- ") + TL("Hold <CTRL> while clicking to create conflicting connections (i.e. at zipper nodes or with incompatible permissions)")).c_str(), 0, GUIDesignLabelFrameInformation);
}


GNEConnectorFrame::ConnectionSelection::~ConnectionSelection() {}

// ---------------------------------------------------------------------------
// GNEConnectorFrame::ConnectionLegend - methods
// ---------------------------------------------------------------------------

GNEConnectorFrame::Legend::Legend(GNEConnectorFrame* connectorFrameParent) :
    MFXGroupBoxModule(connectorFrameParent, TL("Information")) {

    // create possible target label
    FXLabel* possibleTargetLabel = new FXLabel(getCollapsableFrame(), TL("Possible Target"), 0, GUIDesignLabel(JUSTIFY_LEFT));
    possibleTargetLabel->setBackColor(MFXUtils::getFXColor(connectorFrameParent->getViewNet()->getVisualisationSettings().candidateColorSettings.possible));
    possibleTargetLabel->setTextColor(MFXUtils::getFXColor(RGBColor::WHITE));

    // create source label
    FXLabel* sourceLabel = new FXLabel(getCollapsableFrame(), TL("Source lane"), 0, GUIDesignLabel(JUSTIFY_LEFT));
    sourceLabel->setBackColor(MFXUtils::getFXColor(connectorFrameParent->getViewNet()->getVisualisationSettings().candidateColorSettings.source));

    // create target label
    FXLabel* targetLabel = new FXLabel(getCollapsableFrame(), TL("Target lane"), 0, GUIDesignLabel(JUSTIFY_LEFT));
    targetLabel->setBackColor(MFXUtils::getFXColor(connectorFrameParent->getViewNet()->getVisualisationSettings().candidateColorSettings.target));

    // create target (pass) label
    FXLabel* targetPassLabel = new FXLabel(getCollapsableFrame(), TL("Target (pass)"), 0, GUIDesignLabel(JUSTIFY_LEFT));
    targetPassLabel->setBackColor(MFXUtils::getFXColor(connectorFrameParent->getViewNet()->getVisualisationSettings().candidateColorSettings.special));

    // create conflict label
    FXLabel* conflictLabel = new FXLabel(getCollapsableFrame(), TL("Conflict"), 0, GUIDesignLabel(JUSTIFY_LEFT));
    conflictLabel->setBackColor(MFXUtils::getFXColor(connectorFrameParent->getViewNet()->getVisualisationSettings().candidateColorSettings.conflict));
}


GNEConnectorFrame::Legend::~Legend() {}

// ---------------------------------------------------------------------------
// GNEConnectorFrame - methods
// ---------------------------------------------------------------------------

GNEConnectorFrame::GNEConnectorFrame(GNEViewParent* viewParent, GNEViewNet* viewNet):
    GNEFrame(viewParent, viewNet, TL("Edit Connections")),
    myCurrentEditedLane(0),
    myNumChanges(0) {
    // create current lane module
    myCurrentLane = new CurrentLane(this);

    // create connection modifications module
    myConnectionModifications = new ConnectionModifications(this);

    // create connection operations module
    myConnectionOperations = new ConnectionOperations(this);

    // create connection selection module
    myConnectionSelection = new ConnectionSelection(this);

    // create connection legend module
    myLegend = new Legend(this);
}


GNEConnectorFrame::~GNEConnectorFrame() {}


void
GNEConnectorFrame::handleLaneClick(const GNEViewNetHelper::ViewObjectsSelector& viewObjects) {
    // get lane front
    GNELane* clickedLane = viewObjects.getLaneFrontNonLocked();
    // iterate over lanes
    for (const auto& lane : viewObjects.getLanes()) {
        // if parent edge of lane is front element, update clickedLane
        if (lane->getParentEdge() == myViewNet->getFrontAttributeCarrier()) {
            clickedLane = lane;
        }
    }
    // build connection
    buildConnection(clickedLane, myViewNet->getMouseButtonKeyPressed().shiftKeyPressed(), myViewNet->getMouseButtonKeyPressed().controlKeyPressed(), true);
}


GNEConnectorFrame::ConnectionModifications*
GNEConnectorFrame::getConnectionModifications() const {
    return myConnectionModifications;
}


void
GNEConnectorFrame::removeConnections(GNELane* lane) {
    // select lane as current lane
    buildConnection(lane, false, false, true); // select as current lane
    // iterate over all potential targets
    for (const auto& potentialTarget : myPotentialTargets) {
        // remove connections using the appropiate parameters in function "buildConnection"
        buildConnection(potentialTarget, false, false, false);
    }
    // save modifications
    myConnectionModifications->onCmdSaveModifications(0, 0, 0);
}


void
GNEConnectorFrame::buildConnection(GNELane* lane, const bool mayDefinitelyPass, const bool allowConflict, const bool toggle) {
    if (myCurrentEditedLane == 0) {
        myCurrentEditedLane = lane;
        myCurrentEditedLane->setSpecialColor(&myViewNet->getVisualisationSettings().candidateColorSettings.source);
        initTargets();
        myNumChanges = 0;
        myViewNet->getUndoList()->begin(GUIIcon::CONNECTION, TL("modify connections"));
    } else if (myPotentialTargets.count(lane)
               || (allowConflict && lane->getParentEdge()->getFromJunction() == myCurrentEditedLane->getParentEdge()->getToJunction())) {
        const int fromIndex = myCurrentEditedLane->getIndex();
        GNEEdge* srcEdge = myCurrentEditedLane->getParentEdge();
        GNEEdge* destEdge = lane->getParentEdge();
        std::vector<NBEdge::Connection> connections = srcEdge->getNBEdge()->getConnectionsFromLane(fromIndex);
        bool changed = false;
        // get lane status
        LaneStatus status = getLaneStatus(connections, lane);
        if (status == LaneStatus::CONFLICTED && allowConflict) {
            status = LaneStatus::UNCONNECTED;
        }
        // create depending of status
        switch (status) {
            case LaneStatus::UNCONNECTED:
                if (toggle) {
                    // create new connection
                    NBEdge::Connection newCon(fromIndex, destEdge->getNBEdge(), lane->getIndex(), mayDefinitelyPass);
                    // if the connection was previously deleted (by clicking the same lane twice), restore all values
                    for (NBEdge::Connection& c : myDeletedConnections) {
                        // fromLane must be the same, only check toLane
                        if (c.toEdge == destEdge->getNBEdge() && c.toLane == lane->getIndex()) {
                            newCon = c;
                            newCon.mayDefinitelyPass = mayDefinitelyPass;
                        }
                    }
                    NBConnection newNBCon(srcEdge->getNBEdge(), fromIndex, destEdge->getNBEdge(), lane->getIndex(), newCon.tlLinkIndex);
                    myViewNet->getUndoList()->add(new GNEChange_Connection(srcEdge, newCon, false, true), true);
                    if (mayDefinitelyPass) {
                        lane->setSpecialColor(&myViewNet->getVisualisationSettings().candidateColorSettings.special);
                    } else {
                        lane->setSpecialColor(&myViewNet->getVisualisationSettings().candidateColorSettings.target);
                    }
                    srcEdge->getToJunction()->invalidateTLS(myViewNet->getUndoList(), NBConnection::InvalidConnection, newNBCon);
                }
                break;
            case LaneStatus::CONNECTED:
            case LaneStatus::CONNECTED_PASS: {
                // remove connection
                GNEConnection* con = srcEdge->retrieveGNEConnection(fromIndex, destEdge->getNBEdge(), lane->getIndex());
                myDeletedConnections.push_back(con->getNBEdgeConnection());
                myViewNet->getNet()->deleteConnection(con, myViewNet->getUndoList());
                lane->setSpecialColor(&myViewNet->getVisualisationSettings().candidateColorSettings.possible);
                changed = true;
                break;
            }
            case LaneStatus::CONFLICTED:
                SVCPermissions fromPermissions = srcEdge->getNBEdge()->getPermissions(fromIndex);
                SVCPermissions toPermissions = destEdge->getNBEdge()->getPermissions(lane->getIndex());
                if ((fromPermissions & toPermissions) == SVC_PEDESTRIAN) {
                    myViewNet->setStatusBarText(TL("Pedestrian connections are generated automatically"));
                } else if ((fromPermissions & toPermissions) == 0) {
                    myViewNet->setStatusBarText(TL("Incompatible vehicle class permissions"));
                } else {
                    myViewNet->setStatusBarText(TL("Another lane from the same edge already connects to that lane"));
                }
                break;
        }
        if (changed) {
            myNumChanges += 1;
        }
    } else {
        myViewNet->setStatusBarText(TL("Invalid target for connection"));
    }
    myCurrentLane->updateCurrentLaneLabel(myCurrentEditedLane->getID());
}


void
GNEConnectorFrame::initTargets() {
    // gather potential targets
    NBNode* nbn = myCurrentEditedLane->getParentEdge()->getToJunction()->getNBNode();
    // get potential targets
    for (const auto& NBEEdge : nbn->getOutgoingEdges()) {
        GNEEdge* edge = myViewNet->getNet()->getAttributeCarriers()->retrieveEdge(NBEEdge->getID());
        for (const auto& lane : edge->getLanes()) {
            myPotentialTargets.insert(lane);
        }
    }
    // set color for existing connections
    std::vector<NBEdge::Connection> connections = myCurrentEditedLane->getParentEdge()->getNBEdge()->getConnectionsFromLane(myCurrentEditedLane->getIndex());
    for (const auto& lane : myPotentialTargets) {
        switch (getLaneStatus(connections, lane)) {
            case LaneStatus::CONNECTED:
                lane->setSpecialColor(&myViewNet->getVisualisationSettings().candidateColorSettings.target);
                break;
            case LaneStatus::CONNECTED_PASS:
                lane->setSpecialColor(&myViewNet->getVisualisationSettings().candidateColorSettings.special);
                break;
            case LaneStatus::CONFLICTED:
                lane->setSpecialColor(&myViewNet->getVisualisationSettings().candidateColorSettings.conflict);
                break;
            case LaneStatus::UNCONNECTED:
                lane->setSpecialColor(&myViewNet->getVisualisationSettings().candidateColorSettings.possible);
                break;
        }
    }
}


void
GNEConnectorFrame::cleanup() {
    // restore colors of potential targets
    for (auto it : myPotentialTargets) {
        it->setSpecialColor(0);
    }
    // clear attributes
    myPotentialTargets.clear();
    myNumChanges = 0;
    myCurrentEditedLane->setSpecialColor(0);
    myCurrentEditedLane = nullptr;
    myDeletedConnections.clear();
    myCurrentLane->updateCurrentLaneLabel("");
}


GNEConnectorFrame::LaneStatus
GNEConnectorFrame::getLaneStatus(const std::vector<NBEdge::Connection>& connections, const GNELane* targetLane) const {
    NBEdge* srcEdge = myCurrentEditedLane->getParentEdge()->getNBEdge();
    const int fromIndex = myCurrentEditedLane->getIndex();
    NBEdge* destEdge = targetLane->getParentEdge()->getNBEdge();
    const int toIndex = targetLane->getIndex();
    std::vector<NBEdge::Connection>::const_iterator con_it = find_if(
                connections.begin(), connections.end(),
                NBEdge::connections_finder(fromIndex, destEdge, toIndex));
    const bool isConnected = con_it != connections.end();
    if (isConnected) {
        if (con_it->mayDefinitelyPass) {
            return LaneStatus::CONNECTED_PASS;
        } else {
            return LaneStatus::CONNECTED;
        }
    } else if (srcEdge->hasConnectionTo(destEdge, toIndex)
               || (srcEdge->getPermissions(fromIndex) & destEdge->getPermissions(toIndex) & ~SVC_PEDESTRIAN) == 0) {
        return LaneStatus::CONFLICTED;
    } else {
        return LaneStatus::UNCONNECTED;
    }
}


/****************************************************************************/
