/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2001-2025 German Aerospace Center (DLR) and others.
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
/// @file    GNETLSEditorFrame.cpp
/// @author  Jakob Erdmann
/// @date    May 2011
///
// The Widget for modifying traffic lights
/****************************************************************************/

#include <netbuild/NBLoadedSUMOTLDef.h>
#include <netbuild/NBOwnTLDef.h>
#include <netedit/GNEApplicationWindow.h>
#include <netedit/GNENet.h>
#include <netedit/GNETagProperties.h>
#include <netedit/GNEUndoList.h>
#include <netedit/GNEInternalTest.h>
#include <netedit/GNEViewParent.h>
#include <netedit/changes/GNEChange_TLS.h>
#include <netedit/dialogs/GNESingleParametersDialog.h>
#include <netedit/elements/network/GNEConnection.h>
#include <netedit/elements/network/GNECrossing.h>
#include <netedit/elements/network/GNEInternalLane.h>
#include <netedit/frames/GNEOverlappedInspection.h>
#include <netedit/frames/GNETLSTable.h>
#include <netimport/NIXMLTrafficLightsHandler.h>
#include <netwrite/NWWriter_SUMO.h>
#include <utils/foxtools/MFXTextFieldTooltip.h>
#include <utils/foxtools/MFXToggleButtonTooltip.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/xml/XMLSubSys.h>

#include "GNETLSEditorFrame.h"

// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNETLSEditorFrame::TLSJunction) TLSJunctionMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_TLSJUNCTION_ID,            GNETLSEditorFrame::TLSJunction::onCmdRenameTLS),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_TLSJUNCTION_TYPE,          GNETLSEditorFrame::TLSJunction::onCmdChangeType),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_TLSJUNCTION_TOGGLEJOIN,    GNETLSEditorFrame::TLSJunction::onCmdToggleJoinTLS),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_TLSJUNCTION_DISJOIN,       GNETLSEditorFrame::TLSJunction::onCmdDisjoinTLS),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_BUTTON_ACCEPT,                      GNETLSEditorFrame::TLSJunction::onCmdAcceptJoin),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_BUTTON_CANCEL,                      GNETLSEditorFrame::TLSJunction::onCmdCancelJoin)
};

FXDEFMAP(GNETLSEditorFrame::TLSPrograms) TLSProgramsMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_DEFINITION_CREATE,         GNETLSEditorFrame::TLSPrograms::onCmdCreate),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_DEFINITION_DELETE,         GNETLSEditorFrame::TLSPrograms::onCmdDelete),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_DEFINITION_RESETCURRENT,   GNETLSEditorFrame::TLSPrograms::onCmdResetCurrentProgram),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_DEFINITION_RESETALL,       GNETLSEditorFrame::TLSPrograms::onCmdResetAll),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_DEFINITION_SWITCHPROGRAM,  GNETLSEditorFrame::TLSPrograms::onCmdDefSwitchTLSProgram),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_DEFINITION_SAVE,           GNETLSEditorFrame::TLSPrograms::onCmdSaveChanges),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_DEFINITION_DISCARD,        GNETLSEditorFrame::TLSPrograms::onCmdDiscardChanges),
};

FXDEFMAP(GNETLSEditorFrame::TLSAttributes) TLSAttributesMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_ATTRIBUTES_OFFSET,             GNETLSEditorFrame::TLSAttributes::onCmdSetOffset),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_ATTRIBUTES_PARAMETERS,         GNETLSEditorFrame::TLSAttributes::onCmdSetParameters),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_ATTRIBUTES_PARAMETERSDIALOG,   GNETLSEditorFrame::TLSAttributes::onCmdParametersDialog),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_ATTRIBUTES_TOGGLEDETECTOR,     GNETLSEditorFrame::TLSAttributes::onCmdToggleDetectorMode),
};

FXDEFMAP(GNETLSEditorFrame::TLSPhases) TLSPhasesMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_PHASES_CLEANUP,        GNETLSEditorFrame::TLSPhases::onCmdCleanStates),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_PHASES_ADDUNUSED,      GNETLSEditorFrame::TLSPhases::onCmdAddUnusedStates),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_PHASES_GROUPSTATES,    GNETLSEditorFrame::TLSPhases::onCmdGroupStates),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_PHASES_UNGROUPSTATES,  GNETLSEditorFrame::TLSPhases::onCmdUngroupStates),
};

FXDEFMAP(GNETLSEditorFrame::TLSFile) TLSFileMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_FILE_LOADPROGRAM,  GNETLSEditorFrame::TLSFile::onCmdLoadTLSProgram),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSFRAME_FILE_SAVEPROGRAM,  GNETLSEditorFrame::TLSFile::onCmdSaveTLSProgram),
};

// Object implementation
FXIMPLEMENT(GNETLSEditorFrame::TLSJunction,     MFXGroupBoxModule,  TLSJunctionMap,     ARRAYNUMBER(TLSJunctionMap))
FXIMPLEMENT(GNETLSEditorFrame::TLSPrograms,     MFXGroupBoxModule,  TLSProgramsMap,     ARRAYNUMBER(TLSProgramsMap))
FXIMPLEMENT(GNETLSEditorFrame::TLSAttributes,   MFXGroupBoxModule,  TLSAttributesMap,   ARRAYNUMBER(TLSAttributesMap))
FXIMPLEMENT(GNETLSEditorFrame::TLSPhases,       MFXGroupBoxModule,  TLSPhasesMap,       ARRAYNUMBER(TLSPhasesMap))
FXIMPLEMENT(GNETLSEditorFrame::TLSFile,         MFXGroupBoxModule,  TLSFileMap,         ARRAYNUMBER(TLSFileMap))


// ===========================================================================
// method definitions
// ===========================================================================

GNETLSEditorFrame::GNETLSEditorFrame(GNEViewParent* viewParent, GNEViewNet* viewNet):
    GNEFrame(viewParent, viewNet, TL("Edit Traffic Light")),
    myEditedDef(nullptr) {

    // Create Overlapped Inspection module only for junctions
    myOverlappedInspection = new GNEOverlappedInspection(this, true);

    // create TLSJunction module
    myTLSJunction = new GNETLSEditorFrame::TLSJunction(this);

    // create TLSPrograms module
    myTLSPrograms = new GNETLSEditorFrame::TLSPrograms(this);

    // create TLSAttributes module
    myTLSAttributes = new GNETLSEditorFrame::TLSAttributes(this);

    // create TLSPhases module
    myTLSPhases = new GNETLSEditorFrame::TLSPhases(this);

    // create TLSFile module
    myTLSFile = new GNETLSEditorFrame::TLSFile(this);
}


GNETLSEditorFrame::~GNETLSEditorFrame() {
    myTLSPhases->clearPhaseTable();
}


void
GNETLSEditorFrame::show() {
    myOverlappedInspection->clearOverlappedInspection();
    // show
    GNEFrame::show();
}


void
GNETLSEditorFrame::frameWidthUpdated() {
    // recalc table width
    myTLSPhases->getPhaseTable()->recalcTableWidth();
}


void
GNETLSEditorFrame::updateModules() {
    if (myTLSJunction) {
        myTLSJunction->updateTLSJunction();
    }
    if (myTLSPrograms) {
        myTLSPrograms->updateTLSPrograms();
    }
    if (myTLSAttributes) {
        myTLSAttributes->updateTLSAttributes();
    }
    if (myTLSPhases) {
        myTLSPhases->updateTLSPhases();
    }
    update();
}


void
GNETLSEditorFrame::editTLS(GNEViewNetHelper::ViewObjectsSelector& viewObjects, const Position& clickedPosition, const bool shiftKeyPressed) {
    // first check if in viewObjects there is a junction
    if (viewObjects.getJunctionFront()) {
        // check if we're adding or removing joined TLSs
        if (myTLSJunction->isJoiningJunctions()) {
            myTLSJunction->toggleJunctionSelected(viewObjects.getJunctionFront());
        } else {
            // show objects under cursor
            myOverlappedInspection->showOverlappedInspection(viewObjects, clickedPosition, shiftKeyPressed);
            // hide if we inspect only one junction
            if (myOverlappedInspection->getNumberOfOverlappedACs()) {
                if (myOverlappedInspection->getNumberOfOverlappedACs() == 1) {
                    myOverlappedInspection->hiderOverlappedInspection();
                }
                for (const auto& junction : viewObjects.getJunctions()) {
                    if (junction == myOverlappedInspection->getCurrentAC()) {
                        editJunction(junction);
                    }
                }
            }
        }
    } else if (viewObjects.getAdditionalFront() && myTLSAttributes->isSetDetectorsToggleButtonEnabled() &&
               (viewObjects.getAdditionalFront()->getTagProperty()->getTag() == SUMO_TAG_INDUCTION_LOOP)) {
        myTLSAttributes->toggleE1DetectorSelection(viewObjects.getAdditionalFront());
    }
    myViewNet->update();
}


bool
GNETLSEditorFrame::isTLSSaved() {
    if (myTLSPrograms->checkHaveModifications()) {
        // open question box
        FXuint answer = FXMessageBox::question(this, MBOX_YES_NO_CANCEL,
                                               TL("Save TLS Changes"), "%s",
                                               TL("There are unsaved changes in the currently edited traffic light.\nDo you want to save it before changing mode?"));
        if (answer == MBOX_CLICKED_YES) { //1:yes, 2:no, 4:esc/cancel
            // save modifications
            myTLSPrograms->onCmdSaveChanges(nullptr, 0, nullptr);
            return true;
        } else if (answer == MBOX_CLICKED_NO) {
            // cancel modifications
            myTLSPrograms->onCmdSaveChanges(nullptr, 0, nullptr);
            return true;
        } else {
            // abort changing mode
            return false;
        }
    } else {
        return true;
    }
}


bool
GNETLSEditorFrame::parseTLSPrograms(const std::string& file) {
    NBTrafficLightLogicCont& tllCont = myViewNet->getNet()->getTLLogicCont();
    NBTrafficLightLogicCont tmpTLLCont;
    NIXMLTrafficLightsHandler tllHandler(tmpTLLCont, myViewNet->getNet()->getEdgeCont());
    // existing definitions must be available to update their programs
    std::set<NBTrafficLightDefinition*> origDefs;
    for (NBTrafficLightDefinition* def : tllCont.getDefinitions()) {
        // make a duplicate of every program
        NBTrafficLightLogic* logic = tllCont.getLogic(def->getID(), def->getProgramID());
        if (logic != nullptr) {
            NBTrafficLightDefinition* duplicate = new NBLoadedSUMOTLDef(*def, *logic);
            std::vector<NBNode*> nodes = def->getNodes();
            for (auto it_node : nodes) {
                GNEJunction* junction = myViewNet->getNet()->getAttributeCarriers()->retrieveJunction(it_node->getID());
                myViewNet->getUndoList()->add(new GNEChange_TLS(junction, def, false, false), true);
                myViewNet->getUndoList()->add(new GNEChange_TLS(junction, duplicate, true), true);
            }
            tmpTLLCont.insert(duplicate);
            origDefs.insert(duplicate);
        } else {
            WRITE_WARNINGF(TL("tlLogic '%', program '%' could not be built"), def->getID(), def->getProgramID());
        }
    }
    //std::cout << " initialized tmpCont with " << origDefs.size() << " defs\n";
    XMLSubSys::runParser(tllHandler, file);

    std::vector<NBLoadedSUMOTLDef*> loadedTLS;
    for (NBTrafficLightDefinition* def : tmpTLLCont.getDefinitions()) {
        NBLoadedSUMOTLDef* sdef = dynamic_cast<NBLoadedSUMOTLDef*>(def);
        if (sdef != nullptr) {
            loadedTLS.push_back(sdef);
        }
    }
    myViewNet->setStatusBarText(TL("Loaded ") + toString(loadedTLS.size()) + TL(" programs"));
    for (auto def : loadedTLS) {
        if (origDefs.count(def) != 0) {
            // already add to undolist before
            //std::cout << " skip " << def->getDescription() << "\n";
            continue;
        }
        std::vector<NBNode*> nodes = def->getNodes();
        //std::cout << " add " << def->getDescription() << " for nodes=" << toString(nodes) << "\n";
        for (auto it_node : nodes) {
            GNEJunction* junction = myViewNet->getNet()->getAttributeCarriers()->retrieveJunction(it_node->getID());
            //myViewNet->getUndoList()->add(new GNEChange_TLS(junction, myTLSEditorParent->myEditedDef, false), true);
            myViewNet->getUndoList()->add(new GNEChange_TLS(junction, def, true), true);
        }
    }
    // clean up temporary container to avoid deletion of defs when its destruct is called
    for (NBTrafficLightDefinition* def : tmpTLLCont.getDefinitions()) {
        tmpTLLCont.removeProgram(def->getID(), def->getProgramID(), false);
    }
    return true;
}


void
GNETLSEditorFrame::selectedOverlappedElement(GNEAttributeCarrier* AC) {
    auto junction = dynamic_cast<GNEJunction*>(AC);
    if (junction) {
        editJunction(junction);
    }
}


void
GNETLSEditorFrame::cleanup() {
    if (myTLSJunction->getCurrentJunction()) {
        myTLSJunction->getCurrentJunction()->selectTLS(false);
        if (myTLSPrograms->getNumberOfPrograms() > 0) {
            for (const auto& node : myTLSPrograms->getCurrentTLSPrograms()->getNodes()) {
                myViewNet->getNet()->getAttributeCarriers()->retrieveJunction(node->getID())->selectTLS(false);
            }
        }
    }
    // clean data structures
    myTLSJunction->setCurrentJunction(nullptr);
    // check if delete myEditedDef
    if (myEditedDef) {
        delete myEditedDef;
        myEditedDef = nullptr;
    }
    // clear internal lanes
    buildInternalLanes(nullptr);
    // clean up attributes
    myTLSPrograms->clearTLSProgramss();
    // clean up attributes
    myTLSAttributes->clearTLSAttributes();
    // only clears when there are no definitions
    myTLSPhases->initPhaseTable();
}


GNETLSEditorFrame::TLSJunction*
GNETLSEditorFrame::getTLSJunction() const {
    return myTLSJunction;
}


GNETLSEditorFrame::TLSPrograms*
GNETLSEditorFrame::getTLSPrograms() const {
    return myTLSPrograms;
}


GNETLSEditorFrame::TLSAttributes*
GNETLSEditorFrame::getTLSAttributes() const {
    return myTLSAttributes;
}


GNETLSEditorFrame::TLSPhases*
GNETLSEditorFrame::getTLSPhases() const {
    return myTLSPhases;
}


void
GNETLSEditorFrame::buildInternalLanes(const NBTrafficLightDefinition* tlDef) {
    // clean up previous internal lanes
    for (const auto& internalLanes : myInternalLanes) {
        for (const auto& internalLane : internalLanes.second) {
            // remove internal lane from ACs
            myViewNet->getNet()->getAttributeCarriers()->deleteInternalLane(internalLane);
            // delete internal lane
            delete internalLane;
        }
    }
    // clear container
    myInternalLanes.clear();
    // create new internal lanes
    if (tlDef != nullptr) {
        const int NUM_POINTS = 10;
        const NBNode* nbnCurrentJunction = myTLSJunction->getCurrentJunction()->getNBNode();
        // get innerID NWWriter_SUMO::writeInternalEdges
        const std::string innerID = ":" + nbnCurrentJunction->getID();
        const NBConnectionVector& links = tlDef->getControlledLinks();
        // iterate over links
        for (const auto& link : links) {
            int tlIndex = link.getTLIndex();
            PositionVector shape;
            try {
                const NBEdge::Connection& con = link.getFrom()->getConnection(link.getFromLane(), link.getTo(), link.getToLane());
                shape = con.shape;
                shape.append(con.viaShape);
            } catch (ProcessError&) {
                shape = link.getFrom()->getToNode()->computeInternalLaneShape(link.getFrom(), NBEdge::Connection(link.getFromLane(),
                        link.getTo(), link.getToLane()), NUM_POINTS);
            }
            if (shape.length() < 2) {
                // enlarge shape to ensure visibility
                shape.clear();
                const PositionVector laneShapeFrom = link.getFrom()->getLaneShape(link.getFromLane());
                const PositionVector laneShapeTo = link.getTo()->getLaneShape(link.getToLane());
                shape.push_back(laneShapeFrom.positionAtOffset(MAX2(0.0, laneShapeFrom.length() - 1)));
                shape.push_back(laneShapeTo.positionAtOffset(MIN2(1.0, laneShapeFrom.length())));
            }
            GNEInternalLane* internalLane = new GNEInternalLane(this, myTLSJunction->getCurrentJunction(), innerID + '_' + toString(tlIndex),  shape, tlIndex);
            // add into atribute carriers
            myViewNet->getNet()->getAttributeCarriers()->insertInternalLane(internalLane);
            myInternalLanes[tlIndex].push_back(internalLane);
        }
        // iterate over crossings
        for (const auto& nbn : tlDef->getNodes()) {
            for (const auto& crossing : nbn->getCrossings()) {
                if (crossing->tlLinkIndex2 > 0 && crossing->tlLinkIndex2 != crossing->tlLinkIndex) {
                    // draw both directions
                    PositionVector forward = crossing->shape;
                    forward.move2side(crossing->width / 4);
                    GNEInternalLane* internalLane = new GNEInternalLane(this, myTLSJunction->getCurrentJunction(), crossing->id, forward, crossing->tlLinkIndex);
                    // add into atribute carriers
                    myViewNet->getNet()->getAttributeCarriers()->insertInternalLane(internalLane);
                    myInternalLanes[crossing->tlLinkIndex].push_back(internalLane);
                    PositionVector backward = crossing->shape.reverse();
                    backward.move2side(crossing->width / 4);
                    GNEInternalLane* internalLaneReverse = new GNEInternalLane(this, myTLSJunction->getCurrentJunction(), crossing->id + "_r", backward, crossing->tlLinkIndex2);
                    // add into atribute carriers
                    myViewNet->getNet()->getAttributeCarriers()->insertInternalLane(internalLaneReverse);
                    myInternalLanes[crossing->tlLinkIndex2].push_back(internalLaneReverse);
                } else {
                    // draw only one lane for both directions
                    GNEInternalLane* internalLane = new GNEInternalLane(this, myTLSJunction->getCurrentJunction(), crossing->id, crossing->shape, crossing->tlLinkIndex);
                    // add into atribute carriers
                    myViewNet->getNet()->getAttributeCarriers()->insertInternalLane(internalLane);
                    myInternalLanes[crossing->tlLinkIndex].push_back(internalLane);
                }
            }
        }
    }
}


std::string
GNETLSEditorFrame::varDurString(SUMOTime dur) {
    return (dur == NBTrafficLightDefinition::UNSPECIFIED_DURATION) ? "" : getSteps2Time(dur);
}


const NBTrafficLightLogic::PhaseDefinition&
GNETLSEditorFrame::getPhase(const int index) {
    if ((index >= 0) || (index < (int)myEditedDef->getLogic()->getPhases().size())) {
        return myEditedDef->getLogic()->getPhases().at(index);
    } else {
        throw ProcessError(TL("Invalid phase index"));
    }
}


void
GNETLSEditorFrame::handleChange(GNEInternalLane* lane) {
    myTLSPrograms->markAsModified();
    // get current selected row
    const auto selectedRow = myTLSPhases->getPhaseTable()->getCurrentSelectedRow();
    if (myViewNet->changeAllPhases()) {
        for (int row = 0; row < (int)myEditedDef->getLogic()->getPhases().size(); row++) {
            myEditedDef->getLogic()->setPhaseState(row, lane->getTLIndex(), lane->getLinkState());
        }
    } else {
        myEditedDef->getLogic()->setPhaseState(myTLSPhases->getPhaseTable()->getCurrentSelectedRow(), lane->getTLIndex(), lane->getLinkState());
    }
    // init phaseTable
    myTLSPhases->initPhaseTable();
    // select row
    myTLSPhases->getPhaseTable()->selectRow(selectedRow);
    // focus table
    myTLSPhases->getPhaseTable()->setFocus();
}


void
GNETLSEditorFrame::handleMultiChange(GNELane* lane, FXObject* obj, FXSelector sel, void* eventData) {
    if (myEditedDef != nullptr) {
        myTLSPrograms->markAsModified();
        const NBConnectionVector& links = myEditedDef->getControlledLinks();
        std::set<std::string> fromIDs;
        fromIDs.insert(lane->getMicrosimID());
        // if neither the lane nor its edge are selected, apply changes to the whole edge
        if (!lane->getParentEdge()->isAttributeCarrierSelected() && !lane->isAttributeCarrierSelected()) {
            for (auto it_lane : lane->getParentEdge()->getChildLanes()) {
                fromIDs.insert(it_lane->getMicrosimID());
            }
        } else {
            // if the edge is selected, apply changes to all lanes of all selected edges
            if (lane->getParentEdge()->isAttributeCarrierSelected()) {
                const auto selectedEdge = myViewNet->getNet()->getAttributeCarriers()->getSelectedEdges();
                for (const auto& edge : selectedEdge) {
                    for (auto it_lane : edge->getChildLanes()) {
                        fromIDs.insert(it_lane->getMicrosimID());
                    }
                }
            }
            // if the lane is selected, apply changes to all selected lanes
            if (lane->isAttributeCarrierSelected()) {
                const auto selectedLanes = myViewNet->getNet()->getAttributeCarriers()->getSelectedLanes();
                for (auto it_lane : selectedLanes) {
                    fromIDs.insert(it_lane->getMicrosimID());
                }
            }

        }
        // set new state for all connections from the chosen lane IDs
        for (auto it : links) {
            if (fromIDs.count(it.getFrom()->getLaneID(it.getFromLane())) > 0) {
                std::vector<GNEInternalLane*> lanes = myInternalLanes[it.getTLIndex()];
                for (auto it_lane : lanes) {
                    it_lane->onDefault(obj, sel, eventData);
                }
            }
        }
    }
}


bool
GNETLSEditorFrame::controlsEdge(GNEEdge* edge) const {
    if (myEditedDef != nullptr) {
        const NBConnectionVector& links = myEditedDef->getControlledLinks();
        for (auto it : links) {
            if (it.getFrom()->getID() == edge->getMicrosimID()) {
                return true;
            }
        }
    }
    return false;
}


void
GNETLSEditorFrame::editJunction(GNEJunction* junction) {
    if ((myTLSJunction->getCurrentJunction() == nullptr) || (!myTLSPrograms->checkHaveModifications() && (junction != myTLSJunction->getCurrentJunction()))) {
        // discard previous changes
        myTLSPrograms->discardChanges(false);
        // set junction
        myTLSJunction->setCurrentJunction(junction);
        // init TLS definitions
        if (myTLSPrograms->initTLSPrograms()) {
            // init TLSAttributes
            myTLSAttributes->initTLSAttributes();
            // begin undo-list
            myViewNet->getUndoList()->begin(GUIIcon::MODETLS, TL("modifying TLS definition"));
            // only select TLS if getCurrentJunction exist
            if (myTLSJunction->getCurrentJunction()) {
                myTLSJunction->getCurrentJunction()->selectTLS(true);
            }
            if (myTLSPrograms->getNumberOfPrograms() > 0) {
                for (NBNode* node : myTLSPrograms->getCurrentTLSPrograms()->getNodes()) {
                    myViewNet->getNet()->getAttributeCarriers()->retrieveJunction(node->getID())->selectTLS(true);
                }
                // update color
                myTLSPhases->updateTLSColoring();
            }
        }
    } else {
        myViewNet->setStatusBarText(TL("Unsaved modifications. Abort or Save"));
    }
    updateModules();
}


SUMOTime
GNETLSEditorFrame::getSUMOTime(const std::string& string) {
    return TIME2STEPS(GNEAttributeCarrier::parse<double>(string));
}

const std::string
GNETLSEditorFrame::getSteps2Time(const SUMOTime value) {
    return toString(STEPS2TIME(value));
}

// ---------------------------------------------------------------------------
// GNETLSEditorFrame::TLSAttributes - methods
// ---------------------------------------------------------------------------

GNETLSEditorFrame::TLSAttributes::TLSAttributes(GNETLSEditorFrame* TLSEditorParent) :
    MFXGroupBoxModule(TLSEditorParent, TL("Traffic Light Attributes")),
    myTLSEditorParent(TLSEditorParent) {
    // create frame, label and TextField for Offset (By default disabled)
    FXHorizontalFrame* horizontalFrame = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    new FXLabel(horizontalFrame, toString(SUMO_ATTR_OFFSET).c_str(), nullptr, GUIDesignLabelThickedFixed(100));
    myOffsetTextField = new FXTextField(horizontalFrame, GUIDesignTextFieldNCol, this, MID_GNE_TLSFRAME_ATTRIBUTES_OFFSET, GUIDesignTextField);
    myOffsetTextField->disable();
    // create frame, label and TextField for parameters (By default disabled)
    horizontalFrame = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    myButtonEditParameters = GUIDesigns::buildFXButton(horizontalFrame, TL("parameters"), "", "", nullptr, this, MID_GNE_TLSFRAME_ATTRIBUTES_PARAMETERSDIALOG, GUIDesignButtonAttribute);
    myParametersTextField = new FXTextField(horizontalFrame, GUIDesignTextFieldNCol, this, MID_GNE_TLSFRAME_ATTRIBUTES_PARAMETERS, GUIDesignTextField);
    myButtonEditParameters->disable();
    myParametersTextField->disable();
    // create set detectors button (By default disabled)
    mySetDetectorsToggleButton = new MFXToggleButtonTooltip(getCollapsableFrame(),
            TLSEditorParent->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
            TL("Assign E1 detectors") + std::string("\t") + TL("Enable assign E1 mode") + std::string("\t") + TL("Assign E1 detectors to the current TLS"),
            TL("Assign E1 detectors") + std::string("\t") + TL("Disable assign E1 mode") + std::string("\t") + TL("Assign E1 detectors to the current TLS"),
            GUIIconSubSys::getIcon(GUIIcon::E1), GUIIconSubSys::getIcon(GUIIcon::E1),
            this, MID_GNE_TLSFRAME_ATTRIBUTES_TOGGLEDETECTOR, GUIDesignButton);
    mySetDetectorsToggleButton->disable();
}


GNETLSEditorFrame::TLSAttributes::~TLSAttributes() {}


void
GNETLSEditorFrame::TLSAttributes::updateTLSAttributes() {
    if (myTLSEditorParent->myTLSPrograms->getNumberOfPrograms() == 0) {
        // no TLS programs, disable elements
        myOffsetTextField->disable();
        myButtonEditParameters->disable();
        myParametersTextField->disable();
        // disable E1 detector mode
        disableE1DetectorMode();
        mySetDetectorsToggleButton->disable();
        // clear E1 detectors
        if (myE1Detectors.size() > 0) {
            myE1Detectors.clear();
            myTLSEditorParent->getViewNet()->update();
        }
    } else if (myTLSEditorParent->myTLSJunction->isJoiningJunctions()) {
        // joining TLSs, disable button
        myOffsetTextField->disable();
        myButtonEditParameters->disable();
        myParametersTextField->disable();
        // disable E1 detector mode
        disableE1DetectorMode();
        mySetDetectorsToggleButton->disable();
        // clear E1 detectors
        if (myE1Detectors.size() > 0) {
            myE1Detectors.clear();
            myTLSEditorParent->getViewNet()->update();
        }
    } else if (isSetDetectorsToggleButtonEnabled()) {
        // set detectors toggle button is enabled, disable elements
        myOffsetTextField->disable();
        myButtonEditParameters->disable();
        myParametersTextField->disable();
    } else {
        myOffsetTextField->enable();
        myButtonEditParameters->enable();
        myParametersTextField->enable();
        // disable E1 detector mode in static
        if (myTLSEditorParent->myTLSPrograms->getCurrentTLSPrograms()->getType() == TrafficLightType::STATIC) {
            // disable E1 detector mode
            disableE1DetectorMode();
            mySetDetectorsToggleButton->disable();
            // clear E1 detectors
            if (myE1Detectors.size() > 0) {
                myE1Detectors.clear();
                myTLSEditorParent->getViewNet()->update();
            }
        } else {
            mySetDetectorsToggleButton->enable();
        }
    }
}


void
GNETLSEditorFrame::TLSAttributes::showTLSAttributes() {
    show();
}


void
GNETLSEditorFrame::TLSAttributes::hideTLSAttributes() {
    hide();
}


void
GNETLSEditorFrame::TLSAttributes::initTLSAttributes() {
    // get current edited junction
    const auto junction = myTLSEditorParent->myTLSJunction->getCurrentJunction();
    if (junction == nullptr) {
        throw ProcessError("Junction cannot be NULL");
    } else {
        // enable Offset
        myOffsetTextField->enable();
        myOffsetTextField->setTextColor(MFXUtils::getFXColor(RGBColor::BLACK));
        // enable parameters
        myButtonEditParameters->enable();
        myParametersTextField->enable();
        myParametersTextField->setTextColor(MFXUtils::getFXColor(RGBColor::BLACK));
        // reset mySetDetectorsToggleButton
        disableE1DetectorMode();
    }
}


void
GNETLSEditorFrame::TLSAttributes::clearTLSAttributes() {
    // clear and disable Offset TextField
    myOffsetTextField->setText("");
    myOffsetTextField->disable();
    myOffsetTextField->setTextColor(MFXUtils::getFXColor(RGBColor::BLACK));
    // clear and disable parameters TextField
    myButtonEditParameters->disable();
    myParametersTextField->setText("");
    myParametersTextField->disable();
    myParametersTextField->setTextColor(MFXUtils::getFXColor(RGBColor::BLACK));
}


SUMOTime
GNETLSEditorFrame::TLSAttributes::getOffset() const {
    return getSUMOTime(myOffsetTextField->getText().text());
}


void
GNETLSEditorFrame::TLSAttributes::setOffset(const SUMOTime& offset) {
    myOffsetTextField->setText(getSteps2Time(offset).c_str());
    myOffsetTextField->setTextColor(MFXUtils::getFXColor(RGBColor::BLACK));
}


bool
GNETLSEditorFrame::TLSAttributes::isValidOffset() {
    if (GNEAttributeCarrier::canParse<SUMOTime>(myOffsetTextField->getText().text())) {
        myOffsetTextField->setTextColor(MFXUtils::getFXColor(RGBColor::BLACK));
        return true;
    } else {
        myOffsetTextField->setTextColor(MFXUtils::getFXColor(RGBColor::RED));
        return false;
    }
}


std::string
GNETLSEditorFrame::TLSAttributes::getParameters() const {
    return myParametersTextField->getText().text();
}


void
GNETLSEditorFrame::TLSAttributes::setParameters(const std::string& parameters) {
    myParametersTextField->setText(parameters.c_str());
    myParametersTextField->setTextColor(MFXUtils::getFXColor(RGBColor::BLACK));
    // update E1 detectors
    if (myTLSEditorParent->myEditedDef->getType() != TrafficLightType::STATIC) {
        updateE1Detectors();
    }
}


bool
GNETLSEditorFrame::TLSAttributes::isValidParameters() {
    if (Parameterised::areParametersValid(myParametersTextField->getText().text())) {
        myParametersTextField->setTextColor(MFXUtils::getFXColor(RGBColor::BLACK));
        return true;
    } else {
        myParametersTextField->setTextColor(MFXUtils::getFXColor(RGBColor::RED));
        return false;
    }
}


bool
GNETLSEditorFrame::TLSAttributes::isSetDetectorsToggleButtonEnabled() const {
    return (mySetDetectorsToggleButton->getState() == TRUE);
}


bool
GNETLSEditorFrame::TLSAttributes::toggleE1DetectorSelection(const GNEAdditional* E1) {
    // get E1 lane ID
    const auto laneID = E1->getParentLanes().front()->getID();
    // iterate over all E1 detectors
    for (auto it = myE1Detectors.begin(); it != myE1Detectors.end(); it++) {
        if (E1->getID() == it->second) {
            // already selected, then remove it from detectors
            myE1Detectors.erase(it);
            // and remove it from parameters
            myTLSEditorParent->myEditedDef->unsetParameter(laneID);
            myParametersTextField->setText(myTLSEditorParent->myEditedDef->getParametersStr().c_str());
            // mark TL as modified
            myTLSEditorParent->myTLSPrograms->markAsModified();
            return true;
        } else if (laneID == it->first) {
            // there is another E1 in the same lane, then swap
            myE1Detectors.erase(it);
            myE1Detectors[laneID] = E1->getID();
            // also in parameters
            myTLSEditorParent->myEditedDef->setParameter(laneID, E1->getID());
            myParametersTextField->setText(myTLSEditorParent->myEditedDef->getParametersStr().c_str());
            // mark TL as modified
            myTLSEditorParent->myTLSPrograms->markAsModified();
            return true;
        }
    }
    // add it in parameters
    myE1Detectors[laneID] = E1->getID();
    myTLSEditorParent->myEditedDef->setParameter(laneID, E1->getID());
    myParametersTextField->setText(myTLSEditorParent->myEditedDef->getParametersStr().c_str());
    // mark TL as modified
    myTLSEditorParent->myTLSPrograms->markAsModified();
    return true;
}


const std::map<std::string, std::string>&
GNETLSEditorFrame::TLSAttributes::getE1Detectors() const {
    return myE1Detectors;
}


void
GNETLSEditorFrame::TLSAttributes::disableE1DetectorMode() {
    mySetDetectorsToggleButton->setState(FALSE, TRUE);
}


long
GNETLSEditorFrame::TLSAttributes::onCmdSetOffset(FXObject*, FXSelector, void*) {
    if (isValidOffset()) {
        myTLSEditorParent->myTLSPrograms->markAsModified();
        myTLSEditorParent->myEditedDef->setOffset(getOffset());
        myOffsetTextField->killFocus();
        myTLSEditorParent->updateModules();
    }
    return 1;
}


long
GNETLSEditorFrame::TLSAttributes::onCmdSetParameters(FXObject*, FXSelector, void*) {
    if (isValidParameters()) {
        myTLSEditorParent->myTLSPrograms->markAsModified();
        myTLSEditorParent->myEditedDef->setParametersStr(getParameters());
        myParametersTextField->killFocus();
        myTLSEditorParent->updateModules();
    }
    return 1;
}


long
GNETLSEditorFrame::TLSAttributes::onCmdParametersDialog(FXObject*, FXSelector, void*) {
    // continue depending of myEditedDef
    if (myTLSEditorParent->myEditedDef) {
        // get previous parameters
        const auto previousParameters = getParameters();
        const auto internalTests = myTLSEditorParent->getViewNet()->getViewParent()->getGNEAppWindows()->getInternalTest();
        if (GNESingleParametersDialog(myTLSEditorParent->getViewNet()->getApp(), myTLSEditorParent->myEditedDef).openModalDialog(internalTests)) {
            // set parameters in textfield
            setParameters(myTLSEditorParent->myEditedDef->getParametersStr());
            // only mark as modified if parameters are different
            if (getParameters() != previousParameters) {
                myTLSEditorParent->myTLSPrograms->markAsModified();
            }
        }
        myTLSEditorParent->updateModules();
    }
    return 1;
}


long
GNETLSEditorFrame::TLSAttributes::onCmdToggleDetectorMode(FXObject*, FXSelector, void*) {
    if (mySetDetectorsToggleButton->getState()) {
        // set special color
        mySetDetectorsToggleButton->setBackColor(FXRGBA(253, 255, 206, 255));
    } else {
        // restore default color
        mySetDetectorsToggleButton->setBackColor(4293980400);
    }
    myTLSEditorParent->updateModules();
    // update view
    myTLSEditorParent->getViewNet()->update();
    return 1;
}


void
GNETLSEditorFrame::TLSAttributes::updateE1Detectors() {
    // first clear E1 detectors
    myE1Detectors.clear();
    // iterate over parameters
    for (const auto& parameter : myTLSEditorParent->myEditedDef->getParametersMap()) {
        // check if both lane and E1 exists
        if (myTLSEditorParent->getViewNet()->getNet()->getAttributeCarriers()->retrieveLane(parameter.first, false) &&
                myTLSEditorParent->getViewNet()->getNet()->getAttributeCarriers()->retrieveAdditional(SUMO_TAG_INDUCTION_LOOP, parameter.second, false)) {
            // add it into list
            myE1Detectors[parameter.first] = parameter.second;
        }
    }
    myTLSEditorParent->updateModules();
    // update view net
    myTLSEditorParent->getViewNet()->update();
}

// ---------------------------------------------------------------------------
// GNETLSEditorFrame::TLSJunction - methods
// ---------------------------------------------------------------------------

GNETLSEditorFrame::TLSJunction::TLSJunction(GNETLSEditorFrame* TLSEditorParent) :
    MFXGroupBoxModule(TLSEditorParent, TL("Traffic Light")),
    myTLSEditorParent(TLSEditorParent),
    myCurrentJunction(nullptr) {
    // Create frame for junction IDs
    FXHorizontalFrame* junctionIDFrame = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    myJunctionIDLabel = new FXLabel(junctionIDFrame, TL("Junction ID"), nullptr, GUIDesignLabelThickedFixed(100));
    myJunctionIDTextField = new MFXTextFieldTooltip(junctionIDFrame,
            TLSEditorParent->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
            GUIDesignTextFieldNCol, this, 0, GUIDesignTextField);
    // junction ID remains always disabled
    myJunctionIDTextField->disable();
    // Create frame for TLS Program ID
    FXHorizontalFrame* TLSIDFrame = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    new FXLabel(TLSIDFrame, TL("TLS ID"), nullptr, GUIDesignLabelThickedFixed(100));
    myTLSIDTextField = new MFXTextFieldTooltip(TLSIDFrame,
            TLSEditorParent->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
            GUIDesignTextFieldNCol, this, MID_GNE_TLSFRAME_TLSJUNCTION_ID, GUIDesignTextField);
    // create frame, label and textfield for type
    FXHorizontalFrame* typeFrame = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    new FXLabel(typeFrame, toString(SUMO_ATTR_TYPE).c_str(), nullptr, GUIDesignLabelThickedFixed(100));
    myTLSTypeComboBox = new MFXComboBoxIcon(typeFrame, GUIDesignComboBoxNCol, false, GUIDesignComboBoxVisibleItems,
                                            this, MID_GNE_TLSFRAME_TLSJUNCTION_TYPE, GUIDesignComboBoxAttribute);
    // fill comboBox (only certain TL types)
    myTLSTypeComboBox->appendIconItem(toString(TrafficLightType::STATIC).c_str());
    myTLSTypeComboBox->appendIconItem(toString(TrafficLightType::ACTUATED).c_str());
    myTLSTypeComboBox->appendIconItem(toString(TrafficLightType::DELAYBASED).c_str());
    myTLSTypeComboBox->appendIconItem(toString(TrafficLightType::NEMA).c_str());
    // create frame for join buttons
    FXHorizontalFrame* joinButtons = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrameUniform);
    // create join states button
    myJoinTLSToggleButton = new MFXToggleButtonTooltip(joinButtons,
            TLSEditorParent->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
            TL("Join") + std::string("\t") + TL("Enable join mode") + std::string("\t") + TL("Join TLS and junctions in the current junction."),
            TL("Join") + std::string("\t") + TL("Disable join mode") + std::string("\t") + TL("Join TLS and junctions in the current junction."),
            GUIIconSubSys::getIcon(GUIIcon::JOIN), GUIIconSubSys::getIcon(GUIIcon::JOIN),
            this, MID_GNE_TLSFRAME_TLSJUNCTION_TOGGLEJOIN, GUIDesignButton);
    myDisjoinTLSButton = new MFXButtonTooltip(joinButtons,
            TLSEditorParent->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
            TL("Disjoin") + std::string("\t") + TL("Disjoin current TLS") + std::string("\t") + TL("Disjoin current TLS."),
            GUIIconSubSys::getIcon(GUIIcon::DISJOIN), this, MID_GNE_TLSFRAME_TLSJUNCTION_DISJOIN, GUIDesignButton);
    // create frame for join control buttons
    myJoinControlButtons = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrameUniform);
    // create create tlDef button
    GUIDesigns::buildFXButton(myJoinControlButtons, TL("Accept"), "", TL("Finish join."),
                              GUIIconSubSys::getIcon(GUIIcon::ACCEPT), this, MID_GNE_BUTTON_ACCEPT, GUIDesignButton);
    GUIDesigns::buildFXButton(myJoinControlButtons, TL("Cancel"), "", TL("Cancel Join."),
                              GUIIconSubSys::getIcon(GUIIcon::CANCEL), this, MID_GNE_BUTTON_CANCEL, GUIDesignButton);
    // update junction description after creation
    refreshTLSJunction();
    // show TLS Junction
    show();
}


GNETLSEditorFrame::TLSJunction::~TLSJunction() {}


void
GNETLSEditorFrame::TLSJunction::updateTLSJunction() {
    if ((myCurrentJunction == nullptr) ||
            (myCurrentJunction->getNBNode()->getControllingTLS().size() == 0)) {
        // no TLS
        myTLSIDTextField->setText("");
        myTLSIDTextField->disable();
        myTLSTypeComboBox->disable();
        myJoinTLSToggleButton->disable();
        myDisjoinTLSButton->disable();
        myJoinControlButtons->hide();
    } else {
        // get first controled TLS
        const auto TLS = (*myCurrentJunction->getNBNode()->getControllingTLS().begin());
        // set text field ID
        myTLSIDTextField->setText(TLS->getID().c_str());
        // continue with more options
        if (myTLSEditorParent->myTLSAttributes->isSetDetectorsToggleButtonEnabled() ||
                myTLSEditorParent->myTLSPrograms->checkHaveModifications()) {
            // disable if selecting selecting detectors or we modified the program
            myTLSIDTextField->setText("");
            myTLSIDTextField->disable();
            myTLSTypeComboBox->disable();
            myJoinTLSToggleButton->disable();
            myDisjoinTLSButton->disable();
            myJoinControlButtons->hide();
        } else if (myTLSEditorParent->myTLSJunction->isJoiningJunctions()) {
            // partial disable due joining
            myTLSIDTextField->setText("");
            myTLSIDTextField->disable();
            myTLSTypeComboBox->disable();
            myDisjoinTLSButton->disable();
            // enable join TLS and show control buttons
            myJoinTLSToggleButton->enable();
            myJoinControlButtons->show();
        } else {
            // enable
            myTLSIDTextField->enable();
            myTLSTypeComboBox->enable();
            myJoinTLSToggleButton->enable();
            // disjoint button only if we have more than one TLS controlled
            if (TLS->getNodes().size() == 1) {
                myDisjoinTLSButton->disable();
            } else {
                myDisjoinTLSButton->enable();
            }
        }
    }
}


GNEJunction*
GNETLSEditorFrame::TLSJunction::getCurrentJunction() const {
    return myCurrentJunction;
}


void
GNETLSEditorFrame::TLSJunction::setCurrentJunction(GNEJunction* junction) {
    myCurrentJunction = junction;
    // resfresh module
    refreshTLSJunction();
}


bool
GNETLSEditorFrame::TLSJunction::isJoiningJunctions() const {
    return (myJoinTLSToggleButton->getState() == TRUE);
}


bool
GNETLSEditorFrame::TLSJunction::isJunctionSelected(const GNEJunction* junction) const {
    return (std::find(mySelectedJunctionIDs.begin(), mySelectedJunctionIDs.end(), junction->getID()) != mySelectedJunctionIDs.end());
}


void
GNETLSEditorFrame::TLSJunction::toggleJunctionSelected(const GNEJunction* junction) {
    // avoid current junction
    if (junction != myCurrentJunction) {
        // find ID in selected junctions
        auto it = std::find(mySelectedJunctionIDs.begin(), mySelectedJunctionIDs.end(), junction->getID());
        // check if add or remove
        if (it == mySelectedJunctionIDs.end()) {
            mySelectedJunctionIDs.push_back(junction->getID());
        } else {
            mySelectedJunctionIDs.erase(it);
        }
    }
}


const std::vector<std::string>&
GNETLSEditorFrame::TLSJunction::getSelectedJunctionIDs() const {
    return mySelectedJunctionIDs;
}


long
GNETLSEditorFrame::TLSJunction::onCmdRenameTLS(FXObject*, FXSelector, void*) {
    // get IDs
    const std::string currentTLID = (*myCurrentJunction->getNBNode()->getControllingTLS().begin())->getID();
    const std::string newTLID = myTLSIDTextField->getText().text();
    // check if ID is valid
    if (newTLID.empty() || (newTLID == currentTLID)) {
        // same ID or empty
        myTLSIDTextField->setTextColor(FXRGB(0, 0, 0));
        myTLSIDTextField->setText(currentTLID.c_str());
        myTLSIDTextField->killFocus();
        myTLSEditorParent->update();
        // show all moduls
        myTLSEditorParent->myTLSPrograms->showTLSPrograms();
        myTLSEditorParent->myTLSAttributes->showTLSAttributes();
        myTLSEditorParent->myTLSPhases->showTLSPhases();
        myTLSEditorParent->myTLSFile->showTLSFile();
    } else if (!SUMOXMLDefinitions::isValidNetID(newTLID) || myCurrentJunction->getNet()->getTLLogicCont().exist(newTLID)) {
        // set invalid color
        myTLSIDTextField->setTextColor(FXRGB(255, 0, 0));
        // hide moduls
        myTLSEditorParent->myTLSPrograms->hideTLSPrograms();
        myTLSEditorParent->myTLSAttributes->hideTLSAttributes();
        myTLSEditorParent->myTLSPhases->hideTLSPhases();
        myTLSEditorParent->myTLSFile->hideTLSFile();
    } else {
        // make a copy of myCurrentJunction and current tlDef (because will be reset after calling discardChanges)
        auto junction = myCurrentJunction;
        const auto tlDef = myTLSEditorParent->myTLSPrograms->getCurrentTLSPrograms();
        // restore color
        myTLSIDTextField->setTextColor(FXRGB(0, 0, 0));
        myTLSIDTextField->killFocus();
        myTLSEditorParent->update();
        // discard previous changes
        myTLSEditorParent->myTLSPrograms->discardChanges(false);
        // change name using undo-List
        myTLSEditorParent->getViewNet()->getUndoList()->begin(GUIIcon::MODETLS, TL("rename TLS"));
        myTLSEditorParent->getViewNet()->getUndoList()->add(new GNEChange_TLS(junction, tlDef, newTLID), true);
        myTLSEditorParent->getViewNet()->getUndoList()->end();
        // show all moduls
        myTLSEditorParent->myTLSPrograms->showTLSPrograms();
        myTLSEditorParent->myTLSAttributes->showTLSAttributes();
        myTLSEditorParent->myTLSPhases->showTLSPhases();
        myTLSEditorParent->myTLSFile->showTLSFile();
        // edit junction again
        myTLSEditorParent->editJunction(junction);
    }
    myTLSEditorParent->updateModules();
    return 1;
}


long
GNETLSEditorFrame::TLSJunction::onCmdChangeType(FXObject*, FXSelector, void*) {
    // get IDs
    const std::string currentTLType = toString((*myCurrentJunction->getNBNode()->getControllingTLS().begin())->getType());
    const std::string newTLType = myTLSTypeComboBox->getText().text();
    // check if ID is valid
    if (newTLType.empty() || (newTLType == currentTLType)) {
        // same ID or empty, don't change
        myTLSTypeComboBox->setTextColor(FXRGB(0, 0, 0));
        // set value
        const int index = myTLSTypeComboBox->findItem(currentTLType.c_str());
        if (index == -1) {
            myTLSTypeComboBox->disable();
        } else {
            myTLSTypeComboBox->setCurrentItem(index);
            myTLSTypeComboBox->enable();
        }
        myTLSTypeComboBox->killFocus();
        myTLSEditorParent->update();
        // show all moduls
        myTLSEditorParent->myTLSPrograms->showTLSPrograms();
        myTLSEditorParent->myTLSAttributes->showTLSAttributes();
        myTLSEditorParent->myTLSPhases->showTLSPhases();
        myTLSEditorParent->myTLSFile->showTLSFile();
    } else if (!SUMOXMLDefinitions::TrafficLightTypes.hasString(newTLType)) {
        // set invalid color
        myTLSTypeComboBox->setTextColor(FXRGB(255, 0, 0));
        // hide moduls
        myTLSEditorParent->myTLSPrograms->hideTLSPrograms();
        myTLSEditorParent->myTLSAttributes->hideTLSAttributes();
        myTLSEditorParent->myTLSPhases->hideTLSPhases();
        myTLSEditorParent->myTLSFile->hideTLSFile();
    } else {
        // reset color
        myTLSTypeComboBox->setTextColor(FXRGB(0, 0, 0));
        myTLSTypeComboBox->killFocus();
        myTLSEditorParent->update();
        // make a copy of myCurrentJunction (because will be reset after calling discardChanges)
        auto junction = myCurrentJunction;
        // discard previous changes
        myTLSEditorParent->myTLSPrograms->discardChanges(false);
        // change name using undo-List
        myTLSEditorParent->getViewNet()->getUndoList()->begin(GUIIcon::MODETLS, TL("change TLS type"));
        junction->setAttribute(SUMO_ATTR_TLTYPE, newTLType, myTLSEditorParent->getViewNet()->getUndoList());
        myTLSEditorParent->getViewNet()->getUndoList()->end();
        // show all moduls
        myTLSEditorParent->myTLSPrograms->showTLSPrograms();
        myTLSEditorParent->myTLSAttributes->showTLSAttributes();
        myTLSEditorParent->myTLSPhases->showTLSPhases();
        myTLSEditorParent->myTLSFile->showTLSFile();
        // edit junction again
        myTLSEditorParent->editJunction(junction);
    }
    myTLSEditorParent->updateModules();
    return 1;

}


long
GNETLSEditorFrame::TLSJunction::onCmdToggleJoinTLS(FXObject*, FXSelector, void*) {
    if (myJoinTLSToggleButton->getState()) {
        // set special color
        myJoinTLSToggleButton->setBackColor(FXRGBA(253, 255, 206, 255));
        // clear and fill mySelectedJunctionIDs
        mySelectedJunctionIDs.clear();
        // get all nodes controlled by this TLS
        const auto TLNodes = (*myCurrentJunction->getNBNode()->getControllingTLS().begin())->getNodes();
        // fill mySelectedJunctionIDs with TLNodes
        mySelectedJunctionIDs.clear();
        for (const auto& TLNode : TLNodes) {
            mySelectedJunctionIDs.push_back(TLNode->getID());
        }
        // make a copy of selected junctions
        myOriginalSelectedJunctionIDs = mySelectedJunctionIDs;
        // show control buttons
        myJoinControlButtons->show();
        getCollapsableFrame()->recalc();
    } else {
        // hide control buttons
        myJoinControlButtons->hide();
        getCollapsableFrame()->recalc();
        // make a copy of current junction
        const auto currentJunction = myCurrentJunction;
        // declare vectors for junctions
        std::vector<GNEJunction*> selectedJunctions, resetTLJunctions;
        // get selected junctions (all except current
        for (const auto& selectedJunctionID : mySelectedJunctionIDs) {
            if (selectedJunctionID != currentJunction->getID()) {
                selectedJunctions.push_back(myTLSEditorParent->getViewNet()->getNet()->getAttributeCarriers()->retrieveJunction(selectedJunctionID));
            }
        }
        // get junctions to reset TL (all TL nodes except current)
        for (const auto& TLNBNode : (*currentJunction->getNBNode()->getControllingTLS().begin())->getNodes()) {
            if (TLNBNode != currentJunction->getNBNode()) {
                resetTLJunctions.push_back(myTLSEditorParent->getViewNet()->getNet()->getAttributeCarriers()->retrieveJunction(TLNBNode->getID()));
            }
        }
        // discard changes
        myTLSEditorParent->myTLSPrograms->discardChanges(false);
        // begin undo list
        myTLSEditorParent->getViewNet()->getUndoList()->begin(GUIIcon::MODETLS, TL("join TLS"));
        // remove tl from TLNBNode
        for (const auto& resetTLJunction : resetTLJunctions) {
            resetTLJunction->setAttribute(SUMO_ATTR_TYPE, "priority", myTLSEditorParent->getViewNet()->getUndoList());
        }
        // now update it in all joined junctions
        for (const auto& selectedJunction : selectedJunctions) {
            selectedJunction->setAttribute(SUMO_ATTR_TYPE, currentJunction->getAttribute(SUMO_ATTR_TYPE), myTLSEditorParent->getViewNet()->getUndoList());
            selectedJunction->setAttribute(SUMO_ATTR_TLID, currentJunction->getAttribute(SUMO_ATTR_TLID), myTLSEditorParent->getViewNet()->getUndoList());
        }
        // end undo list
        myTLSEditorParent->getViewNet()->getUndoList()->end();
        // restore default color
        myJoinTLSToggleButton->setBackColor(4293980400);
        // clear selected junction IDs
        mySelectedJunctionIDs.clear();
        // edit junction again
        myTLSEditorParent->editJunction(currentJunction);
    }
    myTLSEditorParent->updateModules();
    // update view
    myTLSEditorParent->getViewNet()->update();
    return 1;
}


long
GNETLSEditorFrame::TLSJunction::onCmdDisjoinTLS(FXObject*, FXSelector, void*) {
    // make a copy of current junction
    const auto currentJunction = myCurrentJunction;
    // declare vectors for junctions
    std::vector<GNEJunction*> resetTLJunctions;
    // get junctions to reset TL
    for (const auto& TLNBNode : (*currentJunction->getNBNode()->getControllingTLS().begin())->getNodes()) {
        resetTLJunctions.push_back(myTLSEditorParent->getViewNet()->getNet()->getAttributeCarriers()->retrieveJunction(TLNBNode->getID()));
    }
    // save TL types
    const auto type = resetTLJunctions.front()->getAttribute(SUMO_ATTR_TYPE);
    const auto tlType = resetTLJunctions.front()->getAttribute(SUMO_ATTR_TLTYPE);
    // discard changes
    myTLSEditorParent->myTLSPrograms->discardChanges(false);
    // begin undo list
    myTLSEditorParent->getViewNet()->getUndoList()->begin(GUIIcon::MODETLS, TL("disjoin TLS"));
    // the disjoint tlIds will be the junction ids. Ensure that there is no name clash with the current tlId
    NBTrafficLightLogicCont& tllCont = myTLSEditorParent->getViewNet()->getNet()->getTLLogicCont();
    const std::string oldId = currentJunction->getAttribute(SUMO_ATTR_TLID);
    const std::string tmpIdBase = oldId + "_TMP";
    int tmpIndex = 0;
    std::string tmpId = tmpIdBase + toString(tmpIndex);
    while (tllCont.exist(tmpId)) {
        tmpId = tmpIdBase + toString(++tmpIndex);
    }
    for (NBTrafficLightDefinition* tlDef : currentJunction->getNBNode()->getControllingTLS()) {
        myTLSEditorParent->getViewNet()->getUndoList()->add(new GNEChange_TLS(currentJunction, tlDef, tmpId), true);
    }
    // remove tl from TLNBNode and the re-initialize as single traffic light
    for (const auto& resetTLJunction : resetTLJunctions) {
        resetTLJunction->setAttribute(SUMO_ATTR_TYPE, "priority", myTLSEditorParent->getViewNet()->getUndoList());
        resetTLJunction->setAttribute(SUMO_ATTR_TYPE, type, myTLSEditorParent->getViewNet()->getUndoList());
        resetTLJunction->setAttribute(SUMO_ATTR_TLTYPE, tlType, myTLSEditorParent->getViewNet()->getUndoList());
    }
    // end undo list
    myTLSEditorParent->getViewNet()->getUndoList()->end();
    // restore default color
    myJoinTLSToggleButton->setBackColor(4293980400);
    // clear selected junction IDs
    mySelectedJunctionIDs.clear();
    // edit junction again
    myTLSEditorParent->editJunction(currentJunction);
    myTLSEditorParent->updateModules();
    return 1;
}


long
GNETLSEditorFrame::TLSJunction::onCmdAcceptJoin(FXObject*, FXSelector, void*) {
    myJoinTLSToggleButton->setState(FALSE, TRUE);
    myTLSEditorParent->updateModules();
    return 1;
}


long
GNETLSEditorFrame::TLSJunction::onCmdCancelJoin(FXObject*, FXSelector, void*) {
    // restore selected junction
    mySelectedJunctionIDs = myOriginalSelectedJunctionIDs;
    myJoinTLSToggleButton->setState(FALSE, TRUE);
    myTLSEditorParent->updateModules();
    return 1;
}


void
GNETLSEditorFrame::TLSJunction::refreshTLSJunction() {
    // first reset junction label
    myJunctionIDLabel->setText(TL("Junction ID"));
    // clear selected junctions
    mySelectedJunctionIDs.clear();
    // cancel joining junction mode
    onCmdCancelJoin(nullptr, 0, nullptr);
    // continue depending of current junction
    if (myCurrentJunction == nullptr) {
        myJunctionIDTextField->setText(TL("No junction selected"));
    } else {
        // get all TLS assigned to this node
        const auto& TLSs = myCurrentJunction->getNBNode()->getControllingTLS();
        // check if junction is controlled
        if (TLSs.size() > 0) {
            // get first TLS
            const auto TLS = (*TLSs.begin());
            // update junction ID text field
            if (TLS->getNodes().size() > 1) {
                // declare string
                std::string TLSNodesStr;
                for (auto it = TLS->getNodes().begin(); it != TLS->getNodes().end(); it++) {
                    if (it == (TLS->getNodes().end() - 1)) {
                        TLSNodesStr += (*it)->getID();
                    } else {
                        TLSNodesStr += (*it)->getID() + ", ";
                    }
                }
                // updated junction fields
                myJunctionIDTextField->setText(TLSNodesStr.c_str());
                // update junction label if we have multiple nodes
                if (TLS->getNodes().size() > 1) {
                    myJunctionIDLabel->setText(TL("Junction IDs"));
                }
                // set TLS type in comboBox
                const int index = myTLSTypeComboBox->findItem(myCurrentJunction->getAttribute(SUMO_ATTR_TLTYPE).c_str());
                if (index != -1) {
                    myTLSTypeComboBox->setCurrentItem(index, FALSE);
                }
            }
        } else {
            // update junction ID text field
            myJunctionIDTextField->setText(myCurrentJunction->getID().c_str());
            myTLSTypeComboBox->setCurrentItem(0, FALSE);
        }
    }
    // update TLS junction
    updateTLSJunction();
}

// ---------------------------------------------------------------------------
// GNETLSEditorFrame::TLSPrograms - methods
// ---------------------------------------------------------------------------

GNETLSEditorFrame::TLSPrograms::TLSPrograms(GNETLSEditorFrame* TLSEditorParent) :
    MFXGroupBoxModule(TLSEditorParent, TL("Traffic Light Programs")),
    myTLSEditorParent(TLSEditorParent) {
    // create frame, label and comboBox for programID
    FXHorizontalFrame* programFrame = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    new FXLabel(programFrame, toString(SUMO_ATTR_PROGRAMID).c_str(), nullptr, GUIDesignLabelThickedFixed(100));
    myProgramComboBox = new MFXComboBoxIcon(programFrame, GUIDesignComboBoxNCol, false, GUIDesignComboBoxVisibleItems,
                                            this, MID_GNE_TLSFRAME_DEFINITION_SWITCHPROGRAM, GUIDesignComboBoxAttribute);
    myProgramComboBox->disable();
    // create auxiliar frames
    FXHorizontalFrame* horizontalFrameAux = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrameUniform);
    FXVerticalFrame* verticalFrameAuxA = new FXVerticalFrame(horizontalFrameAux, GUIDesignAuxiliarHorizontalFrame);
    FXVerticalFrame* verticalFrameAuxB = new FXVerticalFrame(horizontalFrameAux, GUIDesignAuxiliarHorizontalFrame);
    // create create tlDef button
    myCreateButton = GUIDesigns::buildFXButton(verticalFrameAuxA, TL("Create TLS"), "", TL("Create a new traffic light program."),
                     GUIIconSubSys::getIcon(GUIIcon::MODETLS), this, MID_GNE_TLSFRAME_DEFINITION_CREATE, GUIDesignButton);
    myCreateButton->disable();
    // create delete tlDef button
    myDeleteButton = GUIDesigns::buildFXButton(verticalFrameAuxA, TL("Delete"), "", TL("Delete a traffic light program. If all programs are deleted the junction turns into a priority junction."),
                     GUIIconSubSys::getIcon(GUIIcon::REMOVE), this, MID_GNE_TLSFRAME_DEFINITION_DELETE, GUIDesignButton);
    myDeleteButton->disable();
    // create reset current tlDef button
    myResetSingleButton = GUIDesigns::buildFXButton(verticalFrameAuxB, TL("Reset single"), "", TL("Reset current TLS program."),
                          GUIIconSubSys::getIcon(GUIIcon::RESET), this, MID_GNE_TLSFRAME_DEFINITION_RESETCURRENT, GUIDesignButton);
    myResetSingleButton->disable();
    // create reset all tlDefs button
    myResetAllButton = GUIDesigns::buildFXButton(verticalFrameAuxB, TL("Reset all"), "", TL("Reset all TLS programs."),
                       GUIIconSubSys::getIcon(GUIIcon::RESET), this, MID_GNE_TLSFRAME_DEFINITION_RESETALL, GUIDesignButton);
    myResetAllButton->disable();
    // create save modifications button
    mySaveButon = GUIDesigns::buildFXButton(verticalFrameAuxA, TL("Save"), "", TL("Save program modifications. (Enter)"),
                                            GUIIconSubSys::getIcon(GUIIcon::OK), this, MID_GNE_TLSFRAME_DEFINITION_SAVE, GUIDesignButton);
    mySaveButon->disable();
    // create cancel modifications buttons
    myCancelButon = GUIDesigns::buildFXButton(verticalFrameAuxB, TL("Cancel"), "", TL("Discard program modifications. (Esc)"),
                    GUIIconSubSys::getIcon(GUIIcon::CANCEL), this, MID_GNE_TLSFRAME_DEFINITION_DISCARD, GUIDesignButton);
    myCancelButon->disable();
    // show GroupBox
    show();
}


GNETLSEditorFrame::TLSPrograms::~TLSPrograms() {}


void
GNETLSEditorFrame::TLSPrograms::updateTLSPrograms() {
    if (getNumberOfPrograms() == 0) {
        // no TLS Programs, disable buttons except create (if we have a junction)
        if (myTLSEditorParent->getTLSJunction()->getCurrentJunction() != nullptr) {
            myCreateButton->enable();
        } else {
            myCreateButton->disable();
        }
        myDeleteButton->disable();
        myResetSingleButton->disable();
        myProgramComboBox->disable();
        myResetAllButton->disable();
        mySaveButon->disable();
        myCancelButon->disable();
    } else if (myTLSEditorParent->myTLSAttributes->isSetDetectorsToggleButtonEnabled()) {
        // selecting E1, disable buttons
        myCreateButton->disable();
        myDeleteButton->disable();
        myResetSingleButton->disable();
        myProgramComboBox->disable();
        myResetAllButton->disable();
        mySaveButon->disable();
        myCancelButon->disable();
    } else if (myTLSEditorParent->myTLSJunction->isJoiningJunctions()) {
        // joining TLSs, disable button
        myCreateButton->disable();
        myDeleteButton->disable();
        myResetSingleButton->disable();
        myProgramComboBox->disable();
        myResetAllButton->disable();
        mySaveButon->disable();
        myCancelButon->disable();
    } else if (myHaveModifications) {
        // modifications, disable button
        myCreateButton->disable();
        myDeleteButton->disable();
        myResetSingleButton->disable();
        myProgramComboBox->disable();
        myResetAllButton->disable();
        // enable save and cancel buttons
        mySaveButon->enable();
        myCancelButon->enable();
    } else {
        myCreateButton->enable();
        myDeleteButton->enable();
        myResetSingleButton->enable();
        myProgramComboBox->enable();
        // disable save and cancel buttons
        mySaveButon->disable();
        myCancelButon->disable();
        // check if enable reset all
        if (getNumberOfPrograms() > 1) {
            myResetAllButton->enable();
        } else {
            myResetAllButton->disable();
        }
    }
    // get current junction
    const auto currentJunction = myTLSEditorParent->myTLSJunction->getCurrentJunction();
    // update button text
    if (currentJunction == nullptr) {
        myCreateButton->setText(TL("Create"));
    } else if (currentJunction->getNBNode()->isTLControlled()) {
        myCreateButton->setText(TL("Duplicate"));
    } else {
        myCreateButton->setText(TL("Create"));
    }
}


void
GNETLSEditorFrame::TLSPrograms::showTLSPrograms() {
    show();
}


void
GNETLSEditorFrame::TLSPrograms::hideTLSPrograms() {
    hide();
}


bool
GNETLSEditorFrame::TLSPrograms::initTLSPrograms() {
    // get current edited junction
    const auto junction = myTLSEditorParent->myTLSJunction->getCurrentJunction();
    if (junction == nullptr) {
        throw ProcessError("Junction cannot be NULL");
    } else {
        // clear definitions
        myTLSPrograms.clear();
        // obtain TLSs sorted by ID
        std::set<std::string> programIDs;
        for (const auto& TLS : junction->getNBNode()->getControllingTLS()) {
            myTLSPrograms.push_back(TLS);
            programIDs.insert(TLS->getProgramID());
        }
        for (const auto& programID : programIDs) {
            myProgramComboBox->appendIconItem(programID.c_str());
        }
        // check if enable TLS definitions
        if (myTLSPrograms.size() > 0) {
            myProgramComboBox->enable();
            myProgramComboBox->setCurrentItem(0);
            // switch TLS Program
            return switchProgram();
        }
        return false;
    }
}


void
GNETLSEditorFrame::TLSPrograms::clearTLSProgramss() {
    // clear definitions
    myTLSPrograms.clear();
    // clear and disable myProgramComboBox
    myProgramComboBox->clearItems();
    myProgramComboBox->disable();
}


int
GNETLSEditorFrame::TLSPrograms::getNumberOfPrograms() const {
    return myProgramComboBox->getNumItems();
}


bool
GNETLSEditorFrame::TLSPrograms::checkHaveModifications() const {
    return myHaveModifications;
}


void
GNETLSEditorFrame::TLSPrograms::markAsModified() {
    myHaveModifications = true;
    myTLSEditorParent->updateModules();
}


NBTrafficLightDefinition*
GNETLSEditorFrame::TLSPrograms::getCurrentTLSPrograms() const {
    // find TLS definition
    for (const auto& TLSPrograms : myTLSPrograms) {
        if (TLSPrograms->getProgramID() == myProgramComboBox->getText().text()) {
            return TLSPrograms;
        }
    }
    throw ProcessError(TL("TLSPrograms cannot be found"));
}


const std::string
GNETLSEditorFrame::TLSPrograms::getCurrentTLSProgramID() const {
    if (myProgramComboBox->getNumItems() == 0) {
        return "";
    } else {
        return myProgramComboBox->getText().text();
    }
}


void
GNETLSEditorFrame::TLSPrograms::discardChanges(const bool editJunctionAgain) {
    // get junction copy
    auto currentJunction = myTLSEditorParent->myTLSJunction->getCurrentJunction();
    if (currentJunction != nullptr) {
        myTLSEditorParent->getViewNet()->getUndoList()->abortAllChangeGroups();
        myTLSEditorParent->cleanup();
        myTLSEditorParent->getViewNet()->updateViewNet();
        // edit junction again
        if (editJunctionAgain) {
            myTLSEditorParent->editJunction(currentJunction);
        }
    }
    myTLSEditorParent->updateModules();
}


long
GNETLSEditorFrame::TLSPrograms::onCmdCreate(FXObject*, FXSelector, void*) {
    // get current edited junction (needed because onCmdDiscardChanges clear junction)
    GNEJunction* currentJunction = myTLSEditorParent->myTLSJunction->getCurrentJunction();
    // abort because we onCmdOk assumes we wish to save an edited definition
    discardChanges(false);
    // check number of edges
    if (currentJunction->getGNEIncomingEdges().empty() && currentJunction->getGNEOutgoingEdges().empty()) {
        // open question box
        FXMessageBox::warning(this, MBOX_OK,
                              TL("TLS cannot be created"), "%s",
                              (TL("Traffic Light cannot be created because junction must have") + std::string("\n") +
                               TL("at least one incoming edge and one outgoing edge.")).c_str());
        return 1;
    }
    // check number of connections
    if (currentJunction->getGNEConnections().empty()) {
        // open question box
        FXMessageBox::warning(this, MBOX_OK,
                              TL("TLS cannot be created"), "%s",
                              (TL("Traffic Light cannot be created because junction") + std::string("\n") +
                               TL("must have at least one connection.")).c_str());
        return 1;
    }
    // check uncontrolled connections
    bool connectionControlled = false;
    for (const auto& connection : currentJunction->getGNEConnections()) {
        if (!connection->getNBEdgeConnection().uncontrolled) {
            connectionControlled = true;
        }
    }
    if (!connectionControlled) {
        // open question box
        FXMessageBox::warning(this, MBOX_OK,
                              TL("TLS cannot be created"), "%s",
                              (TL("Traffic Light cannot be created because junction") + std::string("\n") +
                               TL("must have at least one controlled connection.")).c_str());
        return 1;
    }
    // all checks ok, then create TLS in junction
    createTLS(currentJunction);
    // edit junction
    myTLSEditorParent->editJunction(currentJunction);
    // switch to the last program
    myProgramComboBox->setCurrentItem(myProgramComboBox->getNumItems() - 1, TRUE);
    myTLSEditorParent->updateModules();
    return 1;
}


long
GNETLSEditorFrame::TLSPrograms::onCmdDelete(FXObject*, FXSelector, void*) {
    // get current junction
    GNEJunction* currentJunction = myTLSEditorParent->myTLSJunction->getCurrentJunction();
    // get current edited tlDef
    NBTrafficLightDefinition* tlDef = myTLSEditorParent->myTLSPrograms->getCurrentTLSPrograms();
    // check if remove entire TLS or only one program
    const bool changeJunctionType = (myTLSEditorParent->myTLSPrograms->getNumberOfPrograms() == 1);
    // abort because onCmdOk assumes we wish to save an edited definition
    discardChanges(false);
    // check if change junction type
    if (changeJunctionType) {
        currentJunction->setAttribute(SUMO_ATTR_TYPE, toString(SumoXMLNodeType::PRIORITY), myTLSEditorParent->getViewNet()->getUndoList());
    } else {
        // just remove TLDef
        myTLSEditorParent->getViewNet()->getUndoList()->add(new GNEChange_TLS(currentJunction, tlDef, false), true);
        // edit junction again
        myTLSEditorParent->editJunction(currentJunction);
    }
    myTLSEditorParent->updateModules();
    return 1;
}


long
GNETLSEditorFrame::TLSPrograms::onCmdResetCurrentProgram(FXObject*, FXSelector, void*) {
    // obtain junction and old definitions
    GNEJunction* junction = myTLSEditorParent->myTLSJunction->getCurrentJunction();
    NBTrafficLightDefinition* oldDef = myTLSEditorParent->myTLSPrograms->getCurrentTLSPrograms();
    const std::string programID = oldDef->getProgramID();
    // discard changes
    discardChanges(false);
    // begin undo
    myTLSEditorParent->getViewNet()->getUndoList()->begin(GUIIcon::MODETLS, TL("reset current program"));
    // remove old definition
    myTLSEditorParent->getViewNet()->getUndoList()->add(new GNEChange_TLS(junction, oldDef, false), true);
    // create new definition, and add it
    NBOwnTLDef* newDef = new NBOwnTLDef(oldDef->getID(), oldDef->getNodes(), oldDef->getOffset(), oldDef->getType());
    newDef->setProgramID(programID);
    myTLSEditorParent->getViewNet()->getUndoList()->add(new GNEChange_TLS(junction, newDef, true, true), true);
    // set old index
    // end undo
    myTLSEditorParent->getViewNet()->getUndoList()->end();
    // inspect junction again
    myTLSEditorParent->editJunction(junction);
    // switch to programID
    int index = -1;
    for (int i = 0; i < myProgramComboBox->getNumItems(); i++) {
        if (myProgramComboBox->getItemText(i) == programID) {
            index = i;
        }
    }
    if (index != -1) {
        myProgramComboBox->setCurrentItem(index, TRUE);
    }
    myTLSEditorParent->updateModules();
    return 1;
}


long
GNETLSEditorFrame::TLSPrograms::onCmdResetAll(FXObject*, FXSelector, void*) {
    // obtain junction and old definitions
    GNEJunction* junction = myTLSEditorParent->myTLSJunction->getCurrentJunction();
    NBTrafficLightDefinition* oldDef = myTLSEditorParent->myTLSPrograms->getCurrentTLSPrograms();
    // get a list of all affected nodes
    std::vector<GNEJunction*> TLSJunctions;
    for (const auto& TLSNode : oldDef->getNodes()) {
        TLSJunctions.push_back(myTLSEditorParent->getViewNet()->getNet()->getAttributeCarriers()->retrieveJunction(TLSNode->getID()));
    }
    // discard all previous changes
    discardChanges(false);
    // begin undo
    myTLSEditorParent->getViewNet()->getUndoList()->begin(GUIIcon::MODETLS, TL("reset TLS"));
    // set junction as priority (this will also remove all program, see GNEJunction::setJunctionType)
    for (const auto& TLSJunction : TLSJunctions) {
        TLSJunction->setAttribute(SUMO_ATTR_TYPE, toString(SumoXMLNodeType::PRIORITY), myTLSEditorParent->getViewNet()->getUndoList());
    }
    // create TLS in junction
    createTLS(junction);
    // set TLS in all other junctions
    for (const auto& TLSJunction : TLSJunctions) {
        if (TLSJunction != junction) {
            TLSJunction->setAttribute(SUMO_ATTR_TYPE, TLSJunction->getAttribute(SUMO_ATTR_TYPE), myTLSEditorParent->getViewNet()->getUndoList());
            TLSJunction->setAttribute(SUMO_ATTR_TLID, TLSJunction->getAttribute(SUMO_ATTR_TLID), myTLSEditorParent->getViewNet()->getUndoList());
        }
    }
    // end undo
    myTLSEditorParent->getViewNet()->getUndoList()->end();
    // edit junction
    myTLSEditorParent->editJunction(junction);
    return 1;
}


long
GNETLSEditorFrame::TLSPrograms::onCmdDefSwitchTLSProgram(FXObject*, FXSelector, void*) {
    // check if program is valid (user may input arbitrary text)
    bool found = false;
    for (const auto& TLSPrograms : myTLSPrograms) {
        if (TLSPrograms->getProgramID() == myProgramComboBox->getText().text()) {
            found = true;
            break;
        }
    }
    if (!found) {
        // reset field to a valid program
        myProgramComboBox->setText(myTLSPrograms.front()->getProgramID().c_str());
    }
    switchProgram();
    return 1;
}


long
GNETLSEditorFrame::TLSPrograms::onCmdSaveChanges(FXObject*, FXSelector, void*) {
    // get junction copy
    const auto currentJunction = myTLSEditorParent->myTLSJunction->getCurrentJunction();
    // get current program
    const auto currentProgram = myProgramComboBox->getCurrentItem();
    // check that junction is valid
    if (currentJunction != nullptr) {
        const auto oldDefinition = getCurrentTLSPrograms();
        std::vector<NBNode*> nodes = oldDefinition->getNodes();
        GNEUndoList* undoList = myTLSEditorParent->getViewNet()->getUndoList();
        for (const auto& node : nodes) {
            GNEJunction* junction = myTLSEditorParent->getViewNet()->getNet()->getAttributeCarriers()->retrieveJunction(node->getID());
            undoList->add(new GNEChange_TLS(junction, oldDefinition, false), true);
            undoList->add(new GNEChange_TLS(junction, myTLSEditorParent->myEditedDef, true), true);
            // synchronize crossing tl-indices in case they were changed (via onCmdCleanStates)
            for (const auto& gc : junction->getGNECrossings()) {
                NBNode::Crossing* c = gc->getNBCrossing();
                if (c) {
                    gc->setAttribute(SUMO_ATTR_TLLINKINDEX, toString(c->tlLinkIndex), undoList);
                    gc->setAttribute(SUMO_ATTR_TLLINKINDEX2, toString(c->tlLinkIndex2), undoList);
                }
            }


        }
        // end change
        myTLSEditorParent->getViewNet()->getUndoList()->end();
        // mark as saved
        myHaveModifications = false;
        // reset myEditedDef (because is already in eine undo-list)
        myTLSEditorParent->myEditedDef = nullptr;
        myTLSEditorParent->cleanup();
        myTLSEditorParent->getViewNet()->updateViewNet();
        // edit junction again
        myTLSEditorParent->editJunction(currentJunction);
        // change program
        myProgramComboBox->setCurrentItem(currentProgram, TRUE);
    } else {
        // discard changes inspecting junction again
        discardChanges(true);
    }
    myTLSEditorParent->updateModules();
    return 1;
}


long
GNETLSEditorFrame::TLSPrograms::onCmdDiscardChanges(FXObject*, FXSelector, void*) {
    // discard changes inspecting junction again
    discardChanges(true);
    return 1;
}


void
GNETLSEditorFrame::TLSPrograms::createTLS(GNEJunction* junction) {
    // get current TLS program id
    const auto currentTLS = getCurrentTLSProgramID();
    // check conditions
    if (junction == nullptr) {
        throw ProcessError("junction cannot be null");
    } else if (junction->getAttribute(SUMO_ATTR_TYPE) != toString(SumoXMLNodeType::TRAFFIC_LIGHT)) {
        // set junction as TLS
        junction->setAttribute(SUMO_ATTR_TYPE, toString(SumoXMLNodeType::TRAFFIC_LIGHT), myTLSEditorParent->getViewNet()->getUndoList());
    } else if (junction->getNBNode()->isTLControlled()) {
        // use existing traffic light as template for type, signal groups, controlled nodes etc
        NBTrafficLightDefinition* tpl = nullptr;
        for (const auto& TLS : junction->getNBNode()->getControllingTLS()) {
            if (TLS->getProgramID() == currentTLS) {
                tpl = TLS;
            }
        }
        // if template is empty, use first TLS
        if (tpl == nullptr) {
            tpl = *junction->getNBNode()->getControllingTLS().begin();
        }
        // create new logic
        NBTrafficLightLogic* newLogic = tpl->compute(OptionsCont::getOptions());
        // create new TLDef
        NBLoadedSUMOTLDef* newDef = new NBLoadedSUMOTLDef(*tpl, *newLogic);
        NBTrafficLightLogicCont& tllCont = myTLSEditorParent->getViewNet()->getNet()->getTLLogicCont();
        newDef->setProgramID(tllCont.getNextProgramID(newDef->getID()));
        // remove new logic
        delete newLogic;
        // add it using GNEChange_TLS
        myTLSEditorParent->getViewNet()->getUndoList()->begin(GUIIcon::MODETLS, TLF("duplicate program '%' of traffic light '%'", tpl->getProgramID(), tpl->getID()));
        for (NBNode* node : newDef->getNodes()) {
            GNEJunction* j = myTLSEditorParent->getViewNet()->getNet()->getAttributeCarriers()->retrieveJunction(node->getID());
            // not forcing insertion because we already ensured a new id and multiple junctions will attempt insertion
            myTLSEditorParent->getViewNet()->getUndoList()->add(new GNEChange_TLS(j, newDef, true, false), true);
        }
        myTLSEditorParent->getViewNet()->getUndoList()->end();
    } else {
        // for some reason the traffic light was not built, try again
        myTLSEditorParent->getViewNet()->getUndoList()->add(new GNEChange_TLS(junction, nullptr, true, true), true);
    }
}


bool
GNETLSEditorFrame::TLSPrograms::switchProgram() {
    if (myTLSEditorParent->myTLSJunction->getCurrentJunction() == nullptr) {
        throw ProcessError("Junction cannot be NULL");
    } else {
        // reset save flag
        myHaveModifications = false;
        // get current definition
        NBTrafficLightDefinition* tlDef = getCurrentTLSPrograms();
        // logic may not have been recomputed yet. recompute to be sure
        NBTrafficLightLogicCont& tllCont = myTLSEditorParent->getViewNet()->getNet()->getTLLogicCont();
        // compute junction
        myTLSEditorParent->getViewNet()->getNet()->computeJunction(myTLSEditorParent->myTLSJunction->getCurrentJunction());
        // obtain TrafficLight logic vinculated with tlDef
        NBTrafficLightLogic* tllogic = tllCont.getLogic(tlDef->getID(), tlDef->getProgramID());
        // check that tllLogic exist
        if (tllogic != nullptr) {
            // now we can be sure that the tlDef is up to date (i.e. re-guessed)
            myTLSEditorParent->buildInternalLanes(tlDef);
            // create working duplicate from original def
            delete myTLSEditorParent->myEditedDef;
            myTLSEditorParent->myEditedDef = new NBLoadedSUMOTLDef(*tlDef, *tllogic);
            // set values
            myTLSEditorParent->myTLSAttributes->setOffset(myTLSEditorParent->myEditedDef->getLogic()->getOffset());
            myTLSEditorParent->myTLSAttributes->setParameters(myTLSEditorParent->myEditedDef->getLogic()->getParametersStr());
            // init phaseTable with the new TLS
            myTLSEditorParent->myTLSPhases->initPhaseTable();
        } else {
            // tlDef has no valid logic (probably because id does not control any links
            discardChanges(false);
            myTLSEditorParent->getViewNet()->setStatusBarText(TL("Traffic Light does not control any links"));
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// GNETLSEditorFrame::TLSPhases - methods
// ---------------------------------------------------------------------------

GNETLSEditorFrame::TLSPhases::TLSPhases(GNETLSEditorFrame* TLSEditorParent) :
    MFXGroupBoxModule(TLSEditorParent, TL("Phases"), MFXGroupBoxModule::Options::COLLAPSIBLE | MFXGroupBoxModule::Options::EXTENSIBLE),
    myTLSEditorParent(TLSEditorParent) {
    // create GNETLSTable
    myPhaseTable = new GNETLSTable(this);
    // hide phase table
    myPhaseTable->hide();
    FXHorizontalFrame* phaseButtons = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrameUniform);
    FXVerticalFrame* col1 = new FXVerticalFrame(phaseButtons, GUIDesignAuxiliarHorizontalFrame); // left button columm
    FXVerticalFrame* col2 = new FXVerticalFrame(phaseButtons, GUIDesignAuxiliarHorizontalFrame); // right button column
    // create cleanup states button
    myCleanStatesButton = new MFXButtonTooltip(col1, TLSEditorParent->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
            TL("Clean States") + std::string("\t") +
            TL("Clean unused states from all phase") + std::string("\t") +
            TL("Clean unused states from all phase. (Not allowed for multiple programs)"),
            nullptr, this, MID_GNE_TLSFRAME_PHASES_CLEANUP, GUIDesignButton);
    myCleanStatesButton->disable();
    // add unused states button
    myAddStates = new MFXButtonTooltip(col2, TLSEditorParent->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
                                       TL("Add States") + std::string("\t") +
                                       TL("Extend the state vector for all phases by one entry") + std::string("\t") +
                                       TL("Extend the state vector for all phases by one entry. (Unused until a connection or crossing is assigned to the new index)"),
                                       nullptr, this, MID_GNE_TLSFRAME_PHASES_ADDUNUSED, GUIDesignButton);
    myAddStates->disable();
    // group states button
    myGroupSignalsButton = new MFXButtonTooltip(col1, TLSEditorParent->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
            TL("Group Sig.") + std::string("\t") +
            TL("Shorten state definition by letting connections with the same signal states use the same index") + std::string("\t") +
            TL("Shorten state definition by letting connections with the same signal states use the same index. (Not allowed for multiple programs)"),
            nullptr, this, MID_GNE_TLSFRAME_PHASES_GROUPSTATES, GUIDesignButton);
    myGroupSignalsButton->disable();
    // ungroup states button
    myUngroupSignalsButton = new MFXButtonTooltip(col2, TLSEditorParent->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
            TL("Ungroup Sig.") + std::string("\t") +
            TL("Let every connection use a distinct index (reverse state grouping)") + std::string("\t") +
            TL("Let every connection use a distinct index (reverse state grouping). (Not allowed for multiple programs)"),
            nullptr, this, MID_GNE_TLSFRAME_PHASES_UNGROUPSTATES, GUIDesignButton);
    myUngroupSignalsButton->disable();
    // show TLSFile
    show();
}


GNETLSEditorFrame::TLSPhases::~TLSPhases() {
}


void
GNETLSEditorFrame::TLSPhases::updateTLSPhases() {
    if (myTLSEditorParent->myTLSAttributes->isSetDetectorsToggleButtonEnabled()) {
        // selecting E1, disable buttons
        myPhaseTable->disable();
        myCleanStatesButton->disable();
        myAddStates->disable();
        myGroupSignalsButton->disable();
        myUngroupSignalsButton->disable();
    } else if (myTLSEditorParent->myTLSJunction->isJoiningJunctions()) {
        // joining TLSs, disable buttons
        myPhaseTable->disable();
        myCleanStatesButton->disable();
        myAddStates->disable();
        myGroupSignalsButton->disable();
        myUngroupSignalsButton->disable();
        myUngroupSignalsButton->disable();
    } else if (myTLSEditorParent->myTLSPrograms->getNumberOfPrograms() == 0) {
        // no TLSs, disable buttons
        myPhaseTable->disable();
        myCleanStatesButton->disable();
        myAddStates->disable();
        myGroupSignalsButton->disable();
        myUngroupSignalsButton->disable();
    } else if (myTLSEditorParent->myEditedDef == nullptr) {
        // no edited definition, disable buttons
        myPhaseTable->disable();
        myCleanStatesButton->disable();
        myAddStates->disable();
        myGroupSignalsButton->disable();
        myUngroupSignalsButton->disable();
    } else {
        myPhaseTable->enable();
        myCleanStatesButton->enable();
        myAddStates->enable();
        myGroupSignalsButton->enable();
        if (myTLSEditorParent->myEditedDef->usingSignalGroups()) {
            myUngroupSignalsButton->enable();
        } else {
            myUngroupSignalsButton->disable();
        }
    }
}


void
GNETLSEditorFrame::TLSPhases::showTLSPhases() {
    show();
}


void
GNETLSEditorFrame::TLSPhases::hideTLSPhases() {
    hide();
}


GNETLSEditorFrame*
GNETLSEditorFrame::TLSPhases::getTLSEditorParent() const {
    return myTLSEditorParent;
}


GNETLSTable*
GNETLSEditorFrame::TLSPhases::getPhaseTable() const {
    return myPhaseTable;
}


void
GNETLSEditorFrame::TLSPhases::initPhaseTable() {
    // first clear table
    clearPhaseTable();
    if (myTLSEditorParent->myTLSPrograms->getNumberOfPrograms() > 0) {
        if (myTLSEditorParent->myEditedDef->getType() == TrafficLightType::STATIC) {
            initStaticPhaseTable();
        } else if (myTLSEditorParent->myEditedDef->getType() == TrafficLightType::ACTUATED) {
            initActuatedPhaseTable();
        } else if (myTLSEditorParent->myEditedDef->getType() == TrafficLightType::DELAYBASED) {
            initDelayBasePhaseTable();
        } else if (myTLSEditorParent->myEditedDef->getType() == TrafficLightType::NEMA) {
            initNEMAPhaseTable();
        }
        // select first element (by default)
        myPhaseTable->selectRow(0);
        // recalc width and show
        myPhaseTable->recalcTableWidth();
        myPhaseTable->show();
    } else {
        myPhaseTable->hide();
    }
    update();
}


void
GNETLSEditorFrame::TLSPhases::clearPhaseTable() {
    myPhaseTable->clearTable();
}


bool
GNETLSEditorFrame::TLSPhases::changePhaseValue(const int col, const int row, const std::string& value) {
    // Declare columns
    int colDuration = 1;
    int colState = -1;
    int colNext = -1;
    int colName = -1;
    int colMinDur = -1;
    int colMaxDur = -1;
    int colEarliestEnd = -1;
    int colLatestEnd = -1;
    int colVehExt = -1;
    int colYellow = -1;
    int colRed = -1;
    // set columns depending of traffic light type
    if (myTLSEditorParent->myEditedDef->getType() == TrafficLightType::STATIC) {
        colState = 2;
        colNext = 3;
        colName = 4;
    } else if (myTLSEditorParent->myEditedDef->getType() == TrafficLightType::ACTUATED) {
        colMinDur = 2;
        colMaxDur = 3;
        colState = 4;
        colEarliestEnd = 5;
        colLatestEnd = 6;
        colNext = 7;
        colName = 8;
    } else if (myTLSEditorParent->myEditedDef->getType() == TrafficLightType::DELAYBASED) {
        colMinDur = 2;
        colMaxDur = 3;
        colState = 4;
        colNext = 5;
        colName = 6;
    } else if (myTLSEditorParent->myEditedDef->getType() == TrafficLightType::NEMA) {
        colMinDur = 2;
        colMaxDur = 3;
        colState = 4;
        colVehExt = 5;
        colYellow = 6;
        colRed = 7;
        colNext = 8;
        colName = 9;
    }
    // check column
    if (col == colDuration) {
        return setDuration(col, row, value);
    } else if (col == colState) {
        return setState(col, row, value);
    } else if (col == colNext) {
        return setNext(row, value);
    } else if (col == colName) {
        return setName(row, value);
    } else if (col == colMinDur) {
        return setMinDur(row, value);
    } else if (col == colMaxDur) {
        return setMaxDur(row, value);
    } else if (col == colEarliestEnd) {
        return setEarliestEnd(row, value);
    } else if (col == colLatestEnd) {
        return setLatestEnd(row, value);
    } else if (col == colVehExt) {
        return setVehExt(row, value);
    } else if (col == colYellow) {
        return setYellow(row, value);
    } else if (col == colRed) {
        return setRed(row, value);
    } else {
        throw ProcessError(TL("invalid column"));
    }
}


void
GNETLSEditorFrame::TLSPhases::addPhase(const int row, const char c) {
    // mark TLS as modified
    myTLSEditorParent->myTLSPrograms->markAsModified();
    // build default phase
    const int newIndex = buildDefaultPhase(row);
    // check if override state
    switch (c) {
        case 'r':
        case 'y':
        case 'g':
        case 'G':
            myTLSEditorParent->myEditedDef->getLogic()->overrideState(newIndex, c);
            break;
        default:
            break;
    }
    // int phase table again
    initPhaseTable();
    // mark new row as selected
    myPhaseTable->selectRow(newIndex);
    // set focus in table
    getPhaseTable()->setFocus();
}


void
GNETLSEditorFrame::TLSPhases::duplicatePhase(const int row) {
    // mark TLS as modified
    myTLSEditorParent->myTLSPrograms->markAsModified();
    // build default phase
    const int newIndex = buildDefaultPhase(row);
    // coply old phase in the new phase
    myTLSEditorParent->myEditedDef->getLogic()->copyPhase(row, row + 1);
    // int phase table again
    initPhaseTable();
    // mark new row as selected
    myPhaseTable->selectRow(newIndex);
    // set focus in table
    getPhaseTable()->setFocus();
}


void
GNETLSEditorFrame::TLSPhases::removePhase(const int row) {
    // mark TLS ad modified
    myTLSEditorParent->myTLSPrograms->markAsModified();
    // calculate new row
    const auto newRow = MAX2(0, (row - 1));
    // delete selected row
    myTLSEditorParent->myEditedDef->getLogic()->deletePhase(row);
    // int phase table again
    initPhaseTable();
    // mark new row as selected
    myPhaseTable->selectRow(newRow);
    // set focus in table
    getPhaseTable()->setFocus();
}


void
GNETLSEditorFrame::TLSPhases::movePhaseUp(const int row) {
    // mark TLS ad modified
    myTLSEditorParent->myTLSPrograms->markAsModified();
    // delete selected row
    if (row == 0) {
        myTLSEditorParent->myEditedDef->getLogic()->swapfirstPhase();
    } else {
        myTLSEditorParent->myEditedDef->getLogic()->swapPhase(row, row - 1);
    }
    // int phase table again
    initPhaseTable();
    // mark new row as selected
    if (row == 0) {
        myPhaseTable->selectRow((int)myTLSEditorParent->myEditedDef->getLogic()->getPhases().size() - 1);
    } else {
        myPhaseTable->selectRow(row - 1);
    }
    // set focus in table
    getPhaseTable()->setFocus();
}


void
GNETLSEditorFrame::TLSPhases::movePhaseDown(const int row) {
    // mark TLS ad modified
    myTLSEditorParent->myTLSPrograms->markAsModified();
    // delete selected row
    if (row == (int)myTLSEditorParent->myEditedDef->getLogic()->getPhases().size() - 1) {
        myTLSEditorParent->myEditedDef->getLogic()->swaplastPhase();
    } else {
        myTLSEditorParent->myEditedDef->getLogic()->swapPhase(row, row + 1);
    }
    // int phase table again
    initPhaseTable();
    // mark new row as selected
    if (row == (int)myTLSEditorParent->myEditedDef->getLogic()->getPhases().size() - 1) {
        myPhaseTable->selectRow(0);
    } else {
        myPhaseTable->selectRow(row + 1);
    }
    // set focus in table
    getPhaseTable()->setFocus();
}


void
GNETLSEditorFrame::TLSPhases::updateTLSColoring() {
    // get phase
    const auto& phase = myTLSEditorParent->getPhase(myPhaseTable->getCurrentSelectedRow());
    // need not hold since links could have been deleted somewhere else and indices may be reused
    for (const auto& internalLane : myTLSEditorParent->myInternalLanes) {
        int tlIndex = internalLane.first;
        std::vector<GNEInternalLane*> lanes = internalLane.second;
        LinkState state = LINKSTATE_DEADEND;
        if (tlIndex >= 0 && tlIndex < (int)phase.state.size()) {
            state = (LinkState)phase.state[tlIndex];
        }
        for (const auto& lane : lanes) {
            lane->setLinkState(state);
        }
    }
    // update view net (for coloring)
    myTLSEditorParent->getViewNet()->updateViewNet();
}


long
GNETLSEditorFrame::TLSPhases::onCmdCleanStates(FXObject*, FXSelector, void*) {
    if (myTLSEditorParent->myEditedDef->cleanupStates()) {
        myTLSEditorParent->myTLSPrograms->markAsModified();
    }
    myTLSEditorParent->buildInternalLanes(myTLSEditorParent->myEditedDef);
    initPhaseTable();
    myPhaseTable->setFocus();
    myTLSEditorParent->myTLSPrograms->markAsModified();
    myTLSEditorParent->updateModules();
    return 1;
}


long
GNETLSEditorFrame::TLSPhases::onCmdAddUnusedStates(FXObject*, FXSelector, void*) {
    myTLSEditorParent->myEditedDef->getLogic()->setStateLength(myTLSEditorParent->myEditedDef->getLogic()->getNumLinks() + 1);
    myTLSEditorParent->myTLSPrograms->markAsModified();
    initPhaseTable();
    myPhaseTable->setFocus();
    myTLSEditorParent->updateModules();
    return 1;
}


long
GNETLSEditorFrame::TLSPhases::onCmdGroupStates(FXObject*, FXSelector, void*) {
    myTLSEditorParent->myEditedDef->groupSignals();
    myTLSEditorParent->myTLSPrograms->markAsModified();
    myTLSEditorParent->buildInternalLanes(myTLSEditorParent->myEditedDef);
    initPhaseTable();
    myPhaseTable->setFocus();
    myTLSEditorParent->updateModules();
    return 1;
}


long
GNETLSEditorFrame::TLSPhases::onCmdUngroupStates(FXObject*, FXSelector, void*) {
    myTLSEditorParent->myEditedDef->setParticipantsInformation();
    myTLSEditorParent->myEditedDef->ungroupSignals();
    myTLSEditorParent->myTLSPrograms->markAsModified();
    myTLSEditorParent->buildInternalLanes(myTLSEditorParent->myEditedDef);
    initPhaseTable();
    myPhaseTable->setFocus();
    myTLSEditorParent->updateModules();
    return 1;
}


void
GNETLSEditorFrame::TLSPhases::initStaticPhaseTable() {
    // declare constants for columns
    const int colDuration = 1;
    const int colState = 2;
    const int colNext = 3;
    const int colName = 4;
    // get phases
    const auto& phases = myTLSEditorParent->myEditedDef->getLogic()->getPhases();
    // adjust table
    myPhaseTable->setTableSize("sup-midtb", (int)phases.size());
    // fill rows
    for (int row = 0; row < (int)phases.size(); row++) {
        myPhaseTable->setItemText(row, colDuration, getSteps2Time(phases.at(row).duration).c_str());
        myPhaseTable->setItemText(row, colState, phases.at(row).state.c_str());
        myPhaseTable->setItemText(row, colNext, phases.at(row).next.size() > 0 ? toString(phases.at(row).next).c_str() : "");
        myPhaseTable->setItemText(row, colName, phases.at(row).name.c_str());
    }
    // set columns
    myPhaseTable->setColumnLabelTop(colDuration, "dur");
    myPhaseTable->setColumnLabelTop(colState, "state");
    myPhaseTable->setColumnLabelTop(colNext, "next");
    myPhaseTable->setColumnLabelTop(colName, "name");
    // set bot labels
    updateCycleDuration(colDuration);
    updateStateSize(colState);
    // set focus
    myPhaseTable->setFocus();
}


void
GNETLSEditorFrame::TLSPhases::initActuatedPhaseTable() {
    // declare constants for columns
    const int colDuration = 1;
    const int colMinDur = 2;
    const int colMaxDur = 3;
    const int colState = 4;
    const int colEarliestEnd = 5;
    const int colLatestEnd = 6;
    const int colNext = 7;
    const int colName = 8;
    // get phases
    const auto& phases = myTLSEditorParent->myEditedDef->getLogic()->getPhases();
    // adjust table
    myPhaseTable->setTableSize("suffpff-midtb", (int)phases.size());
    // fill rows
    for (int row = 0; row < (int)phases.size(); row++) {
        myPhaseTable->setItemText(row, colDuration, getSteps2Time(phases.at(row).duration).c_str());
        myPhaseTable->setItemText(row, colMinDur, varDurString(phases.at(row).minDur).c_str());
        myPhaseTable->setItemText(row, colMaxDur, varDurString(phases.at(row).maxDur).c_str());
        myPhaseTable->setItemText(row, colState, phases.at(row).state.c_str());
        myPhaseTable->setItemText(row, colEarliestEnd, varDurString(phases.at(row).earliestEnd).c_str());
        myPhaseTable->setItemText(row, colLatestEnd, varDurString(phases.at(row).latestEnd).c_str());
        myPhaseTable->setItemText(row, colNext, phases.at(row).next.size() > 0 ? toString(phases.at(row).next).c_str() : "");
        myPhaseTable->setItemText(row, colName, phases.at(row).name.c_str());
    }
    // set columns
    myPhaseTable->setColumnLabelTop(colDuration, "dur");
    myPhaseTable->setColumnLabelTop(colMinDur, "min");
    myPhaseTable->setColumnLabelTop(colMaxDur, "max");
    myPhaseTable->setColumnLabelTop(colEarliestEnd, "ear.end", "earlyEnd");
    myPhaseTable->setColumnLabelTop(colLatestEnd, "lat.end", "latestEnd");
    myPhaseTable->setColumnLabelTop(colState, "state");
    myPhaseTable->setColumnLabelTop(colNext, "next");
    myPhaseTable->setColumnLabelTop(colName, "name");
    // set bot labels
    updateCycleDuration(colDuration);
    updateStateSize(colState);
    // set focus
    myPhaseTable->setFocus();
}


void
GNETLSEditorFrame::TLSPhases::initDelayBasePhaseTable() {
    // declare constants for columns
    const int colDuration = 1;
    const int colMinDur = 2;
    const int colMaxDur = 3;
    const int colState = 4;
    const int colNext = 5;
    const int colName = 6;
    // get phases
    const auto& phases = myTLSEditorParent->myEditedDef->getLogic()->getPhases();
    // adjust table
    myPhaseTable->setTableSize("suffp-midtb", (int)phases.size());
    // fill rows
    for (int row = 0; row < (int)phases.size(); row++) {
        myPhaseTable->setItemText(row, colDuration, getSteps2Time(phases.at(row).duration).c_str());
        myPhaseTable->setItemText(row, colMinDur, varDurString(phases.at(row).minDur).c_str());
        myPhaseTable->setItemText(row, colMaxDur, varDurString(phases.at(row).maxDur).c_str());
        myPhaseTable->setItemText(row, colState, phases.at(row).state.c_str());
        myPhaseTable->setItemText(row, colNext, phases.at(row).next.size() > 0 ? toString(phases.at(row).next).c_str() : "");
        myPhaseTable->setItemText(row, colName, phases.at(row).name.c_str());
    }
    // set columns
    myPhaseTable->setColumnLabelTop(colDuration, "dur");
    myPhaseTable->setColumnLabelTop(colMinDur, "min");
    myPhaseTable->setColumnLabelTop(colMaxDur, "max");
    myPhaseTable->setColumnLabelTop(colState, "state");
    myPhaseTable->setColumnLabelTop(colNext, "next");
    myPhaseTable->setColumnLabelTop(colName, "name");
    // set bot labels
    updateCycleDuration(colDuration);
    updateStateSize(colState);
    // set focus
    myPhaseTable->setFocus();
}


void
GNETLSEditorFrame::TLSPhases::initNEMAPhaseTable() {
    // declare constants for columns
    const int colDuration = 1;
    const int colMinDur = 2;
    const int colMaxDur = 3;
    const int colState = 4;
    const int colVehExt = 5;
    const int colYellow = 6;
    const int colRed = 7;
    const int colNext = 8;
    const int colName = 9;
    // get phases
    const auto& phases = myTLSEditorParent->myEditedDef->getLogic()->getPhases();
    // adjust table
    myPhaseTable->setTableSize("suffpfff-midtb", (int)phases.size());
    // fill rows
    for (int row = 0; row < (int)phases.size(); row++) {
        myPhaseTable->setItemText(row, colDuration, getSteps2Time(phases.at(row).duration).c_str());
        myPhaseTable->setItemText(row, colMinDur, varDurString(phases.at(row).minDur).c_str());
        myPhaseTable->setItemText(row, colMaxDur, varDurString(phases.at(row).maxDur).c_str());
        myPhaseTable->setItemText(row, colState, phases.at(row).state.c_str());
        myPhaseTable->setItemText(row, colVehExt, varDurString(phases.at(row).vehExt).c_str());
        myPhaseTable->setItemText(row, colYellow, varDurString(phases.at(row).yellow).c_str());
        myPhaseTable->setItemText(row, colRed, varDurString(phases.at(row).red).c_str());
        myPhaseTable->setItemText(row, colNext, phases.at(row).next.size() > 0 ? toString(phases.at(row).next).c_str() : "");
        myPhaseTable->setItemText(row, colName, phases.at(row).name.c_str());
    }
    // set columns
    myPhaseTable->setColumnLabelTop(colDuration, "dur");
    myPhaseTable->setColumnLabelTop(colMinDur, "min");
    myPhaseTable->setColumnLabelTop(colMaxDur, "max");
    myPhaseTable->setColumnLabelTop(colState, "state");
    myPhaseTable->setColumnLabelTop(colVehExt, "vehExt", "vehicle extension");
    myPhaseTable->setColumnLabelTop(colYellow, "yellow");
    myPhaseTable->setColumnLabelTop(colRed, "red");
    myPhaseTable->setColumnLabelTop(colNext, "next");
    myPhaseTable->setColumnLabelTop(colName, "name");
    // set bot labels
    updateCycleDuration(colDuration);
    updateStateSize(colState);
    // set focus
    myPhaseTable->setFocus();
}


int
GNETLSEditorFrame::TLSPhases::buildDefaultPhase(const int row) {
    // get option container
    const auto& neteditOptions = OptionsCont::getOptions();
    // check if TLS is static
    const bool TLSStatic = (myTLSEditorParent->myEditedDef->getType() == TrafficLightType::STATIC);
    const bool NEMA = (myTLSEditorParent->myEditedDef->getType() == TrafficLightType::NEMA);
    // calculate new index
    const int newIndex = row + 1;
    // duplicate current row
    auto duration = getSUMOTime(myPhaseTable->getItemText(row, 1));
    const auto oldState = myPhaseTable->getItemText(row, TLSStatic ? 2 : 4);
    auto state = oldState;
    // update crossingINdices
    std::set<int> crossingIndices;
    for (const auto& node : myTLSEditorParent->myEditedDef->getNodes()) {
        for (const auto& crossing : node->getCrossings()) {
            crossingIndices.insert(crossing->tlLinkIndex);
            crossingIndices.insert(crossing->tlLinkIndex2);
        }
    }
    // smart adapations for new state
    bool haveGreen = false;
    bool haveYellow = false;
    for (const auto& linkStateChar : state) {
        if ((linkStateChar == LINKSTATE_TL_GREEN_MAJOR) || (linkStateChar == LINKSTATE_TL_GREEN_MINOR)) {
            haveGreen = true;
        } else if ((linkStateChar == LINKSTATE_TL_YELLOW_MAJOR) || (linkStateChar == LINKSTATE_TL_YELLOW_MINOR)) {
            haveYellow = true;
        }
    }
    if (haveGreen && haveYellow) {
        // guess left-mover state
        duration = TIME2STEPS(neteditOptions.getInt("tls.left-green.time"));
        for (int i = 0; i < (int)state.size(); i++) {
            if ((state[i] == LINKSTATE_TL_YELLOW_MAJOR) || (state[i] == LINKSTATE_TL_YELLOW_MINOR)) {
                state[i] = LINKSTATE_TL_RED;
            } else if (state[i] == LINKSTATE_TL_GREEN_MINOR) {
                state[i] = LINKSTATE_TL_GREEN_MAJOR;
            }
        }
    } else if (haveGreen) {
        // guess yellow state
        myTLSEditorParent->myEditedDef->setParticipantsInformation();
        duration = TIME2STEPS(myTLSEditorParent->myEditedDef->computeBrakingTime(neteditOptions.getFloat("tls.yellow.min-decel")));
        for (int i = 0; i < (int)state.size(); i++) {
            if ((state[i] == LINKSTATE_TL_GREEN_MAJOR) || (state[i] == LINKSTATE_TL_GREEN_MINOR)) {
                if (crossingIndices.count(i) == 0) {
                    state[i] = LINKSTATE_TL_YELLOW_MINOR;
                } else {
                    state[i] = LINKSTATE_TL_RED;
                }
            }
        }
    } else if (haveYellow) {
        duration = TIME2STEPS(neteditOptions.isDefault("tls.allred.time") ? 2 :  neteditOptions.getInt("tls.allred.time"));
        // guess all-red state
        for (int i = 0; i < (int)state.size(); i++) {
            if ((state[i] == LINKSTATE_TL_YELLOW_MAJOR) || (state[i] == LINKSTATE_TL_YELLOW_MINOR)) {
                state[i] = LINKSTATE_TL_RED;
            }
        }
    }
    // fix continuous green states
    const int nextIndex = (myPhaseTable->getNumRows() > newIndex) ? newIndex : 0;
    const std::string state2 = myPhaseTable->getItemText(nextIndex, (TLSStatic ? 2 : 4));
    for (int i = 0; i < (int)state.size(); i++) {
        if (((oldState[i] == LINKSTATE_TL_GREEN_MAJOR) || (oldState[i] == LINKSTATE_TL_GREEN_MINOR)) &&
                ((state2[i] == LINKSTATE_TL_GREEN_MAJOR) || (state2[i] == LINKSTATE_TL_GREEN_MINOR))) {
            state[i] = oldState[i];
        }
    }
    // add new step
    if (NEMA) {
        myTLSEditorParent->myEditedDef->getLogic()->addStep(string2time("90"), state, string2time("5"), string2time("50"),
                NBTrafficLightDefinition::UNSPECIFIED_DURATION, NBTrafficLightDefinition::UNSPECIFIED_DURATION,
                string2time("2"), string2time("3"), string2time("2"), "1", std::vector<int>(), newIndex);
    } else {
        myTLSEditorParent->myEditedDef->getLogic()->addStep(duration, state, std::vector<int>(), "", newIndex);
    }
    // return new index
    return newIndex;
}


bool
GNETLSEditorFrame::TLSPhases::setDuration(const int col, const int row, const std::string& value) {
    // check value
    if (value.empty()) {
        // input empty, reset
        getPhaseTable()->setItemText(row, col, getSteps2Time(myTLSEditorParent->getPhase(row).duration).c_str());
        return true;
    } else if (GNEAttributeCarrier::canParse<double>(value)) {
        const auto duration = getSUMOTime(value);
        // check that duration > 0
        if (duration > 0) {
            myTLSEditorParent->myEditedDef->getLogic()->setPhaseDuration(row, duration);
            myTLSEditorParent->myTLSPrograms->markAsModified();
            // update Cycle duration
            updateCycleDuration(col);
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}


bool
GNETLSEditorFrame::TLSPhases::setState(const int col, const int row, const std::string& value) {
    // get state
    const auto& phase = myTLSEditorParent->getPhase(row);
    // declare new state. If value is empty, use previous value (reset)
    const auto newState = value.empty() ? phase.state : value;
    // insert phase
    try {
        myTLSEditorParent->myEditedDef->getLogic()->addStep(phase.duration, newState, phase.next, phase.name, row);
    } catch (ProcessError&) {
        // invalid character in newState
        return false;
    }
    // delete next phase
    try {
        myTLSEditorParent->myEditedDef->getLogic()->deletePhase(row + 1);
    } catch (InvalidArgument&) {
        WRITE_ERROR(TL("Error deleting phase '") + toString(row + 1) + "'");
        return false;
    }
    // mark TLS as modified depending of value
    if (value.size() > 0) {
        myTLSEditorParent->myTLSPrograms->markAsModified();
        // select row
        myPhaseTable->selectRow(row);
    } else {
        // input empty, reset
        getPhaseTable()->setItemText(row, col, newState);
    }
    // update state size
    updateStateSize(col);
    return true;
}


bool
GNETLSEditorFrame::TLSPhases::setNext(const int row, const std::string& value) {
    // check next
    if (GNEAttributeCarrier::canParse<std::vector<int> >(value)) {
        const auto nextEdited = GNEAttributeCarrier::parse<std::vector<int> >(value);
        for (const auto nextPhase : nextEdited) {
            if ((nextPhase < 0) || (nextPhase >= myPhaseTable->getNumRows())) {
                return false;
            }
        }
        // set new next
        myTLSEditorParent->myEditedDef->getLogic()->setPhaseNext(row, nextEdited);
        myTLSEditorParent->myTLSPrograms->markAsModified();
        return true;
    } else {
        return false;
    }
}


bool
GNETLSEditorFrame::TLSPhases::setName(const int row, const std::string& value) {
    // update name (currently no check needed)
    myTLSEditorParent->myEditedDef->getLogic()->setPhaseName(row, value);
    myTLSEditorParent->myTLSPrograms->markAsModified();
    return true;
}


bool
GNETLSEditorFrame::TLSPhases::setMinDur(const int row, const std::string& value) {
    // check value
    if (value.empty()) {
        // set empty value
        myTLSEditorParent->myEditedDef->getLogic()->setPhaseMinDuration(row, NBTrafficLightDefinition::UNSPECIFIED_DURATION);
        myTLSEditorParent->myTLSPrograms->markAsModified();
        return true;
    } else if (GNEAttributeCarrier::canParse<double>(value)) {
        const auto minDur = getSUMOTime(value);
        // check that minDur > 0
        if (minDur > 0) {
            myTLSEditorParent->myEditedDef->getLogic()->setPhaseMinDuration(row, minDur);
            myTLSEditorParent->myTLSPrograms->markAsModified();
            return true;
        } else {
            return false;
        }
    } else if (StringUtils::prune(value).empty()) {
        myTLSEditorParent->myEditedDef->getLogic()->setPhaseMinDuration(row, NBTrafficLightDefinition::UNSPECIFIED_DURATION);
        myTLSEditorParent->myTLSPrograms->markAsModified();
        return true;
    } else {
        return false;
    }
}


bool
GNETLSEditorFrame::TLSPhases::setMaxDur(const int row, const std::string& value) {
    // check value
    if (value.empty()) {
        // set empty value
        myTLSEditorParent->myEditedDef->getLogic()->setPhaseMaxDuration(row, NBTrafficLightDefinition::UNSPECIFIED_DURATION);
        myTLSEditorParent->myTLSPrograms->markAsModified();
        return true;
    } else if (GNEAttributeCarrier::canParse<double>(value)) {
        const auto maxDur = getSUMOTime(value);
        // check that minDur > 0
        if (maxDur > 0) {
            myTLSEditorParent->myEditedDef->getLogic()->setPhaseMaxDuration(row, maxDur);
            myTLSEditorParent->myTLSPrograms->markAsModified();
            return true;
        } else {
            return false;
        }
    } else if (StringUtils::prune(value).empty()) {
        myTLSEditorParent->myEditedDef->getLogic()->setPhaseMaxDuration(row, NBTrafficLightDefinition::UNSPECIFIED_DURATION);
        myTLSEditorParent->myTLSPrograms->markAsModified();
        return true;
    } else {
        return false;
    }
}


bool
GNETLSEditorFrame::TLSPhases::setEarliestEnd(const int row, const std::string& value) {
    // check value
    if (value.empty()) {
        // set empty value
        myTLSEditorParent->myEditedDef->getLogic()->setPhaseEarliestEnd(row, NBTrafficLightDefinition::UNSPECIFIED_DURATION);
        myTLSEditorParent->myTLSPrograms->markAsModified();
        return true;
    } else if (GNEAttributeCarrier::canParse<double>(value)) {
        const auto earliestEnd = getSUMOTime(value);
        // check that earliestEnd > 0
        if (earliestEnd > 0) {
            myTLSEditorParent->myEditedDef->getLogic()->setPhaseEarliestEnd(row, earliestEnd);
            myTLSEditorParent->myTLSPrograms->markAsModified();
            return true;
        } else {
            return false;
        }
    } else if (StringUtils::prune(value).empty()) {
        myTLSEditorParent->myEditedDef->getLogic()->setPhaseEarliestEnd(row, NBTrafficLightDefinition::UNSPECIFIED_DURATION);
        myTLSEditorParent->myTLSPrograms->markAsModified();
        return true;
    } else {
        return false;
    }
}


bool
GNETLSEditorFrame::TLSPhases::setLatestEnd(const int row, const std::string& value) {
    // check value
    if (value.empty()) {
        // set empty value
        myTLSEditorParent->myEditedDef->getLogic()->setPhaseLatestEnd(row, NBTrafficLightDefinition::UNSPECIFIED_DURATION);
        myTLSEditorParent->myTLSPrograms->markAsModified();
        return true;
    } else if (GNEAttributeCarrier::canParse<double>(value)) {
        const auto latestEnd = getSUMOTime(value);
        // check that latestEnd > 0
        if (latestEnd > 0) {
            myTLSEditorParent->myEditedDef->getLogic()->setPhaseLatestEnd(row, latestEnd);
            myTLSEditorParent->myTLSPrograms->markAsModified();
            return true;
        } else {
            return false;
        }
    } else if (StringUtils::prune(value).empty()) {
        myTLSEditorParent->myEditedDef->getLogic()->setPhaseLatestEnd(row, NBTrafficLightDefinition::UNSPECIFIED_DURATION);
        myTLSEditorParent->myTLSPrograms->markAsModified();
        return true;
    } else {
        return false;
    }
}


bool
GNETLSEditorFrame::TLSPhases::setVehExt(const int row, const std::string& value) {
    // check value
    if (value.empty()) {
        // set empty value
        myTLSEditorParent->myEditedDef->getLogic()->setPhaseVehExt(row, NBTrafficLightDefinition::UNSPECIFIED_DURATION);
        myTLSEditorParent->myTLSPrograms->markAsModified();
        return true;
    } else if (GNEAttributeCarrier::canParse<double>(value)) {
        const auto vehExt = getSUMOTime(value);
        // check that vehExt > 0
        if (vehExt > 0) {
            myTLSEditorParent->myEditedDef->getLogic()->setPhaseVehExt(row, vehExt);
            myTLSEditorParent->myTLSPrograms->markAsModified();
            return true;
        } else {
            return false;
        }
    } else if (StringUtils::prune(value).empty()) {
        myTLSEditorParent->myEditedDef->getLogic()->setPhaseVehExt(row, NBTrafficLightDefinition::UNSPECIFIED_DURATION);
        myTLSEditorParent->myTLSPrograms->markAsModified();
        return true;
    } else {
        return false;
    }
}


bool
GNETLSEditorFrame::TLSPhases::setYellow(const int row, const std::string& value) {
    // check value
    if (value.empty()) {
        // set empty value
        myTLSEditorParent->myEditedDef->getLogic()->setPhaseYellow(row, NBTrafficLightDefinition::UNSPECIFIED_DURATION);
        myTLSEditorParent->myTLSPrograms->markAsModified();
        return true;
    } else if (GNEAttributeCarrier::canParse<double>(value)) {
        const auto yellow = getSUMOTime(value);
        // check that yellow > 0
        if (yellow > 0) {
            myTLSEditorParent->myEditedDef->getLogic()->setPhaseYellow(row, yellow);
            myTLSEditorParent->myTLSPrograms->markAsModified();
            return true;
        } else {
            return false;
        }
    } else if (StringUtils::prune(value).empty()) {
        myTLSEditorParent->myEditedDef->getLogic()->setPhaseYellow(row, NBTrafficLightDefinition::UNSPECIFIED_DURATION);
        myTLSEditorParent->myTLSPrograms->markAsModified();
        return true;
    } else {
        return false;
    }
}


bool
GNETLSEditorFrame::TLSPhases::setRed(const int row, const std::string& value) {
    // check value
    if (value.empty()) {
        // set empty value
        myTLSEditorParent->myEditedDef->getLogic()->setPhaseRed(row, NBTrafficLightDefinition::UNSPECIFIED_DURATION);
        myTLSEditorParent->myTLSPrograms->markAsModified();
        return true;
    } else if (GNEAttributeCarrier::canParse<double>(value)) {
        const auto red = getSUMOTime(value);
        // check that red > 0
        if (red > 0) {
            myTLSEditorParent->myEditedDef->getLogic()->setPhaseRed(row, red);
            myTLSEditorParent->myTLSPrograms->markAsModified();
            return true;
        } else {
            return false;
        }
    } else if (StringUtils::prune(value).empty()) {
        myTLSEditorParent->myEditedDef->getLogic()->setPhaseRed(row, NBTrafficLightDefinition::UNSPECIFIED_DURATION);
        myTLSEditorParent->myTLSPrograms->markAsModified();
        return true;
    } else {
        return false;
    }
}


void
GNETLSEditorFrame::TLSPhases::updateCycleDuration(const int col) {
    SUMOTime cycleDuration = 0;
    for (const auto& phase : myTLSEditorParent->myEditedDef->getLogic()->getPhases()) {
        cycleDuration += phase.duration;
    }
    // update bot label with cycle duration
    myPhaseTable->setColumnLabelBot(col, getSteps2Time(cycleDuration));
}


void
GNETLSEditorFrame::TLSPhases::updateStateSize(const int col) {
    // update bot label with number of links
    myPhaseTable->setColumnLabelBot(col, "Links: " + toString(myTLSEditorParent->myEditedDef->getLogic()->getNumLinks()));
}

// ---------------------------------------------------------------------------
// GNETLSEditorFrame::TLSFile - methods
// ---------------------------------------------------------------------------

GNETLSEditorFrame::TLSFile::TLSFile(GNETLSEditorFrame* TLSEditorParent) :
    MFXGroupBoxModule(TLSEditorParent, TL("TLS Program File")),
    myTLSEditorParent(TLSEditorParent) {
    FXHorizontalFrame* buttonsFrame = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    // create create tlDef button
    myLoadButton = GUIDesigns::buildFXButton(buttonsFrame, TL("Load"), "", TL("Load TLS program from additional file"), GUIIconSubSys::getIcon(GUIIcon::OPEN), this, MID_GNE_TLSFRAME_FILE_LOADPROGRAM, GUIDesignButton);
    myLoadButton->disable();
    // create create tlDef button
    mySaveButton = GUIDesigns::buildFXButton(buttonsFrame, TL("Save"), "", TL("Save TLS program to additional file"), GUIIconSubSys::getIcon(GUIIcon::SAVE), this, MID_GNE_TLSFRAME_FILE_SAVEPROGRAM, GUIDesignButton);
    mySaveButton->disable();
    // show TLSFile
    show();
}


GNETLSEditorFrame::TLSFile::~TLSFile() {}


void
GNETLSEditorFrame::TLSFile::updateTLSFile() {
    if (myTLSEditorParent->myTLSPrograms->getNumberOfPrograms() == 0) {
        myLoadButton->disable();
        mySaveButton->disable();
    } else if (myTLSEditorParent->myTLSAttributes->isSetDetectorsToggleButtonEnabled()) {
        // selecting E1, disable buttons
        myLoadButton->disable();
        mySaveButton->disable();
    } else if (myTLSEditorParent->myTLSJunction->isJoiningJunctions()) {
        // joining TLSs, disable button
        myLoadButton->disable();
        mySaveButton->disable();
    } else {
        myLoadButton->enable();
        mySaveButton->enable();
    }
}


void
GNETLSEditorFrame::TLSFile::showTLSFile() {
    show();
}


void
GNETLSEditorFrame::TLSFile::hideTLSFile() {
    hide();
}


long
GNETLSEditorFrame::TLSFile::onCmdLoadTLSProgram(FXObject*, FXSelector, void*) {
    FXFileDialog opendialog(getCollapsableFrame(), "Load TLS Program");
    opendialog.setIcon(GUIIconSubSys::getIcon(GUIIcon::MODETLS));
    opendialog.setSelectMode(SELECTFILE_EXISTING);
    opendialog.setPatternList(SUMOXMLDefinitions::XMLFileExtensions.getMultilineString().c_str());
    if (gCurrentFolder.length() != 0) {
        opendialog.setDirectory(gCurrentFolder);
    }
    if (opendialog.execute()) {
        // run parser
        NBTrafficLightLogicCont tmpTLLCont;
        NIXMLTrafficLightsHandler tllHandler(tmpTLLCont, myTLSEditorParent->getViewNet()->getNet()->getEdgeCont(), true);
        tmpTLLCont.insert(myTLSEditorParent->myEditedDef);
        XMLSubSys::runParser(tllHandler, opendialog.getFilename().text());

        NBLoadedSUMOTLDef* newDefSameProgram = nullptr;
        std::set<NBLoadedSUMOTLDef*> newDefsOtherProgram;
        for (auto item : tmpTLLCont.getPrograms(myTLSEditorParent->myEditedDef->getID())) {
            if (item.second != myTLSEditorParent->myEditedDef) {
                NBLoadedSUMOTLDef* sdef = dynamic_cast<NBLoadedSUMOTLDef*>(item.second);
                if (item.first == myTLSEditorParent->myEditedDef->getProgramID()) {
                    newDefSameProgram = sdef;
                } else {
                    newDefsOtherProgram.insert(sdef);
                }
            }
        }
        const int newPrograms = (int)newDefsOtherProgram.size();
        if (newPrograms > 0 || newDefSameProgram != nullptr) {
            std::vector<NBNode*> nodes = myTLSEditorParent->myEditedDef->getNodes();
            for (auto newProg : newDefsOtherProgram) {
                for (auto it_node : nodes) {
                    GNEJunction* junction = myTLSEditorParent->getViewNet()->getNet()->getAttributeCarriers()->retrieveJunction(it_node->getID());
                    myTLSEditorParent->getViewNet()->getUndoList()->add(new GNEChange_TLS(junction, newProg, true), true);
                }
            }
            if (newPrograms > 0) {
                WRITE_MESSAGE(TL("Loaded ") + toString(newPrograms) + TL(" new programs for tlLogic '") + myTLSEditorParent->myEditedDef->getID() + "'");
            }
            if (newDefSameProgram != nullptr) {
                // replace old program when loading the same program ID
                myTLSEditorParent->myEditedDef = newDefSameProgram;
                WRITE_MESSAGE(TL("Updated program '") + newDefSameProgram->getProgramID() + TL("' for tlLogic '") + myTLSEditorParent->myEditedDef->getID() + "'");
            }
        } else {
            if (tllHandler.getSeenIDs().count(myTLSEditorParent->myEditedDef->getID()) == 0)  {
                myTLSEditorParent->getViewNet()->setStatusBarText(TL("No programs found for traffic light '") + myTLSEditorParent->myEditedDef->getID() + "'");
            }
        }

        // clean up temporary container to avoid deletion of defs when its destruct is called
        for (NBTrafficLightDefinition* def : tmpTLLCont.getDefinitions()) {
            tmpTLLCont.removeProgram(def->getID(), def->getProgramID(), false);
        }

        myTLSEditorParent->myTLSPhases->initPhaseTable();
        myTLSEditorParent->myTLSPrograms->markAsModified();
    }
    myTLSEditorParent->updateModules();
    return 0;
}


long
GNETLSEditorFrame::TLSFile::onCmdSaveTLSProgram(FXObject*, FXSelector, void*) {
    FXString file = MFXUtils::getFilename2Write(this, TL("Save TLS Program as"),
                    SUMOXMLDefinitions::XMLFileExtensions.getMultilineString().c_str(),
                    GUIIconSubSys::getIcon(GUIIcon::MODETLS), gCurrentFolder);
    // check file
    if (file != "") {
        // add xml extension
        file = FileHelpers::addExtension(file.text(), ".xml").c_str();
        OutputDevice& device = OutputDevice::getDevice(file.text());
        // save program
        device.writeXMLHeader("additional", "additional_file.xsd");
        NWWriter_SUMO::writeTrafficLight(device, myTLSEditorParent->myEditedDef->getLogic());
        device.close();
    }
    myTLSEditorParent->updateModules();
    return 1;
}


std::string
GNETLSEditorFrame::TLSFile::writeSUMOTime(SUMOTime steps) {
    const double time = STEPS2TIME(steps);
    if (time == std::floor(time)) {
        return toString(int(time));
    } else {
        return toString(time);
    }
}

/****************************************************************************/
