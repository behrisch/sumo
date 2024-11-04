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
/// @file    GNESelectorFrame.cpp
/// @author  Jakob Erdmann
/// @date    Mar 2011
///
// The Widget for modifying selections of network-elements
/****************************************************************************/
#include <config.h>

#include <netedit/GNENet.h>
#include <netedit/GNEUndoList.h>
#include <netedit/GNEViewNet.h>
#include <netedit/elements/network/GNEConnection.h>
#include <netedit/elements/network/GNECrossing.h>
#include <netedit/elements/network/GNEWalkingArea.h>
#include <utils/foxtools/MFXDynamicLabel.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/gui/globjects/GUIGlObjectStorage.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/xml/NamespaceIDs.h>

#include "GNESelectorFrame.h"
#include "GNEElementSet.h"


// ===========================================================================
// FOX callback mapping
// ===========================================================================
FXDEFMAP(GNESelectorFrame::ModificationMode) ModificationModeMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_CHOOSEN_OPERATION,  GNESelectorFrame::ModificationMode::onCmdSelectModificationMode)
};

FXDEFMAP(GNESelectorFrame::VisualScaling) VisualScalingMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SELECTORFRAME_SELECTSCALE,  GNESelectorFrame::VisualScaling::onCmdScaleSelection)
};

FXDEFMAP(GNESelectorFrame::SelectionOperation) SelectionOperationMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_CHOOSEN_LOAD,   GNESelectorFrame::SelectionOperation::onCmdLoad),
    FXMAPFUNC(SEL_COMMAND,  MID_CHOOSEN_SAVE,   GNESelectorFrame::SelectionOperation::onCmdSave),
    FXMAPFUNC(SEL_COMMAND,  MID_CHOOSEN_INVERT, GNESelectorFrame::SelectionOperation::onCmdInvert),
    FXMAPFUNC(SEL_COMMAND,  MID_CHOOSEN_CLEAR,  GNESelectorFrame::SelectionOperation::onCmdClear),
    FXMAPFUNC(SEL_COMMAND,  MID_CHOOSEN_DELETE, GNESelectorFrame::SelectionOperation::onCmdDelete),
    FXMAPFUNC(SEL_COMMAND,  MID_CHOOSEN_REDUCE, GNESelectorFrame::SelectionOperation::onCmdReduce)
};

FXDEFMAP(GNESelectorFrame::SelectionHierarchy) SelectionHierarchyMap[] = {
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SELECT,                 GNESelectorFrame::SelectionHierarchy::onCmdSelectItem),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SELECTORFRAME_PARENTS,  GNESelectorFrame::SelectionHierarchy::onCmdParents),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_SELECTORFRAME_CHILDREN, GNESelectorFrame::SelectionHierarchy::onCmdChildren),
};

// Object implementation
FXIMPLEMENT(GNESelectorFrame::ModificationMode,     MFXGroupBoxModule,   ModificationModeMap,    ARRAYNUMBER(ModificationModeMap))
FXIMPLEMENT(GNESelectorFrame::VisualScaling,        MFXGroupBoxModule,   VisualScalingMap,       ARRAYNUMBER(VisualScalingMap))
FXIMPLEMENT(GNESelectorFrame::SelectionOperation,   MFXGroupBoxModule,   SelectionOperationMap,  ARRAYNUMBER(SelectionOperationMap))
FXIMPLEMENT(GNESelectorFrame::SelectionHierarchy,   MFXGroupBoxModule,   SelectionHierarchyMap,  ARRAYNUMBER(SelectionHierarchyMap))

// ===========================================================================
// method definitions
// ===========================================================================

// ---------------------------------------------------------------------------
// ModificationMode::SelectionInformation - methods
// ---------------------------------------------------------------------------

GNESelectorFrame::SelectionInformation::SelectionInformation(GNESelectorFrame* selectorFrameParent) :
    MFXGroupBoxModule(selectorFrameParent, TL("Selection information")),
    mySelectorFrameParent(selectorFrameParent) {
    // information label
    myInformationLabel = new FXLabel(getCollapsableFrame(), "", nullptr, GUIDesignLabelFrameInformation);
}


GNESelectorFrame::SelectionInformation::~SelectionInformation() {}


void
GNESelectorFrame::SelectionInformation::updateInformationLabel() {
    // first clear information
    myInformation.clear();
    // get attribute carriers
    const auto ACs = mySelectorFrameParent->getViewNet()->getNet()->getAttributeCarriers();
    // continue depending of supermode
    if (mySelectorFrameParent->getViewNet()->getEditModes().isCurrentSupermodeNetwork()) {
        updateInformationLabel(TL("Junctions"), ACs->getNumberOfSelectedJunctions());
        updateInformationLabel(TL("Edges"), ACs->getNumberOfSelectedEdges());
        updateInformationLabel(TL("Lanes"), ACs->getNumberOfSelectedLanes());
        updateInformationLabel(TL("Connections"), ACs->getNumberOfSelectedConnections());
        updateInformationLabel(TL("Crossings"), ACs->getNumberOfSelectedCrossings());
        updateInformationLabel(TL("WalkingAreas"), ACs->getNumberOfSelectedWalkingAreas());
        updateInformationLabel(TL("Additionals"), ACs->getNumberOfSelectedPureAdditionals());
        updateInformationLabel(TL("Wires"), ACs->getNumberOfSelectedWires());
        updateInformationLabel(TL("TAZs"), ACs->getNumberOfSelectedTAZs());
        updateInformationLabel(TL("TAZSources"), ACs->getNumberOfSelectedTAZSources());
        updateInformationLabel(TL("TAZSinks"), ACs->getNumberOfSelectedTAZSinks());
        updateInformationLabel(TL("Polygons"), ACs->getNumberOfSelectedPolygons());
        updateInformationLabel(TL("POIs"), ACs->getNumberOfSelectedPOIs());
        updateInformationLabel(TL("JuPedSim elements"),
                               ACs->getNumberOfSelectedJpsWalkableAreas() +
                               ACs->getNumberOfSelectedJpsObstacles());
    } else if (mySelectorFrameParent->getViewNet()->getEditModes().isCurrentSupermodeDemand()) {
        updateInformationLabel(TL("Routes"), ACs->getNumberOfSelectedRoutes());
        updateInformationLabel(TL("Vehicles"), ACs->getNumberOfSelectedVehicles());
        updateInformationLabel(TL("Persons"), ACs->getNumberOfSelectedPersons());
        updateInformationLabel(TL("Person trips"), ACs->getNumberOfSelectedPersonTrips());
        updateInformationLabel(TL("Walks"), ACs->getNumberOfSelectedWalks());
        updateInformationLabel(TL("Rides"), ACs->getNumberOfSelectedRides());
        updateInformationLabel(TL("Containers"), ACs->getNumberOfSelectedContainers());
        updateInformationLabel(TL("Transport"), ACs->getNumberOfSelectedTransport());
        updateInformationLabel(TL("Tranships"), ACs->getNumberOfSelectedTranships());
        updateInformationLabel(TL("Stops"), ACs->getNumberOfSelectedStops());
    } else if (mySelectorFrameParent->getViewNet()->getEditModes().isCurrentSupermodeData()) {
        updateInformationLabel(TL("EdgeDatas"), ACs->getNumberOfSelectedEdgeDatas());
        updateInformationLabel(TL("EdgeRelDatas"), ACs->getNumberOfSelectedEdgeRelDatas());
        updateInformationLabel(TL("EdgeTAZRel"), ACs->getNumberOfSelectedEdgeTAZRel());
    }
    // adjust format
    const auto numberLines = std::count(myInformation.begin(), myInformation.end(), ':');
    if (numberLines == 0) {
        myInformation.append(" \n \n");
    } else if (numberLines > 1) {
        myInformation.pop_back();
    }
    // set label
    myInformationLabel->setText(myInformation.c_str());
}


void
GNESelectorFrame::SelectionInformation::updateInformationLabel(const std::string& element, int number) {
    // check number
    if (number > 0) {
        myInformation.append(element + ": " + toString(number) + "\n");
    }
}

// ---------------------------------------------------------------------------
// ModificationMode::ModificationMode - methods
// ---------------------------------------------------------------------------

GNESelectorFrame::ModificationMode::ModificationMode(GNESelectorFrame* selectorFrameParent) :
    MFXGroupBoxModule(selectorFrameParent, TL("Modification Mode")),
    myModificationModeType(Operation::ADD) {
    // Create all options buttons
    myAddRadioButton = GUIDesigns::buildFXRadioButton(getCollapsableFrame(), TL("add"), "", TL("Selected objects are added to the previous selection"),
                       this, MID_CHOOSEN_OPERATION, GUIDesignRadioButton);
    myRemoveRadioButton = GUIDesigns::buildFXRadioButton(getCollapsableFrame(), TL("remove"), "", TL("Selected objects are removed from the previous selection"),
                          this, MID_CHOOSEN_OPERATION, GUIDesignRadioButton);
    myKeepRadioButton = GUIDesigns::buildFXRadioButton(getCollapsableFrame(), TL("keep"), "", TL("Restrict previous selection by the current selection"),
                        this, MID_CHOOSEN_OPERATION, GUIDesignRadioButton);
    myReplaceRadioButton = GUIDesigns::buildFXRadioButton(getCollapsableFrame(), TL("replace"), "", TL("Replace previous selection by the current selection"),
                           this, MID_CHOOSEN_OPERATION, GUIDesignRadioButton);
    myAddRadioButton->setCheck(true);
}


GNESelectorFrame::ModificationMode::~ModificationMode() {}


GNESelectorFrame::ModificationMode::Operation
GNESelectorFrame::ModificationMode::getModificationMode() const {
    return myModificationModeType;
}


long
GNESelectorFrame::ModificationMode::onCmdSelectModificationMode(FXObject* obj, FXSelector, void*) {
    if (obj == myAddRadioButton) {
        myModificationModeType = Operation::ADD;
        myAddRadioButton->setCheck(true);
        myRemoveRadioButton->setCheck(false);
        myKeepRadioButton->setCheck(false);
        myReplaceRadioButton->setCheck(false);
        return 1;
    } else if (obj == myRemoveRadioButton) {
        myModificationModeType = Operation::SUB;
        myAddRadioButton->setCheck(false);
        myRemoveRadioButton->setCheck(true);
        myKeepRadioButton->setCheck(false);
        myReplaceRadioButton->setCheck(false);
        return 1;
    } else if (obj == myKeepRadioButton) {
        myModificationModeType = Operation::RESTRICT;
        myAddRadioButton->setCheck(false);
        myRemoveRadioButton->setCheck(false);
        myKeepRadioButton->setCheck(true);
        myReplaceRadioButton->setCheck(false);
        return 1;
    } else if (obj == myReplaceRadioButton) {
        myModificationModeType = Operation::REPLACE;
        myAddRadioButton->setCheck(false);
        myRemoveRadioButton->setCheck(false);
        myKeepRadioButton->setCheck(false);
        myReplaceRadioButton->setCheck(true);
        return 1;
    } else {
        return 0;
    }
}

// ---------------------------------------------------------------------------
// ModificationMode::VisualScaling - methods
// ---------------------------------------------------------------------------

GNESelectorFrame::VisualScaling::VisualScaling(GNESelectorFrame* selectorFrameParent) :
    MFXGroupBoxModule(selectorFrameParent, TL("Visual Scaling")),
    mySelectorFrameParent(selectorFrameParent) {
    // Create spin button and configure it
    mySelectionScaling = new FXRealSpinner(getCollapsableFrame(), 7, this, MID_GNE_SELECTORFRAME_SELECTSCALE, GUIDesignSpinDial);
    //mySelectionScaling->setNumberFormat(1);
    //mySelectionScaling->setIncrements(0.1, .5, 1);
    mySelectionScaling->setIncrement(0.5);
    mySelectionScaling->setRange(1, 100000);
    mySelectionScaling->setValue(1);
    mySelectionScaling->setHelpText(TL("Enlarge selected objects"));
}


GNESelectorFrame::VisualScaling::~VisualScaling() {}


long
GNESelectorFrame::VisualScaling::onCmdScaleSelection(FXObject*, FXSelector, void*) {
    // set scale in viewnet
    mySelectorFrameParent->myViewNet->setSelectorFrameScale(mySelectionScaling->getValue());
    mySelectorFrameParent->myViewNet->updateViewNet();
    return 1;
}

// ---------------------------------------------------------------------------
// ModificationMode::SelectionHierarchy - methods
// ---------------------------------------------------------------------------

GNESelectorFrame::SelectionOperation::SelectionOperation(GNESelectorFrame* selectorFrameParent) :
    MFXGroupBoxModule(selectorFrameParent, TL("Selection operations")),
    mySelectorFrameParent(selectorFrameParent) {
    // tabular buttons, see GNETLSEditorFrame

    FXHorizontalFrame* selectionButtons = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    FXVerticalFrame* col1 = new FXVerticalFrame(selectionButtons, LAYOUT_FILL_X, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0); // left button columm
    FXVerticalFrame* col2 = new FXVerticalFrame(selectionButtons, LAYOUT_FILL_X, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0); // right button column

    // Create "Clear List" Button
    GUIDesigns::buildFXButton(col1, TL("Clear"), "", TL("Deselect all objects (hotkey: ESC)"), nullptr, this, MID_CHOOSEN_CLEAR, GUIDesignButton);
    // Create "Invert" Button
    GUIDesigns::buildFXButton(col2, TL("Invert"), "", TL("Invert selection status of all objects"), nullptr, this, MID_CHOOSEN_INVERT, GUIDesignButton);
    // Create "Save" Button
    GUIDesigns::buildFXButton(col1, TL("Save"), "", TL("Save ids of currently selected objects to a file."), nullptr, this, MID_CHOOSEN_SAVE, GUIDesignButton);
    // Create "Load" Button
    GUIDesigns::buildFXButton(col2, TL("Load"), "", TL("Load ids from a file according to the current modification mode."), nullptr, this, MID_CHOOSEN_LOAD, GUIDesignButton);
    // Create "Delete" Button
    GUIDesigns::buildFXButton(col1, TL("Delete"), "", TL("Delete all selected objects (hotkey: DEL)"), nullptr, this, MID_CHOOSEN_DELETE, GUIDesignButton);
    // Create "reduce" Button
    GUIDesigns::buildFXButton(col2, TL("Reduce"), "", TL("Reduce network to current selection."), nullptr, this, MID_CHOOSEN_REDUCE, GUIDesignButton);
}


GNESelectorFrame::SelectionOperation::~SelectionOperation() {}


void
GNESelectorFrame::SelectionOperation::loadFromFile(const std::string& file) const {
    std::vector<GNEAttributeCarrier*> loadedACs;
    std::ifstream strm(file.c_str());
    // check if file can be opened
    if (!strm.good()) {
        WRITE_ERRORF(TL("Could not open '%'."), file);
    } else {
        // convert all glObjects into GNEAttributeCarriers
        std::map<const std::string, GNEAttributeCarrier*> GLFUllNameAC;
        const auto GLObjects = GUIGlObjectStorage::gIDStorage.getAllGLObjects();
        for (const auto& GLObject : GLObjects) {
            // try to parse GLObject to AC
            GNEAttributeCarrier* AC = dynamic_cast<GNEAttributeCarrier*>(GLObject);
            // if was successfully parsed and is NOT a template, add into GLFUllNameAC using fullName
            if (AC && !AC->isTemplate()) {
                GLFUllNameAC[GUIGlObject::TypeNames.getString(GLObject->getType()) + ":" + AC->getID()] = AC;
            }
        }
        // continue while stream exist
        while (strm.good()) {
            std::string line;
            strm >> line;
            // check if line isn't empty
            if (line.length() != 0) {
                // obtain AC from GLFUllNameAC
                if (StringUtils::startsWith(line, "node:")) {
                    line = StringUtils::replace(line, "node:", "junction:");
                }
                GNEAttributeCarrier* AC = GLFUllNameAC.count(line) > 0 ? GLFUllNameAC.at(line) : nullptr;
                // check if AC exist, is selectable, and isn't locked
                if (AC && AC->getTagProperty().isSelectable() && !mySelectorFrameParent->getViewNet()->getLockManager().isObjectLocked(AC->getGUIGlObject()->getType(), false)) {
                    // now check if we're in the correct supermode to load this element
                    if (((mySelectorFrameParent->myViewNet->getEditModes().isCurrentSupermodeNetwork()) && !AC->getTagProperty().isDemandElement()) ||
                            ((mySelectorFrameParent->myViewNet->getEditModes().isCurrentSupermodeDemand()) && AC->getTagProperty().isDemandElement()) ||
                            ((mySelectorFrameParent->myViewNet->getEditModes().isCurrentSupermodeData()) && AC->getTagProperty().isDataElement())) {
                        loadedACs.push_back(AC);
                    }
                }
            }
        }
        // change selected attribute in loaded ACs allowing undo/redo
        if (loadedACs.size() > 0) {
            mySelectorFrameParent->myViewNet->getUndoList()->begin(GUIIcon::MODESELECT, TL("load selection"));
            mySelectorFrameParent->handleIDs(loadedACs);
            mySelectorFrameParent->myViewNet->getUndoList()->end();
        }
    }
}


long
GNESelectorFrame::SelectionOperation::onCmdLoad(FXObject*, FXSelector, void*) {
    // get the new file name
    FXFileDialog opendialog(getCollapsableFrame(), TL("Open List of Selected Items"));
    opendialog.setIcon(GUIIconSubSys::getIcon(GUIIcon::OPEN));
    opendialog.setSelectMode(SELECTFILE_EXISTING);
    opendialog.setPatternList("Selection files (*.txt)\nAll files (*)");
    if (gCurrentFolder.length() != 0) {
        opendialog.setDirectory(gCurrentFolder);
    }
    if (opendialog.execute()) {
        gCurrentFolder = opendialog.getDirectory();
        loadFromFile(opendialog.getFilename().text());
    }
    return 1;
}


long
GNESelectorFrame::SelectionOperation::onCmdSave(FXObject*, FXSelector, void*) {
    FXString file = MFXUtils::getFilename2Write(this,
                    TL("Save List of selected Items"), ".txt",
                    GUIIconSubSys::getIcon(GUIIcon::SAVE), gCurrentFolder);
    if (file == "") {
        return 1;
    }
    try {
        OutputDevice& dev = OutputDevice::getDevice(file.text());
        // get selected attribute carriers
        const auto selectedACs = mySelectorFrameParent->myViewNet->getNet()->getAttributeCarriers()->getSelectedAttributeCarriers(false);
        for (const auto& selectedAC : selectedACs) {
            GUIGlObject* object = dynamic_cast<GUIGlObject*>(selectedAC);
            if (object) {
                dev << GUIGlObject::TypeNames.getString(object->getType()) << ":" << selectedAC->getID() << "\n";
            }
        }
        dev.close();
    } catch (IOError& e) {
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Opening FXMessageBox 'error storing selection'");
        // open message box error
        FXMessageBox::error(getCollapsableFrame(), MBOX_OK, "Storing Selection failed", "%s", e.what());
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox 'error storing selection' with 'OK'");
    }
    return 1;
}


long
GNESelectorFrame::SelectionOperation::onCmdClear(FXObject*, FXSelector, void*) {
    // obtain undoList (only for improve code legibly)
    GNEUndoList* undoList = mySelectorFrameParent->myViewNet->getUndoList();
    // get element to select/unselect depending of current supermode
    std::pair<std::vector<std::pair<bool, GNEAttributeCarrier*> >, std::vector<std::pair<bool, GNEAttributeCarrier*> > > ACsToSelectUnselect;
    if (mySelectorFrameParent->myViewNet->getEditModes().isCurrentSupermodeNetwork()) {
        ACsToSelectUnselect = processMassiveNetworkElementSelection(false);
    } else if (mySelectorFrameParent->myViewNet->getEditModes().isCurrentSupermodeDemand()) {
        ACsToSelectUnselect = processMassiveDemandElementSelection();
    } else if (mySelectorFrameParent->myViewNet->getEditModes().isCurrentSupermodeData()) {
        ACsToSelectUnselect = processMassiveDataElementSelection();
    }
    // only continue if there are elements to unselect
    if (ACsToSelectUnselect.second.size() > 0) {
        // check if add locked elements
        bool askedContinueIfLock = false;
        bool addLockedElements = false;
        bool unlockedElements = false;
        for (const auto& AC : ACsToSelectUnselect.second) {
            if (AC.first == false) {
                // there are unlocked elements
                unlockedElements = true;
            } else if (!askedContinueIfLock) {
                addLockedElements = askContinueIfLock();
                // only ask one time for locking
                askedContinueIfLock = true;
            }
        }
        if (unlockedElements || addLockedElements) {
            mySelectorFrameParent->myViewNet->getUndoList()->begin(GUIIcon::MODESELECT, TL("clear selection"));
            for (const auto& AC : ACsToSelectUnselect.second) {
                if (addLockedElements || !AC.first) {
                    AC.second->setAttribute(GNE_ATTR_SELECTED, "false", undoList);
                }
            }
            mySelectorFrameParent->myViewNet->getUndoList()->end();
        }
    }
    return 1;
}

long
GNESelectorFrame::SelectionOperation::onCmdDelete(FXObject*, FXSelector, void*) {
    // acts like the 'del' hotkey
    mySelectorFrameParent->getViewNet()->hotkeyDel();
    return 1;
}


long
GNESelectorFrame::SelectionOperation::onCmdInvert(FXObject*, FXSelector, void*) {
    // obtain undoList (only for improve code legibly)
    GNEUndoList* undoList = mySelectorFrameParent->myViewNet->getUndoList();
    // get element to select/unselect depending of current supermode
    std::pair<std::vector<std::pair<bool, GNEAttributeCarrier*> >, std::vector<std::pair<bool, GNEAttributeCarrier*> > > ACsToSelectUnselect;
    if (mySelectorFrameParent->myViewNet->getEditModes().isCurrentSupermodeNetwork()) {
        ACsToSelectUnselect = processMassiveNetworkElementSelection(true);
    } else if (mySelectorFrameParent->myViewNet->getEditModes().isCurrentSupermodeDemand()) {
        ACsToSelectUnselect = processMassiveDemandElementSelection();
    } else if (mySelectorFrameParent->myViewNet->getEditModes().isCurrentSupermodeData()) {
        ACsToSelectUnselect = processMassiveDataElementSelection();
    }
    // only continue if there are elements to select and unselect
    if ((ACsToSelectUnselect.first.size() + ACsToSelectUnselect.second.size()) > 0) {
        // check if add locked elements
        bool askedContinueIfLock = false;
        bool addLockedElements = false;
        bool unlockedElements = false;
        for (const auto& AC : ACsToSelectUnselect.first) {
            if (AC.first == false) {
                // there are unlocked elements
                unlockedElements = true;
            } else if (!askedContinueIfLock) {
                addLockedElements = askContinueIfLock();
                // only ask one time for locking
                askedContinueIfLock = true;
            }
        }
        for (const auto& AC : ACsToSelectUnselect.second) {
            if (AC.first == false) {
                // there are unlocked elements
                unlockedElements = true;
            } else if (!askedContinueIfLock) {
                addLockedElements = askContinueIfLock();
                // only ask one time for locking
                askedContinueIfLock = true;
            }
        }
        if (unlockedElements || addLockedElements) {
            mySelectorFrameParent->myViewNet->getUndoList()->begin(GUIIcon::MODESELECT, TL("invert selection"));
            for (const auto& AC : ACsToSelectUnselect.first) {
                if (addLockedElements || !AC.first) {
                    AC.second->setAttribute(GNE_ATTR_SELECTED, "true", undoList);
                }
            }
            for (const auto& AC : ACsToSelectUnselect.second) {
                if (addLockedElements || !AC.first) {
                    AC.second->setAttribute(GNE_ATTR_SELECTED, "false", undoList);
                }
            }
            mySelectorFrameParent->myViewNet->getUndoList()->end();
        }
    }
    return 1;
}


long
GNESelectorFrame::SelectionOperation::onCmdReduce(FXObject*, FXSelector, void*) {
    // begin undoList operation
    mySelectorFrameParent->getViewNet()->getUndoList()->begin(Supermode::NETWORK, GUIIcon::SIMPLIFYNETWORK, TL("reduce network"));
    // invert and clear
    onCmdInvert(0, 0, 0);
    onCmdDelete(0, 0, 0);
    // end undoList operation
    mySelectorFrameParent->getViewNet()->getUndoList()->end();
    return 1;
}


std::pair<std::vector<std::pair<bool, GNEAttributeCarrier*> >, std::vector<std::pair<bool, GNEAttributeCarrier*> > >
GNESelectorFrame::SelectionOperation::processMassiveNetworkElementSelection(const bool filterLanes) {
    // get attribute carriers (only for improve code legibly)
    const auto& ACs = mySelectorFrameParent->myViewNet->getNet()->getAttributeCarriers();
    // extract all network elements
    std::vector<GNEAttributeCarrier*> networkACs;
    // add junctions
    for (const auto& junction : ACs->getJunctions()) {
        networkACs.push_back(junction.second);
        // due we iterate over all junctions, only it's necessary iterate over incoming edges
        for (const auto& incomingEdge : junction.second->getGNEIncomingEdges()) {
            if (!filterLanes || mySelectorFrameParent->getViewNet()->getNetworkViewOptions().selectEdges()) {
                networkACs.push_back(incomingEdge);
            }
            // add lanes
            if (!filterLanes || !mySelectorFrameParent->getViewNet()->getNetworkViewOptions().selectEdges()) {
                for (const auto& lane : incomingEdge->getLanes()) {
                    networkACs.push_back(lane);
                }
            }
            // add connections
            for (const auto& connection : incomingEdge->getGNEConnections()) {
                networkACs.push_back(connection);
            }
        }
        // add crossings
        for (const auto& crossing : junction.second->getGNECrossings()) {
            networkACs.push_back(crossing);
        }
        // add walkingArea
        for (const auto& walkingArea : junction.second->getGNEWalkingAreas()) {
            networkACs.push_back(walkingArea);
        }
    }
    // add additionals
    for (const auto& additionalTags : ACs->getAdditionals()) {
        for (const auto& additional : additionalTags.second) {
            if (additional.second->getTagProperty().isSelectable()) {
                networkACs.push_back(additional.second);
            }
        }
    }
    // declare set of checked GLTypes to avoid unnecessary calls to isGLObjectLocked()
    std::map<GUIGlObjectType, bool> checkedTypes;
    // declare vector in which save ACs to select and unselect
    std::pair<std::vector<std::pair<bool, GNEAttributeCarrier*> >, std::vector<std::pair<bool, GNEAttributeCarrier*> > > ACsToSelectUnselect;
    // iterate over network ACs
    for (const auto& networkAC : networkACs) {
        const auto networkACObjectType = networkAC->getGUIGlObject()->getType();
        // save locking status in checkedTypes
        if (checkedTypes.find(networkACObjectType) == checkedTypes.end()) {
            checkedTypes[networkACObjectType] = networkAC->getGUIGlObject()->isGLObjectLocked();
        }
        // save element and their locking status
        if (networkAC->isAttributeCarrierSelected()) {
            ACsToSelectUnselect.second.push_back(std::make_pair(checkedTypes.at(networkACObjectType), networkAC));
        } else {
            ACsToSelectUnselect.first.push_back(std::make_pair(checkedTypes.at(networkACObjectType), networkAC));
        }
    }
    return ACsToSelectUnselect;
}


std::pair<std::vector<std::pair<bool, GNEAttributeCarrier*> >, std::vector<std::pair<bool, GNEAttributeCarrier*> > >
GNESelectorFrame::SelectionOperation::processMassiveDemandElementSelection() {
    // declare set of checked GLTypes to avoid unnecessary calls to isGLObjectLocked()
    std::map<GUIGlObjectType, bool> checkedTypes;
    // declare vector in which save ACs to select and unselect
    std::pair<std::vector<std::pair<bool, GNEAttributeCarrier*> >, std::vector<std::pair<bool, GNEAttributeCarrier*> > > ACsToSelectUnselect;
    // iterate over selectable demand elements
    for (const auto& demandElementTag : mySelectorFrameParent->myViewNet->getNet()->getAttributeCarriers()->getDemandElements()) {
        for (const auto& demandElement : demandElementTag.second) {
            if (demandElement.second->getTagProperty().isSelectable()) {
                const auto networkACObjectType = demandElement.first->getType();
                // save locking status in checkedTypes
                if (checkedTypes.find(networkACObjectType) == checkedTypes.end()) {
                    checkedTypes[networkACObjectType] = demandElement.first->isGLObjectLocked();
                }
                // save element and their locking status
                if (demandElement.second->isAttributeCarrierSelected()) {
                    ACsToSelectUnselect.second.push_back(std::make_pair(checkedTypes.at(networkACObjectType), demandElement.second));
                } else {
                    ACsToSelectUnselect.first.push_back(std::make_pair(checkedTypes.at(networkACObjectType), demandElement.second));
                }
            }
        }
    }
    return ACsToSelectUnselect;
}


std::pair<std::vector<std::pair<bool, GNEAttributeCarrier*> >, std::vector<std::pair<bool, GNEAttributeCarrier*> > >
GNESelectorFrame::SelectionOperation::processMassiveDataElementSelection() {
    // declare set of checked GLTypes to avoid unnecessary calls to isGLObjectLocked()
    std::map<GUIGlObjectType, bool> checkedTypes;
    // declare vector in which save ACs to select and unselect
    std::pair<std::vector<std::pair<bool, GNEAttributeCarrier*> >, std::vector<std::pair<bool, GNEAttributeCarrier*> > > ACsToSelectUnselect;
    // iterate over selectable demand elements
    for (const auto& genericDataTag : mySelectorFrameParent->myViewNet->getNet()->getAttributeCarriers()->getGenericDatas()) {
        for (const auto& genericData : genericDataTag.second) {
            if (genericData.second->getTagProperty().isSelectable()) {
                const auto networkACObjectType = genericData.first->getType();
                // save locking status in checkedTypes
                if (checkedTypes.find(networkACObjectType) == checkedTypes.end()) {
                    checkedTypes[networkACObjectType] = genericData.first->isGLObjectLocked();
                }
                // save element and their locking status
                if (genericData.second->isAttributeCarrierSelected()) {
                    ACsToSelectUnselect.second.push_back(std::make_pair(checkedTypes.at(networkACObjectType), genericData.second));
                } else {
                    ACsToSelectUnselect.first.push_back(std::make_pair(checkedTypes.at(networkACObjectType), genericData.second));
                }
            }
        }
    }
    return ACsToSelectUnselect;
}


bool
GNESelectorFrame::SelectionOperation::askContinueIfLock() const {
    WRITE_DEBUG("Opening FXMessageBox 'confirm selection operation'");
    // open question box
    const FXuint answer = FXMessageBox::question(mySelectorFrameParent->getViewNet()->getApp(),
                          MBOX_YES_NO, "Confirm selection operation", "There are locked elements in the current selection.\nApply operation to locked elements?");
    if (answer != 1) { //1:yes, 2:no, 4:esc
        // write warning if netedit is running in testing mode
        if (answer == 2) {
            WRITE_DEBUG("Closed FXMessageBox 'confirm selection operation' with 'No'");
        } else if (answer == 4) {
            WRITE_DEBUG("Closed FXMessageBox 'confirm selection operation' with 'ESC'");
        }
        return false;
    } else {
        // write warning if netedit is running in testing mode
        WRITE_DEBUG("Closed FXMessageBox 'confirm selection operation' with 'Yes'");
        return true;
    }
}

// ---------------------------------------------------------------------------
// ModificationMode::SelectionHierarchy - methods
// ---------------------------------------------------------------------------

GNESelectorFrame::SelectionHierarchy::SelectionHierarchy(GNESelectorFrame* selectorFrameParent) :
    MFXGroupBoxModule(selectorFrameParent, TL("Hierarchy operations")),
    mySelectorFrameParent(selectorFrameParent),
    myCurrentSelectedParent(Selection::ALL),
    myCurrentSelectedChild(Selection::ALL) {
    // create label for parents
    new FXLabel(getCollapsableFrame(), TL("Select parents"), nullptr, GUIDesignLabelThick(JUSTIFY_NORMAL));
    // Create MFXComboBoxIcon for parent comboBox
    myParentsComboBox = new MFXComboBoxIcon(getCollapsableFrame(), GUIDesignComboBoxNCol, true, GUIDesignComboBoxVisibleItemsMedium,
                                            this, MID_GNE_SELECT, GUIDesignComboBox);
    // create parent buttons
    FXHorizontalFrame* parentButtons = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    // Create "select" Button
    mySelectParentsButton = GUIDesigns::buildFXButton(parentButtons, TL("Select"), "", "", GUIIconSubSys::getIcon(GUIIcon::SELECT), this, MID_GNE_SELECTORFRAME_PARENTS, GUIDesignButton);
    // Create "unselect" Button
    myUnselectParentsButton = GUIDesigns::buildFXButton(parentButtons, TL("Unselect"), "", "", GUIIconSubSys::getIcon(GUIIcon::UNSELECT), this, MID_GNE_SELECTORFRAME_PARENTS, GUIDesignButton);
    // create label for parents
    new FXLabel(getCollapsableFrame(), TL("Select children"), nullptr, GUIDesignLabelThick(JUSTIFY_NORMAL));
    // Create MFXComboBoxIcon for parent comboBox
    myChildrenComboBox = new MFXComboBoxIcon(getCollapsableFrame(), GUIDesignComboBoxNCol, true, GUIDesignComboBoxVisibleItemsMedium,
            this, MID_GNE_SELECT, GUIDesignComboBox);
    // create children buttons
    FXHorizontalFrame* childrenButtons = new FXHorizontalFrame(getCollapsableFrame(), GUIDesignAuxiliarHorizontalFrame);
    // Create "select" Button
    mySelectChildrenButton = GUIDesigns::buildFXButton(childrenButtons, TL("Select"), "", "", GUIIconSubSys::getIcon(GUIIcon::SELECT), this, MID_GNE_SELECTORFRAME_CHILDREN, GUIDesignButton);
    // Create "unselect" Button
    myUnselectChildrenButton = GUIDesigns::buildFXButton(childrenButtons, TL("Unselect"), "", "", GUIIconSubSys::getIcon(GUIIcon::UNSELECT), this, MID_GNE_SELECTORFRAME_CHILDREN, GUIDesignButton);
    // fill comboBoxes
    for (const auto& item : myItems) {
        myParentsComboBox->appendIconItem(item.second.c_str());
        myChildrenComboBox->appendIconItem(item.second.c_str());
    }
}


GNESelectorFrame::SelectionHierarchy::~SelectionHierarchy() {}


long
GNESelectorFrame::SelectionHierarchy::onCmdSelectItem(FXObject* obj, FXSelector, void*) {
    if (obj == myParentsComboBox) {
        for (const auto& item : myItems) {
            if (item.second == myParentsComboBox->getText().text()) {
                // enable buttons
                mySelectParentsButton->enable();
                myUnselectParentsButton->enable();
                // change text color
                myParentsComboBox->setTextColor(FXRGB(0, 0, 0));
                // set current selected parent
                myCurrentSelectedParent = item.first;
                return 1;
            }
        }
        // item not found
        myCurrentSelectedParent = Selection::NOTHING;
        // disable buttons
        mySelectParentsButton->disable();
        myUnselectParentsButton->disable();
        myParentsComboBox->setTextColor(FXRGB(255, 0, 0));
        return 1;
    } else if (obj == myChildrenComboBox) {
        for (const auto& item : myItems) {
            if (item.second == myChildrenComboBox->getText().text()) {
                // enable buttons
                mySelectChildrenButton->enable();
                myUnselectChildrenButton->enable();
                // change text color
                myChildrenComboBox->setTextColor(FXRGB(0, 0, 0));
                // set current selected parent
                myCurrentSelectedChild = item.first;
                return 1;
            }
        }
        // item not found
        myCurrentSelectedChild = Selection::NOTHING;
        // disable buttons
        mySelectChildrenButton->disable();
        myUnselectChildrenButton->disable();
        myChildrenComboBox->setTextColor(FXRGB(255, 0, 0));
        return 1;
    }
    return 0;
}


long
GNESelectorFrame::SelectionHierarchy::onCmdParents(FXObject* obj, FXSelector, void*) {
    // get selected elements
    const auto selectedACs = mySelectorFrameParent->getViewNet()->getNet()->getAttributeCarriers()->getSelectedAttributeCarriers(true);
    // check if there is selected ACs
    if ((selectedACs.size() > 0) && (myCurrentSelectedParent != Selection::NOTHING)) {
        // vector of hierarchical elements to select
        std::vector<GNEHierarchicalElement*> HEToSelect;
        for (const auto& selectedAC : selectedACs) {
            // get hierarchical element
            const auto HE = selectedAC->getHierarchicalElement();
            // junctions
            if ((myCurrentSelectedParent == Selection::ALL) || (myCurrentSelectedParent == Selection::JUNCTION)) {
                HEToSelect.insert(HEToSelect.end(), HE->getParentJunctions().begin(), HE->getParentJunctions().end());
            }
            // edges
            if ((myCurrentSelectedParent == Selection::ALL) || (myCurrentSelectedParent == Selection::EDGE)) {
                if (selectedAC->getTagProperty().getTag() == SUMO_TAG_LANE) {
                    // special case for lanes
                    HEToSelect.push_back(dynamic_cast<GNELane*>(selectedAC)->getParentEdge());
                } else {
                    HEToSelect.insert(HEToSelect.end(), HE->getParentEdges().begin(), HE->getParentEdges().end());
                }
            }
            // lanes
            if ((myCurrentSelectedParent == Selection::ALL) || (myCurrentSelectedParent == Selection::LANE)) {
                HEToSelect.insert(HEToSelect.end(), HE->getParentLanes().begin(), HE->getParentLanes().end());
            }
            // additional
            if ((myCurrentSelectedParent == Selection::ALL) || (myCurrentSelectedParent == Selection::ADDITIONAL)) {
                HEToSelect.insert(HEToSelect.end(), HE->getParentAdditionals().begin(), HE->getParentAdditionals().end());
            }
            // wire
            if ((myCurrentSelectedParent == Selection::ALL) || (myCurrentSelectedParent == Selection::WIRE)) {
                HEToSelect.insert(HEToSelect.end(), HE->getParentAdditionals().begin(), HE->getParentAdditionals().end());
            }
            // demand
            if ((myCurrentSelectedParent == Selection::ALL) || (myCurrentSelectedParent == Selection::DEMAND)) {
                HEToSelect.insert(HEToSelect.end(), HE->getParentDemandElements().begin(), HE->getParentDemandElements().end());
            }
            // data
            if ((myCurrentSelectedParent == Selection::ALL) || (myCurrentSelectedParent == Selection::DATA)) {
                HEToSelect.insert(HEToSelect.end(), HE->getParentGenericDatas().begin(), HE->getParentGenericDatas().end());
            }
        }
        // select HE
        if (HEToSelect.size() > 0) {
            if (HEToSelect.size() > 1) {
                mySelectorFrameParent->getViewNet()->getUndoList()->begin(GUIIcon::SELECT, TL("select parents"));
            }
            for (const auto& HE : HEToSelect) {
                if (obj == mySelectParentsButton) {
                    HE->setAttribute(GNE_ATTR_SELECTED, "true", mySelectorFrameParent->getViewNet()->getUndoList());
                } else {
                    HE->setAttribute(GNE_ATTR_SELECTED, "false", mySelectorFrameParent->getViewNet()->getUndoList());
                }
            }
            if (HEToSelect.size() > 1) {
                mySelectorFrameParent->getViewNet()->getUndoList()->end();
            }
        }
        // update information label
        mySelectorFrameParent->mySelectionInformation->updateInformationLabel();
        // update viewNet
        mySelectorFrameParent->getViewNet()->update();
    }
    return 1;
}


long
GNESelectorFrame::SelectionHierarchy::onCmdChildren(FXObject* obj, FXSelector, void*) {
    // get selected elements
    const auto selectedACs = mySelectorFrameParent->getViewNet()->getNet()->getAttributeCarriers()->getSelectedAttributeCarriers(true);
    // check if there is selected ACs
    if ((selectedACs.size() > 0) && (myCurrentSelectedChild != Selection::NOTHING)) {
        // vector of hierarchical elements to select
        std::vector<GNEHierarchicalElement*> HEToSelect;
        for (const auto& selectedAC : selectedACs) {
            // get hierarchical element
            const auto HE = selectedAC->getHierarchicalElement();
            // junctions
            if ((myCurrentSelectedChild == Selection::ALL) || (myCurrentSelectedChild == Selection::JUNCTION)) {
                if (selectedAC->getTagProperty().getTag() == SUMO_TAG_JUNCTION) {
                    // special case for junction
                    const auto junction = dynamic_cast<GNEJunction*>(selectedAC);
                    // insert edges
                    HEToSelect.insert(HEToSelect.end(), junction->getGNEIncomingEdges().begin(), junction->getGNEIncomingEdges().end());
                    HEToSelect.insert(HEToSelect.end(), junction->getGNEOutgoingEdges().begin(), junction->getGNEOutgoingEdges().end());
                } else {
                    HEToSelect.insert(HEToSelect.end(), HE->getChildJunctions().begin(), HE->getChildJunctions().end());
                }
            }
            // edges
            if ((myCurrentSelectedChild == Selection::ALL) || (myCurrentSelectedChild == Selection::EDGE)) {
                if (selectedAC->getTagProperty().getTag() == SUMO_TAG_EDGE) {
                    // special case for edges
                    const auto edge = dynamic_cast<GNEEdge*>(selectedAC);
                    // insert lanes
                    HEToSelect.insert(HEToSelect.end(), edge->getLanes().begin(), edge->getLanes().end());
                } else {
                    HEToSelect.insert(HEToSelect.end(), HE->getChildEdges().begin(), HE->getChildEdges().end());
                }
            }
            // connections
            if ((myCurrentSelectedChild == Selection::ALL) || (myCurrentSelectedChild == Selection::CONNECTION)) {
                if (selectedAC->getTagProperty().getTag() == SUMO_TAG_EDGE) {
                    // case for edges
                    const auto edge = dynamic_cast<GNEEdge*>(selectedAC);
                    // insert connections
                    HEToSelect.insert(HEToSelect.end(), edge->getGNEConnections().begin(), edge->getGNEConnections().end());
                } else if (selectedAC->getTagProperty().getTag() == SUMO_TAG_LANE) {
                    // case for lanes
                    const auto lane = dynamic_cast<GNELane*>(selectedAC);
                    // insert connections
                    for (const auto& connection : lane->getParentEdge()->getGNEConnections()) {
                        if (connection->getAttribute(SUMO_ATTR_FROM_LANE) == lane->getAttribute(SUMO_ATTR_INDEX)) {
                            HEToSelect.push_back(connection);
                        }
                    }
                } else if (selectedAC->getTagProperty().getTag() == SUMO_TAG_JUNCTION) {
                    // case for junction
                    const auto junction = dynamic_cast<GNEJunction*>(selectedAC);
                    // get connections
                    const auto connections = junction->getGNEConnections();
                    // insert connections
                    HEToSelect.insert(HEToSelect.end(), connections.begin(), connections.end());
                }
            }
            // crossings
            if ((myCurrentSelectedChild == Selection::ALL) || (myCurrentSelectedChild == Selection::CROSSING)) {
                if (selectedAC->getTagProperty().getTag() == SUMO_TAG_JUNCTION) {
                    // case for junction
                    const auto junction = dynamic_cast<GNEJunction*>(selectedAC);
                    // insert crossings
                    HEToSelect.insert(HEToSelect.end(), junction->getGNECrossings().begin(), junction->getGNECrossings().end());
                }
            }
            // lanes
            if ((myCurrentSelectedChild == Selection::ALL) || (myCurrentSelectedChild == Selection::LANE)) {
                HEToSelect.insert(HEToSelect.end(), HE->getChildLanes().begin(), HE->getChildLanes().end());
            }
            // additional
            if ((myCurrentSelectedChild == Selection::ALL) || (myCurrentSelectedChild == Selection::ADDITIONAL)) {
                // avoid insert symbols
                for (const auto& additionalChild : HE->getChildAdditionals()) {
                    if (!additionalChild->getTagProperty().isWireElement() && !additionalChild->getTagProperty().isSymbol()) {
                        HEToSelect.push_back(additionalChild);
                    }
                }
            }
            // wire
            if ((myCurrentSelectedChild == Selection::ALL) || (myCurrentSelectedChild == Selection::WIRE)) {
                // avoid insert symbols
                for (const auto& wireChild : HE->getChildAdditionals()) {
                    if (wireChild->getTagProperty().isWireElement() && !wireChild->getTagProperty().isSymbol()) {
                        HEToSelect.push_back(wireChild);
                    }
                }
            }
            // demand
            if ((myCurrentSelectedChild == Selection::ALL) || (myCurrentSelectedChild == Selection::DEMAND)) {
                HEToSelect.insert(HEToSelect.end(), HE->getChildDemandElements().begin(), HE->getChildDemandElements().end());
            }
            // data
            if ((myCurrentSelectedChild == Selection::ALL) || (myCurrentSelectedChild == Selection::DATA)) {
                HEToSelect.insert(HEToSelect.end(), HE->getChildGenericDatas().begin(), HE->getChildGenericDatas().end());
            }
        }
        // select HE
        if (HEToSelect.size() > 0) {
            if (HEToSelect.size() > 1) {
                mySelectorFrameParent->getViewNet()->getUndoList()->begin(GUIIcon::SELECT, TL("select children"));
            }
            for (const auto& HE : HEToSelect) {
                if (obj == mySelectChildrenButton) {
                    HE->setAttribute(GNE_ATTR_SELECTED, "true", mySelectorFrameParent->getViewNet()->getUndoList());
                } else {
                    HE->setAttribute(GNE_ATTR_SELECTED, "false", mySelectorFrameParent->getViewNet()->getUndoList());
                }
            }
            if (HEToSelect.size() > 1) {
                mySelectorFrameParent->getViewNet()->getUndoList()->end();
            }
        }
        // update information label
        mySelectorFrameParent->mySelectionInformation->updateInformationLabel();
        // update viewNet
        mySelectorFrameParent->getViewNet()->update();
    }
    return 1;
}

// ---------------------------------------------------------------------------
// GNECrossingFrame::Legend - methods
// ---------------------------------------------------------------------------

GNESelectorFrame::Information::Information(GNESelectorFrame* selectorFrameParent) :
    MFXGroupBoxModule(selectorFrameParent, TL("Information")) {
    // Create Selection Hint
    new MFXDynamicLabel(getCollapsableFrame(), (std::string("- ") + TL("Hold <SHIFT> for rectangle selection.") + std::string("\n- ") + TL("Press <DEL> to delete selected objects.")).c_str(), nullptr, GUIDesignLabelFrameInformation);
}


GNESelectorFrame::Information::~Information() {}

// ---------------------------------------------------------------------------
// GNESelectorFrame - methods
// ---------------------------------------------------------------------------

GNESelectorFrame::GNESelectorFrame(GNEViewParent* viewParent, GNEViewNet* viewNet) :
    GNEFrame(viewParent, viewNet, TL("Selection")) {
    // create selection information
    mySelectionInformation = new SelectionInformation(this);
    // create Modification Mode modul
    myModificationMode = new ModificationMode(this);
    // create ElementSet modul
    myNetworkElementSet = new GNEElementSet(this, Supermode::NETWORK, SUMO_TAG_EDGE, SUMO_ATTR_SPEED, ">10.0");
    myDemandElementSet = new GNEElementSet(this, Supermode::DEMAND, SUMO_TAG_VEHICLE, SUMO_ATTR_ID, "");
    myDataElementSet = new GNEElementSet(this, Supermode::DATA, GNE_TAG_EDGEREL_SINGLE, GNE_ATTR_PARAMETERS, "key=value");
    // create VisualScaling modul
    myVisualScaling = new VisualScaling(this);
    // create SelectionOperation modul
    mySelectionOperation = new SelectionOperation(this);
    // create SelectionHierarchy modul
    mySelectionHierarchy = new SelectionHierarchy(this);
    // create Information modul
    myInformation = new Information(this);
}


GNESelectorFrame::~GNESelectorFrame() {}


void
GNESelectorFrame::show() {
    // refresh element set
    if (myViewNet->getEditModes().isCurrentSupermodeNetwork()) {
        // only show network element set
        myNetworkElementSet->showElementSet();
        myDemandElementSet->hideElementSet();
        myDataElementSet->hideElementSet();
    } else if (myViewNet->getEditModes().isCurrentSupermodeDemand()) {
        // only show demand element set
        myNetworkElementSet->hideElementSet();
        myDemandElementSet->showElementSet();
        myDataElementSet->hideElementSet();
    } else if (myViewNet->getEditModes().isCurrentSupermodeData()) {
        // only show data element set
        myNetworkElementSet->hideElementSet();
        myDemandElementSet->hideElementSet();
        myDataElementSet->showElementSet();
    }
    // update information label
    mySelectionInformation->updateInformationLabel();
    // Show frame
    GNEFrame::show();
}


void
GNESelectorFrame::hide() {
    // hide frame
    GNEFrame::hide();
}


void
GNESelectorFrame::updateFrameAfterUndoRedo() {
    // update information label
    mySelectionInformation->updateInformationLabel();
}


void
GNESelectorFrame::clearCurrentSelection() const {
    mySelectionOperation->onCmdClear(nullptr, 0, nullptr);
}


bool
GNESelectorFrame::selectAttributeCarrier(const GNEViewNetHelper::ViewObjectsSelector& viewObjects) {
    // get front AC
    auto AC = viewObjects.getAttributeCarrierFront();
    // check AC
    if (AC == nullptr) {
        return false;
    }
    // check locking
    if (myViewNet->getLockManager().isObjectLocked(AC->getGUIGlObject()->getType(), AC->isAttributeCarrierSelected())) {
        return false;
    }
    // check modes
    if ((AC->getTagProperty().isNetworkElement() || AC->getTagProperty().isAdditionalElement()) &&
            !myViewNet->getEditModes().isCurrentSupermodeNetwork()) {
        return false;
    }
    if (AC->getTagProperty().isDemandElement() && !myViewNet->getEditModes().isCurrentSupermodeDemand()) {
        return false;
    }
    if (AC->getTagProperty().isDataElement() && !myViewNet->getEditModes().isCurrentSupermodeData()) {
        return false;
    }
    // filter GLObjects by layer
    auto filteredGLObjects = GNEViewNetHelper::filterElementsByLayer(viewObjects.getGLObjects());
    // check if we have to open dialog
    if (filteredGLObjects.size() > 1) {
        myViewNet->openSelectDialogAtCursor(filteredGLObjects);
    } else {
        // toggle selection
        if (AC->isAttributeCarrierSelected()) {
            AC->unselectAttributeCarrier();
        } else {
            AC->selectAttributeCarrier();
        }
        // update information label
        mySelectionInformation->updateInformationLabel();
    }
    return true;
}


void
GNESelectorFrame::handleIDs(const std::vector<GNEAttributeCarrier*>& ACs, const ModificationMode::Operation setop) {
    // declare set operation
    const ModificationMode::Operation setOperation = ((setop == ModificationMode::Operation::DEFAULT) ? myModificationMode->getModificationMode() : setop);
    // declare two sets of attribute carriers, one for select and another for unselect
    std::set<std::pair<std::string, GNEAttributeCarrier*> > ACsToSelect, ACsToUnselect;
    // in restrict AND replace mode all current selected attribute carriers will be unselected
    if ((setOperation == ModificationMode::Operation::REPLACE) || (setOperation == ModificationMode::Operation::RESTRICT)) {
        // obtain selected ACs depending of current supermode
        std::vector<GNEAttributeCarrier*> selectedACs = myViewNet->getNet()->getAttributeCarriers()->getSelectedAttributeCarriers(false);
        // add id into ACs to unselect
        for (const auto& selectedAC : selectedACs) {
            ACsToUnselect.insert(std::make_pair(selectedAC->getID(), selectedAC));
        }
    }
    // handle ids
    for (const auto& AC : ACs) {
        // iterate over AttributeCarriers an place it in ACsToSelect or ACsToUnselect
        switch (setOperation) {
            case GNESelectorFrame::ModificationMode::Operation::SUB:
                ACsToUnselect.insert(std::make_pair(AC->getID(), AC));
                break;
            case GNESelectorFrame::ModificationMode::Operation::RESTRICT:
                if (ACsToUnselect.find(std::make_pair(AC->getID(), AC)) != ACsToUnselect.end()) {
                    ACsToSelect.insert(std::make_pair(AC->getID(), AC));
                }
                break;
            default:
                ACsToSelect.insert(std::make_pair(AC->getID(), AC));
                break;
        }
    }
    // select junctions and their connections if Auto select junctions is enabled (note: only for "add mode")
    if (myViewNet->autoSelectNodes() && (setop == ModificationMode::Operation::ADD)) {
        std::set<GNEEdge*> edgesToSelect;
        // iterate over ACsToSelect and extract edges
        for (const auto& AC : ACsToSelect) {
            if (AC.second->getTagProperty().getTag() == SUMO_TAG_EDGE) {
                edgesToSelect.insert(myViewNet->getNet()->getAttributeCarriers()->retrieveEdge(AC.second->getID()));
            }
        }
        // iterate over extracted edges
        for (const auto& edgeToSelect : edgesToSelect) {
            // select junction source and all connections, crossings and walkingAreas
            ACsToSelect.insert(std::make_pair(edgeToSelect->getFromJunction()->getID(), edgeToSelect->getFromJunction()));
            for (const auto& connectionToSelect : edgeToSelect->getFromJunction()->getGNEConnections()) {
                ACsToSelect.insert(std::make_pair(connectionToSelect->getID(), connectionToSelect));
            }
            for (const auto& fromCrossingToSelect : edgeToSelect->getFromJunction()->getGNECrossings()) {
                ACsToSelect.insert(std::make_pair(fromCrossingToSelect->getID(), fromCrossingToSelect));
            }
            for (const auto& fromWalkingAreaToSelect : edgeToSelect->getFromJunction()->getGNEWalkingAreas()) {
                ACsToSelect.insert(std::make_pair(fromWalkingAreaToSelect->getID(), fromWalkingAreaToSelect));
            }
            // select junction destination and all connections, crossings and walkingAreas
            ACsToSelect.insert(std::make_pair(edgeToSelect->getToJunction()->getID(), edgeToSelect->getToJunction()));
            for (const auto& connectionToSelect : edgeToSelect->getToJunction()->getGNEConnections()) {
                ACsToSelect.insert(std::make_pair(connectionToSelect->getID(), connectionToSelect));
            }
            for (const auto& toCrossingToSelect : edgeToSelect->getToJunction()->getGNECrossings()) {
                ACsToSelect.insert(std::make_pair(toCrossingToSelect->getID(), toCrossingToSelect));
            }
            for (const auto& toWalkingAreaToSelect : edgeToSelect->getToJunction()->getGNEWalkingAreas()) {
                ACsToSelect.insert(std::make_pair(toWalkingAreaToSelect->getID(), toWalkingAreaToSelect));
            }
        }
    }
    // only continue if there is ACs to select or unselect
    if ((ACsToSelect.size() + ACsToUnselect.size()) > 0) {
        // first unselect AC of ACsToUnselect and then selects AC of ACsToSelect
        myViewNet->getUndoList()->begin(GUIIcon::MODESELECT, TL("selection"));
        for (const auto& ACToUnselect : ACsToUnselect) {
            if (ACToUnselect.second->getTagProperty().isSelectable()) {
                ACToUnselect.second->setAttribute(GNE_ATTR_SELECTED, "false", myViewNet->getUndoList());
            }
        }
        for (const auto& ACToSelect : ACsToSelect) {
            if (ACToSelect.second->getTagProperty().isSelectable()) {
                ACToSelect.second->setAttribute(GNE_ATTR_SELECTED, "true", myViewNet->getUndoList());
            }
        }
        // finish operation
        myViewNet->getUndoList()->end();
    }
}


std::vector<GNEAttributeCarrier*>
GNESelectorFrame::getMatches(const SumoXMLTag ACTag, const SumoXMLAttr ACAttr, const char compOp, const double val, const std::string& expr) {
    std::vector<GNEAttributeCarrier*> result;
    // first retrieve all ACs using ACTag
    const auto allACbyTag = myViewNet->getNet()->getAttributeCarriers()->retrieveAttributeCarriers(ACTag);
    // get Tag value
    const auto& tagValue = GNEAttributeCarrier::getTagProperty(ACTag);
    // iterate over all ACs
    for (const auto& AC : allACbyTag) {
        if (expr == "" && compOp == '@') {
            result.push_back(AC);
        } else if (tagValue.hasAttribute(ACAttr) && tagValue.getAttributeProperties(ACAttr).isNumerical()) {
            double acVal;
            std::istringstream buf(AC->getAttribute(ACAttr));
            buf >> acVal;
            switch (compOp) {
                case '<':
                    if (acVal < val) {
                        result.push_back(AC);
                    }
                    break;
                case '>':
                    if (acVal > val) {
                        result.push_back(AC);
                    }
                    break;
                case '=':
                    if (acVal == val) {
                        result.push_back(AC);
                    }
                    break;
            }
        } else {
            // string match
            std::string acVal = AC->getAttributeForSelection(ACAttr);
            switch (compOp) {
                case '@':
                    if (acVal.find(expr) != std::string::npos) {
                        result.push_back(AC);
                    }
                    break;
                case '!':
                    if (acVal.find(expr) == std::string::npos) {
                        result.push_back(AC);
                    }
                    break;
                case '=':
                    if (acVal == expr) {
                        result.push_back(AC);
                    }
                    break;
                case '^':
                    if (acVal != expr) {
                        result.push_back(AC);
                    }
                    break;
            }
        }
    }
    return result;
}


std::vector<GNEAttributeCarrier*>
GNESelectorFrame::getGenericMatches(const std::vector<GNEGenericData*>& genericDatas, const std::string& attr, const char compOp, const double val, const std::string& expr) {
    std::vector<GNEAttributeCarrier*> result;
    // iterate over generic datas
    for (const auto& genericData : genericDatas) {
        if (expr == "" && compOp == '@') {
            result.push_back(genericData);
        } else if (attr != toString(GNE_ATTR_PARENT)) {
            double acVal;
            std::istringstream buf(genericData->getParameter(attr, "0"));
            buf >> acVal;
            switch (compOp) {
                case '<':
                    if (acVal < val) {
                        result.push_back(genericData);
                    }
                    break;
                case '>':
                    if (acVal > val) {
                        result.push_back(genericData);
                    }
                    break;
                case '=':
                    if (acVal == val) {
                        result.push_back(genericData);
                    }
                    break;
            }
        } else {
            // string match
            std::string acVal = genericData->getAttributeForSelection(GNE_ATTR_PARENT);
            switch (compOp) {
                case '@':
                    if (acVal.find(expr) != std::string::npos) {
                        result.push_back(genericData);
                    }
                    break;
                case '!':
                    if (acVal.find(expr) == std::string::npos) {
                        result.push_back(genericData);
                    }
                    break;
                case '=':
                    if (acVal == expr) {
                        result.push_back(genericData);
                    }
                    break;
                case '^':
                    if (acVal != expr) {
                        result.push_back(genericData);
                    }
                    break;
            }
        }
    }
    return result;
}


FXVerticalFrame*
GNESelectorFrame::getContentFrame() const {
    return myContentFrame;
}


GNESelectorFrame::ModificationMode*
GNESelectorFrame::getModificationModeModul() const {
    return myModificationMode;
}


GNESelectorFrame::SelectionOperation*
GNESelectorFrame::getSelectionOperationModul() const {
    return mySelectionOperation;
}


GNESelectorFrame::SelectionInformation*
GNESelectorFrame::getSelectionInformation() const {
    return mySelectionInformation;
}

/****************************************************************************/
