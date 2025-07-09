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
/// @file    InternalTestStep.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Mar 2025
///
// Single operation used in InternalTests
/****************************************************************************/
#include <config.h>

#include <fxkeys.h>

#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/gui/windows/GUISUMOAbstractView.h>

#include "InternalTestStep.h"
#include "InternalTest.h"


// ===========================================================================
// static member definitions
// ===========================================================================

const std::string InternalTestStep::ModalArguments::colorValue = "139,131,120";
// this offsets corresponds to the offset of the test magenta square
constexpr int MOUSE_OFFSET_X = 24;
constexpr int MOUSE_OFFSET_Y = 25;
constexpr int MOUSE_REFERENCE_X = 304;
constexpr int MOUSE_REFERENCE_Y = 168;

// ===========================================================================
// member method definitions
// ===========================================================================

// ---------------------------------------------------------------------------
// GNETLSTable::Test - public methods
// ---------------------------------------------------------------------------

InternalTestStep::ModalArguments::ModalArguments(const std::vector<FXuint> values) :
    questionDialogValues(values) {
}

// ---------------------------------------------------------------------------
// GNETLSTable::Test - public methods
// ---------------------------------------------------------------------------

InternalTestStep::TLSTableTest::TLSTableTest(FXSelector sel_, const int row_) :
    sel(sel_),
    row(row_) {
}


InternalTestStep::TLSTableTest::TLSTableTest(FXSelector sel_, const int row_, const int column_, const std::string& text_) :
    sel(sel_),
    row(row_),
    column(column_),
    text(text_) {
}

// ---------------------------------------------------------------------------
// InternalTestStep - public methods
// ---------------------------------------------------------------------------

InternalTestStep::InternalTestStep(InternalTest* testSystem, const std::string& step) :
    myTestSystem(testSystem) {
    // add this testStep to test system
    testSystem->addTestSteps(this);
    // get overlapped tabs
    const int overlappedTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.editElements.overlapped");
    // parse step
    const auto function = parseStep(step);
    // continue depending of function
    if (function == "setupAndStart") {
        setupAndStart();
    } else if (function == "leftClick") {
        mouseClick("left", "");
    } else if (function == "leftClickControl") {
        mouseClick("left", "control");
    } else if (function == "leftClickShift") {
        mouseClick("left", "shift");
    } else if (function == "rightClick") {
        mouseClick("right", "");
    } else if (function == "rightClickControl") {
        mouseClick("right", "control");
    } else if (function == "rightClickShift") {
        mouseClick("right", "shift");
    } else if (function == "leftClickOffset") {
        leftClickOffset("left");
    } else if (function == "typeKey") {
        typeKey();
    } else if (function == "contextualMenuOperation") {
        contextualMenuOperation();
    } else if (function == "modifyAttribute") {
        modifyAttribute(0);
    } else if (function == "modifyAttributeOverlapped") {
        modifyAttribute(overlappedTabs);
    } else if (function == "modifyBoolAttribute") {
        modifyBoolAttribute(0);
    } else if (function == "modifyBoolAttributeOverlapped") {
        modifyBoolAttribute(overlappedTabs);
    } else if (function == "modifyColorAttribute") {
        modifyColorAttribute(0);
    } else if (function == "modifyColorAttributeOverlapped") {
        modifyColorAttribute(overlappedTabs);
    } else if (function == "modifyVClassDialog_NoDisallowAll") {
        modifyVClassDialog_NoDisallowAll(0);
    } else if (function == "modifyVClassDialogOverlapped_NoDisallowAll") {
        modifyVClassDialog_NoDisallowAll(overlappedTabs);
    } else if (function == "modifyVClassDialog_DisallowAll") {
        modifyVClassDialog_DisallowAll(0);
    } else if (function == "modifyVClassDialogOverlapped_DisallowAll") {
        modifyVClassDialog_DisallowAll(overlappedTabs);
    } else if (function == "modifyVClassDialog_Cancel") {
        modifyVClassDialog_Cancel(0);
    } else if (function == "modifyVClassDialogOverlapped_Cancel") {
        modifyVClassDialog_Cancel(overlappedTabs);
    } else if (function == "modifyVClassDialog_Reset") {
        modifyVClassDialog_Reset(0);
    } else if (function == "modifyVClassDialogOverlapped_Reset") {
        modifyVClassDialog_Reset(overlappedTabs);
    } else if (function == "createConnection") {
        createConnection("");
    } else if (function == "createCrossing") {
        createCrossing();
    } else if (function == "modifyCrossingDefaultValue") {
        modifyCrossingDefaultValue();
    } else if (function == "modifyCrossingDefaultBoolValue") {
        modifyCrossingDefaultBoolValue();
    } else if (function == "crossingClearEdges") {
        crossingClearEdges();
    } else if (function == "crossingInvertEdges") {
        crossingInvertEdges();
    } else if (function == "createConnectionConflict") {
        createConnection("control");
    } else if (function == "createConnectionYield") {
        createConnection("shift");
    } else if (function == "saveConnectionEdit") {
        saveConnectionEdit();
    } else if (function == "createTLS") {
        createTLS(0);
    } else if (function == "createTLSOverlapped") {
        createTLS(overlappedTabs);
    } else if (function == "copyTLS") {
        copyTLS();
    } else if (function == "joinTSL") {
        joinTSL();
    } else if (function == "disJoinTLS") {
        disJoinTLS();
    } else if (function == "deleteTLS") {
        deleteTLS();
    } else if (function == "modifyTLSTable") {
        modifyTLSTable();
    } else if (function == "resetSingleTLSPhases") {
        resetSingleTLSPhases();
    } else if (function == "resetAllTLSPhases") {
        resetAllTLSPhases();
    } else if (function == "pressTLSPhaseButton") {
        pressTLSPhaseButton();
    } else if (function == "addDefaultPhase") {
        addPhase("default");
    } else if (function == "addDuplicatePhase") {
        addPhase("duplicate");
    } else if (function == "addRedPhase") {
        addPhase("red");
    } else if (function == "addYellowPhase") {
        addPhase("yellow");
    } else if (function == "addGreenPhase") {
        addPhase("green");
    } else if (function == "addGreenPriorityPhase") {
        addPhase("priorityGreen");
    } else if (function == "tlsDeletePhase") {
        addPhase("deletePhase");
    } else if (function == "tlsMoveUp") {
        addPhase("moveUp");
    } else if (function == "tlsMoveDown") {
        addPhase("moveDown");
    } else if (function == "tlsCleanStates") {
        pressTLSButton("cleanStates");
    } else if (function == "tlsAddStates") {
        pressTLSButton("addStates");
    } else if (function == "tlsGroupSignal") {
        pressTLSButton("groupSignal");
    } else if (function == "tlsUngroupSignal") {
        pressTLSButton("ungroupSignal");
    } else if (function == "checkParameters") {
        checkParameters(0);
    } else if (function == "checkParametersOverlapped") {
        checkParameters(overlappedTabs);
    } else if (function == "changeEditMode") {
        changeEditMode();
    } else if (function == "changeSupermode") {
        changeSupermode();
    } else if (function == "changeMode") {
        changeMode();
    } else if (function == "changeElement") {
        changeElement();
    } else if (function == "changePlan") {
        changePlan();
    } else if (function == "computeJunctions") {
        computeJunctions();
    } else if (function == "computeJunctionsVolatileOptions") {
        computeJunctionsVolatileOptions();
    } else if (function == "selectAdditionalChild") {
        selectAdditionalChild();
    } else if (function == "createRectangledShape") {
        createRectangledShape();
    } else if (function == "createSquaredShape") {
        createSquaredShape();
    } else if (function == "createLineShape") {
        createLineShape();
    } else if (function == "createMeanData") {
        createMeanData();
    } else if (function == "saveExistentShortcut") {
        saveExistentShortcut();
    } else if (function == "checkUndoRedo") {
        checkUndoRedo();
    } else if (function == "delete") {
        deleteFunction();
    } else if (function == "selection") {
        selection();
    } else if (function == "selectNetworkItems") {
        selectNetworkItems();
    } else if (function == "undo") {
        undo();
    } else if (function == "redo") {
        redo();
    } else if (function == "quit") {
        quit();
    } else if (function.size() > 0) {
        std::cout << function << std::endl;
        throw ProcessError("Function " + function + " not implemented in InternalTestStep");
    }
}


InternalTestStep::InternalTestStep(InternalTest* testSystem, FXSelector messageType,
                                   FXSelector messageID, Category category) :
    myTestSystem(testSystem),
    myMessageType(messageType),
    myMessageID(messageID),
    myCategory(category) {
    // add this testStep to test system
    testSystem->addTestSteps(this);
}


InternalTestStep::InternalTestStep(InternalTest* testSystem, FXSelector messageType,
                                   Category category, FXEvent* event, const bool updateView) :
    myTestSystem(testSystem),
    myMessageType(messageType),
    myCategory(category),
    myUpdateView(updateView),
    myEvent(event) {
    // add this testStep to test system
    testSystem->addTestSteps(this);
}


InternalTestStep::InternalTestStep(InternalTestStep* parent, FXSelector messageType, FXEvent* event) :
    myTestSystem(parent->myTestSystem),
    myMessageType(messageType),
    myEvent(event) {
    // add this testStep to parent modal dialgo testSteps
    parent->myModalDialogTestSteps.push_back(this);
}


InternalTestStep::~InternalTestStep() {
    if (myEvent) {
        delete myEvent;
    }
    if (myModalArguments) {
        delete myModalArguments;
    }
    if (myTLSTableTest) {
        delete myTLSTableTest;
    }
    // remove all key steps
    for (auto modalDialogTestStep : myModalDialogTestSteps) {
        delete modalDialogTestStep;
    }
    myModalDialogTestSteps.clear();
}


FXSelector
InternalTestStep::getMessageType() const {
    return myMessageType;
}


FXSelector
InternalTestStep::getMessageID() const {
    return myMessageID;
}


InternalTestStep::ModalArguments*
InternalTestStep::getModalArguments() const {
    return myModalArguments;
}


InternalTestStep::TLSTableTest*
InternalTestStep::getTLSTableTest() const {
    return myTLSTableTest;
}


FXSelector
InternalTestStep::getSelector() const {
    return FXSEL(myMessageType, myMessageID);
}


bool
InternalTestStep::updateView() const {
    return myUpdateView;
}


InternalTestStep::Category
InternalTestStep::getCategory() const {
    return myCategory;
}


void*
InternalTestStep::getEvent() const {
    return myEvent;
}


const std::vector<const InternalTestStep*>&
InternalTestStep::getModalDialogTestSteps() const {
    return myModalDialogTestSteps;
}


std::string
InternalTestStep::parseStep(const std::string& rowText) {
    // first check if this is the netedit.setupAndStart function
    if (rowText.find("netedit.setupAndStart") != std::string::npos) {
        return "setupAndStart";
    } else if (rowText.compare(0, 8, "netedit.") != 0) {
        // proces only lines that start with "netedit."
        return "";
    } else {
        std::string functionName;
        // make a copy to help editing row
        std::string rowStr = rowText;
        // every function has the format <function>(<argument1>, <argument2>,....,)
        if (rowText.empty() || (rowText.front() == '(') || (rowText.back() != ')')) {
            writeError("parseStep", 0, "function(arguments)");
            return "";
        }
        // first extract function
        while (rowStr.size() > 0) {
            if (rowStr.front() == '(') {
                break;
            } else {
                functionName.push_back(rowStr.front());
                rowStr.erase(rowStr.begin());
            }
        }
        // remove prefix "netedit." (size 8) from function
        functionName = functionName.substr(8);
        // check if there are at least two characters (to avoid cases like 'function)')
        if (rowStr.size() < 2) {
            writeError("parseStep", 0, "function(arguments)");
            return functionName;
        }
        // remove both pharentesis
        rowStr.erase(rowStr.begin());
        rowStr.pop_back();
        // now parse arguments
        parseArguments(rowStr);
        // remove "netedit." from frunction
        return functionName;
    }
}


void
InternalTestStep::parseArguments(const std::string& arguments) {
    std::string current;
    bool inQuotes = false;
    for (size_t i = 0; i < arguments.length(); ++i) {
        char c = arguments[i];
        if (c == '\"' || c == '\'') {
            // Toggle quote state
            inQuotes = !inQuotes;
            current.push_back(c);
        } else if (c == ',' && !inQuotes) {
            // End of argument
            if (!current.empty()) {
                // Trim leading/trailing whitespace
                size_t start = current.find_first_not_of(" \t");
                size_t end = current.find_last_not_of(" \t");
                myArguments.push_back(current.substr(start, end - start + 1));
                current.clear();
            }
        } else {
            current += c;
        }
    }
    // Add the last argument
    if (!current.empty()) {
        size_t start = current.find_first_not_of(" \t");
        size_t end = current.find_last_not_of(" \t");
        myArguments.push_back(current.substr(start, end - start + 1));
    }
    // inQuotes MUST be false, in other case we have a case like < "argument1", argument2, "argument3 >
    if (inQuotes) {
        writeError("parseArguments", 0, "<\"argument\", \"argument\">");
        myArguments.clear();
    }
}


void
InternalTestStep::setupAndStart() {
    myCategory = Category::INIT;
    // print in console the following lines
    std::cout << "TestFunctions: Netedit opened successfully" << std::endl;
    std::cout << "Finding reference" << std::endl;
    std::cout << "TestFunctions: 'reference.png' found. Position: " <<
              toString(MOUSE_REFERENCE_X) << " - " <<
              toString(MOUSE_REFERENCE_Y) << std::endl;
}


void
InternalTestStep::mouseClick(const std::string& button, const std::string& modifier) const {
    if ((myArguments.size() != 2) || (myTestSystem->getViewPositions().count(myArguments[1]) == 0)) {
        writeError("leftClick", 0, "<reference, position>");
    } else {
        // parse view position
        const auto& viewPosition = myTestSystem->getViewPositions().at(myArguments[1]);
        // build mouse click
        buildMouseClick(viewPosition, 0, 0, button, modifier);
        // print info
        writeClickInfo(viewPosition, 0, 0, modifier);
    }
}


void
InternalTestStep::leftClickOffset(const std::string& button) const {
    if ((myArguments.size() != 4) || (myTestSystem->getViewPositions().count(myArguments[1]) == 0) ||
            !checkIntArgument(myArguments[2]) || !checkIntArgument(myArguments[3])) {
        writeError("leftClickOffset", 0, "<reference, position, int, int>");
    } else {
        // parse view position
        const auto& viewPosition = myTestSystem->getViewPositions().at(myArguments[1]);
        const int x = getIntArgument(myArguments[2]);
        const int y = getIntArgument(myArguments[3]);
        // build mouse click
        buildMouseClick(viewPosition, x, y, button, "");
        // print info
        writeClickInfo(viewPosition, x, y, button);
    }
}


void
InternalTestStep::typeKey() const {
    if (myArguments.size() != 1) {
        writeError("typeKey", 0, "<key>");
    } else {
        buildPressKeyEvent(getStringArgument(myArguments[0]), true);
    }
}


void
InternalTestStep::contextualMenuOperation() const {
    if ((myArguments.size() != 3) || (myTestSystem->getViewPositions().count(myArguments[1]) == 0) ||
            (myTestSystem->getContextualMenuOperations().count(myArguments[2]) == 0)) {
        writeError("contextualMenuOperation", 0, "<reference, position, contextualMenuOperations>");
    } else {
        // parse arguments
        const auto& viewPosition = myTestSystem->getViewPositions().at(myArguments[1]);
        const auto& contextualMenu = myTestSystem->getContextualMenuOperations().at(myArguments[2]);
        // build mouse click
        buildMouseClick(viewPosition, 0, 0, "right", "");
        // jump to the element
        for (int i = 0; i < contextualMenu.mainMenu; i++) {
            buildPressKeyEvent("down", false);
        }
        // type space for select
        buildPressKeyEvent("space", false);
        // jump to the subMenuA
        if (contextualMenu.subMenuA > 0) {
            for (int i = 0; i < contextualMenu.subMenuA; i++) {
                buildPressKeyEvent("down", false);
            }
            // type space for select
            buildPressKeyEvent("space", false);
        }
        // jump to the subMenuB
        if (contextualMenu.subMenuB > 0) {
            for (int i = 0; i < contextualMenu.subMenuB; i++) {
                buildPressKeyEvent("down", false);
            }
            // type space for select
            buildPressKeyEvent("space", false);
        }
    }
}


void
InternalTestStep::modifyAttribute(const int overlappedTabs) const {
    if ((myArguments.size() != 2) || !checkIntArgument(myArguments[0]) ||
            !checkStringArgument(myArguments[1])) {
        writeError("modifyAttribute", overlappedTabs, "<int/attributeEnum, \"string\">");
    } else {
        // modify attribute
        modifyStringAttribute(getIntArgument(myArguments[0]), overlappedTabs, getStringArgument(myArguments[1]));
    }
}


void
InternalTestStep::modifyBoolAttribute(const int overlappedTabs) const {
    if ((myArguments.size() != 1) || !checkIntArgument(myArguments[0])) {
        writeError("modifyBoolAttribute", overlappedTabs, "<int/attributeEnum>");
    } else {
        // modify bool attribute
        modifyBoolAttribute(getIntArgument(myArguments[0]), overlappedTabs);
    }
}


void
InternalTestStep::modifyColorAttribute(const int overlappedTabs) const {
    if ((myArguments.size() != 1) || !checkIntArgument(myArguments[0])) {
        writeError("modifyColorAttribute", overlappedTabs, "<int/attributeEnum>");
    } else {
        // open dialog
        auto openDialogEvent = modifyBoolAttribute(getIntArgument(myArguments[0]), overlappedTabs);
        // go to the list of colors
        for (int i = 0; i < 2; i++) {
            buildTwoPressKeyEvent(openDialogEvent, "shift", "tab");
        }
        // select color
        for (int i = 0; i < 6; i++) {
            buildPressKeyEvent(openDialogEvent, "down");
        }
        // go to button
        buildPressKeyEvent(openDialogEvent, "tab");
        // press button
        buildPressKeyEvent(openDialogEvent, "space");
    }
}


void
InternalTestStep::modifyVClassDialog_NoDisallowAll(const int overlappedTabs) const {
    if ((myArguments.size() != 2) || !checkIntArgument(myArguments[0]) ||
            !checkIntArgument(myArguments[1])) {
        writeError("modifyVClassDialog_NoDisallowAll", overlappedTabs, "<int/attributeEnum, int/attributeEnum>");
    } else {
        // open dialog
        auto openDialogEvent = modifyBoolAttribute(getIntArgument(myArguments[0]), overlappedTabs);
        // get vClass
        const int vClass = getIntArgument(myArguments[1]);
        // jump to vClass
        for (int i = 0; i < vClass; i++) {
            buildPressKeyEvent(openDialogEvent, "tab");
        }
        // select vclass
        buildPressKeyEvent(openDialogEvent, "space");
        // go to accept button
        for (int i = 0; i < (myTestSystem->getAttributesEnum().at("netedit.attrs.dialog.allowVClass.accept") - vClass); i++) {
            buildPressKeyEvent(openDialogEvent, "tab");
        }
        // press accept
        buildPressKeyEvent(openDialogEvent, "space");
    }
}


void
InternalTestStep::modifyVClassDialog_DisallowAll(const int overlappedTabs) const {
    if ((myArguments.size() != 2) || !checkIntArgument(myArguments[0]) ||
            !checkIntArgument(myArguments[1])) {
        writeError("modifyVClassDialog_DisallowAll", overlappedTabs, "<int/attributeEnum, int/attributeEnum>");
    } else {
        // open dialog
        auto openDialogEvent = modifyBoolAttribute(getIntArgument(myArguments[0]), overlappedTabs);
        // get vClass
        const int vClass = getIntArgument(myArguments[1]);
        // go to disallow all vehicles
        for (int i = 0; i < myTestSystem->getAttributesEnum().at("netedit.attrs.dialog.allowVClass.disallowAll"); i++) {
            buildPressKeyEvent(openDialogEvent, "tab");
        }
        // disallow all vehicles
        buildPressKeyEvent(openDialogEvent, "space");
        // go to vClass
        for (int i = 0; i < (vClass - myTestSystem->getAttributesEnum().at("netedit.attrs.dialog.allowVClass.disallowAll")); i++) {
            buildPressKeyEvent(openDialogEvent, "tab");
        }
        // select vClass
        buildPressKeyEvent(openDialogEvent, "space");
        // go to accept button
        for (int i = 0; i < (myTestSystem->getAttributesEnum().at("netedit.attrs.dialog.allowVClass.accept") - vClass); i++) {
            buildPressKeyEvent(openDialogEvent, "tab");
        }
        // press accept
        buildPressKeyEvent(openDialogEvent, "space");
    }
}


void
InternalTestStep::modifyVClassDialog_Cancel(const int overlappedTabs) const {
    if ((myArguments.size() != 2) ||
            !checkIntArgument(myArguments[0]) ||
            !checkIntArgument(myArguments[1])) {
        writeError("modifyVClassDialog_Cancel", overlappedTabs, "<int/attributeEnum, int/attributeEnum>");
    } else {
        // open dialog
        auto openDialogEvent = modifyBoolAttribute(getIntArgument(myArguments[0]), overlappedTabs);
        // get vClass
        const int vClass = getIntArgument(myArguments[1]);
        // go to disallow all vehicles
        for (int i = 0; i < myTestSystem->getAttributesEnum().at("netedit.attrs.dialog.allowVClass.disallowAll"); i++) {
            buildPressKeyEvent(openDialogEvent, "tab");
        }
        // disallow all vehicles
        buildPressKeyEvent(openDialogEvent, "space");
        // go to vClass
        for (int i = 0; i < (vClass - myTestSystem->getAttributesEnum().at("netedit.attrs.dialog.allowVClass.disallowAll")); i++) {
            buildPressKeyEvent(openDialogEvent, "tab");
        }
        // select vClass
        buildPressKeyEvent(openDialogEvent, "space");
        // go to cancel button
        for (int i = 0; i < (myTestSystem->getAttributesEnum().at("netedit.attrs.dialog.allowVClass.cancel") - vClass); i++) {
            buildPressKeyEvent(openDialogEvent, "tab");
        }
        // press cancel
        buildPressKeyEvent(openDialogEvent, "space");
    }
}


void
InternalTestStep::modifyVClassDialog_Reset(const int overlappedTabs) const {
    if ((myArguments.size() != 2) || !checkIntArgument(myArguments[0]) ||
            !checkIntArgument(myArguments[1])) {
        writeError("modifyVClassDialog_Reset", overlappedTabs, "<int/attributeEnum, int/attributeEnum>");
    } else {
        // open dialog
        auto openDialogEvent = modifyBoolAttribute(getIntArgument(myArguments[0]), overlappedTabs);
        // get vClass
        const int vClass = getIntArgument(myArguments[1]);
        // go to disallow all vehicles
        for (int i = 0; i < myTestSystem->getAttributesEnum().at("netedit.attrs.dialog.allowVClass.disallowAll"); i++) {
            buildPressKeyEvent(openDialogEvent, "tab");
        }
        // disallow all vehicles
        buildPressKeyEvent(openDialogEvent, "space");
        // go to vClass
        for (int i = 0; i < (vClass - myTestSystem->getAttributesEnum().at("netedit.attrs.dialog.allowVClass.disallowAll")); i++) {
            buildPressKeyEvent(openDialogEvent, "tab");
        }
        // select vClass
        buildPressKeyEvent(openDialogEvent, "space");
        // go to reset button
        for (int i = 0; i < (myTestSystem->getAttributesEnum().at("netedit.attrs.dialog.allowVClass.reset") - vClass); i++) {
            buildPressKeyEvent(openDialogEvent, "tab");
        }
        // press reset
        buildPressKeyEvent(openDialogEvent, "space");
        // go to accept button
        for (int i = 0; i < 2; i++) {
            buildTwoPressKeyEvent(openDialogEvent, "shift", "tab");
        }
        // press accept
        buildPressKeyEvent(openDialogEvent, "space");
    }
}


void
InternalTestStep::createConnection(const std::string& keyModifier) const {
    if ((myArguments.size() != 3) ||
            (myTestSystem->getViewPositions().count(myArguments[1]) == 0) ||
            (myTestSystem->getViewPositions().count(myArguments[2]) == 0)) {
        writeError("createConnection", 0, "<reference, position, position>");
    } else {
        // parse view position
        const auto& fromLane = myTestSystem->getViewPositions().at(myArguments[1]);
        const auto& toLane = myTestSystem->getViewPositions().at(myArguments[2]);
        // build mouse click from
        buildMouseClick(fromLane, 0, 0, "left", keyModifier);
        writeClickInfo(fromLane, 0, 0, "");
        // build mouse click from
        buildMouseClick(toLane, 0, 0, "left", keyModifier);
        writeClickInfo(toLane, 0, 0, "");
    }
}


void
InternalTestStep::createCrossing() const {
    if ((myArguments.size() != 1) || !checkBoolArgument(myArguments[0])) {
        writeError("createCrossing", 0, "<bool>");
    } else {
        if (getBoolArgument(myArguments[0])) {
            modifyBoolAttribute(myTestSystem->getAttributesEnum().at("netedit.attrs.crossing.createTLS.button"), 0);
        } else {
            modifyBoolAttribute(myTestSystem->getAttributesEnum().at("netedit.attrs.crossing.create.button"), 0);
        }
    }
}


void
InternalTestStep::modifyCrossingDefaultValue() const {
    if ((myArguments.size() != 2) || !checkIntArgument(myArguments[0]) || !checkStringArgument(myArguments[1])) {
        writeError("modifyCrossingDefaultValue", 0, "<int, value>");
    } else {
        const int tabs = getIntArgument(myArguments[0]);
        const auto value = getStringArgument(myArguments[1]);
        modifyStringAttribute(tabs, myTestSystem->getAttributesEnum().at("netedit.attrs.crossing.firstField"), value);
    }
}


void
InternalTestStep::modifyCrossingDefaultBoolValue() const {
    if ((myArguments.size() != 1) || !checkIntArgument(myArguments[0])) {
        writeError("modifyCrossingDefaultBoolValue", 0, "<int>");
    } else {
        const int tabs = getIntArgument(myArguments[0]);
        modifyBoolAttribute(tabs, myTestSystem->getAttributesEnum().at("netedit.attrs.crossing.firstField"));
    }
}


void
InternalTestStep::crossingClearEdges() const {
    if (myArguments.size() != 0) {
        writeError("crossingClearEdges", 0, "<>");
    } else {
        modifyBoolAttribute(myTestSystem->getAttributesEnum().at("netedit.attrs.crossing.clearEdges"), 0);
    }
}


void
InternalTestStep::crossingInvertEdges() const {
    if (myArguments.size() != 0) {
        writeError("crossingInvertEdges", 0, "<>");
    } else {
        modifyBoolAttribute(myTestSystem->getAttributesEnum().at("netedit.attrs.crossing.invertEdges"), 0);
    }
}


void
InternalTestStep::saveConnectionEdit() const {
    if (myArguments.size() != 0) {
        writeError("saveConnectionEdit", 0, "<>");
    } else {
        modifyBoolAttribute(myTestSystem->getAttributesEnum().at("netedit.attrs.connection.saveConnections"), 0);
    }
}


void
InternalTestStep::createTLS(const int overlappedTabs) const {
    if (myArguments.size() != 0) {
        writeError("createTLS", overlappedTabs, "<>");
    } else {
        modifyBoolAttribute(myTestSystem->getAttributesEnum().at("netedit.attrs.TLS.create"), overlappedTabs);
    }
}


void
InternalTestStep::copyTLS() const {
    if ((myArguments.size() != 1) || !checkBoolArgument(myArguments[0])) {
        writeError("copyTLS", 0, "<bool>");
    } else {
        if (getBoolArgument(myArguments[0])) {
            modifyBoolAttribute(myTestSystem->getAttributesEnum().at("netedit.attrs.TLS.copyJoined"), 0);
        } else {
            modifyBoolAttribute(myTestSystem->getAttributesEnum().at("netedit.attrs.TLS.copySingle"), 0);
        }
    }
}


void
InternalTestStep::joinTSL() const {
    if (myArguments.size() != 0) {
        writeError("joinTSL", 0, "<>");
    } else {
        modifyBoolAttribute(myTestSystem->getAttributesEnum().at("netedit.attrs.TLS.joinTLS"), 0);
    }
}


void
InternalTestStep::disJoinTLS() const {
    if (myArguments.size() != 0) {
        writeError("disJoinTLS", 0, "<>");
    } else {
        modifyBoolAttribute(myTestSystem->getAttributesEnum().at("netedit.attrs.TLS.disjoinTLS"), 0);
    }
}


void
InternalTestStep::deleteTLS() const {
    if ((myArguments.size() != 1) || !checkBoolArgument(myArguments[0])) {
        writeError("deleteTLS", 0, "<bool>");
    } else {
        if (getBoolArgument(myArguments[0])) {
            modifyBoolAttribute(myTestSystem->getAttributesEnum().at("netedit.attrs.TLS.deleteJoined"), 0);
        } else {
            modifyBoolAttribute(myTestSystem->getAttributesEnum().at("netedit.attrs.TLS.deleteSingle"), 0);
        }
    }
}


void
InternalTestStep::modifyTLSTable() {
    if ((myArguments.size() != 3) || !checkIntArgument(myArguments[0]) ||
            !checkIntArgument(myArguments[1]) || !checkStringArgument(myArguments[2])) {
        writeError("modifyTLSTable", 0, "<row, int/attributeEnum, \"string\">");
    } else {
        myCategory = Category::TLS_PHASETABLE;
        // get row
        const int row = getIntArgument(myArguments[0]);
        const int column = getIntArgument(myArguments[1]);
        const std::string text = getStringArgument(myArguments[2]);
        // modify attribute
        myTLSTableTest = new TLSTableTest(MID_GNE_TLSTABLE_TEXTFIELD, row, column, text);
        // show text
        std::cout << text << std::endl;
    }
}


void
InternalTestStep::resetSingleTLSPhases() const {
    if ((myArguments.size() != 1) || !checkBoolArgument(myArguments[0])) {
        writeError("resetSingleTLSPhases", 0, "<bool>");
    } else {
        if (getBoolArgument(myArguments[0])) {
            modifyBoolAttribute(myTestSystem->getAttributesEnum().at("netedit.attrs.TLS.resetPhaseSingle"));
        } else {
            modifyBoolAttribute(myTestSystem->getAttributesEnum().at("netedit.attrs.TLS.resetPhaseJoined"));
        }
    }
}


void
InternalTestStep::resetAllTLSPhases() const {
    if ((myArguments.size() != 1) || !checkBoolArgument(myArguments[0])) {
        writeError("resetAllTLSPhases", 0, "<bool>");
    } else {
        if (getBoolArgument(myArguments[0])) {
            modifyBoolAttribute(myTestSystem->getAttributesEnum().at("netedit.attrs.TLS.resetAllJoined"));
        } else {
            modifyBoolAttribute(myTestSystem->getAttributesEnum().at("netedit.attrs.TLS.resetAllSingle"));
        }
    }
}


void
InternalTestStep::pressTLSPhaseButton() const {
    if ((myArguments.size() != 1) || !checkIntArgument(myArguments[0])) {
        writeError("pressTLSPhaseButton", 0, "<int/attributeEnum>");
    } else {
        modifyBoolAttribute(getIntArgument(myArguments[0]));
    }
}


void
InternalTestStep::addPhase(const std::string& type) {
    if ((myArguments.size() != 1) || !checkIntArgument(myArguments[0])) {
        writeError("addPhase" + type, 0, "<int/attributeEnum>");
    } else {
        myCategory = Category::TLS_PHASETABLE;
        // get row
        const int row = getIntArgument(myArguments[0]);
        // continue depending of the type of phase
        if (type == "default") {
            myTLSTableTest = new TLSTableTest(MID_GNE_TLSTABLE_ADDPHASE, row);
        } else if (type == "duplicate") {
            myTLSTableTest = new TLSTableTest(MID_GNE_TLSTABLE_COPYPHASE, row);
        } else if (type == "red") {
            myTLSTableTest = new TLSTableTest(MID_GNE_TLSTABLE_ADDPHASEALLRED, row);
        } else if (type == "yellow") {
            myTLSTableTest = new TLSTableTest(MID_GNE_TLSTABLE_ADDPHASEALLYELLOW, row);
        } else if (type == "green") {
            myTLSTableTest = new TLSTableTest(MID_GNE_TLSTABLE_ADDPHASEALLGREEN, row);
        } else if (type == "priorityGreen") {
            myTLSTableTest = new TLSTableTest(MID_GNE_TLSTABLE_ADDPHASEALLGREENPRIORITY, row);
        } else if (type == "deletePhase") {
            myTLSTableTest = new TLSTableTest(MID_GNE_TLSTABLE_REMOVEPHASE, row);
        } else if (type == "moveUp") {
            myTLSTableTest = new TLSTableTest(MID_GNE_TLSTABLE_MOVEUPPHASE, row);
        } else if (type == "moveDown") {
            myTLSTableTest = new TLSTableTest(MID_GNE_TLSTABLE_MOVEDOWNPHASE, row);
        }
    }
}


void
InternalTestStep::pressTLSButton(const std::string& type) {
    if (myArguments.size() != 0) {
        writeError("pressTLSButton" + type, 0, "<>");
    } else {
        myCategory = Category::TLS_PHASES;
        // continue depending of the type of phase
        if (type == "cleanStates") {
            myMessageID = MID_GNE_TLSFRAME_PHASES_CLEANUP;
        } else if (type == "addStates") {
            myMessageID = MID_GNE_TLSFRAME_PHASES_ADDUNUSED;
        } else if (type == "groupSignal") {
            myMessageID = MID_GNE_TLSFRAME_PHASES_GROUPSTATES;
        } else if (type == "ungroupSignal") {
            myMessageID = MID_GNE_TLSFRAME_PHASES_UNGROUPSTATES;
        }
    }
}


void
InternalTestStep::checkParameters(const int overlappedTabs) const {
    if ((myArguments.size() != 2) || !checkIntArgument(myArguments[1])) {
        writeError("checkParameters", 0, "<int/attributeEnum>");
    } else {
        const int tabs = getIntArgument(myArguments[1]);
        // check different values
        modifyStringAttribute(tabs, overlappedTabs, "dummyGenericParameters");
        modifyStringAttribute(tabs, overlappedTabs, "key1|key2|key3");
        modifyStringAttribute(tabs, overlappedTabs, "key1=value1|key2=value2|key3=value3");
        modifyStringAttribute(tabs, overlappedTabs, "key1=|key2=|key3=");
        modifyStringAttribute(tabs, overlappedTabs, "");
        modifyStringAttribute(tabs, overlappedTabs, "key1duplicated=value1|key1duplicated=value2|key3=value3");
        modifyStringAttribute(tabs, overlappedTabs, "key1=valueDuplicated|key2=valueDuplicated|key3=valueDuplicated");
        modifyStringAttribute(tabs, overlappedTabs, "keyInvalid.;%>%$$=value1|key2=value2|key3=value3");
        modifyStringAttribute(tabs, overlappedTabs, "key1=valueInvalid%;%$<>$$%|key2=value2|key3=value3");
        modifyStringAttribute(tabs, overlappedTabs, "keyFinal1=value1|keyFinal2=value2|keyFinal3=value3");
        // check undo-redo
        undo(9);
        redo(9);
    }
}


void
InternalTestStep::changeEditMode() {
    if ((myArguments.size() != 1) || (myTestSystem->getAttributesEnum().count(myArguments[0]) == 0)) {
        writeError("changeEditMode", 0, "<int/attributeEnum>");
    } else {
        myCategory = Category::APP;
        // network
        if (myArguments[0] == "netedit.attrs.modes.network.grid") {
            myMessageID = MID_GNE_NETWORKVIEWOPTIONS_TOGGLEGRID;
        } else if (myArguments[0] == "netedit.attrs.modes.network.junctionShape") {
            myMessageID = MID_GNE_NETWORKVIEWOPTIONS_TOGGLEDRAWJUNCTIONSHAPE;
        } else if (myArguments[0] == "netedit.attrs.modes.network.spreadVehicle") {
            myMessageID = MID_GNE_NETWORKVIEWOPTIONS_DRAWSPREADVEHICLES;
        } else if (myArguments[0] == "netedit.attrs.modes.network.showDemandElements") {
            myMessageID = MID_GNE_NETWORKVIEWOPTIONS_SHOWDEMANDELEMENTS;
        } else if (myArguments[0] == "netedit.attrs.modes.network.selectLane") {
            myMessageID = MID_GNE_NETWORKVIEWOPTIONS_SELECTEDGES;
        } else if (myArguments[0] == "netedit.attrs.modes.network.showConnections") {
            myMessageID = MID_GNE_NETWORKVIEWOPTIONS_SHOWCONNECTIONS;
        } else if (myArguments[0] == "netedit.attrs.modes.network.hideConnetions") {
            myMessageID = MID_GNE_NETWORKVIEWOPTIONS_HIDECONNECTIONS;
        } else if (myArguments[0] == "netedit.attrs.modes.network.showSubAdditionals") {
            myMessageID = MID_GNE_NETWORKVIEWOPTIONS_SHOWSUBADDITIONALS;
        } else if (myArguments[0] == "netedit.attrs.modes.network.showTAZElements") {
            myMessageID = MID_GNE_NETWORKVIEWOPTIONS_SHOWTAZELEMENTS;
        } else if (myArguments[0] == "netedit.attrs.modes.network.automaticSelectJunctions") {
            myMessageID = MID_GNE_NETWORKVIEWOPTIONS_EXTENDSELECTION;
        } else if (myArguments[0] == "netedit.attrs.modes.network.applyAllPhases") {
            myMessageID = MID_GNE_NETWORKVIEWOPTIONS_CHANGEALLPHASES;
        } else if (myArguments[0] == "netedit.attrs.modes.network.mergingJunction") {
            myMessageID = MID_GNE_NETWORKVIEWOPTIONS_MERGEAUTOMATICALLY;
        } else if (myArguments[0] == "netedit.attrs.modes.network.showBubbles") {
            myMessageID = MID_GNE_NETWORKVIEWOPTIONS_SHOWBUBBLES;
        } else if (myArguments[0] == "netedit.attrs.modes.network.moveElevation") {
            myMessageID = MID_GNE_NETWORKVIEWOPTIONS_MOVEELEVATION;
        } else if (myArguments[0] == "netedit.attrs.modes.network.chainMode") {
            myMessageID = MID_GNE_NETWORKVIEWOPTIONS_CHAINEDGES;
        } else if (myArguments[0] == "netedit.attrs.modes.network.twoWayMode") {
            myMessageID = MID_GNE_NETWORKVIEWOPTIONS_AUTOOPPOSITEEDGES;
            // demand
        } else if (myArguments[0] == "netedit.attrs.modes.demand.grid") {
            myMessageID = MID_GNE_DEMANDVIEWOPTIONS_SHOWGRID;
        } else if (myArguments[0] == "netedit.attrs.modes.demand.junctionShape") {
            myMessageID = MID_GNE_DEMANDVIEWOPTIONS_TOGGLEDRAWJUNCTIONSHAPE;
        } else if (myArguments[0] == "netedit.attrs.modes.demand.spreadVehicle") {
            myMessageID = MID_GNE_DEMANDVIEWOPTIONS_DRAWSPREADVEHICLES;
        } else if (myArguments[0] == "netedit.attrs.modes.demand.showNonInspected") {
            myMessageID = MID_GNE_DEMANDVIEWOPTIONS_HIDENONINSPECTED;
        } else if (myArguments[0] == "netedit.attrs.modes.demand.showShapes") {
            myMessageID = MID_GNE_DEMANDVIEWOPTIONS_HIDESHAPES;
        } else if (myArguments[0] == "netedit.attrs.modes.demand.showAllTrips") {
            myMessageID = MID_GNE_DEMANDVIEWOPTIONS_SHOWTRIPS;
        } else if (myArguments[0] == "netedit.attrs.modes.demand.showPersonPlans") {
            myMessageID = MID_GNE_DEMANDVIEWOPTIONS_SHOWALLPERSONPLANS;
        } else if (myArguments[0] == "netedit.attrs.modes.demand.lockPerson") {
            myMessageID = MID_GNE_DEMANDVIEWOPTIONS_LOCKPERSON;
        } else if (myArguments[0] == "netedit.attrs.modes.demand.showContainerPlans") {
            myMessageID = MID_GNE_DEMANDVIEWOPTIONS_SHOWALLCONTAINERPLANS;
        } else if (myArguments[0] == "netedit.attrs.modes.demand.lockContainer") {
            myMessageID = MID_GNE_DEMANDVIEWOPTIONS_LOCKCONTAINER;
        } else if (myArguments[0] == "netedit.attrs.modes.demand.showOverlappedRoutes") {
            myMessageID = MID_GNE_DEMANDVIEWOPTIONS_SHOWOVERLAPPEDROUTES;
            // data
        } else if (myArguments[0] == "netedit.attrs.modes.data.junctionShape") {
            myMessageID = MID_GNE_DATAVIEWOPTIONS_TOGGLEDRAWJUNCTIONSHAPE;
        } else if (myArguments[0] == "netedit.attrs.modes.data.showAdditionals") {
            myMessageID = MID_GNE_DATAVIEWOPTIONS_SHOWADDITIONALS;
        } else if (myArguments[0] == "netedit.attrs.modes.data.showShapes") {
            myMessageID = MID_GNE_DATAVIEWOPTIONS_SHOWSHAPES;
        } else if (myArguments[0] == "netedit.attrs.modes.data.showDemandElements") {
            myMessageID = MID_GNE_DATAVIEWOPTIONS_SHOWDEMANDELEMENTS;
        } else if (myArguments[0] == "netedit.attrs.modes.data.TAZRelDrawingMode") {
            myMessageID = MID_GNE_DATAVIEWOPTIONS_TAZRELDRAWING;
        } else if (myArguments[0] == "netedit.attrs.modes.data.TAZFill") {
            myMessageID = MID_GNE_DATAVIEWOPTIONS_TAZDRAWFILL;
        } else if (myArguments[0] == "netedit.attrs.modes.data.TAZRelOnlyFrom") {
            myMessageID = MID_GNE_DATAVIEWOPTIONS_TAZRELONLYFROM;
        } else if (myArguments[0] == "netedit.attrs.modes.data.TAZRelOnlyTo") {
            myMessageID = MID_GNE_DATAVIEWOPTIONS_TAZRELONLYTO;
        } else {
            writeError("changeEditMode", 0, "<enum>");
        }
    }
}


void
InternalTestStep::saveExistentShortcut() {
    if ((myArguments.size() != 1) ||
            !checkStringArgument(myArguments[0])) {
        writeError("save", 0, "<\"string\">");
    } else {
        myCategory = Category::APP;
        const auto savingType = getStringArgument(myArguments[0]);
        if (savingType == "network") {
            myMessageID = MID_HOTKEY_CTRL_S_STOPSIMULATION_SAVENETWORK;
        } else if (savingType == "additionals") {
            myMessageID = MID_HOTKEY_CTRL_SHIFT_A_SAVEADDITIONALELEMENTS;
        } else if (savingType == "demands") {
            myMessageID = MID_HOTKEY_CTRL_SHIFT_D_SAVEDEMANDELEMENTS;
        } else if (savingType == "datas") {
            myMessageID = MID_HOTKEY_CTRL_SHIFT_B_SAVEDATAELEMENTS;
        } else if (savingType == "meanDatas") {
            myMessageID = MID_HOTKEY_CTRL_SHIFT_M_SAVEMEANDATAELEMENTS;
        } else if (savingType == "sumoConfig") {
            myMessageID = MID_HOTKEY_CTRL_SHIFT_S_SAVESUMOCONFIG;
        } else if (savingType == "neteditConfig") {
            myMessageID = MID_HOTKEY_CTRL_SHIFT_E_SAVENETEDITCONFIG;
        } else {
            writeError("save", 0, "<neteditConfig>");
        }
    }
}


void
InternalTestStep::checkUndoRedo() const {
    if (myArguments.size() != 1) {
        writeError("checkUndoRedo", 0, "<referencePosition>");
    } else {
        const int numUndoRedos = 9;
        undo(numUndoRedos);
        redo(numUndoRedos);
    }
}


void
InternalTestStep::deleteFunction() const {
    if (myArguments.size() != 0) {
        writeError("delete", 0, "<>");
    } else {
        new InternalTestStep(myTestSystem, SEL_COMMAND, MID_HOTKEY_DEL, Category::APP);
    }
}


void
InternalTestStep::selection() const {
    if (myArguments.size() != 1 || !checkStringArgument(myArguments[0])) {
        writeError("selection", 0, "<selection operation>");
    } else {
        const std::string selectionType = getStringArgument(myArguments[0]);
        // get number of tabls
        int numTabs = 0;
        if (selectionType == "default") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.selection.basic.default");
        } else if (selectionType == "save") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.selection.basic.save");
        } else if (selectionType == "load") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.selection.basic.load");
        } else if (selectionType == "add") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.selection.basic.add");
        } else if (selectionType == "remove") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.selection.basic.remove");
        } else if (selectionType == "keep") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.selection.basic.keep");
        } else if (selectionType == "replace") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.selection.basic.replace");
        } else if (selectionType == "clear") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.selection.basic.clear");
        } else if (selectionType == "invert") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.selection.basic.invert");
        } else if (selectionType == "invertData") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.selection.basic.invertData");
        } else if (selectionType == "delete") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.selection.basic.delete");
        }
        // focus frame
        new InternalTestStep(myTestSystem, SEL_COMMAND, MID_HOTKEY_SHIFT_F12_FOCUSUPPERELEMENT, Category::APP);
        // jump to the element
        for (int i = 0; i < numTabs; i++) {
            buildPressKeyEvent("tab", false);
        }
        if (selectionType == "save") {
            buildPressKeyEvent("enter", false);
            // complete
        } else if (selectionType == "load") {
            buildPressKeyEvent("enter", false);
            // complete
        } else {
            buildPressKeyEvent("space", true);
        }
    }
}


void
InternalTestStep::selectNetworkItems() const  {
    if (myArguments.size() != 3 || !checkStringArgument(myArguments[0]) ||
            !checkStringArgument(myArguments[1]) || !checkStringArgument(myArguments[2])) {
        writeError("selectNetworkItems", 0, "<element/int, \"attribute\", \"value\">");
    } else {
        const std::string element = getStringArgument(myArguments[0]);
        const std::string attribute = getStringArgument(myArguments[1]);
        const std::string value = getStringArgument(myArguments[2]);
        // focus frame
        new InternalTestStep(myTestSystem, SEL_COMMAND, MID_HOTKEY_SHIFT_F12_FOCUSUPPERELEMENT, Category::APP);
        // got to type
        for (int i = 0; i < myTestSystem->getAttributesEnum().at("netedit.attrs.frames.selection.networkItem.type"); i++) {
            buildPressKeyEvent("tab", false);
        }
        // set network element
        for (const char c : "Network elements") {
            buildPressKeyEvent(c, false);
        }
        // show info
        std::cout << "Network elements" << std::endl;
        // got to type
        for (int i = 0; i < myTestSystem->getAttributesEnum().at("netedit.attrs.frames.selection.networkItem.subType"); i++) {
            buildPressKeyEvent("tab", false);
        }
        // set network element
        for (const char c : element) {
            buildPressKeyEvent(c, false);
        }
        // show info
        std::cout << element << std::endl;
        // got to attribute
        for (int i = 0; i < myTestSystem->getAttributesEnum().at("netedit.attrs.frames.selection.networkItem.attribute"); i++) {
            buildPressKeyEvent("tab", false);
        }
        // set attribute
        for (const char c : attribute) {
            buildPressKeyEvent(c, false);
        }
        // show info
        std::cout << attribute << std::endl;
        // got to value
        for (int i = 0; i < myTestSystem->getAttributesEnum().at("netedit.attrs.frames.selection.networkItem.value"); i++) {
            buildPressKeyEvent("tab", false);
        }
        // set value
        for (const char c : value) {
            buildPressKeyEvent(c, false);
        }
        // show info
        std::cout << value << std::endl;
        // press enter to confirm changes (updating view)
        buildPressKeyEvent("enter", true);
    }
}


void
InternalTestStep::undo() const {
    if ((myArguments.size() != 2) || !checkIntArgument(myArguments[1])) {
        writeError("undo", 0, "<referencePosition, int>");
    } else {
        // do undo
        undo(getIntArgument(myArguments[1]));
    }
}


void
InternalTestStep::redo() const {
    if ((myArguments.size() != 2) || !checkIntArgument(myArguments[1])) {
        writeError("redo", 0, "<referencePosition, int>");
    } else {
        // do redo
        redo(getIntArgument(myArguments[1]));
    }
}


void
InternalTestStep::changeSupermode() {
    if ((myArguments.size() != 1) ||
            !checkStringArgument(myArguments[0])) {
        writeError("supermode", 0, "<\"string\">");
    } else {
        myCategory = Category::APP;
        const std::string supermode = getStringArgument(myArguments[0]);
        if (supermode == "network") {
            myMessageID = MID_HOTKEY_F2_SUPERMODE_NETWORK;
        } else if (supermode == "demand") {
            myMessageID = MID_HOTKEY_F3_SUPERMODE_DEMAND;
        } else if (supermode == "data") {
            myMessageID = MID_HOTKEY_F4_SUPERMODE_DATA;
        } else {
            writeError("supermode", 0, "<network/demand/data>");
        }
    }
}


void
InternalTestStep::changeMode() {
    if ((myArguments.size() != 1) ||
            !checkStringArgument(myArguments[0])) {
        writeError("changeMode", 0, "<\"string\">");
    } else {
        myCategory = Category::APP;
        const std::string networkMode = getStringArgument(myArguments[0]);
        if (networkMode == "inspect") {
            myMessageID = MID_HOTKEY_I_MODE_INSPECT;
        } else if (networkMode == "delete") {
            myMessageID = MID_HOTKEY_D_MODE_SINGLESIMULATIONSTEP_DELETE;
        } else if (networkMode == "select") {
            myMessageID = MID_HOTKEY_S_MODE_STOPSIMULATION_SELECT;
        } else if (networkMode == "move") {
            myMessageID = MID_HOTKEY_M_MODE_MOVE_MEANDATA;
        } else if ((networkMode == "createEdge") || (networkMode == "edgeData")) {
            myMessageID = MID_HOTKEY_E_MODE_EDGE_EDGEDATA;
        } else if ((networkMode == "trafficLight") || (networkMode == "type") || (networkMode == "TLS")) {
            myMessageID = MID_HOTKEY_T_MODE_TLS_TYPE;
        } else if ((networkMode == "connection") || (networkMode == "container")) {
            myMessageID = MID_HOTKEY_C_MODE_CONNECT_CONTAINER;
        } else if ((networkMode == "prohibition") || (networkMode == "containerPlan")) {
            myMessageID = MID_HOTKEY_H_MODE_PROHIBITION_CONTAINERPLAN;
        } else if ((networkMode == "crossing") || (networkMode == "route") || (networkMode == "edgeRelData")) {
            myMessageID = MID_HOTKEY_R_MODE_CROSSING_ROUTE_EDGERELDATA;
        } else if ((networkMode == "additional") || (networkMode == "stop")) {
            myMessageID = MID_HOTKEY_A_MODE_STARTSIMULATION_ADDITIONALS_STOPS;
        } else if ((networkMode == "wire") || (networkMode == "routeDistribution")) {
            myMessageID = MID_HOTKEY_W_MODE_WIRE_ROUTEDISTRIBUTION;
        } else if ((networkMode == "taz") || (networkMode == "TAZ") || (networkMode == "tazRel")) {
            myMessageID = MID_HOTKEY_Z_MODE_TAZ_TAZREL;
        } else if ((networkMode == "shape") || (networkMode == "person")) {
            myMessageID = MID_HOTKEY_P_MODE_POLYGON_PERSON;
        } else if ((networkMode == "decal") || (networkMode == "typeDistribution")) {
            myMessageID = MID_HOTKEY_U_MODE_DECAL_TYPEDISTRIBUTION;
        } else if (networkMode == "personPlan") {
            myMessageID = MID_HOTKEY_L_MODE_PERSONPLAN;
        } else if (networkMode == "vehicle") {
            myMessageID = MID_HOTKEY_V_MODE_VEHICLE;
        } else if (networkMode == "meanData") {
            myMessageID = MID_HOTKEY_M_MODE_MOVE_MEANDATA;
        } else {
            writeError("changeMode", 0, "<inspect/delete/select/move...>");
        }
    }
}


void
InternalTestStep::changeElement() const {
    if ((myArguments.size() != 2) ||
            !checkStringArgument(myArguments[0])) {
        writeError("selectAdditional", 0, "<\"frame\", \"string\">");
    } else {
        const std::string frame = getStringArgument(myArguments[0]);
        const std::string element = getStringArgument(myArguments[1]);
        int numTabs = -1;
        // continue depending of frame
        if (frame == "additionalFrame") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.changeElement.additional");
        } else if (frame == "shapeFrame") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.changeElement.shape");
        } else if (frame == "vehicleFrame") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.changeElement.vehicle");
        } else if (frame == "routeFrame") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.changeElement.route");
        } else if (frame == "personFrame") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.changeElement.person");
        } else if (frame == "containerFrame") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.changeElement.container");
        } else if (frame == "personPlanFrame") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.changeElement.personPlan");
        } else if (frame == "containerPlanFrame") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.changeElement.containerPlan");
        } else if (frame == "stopFrame") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.changeElement.stop");
        } else if (frame == "meanDataFrame") {
            numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.changeElement.meanData");
        } else {
            WRITE_ERRORF("Invalid frame '%' used in function changeElement", frame);
        }
        if (numTabs >= 0) {
            // show info
            std::cout << element << std::endl;
            // focus frame
            new InternalTestStep(myTestSystem, SEL_COMMAND, MID_HOTKEY_SHIFT_F12_FOCUSUPPERELEMENT, Category::APP);
            // jump to select additional argument
            for (int i = 0; i < numTabs; i++) {
                buildPressKeyEvent("tab", false);
            }
            // write additional character by character
            for (const char c : element) {
                buildPressKeyEvent(c, false);
            }
            // press enter to confirm changes (updating view)
            buildPressKeyEvent("enter", true);
        }
    }
}


void
InternalTestStep::changePlan()  const {
    if ((myArguments.size() != 3) ||
            !checkStringArgument(myArguments[0]) ||
            !checkStringArgument(myArguments[1]) ||
            !checkBoolArgument(myArguments[2])) {
        writeError("changePlan", 0, "<\"type\", \"plan\", true/false>");
    } else {
        // get arguments
        const std::string type = getStringArgument(myArguments[0]);
        const std::string plan = getStringArgument(myArguments[1]);
        const bool flow = getBoolArgument(myArguments[2]);
        // check plan
        if ((type != "person") && (type != "container")) {
            WRITE_ERRORF("invalid plan type '%' used in changePlan()", type);
        } else {
            // calculate num tabs
            int numTabs = 0;
            if (flow) {
                numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.changePlan." + type + "Flow");
            } else {
                numTabs = myTestSystem->getAttributesEnum().at("netedit.attrs.frames.changePlan." + type);
            }
            // focus frame
            new InternalTestStep(myTestSystem, SEL_COMMAND, MID_HOTKEY_SHIFT_F12_FOCUSUPPERELEMENT, Category::APP);
            // jump to select additional argument
            for (int i = 0; i < numTabs; i++) {
                buildPressKeyEvent("tab", false);
            }
            // write additional character by character
            for (const char c : plan) {
                buildPressKeyEvent(c, false);
            }
            // print info
            std::cout << plan << std::endl;
            // press enter to confirm changes (updating view)
            buildPressKeyEvent("enter", true);
        }
    }
}


void
InternalTestStep::computeJunctions() {
    if (myArguments.size() > 0) {
        writeError("computeJunctions", 0, "<>");
    } else {
        myCategory = Category::APP;
        myMessageID = MID_HOTKEY_F5_COMPUTE_NETWORK_DEMAND;
    }
}


void
InternalTestStep::computeJunctionsVolatileOptions() {
    if (myArguments.size() > 1) {
        writeError("computeJunctionsVolatileOptions", 0, "<True/False>");
    } else {
        FXuint result = ModalArguments::yes;
        if ((myArguments.size() == 1) && (myArguments[0] == "False")) {
            result = ModalArguments::no;
        }
        myCategory = Category::APP;
        myMessageID = MID_HOTKEY_SHIFT_F5_COMPUTEJUNCTIONS_VOLATILE;
        myModalArguments = new ModalArguments({result});
    }
}


void
InternalTestStep::selectAdditionalChild() {
    if ((myArguments.size() != 2) ||
            !checkIntArgument(myArguments[0]) ||
            !checkIntArgument(myArguments[1])) {
        writeError("selectAdditionalChild", 0, "<int, int>");
    } else {
        const auto tabs = getIntArgument(myArguments[0]);
        const auto downs = getIntArgument(myArguments[1]);
        // focus frame
        new InternalTestStep(myTestSystem, SEL_COMMAND, MID_HOTKEY_SHIFT_F12_FOCUSUPPERELEMENT, Category::APP);
        // jump to the element
        for (int i = 0; i < tabs; i++) {
            buildPressKeyEvent("tab", false);
        }
        // jump to the element
        for (int i = 0; i < downs; i++) {
            buildPressKeyEvent("down", false);
        }
        // select additional child
        buildPressKeyEvent("space", true);
        // leave
        buildPressKeyEvent("tab", true);
    }
}


void
InternalTestStep::createRectangledShape() {
    if ((myArguments.size() != 5) ||
            (myTestSystem->getViewPositions().count(myArguments[1]) == 0) ||
            !checkIntArgument(myArguments[2]) ||
            !checkIntArgument(myArguments[3]) ||
            !checkBoolArgument(myArguments[4])) {
        writeError("createRectangledShape", 0, "<viewPosition, sizeX, sizeY, true/false>");
    } else {
        // create shape
        createShape(myTestSystem->getViewPositions().at(myArguments[1]),
                    getIntArgument(myArguments[2]),
                    getIntArgument(myArguments[3]),
                    getBoolArgument(myArguments[4]),
                    false);
    }
}


void
InternalTestStep::createSquaredShape() {
    if ((myArguments.size() != 4) ||
            (myTestSystem->getViewPositions().count(myArguments[1]) == 0) ||
            !checkIntArgument(myArguments[2]) ||
            !checkBoolArgument(myArguments[3])) {
        writeError("createSquaredShape", 0, "<viewPosition, size, true/false>");
    } else {
        // create shape
        createShape(myTestSystem->getViewPositions().at(myArguments[1]),
                    getIntArgument(myArguments[2]),
                    getIntArgument(myArguments[2]),
                    getBoolArgument(myArguments[3]),
                    false);
    }
}


void
InternalTestStep::createLineShape() {
    if ((myArguments.size() != 5) ||
            (myTestSystem->getViewPositions().count(myArguments[1]) == 0) ||
            !checkIntArgument(myArguments[2]) ||
            !checkIntArgument(myArguments[3]) ||
            !checkBoolArgument(myArguments[4])) {
        writeError("createLineShape", 0, "<viewPosition, sizeX, sizeY, true/false>");
    } else {
        // create shape
        createShape(myTestSystem->getViewPositions().at(myArguments[1]),
                    getIntArgument(myArguments[2]),
                    getIntArgument(myArguments[3]),
                    getBoolArgument(myArguments[4]),
                    true);
    }
}


void
InternalTestStep::createMeanData() {
    if (myArguments.size() != 0) {
        writeError("createMeanData", 0, "<>");
    } else {
        modifyBoolAttribute(5, 0);
    }
}


void
InternalTestStep::quit() {
    if (myArguments.size() == 0) {
        writeError("quit", 0, "<neteditProcess>");
    } else {
        myCategory = Category::APP;
        myMessageID = MID_HOTKEY_CTRL_Q_CLOSE;
        //don't update view if we're closing to avoid problems with drawGL
        myUpdateView = false;
    }
}


bool
InternalTestStep::checkIntArgument(const std::string& argument) const {
    if (StringUtils::isInt(argument)) {
        return true;
    } else if (myTestSystem->getAttributesEnum().count(argument) > 0) {
        return true;
    } else {
        return false;
    }
}


int
InternalTestStep::getIntArgument(const std::string& argument) const {
    if (StringUtils::isInt(argument)) {
        return StringUtils::toInt(argument);
    } else {
        return myTestSystem->getAttributesEnum().at(argument);
    }
}


bool
InternalTestStep::checkBoolArgument(const std::string& argument) const {
    if (argument == "True") {
        return true;
    } else if (argument == "False") {
        return true;
    } else {
        return false;
    }
}


bool
InternalTestStep::getBoolArgument(const std::string& argument) const {
    if (argument == "True") {
        return true;
    } else {
        return false;
    }
}


bool
InternalTestStep::checkStringArgument(const std::string& argument) const {
    if (argument.size() < 2) {
        return false;
    } else if ((argument.front() != argument.back()) || ((argument.front() != '\'') && ((argument.front() != '\"')))) {
        return false;
    } else {
        return true;
    }
}


std::string
InternalTestStep::getStringArgument(const std::string& argument) const {
    std::string argumentParsed;
    for (int i = 1; i < ((int)argument.size() - 1); i++) {
        argumentParsed.push_back(argument[i]);
    }
    return argumentParsed;
}


std::string
InternalTestStep::stripSpaces(const std::string& str) const {
    auto start = std::find_if_not(str.begin(), str.end(), isspace);
    auto end = std::find_if_not(str.rbegin(), str.rend(), isspace).base();
    if (start < end) {
        return std::string(start, end);
    } else {
        return "";
    }
}


void
InternalTestStep::writeError(const std::string& function, const int overlapping, const std::string& expected) const {
    if (overlapping > 0) {
        WRITE_ERRORF("Invalid internal testStep function '%Ovelapped', requires '%' arguments ", function, expected);
    } else {
        WRITE_ERRORF("Invalid internal testStep function '%', requires '%' arguments ", function, expected);
    }
}


void
InternalTestStep::createShape(const InternalTest::ViewPosition& viewPosition,
                              const int sizeX, const int sizeY, const bool close,
                              const bool line) const {
    // calculate half-sizes
    const int halfSizeX = int(sizeX * -0.5);
    const int halfSizeY = int(sizeY * -0.5);
    // focus frame
    new InternalTestStep(myTestSystem, SEL_COMMAND, MID_HOTKEY_SHIFT_F12_FOCUSUPPERELEMENT, Category::APP);
    // press enter to start drawing
    buildPressKeyEvent("enter", true);
    // first edge
    buildMouseClick(viewPosition, 0, 0, "left", "");
    writeClickInfo(viewPosition, 0, 0, "");
    // second edge
    if (!line) {
        buildMouseClick(viewPosition, 0, halfSizeY, "left", "");
        writeClickInfo(viewPosition, 0, halfSizeY, "");
    }
    // third edge
    buildMouseClick(viewPosition, halfSizeX, halfSizeY, "left", "");
    writeClickInfo(viewPosition, halfSizeX, halfSizeY, "");
    // four edge
    if (!line) {
        buildMouseClick(viewPosition, halfSizeX, 0, "left", "");
        writeClickInfo(viewPosition, halfSizeX, 0, "");
    }
    // check if close polygon
    if (close) {
        buildMouseClick(viewPosition, 0, 0, "left", "");
        writeClickInfo(viewPosition, 0, 0, "");
    }
    // press enter to end drawing
    buildPressKeyEvent("enter", true);
}


void
InternalTestStep::modifyStringAttribute(const int tabs, const int overlappedTabs, const std::string& value) const {
    // print info
    std::cout << value << std::endl;
    // focus frame
    new InternalTestStep(myTestSystem, SEL_COMMAND, MID_HOTKEY_SHIFT_F12_FOCUSUPPERELEMENT, Category::APP);
    // jump to the element
    for (int i = 0; i < (tabs + overlappedTabs); i++) {
        buildPressKeyEvent("tab", false);
    }
    // write attribute character by character
    if (value.empty()) {
        buildPressKeyEvent("delete", false);
    } else {
        for (const char c : value) {
            buildPressKeyEvent(c, false);
        }
    }
    // press enter to confirm changes (updating view)
    buildPressKeyEvent("enter", true);
}


InternalTestStep*
InternalTestStep::modifyBoolAttribute(const int tabs, const int overlappedTabs) const {
    // focus frame
    new InternalTestStep(myTestSystem, SEL_COMMAND, MID_HOTKEY_SHIFT_F12_FOCUSUPPERELEMENT, Category::APP);
    // jump to the element
    for (int i = 0; i < (tabs + overlappedTabs); i++) {
        buildPressKeyEvent("tab", false);
    }
    // toogle attribute
    return buildPressKeyEvent("space", true);
}


void
InternalTestStep::undo(const int number) const {
    // get reference position
    const auto& referencePosition = myTestSystem->getViewPositions().at("netedit.positions.reference");
    // focus frame
    new InternalTestStep(myTestSystem, SEL_COMMAND, MID_HOTKEY_SHIFT_F12_FOCUSUPPERELEMENT, Category::APP);
    // go to inspect mode
    new InternalTestStep(myTestSystem, SEL_COMMAND, MID_HOTKEY_I_MODE_INSPECT, Category::APP);
    // click over reference
    std::cout << "TestFunctions: Clicked over position " <<
              toString(MOUSE_REFERENCE_X) << " - " <<
              toString(MOUSE_REFERENCE_Y) << std::endl;
    // build mouse click
    buildMouseClick(referencePosition, 0, 0, "left", "");
    // undo
    for (int i = 0; i < number; i++) {
        new InternalTestStep(myTestSystem, SEL_COMMAND, MID_HOTKEY_CTRL_Z_UNDO, Category::APP);
    }
}


void
InternalTestStep::redo(const int number) const {
    // get reference position
    const auto& referencePosition = myTestSystem->getViewPositions().at("netedit.positions.reference");
    // focus frame
    new InternalTestStep(myTestSystem, SEL_COMMAND, MID_HOTKEY_SHIFT_F12_FOCUSUPPERELEMENT, Category::APP);
    // go to inspect mode
    new InternalTestStep(myTestSystem, SEL_COMMAND, MID_HOTKEY_I_MODE_INSPECT, Category::APP);
    // click over reference
    std::cout << "TestFunctions: Clicked over position " <<
              toString(MOUSE_REFERENCE_X) << " - " <<
              toString(MOUSE_REFERENCE_Y) << std::endl;
    // build mouse click
    buildMouseClick(referencePosition, 0, 0, "left", "");
    // undo
    for (int i = 0; i < number; i++) {
        new InternalTestStep(myTestSystem, SEL_COMMAND, MID_HOTKEY_CTRL_Y_REDO, Category::APP);
    }
}


template <> std::pair<FXint, FXString>
InternalTestStep::translateKey(const std::string key) const {
    std::pair<FXint, FXString> solution;
    // continue depending of key
    if (key == "backspace") {
        solution.first = KEY_BackSpace;
        solution.second = "\b";
    } else if (key == "space") {
        solution.first = KEY_space;
    } else if (key == "tab") {
        solution.first = KEY_Tab;
        solution.second = "\t";
    } else if (key == "clear") {
        solution.first = KEY_Clear;
    } else if (key == "enter" || key == "return") {
        solution.first = KEY_Return;
        solution.second = "\n";
    } else if (key == "pause") {
        solution.first = KEY_Pause;
    } else if (key == "sys_req") {
        solution.first = KEY_Sys_Req;
    } else if (key == "esc" || key == "escape") {
        solution.first = KEY_Escape;
        solution.second = "\x1B";
    } else if (key == "delete") {
        solution.first = KEY_Delete;
        solution.second = "\x7F";
    } else if (key == "multi_key") {
        solution.first = KEY_Multi_key;
        // function
    } else if (key == "shift") {
        solution.first = KEY_Shift_L;
    } else if (key == "control") {
        solution.first = KEY_Control_L;
        // Cursor
    } else if (key == "home") {
        solution.first = KEY_Home;
    } else if (key == "left") {
        solution.first = KEY_Left;
    } else if (key == "up") {
        solution.first = KEY_Up;
    } else if (key == "right") {
        solution.first = KEY_Right;
    } else if (key == "down") {
        solution.first = KEY_Down;
    } else if (key == "prior" || key == "page_up") {
        solution.first = KEY_Page_Up;
    } else if (key == "next" || key == "page_down") {
        solution.first = KEY_Page_Down;
    } else if (key == "end") {
        solution.first = KEY_End;
    } else if (key == "begin") {
        solution.first = KEY_Begin;
        // Function keys
    } else if (key == "f1") {
        solution.first = KEY_F1;
    } else if (key == "f2") {
        solution.first = KEY_F2;
    } else if (key == "f3") {
        solution.first = KEY_F3;
    } else if (key == "f4") {
        solution.first = KEY_F4;
    } else if (key == "f5") {
        solution.first = KEY_F5;
    } else if (key == "f6") {
        solution.first = KEY_F6;
    } else if (key == "f7") {
        solution.first = KEY_F7;
    } else if (key == "f8") {
        solution.first = KEY_F8;
    } else if (key == "f9") {
        solution.first = KEY_F9;
    } else if (key == "f10") {
        solution.first = KEY_F10;
    } else if (key == "f11" || key == "l1") {
        solution.first = KEY_F11;
    } else if (key == "f12" || key == "l2") {
        solution.first = KEY_F12;
    } else {
        writeError("translateKey", 0, "<key>");
        solution.first = KEY_VoidSymbol;
    }
    return solution;
}


template <> std::pair<FXint, FXString>
InternalTestStep::translateKey(const char* key) const {
    return translateKey(std::string(key));
}


template <> std::pair<FXint, FXString>
InternalTestStep::translateKey(const char key) const {
    std::pair<FXint, FXString> solution;
    solution.first = FXint(key);
    solution.second.append(key);
    return solution;
}


void
InternalTestStep::buildMouseClick(const InternalTest::ViewPosition& viewPosition,
                                  const int offsetX, const int offsetY,
                                  const std::string& button,
                                  const std::string& keyModifier) const {
    // move mouse
    new InternalTestStep(myTestSystem, SEL_MOTION, Category::VIEW,
                         buildMouseMoveEvent(viewPosition, offsetX, offsetY), true);
    // continue depending of mouse
    if (button == "left") {
        new InternalTestStep(myTestSystem, SEL_LEFTBUTTONPRESS, Category::VIEW,
                             buildMouseEvent(SEL_LEFTBUTTONPRESS, viewPosition, offsetX, offsetY, keyModifier),
                             true);
        new InternalTestStep(myTestSystem, SEL_LEFTBUTTONRELEASE, Category::VIEW,
                             buildMouseEvent(SEL_LEFTBUTTONRELEASE, viewPosition, offsetX, offsetY, keyModifier),
                             true);
    } else if (button == "right") {
        new InternalTestStep(myTestSystem, SEL_RIGHTBUTTONPRESS, Category::VIEW,
                             buildMouseEvent(SEL_RIGHTBUTTONPRESS, viewPosition, offsetX, offsetY, keyModifier),
                             true);
        new InternalTestStep(myTestSystem, SEL_RIGHTBUTTONRELEASE, Category::VIEW,
                             buildMouseEvent(SEL_RIGHTBUTTONRELEASE, viewPosition, offsetX, offsetY, keyModifier),
                             true);
    } else if (button == "center") {
        new InternalTestStep(myTestSystem, SEL_MIDDLEBUTTONPRESS, Category::VIEW,
                             buildMouseEvent(SEL_MIDDLEBUTTONPRESS, viewPosition, offsetX, offsetY, keyModifier),
                             true);
        new InternalTestStep(myTestSystem, SEL_MIDDLEBUTTONRELEASE, Category::VIEW,
                             buildMouseEvent(SEL_MIDDLEBUTTONRELEASE, viewPosition, offsetX, offsetY, keyModifier),
                             true);
    }
}


FXEvent*
InternalTestStep::buildMouseMoveEvent(const InternalTest::ViewPosition& viewPosition,
                                      const int offsetX, const int offsetY) const {
    FXEvent* moveEvent = new FXEvent();
    // common values
    moveEvent->synthetic = true;
    // set event values
    moveEvent->type = SEL_MOTION;
    moveEvent->win_x = viewPosition.x + MOUSE_OFFSET_X + offsetX;
    moveEvent->win_y = viewPosition.y + MOUSE_OFFSET_Y + offsetY;
    moveEvent->moved = true;
    moveEvent->rect = FXRectangle(0, 0, 0, 0);
    return moveEvent;
}


FXEvent*
InternalTestStep::buildMouseEvent(FXSelType type, const InternalTest::ViewPosition& viewPosition,
                                  const int offsetX, const int offsetY, const std::string& keyModifier) const {
    FXEvent* leftClickPressEvent = new FXEvent();
    // common values
    leftClickPressEvent->synthetic = true;
    // set event values
    leftClickPressEvent->win_x = viewPosition.x + MOUSE_OFFSET_X + offsetX;
    leftClickPressEvent->win_y = viewPosition.y + MOUSE_OFFSET_Y + offsetY;
    leftClickPressEvent->click_x = viewPosition.x + MOUSE_OFFSET_X + offsetX;
    leftClickPressEvent->click_y = viewPosition.y + MOUSE_OFFSET_Y + offsetY;
    leftClickPressEvent->type = type;
    leftClickPressEvent->code = 1;
    leftClickPressEvent->click_button = 1;
    leftClickPressEvent->click_count = 1;
    leftClickPressEvent->moved = false;
    // set modifier
    if (keyModifier == "control") {
        leftClickPressEvent->state = CONTROLMASK;
    } else if (keyModifier == "shift") {
        leftClickPressEvent->state = SHIFTMASK;
    } else {
        leftClickPressEvent->state = 256;
    }
    return leftClickPressEvent;
}


void
InternalTestStep::writeClickInfo(const InternalTest::ViewPosition& viewPosition,
                                 const int offsetX, const int offsetY,
                                 const std::string modifier) const {
    if (modifier == "control") {
        std::cout << "TestFunctions: Clicked with Control key pressed over position " <<
                  toString(viewPosition.x + MOUSE_REFERENCE_X + offsetX) << " - " <<
                  toString(viewPosition.y + MOUSE_REFERENCE_Y + offsetY) << std::endl;
    } else if (modifier == "shift") {
        std::cout << "TestFunctions: Clicked with Shift key pressed over position " <<
                  toString(viewPosition.x + MOUSE_REFERENCE_X + offsetX) << " - " <<
                  toString(viewPosition.y + MOUSE_REFERENCE_Y) << std::endl;
    } else {
        std::cout << "TestFunctions: Clicked over position " <<
                  toString(viewPosition.x + MOUSE_REFERENCE_X + offsetX) << " - " <<
                  toString(viewPosition.y + MOUSE_REFERENCE_Y + offsetY) << std::endl;
    }
}

/****************************************************************************/
