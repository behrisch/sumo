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
/// @file    GNETLSTable.cpp
/// @author  Pablo Alvarez Lopez
/// @date    2022-07-21
///
// Table used in GNETLSFrame for editing TLS programs
/****************************************************************************/

#include <netedit/GNEApplicationWindow.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNEViewParent.h>
#include <netedit/elements/GNEAttributeCarrier.h>
#include <utils/foxtools/MFXLabelTooltip.h>
#include <utils/foxtools/MFXMenuButtonTooltip.h>
#include <utils/foxtools/MFXTextFieldTooltip.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/gui/images/GUIIconSubSys.h>
#include <utils/gui/windows/GUIAppEnum.h>
#include <utils/tests/InternalTestStep.h>

#include "GNETLSTable.h"

// ===========================================================================
// Defines
// ===========================================================================

#define EXTRAMARGIN 1
#define DEFAULTWIDTH 190

// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNETLSTable) GNETLSTableMap[] = {
    FXMAPFUNC(MID_MBTTIP_FOCUS,     0,                                  GNETLSTable::onFocusRow),
    FXMAPFUNC(MID_MBTTIP_SELECTED,  0,                                  GNETLSTable::onCmdAddPhasePressed),
    // text fields
    FXMAPFUNC(SEL_FOCUSIN,  MID_GNE_TLSTABLE_TEXTFIELD,                 GNETLSTable::onFocusRow),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSTABLE_TEXTFIELD,                 GNETLSTable::onCmdEditRow),
    FXMAPFUNC(SEL_KEYPRESS, MID_GNE_TLSTABLE_TEXTFIELD,                 GNETLSTable::onCmdKeyPress),
    // add phase buttons
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSTABLE_ADDPHASE,                  GNETLSTable::onCmdAddPhase),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSTABLE_COPYPHASE,                 GNETLSTable::onCmdDuplicatePhase),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSTABLE_ADDPHASEALLRED,            GNETLSTable::onCmdAddPhaseAllRed),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSTABLE_ADDPHASEALLYELLOW,         GNETLSTable::onCmdAddPhaseAllYellow),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSTABLE_ADDPHASEALLGREEN,          GNETLSTable::onCmdAddPhaseAllGreen),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSTABLE_ADDPHASEALLGREENPRIORITY,  GNETLSTable::onCmdAddPhaseAllGreenPriority),
    // remove phase button
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSTABLE_REMOVEPHASE,               GNETLSTable::onCmdRemovePhase),
    FXMAPFUNC(SEL_KEYPRESS, MID_GNE_TLSTABLE_TEXTFIELD,                 GNETLSTable::onCmdKeyPress),
    // move up phase button
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSTABLE_MOVEUPPHASE,               GNETLSTable::onCmdMoveUpPhase),
    FXMAPFUNC(SEL_KEYPRESS, MID_GNE_TLSTABLE_TEXTFIELD,                 GNETLSTable::onCmdKeyPress),
    // move down phase button
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_TLSTABLE_MOVEDOWNPHASE,             GNETLSTable::onCmdMoveDownPhase),
    FXMAPFUNC(SEL_KEYPRESS, MID_GNE_TLSTABLE_TEXTFIELD,                 GNETLSTable::onCmdKeyPress),
};

// Object implementation
FXIMPLEMENT(GNETLSTable, FXHorizontalFrame, GNETLSTableMap, ARRAYNUMBER(GNETLSTableMap))

// ===========================================================================
// method definitions
// ===========================================================================

// ---------------------------------------------------------------------------
// GNETLSTable - public methods
// ---------------------------------------------------------------------------

GNETLSTable::GNETLSTable(GNETLSEditorFrame::TLSPhases* TLSPhasesParent) :
    FXHorizontalFrame(TLSPhasesParent->getCollapsableFrame(), GUIDesignAuxiliarFrameFixWidth),
    myProgramFont(new FXFont(getApp(), "Courier New", 10)),
    myIndexFont(new FXFont(getApp(), "Segoe UI", 9)),
    myIndexSelectedFont(new FXFont(getApp(), "Segoe UI", 9, FXFont::Bold)),
    myTLSPhasesParent(TLSPhasesParent) {
    // set default width
    recalcTableWidth();
}


GNETLSTable::~GNETLSTable() {
    // delete fonts
    delete myProgramFont;
    delete myIndexFont;
    delete myIndexSelectedFont;
}


void
GNETLSTable::enable() {
    // enable all cells
    for (const auto& row : myRows) {
        for (const auto& cell : row->getCells()) {
            cell->enable();
        }
    }
    // enable horizontal frame
    FXHorizontalFrame::enable();
}


void
GNETLSTable::disable() {
    // disable all cells
    for (const auto& row : myRows) {
        for (const auto& cell : row->getCells()) {
            cell->disable();
        }
    }
    // disable horizontal frame
    FXHorizontalFrame::disable();
}


GNETLSEditorFrame::TLSPhases*
GNETLSTable::getTLSPhasesParent() const {
    return myTLSPhasesParent;
}


void
GNETLSTable::recalcTableWidth() {
    // get minimum width of all elements
    int minimumTableWidth = 0;
    // get pointer to name column
    Column* nameColumn = nullptr;
    // iterate over all columns
    for (const auto& column : myColumns) {
        // check if this is the name column
        if (column->getType() == 'm') {
            // save column
            nameColumn = column;
        } else {
            // get minimum column width
            const auto  minimunColWidth = column->getColumnMinimumWidth();
            // set columnwidth
            column->setColumnWidth(minimunColWidth);
            // update minimum table width
            minimumTableWidth += minimunColWidth;
        }
    }
    // adjust name column
    if (nameColumn) {
        // get column name width
        const int minimumColNameWidth = nameColumn->getColumnMinimumWidth();
        // get scrollBar width
        const int scrollBarWidth = myTLSPhasesParent->getTLSEditorParent()->getScrollBarWidth();
        // get frame area width - padding (30, constant, 15 left, 15 right)
        const auto frameAreaWidth = myTLSPhasesParent->getTLSEditorParent()->getViewNet()->getViewParent()->getFrameAreaWidth() - 30;
        // continue depending of minimum table width
        if ((frameAreaWidth - (minimumTableWidth + minimumColNameWidth + scrollBarWidth)) > 0) {
            nameColumn->setColumnWidth(frameAreaWidth - minimumTableWidth - scrollBarWidth);
            setWidth(frameAreaWidth);
        } else {
            nameColumn->setColumnWidth(minimumColNameWidth);
            setWidth(minimumTableWidth + minimumColNameWidth);
        }
    } else if (minimumTableWidth > 0) {
        setWidth(minimumTableWidth);
    } else {
        setWidth(DEFAULTWIDTH);
    }
}


void
GNETLSTable::clearTable() {
    // clear rows (always before columns, because delete row delete also all cells)
    for (const auto& row : myRows) {
        delete row;
    }
    // clear columns
    for (const auto& column : myColumns) {
        delete column;
    }
    // drop rows and columns
    myRows.clear();
    myColumns.clear();
}


void
GNETLSTable::setTableSize(const std::string& columnsType, const int numberRow) {
    // first clear table
    clearTable();
    // create columns
    for (int i = 0; i < (FXint)columnsType.size(); i++) {
        myColumns.push_back(new Column(this, i, columnsType.at(i)));
    }
    // create rows
    for (int i = 0; i < numberRow; i++) {
        myRows.push_back(new Row(this));
    }
    // if we have only a row, disable remove and move buttons
    if (myRows.size() == 1) {
        myRows.front()->disableButtons();
    }
}


void
GNETLSTable::setItemText(FXint row, FXint column, const std::string& text) {
    if ((row >= 0) && (row < (FXint)myRows.size()) &&
            (column >= 0) && (column < (FXint)myColumns.size())) {
        myRows.at(row)->setText(column, text);
        // check if update accumulated duration
        if (myColumns.at(column)->getType() == 'u') {
            updateAccumulatedDuration();
        }
    } else {
        throw ProcessError(TL("Invalid row or column"));
    }
}


std::string
GNETLSTable::getItemText(const int row, const int column) const {
    if ((row >= 0) && (row < (FXint)myRows.size()) &&
            (column >= 0) && (column < (FXint)myColumns.size())) {
        return myRows.at(row)->getText(column);
    }
    throw ProcessError(TL("Invalid row or column"));
}


int
GNETLSTable::getNumRows() const {
    return (int)myRows.size();
}


int
GNETLSTable::getCurrentSelectedRow() const {
    return myCurrentSelectedRow;
}


void
GNETLSTable::selectRow(const int row) {
    if ((row >= 0) && (row < (FXint)myRows.size())) {
        // update current selected row
        myCurrentSelectedRow = row;
        // update index label
        updateIndexLabel();
    } else {
        throw ProcessError(TL("Invalid row"));
    }
}


void
GNETLSTable::setColumnLabelTop(const int column, const std::string& text, const std::string& tooltip) {
    if ((column >= 0) && (column < (int)myColumns.size())) {
        myColumns.at(column)->setColumnLabelTop(text, tooltip);
    } else {
        throw ProcessError(TL("Invalid column"));
    }
}


void
GNETLSTable::setColumnLabelBot(const int column, const std::string& text) {
    if ((column >= 0) && (column < (int)myColumns.size())) {
        myColumns.at(column)->setColumnLabelBot(text);
    } else {
        throw ProcessError(TL("Invalid column"));
    }
}


long
GNETLSTable::onInternalTest(FXObject*, FXSelector, void* ptr) {
    // parse table test
    InternalTestStep::TLSTableTest* tableTest = static_cast<InternalTestStep::TLSTableTest*>(ptr);
    // obtain cell
    if (tableTest->row > (int)myRows.size()) {
        throw ProcessError(TL("Invalid row in table test"));
    } else if (tableTest->column > (int)myColumns.size()) {
        throw ProcessError(TL("Invalid column in table test"));
    } else {
        // get cell
        Cell* cell = myRows.at(tableTest->row)->getCells().at(tableTest->column);
        // continue depending of operation
        if (tableTest->sel == MID_GNE_TLSTABLE_ADDPHASE) {
            return onCmdAddPhase(cell->getAddPhaseButton(), 0, nullptr);
        } else if (tableTest->sel == MID_GNE_TLSTABLE_COPYPHASE) {
            return onCmdDuplicatePhase(cell->getDuplicatePhaseButton(), 0, nullptr);
        } else if (tableTest->sel == MID_GNE_TLSTABLE_ADDPHASEALLRED) {
            return onCmdAddPhaseAllRed(cell->getAddAllRedPhaseButton(), 0, nullptr);
        } else if (tableTest->sel == MID_GNE_TLSTABLE_ADDPHASEALLYELLOW) {
            return onCmdAddPhaseAllYellow(cell->getAddAllYellowPhaseButton(), 0, nullptr);
        } else if (tableTest->sel == MID_GNE_TLSTABLE_ADDPHASEALLGREEN) {
            return onCmdAddPhaseAllGreen(cell->getAddAllGreenPhaseButton(), 0, nullptr);
        } else if (tableTest->sel == MID_GNE_TLSTABLE_ADDPHASEALLGREENPRIORITY) {
            return onCmdAddPhaseAllGreenPriority(cell->getAddAllGreenPriorityPhaseButton(), 0, nullptr);
        } else if (tableTest->sel == MID_GNE_TLSTABLE_REMOVEPHASE) {
            return onCmdRemovePhase(cell->getButton(), 0, nullptr);
        } else if (tableTest->sel == MID_GNE_TLSTABLE_MOVEUPPHASE) {
            return onCmdMoveUpPhase(cell->getButton(), 0, nullptr);
        } else if (tableTest->sel == MID_GNE_TLSTABLE_MOVEDOWNPHASE) {
            return onCmdMoveDownPhase(cell->getButton(), 0, nullptr);
        } else if (tableTest->sel == MID_GNE_TLSTABLE_TEXTFIELD) {
            // set text in text field
            cell->getTextField()->setText(tableTest->text.c_str(), TRUE);
            return 1;
        } else {
            // unknown operation
            throw ProcessError(TL("Unknown operation in table test"));
        }
    }
}


long
GNETLSTable::onFocusRow(FXObject* sender, FXSelector, void*) {
    int selectedRow = -1;
    // search selected text field
    for (int rowIndex = 0; rowIndex < (int)myRows.size(); rowIndex++) {
        // iterate over every cell
        for (const auto& cell : myRows.at(rowIndex)->getCells()) {
            if ((cell->getTextField() == sender) || (cell->getAddButton() == sender)) {
                selectedRow = rowIndex;
            }
        }
    }
    // update index label
    updateIndexLabel();
    // set new row
    if (myCurrentSelectedRow != selectedRow) {
        myCurrentSelectedRow = selectedRow;
        updateIndexLabel();
    }
    return 0;
}


long
GNETLSTable::onCmdAddPhasePressed(FXObject* sender, FXSelector, void*) {
    // search selected add button
    for (int columnIndex = 0; columnIndex < (int)myColumns.size(); columnIndex++) {
        for (int rowIndex = 0; rowIndex < (int)myRows.size(); rowIndex++) {
            if (myRows.at(rowIndex)->getCells().at(columnIndex)->getAddButton() == sender) {
                myRows.at(rowIndex)->getCells().at(columnIndex)->getAddPhaseButton()->setFocus();
                return 1;
            }
        }
    }
    // nothing to focus
    return 0;
}


long
GNETLSTable::onCmdEditRow(FXObject* sender, FXSelector, void*) {
    // search selected text field
    for (int columnIndex = 0; columnIndex < (int)myColumns.size(); columnIndex++) {
        for (int rowIndex = 0; rowIndex < (int)myRows.size(); rowIndex++) {
            // get text field
            const auto textField = myRows.at(rowIndex)->getCells().at(columnIndex)->getTextField();
            if (textField == sender) {
                // edit value and change value depending of result
                if (myTLSPhasesParent->changePhaseValue(columnIndex, rowIndex, textField->getText().text())) {
                    textField->setTextColor(FXRGB(0, 0, 0));
                    textField->killFocus();
                    myTLSPhasesParent->getTLSEditorParent()->update();
                } else {
                    textField->setTextColor(FXRGB(255, 0, 0));
                }
                return 1;
            }
        }
    }
    // nothing to edit
    return 0;
}


long
GNETLSTable::onCmdKeyPress(FXObject* sender, FXSelector sel, void* ptr) {
    // get FXEvent
    FXEvent* eventInfo = (FXEvent*)ptr;
    // check code
    if (eventInfo->code == 65362) {
        // move up
        if (myCurrentSelectedRow > 0) {
            myCurrentSelectedRow -= 1;
        } else {
            // we're in the first, then select last
            myCurrentSelectedRow = ((int)myRows.size() - 1);
        }
        // update index label
        updateIndexLabel();
        // move focus
        moveFocus();
        return 1;
    } else if (eventInfo->code == 65364) {
        // move down
        if (myCurrentSelectedRow < ((int)myRows.size() - 1)) {
            myCurrentSelectedRow += 1;
        } else {
            // we're in the last, then select first
            myCurrentSelectedRow = 0;
        }
        // update index label
        updateIndexLabel();
        // move focus
        moveFocus();
        return 1;
    } else {
        // continue handling key pres
        return sender->handle(sender, sel, ptr);
    }
}


long
GNETLSTable::onCmdAddPhase(FXObject* sender, FXSelector, void*) {
    // search selected text field
    for (int indexRow = 0; indexRow < (int)myRows.size(); indexRow++) {
        // iterate over every cell
        for (const auto& cell : myRows.at(indexRow)->getCells()) {
            if (cell->getAddPhaseButton() == sender) {
                // hide popup
                cell->hideMenuButtonPopup();
                // add row
                myTLSPhasesParent->addPhase(indexRow);
                // stop
                return 0;
            }
        }
    }
    return 0;
}


long
GNETLSTable::onCmdDuplicatePhase(FXObject* sender, FXSelector, void*) {
    // search selected text field
    for (int indexRow = 0; indexRow < (int)myRows.size(); indexRow++) {
        // iterate over every cell
        for (const auto& cell : myRows.at(indexRow)->getCells()) {
            if (cell->getDuplicatePhaseButton() == sender) {
                // hide popup
                cell->hideMenuButtonPopup();
                // duplicate row
                myTLSPhasesParent->duplicatePhase(indexRow);
                // stop
                return 0;
            }
        }
    }
    return 0;
}


long
GNETLSTable::onCmdAddPhaseAllRed(FXObject* sender, FXSelector, void*) {
    // search selected text field
    for (int indexRow = 0; indexRow < (int)myRows.size(); indexRow++) {
        // iterate over every cell
        for (const auto& cell : myRows.at(indexRow)->getCells()) {
            if (cell->getAddAllRedPhaseButton() == sender) {
                // hide popup
                cell->hideMenuButtonPopup();
                // add row
                myTLSPhasesParent->addPhase(indexRow, 'r');
                // stop
                return 0;
            }
        }
    }
    return 0;
}


long
GNETLSTable::onCmdAddPhaseAllYellow(FXObject* sender, FXSelector, void*) {
    // search selected text field
    for (int indexRow = 0; indexRow < (int)myRows.size(); indexRow++) {
        // iterate over every cell
        for (const auto& cell : myRows.at(indexRow)->getCells()) {
            if (cell->getAddAllYellowPhaseButton() == sender) {
                // hide popup
                cell->hideMenuButtonPopup();
                // add row
                myTLSPhasesParent->addPhase(indexRow, 'y');
                // stop
                return 0;
            }
        }
    }
    return 0;
}


long
GNETLSTable::onCmdAddPhaseAllGreen(FXObject* sender, FXSelector, void*) {
    // search selected text field
    for (int indexRow = 0; indexRow < (int)myRows.size(); indexRow++) {
        // iterate over every cell
        for (const auto& cell : myRows.at(indexRow)->getCells()) {
            if (cell->getAddAllGreenPhaseButton() == sender) {
                // hide popup
                cell->hideMenuButtonPopup();
                // add row
                myTLSPhasesParent->addPhase(indexRow, 'g');
                // stop
                return 0;
            }
        }
    }
    return 0;
}


long
GNETLSTable::onCmdAddPhaseAllGreenPriority(FXObject* sender, FXSelector, void*) {
    // search selected text field
    for (int indexRow = 0; indexRow < (int)myRows.size(); indexRow++) {
        // iterate over every cell
        for (const auto& cell : myRows.at(indexRow)->getCells()) {
            if (cell->getAddAllGreenPriorityPhaseButton() == sender) {
                // hide popup
                cell->hideMenuButtonPopup();
                // add row
                myTLSPhasesParent->addPhase(indexRow, 'G');
                // stop
                return 0;
            }
        }
    }
    return 0;
}


long
GNETLSTable::onCmdRemovePhase(FXObject* sender, FXSelector, void*) {
    // search selected text field
    for (int indexRow = 0; indexRow < (int)myRows.size(); indexRow++) {
        // iterate over every cell
        for (const auto& cell : myRows.at(indexRow)->getCells()) {
            if (cell->getButton() == sender) {
                // remove row
                myTLSPhasesParent->removePhase(indexRow);
                // stop
                return 0;
            }
        }
    }
    return 0;
}


long
GNETLSTable::onCmdMoveUpPhase(FXObject* sender, FXSelector, void*) {
    // search selected text field
    for (int indexRow = 0; indexRow < (int)myRows.size(); indexRow++) {
        // iterate over every cell
        for (const auto& cell : myRows.at(indexRow)->getCells()) {
            if (cell->getButton() == sender) {
                // move phase up
                myTLSPhasesParent->movePhaseUp(indexRow);
                // stop
                return 0;
            }
        }
    }
    return 0;
}


long
GNETLSTable::onCmdMoveDownPhase(FXObject* sender, FXSelector, void*) {
    // search selected text field
    for (int indexRow = 0; indexRow < (int)myRows.size(); indexRow++) {
        // iterate over every cell
        for (const auto& cell : myRows.at(indexRow)->getCells()) {
            if (cell->getButton() == sender) {
                // move phase down
                myTLSPhasesParent->movePhaseDown(indexRow);
                // stop
                return 0;
            }
        }
    }
    return 0;
}


void
GNETLSTable::updateIndexLabel() {
    // update radio buttons checks
    for (int rowIndex = 0; rowIndex < (int)myRows.size(); rowIndex++) {
        // iterate over every cell
        for (const auto& cell : myRows.at(rowIndex)->getCells()) {
            if (cell->getIndexLabel()) {
                if (myCurrentSelectedRow == rowIndex) {
                    cell->showIndexLabelBold();
                } else {
                    cell->showIndexLabelNormal();
                }
            }
        }
    }
    // update coloring
    myTLSPhasesParent->updateTLSColoring();
}


void
GNETLSTable::updateAccumulatedDuration() {
    // first find the duration col
    int durationCol = -1;
    for (int i = 0; i < (int)myColumns.size(); i++) {
        if (myColumns.at(i)->getType() == 'u') {
            durationCol = i;
        }
    }
    // continue depending of durationCol
    if (durationCol != -1) {
        // declare a int vector for saving durations
        std::vector<double> durations;
        // fill durations
        for (const auto& row : myRows) {
            durations.push_back(row->getCells().at(durationCol)->getDoubleValue());
        }
        // update durations
        for (int i = 1; i < (int)durations.size(); i++) {
            durations.at(i) += durations.at(i - 1);
        }
        // set tooltips in row cells
        for (int i = 0; i < (int)myRows.size(); i++) {
            myRows.at(i)->getCells().at(durationCol)->setTooltip(TL("Accumulated: ") + toString(durations.at(i)));
        }
    }
}


bool
GNETLSTable::moveFocus() {
    // first find focus
    // update radio buttons checks
    for (int rowIndex = 0; rowIndex < (int)myRows.size(); rowIndex++) {
        for (int cellIndex = 0; cellIndex < (int)myRows.at(rowIndex)->getCells().size(); cellIndex++) {
            if (myRows.at(rowIndex)->getCells().at(cellIndex)->hasFocus()) {
                // set focus in current row
                myRows.at(myCurrentSelectedRow)->getCells().at(cellIndex)->setFocus();
                return true;
            }
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// GNETLSTable::Cell - methods
// ---------------------------------------------------------------------------

GNETLSTable::Cell::Cell(GNETLSTable* TLSTable, MFXTextFieldTooltip* textField, int col, int row) :
    myTLSTable(TLSTable),
    myTextField(textField),
    myCol(col),
    myRow(row) {
    // create
    textField->create();
}


GNETLSTable::Cell::Cell(GNETLSTable* TLSTable, FXLabel* indexLabel, FXLabel* indexLabelBold, int col, int row) :
    myTLSTable(TLSTable),
    myIndexLabel(indexLabel),
    myIndexLabelBold(indexLabelBold),
    myCol(col),
    myRow(row) {
    // create both
    indexLabel->create();
    indexLabelBold->create();
    // hide bold and set background
    indexLabelBold->hide();
    indexLabelBold->setBackColor(FXRGBA(210, 233, 255, 255));
}


GNETLSTable::Cell::Cell(GNETLSTable* TLSTable, MFXButtonTooltip* button, int col, int row) :
    myTLSTable(TLSTable),
    myButton(button),
    myCol(col),
    myRow(row) {
    // create
    button->create();
}


GNETLSTable::Cell::Cell(GNETLSTable* TLSTable, int col, int row) :
    myTLSTable(TLSTable),
    myCol(col),
    myRow(row) {
    // build locator popup
    myMenuButtonPopup = new FXPopup(TLSTable->myColumns.at(col)->getVerticalCellFrame(), POPUP_HORIZONTAL);
    // build menu button
    myAddButton = new MFXMenuButtonTooltip(TLSTable->myColumns.at(col)->getVerticalCellFrame(),
                                           myTLSTable->getTLSPhasesParent()->getTLSEditorParent()->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
                                           (std::string("\t") + TL("Add phase") + std::string("\t") + TL("Add new phase.")).c_str(),
                                           GUIIconSubSys::getIcon(GUIIcon::ADD), myMenuButtonPopup, TLSTable, GUIDesignTLSTableCheckableButtonIcon);
    // default phase
    myAddPhaseButton = new MFXButtonTooltip(myMenuButtonPopup,
                                            myTLSTable->getTLSPhasesParent()->getTLSEditorParent()->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
                                            (std::string("\t") + TL("Default phase") + std::string("\t") + TL("Add default phase.")).c_str(),
                                            GUIIconSubSys::getIcon(GUIIcon::TLSPHASEDEFAULT), TLSTable, MID_GNE_TLSTABLE_ADDPHASE, GUIDesignButtonIcon);
    // duplicate phase
    myDuplicatePhaseButton = new MFXButtonTooltip(myMenuButtonPopup,
            myTLSTable->getTLSPhasesParent()->getTLSEditorParent()->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
            (std::string("\t") + TL("Duplicate phase") + std::string("\t") + TL("Duplicate this phase.")).c_str(),
            GUIIconSubSys::getIcon(GUIIcon::TLSPHASECOPY), TLSTable, MID_GNE_TLSTABLE_COPYPHASE, GUIDesignButtonIcon);
    // red phase
    myAddAllRedButton = new MFXButtonTooltip(myMenuButtonPopup,
            myTLSTable->getTLSPhasesParent()->getTLSEditorParent()->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
            (std::string("\t") + TL("Red phase") + std::string("\t") + TL("Add red phase.")).c_str(),
            GUIIconSubSys::getIcon(GUIIcon::TLSPHASEALLRED), TLSTable, MID_GNE_TLSTABLE_ADDPHASEALLRED, GUIDesignButtonIcon);
    // yellow phase
    myAddAllYellowButton = new MFXButtonTooltip(myMenuButtonPopup,
            myTLSTable->getTLSPhasesParent()->getTLSEditorParent()->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
            (std::string("\t") + TL("Yellow phase") + std::string("\t") + TL("Add yellow phase.")).c_str(),
            GUIIconSubSys::getIcon(GUIIcon::TLSPHASEALLYELLOW), TLSTable, MID_GNE_TLSTABLE_ADDPHASEALLYELLOW, GUIDesignButtonIcon);
    // green phase
    myAddAllGreenButton = new MFXButtonTooltip(myMenuButtonPopup,
            myTLSTable->getTLSPhasesParent()->getTLSEditorParent()->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
            (std::string("\t") + TL("Green phase") + std::string("\t") + TL("Add green phase.")).c_str(),
            GUIIconSubSys::getIcon(GUIIcon::TLSPHASEALLGREEN), TLSTable, MID_GNE_TLSTABLE_ADDPHASEALLGREEN, GUIDesignButtonIcon);
    // green priority phase
    myAddAllGreenPriorityButton = new MFXButtonTooltip(myMenuButtonPopup,
            myTLSTable->getTLSPhasesParent()->getTLSEditorParent()->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
            (std::string("\t") + TL("Green priority phase") + std::string("\t") + TL("Add green priority phase.")).c_str(),
            GUIIconSubSys::getIcon(GUIIcon::TLSPHASEALLGREENPRIORITY), TLSTable, MID_GNE_TLSTABLE_ADDPHASEALLGREENPRIORITY, GUIDesignButtonIcon);
    // create elements
    myMenuButtonPopup->create();
    myAddButton->create();
    myAddPhaseButton->create();
    myDuplicatePhaseButton->create();
    myAddAllRedButton->create();
    myAddAllYellowButton->create();
    myAddAllGreenButton->create();
    myAddAllGreenPriorityButton->create();
    // set backgrounds
    myAddPhaseButton->setBackColor(FXRGBA(210, 233, 255, 255));
    myDuplicatePhaseButton->setBackColor(FXRGBA(210, 233, 255, 255));
    myAddAllRedButton->setBackColor(FXRGBA(255, 213, 213, 255));
    myAddAllYellowButton->setBackColor(FXRGBA(253, 255, 206, 255));
    myAddAllGreenButton->setBackColor(FXRGBA(240, 255, 205, 255));
    myAddAllGreenPriorityButton->setBackColor(FXRGBA(240, 255, 205, 255));
}

GNETLSTable::Cell::~Cell() {
    // delete all elements
    if (myTextField) {
        delete myTextField;
    }
    if (myIndexLabel) {
        delete myIndexLabel;
    }
    if (myIndexLabelBold) {
        delete myIndexLabelBold;
    }
    if (myButton) {
        delete myButton;
    }
    if (myAddButton) {
        delete myAddButton;
    }
    if (myAddPhaseButton) {
        delete myAddPhaseButton;
    }
    if (myDuplicatePhaseButton) {
        delete myDuplicatePhaseButton;
    }
    if (myAddAllRedButton) {
        delete myAddAllRedButton;
    }
    if (myAddAllYellowButton) {
        delete myAddAllYellowButton;
    }
    if (myAddAllGreenButton) {
        delete myAddAllGreenButton;
    }
    if (myAddAllGreenPriorityButton) {
        delete myAddAllGreenPriorityButton;
    }
    if (myMenuButtonPopup) {
        delete myMenuButtonPopup;
    }
}

void
GNETLSTable::Cell::enable() {
    // enable all elements
    if (myTextField) {
        myTextField->enable();
    }
    if (myIndexLabel) {
        myIndexLabel->enable();
    }
    if (myIndexLabelBold) {
        myIndexLabelBold->enable();
    }
    if (myButton && !myDisableButton) {
        myButton->enable();
    }
    if (myAddButton) {
        myAddButton->enable();
    }
    if (myAddPhaseButton) {
        myAddPhaseButton->enable();
    }
    if (myDuplicatePhaseButton) {
        myDuplicatePhaseButton->enable();
    }
    if (myAddAllRedButton) {
        myAddAllRedButton->enable();
    }
    if (myAddAllYellowButton) {
        myAddAllYellowButton->enable();
    }
    if (myAddAllGreenButton) {
        myAddAllGreenButton->enable();
    }
    if (myAddAllGreenPriorityButton) {
        myAddAllGreenPriorityButton->enable();
    }
    if (myMenuButtonPopup) {
        myMenuButtonPopup->enable();
    }
}


void
GNETLSTable::Cell::disable() {
    // disable all elements
    if (myTextField) {
        myTextField->disable();
    }
    if (myIndexLabel) {
        myIndexLabel->disable();
    }
    if (myIndexLabelBold) {
        myIndexLabelBold->disable();
    }
    if (myButton && !myDisableButton) {
        myButton->disable();
    }
    if (myAddButton) {
        myAddButton->disable();
    }
    if (myAddPhaseButton) {
        myAddPhaseButton->disable();
    }
    if (myDuplicatePhaseButton) {
        myDuplicatePhaseButton->disable();
    }
    if (myAddAllRedButton) {
        myAddAllRedButton->disable();
    }
    if (myAddAllYellowButton) {
        myAddAllYellowButton->disable();
    }
    if (myAddAllGreenButton) {
        myAddAllGreenButton->disable();
    }
    if (myAddAllGreenPriorityButton) {
        myAddAllGreenPriorityButton->disable();
    }
    if (myMenuButtonPopup) {
        myMenuButtonPopup->disable();
    }
}


bool
GNETLSTable::Cell::hasFocus() const {
    // check if one of the cell elements has the focus
    if (myTextField && myTextField->hasFocus()) {
        return true;
    } else if (myButton && myButton->hasFocus()) {
        return true;
    } else if (myAddButton && myAddButton->hasFocus()) {
        return true;
    } else if (myAddPhaseButton && myAddPhaseButton->hasFocus()) {
        return true;
    } else if (myDuplicatePhaseButton && myDuplicatePhaseButton->hasFocus()) {
        return true;
    } else if (myAddAllRedButton && myAddAllRedButton->hasFocus()) {
        return true;
    } else if (myAddAllYellowButton && myAddAllYellowButton->hasFocus()) {
        return true;
    } else if (myAddAllGreenButton && myAddAllGreenButton->hasFocus()) {
        return true;
    } else if (myAddAllGreenPriorityButton && myAddAllGreenPriorityButton->hasFocus()) {
        return true;
    } else {
        return false;
    }
}


void
GNETLSTable::Cell::setFocus() {
    // set focus
    if (myTextField) {
        myTextField->setFocus();
    } else if (myButton) {
        myButton->setFocus();
    } else if (myAddButton) {
        myAddButton->setFocus();
    } else if (myAddPhaseButton) {
        myAddPhaseButton->setFocus();
    } else if (myDuplicatePhaseButton) {
        myDuplicatePhaseButton->setFocus();
    } else if (myAddAllRedButton) {
        myAddAllRedButton->setFocus();
    } else if (myAddAllYellowButton) {
        myAddAllYellowButton->setFocus();
    } else if (myAddAllGreenButton) {
        myAddAllGreenButton->setFocus();
    } else if (myAddAllGreenPriorityButton) {
        myAddAllGreenPriorityButton->setFocus();
    }
}


double
GNETLSTable::Cell::getDoubleValue() const {
    if (myTextField->getText().empty()) {
        return 0;
    } else if (!GNEAttributeCarrier::canParse<double>(myTextField->getText().text())) {
        throw ProcessError(TL("Cannot be parsed to double"));
    } else {
        return GNEAttributeCarrier::parse<double>(myTextField->getText().text());
    }
}


void
GNETLSTable::Cell::setTooltip(const std::string& toolTip) {
    if (myTextField) {
        myTextField->setToolTipText(toolTip.c_str());
    } else {
        throw ProcessError(TL("Tooltips only for TextFields"));
    }
}


MFXTextFieldTooltip*
GNETLSTable::Cell::getTextField() const {
    return myTextField;
}


FXLabel*
GNETLSTable::Cell::getIndexLabel() const {
    return myIndexLabel;
}


MFXMenuButtonTooltip*
GNETLSTable::Cell::getAddButton() const {
    return myAddButton;
}


MFXButtonTooltip*
GNETLSTable::Cell::getButton() {
    return myButton;
}


MFXButtonTooltip*
GNETLSTable::Cell::getAddPhaseButton() {
    return myAddPhaseButton;
}


MFXButtonTooltip*
GNETLSTable::Cell::getDuplicatePhaseButton() {
    return myDuplicatePhaseButton;
}


MFXButtonTooltip*
GNETLSTable::Cell::getAddAllRedPhaseButton() {
    return myAddAllRedButton;
}


MFXButtonTooltip*
GNETLSTable::Cell::getAddAllYellowPhaseButton() {
    return myAddAllYellowButton;
}


MFXButtonTooltip*
GNETLSTable::Cell::getAddAllGreenPhaseButton() {
    return myAddAllGreenButton;
}


MFXButtonTooltip*
GNETLSTable::Cell::getAddAllGreenPriorityPhaseButton() {
    return myAddAllGreenPriorityButton;
}


void
GNETLSTable::Cell::showIndexLabelNormal() {
    myIndexLabel->show();
    myIndexLabelBold->hide();
    // recalc both
    myIndexLabel->recalc();
    myIndexLabelBold->recalc();
}


void
GNETLSTable::Cell::showIndexLabelBold() {
    myIndexLabel->hide();
    myIndexLabelBold->show();
    // recalc both
    myIndexLabel->recalc();
    myIndexLabelBold->recalc();
}


int
GNETLSTable::Cell::getCol() const {
    return myCol;
}


int
GNETLSTable::Cell::getRow() const {
    return myRow;
}


char
GNETLSTable::Cell::getType() const {
    return myTLSTable->myColumns.at(myCol)->getType();
}


void
GNETLSTable::Cell::hideMenuButtonPopup() {
    if (myMenuButtonPopup) {
        myMenuButtonPopup->popdown();
    }
}


void
GNETLSTable::Cell::disableButton() {
    if (myButton) {
        myButton->disable();
        myDisableButton = true;
    }
}


GNETLSTable::Cell::Cell() :
    myCol(-1),
    myRow(-1) {
}

// ---------------------------------------------------------------------------
// GNETLSTable::Column - methods
// ---------------------------------------------------------------------------

GNETLSTable::Column::Column(GNETLSTable* table, const int index, const char type) :
    myTable(table),
    myIndex(index),
    myType(type) {
    // create vertical frame
    myVerticalFrame = new FXVerticalFrame(table, GUIDesignAuxiliarFrameFixWidth);
    // create top label
    switch (myType) {
        case 's':
        case 'i':
        case 'd':
        case 't':
        case 'b':
            // empty label
            myTopLabel = new MFXLabelTooltip(myVerticalFrame,
                                             table->getTLSPhasesParent()->getTLSEditorParent()->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
                                             "", nullptr, GUIDesignLabelFixed(0));
            break;
        default:
            // ticked label
            myTopLabel = new MFXLabelTooltip(myVerticalFrame,
                                             table->getTLSPhasesParent()->getTLSEditorParent()->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
                                             "", nullptr, GUIDesignLabelThickedFixed(0));
            break;
    }
    // create vertical frame for cells
    myVerticalCellFrame = new FXVerticalFrame(myVerticalFrame, GUIDesignAuxiliarFrameFixWidth);
    // create bot label
    switch (myType) {
        case 's':
            // label with icon
            myBotLabel = new FXLabel(myVerticalFrame, "", GUIIconSubSys::getIcon(GUIIcon::SUM), GUIDesignLabelThickedFixed(0));
            break;
        case 'u':
        case 'p':
            // ticked label
            myBotLabel = new FXLabel(myVerticalFrame, "", nullptr, GUIDesignLabelThickedFixed(0));
            break;
        default:
            // empty label
            myBotLabel = new FXLabel(myVerticalFrame, "", nullptr, GUIDesignLabelFixed(0));
            break;
    }
    // create elements
    myVerticalFrame->create();
    myTopLabel->create();
    myVerticalCellFrame->create();
    myBotLabel->create();
}


GNETLSTable::Column::~Column() {
    // delete vertical frame (this also delete all childrens)
    delete myVerticalFrame;
}


FXVerticalFrame*
GNETLSTable::Column::getVerticalCellFrame() const {
    return myVerticalCellFrame;
}


char
GNETLSTable::Column::getType() const {
    return myType;
}


FXString
GNETLSTable::Column::getColumnLabelTop() const {
    return myTopLabel->getText();
}


void
GNETLSTable::Column::setColumnLabelTop(const std::string& text, const std::string& tooltip) {
    myTopLabel->setText(text.c_str());
    myTopLabel->setTipText(tooltip.c_str());
}


void
GNETLSTable::Column::setColumnLabelBot(const std::string& text) {
    myBotLabel->setText(text.c_str());
}


int
GNETLSTable::Column::getColumnMinimumWidth() {
    // declare columnWidth
    int columnWidth = 0;
    // check column type
    if (myType == 's') {
        // set index column width
        columnWidth = 30;
    } else if (isTextFieldColumn()) {
        // calculate top label width
        columnWidth = myTopLabel->getFont()->getTextWidth(myTopLabel->getText().text(), myTopLabel->getText().length() + EXTRAMARGIN);
        // iterate over all textFields and check widths
        for (const auto& row : myTable->myRows) {
            // get text field
            const auto textField = row->getCells().at(myIndex)->getTextField();
            // get textField width
            const auto textFieldWidth = textField->getFont()->getTextWidth(textField->getText().text(), textField->getText().length() + EXTRAMARGIN);
            // compare widths
            if (textFieldWidth > columnWidth) {
                columnWidth = textFieldWidth;
            }
        }
        // calculate bot label width
        const auto botLabelWidth = myBotLabel->getFont()->getTextWidth(myBotLabel->getText().text(), myBotLabel->getText().length() + EXTRAMARGIN);
        if (botLabelWidth > columnWidth) {
            columnWidth = botLabelWidth;
        }
    } else {
        // is an index column, then return icon size
        columnWidth = GUIDesignHeight;
    }
    return columnWidth;
}


void
GNETLSTable::Column::setColumnWidth(const int colWidth) {
    // only adjust for textField columns
    if (isTextFieldColumn()) {
        for (const auto& row : myTable->myRows) {
            row->getCells().at(myIndex)->getTextField()->setWidth(colWidth);
        }
    }
    // adjust labels and vertical frames
    myVerticalFrame->setWidth(colWidth);
    myTopLabel->setWidth(colWidth);
    myVerticalCellFrame->setWidth(colWidth);
    myBotLabel->setWidth(colWidth);
}


bool
GNETLSTable::Column::isTextFieldColumn() const {
    return ((myType == 'u') || (myType == 'f') || (myType == 'p') || (myType == 'm') || (myType == '-'));
}


GNETLSTable::Column::Column() :
    myIndex(0),
    myType('-') {}

// ---------------------------------------------------------------------------
// GNETLSTable::Row - methods
// ---------------------------------------------------------------------------

GNETLSTable::Row::Row(GNETLSTable* table) :
    myTable(table) {
    // build textFields
    for (int columnIndex = 0; columnIndex < (FXint)table->myColumns.size(); columnIndex++) {
        // get number of cells
        const int numCells = (int)myCells.size();
        // continue depending of type
        switch (table->myColumns.at(columnIndex)->getType()) {
            case ('s'): {
                // create labels for index
                auto indexLabel = new FXLabel(table->myColumns.at(columnIndex)->getVerticalCellFrame(),
                                              toString(myTable->myRows.size()).c_str(), nullptr, GUIDesignLabelThickedFixed(30));
                auto indexLabelBold = new FXLabel(table->myColumns.at(columnIndex)->getVerticalCellFrame(),
                                                  toString(myTable->myRows.size()).c_str(), nullptr, GUIDesignLabelThickedFixed(30));
                // set fonts
                indexLabel->setFont(myTable->myIndexFont);
                indexLabelBold->setFont(myTable->myIndexSelectedFont);
                myCells.push_back(new Cell(table, indexLabel, indexLabelBold, columnIndex, numCells));
                break;
            }
            case ('u'):
            case ('f'):
            case ('m'):
            case ('-'): {
                // create textField for values
                auto textField = new MFXTextFieldTooltip(table->myColumns.at(columnIndex)->getVerticalCellFrame(),
                        table->getTLSPhasesParent()->getTLSEditorParent()->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
                        GUIDesignTextFieldNCol, table, MID_GNE_TLSTABLE_TEXTFIELD, GUIDesignTextFieldTLSTable);
                myCells.push_back(new Cell(table, textField, columnIndex, numCells));
                break;
            }
            case ('p'): {
                // create text field for program (state)
                auto textField = new MFXTextFieldTooltip(table->myColumns.at(columnIndex)->getVerticalCellFrame(),
                        table->getTLSPhasesParent()->getTLSEditorParent()->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
                        GUIDesignTextFieldNCol, table, MID_GNE_TLSTABLE_TEXTFIELD, GUIDesignTextFieldTLSTable);
                // set special font
                textField->setFont(myTable->myProgramFont);
                myCells.push_back(new Cell(table, textField, columnIndex, numCells));
                break;
            }
            case ('i'): {
                // create popup for adding new phases
                myCells.push_back(new Cell(table, columnIndex, numCells));
                break;
            }
            case ('d'): {
                // create button for delete phase
                auto button = new MFXButtonTooltip(table->myColumns.at(columnIndex)->getVerticalCellFrame(),
                                                   table->getTLSPhasesParent()->getTLSEditorParent()->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
                                                   (std::string("\t") + TL("Delete phase") + std::string("\t") + TL("Delete this phase.")).c_str(),
                                                   GUIIconSubSys::getIcon(GUIIcon::REMOVE), table, MID_GNE_TLSTABLE_REMOVEPHASE, GUIDesignButtonIcon);
                myCells.push_back(new Cell(table, button, columnIndex, numCells));
                break;
            }
            case ('t'): {
                // create button for move up phase
                auto button = new MFXButtonTooltip(table->myColumns.at(columnIndex)->getVerticalCellFrame(),
                                                   table->getTLSPhasesParent()->getTLSEditorParent()->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
                                                   (std::string("\t") + TL("Move phase up") + std::string("\t") + TL("Move this phase up.")).c_str(),
                                                   GUIIconSubSys::getIcon(GUIIcon::ARROW_UP), table, MID_GNE_TLSTABLE_MOVEUPPHASE, GUIDesignButtonIcon);
                myCells.push_back(new Cell(table, button, columnIndex, numCells));
                break;
            }
            case ('b'): {
                // create button for move down phase
                auto button = new MFXButtonTooltip(table->myColumns.at(columnIndex)->getVerticalCellFrame(),
                                                   table->getTLSPhasesParent()->getTLSEditorParent()->getViewNet()->getViewParent()->getGNEAppWindows()->getStaticTooltipMenu(),
                                                   (std::string("\t") + TL("Move phase down") + std::string("\t") + TL("Move this phase down.")).c_str(),
                                                   GUIIconSubSys::getIcon(GUIIcon::ARROW_DOWN), table, MID_GNE_TLSTABLE_MOVEDOWNPHASE, GUIDesignButtonIcon);
                myCells.push_back(new Cell(table, button, columnIndex, numCells));
                break;
            }
            default:
                throw ProcessError("Invalid Cell type");
        }
    }
}


GNETLSTable::Row::~Row() {
    // delete all cells
    for (const auto& cell : myCells) {
        delete cell;
    }
}


std::string
GNETLSTable::Row::getText(int index) const {
    if (myCells.at(index)->getTextField()) {
        return myCells.at(index)->getTextField()->getText().text();
    } else {
        throw ProcessError("Cell doesn't have a textField");
    }
}


void
GNETLSTable::Row::setText(int index, const std::string& text) const {
    // set text
    myCells.at(index)->getTextField()->setText(text.c_str());
}


const std::vector<GNETLSTable::Cell*>&
GNETLSTable::Row::getCells() const {
    return myCells;
}


void
GNETLSTable::Row::disableButtons() {
    // search move up button and disable it
    for (const auto& cell : myCells) {
        if ((cell->getType() == 'd') || (cell->getType() == 'b') || (cell->getType() == 't')) {
            cell->disableButton();
        }
    }
}


GNETLSTable::Row::Row() {}

/****************************************************************************/
