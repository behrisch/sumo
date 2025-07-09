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
/// @file    GNETLSTable.h
/// @author  Pablo Alvarez Lopez
/// @date    2022-07-21
///
// Table used in GNETLSFrame for editing TLS programs
/****************************************************************************/
#ifndef GNETLSTable_h
#define GNETLSTable_h
#include <config.h>

#include <vector>

#include <utils/common/UtilExceptions.h>
#include <netedit/frames/network/GNETLSEditorFrame.h>

// ===========================================================================
// class declaration
// ===========================================================================

class MFXTextFieldTooltip;
class MFXLabelTooltip;
class MFXMenuButtonTooltip;

// ===========================================================================
// class definitions
// ===========================================================================

class GNETLSTable : public FXHorizontalFrame {
    /// @brief fox declaration
    FXDECLARE(GNETLSTable)

public:
    /// @brief constructor (Exactly like the FXButton constructor)
    GNETLSTable(GNETLSEditorFrame::TLSPhases* TLSPhasesParent);

    /// @brief destructor (Called automatically)
    ~GNETLSTable();

    /// @brief Enable table
    void enable();

    /// @brief Disable table
    void disable();

    /// @frame get pointer to TLSEditorFrame phases parent
    GNETLSEditorFrame::TLSPhases* getTLSPhasesParent() const;

    /// @brief recalc width (call when all labels and contents are fill)
    void recalcTableWidth();

    /// @brief clear table
    void clearTable();

    /// @brief Modify cell text
    void setItemText(FXint row, FXint column, const std::string& text);

    /// @brief Return cell text
    std::string getItemText(const int row, const int column) const;

    /// @brief Get number of rows
    int getNumRows() const;

    /// @brief Get current selected row
    int getCurrentSelectedRow() const;

    /// @brief Select a row
    void selectRow(const int rowIndex);

    /// @brief Change column header text
    void setColumnLabelTop(const int column, const std::string& text, const std::string& tooltip = "");

    /// @brief Change column bottom text
    void setColumnLabelBot(const int column, const std::string& text);

    /**@brief Set the table size to nr rows and nc columns; all existing items will be removed. Format:
     * s -> select row (radio button)
     * u -> duration (float, textField)
     * f -> float (float, textField)
     * p -> program (rrGggy..., textField)
     * m -> name (automatic size, textField)
     * i -> insert phase (button)
     * d -> delete phase (button)
     * t -> move phase up (button)
     * b -> move phase down (button)
     * - -> general text (textField)
     */
    void setTableSize(const std::string& columnsType, const int numberRow);

    /// @name FOX callbacks
    /// @{
    /// @brief called in internal tests
    long onInternalTest(FXObject*, FXSelector, void* ptr);

    /// @brief called when a row is focused
    long onFocusRow(FXObject*, FXSelector, void*);

    /// @brief called when add phase button is selected
    long onCmdAddPhasePressed(FXObject*, FXSelector, void*);

    /// @brief called when a row is modified
    long onCmdEditRow(FXObject*, FXSelector, void*);

    /// @brief called when a key is pressed
    long onCmdKeyPress(FXObject*, FXSelector, void*);

    /// @brief called when an add phase button is pressed
    long onCmdAddPhase(FXObject*, FXSelector, void*);

    /// @brief called when a duplicate phase button is pressed
    long onCmdDuplicatePhase(FXObject*, FXSelector, void*);

    /// @brief called when an add all green red phase button is pressed
    long onCmdAddPhaseAllRed(FXObject*, FXSelector, void*);

    /// @brief called when an add all green red phase button is pressed
    long onCmdAddPhaseAllYellow(FXObject*, FXSelector, void*);

    /// @brief called when an add all green red phase button is pressed
    long onCmdAddPhaseAllGreen(FXObject*, FXSelector, void*);

    /// @brief called when an add all green red phase button is pressed
    long onCmdAddPhaseAllGreenPriority(FXObject*, FXSelector, void*);

    /// @brief called when a remove phase button is pressed
    long onCmdRemovePhase(FXObject*, FXSelector, void*);

    /// @brief called when a move up phase button is pressed
    long onCmdMoveUpPhase(FXObject*, FXSelector, void*);

    /// @brief called when a move up phase button is pressed
    long onCmdMoveDownPhase(FXObject*, FXSelector, void*);

    /// @}

protected:
    /// @brief FOX needs this
    FOX_CONSTRUCTOR(GNETLSTable)

    /// @brief table cell
    class Cell {

    public:
        /// @brief constructor for textField
        Cell(GNETLSTable* TLSTable, MFXTextFieldTooltip* textField, int col, int row);

        /// @brief constructor for index label
        Cell(GNETLSTable* TLSTable, FXLabel* indexLabel, FXLabel* indexLabelBold, int col, int row);

        /// @brief constructor for buttons
        Cell(GNETLSTable* TLSTable, MFXButtonTooltip* button, int col, int row);

        /// @brief constructor for menu buttons
        Cell(GNETLSTable* TLSTable, int col, int row);

        /// @brief destructor
        ~Cell();

        /// @brief Enable cell
        void enable();

        /// @brief Disable cell
        void disable();

        /// @brief check if current cell has focus
        bool hasFocus() const;

        /// @brief set focus in the current cell
        void setFocus();

        /// @brief get double value (only for types 'u' and 'd')
        double getDoubleValue() const;

        /// @brief set tooltip
        void setTooltip(const std::string& toolTip);

        /// @brief get textField
        MFXTextFieldTooltip* getTextField() const;

        /// @brief get index label
        FXLabel* getIndexLabel() const;

        /// @brief get add button
        MFXMenuButtonTooltip* getAddButton() const;

        /// @brief get remove, move up or move down button
        MFXButtonTooltip* getButton();

        /// @brief get add phase button
        MFXButtonTooltip* getAddPhaseButton();

        /// @brief get duplicate phase button
        MFXButtonTooltip* getDuplicatePhaseButton();

        /// @brief get add all red phase button
        MFXButtonTooltip* getAddAllRedPhaseButton();

        /// @brief get add all yellow phase button
        MFXButtonTooltip* getAddAllYellowPhaseButton();

        /// @brief get add all green phase button
        MFXButtonTooltip* getAddAllGreenPhaseButton();

        /// @brief get add all green priority phase button
        MFXButtonTooltip* getAddAllGreenPriorityPhaseButton();

        /// @brief show label index normal
        void showIndexLabelNormal();

        /// @brief show label index bold
        void showIndexLabelBold();

        /// @brief column index
        int getCol() const;

        /// @brief row index
        int getRow() const;

        /// @brief get column type
        char getType() const;

        /// @brief hide menuButton popup
        void hideMenuButtonPopup();

        /// @brief disable button (used for delete, move up and move down)
        void disableButton();

    private:
        /// @brief pointer to TLSTable parent
        GNETLSTable* myTLSTable = nullptr;

        /// @brief MFXTextFieldTooltip
        MFXTextFieldTooltip* myTextField = nullptr;

        /// @brief index label
        FXLabel* myIndexLabel = nullptr;

        /// @brief index label bold
        FXLabel* myIndexLabelBold = nullptr;

        /// @brief button
        MFXButtonTooltip* myButton = nullptr;

        /// @brief popup for buttons
        FXPopup* myMenuButtonPopup = nullptr;

        /// @brief menu button tooltip
        MFXMenuButtonTooltip* myAddButton = nullptr;

        /// @brief add phase button
        MFXButtonTooltip* myAddPhaseButton = nullptr;

        /// @brief duplicate phase button
        MFXButtonTooltip* myDuplicatePhaseButton = nullptr;

        /// @brief add all red phase button
        MFXButtonTooltip* myAddAllRedButton = nullptr;

        /// @brief add all yellow phase button
        MFXButtonTooltip* myAddAllYellowButton = nullptr;

        /// @brief add all green phase button
        MFXButtonTooltip* myAddAllGreenButton = nullptr;

        /// @brief add all green priority phase button
        MFXButtonTooltip* myAddAllGreenPriorityButton = nullptr;

        /// @brief column index
        const int myCol;

        /// @brief row index
        const int myRow;

        /// @brief disable button
        bool myDisableButton = false;

        /// @brief default constructor
        Cell();
    };

    /// @brief table column
    class Column {

    public:
        /// @brief constructor
        Column(GNETLSTable* table, const int index, const char type);

        /// @brief destructor
        ~Column();

        /// @brief get vertical cell frame
        FXVerticalFrame* getVerticalCellFrame() const;

        /// @brief get column type
        char getType() const;

        /// @brief get column label top
        FXString getColumnLabelTop() const;

        /// @brief set column label top
        void setColumnLabelTop(const std::string& text, const std::string& tooltip);

        /// @brief set column label boit
        void setColumnLabelBot(const std::string& text);

        /// @brief get column minimum width
        int getColumnMinimumWidth();

        /// @brief set colum width
        void setColumnWidth(const int colWidth);

    private:
        /// @brief pointer to table
        GNETLSTable* myTable = nullptr;

        /// @brief vertical frame
        FXVerticalFrame* myVerticalFrame = nullptr;

        /// @brief column top tooltip label
        MFXLabelTooltip* myTopLabel = nullptr;

        /// @brief vertical frame
        FXVerticalFrame* myVerticalCellFrame = nullptr;

        /// @brief column bot label
        FXLabel* myBotLabel = nullptr;

        /// @brief column index
        const int myIndex;

        /// @brief column type
        const char myType;

        /// @brief check if current type correspond to a textField
        bool isTextFieldColumn() const;

        /// @brief default constructor
        Column();
    };

    /// @brief table row
    class Row {

    public:
        /// @brief constructor
        Row(GNETLSTable* table);

        /// @brief destructor
        ~Row();

        /// @brief get text
        std::string getText(int index) const;

        /// @brief set text
        void setText(int index, const std::string& text) const;

        /// @brief get cells
        const std::vector<Cell*>& getCells() const;

        /// @brief disable row buttons
        void disableButtons();

    protected:
        /// @brief poiner to table parent
        GNETLSTable* myTable = nullptr;

        /// @brief list wtih cells
        std::vector<Cell*> myCells;

    private:
        /// @brief default constructor
        Row();
    };

    /// @brief update index labels
    void updateIndexLabel();

    /// @brief update accumulated duration();
    void updateAccumulatedDuration();

    /// @brief move focus to current row
    bool moveFocus();

    /// @brief font for the phase table
    FXFont* myProgramFont = nullptr;

    /// @brief font for index
    FXFont* myIndexFont = nullptr;

    /// @brief font for index selected
    FXFont* myIndexSelectedFont = nullptr;

    /// @frame pointer to TLSEditorFrame phases parent
    GNETLSEditorFrame::TLSPhases* myTLSPhasesParent = nullptr;

    /// @brief columns
    std::vector<Column*> myColumns;

    /// @brief rows
    std::vector<Row*> myRows;

    /// @brief current selected row
    int myCurrentSelectedRow = -1;

private:
    /// @brief Invalidated duplicate constructor.
    GNETLSTable(const GNETLSTable&) = delete;

    /// @brief Invalidated assignment operator.
    GNETLSTable& operator=(const GNETLSTable&) = delete;
};

#endif
