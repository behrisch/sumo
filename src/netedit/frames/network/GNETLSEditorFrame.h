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
/// @file    GNETLSEditorFrame.h
/// @author  Jakob Erdmann
/// @date    May 2011
///
// The Widget for modifying traffic lights
/****************************************************************************/
#pragma once
#include <config.h>

#include <netedit/frames/GNEFrame.h>
#include <netedit/GNEViewNetHelper.h>
#include <netbuild/NBTrafficLightLogic.h>

// ===========================================================================
// class declarations
// ===========================================================================

class GNEOverlappedInspection;
class GNETLSTable;
class MFXComboBoxIcon;
class MFXTextFieldTooltip;
class MFXToggleButtonTooltip;
class NBLoadedSUMOTLDef;
class NBOwnTLDef;
class NBTrafficLightDefinition;

// ===========================================================================
// class definitions
// ===========================================================================

class GNETLSEditorFrame : public GNEFrame {

public:
    // ===========================================================================
    // class TLSJunction
    // ===========================================================================

    class TLSJunction : public MFXGroupBoxModule {
        /// @brief FOX-declaration
        FXDECLARE(GNETLSEditorFrame::TLSJunction)

    public:
        /// @brief constructor
        TLSJunction(GNETLSEditorFrame* TLSEditorParent);

        /// @brief destructor
        ~TLSJunction();

        /// @brief update TLSJunction module
        void updateTLSJunction();

        /// @brief get current modified junction
        GNEJunction* getCurrentJunction() const;

        /// @brief set current junction
        void setCurrentJunction(GNEJunction* junction);

        /// @brief is joining junctions
        bool isJoiningJunctions() const;

        /// @brief check if given junction is selected (used fo joining)
        bool isJunctionSelected(const GNEJunction* junction) const;

        /// @brief select or unselect junction in the current TLS
        void toggleJunctionSelected(const GNEJunction* junction);

        /// @brief get selected junction IDs
        const std::vector<std::string>& getSelectedJunctionIDs() const;

        /// @name FOX-callbacks
        /// @{
        /// @brief Called when the user rename TLS
        long onCmdRenameTLS(FXObject*, FXSelector, void*);

        /// @brief Called when the user change TLS Type
        long onCmdChangeType(FXObject*, FXSelector, void*);

        /// @brief Called when the user join TLS
        long onCmdToggleJoinTLS(FXObject*, FXSelector, void*);

        /// @brief Called when the user join TLS
        long onCmdDisjoinTLS(FXObject*, FXSelector, void*);

        /// @brief accept join
        long onCmdAcceptJoin(FXObject*, FXSelector, void*);

        /// @brief cancel join
        long onCmdCancelJoin(FXObject*, FXSelector, void*);

        /// @}

    protected:
        /// @brief FOX needs this
        FOX_CONSTRUCTOR(TLSJunction)

        /// @brief refresh TLJunction module
        void refreshTLSJunction();

    private:
        /// @brief TLS editor frame parent
        GNETLSEditorFrame* myTLSEditorParent;

        /// @brief label for junction ID
        FXLabel* myJunctionIDLabel = nullptr;

        /// @brief text field for junction ID
        MFXTextFieldTooltip* myJunctionIDTextField = nullptr;

        /// @brief text field for junction ID
        MFXTextFieldTooltip* myTLSIDTextField = nullptr;

        /// @brief ComboBox for TLS Types
        MFXComboBoxIcon* myTLSTypeComboBox = nullptr;

        /// @brief Toggle button for join TLS
        MFXToggleButtonTooltip* myJoinTLSToggleButton = nullptr;

        /// @brief button for disjoin TLS
        MFXButtonTooltip* myDisjoinTLSButton = nullptr;

        /// @brief frame for accept/cancel buttons
        FXHorizontalFrame* myJoinControlButtons = nullptr;

        /// @brief the junction of the tls is being modified
        GNEJunction* myCurrentJunction = nullptr;

        /// @brief selected junction (used for join)
        std::vector<std::string> mySelectedJunctionIDs;

        /// @brief original selected junction (used for join)
        std::vector<std::string> myOriginalSelectedJunctionIDs;

        /// @brief Invalidated copy constructor.
        TLSJunction(const TLSJunction&) = delete;

        /// @brief Invalidated assignment operator.
        TLSJunction& operator=(const TLSJunction&) = delete;
    };

    // ===========================================================================
    // class TLSPrograms
    // ===========================================================================

    class TLSPrograms : public MFXGroupBoxModule {
        /// @brief FOX-declaration
        FXDECLARE(GNETLSEditorFrame::TLSPrograms)

    public:
        /// @brief constructor
        TLSPrograms(GNETLSEditorFrame* TLSEditorParent);

        /// @brief destructor
        ~TLSPrograms();

        /// @brief update TLSPrograms module
        void updateTLSPrograms();

        /// @brief show TLSPrograms
        void showTLSPrograms();

        /// @brief hide TLSPrograms
        void hideTLSPrograms();

        /// @brief init TLS Definitions
        bool initTLSPrograms();

        /// @brief clear TLS Definitions
        void clearTLSProgramss();

        /// @brief get number of programs
        int getNumberOfPrograms() const;

        /// @brief check if current TLS was modified
        bool checkHaveModifications() const;

        /// @brief mark Program as modified
        void markAsModified();

        /// @brief get current definition
        NBTrafficLightDefinition* getCurrentTLSPrograms() const;

        /// @brief get current program ID
        const std::string getCurrentTLSProgramID() const;

        /// @brief discard changes
        void discardChanges(const bool editJunctionAgain);

        /// @name FOX-callbacks
        /// @{
        /// @brief Called when the user press button create/duplicate TLS Program
        long onCmdCreate(FXObject*, FXSelector, void*);

        /// @brief Called when the user press button delete TLS Program
        long onCmdDelete(FXObject*, FXSelector, void*);

        /// @brief Called when the user press button reset current TLS Program
        long onCmdResetCurrentProgram(FXObject*, FXSelector, void*);

        /// @brief Called when the user press button reset all TLS Programs
        long onCmdResetAll(FXObject*, FXSelector, void*);

        /// @brief Called when the user switches a TLS
        long onCmdDefSwitchTLSProgram(FXObject*, FXSelector, void*);

        /// @brief Called when the user presses the save-Button
        long onCmdSaveChanges(FXObject*, FXSelector, void*);

        /// @brief Called when the user presses the Cancel-button
        long onCmdDiscardChanges(FXObject*, FXSelector, void*);

        /// @}

    protected:
        /// @brief FOX needs this
        FOX_CONSTRUCTOR(TLSPrograms)

        /// @brief create (or duplicate) TLS in the given junction
        void createTLS(GNEJunction* junction);

        /// @brief switch program
        bool switchProgram();

    private:
        /// @brief pointer to GNETLSEditorFrame parent
        GNETLSEditorFrame* myTLSEditorParent = nullptr;

        /// @brief the list of Definitions for the current junction
        std::vector<NBTrafficLightDefinition*> myTLSPrograms;

        /// @brief the comboBox for selecting the tl-definition to edit
        MFXComboBoxIcon* myProgramComboBox;

        /// @brief whether the current tls was modified
        bool myHaveModifications = false;

        /// @brief button for create new TLS program
        FXButton* myCreateButton = nullptr;

        /// @brief button for delete existent TLS program
        FXButton* myDeleteButton = nullptr;

        /// @brief button for reset TLS program
        FXButton* myResetSingleButton = nullptr;

        /// @brief button for reset all TLS program
        FXButton* myResetAllButton = nullptr;

        /// @brief button for save TLS program
        FXButton* mySaveButon = nullptr;

        /// @brief button for cancel TLS program
        FXButton* myCancelButon = nullptr;

        /// @brief Invalidated copy constructor.
        TLSPrograms(const TLSPrograms&) = delete;

        /// @brief Invalidated assignment operator.
        TLSPrograms& operator=(const TLSPrograms&) = delete;
    };

    // ===========================================================================
    // class TLSAttributes
    // ===========================================================================

    class TLSAttributes : public MFXGroupBoxModule {
        /// @brief FOX-declaration
        FXDECLARE(GNETLSEditorFrame::TLSAttributes)

    public:
        /// @brief constructor
        TLSAttributes(GNETLSEditorFrame* TLSEditorParent);

        /// @brief destructor
        ~TLSAttributes();

        /// @brief update TLSAttributes module
        void updateTLSAttributes();

        /// @brief show TLSAttributes
        void showTLSAttributes();

        /// @brief hide TLSAttributes
        void hideTLSAttributes();

        /// @brief initializes the definitions and corresponding listbox
        void initTLSAttributes();

        /// @brief clear TLS attributes
        void clearTLSAttributes();

        /// @brief get current offset in string format
        SUMOTime getOffset() const;

        /// @brief set new offset
        void setOffset(const SUMOTime& offset);

        /// @brief is current offset valid
        bool isValidOffset();

        /// @brief get current parameters in string format
        std::string getParameters() const;

        /// @brief set new parameters
        void setParameters(const std::string& parameters);

        /// @brief are current parameter valid
        bool isValidParameters();

        /// @brief toggle button for set detectors mode
        bool isSetDetectorsToggleButtonEnabled() const;

        /// @brief select or unselect E1 detector in the current TLS
        bool toggleE1DetectorSelection(const GNEAdditional* E1);

        /// @brief get E1 detectors vinculated with this TLS
        const std::map<std::string, std::string>& getE1Detectors() const;

        /// @brief disable detector mode
        void disableE1DetectorMode();

        /// @name FOX-callbacks
        /// @{
        /// @brief Called when the user changes the offset of a TLS
        long onCmdSetOffset(FXObject*, FXSelector, void*);

        /// @brief Called when the user changes parameters of a TLS
        long onCmdSetParameters(FXObject*, FXSelector, void*);

        /// @brief Called when user press edit parameters button
        long onCmdParametersDialog(FXObject*, FXSelector, void* ptr);

        /// @brief Called when user toggle set detector mode
        long onCmdToggleDetectorMode(FXObject*, FXSelector, void* ptr);

        /// @}

    protected:
        /// @brief FOX needs this
        FOX_CONSTRUCTOR(TLSAttributes)

        /// @brief update E1 detectors
        void updateE1Detectors();

    private:
        /// @brief pointer to TLSEditorParent
        GNETLSEditorFrame* myTLSEditorParent;

        /// @brief the TextField for modifying offset
        FXTextField* myOffsetTextField = nullptr;

        /// @brief button for edit parameters
        FXButton* myButtonEditParameters = nullptr;

        /// @brief the TextField for modifying parameters
        FXTextField* myParametersTextField = nullptr;

        /// @brief toggle button for set detectors mode
        MFXToggleButtonTooltip* mySetDetectorsToggleButton = nullptr;

        /// @brief set with E1 detector IDs and their lanes vinculated with the TLS <laneID, E1ID>
        std::map<std::string, std::string> myE1Detectors;

        /// @brief Invalidated copy constructor.
        TLSAttributes(const TLSAttributes&) = delete;

        /// @brief Invalidated assignment operator.
        TLSAttributes& operator=(const TLSAttributes&) = delete;
    };

    // ===========================================================================
    // class TLSPhases
    // ===========================================================================

    class TLSPhases : public MFXGroupBoxModule {
        /// @brief FOX-declaration
        FXDECLARE(GNETLSEditorFrame::TLSPhases)

    public:
        /// @brief constructor
        TLSPhases(GNETLSEditorFrame* TLSEditorParent);

        /// @brief destructor
        ~TLSPhases();

        /// @brief update TLSPhases module
        void updateTLSPhases();

        /// @brief show TLSPhases
        void showTLSPhases();

        /// @brief hide TLSPhases
        void hideTLSPhases();

        /// @brief get TLSEditor Parent
        GNETLSEditorFrame* getTLSEditorParent() const;

        /// @brief get phase table
        GNETLSTable* getPhaseTable() const;

        /**@brief initializes the phase table
         * @param[in] index The index to select
         */
        void initPhaseTable();

        /// @brief clear phase thable
        void clearPhaseTable();

        /// @brief change phase value (state, name, next, etc.)
        bool changePhaseValue(const int col, const int row, const std::string& value);

        /// @brief add phase
        void addPhase(const int row, const char c = ' ');

        /// @brief duplicate phase
        void duplicatePhase(const int row);

        /// @brief delete phase
        void removePhase(const int row);

        /// @brief move phase up
        void movePhaseUp(const int row);

        /// @brief move phase down
        void movePhaseDown(const int row);

        /// @brief update TLS coloring
        void updateTLSColoring();

        /// @name FOX-callbacks
        /// @{
        /// @brief Called when the user cleans up states
        long onCmdCleanStates(FXObject*, FXSelector, void*);

        /// @brief Called when the user cleans up states
        long onCmdAddUnusedStates(FXObject*, FXSelector, void*);

        /// @brief Called when the user groups states
        long onCmdGroupStates(FXObject*, FXSelector, void*);

        /// @brief Called when the user ungroups states
        long onCmdUngroupStates(FXObject*, FXSelector, void*);

        /// @}

    protected:
        /// @brief FOX needs this
        FOX_CONSTRUCTOR(TLSPhases)

        /// @brief update table
        void updateTable();

    private:
        /// @brief init static phase table
        void initStaticPhaseTable();

        /// @brief init actuated phase table
        void initActuatedPhaseTable();

        /// @brief init delayBase phase table
        void initDelayBasePhaseTable();

        /// @brief init NEMA phase table
        void initNEMAPhaseTable();

        /// @brief build default phase
        int buildDefaultPhase(const int row);

        /// @brief set duration
        bool setDuration(const int col, const int row, const std::string& value);

        /// @brief set state
        bool setState(const int col, const int row, const std::string& value);

        /// @brief set next
        bool setNext(const int row, const std::string& value);

        /// @brief set name
        bool setName(const int row, const std::string& value);

        /// @brief set minDur
        bool setMinDur(const int row, const std::string& value);

        /// @brief set maxDur
        bool setMaxDur(const int row, const std::string& value);

        /// @brief set earliestEnd
        bool setEarliestEnd(const int row, const std::string& value);

        /// @brief set latestEnd
        bool setLatestEnd(const int row, const std::string& value);

        /// @brief set vehExt
        bool setVehExt(const int row, const std::string& value);

        /// @brief set yellow
        bool setYellow(const int row, const std::string& value);

        /// @brief set red
        bool setRed(const int row, const std::string& value);

        /// @brief recomputes cycle duration and updates label
        void updateCycleDuration(const int col);

        /// @brief update state size
        void updateStateSize(const int col);

        /// @brief pointer to TLSEditor Parent
        GNETLSEditorFrame* myTLSEditorParent;

        /// @brief table for selecting and rearranging phases and for changing duration
        GNETLSTable* myPhaseTable;

        /// @brief clean states button
        MFXButtonTooltip* myCleanStatesButton = nullptr;

        /// @brief add states button
        MFXButtonTooltip* myAddStates = nullptr;

        /// @brief group signals button
        MFXButtonTooltip* myGroupSignalsButton = nullptr;

        /// @brief ungroup signals button
        MFXButtonTooltip* myUngroupSignalsButton = nullptr;

        /// @brief Invalidated copy constructor.
        TLSPhases(const TLSPhases&) = delete;

        /// @brief Invalidated assignment operator.
        TLSPhases& operator=(const TLSPhases&) = delete;
    };

    // ===========================================================================
    // class TLSFile
    // ===========================================================================

    class TLSFile : public MFXGroupBoxModule {
        /// @brief FOX-declaration
        FXDECLARE(GNETLSEditorFrame::TLSFile)

    public:
        /// @brief constructor
        TLSFile(GNETLSEditorFrame* TLSEditorParent);

        /// @brief destructor
        ~TLSFile();

        /// @brief update TLSFile module
        void updateTLSFile();

        /// @brief show TLSPhases
        void showTLSFile();

        /// @brief hide TLSPhases
        void hideTLSFile();

        /// @name FOX-callbacks
        /// @{

        /// @brief load TLS Program from an additional file
        long onCmdLoadTLSProgram(FXObject*, FXSelector, void*);

        /// @brief save TLS Program to an additional file
        long onCmdSaveTLSProgram(FXObject*, FXSelector, void*);

        /// @}

    protected:
        /// @brief FOX needs this
        FOX_CONSTRUCTOR(TLSFile)

        /// @brief convert SUMOTime into string
        std::string writeSUMOTime(SUMOTime steps);

    private:
        /// @brief pointer to TLSEditor Parent
        GNETLSEditorFrame* myTLSEditorParent;

        /// @brief load button
        FXButton* myLoadButton = nullptr;

        /// @brief save button
        FXButton* mySaveButton = nullptr;

        /// @brief Invalidated copy constructor.
        TLSFile(const TLSFile&) = delete;

        /// @brief Invalidated assignment operator.
        TLSFile& operator=(const TLSFile&) = delete;
    };

    /**@brief Constructor
     * @brief viewParent GNEViewParent in which this GNEFrame is placed
     * @brief viewNet viewNet that uses this GNEFrame
     */
    GNETLSEditorFrame(GNEViewParent* viewParent, GNEViewNet* viewNet);

    /// @brief Destructor
    ~GNETLSEditorFrame();

    /// @brief show inspector frame
    void show();

    /// @brief function called after setting new width in current frame
    void frameWidthUpdated();

    /// @brief update modules
    void updateModules();

    /**@brief edits the traffic light for the given clicked junction
     * @param[in] clickedPosition clicked position
     * @param[in] viewObjects The clicked objects under cursor
     */
    void editTLS(GNEViewNetHelper::ViewObjectsSelector& viewObjects, const Position& clickedPosition, const bool shiftKeyPressed);

    /// @brief check if modifications in TLS was saved
    bool isTLSSaved();

    /// @brief parse TLS Programs from a file
    bool parseTLSPrograms(const std::string& file);

    /// @brief update phase definition for the current traffic light and phase
    void handleChange(GNEInternalLane* lane);

    /// @brief update phase definition for the current traffic light and phase
    void handleMultiChange(GNELane* lane, FXObject* obj, FXSelector sel, void* data);

    /// @brief whether the given edge is controlled by the currently edited tlDef
    bool controlsEdge(GNEEdge* edge) const;

    /// @brief open GNEAttributesCreator extended dialog (can be reimplemented in frame children)
    void selectedOverlappedElement(GNEAttributeCarrier* AC);

    /**@brief edits the traffic light for the given junction
     * @param[in] junction The junction of which the traffic light shall be edited
     */
    void editJunction(GNEJunction* junction);

    /// @brief get module for TLS Junction
    GNETLSEditorFrame::TLSJunction* getTLSJunction() const;

    /// @brief get module for TLS Definition
    GNETLSEditorFrame::TLSPrograms* getTLSPrograms() const;

    /// @brief get module for TLS attributes
    GNETLSEditorFrame::TLSAttributes* getTLSAttributes() const;

    /// @brief get module for TLS Phases
    GNETLSEditorFrame::TLSPhases* getTLSPhases() const;

protected:
    /// @brief converts to SUMOTime
    static SUMOTime getSUMOTime(const std::string& value);

    /// @brief converts to SUMOTime
    static const std::string getSteps2Time(const SUMOTime value);

private:
    /// @brief Overlapped Inspection
    GNEOverlappedInspection* myOverlappedInspection = nullptr;

    /// @brief module for TLS Junction
    GNETLSEditorFrame::TLSJunction* myTLSJunction = nullptr;

    /// @brief module for TLS Definition
    GNETLSEditorFrame::TLSPrograms* myTLSPrograms = nullptr;

    /// @brief module for TLS attributes
    GNETLSEditorFrame::TLSAttributes* myTLSAttributes = nullptr;

    /// @brief module for TLS Phases
    GNETLSEditorFrame::TLSPhases* myTLSPhases = nullptr;

    /// @brief module for load/Save TLS Programs
    GNETLSEditorFrame::TLSFile* myTLSFile = nullptr;

    /// @brief the internal lanes belonging to the current junction indexed by their tl-index
    std::map<int, std::vector<GNEInternalLane*> > myInternalLanes;

    /// @brief the traffic light definition being edited
    NBLoadedSUMOTLDef* myEditedDef = nullptr;

    /// @brief cleans up previous lanes
    void cleanup();

    /// @brief builds internal lanes for the given tlDef
    void buildInternalLanes(const NBTrafficLightDefinition* tlDef);

    /// @brief get certain phase of the current traffic light
    const NBTrafficLightLogic::PhaseDefinition& getPhase(const int index);

    /// @brief convert duration (potentially undefined) to string
    static std::string varDurString(SUMOTime dur);

    /// @brief Invalidated copy constructor.
    GNETLSEditorFrame(const GNETLSEditorFrame&) = delete;

    /// @brief Invalidated assignment operator.
    GNETLSEditorFrame& operator=(const GNETLSEditorFrame&) = delete;
};
