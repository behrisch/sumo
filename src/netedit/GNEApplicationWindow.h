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
/// @file    GNEApplicationWindow.h
/// @author  Jakob Erdmann
/// @date    Feb 2011
///
// The main window of Netedit (adapted from GUIApplicationWindow)
/****************************************************************************/
#pragma once
#include <config.h>

#include "GNEApplicationWindowHelper.h"

// ===========================================================================
// class declarations
// ===========================================================================

class GNEInternalTest;
class GNELoadThread;
class GNETagPropertiesDatabase;
class GNEUndoList;
class GNEUndoListDialog;
class GUIEvent;

// ===========================================================================
// class definition
// ===========================================================================
/**
 * @class GNEApplicationWindow
 * @brief The main window of Netedit.
 *
 * Contains the file opening support and a canvas to display the network in.
 *
 * Beside views on the simulation, shown within a MDI-window, the main window
 * may also have some further views (children) assigned which are stored
 * within a separate list.
 */
class GNEApplicationWindow : public GUIMainWindow, public MFXInterThreadEventClient {
    /// @brief FOX-declaration
    FXDECLARE(GNEApplicationWindow)

public:

    /**@brief Constructor
     * @param[in] app The FOX application
     * @param[in] tagPropertiesDatabase pointer to tag properties database
     * @param[in] configPattern The pattern used for loading configurations
     */
    GNEApplicationWindow(FXApp* app, const GNETagPropertiesDatabase* tagPropertiesDatabase, const std::string& configPattern);

    /// @brief Destructor
    ~GNEApplicationWindow();

    /// @brief Creates the main window (required by FOX)
    void create();

    /// @brief load net on startup
    void loadOptionOnStartup();

    /// @brief create new network
    void createNewNetwork();

    /// @brief load network
    void loadNetwork(const std::string& networkFile);

    /// @brief starts to load a configuration
    void loadConfiguration(const std::string& configurationFile);

    /// @brief starts to load a OSM File
    void loadOSM(const std::string& OSMFile);

    /// @brief build dependent
    void dependentBuild();

    /// @brief set text of the statusBar
    void setStatusBarText(const std::string& statusBarText);

    /// @brief called if the user selects Processing->compute junctions with volatile options
    long computeJunctionWithVolatileOptions(const InternalTestStep::ModalArguments* modalArguments);

    /// @brief check if console options was already loaded
    bool consoleOptionsLoaded();

    /// @name Inter-thread event handling
    /// @{
    /// @brief called when an event occurred
    void eventOccurred();

    /// @brief handle event of type Network loaded
    void handleEvent_NetworkLoaded(GUIEvent* e);

    /// @brief handle event of type message
    void handleEvent_Message(GUIEvent* e);
    /// @}

    /// @name FOX-callbacks
    /// @{
    /// @brief called when the command/FXCall new window is executed
    long onCmdNewWindow(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall new network is executed
    long onCmdNewNetwork(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall open netconvertconfiguration is executed
    long onCmdOpenNetconvertConfig(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall open network is executed
    long onCmdOpenNetwork(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall open foreign is executed
    long onCmdOpenForeign(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall open netedit config is executed
    long onCmdOpenNeteditConfig(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall open SumoConfig is executed
    long onCmdOpenSumoConfig(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall reload netedit config is executed
    long onCmdReloadNeteditConfig(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall reload SumoConfig is executed
    long onCmdReloadSumoConfig(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall reload netedit config is updated
    long onUpdReloadNeteditConfig(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall reload SumoConfig is updated
    long onUpdReloadSumoConfig(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall open TLS programs is executed
    long onCmdOpenTLSPrograms(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall reload TLS programs is executed
    long onCmdReloadTLSPrograms(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall reload TLS programs is updated
    long onUpdReloadTLSPrograms(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall open edgeType is executed
    long onCmdOpenEdgeTypes(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall reload edge types is executed
    long onCmdReloadEdgeTypes(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall reload edge types is updated
    long onUpdReloadEdgeTypes(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall smart reload is executed
    long onCmdSmartReload(FXObject*, FXSelector, void*);

    /// @brief called when the update/FXCall smart reload is executed
    long onUpdSmartReload(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall network reload is executed
    long onCmdReloadNetwork(FXObject*, FXSelector, void*);

    /// @brief called when the update/FXCall network reload is executed
    long onUpdReloadNetwork(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall open recent is executed
    long onCmdOpenRecent(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall close is executed
    long onCmdClose(FXObject*, FXSelector, void*);

    /// @brief Called on menu Edit->Visualization
    long onCmdEditViewScheme(FXObject*, FXSelector, void*);

    /// @brief Called on menu Edit->Viewport
    long onCmdEditViewport(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall locate is executed
    long onCmdLocate(FXObject*, FXSelector, void*);

    /// @brief called when user press over a tool dialog button
    long onCmdOpenPythonToolDialog(FXObject* obj, FXSelector, void*);

    /// @brief called when user run a tool
    long onCmdRunPythonTool(FXObject* obj, FXSelector, void*);

    /// @brief post processing after run tool
    long onCmdPostProcessingPythonTool(FXObject* obj, FXSelector, void*);

    /// @brief called when the command/FXCall python tool is updated
    long onUpdPythonTool(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save network is executed
    long onCmdSaveNetwork(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save network as is executed
    long onCmdSaveNetworkAs(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save as plain xml is executed
    long onCmdSavePlainXMLAs(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save joined is executed
    long onCmdSaveJoinedJunctionsAs(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save netedit config is executed
    long onCmdSaveNeteditConfig(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save netedit config as is executed
    long onCmdSaveNeteditConfigAs(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save netedit config is updated
    long onUpdSaveNeteditConfig(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save SumoConfig is executed
    long onCmdSaveSumoConfig(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save SumoConfig as is executed
    long onCmdSaveSumoConfigAs(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save SumoConfig is updated
    long onUpdSaveSumoConfig(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save TLSPrograms is executed
    long onCmdSaveTLSPrograms(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save TLSPrograms is updated
    long onUpdSaveTLSPrograms(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save edge types is executed
    long onCmdSaveEdgeTypes(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save edge types is updated
    long onUpdSaveEdgeTypes(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save TLSPrograms as is executed
    long onCmdSaveTLSProgramsAs(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save edgeTypes as is executed
    long onCmdSaveEdgeTypesAs(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save edgeTypes as is updated
    long onUpdSaveEdgeTypesAs(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall open additionals is executed
    long onCmdOpenAdditionalElements(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall reload additionals is executed
    long onCmdReloadAdditionalElements(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall reload additionals is updated
    long onUpdReloadAdditionalElements(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save additionals is executed
    long onCmdSaveAdditionalElements(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save additionals unified is executed
    long onCmdSaveAdditionalElementsUnified(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save JuPedSim elements as is executed
    long onCmdSaveJuPedSimElementsAs(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall open demand is executed
    long onCmdOpenDemandElements(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall reload demand elements is executed
    long onCmdReloadDemandElements(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall reload demand elements is updated
    long onUpdReloadDemandElements(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save demand elements is executed
    long onCmdSaveDemandElements(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall open data is executed
    long onCmdOpenDataElements(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall reload data elements is executed
    long onCmdReloadDataElements(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall reload data elements is updated
    long onUpdReloadDataElements(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save demand elements unified is executed
    long onCmdSaveDemandElementsUnified(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save data elements is executed
    long onCmdSaveDataElements(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save data elements unified is executed
    long onCmdSaveDataElementsUnified(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall open meanDatas is executed
    long onCmdOpenMeanDataElements(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall reload meanDatas is executed
    long onCmdReloadMeanDataElements(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall reload meanDatas is updated
    long onUpdReloadMeanDataElements(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save meanDatas is executed
    long onCmdSaveMeanDataElements(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall save meanDatas unified is executed
    long onCmdSaveMeanDataElementsUnified(FXObject*, FXSelector, void*);

    /// @brief called when the update/FXCall needs network is executed
    long onUpdNeedsNetwork(FXObject*, FXSelector, void*);

    /// @brief called when the update/FXCall needs at least one network element is executed
    long onUpdNeedsNetworkElement(FXObject*, FXSelector, void*);

    /// @brief called when the update/FXCall needs front element is executed
    long onUpdNeedsFrontElement(FXObject*, FXSelector, void*);

    /// @brief called when the update/FXCall save network is executed
    long onUpdSaveNetwork(FXObject*, FXSelector, void*);

    /// @brief called when the update/FXCall save additionals is executed
    long onUpdSaveAdditionalElements(FXObject*, FXSelector, void*);

    /// @brief called when the update/FXCall save additionals unified is executed
    long onUpdSaveAdditionalElementsUnified(FXObject*, FXSelector, void*);

    /// @brief called when the update/FXCall save juPedSim as is executed
    long onUpdSaveJuPedSimElementsAs(FXObject*, FXSelector, void*);

    /// @brief called when the update/FXCall save demand elements is executed
    long onUpdSaveDemandElements(FXObject*, FXSelector, void*);

    /// @brief called when the update/FXCall save demand elements unified is executed
    long onUpdSaveDemandElementsUnified(FXObject*, FXSelector, void*);

    /// @brief called when the update/FXCall save data elements is executed
    long onUpdSaveDataElements(FXObject*, FXSelector, void*);

    /// @brief called when the update/FXCall save data elements unified is executed
    long onUpdSaveDataElementsUnified(FXObject*, FXSelector, void*);

    /// @brief called when the update/FXCall save meanDatas is executed
    long onUpdSaveMeanDataElements(FXObject*, FXSelector, void*);

    /// @brief called when the update/FXCall save meanDatas unified is executed
    long onUpdSaveMeanDataElementsUnified(FXObject*, FXSelector, void*);

    /// @brief called when the update/FXCall undo is executed
    long onUpdUndo(FXObject* obj, FXSelector sel, void* ptr);

    /// @brief called when the update/FXCall redo is executed
    long onUpdRedo(FXObject* obj, FXSelector sel, void* ptr);

    /// @brief called when the update/FXCall compute path manager is executed
    long onUpdComputePathManager(FXObject* obj, FXSelector sel, void* ptr);

    /// @brief toggle viewOption
    long onCmdToggleViewOption(FXObject*, FXSelector, void*);

    /// @brief update viewOption
    long onUpdToggleViewOption(FXObject*, FXSelector, void*);

    /// @brief called when a key is pressed
    long onKeyPress(FXObject* o, FXSelector sel, void* data);

    /// @brief called when a key is released
    long onKeyRelease(FXObject* o, FXSelector sel, void* data);

    /// @brief Called by FOX if the application shall be closed
    long onCmdQuit(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall edit chosen is executed
    long onCmdEditChosen(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall show about dialog is executed
    long onCmdAbout(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall on update open executed
    long onUpdOpen(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall clear message windows is executed
    long onCmdClearMsgWindow(FXObject*, FXSelector, void*);

    /// @brief called when user toggle windows checkbox "load additionals"
    long onCmdLoadAdditionalsInSUMOGUI(FXObject*, FXSelector, void*);

    /// @brief called when user toggle windows checkbox "load demand"
    long onCmdLoadDemandInSUMOGUI(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall load thread is executed
    long onLoadThreadEvent(FXObject*, FXSelector, void*);

    /// @brief called when the command/FXCall clipboard request is executed
    long onClipboardRequest(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief called if the user hits an edit-supermode hotkey
    long onCmdSetSuperMode(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief called if the user hits an edit-mode hotkey
    long onCmdSetMode(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief called when user press a lock menu check
    long onCmdLockElements(FXObject*, FXSelector sel, void*);

    /// @brief called when user press lock all elements button
    long onCmdLockAllElements(FXObject*, FXSelector sel, void*);

    /// @brief called when user press unlock all elements button
    long onCmdUnlockAllElements(FXObject*, FXSelector sel, void*);

    /// @brief called when user press lock select elements button
    long onCmdLockSelectElements(FXObject*, FXSelector sel, void*);

    /// @brief enable or disable lock menu title
    long onUpdLockMenuTitle(FXObject*, FXSelector sel, void*);

    /// @brief called when user press a process button (or a shortcut)
    long onCmdProcessButton(FXObject* sender, FXSelector sel, void* arg);

    /// @brief called if the user hints ctrl + T
    long onCmdOpenSUMOGUI(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief called if the user hits esc
    long onCmdAbort(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief called if the user hits del
    long onCmdDel(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief called if the user hits enter
    long onCmdEnter(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief called if the user hits backspace
    long onCmdBackspace(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief called if the user hits key combination for clear selection
    long onCmdClearSelectionShortcut(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief called if the user hits key combination for focus on frame
    long onCmdFocusFrame(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief called if the user press the toggle time format button
    long onCmdToggleTimeFormat(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief update toggle time format button
    long onUpdToggleTimeFormat(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief run tests
    long onCmdRunTests(FXObject*, FXSelector, void*);

    /// @brief enable or disable sender object depending if viewNet exist
    long onUpdRequireViewNet(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief update label for require recomputing
    long onUpdRequireRecomputing(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief called when run netgenerate is called
    long onCmdRunNetgenerate(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief postprocessing netgenerate
    long onCmdPostprocessingNetgenerate(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief called if the user press key combination Ctrl + G to toggle grid
    long onCmdToggleGrid(FXObject*, FXSelector, void*);

    /// @brief called if the user press key combination Ctrl + J to toggle draw junction shape
    long onCmdToggleDrawJunctionShape(FXObject*, FXSelector, void*);

    /// @brief called if the user call set front element
    long onCmdSetFrontElement(FXObject*, FXSelector, void*);

    /// @brief called if the user press key combination Alt + <0-9>
    long onCmdToggleEditOptions(FXObject*, FXSelector, void*);

    /// @brief called if the user selects help->Documentation
    long onCmdHelp(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief called if the user selects help->Changelog
    long onCmdChangelog(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief called if the user selects help->Hotkeys
    long onCmdHotkeys(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief called if the user selects help->Tutorial
    long onCmdTutorial(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief called if the user selects help->feedback
    long onCmdFeedback(FXObject* sender, FXSelector sel, void* ptr);

    /// @brief called when toggle checkbox compute network when switching between supermodes
    long onCmdToggleComputeNetworkData(FXObject*, FXSelector, void*);

    /// @brief called when toggle checkbox disable undo redo (processing)
    long onCmdToggleUndoRedo(FXObject*, FXSelector, void*);

    /// @brief called when toggle checkbox disable undo redo during loading (file)
    long onCmdToggleUndoRedoLoading(FXObject*, FXSelector, void*);

    /// @brief called when user press "options" button
    long onCmdOpenOptionsDialog(FXObject*, FXSelector, void*);

    /// @brief called when user press "sumo options" button
    long onCmdOpenSumoOptionsDialog(FXObject*, FXSelector, void*);

    /// @brief called when user press "netgenerate" button
    long onCmdOpenNetgenerateDialog(FXObject*, FXSelector, void*);

    /// @brief called when user press "netgenerate options" button
    long onCmdOpenNetgenerateOptionsDialog(FXObject*, FXSelector, void*);

    /// @brief called when user press Ctrl+Z
    long onCmdUndo(FXObject*, FXSelector, void*);

    // @brief called when user press Ctrl+Y
    long onCmdRedo(FXObject*, FXSelector, void*);

    // @brief called when user press open undoList dialog
    long onCmdOpenUndoListDialog(FXObject*, FXSelector, void*);

    // @brief called when openUndoList is updated
    long onUpdOpenUndoListDialog(FXObject*, FXSelector, void*);

    // @brief called when user press compute path manager
    long onCmdComputePathManager(FXObject*, FXSelector, void*);

    /// @brief called when user press Ctrl+Z
    long onCmdCut(FXObject*, FXSelector, void*);

    // @brief called when user press Ctrl+Y
    long onCmdCopy(FXObject*, FXSelector, void*);

    // @brief called when user press Ctrl+Y
    long onCmdPaste(FXObject*, FXSelector, void*);

    // @brief called when user press Ctrl+F1
    long onCmdSetTemplate(FXObject*, FXSelector, void*);

    // @brief called when user press Ctrl+F2
    long onCmdCopyTemplate(FXObject*, FXSelector, void*);

    // @brief called when user press Ctrl+F3
    long onCmdClearTemplate(FXObject*, FXSelector, void*);

    /// @}

    /// @name inherited from GUIMainWindow
    /// @{
    /// @brief get build OpenGL Canvas (due netedit only uses a single View, it always return nullptr)
    FXGLCanvas* getBuildGLCanvas() const;

    /// @brief get current simulation time (pure virtual but we don't need it)
    SUMOTime getCurrentSimTime() const;

    /// @brief get current tracker interval (pure virtual but we don't need it)
    double getTrackerInterval() const;
    /// @}

    /// @brief get default cursor
    FXCursor* getDefaultCursor();

    /// @brief get tag properties database
    const GNETagPropertiesDatabase* getTagPropertiesDatabase() const;

    /// @brief get pointer to undoList
    GNEUndoList* getUndoList();

    /// @brief get pointer to undoList dialog
    GNEUndoListDialog* getUndoListDialog();

    /// @brief get pointer to viewNet
    GNEViewNet* getViewNet();

    /// @brief get ToolbarsGrip
    GNEApplicationWindowHelper::ToolbarsGrip& getToolbarsGrip();

    /// @brief update recomputing label
    void updateRecomputingLabel();

    /// @brief update control contents after undo/redo or recompute
    void updateControls();

    /// @brief update FXMenuCommands depending of supermode
    void updateSuperModeMenuCommands(const Supermode supermode);

    /// @name functions related with undo-redo
    /// @{
    /// @brief check if undo-redo is allow (processing/checkbox)
    bool isUndoRedoAllowed() const;

    /// @brief enable undo-redo temporally (for example, after creating an edge)
    void enableUndoRedoTemporally();

    /// @brief disable undo-redo temporally giving a string with the reason  (for example, if we're creating an edge)
    void disableUndoRedoTemporally(const std::string& reason);

    /// @brief check if undo-redo is enabled temporally
    const std::string& isUndoRedoEnabledTemporally() const;

    /// @brief clear undo list
    void clearUndoList();

    /// @}

    /// @brief get file Menu Commands (needed for show/hide menu commands)
    GNEApplicationWindowHelper::FileMenuCommands& getFileMenuCommands();

    /// @brief get modes Menu Commands (needed for show/hide menu commands)
    GNEApplicationWindowHelper::ModesMenuCommands& getModesMenuCommands();

    /// @brief get Edit Menu Commands (needed for show/hide menu commands)
    GNEApplicationWindowHelper::EditMenuCommands& getEditMenuCommands();

    /// @brief get processing Menu Commands
    const GNEApplicationWindowHelper::ProcessingMenuCommands& getProcessingMenuCommands() const;

    /// @brief get lock Menu Commands
    GNEApplicationWindowHelper::LockMenuCommands& getLockMenuCommands();

    /// @brief get SUMO options container
    OptionsCont& getSumoOptions();

    /// @brief get netgenerate options container
    OptionsCont& getNetgenerateOptions();

    /// @brief load additional elements
    void loadAdditionalElements();

    /// @brief load demand elements
    void loadDemandElements();

    /// @brief load data elements
    void loadDataElements();

    /// @brief load meanData elements
    void loadMeanDataElements();

    /// @name functions related with test system
    /// @{

    /// @brief get netedit test system
    GNEInternalTest* getInternalTest() const;

    /// @brief check if ignore input signal (using during netedit tests)
    bool allowInputSignals(FXObject* obj) const;

    /// @}

protected:
    /// @brief FOX needs this for static members
    GNEApplicationWindow();

    /// @brief the thread that loads the network
    GNELoadThread* myLoadThread = nullptr;

    /// @brief internal test system
    GNEInternalTest* myInternalTest = nullptr;

    /// @brief information whether the gui is currently loading and the load-options shall be greyed out
    bool myAmLoading = false;

    /// @brief the submenus
    FXMenuPane* myFileMenu = nullptr;
    FXMenuPane* myFileMenuNeteditConfig = nullptr;
    FXMenuPane* myFileMenuSumoConfig = nullptr;
    FXMenuPane* myFileMenuTLS = nullptr;
    FXMenuPane* myFileMenuEdgeTypes = nullptr;
    FXMenuPane* myFileMenuAdditionals = nullptr;
    FXMenuPane* myFileMenuDemandElements = nullptr;
    FXMenuPane* myFileMenuDataElements = nullptr;
    FXMenuPane* myFileMenuMeanDataElements = nullptr;
    FXMenuPane* myFileMenuRecentNetworks = nullptr;
    FXMenuPane* myFileMenuRecentConfigs = nullptr;
    FXMenuPane* myModesMenu = nullptr;
    FXMenuPane* myEditMenu = nullptr;
    FXMenuPane* myLockMenu = nullptr;
    FXMenuPane* myProcessingMenu = nullptr;
    FXMenuPane* myLocatorMenu = nullptr;
    FXMenuPane* myToolsMenu = nullptr;
    FXMenuPane* myToolsAssignMenu = nullptr;
    FXMenuPane* myToolsDetectorMenu = nullptr;
    FXMenuPane* myToolsDistrictMenu = nullptr;
    FXMenuPane* myToolsDRTMenu = nullptr;
    FXMenuPane* myToolsEmissionsMenu = nullptr;
    FXMenuPane* myToolsImportMenu = nullptr;
    FXMenuPane* myToolsImportCityBrainMenu = nullptr;
    FXMenuPane* myToolsImportGTFSMenu = nullptr;
    FXMenuPane* myToolsImportVissim = nullptr;
    FXMenuPane* myToolsImportVisum = nullptr;
    FXMenuPane* myToolsNetMenu = nullptr;
    FXMenuPane* myToolsRouteMenu = nullptr;
    FXMenuPane* myToolsOutputMenu = nullptr;
    FXMenuPane* myToolsShapes = nullptr;
    FXMenuPane* myToolsTLS = nullptr;
    FXMenuPane* myToolsTriggerMenu = nullptr;
    FXMenuPane* myToolsTurnDefs = nullptr;
    FXMenuPane* myToolsVisualizationMenu = nullptr;
    FXMenuPane* myToolsXML = nullptr;
    FXMenuPane* myWindowMenu = nullptr;
    FXMenuPane* myHelpMenu = nullptr;

    /// @brief map with menu pane tools and strings
    std::map<std::string, FXMenuPane*> myMenuPaneToolMaps;

    /// @brief menu title for modes
    FXMenuTitle* myModesMenuTitle = nullptr;

    /// @brief menu title for lock
    FXMenuTitle* myLockMenuTitle = nullptr;

    /// @brief A window to display messages, warnings and error in
    GUIMessageWindow* myMessageWindow = nullptr;

    /// @brief The splitter that divides the main window into view and the log window
    FXSplitter* myMainSplitter = nullptr;

    /// @brief Button used for show if recomputing is needed
    MFXButtonTooltip* myRequireRecomputingButton = nullptr;

    /// @brief List of load requests
    MFXSynchQue<GUIEvent*> myThreadEvents;

    /// @brief io-event with the load-thread
    FXEX::MFXThreadEvent myLoadThreadEvent;

    /// @brief check if had dependent build
    bool myHadDependentBuild = false;

    /// @brief tagProperties database
    const GNETagPropertiesDatabase* myTagPropertiesDatabase = nullptr;

    /// @brief we are responsible for the net
    GNENet* myNet = nullptr;

    /// @brief the one and only undo list
    GNEUndoList* myUndoList = nullptr;

    /// @brief undoList dialog
    GNEUndoListDialog* myUndoListDialog = nullptr;

    /// @brief Input file pattern
    std::string myConfigPattern;

    /// @brief string to check if undo/redo list is enabled (a String is used to keep the disabling reason)
    std::string myUndoRedoListEnabled;

    /// @brief original netedit options container
    OptionsCont myOriginalNeteditOptions;

    /// @brief sumo options container
    OptionsCont mySumoOptions;

    /// @brief original sumo options container
    OptionsCont myOriginalSumoOptions;

    /// @brief netgenerate options container
    OptionsCont myNetgenerateOptions;

    /// @brief original netgenerate options container
    OptionsCont myOriginalNetgenerateOptions;

    /// @brief flag for check if console options was already loaded
    bool myConsoleOptionsLoaded = true;

private:
    /// @brief Toolbars Grip
    GNEApplicationWindowHelper::ToolbarsGrip myToolbarsGrip;

    /// @brief MenuBarFile
    GNEApplicationWindowHelper::MenuBarFile myMenuBarFile;

    /// @brief File Menu Commands
    GNEApplicationWindowHelper::FileMenuCommands myFileMenuCommands;

    /// @brief Modes Menu Commands
    GNEApplicationWindowHelper::ModesMenuCommands myModesMenuCommands;

    /// @brief Edit Menu Commands
    GNEApplicationWindowHelper::EditMenuCommands myEditMenuCommands;

    /// @brief Lock Menu Commands
    GNEApplicationWindowHelper::LockMenuCommands myLockMenuCommands;

    /// @brief Processing Menu Commands
    GNEApplicationWindowHelper::ProcessingMenuCommands myProcessingMenuCommands;

    /// @brief Locate Menu Commands
    GNEApplicationWindowHelper::LocateMenuCommands myLocateMenuCommands;

    /// @brief Tools Menu Commands
    GNEApplicationWindowHelper::ToolsMenuCommands myToolsMenuCommands;

    /// @brief Windows Menu Commands
    GNEApplicationWindowHelper::WindowsMenuCommands myWindowsMenuCommands;

    /// @brief Help Menu Commands
    GNEApplicationWindowHelper::HelpMenuCommands myHelpMenuCommands;

    /// @brief Supermode Commands
    GNEApplicationWindowHelper::SupermodeCommands mySupermodeCommands;

    /// @brief pointer to current view net
    GNEViewNet* myViewNet = nullptr;

    /// @brief the prefix for the window title
    const FXString myTitlePrefix;

    /// @brief allow undo-redo (read from registry)
    bool myAllowUndoRedo = true;

    /// @brief allow undo-redo loading (read from registry)
    bool myAllowUndoRedoLoading = true;

    /// @brief The menu used for the MDI-windows
    FXMDIMenu* myMDIMenu = nullptr;

    /// @brief Builds the menu bar
    void fillMenuBar();

    /// @brief this method closes all windows and deletes the current simulation */
    void closeAllWindows();

    /// @brief warns about unsaved changes and gives the user the option to abort
    bool askSaveElements();

    /// @brief set input files in sumo options
    void setInputInSumoOptions(const bool ignoreAdditionals, const bool ignoreRoutes);

    /// @brief extract folder
    FXString getFolder(const std::string& folder) const;

    /// @brief Invalidated copy constructor.
    GNEApplicationWindow(const GNEApplicationWindow&) = delete;

    /// @brief Invalidated assignment operator.
    GNEApplicationWindow& operator=(const GNEApplicationWindow&) = delete;
};
