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
/// @file    GUIAppEnum.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Andreas Gaubatz
/// @date    Mon, 08.03.2004
///
// Message and object IDs used by the FOX-version of the gui
/****************************************************************************/
#pragma once
#include <config.h>

#include <utils/foxtools/fxheader.h>


// ===========================================================================
// enumeration
// ===========================================================================
/**
 * @brief Message and object IDs used by the FOX-version of the gui
 */
enum {
    /// @brief Show about - dialog
    ID_ABOUT = FXMainWindow::ID_LAST,

    /// @name hotkeys
    /// @{

    /// @brief hotkey for start simulation in SUMO and set editing mode additionals AND stops in netedit
    MID_HOTKEY_A_MODE_STARTSIMULATION_ADDITIONALS_STOPS,
    /// @brief hotkey for mode connecting lanes AND container
    MID_HOTKEY_C_MODE_CONNECT_CONTAINER,
    /// @brief hotkey for setting a breakpoint
    MID_HOTKEY_B_BREAKPOINT,
    /// @brief hotkey for perform a single simulation step in SUMO and set delete mode in netedit
    MID_HOTKEY_D_MODE_SINGLESIMULATIONSTEP_DELETE,
    /// @brief hotkey for mode adding edges AND edgeDatas
    MID_HOTKEY_E_MODE_EDGE_EDGEDATA,
    /// @brief hotkey for mode prohibition AND container plan
    MID_HOTKEY_H_MODE_PROHIBITION_CONTAINERPLAN,
    /// @brief hotkey for mode inspecting object attributes
    MID_HOTKEY_I_MODE_INSPECT,
    /// @brief hotkey for mode person plan
    MID_HOTKEY_L_MODE_PERSONPLAN,
    /// @brief hotkey for mode moving element AND mean data
    MID_HOTKEY_M_MODE_MOVE_MEANDATA,
    /// @brief hotkey for mode creating polygons
    MID_HOTKEY_P_MODE_POLYGON_PERSON,
    /// @brief hotkey for stop simulation in SUMO and set select mode in netedit
    MID_HOTKEY_S_MODE_STOPSIMULATION_SELECT,
    /// @brief hotkey for mode editing crossing, routes and edge rel datas
    MID_HOTKEY_R_MODE_CROSSING_ROUTE_EDGERELDATA,
    /// @brief hotkey for mode editing TLS AND Vehicle Types
    MID_HOTKEY_T_MODE_TLS_TYPE,
    /// @brief hotkey for mode decal AND type distribution
    MID_HOTKEY_U_MODE_DECAL_TYPEDISTRIBUTION,
    /// @brief hotkey for mode create vehicles
    MID_HOTKEY_V_MODE_VEHICLE,
    /// @brief hotkey for mode editing overhead wires AND route distributions
    MID_HOTKEY_W_MODE_WIRE_ROUTEDISTRIBUTION,
    /// @brief hotkey for mode editing TAZ and TAZRel
    MID_HOTKEY_Z_MODE_TAZ_TAZREL,

    /// @}

    /// @name Ctrl hotkeys
    /// @{

    /// @brief Start the simulation in SUMO and open Additionals Elements in netedit
    MID_HOTKEY_CTRL_A_STARTSIMULATION_OPENADDITIONALELEMENTS,
    /// @brief Edit simulation breakpoints in SUMO and open Data Elements in netedit
    MID_HOTKEY_CTRL_B_EDITBREAKPOINT_OPENDATAELEMENTS,
    /// @brief Copy the current selection / element
    MID_HOTKEY_CTRL_C_COPY,
    /// @brief Perform a single simulation step in SUMO and open Demand Elements in netedit
    MID_HOTKEY_CTRL_D_SINGLESIMULATIONSTEP_OPENDEMANDELEMENTS,
    /// @brief Edit selection in SUMO and load neteditConfig in netedit
    MID_HOTKEY_CTRL_E_EDITSELECTION_LOADNETEDITCONFIG,
    /// @brief Fullscreen mode - menu entry
    MID_HOTKEY_CTRL_F_FULSCREENMODE,
    /// @brief Toggle Gaming mode in SUMO and grid in netedit
    MID_HOTKEY_CTRL_G_GAMINGMODE_TOGGLEGRID,
    /// @brief open app setting dialog in SUMO and open edge type files in netedit
    MID_HOTKEY_CTRL_H_APPSETTINGS_OPENEDGETYPES,
    /// @brief Open viewport editor
    MID_HOTKEY_CTRL_I_EDITVIEWPORT,
    /// @brief toggle draw junction shape
    MID_HOTKEY_CTRL_J_TOGGLEDRAWJUNCTIONSHAPE,
    /// @brief Load file with TLS Programs
    MID_HOTKEY_CTRL_K_OPENTLSPROGRAMS,
    /// @brief save network as plain XML
    MID_HOTKEY_CTRL_L_SAVEASPLAINXML,
    /// @brief open sumo config
    MID_HOTKEY_CTRL_M_OPENSUMOCONFIG,
    /// @brief open network in SUMO and create new empty network in netedit
    MID_HOTKEY_CTRL_N_OPENNETWORK_NEWNETWORK,
    /// @brief Open simulation in SUMO and open network in netedit
    MID_HOTKEY_CTRL_O_OPENSIMULATION_OPENNETWORK,
    /// @brief Load additional file with poi and polygons
    MID_HOTKEY_CTRL_P_OPENSHAPES,
    /// @brief Main window closes
    MID_HOTKEY_CTRL_Q_CLOSE,
    /// @brief Stop the simulation in SUMO and save network in netedit
    MID_HOTKEY_CTRL_S_STOPSIMULATION_SAVENETWORK,
    /// @brief Reload the previously loaded simulation
    MID_HOTKEY_CTRL_R_RELOAD,
    /// @brief Quick-Reload the previously loaded simulation (keep the net)
    MID_HOTKEY_CTRL_QUICK_RELOAD,
    /// @brief Open current SUMO simulation/network in netedit, or current netedit simulation/network in SUMO
    MID_HOTKEY_CTRL_T_OPENNETEDIT_OPENSUMO,
    /// @brief Load edge data for visualization
    MID_HOTKEY_CTRL_U_OPENEDGEDATA,
    /// @brief paste the current selection / element
    MID_HOTKEY_CTRL_V_PASTE,
    /// @brief Close simulation - ID
    MID_HOTKEY_CTRL_W_CLOSESIMULATION,
    /// @brief cut the current selection / element
    MID_HOTKEY_CTRL_X_CUT,
    /// @brief Undo
    MID_HOTKEY_CTRL_Y_REDO,
    /// @brief Redo
    MID_HOTKEY_CTRL_Z_UNDO,

    /// @}

    /// @name Alt hotkeys
    /// @{

    /// @brief toggle edit option
    MID_HOTKEY_ALT_0_TOGGLEEDITOPTION,
    /// @brief toggle edit option
    MID_HOTKEY_ALT_1_TOGGLEEDITOPTION,
    /// @brief toggle edit option
    MID_HOTKEY_ALT_2_TOGGLEEDITOPTION,
    /// @brief toggle edit option
    MID_HOTKEY_ALT_3_TOGGLEEDITOPTION,
    /// @brief toggle edit option
    MID_HOTKEY_ALT_4_TOGGLEEDITOPTION,
    /// @brief toggle edit option
    MID_HOTKEY_ALT_5_TOGGLEEDITOPTION,
    /// @brief toggle edit option
    MID_HOTKEY_ALT_6_TOGGLEEDITOPTION,
    /// @brief toggle edit option
    MID_HOTKEY_ALT_7_TOGGLEEDITOPTION,
    /// @brief toggle edit option
    MID_HOTKEY_ALT_8_TOGGLEEDITOPTION,
    /// @brief toggle edit option
    MID_HOTKEY_ALT_9_TOGGLEEDITOPTION,
    /// @brief Main window closes
    MID_HOTKEY_ALT_F4_CLOSE,
    /// @brief hotkey for setting a breakpoint ahead of the current time
    MID_HOTKEY_ALT_B_BREAKPOINT_EARLY,
    /// @}

    /// @name shift + hotkeys
    /// @{

    /// @brief Locate additional structure - button
    MID_HOTKEY_SHIFT_A_LOCATEADDITIONAL,
    /// @brief Locate container - button
    MID_HOTKEY_SHIFT_C_LOCATECONTAINER,
    /// @brief Locate edge - button
    MID_HOTKEY_SHIFT_E_LOCATEEDGE,
    /// @brief Locate junction - button
    MID_HOTKEY_SHIFT_J_LOCATEJUNCTION,
    /// @brief Locate polygons - button
    MID_HOTKEY_SHIFT_L_LOCATEPOLY,
    /// @brief Locate poi - button
    MID_HOTKEY_SHIFT_O_LOCATEPOI,
    /// @brief Locate person - button
    MID_HOTKEY_SHIFT_P_LOCATEPERSON,
    /// @brief Locate route - button
    MID_HOTKEY_SHIFT_R_LOCATEROUTE,
    /// @brief Locate stop - button
    MID_HOTKEY_SHIFT_S_LOCATESTOP,
    /// @brief Locate TLS - button
    MID_HOTKEY_SHIFT_T_LOCATETLS,
    /// @brief Locate vehicle - button
    MID_HOTKEY_SHIFT_V_LOCATEVEHICLE,
    /// @brief Locate edge - button
    MID_HOTKEY_SHIFT_W_LOCATEWALKINGAREA,

    /// @}

    /// @name Ctrl + Shift hotkeys
    /// @{

    /// @brief Save Additional Elements
    MID_HOTKEY_CTRL_SHIFT_A_SAVEADDITIONALELEMENTS,
    /// @brief save Data Elements
    MID_HOTKEY_CTRL_SHIFT_B_SAVEDATAELEMENTS,
    /// @brief Save neteditConfig
    MID_HOTKEY_CTRL_SHIFT_C_SAVENETEDITCONFIG,
    /// @brief Save Demand Elements
    MID_HOTKEY_CTRL_SHIFT_D_SAVEDEMANDELEMENTS,
    /// @brief save netedit Config
    MID_HOTKEY_CTRL_SHIFT_E_SAVENETEDITCONFIG,
    /// @brief save Edge Types
    MID_HOTKEY_CTRL_SHIFT_H_SAVEEDGETYPES,
    /// @brief save TLS Programs
    MID_HOTKEY_CTRL_SHIFT_K_SAVETLS,
    /// @brief save Mean Datas
    MID_HOTKEY_CTRL_SHIFT_M_SAVEMEANDATAELEMENTS,
    /// @brief open a new window (SUMO AND netedit)
    MID_HOTKEY_CTRL_SHIFT_N_NEWWINDOW,
    /// @brief open Netconvert file
    MID_HOTKEY_CTRL_SHIFT_O_OPENNETCONVERTFILE,
    /// @brief save SUMOConfig (SUMO AND netedit)
    MID_HOTKEY_CTRL_SHIFT_S_SAVESUMOCONFIG,

    /// @}

    /// @name Functional hotkeys
    /// @{

    /// @brief open online documentation
    MID_HOTKEY_F1_ONLINEDOCUMENTATION,
    /// @brief select network supermode in netedit
    MID_HOTKEY_F2_SUPERMODE_NETWORK,
    /// @brief select demand supermode in netedit
    MID_HOTKEY_F3_SUPERMODE_DEMAND,
    /// @brief select data supermode in netedit
    MID_HOTKEY_F4_SUPERMODE_DATA,
    /// @brief compute Network in network mode and Demand elements in demand mode
    MID_HOTKEY_F5_COMPUTE_NETWORK_DEMAND,
    /// @brief clean junctions without edges in network mode and unused routes in demand mode
    MID_HOTKEY_F6_CLEAN_SOLITARYJUNCTIONS_UNUSEDROUTES,
    /// @brief join selected junctions in network mode and normalice demand element ids in demand mode
    MID_HOTKEY_F7_JOIN_SELECTEDJUNCTIONS_ROUTES,
    /// @brief clean invalid crossings in network mode and demand elements in demand mode
    MID_HOTKEY_F8_CLEANINVALID_CROSSINGS_DEMANDELEMENTS,
    /// @brief open edit scheme menu
    MID_HOTKEY_F9_EDIT_VIEWSCHEME,
    /// @brief open options menu
    MID_HOTKEY_F10_OPTIONSMENU,
    /// @brief set/clear front element
    MID_HOTKEY_F11_FRONTELEMENT,
    /// @brief show the hall of fame dialog
    MID_HOTKEY_SHIFT_F11_HALLOFFAME,
    /// @brief open about dialog
    MID_HOTKEY_F12_ABOUT,

    /// @}

    /// @name shift + Functional hotkeys
    /// @{

    /// @brief set template
    MID_HOTKEY_SHIFT_F1_TEMPLATE_SET,
    /// @brief copy template
    MID_HOTKEY_SHIFT_F2_TEMPLATE_COPY,
    /// @brief clear template
    MID_HOTKEY_SHIFT_F3_TEMPLATE_CLEAR,
    /// @brief compute junctions with volatile options
    MID_HOTKEY_SHIFT_F5_COMPUTEJUNCTIONS_VOLATILE,
    /// @brief Adjust person plans (start und end positions, arrival positions, etc.)
    MID_HOTKEY_SHIFT_F7_ADJUST_PERSON_PLANS,
    /// @brief open SUMO options menu (used in netedit)
    MID_HOTKEY_SHIFT_F10_SUMOOPTIONSMENU,
    /// @brief focus upper element of current frame (only used in netedit)
    MID_HOTKEY_SHIFT_F12_FOCUSUPPERELEMENT,
    /// @brief clear selection
    MID_HOTKEY_SHIFT_ESC_CLEARSELECTION,

    /// @}

    /// @name Other hotkeys
    /// @{

    /// @brief hot key <ESC> abort current edit operation
    MID_HOTKEY_ESC,
    /// @brief hot key <DEL> delete selections or elements
    MID_HOTKEY_DEL,
    /// @brief hot key <ENTER> accept current operation
    MID_HOTKEY_ENTER,
    /// @brief hot key <Backspace> remove last sub-operation
    MID_HOTKEY_BACKSPACE,

    /// @}

    /// @name application specific
    /// @{

    /// @brief Main window-ID
    MID_WINDOW,

    /// @}

    /// @name for common buttons
    /// @{

    /// @brief Ok-button pressed
    MID_OK,
    /// @brief Cancel-button pressed
    MID_CANCEL,
    /// @brief Update-button pressed
    MID_UPDATE,

    /// @}

    /// @name Main Window File Menu - IDs
    /// @{

    /// @brief Loads a file previously loaded
    MID_RECENTFILE,

    /// @}

    /// @name Main Window Simulation Control - IDs
    /// @{

    /// @brief Save state to file
    MID_SIMSAVE,
    /// @brief (quick)-load state from file
    MID_SIMLOAD,

    /// @}

    /// @name Main window windows control - IDs
    /// @{

    /// @brief Open a new microscopic view
    MID_NEW_MICROVIEW,
    /// @brief Open a new microscopic 3D view
    MID_NEW_OSGVIEW,

    /// @}

    /// @name SubThread - IDs
    /// @{

    /// @brief The loading thread
    ID_LOADTHREAD_EVENT,
    /// @brief The testing thread
    ID_RUNTHREAD_EVENT,
    /// @brief ID for message passing between threads
    ID_THREAD_EVENT,

    /// @}

    /// @name Settings Menu Item - IDs
    /// @{

    /// @brief Locator configuration - menu entry
    MID_LISTINTERNAL,
    MID_LISTPARKING,
    MID_LISTTELEPORTING,
    /// @brief The Simulation delay control
    MID_SIMDELAY,

    /// @}

    /// @brief Clear simulation output
    MID_CLEARMESSAGEWINDOW,
    /// @brief Show network statistics
    MID_SHOWNETSTATS,
    /// @brief Show vehicle statistics
    MID_SHOWVEHSTATS,
    /// @brief Show person statistics
    MID_SHOWPERSONSTATS,
    /// @brief update traci status
    MID_TRACI_STATUS,

    /// @name Common View Settings - IDs
    /// @{

    /// @brief Recenter view - button
    MID_RECENTERVIEW,
    /// @brief Allow rotation - button
    MID_ALLOWROTATION,
    /// @brief Show tool tips in view - button
    MID_SHOWTOOLTIPS_VIEW,
    /// @brief Show tool tips in menus - button
    MID_SHOWTOOLTIPS_MENU,
    /// @brief Make snapshot - button
    MID_MAKESNAPSHOT,
    /// @brief toggle zooming style
    MID_ZOOM_STYLE,
    /// @brief toggle time display mode
    MID_TIME_TOGGLE,
    /// @brief increase sim delay
    MID_DELAY_INC,
    /// @brief decrease sim delay
    MID_DELAY_DEC,
    /// @brief toggle delay between alternative value
    MID_DELAY_TOGGLE,
    /// @brief scale traffic
    MID_DEMAND_SCALE,
    /// @brief scale vehicle speed
    MID_SPEEDFACTOR,

    /// @}

    /// @name Common view IDs
    /// @{

    /// @brief GLCanvas - ID
    MID_GLCANVAS,
    /// @brief chore
    MID_CHORE,

    /// @}


    /// @name View settings - IDs
    /// @{

    /// @brief Change coloring scheme - combo
    MID_COLORSCHEMECHANGE,

    /// @}


    /// @name GUICursorDialog PopUp Entries - IDs
    /// @{
    /// @brief set element as front element
    MID_CURSORDIALOG_SETFRONTELEMENT,
    /// @brief delete element
    MID_CURSORDIALOG_DELETEELEMENT,
    /// @brief select element
    MID_CURSORDIALOG_SELECTELEMENT,
    /// @brief open properties popup
    MID_CURSORDIALOG_PROPERTIES,
    /// @brief move list up
    MID_CURSORDIALOG_MOVEUP,
    /// @brief move list down
    MID_CURSORDIALOG_MOVEDOWN,
    /// @brief remove/select/mark front element
    MID_CURSORDIALOG_FRONT,

    /// @}


    /// @name Object PopUp Entries - IDs
    /// @{

    /// @brief Center view to object - popup entry
    MID_CENTER,
    /// @brief Copy object name - popup entry
    MID_COPY_NAME,
    /// @brief Copy typed object name - popup entry
    MID_COPY_TYPED_NAME,
    /// @brief Copy test coordinates
    MID_COPY_TEST_COORDINATES,
    /// @brief Copy edge name (for lanes only)
    MID_COPY_EDGE_NAME,
    /// @brief Copy cursor position - popup entry
    MID_COPY_CURSOR_POSITION,
    /// @brief Copy cursor geo-coordinate position - popup entry
    MID_COPY_CURSOR_GEOPOSITION,
    /// @brief Copy view geo-coordinate boundary - popup entry
    MID_COPY_VIEW_GEOBOUNDARY,
    /// @brief Show the cursor geo-coordinate position online in GeoHack - popup entry
    MID_SHOW_GEOPOSITION_ONLINE,
    /// @brief open additional dialog (used in netedit)
    MID_OPEN_ADDITIONAL_DIALOG,
    /// @brief Show object parameter - popup entry
    MID_SHOWPARS,
    /// @brief Show object type parameter - popup entry
    MID_SHOWTYPEPARS,
    /// @brief Show transportable plan
    MID_SHOWPLAN,
    /// @brief Show TLS phases - popup entry
    MID_SWITCH_OFF,
    MID_SWITCH,
    MID_SHOWPHASES = MID_SWITCH + 20,
    /// @brief Begin to track phases - menu entry
    MID_TRACKPHASES,
    /// @brief show/hide actuation detectors
    MID_SHOW_DETECTORS,
    /// @brief Add to selected items - menu entry
    MID_ADDSELECT,
    /// @brief Remove from selected items - Menu Entry
    MID_REMOVESELECT,
    /// @brief Open the object's manipulator
    MID_MANIP,
    /// @brief Draw a vehicle's route
    MID_DRAWROUTE,
    /// @brief Show vehicle's current route
    MID_SHOW_CURRENTROUTE,
    /// @brief Hide vehicle's current route
    MID_HIDE_CURRENTROUTE,
    /// @brief Show vehicle's future route
    MID_SHOW_FUTUREROUTE,
    /// @brief Hide vehicle's future route
    MID_HIDE_FUTUREROUTE,
    /// @brief Show vehicle's future route (without loops)
    MID_SHOW_ROUTE_NOLOOPS,
    /// @brief Hide vehicle's future route (without loops)
    MID_HIDE_ROUTE_NOLOOPS,
    /// @brief Show vehicle's best lanes
    MID_SHOW_BEST_LANES,
    /// @brief Hide vehicle's best lanes
    MID_HIDE_BEST_LANES,
    /// @brief Show all vehicle's routes
    MID_SHOW_ALLROUTES,
    /// @brief Hide all vehicle's routes
    MID_HIDE_ALLROUTES,
    /// @brief Start to track a vehicle
    MID_START_TRACK,
    /// @brief Stop to track a vehicle
    MID_STOP_TRACK,
    /// @brief select foes of a vehicle
    MID_SHOW_FOES,
    MID_SHOW_LFLINKITEMS,
    MID_HIDE_LFLINKITEMS,
    /// @brief select transportables of a vehicle
    MID_SELECT_TRANSPORTED,
    /// @brief toggle stop state of a vehicle or person
    MID_TOGGLE_STOP,
    /// @brief remove a vehicle or person
    MID_REMOVE_OBJECT,
    /// @brief Show persons's path on walkingarea
    MID_SHOW_WALKINGAREA_PATH,
    /// @brief Hide persons's path on walkingarea
    MID_HIDE_WALKINGAREA_PATH,
    /// @brief show reachability from a given lane
    MID_REACHABILITY,

    /// @}

    /// @name IDs used by Trackers
    /// @{

    /// @brief The Table
    MID_TABLE,
    /// @brief A Simulation step was performed
    MID_SIMSTEP,
    /// @brief A Tracker shall be opened
    MID_OPENTRACKER,

    /// @}

    /// @name General Setting Dialogs - IDs
    /// @{

    /// @brief Ok-button was pushed
    MID_SETTINGS_OK,
    /// @brief Cancel-button was pushed
    MID_SETTINGS_CANCEL,

    /// @}

    /// @name Application Settings - Dialog IDs
    /// @{

    /// @brief Close simulation at end - Option
    MID_QUITONSIMEND,
    /// @brief Start simulation when loaded - Option
    MID_AUTOSTART,
    /// @brief Demo mode - Option
    MID_DEMO,
    /// @brief Allow textures - Option
    MID_ALLOWTEXTURES,
    /// @brief Locate links in messages - Option
    MID_LOCATELINKS,
    /// @brief Set breakpionts from messages - Option
    MID_TIMELINK_BREAKPOINT,

    /// @}

    /// @name GLObjChooser - Dialog IDs
    /// @{

    /// @brief Center object
    MID_CHOOSER_CENTER,
    /// @brief Track object
    MID_CHOOSER_TRACK,
    /// @brief Text entry
    MID_CHOOSER_TEXT,
    /// @brief Object list
    MID_CHOOSER_LIST,
    /// @brief Filter selected
    MID_CHOOSER_FILTER,
    /// @brief Filter list by substring
    MID_CHOOSER_FILTER_SUBSTR,

    /// @}

    /// @name GLChosenEditor - Dialog IDs
    /// @{

    /// @brief set type of selection
    MID_CHOOSEN_OPERATION,
    /// @brief set subset of elements
    MID_CHOOSEN_ELEMENTS,
    /// @brief Load set
    MID_CHOOSEN_LOAD,
    /// @brief Save set
    MID_CHOOSEN_SAVE,
    /// @brief Clear set
    MID_CHOOSEN_CLEAR,
    /// @brief delete set
    MID_CHOOSEN_DELETE,
    /// @brief Reset set
    MID_CHOOSEN_RESET,
    /// @brief select all items
    MID_CHOOSEN_SELECT,
    /// @brief Deselect selected items
    MID_CHOOSEN_DESELECT,
    /// @brief Deselect selected items
    MID_CHOOSEN_INVERT,
    /// @brief Deselect selected items
    MID_CHOOSEN_NAME,
    /// @brief simplify network reduction
    MID_CHOOSEN_REDUCE,

    /// @}

    /// @name IDs for visualization settings - Dialog
    /// @{

    /// @brief Informs the dialog about a value's change
    MID_SIMPLE_VIEW_COLORCHANGE,
    /// @brief Informs the dialog about a size value's change
    MID_SIMPLE_VIEW_SIZECHANGE,
    /// @brief Informs the dialog about switching to another scheme
    MID_SIMPLE_VIEW_NAMECHANGE,
    /// @brief For the save-to-db - button
    MID_SIMPLE_VIEW_SAVE,
    /// @brief For the delete - button
    MID_SIMPLE_VIEW_DELETE,
    /// @brief For the export-to-file - button
    MID_SIMPLE_VIEW_EXPORT,
    /// @brief For the import-from-file - button
    MID_SIMPLE_VIEW_IMPORT,
    /// @brief For the load-decals - button
    MID_SIMPLE_VIEW_LOAD_DECAL,
    /// @brief For the load-decals - button
    MID_SIMPLE_VIEW_LOAD_DECALS_XML,
    /// @brief For the save-decals - button
    MID_SIMPLE_VIEW_SAVE_DECALS_XML,
    /// @brief For the clear-decals - button
    MID_SIMPLE_VIEW_CLEAR_DECALS,
    /// @brief For changing rainbow style
    MID_SIMPLE_VIEW_RAINBOW_CHANGE,
    /// @}

    ///@brief help button
    MID_HELP,
    ///@brief changelog button
    MID_CHANGELOG,
    ///@brief hotkeys button
    MID_HOTKEYS,
    ///@brief tutorial button
    MID_TUTORIAL,
    ///@brief feedback button
    MID_FEEDBACK,

    /// @name dynamic interaction with the simulation via SUMO-GUI
    /// @{

    /// @brief close lane
    MID_CLOSE_LANE,
    /// @brief close edge
    MID_CLOSE_EDGE,
    /// @brief add rerouter
    MID_ADD_REROUTER,
    /// @brief toggle detector override
    MID_VIRTUAL_DETECTOR,

    /// @}

    /// @name toolbar edot
    /// @{

    /// @brief load additionals in sumo-gui/netedit after press ctrl+T
    MID_TOOLBAREDIT_LOADADDITIONALS,
    /// @brief load demand in sumo-gui/netedit after press ctrl+T
    MID_TOOLBAREDIT_LOADDEMAND,

    /// @}

    /// @name groupBoxModule messages
    /// @{
    /// @brief collapse groupBoxModule
    MID_GROUPBOXMODULE_COLLAPSE,
    /// @brief extends groupBoxModule
    MID_GROUPBOXMODULE_EXTEND,
    /// @brief reset widh groupBoxModule
    MID_GROUPBOXMODULE_RESETWIDTH,
    /// @brief save contents
    MID_GROUPBOXMODULE_SAVE,
    /// @brief load contents
    MID_GROUPBOXMODULE_LOAD,
    /// @}

    /// @name Toolbar file messages
    /// @{

    /// @brief reload only network
    MID_GNE_TOOLBARFILE_RELOADNETWORK,
    /// @brief open foreign network
    MID_GNE_TOOLBARFILE_OPENFOREIGN,
    /// @brief save joined junctions
    MID_GNE_SAVEJOINEDJUNCTIONS,
    /// @brief reload neteditConfig
    MID_GNE_TOOLBARFILE_RELOAD_NETEDITCONFIG,
    /// @brief save neteditConfig as
    MID_GNE_TOOLBARFILE_SAVENETEDITCONFIG_AS,
    /// @brief reload SUMOConfig
    MID_GNE_TOOLBARFILE_RELOAD_SUMOCONFIG,
    /// @brief save SUMOConfig as
    MID_GNE_TOOLBARFILE_SAVESUMOCONFIG_AS,
    /// @brief save network as
    MID_GNE_TOOLBARFILE_SAVENETWORK_AS,
    /// @brief save additionals unified
    MID_GNE_TOOLBARFILE_SAVEADDITIONALELEMENTS_UNIFIED,
    /// @brief save JuPedSim as
    MID_GNE_TOOLBARFILE_SAVEJUPEDSIMELEMENTS_AS,
    /// @brief reload additionals
    MID_GNE_TOOLBARFILE_RELOAD_ADDITIONALELEMENTS,
    /// @brief save TLS Programs as
    MID_GNE_TOOLBARFILE_SAVETLSPROGRAMS_AS,
    /// @brief reload TLS Programs
    MID_GNE_TOOLBARFILE_RELOAD_TLSPROGRAMS,
    /// @brief save edgeTypes as
    MID_GNE_TOOLBARFILE_SAVEEDGETYPES_AS,
    /// @brief reload edge types
    MID_GNE_TOOLBARFILE_RELOAD_EDGETYPES,
    /// @brief save demand elements unified
    MID_GNE_TOOLBARFILE_SAVEDEMANDELEMENTS_UNIFIED,
    /// @brief reload demand elements
    MID_GNE_TOOLBARFILE_RELOAD_DEMANDELEMENTS,
    /// @brief save data elements unified
    MID_GNE_TOOLBARFILE_SAVEDATAELEMENTS_UNIFIED,
    /// @brief reload data elements
    MID_GNE_TOOLBARFILE_RELOAD_DATAELEMENTS,
    /// @brief open meanData file
    MID_GNE_TOOLBARFILE_OPENMEANDATAELEMENTS,
    /// @brief save meanDatas unified
    MID_GNE_TOOLBARFILE_SAVEMEANDATAELEMENTS_UNIFIED,
    /// @brief reload meanDatas
    MID_GNE_TOOLBARFILE_RELOAD_MEANDATAELEMENTS,

    /// @}

    /// @name Toolbar tools messages
    /// @{

    /// @brief call tool
    MID_GNE_OPENPYTHONTOOLDIALOG,
    /// @brief run python
    MID_GNE_RUNPYTHONTOOL,
    /// @brief call tool for post processing
    MID_GNE_POSTPROCESSINGPYTHONTOOL,
    /// @brief netgenerate dialog
    MID_GNE_NETGENERATE,
    /// @brief netgenerate options
    MID_GNE_NETGENERATEOPTIONS,
    /// @brief netgenerate grid network
    MID_GNE_NETGENERATE_GRID,
    /// @brief netgenerate spider network
    MID_GNE_NETGENERATE_SPIDER,
    /// @brief netgenerate random grid network
    MID_GNE_NETGENERATE_RANDOMGRID,
    /// @brief netgenerate random network
    MID_GNE_NETGENERATE_RANDOM,
    /// @brief run netgenerate tool
    MID_GNE_RUNNETGENERATE,
    /// @brief postprocesing netgenerate
    MID_GNE_POSTPROCESSINGNETGENERATE,

    /// @}

    /// @name Toolbar view messages
    /// @{

    /// @brief set default view
    MID_GNE_VIEW_DEFAULT,
    /// @brief set juPedSim view
    MID_GNE_VIEW_JUPEDSIM,

    /// @}

    /// @name Toolbar processing messages
    /// @{

    /// @brief compute path manager
    MID_GNE_TOOLBAREDIT_COMPUTEPATHMANAGER,
    /// @brief enable/disable computing after switchin between supermodes
    MID_GNE_TOGGLE_COMPUTE_NETWORK_DATA,
    /// @brief enable/disable undo-redo
    MID_GNE_TOGGLE_UNDOREDO,
    /// @brief enable/disable undo-redo during loading
    MID_GNE_TOGGLE_UNDOREDO_LOADING,
    /// @brief switch time format
    MID_GNE_TOGGLE_TIMEFORMAT,

    /// @}

    /// @name GNEViewParent dialog messages
    /// @{

    /// @brief Size of frame area updated
    MID_GNE_VIEWPARENT_FRAMEAREAWIDTH,

    /// @}

    /// @name GNEViewNet Network view options messages
    /// @{

    /// @brief show grid
    MID_GNE_NETWORKVIEWOPTIONS_TOGGLEGRID,
    /// @brief toggle draw junction shape
    MID_GNE_NETWORKVIEWOPTIONS_TOGGLEDRAWJUNCTIONSHAPE,
    /// @brief Draw vehicles in begin position or spread in lane
    MID_GNE_NETWORKVIEWOPTIONS_DRAWSPREADVEHICLES,
    /// @brief show demand elements
    MID_GNE_NETWORKVIEWOPTIONS_SHOWDEMANDELEMENTS,
    /// @brief select edges
    MID_GNE_NETWORKVIEWOPTIONS_SELECTEDGES,
    /// @brief show connections
    MID_GNE_NETWORKVIEWOPTIONS_SHOWCONNECTIONS,
    /// @brief hide connections
    MID_GNE_NETWORKVIEWOPTIONS_HIDECONNECTIONS,
    /// @brief show sub-additionals
    MID_GNE_NETWORKVIEWOPTIONS_SHOWSUBADDITIONALS,
    /// @brief show TAZ elements
    MID_GNE_NETWORKVIEWOPTIONS_SHOWTAZELEMENTS,
    /// @brief extend selection
    MID_GNE_NETWORKVIEWOPTIONS_EXTENDSELECTION,
    /// @brief change all phases
    MID_GNE_NETWORKVIEWOPTIONS_CHANGEALLPHASES,
    /// @brief don't ask before merging junctions
    MID_GNE_NETWORKVIEWOPTIONS_MERGEAUTOMATICALLY,
    /// @brief show junctions as bubbles
    MID_GNE_NETWORKVIEWOPTIONS_SHOWBUBBLES,
    /// @brief move elevation instead of x,y
    MID_GNE_NETWORKVIEWOPTIONS_MOVEELEVATION,
    /// @brief create edges in chain mode
    MID_GNE_NETWORKVIEWOPTIONS_CHAINEDGES,
    /// @brief automatically create opposite edge
    MID_GNE_NETWORKVIEWOPTIONS_AUTOOPPOSITEEDGES,
    /// @brief Add edge to selected items - menu entry
    MID_GNE_ADDSELECT_EDGE,
    /// @brief Remove edge from selected items - Menu Entry
    MID_GNE_REMOVESELECT_EDGE,
    /// @brief lock element
    MID_GNE_LOCK_ELEMENT,
    /// @brief lock all element
    MID_GNE_LOCK_ALLELEMENTS,
    /// @brief unlock all element
    MID_GNE_UNLOCK_ALLELEMENTS,
    /// @brief lock selected element
    MID_GNE_LOCK_SELECTEDELEMENTS,

    /// @}

    /// @name GNEViewNet Demand view options messages
    /// @{

    /// @brief show grid
    MID_GNE_DEMANDVIEWOPTIONS_SHOWGRID,
    /// @brief toggle draw junction shape
    MID_GNE_DEMANDVIEWOPTIONS_TOGGLEDRAWJUNCTIONSHAPE,
    /// @brief Draw vehicles in begin position or spread in lane
    MID_GNE_DEMANDVIEWOPTIONS_DRAWSPREADVEHICLES,
    /// @brief hide non-inspected demand element
    MID_GNE_DEMANDVIEWOPTIONS_HIDENONINSPECTED,
    /// @brief hide shapes
    MID_GNE_DEMANDVIEWOPTIONS_HIDESHAPES,
    /// @brief show all trips
    MID_GNE_DEMANDVIEWOPTIONS_SHOWTRIPS,
    /// @brief show all person plans
    MID_GNE_DEMANDVIEWOPTIONS_SHOWALLPERSONPLANS,
    /// @brief lock person
    MID_GNE_DEMANDVIEWOPTIONS_LOCKPERSON,
    /// @brief show all container plans
    MID_GNE_DEMANDVIEWOPTIONS_SHOWALLCONTAINERPLANS,
    /// @brief lock container
    MID_GNE_DEMANDVIEWOPTIONS_LOCKCONTAINER,
    /// @brief show overlapped routes
    MID_GNE_DEMANDVIEWOPTIONS_SHOWOVERLAPPEDROUTES,

    /// @}

    /// @name GNEViewNet Data view options messages
    /// @{

    /// @brief toggle draw junction shape
    MID_GNE_DATAVIEWOPTIONS_TOGGLEDRAWJUNCTIONSHAPE,
    /// @brief show additionals
    MID_GNE_DATAVIEWOPTIONS_SHOWADDITIONALS,
    /// @brief show shapes
    MID_GNE_DATAVIEWOPTIONS_SHOWSHAPES,
    /// @brief show demand elements
    MID_GNE_DATAVIEWOPTIONS_SHOWDEMANDELEMENTS,
    /// @brief toggle TAZRel drawing
    MID_GNE_DATAVIEWOPTIONS_TAZRELDRAWING,
    /// @brief toggle draw TAZ fill
    MID_GNE_DATAVIEWOPTIONS_TAZDRAWFILL,
    /// @brief toggle draw TAZRel only from
    MID_GNE_DATAVIEWOPTIONS_TAZRELONLYFROM,
    /// @brief toggle draw TAZRel only to
    MID_GNE_DATAVIEWOPTIONS_TAZRELONLYTO,

    /// @}

    /// @name GNEViewNet interval bar messages
    /// @{

    /// @brief generic data selected
    MID_GNE_INTERVALBAR_GENERICDATATYPE,
    /// @brief data set selected
    MID_GNE_INTERVALBAR_DATASET,
    /// @brief enable/disable show data elements by interval
    MID_GNE_INTERVALBAR_LIMITED,
    /// @brief begin changed in InterbalBar
    MID_GNE_INTERVALBAR_BEGIN,
    /// @brief end changed in InterbalBar
    MID_GNE_INTERVALBAR_END,
    /// @brief parameter changed in InterbalBar
    MID_GNE_INTERVALBAR_PARAMETER,

    /// @}

    /// @name GNEFrame messages
    /// @{

    /// @brief attribute added
    MID_GNE_ADD_ATTRIBUTE,
    /// @brief attribute removed
    MID_GNE_REMOVE_ATTRIBUTE,
    /// @brief attribute edited
    MID_GNE_SET_ATTRIBUTE,
    /// @brief create element
    MID_GNE_CREATE,
    /// @brief delete element
    MID_GNE_DELETE,
    /// @brief inspect element
    MID_GNE_INSPECT,
    /// @brief reset element
    MID_GNE_RESET,
    /// @brief apply element
    MID_GNE_APPLY,
    /// @brief center element
    MID_GNE_CENTER,
    /// @brief copy element
    MID_GNE_COPY,
    /// @brief open element
    MID_GNE_OPEN,
    /// @brief select element
    MID_GNE_SELECT,
    /// @brief used to select a type of element in a combo box
    MID_GNE_SET_TYPE,
    /// @brief tag type selected in ComboBox
    MID_GNE_TAGTYPE_SELECTED,
    /// @brief tag selected in ComboBox
    MID_GNE_TAG_SELECTED,
    /// @brief create new data set
    MID_GNE_DATASET_NEW,
    /// @brief GNEDataInterval selected in comboBox of IntervalBar
    MID_GNE_DATASET_SELECTED,
    /// @brief GNEDataInterval selected in comboBox of IntervalBar
    MID_GNE_DATAINTERVAL_SELECTED,
    /// @brief text attribute edited
    MID_GNE_SET_ATTRIBUTE_TEXT,
    /// @brief bool attribute edited
    MID_GNE_SET_ATTRIBUTE_BOOL,
    /// @brief attribute edited trough dialog
    MID_GNE_SET_ATTRIBUTE_DIALOG,
    /// @brief attribute selected using button (radio button or checkbox)
    MID_GNE_SET_ATTRIBUTE_BUTTON,
    /// @brief abort edge path creation
    MID_GNE_PATHCREATOR_ABORT,
    /// @brief finish edge path creation
    MID_GNE_PATHCREATOR_FINISH,
    /// @brief use last inserted route
    MID_GNE_PATHCREATOR_USELASTROUTE,
    /// @brief remove last inserted element in path
    MID_GNE_PATHCREATOR_REMOVELAST,
    /// @brief enable or disable show path candidates
    MID_GNE_PATHCREATOR_SHOWCANDIDATES,
    /// @brief abort lane path creation
    MID_GNE_ABORT,
    /// @brief finish lane path creation
    MID_GNE_FINISH,
    /// @brief remove last inserted element in path
    MID_GNE_REMOVELAST,
    /// @brief enable or disable show path candidates
    MID_GNE_SHOWCANDIDATES,
    /// @brief In GNEElementTree list, show child menu
    MID_GNE_ACHIERARCHY_SHOWCHILDMENU,
    /// @brief In GNEElementTree list, move element to up
    MID_GNE_ACHIERARCHY_MOVEUP,
    /// @brief In GNEElementTree list, move element to down
    MID_GNE_ACHIERARCHY_MOVEDOWN,
    /// @brief start drawing polygon
    MID_GNE_STARTDRAWING,
    /// @brief stop drawing polygon
    MID_GNE_STOPDRAWING,
    /// @brief abort drawing polygon
    MID_GNE_ABORTDRAWING,
    /// @brief inspect next element in overlapped module
    MID_GNE_OVERLAPPED_NEXT,
    /// @brief inspect previous element in overlapped module
    MID_GNE_OVERLAPPED_PREVIOUS,
    /// @brief show list of overlapped elements
    MID_GNE_OVERLAPPED_SHOWLIST,
    /// @brief list item selected in overlapped module
    MID_GNE_OVERLAPPED_ITEMSELECTED,
    /// @brief mark item as front element
    MID_GNE_MARKFRONTELEMENT,
    /// @brief set custom geometry point
    MID_GNE_CUSTOM_GEOMETRYPOINT,
    /// @brief reset geometry point
    MID_GNE_RESET_GEOMETRYPOINT,
    /// @brief set zero fringe probabilities (used in TAZ Frame)
    MID_GNE_SET_ZEROFRINGEPROB,
    /// @brief use current network/additional/route/edgedata
    MID_GNE_USE_CURRENT,
    /// @brief protect all elements
    MID_GNE_PROTECT_ALL,
    /// @brief unprotect all elements
    MID_GNE_UNPROTECT_ALL,
    /// @}

    /// @name GNEAttributesEditorType messages
    /// @{

    /// @brief mark element as front
    MID_GNE_ATTRIBUTESEDITOR_FRONT,
    /// @brief open element dialog
    MID_GNE_ATTRIBUTESEDITOR_DIALOG,
    /// @brief open extended attributes
    MID_GNE_ATTRIBUTESEDITOR_EXTENDED,
    /// @brief open generic parameters editor
    MID_GNE_ATTRIBUTESEDITOR_PARAMETERS,
    /// @brief open help dialog
    MID_GNE_ATTRIBUTESEDITOR_HELP,
    /// @brief reset attributes
    MID_GNE_ATTRIBUTESEDITOR_RESET,
    /// @brief set attribute (string, bool, etc.) in attributes editor row
    MID_GNE_ATTRIBUTESEDITORROW_SETATTRIBUTE,
    /// @brief toogle enable attribute in attributes editor row
    MID_GNE_ATTRIBUTESEDITORROW_TOGGLEENABLEATTRIBUTE,
    /// @brief open color dialog in attributes editor row
    MID_GNE_ATTRIBUTESEDITORROW_OPENDIALOG_COLOR,
    /// @brief open allow dialog in attributes editor row
    MID_GNE_ATTRIBUTESEDITORROW_OPENDIALOG_ALLOW,
    /// @brief open file dialog in attributes editor row
    MID_GNE_ATTRIBUTESEDITORROW_OPENDIALOG_FILE,
    /// @brief reparent
    MID_GNE_ATTRIBUTESEDITORROW_REPARENT,
    /// @brief inspect parent
    MID_GNE_ATTRIBUTESEDITORROW_INSPECTPARENT,
    /// @brief move lane up
    MID_GNE_ATTRIBUTESEDITORROW_MOVELANEUP,
    /// @brief move lane down
    MID_GNE_ATTRIBUTESEDITORROW_MOVELANEDOWN,

    /// @name GNESelectorFrame messages
    /// @{

    /// @brief select tag in selector frame
    MID_GNE_SELECTORFRAME_SELECTTAG,
    /// @brief select attribute in selector frame
    MID_GNE_SELECTORFRAME_SELECTATTRIBUTE,
    /// @brief toogle only show common
    MID_GNE_SELECTORFRAME_TOGGLECOMMON,
    /// @brief process string
    MID_GNE_SELECTORFRAME_PROCESSSTRING,
    /// @brief changes the visual scaling of selected items
    MID_GNE_SELECTORFRAME_SELECTSCALE,
    /// @brief change interval
    MID_GNE_SELECTORFRAME_SETINTERVAL,
    /// @brief edit begin text field
    MID_GNE_SELECTORFRAME_SETBEGIN,
    /// @brief end text field
    MID_GNE_SELECTORFRAME_SETEND,
    /// @brief from TAZ text field
    MID_GNE_SELECTORFRAME_FROMTAZ,
    /// @brief to TAZ field
    MID_GNE_SELECTORFRAME_TOTAZ,
    /// @brief select/unselect parents
    MID_GNE_SELECTORFRAME_PARENTS,
    /// @brief select/unselect children
    MID_GNE_SELECTORFRAME_CHILDREN,

    /// @}

    /// @name GNEConnectorFrame messages
    /// @{

    /// @brief select dead end lanes
    MID_GNE_CONNECTORFRAME_SELECTDEADENDS,
    /// @brief select lanes that have no connection leading to it
    MID_GNE_CONNECTORFRAME_SELECTDEADSTARTS,
    /// @brief select lanes that are connected from concurrent lanes
    MID_GNE_CONNECTORFRAME_SELECTCONFLICTS,
    /// @brief select lanes with connections that have the pass attribute set to 'true'
    MID_GNE_CONNECTORFRAME_SELECTPASS,

    /// @}

    /// @name GNEInspectorFrame messages
    /// @{

    /// @brief go back to the previous element
    MID_GNE_INSPECTORFRAME_INSPECTPREVIOUSELEMENT,

    /// @}

    /// @name GNECreateEdgeFrame messages
    /// @{
    /// @brief selected radio button
    MID_GNE_CREATEEDGEFRAME_SELECTRADIOBUTTON,
    /// @brief add new edge type
    MID_GNE_CREATEEDGEFRAME_ADD,
    /// @brief delete edge type
    MID_GNE_CREATEEDGEFRAME_DELETE,
    /// @brief select default edge type
    MID_GNE_CREATEEDGEFRAME_SELECTDEFAULT,
    /// @brief select edge template
    MID_GNE_CREATEEDGEFRAME_SELECTTEMPLATE,
    /// @brief select lane (used for default and template edges)
    MID_GNE_CREATEEDGEFRAME_SELECTLANE,
    /// @brief create edge type from template
    MID_GNE_CREATEEDGEFRAME_CREATEFROMTEMPLATE,
    /// @brief check button
    MID_GNE_CREATEEDGEFRAME_CHECKBUTTON,

    /// @}

    /// @name GNETLSEditorFrame messages
    /// @{

    /// @brief current TLS ID
    MID_GNE_TLSFRAME_TLSJUNCTION_ID,
    /// @brief current TLS ID
    MID_GNE_TLSFRAME_TLSJUNCTION_TYPE,
    /// @brief join TLS
    MID_GNE_TLSFRAME_TLSJUNCTION_TOGGLEJOIN,
    /// @brief Disjoin TLS
    MID_GNE_TLSFRAME_TLSJUNCTION_DISJOIN,
    /// @brief replace program with a newly guessed program
    MID_GNE_TLSFRAME_ATTRIBUTES_GUESSPROGRAM,
    /// @brief TLS offset
    MID_GNE_TLSFRAME_ATTRIBUTES_OFFSET,
    /// @brief TLS parameters
    MID_GNE_TLSFRAME_ATTRIBUTES_PARAMETERS,
    /// @brief TLS parameters
    MID_GNE_TLSFRAME_ATTRIBUTES_PARAMETERSDIALOG,
    /// @brief set detectors in TLS
    MID_GNE_TLSFRAME_ATTRIBUTES_TOGGLEDETECTOR,
    /// @brief Create TLS
    MID_GNE_TLSFRAME_DEFINITION_CREATE,
    /// @brief delete TLS
    MID_GNE_TLSFRAME_DEFINITION_DELETE,
    /// @brief reset current (single) TLS program
    MID_GNE_TLSFRAME_DEFINITION_RESETCURRENT,
    /// @brief reset all TLS programs
    MID_GNE_TLSFRAME_DEFINITION_RESETALL,
    /// @brief switch between programs
    MID_GNE_TLSFRAME_DEFINITION_SWITCHPROGRAM,
    /// @brief accept TLS modification
    MID_GNE_TLSFRAME_DEFINITION_SAVE,
    /// @brief cancel TLS modification
    MID_GNE_TLSFRAME_DEFINITION_DISCARD,
    /// @brief rename TLS
    MID_GNE_TLSFRAME_ATTRIBUTES_RENAME,
    /// @brief sub-rename TLS
    MID_GNE_TLSFRAME_ATTRIBUTES_SUBRENAME,
    /// @brief add off to TLS
    MID_GNE_TLSFRAME_ATTRIBUTES_ADDOFF,
    /// @brief create phase thable
    MID_GNE_TLSFRAME_PHASE_CREATE,
    /// @brief delete  phase thable
    MID_GNE_TLSFRAME_PHASE_DELETE,
    /// @brief cleanup unused states
    MID_GNE_TLSFRAME_PHASES_CLEANUP,
    /// @brief mark unused states
    MID_GNE_TLSFRAME_MARKUNUSED,
    /// @brief add unused states
    MID_GNE_TLSFRAME_PHASES_ADDUNUSED,
    /// @brief group states
    MID_GNE_TLSFRAME_PHASES_GROUPSTATES,
    /// @brief ungroup states
    MID_GNE_TLSFRAME_PHASES_UNGROUPSTATES,
    /// @brief Load Program
    MID_GNE_TLSFRAME_FILE_LOADPROGRAM,
    /// @brief cleanup unused states
    MID_GNE_TLSFRAME_FILE_SAVEPROGRAM,

    /// @}

    /// @name GNEAdditionalFrame messages
    /// @{

    /// @brief search element
    MID_GNE_SEARCH,
    /// @brief use selected elements
    MID_GNE_USESELECTED,
    /// @brief clear selection of elements
    MID_GNE_CLEARSELECTION,
    /// @brief stop selection of consecutive edges/lanes
    MID_GNE_STOPSELECTION,
    /// @brief abort selection of consecutive edges/lanes
    MID_GNE_ABORTSELECTION,

    /// @}

    /// @name GNEProhibitionFrame messages
    /// @{

    /// @brief select prohibiting connection
    MID_GNE_PROHIBITIONFRAME_SELECTPROHIBITING,
    /// @brief select connections to be prohibited
    MID_GNE_PROHIBITIONFRAME_SELECTPROHIBITED,
    /// @brief abort drawing polygon
    MID_GNE_PROHIBITIONFRAME_ABORTDRAWING,

    /// @}

    /// @name GNERoute messages
    /// @{

    /// @brief select a route mode
    MID_GNE_ROUTEFRAME_ROUTEMODE,
    /// @brief select a VClass
    MID_GNE_ROUTEFRAME_VCLASS,

    /// @}

    /// @name GNEEdge messages
    /// @{

    /// @brief change default geometry endpoints
    MID_GNE_EDGE_EDIT_ENDPOINT,
    /// @brief reset default geometry endpoints
    MID_GNE_EDGE_RESET_ENDPOINT,
    /// @brief remove inner geometry
    MID_GNE_EDGE_STRAIGHTEN,
    /// @brief smooth geometry
    MID_GNE_EDGE_SMOOTH,
    /// @brief interpolate z values linear between junctions
    MID_GNE_EDGE_STRAIGHTEN_ELEVATION,
    /// @brief smooth elevation with regard to adjoining edges
    MID_GNE_EDGE_SMOOTH_ELEVATION,
    /// @brief split an edge
    MID_GNE_EDGE_SPLIT,
    /// @brief split an edge
    MID_GNE_EDGE_SPLIT_BIDI,
    /// @brief reverse an edge
    MID_GNE_EDGE_REVERSE,
    /// @brief add reverse edge
    MID_GNE_EDGE_ADD_REVERSE,
    /// @brief add reverse edge disconnected (used for for spreadtype center)
    MID_GNE_EDGE_ADD_REVERSE_DISCONNECTED,
    /// @brief reset custom lengths
    MID_GNE_EDGE_RESET_LENGTH,
    /// @brief use edge as tempalte
    MID_GNE_EDGE_USEASTEMPLATE,
    /// @brief apply template
    MID_GNE_EDGE_APPLYTEMPLATE,

    /// @}

    /// @name GNEJunction messages
    /// @{

    /// @brief clear junction's connections
    MID_GNE_JUNCTION_CLEAR_CONNECTIONS,
    /// @brief reset junction's connections
    MID_GNE_JUNCTION_RESET_CONNECTIONS,
    /// @brief turn junction into geometry node
    MID_GNE_JUNCTION_REPLACE,
    /// @brief turn junction into multiple junctions
    MID_GNE_JUNCTION_SPLIT,
    /// @brief turn junction into multiple junctions and reconnect them heuristically
    MID_GNE_JUNCTION_SPLIT_RECONNECT,
    /// @brief edit junction shape
    MID_GNE_JUNCTION_EDIT_SHAPE,
    /// @brief reset junction shape
    MID_GNE_JUNCTION_RESET_SHAPE,
    /// @brief reset edge endpoints
    MID_GNE_JUNCTION_RESET_EDGE_ENDPOINTS,
    /// @brief select all roundabout nodes and edges of the current roundabout
    MID_GNE_JUNCTION_SELECT_ROUNDABOUT,
    /// @brief convert junction to roundabout
    MID_GNE_JUNCTION_CONVERT_ROUNDABOUT,
    /// @brief Add TLS into junction
    MID_GNE_JUNCTION_ADDTLS,
    /// @brief Add join TLS into junctions
    MID_GNE_JUNCTION_ADDJOINTLS,

    /// @}

    /// @name GNEConnection, GNECrossings and GNEWalkingAreas messages
    /// @{

    /// @brief edit connection shape
    MID_GNE_CONNECTION_EDIT_SHAPE,
    /// @ brief smooth connection shape
    MID_GNE_CONNECTION_SMOOTH_SHAPE,
    /// @brief edit crossing shape
    MID_GNE_CROSSING_EDIT_SHAPE,
    /// @brief edit crossing shape
    MID_GNE_WALKINGAREA_EDIT_SHAPE,

    /// @}

    /// @name GNEPoly messages
    /// @{

    /// @brief simplify polygon geometry
    MID_GNE_POLYGON_SIMPLIFY_SHAPE,
    /// @brief close opened polygon
    MID_GNE_POLYGON_CLOSE,
    /// @brief open closed polygon
    MID_GNE_POLYGON_OPEN,
    /// @brief Set a vertex of polygon as first vertex
    MID_GNE_POLYGON_SET_FIRST_POINT,
    /// @brief delete geometry point
    MID_GNE_POLYGON_DELETE_GEOMETRY_POINT,
    /// @brief select elements within polygon boundary
    MID_GNE_POLYGON_SELECT,
    /// @brief triangulate polygon
    MID_GNE_POLYGON_TRIANGULATE,

    /// @}

    /// @name GNEPoly messages
    /// @{

    /// @brief simplify shape edited geometry
    MID_GNE_SHAPEEDITED_SIMPLIFY,
    /// @brief straighten shape edited geometry
    MID_GNE_SHAPEEDITED_STRAIGHTEN,
    /// @brief close opened shape edited
    MID_GNE_SHAPEEDITED_CLOSE,
    /// @brief open closed shape edited
    MID_GNE_SHAPEEDITED_OPEN,
    /// @brief Set a vertex of shape edited as first vertex
    MID_GNE_SHAPEEDITED_SET_FIRST_POINT,
    /// @brief delete geometry point in shape edited
    MID_GNE_SHAPEEDITED_DELETE_GEOMETRY_POINT,
    /// @brief reset shape
    MID_GNE_SHAPEEDITED_RESET,
    /// @brief finish editing shape edited
    MID_GNE_SHAPEEDITED_FINISH,

    /// @}

    /// @name GNEPOI messages
    /// @{

    /// @brief Transform POI to POILane, and viceversa
    MID_GNE_POI_TRANSFORM,

    /// @}

    /// @name GNELane messages
    /// @{

    /// @brief duplicate a lane
    MID_GNE_LANE_DUPLICATE,
    /// @brief edit lane shape
    MID_GNE_LANE_EDIT_SHAPE,
    /// @brief reset custom shape
    MID_GNE_LANE_RESET_CUSTOMSHAPE,
    /// @brief reset opposite lane
    MID_GNE_LANE_RESET_OPPOSITELANE,
    /// @brief transform lane to sidewalk
    MID_GNE_LANE_TRANSFORM_SIDEWALK,
    /// @brief transform lane to bikelane
    MID_GNE_LANE_TRANSFORM_BIKE,
    /// @brief transform lane to busLane
    MID_GNE_LANE_TRANSFORM_BUS,
    /// @brief transform lane to greenVerge
    MID_GNE_LANE_TRANSFORM_GREENVERGE,
    /// @brief add sidewalk
    MID_GNE_LANE_ADD_SIDEWALK,
    /// @brief add bikelane
    MID_GNE_LANE_ADD_BIKE,
    /// @brief add busLane
    MID_GNE_LANE_ADD_BUS,
    /// @brief add greenVerge front of current lane
    MID_GNE_LANE_ADD_GREENVERGE_FRONT,
    /// @brief add greenVerge back of current lane
    MID_GNE_LANE_ADD_GREENVERGE_BACK,
    /// @brief remove sidewalk
    MID_GNE_LANE_REMOVE_SIDEWALK,
    /// @brief remove bikelane
    MID_GNE_LANE_REMOVE_BIKE,
    /// @brief remove busLane
    MID_GNE_LANE_REMOVE_BUS,
    /// @brief remove greenVerge
    MID_GNE_LANE_REMOVE_GREENVERGE,

    /// @}

    /// @name GNEDemandElements messages
    /// @{

    /// @brief apply distance
    MID_GNE_ROUTE_APPLY_DISTANCE,
    /// @brief transform vehicle to trip
    MID_GNE_VEHICLE_TRANSFORM_TRIP,
    /// @brief transform vehicle to flow
    MID_GNE_VEHICLE_TRANSFORM_FLOW,
    /// @brief transform vehicle to vehicle over route
    MID_GNE_VEHICLE_TRANSFORM_VEHICLE,
    /// @brief transform vehicle to flow over route
    MID_GNE_VEHICLE_TRANSFORM_ROUTEFLOW,
    /// @brief transform vehicle to vehicle over junctions
    MID_GNE_VEHICLE_TRANSFORM_VEHICLE_EMBEDDED,
    /// @brief transform vehicle to flow over junctions
    MID_GNE_VEHICLE_TRANSFORM_FLOW_EMBEDDED,
    /// @brief transform vehicle to trip over junctions
    MID_GNE_VEHICLE_TRANSFORM_TRIP_JUNCTIONS,
    /// @brief transform vehicle to flow over jucntions
    MID_GNE_VEHICLE_TRANSFORM_FLOW_JUNCTIONS,
    /// @brief transform vehicle to trip over TAZs
    MID_GNE_VEHICLE_TRANSFORM_TRIP_TAZS,
    /// @brief transform vehicle to flow over TAZs
    MID_GNE_VEHICLE_TRANSFORM_FLOW_TAZS,
    /// @brief transform person to another person type (ej: person to personflow)
    MID_GNE_PERSON_TRANSFORM,
    /// @brief transform container to another container type (ej: container to containerflow)
    MID_GNE_CONTAINER_TRANSFORM,
    /// @brief reverse current element
    MID_GNE_REVERSE,
    /// @brief add reverse element
    MID_GNE_ADDREVERSE,
    /// @}

    /// @name Netedit Dialogs
    /// @{

    /// @brief accept button
    MID_GNE_BUTTON_ACCEPT,
    /// @brief cancel button
    MID_GNE_BUTTON_CANCEL,
    /// @brief reset button
    MID_GNE_BUTTON_RESET,
    /// @brief load button
    MID_GNE_BUTTON_LOAD,
    /// @brief save button
    MID_GNE_BUTTON_SAVE,
    /// @brief clear button
    MID_GNE_BUTTON_CLEAR,
    /// @brief sort button
    MID_GNE_BUTTON_SORT,
    /// @brief focus button
    MID_GNE_BUTTON_FOCUS,
    /// @brief run button
    MID_GNE_BUTTON_RUN,
    /// @brief advanced button
    MID_GNE_BUTTON_ADVANCED,
    /// @brief rerun button
    MID_GNE_BUTTON_RERUN,
    /// @brief abort button
    MID_GNE_BUTTON_ABORT,
    /// @brief back button
    MID_GNE_BUTTON_BACK,
    /// @brief default button
    MID_GNE_BUTTON_DEFAULT,
    /// @brief add button
    MID_GNE_BUTTON_ADD,
    /// @brief remove button
    MID_GNE_BUTTON_REMOVE,

    /// @}

    /// @name Variable Speed Sign Dialog messages
    /// @{

    /// @brief add row
    MID_GNE_VARIABLESPEEDSIGN_ADDROW,
    /// @brief Click over Table
    MID_GNE_VARIABLESPEEDSIGN_TABLE,
    /// @brief sort table values
    MID_GNE_VARIABLESPEEDSIGN_SORT,

    /// @}

    /// @name Rerouter Dialog messages
    /// @{

    /// @brief select table interval
    MID_GNE_REROUTEDIALOG_TABLE_INTERVAL,
    /// @brief change table closing lane reroute reroute
    MID_GNE_REROUTEDIALOG_TABLE_CLOSINGLANEREROUTE,
    /// @brief change table route closing reroute
    MID_GNE_REROUTEDIALOG_TABLE_CLOSINGREROUTE,
    /// @brief change table destination probability reroute
    MID_GNE_REROUTEDIALOG_TABLE_DESTPROBREROUTE,
    /// @brief change table route probability reroute
    MID_GNE_REROUTEDIALOG_TABLE_ROUTEPROBREROUTE,
    /// @brief change table parkingAreaReroute
    MID_GNE_REROUTEDIALOG_TABLE_PARKINGAREAREROUTE,
    /// @brief add interval
    MID_GNE_REROUTEDIALOG_ADD_INTERVAL,
    /// @brief sort rerouter intervals
    MID_GNE_REROUTEDIALOG_SORT_INTERVAL,
    /// @brief add closing lane reroute
    MID_GNE_REROUTEDIALOG_ADD_CLOSINGLANEREROUTE,
    /// @brief add closing reroute
    MID_GNE_REROUTEDIALOG_ADD_CLOSINGREROUTE,
    /// @brief add destination probability route
    MID_GNE_REROUTEDIALOG_ADD_DESTPROBREROUTE,
    /// @brief add route probability route
    MID_GNE_REROUTEDIALOG_ADD_ROUTEPROBREROUTE,
    /// @brief add parkingAreaReroute
    MID_GNE_REROUTEDIALOG_ADD_PARKINGAREAREROUTE,
    /// @brief edit interval
    MID_GNE_REROUTEDIALOG_EDIT_INTERVAL,

    /// @}

    /// @name Calibrator Dialog messages
    /// @{

    /// @brief change table route
    MID_GNE_CALIBRATORDIALOG_TABLE_ROUTE,
    /// @brief add new route
    MID_GNE_CALIBRATORDIALOG_ADD_ROUTE,
    /// @brief change table flow
    MID_GNE_CALIBRATORDIALOG_TABLE_FLOW,
    MID_GNE_CALIBRATORDIALOG_ADD_FLOW,
    /// @brief change table route
    MID_GNE_CALIBRATORDIALOG_TABLE_VEHICLETYPE,
    /// @brief add vehicle type
    MID_GNE_CALIBRATORDIALOG_ADD_VEHICLETYPE,

    /// @}

    /// @name allowDisallow Dialog messages
    /// @{

    /// @brief select/unselect single vehicle
    MID_GNE_ALLOWDISALLOW_CHANGE,
    /// @brief select all vehicles
    MID_GNE_ALLOWDISALLOW_SELECTALL,
    /// @brief unselect all vehicles
    MID_GNE_ALLOWDISALLOW_UNSELECTALL,
    /// @brief select only non road vehicles
    MID_GNE_ALLOWDISALLOW_ONLY_ROAD,
    /// @brief select only rail vehicles
    MID_GNE_ALLOWDISALLOW_ONLY_RAIL,

    /// @}

    /// @name Fix Stoppingplaces Dialog messages
    /// @{

    /// @brief FixStoppingPlaces dialog
    MID_GNE_FIXSTOPPINGPLACES_CHANGE,

    /// @}

    /// @name GNETLSTable
    /// @{

    /// @brief TLSTable textField
    MID_GNE_TLSTABLE_TEXTFIELD,
    /// @brief TLSTable button for add phase
    MID_GNE_TLSTABLE_ADDPHASE,
    /// @brief TLSTable button for copy phase
    MID_GNE_TLSTABLE_COPYPHASE,
    /// @brief TLSTable button for add phase red
    MID_GNE_TLSTABLE_ADDPHASEALLRED,
    /// @brief TLSTable button for add phase yelllow
    MID_GNE_TLSTABLE_ADDPHASEALLYELLOW,
    /// @brief TLSTable button for add phase green
    MID_GNE_TLSTABLE_ADDPHASEALLGREEN,
    /// @brief TLSTable button for add phase green priority
    MID_GNE_TLSTABLE_ADDPHASEALLGREENPRIORITY,
    /// @brief TLSTable button for remove phase
    MID_GNE_TLSTABLE_REMOVEPHASE,
    /// @brief TLSTable button for move up phase
    MID_GNE_TLSTABLE_MOVEUPPHASE,
    /// @brief TLSTable button for move down phase
    MID_GNE_TLSTABLE_MOVEDOWNPHASE,

    /// @}

    /// @name GNEOptionsDialog
    /// @{

    /// @brief use search description
    MID_GNE_SEARCH_USEDESCRIPTION,

    /// @}

    /// @name MFXDecalsTable
    /// @{

    /// @brief textField
    MID_DECALSTABLE_TEXTFIELD,
    /// @brief checkbox
    MID_DECALSTABLE_CHECKBOX,
    /// @brief spinner
    MID_DECALSTABLE_SPINNER,
    /// @brief open decal
    MID_DECALSTABLE_OPEN,
    /// @brief add row
    MID_DECALSTABLE_ADD,
    /// @brief add row
    MID_DECALSTABLE_REMOVE,

    /// @}

    /// @name other
    /// @{

    /// @brief selector for ModesMenuTitle
    MID_GNE_MODESMENUTITLE,
    /// @brief selector for LockMenuTitle
    MID_GNE_LOCK_MENUTITLE,
    /// @brief open undo list dialog
    MID_GNE_UNDOLISTDIALOG,
    /// @brief update undolist
    MID_GNE_UNDOLIST_UPDATE,
    /// @brief check if recomputing is needed
    MID_GNE_RECOMPUTINGNEEDED,
    /// @brief create automatic filename if it was not defined previously
    MID_GNE_AUTOMATICFILENAME,

    /// @}

    /// @brief callback for MFXTextFieldSearch
    /// @{

    MID_MTEXTFIELDSEARCH_UPDATED,

    /// @}

    /// @brief callback for MFXMenuButtonTooltip
    /// @{

    MID_MBTTIP_FOCUS,
    MID_MBTTIP_SELECTED,

    /// @}

    /// @name languages
    /// @{

    /// @brief change language to english
    MID_LANGUAGE_EN,
    /// @brief change language to german
    MID_LANGUAGE_DE,
    /// @brief change language to spanish
    MID_LANGUAGE_ES,
    /// @brief change language to french
    MID_LANGUAGE_FR,
    /// @brief change language to italian
    MID_LANGUAGE_IT,
    /// @brief change language to chinese (simplified)
    MID_LANGUAGE_ZH,
    /// @brief change language to chinese (traditional)
    MID_LANGUAGE_ZHT,
    /// @brief change language to turkish
    MID_LANGUAGE_TR,
    /// @brief change language to hungarian
    MID_LANGUAGE_HU,
    /// @brief change language to japanese
    MID_LANGUAGE_JA,

    /// @}

    /// @brief run tests
    MID_RUNTESTS,

    /// @brief last element of enum (not used)
    MID_LAST
};
