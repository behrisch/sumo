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
/// @file    GUIShortcutsSubSys.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Feb 2019
///
// Helper for Shortcuts
/****************************************************************************/
#include <config.h>

#include <utils/gui/windows/GUIAppEnum.h>

#include "GUIShortcutsSubSys.h"

// ===========================================================================
// member definitions
// ===========================================================================

void
GUIShortcutsSubSys::buildAccelerators(FXAccelTable* accelTable, FXObject* target, const bool sumogui) {

    // basic hotkeys

    accelTable->addAccel(parseKey(KEY_a), target, FXSEL(SEL_COMMAND, MID_HOTKEY_A_MODE_STARTSIMULATION_ADDITIONALS_STOPS));
    accelTable->addAccel(parseKey(KEY_A), target, FXSEL(SEL_COMMAND, MID_HOTKEY_A_MODE_STARTSIMULATION_ADDITIONALS_STOPS));

    accelTable->addAccel(parseKey(KEY_b), target, FXSEL(SEL_COMMAND, MID_HOTKEY_B_BREAKPOINT));
    accelTable->addAccel(parseKey(KEY_B), target, FXSEL(SEL_COMMAND, MID_HOTKEY_B_BREAKPOINT));

    accelTable->addAccel(parseKey(KEY_c), target, FXSEL(SEL_COMMAND, MID_HOTKEY_C_MODE_CONNECT_CONTAINER));
    accelTable->addAccel(parseKey(KEY_C), target, FXSEL(SEL_COMMAND, MID_HOTKEY_C_MODE_CONNECT_CONTAINER));

    accelTable->addAccel(parseKey(KEY_d), target, FXSEL(SEL_COMMAND, MID_HOTKEY_D_MODE_SINGLESIMULATIONSTEP_DELETE));
    accelTable->addAccel(parseKey(KEY_D), target, FXSEL(SEL_COMMAND, MID_HOTKEY_D_MODE_SINGLESIMULATIONSTEP_DELETE));

    accelTable->addAccel(parseKey(KEY_e), target, FXSEL(SEL_COMMAND, MID_HOTKEY_E_MODE_EDGE_EDGEDATA));
    accelTable->addAccel(parseKey(KEY_E), target, FXSEL(SEL_COMMAND, MID_HOTKEY_E_MODE_EDGE_EDGEDATA));

    accelTable->addAccel(parseKey(KEY_h), target, FXSEL(SEL_COMMAND, MID_HOTKEY_H_MODE_PROHIBITION_CONTAINERPLAN));
    accelTable->addAccel(parseKey(KEY_H), target, FXSEL(SEL_COMMAND, MID_HOTKEY_H_MODE_PROHIBITION_CONTAINERPLAN));

    accelTable->addAccel(parseKey(KEY_i), target, FXSEL(SEL_COMMAND, MID_HOTKEY_I_MODE_INSPECT));
    accelTable->addAccel(parseKey(KEY_I), target, FXSEL(SEL_COMMAND, MID_HOTKEY_I_MODE_INSPECT));

    accelTable->addAccel(parseKey(KEY_l), target, FXSEL(SEL_COMMAND, MID_HOTKEY_L_MODE_PERSONPLAN));
    accelTable->addAccel(parseKey(KEY_L), target, FXSEL(SEL_COMMAND, MID_HOTKEY_L_MODE_PERSONPLAN));

    accelTable->addAccel(parseKey(KEY_m), target, FXSEL(SEL_COMMAND, MID_HOTKEY_M_MODE_MOVE_MEANDATA));
    accelTable->addAccel(parseKey(KEY_M), target, FXSEL(SEL_COMMAND, MID_HOTKEY_M_MODE_MOVE_MEANDATA));

    accelTable->addAccel(parseKey(KEY_p), target, FXSEL(SEL_COMMAND, MID_HOTKEY_P_MODE_POLYGON_PERSON));
    accelTable->addAccel(parseKey(KEY_P), target, FXSEL(SEL_COMMAND, MID_HOTKEY_P_MODE_POLYGON_PERSON));

    accelTable->addAccel(parseKey(KEY_r), target, FXSEL(SEL_COMMAND, MID_HOTKEY_R_MODE_CROSSING_ROUTE_EDGERELDATA));
    accelTable->addAccel(parseKey(KEY_R), target, FXSEL(SEL_COMMAND, MID_HOTKEY_R_MODE_CROSSING_ROUTE_EDGERELDATA));

    accelTable->addAccel(parseKey(KEY_s), target, FXSEL(SEL_COMMAND, MID_HOTKEY_S_MODE_STOPSIMULATION_SELECT));
    accelTable->addAccel(parseKey(KEY_S), target, FXSEL(SEL_COMMAND, MID_HOTKEY_S_MODE_STOPSIMULATION_SELECT));

    accelTable->addAccel(parseKey(KEY_t), target, FXSEL(SEL_COMMAND, MID_HOTKEY_T_MODE_TLS_TYPE));
    accelTable->addAccel(parseKey(KEY_T), target, FXSEL(SEL_COMMAND, MID_HOTKEY_T_MODE_TLS_TYPE));

    accelTable->addAccel(parseKey(KEY_u), target, FXSEL(SEL_COMMAND, MID_HOTKEY_U_MODE_DECAL_TYPEDISTRIBUTION));
    accelTable->addAccel(parseKey(KEY_U), target, FXSEL(SEL_COMMAND, MID_HOTKEY_U_MODE_DECAL_TYPEDISTRIBUTION));

    accelTable->addAccel(parseKey(KEY_v), target, FXSEL(SEL_COMMAND, MID_HOTKEY_V_MODE_VEHICLE));
    accelTable->addAccel(parseKey(KEY_V), target, FXSEL(SEL_COMMAND, MID_HOTKEY_V_MODE_VEHICLE));

    accelTable->addAccel(parseKey(KEY_w), target, FXSEL(SEL_COMMAND, MID_HOTKEY_W_MODE_WIRE_ROUTEDISTRIBUTION));
    accelTable->addAccel(parseKey(KEY_W), target, FXSEL(SEL_COMMAND, MID_HOTKEY_W_MODE_WIRE_ROUTEDISTRIBUTION));

    accelTable->addAccel(parseKey(KEY_z), target, FXSEL(SEL_COMMAND, MID_HOTKEY_Z_MODE_TAZ_TAZREL));
    accelTable->addAccel(parseKey(KEY_Z), target, FXSEL(SEL_COMMAND, MID_HOTKEY_Z_MODE_TAZ_TAZREL));

    // function Hotkeys

    accelTable->addAccel(parseKey(KEY_F1),  target, FXSEL(SEL_COMMAND, MID_HOTKEY_F1_ONLINEDOCUMENTATION));
    accelTable->addAccel(parseKey(KEY_F2),  target, FXSEL(SEL_COMMAND, MID_HOTKEY_F2_SUPERMODE_NETWORK));
    accelTable->addAccel(parseKey(KEY_F3),  target, FXSEL(SEL_COMMAND, MID_HOTKEY_F3_SUPERMODE_DEMAND));
    accelTable->addAccel(parseKey(KEY_F4),  target, FXSEL(SEL_COMMAND, MID_HOTKEY_F4_SUPERMODE_DATA));
    accelTable->addAccel(parseKey(KEY_F5),  target, FXSEL(SEL_COMMAND, MID_HOTKEY_F5_COMPUTE_NETWORK_DEMAND));
    accelTable->addAccel(parseKey(KEY_F6),  target, FXSEL(SEL_COMMAND, MID_HOTKEY_F6_CLEAN_SOLITARYJUNCTIONS_UNUSEDROUTES));
    accelTable->addAccel(parseKey(KEY_F7),  target, FXSEL(SEL_COMMAND, MID_HOTKEY_F7_JOIN_SELECTEDJUNCTIONS_ROUTES));
    accelTable->addAccel(parseKey(KEY_F8),  target, FXSEL(SEL_COMMAND, MID_HOTKEY_F8_CLEANINVALID_CROSSINGS_DEMANDELEMENTS));
    accelTable->addAccel(parseKey(KEY_F9),  target, FXSEL(SEL_COMMAND, MID_HOTKEY_F9_EDIT_VIEWSCHEME));
    accelTable->addAccel(parseKey(KEY_F10), target, FXSEL(SEL_COMMAND, MID_HOTKEY_F10_OPTIONSMENU));
    accelTable->addAccel(parseKey(KEY_F11), target, FXSEL(SEL_COMMAND, MID_HOTKEY_F11_FRONTELEMENT));
    accelTable->addAccel(parseKey(KEY_F12), target, FXSEL(SEL_COMMAND, MID_HOTKEY_F12_ABOUT));

    // other hotkeys

    accelTable->addAccel(parseKey(KEY_ESC),        target, FXSEL(SEL_COMMAND, MID_HOTKEY_ESC));
    accelTable->addAccel(parseKey(KEY_DEL),        target, FXSEL(SEL_COMMAND, MID_HOTKEY_DEL));
    accelTable->addAccel(parseKey(KEY_ENTER),      target, FXSEL(SEL_COMMAND, MID_HOTKEY_ENTER));
    accelTable->addAccel(parseKey(KEY_BACKSPACE),  target, FXSEL(SEL_COMMAND, MID_HOTKEY_BACKSPACE));

    // control hotkeys

    accelTable->addAccel(parseKey(KEY_a, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_A_STARTSIMULATION_OPENADDITIONALELEMENTS));
    accelTable->addAccel(parseKey(KEY_A, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_A_STARTSIMULATION_OPENADDITIONALELEMENTS));

    accelTable->addAccel(parseKey(KEY_b, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_B_EDITBREAKPOINT_OPENDATAELEMENTS));
    accelTable->addAccel(parseKey(KEY_B, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_B_EDITBREAKPOINT_OPENDATAELEMENTS));

    accelTable->addAccel(parseKey(KEY_c, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_C_COPY));
    accelTable->addAccel(parseKey(KEY_C, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_C_COPY));

    accelTable->addAccel(parseKey(KEY_d, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_D_SINGLESIMULATIONSTEP_OPENDEMANDELEMENTS));
    accelTable->addAccel(parseKey(KEY_D, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_D_SINGLESIMULATIONSTEP_OPENDEMANDELEMENTS));

    // special case: we can NOT overwritte here the ctrl+E shortcut in SUMO-GUI
    if (!sumogui) {
        accelTable->addAccel(parseKey(KEY_e, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_E_EDITSELECTION_LOADNETEDITCONFIG));
        accelTable->addAccel(parseKey(KEY_E, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_E_EDITSELECTION_LOADNETEDITCONFIG));
    }

    accelTable->addAccel(parseKey(KEY_f, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_F_FULSCREENMODE));
    accelTable->addAccel(parseKey(KEY_F, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_F_FULSCREENMODE));

    accelTable->addAccel(parseKey(KEY_g, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_G_GAMINGMODE_TOGGLEGRID));
    accelTable->addAccel(parseKey(KEY_G, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_G_GAMINGMODE_TOGGLEGRID));

    accelTable->addAccel(parseKey(KEY_h, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_H_APPSETTINGS_OPENEDGETYPES));
    accelTable->addAccel(parseKey(KEY_H, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_H_APPSETTINGS_OPENEDGETYPES));

    accelTable->addAccel(parseKey(KEY_i, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_I_EDITVIEWPORT));
    accelTable->addAccel(parseKey(KEY_I, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_I_EDITVIEWPORT));

    accelTable->addAccel(parseKey(KEY_j, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_J_TOGGLEDRAWJUNCTIONSHAPE));
    accelTable->addAccel(parseKey(KEY_J, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_J_TOGGLEDRAWJUNCTIONSHAPE));

    accelTable->addAccel(parseKey(KEY_k, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_K_OPENTLSPROGRAMS));
    accelTable->addAccel(parseKey(KEY_K, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_K_OPENTLSPROGRAMS));

    accelTable->addAccel(parseKey(KEY_l, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_L_SAVEASPLAINXML));
    accelTable->addAccel(parseKey(KEY_L, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_L_SAVEASPLAINXML));

    accelTable->addAccel(parseKey(KEY_m, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_M_OPENSUMOCONFIG));
    accelTable->addAccel(parseKey(KEY_M, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_M_OPENSUMOCONFIG));

    accelTable->addAccel(parseKey(KEY_n, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_N_OPENNETWORK_NEWNETWORK));
    accelTable->addAccel(parseKey(KEY_N, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_N_OPENNETWORK_NEWNETWORK));

    accelTable->addAccel(parseKey(KEY_o, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_O_OPENSIMULATION_OPENNETWORK));
    accelTable->addAccel(parseKey(KEY_O, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_O_OPENSIMULATION_OPENNETWORK));

    accelTable->addAccel(parseKey(KEY_p, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_P_OPENSHAPES));
    accelTable->addAccel(parseKey(KEY_P, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_P_OPENSHAPES));

    accelTable->addAccel(parseKey(KEY_q, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_Q_CLOSE));
    accelTable->addAccel(parseKey(KEY_Q, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_Q_CLOSE));

    accelTable->addAccel(parseKey(KEY_r, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_R_RELOAD));
    accelTable->addAccel(parseKey(KEY_R, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_R_RELOAD));

    accelTable->addAccel(parseKey(KEY_s, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_S_STOPSIMULATION_SAVENETWORK));
    accelTable->addAccel(parseKey(KEY_S, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_S_STOPSIMULATION_SAVENETWORK));

    accelTable->addAccel(parseKey(KEY_t, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_T_OPENNETEDIT_OPENSUMO));
    accelTable->addAccel(parseKey(KEY_T, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_T_OPENNETEDIT_OPENSUMO));

    accelTable->addAccel(parseKey(KEY_u, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_U_OPENEDGEDATA));
    accelTable->addAccel(parseKey(KEY_U, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_U_OPENEDGEDATA));

    accelTable->addAccel(parseKey(KEY_w, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_W_CLOSESIMULATION));
    accelTable->addAccel(parseKey(KEY_W, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_W_CLOSESIMULATION));

    accelTable->addAccel(parseKey(KEY_v, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_V_PASTE));
    accelTable->addAccel(parseKey(KEY_V, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_V_PASTE));

    accelTable->addAccel(parseKey(KEY_x, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_X_CUT));
    accelTable->addAccel(parseKey(KEY_X, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_X_CUT));

    accelTable->addAccel(parseKey(KEY_y, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_Y_REDO));
    accelTable->addAccel(parseKey(KEY_Y, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_Y_REDO));

    accelTable->addAccel(parseKey(KEY_z, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_Z_UNDO));
    accelTable->addAccel(parseKey(KEY_Z, KEYMODIFIER_CONTROL), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_Z_UNDO));

    // control + shift hotkeys

    accelTable->addAccel(parseKey(KEY_a, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_A_SAVEADDITIONALELEMENTS));
    accelTable->addAccel(parseKey(KEY_A, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_A_SAVEADDITIONALELEMENTS));

    accelTable->addAccel(parseKey(KEY_b, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_B_SAVEDATAELEMENTS));
    accelTable->addAccel(parseKey(KEY_B, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_B_SAVEDATAELEMENTS));

    accelTable->addAccel(parseKey(KEY_c, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_C_SAVENETEDITCONFIG));
    accelTable->addAccel(parseKey(KEY_C, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_C_SAVENETEDITCONFIG));

    accelTable->addAccel(parseKey(KEY_d, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_D_SAVEDEMANDELEMENTS));
    accelTable->addAccel(parseKey(KEY_D, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_D_SAVEDEMANDELEMENTS));

    accelTable->addAccel(parseKey(KEY_e, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_E_SAVENETEDITCONFIG));
    accelTable->addAccel(parseKey(KEY_E, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_E_SAVENETEDITCONFIG));

    accelTable->addAccel(parseKey(KEY_h, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_H_SAVEEDGETYPES));
    accelTable->addAccel(parseKey(KEY_H, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_H_SAVEEDGETYPES));

    accelTable->addAccel(parseKey(KEY_k, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_K_SAVETLS));
    accelTable->addAccel(parseKey(KEY_K, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_K_SAVETLS));

    accelTable->addAccel(parseKey(KEY_m, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_M_SAVEMEANDATAELEMENTS));
    accelTable->addAccel(parseKey(KEY_M, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_M_SAVEMEANDATAELEMENTS));

    accelTable->addAccel(parseKey(KEY_n, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_N_NEWWINDOW));
    accelTable->addAccel(parseKey(KEY_N, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_N_NEWWINDOW));

    accelTable->addAccel(parseKey(KEY_o, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_O_OPENNETCONVERTFILE));
    accelTable->addAccel(parseKey(KEY_O, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_O_OPENNETCONVERTFILE));

    accelTable->addAccel(parseKey(KEY_s, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_S_SAVESUMOCONFIG));
    accelTable->addAccel(parseKey(KEY_S, KEYMODIFIER_CONTROL, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_CTRL_SHIFT_S_SAVESUMOCONFIG));

    // shift hotkeys

    accelTable->addAccel(parseKey(KEY_a, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_A_LOCATEADDITIONAL));
    accelTable->addAccel(parseKey(KEY_A, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_A_LOCATEADDITIONAL));

    accelTable->addAccel(parseKey(KEY_c, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_C_LOCATECONTAINER));
    accelTable->addAccel(parseKey(KEY_C, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_C_LOCATECONTAINER));

    accelTable->addAccel(parseKey(KEY_e, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_E_LOCATEEDGE));
    accelTable->addAccel(parseKey(KEY_E, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_E_LOCATEEDGE));

    accelTable->addAccel(parseKey(KEY_j, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_J_LOCATEJUNCTION));
    accelTable->addAccel(parseKey(KEY_J, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_J_LOCATEJUNCTION));

    accelTable->addAccel(parseKey(KEY_l, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_L_LOCATEPOLY));
    accelTable->addAccel(parseKey(KEY_L, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_L_LOCATEPOLY));

    accelTable->addAccel(parseKey(KEY_o, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_O_LOCATEPOI));
    accelTable->addAccel(parseKey(KEY_O, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_O_LOCATEPOI));

    accelTable->addAccel(parseKey(KEY_p, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_P_LOCATEPERSON));
    accelTable->addAccel(parseKey(KEY_P, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_P_LOCATEPERSON));

    accelTable->addAccel(parseKey(KEY_t, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_T_LOCATETLS));
    accelTable->addAccel(parseKey(KEY_T, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_T_LOCATETLS));

    accelTable->addAccel(parseKey(KEY_v, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_V_LOCATEVEHICLE));
    accelTable->addAccel(parseKey(KEY_V, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_V_LOCATEVEHICLE));

    accelTable->addAccel(parseKey(KEY_F1,  KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_F1_TEMPLATE_SET));
    accelTable->addAccel(parseKey(KEY_F2,  KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_F2_TEMPLATE_COPY));
    accelTable->addAccel(parseKey(KEY_F3,  KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_F3_TEMPLATE_CLEAR));
    accelTable->addAccel(parseKey(KEY_F5,  KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_F5_COMPUTEJUNCTIONS_VOLATILE));
    accelTable->addAccel(parseKey(KEY_F7,  KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_F7_ADJUST_PERSON_PLANS));
    accelTable->addAccel(parseKey(KEY_F10, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_F10_SUMOOPTIONSMENU));
    accelTable->addAccel(parseKey(KEY_F11, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_F11_HALLOFFAME));
    accelTable->addAccel(parseKey(KEY_F12, KEYMODIFIER_SHIFT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_F12_FOCUSUPPERELEMENT));

    accelTable->addAccel(parseKey(KEY_ESC, KEYMODIFIER_SHIFT),  target, FXSEL(SEL_COMMAND, MID_HOTKEY_SHIFT_ESC_CLEARSELECTION));

    // alt hotkeys

    accelTable->addAccel(parseKey(KEY_0, KEYMODIFIER_ALT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_ALT_0_TOGGLEEDITOPTION));
    accelTable->addAccel(parseKey(KEY_1, KEYMODIFIER_ALT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_ALT_1_TOGGLEEDITOPTION));
    accelTable->addAccel(parseKey(KEY_2, KEYMODIFIER_ALT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_ALT_2_TOGGLEEDITOPTION));
    accelTable->addAccel(parseKey(KEY_3, KEYMODIFIER_ALT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_ALT_3_TOGGLEEDITOPTION));
    accelTable->addAccel(parseKey(KEY_4, KEYMODIFIER_ALT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_ALT_4_TOGGLEEDITOPTION));
    accelTable->addAccel(parseKey(KEY_5, KEYMODIFIER_ALT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_ALT_5_TOGGLEEDITOPTION));
    accelTable->addAccel(parseKey(KEY_6, KEYMODIFIER_ALT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_ALT_6_TOGGLEEDITOPTION));
    accelTable->addAccel(parseKey(KEY_7, KEYMODIFIER_ALT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_ALT_7_TOGGLEEDITOPTION));
    accelTable->addAccel(parseKey(KEY_8, KEYMODIFIER_ALT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_ALT_8_TOGGLEEDITOPTION));
    accelTable->addAccel(parseKey(KEY_9, KEYMODIFIER_ALT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_ALT_9_TOGGLEEDITOPTION));

    accelTable->addAccel(parseKey(KEY_F4, KEYMODIFIER_ALT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_ALT_F4_CLOSE));
    accelTable->addAccel(parseKey(KEY_b, KEYMODIFIER_ALT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_ALT_B_BREAKPOINT_EARLY));
    accelTable->addAccel(parseKey(KEY_B, KEYMODIFIER_ALT), target, FXSEL(SEL_COMMAND, MID_HOTKEY_ALT_B_BREAKPOINT_EARLY));
}


void
GUIShortcutsSubSys::changeAccelerator(FXAccelTable* accelTable, FXObject* target, GUIShortcut keysym, long msg) {
    accelTable->addAccel(parseKey(keysym), target, FXSEL(SEL_COMMAND, msg));
}


int
GUIShortcutsSubSys::parseKey(GUIShortcut key) {
    // beware!: xx in the key - xx + yy expression may change when the content of enum GUIShortcut is changed (e.g. addition of space key)
    if ((key >= KEY_0) &&  key <= KEY_9) {
        return (key + 48); // 48 is '0' in ASCII
    } else if ((key >= KEY_a) &&  key <= KEY_z) {
        return (key - 10 + 97); // 97 is 'a' in ASCII
    } else if ((key >= KEY_A) &&  key <= KEY_Z) {
        return (key - 36 + 65); // 65 is 'A' in ASCII
    } else if (key == KEY_SPACE) {
        return (key - 62 + 32); // 32 is SPACE  in ASCII
    } else if ((key >= KEY_F1) &&  key <= KEY_F12) {
        return (key - 63 + 65470); // 65470 is 'F1' in ASCII
    } else if (key == KEY_ESC) {
        return parseAccel("Esc");
    } else if (key == KEY_ENTER) {
        return parseAccel("Enter");
    } else if (key == KEY_BACKSPACE) {
        return parseAccel("Back");
    } else if (key == KEY_DEL) {
        return parseAccel("Del");
    } else {
        return 0;
    }
}


int
GUIShortcutsSubSys::parseKey(GUIShortcut key, GUIShortcutModifier firstModifier) {
    int keyCode = parseKey(key);
    // add first modifier
    if (firstModifier == KEYMODIFIER_SHIFT) {
        keyCode += 65536;
    } else if (firstModifier == KEYMODIFIER_ALT) {
        keyCode += 524288;
    } else if (firstModifier == KEYMODIFIER_CONTROL) {
        keyCode += 262144;
    }
    return keyCode;
}


int
GUIShortcutsSubSys::parseKey(GUIShortcut key, GUIShortcutModifier firstModifier, GUIShortcutModifier secondModifier) {
    int keyCode = parseKey(key, firstModifier);
    // add second modifier
    if (secondModifier == KEYMODIFIER_SHIFT) {
        keyCode += 65536;
    } else if (secondModifier == KEYMODIFIER_ALT) {
        keyCode += 524288;
    } else if (secondModifier == KEYMODIFIER_CONTROL) {
        keyCode += 262144;
    }
    return keyCode;
}


/****************************************************************************/
