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
/// @file    GNEViewNet.cpp
/// @author  Jakob Erdmann
/// @author  Pablo Alvarez Lopez
/// @date    Feb 2011
///
// A view on the network being edited (adapted from GUIViewTraffic)
/****************************************************************************/

#include <netbuild/NBEdgeCont.h>
#include <netedit/changes/GNEChange_Attribute.h>
#include <netedit/changes/GNEChange_TLS.h>
#include <netedit/dialogs/GNEGeometryPointDialog.h>
#include <netedit/elements/additional/GNEAdditionalHandler.h>
#include <netedit/elements/additional/GNEPOI.h>
#include <netedit/elements/additional/GNEPoly.h>
#include <netedit/elements/additional/GNETAZ.h>
#include <netedit/elements/demand/GNERoute.h>
#include <netedit/elements/network/GNEWalkingArea.h>
#include <netedit/elements/network/GNEConnection.h>
#include <netedit/elements/network/GNECrossing.h>
#include <netedit/frames/common/GNEDeleteFrame.h>
#include <netedit/frames/common/GNEInspectorFrame.h>
#include <netedit/frames/common/GNEMoveFrame.h>
#include <netedit/frames/common/GNESelectorFrame.h>
#include <netedit/frames/data/GNEEdgeDataFrame.h>
#include <netedit/frames/data/GNEEdgeRelDataFrame.h>
#include <netedit/frames/data/GNETAZRelDataFrame.h>
#include <netedit/frames/data/GNEMeanDataFrame.h>
#include <netedit/frames/demand/GNEContainerFrame.h>
#include <netedit/frames/demand/GNEContainerPlanFrame.h>
#include <netedit/frames/demand/GNEPersonFrame.h>
#include <netedit/frames/demand/GNEPersonPlanFrame.h>
#include <netedit/frames/demand/GNERouteFrame.h>
#include <netedit/frames/demand/GNERouteDistributionFrame.h>
#include <netedit/frames/demand/GNEStopFrame.h>
#include <netedit/frames/demand/GNETypeFrame.h>
#include <netedit/frames/demand/GNETypeDistributionFrame.h>
#include <netedit/frames/demand/GNEVehicleFrame.h>
#include <netedit/frames/network/GNEAdditionalFrame.h>
#include <netedit/frames/network/GNEConnectorFrame.h>
#include <netedit/frames/network/GNECreateEdgeFrame.h>
#include <netedit/frames/network/GNECrossingFrame.h>
#include <netedit/frames/network/GNEDecalFrame.h>
#include <netedit/frames/network/GNEProhibitionFrame.h>
#include <netedit/frames/network/GNEShapeFrame.h>
#include <netedit/frames/network/GNETAZFrame.h>
#include <netedit/frames/network/GNETLSEditorFrame.h>
#include <netedit/frames/network/GNEWireFrame.h>
#include <utils/foxtools/MFXMenuCheckIcon.h>
#include <utils/gui/cursors/GUICursorSubSys.h>
#include <utils/gui/div/GLHelper.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/gui/div/GUIGlobalViewObjectsHandler.h>
#include <utils/gui/globjects/GUICursorDialog.h>
#include <utils/gui/globjects/GUIGlObjectStorage.h>
#include <utils/gui/settings/GUICompleteSchemeStorage.h>
#include <utils/gui/windows/GUIDanielPerspectiveChanger.h>
#include <utils/gui/windows/GUIDialog_ViewSettings.h>
#include <utils/options/OptionsCont.h>

#include <unordered_set>

#include "GNENet.h"
#include "GNEUndoList.h"
#include "GNEViewNet.h"
#include "GNEViewParent.h"
#include "GNEApplicationWindow.h"


// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXDEFMAP(GNEViewNet) GNEViewNetMap[] = {
    // Super Modes
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_F2_SUPERMODE_NETWORK, GNEViewNet::onCmdSetSupermode),
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_F3_SUPERMODE_DEMAND,  GNEViewNet::onCmdSetSupermode),
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_F4_SUPERMODE_DATA,    GNEViewNet::onCmdSetSupermode),
    // Modes
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_A_MODE_STARTSIMULATION_ADDITIONALS_STOPS,    GNEViewNet::onCmdSetMode),
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_C_MODE_CONNECT_CONTAINER,                 GNEViewNet::onCmdSetMode),
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_D_MODE_SINGLESIMULATIONSTEP_DELETE,       GNEViewNet::onCmdSetMode),
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_E_MODE_EDGE_EDGEDATA,                     GNEViewNet::onCmdSetMode),
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_H_MODE_PROHIBITION_CONTAINERPLAN,         GNEViewNet::onCmdSetMode),
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_I_MODE_INSPECT,                           GNEViewNet::onCmdSetMode),
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_L_MODE_PERSONPLAN,                        GNEViewNet::onCmdSetMode),
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_M_MODE_MOVE_MEANDATA,                     GNEViewNet::onCmdSetMode),
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_P_MODE_POLYGON_PERSON,                    GNEViewNet::onCmdSetMode),
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_R_MODE_CROSSING_ROUTE_EDGERELDATA,        GNEViewNet::onCmdSetMode),
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_S_MODE_STOPSIMULATION_SELECT,             GNEViewNet::onCmdSetMode),
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_T_MODE_TLS_TYPE,                          GNEViewNet::onCmdSetMode),
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_U_MODE_DECAL_TYPEDISTRIBUTION,            GNEViewNet::onCmdSetMode),
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_V_MODE_VEHICLE,                           GNEViewNet::onCmdSetMode),
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_W_MODE_WIRE_ROUTEDISTRIBUTION,            GNEViewNet::onCmdSetMode),
    FXMAPFUNC(SEL_COMMAND, MID_HOTKEY_Z_MODE_TAZ_TAZREL,                        GNEViewNet::onCmdSetMode),
    // Network view options
    FXMAPFUNC(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_TOGGLEGRID,               GNEViewNet::onCmdToggleShowGrid),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_TOGGLEDRAWJUNCTIONSHAPE,  GNEViewNet::onCmdToggleDrawJunctionShape),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_DRAWSPREADVEHICLES,       GNEViewNet::onCmdToggleDrawSpreadVehicles),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_SHOWDEMANDELEMENTS,       GNEViewNet::onCmdToggleShowDemandElementsNetwork),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_SELECTEDGES,              GNEViewNet::onCmdToggleSelectEdges),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_SHOWCONNECTIONS,          GNEViewNet::onCmdToggleShowConnections),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_HIDECONNECTIONS,          GNEViewNet::onCmdToggleHideConnections),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_SHOWSUBADDITIONALS,       GNEViewNet::onCmdToggleShowAdditionalSubElements),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_SHOWTAZELEMENTS,          GNEViewNet::onCmdToggleShowTAZElements),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_EXTENDSELECTION,          GNEViewNet::onCmdToggleExtendSelection),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_CHANGEALLPHASES,          GNEViewNet::onCmdToggleChangeAllPhases),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_ASKFORMERGE,              GNEViewNet::onCmdToggleWarnAboutMerge),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_SHOWBUBBLES,              GNEViewNet::onCmdToggleShowJunctionBubbles),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_MOVEELEVATION,            GNEViewNet::onCmdToggleMoveElevation),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_CHAINEDGES,               GNEViewNet::onCmdToggleChainEdges),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_AUTOOPPOSITEEDGES,        GNEViewNet::onCmdToggleAutoOppositeEdge),
    // Demand view options
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_SHOWGRID,                  GNEViewNet::onCmdToggleShowGrid),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_TOGGLEDRAWJUNCTIONSHAPE,   GNEViewNet::onCmdToggleDrawJunctionShape),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_DRAWSPREADVEHICLES,        GNEViewNet::onCmdToggleDrawSpreadVehicles),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_HIDENONINSPECTED,          GNEViewNet::onCmdToggleHideNonInspecteDemandElements),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_HIDESHAPES,                GNEViewNet::onCmdToggleHideShapes),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_SHOWTRIPS,                 GNEViewNet::onCmdToggleShowTrips),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_SHOWALLPERSONPLANS,        GNEViewNet::onCmdToggleShowAllPersonPlans),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_LOCKPERSON,                GNEViewNet::onCmdToggleLockPerson),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_SHOWALLCONTAINERPLANS,     GNEViewNet::onCmdToggleShowAllContainerPlans),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_LOCKCONTAINER,             GNEViewNet::onCmdToggleLockContainer),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_SHOWOVERLAPPEDROUTES,      GNEViewNet::onCmdToggleShowOverlappedRoutes),
    // Data view options
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DATAVIEWOPTIONS_TOGGLEDRAWJUNCTIONSHAPE, GNEViewNet::onCmdToggleDrawJunctionShape),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DATAVIEWOPTIONS_SHOWADDITIONALS,         GNEViewNet::onCmdToggleShowAdditionals),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DATAVIEWOPTIONS_SHOWSHAPES,              GNEViewNet::onCmdToggleShowShapes),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DATAVIEWOPTIONS_SHOWDEMANDELEMENTS,      GNEViewNet::onCmdToggleShowDemandElementsData),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DATAVIEWOPTIONS_TAZRELDRAWING,           GNEViewNet::onCmdToggleTAZRelDrawing),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DATAVIEWOPTIONS_TAZDRAWFILL,             GNEViewNet::onCmdToggleTAZDrawFill),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DATAVIEWOPTIONS_TAZRELONLYFROM,          GNEViewNet::onCmdToggleTAZRelOnlyFrom),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_DATAVIEWOPTIONS_TAZRELONLYTO,            GNEViewNet::onCmdToggleTAZRelOnlyTo),
    // Select elements
    FXMAPFUNC(SEL_COMMAND, MID_ADDSELECT,               GNEViewNet::onCmdAddSelected),
    FXMAPFUNC(SEL_COMMAND, MID_REMOVESELECT,            GNEViewNet::onCmdRemoveSelected),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_ADDSELECT_EDGE,      GNEViewNet::onCmdAddEdgeSelected),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_REMOVESELECT_EDGE,   GNEViewNet::onCmdRemoveEdgeSelected),
    // Junctions
    FXMAPFUNC(SEL_COMMAND, MID_GNE_JUNCTION_RESET_EDGE_ENDPOINTS,   GNEViewNet::onCmdResetEdgeEndPoints),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_JUNCTION_EDIT_SHAPE,             GNEViewNet::onCmdEditJunctionShape),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_JUNCTION_RESET_SHAPE,            GNEViewNet::onCmdResetJunctionShape),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_JUNCTION_REPLACE,                GNEViewNet::onCmdReplaceJunction),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_JUNCTION_SPLIT,                  GNEViewNet::onCmdSplitJunction),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_JUNCTION_SPLIT_RECONNECT,        GNEViewNet::onCmdSplitJunctionReconnect),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_JUNCTION_SELECT_ROUNDABOUT,      GNEViewNet::onCmdSelectRoundabout),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_JUNCTION_CONVERT_ROUNDABOUT,     GNEViewNet::onCmdConvertRoundabout),
    FXMAPFUNC(SEL_ENTER,   MID_GNE_JUNCTION_CONVERT_ROUNDABOUT,     GNEViewNet::onEnterConvertRoundabout),
    FXMAPFUNC(SEL_LEAVE,   MID_GNE_JUNCTION_CONVERT_ROUNDABOUT,     GNEViewNet::onLeaveConvertRoundabout),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_JUNCTION_CLEAR_CONNECTIONS,      GNEViewNet::onCmdClearConnections),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_JUNCTION_RESET_CONNECTIONS,      GNEViewNet::onCmdResetConnections),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_JUNCTION_ADDTLS,                 GNEViewNet::onCmdAddTLS),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_JUNCTION_ADDJOINTLS,             GNEViewNet::onCmdAddJoinTLS),
    // Connections
    FXMAPFUNC(SEL_COMMAND, MID_GNE_CONNECTION_EDIT_SHAPE,           GNEViewNet::onCmdEditConnectionShape),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_CONNECTION_SMOOTH_SHAPE,         GNEViewNet::onCmdSmoothConnectionShape),
    // Crossings
    FXMAPFUNC(SEL_COMMAND, MID_GNE_CROSSING_EDIT_SHAPE,             GNEViewNet::onCmdEditCrossingShape),
    // WalkingArea
    FXMAPFUNC(SEL_COMMAND, MID_GNE_WALKINGAREA_EDIT_SHAPE,          GNEViewNet::onCmdEditWalkingAreaShape),
    // Edges
    FXMAPFUNC(SEL_COMMAND, MID_GNE_EDGE_SPLIT,                      GNEViewNet::onCmdSplitEdge),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_EDGE_SPLIT_BIDI,                 GNEViewNet::onCmdSplitEdgeBidi),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_EDGE_REVERSE,                    GNEViewNet::onCmdReverseEdge),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_EDGE_ADD_REVERSE,                GNEViewNet::onCmdAddReversedEdge),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_EDGE_ADD_REVERSE_DISCONNECTED,   GNEViewNet::onCmdAddReversedEdgeDisconnected),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_EDGE_EDIT_ENDPOINT,              GNEViewNet::onCmdEditEdgeEndpoint),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_EDGE_RESET_ENDPOINT,             GNEViewNet::onCmdResetEdgeEndpoint),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_EDGE_STRAIGHTEN,                 GNEViewNet::onCmdStraightenEdges),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_EDGE_SMOOTH,                     GNEViewNet::onCmdSmoothEdges),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_EDGE_STRAIGHTEN_ELEVATION,       GNEViewNet::onCmdStraightenEdgesElevation),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_EDGE_SMOOTH_ELEVATION,           GNEViewNet::onCmdSmoothEdgesElevation),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_EDGE_RESET_LENGTH,               GNEViewNet::onCmdResetLength),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_EDGE_USEASTEMPLATE,              GNEViewNet::onCmdEdgeUseAsTemplate),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_EDGE_APPLYTEMPLATE,              GNEViewNet::onCmdEgeApplyTemplate),
    // Lanes
    FXMAPFUNC(SEL_COMMAND, MID_GNE_LANE_DUPLICATE,              GNEViewNet::onCmdDuplicateLane),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_LANE_EDIT_SHAPE,             GNEViewNet::onCmdEditLaneShape),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_LANE_RESET_CUSTOMSHAPE,      GNEViewNet::onCmdResetLaneCustomShape),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_LANE_RESET_OPPOSITELANE,     GNEViewNet::onCmdResetOppositeLane),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_LANE_TRANSFORM_SIDEWALK,     GNEViewNet::onCmdLaneOperation),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_LANE_TRANSFORM_BIKE,         GNEViewNet::onCmdLaneOperation),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_LANE_TRANSFORM_BUS,          GNEViewNet::onCmdLaneOperation),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_LANE_TRANSFORM_GREENVERGE,   GNEViewNet::onCmdLaneOperation),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_LANE_ADD_SIDEWALK,           GNEViewNet::onCmdLaneOperation),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_LANE_ADD_BIKE,               GNEViewNet::onCmdLaneOperation),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_LANE_ADD_BUS,                GNEViewNet::onCmdLaneOperation),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_LANE_ADD_GREENVERGE_FRONT,   GNEViewNet::onCmdLaneOperation),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_LANE_ADD_GREENVERGE_BACK,    GNEViewNet::onCmdLaneOperation),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_LANE_REMOVE_SIDEWALK,        GNEViewNet::onCmdLaneOperation),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_LANE_REMOVE_BIKE,            GNEViewNet::onCmdLaneOperation),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_LANE_REMOVE_BUS,             GNEViewNet::onCmdLaneOperation),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_LANE_REMOVE_GREENVERGE,      GNEViewNet::onCmdLaneOperation),
    FXMAPFUNC(SEL_COMMAND, MID_REACHABILITY,                    GNEViewNet::onCmdLaneReachability),
    // Additionals
    FXMAPFUNC(SEL_COMMAND, MID_OPEN_ADDITIONAL_DIALOG,  GNEViewNet::onCmdOpenAdditionalDialog),
    // Polygons
    FXMAPFUNC(SEL_COMMAND, MID_GNE_POLYGON_SIMPLIFY_SHAPE,          GNEViewNet::onCmdSimplifyShape),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_POLYGON_CLOSE,                   GNEViewNet::onCmdClosePolygon),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_POLYGON_OPEN,                    GNEViewNet::onCmdOpenPolygon),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_POLYGON_SELECT,                  GNEViewNet::onCmdSelectPolygonElements),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_POLYGON_SET_FIRST_POINT,         GNEViewNet::onCmdSetFirstGeometryPoint),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_POLYGON_DELETE_GEOMETRY_POINT,   GNEViewNet::onCmdDeleteGeometryPoint),
    // edit custom shapes
    FXMAPFUNC(SEL_COMMAND, MID_GNE_SHAPEEDITED_SIMPLIFY,                GNEViewNet::onCmdSimplifyShapeEdited),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_SHAPEEDITED_STRAIGHTEN,              GNEViewNet::onCmdStraightenShapeEdited),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_SHAPEEDITED_CLOSE,                   GNEViewNet::onCmdCloseShapeEdited),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_SHAPEEDITED_OPEN,                    GNEViewNet::onCmdOpenShapeEdited),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_SHAPEEDITED_SET_FIRST_POINT,         GNEViewNet::onCmdSetFirstGeometryPointShapeEdited),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_SHAPEEDITED_DELETE_GEOMETRY_POINT,   GNEViewNet::onCmdDeleteGeometryPointShapeEdited),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_SHAPEEDITED_RESET,                   GNEViewNet::onCmdResetShapeEdited),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_SHAPEEDITED_FINISH,                  GNEViewNet::onCmdFinishShapeEdited),
    // POIs
    FXMAPFUNC(SEL_COMMAND, MID_GNE_POI_TRANSFORM,   GNEViewNet::onCmdTransformPOI),
    // Demand elements
    FXMAPFUNC(SEL_COMMAND, MID_GNE_REVERSE,     GNEViewNet::onCmdReverse),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_ADDREVERSE,  GNEViewNet::onCmdAddReverse),
    // Geometry Points
    FXMAPFUNC(SEL_COMMAND, MID_GNE_CUSTOM_GEOMETRYPOINT,    GNEViewNet::onCmdSetCustomGeometryPoint),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_RESET_GEOMETRYPOINT,     GNEViewNet::onCmdResetEndPoints),
    // toolbar views
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_VIEW_DEFAULT,   GNEViewNet::onCmdSetNeteditView),
    FXMAPFUNC(SEL_COMMAND,  MID_GNE_VIEW_JUPEDSIM,  GNEViewNet::onCmdSetNeteditView),
    // IntervalBar
    FXMAPFUNC(SEL_COMMAND, MID_GNE_INTERVALBAR_GENERICDATATYPE,     GNEViewNet::onCmdIntervalBarGenericDataType),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_INTERVALBAR_DATASET,             GNEViewNet::onCmdIntervalBarDataSet),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_INTERVALBAR_LIMITED,             GNEViewNet::onCmdIntervalBarLimit),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_INTERVALBAR_BEGIN,               GNEViewNet::onCmdIntervalBarSetBegin),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_INTERVALBAR_END,                 GNEViewNet::onCmdIntervalBarSetEnd),
    FXMAPFUNC(SEL_COMMAND, MID_GNE_INTERVALBAR_PARAMETER,           GNEViewNet::onCmdIntervalBarSetParameter),
};

// Object implementation
FXIMPLEMENT(GNEViewNet, GUISUMOAbstractView, GNEViewNetMap, ARRAYNUMBER(GNEViewNetMap))


// ===========================================================================
// member method definitions
// ===========================================================================
GNEViewNet::GNEViewNet(FXComposite* tmpParent, FXComposite* actualParent, GUIMainWindow& app,
                       GNEViewParent* viewParent, GNENet* net, GNEUndoList* undoList,
                       FXGLVisual* glVis, FXGLCanvas* share) :
    GUISUMOAbstractView(tmpParent, app, viewParent, net->getGrid(), glVis, share),
    myViewObjectsSelector(this),
    myEditModes(this),
    myTestingMode(this),
    myCommonCheckableButtons(this),
    myNetworkCheckableButtons(this),
    myDemandCheckableButtons(this),
    myDataCheckableButtons(this),
    myNetworkViewOptions(this),
    myDemandViewOptions(this),
    myDataViewOptions(this),
    myIntervalBar(this),
    myMoveSingleElement(this),
    myMoveMultipleElements(this),
    myVehicleOptions(this),
    myVehicleTypeOptions(this),
    mySaveElements(this),
    myTimeFormat(this),
    mySelectingArea(this),
    myEditNetworkElementShapes(this),
    myLockManager(this),
    myViewParent(viewParent),
    myNet(net),
    myUndoList(undoList) {
    // view must be the final member of actualParent
    reparent(actualParent);
    // Build edit modes
    buildEditModeControls();
    // set this net in Net
    myNet->setViewNet(this);
    // set drag delay
    ((GUIDanielPerspectiveChanger*)myChanger)->setDragDelay(100000000); // 100 milliseconds
    // Reset textures
    GUITextureSubSys::resetTextures();
    // init testing mode
    myTestingMode.initTestingMode();
    // update grid flags
    myNetworkViewOptions.menuCheckToggleGrid->setChecked(myVisualizationSettings->showGrid);
    myDemandViewOptions.menuCheckToggleGrid->setChecked(myVisualizationSettings->showGrid);
    // update junction shape flags
    const bool hide = !myVisualizationSettings->drawJunctionShape;
    myNetworkViewOptions.menuCheckToggleDrawJunctionShape->setChecked(hide);
    myDemandViewOptions.menuCheckToggleDrawJunctionShape->setChecked(hide);
    myDataViewOptions.menuCheckToggleDrawJunctionShape->setChecked(hide);
}


GNEViewNet::~GNEViewNet() {}


void
GNEViewNet::recalculateBoundaries() {
    if (myNet && makeCurrent()) {
        // declare boundary
        const Boundary maxBoundary(1000000000.0, 1000000000.0, -1000000000.0, -1000000000.0);
        // get all objects in boundary
        const std::vector<GUIGlID> GLIDs = getObjectsInBoundary(maxBoundary);
        //  finish make OpenGL context current
        makeNonCurrent();
        // declare set
        std::set<GNEAttributeCarrier*> ACs;
        // iterate over GUIGlIDs
        for (const auto& GLId : GLIDs) {
            GNEAttributeCarrier* AC = myNet->getAttributeCarriers()->retrieveAttributeCarrier(GLId);
            // Make sure that object exists
            if (AC && AC->getTagProperty().isPlacedInRTree()) {
                ACs.insert(AC);
            }
        }
        // interate over ACs
        for (const auto& AC : ACs) {
            // remove object and insert again with exaggeration
            myNet->getGrid().removeAdditionalGLObject(AC->getGUIGlObject());
            myNet->getGrid().addAdditionalGLObject(AC->getGUIGlObject(), AC->getGUIGlObject()->getExaggeration(*myVisualizationSettings));
        }
    }
}


void
GNEViewNet::doInit() {}


GUIGlID
GNEViewNet::getToolTipID() {
    if (myViewObjectsSelector.getGUIGlObjectFront()) {
        return myViewObjectsSelector.getGUIGlObjectFront()->getGlID();
    } else {
        return 0;
    }
}


void
GNEViewNet::buildViewToolBars(GUIGlChildWindow* v) {
    // build coloring tools
    {
        for (auto it_names : gSchemeStorage.getNames()) {
            v->getColoringSchemesCombo()->appendIconItem(it_names.c_str());
            if (it_names == myVisualizationSettings->name) {
                v->getColoringSchemesCombo()->setCurrentItem(v->getColoringSchemesCombo()->getNumItems() - 1);
            }
        }
    }
    // for junctions
    new MFXButtonTooltip(v->getLocatorPopup(), myViewParent->getGNEAppWindows()->getStaticTooltipMenu(),
                         std::string("\t") + TL("Locate Junctions") + std::string("\t") + TL("Locate a junction within the network. (Shift+J)"),
                         GUIIconSubSys::getIcon(GUIIcon::LOCATEJUNCTION), v, MID_HOTKEY_SHIFT_J_LOCATEJUNCTION, GUIDesignButtonPopup);
    // for edges
    new MFXButtonTooltip(v->getLocatorPopup(), myViewParent->getGNEAppWindows()->getStaticTooltipMenu(),
                         std::string("\t") + TL("Locate Edges") + std::string("\t") + TL("Locate an edge within the network. (Shift+E)"),
                         GUIIconSubSys::getIcon(GUIIcon::LOCATEEDGE), v, MID_HOTKEY_SHIFT_E_LOCATEEDGE, GUIDesignButtonPopup);
    // for walkingAreas
    new MFXButtonTooltip(v->getLocatorPopup(), myViewParent->getGNEAppWindows()->getStaticTooltipMenu(),
                         std::string("\t") + TL("Locate WalkingAreas") + std::string("\t") + TL("Locate a walkingArea within the network. (Shift+W)"),
                         GUIIconSubSys::getIcon(GUIIcon::LOCATEWALKINGAREA), v, MID_HOTKEY_SHIFT_W_LOCATEWALKINGAREA, GUIDesignButtonPopup);
    // for vehicles
    new MFXButtonTooltip(v->getLocatorPopup(), myViewParent->getGNEAppWindows()->getStaticTooltipMenu(),
                         std::string("\t") + TL("Locate Vehicles") + std::string("\t") + TL("Locate a vehicle within the network. (Shift+V)"),
                         GUIIconSubSys::getIcon(GUIIcon::LOCATEVEHICLE), v, MID_HOTKEY_SHIFT_V_LOCATEVEHICLE, GUIDesignButtonPopup);
    // for person
    new MFXButtonTooltip(v->getLocatorPopup(), myViewParent->getGNEAppWindows()->getStaticTooltipMenu(),
                         std::string("\t") + TL("Locate Persons") + std::string("\t") + TL("Locate a person within the network. (Shift+P)"),
                         GUIIconSubSys::getIcon(GUIIcon::LOCATEPERSON), v, MID_HOTKEY_SHIFT_P_LOCATEPERSON, GUIDesignButtonPopup);
    // for container
    new MFXButtonTooltip(v->getLocatorPopup(), myViewParent->getGNEAppWindows()->getStaticTooltipMenu(),
                         std::string("\t") + TL("Locate Containers") + std::string("\t") + TL("Locate a container within the network. (Shift+C)"),
                         GUIIconSubSys::getIcon(GUIIcon::LOCATECONTAINER), v, MID_HOTKEY_SHIFT_C_LOCATECONTAINER, GUIDesignButtonPopup);
    // for routes
    new MFXButtonTooltip(v->getLocatorPopup(), myViewParent->getGNEAppWindows()->getStaticTooltipMenu(),
                         std::string("\t") + TL("Locate Route") + std::string("\t") + TL("Locate a route within the network. (Shift+R)"),
                         GUIIconSubSys::getIcon(GUIIcon::LOCATEROUTE), v, MID_HOTKEY_SHIFT_R_LOCATEROUTE, GUIDesignButtonPopup);
    // for routes
    new MFXButtonTooltip(v->getLocatorPopup(), myViewParent->getGNEAppWindows()->getStaticTooltipMenu(),
                         std::string("\t") + TL("Locate Stops") + std::string("\t") + TL("Locate a stop within the network. (Shift+S)"),
                         GUIIconSubSys::getIcon(GUIIcon::LOCATESTOP), v, MID_HOTKEY_SHIFT_S_LOCATESTOP, GUIDesignButtonPopup);
    // for persons (currently unused)
    /*
    new MFXButtonTooltip(v->getLocatorPopup(),
                         std::string("\t") + TL("Locate Vehicle\tLocate a person within the network.",
                         GUIIconSubSys::getIcon(GUIIcon::LOCATEPERSON), &v, MID_LOCATEPERSON,
                         GUIDesignButtonPopup);
    */
    // for tls
    new MFXButtonTooltip(v->getLocatorPopup(), myViewParent->getGNEAppWindows()->getStaticTooltipMenu(),
                         std::string("\t") + TL("Locate TLS") + std::string("\t") + TL("Locate a tls within the network. (Shift+T)"),
                         GUIIconSubSys::getIcon(GUIIcon::LOCATETLS), v, MID_HOTKEY_SHIFT_T_LOCATETLS, GUIDesignButtonPopup);
    // for additional stuff
    new MFXButtonTooltip(v->getLocatorPopup(), myViewParent->getGNEAppWindows()->getStaticTooltipMenu(),
                         std::string("\t") + TL("Locate Additional") + std::string("\t") + TL("Locate an additional structure within the network. (Shift+A)"),
                         GUIIconSubSys::getIcon(GUIIcon::LOCATEADD), v, MID_HOTKEY_SHIFT_A_LOCATEADDITIONAL, GUIDesignButtonPopup);
    // for pois
    new MFXButtonTooltip(v->getLocatorPopup(), myViewParent->getGNEAppWindows()->getStaticTooltipMenu(),
                         std::string("\t") + TL("Locate PoI") + std::string("\t") + TL("Locate a PoI within the network. (Shift+O)"),
                         GUIIconSubSys::getIcon(GUIIcon::LOCATEPOI), v, MID_HOTKEY_SHIFT_O_LOCATEPOI, GUIDesignButtonPopup);
    // for polygons
    new MFXButtonTooltip(v->getLocatorPopup(), myViewParent->getGNEAppWindows()->getStaticTooltipMenu(),
                         std::string("\t") + TL("Locate Polygon") + std::string("\t") + TL("Locate a Polygon within the network. (Shift+L)"),
                         GUIIconSubSys::getIcon(GUIIcon::LOCATEPOLY), v, MID_HOTKEY_SHIFT_L_LOCATEPOLY, GUIDesignButtonPopup);
}


void
GNEViewNet::updateViewNet() const {
    // this call is only used for breakpoints (to check when view is updated)
    GUISUMOAbstractView::update();
}


void
GNEViewNet::forceSupemodeNetwork() {
    myEditModes.setSupermode(Supermode::NETWORK, true);
}


void
GNEViewNet::viewUpdated() {
    if (myEditModes.isJuPedSimView()) {
        // hide data button (and adjust width)
        myEditModes.dataButton->hide();
        // check network modes
        switch (myEditModes.networkEditMode) {
            case NetworkEditMode::NETWORK_CONNECT:
            case NetworkEditMode::NETWORK_TLS:
            case NetworkEditMode::NETWORK_PROHIBITION:
            case NetworkEditMode::NETWORK_WIRE:
            case NetworkEditMode::NETWORK_DECAL:
                myEditModes.setNetworkEditMode(NetworkEditMode::NETWORK_INSPECT);
                break;
            default:
                break;
        }
        // check demand modes
        switch (myEditModes.demandEditMode) {
            case DemandEditMode::DEMAND_MOVE:
            case DemandEditMode::DEMAND_VEHICLE:
            case DemandEditMode::DEMAND_STOP:
            case DemandEditMode::DEMAND_CONTAINER:
            case DemandEditMode::DEMAND_CONTAINERPLAN:
                myEditModes.setDemandEditMode(DemandEditMode::DEMAND_INSPECT);
                break;
            default:
                break;
        }
        // go to network mode if we're in data mode
        if (myEditModes.isCurrentSupermodeData()) {
            forceSupemodeNetwork();
        } else {
            // refresh current supermode
            myEditModes.setSupermode(myEditModes.currentSupermode, true);
        }
    } else {
        // show data button
        myEditModes.dataButton->show();
        // refresh current supermode
        myEditModes.setSupermode(myEditModes.currentSupermode, true);
    }
}


const GNEViewNetHelper::ViewObjectsSelector&
GNEViewNet::getViewObjectsSelector() const {
    return myViewObjectsSelector;
}


void
GNEViewNet::updateObjectsInBoundary(const Boundary& boundary) {
    // clear post drawing elements
    gViewObjectsHandler.clearSelectedElements();
    // set selection boundary in gObjectsInPosition
    gViewObjectsHandler.setSelectionBoundary(boundary);
    // push matrix
    GLHelper::pushMatrix();
    // enable draw for object under cursor
    myVisualizationSettings->drawForViewObjectsHandler = true;
    // draw all GL elements within the small boundary
    drawGLElements(boundary);
    // restore draw for object under cursor
    myVisualizationSettings->drawForViewObjectsHandler = false;
    // pop matrix
    GLHelper::popMatrix();
    // check if update front element
    if (myFrontAttributeCarrier) {
        gViewObjectsHandler.updateFrontElement(myFrontAttributeCarrier->getGUIGlObject());
    }
    // after draw elements, update objects under cursor
    myViewObjectsSelector.updateObjects();
}


void
GNEViewNet::updateObjectsInPosition(const Position& pos) {
    // clear post drawing elements
    gViewObjectsHandler.clearSelectedElements();
    // set selection position in gObjectsInPosition
    gViewObjectsHandler.setSelectionPosition(pos);
    // create an small boundary
    Boundary positionBoundary;
    positionBoundary.add(pos);
    positionBoundary.grow(POSITION_EPS);
    // push matrix
    GLHelper::pushMatrix();
    // enable draw for object under cursor
    myVisualizationSettings->drawForViewObjectsHandler = true;
    // draw all GL elements within the small boundary
    drawGLElements(positionBoundary);
    // check if filter edges that have the mouse over their geometry points
    if (myEditModes.isCurrentSupermodeNetwork() && myEditModes.networkEditMode == NetworkEditMode::NETWORK_MOVE) {
        gViewObjectsHandler.isolateEdgeGeometryPoints();
    }
    // restore draw for object under cursor
    myVisualizationSettings->drawForViewObjectsHandler = false;
    // pop matrix
    GLHelper::popMatrix();
    // check if update front element
    if (myFrontAttributeCarrier) {
        gViewObjectsHandler.updateFrontElement(myFrontAttributeCarrier->getGUIGlObject());
    }
    // after draw elements, update objects under cursor
    myViewObjectsSelector.updateObjects();
}


const GNEViewNetHelper::MoveSingleElementModul&
GNEViewNet::getMoveSingleElementValues() const {
    return myMoveSingleElement;
}


const GNEViewNetHelper::MoveMultipleElementModul&
GNEViewNet::getMoveMultipleElementValues() const {
    return myMoveMultipleElements;
}


void
GNEViewNet::buildSelectionACPopupEntry(GUIGLObjectPopupMenu* ret, GNEAttributeCarrier* AC) {
    if (AC->isAttributeCarrierSelected()) {
        GUIDesigns::buildFXMenuCommand(ret, TL("Remove from Selected"), GUIIconSubSys::getIcon(GUIIcon::FLAG_MINUS), this, MID_REMOVESELECT);
    } else {
        GUIDesigns::buildFXMenuCommand(ret, TL("Add to Selected"), GUIIconSubSys::getIcon(GUIIcon::FLAG_PLUS), this, MID_ADDSELECT);
    }
    new FXMenuSeparator(ret);
}


bool
GNEViewNet::setColorScheme(const std::string& name) {
    if (!gSchemeStorage.contains(name)) {
        return false;
    }
    if (myGUIDialogViewSettings != nullptr) {
        if (myGUIDialogViewSettings->getCurrentScheme() != name) {
            myGUIDialogViewSettings->setCurrentScheme(name);
        }
    }
    myVisualizationSettings = &gSchemeStorage.get(name.c_str());
    updateViewNet();
    return true;
}


void
GNEViewNet::openObjectDialogAtCursor(const FXEvent* /*ev*/) {
    // reimplemented from GUISUMOAbstractView due GNEOverlappedInspection
    ungrab();
    // make network current
    if (isEnabled() && myAmInitialised) {
        // check if we're cliking while alt button is pressed
        if (myMouseButtonKeyPressed.altKeyPressed()) {
            // set clicked popup position
            myClickedPopupPosition = getPositionInformation();
            // create cursor popup dialog for mark front element
            myPopup = new GUICursorDialog(GUIGLObjectPopupMenu::PopupType::FRONT_ELEMENT, this, myViewObjectsSelector.getGLObjects());
            // open popup dialog
            openPopupDialog();
        } else if (myViewObjectsSelector.getGLObjects().empty()) {
            openObjectDialog({myNet});
        } else {
            // declare filtered objects
            std::vector<GUIGlObject*> filteredGLObjects;
            // get GUIGLObject front
            GUIGlObject* overlappedElement = nullptr;
            // we need to check if we're inspecting a overlapping element
            if (myViewParent->getInspectorFrame()->getOverlappedInspection()->overlappedInspectionShown() &&
                    myViewParent->getInspectorFrame()->getOverlappedInspection()->checkSavedPosition(getPositionInformation()) &&
                    myInspectedAttributeCarriers.size() > 0) {
                overlappedElement = myInspectedAttributeCarriers.front()->getGUIGlObject();
                filteredGLObjects.push_back(overlappedElement);
            }
            bool connections = false;
            bool TLS = false;
            // fill filtered objects
            if ((myViewObjectsSelector.getGLObjects().size() == 1) && (myViewObjectsSelector.getGLObjects().back()->getType() == GLO_EDGE)) {
                // special case for edge geometry points (because edges uses the lane pop ups)
                filteredGLObjects.push_back(myViewObjectsSelector.getGLObjects().back());
            } else {
                for (const auto& glObject : myViewObjectsSelector.getGLObjects()) {
                    // always avoid edges
                    if (glObject->getType() == GLO_EDGE) {
                        continue;
                    }
                    if (glObject->getType() == GLO_CONNECTION) {
                        connections = true;
                    }
                    if (glObject->getType() == GLO_TLLOGIC) {
                        TLS = true;
                    }
                    filteredGLObjects.push_back(glObject);
                }
                auto it = filteredGLObjects.begin();
                if (connections) {
                    // filter junctions if there are connections
                    while (it != filteredGLObjects.end()) {
                        if ((*it)->getType() == GLO_JUNCTION) {
                            it = filteredGLObjects.erase(it);
                        } else {
                            it++;
                        }
                    }
                } else if (TLS) {
                    // filter all elements except TLLogic
                    while (it != filteredGLObjects.end()) {
                        if ((*it)->getType() != GLO_TLLOGIC) {
                            it = filteredGLObjects.erase(it);
                        } else {
                            it++;
                        }
                    }
                }
            }
            // remove duplicated elements using an unordered set
            auto itDuplicated = filteredGLObjects.begin();
            std::unordered_set<GUIGlObject*> unorderedSet;
            for (auto itElement = filteredGLObjects.begin(); itElement != filteredGLObjects.end(); itElement++) {
                if (unorderedSet.insert(*itElement).second) {
                    *itDuplicated++ = *itElement;
                }
            }
            filteredGLObjects.erase(itDuplicated, filteredGLObjects.end());
            // open object dialog
            openObjectDialog(filteredGLObjects);
        }
    }
}


void
GNEViewNet::openDeleteDialogAtCursor(const std::vector<GUIGlObject*>& GLObjects) {
    if (myPopup) {
        destroyPopup();
    }
    // set clicked popup position
    myClickedPopupPosition = getPositionInformation();
    // create cursor popup dialog for delete element
    myPopup = new GUICursorDialog(GUIGLObjectPopupMenu::PopupType::DELETE_ELEMENT, this, GLObjects);
    myCreatedPopup = true;
    // open popup dialog
    openPopupDialog();
}


void
GNEViewNet::openSelectDialogAtCursor(const std::vector<GUIGlObject*>& GLObjects) {
    if (myPopup) {
        destroyPopup();
    }
    // set clicked popup position
    myClickedPopupPosition = getPositionInformation();
    // create cursor popup dialog for select element
    myPopup = new GUICursorDialog(GUIGLObjectPopupMenu::PopupType::SELECT_ELEMENT, this, GLObjects);
    myCreatedPopup = true;
    // open popup dialog
    openPopupDialog();
}


void
GNEViewNet::saveVisualizationSettings() const {
    // first check if we have to save gui settings in a file (only used for testing purposes)
    const auto& neteditOptions = OptionsCont::getOptions();
    if (neteditOptions.getString("gui-testing.setting-output").size() > 0) {
        try {
            // open output device
            OutputDevice& output = OutputDevice::getDevice(neteditOptions.getString("gui-testing.setting-output"));
            // save view settings
            output.openTag(SUMO_TAG_VIEWSETTINGS);
            myVisualizationSettings->save(output);
            // save viewport (zoom, X, Y and Z)
            output.openTag(SUMO_TAG_VIEWPORT);
            output.writeAttr(SUMO_ATTR_ZOOM, myChanger->getZoom());
            output.writeAttr(SUMO_ATTR_X, myChanger->getXPos());
            output.writeAttr(SUMO_ATTR_Y, myChanger->getYPos());
            output.writeAttr(SUMO_ATTR_ANGLE, myChanger->getRotation());
            output.closeTag();
            output.closeTag();
            // close output device
            output.close();
        } catch (...) {
            WRITE_ERROR(TL("GUI-Settings cannot be saved in ") + neteditOptions.getString("gui-testing.setting-output"));
        }
    }
}


const GNEViewNetHelper::EditModes&
GNEViewNet::getEditModes() const {
    return myEditModes;
}


const GNEViewNetHelper::TestingMode&
GNEViewNet::getTestingMode() const {
    return myTestingMode;
}


const GNEViewNetHelper::NetworkViewOptions&
GNEViewNet::getNetworkViewOptions() const {
    return myNetworkViewOptions;
}


const GNEViewNetHelper::DemandViewOptions&
GNEViewNet::getDemandViewOptions() const {
    return myDemandViewOptions;
}


const GNEViewNetHelper::DataViewOptions&
GNEViewNet::getDataViewOptions() const {
    return myDataViewOptions;
}


const GNEViewNetHelper::MouseButtonKeyPressed&
GNEViewNet::getMouseButtonKeyPressed() const {
    return myMouseButtonKeyPressed;
}


const GNEViewNetHelper::EditNetworkElementShapes&
GNEViewNet::getEditNetworkElementShapes() const {
    return myEditNetworkElementShapes;
}


void
GNEViewNet::buildColorRainbow(const GUIVisualizationSettings& s, GUIColorScheme& scheme, int active, GUIGlObjectType objectType,
                              const GUIVisualizationRainbowSettings& rs) {
    assert(!scheme.isFixed());
    double minValue = std::numeric_limits<double>::infinity();
    double maxValue = -std::numeric_limits<double>::infinity();
    // retrieve range
    bool hasMissingData = false;
    if (objectType == GLO_LANE) {
        // XXX (see #3409) multi-colors are not currently handled. this is a quick hack
        if (active == 9) {
            active = 8; // segment height, fall back to start height
        } else if (active == 11) {
            active = 10; // segment incline, fall back to total incline
        }
        for (const auto& lane : myNet->getAttributeCarriers()->getLanes()) {
            const double val = lane.second->getColorValue(s, active);
            if (val == s.MISSING_DATA) {
                hasMissingData = true;
                continue;
            }
            minValue = MIN2(minValue, val);
            maxValue = MAX2(maxValue, val);
        }
    } else if (objectType == GLO_JUNCTION) {
        if (active == 3) {
            for (const auto& junction : myNet->getAttributeCarriers()->getJunctions()) {
                minValue = MIN2(minValue, junction.second->getPositionInView().z());
                maxValue = MAX2(maxValue, junction.second->getPositionInView().z());
            }
        }
    } else if (objectType == GLO_TAZRELDATA) {
        if (active == 4) {
            for (const auto& genericData : myNet->getAttributeCarriers()->getGenericDatas().at(SUMO_TAG_TAZREL)) {
                const double value = genericData.second->getColorValue(s, active);
                if (value == s.MISSING_DATA) {
                    continue;
                }
                minValue = MIN2(minValue, value);
                maxValue = MAX2(maxValue, value);
            }
        }
    }
    if (scheme.getName() == GUIVisualizationSettings::SCHEME_NAME_PERMISSION_CODE) {
        scheme.clear();
        // add threshold for every distinct value
        std::set<SVCPermissions> codes;
        for (const auto& lane : myNet->getAttributeCarriers()->getLanes()) {
            codes.insert(lane.second->getParentEdge()->getNBEdge()->getPermissions(lane.second->getIndex()));
        }
        int step = MAX2(1, 360 / (int)codes.size());
        int hue = 0;
        for (SVCPermissions p : codes) {
            scheme.addColor(RGBColor::fromHSV(hue, 1, 1), (double)p);
            hue = (hue + step) % 360;
        }
        return;
    }
    buildMinMaxRainbow(s, scheme, rs, minValue, maxValue, hasMissingData);
}


void
GNEViewNet::setStatusBarText(const std::string& text) {
    myApp->setStatusBarText(text);
}


void
GNEViewNet::resetLastClickedPosition() {
    myLastClickedPosition = Position::INVALID;
}


bool
GNEViewNet::autoSelectNodes() {
    if (myLockManager.isObjectLocked(GLO_JUNCTION, false)) {
        return false;
    } else {
        return myNetworkViewOptions.menuCheckExtendSelection->amChecked();
    }
}


void
GNEViewNet::setSelectorFrameScale(double selectionScale) {
    myVisualizationSettings->selectorFrameScale = selectionScale;
}


bool
GNEViewNet::changeAllPhases() const {
    return (myNetworkViewOptions.menuCheckChangeAllPhases->amChecked() != 0);
}


bool
GNEViewNet::showJunctionAsBubbles() const {
    return (myEditModes.networkEditMode == NetworkEditMode::NETWORK_MOVE) && (myNetworkViewOptions.menuCheckShowJunctionBubble->amChecked());
}


bool
GNEViewNet::checkMergeJunctions() {
    // first check if there are junctions to merging
    if (gViewObjectsHandler.getMergingJunctions().size() > 1) {
        // get junctions (this call is neccesary because merging junctions are constants)
        auto movedJunction = myNet->getAttributeCarriers()->retrieveJunction(gViewObjectsHandler.getMergingJunctions().at(0)->getID());
        auto targetJunction = myNet->getAttributeCarriers()->retrieveJunction(gViewObjectsHandler.getMergingJunctions().at(1)->getID());
        if (askMergeJunctions(movedJunction, targetJunction)) {
            // merge moved and targed junctions
            myNet->mergeJunctions(movedJunction, targetJunction, myUndoList);
            return true;
        }
    }
    return false;
}


bool
GNEViewNet::askMergeJunctions(const GNEJunction* movedJunction, const GNEJunction* targetJunction) {
    // optionally ask for confirmation
    if (!myNetworkViewOptions.menuCheckWarnAboutMerge->amChecked()) {
        WRITE_DEBUG("Opening FXMessageBox 'merge junctions'");
        // open question box
        const std::string header = TL("Confirm Junction Merger");
        const std::string body = (TLF("Do you wish to merge junctions '%' and '%'?\n('%' will be eliminated and its roads added to '%')", movedJunction->getMicrosimID(), targetJunction->getMicrosimID(), movedJunction->getMicrosimID(), targetJunction->getMicrosimID()));
        const FXuint answer = FXMessageBox::question(this, MBOX_YES_NO, header.c_str(), "%s", body.c_str());
        if (answer != 1) { //1:yes, 2:no, 4:esc
            // write warning if netedit is running in testing mode
            if (answer == 2) {
                WRITE_DEBUG("Closed FXMessageBox 'merge junctions' with 'No'");
            } else if (answer == 4) {
                WRITE_DEBUG("Closed FXMessageBox 'merge junctions' with 'ESC'");
            }
            return false;
        } else {
            // write warning if netedit is running in testing mode
            WRITE_DEBUG("Closed FXMessageBox 'merge junctions' with 'Yes'");
        }
    }
    return true;
}


bool
GNEViewNet::aksChangeSupermode(const std::string& operation, Supermode expectedSupermode) {
    // first check if ignore option is enabled
    if (OptionsCont::getOptions().getBool("ignore-supermode-question")) {
        return true;
    }
    std::string body;
    if (expectedSupermode == Supermode::NETWORK) {
        body = (operation + TL(" requires switch to network mode. Continue?"));
    } else if (expectedSupermode == Supermode::DEMAND) {
        body = (operation + TL(" requires switch to demand mode. Continue?"));
    } else if (expectedSupermode == Supermode::DATA) {
        body = (operation + TL(" requires switch to data mode. Continue?"));
    } else {
        throw ProcessError("invalid expected supermode");
    }
    // open question box
    const auto answer = FXMessageBox::question(myApp, MBOX_YES_NO, TL("Confirm switch mode"), "%s", body.c_str());
    // restore focus to view net
    setFocus();
    // return answer
    if (answer == MBOX_CLICKED_YES) {
        myEditModes.setSupermode(expectedSupermode, true);
        return true;
    } else {
        return false;
    }
}


bool
GNEViewNet::selectingDetectorsTLSMode() const {
    // separate conditions for code legibly
    const bool TLSMode = (myEditModes.isCurrentSupermodeNetwork() && (myEditModes.networkEditMode == NetworkEditMode::NETWORK_TLS));
    const bool selectingDetectors = myViewParent->getTLSEditorFrame()->getTLSAttributes()->isSetDetectorsToggleButtonEnabled();
    return (TLSMode && selectingDetectors);
}


bool
GNEViewNet::selectingJunctionsTLSMode() const {
    // separate conditions for code legibly
    const bool TLSMode = (myEditModes.isCurrentSupermodeNetwork() && (myEditModes.networkEditMode == NetworkEditMode::NETWORK_TLS));
    const bool selectingJunctions = myViewParent->getTLSEditorFrame()->getTLSJunction()->isJoiningJunctions();
    return (TLSMode && selectingJunctions);
}


GNEViewNetHelper::SaveElements&
GNEViewNet::getSaveElements() {
    return mySaveElements;
}


GNEViewNetHelper::TimeFormat&
GNEViewNet::getTimeFormat() {
    return myTimeFormat;
}


bool
GNEViewNet::restrictLane(GNELane* lane, SUMOVehicleClass vclass) {
    // Get selected lanes
    const auto selectedLanes = myNet->getAttributeCarriers()->getSelectedLanes();
    // Declare map of edges and lanes
    std::map<GNEEdge*, GNELane*> mapOfEdgesAndLanes;
    // Iterate over selected lanes
    for (const auto& selectedLane : selectedLanes) {
        mapOfEdgesAndLanes[myNet->getAttributeCarriers()->retrieveEdge(selectedLane->getParentEdge()->getID())] = selectedLane;
    }
    // Throw warning dialog if there hare multiple lanes selected in the same edge
    if (mapOfEdgesAndLanes.size() != selectedLanes.size()) {
        const std::string header = TL("Multiple lane in the same edge selected");
        const std::string bodyA = TL("There are selected lanes that belong to the same edge.");
        const std::string bodyB = TLF("Only one lane per edge will be restricted to %.", toString(vclass));
        FXMessageBox::information(getApp(), MBOX_OK, header.c_str(), "%s", (bodyA + std::string("\n") + bodyB).c_str());
    }
    // If we handeln a set of lanes
    if (mapOfEdgesAndLanes.size() > 0) {
        // declare counter for number of Sidewalks
        int counter = 0;
        // iterate over selected lanes
        for (const auto& edgeLane : mapOfEdgesAndLanes) {
            if (edgeLane.first->hasRestrictedLane(vclass)) {
                counter++;
            }
        }
        // if all edges parent own a Sidewalk, stop function
        if (counter == (int)mapOfEdgesAndLanes.size()) {
            const std::string header = TLF("Set vclass to % for selected lanes", toString(vclass));
            const std::string body = TL("All lanes own already another lane in the same edge with a restriction for ");
            FXMessageBox::information(getApp(), MBOX_OK, header.c_str(), "%s", (body + toString(vclass) + ".").c_str());
            return 0;
        } else {
            WRITE_DEBUG("Opening FXMessageBox 'restrict lanes'");
            // Ask confirmation to user
            const std::string header = TLF("Set vclass to % for selected lanes", toString(vclass));
            const std::string body = TLF("% lanes will be restricted to %. Continue?", toString(mapOfEdgesAndLanes.size() - counter), toString(vclass));
            FXuint answer = FXMessageBox::question(getApp(), MBOX_YES_NO, header.c_str(), "%s", body.c_str());
            if (answer != 1) { //1:yes, 2:no, 4:esc
                // write warning if netedit is running in testing mode
                if (answer == 2) {
                    WRITE_DEBUG("Closed FXMessageBox 'restrict lanes' with 'No'");
                } else if (answer == 4) {
                    WRITE_DEBUG("Closed FXMessageBox 'restrict lanes' with 'ESC'");
                }
                return 0;
            } else {
                // write warning if netedit is running in testing mode
                WRITE_DEBUG("Closed FXMessageBox 'restrict lanes' with 'Yes'");
            }
        }
        // begin undo operation
        myUndoList->begin(lane, "restrict lanes to " + toString(vclass));
        // iterate over selected lanes
        for (const auto& edgeLane : mapOfEdgesAndLanes) {
            // Transform lane to Sidewalk
            myNet->restrictLane(vclass, edgeLane.second, myUndoList);
        }
        // end undo operation
        myUndoList->end();
    } else {
        // If only have a single lane, start undo/redo operation
        myUndoList->begin(lane, TL("restrict lane to ") + toString(vclass));
        // Transform lane to Sidewalk
        myNet->restrictLane(vclass, lane, myUndoList);
        // end undo operation
        myUndoList->end();
    }
    return 1;
}


bool
GNEViewNet::addRestrictedLane(GNELane* lane, SUMOVehicleClass vclass, const bool insertAtFront) {
    // Get selected edges
    const auto selectedEdges = myNet->getAttributeCarriers()->getSelectedEdges();
    // get selected lanes
    const auto selectedLanes = myNet->getAttributeCarriers()->getSelectedLanes();
    // Declare set of edges
    std::set<GNEEdge*> setOfEdges;
    // Fill set of edges with vector of edges
    for (const auto& edge : selectedEdges) {
        setOfEdges.insert(edge);
    }
    // iterate over selected lanes
    for (const auto& selectedLane : selectedLanes) {
        // Insert pointer to edge into set of edges (To avoid duplicates)
        setOfEdges.insert(myNet->getAttributeCarriers()->retrieveEdge(selectedLane->getParentEdge()->getID()));
    }
    // If we handeln a set of edges
    if (lane->isAttributeCarrierSelected() || lane->getParentEdge()->isAttributeCarrierSelected()) {
        // declare counter for number of restrictions
        int counter = 0;
        // iterate over set of edges
        for (const auto& edge : setOfEdges) {
            // update counter if edge has already a restricted lane of type "vclass"
            if (edge->hasRestrictedLane(vclass)) {
                counter++;
            }
        }
        // if all lanes own a Sidewalk, stop function
        if (counter == (int)setOfEdges.size()) {
            const std::string header = TLF("Add vclass % to selected lanes", toString(vclass));
            const std::string body = TLF("All lanes own already another lane in the same edge with a restriction to %.", toString(vclass));
            FXMessageBox::information(getApp(), MBOX_OK, header.c_str(), "%s", body.c_str());
            return 0;
        } else {
            WRITE_DEBUG("Opening FXMessageBox 'restrict lanes'");
            // Ask confirmation to user
            const std::string header = TLF("Add vclass % to selected lanes", toString(vclass));
            const std::string body = TLF("% restrictions to % will be added. Continue?", toString(setOfEdges.size() - counter), toString(vclass));
            FXuint answer = FXMessageBox::question(getApp(), MBOX_YES_NO, header.c_str(), "%s", body.c_str());
            if (answer != 1) { //1:yes, 2:no, 4:esc
                // write warning if netedit is running in testing mode
                if (answer == 2) {
                    WRITE_DEBUG("Closed FXMessageBox 'restrict lanes' with 'No'");
                } else if (answer == 4) {
                    WRITE_DEBUG("Closed FXMessageBox 'restrict lanes' with 'ESC'");
                }
                return 0;
            } else {
                // write warning if netedit is running in testing mode
                WRITE_DEBUG("Closed FXMessageBox 'restrict lanes' with 'Yes'");
            }
        }
        // begin undo operation
        myUndoList->begin(lane, TL("add restrictions for ") + toString(vclass));
        // iterate over set of edges
        for (const auto& edge : setOfEdges) {
            // add restricted lane (guess target)
            myNet->addRestrictedLane(vclass, edge, -1, myUndoList);
        }
        // end undo operation
        myUndoList->end();
    } else {
        // If only have a single lane, start undo/redo operation
        myUndoList->begin(lane, TL("add vclass for ") + toString(vclass));
        // Add restricted lane
        if (vclass == SVC_PEDESTRIAN) {
            // always add pedestrian lanes on the right
            myNet->addRestrictedLane(vclass, lane->getParentEdge(), 0, myUndoList);
        } else if (vclass == SVC_IGNORING) {
            if (insertAtFront) {
                myNet->addGreenVergeLane(lane->getParentEdge(), lane->getIndex() + 1, myUndoList);
            } else {
                myNet->addGreenVergeLane(lane->getParentEdge(), lane->getIndex(), myUndoList);
            }
        } else if (lane->getParentEdge()->getLanes().size() == 1) {
            // guess insertion position if there is only 1 lane
            myNet->addRestrictedLane(vclass, lane->getParentEdge(), -1, myUndoList);
        } else {
            myNet->addRestrictedLane(vclass, lane->getParentEdge(), lane->getIndex(), myUndoList);
        }
        // end undo/redo operation
        myUndoList->end();
    }
    return 1;
}


bool
GNEViewNet::removeRestrictedLane(GNELane* lane, SUMOVehicleClass vclass) {
    // Get selected edges
    const auto selectedEdges = myNet->getAttributeCarriers()->getSelectedEdges();
    // get selected lanes
    const auto selectedLanes = myNet->getAttributeCarriers()->getSelectedLanes();
    // Declare set of edges
    std::set<GNEEdge*> setOfEdges;
    // Fill set of edges with vector of edges
    for (const auto& edge : selectedEdges) {
        setOfEdges.insert(edge);
    }
    // iterate over selected lanes
    for (const auto& selectedLane : selectedLanes) {
        // Insert pointer to edge into set of edges (To avoid duplicates)
        setOfEdges.insert(myNet->getAttributeCarriers()->retrieveEdge(selectedLane->getParentEdge()->getID()));
    }
    // If we handeln a set of edges
    if (setOfEdges.size() > 0) {
        // declare counter for number of restrictions
        int counter = 0;
        // iterate over set of edges
        for (const auto& edge : setOfEdges) {
            // update counter if edge has already a restricted lane of type "vclass"
            if (edge->hasRestrictedLane(vclass)) {
                counter++;
            }
        }
        // if all lanes don't own a Sidewalk, stop function
        if (counter == 0) {
            const std::string header = TLF("Remove vclass % from selected lanes", toString(vclass));
            const std::string body = TLF("The selected lanes and edges don't have a restriction to %.", toString(vclass));
            FXMessageBox::information(getApp(), MBOX_OK, header.c_str(), "%s", body.c_str());
            return 0;
        } else {
            WRITE_DEBUG("Opening FXMessageBox 'restrict lanes'");
            // Ask confirmation to user
            const std::string header = TLF("Remove vclass % from selected lanes", toString(vclass));
            const std::string body = TLF("% restrictions to % will be removed. Continue?", toString(setOfEdges.size() - counter), toString(vclass));
            FXuint answer = FXMessageBox::question(getApp(), MBOX_YES_NO, header.c_str(), "%s", body.c_str());
            if (answer != 1) { //1:yes, 2:no, 4:esc
                // write warning if netedit is running in testing mode
                if (answer == 2) {
                    WRITE_DEBUG("Closed FXMessageBox 'restrict lanes' with 'No'");
                } else if (answer == 4) {
                    WRITE_DEBUG("Closed FXMessageBox 'restrict lanes' with 'ESC'");
                }
                return 0;
            } else {
                // write warning if netedit is running in testing mode
                WRITE_DEBUG("Closed FXMessageBox 'restrict lanes' with 'Yes'");
            }
        }
        // begin undo operation
        myUndoList->begin(lane, "Remove restrictions for " + toString(vclass));
        // iterate over set of edges
        for (const auto& edge : setOfEdges) {
            // add Sidewalk
            myNet->removeRestrictedLane(vclass, edge, myUndoList);
        }
        // end undo operation
        myUndoList->end();
    } else {
        // If only have a single lane, start undo/redo operation
        myUndoList->begin(lane, TL("Remove vclass for ") + toString(vclass));
        // Remove Sidewalk
        myNet->removeRestrictedLane(vclass, lane->getParentEdge(), myUndoList);
        // end undo/redo operation
        myUndoList->end();
    }
    return 1;
}


GNEViewNet::GNEViewNet() :
    myViewObjectsSelector(this),
    myEditModes(this),
    myTestingMode(this),
    myCommonCheckableButtons(this),
    myNetworkCheckableButtons(this),
    myDemandCheckableButtons(this),
    myDataCheckableButtons(this),
    myNetworkViewOptions(this),
    myDemandViewOptions(this),
    myDataViewOptions(this),
    myIntervalBar(this),
    myMoveSingleElement(this),
    myMoveMultipleElements(this),
    myVehicleOptions(this),
    myVehicleTypeOptions(this),
    mySaveElements(this),
    myTimeFormat(this),
    mySelectingArea(this),
    myEditNetworkElementShapes(this),
    myLockManager(this) {
}


std::vector<std::string>
GNEViewNet::getEdgeLaneParamKeys(bool edgeKeys) const {
    std::set<std::string> keys;
    for (const NBEdge* e : myNet->getEdgeCont().getAllEdges()) {
        if (edgeKeys) {
            for (const auto& item : e->getParametersMap()) {
                keys.insert(item.first);
            }
            for (const auto& con : e->getConnections()) {
                for (const auto& item : con.getParametersMap()) {
                    keys.insert(item.first);
                }
            }
        } else {
            for (const auto& lane : e->getLanes()) {
                int i = 0;
                for (const auto& item : lane.getParametersMap()) {
                    keys.insert(item.first);
                }
                for (const auto& con : e->getConnectionsFromLane(i)) {
                    for (const auto& item : con.getParametersMap()) {
                        keys.insert(item.first);
                    }
                }
                i++;
            }
        }
    }
    return std::vector<std::string>(keys.begin(), keys.end());
}


std::vector<std::string>
GNEViewNet::getEdgeDataAttrs() const {
    std::set<std::string> keys;
    for (const auto& genericData : myNet->getAttributeCarriers()->getGenericDatas().at(GNE_TAG_EDGEREL_SINGLE)) {
        for (const auto& parameter : genericData.second->getACParametersMap()) {
            keys.insert(parameter.first);
        }
    }
    return std::vector<std::string>(keys.begin(), keys.end());
}


std::vector<std::string>
GNEViewNet::getRelDataAttrs() const {
    std::set<std::string> keys;
    for (const auto& genericData : myNet->getAttributeCarriers()->getGenericDatas().at(SUMO_TAG_TAZREL)) {
        for (const auto& parameter : genericData.second->getACParametersMap()) {
            keys.insert(parameter.first);
        }
    }
    for (const auto& genericData : myNet->getAttributeCarriers()->getGenericDatas().at(SUMO_TAG_EDGEREL)) {
        for (const auto& parameter : genericData.second->getACParametersMap()) {
            keys.insert(parameter.first);
        }
    }
    return std::vector<std::string>(keys.begin(), keys.end());
}


bool
GNEViewNet::checkSelectEdges() const {
    if ((myNetworkViewOptions.selectEdges() && !myMouseButtonKeyPressed.shiftKeyPressed()) ||
            (!myNetworkViewOptions.selectEdges() && myMouseButtonKeyPressed.shiftKeyPressed())) {
        return true;
    } else {
        return false;
    }
}


int
GNEViewNet::getDrawingToggle() const {
    return myDrawingToggle;
}


int
GNEViewNet::doPaintGL(int mode, const Boundary& bound) {
    // set lefthand and laneIcons
    myVisualizationSettings->lefthand = OptionsCont::getOptions().getBool("lefthand");
    myVisualizationSettings->disableLaneIcons = OptionsCont::getOptions().getBool("disable-laneIcons");
    // first step: update objects under cursor
    updateObjectsInPosition(myNet->getViewNet()->getPositionInformation());
    // set render modes
    glRenderMode(mode);
    glMatrixMode(GL_MODELVIEW);
    GLHelper::pushMatrix();
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_ALPHA_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);
    // visualize rectangular selection
    mySelectingArea.drawRectangleSelection(myVisualizationSettings->colorSettings.selectionColor);
    // draw certain elements only if we aren't in rectangle selection mode
    if (!myVisualizationSettings->drawForRectangleSelection) {
        // draw decals
        drawDecals();
        // draw grid (and update grid button)
        drawGrid();
        // update show connections
        myNetworkViewOptions.menuCheckShowConnections->setChecked(myVisualizationSettings->showLane2Lane);
        // draw temporal junction
        drawTemporalJunction();
        // draw temporal drawing shape
        drawTemporalDrawingShape();
        // draw testing elements
        myTestingMode.drawTestingElements(myApp);
        // draw temporal E2 multilane detectors
        myViewParent->getAdditionalFrame()->getConsecutiveLaneSelector()->drawTemporalConsecutiveLanePath();
        // draw temporal overhead wires
        myViewParent->getWireFrame()->getConsecutiveLaneSelector()->drawTemporalConsecutiveLanePath();
        // draw temporal trip/flow route
        myViewParent->getVehicleFrame()->getPathCreator()->drawTemporalRoute(*myVisualizationSettings);
        // draw temporal person plan route
        myViewParent->getPersonFrame()->getPlanCreator()->drawTemporalRoute(*myVisualizationSettings);
        myViewParent->getPersonPlanFrame()->getPlanCreator()->drawTemporalRoute(*myVisualizationSettings);
        // draw temporal container plan route
        myViewParent->getContainerFrame()->getPlanCreator()->drawTemporalRoute(*myVisualizationSettings);
        myViewParent->getContainerPlanFrame()->getPlanCreator()->drawTemporalRoute(*myVisualizationSettings);
        // draw temporal route
        myViewParent->getRouteFrame()->getPathCreator()->drawTemporalRoute(*myVisualizationSettings);
        // draw temporal edgeRelPath
        myViewParent->getEdgeRelDataFrame()->getPathCreator()->drawTemporalRoute(*myVisualizationSettings);
    }
    // check menu checks of supermode demand
    if (myEditModes.isCurrentSupermodeDemand()) {
        // enable or disable menuCheckShowAllPersonPlans depending of there is a locked person
        if (myDemandViewOptions.getLockedPerson()) {
            myDemandViewOptions.menuCheckShowAllPersonPlans->disable();
        } else {
            myDemandViewOptions.menuCheckShowAllPersonPlans->enable();
        }
    }
    // clear pathDraw
    myNet->getPathManager()->getPathDraw()->clearPathDraw();
    // update ignore hide by zoom
    myVisualizationSettings->updateIgnoreHideByZoom();
    // draw network (boundary
    myNet->drawGL(*myVisualizationSettings);
    // draw all GL elements
    int hits = drawGLElements(bound);
    // draw temporal split junction
    drawTemporalSplitJunction();
    // draw temporal roundabout
    drawTemporalRoundabout();
    // draw temporal lines between E1 detectors and junctions in TLS Mode
    drawTemporalE1TLSLines();
    // draw temporal lines between junctions in TLS Mode
    drawTemporalJunctionTLSLines();
    // draw netedit attributes references
    drawNeteditAttributesReferences();
    // draw test circle
    drawTestsCircle();
    // pop draw matrix
    GLHelper::popMatrix();
    // update interval bar
    myIntervalBar.markForUpdate();
    // check if recopute boundaries (Deactivated, continue after 1.14 release)
    /*
        if (gObjectsInPosition.recomputeBoundaries != GLO_NETWORK) {
            myNet->getGrid().updateBoundaries(gObjectsInPosition.recomputeBoundaries);
        }
    */
    return hits;
}


long
GNEViewNet::onLeftBtnPress(FXObject*, FXSelector, void* eventData) {
    // set focus in view net
    setFocus();
    // update MouseButtonKeyPressed
    myMouseButtonKeyPressed.update(eventData);
    // process left button press function depending of supermode
    if (myEditModes.isCurrentSupermodeNetwork()) {
        processLeftButtonPressNetwork(eventData);
    } else if (myEditModes.isCurrentSupermodeDemand()) {
        processLeftButtonPressDemand(eventData);
    } else if (myEditModes.isCurrentSupermodeData()) {
        processLeftButtonPressData(eventData);
    }
    // update cursor
    updateCursor();
    // update view
    updateViewNet();
    return 1;
}


long
GNEViewNet::onLeftBtnRelease(FXObject* obj, FXSelector sel, void* eventData) {
    // avoid closing Popup dialog in Linux
    if (myCreatedPopup) {
        myCreatedPopup = false;
        return 1;
    }
    // process parent function
    GUISUMOAbstractView::onLeftBtnRelease(obj, sel, eventData);
    // update MouseButtonKeyPressed
    myMouseButtonKeyPressed.update(eventData);
    // process left button release function depending of supermode
    if (myEditModes.isCurrentSupermodeNetwork()) {
        processLeftButtonReleaseNetwork();
    } else if (myEditModes.isCurrentSupermodeDemand()) {
        processLeftButtonReleaseDemand();
    } else if (myEditModes.isCurrentSupermodeData()) {
        processLeftButtonReleaseData();
    }
    // update cursor
    updateCursor();
    // update view
    updateViewNet();
    return 1;
}


long
GNEViewNet::onMiddleBtnPress(FXObject* obj, FXSelector sel, void* eventData) {
    // process parent function
    GUISUMOAbstractView::onMiddleBtnPress(obj, sel, eventData);
    // update cursor
    updateCursor();
    // update view
    updateViewNet();
    return 1;
}


long
GNEViewNet::onMiddleBtnRelease(FXObject* obj, FXSelector sel, void* eventData) {
    // process parent function
    GUISUMOAbstractView::onMiddleBtnRelease(obj, sel, eventData);
    // update cursor
    updateCursor();
    // update view
    updateViewNet();
    return 1;
}


long
GNEViewNet::onRightBtnPress(FXObject* obj, FXSelector sel, void* eventData) {
    // update MouseButtonKeyPressed
    myMouseButtonKeyPressed.update(eventData);
    // update cursor
    updateCursor();
    if ((myEditModes.networkEditMode == NetworkEditMode::NETWORK_SHAPE) && myViewParent->getShapeFrame()->getDrawingShapeModule()->isDrawing()) {
        // disable right button press during drawing polygon
        return 1;
    } else {
        return GUISUMOAbstractView::onRightBtnPress(obj, sel, eventData);
    }
}


long
GNEViewNet::onRightBtnRelease(FXObject* obj, FXSelector sel, void* eventData) {
    // update MouseButtonKeyPressed
    myMouseButtonKeyPressed.update(eventData);
    // update cursor
    updateCursor();
    // disable right button release during drawing polygon
    if ((myEditModes.networkEditMode == NetworkEditMode::NETWORK_SHAPE) && myViewParent->getShapeFrame()->getDrawingShapeModule()->isDrawing()) {
        return 1;
    } else {
        return GUISUMOAbstractView::onRightBtnRelease(obj, sel, eventData);
    }
}


long
GNEViewNet::onMouseMove(FXObject* obj, FXSelector sel, void* eventData) {
    // process mouse move in GUISUMOAbstractView
    GUISUMOAbstractView::onMouseMove(obj, sel, eventData);
    // update MouseButtonKeyPressed
    myMouseButtonKeyPressed.update(eventData);
    // update cursor
    updateCursor();
    // process mouse move function depending of supermode
    if (myEditModes.isCurrentSupermodeNetwork()) {
        processMoveMouseNetwork(myMouseButtonKeyPressed.mouseLeftButtonPressed());
    } else if (myEditModes.isCurrentSupermodeDemand()) {
        processMoveMouseDemand(myMouseButtonKeyPressed.mouseLeftButtonPressed());
    } else if (myEditModes.isCurrentSupermodeData()) {
        processMoveMouseData(myMouseButtonKeyPressed.mouseLeftButtonPressed());
    }
    // update view
    updateViewNet();
    return 1;
}


long
GNEViewNet::onKeyPress(FXObject* o, FXSelector sel, void* eventData) {
    // update MouseButtonKeyPressed
    myMouseButtonKeyPressed.update(eventData);
    // update cursor
    updateCursor();
    // continue depending of current edit mode
    if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_CREATE_EDGE) {
        // update viewNet (for temporal junction)
        updateViewNet();
    } else if ((myEditModes.networkEditMode == NetworkEditMode::NETWORK_SHAPE) && myViewParent->getShapeFrame()->getDrawingShapeModule()->isDrawing()) {
        // change "delete last created point" depending of shift key
        myViewParent->getShapeFrame()->getDrawingShapeModule()->setDeleteLastCreatedPoint(myMouseButtonKeyPressed.shiftKeyPressed());
        updateViewNet();
    } else if ((myEditModes.networkEditMode == NetworkEditMode::NETWORK_TAZ) && myViewParent->getTAZFrame()->getDrawingShapeModule()->isDrawing()) {
        // change "delete last created point" depending of shift key
        myViewParent->getTAZFrame()->getDrawingShapeModule()->setDeleteLastCreatedPoint(myMouseButtonKeyPressed.shiftKeyPressed());
        updateViewNet();
    } else if ((myEditModes.networkEditMode == NetworkEditMode::NETWORK_MOVE) || (myEditModes.demandEditMode == DemandEditMode::DEMAND_MOVE)) {
        updateViewNet();
    }
    return GUISUMOAbstractView::onKeyPress(o, sel, eventData);
}


long
GNEViewNet::onKeyRelease(FXObject* o, FXSelector sel, void* eventData) {
    // update MouseButtonKeyPressed
    myMouseButtonKeyPressed.update(eventData);
    // update cursor
    updateCursor();
    // continue depending of current edit mode
    if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_CREATE_EDGE) {
        // update viewNet (for temporal junction)
        updateViewNet();
    } else if ((myEditModes.networkEditMode == NetworkEditMode::NETWORK_SHAPE) && myViewParent->getShapeFrame()->getDrawingShapeModule()->isDrawing()) {
        // change "delete last created point" depending of shift key
        myViewParent->getShapeFrame()->getDrawingShapeModule()->setDeleteLastCreatedPoint(myMouseButtonKeyPressed.shiftKeyPressed());
        updateViewNet();
    } else if ((myEditModes.networkEditMode == NetworkEditMode::NETWORK_TAZ) && myViewParent->getTAZFrame()->getDrawingShapeModule()->isDrawing()) {
        // change "delete last created point" depending of shift key
        myViewParent->getTAZFrame()->getDrawingShapeModule()->setDeleteLastCreatedPoint(myMouseButtonKeyPressed.shiftKeyPressed());
        updateViewNet();
    } else if ((myEditModes.networkEditMode == NetworkEditMode::NETWORK_MOVE) || (myEditModes.demandEditMode == DemandEditMode::DEMAND_MOVE)) {
        updateViewNet();
    }
    // check if selecting using rectangle has to be disabled
    if (mySelectingArea.selectingUsingRectangle && !myMouseButtonKeyPressed.shiftKeyPressed()) {
        mySelectingArea.selectingUsingRectangle = false;
        updateViewNet();
    }
    return GUISUMOAbstractView::onKeyRelease(o, sel, eventData);
}


void
GNEViewNet::abortOperation(bool clearSelection) {
    // steal focus from any text fields and place it over view net
    setFocus();
    // check what supermode is enabled
    if (myEditModes.isCurrentSupermodeNetwork()) {
        // abort operation depending of current mode
        if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_CREATE_EDGE) {
            // abort edge creation in create edge frame
            myViewParent->getCreateEdgeFrame()->abortEdgeCreation();
        } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_SELECT) {
            mySelectingArea.selectingUsingRectangle = false;
            // check if current selection has to be cleaned
            if (clearSelection) {
                myViewParent->getSelectorFrame()->clearCurrentSelection();
            }
        } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_CONNECT) {
            // abort changes in Connector Frame
            myViewParent->getConnectorFrame()->getConnectionModifications()->onCmdCancelModifications(0, 0, 0);
        } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_TLS) {
            // continue depending of current TLS frame state
            if (myViewParent->getTLSEditorFrame()->getTLSAttributes()->isSetDetectorsToggleButtonEnabled()) {
                myViewParent->getTLSEditorFrame()->getTLSAttributes()->disableE1DetectorMode();
            } else if (myViewParent->getTLSEditorFrame()->getTLSJunction()->isJoiningJunctions()) {
                myViewParent->getTLSEditorFrame()->getTLSJunction()->onCmdCancelJoin(nullptr, 0, nullptr);
            } else {
                myViewParent->getTLSEditorFrame()->getTLSDefinition()->discardChanges(false);
            }
        } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_MOVE) {
            myEditNetworkElementShapes.stopEditCustomShape();
        } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_CROSSING) {
            myViewParent->getCrossingFrame()->clearEdgesHotkey();
        } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_SHAPE) {
            // abort current drawing
            myViewParent->getShapeFrame()->getDrawingShapeModule()->abortDrawing();
        } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_TAZ) {
            if (myViewParent->getTAZFrame()->getDrawingShapeModule()->isDrawing()) {
                // abort current drawing
                myViewParent->getTAZFrame()->getDrawingShapeModule()->abortDrawing();
            } else if (myViewParent->getTAZFrame()->getCurrentTAZModule()->getTAZ() != nullptr) {
                // finish current editing TAZ
                myViewParent->getTAZFrame()->getCurrentTAZModule()->setTAZ(nullptr);
            }
        } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_PROHIBITION) {
            myViewParent->getProhibitionFrame()->getSelectionModul()->onCmdCancel(nullptr, 0, nullptr);
        } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_ADDITIONAL) {
            // abort both network elements selections
            myViewParent->getAdditionalFrame()->getEdgesSelector()->clearSelection();
            myViewParent->getAdditionalFrame()->getLanesSelector()->clearSelection();
            // abort path
            myViewParent->getAdditionalFrame()->getConsecutiveLaneSelector()->abortPathCreation();
        } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_WIRE) {
            // abort path
            myViewParent->getWireFrame()->getConsecutiveLaneSelector()->abortPathCreation();
        }
    } else if (myEditModes.isCurrentSupermodeDemand()) {
        // abort operation depending of current mode
        if (myEditModes.demandEditMode == DemandEditMode::DEMAND_SELECT) {
            mySelectingArea.selectingUsingRectangle = false;
            // check if current selection has to be cleaned
            if (clearSelection) {
                myViewParent->getSelectorFrame()->clearCurrentSelection();
            }
        } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_ROUTE) {
            myViewParent->getRouteFrame()->getPathCreator()->abortPathCreation();
        } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_VEHICLE) {
            myViewParent->getVehicleFrame()->getPathCreator()->abortPathCreation();
        } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_PERSON) {
            myViewParent->getPersonFrame()->getPlanCreator()->abortPathCreation();
        } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_PERSONPLAN) {
            myViewParent->getPersonPlanFrame()->getPlanCreator()->abortPathCreation();
        } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_CONTAINER) {
            myViewParent->getContainerFrame()->getPlanCreator()->abortPathCreation();
        } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_CONTAINERPLAN) {
            myViewParent->getContainerPlanFrame()->getPlanCreator()->abortPathCreation();
        }
    } else if (myEditModes.isCurrentSupermodeData()) {
        // abort operation depending of current mode
        if (myEditModes.dataEditMode == DataEditMode::DATA_SELECT) {
            mySelectingArea.selectingUsingRectangle = false;
            // check if current selection has to be cleaned
            if (clearSelection) {
                myViewParent->getSelectorFrame()->clearCurrentSelection();
            }
        } else if (myEditModes.dataEditMode == DataEditMode::DATA_EDGERELDATA) {
            myViewParent->getEdgeRelDataFrame()->getPathCreator()->abortPathCreation();
        } else if (myEditModes.dataEditMode == DataEditMode::DATA_TAZRELDATA) {
            myViewParent->getTAZRelDataFrame()->clearTAZSelection();
        }
    }
    // abort undo list
    myUndoList->abortAllChangeGroups();
    // update view
    updateViewNet();
}


void
GNEViewNet::hotkeyDel() {
    // delete elements depending of current supermode
    if (myEditModes.isCurrentSupermodeNetwork()) {
        if ((myEditModes.networkEditMode == NetworkEditMode::NETWORK_CONNECT) || (myEditModes.networkEditMode == NetworkEditMode::NETWORK_TLS)) {
            setStatusBarText(TL("Cannot delete in this mode"));
        } else if ((myEditModes.networkEditMode == NetworkEditMode::NETWORK_INSPECT) && (myInspectedAttributeCarriers.size() > 0)) {
            // delete inspected elements
            myUndoList->begin(GUIIcon::MODEDELETE, TL("delete network inspected elements"));
            deleteNetworkAttributeCarriers(myInspectedAttributeCarriers);
            myUndoList->end();
        } else {
            // get selected ACs
            const auto selectedACs = myNet->getAttributeCarriers()->getSelectedAttributeCarriers(false);
            // delete selected elements
            if (selectedACs.size() > 0) {
                myUndoList->begin(GUIIcon::MODEDELETE, TL("delete network selection"));
                deleteNetworkAttributeCarriers(selectedACs);
                myUndoList->end();
            }
        }
    } else if (myEditModes.isCurrentSupermodeDemand()) {
        if ((myEditModes.demandEditMode == DemandEditMode::DEMAND_INSPECT) && (myInspectedAttributeCarriers.size() > 0)) {
            // delete inspected elements
            myUndoList->begin(GUIIcon::MODEDELETE, TL("delete demand inspected elements"));
            deleteDemandAttributeCarriers(myInspectedAttributeCarriers);
            myUndoList->end();
        } else {
            // get selected ACs
            const auto selectedACs = myNet->getAttributeCarriers()->getSelectedAttributeCarriers(false);
            // delete selected elements
            if (selectedACs.size() > 0) {
                myUndoList->begin(GUIIcon::MODEDELETE, TL("delete demand selection"));
                deleteDemandAttributeCarriers(selectedACs);
                myUndoList->end();
            }
        }
    } else if (myEditModes.isCurrentSupermodeData()) {
        if ((myEditModes.demandEditMode == DemandEditMode::DEMAND_INSPECT) && (myInspectedAttributeCarriers.size() > 0)) {
            // delete inspected elements
            myUndoList->begin(GUIIcon::MODEDELETE, TL("delete data inspected elements"));
            deleteDataAttributeCarriers(myInspectedAttributeCarriers);
            myUndoList->end();
        } else {
            // get selected ACs
            const auto selectedACs = myNet->getAttributeCarriers()->getSelectedAttributeCarriers(false);
            // delete selected elements
            if (selectedACs.size() > 0) {
                myUndoList->begin(GUIIcon::MODEDELETE, TL("delete data selection"));
                deleteDataAttributeCarriers(selectedACs);
                myUndoList->end();
            }
        }
    }
    // update view
    updateViewNet();
}


void
GNEViewNet::hotkeyEnter() {
    // check what supermode is enabled
    if (myEditModes.isCurrentSupermodeNetwork()) {
        // abort operation depending of current mode
        if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_CONNECT) {
            // Accept changes in Connector Frame
            myViewParent->getConnectorFrame()->getConnectionModifications()->onCmdSaveModifications(0, 0, 0);
        } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_TLS) {
            // continue depending of current TLS frame state
            if (myViewParent->getTLSEditorFrame()->getTLSAttributes()->isSetDetectorsToggleButtonEnabled()) {
                myViewParent->getTLSEditorFrame()->getTLSAttributes()->disableE1DetectorMode();
            } else if (myViewParent->getTLSEditorFrame()->getTLSJunction()->isJoiningJunctions()) {
                myViewParent->getTLSEditorFrame()->getTLSJunction()->onCmdAcceptJoin(nullptr, 0, nullptr);
            } else if (myViewParent->getTLSEditorFrame()->getTLSDefinition()->checkHaveModifications()) {
                myViewParent->getTLSEditorFrame()->getTLSDefinition()->onCmdSaveChanges(nullptr, 0, nullptr);
            }
        } else if ((myEditModes.networkEditMode == NetworkEditMode::NETWORK_MOVE) && (myEditNetworkElementShapes.getEditedNetworkElement() != nullptr)) {
            myEditNetworkElementShapes.commitShapeEdited();
        } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_SHAPE) {
            if (myViewParent->getShapeFrame()->getDrawingShapeModule()->isDrawing()) {
                // stop current drawing
                myViewParent->getShapeFrame()->getDrawingShapeModule()->stopDrawing();
            } else {
                // start drawing
                myViewParent->getShapeFrame()->getDrawingShapeModule()->startDrawing();
            }
        } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_CROSSING) {
            myViewParent->getCrossingFrame()->createCrossingHotkey();
        } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_TAZ) {
            if (myViewParent->getTAZFrame()->getDrawingShapeModule()->isDrawing()) {
                // stop current drawing
                myViewParent->getTAZFrame()->getDrawingShapeModule()->stopDrawing();
            } else if (myViewParent->getTAZFrame()->getCurrentTAZModule()->getTAZ() == nullptr) {
                // start drawing
                myViewParent->getTAZFrame()->getDrawingShapeModule()->startDrawing();
            } else if (myViewParent->getTAZFrame()->getTAZSaveChangesModule()->isChangesPending()) {
                // save pending changes
                myViewParent->getTAZFrame()->getTAZSaveChangesModule()->onCmdSaveChanges(0, 0, 0);
            }
        } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_ADDITIONAL) {
            // create path element
            myViewParent->getAdditionalFrame()->createPath(false);
        } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_WIRE) {
            // create path element
            myViewParent->getWireFrame()->createPath(false);
        }
    } else if (myEditModes.isCurrentSupermodeDemand()) {
        if (myEditModes.demandEditMode == DemandEditMode::DEMAND_ROUTE) {
            myViewParent->getRouteFrame()->getPathCreator()->createPath(false);
        } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_VEHICLE) {
            myViewParent->getVehicleFrame()->getPathCreator()->onCmdCreatePath(nullptr, 0, nullptr);
        } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_PERSON) {
            myViewParent->getPersonFrame()->getPlanCreator()->onCmdCreatePath(nullptr, 0, nullptr);
        } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_PERSONPLAN) {
            myViewParent->getPersonPlanFrame()->getPlanCreator()->onCmdCreatePath(nullptr, 0, nullptr);
        } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_CONTAINER) {
            myViewParent->getContainerFrame()->getPlanCreator()->onCmdCreatePath(nullptr, 0, nullptr);
        } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_CONTAINERPLAN) {
            myViewParent->getContainerPlanFrame()->getPlanCreator()->onCmdCreatePath(nullptr, 0, nullptr);
        }
    } else if (myEditModes.isCurrentSupermodeData()) {
        if (myEditModes.dataEditMode == DataEditMode::DATA_EDGERELDATA) {
            myViewParent->getEdgeRelDataFrame()->getPathCreator()->createPath(false);
        } else if (myEditModes.dataEditMode == DataEditMode::DATA_TAZRELDATA) {
            myViewParent->getTAZRelDataFrame()->buildTAZRelationData();
        }
    }
}


void
GNEViewNet::hotkeyBackSpace() {
    // check what supermode is enabled
    if (myEditModes.isCurrentSupermodeNetwork()) {
        if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_ADDITIONAL) {
            myViewParent->getAdditionalFrame()->getConsecutiveLaneSelector()->removeLastElement();
        }
    } else if (myEditModes.isCurrentSupermodeDemand()) {
        if (myEditModes.demandEditMode == DemandEditMode::DEMAND_ROUTE) {
            myViewParent->getRouteFrame()->getPathCreator()->removeLastElement();
        } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_VEHICLE) {
            myViewParent->getVehicleFrame()->getPathCreator()->removeLastElement();
        } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_PERSON) {
            myViewParent->getPersonFrame()->getPlanCreator()->removeLastElement();
        } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_PERSONPLAN) {
            myViewParent->getPersonPlanFrame()->getPlanCreator()->removeLastElement();
        } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_CONTAINER) {
            myViewParent->getContainerFrame()->getPlanCreator()->removeLastElement();
        } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_CONTAINERPLAN) {
            myViewParent->getContainerPlanFrame()->getPlanCreator()->removeLastElement();
        }
    } else if (myEditModes.isCurrentSupermodeData()) {
        if (myEditModes.dataEditMode == DataEditMode::DATA_EDGERELDATA) {
            myViewParent->getEdgeRelDataFrame()->getPathCreator()->removeLastElement();
        }
    }
}

void
GNEViewNet::hotkeyFocusFrame() {
    // if there is a visible frame, set focus over it. In other case, set focus over ViewNet
    if (myCurrentFrame != nullptr) {
        myCurrentFrame->focusUpperElement();
    } else {
        setFocus();
    }
}


GNEViewParent*
GNEViewNet::getViewParent() const {
    return myViewParent;
}


GNENet*
GNEViewNet::getNet() const {
    return myNet;
}


GNEUndoList*
GNEViewNet::getUndoList() const {
    return myUndoList;
}


GNEViewNetHelper::IntervalBar&
GNEViewNet::getIntervalBar() {
    return myIntervalBar;
}


const std::vector<GNEAttributeCarrier*>&
GNEViewNet::getInspectedAttributeCarriers() const {
    return myInspectedAttributeCarriers;
}


GNEViewNetHelper::LockManager&
GNEViewNet::getLockManager() {
    return myLockManager;
}


void
GNEViewNet::setInspectedAttributeCarriers(const std::vector<GNEAttributeCarrier*> ACs) {
    myInspectedAttributeCarriers = ACs;
}


bool
GNEViewNet::isAttributeCarrierInspected(const GNEAttributeCarrier* AC) const {
    if (myInspectedAttributeCarriers.empty()) {
        return false;
    } else {
        // search AC in myInspectedAttributeCarriers
        const auto it = std::find(myInspectedAttributeCarriers.begin(), myInspectedAttributeCarriers.end(), AC);
        if (it == myInspectedAttributeCarriers.end()) {
            return false;
        } else {
            return true;
        }
    }
}


void
GNEViewNet::removeFromAttributeCarrierInspected(const GNEAttributeCarrier* AC) {
    // search AC in myInspectedAttributeCarriers
    const auto it = std::find(myInspectedAttributeCarriers.begin(), myInspectedAttributeCarriers.end(), AC);
    if (it != myInspectedAttributeCarriers.end()) {
        myInspectedAttributeCarriers.erase(it);
        myViewParent->getInspectorFrame()->inspectMultisection(myInspectedAttributeCarriers);
    }
}


const GNEAttributeCarrier*
GNEViewNet::getFrontAttributeCarrier() const {
    return myFrontAttributeCarrier;
}


const GUIGlObject*
GNEViewNet::getFrontGLObject() const {
    if (myFrontAttributeCarrier) {
        return myFrontAttributeCarrier->getGUIGlObject();
    } else {
        return nullptr;
    }
}


void
GNEViewNet::setFrontAttributeCarrier(GNEAttributeCarrier* AC) {
    myFrontAttributeCarrier = AC;
    update();
}


void
GNEViewNet::drawTranslateFrontAttributeCarrier(const GNEAttributeCarrier* AC, double typeOrLayer, const double extraOffset) {
    if (myFrontAttributeCarrier == AC) {
        glTranslated(0, 0, GLO_FRONTELEMENT + extraOffset);
    } else {
        glTranslated(0, 0, typeOrLayer + extraOffset);
    }
}


bool
GNEViewNet::isCurrentlyMovingElements() const {
    return myMoveSingleElement.isCurrentlyMovingSingleElement() || myMoveMultipleElements.isCurrentlyMovingMultipleElements();
}


bool
GNEViewNet::checkOverLockedElement(const GUIGlObject* GLObject, const bool isSelected) const {
    // check if elemet is blocked
    if (myLockManager.isObjectLocked(GLObject->getType(), isSelected)) {
        return false;
    }
    // get front GLObject
    const auto glObjectFront = myViewObjectsSelector.getGUIGlObjectFront();
    // check if element is under cursor
    if (glObjectFront) {
        if (glObjectFront == GLObject) {
            return true;
        } else if (glObjectFront->getType() == GLObject->getType()) {
            for (const auto& glObjectUnderCursor : myViewObjectsSelector.getGLObjects()) {
                if (glObjectUnderCursor == GLObject) {
                    return true;
                }
            }
        }
    }
    return false;
}


GNEDemandElement*
GNEViewNet::getLastCreatedRoute() const {
    return myLastCreatedRoute;
}


void
GNEViewNet::setLastCreatedRoute(GNEDemandElement* lastCreatedRoute) {
    myLastCreatedRoute = lastCreatedRoute;
}


GNEJunction*
GNEViewNet::getJunctionAtPopupPosition() {
    // get first object that can be found in their container
    for (const auto& glObjectLayer : gViewObjectsHandler.getSelectedObjects()) {
        for (const auto& glObject : glObjectLayer.second) {
            auto junction = myNet->getAttributeCarriers()->retrieveJunction(glObject.object->getMicrosimID(), false);
            if (junction) {
                return junction;
            }
        }
    }
    return nullptr;
}


GNEConnection*
GNEViewNet::getConnectionAtPopupPosition() {
    // get first object that can be found in their container
    for (const auto& glObjectLayer : gViewObjectsHandler.getSelectedObjects()) {
        for (const auto& glObject : glObjectLayer.second) {
            auto connection = myNet->getAttributeCarriers()->retrieveConnection(glObject.object, false);
            if (connection) {
                return connection;
            }
        }
    }
    return nullptr;
}


GNECrossing*
GNEViewNet::getCrossingAtPopupPosition() {
    // get first object that can be found in their container
    for (const auto& glObjectLayer : gViewObjectsHandler.getSelectedObjects()) {
        for (const auto& glObject : glObjectLayer.second) {
            auto crossing = myNet->getAttributeCarriers()->retrieveCrossing(glObject.object, false);
            if (crossing) {
                return crossing;
            }
        }
    }
    return nullptr;
}


GNEWalkingArea*
GNEViewNet::getWalkingAreaAtPopupPosition() {
    // get first object that can be found in their container
    for (const auto& glObjectLayer : gViewObjectsHandler.getSelectedObjects()) {
        for (const auto& glObject : glObjectLayer.second) {
            auto walkingArea = myNet->getAttributeCarriers()->retrieveWalkingArea(glObject.object, false);
            if (walkingArea) {
                return walkingArea;
            }
        }
    }
    return nullptr;
}


GNEEdge*
GNEViewNet::getEdgeAtPopupPosition() {
    // get first object that can be found in their container
    for (const auto& glObjectLayer : gViewObjectsHandler.getSelectedObjects()) {
        for (const auto& glObject : glObjectLayer.second) {
            auto edge = myNet->getAttributeCarriers()->retrieveEdge(glObject.object->getMicrosimID(), false);
            if (edge) {
                return edge;
            }
        }
    }
    return nullptr;
}


GNELane*
GNEViewNet::getLaneAtPopupPosition() {
    // get first object that can be found in their container
    for (const auto& glObjectLayer : gViewObjectsHandler.getSelectedObjects()) {
        for (const auto& glObject : glObjectLayer.second) {
            auto lane = myNet->getAttributeCarriers()->retrieveLane(glObject.object, false);
            if (lane) {
                return lane;
            }
        }
    }
    return nullptr;
}


GNEAdditional*
GNEViewNet::getAdditionalAtPopupPosition() {
    // get first object that can be found in their container
    for (const auto& glObjectLayer : gViewObjectsHandler.getSelectedObjects()) {
        for (const auto& glObject : glObjectLayer.second) {
            auto additionalElement = myNet->getAttributeCarriers()->retrieveAdditional(glObject.object, false);
            if (additionalElement) {
                return additionalElement;
            }
        }
    }
    return nullptr;
}


GNEDemandElement*
GNEViewNet::getDemandElementAtPopupPosition() {
    // get first object that can be found in their container
    for (const auto& glObjectLayer : gViewObjectsHandler.getSelectedObjects()) {
        for (const auto& glObject : glObjectLayer.second) {
            auto demandElement = myNet->getAttributeCarriers()->retrieveDemandElement(glObject.object, false);
            if (demandElement) {
                return demandElement;
            }
        }
    }
    return nullptr;
}


GNEPoly*
GNEViewNet::getPolygonAtPopupPosition() {
    // get first object that can be parsed to poly element
    for (const auto& glObjectLayer : gViewObjectsHandler.getSelectedObjects()) {
        for (const auto& glObject : glObjectLayer.second) {
            auto polygon = dynamic_cast<GNEPoly*>(myNet->getAttributeCarriers()->retrieveAdditional(glObject.object, false));
            if (polygon) {
                return polygon;
            }
        }
    }
    return nullptr;
}


GNEPOI*
GNEViewNet::getPOIAtPopupPosition() {
    // get first object that can be parsed to POI element
    for (const auto& glObjectLayer : gViewObjectsHandler.getSelectedObjects()) {
        for (const auto& glObject : glObjectLayer.second) {
            auto POI = dynamic_cast<GNEPOI*>(myNet->getAttributeCarriers()->retrieveAdditional(glObject.object, false));
            if (POI) {
                return POI;
            }
        }
    }
    return nullptr;
}


GNETAZ*
GNEViewNet::getTAZAtPopupPosition() {
    // get first object that can be parsed to TAZ element
    for (const auto& glObjectLayer : gViewObjectsHandler.getSelectedObjects()) {
        for (const auto& glObject : glObjectLayer.second) {
            auto TAZ = dynamic_cast<GNETAZ*>(myNet->getAttributeCarriers()->retrieveAdditional(glObject.object, false));
            if (TAZ) {
                return TAZ;
            }
        }
    }
    return nullptr;
}


GNENetworkElement*
GNEViewNet::getShapeEditedAtPopupPosition() {
    // get first object that can be parsed to TAZ element
    for (const auto& glObjectLayer : gViewObjectsHandler.getSelectedObjects()) {
        for (const auto& glObject : glObjectLayer.second) {
            if (glObject.object->getType() == GLO_JUNCTION) {
                auto junction = myNet->getAttributeCarriers()->retrieveJunction(glObject.object->getMicrosimID());
                if (junction->isShapeEdited()) {
                    return junction;
                }
            } else if (glObject.object->getType() == GLO_CROSSING) {
                auto crossing = myNet->getAttributeCarriers()->retrieveCrossing(glObject.object);
                if (crossing->isShapeEdited()) {
                    return crossing;
                }
            } else if (glObject.object->getType() == GLO_CONNECTION) {
                auto connection = myNet->getAttributeCarriers()->retrieveConnection(glObject.object);
                if (connection->isShapeEdited()) {
                    return connection;
                }
            }
        }
    }
    return nullptr;
}


long
GNEViewNet::onCmdSetSupermode(FXObject*, FXSelector sel, void*) {
    // check what network mode will be set
    switch (FXSELID(sel)) {
        case MID_HOTKEY_F2_SUPERMODE_NETWORK:
            if (myEditModes.networkButton->shown()) {
                myEditModes.setSupermode(Supermode::NETWORK, false);
            }
            break;
        case MID_HOTKEY_F3_SUPERMODE_DEMAND:
            if (myEditModes.demandButton->shown()) {
                myEditModes.setSupermode(Supermode::DEMAND, false);
            }
            break;
        case MID_HOTKEY_F4_SUPERMODE_DATA:
            if (myEditModes.dataButton->shown()) {
                myEditModes.setSupermode(Supermode::DATA, false);
            }
            break;
        default:
            break;
    }
    return 1;
}


long
GNEViewNet::onCmdSetMode(FXObject*, FXSelector sel, void*) {
    // first filter modes depending of view
    if (myEditModes.isJuPedSimView()) {
        // network
        if (myEditModes.isCurrentSupermodeNetwork()) {
            switch (FXSELID(sel)) {
                // common
                case MID_HOTKEY_I_MODE_INSPECT:
                case MID_HOTKEY_D_MODE_SINGLESIMULATIONSTEP_DELETE:
                case MID_HOTKEY_S_MODE_STOPSIMULATION_SELECT:
                // infrastructure
                case MID_HOTKEY_M_MODE_MOVE_MEANDATA:
                case MID_HOTKEY_E_MODE_EDGE_EDGEDATA:
                case MID_HOTKEY_A_MODE_STARTSIMULATION_ADDITIONALS_STOPS:
                case MID_HOTKEY_R_MODE_CROSSING_ROUTE_EDGERELDATA:
                // shapes
                case MID_HOTKEY_P_MODE_POLYGON_PERSON:
                case MID_HOTKEY_Z_MODE_TAZ_TAZREL:
                    break;
                default:
                    return 0;
            }
        }
        // demand
        if (myEditModes.isCurrentSupermodeDemand()) {
            switch (FXSELID(sel)) {
                // common
                case MID_HOTKEY_I_MODE_INSPECT:
                case MID_HOTKEY_D_MODE_SINGLESIMULATIONSTEP_DELETE:
                case MID_HOTKEY_S_MODE_STOPSIMULATION_SELECT:
                // persons
                case MID_HOTKEY_P_MODE_POLYGON_PERSON:
                case MID_HOTKEY_L_MODE_PERSONPLAN:
                // routes
                case MID_HOTKEY_R_MODE_CROSSING_ROUTE_EDGERELDATA:
                case MID_HOTKEY_W_MODE_WIRE_ROUTEDISTRIBUTION:
                // types
                case MID_HOTKEY_T_MODE_TLS_TYPE:
                case MID_HOTKEY_U_MODE_DECAL_TYPEDISTRIBUTION:
                    break;
                default:
                    return 0;
            }
        }
        // data
        if (myEditModes.isCurrentSupermodeData()) {
            // all modes disabled
            return 0;
        }
    }
    // continue depending of supermode
    if (myEditModes.isCurrentSupermodeNetwork()) {
        // check what network mode will be set
        switch (FXSELID(sel)) {
            case MID_HOTKEY_I_MODE_INSPECT:
                myEditModes.setNetworkEditMode(NetworkEditMode::NETWORK_INSPECT);
                break;
            case MID_HOTKEY_D_MODE_SINGLESIMULATIONSTEP_DELETE:
                myEditModes.setNetworkEditMode(NetworkEditMode::NETWORK_DELETE);
                break;
            case MID_HOTKEY_S_MODE_STOPSIMULATION_SELECT:
                myEditModes.setNetworkEditMode(NetworkEditMode::NETWORK_SELECT);
                break;
            case MID_HOTKEY_M_MODE_MOVE_MEANDATA:
                myEditModes.setNetworkEditMode(NetworkEditMode::NETWORK_MOVE);
                break;
            case MID_HOTKEY_E_MODE_EDGE_EDGEDATA:
                myEditModes.setNetworkEditMode(NetworkEditMode::NETWORK_CREATE_EDGE);
                break;
            case MID_HOTKEY_C_MODE_CONNECT_CONTAINER:
                myEditModes.setNetworkEditMode(NetworkEditMode::NETWORK_CONNECT);
                break;
            case MID_HOTKEY_T_MODE_TLS_TYPE:
                myEditModes.setNetworkEditMode(NetworkEditMode::NETWORK_TLS);
                break;
            case MID_HOTKEY_A_MODE_STARTSIMULATION_ADDITIONALS_STOPS:
                myEditModes.setNetworkEditMode(NetworkEditMode::NETWORK_ADDITIONAL);
                break;
            case MID_HOTKEY_R_MODE_CROSSING_ROUTE_EDGERELDATA:
                myEditModes.setNetworkEditMode(NetworkEditMode::NETWORK_CROSSING);
                break;
            case MID_HOTKEY_Z_MODE_TAZ_TAZREL:
                myEditModes.setNetworkEditMode(NetworkEditMode::NETWORK_TAZ);
                break;
            case MID_HOTKEY_P_MODE_POLYGON_PERSON:
                myEditModes.setNetworkEditMode(NetworkEditMode::NETWORK_SHAPE);
                break;
            case MID_HOTKEY_H_MODE_PROHIBITION_CONTAINERPLAN:
                myEditModes.setNetworkEditMode(NetworkEditMode::NETWORK_PROHIBITION);
                break;
            case MID_HOTKEY_W_MODE_WIRE_ROUTEDISTRIBUTION:
                myEditModes.setNetworkEditMode(NetworkEditMode::NETWORK_WIRE);
                break;
            case MID_HOTKEY_U_MODE_DECAL_TYPEDISTRIBUTION:
                myEditModes.setNetworkEditMode(NetworkEditMode::NETWORK_DECAL);
                break;
            default:
                break;
        }
    } else if (myEditModes.isCurrentSupermodeDemand()) {
        // check what demand mode will be set
        switch (FXSELID(sel)) {
            case MID_HOTKEY_L_MODE_PERSONPLAN:
                myEditModes.setDemandEditMode(DemandEditMode::DEMAND_PERSONPLAN);
                break;
            case MID_HOTKEY_H_MODE_PROHIBITION_CONTAINERPLAN:
                myEditModes.setDemandEditMode(DemandEditMode::DEMAND_CONTAINERPLAN);
                break;
            case MID_HOTKEY_I_MODE_INSPECT:
                myEditModes.setDemandEditMode(DemandEditMode::DEMAND_INSPECT);
                break;
            case MID_HOTKEY_D_MODE_SINGLESIMULATIONSTEP_DELETE:
                myEditModes.setDemandEditMode(DemandEditMode::DEMAND_DELETE);
                break;
            case MID_HOTKEY_S_MODE_STOPSIMULATION_SELECT:
                myEditModes.setDemandEditMode(DemandEditMode::DEMAND_SELECT);
                break;
            case MID_HOTKEY_M_MODE_MOVE_MEANDATA:
                myEditModes.setDemandEditMode(DemandEditMode::DEMAND_MOVE);
                break;
            case MID_HOTKEY_R_MODE_CROSSING_ROUTE_EDGERELDATA:
                myEditModes.setDemandEditMode(DemandEditMode::DEMAND_ROUTE);
                break;
            case MID_HOTKEY_W_MODE_WIRE_ROUTEDISTRIBUTION:
                myEditModes.setDemandEditMode(DemandEditMode::DEMAND_ROUTEDISTRIBUTION);
                break;
            case MID_HOTKEY_V_MODE_VEHICLE:
                myEditModes.setDemandEditMode(DemandEditMode::DEMAND_VEHICLE);
                break;
            case MID_HOTKEY_T_MODE_TLS_TYPE:
                myEditModes.setDemandEditMode(DemandEditMode::DEMAND_TYPE);
                break;
            case MID_HOTKEY_U_MODE_DECAL_TYPEDISTRIBUTION:
                myEditModes.setDemandEditMode(DemandEditMode::DEMAND_TYPEDISTRIBUTION);
                break;
            case MID_HOTKEY_A_MODE_STARTSIMULATION_ADDITIONALS_STOPS:
                myEditModes.setDemandEditMode(DemandEditMode::DEMAND_STOP);
                break;
            case MID_HOTKEY_P_MODE_POLYGON_PERSON:
                myEditModes.setDemandEditMode(DemandEditMode::DEMAND_PERSON);
                break;
            case MID_HOTKEY_C_MODE_CONNECT_CONTAINER:
                myEditModes.setDemandEditMode(DemandEditMode::DEMAND_CONTAINER);
                break;
            default:
                break;
        }
    } else if (myEditModes.isCurrentSupermodeData()) {
        // check what demand mode will be set
        switch (FXSELID(sel)) {
            case MID_HOTKEY_I_MODE_INSPECT:
                myEditModes.setDataEditMode(DataEditMode::DATA_INSPECT);
                break;
            case MID_HOTKEY_D_MODE_SINGLESIMULATIONSTEP_DELETE:
                myEditModes.setDataEditMode(DataEditMode::DATA_DELETE);
                break;
            case MID_HOTKEY_S_MODE_STOPSIMULATION_SELECT:
                myEditModes.setDataEditMode(DataEditMode::DATA_SELECT);
                break;
            case MID_HOTKEY_E_MODE_EDGE_EDGEDATA:
                myEditModes.setDataEditMode(DataEditMode::DATA_EDGEDATA);
                break;
            case MID_HOTKEY_R_MODE_CROSSING_ROUTE_EDGERELDATA:
                myEditModes.setDataEditMode(DataEditMode::DATA_EDGERELDATA);
                break;
            case MID_HOTKEY_Z_MODE_TAZ_TAZREL:
                myEditModes.setDataEditMode(DataEditMode::DATA_TAZRELDATA);
                break;
            case MID_HOTKEY_M_MODE_MOVE_MEANDATA:
                myEditModes.setDataEditMode(DataEditMode::DATA_MEANDATA);
                break;
            default:
                break;
        }
    }
    return 1;
}


long
GNEViewNet::onCmdSplitEdge(FXObject*, FXSelector, void*) {
    GNEEdge* edge = getEdgeAtPopupPosition();
    if (edge != nullptr) {
        myNet->splitEdge(edge, edge->getSplitPos(getPopupPosition()), myUndoList);
    }
    return 1;
}


long
GNEViewNet::onCmdSplitEdgeBidi(FXObject*, FXSelector, void*) {
    GNEEdge* edge = getEdgeAtPopupPosition();
    if (edge != nullptr) {
        // obtain reverse edge
        const auto oppositeEdges = edge->getOppositeEdges();
        // check that reverse edge works
        if (oppositeEdges.size() > 0) {
            for (const auto& oppositeEdge : oppositeEdges) {
                // get reverse inner geometry
                const auto reverseGeometry = oppositeEdge->getNBEdge()->getInnerGeometry().reverse();
                if (reverseGeometry == edge->getNBEdge()->getInnerGeometry()) {
                    myNet->splitEdgesBidi(edge, oppositeEdge, edge->getSplitPos(getPopupPosition()), myUndoList);
                    return 1;
                }
            }
        }
    }
    return 1;
}


long
GNEViewNet::onCmdReverseEdge(FXObject*, FXSelector, void*) {
    GNEEdge* edge = getEdgeAtPopupPosition();
    if (edge != nullptr) {
        if (edge->isAttributeCarrierSelected()) {
            myUndoList->begin(edge, TL("Reverse selected edges"));
            const auto selectedEdges = myNet->getAttributeCarriers()->getSelectedEdges();
            for (const auto& selectedEdge : selectedEdges) {
                myNet->reverseEdge(selectedEdge, myUndoList);
            }
            myUndoList->end();
        } else {
            myUndoList->begin(edge, TL("Reverse edge"));
            myNet->reverseEdge(edge, myUndoList);
            myUndoList->end();
        }
    }
    return 1;
}


long
GNEViewNet::onCmdAddReversedEdge(FXObject*, FXSelector, void*) {
    GNEEdge* edge = getEdgeAtPopupPosition();
    if (edge != nullptr) {
        if (edge->isAttributeCarrierSelected()) {
            myUndoList->begin(edge, TL("Add Reverse edge for selected edges"));
            const auto selectedEdges = myNet->getAttributeCarriers()->getSelectedEdges();
            for (const auto& selectedEdge : selectedEdges) {
                myNet->addReversedEdge(selectedEdge, false, myUndoList);
            }
            myUndoList->end();
        } else {
            myUndoList->begin(edge, TL("Add reverse edge"));
            myNet->addReversedEdge(edge, false, myUndoList);
            myUndoList->end();
        }
    }
    return 1;
}


long
GNEViewNet::onCmdAddReversedEdgeDisconnected(FXObject*, FXSelector, void*) {
    GNEEdge* edge = getEdgeAtPopupPosition();
    if (edge != nullptr) {
        if (edge->isAttributeCarrierSelected()) {
            myUndoList->begin(edge, TL("Add Reverse disconnected edge for selected edges"));
            const auto selectedEdges = myNet->getAttributeCarriers()->getSelectedEdges();
            for (const auto& selectedEdge : selectedEdges) {
                myNet->addReversedEdge(selectedEdge, true, myUndoList);
            }
            myUndoList->end();
        } else {
            myUndoList->begin(edge, TL("Add reverse disconnected edge"));
            myNet->addReversedEdge(edge, true, myUndoList);
            myUndoList->end();
        }
    }
    return 1;
}


long
GNEViewNet::onCmdEditEdgeEndpoint(FXObject*, FXSelector, void*) {
    GNEEdge* edge = getEdgeAtPopupPosition();
    if (edge != nullptr) {
        // snap to active grid the Popup position
        edge->editEndpoint(getPopupPosition(), myUndoList);
    }
    return 1;
}


long
GNEViewNet::onCmdResetEdgeEndpoint(FXObject*, FXSelector, void*) {
    GNEEdge* edge = getEdgeAtPopupPosition();
    if (edge != nullptr) {
        // check if edge is selected
        if (edge->isAttributeCarrierSelected()) {
            // get all selected edges
            const auto selectedEdges = myNet->getAttributeCarriers()->getSelectedEdges();
            // begin operation
            myUndoList->begin(edge, TL("reset geometry points"));
            // iterate over selected edges
            for (const auto& selectedEdge : selectedEdges) {
                // reset both end points
                selectedEdge->resetBothEndpoint(myUndoList);
            }
            // end operation
            myUndoList->end();
        } else {
            edge->resetEndpoint(getPopupPosition(), myUndoList);
        }
    }
    return 1;
}


long
GNEViewNet::onCmdStraightenEdges(FXObject*, FXSelector, void*) {
    GNEEdge* edge = getEdgeAtPopupPosition();
    if (edge != nullptr) {
        if (edge->isAttributeCarrierSelected()) {
            myUndoList->begin(edge, TL("straighten selected edges"));
            const auto selectedEdges = myNet->getAttributeCarriers()->getSelectedEdges();
            for (const auto& selectedEdge : selectedEdges) {
                selectedEdge->setAttribute(SUMO_ATTR_SHAPE, "", myUndoList);
            }
            myUndoList->end();
        } else {

            myUndoList->begin(edge, TL("straighten edge"));
            edge->setAttribute(SUMO_ATTR_SHAPE, "", myUndoList);
            myUndoList->end();
        }
    }
    return 1;
}


long
GNEViewNet::onCmdSmoothEdges(FXObject*, FXSelector, void*) {
    GNEEdge* edge = getEdgeAtPopupPosition();
    if (edge != nullptr) {
        if (edge->isAttributeCarrierSelected()) {
            myUndoList->begin(edge, TL("smooth selected edges"));
            const auto selectedEdges = myNet->getAttributeCarriers()->getSelectedEdges();
            for (const auto& selectedEdge : selectedEdges) {
                selectedEdge->smooth(myUndoList);
            }
            myUndoList->end();
        } else {
            myUndoList->begin(edge, TL("smooth edge"));
            edge->smooth(myUndoList);
            myUndoList->end();
        }
    }
    return 1;
}


long
GNEViewNet::onCmdStraightenEdgesElevation(FXObject*, FXSelector, void*) {
    GNEEdge* edge = getEdgeAtPopupPosition();
    if (edge != nullptr) {
        if (edge->isAttributeCarrierSelected()) {
            myUndoList->begin(edge, TL("straighten elevation of selected edges"));
            const auto selectedEdges = myNet->getAttributeCarriers()->getSelectedEdges();
            for (const auto& selectedEdge : selectedEdges) {
                selectedEdge->straightenElevation(myUndoList);
            }
            myUndoList->end();
        } else {
            myUndoList->begin(edge, TL("straighten edge elevation"));
            edge->straightenElevation(myUndoList);
            myUndoList->end();
        }
    }
    return 1;
}


long
GNEViewNet::onCmdSmoothEdgesElevation(FXObject*, FXSelector, void*) {
    GNEEdge* edge = getEdgeAtPopupPosition();
    if (edge != nullptr) {
        if (edge->isAttributeCarrierSelected()) {
            myUndoList->begin(edge, TL("smooth elevation of selected edges"));
            const auto selectedEdges = myNet->getAttributeCarriers()->getSelectedEdges();
            for (const auto& selectedEdge : selectedEdges) {
                selectedEdge->smoothElevation(myUndoList);
            }
            myUndoList->end();
        } else {
            myUndoList->begin(edge, TL("smooth edge elevation"));
            edge->smoothElevation(myUndoList);
            myUndoList->end();
        }
    }
    return 1;
}


long
GNEViewNet::onCmdResetLength(FXObject*, FXSelector, void*) {
    GNEEdge* edge = getEdgeAtPopupPosition();
    if (edge != nullptr) {
        if (edge->isAttributeCarrierSelected()) {
            myUndoList->begin(edge, TL("reset edge lengths"));
            const auto selectedEdges = myNet->getAttributeCarriers()->getSelectedEdges();
            for (const auto& selectedEdge : selectedEdges) {
                selectedEdge->setAttribute(SUMO_ATTR_LENGTH, "-1", myUndoList);
            }
            myUndoList->end();
        } else {
            edge->setAttribute(SUMO_ATTR_LENGTH, "-1", myUndoList);
        }
    }
    return 1;
}


long
GNEViewNet::onCmdEdgeUseAsTemplate(FXObject*, FXSelector, void*) {
    GNEEdge* edge = getEdgeAtPopupPosition();
    if (edge != nullptr) {
        myViewParent->getInspectorFrame()->getTemplateEditor()->setEdgeTemplate(edge);
    }
    return 1;
}


long
GNEViewNet::onCmdEgeApplyTemplate(FXObject*, FXSelector, void*) {
    GNEEdge* edgeAtPosition = getEdgeAtPopupPosition();
    if ((edgeAtPosition != nullptr) && myViewParent->getInspectorFrame()->getTemplateEditor()->getEdgeTemplate()) {
        // begin copy template
        myUndoList->begin(edgeAtPosition, TL("copy edge template"));
        if (edgeAtPosition->isAttributeCarrierSelected()) {
            // copy template in all selected edges
            for (const auto& edge : myNet->getAttributeCarriers()->getEdges()) {
                if (edge.second->isAttributeCarrierSelected()) {
                    edge.second->copyTemplate(myViewParent->getInspectorFrame()->getTemplateEditor()->getEdgeTemplate(), myUndoList);
                }
            }
        } else {
            // copy template
            edgeAtPosition->copyTemplate(myViewParent->getInspectorFrame()->getTemplateEditor()->getEdgeTemplate(), myUndoList);
        }
        // end copy template
        myUndoList->end();
        // update view (to see visual changes)
        updateViewNet();
    }
    return 1;
}


long
GNEViewNet::onCmdSimplifyShape(FXObject*, FXSelector, void*) {
    // get polygon under mouse
    GNEPoly* polygonUnderMouse = getPolygonAtPopupPosition();
    // check polygon
    if (polygonUnderMouse) {
        // check if shape is selected
        if (polygonUnderMouse->isAttributeCarrierSelected()) {
            // begin undo-list
            myNet->getViewNet()->getUndoList()->begin(polygonUnderMouse, TL("simplify shapes"));
            // get shapes
            const auto selectedShapes = myNet->getAttributeCarriers()->getSelectedShapes();
            // iterate over shapes
            for (const auto& selectedShape : selectedShapes) {
                // check if shape is a poly
                if ((selectedShape->getTagProperty().getTag() == SUMO_TAG_POLY) ||
                        (selectedShape->getTagProperty().getTag() == GNE_TAG_JPS_WALKABLEAREA) ||
                        (selectedShape->getTagProperty().getTag() == GNE_TAG_JPS_OBSTACLE)) {
                    // simplify shape
                    dynamic_cast<GNEPoly*>(selectedShape)->simplifyShape();
                }
            }
            // end undo-list
            myNet->getViewNet()->getUndoList()->end();
        } else {
            polygonUnderMouse->simplifyShape();
        }
    }
    updateViewNet();
    return 1;
}


long
GNEViewNet::onCmdDeleteGeometryPoint(FXObject*, FXSelector, void*) {
    GNEPoly* polygonUnderMouse = getPolygonAtPopupPosition();
    if (polygonUnderMouse) {
        polygonUnderMouse->deleteGeometryPoint(getPopupPosition());
    }
    updateViewNet();
    return 1;
}


long
GNEViewNet::onCmdClosePolygon(FXObject*, FXSelector, void*) {
    // get polygon under mouse
    GNEPoly* polygonUnderMouse = getPolygonAtPopupPosition();
    // check polygon
    if (polygonUnderMouse) {
        // check if shape is selected
        if (polygonUnderMouse->isAttributeCarrierSelected()) {
            // begin undo-list
            myNet->getViewNet()->getUndoList()->begin(polygonUnderMouse, TL("close polygon shapes"));
            // get selectedshapes
            const auto selectedShapes = myNet->getAttributeCarriers()->getSelectedShapes();
            // iterate over shapes
            for (const auto& selectedShape : selectedShapes) {
                // check if shape is a poly
                if ((selectedShape->getTagProperty().getTag() == SUMO_TAG_POLY) ||
                        (selectedShape->getTagProperty().getTag() == GNE_TAG_JPS_WALKABLEAREA) ||
                        (selectedShape->getTagProperty().getTag() == GNE_TAG_JPS_OBSTACLE)) {
                    // close polygon
                    dynamic_cast<GNEPoly*>(selectedShape)->closePolygon();
                }
            }
            // end undo-list
            myNet->getViewNet()->getUndoList()->end();
        } else {
            polygonUnderMouse->closePolygon();
        }
    }
    updateViewNet();
    return 1;
}


long
GNEViewNet::onCmdOpenPolygon(FXObject*, FXSelector, void*) {
    // get polygon under mouse
    GNEPoly* polygonUnderMouse = getPolygonAtPopupPosition();
    // check polygon
    if (polygonUnderMouse) {
        // check if shape is selected
        if (polygonUnderMouse->isAttributeCarrierSelected()) {
            // begin undo-list
            myNet->getViewNet()->getUndoList()->begin(polygonUnderMouse, TL("open polygon shapes"));
            // get shapes
            const auto selectedShapes = myNet->getAttributeCarriers()->getSelectedShapes();
            // iterate over shapes
            for (const auto& selectedShape : selectedShapes) {
                // check if shape is a poly
                if ((selectedShape->getTagProperty().getTag() == SUMO_TAG_POLY) ||
                        (selectedShape->getTagProperty().getTag() == GNE_TAG_JPS_WALKABLEAREA) ||
                        (selectedShape->getTagProperty().getTag() == GNE_TAG_JPS_OBSTACLE)) {
                    // open polygon
                    dynamic_cast<GNEPoly*>(selectedShape)->openPolygon();
                }
            }
            // end undo-list
            myNet->getViewNet()->getUndoList()->end();
        } else {
            polygonUnderMouse->openPolygon();
        }
    }
    updateViewNet();
    return 1;
}


long
GNEViewNet::onCmdSelectPolygonElements(FXObject*, FXSelector, void*) {
    // get polygon under mouse
    GNEPoly* polygonUnderMouse = getPolygonAtPopupPosition();
    // check polygon
    if (polygonUnderMouse) {
        // get ACs in boundary
        updateObjectsInBoundary(polygonUnderMouse->getShape().getBoxBoundary());
        // declare filtered ACs
        std::vector<GNEAttributeCarrier*> filteredACs;
        // iterate over obtained GUIGlIDs
        for (const auto& AC : myViewObjectsSelector.getAttributeCarriers()) {
            if (AC->getTagProperty().getTag() == SUMO_TAG_EDGE) {
                if (checkSelectEdges() && myNet->getAttributeCarriers()->isNetworkElementAroundShape(AC, polygonUnderMouse->getShape())) {
                    filteredACs.push_back(AC);
                }
            } else if (AC->getTagProperty().getTag() == SUMO_TAG_LANE) {
                if (!checkSelectEdges() && myNet->getAttributeCarriers()->isNetworkElementAroundShape(AC, polygonUnderMouse->getShape())) {
                    filteredACs.push_back(AC);
                }
            } else if ((AC != polygonUnderMouse) && myNet->getAttributeCarriers()->isNetworkElementAroundShape(AC, polygonUnderMouse->getShape())) {
                filteredACs.push_back(AC);
            }
        }
        // continue if there are ACs
        if (filteredACs.size() > 0) {
            // begin undo-list
            myNet->getViewNet()->getUndoList()->begin(GUIIcon::MODESELECT, TL("select within polygon boundary"));
            // iterate over shapes
            for (const auto& AC : filteredACs) {
                AC->setAttribute(GNE_ATTR_SELECTED, "true", myUndoList);
            }
            // end undo-list
            myNet->getViewNet()->getUndoList()->end();
        }
    }
    updateViewNet();
    return 1;
}


long
GNEViewNet::onCmdSetFirstGeometryPoint(FXObject*, FXSelector, void*) {
    GNEPoly* polygonUnderMouse = getPolygonAtPopupPosition();
    if (polygonUnderMouse) {
        polygonUnderMouse->changeFirstGeometryPoint(polygonUnderMouse->getVertexIndex(getPopupPosition(), false));
        updateViewNet();
    }

    return 1;
}


long
GNEViewNet::onCmdSimplifyShapeEdited(FXObject*, FXSelector, void*) {
    auto undoList = myNet->getViewNet()->getUndoList();
    // get shape edited under mouse
    GNENetworkElement* shapeEdited = getShapeEditedAtPopupPosition();
    if (shapeEdited) {
        // simplify edited shape using undo-redo
        undoList->begin(shapeEdited, TL("simplify edited shape"));
        shapeEdited->simplifyShapeEdited(undoList);
        undoList->end();
    }
    return 1;
}


long
GNEViewNet::onCmdStraightenShapeEdited(FXObject*, FXSelector, void*) {
    auto undoList = myNet->getViewNet()->getUndoList();
    // get shape edited under mouse
    GNENetworkElement* shapeEdited = getShapeEditedAtPopupPosition();
    if (shapeEdited) {
        // simplify edited shape using undo-redo
        undoList->begin(shapeEdited, TL("straigthen edited shape"));
        shapeEdited->straigthenShapeEdited(undoList);
        undoList->end();
    }
    return 1;
}


long
GNEViewNet::onCmdCloseShapeEdited(FXObject*, FXSelector, void*) {
    auto undoList = myNet->getViewNet()->getUndoList();
    // get shape edited under mouse
    GNENetworkElement* shapeEdited = getShapeEditedAtPopupPosition();
    if (shapeEdited) {
        // close edited shape using undo-redo
        undoList->begin(shapeEdited, TL("simplify edited shape"));
        shapeEdited->closeShapeEdited(undoList);
        undoList->end();
    }
    return 1;
}


long
GNEViewNet::onCmdOpenShapeEdited(FXObject*, FXSelector, void*) {
    auto undoList = myNet->getViewNet()->getUndoList();
    // get shape edited under mouse
    GNENetworkElement* shapeEdited = getShapeEditedAtPopupPosition();
    if (shapeEdited) {
        // open edited shape using undo-redo
        undoList->begin(shapeEdited, TL("simplify edited shape"));
        shapeEdited->openShapeEdited(undoList);
        undoList->end();
    }
    return 1;
}


long
GNEViewNet::onCmdSetFirstGeometryPointShapeEdited(FXObject*, FXSelector, void*) {
    auto undoList = myNet->getViewNet()->getUndoList();
    // get shape edited under mouse
    GNENetworkElement* shapeEdited = getShapeEditedAtPopupPosition();
    if (shapeEdited) {
        // get geometry point index under cursor
        const auto geometryPointIndex = shapeEdited->getGeometryPointUnderCursorShapeEdited();
        // set first geometry point in edited shape using undo-redo
        undoList->begin(shapeEdited, TL("simplify edited shape"));
        shapeEdited->setFirstGeometryPointShapeEdited(geometryPointIndex, undoList);
        undoList->end();
    }
    return 1;
}


long
GNEViewNet::onCmdDeleteGeometryPointShapeEdited(FXObject*, FXSelector, void*) {
    auto undoList = myNet->getViewNet()->getUndoList();
    // get shape edited under mouse
    GNENetworkElement* shapeEdited = getShapeEditedAtPopupPosition();
    if (shapeEdited) {
        // get geometry point index under cursor
        const auto geometryPointIndex = shapeEdited->getGeometryPointUnderCursorShapeEdited();
        // delete geometry point edited shape using undo-redo
        undoList->begin(shapeEdited, TL("simplify edited shape"));
        shapeEdited->deleteGeometryPointShapeEdited(geometryPointIndex, undoList);
        undoList->end();
    }
    return 1;
}


long
GNEViewNet::onCmdResetShapeEdited(FXObject*, FXSelector, void*) {
    auto undoList = myNet->getViewNet()->getUndoList();
    // get shape edited under mouse
    GNENetworkElement* shapeEdited = getShapeEditedAtPopupPosition();
    if (shapeEdited) {
        // simplify edited shape using undo-redo
        undoList->begin(shapeEdited, TL("simplify edited shape"));
        shapeEdited->resetShapeEdited(undoList);
        undoList->end();
    }
    return 1;
}


long
GNEViewNet::onCmdFinishShapeEdited(FXObject*, FXSelector, void*) {
    // the same behavior as when we press enter
    hotkeyEnter();
    return 1;
}


long
GNEViewNet::onCmdTransformPOI(FXObject*, FXSelector, void*) {
    // declare additional handler
    GNEAdditionalHandler additionalHanlder(myNet, true, false);
    // obtain POI at popup position
    GNEPOI* POI = getPOIAtPopupPosition();
    if (POI) {
        // check what type of POI will be transformed
        if (POI->getTagProperty().getTag() == SUMO_TAG_POI) {
            // obtain lanes around POI boundary
            std::vector<GUIGlID> GLIDs = getObjectsInBoundary(POI->getCenteringBoundary());
            std::vector<GNELane*> lanes;
            for (const auto& GLID : GLIDs) {
                GNELane* lane = dynamic_cast<GNELane*>(GUIGlObjectStorage::gIDStorage.getObjectBlocking(GLID));
                if (lane) {
                    lanes.push_back(lane);
                }
            }
            if (lanes.empty()) {
                WRITE_WARNINGF(TL("No lanes around % to attach it"), toString(SUMO_TAG_POI));
            } else {
                // obtain nearest lane to POI
                GNELane* nearestLane = lanes.front();
                double minorPosOverLane = nearestLane->getLaneShape().nearest_offset_to_point2D(POI->getPositionInView());
                double minorLateralOffset = nearestLane->getLaneShape().positionAtOffset(minorPosOverLane).distanceTo(POI->getPositionInView());
                for (const auto& lane : lanes) {
                    double posOverLane = lane->getLaneShape().nearest_offset_to_point2D(POI->getPositionInView());
                    double lateralOffset = lane->getLaneShape().positionAtOffset(posOverLane).distanceTo(POI->getPositionInView());
                    if (lateralOffset < minorLateralOffset) {
                        minorPosOverLane = posOverLane;
                        minorLateralOffset = lateralOffset;
                        nearestLane = lane;
                    }
                }
                // get sumo base object of POI (And all common attributes)
                CommonXMLStructure::SumoBaseObject* POIBaseObject = POI->getSumoBaseObject();
                // add specific attributes
                POIBaseObject->addStringAttribute(SUMO_ATTR_LANE, nearestLane->getID());
                POIBaseObject->addDoubleAttribute(SUMO_ATTR_POSITION, minorPosOverLane);
                POIBaseObject->addBoolAttribute(SUMO_ATTR_FRIENDLY_POS, POI->getFriendlyPos());
                POIBaseObject->addDoubleAttribute(SUMO_ATTR_POSITION_LAT, 0);
                // remove POI
                myUndoList->begin(POI, TL("attach POI into lane"));
                myNet->deleteAdditional(POI, myUndoList);
                // add new POI use route handler
                additionalHanlder.parseSumoBaseObject(POIBaseObject);
                myUndoList->end();
            }
        } else {
            // get sumo base object of POI (And all common attributes)
            CommonXMLStructure::SumoBaseObject* POIBaseObject = POI->getSumoBaseObject();
            // add specific attributes
            POIBaseObject->addDoubleAttribute(SUMO_ATTR_X, POI->x());
            POIBaseObject->addDoubleAttribute(SUMO_ATTR_Y, POI->y());
            // remove POI
            myUndoList->begin(POI, TL("release POI from lane"));
            myNet->deleteAdditional(POI, myUndoList);
            // add new POI use route handler
            additionalHanlder.parseSumoBaseObject(POIBaseObject);
            myUndoList->end();
        }
        // update view after transform
        updateViewNet();
    }
    return 1;
}


long
GNEViewNet::onCmdReverse(FXObject*, FXSelector, void*) {
    // obtain demand element at popup position
    GNEDemandElement* demandElement = getDemandElementAtPopupPosition();
    if (demandElement) {
        // begin undo list
        myUndoList->begin(demandElement, TLF("reverse % '%'", demandElement->getTagStr(), demandElement->getID()));
        GNERouteHandler::reverse(demandElement);
        myUndoList->end();
    }
    return 1;
}


long
GNEViewNet::onCmdAddReverse(FXObject*, FXSelector, void*) {
    // obtain demand element at popup position
    GNEDemandElement* demandElement = getDemandElementAtPopupPosition();
    if (demandElement) {
        // begin undo list
        myUndoList->begin(demandElement, TLF("add reverse '%'", demandElement->getTagStr()));
        GNERouteHandler::addReverse(demandElement);
        myUndoList->end();
    }
    return 1;
}


long
GNEViewNet::onCmdSetCustomGeometryPoint(FXObject*, FXSelector, void*) {
    // get element at popup position
    GNELane* lane = getLaneAtPopupPosition();
    GNEPoly* poly = getPolygonAtPopupPosition();
    GNETAZ* TAZ = getTAZAtPopupPosition();
    // check element
    if (lane != nullptr) {
        // make a copy of edge geometry
        PositionVector edgeGeometry = lane->getParentEdge()->getNBEdge()->getGeometry();
        // get index position
        const int index = edgeGeometry.indexOfClosest(getPositionInformation(), true);
        // get new position
        Position newPosition = edgeGeometry[index];
        // edit using modal GNEGeometryPointDialog
        GNEGeometryPointDialog(this, &newPosition);  // NOSONAR, constructor returns after dialog has been closed
        // now check position
        if (newPosition != edgeGeometry[index]) {
            // update new position
            edgeGeometry[index] = newPosition;
            // begin undo list
            myUndoList->begin(lane->getParentEdge(), TL("change edge Geometry Point position"));
            // continue depending of index
            if (index == 0) {
                // change shape start
                GNEChange_Attribute::changeAttribute(lane->getParentEdge(), GNE_ATTR_SHAPE_START, toString(edgeGeometry.front()), myUndoList);
            } else if (index == ((int)edgeGeometry.size() - 1)) {
                // change shape end
                GNEChange_Attribute::changeAttribute(lane->getParentEdge(), GNE_ATTR_SHAPE_END, toString(edgeGeometry.back()), myUndoList);
            } else {
                // remove front and back geometry points
                edgeGeometry.pop_front();
                edgeGeometry.pop_back();
                // change shape
                GNEChange_Attribute::changeAttribute(lane->getParentEdge(), SUMO_ATTR_SHAPE, toString(edgeGeometry), myUndoList);
            }
            // end undo list
            myUndoList->end();
        }
    } else if (poly != nullptr) {
        // make a copy of polygon geometry
        PositionVector polygonGeometry = poly->getShape();
        // get index position
        const int index = polygonGeometry.indexOfClosest(getPositionInformation(), true);
        // get new position
        Position newPosition = polygonGeometry[index];
        // edit using modal GNEGeometryPointDialog
        GNEGeometryPointDialog(this, &newPosition);  // NOSONAR, constructor returns after dialog has been closed
        // now check position
        if (newPosition != polygonGeometry[index]) {
            // update new position
            polygonGeometry[index] = newPosition;
            // begin undo list
            myUndoList->begin(poly, TL("change polygon Geometry Point position"));
            // change shape
            GNEChange_Attribute::changeAttribute(poly, SUMO_ATTR_SHAPE, toString(polygonGeometry), myUndoList);
            // end undo list
            myUndoList->end();
        }
    } else if (TAZ != nullptr) {
        // make a copy of TAZ geometry
        PositionVector TAZGeometry = TAZ->getAdditionalGeometry().getShape();
        // get index position
        const int index = TAZGeometry.indexOfClosest(getPositionInformation(), true);
        // get new position
        Position newPosition = TAZGeometry[index];
        // edit using modal GNEGeometryPointDialog
        GNEGeometryPointDialog(this, &newPosition);  // NOSONAR, constructor returns after dialog has been closed
        // now check position
        if (newPosition != TAZGeometry[index]) {
            // update new position
            TAZGeometry[index] = newPosition;
            // begin undo list
            myUndoList->begin(TAZ, TL("change TAZ Geometry Point position"));
            // change shape
            GNEChange_Attribute::changeAttribute(TAZ, SUMO_ATTR_SHAPE, toString(TAZGeometry), myUndoList);
            // end undo list
            myUndoList->end();
        }
    }
    return 1;
}


long
GNEViewNet::onCmdResetEndPoints(FXObject*, FXSelector, void*) {
    // get lane at popup position
    GNELane* laneAtPopupPosition = getLaneAtPopupPosition();
    // check element
    if (laneAtPopupPosition != nullptr) {
        // get parent edge
        GNEEdge* edge = laneAtPopupPosition->getParentEdge();
        // check if edge is selected
        if (edge->isAttributeCarrierSelected()) {
            // get selected edges
            const auto selectedEdges = myNet->getAttributeCarriers()->getSelectedEdges();
            // begin undo list
            myUndoList->begin(edge, TL("reset end points of selected edges"));
            // iterate over edges
            for (const auto& selectedEdge : selectedEdges) {
                // reset both end points
                selectedEdge->setAttribute(GNE_ATTR_SHAPE_START, "", myUndoList);
                selectedEdge->setAttribute(GNE_ATTR_SHAPE_END, "", myUndoList);
            }
            // end undo list
            myUndoList->end();
        } else {
            // begin undo list
            myUndoList->begin(edge, TL("reset end points of edge '") + edge->getID());
            // reset both end points
            edge->setAttribute(GNE_ATTR_SHAPE_START, "", myUndoList);
            edge->setAttribute(GNE_ATTR_SHAPE_END, "", myUndoList);
            // end undo list
            myUndoList->end();
        }
    }
    return 1;
}


long
GNEViewNet::onCmdDuplicateLane(FXObject*, FXSelector, void*) {
    GNELane* laneAtPopupPosition = getLaneAtPopupPosition();
    if (laneAtPopupPosition != nullptr) {
        // when duplicating an unselected lane, keep all connections as they
        // are, otherwise recompute them
        if (laneAtPopupPosition->isAttributeCarrierSelected()) {
            myUndoList->begin(laneAtPopupPosition, TL("duplicate selected lanes"));
            const auto selectedLanes = myNet->getAttributeCarriers()->getSelectedLanes();
            for (const auto& lane : selectedLanes) {
                myNet->duplicateLane(lane, myUndoList, true);
            }
            myUndoList->end();
        } else {
            myUndoList->begin(laneAtPopupPosition, TL("duplicate lane"));
            myNet->duplicateLane(laneAtPopupPosition, myUndoList, false);
            myUndoList->end();
        }
    }
    return 1;
}


long
GNEViewNet::onCmdEditLaneShape(FXObject*, FXSelector, void*) {
    // Obtain lane under mouse
    GNELane* lane = getLaneAtPopupPosition();
    if (lane) {
        myEditNetworkElementShapes.startEditCustomShape(lane);
    }
    // destroy pop-up and update view Net
    destroyPopup();
    setFocus();
    return 1;
}


long
GNEViewNet::onCmdResetLaneCustomShape(FXObject*, FXSelector, void*) {
    GNELane* laneAtPopupPosition = getLaneAtPopupPosition();
    if (laneAtPopupPosition != nullptr) {
        // when duplicating an unselected lane, keep all connections as they
        // are, otherwise recompute them
        if (laneAtPopupPosition->isAttributeCarrierSelected()) {
            myUndoList->begin(laneAtPopupPosition, TL("reset custom lane shapes"));
            const auto selectedLanes = myNet->getAttributeCarriers()->getSelectedLanes();
            for (const auto& lane : selectedLanes) {
                lane->setAttribute(SUMO_ATTR_CUSTOMSHAPE, "", myUndoList);
            }
            myUndoList->end();
        } else {
            myUndoList->begin(laneAtPopupPosition, TL("reset custom lane shape"));
            laneAtPopupPosition->setAttribute(SUMO_ATTR_CUSTOMSHAPE, "", myUndoList);
            myUndoList->end();
        }
    }
    return 1;
}


long
GNEViewNet::onCmdResetOppositeLane(FXObject*, FXSelector, void*) {
    GNELane* laneAtPopupPosition = getLaneAtPopupPosition();
    if (laneAtPopupPosition != nullptr) {
        // when duplicating an unselected lane, keep all connections as they
        // are, otherwise recompute them
        if (laneAtPopupPosition->isAttributeCarrierSelected()) {
            myUndoList->begin(laneAtPopupPosition, TL("reset opposite lanes"));
            const auto selectedLanes = myNet->getAttributeCarriers()->getSelectedLanes();
            for (const auto& lane : selectedLanes) {
                lane->setAttribute(GNE_ATTR_OPPOSITE, "", myUndoList);
            }
            myUndoList->end();
        } else {
            myUndoList->begin(laneAtPopupPosition, TL("reset opposite lane"));
            laneAtPopupPosition->setAttribute(GNE_ATTR_OPPOSITE, "", myUndoList);
            myUndoList->end();
        }
    }
    return 1;
}


long
GNEViewNet::onCmdLaneOperation(FXObject*, FXSelector sel, void*) {
    GNELane* laneAtPopupPosition = getLaneAtPopupPosition();
    if (laneAtPopupPosition) {
        // check lane operation
        switch (FXSELID(sel)) {
            case MID_GNE_LANE_TRANSFORM_SIDEWALK:
                return restrictLane(laneAtPopupPosition, SVC_PEDESTRIAN);
            case MID_GNE_LANE_TRANSFORM_BIKE:
                return restrictLane(laneAtPopupPosition, SVC_BICYCLE);
            case MID_GNE_LANE_TRANSFORM_BUS:
                return restrictLane(laneAtPopupPosition, SVC_BUS);
            case MID_GNE_LANE_TRANSFORM_GREENVERGE:
                return restrictLane(laneAtPopupPosition, SVC_IGNORING);
            case MID_GNE_LANE_ADD_SIDEWALK:
                return addRestrictedLane(laneAtPopupPosition, SVC_PEDESTRIAN, false);
            case MID_GNE_LANE_ADD_BIKE:
                return addRestrictedLane(laneAtPopupPosition, SVC_BICYCLE, false);
            case MID_GNE_LANE_ADD_BUS:
                return addRestrictedLane(laneAtPopupPosition, SVC_BUS, false);
            case MID_GNE_LANE_ADD_GREENVERGE_FRONT:
                return addRestrictedLane(laneAtPopupPosition, SVC_IGNORING, true);
            case MID_GNE_LANE_ADD_GREENVERGE_BACK:
                return addRestrictedLane(laneAtPopupPosition, SVC_IGNORING, false);
            case MID_GNE_LANE_REMOVE_SIDEWALK:
                return removeRestrictedLane(laneAtPopupPosition, SVC_PEDESTRIAN);
            case MID_GNE_LANE_REMOVE_BIKE:
                return removeRestrictedLane(laneAtPopupPosition, SVC_BICYCLE);
            case MID_GNE_LANE_REMOVE_BUS:
                return removeRestrictedLane(laneAtPopupPosition, SVC_BUS);
            case MID_GNE_LANE_REMOVE_GREENVERGE:
                return removeRestrictedLane(laneAtPopupPosition, SVC_IGNORING);
            default:
                return 0;
        }
    } else {
        return 0;
    }
}


long
GNEViewNet::onCmdLaneReachability(FXObject* menu, FXSelector, void*) {
    GNELane* laneAtPopupPosition = getLaneAtPopupPosition();
    if (laneAtPopupPosition != nullptr) {
        // obtain vClass
        const SUMOVehicleClass vClass = SumoVehicleClassStrings.get(dynamic_cast<FXMenuCommand*>(menu)->getText().text());
        // calculate reachability
        myNet->getPathManager()->getPathCalculator()->calculateReachability(vClass, laneAtPopupPosition->getParentEdge());
        // select all lanes with reachability greater than 0
        myUndoList->begin(laneAtPopupPosition, TL("select lane reachability"));
        for (const auto& edge : myNet->getAttributeCarriers()->getEdges()) {
            for (const auto& lane : edge.second->getLanes()) {
                if (lane->getReachability() >= 0) {
                    lane->setAttribute(GNE_ATTR_SELECTED, "true", myUndoList);
                }
            }
        }
        myUndoList->end();
    }
    // update viewNet
    updateViewNet();
    return 1;
}


long
GNEViewNet::onCmdOpenAdditionalDialog(FXObject*, FXSelector, void*) {
    // retrieve additional under cursor
    GNEAdditional* addtional = getAdditionalAtPopupPosition();
    // check if additional can open dialog
    if (addtional && addtional->getTagProperty().hasDialog()) {
        addtional->openAdditionalDialog();
    }
    return 1;
}


void
GNEViewNet::processClick(void* eventData) {
    FXEvent* evt = (FXEvent*)eventData;
    // process click
    destroyPopup();
    setFocus();
    myChanger->onLeftBtnPress(eventData);
    grab();
    // Check there are double click
    if (evt->click_count == 2) {
        handle(this, FXSEL(SEL_DOUBLECLICKED, 0), eventData);
    }
}


void
GNEViewNet::updateCursor() {
    // first check if we're panning
    if (myPanning) {
        // move view
        setDefaultCursor(GUICursorSubSys::getCursor(GUICursor::MOVEVIEW));
        setDragCursor(GUICursorSubSys::getCursor(GUICursor::MOVEVIEW));
    } else {
        // declare flags
        bool cursorMoveView = false;
        bool cursorInspect = false;
        bool cursorSelect = false;
        bool cursorMoveElement = false;
        bool cursorDelete = false;
        // continue depending of supermode
        if (myEditModes.isCurrentSupermodeNetwork()) {
            // move view
            if ((myEditModes.networkEditMode == NetworkEditMode::NETWORK_SELECT) ||
                    (myEditModes.networkEditMode == NetworkEditMode::NETWORK_CREATE_EDGE) ||
                    (myEditModes.networkEditMode == NetworkEditMode::NETWORK_ADDITIONAL) ||
                    (myEditModes.networkEditMode == NetworkEditMode::NETWORK_SHAPE) ||
                    (myEditModes.networkEditMode == NetworkEditMode::NETWORK_TAZ)) {
                cursorMoveView = true;
            }
            // specific mode
            if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_INSPECT) {
                cursorInspect = true;
            } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_SELECT) {
                cursorSelect = true;
            } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_MOVE) {
                cursorMoveElement = true;
            } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_DELETE) {
                cursorDelete = true;
            }
        } else if (myEditModes.isCurrentSupermodeDemand()) {
            // move view
            if ((myEditModes.demandEditMode == DemandEditMode::DEMAND_SELECT) ||
                    (myEditModes.demandEditMode == DemandEditMode::DEMAND_VEHICLE) ||
                    (myEditModes.demandEditMode == DemandEditMode::DEMAND_STOP)) {
                cursorMoveView = true;
            }
            // specific mode
            if (myEditModes.demandEditMode == DemandEditMode::DEMAND_INSPECT) {
                cursorInspect = true;
            } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_SELECT) {
                cursorSelect = true;
            } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_MOVE) {
                cursorMoveElement = true;
            } else if (myEditModes.demandEditMode == DemandEditMode::DEMAND_DELETE) {
                cursorDelete = true;
            }
        } else if (myEditModes.isCurrentSupermodeData()) {
            // move view
            if (myEditModes.dataEditMode == DataEditMode::DATA_SELECT) {
                cursorMoveView = true;
            }
            // specific mode
            if (myEditModes.dataEditMode == DataEditMode::DATA_INSPECT) {
                cursorInspect = true;
            } else if (myEditModes.dataEditMode == DataEditMode::DATA_SELECT) {
                cursorSelect = true;
            } else if (myEditModes.dataEditMode == DataEditMode::DATA_DELETE) {
                cursorDelete = true;
            }
        }
        // set cursor
        if (myMouseButtonKeyPressed.controlKeyPressed() && cursorMoveView) {
            // move view cursor if control key is pressed
            setDefaultCursor(GUICursorSubSys::getCursor(GUICursor::MOVEVIEW));
            setDragCursor(GUICursorSubSys::getCursor(GUICursor::MOVEVIEW));
        } else if (cursorInspect) {
            // special case for inspect lanes
            if (!checkSelectEdges() && myEditModes.isCurrentSupermodeNetwork() && (myEditModes.networkEditMode == NetworkEditMode::NETWORK_INSPECT)) {
                // inspect lane cursor
                setDefaultCursor(GUICursorSubSys::getCursor(GUICursor::INSPECT_LANE));
                setDragCursor(GUICursorSubSys::getCursor(GUICursor::INSPECT_LANE));
            } else {
                // inspect cursor
                setDefaultCursor(GUICursorSubSys::getCursor(GUICursor::INSPECT));
                setDragCursor(GUICursorSubSys::getCursor(GUICursor::INSPECT));
            }
        } else if (cursorSelect) {
            // special case for select lanes
            if (!checkSelectEdges() && myEditModes.isCurrentSupermodeNetwork() && (myEditModes.networkEditMode == NetworkEditMode::NETWORK_SELECT)) {
                // select lane cursor
                setDefaultCursor(GUICursorSubSys::getCursor(GUICursor::SELECT_LANE));
                setDragCursor(GUICursorSubSys::getCursor(GUICursor::SELECT_LANE));
            } else {
                // select cursor
                setDefaultCursor(GUICursorSubSys::getCursor(GUICursor::SELECT));
                setDragCursor(GUICursorSubSys::getCursor(GUICursor::SELECT));
            }
        } else if (cursorMoveElement) {
            // move cursor
            setDefaultCursor(GUICursorSubSys::getCursor(GUICursor::MOVEELEMENT));
            setDragCursor(GUICursorSubSys::getCursor(GUICursor::MOVEELEMENT));
        } else if (cursorDelete) {
            // delete cursor
            setDefaultCursor(GUICursorSubSys::getCursor(GUICursor::DELETE_CURSOR));
            setDragCursor(GUICursorSubSys::getCursor(GUICursor::DELETE_CURSOR));
        } else {
            // default cursor
            setDefaultCursor(GUICursorSubSys::getCursor(GUICursor::DEFAULT));
            setDragCursor(GUICursorSubSys::getCursor(GUICursor::DEFAULT));
        }
    }
}


int
GNEViewNet::drawGLElements(const Boundary& bound) {
    // udpdate drawing toggle
    if (myDrawingToggle > 10000) {
        myDrawingToggle = 0;
    } else {
        myDrawingToggle++;
    }
    // set default scale
    myVisualizationSettings->scale = m2p(SUMO_const_laneWidth);
    // calculate boundary extremes
    const float minB[2] = { (float)bound.xmin(), (float)bound.ymin() };
    const float maxB[2] = { (float)bound.xmax(), (float)bound.ymax() };
    // reset gl line to 1
    glLineWidth(1);
    // set drawing modes
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glEnable(GL_POLYGON_OFFSET_LINE);
    // draw all elements between minB and maxB, obtain objects included in minB and maxB
    return myGrid->Search(minB, maxB, *myVisualizationSettings);
}


void
GNEViewNet::drawGrid() const {
    // depending of the visualizationSettings, enable or disable check box show grid
    if (myVisualizationSettings->showGrid) {
        // change show grid
        if (!myNetworkViewOptions.menuCheckToggleGrid->amChecked() ||
                !myDemandViewOptions.menuCheckToggleGrid->amChecked()) {
            // change to true
            myNetworkViewOptions.menuCheckToggleGrid->setChecked(true);
            myDemandViewOptions.menuCheckToggleGrid->setChecked(true);
            // update show grid buttons
            myNetworkViewOptions.menuCheckToggleGrid->update();
            myNetworkViewOptions.menuCheckToggleGrid->update();
        }
        // draw grid only in network and demand mode
        if (myEditModes.isCurrentSupermodeNetwork() || myEditModes.isCurrentSupermodeDemand()) {
            paintGLGrid();
        }
    } else {
        // change show grid
        if (myNetworkViewOptions.menuCheckToggleGrid->amChecked() ||
                myDemandViewOptions.menuCheckToggleGrid->amChecked()) {
            // change to false
            myNetworkViewOptions.menuCheckToggleGrid->setChecked(false);
            myDemandViewOptions.menuCheckToggleGrid->setChecked(false);
            // update show grid buttons
            myNetworkViewOptions.menuCheckToggleGrid->update();
            myNetworkViewOptions.menuCheckToggleGrid->update();
        }
    }
}


long
GNEViewNet::onCmdResetEdgeEndPoints(FXObject*, FXSelector, void*) {
    // Obtain junction under mouse
    GNEJunction* junction = getJunctionAtPopupPosition();
    if (junction) {
        myUndoList->begin(GUIIcon::EDGE, TL("reset edge endpoints"));
        // are, otherwise recompute them
        if (junction->isAttributeCarrierSelected()) {
            const auto selectedJunctions = myNet->getAttributeCarriers()->getSelectedJunctions();
            for (const auto& selectedJunction : selectedJunctions) {
                // reset shape end from incoming edges
                for (const auto& incomingEdge : selectedJunction->getGNEIncomingEdges()) {
                    incomingEdge->setAttribute(GNE_ATTR_SHAPE_END, "", myUndoList);
                }
                // reset shape start from outgoing edges
                for (const auto& outgoingEdge : selectedJunction->getGNEOutgoingEdges()) {
                    outgoingEdge->setAttribute(GNE_ATTR_SHAPE_START, "", myUndoList);
                }
            }
        } else {
            // reset shape end from incoming edges
            for (const auto& incomingEdge : junction->getGNEIncomingEdges()) {
                incomingEdge->setAttribute(GNE_ATTR_SHAPE_END, "", myUndoList);
            }
            // reset shape start from outgoing edges
            for (const auto& outgoingEdge : junction->getGNEOutgoingEdges()) {
                outgoingEdge->setAttribute(GNE_ATTR_SHAPE_START, "", myUndoList);
            }
        }
        myUndoList->end();
    }
    // destroy pop-up and set focus in view net
    destroyPopup();
    setFocus();
    return 1;
}


long
GNEViewNet::onCmdEditJunctionShape(FXObject*, FXSelector, void*) {
    // Obtain junction under mouse
    GNEJunction* junction = getJunctionAtPopupPosition();
    if (junction) {
        // check if network has to be updated
        if (junction->getNBNode()->getShape().size() == 0) {
            // recompute the whole network
            myNet->computeAndUpdate(OptionsCont::getOptions(), false);
        }
        // if grid is enabled, show warning
        if (myVisualizationSettings->showGrid) {
            WRITE_WARNING(TL("Grid is still active, press ctrl+g to deactivate"));
        }
        // start edit custom shape
        myEditNetworkElementShapes.startEditCustomShape(junction);
    }
    // destroy pop-up and set focus in view net
    destroyPopup();
    setFocus();
    return 1;
}


long
GNEViewNet::onCmdResetJunctionShape(FXObject*, FXSelector, void*) {
    // Obtain junction under mouse
    GNEJunction* junction = getJunctionAtPopupPosition();
    if (junction) {
        // are, otherwise recompute them
        if (junction->isAttributeCarrierSelected()) {
            myUndoList->begin(junction, TL("reset custom junction shapes"));
            const auto selectedJunctions = myNet->getAttributeCarriers()->getSelectedJunctions();
            for (const auto& selectedJunction : selectedJunctions) {
                selectedJunction->setAttribute(SUMO_ATTR_SHAPE, "", myUndoList);
            }
            myUndoList->end();
        } else {
            myUndoList->begin(junction, TL("reset custom junction shape"));
            junction->setAttribute(SUMO_ATTR_SHAPE, "", myUndoList);
            myUndoList->end();
        }
    }
    // destroy pop-up and set focus in view net
    destroyPopup();
    setFocus();
    return 1;
}


long
GNEViewNet::onCmdReplaceJunction(FXObject*, FXSelector, void*) {
    GNEJunction* junction = getJunctionAtPopupPosition();
    if (junction != nullptr) {
        myNet->replaceJunctionByGeometry(junction, myUndoList);
        updateViewNet();
    }
    // destroy pop-up and set focus in view net
    destroyPopup();
    setFocus();
    return 1;
}


long
GNEViewNet::onCmdSplitJunction(FXObject*, FXSelector, void*) {
    GNEJunction* junction = getJunctionAtPopupPosition();
    if (junction != nullptr) {
        myNet->splitJunction(junction, false, myUndoList);
        updateViewNet();
    }
    // destroy pop-up and set focus in view net
    destroyPopup();
    setFocus();
    return 1;
}


long
GNEViewNet::onCmdSplitJunctionReconnect(FXObject*, FXSelector, void*) {
    GNEJunction* junction = getJunctionAtPopupPosition();
    if (junction != nullptr) {
        myNet->splitJunction(junction, true, myUndoList);
        updateViewNet();
    }
    // destroy pop-up and set focus in view net
    destroyPopup();
    setFocus();
    return 1;
}

long
GNEViewNet::onCmdSelectRoundabout(FXObject*, FXSelector, void*) {
    GNEJunction* junction = getJunctionAtPopupPosition();
    if (junction != nullptr) {
        myNet->selectRoundabout(junction, myUndoList);
        updateViewNet();
    }
    // destroy pop-up and set focus in view net
    destroyPopup();
    setFocus();
    return 1;
}

long
GNEViewNet::onCmdConvertRoundabout(FXObject*, FXSelector, void*) {
    GNEJunction* junction = getJunctionAtPopupPosition();
    if (junction != nullptr) {
        myNet->createRoundabout(junction, myUndoList);
        updateViewNet();
    }
    // destroy pop-up and set focus in view net
    destroyPopup();
    setFocus();
    return 1;
}


long
GNEViewNet::onEnterConvertRoundabout(FXObject*, FXSelector, void*) {
    myDrawPreviewRoundabout = true;
    update();
    return 1;
}


long
GNEViewNet::onLeaveConvertRoundabout(FXObject*, FXSelector, void*) {
    myDrawPreviewRoundabout = false;
    update();
    return 1;
}


long
GNEViewNet::onCmdClearConnections(FXObject*, FXSelector, void*) {
    GNEJunction* junction = getJunctionAtPopupPosition();
    if (junction != nullptr) {
        // make sure we do not inspect the connection will it is being deleted
        if ((myInspectedAttributeCarriers.size() > 0) && (myInspectedAttributeCarriers.front()->getTagProperty().getTag() == SUMO_TAG_CONNECTION)) {
            myViewParent->getInspectorFrame()->clearInspectedAC();
        }
        // make sure that connections isn't the front attribute
        if (myFrontAttributeCarrier != nullptr && (myFrontAttributeCarrier->getTagProperty().getTag() == SUMO_TAG_CONNECTION)) {
            myFrontAttributeCarrier = nullptr;
        }
        // check if we're handling a selection
        if (junction->isAttributeCarrierSelected()) {
            const auto selectedJunctions = myNet->getAttributeCarriers()->getSelectedJunctions();
            myUndoList->begin(GUIIcon::CONNECTION, TL("clear connections of selected junctions"));
            for (const auto& selectedJunction : selectedJunctions) {
                myNet->clearJunctionConnections(selectedJunction, myUndoList);
            }
            myUndoList->end();
        } else {
            myNet->clearJunctionConnections(junction, myUndoList);
        }
        updateViewNet();
    }
    // destroy pop-up and set focus in view net
    destroyPopup();
    setFocus();
    return 1;
}


long
GNEViewNet::onCmdResetConnections(FXObject*, FXSelector, void*) {
    GNEJunction* junction = getJunctionAtPopupPosition();
    if (junction != nullptr) {
        // make sure we do not inspect the connection will it is being deleted
        if ((myInspectedAttributeCarriers.size() > 0) && myInspectedAttributeCarriers.front()->getTagProperty().getTag() == SUMO_TAG_CONNECTION) {
            myViewParent->getInspectorFrame()->clearInspectedAC();
        }
        // make sure that connections isn't the front attribute
        if (myFrontAttributeCarrier != nullptr && (myFrontAttributeCarrier->getTagProperty().getTag() == SUMO_TAG_CONNECTION)) {
            myFrontAttributeCarrier = nullptr;
        }
        // check if we're handling a selection
        if (junction->isAttributeCarrierSelected()) {
            const auto selectedJunctions = myNet->getAttributeCarriers()->getSelectedJunctions();
            myUndoList->begin(GUIIcon::CONNECTION, TL("reset connections of selected junctions"));
            for (const auto& selectedJunction : selectedJunctions) {
                myNet->resetJunctionConnections(selectedJunction, myUndoList);
            }
            myUndoList->end();
        } else {
            myNet->resetJunctionConnections(junction, myUndoList);
        }
        updateViewNet();
    }
    // destroy pop-up and set focus in view net
    destroyPopup();
    setFocus();
    return 1;
}


long
GNEViewNet::onCmdAddTLS(FXObject*, FXSelector, void*) {
    GNEJunction* junction = getJunctionAtPopupPosition();
    if (junction != nullptr) {
        // check if we're adding TLS in multiple junctions
        if (junction->isAttributeCarrierSelected()) {
            const auto selectedJunctions = myNet->getAttributeCarriers()->getSelectedJunctions();
            myNet->getViewNet()->getUndoList()->begin(GUIIcon::MODETLS, TL("add TLS in multiple junctions"));
            for (const auto& selectedJunction : selectedJunctions) {
                selectedJunction->setAttribute(SUMO_ATTR_TYPE, "traffic_light", myUndoList);
            }
            myNet->getViewNet()->getUndoList()->end();
        } else {
            // change junction type
            junction->setAttribute(SUMO_ATTR_TYPE, "traffic_light", myUndoList);
            // change to TLS Mode
            myEditModes.setNetworkEditMode(NetworkEditMode::NETWORK_TLS, true);
            // set junction in TLS mode
            myViewParent->getTLSEditorFrame()->editJunction(junction);
        }
    }
    // destroy pop-up and set focus in view net
    destroyPopup();
    setFocus();
    return 1;
}


long
GNEViewNet::onCmdAddJoinTLS(FXObject*, FXSelector, void*) {
    GNEJunction* junction = getJunctionAtPopupPosition();
    if (junction != nullptr) {
        // check if we're adding TLS in multiple junctions
        if (junction->isAttributeCarrierSelected()) {
            myNet->getViewNet()->getUndoList()->begin(GUIIcon::MODETLS, TL("add TLS in multiple junctions"));
        }
        // change junction type
        junction->setAttribute(SUMO_ATTR_TYPE, "traffic_light", myUndoList);
        // if TLS was sucesfully created, apply the same TLID to other selected junctions
        if (junction->getAttribute(SUMO_ATTR_TLID).size() > 0) {
            const auto selectedJunctions = myNet->getAttributeCarriers()->getSelectedJunctions();
            // iterate over all selected junctions
            for (const auto& selectedJunction : selectedJunctions) {
                // check that doesn't have a TL
                if (selectedJunction->getNBNode()->getControllingTLS().empty()) {
                    selectedJunction->setAttribute(SUMO_ATTR_TYPE, "traffic_light", myUndoList);
                    selectedJunction->setAttribute(SUMO_ATTR_TLID, junction->getAttribute(SUMO_ATTR_TLID), myUndoList);
                }
            }
        }
        // rename traffic light
        if (junction->getNBNode()->getControllingTLS().size() > 0) {
            const auto TLSDef = (*junction->getNBNode()->getControllingTLS().begin());
            if (!myNet->getTLLogicCont().exist(TLSDef->getID() + "_joined")) {
                myUndoList->add(new GNEChange_TLS(junction, TLSDef, TLSDef->getID() + "_joined"), true);
            }
        }
        // end undoList
        if (junction->isAttributeCarrierSelected()) {
            myNet->getViewNet()->getUndoList()->end();
        }
        // change to TLS Mode
        myEditModes.setNetworkEditMode(NetworkEditMode::NETWORK_TLS, true);
        // set junction in TLS mode
        myViewParent->getTLSEditorFrame()->editJunction(junction);
    }
    // destroy pop-up and set focus in view net
    destroyPopup();
    setFocus();
    return 1;
}

long
GNEViewNet::onCmdEditConnectionShape(FXObject*, FXSelector, void*) {
    // Obtain connection under mouse
    GNEConnection* connection = getConnectionAtPopupPosition();
    if (connection) {
        myEditNetworkElementShapes.startEditCustomShape(connection);
    }
    // if grid is enabled, show warning
    if (myVisualizationSettings->showGrid) {
        WRITE_WARNING(TL("Grid is still active, press ctrl+g to deactivate"));
    }
    // destroy pop-up and update view Net
    destroyPopup();
    setFocus();
    return 1;
}


long
GNEViewNet::onCmdSmoothConnectionShape(FXObject*, FXSelector, void*) {
    // Obtain connection under mouse
    GNEConnection* connection = getConnectionAtPopupPosition();
    if (connection) {
        connection->smootShape();
    }
    // destroy pop-up and update view Net
    destroyPopup();
    setFocus();
    return 1;
}


long
GNEViewNet::onCmdEditCrossingShape(FXObject*, FXSelector, void*) {
    // Obtain crossing under mouse
    GNECrossing* crossing = getCrossingAtPopupPosition();
    if (crossing) {
        // check if network has to be updated
        if (crossing->getParentJunction()->getNBNode()->getShape().size() == 0) {
            // recompute the whole network
            myNet->computeAndUpdate(OptionsCont::getOptions(), false);
        }
        // if grid is enabled, show warning
        if (myVisualizationSettings->showGrid) {
            WRITE_WARNING(TL("Grid is still active, press ctrl+g to deactivate"));
        }
        // start edit custom shape
        myEditNetworkElementShapes.startEditCustomShape(crossing);
    }
    // destroy pop-up and update view Net
    destroyPopup();
    setFocus();
    return 1;
}


long
GNEViewNet::onCmdEditWalkingAreaShape(FXObject*, FXSelector, void*) {
    // Obtain walkingArea under mouse
    GNEWalkingArea* walkingArea = getWalkingAreaAtPopupPosition();
    if (walkingArea) {
        // check if network has to be updated
        if (walkingArea->getParentJunction()->getNBNode()->getShape().size() == 0) {
            // recompute the whole network
            myNet->computeAndUpdate(OptionsCont::getOptions(), false);
            // if grid is enabled, show warning
            if (myVisualizationSettings->showGrid) {
                WRITE_WARNING(TL("Grid is still active, press ctrl+g to deactivate"));
            }
        }
        // start edit custom shape
        myEditNetworkElementShapes.startEditCustomShape(walkingArea);
    }
    // destroy pop-up and update view Net
    destroyPopup();
    setFocus();
    return 1;
}


long
GNEViewNet::onCmdToggleSelectEdges(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckSelectEdges
    if (myNetworkViewOptions.menuCheckSelectEdges->amChecked() == TRUE) {
        myNetworkViewOptions.menuCheckSelectEdges->setChecked(FALSE);
    } else {
        myNetworkViewOptions.menuCheckSelectEdges->setChecked(TRUE);
    }
    myNetworkViewOptions.menuCheckSelectEdges->update();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_SELECTEDGES)) {
        myNetworkViewOptions.menuCheckSelectEdges->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleShowConnections(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckShowConnections
    if (myNetworkViewOptions.menuCheckShowConnections->amChecked() == TRUE) {
        myNetworkViewOptions.menuCheckShowConnections->setChecked(FALSE);
    } else {
        myNetworkViewOptions.menuCheckShowConnections->setChecked(TRUE);
    }
    myNetworkViewOptions.menuCheckShowConnections->update();
    // if show was enabled, init GNEConnections
    if (myNetworkViewOptions.menuCheckShowConnections->amChecked() == TRUE) {
        getNet()->initGNEConnections();
    }
    // change flag "showLane2Lane" in myVisualizationSettings
    myVisualizationSettings->showLane2Lane = (myNetworkViewOptions.menuCheckShowConnections->amChecked() == TRUE);
    // Hide/show connections require recompute
    getNet()->requireRecompute();
    // Update viewNet to show/hide connections
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_SHOWCONNECTIONS)) {
        myNetworkViewOptions.menuCheckShowConnections->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleHideConnections(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckHideConnections
    if (myNetworkViewOptions.menuCheckHideConnections->amChecked() == TRUE) {
        myNetworkViewOptions.menuCheckHideConnections->setChecked(FALSE);
    } else {
        myNetworkViewOptions.menuCheckHideConnections->setChecked(TRUE);
    }
    myNetworkViewOptions.menuCheckHideConnections->update();
    // Update viewNet to show/hide connections
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_HIDECONNECTIONS)) {
        myNetworkViewOptions.menuCheckHideConnections->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleShowAdditionalSubElements(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckShowAdditionalSubElements
    if (myNetworkViewOptions.menuCheckShowAdditionalSubElements->amChecked() == TRUE) {
        myNetworkViewOptions.menuCheckShowAdditionalSubElements->setChecked(FALSE);
    } else {
        myNetworkViewOptions.menuCheckShowAdditionalSubElements->setChecked(TRUE);
    }
    myNetworkViewOptions.menuCheckShowAdditionalSubElements->update();
    // Update viewNet to show/hide sub elements
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_SHOWSUBADDITIONALS)) {
        myNetworkViewOptions.menuCheckShowAdditionalSubElements->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleShowTAZElements(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckShowAdditionalSubElements
    if (myNetworkViewOptions.menuCheckShowTAZElements->amChecked() == TRUE) {
        myNetworkViewOptions.menuCheckShowTAZElements->setChecked(FALSE);
    } else {
        myNetworkViewOptions.menuCheckShowTAZElements->setChecked(TRUE);
    }
    myNetworkViewOptions.menuCheckShowTAZElements->update();
    // Update viewNet to show/hide TAZ elements
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_SHOWTAZELEMENTS)) {
        myNetworkViewOptions.menuCheckShowTAZElements->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleExtendSelection(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckExtendSelection
    if (myNetworkViewOptions.menuCheckExtendSelection->amChecked() == TRUE) {
        myNetworkViewOptions.menuCheckExtendSelection->setChecked(FALSE);
    } else {
        myNetworkViewOptions.menuCheckExtendSelection->setChecked(TRUE);
    }
    myNetworkViewOptions.menuCheckExtendSelection->update();
    // Only update view
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_EXTENDSELECTION)) {
        myNetworkViewOptions.menuCheckExtendSelection->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleChangeAllPhases(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckChangeAllPhases
    if (myNetworkViewOptions.menuCheckChangeAllPhases->amChecked() == TRUE) {
        myNetworkViewOptions.menuCheckChangeAllPhases->setChecked(FALSE);
    } else {
        myNetworkViewOptions.menuCheckChangeAllPhases->setChecked(TRUE);
    }
    myNetworkViewOptions.menuCheckChangeAllPhases->update();
    // Only update view
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_CHANGEALLPHASES)) {
        myNetworkViewOptions.menuCheckChangeAllPhases->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleShowGrid(FXObject*, FXSelector sel, void*) {
    // show or hide grid depending of myNetworkViewOptions.menuCheckToggleGrid
    if (myVisualizationSettings->showGrid) {
        myVisualizationSettings->showGrid = false;
        myNetworkViewOptions.menuCheckToggleGrid->setChecked(false);
        myDemandViewOptions.menuCheckToggleGrid->setChecked(false);
    } else {
        myVisualizationSettings->showGrid = true;
        myNetworkViewOptions.menuCheckToggleGrid->setChecked(true);
        myDemandViewOptions.menuCheckToggleGrid->setChecked(true);
    }
    myNetworkViewOptions.menuCheckToggleGrid->update();
    myDemandViewOptions.menuCheckToggleGrid->update();
    // update view to show grid
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_TOGGLEGRID)) {
        myNetworkViewOptions.menuCheckToggleGrid->setFocus();
    } else if (sel == FXSEL(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_SHOWGRID)) {
        myDemandViewOptions.menuCheckToggleGrid->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleDrawJunctionShape(FXObject*, FXSelector sel, void*) {
    // toggle state
    myVisualizationSettings->drawJunctionShape = !myVisualizationSettings->drawJunctionShape;
    // gui button has 'hide' semantics
    const bool hide = !myVisualizationSettings->drawJunctionShape;
    myNetworkViewOptions.menuCheckToggleDrawJunctionShape->setChecked(hide);
    myDemandViewOptions.menuCheckToggleDrawJunctionShape->setChecked(hide);
    myDataViewOptions.menuCheckToggleDrawJunctionShape->setChecked(hide);

    myNetworkViewOptions.menuCheckToggleDrawJunctionShape->update();
    myDemandViewOptions.menuCheckToggleDrawJunctionShape->update();
    myDataViewOptions.menuCheckToggleDrawJunctionShape->update();
    // update view to show DrawJunctionShape
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_TOGGLEDRAWJUNCTIONSHAPE)) {
        myNetworkViewOptions.menuCheckToggleDrawJunctionShape->setFocus();
    } else if (sel == FXSEL(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_TOGGLEDRAWJUNCTIONSHAPE)) {
        myDemandViewOptions.menuCheckToggleDrawJunctionShape->setFocus();
    } else if (sel == FXSEL(SEL_COMMAND, MID_GNE_DATAVIEWOPTIONS_TOGGLEDRAWJUNCTIONSHAPE)) {
        myDataViewOptions.menuCheckToggleDrawJunctionShape->setFocus();
    }
    return 1;
}

long
GNEViewNet::onCmdToggleDrawSpreadVehicles(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckShowDemandElements
    if ((myNetworkViewOptions.menuCheckDrawSpreadVehicles->amChecked() == TRUE) ||
            (myDemandViewOptions.menuCheckDrawSpreadVehicles->amChecked() == TRUE)) {
        myNetworkViewOptions.menuCheckDrawSpreadVehicles->setChecked(FALSE);
        myDemandViewOptions.menuCheckDrawSpreadVehicles->setChecked(FALSE);
    } else {
        myNetworkViewOptions.menuCheckDrawSpreadVehicles->setChecked(TRUE);
        myDemandViewOptions.menuCheckDrawSpreadVehicles->setChecked(TRUE);
    }
    myNetworkViewOptions.menuCheckDrawSpreadVehicles->update();
    myDemandViewOptions.menuCheckDrawSpreadVehicles->update();
    // declare edge set
    std::set<GNEEdge*> edgesToUpdate;
    // compute vehicle geometry
    for (const auto& vehicle : myNet->getAttributeCarriers()->getDemandElements().at(SUMO_TAG_VEHICLE)) {
        if (vehicle.second->getParentEdges().size() > 0) {
            edgesToUpdate.insert(vehicle.second->getParentEdges().front());
        } else if (vehicle.second->getChildDemandElements().size() > 0 && (vehicle.second->getChildDemandElements().front()->getTagProperty().getTag() == GNE_TAG_ROUTE_EMBEDDED)) {
            edgesToUpdate.insert(vehicle.second->getChildDemandElements().front()->getParentEdges().front());
        }
    }
    for (const auto& routeFlow : myNet->getAttributeCarriers()->getDemandElements().at(GNE_TAG_FLOW_ROUTE)) {
        if (routeFlow.second->getParentEdges().size() > 0) {
            edgesToUpdate.insert(routeFlow.second->getParentEdges().front());
        } else if (routeFlow.second->getChildDemandElements().size() > 0 && (routeFlow.second->getChildDemandElements().front()->getTagProperty().getTag() == GNE_TAG_ROUTE_EMBEDDED)) {
            edgesToUpdate.insert(routeFlow.second->getChildDemandElements().front()->getParentEdges().front());
        }
    }
    for (const auto& trip : myNet->getAttributeCarriers()->getDemandElements().at(SUMO_TAG_TRIP)) {
        if (trip.second->getParentEdges().size() > 0) {
            edgesToUpdate.insert(trip.second->getParentEdges().front());
        }
    }
    for (const auto& flow : myNet->getAttributeCarriers()->getDemandElements().at(SUMO_TAG_FLOW)) {
        if (flow.second->getParentEdges().size() > 0) {
            edgesToUpdate.insert(flow.second->getParentEdges().front());
        }
    }
    // update spread geometries of all edges
    for (const auto& edge : edgesToUpdate) {
        edge->updateVehicleSpreadGeometries();
    }
    // update view to show new vehicles positions
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_DRAWSPREADVEHICLES)) {
        myNetworkViewOptions.menuCheckDrawSpreadVehicles->setFocus();
    } else if (sel == FXSEL(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_DRAWSPREADVEHICLES)) {
        myDemandViewOptions.menuCheckDrawSpreadVehicles->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleWarnAboutMerge(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckWarnAboutMerge
    if (myNetworkViewOptions.menuCheckWarnAboutMerge->amChecked() == TRUE) {
        myNetworkViewOptions.menuCheckWarnAboutMerge->setChecked(FALSE);
    } else {
        myNetworkViewOptions.menuCheckWarnAboutMerge->setChecked(TRUE);
    }
    myNetworkViewOptions.menuCheckWarnAboutMerge->update();
    // Only update view
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_ASKFORMERGE)) {
        myNetworkViewOptions.menuCheckWarnAboutMerge->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleShowJunctionBubbles(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckShowJunctionBubble
    if (myNetworkViewOptions.menuCheckShowJunctionBubble->amChecked() == TRUE) {
        myNetworkViewOptions.menuCheckShowJunctionBubble->setChecked(FALSE);
    } else {
        myNetworkViewOptions.menuCheckShowJunctionBubble->setChecked(TRUE);
    }
    myNetworkViewOptions.menuCheckShowJunctionBubble->update();
    // Only update view
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_SHOWBUBBLES)) {
        myNetworkViewOptions.menuCheckShowJunctionBubble->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleMoveElevation(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckMoveElevation
    if (myNetworkViewOptions.menuCheckMoveElevation->amChecked() == TRUE) {
        myNetworkViewOptions.menuCheckMoveElevation->setChecked(FALSE);
    } else {
        myNetworkViewOptions.menuCheckMoveElevation->setChecked(TRUE);
    }
    myNetworkViewOptions.menuCheckMoveElevation->update();
    // Only update view
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_MOVEELEVATION)) {
        myNetworkViewOptions.menuCheckMoveElevation->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleChainEdges(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckMoveElevation
    if (myNetworkViewOptions.menuCheckChainEdges->amChecked() == TRUE) {
        myNetworkViewOptions.menuCheckChainEdges->setChecked(FALSE);
    } else {
        myNetworkViewOptions.menuCheckChainEdges->setChecked(TRUE);
    }
    myNetworkViewOptions.menuCheckChainEdges->update();
    // Only update view
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_CHAINEDGES)) {
        myNetworkViewOptions.menuCheckChainEdges->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleAutoOppositeEdge(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckAutoOppositeEdge
    if (myNetworkViewOptions.menuCheckAutoOppositeEdge->amChecked() == TRUE) {
        myNetworkViewOptions.menuCheckAutoOppositeEdge->setChecked(FALSE);
    } else {
        myNetworkViewOptions.menuCheckAutoOppositeEdge->setChecked(TRUE);
    }
    myNetworkViewOptions.menuCheckAutoOppositeEdge->update();
    // Only update view
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_AUTOOPPOSITEEDGES)) {
        myNetworkViewOptions.menuCheckAutoOppositeEdge->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleHideNonInspecteDemandElements(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckHideNonInspectedDemandElements
    if (myDemandViewOptions.menuCheckHideNonInspectedDemandElements->amChecked() == TRUE) {
        myDemandViewOptions.menuCheckHideNonInspectedDemandElements->setChecked(FALSE);
    } else {
        myDemandViewOptions.menuCheckHideNonInspectedDemandElements->setChecked(TRUE);
    }
    myDemandViewOptions.menuCheckHideNonInspectedDemandElements->update();
    // Only update view
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_HIDENONINSPECTED)) {
        myDemandViewOptions.menuCheckHideNonInspectedDemandElements->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleShowOverlappedRoutes(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckShowOverlappedRoutes
    if (myDemandViewOptions.menuCheckShowOverlappedRoutes->amChecked() == TRUE) {
        myDemandViewOptions.menuCheckShowOverlappedRoutes->setChecked(FALSE);
    } else {
        myDemandViewOptions.menuCheckShowOverlappedRoutes->setChecked(TRUE);
    }
    myDemandViewOptions.menuCheckShowOverlappedRoutes->update();
    // Only update view
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_SHOWOVERLAPPEDROUTES)) {
        myDemandViewOptions.menuCheckShowOverlappedRoutes->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleHideShapes(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckHideShapes
    if (myDemandViewOptions.menuCheckHideShapes->amChecked() == TRUE) {
        myDemandViewOptions.menuCheckHideShapes->setChecked(FALSE);
    } else {
        myDemandViewOptions.menuCheckHideShapes->setChecked(TRUE);
    }
    myDemandViewOptions.menuCheckHideShapes->update();
    // Only update view
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_HIDESHAPES)) {
        myDemandViewOptions.menuCheckHideShapes->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleShowTrips(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckHideShapes
    if (myDemandViewOptions.menuCheckShowAllTrips->amChecked() == TRUE) {
        myDemandViewOptions.menuCheckShowAllTrips->setChecked(FALSE);
    } else {
        myDemandViewOptions.menuCheckShowAllTrips->setChecked(TRUE);
    }
    myDemandViewOptions.menuCheckShowAllTrips->update();
    // Only update view
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_SHOWTRIPS)) {
        myDemandViewOptions.menuCheckShowAllTrips->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleShowAllPersonPlans(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckShowAllPersonPlans
    if (myDemandViewOptions.menuCheckShowAllPersonPlans->amChecked() == TRUE) {
        myDemandViewOptions.menuCheckShowAllPersonPlans->setChecked(FALSE);
    } else {
        myDemandViewOptions.menuCheckShowAllPersonPlans->setChecked(TRUE);
    }
    myDemandViewOptions.menuCheckShowAllPersonPlans->update();
    // Only update view
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_SHOWALLPERSONPLANS)) {
        myDemandViewOptions.menuCheckShowAllPersonPlans->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleLockPerson(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckLockPerson
    if (myDemandViewOptions.menuCheckLockPerson->amChecked() == TRUE) {
        myDemandViewOptions.menuCheckLockPerson->setChecked(FALSE);
    } else if ((myInspectedAttributeCarriers.size() > 0) && myInspectedAttributeCarriers.front()->getTagProperty().isPerson()) {
        myDemandViewOptions.menuCheckLockPerson->setChecked(TRUE);
    }
    myDemandViewOptions.menuCheckLockPerson->update();
    // lock or unlock current inspected person depending of menuCheckLockPerson value
    if (myDemandViewOptions.menuCheckLockPerson->amChecked()) {
        // obtain locked person or person plan
        const GNEDemandElement* personOrPersonPlan = dynamic_cast<const GNEDemandElement*>(myInspectedAttributeCarriers.front());
        if (personOrPersonPlan) {
            // lock person depending if casted demand element is either a person or a person plan
            if (personOrPersonPlan->getTagProperty().isPerson()) {
                myDemandViewOptions.lockPerson(personOrPersonPlan);
            } else {
                myDemandViewOptions.lockPerson(personOrPersonPlan->getParentDemandElements().front());
            }
        }
    } else {
        // unlock current person
        myDemandViewOptions.unlockPerson();
    }
    // update view
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_LOCKPERSON)) {
        myDemandViewOptions.menuCheckLockPerson->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleShowAllContainerPlans(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckShowAllContainerPlans
    if (myDemandViewOptions.menuCheckShowAllContainerPlans->amChecked() == TRUE) {
        myDemandViewOptions.menuCheckShowAllContainerPlans->setChecked(FALSE);
    } else {
        myDemandViewOptions.menuCheckShowAllContainerPlans->setChecked(TRUE);
    }
    myDemandViewOptions.menuCheckShowAllContainerPlans->update();
    // Only update view
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_SHOWALLCONTAINERPLANS)) {
        myDemandViewOptions.menuCheckShowAllContainerPlans->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleLockContainer(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckLockContainer
    if (myDemandViewOptions.menuCheckLockContainer->amChecked() == TRUE) {
        myDemandViewOptions.menuCheckLockContainer->setChecked(FALSE);
    } else if ((myInspectedAttributeCarriers.size() > 0) && myInspectedAttributeCarriers.front()->getTagProperty().isContainer()) {
        myDemandViewOptions.menuCheckLockContainer->setChecked(TRUE);
    }
    myDemandViewOptions.menuCheckLockContainer->update();
    // lock or unlock current inspected container depending of menuCheckLockContainer value
    if (myDemandViewOptions.menuCheckLockContainer->amChecked()) {
        // obtain locked container or container plan
        const GNEDemandElement* containerOrContainerPlan = dynamic_cast<const GNEDemandElement*>(myInspectedAttributeCarriers.front());
        if (containerOrContainerPlan) {
            // lock container depending if casted demand element is either a container or a container plan
            if (containerOrContainerPlan->getTagProperty().isContainer()) {
                myDemandViewOptions.lockContainer(containerOrContainerPlan);
            } else {
                myDemandViewOptions.lockContainer(containerOrContainerPlan->getParentDemandElements().front());
            }
        }
    } else {
        // unlock current container
        myDemandViewOptions.unlockContainer();
    }
    // update view
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_DEMANDVIEWOPTIONS_LOCKCONTAINER)) {
        myDemandViewOptions.menuCheckLockContainer->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleShowAdditionals(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckShowAdditionals
    if (myDataViewOptions.menuCheckShowAdditionals->amChecked() == TRUE) {
        myDataViewOptions.menuCheckShowAdditionals->setChecked(FALSE);
    } else {
        myDataViewOptions.menuCheckShowAdditionals->setChecked(TRUE);
    }
    myDataViewOptions.menuCheckShowAdditionals->update();
    // Only update view
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_DATAVIEWOPTIONS_SHOWADDITIONALS)) {
        myDataViewOptions.menuCheckShowAdditionals->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleShowShapes(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckShowShapes
    if (myDataViewOptions.menuCheckShowShapes->amChecked() == TRUE) {
        myDataViewOptions.menuCheckShowShapes->setChecked(FALSE);
    } else {
        myDataViewOptions.menuCheckShowShapes->setChecked(TRUE);
    }
    myDataViewOptions.menuCheckShowShapes->update();
    // Only update view
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_DATAVIEWOPTIONS_SHOWSHAPES)) {
        myDataViewOptions.menuCheckShowShapes->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleShowDemandElementsNetwork(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckShowDemandElements
    if (myNetworkViewOptions.menuCheckShowDemandElements->amChecked() == TRUE) {
        myNetworkViewOptions.menuCheckShowDemandElements->setChecked(FALSE);
    } else {
        myNetworkViewOptions.menuCheckShowDemandElements->setChecked(TRUE);
    }
    myNetworkViewOptions.menuCheckShowDemandElements->update();
    // compute demand elements
    myNet->computeDemandElements(myViewParent->getGNEAppWindows());
    // update view to show demand elements
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_NETWORKVIEWOPTIONS_SHOWDEMANDELEMENTS)) {
        myNetworkViewOptions.menuCheckShowDemandElements->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleShowDemandElementsData(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckShowDemandElements
    if (myDataViewOptions.menuCheckShowDemandElements->amChecked() == TRUE) {
        myDataViewOptions.menuCheckShowDemandElements->setChecked(FALSE);
    } else {
        myDataViewOptions.menuCheckShowDemandElements->setChecked(TRUE);
    }
    myDataViewOptions.menuCheckShowDemandElements->update();
    // compute demand elements
    myNet->computeDemandElements(myViewParent->getGNEAppWindows());
    // update view to show demand elements
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_DATAVIEWOPTIONS_SHOWDEMANDELEMENTS)) {
        myDataViewOptions.menuCheckShowDemandElements->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleTAZRelDrawing(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckShowDemandElements
    if (myDataViewOptions.menuCheckToggleTAZRelDrawing->amChecked() == TRUE) {
        myDataViewOptions.menuCheckToggleTAZRelDrawing->setChecked(FALSE);
    } else {
        myDataViewOptions.menuCheckToggleTAZRelDrawing->setChecked(TRUE);
    }
    myDataViewOptions.menuCheckToggleTAZRelDrawing->update();
    // update view to show demand elements
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_DATAVIEWOPTIONS_TAZRELDRAWING)) {
        myDataViewOptions.menuCheckToggleTAZRelDrawing->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleTAZDrawFill(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckShowDemandElements
    if (myDataViewOptions.menuCheckToggleTAZDrawFill->amChecked() == TRUE) {
        myDataViewOptions.menuCheckToggleTAZDrawFill->setChecked(FALSE);
    } else {
        myDataViewOptions.menuCheckToggleTAZDrawFill->setChecked(TRUE);
    }
    myDataViewOptions.menuCheckToggleTAZDrawFill->update();
    // update view to show demand elements
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_DATAVIEWOPTIONS_TAZDRAWFILL)) {
        myDataViewOptions.menuCheckToggleTAZDrawFill->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleTAZRelOnlyFrom(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckShowDemandElements
    if (myDataViewOptions.menuCheckToggleTAZRelOnlyFrom->amChecked() == TRUE) {
        myDataViewOptions.menuCheckToggleTAZRelOnlyFrom->setChecked(FALSE);
    } else {
        myDataViewOptions.menuCheckToggleTAZRelOnlyFrom->setChecked(TRUE);
    }
    myDataViewOptions.menuCheckToggleTAZRelOnlyFrom->update();
    // update view to show demand elements
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_DATAVIEWOPTIONS_TAZRELONLYFROM)) {
        myDataViewOptions.menuCheckToggleTAZRelOnlyFrom->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdToggleTAZRelOnlyTo(FXObject*, FXSelector sel, void*) {
    // Toggle menuCheckShowDemandElements
    if (myDataViewOptions.menuCheckToggleTAZRelOnlyTo->amChecked() == TRUE) {
        myDataViewOptions.menuCheckToggleTAZRelOnlyTo->setChecked(FALSE);
    } else {
        myDataViewOptions.menuCheckToggleTAZRelOnlyTo->setChecked(TRUE);
    }
    myDataViewOptions.menuCheckToggleTAZRelOnlyTo->update();
    // update view to show demand elements
    updateViewNet();
    // set focus in menu check again, if this function was called clicking over menu check instead using alt+<key number>
    if (sel == FXSEL(SEL_COMMAND, MID_GNE_DATAVIEWOPTIONS_TAZRELONLYTO)) {
        myDataViewOptions.menuCheckToggleTAZRelOnlyTo->setFocus();
    }
    return 1;
}


long
GNEViewNet::onCmdIntervalBarGenericDataType(FXObject*, FXSelector, void*) {
    myIntervalBar.setGenericDataType();
    return 1;
}


long
GNEViewNet::onCmdIntervalBarDataSet(FXObject*, FXSelector, void*) {
    myIntervalBar.setDataSet();
    return 1;
}


long
GNEViewNet::onCmdIntervalBarLimit(FXObject*, FXSelector, void*) {
    myIntervalBar.setInterval();
    return 1;
}


long
GNEViewNet::onCmdIntervalBarSetBegin(FXObject*, FXSelector, void*) {
    myIntervalBar.setBegin();
    return 1;
}


long
GNEViewNet::onCmdIntervalBarSetEnd(FXObject*, FXSelector, void*) {
    myIntervalBar.setEnd();
    return 1;
}


long
GNEViewNet::onCmdIntervalBarSetParameter(FXObject*, FXSelector, void*) {
    myIntervalBar.setParameter();
    return 1;
}


long
GNEViewNet::onCmdAddSelected(FXObject*, FXSelector, void*) {
    // only select if AC under cursor isn't previously selected
    auto AC = myViewObjectsSelector.getAttributeCarrierFront();
    if (AC && !AC->isAttributeCarrierSelected()) {
        AC->selectAttributeCarrier();
    }
    return 1;
}


long
GNEViewNet::onCmdRemoveSelected(FXObject*, FXSelector, void*) {
    // only unselect if AC under cursor isn't previously selected
    auto AC = myViewObjectsSelector.getAttributeCarrierFront();
    if (AC && AC->isAttributeCarrierSelected()) {
        AC->unselectAttributeCarrier();
    }
    return 1;
}


long
GNEViewNet::onCmdAddEdgeSelected(FXObject*, FXSelector, void*) {
    // only select if edge under cursor isn't previously selected
    auto edge = myViewObjectsSelector.getEdgeFront();
    if (edge && !edge->isAttributeCarrierSelected()) {
        edge->selectAttributeCarrier();
    }
    return 1;
}


long
GNEViewNet::onCmdRemoveEdgeSelected(FXObject*, FXSelector, void*) {
    // only unselect if edge under cursor isn't previously selected
    auto edge = myViewObjectsSelector.getEdgeFront();
    if (edge && edge->isAttributeCarrierSelected()) {
        edge->unselectAttributeCarrier();
    }
    return 1;
}


long
GNEViewNet::onCmdSetNeteditView(FXObject*, FXSelector sel, void*) {
    myEditModes.setView(FXSELID(sel));
    update();
    return 1;
}

// ===========================================================================
// private
// ===========================================================================

void
GNEViewNet::buildEditModeControls() {
    // build supermode buttons
    myEditModes.buildSuperModeButtons();

    // build save elements buttons
    mySaveElements.buildSaveElementsButtons();

    // build time switch buttons
    myTimeFormat.buildTimeFormatButtons();

    // build menu checks for Common checkable buttons
    myCommonCheckableButtons.buildCommonCheckableButtons();

    // build menu checks for Network checkable buttons
    myNetworkCheckableButtons.buildNetworkCheckableButtons();

    // build menu checks for Demand checkable buttons
    myDemandCheckableButtons.buildDemandCheckableButtons();

    // build menu checks of view options Data
    myDataCheckableButtons.buildDataCheckableButtons();

    // Create Vertical separator
    new FXVerticalSeparator(myViewParent->getGNEAppWindows()->getToolbarsGrip().modes, GUIDesignVerticalSeparator);
    // XXX for some reason the vertical groove is not visible. adding more spacing to emphasize the separation
    new FXVerticalSeparator(myViewParent->getGNEAppWindows()->getToolbarsGrip().modes, GUIDesignVerticalSeparator);
    new FXVerticalSeparator(myViewParent->getGNEAppWindows()->getToolbarsGrip().modes, GUIDesignVerticalSeparator);

    // build menu checks of view options Network
    myNetworkViewOptions.buildNetworkViewOptionsMenuChecks();

    // build menu checks of view options Demand
    myDemandViewOptions.buildDemandViewOptionsMenuChecks();

    // build menu checks of view options Data
    myDataViewOptions.buildDataViewOptionsMenuChecks();

    // build interval bar
    myIntervalBar.buildIntervalBarElements();
}


void
GNEViewNet::updateNetworkModeSpecificControls() {
    // get menu checks
    auto& menuChecks = myViewParent->getGNEAppWindows()->getEditMenuCommands().networkViewOptions;
    // hide all checkbox of view options Network
    myNetworkViewOptions.hideNetworkViewOptionsMenuChecks();
    // hide all checkbox of view options Demand
    myDemandViewOptions.hideDemandViewOptionsMenuChecks();
    // hide all checkbox of view options Data
    myDataViewOptions.hideDataViewOptionsMenuChecks();
    // disable all common edit modes
    myCommonCheckableButtons.disableCommonCheckableButtons();
    // disable all network edit modes
    myNetworkCheckableButtons.disableNetworkCheckableButtons();
    // disable all network edit modes
    myDataCheckableButtons.disableDataCheckableButtons();
    // hide interval bar
    myIntervalBar.hideIntervalBar();
    // hide all frames
    myViewParent->hideAllFrames();
    // hide all menuchecks
    myViewParent->getGNEAppWindows()->getEditMenuCommands().networkViewOptions.hideNetworkViewOptionsMenuChecks();
    myViewParent->getGNEAppWindows()->getEditMenuCommands().demandViewOptions.hideDemandViewOptionsMenuChecks();
    myViewParent->getGNEAppWindows()->getEditMenuCommands().dataViewOptions.hideDataViewOptionsMenuChecks();
    // In network mode, always show option "show grid", "draw spread vehicles" and "show demand elements"
    myNetworkViewOptions.menuCheckToggleGrid->show();
    myNetworkViewOptions.menuCheckToggleDrawJunctionShape->show();
    myNetworkViewOptions.menuCheckDrawSpreadVehicles->show();
    myNetworkViewOptions.menuCheckShowDemandElements->show();
    menuChecks.menuCheckToggleGrid->show();
    menuChecks.menuCheckToggleDrawJunctionShape->show();
    menuChecks.menuCheckDrawSpreadVehicles->show();
    menuChecks.menuCheckShowDemandElements->show();
    // show separator
    menuChecks.separator->show();
    // enable selected controls
    switch (myEditModes.networkEditMode) {
        // common modes
        case NetworkEditMode::NETWORK_INSPECT:
            myViewParent->getInspectorFrame()->show();
            myViewParent->getInspectorFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getInspectorFrame();
            myCommonCheckableButtons.inspectButton->setChecked(true);
            // show view options
            myNetworkViewOptions.menuCheckSelectEdges->show();
            myNetworkViewOptions.menuCheckShowConnections->show();
            myNetworkViewOptions.menuCheckShowAdditionalSubElements->show();
            myNetworkViewOptions.menuCheckShowTAZElements->show();
            // show menu checks
            menuChecks.menuCheckSelectEdges->show();
            menuChecks.menuCheckShowConnections->show();
            menuChecks.menuCheckShowAdditionalSubElements->show();
            menuChecks.menuCheckShowTAZElements->show();
            // update lock menu bar
            myLockManager.updateLockMenuBar();
            // show
            break;
        case NetworkEditMode::NETWORK_DELETE:
            myViewParent->getDeleteFrame()->show();
            myViewParent->getDeleteFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getDeleteFrame();
            myCommonCheckableButtons.deleteButton->setChecked(true);
            myNetworkViewOptions.menuCheckShowConnections->show();
            myNetworkViewOptions.menuCheckShowAdditionalSubElements->show();
            myNetworkViewOptions.menuCheckShowTAZElements->show();
            // show view options
            myNetworkViewOptions.menuCheckSelectEdges->show();
            myNetworkViewOptions.menuCheckShowConnections->show();
            menuChecks.menuCheckShowAdditionalSubElements->show();
            menuChecks.menuCheckShowTAZElements->show();
            // show menu checks
            menuChecks.menuCheckSelectEdges->show();
            menuChecks.menuCheckShowConnections->show();
            break;
        case NetworkEditMode::NETWORK_SELECT:
            myViewParent->getSelectorFrame()->show();
            myViewParent->getSelectorFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getSelectorFrame();
            myCommonCheckableButtons.selectButton->setChecked(true);
            // show view options
            myNetworkViewOptions.menuCheckSelectEdges->show();
            myNetworkViewOptions.menuCheckShowConnections->show();
            myNetworkViewOptions.menuCheckExtendSelection->show();
            myNetworkViewOptions.menuCheckShowAdditionalSubElements->show();
            myNetworkViewOptions.menuCheckShowTAZElements->show();
            // show menu checks
            menuChecks.menuCheckSelectEdges->show();
            menuChecks.menuCheckShowConnections->show();
            menuChecks.menuCheckExtendSelection->show();
            menuChecks.menuCheckShowAdditionalSubElements->show();
            menuChecks.menuCheckShowTAZElements->show();
            break;
        // specific modes
        case NetworkEditMode::NETWORK_CREATE_EDGE:
            myViewParent->getCreateEdgeFrame()->show();
            myViewParent->getCreateEdgeFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getCreateEdgeFrame();
            myNetworkCheckableButtons.createEdgeButton->setChecked(true);
            // show view options
            myNetworkViewOptions.menuCheckChainEdges->show();
            myNetworkViewOptions.menuCheckAutoOppositeEdge->show();
            // show menu checks
            menuChecks.menuCheckChainEdges->show();
            menuChecks.menuCheckAutoOppositeEdge->show();
            break;
        case NetworkEditMode::NETWORK_MOVE:
            myViewParent->getMoveFrame()->show();
            myViewParent->getMoveFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getMoveFrame();
            myNetworkCheckableButtons.moveNetworkElementsButton->setChecked(true);
            // show view options
            myNetworkViewOptions.menuCheckWarnAboutMerge->show();
            myNetworkViewOptions.menuCheckShowJunctionBubble->show();
            myNetworkViewOptions.menuCheckMoveElevation->show();
            // show menu checks
            menuChecks.menuCheckWarnAboutMerge->show();
            menuChecks.menuCheckShowJunctionBubble->show();
            menuChecks.menuCheckMoveElevation->show();
            break;
        case NetworkEditMode::NETWORK_CONNECT:
            myViewParent->getConnectorFrame()->show();
            myViewParent->getConnectorFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getConnectorFrame();
            myNetworkCheckableButtons.connectionButton->setChecked(true);
            break;
        case NetworkEditMode::NETWORK_TLS:
            myViewParent->getTLSEditorFrame()->show();
            myViewParent->getTLSEditorFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getTLSEditorFrame();
            myNetworkCheckableButtons.trafficLightButton->setChecked(true);
            // show view options
            myNetworkViewOptions.menuCheckChangeAllPhases->show();
            // show menu checks
            menuChecks.menuCheckChangeAllPhases->show();
            break;
        case NetworkEditMode::NETWORK_ADDITIONAL:
            myViewParent->getAdditionalFrame()->show();
            myViewParent->getAdditionalFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getAdditionalFrame();
            myNetworkCheckableButtons.additionalButton->setChecked(true);
            // show view options
            myNetworkViewOptions.menuCheckShowAdditionalSubElements->show();
            // show menu checks
            menuChecks.menuCheckShowAdditionalSubElements->show();
            break;
        case NetworkEditMode::NETWORK_CROSSING:
            myViewParent->getCrossingFrame()->show();
            myViewParent->getCrossingFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getCrossingFrame();
            myNetworkCheckableButtons.crossingButton->setChecked(true);
            break;
        case NetworkEditMode::NETWORK_TAZ:
            myViewParent->getTAZFrame()->show();
            myViewParent->getTAZFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getTAZFrame();
            myNetworkCheckableButtons.TAZButton->setChecked(true);
            break;
        case NetworkEditMode::NETWORK_SHAPE:
            myViewParent->getShapeFrame()->show();
            myViewParent->getShapeFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getShapeFrame();
            myNetworkCheckableButtons.shapeButton->setChecked(true);
            break;
        case NetworkEditMode::NETWORK_PROHIBITION:
            myViewParent->getProhibitionFrame()->show();
            myViewParent->getProhibitionFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getProhibitionFrame();
            myNetworkCheckableButtons.prohibitionButton->setChecked(true);
            break;
        case NetworkEditMode::NETWORK_WIRE:
            myViewParent->getWireFrame()->show();
            myViewParent->getWireFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getWireFrame();
            myNetworkCheckableButtons.wireButton->setChecked(true);
            break;
        case NetworkEditMode::NETWORK_DECAL:
            myViewParent->getDecalFrame()->show();
            myViewParent->getDecalFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getDecalFrame();
            myNetworkCheckableButtons.decalButton->setChecked(true);
            break;
        default:
            break;
    }
    // update menuChecks shorcuts
    menuChecks.updateShortcuts();
    // update common Network buttons
    myCommonCheckableButtons.updateCommonCheckableButtons();
    // Update Network buttons
    myNetworkCheckableButtons.updateNetworkCheckableButtons();
    // recalc toolbar
    myViewParent->getGNEAppWindows()->getToolbarsGrip().modes->recalc();
    myViewParent->getGNEAppWindows()->getToolbarsGrip().modes->repaint();
    // force repaint because different modes draw different things
    onPaint(nullptr, 0, nullptr);
    // finally update view
    updateViewNet();
}


void
GNEViewNet::updateDemandModeSpecificControls() {
    // get menu checks
    auto& menuChecks = myViewParent->getGNEAppWindows()->getEditMenuCommands().demandViewOptions;
    // hide all checkbox of view options Network
    myNetworkViewOptions.hideNetworkViewOptionsMenuChecks();
    // hide all checkbox of view options Demand
    myDemandViewOptions.hideDemandViewOptionsMenuChecks();
    // hide all checkbox of view options Data
    myDataViewOptions.hideDataViewOptionsMenuChecks();
    // disable all common edit modes
    myCommonCheckableButtons.disableCommonCheckableButtons();
    // disable all Demand edit modes
    myDemandCheckableButtons.disableDemandCheckableButtons();
    // disable all network edit modes
    myDataCheckableButtons.disableDataCheckableButtons();
    // hide interval bar
    myIntervalBar.hideIntervalBar();
    // hide all frames
    myViewParent->hideAllFrames();
    // hide all menuchecks
    myViewParent->getGNEAppWindows()->getEditMenuCommands().networkViewOptions.hideNetworkViewOptionsMenuChecks();
    myViewParent->getGNEAppWindows()->getEditMenuCommands().demandViewOptions.hideDemandViewOptionsMenuChecks();
    myViewParent->getGNEAppWindows()->getEditMenuCommands().dataViewOptions.hideDataViewOptionsMenuChecks();
    // always show "hide shapes", "show grid", "draw spread vehicles", "show overlapped routes" and show/lock persons and containers
    myDemandViewOptions.menuCheckToggleGrid->show();
    myDemandViewOptions.menuCheckToggleDrawJunctionShape->show();
    myDemandViewOptions.menuCheckDrawSpreadVehicles->show();
    myDemandViewOptions.menuCheckHideShapes->show();
    myDemandViewOptions.menuCheckShowAllTrips->show();
    myDemandViewOptions.menuCheckShowAllPersonPlans->show();
    myDemandViewOptions.menuCheckLockPerson->show();
    myDemandViewOptions.menuCheckShowAllContainerPlans->show();
    myDemandViewOptions.menuCheckLockContainer->show();
    myDemandViewOptions.menuCheckShowOverlappedRoutes->show();
    menuChecks.menuCheckToggleGrid->show();
    menuChecks.menuCheckToggleDrawJunctionShape->show();
    menuChecks.menuCheckDrawSpreadVehicles->show();
    menuChecks.menuCheckHideShapes->show();
    menuChecks.menuCheckShowAllTrips->show();
    menuChecks.menuCheckShowAllPersonPlans->show();
    menuChecks.menuCheckLockPerson->show();
    menuChecks.menuCheckShowAllContainerPlans->show();
    menuChecks.menuCheckLockContainer->show();
    menuChecks.menuCheckShowOverlappedRoutes->show();
    // show separator
    menuChecks.separator->show();
    // enable selected controls
    switch (myEditModes.demandEditMode) {
        // common modes
        case DemandEditMode::DEMAND_INSPECT:
            myViewParent->getInspectorFrame()->show();
            myViewParent->getInspectorFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getInspectorFrame();
            // set checkable button
            myCommonCheckableButtons.inspectButton->setChecked(true);
            // show view options
            myDemandViewOptions.menuCheckHideNonInspectedDemandElements->show();
            // show menu checks
            menuChecks.menuCheckHideNonInspectedDemandElements->show();
            break;
        case DemandEditMode::DEMAND_DELETE:
            myViewParent->getDeleteFrame()->show();
            myViewParent->getDeleteFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getDeleteFrame();
            // set checkable button
            myCommonCheckableButtons.deleteButton->setChecked(true);
            break;
        case DemandEditMode::DEMAND_SELECT:
            myViewParent->getSelectorFrame()->show();
            myViewParent->getSelectorFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getSelectorFrame();
            // set checkable button
            myCommonCheckableButtons.selectButton->setChecked(true);
            break;
        case DemandEditMode::DEMAND_MOVE:
            myViewParent->getMoveFrame()->show();
            myViewParent->getMoveFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getMoveFrame();
            // set checkable button
            myDemandCheckableButtons.moveDemandElementsButton->setChecked(true);
            break;
        // specific modes
        case DemandEditMode::DEMAND_ROUTE:
            myViewParent->getRouteFrame()->show();
            myViewParent->getRouteFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getRouteFrame();
            // set checkable button
            myDemandCheckableButtons.routeButton->setChecked(true);
            break;
        case DemandEditMode::DEMAND_ROUTEDISTRIBUTION:
            myViewParent->getRouteDistributionFrame()->show();
            myViewParent->getRouteDistributionFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getRouteDistributionFrame();
            // set checkable button
            myDemandCheckableButtons.routeDistributionButton->setChecked(true);
            break;
        case DemandEditMode::DEMAND_VEHICLE:
            myViewParent->getVehicleFrame()->show();
            myViewParent->getVehicleFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getVehicleFrame();
            // set checkable button
            myDemandCheckableButtons.vehicleButton->setChecked(true);
            break;
        case DemandEditMode::DEMAND_TYPE:
            myViewParent->getTypeFrame()->show();
            myViewParent->getTypeFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getTypeFrame();
            // set checkable button
            myDemandCheckableButtons.typeButton->setChecked(true);
            break;
        case DemandEditMode::DEMAND_TYPEDISTRIBUTION:
            myViewParent->getTypeDistributionFrame()->show();
            myViewParent->getTypeDistributionFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getTypeDistributionFrame();
            // set checkable button
            myDemandCheckableButtons.typeDistributionButton->setChecked(true);
            break;
        case DemandEditMode::DEMAND_STOP:
            myViewParent->getStopFrame()->show();
            myViewParent->getStopFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getStopFrame();
            // set checkable button
            myDemandCheckableButtons.stopButton->setChecked(true);
            break;
        case DemandEditMode::DEMAND_PERSON:
            myViewParent->getPersonFrame()->show();
            myViewParent->getPersonFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getPersonFrame();
            // set checkable button
            myDemandCheckableButtons.personButton->setChecked(true);
            break;
        case DemandEditMode::DEMAND_PERSONPLAN:
            myViewParent->getPersonPlanFrame()->show();
            myViewParent->getPersonPlanFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getPersonPlanFrame();
            // set checkable button
            myDemandCheckableButtons.personPlanButton->setChecked(true);
            break;
        case DemandEditMode::DEMAND_CONTAINER:
            myViewParent->getContainerFrame()->show();
            myViewParent->getContainerFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getContainerFrame();
            // set checkable button
            myDemandCheckableButtons.containerButton->setChecked(true);
            break;
        case DemandEditMode::DEMAND_CONTAINERPLAN:
            myViewParent->getContainerPlanFrame()->show();
            myViewParent->getContainerPlanFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getContainerPlanFrame();
            // set checkable button
            myDemandCheckableButtons.containerPlanButton->setChecked(true);
            break;
        default:
            break;
    }
    // update menuChecks shorcuts
    menuChecks.updateShortcuts();
    // update common Network buttons
    myCommonCheckableButtons.updateCommonCheckableButtons();
    // Update Demand buttons
    myDemandCheckableButtons.updateDemandCheckableButtons();
    // recalc toolbar
    myViewParent->getGNEAppWindows()->getToolbarsGrip().modes->recalc();
    myViewParent->getGNEAppWindows()->getToolbarsGrip().modes->repaint();
    // force repaint because different modes draw different things
    onPaint(nullptr, 0, nullptr);
    // finally update view
    updateViewNet();
}


void
GNEViewNet::updateDataModeSpecificControls() {
    // get menu checks
    auto& menuChecks = myViewParent->getGNEAppWindows()->getEditMenuCommands().dataViewOptions;
    // hide all checkbox of view options Network
    myNetworkViewOptions.hideNetworkViewOptionsMenuChecks();
    // hide all checkbox of view options Demand
    myDemandViewOptions.hideDemandViewOptionsMenuChecks();
    // hide all checkbox of view options Data
    myDataViewOptions.hideDataViewOptionsMenuChecks();
    // disable all common edit modes
    myCommonCheckableButtons.disableCommonCheckableButtons();
    // disable all Data edit modes
    myDataCheckableButtons.disableDataCheckableButtons();
    // show interval bar
    myIntervalBar.showIntervalBar();
    // hide all frames
    myViewParent->hideAllFrames();
    // hide all menuchecks
    myViewParent->getGNEAppWindows()->getEditMenuCommands().networkViewOptions.hideNetworkViewOptionsMenuChecks();
    myViewParent->getGNEAppWindows()->getEditMenuCommands().demandViewOptions.hideDemandViewOptionsMenuChecks();
    myViewParent->getGNEAppWindows()->getEditMenuCommands().dataViewOptions.hideDataViewOptionsMenuChecks();
    // In data mode, always show options for show elements
    myDataViewOptions.menuCheckToggleDrawJunctionShape->show();
    myDataViewOptions.menuCheckShowAdditionals->show();
    myDataViewOptions.menuCheckShowShapes->show();
    myDataViewOptions.menuCheckShowDemandElements->show();
    menuChecks.menuCheckToggleDrawJunctionShape->show();
    menuChecks.menuCheckShowAdditionals->show();
    menuChecks.menuCheckShowShapes->show();
    menuChecks.menuCheckShowDemandElements->show();
    // show separator
    menuChecks.separator->show();
    // enable selected controls
    switch (myEditModes.dataEditMode) {
        // common modes
        case DataEditMode::DATA_INSPECT:
            myViewParent->getInspectorFrame()->show();
            myViewParent->getInspectorFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getInspectorFrame();
            // set checkable button
            myCommonCheckableButtons.inspectButton->setChecked(true);
            // show view option
            myDataViewOptions.menuCheckToggleTAZRelDrawing->show();
            myDataViewOptions.menuCheckToggleTAZDrawFill->show();
            myDataViewOptions.menuCheckToggleTAZRelOnlyFrom->show();
            myDataViewOptions.menuCheckToggleTAZRelOnlyTo->show();
            // show menu check
            menuChecks.menuCheckToggleTAZRelDrawing->show();
            menuChecks.menuCheckToggleTAZDrawFill->show();
            menuChecks.menuCheckToggleTAZRelOnlyFrom->show();
            menuChecks.menuCheckToggleTAZRelOnlyTo->show();
            break;
        case DataEditMode::DATA_DELETE:
            myViewParent->getDeleteFrame()->show();
            myViewParent->getDeleteFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getDeleteFrame();
            // set checkable button
            myCommonCheckableButtons.deleteButton->setChecked(true);
            // show toggle TAZRel drawing view option
            myDataViewOptions.menuCheckToggleTAZRelDrawing->show();
            myDataViewOptions.menuCheckToggleTAZRelOnlyFrom->show();
            myDataViewOptions.menuCheckToggleTAZRelOnlyTo->show();
            // show toggle TAZRel drawing menu check
            menuChecks.menuCheckToggleTAZRelDrawing->show();
            menuChecks.menuCheckToggleTAZRelOnlyFrom->show();
            menuChecks.menuCheckToggleTAZRelOnlyTo->show();
            break;
        case DataEditMode::DATA_SELECT:
            myViewParent->getSelectorFrame()->show();
            myViewParent->getSelectorFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getSelectorFrame();
            // set checkable button
            myCommonCheckableButtons.selectButton->setChecked(true);
            // show toggle TAZRel drawing view option
            myDataViewOptions.menuCheckToggleTAZRelDrawing->show();
            myDataViewOptions.menuCheckToggleTAZRelOnlyFrom->show();
            myDataViewOptions.menuCheckToggleTAZRelOnlyTo->show();
            // show toggle TAZRel drawing menu check
            menuChecks.menuCheckToggleTAZRelDrawing->show();
            menuChecks.menuCheckToggleTAZRelOnlyFrom->show();
            menuChecks.menuCheckToggleTAZRelOnlyTo->show();
            break;
        case DataEditMode::DATA_EDGEDATA:
            myViewParent->getEdgeDataFrame()->show();
            myViewParent->getEdgeDataFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getEdgeDataFrame();
            // set checkable button
            myDataCheckableButtons.edgeDataButton->setChecked(true);
            break;
        case DataEditMode::DATA_EDGERELDATA:
            myViewParent->getEdgeRelDataFrame()->show();
            myViewParent->getEdgeRelDataFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getEdgeRelDataFrame();
            // set checkable button
            myDataCheckableButtons.edgeRelDataButton->setChecked(true);
            break;
        case DataEditMode::DATA_TAZRELDATA:
            myViewParent->getTAZRelDataFrame()->show();
            myViewParent->getTAZRelDataFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getTAZRelDataFrame();
            // set checkable button
            myDataCheckableButtons.TAZRelDataButton->setChecked(true);
            // show view option
            myDataViewOptions.menuCheckToggleTAZRelDrawing->show();
            myDataViewOptions.menuCheckToggleTAZDrawFill->show();
            myDataViewOptions.menuCheckToggleTAZRelOnlyFrom->show();
            myDataViewOptions.menuCheckToggleTAZRelOnlyTo->show();
            // show menu check
            menuChecks.menuCheckToggleTAZRelDrawing->show();
            menuChecks.menuCheckToggleTAZDrawFill->show();
            menuChecks.menuCheckToggleTAZRelOnlyFrom->show();
            menuChecks.menuCheckToggleTAZRelOnlyTo->show();
            break;
        case DataEditMode::DATA_MEANDATA:
            myViewParent->getMeanDataFrame()->show();
            myViewParent->getMeanDataFrame()->focusUpperElement();
            myCurrentFrame = myViewParent->getMeanDataFrame();
            // set checkable button
            myDataCheckableButtons.meanDataButton->setChecked(true);
            break;
        default:
            break;
    }
    // update menuChecks shorcuts
    menuChecks.updateShortcuts();
    // update common Network buttons
    myCommonCheckableButtons.updateCommonCheckableButtons();
    // Update Data buttons
    myDataCheckableButtons.updateDataCheckableButtons();
    // recalc toolbar
    myViewParent->getGNEAppWindows()->getToolbarsGrip().modes->recalc();
    myViewParent->getGNEAppWindows()->getToolbarsGrip().modes->repaint();
    // force repaint because different modes draw different things
    onPaint(nullptr, 0, nullptr);
    // finally update view
    updateViewNet();
}


void
GNEViewNet::deleteNetworkAttributeCarriers(const std::vector<GNEAttributeCarrier*> ACs) {
    // iterate over ACs and delete it
    for (const auto& AC : ACs) {
        if (AC->getTagProperty().getTag() == SUMO_TAG_JUNCTION) {
            // get junction (note: could be already removed if is a child, then hardfail=false)
            GNEJunction* junction = myNet->getAttributeCarriers()->retrieveJunction(AC->getID(), false);
            // if exist, remove it
            if (junction) {
                myNet->deleteJunction(junction, myUndoList);
            }
        } else if (AC->getTagProperty().getTag() == SUMO_TAG_CROSSING) {
            // get crossing (note: could be already removed if is a child, then hardfail=false)
            GNECrossing* crossing = myNet->getAttributeCarriers()->retrieveCrossing(AC->getGUIGlObject(), false);
            // if exist, remove it
            if (crossing) {
                myNet->deleteCrossing(crossing, myUndoList);
            }
        } else if (AC->getTagProperty().getTag() == SUMO_TAG_EDGE) {
            // get edge (note: could be already removed if is a child, then hardfail=false)
            GNEEdge* edge = myNet->getAttributeCarriers()->retrieveEdge(AC->getID(), false);
            // if exist, remove it
            if (edge) {
                myNet->deleteEdge(edge, myUndoList, false);
            }
        } else if (AC->getTagProperty().getTag() == SUMO_TAG_LANE) {
            // get lane (note: could be already removed if is a child, then hardfail=false)
            GNELane* lane = myNet->getAttributeCarriers()->retrieveLane(AC->getGUIGlObject(), false);
            // if exist, remove it
            if (lane) {
                myNet->deleteLane(lane, myUndoList, false);
            }
        } else if (AC->getTagProperty().getTag() == SUMO_TAG_CONNECTION) {
            // get connection (note: could be already removed if is a child, then hardfail=false)
            GNEConnection* connection = myNet->getAttributeCarriers()->retrieveConnection(AC->getGUIGlObject(), false);
            // if exist, remove it
            if (connection) {
                myNet->deleteConnection(connection, myUndoList);
            }
        } else if (AC->getTagProperty().isAdditionalElement()) {
            // get additional Element (note: could be already removed if is a child, then hardfail=false)
            GNEAdditional* additionalElement = myNet->getAttributeCarriers()->retrieveAdditional(AC->getGUIGlObject(), false);
            // if exist, remove it
            if (additionalElement) {
                myNet->deleteAdditional(additionalElement, myUndoList);
            }
        }
    }
}


void
GNEViewNet::deleteDemandAttributeCarriers(const std::vector<GNEAttributeCarrier*> ACs) {
    // iterate over ACs and delete it
    for (const auto& AC : ACs) {
        // get demand Element (note: could be already removed if is a child, then hardfail=false)
        GNEDemandElement* demandElement = myNet->getAttributeCarriers()->retrieveDemandElement(AC->getGUIGlObject(), false);
        // if exist, remove it
        if (demandElement) {
            myNet->deleteDemandElement(demandElement, myUndoList);
        }
    }
}


void
GNEViewNet::deleteDataAttributeCarriers(const std::vector<GNEAttributeCarrier*> ACs) {
    // iterate over ACs and delete it
    for (const auto& AC : ACs) {
        if (AC->getTagProperty().getTag() == SUMO_TAG_DATASET) {
            // get data set (note: could be already removed if is a child, then hardfail=false)
            GNEDataSet* dataSet = myNet->getAttributeCarriers()->retrieveDataSet(AC->getID(), false);
            // if exist, remove it
            if (dataSet) {
                myNet->deleteDataSet(dataSet, myUndoList);
            }
        } else if (AC->getTagProperty().getTag() == SUMO_TAG_DATAINTERVAL) {
            // get data interval (note: could be already removed if is a child, then hardfail=false)
            GNEDataInterval* dataInterval = myNet->getAttributeCarriers()->retrieveDataInterval(AC, false);
            // if exist, remove it
            if (dataInterval) {
                myNet->deleteDataInterval(dataInterval, myUndoList);
            }
        } else {
            // get generic data (note: could be already removed if is a child, then hardfail=false)
            GNEGenericData* genericData = myNet->getAttributeCarriers()->retrieveGenericData(AC->getGUIGlObject(), false);
            // if exist, remove it
            if (genericData) {
                myNet->deleteGenericData(genericData, myUndoList);
            }
        }
    }
}


void
GNEViewNet::updateControls() {
    if (myEditModes.isCurrentSupermodeNetwork()) {
        switch (myEditModes.networkEditMode) {
            case NetworkEditMode::NETWORK_INSPECT:
                myViewParent->getInspectorFrame()->update();
                break;
            default:
                break;
        }
    }
    if (myEditModes.isCurrentSupermodeDemand()) {
        switch (myEditModes.demandEditMode) {
            case DemandEditMode::DEMAND_INSPECT:
                myViewParent->getInspectorFrame()->update();
                break;
            case DemandEditMode::DEMAND_VEHICLE:
                myViewParent->getVehicleFrame()->show();
                break;
            case DemandEditMode::DEMAND_TYPE:
                myViewParent->getTypeFrame()->show();
                break;
            case DemandEditMode::DEMAND_TYPEDISTRIBUTION:
                myViewParent->getTypeDistributionFrame()->getDistributionSelector()->refreshDistributionSelector();
                break;
            case DemandEditMode::DEMAND_ROUTEDISTRIBUTION:
                myViewParent->getRouteDistributionFrame()->getDistributionSelector()->refreshDistributionSelector();
                break;
            case DemandEditMode::DEMAND_PERSON:
                myViewParent->getPersonFrame()->show();
                break;
            case DemandEditMode::DEMAND_PERSONPLAN:
                myViewParent->getPersonPlanFrame()->show();
                break;
            case DemandEditMode::DEMAND_CONTAINER:
                myViewParent->getContainerFrame()->show();
                break;
            case DemandEditMode::DEMAND_CONTAINERPLAN:
                myViewParent->getContainerPlanFrame()->show();
                break;
            case DemandEditMode::DEMAND_STOP:
                myViewParent->getStopFrame()->show();
                break;
            default:
                break;
        }
    }
    if (myEditModes.isCurrentSupermodeData()) {
        switch (myEditModes.dataEditMode) {
            case DataEditMode::DATA_INSPECT:
                myViewParent->getInspectorFrame()->update();
                break;
            default:
                break;
        }
        // update data interval
        myIntervalBar.markForUpdate();
    }
    // update view
    updateViewNet();
}

// ---------------------------------------------------------------------------
// Private methods
// ---------------------------------------------------------------------------

void
GNEViewNet::drawTemporalDrawingShape() const {
    PositionVector temporalShape;
    bool deleteLastCreatedPoint = false;
    // obtain temporal shape and delete last created point flag
    if (myViewParent->getShapeFrame()->getDrawingShapeModule()->isDrawing()) {
        temporalShape = myViewParent->getShapeFrame()->getDrawingShapeModule()->getTemporalShape();
        deleteLastCreatedPoint = myViewParent->getShapeFrame()->getDrawingShapeModule()->getDeleteLastCreatedPoint();
    } else if (myViewParent->getTAZFrame()->getDrawingShapeModule()->isDrawing()) {
        temporalShape = myViewParent->getTAZFrame()->getDrawingShapeModule()->getTemporalShape();
        deleteLastCreatedPoint = myViewParent->getTAZFrame()->getDrawingShapeModule()->getDeleteLastCreatedPoint();
    }
    // check if we're in drawing mode
    if (temporalShape.size() > 0) {
        // draw blue line with the current drawed shape
        GLHelper::pushMatrix();
        glLineWidth(2);
        glTranslated(0, 0, GLO_TEMPORALSHAPE);
        GLHelper::setColor(RGBColor::BLUE);
        GLHelper::drawLine(temporalShape);
        GLHelper::popMatrix();
        // draw red line from the last point of shape to the current mouse position
        GLHelper::pushMatrix();
        glLineWidth(2);
        glTranslated(0, 0, GLO_TEMPORALSHAPE);
        // draw last line depending if shift key (delete last created point) is pressed
        if (deleteLastCreatedPoint) {
            GLHelper::setColor(RGBColor::RED);
        } else {
            GLHelper::setColor(RGBColor::GREEN);
        }
        GLHelper::drawLine(temporalShape.back(), snapToActiveGrid(getPositionInformation()));
        GLHelper::popMatrix();
    }
}


void
GNEViewNet::drawTemporalJunction() const {
    // first check if we're in correct mode
    if (myEditModes.isCurrentSupermodeNetwork() && (myEditModes.networkEditMode == NetworkEditMode::NETWORK_CREATE_EDGE) &&
            !myMouseButtonKeyPressed.controlKeyPressed() &&
            !myMouseButtonKeyPressed.shiftKeyPressed() &&
            !myMouseButtonKeyPressed.altKeyPressed()) {
        // get mouse position
        const Position mousePosition = snapToActiveGrid(getPositionInformation());
        // get junction exaggeration
        const double junctionExaggeration = myVisualizationSettings->junctionSize.getExaggeration(*myVisualizationSettings, nullptr, 4);
        // get bubble color
        RGBColor bubbleColor = myVisualizationSettings->junctionColorer.getScheme().getColor(1);
        // change alpha
        bubbleColor.setAlpha(200);
        // push layer matrix
        GLHelper::pushMatrix();
        // translate to temporal shape layer
        glTranslated(0, 0, GLO_TEMPORALSHAPE);
        // push junction matrix
        GLHelper::pushMatrix();
        // move matrix junction center
        glTranslated(mousePosition.x(), mousePosition.y(), 0.1);
        // set color
        GLHelper::setColor(bubbleColor);
        // draw outline circle
        const double circleWidth = myVisualizationSettings->neteditSizeSettings.junctionBubbleRadius * junctionExaggeration;
        GLHelper::drawOutlineCircle(circleWidth, circleWidth * 0.75, 32);
        // pop junction matrix
        GLHelper::popMatrix();
        // draw temporal edge
        if (myViewParent->getCreateEdgeFrame()->getJunctionSource() && (mousePosition.distanceSquaredTo(myViewParent->getCreateEdgeFrame()->getJunctionSource()->getPositionInView()) > 1)) {
            // set temporal edge color
            RGBColor temporalEdgeColor = RGBColor::BLACK;
            temporalEdgeColor.setAlpha(200);
            // declare temporal edge geometry
            GUIGeometry temporalEdgeGeometry;
            // calculate geometry between source junction and mouse position
            PositionVector temporalEdge = {mousePosition, myViewParent->getCreateEdgeFrame()->getJunctionSource()->getPositionInView()};
            // move temporal edge 2 side
            temporalEdge.move2side(-1);
            // update geometry
            temporalEdgeGeometry.updateGeometry(temporalEdge);
            // push temporal edge matrix
            GLHelper::pushMatrix();
            // set color
            GLHelper::setColor(temporalEdgeColor);
            // draw temporal edge
            GUIGeometry::drawGeometry(GUIVisualizationSettings::Detail::Level0, temporalEdgeGeometry, 0.75);
            // check if we have to draw opposite edge
            if (myNetworkViewOptions.menuCheckAutoOppositeEdge->amChecked() == TRUE) {
                // move temporal edge to opposite edge
                temporalEdge.move2side(2);
                // update geometry
                temporalEdgeGeometry.updateGeometry(temporalEdge);
                // draw temporal edge
                GUIGeometry::drawGeometry(GUIVisualizationSettings::Detail::Level0, temporalEdgeGeometry, 0.75);
            }
            // pop temporal edge matrix
            GLHelper::popMatrix();
        }
        // pop layer matrix
        GLHelper::popMatrix();
    }
}


void
GNEViewNet::drawTemporalSplitJunction() const {
    // first check if we're in correct mode
    if (myEditModes.isCurrentSupermodeNetwork() && (myEditModes.networkEditMode == NetworkEditMode::NETWORK_CREATE_EDGE) &&
            !myMouseButtonKeyPressed.controlKeyPressed() &&
            myMouseButtonKeyPressed.shiftKeyPressed() &&
            !myMouseButtonKeyPressed.altKeyPressed() &&
            (gViewObjectsHandler.markedEdge != nullptr)) {
        // calculate split position
        const auto lane = gViewObjectsHandler.markedEdge->getLanes().back();
        auto shape = lane->getLaneShape();
        // move shape to side
        shape.move2side(lane->getDrawingConstants()->getDrawingWidth() * -1);
        const auto offset = shape.nearest_offset_to_point2D(snapToActiveGrid(getPositionInformation()));
        const auto splitPosition = shape.positionAtOffset2D(offset);
        // get junction exaggeration
        const double junctionExaggeration = myVisualizationSettings->junctionSize.getExaggeration(*myVisualizationSettings, nullptr, 4);
        // get bubble color
        RGBColor bubbleColor = myVisualizationSettings->junctionColorer.getScheme().getColor(1);
        // push layer matrix
        GLHelper::pushMatrix();
        // translate to temporal shape layer
        glTranslated(0, 0, GLO_TEMPORALSHAPE);
        // push junction matrix
        GLHelper::pushMatrix();
        // move matrix junction center
        glTranslated(splitPosition.x(), splitPosition.y(), 0.1);
        // set color
        GLHelper::setColor(bubbleColor);
        // draw outline circle
        const double circleWidth = myVisualizationSettings->neteditSizeSettings.junctionBubbleRadius * junctionExaggeration;
        GLHelper::drawOutlineCircle(circleWidth, circleWidth * 0.75, 32);
        // draw filled circle
        GLHelper::drawFilledCircle(0.5, 32);
        // pop junction matrix
        GLHelper::popMatrix();
        // pop layer matrix
        GLHelper::popMatrix();
    }
}


void
GNEViewNet::drawTemporalRoundabout() const {
    // check conditions
    if ((myCurrentObjectsDialog.size() > 0) && (myCurrentObjectsDialog.front()->getType() == GLO_JUNCTION) && myDrawPreviewRoundabout) {
        // get junction
        const auto junction = myNet->getAttributeCarriers()->retrieveJunction(myCurrentObjectsDialog.front()->getMicrosimID());
        // push layer matrix
        GLHelper::pushMatrix();
        // translate to temporal shape layer
        glTranslated(0, 0, GLO_TEMPORALSHAPE);
        // push junction matrix
        GLHelper::pushMatrix();
        // move matrix junction center
        glTranslated(junction->getNBNode()->getPosition().x(), junction->getNBNode()->getPosition().y(), 0.1);
        // set color
        GLHelper::setColor(RGBColor::GREEN);
        // draw outline circle
        const double circleWidth = (junction->getNBNode()->getRadius() < 0) ? 4.0 : junction->getNBNode()->getRadius();
        GLHelper::drawOutlineCircle(circleWidth * 1.30, circleWidth, 32);
        // pop junction matrix
        GLHelper::popMatrix();
        // pop layer matrix
        GLHelper::popMatrix();
    }
}


void
GNEViewNet::drawTemporalE1TLSLines() const {
    // check conditions
    if (selectingDetectorsTLSMode() && (myViewParent->getTLSEditorFrame()->getTLSJunction()->getCurrentJunction() != nullptr)) {
        // get junction
        const auto junctionPos = myViewParent->getTLSEditorFrame()->getTLSJunction()->getCurrentJunction()->getNBNode()->getPosition();
        // push layer matrix
        GLHelper::pushMatrix();
        // translate to TLLogic
        glTranslated(0, 0, GLO_TEMPORALSHAPE);
        // iterate over all E1 detectors
        for (const auto& E1ID : myViewParent->getTLSEditorFrame()->getTLSAttributes()->getE1Detectors()) {
            // first check if E1 exists
            const auto E1 = myNet->getAttributeCarriers()->retrieveAdditional(SUMO_TAG_INDUCTION_LOOP, E1ID.second, false);
            if (E1) {
                // push line matrix
                GLHelper::pushMatrix();
                // draw line between junction and E1
                GUIGeometry::drawChildLine(*myVisualizationSettings, junctionPos, E1->getPositionInView(),
                                           myVisualizationSettings->additionalSettings.TLSConnectionColor, true, 0.25);
                // pop line matrix
                GLHelper::popMatrix();
            }
        }
        // pop layer matrix
        GLHelper::popMatrix();
    }
}


void
GNEViewNet::drawTemporalJunctionTLSLines() const {
    // check conditions
    if (selectingJunctionsTLSMode() && (myViewParent->getTLSEditorFrame()->getTLSJunction()->getCurrentJunction() != nullptr)) {
        // get junction
        const auto junctionPos = myViewParent->getTLSEditorFrame()->getTLSJunction()->getCurrentJunction()->getNBNode()->getPosition();
        // push layer matrix
        GLHelper::pushMatrix();
        // translate to TLLogic
        glTranslated(0, 0, GLO_TEMPORALSHAPE);
        // iterate over all Junction detectors
        for (const auto& selectedJunctionID : myViewParent->getTLSEditorFrame()->getTLSJunction()->getSelectedJunctionIDs()) {
            // get junction
            const auto selectedJunction = myNet->getAttributeCarriers()->retrieveJunction(selectedJunctionID);
            // push line matrix
            GLHelper::pushMatrix();
            // draw line between junction and Junction
            GUIGeometry::drawChildLine(*myVisualizationSettings, junctionPos, selectedJunction->getPositionInView(),
                                       myVisualizationSettings->additionalSettings.TLSConnectionColor, true, 0.25);
            // pop line matrix
            GLHelper::popMatrix();
        }
        // pop layer matrix
        GLHelper::popMatrix();
    }
}


void
GNEViewNet::drawNeteditAttributesReferences() {
    if (myEditModes.isCurrentSupermodeNetwork() && (myEditModes.networkEditMode == NetworkEditMode::NETWORK_ADDITIONAL)) {
        myViewParent->getAdditionalFrame()->getNeteditAttributes()->drawLaneReference(gViewObjectsHandler.markedLane);
    }
}


void
GNEViewNet::drawTestsCircle() const {
    if (myVisualizationSettings->showGrid && OptionsCont::getOptions().getBool("gui-testing")) {
        // get mouse position
        const Position mousePosition = snapToActiveGrid(getPositionInformation());
        // push layer matrix
        GLHelper::pushMatrix();
        // translate to test layer, but under magenta square
        glTranslated(mousePosition.x(), mousePosition.y(), GLO_TESTELEMENT - 1);
        // set color
        GLHelper::setColor(RGBColor::ORANGE);
        // draw circle
        GLHelper::drawFilledCircle(1, 8);
        // pop layer matrix
        GLHelper::popMatrix();
    }
}


void
GNEViewNet::processLeftButtonPressNetwork(void* eventData) {
    // reset moving selected edge
    myMoveMultipleElements.resetMovingSelectedEdge();
    // decide what to do based on mode
    switch (myEditModes.networkEditMode) {
        case NetworkEditMode::NETWORK_INSPECT: {
            // first swap lane to edges if mySelectEdges is enabled and shift key isn't pressed
            if (checkSelectEdges()) {
                myViewObjectsSelector.filterLanes();
            } else {
                myViewObjectsSelector.filterEdges();
            }
            // now filter locked elements
            myViewObjectsSelector.filterLockedElements();
            // check if we're selecting a new parent for the current inspected element
            if (myViewParent->getInspectorFrame()->getNeteditAttributesEditor()->isSelectingParent()) {
                myViewParent->getInspectorFrame()->getNeteditAttributesEditor()->setNewParent(myViewObjectsSelector.getAttributeCarrierFront());
            } else {
                // process left click in Inspector Frame
                myViewParent->getInspectorFrame()->processNetworkSupermodeClick(getPositionInformation(), myViewObjectsSelector);
            }
            // process click
            processClick(eventData);
            break;
        }
        case NetworkEditMode::NETWORK_DELETE: {
            // first swap lane to edges if mySelectEdges is enabled and shift key isn't pressed
            if (checkSelectEdges()) {
                myViewObjectsSelector.filterLanes();
            } else {
                myViewObjectsSelector.filterEdges();
            }
            // now filter locked elements forcing excluding walkingAreas
            myViewObjectsSelector.filterLockedElements({GLO_WALKINGAREA});
            // continue depending of AC
            if (myViewObjectsSelector.getAttributeCarrierFront()) {
                // now check if we want only delete geometry points
                if (myViewParent->getDeleteFrame()->getDeleteOptions()->deleteOnlyGeometryPoints()) {
                    // only remove geometry point
                    myViewParent->getDeleteFrame()->removeGeometryPoint(myViewObjectsSelector);
                } else if (myViewObjectsSelector.getAttributeCarrierFront()->isAttributeCarrierSelected()) {
                    // remove all selected attribute carriers
                    if (!myViewObjectsSelector.getAttributeCarrierFront()->getGUIGlObject()->isGLObjectLocked()) {
                        myViewParent->getDeleteFrame()->removeSelectedAttributeCarriers();
                    }
                } else {
                    // remove attribute carrier under cursor
                    myViewParent->getDeleteFrame()->removeAttributeCarrier(myViewObjectsSelector);
                }
            } else {
                // process click
                processClick(eventData);
            }
            break;
        }
        case NetworkEditMode::NETWORK_SELECT:
            // first swap lane to edges if mySelectEdges is enabled and shift key isn't pressed
            if (checkSelectEdges()) {
                myViewObjectsSelector.filterLanes();
            } else {
                myViewObjectsSelector.filterEdges();
            }
            // now filter locked elements
            myViewObjectsSelector.filterLockedElements();
            // avoid to select if control key is pressed
            if (!myMouseButtonKeyPressed.controlKeyPressed()) {
                // check if a rect for selecting is being created
                if (myMouseButtonKeyPressed.shiftKeyPressed()) {
                    // begin rectangle selection
                    mySelectingArea.beginRectangleSelection();
                } else if (!myViewParent->getSelectorFrame()->selectAttributeCarrier(myViewObjectsSelector)) {
                    // process click
                    processClick(eventData);
                }
            } else {
                // process click
                processClick(eventData);
            }
            break;
        case NetworkEditMode::NETWORK_CREATE_EDGE: {
            // check what buttons are pressed
            if (myMouseButtonKeyPressed.shiftKeyPressed()) {
                // get edge under cursor
                GNEEdge* edge = myViewObjectsSelector.getEdgeFront();
                if (edge) {
                    // obtain reverse edge
                    const auto oppositeEdges = edge->getOppositeEdges();
                    // check if we're split one or both edges
                    if (myMouseButtonKeyPressed.altKeyPressed()) {
                        myNet->splitEdge(edge, edge->getSplitPos(getPositionInformation()), myUndoList);
                    } else if (oppositeEdges.size() > 0) {
                        myNet->splitEdgesBidi(edge, oppositeEdges.front(), edge->getSplitPos(getPositionInformation()), myUndoList);
                    } else {
                        myNet->splitEdge(edge, edge->getSplitPos(getPositionInformation()), myUndoList);
                    }
                }
            } else if (!myMouseButtonKeyPressed.controlKeyPressed()) {
                // process left click in create edge frame Frame
                myViewParent->getCreateEdgeFrame()->processClick(getPositionInformation(),
                        myViewObjectsSelector,
                        (myNetworkViewOptions.menuCheckAutoOppositeEdge->amChecked() == TRUE),
                        (myNetworkViewOptions.menuCheckChainEdges->amChecked() == TRUE));
            }
            // process click
            processClick(eventData);
            break;
        }
        case NetworkEditMode::NETWORK_MOVE: {
            // editing lane shapes in move mode isn't finished, then always filter lanes
            myViewObjectsSelector.filterLanes();
            // filter locked elements
            myViewObjectsSelector.filterLockedElements({GLO_WALKINGAREA});
            // check if we're editing a shape
            if (myEditNetworkElementShapes.getEditedNetworkElement()) {
                // check if we're removing a geometry point
                if (myMouseButtonKeyPressed.shiftKeyPressed()) {
                    // remove geometry point
                    if (myViewObjectsSelector.getNetworkElementFront() == myEditNetworkElementShapes.getEditedNetworkElement()) {
                        myViewObjectsSelector.getNetworkElementFront()->removeGeometryPoint(getPositionInformation(), myUndoList);
                    }
                } else if (!myMoveSingleElement.beginMoveNetworkElementShape()) {
                    // process click  if there isn't movable elements (to move camera using drag an drop)
                    processClick(eventData);
                }
            } else {
                // get AC under cursor
                auto AC = myViewObjectsSelector.getAttributeCarrierFront();
                // check that AC is an network or additional element
                if (AC && (AC->getTagProperty().isNetworkElement() || AC->getTagProperty().isAdditionalElement())) {
                    // check if we're moving a set of selected items
                    if (AC->isAttributeCarrierSelected()) {
                        // move selected ACs
                        myMoveMultipleElements.beginMoveSelection();
                        // update view
                        updateViewNet();
                    } else if (!myMoveSingleElement.beginMoveSingleElementNetworkMode()) {
                        // process click  if there isn't movable elements (to move camera using drag an drop)
                        processClick(eventData);
                    }
                } else {
                    // process click  if there isn't movable elements (to move camera using drag an drop)
                    processClick(eventData);
                }
            }
            break;
        }
        case NetworkEditMode::NETWORK_CONNECT: {
            // check if we're clicked over a non locked lane
            if (myViewObjectsSelector.getLaneFrontNonLocked()) {
                // Handle laneclick (shift key may pass connections, Control key allow conflicts)
                myViewParent->getConnectorFrame()->handleLaneClick(myViewObjectsSelector);
                updateViewNet();
            }
            // process click
            processClick(eventData);
            break;
        }
        case NetworkEditMode::NETWORK_TLS: {
            if (myViewObjectsSelector.getJunctionFront() || myViewObjectsSelector.getAdditionalFront()) {
                // edit TLS in TLSEditor frame
                myViewParent->getTLSEditorFrame()->editTLS(getPositionInformation(), myViewObjectsSelector);
                updateViewNet();
            }
            // process click
            processClick(eventData);
            break;
        }
        case NetworkEditMode::NETWORK_ADDITIONAL: {
            // avoid create additionals if control key is pressed
            if (!myMouseButtonKeyPressed.controlKeyPressed()) {
                if ((getPositionInformation() == myLastClickedPosition) && !myMouseButtonKeyPressed.shiftKeyPressed()) {
                    WRITE_WARNING(TL("Shift + click to create two additionals in the same position"));
                } else if (myViewParent->getAdditionalFrame()->addAdditional(myViewObjectsSelector)) {
                    // save last mouse position
                    myLastClickedPosition = getPositionInformation();
                    // update view to show the new additional
                    updateViewNet();
                }
            }
            // process click
            processClick(eventData);
            break;
        }
        case NetworkEditMode::NETWORK_CROSSING: {
            // call function addCrossing from crossing frame
            myViewParent->getCrossingFrame()->addCrossing(myViewObjectsSelector);
            // process click
            processClick(eventData);
            break;
        }
        case NetworkEditMode::NETWORK_TAZ: {
            // avoid create TAZs if control key is pressed
            if (!myMouseButtonKeyPressed.controlKeyPressed()) {
                // check if we want to create a rect for selecting edges
                if (myMouseButtonKeyPressed.shiftKeyPressed() && (myViewParent->getTAZFrame()->getCurrentTAZModule()->getTAZ() != nullptr)) {
                    // begin rectangle selection
                    mySelectingArea.beginRectangleSelection();
                } else {
                    // check if process click was successfully
                    if (myViewParent->getTAZFrame()->processClick(snapToActiveGrid(getPositionInformation()), myViewObjectsSelector)) {
                        // view net must be always update
                        updateViewNet();
                    }
                    // process click
                    processClick(eventData);
                }
            } else {
                // process click
                processClick(eventData);
            }
            break;
        }
        case NetworkEditMode::NETWORK_SHAPE: {
            // avoid create shapes if control key is pressed
            if (!myMouseButtonKeyPressed.controlKeyPressed()) {
                if (!myViewObjectsSelector.getPOIFront()) {
                    // declare processClick flag
                    bool updateTemporalShape = false;
                    // process click
                    myViewParent->getShapeFrame()->processClick(snapToActiveGrid(getPositionInformation()), myViewObjectsSelector, updateTemporalShape);
                    // view net must be always update
                    updateViewNet();
                    // process click depending of the result of "process click"
                    if (!updateTemporalShape) {
                        // process click
                        processClick(eventData);
                    }
                }
            } else {
                // process click
                processClick(eventData);
            }
            break;
        }
        case NetworkEditMode::NETWORK_PROHIBITION: {
            if (myViewObjectsSelector.getConnectionFront()) {
                // shift key may pass connections, Control key allow conflicts.
                myViewParent->getProhibitionFrame()->handleProhibitionClick(myViewObjectsSelector);
                updateViewNet();
            }
            // process click
            processClick(eventData);
            break;
        }
        case NetworkEditMode::NETWORK_WIRE: {
            // avoid create wires if control key is pressed
            if (!myMouseButtonKeyPressed.controlKeyPressed()) {
                myViewParent->getWireFrame()->addWire(myViewObjectsSelector);
                // update view to show the new wire
                updateViewNet();
            }
            // process click
            processClick(eventData);
            break;
        }
        case NetworkEditMode::NETWORK_DECAL: {
            // process click
            processClick(eventData);
            break;
        }
        default: {
            // process click
            processClick(eventData);
        }
    }
}


void
GNEViewNet::processLeftButtonReleaseNetwork() {
    // check moved items
    if (myMoveMultipleElements.isMovingSelection()) {
        myMoveMultipleElements.finishMoveSelection();
    } else if (mySelectingArea.selectingUsingRectangle) {
        // check if we're creating a rectangle selection or we want only to select a lane
        if (mySelectingArea.startDrawing) {
            // check if we're selecting all type of elements o we only want a set of edges for TAZ
            if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_SELECT) {
                mySelectingArea.processRectangleSelection();
            } else if (myEditModes.networkEditMode == NetworkEditMode::NETWORK_TAZ) {
                // process edge selection
                myViewParent->getTAZFrame()->processEdgeSelection(mySelectingArea.processEdgeRectangleSelection());
            }
        } else if (myMouseButtonKeyPressed.shiftKeyPressed()) {
            // check if there is a lane in objects under cursor
            if (myViewObjectsSelector.getLaneFront()) {
                // if we clicked over an lane with shift key pressed, select or unselect it
                if (myViewObjectsSelector.getLaneFront()->isAttributeCarrierSelected()) {
                    myViewObjectsSelector.getLaneFront()->unselectAttributeCarrier();
                } else {
                    myViewObjectsSelector.getLaneFront()->selectAttributeCarrier();
                }
            }
        }
        // finish selection
        mySelectingArea.finishRectangleSelection();
    } else {
        // finish moving of single elements
        myMoveSingleElement.finishMoveSingleElement();
    }
}


void
GNEViewNet::processMoveMouseNetwork(const bool mouseLeftButtonPressed) {
    // change "delete last created point" depending if during movement shift key is pressed
    if ((myEditModes.networkEditMode == NetworkEditMode::NETWORK_SHAPE) && myViewParent->getShapeFrame()->getDrawingShapeModule()->isDrawing()) {
        myViewParent->getShapeFrame()->getDrawingShapeModule()->setDeleteLastCreatedPoint(myMouseButtonKeyPressed.shiftKeyPressed());
    } else if ((myEditModes.networkEditMode == NetworkEditMode::NETWORK_TAZ) && myViewParent->getTAZFrame()->getDrawingShapeModule()->isDrawing()) {
        myViewParent->getTAZFrame()->getDrawingShapeModule()->setDeleteLastCreatedPoint(myMouseButtonKeyPressed.shiftKeyPressed());
    }
    // check what type of additional is moved
    if (myMoveMultipleElements.isMovingSelection()) {
        // move entire selection
        myMoveMultipleElements.moveSelection(mouseLeftButtonPressed);
    } else if (mySelectingArea.selectingUsingRectangle) {
        // update selection corner of selecting area
        mySelectingArea.moveRectangleSelection();
    } else {
        // move single elements
        myMoveSingleElement.moveSingleElement(mouseLeftButtonPressed);
    }
}


void
GNEViewNet::processLeftButtonPressDemand(void* eventData) {
    // filter shapes (because POIs and polygons doesn't interact in demand mode)
    myViewObjectsSelector.filterShapes();
    // decide what to do based on mode
    switch (myEditModes.demandEditMode) {
        case DemandEditMode::DEMAND_INSPECT: {
            // filter locked elements
            myViewObjectsSelector.filterLockedElements();
            // process left click in Inspector Frame
            myViewParent->getInspectorFrame()->processDemandSupermodeClick(getPositionInformation(), myViewObjectsSelector);
            // process click
            processClick(eventData);
            break;
        }
        case DemandEditMode::DEMAND_DELETE: {
            // filter locked elements
            myViewObjectsSelector.filterLockedElements();
            // get front AC
            const auto frontAC = myViewObjectsSelector.getAttributeCarrierFront();
            // check conditions
            if (frontAC) {
                // check if we are deleting a selection or an single attribute carrier
                if (frontAC->isAttributeCarrierSelected()) {
                    myViewParent->getDeleteFrame()->removeSelectedAttributeCarriers();
                } else {
                    myViewParent->getDeleteFrame()->removeAttributeCarrier(myViewObjectsSelector);
                }
            } else {
                // process click
                processClick(eventData);
            }
            break;
        }
        case DemandEditMode::DEMAND_SELECT:
            // filter locked elements
            myViewObjectsSelector.filterLockedElements();
            // avoid to select if control key is pressed
            if (!myMouseButtonKeyPressed.controlKeyPressed()) {
                // check if a rect for selecting is being created
                if (myMouseButtonKeyPressed.shiftKeyPressed()) {
                    // begin rectangle selection
                    mySelectingArea.beginRectangleSelection();
                } else if (!myViewParent->getSelectorFrame()->selectAttributeCarrier(myViewObjectsSelector)) {
                    // process click
                    processClick(eventData);
                }
            } else {
                // process click
                processClick(eventData);
            }
            break;
        case DemandEditMode::DEMAND_MOVE: {
            // filter locked elements
            myViewObjectsSelector.filterLockedElements();
            // get front AC
            const auto frontAC = myViewObjectsSelector.getAttributeCarrierFront();
            // check that AC under cursor is a demand element
            if (frontAC) {
                // check if we're moving a set of selected items
                if (frontAC->isAttributeCarrierSelected()) {
                    // move selected ACs
                    myMoveMultipleElements.beginMoveSelection();
                    // update view
                    updateViewNet();
                } else if (!myMoveSingleElement.beginMoveSingleElementDemandMode()) {
                    // process click  if there isn't movable elements (to move camera using drag an drop)
                    processClick(eventData);
                }
            } else {
                // process click  if there isn't movable elements (to move camera using drag an drop)
                processClick(eventData);
            }
            break;
        }
        case DemandEditMode::DEMAND_ROUTE: {
            // check if we clicked over a lane
            if (myViewObjectsSelector.getLaneFront()) {
                // Handle edge click
                myViewParent->getRouteFrame()->addEdgeRoute(myViewObjectsSelector.getLaneFront()->getParentEdge(), myMouseButtonKeyPressed);
            }
            // process click
            processClick(eventData);
            break;
        }
        case DemandEditMode::DEMAND_VEHICLE: {
            // filter additionals (except TAZs) and demands (except routes)
            myViewObjectsSelector.filterAdditionals(true, false);
            myViewObjectsSelector.filterDemandElements(false);
            // Handle click
            myViewParent->getVehicleFrame()->addVehicle(myViewObjectsSelector, myMouseButtonKeyPressed);
            // process click
            processClick(eventData);
            break;
        }
        case DemandEditMode::DEMAND_STOP: {
            // filter additionals (except stoppingPlaces) and demands
            myViewObjectsSelector.filterAdditionals(false, true);
            myViewObjectsSelector.filterDemandElements(true);
            // Handle click
            if ((getPositionInformation() == myLastClickedPosition) && !myMouseButtonKeyPressed.controlKeyPressed()) {
                WRITE_WARNING(TL("Control + click to create two stop in the same position"));
            } else if (myViewParent->getStopFrame()->addStop(myViewObjectsSelector, myMouseButtonKeyPressed)) {
                // save last mouse position
                myLastClickedPosition = getPositionInformation();
                // update view to show the new additional
                updateViewNet();
            }
            // process click
            processClick(eventData);
            break;
        }
        case DemandEditMode::DEMAND_PERSON: {
            // filter additionals (except stoppingPlaces and TAZs)
            myViewObjectsSelector.filterAdditionals(false, false);
            // special case if we're creating person over walk routes
            if (myViewParent->getPersonFrame()->getPlanSelector()->getCurrentPlanTagProperties().planRoute()) {
                myViewObjectsSelector.filterDemandElements(false);
            } else {
                myViewObjectsSelector.filterDemandElements(true);
            }
            // Handle click
            myViewParent->getPersonFrame()->addPerson(myViewObjectsSelector);
            // process click
            processClick(eventData);
            break;
        }
        case DemandEditMode::DEMAND_PERSONPLAN: {
            // filter additionals (except stoppingPlaces and TAZs)
            myViewObjectsSelector.filterAdditionals(false, false);
            // special case if we're creating person over walk routes
            if (myViewParent->getPersonPlanFrame()->getPlanSelector()->getCurrentPlanTagProperties().planRoute()) {
                myViewObjectsSelector.filterDemandElements(false);
            } else {
                myViewObjectsSelector.filterDemandElements(true);
            }
            // Handle person plan click
            myViewParent->getPersonPlanFrame()->addPersonPlanElement(myViewObjectsSelector);
            // process click
            processClick(eventData);
            break;
        }
        case DemandEditMode::DEMAND_CONTAINER: {
            // filter additionals (except stoppingPlaces and TAZs) and demands
            myViewObjectsSelector.filterAdditionals(false, false);
            myViewObjectsSelector.filterDemandElements(true);
            // Handle click
            myViewParent->getContainerFrame()->addContainer(myViewObjectsSelector);
            // process click
            processClick(eventData);
            break;
        }
        case DemandEditMode::DEMAND_CONTAINERPLAN: {
            // filter additionals (except stoppingPlaces and TAZs) and demands
            myViewObjectsSelector.filterAdditionals(false, false);
            myViewObjectsSelector.filterDemandElements(true);
            // Handle container plan click
            myViewParent->getContainerPlanFrame()->addContainerPlanElement(myViewObjectsSelector);
            // process click
            processClick(eventData);
            break;
        }
        default: {
            // process click
            processClick(eventData);
        }
    }
}


void
GNEViewNet::processLeftButtonReleaseDemand() {
    // check moved items
    if (myMoveMultipleElements.isMovingSelection()) {
        myMoveMultipleElements.finishMoveSelection();
    } else if (mySelectingArea.selectingUsingRectangle) {
        // check if we're creating a rectangle selection or we want only to select a lane
        if (mySelectingArea.startDrawing) {
            mySelectingArea.processRectangleSelection();
        }
        // finish selection
        mySelectingArea.finishRectangleSelection();
    } else {
        // finish moving of single elements
        myMoveSingleElement.finishMoveSingleElement();
    }
}


void
GNEViewNet::processMoveMouseDemand(const bool mouseLeftButtonPressed) {
    if (mySelectingArea.selectingUsingRectangle) {
        // update selection corner of selecting area
        mySelectingArea.moveRectangleSelection();
    } else {
        // move single elements
        myMoveSingleElement.moveSingleElement(mouseLeftButtonPressed);
    }
}


void
GNEViewNet::processLeftButtonPressData(void* eventData) {
    // get AC
    const auto AC = myViewObjectsSelector.getAttributeCarrierFront();
    // decide what to do based on mode
    switch (myEditModes.dataEditMode) {
        case DataEditMode::DATA_INSPECT: {
            // filter locked elements
            myViewObjectsSelector.filterLockedElements();
            // process left click in Inspector Frame
            if (AC && AC->getTagProperty().getTag() == SUMO_TAG_TAZ) {
                myViewParent->getInspectorFrame()->inspectSingleElement(AC);
            } else {
                myViewParent->getInspectorFrame()->processDataSupermodeClick(getPositionInformation(), myViewObjectsSelector);
            }
            // process click
            processClick(eventData);
            break;
        }
        case DataEditMode::DATA_DELETE: {
            // check conditions
            if (AC) {
                // check if we are deleting a selection or an single attribute carrier
                if (AC->isAttributeCarrierSelected()) {
                    if (!AC->getGUIGlObject()->isGLObjectLocked()) {
                        myViewParent->getDeleteFrame()->removeSelectedAttributeCarriers();
                    }
                } else {
                    myViewParent->getDeleteFrame()->removeAttributeCarrier(myViewObjectsSelector);
                }
            } else {
                // process click
                processClick(eventData);
            }
            break;
        }
        case DataEditMode::DATA_SELECT:
            // filter locked elements
            myViewObjectsSelector.filterLockedElements();
            // avoid to select if control key is pressed
            if (!myMouseButtonKeyPressed.controlKeyPressed()) {
                // check if a rect for selecting is being created
                if (myMouseButtonKeyPressed.shiftKeyPressed()) {
                    // begin rectangle selection
                    mySelectingArea.beginRectangleSelection();
                } else if (!myViewParent->getSelectorFrame()->selectAttributeCarrier(myViewObjectsSelector)) {
                    // process click
                    processClick(eventData);
                }
            } else {
                // process click
                processClick(eventData);
            }
            break;
        case DataEditMode::DATA_EDGEDATA:
            // avoid create edgeData if control key is pressed
            if (!myMouseButtonKeyPressed.controlKeyPressed()) {
                if (myViewParent->getEdgeDataFrame()->addEdgeData(myViewObjectsSelector, myMouseButtonKeyPressed)) {
                    // update view to show the new edge data
                    updateViewNet();
                }
            }
            // process click
            processClick(eventData);
            break;
        case DataEditMode::DATA_EDGERELDATA:
            // avoid create edgeData if control key is pressed
            if (!myMouseButtonKeyPressed.controlKeyPressed()) {
                if (myViewParent->getEdgeRelDataFrame()->addEdgeRelationData(myViewObjectsSelector, myMouseButtonKeyPressed)) {
                    // update view to show the new edge data
                    updateViewNet();
                }
            }
            // process click
            processClick(eventData);
            break;
        case DataEditMode::DATA_TAZRELDATA:
            // avoid create TAZData if control key is pressed
            if (!myMouseButtonKeyPressed.controlKeyPressed()) {
                if (myViewParent->getTAZRelDataFrame()->setTAZ(myViewObjectsSelector)) {
                    // update view to show the new TAZ data
                    updateViewNet();
                }
            }
            // process click
            processClick(eventData);
            break;
        case DataEditMode::DATA_MEANDATA:
            // avoid create TAZData if control key is pressed
            if (!myMouseButtonKeyPressed.controlKeyPressed()) {
                //
            }
            // process click
            processClick(eventData);
            break;
        default: {
            // process click
            processClick(eventData);
        }
    }
}


void
GNEViewNet::processLeftButtonReleaseData() {
    // check moved items
    if (myMoveMultipleElements.isMovingSelection()) {
        myMoveMultipleElements.finishMoveSelection();
    } else if (mySelectingArea.selectingUsingRectangle) {
        // check if we're creating a rectangle selection or we want only to select a lane
        if (mySelectingArea.startDrawing) {
            mySelectingArea.processRectangleSelection();
        }
        // finish selection
        mySelectingArea.finishRectangleSelection();
    } else {
        // finish moving of single elements
        myMoveSingleElement.finishMoveSingleElement();
    }
}


void
GNEViewNet::processMoveMouseData(const bool mouseLeftButtonPressed) {
    if (mySelectingArea.selectingUsingRectangle) {
        // update selection corner of selecting area
        mySelectingArea.moveRectangleSelection();
    } else {
        // move single elements
        myMoveSingleElement.moveSingleElement(mouseLeftButtonPressed);
    }
}


/****************************************************************************/
