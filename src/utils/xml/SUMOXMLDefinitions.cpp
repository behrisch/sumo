/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2002-2024 German Aerospace Center (DLR) and others.
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
/// @file    SUMOXMLDefinitions.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Clemens Honomichl
/// @author  Piotr Woznica
/// @author  Michael Behrisch
/// @author  Walter Bamberger
/// @date    Sept 2002
///
// Definitions of elements and attributes known by SUMO
/****************************************************************************/
#include <config.h>

#include <algorithm>
#include <cassert>
#include <utils/common/StringTokenizer.h>
#include <utils/common/StringUtils.h>

#include "SUMOXMLDefinitions.h"

// ===========================================================================
// definitions
// ===========================================================================

SequentialStringBijection::Entry SUMOXMLDefinitions::tags[] = {
    // Simulation elements
    { "net",                                    SUMO_TAG_NET },
    { "edge",                                   SUMO_TAG_EDGE },
    { "lane",                                   SUMO_TAG_LANE },
    { "neigh",                                  SUMO_TAG_NEIGH },
    { "poly",                                   SUMO_TAG_POLY },
    { "poi",                                    SUMO_TAG_POI },
    { "junction",                               SUMO_TAG_JUNCTION },
    { "restriction",                            SUMO_TAG_RESTRICTION },
    { "meso",                                   SUMO_TAG_MESO },
    { "busStop",                                SUMO_TAG_BUS_STOP },
    { "trainStop",                              SUMO_TAG_TRAIN_STOP },
    { "ptLine",                                 SUMO_TAG_PT_LINE },
    { "access",                                 SUMO_TAG_ACCESS },
    { "containerStop",                          SUMO_TAG_CONTAINER_STOP },
    { "chargingStation",                        SUMO_TAG_CHARGING_STATION },
    { "chargingEvent",                          SUMO_TAG_CHARGING_EVENT },
    { "parkingArea",                            SUMO_TAG_PARKING_AREA },
    { "space",                                  SUMO_TAG_PARKING_SPACE },
    { "e1Detector",                             SUMO_TAG_E1DETECTOR },
    { "inductionLoop",                          SUMO_TAG_INDUCTION_LOOP },
    { "e2Detector",                             SUMO_TAG_E2DETECTOR },
    { "laneAreaDetector",                       SUMO_TAG_LANE_AREA_DETECTOR },
    { "multiLaneAreaDetector",                  GNE_TAG_MULTI_LANE_AREA_DETECTOR },
    { "e3Detector",                             SUMO_TAG_E3DETECTOR },
    { "entryExitDetector",                      SUMO_TAG_ENTRY_EXIT_DETECTOR },
    { "detEntry",                               SUMO_TAG_DET_ENTRY },
    { "detExit",                                SUMO_TAG_DET_EXIT },
    { "edgeData",                               SUMO_TAG_MEANDATA_EDGE },
    { "laneData",                               SUMO_TAG_MEANDATA_LANE },
    { "edgeFollowDetector",                     SUMO_TAG_EDGEFOLLOWDETECTOR },
    { "instantInductionLoop",                   SUMO_TAG_INSTANT_INDUCTION_LOOP },
    { "routeProbe",                             SUMO_TAG_ROUTEPROBE },
    { "calibrator",                             SUMO_TAG_CALIBRATOR },
    { "calibratorLane",                         GNE_TAG_CALIBRATOR_LANE },
    { "calibratorFlow",                         GNE_TAG_CALIBRATOR_FLOW },
    { "rerouter",                               SUMO_TAG_REROUTER },
    { "rerouterSymbol",                         GNE_TAG_REROUTER_SYMBOL },
    { "interval",                               SUMO_TAG_INTERVAL },
    { "destProbReroute",                        SUMO_TAG_DEST_PROB_REROUTE },
    { "closingReroute",                         SUMO_TAG_CLOSING_REROUTE },
    { "closingLaneReroute",                     SUMO_TAG_CLOSING_LANE_REROUTE },
    { "routeProbReroute",                       SUMO_TAG_ROUTE_PROB_REROUTE },
    { "parkingAreaReroute",                     SUMO_TAG_PARKING_AREA_REROUTE },
    { "viaProbReroute",                         SUMO_TAG_VIA_PROB_REROUTE },
    { "step",                                   SUMO_TAG_STEP },
    { "variableSpeedSign",                      SUMO_TAG_VSS },
    { "variableSpeedSignSymbol",                GNE_TAG_VSS_SYMBOL },
    { "vaporizer",                              SUMO_TAG_VAPORIZER },
    { "tractionSubstation",                     SUMO_TAG_TRACTION_SUBSTATION },
    { "overheadWireSegment",                    SUMO_TAG_OVERHEAD_WIRE_SEGMENT },   // <- Deprecate
    { "overheadWire",                           SUMO_TAG_OVERHEAD_WIRE_SECTION },
    { "overheadWireClamp",                      SUMO_TAG_OVERHEAD_WIRE_CLAMP },
    { "vTypeProbe",                             SUMO_TAG_VTYPEPROBE },
    { "routes",                                 SUMO_TAG_ROUTES },
    { "trip",                                   SUMO_TAG_TRIP },
    { "tripJunctions",                          GNE_TAG_TRIP_JUNCTIONS },
    { "tripTAZs",                               GNE_TAG_TRIP_TAZS },
    { "vehicle",                                SUMO_TAG_VEHICLE },
    { "vehicleWithRoute",                       GNE_TAG_VEHICLE_WITHROUTE },
    { "flow",                                   SUMO_TAG_FLOW },
    { "flowJunctions",                          GNE_TAG_FLOW_JUNCTIONS },
    { "flowTAZs",                               GNE_TAG_FLOW_TAZS },
    { "flowState",                              SUMO_TAG_FLOWSTATE },
    { "vType",                                  SUMO_TAG_VTYPE },
    { "route",                                  SUMO_TAG_ROUTE },
    { "routeEmbedded",                          GNE_TAG_ROUTE_EMBEDDED },
    { "request",                                SUMO_TAG_REQUEST },
    { "source",                                 SUMO_TAG_SOURCE },
    { "taz",                                    SUMO_TAG_TAZ },
    { "tazSource",                              SUMO_TAG_TAZSOURCE },
    { "tazSink",                                SUMO_TAG_TAZSINK },
    { "trafficLight",                           SUMO_TAG_TRAFFIC_LIGHT },
    { "tlLogic",                                SUMO_TAG_TLLOGIC },
    { "phase",                                  SUMO_TAG_PHASE },
    { "condition",                              SUMO_TAG_CONDITION },
    { "assignment",                             SUMO_TAG_ASSIGNMENT },
    { "function",                               SUMO_TAG_FUNCTION },
    { "edgeControl",                            SUMO_TAG_EDGECONTROL },

    { "edgeRelation",                           SUMO_TAG_EDGEREL },
    { "tazRelation",                            SUMO_TAG_TAZREL },
    { "timedEvent",                             SUMO_TAG_TIMEDEVENT },
    { "fromEdge",                               SUMO_TAG_FROMEDGE },
    { "toEdge",                                 SUMO_TAG_TOEDGE },
    { "sink",                                   SUMO_TAG_SINK },
    { "param",                                  SUMO_TAG_PARAM },
    { "WAUT",                                   SUMO_TAG_WAUT },
    { "wautSwitch",                             SUMO_TAG_WAUT_SWITCH },
    { "wautJunction",                           SUMO_TAG_WAUT_JUNCTION },
    { "segment",                                SUMO_TAG_SEGMENT },
    { "delete",                                 SUMO_TAG_DEL },
    { "connections",                            SUMO_TAG_CONNECTIONS },
    { "stop",                                   SUMO_TAG_STOP },
    { "stopBusStop",                            GNE_TAG_STOP_BUSSTOP },
    { "stopTrainStop",                          GNE_TAG_STOP_TRAINSTOP },
    { "stopContainerStop",                      GNE_TAG_STOP_CONTAINERSTOP },
    { "stopChargingStation",                    GNE_TAG_STOP_CHARGINGSTATION },
    { "stopParkingArea",                        GNE_TAG_STOP_PARKINGAREA },
    { "stopLane",                               GNE_TAG_STOP_LANE },
    { "polygonType",                            SUMO_TAG_POLYTYPE },
    { "connection",                             SUMO_TAG_CONNECTION },
    { "conflict",                               SUMO_TAG_CONFLICT },
    { "prohibition",                            SUMO_TAG_PROHIBITION },
    { "split",                                  SUMO_TAG_SPLIT },
    { "node",                                   SUMO_TAG_NODE },
    { "type",                                   SUMO_TAG_TYPE },
    { "laneType",                               SUMO_TAG_LANETYPE },
    { "detectorDefinition",                     SUMO_TAG_DETECTOR_DEFINITION },
    { "routeDistribution",                      SUMO_TAG_ROUTE_DISTRIBUTION },
    { "vTypeDistribution",                      SUMO_TAG_VTYPE_DISTRIBUTION },
    { "roundabout",                             SUMO_TAG_ROUNDABOUT },
    { "join",                                   SUMO_TAG_JOIN },
    { "joinExclude",                            SUMO_TAG_JOINEXCLUDE },
    { "crossing",                               SUMO_TAG_CROSSING },
    { "walkingArea",                            SUMO_TAG_WALKINGAREA },
    { "stopOffset",                             SUMO_TAG_STOPOFFSET },
    { "railSignalConstraints",                  SUMO_TAG_RAILSIGNAL_CONSTRAINTS },
    { "predecessor",                            SUMO_TAG_PREDECESSOR },
    { "insertionPredecessor",                   SUMO_TAG_INSERTION_PREDECESSOR },
    { "foeInsertion",                           SUMO_TAG_FOE_INSERTION },
    { "insertionOrder",                         SUMO_TAG_INSERTION_ORDER },
    { "bidiPredecessor",                        SUMO_TAG_BIDI_PREDECESSOR },
    { "railSignalConstraintTracker",            SUMO_TAG_RAILSIGNAL_CONSTRAINT_TRACKER },
    { "deadlock",                               SUMO_TAG_DEADLOCK },
    { "driveWay",                               SUMO_TAG_DRIVEWAY },
    { "subDriveWay",                            SUMO_TAG_SUBDRIVEWAY },
    { "link",                                   SUMO_TAG_LINK },
    { "approaching",                            SUMO_TAG_APPROACHING },
    // OSM
    { "way",                                    SUMO_TAG_WAY },
    { "nd",                                     SUMO_TAG_ND },
    { "tag",                                    SUMO_TAG_TAG },
    { "relation",                               SUMO_TAG_RELATION },
    { "member",                                 SUMO_TAG_MEMBER },
    // View
    { "viewsettings",                           SUMO_TAG_VIEWSETTINGS },
    { "view3D",                                 SUMO_TAG_VIEWSETTINGS_3D },
    { "decal",                                  SUMO_TAG_VIEWSETTINGS_DECAL },
    { "light",                                  SUMO_TAG_VIEWSETTINGS_LIGHT },
    { "scheme",                                 SUMO_TAG_VIEWSETTINGS_SCHEME },
    { "opengl",                                 SUMO_TAG_VIEWSETTINGS_OPENGL },
    { "background",                             SUMO_TAG_VIEWSETTINGS_BACKGROUND },
    { "edges",                                  SUMO_TAG_VIEWSETTINGS_EDGES },
    { "vehicles",                               SUMO_TAG_VIEWSETTINGS_VEHICLES },
    { "persons",                                SUMO_TAG_VIEWSETTINGS_PERSONS },
    { "containers",                             SUMO_TAG_VIEWSETTINGS_CONTAINERS },
    { "junctions",                              SUMO_TAG_VIEWSETTINGS_JUNCTIONS },
    { "additionals",                            SUMO_TAG_VIEWSETTINGS_ADDITIONALS },
    { "pois",                                   SUMO_TAG_VIEWSETTINGS_POIS },
    { "polys",                                  SUMO_TAG_VIEWSETTINGS_POLYS },
    { "legend",                                 SUMO_TAG_VIEWSETTINGS_LEGEND },
    { "event",                                  SUMO_TAG_VIEWSETTINGS_EVENT },
    { "jamTime",                                SUMO_TAG_VIEWSETTINGS_EVENT_JAM_TIME },
    { "include",                                SUMO_TAG_INCLUDE },
    { "delay",                                  SUMO_TAG_DELAY },
    { "viewport",                               SUMO_TAG_VIEWPORT },
    { "snapshot",                               SUMO_TAG_SNAPSHOT },
    { "breakpoint",                             SUMO_TAG_BREAKPOINT },
    { "location",                               SUMO_TAG_LOCATION },
    { "colorScheme",                            SUMO_TAG_COLORSCHEME },
    { "scalingScheme",                          SUMO_TAG_SCALINGSCHEME },
    { "entry",                                  SUMO_TAG_ENTRY },
    { "rngState",                               SUMO_TAG_RNGSTATE },
    { "rngLane",                                SUMO_TAG_RNGLANE },
    { "vehicleTransfer",                        SUMO_TAG_VEHICLETRANSFER },
    { "device",                                 SUMO_TAG_DEVICE },
    // Cars
    { "carFollowing-IDM",                       SUMO_TAG_CF_IDM },
    { "carFollowing-IDMM",                      SUMO_TAG_CF_IDMM },
    { "carFollowing-Krauss",                    SUMO_TAG_CF_KRAUSS },
    { "carFollowing-KraussPS",                  SUMO_TAG_CF_KRAUSS_PLUS_SLOPE },
    { "carFollowing-KraussOrig1",               SUMO_TAG_CF_KRAUSS_ORIG1 },
    { "carFollowing-KraussX",                   SUMO_TAG_CF_KRAUSSX },
    { "carFollowing-EIDM",                      SUMO_TAG_CF_EIDM },
    { "carFollowing-SmartSK",                   SUMO_TAG_CF_SMART_SK },
    { "carFollowing-Daniel1",                   SUMO_TAG_CF_DANIEL1 },
    { "carFollowing-PWagner2009",               SUMO_TAG_CF_PWAGNER2009 },
    { "carFollowing-BKerner",                   SUMO_TAG_CF_BKERNER },
    { "carFollowing-Wiedemann",                 SUMO_TAG_CF_WIEDEMANN },
    { "carFollowing-W99",                       SUMO_TAG_CF_W99 },
    { "carFollowing-Rail",                      SUMO_TAG_CF_RAIL },
    { "carFollowing-ACC",                       SUMO_TAG_CF_ACC },
    { "carFollowing-CACC",                      SUMO_TAG_CF_CACC },
    { "carFollowing-CC",                        SUMO_TAG_CF_CC },
    // Person
    { "person",                                 SUMO_TAG_PERSON },
    { "personTrip",                             SUMO_TAG_PERSONTRIP },
    { "ride",                                   SUMO_TAG_RIDE },
    { "walk",                                   SUMO_TAG_WALK },
    { "personFlow",                             SUMO_TAG_PERSONFLOW },
    // Data (Netedit)
    { "dataSet",                                SUMO_TAG_DATASET },
    { "dataInterval",                           SUMO_TAG_DATAINTERVAL },
    // Transport
    { "container",                              SUMO_TAG_CONTAINER },
    { "transport",                              SUMO_TAG_TRANSPORT },
    { "tranship",                               SUMO_TAG_TRANSHIP },
    { "containerFlow",					        SUMO_TAG_CONTAINERFLOW },
    //Trajectories
    { "trajectories",                           SUMO_TAG_TRAJECTORIES },
    { "timestep",                               SUMO_TAG_TIMESTEP },
    { "timeSlice",                              SUMO_TAG_TIMESLICE },
    { "actorConfig",                            SUMO_TAG_ACTORCONFIG },
    { "motionState",                            SUMO_TAG_MOTIONSTATE },
    { "odPair",                                 SUMO_TAG_OD_PAIR },
    { "transportables",                         SUMO_TAG_TRANSPORTABLES },
    // ActivityGen statistics file
    { "general",                                AGEN_TAG_GENERAL },
    { "street",                                 AGEN_TAG_STREET },
    { "workHours",                              AGEN_TAG_WORKHOURS },
    { "opening",                                AGEN_TAG_OPENING },
    { "closing",                                AGEN_TAG_CLOSING },
    { "schools",                                AGEN_TAG_SCHOOLS },
    { "school",                                 AGEN_TAG_SCHOOL },
    { "busStation",                             AGEN_TAG_BUSSTATION },
    { "busLine",                                AGEN_TAG_BUSLINE },
    { "stations",                               AGEN_TAG_STATIONS },
    { "revStations",                            AGEN_TAG_REV_STATIONS },
    { "station",                                AGEN_TAG_STATION },
    { "frequency",                              AGEN_TAG_FREQUENCY },
    { "population",                             AGEN_TAG_POPULATION },
    { "bracket",                                AGEN_TAG_BRACKET },
    { "cityGates",                              AGEN_TAG_CITYGATES },
    { "entrance",                               AGEN_TAG_ENTRANCE },
    { "parameters",                             AGEN_TAG_PARAM },
    // Netedit
    { "edgeData-edge",                          GNE_TAG_EDGEREL_SINGLE },
    { "internalLane",                           GNE_TAG_INTERNAL_LANE },
    { "poiLane",                                GNE_TAG_POILANE },
    { "poiGeo",                                 GNE_TAG_POIGEO },
    { "jupedsim.walkable_area",                 GNE_TAG_JPS_WALKABLEAREA },
    { "jupedsim.obstacle",                      GNE_TAG_JPS_OBSTACLE },
    { "flowRoute",                              GNE_TAG_FLOW_ROUTE },
    { "flowWithRoute",                          GNE_TAG_FLOW_WITHROUTE },
    // GNE waypoints
    { "waypoint",                               GNE_TAG_WAYPOINT },
    { "waypointLane",                           GNE_TAG_WAYPOINT_LANE },
    { "waypointBusStop",                        GNE_TAG_WAYPOINT_BUSSTOP },
    { "waypointTrainStop",                      GNE_TAG_WAYPOINT_TRAINSTOP },
    { "waypointContainerStop",                  GNE_TAG_WAYPOINT_CONTAINERSTOP },
    { "waypointChargingStation",                GNE_TAG_WAYPOINT_CHARGINGSTATION },
    { "waypointParkingArea",                    GNE_TAG_WAYPOINT_PARKINGAREA },
    // GNE Person trips
    { "persontrip: edge->edge",                       GNE_TAG_PERSONTRIP_EDGE_EDGE },
    { "persontrip: edge->taz",                        GNE_TAG_PERSONTRIP_EDGE_TAZ },
    { "persontrip: edge->junction",                   GNE_TAG_PERSONTRIP_EDGE_JUNCTION },
    { "persontrip: edge->busstop",                    GNE_TAG_PERSONTRIP_EDGE_BUSSTOP },
    { "persontrip: edge->trainstop",                  GNE_TAG_PERSONTRIP_EDGE_TRAINSTOP },
    { "persontrip: edge->containerstop",              GNE_TAG_PERSONTRIP_EDGE_CONTAINERSTOP },
    { "persontrip: edge->chargingstation",            GNE_TAG_PERSONTRIP_EDGE_CHARGINGSTATION },
    { "persontrip: edge->parkingarea",                GNE_TAG_PERSONTRIP_EDGE_PARKINGAREA },
    { "persontrip: taz->edge",                        GNE_TAG_PERSONTRIP_TAZ_EDGE },
    { "persontrip: taz->taz",                         GNE_TAG_PERSONTRIP_TAZ_TAZ },
    { "persontrip: taz->junction",                    GNE_TAG_PERSONTRIP_TAZ_JUNCTION },
    { "persontrip: taz->busstop",                     GNE_TAG_PERSONTRIP_TAZ_BUSSTOP },
    { "persontrip: taz->trainstop",                   GNE_TAG_PERSONTRIP_TAZ_TRAINSTOP },
    { "persontrip: taz->containerstop",               GNE_TAG_PERSONTRIP_TAZ_CONTAINERSTOP },
    { "persontrip: taz->chargingstation",             GNE_TAG_PERSONTRIP_TAZ_CHARGINGSTATION },
    { "persontrip: taz->parkingarea",                 GNE_TAG_PERSONTRIP_TAZ_PARKINGAREA },
    { "persontrip: junction->edge",                   GNE_TAG_PERSONTRIP_JUNCTION_EDGE },
    { "persontrip: junction->taz",                    GNE_TAG_PERSONTRIP_JUNCTION_TAZ },
    { "persontrip: junction->junction",               GNE_TAG_PERSONTRIP_JUNCTION_JUNCTION },
    { "persontrip: junction->busstop",                GNE_TAG_PERSONTRIP_JUNCTION_BUSSTOP },
    { "persontrip: junction->trainstop",              GNE_TAG_PERSONTRIP_JUNCTION_TRAINSTOP },
    { "persontrip: junction->containerstop",          GNE_TAG_PERSONTRIP_JUNCTION_CONTAINERSTOP },
    { "persontrip: junction->chargingstation",        GNE_TAG_PERSONTRIP_JUNCTION_CHARGINGSTATION },
    { "persontrip: junction->parkingarea",            GNE_TAG_PERSONTRIP_JUNCTION_PARKINGAREA },
    { "persontrip: busstop->edge",                    GNE_TAG_PERSONTRIP_BUSSTOP_EDGE },
    { "persontrip: busstop->taz",                     GNE_TAG_PERSONTRIP_BUSSTOP_TAZ },
    { "persontrip: busstop->junction",                GNE_TAG_PERSONTRIP_BUSSTOP_JUNCTION },
    { "persontrip: busstop->busstop",                 GNE_TAG_PERSONTRIP_BUSSTOP_BUSSTOP },
    { "persontrip: busstop->trainstop",               GNE_TAG_PERSONTRIP_BUSSTOP_TRAINSTOP },
    { "persontrip: busstop->containerstop",           GNE_TAG_PERSONTRIP_BUSSTOP_CONTAINERSTOP },
    { "persontrip: busstop->chargingstation",         GNE_TAG_PERSONTRIP_BUSSTOP_CHARGINGSTATION },
    { "persontrip: busstop->parkingarea",             GNE_TAG_PERSONTRIP_BUSSTOP_PARKINGAREA },
    { "persontrip: trainstop->edge",                  GNE_TAG_PERSONTRIP_TRAINSTOP_EDGE },
    { "persontrip: trainstop->taz",                   GNE_TAG_PERSONTRIP_TRAINSTOP_TAZ },
    { "persontrip: trainstop->junction",              GNE_TAG_PERSONTRIP_TRAINSTOP_JUNCTION },
    { "persontrip: trainstop->busstop",               GNE_TAG_PERSONTRIP_TRAINSTOP_BUSSTOP },
    { "persontrip: trainstop->trainstop",             GNE_TAG_PERSONTRIP_TRAINSTOP_TRAINSTOP },
    { "persontrip: trainstop->containerstop",         GNE_TAG_PERSONTRIP_TRAINSTOP_CONTAINERSTOP },
    { "persontrip: trainstop->chargingstation",       GNE_TAG_PERSONTRIP_TRAINSTOP_CHARGINGSTATION },
    { "persontrip: trainstop->parkingarea",           GNE_TAG_PERSONTRIP_TRAINSTOP_PARKINGAREA },
    { "persontrip: containerstop->edge",              GNE_TAG_PERSONTRIP_CONTAINERSTOP_EDGE },
    { "persontrip: containerstop->taz",               GNE_TAG_PERSONTRIP_CONTAINERSTOP_TAZ },
    { "persontrip: containerstop->junction",          GNE_TAG_PERSONTRIP_CONTAINERSTOP_JUNCTION },
    { "persontrip: containerstop->busstop",           GNE_TAG_PERSONTRIP_CONTAINERSTOP_BUSSTOP },
    { "persontrip: containerstop->trainstop",         GNE_TAG_PERSONTRIP_CONTAINERSTOP_TRAINSTOP },
    { "persontrip: containerstop->containerstop",     GNE_TAG_PERSONTRIP_CONTAINERSTOP_CONTAINERSTOP },
    { "persontrip: containerstop->chargingstation",   GNE_TAG_PERSONTRIP_CONTAINERSTOP_CHARGINGSTATION },
    { "persontrip: containerstop->parkingarea",       GNE_TAG_PERSONTRIP_CONTAINERSTOP_PARKINGAREA },
    { "persontrip: chargingstation->edge",            GNE_TAG_PERSONTRIP_CHARGINGSTATION_EDGE },
    { "persontrip: chargingstation->taz",             GNE_TAG_PERSONTRIP_CHARGINGSTATION_TAZ },
    { "persontrip: chargingstation->junction",        GNE_TAG_PERSONTRIP_CHARGINGSTATION_JUNCTION },
    { "persontrip: chargingstation->busstop",         GNE_TAG_PERSONTRIP_CHARGINGSTATION_BUSSTOP },
    { "persontrip: chargingstation->trainstop",       GNE_TAG_PERSONTRIP_CHARGINGSTATION_TRAINSTOP },
    { "persontrip: chargingstation->containestop",    GNE_TAG_PERSONTRIP_CHARGINGSTATION_CONTAINERSTOP },
    { "persontrip: chargingstation->chargingstation", GNE_TAG_PERSONTRIP_CHARGINGSTATION_CHARGINGSTATION },
    { "persontrip: chargingstation->parkingarea",     GNE_TAG_PERSONTRIP_CHARGINGSTATION_PARKINGAREA },
    { "persontrip: parkingarea->edge",                GNE_TAG_PERSONTRIP_PARKINGAREA_EDGE },
    { "persontrip: parkingarea->taz",                 GNE_TAG_PERSONTRIP_PARKINGAREA_TAZ },
    { "persontrip: parkingarea->junction",            GNE_TAG_PERSONTRIP_PARKINGAREA_JUNCTION },
    { "persontrip: parkingarea->busstop",             GNE_TAG_PERSONTRIP_PARKINGAREA_BUSSTOP },
    { "persontrip: parkingarea->trainstop",           GNE_TAG_PERSONTRIP_PARKINGAREA_TRAINSTOP },
    { "persontrip: parkingarea->containerstop",       GNE_TAG_PERSONTRIP_PARKINGAREA_CONTAINERSTOP },
    { "persontrip: parkingarea->chargingstation",     GNE_TAG_PERSONTRIP_PARKINGAREA_CHARGINGSTATION },
    { "persontrip: parkingarea->parkingarea",         GNE_TAG_PERSONTRIP_PARKINGAREA_PARKINGAREA },
    // GNE Walks
    { "walk: edge->edge",                       GNE_TAG_WALK_EDGE_EDGE },
    { "walk: edge->taz",                        GNE_TAG_WALK_EDGE_TAZ },
    { "walk: edge->junction",                   GNE_TAG_WALK_EDGE_JUNCTION },
    { "walk: edge->busstop",                    GNE_TAG_WALK_EDGE_BUSSTOP },
    { "walk: edge->trainstop",                  GNE_TAG_WALK_EDGE_TRAINSTOP },
    { "walk: edge->containerstop",              GNE_TAG_WALK_EDGE_CONTAINERSTOP },
    { "walk: edge->chargingstation",            GNE_TAG_WALK_EDGE_CHARGINGSTATION },
    { "walk: edge->parkingarea",                GNE_TAG_WALK_EDGE_PARKINGAREA },
    { "walk: taz->edge",                        GNE_TAG_WALK_TAZ_EDGE },
    { "walk: taz->taz",                         GNE_TAG_WALK_TAZ_TAZ },
    { "walk: taz->junction",                    GNE_TAG_WALK_TAZ_JUNCTION },
    { "walk: taz->busstop",                     GNE_TAG_WALK_TAZ_BUSSTOP },
    { "walk: taz->trainstop",                   GNE_TAG_WALK_TAZ_TRAINSTOP },
    { "walk: taz->containerstop",               GNE_TAG_WALK_TAZ_CONTAINERSTOP },
    { "walk: taz->chargingstation",             GNE_TAG_WALK_TAZ_CHARGINGSTATION },
    { "walk: taz->parkingarea",                 GNE_TAG_WALK_TAZ_PARKINGAREA },
    { "walk: junction->edge",                   GNE_TAG_WALK_JUNCTION_EDGE },
    { "walk: junction->taz",                    GNE_TAG_WALK_JUNCTION_TAZ },
    { "walk: junction->junction",               GNE_TAG_WALK_JUNCTION_JUNCTION },
    { "walk: junction->busstop",                GNE_TAG_WALK_JUNCTION_BUSSTOP },
    { "walk: junction->trainstop",              GNE_TAG_WALK_JUNCTION_TRAINSTOP },
    { "walk: junction->containerstop",          GNE_TAG_WALK_JUNCTION_CONTAINERSTOP },
    { "walk: junction->chargingstation",        GNE_TAG_WALK_JUNCTION_CHARGINGSTATION },
    { "walk: junction->parkingarea",            GNE_TAG_WALK_JUNCTION_PARKINGAREA },
    { "walk: busstop->edge",                    GNE_TAG_WALK_BUSSTOP_EDGE },
    { "walk: busstop->taz",                     GNE_TAG_WALK_BUSSTOP_TAZ },
    { "walk: busstop->junction",                GNE_TAG_WALK_BUSSTOP_JUNCTION },
    { "walk: busstop->busstop",                 GNE_TAG_WALK_BUSSTOP_BUSSTOP },
    { "walk: busstop->trainstop",               GNE_TAG_WALK_BUSSTOP_TRAINSTOP },
    { "walk: busstop->containerstop",           GNE_TAG_WALK_BUSSTOP_CONTAINERSTOP },
    { "walk: busstop->chargingstation",         GNE_TAG_WALK_BUSSTOP_CHARGINGSTATION },
    { "walk: busstop->parkingarea",             GNE_TAG_WALK_BUSSTOP_PARKINGAREA },
    { "walk: trainstop->edge",                  GNE_TAG_WALK_TRAINSTOP_EDGE },
    { "walk: trainstop->taz",                   GNE_TAG_WALK_TRAINSTOP_TAZ },
    { "walk: trainstop->junction",              GNE_TAG_WALK_TRAINSTOP_JUNCTION },
    { "walk: trainstop->busstop",               GNE_TAG_WALK_TRAINSTOP_BUSSTOP },
    { "walk: trainstop->trainstop",             GNE_TAG_WALK_TRAINSTOP_TRAINSTOP },
    { "walk: trainstop->containerstop",         GNE_TAG_WALK_TRAINSTOP_CONTAINERSTOP },
    { "walk: trainstop->chargingstation",       GNE_TAG_WALK_TRAINSTOP_CHARGINGSTATION },
    { "walk: trainstop->parkingarea",           GNE_TAG_WALK_TRAINSTOP_PARKINGAREA },
    { "walk: containerstop->edge",              GNE_TAG_WALK_CONTAINERSTOP_EDGE },
    { "walk: containerstop->taz",               GNE_TAG_WALK_CONTAINERSTOP_TAZ },
    { "walk: containerstop->junction",          GNE_TAG_WALK_CONTAINERSTOP_JUNCTION },
    { "walk: containerstop->busstop",           GNE_TAG_WALK_CONTAINERSTOP_BUSSTOP },
    { "walk: containerstop->trainstop",         GNE_TAG_WALK_CONTAINERSTOP_TRAINSTOP },
    { "walk: containerstop->containerstop",     GNE_TAG_WALK_CONTAINERSTOP_CONTAINERSTOP },
    { "walk: containerstop->chargingstation",   GNE_TAG_WALK_CONTAINERSTOP_CHARGINGSTATION },
    { "walk: containerstop->parkingarea",       GNE_TAG_WALK_CONTAINERSTOP_PARKINGAREA },
    { "walk: chargingstation->edge",            GNE_TAG_WALK_CHARGINGSTATION_EDGE },
    { "walk: chargingstation->taz",             GNE_TAG_WALK_CHARGINGSTATION_TAZ },
    { "walk: chargingstation->junction",        GNE_TAG_WALK_CHARGINGSTATION_JUNCTION },
    { "walk: chargingstation->busstop",         GNE_TAG_WALK_CHARGINGSTATION_BUSSTOP },
    { "walk: chargingstation->trainstop",       GNE_TAG_WALK_CHARGINGSTATION_TRAINSTOP },
    { "walk: chargingstation->containestop",    GNE_TAG_WALK_CHARGINGSTATION_CONTAINERSTOP },
    { "walk: chargingstation->chargingstation", GNE_TAG_WALK_CHARGINGSTATION_CHARGINGSTATION },
    { "walk: chargingstation->parkingarea",     GNE_TAG_WALK_CHARGINGSTATION_PARKINGAREA },
    { "walk: parkingarea->edge",                GNE_TAG_WALK_PARKINGAREA_EDGE },
    { "walk: parkingarea->taz",                 GNE_TAG_WALK_PARKINGAREA_TAZ },
    { "walk: parkingarea->junction",            GNE_TAG_WALK_PARKINGAREA_JUNCTION },
    { "walk: parkingarea->busstop",             GNE_TAG_WALK_PARKINGAREA_BUSSTOP },
    { "walk: parkingarea->trainstop",           GNE_TAG_WALK_PARKINGAREA_TRAINSTOP },
    { "walk: parkingarea->containerstop",       GNE_TAG_WALK_PARKINGAREA_CONTAINERSTOP },
    { "walk: parkingarea->chargingstation",     GNE_TAG_WALK_PARKINGAREA_CHARGINGSTATION },
    { "walk: parkingarea->parkingarea",         GNE_TAG_WALK_PARKINGAREA_PARKINGAREA },
    { "walk: edges",                            GNE_TAG_WALK_EDGES },
    { "walk: route",                            GNE_TAG_WALK_ROUTE },
    // GNE Rides
    { "ride: edge->edge",                       GNE_TAG_RIDE_EDGE_EDGE },
    { "ride: edge->taz",                        GNE_TAG_RIDE_EDGE_TAZ },
    { "ride: edge->junction",                   GNE_TAG_RIDE_EDGE_JUNCTION },
    { "ride: edge->busstop",                    GNE_TAG_RIDE_EDGE_BUSSTOP },
    { "ride: edge->trainstop",                  GNE_TAG_RIDE_EDGE_TRAINSTOP },
    { "ride: edge->containerstop",              GNE_TAG_RIDE_EDGE_CONTAINERSTOP },
    { "ride: edge->chargingstation",            GNE_TAG_RIDE_EDGE_CHARGINGSTATION },
    { "ride: edge->parkingarea",                GNE_TAG_RIDE_EDGE_PARKINGAREA },
    { "ride: taz->edge",                        GNE_TAG_RIDE_TAZ_EDGE },
    { "ride: taz->taz",                         GNE_TAG_RIDE_TAZ_TAZ },
    { "ride: taz->junction",                    GNE_TAG_RIDE_TAZ_JUNCTION },
    { "ride: taz->busstop",                     GNE_TAG_RIDE_TAZ_BUSSTOP },
    { "ride: taz->trainstop",                   GNE_TAG_RIDE_TAZ_TRAINSTOP },
    { "ride: taz->containerstop",               GNE_TAG_RIDE_TAZ_CONTAINERSTOP },
    { "ride: taz->chargingstation",             GNE_TAG_RIDE_TAZ_CHARGINGSTATION },
    { "ride: taz->parkingarea",                 GNE_TAG_RIDE_TAZ_PARKINGAREA },
    { "ride: junction->edge",                   GNE_TAG_RIDE_JUNCTION_EDGE },
    { "ride: junction->taz",                    GNE_TAG_RIDE_JUNCTION_TAZ },
    { "ride: junction->junction",               GNE_TAG_RIDE_JUNCTION_JUNCTION },
    { "ride: junction->busstop",                GNE_TAG_RIDE_JUNCTION_BUSSTOP },
    { "ride: junction->trainstop",              GNE_TAG_RIDE_JUNCTION_TRAINSTOP },
    { "ride: junction->containerstop",          GNE_TAG_RIDE_JUNCTION_CONTAINERSTOP },
    { "ride: junction->chargingstation",        GNE_TAG_RIDE_JUNCTION_CHARGINGSTATION },
    { "ride: junction->parkingarea",            GNE_TAG_RIDE_JUNCTION_PARKINGAREA },
    { "ride: busstop->edge",                    GNE_TAG_RIDE_BUSSTOP_EDGE },
    { "ride: busstop->taz",                     GNE_TAG_RIDE_BUSSTOP_TAZ },
    { "ride: busstop->junction",                GNE_TAG_RIDE_BUSSTOP_JUNCTION },
    { "ride: busstop->busstop",                 GNE_TAG_RIDE_BUSSTOP_BUSSTOP },
    { "ride: busstop->trainstop",               GNE_TAG_RIDE_BUSSTOP_TRAINSTOP },
    { "ride: busstop->containerstop",           GNE_TAG_RIDE_BUSSTOP_CONTAINERSTOP },
    { "ride: busstop->chargingstation",         GNE_TAG_RIDE_BUSSTOP_CHARGINGSTATION },
    { "ride: busstop->parkingarea",             GNE_TAG_RIDE_BUSSTOP_PARKINGAREA },
    { "ride: trainstop->edge",                  GNE_TAG_RIDE_TRAINSTOP_EDGE },
    { "ride: trainstop->taz",                   GNE_TAG_RIDE_TRAINSTOP_TAZ },
    { "ride: trainstop->junction",              GNE_TAG_RIDE_TRAINSTOP_JUNCTION },
    { "ride: trainstop->busstop",               GNE_TAG_RIDE_TRAINSTOP_BUSSTOP },
    { "ride: trainstop->trainstop",             GNE_TAG_RIDE_TRAINSTOP_TRAINSTOP },
    { "ride: trainstop->containerstop",         GNE_TAG_RIDE_TRAINSTOP_CONTAINERSTOP },
    { "ride: trainstop->chargingstation",       GNE_TAG_RIDE_TRAINSTOP_CHARGINGSTATION },
    { "ride: trainstop->parkingarea",           GNE_TAG_RIDE_TRAINSTOP_PARKINGAREA },
    { "ride: containerstop->edge",              GNE_TAG_RIDE_CONTAINERSTOP_EDGE },
    { "ride: containerstop->taz",               GNE_TAG_RIDE_CONTAINERSTOP_TAZ },
    { "ride: containerstop->junction",          GNE_TAG_RIDE_CONTAINERSTOP_JUNCTION },
    { "ride: containerstop->busstop",           GNE_TAG_RIDE_CONTAINERSTOP_BUSSTOP },
    { "ride: containerstop->trainstop",         GNE_TAG_RIDE_CONTAINERSTOP_TRAINSTOP },
    { "ride: containerstop->containerstop",     GNE_TAG_RIDE_CONTAINERSTOP_CONTAINERSTOP },
    { "ride: containerstop->chargingstation",   GNE_TAG_RIDE_CONTAINERSTOP_CHARGINGSTATION },
    { "ride: containerstop->parkingarea",       GNE_TAG_RIDE_CONTAINERSTOP_PARKINGAREA },
    { "ride: chargingstation->edge",            GNE_TAG_RIDE_CHARGINGSTATION_EDGE },
    { "ride: chargingstation->taz",             GNE_TAG_RIDE_CHARGINGSTATION_TAZ },
    { "ride: chargingstation->junction",        GNE_TAG_RIDE_CHARGINGSTATION_JUNCTION },
    { "ride: chargingstation->busstop",         GNE_TAG_RIDE_CHARGINGSTATION_BUSSTOP },
    { "ride: chargingstation->trainstop",       GNE_TAG_RIDE_CHARGINGSTATION_TRAINSTOP },
    { "ride: chargingstation->containestop",    GNE_TAG_RIDE_CHARGINGSTATION_CONTAINERSTOP },
    { "ride: chargingstation->chargingstation", GNE_TAG_RIDE_CHARGINGSTATION_CHARGINGSTATION },
    { "ride: chargingstation->parkingarea",     GNE_TAG_RIDE_CHARGINGSTATION_PARKINGAREA },
    { "ride: parkingarea->edge",                GNE_TAG_RIDE_PARKINGAREA_EDGE },
    { "ride: parkingarea->taz",                 GNE_TAG_RIDE_PARKINGAREA_TAZ },
    { "ride: parkingarea->junction",            GNE_TAG_RIDE_PARKINGAREA_JUNCTION },
    { "ride: parkingarea->busstop",             GNE_TAG_RIDE_PARKINGAREA_BUSSTOP },
    { "ride: parkingarea->trainstop",           GNE_TAG_RIDE_PARKINGAREA_TRAINSTOP },
    { "ride: parkingarea->containerstop",       GNE_TAG_RIDE_PARKINGAREA_CONTAINERSTOP },
    { "ride: parkingarea->chargingstation",     GNE_TAG_RIDE_PARKINGAREA_CHARGINGSTATION },
    { "ride: parkingarea->parkingarea",         GNE_TAG_RIDE_PARKINGAREA_PARKINGAREA },
    // GNE Person Stops
    { "stopPerson",                             GNE_TAG_STOPPERSON },
    { "stopPerson: edge",                       GNE_TAG_STOPPERSON_EDGE },
    { "stopPerson: busStop",                    GNE_TAG_STOPPERSON_BUSSTOP },
    { "stopPerson: trainStop",                  GNE_TAG_STOPPERSON_TRAINSTOP },
    { "stopPerson: containerStop",              GNE_TAG_STOPPERSON_CONTAINERSTOP },
    { "stopPerson: chargingStation",            GNE_TAG_STOPPERSON_CHARGINGSTATION },
    { "stopPerson: parkingArea",                GNE_TAG_STOPPERSON_PARKINGAREA },
    // GNE Transports
    { "transport: edge->edge",                       GNE_TAG_TRANSPORT_EDGE_EDGE },
    { "transport: edge->taz",                        GNE_TAG_TRANSPORT_EDGE_TAZ },
    { "transport: edge->junction",                   GNE_TAG_TRANSPORT_EDGE_JUNCTION },
    { "transport: edge->busstop",                    GNE_TAG_TRANSPORT_EDGE_BUSSTOP },
    { "transport: edge->trainstop",                  GNE_TAG_TRANSPORT_EDGE_TRAINSTOP },
    { "transport: edge->containerstop",              GNE_TAG_TRANSPORT_EDGE_CONTAINERSTOP },
    { "transport: edge->chargingstation",            GNE_TAG_TRANSPORT_EDGE_CHARGINGSTATION },
    { "transport: edge->parkingarea",                GNE_TAG_TRANSPORT_EDGE_PARKINGAREA },
    { "transport: taz->edge",                        GNE_TAG_TRANSPORT_TAZ_EDGE },
    { "transport: taz->taz",                         GNE_TAG_TRANSPORT_TAZ_TAZ },
    { "transport: taz->junction",                    GNE_TAG_TRANSPORT_TAZ_JUNCTION },
    { "transport: taz->busstop",                     GNE_TAG_TRANSPORT_TAZ_BUSSTOP },
    { "transport: taz->trainstop",                   GNE_TAG_TRANSPORT_TAZ_TRAINSTOP },
    { "transport: taz->containerstop",               GNE_TAG_TRANSPORT_TAZ_CONTAINERSTOP },
    { "transport: taz->chargingstation",             GNE_TAG_TRANSPORT_TAZ_CHARGINGSTATION },
    { "transport: taz->parkingarea",                 GNE_TAG_TRANSPORT_TAZ_PARKINGAREA },
    { "transport: junction->edge",                   GNE_TAG_TRANSPORT_JUNCTION_EDGE },
    { "transport: junction->taz",                    GNE_TAG_TRANSPORT_JUNCTION_TAZ },
    { "transport: junction->junction",               GNE_TAG_TRANSPORT_JUNCTION_JUNCTION },
    { "transport: junction->busstop",                GNE_TAG_TRANSPORT_JUNCTION_BUSSTOP },
    { "transport: junction->trainstop",              GNE_TAG_TRANSPORT_JUNCTION_TRAINSTOP },
    { "transport: junction->containerstop",          GNE_TAG_TRANSPORT_JUNCTION_CONTAINERSTOP },
    { "transport: junction->chargingstation",        GNE_TAG_TRANSPORT_JUNCTION_CHARGINGSTATION },
    { "transport: junction->parkingarea",            GNE_TAG_TRANSPORT_JUNCTION_PARKINGAREA },
    { "transport: busstop->edge",                    GNE_TAG_TRANSPORT_BUSSTOP_EDGE },
    { "transport: busstop->taz",                     GNE_TAG_TRANSPORT_BUSSTOP_TAZ },
    { "transport: busstop->junction",                GNE_TAG_TRANSPORT_BUSSTOP_JUNCTION },
    { "transport: busstop->busstop",                 GNE_TAG_TRANSPORT_BUSSTOP_BUSSTOP },
    { "transport: busstop->trainstop",               GNE_TAG_TRANSPORT_BUSSTOP_TRAINSTOP },
    { "transport: busstop->containerstop",           GNE_TAG_TRANSPORT_BUSSTOP_CONTAINERSTOP },
    { "transport: busstop->chargingstation",         GNE_TAG_TRANSPORT_BUSSTOP_CHARGINGSTATION },
    { "transport: busstop->parkingarea",             GNE_TAG_TRANSPORT_BUSSTOP_PARKINGAREA },
    { "transport: trainstop->edge",                  GNE_TAG_TRANSPORT_TRAINSTOP_EDGE },
    { "transport: trainstop->taz",                   GNE_TAG_TRANSPORT_TRAINSTOP_TAZ },
    { "transport: trainstop->junction",              GNE_TAG_TRANSPORT_TRAINSTOP_JUNCTION },
    { "transport: trainstop->busstop",               GNE_TAG_TRANSPORT_TRAINSTOP_BUSSTOP },
    { "transport: trainstop->trainstop",             GNE_TAG_TRANSPORT_TRAINSTOP_TRAINSTOP },
    { "transport: trainstop->containerstop",         GNE_TAG_TRANSPORT_TRAINSTOP_CONTAINERSTOP },
    { "transport: trainstop->chargingstation",       GNE_TAG_TRANSPORT_TRAINSTOP_CHARGINGSTATION },
    { "transport: trainstop->parkingarea",           GNE_TAG_TRANSPORT_TRAINSTOP_PARKINGAREA },
    { "transport: containerstop->edge",              GNE_TAG_TRANSPORT_CONTAINERSTOP_EDGE },
    { "transport: containerstop->taz",               GNE_TAG_TRANSPORT_CONTAINERSTOP_TAZ },
    { "transport: containerstop->junction",          GNE_TAG_TRANSPORT_CONTAINERSTOP_JUNCTION },
    { "transport: containerstop->busstop",           GNE_TAG_TRANSPORT_CONTAINERSTOP_BUSSTOP },
    { "transport: containerstop->trainstop",         GNE_TAG_TRANSPORT_CONTAINERSTOP_TRAINSTOP },
    { "transport: containerstop->containerstop",     GNE_TAG_TRANSPORT_CONTAINERSTOP_CONTAINERSTOP },
    { "transport: containerstop->chargingstation",   GNE_TAG_TRANSPORT_CONTAINERSTOP_CHARGINGSTATION },
    { "transport: containerstop->parkingarea",       GNE_TAG_TRANSPORT_CONTAINERSTOP_PARKINGAREA },
    { "transport: chargingstation->edge",            GNE_TAG_TRANSPORT_CHARGINGSTATION_EDGE },
    { "transport: chargingstation->taz",             GNE_TAG_TRANSPORT_CHARGINGSTATION_TAZ },
    { "transport: chargingstation->junction",        GNE_TAG_TRANSPORT_CHARGINGSTATION_JUNCTION },
    { "transport: chargingstation->busstop",         GNE_TAG_TRANSPORT_CHARGINGSTATION_BUSSTOP },
    { "transport: chargingstation->trainstop",       GNE_TAG_TRANSPORT_CHARGINGSTATION_TRAINSTOP },
    { "transport: chargingstation->containestop",    GNE_TAG_TRANSPORT_CHARGINGSTATION_CONTAINERSTOP },
    { "transport: chargingstation->chargingstation", GNE_TAG_TRANSPORT_CHARGINGSTATION_CHARGINGSTATION },
    { "transport: chargingstation->parkingarea",     GNE_TAG_TRANSPORT_CHARGINGSTATION_PARKINGAREA },
    { "transport: parkingarea->edge",                GNE_TAG_TRANSPORT_PARKINGAREA_EDGE },
    { "transport: parkingarea->taz",                 GNE_TAG_TRANSPORT_PARKINGAREA_TAZ },
    { "transport: parkingarea->junction",            GNE_TAG_TRANSPORT_PARKINGAREA_JUNCTION },
    { "transport: parkingarea->busstop",             GNE_TAG_TRANSPORT_PARKINGAREA_BUSSTOP },
    { "transport: parkingarea->trainstop",           GNE_TAG_TRANSPORT_PARKINGAREA_TRAINSTOP },
    { "transport: parkingarea->containerstop",       GNE_TAG_TRANSPORT_PARKINGAREA_CONTAINERSTOP },
    { "transport: parkingarea->chargingstation",     GNE_TAG_TRANSPORT_PARKINGAREA_CHARGINGSTATION },
    { "transport: parkingarea->parkingarea",         GNE_TAG_TRANSPORT_PARKINGAREA_PARKINGAREA },
    // GNE Tranships
    { "tranship: edge->edge",                       GNE_TAG_TRANSHIP_EDGE_EDGE },
    { "tranship: edge->taz",                        GNE_TAG_TRANSHIP_EDGE_TAZ },
    { "tranship: edge->junction",                   GNE_TAG_TRANSHIP_EDGE_JUNCTION },
    { "tranship: edge->busstop",                    GNE_TAG_TRANSHIP_EDGE_BUSSTOP },
    { "tranship: edge->trainstop",                  GNE_TAG_TRANSHIP_EDGE_TRAINSTOP },
    { "tranship: edge->containerstop",              GNE_TAG_TRANSHIP_EDGE_CONTAINERSTOP },
    { "tranship: edge->chargingstation",            GNE_TAG_TRANSHIP_EDGE_CHARGINGSTATION },
    { "tranship: edge->parkingarea",                GNE_TAG_TRANSHIP_EDGE_PARKINGAREA },
    { "tranship: taz->edge",                        GNE_TAG_TRANSHIP_TAZ_EDGE },
    { "tranship: taz->taz",                         GNE_TAG_TRANSHIP_TAZ_TAZ },
    { "tranship: taz->junction",                    GNE_TAG_TRANSHIP_TAZ_JUNCTION },
    { "tranship: taz->busstop",                     GNE_TAG_TRANSHIP_TAZ_BUSSTOP },
    { "tranship: taz->trainstop",                   GNE_TAG_TRANSHIP_TAZ_TRAINSTOP },
    { "tranship: taz->containerstop",               GNE_TAG_TRANSHIP_TAZ_CONTAINERSTOP },
    { "tranship: taz->chargingstation",             GNE_TAG_TRANSHIP_TAZ_CHARGINGSTATION },
    { "tranship: taz->parkingarea",                 GNE_TAG_TRANSHIP_TAZ_PARKINGAREA },
    { "tranship: junction->edge",                   GNE_TAG_TRANSHIP_JUNCTION_EDGE },
    { "tranship: junction->taz",                    GNE_TAG_TRANSHIP_JUNCTION_TAZ },
    { "tranship: junction->junction",               GNE_TAG_TRANSHIP_JUNCTION_JUNCTION },
    { "tranship: junction->busstop",                GNE_TAG_TRANSHIP_JUNCTION_BUSSTOP },
    { "tranship: junction->trainstop",              GNE_TAG_TRANSHIP_JUNCTION_TRAINSTOP },
    { "tranship: junction->containerstop",          GNE_TAG_TRANSHIP_JUNCTION_CONTAINERSTOP },
    { "tranship: junction->chargingstation",        GNE_TAG_TRANSHIP_JUNCTION_CHARGINGSTATION },
    { "tranship: junction->parkingarea",            GNE_TAG_TRANSHIP_JUNCTION_PARKINGAREA },
    { "tranship: busstop->edge",                    GNE_TAG_TRANSHIP_BUSSTOP_EDGE },
    { "tranship: busstop->taz",                     GNE_TAG_TRANSHIP_BUSSTOP_TAZ },
    { "tranship: busstop->junction",                GNE_TAG_TRANSHIP_BUSSTOP_JUNCTION },
    { "tranship: busstop->busstop",                 GNE_TAG_TRANSHIP_BUSSTOP_BUSSTOP },
    { "tranship: busstop->trainstop",               GNE_TAG_TRANSHIP_BUSSTOP_TRAINSTOP },
    { "tranship: busstop->containerstop",           GNE_TAG_TRANSHIP_BUSSTOP_CONTAINERSTOP },
    { "tranship: busstop->chargingstation",         GNE_TAG_TRANSHIP_BUSSTOP_CHARGINGSTATION },
    { "tranship: busstop->parkingarea",             GNE_TAG_TRANSHIP_BUSSTOP_PARKINGAREA },
    { "tranship: trainstop->edge",                  GNE_TAG_TRANSHIP_TRAINSTOP_EDGE },
    { "tranship: trainstop->taz",                   GNE_TAG_TRANSHIP_TRAINSTOP_TAZ },
    { "tranship: trainstop->junction",              GNE_TAG_TRANSHIP_TRAINSTOP_JUNCTION },
    { "tranship: trainstop->busstop",               GNE_TAG_TRANSHIP_TRAINSTOP_BUSSTOP },
    { "tranship: trainstop->trainstop",             GNE_TAG_TRANSHIP_TRAINSTOP_TRAINSTOP },
    { "tranship: trainstop->containerstop",         GNE_TAG_TRANSHIP_TRAINSTOP_CONTAINERSTOP },
    { "tranship: trainstop->chargingstation",       GNE_TAG_TRANSHIP_TRAINSTOP_CHARGINGSTATION },
    { "tranship: trainstop->parkingarea",           GNE_TAG_TRANSHIP_TRAINSTOP_PARKINGAREA },
    { "tranship: containerstop->edge",              GNE_TAG_TRANSHIP_CONTAINERSTOP_EDGE },
    { "tranship: containerstop->taz",               GNE_TAG_TRANSHIP_CONTAINERSTOP_TAZ },
    { "tranship: containerstop->junction",          GNE_TAG_TRANSHIP_CONTAINERSTOP_JUNCTION },
    { "tranship: containerstop->busstop",           GNE_TAG_TRANSHIP_CONTAINERSTOP_BUSSTOP },
    { "tranship: containerstop->trainstop",         GNE_TAG_TRANSHIP_CONTAINERSTOP_TRAINSTOP },
    { "tranship: containerstop->containerstop",     GNE_TAG_TRANSHIP_CONTAINERSTOP_CONTAINERSTOP },
    { "tranship: containerstop->chargingstation",   GNE_TAG_TRANSHIP_CONTAINERSTOP_CHARGINGSTATION },
    { "tranship: containerstop->parkingarea",       GNE_TAG_TRANSHIP_CONTAINERSTOP_PARKINGAREA },
    { "tranship: chargingstation->edge",            GNE_TAG_TRANSHIP_CHARGINGSTATION_EDGE },
    { "tranship: chargingstation->taz",             GNE_TAG_TRANSHIP_CHARGINGSTATION_TAZ },
    { "tranship: chargingstation->junction",        GNE_TAG_TRANSHIP_CHARGINGSTATION_JUNCTION },
    { "tranship: chargingstation->busstop",         GNE_TAG_TRANSHIP_CHARGINGSTATION_BUSSTOP },
    { "tranship: chargingstation->trainstop",       GNE_TAG_TRANSHIP_CHARGINGSTATION_TRAINSTOP },
    { "tranship: chargingstation->containestop",    GNE_TAG_TRANSHIP_CHARGINGSTATION_CONTAINERSTOP },
    { "tranship: chargingstation->chargingstation", GNE_TAG_TRANSHIP_CHARGINGSTATION_CHARGINGSTATION },
    { "tranship: chargingstation->parkingarea",     GNE_TAG_TRANSHIP_CHARGINGSTATION_PARKINGAREA },
    { "tranship: parkingarea->edge",                GNE_TAG_TRANSHIP_PARKINGAREA_EDGE },
    { "tranship: parkingarea->taz",                 GNE_TAG_TRANSHIP_PARKINGAREA_TAZ },
    { "tranship: parkingarea->junction",            GNE_TAG_TRANSHIP_PARKINGAREA_JUNCTION },
    { "tranship: parkingarea->busstop",             GNE_TAG_TRANSHIP_PARKINGAREA_BUSSTOP },
    { "tranship: parkingarea->trainstop",           GNE_TAG_TRANSHIP_PARKINGAREA_TRAINSTOP },
    { "tranship: parkingarea->containerstop",       GNE_TAG_TRANSHIP_PARKINGAREA_CONTAINERSTOP },
    { "tranship: parkingarea->chargingstation",     GNE_TAG_TRANSHIP_PARKINGAREA_CHARGINGSTATION },
    { "tranship: parkingarea->parkingarea",         GNE_TAG_TRANSHIP_PARKINGAREA_PARKINGAREA },
    { "tranship: edges",                            GNE_TAG_TRANSHIP_EDGES },
    // GNE Container Stops
    { "stopContainer",                          GNE_TAG_STOPCONTAINER },
    { "stopContainer: edge",                    GNE_TAG_STOPCONTAINER_EDGE },
    { "stopContainer: busStop",                 GNE_TAG_STOPCONTAINER_BUSSTOP },
    { "stopContainer: trainStop",               GNE_TAG_STOPCONTAINER_TRAINSTOP },
    { "stopContainer: containerStop",           GNE_TAG_STOPCONTAINER_CONTAINERSTOP },
    { "stopContainer: chargingStation",         GNE_TAG_STOPCONTAINER_CHARGINGSTATION },
    { "stopContainer: parkingArea",             GNE_TAG_STOPCONTAINER_PARKINGAREA },
    // root file
    { "rootFile",                               SUMO_TAG_ROOTFILE },
    // Last element
    { "",                                       SUMO_TAG_NOTHING }  // -> must be the last one
};


SequentialStringBijection::Entry SUMOXMLDefinitions::attrs[] = {
    // meta value for attribute enum
    { "default",                SUMO_ATTR_DEFAULT },
    // meandata
    { "sampledSeconds",         SUMO_ATTR_SAMPLEDSECONDS },
    { "density",                SUMO_ATTR_DENSITY },
    { "laneDensity",            SUMO_ATTR_LANEDENSITY },
    { "occupancy",              SUMO_ATTR_OCCUPANCY },
    { "waitingTime",            SUMO_ATTR_WAITINGTIME },
    { "timeLoss",               SUMO_ATTR_TIMELOSS },
    { "speed",                  SUMO_ATTR_SPEED },
    { "speedRelative",          SUMO_ATTR_SPEEDREL },
    { "departed",               SUMO_ATTR_DEPARTED },
    { "arrived",                SUMO_ATTR_ARRIVED },
    { "entered",                SUMO_ATTR_ENTERED },
    { "left",                   SUMO_ATTR_LEFT },
    { "vaporized",              SUMO_ATTR_VAPORIZED },
    { "teleported",             SUMO_ATTR_TELEPORTED },
    { "traveltime",             SUMO_ATTR_TRAVELTIME },
    { "laneChangedFrom",        SUMO_ATTR_LANECHANGEDFROM },
    { "laneChangedTo",          SUMO_ATTR_LANECHANGEDTO },
    { "overlapTraveltime",      SUMO_ATTR_OVERLAPTRAVELTIME },
    { "CO_abs",                 SUMO_ATTR_CO_ABS },
    { "CO2_abs",                SUMO_ATTR_CO2_ABS },
    { "HC_abs",                 SUMO_ATTR_HC_ABS },
    { "PMx_abs",                SUMO_ATTR_PMX_ABS },
    { "NOx_abs",                SUMO_ATTR_NOX_ABS },
    { "fuel_abs",               SUMO_ATTR_FUEL_ABS },
    { "electricity_abs",        SUMO_ATTR_ELECTRICITY_ABS },
    { "CO_normed",              SUMO_ATTR_CO_NORMED },
    { "CO2_normed",             SUMO_ATTR_CO2_NORMED },
    { "HC_normed",              SUMO_ATTR_HC_NORMED },
    { "PMx_normed",             SUMO_ATTR_PMX_NORMED },
    { "NOx_normed",             SUMO_ATTR_NOX_NORMED },
    { "fuel_normed",            SUMO_ATTR_FUEL_NORMED },
    { "electricity_normed",     SUMO_ATTR_ELECTRICITY_NORMED },
    { "CO_perVeh",              SUMO_ATTR_CO_PERVEH },
    { "CO2_perVeh",             SUMO_ATTR_CO2_PERVEH },
    { "HC_perVeh",              SUMO_ATTR_HC_PERVEH },
    { "PMx_perVeh",             SUMO_ATTR_PMX_PERVEH },
    { "NOx_perVeh",             SUMO_ATTR_NOX_PERVEH },
    { "fuel_perVeh",            SUMO_ATTR_FUEL_PERVEH },
    { "electricity_perVeh",     SUMO_ATTR_ELECTRICITY_PERVEH },
    { "noise",                  SUMO_ATTR_NOISE },
    { "amount",                 SUMO_ATTR_AMOUNT },
    { "averageSpeed",           SUMO_ATTR_AVERAGESPEED },
    // FCD
    { "x",                      SUMO_ATTR_X },
    { "y",                      SUMO_ATTR_Y },
    { "z",                      SUMO_ATTR_Z },
    { "angle",                  SUMO_ATTR_ANGLE },
    { "type",                   SUMO_ATTR_TYPE },
    { "pos",                    SUMO_ATTR_POSITION },
    { "edge",                   SUMO_ATTR_EDGE },
    { "lane",                   SUMO_ATTR_LANE },
    { "slope",                  SUMO_ATTR_SLOPE },
    { "signals",                SUMO_ATTR_SIGNALS },
    { "acceleration",           SUMO_ATTR_ACCELERATION },
    { "accelerationLat",        SUMO_ATTR_ACCELERATION_LAT },
    { "distance",               SUMO_ATTR_DISTANCE },
    { "leaderID",               SUMO_ATTR_LEADER_ID },
    { "leaderSpeed",            SUMO_ATTR_LEADER_SPEED },
    { "leaderGap",              SUMO_ATTR_LEADER_GAP },
    { "vehicle",                SUMO_ATTR_VEHICLE },
    { "odometer",               SUMO_ATTR_ODOMETER },
    { "posLat",                 SUMO_ATTR_POSITION_LAT },
    { "speedLat",               SUMO_ATTR_SPEED_LAT },
    // only usable with SumoXMLAttrMask
    { "arrivalDelay",           SUMO_ATTR_ARRIVALDELAY },
    // emission-output
    { "CO",                     SUMO_ATTR_CO },
    { "CO2",                    SUMO_ATTR_CO2 },
    { "HC",                     SUMO_ATTR_HC },
    { "PMx",                    SUMO_ATTR_PMX },
    { "NOx",                    SUMO_ATTR_NOX },
    { "fuel",                   SUMO_ATTR_FUEL },
    { "electricity",            SUMO_ATTR_ELECTRICITY },
    { "route",                  SUMO_ATTR_ROUTE },
    { "eclass",                 SUMO_ATTR_ECLASS },
    { "waiting",                SUMO_ATTR_WAITING },
    // Edge
    { "id",                     SUMO_ATTR_ID },
    { "refId",                  SUMO_ATTR_REFID },
    { "name",                   SUMO_ATTR_NAME },
    { "version",                SUMO_ATTR_VERSION },
    { "priority",               SUMO_ATTR_PRIORITY },
    { "numLanes",               SUMO_ATTR_NUMLANES },
    { "friction",               SUMO_ATTR_FRICTION },
    { "oneway",                 SUMO_ATTR_ONEWAY },
    { "width",                  SUMO_ATTR_WIDTH },
    { "widthResolution",        SUMO_ATTR_WIDTHRESOLUTION },
    { "maxWidth",               SUMO_ATTR_MAXWIDTH },
    { "minWidth",               SUMO_ATTR_MINWIDTH },
    { "sidewalkWidth",          SUMO_ATTR_SIDEWALKWIDTH },
    { "bikeLaneWidth",          SUMO_ATTR_BIKELANEWIDTH },
    { "remove",                 SUMO_ATTR_REMOVE },
    { "length",                 SUMO_ATTR_LENGTH },
    { "bidi",                   SUMO_ATTR_BIDI },
    // Split
    { "idBefore",               SUMO_ATTR_ID_BEFORE },
    { "idAfter",                SUMO_ATTR_ID_AFTER },
    // Positions
    { "center",                 SUMO_ATTR_CENTER },
    { "centerX",                SUMO_ATTR_CENTER_X },
    { "centerY",                SUMO_ATTR_CENTER_Y },
    { "centerZ",                SUMO_ATTR_CENTER_Z },

    { "key",                    SUMO_ATTR_KEY },
    { "requestSize",            SUMO_ATTR_REQUESTSIZE },
    { "request",                SUMO_ATTR_REQUEST },
    { "response",               SUMO_ATTR_RESPONSE },
    { "programID",              SUMO_ATTR_PROGRAMID },
    { "phase",                  SUMO_ATTR_PHASE },
    { "offset",                 SUMO_ATTR_OFFSET },
    { "endOffset",              SUMO_ATTR_ENDOFFSET },
    { "incLanes",               SUMO_ATTR_INCLANES },
    { "intLanes",               SUMO_ATTR_INTLANES },

    { "weight",                 SUMO_ATTR_WEIGHT },
    { "node",                   SUMO_ATTR_NODE },
    { "edges",                  SUMO_ATTR_EDGES },
    // Vehicle
    { "depart",                 SUMO_ATTR_DEPART },
    { "departLane",             SUMO_ATTR_DEPARTLANE },
    { "departPos",              SUMO_ATTR_DEPARTPOS },
    { "departPosLat",           SUMO_ATTR_DEPARTPOS_LAT },
    { "departSpeed",            SUMO_ATTR_DEPARTSPEED },
    { "departEdge",             SUMO_ATTR_DEPARTEDGE },
    { "arrivalLane",            SUMO_ATTR_ARRIVALLANE },
    { "arrivalPos",             SUMO_ATTR_ARRIVALPOS },
    { "arrivalPosLat",          SUMO_ATTR_ARRIVALPOS_LAT },
    { "arrivalSpeed",           SUMO_ATTR_ARRIVALSPEED },
    { "arrivalEdge",            SUMO_ATTR_ARRIVALEDGE },
    { "maxSpeed",               SUMO_ATTR_MAXSPEED },
    { "desiredMaxSpeed",        SUMO_ATTR_DESIRED_MAXSPEED },
    { "maxSpeedLat",            SUMO_ATTR_MAXSPEED_LAT },
    { "latAlignment",           SUMO_ATTR_LATALIGNMENT },
    { "minGapLat",              SUMO_ATTR_MINGAP_LAT },
    { "accel",                  SUMO_ATTR_ACCEL },
    { "decel",                  SUMO_ATTR_DECEL },
    { "emergencyDecel",         SUMO_ATTR_EMERGENCYDECEL },
    { "apparentDecel",          SUMO_ATTR_APPARENTDECEL },
    { "actionStepLength",       SUMO_ATTR_ACTIONSTEPLENGTH },
    { "vClass",                 SUMO_ATTR_VCLASS },
    { "vClasses",               SUMO_ATTR_VCLASSES },
    { "exceptions",             SUMO_ATTR_EXCEPTIONS },
    { "repno",                  SUMO_ATTR_REPNUMBER },
    { "speedFactor",            SUMO_ATTR_SPEEDFACTOR },
    { "speedDev",               SUMO_ATTR_SPEEDDEV },
    { "laneChangeModel",        SUMO_ATTR_LANE_CHANGE_MODEL },
    { "carFollowModel",         SUMO_ATTR_CAR_FOLLOW_MODEL },
    { "minGap",                 SUMO_ATTR_MINGAP },
    { "collisionMinGapFactor",  SUMO_ATTR_COLLISION_MINGAP_FACTOR },
    { "boardingDuration",       SUMO_ATTR_BOARDING_DURATION },
    { "loadingDuration",        SUMO_ATTR_LOADING_DURATION },
    { "boardingFactor",         SUMO_ATTR_BOARDING_FACTOR },
    { "scale",                  SUMO_ATTR_SCALE },
    { "insertionChecks",        SUMO_ATTR_INSERTIONCHECKS },
    { "timeToTeleport",         SUMO_ATTR_TIME_TO_TELEPORT },
    { "timeToTeleportBidi",     SUMO_ATTR_TIME_TO_TELEPORT_BIDI },
    { "speedFactorPremature",   SUMO_ATTR_SPEEDFACTOR_PREMATURE },
    { "maneuverAngleTimes",     SUMO_ATTR_MANEUVER_ANGLE_TIMES },
    { "parkingBadges",          SUMO_ATTR_PARKING_BADGES },
    // MSDevice_ElecHybrid
    { "overheadWireChargingPower",      SUMO_ATTR_OVERHEADWIRECHARGINGPOWER },
    // OverheadWire
    { "overheadWireSegment",    SUMO_ATTR_OVERHEAD_WIRE_SEGMENT },
    { "segments",               SUMO_ATTR_OVERHEAD_WIRE_SEGMENTS },     // <- deprecate
    { "voltage",                SUMO_ATTR_VOLTAGE },
    { "voltageSource",          SUMO_ATTR_VOLTAGESOURCE },
    { "currentLimit",           SUMO_ATTR_CURRENTLIMIT },
    { "substationId",           SUMO_ATTR_SUBSTATIONID },
    { "wireResistivity",        SUMO_ATTR_OVERHEAD_WIRE_RESISTIVITY },
    { "forbiddenInnerLanes",    SUMO_ATTR_OVERHEAD_WIRE_FORBIDDEN },
    { "clamps",                 SUMO_ATTR_OVERHEAD_WIRE_CLAMPS },       // <- deprecate
    { "idSegmentStartClamp",    SUMO_ATTR_OVERHEAD_WIRE_CLAMP_START },  // <- deprecate
    { "idSegmentEndClamp",      SUMO_ATTR_OVERHEAD_WIRE_CLAMP_END },    // <- deprecate
    { "wireClampStart",         SUMO_ATTR_OVERHEAD_WIRECLAMP_START },
    { "wireClampEnd",           SUMO_ATTR_OVERHEAD_WIRECLAMP_END },
    { "wireClampLaneStart",     SUMO_ATTR_OVERHEAD_WIRECLAMP_LANESTART },
    { "wireClampLaneEnd",       SUMO_ATTR_OVERHEAD_WIRECLAMP_LANEEND },
    // Charging Station
    { "power",                  SUMO_ATTR_CHARGINGPOWER },
    { "efficiency",             SUMO_ATTR_EFFICIENCY },
    { "chargeInTransit",        SUMO_ATTR_CHARGEINTRANSIT },
    { "chargeDelay",            SUMO_ATTR_CHARGEDELAY},
    { "chargeType",             SUMO_ATTR_CHARGETYPE},
    // MSDevice_Battery
    { "actualBatteryCapacity",          SUMO_ATTR_ACTUALBATTERYCAPACITY },
    { "chargeLevel",                    SUMO_ATTR_CHARGELEVEL },
    { "maximumBatteryCapacity",         SUMO_ATTR_MAXIMUMBATTERYCAPACITY },
    { "maximumChargeRate",              SUMO_ATTR_MAXIMUMCHARGERATE },
    { "chargeLevelTable",               SUMO_ATTR_CHARGELEVELTABLE },
    { "chargeCurveTable",               SUMO_ATTR_CHARGECURVETABLE },
    { "maximumPower",                   SUMO_ATTR_MAXIMUMPOWER },
    { "vehicleMass",                    SUMO_ATTR_VEHICLEMASS },
    { "rotatingMass",                   SUMO_ATTR_ROTATINGMASS },
    { "frontSurfaceArea",               SUMO_ATTR_FRONTSURFACEAREA },
    { "airDragCoefficient",             SUMO_ATTR_AIRDRAGCOEFFICIENT },
    { "internalMomentOfInertia",        SUMO_ATTR_INTERNALMOMENTOFINERTIA },
    { "radialDragCoefficient",          SUMO_ATTR_RADIALDRAGCOEFFICIENT },
    { "rollDragCoefficient",            SUMO_ATTR_ROLLDRAGCOEFFICIENT },
    { "constantPowerIntake",            SUMO_ATTR_CONSTANTPOWERINTAKE },
    { "propulsionEfficiency",           SUMO_ATTR_PROPULSIONEFFICIENCY },
    { "recuperationEfficiency",         SUMO_ATTR_RECUPERATIONEFFICIENCY },
    { "recuperationEfficiencyByDecel",  SUMO_ATTR_RECUPERATIONEFFICIENCY_BY_DECELERATION },
    { "stoppingTreshold",               SUMO_ATTR_STOPPINGTHRESHOLD },
    // MSDevice_Tripinfo
    { "waitingCount",                   SUMO_ATTR_WAITINGCOUNT },
    { "stopTime",                       SUMO_ATTR_STOPTIME },

    // MSElecHybridExport
    { "overheadWireId",         SUMO_ATTR_OVERHEADWIREID },
    { "tractionSubstationId",   SUMO_ATTR_TRACTIONSUBSTATIONID },
    { "current",                SUMO_ATTR_CURRENTFROMOVERHEADWIRE },
    { "circuitVoltage",         SUMO_ATTR_VOLTAGEOFOVERHEADWIRE },
    { "alphaCircuitSolver",     SUMO_ATTR_ALPHACIRCUITSOLVER },
    { "recuperationEnabled",     SUMO_ATTR_RECUPERATIONENABLE },
    // MSBatteryExport
    { "energyConsumed",         SUMO_ATTR_ENERGYCONSUMED },
    { "totalEnergyConsumed",    SUMO_ATTR_TOTALENERGYCONSUMED },
    { "totalEnergyRegenerated", SUMO_ATTR_TOTALENERGYREGENERATED },
    { "chargingStationId",      SUMO_ATTR_CHARGINGSTATIONID },
    { "energyCharged",          SUMO_ATTR_ENERGYCHARGED },
    { "energyChargedInTransit", SUMO_ATTR_ENERGYCHARGEDINTRANSIT },
    { "energyChargedStopped",   SUMO_ATTR_ENERGYCHARGEDSTOPPED },
    { "posOnLane",              SUMO_ATTR_POSONLANE },
    { "timeStopped",            SUMO_ATTR_TIMESTOPPED },
    // MMPEVEM
    { "wheelRadius",               SUMO_ATTR_WHEELRADIUS },
    { "maximumTorque",             SUMO_ATTR_MAXIMUMTORQUE },
    { "maximumSpeed",              SUMO_ATTR_MAXIMUMSPEED },
    { "gearEfficiency",            SUMO_ATTR_GEAREFFICIENCY },
    { "gearRatio",                 SUMO_ATTR_GEARRATIO },
    { "maximumRecuperationTorque", SUMO_ATTR_MAXIMUMRECUPERATIONTORQUE },
    { "maximumRecuperationPower",  SUMO_ATTR_MAXIMUMRECUPERATIONPOWER },
    { "internalBatteryResistance", SUMO_ATTR_INTERNALBATTERYRESISTANCE },
    { "nominalBatteryVoltage",     SUMO_ATTR_NOMINALBATTERYVOLTAGE },
    { "powerLossMap",              SUMO_ATTR_POWERLOSSMAP },
    // ChargingStation output
    { "chargingStatus",                 SUMO_ATTR_CHARGING_STATUS },
    { "totalEnergyCharged",             SUMO_ATTR_TOTALENERGYCHARGED },
    { "chargingSteps",                  SUMO_ATTR_CHARGINGSTEPS },
    { "totalEnergyChargedIntoVehicle",  SUMO_ATTR_TOTALENERGYCHARGED_VEHICLE },
    { "chargingBegin",                  SUMO_ATTR_CHARGINGBEGIN },
    { "chargingEnd",                    SUMO_ATTR_CHARGINGEND },
    { "partialCharge",                  SUMO_ATTR_PARTIALCHARGE },
    { "minPower",                       SUMO_ATTR_MINPOWER },
    { "minCharge",                      SUMO_ATTR_MINCHARGE },
    { "maxCharge",                      SUMO_ATTR_MAXCHARGE },
    { "minEfficiency",                  SUMO_ATTR_MINEFFICIENCY },
    { "maxEfficiency",                  SUMO_ATTR_MAXEFFICIENCY },

    // general emission / consumption
    { "shutOffStopDuration",    SUMO_ATTR_SHUT_OFF_STOP },
    { "shutOffAutoDuration",    SUMO_ATTR_SHUT_OFF_AUTO },
    { "loading",                SUMO_ATTR_LOADING },

    /// @name carFollow model attributes
    { "sigma",                  SUMO_ATTR_SIGMA },
    { "sigmaStep",              SUMO_ATTR_SIGMA_STEP },
    { "startupDelay",           SUMO_ATTR_STARTUP_DELAY },
    { "tau",                    SUMO_ATTR_TAU },
    { "tmp1",                   SUMO_ATTR_TMP1 },
    { "tmp2",                   SUMO_ATTR_TMP2 },
    { "tmp3",                   SUMO_ATTR_TMP3 },
    { "tmp4",                   SUMO_ATTR_TMP4 },
    { "tmp5",                   SUMO_ATTR_TMP5 },

    { "vehdynamics",            SUMO_ATTR_CF_EIDM_USEVEHDYNAMICS },
    { "tpreview",               SUMO_ATTR_CF_EIDM_T_LOOK_AHEAD },
    { "tPersDrive",             SUMO_ATTR_CF_EIDM_T_PERSISTENCE_DRIVE },
    { "treaction",              SUMO_ATTR_CF_EIDM_T_REACTION },
    { "tPersEstimate",          SUMO_ATTR_CF_EIDM_T_PERSISTENCE_ESTIMATE },
    { "ccoolness",              SUMO_ATTR_CF_EIDM_C_COOLNESS },
    { "sigmaleader",            SUMO_ATTR_CF_EIDM_SIG_LEADER },
    { "sigmagap",               SUMO_ATTR_CF_EIDM_SIG_GAP },
    { "sigmaerror",             SUMO_ATTR_CF_EIDM_SIG_ERROR },
    { "jerkmax",                SUMO_ATTR_CF_EIDM_JERK_MAX },
    { "epsilonacc",             SUMO_ATTR_CF_EIDM_EPSILON_ACC },
    { "taccmax",                SUMO_ATTR_CF_EIDM_T_ACC_MAX },
    { "Mflatness",              SUMO_ATTR_CF_EIDM_M_FLATNESS },
    { "Mbegin",                 SUMO_ATTR_CF_EIDM_M_BEGIN },
    { "maxvehpreview",          SUMO_ATTR_CF_EIDM_MAX_VEH_PREVIEW },

    { "speedControlGain",               SUMO_ATTR_SC_GAIN },
    { "gapClosingControlGainSpeed",     SUMO_ATTR_GCC_GAIN_SPEED },
    { "gapClosingControlGainSpace",     SUMO_ATTR_GCC_GAIN_SPACE },
    { "gapControlGainSpeed",            SUMO_ATTR_GC_GAIN_SPEED },
    { "gapControlGainSpace",            SUMO_ATTR_GC_GAIN_SPACE },
    { "collisionAvoidanceGainSpeed",    SUMO_ATTR_CA_GAIN_SPEED },
    { "collisionAvoidanceGainSpace",    SUMO_ATTR_CA_GAIN_SPACE },
    { "collisionAvoidanceOverride",     SUMO_ATTR_CA_OVERRIDE },

    { "speedControlGainCACC",           SUMO_ATTR_SC_GAIN_CACC },
    { "gapClosingControlGainGap",       SUMO_ATTR_GCC_GAIN_GAP_CACC },
    { "gapClosingControlGainGapDot",    SUMO_ATTR_GCC_GAIN_GAP_DOT_CACC },
    { "gapControlGainGap",              SUMO_ATTR_GC_GAIN_GAP_CACC },
    { "gapControlGainGapDot",           SUMO_ATTR_GC_GAIN_GAP_DOT_CACC },
    { "collisionAvoidanceGainGap",      SUMO_ATTR_CA_GAIN_GAP_CACC },
    { "collisionAvoidanceGainGapDot",   SUMO_ATTR_CA_GAIN_GAP_DOT_CACC },
    { "tauCACCToACC",                   SUMO_ATTR_HEADWAY_TIME_CACC_TO_ACC },
    { "speedControlMinGap",             SUMO_ATTR_SC_MIN_GAP },
    { "applyDriverState",               SUMO_ATTR_APPLYDRIVERSTATE },

    { "trainType",                      SUMO_ATTR_TRAIN_TYPE },
    { "speedTable",                     SUMO_ATTR_SPEED_TABLE },
    { "tractionTable",                  SUMO_ATTR_TRACTION_TABLE },
    { "resistanceTable",                SUMO_ATTR_RESISTANCE_TABLE },
    { "massFactor",                     SUMO_ATTR_MASSFACTOR },
    { "maxPower",                       SUMO_ATTR_MAXPOWER },
    { "maxTraction",                    SUMO_ATTR_MAXTRACTION },
    { "resCoef_constant",               SUMO_ATTR_RESISTANCE_COEFFICIENT_CONSTANT },
    { "resCoef_linear",                 SUMO_ATTR_RESISTANCE_COEFFICIENT_LINEAR },
    { "resCoef_quadratic",              SUMO_ATTR_RESISTANCE_COEFFICIENT_QUADRATIC },
    /// @}

    /// @name Lane changing model attributes
    /// @{
    { "lcStrategic",                SUMO_ATTR_LCA_STRATEGIC_PARAM },
    { "lcCooperative",              SUMO_ATTR_LCA_COOPERATIVE_PARAM },
    { "lcSpeedGain",                SUMO_ATTR_LCA_SPEEDGAIN_PARAM },
    { "lcKeepRight",                SUMO_ATTR_LCA_KEEPRIGHT_PARAM },
    { "lcSublane",                  SUMO_ATTR_LCA_SUBLANE_PARAM },
    { "lcOpposite",                 SUMO_ATTR_LCA_OPPOSITE_PARAM },
    { "lcPushy",                    SUMO_ATTR_LCA_PUSHY },
    { "lcPushyGap",                 SUMO_ATTR_LCA_PUSHYGAP },
    { "lcAssertive",                SUMO_ATTR_LCA_ASSERTIVE },
    { "lcImpatience",               SUMO_ATTR_LCA_IMPATIENCE },
    { "lcTimeToImpatience",         SUMO_ATTR_LCA_TIME_TO_IMPATIENCE },
    { "lcAccelLat",                 SUMO_ATTR_LCA_ACCEL_LAT },
    { "lcTurnAlignmentDistance",    SUMO_ATTR_LCA_TURN_ALIGNMENT_DISTANCE },
    { "lcOvertakeRight",            SUMO_ATTR_LCA_OVERTAKE_RIGHT },
    { "lcLookaheadLeft",            SUMO_ATTR_LCA_LOOKAHEADLEFT },
    { "lcSpeedGainRight",           SUMO_ATTR_LCA_SPEEDGAINRIGHT },
    { "lcSpeedGainLookahead",       SUMO_ATTR_LCA_SPEEDGAIN_LOOKAHEAD },
    { "lcCooperativeRoundabout",    SUMO_ATTR_LCA_COOPERATIVE_ROUNDABOUT },
    { "lcCooperativeSpeed",         SUMO_ATTR_LCA_COOPERATIVE_SPEED },
    { "lcMaxSpeedLatStanding",      SUMO_ATTR_LCA_MAXSPEEDLATSTANDING },
    { "lcMaxSpeedLatFactor",        SUMO_ATTR_LCA_MAXSPEEDLATFACTOR },
    { "lcMaxDistLatStanding",       SUMO_ATTR_LCA_MAXDISTLATSTANDING },
    { "lcLaneDiscipline",           SUMO_ATTR_LCA_LANE_DISCIPLINE },
    { "lcSigma",                    SUMO_ATTR_LCA_SIGMA },
    { "lcKeepRightAcceptanceTime",  SUMO_ATTR_LCA_KEEPRIGHT_ACCEPTANCE_TIME },
    { "lcOvertakeDeltaSpeedFactor", SUMO_ATTR_LCA_OVERTAKE_DELTASPEED_FACTOR },
    { "lcContRight",                SUMO_ATTR_LCA_CONTRIGHT },
    { "lcExperimental1",            SUMO_ATTR_LCA_EXPERIMENTAL1 },
    /// @}

    /// @name junction model attributes
    /// @{
    { "jmCrossingGap",          SUMO_ATTR_JM_CROSSING_GAP },
    { "jmDriveAfterYellowTime", SUMO_ATTR_JM_DRIVE_AFTER_YELLOW_TIME },
    { "jmDriveAfterRedTime",    SUMO_ATTR_JM_DRIVE_AFTER_RED_TIME },
    { "jmDriveRedSpeed",        SUMO_ATTR_JM_DRIVE_RED_SPEED },
    { "jmIgnoreKeepClearTime",  SUMO_ATTR_JM_IGNORE_KEEPCLEAR_TIME },
    { "jmIgnoreFoeSpeed",       SUMO_ATTR_JM_IGNORE_FOE_SPEED },
    { "jmIgnoreFoeProb",        SUMO_ATTR_JM_IGNORE_FOE_PROB },
    { "jmIgnoreJunctionFoeProb", SUMO_ATTR_JM_IGNORE_JUNCTION_FOE_PROB },
    { "jmSigmaMinor",           SUMO_ATTR_JM_SIGMA_MINOR },
    { "jmStoplineGap",          SUMO_ATTR_JM_STOPLINE_GAP },
    { "jmStoplineCrossingGap",  SUMO_ATTR_JM_STOPLINE_CROSSING_GAP },
    { "jmTimegapMinor",         SUMO_ATTR_JM_TIMEGAP_MINOR },
    { "jmExtraGap",             SUMO_ATTR_JM_EXTRA_GAP },
    { "jmAdvance",              SUMO_ATTR_JM_ADVANCE },
    { "jmStopSignWait",         SUMO_ATTR_JM_STOPSIGN_WAIT },
    { "jmAllwayStopWait",       SUMO_ATTR_JM_ALLWAYSTOP_WAIT },
    { "junctionModel.ignoreIDs",    SUMO_ATTR_JM_IGNORE_IDS },
    { "junctionModel.ignoreTypes",  SUMO_ATTR_JM_IGNORE_TYPES },
    { "carFollowModel.ignoreIDs",   SUMO_ATTR_CF_IGNORE_IDS },
    { "carFollowModel.ignoreTypes", SUMO_ATTR_CF_IGNORE_TYPES },
    /// @}
    { "flexArrival", SUMO_ATTR_FLEX_ARRIVAL },

    { "last",                   SUMO_ATTR_LAST },
    { "cost",                   SUMO_ATTR_COST },
    { "costs",                  SUMO_ATTR_COSTS },
    { "savings",                SUMO_ATTR_SAVINGS },
    { "exitTimes",              SUMO_ATTR_EXITTIMES },
    { "probability",            SUMO_ATTR_PROB },
    { "replacedAtTime",         SUMO_ATTR_REPLACED_AT_TIME },
    { "replacedOnIndex",        SUMO_ATTR_REPLACED_ON_INDEX },
    { "count",                  SUMO_ATTR_COUNT },
    { "probabilities",          SUMO_ATTR_PROBS },
    { "routes",                 SUMO_ATTR_ROUTES },
    { "vTypes",                 SUMO_ATTR_VTYPES },
    { "nextEdges",              SUMO_ATTR_NEXT_EDGES },
    { "deterministic",          SUMO_ATTR_DETERMINISTIC },

    { "lanes",                  SUMO_ATTR_LANES },
    { "from",                   SUMO_ATTR_FROM },
    { "to",                     SUMO_ATTR_TO },
    { "fromLonLat",             SUMO_ATTR_FROMLONLAT },
    { "toLonLat",               SUMO_ATTR_TOLONLAT },
    { "fromXY",                 SUMO_ATTR_FROMXY },
    { "toXY",                   SUMO_ATTR_TOXY },
    { "fromJunction",           SUMO_ATTR_FROM_JUNCTION },
    { "toJunction",             SUMO_ATTR_TO_JUNCTION },
    { "period",                 SUMO_ATTR_PERIOD },
    { "repeat",                 SUMO_ATTR_REPEAT },
    { "cycleTime",              SUMO_ATTR_CYCLETIME },
    { "fromTaz",                SUMO_ATTR_FROM_TAZ },
    { "toTaz",                  SUMO_ATTR_TO_TAZ },
    { "reroute",                SUMO_ATTR_REROUTE },
    { "personCapacity",         SUMO_ATTR_PERSON_CAPACITY },
    { "containerCapacity",      SUMO_ATTR_CONTAINER_CAPACITY },
    { "parkingLength",          SUMO_ATTR_PARKING_LENGTH },
    { "personNumber",           SUMO_ATTR_PERSON_NUMBER },
    { "containerNumber",        SUMO_ATTR_CONTAINER_NUMBER },
    { "modes",                  SUMO_ATTR_MODES },
    { "walkFactor",             SUMO_ATTR_WALKFACTOR },

    { "function",               SUMO_ATTR_FUNCTION },
    { "freq",                   SUMO_ATTR_FREQUENCY },
    { "style",                  SUMO_ATTR_STYLE },
    { "file",                   SUMO_ATTR_FILE },
    { "local",                  SUMO_ATTR_LOCAL },
    { "number",                 SUMO_ATTR_NUMBER },
    { "duration",               SUMO_ATTR_DURATION },
    { "until",                  SUMO_ATTR_UNTIL },
    { "arrival",                SUMO_ATTR_ARRIVAL },
    { "extension",              SUMO_ATTR_EXTENSION },
    { "started",                SUMO_ATTR_STARTED },
    { "ended",                  SUMO_ATTR_ENDED },
    { "routeProbe",             SUMO_ATTR_ROUTEPROBE },
    { "crossingEdges",          SUMO_ATTR_CROSSING_EDGES },
    // Traffic light & Nodes
    { "time",                   SUMO_ATTR_TIME },
    { "begin",                  SUMO_ATTR_BEGIN },
    { "end",                    SUMO_ATTR_END },
    { "tl",                     SUMO_ATTR_TLID },
    { "tlType",                 SUMO_ATTR_TLTYPE },
    { "tlLayout",               SUMO_ATTR_TLLAYOUT },
    { "linkIndex",              SUMO_ATTR_TLLINKINDEX },
    { "linkIndex2",             SUMO_ATTR_TLLINKINDEX2 },
    { "shape",                  SUMO_ATTR_SHAPE },
    { "outlineShape",           SUMO_ATTR_OUTLINESHAPE },
    { "spreadType",             SUMO_ATTR_SPREADTYPE },
    { "radius",                 SUMO_ATTR_RADIUS },
    { "customShape",            SUMO_ATTR_CUSTOMSHAPE },
    { "keepClear",              SUMO_ATTR_KEEP_CLEAR },
    { "indirect",               SUMO_ATTR_INDIRECT },
    { "rightOfWay",             SUMO_ATTR_RIGHT_OF_WAY },
    { "fringe",                 SUMO_ATTR_FRINGE },
    { "color",                  SUMO_ATTR_COLOR },
    { "dir",                    SUMO_ATTR_DIR },
    { "state",                  SUMO_ATTR_STATE },
    { "lcState",                SUMO_ATTR_LCSTATE },
    { "icon",                   SUMO_ATTR_ICON },
    { "layer",                  SUMO_ATTR_LAYER },
    { "fill",                   SUMO_ATTR_FILL },
    { "lineWidth",              SUMO_ATTR_LINEWIDTH },
    { "prefix",                 SUMO_ATTR_PREFIX },
    { "discard",                SUMO_ATTR_DISCARD },

    { "fromLane",               SUMO_ATTR_FROM_LANE },
    { "toLane",                 SUMO_ATTR_TO_LANE },
    { "dest",                   SUMO_ATTR_DEST },
    { "source",                 SUMO_ATTR_SOURCE },
    { "via",                    SUMO_ATTR_VIA },
    { "viaLonLat",              SUMO_ATTR_VIALONLAT },
    { "viaXY",                  SUMO_ATTR_VIAXY },
    { "viaJunctions",           SUMO_ATTR_VIAJUNCTIONS },
    { "nodes",                  SUMO_ATTR_NODES },
    { "visibility",             SUMO_ATTR_VISIBILITY_DISTANCE },

    { "minDur",                 SUMO_ATTR_MINDURATION },
    { "maxDur",                 SUMO_ATTR_MAXDURATION },
    { "earliestEnd",            SUMO_ATTR_EARLIEST_END },
    { "latestEnd",              SUMO_ATTR_LATEST_END },
    { "earlyTarget",            SUMO_ATTR_EARLY_TARGET },
    { "finalTarget",            SUMO_ATTR_FINAL_TARGET },
    { "check",                  SUMO_ATTR_CHECK },
    { "nArgs",                  SUMO_ATTR_NARGS },
    { "vehext",                 SUMO_ATTR_VEHICLEEXTENSION },
    { "yellow",                 SUMO_ATTR_YELLOW },
    { "red",                    SUMO_ATTR_RED },
    { "next",                   SUMO_ATTR_NEXT },
    { "foes",                   SUMO_ATTR_FOES },
    { "constraints",            SUMO_ATTR_CONSTRAINTS },
    { "rail",                   SUMO_ATTR_RAIL },
    { "detectors",              SUMO_ATTR_DETECTORS },
    { "conditions",             SUMO_ATTR_CONDITIONS },
    { "saveDetectors",          SUMO_ATTR_SAVE_DETECTORS },
    { "saveConditions",         SUMO_ATTR_SAVE_CONDITIONS },

    // E2 detector
    { "cont",                   SUMO_ATTR_CONT },
    { "contPos",                SUMO_ATTR_CONTPOS },
    { "timeThreshold",          SUMO_ATTR_HALTING_TIME_THRESHOLD },
    { "speedThreshold",         SUMO_ATTR_HALTING_SPEED_THRESHOLD },
    { "jamThreshold",           SUMO_ATTR_JAM_DIST_THRESHOLD },
    { "show",                   SUMO_ATTR_SHOW_DETECTOR },
    // E3 detector
    { "openEntry",              SUMO_ATTR_OPEN_ENTRY },
    { "expectArrival",          SUMO_ATTR_EXPECT_ARRIVAL },

    { "wautID",                 SUMO_ATTR_WAUT_ID },
    { "junctionID",             SUMO_ATTR_JUNCTION_ID },
    { "procedure",              SUMO_ATTR_PROCEDURE },
    { "synchron",               SUMO_ATTR_SYNCHRON },
    { "refTime",                SUMO_ATTR_REF_TIME },
    { "startProg",              SUMO_ATTR_START_PROG },
    { "off",                    SUMO_ATTR_OFF },
    { "friendlyPos",            SUMO_ATTR_FRIENDLY_POS },
    { "splitByType",            SUMO_ATTR_SPLIT_VTYPE },
    { "uncontrolled",           SUMO_ATTR_UNCONTROLLED },
    { "pass",                   SUMO_ATTR_PASS },
    { "busStop",                SUMO_ATTR_BUS_STOP },
    { "trainStop",              SUMO_ATTR_TRAIN_STOP },
    { "containerStop",          SUMO_ATTR_CONTAINER_STOP },
    { "parkingArea",            SUMO_ATTR_PARKING_AREA },
    { "roadsideCapacity",       SUMO_ATTR_ROADSIDE_CAPACITY },
    { "acceptedBadges",         SUMO_ATTR_ACCEPTED_BADGES },
    { "onRoad",                 SUMO_ATTR_ONROAD },
    { "chargingStation",        SUMO_ATTR_CHARGING_STATION },
    { "group",                  SUMO_ATTR_GROUP },
    { "line",                   SUMO_ATTR_LINE },
    { "lines",                  SUMO_ATTR_LINES },
    { "tripId",                 SUMO_ATTR_TRIP_ID },
    { "split",                  SUMO_ATTR_SPLIT },
    { "join",                   SUMO_ATTR_JOIN },
    { "intended",               SUMO_ATTR_INTENDED },
    { "onDemand",               SUMO_ATTR_ONDEMAND },
    { "jump",                   SUMO_ATTR_JUMP },
    { "usedEnded",              SUMO_ATTR_USED_ENDED },
    { "collision",              SUMO_ATTR_COLLISION },
    { "value",                  SUMO_ATTR_VALUE },
    { "prohibitor",             SUMO_ATTR_PROHIBITOR },
    { "prohibited",             SUMO_ATTR_PROHIBITED },
    { "allow",                  SUMO_ATTR_ALLOW },
    { "disallow",               SUMO_ATTR_DISALLOW },
    { "prefer",                 SUMO_ATTR_PREFER },
    { "changeLeft",             SUMO_ATTR_CHANGE_LEFT },
    { "changeRight",            SUMO_ATTR_CHANGE_RIGHT },
    { "controlledInner",        SUMO_ATTR_CONTROLLED_INNER },
    { "vehsPerHour",            SUMO_ATTR_VEHSPERHOUR },
    { "personsPerHour",         SUMO_ATTR_PERSONSPERHOUR },
    { "containersPerHour",		SUMO_ATTR_CONTAINERSPERHOUR },
    { "perHour",				SUMO_ATTR_PERHOUR },
    { "done",	  			    SUMO_ATTR_DONE },
    { "output",                 SUMO_ATTR_OUTPUT },
    { "height",                 SUMO_ATTR_HEIGHT },
    { "guiShape",               SUMO_ATTR_GUISHAPE },
    { "osgFile",                SUMO_ATTR_OSGFILE },
    { "imgFile",                SUMO_ATTR_IMGFILE },
    { "relativePath",           SUMO_ATTR_RELATIVEPATH },
    { "emissionClass",          SUMO_ATTR_EMISSIONCLASS },
    { "mass",                   SUMO_ATTR_MASS },
    { "impatience",             SUMO_ATTR_IMPATIENCE },
    { "startPos",               SUMO_ATTR_STARTPOS },
    { "endPos",                 SUMO_ATTR_ENDPOS },
    { "triggered",              SUMO_ATTR_TRIGGERED },
    { "containerTriggered",     SUMO_ATTR_CONTAINER_TRIGGERED },
    { "parking",                SUMO_ATTR_PARKING },
    { "expected",               SUMO_ATTR_EXPECTED },
    { "permitted",              SUMO_ATTR_PERMITTED },
    { "expectedContainers",     SUMO_ATTR_EXPECTED_CONTAINERS },
    { "index",                  SUMO_ATTR_INDEX },

    { "entering",               SUMO_ATTR_ENTERING },
    { "excludeEmpty",           SUMO_ATTR_EXCLUDE_EMPTY },
    { "withInternal",           SUMO_ATTR_WITH_INTERNAL },
    { "trackVehicles",          SUMO_ATTR_TRACK_VEHICLES },
    { "detectPersons",          SUMO_ATTR_DETECT_PERSONS },
    { "maxTraveltime",          SUMO_ATTR_MAX_TRAVELTIME },
    { "minSamples",             SUMO_ATTR_MIN_SAMPLES },
    { "writeAttributes",        SUMO_ATTR_WRITE_ATTRIBUTES },
    { "edgesFile",              SUMO_ATTR_EDGESFILE },
    { "aggregate",              SUMO_ATTR_AGGREGATE },
    { "numEdges",               SUMO_ATTR_NUMEDGES },

    { "lon",                    SUMO_ATTR_LON },
    { "lat",                    SUMO_ATTR_LAT },
    { "action",                 SUMO_ATTR_ACTION },
    { "geo",                    SUMO_ATTR_GEO },
    { "geoShape",               SUMO_ATTR_GEOSHAPE },
    { "k",                      SUMO_ATTR_K },
    { "v",                      SUMO_ATTR_V },
    { "ref",                    SUMO_ATTR_REF },
    { "href",                   SUMO_ATTR_HREF },
    { "zoom",                   SUMO_ATTR_ZOOM },
    { "interpolated",           SUMO_ATTR_INTERPOLATED },
    { "threshold",              SUMO_ATTR_THRESHOLD },

    { "netOffset",              SUMO_ATTR_NET_OFFSET },
    { "convBoundary",           SUMO_ATTR_CONV_BOUNDARY },
    { "origBoundary",           SUMO_ATTR_ORIG_BOUNDARY },
    { "projParameter",          SUMO_ATTR_ORIG_PROJ },

    { "tauLast",                SUMO_ATTR_CF_PWAGNER2009_TAULAST },
    { "apProb",                 SUMO_ATTR_CF_PWAGNER2009_APPROB },
    { "delta",                  SUMO_ATTR_CF_IDM_DELTA },
    { "stepping",               SUMO_ATTR_CF_IDM_STEPPING },
    { "adaptFactor",            SUMO_ATTR_CF_IDMM_ADAPT_FACTOR },
    { "adaptTime",              SUMO_ATTR_CF_IDMM_ADAPT_TIME },
    { "phi",                    SUMO_ATTR_CF_KERNER_PHI },

    { "security",               SUMO_ATTR_CF_WIEDEMANN_SECURITY },
    { "estimation",             SUMO_ATTR_CF_WIEDEMANN_ESTIMATION },
    { "cc1",                    SUMO_ATTR_CF_W99_CC1 },
    { "cc2",                    SUMO_ATTR_CF_W99_CC2 },
    { "cc3",                    SUMO_ATTR_CF_W99_CC3 },
    { "cc4",                    SUMO_ATTR_CF_W99_CC4 },
    { "cc5",                    SUMO_ATTR_CF_W99_CC5 },
    { "cc6",                    SUMO_ATTR_CF_W99_CC6 },
    { "cc7",                    SUMO_ATTR_CF_W99_CC7 },
    { "cc8",                    SUMO_ATTR_CF_W99_CC8 },
    { "cc9",                    SUMO_ATTR_CF_W99_CC9 },

    { "ccDecel",                SUMO_ATTR_CF_CC_CCDECEL },
    { "constSpacing",           SUMO_ATTR_CF_CC_CONSTSPACING },
    { "kp",                     SUMO_ATTR_CF_CC_KP },
    { "lambda",                 SUMO_ATTR_CF_CC_LAMBDA },
    { "c1",                     SUMO_ATTR_CF_CC_C1 },
    { "xi",                     SUMO_ATTR_CF_CC_XI },
    { "omegaN",                 SUMO_ATTR_CF_CC_OMEGAN },
    { "tauEngine",              SUMO_ATTR_CF_CC_TAU },
    { "lanesCount",             SUMO_ATTR_CF_CC_LANES_COUNT },
    { "ccAccel",                SUMO_ATTR_CF_CC_CCACCEL },
    { "ploegKp",                SUMO_ATTR_CF_CC_PLOEG_KP },
    { "ploegKd",                SUMO_ATTR_CF_CC_PLOEG_KD },
    { "ploegH",                 SUMO_ATTR_CF_CC_PLOEG_H },
    { "flatbedKa",              SUMO_ATTR_CF_CC_FLATBED_KA },
    { "flatbedKv",              SUMO_ATTR_CF_CC_FLATBED_KV },
    { "flatbedKp",              SUMO_ATTR_CF_CC_FLATBED_KP },
    { "flatbedD",               SUMO_ATTR_CF_CC_FLATBED_D },
    { "flatbedH",               SUMO_ATTR_CF_CC_FLATBED_H },

    { "generateWalks",          SUMO_ATTR_GENERATE_WALKS },
    { "actType",                SUMO_ATTR_ACTTYPE },
    { "junctionCornerDetail",   SUMO_ATTR_CORNERDETAIL },
    { "junctionLinkDetail",     SUMO_ATTR_LINKDETAIL },
    { "rectangularLaneCut",     SUMO_ATTR_RECTANGULAR_LANE_CUT },
    { "walkingareas",           SUMO_ATTR_WALKINGAREAS },
    { "lefthand",               SUMO_ATTR_LEFTHAND },
    { "limitTurnSpeed",         SUMO_ATTR_LIMIT_TURN_SPEED },
    { "checkLaneFoesAll",       SUMO_ATTR_CHECKLANEFOES_ALL },
    { "checkLaneFoesRoundabout", SUMO_ATTR_CHECKLANEFOES_ROUNDABOUT },
    { "tlsIgnoreInternalJunctionJam", SUMO_ATTR_TLS_IGNORE_INTERNAL_JUNCTION_JAM },
    { "avoidOverlap",           SUMO_ATTR_AVOID_OVERLAP },
    { "junctionHigherSpeed",    SUMO_ATTR_HIGHER_SPEED },
    { "internalJunctionsVehicleWidth", SUMO_ATTR_INTERNAL_JUNCTIONS_VEHICLE_WIDTH },
    { "junctionsMinimalShape",  SUMO_ATTR_JUNCTIONS_MINIMAL_SHAPE },
    { "junctionsEndpointShape", SUMO_ATTR_JUNCTIONS_ENDPOINT_SHAPE },

    { "actorConfig",            SUMO_ATTR_ACTORCONFIG },
    { "startTime",              SUMO_ATTR_STARTTIME },
    { "vehicleClass",           SUMO_ATTR_VEHICLECLASS },
    { "origin",                 SUMO_ATTR_ORIGIN },
    { "destination",            SUMO_ATTR_DESTINATION },
    { "visible",                SUMO_ATTR_VISIBLE },
    { "limit",                  SUMO_ATTR_LIMIT },
    { "active",                 SUMO_ATTR_ACTIVE },
    { "arrivalTime",            SUMO_ATTR_ARRIVALTIME },
    { "arrivalTimeBraking",     SUMO_ATTR_ARRIVALTIMEBRAKING },
    { "arrivalSpeedBraking",    SUMO_ATTR_ARRIVALSPEEDBRAKING },
    { "optional",               SUMO_ATTR_OPTIONAL },
    { "vehicles",               SUMO_ATTR_VEHICLES },

#ifndef WIN32
    { "commandPosix",   SUMO_ATTR_COMMAND },
#else
    { "commandWindows", SUMO_ATTR_COMMAND },
#endif

    // ActivityGen statistics file
    { "inhabitants",            AGEN_ATTR_INHABITANTS },
    { "households",             AGEN_ATTR_HOUSEHOLDS },
    { "childrenAgeLimit",       AGEN_ATTR_CHILDREN },
    { "retirementAgeLimit",     AGEN_ATTR_RETIREMENT },
    { "carRate",                AGEN_ATTR_CARS },
    { "unemploymentRate",       AGEN_ATTR_UNEMPLOYEMENT },
    { "laborDemand",            AGEN_ATTR_LABORDEMAND },
    { "footDistanceLimit",      AGEN_ATTR_MAX_FOOT_DIST },
    { "incomingTraffic",        AGEN_ATTR_IN_TRAFFIC },
    { "incoming",               AGEN_ATTR_INCOMING },
    { "outgoingTraffic",        AGEN_ATTR_OUT_TRAFFIC },
    { "outgoing",               AGEN_ATTR_OUTGOING },
    { "population",             AGEN_ATTR_POPULATION },
    { "workPosition",           AGEN_ATTR_OUT_WORKPOSITION },
    { "hour",                   AGEN_ATTR_HOUR },
    { "proportion",             AGEN_ATTR_PROP },
    { "capacity",               AGEN_ATTR_CAPACITY },
    { "opening",                AGEN_ATTR_OPENING },
    { "closing",                AGEN_ATTR_CLOSING },
    { "maxTripDuration",        AGEN_ATTR_MAX_TRIP_DURATION },
    { "rate",                   AGEN_ATTR_RATE },
    { "beginAge",               AGEN_ATTR_BEGINAGE },
    { "endAge",                 AGEN_ATTR_ENDAGE },
    { "peopleNbr",              AGEN_ATTR_PEOPLENBR },
    { "carPreference",          AGEN_ATTR_CARPREF },
    { "meanTimePerKmInCity",    AGEN_ATTR_CITYSPEED },
    { "freeTimeActivityRate",   AGEN_ATTR_FREETIMERATE },
    { "uniformRandomTraffic",   AGEN_ATTR_UNI_RAND_TRAFFIC },
    { "departureVariation",     AGEN_ATTR_DEP_VARIATION },

    // SUMOConfig files
    { "config-file",        SUMO_ATTR_CONFIGFILE },
    { "net-file",           SUMO_ATTR_NETFILE },
    { "additional-files",   SUMO_ATTR_ADDITIONALFILES },
    { "route-files",        SUMO_ATTR_ROUTEFILES },
    { "data-files",         SUMO_ATTR_DATAFILES },
    { "meandata-files",     SUMO_ATTR_MEANDATAFILES },

    // Netedit attributes
    { "selected",                           GNE_ATTR_SELECTED },
    { "modificationStatusNotForPrinting",   GNE_ATTR_MODIFICATION_STATUS },
    { "shapeStart",                         GNE_ATTR_SHAPE_START },
    { "shapeEnd",                           GNE_ATTR_SHAPE_END },
    { "isBidi",                             GNE_ATTR_BIDIR },
    { "closedShape",                        GNE_ATTR_CLOSE_SHAPE },
    { "parentItem",                         GNE_ATTR_PARENT },
    { "dataSet",                            GNE_ATTR_DATASET },
    { "genericParameter",                   GNE_ATTR_PARAMETERS },
    { "flowParameter",                      GNE_ATTR_FLOWPARAMETERS },
    { "defaultVTypeModified",               GNE_ATTR_DEFAULT_VTYPE_MODIFIED },
    { "centerAfterCreation",                GNE_ATTR_CENTER_AFTER_CREATION },
    { "opposite",                           GNE_ATTR_OPPOSITE },
    { "shiftLaneIndex",                     GNE_ATTR_SHIFTLANEINDEX },
    { "stopOffset",                         GNE_ATTR_STOPOFFSET },
    { "stopOException",                     GNE_ATTR_STOPOEXCEPTION },
    { "routeDist.",                         GNE_ATTR_ROUTE_DISTRIBUTION },
    { "typeDist.",                          GNE_ATTR_VTYPE_DISTRIBUTION },
    { "poisson",                            GNE_ATTR_POISSON },
    { "stopIndex",                          GNE_ATTR_STOPINDEX },
    { "pathStopIndex",                      GNE_ATTR_PATHSTOPINDEX },
    { "additionalChildren",                 GNE_ATTR_ADDITIONALCHILDREN },
    { "planGeometryStartPos",               GNE_ATTR_PLAN_GEOMETRY_STARTPOS },
    { "planGeometryEndPos",                 GNE_ATTR_PLAN_GEOMETRY_ENDPOS },
    { "fromLaneID",                         GNE_ATTR_FROM_LANEID },
    { "toLaneID",                           GNE_ATTR_TO_LANEID },
    { "tazCentroid",                        GNE_ATTR_TAZ_CENTROID },
    // mapped to additional elements on writing
    { "fromBusStop",                        GNE_ATTR_FROM_BUSSTOP },
    { "fromTrainStop",                      GNE_ATTR_FROM_TRAINSTOP },
    { "fromContainerStop",                  GNE_ATTR_FROM_CONTAINERSTOP },
    { "fromChargingStation",                GNE_ATTR_FROM_CHARGINGSTATION },
    { "fromParkingArea",                    GNE_ATTR_FROM_PARKINGAREA },
    { "fromRoute",                          GNE_ATTR_FROM_ROUTE },
    { "isRoundabout",                       GNE_ATTR_IS_ROUNDABOUT },

    { "carriageLength",     SUMO_ATTR_CARRIAGE_LENGTH },
    { "locomotiveLength",   SUMO_ATTR_LOCOMOTIVE_LENGTH },
    { "carriageGap",        SUMO_ATTR_CARRIAGE_GAP },
    { "carriageDoors",      SUMO_ATTR_CARRIAGE_DOORS },

    { "targetLanes",    SUMO_ATTR_TARGETLANE },
    { "crossing",       SUMO_ATTR_CROSSING },

    { "xmlns:xsi",                      SUMO_ATTR_XMLNS },
    { "xsi:noNamespaceSchemaLocation",  SUMO_ATTR_SCHEMA_LOCATION },

    //@name RNG state saving attributes
    // @{
    { "routeHandler",      SUMO_ATTR_RNG_ROUTEHANDLER },
    { "insertionControl",  SUMO_ATTR_RNG_INSERTIONCONTROL },
    { "device",            SUMO_ATTR_RNG_DEVICE },
    { "device.btreceiver", SUMO_ATTR_RNG_DEVICE_BT },
    { "device.toc",        SUMO_ATTR_RNG_DEVICE_TOC },
    { "driverState",       SUMO_ATTR_RNG_DRIVERSTATE },
    // @}

    //@name meso edge type attributes
    // @{
    { "tauff",             SUMO_ATTR_MESO_TAUFF },
    { "taufj",             SUMO_ATTR_MESO_TAUFJ },
    { "taujf",             SUMO_ATTR_MESO_TAUJF },
    { "taujj",             SUMO_ATTR_MESO_TAUJJ },
    { "junctionControl",   SUMO_ATTR_MESO_JUNCTION_CONTROL },
    { "tlsPenalty",        SUMO_ATTR_MESO_TLS_PENALTY },
    { "tlsFlowPenalty",    SUMO_ATTR_MESO_TLS_FLOW_PENALTY },
    { "minorPenalty",      SUMO_ATTR_MESO_MINOR_PENALTY },
    { "overtaking",        SUMO_ATTR_MESO_OVERTAKING },
    // @}

    // Other
    { "",                       SUMO_ATTR_NOTHING } //< must be the last one
};


const std::string SUMO_PARAM_ORIGID("origId");


StringBijection<SumoXMLNodeType>::Entry SUMOXMLDefinitions::sumoNodeTypeValues[] = {
    {"traffic_light",               SumoXMLNodeType::TRAFFIC_LIGHT},
    {"traffic_light_unregulated",   SumoXMLNodeType::TRAFFIC_LIGHT_NOJUNCTION},
    {"traffic_light_right_on_red",  SumoXMLNodeType::TRAFFIC_LIGHT_RIGHT_ON_RED},
    {"rail_signal",                 SumoXMLNodeType::RAIL_SIGNAL},
    {"rail_crossing",               SumoXMLNodeType::RAIL_CROSSING},
    {"priority",                    SumoXMLNodeType::PRIORITY},
    {"priority_stop",               SumoXMLNodeType::PRIORITY_STOP},
    {"right_before_left",           SumoXMLNodeType::RIGHT_BEFORE_LEFT},
    {"left_before_right",           SumoXMLNodeType::LEFT_BEFORE_RIGHT},
    {"allway_stop",                 SumoXMLNodeType::ALLWAY_STOP},
    {"zipper",                      SumoXMLNodeType::ZIPPER},
    {"district",                    SumoXMLNodeType::DISTRICT},
    {"unregulated",                 SumoXMLNodeType::NOJUNCTION},
    {"internal",                    SumoXMLNodeType::INTERNAL},
    {"dead_end",                    SumoXMLNodeType::DEAD_END},
    {"DEAD_END",                    SumoXMLNodeType::DEAD_END_DEPRECATED},
    {"unknown",                     SumoXMLNodeType::UNKNOWN} //< must be the last one
};


StringBijection<SumoXMLEdgeFunc>::Entry SUMOXMLDefinitions::sumoEdgeFuncValues[] = {
    {"unknown",     SumoXMLEdgeFunc::UNKNOWN},
    {"normal",      SumoXMLEdgeFunc::NORMAL},
    {"connector",   SumoXMLEdgeFunc::CONNECTOR},
    {"crossing",    SumoXMLEdgeFunc::CROSSING},
    {"walkingarea", SumoXMLEdgeFunc::WALKINGAREA},
    {"internal",    SumoXMLEdgeFunc::INTERNAL} //< must be the last one
};


StringBijection<LaneSpreadFunction>::Entry SUMOXMLDefinitions::laneSpreadFunctionValues[] = {
    {"right",      LaneSpreadFunction::RIGHT }, // default: geometry is left edge border, lanes flare to the right
    {"roadCenter", LaneSpreadFunction::ROADCENTER }, // geometry is center of the bidirectional road
    {"center",     LaneSpreadFunction::CENTER } // geometry is center of the edge (must be the last one)
};

StringBijection<ParkingType>::Entry SUMOXMLDefinitions::parkingTypeValues[] = {
    {"0",              ParkingType::ONROAD },   // default: park on the street
    {"1",              ParkingType::OFFROAD },    // parking off the street
    {"opportunistic",  ParkingType::OPPORTUNISTIC } // park of the street if there is an opportunity for it
};

StringBijection<RightOfWay>::Entry SUMOXMLDefinitions::rightOfWayValuesInitializer[] = {
    {"edgePriority",  RightOfWay::EDGEPRIORITY }, // use only edge priority values
    {"mixedPriority", RightOfWay::MIXEDPRIORITY }, // use the default behavior but encode this explicitly (only needed for overriding the NEMA fallback behavior)
    {"allwayStop",    RightOfWay::ALLWAYSTOP }, // only used for setting the fall-back behavior of TLS-off
    {"default",       RightOfWay::DEFAULT } // default (must be the last one)
};

StringBijection<FringeType>::Entry SUMOXMLDefinitions::fringeTypeValuesInitializer[] = {
    {"outer",   FringeType::OUTER },
    {"inner",   FringeType::INNER },
    {"default", FringeType::DEFAULT } // default (must be the last one)
};

StringBijection<PersonMode>::Entry SUMOXMLDefinitions::personModeValuesInitializer[] = {
    {"none",         PersonMode::NONE},
    {"walkForward",  PersonMode::WALK_FORWARD},
    {"walkBackward", PersonMode::WALK_BACKWARD},
    {"walk",         PersonMode::WALK},
    {"bicycle",      PersonMode::BICYCLE },
    {"car",          PersonMode::CAR },
    {"taxi",         PersonMode::TAXI },
    {"public",       PersonMode::PUBLIC} // (must be the last one)
};

StringBijection<LinkState>::Entry SUMOXMLDefinitions::linkStateValues[] = {
    { "G", LINKSTATE_TL_GREEN_MAJOR },
    { "g", LINKSTATE_TL_GREEN_MINOR },
    { "r", LINKSTATE_TL_RED },
    { "u", LINKSTATE_TL_REDYELLOW },
    { "Y", LINKSTATE_TL_YELLOW_MAJOR },
    { "y", LINKSTATE_TL_YELLOW_MINOR },
    { "o", LINKSTATE_TL_OFF_BLINKING },
    { "O", LINKSTATE_TL_OFF_NOSIGNAL },
    { "M", LINKSTATE_MAJOR },
    { "m", LINKSTATE_MINOR },
    { "=", LINKSTATE_EQUAL },
    { "s", LINKSTATE_STOP },
    { "w", LINKSTATE_ALLWAY_STOP },
    { "Z", LINKSTATE_ZIPPER },
    { "-", LINKSTATE_DEADEND } //< must be the last one
};

const char SUMOXMLDefinitions::AllowedTLS_linkStatesInitializer[] = {
    LINKSTATE_TL_GREEN_MAJOR,
    LINKSTATE_TL_GREEN_MINOR,
    LINKSTATE_STOP, // used for SumoXMLNodeType::TRAFFIC_LIGHT_RIGHT_ON_RED
    LINKSTATE_TL_RED,
    LINKSTATE_TL_REDYELLOW,
    LINKSTATE_TL_YELLOW_MAJOR,
    LINKSTATE_TL_YELLOW_MINOR,
    LINKSTATE_TL_OFF_BLINKING,
    LINKSTATE_TL_OFF_NOSIGNAL
};
const std::string SUMOXMLDefinitions::ALLOWED_TLS_LINKSTATES(AllowedTLS_linkStatesInitializer, 9);

StringBijection<LinkDirection>::Entry SUMOXMLDefinitions::linkDirectionValues[] = {
    { "s",      LinkDirection::STRAIGHT },
    { "t",      LinkDirection::TURN },
    { "T",      LinkDirection::TURN_LEFTHAND },
    { "l",      LinkDirection::LEFT },
    { "r",      LinkDirection::RIGHT },
    { "L",      LinkDirection::PARTLEFT },
    { "R",      LinkDirection::PARTRIGHT },
    { "invalid", LinkDirection::NODIR } //< must be the last one
};


StringBijection<TrafficLightType>::Entry SUMOXMLDefinitions::trafficLightTypesValues[] = {
    { "static",         TrafficLightType::STATIC },
    { "railSignal",     TrafficLightType::RAIL_SIGNAL },
    { "railCrossing",   TrafficLightType::RAIL_CROSSING },
    { "actuated",       TrafficLightType::ACTUATED },
    { "NEMA",           TrafficLightType::NEMA },
    { "delay_based",    TrafficLightType::DELAYBASED },
    { "sotl_phase",     TrafficLightType::SOTL_PHASE },
    { "sotl_platoon",   TrafficLightType::SOTL_PLATOON },
    { "sotl_request",   TrafficLightType::SOTL_REQUEST },
    { "sotl_wave",      TrafficLightType::SOTL_WAVE },
    { "sotl_marching",  TrafficLightType::SOTL_MARCHING },
    { "swarm",          TrafficLightType::SWARM_BASED },
    { "deterministic",  TrafficLightType::HILVL_DETERMINISTIC },
    { "off",            TrafficLightType::OFF },
    { "<invalid>",      TrafficLightType::INVALID } //< must be the last one
};


StringBijection<TrafficLightLayout>::Entry SUMOXMLDefinitions::trafficLightLayoutValues[] = {
    { "opposites",       TrafficLightLayout::OPPOSITES },
    { "incoming",        TrafficLightLayout::INCOMING },
    { "alternateOneWay", TrafficLightLayout::ALTERNATE_ONEWAY },
    { "default",         TrafficLightLayout::DEFAULT } //< must be the last one
};

StringBijection<InsertionCheck>::Entry SUMOXMLDefinitions::insertionCheckValues[] = {
    { "none",          InsertionCheck::NONE },
    { "collision",     InsertionCheck::COLLISION },
    { "leaderGap",     InsertionCheck::LEADER_GAP },
    { "followerGap",   InsertionCheck::FOLLOWER_GAP },
    { "junction",      InsertionCheck::JUNCTION },
    { "stop",          InsertionCheck::STOP },
    { "arrivalSpeed",  InsertionCheck::ARRIVAL_SPEED },
    { "oncomingTrain", InsertionCheck::ONCOMING_TRAIN },
    { "speedLimit",    InsertionCheck::SPEED_LIMIT },
    { "pedestrian",    InsertionCheck::PEDESTRIAN },
    { "bidi",          InsertionCheck::BIDI },
    { "laneChange",    InsertionCheck::LANECHANGE },
    { "all",           InsertionCheck::ALL } //< must be the last one
};

StringBijection<LaneChangeModel>::Entry SUMOXMLDefinitions::laneChangeModelValues[] = {
    { "DK2008",     LaneChangeModel::DK2008 },
    { "LC2013",     LaneChangeModel::LC2013 },
    { "LC2013_CC",  LaneChangeModel::LC2013_CC },
    { "SL2015",     LaneChangeModel::SL2015 },
    { "default",    LaneChangeModel::DEFAULT } //< must be the last one
};

StringBijection<SumoXMLTag>::Entry SUMOXMLDefinitions::carFollowModelValues[] = {
    { "IDM",         SUMO_TAG_CF_IDM },
    { "IDMM",        SUMO_TAG_CF_IDMM },
    { "Krauss",      SUMO_TAG_CF_KRAUSS },
    { "KraussPS",    SUMO_TAG_CF_KRAUSS_PLUS_SLOPE },
    { "KraussOrig1", SUMO_TAG_CF_KRAUSS_ORIG1 },
    { "KraussX",     SUMO_TAG_CF_KRAUSSX }, // experimental extensions to the Krauss model
    { "EIDM",        SUMO_TAG_CF_EIDM },
    { "SmartSK",     SUMO_TAG_CF_SMART_SK },
    { "Daniel1",     SUMO_TAG_CF_DANIEL1 },
    { "PWagner2009", SUMO_TAG_CF_PWAGNER2009 },
    { "BKerner",     SUMO_TAG_CF_BKERNER },
    { "Rail",        SUMO_TAG_CF_RAIL },
    { "CC",          SUMO_TAG_CF_CC },
    { "ACC",         SUMO_TAG_CF_ACC },
    { "CACC",        SUMO_TAG_CF_CACC },
    { "W99",         SUMO_TAG_CF_W99 },
    { "Wiedemann",   SUMO_TAG_CF_WIEDEMANN } //< must be the last one
};

StringBijection<LaneChangeAction>::Entry SUMOXMLDefinitions::laneChangeActionValues[] = {
    { "stay",        LCA_STAY },
    { "left",        LCA_LEFT },
    { "right",       LCA_RIGHT },
    { "strategic",   LCA_STRATEGIC },
    { "cooperative", LCA_COOPERATIVE },
    { "speedGain",   LCA_SPEEDGAIN },
    { "keepRight",   LCA_KEEPRIGHT },
    { "sublane",     LCA_SUBLANE },
    { "traci",       LCA_TRACI },
    { "urgent",      LCA_URGENT },
    { "overlapping", LCA_OVERLAPPING },
    { "blocked",     LCA_BLOCKED },
    { "amBL",        LCA_AMBLOCKINGLEADER },
    { "amBF",        LCA_AMBLOCKINGFOLLOWER },
    { "amBB",        LCA_AMBACKBLOCKER },
    { "amBBS",       LCA_AMBACKBLOCKER_STANDING },
    { "MR",          LCA_MRIGHT },
    { "ML",          LCA_MLEFT },

    { "unknown",     LCA_UNKNOWN } //< must be the last one
};

StringBijection<TrainType>::Entry SUMOXMLDefinitions::trainTypeValues[] = {
    { "custom",     TrainType::CUSTOM },
    { "NGT400",     TrainType::NGT400 },
    { "NGT400_16",  TrainType::NGT400_16 },
    { "RB425",      TrainType::RB425 },
    { "RB628",      TrainType::RB628 },
    { "ICE1",       TrainType::ICE1 },
    { "REDosto7",   TrainType::REDOSTO7 },
    { "Freight",    TrainType::FREIGHT },
    { "ICE3",       TrainType::ICE3 },
    { "MireoPlusB", TrainType::MIREOPLUSB },
    { "MireoPlusH", TrainType::MIREOPLUSH }
};

StringBijection<POIIcon>::Entry SUMOXMLDefinitions::POIIconValues[] = {
    {"pin",              POIIcon::PIN},
    {"tree",             POIIcon::NATURE},
    {"hotel",            POIIcon::HOTEL},
    {"fuel",             POIIcon::FUEL},
    {"charging_station", POIIcon::CHARGING_STATION},
    {"",                 POIIcon::NONE} //< must be the last one
};

SequentialStringBijection SUMOXMLDefinitions::Tags(
    SUMOXMLDefinitions::tags, SUMO_TAG_NOTHING);

SequentialStringBijection SUMOXMLDefinitions::Attrs(
    SUMOXMLDefinitions::attrs, SUMO_ATTR_NOTHING);

StringBijection<SumoXMLNodeType> SUMOXMLDefinitions::NodeTypes(
    SUMOXMLDefinitions::sumoNodeTypeValues, SumoXMLNodeType::UNKNOWN);

StringBijection<SumoXMLEdgeFunc> SUMOXMLDefinitions::EdgeFunctions(
    SUMOXMLDefinitions::sumoEdgeFuncValues, SumoXMLEdgeFunc::INTERNAL);

StringBijection<LaneSpreadFunction> SUMOXMLDefinitions::LaneSpreadFunctions(
    SUMOXMLDefinitions::laneSpreadFunctionValues, LaneSpreadFunction::CENTER);

StringBijection<ParkingType> SUMOXMLDefinitions::ParkingTypes(
    SUMOXMLDefinitions::parkingTypeValues, ParkingType::OPPORTUNISTIC);

StringBijection<RightOfWay> SUMOXMLDefinitions::RightOfWayValues(
    SUMOXMLDefinitions::rightOfWayValuesInitializer, RightOfWay::DEFAULT);

StringBijection<FringeType> SUMOXMLDefinitions::FringeTypeValues(
    SUMOXMLDefinitions::fringeTypeValuesInitializer, FringeType::DEFAULT);

StringBijection<PersonMode> SUMOXMLDefinitions::PersonModeValues(
    SUMOXMLDefinitions::personModeValuesInitializer, PersonMode::PUBLIC);

StringBijection<LinkState> SUMOXMLDefinitions::LinkStates(
    SUMOXMLDefinitions::linkStateValues, LINKSTATE_DEADEND);

StringBijection<LinkDirection> SUMOXMLDefinitions::LinkDirections(
    SUMOXMLDefinitions::linkDirectionValues, LinkDirection::NODIR);

StringBijection<TrafficLightType> SUMOXMLDefinitions::TrafficLightTypes(
    SUMOXMLDefinitions::trafficLightTypesValues, TrafficLightType::INVALID);

StringBijection<TrafficLightLayout> SUMOXMLDefinitions::TrafficLightLayouts(
    SUMOXMLDefinitions::trafficLightLayoutValues, TrafficLightLayout::DEFAULT);

StringBijection<InsertionCheck> SUMOXMLDefinitions::InsertionChecks(
    SUMOXMLDefinitions::insertionCheckValues, InsertionCheck::ALL);

StringBijection<LaneChangeModel> SUMOXMLDefinitions::LaneChangeModels(
    SUMOXMLDefinitions::laneChangeModelValues, LaneChangeModel::DEFAULT);

StringBijection<SumoXMLTag> SUMOXMLDefinitions::CarFollowModels(
    SUMOXMLDefinitions::carFollowModelValues, SUMO_TAG_CF_WIEDEMANN);

StringBijection<LaneChangeAction> SUMOXMLDefinitions::LaneChangeActions(
    SUMOXMLDefinitions::laneChangeActionValues, LCA_UNKNOWN);

StringBijection<TrainType> SUMOXMLDefinitions::TrainTypes(
    SUMOXMLDefinitions::trainTypeValues, TrainType::MIREOPLUSH);

StringBijection<POIIcon> SUMOXMLDefinitions::POIIcons(
    SUMOXMLDefinitions::POIIconValues, POIIcon::NONE, false);


std::string
SUMOXMLDefinitions::getJunctionIDFromInternalEdge(const std::string internalEdge) {
    assert(internalEdge[0] == ':');
    return internalEdge.substr(1, internalEdge.rfind('_') - 1);
}


std::string
SUMOXMLDefinitions::getEdgeIDFromLane(const std::string laneID) {
    return laneID.substr(0, laneID.rfind('_'));
}

int
SUMOXMLDefinitions::getIndexFromLane(const std::string laneID) {
    return StringUtils::toInt(laneID.substr(laneID.rfind('_') + 1));
}


bool
SUMOXMLDefinitions::isValidNetID(const std::string& value) {
    return (value.size() > 0) && value.find_first_of(" \t\n\r|\\'\";,<>&") == std::string::npos && value[0] != ':';
}


bool
SUMOXMLDefinitions::isValidVehicleID(const std::string& value) {
    return (value.size() > 0) && value.find_first_of(" \t\n\r|\\'\";,<>&") == std::string::npos;
}


bool
SUMOXMLDefinitions::isValidTypeID(const std::string& value) {
    return (value.size() > 0) && value.find_first_of(" \t\n\r|\\'\";,<>&") == std::string::npos;
}

bool
SUMOXMLDefinitions::isValidAdditionalID(const std::string& value) {
    return (value.size() > 0) && value.find_first_of(" \t\n\r|\\'\";,<>&") == std::string::npos;
}

bool
SUMOXMLDefinitions::isValidDetectorID(const std::string& value) {
    // special case: ' ' allowed
    return (value.size() > 0) && value.find_first_of("\t\n\r|\\'\";,<>&") == std::string::npos;
}

bool
SUMOXMLDefinitions::isValidAttribute(const std::string& value) {
    return value.find_first_of("\t\n\r&|\\'\"<>") == std::string::npos;
}


bool
SUMOXMLDefinitions::isValidFilename(const std::string& value) {
    return (value.find_first_of("\t\n\r@$%^&|{}*'\";<>") == std::string::npos);
}


bool
SUMOXMLDefinitions::isValidListOfNetIDs(const std::string& value) {
    const std::vector<std::string>& typeIDs = StringTokenizer(value).getVector();
    if (typeIDs.empty()) {
        return false;
    } else {
        // check that gives IDs are valid
        for (auto i : typeIDs) {
            if (!SUMOXMLDefinitions::isValidNetID(i)) {
                return false;
            }
        }
        return true;
    }
}


bool
SUMOXMLDefinitions::isValidListOfTypeID(const std::string& value) {
    return isValidListOfTypeID(StringTokenizer(value).getVector());
}


bool
SUMOXMLDefinitions::isValidListOfTypeID(const std::vector<std::string>& typeIDs) {
    if (typeIDs.empty()) {
        return false;
    } else {
        // check that gives IDs are valid
        for (const auto& typeID : typeIDs) {
            if (!SUMOXMLDefinitions::isValidTypeID(typeID)) {
                return false;
            }
        }
        return true;
    }
}


bool
SUMOXMLDefinitions::isValidParameterKey(const std::string& value) {
    // Generic parameters keys cannot be empty
    if (value.empty()) {
        return false;
    } else {
        return isValidAttribute(value);
    }
}


std::string
SUMOXMLDefinitions::makeValidID(const std::string& value) {
    if (value.empty()) {
        return "_";
    }
    std::string result(value);
    if (result[0] == ':') {
        result[0] = '_';
    }
    for (const char c : " \t\n\r|\\'\";,<>&") {
        std::replace(result.begin(), result.end(), c, '_');
    }
    return result;
}


/****************************************************************************/
