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
/// @file    SUMOXMLDefinitions.h
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
#pragma once
#include <config.h>
#include <bitset>

#include <utils/common/StringBijection.h>
#include <utils/common/SequentialStringBijection.h>

// ===========================================================================
// definitions
// ===========================================================================
/**
 * @enum SumoXMLTag
 * @brief Numbers representing SUMO-XML - element names
 * @see GenericSAXHandler
 * @see SUMOSAXHandler
 */
enum SumoXMLTag {
    /// @brief root file
    SUMO_TAG_ROOTFILE,
    /// @brief root element of a network file
    SUMO_TAG_NET,
    /// @brief begin/end of the description of an edge
    SUMO_TAG_EDGE,
    /// @brief begin/end of the description of a single lane
    SUMO_TAG_LANE,
    /// @brief begin/end of the description of a neighboring lane
    SUMO_TAG_NEIGH,
    /// @brief begin/end of the description of a polygon
    SUMO_TAG_POLY,
    /// @brief begin/end of the description of a Point of interest
    SUMO_TAG_POI,
    /// @brief begin/end of the description of a junction
    SUMO_TAG_JUNCTION,
    /// @brief begin/end of the description of an edge restriction
    SUMO_TAG_RESTRICTION,
    /// @brief edge-specific meso settings
    SUMO_TAG_MESO,
    /// @brief  A bus stop
    SUMO_TAG_BUS_STOP,
    /// @brief  A train stop (alias for bus stop)
    SUMO_TAG_TRAIN_STOP,
    /// @brief A pt line
    SUMO_TAG_PT_LINE,
    /// @brief  An access point for a train stop
    SUMO_TAG_ACCESS,
    /// @brief  A container stop
    SUMO_TAG_CONTAINER_STOP,
    /// @brief A Charging Station
    SUMO_TAG_CHARGING_STATION,
    /// @brief A charging event (charging a vehicle at a charging station)
    SUMO_TAG_CHARGING_EVENT,
    /// @brief A parking area
    SUMO_TAG_PARKING_AREA,
    /// @brief A parking space for a single vehicle within a parking area
    SUMO_TAG_PARKING_SPACE,
    /// @brief an e1 detector
    SUMO_TAG_E1DETECTOR,
    /// @brief alternative tag for e1 detector
    SUMO_TAG_INDUCTION_LOOP,
    /// @brief an e2 detector
    SUMO_TAG_E2DETECTOR,
    /// @brief alternative tag for e2 detector
    SUMO_TAG_LANE_AREA_DETECTOR,
    /// @brief an e2 detector over multiple lanes (placed here due create Additional Frame)
    GNE_TAG_MULTI_LANE_AREA_DETECTOR,
    /// @brief an e3 detector
    SUMO_TAG_E3DETECTOR,
    /// @brief alternative tag for e3 detector
    SUMO_TAG_ENTRY_EXIT_DETECTOR,
    /// @brief an e3 entry point
    SUMO_TAG_DET_ENTRY,
    /// @brief an e3 exit point
    SUMO_TAG_DET_EXIT,
    /// @brief an edge based mean data detector
    SUMO_TAG_MEANDATA_EDGE,
    /// @brief a lane based mean data detector
    SUMO_TAG_MEANDATA_LANE,
    /// @brief  An edge-following detector
    SUMO_TAG_EDGEFOLLOWDETECTOR,
    /// @brief An instantenous induction loop
    SUMO_TAG_INSTANT_INDUCTION_LOOP,
    /// @brief a routeprobe detector
    SUMO_TAG_ROUTEPROBE,
    /// @brief A calibrator placed over edge
    SUMO_TAG_CALIBRATOR,
    /// @brief A calibrator placed over lane
    GNE_TAG_CALIBRATOR_LANE,
    /// @brief a flow definition within in Calibrator
    GNE_TAG_CALIBRATOR_FLOW,
    /// @brief  A rerouter
    SUMO_TAG_REROUTER,
    /// @brief Rerouter Symbol
    GNE_TAG_REROUTER_SYMBOL,
    /// @brief an aggreagated-output interval
    SUMO_TAG_INTERVAL,
    /// @brief probability of destination of a reroute
    SUMO_TAG_DEST_PROB_REROUTE,
    /// @brief reroute of type closing
    SUMO_TAG_CLOSING_REROUTE,
    /// @brief lane of a reroute of type closing
    SUMO_TAG_CLOSING_LANE_REROUTE,
    /// @brief probability of route of a reroute
    SUMO_TAG_ROUTE_PROB_REROUTE,
    /// @brief entry for an alternative parking zone
    SUMO_TAG_PARKING_AREA_REROUTE,
    /// @brief probability of a via fora reroute
    SUMO_TAG_VIA_PROB_REROUTE,
    /// @brief A variable speed sign
    SUMO_TAG_VSS,
    /// @brief VSS Symbol
    GNE_TAG_VSS_SYMBOL,
    /// @brief trigger: a step description
    SUMO_TAG_STEP,
    /// @brief vaporizer of vehicles
    SUMO_TAG_VAPORIZER,
    /// @brief  A traction substation
    SUMO_TAG_TRACTION_SUBSTATION,
    /// @brief  An overhead wire segment
    SUMO_TAG_OVERHEAD_WIRE_SEGMENT,
    /// @brief  An overhead wire section
    SUMO_TAG_OVERHEAD_WIRE_SECTION,
    /// @brief  An overhead wire clamp (connection of wires in opposite directions)
    SUMO_TAG_OVERHEAD_WIRE_CLAMP,
    /// @brief a vtypeprobe detector
    SUMO_TAG_VTYPEPROBE,
    /// @brief root element of a route file
    SUMO_TAG_ROUTES,
    /// @brief a single trip definition (used by router)
    SUMO_TAG_TRIP,
    /// @brief a trip between junctions
    GNE_TAG_TRIP_JUNCTIONS,
    /// @brief a single trip definition that uses TAZs
    GNE_TAG_TRIP_TAZS,
    /// @brief description of a vehicle
    SUMO_TAG_VEHICLE,
    /// @brief description of a vehicle with an embedded route
    GNE_TAG_VEHICLE_WITHROUTE,
    /// @brief a flow definition using from and to edges or a route
    SUMO_TAG_FLOW,
    /// @brief a flow between junctions
    GNE_TAG_FLOW_JUNCTIONS,
    /// @brief a flow between TAZs
    GNE_TAG_FLOW_TAZS,
    /// @brief a flow state definition (used when saving and loading simulatino state)
    SUMO_TAG_FLOWSTATE,
    /// @brief description of a vehicle/person/container type
    SUMO_TAG_VTYPE,
    /// @brief begin/end of the description of a route
    SUMO_TAG_ROUTE,
    /// @brief embedded route
    GNE_TAG_ROUTE_EMBEDDED,
    /// @brief description of a logic request within the junction
    SUMO_TAG_REQUEST,
    /// @brief a source
    SUMO_TAG_SOURCE,
    /// @brief a traffic assignment zone
    SUMO_TAG_TAZ,
    /// @brief a source within a district (connection road)
    SUMO_TAG_TAZSOURCE,
    /// @brief a sink within a district (connection road)
    SUMO_TAG_TAZSINK,
    /// @brief a traffic light
    SUMO_TAG_TRAFFIC_LIGHT,
    /// @brief a traffic light logic
    SUMO_TAG_TLLOGIC,
    /// @brief a single phase description
    SUMO_TAG_PHASE,
    /// @brief a condition for phase switching
    SUMO_TAG_CONDITION,
    /// @brief a conditional variable assignment for phase switching
    SUMO_TAG_ASSIGNMENT,
    /// @brief a sequence of assignments evaluated in the context of passed arguments
    SUMO_TAG_FUNCTION,
    /// @brief the internal state for edge control
    SUMO_TAG_EDGECONTROL,
    /// @brief a relation between two edges
    SUMO_TAG_EDGEREL,
    /// @brief a relation between two TAZs
    SUMO_TAG_TAZREL,
    /// @brief The definition of a periodic event
    SUMO_TAG_TIMEDEVENT,
    /// @brief Incoming edge specification (jtrrouter)
    SUMO_TAG_FROMEDGE,
    /// @brief Outgoing edge specification (jtrrouter)
    SUMO_TAG_TOEDGE,
    /// @brief Sink(s) specification
    SUMO_TAG_SINK,
    /// @brief parameter associated to a certain key
    SUMO_TAG_PARAM,
    SUMO_TAG_WAUT,
    SUMO_TAG_WAUT_SWITCH,
    SUMO_TAG_WAUT_JUNCTION,
    /// @brief segment of a lane
    SUMO_TAG_SEGMENT,
    /// @brief delete certain element (note: DELETE is a macro)
    SUMO_TAG_DEL,
    /// @brief root element of connections file
    SUMO_TAG_CONNECTIONS,
    /// @brief stop for vehicles
    SUMO_TAG_STOP,
    /// @brief stop placed over a lane
    GNE_TAG_STOP_LANE,
    /// @brief stop placed over a busStop
    GNE_TAG_STOP_BUSSTOP,
    /// @brief stop placed over a trainStop
    GNE_TAG_STOP_TRAINSTOP,
    /// @brief stop placed over a containerStop
    GNE_TAG_STOP_CONTAINERSTOP,
    /// @brief stop placed over a charging station
    GNE_TAG_STOP_CHARGINGSTATION,
    /// @brief stop placed over a parking area
    GNE_TAG_STOP_PARKINGAREA,
    /// @brief type of polygon
    SUMO_TAG_POLYTYPE,
    /// @brief connectioon between two lanes
    SUMO_TAG_CONNECTION,
    /// @brief conflict between two connections
    SUMO_TAG_CONFLICT,
    /// @brief prohibition of circulation between two edges
    SUMO_TAG_PROHIBITION,
    /// @brief split something
    SUMO_TAG_SPLIT,
    /// @brief alternative definition for junction
    SUMO_TAG_NODE,
    /// @brief type (edge)
    SUMO_TAG_TYPE,
    /// @brief lane type
    SUMO_TAG_LANETYPE,
    /// @brief definition of a detector
    SUMO_TAG_DETECTOR_DEFINITION,
    /// @brief distribution of a route
    SUMO_TAG_ROUTE_DISTRIBUTION,
    /// @brief distribution of a vehicle type
    SUMO_TAG_VTYPE_DISTRIBUTION,
    /// @brief roundabout defined in junction
    SUMO_TAG_ROUNDABOUT,
    /// @brief Join operation
    SUMO_TAG_JOIN,
    /// @brief join exlude operation
    SUMO_TAG_JOINEXCLUDE,
    /// @brief crossing between edges for pedestrians
    SUMO_TAG_CROSSING,
    /// @brief walking area for pedestrians
    SUMO_TAG_WALKINGAREA,
    /// @brief Information on vClass specific stop offsets at lane end
    SUMO_TAG_STOPOFFSET,
    /// @brief Constraints on switching a rail signal
    SUMO_TAG_RAILSIGNAL_CONSTRAINTS,
    /// @brief Predecessor constraint on switching a rail signal
    SUMO_TAG_PREDECESSOR,
    /// @brief Predecessor constraint on insertion before rail signal
    SUMO_TAG_INSERTION_PREDECESSOR,
    /// @brief Predecessor constraint on switching a rail signal
    SUMO_TAG_FOE_INSERTION,
    /// @brief Predecessor constraint on insertion before rail signal
    SUMO_TAG_INSERTION_ORDER,
    /// @brief Predecessor constraint for rail signal before bidirectional section
    SUMO_TAG_BIDI_PREDECESSOR,
    /// @brief Saved state for constraint tracker
    SUMO_TAG_RAILSIGNAL_CONSTRAINT_TRACKER,
    /// @brief Saved deadlock information, also for loading as an extra check
    SUMO_TAG_DEADLOCK,
    /// @brief Saved driveway information
    SUMO_TAG_DRIVEWAY,
    SUMO_TAG_SUBDRIVEWAY,
    /// @brief Link information for state-saving
    SUMO_TAG_LINK,
    /// @brief Link-approaching vehicle information for state-saving
    SUMO_TAG_APPROACHING,

    SUMO_TAG_WAY,
    SUMO_TAG_ND,
    SUMO_TAG_TAG,
    SUMO_TAG_RELATION,
    SUMO_TAG_MEMBER,

    /// @name parameters associated to view settings
    /// @{
    SUMO_TAG_VIEWSETTINGS,
    SUMO_TAG_VIEWSETTINGS_3D,
    SUMO_TAG_VIEWSETTINGS_DECAL,
    SUMO_TAG_VIEWSETTINGS_LIGHT,
    SUMO_TAG_VIEWSETTINGS_SCHEME,
    SUMO_TAG_VIEWSETTINGS_OPENGL,
    SUMO_TAG_VIEWSETTINGS_BACKGROUND,
    SUMO_TAG_VIEWSETTINGS_EDGES,
    SUMO_TAG_VIEWSETTINGS_VEHICLES,
    SUMO_TAG_VIEWSETTINGS_PERSONS,
    SUMO_TAG_VIEWSETTINGS_CONTAINERS,
    SUMO_TAG_VIEWSETTINGS_JUNCTIONS,
    SUMO_TAG_VIEWSETTINGS_ADDITIONALS,
    SUMO_TAG_VIEWSETTINGS_POIS,
    SUMO_TAG_VIEWSETTINGS_POLYS,
    SUMO_TAG_VIEWSETTINGS_LEGEND,
    SUMO_TAG_VIEWSETTINGS_EVENT,
    SUMO_TAG_VIEWSETTINGS_EVENT_JAM_TIME,
    SUMO_TAG_INCLUDE,
    SUMO_TAG_DELAY,
    SUMO_TAG_VIEWPORT,
    SUMO_TAG_SNAPSHOT,
    SUMO_TAG_BREAKPOINT,
    SUMO_TAG_LOCATION,
    SUMO_TAG_COLORSCHEME,
    SUMO_TAG_SCALINGSCHEME,
    SUMO_TAG_ENTRY,
    SUMO_TAG_RNGSTATE,
    SUMO_TAG_RNGLANE,
    /// @}

    SUMO_TAG_VEHICLETRANSFER,
    SUMO_TAG_DEVICE,

    /// @name Car-Following models
    /// @{
    SUMO_TAG_CF_KRAUSS,
    SUMO_TAG_CF_KRAUSS_PLUS_SLOPE,
    SUMO_TAG_CF_KRAUSS_ORIG1,
    SUMO_TAG_CF_KRAUSSX,
    SUMO_TAG_CF_EIDM,
    SUMO_TAG_CF_SMART_SK,
    SUMO_TAG_CF_DANIEL1,
    SUMO_TAG_CF_IDM,
    SUMO_TAG_CF_IDMM,
    SUMO_TAG_CF_PWAGNER2009,
    SUMO_TAG_CF_BKERNER,
    SUMO_TAG_CF_WIEDEMANN,
    SUMO_TAG_CF_W99,
    SUMO_TAG_CF_ACC,
    SUMO_TAG_CF_CACC,
    SUMO_TAG_CF_RAIL,
    SUMO_TAG_CF_CC,
    /// @}

    /// @name Persons
    /// @{
    SUMO_TAG_PERSON,
    SUMO_TAG_PERSONTRIP,
    SUMO_TAG_RIDE,
    SUMO_TAG_WALK,
    SUMO_TAG_PERSONFLOW,
    /// @}

    /// @name Data elements (used by Netedit)
    /// @{
    SUMO_TAG_DATASET,
    SUMO_TAG_DATAINTERVAL,
    /// @}

    /// @name Containers
    /// @{
    SUMO_TAG_CONTAINER,
    SUMO_TAG_TRANSPORT,
    SUMO_TAG_TRANSHIP,
    SUMO_TAG_CONTAINERFLOW,
    /// @}

    SUMO_TAG_TRAJECTORIES,
    SUMO_TAG_TIMESTEP,
    SUMO_TAG_TIMESLICE,
    SUMO_TAG_ACTORCONFIG,
    SUMO_TAG_MOTIONSTATE,
    SUMO_TAG_OD_PAIR,
    SUMO_TAG_TRANSPORTABLES,

    /// @brief ActivityGen Tags
    AGEN_TAG_GENERAL,
    /// @brief streets object
    AGEN_TAG_STREET,
    /// @brief workingHours object
    AGEN_TAG_WORKHOURS,
    /// @brief opening for workingHours object
    AGEN_TAG_OPENING,
    /// @brief closing for workingHours object
    AGEN_TAG_CLOSING,
    /// @brief school object
    AGEN_TAG_SCHOOLS,
    /// @brief schools object
    AGEN_TAG_SCHOOL,
    /// @brief busStation and bus objects
    AGEN_TAG_BUSSTATION,
    /// @brief  bus line
    AGEN_TAG_BUSLINE,
    /// @brief stations for certain vehicles
    AGEN_TAG_STATIONS,
    /// @brief rev stations for certain vehicles
    AGEN_TAG_REV_STATIONS,
    /// @brief station for a certain vehicle
    AGEN_TAG_STATION,
    /// @brief frequency of a object
    AGEN_TAG_FREQUENCY,
    /// @brief population and children accompaniment brackets
    AGEN_TAG_POPULATION,
    /// @brief alternative definition for Population
    AGEN_TAG_BRACKET,
    //AGEN_TAG_CHILD_ACOMP,
    /// @brief city entrances
    AGEN_TAG_CITYGATES,
    /// @brief alternative definition for city entrances
    AGEN_TAG_ENTRANCE,
    /// @brief parameters
    AGEN_TAG_PARAM,

    /// @name Netedit elements
    /// @{
    /// @brief edge rel (defined in a single lane)
    GNE_TAG_EDGEREL_SINGLE,
    /// @brief internal lane
    GNE_TAG_INTERNAL_LANE,
    /// @brief Point of interest over Lane
    GNE_TAG_POILANE,
    /// @brief Point of interest over view with GEO attributes
    GNE_TAG_POIGEO,
    /// @brief polygon used for draw juPedSim walkable areas
    GNE_TAG_JPS_WALKABLEAREA,
    /// @brief polygon used for draw juPedSim obstacles
    GNE_TAG_JPS_OBSTACLE,
    /// @brief COF Symbol
    GNE_TAG_COF_SYMBOL,
    /// @brief a flow definition using a route instead of a from-to edges route
    GNE_TAG_FLOW_ROUTE,
    /// @brief description of a vehicle with an embedded route
    GNE_TAG_FLOW_WITHROUTE,
    /// @brief waypoints
    GNE_TAG_WAYPOINT,
    GNE_TAG_WAYPOINT_LANE,
    GNE_TAG_WAYPOINT_BUSSTOP,
    GNE_TAG_WAYPOINT_TRAINSTOP,
    GNE_TAG_WAYPOINT_CONTAINERSTOP,
    GNE_TAG_WAYPOINT_CHARGINGSTATION,
    GNE_TAG_WAYPOINT_PARKINGAREA,
    // @brief person trips
    GNE_TAG_PERSONTRIP_EDGE_EDGE,
    GNE_TAG_PERSONTRIP_EDGE_TAZ,
    GNE_TAG_PERSONTRIP_EDGE_JUNCTION,
    GNE_TAG_PERSONTRIP_EDGE_BUSSTOP,
    GNE_TAG_PERSONTRIP_EDGE_TRAINSTOP,
    GNE_TAG_PERSONTRIP_EDGE_CONTAINERSTOP,
    GNE_TAG_PERSONTRIP_EDGE_CHARGINGSTATION,
    GNE_TAG_PERSONTRIP_EDGE_PARKINGAREA,
    GNE_TAG_PERSONTRIP_TAZ_EDGE,
    GNE_TAG_PERSONTRIP_TAZ_TAZ,
    GNE_TAG_PERSONTRIP_TAZ_JUNCTION,
    GNE_TAG_PERSONTRIP_TAZ_BUSSTOP,
    GNE_TAG_PERSONTRIP_TAZ_TRAINSTOP,
    GNE_TAG_PERSONTRIP_TAZ_CONTAINERSTOP,
    GNE_TAG_PERSONTRIP_TAZ_CHARGINGSTATION,
    GNE_TAG_PERSONTRIP_TAZ_PARKINGAREA,
    GNE_TAG_PERSONTRIP_JUNCTION_EDGE,
    GNE_TAG_PERSONTRIP_JUNCTION_TAZ,
    GNE_TAG_PERSONTRIP_JUNCTION_JUNCTION,
    GNE_TAG_PERSONTRIP_JUNCTION_BUSSTOP,
    GNE_TAG_PERSONTRIP_JUNCTION_TRAINSTOP,
    GNE_TAG_PERSONTRIP_JUNCTION_CONTAINERSTOP,
    GNE_TAG_PERSONTRIP_JUNCTION_CHARGINGSTATION,
    GNE_TAG_PERSONTRIP_JUNCTION_PARKINGAREA,
    GNE_TAG_PERSONTRIP_BUSSTOP_EDGE,
    GNE_TAG_PERSONTRIP_BUSSTOP_TAZ,
    GNE_TAG_PERSONTRIP_BUSSTOP_JUNCTION,
    GNE_TAG_PERSONTRIP_BUSSTOP_BUSSTOP,
    GNE_TAG_PERSONTRIP_BUSSTOP_TRAINSTOP,
    GNE_TAG_PERSONTRIP_BUSSTOP_CONTAINERSTOP,
    GNE_TAG_PERSONTRIP_BUSSTOP_CHARGINGSTATION,
    GNE_TAG_PERSONTRIP_BUSSTOP_PARKINGAREA,
    GNE_TAG_PERSONTRIP_TRAINSTOP_EDGE,
    GNE_TAG_PERSONTRIP_TRAINSTOP_TAZ,
    GNE_TAG_PERSONTRIP_TRAINSTOP_JUNCTION,
    GNE_TAG_PERSONTRIP_TRAINSTOP_BUSSTOP,
    GNE_TAG_PERSONTRIP_TRAINSTOP_TRAINSTOP,
    GNE_TAG_PERSONTRIP_TRAINSTOP_CONTAINERSTOP,
    GNE_TAG_PERSONTRIP_TRAINSTOP_CHARGINGSTATION,
    GNE_TAG_PERSONTRIP_TRAINSTOP_PARKINGAREA,
    GNE_TAG_PERSONTRIP_CONTAINERSTOP_EDGE,
    GNE_TAG_PERSONTRIP_CONTAINERSTOP_TAZ,
    GNE_TAG_PERSONTRIP_CONTAINERSTOP_JUNCTION,
    GNE_TAG_PERSONTRIP_CONTAINERSTOP_BUSSTOP,
    GNE_TAG_PERSONTRIP_CONTAINERSTOP_TRAINSTOP,
    GNE_TAG_PERSONTRIP_CONTAINERSTOP_CONTAINERSTOP,
    GNE_TAG_PERSONTRIP_CONTAINERSTOP_CHARGINGSTATION,
    GNE_TAG_PERSONTRIP_CONTAINERSTOP_PARKINGAREA,
    GNE_TAG_PERSONTRIP_CHARGINGSTATION_EDGE,
    GNE_TAG_PERSONTRIP_CHARGINGSTATION_TAZ,
    GNE_TAG_PERSONTRIP_CHARGINGSTATION_JUNCTION,
    GNE_TAG_PERSONTRIP_CHARGINGSTATION_BUSSTOP,
    GNE_TAG_PERSONTRIP_CHARGINGSTATION_TRAINSTOP,
    GNE_TAG_PERSONTRIP_CHARGINGSTATION_CONTAINERSTOP,
    GNE_TAG_PERSONTRIP_CHARGINGSTATION_CHARGINGSTATION,
    GNE_TAG_PERSONTRIP_CHARGINGSTATION_PARKINGAREA,
    GNE_TAG_PERSONTRIP_PARKINGAREA_EDGE,
    GNE_TAG_PERSONTRIP_PARKINGAREA_TAZ,
    GNE_TAG_PERSONTRIP_PARKINGAREA_JUNCTION,
    GNE_TAG_PERSONTRIP_PARKINGAREA_BUSSTOP,
    GNE_TAG_PERSONTRIP_PARKINGAREA_TRAINSTOP,
    GNE_TAG_PERSONTRIP_PARKINGAREA_CONTAINERSTOP,
    GNE_TAG_PERSONTRIP_PARKINGAREA_CHARGINGSTATION,
    GNE_TAG_PERSONTRIP_PARKINGAREA_PARKINGAREA,
    // @brief walks
    GNE_TAG_WALK_EDGE_EDGE,
    GNE_TAG_WALK_EDGE_TAZ,
    GNE_TAG_WALK_EDGE_JUNCTION,
    GNE_TAG_WALK_EDGE_BUSSTOP,
    GNE_TAG_WALK_EDGE_TRAINSTOP,
    GNE_TAG_WALK_EDGE_CONTAINERSTOP,
    GNE_TAG_WALK_EDGE_CHARGINGSTATION,
    GNE_TAG_WALK_EDGE_PARKINGAREA,
    GNE_TAG_WALK_TAZ_EDGE,
    GNE_TAG_WALK_TAZ_TAZ,
    GNE_TAG_WALK_TAZ_JUNCTION,
    GNE_TAG_WALK_TAZ_BUSSTOP,
    GNE_TAG_WALK_TAZ_TRAINSTOP,
    GNE_TAG_WALK_TAZ_CONTAINERSTOP,
    GNE_TAG_WALK_TAZ_CHARGINGSTATION,
    GNE_TAG_WALK_TAZ_PARKINGAREA,
    GNE_TAG_WALK_JUNCTION_EDGE,
    GNE_TAG_WALK_JUNCTION_TAZ,
    GNE_TAG_WALK_JUNCTION_JUNCTION,
    GNE_TAG_WALK_JUNCTION_BUSSTOP,
    GNE_TAG_WALK_JUNCTION_TRAINSTOP,
    GNE_TAG_WALK_JUNCTION_CONTAINERSTOP,
    GNE_TAG_WALK_JUNCTION_CHARGINGSTATION,
    GNE_TAG_WALK_JUNCTION_PARKINGAREA,
    GNE_TAG_WALK_BUSSTOP_EDGE,
    GNE_TAG_WALK_BUSSTOP_TAZ,
    GNE_TAG_WALK_BUSSTOP_JUNCTION,
    GNE_TAG_WALK_BUSSTOP_BUSSTOP,
    GNE_TAG_WALK_BUSSTOP_TRAINSTOP,
    GNE_TAG_WALK_BUSSTOP_CONTAINERSTOP,
    GNE_TAG_WALK_BUSSTOP_CHARGINGSTATION,
    GNE_TAG_WALK_BUSSTOP_PARKINGAREA,
    GNE_TAG_WALK_TRAINSTOP_EDGE,
    GNE_TAG_WALK_TRAINSTOP_TAZ,
    GNE_TAG_WALK_TRAINSTOP_JUNCTION,
    GNE_TAG_WALK_TRAINSTOP_BUSSTOP,
    GNE_TAG_WALK_TRAINSTOP_TRAINSTOP,
    GNE_TAG_WALK_TRAINSTOP_CONTAINERSTOP,
    GNE_TAG_WALK_TRAINSTOP_CHARGINGSTATION,
    GNE_TAG_WALK_TRAINSTOP_PARKINGAREA,
    GNE_TAG_WALK_CONTAINERSTOP_EDGE,
    GNE_TAG_WALK_CONTAINERSTOP_TAZ,
    GNE_TAG_WALK_CONTAINERSTOP_JUNCTION,
    GNE_TAG_WALK_CONTAINERSTOP_BUSSTOP,
    GNE_TAG_WALK_CONTAINERSTOP_TRAINSTOP,
    GNE_TAG_WALK_CONTAINERSTOP_CONTAINERSTOP,
    GNE_TAG_WALK_CONTAINERSTOP_CHARGINGSTATION,
    GNE_TAG_WALK_CONTAINERSTOP_PARKINGAREA,
    GNE_TAG_WALK_CHARGINGSTATION_EDGE,
    GNE_TAG_WALK_CHARGINGSTATION_TAZ,
    GNE_TAG_WALK_CHARGINGSTATION_JUNCTION,
    GNE_TAG_WALK_CHARGINGSTATION_BUSSTOP,
    GNE_TAG_WALK_CHARGINGSTATION_TRAINSTOP,
    GNE_TAG_WALK_CHARGINGSTATION_CONTAINERSTOP,
    GNE_TAG_WALK_CHARGINGSTATION_CHARGINGSTATION,
    GNE_TAG_WALK_CHARGINGSTATION_PARKINGAREA,
    GNE_TAG_WALK_PARKINGAREA_EDGE,
    GNE_TAG_WALK_PARKINGAREA_TAZ,
    GNE_TAG_WALK_PARKINGAREA_JUNCTION,
    GNE_TAG_WALK_PARKINGAREA_BUSSTOP,
    GNE_TAG_WALK_PARKINGAREA_TRAINSTOP,
    GNE_TAG_WALK_PARKINGAREA_CONTAINERSTOP,
    GNE_TAG_WALK_PARKINGAREA_CHARGINGSTATION,
    GNE_TAG_WALK_PARKINGAREA_PARKINGAREA,
    GNE_TAG_WALK_EDGES,
    GNE_TAG_WALK_ROUTE,
    // @brief rides
    GNE_TAG_RIDE_EDGE_EDGE,
    GNE_TAG_RIDE_EDGE_TAZ,
    GNE_TAG_RIDE_EDGE_JUNCTION,
    GNE_TAG_RIDE_EDGE_BUSSTOP,
    GNE_TAG_RIDE_EDGE_TRAINSTOP,
    GNE_TAG_RIDE_EDGE_CONTAINERSTOP,
    GNE_TAG_RIDE_EDGE_CHARGINGSTATION,
    GNE_TAG_RIDE_EDGE_PARKINGAREA,
    GNE_TAG_RIDE_TAZ_EDGE,
    GNE_TAG_RIDE_TAZ_TAZ,
    GNE_TAG_RIDE_TAZ_JUNCTION,
    GNE_TAG_RIDE_TAZ_BUSSTOP,
    GNE_TAG_RIDE_TAZ_TRAINSTOP,
    GNE_TAG_RIDE_TAZ_CONTAINERSTOP,
    GNE_TAG_RIDE_TAZ_CHARGINGSTATION,
    GNE_TAG_RIDE_TAZ_PARKINGAREA,
    GNE_TAG_RIDE_JUNCTION_EDGE,
    GNE_TAG_RIDE_JUNCTION_TAZ,
    GNE_TAG_RIDE_JUNCTION_JUNCTION,
    GNE_TAG_RIDE_JUNCTION_BUSSTOP,
    GNE_TAG_RIDE_JUNCTION_TRAINSTOP,
    GNE_TAG_RIDE_JUNCTION_CONTAINERSTOP,
    GNE_TAG_RIDE_JUNCTION_CHARGINGSTATION,
    GNE_TAG_RIDE_JUNCTION_PARKINGAREA,
    GNE_TAG_RIDE_BUSSTOP_EDGE,
    GNE_TAG_RIDE_BUSSTOP_TAZ,
    GNE_TAG_RIDE_BUSSTOP_JUNCTION,
    GNE_TAG_RIDE_BUSSTOP_BUSSTOP,
    GNE_TAG_RIDE_BUSSTOP_TRAINSTOP,
    GNE_TAG_RIDE_BUSSTOP_CONTAINERSTOP,
    GNE_TAG_RIDE_BUSSTOP_CHARGINGSTATION,
    GNE_TAG_RIDE_BUSSTOP_PARKINGAREA,
    GNE_TAG_RIDE_TRAINSTOP_EDGE,
    GNE_TAG_RIDE_TRAINSTOP_TAZ,
    GNE_TAG_RIDE_TRAINSTOP_JUNCTION,
    GNE_TAG_RIDE_TRAINSTOP_BUSSTOP,
    GNE_TAG_RIDE_TRAINSTOP_TRAINSTOP,
    GNE_TAG_RIDE_TRAINSTOP_CONTAINERSTOP,
    GNE_TAG_RIDE_TRAINSTOP_CHARGINGSTATION,
    GNE_TAG_RIDE_TRAINSTOP_PARKINGAREA,
    GNE_TAG_RIDE_CONTAINERSTOP_EDGE,
    GNE_TAG_RIDE_CONTAINERSTOP_TAZ,
    GNE_TAG_RIDE_CONTAINERSTOP_JUNCTION,
    GNE_TAG_RIDE_CONTAINERSTOP_BUSSTOP,
    GNE_TAG_RIDE_CONTAINERSTOP_TRAINSTOP,
    GNE_TAG_RIDE_CONTAINERSTOP_CONTAINERSTOP,
    GNE_TAG_RIDE_CONTAINERSTOP_CHARGINGSTATION,
    GNE_TAG_RIDE_CONTAINERSTOP_PARKINGAREA,
    GNE_TAG_RIDE_CHARGINGSTATION_EDGE,
    GNE_TAG_RIDE_CHARGINGSTATION_TAZ,
    GNE_TAG_RIDE_CHARGINGSTATION_JUNCTION,
    GNE_TAG_RIDE_CHARGINGSTATION_BUSSTOP,
    GNE_TAG_RIDE_CHARGINGSTATION_TRAINSTOP,
    GNE_TAG_RIDE_CHARGINGSTATION_CONTAINERSTOP,
    GNE_TAG_RIDE_CHARGINGSTATION_CHARGINGSTATION,
    GNE_TAG_RIDE_CHARGINGSTATION_PARKINGAREA,
    GNE_TAG_RIDE_PARKINGAREA_EDGE,
    GNE_TAG_RIDE_PARKINGAREA_TAZ,
    GNE_TAG_RIDE_PARKINGAREA_JUNCTION,
    GNE_TAG_RIDE_PARKINGAREA_BUSSTOP,
    GNE_TAG_RIDE_PARKINGAREA_TRAINSTOP,
    GNE_TAG_RIDE_PARKINGAREA_CONTAINERSTOP,
    GNE_TAG_RIDE_PARKINGAREA_CHARGINGSTATION,
    GNE_TAG_RIDE_PARKINGAREA_PARKINGAREA,
    // @brief person stops
    GNE_TAG_STOPPERSON,
    GNE_TAG_STOPPERSON_EDGE,
    GNE_TAG_STOPPERSON_BUSSTOP,
    GNE_TAG_STOPPERSON_TRAINSTOP,
    GNE_TAG_STOPPERSON_CONTAINERSTOP,
    GNE_TAG_STOPPERSON_CHARGINGSTATION,
    GNE_TAG_STOPPERSON_PARKINGAREA,
    // @brief transports
    GNE_TAG_TRANSPORT_EDGE_EDGE,
    GNE_TAG_TRANSPORT_EDGE_TAZ,
    GNE_TAG_TRANSPORT_EDGE_JUNCTION,
    GNE_TAG_TRANSPORT_EDGE_BUSSTOP,
    GNE_TAG_TRANSPORT_EDGE_TRAINSTOP,
    GNE_TAG_TRANSPORT_EDGE_CONTAINERSTOP,
    GNE_TAG_TRANSPORT_EDGE_CHARGINGSTATION,
    GNE_TAG_TRANSPORT_EDGE_PARKINGAREA,
    GNE_TAG_TRANSPORT_TAZ_EDGE,
    GNE_TAG_TRANSPORT_TAZ_TAZ,
    GNE_TAG_TRANSPORT_TAZ_JUNCTION,
    GNE_TAG_TRANSPORT_TAZ_BUSSTOP,
    GNE_TAG_TRANSPORT_TAZ_TRAINSTOP,
    GNE_TAG_TRANSPORT_TAZ_CONTAINERSTOP,
    GNE_TAG_TRANSPORT_TAZ_CHARGINGSTATION,
    GNE_TAG_TRANSPORT_TAZ_PARKINGAREA,
    GNE_TAG_TRANSPORT_JUNCTION_EDGE,
    GNE_TAG_TRANSPORT_JUNCTION_TAZ,
    GNE_TAG_TRANSPORT_JUNCTION_JUNCTION,
    GNE_TAG_TRANSPORT_JUNCTION_BUSSTOP,
    GNE_TAG_TRANSPORT_JUNCTION_TRAINSTOP,
    GNE_TAG_TRANSPORT_JUNCTION_CONTAINERSTOP,
    GNE_TAG_TRANSPORT_JUNCTION_CHARGINGSTATION,
    GNE_TAG_TRANSPORT_JUNCTION_PARKINGAREA,
    GNE_TAG_TRANSPORT_BUSSTOP_EDGE,
    GNE_TAG_TRANSPORT_BUSSTOP_TAZ,
    GNE_TAG_TRANSPORT_BUSSTOP_JUNCTION,
    GNE_TAG_TRANSPORT_BUSSTOP_BUSSTOP,
    GNE_TAG_TRANSPORT_BUSSTOP_TRAINSTOP,
    GNE_TAG_TRANSPORT_BUSSTOP_CONTAINERSTOP,
    GNE_TAG_TRANSPORT_BUSSTOP_CHARGINGSTATION,
    GNE_TAG_TRANSPORT_BUSSTOP_PARKINGAREA,
    GNE_TAG_TRANSPORT_TRAINSTOP_EDGE,
    GNE_TAG_TRANSPORT_TRAINSTOP_TAZ,
    GNE_TAG_TRANSPORT_TRAINSTOP_JUNCTION,
    GNE_TAG_TRANSPORT_TRAINSTOP_BUSSTOP,
    GNE_TAG_TRANSPORT_TRAINSTOP_TRAINSTOP,
    GNE_TAG_TRANSPORT_TRAINSTOP_CONTAINERSTOP,
    GNE_TAG_TRANSPORT_TRAINSTOP_CHARGINGSTATION,
    GNE_TAG_TRANSPORT_TRAINSTOP_PARKINGAREA,
    GNE_TAG_TRANSPORT_CONTAINERSTOP_EDGE,
    GNE_TAG_TRANSPORT_CONTAINERSTOP_TAZ,
    GNE_TAG_TRANSPORT_CONTAINERSTOP_JUNCTION,
    GNE_TAG_TRANSPORT_CONTAINERSTOP_BUSSTOP,
    GNE_TAG_TRANSPORT_CONTAINERSTOP_TRAINSTOP,
    GNE_TAG_TRANSPORT_CONTAINERSTOP_CONTAINERSTOP,
    GNE_TAG_TRANSPORT_CONTAINERSTOP_CHARGINGSTATION,
    GNE_TAG_TRANSPORT_CONTAINERSTOP_PARKINGAREA,
    GNE_TAG_TRANSPORT_CHARGINGSTATION_EDGE,
    GNE_TAG_TRANSPORT_CHARGINGSTATION_TAZ,
    GNE_TAG_TRANSPORT_CHARGINGSTATION_JUNCTION,
    GNE_TAG_TRANSPORT_CHARGINGSTATION_BUSSTOP,
    GNE_TAG_TRANSPORT_CHARGINGSTATION_TRAINSTOP,
    GNE_TAG_TRANSPORT_CHARGINGSTATION_CONTAINERSTOP,
    GNE_TAG_TRANSPORT_CHARGINGSTATION_CHARGINGSTATION,
    GNE_TAG_TRANSPORT_CHARGINGSTATION_PARKINGAREA,
    GNE_TAG_TRANSPORT_PARKINGAREA_EDGE,
    GNE_TAG_TRANSPORT_PARKINGAREA_TAZ,
    GNE_TAG_TRANSPORT_PARKINGAREA_JUNCTION,
    GNE_TAG_TRANSPORT_PARKINGAREA_BUSSTOP,
    GNE_TAG_TRANSPORT_PARKINGAREA_TRAINSTOP,
    GNE_TAG_TRANSPORT_PARKINGAREA_CONTAINERSTOP,
    GNE_TAG_TRANSPORT_PARKINGAREA_CHARGINGSTATION,
    GNE_TAG_TRANSPORT_PARKINGAREA_PARKINGAREA,
    // @brief tranships
    GNE_TAG_TRANSHIP_EDGE_EDGE,
    GNE_TAG_TRANSHIP_EDGE_TAZ,
    GNE_TAG_TRANSHIP_EDGE_JUNCTION,
    GNE_TAG_TRANSHIP_EDGE_BUSSTOP,
    GNE_TAG_TRANSHIP_EDGE_TRAINSTOP,
    GNE_TAG_TRANSHIP_EDGE_CONTAINERSTOP,
    GNE_TAG_TRANSHIP_EDGE_CHARGINGSTATION,
    GNE_TAG_TRANSHIP_EDGE_PARKINGAREA,
    GNE_TAG_TRANSHIP_TAZ_EDGE,
    GNE_TAG_TRANSHIP_TAZ_TAZ,
    GNE_TAG_TRANSHIP_TAZ_JUNCTION,
    GNE_TAG_TRANSHIP_TAZ_BUSSTOP,
    GNE_TAG_TRANSHIP_TAZ_TRAINSTOP,
    GNE_TAG_TRANSHIP_TAZ_CONTAINERSTOP,
    GNE_TAG_TRANSHIP_TAZ_CHARGINGSTATION,
    GNE_TAG_TRANSHIP_TAZ_PARKINGAREA,
    GNE_TAG_TRANSHIP_JUNCTION_EDGE,
    GNE_TAG_TRANSHIP_JUNCTION_TAZ,
    GNE_TAG_TRANSHIP_JUNCTION_JUNCTION,
    GNE_TAG_TRANSHIP_JUNCTION_BUSSTOP,
    GNE_TAG_TRANSHIP_JUNCTION_TRAINSTOP,
    GNE_TAG_TRANSHIP_JUNCTION_CONTAINERSTOP,
    GNE_TAG_TRANSHIP_JUNCTION_CHARGINGSTATION,
    GNE_TAG_TRANSHIP_JUNCTION_PARKINGAREA,
    GNE_TAG_TRANSHIP_BUSSTOP_EDGE,
    GNE_TAG_TRANSHIP_BUSSTOP_TAZ,
    GNE_TAG_TRANSHIP_BUSSTOP_JUNCTION,
    GNE_TAG_TRANSHIP_BUSSTOP_BUSSTOP,
    GNE_TAG_TRANSHIP_BUSSTOP_TRAINSTOP,
    GNE_TAG_TRANSHIP_BUSSTOP_CONTAINERSTOP,
    GNE_TAG_TRANSHIP_BUSSTOP_CHARGINGSTATION,
    GNE_TAG_TRANSHIP_BUSSTOP_PARKINGAREA,
    GNE_TAG_TRANSHIP_TRAINSTOP_EDGE,
    GNE_TAG_TRANSHIP_TRAINSTOP_TAZ,
    GNE_TAG_TRANSHIP_TRAINSTOP_JUNCTION,
    GNE_TAG_TRANSHIP_TRAINSTOP_BUSSTOP,
    GNE_TAG_TRANSHIP_TRAINSTOP_TRAINSTOP,
    GNE_TAG_TRANSHIP_TRAINSTOP_CONTAINERSTOP,
    GNE_TAG_TRANSHIP_TRAINSTOP_CHARGINGSTATION,
    GNE_TAG_TRANSHIP_TRAINSTOP_PARKINGAREA,
    GNE_TAG_TRANSHIP_CONTAINERSTOP_EDGE,
    GNE_TAG_TRANSHIP_CONTAINERSTOP_TAZ,
    GNE_TAG_TRANSHIP_CONTAINERSTOP_JUNCTION,
    GNE_TAG_TRANSHIP_CONTAINERSTOP_BUSSTOP,
    GNE_TAG_TRANSHIP_CONTAINERSTOP_TRAINSTOP,
    GNE_TAG_TRANSHIP_CONTAINERSTOP_CONTAINERSTOP,
    GNE_TAG_TRANSHIP_CONTAINERSTOP_CHARGINGSTATION,
    GNE_TAG_TRANSHIP_CONTAINERSTOP_PARKINGAREA,
    GNE_TAG_TRANSHIP_CHARGINGSTATION_EDGE,
    GNE_TAG_TRANSHIP_CHARGINGSTATION_TAZ,
    GNE_TAG_TRANSHIP_CHARGINGSTATION_JUNCTION,
    GNE_TAG_TRANSHIP_CHARGINGSTATION_BUSSTOP,
    GNE_TAG_TRANSHIP_CHARGINGSTATION_TRAINSTOP,
    GNE_TAG_TRANSHIP_CHARGINGSTATION_CONTAINERSTOP,
    GNE_TAG_TRANSHIP_CHARGINGSTATION_CHARGINGSTATION,
    GNE_TAG_TRANSHIP_CHARGINGSTATION_PARKINGAREA,
    GNE_TAG_TRANSHIP_PARKINGAREA_EDGE,
    GNE_TAG_TRANSHIP_PARKINGAREA_TAZ,
    GNE_TAG_TRANSHIP_PARKINGAREA_JUNCTION,
    GNE_TAG_TRANSHIP_PARKINGAREA_BUSSTOP,
    GNE_TAG_TRANSHIP_PARKINGAREA_TRAINSTOP,
    GNE_TAG_TRANSHIP_PARKINGAREA_CONTAINERSTOP,
    GNE_TAG_TRANSHIP_PARKINGAREA_CHARGINGSTATION,
    GNE_TAG_TRANSHIP_PARKINGAREA_PARKINGAREA,
    GNE_TAG_TRANSHIP_EDGES,
    // @brief container stops
    GNE_TAG_STOPCONTAINER,
    GNE_TAG_STOPCONTAINER_EDGE,
    GNE_TAG_STOPCONTAINER_BUSSTOP,
    GNE_TAG_STOPCONTAINER_TRAINSTOP,
    GNE_TAG_STOPCONTAINER_CONTAINERSTOP,
    GNE_TAG_STOPCONTAINER_CHARGINGSTATION,
    GNE_TAG_STOPCONTAINER_PARKINGAREA,
    /// @}

    /// @brief invalid tag, must be the last one
    SUMO_TAG_NOTHING,
};

typedef std::bitset<96> SumoXMLAttrMask;

/**
 * @enum SumoXMLAttr
 * @brief Numbers representing SUMO-XML - attributes
 * @see GenericSAXHandler
 * @see SUMOSAXHandler
 */
enum SumoXMLAttr {
    /// @name meanData output attributes
    /// @note: sorted first to simplify filtering written attributes with bit operations
    /// @{
    SUMO_ATTR_DEFAULT            =  1,
    /// MSMeanData_Net
    SUMO_ATTR_SAMPLEDSECONDS     =  2,
    SUMO_ATTR_DENSITY            =  3,
    SUMO_ATTR_LANEDENSITY        =  4,
    SUMO_ATTR_OCCUPANCY          =  5,
    SUMO_ATTR_WAITINGTIME        =  6,
    SUMO_ATTR_TIMELOSS           =  7,
    SUMO_ATTR_SPEED              =  8,
    SUMO_ATTR_SPEEDREL           =  9,
    SUMO_ATTR_DEPARTED           = 10,
    SUMO_ATTR_ARRIVED            = 11,
    SUMO_ATTR_ENTERED            = 12,
    SUMO_ATTR_LEFT               = 13,
    SUMO_ATTR_VAPORIZED          = 14,
    SUMO_ATTR_TELEPORTED         = 15,
    SUMO_ATTR_TRAVELTIME         = 16,
    SUMO_ATTR_LANECHANGEDFROM    = 17,
    SUMO_ATTR_LANECHANGEDTO      = 18,
    SUMO_ATTR_OVERLAPTRAVELTIME  = 19,
    /// MSMeanData_Emissions
    SUMO_ATTR_CO_ABS             = 20,
    SUMO_ATTR_CO2_ABS            = 21,
    SUMO_ATTR_HC_ABS             = 22,
    SUMO_ATTR_PMX_ABS            = 23,
    SUMO_ATTR_NOX_ABS            = 24,
    SUMO_ATTR_FUEL_ABS           = 25,
    SUMO_ATTR_ELECTRICITY_ABS    = 26,
    SUMO_ATTR_CO_NORMED          = 27,
    SUMO_ATTR_CO2_NORMED         = 28,
    SUMO_ATTR_HC_NORMED          = 29,
    SUMO_ATTR_PMX_NORMED         = 30,
    SUMO_ATTR_NOX_NORMED         = 31,
    SUMO_ATTR_FUEL_NORMED        = 32,
    SUMO_ATTR_ELECTRICITY_NORMED = 33,
    SUMO_ATTR_CO_PERVEH          = 34,
    SUMO_ATTR_CO2_PERVEH         = 35,
    SUMO_ATTR_HC_PERVEH          = 36,
    SUMO_ATTR_PMX_PERVEH         = 37,
    SUMO_ATTR_NOX_PERVEH         = 38,
    SUMO_ATTR_FUEL_PERVEH        = 39,
    SUMO_ATTR_ELECTRICITY_PERVEH = 40,
    /// MSMeanData_Harmonoise
    SUMO_ATTR_NOISE              = 41,
    /// MSMeanData_Amitran
    SUMO_ATTR_AMOUNT             = 42,
    SUMO_ATTR_AVERAGESPEED       = 43,
    // FCD-Output
    SUMO_ATTR_X                  = 44,
    SUMO_ATTR_Y                  = 45,
    SUMO_ATTR_Z                  = 46,
    SUMO_ATTR_ANGLE              = 47,
    SUMO_ATTR_TYPE               = 48,
    SUMO_ATTR_POSITION           = 49,
    SUMO_ATTR_EDGE               = 50,
    SUMO_ATTR_LANE               = 51,
    SUMO_ATTR_SLOPE              = 52,
    SUMO_ATTR_SIGNALS            = 53,
    SUMO_ATTR_ACCELERATION       = 54,
    SUMO_ATTR_ACCELERATION_LAT   = 55,
    SUMO_ATTR_DISTANCE           = 56,
    SUMO_ATTR_LEADER_ID          = 57,
    SUMO_ATTR_LEADER_SPEED       = 58,
    SUMO_ATTR_LEADER_GAP         = 59,
    SUMO_ATTR_VEHICLE            = 60,
    SUMO_ATTR_ODOMETER           = 61,
    SUMO_ATTR_POSITION_LAT       = 62,
    SUMO_ATTR_SPEED_LAT          = 63,

    // only usable with SumoXMLAttrMask
    SUMO_ATTR_ARRIVALDELAY       = 64,

    // emission-output
    SUMO_ATTR_CO                 = 65,
    SUMO_ATTR_CO2                = 66,
    SUMO_ATTR_HC                 = 67,
    SUMO_ATTR_PMX                = 68,
    SUMO_ATTR_NOX                = 69,
    SUMO_ATTR_FUEL               = 70,
    SUMO_ATTR_ELECTRICITY        = 71,
    SUMO_ATTR_ROUTE              = 72,
    SUMO_ATTR_ECLASS             = 73,
    SUMO_ATTR_WAITING            = 74,

    /// @}

    /// @name common attributes
    /// @{
    SUMO_ATTR_ID,
    SUMO_ATTR_REFID,
    SUMO_ATTR_NAME,
    SUMO_ATTR_VERSION,
    SUMO_ATTR_PRIORITY,
    SUMO_ATTR_NUMLANES,
    SUMO_ATTR_FRICTION,
    SUMO_ATTR_ONEWAY,
    SUMO_ATTR_WIDTH,
    SUMO_ATTR_WIDTHRESOLUTION,
    SUMO_ATTR_MAXWIDTH,
    SUMO_ATTR_MINWIDTH,
    SUMO_ATTR_SIDEWALKWIDTH,
    SUMO_ATTR_BIKELANEWIDTH,
    SUMO_ATTR_REMOVE,
    SUMO_ATTR_LENGTH,
    SUMO_ATTR_BIDI,
    SUMO_ATTR_ID_BEFORE,
    SUMO_ATTR_ID_AFTER,
    SUMO_ATTR_CENTER,
    SUMO_ATTR_CENTER_X,
    SUMO_ATTR_CENTER_Y,
    SUMO_ATTR_CENTER_Z,
    /// @}

    /// @name sumo-junction attributes
    /// @{
    SUMO_ATTR_KEY,
    SUMO_ATTR_REQUESTSIZE,
    SUMO_ATTR_REQUEST,
    SUMO_ATTR_RESPONSE,
    SUMO_ATTR_PROGRAMID,
    SUMO_ATTR_PHASE,
    SUMO_ATTR_OFFSET,
    SUMO_ATTR_ENDOFFSET,
    SUMO_ATTR_INCLANES,
    SUMO_ATTR_INTLANES,
    /// @}

    /// @name the weight of a district's source or sink
    /// @{
    SUMO_ATTR_WEIGHT,
    SUMO_ATTR_NODE,
    /// @}

    /// @brief the edges of a route
    SUMO_ATTR_EDGES,

    /// @name vehicle attributes
    /// @{
    SUMO_ATTR_DEPART,
    SUMO_ATTR_DEPARTLANE,
    SUMO_ATTR_DEPARTPOS,
    SUMO_ATTR_DEPARTPOS_LAT,
    SUMO_ATTR_DEPARTSPEED,
    SUMO_ATTR_DEPARTEDGE,
    SUMO_ATTR_ARRIVALLANE,
    SUMO_ATTR_ARRIVALPOS,
    SUMO_ATTR_ARRIVALPOS_LAT,
    SUMO_ATTR_ARRIVALSPEED,
    SUMO_ATTR_ARRIVALEDGE,
    SUMO_ATTR_MAXSPEED,
    SUMO_ATTR_DESIRED_MAXSPEED,
    SUMO_ATTR_MAXSPEED_LAT,
    SUMO_ATTR_LATALIGNMENT,
    SUMO_ATTR_MINGAP_LAT,
    SUMO_ATTR_ACCEL,
    SUMO_ATTR_DECEL,
    SUMO_ATTR_EMERGENCYDECEL,
    SUMO_ATTR_APPARENTDECEL,
    SUMO_ATTR_ACTIONSTEPLENGTH,
    SUMO_ATTR_VCLASS,
    SUMO_ATTR_VCLASSES,
    SUMO_ATTR_EXCEPTIONS,
    SUMO_ATTR_REPNUMBER,
    SUMO_ATTR_SPEEDFACTOR,
    SUMO_ATTR_SPEEDDEV,
    SUMO_ATTR_LANE_CHANGE_MODEL,
    SUMO_ATTR_CAR_FOLLOW_MODEL,
    SUMO_ATTR_MINGAP,
    SUMO_ATTR_COLLISION_MINGAP_FACTOR,
    SUMO_ATTR_BOARDING_DURATION,
    SUMO_ATTR_LOADING_DURATION,
    SUMO_ATTR_BOARDING_FACTOR,
    SUMO_ATTR_SCALE,
    SUMO_ATTR_INSERTIONCHECKS,
    SUMO_ATTR_TIME_TO_TELEPORT,
    SUMO_ATTR_TIME_TO_TELEPORT_BIDI,
    SUMO_ATTR_SPEEDFACTOR_PREMATURE,
    /// @brief Class specific timing values for vehicle maneuvering through angle ranges
    SUMO_ATTR_MANEUVER_ANGLE_TIMES,
    SUMO_ATTR_PARKING_BADGES,
    /// @}

    /// @name elecHybrid output attributes
    /// @{
    /// @brief power charging from overhead wire to battery if the battery SoC is not full
    SUMO_ATTR_OVERHEADWIRECHARGINGPOWER,
    /// @}

    /// @name overheadWire attributes
    /// @{
    SUMO_ATTR_OVERHEAD_WIRE_SEGMENT,
    SUMO_ATTR_OVERHEAD_WIRE_SEGMENTS,
    /// @brief voltage of the traction substation [V]
    SUMO_ATTR_VOLTAGE,
    /// @brief a voltage source on the overhead wire segment [bool]
    SUMO_ATTR_VOLTAGESOURCE,
    /// @brief current limit of the traction substation [A]
    SUMO_ATTR_CURRENTLIMIT,
    /// @brief id of a traction substation substation
    SUMO_ATTR_SUBSTATIONID,
    /// @brief resistivity of overhead wires
    SUMO_ATTR_OVERHEAD_WIRE_RESISTIVITY,
    /// @brief forbidden lanes for overhead wire segment
    SUMO_ATTR_OVERHEAD_WIRE_FORBIDDEN,
    /// @brief overhead wire clamps for overhead wire segment
    SUMO_ATTR_OVERHEAD_WIRE_CLAMPS,
    /// @brief id of the overhead wire segment, to the start of which the overhead wire clamp is connected
    SUMO_ATTR_OVERHEAD_WIRE_CLAMP_START,
    /// @brief id of the overhead wire segment, to the end of which the overhead wire clamp is connected
    SUMO_ATTR_OVERHEAD_WIRE_CLAMP_END,
    /// @brief id of the overhead wire, to the start of which the overhead wire clamp is connected
    SUMO_ATTR_OVERHEAD_WIRECLAMP_START,
    /// @brief id of the overhead wire, to the end of which the overhead wire clamp is connected
    SUMO_ATTR_OVERHEAD_WIRECLAMP_END,
    /// @brief id of the overhead wire lane, to the start of which the overhead wire clamp is connected
    SUMO_ATTR_OVERHEAD_WIRECLAMP_LANESTART,
    /// @brief id of the overhead wire lane, to the end of which the overhead wire clamp is connected
    SUMO_ATTR_OVERHEAD_WIRECLAMP_LANEEND,
    /// @}

    /// @name charging stations attributes
    /// @{
    /// @brief charge in W/s of the Charging Stations
    SUMO_ATTR_CHARGINGPOWER,
    /// @brief Eficiency of the charge in Charging Stations
    SUMO_ATTR_EFFICIENCY,
    /// @brief Allow/disallow charge in transit in Charging Stations
    SUMO_ATTR_CHARGEINTRANSIT,
    /// @brief Delay in the charge of charging stations (different of waiting time)
    SUMO_ATTR_CHARGEDELAY,
    /// @brief Charge type (fuel or electric)
    SUMO_ATTR_CHARGETYPE,
    /// @}

    /// @name battery device parameters
    /// @{
    /// @brief Actual battery capacity
    SUMO_ATTR_ACTUALBATTERYCAPACITY, // deprecated
    SUMO_ATTR_CHARGELEVEL,
    /// @brief Maxium battery capacity
    SUMO_ATTR_MAXIMUMBATTERYCAPACITY, // deprecated
    /// @brief Maximum Power
    SUMO_ATTR_MAXIMUMCHARGERATE,
    /// @brief Charge curve axis values
    SUMO_ATTR_CHARGELEVELTABLE,
    /// @brief Charge curve
    SUMO_ATTR_CHARGECURVETABLE,
    /// @brief Maximum Power
    SUMO_ATTR_MAXIMUMPOWER,
    /// @brief Vehicle mass
    SUMO_ATTR_VEHICLEMASS,
    /// @brief Mass equivalent of rotating elements
    SUMO_ATTR_ROTATINGMASS,
    /// @brief Front surface area
    SUMO_ATTR_FRONTSURFACEAREA,
    /// @brief Air drag coefficient
    SUMO_ATTR_AIRDRAGCOEFFICIENT,
    /// @brief Internal moment of inertia
    SUMO_ATTR_INTERNALMOMENTOFINERTIA,
    /// @brief Radial drag coefficient
    SUMO_ATTR_RADIALDRAGCOEFFICIENT,
    /// @brief Roll Drag coefficient
    SUMO_ATTR_ROLLDRAGCOEFFICIENT,
    /// @brief Constant Power Intake
    SUMO_ATTR_CONSTANTPOWERINTAKE,
    /// @brief Propulsion efficiency
    SUMO_ATTR_PROPULSIONEFFICIENCY,
    /// @brief Recuperation efficiency (constant)
    SUMO_ATTR_RECUPERATIONEFFICIENCY,
    /// @brief Recuperation efficiency (by deceleration)
    SUMO_ATTR_RECUPERATIONEFFICIENCY_BY_DECELERATION,
    /// @brief Stopping threshold
    SUMO_ATTR_STOPPINGTHRESHOLD,
    /// @}

    /// @name tripinfo output attributes
    /// @{
    SUMO_ATTR_WAITINGCOUNT,
    SUMO_ATTR_STOPTIME,
    /// @}

    /// @name elecHybrid device export parameters
    /// @{
    // @brief Overhead Wire Segment ID
    SUMO_ATTR_OVERHEADWIREID,
    // @brief Traction substation ID
    SUMO_ATTR_TRACTIONSUBSTATIONID,
    // @brief Electric current drawn from overhead wire
    SUMO_ATTR_CURRENTFROMOVERHEADWIRE,
    // @brief Voltage of overhead wire (above the vehicle)
    SUMO_ATTR_VOLTAGEOFOVERHEADWIRE,
    // @brief Circuit solver alpha parameter
    SUMO_ATTR_ALPHACIRCUITSOLVER,
    // @brief Recuperate the excess energy into overhead wires (0/1)
    SUMO_ATTR_RECUPERATIONENABLE,
    /// @}

    /// @name battery export parameters
    /// @{
    /// @brief charging status
    SUMO_ATTR_CHARGING_STATUS,
    /// @brief Energy consumed
    SUMO_ATTR_ENERGYCONSUMED,
    /// @brief Total energy consumed
    SUMO_ATTR_TOTALENERGYCONSUMED,
    /// @brief Total energy regenerated
    SUMO_ATTR_TOTALENERGYREGENERATED,
    /// @brief Charging Station ID
    SUMO_ATTR_CHARGINGSTATIONID,
    /// @brief tgotal of Energy charged
    SUMO_ATTR_ENERGYCHARGED,
    /// @brief Energy charged in transit
    SUMO_ATTR_ENERGYCHARGEDINTRANSIT,
    /// @brief Energy charged stopped
    SUMO_ATTR_ENERGYCHARGEDSTOPPED,
    /// @brief Position on lane
    SUMO_ATTR_POSONLANE,
    /// @brief Time stopped
    SUMO_ATTR_TIMESTOPPED,
    /// @}

    /// @name MMPEVEM parameters
    /// @{
    /// @brief Wheel radius
    SUMO_ATTR_WHEELRADIUS,
    /// @brief Maximum torque
    SUMO_ATTR_MAXIMUMTORQUE,
    /// @brief Maximum speed
    SUMO_ATTR_MAXIMUMSPEED,
    /// @brief Gear efficiency
    SUMO_ATTR_GEAREFFICIENCY,
    /// @brief Gear ratio
    SUMO_ATTR_GEARRATIO,
    /// @brief Maximum recuperation torque
    SUMO_ATTR_MAXIMUMRECUPERATIONTORQUE,
    /// @brief Maximum recuperation power
    SUMO_ATTR_MAXIMUMRECUPERATIONPOWER,
    /// @brief Internal battery resistance
    SUMO_ATTR_INTERNALBATTERYRESISTANCE,
    /// @brief Nominal battery voltage
    SUMO_ATTR_NOMINALBATTERYVOLTAGE,
    /// @brief A string encoding the power loss map
    SUMO_ATTR_POWERLOSSMAP,
    /// @}

    /// @name chargingStations export parameters
    /// @{
    /// @brief total energy charged by charging station
    SUMO_ATTR_TOTALENERGYCHARGED,
    /// @brief number of steps that a vehicle is charging
    SUMO_ATTR_CHARGINGSTEPS,
    /// @brief total energy charged into a single vehicle
    SUMO_ATTR_TOTALENERGYCHARGED_VEHICLE,
    /// @brief timestep in which charging begins
    SUMO_ATTR_CHARGINGBEGIN,
    /// @brief timesteps in which charging ends
    SUMO_ATTR_CHARGINGEND,
    /// @brief energy provided by charging station at certain timestep
    SUMO_ATTR_PARTIALCHARGE,
    /// @brief minimum charging power encountered during the charging event
    SUMO_ATTR_MINPOWER,
    /// @brief minimum energy charged during one time step of the charging event
    SUMO_ATTR_MINCHARGE,
    /// @brief maximum energy charged during one time step of the charging event
    SUMO_ATTR_MAXCHARGE,
    /// @brief minimum charging efficiency encountered during the charging event
    SUMO_ATTR_MINEFFICIENCY,
    /// @brief maximum charging efficiency encountered during the charging event
    SUMO_ATTR_MAXEFFICIENCY,
    /// @}

    /// @name general emission / consumption parameters
    /// @{
    /// @brief engine gets switched off if remaining planned stop duration exceeds value
    SUMO_ATTR_SHUT_OFF_STOP,
    /// @brief engine gets switched off if stopping duration exceeds value
    SUMO_ATTR_SHUT_OFF_AUTO,
    /// @brief additional mass loaded on the vehicle
    SUMO_ATTR_LOADING,
    /// @}

    /// @name Car following model attributes
    /// @{
    SUMO_ATTR_SIGMA,      // used by: Krauss
    SUMO_ATTR_SIGMA_STEP, // used by: Krauss
    SUMO_ATTR_STARTUP_DELAY,
    SUMO_ATTR_TAU,
    SUMO_ATTR_TMP1,
    SUMO_ATTR_TMP2,
    SUMO_ATTR_TMP3,
    SUMO_ATTR_TMP4,
    SUMO_ATTR_TMP5,
    // Car Following Model attributes of the Extended IDM
    SUMO_ATTR_CF_EIDM_USEVEHDYNAMICS,
    SUMO_ATTR_CF_EIDM_T_LOOK_AHEAD,
    SUMO_ATTR_CF_EIDM_T_PERSISTENCE_DRIVE,
    SUMO_ATTR_CF_EIDM_T_REACTION,
    SUMO_ATTR_CF_EIDM_T_PERSISTENCE_ESTIMATE,
    SUMO_ATTR_CF_EIDM_C_COOLNESS,
    SUMO_ATTR_CF_EIDM_SIG_LEADER,
    SUMO_ATTR_CF_EIDM_SIG_GAP,
    SUMO_ATTR_CF_EIDM_SIG_ERROR,
    SUMO_ATTR_CF_EIDM_JERK_MAX,
    SUMO_ATTR_CF_EIDM_EPSILON_ACC,
    SUMO_ATTR_CF_EIDM_T_ACC_MAX,
    SUMO_ATTR_CF_EIDM_M_FLATNESS,
    SUMO_ATTR_CF_EIDM_M_BEGIN,
    SUMO_ATTR_CF_EIDM_MAX_VEH_PREVIEW,
    /// @}

    // @name Train ACC model attributes
    /// @{
    SUMO_ATTR_SC_GAIN,
    SUMO_ATTR_GCC_GAIN_SPEED,
    SUMO_ATTR_GCC_GAIN_SPACE,
    SUMO_ATTR_GC_GAIN_SPEED,
    SUMO_ATTR_GC_GAIN_SPACE,
    SUMO_ATTR_CA_GAIN_SPEED,
    SUMO_ATTR_CA_GAIN_SPACE,
    SUMO_ATTR_CA_OVERRIDE,
    /// @}

    // @name Train CACC model attributes
    /// @{
    SUMO_ATTR_SC_GAIN_CACC,
    SUMO_ATTR_GCC_GAIN_GAP_CACC,
    SUMO_ATTR_GCC_GAIN_GAP_DOT_CACC,
    SUMO_ATTR_GC_GAIN_GAP_CACC,
    SUMO_ATTR_GC_GAIN_GAP_DOT_CACC,
    SUMO_ATTR_CA_GAIN_GAP_CACC,
    SUMO_ATTR_CA_GAIN_GAP_DOT_CACC,
    SUMO_ATTR_HEADWAY_TIME_CACC_TO_ACC,
    SUMO_ATTR_SC_MIN_GAP,
    SUMO_ATTR_APPLYDRIVERSTATE,
    /// @}


    /// @name Train model attributes
    /// @{
    SUMO_ATTR_TRAIN_TYPE, //used by: Rail
    SUMO_ATTR_SPEED_TABLE, // list of speeds for traction and resistance value tables
    SUMO_ATTR_TRACTION_TABLE, // list of traction values for the speeds table
    SUMO_ATTR_RESISTANCE_TABLE, // list of resistance values for the speeds table
    SUMO_ATTR_MASSFACTOR,
    SUMO_ATTR_MAXPOWER,
    SUMO_ATTR_MAXTRACTION,
    SUMO_ATTR_RESISTANCE_COEFFICIENT_CONSTANT,
    SUMO_ATTR_RESISTANCE_COEFFICIENT_LINEAR,
    SUMO_ATTR_RESISTANCE_COEFFICIENT_QUADRATIC,

    SUMO_ATTR_LCA_STRATEGIC_PARAM,
    SUMO_ATTR_LCA_COOPERATIVE_PARAM,
    SUMO_ATTR_LCA_SPEEDGAIN_PARAM,
    SUMO_ATTR_LCA_KEEPRIGHT_PARAM,
    SUMO_ATTR_LCA_SUBLANE_PARAM,
    SUMO_ATTR_LCA_OPPOSITE_PARAM,
    SUMO_ATTR_LCA_PUSHY,
    SUMO_ATTR_LCA_PUSHYGAP,
    SUMO_ATTR_LCA_ASSERTIVE,
    SUMO_ATTR_LCA_IMPATIENCE,
    SUMO_ATTR_LCA_TIME_TO_IMPATIENCE,
    SUMO_ATTR_LCA_ACCEL_LAT,
    SUMO_ATTR_LCA_LOOKAHEADLEFT,
    SUMO_ATTR_LCA_SPEEDGAINRIGHT,
    SUMO_ATTR_LCA_SPEEDGAIN_LOOKAHEAD,
    SUMO_ATTR_LCA_COOPERATIVE_ROUNDABOUT,
    SUMO_ATTR_LCA_COOPERATIVE_SPEED,
    SUMO_ATTR_LCA_MAXSPEEDLATSTANDING,
    SUMO_ATTR_LCA_MAXSPEEDLATFACTOR,
    SUMO_ATTR_LCA_MAXDISTLATSTANDING,
    SUMO_ATTR_LCA_TURN_ALIGNMENT_DISTANCE,
    SUMO_ATTR_LCA_OVERTAKE_RIGHT,
    SUMO_ATTR_LCA_LANE_DISCIPLINE,
    SUMO_ATTR_LCA_SIGMA,
    SUMO_ATTR_LCA_KEEPRIGHT_ACCEPTANCE_TIME,
    SUMO_ATTR_LCA_OVERTAKE_DELTASPEED_FACTOR,
    SUMO_ATTR_LCA_CONTRIGHT,
    SUMO_ATTR_LCA_EXPERIMENTAL1,
    /// @}

    /// @name Junction model attributes
    /// @{
    SUMO_ATTR_JM_CROSSING_GAP,
    SUMO_ATTR_JM_DRIVE_AFTER_YELLOW_TIME,
    SUMO_ATTR_JM_DRIVE_AFTER_RED_TIME,
    SUMO_ATTR_JM_DRIVE_RED_SPEED,
    SUMO_ATTR_JM_IGNORE_KEEPCLEAR_TIME,
    SUMO_ATTR_JM_IGNORE_FOE_SPEED,
    SUMO_ATTR_JM_IGNORE_FOE_PROB,
    SUMO_ATTR_JM_IGNORE_JUNCTION_FOE_PROB,
    SUMO_ATTR_JM_SIGMA_MINOR,
    SUMO_ATTR_JM_STOPLINE_GAP,
    SUMO_ATTR_JM_STOPLINE_CROSSING_GAP,
    SUMO_ATTR_JM_TIMEGAP_MINOR,
    SUMO_ATTR_JM_EXTRA_GAP,
    SUMO_ATTR_JM_ADVANCE,
    SUMO_ATTR_JM_STOPSIGN_WAIT,
    SUMO_ATTR_JM_ALLWAYSTOP_WAIT,
    SUMO_ATTR_JM_IGNORE_IDS,
    SUMO_ATTR_JM_IGNORE_TYPES,
    SUMO_ATTR_CF_IGNORE_IDS,
    SUMO_ATTR_CF_IGNORE_TYPES,
    /// @}
    SUMO_ATTR_FLEX_ARRIVAL,

    /// @name route alternatives / distribution attributes
    /// @{
    SUMO_ATTR_LAST,
    SUMO_ATTR_COST,
    SUMO_ATTR_COSTS,
    SUMO_ATTR_SAVINGS,
    SUMO_ATTR_EXITTIMES,
    SUMO_ATTR_PROB,
    SUMO_ATTR_REPLACED_AT_TIME,
    SUMO_ATTR_REPLACED_ON_INDEX,
    SUMO_ATTR_COUNT,
    SUMO_ATTR_PROBS,
    SUMO_ATTR_ROUTES,
    SUMO_ATTR_VTYPES,
    SUMO_ATTR_NEXT_EDGES,
    SUMO_ATTR_DETERMINISTIC,
    /// @}

    /// @name trip definition attributes
    /// @{
    SUMO_ATTR_LANES,
    SUMO_ATTR_FROM,
    SUMO_ATTR_TO,
    SUMO_ATTR_FROMLONLAT,
    SUMO_ATTR_TOLONLAT,
    SUMO_ATTR_FROMXY,
    SUMO_ATTR_TOXY,
    SUMO_ATTR_FROM_JUNCTION,
    SUMO_ATTR_TO_JUNCTION,
    SUMO_ATTR_PERIOD,
    SUMO_ATTR_REPEAT,
    SUMO_ATTR_CYCLETIME,
    SUMO_ATTR_FROM_TAZ,
    SUMO_ATTR_TO_TAZ,
    SUMO_ATTR_REROUTE,
    SUMO_ATTR_PERSON_CAPACITY,
    SUMO_ATTR_CONTAINER_CAPACITY,
    SUMO_ATTR_PARKING_LENGTH,
    SUMO_ATTR_PERSON_NUMBER,
    SUMO_ATTR_CONTAINER_NUMBER,
    SUMO_ATTR_MODES,
    SUMO_ATTR_WALKFACTOR,
    /// @}

    /// @name source definitions
    /// @{
    SUMO_ATTR_FUNCTION,
    SUMO_ATTR_FREQUENCY,
    SUMO_ATTR_STYLE,
    SUMO_ATTR_FILE,
    SUMO_ATTR_LOCAL,
    SUMO_ATTR_NUMBER,
    SUMO_ATTR_DURATION,
    SUMO_ATTR_UNTIL,
    SUMO_ATTR_ARRIVAL,
    SUMO_ATTR_EXTENSION,
    SUMO_ATTR_ROUTEPROBE,
    SUMO_ATTR_STARTED,
    SUMO_ATTR_ENDED,
    /// @}

    /// @brief the edges crossed by a pedestrian crossing
    SUMO_ATTR_CROSSING_EDGES,
    /// @brief trigger: the time of the step
    SUMO_ATTR_TIME,
    /// @brief weights: time range begin
    SUMO_ATTR_BEGIN,
    /// @brief weights: time range end
    SUMO_ATTR_END,
    /// @brief link,node: the traffic light id responsible for this link
    SUMO_ATTR_TLID,
    /// @brief node: the type of traffic light
    SUMO_ATTR_TLTYPE,
    /// @brief node: the layout of the traffic light program
    SUMO_ATTR_TLLAYOUT,
    /// @brief link: the index of the link within the traffic light
    SUMO_ATTR_TLLINKINDEX,
    /// @brief link: the index of the opposite direction link of a pedestrian crossing
    SUMO_ATTR_TLLINKINDEX2,
    /// @brief edge: the shape in xml-definition
    SUMO_ATTR_SHAPE,
    /// @brief edge: the outline shape in xml-definition
    SUMO_ATTR_OUTLINESHAPE,
    /// @brief The information about how to spread the lanes from the given position
    SUMO_ATTR_SPREADTYPE,
    /// @brief The turning radius at an intersection in m
    SUMO_ATTR_RADIUS,
    /// @brief Whether vehicles must keep the junction clear
    SUMO_ATTR_KEEP_CLEAR,
    /// @brief Whether this connection is an indirect (left) turn
    SUMO_ATTR_INDIRECT,
    /// @brief How to compute right of way
    SUMO_ATTR_RIGHT_OF_WAY,
    /// @brief Fringe type of node
    SUMO_ATTR_FRINGE,
    /// @brief whether a given shape is user-defined
    SUMO_ATTR_CUSTOMSHAPE,
    /// @brief A color information
    SUMO_ATTR_COLOR,
    /// @brief The abstract direction of a link
    SUMO_ATTR_DIR,
    /// @brief The state of a link
    SUMO_ATTR_STATE,
    /// @brief The state of the lanechange model
    SUMO_ATTR_LCSTATE,
    /// @brief foe visibility distance of a link
    SUMO_ATTR_VISIBILITY_DISTANCE,
    /// @brief icon
    SUMO_ATTR_ICON,
    /// @brief A layer number
    SUMO_ATTR_LAYER,
    /// @brief Fill the polygon
    SUMO_ATTR_FILL,
    SUMO_ATTR_LINEWIDTH,
    SUMO_ATTR_PREFIX,
    SUMO_ATTR_DISCARD,

    SUMO_ATTR_FROM_LANE,
    SUMO_ATTR_TO_LANE,
    SUMO_ATTR_DEST,
    SUMO_ATTR_SOURCE,
    SUMO_ATTR_VIA,
    SUMO_ATTR_VIALONLAT,
    SUMO_ATTR_VIAXY,
    SUMO_ATTR_VIAJUNCTIONS,
    /// @brief a list of node ids, used for controlling joining
    SUMO_ATTR_NODES,

    /// @name Attributes for actuated traffic lights:
    /// @{
    /// @brief minimum duration of a phase
    SUMO_ATTR_MINDURATION,
    /// @brief maximum duration of a phase
    SUMO_ATTR_MAXDURATION,
    /// @brief The minimum time within the cycle for switching (for coordinated actuation)
    SUMO_ATTR_EARLIEST_END,
    /// @brief The maximum time within the cycle for switching (for coordinated actuation)
    SUMO_ATTR_LATEST_END,
    /// @brief The condition expression for an early switch into this phase
    SUMO_ATTR_EARLY_TARGET,
    /// @brief The condition expression for switching into this phase when the active phase must end
    SUMO_ATTR_FINAL_TARGET,
    /// @brief The expression for a condition assignment
    SUMO_ATTR_CHECK,
    /// @brief The number of arguments for a condition function
    SUMO_ATTR_NARGS,
    /// @brief vehicle extension time of a phase
    SUMO_ATTR_VEHICLEEXTENSION,
    /// @brief yellow duration of a phase
    SUMO_ATTR_YELLOW,
    /// @brief red duration of a phase
    SUMO_ATTR_RED,
    /// @brief succesor phase index
    SUMO_ATTR_NEXT,
    /// @}

    /// @name Attributes for junction-internal lanes
    /// @{
    /// @brief Information within the junction logic which internal lanes block external
    SUMO_ATTR_FOES,
    /// @}
    SUMO_ATTR_CONSTRAINTS,
    SUMO_ATTR_RAIL,

    SUMO_ATTR_DETECTORS,
    SUMO_ATTR_CONDITIONS,
    SUMO_ATTR_SAVE_DETECTORS,
    SUMO_ATTR_SAVE_CONDITIONS,

    /// @name Attributes for detectors
    /// @{
    /// @brief Information whether the detector shall be continued on the folowing lanes
    SUMO_ATTR_CONT,
    SUMO_ATTR_CONTPOS,
    SUMO_ATTR_HALTING_TIME_THRESHOLD,
    SUMO_ATTR_HALTING_SPEED_THRESHOLD,
    SUMO_ATTR_JAM_DIST_THRESHOLD,
    SUMO_ATTR_SHOW_DETECTOR,
    SUMO_ATTR_OPEN_ENTRY,
    SUMO_ATTR_EXPECT_ARRIVAL,
    /// @}

    SUMO_ATTR_WAUT_ID,
    SUMO_ATTR_JUNCTION_ID,
    SUMO_ATTR_PROCEDURE,
    SUMO_ATTR_SYNCHRON,
    SUMO_ATTR_REF_TIME,
    SUMO_ATTR_START_PROG,

    SUMO_ATTR_OFF,
    SUMO_ATTR_FRIENDLY_POS,
    SUMO_ATTR_SPLIT_VTYPE,
    SUMO_ATTR_UNCONTROLLED,
    SUMO_ATTR_PASS,
    SUMO_ATTR_BUS_STOP,
    SUMO_ATTR_TRAIN_STOP,
    SUMO_ATTR_CONTAINER_STOP,
    SUMO_ATTR_PARKING_AREA,
    SUMO_ATTR_ROADSIDE_CAPACITY,
    SUMO_ATTR_ACCEPTED_BADGES,
    SUMO_ATTR_ONROAD,
    SUMO_ATTR_CHARGING_STATION,
    SUMO_ATTR_GROUP,
    SUMO_ATTR_LINE,
    SUMO_ATTR_LINES,
    SUMO_ATTR_TRIP_ID,
    SUMO_ATTR_SPLIT,
    SUMO_ATTR_JOIN,
    SUMO_ATTR_INTENDED,
    SUMO_ATTR_ONDEMAND,
    SUMO_ATTR_JUMP,
    SUMO_ATTR_USED_ENDED,
    SUMO_ATTR_COLLISION,
    SUMO_ATTR_VALUE,
    SUMO_ATTR_PROHIBITOR,
    SUMO_ATTR_PROHIBITED,
    SUMO_ATTR_ALLOW,
    SUMO_ATTR_DISALLOW,
    SUMO_ATTR_PREFER,
    SUMO_ATTR_CHANGE_LEFT,
    SUMO_ATTR_CHANGE_RIGHT,
    SUMO_ATTR_CONTROLLED_INNER,
    SUMO_ATTR_VEHSPERHOUR,
    SUMO_ATTR_PERSONSPERHOUR,
    SUMO_ATTR_CONTAINERSPERHOUR,
    SUMO_ATTR_PERHOUR,
    SUMO_ATTR_DONE,
    SUMO_ATTR_OUTPUT,
    SUMO_ATTR_HEIGHT,
    SUMO_ATTR_GUISHAPE,
    SUMO_ATTR_OSGFILE,
    SUMO_ATTR_IMGFILE,
    SUMO_ATTR_RELATIVEPATH,
    SUMO_ATTR_EMISSIONCLASS,
    SUMO_ATTR_MASS,
    SUMO_ATTR_IMPATIENCE,
    SUMO_ATTR_STARTPOS,
    SUMO_ATTR_ENDPOS,
    SUMO_ATTR_TRIGGERED,
    SUMO_ATTR_CONTAINER_TRIGGERED,
    SUMO_ATTR_PARKING,
    SUMO_ATTR_EXPECTED,
    SUMO_ATTR_PERMITTED,
    SUMO_ATTR_EXPECTED_CONTAINERS,
    SUMO_ATTR_INDEX,

    SUMO_ATTR_ENTERING,
    SUMO_ATTR_EXCLUDE_EMPTY,
    SUMO_ATTR_WITH_INTERNAL,
    SUMO_ATTR_TRACK_VEHICLES,
    SUMO_ATTR_DETECT_PERSONS,
    SUMO_ATTR_MAX_TRAVELTIME,
    SUMO_ATTR_MIN_SAMPLES,
    SUMO_ATTR_WRITE_ATTRIBUTES,
    SUMO_ATTR_EDGESFILE,
    SUMO_ATTR_AGGREGATE,
    SUMO_ATTR_NUMEDGES,

    SUMO_ATTR_LON,
    SUMO_ATTR_LAT,
    SUMO_ATTR_ACTION,
    SUMO_ATTR_GEO,
    SUMO_ATTR_GEOSHAPE,
    SUMO_ATTR_K,
    SUMO_ATTR_V,
    SUMO_ATTR_REF,
    SUMO_ATTR_HREF,
    SUMO_ATTR_ZOOM,
    SUMO_ATTR_INTERPOLATED,
    SUMO_ATTR_THRESHOLD,

    SUMO_ATTR_NET_OFFSET,
    SUMO_ATTR_CONV_BOUNDARY,
    SUMO_ATTR_ORIG_BOUNDARY,
    SUMO_ATTR_ORIG_PROJ,

    /// @name car-following model attributes
    /// @{
    SUMO_ATTR_CF_PWAGNER2009_TAULAST,
    SUMO_ATTR_CF_PWAGNER2009_APPROB,
    SUMO_ATTR_CF_IDM_DELTA,
    SUMO_ATTR_CF_IDM_STEPPING,
    SUMO_ATTR_CF_IDMM_ADAPT_FACTOR,
    SUMO_ATTR_CF_IDMM_ADAPT_TIME,
    SUMO_ATTR_CF_KERNER_PHI,
    SUMO_ATTR_CF_WIEDEMANN_SECURITY,
    SUMO_ATTR_CF_WIEDEMANN_ESTIMATION,
    SUMO_ATTR_CF_W99_CC1,
    SUMO_ATTR_CF_W99_CC2,
    SUMO_ATTR_CF_W99_CC3,
    SUMO_ATTR_CF_W99_CC4,
    SUMO_ATTR_CF_W99_CC5,
    SUMO_ATTR_CF_W99_CC6,
    SUMO_ATTR_CF_W99_CC7,
    SUMO_ATTR_CF_W99_CC8,
    SUMO_ATTR_CF_W99_CC9,

    SUMO_ATTR_CF_CC_CCDECEL,
    SUMO_ATTR_CF_CC_CONSTSPACING,
    SUMO_ATTR_CF_CC_KP,
    SUMO_ATTR_CF_CC_LAMBDA,
    SUMO_ATTR_CF_CC_C1,
    SUMO_ATTR_CF_CC_XI,
    SUMO_ATTR_CF_CC_OMEGAN,
    SUMO_ATTR_CF_CC_TAU,
    SUMO_ATTR_CF_CC_LANES_COUNT,
    SUMO_ATTR_CF_CC_CCACCEL,
    SUMO_ATTR_CF_CC_PLOEG_KP,
    SUMO_ATTR_CF_CC_PLOEG_KD,
    SUMO_ATTR_CF_CC_PLOEG_H,
    SUMO_ATTR_CF_CC_FLATBED_KA,
    SUMO_ATTR_CF_CC_FLATBED_KV,
    SUMO_ATTR_CF_CC_FLATBED_KP,
    SUMO_ATTR_CF_CC_FLATBED_D,
    SUMO_ATTR_CF_CC_FLATBED_H,
    /// @}

    SUMO_ATTR_GENERATE_WALKS,
    SUMO_ATTR_ACTTYPE,
    SUMO_ATTR_CORNERDETAIL,
    SUMO_ATTR_LINKDETAIL,
    SUMO_ATTR_RECTANGULAR_LANE_CUT,
    SUMO_ATTR_WALKINGAREAS,
    SUMO_ATTR_LEFTHAND,
    SUMO_ATTR_LIMIT_TURN_SPEED,
    SUMO_ATTR_CHECKLANEFOES_ALL,
    SUMO_ATTR_CHECKLANEFOES_ROUNDABOUT,
    SUMO_ATTR_TLS_IGNORE_INTERNAL_JUNCTION_JAM,
    SUMO_ATTR_AVOID_OVERLAP,
    SUMO_ATTR_HIGHER_SPEED,
    SUMO_ATTR_INTERNAL_JUNCTIONS_VEHICLE_WIDTH,
    SUMO_ATTR_JUNCTIONS_MINIMAL_SHAPE,
    SUMO_ATTR_JUNCTIONS_ENDPOINT_SHAPE,
    SUMO_ATTR_COMMAND,

    SUMO_ATTR_ACTORCONFIG,
    SUMO_ATTR_STARTTIME,
    SUMO_ATTR_VEHICLECLASS,
    SUMO_ATTR_ORIGIN,
    SUMO_ATTR_DESTINATION,
    SUMO_ATTR_VISIBLE,
    SUMO_ATTR_LIMIT,
    SUMO_ATTR_ACTIVE,
    SUMO_ATTR_ARRIVALTIME,
    SUMO_ATTR_ARRIVALTIMEBRAKING,
    SUMO_ATTR_ARRIVALSPEEDBRAKING,
    SUMO_ATTR_OPTIONAL,
    SUMO_ATTR_VEHICLES,

    /// @name ActivityGen Tags
    /// @{

    /// @name general object
    /// @{
    AGEN_ATTR_INHABITANTS,
    AGEN_ATTR_HOUSEHOLDS,
    AGEN_ATTR_CHILDREN,
    AGEN_ATTR_RETIREMENT,
    AGEN_ATTR_CARS,
    AGEN_ATTR_UNEMPLOYEMENT,
    AGEN_ATTR_LABORDEMAND,
    AGEN_ATTR_MAX_FOOT_DIST,
    AGEN_ATTR_IN_TRAFFIC,
    AGEN_ATTR_OUT_TRAFFIC,
    /// @}

    /// @name streets object
    /// @{
    //SUMO_ATTR_EDGE already defined
    AGEN_ATTR_POPULATION,
    AGEN_ATTR_OUT_WORKPOSITION,
    /// @}

    /// @name workHours object
    /// @{
    AGEN_ATTR_HOUR,
    AGEN_ATTR_PROP,
    /// @}

    /// @name school object
    /// @{
    //SUMO_ATTR_EDGE, SUMO_ATTR_POSITION, SUMO_ATTR_TYPE already defined
    AGEN_ATTR_CAPACITY,
    AGEN_ATTR_OPENING,
    AGEN_ATTR_CLOSING,
    /// @}

    /// @name busStation and Bus objects
    /// @{
    // ID, EDGE, POSITION, REFID, BEGIN and END are already defined
    AGEN_ATTR_MAX_TRIP_DURATION,
    //AGEN_ATTR_ORDER,
    AGEN_ATTR_RATE,
    /// @}

    /// @name population and children accompaniment brackets
    /// @{
    AGEN_ATTR_BEGINAGE,
    AGEN_ATTR_ENDAGE,
    AGEN_ATTR_PEOPLENBR,
    /// @}

    /// @name parameters
    /// @{
    AGEN_ATTR_CARPREF,
    AGEN_ATTR_CITYSPEED,
    AGEN_ATTR_FREETIMERATE,
    AGEN_ATTR_UNI_RAND_TRAFFIC,
    AGEN_ATTR_DEP_VARIATION,
    /// @}

    /// @name city gates
    /// @{
    AGEN_ATTR_INCOMING,
    AGEN_ATTR_OUTGOING,
    /// @}
    /// @}

    /// @name SUMOConfig elements
    /// @brief config file
    SUMO_ATTR_CONFIGFILE,
    /// @brief net file
    SUMO_ATTR_NETFILE,
    /// @brief additional files
    SUMO_ATTR_ADDITIONALFILES,
    /// @brief route files
    SUMO_ATTR_ROUTEFILES,
    /// @brief route files
    SUMO_ATTR_DATAFILES,
    /// @brief meanData files
    SUMO_ATTR_MEANDATAFILES,
    ///@}

    //@name Netedit Attributes (used as virtual property holders, must be in SumoXMLAttr)
    //@{
    /// @brief element is selected
    GNE_ATTR_SELECTED,
    /// @brief whether a feature has been loaded,guessed,modified or approved
    GNE_ATTR_MODIFICATION_STATUS,
    /// @brief first coordinate of edge shape
    GNE_ATTR_SHAPE_START,
    /// @brief last coordinate of edge shape
    GNE_ATTR_SHAPE_END,
    /// @brief whether an edge is part of a bidirectional railway
    GNE_ATTR_BIDIR,
    /// @brief Close shape of a polygon (Used by GNEPolys)
    GNE_ATTR_CLOSE_SHAPE,
    /// @brief parent of an additional element
    GNE_ATTR_PARENT,
    /// @brief data set of a generic data
    GNE_ATTR_DATASET,
    /// @brief parameters "key1=value1|key2=value2|...|keyN=valueN"
    GNE_ATTR_PARAMETERS,
    /// @brief flow parameters (integer for mask end, number, etc...)
    GNE_ATTR_FLOWPARAMETERS,
    /// @brief min source (used only by TAZs)
    GNE_ATTR_MIN_SOURCE,
    /// @brief min sink (used only by TAZs)
    GNE_ATTR_MIN_SINK,
    /// @brief max source (used only by TAZs)
    GNE_ATTR_MAX_SOURCE,
    /// @brief max sink (used only by TAZs)
    GNE_ATTR_MAX_SINK,
    /// @brief average source (used only by TAZs)
    GNE_ATTR_AVERAGE_SOURCE,
    /// @brief average sink (used only by TAZs)
    GNE_ATTR_AVERAGE_SINK,
    /// @brief Color of TAZSources/TAZSinks
    GNE_ATTR_TAZCOLOR,
    /// @brief Flag to check if VType is a default VType
    GNE_ATTR_DEFAULT_VTYPE,
    /// @brief Flag to check if a default VType was modified
    GNE_ATTR_DEFAULT_VTYPE_MODIFIED,
    /// @brief flag to center camera after element creation
    GNE_ATTR_CENTER_AFTER_CREATION,
    /// @brief to busStop (used by personPlans)
    GNE_ATTR_OPPOSITE,
    /// @brief shift lane index (only used by elements over lanes)
    GNE_ATTR_SHIFTLANEINDEX,
    /// @brief stop offset (virtual, used by edge and lanes)
    GNE_ATTR_STOPOFFSET,
    /// @brief stop exceptions (virtual, used by edge and lanes)
    GNE_ATTR_STOPOEXCEPTION,
    /// @brief route distribution
    GNE_ATTR_ROUTE_DISTRIBUTION,
    /// @brief vehicle type distribution
    GNE_ATTR_VTYPE_DISTRIBUTION,
    /// @brief poisson definition (used in flow)
    GNE_ATTR_POISSON,
    /// @brief stop index (position in the parent's children
    GNE_ATTR_STOPINDEX,
    /// @brief stop index (position in the parent's path)
    GNE_ATTR_PATHSTOPINDEX,
    /// @brief check number of additional children (used in vTypeDistribution)
    GNE_ATTR_ADDITIONALCHILDREN,
    /// @brief person/container geometry start position
    GNE_ATTR_PLAN_GEOMETRY_STARTPOS,
    /// @brief person/container geometry end position
    GNE_ATTR_PLAN_GEOMETRY_ENDPOS,
    /// @brief from lane ID (used in GNEConnection)
    GNE_ATTR_FROM_LANEID,
    /// @brief to lane ID (used in GNEConnection)
    GNE_ATTR_TO_LANEID,
    /// @brief TAZ Center (uses to return the TAZ centroid if center is not defined)
    GNE_ATTR_TAZ_CENTROID,

    // virtual attributes for easier UI
    GNE_ATTR_FROM_BUSSTOP,
    GNE_ATTR_FROM_TRAINSTOP,
    GNE_ATTR_FROM_CONTAINERSTOP,
    GNE_ATTR_FROM_CHARGINGSTATION,
    GNE_ATTR_FROM_PARKINGAREA,
    GNE_ATTR_FROM_ROUTE,
    GNE_ATTR_IS_ROUNDABOUT,
    // @}

    /// @name train parameters
    /// @{
    SUMO_ATTR_CARRIAGE_LENGTH,
    SUMO_ATTR_LOCOMOTIVE_LENGTH,
    SUMO_ATTR_CARRIAGE_GAP,
    SUMO_ATTR_CARRIAGE_DOORS,
    /// @}

    SUMO_ATTR_TARGETLANE,
    SUMO_ATTR_CROSSING,
    SUMO_ATTR_XMLNS,
    SUMO_ATTR_SCHEMA_LOCATION,

    //@name RNG state saving attributes
    // @{
    SUMO_ATTR_RNG_ROUTEHANDLER,
    SUMO_ATTR_RNG_INSERTIONCONTROL,
    SUMO_ATTR_RNG_DEVICE,
    SUMO_ATTR_RNG_DEVICE_BT,
    SUMO_ATTR_RNG_DEVICE_TOC,
    SUMO_ATTR_RNG_DRIVERSTATE,
    // @}

    //@name meso edge type attributes
    // @{
    SUMO_ATTR_MESO_TAUFF,
    SUMO_ATTR_MESO_TAUFJ,
    SUMO_ATTR_MESO_TAUJF,
    SUMO_ATTR_MESO_TAUJJ,
    SUMO_ATTR_MESO_JUNCTION_CONTROL,
    SUMO_ATTR_MESO_TLS_PENALTY,
    SUMO_ATTR_MESO_TLS_FLOW_PENALTY,
    SUMO_ATTR_MESO_MINOR_PENALTY,
    SUMO_ATTR_MESO_OVERTAKING,
    // @}
    /// @brief invalid attribute, must be the last one
    SUMO_ATTR_NOTHING,
};

/*
 * @brief definitions of special SumoXML-attribute values.
 * Since these enums shall be used in switch statements we keep them separated
 * @{
 */

/**
 * SumoXMLParam Key Constants. Since all usage contexts needs strings rather
 * than enum values we do not bother with a StringBijection
 */
extern const std::string SUMO_PARAM_ORIGID;

/**
 * @enum SumoXMLNodeType
 * @brief Numbers representing special SUMO-XML-attribute values
 * for representing node- (junction-) types used in netbuild/netimport and netload
 */
enum class SumoXMLNodeType {
    UNKNOWN, // terminator
    TRAFFIC_LIGHT,
    TRAFFIC_LIGHT_NOJUNCTION, // junction controlled only by traffic light but without other prohibitions,
    TRAFFIC_LIGHT_RIGHT_ON_RED,
    RAIL_SIGNAL,
    RAIL_CROSSING,
    PRIORITY,
    PRIORITY_STOP, // like priority but all minor links have stop signs
    RIGHT_BEFORE_LEFT,
    LEFT_BEFORE_RIGHT,
    ALLWAY_STOP,
    ZIPPER,
    DISTRICT,
    NOJUNCTION,
    INTERNAL,
    DEAD_END,
    DEAD_END_DEPRECATED
};


/**
 * @enum SumoXMLEdgeFunc
 * @brief Numbers representing special SUMO-XML-attribute values
 * for representing edge functions used in netbuild/netimport and netload
 */
enum class SumoXMLEdgeFunc {
    UNKNOWN,
    NORMAL,
    CONNECTOR,
    CROSSING,
    WALKINGAREA,
    INTERNAL
};


/**
 * @enum LaneSpreadFunction
 * @brief Numbers representing special SUMO-XML-attribute values
 * Information how the edge's lateral offset shall be computed
 * In dependence to this value, lanes will be spread to the right side or
 * to both sides from the given edge geometry (Also used when node
 * positions are used as edge geometry).
 */
enum class LaneSpreadFunction {
    RIGHT = 0,
    ROADCENTER = 1,
    CENTER = 2
};

/**
 * @enum ParkingType
 * @brief Numbers representing special SUMO-XML-attribute values
 * Information on whether a car is parking on the road or to the side of it
 */
enum class ParkingType {
    ONROAD = 0,
    OFFROAD = 1,
    OPPORTUNISTIC = 2
};

/// @brief algorithms for computing right of way
enum class RightOfWay {
    DEFAULT,
    EDGEPRIORITY,
    MIXEDPRIORITY,
    ALLWAYSTOP,
};

/// @brief classifying boundary nodes
enum class FringeType {
    // outer border of the network
    OUTER,
    // fringe edge within the network (i.e. due to pruning some road types)
    INNER,
    // not fringe (nothing is cut off)
    DEFAULT
};

/// @brief travel modes for persons
enum class PersonMode {
    NONE = 0,
    WALK_FORWARD = 1,
    WALK_BACKWARD = 2,
    WALK = 3,
    BICYCLE = 1 << 2,
    CAR = 1 << 3,
    PUBLIC = 1 << 4,
    TAXI = 1 << 5
};

/**
 * @enum LinkState
 * @brief The right-of-way state of a link between two lanes
 * used when constructing a NBTrafficLightLogic, in MSLink and GNEInternalLane
 *
 * This enumerations holds the possible right-of-way rules a link
 *  may have. Beyond the righ-of-way rules, this enumeration also
 *  holds the possible traffic light states.
 *
 *  enum values are assigned so that chars can be cast back to linkstates
 *  @todo fix redundancy
 */
enum LinkState {
    /// @brief The link has green light, may pass
    LINKSTATE_TL_GREEN_MAJOR = 'G',
    /// @brief The link has green light, has to brake
    LINKSTATE_TL_GREEN_MINOR = 'g',
    /// @brief The link has red light (must brake)
    LINKSTATE_TL_RED = 'r',
    /// @brief The link has red light (must brake) but indicates upcoming green
    LINKSTATE_TL_REDYELLOW = 'u',
    /// @brief The link has yellow light, may pass
    LINKSTATE_TL_YELLOW_MAJOR = 'Y',
    /// @brief The link has yellow light, has to brake anyway
    LINKSTATE_TL_YELLOW_MINOR = 'y',
    /// @brief The link is controlled by a tls which is off and blinks, has to brake
    LINKSTATE_TL_OFF_BLINKING = 'o',
    /// @brief The link is controlled by a tls which is off, not blinking, may pass
    LINKSTATE_TL_OFF_NOSIGNAL = 'O',
    /// @brief This is an uncontrolled, major link, may pass
    LINKSTATE_MAJOR = 'M',
    /// @brief This is an uncontrolled, minor link, has to brake
    LINKSTATE_MINOR = 'm',
    /// @brief This is an uncontrolled, right-before-left link
    LINKSTATE_EQUAL = '=',
    /// @brief This is an uncontrolled, minor link, has to stop
    LINKSTATE_STOP = 's',
    /// @brief This is an uncontrolled, all-way stop link.
    LINKSTATE_ALLWAY_STOP = 'w',
    /// @brief This is an uncontrolled, zipper-merge link
    LINKSTATE_ZIPPER = 'Z',
    /// @brief This is a dead end link
    LINKSTATE_DEADEND = '-'
};

/**
 * @enum LinkDirection
 * @brief The different directions a link between two lanes may take (or a
 * stream between two edges).
 * used in netbuild (formerly NBMMLDirection) and MSLink
 */
enum class LinkDirection {
    /// @brief The link is a (hard) right direction
    RIGHT = 1 << 0,
    /// @brief The link is a partial right direction
    PARTRIGHT = 1 << 1,
    /// @brief The link is a straight direction
    STRAIGHT = 1 << 2,
    /// @brief The link is a partial left direction
    PARTLEFT = 1 << 3,
    /// @brief The link is a (hard) left direction
    LEFT = 1 << 4,
    /// @brief The link is a 180 degree turn
    TURN = 1 << 5,
    /// @brief The link is a 180 degree turn (left-hand network)
    TURN_LEFTHAND = 1 << 6,
    /// @brief The link has no direction (is a dead end link)
    NODIR = 1 << 7
};


/// @enum TrafficLightType
enum class TrafficLightType {
    STATIC,
    RAIL_SIGNAL,
    RAIL_CROSSING,
    ACTUATED,
    NEMA,
    DELAYBASED,
    SOTL_PHASE,
    SOTL_PLATOON,
    SOTL_REQUEST,
    SOTL_WAVE,
    SOTL_MARCHING,
    SWARM_BASED,
    HILVL_DETERMINISTIC,
    OFF,
    INVALID //< must be the last one
};

/// @enum TrafficLightLayout
enum class TrafficLightLayout {
    OPPOSITES,
    INCOMING,
    ALTERNATE_ONEWAY,
    DEFAULT //< must be the last one
};


/// @brief different checking levels for vehicle insertion
enum class InsertionCheck {
    NONE = 0,
    COLLISION = 1 << 0,
    LEADER_GAP = 1 << 1,
    FOLLOWER_GAP = 1 << 2,
    JUNCTION = 1 << 3,
    STOP = 1 << 4,
    ARRIVAL_SPEED = 1 << 5,
    ONCOMING_TRAIN = 1 << 6,
    SPEED_LIMIT = 1 << 7,
    PEDESTRIAN = 1 << 8,
    BIDI = 1 << 9,
    LANECHANGE = 1 << 10,
    ALL = ((1 << 11) - 1) // <- must be the last one
};


/** @enum LaneChangeAction
 * @brief The state of a vehicle's lane-change behavior
 */
enum LaneChangeAction {
    /// @name currently wanted lane-change action
    /// @{
    /// @brief No action desired
    LCA_NONE = 0,
    /// @brief Needs to stay on the current lane
    LCA_STAY = 1 << 0,
    /// @brief Wants go to the left
    LCA_LEFT = 1 << 1,
    /// @brief Wants go to the right
    LCA_RIGHT = 1 << 2,
    /// @brief The action is needed to follow the route (navigational lc)
    LCA_STRATEGIC = 1 << 3,
    /// @brief The action is done to help someone else
    LCA_COOPERATIVE = 1 << 4,
    /// @brief The action is due to the wish to be faster (tactical lc)
    LCA_SPEEDGAIN = 1 << 5,
    /// @brief The action is due to the default of keeping right "Rechtsfahrgebot"
    LCA_KEEPRIGHT = 1 << 6,
    /// @brief The action is due to a TraCI request
    LCA_TRACI = 1 << 7,
    /// @brief The action is urgent (to be defined by lc-model)
    LCA_URGENT = 1 << 8,
    /// @brief The action has not been determined
    LCA_UNKNOWN = 1 << 30,
    /// @}

    /// @name External state
    /// @{
    /// @brief The vehicle is blocked by left leader
    LCA_BLOCKED_BY_LEFT_LEADER = 1 << 9,
    /// @brief The vehicle is blocked by left follower
    LCA_BLOCKED_BY_LEFT_FOLLOWER = 1 << 10,
    /// @brief The vehicle is blocked by right leader
    LCA_BLOCKED_BY_RIGHT_LEADER = 1 << 11,
    /// @brief The vehicle is blocked by right follower
    LCA_BLOCKED_BY_RIGHT_FOLLOWER = 1 << 12,
    /// @brief The vehicle is blocked being overlapping
    LCA_OVERLAPPING = 1 << 13,
    /// @brief The vehicle does not have enough space to complete a continuous change before the next turn
    LCA_INSUFFICIENT_SPACE = 1 << 14,
    /// @brief used by the sublane model
    LCA_SUBLANE = 1 << 15,
    /// @brief Vehicle is too slow to complete a continuous lane change (in case that maxSpeedLatStanding==0)
    LCA_INSUFFICIENT_SPEED = 1 << 28,
    /// @brief lane can change
    LCA_WANTS_LANECHANGE = LCA_LEFT | LCA_RIGHT,
    /// @brief lane can change or stay
    LCA_WANTS_LANECHANGE_OR_STAY = LCA_WANTS_LANECHANGE | LCA_STAY,
    /// @brief blocked left
    LCA_BLOCKED_LEFT = LCA_BLOCKED_BY_LEFT_LEADER | LCA_BLOCKED_BY_LEFT_FOLLOWER,
    /// @brief blocked right
    LCA_BLOCKED_RIGHT = LCA_BLOCKED_BY_RIGHT_LEADER | LCA_BLOCKED_BY_RIGHT_FOLLOWER,
    /// @brief blocked by leader
    LCA_BLOCKED_BY_LEADER = LCA_BLOCKED_BY_LEFT_LEADER | LCA_BLOCKED_BY_RIGHT_LEADER,
    /// @brief blocker by follower
    LCA_BLOCKED_BY_FOLLOWER = LCA_BLOCKED_BY_LEFT_FOLLOWER | LCA_BLOCKED_BY_RIGHT_FOLLOWER,
    /// @brief blocked in all directions
    LCA_BLOCKED = LCA_BLOCKED_LEFT | LCA_BLOCKED_RIGHT | LCA_INSUFFICIENT_SPACE | LCA_INSUFFICIENT_SPEED,
    /// @brief reasons of lane change
    LCA_CHANGE_REASONS = (LCA_STRATEGIC | LCA_COOPERATIVE | LCA_SPEEDGAIN | LCA_KEEPRIGHT | LCA_SUBLANE | LCA_TRACI),
    // LCA_BLOCKED_BY_CURRENT_LEADER = 1 << 28
    // LCA_BLOCKED_BY_CURRENT_FOLLOWER = 1 << 29
    /// @}

    /// @name originally model specific states (migrated here since
    ///       they were duplicated in all current models)
    /// @{
    LCA_AMBLOCKINGLEADER = 1 << 16,
    LCA_AMBLOCKINGFOLLOWER = 1 << 17,
    LCA_MRIGHT = 1 << 18,
    LCA_MLEFT = 1 << 19,
    // !!! never set LCA_UNBLOCK = 1 << 20,
    LCA_AMBLOCKINGFOLLOWER_DONTBRAKE = 1 << 21,
    // !!! never used LCA_AMBLOCKINGSECONDFOLLOWER = 1 << 22,
    LCA_CHANGE_TO_HELP = 1 << 23,
    // !!! never read LCA_KEEP1 = 1 << 24,
    // !!! never used LCA_KEEP2 = 1 << 25,
    LCA_AMBACKBLOCKER = 1 << 26,
    LCA_AMBACKBLOCKER_STANDING = 1 << 27
                                 /// @}
};


/// @enum LaneChangeModel
enum class LaneChangeModel {
    DK2008,
    LC2013,
    LC2013_CC,
    SL2015,
    DEFAULT
};

/// @enum train types
enum class TrainType {
    CUSTOM,
    NGT400,
    NGT400_16,
    RB425,
    RB628,
    ICE1,
    REDOSTO7,
    FREIGHT,
    ICE3,
    MIREOPLUSB,
    MIREOPLUSH,
    UNKNOWN
};

/// @brief POI icons
enum class POIIcon {
    PIN = 1,
    NATURE = 2,
    HOTEL = 3,
    FUEL = 4,
    CHARGING_STATION = 5,
    NONE = 0,
};

// @}

/**
 * @class SUMOXMLDefinitions
 * @brief class for maintaining associations between enums and xml-strings
 */
class SUMOXMLDefinitions {

public:
    /// @brief The names of SUMO-XML elements (for passing to GenericSAXHandler)
    static SequentialStringBijection::Entry tags[];

    /// @brief The names of SUMO-XML attributes (for passing to GenericSAXHandler)
    static SequentialStringBijection::Entry attrs[];

    /// @brief The names of SUMO-XML elements for use in netbuild
    static SequentialStringBijection Tags;

    /// @brief The names of SUMO-XML attributes for use in netbuild
    static SequentialStringBijection Attrs;

    /// @name Special values of SUMO-XML attributes
    /// @{

    /// @brief node types
    static StringBijection<SumoXMLNodeType> NodeTypes;

    /// @brief edge functions
    static StringBijection<SumoXMLEdgeFunc> EdgeFunctions;

    /// @brief lane spread functions
    static StringBijection<LaneSpreadFunction> LaneSpreadFunctions;

    /// @brief parking types
    static StringBijection<ParkingType> ParkingTypes;

    /// @brief righ of way algorithms
    static StringBijection<RightOfWay> RightOfWayValues;

    /// @brief fringe types
    static StringBijection<FringeType> FringeTypeValues;

    /// @brief person modes
    static StringBijection<PersonMode> PersonModeValues;

    /// @brief link states
    static StringBijection<LinkState> LinkStates;

    /// @brief link directions
    static StringBijection<LinkDirection> LinkDirections;

    /// @brief traffic light types
    static StringBijection<TrafficLightType> TrafficLightTypes;

    /// @brief traffic light layouts
    static StringBijection<TrafficLightLayout> TrafficLightLayouts;

    /// @brief traffic light layouts
    static StringBijection<InsertionCheck> InsertionChecks;

    /// @brief lane change models
    static StringBijection<LaneChangeModel> LaneChangeModels;

    /// @brief car following models
    static StringBijection<SumoXMLTag> CarFollowModels;

    /// @brief lane change actions
    static StringBijection<LaneChangeAction> LaneChangeActions;

    /// @brief train types
    static StringBijection<TrainType> TrainTypes;

    /// @brief POI icon values
    static StringBijection<POIIcon> POIIcons;
    /// @}

    /// @name Helper functions for ID-string manipulations
    /// @{

    /// @brief whether the given string is a valid id for a network element
    static bool isValidNetID(const std::string& value);

    /// @brief whether the given string is a valid id for a vehicle or flow
    static bool isValidVehicleID(const std::string& value);

    /// @brief whether the given string is a valid id for an edge or vehicle type
    static bool isValidTypeID(const std::string& value);

    /// @brief whether the given string is a valid id for an additional object
    static bool isValidAdditionalID(const std::string& value);

    /// @brief whether the given string is a valid id for an detector
    static bool isValidDetectorID(const std::string& value);

    /// @brief whether the given string is a valid attribute for a certain key (for example, a name)
    static bool isValidAttribute(const std::string& value);

    /// @brief whether the given string is a valid attribute for a filename (for example, a name)
    static bool isValidFilename(const std::string& value);

    /// @brief whether the given string is a valid list of id for a network (empty aren't allowed)
    static bool isValidListOfNetIDs(const std::string& value);

    /// @brief whether the given string is a valid list of ids for an edge or vehicle type (empty aren't allowed)
    static bool isValidListOfTypeID(const std::string& value);

    /// @brief whether the given list of strings is a valid list of ids for an edge or vehicle type (empty aren't allowed)
    static bool isValidListOfTypeID(const std::vector<std::string>& typeIDs);

    /// @brief whether the given string is a valid key for a parameter
    static bool isValidParameterKey(const std::string& value);

    /// @brief transform the given string into a valid id for a network element (or any other id)
    // Replaces all invalid characters for a netID by an underscore, this may violate uniqueness!
    // Since the netID has the most restrictions, the resulting ID should be valid for the other use cases too
    static std::string makeValidID(const std::string& value);

    /// @brief return the junction id when given an edge of type internal, crossing or WalkingArea
    static std::string getJunctionIDFromInternalEdge(const std::string internalEdge);

    /// @brief return edge id when given the lane ID
    static std::string getEdgeIDFromLane(const std::string laneID);

    /// @brief return lane index when given the lane ID
    static int getIndexFromLane(const std::string laneID);
    /// @}

    /// @brief all allowed characters for phase state
    static const std::string ALLOWED_TLS_LINKSTATES;

private:
    /// @brief containers for the different SUMOXMLDefinitions
    /// @{

    /// @brief node type values
    static StringBijection<SumoXMLNodeType>::Entry sumoNodeTypeValues[];

    /// @brief edge function values
    static StringBijection<SumoXMLEdgeFunc>::Entry sumoEdgeFuncValues[];

    /// @brief lane spread function values
    static StringBijection<LaneSpreadFunction>::Entry laneSpreadFunctionValues[];

    /// @brief lane spread function values
    static StringBijection<ParkingType>::Entry parkingTypeValues[];

    /// @brief lane spread function values
    static StringBijection<RightOfWay>::Entry rightOfWayValuesInitializer[];

    /// @brief lane spread function values
    static StringBijection<FringeType>::Entry fringeTypeValuesInitializer[];

    /// @brief person mode values
    static StringBijection<PersonMode>::Entry personModeValuesInitializer[];

    /// @brief link state values
    static StringBijection<LinkState>::Entry linkStateValues[];

    /// @brief link direction values
    static StringBijection<LinkDirection>::Entry linkDirectionValues[];

    /// @brief traffic light types values
    static StringBijection<TrafficLightType>::Entry trafficLightTypesValues[];

    /// @brief traffic light layout values
    static StringBijection<TrafficLightLayout>::Entry trafficLightLayoutValues[];

    /// @brief traffic light layout values
    static StringBijection<InsertionCheck>::Entry insertionCheckValues[];

    /// @brief lane change model values
    static StringBijection<LaneChangeModel>::Entry laneChangeModelValues[];

    /// @brief car follwing model values
    static StringBijection<SumoXMLTag>::Entry carFollowModelValues[];

    /// @brief lane change action values
    static StringBijection<LaneChangeAction>::Entry laneChangeActionValues[];

    /// @brief train type values
    static StringBijection<TrainType>::Entry trainTypeValues[];

    /// @brief POI icon values
    static StringBijection<POIIcon>::Entry POIIconValues[];

    /// @}

    /// @brief all allowed characters for phase state
    static const char AllowedTLS_linkStatesInitializer[];
};
