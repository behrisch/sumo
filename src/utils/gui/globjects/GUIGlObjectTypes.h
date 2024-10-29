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
/// @file    GUIGlObjectTypes.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
///
// A list of object types which may be displayed within the gui
// each type has an associated string which will be prefefixed to an object id
// when constructing the full name
/****************************************************************************/
#pragma once
#include <config.h>


// ===========================================================================
// definitions
// ===========================================================================
/**
 * ChooseableArtifact
 * Enumeration to differ to show the list of which artefact
 * The order is important during the selection of items for displaying their
 * tooltips; the item with the hightest type value will be chosen.
 */
enum GUIGlObjectType {
    /// @brief The network - empty
    GLO_NETWORK = 0,

    /// @name nettork elements
    /// @{

    /// @brief reserved GLO type to pack all network elements
    GLO_NETWORKELEMENT = 1,
    /// @brief an edge
    GLO_EDGE = 2,
    /// @brief a lane
    GLO_LANE = 3,
    /// @brief a junction
    GLO_JUNCTION = 4,
    /// @brief a walkingArea
    GLO_WALKINGAREA = 5,
    /// @brief a connection
    GLO_CONNECTION = 6,
    /// @brief a tl-logic
    GLO_CROSSING = 7,
    /// @brief a tl-logic
    GLO_TLLOGIC = 8,
    /// @brief an edgeType
    GLO_EDGETYPE = 9,
    /// @brief a laneType
    GLO_LANETYPE = 10,

    /// @}

    /// @brief line between parent and childrens
    GLO_PARENTCHILDLINE = 90,

    /// @name additional elements
    /// @{

    /// @brief reserved GLO type for packing all additionals elements
    GLO_ADDITIONALELEMENT = 100,
    /// @brief reserved GLO type for packing all StoppingPlace elements
    GLO_STOPPING_PLACE = 101,
    /// @brief a busStop
    GLO_BUS_STOP = 102,
    /// @brief a trainStop
    GLO_TRAIN_STOP = 103,
    /// @brief a containerStop
    GLO_CONTAINER_STOP = 104,
    /// @brief a chargingStation
    GLO_CHARGING_STATION = 105,
    /// @brief a ParkingArea
    GLO_PARKING_AREA = 106,
    /// @brief reserved GLO type for packing all StoppingPlace elements
    GLO_STOPPING_PLACE_LAST = 107,
    /// @brief a ParkingSpace
    GLO_PARKING_SPACE = 108,
    /// @brief a E1 detector
    GLO_E1DETECTOR = 109,
    /// @brief a E1 detector
    GLO_E1DETECTOR_ME = 110,
    /// @brief a E1 detector
    GLO_E1DETECTOR_INSTANT = 111,
    /// @brief a E2 detector
    GLO_E2DETECTOR = 112,
    /// @brief a E3 detector
    GLO_E3DETECTOR = 113,
    /// @brief a DetEntry detector
    GLO_DET_ENTRY = 114,
    /// @brief a DetExit detector
    GLO_DET_EXIT = 115,
    /// @brief a Rerouter
    GLO_REROUTER = 116,
    /// @brief a rerouter interval
    GLO_REROUTER_INTERVAL = 117,
    /// @brief a closing reroute
    GLO_REROUTER_CLOSINGREROUTE = 118,
    /// @brief a closing lane reroute
    GLO_REROUTER_CLOSINGLANEREROUTE = 119,
    /// @brief a parking area reroute
    GLO_REROUTER_PARKINGAREAREROUTE = 120,
    /// @brief a destination probability reroute
    GLO_REROUTER_DESTPROBREROUTE = 121,
    /// @brief a route probability reroute
    GLO_REROUTER_ROUTEPROBREROUTE = 122,
    /// @brief a Rerouter over edge
    GLO_REROUTER_EDGE = 123,
    /// @brief a Variable Speed Sign
    GLO_VSS = 124,
    /// @brief a Variable Speed Sign step
    GLO_VSS_STEP = 125,
    /// @brief a Calibrator
    GLO_CALIBRATOR = 126,
    /// @brief a RouteProbe
    GLO_ROUTEPROBE = 127,
    /// @brief a Vaporizer
    GLO_VAPORIZER = 128,
    /// @brief a Acces
    GLO_ACCESS = 129,
    /// @brief reserved GLO type for packing all wire elements
    GLO_WIRE = 140,
    /// @brief a segment of an overhead line
    GLO_OVERHEAD_WIRE_SEGMENT = 141,
    /// @brief a segment of an overhead line
    GLO_TRACTIONSUBSTATION = 142,
    /// @brief reserved GLO type for packing all additionals elements
    GLO_ADDITIONALELEMENT_LAST = 189,

    /// @}

    /// @brief lane details
    GLO_LANEARROWS = 190,

    /// @name shape elements
    /// @{

    /// @brief reserved GLO type to pack shapes
    GLO_SHAPE = 200,
    /// @brief polygon
    GLO_POLYGON = 201,
    /// @brief poi (over view, geo and lane)
    GLO_POI = 202,
    /// @brief walkable area
    GLO_JPS_WALKABLEAREA = 203,
    /// @brief obstacles
    GLO_JPS_OBSTACLE = 204,
    /// @brief reserved GLO type to pack shapes
    GLO_SHAPE_LAST = 299,

    /// @}

    /// @name demand elements
    /// @{

    /// @brief reserved GLO type to pack all RouteElements (note: In this case the sorting of GLO_<element> is important!)
    GLO_ROUTEELEMENT = 300,
    /// @bief vTypes
    GLO_VTYPE = 301,

    /// @}

    /// @name routes
    /// @{

    /// @brief a route
    GLO_ROUTE = 310,
    /// @brief a route
    GLO_ROUTE_EMBEDDED = 311,

    /// @}

    /// @name Person plans
    /// @{

    /// @brief a ride
    GLO_RIDE = 320,
    /// @brief a walk
    GLO_WALK = 321,
    /// @brief a person trip
    GLO_PERSONTRIP = 322,
    /// @brief a container transport
    GLO_TRANSPORT = 323,
    /// @brief a container tranship
    GLO_TRANSHIP = 324,

    /// @}

    /// @name stops
    /// @{

    /// @brief a stop
    GLO_STOP = 330,
    /// @brief a stop plan stop
    GLO_STOP_PLAN = 331,

    /// @}

    /// @name vehicles
    /// @{

    /// @brief a vehicle
    GLO_VEHICLE = 340,
    /// @brief a trip
    GLO_TRIP = 341,
    /// @brief a flow
    GLO_FLOW = 342,
    /// @brief a routeFlow
    GLO_ROUTEFLOW = 343,

    /// @}

    /// @name containers (carried by vehicles)
    /// @{

    /// @brief a container
    GLO_CONTAINER = 350,
    /// @brief a person flow
    GLO_CONTAINERFLOW = 351,

    /// @}

    /// @name persons
    /// @{

    /// @brief a person
    GLO_PERSON = 360,
    /// @brief a person flow
    GLO_PERSONFLOW = 361,

    /// @}

    /// @brief Traffic Assignment Zones (TAZs)
    GLO_TAZ = 400,

    /// @name data elements
    /// @{

    /// @brief edge data
    GLO_EDGEDATA = 500,
    /// @brief edge relation data
    GLO_EDGERELDATA = 501,
    /// @brief TAZ relation data
    GLO_TAZRELDATA = 502,

    /// @}

    /// @name other
    /// @{

    /// @brief Lock icon (used in netedit)
    GLO_LOCKICON = 1000,

    /// @brief geometry point (used in netedit)
    GLO_GEOMETRYPOINT = 1020,

    /// @brief front element (used in netedit)
    GLO_FRONTELEMENT = 1030,

    /// @brief dotted contour (used in netedit)
    GLO_DOTTEDCONTOUR = 1040,

    /// @brief temporal shape (used in netedit)
    GLO_TEMPORALSHAPE = 1050,

    /// @brief text element (used in netedit)
    GLO_TEXTNAME = 1055,

    /// @brief rectangle selection shape (used in netedit)
    GLO_RECTANGLESELECTION = 1060,

    /// @brief test element (used in netedit)
    GLO_TESTELEMENT = 1070,

    /// @}

    /// @brief empty max
    GLO_MAX = 2048
};
