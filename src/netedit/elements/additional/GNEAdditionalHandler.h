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
/// @file    GNEAdditionalHandler.h
/// @author  Pablo Alvarez Lopez
/// @date    Nov 2015
///
// Builds additional objects for netedit
/****************************************************************************/
#pragma once
#include <config.h>

#include <utils/common/SUMOVehicleClass.h>
#include <utils/handlers/AdditionalHandler.h>


// ===========================================================================
// class declarations
// ===========================================================================
class GNENet;
class GNEEdge;
class GNELane;
class GNEAdditional;

// ===========================================================================
// class definitions
// ===========================================================================

/// @class GNEAdditionalHandler
/// @brief Builds additional objects for GNENet (busStops, chargingStations, detectors, etc..)
class GNEAdditionalHandler : public AdditionalHandler {

public:
    /// @brief Constructor
    GNEAdditionalHandler(GNENet* net, const bool allowUndoRedo, const bool overwrite);

    /// @brief Destructor
    ~GNEAdditionalHandler();

    /// @name build functions
    /// @{
    /**@brief Builds a bus stop
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The id of the bus stop
     * @param[in] laneID The lane the bus stop is placed on
     * @param[in] startPos Begin position of the bus stop on the lane
     * @param[in] endPos End position of the bus stop on the lane
     * @param[in] name Name of busStop
     * @param[in] lines Names of the bus lines that halt on this bus stop
     * @param[in] personCapacity larger numbers of persons trying to enter will create an upstream jam on the sidewalk.
     * @param[in] parkingLength parking length
     * @param[in[ color busStop color
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] parameters generic parameters
     */
    void buildBusStop(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const std::string& laneID,
                      const double startPos, const double endPos, const std::string& name, const std::vector<std::string>& lines,
                      const int personCapacity, const double parkingLength, const RGBColor& color, const bool friendlyPosition,
                      const Parameterised::Map& parameters);

    /**@brief Builds a train stop
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The id of the train stop
     * @param[in] laneID The lane the train stop is placed on
     * @param[in] startPos Begin position of the train stop on the lane
     * @param[in] endPos End position of the train stop on the lane
     * @param[in] name Name of trainStop
     * @param[in] lines Names of the train lines that halt on this train stop
     * @param[in] personCapacity larger numbers of persons trying to enter will create an upstream jam on the sidewalk.
     * @param[in] parkingLength parking length
     * @param[in[ color trainStop color
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] parameters generic parameters
     */
    void buildTrainStop(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const std::string& laneID,
                        const double startPos, const double endPos, const std::string& name, const std::vector<std::string>& lines,
                        const int personCapacity, const double parkingLength, const RGBColor& color, const bool friendlyPosition,
                        const Parameterised::Map& parameters);

    /**@brief Builds an Access
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] busStop GNEAdditional of this Access belongs
     * @param[in] laneID The lane the Access is placed on
     * @param[in] pos position of the Access on the lane
     * @param[in[ length length of the Access
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] parameters generic parameters
     */
    void buildAccess(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& laneID, const std::string& pos,
                     const double length, const bool friendlyPos, const Parameterised::Map& parameters);

    /**@brief Builds a container stop
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The id of the container stop
     * @param[in] laneID The lane the container stop is placed on
     * @param[in] startPos Begin position of the container stop on the lane
     * @param[in] endPos End position of the container stop on the lane
     * @param[in] name Name of container stop
     * @param[in] lines Names of the bus lines that halt on this container stop
     * @param[in] containerCapacity larger numbers of containers
     * @param[in] parkingLength parking length
     * @param[in[ color containerStop color
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] parameters generic parameters
     */
    void buildContainerStop(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const std::string& laneID,
                            const double startPos, const double endPos, const std::string& name, const std::vector<std::string>& lines,
                            const int containerCapacity, const double parkingLength, const RGBColor& color, const bool friendlyPosition,
                            const Parameterised::Map& parameters);

    /**@brief Builds a charging Station
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The id of the charging Station
     * @param[in] laneID The lane the charging Station is placed on
     * @param[in] startPos Begin position of the charging Station on the lane
     * @param[in] endPos End position of the charging Station on the lane
     * @param[in] name Name of charging station
     * @param[in] chargingPower power charged in every timeStep
     * @param[in] efficiency efficiency of the charge
     * @param[in] chargeInTransit enable or disable charge in transit
     * @param[in] chargeDelay delay in the charge
     * @param[in] chargeType charge type (normal, electric or fuel)
     * @param[in] waitingTime waiting time until start charging
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] parameters generic parameters
     */
    void buildChargingStation(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const std::string& laneID,
                              const double startPos, const double endPos, const std::string& name, const double chargingPower,
                              const double efficiency, const bool chargeInTransit, const SUMOTime chargeDelay, const std::string& chargeType,
                              const SUMOTime waitingTime, const bool friendlyPosition, const std::string& parkingAreaID, const Parameterised::Map& parameters);

    /**@brief Builds a Parking Area
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The id of the Parking >Area
     * @param[in] laneID The lane the Parking Area is placed on
     * @param[in] startPos Begin position of the Parking Area on the lane
     * @param[in] endPos End position of the Parking Area on the lane
     * @param[in] departPos lane position in that vehicle must depart when leaves parkingArea
     * @param[in] name Name of Parking Area
     * @param[in] badges names which grant access to the parkingArea
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] roadSideCapacity road side capacity of ParkingArea
     * @param[in] width ParkingArea's width
     * @param[in] length ParkingArea's length
     * @param[in] angle ParkingArea's angle
     * @param[in] lefthand enable or disable lefthand
     * @param[in] parameters generic parameters
     */
    void buildParkingArea(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const std::string& laneID,
                          const double startPos, const double endPos, const std::string& departPos, const std::string& name,
                          const std::vector<std::string>& badges, const bool friendlyPosition, const int roadSideCapacity, const bool onRoad,
                          const double width, const double length, const double angle, const bool lefthand, const Parameterised::Map& parameters);

    /**@brief Builds a Parking Space
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] x Position X
     * @param[in] y Position Y
     * @param[in] z Position Z
     * @param[in] name Name of Parking Area
     * @param[in] width ParkingArea's width
     * @param[in] length ParkingArea's length
     * @param[in] angle ParkingArea's angle
     * @param[in] slope ParkingArea's slope (of this space)
     * @param[in] parameters generic parameters
     */
    void buildParkingSpace(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const double x, const double y, const double z,
                           const std::string& name, const std::string& width, const std::string& length, const std::string& angle,
                           const double slope, const Parameterised::Map& parameters);

    /**@brief Builds a induction loop detector (E1)
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The id of the detector
     * @param[in] laneID The lane the detector is placed on
     * @param[in] pos position of the detector on the lane
     * @param[in] period the aggregation period the values the detector collects shall be summed up.
     * @param[in] filename The path to the output file.
     * @param[in] vtypes list of vehicle types to be reported
     * @param[in] nextEdges list of edge ids that must all be part of the future route of the vehicle to qualify for detection
     * @param[in] detectPersons detect persons instead of vehicles (pedestrians or passengers)
     * @param[in] name E1 detector name
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] parameters generic parameters
     */
    void buildE1Detector(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const std::string& laneID,
                         const double position, const SUMOTime period, const std::string& file, const std::vector<std::string>& vehicleTypes,
                         const std::vector<std::string>& nextEdges, const std::string& detectPersons, const std::string& name,
                         const bool friendlyPos, const Parameterised::Map& parameters);

    /**@brief Builds a single-lane Area Detector (E2)
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The id of the detector
     * @param[in] laneID The lane the detector is placed on
     * @param[in] pos position of the detector on the lane
     * @param[in[ length length of the detector
     * @param[in] period the aggregation period the values the detector collects shall be summed up.
     * @param[in] trafficLight The traffic light that triggers aggregation when switching.
     * @param[in] filename The path to the output file.
     * @param[in] vtypes list of vehicle types to be reported
     * @param[in] nextEdges list of edge ids that must all be part of the future route of the vehicle to qualify for detection
     * @param[in] detectPersons detect persons instead of vehicles (pedestrians or passengers)
     * @param[in] name E2 detector name
     * @param[in] timeThreshold The time-based threshold that describes how much time has to pass until a vehicle is recognized as halting
     * @param[in] speedThreshold The speed-based threshold that describes how slow a vehicle has to be to be recognized as halting
     * @param[in] jamThreshold The minimum distance to the next standing vehicle in order to make this vehicle count as a participant to the jam
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] show detector in sumo-gui
     * @param[in] parameters generic parameters
     */
    void buildSingleLaneDetectorE2(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const std::string& laneID,
                                   const double pos, const double length, const SUMOTime period, const std::string& trafficLight, const std::string& filename,
                                   const std::vector<std::string>& vehicleTypes, const std::vector<std::string>& nextEdges, const std::string& detectPersons,
                                   const std::string& name, const SUMOTime timeThreshold, const double speedThreshold, const double jamThreshold,
                                   const bool friendlyPos, const bool show, const Parameterised::Map& parameters);

    /**@brief Builds a multi-lane Area Detector (E2)
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The id of the detector
     * @param[in] laneIDs The lanes the detector is placed on
     * @param[in] pos position of the detector on the first lane
     * @param[in] endPos position of the detector on the last lane
     * @param[in] period the aggregation period the values the detector collects shall be summed up.
     * @param[in] trafficLight The traffic light that triggers aggregation when switching.
     * @param[in] filename The path to the output file.
     * @param[in] vtypes list of vehicle types to be reported
     * @param[in] nextEdges list of edge ids that must all be part of the future route of the vehicle to qualify for detection
     * @param[in] detectPersons detect persons instead of vehicles (pedestrians or passengers)
     * @param[in] name E2 detector name
     * @param[in] timeThreshold The time-based threshold that describes how much time has to pass until a vehicle is recognized as halting
     * @param[in] speedThreshold The speed-based threshold that describes how slow a vehicle has to be to be recognized as halting
     * @param[in] jamThreshold The minimum distance to the next standing vehicle in order to make this vehicle count as a participant to the jam
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] show detector in sumo-gui
     * @param[in] parameters generic parameters
     */
    void buildMultiLaneDetectorE2(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const std::vector<std::string>& laneIDs,
                                  const double pos, const double endPos, const SUMOTime period, const std::string& trafficLight, const std::string& filename,
                                  const std::vector<std::string>& vehicleTypes, const std::vector<std::string>& nextEdges, const std::string& detectPersons,
                                  const std::string& name, const SUMOTime timeThreshold, const double speedThreshold, const double jamThreshold,
                                  const bool friendlyPos, const bool show, const Parameterised::Map& parameters);

    /**@brief Builds a multi entry exit detector (E3)
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The id of the detector
     * @param[in] pos position of the detector in the map
     * @param[in] period the aggregation period the values the detector collects shall be summed up.
     * @param[in] filename The path to the output file.
     * @param[in] vtypes list of vehicle types to be reported
     * @param[in] nextEdges list of edge ids that must all be part of the future route of the vehicle to qualify for detection
     * @param[in] detectPersons detect persons instead of vehicles (pedestrians or passengers)
     * @param[in] name E2 detector name
     * @param[in] timeThreshold The time-based threshold that describes how much time has to pass until a vehicle is recognized as halting
     * @param[in] speedThreshold The speed-based threshold that describes how slow a vehicle has to be to be recognized as halting
     * @param[in] openEntry enable or disable open Entry
     * @param[in] expectedArrival Whether no warning should be issued when a vehicle arrives within the detector area
     * @param[in] parameters generic parameters
     */
    void buildDetectorE3(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const Position& pos, const SUMOTime period,
                         const std::string& filename, const std::vector<std::string>& vehicleTypes, const std::vector<std::string>& nextEdges,
                         const std::string& detectPersons, const std::string& name, const SUMOTime timeThreshold, const double speedThreshold,
                         const bool openEntry, const bool expectedArrival, const Parameterised::Map& parameters);

    /**@brief Builds a entry detector (E3)
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] laneID The lane in which the entry detector is placed on
     * @param[in] pos position of the entry detector on the lane
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] parameters generic parameters
     */
    void buildDetectorEntry(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& laneID, const double pos, const bool friendlyPos,
                            const Parameterised::Map& parameters);

    /**@brief Builds a exit detector (E3)
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] laneID The lane in which the exit detector is placed on
     * @param[in] pos position of the exit detector on the lane
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] parameters generic parameters
     */
    void buildDetectorExit(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& laneID, const double pos, const bool friendlyPos,
                           const Parameterised::Map& parameters);

    /**@brief Builds a Instant Induction Loop Detector (E1Instant)
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The id of the detector
     * @param[in] laneID The lane the detector is placed on
     * @param[in] pos position of the detector on the lane
     * @param[in] filename The path to the output file.
     * @param[in] name E2 detector name
     * @param[in] vtypes list of vehicle types to be reported
     * @param[in] nextEdges list of edge ids that must all be part of the future route of the vehicle to qualify for detection
     * @param[in] detectPersons detect persons instead of vehicles (pedestrians or passengers)
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] parameters generic parameters
     */
    void buildDetectorE1Instant(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const std::string& laneID,
                                const double pos, const std::string& filename, const std::vector<std::string>& vehicleTypes,
                                const std::vector<std::string>& nextEdges, const std::string& detectPersons, const std::string& name,
                                const bool friendlyPos, const Parameterised::Map& parameters);

    /**@brief builds a microscopic calibrator over a lane
     * @param[in] id The id of the calibrator
     * @param[in] laneID The lane the calibrator is placed at
     * @param[in] pos The position on the edge the calibrator lies at
     * @param[in] name Calibrator name
     * @param[in] outfile te file in which write results
     * @param[in] jamThreshold A threshold value to detect and clear unexpected jamming
     * @param[in] vTypes space separated list of vehicle type ids to consider
     * @param[in] parameters generic parameters
     */
    void buildLaneCalibrator(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const std::string& laneID,
                             const double pos, const std::string& name, const std::string& outfile, const SUMOTime period, const std::string& routeprobe,
                             const double jamThreshold, const std::vector<std::string>& vTypes, const Parameterised::Map& parameters);

    /**@brief builds a microscopic calibrator over an edge
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The id of the calibrator
     * @param[in] edgeID The edge the calibrator is placed at
     * @param[in] pos The position on the edge the calibrator lies at
     * @param[in] name Calibrator name
     * @param[in] outfile te file in which write results
     * @param[in] routeProbe route probe vinculated with this calibrator
     * @param[in] jamThreshold A threshold value to detect and clear unexpected jamming
     * @param[in] vTypes space separated list of vehicle type ids to consider
     * @param[in] parameters generic parameters
     */
    void buildEdgeCalibrator(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const std::string& edgeID,
                             const double pos, const std::string& name, const std::string& outfile, const SUMOTime period, const std::string& routeprobe,
                             const double jamThreshold, const std::vector<std::string>& vTypes, const Parameterised::Map& parameters);

    /**@brief builds a calibrator flow
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] vehicleParameter calibratorFlow parameter
     */
    void buildCalibratorFlow(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const SUMOVehicleParameter& vehicleParameter) ;

    /**@brief builds a rerouter
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The id of the rerouter
     * @param[in] pos position of the rerouter in the map
     * @param[in] edgeIDs The edges the rerouter is placed at
     * @param[in] prob The probability the rerouter reroutes vehicles with
     * @param[in] name Calibrator name
     * @param[in] parameters generic parameters
     */
    void buildRerouter(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const Position& pos,
                       const std::vector<std::string>& edgeIDs, const double prob, const std::string& name, const bool off, const bool optional,
                       const SUMOTime timeThreshold, const std::vector<std::string>& vTypes, const Parameterised::Map& parameters);

    /**@brief builds a rerouter interval
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] begin begin of interval
     * @param[in] end end of interval
     */
    void buildRerouterInterval(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const SUMOTime begin, const SUMOTime end);

    /**@brief builds a closing lane reroute
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] closedLane closed lane
     * @param[in] permissions vClasses disallowed for the lane
     */
    void buildClosingLaneReroute(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& closedLane, SVCPermissions permissions);

    /**@brief builds a closing edge reroute
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] closedEdgeID closed edge
     * @param[in] permissions vClasses disallowed for the lane
     */
    void buildClosingReroute(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& closedEdgeID, SVCPermissions permissions);

    /**@brief builds a dest prob reroute
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] newEdgeDestinationID new edge destination ID
     * @param[in] probability rerouting probability
     */
    void buildDestProbReroute(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& newEdgeDestinationID, const double probability);

    /**@brief builds a parking area reroute
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] newParkignAreaID new parkingArea ID
     * @param[in] probability rerouting probability
     */
    void buildParkingAreaReroute(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& newParkignAreaID, const double probability, const bool visible);

    /**@brief builds a route prob reroute
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] newRouteID new route ID
     * @param[in] probability rerouting probability
     */
    void buildRouteProbReroute(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& newRouteID, const double probability);

    /**@brief builds a Route probe
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The id of the routeprobe
     * @param[in] edgeID The edges the routeprobe is placed at
     * @param[in] period the aggregation period the values the routeprobe collects shall be summed up.
     * @param[in] name Calibrator name
     * @param[in] file The file to read the routeprobe definitions from
     * @param[in] begin The time at which to start generating output
     * @param[in] vTypes list of vehicle types to be affected
     * @param[in] parameters generic parameters
     */
    void buildRouteProbe(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const std::string& edgeID, const SUMOTime period,
                         const std::string& name, const std::string& file, const SUMOTime begin, const std::vector<std::string>& vTypes,
                         const Parameterised::Map& parameters);

    /**@brief Builds a VariableSpeedSign (lane speed additional)
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The id of the lane speed additional
     * @param[in] destLanes List of lanes affected by this speed additional
     * @param[in] name VSS name
     * @param[in] vTypes list of vehicle types to be affected
     * @param[in] parameters generic parameters
     */
    void buildVariableSpeedSign(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const Position& pos,
                                const std::vector<std::string>& laneIDs, const std::string& name, const std::vector<std::string>& vTypes,
                                const Parameterised::Map& parameters);

    /**@brief Builds a VariableSpeedSign Step
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] time step's time
     * @param[in] speed step's speed
     */
    void buildVariableSpeedSignStep(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const SUMOTime time, const std::string& speed);

    /**@brief Builds a vaporizer (lane speed additional)
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] edgeID edge in which this vaporizer is placed
     * @param[in] from time in which this vaporizer start
     * @param[in] endTime time in which this vaporizer ends
     * @param[in] name Vaporizer name
     * @param[in] parameters generic parameters
     */
    void buildVaporizer(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& edgeID, const SUMOTime from,
                        const SUMOTime endTime, const std::string& name, const Parameterised::Map& parameters);

    /**@brief Builds a TAZ (Traffic Assignment Zone)
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id TAZ ID
     * @param[in] shape TAZ shape
     * @param[in] center shape center
     * @param[in] fill Whether the TAZ shall be filled
     * @param[in] edgeIDs list of edges (note: This will create GNETAZSourceSinks/Sinks with default values)
     * @param[in] name Vaporizer name
     * @param[in] parameters generic parameters
     */
    void buildTAZ(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const PositionVector& shape,
                  const Position& center, const bool fill, const RGBColor& color, const std::vector<std::string>& edgeIDs,
                  const std::string& name, const Parameterised::Map& parameters);

    /**@brief Builds a TAZSource (Traffic Assignment Zone)
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] TAZ Traffic Assignment Zone in which this TAZSource is palced
     * @param[in] edgeID edge in which TAZSource is placed
     * @param[in] departWeight depart weight of TAZSource
     */
    void buildTAZSource(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& edgeID, const double departWeight);

    /**@brief Builds a TAZSink (Traffic Assignment Zone)
     * @param[in] net net in which element will be inserted
     * @param[in] allowUndoRedo enable or disable remove created additional with ctrl + Z / ctrl + Y
     * @param[in] TAZ Traffic Assignment Zone in which this TAZSink is palced
     * @param[in] edgeID edge in which TAZSink is placed
     * @param[in] arrivalWeight arrival weight of TAZSink
     */
    void buildTAZSink(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& edgeID, const double arrivalWeight);

    /**@brief build traction substation
     * @param[in] id Traction substation ID
     * @param[in] pos Position of traction substation in view (optional)
     * @param[in] voltage Voltage of at connection point for the overhead wire
     * @param[in] currentLimit Current limit of the feeder line
     * @param[in] parameters generic parameters
     */
    void buildTractionSubstation(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const Position& pos,
                                 const double voltage, const double currentLimit, const Parameterised::Map& parameters);

    /** @brief build overhead wire
     * @param[in] id Overhead wire ID
     * @param[in] lane Lane over which the segment is placed
     * @param[in] substationId Substation to which the circuit is connected
     * @param[in] laneIDs list of consecutive lanes of the circuit
     * @param[in] startPos Starting position in the specified lane
     * @param[in] endPos Ending position in the specified lane
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] forbiddenInnerLanes Inner lanes, where placing of overhead wire is restricted
     * @param[in] parameters generic parameters
     */
    void buildOverheadWire(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const std::string& substationId,
                           const std::vector<std::string>& laneIDs, const double startPos, const double endPos, const bool friendlyPos,
                           const std::vector<std::string>& forbiddenInnerLanes, const Parameterised::Map& parameters);

    /** @brief build overhead wire clamp
     * @param[in] id Overhead wire clamp ID
     * @param[in] overheadWireIDStartClamp ID of the overhead wire segment, to the start of which the overhead wire clamp is connected
     * @param[in] laneIDStartClamp ID of the overhead wire segment lane of overheadWireIDStartClamp
     * @param[in] overheadWireIDEndClamp ID of the overhead wire segment, to the end of which the overhead wire clamp is connected
     * @param[in] laneIDEndClamp ID of the overhead wire segment lane of overheadWireIDEndClamp
     * @param[in] parameters generic parameters
     */
    void buildOverheadWireClamp(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const std::string& overheadWireIDStartClamp,
                                const std::string& laneIDStartClamp, const std::string& overheadWireIDEndClamp, const std::string& laneIDEndClamp,
                                const Parameterised::Map& parameters);

    /**@brief Builds a polygon using the given values
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The name of the polygon
     * @param[in] type The (abstract) type of the polygon
     * @param[in] color The color of the polygon
     * @param[in] layer The layer of the polygon
     * @param[in] angle The rotation of the polygon
     * @param[in] imgFile The raster image of the polygon
     * @param[in] relativePath set image file as relative path
     * @param[in] shape The shape of the polygon
     * @param[in] geo specify if shape was loaded as GEO coordinate
     * @param[in] fill Whether the polygon shall be filled
     * @param[in] lineWidth Line width when drawing unfilled polygon
     * @param[in] name polygon name
     * @param[in] parameters generic parameters
     */
    void buildPolygon(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const std::string& type,
                      const RGBColor& color, const double layer, const double angle, const std::string& imgFile, const bool relativePath,
                      const PositionVector& shape, const bool geo, const bool fill, const double lineWidth, const std::string& name,
                      const Parameterised::Map& parameters);

    /**@brief Builds a POI using the given values
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The name of the POI
     * @param[in] type The (abstract) type of the POI
     * @param[in] color The color of the POI
     * @param[in] x POI's x position
     * @param[in] y POI's y position
     * @param[in] icon The icon of the POI
     * @param[in] layer The layer of the POI
     * @param[in] angle The rotation of the POI
     * @param[in] imgFile The raster image of the POI
     * @param[in] relativePath set image file as relative path
     * @param[in] width The width of the POI image
     * @param[in] height The height of the POI image
     * @param[in] name POI name
     * @param[in] parameters generic parameters
     */
    void buildPOI(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const std::string& type,
                  const RGBColor& color, const double x, const double y, const std::string& icon, const double layer,
                  const double angle, const std::string& imgFile, bool relativePath, const double width, const double height,
                  const std::string& name, const Parameterised::Map& parameters);

    /**@brief Builds a POI over lane using the given values
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The name of the POI
     * @param[in] type The (abstract) type of the POI
     * @param[in] color The color of the POI
     * @param[in] laneID The Lane in which this POI is placed
     * @param[in] posOverLane The position over Lane
     * @param[in] friendlyPos enable or disable friendly position
     * @param[in] posLat The position lateral over Lane
     * @param[in] icon The icon of the POI
     * @param[in] layer The layer of the POI
     * @param[in] angle The rotation of the POI
     * @param[in] imgFile The raster image of the POI
     * @param[in] relativePath set image file as relative path
     * @param[in] width The width of the POI image
     * @param[in] height The height of the POI image
     * @param[in] name POI name
     * @param[in] parameters generic parameters
     */
    void buildPOILane(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const std::string& type, const RGBColor& color,
                      const std::string& laneID, const double posOverLane, const bool friendlyPos, const double posLat, const std::string& icon,
                      const double layer, const double angle, const std::string& imgFile, const bool relativePath, const double width,
                      const double height, const std::string& name, const Parameterised::Map& parameters);

    /**@brief Builds a POI in GEO coordinaten using the given values
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The name of the POI
     * @param[in] type The (abstract) type of the POI
     * @param[in] color The color of the POI
     * @param[in] lon POI's longitude
     * @param[in] lat POI's latitude
     * @param[in] icon The icon of the POI
     * @param[in] layer The layer of the POI
     * @param[in] angle The rotation of the POI
     * @param[in] imgFile The raster image of the POI
     * @param[in] relativePath set image file as relative path
     * @param[in] width The width of the POI image
     * @param[in] height The height of the POI image
     * @param[in] name POI name
     * @param[in] parameters generic parameters
     */
    void buildPOIGeo(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const std::string& type,
                     const RGBColor& color, const double lon, const double lat, const std::string& icon, const double layer,
                     const double angle, const std::string& imgFile, bool relativePath, const double width, const double height,
                     const std::string& name, const Parameterised::Map& parameters);

    /**@brief Builds a JuPedSim walkable area using the given values
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The name of the walkable area
     * @param[in] shape The shape of the walkable area
     * @param[in] geo specify if shape was loaded as GEO
     * @param[in] name walkable area name
     * @param[in] parameters generic parameters
     */
    void buildJpsWalkableArea(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const PositionVector& shape,
                              bool geo, const std::string& name, const Parameterised::Map& parameters);

    /**@brief Builds a JuPedSim obstacle using the given values
     * @param[in] sumoBaseObject sumo base object used for build
     * @param[in] id The name of the obstacle
     * @param[in] shape The shape of the obstacle
     * @param[in] geo specify if shape was loaded as GEO
     * @param[in] name obstacle name
     * @param[in] parameters generic parameters
     */
    void buildJpsObstacle(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, const std::string& id, const PositionVector& shape,
                          bool geo, const std::string& name, const Parameterised::Map& parameters);

    /// @}

    /// @brief check if a GNEAccess can be created in a certain Edge
    static bool accessCanBeCreated(GNEAdditional* busStopParent, GNEEdge* edge);

    /// @brief check if an overlapping is produced in rerouter if a interval with certain begin and end is inserted
    static bool checkOverlappingRerouterIntervals(GNEAdditional* rerouter, const SUMOTime newBegin, const SUMOTime newEnd);

    /**@brief check if the given position over a lane is valid
     * @param[in] pos pos position of element over lane
     * @param[in] length element's length
     * @param[in] laneLength Length of the lane
     * @param[in] friendlyPos Attribute of element
     * @return true if the element position is valid, false in otherweise
     */
    static bool checkLanePosition(double pos, const double length, const double laneLength, const bool friendlyPos);

    /**@brief fix given position over lane
     * @param[in] pos pos position of element over lane
     * @param[in] laneLength Length of the lane
     */
    static void fixLanePosition(double& pos, double& length, const double laneLength);

    /**@brief check if enable friendly pos in small lanes
     * @param[in] pos pos position of element over lane
     * @param[in] length element's length
     * @param[in] laneLength Length of the lane
     * @param[in] friendlyPos Attribute of element
     * @return true if the element position is valid, false in otherweise
     */
    static bool checkFriendlyPosSmallLanes(double pos, const double length, const double laneLength, const bool friendlyPos);

    /**@brief check if the given positions over a lane is valid
     * @param[in] from begin position of element over lane
     * @param[in] to end position of element over lane
     * @param[in] laneLength Length of the lane
     * @param[in] friendlyPos Attribute of element
     * @return true if the element positions is valid, false in otherwise
     */
    static bool checkLaneDoublePosition(double from, const double to, const double laneLength, const bool friendlyPos);

    /**@brief fix the given positions over lane
     * @param[in] from begin position of element over lane
     * @param[in] to end position of element over lane
     * @param[in] laneLength Length of the lane
     */
    static void fixLaneDoublePosition(double& from, double& to, const double laneLengt);

    /**@brief check if the given positions over two lanes are valid
     * @param[in] fromPos position of element over first lane
     * @param[in] fromLaneLength length of the first lane
     * @param[in] toPos position of element over second lane
     * @param[in] toLaneLength length of the second lane
     * @param[in] friendlyPos flag for friendlyPos
     * @return true if the element positions is valid, false in otherwise
     */
    static bool checkMultiLanePosition(double fromPos, const double fromLaneLength, const double toPos, const double tolaneLength, const bool friendlyPos);

    /**@brief fix the given positions over two lanes
     * @param[in] fromPos position of element over first lane
     * @param[in] fromLaneLength length of the first lane
     * @param[in] toPos position of element over second lane
     * @param[in] toLaneLength length of the second lane
     */
    static void fixMultiLanePosition(double fromPos, const double fromLaneLength, double toPos, const double tolaneLength);

protected:
    /// @brief write invalid id
    void writeInvalidID(const SumoXMLTag tag, const std::string& id);

    /// @brief write error "invalid position"
    void writeErrorInvalidPosition(const SumoXMLTag tag, const std::string& id);

    /// @brief write error "duplicated additional"
    void writeErrorDuplicated(const SumoXMLTag tag, const std::string& id);

    /// @brief write error "invalid parent element"
    void writeErrorInvalidParent(const SumoXMLTag tag, const std::string& id, const SumoXMLTag parent, const std::string& parentID);

    /// @brief write error "invalid negative element"
    void writeErrorInvalidNegativeValue(const SumoXMLTag tag, const std::string& id, const SumoXMLAttr attribute);

    /// @brief write error "invalid list of vehicle types"
    void writeErrorInvalidVTypes(const SumoXMLTag tag, const std::string& id);

    /// @brief write error "invalid filename"
    void writeErrorInvalidFilename(const SumoXMLTag tag, const std::string& id);

    /// @brief write error "invalid list of lanes"
    void writeErrorInvalidLanes(const SumoXMLTag tag, const std::string& id);

    /// @brief check list of IDs
    bool checkListOfVehicleTypes(const std::vector<std::string>& vTypeIDs) const;

    /// @brief get additional parent
    GNEAdditional* getAdditionalParent(const CommonXMLStructure::SumoBaseObject* sumoBaseObject, SumoXMLTag tag) const;

    /// @brief get rerouter interval parent
    GNEAdditional* getRerouterIntervalParent(const CommonXMLStructure::SumoBaseObject* sumoBaseObject) const;

    /// @brief parse edges
    std::vector<GNEEdge*> parseEdges(const SumoXMLTag tag, const std::vector<std::string>& edgeIDs);

    /// @brief parse lanes
    std::vector<GNELane*> parseLanes(const SumoXMLTag tag, const std::vector<std::string>& laneIDs);

    /// @brief check if given ID correspond to a duplicated additionals
    bool checkDuplicatedID(const std::vector<SumoXMLTag> tags, const std::string& id);

    /// @brief remove overwritten additional
    void overwriteAdditional();

    /// @brief struct for Netedit parameters
    struct NeteditParameters {
        /// @brief parameter constructor
        NeteditParameters(const CommonXMLStructure::SumoBaseObject* sumoBaseObject);

        /// @brief destructor
        ~NeteditParameters();

        /// @brief select
        const bool select;

        /// @brief center view after creation
        const bool centerAfterCreation;

    private:
        /// @brief default constructor
        NeteditParameters();

        /// @brief invalidate copy constructor
        NeteditParameters(const NeteditParameters& s) = delete;

        /// @brief invalidate assignment operator
        NeteditParameters& operator=(const NeteditParameters& s) = delete;
    };

private:
    /// @brief pointer to GNENet
    GNENet* myNet;

    /// @brief allow undo/redo
    const bool myAllowUndoRedo;

    /// @brief check if overwrite
    const bool myOverwrite;

    /// @brief additional to overwrite (using undor-redo
    GNEAdditional* myAdditionalToOverwrite = nullptr;

    /// @brief invalidate default constructo
    GNEAdditionalHandler();

    /// @brief invalidate copy constructor
    GNEAdditionalHandler(const GNEAdditionalHandler& s) = delete;

    /// @brief invalidate assignment operator
    GNEAdditionalHandler& operator=(const GNEAdditionalHandler& s) = delete;
};
