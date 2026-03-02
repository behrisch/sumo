/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2012-2026 German Aerospace Center (DLR) and others.
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
/// @file    Vehicle.h
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    15.03.2017
///
// C++ Vehicle API
/****************************************************************************/
#pragma once
#include <limits>
#include <vector>
#include <libsumo/TraCIDefs.h>
#include <libsumo/VehicleType.h>
#include <libsumo/TraCIConstants.h>


// ===========================================================================
// class declarations
// ===========================================================================
#ifndef LIBTRACI
class PositionVector;
class SUMOVehicle;
#endif


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class Vehicle
 * @brief C++ TraCI client API implementation
 */
namespace LIBSUMO_NAMESPACE {
class Vehicle {
public:
    /// @name Value retrieval
    /// @{
    static double getSpeed(const std::string& vehID);
    static double getLateralSpeed(const std::string& vehID);
    static double getAcceleration(const std::string& vehID);
    static double getSpeedWithoutTraCI(const std::string& vehID);
    static libsumo::TraCIPosition getPosition(const std::string& vehID, const bool includeZ = false);
    static libsumo::TraCIPosition getPosition3D(const std::string& vehID);
    static double getAngle(const std::string& vehID);
    static double getSlope(const std::string& vehID);
    static std::string getRoadID(const std::string& vehID);
    static std::string getLaneID(const std::string& vehID);
    static int getLaneIndex(const std::string& vehID);
    static std::string getSegmentID(const std::string& vehID);
    static int getSegmentIndex(const std::string& vehID);
    static std::string getTypeID(const std::string& vehID);
    static std::string getRouteID(const std::string& vehID);
    static double getDeparture(const std::string& vehID);
    static double getDepartDelay(const std::string& vehID);
    static int getRouteIndex(const std::string& vehID);
    static double getLanePosition(const std::string& vehID);
    static double getLateralLanePosition(const std::string& vehID);
    static double getCO2Emission(const std::string& vehID);
    static double getCOEmission(const std::string& vehID);
    static double getHCEmission(const std::string& vehID);
    static double getPMxEmission(const std::string& vehID);
    static double getNOxEmission(const std::string& vehID);
    static double getFuelConsumption(const std::string& vehID);
    static double getNoiseEmission(const std::string& vehID);
    static double getElectricityConsumption(const std::string& vehID);
    static int getPersonNumber(const std::string& vehID);
    static std::vector<std::string> getPersonIDList(const std::string& vehID);
    static std::pair<std::string, double> getLeader(const std::string& vehID, double dist = 100.);
    static std::pair<std::string, double> getFollower(const std::string& vehID, double dist = 0.);
    static std::vector<libsumo::TraCIJunctionFoe> getJunctionFoes(const std::string& vehID, double dist = 0.);
    static double getWaitingTime(const std::string& vehID);
    static double getAccumulatedWaitingTime(const std::string& vehID);
    static double getAdaptedTraveltime(const std::string& vehID, double time, const std::string& edgeID);
    static double getEffort(const std::string& vehID, double time, const std::string& edgeID);
    static bool isRouteValid(const std::string& vehID);
    static std::vector<std::string> getRoute(const std::string& vehID);
    static int getSignals(const std::string& vehID);
    static std::vector<libsumo::TraCIBestLanesData> getBestLanes(const std::string& vehID);
    static std::vector<libsumo::TraCINextTLSData> getNextTLS(const std::string& vehID);
    static std::vector<libsumo::TraCINextStopData> getNextStops(const std::string& vehID);
    static std::vector<libsumo::TraCIConnection> getNextLinks(const std::string& vehID);
    static std::vector<libsumo::TraCINextStopData> getStops(const std::string& vehID, int limit = 0);
    static int getStopState(const std::string& vehID);
    static std::string getStopParameter(const std::string& vehID, int nextStopIndex, const std::string& param, bool customParam = false);

    static double getDistance(const std::string& vehID);
    static double getDrivingDistance(const std::string& vehID, const std::string& edgeID, double pos, int laneIndex = 0);
    static double getDrivingDistance2D(const std::string& vehID, double x, double y);
    static double getReferenceDistance(const std::string& vehID);
    static double getAllowedSpeed(const std::string& vehID);
    static int getSpeedMode(const std::string& vehID);
    static int getLaneChangeMode(const std::string& vehID);
    static int getRoutingMode(const std::string& vehID);
    static std::string getLine(const std::string& vehID);
    static std::vector<std::string> getVia(const std::string& vehID);
    static std::pair<int, int> getLaneChangeState(const std::string& vehID, int direction);
    static double getLastActionTime(const std::string& vehID);
    static std::vector<std::pair<std::string, double> > getNeighbors(const std::string& vehID, const int mode);
    static double getFollowSpeed(const std::string& vehID, double speed, double gap, double leaderSpeed, double leaderMaxDecel, const std::string& leaderID = "");
    static double getSecureGap(const std::string& vehID, double speed, double leaderSpeed, double leaderMaxDecel, const std::string& leaderID = "");
    static double getStopSpeed(const std::string& vehID, double speed, double gap);
    static double getStopDelay(const std::string& vehID);
    static double getStopArrivalDelay(const std::string& vehID);
    static double getTimeLoss(const std::string& vehID);
    static std::vector<std::string> getTaxiFleet(int taxiState = 0);

    static std::vector<std::string> getLoadedIDList();
    static std::vector<std::string> getTeleportingIDList();
    /// @}

    LIBSUMO_ID_PARAMETER_API
    LIBSUMO_VEHICLE_TYPE_GETTER

    /// @name State Changing
    /// @{

    static void setStop(const std::string& vehID,
                        const std::string& edgeID,
                        double pos = 1.,
                        int laneIndex = 0,
                        double duration = libsumo::INVALID_DOUBLE_VALUE,
                        int flags = libsumo::STOP_DEFAULT,
                        double startPos = libsumo::INVALID_DOUBLE_VALUE,
                        double until = libsumo::INVALID_DOUBLE_VALUE);

    static void replaceStop(const std::string& vehID,
                            int nextStopIndex,
                            const std::string& edgeID,
                            double pos = 1.,
                            int laneIndex = 0,
                            double duration = libsumo::INVALID_DOUBLE_VALUE,
                            int flags = libsumo::STOP_DEFAULT,
                            double startPos = libsumo::INVALID_DOUBLE_VALUE,
                            double until = libsumo::INVALID_DOUBLE_VALUE,
                            int teleport = 0);

    static void insertStop(const std::string& vehID,
                           int nextStopIndex,
                           const std::string& edgeID,
                           double pos = 1.,
                           int laneIndex = 0,
                           double duration = libsumo::INVALID_DOUBLE_VALUE,
                           int flags = libsumo::STOP_DEFAULT,
                           double startPos = libsumo::INVALID_DOUBLE_VALUE,
                           double until = libsumo::INVALID_DOUBLE_VALUE,
                           int teleport = 0);

    static void setStopParameter(const std::string& vehID, int nextStopIndex,
                                 const std::string& param, const std::string& value,
                                 bool customParam = false);

    static void rerouteParkingArea(const std::string& vehID,
                                   const std::string& parkingAreaID);

    static void resume(const std::string& vehID);

    static void add(const std::string& vehID,
                    const std::string& routeID,
                    const std::string& typeID = "DEFAULT_VEHTYPE",
                    const std::string& depart = "now",
                    const std::string& departLane = "first",
                    const std::string& departPos = "base",
                    const std::string& departSpeed = "0",
                    const std::string& arrivalLane = "current",
                    const std::string& arrivalPos = "max",
                    const std::string& arrivalSpeed = "current",
                    const std::string& fromTaz = "",
                    const std::string& toTaz = "",
                    const std::string& line = "",
                    int personCapacity = 4,
                    int personNumber = 0);

    static void changeTarget(const std::string& vehID, const std::string& edgeID);
    static void changeLane(const std::string& vehID, int laneIndex, double duration);
    static void changeLaneRelative(const std::string& vehID, int indexOffset, double duration);
    static void changeSublane(const std::string& vehID, double latDist);

    static void slowDown(const std::string& vehID, double speed, double duration);
    static void openGap(const std::string& vehID, double newTimeHeadway, double newSpaceHeadway, double duration, double changeRate, double maxDecel = -1, const std::string& referenceVehID = "");
    static void deactivateGapControl(const std::string& vehID);
    static void requestToC(const std::string& vehID, double leadTime);
    static void setSpeed(const std::string& vehID, double speed);
    static void setAcceleration(const std::string& vehID, double acceleration, double duration);
    static void setPreviousSpeed(const std::string& vehID, double prevSpeed, double prevAcceleration = libsumo::INVALID_DOUBLE_VALUE);
    static void setSpeedMode(const std::string& vehID, int speedMode);
    static void setLaneChangeMode(const std::string& vehID, int laneChangeMode);
    static void setRoutingMode(const std::string& vehID, int routingMode);
    static void setType(const std::string& vehID, const std::string& typeID);
    static void setRouteID(const std::string& vehID, const std::string& routeID);
    static void setRoute(const std::string& vehID, const std::string& edgeID);
    static void setRoute(const std::string& vehID, const std::vector<std::string>& edgeIDs);
    static void setLateralLanePosition(const std::string& vehID, double posLat);
    static void updateBestLanes(const std::string& vehID);
    static void setAdaptedTraveltime(const std::string& vehID, const std::string& edgeID,
                                     double time = libsumo::INVALID_DOUBLE_VALUE, double begSeconds = 0, double endSeconds = std::numeric_limits<double>::max());
    static void setEffort(const std::string& vehID, const std::string& edgeID,
                          double effort = libsumo::INVALID_DOUBLE_VALUE, double begSeconds = 0, double endSeconds = std::numeric_limits<double>::max());
    static void rerouteTraveltime(const std::string& vehID, const bool currentTravelTimes = true);
    static void rerouteEffort(const std::string& vehID);
    static void setSignals(const std::string& vehID, int signals);
    static void moveTo(const std::string& vehID, const std::string& laneID, double pos, int reason = libsumo::MOVE_AUTOMATIC);
    static void moveToXY(const std::string& vehID, const std::string& edgeID, const int laneIndex, const double x, const double y, double angle = libsumo::INVALID_DOUBLE_VALUE, const int keepRoute = 1, double matchThreshold = 100);
    static void remove(const std::string& vehID, char reason = libsumo::REMOVE_VAPORIZED);
    static void setLine(const std::string& vehID, const std::string& line);
    static void setVia(const std::string& vehID, const std::vector<std::string>& edgeList);
    static void highlight(const std::string& vehID, const libsumo::TraCIColor& col = libsumo::TraCIColor(255, 0, 0, 255), double size = -1, const int alphaMax = -1, const double duration = -1, const int type = 0);
    static void dispatchTaxi(const std::string& vehID,  const std::vector<std::string>& reservations);
    /// @}

    LIBSUMO_VEHICLE_TYPE_SETTER

    LIBSUMO_SUBSCRIPTION_API

    static void subscribeLeader(const std::string& vehID, double dist = 0., double begin = libsumo::INVALID_DOUBLE_VALUE, double end = libsumo::INVALID_DOUBLE_VALUE);

    static void addSubscriptionFilterLanes(const std::vector<int>& lanes, bool noOpposite = false, double downstreamDist = libsumo::INVALID_DOUBLE_VALUE, double upstreamDist = libsumo::INVALID_DOUBLE_VALUE);

    static void addSubscriptionFilterNoOpposite();

    static void addSubscriptionFilterDownstreamDistance(double dist);

    static void addSubscriptionFilterUpstreamDistance(double dist);

    static void addSubscriptionFilterCFManeuver(double downstreamDist = libsumo::INVALID_DOUBLE_VALUE, double upstreamDist = libsumo::INVALID_DOUBLE_VALUE);

    static void addSubscriptionFilterLCManeuver(int direction = libsumo::INVALID_INT_VALUE, bool noOpposite = false, double downstreamDist = libsumo::INVALID_DOUBLE_VALUE, double upstreamDist = libsumo::INVALID_DOUBLE_VALUE);

    static void addSubscriptionFilterLeadFollow(const std::vector<int>& lanes);

    static void addSubscriptionFilterTurn(double downstreamDist = libsumo::INVALID_DOUBLE_VALUE, double foeDistToJunction = libsumo::INVALID_DOUBLE_VALUE);

    static void addSubscriptionFilterVClass(const std::vector<std::string>& vClasses);

    static void addSubscriptionFilterVType(const std::vector<std::string>& vTypes);

    static void addSubscriptionFilterFieldOfVision(double openingAngle);

    static void addSubscriptionFilterLateralDistance(double lateralDist, double downstreamDist = libsumo::INVALID_DOUBLE_VALUE, double upstreamDist = libsumo::INVALID_DOUBLE_VALUE);

#ifndef LIBTRACI
#ifndef SWIG
    /** @brief Saves the shape of the requested object in the given container
    *  @param id The id of the poi to retrieve
    *  @param shape The container to fill
    */
    static void storeShape(const std::string& id, PositionVector& shape);

    static std::shared_ptr<VariableWrapper> makeWrapper();

    static bool handleVariable(const std::string& objID, const int variable, VariableWrapper* wrapper, tcpip::Storage* paramData);


private:
    static bool isVisible(const SUMOVehicle* veh);

    static bool isOnInit(const std::string& vehID);

private:
    static SubscriptionResults mySubscriptionResults;
    static ContextSubscriptionResults myContextSubscriptionResults;
#endif
#endif

    /// @brief invalidated standard constructor
    Vehicle() = delete;
};


}
