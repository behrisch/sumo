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
/// @file    MSDevice_StationFinder.h
/// @author  Michael Behrisch
/// @author  Mirko Barthauer
/// @date    2023-05-24
///
// A device which triggers rerouting to nearby charging stations
/****************************************************************************/
#pragma once
#include <config.h>

#include <utils/common/WrappingCommand.h>
#include <microsim/trigger/MSStoppingPlaceRerouter.h>
#include "MSVehicleDevice.h"


#define DEFAULT_SOC_INTERVAL 0.1
#define DEFAULT_ENERGY_PER_DISTANCE 200 // Wh/km
#define DEFAULT_AVG_WAITING_TIME 900. // s
#define DEFAULT_CHARGINGSTATION_VIEW_DIST 10 // m
#define DEFAULT_CONSUMPTION_ESTIMATE_HISTORY 10 // s

// ===========================================================================
// class declarations
// ===========================================================================
class MSDevice_Battery;
class MSStoppingPlace;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSDevice_StationFinder
 * @brief A device which triggers rerouting to nearby charging stations
 *
 * Each device checks the battery status by being
 *  called each time step and initiaiting the search for a compatible
 *  charging station if the battery level is too low.
 *
 * @see MSDevice
 */
class MSDevice_StationFinder : public MSVehicleDevice, MSStoppingPlaceRerouter {
public:
    enum ChargeType {
        CHARGETYPE_CHARGING,
        CHARGETYPE_BIDIRECTIONAL,
        CHARGETYPE_BATTERYEXCHANGE,
        CHARGETYPE_FUEL
    };

    enum ChargingStrategy {
        CHARGINGSTRATEGY_NONE,
        CHARGINGSTRATEGY_BALANCED,
        CHARGINGSTRATEGY_LATEST
    };

    enum RescueAction {
        RESCUEACTION_NONE,
        RESCUEACTION_REMOVE,
        RESCUEACTION_TOW
    };

    enum SearchState {
        SEARCHSTATE_NONE = 0,
        SEARCHSTATE_SUCCESSFUL,
        SEARCHSTATE_UNSUCCESSFUL,
        SEARCHSTATE_CHARGING,
        SEARCHSTATE_WAITING,
        SEARCHSTATE_BROKEN_DOWN
    };

    /** @brief Inserts MSDevice_StationFinder-options
     */
    static void insertOptions(OptionsCont& oc);


    /** @brief Build devices for the given vehicle, if needed
     *
     * The options are read and evaluated whether stationFinder-devices shall be built
     *  for the given vehicle.
     *
     * For each seen vehicle, the global vehicle index is increased.
     *
     * The built device is stored in the given vector.
     *
     * @param[in] v The vehicle for which a device may be built
     * @param[in, filled] into The vector to store the built device in
     */
    static void buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into);

    /** @brief Constructor
     *
     * @param[in] holder The vehicle that holds this device
     */
    MSDevice_StationFinder(SUMOVehicle& holder);

    /// @brief Destructor.
    ~MSDevice_StationFinder();

    /// @name Methods called on vehicle movement / state change, overwriting MSDevice
    /// @{

    /** @brief Computes current emission values and adds them to their sums
        *
        * The vehicle's current emission values
        *  are computed using the current velocity and acceleration.
        *
        * @param[in] veh The regarded vehicle
        * @param[in] oldPos Position before the move-micro-timestep.
        * @param[in] newPos Position after the move-micro-timestep.
        * @param[in] newSpeed The vehicle's current speed
        * @return false, if the vehicle is beyond the lane, true otherwise
        * @see MSMoveReminder
        * @see MSMoveReminder::notifyMove
        * @see PollutantsInterface
        */
    bool notifyMove(SUMOTrafficObject& veh, double oldPos, double newPos, double newSpeed) override;

    /** @brief Computes idling emission values and adds them to the emission sums
        *
        * Idling implied by zero velocity, acceleration and slope
        *
        * @param[in] veh The vehicle
        *
        * @see MSMoveReminder::notifyMove
        * @see PollutantsInterface
        */
    bool notifyIdle(SUMOTrafficObject& veh) override;

    /// @}

    /** @brief Saves the state of the device
     *
     * @param[in] out The OutputDevice to write the information into
     */
    void saveState(OutputDevice& out) const;

    /** @brief Loads the state of the device from the given description
     *
     * @param[in] attrs XML attributes describing the current state
     */
    void loadState(const SUMOSAXAttributes& attrs);

    /// @brief return the name for this type of device
    const std::string deviceName() const override {
        return "stationfinder";
    }

    /// @brief return the string representation of the chosen charging strategy
    const std::string getChargingStrategy() const {
        if (myChargingStrategy == CHARGINGSTRATEGY_NONE) {
            return "none";
        } else if (myChargingStrategy == CHARGINGSTRATEGY_BALANCED) {
            return "balanced";
        } else {
            return "latest";
        }
    }

    /** @brief Called on writing tripinfo output
     *
     * @param[in] os The stream to write the information into
     * @exception IOError not yet implemented
     * @see MSDevice::tripInfoOutput
     */
    void generateOutput(OutputDevice* tripinfoOut) const override;

    void setBattery(MSDevice_Battery* battery) {
        myBattery = battery;
    }

    std::string getParameter(const std::string& key) const override;

    /// @brief try to set the given parameter for this device. Throw exception for unsupported key
    void setParameter(const std::string& key, const std::string& value);

    /** @brief Compute some custom target function components
     *
     * @param[in] veh the concerned vehicle
     * @param[in] brakeGap the distance before which the vehicle cannot stop
     * @param[in] newDestination whether the destination changed
     * @param[in] alternative the stopping place to evaluate
     * @param[in] occupancy occupancy of the stopping place
     * @param[in] router the router to use for evaluation if needed
     * @param[in,out] stoppingPlaceValues the data structure to write the evaluation values to
     * @param[in] newRoute the complete route to the destination passing by the stopping place
     * @param[in] stoppingPlaceApproach the route to the stopping place
     * @param[in] maxValues the maximum values of the components
     * @param[in] addInput external input data
     * @return false if the stopping place cannot be used according to the custom evaluation components
     */
    bool evaluateCustomComponents(SUMOVehicle& veh, double brakeGap, bool newDestination,
                                  MSStoppingPlace* alternative, double occupancy, double prob,
                                  SUMOAbstractRouter<MSEdge, SUMOVehicle>& router, StoppingPlaceParamMap_t& stoppingPlaceValues,
                                  ConstMSEdgeVector& newRoute,
                                  ConstMSEdgeVector& stoppingPlaceApproach,
                                  StoppingPlaceParamMap_t& maxValues,
                                  StoppingPlaceParamMap_t& addInput) override;

    /// @brief Whether the stopping place should be discarded due to its results from the component evaluation
    bool validComponentValues(StoppingPlaceParamMap_t& stoppingPlaceValues) override;

    /// @brief Whether the stopping place should be included in the search (can be used to add an additional filter)
    bool useStoppingPlace(MSStoppingPlace* stoppingPlace) override;

    /// @brief Provide the router to use (MSNet::getRouterTT or MSRoutingEngine)
    SUMOAbstractRouter<MSEdge, SUMOVehicle>& getRouter(SUMOVehicle& veh, const MSEdgeVector& prohibited) override;

    /// @brief Return the number of occupied places of the StoppingPlace
    double getStoppingPlaceOccupancy(MSStoppingPlace* stoppingPlace) override;

    /// @brief Return the number of occupied places of the StoppingPlace from the previous time step
    double getLastStepStoppingPlaceOccupancy(MSStoppingPlace* stoppingPlace) override;

    /// @brief Return the number of places the StoppingPlace provides
    double getStoppingPlaceCapacity(MSStoppingPlace* stoppingPlace) override;

    /// @brief store the blocked stopping place in the vehicle
    void rememberBlockedStoppingPlace(SUMOVehicle& veh, const MSStoppingPlace* stoppingPlace, bool blocked) override;

    /// @brief store the stopping place score in the vehicle
    void rememberStoppingPlaceScore(SUMOVehicle& veh, MSStoppingPlace* place, const std::string& score) override;

    /// @brief forget all stopping place score for this vehicle
    void resetStoppingPlaceScores(SUMOVehicle& veh) override;

    /// @brief ask the vehicle when it has seen the stopping place
    SUMOTime sawBlockedStoppingPlace(SUMOVehicle& veh, MSStoppingPlace* place, bool local) override;

    /// @brief ask how many times already the vehicle has been rerouted to another stopping place
    int getNumberStoppingPlaceReroutes(SUMOVehicle& veh) override;

    /// @brief update the number of reroutes for the vehicle
    void setNumberStoppingPlaceReroutes(SUMOVehicle& veh, int value) override;

protected:
    /** @brief Internal notification about the vehicle moves, see MSMoveReminder::notifyMoveInternal()
     *
     */
    void notifyMoveInternal(const SUMOTrafficObject& veh,
                            const double frontOnLane,
                            const double timeOnLane,
                            const double meanSpeedFrontOnLane,
                            const double meanSpeedVehicleOnLane,
                            const double travelledDistanceFrontOnLane,
                            const double travelledDistanceVehicleOnLane,
                            const double meanLengthOnLane) override;

private:
    /** @brief central search function for close charging stations
     *
     * @param[in] router
     * @param[in] expectedConsumption
     * @param[in,out] scores additional input for score computation and scores of the best charging station
     * @param[in] constrainTT whether to constrain the search radius by a maximum travel time
     * @param[in] skipVisited whether to skip charging stations which have not been available when passing by recently
     * @param[in] skipOccupied whether to skip fully occupied charging stations
     * @return The found charging station, otherwise nullptr
     */
    MSChargingStation* findChargingStation(SUMOAbstractRouter<MSEdge, SUMOVehicle>& router, double expectedConsumption, StoppingPlaceParamMap_t& scores, bool constrainTT = true, bool skipVisited = true, bool skipOccupied = false);


    /** @brief reroute to a charging station
     *
     * @param[in] replace if the already planned next stop should be replaced (a new stop will be prepended if false)
     * @return true if the vehicle has been redirected to a charging station, false otherwise
     */
    bool rerouteToChargingStation(bool replace = false);

    /** @brief search for a charging station and teleport the vehicle there as a rescue measure
     */
    SUMOTime teleportToChargingStation(const SUMOTime currentTime);

    /** @brief estimate the energy needed for the planned route / up to a target edge
     *
     * @param[in] target edge along the route up to which the consumption shall be estimated - the complete route will be used if defaulting to nullptr
     * @param[in] includeEmptySoC whether to add an additional buffer for the range up to the "empty" threshold
     * @param[in] stopDiscount duration in seconds to discount in the consumption estimation due to occurred stopping time
     * @return energy in Wh needed to complete the planned route
     */
    double estimateConsumption(const MSEdge* target = nullptr, const bool includeEmptySoC = true, const double stopDiscount = 0.) const;

    /** @brief compute the free space at a charging station
     *
     * @param[in] cs the charging station to compute the free space for
     * @return the free space at the charging station as a fraction of the holder vehicle
     */
    double freeSpaceAtChargingStation(MSChargingStation* cs) const;

    /** @brief adopt a planned charging stop outside of the device
     *
     * @return whether an already present stop was adopted to be used with the device logic
     */
    bool alreadyPlannedCharging();

    /** @brief create the event command for teleporting in case of brake-down
     */
    void initRescueCommand();

    /** @brief create the event command for changing charging rates
     */
    void initChargeLimitCommand();

    /** @brief update the maximum charge rate of the battery to simulate charging strategies
     */
    SUMOTime updateChargeLimit(const SUMOTime currentTime);

    /** @brief
     */
    void implementChargingStrategy(SUMOTime begin, SUMOTime end, const double plannedCharge, const MSChargingStation* cs);

private:
    /// @brief myHolder cast to needed type
    MSVehicle& myVeh;

    /// @brief The corresponding battery device
    MSDevice_Battery* myBattery;

    /// @brief To which station we are currently travelling
    MSStoppingPlace* myChargingStation;

    /// @brief The command responsible for rescue actions
    WrappingCommand<MSDevice_StationFinder>* myRescueCommand;

    /// @brief The command responsible for limiting the charging rate (~ implement charging strategies)
    WrappingCommand<MSDevice_StationFinder>* myChargeLimitCommand;

    /// @brief The next charging rates to set via myChargingRateCommand
    std::vector<std::pair<SUMOTime, double>> myChargeLimits;

    /// @brief Last time the SoC was checked
    SUMOTime myLastChargeCheck;

    /// @brief Time interval after which the SoC has to be checked
    SUMOTime myCheckInterval;

    /// @brief Arrival time in the vicinity of the target charging station (to track the waiting time before accessing it)
    SUMOTime myArrivalAtChargingStation;

    /// @brief Last time charging stations have been searched
    SUMOTime myLastSearch;

    /// @brief The time to wait for a rescue vehicle in case the battery is empty
    double myRescueTime;

    /// @brief The safety buffer when calculating expected consumption
    double myReserveFactor;

    /// @brief The state of charge threshold below which rescue mode is activated
    double myEmptySoC;

    /// @brief The maximum euclidean distance between the vehicle and the charging station (-1 deactivates the condition)
    double myMaxEuclideanDistance;

    /// @brief The max travel time to the next charging station
    SUMOTime myRadius;

    /// @brief Time interval to search again for a charging station if the first attempt failed
    SUMOTime myRepeatInterval;

    /// @brief Accepted waiting time at the charging station before a place becomes available
    SUMOTime myWaitForCharge;

    /// @brief SoC the last time the station finder algorithm was run completely
    double myUpdateSoC;

    /// @brief The maximum charging speed of the vehicle battery in W
    double myMaxChargePower;

    /// @brief The target state of charge where the vehicle stops charging
    double myTargetSoC;

    /// @brief The state of charge at which the vehicle starts looking for charging stations
    double mySearchSoC;

    /// @brief The share of stopping time a charging stop should take from the next regulr (non-charging) stop under certain conditions
    double myReplacePlannedStop;

    /// @brief The distance in meters to the original stop replaced by the charging stop (models charging close to the activity location)
    double myDistanceToOriginalStop;

    /// @brief The type of charging permitted by the battery (charging, bidirectional, battery exchange)
    ChargeType myChargeType;

    /// @brief The chosen charging strategy
    ChargingStrategy myChargingStrategy;

    /// @brief What to do when the state of charge gets very low
    RescueAction myRescueAction;

    /// @brief The current state of the charging search (remember for decision logic)
    SearchState mySearchState = SEARCHSTATE_NONE;

private:
    /// @brief Invalidated copy constructor.
    MSDevice_StationFinder(const MSDevice_StationFinder&);

    /// @brief Invalidated assignment operator.
    MSDevice_StationFinder& operator=(const MSDevice_StationFinder&);
};
