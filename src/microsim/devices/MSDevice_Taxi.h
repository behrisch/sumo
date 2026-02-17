/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2013-2026 German Aerospace Center (DLR) and others.
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
/// @file    MSDevice_Taxi.h
/// @author  Jakob Erdmann
/// @date    16.12.2019
///
// A device which controls a taxi
/****************************************************************************/
#pragma once
#include <config.h>

#include <utils/common/SUMOTime.h>
#include <utils/common/WrappingCommand.h>
#include "MSVehicleDevice.h"


// ===========================================================================
// class declarations
// ===========================================================================
class SUMOTrafficObject;
class MSDispatch;
class MSIdling;
class MSDevice_Routing;
struct Reservation;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSDevice_Taxi
 * @brief A device which collects info on the vehicle trip (mainly on departure and arrival)
 *
 * Each device collects departure time, lane and speed and the same for arrival.
 *
 * @see MSDevice
 */
class MSDevice_Taxi : public MSVehicleDevice {
public:

    enum TaxiState {
        EMPTY = 0, // empty (available for servicing customers)
        PICKUP = 1, // driving to pick up customer
        OCCUPIED = 2 // occupied with customer
    };

    /** @brief Inserts MSDevice_Taxi-options
     * @param[filled] oc The options container to add the options to
     */
    static void insertOptions(OptionsCont& oc);


    /** @brief Build devices for the given vehicle, if needed
     *
     * The options are read and evaluated whether a Taxi-device shall be built
     *  for the given vehicle.
     *
     * The built device is stored in the given vector.
     *
     * @param[in] v The vehicle for which a device may be built
     * @param[filled] into The vector to store the built device in
     */
    static void buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into);

    /// @brief whether the given lines description is a taxi call
    static bool isReservation(const std::set<std::string>& lines);

    /// add new reservation
    static void addReservation(MSTransportable* person,
                               const std::set<std::string>& lines,
                               SUMOTime reservationTime,
                               SUMOTime pickupTime,
                               SUMOTime earliestPickupTime,
                               const MSEdge* from, double fromPos,
                               const MSStoppingPlace* fromStop,
                               const MSEdge* to, double toPos,
                               const MSStoppingPlace* toStop,
                               const std::string& group);

    /// @brief retract reservation
    static void removeReservation(MSTransportable* person,
                                  const std::set<std::string>& lines,
                                  const MSEdge* from, double fromPos,
                                  const MSEdge* to, double toPos,
                                  const std::string& group);

    /// @brief update reservation's fromPos due to pre-booking
    static void updateReservationFromPos(MSTransportable* person,
                                         const std::set<std::string>& lines,
                                         const MSEdge* from, double fromPos,
                                         const MSEdge* to, double toPos,
                                         const std::string& group, double newFromPos);

    /// @brief period command to trigger the dispatch algorithm
    static SUMOTime triggerDispatch(SUMOTime currentTime);

    /// @brief check whether there are still (servable) reservations in the system
    static bool hasServableReservations();

    /// @brief resets counters
    static void cleanup();

    static MSDispatch* getDispatchAlgorithm() {
        return myDispatcher;
    }

    static const std::vector<MSDevice_Taxi*>& getFleet() {
        return myFleet;
    }

public:
    /// @brief Destructor.
    ~MSDevice_Taxi();

    /// @name Methods called on vehicle movement / state change, overwriting MSDevice
    /// @{

    /** @brief Checks for waiting steps when the vehicle moves
     *
     * @param[in] veh Vehicle that asks this reminder.
     * @param[in] oldPos Position before move.
     * @param[in] newPos Position after move with newSpeed.
     * @param[in] newSpeed Moving speed.
     *
     * @return True (always).
     */
    bool notifyMove(SUMOTrafficObject& veh, double oldPos,
                    double newPos, double newSpeed);


    /** @brief Saves departure info on insertion
     *
     * @param[in] veh The entering vehicle.
     * @param[in] reason how the vehicle enters the lane
     * @return Always true
     * @see MSMoveReminder::notifyEnter
     * @see MSMoveReminder::Notification
     */
    bool notifyEnter(SUMOTrafficObject& veh, MSMoveReminder::Notification reason, const MSLane* enteredLane = 0);
    /// @}


    /// @brief return the name for this type of device
    const std::string deviceName() const {
        return "taxi";
    }

    /// @brief whether the taxi is empty
    bool isEmpty();

    int getState() const {
        return myState;
    }

    /// @brief returns whether taxis have been loaded
    static bool hasFleet();

    /// @brief service the given reservation
    void dispatch(const Reservation& res);

    /// @brief service the given reservations
    void dispatchShared(std::vector<const Reservation*> reservations);

    /// @brief remove the persons the taxi is currently waiting for from reservations
    void cancelCurrentCustomers();

    /// @brief remove person from reservations
    bool cancelCustomer(const MSTransportable* t);

    /// @brief add person after extending reservation
    void addCustomer(const MSTransportable* t, const Reservation* res);

    /// @brief whether the given person is allowed to board this taxi
    bool allowsBoarding(const MSTransportable* t) const;

    /// @brief called by MSDevice_Transportable upon loading a person
    void customerEntered(const MSTransportable* t);

    /// @brief called by MSDevice_Transportable upon unloading a person
    void customerArrived(const MSTransportable* person);

    /// @brief try to retrieve the given parameter from this device. Throw exception for unsupported key
    std::string getParameter(const std::string& key) const;

    /// @brief try to set the given parameter for this device. Throw exception for unsupported key
    void setParameter(const std::string& key, const std::string& value);

    /** @brief Called on writing tripinfo output
     *
     * @param[in] os The stream to write the information into
     * @exception IOError not yet implemented
     * @see MSDevice::generateOutput
     */
    void generateOutput(OutputDevice* tripinfoOut) const;

    /// @brief whether the given reservation is compatible with the taxi line
    bool compatibleLine(const Reservation* res);

    static bool compatibleLine(const std::string& taxiLine, const std::string& rideLine);

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
                            const double meanLengthOnLane);

private:
    /** @brief Constructor
     *
     * @param[in] holder The vehicle that holds this device
     * @param[in] id The ID of the device
     */
    MSDevice_Taxi(SUMOVehicle& holder, const std::string& id);

    void updateMove(const SUMOTime traveltime, const double travelledDist);

    /// @brief prepare stop for the given action
    void prepareStop(ConstMSEdgeVector& edges,
                     StopParVector& stops,
                     double& lastPos, const MSEdge* stopEdge, double stopPos,
                     const MSStoppingPlace* stopPlace,
                     const std::string& action, const Reservation* res, const bool isPickup);

    /// @brief determine stopping lane for taxi
    MSLane* getStopLane(const MSEdge* edge, const std::string& action);

    /// @brief whether the taxi has another pickup scheduled
    bool hasFuturePickup();

    /// @brief optionally swap tasks when a taxi becomes idle
    void checkTaskSwap();

    /// @brief initialize the dispatch algorithm
    static void initDispatch();

private:

    int myState = EMPTY;
    /// @brief number of customers that were served
    int myCustomersServed = 0;
    /// @brief distance driven with customers
    double myOccupiedDistance = 0;
    /// @brief time spent driving with customers
    SUMOTime myOccupiedTime = 0;
    /// @brief the time at which the taxi service ends (end the vehicle may leave the simulation)
    SUMOTime myServiceEnd = SUMOTime_MAX;
    /// @brief whether the vehicle is currently stopped
    bool myIsStopped = false;
    /// @brief the customer of the current reservation
    std::set<const MSTransportable*> myCustomers;

    /// @brief algorithm for controlling idle behavior
    MSIdling* myIdleAlgorithm;

    /// @brief whether the taxi has reached its schedule service end
    bool myReachedServiceEnd = false;

    /// @brief reservations currently being served
    std::set<const Reservation*> myCurrentReservations;

    /// @brief routing device (if the vehicle has one)
    MSDevice_Routing* myRoutingDevice = nullptr;

    /// @brief the time between successive calls to the dispatcher
    static SUMOTime myDispatchPeriod;
    /// @brief the dispatch algorithm
    static MSDispatch* myDispatcher;
    /// @brief The repeated call to the dispatcher
    static Command* myDispatchCommand;
    /// @brief the last dispatch order
    std::vector<const Reservation*> myLastDispatch;
    // @brief the list of available taxis
    static std::vector<MSDevice_Taxi*> myFleet;
    // @brief the maximum personCapacity in the fleet
    static int myMaxCapacity;
    // @brief the maximum container capacity in the fleet
    static int myMaxContainerCapacity;

    static std::set<std::string> myVClassWarningVTypes;

private:
    /// @brief Invalidated copy constructor.
    MSDevice_Taxi(const MSDevice_Taxi&);

    /// @brief Invalidated assignment operator.
    MSDevice_Taxi& operator=(const MSDevice_Taxi&);


};
