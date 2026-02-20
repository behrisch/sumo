/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2001-2026 German Aerospace Center (DLR) and others.
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
/// @file    MSTransportableControl.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @date    Mon, 9 Jul 2001
///
// Stores all persons or containers in the net and handles their waiting for cars.
/****************************************************************************/
#pragma once
#include <config.h>

#include <vector>
#include <map>
#include <microsim/MSVehicle.h>
#include <microsim/transportables/MSTransportable.h>
#include <microsim/devices/MSDevice_Vehroutes.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MSNet;
class MSPModel;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 *
 * @class MSTransportableControl
 * The class is used to handle transportables (persons and containers)
 *  who are not using a transportation
 *  system but are walking or waiting. This includes waiting
 *  for the arrival or departure time / the time the waiting is over.
 */
class MSTransportableControl {
public:
    /// @brief Definition of a list of transportables
    typedef std::vector<MSTransportable*> TransportableVector;

    /// @brief Definition of the internal transportables map iterator
    typedef std::map<std::string, MSTransportable*>::const_iterator constVehIt;


public:
    /// @brief Constructor
    MSTransportableControl(const bool isPerson);


    /// @brief Destructor
    virtual ~MSTransportableControl();


    /** @brief Adds a single transportable, returns false if an id clash occurred
     * @param[in] transportable The transportable to add
     * @return Whether the transportable could be added (none with the same id existed before)
     */
    bool add(MSTransportable* transportable);


    /** @brief Returns the named transportable, if existing
     * @param[in] id The id of the transportable
     * @return The named transportable, if existing, otherwise nullptr
     */
    MSTransportable* get(const std::string& id) const;


    /// removes a single transportable
    virtual void erase(MSTransportable* transportable);

    /// @removes all transportables
    void eraseAll();

    /// sets the arrival time for a waiting transportable
    void setWaitEnd(SUMOTime time, MSTransportable* transportable);

    /// checks whether any transportables waiting time is over
    void checkWaiting(MSNet* net, const SUMOTime time);

    /// adds a transportable to the list of transportables waiting for a vehicle on the specified edge
    void addWaiting(const MSEdge* edge, MSTransportable* person);

    /// register forced (traci) departure
    void forceDeparture();

    /// @brief check whether any transportables are waiting for the given vehicle
    bool hasAnyWaiting(const MSEdge* edge, SUMOVehicle* vehicle) const;

    /** @brief load any applicable transportables
    * Loads any person / container that is waiting on that edge for the given vehicle and removes them from myWaiting
    * @param[in] edge the edge on which the loading should take place
    * @param[in] vehicle the vehicle which is taking on containers
    * @param[in,out] timeToLoadNext earliest time for the next loading process (gets updated)
    * @param[in,out] stopDuration the duration of the stop where the loading takes place (might be extended)
    * @param[in] force load the specified transportable even if the vehicle is not on a stop (needed for replay)
    * @return Whether any transportables have been loaded
    */
    bool loadAnyWaiting(const MSEdge* edge, SUMOVehicle* vehicle, SUMOTime& timeToLoadNext, SUMOTime& stopDuration, MSTransportable* const force = nullptr);

    /// checks whether any transportable waits to finish her plan
    bool hasTransportables() const;

    /// checks whether any transportable is still engaged in walking / stopping
    bool hasNonWaiting() const;

    /// @brief return the number of active transportable objects
    int getActiveCount();

    /// aborts the plan for any transportable that is still waiting for a ride
    void abortAnyWaitingForVehicle();

    /// let the given transportable abort waiting for a vehicle (when removing stage via TraCI)
    void abortWaitingForVehicle(MSTransportable* t);

    /// aborts waiting stage of transportable
    void abortWaiting(MSTransportable* t);

    /** @brief Builds a new person
     * @param[in] pars The parameter
     * @param[in] vtype The type (reusing vehicle type container here)
     * @param[in] plan This person's plan
     * @param[in] rng The RNG to compute the optional speed deviation
     */
    virtual MSTransportable* buildPerson(const SUMOVehicleParameter* pars, MSVehicleType* vtype, MSTransportable::MSTransportablePlan* plan,
                                         SumoRNG* rng) const;

    /** @brief Builds a new container
    * @param[in] pars The parameter
    * @param[in] vtype The type (reusing vehicle type container here)
    * @param[in] plan This container's plan
    */
    virtual MSTransportable* buildContainer(const SUMOVehicleParameter* pars, MSVehicleType* vtype, MSTransportable::MSTransportablePlan* plan) const;

    /** @brief Returns the begin of the internal transportables map
     * @return The begin of the internal transportables map
     */
    constVehIt loadedBegin() const {
        return myTransportables.begin();
    }


    /** @brief Returns the end of the internal transportables map
     * @return The end of the internal transportables map
     */
    constVehIt loadedEnd() const {
        return myTransportables.end();
    }


    /** @brief Returns the number of known transportables
     * @return The number of stored transportables
     */
    int size() const {
        return (int)myTransportables.size();
    }

    /// @brief register a jammed transportable
    void registerJammed() {
        myJammedNumber++;
    }

    /// @brief register a teleport after aborting a long wait
    void registerTeleportAbortWait() {
        myTeleportsAbortWait++;
    }

    /// @brief register a teleport to the final destination
    void registerTeleportWrongDest() {
        myTeleportsWrongDest++;
    }

    /// @brief decrement counter to avoid double counting transportables loaded from state
    void fixLoadCount(const MSTransportable* transportable);

    /// @name Retrieval of transportable statistics (always accessible)
    /// @{

    /** @brief Returns the number of build transportables
     * @return The number of loaded (build) transportables
     */
    int getLoadedNumber() const {
        return myLoadedNumber;
    }

    int getDepartedNumber() const;

    /** @brief Returns the number of build and inserted, but not yet deleted transportables
     * @return The number of simulated transportables
     */
    int getRunningNumber() const {
        return myRunningNumber;
    }

    /** @brief Returns the number of times a transportables was jammed
     * @return The number of times transportables were jammed
     */
    int getJammedNumber() const {
        return myJammedNumber;
    }

    /** @brief Returns the number of transportables waiting for a ride
     */
    int getWaitingForVehicleNumber() const {
        return myWaitingForVehicleNumber;
    }

    /** @brief Returns the number of transportables waiting for a specified
     * amount of time
     */
    int getWaitingUntilNumber() const {
        return myWaitingUntilNumber;
    }

    /** @brief Returns the number of transportables moving by themselvs (i.e. walking)
     */
    int getMovingNumber() const;

    /** @brief Returns the number of transportables riding a vehicle
     */
    int getRidingNumber() const;

    /** @brief Returns the number of transportables that exited the simulation
     */
    int getEndedNumber() const {
        return myEndedNumber;
    }

    /** @brief Returns the number of transportables that arrived at their
     * destination
     */
    int getArrivedNumber() const {
        return myArrivedNumber;
    }

    /** @brief Returns the number of discarded transportables
     */
    int getDiscardedNumber() const {
        return myDiscardedNumber;
    }

    /// @brief return the number of teleports due to excessive waiting for a ride
    int getTeleportsAbortWait() const {
        return myTeleportsAbortWait;
    }

    /// @brief return the number of teleports of transportables riding to the wrong destination
    int getTeleportsWrongDest() const {
        return myTeleportsWrongDest;
    }

    /// @brief Returns the number of teleports transportables did
    int getTeleportCount() const {
        return myTeleportsAbortWait + myTeleportsWrongDest;
    }

    /// @}

    /** @brief Returns the default movement model for this kind of transportables
     * @return The movement model
     */
    inline MSPModel* getMovementModel() {
        return myMovementModel;
    }

    /** @brief Returns the non interacting movement model (for tranship and "beaming")
     * @return The non interacting movement model
     */
    inline MSPModel* getNonInteractingModel() {
        return myNonInteractingModel;
    }

    void addArrived() {
        myArrivedNumber++;
    }

    void addDiscarded() {
        myLoadedNumber++;
        myDiscardedNumber++;
    }

    void startedAccess() {
        myAccessNumber++;
    }

    void endedAccess() {
        myAccessNumber--;
    }

    /** @brief Saves the current state into the given stream
     */
    void saveState(OutputDevice& out);

    /** @brief Reconstruct the current state
     */
    void loadState(const std::string& state);

    /// @brief Resets transportables when quick-loading state
    void clearState();

    const MSDevice_Vehroutes::SortedRouteInfo& getRouteInfo() {
        return myRouteInfos;
    }

protected:
    /// all currently created transportables by id
    std::map<std::string, MSTransportable*> myTransportables;

    /// @brief Transportables waiting for departure
    std::map<SUMOTime, TransportableVector> myWaiting4Departure;

    /// the lists of walking / stopping transportables
    std::map<SUMOTime, TransportableVector> myWaitingUntil;

    /// the lists of waiting transportables
    std::map<const MSEdge*, TransportableVector, ComparatorNumericalIdLess> myWaiting4Vehicle;

    /// @brief The number of build transportables
    int myLoadedNumber;

    /// @brief The number of discarded transportables
    int myDiscardedNumber;

    /// @brief The number of transportables within the network (build and inserted but not removed)
    int myRunningNumber;

    /// @brief The number of jammed transportables
    int myJammedNumber;

    /// @brief The number of transportables waiting for departure
    int myWaitingForDepartureNumber;

    /// @brief The number of transportables waiting for vehicles
    int myWaitingForVehicleNumber;

    /// @brief The number of transportables waiting for a specified time
    int myWaitingUntilNumber;

    /// @brief The number of transportables currently in an access stage
    int myAccessNumber;

    /// @brief The number of transportables that exited the simulation
    int myEndedNumber;

    /// @brief The number of transportables that arrived at their destination
    int myArrivedNumber;

    /// @brief The number of teleports due to long waits for a ride
    int myTeleportsAbortWait;

    /// @brief The number of teleports due to wrong destination
    int myTeleportsWrongDest;

    /// @brief whether a new transportable waiting for a vehicle has been added in the last step
    bool myHaveNewWaiting;

    /// @brief maximum transportable count
    int myMaxTransportableNumber;

private:
    MSPModel* myMovementModel;

    MSPModel* myNonInteractingModel;

    /// @brief Information needed to sort transportable output by departure time
    MSDevice_Vehroutes::SortedRouteInfo myRouteInfos;

    /// @brief The time until waiting for a ride is aborted
    SUMOTime myAbortWaitingTimeout;

private:
    /// @brief invalidated assignment operator
    MSTransportableControl& operator=(const MSTransportableControl& src) = delete;
};
