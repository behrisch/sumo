/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2007-2026 German Aerospace Center (DLR) and others.
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
/// @file    MSDispatch.h
/// @author  Jakob Erdmann
/// @date    16.12.2019
///
// An algorithm that performs dispatch for the taxi device
/****************************************************************************/
#pragma once
#include <config.h>

#include <set>
#include <vector>
#include <map>
#include <utils/common/Parameterised.h>
#include <utils/common/SUMOTime.h>
#include "MSDevice_Taxi.h"

// ===========================================================================
// class declarations
// ===========================================================================
class MSTransportable;
class MSStoppingPlace;

// ===========================================================================
// class definitions
// ===========================================================================
struct Reservation {
    enum ReservationState {
        NEW = 1, // new reservation (not yet retrieved)
        RETRIEVED = 2, // retrieved at least once via MSDispatch_TraCI
        ASSIGNED = 4, // a taxi was dispatched to service this reservation
        ONBOARD = 8, // a taxi has picked up the persons belonging to this reservation
        FULFILLED = 16, // the persons belonging to this reservation have been dropped off
    };

    Reservation(const std::string& _id,
                const std::vector<const MSTransportable*>& _persons,
                SUMOTime _reservationTime,
                SUMOTime _pickupTime,
                SUMOTime _earliestPickupTime,
                const MSEdge* _from, double _fromPos,
                const MSStoppingPlace* _fromStop,
                const MSEdge* _to, double _toPos,
                const MSStoppingPlace* _toStop,
                const std::string& _group,
                const std::string& _line) :
        id(_id),
        persons(_persons.begin(), _persons.end()),
        reservationTime(_reservationTime),
        pickupTime(_pickupTime),
        earliestPickupTime(_earliestPickupTime),
        from(_from),
        fromPos(_fromPos),
        fromStop(_fromStop),
        to(_to),
        toPos(_toPos),
        toStop(_toStop),
        group(_group),
        line(_line),
        recheck(_reservationTime),
        state(NEW)
    {}

    std::string id;
    std::set<const MSTransportable*> persons;
    SUMOTime reservationTime;
    SUMOTime pickupTime;
    SUMOTime earliestPickupTime;
    const MSEdge* from;
    double fromPos;
    const MSStoppingPlace* fromStop;
    const MSEdge* to;
    double toPos;
    const MSStoppingPlace* toStop;
    std::string group;
    std::string line;
    SUMOTime recheck;
    ReservationState state;

    bool operator==(const Reservation& other) const {
        return persons == other.persons
               && reservationTime == other.reservationTime
               && pickupTime == other.pickupTime
               && from == other.from
               && fromPos == other.fromPos
               && to == other.to
               && toPos == other.toPos
               && group == other.group
               && line == other.line;
    }

    /// @brief debug identification
    std::string getID() const;
};

/**
 * @class MSDispatch
 * @brief An algorithm that performs distpach for a taxi fleet
 */
class MSDispatch : public Parameterised {
public:

    /// @brief sorts reservations by time
    class time_sorter {
    public:
        /// @brief Constructor
        explicit time_sorter() {}

        /// @brief Comparing operator
        int operator()(const Reservation* r1, const Reservation* r2) const {
            return MAX2(r1->reservationTime, r1->earliestPickupTime) < MAX2(r2->reservationTime, r2->earliestPickupTime);
        }
    };

    /// @brief Constructor;
    MSDispatch(const Parameterised::Map& params);

    /// @brief Destructor
    virtual ~MSDispatch();

    /// @brief add a new reservation
    virtual Reservation* addReservation(MSTransportable* person,
                                        SUMOTime reservationTime,
                                        SUMOTime pickupTime,
                                        SUMOTime earliestPickupTime,
                                        const MSEdge* from, double fromPos,
                                        const MSStoppingPlace* fromStop,
                                        const MSEdge* to, double toPos,
                                        const MSStoppingPlace* tostop,
                                        std::string group,
                                        const std::string& line,
                                        int maxCapacity,
                                        int maxContainerCapacity);

    /// @brief remove person from reservation. If the whole reservation is removed, return its id
    virtual std::string removeReservation(MSTransportable* person,
                                          const MSEdge* from, double fromPos,
                                          const MSEdge* to, double toPos,
                                          std::string group);

    /// @brief update fromPos of the person's reservation.
    /// TODO: if there is already a reservation with the newFromPos, add to this reservation
    /// TODO: if there are other persons in this reservation, create a new reservation for the updated one
    virtual Reservation* updateReservationFromPos(MSTransportable* person,
            const MSEdge* from, double fromPos,
            const MSEdge* to, double toPos,
            std::string group, double newFromPos);

    /// @brief erase reservation from storage
    virtual void fulfilledReservation(const Reservation* res);

    /// @brief computes dispatch and updates reservations
    virtual void computeDispatch(SUMOTime now, const std::vector<MSDevice_Taxi*>& fleet) = 0;

    /// @brief retrieve all reservations
    std::vector<Reservation*> getReservations();

    /// @brief retrieve all reservations that were already dispatched and are still active
    virtual std::vector<const Reservation*> getRunningReservations();

    /// @brief check whether there are still (servable) reservations in the system
    bool hasServableReservations() {
        return myHasServableReservations;
    }

    virtual SUMOAbstractRouter<MSEdge, SUMOVehicle>& getRouter() const;

    ///@brief compute time to pick up the given reservation
    static SUMOTime computePickupTime(SUMOTime t, const MSDevice_Taxi* taxi, const Reservation& res, SUMOAbstractRouter<MSEdge, SUMOVehicle>& router);

    ///@brief compute whether the reservation is servable
    bool isReachable(SUMOTime t, const MSDevice_Taxi* taxi, const Reservation& res, SUMOAbstractRouter<MSEdge, SUMOVehicle>& router);

    ///@brief compute directTime and detourTime
    static double computeDetourTime(SUMOTime t, SUMOTime viaTime, const MSDevice_Taxi* taxi,
                                    const MSEdge* from, double fromPos,
                                    const MSEdge* via, double viaPos,
                                    const MSEdge* to, double toPos,
                                    SUMOAbstractRouter<MSEdge, SUMOVehicle>& router,
                                    double& timeDirect) ;


    /// @brief whether the last call to computeDispatch has left servable reservations
    bool myHasServableReservations = false;

protected:
    void servedReservation(const Reservation* res, MSDevice_Taxi* taxi);

    /// @brief whether the given taxi has sufficient capacity to serve the reservation
    int remainingCapacity(const MSDevice_Taxi* taxi, const Reservation* res);

    // reservations that are currently being served (could still be used during re-dispatch)
    std::map<std::string, std::map<const Reservation*, MSDevice_Taxi*> > myRunningReservations;

    /// @brief optional file output for dispatch information
    OutputDevice* myOutput;

    int myReservationCount;

    /// @brief the duration before canceling unreachable reservations
    SUMOTime myKeepUnreachableResTime;

    std::map<std::string, std::vector<Reservation*> > myGroupReservations;

    /// @brief which router/edge weights to use
    const int myRoutingMode;

};
