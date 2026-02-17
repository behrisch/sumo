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
/// @file    MSDispatch.cpp
/// @author  Jakob Erdmann
/// @date    16.12.2019
///
// An algorithm that performs dispatch for the taxi device
/****************************************************************************/
#include <config.h>

#include <limits>
#include <microsim/MSNet.h>
#include <microsim/MSEdge.h>
#include <microsim/transportables/MSTransportable.h>
#include "MSRoutingEngine.h"
#include "MSDispatch.h"

//#define DEBUG_RESERVATION
//#define DEBUG_DETOUR
//#define DEBUG_COND2(obj) (obj->getID() == "p0")
#define DEBUG_COND2(obj) (true)


// ===========================================================================
// Reservation methods
// ===========================================================================

std::string
Reservation::getID() const {
    return toString(persons);
}

// ===========================================================================
// MSDispatch methods
// ===========================================================================

MSDispatch::MSDispatch(const Parameterised::Map& params) :
    Parameterised(params),
    myOutput(nullptr),
    myReservationCount(0),
    myRoutingMode(StringUtils::toInt(getParameter("routingMode", "1")))
{
    const std::string opt = "device.taxi.dispatch-algorithm.output";
    if (OptionsCont::getOptions().isSet(opt)) {
        OutputDevice::createDeviceByOption(opt, "DispatchInfo");
        myOutput = &OutputDevice::getDeviceByOption(opt);
    }
    myKeepUnreachableResTime = string2time(OptionsCont::getOptions().getString("device.taxi.dispatch-keep-unreachable"));
}

MSDispatch::~MSDispatch() {
    for (auto item : myGroupReservations) {
        for (Reservation* res : item.second) {
            delete res;
        }
    }
    myGroupReservations.clear();
}


Reservation*
MSDispatch::addReservation(MSTransportable* person,
                           SUMOTime reservationTime,
                           SUMOTime pickupTime,
                           SUMOTime earliestPickupTime,
                           const MSEdge* from, double fromPos,
                           const MSStoppingPlace* fromStop,
                           const MSEdge* to, double toPos,
                           const MSStoppingPlace* toStop,
                           std::string group,
                           const std::string& line,
                           int maxCapacity,
                           int maxContainerCapacity) {
    // no new reservation nedded if the person can be added to an existing group
    if (group == "") {
        // the default empty group implies, no grouping is wanted (and
        // transportable ids are unique)
        group = person->getID();
    }
    Reservation* result = nullptr;
    bool added = false;
    auto it = myGroupReservations.find(group);
    if (it != myGroupReservations.end()) {
        // try to add to existing reservation
        for (Reservation* res : it->second) {
            if (res->persons.count(person) == 0
                    && res->from == from
                    && res->to == to
                    && res->fromPos == fromPos
                    && res->toPos == toPos) {
                if (res->persons.size() > 0 && (*res->persons.begin())->isPerson() != person->isPerson()) {
                    WRITE_WARNINGF(TL("Mixing reservations of persons and containers with the same group is not supported for % and %"),
                                   (*res->persons.begin())->getID(), person->getID());
                }
                if ((person->isPerson() && (int)res->persons.size() >= maxCapacity) ||
                        (!person->isPerson() && (int)res->persons.size() >= maxContainerCapacity)) {
                    // split group to ensure that at least one taxi is capable of delivering group size.
                    continue;
                }
                res->persons.insert(person);
                result = res;
                added = true;
                break;
            }
        }
    }
    if (!added) {
        Reservation* newRes = new Reservation(toString(myReservationCount++), {person}, reservationTime, pickupTime, earliestPickupTime, from, fromPos, fromStop, to, toPos, toStop, group, line);
        myGroupReservations[group].push_back(newRes);
        result = newRes;
    }
    myHasServableReservations = true;
#ifdef DEBUG_RESERVATION
    if (DEBUG_COND2(person)) std::cout << SIMTIME
                                           << " addReservation p=" << person->getID()
                                           << " rT=" << time2string(reservationTime)
                                           << " pT=" << time2string(pickupTime)
                                           << " from=" << from->getID() << " fromPos=" << fromPos
                                           << " to=" << to->getID() << " toPos=" << toPos
                                           << " group=" << group
                                           << " added=" << added
                                           << "\n";
#endif
    return result;
}


std::string
MSDispatch::removeReservation(MSTransportable* person,
                              const MSEdge* from, double fromPos,
                              const MSEdge* to, double toPos,
                              std::string group) {
    if (group == "") {
        // the default empty group implies, no grouping is wanted (and
        // transportable ids are unique)
        group = person->getID();
    }
    std::string removedID = "";
    auto it = myGroupReservations.find(group);
    if (it != myGroupReservations.end()) {
        for (auto itRes = it->second.begin(); itRes != it->second.end(); itRes++) {
            Reservation* res = *itRes;
            if (res->persons.count(person) != 0
                    && res->from == from
                    && res->to == to
                    && res->fromPos == fromPos
                    && res->toPos == toPos) {
                res->persons.erase(person);
                if (res->persons.empty()) {
                    removedID = res->id;
                    it->second.erase(itRes);
                    // cleans up MSDispatch_Greedy
                    fulfilledReservation(res);
                    if (it->second.empty()) {
                        myGroupReservations.erase(it);
                    }
                }
                break;
            }
        }
    } else {
        auto it2 = myRunningReservations.find(group);
        if (it2 != myRunningReservations.end()) {
            for (auto item : it2->second) {
                const Reservation* res = item.first;
                if (res->persons.count(person) != 0
                        && res->from == from
                        && res->to == to
                        && res->fromPos == fromPos
                        && res->toPos == toPos) {
                    MSDevice_Taxi* taxi = item.second;
                    taxi->cancelCustomer(person);
                    if (res->persons.empty()) {
                        removedID = res->id;
                    }
                    break;
                }
            }
        }
    }
    myHasServableReservations = myGroupReservations.size() > 0;
#ifdef DEBUG_RESERVATION
    if (DEBUG_COND2(person)) std::cout << SIMTIME
                                           << " removeReservation p=" << person->getID()
                                           << " from=" << from->getID() << " fromPos=" << fromPos
                                           << " to=" << to->getID() << " toPos=" << toPos
                                           << " group=" << group
                                           << " removedID=" << removedID
                                           << " hasServable=" << myHasServableReservations
                                           << "\n";
#endif
    return removedID;
}


Reservation*
MSDispatch::updateReservationFromPos(MSTransportable* person,
                                     const MSEdge* from, double fromPos,
                                     const MSEdge* to, double toPos,
                                     std::string group, double newFromPos) {
    if (group == "") {
        // the default empty group implies, no grouping is wanted (and
        // transportable ids are unique)
        group = person->getID();
    }
    Reservation* result = nullptr;
    std::string updatedID = "";
    auto it = myGroupReservations.find(group);
    if (it != myGroupReservations.end()) {
        for (auto itRes = it->second.begin(); itRes != it->second.end(); itRes++) {
            Reservation* res = *itRes;
            // TODO: if there is already a reservation with the newFromPos, add to this reservation
            // TODO: if there are other persons in this reservation, create a new reservation for the updated one
            if (res->persons.count(person) != 0
                    && res->from == from
                    && res->to == to
                    && res->fromPos == fromPos
                    && res->toPos == toPos) {
                // update fromPos
                res->fromPos = newFromPos;
                result = res;
                updatedID = res->id;
                break;
            }
        }
    }
#ifdef DEBUG_RESERVATION
    if (DEBUG_COND2(person)) std::cout << SIMTIME
                                           << " updateReservationFromPos p=" << person->getID()
                                           << " from=" << from->getID() << " fromPos=" << fromPos
                                           << " to=" << to->getID() << " toPos=" << toPos
                                           << " group=" << group
                                           << " newFromPos=" << newFromPos
                                           << " updatedID=" << updatedID
                                           << "\n";
#endif
    return result;
}


std::vector<Reservation*>
MSDispatch::getReservations() {
    std::vector<Reservation*> reservations;
    for (const auto& it : myGroupReservations) {
        reservations.insert(reservations.end(), it.second.begin(), it.second.end());
    }
    return reservations;
}


std::vector<const Reservation*>
MSDispatch::getRunningReservations() {
    std::vector<const Reservation*> result;
    for (auto item : myRunningReservations) {
        for (auto item2 : item.second) {
            result.push_back(item2.first);
        }
    }
    return result;
}


void
MSDispatch::servedReservation(const Reservation* res, MSDevice_Taxi* taxi) {
    auto itR = myRunningReservations.find(res->group);
    if (itR != myRunningReservations.end() && itR->second.count(res) != 0) {
        return; // was redispatch
    }
    auto it = myGroupReservations.find(res->group);
    if (it == myGroupReservations.end()) {
        throw ProcessError(TL("Inconsistent group reservations."));
    }
    auto it2 = std::find(it->second.begin(), it->second.end(), res);
    if (it2 == it->second.end()) {
        throw ProcessError(TL("Inconsistent group reservations (2)."));
    }
    myRunningReservations[res->group][res] = taxi;
    const_cast<Reservation*>(*it2)->state = Reservation::ASSIGNED;
    it->second.erase(it2);
    if (it->second.empty()) {
        myGroupReservations.erase(it);
    }
}


void
MSDispatch::fulfilledReservation(const Reservation* res) {
    myRunningReservations[res->group].erase(res);
    if (myRunningReservations[res->group].empty()) {
        myRunningReservations.erase(res->group);
    }
    delete res;
}


SUMOAbstractRouter<MSEdge, SUMOVehicle>&
MSDispatch::getRouter() const {
    return myRoutingMode == 1 ? MSRoutingEngine::getRouterTT(0, SVC_TAXI) : MSNet::getInstance()->getRouterTT(0);
}


SUMOTime
MSDispatch::computePickupTime(SUMOTime t, const MSDevice_Taxi* taxi, const Reservation& res, SUMOAbstractRouter<MSEdge, SUMOVehicle>& router) {
    ConstMSEdgeVector edges;
    double fromPos = taxi->getHolder().getPositionOnLane() - NUMERICAL_EPS;
    const MSEdge* from = *taxi->getHolder().getRerouteOrigin();
    const bool originDiffers = from != taxi->getHolder().getEdge();
    router.compute(from, originDiffers ? 0 : fromPos, res.from, res.fromPos, &taxi->getHolder(), t, edges, true);
    if (edges.empty()) {
        return SUMOTime_MAX;
    } else {
        if (originDiffers) {
            assert(from = *(taxi->getHolder().getCurrentRouteEdge() + 1));
            edges.insert(edges.begin(), taxi->getHolder().getEdge());
        }
        return TIME2STEPS(router.recomputeCostsPos(edges, &taxi->getHolder(), fromPos, res.fromPos, t));
    }
}


bool
MSDispatch::isReachable(SUMOTime t, const MSDevice_Taxi* taxi, const Reservation& res, SUMOAbstractRouter<MSEdge, SUMOVehicle>& router) {
    ConstMSEdgeVector edges;
    router.compute(res.from, res.fromPos, res.to, res.toPos, &taxi->getHolder(), t, edges, true);
    return !edges.empty();
}


double
MSDispatch::computeDetourTime(SUMOTime t, SUMOTime viaTime, const MSDevice_Taxi* taxi,
                              const MSEdge* from, double fromPos,
                              const MSEdge* via, double viaPos,
                              const MSEdge* to, double toPos,
                              SUMOAbstractRouter<MSEdge, SUMOVehicle>& router,
                              double& timeDirect) {
    ConstMSEdgeVector edges;
    if (timeDirect < 0) {
        router.compute(from, fromPos, to, toPos, &taxi->getHolder(), t, edges, true);
        timeDirect = router.recomputeCostsPos(edges, &taxi->getHolder(), fromPos, toPos, t);
        edges.clear();
    }

    router.compute(from, fromPos, via, viaPos, &taxi->getHolder(), t, edges, true);
    const double start = STEPS2TIME(t);
    const double leg1 = router.recomputeCostsPos(edges, &taxi->getHolder(), fromPos, viaPos, t);
#ifdef DEBUG_DETOUR
    std::cout << "        leg1=" << toString(edges) << " startPos=" << fromPos << " toPos=" << viaPos << " time=" << leg1 << "\n";
#endif
    const double wait = MAX2(0.0, STEPS2TIME(viaTime) - (start + leg1));
    edges.clear();
    const SUMOTime timeContinue = TIME2STEPS(start + leg1 + wait);
    router.compute(via, viaPos, to, toPos, &taxi->getHolder(), timeContinue, edges, true);
    const double leg2 = router.recomputeCostsPos(edges, &taxi->getHolder(), viaPos, toPos, timeContinue);
    const double timeDetour = leg1 + wait + leg2;
#ifdef DEBUG_DETOUR
    std::cout << "        leg2=" << toString(edges) << " startPos=" << viaPos << " toPos=" << toPos << " time=" << leg2 << "\n";
    std::cout << "    t=" << STEPS2TIME(t) << " vt=" << STEPS2TIME(viaTime)
              << " from=" << from->getID() << " to=" << to->getID() << " via=" << via->getID()
              << " direct=" << timeDirect << " detour=" << timeDetour << " wait=" << wait << "\n";
#endif
    return timeDetour;
}


int
MSDispatch::remainingCapacity(const MSDevice_Taxi* taxi, const Reservation* res) {
    assert(res->persons.size() > 0);
    return ((*res->persons.begin())->isPerson()
            ? taxi->getHolder().getVehicleType().getPersonCapacity()
            : taxi->getHolder().getVehicleType().getContainerCapacity()) - (int)res->persons.size();
}


/****************************************************************************/
