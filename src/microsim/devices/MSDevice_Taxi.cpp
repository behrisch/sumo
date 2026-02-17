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
/// @file    MSDevice_Taxi.cpp
/// @author  Jakob Erdmann
/// @date    16.12.2019
///
// A device which controls a taxi
/****************************************************************************/
#include <config.h>

#include <utils/common/StringUtils.h>
#include <utils/common/StaticCommand.h>
#include <utils/common/StringTokenizer.h>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/vehicle/SUMOVehicle.h>
#include <utils/router/SUMOAbstractRouter.h>
#include <microsim/transportables/MSTransportable.h>
#include <microsim/MSEventControl.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSStop.h>
#include <microsim/MSStoppingPlace.h>
#include <microsim/trigger/MSTriggeredRerouter.h>

#include "MSDispatch.h"
#include "MSDispatch_Greedy.h"
#include "MSDispatch_GreedyShared.h"
#include "MSDispatch_RouteExtension.h"
#include "MSDispatch_TraCI.h"

#include "MSIdling.h"

#include "MSRoutingEngine.h"
#include "MSDevice_Routing.h"
#include "MSDevice_Taxi.h"

//#define DEBUG_DISPATCH
//#define DEBUG_CANCEL

//#define DEBUG_COND (myHolder.isSelected())
#define DEBUG_COND (true)

// ===========================================================================
// static member variables
// ===========================================================================
SUMOTime MSDevice_Taxi::myDispatchPeriod(0);
/// @brief the dispatch algorithm
MSDispatch* MSDevice_Taxi::myDispatcher(nullptr);
/// @brief The repeated call to the dispatcher
Command* MSDevice_Taxi::myDispatchCommand(nullptr);
// @brief the list of available taxis
std::vector<MSDevice_Taxi*> MSDevice_Taxi::myFleet;
int MSDevice_Taxi::myMaxCapacity(0);
int MSDevice_Taxi::myMaxContainerCapacity(0);
std::set<std::string> MSDevice_Taxi::myVClassWarningVTypes;

#define TAXI_SERVICE "taxi"
#define TAXI_SERVICE_PREFIX "taxi:"
#define SWAP_THRESHOLD 5

// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_Taxi::insertOptions(OptionsCont& oc) {
    oc.addOptionSubTopic("Taxi Device");
    insertDefaultAssignmentOptions("taxi", "Taxi Device", oc);

    oc.doRegister("device.taxi.dispatch-algorithm", new Option_String("greedy"));
    oc.addDescription("device.taxi.dispatch-algorithm", "Taxi Device", TL("The dispatch algorithm [greedy|greedyClosest|greedyShared|routeExtension|traci]"));

    oc.doRegister("device.taxi.dispatch-algorithm.output", new Option_FileName());
    oc.addDescription("device.taxi.dispatch-algorithm.output", "Taxi Device", TL("Write information from the dispatch algorithm to FILE"));

    oc.doRegister("device.taxi.dispatch-algorithm.params", new Option_String(""));
    oc.addDescription("device.taxi.dispatch-algorithm.params", "Taxi Device", TL("Load dispatch algorithm parameters in format KEY1:VALUE1[,KEY2:VALUE]"));

    oc.doRegister("device.taxi.dispatch-period", new Option_String("60", "TIME"));
    oc.addDescription("device.taxi.dispatch-period", "Taxi Device", TL("The period between successive calls to the dispatcher"));

    oc.doRegister("device.taxi.dispatch-keep-unreachable", new Option_String("3600", "TIME"));
    oc.addDescription("device.taxi.dispatch-keep-unreachable", "Taxi Device", TL("The time before aborting unreachable reservations"));

    oc.doRegister("device.taxi.idle-algorithm", new Option_String("stop"));
    oc.addDescription("device.taxi.idle-algorithm", "Taxi Device", TL("The behavior of idle taxis [stop|randomCircling|taxistand]"));

    oc.doRegister("device.taxi.idle-algorithm.output", new Option_FileName());
    oc.addDescription("device.taxi.idle-algorithm.output", "Taxi Device", TL("Write information from the idling algorithm to FILE"));
}


void
MSDevice_Taxi::buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into) {
    OptionsCont& oc = OptionsCont::getOptions();
    if (equippedByDefaultAssignmentOptions(oc, "taxi", v, false)) {
        // build the device
        MSDevice_Taxi* device = new MSDevice_Taxi(v, "taxi_" + v.getID());
        into.push_back(device);
        myFleet.push_back(device);
        if (v.getParameter().line == "") {
            // automatically set the line so that persons are willing to enter
            // (see MSStageDriving::isWaitingFor)
            const_cast<SUMOVehicleParameter&>(v.getParameter()).line = TAXI_SERVICE;
        }
        if (v.getVClass() != SVC_TAXI && myVClassWarningVTypes.count(v.getVehicleType().getID()) == 0) {
            WRITE_WARNINGF(TL("Vehicle '%' with device.taxi should have vClass taxi instead of '%'."), v.getID(), toString(v.getVClass()));
            myVClassWarningVTypes.insert(v.getVehicleType().getID());
        }
        const int personCapacity = v.getVehicleType().getPersonCapacity();
        const int containerCapacity = v.getVehicleType().getContainerCapacity();
        myMaxCapacity = MAX2(myMaxCapacity, personCapacity);
        myMaxContainerCapacity = MAX2(myMaxContainerCapacity, containerCapacity);
        if (personCapacity < 1 && containerCapacity < 1) {
            WRITE_WARNINGF(TL("Vehicle '%' with personCapacity % and containerCapacity % is not usable as taxi."), v.getID(), toString(personCapacity), toString(containerCapacity));
        }
    }
}


void
MSDevice_Taxi::initDispatch() {
    OptionsCont& oc = OptionsCont::getOptions();
    myDispatchPeriod = string2time(oc.getString("device.taxi.dispatch-period"));
    // init dispatch algorithm
    std::string algo = oc.getString("device.taxi.dispatch-algorithm");
    Parameterised params;
    params.setParametersStr(OptionsCont::getOptions().getString("device.taxi.dispatch-algorithm.params"), ":", ",");
    if (algo == "greedy") {
        myDispatcher = new MSDispatch_Greedy(params.getParametersMap());
    } else if (algo == "greedyClosest") {
        myDispatcher = new MSDispatch_GreedyClosest(params.getParametersMap());
    } else if (algo == "greedyShared") {
        myDispatcher = new MSDispatch_GreedyShared(params.getParametersMap());
    } else if (algo == "routeExtension") {
        myDispatcher = new MSDispatch_RouteExtension(params.getParametersMap());
    } else if (algo == "traci") {
        myDispatcher = new MSDispatch_TraCI(params.getParametersMap());
    } else {
        throw ProcessError(TLF("Dispatch algorithm '%' is not known", algo));
    }
    myDispatchCommand = new StaticCommand<MSDevice_Taxi>(&MSDevice_Taxi::triggerDispatch);
    // round to next multiple of myDispatchPeriod
    const SUMOTime now = MSNet::getInstance()->getCurrentTimeStep();
    const SUMOTime begin = string2time(oc.getString("begin"));
    const SUMOTime delay = (myDispatchPeriod - ((now - begin) % myDispatchPeriod)) % myDispatchPeriod;
    MSNet::getInstance()->getEndOfTimestepEvents()->addEvent(myDispatchCommand, now + delay);
}

bool
MSDevice_Taxi::isReservation(const std::set<std::string>& lines) {
    return lines.size() == 1 && (
               *lines.begin() == TAXI_SERVICE
               || StringUtils::startsWith(*lines.begin(), TAXI_SERVICE_PREFIX));
}

void
MSDevice_Taxi::addReservation(MSTransportable* person,
                              const std::set<std::string>& lines,
                              SUMOTime reservationTime,
                              SUMOTime pickupTime,
                              SUMOTime earliestPickupTime,
                              const MSEdge* from, double fromPos,
                              const MSStoppingPlace* fromStop,
                              const MSEdge* to, double toPos,
                              const MSStoppingPlace* toStop,
                              const std::string& group) {
    if (!isReservation(lines)) {
        return;
    }
    if ((to->getPermissions() & SVC_TAXI) == 0) {
        throw ProcessError("Cannot add taxi reservation for " + std::string(person->isPerson() ? "person" : "container")
                           + " '" + person->getID() + "' because destination edge '" + to->getID() + "'"
                           + " does not permit taxi access");
    }
    if ((from->getPermissions() & SVC_TAXI) == 0) {
        throw ProcessError("Cannot add taxi reservation for " + std::string(person->isPerson() ? "person" : "container")
                           + " '" + person->getID() + "' because origin edge '" + from->getID() + "'"
                           + " does not permit taxi access");
    }
    if (myDispatchCommand == nullptr) {
        initDispatch();
    }
    if (fromStop != nullptr && &fromStop->getLane().getEdge() == from) {
        // pickup position should be at the stop-endPos
        fromPos = fromStop->getEndLanePosition();
    }
    myDispatcher->addReservation(person, reservationTime, pickupTime, earliestPickupTime, from, fromPos, fromStop, to, toPos, toStop, group, *lines.begin(), myMaxCapacity, myMaxContainerCapacity);
}

void
MSDevice_Taxi::removeReservation(MSTransportable* person,
                                 const std::set<std::string>& lines,
                                 const MSEdge* from, double fromPos,
                                 const MSEdge* to, double toPos,
                                 const std::string& group) {
    if (myDispatcher != nullptr && lines.size() == 1 && *lines.begin() == TAXI_SERVICE) {
        myDispatcher->removeReservation(person, from, fromPos, to, toPos, group);
    }
}

void
MSDevice_Taxi::updateReservationFromPos(MSTransportable* person,
                                        const std::set<std::string>& lines,
                                        const MSEdge* from, double fromPos,
                                        const MSEdge* to, double toPos,
                                        const std::string& group, double newFromPos) {
    if (myDispatcher != nullptr && lines.size() == 1 && *lines.begin() == TAXI_SERVICE) {
        myDispatcher->updateReservationFromPos(person, from, fromPos, to, toPos, group, newFromPos);
    }
}


SUMOTime
MSDevice_Taxi::triggerDispatch(SUMOTime currentTime) {
    std::vector<MSDevice_Taxi*> active;
    for (MSDevice_Taxi* taxi : myFleet) {
        if (taxi->getHolder().hasDeparted()) {
            active.push_back(taxi);
        }
    }
    myDispatcher->computeDispatch(currentTime, active);
    return myDispatchPeriod;
}

bool
MSDevice_Taxi::hasServableReservations() {
    return myDispatcher != nullptr && myDispatcher->hasServableReservations();
}

void
MSDevice_Taxi::cleanup() {
    if (myDispatcher != nullptr) {
        delete myDispatcher;
        myDispatcher = nullptr;
    }
    myDispatchCommand = nullptr;
    myVClassWarningVTypes.clear();
}

// ---------------------------------------------------------------------------
// MSDevice_Taxi-methods
// ---------------------------------------------------------------------------
MSDevice_Taxi::MSDevice_Taxi(SUMOVehicle& holder, const std::string& id) :
    MSVehicleDevice(holder, id) {
    std::string defaultServiceEnd = toString(1e15);
    const std::string algo = holder.getStringParam("device.taxi.idle-algorithm");
    if (algo == "stop") {
        myIdleAlgorithm = new MSIdling_Stop();
    } else if (algo == "randomCircling") {
        myIdleAlgorithm = new MSIdling_RandomCircling();
        // make sure simulation terminates
        defaultServiceEnd = toString(STEPS2TIME(
                                         myHolder.getParameter().departProcedure == DepartDefinition::GIVEN
                                         ? myHolder.getParameter().depart
                                         : MSNet::getInstance()->getCurrentTimeStep()) + (3600 * 8));
    } else if (algo == "taxistand") {
        const std::string rerouterID = holder.getStringParam("device.taxi.stands-rerouter");
        if (rerouterID.empty()) {
            throw ProcessError("Idle algorithm '" + algo + "' requires a rerouter id to be defined using device param 'stands-rerouter' for vehicle '" + myHolder.getID() + "'");
        }
        if (MSTriggeredRerouter::getInstances().count(rerouterID) == 0) {
            throw ProcessError("Unknown rerouter '" + rerouterID + "' when loading taxi stands for vehicle '" + myHolder.getID() + "'");
        }
        MSTriggeredRerouter* rerouter = MSTriggeredRerouter::getInstances().find(rerouterID)->second;
        myIdleAlgorithm = new MSIdling_TaxiStand(rerouter);
    } else {
        throw ProcessError("Idle algorithm '" + algo + "' is not known for vehicle '" + myHolder.getID() + "'");
    }
    myServiceEnd = string2time(holder.getStringParam("device.taxi.end", false, defaultServiceEnd));
    myRoutingDevice = static_cast<MSDevice_Routing*>(myHolder.getDevice(typeid(MSDevice_Routing)));
}


MSDevice_Taxi::~MSDevice_Taxi() {
    myFleet.erase(std::find(myFleet.begin(), myFleet.end(), this));
    // recompute myMaxCapacity
    myMaxCapacity = 0;
    myMaxContainerCapacity = 0;
    for (MSDevice_Taxi* taxi : myFleet) {
        myMaxCapacity = MAX2(myMaxCapacity, taxi->getHolder().getVehicleType().getPersonCapacity());
        myMaxContainerCapacity = MAX2(myMaxContainerCapacity, taxi->getHolder().getVehicleType().getContainerCapacity());
    }
    delete myIdleAlgorithm;
}


bool
MSDevice_Taxi::hasFleet() {
    return myFleet.size() > 0;;
}


void
MSDevice_Taxi::dispatch(const Reservation& res) {
    dispatchShared({&res, &res});
}


void
MSDevice_Taxi::dispatchShared(std::vector<const Reservation*> reservations) {
#ifdef DEBUG_DISPATCH
    if (DEBUG_COND) {
        std::cout << SIMTIME << " taxi=" << myHolder.getID() << " dispatch:\n";
        for (const Reservation* res : reservations) {
            std::cout << "   persons=" << toString(res->persons) << "\n";
        }
    }
#endif
    myLastDispatch = reservations;
    ConstMSEdgeVector tmpEdges;
    StopParVector stops;
    double lastPos = myHolder.getPositionOnLane();
    const MSEdge* rerouteOrigin = *myHolder.getRerouteOrigin();
    if (isEmpty()) {
        // start fresh from the current edge
        if (myHolder.isStoppedParking()) {
            // parking stop must be ended normally
            MSStop& stop = myHolder.getNextStopMutable();
            stop.duration = 0;
            lastPos = stop.pars.endPos;
            if (myHolder.isStoppedTriggered()) {
                stop.triggered = false;
                stop.containerTriggered = false;
                stop.joinTriggered = false;
                myHolder.unregisterWaiting();
            }
            // prevent unauthorized/premature entry
            const_cast<SUMOVehicleParameter::Stop&>(stop.pars).permitted.insert("");
            while (myHolder.getStops().size() > 1) {
                myHolder.abortNextStop(1);
            }
        } else {
            while (myHolder.hasStops()) {
                // in meso there might be more than 1 stop at this point
                myHolder.abortNextStop();
            }
            assert(!myHolder.hasStops());
        }
        tmpEdges.push_back(myHolder.getEdge());
        if (myHolder.getEdge() != rerouteOrigin) {
            tmpEdges.push_back(rerouteOrigin);
        }
    } else {
        assert(myHolder.hasStops());
        // check how often existing customers appear in the new reservations
        std::map<const MSTransportable*, int> nOccur;
        for (const Reservation* res : reservations) {
            for (const MSTransportable* person : res->persons) {
                if (myCustomers.count(person) != 0) {
                    nOccur[person] += 1;
                    if (myCurrentReservations.count(res) == 0) {
                        throw ProcessError(TLF("Invalid Re-dispatch for existing customer '%' with a new reservation", person->getID()));
                    }
                }
            }
        }
#ifdef DEBUG_DISPATCH
        if (DEBUG_COND) {
            for (auto item : nOccur) {
                std::cout << "   previousCustomer=" << item.first->getID() << " occurs=" << item.second << "\n";
            }
        }
#endif
        if (nOccur.size() == 0) {
            // no overlap with existing customers - extend route
            tmpEdges = myHolder.getRoute().getEdges();
            lastPos = myHolder.getStops().back().pars.endPos;
#ifdef DEBUG_DISPATCH
            if (DEBUG_COND) {
                std::cout << " re-dispatch with route-extension\n";
            }
#endif
        } else if (nOccur.size() == myCustomers.size()) {
            // redefine route (verify correct number of mentions)
            std::set<const MSTransportable*> onBoard;
            const std::vector<MSTransportable*>& onBoardP = myHolder.getPersons();
            const std::vector<MSTransportable*>& onBoardC = myHolder.getContainers();
            onBoard.insert(onBoardP.begin(), onBoardP.end());
            onBoard.insert(onBoardC.begin(), onBoardC.end());
            std::set<const MSTransportable*> redundantPickup;
            for (auto item : nOccur) {
                if (item.second == 1) {
                    // customers must already be on board
                    if (onBoard.count(item.first) == 0) {
                        throw ProcessError(TLF("Re-dispatch did not mention pickup for existing customer '%'", item.first->getID()));
                    }
                } else if (item.second == 2) {
                    if (onBoard.count(item.first) == 0) {
                        // treat like a new customer
                        // TODO: need to be checked
                        myCustomers.erase(item.first);
                    } else {
                        redundantPickup.insert(item.first);
                    }
                } else {
                    throw ProcessError("Re-dispatch mentions existing customer '" + item.first->getID() + "' " + toString(item.second) + " times");
                }
            }
            // remove redundancy
            if (!redundantPickup.empty()) {
                for (auto it = reservations.begin(); it != reservations.end();) {
                    bool isRedundant = false;
                    for (const MSTransportable* person : (*it)->persons) {
                        if (redundantPickup.count(person) != 0) {
                            isRedundant = true;
                            break;
                        }
                    }
                    if (isRedundant) {
                        for (const MSTransportable* person : (*it)->persons) {
                            redundantPickup.erase(person);
                        }
                        it = reservations.erase(it);
                    } else {
                        it++;
                    }
                }
            }
            while (myHolder.hasStops()) {
                myHolder.abortNextStop();
            }
            tmpEdges.push_back(myHolder.getEdge());
            if (myHolder.getEdge() != rerouteOrigin) {
                tmpEdges.push_back(rerouteOrigin);
            }
#ifdef DEBUG_DISPATCH
            if (DEBUG_COND) {
                std::cout << " re-dispatch from scratch\n";
            }
#endif
        } else {
            // inconsistent re-dispatch
            std::vector<std::string> missing;
            for (const MSTransportable* c : myCustomers) {
                if (nOccur.count(c) == 0) {
                    missing.push_back(c->getID());
                }
            }
            throw ProcessError("Re-dispatch did mention some customers but failed to mention " + joinToStringSorting(missing, " "));
        }
    }

    const SUMOTime t = MSNet::getInstance()->getCurrentTimeStep();
    bool hasPickup = false;
    for (const Reservation* res : reservations) {
        myCurrentReservations.insert(res);
        bool isPickup = false;
        for (const MSTransportable* person : res->persons) {
            if (myCustomers.count(person) == 0) {
                myCustomers.insert(person);
                isPickup = true;
                hasPickup = true;
            }
        }
        if (isPickup) {
            prepareStop(tmpEdges, stops, lastPos, res->from, res->fromPos, res->fromStop, "pickup " + toString(res->persons) + " (" + res->id + ")", res, isPickup);
            for (const MSTransportable* const transportable : res->persons) {
                if (transportable->isPerson()) {
                    stops.back().triggered = true;
                } else {
                    stops.back().containerTriggered = true;
                }
                stops.back().permitted.insert(transportable->getID());
            }
            // proof this lines: Is needed for pre-booking?
            std::set<const MSTransportable*> persons = res->persons;
            for (auto itr = persons.begin(); itr != persons.end(); itr++) {
                stops.back().awaitedPersons.insert((*itr)->getID());
            }

            stops.back().parametersSet |= STOP_PERMITTED_SET;
            if (stops.back().duration == -1) {
                // keep dropOffDuration if the stop is dropOff and pickUp
                stops.back().duration = TIME2STEPS(myHolder.getFloatParam("device.taxi.pickUpDuration", false, 0));
            }
        } else {
            prepareStop(tmpEdges, stops, lastPos, res->to, res->toPos, res->toStop, "dropOff " + toString(res->persons) + " (" + res->id + ")", res, isPickup);
            stops.back().duration = TIME2STEPS(myHolder.getFloatParam("device.taxi.dropOffDuration", false, 60)); // pay and collect bags
        }
    }
#ifdef DEBUG_DISPATCH
    if (DEBUG_COND) {
        std::cout << "   tmpEdges=" << toString(tmpEdges) << "\n";
    }
#endif
    if (!myHolder.replaceRouteEdges(tmpEdges, -1, 0, "taxi:prepare_dispatch", false, false, false)) {
        throw ProcessError("Route replacement for taxi dispatch failed for vehicle '" + myHolder.getID()
                           + "' at time=" + time2string(t) + ".");
    }
#ifdef DEBUG_DISPATCH
    if (DEBUG_COND) std::cout << "   replacedRoute=" << toString(tmpEdges)
                                  << "\n     actualRoute=" << toString(myHolder.getRoute().getEdges()) << "\n";
#endif
    for (SUMOVehicleParameter::Stop& stop : stops) {
        std::string error;
        myHolder.addStop(stop, error);
        if (error != "") {
            WRITE_WARNINGF(TL("Could not add taxi stop, time=%, error=%"), myHolder.getID(), stop.actType, time2string(t), error)
        }
    }
    SUMOAbstractRouter<MSEdge, SUMOVehicle>& router = MSRoutingEngine::getRouterTT(myHolder.getRNGIndex(), myHolder.getVClass());
    // SUMOAbstractRouter<MSEdge, SUMOVehicle>& router = myHolder.getInfluencer().getRouterTT(veh->getRNGIndex())
    myHolder.reroute(t, "taxi:dispatch", router, false);
#ifdef DEBUG_DISPATCH
    if (DEBUG_COND) {
        std::cout << "\n      finalRoute=" << toString(myHolder.getRoute().getEdges()) << " routeIndex=" << myHolder.getRoutePosition() << "\n";
    }
#endif
    if (hasPickup) {
        myState |= PICKUP;
    }
}


void
MSDevice_Taxi::cancelCurrentCustomers() {
    // check if taxi has stopped
    if (myHolder.getNextStopParameter() == nullptr) {
        return;
    }
    // find customers of the current stop
    std::set<const MSTransportable*> customersToBeRemoved;
    std::set<const MSTransportable*> onBoard;
    onBoard.insert(myHolder.getPersons().begin(), myHolder.getPersons().end());
    onBoard.insert(myHolder.getContainers().begin(), myHolder.getContainers().end());
    for (std::string tID : myHolder.getNextStopParameter()->permitted) {
        for (auto t : myCustomers) {
            if (t->getID() == tID && onBoard.count(t) == 0) {
                customersToBeRemoved.insert(t);
            }
        }
    }
    if (!customersToBeRemoved.empty()) {
        WRITE_WARNINGF(TL("Taxi '%' aborts waiting for customers: % at time=%."),
                       myHolder.getID(), toString(customersToBeRemoved), time2string(SIMSTEP));
    }
    for (auto t : customersToBeRemoved) {
        cancelCustomer(t);
    }
}


bool
MSDevice_Taxi::cancelCustomer(const MSTransportable* t) {
#ifdef DEBUG_CANCEL
    if (DEBUG_COND) {
        std::cout << SIMTIME << " taxi=" << myHolder.getID() << " cancelCustomer " << t->getID() << "\n";
    }
#endif

    // is the given transportable a customer of the reservations?
    if (myCustomers.count(t) == 0) {
        return false;
    }
    myCustomers.erase(t);
    // check whether a single reservation has been fulfilled or another customer is part of the reservation
    for (auto resIt = myCurrentReservations.begin(); resIt != myCurrentReservations.end();) {
        bool fulfilled = false;
        if ((*resIt)->persons.size() == 1 && (*resIt)->persons.count(t) != 0) {
            // the reservation contains only the customer
            fulfilled = true;
        }
        if (fulfilled) {
            const Reservation* res = *resIt;
            // remove reservation from the current dispatch
            for (auto it = myLastDispatch.begin(); it != myLastDispatch.end();) {
                if (*it == res) {
                    it = myLastDispatch.erase(it);
                } else {
                    ++it;
                }
            }
            // remove reservation from the served reservations
            resIt = myCurrentReservations.erase(resIt);
            // delete the reservation
            myDispatcher->fulfilledReservation(res);
        } else {
            ++resIt;
        }
    }
    myState &= ~PICKUP;  // remove state PICKUP
    for (const Reservation* res : myCurrentReservations) {
        // if there is another pickup in the dispatch left, add the state PICKUP
        if (std::count(myLastDispatch.begin(), myLastDispatch.end(), res) == 2) {
            myState |= PICKUP;  // add state PICKUP
        }
    }
    // we also have to clean reservations from myLastDispatch where the customers arrived in the meantime
    for (auto it = myLastDispatch.begin(); it != myLastDispatch.end();) {
        if (myCurrentReservations.count(*it) == 0) {
            it = myLastDispatch.erase(it);
        } else {
            ++it;
        }
    }
    // if there are reservations left, go on with the dispatch
    // in meso, wait for the next dispatch cycle to avoid updating stops in this stage
    if (!MSGlobals::gUseMesoSim) {
        dispatchShared(myLastDispatch);
    }
    return true;
}


void
MSDevice_Taxi::addCustomer(const MSTransportable* t, const Reservation* res) {
    myCustomers.insert(t);
    int stopIndex = 0;
    MSBaseVehicle& veh = dynamic_cast<MSBaseVehicle&>(myHolder);
    for (const Reservation* res2 : myCurrentReservations) {
        if (res == res2) {
            SUMOVehicleParameter::Stop& stop = const_cast<SUMOVehicleParameter::Stop&>(veh.getStop(stopIndex).pars);
            stop.awaitedPersons.insert(t->getID());
            stop.permitted.insert(t->getID());
            break;
        }
        stopIndex++;
    }
}


void
MSDevice_Taxi::prepareStop(ConstMSEdgeVector& edges,
                           StopParVector& stops,
                           double& lastPos, const MSEdge* stopEdge, double stopPos,
                           const MSStoppingPlace* stopPlace,
                           const std::string& action, const Reservation* res, const bool isPickup) {
    assert(!edges.empty());
    if (stopPlace != nullptr && &stopPlace->getLane().getEdge() == stopEdge) {
        stopPos = stopPlace->getEndLanePosition();
    }
    if (stopPos < lastPos && stopPos + NUMERICAL_EPS >= lastPos) {
        stopPos = lastPos;
    }
    bool addedEdge = false;

    if (stops.empty()) {
        // check brakeGap
        double distToStop = stopPos - lastPos;
        const double brakeGap = myHolder.getBrakeGap();
        if (myHolder.getLane() != nullptr && myHolder.getLane()->isInternal()) {
            distToStop += myHolder.getLane()->getLength();
        }
        if (stopEdge != edges.back()) {
            distToStop += edges.back()->getLength();
            if (distToStop < brakeGap) {
                // the distance between current edge and stop edge may be small
                SUMOAbstractRouter<MSEdge, SUMOVehicle>& router = MSRoutingEngine::getRouterTT(myHolder.getRNGIndex(), myHolder.getVClass());
                ConstMSEdgeVector toFirstStop;
                router.compute(edges.back(), stopEdge, &myHolder, SIMSTEP, toFirstStop, true);
                for (int i = 1; i < (int)toFirstStop.size() - 1; i++) {
                    distToStop += toFirstStop[i]->getLength();
                }
            }
        }
        if (distToStop < brakeGap) {
            // circle back to stopEdge
            //std::cout << SIMTIME << " taxi=" << getID() << " brakeGap=" << brakeGap << " distToStop=" << distToStop << "\n";
            edges.push_back(stopEdge);
            addedEdge = true;
        }
    }

    if (stopEdge == edges.back() && !stops.empty()) {
        if (stopPos >= lastPos && stopPos <= stops.back().endPos) {
            // no new stop and no adaption needed
            stops.back().actType += "," + action;
            return;
        }
        if (stopPos >= lastPos && stopPos <= lastPos + myHolder.getVehicleType().getLength()) {
            // stop length adaption needed
            stops.back().endPos = MIN2(lastPos + myHolder.getVehicleType().getLength(), stopEdge->getLength());
            stops.back().actType += "," + action;
            return;
        }
    }
    if (!addedEdge && (stopEdge != edges.back() || stopPos < lastPos)) {
        //std::cout << SIMTIME << " stopPos=" << stopPos << " lastPos=" << lastPos << "\n";
        edges.push_back(stopEdge);
    }
    lastPos = stopPos;
    SUMOVehicleParameter::Stop stop;
    stop.lane = getStopLane(stopEdge, action)->getID();
    if (stopPlace != nullptr && &stopPlace->getLane().getEdge() == stopEdge) {
        stop.startPos = stopPlace->getBeginLanePosition();
        stop.endPos = stopPlace->getEndLanePosition();
        const SumoXMLTag tag = stopPlace->getElement();
        if (tag == SUMO_TAG_BUS_STOP || tag == SUMO_TAG_TRAIN_STOP) {
            stop.busstop = stopPlace->getID();
        } else if (tag == SUMO_TAG_PARKING_AREA) {
            stop.parkingarea = stopPlace->getID();
        } else if (tag == SUMO_TAG_CONTAINER_STOP) {
            stop.containerstop = stopPlace->getID();
        }
    } else {
        stop.startPos = stopPos;
        stop.endPos = MAX2(stopPos, MIN2(myHolder.getVehicleType().getLength(), stopEdge->getLength()));
    }
    stop.parking = SUMOVehicleParameter::parseParkingType(myHolder.getStringParam("device.taxi.parking", false, "true"));
    stop.actType = action;
    stop.index = STOP_INDEX_END;
    // In case of prebooking if person is not there/ comes to late for pickup set maximum waiting time:
    SUMOTime earliestPickupTime = res->earliestPickupTime;
    if (isPickup && earliestPickupTime >= 0) {
        stop.waitUntil = earliestPickupTime;
        // TODO: replace hard coded extension with parameter
        stop.extension = static_cast<SUMOTime>(3 * 60 * 1000);  // 3mins
    }
    stops.push_back(stop);
}


MSLane*
MSDevice_Taxi::getStopLane(const MSEdge* edge, const std::string& action) {
    const std::vector<MSLane*>* allowedLanes = edge->allowedLanes(myHolder.getVClass());
    if (allowedLanes == nullptr) {
        throw ProcessError("Taxi vehicle '" + myHolder.getID() + "' cannot stop on edge '" + edge->getID() + "' (" + action + ")");
    }
    return allowedLanes->front();
}

bool
MSDevice_Taxi::isEmpty() {
    return myState == EMPTY;
}


bool
MSDevice_Taxi::allowsBoarding(const MSTransportable* t) const {
    return myCustomers.count(t) != 0;
}


void
MSDevice_Taxi::updateMove(const SUMOTime traveltime, const double travelledDist) {
    if (myHolder.getPersonNumber() > 0 || myHolder.getContainerNumber() > 0) {
        myOccupiedDistance += travelledDist;
        myOccupiedTime += traveltime;
    }
    if (isEmpty()) {
        if (MSNet::getInstance()->getCurrentTimeStep() < myServiceEnd) {
            myIdleAlgorithm->idle(this);
            if (myRoutingDevice != nullptr) {
                // prevent rerouting during idling (#11079)
                myRoutingDevice->setActive(false);
            }
        } else if (!myReachedServiceEnd) {
            WRITE_WARNINGF(TL("Taxi '%' reaches scheduled end of service at time=%."), myHolder.getID(), time2string(SIMSTEP));
            myReachedServiceEnd = true;
        }
    } else if (myRoutingDevice != nullptr) {
        myRoutingDevice->setActive(true);
    }
    if (myHolder.isStopped() && (isEmpty() || MSGlobals::gUseMesoSim) && myHolder.getNextStop().endBoarding > myServiceEnd) {
        // limit duration of stop (but only for idling-related stops)
        myHolder.getNextStopMutable().endBoarding = myServiceEnd;
    }
#ifdef DEBUG_DISPATCH
    if (DEBUG_COND && myIsStopped != myHolder.isStopped()) {
        std::cout << SIMTIME << " updateMove veh=" << myHolder.getID() << " myIsStopped=" << myIsStopped << " myHolderStopped=" << myHolder.isStopped() << " myState=" << myState << "\n";
    }
#endif
    myIsStopped = myHolder.isStopped();
}


bool
MSDevice_Taxi::notifyMove(SUMOTrafficObject& /*tObject*/, double oldPos,
                          double newPos, double /*newSpeed*/) {
    updateMove(DELTA_T, newPos - oldPos);
    return true; // keep the device
}


void
MSDevice_Taxi::notifyMoveInternal(const SUMOTrafficObject& /* veh */,
                                  const double /* frontOnLane */,
                                  const double timeOnLane,
                                  const double /* meanSpeedFrontOnLane */,
                                  const double /* meanSpeedVehicleOnLane */,
                                  const double travelledDistanceFrontOnLane,
                                  const double /* travelledDistanceVehicleOnLane */,
                                  const double /* meanLengthOnLane */) {
    updateMove(TIME2STEPS(timeOnLane), travelledDistanceFrontOnLane);
}


bool
MSDevice_Taxi::notifyEnter(SUMOTrafficObject& /*veh*/, MSMoveReminder::Notification /*reason*/, const MSLane* /* enteredLane */) {
    if (isEmpty() && MSNet::getInstance()->getCurrentTimeStep() < myServiceEnd) {
        myIdleAlgorithm->idle(this);
    }
    return true; // keep the device
}


void
MSDevice_Taxi::customerEntered(const MSTransportable* t) {
    myState |= OCCUPIED;
    if (!hasFuturePickup()) {
        myState &= ~PICKUP;
    }
    for (const Reservation* res : myCurrentReservations) {
        for (const MSTransportable* cand : res->persons) {
            if (cand == t) {
                const_cast<Reservation*>(res)->state = Reservation::ONBOARD;
                break;
            }
        }
    }
}


void
MSDevice_Taxi::customerArrived(const MSTransportable* person) {
    myCustomersServed++;
    myCustomers.erase(person);
    if (myHolder.getPersonNumber() == 0 && myHolder.getContainerNumber() == 0) {
        myState &= ~OCCUPIED;
        if (myHolder.getStops().size() > 1 && (myState & PICKUP) == 0) {
            WRITE_WARNINGF(TL("All customers left vehicle '%' at time=% but there are % remaining stops"),
                           myHolder.getID(), time2string(SIMSTEP), myHolder.getStops().size() - 1);
            while (myHolder.getStops().size() > 1) {
                myHolder.abortNextStop(1);
            }
        }
    }
    if (isEmpty()) {
        // cleanup
        for (const Reservation* res : myCurrentReservations) {
            myDispatcher->fulfilledReservation(res);
        }
        myCurrentReservations.clear();
        checkTaskSwap();
        if (isEmpty()) {
            if (MSGlobals::gUseMesoSim && MSNet::getInstance()->getCurrentTimeStep() < myServiceEnd) {
                myIdleAlgorithm->idle(this);
            }
        }
    } else {
        // check whether a single reservation has been fulfilled
        for (auto resIt = myCurrentReservations.begin(); resIt != myCurrentReservations.end();) {
            bool fulfilled = true;
            for (const MSTransportable* t : (*resIt)->persons) {
                if (myCustomers.count(t) != 0) {
                    fulfilled = false;
                    break;
                }
            }
            if (fulfilled) {
                myDispatcher->fulfilledReservation(*resIt);
                resIt = myCurrentReservations.erase(resIt);
            } else {
                ++resIt;
            }
        }
    }
}


void
MSDevice_Taxi::checkTaskSwap() {
    const std::string swapGroup = myHolder.getStringParam("device.taxi.swapGroup", false, "");
    if (swapGroup != "") {
        SUMOAbstractRouter<MSEdge, SUMOVehicle>& router = myDispatcher->getRouter();
        const double stopTime = myHolder.isStopped() ? MAX2(0.0, STEPS2TIME(myHolder.getNextStop().duration)) : 0;
        double maxSaving = 0;
        MSDevice_Taxi* bestSwap = nullptr;
        for (MSDevice_Taxi* taxi : myFleet) {
            if (taxi->getHolder().hasDeparted() && taxi->getState() == PICKUP
                    && taxi->getHolder().getStringParam("device.taxi.swapGroup", false, "") == swapGroup) {
                SUMOVehicle& veh = taxi->getHolder();
                const MSStop& stop = veh.getNextStop();
                ConstMSEdgeVector toPickup(veh.getCurrentRouteEdge(), stop.edge + 1);
                const double cost = router.recomputeCostsPos(toPickup, &veh, veh.getPositionOnLane(), stop.pars.endPos, SIMSTEP);
                ConstMSEdgeVector toPickup2;
                router.compute(myHolder.getEdge(), myHolder.getPositionOnLane(), *stop.edge, stop.pars.endPos, &myHolder, SIMSTEP, toPickup2, true);
                if (!toPickup2.empty()) {
                    const double cost2 = router.recomputeCostsPos(toPickup2, &myHolder, myHolder.getPositionOnLane(), stop.pars.endPos, SIMSTEP);
                    const double saving = cost - cost2 - stopTime;
                    //std::cout << SIMTIME << " taxi=" << getID() << " other=" << veh.getID() << " cost=" << cost << " cost2=" << cost2 << " stopTime=" << stopTime << " saving=" << saving << " route1=" << toString(toPickup) << " route2=" << toString(toPickup2) << "\n";
                    if (saving > maxSaving) {
                        maxSaving = saving;
                        bestSwap = taxi;
                    }
                }
            }
        }
        if (maxSaving > SWAP_THRESHOLD) {
            dispatchShared(bestSwap->myLastDispatch);
            bestSwap->myCurrentReservations.clear();
            bestSwap->myCustomers.clear();
            bestSwap->myState = EMPTY;
            while (bestSwap->getHolder().hasStops()) {
                bestSwap->getHolder().abortNextStop();
            }
            for (const Reservation* res : myCurrentReservations) {
                myDispatcher->swappedRunning(res, this);
            }
        }
    }
}


bool
MSDevice_Taxi::hasFuturePickup() {
    for (const auto& stop : myHolder.getStops()) {
        if (stop.reached) {
            continue;
        }
        if (stop.pars.permitted.size() > 0) {
            return true;
        }
    }
    return false;
}

void
MSDevice_Taxi::generateOutput(OutputDevice* tripinfoOut) const {
    if (tripinfoOut != nullptr) {
        tripinfoOut->openTag("taxi");
        tripinfoOut->writeAttr("customers", toString(myCustomersServed));
        tripinfoOut->writeAttr("occupiedDistance", toString(myOccupiedDistance));
        tripinfoOut->writeAttr("occupiedTime", time2string(myOccupiedTime));
        tripinfoOut->closeTag();
    }
}

std::string
MSDevice_Taxi::getParameter(const std::string& key) const {
    if (key == "customers") {
        return toString(myCustomersServed);
    } else if (key == "occupiedDistance") {
        return toString(myOccupiedDistance);
    } else if (key == "occupiedTime") {
        return toString(STEPS2TIME(myOccupiedTime));
    } else if (key == "state") {
        return toString(myState);
    } else if (key == "currentCustomers") {
        return joinNamedToStringSorting(myCustomers, " ");
    } else if (key == "pickUpDuration") {
        return myHolder.getStringParam("device.taxi.pickUpDuration", false, "0");
    } else if (key == "dropOffDuration") {
        return myHolder.getStringParam("device.taxi.dropOffDuration", false, "60");
    }
    throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
}


void
MSDevice_Taxi::setParameter(const std::string& key, const std::string& value) {
    double doubleValue;
    try {
        doubleValue = StringUtils::toDouble(value);
    } catch (NumberFormatException&) {
        throw InvalidArgument("Setting parameter '" + key + "' requires a number for device of type '" + deviceName() + "'");
    }
    if (key == "pickUpDuration" || key == "dropOffDuration") {
        // store as generic vehicle parameters
        ((SUMOVehicleParameter&)myHolder.getParameter()).setParameter("device.taxi." + key, value);
    } else {
        UNUSED_PARAMETER(doubleValue);
        throw InvalidArgument("Setting parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
    }
}

bool
MSDevice_Taxi::compatibleLine(const std::string& taxiLine, const std::string& rideLine) {
    return ((taxiLine == rideLine && StringUtils::startsWith(rideLine, "taxi") && StringUtils::startsWith(taxiLine, "taxi"))
            || (taxiLine == TAXI_SERVICE && StringUtils::startsWith(rideLine, "taxi:"))
            || (rideLine == TAXI_SERVICE && StringUtils::startsWith(taxiLine, "taxi:")));
}

bool
MSDevice_Taxi::compatibleLine(const Reservation* res) {
    return compatibleLine(myHolder.getParameter().line, res->line);
}


/****************************************************************************/
