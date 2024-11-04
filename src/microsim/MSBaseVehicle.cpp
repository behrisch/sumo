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
/// @file    MSBaseVehicle.cpp
/// @author  Michael Behrisch
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Mirko Barthauer
/// @date    Mon, 8 Nov 2010
///
// A base class for vehicle implementations
/****************************************************************************/
#include <config.h>

#include <iostream>
#include <cassert>
#include <utils/common/StdDefs.h>
#include <utils/common/MsgHandler.h>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/emissions/PollutantsInterface.h>
#include <utils/emissions/HelpersHarmonoise.h>
#include <libsumo/TraCIConstants.h>
#include <mesosim/MELoop.h>
#include <mesosim/MEVehicle.h>
#include <microsim/devices/MSRoutingEngine.h>
#include <microsim/devices/MSDevice_Transportable.h>
#include <microsim/devices/MSDevice_Emissions.h>
#include <microsim/devices/MSDevice_Battery.h>
#include <microsim/devices/MSDevice_ElecHybrid.h>
#include <microsim/devices/MSDevice_Taxi.h>
#include <microsim/devices/MSDevice_Routing.h>
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <microsim/transportables/MSPerson.h>
#include <microsim/transportables/MSStageDriving.h>
#include <microsim/trigger/MSChargingStation.h>
#include <microsim/trigger/MSStoppingPlaceRerouter.h>
#include <microsim/traffic_lights/MSRailSignalConstraint.h>
#include "MSGlobals.h"
#include "MSVehicleControl.h"
#include "MSVehicleType.h"
#include "MSEdge.h"
#include "MSLane.h"
#include "MSMoveReminder.h"
#include "MSEdgeWeightsStorage.h"
#include "MSNet.h"
#include "MSStop.h"
#include "MSParkingArea.h"
#include "MSInsertionControl.h"
#include "MSBaseVehicle.h"

//#define DEBUG_REROUTE
//#define DEBUG_ADD_STOP
//#define DEBUG_COND (getID() == "")
//#define DEBUG_COND (true)
//#define DEBUG_REPLACE_ROUTE
#define DEBUG_COND (isSelected())

// ===========================================================================
// static members
// ===========================================================================
const SUMOTime MSBaseVehicle::NOT_YET_DEPARTED = SUMOTime_MAX;
std::vector<MSTransportable*> MSBaseVehicle::myEmptyTransportableVector;
#ifdef _DEBUG
std::set<std::string> MSBaseVehicle::myShallTraceMoveReminders;
#endif
SUMOTrafficObject::NumericalID MSBaseVehicle::myCurrentNumericalIndex = 0;

// ===========================================================================
// Influencer method definitions
// ===========================================================================

MSBaseVehicle::BaseInfluencer::BaseInfluencer()
{}

// ===========================================================================
// method definitions
// ===========================================================================

double
MSBaseVehicle::getPreviousSpeed() const {
    throw ProcessError("getPreviousSpeed() is not available for non-MSVehicles.");
}


MSBaseVehicle::MSBaseVehicle(SUMOVehicleParameter* pars, ConstMSRoutePtr route,
                             MSVehicleType* type, const double speedFactor) :
    SUMOVehicle(pars->id),
    myParameter(pars),
    myRoute(route),
    myType(type),
    myCurrEdge(route->begin()),
    myChosenSpeedFactor(pars->speedFactor < 0 ? speedFactor : pars->speedFactor),
    myMoveReminders(0),
    myPersonDevice(nullptr),
    myContainerDevice(nullptr),
    myEnergyParams(nullptr),
    myDeparture(NOT_YET_DEPARTED),
    myDepartPos(-1),
    myArrivalPos(-1),
    myArrivalLane(-1),
    myNumberReroutes(0),
    myStopUntilOffset(0),
    myOdometer(0.),
    myRouteValidity(ROUTE_UNCHECKED),
    myRoutingMode(libsumo::ROUTING_MODE_DEFAULT),
    myNumericalID(myCurrentNumericalIndex++),
    myEdgeWeights(nullptr)
#ifdef _DEBUG
    , myTraceMoveReminders(myShallTraceMoveReminders.count(pars->id) > 0)
#endif
{
    if ((*myRoute->begin())->isTazConnector() || myRoute->getLastEdge()->isTazConnector()) {
        pars->parametersSet |= VEHPARS_FORCE_REROUTE;
    }
    if ((pars->parametersSet & VEHPARS_FORCE_REROUTE) == 0) {
        setDepartAndArrivalEdge();
    }
    if (!pars->wasSet(VEHPARS_FORCE_REROUTE)) {
        calculateArrivalParams(true);
    }
    initTransientModelParams();
}


MSBaseVehicle::~MSBaseVehicle() {
    delete myEdgeWeights;
    if (myParameter->repetitionNumber == -1) {
        // this is not a flow (flows call checkDist in MSInsertionControl::determineCandidates)
        MSRoute::checkDist(myParameter->routeid);
    }
    for (MSVehicleDevice* dev : myDevices) {
        delete dev;
    }
    delete myEnergyParams;
    delete myParkingMemory;
    delete myChargingMemory;
    checkRouteRemoval();
    delete myParameter;
}


void
MSBaseVehicle::checkRouteRemoval() {
    // the check for an instance is needed for the unit tests which do not construct a network
    // TODO Optimize for speed and there should be a better way to check whether a vehicle is part of a flow
    if (MSNet::hasInstance() && !MSNet::getInstance()->hasFlow(getFlowID())) {
        myRoute->checkRemoval();
    }
}


std::string
MSBaseVehicle::getFlowID() const {
    return getID().substr(0, getID().rfind('.'));
}


void
MSBaseVehicle::initDevices() {
    MSDevice::buildVehicleDevices(*this, myDevices);
    for (MSVehicleDevice* dev : myDevices) {
        myMoveReminders.push_back(std::make_pair(dev, 0.));
    }
    if (MSGlobals::gHaveEmissions) {
        // ensure we have the emission parameters even if we don't have the device
        getEmissionParameters();
    }
}


void
MSBaseVehicle::setID(const std::string& /*newID*/) {
    throw ProcessError(TL("Changing a vehicle ID is not permitted"));
}

const SUMOVehicleParameter&
MSBaseVehicle::getParameter() const {
    return *myParameter;
}


void
MSBaseVehicle::replaceParameter(const SUMOVehicleParameter* newParameter) {
    delete myParameter;
    myParameter = newParameter;
}


bool
MSBaseVehicle::ignoreTransientPermissions() const {
    return (getRoutingMode() & libsumo::ROUTING_MODE_IGNORE_TRANSIENT_PERMISSIONS) != 0;
}

double
MSBaseVehicle::getMaxSpeed() const {
    return MIN2(myType->getMaxSpeed(), myType->getDesiredMaxSpeed() * myChosenSpeedFactor);
}


const MSEdge*
MSBaseVehicle::succEdge(int nSuccs) const {
    if (myCurrEdge + nSuccs < myRoute->end() && std::distance(myCurrEdge, myRoute->begin()) <= nSuccs) {
        return *(myCurrEdge + nSuccs);
    } else {
        return nullptr;
    }
}


const MSEdge*
MSBaseVehicle::getEdge() const {
    return *myCurrEdge;
}


const std::set<SUMOTrafficObject::NumericalID>
MSBaseVehicle::getUpcomingEdgeIDs() const {
    std::set<SUMOTrafficObject::NumericalID> result;
    for (auto e = myCurrEdge; e != myRoute->end(); ++e) {
        result.insert((*e)->getNumericalID());
    }
    return result;
}


bool
MSBaseVehicle::stopsAt(MSStoppingPlace* stop) const {
    if (stop == nullptr) {
        return false;
    }
    for (const MSStop& s : myStops) {
        if (s.busstop == stop
                || s.containerstop == stop
                || s.parkingarea == stop
                || s.chargingStation == stop) {
            return true;
        }
    }
    return false;
}

bool
MSBaseVehicle::stopsAtEdge(const MSEdge* edge) const {
    for (const MSStop& s : myStops) {
        if (&s.lane->getEdge() == edge) {
            return true;
        }
    }
    return myRoute->getLastEdge() == edge;
}


bool
MSBaseVehicle::reroute(SUMOTime t, const std::string& info, SUMOAbstractRouter<MSEdge, SUMOVehicle>& router, const bool onInit, const bool withTaz, const bool silent, const MSEdge* sink) {
    // check whether to reroute
    const MSEdge* source = withTaz && onInit ? MSEdge::dictionary(myParameter->fromTaz + "-source") : *getRerouteOrigin();
    if (source == nullptr) {
        source = *getRerouteOrigin();
    }
    if (sink == nullptr) {
        sink = withTaz ? MSEdge::dictionary(myParameter->toTaz + "-sink") : myRoute->getLastEdge();
        if (sink == nullptr) {
            sink = myRoute->getLastEdge();
        }
    }
    ConstMSEdgeVector oldEdgesRemaining(source == *myCurrEdge ? myCurrEdge : myCurrEdge + 1, myRoute->end());
    ConstMSEdgeVector edges;
    ConstMSEdgeVector stops;
    std::set<int> jumps;
    bool stopAtSink = false;
    if (myParameter->via.size() == 0) {
        double firstPos = INVALID_DOUBLE;
        double lastPos = INVALID_DOUBLE;
        stops = getStopEdges(firstPos, lastPos, jumps);
        if (stops.size() > 0) {
            double sourcePos = onInit ? 0 : getPositionOnLane();
            if (MSGlobals::gUseMesoSim && isStopped()) {
                sourcePos = getNextStop().pars.endPos;
            }
            // avoid superfluous waypoints for first and last edge
            const bool skipFirst = stops.front() == source && (source != getEdge() || sourcePos + getBrakeGap() <= firstPos + NUMERICAL_EPS);
            const bool skipLast = (stops.back() == sink
                                   && myArrivalPos >= lastPos
                                   && (stops.size() < 2 || stops.back() != stops[stops.size() - 2])
                                   && (stops.size() > 1 || skipFirst));
#ifdef DEBUG_REROUTE
            if (DEBUG_COND) {
                std::cout << SIMTIME << " reroute " << info << " veh=" << getID() << " lane=" << Named::getIDSecure(getLane())
                          << " source=" << source->getID() << " sourcePos=" << sourcePos << " firstPos=" << firstPos << " arrivalPos=" << myArrivalPos << " lastPos=" << lastPos
                          << " route=" << toString(myRoute->getEdges()) << " stopEdges=" << toString(stops) << " skipFirst=" << skipFirst << " skipLast=" << skipLast << "\n";
            }
#endif
            if (stops.size() == 1 && (skipFirst || skipLast)) {
                stops.clear();
            } else {
                if (skipFirst) {
                    stops.erase(stops.begin());
                }
                if (skipLast) {
                    stops.erase(stops.end() - 1);
                }
            }
            stopAtSink = stops.size() > 0 && stops.back() == sink && jumps.size() == 0;
        }
    } else {
        std::set<const MSEdge*> jumpEdges;
        for (const MSStop& stop : myStops) {
            if (stop.pars.jump >= 0) {
                jumpEdges.insert(*stop.edge);
            }
        }
        // via takes precedence over stop edges
        // there is a consistency check in MSRouteHandler::addStop that warns when a stop edge is not part of the via edges
        for (std::vector<std::string>::const_iterator it = myParameter->via.begin(); it != myParameter->via.end(); ++it) {
            MSEdge* viaEdge = MSEdge::dictionary(*it);
            if ((viaEdge == source && it == myParameter->via.begin()) || (viaEdge == sink && myParameter->via.end() - it == 1)) {
                continue;
            }
            assert(viaEdge != 0);
            if (!viaEdge->isTazConnector() && viaEdge->allowedLanes(getVClass()) == nullptr) {
                throw ProcessError(TLF("Vehicle '%' is not allowed on any lane of via edge '%'.", getID(), viaEdge->getID()));
            }
            stops.push_back(viaEdge);
            if (jumpEdges.count(viaEdge) != 0) {
                jumps.insert((int)stops.size());
            }
        }
    }

    int stopIndex = -1;
    for (const MSEdge* const stopEdge : stops) {
        stopIndex++;
        // !!! need to adapt t here
        ConstMSEdgeVector into;
        if (jumps.count(stopIndex) != 0) {
            edges.push_back(source);
            source = stopEdge;
            continue;
        }
        router.computeLooped(source, stopEdge, this, t, into, silent);
        //std::cout << SIMTIME << " reroute veh=" << getID() << " source=" << source->getID() << " target=" << (*s)->getID() << " edges=" << toString(into) << "\n";
        if (into.size() > 0) {
            into.pop_back();
            edges.insert(edges.end(), into.begin(), into.end());
            if (stopEdge->isTazConnector()) {
                source = into.back();
                edges.pop_back();
            } else {
                source = stopEdge;
            }
        } else {
            if ((source != sink || !stopAtSink)) {
                std::string error = TLF("Vehicle '%' has no valid route from edge '%' to stop edge '%'.", getID(), source->getID(), stopEdge->getID());
                if (MSGlobals::gCheckRoutes || silent) {
                    throw ProcessError(error);
                } else {
                    WRITE_WARNING(error);
                    edges.push_back(source);
                }
            }
            source = stopEdge;
        }
    }
    if (stops.empty() && source == sink && onInit
            && myParameter->departPosProcedure == DepartPosDefinition::GIVEN
            && myParameter->arrivalPosProcedure == ArrivalPosDefinition::GIVEN
            && myParameter->departPos > myParameter->arrivalPos) {
        router.computeLooped(source, sink, this, t, edges, silent);
    } else {
        if (!router.compute(source, sink, this, t, edges, silent)) {
            edges.clear();
        }
    }

    // router.setHint(myCurrEdge, myRoute->end(), this, t);
    if (edges.empty() && silent) {
        return false;
    }
    if (!edges.empty() && edges.front()->isTazConnector()) {
        edges.erase(edges.begin());
    }
    if (!edges.empty() && edges.back()->isTazConnector()) {
        edges.pop_back();
    }
    const double routeCost = router.recomputeCosts(edges, this, t);
    const double previousCost = onInit ? routeCost : router.recomputeCosts(oldEdgesRemaining, this, t);
    const double savings = previousCost - routeCost;
    //if (getID() == "43") std::cout << SIMTIME << " pCost=" << previousCost << " cost=" << routeCost
    //    << " onInit=" << onInit
    //        << " prevEdges=" << toString(oldEdgesRemaining)
    //        << " newEdges=" << toString(edges)
    //        << "\n";
    replaceRouteEdges(edges, routeCost, savings, info, onInit);
    // this must be called even if the route could not be replaced
    if (onInit) {
        if (edges.empty()) {
            if (MSGlobals::gCheckRoutes) {
                throw ProcessError(TLF("Vehicle '%' has no valid route.", getID()));
            } else if (source->isTazConnector()) {
                WRITE_WARNINGF(TL("Removing vehicle '%' which has no valid route."), getID());
                MSNet::getInstance()->getInsertionControl().descheduleDeparture(this);
                return false;
            }
        }
        setDepartAndArrivalEdge();
        calculateArrivalParams(onInit);
    }
    return !edges.empty();
}


bool
MSBaseVehicle::replaceRouteEdges(ConstMSEdgeVector& edges, double cost, double savings, const std::string& info, bool onInit, bool check, bool removeStops, std::string* msgReturn) {
    if (edges.empty()) {
        WRITE_WARNINGF(TL("No route for vehicle '%' found."), getID());
        if (msgReturn != nullptr) {
            *msgReturn = "No route found";
        }
        return false;
    }
    // build a new id, first
    std::string id = getID();
    if (id[0] != '!') {
        id = "!" + id;
    }
    const std::string idSuffix = id + "!var#";
    int varIndex = 1;
    id = idSuffix + toString(varIndex);
    while (MSRoute::hasRoute(id)) {
        id = idSuffix + toString(++varIndex);
    }
    int oldSize = (int)edges.size();
    if (!onInit) {
        const MSEdge* const origin = *getRerouteOrigin();
        if (origin != *myCurrEdge && edges.front() == origin) {
            edges.insert(edges.begin(), *myCurrEdge);
            oldSize = (int)edges.size();
        }
        edges.insert(edges.begin(), myRoute->begin(), myCurrEdge);
    }
    if (edges == myRoute->getEdges() && haveValidStopEdges(true)) {
        // re-assign stop iterators when rerouting to a new parkingArea / insertStop
        return true;
    }
    const RGBColor& c = myRoute->getColor();
    MSRoute* newRoute = new MSRoute(id, edges, false, &c == &RGBColor::DEFAULT_COLOR ? nullptr : new RGBColor(c), std::vector<SUMOVehicleParameter::Stop>());
    newRoute->setCosts(cost);
    newRoute->setSavings(savings);
    ConstMSRoutePtr constRoute = std::shared_ptr<MSRoute>(newRoute);
    if (!MSRoute::dictionary(id, constRoute)) {
        delete newRoute;
        if (msgReturn != nullptr) {
            *msgReturn = "duplicate routeID '" + id + "'";
        }
        return false;
    }

    std::string msg;
    if (check && !hasValidRoute(msg, constRoute)) {
        WRITE_WARNINGF(TL("Invalid route replacement for vehicle '%'. %"), getID(), msg);
        if (MSGlobals::gCheckRoutes) {
            if (msgReturn != nullptr) {
                *msgReturn = msg;
            }
            return false;
        }
    }
    if (!replaceRoute(constRoute, info, onInit, (int)edges.size() - oldSize, false, removeStops, msgReturn)) {
        return false;
    }
    return true;
}


bool
MSBaseVehicle::replaceRoute(ConstMSRoutePtr newRoute, const std::string& info, bool onInit, int offset, bool addRouteStops, bool removeStops, std::string* msgReturn) {
    const ConstMSEdgeVector& edges = newRoute->getEdges();
    // rebuild in-vehicle route information
    if (onInit) {
        myCurrEdge = newRoute->begin();
    } else {
        MSRouteIterator newCurrEdge = std::find(edges.begin() + offset, edges.end(), *myCurrEdge);
        if (newCurrEdge == edges.end()) {
            if (msgReturn != nullptr) {
                *msgReturn = TLF("current edge '%' not found in new route", (*myCurrEdge)->getID());
            }
#ifdef DEBUG_REPLACE_ROUTE
            if (DEBUG_COND) {
                std::cout << "  newCurrEdge not found\n";
            }
#endif
            return false;
        }
        if (getLane() != nullptr) {
            if (getLane()->getEdge().isInternal() && (
                        (newCurrEdge + 1) == edges.end() || (*(newCurrEdge + 1)) != &(getLane()->getOutgoingViaLanes().front().first->getEdge()))) {
                if (msgReturn != nullptr) {
                    *msgReturn = TL("Vehicle is on junction-internal edge leading elsewhere");
                }
#ifdef DEBUG_REPLACE_ROUTE
                if (DEBUG_COND) {
                    std::cout << "  Vehicle is on junction-internal edge leading elsewhere\n";
                }
#endif
                return false;
            } else if (getPositionOnLane() > getLane()->getLength()
                       && (myCurrEdge + 1) != myRoute->end()
                       && (newCurrEdge + 1) != edges.end()
                       && *(myCurrEdge + 1) != *(newCurrEdge + 1)) {
                if (msgReturn != nullptr) {
                    *msgReturn = TL("Vehicle is moving past junction and committed to move to another successor edge");
                }
#ifdef DEBUG_REPLACE_ROUTE
                if (DEBUG_COND) {
                    std::cout << "  Vehicle is moving past junction and committed to move to another successor edge\n";
                }
#endif
                return false;
            }
        }
        myCurrEdge = newCurrEdge;
    }
    const bool stopsFromScratch = onInit && myRoute->getStops().empty();
    // assign new route
    checkRouteRemoval();
    myRoute = newRoute;
    // update arrival definition
    calculateArrivalParams(onInit);
    // save information that the vehicle was rerouted
    myNumberReroutes++;
    myStopUntilOffset += myRoute->getPeriod();
    MSNet::getInstance()->informVehicleStateListener(this, MSNet::VehicleState::NEWROUTE, info);
#ifdef DEBUG_REPLACE_ROUTE
    if (DEBUG_COND) {
        std::cout << SIMTIME << " veh=" << getID() << " replaceRoute info=" << info << " on " << (*myCurrEdge)->getID()
                  << " lane=" << Named::getIDSecure(getLane())
                  << " stopsFromScratch=" << stopsFromScratch
                  << "  newSize=" << newRoute->getEdges().size()
                  << " newIndex=" << (myCurrEdge - newRoute->begin())
                  << " edges=" << toString(newRoute->getEdges())
                  << "\n";
    }
#endif
    // remove past stops which are not on the route anymore
    for (std::vector<SUMOVehicleParameter::Stop>::iterator it = myPastStops.begin(); it != myPastStops.end();) {
        const MSEdge* stopEdge = (it->edge.empty()) ? &MSLane::dictionary(it->lane)->getEdge() : MSEdge::dictionary(it->edge);
        if (std::find(myRoute->begin(), myRoute->end(), stopEdge) == myRoute->end()) {
            it = myPastStops.erase(it);
        } else {
            ++it;
        }
    }
    // if we did not drive yet it may be best to simply reassign the stops from scratch
    if (stopsFromScratch) {
        myStops.clear();
        addStops(!MSGlobals::gCheckRoutes);
    } else {
        // recheck old stops
        MSRouteIterator searchStart = myCurrEdge;
        double lastPos = getPositionOnLane() + getBrakeGap();
        if (getLane() != nullptr && getLane()->isInternal()
                && myStops.size() > 0 && !myStops.front().lane->isInternal()) {
            // searchStart is still incoming to the intersection so lastPos
            // relative to that edge must be adapted
            lastPos += (*myCurrEdge)->getLength();
        }
        int stopIndex = 0;
        for (std::list<MSStop>::iterator iter = myStops.begin(); iter != myStops.end();) {
            double endPos = iter->getEndPos(*this);
#ifdef DEBUG_REPLACE_ROUTE
            if (DEBUG_COND) {
                std::cout << "     stopEdge=" << iter->lane->getEdge().getID() << " start=" << (searchStart - myCurrEdge) << " endPos=" << endPos << " lastPos=" << lastPos << "\n";
            }
#endif
            if (*searchStart != &iter->lane->getEdge()
                    || endPos + NUMERICAL_EPS < lastPos) {
                if (searchStart != edges.end() && !iter->reached) {
                    searchStart++;
                }
            }
            lastPos = endPos;

            iter->edge = std::find(searchStart, edges.end(), &iter->lane->getEdge());
#ifdef DEBUG_REPLACE_ROUTE
            if (DEBUG_COND) {
                std::cout << "        foundIndex=" << (iter->edge - myCurrEdge) << " end=" << (edges.end() - myCurrEdge) << "\n";
            }
#endif
            if (iter->edge == edges.end()) {
                if (!removeStops) {
                    WRITE_ERRORF(TL("Vehicle '%' could not assign stop '%' after rerouting (%) at time=%."), getID(), iter->getDescription(), info, time2string(SIMSTEP));
                }
                iter = myStops.erase(iter);
                continue;
            } else {
                setSkips(*iter, stopIndex);
                searchStart = iter->edge;
            }
            ++iter;
            stopIndex++;
        }
        // add new stops
        if (addRouteStops) {
            for (std::vector<SUMOVehicleParameter::Stop>::const_iterator i = newRoute->getStops().begin(); i != newRoute->getStops().end(); ++i) {
                std::string error;
                addStop(*i, error, myParameter->depart + myStopUntilOffset);
                if (error != "") {
                    WRITE_WARNING(error);
                }
            }
        }
    }
    return true;
}


double
MSBaseVehicle::getAcceleration() const {
    return 0;
}


void
MSBaseVehicle::onDepart() {
    myDeparture = MSNet::getInstance()->getCurrentTimeStep();
    myDepartPos = getPositionOnLane();
    MSNet::getInstance()->getVehicleControl().vehicleDeparted(*this);
}


SUMOTime
MSBaseVehicle:: getDepartDelay() const {
    const SUMOTime dep = getParameter().depart;
    if (dep < 0) {
        return 0;
    }
    return hasDeparted() ? getDeparture() - dep : SIMSTEP - dep;
}


bool
MSBaseVehicle::hasArrived() const {
    return succEdge(1) == nullptr;
}


int
MSBaseVehicle::getRoutePosition() const {
    return (int) std::distance(myRoute->begin(), myCurrEdge);
}


void
MSBaseVehicle::resetRoutePosition(int index, DepartLaneDefinition departLaneProcedure) {
    myCurrEdge = myRoute->begin() + index;
    const_cast<SUMOVehicleParameter*>(myParameter)->departLaneProcedure = departLaneProcedure;
    // !!! hack
    myArrivalPos = (*(myRoute->end() - 1))->getLanes()[0]->getLength();
}

double
MSBaseVehicle::getOdometer() const {
    return -myDepartPos + myOdometer + (hasArrived() ? myArrivalPos : getPositionOnLane());
}

bool
MSBaseVehicle::allowsBoarding(const MSTransportable* t) const {
    if (t->isPerson() && getPersonNumber() >= getVehicleType().getPersonCapacity()) {
        return false;
    } else if (!t->isPerson() && getContainerNumber() >= getVehicleType().getContainerCapacity()) {
        return false;
    }
    if (isStopped() && myStops.begin()->pars.permitted.size() > 0
            && myStops.begin()->pars.permitted.count(t->getID()) == 0) {
        return false;
    }
    MSDevice_Taxi* taxiDevice = static_cast<MSDevice_Taxi*>(getDevice(typeid(MSDevice_Taxi)));
    if (taxiDevice != nullptr) {
        return taxiDevice->allowsBoarding(t);
    }
    return true;
}


void
MSBaseVehicle::addTransportable(MSTransportable* transportable) {
    if (transportable->isPerson()) {
        if (myPersonDevice == nullptr) {
            myPersonDevice = MSDevice_Transportable::buildVehicleDevices(*this, myDevices, false);
            myMoveReminders.insert(myMoveReminders.begin(), std::make_pair(myPersonDevice, 0.));
            if (myParameter->departProcedure == DepartDefinition::TRIGGERED && myParameter->depart == -1) {
                const_cast<SUMOVehicleParameter*>(myParameter)->depart = MSNet::getInstance()->getCurrentTimeStep();
            }
        }
        myPersonDevice->addTransportable(transportable);
    } else {
        if (myContainerDevice == nullptr) {
            myContainerDevice = MSDevice_Transportable::buildVehicleDevices(*this, myDevices, true);
            myMoveReminders.insert(myMoveReminders.begin(), std::make_pair(myContainerDevice, 0.));
            if (myParameter->departProcedure == DepartDefinition::CONTAINER_TRIGGERED && myParameter->depart == -1) {
                const_cast<SUMOVehicleParameter*>(myParameter)->depart = MSNet::getInstance()->getCurrentTimeStep();
            }
        }
        myContainerDevice->addTransportable(transportable);
    }
}


bool
MSBaseVehicle::hasJump(const MSRouteIterator& it) const {
    for (const MSStop& stop : myStops) {
        if (stop.edge == it) {
            return stop.pars.jump >= 0;
        }
    }
    return false;
}


bool
MSBaseVehicle::hasValidRoute(std::string& msg, ConstMSRoutePtr route) const {
    MSRouteIterator start = myCurrEdge;
    if (route == nullptr) {
        route = myRoute;
    } else {
        start = route->begin();
    }
    const bool checkJumps = route == myRoute;  // the edge iterators in the stops are invalid otherwise
    MSRouteIterator last = route->end() - 1;
    // check connectivity, first
    for (MSRouteIterator e = start; e != last; ++e) {
        const MSEdge& next = **(e + 1);
        if ((*e)->allowedLanes(next, myType->getVehicleClass()) == nullptr) {
            if (!checkJumps || !hasJump(e)) {
                if ((myRoutingMode & libsumo::ROUTING_MODE_IGNORE_TRANSIENT_PERMISSIONS) == 0
                        || (!next.hasTransientPermissions() && !(*e)->hasTransientPermissions())) {
                    msg = TLF("No connection between edge '%' and edge '%'.", (*e)->getID(), (*(e + 1))->getID());
                    return false;
                }
            }
        }
    }
    last = route->end();
    // check usable lanes, then
    for (MSRouteIterator e = start; e != last; ++e) {
        if ((*e)->prohibits(this)) {
            msg = TLF("Edge '%' prohibits.", (*e)->getID());
            return false;
        }
    }
    return true;
}


bool
MSBaseVehicle::hasValidRouteStart(std::string& msg) {
    if (!(*myCurrEdge)->isTazConnector()) {
        if (myParameter->departSpeedProcedure == DepartSpeedDefinition::GIVEN && myParameter->departSpeed > myType->getMaxSpeed() + SPEED_EPS) {
            msg = TLF("Departure speed for vehicle '%' is too high for the vehicle type '%'.", getID(), myType->getID());
            myRouteValidity |= ROUTE_START_INVALID_LANE;
            return false;
        }
    }
    if (myRoute->getEdges().size() > 0 && !(*myCurrEdge)->prohibits(this)) {
        myRouteValidity &= ~ROUTE_START_INVALID_PERMISSIONS;
        return true;
    } else {
        msg = TLF("Vehicle '%' is not allowed to depart on any lane of edge '%'.", getID(), (*myCurrEdge)->getID());
        myRouteValidity |= ROUTE_START_INVALID_PERMISSIONS;
        return false;
    }
}


int
MSBaseVehicle::getRouteValidity(bool update, bool silent, std::string* msgReturn) {
    if (!update) {
        return myRouteValidity;
    }
    // insertion check must be done in any case
    std::string msg;
    if (!hasValidRouteStart(msg)) {
        if (MSGlobals::gCheckRoutes) {
            throw ProcessError(msg);
        } else if (!silent) {
            // vehicle will be discarded
            WRITE_WARNING(msg);
        } else if (msgReturn != nullptr) {
            *msgReturn = msg;
        }
    }
    if (MSGlobals::gCheckRoutes
            && (myRouteValidity & ROUTE_UNCHECKED) != 0
            // we could check after the first rerouting
            && (!myParameter->wasSet(VEHPARS_FORCE_REROUTE))) {
        if (!hasValidRoute(msg, myRoute)) {
            myRouteValidity |= ROUTE_INVALID;
            throw ProcessError(TLF("Vehicle '%' has no valid route. %", getID(), msg));
        }
    }
    myRouteValidity &= ~ROUTE_UNCHECKED;
    return myRouteValidity;
}

void
MSBaseVehicle::addReminder(MSMoveReminder* rem) {
#ifdef _DEBUG
    if (myTraceMoveReminders) {
        traceMoveReminder("add", rem, 0, true);
    }
#endif
    myMoveReminders.push_back(std::make_pair(rem, 0.));
}


void
MSBaseVehicle::removeReminder(MSMoveReminder* rem) {
    for (MoveReminderCont::iterator r = myMoveReminders.begin(); r != myMoveReminders.end(); ++r) {
        if (r->first == rem) {
#ifdef _DEBUG
            if (myTraceMoveReminders) {
                traceMoveReminder("remove", rem, 0, false);
            }
#endif
            myMoveReminders.erase(r);
            return;
        }
    }
}


void
MSBaseVehicle::activateReminders(const MSMoveReminder::Notification reason, const MSLane* enteredLane) {
    for (MoveReminderCont::iterator rem = myMoveReminders.begin(); rem != myMoveReminders.end();) {
        // skip the reminder if it is a lane reminder but not for my lane (indicated by rem->second > 0.)
        if (rem->first->getLane() != nullptr && rem->second > 0.) {
#ifdef _DEBUG
            if (myTraceMoveReminders) {
                traceMoveReminder("notifyEnter_skipped", rem->first, rem->second, true);
            }
#endif
            ++rem;
        } else {
            if (rem->first->notifyEnter(*this, reason, enteredLane)) {
#ifdef _DEBUG
                if (myTraceMoveReminders) {
                    traceMoveReminder("notifyEnter", rem->first, rem->second, true);
                }
#endif
                ++rem;
            } else {
#ifdef _DEBUG
                if (myTraceMoveReminders) {
                    traceMoveReminder("notifyEnter", rem->first, rem->second, false);
                }
#endif
                rem = myMoveReminders.erase(rem);
            }
        }
    }
}


void
MSBaseVehicle::calculateArrivalParams(bool onInit) {
    if (myRoute->getLastEdge()->isTazConnector()) {
        return;
    }
    const int arrivalEdgeIndex = MIN2(myParameter->arrivalEdge, (int)myRoute->getEdges().size() - 1);
    if (arrivalEdgeIndex != myParameter->arrivalEdge) {
        WRITE_WARNINGF(TL("Vehicle '%' ignores attribute arrivalEdge=% after rerouting at time=% (routeLength=%)"),
                       getID(), myParameter->arrivalEdge, time2string(SIMSTEP), myRoute->getEdges().size() - 1);
    }
    const MSEdge* arrivalEdge = myParameter->arrivalEdge >= 0 ? myRoute->getEdges()[arrivalEdgeIndex] : myRoute->getLastEdge();
    if (!onInit) {
        arrivalEdge = myRoute->getLastEdge();
        // ignore arrivalEdge parameter after rerouting
        const_cast<SUMOVehicleParameter*>(myParameter)->arrivalEdge = -1;
    }
    const std::vector<MSLane*>& lanes = arrivalEdge->getLanes();
    const double lastLaneLength = lanes[0]->getLength();
    switch (myParameter->arrivalPosProcedure) {
        case ArrivalPosDefinition::GIVEN:
            if (fabs(myParameter->arrivalPos) > lastLaneLength) {
                WRITE_WARNINGF(TL("Vehicle '%' will not be able to arrive at the given position!"), getID());
            }
            // Maybe we should warn the user about invalid inputs!
            myArrivalPos = MIN2(myParameter->arrivalPos, lastLaneLength);
            if (myArrivalPos < 0) {
                myArrivalPos = MAX2(myArrivalPos + lastLaneLength, 0.);
            }
            break;
        case ArrivalPosDefinition::RANDOM:
            myArrivalPos = RandHelper::rand(lastLaneLength);
            break;
        case ArrivalPosDefinition::CENTER:
            myArrivalPos = lastLaneLength / 2.;
            break;
        default:
            myArrivalPos = lastLaneLength;
            break;
    }
    if (myParameter->arrivalLaneProcedure == ArrivalLaneDefinition::GIVEN) {
        if (myParameter->arrivalLane >= (int)lanes.size() || !lanes[myParameter->arrivalLane]->allowsVehicleClass(myType->getVehicleClass())) {
            WRITE_WARNINGF(TL("Vehicle '%' will not be able to arrive at the given lane '%_%'!"), getID(), arrivalEdge->getID(), toString(myParameter->arrivalLane));
        }
        myArrivalLane = MIN2(myParameter->arrivalLane, (int)(lanes.size() - 1));
    } else if (myParameter->arrivalLaneProcedure == ArrivalLaneDefinition::FIRST_ALLOWED) {
        myArrivalLane = -1;
        for (MSLane* lane : lanes) {
            if (lane->allowsVehicleClass(myType->getVehicleClass())) {
                myArrivalLane = lane->getIndex();
                break;
            }
        }
        if (myArrivalLane == -1) {
            WRITE_WARNINGF(TL("Vehicle '%' has no usable arrivalLane on edge '%'."), getID(), arrivalEdge->getID());
            myArrivalLane = 0;
        }
    } else if (myParameter->arrivalLaneProcedure == ArrivalLaneDefinition::RANDOM) {
        // pick random lane among all usable lanes
        std::vector<MSLane*> usable;
        for (MSLane* lane : lanes) {
            if (lane->allowsVehicleClass(myType->getVehicleClass())) {
                usable.push_back(lane);
            }
        }
        if (usable.empty()) {
            WRITE_WARNINGF(TL("Vehicle '%' has no usable arrivalLane on edge '%'."), getID(), arrivalEdge->getID());
            myArrivalLane = 0;
        } else {
            myArrivalLane = usable[RandHelper::rand(0, (int)usable.size())]->getIndex();
        }
    }
    if (myParameter->arrivalSpeedProcedure == ArrivalSpeedDefinition::GIVEN) {
        for (std::vector<MSLane*>::const_iterator l = lanes.begin(); l != lanes.end(); ++l) {
            if (myParameter->arrivalSpeed <= (*l)->getVehicleMaxSpeed(this)) {
                return;
            }
        }
        WRITE_WARNINGF(TL("Vehicle '%' will not be able to arrive with the given speed!"), getID());
    }
}

void
MSBaseVehicle::setDepartAndArrivalEdge() {
    SUMOVehicleParameter* pars = const_cast<SUMOVehicleParameter*>(myParameter);
    if (pars->departEdgeProcedure != RouteIndexDefinition::DEFAULT) {
        const int routeEdges = (int)myRoute->getEdges().size();
        if (pars->departEdgeProcedure == RouteIndexDefinition::RANDOM) {
            // write specific edge in vehroute output for reproducibility
            pars->departEdge = RandHelper::rand(0, routeEdges);
            pars->departEdgeProcedure = RouteIndexDefinition::GIVEN;
        }
        assert(pars->departEdge >= 0);
        if (pars->departEdge >= routeEdges) {
            WRITE_WARNINGF(TL("Ignoring departEdge % for vehicle '%' with % route edges"), toString(pars->departEdge), getID(), toString(routeEdges));
        } else {
            myCurrEdge += pars->departEdge;
        }
    }
    if (pars->arrivalEdgeProcedure == RouteIndexDefinition::RANDOM) {
        const int routeEdges = (int)myRoute->getEdges().size();
        const int begin = (int)(myCurrEdge - myRoute->begin());
        // write specific edge in vehroute output for reproducibility
        pars->arrivalEdge = RandHelper::rand(begin, routeEdges);
        pars->arrivalEdgeProcedure = RouteIndexDefinition::GIVEN;
        assert(pars->arrivalEdge >= begin);
        assert(pars->arrivalEdge < routeEdges);
    }
}


double
MSBaseVehicle::getImpatience() const {
    return MAX2(0., MIN2(1., getVehicleType().getImpatience()
                         + (hasInfluencer() ? getBaseInfluencer()->getExtraImpatience() : 0)
                         + (MSGlobals::gTimeToImpatience > 0 ? (double)getWaitingTime() / (double)MSGlobals::gTimeToImpatience : 0.)));
}


MSDevice*
MSBaseVehicle::getDevice(const std::type_info& type) const {
    for (MSVehicleDevice* const dev : myDevices) {
        if (typeid(*dev) == type) {
            return dev;
        }
    }
    return nullptr;
}


void
MSBaseVehicle::saveState(OutputDevice& out) {
    // the parameters may hold the name of a vTypeDistribution but we are interested in the actual type
    const std::string& typeID = MSNet::getInstance()->getVehicleControl().hasVTypeDistribution(myParameter->vtypeid) || getVehicleType().isVehicleSpecific() ? getVehicleType().getID() : "";
    myParameter->write(out, OptionsCont::getOptions(), SUMO_TAG_VEHICLE, typeID);
    // params and stops must be written in child classes since they may wish to add additional attributes first
    out.writeAttr(SUMO_ATTR_ROUTE, myRoute->getID());
    std::ostringstream os;
    os << myOdometer << " " << myNumberReroutes;
    out.writeAttr(SUMO_ATTR_DISTANCE, os.str());
    if (!myParameter->wasSet(VEHPARS_SPEEDFACTOR_SET)) {
        const int precision = out.precision();
        out.setPrecision(MAX2(gPrecisionRandom, precision));
        out.writeAttr(SUMO_ATTR_SPEEDFACTOR, myChosenSpeedFactor);
        out.setPrecision(precision);
    }
    if (myParameter->wasSet(VEHPARS_FORCE_REROUTE)) {
        out.writeAttr(SUMO_ATTR_REROUTE, true);
    }
    if (!myParameter->wasSet(VEHPARS_LINE_SET) && myParameter->line != "") {
        // could be set from stop
        out.writeAttr(SUMO_ATTR_LINE, myParameter->line);
    }
    // here starts the vehicle internal part (see loading)
    // @note: remember to close the vehicle tag when calling this in a subclass!
}


bool
MSBaseVehicle::handleCollisionStop(MSStop& stop, const double distToStop) {
    UNUSED_PARAMETER(stop);
    UNUSED_PARAMETER(distToStop);
    return true;
}


bool
MSBaseVehicle::isStopped() const {
    return !myStops.empty() && myStops.begin()->reached /*&& myState.mySpeed < SUMO_const_haltingSpeed @todo #1864#*/;
}


bool
MSBaseVehicle::isParking() const {
    return (isStopped() && (myStops.begin()->pars.parking == ParkingType::OFFROAD)
            && (myStops.begin()->parkingarea == nullptr || !myStops.begin()->parkingarea->parkOnRoad())
            && (myStops.begin()->getSpeed() == 0 || getSpeed() < SUMO_const_haltingSpeed));
}


bool
MSBaseVehicle::isJumping() const {
    return myPastStops.size() > 0 && myPastStops.back().jump >= 0 && getEdge()->getID() == myPastStops.back().edge && myPastStops.back().ended == SIMSTEP;
}


bool
MSBaseVehicle::isStoppedTriggered() const {
    return isStopped() && (myStops.begin()->triggered || myStops.begin()->containerTriggered || myStops.begin()->joinTriggered);
}


bool
MSBaseVehicle::isStoppedParking() const {
    return isStopped() && (myStops.begin()->pars.parking == ParkingType::OFFROAD);
}


bool
MSBaseVehicle::isStoppedInRange(const double pos, const double tolerance, bool checkFuture) const {
    if (isStopped() || (checkFuture && hasStops())) {
        const MSStop& stop = myStops.front();
        return stop.pars.startPos - tolerance <= pos && stop.pars.endPos + tolerance >= pos;
    }
    return false;
}

bool
MSBaseVehicle::replaceParkingArea(MSParkingArea* parkingArea, std::string& errorMsg) {
    // Check if there is a parking area to be replaced
    if (parkingArea == 0) {
        errorMsg = "new parkingArea is NULL";
        return false;
    }
    if (myStops.size() == 0) {
        errorMsg = "vehicle has no stops";
        return false;
    }
    if (myStops.front().parkingarea == 0) {
        errorMsg = "first stop is not at parkingArea";
        return false;
    }
    MSStop& first = myStops.front();
    SUMOVehicleParameter::Stop& stopPar = const_cast<SUMOVehicleParameter::Stop&>(first.pars);
    std::string oldStopEdgeID = first.lane->getEdge().getID();
    // merge subsequent duplicate stops equals to parking area
    for (std::list<MSStop>::iterator iter = ++myStops.begin(); iter != myStops.end();) {
        if (iter->parkingarea == parkingArea) {
            stopPar.duration += iter->duration;
            myStops.erase(iter++);
        } else {
            break;
        }
    }
    stopPar.lane = parkingArea->getLane().getID();
    stopPar.parkingarea = parkingArea->getID();
    stopPar.startPos = parkingArea->getBeginLanePosition();
    stopPar.endPos = parkingArea->getEndLanePosition();
    first.edge = myRoute->end(); // will be patched in replaceRoute
    first.lane = &parkingArea->getLane();
    first.parkingarea = parkingArea;

    // patch via edges
    std::string newStopEdgeID = parkingArea->getLane().getEdge().getID();
    if (myParameter->via.size() > 0 && myParameter->via.front() != newStopEdgeID) {
        myParameter->via.erase(myParameter->via.begin());
        myParameter->via.insert(myParameter->via.begin(), newStopEdgeID);
    }
    return true;
}


MSParkingArea*
MSBaseVehicle::getNextParkingArea() {
    MSParkingArea* nextParkingArea = nullptr;
    if (!myStops.empty()) {
        SUMOVehicleParameter::Stop stopPar;
        MSStop stop = myStops.front();
        if (!stop.reached && stop.parkingarea != nullptr) {
            nextParkingArea = stop.parkingarea;
        }
    }
    return nextParkingArea;
}


MSParkingArea*
MSBaseVehicle::getCurrentParkingArea() {
    MSParkingArea* currentParkingArea = nullptr;
    if (isParking()) {
        currentParkingArea = myStops.begin()->parkingarea;
    }
    return currentParkingArea;
}


const std::vector<std::string>&
MSBaseVehicle::getParkingBadges() const {
    if (myParameter->wasSet(VEHPARS_PARKING_BADGES_SET)) {
        return myParameter->parkingBadges;
    } else {
        return getVehicleType().getParkingBadges();
    }
}


double
MSBaseVehicle::basePos(const MSEdge* edge) const {
    double result = MIN2(getVehicleType().getLength() + POSITION_EPS, edge->getLength());
    if (hasStops()
            && myStops.front().edge == myRoute->begin()
            && (&myStops.front().lane->getEdge()) == *myStops.front().edge) {
        result = MIN2(result, MAX2(0.0, myStops.front().getEndPos(*this)));
    }
    return result;
}


MSLane*
MSBaseVehicle::interpretOppositeStop(SUMOVehicleParameter::Stop& stop) {
    const std::string edgeID = SUMOXMLDefinitions::getEdgeIDFromLane(stop.lane);
    const MSEdge* edge = MSEdge::dictionary(edgeID);
    if (edge == nullptr || edge->getOppositeEdge() == nullptr || stop.lane.find("_") == std::string::npos) {
        return nullptr;
    }
    const int laneIndex = SUMOXMLDefinitions::getIndexFromLane(stop.lane);
    if (laneIndex < (edge->getNumLanes() + edge->getOppositeEdge()->getNumLanes())) {
        const int oppositeIndex = edge->getOppositeEdge()->getNumLanes() + edge->getNumLanes() - 1 - laneIndex;
        stop.edge = edgeID;
        return edge->getOppositeEdge()->getLanes()[oppositeIndex];
    }
    return nullptr;
}


bool
MSBaseVehicle::addStop(const SUMOVehicleParameter::Stop& stopPar, std::string& errorMsg, SUMOTime untilOffset,
                       MSRouteIterator* searchStart) {
    MSStop stop(stopPar);
    if (stopPar.lane == "") {
        MSEdge* e = MSEdge::dictionary(stopPar.edge);
        stop.lane = e->getFirstAllowed(getVClass());
        if (stop.lane == nullptr) {
            errorMsg = "Vehicle '" + myParameter->id + "' is not allowed to stop on any lane of edge '" + stopPar.edge + "'.";
            return false;
        }
    } else {
        stop.lane = MSLane::dictionary(stopPar.lane);
        if (stop.lane == nullptr) {
            // must be an opposite stop
            SUMOVehicleParameter::Stop tmp = stopPar;
            stop.lane = interpretOppositeStop(tmp);
            assert(stop.lane != nullptr);
        }
        if (!stop.lane->allowsVehicleClass(myType->getVehicleClass())) {
            errorMsg = "Vehicle '" + myParameter->id + "' is not allowed to stop on lane '" + stopPar.lane + "'.";
            return false;
        }
    }
    if (MSGlobals::gUseMesoSim) {
        stop.segment = MSGlobals::gMesoNet->getSegmentForEdge(stop.lane->getEdge(), stop.getEndPos(*this));
        if (stop.lane->isInternal()) {
            errorMsg = "Mesoscopic simulation does not allow stopping on internal edge '" + stopPar.edge + "' for vehicle '" + myParameter->id + "'.";
            return false;
        }
    }
    stop.initPars(stopPar);
    if (stopPar.until != -1) {
        // !!! it would be much cleaner to invent a constructor for stops which takes "until" as an argument
        const_cast<SUMOVehicleParameter::Stop&>(stop.pars).until += untilOffset;
    }
    if (stopPar.arrival != -1) {
        const_cast<SUMOVehicleParameter::Stop&>(stop.pars).arrival += untilOffset;
    }
    std::string stopType = "stop";
    std::string stopID = "";
    if (stop.busstop != nullptr) {
        stopType = "busStop";
        stopID = stop.busstop->getID();
    } else if (stop.containerstop != nullptr) {
        stopType = "containerStop";
        stopID = stop.containerstop->getID();
    } else if (stop.chargingStation != nullptr) {
        stopType = "chargingStation";
        stopID = stop.chargingStation->getID();
    } else if (stop.overheadWireSegment != nullptr) {
        stopType = "overheadWireSegment";
        stopID = stop.overheadWireSegment->getID();
    } else if (stop.parkingarea != nullptr) {
        stopType = "parkingArea";
        stopID = stop.parkingarea->getID();
    }
    const std::string errorMsgStart = stopID == "" ? stopType : stopType + " '" + stopID + "'";

    if (stop.pars.startPos < 0 || stop.pars.endPos > stop.lane->getLength()) {
        errorMsg = errorMsgStart + " for vehicle '" + myParameter->id + "' on lane '" + stop.lane->getID() + "' has an invalid position.";
        return false;
    }
    if (stopType != "stop" && stopType != "parkingArea" && myType->getLength() / 2. > stop.pars.endPos - stop.pars.startPos
            && MSNet::getInstance()->warnOnce(stopType + ":" + stopID)) {
        errorMsg = errorMsgStart + " on lane '" + stop.lane->getID() + "' is too short for vehicle '" + myParameter->id + "'.";
    }
    if (stopType == "parkingArea" && !stop.parkingarea->accepts(this)) {
        // forbid access in case the parking requests other badges
        errorMsg = errorMsgStart + "on lane '" + stop.lane->getID() + "' forbids access because vehicle '" + myParameter->id + "' does not provide any valid badge.";
        return false;
    }
    const MSEdge* stopLaneEdge = &stop.lane->getEdge();
    const MSEdge* stopEdge;
    if (stopLaneEdge->getOppositeEdge() != nullptr && stopLaneEdge->getOppositeEdge()->getID() == stopPar.edge) {
        // stop lane is on the opposite side
        stopEdge = stopLaneEdge->getOppositeEdge();
        stop.isOpposite = true;
    } else {
        // if stop is on an internal edge the normal edge before the intersection is used
        stopEdge = stopLaneEdge->getNormalBefore();
    }
    MSRouteIterator succ = myCurrEdge + 1; // we're using the address but only within the scope of this function (and recursive calls)
    if (searchStart == nullptr) {
        searchStart = &myCurrEdge;
        if (stopLaneEdge->isNormal() && getLane() != nullptr && getLane()->isInternal()) {
            // already on the intersection but myCurrEdge is before it
            searchStart = &succ;
        }
    }
#ifdef DEBUG_ADD_STOP
    if (DEBUG_COND) {
        std::cout << "addStop desc=" << stop.getDescription() << " stopEdge=" << stopEdge->getID()
                  << " searchStart=" << ((*searchStart) == myRoute->end() ? "END" : (**searchStart)->getID())
                  << " index=" << (int)((*searchStart) - myRoute->begin()) << " route=" << toString(myRoute->getEdges())
                  << "\n";
    }
#endif
    stop.edge = std::find(*searchStart, myRoute->end(), stopEdge);
    MSRouteIterator prevStopEdge = myCurrEdge;
    const MSEdge* prevEdge = (getLane() == nullptr ? getEdge() : &getLane()->getEdge());
    double prevStopPos = getPositionOnLane();
    // where to insert the stop
    std::list<MSStop>::iterator iter = myStops.begin();
    if (stopPar.index == STOP_INDEX_END || stopPar.index >= static_cast<int>(myStops.size()) || stopPar.index == STOP_INDEX_REPEAT) {
        iter = myStops.end();
        if (myStops.size() > 0 && myStops.back().edge >= *searchStart) {
            prevStopEdge = myStops.back().edge;
            prevEdge = &myStops.back().lane->getEdge();
            prevStopPos = myStops.back().pars.endPos;
            stop.edge = std::find(prevStopEdge, myRoute->end(), stopEdge);
            if (prevStopEdge == stop.edge                // laneEdge check is insufficient for looped routes
                    && prevEdge == &stop.lane->getEdge() // route iterator check insufficient for internal lane stops
                    && (prevStopPos > stop.pars.endPos ||
                        (prevStopPos == stop.pars.endPos && stopPar.index == STOP_INDEX_REPEAT))) {
                stop.edge = std::find(prevStopEdge + 1, myRoute->end(), stopEdge);
            }
#ifdef DEBUG_ADD_STOP
            if (DEBUG_COND) {
                std::cout << " (@end) prevStopEdge=" << (*prevStopEdge)->getID() << " prevStopPos=" << prevStopPos << " index=" << (int)(prevStopEdge - myRoute->begin())
                          << " foundIndex=" << (stop.edge == myRoute->end() ? -1 : (int)(stop.edge - myRoute->begin())) << "\n";
            }
#endif
        }
        // skip a number of occurences of stopEdge in looped route
        int skipLooped = stopPar.index - static_cast<int>(myStops.size());
        for (int j = 0; j < skipLooped; j++) {
            auto nextIt = std::find(stop.edge + 1, myRoute->end(), stopEdge);
            if (nextIt == myRoute->end()) {
                if (std::find(myRoute->begin(), stop.edge, stopEdge) != stop.edge) {
                    // only warn if the route loops over the stop edge at least once
                    errorMsg = errorMsgStart + " for vehicle '" + myParameter->id + "' could not skip " + toString(skipLooped) + " occurences of stop edge '" + stopEdge->getID() + "' in looped route.";
                }
                break;
            } else {
                stop.edge = nextIt;
            }
        }
    } else {
        if (stopPar.index == STOP_INDEX_FIT) {
            while (iter != myStops.end() && (iter->edge < stop.edge ||
                                             (iter->pars.endPos < stop.pars.endPos && iter->edge == stop.edge) ||
                                             (stop.lane->getEdge().isInternal() && iter->edge == stop.edge))) {
                prevStopEdge = iter->edge;
                prevStopPos = iter->pars.endPos;
                ++iter;
            }
        } else {
            int index = stopPar.index;
            while (index > 0) {
                prevStopEdge = iter->edge;
                prevStopPos = iter->pars.endPos;
                ++iter;
                --index;
            }
#ifdef DEBUG_ADD_STOP
            if (DEBUG_COND) {
                std::cout << " (@fit) prevStopEdge=" << (*prevStopEdge)->getID() << " index=" << (int)(prevStopEdge - myRoute->begin()) << "\n";
            }
#endif
            stop.edge = std::find(prevStopEdge, myRoute->end(), stopEdge);
        }
    }
    const bool wasTooClose = errorMsg != "" && errorMsg.find("too close") != std::string::npos;
    if (stop.edge == myRoute->end()) {
        if (!wasTooClose) {
            errorMsg = errorMsgStart + " for vehicle '" + myParameter->id + "' on lane '" + stop.lane->getID() + "' is not downstream the current route.";
        }
        return false;
    }

    const bool tooClose = (prevStopEdge == stop.edge && prevEdge == &stop.lane->getEdge() &&
                           prevStopPos + (iter == myStops.begin() ? getBrakeGap() : 0) > stop.pars.endPos + POSITION_EPS);

    if (prevStopEdge > stop.edge ||
            // a collision-stop happens after vehicle movement and may move the
            // vehicle backwards on its lane (prevStopPos is the vehicle position)
            (tooClose && !stop.pars.collision)
            || (stop.lane->getEdge().isInternal() && stop.lane->getNextNormal() != *(stop.edge + 1))) {
        // check if the edge occurs again later in the route
        //std::cout << " could not add stop " << errorMsgStart << " prevStops=" << myStops.size() << " searchStart=" << (*searchStart - myRoute->begin()) << " route=" << toString(myRoute->getEdges())  << "\n";
        if (tooClose && prevStopPos <= stop.pars.endPos + POSITION_EPS) {
            errorMsg = errorMsgStart + " for vehicle '" + myParameter->id + "' on lane '" + stop.pars.lane + "' is too close to brake.";
        }
        MSRouteIterator next = stop.edge + 1;
        return addStop(stopPar, errorMsg, untilOffset, &next);
    }
    if (wasTooClose) {
        errorMsg = "";
    }
    // David.C:
    //if (!stop.parking && (myCurrEdge == stop.edge && myState.myPos > stop.endPos - getCarFollowModel().brakeGap(myState.mySpeed))) {
    const double endPosOffset = stop.lane->getEdge().isInternal() ? (*stop.edge)->getLength() : 0;
    const double distToStop = stop.pars.endPos + endPosOffset - getPositionOnLane();
    if (stop.pars.collision && !handleCollisionStop(stop, distToStop)) {
        return false;
    }
    if (!hasDeparted() && myCurrEdge == stop.edge) {
        double pos = -1;
        if (myParameter->departPosProcedure == DepartPosDefinition::GIVEN) {
            pos = myParameter->departPos;
            if (pos < 0.) {
                pos += (*myCurrEdge)->getLength();
            }
        }
        if (myParameter->departPosProcedure == DepartPosDefinition::BASE || myParameter->departPosProcedure == DepartPosDefinition::DEFAULT) {
            pos = MIN2(stop.pars.endPos + endPosOffset, basePos(*myCurrEdge));
        }
        if (pos > stop.pars.endPos + endPosOffset) {
            if (stop.edge != myRoute->end()) {
                // check if the edge occurs again later in the route
                MSRouteIterator next = stop.edge + 1;
                return addStop(stopPar, errorMsg, untilOffset, &next);
            }
            errorMsg = errorMsgStart + " for vehicle '" + myParameter->id + "' on lane '" + stop.lane->getID() + "' is before departPos.";
            return false;
        }
    }
    if (iter != myStops.begin()) {
        std::list<MSStop>::iterator iter2 = iter;
        iter2--;
        if (stop.getUntil() >= 0 && iter2->getUntil() > stop.getUntil()
                && (!MSGlobals::gUseStopEnded || iter2->pars.ended < 0 || stop.pars.ended >= 0)) {
            errorMsg = errorMsgStart + " for vehicle '" + myParameter->id + "' on lane '" + stop.lane->getID()
                       + "' set to end at " + time2string(stop.getUntil())
                       + " earlier than previous stop at " + time2string(iter2->getUntil()) + ".";
        }
        if (stop.pars.arrival >= 0 && iter2->pars.arrival > stop.pars.arrival) {
            errorMsg = errorMsgStart + " for vehicle '" + myParameter->id + "' on lane '" + stop.lane->getID()
                       + "' set to start at " + time2string(stop.pars.arrival)
                       + " earlier than previous stop end at " + time2string(iter2->getUntil()) + ".";
        }
        if (stop.pars.arrival >= 0 && iter2->pars.arrival > stop.pars.arrival) {
            errorMsg = errorMsgStart + " for vehicle '" + myParameter->id + "' on lane '" + stop.lane->getID()
                       + "' set to start at " + time2string(stop.pars.arrival)
                       + " earlier than previous stop arrival at " + time2string(iter2->pars.arrival) + ".";
        }
    } else {
        if (stop.getUntil() >= 0 && getParameter().depart > stop.getUntil()
                && (!MSGlobals::gUseStopEnded || stop.pars.ended < 0)) {
            errorMsg = errorMsgStart + " for vehicle '" + myParameter->id + "' on lane '" + stop.lane->getID()
                       + "' set to end at " + time2string(stop.getUntil())
                       + " earlier than departure at " + time2string(getParameter().depart) + ".";
        }
    }
    if (stop.getUntil() >= 0 && stop.getArrival() > stop.getUntil() && errorMsg == "") {
        errorMsg = errorMsgStart + " for vehicle '" + myParameter->id + "' on lane '" + stop.lane->getID()
                   + "' set to end at " + time2string(stop.getUntil())
                   + " earlier than arrival at " + time2string(stop.getArrival()) + ".";
    }
    setSkips(stop, (int)myStops.size());
    myStops.insert(iter, stop);
    if (stopPar.tripId != "") {
        MSRailSignalConstraint::storeTripId(stopPar.tripId, getID());
    }
    //std::cout << " added stop " << errorMsgStart << " totalStops=" << myStops.size() << " searchStart=" << (*searchStart - myRoute->begin())
    //    << " routeIndex=" << (stop.edge - myRoute->begin())
    //    << " stopIndex=" << std::distance(myStops.begin(), iter)
    //    << " route=" << toString(myRoute->getEdges())  << "\n";
    return true;
}


void
MSBaseVehicle::setSkips(MSStop& stop, int prevActiveStops) {
    if (hasDeparted() && stop.edge > myRoute->begin()) {
        // if the route is looped we must patch the index to ensure that state
        // loading (and vehroute-output) encode the correct number of skips
        int foundSkips = 0;
        MSRouteIterator itPrev;
        double prevEndPos;
        if (prevActiveStops > 0) {
            assert((int)myStops.size() >= prevActiveStops);
            auto prevStopIt = myStops.begin();
            std::advance(prevStopIt, prevActiveStops - 1);
            const MSStop& prev = *prevStopIt;
            itPrev = prev.edge;
            prevEndPos = prev.pars.endPos;
        } else if (myPastStops.size() > 0) {
            itPrev = myRoute->begin() + myPastStops.back().routeIndex;
            prevEndPos = myPastStops.back().endPos;
        } else {
            itPrev = myRoute->begin() + myParameter->departEdge;
            prevEndPos = myDepartPos;
        }
        //auto itPrevOrig = itPrev;
        if (*itPrev == *stop.edge && prevEndPos > stop.pars.endPos) {
            itPrev++;
        }
        //std::cout << SIMTIME << " veh=" << getID() << " prevActive=" << prevActiveStops << " edge=" << (*stop.edge)->getID() << " routeIndex=" << (stop.edge - myRoute->begin()) << " prevIndex=" << (itPrev - myRoute->begin()) << "\n";
        while (itPrev < stop.edge) {
            if (*itPrev == *stop.edge) {
                foundSkips++;
            }
            itPrev++;
        }
        int newIndex = STOP_INDEX_END;
        if (foundSkips > 0) {
            //if (getID() == "77_0_0") {
            //    std::cout << SIMTIME << " veh=" << getID() << " past=" << myPastStops.size() << " prevActive=" << prevActiveStops
            //        << " edge=" << (*stop.edge)->getID() << " routeIndex=" << (stop.edge - myRoute->begin())
            //        << " prevEdge=" << (*itPrevOrig)->getID()
            //        << " prevIndex=" << (itPrevOrig - myRoute->begin())
            //        << " skips=" << foundSkips << "\n";
            //}
            newIndex = (int)myPastStops.size() + prevActiveStops + foundSkips;
        }
        const_cast<SUMOVehicleParameter::Stop&>(stop.pars).index = newIndex;
    }
}


void
MSBaseVehicle::addStops(const bool ignoreStopErrors, MSRouteIterator* searchStart, bool addRouteStops) {
    if (addRouteStops) {
        for (const SUMOVehicleParameter::Stop& stop : myRoute->getStops()) {
            std::string errorMsg;
            if (!addStop(stop, errorMsg, myParameter->depart, searchStart) && !ignoreStopErrors) {
                throw ProcessError(errorMsg);
            }
            if (errorMsg != "") {
                WRITE_WARNING(errorMsg);
            }
        }
    }
    const SUMOTime untilOffset = myParameter->repetitionOffset > 0 ? myParameter->repetitionsDone * myParameter->repetitionOffset : 0;
    for (const SUMOVehicleParameter::Stop& stop : myParameter->stops) {
        std::string errorMsg;
        if (!addStop(stop, errorMsg, untilOffset, searchStart) && !ignoreStopErrors) {
            throw ProcessError(errorMsg);
        }
        if (errorMsg != "") {
            WRITE_WARNING(errorMsg);
        }
    }
}


bool
MSBaseVehicle::haveValidStopEdges(bool silent) const {
    MSRouteIterator start = myCurrEdge;
    int i = 0;
    bool ok = true;
    for (const MSStop& stop : myStops) {
        MSRouteIterator it;
        if (stop.lane->isInternal()) {
            // find the normal predecessor and ensure that the next route edge
            // matches the successor of the internal edge successor
            it = std::find(start, myRoute->end(), stop.lane->getEdge().getNormalBefore());
            if (it != myRoute->end() && (
                        it + 1 == myRoute->end() || *(it + 1) != stop.lane->getEdge().getNormalSuccessor())) {
                it = myRoute->end(); // signal failure
            }
        } else {
            it = std::find(start, myRoute->end(), &stop.lane->getEdge());
        }
        if (it == myRoute->end()) {
            if (!silent) {
                WRITE_ERRORF("Stop % on edge '%' is not found after edge '%' (% after current) for vehicle '%' at time=%.",
                             i, stop.lane->getEdge().getID(), (*start)->getID(), toString(start - myCurrEdge), getID(), time2string(SIMSTEP));
            }
            ok = false;
        } else {
            MSRouteIterator it2;
            for (it2 = myRoute->begin(); it2 != myRoute->end(); it2++) {
                if (it2 == stop.edge) {
                    break;
                }
            }
            if (it2 == myRoute->end()) {
                if (!silent) {
                    WRITE_ERRORF("Stop % on edge '%' used invalid route index for vehicle '%' at time=%.",
                                 i, stop.lane->getEdge().getID(), getID(), time2string(SIMSTEP));
                }
                ok = false;
            } else if (it2 < start) {
                if (!silent) {
                    WRITE_ERRORF("Stop % on edge '%' used invalid (relative) route index % expected after % for vehicle '%' at time=%.",
                                 i, stop.lane->getEdge().getID(), toString(it2 - myCurrEdge), toString(start - myCurrEdge), getID(), time2string(SIMSTEP));
                }
                ok = false;
            } else {
                start = stop.edge;
            }
        }
        i++;
    }
    return ok;
}


const ConstMSEdgeVector
MSBaseVehicle::getStopEdges(double& firstPos, double& lastPos, std::set<int>& jumps) const {
    assert(haveValidStopEdges());
    ConstMSEdgeVector result;
    const MSStop* prev = nullptr;
    const MSEdge* internalSuccessor = nullptr;
    for (const MSStop& stop : myStops) {
        if (stop.reached) {
            if (stop.pars.jump >= 0) {
                jumps.insert((int)result.size());
            }
            continue;
        }
        const double stopPos = stop.getEndPos(*this);
        if ((prev == nullptr
                || prev->edge != stop.edge
                || (prev->lane == stop.lane && prev->getEndPos(*this) > stopPos))
                && *stop.edge != internalSuccessor) {
            result.push_back(*stop.edge);
            if (stop.lane->isInternal()) {
                internalSuccessor = stop.lane->getNextNormal();
                result.push_back(internalSuccessor);
            } else {
                internalSuccessor = nullptr;
            }
        }
        prev = &stop;
        if (firstPos == INVALID_DOUBLE) {
            if (stop.parkingarea != nullptr) {
                firstPos = MAX2(0., stopPos);
            } else {
                firstPos = stopPos;
            }
        }
        lastPos = stopPos;
        if (stop.pars.jump >= 0) {
            jumps.insert((int)result.size() - 1);
        }
    }
    //std::cout << "getStopEdges veh=" << getID() << " result=" << toString(result) << "\n";
    return result;
}


std::vector<std::pair<int, double> >
MSBaseVehicle::getStopIndices() const {
    std::vector<std::pair<int, double> > result;
    for (std::list<MSStop>::const_iterator iter = myStops.begin(); iter != myStops.end(); ++iter) {
        result.push_back(std::make_pair(
                             (int)(iter->edge - myRoute->begin()),
                             iter->getEndPos(*this)));
    }
    return result;
}


MSStop&
MSBaseVehicle::getNextStop() {
    assert(myStops.size() > 0);
    return myStops.front();
}

SUMOTime
MSBaseVehicle::getStopDuration() const {
    if (isStopped()) {
        return myStops.front().duration;
    } else {
        return 0;
    }
}


MSStop&
MSBaseVehicle::getStop(int nextStopIndex) {
    if (nextStopIndex < 0 || (int)myStops.size() <= nextStopIndex) {
        throw InvalidArgument(TLF("Invalid stop index % (has % stops).", nextStopIndex, myStops.size()));
    }
    auto stopIt = myStops.begin();
    std::advance(stopIt, nextStopIndex);
    return *stopIt;
}


const SUMOVehicleParameter::Stop*
MSBaseVehicle::getNextStopParameter() const {
    if (hasStops()) {
        return &myStops.front().pars;
    }
    return nullptr;
}


bool
MSBaseVehicle::addTraciStop(SUMOVehicleParameter::Stop stop, std::string& errorMsg) {
    //if the stop exists update the duration
    for (std::list<MSStop>::iterator iter = myStops.begin(); iter != myStops.end(); iter++) {
        if (iter->lane->getID() == stop.lane && fabs(iter->pars.endPos - stop.endPos) < POSITION_EPS) {
            // update existing stop
            if (stop.duration == 0 && stop.until < 0 && !iter->reached) {
                myStops.erase(iter);
            } else {
                iter->duration = stop.duration;
                iter->triggered = stop.triggered;
                iter->containerTriggered = stop.containerTriggered;
                const_cast<SUMOVehicleParameter::Stop&>(iter->pars).until = stop.until;
                const_cast<SUMOVehicleParameter::Stop&>(iter->pars).parking = stop.parking;
            }
            return true;
        }
    }
    const bool result = addStop(stop, errorMsg);
    if (result) {
        /// XXX handle stops added out of order
        myParameter->stops.push_back(stop);
    }
    return result;
}


void
MSBaseVehicle::unregisterWaiting() {
    if (myAmRegisteredAsWaiting) {
        MSNet::getInstance()->getVehicleControl().unregisterOneWaiting();
        myAmRegisteredAsWaiting = false;
    }
}


bool
MSBaseVehicle::abortNextStop(int nextStopIndex) {
    if (hasStops() && nextStopIndex < (int)myStops.size()) {
        if (nextStopIndex == 0 && isStopped()) {
            resumeFromStopping();
        } else {
            auto stopIt = myStops.begin();
            std::advance(stopIt, nextStopIndex);
            myStops.erase(stopIt);
        }
        if (!hasDeparted() && (int)myParameter->stops.size() > nextStopIndex) {
            // stops will be rebuilt from scratch on rerouting so we must patch the stops in myParameter
            auto stopIt2 = myParameter->stops.begin();
            std::advance(stopIt2, nextStopIndex);
            const_cast<SUMOVehicleParameter*>(myParameter)->stops.erase(stopIt2);
        }
        return true;
    } else {
        return false;
    }
}


bool
MSBaseVehicle::replaceStop(int nextStopIndex, SUMOVehicleParameter::Stop stop, const std::string& info, bool teleport, std::string& errorMsg) {
    const int n = (int)myStops.size();
    if (nextStopIndex < 0 || nextStopIndex >= n) {
        errorMsg = TLF("invalid nextStopIndex % for % remaining stops", nextStopIndex, n);
        return false;
    }
    if (nextStopIndex == 0 && isStopped()) {
        errorMsg = TL("cannot replace reached stop");
        return false;
    }
    const SUMOTime t = MSNet::getInstance()->getCurrentTimeStep();
    MSLane* stopLane = MSLane::dictionary(stop.lane);
    MSEdge* stopEdge = &stopLane->getEdge();

    auto itStop = myStops.begin();
    std::advance(itStop, nextStopIndex);
    MSStop& replacedStop = *itStop;

    // check parking access rights
    if (stop.parkingarea != "") {
        MSParkingArea* pa = dynamic_cast<MSParkingArea*>(MSNet::getInstance()->getStoppingPlace(stop.parkingarea, SUMO_TAG_PARKING_AREA));
        if (pa != nullptr && !pa->accepts(this)) {
            errorMsg = TLF("vehicle '%' does not have the right badge to access parkingArea '%'", getID(), stop.parkingarea);
            return false;
        }
    }

    if (replacedStop.lane == stopLane && replacedStop.pars.endPos == stop.endPos && !teleport) {
        // only replace stop attributes
        const_cast<SUMOVehicleParameter::Stop&>(replacedStop.pars) = stop;
        replacedStop.initPars(stop);
        return true;
    }

    if (!stopLane->allowsVehicleClass(getVClass(), myRoutingMode)) {
        errorMsg = TLF("disallowed stop lane '%'", stopLane->getID());
        return false;
    }

    const ConstMSEdgeVector& oldEdges = getRoute().getEdges();
    std::vector<MSStop> stops(myStops.begin(), myStops.end());
    const int junctionOffset = getLane() != nullptr && getLane()->isInternal() ? 1 : 0;
    MSRouteIterator itStart = nextStopIndex == 0 ? getCurrentRouteEdge() + junctionOffset : stops[nextStopIndex - 1].edge;
    double startPos = nextStopIndex == 0 ? getPositionOnLane() : stops[nextStopIndex - 1].pars.endPos;
    MSRouteIterator itEnd = nextStopIndex == n - 1 ? oldEdges.end() - 1 : stops[nextStopIndex + 1].edge;
    auto endPos = nextStopIndex == n - 1 ? getArrivalPos() : stops[nextStopIndex + 1].pars.endPos;
    SUMOAbstractRouter<MSEdge, SUMOVehicle>& router = getRouterTT();

    bool newDestination = nextStopIndex == n - 1 && stops[nextStopIndex].edge == oldEdges.end() - 1;

    ConstMSEdgeVector toNewStop;
    if (!teleport) {
        router.compute(*itStart, startPos, stopEdge, stop.endPos, this, t, toNewStop, true);
        if (toNewStop.size() == 0) {
            errorMsg = TLF("no route found from edge '%' to stop edge '%'", (*itStart)->getID(), stopEdge->getID());
            return false;
        }
    }

    ConstMSEdgeVector fromNewStop;
    if (!newDestination) {
        router.compute(stopEdge, stop.endPos, *itEnd, endPos, this, t, fromNewStop, true);
        if (fromNewStop.size() == 0) {
            errorMsg = TLF("no route found from stop edge '%' to edge '%'", stopEdge->getID(), (*itEnd)->getID());
            return false;
        }
    }

    const_cast<SUMOVehicleParameter::Stop&>(replacedStop.pars) = stop;
    replacedStop.initPars(stop);
    replacedStop.edge = myRoute->end(); // will be patched in replaceRoute
    replacedStop.lane = stopLane;
    if (MSGlobals::gUseMesoSim) {
        replacedStop.segment = MSGlobals::gMesoNet->getSegmentForEdge(replacedStop.lane->getEdge(), replacedStop.getEndPos(*this));
        if (replacedStop.lane->isInternal()) {
            errorMsg = TLF("Mesoscopic simulation does not allow stopping on internal edge '%' for vehicle '%'.", stop.edge, getID());
            return false;
        }
    }

    ConstMSEdgeVector oldRemainingEdges(myCurrEdge, getRoute().end());
    ConstMSEdgeVector newEdges; // only remaining
    newEdges.insert(newEdges.end(), myCurrEdge, itStart);
    if (!teleport) {
        newEdges.insert(newEdges.end(), toNewStop.begin(), toNewStop.end() - 1);
    } else {
        newEdges.push_back(*itStart);
    }
    if (!newDestination) {
        newEdges.insert(newEdges.end(), fromNewStop.begin(), fromNewStop.end() - 1);
        newEdges.insert(newEdges.end(), itEnd, oldEdges.end());
    } else {
        newEdges.push_back(stopEdge);
    }
    //std::cout << SIMTIME << " replaceStop veh=" << getID()
    //    << " teleport=" << teleport
    //    << " busStop=" << stop.busstop
    //    << " oldEdges=" << oldRemainingEdges.size()
    //    << " newEdges=" << newEdges.size()
    //    << " toNewStop=" << toNewStop.size()
    //    << " fromNewStop=" << fromNewStop.size()
    //    << "\n";

    const double routeCost = router.recomputeCosts(newEdges, this, t);
    const double previousCost = router.recomputeCosts(oldRemainingEdges, this, t);
    const double savings = previousCost - routeCost;
    if (!hasDeparted() && (int)myParameter->stops.size() > nextStopIndex) {
        // stops will be rebuilt from scratch so we must patch the stops in myParameter
        const_cast<SUMOVehicleParameter*>(myParameter)->stops[nextStopIndex] = stop;
    }
    if (teleport) {
        // let the vehicle jump rather than teleport
        // we add a jump-stop at the end of the edge (unless the vehicle is
        // already configure to jump before the replaced stop)
        if (!insertJump(nextStopIndex, itStart, errorMsg)) {
            return false;
        };
    }
    return replaceRouteEdges(newEdges, routeCost, savings, info, !hasDeparted(), false, false, &errorMsg);
}


bool
MSBaseVehicle::rerouteBetweenStops(int nextStopIndex, const std::string& info, bool teleport, std::string& errorMsg) {
    const int n = (int)myStops.size();
    if (nextStopIndex < 0 || nextStopIndex > n) {
        errorMsg = TLF("invalid nextStopIndex % for % remaining stops", nextStopIndex, n);
        return false;
    }
    if (nextStopIndex == 0 && isStopped()) {
        errorMsg = TL("cannot reroute towards reached stop");
        return false;
    }
    const SUMOTime t = MSNet::getInstance()->getCurrentTimeStep();

    const ConstMSEdgeVector& oldEdges = getRoute().getEdges();
    std::vector<MSStop> stops(myStops.begin(), myStops.end());
    const int junctionOffset = getLane() != nullptr && getLane()->isInternal() ? 1 : 0;
    MSRouteIterator itStart = nextStopIndex == 0 ? getCurrentRouteEdge() + junctionOffset : stops[nextStopIndex - 1].edge;
    double startPos = nextStopIndex == 0 ? getPositionOnLane() : stops[nextStopIndex - 1].pars.endPos;
    MSRouteIterator itEnd = nextStopIndex == n ? oldEdges.end() - 1 : stops[nextStopIndex].edge;
    auto endPos = nextStopIndex == n ? getArrivalPos() : stops[nextStopIndex].pars.endPos;
    SUMOAbstractRouter<MSEdge, SUMOVehicle>& router = getRouterTT();

    ConstMSEdgeVector newBetween;
    if (!teleport) {
        router.compute(*itStart, startPos, *itEnd, endPos, this, t, newBetween, true);
        if (newBetween.size() == 0) {
            errorMsg = TLF("no route found from edge '%' to stop edge '%'", (*itStart)->getID(), (*itEnd)->getID());
            return false;
        }
    }

    ConstMSEdgeVector oldRemainingEdges(myCurrEdge, getRoute().end());
    ConstMSEdgeVector newEdges; // only remaining
    newEdges.insert(newEdges.end(), myCurrEdge, itStart);
    if (!teleport) {
        newEdges.insert(newEdges.end(), newBetween.begin(), newBetween.end() - 1);
    } else {
        newEdges.push_back(*itStart);
    }
    newEdges.insert(newEdges.end(), itEnd, oldEdges.end());
    //std::cout << SIMTIME << " rerouteBetweenStops veh=" << getID()
    //    << " oldEdges=" << oldRemainingEdges.size()
    //    << " newEdges=" << newEdges.size()
    //    << " toNewStop=" << toNewStop.size()
    //    << " fromNewStop=" << fromNewStop.size()
    //    << "\n";

    const double routeCost = router.recomputeCosts(newEdges, this, t);
    const double previousCost = router.recomputeCosts(oldRemainingEdges, this, t);
    const double savings = previousCost - routeCost;

    if (teleport) {
        // let the vehicle jump rather than teleport
        // we add a jump-stop at the end of the edge (unless the vehicle is
        // already configure to jump before the replaced stop)
        if (!insertJump(nextStopIndex, itStart, errorMsg)) {
            return false;
        };
    }
    return replaceRouteEdges(newEdges, routeCost, savings, info, !hasDeparted(), false, false, &errorMsg);
}


bool
MSBaseVehicle::insertJump(int nextStopIndex, MSRouteIterator itStart, std::string& errorMsg) {
    bool needJump = true;
    if (nextStopIndex > 0) {
        auto itPriorStop = myStops.begin();
        std::advance(itPriorStop, nextStopIndex - 1);
        const MSStop& priorStop = *itPriorStop;
        if (priorStop.pars.jump >= 0) {
            needJump = false;
        }
    }
    if (needJump) {
        SUMOVehicleParameter::Stop jumpStopPars;
        jumpStopPars.endPos = (*itStart)->getLength();
        jumpStopPars.speed = 1000;
        jumpStopPars.jump = 0;
        jumpStopPars.edge = (*itStart)->getID();
        jumpStopPars.parametersSet = STOP_SPEED_SET | STOP_JUMP_SET;
        MSLane* jumpStopLane = nullptr;
        for (MSLane* cand : (*itStart)->getLanes()) {
            if (cand->allowsVehicleClass(getVClass())) {
                jumpStopLane = cand;
                break;
            }
        }
        if (jumpStopLane == nullptr) {
            errorMsg = TL("unable to replace stop with teleporting");
            return false;
        }
        auto itStop = myStops.begin();
        std::advance(itStop, nextStopIndex);
        MSStop jumpStop(jumpStopPars);
        jumpStop.initPars(jumpStopPars);
        jumpStop.lane = jumpStopLane;
        jumpStop.edge = myRoute->end(); // will be patched in replaceRoute
        myStops.insert(itStop, jumpStop);
        if (!hasDeparted() && (int)myParameter->stops.size() > nextStopIndex) {
            // stops will be rebuilt from scratch so we must patch the stops in myParameter
            auto it = myParameter->stops.begin() + nextStopIndex;
            const_cast<SUMOVehicleParameter*>(myParameter)->stops.insert(it, jumpStopPars);
        }
    }
    return true;
}


bool
MSBaseVehicle::insertStop(int nextStopIndex, SUMOVehicleParameter::Stop stop, const std::string& info, bool teleport, std::string& errorMsg) {
    const int n = (int)myStops.size();
    if (nextStopIndex < 0 || nextStopIndex > n) {
        errorMsg = TLF("invalid nextStopIndex % for % remaining stops", nextStopIndex, n);
        return false;
    }
    if (nextStopIndex == 0 && isStopped()) {
        errorMsg = TL("cannot insert stop before the currently reached stop");
        return false;
    }
    const SUMOTime t = MSNet::getInstance()->getCurrentTimeStep();
    MSLane* stopLane = MSLane::dictionary(stop.lane);
    MSEdge* stopEdge = &stopLane->getEdge();

    if (!stopLane->allowsVehicleClass(getVClass(), myRoutingMode)) {
        errorMsg = TLF("disallowed stop lane '%'", stopLane->getID());
        return false;
    }

    // check parking access rights
    if (stop.parkingarea != "") {
        MSParkingArea* pa = dynamic_cast<MSParkingArea*>(MSNet::getInstance()->getStoppingPlace(stop.parkingarea, SUMO_TAG_PARKING_AREA));
        if (pa != nullptr && !pa->accepts(this)) {
            errorMsg = TLF("Vehicle '%' does not have the right badge to access parkingArea '%'.", getID(), stop.parkingarea);
            return false;
        }
    }

    const ConstMSEdgeVector& oldEdges = getRoute().getEdges();
    std::vector<MSStop> stops(myStops.begin(), myStops.end());
    const int junctionOffset = getLane() != nullptr && getLane()->isInternal() ? 1 : 0;
    MSRouteIterator itStart = nextStopIndex == 0 ? getCurrentRouteEdge() + junctionOffset : stops[nextStopIndex - 1].edge;
    double startPos = nextStopIndex == 0 ? getPositionOnLane() : stops[nextStopIndex - 1].pars.endPos;
    MSRouteIterator itEnd = nextStopIndex == n ? oldEdges.end() - 1 : stops[nextStopIndex].edge;
    auto endPos = nextStopIndex == n ? getArrivalPos() : stops[nextStopIndex].pars.endPos;
    SUMOAbstractRouter<MSEdge, SUMOVehicle>& router = getRouterTT();

    bool newDestination = nextStopIndex == n && stopEdge == oldEdges.back();

    ConstMSEdgeVector toNewStop;
    if (!teleport) {
        router.compute(*itStart, startPos, stopEdge, stop.endPos, this, t, toNewStop, true);
        if (toNewStop.size() == 0) {
            errorMsg = TLF("no route found from edge '%' to stop edge '%'", (*itStart)->getID(), stopEdge->getID());
            return false;
        }
    }

    ConstMSEdgeVector fromNewStop;
    if (!newDestination) {
        router.compute(stopEdge, stop.endPos, *itEnd, endPos, this, t, fromNewStop, true);
        if (fromNewStop.size() == 0) {
            errorMsg = TLF("no route found from stop edge '%' to edge '%'", stopEdge->getID(), (*itEnd)->getID());
            return false;
        }
    }

    auto itStop = myStops.begin();
    std::advance(itStop, nextStopIndex);
    MSStop newStop(stop);
    newStop.initPars(stop);
    newStop.edge = myRoute->end(); // will be patched in replaceRoute
    newStop.lane = stopLane;
    if (MSGlobals::gUseMesoSim) {
        newStop.segment = MSGlobals::gMesoNet->getSegmentForEdge(newStop.lane->getEdge(), newStop.getEndPos(*this));
        if (newStop.lane->isInternal()) {
            errorMsg = TLF("Mesoscopic simulation does not allow stopping on internal edge '%' for vehicle '%'.", stop.edge, getID());
            return false;
        }
    }
    myStops.insert(itStop, newStop);

    ConstMSEdgeVector oldRemainingEdges(myCurrEdge, getRoute().end());
    ConstMSEdgeVector newEdges; // only remaining
    newEdges.insert(newEdges.end(), myCurrEdge, itStart);
    if (!teleport) {
        newEdges.insert(newEdges.end(), toNewStop.begin(), toNewStop.end() - 1);
    } else {
        newEdges.push_back(*itStart);
    }
    if (!newDestination) {
        newEdges.insert(newEdges.end(), fromNewStop.begin(), fromNewStop.end() - 1);
        newEdges.insert(newEdges.end(), itEnd, oldEdges.end());
    } else {
        newEdges.push_back(stopEdge);
    }
    //std::cout << SIMTIME << " insertStop veh=" << getID()
    //    << " teleport=" << teleport
    //    << " busStop=" << stop.busstop
    //    << " oldEdges=" << oldRemainingEdges.size()
    //    << " newEdges=" << newEdges.size()
    //    << " toNewStop=" << toNewStop.size()
    //    << " fromNewStop=" << fromNewStop.size()
    //    << "\n";

    const double routeCost = router.recomputeCosts(newEdges, this, t);
    const double previousCost = router.recomputeCosts(oldRemainingEdges, this, t);
    const double savings = previousCost - routeCost;

    if (!hasDeparted() && (int)myParameter->stops.size() >= nextStopIndex) {
        // stops will be rebuilt from scratch so we must patch the stops in myParameter
        auto it = myParameter->stops.begin() + nextStopIndex;
        const_cast<SUMOVehicleParameter*>(myParameter)->stops.insert(it, stop);
    }
    return replaceRouteEdges(newEdges, routeCost, savings, info, !hasDeparted(), false, false, &errorMsg);
}


double
MSBaseVehicle::getStateOfCharge() const {
    if (static_cast<MSDevice_Battery*>(getDevice(typeid(MSDevice_Battery))) != 0) {
        MSDevice_Battery* batteryOfVehicle = dynamic_cast<MSDevice_Battery*>(getDevice(typeid(MSDevice_Battery)));
        return batteryOfVehicle->getActualBatteryCapacity();
    } else {
        if (static_cast<MSDevice_ElecHybrid*>(getDevice(typeid(MSDevice_ElecHybrid))) != 0) {
            MSDevice_ElecHybrid* batteryOfVehicle = dynamic_cast<MSDevice_ElecHybrid*>(getDevice(typeid(MSDevice_ElecHybrid)));
            return batteryOfVehicle->getActualBatteryCapacity();
        }
    }
    return -1;
}


double
MSBaseVehicle::getRelativeStateOfCharge() const {
    if (static_cast<MSDevice_Battery*>(getDevice(typeid(MSDevice_Battery))) != 0) {
        MSDevice_Battery* batteryOfVehicle = dynamic_cast<MSDevice_Battery*>(getDevice(typeid(MSDevice_Battery)));
        return batteryOfVehicle->getActualBatteryCapacity() / batteryOfVehicle->getMaximumBatteryCapacity();
    } else {
        if (static_cast<MSDevice_ElecHybrid*>(getDevice(typeid(MSDevice_ElecHybrid))) != 0) {
            MSDevice_ElecHybrid* batteryOfVehicle = dynamic_cast<MSDevice_ElecHybrid*>(getDevice(typeid(MSDevice_ElecHybrid)));
            return batteryOfVehicle->getActualBatteryCapacity() / batteryOfVehicle->getMaximumBatteryCapacity();
        }
    }
    return -1;
}


double
MSBaseVehicle::getChargedEnergy() const {
    if (static_cast<MSDevice_Battery*>(getDevice(typeid(MSDevice_Battery))) != 0) {
        MSDevice_Battery* batteryOfVehicle = dynamic_cast<MSDevice_Battery*>(getDevice(typeid(MSDevice_Battery)));
        return batteryOfVehicle->getEnergyCharged();
    } else {
        if (static_cast<MSDevice_ElecHybrid*>(getDevice(typeid(MSDevice_ElecHybrid))) != 0) {
            MSDevice_ElecHybrid* batteryOfVehicle = dynamic_cast<MSDevice_ElecHybrid*>(getDevice(typeid(MSDevice_ElecHybrid)));
            return batteryOfVehicle->getEnergyCharged();
        }
    }
    return -1;
}


double
MSBaseVehicle::getMaxChargeRate() const {
    if (static_cast<MSDevice_Battery*>(getDevice(typeid(MSDevice_Battery))) != 0) {
        MSDevice_Battery* batteryOfVehicle = dynamic_cast<MSDevice_Battery*>(getDevice(typeid(MSDevice_Battery)));
        return batteryOfVehicle->getMaximumChargeRate();
    }
    return -1;
}


double
MSBaseVehicle::getElecHybridCurrent() const {
    if (static_cast<MSDevice_ElecHybrid*>(getDevice(typeid(MSDevice_ElecHybrid))) != 0) {
        MSDevice_ElecHybrid* elecHybridDevice = dynamic_cast<MSDevice_ElecHybrid*>(getDevice(typeid(MSDevice_ElecHybrid)));
        return elecHybridDevice->getCurrentFromOverheadWire();
    }

    return NAN;
}

double
MSBaseVehicle::getHarmonoise_NoiseEmissions() const {
    if (isOnRoad() || isIdling()) {
        return HelpersHarmonoise::computeNoise(myType->getEmissionClass(), getSpeed(), getAcceleration());
    } else {
        return 0.;
    }
}


const MSEdgeWeightsStorage&
MSBaseVehicle::getWeightsStorage() const {
    return _getWeightsStorage();
}


MSEdgeWeightsStorage&
MSBaseVehicle::getWeightsStorage() {
    return _getWeightsStorage();
}


MSEdgeWeightsStorage&
MSBaseVehicle::_getWeightsStorage() const {
    if (myEdgeWeights == nullptr) {
        myEdgeWeights = new MSEdgeWeightsStorage();
    }
    return *myEdgeWeights;
}




int
MSBaseVehicle::getPersonNumber() const {
    int boarded = myPersonDevice == nullptr ? 0 : myPersonDevice->size();
    return boarded + myParameter->personNumber;
}

int
MSBaseVehicle::getLeavingPersonNumber() const {
    int leavingPersonNumber = 0;
    const std::vector<MSTransportable*>& persons = getPersons();
    for (std::vector<MSTransportable*>::const_iterator it_p = persons.begin(); it_p != persons.end(); ++it_p) {
        MSStageDriving* const stage = dynamic_cast<MSStageDriving*>((*it_p)->getCurrentStage());
        const MSStop* stop = &myStops.front();
        const MSVehicle* joinVeh = dynamic_cast<MSVehicle*>(MSNet::getInstance()->getVehicleControl().getVehicle((*stop).pars.join));
        if (stop && stage->canLeaveVehicle(*it_p, *this, *stop) && !MSDevice_Transportable::willTransferAtJoin(*it_p, joinVeh)) {
            leavingPersonNumber++;
        }
    }
    return leavingPersonNumber;
}

std::vector<std::string>
MSBaseVehicle::getPersonIDList() const {
    std::vector<std::string> ret;
    const std::vector<MSTransportable*>& persons = getPersons();
    for (std::vector<MSTransportable*>::const_iterator it_p = persons.begin(); it_p != persons.end(); ++it_p) {
        ret.push_back((*it_p)->getID());
    }
    return ret;
}

int
MSBaseVehicle::getContainerNumber() const {
    int loaded = myContainerDevice == nullptr ? 0 : myContainerDevice->size();
    return loaded + myParameter->containerNumber;
}


void
MSBaseVehicle::removeTransportable(MSTransportable* t) {
    // this might be called from the MSTransportable destructor so we cannot do a dynamic cast to determine the type
    if (myPersonDevice != nullptr) {
        myPersonDevice->removeTransportable(t);
    }
    if (myContainerDevice != nullptr) {
        myContainerDevice->removeTransportable(t);
    }
}


const std::vector<MSTransportable*>&
MSBaseVehicle::getPersons() const {
    if (myPersonDevice == nullptr) {
        return myEmptyTransportableVector;
    } else {
        return myPersonDevice->getTransportables();
    }
}


const std::vector<MSTransportable*>&
MSBaseVehicle::getContainers() const {
    if (myContainerDevice == nullptr) {
        return myEmptyTransportableVector;
    } else {
        return myContainerDevice->getTransportables();
    }
}


bool
MSBaseVehicle::isLineStop(double position) const {
    if (myParameter->line == "") {
        // not a public transport line
        return false;
    }
    for (const SUMOVehicleParameter::Stop& stop : myParameter->stops) {
        if (stop.startPos <= position && position <= stop.endPos) {
            return true;
        }
    }
    for (const SUMOVehicleParameter::Stop& stop : myRoute->getStops()) {
        if (stop.startPos <= position && position <= stop.endPos) {
            return true;
        }
    }
    return false;
}


bool
MSBaseVehicle::hasDevice(const std::string& deviceName) const {
    for (MSDevice* const dev : myDevices) {
        if (dev->deviceName() == deviceName) {
            return true;
        }
    }
    return false;
}


void
MSBaseVehicle::createDevice(const std::string& deviceName) {
    if (!hasDevice(deviceName)) {
        if (deviceName == "rerouting") {
            ((SUMOVehicleParameter*)myParameter)->setParameter("has." + deviceName + ".device", "true");
            MSDevice_Routing::buildVehicleDevices(*this, myDevices);
            if (hasDeparted()) {
                // vehicle already departed: disable pre-insertion rerouting and enable regular routing behavior
                MSDevice_Routing* routingDevice = static_cast<MSDevice_Routing*>(getDevice(typeid(MSDevice_Routing)));
                assert(routingDevice != 0);
                routingDevice->notifyEnter(*this, MSMoveReminder::NOTIFICATION_DEPARTED);
            }
        } else {
            throw InvalidArgument(TLF("creating device of type '%' is not supported", deviceName));
        }
    }
}


std::string
MSBaseVehicle::getDeviceParameter(const std::string& deviceName, const std::string& key) const {
    for (MSVehicleDevice* const dev : myDevices) {
        if (dev->deviceName() == deviceName) {
            return dev->getParameter(key);
        }
    }
    throw InvalidArgument(TLF("no device of type '%' exists", deviceName));
}


void
MSBaseVehicle::setDeviceParameter(const std::string& deviceName, const std::string& key, const std::string& value) {
    for (MSVehicleDevice* const dev : myDevices) {
        if (dev->deviceName() == deviceName) {
            dev->setParameter(key, value);
            return;
        }
    }
    throw InvalidArgument(TLF("no device of type '%' exists", deviceName));
}


void
MSBaseVehicle::setJunctionModelParameter(const std::string& key, const std::string& value) {
    if (key == toString(SUMO_ATTR_JM_IGNORE_IDS) || key == toString(SUMO_ATTR_JM_IGNORE_TYPES)) {
        getParameter().parametersSet |= VEHPARS_JUNCTIONMODEL_PARAMS_SET;
        const_cast<SUMOVehicleParameter&>(getParameter()).setParameter(key, value);
        // checked in MSLink::ignoreFoe
    } else {
        throw InvalidArgument(TLF("Vehicle '%' does not support junctionModel parameter '%'.", getID(), key));
    }
}


void
MSBaseVehicle::setCarFollowModelParameter(const std::string& key, const std::string& value) {
    // handle some generic params first and then delegate to the carFollowModel itself
    if (key == toString(SUMO_ATTR_CF_IGNORE_IDS) || key == toString(SUMO_ATTR_CF_IGNORE_TYPES)) {
        getParameter().parametersSet |= VEHPARS_CFMODEL_PARAMS_SET;
        const_cast<SUMOVehicleParameter&>(getParameter()).setParameter(key, value);
        // checked in MSVehicle::planMove
    } else {
        MSVehicle* microVeh = dynamic_cast<MSVehicle*>(this);
        if (microVeh) {
            // remove 'carFollowModel.' prefix
            const std::string attrName = key.substr(15);
            microVeh->getCarFollowModel().setParameter(microVeh, attrName, value);
        }
    }
}


void
MSBaseVehicle::initTransientModelParams() {
    /* Design idea for additional junction model parameters:
       We can distinguish between 3 levels of parameters
       1. typically shared by multiple vehicles -> vType parameter
       2. specific to one vehicle but stays constant throughout the simulation -> vehicle parameter
       3. specific to one vehicle and expected to change during simulation -> prefixed generic vehicle parameter
       */
    for (auto item : getParameter().getParametersMap()) {
        if (StringUtils::startsWith(item.first, "junctionModel.")) {
            setJunctionModelParameter(item.first, item.second);
        } else if (StringUtils::startsWith(item.first, "carFollowModel.")) {
            setCarFollowModelParameter(item.first, item.second);
        }
    }
    const std::string routingModeStr = getStringParam("device.rerouting.mode");
    try {
        int routingMode = StringUtils::toInt(routingModeStr);
        if (routingMode != libsumo::ROUTING_MODE_DEFAULT) {
            setRoutingMode(routingMode);
        }
    } catch (NumberFormatException&) {
        // @todo interpret symbolic constants
        throw ProcessError(TLF("could not interpret routing.mode '%'", routingModeStr));
    }
}


SUMOAbstractRouter<MSEdge, SUMOVehicle>&
MSBaseVehicle::getRouterTT() const {
    if (myRoutingMode == libsumo::ROUTING_MODE_AGGREGATED) {
        return MSRoutingEngine::getRouterTT(getRNGIndex(), getVClass());
    } else {
        return MSNet::getInstance()->getRouterTT(getRNGIndex());
    }
}


void
MSBaseVehicle::replaceVehicleType(MSVehicleType* type) {
    assert(type != nullptr);
    // save old parameters before possible type deletion
    const double oldMu = myType->getSpeedFactor().getParameter()[0];
    const double oldDev = myType->getSpeedFactor().getParameter()[1];
    if (myType->isVehicleSpecific() && type != myType) {
        MSNet::getInstance()->getVehicleControl().removeVType(myType);
    }
    // adapt myChosenSpeedFactor to the new type
    if (oldDev == 0.) {
        // old type had speedDev 0, reroll
        myChosenSpeedFactor = type->computeChosenSpeedDeviation(getRNG());
    } else {
        // map old speedFactor onto new distribution
        const double distPoint = (myChosenSpeedFactor - oldMu) / oldDev;
        const double newMu = type->getSpeedFactor().getParameter()[0];
        const double newDev = type->getSpeedFactor().getParameter()[1];
        myChosenSpeedFactor = newMu + distPoint * newDev;
        // respect distribution limits
        myChosenSpeedFactor = MIN2(myChosenSpeedFactor, type->getSpeedFactor().getMax());
        myChosenSpeedFactor = MAX2(myChosenSpeedFactor, type->getSpeedFactor().getMin());
    }
    myType = type;
    if (myEnergyParams != nullptr) {
        myEnergyParams->setSecondary(type->getEmissionParameters());
    }
}


MSVehicleType&
MSBaseVehicle::getSingularType() {
    if (myType->isVehicleSpecific()) {
        return *myType;
    }
    MSVehicleType* type = myType->buildSingularType(myType->getID() + "@" + getID());
    replaceVehicleType(type);
    return *type;
}


int
MSBaseVehicle::getRNGIndex() const {
    const MSLane* const lane = getLane();
    if (lane == nullptr) {
        return getEdge()->getLanes()[0]->getRNGIndex();
    } else {
        return lane->getRNGIndex();
    }
}


SumoRNG*
MSBaseVehicle::getRNG() const {
    const MSLane* lane = getLane();
    if (lane == nullptr) {
        return getEdge()->getLanes()[0]->getRNG();
    } else {
        return lane->getRNG();
    }
}

std::string
MSBaseVehicle::getPrefixedParameter(const std::string& key, std::string& error) const {
    const MSVehicle* microVeh = dynamic_cast<const MSVehicle*>(this);
    if (StringUtils::startsWith(key, "device.")) {
        StringTokenizer tok(key, ".");
        if (tok.size() < 3) {
            error = TLF("Invalid device parameter '%' for vehicle '%'.", key, getID());
            return "";
        }
        try {
            return getDeviceParameter(tok.get(1), key.substr(tok.get(0).size() + tok.get(1).size() + 2));
        } catch (InvalidArgument& e) {
            error = TLF("Vehicle '%' does not support device parameter '%' (%).", getID(), key, e.what());
            return "";
        }
    } else if (StringUtils::startsWith(key, "laneChangeModel.")) {
        if (microVeh == nullptr) {
            error = TLF("Mesoscopic vehicle '%' does not support laneChangeModel parameters.", getID());
            return "";
        }
        const std::string attrName = key.substr(16);
        try {
            return microVeh->getLaneChangeModel().getParameter(attrName);
        } catch (InvalidArgument& e) {
            error = TLF("Vehicle '%' does not support laneChangeModel parameter '%' (%).", getID(), key, e.what());
            return "";
        }
    } else if (StringUtils::startsWith(key, "carFollowModel.")) {
        if (microVeh == nullptr) {
            error = TLF("Mesoscopic vehicle '%' does not support carFollowModel parameters.", getID());
            return "";
        }
        const std::string attrName = key.substr(15);
        try {
            return microVeh->getCarFollowModel().getParameter(microVeh, attrName);
        } catch (InvalidArgument& e) {
            error = TLF("Vehicle '%' does not support carFollowModel parameter '%' (%).", getID(), key, e.what());
            return "";
        }
    } else if (StringUtils::startsWith(key, "has.") && StringUtils::endsWith(key, ".device")) {
        StringTokenizer tok(key, ".");
        if (tok.size() != 3) {
            error = TL("Invalid check for device. Expected format is 'has.DEVICENAME.device'.");
            return "";
        }
        return hasDevice(tok.get(1)) ? "true" : "false";
        // parking related parameters start here
    } else if (key == "parking.rerouteCount") {
        return toString(getNumberParkingReroutes());
    } else if (StringUtils::startsWith(key, "parking.memory.")) {
        std::vector<std::string> values;
        if (getParkingMemory()) {
            if (key == "parking.memory.IDList") {
                for (const auto& item : *getParkingMemory()) {
                    values.push_back(item.first->getID());
                }
            } else if (key == "parking.memory.score") {
                for (const auto& item : *getParkingMemory()) {
                    values.push_back(item.second.score);
                }
            } else if (key == "parking.memory.blockedAtTime") {
                for (const auto& item : *getParkingMemory()) {
                    values.push_back(toString(STEPS2TIME(item.second.blockedAtTime)));
                }
            } else if (key == "parking.memory.blockedAtTimeLocal") {
                for (const auto& item : *getParkingMemory()) {
                    values.push_back(toString(STEPS2TIME(item.second.blockedAtTimeLocal)));
                }
            } else {
                error = TLF("Unsupported parking parameter '%' for vehicle '%'.", key, getID());
            }
        }
        return toString(values);
    } else {
        // default: custom user parameter
        return getParameter().getParameter(key, "");
    }
}


void
MSBaseVehicle::rememberBlockedParkingArea(const MSStoppingPlace* pa, bool local) {
    if (myParkingMemory == nullptr) {
        myParkingMemory = new StoppingPlaceMemory();
    }
    myParkingMemory->rememberBlockedStoppingPlace(pa, local);
}


void
MSBaseVehicle::resetParkingAreaScores() {
    if (myParkingMemory != nullptr) {
        myParkingMemory->resetStoppingPlaceScores();
    }
}


void
MSBaseVehicle::rememberChargingStationScore(const MSStoppingPlace* cs, const std::string& score) {
    if (myChargingMemory == nullptr) {
        myChargingMemory = new StoppingPlaceMemory();
    }
    myChargingMemory->rememberStoppingPlaceScore(cs, score);
}


void
MSBaseVehicle::resetChargingStationScores() {
    if (myChargingMemory != nullptr) {
        myChargingMemory->resetStoppingPlaceScores();
    }
}


void
MSBaseVehicle::rememberParkingAreaScore(const MSStoppingPlace* pa, const std::string& score) {
    if (myParkingMemory == nullptr) {
        myParkingMemory = new StoppingPlaceMemory();
    }
    myParkingMemory->rememberStoppingPlaceScore(pa, score);
}


SUMOTime
MSBaseVehicle::sawBlockedParkingArea(const MSStoppingPlace* pa, bool local) const {
    if (myParkingMemory == nullptr) {
        return -1;
    }
    return myParkingMemory->sawBlockedStoppingPlace(pa, local);
}


void MSBaseVehicle::rememberBlockedChargingStation(const MSStoppingPlace* cs, bool local) {
    if (myChargingMemory == nullptr) {
        myChargingMemory = new StoppingPlaceMemory();
    }
    myChargingMemory->rememberBlockedStoppingPlace(cs, local);
}


SUMOTime
MSBaseVehicle::sawBlockedChargingStation(const MSStoppingPlace* cs, bool local) const {
    if (myChargingMemory == nullptr) {
        return -1;
    }
    return myChargingMemory->sawBlockedStoppingPlace(cs, local);
}


#ifdef _DEBUG
void
MSBaseVehicle::initMoveReminderOutput(const OptionsCont& oc) {
    if (oc.isSet("movereminder-output.vehicles")) {
        const std::vector<std::string> vehicles = oc.getStringVector("movereminder-output.vehicles");
        myShallTraceMoveReminders.insert(vehicles.begin(), vehicles.end());
    }
}


void
MSBaseVehicle::traceMoveReminder(const std::string& type, MSMoveReminder* rem, double pos, bool keep) const {
    OutputDevice& od = OutputDevice::getDeviceByOption("movereminder-output");
    od.openTag("movereminder");
    od.writeAttr(SUMO_ATTR_TIME, STEPS2TIME(MSNet::getInstance()->getCurrentTimeStep()));
    od.writeAttr("veh", getID());
    od.writeAttr(SUMO_ATTR_ID, rem->getDescription());
    od.writeAttr("type", type);
    od.writeAttr("pos", toString(pos));
    od.writeAttr("keep", toString(keep));
    od.closeTag();
}
#endif


/****************************************************************************/
