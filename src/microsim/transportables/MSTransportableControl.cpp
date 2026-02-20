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
/// @file    MSTransportableControl.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @date    Mon, 9 Jul 2001
///
// Stores all persons in the net and handles their waiting for cars.
/****************************************************************************/
#include <config.h>

#include <vector>
#include <algorithm>
#include <utils/iodevices/OutputDevice.h>
#include <utils/iodevices/OutputDevice_String.h>
#include <utils/options/OptionsCont.h>
#include <microsim/transportables/MSPerson.h>
#include <microsim/transportables/MSStageDriving.h>
#include <microsim/transportables/MSPModel_NonInteracting.h>
#ifdef JPS_VERSION
#include <microsim/transportables/MSPModel_JuPedSim.h>
#endif
#include <microsim/transportables/MSPModel_Striping.h>
#include <microsim/transportables/MSTransportableControl.h>
#include <microsim/devices/MSDevice_Vehroutes.h>
#include <microsim/devices/MSDevice_Taxi.h>
#include <microsim/MSNet.h>
#include <microsim/MSEdge.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSStoppingPlace.h>


// ===========================================================================
// method definitions
// ===========================================================================
MSTransportableControl::MSTransportableControl(const bool isPerson):
    myLoadedNumber(0),
    myDiscardedNumber(0),
    myRunningNumber(0),
    myJammedNumber(0),
    myWaitingForDepartureNumber(0),
    myWaitingForVehicleNumber(0),
    myWaitingUntilNumber(0),
    myAccessNumber(0),
    myEndedNumber(0),
    myArrivedNumber(0),
    myTeleportsAbortWait(0),
    myTeleportsWrongDest(0),
    myHaveNewWaiting(false) {
    const OptionsCont& oc = OptionsCont::getOptions();
    MSNet* const net = MSNet::getInstance();
    myMovementModel = myNonInteractingModel = new MSPModel_NonInteracting(oc, net);
    if (isPerson) {
        const std::string& model = oc.getString("pedestrian.model");
        if (model == "striping") {
            myMovementModel = new MSPModel_Striping(oc, net);
#ifdef JPS_VERSION
        } else if (model == "jupedsim") {
            myMovementModel = new MSPModel_JuPedSim(oc, net);
#endif
        } else if (model != "nonInteracting") {
            delete myNonInteractingModel;
            throw ProcessError(TLF("Unknown pedestrian model '%'", model));
        }
    }
    if (oc.isSet("vehroute-output")) {
        myRouteInfos.routeOut = &OutputDevice::getDeviceByOption("vehroute-output");
    }
    if (oc.isSet("personroute-output")) {
        OutputDevice::createDeviceByOption("personroute-output", "routes", "routes_file.xsd");
        myRouteInfos.routeOut = &OutputDevice::getDeviceByOption("personroute-output");
    }
    if (oc.isSet("personinfo-output")) {
        OutputDevice::createDeviceByOption("personinfo-output", "tripinfos", "tripinfo_file.xsd");
    }
    myAbortWaitingTimeout = string2time(oc.getString("time-to-teleport.ride"));
    myMaxTransportableNumber = isPerson ? oc.getInt("max-num-persons") : -1;
}


MSTransportableControl::~MSTransportableControl() {
    clearState();
    if (myMovementModel != myNonInteractingModel) {
        delete myMovementModel;
    }
    delete myNonInteractingModel;
}


bool
MSTransportableControl::add(MSTransportable* transportable) {
    const SUMOVehicleParameter& param = transportable->getParameter();
    if (myTransportables.find(param.id) == myTransportables.end()) {
        myTransportables[param.id] = transportable;
        const SUMOTime step = param.depart % DELTA_T == 0 ? param.depart : (param.depart / DELTA_T + 1) * DELTA_T;
        myWaiting4Departure[step].push_back(transportable);
        myLoadedNumber++;
        myWaitingForDepartureNumber++;
        return true;
    }
    return false;
}


void
MSTransportableControl::fixLoadCount(const MSTransportable* transportable) {
    myLoadedNumber--;
    if (transportable->hasDeparted()) {
        const SUMOVehicleParameter& param = transportable->getParameter();
        const SUMOTime step = param.depart % DELTA_T == 0 ? param.depart : (param.depart / DELTA_T + 1) * DELTA_T;
        TransportableVector& waiting = myWaiting4Departure[step];
        auto it = std::find(waiting.begin(), waiting.end(), transportable);
        if (it != waiting.end()) {
            waiting.erase(it);
            if (waiting.size() == 0) {
                myWaiting4Departure.erase(step);
            }
        }
    }
}


MSTransportable*
MSTransportableControl::get(const std::string& id) const {
    std::map<std::string, MSTransportable*>::const_iterator i = myTransportables.find(id);
    if (i == myTransportables.end()) {
        return nullptr;
    }
    return (*i).second;
}


void
MSTransportableControl::erase(MSTransportable* transportable) {
    const OptionsCont& oc = OptionsCont::getOptions();
    if (oc.isSet("personinfo-output")) {
        transportable->tripInfoOutput(OutputDevice::getDeviceByOption("personinfo-output"));
    } else if (oc.isSet("tripinfo-output")) {
        transportable->tripInfoOutput(OutputDevice::getDeviceByOption("tripinfo-output"));
    } else if (oc.getBool("duration-log.statistics")) {
        // collecting statistics is a sideffect
        OutputDevice_String dev;
        transportable->tripInfoOutput(dev);
    }
    if (oc.isSet("vehroute-output") || oc.isSet("personroute-output")) {
        if (transportable->hasArrived() || oc.getBool("vehroute-output.write-unfinished")) {
            // @todo: adapt this if we ever introduce person-device.vehroute
            if (transportable->getBoolParam("has.vehroute.person-device", false, true)) {
                if (oc.getBool("vehroute-output.sorted")) {
                    const SUMOTime departure = oc.getBool("vehroute-output.intended-depart") ? transportable->getParameter().depart : transportable->getDeparture();
                    OutputDevice_String od(1);
                    transportable->routeOutput(od, oc.getBool("vehroute-output.route-length"));
                    MSDevice_Vehroutes::writeSortedOutput(&myRouteInfos,
                                                          departure, transportable->getID(), od.getString());
                } else {
                    transportable->routeOutput(*myRouteInfos.routeOut, oc.getBool("vehroute-output.route-length"));
                }
            }
        }
    }
    const std::map<std::string, MSTransportable*>::iterator i = myTransportables.find(transportable->getID());
    if (i != myTransportables.end()) {
        if (i->second->hasDeparted()) {
            myRunningNumber--;
        }
        myEndedNumber++;
        MSNet::getInstance()->informTransportableStateListener(transportable,
                transportable->isPerson() ? MSNet::TransportableState::PERSON_ARRIVED : MSNet::TransportableState::CONTAINER_ARRIVED);
        delete i->second;
        myTransportables.erase(i);
    }
}


void
MSTransportableControl::eraseAll() {
    // avoid taxi device having dangling references to erased customers
    MSDevice_Taxi::allCustomersErased();
    while (loadedBegin() != loadedEnd()) {
        erase(loadedBegin()->second);
    }
}


void
MSTransportableControl::setWaitEnd(const SUMOTime time, MSTransportable* transportable) {
    const SUMOTime step = time % DELTA_T == 0 ? time : (time / DELTA_T + 1) * DELTA_T;
    // avoid double registration
    const TransportableVector& transportables = myWaitingUntil[step];
    if (std::find(transportables.begin(), transportables.end(), transportable) == transportables.end()) {
        myWaitingUntil[step].push_back(transportable);
        myWaitingUntilNumber++;
    }
}


void
MSTransportableControl::checkWaiting(MSNet* net, const SUMOTime time) {
    myHaveNewWaiting = false;
    while (myWaiting4Departure.find(time) != myWaiting4Departure.end()) {
        TransportableVector& transportables = myWaiting4Departure[time];
        // we cannot use an iterator here because there might be additions to the vector while proceeding
        for (auto it = transportables.begin(); it != transportables.end();) {
            MSTransportable* t = *it;
            if (myMaxTransportableNumber > 0 && myRunningNumber >= myMaxTransportableNumber) {
                TransportableVector& nextStep = myWaiting4Departure[time + DELTA_T];
                nextStep.insert(nextStep.begin(), transportables.begin(), transportables.end());
                transportables.clear();
                break;
            } else if (myMaxTransportableNumber == 0) {
                erase(t);
                it = transportables.erase(it);
                continue;
            }
            it = transportables.erase(it);
            myWaitingForDepartureNumber--;
            const bool isPerson = t->isPerson();
            t->setDeparted(time);
            if (t->proceed(net, time)) {
                myRunningNumber++;
                MSNet::getInstance()->informTransportableStateListener(t,
                        isPerson ? MSNet::TransportableState::PERSON_DEPARTED : MSNet::TransportableState::CONTAINER_DEPARTED);
                const OptionsCont& oc = OptionsCont::getOptions();
                if (oc.getBool("vehroute-output.sorted")) {
                    const SUMOTime departure = oc.getBool("vehroute-output.intended-depart") ? t->getParameter().depart : time;
                    if (oc.isSet("personroute-output")) {
                        myRouteInfos.departureCounts[departure]++;
                    } else {
                        MSDevice_Vehroutes::registerTransportableDepart(departure);
                    }
                }
            } else {
                erase(t);
            }
        }
        myWaiting4Departure.erase(time);
    }
    while (myWaitingUntil.find(time) != myWaitingUntil.end()) {
        // make a copy because 0-duration stops might modify the vector
        const TransportableVector transportables = myWaitingUntil[time];
        myWaitingUntil.erase(time);
        for (MSTransportable* t : transportables) {
            myWaitingUntilNumber--;
            if (!t->proceed(net, time)) {
                erase(t);
            }
        }
    }
}


void
MSTransportableControl::forceDeparture() {
    myRunningNumber++;
}


void
MSTransportableControl::addWaiting(const MSEdge* const edge, MSTransportable* transportable) {
    myWaiting4Vehicle[edge].push_back(transportable);
    myWaitingForVehicleNumber++;
    myHaveNewWaiting = true;
    if (myAbortWaitingTimeout >= 0) {
        transportable->setAbortWaiting(myAbortWaitingTimeout);
    }
}


bool
MSTransportableControl::hasAnyWaiting(const MSEdge* edge, SUMOVehicle* vehicle) const {
    const auto wait = myWaiting4Vehicle.find(edge);
    if (wait != myWaiting4Vehicle.end()) {
        for (const MSTransportable* t : wait->second) {
            if (t->isWaitingFor(vehicle)
                    && vehicle->allowsBoarding(t)
                    && vehicle->isStoppedInRange(t->getEdgePos(), MSGlobals::gStopTolerance, true)) {
                return true;
            }
        }
    }
    return false;
}


bool
MSTransportableControl::loadAnyWaiting(const MSEdge* edge, SUMOVehicle* vehicle, SUMOTime& timeToLoadNext, SUMOTime& stopDuration, MSTransportable* const force) {
    bool ret = false;
    const auto wait = myWaiting4Vehicle.find(edge);
    if (wait != myWaiting4Vehicle.end()) {
        const SUMOTime currentTime = SIMSTEP;
        TransportableVector& transportables = wait->second;
        for (TransportableVector::iterator i = transportables.begin(); i != transportables.end();) {
            MSTransportable* const t = *i;
            if (t->isWaitingFor(vehicle) && (t == force ||
                                             (vehicle->allowsBoarding(t)
                                              && timeToLoadNext - DELTA_T <= currentTime
                                              && vehicle->isStoppedInRange(t->getEdgePos(), MSGlobals::gStopTolerance)))) {
                edge->removeTransportable(t);
                vehicle->addTransportable(t);
                if (myAbortWaitingTimeout >= 0) {
                    t->setAbortWaiting(-1);
                }
                if (timeToLoadNext >= 0) { // meso does not have loading times
                    const SUMOTime loadingDuration = (SUMOTime)((double)vehicle->getVehicleType().getLoadingDuration(t->isPerson()) * t->getVehicleType().getBoardingFactor());
                    //update the time point at which the next transportable can be loaded on the vehicle
                    if (timeToLoadNext > currentTime - DELTA_T) {
                        timeToLoadNext += loadingDuration;
                    } else {
                        timeToLoadNext = currentTime + loadingDuration;
                    }
                }

                static_cast<MSStageDriving*>(t->getCurrentStage())->setVehicle(vehicle);
                if (t->getCurrentStage()->getOriginStop() != nullptr) {
                    t->getCurrentStage()->getOriginStop()->removeTransportable(*i);
                }
                i = transportables.erase(i);
                myWaitingForVehicleNumber--;
                ret = true;
            } else {
                ++i;
            }
        }
        if (transportables.empty()) {
            myWaiting4Vehicle.erase(wait);
        }
        if (ret && timeToLoadNext >= 0) {
            //if the time a transportable needs to get loaded on the vehicle extends the duration of the stop of the vehicle extend
            //the duration by setting it to the loading duration of the transportable
            stopDuration = MAX2(stopDuration, timeToLoadNext - currentTime);
        }
    }
    return ret;
}


bool
MSTransportableControl::hasTransportables() const {
    return !myTransportables.empty();
}


bool
MSTransportableControl::hasNonWaiting() const {
    return !myWaiting4Departure.empty() || getMovingNumber() > 0 || myWaitingUntilNumber > 0 || myHaveNewWaiting;
}


int
MSTransportableControl::getActiveCount() {
    return (int)myWaiting4Departure.size() + myRunningNumber - myWaitingForVehicleNumber;
}


int
MSTransportableControl::getMovingNumber() const {
    return myMovementModel->getActiveNumber() + myAccessNumber;
}


int
MSTransportableControl::getRidingNumber() const {
    return myRunningNumber - myWaitingUntilNumber - myWaitingForVehicleNumber - getMovingNumber();
}

int
MSTransportableControl::getDepartedNumber() const {
    return myLoadedNumber - myWaitingForDepartureNumber - myDiscardedNumber;
}

void
MSTransportableControl::abortAnyWaitingForVehicle() {
    for (const auto& it : myWaiting4Vehicle) {
        const MSEdge* edge = it.first;
        for (MSTransportable* const p : it.second) {
            edge->removeTransportable(p);
            MSStageDriving* stage = dynamic_cast<MSStageDriving*>(p->getCurrentStage());
            const std::string waitDescription = stage == nullptr ? "waiting" : stage->getWaitingDescription();
            WRITE_WARNING(p->getObjectType() + " '" + p->getID() + "' aborted " + waitDescription + ".");
            if (myAbortWaitingTimeout >= 0) {
                p->setAbortWaiting(-1);
            }
            erase(p);
        }
    }
    myWaiting4Vehicle.clear();
    myWaitingForVehicleNumber = 0;
}

void
MSTransportableControl::abortWaitingForVehicle(MSTransportable* t) {
    const MSEdge* edge = t->getEdge();
    auto it = myWaiting4Vehicle.find(edge);
    if (it != myWaiting4Vehicle.end()) {
        TransportableVector& waiting = it->second;
        auto it2 = std::find(waiting.begin(), waiting.end(), t);
        if (it2 != waiting.end()) {
            if (myAbortWaitingTimeout >= 0) {
                (*it2)->setAbortWaiting(-1);
            }
            waiting.erase(it2);
            myWaitingForVehicleNumber--;
        }
    }
}

void
MSTransportableControl::abortWaiting(MSTransportable* t) {
    for (std::map<SUMOTime, TransportableVector>::iterator it = myWaiting4Departure.begin(); it != myWaiting4Departure.end(); ++it) {
        TransportableVector& ts = it->second;
        TransportableVector::iterator it2 = std::find(ts.begin(), ts.end(), t);
        if (it2 != ts.end()) {
            ts.erase(it2);
        }
    }
    for (std::map<SUMOTime, TransportableVector>::iterator it = myWaitingUntil.begin(); it != myWaitingUntil.end(); ++it) {
        TransportableVector& ts = it->second;
        TransportableVector::iterator it2 = std::find(ts.begin(), ts.end(), t);
        if (it2 != ts.end()) {
            ts.erase(it2);
        }
    }
}


MSTransportable*
MSTransportableControl::buildPerson(const SUMOVehicleParameter* pars, MSVehicleType* vtype, MSTransportable::MSTransportablePlan* plan,
                                    SumoRNG* rng) const {
    const double speedFactor = vtype->computeChosenSpeedDeviation(rng);
    return new MSPerson(pars, vtype, plan, speedFactor);
}


MSTransportable*
MSTransportableControl::buildContainer(const SUMOVehicleParameter* pars, MSVehicleType* vtype, MSTransportable::MSTransportablePlan* plan) const {
    return new MSTransportable(pars, vtype, plan, false);
}


void
MSTransportableControl::saveState(OutputDevice& out) {
    std::ostringstream oss;
    oss << myRunningNumber << " " << myLoadedNumber << " " << myEndedNumber << " " << myWaitingForDepartureNumber << " " << myArrivedNumber << " " << myDiscardedNumber;
    oss << " " << myJammedNumber << " " << myWaitingForVehicleNumber << " " << myWaitingUntilNumber << " " << myHaveNewWaiting;
    out.writeAttr(SUMO_ATTR_STATE, oss.str());
    for (const auto& it : myTransportables) {
        it.second->saveState(out);
    }
}


void
MSTransportableControl::loadState(const std::string& state) {
    std::istringstream iss(state);
    iss >> myRunningNumber >> myLoadedNumber >> myEndedNumber >> myWaitingForDepartureNumber >> myArrivedNumber >> myDiscardedNumber;
    iss >> myJammedNumber >> myWaitingForVehicleNumber >> myWaitingUntilNumber >> myHaveNewWaiting;
}

void
MSTransportableControl::clearState() {
    for (std::map<std::string, MSTransportable*>::iterator i = myTransportables.begin(); i != myTransportables.end(); ++i) {
        delete (*i).second;
    }
    myTransportables.clear();
    myWaiting4Vehicle.clear();
    myWaiting4Departure.clear();
    myWaitingUntil.clear();
    myLoadedNumber = 0;
    myDiscardedNumber = 0;
    myRunningNumber = 0;
    myJammedNumber = 0;
    myWaitingForDepartureNumber = 0;
    myWaitingForVehicleNumber = 0;
    myWaitingUntilNumber = 0;
    myEndedNumber = 0;
    myArrivedNumber = 0;
    myHaveNewWaiting = false;
    if (myMovementModel != myNonInteractingModel) {
        myMovementModel->clearState();
    }
    myNonInteractingModel->clearState();
}

/****************************************************************************/
