/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2001-2025 German Aerospace Center (DLR) and others.
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
/// @file    MSVehicleControl.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Wed, 10. Dec 2003
///
// The class responsible for building and deletion of vehicles
/****************************************************************************/
#include <config.h>

#include "MSVehicleControl.h"
#include "MSVehicle.h"
#include "MSLane.h"
#include "MSEdge.h"
#include "MSNet.h"
#include "MSRouteHandler.h"
#include "MSEventControl.h"
#include "MSStop.h"
#include <microsim/devices/MSVehicleDevice.h>
#include <microsim/devices/MSDevice_Tripinfo.h>
#include <utils/common/FileHelpers.h>
#include <utils/common/Named.h>
#include <utils/common/RGBColor.h>
#include <utils/vehicle/SUMOVTypeParameter.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/options/OptionsCont.h>
#include <utils/router/IntermodalRouter.h>


// ===========================================================================
// member method definitions
// ===========================================================================
MSVehicleControl::MSVehicleControl() :
    myLoadedVehNo(0),
    myRunningVehNo(0),
    myEndedVehNo(0),
    myDiscarded(0),
    myCollisions(0),
    myTeleportsCollision(0),
    myTeleportsJam(0),
    myTeleportsYield(0),
    myTeleportsWrongLane(0),
    myEmergencyStops(0),
    myEmergencyBrakingCount(0),
    myStoppedVehicles(0),
    myTotalDepartureDelay(0),
    myTotalTravelTime(0),
    myWaitingForTransportable(0),
    myMaxSpeedFactor(1),
    myMinDeceleration(SUMOVTypeParameter::getDefaultDecel(SVC_IGNORING)),
    myMinDecelerationRail(SUMOVTypeParameter::getDefaultDecel(SVC_RAIL)),
    myPendingRemovals(MSGlobals::gNumSimThreads > 1) {

    initDefaultTypes();
    myScale = OptionsCont::getOptions().getFloat("scale");
    myKeepTime = string2time(OptionsCont::getOptions().getString("keep-after-arrival"));
}


MSVehicleControl::~MSVehicleControl() {
    clearState(false);
}


void
MSVehicleControl::initDefaultTypes() {
    SUMOVTypeParameter defType(DEFAULT_VTYPE_ID, SVC_PASSENGER);
    myVTypeDict[DEFAULT_VTYPE_ID] = MSVehicleType::build(defType);

    SUMOVTypeParameter defPedType(DEFAULT_PEDTYPE_ID, SVC_PEDESTRIAN);
    defPedType.parametersSet |= VTYPEPARS_VEHICLECLASS_SET;
    myVTypeDict[DEFAULT_PEDTYPE_ID] = MSVehicleType::build(defPedType);

    SUMOVTypeParameter defBikeType(DEFAULT_BIKETYPE_ID, SVC_BICYCLE);
    defBikeType.parametersSet |= VTYPEPARS_VEHICLECLASS_SET;
    myVTypeDict[DEFAULT_BIKETYPE_ID] = MSVehicleType::build(defBikeType);

    SUMOVTypeParameter defTaxiType(DEFAULT_TAXITYPE_ID, SVC_TAXI);
    defTaxiType.parametersSet |= VTYPEPARS_VEHICLECLASS_SET;
    myVTypeDict[DEFAULT_TAXITYPE_ID] = MSVehicleType::build(defTaxiType);

    SUMOVTypeParameter defRailType(DEFAULT_RAILTYPE_ID, SVC_RAIL);
    defRailType.parametersSet |= VTYPEPARS_VEHICLECLASS_SET;
    myVTypeDict[DEFAULT_RAILTYPE_ID] = MSVehicleType::build(defRailType);

    SUMOVTypeParameter defContainerType(DEFAULT_CONTAINERTYPE_ID, SVC_CONTAINER);
    defContainerType.parametersSet |= VTYPEPARS_VEHICLECLASS_SET;
    myVTypeDict[DEFAULT_CONTAINERTYPE_ID] = MSVehicleType::build(defContainerType);

    myReplaceableDefaultVTypes = DEFAULT_VTYPES;
}


SUMOVehicle*
MSVehicleControl::buildVehicle(SUMOVehicleParameter* defs,
                               ConstMSRoutePtr route, MSVehicleType* type,
                               const bool ignoreStopErrors, const VehicleDefinitionSource source, bool addRouteStops) {
    const double speedFactor = (source == VehicleDefinitionSource::STATE ? 1 :
            type->computeChosenSpeedDeviation(source == VehicleDefinitionSource::ROUTEFILE ? MSRouteHandler::getParsingRNG() : nullptr));
    MSVehicle* built = new MSVehicle(defs, route, type, speedFactor);
    initVehicle(built, ignoreStopErrors, addRouteStops, source);
    return built;
}


void
MSVehicleControl::initVehicle(MSBaseVehicle* built, const bool ignoreStopErrors, bool addRouteStops, const VehicleDefinitionSource source) {
    myLoadedVehNo++;
    try {
        built->initDevices();
        if (source != VehicleDefinitionSource::STATE) {
            built->addStops(ignoreStopErrors, nullptr, addRouteStops);
        }
    } catch (ProcessError&) {
        delete built;
        throw;
    }
    MSNet::getInstance()->informVehicleStateListener(built, MSNet::VehicleState::BUILT);
}


void
MSVehicleControl::scheduleVehicleRemoval(SUMOVehicle* veh, bool checkDuplicate) {
    assert(myRunningVehNo > 0);
    if (!checkDuplicate || !isPendingRemoval(veh)) {
        myPendingRemovals.push_back(veh);
    }
}


bool
MSVehicleControl::isPendingRemoval(SUMOVehicle* veh) {
#ifdef HAVE_FOX
    return myPendingRemovals.contains(veh);
#else
    return std::find(myPendingRemovals.begin(), myPendingRemovals.end(), veh) == myPendingRemovals.end();
#endif
}


void
MSVehicleControl::removePending() {
    OutputDevice* const tripinfoOut = OptionsCont::getOptions().isSet("tripinfo-output") ? &OutputDevice::getDeviceByOption("tripinfo-output") : nullptr;
#ifdef HAVE_FOX
    std::vector<SUMOVehicle*>& vehs = myPendingRemovals.getContainer();
#else
    std::vector<SUMOVehicle*>& vehs = myPendingRemovals;
#endif
    std::sort(vehs.begin(), vehs.end(), ComparatorNumericalIdLess());
    for (SUMOVehicle* const veh : vehs) {
        myTotalTravelTime += STEPS2TIME(MSNet::getInstance()->getCurrentTimeStep() - veh->getDeparture());
        myRunningVehNo--;
        MSNet::getInstance()->informVehicleStateListener(veh, MSNet::VehicleState::ARRIVED);
        // vehicle is equipped with tripinfo device (not all vehicles are)
        const bool hasTripinfo = veh->getDevice(typeid(MSDevice_Tripinfo)) != nullptr;
        for (MSVehicleDevice* const dev : veh->getDevices()) {
            dev->generateOutput(hasTripinfo ? tripinfoOut : nullptr);
        }
        if (tripinfoOut != nullptr && hasTripinfo) {
            // close tag after tripinfo (possibly including emissions from another device) have been written
            tripinfoOut->closeTag();
        }
        if (myKeepTime == 0) {
            deleteVehicle(veh);
        } else {
            deleteKeptVehicle(veh);
        }
    }
    vehs.clear();
    if (tripinfoOut != nullptr) {
        // there seem to be people who think reading an unfinished xml is a good idea ;-)
        tripinfoOut->flush();
    }
#ifdef HAVE_FOX
    myPendingRemovals.unlock();
#endif
}


void
MSVehicleControl::deleteKeptVehicle(SUMOVehicle* veh) {
    myEndedVehNo++;
    MSNet::getInstance()->getEndOfTimestepEvents()->addEvent(new DeleteKeptVehicle(veh), SIMSTEP + myKeepTime);
}

void
MSVehicleControl::vehicleDeparted(const SUMOVehicle& v) {
    ++myRunningVehNo;
    myTotalDepartureDelay += STEPS2TIME(v.getDeparture() - STEPFLOOR(v.getParameter().depart));
    MSNet::getInstance()->informVehicleStateListener(&v, MSNet::VehicleState::DEPARTED);
    myMaxSpeedFactor = MAX2(myMaxSpeedFactor, v.getChosenSpeedFactor());
    if ((v.getVClass() & (SVC_PEDESTRIAN | SVC_NON_ROAD)) == 0) {
        // only  worry about deceleration of road users
        myMinDeceleration = MIN2(myMinDeceleration, v.getVehicleType().getCarFollowModel().getMaxDecel());
    } else if ((v.getVClass() & SVC_RAIL_CLASSES) != 0) {
        myMinDecelerationRail = MIN2(myMinDecelerationRail, v.getVehicleType().getCarFollowModel().getMaxDecel());
    }
}


void
MSVehicleControl::setState(int runningVehNo, int loadedVehNo, int endedVehNo, double totalDepartureDelay, double totalTravelTime) {
    myRunningVehNo = runningVehNo;
    myLoadedVehNo = loadedVehNo;
    myEndedVehNo = endedVehNo;
    myTotalDepartureDelay = totalDepartureDelay;
    myTotalTravelTime = totalTravelTime;
}


void
MSVehicleControl::saveState(OutputDevice& out) {
    out.openTag(SUMO_TAG_DELAY);
    out.writeAttr(SUMO_ATTR_NUMBER, myRunningVehNo);
    out.writeAttr(SUMO_ATTR_BEGIN, myLoadedVehNo);
    out.writeAttr(SUMO_ATTR_END, myEndedVehNo);
    out.writeAttr(SUMO_ATTR_DEPART, myTotalDepartureDelay);
    out.writeAttr(SUMO_ATTR_TIME, myTotalTravelTime).closeTag();
    // save vehicle types
    for (const auto& item : myVTypeDict) {
        if (myReplaceableDefaultVTypes.count(item.first) == 0) {
            item.second->getParameter().write(out);
        }
    }
    for (const auto& item : myVTypeDistDict) {
        out.openTag(SUMO_TAG_VTYPE_DISTRIBUTION).writeAttr(SUMO_ATTR_ID, item.first);
        out.writeAttr(SUMO_ATTR_VTYPES, item.second->getVals());
        out.writeAttr(SUMO_ATTR_PROBS, item.second->getProbs());
        out.closeTag();
    }
    std::vector<SUMOVehicle*> sortedVehs;
    for (const auto& item : myVehicleDict) {
        sortedVehs.push_back(item.second);
    }
    std::sort(sortedVehs.begin(), sortedVehs.end(), ComparatorNumericalIdLess());
    for (SUMOVehicle* veh : sortedVehs) {
        veh->saveState(out);
    }
}


void
MSVehicleControl::clearState(const bool reinit) {
    for (const auto& item : myVehicleDict) {
        delete item.second;
    }
    myVehicleDict.clear();
    // delete vehicle type distributions
    for (const auto& item : myVTypeDistDict) {
        delete item.second;
    }
    myVTypeDistDict.clear();
    // delete vehicle types
    for (const auto& item : myVTypeDict) {
        delete item.second;
    }
    myVTypeDict.clear();
    myPendingRemovals.clear(); // could be leftovers from MSVehicleTransfer::checkInsertions (teleport beyond arrival)
    if (reinit) {
        initDefaultTypes();
    }
    myLoadedVehNo = 0;
    myRunningVehNo = 0;
    myEndedVehNo = 0;
    myDiscarded = 0;
    myCollisions = 0;
    myTeleportsCollision = 0;
    myTeleportsJam = 0;
    myTeleportsYield = 0;
    myTeleportsWrongLane = 0;
    myEmergencyStops = 0;
    myEmergencyBrakingCount = 0;
    myStoppedVehicles = 0;
    myTotalDepartureDelay = 0;
    myTotalTravelTime = 0;
}


bool
MSVehicleControl::addVehicle(const std::string& id, SUMOVehicle* v) {
    VehicleDictType::iterator it = myVehicleDict.find(id);
    if (it == myVehicleDict.end()) {
        // id not in myVehicleDict.
        myVehicleDict[id] = v;
        handleTriggeredDepart(v, true);
        const SUMOVehicleParameter& pars = v->getParameter();
        if (v->getVClass() != SVC_TAXI && pars.line != "" && pars.repetitionNumber < 0) {
            myPTVehicles.push_back(v);
        }
        return true;
    }
    return false;
}


void
MSVehicleControl::handleTriggeredDepart(SUMOVehicle* v, bool add) {
    const SUMOVehicleParameter& pars = v->getParameter();
    if (pars.departProcedure == DepartDefinition::TRIGGERED || pars.departProcedure == DepartDefinition::CONTAINER_TRIGGERED || pars.departProcedure == DepartDefinition::SPLIT) {
        const MSEdge* const firstEdge = v->getRoute().getEdges()[pars.departEdge];
        if (add) {
            if (!MSGlobals::gUseMesoSim) {
                // position will be checked against person position later
                static_cast<MSVehicle*>(v)->setTentativeLaneAndPosition(nullptr, v->getParameter().departPos);
            }
            if (firstEdge->isTazConnector()) {
                for (MSEdge* out : firstEdge->getSuccessors()) {
                    out->addWaiting(v);
                }
            } else {
                firstEdge->addWaiting(v);
            }
            registerOneWaiting();
        } else {
            if (firstEdge->isTazConnector()) {
                for (MSEdge* out : firstEdge->getSuccessors()) {
                    out->removeWaiting(v);
                }
            } else {
                firstEdge->removeWaiting(v);
            }
            unregisterOneWaiting();
        }
    }
}


SUMOVehicle*
MSVehicleControl::getVehicle(const std::string& id) const {
    VehicleDictType::const_iterator it = myVehicleDict.find(id);
    if (it == myVehicleDict.end()) {
        return nullptr;
    }
    return it->second;
}


void
MSVehicleControl::deleteVehicle(SUMOVehicle* veh, bool discard, bool wasKept) {
    if (!wasKept) {
        myEndedVehNo++;
        if (discard) {
            myDiscarded++;
        }
    }
    if (veh != nullptr) {
        myVehicleDict.erase(veh->getID());
    }
    auto ptVehIt = std::find(myPTVehicles.begin(), myPTVehicles.end(), veh);
    if (ptVehIt != myPTVehicles.end()) {
        myPTVehicles.erase(ptVehIt);
    }
    delete veh;
}


bool
MSVehicleControl::checkVType(const std::string& id) {
    if (myReplaceableDefaultVTypes.erase(id) > 0) {
        delete myVTypeDict[id];
        myVTypeDict.erase(myVTypeDict.find(id));
    } else {
        if (myVTypeDict.find(id) != myVTypeDict.end() || myVTypeDistDict.find(id) != myVTypeDistDict.end()) {
            return false;
        }
    }
    return true;
}


bool
MSVehicleControl::addVType(MSVehicleType* vehType) {
    if (checkVType(vehType->getID())) {
        myVTypeDict[vehType->getID()] = vehType;
        return true;
    }
    return false;
}


void
MSVehicleControl::removeVType(const MSVehicleType* vehType) {
    assert(vehType != nullptr);
    assert(myVTypeDict.find(vehType->getID()) != myVTypeDict.end());
    myVTypeDict.erase(vehType->getID());
    if (myVTypeToDist.find(vehType->getID()) != myVTypeToDist.end()) {
        myVTypeToDist.erase(vehType->getID());
    }
    delete vehType;
}


bool
MSVehicleControl::addVTypeDistribution(const std::string& id, RandomDistributor<MSVehicleType*>* vehTypeDistribution) {
    if (checkVType(id)) {
        myVTypeDistDict[id] = vehTypeDistribution;
        std::vector<MSVehicleType*> vehTypes = vehTypeDistribution->getVals();
        for (auto vehType : vehTypes) {
            if (myVTypeToDist.find(vehType->getID()) != myVTypeToDist.end()) {
                myVTypeToDist[vehType->getID()].insert(id);
            } else {
                myVTypeToDist[vehType->getID()] = { id };
            }
        }
        return true;
    }
    return false;
}


bool
MSVehicleControl::hasVType(const std::string& id) const {
    return myVTypeDict.count(id) > 0 || myVTypeDistDict.count(id) > 0;
}


bool
MSVehicleControl::hasVTypeDistribution(const std::string& id) const {
    return myVTypeDistDict.count(id) > 0;
}


MSVehicleType*
MSVehicleControl::getVType(const std::string& id, SumoRNG* rng, bool readOnly) {
    VTypeDictType::iterator it = myVTypeDict.find(id);
    if (it == myVTypeDict.end()) {
        VTypeDistDictType::iterator it2 = myVTypeDistDict.find(id);
        if (it2 == myVTypeDistDict.end()) {
            return nullptr;
        }
        return it2->second->get(rng);
    }
    if (!readOnly && myReplaceableDefaultVTypes.erase(id) > 0) {
        it->second->check();
    }
    return it->second;
}


void
MSVehicleControl::insertVTypeIDs(std::vector<std::string>& into) const {
    into.reserve(into.size() + myVTypeDict.size() + myVTypeDistDict.size());
    for (VTypeDictType::const_iterator i = myVTypeDict.begin(); i != myVTypeDict.end(); ++i) {
        into.push_back((*i).first);
    }
    for (VTypeDistDictType::const_iterator i = myVTypeDistDict.begin(); i != myVTypeDistDict.end(); ++i) {
        into.push_back((*i).first);
    }
}


const std::set<std::string>
MSVehicleControl::getVTypeDistributionMembership(const std::string& id) const {
    std::map<std::string, std::set<std::string>>::const_iterator it = myVTypeToDist.find(id);
    if (it == myVTypeToDist.end()) {
        return std::set<std::string>();
    }
    return it->second;
}


const RandomDistributor<MSVehicleType*>*
MSVehicleControl::getVTypeDistribution(const std::string& typeDistID) const {
    const auto it = myVTypeDistDict.find(typeDistID);
    if (it != myVTypeDistDict.end()) {
        return it->second;
    }
    return nullptr;
}


const std::vector<MSVehicleType*>
MSVehicleControl::getPedestrianTypes(void) const {
    std::vector<MSVehicleType*> pedestrianTypes;
    for (auto const& e : myVTypeDict)
        if (e.second->getVehicleClass() == SUMOVehicleClass::SVC_PEDESTRIAN) {
            pedestrianTypes.push_back(e.second);
        }
    return pedestrianTypes;
}


void
MSVehicleControl::abortWaiting() {
    for (VehicleDictType::iterator i = myVehicleDict.begin(); i != myVehicleDict.end(); ++i) {
        SUMOVehicle* veh = i->second;
        std::string waitReason;
        if (veh->isStoppedTriggered()) {
            const MSStop& stop = veh->getNextStop();
            if (stop.triggered) {
                waitReason = "for a person that will never come";
            } else if (stop.containerTriggered) {
                waitReason = "for a container that will never come";
            } else if (stop.joinTriggered) {
                if (stop.pars.join != "") {
                    waitReason = "to be joined to vehicle '" + stop.pars.join + "'";
                } else {
                    waitReason = "for a joining vehicle that will never come";
                }
            } else {
                waitReason = "for an unknown trigger";
            }
        } else if (!veh->hasDeparted()) {
            if (veh->getParameter().departProcedure == DepartDefinition::SPLIT) {
                waitReason = "for a train from which to split";
            } else if (veh->getParameter().departProcedure == DepartDefinition::TRIGGERED) {
                waitReason = "at insertion for a person that will never come";
            } else if (veh->getParameter().departProcedure == DepartDefinition::CONTAINER_TRIGGERED) {
                waitReason = "at insertion for a container that will never come";
            } else {
                waitReason = "for an unknown departure trigger";
            }
        } else {
            waitReason = "for an unknown reason";
        }
        WRITE_WARNINGF(TL("Vehicle '%' aborted waiting %."), i->first, waitReason);
    }
}


int
MSVehicleControl::getHaltingVehicleNo() const {
    int result = 0;
    for (MSVehicleControl::constVehIt it = loadedVehBegin(); it != loadedVehEnd(); ++it) {
        const SUMOVehicle* veh = it->second;
        if ((veh->isOnRoad() || veh->isRemoteControlled()) && veh->getSpeed() < SUMO_const_haltingSpeed)  {
            result++;
        }
    }
    return result;
}


std::pair<double, double>
MSVehicleControl::getVehicleMeanSpeeds() const {
    double speedSum = 0;
    double relSpeedSum = 0;
    int count = 0;
    for (MSVehicleControl::constVehIt it = loadedVehBegin(); it != loadedVehEnd(); ++it) {
        const SUMOVehicle* veh = it->second;
        if ((veh->isOnRoad() || veh->isRemoteControlled()) && !veh->isStopped()) {
            count++;
            speedSum += veh->getSpeed();
            relSpeedSum += veh->getEdge()->getSpeedLimit() > 0 ? veh->getSpeed() / veh->getEdge()->getSpeedLimit() : 0;
        }
    }
    if (count > 0) {
        return std::make_pair(speedSum / count, relSpeedSum / count);
    } else {
        return std::make_pair(-1, -1);
    }
}


int
MSVehicleControl::getQuota(double frac, int loaded) const {
    frac = frac < 0 ? myScale : frac;
    const int origLoaded = (loaded < 1
                            // the vehicle in question has already been loaded, hence  the '-1'
                            ? frac > 1. ? (int)(myLoadedVehNo / frac) : myLoadedVehNo - 1
                            // given transportable number reflects only previously loaded
                            : frac > 1. ? (int)(loaded / frac) : loaded);
    return getScalingQuota(frac, origLoaded);
}


int
MSVehicleControl::getTeleportCount() const {
    return myTeleportsCollision + myTeleportsJam + myTeleportsYield + myTeleportsWrongLane;
}


void
MSVehicleControl::adaptIntermodalRouter(MSTransportableRouter& router) const {
    for (const SUMOVehicle* const veh : myPTVehicles) {
        // add single vehicles with line attribute which are not part of a flow
        ConstMSRoutePtr const route = MSRoute::dictionary(veh->getParameter().routeid);
        router.getNetwork()->addSchedule(veh->getParameter(), route == nullptr ? nullptr : &route->getStops());
    }
}

// ===========================================================================
// MSVehicleControl::DeleteKeptVehicle method definitions
// ===========================================================================

SUMOTime
MSVehicleControl::DeleteKeptVehicle::execute(SUMOTime /*currentTime*/) {
    MSNet::getInstance()->getVehicleControl().deleteVehicle(myVehicle, false, true);
    return 0;
}

/****************************************************************************/
