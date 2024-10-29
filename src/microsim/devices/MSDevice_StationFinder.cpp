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
/// @file    MSDevice_StationFinder.cpp
/// @author  Michael Behrisch
/// @author  Mirko Barthauer
/// @date    2023-05-24
///
// A device which triggers rerouting to nearby charging stations
/****************************************************************************/
#include <config.h>

#include <microsim/MSEdge.h>
#include <microsim/MSEdgeWeightsStorage.h>
#include <microsim/MSEventControl.h>
#include <microsim/MSNet.h>
#include <microsim/MSParkingArea.h>
#include <microsim/MSStop.h>
#include <microsim/MSVehicleControl.h>
#include <microsim/MSVehicleTransfer.h>
#include <microsim/trigger/MSChargingStation.h>
#include <microsim/output/MSDetectorControl.h>
#include <utils/common/ParametrisedWrappingCommand.h>
#include <utils/options/OptionsCont.h>
#include <utils/emissions/PollutantsInterface.h>
#include <utils/emissions/HelpersEnergy.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/xml/SUMOSAXAttributes.h>
#include "MSRoutingEngine.h"
#include "MSDevice_Battery.h"
#include "MSDevice_StationFinder.h"

//#define DEBUG_STATIONFINDER_RESCUE
//#define DEBUG_STATIONFINDER_REROUTE


// ===========================================================================
// static variables
// ===========================================================================


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_StationFinder::insertOptions(OptionsCont& oc) {
    insertDefaultAssignmentOptions("stationfinder", "Battery", oc);
    oc.doRegister("device.stationfinder.rescueTime", new Option_String("1800", "TIME"));
    oc.addDescription("device.stationfinder.rescueTime", "Battery", TL("Time to wait for a rescue vehicle on the road side when the battery is empty"));
    oc.doRegister("device.stationfinder.rescueAction", new Option_String("remove"));
    oc.addDescription("device.stationfinder.rescueAction", "Battery", TL("How to deal with a vehicle which has to stop due to low battery: [none, remove, tow]"));
    oc.doRegister("device.stationfinder.reserveFactor", new Option_Float(1.1));
    oc.addDescription("device.stationfinder.reserveFactor", "Battery", TL("Scale battery need with this factor to account for unexpected traffic situations"));
    oc.doRegister("device.stationfinder.emptyThreshold", new Option_Float(0.05));
    oc.addDescription("device.stationfinder.emptyThreshold", "Battery", TL("Battery percentage to go into rescue mode"));
    oc.doRegister("device.stationfinder.radius", new Option_String("180", "TIME"));
    oc.addDescription("device.stationfinder.radius", "Battery", TL("Search radius in travel time seconds"));
    oc.doRegister("device.stationfinder.maxEuclideanDistance", new Option_Float(-1));
    oc.addDescription("device.stationfinder.maxEuclideanDistance", "Battery", TL("Euclidean search distance in meters (a negative value disables the restriction)"));
    oc.doRegister("device.stationfinder.repeat", new Option_String("60", "TIME"));
    oc.addDescription("device.stationfinder.repeat", "Battery", TL("When to trigger a new search if no station has been found"));
    oc.doRegister("device.stationfinder.maxChargePower", new Option_Float(100000.));
    oc.addDescription("device.stationfinder.maxChargePower", "Battery", TL("The maximum charging speed of the vehicle battery"));
    oc.doRegister("device.stationfinder.chargeType", new Option_String("charging"));
    oc.addDescription("device.stationfinder.chargeType", "Battery", TL("Type of energy transfer"));
    oc.doRegister("device.stationfinder.waitForCharge", new Option_String("600", "TIME"));
    oc.addDescription("device.stationfinder.waitForCharge", "Battery", TL("After this waiting time vehicle searches for a new station when the initial one is blocked"));
    oc.doRegister("device.stationfinder.saturatedChargeLevel", new Option_Float(0.8));
    oc.addDescription("device.stationfinder.saturatedChargeLevel", "Battery", TL("Target state of charge after which the vehicle stops charging"));
    oc.doRegister("device.stationfinder.needToChargeLevel", new Option_Float(0.4));
    oc.addDescription("device.stationfinder.needToChargeLevel", "Battery", TL("State of charge the vehicle begins searching for charging stations"));
    oc.doRegister("device.stationfinder.replacePlannedStop", new Option_Float(0.));
    oc.addDescription("device.stationfinder.replacePlannedStop", "Battery", TL("Share of stopping time of the next independently planned stop to use for charging instead"));
    oc.doRegister("device.stationfinder.maxDistanceToReplacedStop", new Option_Float(300.));
    oc.addDescription("device.stationfinder.maxDistanceToReplacedStop", "Battery", TL("Maximum distance in meters from the original stop to be replaced by the charging stop"));
    oc.doRegister("device.stationfinder.chargingStrategy", new Option_String("none"));
    oc.addDescription("device.stationfinder.chargingStrategy", "Battery", TL("Set a charging strategy to alter time and charging load from the set: [none, balanced, latest]"));
}



void
MSDevice_StationFinder::buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into) {
    OptionsCont& oc = OptionsCont::getOptions();
    if (equippedByDefaultAssignmentOptions(oc, "stationfinder", v, false)) {
        into.push_back(new MSDevice_StationFinder(v));
    }
}


// ---------------------------------------------------------------------------
// MSDevice_StationFinder-methods
// ---------------------------------------------------------------------------
MSDevice_StationFinder::MSDevice_StationFinder(SUMOVehicle& holder)
    : MSVehicleDevice(holder, "stationfinder_" + holder.getID()),
      MSStoppingPlaceRerouter(SUMO_TAG_CHARGING_STATION, "device.stationfinder.charging", true, false, {
    {"waitingTime", 1.}, {"chargingTime", 1.}
}, { {"waitingTime", false}, {"chargingTime", false} }),
myVeh(dynamic_cast<MSVehicle&>(holder)),
myBattery(nullptr), myChargingStation(nullptr), myRescueCommand(nullptr), myChargeLimitCommand(nullptr),
myLastChargeCheck(0), myCheckInterval(1000), myArrivalAtChargingStation(-1), myLastSearch(-1) {
    // consider whole path to/from a charging station in the search
    myEvalParams["distanceto"] = 0.;
    myEvalParams["timeto"] = 1.;
    myEvalParams["timefrom"] = 1.;
    myNormParams["chargingTime"] = true;
    myNormParams["waitingTime"] = true;
    myRescueTime = STEPS2TIME(holder.getTimeParam("device.stationfinder.rescueTime"));
    const std::string chargingStrategy = holder.getStringParam("device.stationfinder.chargingStrategy");
    if (chargingStrategy == "balanced") {
        myChargingStrategy = CHARGINGSTRATEGY_BALANCED;
    } else if (chargingStrategy == "latest") {
        myChargingStrategy = CHARGINGSTRATEGY_LATEST;
    } else if (chargingStrategy == "none") {
        myChargingStrategy = CHARGINGSTRATEGY_NONE;
    } else {
        WRITE_ERRORF(TL("Invalid device.stationfinder.chargingStrategy '%'."), chargingStrategy);
    }
    const std::string rescueAction = holder.getStringParam("device.stationfinder.rescueAction");
    if (rescueAction == "remove") {
        myRescueAction = RESCUEACTION_REMOVE;
    }  else if (rescueAction == "tow") {
        myRescueAction = RESCUEACTION_TOW;
    } else if (rescueAction == "none") {
        myRescueAction = RESCUEACTION_NONE;
    } else {
        WRITE_ERRORF(TL("Invalid device.stationfinder.rescueAction '%'."), rescueAction);
    }
    initRescueCommand();
    myReserveFactor = MAX2(1., holder.getFloatParam("device.stationfinder.reserveFactor"));
    myEmptySoC = MAX2(0., MIN2(holder.getFloatParam("device.stationfinder.emptyThreshold"), 1.));
    myRadius = holder.getTimeParam("device.stationfinder.radius");
    myMaxEuclideanDistance = holder.getFloatParam("device.stationfinder.maxEuclideanDistance");
    myRepeatInterval = holder.getTimeParam("device.stationfinder.repeat");
    myMaxChargePower = holder.getFloatParam("device.stationfinder.maxChargePower");
    myChargeType = CHARGETYPE_CHARGING;

    myWaitForCharge = holder.getTimeParam("device.stationfinder.waitForCharge");
    myTargetSoC = MAX2(0., MIN2(holder.getFloatParam("device.stationfinder.saturatedChargeLevel"), 1.));
    mySearchSoC = MAX2(0., MIN2(holder.getFloatParam("device.stationfinder.needToChargeLevel"), 1.));
    if (mySearchSoC <= myEmptySoC) {
        WRITE_WARNINGF(TL("Vehicle '%' searches for charging stations only in the rescue case due to search threshold % <= rescue threshold %."), myHolder.getID(), mySearchSoC, myEmptySoC);
    }
    myReplacePlannedStop = MAX2(0., holder.getFloatParam("device.stationfinder.replacePlannedStop"));
    myDistanceToOriginalStop = holder.getFloatParam("device.stationfinder.maxDistanceToReplacedStop");
    myUpdateSoC = -1.; // MAX2(0., mySearchSoC - DEFAULT_SOC_INTERVAL);
}


MSDevice_StationFinder::~MSDevice_StationFinder() {
    // make the rescue command invalid if there is one
    if (myRescueCommand != nullptr) {
        myRescueCommand->deschedule();
    }
    if (myChargeLimitCommand != nullptr) {
        myChargeLimitCommand->deschedule();
    }
}


bool
MSDevice_StationFinder::notifyMove(SUMOTrafficObject& veh, double /*oldPos*/, double /*newPos*/, double /*newSpeed*/) {
    if (myBattery->getEnergyCharged() > 0. && myChargingStation != nullptr) {
        myArrivalAtChargingStation = -1;
        myChargingStation = nullptr;
        mySearchState = SEARCHSTATE_CHARGING;
        return true;
    } else if (mySearchState == SEARCHSTATE_CHARGING) {
        if (myBattery->getChargingStationID() == "") {
            mySearchState = SEARCHSTATE_NONE;
        } else {
            return true;
        }
    }
    // check if the vehicle travels at most an edge length to the charging station after jump/teleport
    if (mySearchState == SEARCHSTATE_BROKEN_DOWN && myVeh.hasStops() && myVeh.getStop(0).chargingStation != nullptr && myVeh.getStop(0).chargingStation->getLane().getEdge().getID() == myVeh.getLane()->getEdge().getID()) {
        return true;
    }
    const SUMOTime now = SIMSTEP;
    if (myChargingStation != nullptr) {
        if (myArrivalAtChargingStation > 0 && now - myArrivalAtChargingStation > myWaitForCharge) {
            // waited for too long, try another charging station
            if (rerouteToChargingStation(true)) {
                WRITE_MESSAGE(TLF("Rerouted vehicle '%' after waiting too long at the previous charging station at time=%.", veh.getID(), toString(SIMTIME)));
            }
        } else if (myArrivalAtChargingStation < 0 && myVeh.willStop() && myVeh.getDistanceToPosition(myChargingStation->getBeginLanePosition(), myVeh.getLane()) < DEFAULT_CHARGINGSTATION_VIEW_DIST) {
            // remember when the vehicle arrived close to the target charging station
            mySearchState = SEARCHSTATE_WAITING;
            myArrivalAtChargingStation = now;
        }
    }
    const double currentSoC = myBattery->getActualBatteryCapacity() / myBattery->getMaximumBatteryCapacity();
    if (currentSoC > mySearchSoC || mySearchState == SEARCHSTATE_BROKEN_DOWN) {
        // battery SoC is too high to look for charging facilities or the vehicle is already in rescue mode
        return true;
    }
    // only check once per second
    if (now - myLastChargeCheck < 1000) {
        return true;
    } else if (myRescueAction != RESCUEACTION_NONE  && (currentSoC < myEmptySoC || currentSoC < NUMERICAL_EPS)) {

        // vehicle has to stop at the end of the  because battery SoC is too low
        double brakeGap = myVeh.getCarFollowModel().brakeGap(myVeh.getSpeed());
        std::pair<const MSLane*, double> stopPos = myVeh.getLanePosAfterDist(brakeGap);
        if (stopPos.first != nullptr) {
            const MSLane* stopLane = (stopPos.first->isInternal()) ? stopPos.first->getNormalSuccessorLane() : stopPos.first;
            double endPos = stopPos.second;
            if (stopLane != stopPos.first) {
                endPos = MIN2(POSITION_EPS, stopLane->getLength());
            }
            // remove possibly scheduled charging stop
            if (myVeh.hasStops() && myVeh.getStop(0).chargingStation != nullptr) {
                myVeh.abortNextStop();
            }

            // schedule the rescue stop
            SUMOVehicleParameter::Stop rescueStop;
            rescueStop.index = 0;
            rescueStop.edge = stopLane->getEdge().getID();
            rescueStop.lane = stopLane->getID();
            rescueStop.startPos = MAX2(endPos - 2 * myHolder.getVehicleType().getLength(), 0.);
            rescueStop.endPos = endPos;
            rescueStop.parametersSet |= STOP_START_SET | STOP_END_SET;
            WRITE_MESSAGEF(TL("Vehicle '%' wants to stop on lane % at pos % because of low battery charge % at time=%."), myHolder.getID(), rescueStop.lane, toString(rescueStop.endPos), toString(currentSoC), toString(SIMTIME));

            if (myRescueAction == RESCUEACTION_REMOVE) {
                // remove vehicle from network
                rescueStop.until = SUMOTime_MAX;
                rescueStop.breakDown = true;
                std::string errorMsg = "Could not insert the rescue stop.";
                if (!myVeh.insertStop(0, rescueStop, "stationfinder:rescue", false, errorMsg)) {
                    WRITE_ERROR(errorMsg);
                }
                mySearchState = SEARCHSTATE_BROKEN_DOWN;
                return true;
            } else if (myRescueAction == RESCUEACTION_TOW) {
                // wait next to the road and get teleported to a charging station
                SUMOTime rescueTime = TIME2STEPS(myRescueTime);
                rescueStop.duration = rescueTime;
                rescueStop.parking = ParkingType::ONROAD;
                rescueStop.jump = 0;
                std::string errorMsg = "Could not insert the rescue stop.";
                if (!myVeh.insertStop(0, rescueStop, "stationfinder:rescue", false, errorMsg)) {
                    WRITE_ERROR(errorMsg);
                }
                initRescueCommand();
                MSNet::getInstance()->getBeginOfTimestepEvents()->addEvent(myRescueCommand, SIMSTEP + rescueStop.duration - DELTA_T);
                mySearchState = SEARCHSTATE_BROKEN_DOWN;
                return true;
            }
        }
    } else if (myChargingStation == nullptr &&
               (myUpdateSoC < 0. || myUpdateSoC - currentSoC > DEFAULT_SOC_INTERVAL || (mySearchState == SEARCHSTATE_UNSUCCESSFUL &&
                       now - myLastSearch >= myRepeatInterval && !myHolder.isStopped()))) {
        // check if a charging stop is already planned without the device, otherwise reroute inside this device
        if (!alreadyPlannedCharging() && now > myHolder.getDeparture()) {
            rerouteToChargingStation();
        }
        myUpdateSoC = currentSoC;
    }
    myLastChargeCheck = SIMSTEP;
    return true;
}


bool
MSDevice_StationFinder::notifyIdle(SUMOTrafficObject& /*veh*/) {
    return true;
}


void
MSDevice_StationFinder::saveState(OutputDevice& out) const {
    out.openTag(SUMO_TAG_DEVICE);
    out.writeAttr(SUMO_ATTR_ID, getID());
    std::vector<std::string> internals;
    internals.push_back(toString(myLastChargeCheck));
    internals.push_back(toString(myUpdateSoC));
    internals.push_back(toString(mySearchSoC));
    internals.push_back(toString(myTargetSoC));
    internals.push_back(toString(myWaitForCharge));
    internals.push_back(toString(myRepeatInterval));
    internals.push_back(toString(myRadius));
    internals.push_back(toString(myLastSearch));
    internals.push_back(toString(myReserveFactor));
    internals.push_back(toString(mySearchState));
    internals.push_back(toString(myArrivalAtChargingStation));
    internals.push_back((myChargingStation == nullptr) ? "NULL" : myChargingStation->getID());
    internals.push_back(toString(myChargeLimits.size()));
    for (auto chargeLimit : myChargeLimits) {
        internals.push_back(toString(chargeLimit.first));
        internals.push_back(toString(chargeLimit.second));
    }
    out.writeAttr(SUMO_ATTR_STATE, toString(internals));
    out.closeTag();
}


void
MSDevice_StationFinder::loadState(const SUMOSAXAttributes& attrs) {
    std::istringstream bis(attrs.getString(SUMO_ATTR_STATE));
    bis >> myLastChargeCheck;
    bis >> myUpdateSoC;
    bis >> mySearchSoC;
    bis >> myTargetSoC;
    bis >> myWaitForCharge;
    bis >> myRepeatInterval;
    bis >> myRadius;
    bis >> myLastSearch;
    bis >> myReserveFactor;
    int searchState;
    bis >> searchState;
    mySearchState = (SearchState)searchState;
    bis >> myArrivalAtChargingStation;
    std::string csID;
    bis >> csID;
    if (csID != "NULL") {
        myChargingStation = dynamic_cast<MSChargingStation*>(MSNet::getInstance()->getStoppingPlace(csID, SUMO_TAG_CHARGING_STATION));
    }
    int chargeLimitCount = 0;
    bis >> chargeLimitCount;
    for (int i = 0; i < chargeLimitCount; ++i) {
        SUMOTime t = 0;
        double limit = 0.;
        bis >> t;
        bis >> limit;
        myChargeLimits.push_back({ t, limit });
    }
}


void
MSDevice_StationFinder::notifyMoveInternal(const SUMOTrafficObject& /*veh*/,
        const double /* frontOnLane */,
        const double /* timeOnLane */,
        const double /* meanSpeedFrontOnLane */,
        const double /* meanSpeedVehicleOnLane */,
        const double /* travelledDistanceFrontOnLane */,
        const double /* travelledDistanceVehicleOnLane */,
        const double /* meanLengthOnLane */) {

    // called by meso (see MSMeanData_Emissions::MSLaneMeanDataValues::notifyMoveInternal)
}


MSChargingStation*
MSDevice_StationFinder::findChargingStation(SUMOAbstractRouter<MSEdge, SUMOVehicle>& /*router*/, double expectedConsumption, StoppingPlaceParamMap_t& scores, bool constrainTT, bool skipVisited, bool skipOccupied) {
    MSChargingStation* minStation = nullptr;
    std::vector<StoppingPlaceVisible> candidates;
    const StoppingPlaceMemory* chargingMemory = myVeh.getChargingMemory();
    if (chargingMemory == nullptr) {
        skipVisited = false;
    }
    const SUMOTime stoppingPlaceMemory = TIME2STEPS(getWeight(myHolder, "memory", 600));
    for (const auto& stop : MSNet::getInstance()->getStoppingPlaces(SUMO_TAG_CHARGING_STATION)) {
        MSChargingStation* cs = static_cast<MSChargingStation*>(stop.second);
        if (cs->getEfficency() < NUMERICAL_EPS || cs->getChargingPower(false) < NUMERICAL_EPS) {
            continue;
        }
        if (cs->getParkingArea() != nullptr && !cs->getParkingArea()->accepts(&myVeh)) {
            // skip stations where the linked parking area does not grant access to the device holder
            continue;
        }
        if (skipOccupied && freeSpaceAtChargingStation(cs) < 1.) {
            continue;
        }
        if (skipVisited && chargingMemory->sawBlockedStoppingPlace(cs, false) > 0 && SIMSTEP - chargingMemory->sawBlockedStoppingPlace(cs, false) < stoppingPlaceMemory) {
            // skip recently visited
            continue;
        }
        if (constrainTT && myMaxEuclideanDistance > 0 && stop.second->getLane().geometryPositionAtOffset(stop.second->getBeginLanePosition()).distanceTo2D(myHolder.getPosition()) > myMaxEuclideanDistance) {
            // skip probably too distant charging stations
            continue;
        }
        candidates.push_back({cs, false});
    }
    ConstMSEdgeVector newRoute;
    scores["expectedConsumption"] = expectedConsumption;
    std::vector<double> probs(candidates.size(), 1.);
    bool newDestination;
    myCheckValidity = constrainTT;
    MSStoppingPlace* bestCandidate = reroute(candidates, probs, myHolder, newDestination, newRoute, scores);
    myCheckValidity = true;
    minStation = dynamic_cast<MSChargingStation*>(bestCandidate);
    return minStation;
}


bool
MSDevice_StationFinder::rerouteToChargingStation(bool replace) {
    double expectedConsumption = MIN2(estimateConsumption() * myReserveFactor, myBattery->getMaximumBatteryCapacity() * myTargetSoC);
    if (myBattery->getActualBatteryCapacity() < expectedConsumption) {
        myLastSearch = SIMSTEP;
        MSVehicleRouter& router = MSRoutingEngine::getRouterTT(myHolder.getRNGIndex(), myHolder.getVClass());
        StoppingPlaceParamMap_t scores = {};
        MSChargingStation* cs = findChargingStation(router, expectedConsumption, scores);
        if (cs != nullptr) {
            // integrate previously planned stops which do not have charging facilities
            myChargingStation = cs;
            SUMOVehicleParameter::Stop stopPar;
            stopPar.chargingStation = cs->getID();
            if (cs->getParkingArea() != nullptr) {
                stopPar.parkingarea = cs->getParkingArea()->getID();
                stopPar.parking = (cs->getParkingArea()->parkOnRoad()) ? ParkingType::ONROAD : ParkingType::OFFROAD;
            }
            stopPar.edge = cs->getLane().getEdge().getID();
            stopPar.lane = cs->getLane().getID();
            stopPar.duration = TIME2STEPS(expectedConsumption / (cs->getChargingPower(false) * cs->getEfficency()));
            stopPar.parametersSet = STOP_DURATION_SET;
            if (myReplacePlannedStop > 0) {
                // "reuse" a previously planned stop (stop at charging station instead of a different stop)
                // what if the charging station is skipped due to long waiting time?
                if (myReplacePlannedStop > 0. && myHolder.hasStops() && myHolder.getNextStopParameter()->chargingStation.empty()) {
                    // compare the distance to the original target
                    if (scores["distfrom"] < myDistanceToOriginalStop /*actualDist < myDistanceToOriginalStop*/) {
                        // compute the arrival time at the original stop
                        const SUMOTime timeToOriginalStop = TIME2STEPS(scores["timefrom"]);
                        const SUMOTime originalUntil = myHolder.getNextStopParameter()->until;
                        if (timeToOriginalStop + myLastSearch < originalUntil) {
                            const SUMOTime delta = originalUntil - (timeToOriginalStop + myLastSearch);
                            stopPar.until = timeToOriginalStop + myLastSearch + (SUMOTime)((double)delta * MIN2(myReplacePlannedStop, 1.));
                            stopPar.parametersSet |= STOP_UNTIL_SET;
                            if (myReplacePlannedStop > 1.) {
                                myHolder.abortNextStop();
                            }
                            // optionally implement a charging strategy by adjusting the accepted charging rates
                            if (myChargingStrategy != CHARGINGSTRATEGY_NONE) {
                                // the charging strategy should actually only be computed at the arrival at the charging station
                                implementChargingStrategy(myLastSearch + TIME2STEPS(scores["timeto"]), stopPar.until, expectedConsumption, cs);
                            }
                        }
                    }
                }
            }
            stopPar.startPos = cs->getBeginLanePosition();
            stopPar.endPos = cs->getEndLanePosition();
            std::string errorMsg;
#ifdef DEBUG_STATIONFINDER_REROUTE
            std::ostringstream os;
            const ConstMSEdgeVector edgesBefore = myVeh.getRoute().getEdges();
            for (auto edge : edgesBefore) {
                os << edge->getID() << " ";
            }
            std::cout << "MSDevice_StationFinder::rerouteToChargingStation: \n\tRoute before scheduling the charging station: " << os.str() << "\n";
#endif
            if ((replace && !myVeh.replaceStop(0, stopPar, "stationfinder:search", false, errorMsg)) || (!replace && !myVeh.insertStop(0, stopPar, "stationfinder:search", false, errorMsg))) {
                WRITE_MESSAGE(TLF("Problem with inserting the charging station stop for vehicle %.", myHolder.getID()));
                WRITE_ERROR(errorMsg);
            }

#ifdef DEBUG_STATIONFINDER_REROUTE
            std::ostringstream os2;
            const ConstMSEdgeVector edgesAfter = myVeh.getRoute().getEdges();
            for (auto edge : edgesAfter) {
                os2 << edge->getID() << " ";
            }
            std::cout << "\tRoute after scheduling the charging station: " << os2.str() << "\n";
#endif
            myArrivalAtChargingStation = -1;
            mySearchState = SEARCHSTATE_SUCCESSFUL;
#ifdef DEBUG_STATIONFINDER_REROUTE
            std::cout << "\tVehicle " << myHolder.getID() << " gets rerouted to charging station " << cs->getID()  << " on edge " << stopPar.edge  << " at time " << SIMTIME << "\n";
#endif
            return true;
        }
        mySearchState = SEARCHSTATE_UNSUCCESSFUL;
        WRITE_MESSAGEF(TL("Vehicle '%' wants to charge at time=% but does not find any charging station nearby."), myHolder.getID(), toString(SIMTIME));
    }
    return false;
}


SUMOTime
MSDevice_StationFinder::teleportToChargingStation(const SUMOTime /*currentTime*/) {
    // find closest charging station
    MSVehicleRouter& router = MSRoutingEngine::getRouterTT(myHolder.getRNGIndex(), myHolder.getVClass());
    double expectedConsumption = MIN2(estimateConsumption(nullptr, true, STEPS2TIME(myVeh.getStops().front().pars.duration)) * myReserveFactor, myBattery->getMaximumBatteryCapacity() * myTargetSoC);
    StoppingPlaceParamMap_t scores = {};
    MSChargingStation* cs = findChargingStation(router, expectedConsumption, scores, false, false, true);
    if (cs == nullptr) {
        // continue waiting if all charging stations are occupied
#ifdef DEBUG_STATIONFINDER_RESCUE
        std::cout << "MSDevice_StationFinder::teleportToChargingStation: No charging station available to teleport the broken-down vehicle " << myHolder.getID() << " to at time " << SIMTIME << ".\n.";
#endif
        // remove the vehicle if teleport to a charging station fails
        if (myHolder.isStopped()) {
            MSStop& currentStop = myHolder.getNextStop();
            currentStop.duration += DELTA_T;
            SUMOVehicleParameter::Stop& stopPar = const_cast<SUMOVehicleParameter::Stop&>(currentStop.pars);
            stopPar.jump = -1;
            stopPar.breakDown = true;
            mySearchState = SEARCHSTATE_BROKEN_DOWN;
            WRITE_WARNINGF(TL("There is no charging station available to teleport the vehicle '%' to at time=%. Thus the vehicle will be removed."), myHolder.getID(), toString(SIMTIME));
        }
#ifdef DEBUG_STATIONFINDER_RESCUE
        else {
#ifdef DEBUG_STATIONFINDER_RESCUE
            std::cout << "MSDevice_StationFinder::teleportToChargingStation: Rescue stop of " << myHolder.getID() << " ended prematurely before regular end at " << SIMTIME << ".\n.";
#endif
        }
#endif
        return myRepeatInterval;
    }

    // teleport to the charging station, stop there for charging
    myChargingStation = cs;
    SUMOVehicleParameter::Stop stopPar;
    stopPar.chargingStation = cs->getID();
    if (cs->getParkingArea() != nullptr) {
        stopPar.parkingarea = cs->getParkingArea()->getID();
        stopPar.parking = (cs->getParkingArea()->parkOnRoad()) ? ParkingType::ONROAD : ParkingType::OFFROAD;
    }
    stopPar.edge = cs->getLane().getEdge().getID();
    stopPar.lane = cs->getLane().getID();
    stopPar.startPos = cs->getBeginLanePosition();
    stopPar.endPos = cs->getEndLanePosition();
    stopPar.duration = TIME2STEPS(expectedConsumption / (cs->getChargingPower(false) * cs->getEfficency()));
    std::string errorMsg;
    if (!myVeh.insertStop(1, stopPar, "stationfinder:search", true, errorMsg)) {
        WRITE_ERROR(errorMsg);
    }
    myRescueCommand->deschedule();
    myRescueCommand = nullptr;
    return 0;
}


double
MSDevice_StationFinder::estimateConsumption(const MSEdge* target, const bool includeEmptySoC, const double stopDiscount) const {
    const SUMOTime now = SIMSTEP;
    MSVehicleRouter& router = MSRoutingEngine::getRouterTT(myHolder.getRNGIndex(), myHolder.getVClass());
    const ConstMSEdgeVector& route = myHolder.getRoute().getEdges();
    ConstMSEdgeVector::const_iterator targetIt = (target == nullptr) ? route.end() : std::find(route.begin(), route.end(), target) + 1;
    const ConstMSEdgeVector remainingRoute(route.begin() + myHolder.getRoutePosition(), targetIt);
    const double remainingTime = router.recomputeCosts(remainingRoute, &myHolder, now);
    if (now > myHolder.getDeparture()) {
        const double totalConsumption = myBattery->getTotalConsumption();
        double expectedConsumption = 0.;
        double passedTime = STEPS2TIME(now - myHolder.getDeparture());
        if (totalConsumption > 0. && passedTime - stopDiscount > DEFAULT_CONSUMPTION_ESTIMATE_HISTORY) {
            expectedConsumption = totalConsumption / (passedTime - stopDiscount) * remainingTime;
        } else {
            // fallback consumption rate for vehicles starting with low battery
            const double speed = MIN2(myHolder.getMaxSpeed(), myHolder.getLane()->getSpeedLimit());
            EnergyParams* const params = myHolder.getEmissionParameters();
            expectedConsumption = PollutantsInterface::compute(myVeh.getVehicleType().getEmissionClass(), PollutantsInterface::ELEC,
                                  speed * 0.8, 0., 0., params) * (remainingTime - passedTime);
        }
        if (includeEmptySoC) {
            expectedConsumption += MAX2(0., myEmptySoC * myBattery->getMaximumBatteryCapacity() - myBattery->getActualBatteryCapacity());
        }
        expectedConsumption /= myHolder.getEmissionParameters()->getDouble(SUMO_ATTR_PROPULSIONEFFICIENCY);
        return expectedConsumption;
    }
    return 0.;
}


double
MSDevice_StationFinder::freeSpaceAtChargingStation(MSChargingStation* cs) const {
    return (cs->getParkingArea() != nullptr) ? cs->getParkingArea()->getCapacity() - cs->getParkingArea()->getOccupancy() : (cs->getLastFreePos() - cs->getBeginLanePosition()) / myHolder.getVehicleType().getParameter().length;
}


bool
MSDevice_StationFinder::alreadyPlannedCharging() {
    if (myChargingStation == nullptr) {
        auto stops = myHolder.getStops();
        for (auto stop : stops) {
            if (stop.chargingStation != nullptr) {
                // compare whether we'll make it there without intermediate charging
                double expectedConsumption = estimateConsumption(*stop.edge);
                if (myBattery->getActualBatteryCapacity() < expectedConsumption) {
                    myChargingStation = stop.chargingStation;
                    return true;
                }
            }
        }
    }
    return false;
}


void
MSDevice_StationFinder::initRescueCommand() {
    if (myRescueAction == RESCUEACTION_TOW && myRescueCommand == nullptr) {
        myRescueCommand = new WrappingCommand<MSDevice_StationFinder>(this, &MSDevice_StationFinder::teleportToChargingStation);
    }
}


void
MSDevice_StationFinder::initChargeLimitCommand() {
    if (myChargingStrategy != CHARGINGSTRATEGY_NONE && myChargeLimitCommand == nullptr) {
        myChargeLimitCommand = new WrappingCommand<MSDevice_StationFinder>(this, &MSDevice_StationFinder::updateChargeLimit);
    }
}


SUMOTime
MSDevice_StationFinder::updateChargeLimit(const SUMOTime currentTime) {
    if (myChargeLimits.size() > 0 && myChargeLimits.begin()->first < currentTime - DELTA_T) {
        myChargeLimits.clear();
    }
    if (myChargeLimits.size() > 0) {
        double chargeLimit = myChargeLimits.begin()->second;
        myBattery->setChargeLimit(chargeLimit);
        if (chargeLimit < 0) {
            WRITE_MESSAGEF(TL("The charging rate limit of vehicle '%' is lifted at time=%"), myHolder.getID(), STEPS2TIME(SIMSTEP));
        } else {
            WRITE_MESSAGEF(TL("The charging rate of vehicle '%' is limited to % at time=%"), myHolder.getID(), chargeLimit, STEPS2TIME(SIMSTEP));
        }
        myChargeLimits.erase(myChargeLimits.begin());
    }
    if (myChargeLimits.size() == 0) {
        myChargeLimitCommand->deschedule();
        myChargeLimitCommand = nullptr;
        return 0;
    } else {
        return myChargeLimits.begin()->first - currentTime;
    }
}


void
MSDevice_StationFinder::implementChargingStrategy(SUMOTime begin, SUMOTime end, const double plannedCharge, const MSChargingStation* cs) {
    myChargeLimits.clear();
    if (myChargingStrategy == CHARGINGSTRATEGY_BALANCED) {
        const double balancedCharge = plannedCharge / STEPS2TIME(end - begin);
        myChargeLimits.push_back({ begin, balancedCharge });
        myChargeLimits.push_back({ end, -1});
    } else { // CHARGINGSTRATEGY_LATEST
        SUMOTime expectedDuration = myBattery->estimateChargingDuration(plannedCharge, cs->getChargingPower(false) * cs->getEfficency());
        if (end - expectedDuration > begin) {
            myChargeLimits.push_back({ begin, 0 });
            myChargeLimits.push_back({ end - expectedDuration, -1 });
        }
    }
    if (myChargeLimits.size() > 0) {
        initChargeLimitCommand();
        MSNet::getInstance()->getBeginOfTimestepEvents()->addEvent(myChargeLimitCommand, begin);
    }
}


void
MSDevice_StationFinder::generateOutput(OutputDevice* tripinfoOut) const {
    if (tripinfoOut != nullptr && myChargingStation != nullptr) {
        tripinfoOut->openTag("stationfinder");
        tripinfoOut->writeAttr("chargingStation", myChargingStation->getID());
        tripinfoOut->closeTag();
    }
}


std::string
MSDevice_StationFinder::getParameter(const std::string& key) const {
    if (key == "chargingStation") { // eventually abstract with enum
        return (myChargingStation == nullptr) ? "" : myChargingStation->getID();
    } else if (key == "batteryNeed") {
        return toString(estimateConsumption() * myReserveFactor);
    } else if (key == "needToChargeLevel") {
        return toString(myRadius);
    } else if (key == "saturatedChargeLevel") {
        return toString(myRadius);
    } else if (key == "waitForCharge") {
        return toString(myRadius);
    } else if (key == "repeat") {
        return toString(myRadius);
    } else if (key == "radius") {
        return toString(myRadius);
    } else if (key == "reserveFactor") {
        return toString(myRadius);
    }
    throw InvalidArgument(TLF("Parameter '%' is not supported for device of type '%'", key, deviceName()));
}


void
MSDevice_StationFinder::setParameter(const std::string& key, const std::string& value) {
    double doubleValue;
    try {
        doubleValue = StringUtils::toDouble(value);
    } catch (NumberFormatException&) {
        throw InvalidArgument(TLF("Setting parameter '%' requires a number for device of type '%'", key, deviceName()));
    }
    if (key == "needToChargeLevel") {
        mySearchSoC = MAX2(0., MIN2(1., doubleValue));
    } else if (key == "saturatedChargeLevel") {
        myTargetSoC = MAX2(0., MIN2(1., doubleValue));
    } else if (key == "waitForCharge") {
        myWaitForCharge = TIME2STEPS(MAX2(0., doubleValue));
    } else if (key == "repeat") {
        myRepeatInterval = TIME2STEPS(MAX2(0., doubleValue));
    } else if (key == "radius") {
        myRadius = TIME2STEPS(MAX2(0., doubleValue));
    } else if (key == "reserveFactor") {
        myReserveFactor = MAX2(1., doubleValue);
    } else {
        throw InvalidArgument(TLF("Setting parameter '%' is not supported for device of type '%'", key, deviceName()));
    }
}


bool
MSDevice_StationFinder::evaluateCustomComponents(SUMOVehicle& /* veh */, double /* brakeGap */, bool /* newDestination */,
        MSStoppingPlace* alternative, double /* occupancy */, double /* prob */,
        SUMOAbstractRouter<MSEdge, SUMOVehicle>& /* router */,
        StoppingPlaceParamMap_t& stoppingPlaceValues,
        ConstMSEdgeVector& /* newRoute */, ConstMSEdgeVector& /* stoppingPlaceApproach */,
        StoppingPlaceParamMap_t& /* maxValues */, StoppingPlaceParamMap_t& addInput) {
    // estimated waiting time and charging time
    MSChargingStation* cs = dynamic_cast<MSChargingStation*>(alternative);
    double parkingCapacity = (cs->getParkingArea() != nullptr) ? cs->getParkingArea()->getCapacity() : (cs->getEndLanePosition() - cs->getBeginLanePosition()) / myHolder.getVehicleType().getParameter().length;
    double freeParkingCapacity = freeSpaceAtChargingStation(cs);
    stoppingPlaceValues["waitingTime"] = (freeParkingCapacity < 1.) ? DEFAULT_AVG_WAITING_TIME / parkingCapacity : 0.;
    stoppingPlaceValues["chargingTime"] = STEPS2TIME(cs->getChargeDelay()) + addInput["expectedConsumption"] / cs->getChargingPower(false);
    return true;
}


bool
MSDevice_StationFinder::validComponentValues(StoppingPlaceParamMap_t& stoppingPlaceValues) {
    if (stoppingPlaceValues["timeto"] > STEPS2TIME(myRadius)) {
        return false;
    }
    return true;
}


bool
MSDevice_StationFinder::useStoppingPlace(MSStoppingPlace* /* stoppingPlace */) {
    return true;
}


SUMOAbstractRouter<MSEdge, SUMOVehicle>& MSDevice_StationFinder::getRouter(SUMOVehicle& veh, const MSEdgeVector& prohibited) {
    return MSRoutingEngine::getRouterTT(veh.getRNGIndex(), veh.getVClass(), prohibited);
}


double
MSDevice_StationFinder::getStoppingPlaceOccupancy(MSStoppingPlace* stoppingPlace) {
    MSChargingStation* cs = dynamic_cast<MSChargingStation*>(stoppingPlace);
    if (cs->getParkingArea() != nullptr) {
        return cs->getParkingArea()->getOccupancy();
    }
    return (cs->getEndLanePosition() - cs->getLastFreePos()) / (myHolder.getLength() + myHolder.getVehicleType().getMinGap());
}


double
MSDevice_StationFinder::getLastStepStoppingPlaceOccupancy(MSStoppingPlace* stoppingPlace) {
    MSChargingStation* cs = dynamic_cast<MSChargingStation*>(stoppingPlace);
    if (cs->getParkingArea() != nullptr) {
        return cs->getParkingArea()->getLastStepOccupancy();
    }
    return (cs->getEndLanePosition() - cs->getLastFreePos()) / (myHolder.getLength() + myHolder.getVehicleType().getMinGap());
}


double
MSDevice_StationFinder::getStoppingPlaceCapacity(MSStoppingPlace* stoppingPlace) {
    MSChargingStation* cs = dynamic_cast<MSChargingStation*>(stoppingPlace);
    if (cs->getParkingArea() != nullptr) {
        return cs->getParkingArea()->getCapacity();
    }
    return (cs->getEndLanePosition() - cs->getBeginLanePosition()) / (myHolder.getLength() + myHolder.getVehicleType().getMinGap());
}


void
MSDevice_StationFinder::rememberBlockedStoppingPlace(SUMOVehicle& veh, const MSStoppingPlace* stoppingPlace, bool blocked) {
    veh.rememberBlockedChargingStation(stoppingPlace, blocked);
}


void
MSDevice_StationFinder::rememberStoppingPlaceScore(SUMOVehicle& veh, MSStoppingPlace* place, const std::string& score) {
    veh.rememberChargingStationScore(place, score);
}


void
MSDevice_StationFinder::resetStoppingPlaceScores(SUMOVehicle& veh) {
    veh.resetChargingStationScores();
}


SUMOTime
MSDevice_StationFinder::sawBlockedStoppingPlace(SUMOVehicle& veh, MSStoppingPlace* place, bool local) {
    return veh.sawBlockedChargingStation(place, local);
}


int
MSDevice_StationFinder::getNumberStoppingPlaceReroutes(SUMOVehicle& /* veh */) {
    return 0;
}


void
MSDevice_StationFinder::setNumberStoppingPlaceReroutes(SUMOVehicle& /* veh */, int /* value */) {
}


/****************************************************************************/
