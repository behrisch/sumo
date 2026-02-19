/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2004-2026 German Aerospace Center (DLR) and others.
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
/// @file    MSMeanData_Net.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    Mon, 10.05.2004
///
// Network state mean data collector for edges/lanes
/****************************************************************************/
#include <config.h>

#ifdef HAVE_FOX
#include <utils/common/ScopedLocker.h>
#endif
#include <utils/common/SUMOTime.h>
#include <utils/common/ToString.h>
#include <utils/iodevices/OutputDevice.h>
#include <microsim/MSEdgeControl.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSGlobals.h>
#include <mesosim/MELoop.h>
#include <mesosim/MESegment.h>
#include "MSMeanData_Net.h"


// ===========================================================================
// debug constants
// ===========================================================================
//#define DEBUG_OCCUPANCY
//#define DEBUG_OCCUPANCY2
//#define DEBUG_NOTIFY_ENTER
//#define DEBUG_COND (veh.getLane()->getID() == "")
//#define DEBUG_COND (false)
//#define DEBUG_COND2 (veh.getEdge()->getID() == "")


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// MSMeanData_Net::MSLaneMeanDataValues - methods
// ---------------------------------------------------------------------------
MSMeanData_Net::MSLaneMeanDataValues::MSLaneMeanDataValues(MSLane* const lane,
        const double length,
        const bool doAdd,
        const MSMeanData_Net* parent)
    : MSMeanData::MeanDataValues(lane, length, doAdd, parent),
      nVehDeparted(0), nVehArrived(0), nVehEntered(0), nVehLeft(0),
      nVehVaporized(0), nVehTeleported(0), waitSeconds(0), timeLoss(0),
      nVehLaneChangeFrom(0), nVehLaneChangeTo(0),
      frontSampleSeconds(0), frontTravelledDistance(0),
      vehLengthSum(0), occupationSum(0),
      minimalVehicleLength(INVALID_DOUBLE),
      myParent(parent) {}


MSMeanData_Net::MSLaneMeanDataValues::~MSLaneMeanDataValues() {
}


void
MSMeanData_Net::MSLaneMeanDataValues::reset(bool) {
    nVehDeparted = 0;
    nVehArrived = 0;
    nVehEntered = 0;
    nVehLeft = 0;
    nVehVaporized = 0;
    nVehTeleported = 0;
    nVehLaneChangeFrom = 0;
    nVehLaneChangeTo = 0;
    sampleSeconds = 0.;
    travelledDistance = 0;
    waitSeconds = 0;
    timeLoss = 0;
    frontSampleSeconds = 0;
    frontTravelledDistance = 0;
    vehLengthSum = 0;
    occupationSum = 0;
    minimalVehicleLength = INVALID_DOUBLE;
    resetTime = SIMSTEP;
}


void
MSMeanData_Net::MSLaneMeanDataValues::addTo(MSMeanData::MeanDataValues& val) const {
    MSLaneMeanDataValues& v = (MSLaneMeanDataValues&) val;
    v.nVehDeparted += nVehDeparted;
    v.nVehArrived += nVehArrived;
    v.nVehEntered += nVehEntered;
    v.nVehLeft += nVehLeft;
    v.nVehVaporized += nVehVaporized;
    v.nVehTeleported += nVehTeleported;
    v.nVehLaneChangeFrom += nVehLaneChangeFrom;
    v.nVehLaneChangeTo += nVehLaneChangeTo;
    v.sampleSeconds += sampleSeconds;
    v.travelledDistance += travelledDistance;
    v.waitSeconds += waitSeconds;
    v.timeLoss += timeLoss;
    v.frontSampleSeconds += frontSampleSeconds;
    v.frontTravelledDistance += frontTravelledDistance;
    v.vehLengthSum += vehLengthSum;
    v.occupationSum += occupationSum;
    if (v.minimalVehicleLength == INVALID_DOUBLE) {
        v.minimalVehicleLength = minimalVehicleLength;
    } else {
        v.minimalVehicleLength = MIN2(minimalVehicleLength, v.minimalVehicleLength);
    }
}


void
MSMeanData_Net::MSLaneMeanDataValues::notifyMoveInternal(
    const SUMOTrafficObject& veh, const double frontOnLane,
    const double timeOnLane, const double /* meanSpeedFrontOnLane */,
    const double meanSpeedVehicleOnLane,
    const double travelledDistanceFrontOnLane,
    const double travelledDistanceVehicleOnLane,
    const double meanLengthOnLane) {
#ifdef DEBUG_OCCUPANCY
    if (DEBUG_COND2) {
        std::cout << SIMTIME << "\n  MSMeanData_Net::MSLaneMeanDataValues::notifyMoveInternal()\n"
                  << "  veh '" << veh.getID() << "' on edge '" << veh.getEdge()->getID() << "'"
                  << ", timeOnLane=" << timeOnLane
                  << ", meanSpeedVehicleOnLane=" << meanSpeedVehicleOnLane
                  << ",\ntravelledDistanceFrontOnLane=" << travelledDistanceFrontOnLane
                  << ", travelledDistanceVehicleOnLane=" << travelledDistanceVehicleOnLane
                  << ", meanLengthOnLane=" << meanLengthOnLane
                  << std::endl;
    }
#endif
    if (myParent != nullptr && !myParent->vehicleApplies(veh)) {
        return;
    }
    sampleSeconds += timeOnLane;
    travelledDistance += travelledDistanceVehicleOnLane;
    vehLengthSum += veh.getVehicleType().getLength() * timeOnLane;
    if (MSGlobals::gUseMesoSim) {
        // For the mesosim case no information on whether the vehicle was occupying
        // the lane with its whole length is available. We assume the whole length
        // Therefore this increment is taken out with more information on the vehicle movement.
        occupationSum += veh.getVehicleType().getLength() * timeOnLane;
    } else {
        // for the microsim case more elaborate calculation of the average length on the lane,
        // is taken out in notifyMove(), refs #153
        occupationSum += meanLengthOnLane * TS;
    }
    if (!veh.isStopped()) {
        if (myParent != nullptr && meanSpeedVehicleOnLane < myParent->myHaltSpeed) {
            waitSeconds += timeOnLane;
        } else if (MSGlobals::gUseMesoSim) {
            waitSeconds += STEPS2TIME(veh.getWaitingTime());
        }
        const double vmax = veh.getLane() == nullptr ? veh.getEdge()->getVehicleMaxSpeed(&veh) : veh.getLane()->getVehicleMaxSpeed(&veh);
        if (vmax > 0) {
            timeLoss += timeOnLane * MAX2(0.0, vmax - meanSpeedVehicleOnLane) / vmax;
        }
    }
    frontSampleSeconds += frontOnLane;
    frontTravelledDistance += travelledDistanceFrontOnLane;
    if (minimalVehicleLength == INVALID_DOUBLE) {
        minimalVehicleLength = veh.getVehicleType().getLengthWithGap();
    } else {
        minimalVehicleLength = MIN2(minimalVehicleLength, veh.getVehicleType().getLengthWithGap());
    }
#ifdef DEBUG_OCCUPANCY2
    // refs #3265
    std::cout << SIMTIME << "ID: " << getDescription() << " minVehicleLength=" << minimalVehicleLength << std::endl;
#endif
}


bool
MSMeanData_Net::MSLaneMeanDataValues::notifyLeave(SUMOTrafficObject& veh, double /*lastPos*/, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
    if ((myParent == nullptr || myParent->vehicleApplies(veh)) && (
                getLane() == nullptr || !veh.isVehicle() || getLane() == static_cast<MSVehicle&>(veh).getLane())) {
#ifdef HAVE_FOX
        ScopedLocker<> lock(myNotificationMutex, MSGlobals::gNumSimThreads > 1);
#endif
        if (MSGlobals::gUseMesoSim) {
            removeFromVehicleUpdateValues(veh);
        }
        if (reason == MSMoveReminder::NOTIFICATION_ARRIVED) {
            ++nVehArrived;
        } else if (reason == MSMoveReminder::NOTIFICATION_LANE_CHANGE) {
            ++nVehLaneChangeFrom;
        } else if (myParent == nullptr || reason != MSMoveReminder::NOTIFICATION_SEGMENT) {
            ++nVehLeft;
            if (reason == MSMoveReminder::NOTIFICATION_TELEPORT || reason == MSMoveReminder::NOTIFICATION_TELEPORT_ARRIVED) {
                ++nVehTeleported;
            } else if (reason >= MSMoveReminder::NOTIFICATION_VAPORIZED_CALIBRATOR) {
                ++nVehVaporized;
            }
        }
    }
    if (MSGlobals::gUseMesoSim) {
        return false;
    }
    return reason == MSMoveReminder::NOTIFICATION_JUNCTION;
}


bool
MSMeanData_Net::MSLaneMeanDataValues::notifyEnter(SUMOTrafficObject& veh, MSMoveReminder::Notification reason, const MSLane* enteredLane) {
#ifdef DEBUG_NOTIFY_ENTER
    std::cout << "\n" << SIMTIME << " MSMeanData_Net::MSLaneMeanDataValues: veh '" << veh.getID() << "' enters lane '" << enteredLane->getID() << "'" << std::endl;
#else
    UNUSED_PARAMETER(enteredLane);
#endif
    if (myParent == nullptr || myParent->vehicleApplies(veh)) {
        if (getLane() == nullptr || !veh.isVehicle() || getLane() == static_cast<MSVehicle&>(veh).getLane()) {
#ifdef HAVE_FOX
            ScopedLocker<> lock(myNotificationMutex, MSGlobals::gNumSimThreads > 1);
#endif
            if (reason == MSMoveReminder::NOTIFICATION_DEPARTED) {
                ++nVehDeparted;
            } else if (reason == MSMoveReminder::NOTIFICATION_LANE_CHANGE) {
                ++nVehLaneChangeTo;
            } else if (myParent == nullptr || reason != MSMoveReminder::NOTIFICATION_SEGMENT) {
                ++nVehEntered;
            }
        }
        return true;
    }
    return false;
}


bool
MSMeanData_Net::MSLaneMeanDataValues::isEmpty() const {
    return sampleSeconds == 0 && nVehDeparted == 0 && nVehArrived == 0 && nVehEntered == 0
           && nVehLeft == 0 && nVehVaporized == 0 && nVehTeleported == 0 && nVehLaneChangeFrom == 0 && nVehLaneChangeTo == 0;
}

double
MSMeanData_Net::MSLaneMeanDataValues::getOccupancy(SUMOTime period, int numLanes) const {
    return occupationSum / STEPS2TIME(period) / myLaneLength / (double)numLanes * 100.;
}

void
MSMeanData_Net::MSLaneMeanDataValues::write(OutputDevice& dev, const SumoXMLAttrMask& attributeMask, const SUMOTime period,
        const int numLanes, const double speedLimit, const double defaultTravelTime, const int numVehicles) const {

    double density = frontSampleSeconds / STEPS2TIME(period) * 1000. / myLaneLength;
    double overlapDensity = sampleSeconds / STEPS2TIME(period) * 1000. / myLaneLength;
    if (MSGlobals::gLateralResolution < 0) {
        // avoid exceeding upper bound
        density = MIN2(density, 1000 * (double)numLanes / MAX2(minimalVehicleLength, NUMERICAL_EPS));
        overlapDensity = MIN2(overlapDensity, 1000 * (double)numLanes / MAX2(minimalVehicleLength, NUMERICAL_EPS));
    }
    const double laneDensity = density / (double)numLanes;
    const double occupancy = getOccupancy(period, numLanes);
#ifdef DEBUG_OCCUPANCY2
    // tests #3264
    if (occupancy > 100) {
        std::cout << SIMTIME << " Encountered bad occupancy: " << occupancy
                  << ", myLaneLength=" << myLaneLength << ", period=" << STEPS2TIME(period) << ", occupationSum=" << occupationSum
                  << std::endl;
    }
    // refs #3265
    std::cout << SIMTIME << "ID: " << getDescription() << " minVehicleLength=" << minimalVehicleLength
              << "\ndensity=" << density << "\n";
#endif

    if (myParent == nullptr) {
        const double speed = sampleSeconds == 0 ? 0. : travelledDistance / sampleSeconds;
        const double frontSpeed = frontSampleSeconds == 0 ? 0. : frontTravelledDistance / frontSampleSeconds;
        dev.writeOptionalAttr(SUMO_ATTR_DENSITY, density, attributeMask, sampleSeconds == 0);
        dev.writeOptionalAttr(SUMO_ATTR_LANEDENSITY, laneDensity, attributeMask, sampleSeconds == 0);
        dev.writeOptionalAttr(SUMO_ATTR_OCCUPANCY, occupancy, attributeMask, sampleSeconds == 0);
        dev.writeOptionalAttr(SUMO_ATTR_WAITINGTIME, waitSeconds, attributeMask, sampleSeconds == 0);
        dev.writeOptionalAttr(SUMO_ATTR_TIMELOSS, timeLoss, attributeMask, sampleSeconds == 0);
        dev.writeOptionalAttr(SUMO_ATTR_SPEED, speed, attributeMask, sampleSeconds == 0);
        dev.writeOptionalAttr(SUMO_ATTR_SPEEDREL, speedLimit == 0. || sampleSeconds == 0 ? 0. : travelledDistance / sampleSeconds / speedLimit,
                              attributeMask, sampleSeconds == 0);
        dev.writeOptionalAttr(SUMO_ATTR_DEPARTED, nVehDeparted, attributeMask);
        dev.writeOptionalAttr(SUMO_ATTR_ARRIVED, nVehArrived, attributeMask);
        dev.writeOptionalAttr(SUMO_ATTR_ENTERED, nVehEntered, attributeMask);
        dev.writeOptionalAttr(SUMO_ATTR_LEFT, nVehLeft, attributeMask);
        dev.writeOptionalAttr(SUMO_ATTR_VAPORIZED, nVehVaporized, attributeMask, nVehVaporized == 0);
        dev.writeOptionalAttr(SUMO_ATTR_TELEPORTED, nVehTeleported, attributeMask, nVehTeleported == 0);
        dev.writeOptionalAttr(SUMO_ATTR_FLOW, density * frontSpeed * 3.6, attributeMask, frontSampleSeconds == 0);
        dev.closeTag();
        return;
    }
    const bool haveSamples = sampleSeconds > myParent->myMinSamples;
    const bool haveFrontSamples = frontSampleSeconds > myParent->myMinSamples;
    const bool haveSamplesOrDefault = haveSamples || defaultTravelTime >= 0.;
    bool haveTravelTime = haveSamplesOrDefault;
    double traveltime = myParent->myMaxTravelTime;
    if (haveSamples) {
        if (numVehicles > 0) {
            traveltime = sampleSeconds / numVehicles;
        } else {
            traveltime = myParent->myMaxTravelTime;
            if (frontTravelledDistance > NUMERICAL_EPS) {
                traveltime = MIN2(traveltime, myLaneLength * frontSampleSeconds / frontTravelledDistance);
            } else if (defaultTravelTime >= 0.) {
                traveltime = defaultTravelTime;
            } else {
                haveTravelTime = false;
            }
        }
    } else if (defaultTravelTime >= 0.) {
        traveltime = defaultTravelTime;
    }
    dev.writeOptionalAttr(SUMO_ATTR_TRAVELTIME, traveltime, attributeMask, !haveTravelTime);
    double overlapTraveltime = myParent->myMaxTravelTime;
    if (travelledDistance > 0.) {
        // one vehicle has to drive lane length + vehicle length before it has left the lane
        // thus we need to scale with an extended length, approximated by lane length + average vehicle length
        overlapTraveltime = MIN2(overlapTraveltime, (myLaneLength + vehLengthSum / sampleSeconds) * sampleSeconds / travelledDistance);
    }
    dev.writeOptionalAttr(SUMO_ATTR_OVERLAPTRAVELTIME, overlapTraveltime, attributeMask, !haveSamples || numVehicles > 0);
    dev.writeOptionalAttr(SUMO_ATTR_DENSITY, density, attributeMask, !haveSamples || numVehicles > 0);
    dev.writeOptionalAttr(SUMO_ATTR_OVERLAPDENSITY, overlapDensity, attributeMask, sampleSeconds == 0);
    dev.writeOptionalAttr(SUMO_ATTR_LANEDENSITY, laneDensity, attributeMask, !haveSamples || numVehicles > 0);
    dev.writeOptionalAttr(SUMO_ATTR_OCCUPANCY, occupancy, attributeMask, !haveSamples || numVehicles > 0);
    dev.writeOptionalAttr(SUMO_ATTR_WAITINGTIME, waitSeconds, attributeMask, !haveSamples);
    dev.writeOptionalAttr(SUMO_ATTR_TIMELOSS, timeLoss, attributeMask, !haveSamples);
    double speed = 0.;
    double frontSpeed = 0.;
    if (haveSamples) {
        speed = travelledDistance / sampleSeconds;
    } else if (defaultTravelTime > 0.) {
        speed = myLaneLength / defaultTravelTime;
    }
    if (haveFrontSamples) {
        frontSpeed = frontTravelledDistance / frontSampleSeconds;
    } else if (defaultTravelTime > 0.) {
        frontSpeed = myLaneLength / defaultTravelTime;
    }
    dev.writeOptionalAttr(SUMO_ATTR_SPEED, speed, attributeMask, !haveSamplesOrDefault);
    dev.writeOptionalAttr(SUMO_ATTR_SPEEDREL, speedLimit == 0. ? 0. : speed / speedLimit, attributeMask, !haveSamplesOrDefault);
    dev.writeOptionalAttr(SUMO_ATTR_DEPARTED, nVehDeparted, attributeMask);
    dev.writeOptionalAttr(SUMO_ATTR_ARRIVED, nVehArrived, attributeMask);
    dev.writeOptionalAttr(SUMO_ATTR_ENTERED, nVehEntered, attributeMask);
    dev.writeOptionalAttr(SUMO_ATTR_LEFT, nVehLeft, attributeMask);
    dev.writeOptionalAttr(SUMO_ATTR_LANECHANGEDFROM, nVehLaneChangeFrom, attributeMask);
    dev.writeOptionalAttr(SUMO_ATTR_LANECHANGEDTO, nVehLaneChangeTo, attributeMask);
    dev.writeOptionalAttr(SUMO_ATTR_VAPORIZED, nVehVaporized, attributeMask, nVehVaporized == 0);
    dev.writeOptionalAttr(SUMO_ATTR_TELEPORTED, nVehTeleported, attributeMask, nVehTeleported == 0);
    dev.writeOptionalAttr(SUMO_ATTR_FLOW, density * frontSpeed * 3.6, attributeMask, !haveSamples || numVehicles == 0);
    dev.closeTag();
}


double
MSMeanData_Net::MSLaneMeanDataValues::getAttributeValue(SumoXMLAttr a,
        const SUMOTime period, const double numLanes, const double speedLimit) const {
    /// @todo: remove redundancy in derived values (density, laneDensity)
    switch (a) {
        case SUMO_ATTR_DENSITY:
            return MIN2(frontSampleSeconds / STEPS2TIME(period) * (double) 1000 / myLaneLength,
                        1000. * numLanes / MAX2(minimalVehicleLength, NUMERICAL_EPS));
        case SUMO_ATTR_LANEDENSITY: {
            const double density = MIN2(frontSampleSeconds / STEPS2TIME(period) * (double) 1000 / myLaneLength,
                                        1000. * numLanes / MAX2(minimalVehicleLength, NUMERICAL_EPS));
            return density / numLanes;
        }
        case SUMO_ATTR_OVERLAPDENSITY: {
            const double overlapDensity = MIN2(sampleSeconds / STEPS2TIME(period) * (double) 1000 / myLaneLength,
                                               1000. * numLanes / MAX2(minimalVehicleLength, NUMERICAL_EPS));
            return overlapDensity / numLanes;
        }
        case SUMO_ATTR_OCCUPANCY:
            return occupationSum / STEPS2TIME(period) / myLaneLength / numLanes * (double) 1000;
        case SUMO_ATTR_WAITINGTIME:
            return waitSeconds;
        case SUMO_ATTR_TIMELOSS:
            return timeLoss;
        case SUMO_ATTR_SPEED:
            return travelledDistance / sampleSeconds;
        case SUMO_ATTR_SPEEDREL:
            return speedLimit == 0. ? 0. : travelledDistance / sampleSeconds / speedLimit;
        case SUMO_ATTR_DEPARTED:
            return nVehDeparted;
        case SUMO_ATTR_ARRIVED:
            return nVehArrived;
        case SUMO_ATTR_ENTERED:
            return nVehEntered;
        case SUMO_ATTR_LEFT:
            return nVehLeft;
        case SUMO_ATTR_VAPORIZED:
            return nVehVaporized;
        case SUMO_ATTR_TELEPORTED:
            return nVehTeleported;
        case SUMO_ATTR_FLOW: {
            const double density = MIN2(frontSampleSeconds / STEPS2TIME(period) * (double) 1000 / myLaneLength,
                                        1000. * numLanes / MAX2(minimalVehicleLength, NUMERICAL_EPS));
            const double speed = frontTravelledDistance / frontSampleSeconds;
            return density * speed * 3.6;
        }
        default:
            return 0;
    }
}

// ---------------------------------------------------------------------------
// MSMeanData_Net - methods
// ---------------------------------------------------------------------------
MSMeanData_Net::MSMeanData_Net(const std::string& id,
                               const SUMOTime dumpBegin,
                               const SUMOTime dumpEnd, const bool useLanes,
                               const std::string& excludeEmpty,
                               const bool withInternal,
                               const bool trackVehicles,
                               const int detectPersons,
                               const double maxTravelTime,
                               const double minSamples,
                               const double haltSpeed,
                               const std::string& vTypes,
                               const std::string& writeAttributes,
                               const std::vector<MSEdge*>& edges,
                               AggregateType aggregate) :
    MSMeanData(id, dumpBegin, dumpEnd, useLanes, excludeEmpty,
               withInternal, trackVehicles, detectPersons, maxTravelTime, minSamples, vTypes, writeAttributes, edges, aggregate),
    myHaltSpeed(haltSpeed)
{ }


MSMeanData_Net::~MSMeanData_Net() {}


MSMeanData::MeanDataValues*
MSMeanData_Net::createValues(MSLane* const lane, const double length, const bool doAdd) const {
    return new MSLaneMeanDataValues(lane, length, doAdd, this);
}


std::vector<std::string>
MSMeanData_Net::getAttributeNames() const {
    std::vector<std::string> result;
    result.push_back(toString(SUMO_ATTR_DENSITY));
    result.push_back(toString(SUMO_ATTR_LANEDENSITY));
    result.push_back(toString(SUMO_ATTR_OCCUPANCY));
    result.push_back(toString(SUMO_ATTR_WAITINGTIME));
    result.push_back(toString(SUMO_ATTR_TIMELOSS));
    result.push_back(toString(SUMO_ATTR_SPEED));
    result.push_back(toString(SUMO_ATTR_SPEEDREL));
    result.push_back(toString(SUMO_ATTR_DEPARTED));
    result.push_back(toString(SUMO_ATTR_ARRIVED));
    result.push_back(toString(SUMO_ATTR_ENTERED));
    result.push_back(toString(SUMO_ATTR_LEFT));
    result.push_back(toString(SUMO_ATTR_VAPORIZED));
    result.push_back(toString(SUMO_ATTR_TELEPORTED));
    result.push_back(toString(SUMO_ATTR_FLOW));
    return result;
}


double
MSMeanData_Net::getAttributeValue(const MSLane* lane, SumoXMLAttr a, double defaultValue) const {
    double result = defaultValue;
    const std::vector<MeanDataValues*>* edgeValues = getEdgeValues(&lane->getEdge());
    if (edgeValues == nullptr) {
        return result;
    }
    MeanDataValues* values = nullptr;
    if (!myAmEdgeBased) {
        values = (*edgeValues)[lane->getIndex()];
    } else {
        MeanDataValues* sumData = createValues(nullptr, lane->getLength(), false);
        for (MeanDataValues* meanData : (*edgeValues)) {
            meanData->addTo(*sumData);
        }
        values = sumData;
    }
    const SUMOTime myLastResetTime = 0; // XXX store last reset time
    const SUMOTime period = SIMSTEP - myLastResetTime;
    result = values->getAttributeValue(a, period, lane->getEdge().getNumLanes(), lane->getSpeedLimit());
    if (myAmEdgeBased) {
        delete values;
    }
    return result;
}


/****************************************************************************/
