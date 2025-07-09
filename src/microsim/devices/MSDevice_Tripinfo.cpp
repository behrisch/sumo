/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2009-2025 German Aerospace Center (DLR) and others.
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
/// @file    MSDevice_Tripinfo.cpp
/// @author  Daniel Krajzewicz
/// @author  Laura Bieker
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    Fri, 30.01.2009
///
// A device which collects info on the vehicle trip
/****************************************************************************/
#include <config.h>

#include <microsim/MSGlobals.h>
#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSVehicle.h>
#include <microsim/transportables/MSTransportableControl.h>
#include <mesosim/MEVehicle.h>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/xml/SUMOSAXAttributes.h>
#include "MSDevice_Vehroutes.h"
#include "MSDevice_Tripinfo.h"

#define NOT_ARRIVED TIME2STEPS(-1)
#define STATE_EMPTY_ARRIVALLANE "NONE"


// ===========================================================================
// static members
// ===========================================================================
std::set<const MSDevice_Tripinfo*, ComparatorNumericalIdLess> MSDevice_Tripinfo::myPendingOutput;

int MSDevice_Tripinfo::myVehicleCount(0);
int MSDevice_Tripinfo::myUndepartedVehicleCount(0);
double MSDevice_Tripinfo::myTotalRouteLength(0);
double MSDevice_Tripinfo::myTotalSpeed(0);
SUMOTime MSDevice_Tripinfo::myTotalDuration(0);
SUMOTime MSDevice_Tripinfo::myTotalWaitingTime(0);
SUMOTime MSDevice_Tripinfo::myTotalTimeLoss(0);
SUMOTime MSDevice_Tripinfo::myTotalDepartDelay(0);
SUMOTime MSDevice_Tripinfo::myWaitingDepartDelay(-1);

int MSDevice_Tripinfo::myBikeCount(0);
double MSDevice_Tripinfo::myTotalBikeRouteLength(0);
double MSDevice_Tripinfo::myTotalBikeSpeed(0);
SUMOTime MSDevice_Tripinfo::myTotalBikeDuration(0);
SUMOTime MSDevice_Tripinfo::myTotalBikeWaitingTime(0);
SUMOTime MSDevice_Tripinfo::myTotalBikeTimeLoss(0);
SUMOTime MSDevice_Tripinfo::myTotalBikeDepartDelay(0);

int MSDevice_Tripinfo::myWalkCount(0);
double MSDevice_Tripinfo::myTotalWalkRouteLength(0);
SUMOTime MSDevice_Tripinfo::myTotalWalkDuration(0);
SUMOTime MSDevice_Tripinfo::myTotalWalkTimeLoss(0);
std::vector<int> MSDevice_Tripinfo::myRideCount({0, 0});
std::vector<int> MSDevice_Tripinfo::myRideBusCount({0, 0});
std::vector<int> MSDevice_Tripinfo::myRideRailCount({0, 0});
std::vector<int> MSDevice_Tripinfo::myRideTaxiCount({0, 0});
std::vector<int> MSDevice_Tripinfo::myRideBikeCount({0, 0});
std::vector<int> MSDevice_Tripinfo::myRideAbortCount({0, 0});
std::vector<SUMOTime> MSDevice_Tripinfo::myTotalRideWaitingTime({0, 0});
std::vector<double> MSDevice_Tripinfo::myTotalRideRouteLength({0., 0.});
std::vector<SUMOTime> MSDevice_Tripinfo::myTotalRideDuration({0, 0});

// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_Tripinfo::insertOptions(OptionsCont& oc) {
    oc.addOptionSubTopic("Tripinfo Device");
    insertDefaultAssignmentOptions("tripinfo", "Tripinfo Device", oc);
}


void
MSDevice_Tripinfo::buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into) {
    OptionsCont& oc = OptionsCont::getOptions();
    const bool enableByOutputOption = oc.isSet("tripinfo-output") || oc.getBool("duration-log.statistics");
    if (equippedByDefaultAssignmentOptions(oc, "tripinfo", v, enableByOutputOption)) {
        MSDevice_Tripinfo* device = new MSDevice_Tripinfo(v, "tripinfo_" + v.getID());
        into.push_back(device);
        myPendingOutput.insert(device);
    }
}


// ---------------------------------------------------------------------------
// MSDevice_Tripinfo-methods
// ---------------------------------------------------------------------------
MSDevice_Tripinfo::MSDevice_Tripinfo(SUMOVehicle& holder, const std::string& id) :
    MSVehicleDevice(holder, id),
    myDepartLane(""),
    myDepartSpeed(-1),
    myDepartPosLat(0),
    myWaitingTime(0),
    myAmWaiting(false),
    myWaitingCount(0),
    myStoppingTime(0),
    myParkingStarted(-1),
    myArrivalTime(NOT_ARRIVED),
    myArrivalLane(""),
    myArrivalPos(-1),
    myArrivalPosLat(0.),
    myArrivalSpeed(-1),
    myArrivalReason(MSMoveReminder::NOTIFICATION_ARRIVED),
    myMesoTimeLoss(0),
    myRouteLength(0.) {
}


MSDevice_Tripinfo::~MSDevice_Tripinfo() {
    // ensure clean up for vaporized vehicles which do not generate output
    myPendingOutput.erase(this);
}

void
MSDevice_Tripinfo::cleanup() {
    myVehicleCount = 0;
    myTotalRouteLength = 0;
    myTotalSpeed = 0;
    myTotalDuration = 0;
    myTotalWaitingTime = 0;
    myTotalTimeLoss = 0;
    myTotalDepartDelay = 0;
    myWaitingDepartDelay = -1;

    myBikeCount = 0;
    myTotalBikeRouteLength = 0;
    myTotalBikeSpeed = 0;
    myTotalBikeDuration = 0;
    myTotalBikeWaitingTime = 0;
    myTotalBikeTimeLoss = 0;
    myTotalBikeDepartDelay = 0;

    myWalkCount = 0;
    myTotalWalkRouteLength = 0;
    myTotalWalkDuration = 0;
    myTotalWalkTimeLoss = 0;

    myRideCount = {0, 0};
    myRideBusCount = {0, 0};
    myRideRailCount = {0, 0};
    myRideTaxiCount = {0, 0};
    myRideBikeCount = {0, 0};
    myRideAbortCount = {0, 0};
    myTotalRideWaitingTime = {0, 0};
    myTotalRideRouteLength = {0., 0.};
    myTotalRideDuration = {0, 0};
}

bool
MSDevice_Tripinfo::notifyIdle(SUMOTrafficObject& veh) {
    if (veh.isVehicle()) {
        myWaitingTime += DELTA_T;
        if (!myAmWaiting) {
            myWaitingCount++;
            myAmWaiting = true;
        }
    }
    return true;
}


bool
MSDevice_Tripinfo::notifyMove(SUMOTrafficObject& veh, double /*oldPos*/,
                              double /*newPos*/, double newSpeed) {
    if (veh.isStopped()) {
        if (newSpeed <= SUMO_const_haltingSpeed) {
            myStoppingTime += DELTA_T;
        }
    } else if (newSpeed <= SUMO_const_haltingSpeed && lowAcceleration(veh)) {
        myWaitingTime += DELTA_T;
        if (!myAmWaiting) {
            myWaitingCount++;
            myAmWaiting = true;
        }
    } else {
        myAmWaiting = false;
    }
    return true;
}


bool
MSDevice_Tripinfo::lowAcceleration(const SUMOTrafficObject& veh) {
    if (MSGlobals::gUseMesoSim) {
        // acceleration is not modelled
        return false;
    } else {
        const MSVehicle& v = dynamic_cast<const MSVehicle&>(veh);
        return v.getAcceleration() <= v.accelThresholdForWaiting();
    }
}


void
MSDevice_Tripinfo::notifyMoveInternal(const SUMOTrafficObject& veh,
                                      const double /* frontOnLane */,
                                      const double timeOnLane,
                                      const double /* meanSpeedFrontOnLane */,
                                      const double meanSpeedVehicleOnLane,
                                      const double /* travelledDistanceFrontOnLane */,
                                      const double /* travelledDistanceVehicleOnLane */,
                                      const double /* meanLengthOnLane */) {

    // called by meso
    const double vmax = veh.getEdge()->getVehicleMaxSpeed(&veh);
    if (vmax > 0) {
        myMesoTimeLoss += TIME2STEPS(timeOnLane * (vmax - meanSpeedVehicleOnLane) / vmax);
    }
    myWaitingTime += veh.getWaitingTime();
}


void
MSDevice_Tripinfo::updateParkingStopTime() {
    if (myParkingStarted >= 0) {
        myStoppingTime += (MSNet::getInstance()->getCurrentTimeStep() - myParkingStarted);
        myParkingStarted = -1;
    }
}

bool
MSDevice_Tripinfo::notifyEnter(SUMOTrafficObject& veh, MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
    if (reason == MSMoveReminder::NOTIFICATION_DEPARTED) {
        if (!MSGlobals::gUseMesoSim) {
            myDepartLane = static_cast<MSVehicle&>(veh).getLane()->getID();
            myDepartPosLat = static_cast<MSVehicle&>(veh).getLateralPositionOnLane();
        } else {
            myDepartLane = veh.getEdge()->getFirstAllowed(veh.getVClass(), true)->getID();
        }
        myDepartSpeed = veh.getSpeed();
        myRouteLength = -veh.getPositionOnLane();
    } else if (reason == MSMoveReminder::NOTIFICATION_PARKING) {
        // notifyMove is not called while parking
        // @note insertion delay when resuming after parking is included
        updateParkingStopTime();
    }
    return true;
}


bool
MSDevice_Tripinfo::notifyLeave(SUMOTrafficObject& veh, double /*lastPos*/,
                               MSMoveReminder::Notification reason, const MSLane* /* enteredLane */) {
    if (reason >= MSMoveReminder::NOTIFICATION_ARRIVED) {
        myArrivalTime = MSNet::getInstance()->getCurrentTimeStep();
        myArrivalReason = reason;
        if (!MSGlobals::gUseMesoSim) {
            myArrivalLane = static_cast<MSVehicle&>(veh).getLane()->getID();
            myArrivalPosLat = static_cast<MSVehicle&>(veh).getLateralPositionOnLane();
        } else {
            myArrivalLane = veh.getEdge()->getFirstAllowed(veh.getVClass(), true)->getID();
        }
        // @note vehicle may have moved past its arrivalPos during the last step
        // due to non-zero arrivalspeed but we consider it as arrived at the desired position
        // However, vaporization may happen anywhere (via TraCI)
        if (reason > MSMoveReminder::NOTIFICATION_TELEPORT_ARRIVED) {
            // vaporized
            myArrivalPos = veh.getPositionOnLane();
        } else {
            myArrivalPos = myHolder.getArrivalPos();
        }
        myArrivalSpeed = veh.getSpeed();
        updateParkingStopTime();
    } else if (reason == MSMoveReminder::NOTIFICATION_PARKING) {
        myParkingStarted = MSNet::getInstance()->getCurrentTimeStep();
    } else if (reason == NOTIFICATION_JUNCTION
               || reason == NOTIFICATION_TELEPORT
               || reason == NOTIFICATION_TELEPORT_CONTINUATION) {
        if (MSGlobals::gUseMesoSim) {
            myRouteLength += myHolder.getEdge()->getLength();
        } else {
            const MSLane* lane = static_cast<MSVehicle&>(veh).getLane();
            if (lane != nullptr) {
                myRouteLength += lane->getLength();
            }
        }
    }
    return true;
}


void
MSDevice_Tripinfo::generateOutput(OutputDevice* tripinfoOut) const {
    const SUMOTime timeLoss = MSGlobals::gUseMesoSim ? myMesoTimeLoss : static_cast<MSVehicle&>(myHolder).getTimeLoss();
    const double routeLength = myRouteLength + (myArrivalTime == NOT_ARRIVED ? myHolder.getPositionOnLane() : myArrivalPos);
    SUMOTime duration = 0;
    if (myHolder.hasDeparted()) {
        duration = (myArrivalTime == NOT_ARRIVED ? SIMSTEP : myArrivalTime) - myHolder.getDeparture();
        if (myHolder.getVClass() == SVC_BICYCLE) {
            myBikeCount++;
            myTotalBikeRouteLength += routeLength;
            myTotalBikeSpeed += routeLength / STEPS2TIME(duration);
            myTotalBikeDuration += duration;
            myTotalBikeWaitingTime += myWaitingTime;
            myTotalBikeTimeLoss += timeLoss;
            myTotalBikeDepartDelay += myHolder.getDepartDelay();
        } else {
            myVehicleCount++;
            myTotalRouteLength += routeLength;
            myTotalSpeed += routeLength / STEPS2TIME(duration);
            myTotalDuration += duration;
            myTotalWaitingTime += myWaitingTime;
            myTotalTimeLoss += timeLoss;
            myTotalDepartDelay += myHolder.getDepartDelay();
        }
    }

    myPendingOutput.erase(this);
    if (tripinfoOut == nullptr) {
        return;
    }
    // write
    OutputDevice& os = *tripinfoOut;
    os.openTag("tripinfo").writeAttr("id", myHolder.getID());
    os.writeAttr("depart", myHolder.hasDeparted() ? time2string(myHolder.getDeparture()) : "-1");
    os.writeAttr("departLane", myDepartLane);
    os.writeAttr("departPos", myHolder.getDepartPos());
    if (MSGlobals::gLateralResolution > 0) {
        os.writeAttr("departPosLat", myDepartPosLat);
    }
    os.writeAttr("departSpeed", myDepartSpeed);
    SUMOTime departDelay = myHolder.getDepartDelay();
    const SUMOVehicleParameter& param = myHolder.getParameter();
    if (!myHolder.hasDeparted()) {
        assert(param.depart <= SIMSTEP || param.departProcedure != DepartDefinition::GIVEN);
        departDelay = SIMSTEP - param.depart;
    }
    os.writeAttr("departDelay", time2string(departDelay));
    os.writeAttr("arrival", time2string(myArrivalTime));
    os.writeAttr("arrivalLane", myArrivalLane);
    os.writeAttr("arrivalPos", myArrivalPos);
    if (MSGlobals::gLateralResolution > 0) {
        os.writeAttr("arrivalPosLat", myArrivalPosLat);
    }
    os.writeAttr("arrivalSpeed", myArrivalSpeed);
    os.writeAttr("duration", time2string(duration));
    os.writeAttr("routeLength", routeLength);
    os.writeAttr(SUMO_ATTR_WAITINGTIME, time2string(myWaitingTime));
    os.writeAttr(SUMO_ATTR_WAITINGCOUNT, myWaitingCount);
    os.writeAttr(SUMO_ATTR_STOPTIME, time2string(myStoppingTime));
    os.writeAttr(SUMO_ATTR_TIMELOSS, time2string(timeLoss));
    os.writeAttr("rerouteNo", myHolder.getNumberReroutes());
    os.writeAttr("devices", toString(myHolder.getDevices()));
    os.writeAttr("vType", myHolder.getVehicleType().getID());
    os.writeAttr("speedFactor", myHolder.getChosenSpeedFactor());
    std::string vaporized;
    switch (myArrivalReason) {
        case MSMoveReminder::NOTIFICATION_VAPORIZED_CALIBRATOR:
            vaporized = "calibrator";
            break;
        case MSMoveReminder::NOTIFICATION_VAPORIZED_GUI:
            vaporized = "gui";
            break;
        case MSMoveReminder::NOTIFICATION_VAPORIZED_COLLISION:
            vaporized = "collision";
            break;
        case MSMoveReminder::NOTIFICATION_VAPORIZED_VAPORIZER:
            vaporized = "vaporizer";
            break;
        case MSMoveReminder::NOTIFICATION_VAPORIZED_TRACI:
            vaporized = "traci";
            break;
        case MSMoveReminder::NOTIFICATION_TELEPORT_ARRIVED:
            vaporized = "teleport";
            break;
        default:
            if (myHolder.getEdge() == myHolder.getRoute().getLastEdge() ||
                    (param.arrivalEdge >= 0 && myHolder.getRoutePosition() >= param.arrivalEdge)) {
                vaporized = "";
            } else {
                vaporized = "end";
            }
            break;
    }
    os.writeAttr("vaporized", vaporized);
    // cannot close tag because emission device output might follow
}


void
MSDevice_Tripinfo::generateOutputForUnfinished() {
    MSNet* net = MSNet::getInstance();
    OutputDevice* tripinfoOut = (OptionsCont::getOptions().isSet("tripinfo-output") ?
                                 &OutputDevice::getDeviceByOption("tripinfo-output") : nullptr);
    myWaitingDepartDelay = 0;
    myUndepartedVehicleCount = 0;
    const bool writeUndeparted = OptionsCont::getOptions().getBool("tripinfo-output.write-undeparted");
    const SUMOTime t = net->getCurrentTimeStep();
    while (myPendingOutput.size() > 0) {
        const MSDevice_Tripinfo* d = *myPendingOutput.begin();
        const bool departed = d->myHolder.hasDeparted();
        const bool departDelayed = d->myHolder.getParameter().depart <= t;
        if (!departed && departDelayed) {
            myUndepartedVehicleCount++;
            myWaitingDepartDelay += (t - d->myHolder.getParameter().depart);
        }
        if (departed || (writeUndeparted && departDelayed)) {
            const_cast<MSDevice_Tripinfo*>(d)->updateParkingStopTime();
            d->generateOutput(tripinfoOut);
            if (tripinfoOut != nullptr) {
                for (MSVehicleDevice* const dev : d->myHolder.getDevices()) {
                    if (typeid(*dev) == typeid(MSDevice_Tripinfo) || typeid(*dev) == typeid(MSDevice_Vehroutes)) {
                        // tripinfo is special and vehroute has its own write-unfinished option
                        continue;
                    }
                    dev->generateOutput(tripinfoOut);
                }
                OutputDevice::getDeviceByOption("tripinfo-output").closeTag();
            }
        } else {
            myPendingOutput.erase(d);
        }
    }
    // unfinished persons
    if (net->hasPersons()) {
        MSTransportableControl& pc = net->getPersonControl();
        while (pc.loadedBegin() != pc.loadedEnd()) {
            pc.erase(pc.loadedBegin()->second);
        }
    }

}


void
MSDevice_Tripinfo::addPedestrianData(double walkLength, SUMOTime walkDuration, SUMOTime walkTimeLoss) {
    myWalkCount++;
    myTotalWalkRouteLength += walkLength;
    myTotalWalkDuration += walkDuration;
    myTotalWalkTimeLoss += walkTimeLoss;
}


void
MSDevice_Tripinfo::addRideTransportData(const bool isPerson, const double distance, const SUMOTime duration,
                                        const SUMOVehicleClass vClass, const std::string& line, const SUMOTime waitingTime) {
    const int index = isPerson ? 0 : 1;
    myRideCount[index]++;
    if (duration > 0) {
        myTotalRideWaitingTime[index] += waitingTime;
        myTotalRideRouteLength[index] += distance;
        myTotalRideDuration[index] += duration;
        if (vClass == SVC_BICYCLE) {
            myRideBikeCount[index]++;
        } else if (!line.empty()) {
            if (isRailway(vClass)) {
                myRideRailCount[index]++;
            } else if (vClass == SVC_TAXI) {
                myRideTaxiCount[index]++;
            } else {
                // some kind of road vehicle
                myRideBusCount[index]++;
            }
        }
    } else {
        myRideAbortCount[index]++;
    }
}


std::string
MSDevice_Tripinfo::printStatistics() {
    std::ostringstream msg;
    msg.setf(msg.fixed);
    msg.precision(gPrecision);
    if (myBikeCount == 0 || myVehicleCount > 0) {
        msg << "Statistics (avg of " << myVehicleCount << "):\n";
        msg << " RouteLength: " << getAvgRouteLength() << "\n"
            << " Speed: " << getAvgTripSpeed() << "\n"
            << " Duration: " << getAvgDuration() << "\n"
            << " WaitingTime: " << getAvgWaitingTime() << "\n"
            << " TimeLoss: " << getAvgTimeLoss() << "\n"
            << " DepartDelay: " << getAvgDepartDelay() << "\n";
    }
    if (myBikeCount > 0) {
        msg << "Bike Statistics (avg of " << myBikeCount << "):\n"
            << " RouteLength: " << getAvgBikeRouteLength() << "\n"
            << " Speed: " << getAvgBikeTripSpeed() << "\n"
            << " Duration: " << getAvgBikeDuration() << "\n"
            << " WaitingTime: " << getAvgBikeWaitingTime() << "\n"
            << " TimeLoss: " << getAvgBikeTimeLoss() << "\n"
            << " DepartDelay: " << getAvgBikeDepartDelay() << "\n";
        if (myVehicleCount > 0 && myWaitingDepartDelay >= 0) {
            msg << "Statistics (avg of " << (myVehicleCount + myBikeCount) << "):\n";
        }
    }
    if (myWaitingDepartDelay >= 0) {
        msg << " DepartDelayWaiting: " << getAvgDepartDelayWaiting() << "\n";
    }
    if (myWalkCount > 0) {
        msg << "Pedestrian Statistics (avg of " << myWalkCount << " walks):\n"
            << " RouteLength: " << getAvgWalkRouteLength() << "\n"
            << " Duration: " << getAvgWalkDuration() << "\n"
            << " TimeLoss: " << getAvgWalkTimeLoss() << "\n";
    }
    printRideStatistics(msg, "Ride", "rides", 0);
    printRideStatistics(msg, "Transport", "transports", 1);
    return msg.str();
}

void
MSDevice_Tripinfo::printRideStatistics(std::ostringstream& msg, const std::string& category, const std::string& modeName, const int index) {
    if (myRideCount[index] > 0) {
        msg << category << " Statistics (avg of " << myRideCount[index] << " " << modeName << "):\n";
        msg << " WaitingTime: " << STEPS2TIME(myTotalRideWaitingTime[index] / myRideCount[index]) << "\n";
        msg << " RouteLength: " << myTotalRideRouteLength[index] / myRideCount[index] << "\n";
        msg << " Duration: " << STEPS2TIME(myTotalRideDuration[index] / myRideCount[index]) << "\n";
        if (myRideBusCount[index] > 0) {
            msg << " Bus: " << myRideBusCount[index] << "\n";
        }
        if (myRideRailCount[index] > 0) {
            msg << " Train: " << myRideRailCount[index] << "\n";
        }
        if (myRideTaxiCount[index] > 0) {
            msg << " Taxi: " << myRideTaxiCount[index] << "\n";
        }
        if (myRideBikeCount[index] > 0) {
            msg << " Bike: " << myRideBikeCount[index] << "\n";
        }
        if (myRideAbortCount[index] > 0) {
            msg << " Aborted: " << myRideAbortCount[index] << "\n";
        }
    }

}


void
MSDevice_Tripinfo::writeStatistics(OutputDevice& od) {
    od.setPrecision(gPrecision);
    od.openTag("vehicleTripStatistics");
    od.writeAttr("count", myVehicleCount);
    od.writeAttr("routeLength", getAvgRouteLength());
    od.writeAttr("speed", getAvgTripSpeed());
    od.writeAttr("duration", getAvgDuration());
    od.writeAttr("waitingTime", getAvgWaitingTime());
    od.writeAttr("timeLoss", getAvgTimeLoss());
    od.writeAttr("departDelay", getAvgDepartDelay());
    od.writeAttr("departDelayWaiting", getAvgDepartDelayWaiting());
    od.writeAttr("totalTravelTime", time2string(myTotalDuration));
    od.writeAttr("totalDepartDelay", time2string(TIME2STEPS(getTotalDepartDelay() + getTotalBikeDepartDelay())));
    od.closeTag();
    if (myBikeCount > 0) {
        od.openTag("bikeTripStatistics");
        od.writeAttr("count", myBikeCount);
        od.writeAttr("routeLength", getAvgBikeRouteLength());
        od.writeAttr("speed", getAvgBikeTripSpeed());
        od.writeAttr("duration", getAvgBikeDuration());
        od.writeAttr("waitingTime", getAvgBikeWaitingTime());
        od.writeAttr("timeLoss", getAvgBikeTimeLoss());
        od.writeAttr("departDelay", getAvgBikeDepartDelay());
        od.writeAttr("totalTravelTime", time2string(myTotalBikeDuration));
        od.closeTag();
    }
    od.openTag("pedestrianStatistics");
    od.writeAttr("number", myWalkCount);
    od.writeAttr("routeLength", getAvgWalkRouteLength());
    od.writeAttr("duration", getAvgWalkDuration());
    od.writeAttr("timeLoss", getAvgWalkTimeLoss());
    od.closeTag();
    writeRideStatistics(od, "rideStatistics", 0);
    writeRideStatistics(od, "transportStatistics", 1);
}

void
MSDevice_Tripinfo::writeRideStatistics(OutputDevice& od, const std::string& category, const int index) {
    od.openTag(category);
    od.writeAttr("number", myRideCount[index]);
    if (myRideCount[index] > 0) {
        od.writeAttr("waitingTime", STEPS2TIME(myTotalRideWaitingTime[index] / myRideCount[index]));
        od.writeAttr("routeLength", myTotalRideRouteLength[index] / myRideCount[index]);
        od.writeAttr("duration", STEPS2TIME(myTotalRideDuration[index] / myRideCount[index]));
        od.writeAttr("bus", myRideBusCount[index]);
        od.writeAttr("train", myRideRailCount[index]);
        od.writeAttr("taxi", myRideTaxiCount[index]);
        od.writeAttr("bike", myRideBikeCount[index]);
        od.writeAttr("aborted", myRideAbortCount[index]);
    }
    od.closeTag();
}


double
MSDevice_Tripinfo::getAvgRouteLength() {
    if (myVehicleCount > 0) {
        return myTotalRouteLength / myVehicleCount;
    } else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgTripSpeed() {
    if (myVehicleCount > 0) {
        return myTotalSpeed / myVehicleCount;
    } else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgDuration() {
    if (myVehicleCount > 0) {
        return STEPS2TIME(myTotalDuration / myVehicleCount);
    } else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgWaitingTime() {
    if (myVehicleCount > 0) {
        return STEPS2TIME(myTotalWaitingTime / myVehicleCount);
    } else {
        return 0;
    }
}


double
MSDevice_Tripinfo::getAvgTimeLoss() {
    if (myVehicleCount > 0) {
        return STEPS2TIME(myTotalTimeLoss / myVehicleCount);
    } else {
        return 0;
    }
}


double
MSDevice_Tripinfo::getAvgDepartDelay() {
    if (myVehicleCount > 0) {
        return STEPS2TIME(myTotalDepartDelay / myVehicleCount);
    } else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgDepartDelayWaiting() {
    if (myWaitingDepartDelay >= 0) {
        return STEPS2TIME(myWaitingDepartDelay / MAX2(1, myUndepartedVehicleCount));
    } else {
        return -1;
    }
}


double
MSDevice_Tripinfo::getTotalDepartDelay() {
    return STEPS2TIME(myTotalDepartDelay + MAX2((SUMOTime)0, myWaitingDepartDelay));
}

double
MSDevice_Tripinfo::getAvgBikeRouteLength() {
    if (myBikeCount > 0) {
        return myTotalBikeRouteLength / myBikeCount;
    } else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgBikeTripSpeed() {
    if (myBikeCount > 0) {
        return myTotalBikeSpeed / myBikeCount;
    } else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgBikeDuration() {
    if (myBikeCount > 0) {
        return STEPS2TIME(myTotalBikeDuration / myBikeCount);
    } else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgBikeWaitingTime() {
    if (myBikeCount > 0) {
        return STEPS2TIME(myTotalBikeWaitingTime / myBikeCount);
    } else {
        return 0;
    }
}


double
MSDevice_Tripinfo::getAvgBikeTimeLoss() {
    if (myBikeCount > 0) {
        return STEPS2TIME(myTotalBikeTimeLoss / myBikeCount);
    } else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgBikeDepartDelay() {
    if (myBikeCount > 0) {
        return STEPS2TIME(myTotalBikeDepartDelay / myBikeCount);
    } else {
        return 0;
    }
}


double
MSDevice_Tripinfo::getTotalBikeDepartDelay() {
    return STEPS2TIME(myTotalBikeDepartDelay);
}

double
MSDevice_Tripinfo::getAvgWalkRouteLength() {
    if (myWalkCount > 0) {
        return myTotalWalkRouteLength / myWalkCount;
    } else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgWalkDuration() {
    if (myWalkCount > 0) {
        return STEPS2TIME(myTotalWalkDuration / myWalkCount);
    } else {
        return 0;
    }
}


double
MSDevice_Tripinfo::getAvgWalkTimeLoss() {
    if (myWalkCount > 0) {
        return STEPS2TIME(myTotalWalkTimeLoss / myWalkCount);
    } else {
        return 0;
    }
}


double
MSDevice_Tripinfo::getAvgRideDuration() {
    if (myRideCount[0] > 0) {
        return STEPS2TIME(myTotalRideDuration[0] / myRideCount[0]);
    } else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgRideWaitingTime() {
    if (myRideCount[0] > 0) {
        return STEPS2TIME(myTotalRideWaitingTime[0] / myRideCount[0]);
    } else {
        return 0;
    }
}

double
MSDevice_Tripinfo::getAvgRideRouteLength() {
    if (myRideCount[0] > 0) {
        return myTotalRideRouteLength[0] / myRideCount[0];
    } else {
        return 0;
    }
}


std::string
MSDevice_Tripinfo::getParameter(const std::string& key) const {
    if (key == toString(SUMO_ATTR_WAITINGTIME)) {
        return toString(STEPS2TIME(myWaitingTime));
    } else if (key == toString(SUMO_ATTR_WAITINGCOUNT)) {
        return toString(myWaitingCount);
    } else if (key == toString(SUMO_ATTR_STOPTIME)) {
        return toString(STEPS2TIME(myStoppingTime));
    } else if (key == toString(SUMO_ATTR_ARRIVALTIME)) {
        return toString(STEPS2TIME(myArrivalTime));
    } else if (key == toString(SUMO_ATTR_ARRIVALLANE)) {
        return toString(myArrivalLane);
    } else if (key == toString(SUMO_ATTR_ARRIVALPOS)) {
        return toString(myArrivalPos);
    } else if (key == toString(SUMO_ATTR_ARRIVALPOS_LAT)) {
        return toString(myArrivalPosLat);
    } else if (key == toString(SUMO_ATTR_ARRIVALSPEED)) {
        return toString(myArrivalSpeed);
    }
    throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
}


std::string
MSDevice_Tripinfo::getGlobalParameter(const std::string& prefixedKey) {
    std::string key = prefixedKey; // by default, assume vehicleTripStatistics;
    const std::string err = "Parameter '" + prefixedKey + "' is not supported for device of type 'tripinfo'";
    if (StringUtils::startsWith(key, "vehicleTripStatistics.")) {
        key = prefixedKey.substr(22);
    } else if (StringUtils::startsWith(key, "bikeTripStatistics.")) {
        key = prefixedKey.substr(19);
        if (key == toString(SUMO_ATTR_COUNT)) {
            return toString(myBikeCount);
        } else if (key == "routeLength") {
            return toString(getAvgBikeRouteLength());
        } else if (key == toString(SUMO_ATTR_SPEED)) {
            return toString(getAvgBikeTripSpeed());
        } else if (key == toString(SUMO_ATTR_DURATION)) {
            return toString(getAvgBikeDuration());
        } else if (key == toString(SUMO_ATTR_WAITINGTIME)) {
            return toString(getAvgBikeWaitingTime());
        } else if (key == toString(SUMO_ATTR_TIMELOSS)) {
            return toString(getAvgBikeTimeLoss());
        } else if (key == "departDelay") {
            return toString(getAvgBikeDepartDelay());
        } else if (key == "totalTravelTime") {
            // avoid human readable output
            return toString(STEPS2TIME((myTotalBikeDuration)));
        }
        throw InvalidArgument(err);

    } else if (StringUtils::startsWith(key, "pedestrianStatistics.")) {
        key = prefixedKey.substr(21);
        if (key == toString(SUMO_ATTR_NUMBER) || key == toString(SUMO_ATTR_COUNT)) {
            return toString(myWalkCount);
        } else if (key == "routeLength") {
            return toString(getAvgWalkRouteLength());
        } else if (key == toString(SUMO_ATTR_DURATION)) {
            return toString(getAvgWalkDuration());
        } else if (key == toString(SUMO_ATTR_TIMELOSS)) {
            return toString(getAvgWalkTimeLoss());
        }
        throw InvalidArgument(err);

    } else if (StringUtils::startsWith(key, "rideStatistics.") ||
               StringUtils::startsWith(key, "transportStatistics.")) {
        int index = 0;
        if (StringUtils::startsWith(key, "rideStatistics.")) {
            key = prefixedKey.substr(15);
        } else {
            index = 1;
            key = prefixedKey.substr(20);
        }
        if (key == toString(SUMO_ATTR_NUMBER) || key == toString(SUMO_ATTR_COUNT)) {
            return toString(myRideCount[index]);
        } else if (key == toString(SUMO_ATTR_WAITINGTIME)) {
            return toString(STEPS2TIME(myTotalRideWaitingTime[index] / MAX2(1, myRideCount[index])));
        } else if (key == "routeLength") {
            return toString(myTotalRideRouteLength[index] / MAX2(1, myRideCount[index]));
        } else if (key == toString(SUMO_ATTR_DURATION)) {
            return toString(myTotalRideRouteLength[index] / MAX2(1, myRideCount[index]));
        } else if (key == "bus") {
            return toString(myRideBusCount[index]);
        } else if (key == "train") {
            return toString(myRideRailCount[index]);
        } else if (key == "taxi") {
            return toString(myRideTaxiCount[index]);
        } else if (key == "bike") {
            return toString(myRideBikeCount[index]);
        } else if (key == "aborted") {
            return toString(myRideAbortCount[index]);
        }
        throw InvalidArgument(err);
    }
    // vehicleTripStatistics
    if (key == toString(SUMO_ATTR_COUNT)) {
        return toString(myVehicleCount);
    } else if (key == "routeLength") {
        return toString(getAvgRouteLength());
    } else if (key == toString(SUMO_ATTR_SPEED)) {
        return toString(getAvgTripSpeed());
    } else if (key == toString(SUMO_ATTR_DURATION)) {
        return toString(getAvgDuration());
    } else if (key == toString(SUMO_ATTR_WAITINGTIME)) {
        return toString(getAvgWaitingTime());
    } else if (key == toString(SUMO_ATTR_TIMELOSS)) {
        return toString(getAvgTimeLoss());
    } else if (key == "departDelay") {
        return toString(getAvgDepartDelay());
    } else if (key == "departDelayWaiting") {
        return toString(getAvgDepartDelayWaiting());
    } else if (key == "totalTravelTime") {
        // avoid human readable output
        return toString(STEPS2TIME((myTotalDuration)));
    } else if (key == "totalDepartDelay") {
        return toString(getTotalDepartDelay());
    }
    throw InvalidArgument(err);
}


void
MSDevice_Tripinfo::saveState(OutputDevice& out) const {
    // always write device id to replicate stochastic assignment
    out.openTag(SUMO_TAG_DEVICE);
    out.writeAttr(SUMO_ATTR_ID, getID());
    if (myHolder.hasDeparted()) {
        std::ostringstream internals;
        internals << myDepartLane << " ";
        if (!MSGlobals::gUseMesoSim) {
            internals << myDepartPosLat << " ";
        }
        std::string state_arrivalLane = myArrivalLane == "" ? STATE_EMPTY_ARRIVALLANE : myArrivalLane;
        internals << myDepartSpeed << " " << myRouteLength << " " << myWaitingTime << " " << myAmWaiting << " " << myWaitingCount << " ";
        internals << myStoppingTime << " " << myParkingStarted << " ";
        internals << myArrivalTime << " " << state_arrivalLane << " " << myArrivalPos << " " << myArrivalPosLat << " " << myArrivalSpeed;
        out.writeAttr(SUMO_ATTR_STATE, internals.str());
    }
    out.closeTag();
}


void
MSDevice_Tripinfo::loadState(const SUMOSAXAttributes& attrs) {
    if (attrs.hasAttribute(SUMO_ATTR_STATE)) {
        std::istringstream bis(attrs.getString(SUMO_ATTR_STATE));
        bis >> myDepartLane;
        if (!MSGlobals::gUseMesoSim) {
            bis >> myDepartPosLat;
        }
        bis >> myDepartSpeed >> myRouteLength >> myWaitingTime >> myAmWaiting >> myWaitingCount;
        bis >> myStoppingTime >> myParkingStarted;
        bis >> myArrivalTime >> myArrivalLane >> myArrivalPos >> myArrivalPosLat >> myArrivalSpeed;
        if (myArrivalLane == STATE_EMPTY_ARRIVALLANE) {
            myArrivalLane = "";
        }
    }
}


/****************************************************************************/
