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
/// @file    MEVehicle.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Tue, May 2005
///
// A vehicle from the mesoscopic point of view
/****************************************************************************/
#include <config.h>

#include <iostream>
#include <cassert>
#include <utils/common/StdDefs.h>
#include <utils/common/FileHelpers.h>
#include <utils/common/MsgHandler.h>
#include <utils/geom/GeomHelper.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/xml/SUMOSAXAttributes.h>
#include <microsim/devices/MSDevice_Tripinfo.h>
#include <microsim/devices/MSDevice_Vehroutes.h>
#include <microsim/devices/MSDevice_Taxi.h>
#include <microsim/output/MSStopOut.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSNet.h>
#include <microsim/MSVehicleType.h>
#include <microsim/MSLink.h>
#include <microsim/MSStop.h>
#include <microsim/MSVehicleControl.h>
#include <microsim/transportables/MSTransportableControl.h>
#include <microsim/devices/MSDevice.h>
#include "MELoop.h"
#include "MEVehicle.h"
#include "MESegment.h"


// ===========================================================================
// method definitions
// ===========================================================================
MEVehicle::MEVehicle(SUMOVehicleParameter* pars, ConstMSRoutePtr route,
                     MSVehicleType* type, const double speedFactor) :
    MSBaseVehicle(pars, route, type, speedFactor),
    mySegment(nullptr),
    myQueIndex(0),
    myEventTime(SUMOTime_MIN),
    myLastEntryTime(SUMOTime_MIN),
    myBlockTime(SUMOTime_MAX),
    myInfluencer(nullptr) {
}


double
MEVehicle::getBackPositionOnLane(const MSLane* /* lane */) const {
    return getPositionOnLane() - getVehicleType().getLength();
}


double
MEVehicle::getPositionOnLane() const {
// the following interpolation causes problems with arrivals and calibrators
//    const double fracOnSegment = MIN2(double(1), STEPS2TIME(MSNet::getInstance()->getCurrentTimeStep() - myLastEntryTime) / STEPS2TIME(myEventTime - myLastEntryTime));
    return mySegment == nullptr ? 0 : (double(mySegment->getIndex()) /* + fracOnSegment */) * mySegment->getLength();
}


double
MEVehicle::getAngle() const {
    const MSLane* const lane = getEdge()->getLanes()[0];
    return lane->getShape().rotationAtOffset(lane->interpolateLanePosToGeometryPos(getPositionOnLane()));
}


double
MEVehicle::getSlope() const {
    const MSLane* const lane = getEdge()->getLanes()[0];
    return lane->getShape().slopeDegreeAtOffset(lane->interpolateLanePosToGeometryPos(getPositionOnLane()));
}


Position
MEVehicle::getPosition(const double offset) const {
    const MSLane* const lane = getEdge()->getLanes()[0];
    return lane->geometryPositionAtOffset(getPositionOnLane() + offset);
}

PositionVector
MEVehicle::getBoundingBox(double offset) const {
    double a = getAngle() + M_PI; // angle pointing backwards
    double l = getLength();
    Position pos = getPosition();
    Position backPos = pos + Position(l * cos(a), l * sin(a));
    PositionVector centerLine;
    centerLine.push_back(pos);
    centerLine.push_back(backPos);
    if (offset != 0) {
        centerLine.extrapolate2D(offset);
    }
    PositionVector result = centerLine;
    result.move2side(MAX2(0.0, 0.5 * myType->getWidth() + offset));
    centerLine.move2side(MIN2(0.0, -0.5 * myType->getWidth() - offset));
    result.append(centerLine.reverse(), POSITION_EPS);
    return result;
}

double
MEVehicle::getSpeed() const {
    if (getWaitingTime() > 0 || isStopped()) {
        return 0;
    } else {
        return getAverageSpeed();
    }
}


double
MEVehicle::getAverageSpeed() const {
    if (mySegment == nullptr || myQueIndex == MESegment::PARKING_QUEUE) {
        return 0;
    } else {
        return MIN2(mySegment->getLength() / STEPS2TIME(myEventTime - myLastEntryTime),
                    getEdge()->getLanes()[myQueIndex]->getVehicleMaxSpeed(this));
    }
}


double
MEVehicle::estimateLeaveSpeed(const MSLink* link) const {
    /// @see MSVehicle.cpp::estimateLeaveSpeed
    const double v = getSpeed();
    return MIN2(link->getViaLaneOrLane()->getVehicleMaxSpeed(this),
                (double)sqrt(2 * link->getLength() * getVehicleType().getCarFollowModel().getMaxAccel() + v * v));
}


double
MEVehicle::getConservativeSpeed(SUMOTime& earliestArrival) const {
    earliestArrival = MAX2(myEventTime, earliestArrival - DELTA_T); // event times have subsecond resolution
    return mySegment->getLength() / STEPS2TIME(earliestArrival - myLastEntryTime);
}


bool
MEVehicle::moveRoutePointer() {
    // vehicle has just entered a new edge. Position is 0
    if (myCurrEdge == myRoute->end() - 1 || (myParameter->arrivalEdge >= 0 && getRoutePosition() >= myParameter->arrivalEdge)) { // may happen during teleport
        return true;
    }
    ++myCurrEdge;
    if ((*myCurrEdge)->isVaporizing()) {
        return true;
    }
    // update via
    if (myParameter->via.size() > 0 && (*myCurrEdge)->getID() == myParameter->via.front()) {
        myParameter->via.erase(myParameter->via.begin());
    }
    return hasArrived();
}


bool
MEVehicle::hasArrived() const {
    // mySegment may be 0 due to teleporting or arrival
    return (myCurrEdge == myRoute->end() - 1 || (myParameter->arrivalEdge >= 0 && getRoutePosition() >= myParameter->arrivalEdge)) && (
               (mySegment == nullptr)
               || myEventTime == SUMOTime_MIN
               || getPositionOnLane() > myArrivalPos - POSITION_EPS);
}


bool
MEVehicle::isOnRoad() const {
    return getSegment() != nullptr;
}


bool
MEVehicle::isIdling() const {
    return false;
}


void
MEVehicle::setApproaching(MSLink* link) {
    if (link != nullptr) {
        const double speed = getSpeed();
        link->setApproaching(this, getEventTime() + (link->getState() == LINKSTATE_ALLWAY_STOP ?
                             (SUMOTime)RandHelper::rand((int)2) : 0), // tie braker
                             speed, speed, true,
                             speed, getWaitingTime(),
                             // @note: dist is not used by meso (getZipperSpeed is never called)
                             getSegment()->getLength(), 0);
    }
}


bool
MEVehicle::replaceRoute(ConstMSRoutePtr newRoute, const std::string& info,  bool onInit, int offset, bool addRouteStops, bool removeStops, std::string* msgReturn) {
    MSLink* const oldLink = mySegment != nullptr ? mySegment->getLink(this) : nullptr;
    if (MSBaseVehicle::replaceRoute(newRoute, info, onInit, offset, addRouteStops, removeStops, msgReturn)) {
        if (mySegment != nullptr) {
            MSLink* const newLink = mySegment->getLink(this);
            // update approaching vehicle information
            if (oldLink != newLink) {
                if (oldLink != nullptr) {
                    oldLink->removeApproaching(this);
                }
                setApproaching(newLink);
            }
        }
        return true;
    }
    return false;
}


SUMOTime
MEVehicle::checkStop(SUMOTime time) {
    const SUMOTime initialTime = time;
    bool hadStop = false;
    for (MSStop& stop : myStops) {
        if (stop.joinTriggered) {
            WRITE_WARNINGF(TL("Join stops are not available in meso yet (vehicle '%', segment '%')."),
                           getID(), mySegment->getID());
            continue;
        }
        if (stop.edge != myCurrEdge || stop.segment != mySegment) {
            break;
        }
        const SUMOTime cur = time;
        if (stop.duration > 0) { // it might be a triggered stop with duration -1
            time += stop.duration;
        }
        if (stop.pars.until > time) {
            // @note: this assumes the stop is reached at time. With the way this is called in MESegment (time == entryTime),
            // travel time is overestimated of the stop is not at the start of the segment
            time = stop.pars.until;
        }
        if (MSGlobals::gUseStopEnded && stop.pars.ended >= 0) {
            time = MAX2(cur, stop.pars.ended);
        }
        if (!stop.reached) {
            stop.reached = true;
            stop.pars.started = myLastEntryTime;
            stop.endBoarding = stop.pars.extension >= 0 ? time + stop.pars.extension : SUMOTime_MAX;
            if (MSStopOut::active()) {
                if (!hadStop) {
                    MSStopOut::getInstance()->stopStarted(this, getPersonNumber(), getContainerNumber(), myLastEntryTime);
                } else {
                    WRITE_WARNINGF(TL("Vehicle '%' has multiple stops on segment '%', time=% (stop-output will be merged)."),
                                   getID(), mySegment->getID(), time2string(time));
                }
            }
            MSDevice_Taxi* taxi = static_cast<MSDevice_Taxi*>(getDevice(typeid(MSDevice_Taxi)));
            if (taxi != nullptr) {
                taxi->notifyMove(*this, 0, 0, 0);
            }
        }
        if (stop.triggered || stop.containerTriggered || stop.joinTriggered) {
            time = MAX2(time, cur + DELTA_T);
        }
        hadStop = true;
    }
    MSDevice_Tripinfo* tripinfo = static_cast<MSDevice_Tripinfo*>(getDevice(typeid(MSDevice_Tripinfo)));
    if (tripinfo != nullptr) {
        tripinfo->updateStopTime(time - initialTime);
    }
    return time;
}


bool
MEVehicle::resumeFromStopping() {
    if (isStopped()) {
        const SUMOTime now = SIMSTEP;
        MSStop& stop = myStops.front();
        stop.pars.ended = now;
        for (const auto& rem : myMoveReminders) {
            rem.first->notifyStopEnded();
        }
        if (MSStopOut::active()) {
            MSStopOut::getInstance()->stopEnded(this, stop.pars, mySegment->getEdge().getID());
        }
        myPastStops.push_back(stop.pars);
        myPastStops.back().routeIndex = (int)(stop.edge - myRoute->begin());
        if (myAmRegisteredAsWaiting && (stop.triggered || stop.containerTriggered || stop.joinTriggered)) {
            MSNet::getInstance()->getVehicleControl().unregisterOneWaiting();
            myAmRegisteredAsWaiting = false;
        }
        myStops.pop_front();
        if (myEventTime > now) {
            // if this is an aborted stop we need to change the event time of the vehicle
            if (MSGlobals::gMesoNet->removeLeaderCar(this)) {
                myEventTime = now + 1;
                MSGlobals::gMesoNet->addLeaderCar(this, nullptr);
            }
        }
        return true;
    }
    return false;
}


double
MEVehicle::getCurrentStoppingTimeSeconds() const {
    SUMOTime time = myLastEntryTime;
    for (const MSStop& stop : myStops) {
        if (stop.reached) {
            time += stop.duration;
            if (stop.pars.until > time) {
                // @note: this assumes the stop is reached at time. With the way this is called in MESegment (time == entryTime),
                // travel time is overestimated of the stop is not at the start of the segment
                time = stop.pars.until;
            }
        } else {
            break;
        }
    }
    return STEPS2TIME(time - myLastEntryTime);
}


void
MEVehicle::processStop() {
    assert(isStopped());
    double lastPos = -1;
    bool hadStop = false;
    while (!myStops.empty()) {
        MSStop& stop = myStops.front();
        if (stop.edge != myCurrEdge || stop.segment != mySegment || stop.pars.endPos <= lastPos) {
            break;
        }
        lastPos = stop.pars.endPos;
        MSNet* const net = MSNet::getInstance();
        SUMOTime dummy = -1; // boarding- and loading-time are not considered
        if (hadStop && MSStopOut::active()) {
            stop.reached = true;
            MSStopOut::getInstance()->stopStarted(this, getPersonNumber(), getContainerNumber(), myLastEntryTime);
        }
        if (net->hasPersons()) {
            net->getPersonControl().loadAnyWaiting(&mySegment->getEdge(), this, dummy, dummy);
        }
        if (net->hasContainers()) {
            net->getContainerControl().loadAnyWaiting(&mySegment->getEdge(), this, dummy, dummy);
        }
        resumeFromStopping();
        hadStop = true;
    }
    mySegment->getEdge().removeWaiting(this);
}


bool
MEVehicle::mayProceed() {
    if (mySegment == nullptr) {
        return true;
    }
    MSNet* const net = MSNet::getInstance();
    SUMOTime dummy = -1; // boarding- and loading-time are not considered
    for (MSStop& stop : myStops) {
        if (!stop.reached) {
            break;
        }
        if (net->getCurrentTimeStep() > stop.endBoarding) {
            if (stop.triggered || stop.containerTriggered) {
                MSDevice_Taxi* taxiDevice = static_cast<MSDevice_Taxi*>(getDevice(typeid(MSDevice_Taxi)));
                if (taxiDevice != nullptr) {
                    taxiDevice->cancelCurrentCustomers();
                }
                stop.triggered = false;
                stop.containerTriggered = false;
            }
            if (myAmRegisteredAsWaiting) {
                net->getVehicleControl().unregisterOneWaiting();
                myAmRegisteredAsWaiting = false;
            }
        }
        if (stop.triggered) {
            if (getVehicleType().getPersonCapacity() == getPersonNumber()) {
                // we could not check this on entering the segment because there may be persons who still want to leave
                WRITE_WARNINGF(TL("Vehicle '%' ignores triggered stop on lane '%' due to capacity constraints."), getID(), stop.lane->getID());
                stop.triggered = false;
                if (myAmRegisteredAsWaiting) {
                    net->getVehicleControl().unregisterOneWaiting();
                    myAmRegisteredAsWaiting = false;
                }
            } else if (!net->hasPersons() || !net->getPersonControl().loadAnyWaiting(&mySegment->getEdge(), this, dummy, dummy)) {
                if (!myAmRegisteredAsWaiting) {
                    MSNet::getInstance()->getVehicleControl().registerOneWaiting();
                    myAmRegisteredAsWaiting = true;
                }
                return false;
            }
        }
        if (stop.containerTriggered) {
            if (getVehicleType().getContainerCapacity() == getContainerNumber()) {
                // we could not check this on entering the segment because there may be containers who still want to leave
                WRITE_WARNINGF(TL("Vehicle '%' ignores container triggered stop on lane '%' due to capacity constraints."), getID(), stop.lane->getID());
                stop.containerTriggered = false;
                if (myAmRegisteredAsWaiting) {
                    net->getVehicleControl().unregisterOneWaiting();
                    myAmRegisteredAsWaiting = false;
                }
            } else if (!net->hasContainers() || !net->getContainerControl().loadAnyWaiting(&mySegment->getEdge(), this, dummy, dummy)) {
                if (!myAmRegisteredAsWaiting) {
                    MSNet::getInstance()->getVehicleControl().registerOneWaiting();
                    myAmRegisteredAsWaiting = true;
                }
                return false;
            }
        }
        if (stop.joinTriggered) {
            // TODO do something useful here
            return false;
        }
    }
    return mySegment->isOpen(this);
}


double
MEVehicle::getCurrentLinkPenaltySeconds() const {
    if (mySegment == nullptr) {
        return 0;
    } else {
        return STEPS2TIME(mySegment->getLinkPenalty(this));
    }
}


void
MEVehicle::updateDetectorForWriting(MSMoveReminder* rem, SUMOTime currentTime, SUMOTime exitTime) {
    for (MoveReminderCont::iterator i = myMoveReminders.begin(); i != myMoveReminders.end(); ++i) {
        if (i->first == rem) {
            rem->updateDetector(*this, mySegment->getIndex() * mySegment->getLength(),
                                (mySegment->getIndex() + 1) * mySegment->getLength(),
                                getLastEntryTime(), currentTime, exitTime, false);
#ifdef _DEBUG
            if (myTraceMoveReminders) {
                traceMoveReminder("notifyMove", i->first, i->second, true);
            }
#endif
            return;
        }
    }
}


void
MEVehicle::updateDetectors(SUMOTime currentTime, const bool isLeave, const MSMoveReminder::Notification reason) {
    // segments of the same edge have the same reminder so no cleaning up must take place
    const bool cleanUp = isLeave && (reason != MSMoveReminder::NOTIFICATION_SEGMENT);
    for (MoveReminderCont::iterator rem = myMoveReminders.begin(); rem != myMoveReminders.end();) {
        if (currentTime != getLastEntryTime()) {
            rem->first->updateDetector(*this, mySegment->getIndex() * mySegment->getLength(),
                                       (mySegment->getIndex() + 1) * mySegment->getLength(),
                                       getLastEntryTime(), currentTime, getEventTime(), cleanUp);
#ifdef _DEBUG
            if (myTraceMoveReminders) {
                traceMoveReminder("notifyMove", rem->first, rem->second, true);
            }
#endif
        }
        if (!isLeave || rem->first->notifyLeave(*this, mySegment->getLength(), reason)) {
#ifdef _DEBUG
            if (isLeave && myTraceMoveReminders) {
                traceMoveReminder("notifyLeave", rem->first, rem->second, true);
            }
#endif

            if (isLeave) {
                rem->second += getEdge()->getLength();
#ifdef _DEBUG
                if (myTraceMoveReminders) {
                    traceMoveReminder("adaptedPos", rem->first, rem->second, true);
                }
#endif
            }
            ++rem;
        } else {
#ifdef _DEBUG
            if (myTraceMoveReminders) {
                traceMoveReminder("remove", rem->first, rem->second, false);
            }
#endif
            rem = myMoveReminders.erase(rem);
        }
    }
    if (reason == MSMoveReminder::NOTIFICATION_JUNCTION || reason == MSMoveReminder::NOTIFICATION_TELEPORT) {
        myOdometer += getEdge()->getLength();
    }
}


MEVehicle::BaseInfluencer&
MEVehicle::getBaseInfluencer() {
    if (myInfluencer == nullptr) {
        myInfluencer = new BaseInfluencer();
    }
    return *myInfluencer;
}


const MEVehicle::BaseInfluencer*
MEVehicle::getBaseInfluencer() const {
    return myInfluencer;
}


void
MEVehicle::onRemovalFromNet(const MSMoveReminder::Notification reason) {
    MSGlobals::gMesoNet->removeLeaderCar(this);
    MSGlobals::gMesoNet->changeSegment(this, MSNet::getInstance()->getCurrentTimeStep(), nullptr, reason);
}

double
MEVehicle::getRightSideOnEdge(const MSLane* /*lane*/) const {
    if (mySegment == nullptr || mySegment->getIndex() >= getEdge()->getNumLanes()) {
        return 0;
    }
    const MSLane* lane = getEdge()->getLanes()[mySegment->getIndex()];
    return lane->getRightSideOnEdge() + lane->getWidth() * 0.5 - 0.5 * getVehicleType().getWidth();

}


void
MEVehicle::saveState(OutputDevice& out) {
    if (mySegment != nullptr && MESegment::isInvalid(mySegment)) {
        // segment is vaporization target, do not write this vehicle
        return;
    }
    MSBaseVehicle::saveState(out);
    assert(mySegment == nullptr || *myCurrEdge == &mySegment->getEdge());
    std::vector<SUMOTime> internals;
    internals.push_back(myParameter->parametersSet);
    internals.push_back(myDeparture);
    internals.push_back((SUMOTime)distance(myRoute->begin(), myCurrEdge));
    internals.push_back((SUMOTime)myDepartPos * 1000); // store as mm
    internals.push_back(mySegment == nullptr ? (SUMOTime) - 1 : (SUMOTime)mySegment->getIndex());
    internals.push_back((SUMOTime)getQueIndex());
    internals.push_back(myEventTime);
    internals.push_back(myLastEntryTime);
    internals.push_back(myBlockTime);
    out.writeAttr(SUMO_ATTR_STATE, toString(internals));
    // save past stops
    for (SUMOVehicleParameter::Stop stop : myPastStops) {
        stop.write(out, false);
        // do not write started and ended twice
        if ((stop.parametersSet & STOP_STARTED_SET) == 0) {
            out.writeAttr(SUMO_ATTR_STARTED, time2string(stop.started));
        }
        if ((stop.parametersSet & STOP_ENDED_SET) == 0) {
            out.writeAttr(SUMO_ATTR_ENDED, time2string(stop.ended));
        }
        out.closeTag();
    }
    // save upcoming stops
    for (const MSStop& stop : myStops) {
        stop.write(out);
    }
    // save parameters
    myParameter->writeParams(out);
    for (MSDevice* dev : myDevices) {
        dev->saveState(out);
    }
    out.closeTag();
}


void
MEVehicle::loadState(const SUMOSAXAttributes& attrs, const SUMOTime offset) {
    if (attrs.hasAttribute(SUMO_ATTR_POSITION)) {
        throw ProcessError(TL("Error: Invalid vehicles in state (may be a micro state)!"));
    }
    int routeOffset;
    int segIndex;
    int queIndex;
    std::istringstream bis(attrs.getString(SUMO_ATTR_STATE));
    bis >> myParameter->parametersSet;
    bis >> myDeparture;
    bis >> routeOffset;
    bis >> myDepartPos;
    bis >> segIndex;
    bis >> queIndex;
    bis >> myEventTime;
    bis >> myLastEntryTime;
    bis >> myBlockTime;
    myDepartPos /= 1000.; // was stored as mm

    // load stops
    myStops.clear();
    addStops(!MSGlobals::gCheckRoutes, &myCurrEdge, false);

    if (hasDeparted()) {
        myDeparture -= offset;
        myEventTime -= offset;
        myLastEntryTime -= offset;
        myCurrEdge = myRoute->begin() + routeOffset;
        if (segIndex >= 0) {
            MESegment* seg = MSGlobals::gMesoNet->getSegmentForEdge(**myCurrEdge);
            while (seg->getIndex() != (int)segIndex) {
                seg = seg->getNextSegment();
                assert(seg != 0);
            }
            setSegment(seg, queIndex);
            if (queIndex == MESegment::PARKING_QUEUE) {
                MSGlobals::gMesoNet->addLeaderCar(this, nullptr);
            }
        } else {
            // on teleport
            setSegment(nullptr, 0);
            assert(myEventTime != SUMOTime_MIN);
            MSGlobals::gMesoNet->addLeaderCar(this, nullptr);
        }
        // see MSBaseVehicle constructor
        if (myParameter->wasSet(VEHPARS_FORCE_REROUTE)) {
            calculateArrivalParams(true);
        }
    }
    if (myBlockTime != SUMOTime_MAX) {
        myBlockTime -= offset;
    }
    std::istringstream dis(attrs.getString(SUMO_ATTR_DISTANCE));
    dis >> myOdometer >> myNumberReroutes;
}


/****************************************************************************/
