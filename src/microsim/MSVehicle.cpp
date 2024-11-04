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
/// @file    MSVehicle.cpp
/// @author  Christian Roessel
/// @author  Jakob Erdmann
/// @author  Bjoern Hendriks
/// @author  Daniel Krajzewicz
/// @author  Thimor Bohn
/// @author  Friedemann Wesner
/// @author  Laura Bieker
/// @author  Clemens Honomichl
/// @author  Michael Behrisch
/// @author  Axel Wegener
/// @author  Christoph Sommer
/// @author  Leonhard Luecken
/// @author  Lara Codeca
/// @author  Mirko Barthauer
/// @date    Mon, 05 Mar 2001
///
// Representation of a vehicle in the micro simulation
/****************************************************************************/
#include <config.h>

#include <iostream>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <map>
#include <memory>
#include <utils/common/ToString.h>
#include <utils/common/FileHelpers.h>
#include <utils/router/DijkstraRouter.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/RandHelper.h>
#include <utils/common/StringUtils.h>
#include <utils/common/StdDefs.h>
#include <utils/geom/GeomHelper.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/xml/SUMOSAXAttributes.h>
#include <utils/vehicle/SUMOVehicleParserHelper.h>
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <microsim/transportables/MSPerson.h>
#include <microsim/transportables/MSPModel.h>
#include <microsim/devices/MSDevice_Transportable.h>
#include <microsim/devices/MSDevice_DriverState.h>
#include <microsim/devices/MSDevice_Friction.h>
#include <microsim/devices/MSDevice_Taxi.h>
#include <microsim/devices/MSDevice_Vehroutes.h>
#include <microsim/devices/MSDevice_ElecHybrid.h>
#include <microsim/devices/MSDevice_GLOSA.h>
#include <microsim/output/MSStopOut.h>
#include <microsim/trigger/MSChargingStation.h>
#include <microsim/trigger/MSOverheadWire.h>
#include <microsim/traffic_lights/MSTrafficLightLogic.h>
#include <microsim/traffic_lights/MSRailSignalControl.h>
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <microsim/transportables/MSTransportableControl.h>
#include <microsim/devices/MSDevice_Transportable.h>
#include "MSEdgeControl.h"
#include "MSVehicleControl.h"
#include "MSInsertionControl.h"
#include "MSVehicleTransfer.h"
#include "MSGlobals.h"
#include "MSJunctionLogic.h"
#include "MSStop.h"
#include "MSStoppingPlace.h"
#include "MSParkingArea.h"
#include "MSMoveReminder.h"
#include "MSLane.h"
#include "MSJunction.h"
#include "MSEdge.h"
#include "MSVehicleType.h"
#include "MSNet.h"
#include "MSRoute.h"
#include "MSLeaderInfo.h"
#include "MSDriverState.h"
#include "MSVehicle.h"


//#define DEBUG_PLAN_MOVE
//#define DEBUG_PLAN_MOVE_LEADERINFO
//#define DEBUG_CHECKREWINDLINKLANES
//#define DEBUG_EXEC_MOVE
//#define DEBUG_FURTHER
//#define DEBUG_SETFURTHER
//#define DEBUG_TARGET_LANE
//#define DEBUG_STOPS
//#define DEBUG_BESTLANES
//#define DEBUG_IGNORE_RED
//#define DEBUG_ACTIONSTEPS
//#define DEBUG_NEXT_TURN
//#define DEBUG_TRACI
//#define DEBUG_REVERSE_BIDI
//#define DEBUG_EXTRAPOLATE_DEPARTPOS
//#define DEBUG_REMOTECONTROL
//#define DEBUG_MOVEREMINDERS
//#define DEBUG_COND (getID() == "ego")
//#define DEBUG_COND (true)
#define DEBUG_COND (isSelected())
//#define DEBUG_COND2(obj) (obj->getID() == "ego")
#define DEBUG_COND2(obj) (obj->isSelected())

//#define PARALLEL_STOPWATCH


#define STOPPING_PLACE_OFFSET 0.5

#define CRLL_LOOK_AHEAD 5

#define JUNCTION_BLOCKAGE_TIME 5 // s

// @todo Calibrate with real-world values / make configurable
#define DIST_TO_STOPLINE_EXPECT_PRIORITY 1.0

#define NUMERICAL_EPS_SPEED (0.1 * NUMERICAL_EPS * TS)

// ===========================================================================
// static value definitions
// ===========================================================================
std::vector<MSLane*> MSVehicle::myEmptyLaneVector;


// ===========================================================================
// method definitions
// ===========================================================================
/* -------------------------------------------------------------------------
 * methods of MSVehicle::State
 * ----------------------------------------------------------------------- */
MSVehicle::State::State(const State& state) {
    myPos = state.myPos;
    mySpeed = state.mySpeed;
    myPosLat = state.myPosLat;
    myBackPos = state.myBackPos;
    myPreviousSpeed = state.myPreviousSpeed;
    myLastCoveredDist = state.myLastCoveredDist;
}


MSVehicle::State&
MSVehicle::State::operator=(const State& state) {
    myPos   = state.myPos;
    mySpeed = state.mySpeed;
    myPosLat   = state.myPosLat;
    myBackPos = state.myBackPos;
    myPreviousSpeed = state.myPreviousSpeed;
    myLastCoveredDist = state.myLastCoveredDist;
    return *this;
}


bool
MSVehicle::State::operator!=(const State& state) {
    return (myPos    != state.myPos ||
            mySpeed  != state.mySpeed ||
            myPosLat != state.myPosLat ||
            myLastCoveredDist != state.myLastCoveredDist ||
            myPreviousSpeed != state.myPreviousSpeed ||
            myBackPos != state.myBackPos);
}


MSVehicle::State::State(double pos, double speed, double posLat, double backPos, double previousSpeed) :
    myPos(pos), mySpeed(speed), myPosLat(posLat), myBackPos(backPos), myPreviousSpeed(previousSpeed), myLastCoveredDist(SPEED2DIST(speed)) {}



/* -------------------------------------------------------------------------
 * methods of MSVehicle::WaitingTimeCollector
 * ----------------------------------------------------------------------- */
MSVehicle::WaitingTimeCollector::WaitingTimeCollector(SUMOTime memory) : myMemorySize(memory) {}


SUMOTime
MSVehicle::WaitingTimeCollector::cumulatedWaitingTime(SUMOTime memorySpan) const {
    assert(memorySpan <= myMemorySize);
    if (memorySpan == -1) {
        memorySpan = myMemorySize;
    }
    SUMOTime totalWaitingTime = 0;
    for (const auto& interval : myWaitingIntervals) {
        if (interval.second >= memorySpan) {
            if (interval.first >= memorySpan) {
                break;
            } else {
                totalWaitingTime += memorySpan - interval.first;
            }
        } else {
            totalWaitingTime += interval.second - interval.first;
        }
    }
    return totalWaitingTime;
}


void
MSVehicle::WaitingTimeCollector::passTime(SUMOTime dt, bool waiting) {
    auto i = myWaitingIntervals.begin();
    const auto end = myWaitingIntervals.end();
    const bool startNewInterval = i == end || (i->first != 0);
    while (i != end) {
        i->first += dt;
        if (i->first >= myMemorySize) {
            break;
        }
        i->second += dt;
        i++;
    }

    // remove intervals beyond memorySize
    auto d = std::distance(i, end);
    while (d > 0) {
        myWaitingIntervals.pop_back();
        d--;
    }

    if (!waiting) {
        return;
    } else if (!startNewInterval) {
        myWaitingIntervals.begin()->first = 0;
    } else {
        myWaitingIntervals.push_front(std::make_pair(0, dt));
    }
    return;
}


const std::string
MSVehicle::WaitingTimeCollector::getState() const {
    std::ostringstream state;
    state << myMemorySize << " " << myWaitingIntervals.size();
    for (const auto& interval : myWaitingIntervals) {
        state << " " << interval.first << " " << interval.second;
    }
    return state.str();
}


void
MSVehicle::WaitingTimeCollector::setState(const std::string& state) {
    std::istringstream is(state);
    int numIntervals;
    SUMOTime begin, end;
    is >> myMemorySize >> numIntervals;
    while (numIntervals-- > 0) {
        is >> begin >> end;
        myWaitingIntervals.emplace_back(begin, end);
    }
}


/* -------------------------------------------------------------------------
 * methods of MSVehicle::Influencer::GapControlState
 * ----------------------------------------------------------------------- */
void
MSVehicle::Influencer::GapControlVehStateListener::vehicleStateChanged(const SUMOVehicle* const vehicle, MSNet::VehicleState to, const std::string& /*info*/) {
//    std::cout << "GapControlVehStateListener::vehicleStateChanged() vehicle=" << vehicle->getID() << ", to=" << to << std::endl;
    switch (to) {
        case MSNet::VehicleState::STARTING_TELEPORT:
        case MSNet::VehicleState::ARRIVED:
        case MSNet::VehicleState::STARTING_PARKING: {
            // Vehicle left road
//         Look up reference vehicle in refVehMap and in case deactivate corresponding gap control
            const MSVehicle* msVeh = static_cast<const MSVehicle*>(vehicle);
//        std::cout << "GapControlVehStateListener::vehicleStateChanged() vehicle=" << vehicle->getID() << " left the road." << std::endl;
            if (GapControlState::refVehMap.find(msVeh) != end(GapControlState::refVehMap)) {
//            std::cout << "GapControlVehStateListener::deactivating ref vehicle=" << vehicle->getID() << std::endl;
                GapControlState::refVehMap[msVeh]->deactivate();
            }
        }
        break;
        default:
        {};
            // do nothing, vehicle still on road
    }
}

std::map<const MSVehicle*, MSVehicle::Influencer::GapControlState*>
MSVehicle::Influencer::GapControlState::refVehMap;

MSVehicle::Influencer::GapControlVehStateListener
MSVehicle::Influencer::GapControlState::vehStateListener;

MSVehicle::Influencer::GapControlState::GapControlState() :
    tauOriginal(-1), tauCurrent(-1), tauTarget(-1), addGapCurrent(-1), addGapTarget(-1),
    remainingDuration(-1), changeRate(-1), maxDecel(-1), referenceVeh(nullptr), active(false), gapAttained(false), prevLeader(nullptr),
    lastUpdate(-1), timeHeadwayIncrement(0.0), spaceHeadwayIncrement(0.0) {}


MSVehicle::Influencer::GapControlState::~GapControlState() {
    deactivate();
}

void
MSVehicle::Influencer::GapControlState::init() {
//    std::cout << "GapControlState::init()" << std::endl;
    if (MSNet::hasInstance()) {
        MSNet::VehicleStateListener* vsl = dynamic_cast<MSNet::VehicleStateListener*>(&vehStateListener);
        MSNet::getInstance()->addVehicleStateListener(vsl);
    } else {
        WRITE_ERROR("MSVehicle::Influencer::GapControlState::init(): No MSNet instance found!")
    }
}

void
MSVehicle::Influencer::GapControlState::cleanup() {
    MSNet::VehicleStateListener* vsl = dynamic_cast<MSNet::VehicleStateListener*>(&vehStateListener);
    MSNet::getInstance()->removeVehicleStateListener(vsl);
}

void
MSVehicle::Influencer::GapControlState::activate(double tauOrig, double tauNew, double additionalGap, double dur, double rate, double decel, const MSVehicle* refVeh) {
    if (MSGlobals::gUseMesoSim) {
        WRITE_ERROR(TL("No gap control available for meso."))
    } else {
        // always deactivate control before activating (triggers clean-up of refVehMap)
//        std::cout << "activate gap control with refVeh=" << (refVeh==nullptr? "NULL" : refVeh->getID()) << std::endl;
        tauOriginal = tauOrig;
        tauCurrent = tauOrig;
        tauTarget = tauNew;
        addGapCurrent = 0.0;
        addGapTarget = additionalGap;
        remainingDuration = dur;
        changeRate = rate;
        maxDecel = decel;
        referenceVeh = refVeh;
        active = true;
        gapAttained = false;
        prevLeader = nullptr;
        lastUpdate = SIMSTEP - DELTA_T;
        timeHeadwayIncrement = changeRate * TS * (tauTarget - tauOriginal);
        spaceHeadwayIncrement = changeRate * TS * addGapTarget;

        if (referenceVeh != nullptr) {
            // Add refVeh to refVehMap
            GapControlState::refVehMap[referenceVeh] = this;
        }
    }
}

void
MSVehicle::Influencer::GapControlState::deactivate() {
    active = false;
    if (referenceVeh != nullptr) {
        // Remove corresponding refVehMapEntry if appropriate
        GapControlState::refVehMap.erase(referenceVeh);
        referenceVeh = nullptr;
    }
}


/* -------------------------------------------------------------------------
 * methods of MSVehicle::Influencer
 * ----------------------------------------------------------------------- */
MSVehicle::Influencer::Influencer() :
    myGapControlState(nullptr),
    myOriginalSpeed(-1),
    myLatDist(0),
    mySpeedAdaptationStarted(true),
    myConsiderSafeVelocity(true),
    myConsiderMaxAcceleration(true),
    myConsiderMaxDeceleration(true),
    myRespectJunctionPriority(true),
    myEmergencyBrakeRedLight(true),
    myRespectJunctionLeaderPriority(true),
    myLastRemoteAccess(-TIME2STEPS(20)),
    myStrategicLC(LC_NOCONFLICT),
    myCooperativeLC(LC_NOCONFLICT),
    mySpeedGainLC(LC_NOCONFLICT),
    myRightDriveLC(LC_NOCONFLICT),
    mySublaneLC(LC_NOCONFLICT),
    myTraciLaneChangePriority(LCP_URGENT),
    myTraCISignals(-1)
{}


MSVehicle::Influencer::~Influencer() {}

void
MSVehicle::Influencer::init() {
    GapControlState::init();
}

void
MSVehicle::Influencer::cleanup() {
    GapControlState::cleanup();
}

void
MSVehicle::Influencer::setSpeedTimeLine(const std::vector<std::pair<SUMOTime, double> >& speedTimeLine) {
    mySpeedAdaptationStarted = true;
    mySpeedTimeLine = speedTimeLine;
}

void
MSVehicle::Influencer::activateGapController(double originalTau, double newTimeHeadway, double newSpaceHeadway, double duration, double changeRate, double maxDecel, MSVehicle* refVeh) {
    if (myGapControlState == nullptr) {
        myGapControlState = std::make_shared<GapControlState>();
    }
    myGapControlState->activate(originalTau, newTimeHeadway, newSpaceHeadway, duration, changeRate, maxDecel, refVeh);
}

void
MSVehicle::Influencer::deactivateGapController() {
    if (myGapControlState != nullptr && myGapControlState->active) {
        myGapControlState->deactivate();
    }
}

void
MSVehicle::Influencer::setLaneTimeLine(const std::vector<std::pair<SUMOTime, int> >& laneTimeLine) {
    myLaneTimeLine = laneTimeLine;
}


void
MSVehicle::Influencer::adaptLaneTimeLine(int indexShift) {
    for (auto& item : myLaneTimeLine) {
        item.second += indexShift;
    }
}


void
MSVehicle::Influencer::setSublaneChange(double latDist) {
    myLatDist = latDist;
}

int
MSVehicle::Influencer::getSpeedMode() const {
    return (1 * myConsiderSafeVelocity +
            2 * myConsiderMaxAcceleration +
            4 * myConsiderMaxDeceleration +
            8 * myRespectJunctionPriority +
            16 * myEmergencyBrakeRedLight +
            32 * !myRespectJunctionLeaderPriority // inverted!
           );
}


int
MSVehicle::Influencer::getLaneChangeMode() const {
    return (1 * myStrategicLC +
            4 * myCooperativeLC +
            16 * mySpeedGainLC +
            64 * myRightDriveLC +
            256 * myTraciLaneChangePriority +
            1024 * mySublaneLC);
}

SUMOTime
MSVehicle::Influencer::getLaneTimeLineDuration() {
    SUMOTime duration = -1;
    for (std::vector<std::pair<SUMOTime, int>>::iterator i = myLaneTimeLine.begin(); i != myLaneTimeLine.end(); ++i) {
        if (duration < 0) {
            duration = i->first;
        } else {
            duration -=  i->first;
        }
    }
    return -duration;
}

SUMOTime
MSVehicle::Influencer::getLaneTimeLineEnd() {
    if (!myLaneTimeLine.empty()) {
        return myLaneTimeLine.back().first;
    } else {
        return -1;
    }
}


double
MSVehicle::Influencer::influenceSpeed(SUMOTime currentTime, double speed, double vSafe, double vMin, double vMax) {
    // remove leading commands which are no longer valid
    while (mySpeedTimeLine.size() == 1 || (mySpeedTimeLine.size() > 1 && currentTime > mySpeedTimeLine[1].first)) {
        mySpeedTimeLine.erase(mySpeedTimeLine.begin());
    }

    if (!(mySpeedTimeLine.size() < 2 || currentTime < mySpeedTimeLine[0].first)) {
        // Speed advice is active -> compute new speed according to speedTimeLine
        if (!mySpeedAdaptationStarted) {
            mySpeedTimeLine[0].second = speed;
            mySpeedAdaptationStarted = true;
        }
        currentTime += DELTA_T;
        const double td = STEPS2TIME(currentTime - mySpeedTimeLine[0].first) / STEPS2TIME(mySpeedTimeLine[1].first + DELTA_T - mySpeedTimeLine[0].first);
        speed = mySpeedTimeLine[0].second - (mySpeedTimeLine[0].second - mySpeedTimeLine[1].second) * td;
        if (myConsiderSafeVelocity) {
            speed = MIN2(speed, vSafe);
        }
        if (myConsiderMaxAcceleration) {
            speed = MIN2(speed, vMax);
        }
        if (myConsiderMaxDeceleration) {
            speed = MAX2(speed, vMin);
        }
    }
    return speed;
}

double
MSVehicle::Influencer::gapControlSpeed(SUMOTime currentTime, const SUMOVehicle* veh, double speed, double vSafe, double vMin, double vMax) {
#ifdef DEBUG_TRACI
    if DEBUG_COND2(veh) {
        std::cout << currentTime << " Influencer::gapControlSpeed(): speed=" << speed
                  << ", vSafe=" << vSafe
                  << ", vMin=" << vMin
                  << ", vMax=" << vMax
                  << std::endl;
    }
#endif
    double gapControlSpeed = speed;
    if (myGapControlState != nullptr && myGapControlState->active) {
        // Determine leader and the speed that would be chosen by the gap controller
        const double currentSpeed = veh->getSpeed();
        const MSVehicle* msVeh = dynamic_cast<const MSVehicle*>(veh);
        assert(msVeh != nullptr);
        const double desiredTargetTimeSpacing = myGapControlState->tauTarget * currentSpeed;
        std::pair<const MSVehicle*, double> leaderInfo;
        if (myGapControlState->referenceVeh == nullptr) {
            // No reference vehicle specified -> use current leader as reference
            const double brakeGap = msVeh->getBrakeGap(true);
            leaderInfo = msVeh->getLeader(MAX2(desiredTargetTimeSpacing, myGapControlState->addGapCurrent)  + MAX2(brakeGap, 20.0));
#ifdef DEBUG_TRACI
            if DEBUG_COND2(veh) {
                std::cout <<  "  ---   no refVeh; myGapControlState->addGapCurrent: " << myGapControlState->addGapCurrent << ", brakeGap: " << brakeGap << " in simstep: " << SIMSTEP << std::endl;
            }
#endif
        } else {
            // Control gap wrt reference vehicle
            const MSVehicle* leader = myGapControlState->referenceVeh;
            double dist = msVeh->getDistanceToPosition(leader->getPositionOnLane(), leader->getLane()) - leader->getLength();
            if (dist > 100000) {
                // Reference vehicle was not found downstream the ego's route
                // Maybe, it is behind the ego vehicle
                dist = - leader->getDistanceToPosition(msVeh->getPositionOnLane(), msVeh->getLane()) - leader->getLength();
#ifdef DEBUG_TRACI
                if DEBUG_COND2(veh) {
                    if (dist < -100000) {
                        // also the ego vehicle is not ahead of the reference vehicle -> no CF-relation
                        std::cout <<  " Ego and reference vehicle are not in CF relation..." << std::endl;
                    } else {
                        std::cout <<  " Reference vehicle is behind ego..." << std::endl;
                    }
                }
#endif
            }
            leaderInfo = std::make_pair(leader, dist - msVeh->getVehicleType().getMinGap());
        }
        const double fakeDist = MAX2(0.0, leaderInfo.second - myGapControlState->addGapCurrent);
#ifdef DEBUG_TRACI
        if DEBUG_COND2(veh) {
            const double desiredCurrentSpacing = myGapControlState->tauCurrent * currentSpeed;
            std::cout <<  " Gap control active:"
                      << " currentSpeed=" << currentSpeed
                      << ", desiredTargetTimeSpacing=" << desiredTargetTimeSpacing
                      << ", desiredCurrentSpacing=" << desiredCurrentSpacing
                      << ", leader=" << (leaderInfo.first == nullptr ? "NULL" : leaderInfo.first->getID())
                      << ", dist=" << leaderInfo.second
                      << ", fakeDist=" << fakeDist
                      << ",\n tauOriginal=" << myGapControlState->tauOriginal
                      << ", tauTarget=" << myGapControlState->tauTarget
                      << ", tauCurrent=" << myGapControlState->tauCurrent
                      << std::endl;
        }
#endif
        if (leaderInfo.first != nullptr) {
            if (myGapControlState->prevLeader != nullptr && myGapControlState->prevLeader != leaderInfo.first) {
                // TODO: The leader changed. What to do?
            }
            // Remember leader
            myGapControlState->prevLeader = leaderInfo.first;

            // Calculate desired following speed assuming the alternative headway time
            MSCFModel* cfm = (MSCFModel*) & (msVeh->getVehicleType().getCarFollowModel());
            const double origTau = cfm->getHeadwayTime();
            cfm->setHeadwayTime(myGapControlState->tauCurrent);
            gapControlSpeed = MIN2(gapControlSpeed,
                                   cfm->followSpeed(msVeh, currentSpeed, fakeDist, leaderInfo.first->getSpeed(), leaderInfo.first->getCurrentApparentDecel(), leaderInfo.first));
            cfm->setHeadwayTime(origTau);
#ifdef DEBUG_TRACI
            if DEBUG_COND2(veh) {
                std::cout << " -> gapControlSpeed=" << gapControlSpeed;
                if (myGapControlState->maxDecel > 0) {
                    std::cout << ", with maxDecel bound: " << MAX2(gapControlSpeed, currentSpeed - TS * myGapControlState->maxDecel);
                }
                std::cout << std::endl;
            }
#endif
            if (myGapControlState->maxDecel > 0) {
                gapControlSpeed = MAX2(gapControlSpeed, currentSpeed - TS * myGapControlState->maxDecel);
            }
        }

        // Update gap controller
        // Check (1) if the gap control has established the desired gap,
        // and (2) if it has maintained active for the given duration afterwards
        if (myGapControlState->lastUpdate < currentTime) {
#ifdef DEBUG_TRACI
            if DEBUG_COND2(veh) {
                std::cout << " Updating GapControlState." << std::endl;
            }
#endif
            if (myGapControlState->tauCurrent == myGapControlState->tauTarget && myGapControlState->addGapCurrent == myGapControlState->addGapTarget) {
                if (!myGapControlState->gapAttained) {
                    // Check if the desired gap was established (add the POSITION_EPS to avoid infinite asymptotic behavior without having established the gap)
                    myGapControlState->gapAttained = leaderInfo.first == nullptr ||  leaderInfo.second > MAX2(desiredTargetTimeSpacing, myGapControlState->addGapTarget) - POSITION_EPS;
#ifdef DEBUG_TRACI
                    if DEBUG_COND2(veh) {
                        if (myGapControlState->gapAttained) {
                            std::cout << "   Target gap was established." << std::endl;
                        }
                    }
#endif
                } else {
                    // Count down remaining time if desired gap was established
                    myGapControlState->remainingDuration -= TS;
#ifdef DEBUG_TRACI
                    if DEBUG_COND2(veh) {
                        std::cout << "   Gap control remaining duration: " << myGapControlState->remainingDuration << std::endl;
                    }
#endif
                    if (myGapControlState->remainingDuration <= 0) {
#ifdef DEBUG_TRACI
                        if DEBUG_COND2(veh) {
                            std::cout << "   Gap control duration expired, deactivating control." << std::endl;
                        }
#endif
                        // switch off gap control
                        myGapControlState->deactivate();
                    }
                }
            } else {
                // Adjust current headway values
                myGapControlState->tauCurrent = MIN2(myGapControlState->tauCurrent + myGapControlState->timeHeadwayIncrement, myGapControlState->tauTarget);
                myGapControlState->addGapCurrent = MIN2(myGapControlState->addGapCurrent + myGapControlState->spaceHeadwayIncrement, myGapControlState->addGapTarget);
            }
        }
        if (myConsiderSafeVelocity) {
            gapControlSpeed = MIN2(gapControlSpeed, vSafe);
        }
        if (myConsiderMaxAcceleration) {
            gapControlSpeed = MIN2(gapControlSpeed, vMax);
        }
        if (myConsiderMaxDeceleration) {
            gapControlSpeed = MAX2(gapControlSpeed, vMin);
        }
        return MIN2(speed, gapControlSpeed);
    } else {
        return speed;
    }
}

double
MSVehicle::Influencer::getOriginalSpeed() const {
    return myOriginalSpeed;
}

void
MSVehicle::Influencer::setOriginalSpeed(double speed) {
    myOriginalSpeed = speed;
}


int
MSVehicle::Influencer::influenceChangeDecision(const SUMOTime currentTime, const MSEdge& currentEdge, const int currentLaneIndex, int state) {
    // remove leading commands which are no longer valid
    while (myLaneTimeLine.size() == 1 || (myLaneTimeLine.size() > 1 && currentTime > myLaneTimeLine[1].first)) {
        myLaneTimeLine.erase(myLaneTimeLine.begin());
    }
    ChangeRequest changeRequest = REQUEST_NONE;
    // do nothing if the time line does not apply for the current time
    if (myLaneTimeLine.size() >= 2 && currentTime >= myLaneTimeLine[0].first) {
        const int destinationLaneIndex = myLaneTimeLine[1].second;
        if (destinationLaneIndex < (int)currentEdge.getLanes().size()) {
            if (currentLaneIndex > destinationLaneIndex) {
                changeRequest = REQUEST_RIGHT;
            } else if (currentLaneIndex < destinationLaneIndex) {
                changeRequest = REQUEST_LEFT;
            } else {
                changeRequest = REQUEST_HOLD;
            }
        } else if (currentEdge.getLanes().back()->getOpposite() != nullptr) { // change to opposite direction driving
            changeRequest = REQUEST_LEFT;
            state = state | LCA_TRACI;
        }
    }
    // check whether the current reason shall be canceled / overridden
    if ((state & LCA_WANTS_LANECHANGE_OR_STAY) != 0) {
        // flags for the current reason
        LaneChangeMode mode = LC_NEVER;
        if ((state & LCA_TRACI) != 0 && myLatDist != 0) {
            // security checks
            if ((myTraciLaneChangePriority == LCP_ALWAYS)
                    || (myTraciLaneChangePriority == LCP_NOOVERLAP && (state & LCA_OVERLAPPING) == 0)) {
                state &= ~(LCA_BLOCKED | LCA_OVERLAPPING);
            }
            // continue sublane change manoeuvre
            return state;
        } else if ((state & LCA_STRATEGIC) != 0) {
            mode = myStrategicLC;
        } else if ((state & LCA_COOPERATIVE) != 0) {
            mode = myCooperativeLC;
        } else if ((state & LCA_SPEEDGAIN) != 0) {
            mode = mySpeedGainLC;
        } else if ((state & LCA_KEEPRIGHT) != 0) {
            mode = myRightDriveLC;
        } else if ((state & LCA_SUBLANE) != 0) {
            mode = mySublaneLC;
        } else if ((state & LCA_TRACI) != 0) {
            mode = LC_NEVER;
        } else {
            WRITE_WARNINGF(TL("Lane change model did not provide a reason for changing (state=%, time=%\n"), toString(state), time2string(currentTime));
        }
        if (mode == LC_NEVER) {
            // cancel all lcModel requests
            state &= ~LCA_WANTS_LANECHANGE_OR_STAY;
            state &= ~LCA_URGENT;
        } else if (mode == LC_NOCONFLICT && changeRequest != REQUEST_NONE) {
            if (
                ((state & LCA_LEFT) != 0 && changeRequest != REQUEST_LEFT) ||
                ((state & LCA_RIGHT) != 0 && changeRequest != REQUEST_RIGHT) ||
                ((state & LCA_STAY) != 0 && changeRequest != REQUEST_HOLD)) {
                // cancel conflicting lcModel request
                state &= ~LCA_WANTS_LANECHANGE_OR_STAY;
                state &= ~LCA_URGENT;
            }
        } else if (mode == LC_ALWAYS) {
            // ignore any TraCI requests
            return state;
        }
    }
    // apply traci requests
    if (changeRequest == REQUEST_NONE) {
        return state;
    } else {
        state |= LCA_TRACI;
        // security checks
        if ((myTraciLaneChangePriority == LCP_ALWAYS)
                || (myTraciLaneChangePriority == LCP_NOOVERLAP && (state & LCA_OVERLAPPING) == 0)) {
            state &= ~(LCA_BLOCKED | LCA_OVERLAPPING);
        }
        if (changeRequest != REQUEST_HOLD && myTraciLaneChangePriority != LCP_OPPORTUNISTIC) {
            state |= LCA_URGENT;
        }
        switch (changeRequest) {
            case REQUEST_HOLD:
                return state | LCA_STAY;
            case REQUEST_LEFT:
                return state | LCA_LEFT;
            case REQUEST_RIGHT:
                return state | LCA_RIGHT;
            default:
                throw ProcessError(TL("should not happen"));
        }
    }
}


double
MSVehicle::Influencer::changeRequestRemainingSeconds(const SUMOTime currentTime) const {
    assert(myLaneTimeLine.size() >= 2);
    assert(currentTime >= myLaneTimeLine[0].first);
    return STEPS2TIME(myLaneTimeLine[1].first - currentTime);
}


void
MSVehicle::Influencer::setSpeedMode(int speedMode) {
    myConsiderSafeVelocity = ((speedMode & 1) != 0);
    myConsiderMaxAcceleration = ((speedMode & 2) != 0);
    myConsiderMaxDeceleration = ((speedMode & 4) != 0);
    myRespectJunctionPriority = ((speedMode & 8) != 0);
    myEmergencyBrakeRedLight = ((speedMode & 16) != 0);
    myRespectJunctionLeaderPriority = ((speedMode & 32) == 0); // inverted!
}


void
MSVehicle::Influencer::setLaneChangeMode(int value) {
    myStrategicLC = (LaneChangeMode)(value & (1 + 2));
    myCooperativeLC = (LaneChangeMode)((value & (4 + 8)) >> 2);
    mySpeedGainLC = (LaneChangeMode)((value & (16 + 32)) >> 4);
    myRightDriveLC = (LaneChangeMode)((value & (64 + 128)) >> 6);
    myTraciLaneChangePriority = (TraciLaneChangePriority)((value & (256 + 512)) >> 8);
    mySublaneLC = (LaneChangeMode)((value & (1024 + 2048)) >> 10);
}


void
MSVehicle::Influencer::setRemoteControlled(Position xyPos, MSLane* l, double pos, double posLat, double angle, int edgeOffset, const ConstMSEdgeVector& route, SUMOTime t) {
    myRemoteXYPos = xyPos;
    myRemoteLane = l;
    myRemotePos = pos;
    myRemotePosLat = posLat;
    myRemoteAngle = angle;
    myRemoteEdgeOffset = edgeOffset;
    myRemoteRoute = route;
    myLastRemoteAccess = t;
}


bool
MSVehicle::Influencer::isRemoteControlled() const {
    return myLastRemoteAccess == MSNet::getInstance()->getCurrentTimeStep();
}


bool
MSVehicle::Influencer::isRemoteAffected(SUMOTime t) const {
    return myLastRemoteAccess >= t - TIME2STEPS(10);
}


void
MSVehicle::Influencer::updateRemoteControlRoute(MSVehicle* v) {
    if (myRemoteRoute.size() != 0 && myRemoteRoute != v->getRoute().getEdges()) {
        // only replace route at this time if the vehicle is moving with the flow
        const bool isForward = v->getLane() != 0 && &v->getLane()->getEdge() == myRemoteRoute[0];
#ifdef DEBUG_REMOTECONTROL
        std::cout << SIMSTEP << " updateRemoteControlRoute veh=" << v->getID() << " old=" << toString(v->getRoute().getEdges()) << " new=" << toString(myRemoteRoute) << " fwd=" << isForward << "\n";
#endif
        if (isForward) {
            v->replaceRouteEdges(myRemoteRoute, -1, 0, "traci:moveToXY", true);
            v->updateBestLanes();
        }
    }
}


void
MSVehicle::Influencer::postProcessRemoteControl(MSVehicle* v) {
    const bool wasOnRoad = v->isOnRoad();
    const bool withinLane = myRemoteLane != nullptr && fabs(myRemotePosLat) < 0.5 * (myRemoteLane->getWidth() + v->getVehicleType().getWidth());
    const bool keepLane = wasOnRoad && v->getLane() == myRemoteLane;
    if (v->isOnRoad() && !(keepLane && withinLane)) {
        if (myRemoteLane != nullptr && &v->getLane()->getEdge() == &myRemoteLane->getEdge()) {
            // correct odometer which gets incremented via onRemovalFromNet->leaveLane
            v->myOdometer -= v->getLane()->getLength();
        }
        v->onRemovalFromNet(MSMoveReminder::NOTIFICATION_TELEPORT);
        v->getMutableLane()->removeVehicle(v, MSMoveReminder::NOTIFICATION_TELEPORT, false);
    }
    if (myRemoteRoute.size() != 0 && myRemoteRoute != v->getRoute().getEdges()) {
        // needed for the insertion step
#ifdef DEBUG_REMOTECONTROL
        std::cout << SIMSTEP << " postProcessRemoteControl veh=" << v->getID()
                  << "\n  oldLane=" << Named::getIDSecure(v->getLane())
                  << " oldRoute=" << toString(v->getRoute().getEdges())
                  << "\n  newLane=" << Named::getIDSecure(myRemoteLane)
                  << " newRoute=" << toString(myRemoteRoute)
                  << " newRouteEdge=" << myRemoteRoute[myRemoteEdgeOffset]->getID()
                  << "\n";
#endif
        // clear any prior stops because they cannot apply to the new route
        const_cast<SUMOVehicleParameter&>(v->getParameter()).stops.clear();
        v->replaceRouteEdges(myRemoteRoute, -1, 0, "traci:moveToXY", true);
        myRemoteRoute.clear();
    }
    v->myCurrEdge = v->getRoute().begin() + myRemoteEdgeOffset;
    if (myRemoteLane != nullptr && myRemotePos > myRemoteLane->getLength()) {
        myRemotePos = myRemoteLane->getLength();
    }
    if (myRemoteLane != nullptr && withinLane) {
        if (keepLane) {
            // TODO this handles only the case when the new vehicle is completely on the edge
            const bool needFurtherUpdate = v->myState.myPos < v->getVehicleType().getLength() && myRemotePos >= v->getVehicleType().getLength();
            v->myState.myPos = myRemotePos;
            v->myState.myPosLat = myRemotePosLat;
            if (needFurtherUpdate) {
                v->myState.myBackPos = v->updateFurtherLanes(v->myFurtherLanes, v->myFurtherLanesPosLat, std::vector<MSLane*>());
            }
        } else {
            MSMoveReminder::Notification notify = v->getDeparture() == NOT_YET_DEPARTED
                                                  ? MSMoveReminder::NOTIFICATION_DEPARTED
                                                  : MSMoveReminder::NOTIFICATION_TELEPORT_ARRIVED;
            if (!v->isOnRoad()) {
                MSVehicleTransfer::getInstance()->remove(v);  // TODO may need optimization, this is linear in the number of vehicles in transfer
            }
            myRemoteLane->forceVehicleInsertion(v, myRemotePos, notify, myRemotePosLat);
            v->updateBestLanes();
        }
        if (!wasOnRoad) {
            v->drawOutsideNetwork(false);
        }
        //std::cout << "on road network p=" << myRemoteXYPos << " a=" << myRemoteAngle << " l=" << Named::getIDSecure(myRemoteLane) << " pos=" << myRemotePos << " posLat=" << myRemotePosLat << "\n";
        myRemoteLane->requireCollisionCheck();
    } else {
        if (v->getDeparture() == NOT_YET_DEPARTED) {
            v->onDepart();
        }
        v->drawOutsideNetwork(true);
        // see updateState
        double vNext = v->processTraCISpeedControl(
                           v->getMaxSpeed(), v->getSpeed());
        v->setBrakingSignals(vNext);
        v->myState.myPreviousSpeed = v->getSpeed();
        v->myAcceleration = SPEED2ACCEL(vNext - v->getSpeed());
        v->myState.mySpeed = vNext;
        v->updateWaitingTime(vNext);
        //std::cout << "outside network p=" << myRemoteXYPos << " a=" << myRemoteAngle << " l=" << Named::getIDSecure(myRemoteLane) << "\n";
    }
    // ensure that the position is correct (i.e. when the lanePosition is ambiguous at corners)
    v->setRemoteState(myRemoteXYPos);
    v->setAngle(GeomHelper::fromNaviDegree(myRemoteAngle));
}


double
MSVehicle::Influencer::implicitSpeedRemote(const MSVehicle* veh, double oldSpeed) {
    if (veh->getPosition() == Position::INVALID) {
        return oldSpeed;
    }
    double dist = veh->getPosition().distanceTo2D(myRemoteXYPos);
    if (myRemoteLane != nullptr) {
        // if the vehicles is frequently placed on a new edge, the route may
        // consist only of a single edge. In this case the new edge may not be
        // on the route so distAlongRoute will be double::max.
        // In this case we still want a sensible speed value
        const double distAlongRoute = veh->getDistanceToPosition(myRemotePos, myRemoteLane);
        if (distAlongRoute != std::numeric_limits<double>::max()) {
            dist = distAlongRoute;
        }
    }
    //std::cout << SIMTIME << " veh=" << veh->getID() << " oldPos=" << veh->getPosition() << " traciPos=" << myRemoteXYPos << " dist=" << dist << "\n";
    const double minSpeed = myConsiderMaxDeceleration ?
                            veh->getCarFollowModel().minNextSpeedEmergency(oldSpeed, veh) : 0;
    const double maxSpeed = (myRemoteLane != nullptr
                             ? myRemoteLane->getVehicleMaxSpeed(veh)
                             : (veh->getLane() != nullptr
                                ? veh->getLane()->getVehicleMaxSpeed(veh)
                                : veh->getMaxSpeed()));
    return MIN2(maxSpeed, MAX2(minSpeed, DIST2SPEED(dist)));
}


double
MSVehicle::Influencer::implicitDeltaPosRemote(const MSVehicle* veh) {
    double dist = 0;
    if (myRemoteLane == nullptr) {
        dist = veh->getPosition().distanceTo2D(myRemoteXYPos);
    } else {
        // if the vehicles is frequently placed on a new edge, the route may
        // consist only of a single edge. In this case the new edge may not be
        // on the route so getDistanceToPosition will return double::max.
        // In this case we would rather not move the vehicle in executeMove
        // (updateState) as it would result in emergency braking
        dist = veh->getDistanceToPosition(myRemotePos, myRemoteLane);
    }
    if (dist == std::numeric_limits<double>::max()) {
        return 0;
    } else {
        if (DIST2SPEED(dist) > veh->getMaxSpeed() * 1.1) {
            WRITE_WARNINGF(TL("Vehicle '%' moved by TraCI from % to % (dist %) with implied speed of % (exceeding maximum speed %). time=%."),
                           veh->getID(), veh->getPosition(), myRemoteXYPos, dist, DIST2SPEED(dist), veh->getMaxSpeed(), time2string(SIMSTEP));
            // some sanity check here
            dist = MIN2(dist, SPEED2DIST(veh->getMaxSpeed() * 2));
        }
        return dist;
    }
}


/* -------------------------------------------------------------------------
 * MSVehicle-methods
 * ----------------------------------------------------------------------- */
MSVehicle::MSVehicle(SUMOVehicleParameter* pars, ConstMSRoutePtr route,
                     MSVehicleType* type, const double speedFactor) :
    MSBaseVehicle(pars, route, type, speedFactor),
    myWaitingTime(0),
    myWaitingTimeCollector(),
    myTimeLoss(0),
    myState(0, 0, 0, 0, 0),
    myDriverState(nullptr),
    myActionStep(true),
    myLastActionTime(0),
    myLane(nullptr),
    myLaneChangeModel(nullptr),
    myLastBestLanesEdge(nullptr),
    myLastBestLanesInternalLane(nullptr),
    myAcceleration(0),
    myNextTurn(0., nullptr),
    mySignals(0),
    myAmOnNet(false),
    myAmIdling(false),
    myHaveToWaitOnNextLink(false),
    myAngle(0),
    myStopDist(std::numeric_limits<double>::max()),
    myCollisionImmunity(-1),
    myCachedPosition(Position::INVALID),
    myJunctionEntryTime(SUMOTime_MAX),
    myJunctionEntryTimeNeverYield(SUMOTime_MAX),
    myJunctionConflictEntryTime(SUMOTime_MAX),
    myTimeSinceStartup(TIME2STEPS(3600 * 24)),
    myInfluencer(nullptr) {
    myCFVariables = type->getCarFollowModel().createVehicleVariables();
    myNextDriveItem = myLFLinkLanes.begin();
}


MSVehicle::~MSVehicle() {
    cleanupFurtherLanes();
    delete myLaneChangeModel;
    if (myType->isVehicleSpecific()) {
        MSNet::getInstance()->getVehicleControl().removeVType(myType);
    }
    delete myInfluencer;
    delete myCFVariables;
}


void
MSVehicle::cleanupFurtherLanes() {
    for (MSLane* further : myFurtherLanes) {
        further->resetPartialOccupation(this);
        if (further->getBidiLane() != nullptr
                && (!isRailway(getVClass()) || (further->getPermissions() & ~SVC_RAIL_CLASSES) != 0)) {
            further->getBidiLane()->resetPartialOccupation(this);
        }
    }
    if (myLaneChangeModel != nullptr) {
        removeApproachingInformation(myLFLinkLanes);
        myLaneChangeModel->cleanupShadowLane();
        myLaneChangeModel->cleanupTargetLane();
        // still needed when calling resetPartialOccupation (getShadowLane) and when removing
        // approach information from parallel links
    }
    myFurtherLanes.clear();
    myFurtherLanesPosLat.clear();
}


void
MSVehicle::onRemovalFromNet(const MSMoveReminder::Notification reason) {
#ifdef DEBUG_ACTIONSTEPS
    if (DEBUG_COND) {
        std::cout << SIMTIME << " Removing vehicle '" << getID() << "' (reason: " << toString(reason) << ")" << std::endl;
    }
#endif
    MSVehicleTransfer::getInstance()->remove(this);
    removeApproachingInformation(myLFLinkLanes);
    leaveLane(reason);
    if (reason == MSMoveReminder::NOTIFICATION_VAPORIZED_COLLISION) {
        cleanupFurtherLanes();
    }
}


void
MSVehicle::initDevices() {
    MSBaseVehicle::initDevices();
    myLaneChangeModel = MSAbstractLaneChangeModel::build(myType->getLaneChangeModel(), *this);
    myDriverState = static_cast<MSDevice_DriverState*>(getDevice(typeid(MSDevice_DriverState)));
    myFrictionDevice = static_cast<MSDevice_Friction*>(getDevice(typeid(MSDevice_Friction)));
}


// ------------ interaction with the route
bool
MSVehicle::hasValidRouteStart(std::string& msg) {
    // note: not a const method because getDepartLane may call updateBestLanes
    if (!(*myCurrEdge)->isTazConnector()) {
        if (myParameter->departLaneProcedure == DepartLaneDefinition::GIVEN) {
            if ((*myCurrEdge)->getDepartLane(*this) == nullptr) {
                msg = "Invalid departlane definition for vehicle '" + getID() + "'.";
                if (myParameter->departLane >= (int)(*myCurrEdge)->getLanes().size()) {
                    myRouteValidity |= ROUTE_START_INVALID_LANE;
                } else {
                    myRouteValidity |= ROUTE_START_INVALID_PERMISSIONS;
                }
                return false;
            }
        } else {
            if ((*myCurrEdge)->allowedLanes(getVClass()) == nullptr) {
                msg = "Vehicle '" + getID() + "' is not allowed to depart on any lane of edge '" + (*myCurrEdge)->getID() + "'.";
                myRouteValidity |= ROUTE_START_INVALID_PERMISSIONS;
                return false;
            }
        }
        if (myParameter->departSpeedProcedure == DepartSpeedDefinition::GIVEN && myParameter->departSpeed > myType->getMaxSpeed() + SPEED_EPS) {
            msg = "Departure speed for vehicle '" + getID() + "' is too high for the vehicle type '" + myType->getID() + "'.";
            myRouteValidity |= ROUTE_START_INVALID_LANE;
            return false;
        }
    }
    myRouteValidity &= ~(ROUTE_START_INVALID_LANE | ROUTE_START_INVALID_PERMISSIONS);
    return true;
}


bool
MSVehicle::hasArrived() const {
    return hasArrivedInternal(false);
}


bool
MSVehicle::hasArrivedInternal(bool oppositeTransformed) const {
    return ((myCurrEdge == myRoute->end() - 1 || (myParameter->arrivalEdge >= 0 && getRoutePosition() >= myParameter->arrivalEdge))
            && (myStops.empty() || myStops.front().edge != myCurrEdge || myStops.front().getSpeed() > 0)
            && ((myLaneChangeModel->isOpposite() && !oppositeTransformed) ? myLane->getLength() - myState.myPos : myState.myPos) > myArrivalPos - POSITION_EPS
            && !isRemoteControlled());
}


bool
MSVehicle::replaceRoute(ConstMSRoutePtr newRoute, const std::string& info, bool onInit, int offset, bool addRouteStops, bool removeStops, std::string* msgReturn) {
    if (MSBaseVehicle::replaceRoute(newRoute, info, onInit, offset, addRouteStops, removeStops, msgReturn)) {
        // update best lanes (after stops were added)
        myLastBestLanesEdge = nullptr;
        myLastBestLanesInternalLane = nullptr;
        updateBestLanes(true, onInit ? (*myCurrEdge)->getLanes().front() : 0);
        assert(!removeStops || haveValidStopEdges());
        if (myStops.size() == 0) {
            myStopDist = std::numeric_limits<double>::max();
        }
        return true;
    }
    return false;
}


// ------------ Interaction with move reminders
void
MSVehicle::workOnMoveReminders(double oldPos, double newPos, double newSpeed) {
    // This erasure-idiom works for all stl-sequence-containers
    // See Meyers: Effective STL, Item 9
    for (MoveReminderCont::iterator rem = myMoveReminders.begin(); rem != myMoveReminders.end();) {
        // XXX: calling notifyMove with newSpeed seems not the best choice. For the ballistic update, the average speed is calculated and used
        //      although a higher order quadrature-formula might be more adequate.
        //      For the euler case (where the speed is considered constant for each time step) it is conceivable that
        //      the current calculations may lead to systematic errors for large time steps (compared to reality). Refs. #2579
        if (!rem->first->notifyMove(*this, oldPos + rem->second, newPos + rem->second, MAX2(0., newSpeed))) {
#ifdef _DEBUG
            if (myTraceMoveReminders) {
                traceMoveReminder("notifyMove", rem->first, rem->second, false);
            }
#endif
            rem = myMoveReminders.erase(rem);
        } else {
#ifdef _DEBUG
            if (myTraceMoveReminders) {
                traceMoveReminder("notifyMove", rem->first, rem->second, true);
            }
#endif
            ++rem;
        }
    }
    if (myEnergyParams != nullptr) {
        // TODO make the vehicle energy params a derived class which is a move reminder
        const double duration = myEnergyParams->getDouble(SUMO_ATTR_DURATION);
        if (isStopped()) {
            if (duration < 0) {
                myEnergyParams->setDouble(SUMO_ATTR_DURATION, STEPS2TIME(getNextStop().duration));
                myEnergyParams->setDouble(SUMO_ATTR_PARKING, isParking() ? 1. : 0.);
            }
        } else {
            if (duration >= 0) {
                myEnergyParams->setDouble(SUMO_ATTR_DURATION, -1.);
            }
        }
        myEnergyParams->setDouble(SUMO_ATTR_WAITINGTIME, getWaitingSeconds());
    }
}


void
MSVehicle::workOnIdleReminders() {
    updateWaitingTime(0.);   // cf issue 2233

    // vehicle move reminders
    for (const auto& rem : myMoveReminders) {
        rem.first->notifyIdle(*this);
    }

    // lane move reminders - for aggregated values
    for (MSMoveReminder* rem : getLane()->getMoveReminders()) {
        rem->notifyIdle(*this);
    }
}

// XXX: consider renaming...
void
MSVehicle::adaptLaneEntering2MoveReminder(const MSLane& enteredLane) {
    // save the old work reminders, patching the position information
    //  add the information about the new offset to the old lane reminders
    const double oldLaneLength = myLane->getLength();
    for (auto& rem : myMoveReminders) {
        rem.second += oldLaneLength;
#ifdef _DEBUG
//        if (rem->first==0) std::cout << "Null reminder (?!)" << std::endl;
//        std::cout << "Adapted MoveReminder on lane " << ((rem->first->getLane()==0) ? "NULL" : rem->first->getLane()->getID()) <<" position to " << rem->second << std::endl;
        if (myTraceMoveReminders) {
            traceMoveReminder("adaptedPos", rem.first, rem.second, true);
        }
#endif
    }
    for (MSMoveReminder* const rem : enteredLane.getMoveReminders()) {
        addReminder(rem);
    }
}


// ------------ Other getter methods
double
MSVehicle::getSlope() const {
    if (isParking() && getStops().begin()->parkingarea != nullptr) {
        return getStops().begin()->parkingarea->getVehicleSlope(*this);
    }
    if (myLane == nullptr) {
        return 0;
    }
    const double posLat = myState.myPosLat; // @todo get rid of the '-'
    Position p1 = getPosition();
    Position p2 = getBackPosition();
    if (p2 == Position::INVALID) {
        // Handle special case of vehicle's back reaching out of the network
        if (myFurtherLanes.size() > 0) {
            p2 = myFurtherLanes.back()->geometryPositionAtOffset(0, -myFurtherLanesPosLat.back());
            if (p2 == Position::INVALID) {
                // unsuitable lane geometry
                p2 = myLane->geometryPositionAtOffset(0, posLat);
            }
        } else {
            p2 = myLane->geometryPositionAtOffset(0, posLat);
        }
    }
    return (p1 != p2 ? RAD2DEG(p2.slopeTo2D(p1)) : myLane->getShape().slopeDegreeAtOffset(myLane->interpolateLanePosToGeometryPos(getPositionOnLane())));
}


Position
MSVehicle::getPosition(const double offset) const {
    if (myLane == nullptr) {
        // when called in the context of GUI-Drawing, the simulation step is already incremented
        if (myInfluencer != nullptr && myInfluencer->isRemoteAffected(MSNet::getInstance()->getCurrentTimeStep())) {
            return myCachedPosition;
        } else {
            return Position::INVALID;
        }
    }
    if (isParking()) {
        if (myStops.begin()->parkingarea != nullptr) {
            return myStops.begin()->parkingarea->getVehiclePosition(*this);
        } else {
            // position beside the road
            PositionVector shp = myLane->getEdge().getLanes()[0]->getShape();
            shp.move2side(SUMO_const_laneWidth * (MSGlobals::gLefthand ? -1 : 1));
            return shp.positionAtOffset(myLane->interpolateLanePosToGeometryPos(getPositionOnLane() + offset));
        }
    }
    const bool changingLanes = myLaneChangeModel->isChangingLanes();
    const double posLat = (MSGlobals::gLefthand ? 1 : -1) * getLateralPositionOnLane();
    if (offset == 0. && !changingLanes) {
        if (myCachedPosition == Position::INVALID) {
            myCachedPosition = validatePosition(myLane->geometryPositionAtOffset(myState.myPos, posLat));
            if (MSNet::getInstance()->hasElevation() && MSGlobals::gSublane) {
                interpolateLateralZ(myCachedPosition, myState.myPos, posLat);
            }
        }
        return myCachedPosition;
    }
    Position result = validatePosition(myLane->geometryPositionAtOffset(getPositionOnLane() + offset, posLat), offset);
    interpolateLateralZ(result, getPositionOnLane() + offset, posLat);
    return result;
}


void
MSVehicle::interpolateLateralZ(Position& pos, double offset, double posLat) const {
    const MSLane* shadow = myLaneChangeModel->getShadowLane();
    if (shadow != nullptr && pos != Position::INVALID) {
        // ignore negative offset
        const Position shadowPos = shadow->geometryPositionAtOffset(MAX2(0.0, offset));
        if (shadowPos != Position::INVALID && pos.z() != shadowPos.z()) {
            const double centerDist = (myLane->getWidth() + shadow->getWidth()) * 0.5;
            double relOffset = fabs(posLat) / centerDist;
            double newZ = (1 - relOffset) * pos.z() + relOffset * shadowPos.z();
            pos.setz(newZ);
        }
    }
}


double
MSVehicle::getDistanceToLeaveJunction() const {
    double result = getLength() - getPositionOnLane();
    if (myLane->isNormal()) {
        return MAX2(0.0, result);
    }
    const MSLane* lane = myLane;
    while (lane->isInternal()) {
        result += lane->getLength();
        lane = lane->getCanonicalSuccessorLane();
    }
    return result;
}


Position
MSVehicle::getPositionAlongBestLanes(double offset) const {
    assert(MSGlobals::gUsingInternalLanes);
    if (!isOnRoad()) {
        return Position::INVALID;
    }
    const std::vector<MSLane*>& bestLanes = getBestLanesContinuation();
    auto nextBestLane = bestLanes.begin();
    const bool opposite = myLaneChangeModel->isOpposite();
    double pos = opposite ? myLane->getLength() - myState.myPos : myState.myPos;
    const MSLane* lane = opposite ? myLane->getParallelOpposite() : getLane();
    assert(lane != 0);
    bool success = true;

    while (offset > 0) {
        // take into account lengths along internal lanes
        while (lane->isInternal() && offset > 0) {
            if (offset > lane->getLength() - pos) {
                offset -= lane->getLength() - pos;
                lane = lane->getLinkCont()[0]->getViaLaneOrLane();
                pos = 0.;
                if (lane == nullptr) {
                    success = false;
                    offset = 0.;
                }
            } else {
                pos += offset;
                offset = 0;
            }
        }
        // set nextBestLane to next non-internal lane
        while (nextBestLane != bestLanes.end() && *nextBestLane == nullptr) {
            ++nextBestLane;
        }
        if (offset > 0) {
            assert(!lane->isInternal());
            assert(lane == *nextBestLane);
            if (offset > lane->getLength() - pos) {
                offset -= lane->getLength() - pos;
                ++nextBestLane;
                assert(nextBestLane == bestLanes.end() || *nextBestLane != 0);
                if (nextBestLane == bestLanes.end()) {
                    success = false;
                    offset = 0.;
                } else {
                    const MSLink* link = lane->getLinkTo(*nextBestLane);
                    assert(link != nullptr);
                    lane = link->getViaLaneOrLane();
                    pos = 0.;
                }
            } else {
                pos += offset;
                offset = 0;
            }
        }

    }

    if (success) {
        return lane->geometryPositionAtOffset(pos, -getLateralPositionOnLane());
    } else {
        return Position::INVALID;
    }
}


double
MSVehicle::getMaxSpeedOnLane() const {
    if (myLane != nullptr) {
        return myLane->getVehicleMaxSpeed(this);
    }
    return myType->getMaxSpeed();
}


Position
MSVehicle::validatePosition(Position result, double offset) const {
    int furtherIndex = 0;
    double lastLength = getPositionOnLane();
    while (result == Position::INVALID) {
        if (furtherIndex >= (int)myFurtherLanes.size()) {
            //WRITE_WARNINGF(TL("Could not compute position for vehicle '%', time=%."), getID(), time2string(MSNet::getInstance()->getCurrentTimeStep()));
            break;
        }
        //std::cout << SIMTIME << " veh=" << getID() << " lane=" << myLane->getID() << " pos=" << getPositionOnLane() << " posLat=" << getLateralPositionOnLane() << " offset=" << offset << " result=" << result << " i=" << furtherIndex << " further=" << myFurtherLanes.size() << "\n";
        MSLane* further = myFurtherLanes[furtherIndex];
        offset += lastLength;
        result = further->geometryPositionAtOffset(further->getLength() + offset, -getLateralPositionOnLane());
        lastLength = further->getLength();
        furtherIndex++;
        //std::cout << SIMTIME << "   newResult=" << result << "\n";
    }
    return result;
}


ConstMSEdgeVector::const_iterator
MSVehicle::getRerouteOrigin() const {
    // too close to the next junction, so avoid an emergency brake here
    if (myLane != nullptr && (myCurrEdge + 1) != myRoute->end()) {
        if (myLane->isInternal()) {
            return myCurrEdge + 1;
        }
        if (myState.myPos > myLane->getLength() - getCarFollowModel().brakeGap(myState.mySpeed, getCarFollowModel().getMaxDecel(), 0.)) {
            return myCurrEdge + 1;
        }
        if (myLane->getEdge().hasChangeProhibitions(getVClass(), myLane->getIndex())) {
            return myCurrEdge + 1;
        }
    }
    return myCurrEdge;
}

void
MSVehicle::setAngle(double angle, bool straightenFurther) {
#ifdef DEBUG_FURTHER
    if (DEBUG_COND) {
        std::cout << SIMTIME << " veh '" << getID() << " setAngle(" << angle <<  ") straightenFurther=" << straightenFurther << std::endl;
    }
#endif
    myAngle = angle;
    MSLane* next = myLane;
    if (straightenFurther && myFurtherLanesPosLat.size() > 0) {
        for (int i = 0; i < (int)myFurtherLanes.size(); i++) {
            MSLane* further = myFurtherLanes[i];
            const MSLink* link = further->getLinkTo(next);
            if (link  != nullptr) {
                myFurtherLanesPosLat[i] = getLateralPositionOnLane() - link->getLateralShift();
                next = further;
            } else {
                break;
            }
        }
    }
}


void
MSVehicle::setActionStepLength(double actionStepLength, bool resetOffset) {
    SUMOTime actionStepLengthMillisecs = SUMOVehicleParserHelper::processActionStepLength(actionStepLength);
    SUMOTime previousActionStepLength = getActionStepLength();
    const bool newActionStepLength = actionStepLengthMillisecs != previousActionStepLength;
    if (newActionStepLength) {
        getSingularType().setActionStepLength(actionStepLengthMillisecs, resetOffset);
        if (!resetOffset) {
            updateActionOffset(previousActionStepLength, actionStepLengthMillisecs);
        }
    }
    if (resetOffset) {
        resetActionOffset();
    }
}


bool
MSVehicle::congested() const {
    return myState.mySpeed < (60.0 / 3.6) || myLane->getSpeedLimit() < (60.1 / 3.6);
}


double
MSVehicle::computeAngle() const {
    Position p1;
    const double posLat = -myState.myPosLat; // @todo get rid of the '-'
    const double lefthandSign = (MSGlobals::gLefthand ? -1 : 1);

    // if parking manoeuvre is happening then rotate vehicle on each step
    if (MSGlobals::gModelParkingManoeuver && !manoeuvreIsComplete()) {
        return getAngle() + myManoeuvre.getGUIIncrement();
    }

    if (isParking()) {
        if (myStops.begin()->parkingarea != nullptr) {
            return myStops.begin()->parkingarea->getVehicleAngle(*this);
        } else {
            return myLane->getShape().rotationAtOffset(myLane->interpolateLanePosToGeometryPos(getPositionOnLane()));
        }
    }
    if (myLaneChangeModel->isChangingLanes()) {
        // cannot use getPosition() because it already includes the offset to the side and thus messes up the angle
        p1 = myLane->geometryPositionAtOffset(myState.myPos, lefthandSign * posLat);
        if (p1 == Position::INVALID && myLane->getShape().length2D() == 0. && myLane->isInternal()) {
            // workaround: extrapolate the preceding lane shape
            MSLane* predecessorLane = myLane->getCanonicalPredecessorLane();
            p1 = predecessorLane->geometryPositionAtOffset(predecessorLane->getLength() + myState.myPos, lefthandSign * posLat);
        }
    } else {
        p1 = getPosition();
    }

    Position p2;
    if (getVehicleType().getParameter().locomotiveLength > 0) {
        // articulated vehicle should use the heading of the first part
        const double locoLength = MIN2(getVehicleType().getParameter().locomotiveLength, getLength());
        p2 = getPosition(-locoLength);
    } else {
        p2 = getBackPosition();
    }
    if (p2 == Position::INVALID) {
        // Handle special case of vehicle's back reaching out of the network
        if (myFurtherLanes.size() > 0) {
            p2 = myFurtherLanes.back()->geometryPositionAtOffset(0, -myFurtherLanesPosLat.back());
            if (p2 == Position::INVALID) {
                // unsuitable lane geometry
                p2 = myLane->geometryPositionAtOffset(0, posLat);
            }
        } else {
            p2 = myLane->geometryPositionAtOffset(0, posLat);
        }
    }
    double result = (p1 != p2 ? p2.angleTo2D(p1) :
                     myLane->getShape().rotationAtOffset(myLane->interpolateLanePosToGeometryPos(getPositionOnLane())));

    result += lefthandSign * myLaneChangeModel->calcAngleOffset();

#ifdef DEBUG_FURTHER
    if (DEBUG_COND) {
        std::cout << SIMTIME << " computeAngle veh=" << getID() << " p1=" << p1 << " p2=" << p2 << " angle=" << RAD2DEG(result) << " naviDegree=" << GeomHelper::naviDegree(result) << "\n";
    }
#endif
    return result;
}


const Position
MSVehicle::getBackPosition() const {
    const double posLat = MSGlobals::gLefthand ? myState.myPosLat : -myState.myPosLat;
    Position result;
    if (myState.myPos >= myType->getLength()) {
        // vehicle is fully on the new lane
        result = myLane->geometryPositionAtOffset(myState.myPos - myType->getLength(), posLat);
    } else {
        if (myLaneChangeModel->isChangingLanes() && myFurtherLanes.size() > 0 && myLaneChangeModel->getShadowLane(myFurtherLanes.back()) == nullptr) {
            // special case where the target lane has no predecessor
#ifdef DEBUG_FURTHER
            if (DEBUG_COND) {
                std::cout << "    getBackPosition veh=" << getID() << " specialCase using myLane=" << myLane->getID() << " pos=0 posLat=" << myState.myPosLat << " result=" << myLane->geometryPositionAtOffset(0, posLat) << "\n";
            }
#endif
            result = myLane->geometryPositionAtOffset(0, posLat);
        } else {
#ifdef DEBUG_FURTHER
            if (DEBUG_COND) {
                std::cout << "    getBackPosition veh=" << getID() << " myLane=" << myLane->getID() << " further=" << toString(myFurtherLanes) << " myFurtherLanesPosLat=" << toString(myFurtherLanesPosLat) << "\n";
            }
#endif
            if (myFurtherLanes.size() > 0 && !myLaneChangeModel->isChangingLanes()) {
                // truncate to 0 if vehicle starts on an edge that is shorter than its length
                const double backPos = MAX2(0.0, getBackPositionOnLane(myFurtherLanes.back()));
                result = myFurtherLanes.back()->geometryPositionAtOffset(backPos, -myFurtherLanesPosLat.back() * (MSGlobals::gLefthand ? -1 : 1));
            } else {
                result = myLane->geometryPositionAtOffset(0, posLat);
            }
        }
    }
    if (MSNet::getInstance()->hasElevation() && MSGlobals::gSublane) {
        interpolateLateralZ(result, myState.myPos - myType->getLength(), posLat);
    }
    return result;
}


bool
MSVehicle::willStop() const {
    return !isStopped() && !myStops.empty() && myLane != nullptr && &myStops.front().lane->getEdge() == &myLane->getEdge();
}

bool
MSVehicle::isStoppedOnLane() const {
    return isStopped() && myStops.front().lane == myLane;
}

bool
MSVehicle::keepStopping(bool afterProcessing) const {
    if (isStopped()) {
        // when coming out of vehicleTransfer we must shift the time forward
        return (myStops.front().duration - (afterProcessing ? DELTA_T : 0) > 0 || isStoppedTriggered() || myStops.front().pars.collision
                || myStops.front().pars.breakDown || (myStops.front().getSpeed() > 0
                        && (myState.myPos < MIN2(myStops.front().pars.endPos, myStops.front().lane->getLength() - POSITION_EPS))
                        && (myStops.front().pars.parking == ParkingType::ONROAD || getSpeed() >= SUMO_const_haltingSpeed)));
    } else {
        return false;
    }
}


SUMOTime
MSVehicle::remainingStopDuration() const {
    if (isStopped()) {
        return myStops.front().duration;
    }
    return 0;
}


SUMOTime
MSVehicle::collisionStopTime() const {
    return (myStops.empty() || !myStops.front().pars.collision) ? myCollisionImmunity : MAX2((SUMOTime)0, myStops.front().duration);
}


bool
MSVehicle::brokeDown() const {
    return isStopped() && !myStops.empty() && myStops.front().pars.breakDown;
}


bool
MSVehicle::ignoreCollision() const {
    return myCollisionImmunity > 0;
}


double
MSVehicle::processNextStop(double currentVelocity) {
    if (myStops.empty()) {
        // no stops; pass
        return currentVelocity;
    }

#ifdef DEBUG_STOPS
    if (DEBUG_COND) {
        std::cout << "\nPROCESS_NEXT_STOP\n" << SIMTIME << " vehicle '" << getID() << "'" << std::endl;
    }
#endif

    MSStop& stop = myStops.front();
    const SUMOTime time = MSNet::getInstance()->getCurrentTimeStep();
    if (stop.reached) {
        stop.duration -= getActionStepLength();

#ifdef DEBUG_STOPS
        if (DEBUG_COND) {
            std::cout << SIMTIME << " vehicle '" << getID() << "' reached stop.\n"
                      << "Remaining duration: " << STEPS2TIME(stop.duration) << std::endl;
            if (stop.getSpeed() > 0) {
                std::cout << " waypointSpeed=" << stop.getSpeed() << " vehPos=" << myState.myPos << " endPos=" << stop.pars.endPos << "\n";
            }
        }
#endif
        if (stop.duration <= 0 && stop.pars.join != "") {
            // join this train (part) to another one
            MSVehicle* joinVeh = dynamic_cast<MSVehicle*>(MSNet::getInstance()->getVehicleControl().getVehicle(stop.pars.join));
            if (joinVeh && joinVeh->hasDeparted() && (joinVeh->joinTrainPart(this) || joinVeh->joinTrainPartFront(this))) {
                stop.joinTriggered = false;
                if (myAmRegisteredAsWaiting) {
                    MSNet::getInstance()->getVehicleControl().unregisterOneWaiting();
                    myAmRegisteredAsWaiting = false;
                }
                // avoid collision warning before this vehicle is removed (joinVeh was already made longer)
                myCollisionImmunity = TIME2STEPS(100);
                // mark this vehicle as arrived
                myArrivalPos = getPositionOnLane();
                const_cast<SUMOVehicleParameter*>(myParameter)->arrivalEdge = getRoutePosition();
                // handle transportables that want to continue in the other vehicle
                if (myPersonDevice != nullptr) {
                    myPersonDevice->transferAtSplitOrJoin(joinVeh);
                }
                if (myContainerDevice != nullptr) {
                    myContainerDevice->transferAtSplitOrJoin(joinVeh);
                }
            }
        }
        boardTransportables(stop);
        if (!keepStopping() && isOnRoad()) {
#ifdef DEBUG_STOPS
            if (DEBUG_COND) {
                std::cout << SIMTIME << " vehicle '" << getID() << "' resumes from stopping." << std::endl;
            }
#endif
            resumeFromStopping();
            if (isRailway(getVClass())
                    && hasStops()) {
                // stay on the current lane in case of a double stop
                const MSStop& nextStop = getNextStop();
                if (nextStop.edge == myCurrEdge) {
                    const double stopSpeed = getCarFollowModel().stopSpeed(this, getSpeed(), nextStop.pars.endPos - myState.myPos);
                    //std::cout << SIMTIME << " veh=" << getID() << " resumedFromStopping currentVelocity=" << currentVelocity << " stopSpeed=" << stopSpeed << "\n";
                    return stopSpeed;
                }
            }
        } else {
            if (stop.triggered) {
                if (getVehicleType().getPersonCapacity() == getPersonNumber()) {
                    WRITE_WARNINGF(TL("Vehicle '%' ignores triggered stop on lane '%' due to capacity constraints."), getID(), stop.lane->getID());
                    stop.triggered = false;
                } else if (!myAmRegisteredAsWaiting && stop.duration <= DELTA_T) {
                    // we can only register after waiting for one step. otherwise we might falsely signal a deadlock
                    MSNet::getInstance()->getVehicleControl().registerOneWaiting();
                    myAmRegisteredAsWaiting = true;
#ifdef DEBUG_STOPS
                    if (DEBUG_COND) {
                        std::cout << SIMTIME << " vehicle '" << getID() << "' registers as waiting for person." << std::endl;
                    }
#endif
                }
            }
            if (stop.containerTriggered) {
                if (getVehicleType().getContainerCapacity() == getContainerNumber()) {
                    WRITE_WARNINGF(TL("Vehicle '%' ignores container triggered stop on lane '%' due to capacity constraints."), getID(), stop.lane->getID());
                    stop.containerTriggered = false;
                } else if (stop.containerTriggered && !myAmRegisteredAsWaiting && stop.duration <= DELTA_T) {
                    // we can only register after waiting for one step. otherwise we might falsely signal a deadlock
                    MSNet::getInstance()->getVehicleControl().registerOneWaiting();
                    myAmRegisteredAsWaiting = true;
#ifdef DEBUG_STOPS
                    if (DEBUG_COND) {
                        std::cout << SIMTIME << " vehicle '" << getID() << "' registers as waiting for container." << std::endl;
                    }
#endif
                }
            }
            // joining only takes place after stop duration is over
            if (stop.joinTriggered && !myAmRegisteredAsWaiting
                    && stop.duration <= (stop.pars.extension >= 0 ? -stop.pars.extension : 0)) {
                if (stop.pars.extension >= 0) {
                    WRITE_WARNINGF(TL("Vehicle '%' aborts joining after extension of %s at time %."), getID(), STEPS2TIME(stop.pars.extension), time2string(SIMSTEP));
                    stop.joinTriggered = false;
                } else {
                    // keep stopping indefinitely but ensure that simulation terminates
                    MSNet::getInstance()->getVehicleControl().registerOneWaiting();
                    myAmRegisteredAsWaiting = true;
                }
            }
            if (stop.getSpeed() > 0) {
                //waypoint mode
                if (stop.duration == 0) {
                    return stop.getSpeed();
                } else {
                    // stop for 'until' (computed in planMove)
                    return currentVelocity;
                }
            } else {
                // brake
                if (MSGlobals::gSemiImplicitEulerUpdate || stop.getSpeed() > 0) {
                    return 0;
                } else {
                    // ballistic:
                    return getSpeed() - getCarFollowModel().getMaxDecel();
                }
            }
        }
    } else {

#ifdef DEBUG_STOPS
        if (DEBUG_COND) {
            std::cout << SIMTIME << " vehicle '" << getID() << "' hasn't reached next stop." << std::endl;
        }
#endif
        //std::cout << SIMTIME <<  " myStopDist=" << myStopDist << " bGap=" << getBrakeGap(myLane->getVehicleMaxSpeed(this)) << "\n";
        if (stop.pars.onDemand && !stop.skipOnDemand && myStopDist <= getCarFollowModel().brakeGap(myLane->getVehicleMaxSpeed(this))) {
            MSNet* const net = MSNet::getInstance();
            const bool noExits = ((myPersonDevice == nullptr || !myPersonDevice->anyLeavingAtStop(stop))
                                  && (myContainerDevice == nullptr || !myContainerDevice->anyLeavingAtStop(stop)));
            const bool noEntries = ((!net->hasPersons() || !net->getPersonControl().hasAnyWaiting(stop.getEdge(), this))
                                    && (!net->hasContainers() || !net->getContainerControl().hasAnyWaiting(stop.getEdge(), this)));
            if (noExits && noEntries) {
                //std::cout << " skipOnDemand\n";
                stop.skipOnDemand = true;
            }
        }
        // is the next stop on the current lane?
        if (stop.edge == myCurrEdge) {
            // get the stopping position
            bool useStoppingPlace = stop.busstop != nullptr || stop.containerstop != nullptr || stop.parkingarea != nullptr;
            bool fitsOnStoppingPlace = true;
            if (!stop.skipOnDemand) {  // no need to check available space if we skip it anyway
                if (stop.busstop != nullptr) {
                    fitsOnStoppingPlace &= stop.busstop->fits(myState.myPos, *this);
                }
                if (stop.containerstop != nullptr) {
                    fitsOnStoppingPlace &= stop.containerstop->fits(myState.myPos, *this);
                }
                // if the stop is a parking area we check if there is a free position on the area
                if (stop.parkingarea != nullptr) {
                    fitsOnStoppingPlace &= myState.myPos > stop.parkingarea->getBeginLanePosition();
                    if (stop.parkingarea->getOccupancy() >= stop.parkingarea->getCapacity()) {
                        fitsOnStoppingPlace = false;
                        // trigger potential parkingZoneReroute
                        MSParkingArea* oldParkingArea = stop.parkingarea;
                        for (MSMoveReminder* rem : myLane->getMoveReminders()) {
                            if (rem->isParkingRerouter()) {
                                rem->notifyEnter(*this, MSMoveReminder::NOTIFICATION_PARKING_REROUTE, myLane);
                            }
                        }
                        if (myStops.empty() || myStops.front().parkingarea != oldParkingArea) {
                            // rerouted, keep driving
                            return currentVelocity;
                        }
                    } else if (stop.parkingarea->getOccupancyIncludingBlocked() >= stop.parkingarea->getCapacity()) {
                        fitsOnStoppingPlace = false;
                    } else if (stop.parkingarea->parkOnRoad() && stop.parkingarea->getLotIndex(this) < 0) {
                        fitsOnStoppingPlace = false;
                    }
                }
            }
            const double targetPos = myState.myPos + myStopDist + (stop.getSpeed() > 0 ? (stop.pars.startPos - stop.pars.endPos) : 0);
            const double reachedThreshold = (useStoppingPlace ? targetPos - STOPPING_PLACE_OFFSET : stop.getReachedThreshold()) - NUMERICAL_EPS;
#ifdef DEBUG_STOPS
            if (DEBUG_COND) {
                std::cout <<  "   pos=" << myState.pos() << " speed=" << currentVelocity << " targetPos=" << targetPos << " fits=" << fitsOnStoppingPlace
                          << " reachedThresh=" << reachedThreshold
                          << " myLane=" << Named::getIDSecure(myLane)
                          << " stopLane=" << Named::getIDSecure(stop.lane)
                          << "\n";
            }
#endif
            if (myState.pos() >= reachedThreshold && fitsOnStoppingPlace && currentVelocity <= stop.getSpeed() + SUMO_const_haltingSpeed && myLane == stop.lane
                    && (!MSGlobals::gModelParkingManoeuver || myManoeuvre.entryManoeuvreIsComplete(this))) {
                // ok, we may stop (have reached the stop)  and either we are not modelling maneuvering or have completed entry
                stop.reached = true;
                if (!stop.startedFromState) {
                    stop.pars.started = time;
                }
#ifdef DEBUG_STOPS
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " vehicle '" << getID() << "' reached next stop." << std::endl;
                }
#endif
                if (MSStopOut::active()) {
                    MSStopOut::getInstance()->stopStarted(this, getPersonNumber(), getContainerNumber(), time);
                }
                myLane->getEdge().addWaiting(this);
                MSNet::getInstance()->informVehicleStateListener(this, MSNet::VehicleState::STARTING_STOP);
                MSNet::getInstance()->getVehicleControl().registerStopStarted();
                // compute stopping time
                stop.duration = stop.getMinDuration(time);
                stop.endBoarding = stop.pars.extension >= 0 ? time + stop.duration + stop.pars.extension : SUMOTime_MAX;
                MSDevice_Taxi* taxiDevice = static_cast<MSDevice_Taxi*>(getDevice(typeid(MSDevice_Taxi)));
                if (taxiDevice != nullptr && stop.pars.extension >= 0) {
                    // earliestPickupTime is set with waitUntil
                    stop.endBoarding = MAX2(time, stop.pars.waitUntil) + stop.pars.extension;
                }
                if (stop.getSpeed() > 0) {
                    // ignore duration parameter in waypoint mode unless 'until' or 'ended' are set
                    if (stop.getUntil() > time) {
                        stop.duration = stop.getUntil() - time;
                    } else {
                        stop.duration = 0;
                    }
                }
                if (stop.busstop != nullptr) {
                    // let the bus stop know the vehicle
                    stop.busstop->enter(this, stop.pars.parking == ParkingType::OFFROAD);
                }
                if (stop.containerstop != nullptr) {
                    // let the container stop know the vehicle
                    stop.containerstop->enter(this, stop.pars.parking == ParkingType::OFFROAD);
                }
                if (stop.parkingarea != nullptr && stop.getSpeed() <= 0) {
                    // let the parking area know the vehicle
                    stop.parkingarea->enter(this);
                }
                if (stop.chargingStation != nullptr) {
                    // let the container stop know the vehicle
                    stop.chargingStation->enter(this, stop.pars.parking == ParkingType::OFFROAD);
                }

                if (stop.pars.tripId != "") {
                    ((SUMOVehicleParameter&)getParameter()).setParameter("tripId", stop.pars.tripId);
                }
                if (stop.pars.line != "") {
                    ((SUMOVehicleParameter&)getParameter()).line = stop.pars.line;
                }
                if (stop.pars.split != "") {
                    // split the train
                    MSVehicle* splitVeh = dynamic_cast<MSVehicle*>(MSNet::getInstance()->getVehicleControl().getVehicle(stop.pars.split));
                    if (splitVeh == nullptr) {
                        WRITE_WARNINGF(TL("Vehicle '%' to split from vehicle '%' is not known. time=%."), stop.pars.split, getID(), SIMTIME)
                    } else {
                        MSNet::getInstance()->getInsertionControl().add(splitVeh);
                        splitVeh->getRoute().getEdges()[0]->removeWaiting(splitVeh);
                        MSNet::getInstance()->getVehicleControl().unregisterOneWaiting();
                        const double newLength = MAX2(myType->getLength() - splitVeh->getVehicleType().getLength(),
                                                      myType->getParameter().locomotiveLength);
                        getSingularType().setLength(newLength);
                        // handle transportables that want to continue in the split part
                        if (myPersonDevice != nullptr) {
                            myPersonDevice->transferAtSplitOrJoin(splitVeh);
                        }
                        if (myContainerDevice != nullptr) {
                            myContainerDevice->transferAtSplitOrJoin(splitVeh);
                        }
                        if (splitVeh->getParameter().departPosProcedure == DepartPosDefinition::SPLIT_FRONT) {
                            const double backShift = splitVeh->getLength() + getVehicleType().getMinGap();
                            myState.myPos -= backShift;
                            myState.myBackPos -= backShift;
                        }
                    }
                }

                boardTransportables(stop);
                if (stop.pars.posLat != INVALID_DOUBLE) {
                    myState.myPosLat = stop.pars.posLat;
                }
            }
        }
    }
    return currentVelocity;
}


void
MSVehicle::boardTransportables(MSStop& stop) {
    if (stop.skipOnDemand) {
        return;
    }
    // we have reached the stop
    // any waiting persons may board now
    const SUMOTime time = MSNet::getInstance()->getCurrentTimeStep();
    MSNet* const net = MSNet::getInstance();
    const bool boarded = (time <= stop.endBoarding
                          && net->hasPersons()
                          && net->getPersonControl().loadAnyWaiting(&myLane->getEdge(), this, stop.timeToBoardNextPerson, stop.duration)
                          && stop.numExpectedPerson == 0);
    // load containers
    const bool loaded = (time <= stop.endBoarding
                         && net->hasContainers()
                         && net->getContainerControl().loadAnyWaiting(&myLane->getEdge(), this, stop.timeToLoadNextContainer, stop.duration)
                         && stop.numExpectedContainer == 0);

    bool unregister = false;
    if (time > stop.endBoarding) {
        // for taxi: cancel customers
        MSDevice_Taxi* taxiDevice = static_cast<MSDevice_Taxi*>(getDevice(typeid(MSDevice_Taxi)));
        if (taxiDevice != nullptr) {
            taxiDevice->cancelCurrentCustomers();
        }

        stop.triggered = false;
        stop.containerTriggered = false;
        if (myAmRegisteredAsWaiting) {
            unregister = true;
            myAmRegisteredAsWaiting = false;
        }
    }
    if (boarded) {
        // the triggering condition has been fulfilled. Maybe we want to wait a bit longer for additional riders (car pooling)
        if (myAmRegisteredAsWaiting) {
            unregister = true;
        }
        stop.triggered = false;
        myAmRegisteredAsWaiting = false;
    }
    if (loaded) {
        // the triggering condition has been fulfilled
        if (myAmRegisteredAsWaiting) {
            unregister = true;
        }
        stop.containerTriggered = false;
        myAmRegisteredAsWaiting = false;
    }

    if (unregister) {
        MSNet::getInstance()->getVehicleControl().unregisterOneWaiting();
#ifdef DEBUG_STOPS
        if (DEBUG_COND) {
            std::cout << SIMTIME << " vehicle '" << getID() << "' unregisters as waiting for transportable." << std::endl;
        }
#endif
    }
}

bool
MSVehicle::joinTrainPart(MSVehicle* veh) {
    // check if veh is close enough to be joined to the rear of this vehicle
    MSLane* backLane = myFurtherLanes.size() == 0 ? myLane : myFurtherLanes.back();
    double gap = getBackPositionOnLane() - veh->getPositionOnLane();
    if (isStopped() && myStops.begin()->duration <= DELTA_T && myStops.begin()->joinTriggered && backLane == veh->getLane()
            && gap >= 0 && gap <= getVehicleType().getMinGap() + 1) {
        const double newLength = myType->getLength() + veh->getVehicleType().getLength();
        getSingularType().setLength(newLength);
        myStops.begin()->joinTriggered = false;
        if (myAmRegisteredAsWaiting) {
            MSNet::getInstance()->getVehicleControl().unregisterOneWaiting();
            myAmRegisteredAsWaiting = false;
        }
        return true;
    } else {
        return false;
    }
}


bool
MSVehicle::joinTrainPartFront(MSVehicle* veh) {
    // check if veh is close enough to be joined to the front of this vehicle
    MSLane* backLane = veh->myFurtherLanes.size() == 0 ? veh->myLane : veh->myFurtherLanes.back();
    double gap = veh->getBackPositionOnLane(backLane) - getPositionOnLane();
    if (isStopped() && myStops.begin()->duration <= DELTA_T && myStops.begin()->joinTriggered && backLane == getLane()
            && gap >= 0 && gap <= getVehicleType().getMinGap() + 1) {
        double skippedLaneLengths = 0;
        if (veh->myFurtherLanes.size() > 0) {
            skippedLaneLengths += getLane()->getLength();
            // this vehicle must be moved to the lane of veh
            // ensure that lane and furtherLanes of veh match our route
            int routeIndex = getRoutePosition();
            if (myLane->isInternal()) {
                routeIndex++;
            }
            for (int i = (int)veh->myFurtherLanes.size() - 1; i >= 0; i--) {
                MSEdge* edge = &veh->myFurtherLanes[i]->getEdge();
                if (edge->isInternal()) {
                    continue;
                }
                if (!edge->isInternal() && edge != myRoute->getEdges()[routeIndex]) {
                    std::string warn = TL("Cannot join vehicle '%' to vehicle '%' due to incompatible routes. time=%.");
                    WRITE_WARNINGF(warn, veh->getID(), getID(), time2string(SIMSTEP));
                    return false;
                }
                routeIndex++;
            }
            if (veh->getCurrentEdge()->getNormalSuccessor() != myRoute->getEdges()[routeIndex]) {
                std::string warn = TL("Cannot join vehicle '%' to vehicle '%' due to incompatible routes. time=%.");
                WRITE_WARNINGF(warn, veh->getID(), getID(), time2string(SIMSTEP));
                return false;
            }
            for (int i = (int)veh->myFurtherLanes.size() - 2; i >= 0; i--) {
                skippedLaneLengths += veh->myFurtherLanes[i]->getLength();
            }
        }

        const double newLength = myType->getLength() + veh->getVehicleType().getLength();
        getSingularType().setLength(newLength);
        // lane will be advanced just as for regular movement
        myState.myPos = skippedLaneLengths + veh->getPositionOnLane();
        myStops.begin()->joinTriggered = false;
        if (myAmRegisteredAsWaiting) {
            MSNet::getInstance()->getVehicleControl().unregisterOneWaiting();
            myAmRegisteredAsWaiting = false;
        }
        return true;
    } else {
        return false;
    }
}

double
MSVehicle::getBrakeGap(bool delayed) const {
    return getCarFollowModel().brakeGap(getSpeed(), getCarFollowModel().getMaxDecel(), delayed ? getCarFollowModel().getHeadwayTime() : 0);
}


bool
MSVehicle::checkActionStep(const SUMOTime t) {
    myActionStep = isActionStep(t);
    if (myActionStep) {
        myLastActionTime = t;
    }
    return myActionStep;
}


void
MSVehicle::resetActionOffset(const SUMOTime timeUntilNextAction) {
    myLastActionTime = MSNet::getInstance()->getCurrentTimeStep() + timeUntilNextAction;
}


void
MSVehicle::updateActionOffset(const SUMOTime oldActionStepLength, const SUMOTime newActionStepLength) {
    SUMOTime now = MSNet::getInstance()->getCurrentTimeStep();
    SUMOTime timeSinceLastAction = now - myLastActionTime;
    if (timeSinceLastAction == 0) {
        // Action was scheduled now, may be delayed be new action step length
        timeSinceLastAction = oldActionStepLength;
    }
    if (timeSinceLastAction >= newActionStepLength) {
        // Action point required in this step
        myLastActionTime = now;
    } else {
        SUMOTime timeUntilNextAction = newActionStepLength - timeSinceLastAction;
        resetActionOffset(timeUntilNextAction);
    }
}



void
MSVehicle::planMove(const SUMOTime t, const MSLeaderInfo& ahead, const double lengthsInFront) {
#ifdef DEBUG_PLAN_MOVE
    if (DEBUG_COND) {
        std::cout
                << "\nPLAN_MOVE\n"
                << SIMTIME
                << std::setprecision(gPrecision)
                << " veh=" << getID()
                << " lane=" << myLane->getID()
                << " pos=" << getPositionOnLane()
                << " posLat=" << getLateralPositionOnLane()
                << " speed=" << getSpeed()
                << "\n";
    }
#endif
    // Update the driver state
    if (hasDriverState()) {
        myDriverState->update();
        setActionStepLength(myDriverState->getDriverState()->getActionStepLength(), false);
    }

    if (!checkActionStep(t)) {
#ifdef DEBUG_ACTIONSTEPS
        if (DEBUG_COND) {
            std::cout << STEPS2TIME(t) << " vehicle '" << getID() << "' skips action." << std::endl;
        }
#endif
        // During non-action passed drive items still need to be removed
        // @todo rather work with updating myCurrentDriveItem (refs #3714)
        removePassedDriveItems();
        return;
    } else {
#ifdef DEBUG_ACTIONSTEPS
        if (DEBUG_COND) {
            std::cout << STEPS2TIME(t) << " vehicle = '" << getID() << "' takes action." << std::endl;
        }
#endif
        myLFLinkLanesPrev = myLFLinkLanes;
        if (myInfluencer != nullptr) {
            myInfluencer->updateRemoteControlRoute(this);
        }
        planMoveInternal(t, ahead, myLFLinkLanes, myStopDist, myNextTurn);
#ifdef DEBUG_PLAN_MOVE
        if (DEBUG_COND) {
            DriveItemVector::iterator i;
            for (i = myLFLinkLanes.begin(); i != myLFLinkLanes.end(); ++i) {
                std::cout
                        << " vPass=" << (*i).myVLinkPass
                        << " vWait=" << (*i).myVLinkWait
                        << " linkLane=" << ((*i).myLink == 0 ? "NULL" : (*i).myLink->getViaLaneOrLane()->getID())
                        << " request=" << (*i).mySetRequest
                        << "\n";
            }
        }
#endif
        checkRewindLinkLanes(lengthsInFront, myLFLinkLanes);
        myNextDriveItem = myLFLinkLanes.begin();
        // ideally would only do this with the call inside planMoveInternal - but that needs a const method
        //   so this is a kludge here - nuisance as it adds an extra check in a busy loop
        if (MSGlobals::gModelParkingManoeuver) {
            if (getManoeuvreType() == MSVehicle::MANOEUVRE_EXIT && manoeuvreIsComplete()) {
                setManoeuvreType(MSVehicle::MANOEUVRE_NONE);
            }
        }
    }
    myLaneChangeModel->resetChanged();
}


bool
MSVehicle::brakeForOverlap(const MSLink* link, const MSLane* lane) const {
    // @review needed
    //const double futurePosLat = getLateralPositionOnLane() + link->getLateralShift();
    //const double overlap = getLateralOverlap(futurePosLat, link->getViaLaneOrLane());
    //const double edgeWidth = link->getViaLaneOrLane()->getEdge().getWidth();
    const double futurePosLat = getLateralPositionOnLane() + (
                                    lane != myLane && lane->isInternal() ? lane->getIncomingLanes()[0].viaLink->getLateralShift() : 0);
    const double overlap = getLateralOverlap(futurePosLat, lane);
    const double edgeWidth = lane->getEdge().getWidth();
    const bool result = (overlap > POSITION_EPS
                         // do not get stuck on narrow edges
                         && getVehicleType().getWidth() <= edgeWidth
                         && link->getViaLane() == nullptr
                         // this is the exit link of a junction. The normal edge should support the shadow
                         && ((myLaneChangeModel->getShadowLane(link->getLane()) == nullptr)
                             // the internal lane after an internal junction has no parallel lane. make sure there is no shadow before continuing
                             || (lane->getEdge().isInternal() && lane->getIncomingLanes()[0].lane->getEdge().isInternal()))
                         // ignore situations where the shadow lane is part of a double-connection with the current lane
                         && (myLaneChangeModel->getShadowLane() == nullptr
                             || myLaneChangeModel->getShadowLane()->getLinkCont().size() == 0
                             || myLaneChangeModel->getShadowLane()->getLinkCont().front()->getLane() != link->getLane()));

#ifdef DEBUG_PLAN_MOVE
    if (DEBUG_COND) {
        std::cout << SIMTIME << " veh=" << getID() << " link=" << link->getDescription() << " lane=" << lane->getID()
                  << " shift=" << link->getLateralShift()
                  << " fpLat=" << futurePosLat << " overlap=" << overlap << " w=" << getVehicleType().getWidth() << " result=" << result << "\n";
    }
#endif
    return result;
}



void
MSVehicle::planMoveInternal(const SUMOTime t, MSLeaderInfo ahead, DriveItemVector& lfLinks, double& newStopDist, std::pair<double, const MSLink*>& nextTurn) const {
    lfLinks.clear();
    newStopDist = std::numeric_limits<double>::max();
    //
    const MSCFModel& cfModel = getCarFollowModel();
    const double vehicleLength = getVehicleType().getLength();
    const double maxV = cfModel.maxNextSpeed(myState.mySpeed, this);
    const bool opposite = myLaneChangeModel->isOpposite();
    double laneMaxV = myLane->getVehicleMaxSpeed(this);
    const double vMinComfortable = cfModel.minNextSpeed(getSpeed(), this);
    double lateralShift = 0;
    if (isRailway((SVCPermissions)getVehicleType().getVehicleClass())) {
        // speed limits must hold for the whole length of the train
        for (MSLane* l : myFurtherLanes) {
            laneMaxV = MIN2(laneMaxV, l->getVehicleMaxSpeed(this));
#ifdef DEBUG_PLAN_MOVE
            if (DEBUG_COND) {
                std::cout << "   laneMaxV=" << laneMaxV << " lane=" << l->getID() << "\n";
            }
#endif
        }
    }
    //  speed limits are not emergencies (e.g. when the limit changes suddenly due to TraCI or a variableSpeedSignal)
    laneMaxV = MAX2(laneMaxV, vMinComfortable);
    if (myInfluencer && !myInfluencer->considerSafeVelocity()) {
        laneMaxV = std::numeric_limits<double>::max();
    }
    // v is the initial maximum velocity of this vehicle in this step
    double v = cfModel.maximumLaneSpeedCF(this, maxV, laneMaxV);
    // if we are modelling parking then we dawdle until the manoeuvre is complete - by setting a very low max speed
    //   in practice this only applies to exit manoeuvre because entry manoeuvre just delays setting stop.reached - when the vehicle is virtually stopped
    if (MSGlobals::gModelParkingManoeuver && !manoeuvreIsComplete()) {
        v = NUMERICAL_EPS_SPEED;
    }

    if (myInfluencer != nullptr) {
        const double vMin = MAX2(0., cfModel.minNextSpeed(myState.mySpeed, this));
#ifdef DEBUG_TRACI
        if (DEBUG_COND) {
            std::cout << SIMTIME << " veh=" << getID() << " speedBeforeTraci=" << v;
        }
#endif
        v = myInfluencer->influenceSpeed(t, v, v, vMin, maxV);
#ifdef DEBUG_TRACI
        if (DEBUG_COND) {
            std::cout << " influencedSpeed=" << v;
        }
#endif
        v = myInfluencer->gapControlSpeed(t, this, v, v, vMin, maxV);
#ifdef DEBUG_TRACI
        if (DEBUG_COND) {
            std::cout << " gapControlSpeed=" << v << "\n";
        }
#endif
    }
    // all links within dist are taken into account (potentially)
    const double dist = SPEED2DIST(maxV) + cfModel.brakeGap(maxV);

    const std::vector<MSLane*>& bestLaneConts = getBestLanesContinuation();
#ifdef DEBUG_PLAN_MOVE
    if (DEBUG_COND) {
        std::cout << "   dist=" << dist << " bestLaneConts=" << toString(bestLaneConts)
                  << "\n   maxV=" << maxV << " laneMaxV=" << laneMaxV << " v=" << v << "\n";
    }
#endif
    assert(bestLaneConts.size() > 0);
    bool hadNonInternal = false;
    // the distance already "seen"; in the following always up to the end of the current "lane"
    double seen = opposite ? myState.myPos : myLane->getLength() - myState.myPos;
    nextTurn.first = seen;
    nextTurn.second = nullptr;
    bool encounteredTurn = (MSGlobals::gLateralResolution <= 0); // next turn is only needed for sublane
    double seenNonInternal = 0;
    double seenInternal = myLane->isInternal() ? seen : 0;
    double vLinkPass = MIN2(cfModel.estimateSpeedAfterDistance(seen, v, cfModel.getMaxAccel()), laneMaxV); // upper bound
    int view = 0;
    DriveProcessItem* lastLink = nullptr;
    bool slowedDownForMinor = false; // whether the vehicle already had to slow down on approach to a minor link
    double mustSeeBeforeReversal = 0;
    // iterator over subsequent lanes and fill lfLinks until stopping distance or stopped
    const MSLane* lane = opposite ? myLane->getParallelOpposite() : myLane;
    assert(lane != 0);
    const MSLane* leaderLane = myLane;
    bool foundRailSignal = !isRailway(getVClass());
    bool planningToStop = false;
#ifdef PARALLEL_STOPWATCH
    myLane->getStopWatch()[0].start();
#endif

    // optionally slow down to match arrival time
    const double sfp = getVehicleType().getParameter().speedFactorPremature;
    if (v > vMinComfortable && hasStops() && myStops.front().pars.arrival >= 0 && sfp > 0
            && v > myLane->getSpeedLimit() * sfp
            && !myStops.front().reached) {
        const double vSlowDown = slowDownForSchedule(vMinComfortable);
        v = MIN2(v, vSlowDown);
    }
    auto stopIt = myStops.begin();
    while (true) {
        // check leader on lane
        //  leader is given for the first edge only
        if (opposite &&
                (leaderLane->getVehicleNumberWithPartials() > 1
                 || (leaderLane != myLane && leaderLane->getVehicleNumber() > 0))) {
            ahead.clear();
            // find opposite-driving leader that must be respected on the currently looked at lane
            // (only looking at one lane at a time)
            const double backOffset = leaderLane == myLane ? getPositionOnLane() : leaderLane->getLength();
            const double gapOffset = leaderLane == myLane ? 0 : seen - leaderLane->getLength();
            const MSLeaderDistanceInfo cands = leaderLane->getFollowersOnConsecutive(this, backOffset, true, backOffset, MSLane::MinorLinkMode::FOLLOW_NEVER);
            MSLeaderDistanceInfo oppositeLeaders(leaderLane->getWidth(), this, 0.);
            const double minTimeToLeaveLane = MSGlobals::gSublane ? MAX2(TS, (0.5 *  myLane->getWidth() - getLateralPositionOnLane()) / getVehicleType().getMaxSpeedLat()) : TS;
            for (int i = 0; i < cands.numSublanes(); i++) {
                CLeaderDist cand = cands[i];
                if (cand.first != 0) {
                    if ((cand.first->myLaneChangeModel->isOpposite() && cand.first->getLaneChangeModel().getShadowLane() != leaderLane)
                            || (!cand.first->myLaneChangeModel->isOpposite() && cand.first->getLaneChangeModel().getShadowLane() == leaderLane)) {
                        // respect leaders that also drive in the opposite direction (fully or with some overlap)
                        oppositeLeaders.addLeader(cand.first, cand.second + gapOffset - getVehicleType().getMinGap() + cand.first->getVehicleType().getMinGap() - cand.first->getVehicleType().getLength());
                    } else {
                        // avoid frontal collision
                        const bool assumeStopped = cand.first->isStopped() || cand.first->getWaitingSeconds() > 1;
                        const double predMaxDist = cand.first->getSpeed() + (assumeStopped ? 0 : cand.first->getCarFollowModel().getMaxAccel()) * minTimeToLeaveLane;
                        if (cand.second >= 0 && (cand.second - v * minTimeToLeaveLane - predMaxDist < 0 || assumeStopped)) {
                            oppositeLeaders.addLeader(cand.first, cand.second + gapOffset - predMaxDist - getVehicleType().getMinGap());
                        }
                    }
                }
            }
#ifdef DEBUG_PLAN_MOVE
            if (DEBUG_COND) {
                std::cout <<  " leaderLane=" << leaderLane->getID() << " gapOffset=" << gapOffset << " minTimeToLeaveLane=" << minTimeToLeaveLane
                          << " cands=" << cands.toString() << " oppositeLeaders=" <<  oppositeLeaders.toString() << "\n";
            }
#endif
            adaptToLeaderDistance(oppositeLeaders, 0, seen, lastLink, v, vLinkPass);
        } else {
            if (MSGlobals::gLateralResolution > 0 && myLaneChangeModel->getShadowLane() == nullptr) {
                const double rightOL = getRightSideOnLane(lane) + lateralShift;
                const double leftOL = getLeftSideOnLane(lane) + lateralShift;
                const bool outsideLeft = leftOL > lane->getWidth();
#ifdef DEBUG_PLAN_MOVE
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " veh=" << getID() << " lane=" << lane->getID() << " rightOL=" << rightOL << " leftOL=" << leftOL << "\n";
                }
#endif
                if (rightOL < 0 || outsideLeft) {
                    MSLeaderInfo outsideLeaders(lane->getWidth());
                    // if ego is driving outside lane bounds we must consider
                    // potential leaders that are also outside bounds
                    int sublaneOffset = 0;
                    if (outsideLeft) {
                        sublaneOffset = MIN2(-1, -(int)ceil((leftOL - lane->getWidth()) / MSGlobals::gLateralResolution));
                    } else {
                        sublaneOffset = MAX2(1, (int)ceil(-rightOL / MSGlobals::gLateralResolution));
                    }
                    outsideLeaders.setSublaneOffset(sublaneOffset);
#ifdef DEBUG_PLAN_MOVE
                    if (DEBUG_COND) {
                        std::cout << SIMTIME << " veh=" << getID() << " lane=" << lane->getID() << " sublaneOffset=" << sublaneOffset << " outsideLeft=" << outsideLeft << "\n";
                    }
#endif
                    for (const MSVehicle* cand : lane->getVehiclesSecure()) {
                        if ((lane != myLane || cand->getPositionOnLane() > getPositionOnLane())
                                && ((!outsideLeft && cand->getLeftSideOnEdge() < 0)
                                    || (outsideLeft && cand->getLeftSideOnEdge() > lane->getEdge().getWidth()))) {
                            outsideLeaders.addLeader(cand, true);
#ifdef DEBUG_PLAN_MOVE
                            if (DEBUG_COND) {
                                std::cout << " outsideLeader=" << cand->getID() << " ahead=" << outsideLeaders.toString() << "\n";
                            }
#endif
                        }
                    }
                    lane->releaseVehicles();
                    if (outsideLeaders.hasVehicles()) {
                        adaptToLeaders(outsideLeaders, lateralShift, seen, lastLink, leaderLane, v, vLinkPass);
                    }
                }
            }
            adaptToLeaders(ahead, lateralShift, seen, lastLink, leaderLane, v, vLinkPass);
        }
        if (lastLink != nullptr) {
            lastLink->myVLinkWait = MIN2(lastLink->myVLinkWait, v);
        }
#ifdef DEBUG_PLAN_MOVE
        if (DEBUG_COND) {
            std::cout << "\nv = " << v << "\n";

        }
#endif
        // XXX efficiently adapt to shadow leaders using neighAhead by iteration over the whole edge in parallel (lanechanger-style)
        if (myLaneChangeModel->getShadowLane() != nullptr) {
            // also slow down for leaders on the shadowLane relative to the current lane
            const MSLane* shadowLane = myLaneChangeModel->getShadowLane(leaderLane);
            if (shadowLane != nullptr
                    && (MSGlobals::gLateralResolution > 0 || getLateralOverlap() > POSITION_EPS
                        // continous lane change cannot be stopped so we must adapt to the leader on the target lane
                        || myLaneChangeModel->getLaneChangeCompletion() < 0.5)) {
                if ((&shadowLane->getEdge() == &leaderLane->getEdge() || myLaneChangeModel->isOpposite())) {
                    double latOffset = getLane()->getRightSideOnEdge() - myLaneChangeModel->getShadowLane()->getRightSideOnEdge();
                    if (myLaneChangeModel->isOpposite()) {
                        // ego posLat is added when retrieving sublanes but it
                        // should be negated (subtract twice to compensate)
                        latOffset = ((myLane->getWidth() + shadowLane->getWidth()) * 0.5
                                     - 2 * getLateralPositionOnLane());

                    }
                    MSLeaderInfo shadowLeaders = shadowLane->getLastVehicleInformation(this, latOffset, lane->getLength() - seen);
#ifdef DEBUG_PLAN_MOVE
                    if (DEBUG_COND && myLaneChangeModel->isOpposite()) {
                        std::cout << SIMTIME << " opposite veh=" << getID() << " shadowLane=" << shadowLane->getID() << " latOffset=" << latOffset << " shadowLeaders=" << shadowLeaders.toString() << "\n";
                    }
#endif
                    if (myLaneChangeModel->isOpposite()) {
                        // ignore oncoming vehicles on the shadow lane
                        shadowLeaders.removeOpposite(shadowLane);
                    }
                    const double turningDifference = MAX2(0.0, leaderLane->getLength() - shadowLane->getLength());
                    adaptToLeaders(shadowLeaders, latOffset, seen - turningDifference, lastLink, shadowLane, v, vLinkPass);
                } else if (shadowLane == myLaneChangeModel->getShadowLane() && leaderLane == myLane) {
                    // check for leader vehicles driving in the opposite direction on the opposite-direction shadow lane
                    // (and thus in the same direction as ego)
                    MSLeaderDistanceInfo shadowLeaders = shadowLane->getFollowersOnConsecutive(this, myLane->getOppositePos(getPositionOnLane()), true);
                    const double latOffset = 0;
#ifdef DEBUG_PLAN_MOVE
                    if (DEBUG_COND) {
                        std::cout << SIMTIME << " opposite shadows veh=" << getID() << " shadowLane=" << shadowLane->getID()
                                  << " latOffset=" << latOffset << " shadowLeaders=" << shadowLeaders.toString() << "\n";
                    }
#endif
                    shadowLeaders.fixOppositeGaps(true);
#ifdef DEBUG_PLAN_MOVE
                    if (DEBUG_COND) {
                        std::cout << "   shadowLeadersFixed=" << shadowLeaders.toString() << "\n";
                    }
#endif
                    adaptToLeaderDistance(shadowLeaders, latOffset, seen, lastLink, v, vLinkPass);
                }
            }
        }
        // adapt to pedestrians on the same lane
        if (lane->getEdge().getPersons().size() > 0 && lane->hasPedestrians()) {
            const double relativePos = lane->getLength() - seen;
#ifdef DEBUG_PLAN_MOVE
            if (DEBUG_COND) {
                std::cout << SIMTIME << " adapt to pedestrians on lane=" << lane->getID() << " relPos=" << relativePos << "\n";
            }
#endif
            const double stopTime = MAX2(1.0, ceil(getSpeed() / cfModel.getMaxDecel()));
            PersonDist leader = lane->nextBlocking(relativePos,
                                                   getRightSideOnLane(lane), getRightSideOnLane(lane) + getVehicleType().getWidth(), stopTime);
            if (leader.first != 0) {
                const double stopSpeed = cfModel.stopSpeed(this, getSpeed(), leader.second - getVehicleType().getMinGap());
                v = MIN2(v, stopSpeed);
#ifdef DEBUG_PLAN_MOVE
                if (DEBUG_COND) {
                    std::cout << SIMTIME << "    pedLeader=" << leader.first->getID() << " dist=" << leader.second << " v=" << v << "\n";
                }
#endif
            }
        }
        if (lane->getBidiLane() != nullptr) {
            // adapt to pedestrians on the bidi lane
            const MSLane* bidiLane = lane->getBidiLane();
            if (bidiLane->getEdge().getPersons().size() > 0 && bidiLane->hasPedestrians()) {
                const double relativePos = seen;
#ifdef DEBUG_PLAN_MOVE
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " adapt to pedestrians on lane=" << lane->getID() << " relPos=" << relativePos << "\n";
                }
#endif
                const double stopTime = ceil(getSpeed() / cfModel.getMaxDecel());
                const double leftSideOnLane = bidiLane->getWidth() - getRightSideOnLane(lane);
                PersonDist leader = bidiLane->nextBlocking(relativePos,
                                    leftSideOnLane - getVehicleType().getWidth(), leftSideOnLane, stopTime, true);
                if (leader.first != 0) {
                    const double stopSpeed = cfModel.stopSpeed(this, getSpeed(), leader.second - getVehicleType().getMinGap());
                    v = MIN2(v, stopSpeed);
#ifdef DEBUG_PLAN_MOVE
                    if (DEBUG_COND) {
                        std::cout << SIMTIME << "    pedLeader=" << leader.first->getID() << " dist=" << leader.second << " v=" << v << "\n";
                    }
#endif
                }
            }
        }

        // process all stops and waypoints on the current edge
        bool foundRealStop = false;
        while (stopIt != myStops.end()
                && ((&stopIt->lane->getEdge() == &lane->getEdge())
                    || (stopIt->isOpposite && stopIt->lane->getEdge().getOppositeEdge() == &lane->getEdge()))
                // ignore stops that occur later in a looped route
                && stopIt->edge == myCurrEdge + view) {
            double stopDist = std::numeric_limits<double>::max();
            const MSStop& stop = *stopIt;
            const bool isFirstStop = stopIt == myStops.begin();
            stopIt++;
            if (!stop.reached || (stop.getSpeed() > 0 && keepStopping())) {
                // we are approaching a stop on the edge; must not drive further
                bool isWaypoint = stop.getSpeed() > 0;
                double endPos = stop.getEndPos(*this) + NUMERICAL_EPS;
                if (stop.parkingarea != nullptr) {
                    // leave enough space so parking vehicles can exit
                    const double brakePos = getBrakeGap() + lane->getLength() - seen;
                    endPos = stop.parkingarea->getLastFreePosWithReservation(t, *this, brakePos);
                } else if (isWaypoint && !stop.reached) {
                    endPos = stop.pars.startPos;
                }
                stopDist = seen + endPos - lane->getLength();
#ifdef DEBUG_STOPS
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " veh=" << getID() <<  " stopDist=" << stopDist << " stopLane=" << stop.lane->getID() << " stopEndPos=" << endPos << "\n";
                }
#endif
                // regular stops are not emergencies
                double stopSpeed = laneMaxV;
                if (isWaypoint) {
                    bool waypointWithStop = false;
                    if (stop.getUntil() > t) {
                        // check if we have to slow down or even stop
                        SUMOTime time2end = 0;
                        if (stop.reached) {
                            time2end = TIME2STEPS((stop.pars.endPos - myState.myPos) / stop.getSpeed());
                        } else {
                            time2end = TIME2STEPS(
                                           // time to reach waypoint start
                                           stopDist / ((getSpeed() + stop.getSpeed()) / 2)
                                           // time to reach waypoint end
                                           + (stop.pars.endPos - stop.pars.startPos) / stop.getSpeed());
                        }
                        if (stop.getUntil() > t + time2end) {
                            // we need to stop
                            double distToEnd = stopDist;
                            if (!stop.reached) {
                                distToEnd += stop.pars.endPos - stop.pars.startPos;
                            }
                            stopSpeed = MAX2(cfModel.stopSpeed(this, getSpeed(), distToEnd), vMinComfortable);
                            waypointWithStop = true;
                        }
                    }
                    if (stop.reached) {
                        stopSpeed = MIN2(stop.getSpeed(), stopSpeed);
                        if (myState.myPos >= stop.pars.endPos && !waypointWithStop) {
                            stopDist = std::numeric_limits<double>::max();
                        }
                    } else {
                        stopSpeed = MIN2(MAX2(cfModel.freeSpeed(this, getSpeed(), stopDist, stop.getSpeed()), vMinComfortable), stopSpeed);
                        if (!stop.reached) {
                            stopDist += stop.pars.endPos - stop.pars.startPos;
                        }
                        if (lastLink != nullptr) {
                            lastLink->adaptLeaveSpeed(cfModel.freeSpeed(this, vLinkPass, endPos, stop.getSpeed(), false, MSCFModel::CalcReason::FUTURE));
                        }
                    }
                } else {
                    stopSpeed = MAX2(cfModel.stopSpeed(this, getSpeed(), stopDist), vMinComfortable);
                    if (lastLink != nullptr) {
                        lastLink->adaptLeaveSpeed(cfModel.stopSpeed(this, vLinkPass, endPos, MSCFModel::CalcReason::FUTURE));
                    }
                }
                v = MIN2(v, stopSpeed);
                if (lane->isInternal()) {
                    std::vector<MSLink*>::const_iterator exitLink = MSLane::succLinkSec(*this, view + 1, *lane, bestLaneConts);
                    assert(!lane->isLinkEnd(exitLink));
                    bool dummySetRequest;
                    double dummyVLinkWait;
                    checkLinkLeaderCurrentAndParallel(*exitLink, lane, seen, lastLink, v, vLinkPass, dummyVLinkWait, dummySetRequest);
                }

#ifdef DEBUG_PLAN_MOVE
                if (DEBUG_COND) {
                    std::cout << "\n" << SIMTIME << " next stop: distance = " << stopDist << " requires stopSpeed = " << stopSpeed << "\n";

                }
#endif
                if (isFirstStop) {
                    newStopDist = stopDist;
                    // if the vehicle is going to stop we don't need to look further
                    // (except for trains that make use of further link-approach registration for safety purposes)
                    if (!isWaypoint) {
                        planningToStop = true;
                        if (!isRailway(getVClass())) {
                            lfLinks.emplace_back(v, stopDist);
                            foundRealStop = true;
                            break;
                        }
                    }
                }
            }
        }
        if (foundRealStop) {
            break;
        }

        // move to next lane
        //  get the next link used
        std::vector<MSLink*>::const_iterator link = MSLane::succLinkSec(*this, view + 1, *lane, bestLaneConts);

        // Check whether this is a turn (to save info about the next upcoming turn)
        if (!encounteredTurn) {
            if (!lane->isLinkEnd(link) && lane->getLinkCont().size() > 1) {
                LinkDirection linkDir = (*link)->getDirection();
                switch (linkDir) {
                    case LinkDirection::STRAIGHT:
                    case LinkDirection::NODIR:
                        break;
                    default:
                        nextTurn.first = seen;
                        nextTurn.second = *link;
                        encounteredTurn = true;
#ifdef DEBUG_NEXT_TURN
                        if (DEBUG_COND) {
                            std::cout << SIMTIME << " veh '" << getID() << "' nextTurn: " << toString(linkDir)
                                      << " at " << nextTurn.first << "m." << std::endl;
                        }
#endif
                }
            }
        }

        //  check whether the vehicle is on its final edge
        if (myCurrEdge + view + 1 == myRoute->end()
                || (myParameter->arrivalEdge >= 0 && getRoutePosition() + view == myParameter->arrivalEdge)) {
            const double arrivalSpeed = (myParameter->arrivalSpeedProcedure == ArrivalSpeedDefinition::GIVEN ?
                                         myParameter->arrivalSpeed : laneMaxV);
            // subtract the arrival speed from the remaining distance so we get one additional driving step with arrival speed
            // XXX: This does not work for ballistic update refs #2579
            const double distToArrival = seen + myArrivalPos - lane->getLength() - SPEED2DIST(arrivalSpeed);
            const double va = MAX2(NUMERICAL_EPS, cfModel.freeSpeed(this, getSpeed(), distToArrival, arrivalSpeed));
            v = MIN2(v, va);
            if (lastLink != nullptr) {
                lastLink->adaptLeaveSpeed(va);
            }
            lfLinks.push_back(DriveProcessItem(v, seen, lane->getEdge().isFringe() ? 1000 : 0));
            break;
        }
        // check whether the lane or the shadowLane is a dead end (allow some leeway on intersections)
        if (lane->isLinkEnd(link)
                || (MSGlobals::gSublane && brakeForOverlap(*link, lane))
                || (opposite && (*link)->getViaLaneOrLane()->getParallelOpposite() == nullptr
                    && !myLaneChangeModel->hasBlueLight())) {
            double va = cfModel.stopSpeed(this, getSpeed(), seen);
            if (lastLink != nullptr) {
                lastLink->adaptLeaveSpeed(va);
            }
            if (myLaneChangeModel->getCommittedSpeed() > 0) {
                v = MIN2(myLaneChangeModel->getCommittedSpeed(), v);
            } else {
                v = MIN2(va, v);
            }
#ifdef DEBUG_PLAN_MOVE
            if (DEBUG_COND) {
                std::cout << "   braking for link end lane=" << lane->getID() << " seen=" << seen
                          << " overlap=" << getLateralOverlap() << " va=" << va << " committed=" << myLaneChangeModel->getCommittedSpeed() << " v=" << v << "\n";

            }
#endif
            if (lane->isLinkEnd(link)) {
                lfLinks.emplace_back(v, seen);
                break;
            }
        }
        lateralShift += (*link)->getLateralShift();
        const bool yellowOrRed = (*link)->haveRed() || (*link)->haveYellow();
        // We distinguish 3 cases when determining the point at which a vehicle stops:
        // - links that require stopping: here the vehicle needs to stop close to the stop line
        //   to ensure it gets onto the junction in the next step. Otherwise the vehicle would 'forget'
        //   that it already stopped and need to stop again. This is necessary pending implementation of #999
        // - red/yellow light: here the vehicle 'knows' that it will have priority eventually and does not need to stop on a precise spot
        // - other types of minor links: the vehicle needs to stop as close to the junction as necessary
        //   to minimize the time window for passing the junction. If the
        //   vehicle 'decides' to accelerate and cannot enter the junction in
        //   the next step, new foes may appear and cause a collision (see #1096)
        // - major links: stopping point is irrelevant
        double laneStopOffset;
        const double majorStopOffset = MAX2(getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_STOPLINE_GAP, DIST_TO_STOPLINE_EXPECT_PRIORITY), lane->getVehicleStopOffset(this));
        // override low desired decel at yellow and red
        const double stopDecel = yellowOrRed && !isRailway(getVClass()) ? MAX2(MIN2(MSGlobals::gTLSYellowMinDecel, cfModel.getEmergencyDecel()), cfModel.getMaxDecel()) : cfModel.getMaxDecel();
        const double brakeDist = cfModel.brakeGap(myState.mySpeed, stopDecel, 0);
        const bool canBrakeBeforeLaneEnd = seen >= brakeDist;
        const bool canBrakeBeforeStopLine = seen - lane->getVehicleStopOffset(this) >= brakeDist;
        if (yellowOrRed) {
            // Wait at red traffic light with full distance if possible
            laneStopOffset = majorStopOffset;
        } else if ((*link)->havePriority()) {
            // On priority link, we should never stop below visibility distance
            laneStopOffset = MIN2((*link)->getFoeVisibilityDistance() - POSITION_EPS, majorStopOffset);
        } else {
            // On minor link, we should likewise never stop below visibility distance
            const double minorStopOffset = MAX2(lane->getVehicleStopOffset(this),
                                                getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_STOPLINE_CROSSING_GAP, MSPModel::SAFETY_GAP) - (*link)->getDistToFoePedCrossing());
            laneStopOffset = MIN2((*link)->getFoeVisibilityDistance() - POSITION_EPS, minorStopOffset);
        }
#ifdef DEBUG_PLAN_MOVE
        if (DEBUG_COND) {
            std::cout << SIMTIME << " veh=" << getID() << " desired stopOffset on lane '" << lane->getID() << "' is " << laneStopOffset << "\n";
        }
#endif
        if (canBrakeBeforeLaneEnd) {
            // avoid emergency braking if possible
            laneStopOffset = MIN2(laneStopOffset, seen - brakeDist);
        }
        laneStopOffset = MAX2(POSITION_EPS, laneStopOffset);
        double stopDist = MAX2(0., seen - laneStopOffset);
        if (yellowOrRed && getDevice(typeid(MSDevice_GLOSA)) != nullptr
                && static_cast<MSDevice_GLOSA*>(getDevice(typeid(MSDevice_GLOSA)))->getOverrideSafety()
                && static_cast<MSDevice_GLOSA*>(getDevice(typeid(MSDevice_GLOSA)))->isSpeedAdviceActive()) {
            stopDist = std::numeric_limits<double>::max();
        }
        if (newStopDist != std::numeric_limits<double>::max()) {
            stopDist = MAX2(stopDist, newStopDist);
        }
#ifdef DEBUG_PLAN_MOVE
        if (DEBUG_COND) {
            std::cout << SIMTIME << " veh=" << getID() << " effective stopOffset on lane '" << lane->getID()
                      << "' is " << laneStopOffset << " (-> stopDist=" << stopDist << ")" << std::endl;
        }
#endif
        if (isRailway(getVClass())
                && !lane->isInternal()) {
            // check for train direction reversal
            if (lane->getBidiLane() != nullptr
                    && (*link)->getLane()->getBidiLane() == lane) {
                double vMustReverse = getCarFollowModel().stopSpeed(this, getSpeed(), seen - POSITION_EPS);
                if (seen < 1) {
                    mustSeeBeforeReversal = 2 * seen + getLength();
                }
                v = MIN2(v, vMustReverse);
            }
            // signal that is passed in the current step does not count
            foundRailSignal |= ((*link)->getTLLogic() != nullptr
                                && (*link)->getTLLogic()->getLogicType() == TrafficLightType::RAIL_SIGNAL
                                && seen > SPEED2DIST(v));
        }

        bool canReverseEventually = false;
        const double vReverse = checkReversal(canReverseEventually, laneMaxV, seen);
        v = MIN2(v, vReverse);
#ifdef DEBUG_PLAN_MOVE
        if (DEBUG_COND) {
            std::cout << SIMTIME << " veh=" << getID() << " canReverseEventually=" << canReverseEventually << " v=" << v << "\n";
        }
#endif

        // check whether we need to slow down in order to finish a continuous lane change
        if (myLaneChangeModel->isChangingLanes()) {
            if (    // slow down to finish lane change before a turn lane
                ((*link)->getDirection() == LinkDirection::LEFT || (*link)->getDirection() == LinkDirection::RIGHT) ||
                // slow down to finish lane change before the shadow lane ends
                (myLaneChangeModel->getShadowLane() != nullptr &&
                 (*link)->getViaLaneOrLane()->getParallelLane(myLaneChangeModel->getShadowDirection()) == nullptr)) {
                // XXX maybe this is too harsh. Vehicles could cut some corners here
                const double timeRemaining = STEPS2TIME(myLaneChangeModel->remainingTime());
                assert(timeRemaining != 0);
                // XXX: Euler-logic (#860), but I couldn't identify problems from this yet (Leo). Refs. #2575
                const double va = MAX2(cfModel.stopSpeed(this, getSpeed(), seen - POSITION_EPS),
                                       (seen - POSITION_EPS) / timeRemaining);
#ifdef DEBUG_PLAN_MOVE
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " veh=" << getID() << " slowing down to finish continuous change before"
                              << " link=" << (*link)->getViaLaneOrLane()->getID()
                              << " timeRemaining=" << timeRemaining
                              << " v=" << v
                              << " va=" << va
                              << std::endl;
                }
#endif
                v = MIN2(va, v);
            }
        }

        // - always issue a request to leave the intersection we are currently on
        const bool leavingCurrentIntersection = myLane->getEdge().isInternal() && lastLink == nullptr;
        // - do not issue a request to enter an intersection after we already slowed down for an earlier one
        const bool abortRequestAfterMinor = slowedDownForMinor && (*link)->getInternalLaneBefore() == nullptr;
        // - even if red, if we cannot break we should issue a request
        bool setRequest = (v > NUMERICAL_EPS_SPEED && !abortRequestAfterMinor) || (leavingCurrentIntersection);

        double stopSpeed = cfModel.stopSpeed(this, getSpeed(), stopDist, stopDecel, MSCFModel::CalcReason::CURRENT_WAIT);
        double vLinkWait = MIN2(v, stopSpeed);
#ifdef DEBUG_PLAN_MOVE
        if (DEBUG_COND) {
            std::cout
                    << " stopDist=" << stopDist
                    << " stopDecel=" << stopDecel
                    << " vLinkWait=" << vLinkWait
                    << " brakeDist=" << brakeDist
                    << " seen=" << seen
                    << " leaveIntersection=" << leavingCurrentIntersection
                    << " setRequest=" << setRequest
                    //<< std::setprecision(16)
                    //<< " v=" << v
                    //<< " speedEps=" << NUMERICAL_EPS_SPEED
                    //<< std::setprecision(gPrecision)
                    << "\n";
        }
#endif

        if (yellowOrRed && canBrakeBeforeStopLine && !ignoreRed(*link, canBrakeBeforeStopLine) && seen >= mustSeeBeforeReversal) {
            if (lane->isInternal()) {
                checkLinkLeaderCurrentAndParallel(*link, lane, seen, lastLink, v, vLinkPass, vLinkWait, setRequest);
            }
            // arrivalSpeed / arrivalTime when braking for red light is only relevent for rail signal switching
            const SUMOTime arrivalTime = getArrivalTime(t, seen, v, vLinkPass);
            // the vehicle is able to brake in front of a yellow/red traffic light
            lfLinks.push_back(DriveProcessItem(*link, v, vLinkWait, false, arrivalTime, vLinkWait, 0, seen, -1));
            //lfLinks.push_back(DriveProcessItem(0, vLinkWait, vLinkWait, false, 0, 0, stopDist));
            break;
        }

        const MSLink* entryLink = (*link)->getCorrespondingEntryLink();
        if (entryLink->haveRed() && ignoreRed(*link, canBrakeBeforeStopLine) && STEPS2TIME(t - entryLink->getLastStateChange()) > 2) {
            // restrict speed when ignoring a red light
            const double redSpeed = MIN2(v, getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_DRIVE_RED_SPEED, v));
            const double va = MAX2(redSpeed, cfModel.freeSpeed(this, getSpeed(), seen, redSpeed));
            v = MIN2(va, v);
#ifdef DEBUG_PLAN_MOVE
            if (DEBUG_COND) std::cout
                        << "   ignoreRed spent=" << STEPS2TIME(t - (*link)->getLastStateChange())
                        << " redSpeed=" << redSpeed
                        << " va=" << va
                        << " v=" << v
                        << "\n";
#endif
        }

        checkLinkLeaderCurrentAndParallel(*link, lane, seen, lastLink, v, vLinkPass, vLinkWait, setRequest);

        if (lastLink != nullptr) {
            lastLink->adaptLeaveSpeed(laneMaxV);
        }
        double arrivalSpeed = vLinkPass;
        // vehicles should decelerate when approaching a minor link
        // - unless they are close enough to have clear visibility of all relevant foe lanes and may start to accelerate again
        // - and unless they are so close that stopping is impossible (i.e. when a green light turns to yellow when close to the junction)

        // whether the vehicle/driver is close enough to the link to see all possible foes #2123
        const double visibilityDistance = (*link)->getFoeVisibilityDistance();
        const double determinedFoePresence = seen <= visibilityDistance;
//        // VARIANT: account for time needed to recognize whether relevant vehicles are on the foe lanes. (Leo)
//        double foeRecognitionTime = 0.0;
//        double determinedFoePresence = seen < visibilityDistance - myState.mySpeed*foeRecognitionTime;

#ifdef DEBUG_PLAN_MOVE
        if (DEBUG_COND) {
            std::cout << " approaching link=" << (*link)->getViaLaneOrLane()->getID() << " prio=" << (*link)->havePriority() << " seen=" << seen << " visibilityDistance=" << visibilityDistance << " brakeDist=" << brakeDist << "\n";
        }
#endif

        const bool couldBrakeForMinor = !(*link)->havePriority() && brakeDist < seen && !(*link)->lastWasContMajor();
        if (couldBrakeForMinor && !determinedFoePresence) {
            // vehicle decelerates just enough to be able to stop if necessary and then accelerates
            double maxSpeedAtVisibilityDist = cfModel.maximumSafeStopSpeed(visibilityDistance, cfModel.getMaxDecel(), myState.mySpeed, false, 0., false);
            // XXX: estimateSpeedAfterDistance does not use euler-logic (thus returns a lower value than possible here...)
            double maxArrivalSpeed = cfModel.estimateSpeedAfterDistance(visibilityDistance, maxSpeedAtVisibilityDist, cfModel.getMaxAccel());
            arrivalSpeed = MIN2(vLinkPass, maxArrivalSpeed);
            slowedDownForMinor = true;
#ifdef DEBUG_PLAN_MOVE
            if (DEBUG_COND) {
                std::cout << "   slowedDownForMinor maxSpeedAtVisDist=" << maxSpeedAtVisibilityDist << " maxArrivalSpeed=" << maxArrivalSpeed << " arrivalSpeed=" << arrivalSpeed << "\n";
            }
#endif
        } else if ((*link)->getState() == LINKSTATE_EQUAL && myWaitingTime > 0) {
            // check for deadlock (circular yielding)
            //std::cout << SIMTIME << " veh=" << getID() << " check rbl-deadlock\n";
            std::pair<const SUMOVehicle*, const MSLink*> blocker = (*link)->getFirstApproachingFoe(*link);
            //std::cout << "   blocker=" << Named::getIDSecure(blocker.first) << "\n";
            int n = 100;
            while (blocker.second != nullptr && blocker.second != *link && n > 0) {
                blocker = blocker.second->getFirstApproachingFoe(*link);
                n--;
                //std::cout << "   blocker=" << Named::getIDSecure(blocker.first) << "\n";
            }
            if (n == 0) {
                WRITE_WARNINGF(TL("Suspicious right_before_left junction '%'."), lane->getEdge().getToJunction()->getID());
            }
            //std::cout << "   blockerLink=" << blocker.second << " link=" << *link << "\n";
            if (blocker.second == *link) {
                const double threshold = (*link)->getDirection() == LinkDirection::STRAIGHT ? 0.25 : 0.75;
                if (RandHelper::rand(getRNG()) < threshold) {
                    //std::cout << "   abort request, threshold=" << threshold << "\n";
                    setRequest = false;
                }
            }
        }

        const SUMOTime arrivalTime = getArrivalTime(t, seen, v, arrivalSpeed);
        if (couldBrakeForMinor && determinedFoePresence && (*link)->getLane()->getEdge().isRoundabout()) {
            const bool wasOpened = (*link)->opened(arrivalTime, arrivalSpeed, arrivalSpeed,
                                                   getLength(), getImpatience(),
                                                   getCarFollowModel().getMaxDecel(),
                                                   getWaitingTime(), getLateralPositionOnLane(),
                                                   nullptr, false, this);
            if (!wasOpened) {
                slowedDownForMinor = true;
            }
#ifdef DEBUG_PLAN_MOVE
            if (DEBUG_COND) {
                std::cout << "   slowedDownForMinor at roundabout=" << (!wasOpened) << "\n";
            }
#endif
        }

        // compute arrival speed and arrival time if vehicle starts braking now
        // if stopping is possible, arrivalTime can be arbitrarily large. A small value keeps fractional times (impatience) meaningful
        double arrivalSpeedBraking = 0;
        const double bGap = cfModel.brakeGap(v);
        if (seen < bGap && !isStopped() && !planningToStop) { // XXX: should this use the current speed (at least for the ballistic case)? (Leo) Refs. #2575
            // vehicle cannot come to a complete stop in time
            if (MSGlobals::gSemiImplicitEulerUpdate) {
                arrivalSpeedBraking = cfModel.getMinimalArrivalSpeedEuler(seen, v);
                // due to discrete/continuous mismatch (when using Euler update) we have to ensure that braking actually helps
                arrivalSpeedBraking = MIN2(arrivalSpeedBraking, arrivalSpeed);
            } else {
                arrivalSpeedBraking = cfModel.getMinimalArrivalSpeed(seen, myState.mySpeed);
            }
        }

        // estimate leave speed for passing time computation
        // l=linkLength, a=accel, t=continuousTime, v=vLeave
        // l=v*t + 0.5*a*t^2, solve for t and multiply with a, then add v
        const double estimatedLeaveSpeed = MIN2((*link)->getViaLaneOrLane()->getVehicleMaxSpeed(this),
                                                getCarFollowModel().estimateSpeedAfterDistance((*link)->getLength(), arrivalSpeed, getVehicleType().getCarFollowModel().getMaxAccel()));
        lfLinks.push_back(DriveProcessItem(*link, v, vLinkWait, setRequest,
                                           arrivalTime, arrivalSpeed,
                                           arrivalSpeedBraking,
                                           seen, estimatedLeaveSpeed));
        if ((*link)->getViaLane() == nullptr) {
            hadNonInternal = true;
            ++view;
        }
#ifdef DEBUG_PLAN_MOVE
        if (DEBUG_COND) {
            std::cout << "   checkAbort setRequest=" << setRequest << " v=" << v << " seen=" << seen << " dist=" << dist
                      << " seenNonInternal=" << seenNonInternal
                      << " seenInternal=" << seenInternal << " length=" << vehicleLength << "\n";
        }
#endif
        // we need to look ahead far enough to see available space for checkRewindLinkLanes
        if ((!setRequest || v <= 0 || seen > dist) && hadNonInternal && seenNonInternal > MAX2(vehicleLength * CRLL_LOOK_AHEAD, vehicleLength + seenInternal) && foundRailSignal) {
            break;
        }
        // get the following lane
        lane = (*link)->getViaLaneOrLane();
        laneMaxV = lane->getVehicleMaxSpeed(this);
        if (myInfluencer && !myInfluencer->considerSafeVelocity()) {
            laneMaxV = std::numeric_limits<double>::max();
        }
        // the link was passed
        // compute the velocity to use when the link is not blocked by other vehicles
        //  the vehicle shall be not faster when reaching the next lane than allowed
        //  speed limits are not emergencies (e.g. when the limit changes suddenly due to TraCI or a variableSpeedSignal)
        const double va = MAX2(cfModel.freeSpeed(this, getSpeed(), seen, laneMaxV), vMinComfortable);
        v = MIN2(va, v);
#ifdef DEBUG_PLAN_MOVE
        if (DEBUG_COND) {
            std::cout << "   laneMaxV=" << laneMaxV << " freeSpeed=" << va << " v=" << v << "\n";
        }
#endif
        if (lane->getEdge().isInternal()) {
            seenInternal += lane->getLength();
        } else {
            seenNonInternal += lane->getLength();
        }
        // do not restrict results to the current vehicle to allow caching for the current time step
        leaderLane = opposite ? lane->getParallelOpposite() : lane;
        if (leaderLane == nullptr) {

            break;
        }
        ahead = opposite ? MSLeaderInfo(leaderLane->getWidth()) : leaderLane->getLastVehicleInformation(nullptr, 0);
        seen += lane->getLength();
        vLinkPass = MIN2(cfModel.estimateSpeedAfterDistance(lane->getLength(), v, cfModel.getMaxAccel()), laneMaxV); // upper bound
        lastLink = &lfLinks.back();
    }

//#ifdef DEBUG_PLAN_MOVE
//    if(DEBUG_COND){
//        std::cout << "planMoveInternal found safe speed v = " << v << std::endl;
//    }
//#endif

#ifdef PARALLEL_STOPWATCH
    myLane->getStopWatch()[0].stop();
#endif
}


double
MSVehicle::slowDownForSchedule(double vMinComfortable) const {
    const double sfp = getVehicleType().getParameter().speedFactorPremature;
    const MSStop& stop = myStops.front();
    std::pair<double, double> timeDist = estimateTimeToNextStop();
    double arrivalDelay = SIMTIME + timeDist.first - STEPS2TIME(stop.pars.arrival);
    double t = STEPS2TIME(stop.pars.arrival - SIMSTEP);
    if (stop.pars.hasParameter(toString(SUMO_ATTR_FLEX_ARRIVAL))) {
        SUMOTime flexStart = string2time(stop.pars.getParameter(toString(SUMO_ATTR_FLEX_ARRIVAL)));
        arrivalDelay += STEPS2TIME(stop.pars.arrival - flexStart);
        t = STEPS2TIME(flexStart - SIMSTEP);
    } else if (stop.pars.started >= 0 && MSGlobals::gUseStopStarted) {
        arrivalDelay += STEPS2TIME(stop.pars.arrival - stop.pars.started);
        t = STEPS2TIME(stop.pars.started - SIMSTEP);
    }
    if (arrivalDelay < 0 && sfp < getChosenSpeedFactor()) {
        // we can slow down to better match the schedule (and increase energy efficiency)
        const double vSlowDownMin = MAX2(myLane->getSpeedLimit() * sfp, vMinComfortable);
        const double s = timeDist.second;
        const double b = getCarFollowModel().getMaxDecel();
        // x = speed for arriving in t seconds
        // u = time at full speed
        // u * x + (t - u) * 0.5 * x = s
        // t - u = x / b
        // eliminate u, solve x
        const double radicand = 4 * t * t * b * b - 8 * s * b;
        const double x = radicand >= 0 ? t * b - sqrt(radicand) * 0.5 : vSlowDownMin;
        double vSlowDown = x < vSlowDownMin ? vSlowDownMin : x;
#ifdef DEBUG_PLAN_MOVE
        if (DEBUG_COND) {
            std::cout << SIMTIME << " veh=" << getID() << " ad=" << arrivalDelay << " t=" << t << " vsm=" << vSlowDownMin
                      << " r=" << radicand << " vs=" << vSlowDown << "\n";
        }
#endif
        return vSlowDown;
    } else if (arrivalDelay > 0 && sfp > getChosenSpeedFactor()) {
        // in principle we could up to catch up with the schedule
        // but at this point we can only lower the speed, the
        // information would have to be used when computing getVehicleMaxSpeed
    }
    return getMaxSpeed();
}

SUMOTime
MSVehicle::getArrivalTime(SUMOTime t, double seen, double v, double arrivalSpeed) const {
    const MSCFModel& cfModel = getCarFollowModel();
    SUMOTime arrivalTime;
    if (MSGlobals::gSemiImplicitEulerUpdate) {
        // @note intuitively it would make sense to compare arrivalSpeed with getSpeed() instead of v
        // however, due to the current position update rule (ticket #860) the vehicle moves with v in this step
        // subtract DELTA_T because t is the time at the end of this step and the movement is not carried out yet
        arrivalTime = t - DELTA_T + cfModel.getMinimalArrivalTime(seen, v, arrivalSpeed);
    } else {
        arrivalTime = t - DELTA_T + cfModel.getMinimalArrivalTime(seen, myState.mySpeed, arrivalSpeed);
    }
    if (isStopped()) {
        arrivalTime += MAX2((SUMOTime)0, myStops.front().duration);
    }
    return arrivalTime;
}


void
MSVehicle::adaptToLeaders(const MSLeaderInfo& ahead, double latOffset,
                          const double seen, DriveProcessItem* const lastLink,
                          const MSLane* const lane, double& v, double& vLinkPass) const {
    int rightmost;
    int leftmost;
    ahead.getSubLanes(this, latOffset, rightmost, leftmost);
#ifdef DEBUG_PLAN_MOVE
    if (DEBUG_COND) std::cout << SIMTIME
                                  << "\nADAPT_TO_LEADERS\nveh=" << getID()
                                  << " lane=" << lane->getID()
                                  << " latOffset=" << latOffset
                                  << " rm=" << rightmost
                                  << " lm=" << leftmost
                                  << " shift=" << ahead.getSublaneOffset()
                                  << " ahead=" << ahead.toString()
                                  << "\n";
#endif
    /*
    if (myLaneChangeModel->getCommittedSpeed() > 0) {
        v = MIN2(v, myLaneChangeModel->getCommittedSpeed());
        vLinkPass = MIN2(vLinkPass, myLaneChangeModel->getCommittedSpeed());
    #ifdef DEBUG_PLAN_MOVE
        if (DEBUG_COND) std::cout << "   hasCommitted=" << myLaneChangeModel->getCommittedSpeed() << "\n";
    #endif
        return;
    }
    */
    for (int sublane = rightmost; sublane <= leftmost; ++sublane) {
        const MSVehicle* pred = ahead[sublane];
        if (pred != nullptr && pred != this) {
            // @todo avoid multiple adaptations to the same leader
            const double predBack = pred->getBackPositionOnLane(lane);
            double gap = (lastLink == nullptr
                          ? predBack - myState.myPos - getVehicleType().getMinGap()
                          : predBack + seen - lane->getLength() - getVehicleType().getMinGap());
            bool oncoming = false;
            if (myLaneChangeModel->isOpposite()) {
                if (pred->getLaneChangeModel().isOpposite() || lane == pred->getLaneChangeModel().getShadowLane()) {
                    // ego might and leader are driving against lane
                    gap = (lastLink == nullptr
                           ? myState.myPos - predBack - getVehicleType().getMinGap()
                           : predBack + seen - lane->getLength() - getVehicleType().getMinGap());
                } else {
                    // ego and leader are driving in the same direction as lane (shadowlane for ego)
                    gap = (lastLink == nullptr
                           ? predBack - (myLane->getLength() - myState.myPos) - getVehicleType().getMinGap()
                           : predBack + seen - lane->getLength() - getVehicleType().getMinGap());
                }
            } else if (pred->getLaneChangeModel().isOpposite() && pred->getLaneChangeModel().getShadowLane() != lane) {
                // must react to stopped / dangerous oncoming vehicles
                gap += -pred->getVehicleType().getLength() + getVehicleType().getMinGap() - MAX2(getVehicleType().getMinGap(), pred->getVehicleType().getMinGap());
                // try to avoid collision in the next second
                const double predMaxDist = pred->getSpeed() + pred->getCarFollowModel().getMaxAccel();
#ifdef DEBUG_PLAN_MOVE
                if (DEBUG_COND) {
                    std::cout << "    fixedGap=" << gap << " predMaxDist=" << predMaxDist << "\n";
                }
#endif
                if (gap < predMaxDist + getSpeed() || pred->getLane() == lane->getBidiLane()) {
                    gap -= predMaxDist;
                }
            } else if (pred->getLane() == lane->getBidiLane()) {
                gap -= pred->getVehicleType().getLengthWithGap();
                oncoming = true;
            }
#ifdef DEBUG_PLAN_MOVE
            if (DEBUG_COND) {
                std::cout << "     pred=" << pred->getID() << " predLane=" << pred->getLane()->getID() << " predPos=" << pred->getPositionOnLane() << " gap=" << gap << " predBack=" << predBack << " seen=" << seen << " lane=" << lane->getID() << " myLane=" << myLane->getID() << " lastLink=" << (lastLink == nullptr ? "NULL" : lastLink->myLink->getDescription()) << " oncoming=" << oncoming << "\n";
            }
#endif
            if (oncoming && gap >= 0) {
                adaptToOncomingLeader(std::make_pair(pred, gap), lastLink, v, vLinkPass);
            } else {
                adaptToLeader(std::make_pair(pred, gap), seen, lastLink, v, vLinkPass);
            }
        }
    }
}

void
MSVehicle::adaptToLeaderDistance(const MSLeaderDistanceInfo& ahead, double latOffset,
                                 double seen,
                                 DriveProcessItem* const lastLink,
                                 double& v, double& vLinkPass) const {
    int rightmost;
    int leftmost;
    ahead.getSubLanes(this, latOffset, rightmost, leftmost);
#ifdef DEBUG_PLAN_MOVE
    if (DEBUG_COND) std::cout << SIMTIME
                                  << "\nADAPT_TO_LEADERS_DISTANCE\nveh=" << getID()
                                  << " latOffset=" << latOffset
                                  << " rm=" << rightmost
                                  << " lm=" << leftmost
                                  << " ahead=" << ahead.toString()
                                  << "\n";
#endif
    for (int sublane = rightmost; sublane <= leftmost; ++sublane) {
        CLeaderDist predDist = ahead[sublane];
        const MSVehicle* pred = predDist.first;
        if (pred != nullptr && pred != this) {
#ifdef DEBUG_PLAN_MOVE
            if (DEBUG_COND) {
                std::cout << "     pred=" << pred->getID() << " predLane=" << pred->getLane()->getID() << " predPos=" << pred->getPositionOnLane() << " gap=" << predDist.second << "\n";
            }
#endif
            adaptToLeader(predDist, seen, lastLink, v, vLinkPass);
        }
    }
}


void
MSVehicle::adaptToLeader(const std::pair<const MSVehicle*, double> leaderInfo,
                         double seen,
                         DriveProcessItem* const lastLink,
                         double& v, double& vLinkPass) const {
    if (leaderInfo.first != 0) {
        if (ignoreFoe(leaderInfo.first)) {
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
            if (DEBUG_COND) {
                std::cout << "  foe ignored\n";
            }
#endif
            return;
        }
        const MSCFModel& cfModel = getCarFollowModel();
        double vsafeLeader = 0;
        if (!MSGlobals::gSemiImplicitEulerUpdate) {
            vsafeLeader = -std::numeric_limits<double>::max();
        }
        bool backOnRoute = true;
        if (leaderInfo.second < 0 && lastLink != nullptr && lastLink->myLink != nullptr) {
            backOnRoute = false;
            // this can either be
            // a) a merging situation (leader back is is not our route) or
            // b) a minGap violation / collision
            MSLane* current = lastLink->myLink->getViaLaneOrLane();
            if (leaderInfo.first->getBackLane() == current) {
                backOnRoute = true;
            } else {
                for (MSLane* lane : getBestLanesContinuation()) {
                    if (lane == current) {
                        break;
                    }
                    if (leaderInfo.first->getBackLane() == lane) {
                        backOnRoute = true;
                    }
                }
            }
#ifdef DEBUG_PLAN_MOVE
            if (DEBUG_COND) {
                std::cout << SIMTIME << " current=" << current->getID() << " leaderBackLane=" << leaderInfo.first->getBackLane()->getID() << " backOnRoute=" << backOnRoute << "\n";
            }
#endif
            if (!backOnRoute) {
                double stopDist = seen - current->getLength() - POSITION_EPS;
                if (lastLink->myLink->getInternalLaneBefore() != nullptr) {
                    // do not drive onto the junction conflict area
                    stopDist -= lastLink->myLink->getInternalLaneBefore()->getLength();
                }
                vsafeLeader = cfModel.stopSpeed(this, getSpeed(), stopDist);
            }
        }
        if (backOnRoute) {
            vsafeLeader = cfModel.followSpeed(this, getSpeed(), leaderInfo.second, leaderInfo.first->getSpeed(), leaderInfo.first->getCurrentApparentDecel(), leaderInfo.first);
        }
        if (lastLink != nullptr) {
            const double futureVSafe = cfModel.followSpeed(this, lastLink->accelV, leaderInfo.second, leaderInfo.first->getSpeed(), leaderInfo.first->getCurrentApparentDecel(), leaderInfo.first, MSCFModel::CalcReason::FUTURE);
            lastLink->adaptLeaveSpeed(futureVSafe);
#ifdef DEBUG_PLAN_MOVE
            if (DEBUG_COND) {
                std::cout << "   vlinkpass=" << lastLink->myVLinkPass << " futureVSafe=" << futureVSafe << "\n";
            }
#endif
        }
        v = MIN2(v, vsafeLeader);
        vLinkPass = MIN2(vLinkPass, vsafeLeader);
#ifdef DEBUG_PLAN_MOVE
        if (DEBUG_COND) std::cout
                    << SIMTIME
                    //std::cout << std::setprecision(10);
                    << " veh=" << getID()
                    << " lead=" << leaderInfo.first->getID()
                    << " leadSpeed=" << leaderInfo.first->getSpeed()
                    << " gap=" << leaderInfo.second
                    << " leadLane=" << leaderInfo.first->getLane()->getID()
                    << " predPos=" << leaderInfo.first->getPositionOnLane()
                    << " myLane=" << myLane->getID()
                    << " v=" << v
                    << " vSafeLeader=" << vsafeLeader
                    << " vLinkPass=" << vLinkPass
                    << "\n";
#endif
    }
}


void
MSVehicle::adaptToJunctionLeader(const std::pair<const MSVehicle*, double> leaderInfo,
                                 const double seen, DriveProcessItem* const lastLink,
                                 const MSLane* const lane, double& v, double& vLinkPass,
                                 double distToCrossing) const {
    if (leaderInfo.first != 0) {
        if (ignoreFoe(leaderInfo.first)) {
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
            if (DEBUG_COND) {
                std::cout << "  junction foe ignored\n";
            }
#endif
            return;
        }
        const MSCFModel& cfModel = getCarFollowModel();
        double vsafeLeader = 0;
        if (!MSGlobals::gSemiImplicitEulerUpdate) {
            vsafeLeader = -std::numeric_limits<double>::max();
        }
        if (leaderInfo.second >= 0) {
            if (hasDeparted()) {
                vsafeLeader = cfModel.followSpeed(this, getSpeed(), leaderInfo.second, leaderInfo.first->getSpeed(), leaderInfo.first->getCurrentApparentDecel(), leaderInfo.first);
            } else {
                // called in the context of MSLane::isInsertionSuccess
                vsafeLeader = cfModel.insertionFollowSpeed(this, getSpeed(), leaderInfo.second, leaderInfo.first->getSpeed(), leaderInfo.first->getCurrentApparentDecel(), leaderInfo.first);
            }
        } else if (leaderInfo.first != this) {
            // the leading, in-lapping vehicle is occupying the complete next lane
            // stop before entering this lane
            vsafeLeader = cfModel.stopSpeed(this, getSpeed(), seen - lane->getLength() - POSITION_EPS);
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
            if (DEBUG_COND) {
                std::cout << SIMTIME << " veh=" << getID() << "  stopping before junction: lane=" << lane->getID() << " seen=" << seen
                          << " laneLength=" << lane->getLength()
                          << " stopDist=" << seen - lane->getLength()  - POSITION_EPS
                          << " vsafeLeader=" << vsafeLeader
                          << " distToCrossing=" << distToCrossing
                          << "\n";
            }
#endif
        }
        if (distToCrossing >= 0) {
            // can the leader still stop in the way?
            const double vStop = cfModel.stopSpeed(this, getSpeed(), distToCrossing - getVehicleType().getMinGap());
            if (leaderInfo.first == this) {
                // braking for pedestrian
                const double vStopCrossing = cfModel.stopSpeed(this, getSpeed(), distToCrossing);
                vsafeLeader = vStopCrossing;
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
                if (DEBUG_COND) {
                    std::cout << "  breaking for pedestrian distToCrossing=" << distToCrossing << " vStopCrossing=" << vStopCrossing << "\n";
                }
#endif
                if (lastLink != nullptr) {
                    lastLink->adaptStopSpeed(vsafeLeader);
                }
            } else if (leaderInfo.second == -std::numeric_limits<double>::max()) {
                // drive up to the crossing point and stop
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
                if (DEBUG_COND) {
                    std::cout << "  stop at crossing point for critical leader vStop=" << vStop << "\n";
                };
#endif
                vsafeLeader = MAX2(vsafeLeader, vStop);
            } else {
                const double leaderDistToCrossing = distToCrossing - leaderInfo.second;
                // estimate the time at which the leader has gone past the crossing point
                const double leaderPastCPTime = leaderDistToCrossing / MAX2(leaderInfo.first->getSpeed(), SUMO_const_haltingSpeed);
                // reach distToCrossing after that time
                // avgSpeed * leaderPastCPTime = distToCrossing
                // ballistic: avgSpeed = (getSpeed + vFinal) / 2
                const double vFinal = MAX2(getSpeed(), 2 * (distToCrossing - getVehicleType().getMinGap()) / leaderPastCPTime - getSpeed());
                const double v2 = getSpeed() + ACCEL2SPEED((vFinal - getSpeed()) / leaderPastCPTime);
                vsafeLeader = MAX2(vsafeLeader, MIN2(v2, vStop));
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
                if (DEBUG_COND) {
                    std::cout << "    driving up to the crossing point (distToCrossing=" << distToCrossing << ")"
                              << " leaderPastCPTime=" << leaderPastCPTime
                              << " vFinal=" << vFinal
                              << " v2=" << v2
                              << " vStop=" << vStop
                              << " vsafeLeader=" << vsafeLeader << "\n";
                }
#endif
            }
        }
        if (lastLink != nullptr) {
            lastLink->adaptLeaveSpeed(vsafeLeader);
        }
        v = MIN2(v, vsafeLeader);
        vLinkPass = MIN2(vLinkPass, vsafeLeader);
#ifdef DEBUG_PLAN_MOVE
        if (DEBUG_COND) std::cout
                    << SIMTIME
                    //std::cout << std::setprecision(10);
                    << " veh=" << getID()
                    << " lead=" << leaderInfo.first->getID()
                    << " leadSpeed=" << leaderInfo.first->getSpeed()
                    << " gap=" << leaderInfo.second
                    << " leadLane=" << leaderInfo.first->getLane()->getID()
                    << " predPos=" << leaderInfo.first->getPositionOnLane()
                    << " seen=" << seen
                    << " lane=" << lane->getID()
                    << " myLane=" << myLane->getID()
                    << " dTC=" << distToCrossing
                    << " v=" << v
                    << " vSafeLeader=" << vsafeLeader
                    << " vLinkPass=" << vLinkPass
                    << "\n";
#endif
    }
}


void
MSVehicle::adaptToOncomingLeader(const std::pair<const MSVehicle*, double> leaderInfo,
                                 DriveProcessItem* const lastLink,
                                 double& v, double& vLinkPass) const {
    if (leaderInfo.first != 0) {
        if (ignoreFoe(leaderInfo.first)) {
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
            if (DEBUG_COND) {
                std::cout << "  oncoming foe ignored\n";
            }
#endif
            return;
        }
        const MSCFModel& cfModel = getCarFollowModel();
        const MSVehicle* lead = leaderInfo.first;
        const MSCFModel& cfModelL = lead->getCarFollowModel();
        // assume the leader reacts symmetrically (neither stopping instantly nor ignoring ego)
        const double leaderBrakeGap = cfModelL.brakeGap(lead->getSpeed(), cfModelL.getMaxDecel(), 0);
        const double egoBrakeGap = cfModel.brakeGap(getSpeed(), cfModel.getMaxDecel(), 0);
        const double gapSum = leaderBrakeGap + egoBrakeGap;
        // ensure that both vehicles can leave an intersection if they are currently on it
        double egoExit = getDistanceToLeaveJunction();
        const double leaderExit = lead->getDistanceToLeaveJunction();
        double gap = leaderInfo.second;
        if (egoExit + leaderExit < gap) {
            gap -= egoExit + leaderExit;
        } else {
            egoExit = 0;
        }
        // split any distance in excess of brakeGaps evenly
        const double freeGap = MAX2(0.0, gap - gapSum);
        const double splitGap = MIN2(gap, gapSum);
        // assume remaining distance is allocated in proportion to braking distance
        const double gapRatio = gapSum > 0 ? egoBrakeGap / gapSum : 0.5;
        const double vsafeLeader = cfModel.stopSpeed(this, getSpeed(), splitGap * gapRatio + egoExit + 0.5 * freeGap);
        if (lastLink != nullptr) {
            const double futureVSafe = cfModel.stopSpeed(this, lastLink->accelV, leaderInfo.second, MSCFModel::CalcReason::FUTURE);
            lastLink->adaptLeaveSpeed(futureVSafe);
#ifdef DEBUG_PLAN_MOVE
            if (DEBUG_COND) {
                std::cout << "   vlinkpass=" << lastLink->myVLinkPass << " futureVSafe=" << futureVSafe << "\n";
            }
#endif
        }
        v = MIN2(v, vsafeLeader);
        vLinkPass = MIN2(vLinkPass, vsafeLeader);
#ifdef DEBUG_PLAN_MOVE
        if (DEBUG_COND) std::cout
                    << SIMTIME
                    //std::cout << std::setprecision(10);
                    << " veh=" << getID()
                    << " oncomingLead=" << lead->getID()
                    << " leadSpeed=" << lead->getSpeed()
                    << " gap=" << leaderInfo.second
                    << " gap2=" << gap
                    << " gapRatio=" << gapRatio
                    << " leadLane=" << lead->getLane()->getID()
                    << " predPos=" << lead->getPositionOnLane()
                    << " myLane=" << myLane->getID()
                    << " v=" << v
                    << " vSafeLeader=" << vsafeLeader
                    << " vLinkPass=" << vLinkPass
                    << "\n";
#endif
    }
}


void
MSVehicle::checkLinkLeaderCurrentAndParallel(const MSLink* link, const MSLane* lane, double seen,
        DriveProcessItem* const lastLink, double& v, double& vLinkPass, double& vLinkWait, bool& setRequest) const {
    if (MSGlobals::gUsingInternalLanes && (myInfluencer == nullptr || myInfluencer->getRespectJunctionLeaderPriority())) {
        // we want to pass the link but need to check for foes on internal lanes
        checkLinkLeader(link, lane, seen, lastLink, v, vLinkPass, vLinkWait, setRequest);
        if (myLaneChangeModel->getShadowLane() != nullptr) {
            const MSLink* const parallelLink = link->getParallelLink(myLaneChangeModel->getShadowDirection());
            if (parallelLink != nullptr) {
                checkLinkLeader(parallelLink, lane, seen, lastLink, v, vLinkPass, vLinkWait, setRequest, true);
            }
        }
    }

}

void
MSVehicle::checkLinkLeader(const MSLink* link, const MSLane* lane, double seen,
                           DriveProcessItem* const lastLink, double& v, double& vLinkPass, double& vLinkWait, bool& setRequest,
                           bool isShadowLink) const {
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
    if (DEBUG_COND) {
        gDebugFlag1 = true;    // See MSLink::getLeaderInfo
    }
#endif
    const MSLink::LinkLeaders linkLeaders = link->getLeaderInfo(this, seen, nullptr, isShadowLink);
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
    if (DEBUG_COND) {
        gDebugFlag1 = false;    // See MSLink::getLeaderInfo
    }
#endif
    for (MSLink::LinkLeaders::const_iterator it = linkLeaders.begin(); it != linkLeaders.end(); ++it) {
        // the vehicle to enter the junction first has priority
        const MSVehicle* leader = (*it).vehAndGap.first;
        if (leader == nullptr) {
            // leader is a pedestrian. Passing 'this' as a dummy.
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
            if (DEBUG_COND) {
                std::cout << SIMTIME << " veh=" << getID() << " is blocked on link to " << link->getViaLaneOrLane()->getID() << " by pedestrian. dist=" << it->distToCrossing << "\n";
            }
#endif
            if (getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_IGNORE_JUNCTION_FOE_PROB, 0) > 0
                    && getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_IGNORE_JUNCTION_FOE_PROB, 0) >= RandHelper::rand(getRNG())) {
#ifdef DEBUG_PLAN_MOVE
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " veh=" << getID() << " is ignoring pedestrian (jmIgnoreJunctionFoeProb)\n";
                }
#endif
                continue;
            }
            adaptToJunctionLeader(std::make_pair(this, -1), seen, lastLink, lane, v, vLinkPass, it->distToCrossing);
            // if blocked by a pedestrian for too long we must yield our request
            if (v < SUMO_const_haltingSpeed && getWaitingTime() > TIME2STEPS(JUNCTION_BLOCKAGE_TIME)) {
                setRequest = false;
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
                if (DEBUG_COND) {
                    std::cout << "   aborting request\n";
                }
#endif
            }
        } else if (isLeader(link, leader, (*it).vehAndGap.second) || (*it).inTheWay()) {
            if (getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_IGNORE_JUNCTION_FOE_PROB, 0) > 0
                    && getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_IGNORE_JUNCTION_FOE_PROB, 0) >= RandHelper::rand(getRNG())) {
#ifdef DEBUG_PLAN_MOVE
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " veh=" << getID() << " is ignoring linkLeader=" << leader->getID() << " (jmIgnoreJunctionFoeProb)\n";
                }
#endif
                continue;
            }
            if (MSGlobals::gLateralResolution > 0 &&
                    // sibling link (XXX: could also be partial occupator where this check fails)
                    &leader->getLane()->getEdge() == &lane->getEdge()) {
                // check for sublane obstruction (trivial for sibling link leaders)
                const MSLane* conflictLane = link->getInternalLaneBefore();
                MSLeaderInfo linkLeadersAhead = MSLeaderInfo(conflictLane->getWidth());
                linkLeadersAhead.addLeader(leader, false, 0); // assume sibling lane has the same geometry as the leader lane
                const double latOffset = isShadowLink ? (getLane()->getRightSideOnEdge() - myLaneChangeModel->getShadowLane()->getRightSideOnEdge()) : 0;
                // leader is neither on lane nor conflictLane (the conflict is only established geometrically)
                adaptToLeaders(linkLeadersAhead, latOffset, seen, lastLink, leader->getLane(), v, vLinkPass);
#ifdef DEBUG_PLAN_MOVE
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " veh=" << getID()
                              << " siblingFoe link=" << link->getViaLaneOrLane()->getID()
                              << " isShadowLink=" << isShadowLink
                              << " lane=" << lane->getID()
                              << " foe=" << leader->getID()
                              << " foeLane=" << leader->getLane()->getID()
                              << " latOffset=" << latOffset
                              << " latOffsetFoe=" << leader->getLatOffset(lane)
                              << " linkLeadersAhead=" << linkLeadersAhead.toString()
                              << "\n";
                }
#endif
            } else {
#ifdef DEBUG_PLAN_MOVE
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " veh=" << getID() << " linkLeader=" << leader->getID() << " gap=" << it->vehAndGap.second
                              << " ET=" << myJunctionEntryTime << " lET=" << leader->myJunctionEntryTime
                              << " ETN=" << myJunctionEntryTimeNeverYield << " lETN=" << leader->myJunctionEntryTimeNeverYield
                              << " CET=" << myJunctionConflictEntryTime << " lCET=" << leader->myJunctionConflictEntryTime
                              << "\n";
                }
#endif
                adaptToJunctionLeader(it->vehAndGap, seen, lastLink, lane, v, vLinkPass, it->distToCrossing);
            }
            if (lastLink != nullptr) {
                // we are not yet on the junction with this linkLeader.
                // at least we can drive up to the previous link and stop there
                v = MAX2(v, lastLink->myVLinkWait);
            }
            // if blocked by a leader from the same or next lane we must yield our request
            // also, if blocked by a stopped or blocked leader
            if (v < SUMO_const_haltingSpeed
                    //&& leader->getSpeed() < SUMO_const_haltingSpeed
                    && (leader->getLane()->getLogicalPredecessorLane() == myLane->getLogicalPredecessorLane()
                        || leader->getLane()->getLogicalPredecessorLane() == myLane
                        || leader->isStopped()
                        || leader->getWaitingTime() > TIME2STEPS(JUNCTION_BLOCKAGE_TIME))) {
                setRequest = false;
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
                if (DEBUG_COND) {
                    std::cout << "   aborting request\n";
                }
#endif
                if (lastLink != nullptr && leader->getLane()->getLogicalPredecessorLane() == myLane) {
                    // we are not yet on the junction so must abort that request as well
                    // (or maybe we are already on the junction and the leader is a partial occupator beyond)
                    lastLink->mySetRequest = false;
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
                    if (DEBUG_COND) {
                        std::cout << "      aborting previous request\n";
                    }
#endif
                }
            }
        }
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
        else {
            if (DEBUG_COND) {
                std::cout << SIMTIME << " veh=" << getID() << " ignoring leader " << leader->getID() << " gap=" << (*it).vehAndGap.second << " dtC=" << (*it).distToCrossing
                          << " ET=" << myJunctionEntryTime << " lET=" << leader->myJunctionEntryTime
                          << " ETN=" << myJunctionEntryTimeNeverYield << " lETN=" << leader->myJunctionEntryTimeNeverYield
                          << " CET=" << myJunctionConflictEntryTime << " lCET=" << leader->myJunctionConflictEntryTime
                          << "\n";
            }
        }
#endif
    }
    // if this is the link between two internal lanes we may have to slow down for pedestrians
    vLinkWait = MIN2(vLinkWait, v);
}


double
MSVehicle::getDeltaPos(const double accel) const {
    double vNext = myState.mySpeed + ACCEL2SPEED(accel);
    if (MSGlobals::gSemiImplicitEulerUpdate) {
        // apply implicit Euler positional update
        return SPEED2DIST(MAX2(vNext, 0.));
    } else {
        // apply ballistic update
        if (vNext >= 0) {
            // assume constant acceleration during this time step
            return SPEED2DIST(myState.mySpeed + 0.5 * ACCEL2SPEED(accel));
        } else {
            // negative vNext indicates a stop within the middle of time step
            // The corresponding stop time is s = mySpeed/deceleration \in [0,dt], and the
            // covered distance is therefore deltaPos = mySpeed*s - 0.5*deceleration*s^2.
            // Here, deceleration = (myState.mySpeed - vNext)/dt is the constant deceleration
            // until the vehicle stops.
            return -SPEED2DIST(0.5 * myState.mySpeed * myState.mySpeed / ACCEL2SPEED(accel));
        }
    }
}

void
MSVehicle::processLinkApproaches(double& vSafe, double& vSafeMin, double& vSafeMinDist) {

    // Speed limit due to zipper merging
    double vSafeZipper = std::numeric_limits<double>::max();

    myHaveToWaitOnNextLink = false;
    bool canBrakeVSafeMin = false;

    // Get safe velocities from DriveProcessItems.
    assert(myLFLinkLanes.size() != 0 || isRemoteControlled());
    for (const DriveProcessItem& dpi : myLFLinkLanes) {
        MSLink* const link = dpi.myLink;

#ifdef DEBUG_EXEC_MOVE
        if (DEBUG_COND) {
            std::cout
                    << SIMTIME
                    << " veh=" << getID()
                    << " link=" << (link == 0 ? "NULL" : link->getViaLaneOrLane()->getID())
                    << " req=" << dpi.mySetRequest
                    << " vP=" << dpi.myVLinkPass
                    << " vW=" << dpi.myVLinkWait
                    << " d=" << dpi.myDistance
                    << "\n";
            gDebugFlag1 = true; // See MSLink_DEBUG_OPENED
        }
#endif

        // the vehicle must change the lane on one of the next lanes (XXX: refs to code further below???, Leo)
        if (link != nullptr && dpi.mySetRequest) {

            const LinkState ls = link->getState();
            // vehicles should brake when running onto a yellow light if the distance allows to halt in front
            const bool yellow = link->haveYellow();
            const bool canBrake = (dpi.myDistance > getCarFollowModel().brakeGap(myState.mySpeed, getCarFollowModel().getMaxDecel(), 0.)
                                   || (MSGlobals::gSemiImplicitEulerUpdate && myState.mySpeed < ACCEL2SPEED(getCarFollowModel().getMaxDecel())));
            assert(link->getLaneBefore() != nullptr);
            const bool beyondStopLine = dpi.myDistance < link->getLaneBefore()->getVehicleStopOffset(this);
            const bool ignoreRedLink = ignoreRed(link, canBrake) || beyondStopLine;
            if (yellow && canBrake && !ignoreRedLink) {
                vSafe = dpi.myVLinkWait;
                myHaveToWaitOnNextLink = true;
#ifdef DEBUG_CHECKREWINDLINKLANES
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " veh=" << getID() << " haveToWait (yellow)\n";
                }
#endif
                break;
            }
            const bool influencerPrio = (myInfluencer != nullptr && !myInfluencer->getRespectJunctionPriority());
            MSLink::BlockingFoes collectFoes;
            bool opened = (yellow || influencerPrio
                           || link->opened(dpi.myArrivalTime, dpi.myArrivalSpeed, dpi.getLeaveSpeed(),
                                           getVehicleType().getLength(),
                                           canBrake ? getImpatience() : 1,
                                           getCarFollowModel().getMaxDecel(),
                                           getWaitingTime(), getLateralPositionOnLane(),
                                           ls == LINKSTATE_ZIPPER ? &collectFoes : nullptr,
                                           ignoreRedLink, this, dpi.myDistance));
            if (opened && myLaneChangeModel->getShadowLane() != nullptr) {
                const MSLink* const parallelLink = dpi.myLink->getParallelLink(myLaneChangeModel->getShadowDirection());
                if (parallelLink != nullptr) {
                    const double shadowLatPos = getLateralPositionOnLane() - myLaneChangeModel->getShadowDirection() * 0.5 * (
                                                    myLane->getWidth() + myLaneChangeModel->getShadowLane()->getWidth());
                    opened = yellow || influencerPrio || (opened && parallelLink->opened(dpi.myArrivalTime, dpi.myArrivalSpeed, dpi.getLeaveSpeed(),
                                                          getVehicleType().getLength(), getImpatience(),
                                                          getCarFollowModel().getMaxDecel(),
                                                          getWaitingTime(), shadowLatPos, nullptr,
                                                          ignoreRedLink, this, dpi.myDistance));
#ifdef DEBUG_EXEC_MOVE
                    if (DEBUG_COND) {
                        std::cout << SIMTIME
                                  << " veh=" << getID()
                                  << " shadowLane=" << myLaneChangeModel->getShadowLane()->getID()
                                  << " shadowDir=" << myLaneChangeModel->getShadowDirection()
                                  << " parallelLink=" << (parallelLink == 0 ? "NULL" : parallelLink->getViaLaneOrLane()->getID())
                                  << " opened=" << opened
                                  << "\n";
                    }
#endif
                }
            }
            // vehicles should decelerate when approaching a minor link
#ifdef DEBUG_EXEC_MOVE
            if (DEBUG_COND) {
                std::cout << SIMTIME
                          << "   opened=" << opened
                          << " influencerPrio=" << influencerPrio
                          << " linkPrio=" << link->havePriority()
                          << " lastContMajor=" << link->lastWasContMajor()
                          << " isCont=" << link->isCont()
                          << " ignoreRed=" << ignoreRedLink
                          << "\n";
            }
#endif
            if (opened && !influencerPrio && !link->havePriority() && !link->lastWasContMajor() && !link->isCont() && !ignoreRedLink) {
                double visibilityDistance = link->getFoeVisibilityDistance();
                double determinedFoePresence = dpi.myDistance <= visibilityDistance;
                if (!determinedFoePresence && (canBrake || !yellow)) {
                    vSafe = dpi.myVLinkWait;
                    myHaveToWaitOnNextLink = true;
#ifdef DEBUG_CHECKREWINDLINKLANES
                    if (DEBUG_COND) {
                        std::cout << SIMTIME << " veh=" << getID() << " haveToWait (minor)\n";
                    }
#endif
                    break;
                } else {
                    // past the point of no return. we need to drive fast enough
                    // to make it across the link. However, minor slowdowns
                    // should be permissible to follow leading traffic safely
                    // basically, this code prevents dawdling
                    // (it's harder to do this later using
                    // SUMO_ATTR_JM_SIGMA_MINOR because we don't know whether the
                    // vehicle is already too close to stop at that part of the code)
                    //
                    // XXX: There is a problem in subsecond simulation: If we cannot
                    // make it across the minor link in one step, new traffic
                    // could appear on a major foe link and cause a collision. Refs. #1845, #2123
                    vSafeMinDist = dpi.myDistance; // distance that must be covered
                    if (MSGlobals::gSemiImplicitEulerUpdate) {
                        vSafeMin = MIN3((double)DIST2SPEED(vSafeMinDist + POSITION_EPS), dpi.myVLinkPass, getCarFollowModel().maxNextSafeMin(getSpeed(), this));
                    } else {
                        vSafeMin = MIN3((double)DIST2SPEED(2 * vSafeMinDist + NUMERICAL_EPS) - getSpeed(), dpi.myVLinkPass, getCarFollowModel().maxNextSafeMin(getSpeed(), this));
                    }
                    canBrakeVSafeMin = canBrake;
#ifdef DEBUG_EXEC_MOVE
                    if (DEBUG_COND) {
                        std::cout << "     vSafeMin=" << vSafeMin << " vSafeMinDist=" << vSafeMinDist << " canBrake=" << canBrake << "\n";
                    }
#endif
                }
            }
            // have waited; may pass if opened...
            if (opened) {
                vSafe = dpi.myVLinkPass;
                if (vSafe < getCarFollowModel().getMaxDecel() && vSafe <= dpi.myVLinkWait && vSafe < getCarFollowModel().maxNextSpeed(getSpeed(), this)) {
                    // this vehicle is probably not gonna drive across the next junction (heuristic)
                    myHaveToWaitOnNextLink = true;
#ifdef DEBUG_CHECKREWINDLINKLANES
                    if (DEBUG_COND) {
                        std::cout << SIMTIME << " veh=" << getID() << " haveToWait (very slow)\n";
                    }
#endif
                }
            } else if (link->getState() == LINKSTATE_ZIPPER) {
                vSafeZipper = MIN2(vSafeZipper,
                                   link->getZipperSpeed(this, dpi.myDistance, dpi.myVLinkPass, dpi.myArrivalTime, &collectFoes));
            } else if (!canBrake
                       // always brake hard for traffic lights (since an emergency stop is necessary anyway)
                       && link->getTLLogic() == nullptr
                       // cannot brake even with emergency deceleration
                       && dpi.myDistance < getCarFollowModel().brakeGap(myState.mySpeed, getCarFollowModel().getEmergencyDecel(), 0.)) {
#ifdef DEBUG_EXEC_MOVE
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " too fast to brake for closed link\n";
                }
#endif
                vSafe = dpi.myVLinkPass;
            } else {
                vSafe = dpi.myVLinkWait;
                myHaveToWaitOnNextLink = true;
#ifdef DEBUG_CHECKREWINDLINKLANES
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " veh=" << getID() << " haveToWait (closed)\n";
                }
#endif
#ifdef DEBUG_EXEC_MOVE
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " braking for closed link=" << link->getViaLaneOrLane()->getID() << "\n";
                }
#endif
                break;
            }
        } else {
            if (link != nullptr && link->getInternalLaneBefore() != nullptr && myLane->isInternal() && link->getJunction() == myLane->getEdge().getToJunction()) {
                // blocked on the junction. yield request so other vehicles may
                // become junction leader
#ifdef DEBUG_EXEC_MOVE
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " resetting junctionEntryTime at junction '" << link->getJunction()->getID() << "' beause of non-request exitLink\n";
                }
#endif
                myJunctionEntryTime = SUMOTime_MAX;
                myJunctionConflictEntryTime = SUMOTime_MAX;
            }
            // we have: i->link == 0 || !i->setRequest
            vSafe = dpi.myVLinkWait;
            if (vSafe < getSpeed()) {
                myHaveToWaitOnNextLink = true;
#ifdef DEBUG_CHECKREWINDLINKLANES
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " veh=" << getID() << " haveToWait (no request, braking) vSafe=" << vSafe << "\n";
                }
#endif
            } else if (vSafe < SUMO_const_haltingSpeed) {
                myHaveToWaitOnNextLink = true;
#ifdef DEBUG_CHECKREWINDLINKLANES
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " veh=" << getID() << " haveToWait (no request, stopping)\n";
                }
#endif
            }
            if (link == nullptr && myLFLinkLanes.size() == 1
                    && getBestLanesContinuation().size() > 1
                    && getBestLanesContinuation()[1]->hadPermissionChanges()
                    && myLane->getFirstAnyVehicle() == this) {
                // temporal lane closing without notification, visible to the
                // vehicle at the front of the queue
                updateBestLanes(true);
                //std::cout << SIMTIME << " veh=" << getID() << " updated bestLanes=" << toString(getBestLanesContinuation()) << "\n";
            }
            break;
        }
    }

//#ifdef DEBUG_EXEC_MOVE
//    if (DEBUG_COND) {
//        std::cout << "\nvCurrent = " << toString(getSpeed(), 24) << "" << std::endl;
//        std::cout << "vSafe = " << toString(vSafe, 24) << "" << std::endl;
//        std::cout << "vSafeMin = " << toString(vSafeMin, 24) << "" << std::endl;
//        std::cout << "vSafeMinDist = " << toString(vSafeMinDist, 24) << "" << std::endl;
//
//        double gap = getLeader().second;
//        std::cout << "gap = " << toString(gap, 24) << std::endl;
//        std::cout << "vSafeStoppedLeader = " << toString(getCarFollowModel().stopSpeed(this, getSpeed(), gap, MSCFModel::CalcReason::FUTURE), 24)
//                << "\n" << std::endl;
//    }
//#endif

    if ((MSGlobals::gSemiImplicitEulerUpdate && vSafe + NUMERICAL_EPS < vSafeMin)
            || (!MSGlobals::gSemiImplicitEulerUpdate && (vSafe + NUMERICAL_EPS < vSafeMin && vSafeMin != 0))) { // this might be good for the euler case as well
        // XXX: (Leo) This often called stopSpeed with vSafeMinDist==0 (for the ballistic update), since vSafe can become negative
        //      For the Euler update the term '+ NUMERICAL_EPS' prevented a call here... Recheck, consider of -INVALID_SPEED instead of 0 to indicate absence of vSafeMin restrictions. Refs. #2577
#ifdef DEBUG_EXEC_MOVE
        if (DEBUG_COND) {
            std::cout << "vSafeMin Problem? vSafe=" << vSafe << " vSafeMin=" << vSafeMin << " vSafeMinDist=" << vSafeMinDist << std::endl;
        }
#endif
        if (canBrakeVSafeMin && vSafe < getSpeed()) {
            // cannot drive across a link so we need to stop before it
            vSafe = MIN2(vSafe, MAX2(getCarFollowModel().minNextSpeed(getSpeed(), this),
                                     getCarFollowModel().stopSpeed(this, getSpeed(), vSafeMinDist)));
            vSafeMin = 0;
            myHaveToWaitOnNextLink = true;
#ifdef DEBUG_CHECKREWINDLINKLANES
            if (DEBUG_COND) {
                std::cout << SIMTIME << " veh=" << getID() << " haveToWait (vSafe=" << vSafe << " < vSafeMin=" << vSafeMin << ")\n";
            }
#endif
        } else {
            // if the link is yellow or visibility distance is large
            // then we might not make it across the link in one step anyway..
            // Possibly, the lane after the intersection has a lower speed limit so
            // we really need to drive slower already
            // -> keep driving without dawdling
            vSafeMin = vSafe;
        }
    }

    // vehicles inside a roundabout should maintain their requests
    if (myLane->getEdge().isRoundabout()) {
        myHaveToWaitOnNextLink = false;
    }

    vSafe = MIN2(vSafe, vSafeZipper);
}


double
MSVehicle::processTraCISpeedControl(double vSafe, double vNext) {
    if (myInfluencer != nullptr) {
        myInfluencer->setOriginalSpeed(vNext);
#ifdef DEBUG_TRACI
        if DEBUG_COND2(this) {
            std::cout << SIMTIME << " MSVehicle::processTraCISpeedControl() for vehicle '" << getID() << "'"
                      << " vSafe=" << vSafe << " (init)vNext=" << vNext << " keepStopping=" << keepStopping();
        }
#endif
        if (myInfluencer->isRemoteControlled()) {
            vNext = myInfluencer->implicitSpeedRemote(this, myState.mySpeed);
        }
        const double vMax = getVehicleType().getCarFollowModel().maxNextSpeed(myState.mySpeed, this);
        double vMin = getVehicleType().getCarFollowModel().minNextSpeed(myState.mySpeed, this);
        if (MSGlobals::gSemiImplicitEulerUpdate) {
            vMin = MAX2(0., vMin);
        }
        vNext = myInfluencer->influenceSpeed(MSNet::getInstance()->getCurrentTimeStep(), vNext, vSafe, vMin, vMax);
        if (keepStopping() && myStops.front().getSpeed() == 0) {
            // avoid driving while stopped (unless it's actually a waypoint
            vNext = myInfluencer->getOriginalSpeed();
        }
#ifdef DEBUG_TRACI
        if DEBUG_COND2(this) {
            std::cout << " (processed)vNext=" << vNext << std::endl;
        }
#endif
    }
    return vNext;
}


void
MSVehicle::removePassedDriveItems() {
#ifdef DEBUG_ACTIONSTEPS
    if (DEBUG_COND) {
        std::cout << SIMTIME << " veh=" << getID() << " removePassedDriveItems()\n"
                  << "    Current items: ";
        for (auto& j : myLFLinkLanes) {
            if (j.myLink == 0) {
                std::cout << "\n    Stop at distance " << j.myDistance;
            } else {
                const MSLane* to = j.myLink->getViaLaneOrLane();
                const MSLane* from = j.myLink->getLaneBefore();
                std::cout << "\n    Link at distance " << j.myDistance << ": '"
                          << (from == 0 ? "NONE" : from->getID()) << "' -> '" << (to == 0 ? "NONE" : to->getID()) << "'";
            }
        }
        std::cout << "\n    myNextDriveItem: ";
        if (myLFLinkLanes.size() != 0) {
            if (myNextDriveItem->myLink == 0) {
                std::cout << "\n    Stop at distance " << myNextDriveItem->myDistance;
            } else {
                const MSLane* to = myNextDriveItem->myLink->getViaLaneOrLane();
                const MSLane* from = myNextDriveItem->myLink->getLaneBefore();
                std::cout << "\n    Link at distance " << myNextDriveItem->myDistance << ": '"
                          << (from == 0 ? "NONE" : from->getID()) << "' -> '" << (to == 0 ? "NONE" : to->getID()) << "'";
            }
        }
        std::cout << std::endl;
    }
#endif
    for (auto j = myLFLinkLanes.begin(); j != myNextDriveItem; ++j) {
#ifdef DEBUG_ACTIONSTEPS
        if (DEBUG_COND) {
            std::cout << "    Removing item: ";
            if (j->myLink == 0) {
                std::cout << "Stop at distance " << j->myDistance;
            } else {
                const MSLane* to = j->myLink->getViaLaneOrLane();
                const MSLane* from = j->myLink->getLaneBefore();
                std::cout << "Link at distance " << j->myDistance << ": '"
                          << (from == 0 ? "NONE" : from->getID()) << "' -> '" << (to == 0 ? "NONE" : to->getID()) << "'";
            }
            std::cout << std::endl;
        }
#endif
        if (j->myLink != nullptr) {
            j->myLink->removeApproaching(this);
        }
    }
    myLFLinkLanes.erase(myLFLinkLanes.begin(), myNextDriveItem);
    myNextDriveItem = myLFLinkLanes.begin();
}


void
MSVehicle::updateDriveItems() {
#ifdef DEBUG_ACTIONSTEPS
    if (DEBUG_COND) {
        std::cout << SIMTIME << " updateDriveItems(), veh='" << getID() << "' (lane: '" << getLane()->getID() << "')\nCurrent drive items:" << std::endl;
        for (const auto& dpi : myLFLinkLanes) {
            std::cout
                    << " vPass=" << dpi.myVLinkPass
                    << " vWait=" << dpi.myVLinkWait
                    << " linkLane=" << (dpi.myLink == 0 ? "NULL" : dpi.myLink->getViaLaneOrLane()->getID())
                    << " request=" << dpi.mySetRequest
                    << "\n";
        }
        std::cout << " myNextDriveItem's linked lane: " << (myNextDriveItem->myLink == 0 ? "NULL" : myNextDriveItem->myLink->getViaLaneOrLane()->getID()) << std::endl;
    }
#endif
    if (myLFLinkLanes.size() == 0) {
        // nothing to update
        return;
    }
    const MSLink* nextPlannedLink = nullptr;
//    auto i = myLFLinkLanes.begin();
    auto i = myNextDriveItem;
    while (i != myLFLinkLanes.end() && nextPlannedLink == nullptr) {
        nextPlannedLink = i->myLink;
        ++i;
    }

    if (nextPlannedLink == nullptr) {
        // No link for upcoming item -> no need for an update
#ifdef DEBUG_ACTIONSTEPS
        if (DEBUG_COND) {
            std::cout << "Found no link-related drive item." << std::endl;
        }
#endif
        return;
    }

    if (getLane() == nextPlannedLink->getLaneBefore()) {
        // Current lane approaches the stored next link, i.e. no LC happend and no update is required.
#ifdef DEBUG_ACTIONSTEPS
        if (DEBUG_COND) {
            std::cout << "Continuing on planned lane sequence, no update required." << std::endl;
        }
#endif
        return;
    }
    // Lane must have been changed, determine the change direction
    const MSLink* parallelLink = nextPlannedLink->getParallelLink(1);
    if (parallelLink != nullptr && parallelLink->getLaneBefore() == getLane()) {
        // lcDir = 1;
    } else {
        parallelLink = nextPlannedLink->getParallelLink(-1);
        if (parallelLink != nullptr && parallelLink->getLaneBefore() == getLane()) {
            // lcDir = -1;
        } else {
            // If the vehicle's current lane is not the approaching lane for the next
            // drive process item's link, it is expected to lead to a parallel link,
            // XXX: What if the lc was an overtaking maneuver and there is no upcoming link?
            //      Then a stop item should be scheduled! -> TODO!
            //assert(false);
            return;
        }
    }
#ifdef DEBUG_ACTIONSTEPS
    if (DEBUG_COND) {
        std::cout << "Changed lane. Drive items will be updated along the current lane continuation." << std::endl;
    }
#endif
    // Trace link sequence along current best lanes and transfer drive items to the corresponding links
//        DriveItemVector::iterator driveItemIt = myLFLinkLanes.begin();
    DriveItemVector::iterator driveItemIt = myNextDriveItem;
    // In the loop below, lane holds the currently considered lane on the vehicles continuation (including internal lanes)
    const MSLane* lane = myLane;
    assert(myLane == parallelLink->getLaneBefore());
    // *lit is a pointer to the next lane in best continuations for the current lane (always non-internal)
    std::vector<MSLane*>::const_iterator bestLaneIt = getBestLanesContinuation().begin() + 1;
    // Pointer to the new link for the current drive process item
    MSLink* newLink = nullptr;
    while (driveItemIt != myLFLinkLanes.end()) {
        if (driveItemIt->myLink == nullptr) {
            // Items not related to a specific link are not updated
            // (XXX: when a stop item corresponded to a dead end, which is overcome by the LC that made
            //       the update necessary, this may slow down the vehicle's continuation on the new lane...)
            ++driveItemIt;
            continue;
        }
        // Continuation links for current best lanes are less than for the former drive items (myLFLinkLanes)
        // We just remove the leftover link-items, as they cannot be mapped to new links.
        if (bestLaneIt == getBestLanesContinuation().end()) {
#ifdef DEBUG_ACTIONSTEPS
            if (DEBUG_COND) {
                std::cout << "Reached end of the new continuation sequence. Erasing leftover link-items." << std::endl;
            }
#endif
            while (driveItemIt != myLFLinkLanes.end()) {
                if (driveItemIt->myLink == nullptr) {
                    ++driveItemIt;
                    continue;
                } else {
                    driveItemIt->myLink->removeApproaching(this);
                    driveItemIt = myLFLinkLanes.erase(driveItemIt);
                }
            }
            break;
        }
        // Do the actual link-remapping for the item. And un/register approaching information on the corresponding links
        const MSLane* const target = *bestLaneIt;
        assert(!target->isInternal());
        newLink = nullptr;
        for (MSLink* const link : lane->getLinkCont()) {
            if (link->getLane() == target) {
                newLink = link;
                break;
            }
        }

        if (newLink == driveItemIt->myLink) {
            // new continuation merged into previous - stop update
#ifdef DEBUG_ACTIONSTEPS
            if (DEBUG_COND) {
                std::cout << "Old and new continuation sequences merge at link\n"
                          << "'" << newLink->getLaneBefore()->getID() << "'->'" << newLink->getViaLaneOrLane()->getID() << "'"
                          << "\nNo update beyond merge required." << std::endl;
            }
#endif
            break;
        }

#ifdef DEBUG_ACTIONSTEPS
        if (DEBUG_COND) {
            std::cout << "lane=" << lane->getID() << "\nUpdating link\n    '" << driveItemIt->myLink->getLaneBefore()->getID() << "'->'" << driveItemIt->myLink->getViaLaneOrLane()->getID() << "'"
                      << "==> " << "'" << newLink->getLaneBefore()->getID() << "'->'" << newLink->getViaLaneOrLane()->getID() << "'" << std::endl;
        }
#endif
        newLink->setApproaching(this, driveItemIt->myLink->getApproaching(this));
        driveItemIt->myLink->removeApproaching(this);
        driveItemIt->myLink = newLink;
        lane = newLink->getViaLaneOrLane();
        ++driveItemIt;
        if (!lane->isInternal()) {
            ++bestLaneIt;
        }
    }
#ifdef DEBUG_ACTIONSTEPS
    if (DEBUG_COND) {
        std::cout << "Updated drive items:" << std::endl;
        for (const auto& dpi : myLFLinkLanes) {
            std::cout
                    << " vPass=" << dpi.myVLinkPass
                    << " vWait=" << dpi.myVLinkWait
                    << " linkLane=" << (dpi.myLink == 0 ? "NULL" : dpi.myLink->getViaLaneOrLane()->getID())
                    << " request=" << dpi.mySetRequest
                    << "\n";
        }
    }
#endif
}


void
MSVehicle::setBrakingSignals(double vNext) {
    // To avoid casual blinking brake lights at high speeds due to dawdling of the
    // leading vehicle, we don't show brake lights when the deceleration could be caused
    // by frictional forces and air resistance (i.e. proportional to v^2, coefficient could be adapted further)
    double pseudoFriction = (0.05 +  0.005 * getSpeed()) * getSpeed();
    bool brakelightsOn = vNext < getSpeed() - ACCEL2SPEED(pseudoFriction);

    if (vNext <= SUMO_const_haltingSpeed) {
        brakelightsOn = true;
    }
    if (brakelightsOn && !isStopped()) {
        switchOnSignal(VEH_SIGNAL_BRAKELIGHT);
    } else {
        switchOffSignal(VEH_SIGNAL_BRAKELIGHT);
    }
}


void
MSVehicle::updateWaitingTime(double vNext) {
    if (vNext <= SUMO_const_haltingSpeed && (!isStopped() || isIdling()) && myAcceleration <= accelThresholdForWaiting())  {
        myWaitingTime += DELTA_T;
        myWaitingTimeCollector.passTime(DELTA_T, true);
    } else {
        myWaitingTime = 0;
        myWaitingTimeCollector.passTime(DELTA_T, false);
        if (hasInfluencer()) {
            getInfluencer().setExtraImpatience(0);
        }
    }
}


void
MSVehicle::updateTimeLoss(double vNext) {
    // update time loss (depends on the updated edge)
    if (!isStopped()) {
        const double vmax = myLane->getVehicleMaxSpeed(this);
        if (vmax > 0) {
            myTimeLoss += TS * (vmax - vNext) / vmax;
        }
    }
}


double
MSVehicle::checkReversal(bool& canReverse, double speedThreshold, double seen) const {
    const bool stopOk = (myStops.empty() || myStops.front().edge != myCurrEdge
                         || (myStops.front().getSpeed() > 0 && myState.myPos > myStops.front().pars.endPos - 2 * POSITION_EPS));
#ifdef DEBUG_REVERSE_BIDI
    if (DEBUG_COND) std::cout << SIMTIME  << " checkReversal lane=" << myLane->getID()
                                  << " pos=" << myState.myPos
                                  << " speed=" << std::setprecision(6) << getPreviousSpeed() << std::setprecision(gPrecision)
                                  << " speedThreshold=" << speedThreshold
                                  << " seen=" << seen
                                  << " isRail=" << ((getVClass() & SVC_RAIL_CLASSES) != 0)
                                  << " speedOk=" << (getPreviousSpeed() <= speedThreshold)
                                  << " posOK=" << (myState.myPos <= myLane->getLength())
                                  << " normal=" << !myLane->isInternal()
                                  << " routeOK=" << ((myCurrEdge + 1) != myRoute->end())
                                  << " bidi=" << (myLane->getEdge().getBidiEdge() == *(myCurrEdge + 1))
                                  << " stopOk=" << stopOk
                                  << "\n";
#endif
    if ((getVClass() & SVC_RAIL_CLASSES) != 0
            && getPreviousSpeed() <= speedThreshold
            && myState.myPos <= myLane->getLength()
            && !myLane->isInternal()
            && (myCurrEdge + 1) != myRoute->end()
            && myLane->getEdge().getBidiEdge() == *(myCurrEdge + 1)
            // ensure there are no further stops on this edge
            && stopOk
       ) {
        //if (isSelected()) std::cout << "   check1 passed\n";

        // ensure that the vehicle is fully on bidi edges that allow reversal
        const int neededFutureRoute = 1 + (int)(MSGlobals::gUsingInternalLanes
                                                ? myFurtherLanes.size()
                                                : ceil((double)myFurtherLanes.size() / 2.0));
        const int remainingRoute = int(myRoute->end() - myCurrEdge) - 1;
        if (remainingRoute < neededFutureRoute) {
#ifdef DEBUG_REVERSE_BIDI
            if (DEBUG_COND) {
                std::cout << "    fail: remainingEdges=" << ((int)(myRoute->end() - myCurrEdge)) << " further=" << myFurtherLanes.size() << "\n";
            }
#endif
            return getMaxSpeed();
        }
        //if (isSelected()) std::cout << "   check2 passed\n";

        // ensure that the turn-around connection exists from the current edge to its bidi-edge
        const MSEdgeVector& succ = myLane->getEdge().getSuccessors();
        if (std::find(succ.begin(), succ.end(), myLane->getEdge().getBidiEdge()) == succ.end()) {
#ifdef DEBUG_REVERSE_BIDI
            if (DEBUG_COND) {
                std::cout << "    noTurn (bidi=" << myLane->getEdge().getBidiEdge()->getID() << " succ=" << toString(succ) << "\n";
            }
#endif
            return getMaxSpeed();
        }
        //if (isSelected()) std::cout << "   check3 passed\n";

        // ensure that the vehicle front will not move past a stop on the bidi edge of the current edge
        if (!myStops.empty() && myStops.front().edge == (myCurrEdge + 1)) {
            const double stopPos = myStops.front().getEndPos(*this);
            const double brakeDist = getCarFollowModel().brakeGap(getSpeed(), getCarFollowModel().getMaxDecel(), 0);
            const double newPos = myLane->getLength() - (getBackPositionOnLane() + brakeDist);
            if (newPos > stopPos) {
#ifdef DEBUG_REVERSE_BIDI
                if (DEBUG_COND) {
                    std::cout << "    reversal would go past stop on " << myLane->getBidiLane()->getID() << "\n";
                }
#endif
                if (seen > MAX2(brakeDist, 1.0)) {
                    return getMaxSpeed();
                } else {
#ifdef DEBUG_REVERSE_BIDI
                    if (DEBUG_COND) {
                        std::cout << "    train is too long, skipping stop at " << stopPos << " cannot be avoided\n";
                    }
#endif
                }
            }
        }
        //if (isSelected()) std::cout << "   check4 passed\n";

        // ensure that bidi-edges exist for all further edges
        // and that no stops will be skipped when reversing
        // and that the train will not be on top of a red rail signal after reversal
        const MSLane* bidi = myLane->getBidiLane();
        int view = 2;
        for (MSLane* further : myFurtherLanes) {
            if (!further->getEdge().isInternal()) {
                if (further->getEdge().getBidiEdge() != *(myCurrEdge + view)) {
#ifdef DEBUG_REVERSE_BIDI
                    if (DEBUG_COND) {
                        std::cout << "    noBidi view=" << view << " further=" << further->getID() << " furtherBidi=" << Named::getIDSecure(further->getEdge().getBidiEdge()) << " future=" << (*(myCurrEdge + view))->getID() << "\n";
                    }
#endif
                    return getMaxSpeed();
                }
                const MSLane* nextBidi = further->getBidiLane();
                const MSLink* toNext = bidi->getLinkTo(nextBidi);
                if (toNext == nullptr) {
                    // can only happen if the route is invalid
                    return getMaxSpeed();
                }
                if (toNext->haveRed()) {
#ifdef DEBUG_REVERSE_BIDI
                    if (DEBUG_COND) {
                        std::cout << "    do not reverse on a red signal\n";
                    }
#endif
                    return getMaxSpeed();
                }
                bidi = nextBidi;
                if (!myStops.empty() && myStops.front().edge == (myCurrEdge + view)) {
                    const double brakeDist = getCarFollowModel().brakeGap(getSpeed(), getCarFollowModel().getMaxDecel(), 0);
                    const double stopPos = myStops.front().getEndPos(*this);
                    const double newPos = further->getLength() - (getBackPositionOnLane(further) + brakeDist);
                    if (newPos > stopPos) {
#ifdef DEBUG_REVERSE_BIDI
                        if (DEBUG_COND) {
                            std::cout << "    reversal would go past stop on further-opposite lane " << further->getBidiLane()->getID() << "\n";
                        }
#endif
                        if (seen > MAX2(brakeDist, 1.0)) {
                            canReverse = false;
                            return getMaxSpeed();
                        } else {
#ifdef DEBUG_REVERSE_BIDI
                            if (DEBUG_COND) {
                                std::cout << "    train is too long, skipping stop at " << stopPos << " cannot be avoided\n";
                            }
#endif
                        }
                    }
                }
                view++;
            }
        }
        // reverse as soon as comfortably possible
        const double vMinComfortable = getCarFollowModel().minNextSpeed(getSpeed(), this);
#ifdef DEBUG_REVERSE_BIDI
        if (DEBUG_COND) {
            std::cout << SIMTIME << " seen=" << seen  << " vReverseOK=" << vMinComfortable << "\n";
        }
#endif
        canReverse = true;
        return vMinComfortable;
    }
    return getMaxSpeed();
}


void
MSVehicle::processLaneAdvances(std::vector<MSLane*>& passedLanes, std::string& emergencyReason) {
    for (std::vector<MSLane*>::reverse_iterator i = myFurtherLanes.rbegin(); i != myFurtherLanes.rend(); ++i) {
        passedLanes.push_back(*i);
    }
    if (passedLanes.size() == 0 || passedLanes.back() != myLane) {
        passedLanes.push_back(myLane);
    }
    // let trains reverse direction
    bool reverseTrain = false;
    checkReversal(reverseTrain);
    if (reverseTrain) {
        // Train is 'reversing' so toggle the logical state
        myAmReversed = !myAmReversed;
        // add some slack to ensure that the back of train does appear looped
        myState.myPos += 2 * (myLane->getLength() - myState.myPos) + myType->getLength() + NUMERICAL_EPS;
        myState.mySpeed = 0;
#ifdef DEBUG_REVERSE_BIDI
        if (DEBUG_COND) {
            std::cout << SIMTIME << " reversing train=" << getID() << " newPos=" << myState.myPos << "\n";
        }
#endif
    }
    // move on lane(s)
    if (myState.myPos > myLane->getLength()) {
        // The vehicle has moved at least to the next lane (maybe it passed even more than one)
        if (myCurrEdge != myRoute->end() - 1) {
            MSLane* approachedLane = myLane;
            // move the vehicle forward
            myNextDriveItem = myLFLinkLanes.begin();
            while (myNextDriveItem != myLFLinkLanes.end() && approachedLane != nullptr && myState.myPos > approachedLane->getLength()) {
                const MSLink* link = myNextDriveItem->myLink;
                const double linkDist = myNextDriveItem->myDistance;
                ++myNextDriveItem;
                // check whether the vehicle was allowed to enter lane
                //  otherwise it is decelerated and we do not need to test for it's
                //  approach on the following lanes when a lane changing is performed
                // proceed to the next lane
                if (approachedLane->mustCheckJunctionCollisions()) {
                    // vehicle moves past approachedLane within a single step, collision checking must still be done
                    MSNet::getInstance()->getEdgeControl().checkCollisionForInactive(approachedLane);
                }
                if (link != nullptr) {
                    if ((getVClass() & SVC_RAIL_CLASSES) != 0
                            && !myLane->isInternal()
                            && myLane->getBidiLane() != nullptr
                            && link->getLane()->getBidiLane() == myLane
                            && !reverseTrain) {
                        emergencyReason = " because it must reverse direction";
                        approachedLane = nullptr;
                        break;
                    }
                    if ((getVClass() & SVC_RAIL_CLASSES) != 0
                            && myState.myPos < myLane->getLength() + NUMERICAL_EPS
                            && hasStops() && getNextStop().edge == myCurrEdge) {
                        // avoid skipping stop due to numerical instability
                        // this is a special case for rail vehicles because they
                        // continue myLFLinkLanes past stops
                        approachedLane = myLane;
                        myState.myPos = myLane->getLength();
                        break;
                    }
                    approachedLane = link->getViaLaneOrLane();
                    if (myInfluencer == nullptr || myInfluencer->getEmergencyBrakeRedLight()) {
                        bool beyondStopLine = linkDist < link->getLaneBefore()->getVehicleStopOffset(this);
                        if (link->haveRed() && !ignoreRed(link, false) && !beyondStopLine && !reverseTrain) {
                            emergencyReason = " because of a red traffic light";
                            break;
                        }
                    }
                    if (reverseTrain && approachedLane->isInternal()) {
                        // avoid getting stuck on a slow turn-around internal lane
                        myState.myPos += approachedLane->getLength();
                    }
                } else if (myState.myPos < myLane->getLength() + NUMERICAL_EPS) {
                    // avoid warning due to numerical instability
                    approachedLane = myLane;
                    myState.myPos = myLane->getLength();
                } else if (reverseTrain) {
                    approachedLane = (*(myCurrEdge + 1))->getLanes()[0];
                    link = myLane->getLinkTo(approachedLane);
                    assert(link != 0);
                    while (link->getViaLane() != nullptr) {
                        link = link->getViaLane()->getLinkCont()[0];
                    }
                    --myNextDriveItem;
                } else {
                    emergencyReason = " because there is no connection to the next edge";
                    approachedLane = nullptr;
                    break;
                }
                if (approachedLane != myLane && approachedLane != nullptr) {
                    leaveLane(MSMoveReminder::NOTIFICATION_JUNCTION, approachedLane);
                    myState.myPos -= myLane->getLength();
                    assert(myState.myPos > 0);
                    enterLaneAtMove(approachedLane);
                    if (link->isEntryLink()) {
                        myJunctionEntryTime = MSNet::getInstance()->getCurrentTimeStep();
                        myJunctionEntryTimeNeverYield = myJunctionEntryTime;
                    }
                    if (link->isConflictEntryLink()) {
                        myJunctionConflictEntryTime = MSNet::getInstance()->getCurrentTimeStep();
                        // renew yielded request
                        myJunctionEntryTime = myJunctionEntryTimeNeverYield;
                    }
                    if (link->isExitLink()) {
                        // passed junction, reset for approaching the next one
                        myJunctionEntryTime = SUMOTime_MAX;
                        myJunctionEntryTimeNeverYield = SUMOTime_MAX;
                        myJunctionConflictEntryTime = SUMOTime_MAX;
                    }
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
                    if (DEBUG_COND) {
                        std::cout << "Update junctionTimes link=" << link->getViaLaneOrLane()->getID()
                                  << " entry=" << link->isEntryLink() << " conflict=" << link->isConflictEntryLink() << " exit=" << link->isExitLink()
                                  << " ET=" << myJunctionEntryTime
                                  << " ETN=" << myJunctionEntryTimeNeverYield
                                  << " CET=" << myJunctionConflictEntryTime
                                  << "\n";
                    }
#endif
                    if (hasArrivedInternal()) {
                        break;
                    }
                    if (myLaneChangeModel->isChangingLanes()) {
                        if (link->getDirection() == LinkDirection::LEFT || link->getDirection() == LinkDirection::RIGHT) {
                            // abort lane change
                            WRITE_WARNING("Vehicle '" + getID() + "' could not finish continuous lane change (turn lane) time=" +
                                          time2string(MSNet::getInstance()->getCurrentTimeStep()) + ".");
                            myLaneChangeModel->endLaneChangeManeuver();
                        }
                    }
                    if (approachedLane->getEdge().isVaporizing()) {
                        leaveLane(MSMoveReminder::NOTIFICATION_VAPORIZED_VAPORIZER);
                        break;
                    }
                    passedLanes.push_back(approachedLane);
                }
            }
            // NOTE: Passed drive items will be erased in the next simstep's planMove()

#ifdef DEBUG_ACTIONSTEPS
            if (DEBUG_COND && myNextDriveItem != myLFLinkLanes.begin()) {
                std::cout << "Updated drive items:" << std::endl;
                for (DriveItemVector::iterator i = myLFLinkLanes.begin(); i != myLFLinkLanes.end(); ++i) {
                    std::cout
                            << " vPass=" << (*i).myVLinkPass
                            << " vWait=" << (*i).myVLinkWait
                            << " linkLane=" << ((*i).myLink == 0 ? "NULL" : (*i).myLink->getViaLaneOrLane()->getID())
                            << " request=" << (*i).mySetRequest
                            << "\n";
                }
            }
#endif
        } else if (!hasArrivedInternal() && myState.myPos < myLane->getLength() + NUMERICAL_EPS) {
            // avoid warning due to numerical instability when stopping at the end of the route
            myState.myPos = myLane->getLength();
        }

    }
}



bool
MSVehicle::executeMove() {
#ifdef DEBUG_EXEC_MOVE
    if (DEBUG_COND) {
        std::cout << "\nEXECUTE_MOVE\n"
                  << SIMTIME
                  << " veh=" << getID()
                  << " speed=" << getSpeed() // toString(getSpeed(), 24)
                  << std::endl;
    }
#endif


    // Maximum safe velocity
    double vSafe = std::numeric_limits<double>::max();
    // Minimum safe velocity (lower bound).
    double vSafeMin = -std::numeric_limits<double>::max();
    // The distance to a link, which should either be crossed this step
    // or in front of which we need to stop.
    double vSafeMinDist = 0;

    if (myActionStep) {
        // Actuate control (i.e. choose bounds for safe speed in current simstep (euler), resp. after current sim step (ballistic))
        processLinkApproaches(vSafe, vSafeMin, vSafeMinDist);
#ifdef DEBUG_ACTIONSTEPS
        if (DEBUG_COND) {
            std::cout << SIMTIME << " vehicle '" << getID() << "'\n"
                      "   vsafe from processLinkApproaches(): vsafe " << vSafe << std::endl;
        }
#endif
    } else {
        // Continue with current acceleration
        vSafe = getSpeed() + ACCEL2SPEED(myAcceleration);
#ifdef DEBUG_ACTIONSTEPS
        if (DEBUG_COND) {
            std::cout << SIMTIME << " vehicle '" << getID() << "' skips processLinkApproaches()\n"
                      "   continues with constant accel " <<  myAcceleration << "...\n"
                      << "speed: "  << getSpeed() << " -> " << vSafe << std::endl;
        }
#endif
    }


//#ifdef DEBUG_EXEC_MOVE
//    if (DEBUG_COND) {
//        std::cout << "vSafe = " << toString(vSafe,12) << "\n" << std::endl;
//    }
//#endif

    // Determine vNext = speed after current sim step (ballistic), resp. in current simstep (euler)
    // Call to finalizeSpeed applies speed reduction due to dawdling / lane changing but ensures minimum safe speed
    double vNext = vSafe;
    const double rawAccel = SPEED2ACCEL(MAX2(vNext, 0.) - myState.mySpeed);
    if (vNext <= SUMO_const_haltingSpeed * TS && myWaitingTime > MSGlobals::gStartupWaitThreshold && rawAccel <= accelThresholdForWaiting() && myActionStep) {
        myTimeSinceStartup = 0;
    } else if (isStopped()) {
        // do not apply startupDelay for waypoints
        if (getCarFollowModel().startupDelayStopped() && getNextStop().pars.speed <= 0) {
            myTimeSinceStartup = DELTA_T;
        } else {
            // do not apply startupDelay but signal that a stop has taken place
            myTimeSinceStartup = getCarFollowModel().getStartupDelay() + DELTA_T;
        }
    } else {
        // identify potential startup (before other effects reduce the speed again)
        myTimeSinceStartup += DELTA_T;
    }
    if (myActionStep) {
        vNext = getCarFollowModel().finalizeSpeed(this, vSafe);
        if (vNext > 0) {
            vNext = MAX2(vNext, vSafeMin);
        }
    }
    // (Leo) to avoid tiny oscillations (< 1e-10) of vNext in a standing vehicle column (observed for ballistic update), we cap off vNext
    //       (We assure to do this only for vNext<<NUMERICAL_EPS since otherwise this would nullify the workaround for #2995
    // (Jakob) We also need to make sure to reach a stop at the start of the next edge
    if (fabs(vNext) < NUMERICAL_EPS_SPEED && myStopDist > POSITION_EPS) {
        vNext = 0.;
    }
#ifdef DEBUG_EXEC_MOVE
    if (DEBUG_COND) {
        std::cout << SIMTIME << " finalizeSpeed vSafe=" << vSafe << " vSafeMin=" << (vSafeMin == -std::numeric_limits<double>::max() ? "-Inf" : toString(vSafeMin))
                  << " vNext=" << vNext << " (i.e. accel=" << SPEED2ACCEL(vNext - getSpeed()) << ")" << std::endl;
    }
#endif

    // vNext may be higher than vSafe without implying a bug:
    //  - when approaching a green light that suddenly switches to yellow
    //  - when using unregulated junctions
    //  - when using tau < step-size
    //  - when using unsafe car following models
    //  - when using TraCI and some speedMode / laneChangeMode settings
    //if (vNext > vSafe + NUMERICAL_EPS) {
    //    WRITE_WARNING("vehicle '" + getID() + "' cannot brake hard enough to reach safe speed "
    //            + toString(vSafe, 4) + ", moving at " + toString(vNext, 4) + " instead. time="
    //            + time2string(MSNet::getInstance()->getCurrentTimeStep()) + ".");
    //}

    if (MSGlobals::gSemiImplicitEulerUpdate) {
        vNext = MAX2(vNext, 0.);
    } else {
        // (Leo) Ballistic: negative vNext can be used to indicate a stop within next step.
    }

    // Check for speed advices from the traci client
    vNext = processTraCISpeedControl(vSafe, vNext);

    // the acceleration of a vehicle equipped with the elecHybrid device is restricted by the maximal power of the electric drive as well
    MSDevice_ElecHybrid* elecHybridOfVehicle = dynamic_cast<MSDevice_ElecHybrid*>(getDevice(typeid(MSDevice_ElecHybrid)));
    if (elecHybridOfVehicle != nullptr) {
        // this is the consumption given by the car following model-computed acceleration
        elecHybridOfVehicle->setConsum(elecHybridOfVehicle->consumption(*this, (vNext - this->getSpeed()) / TS, vNext));
        // but the maximum power of the electric motor may be lower
        // it needs to be converted from [W] to [Wh/s] (3600s / 1h) so that TS can be taken into account
        double maxPower = elecHybridOfVehicle->getParameterDouble(toString(SUMO_ATTR_MAXIMUMPOWER)) / 3600;
        if (elecHybridOfVehicle->getConsum() / TS > maxPower) {
            // no, we cannot accelerate that fast, recompute the maximum possible acceleration
            double accel = elecHybridOfVehicle->acceleration(*this, maxPower, this->getSpeed());
            // and update the speed of the vehicle
            vNext = MIN2(vNext, this->getSpeed() + accel * TS);
            vNext = MAX2(vNext, 0.);
            // and set the vehicle consumption to reflect this
            elecHybridOfVehicle->setConsum(elecHybridOfVehicle->consumption(*this, (vNext - this->getSpeed()) / TS, vNext));
        }
    }

    setBrakingSignals(vNext);

    // update position and speed
    int oldLaneOffset = myLane->getEdge().getNumLanes() - myLane->getIndex();
    const MSLane* oldLaneMaybeOpposite = myLane;
    if (myLaneChangeModel->isOpposite()) {
        // transform to the forward-direction lane, move and then transform back
        myState.myPos = myLane->getOppositePos(myState.myPos);
        myLane = myLane->getParallelOpposite();
    }
    updateState(vNext);
    updateWaitingTime(vNext);

    // Lanes, which the vehicle touched at some moment of the executed simstep
    std::vector<MSLane*> passedLanes;
    // remember previous lane (myLane is updated in processLaneAdvances)
    const MSLane* oldLane = myLane;
    // Reason for a possible emergency stop
    std::string emergencyReason;
    processLaneAdvances(passedLanes, emergencyReason);

    updateTimeLoss(vNext);
    myCollisionImmunity = MAX2((SUMOTime) - 1, myCollisionImmunity - DELTA_T);

    if (!hasArrivedInternal() && !myLane->getEdge().isVaporizing()) {
        if (myState.myPos > myLane->getLength()) {
            if (emergencyReason == "") {
                emergencyReason = TL(" for unknown reasons");
            }
            WRITE_WARNINGF(TL("Vehicle '%' performs emergency stop at the end of lane '%'% (decel=%, offset=%), time=%."),
                           getID(), myLane->getID(), emergencyReason, myAcceleration - myState.mySpeed,
                           myState.myPos - myLane->getLength(), time2string(SIMSTEP));
            MSNet::getInstance()->getVehicleControl().registerEmergencyStop();
            MSNet::getInstance()->informVehicleStateListener(this, MSNet::VehicleState::EMERGENCYSTOP);
            myState.myPos = myLane->getLength();
            myState.mySpeed = 0;
            myAcceleration = 0;
        }
        const MSLane* oldBackLane = getBackLane();
        if (myLaneChangeModel->isOpposite()) {
            passedLanes.clear(); // ignore back occupation
        }
#ifdef DEBUG_ACTIONSTEPS
        if (DEBUG_COND) {
            std::cout << SIMTIME << " veh '" << getID() << "' updates further lanes." << std::endl;
        }
#endif
        myState.myBackPos = updateFurtherLanes(myFurtherLanes, myFurtherLanesPosLat, passedLanes);
        if (passedLanes.size() > 1 && isRailway(getVClass())) {
            for (auto pi = passedLanes.rbegin(); pi != passedLanes.rend(); ++pi) {
                MSLane* pLane = *pi;
                if (pLane != myLane && std::find(myFurtherLanes.begin(), myFurtherLanes.end(), pLane) == myFurtherLanes.end()) {
                    leaveLaneBack(MSMoveReminder::NOTIFICATION_JUNCTION, *pi);
                }
            }
        }
        // bestLanes need to be updated before lane changing starts. NOTE: This call is also a presumption for updateDriveItems()
        updateBestLanes();
        if (myLane != oldLane || oldBackLane != getBackLane()) {
            if (myLaneChangeModel->getShadowLane() != nullptr || getLateralOverlap() > POSITION_EPS) {
                // shadow lane must be updated if the front or back lane changed
                // either if we already have a shadowLane or if there is lateral overlap
                myLaneChangeModel->updateShadowLane();
            }
            if (MSGlobals::gLateralResolution > 0 && !myLaneChangeModel->isOpposite()) {
                // The vehicles target lane must be also be updated if the front or back lane changed
                myLaneChangeModel->updateTargetLane();
            }
        }
        setBlinkerInformation(); // needs updated bestLanes
        //change the blue light only for emergency vehicles SUMOVehicleClass
        if (myType->getVehicleClass() == SVC_EMERGENCY) {
            setEmergencyBlueLight(MSNet::getInstance()->getCurrentTimeStep());
        }
        // must be done before angle computation
        // State needs to be reset for all vehicles before the next call to MSEdgeControl::changeLanes
        if (myActionStep) {
            // check (#2681): Can this be skipped?
            myLaneChangeModel->prepareStep();
        } else {
            myLaneChangeModel->resetSpeedLat();
#ifdef DEBUG_ACTIONSTEPS
            if (DEBUG_COND) {
                std::cout << SIMTIME << " veh '" << getID() << "' skips LCM->prepareStep()." << std::endl;
            }
#endif
        }
        myLaneChangeModel->setPreviousAngleOffset(myLaneChangeModel->getAngleOffset());
        myAngle = computeAngle();
    }

#ifdef DEBUG_EXEC_MOVE
    if (DEBUG_COND) {
        std::cout << SIMTIME << " executeMove finished veh=" << getID() << " lane=" << myLane->getID() << " myPos=" << getPositionOnLane() << " myPosLat=" << getLateralPositionOnLane() << "\n";
        gDebugFlag1 = false; // See MSLink_DEBUG_OPENED
    }
#endif
    if (myLaneChangeModel->isOpposite()) {
        // transform back to the opposite-direction lane
        MSLane* newOpposite = nullptr;
        const MSEdge* newOppositeEdge = myLane->getEdge().getOppositeEdge();
        if (newOppositeEdge != nullptr) {
            newOpposite = newOppositeEdge->getLanes()[newOppositeEdge->getNumLanes() - MAX2(1, oldLaneOffset)];
#ifdef DEBUG_EXEC_MOVE
            if (DEBUG_COND) {
                std::cout << SIMTIME << "   newOppositeEdge=" << newOppositeEdge->getID() << " oldLaneOffset=" << oldLaneOffset << " leftMost=" << newOppositeEdge->getNumLanes() - 1 << " newOpposite=" << Named::getIDSecure(newOpposite) << "\n";
            }
#endif
        }
        if (newOpposite == nullptr) {
            if (!myLaneChangeModel->hasBlueLight()) {
                // unusual overtaking at junctions is ok for emergency vehicles
                WRITE_WARNINGF(TL("Unexpected end of opposite lane for vehicle '%' at lane '%', time=%."),
                               getID(), myLane->getID(), time2string(SIMSTEP));
            }
            myLaneChangeModel->changedToOpposite();
            if (myState.myPos < getLength()) {
                // further lanes is always cleared during opposite driving
                MSLane* oldOpposite = oldLane->getOpposite();
                if (oldOpposite != nullptr) {
                    myFurtherLanes.push_back(oldOpposite);
                    myFurtherLanesPosLat.push_back(0);
                    // small value since the lane is going in the other direction
                    myState.myBackPos = getLength() - myState.myPos;
                    myAngle = computeAngle();
                } else {
                    SOFT_ASSERT(false);
                }
            }
        } else {
            myState.myPos = myLane->getOppositePos(myState.myPos);
            myLane = newOpposite;
            oldLane = oldLaneMaybeOpposite;
            //std::cout << SIMTIME << " updated myLane=" << Named::getIDSecure(myLane) << " oldLane=" << oldLane->getID() << "\n";
            myCachedPosition = Position::INVALID;
            myLaneChangeModel->updateShadowLane();
        }
    }
    workOnMoveReminders(myState.myPos - myState.myLastCoveredDist, myState.myPos, myState.mySpeed);
    // Return whether the vehicle did move to another lane
    return myLane != oldLane;
}

void
MSVehicle::executeFractionalMove(double dist) {
    myState.myPos += dist;
    myState.myLastCoveredDist = dist;
    myCachedPosition = Position::INVALID;

    const std::vector<const MSLane*> lanes = getUpcomingLanesUntil(dist);
    const SUMOTime t = MSNet::getInstance()->getCurrentTimeStep();
    for (int i = 0; i < (int)lanes.size(); i++) {
        MSLink* link = nullptr;
        if (i + 1 < (int)lanes.size()) {
            const MSLane* const to = lanes[i + 1];
            const bool internal = to->isInternal();
            for (MSLink* const l : lanes[i]->getLinkCont()) {
                if ((internal && l->getViaLane() == to) || (!internal && l->getLane() == to)) {
                    link = l;
                    break;
                }
            }
        }
        myLFLinkLanes.emplace_back(link, getSpeed(), getSpeed(), true, t, getSpeed(), 0, 0, dist);
    }
    // minimum execute move:
    std::vector<MSLane*> passedLanes;
    // Reason for a possible emergency stop
    if (lanes.size() > 1) {
        myLane->removeVehicle(this, MSMoveReminder::NOTIFICATION_JUNCTION, false);
    }
    std::string emergencyReason;
    processLaneAdvances(passedLanes, emergencyReason);
#ifdef DEBUG_EXTRAPOLATE_DEPARTPOS
    if (DEBUG_COND) {
        std::cout << SIMTIME << " veh=" << getID() << " executeFractionalMove dist=" << dist
                  << " passedLanes=" << toString(passedLanes) << " lanes=" << toString(lanes)
                  << " finalPos=" << myState.myPos
                  << " speed=" << getSpeed()
                  << " myFurtherLanes=" << toString(myFurtherLanes)
                  << "\n";
    }
#endif
    workOnMoveReminders(myState.myPos - myState.myLastCoveredDist, myState.myPos, myState.mySpeed);
    if (lanes.size() > 1) {
        for (std::vector<MSLane*>::iterator i = myFurtherLanes.begin(); i != myFurtherLanes.end(); ++i) {
#ifdef DEBUG_FURTHER
            if (DEBUG_COND) {
                std::cout << SIMTIME << " leaveLane \n";
            }
#endif
            (*i)->resetPartialOccupation(this);
        }
        myFurtherLanes.clear();
        myFurtherLanesPosLat.clear();
        myLane->forceVehicleInsertion(this, getPositionOnLane(), MSMoveReminder::NOTIFICATION_JUNCTION, getLateralPositionOnLane());
    }
}


void
MSVehicle::updateState(double vNext) {
    // update position and speed
    double deltaPos; // positional change
    if (MSGlobals::gSemiImplicitEulerUpdate) {
        // euler
        deltaPos = SPEED2DIST(vNext);
    } else {
        // ballistic
        deltaPos = getDeltaPos(SPEED2ACCEL(vNext - myState.mySpeed));
    }

    // the *mean* acceleration during the next step (probably most appropriate for emission calculation)
    // NOTE: for the ballistic update vNext may be negative, indicating a stop.
    myAcceleration = SPEED2ACCEL(MAX2(vNext, 0.) - myState.mySpeed);

#ifdef DEBUG_EXEC_MOVE
    if (DEBUG_COND) {
        std::cout << SIMTIME << " updateState() for veh '" << getID() << "': deltaPos=" << deltaPos
                  << " pos=" << myState.myPos << " newPos=" << myState.myPos + deltaPos << std::endl;
    }
#endif
    double decelPlus = -myAcceleration - getCarFollowModel().getMaxDecel() - NUMERICAL_EPS;
    if (decelPlus > 0) {
        const double previousAcceleration = SPEED2ACCEL(myState.mySpeed - myState.myPreviousSpeed);
        if (myAcceleration + NUMERICAL_EPS < previousAcceleration) {
            // vehicle brakes beyond wished maximum deceleration (only warn at the start of the braking manoeuvre)
            decelPlus += 2 * NUMERICAL_EPS;
            const double emergencyFraction = decelPlus / MAX2(NUMERICAL_EPS, getCarFollowModel().getEmergencyDecel() - getCarFollowModel().getMaxDecel());
            if (emergencyFraction >= MSGlobals::gEmergencyDecelWarningThreshold) {
                WRITE_WARNINGF(TL("Vehicle '%' performs emergency braking on lane '%' with decel=%, wished=%, severity=%, time=%."),
                               //+ " decelPlus=" + toString(decelPlus)
                               //+ " prevAccel=" + toString(previousAcceleration)
                               //+ " reserve=" + toString(MAX2(NUMERICAL_EPS, getCarFollowModel().getEmergencyDecel() - getCarFollowModel().getMaxDecel()))
                               getID(), myLane->getID(), -myAcceleration, getCarFollowModel().getMaxDecel(), emergencyFraction, time2string(SIMSTEP));
                MSNet::getInstance()->getVehicleControl().registerEmergencyBraking();
            }
        }
    }

    myState.myPreviousSpeed = myState.mySpeed;
    myState.mySpeed = MAX2(vNext, 0.);

    if (isRemoteControlled()) {
        deltaPos = myInfluencer->implicitDeltaPosRemote(this);
    }

    myState.myPos += deltaPos;
    myState.myLastCoveredDist = deltaPos;
    myNextTurn.first -= deltaPos;

    myCachedPosition = Position::INVALID;
}

void
MSVehicle::updateParkingState() {
    updateState(0);
    // deboard while parked
    if (myPersonDevice != nullptr) {
        myPersonDevice->notifyMove(*this, getPositionOnLane(), getPositionOnLane(), 0);
    }
    if (myContainerDevice != nullptr) {
        myContainerDevice->notifyMove(*this, getPositionOnLane(), getPositionOnLane(), 0);
    }
    for (MSVehicleDevice* const dev : myDevices) {
        dev->notifyParking();
    }
}


void
MSVehicle::replaceVehicleType(MSVehicleType* type) {
    MSBaseVehicle::replaceVehicleType(type);
    delete myCFVariables;
    myCFVariables = type->getCarFollowModel().createVehicleVariables();
}


const MSLane*
MSVehicle::getBackLane() const {
    if (myFurtherLanes.size() > 0) {
        return myFurtherLanes.back();
    } else {
        return myLane;
    }
}


double
MSVehicle::updateFurtherLanes(std::vector<MSLane*>& furtherLanes, std::vector<double>& furtherLanesPosLat,
                              const std::vector<MSLane*>& passedLanes) {
#ifdef DEBUG_SETFURTHER
    if (DEBUG_COND) std::cout << SIMTIME << " veh=" << getID()
                                  << " updateFurtherLanes oldFurther=" << toString(furtherLanes)
                                  << " oldFurtherPosLat=" << toString(furtherLanesPosLat)
                                  << " passed=" << toString(passedLanes)
                                  << "\n";
#endif
    for (MSLane* further : furtherLanes) {
        further->resetPartialOccupation(this);
        if (further->getBidiLane() != nullptr
                && (!isRailway(getVClass()) || (further->getPermissions() & ~SVC_RAIL_CLASSES) != 0)) {
            further->getBidiLane()->resetPartialOccupation(this);
        }
    }

    std::vector<MSLane*> newFurther;
    std::vector<double> newFurtherPosLat;
    double backPosOnPreviousLane = myState.myPos - getLength();
    bool widthShift = myFurtherLanesPosLat.size() > myFurtherLanes.size();
    if (passedLanes.size() > 1) {
        // There are candidates for further lanes. (passedLanes[-1] is the current lane, or current shadow lane in context of updateShadowLanes())
        std::vector<MSLane*>::const_iterator fi = furtherLanes.begin();
        std::vector<double>::const_iterator fpi = furtherLanesPosLat.begin();
        for (auto pi = passedLanes.rbegin() + 1; pi != passedLanes.rend() && backPosOnPreviousLane < 0; ++pi) {
            // As long as vehicle back reaches into passed lane, add it to the further lanes
            MSLane* further = *pi;
            newFurther.push_back(further);
            backPosOnPreviousLane += further->setPartialOccupation(this);
            if (further->getBidiLane() != nullptr
                    && (!isRailway(getVClass()) || (further->getPermissions() & ~SVC_RAIL_CLASSES) != 0)) {
                further->getBidiLane()->setPartialOccupation(this);
            }
            if (fi != furtherLanes.end() && further == *fi) {
                // Lateral position on this lane is already known. Assume constant and use old value.
                newFurtherPosLat.push_back(*fpi);
                ++fi;
                ++fpi;
            } else {
                // The lane *pi was not in furtherLanes before.
                // If it is downstream, we assume as lateral position the current position
                // If it is a new lane upstream (can appear as shadow further in case of LC-maneuvering, e.g.)
                // we assign the last known lateral position.
                if (newFurtherPosLat.size() == 0) {
                    if (widthShift) {
                        newFurtherPosLat.push_back(myFurtherLanesPosLat.back());
                    } else {
                        newFurtherPosLat.push_back(myState.myPosLat);
                    }
                } else {
                    newFurtherPosLat.push_back(newFurtherPosLat.back());
                }
            }
#ifdef DEBUG_SETFURTHER
            if (DEBUG_COND) {
                std::cout << SIMTIME << " updateFurtherLanes \n"
                          << "    further lane '" << further->getID() << "' backPosOnPreviousLane=" << backPosOnPreviousLane
                          << std::endl;
            }
#endif
        }
        furtherLanes = newFurther;
        furtherLanesPosLat = newFurtherPosLat;
    } else {
        furtherLanes.clear();
        furtherLanesPosLat.clear();
    }
#ifdef DEBUG_SETFURTHER
    if (DEBUG_COND) std::cout
                << " newFurther=" << toString(furtherLanes)
                << " newFurtherPosLat=" << toString(furtherLanesPosLat)
                << " newBackPos=" << backPosOnPreviousLane
                << "\n";
#endif
    return backPosOnPreviousLane;
}


double
MSVehicle::getBackPositionOnLane(const MSLane* lane, bool calledByGetPosition) const {
#ifdef DEBUG_FURTHER
    if (DEBUG_COND) {
        std::cout << SIMTIME
                  << " getBackPositionOnLane veh=" << getID()
                  << " lane=" << Named::getIDSecure(lane)
                  << " cbgP=" << calledByGetPosition
                  << " pos=" << myState.myPos
                  << " backPos=" << myState.myBackPos
                  << " myLane=" << myLane->getID()
                  << " myLaneBidi=" << Named::getIDSecure(myLane->getBidiLane())
                  << " further=" << toString(myFurtherLanes)
                  << " furtherPosLat=" << toString(myFurtherLanesPosLat)
                  << "\n     shadowLane=" << Named::getIDSecure(myLaneChangeModel->getShadowLane())
                  << " shadowFurther=" << toString(myLaneChangeModel->getShadowFurtherLanes())
                  << " shadowFurtherPosLat=" << toString(myLaneChangeModel->getShadowFurtherLanesPosLat())
                  << "\n     targetLane=" << Named::getIDSecure(myLaneChangeModel->getTargetLane())
                  << " furtherTargets=" << toString(myLaneChangeModel->getFurtherTargetLanes())
                  << std::endl;
    }
#endif
    if (lane == myLane
            || lane == myLaneChangeModel->getShadowLane()
            || lane == myLaneChangeModel->getTargetLane()) {
        if (myLaneChangeModel->isOpposite()) {
            if (lane == myLaneChangeModel->getShadowLane()) {
                return lane->getLength() - myState.myPos - myType->getLength();
            } else {
                return myState.myPos + (calledByGetPosition ? -1 : 1) * myType->getLength();
            }
        } else if (&lane->getEdge() != &myLane->getEdge()) {
            return lane->getLength() - myState.myPos + (calledByGetPosition ? -1 : 1) * myType->getLength();
        } else {
            // account for parallel lanes of different lengths in the most conservative manner (i.e. while turning)
            return myState.myPos - myType->getLength() + MIN2(0.0, lane->getLength() - myLane->getLength());
        }
    } else if (lane == myLane->getBidiLane()) {
        return lane->getLength() - myState.myPos + myType->getLength() * (calledByGetPosition ? -1 : 1);
    } else if (myFurtherLanes.size() > 0 && lane == myFurtherLanes.back()) {
        return myState.myBackPos;
    } else if ((myLaneChangeModel->getShadowFurtherLanes().size() > 0 && lane == myLaneChangeModel->getShadowFurtherLanes().back())
               || (myLaneChangeModel->getFurtherTargetLanes().size() > 0 && lane == myLaneChangeModel->getFurtherTargetLanes().back())) {
        assert(myFurtherLanes.size() > 0);
        if (lane->getLength() == myFurtherLanes.back()->getLength()) {
            return myState.myBackPos;
        } else {
            // interpolate
            //if (DEBUG_COND) {
            //if (myFurtherLanes.back()->getLength() != lane->getLength()) {
            //    std::cout << SIMTIME << " veh=" << getID() << " lane=" << lane->getID() << " further=" << myFurtherLanes.back()->getID()
            //        << " len=" << lane->getLength() << " fLen=" << myFurtherLanes.back()->getLength()
            //        << " backPos=" << myState.myBackPos << " result=" << myState.myBackPos / myFurtherLanes.back()->getLength() * lane->getLength() << "\n";
            //}
            return myState.myBackPos / myFurtherLanes.back()->getLength() * lane->getLength();
        }
    } else {
        //if (DEBUG_COND) std::cout << SIMTIME << " veh=" << getID() << " myFurtherLanes=" << toString(myFurtherLanes) << "\n";
        double leftLength = myType->getLength() - myState.myPos;

        std::vector<MSLane*>::const_iterator i = myFurtherLanes.begin();
        while (leftLength > 0 && i != myFurtherLanes.end()) {
            leftLength -= (*i)->getLength();
            //if (DEBUG_COND) std::cout << " comparing i=" << (*i)->getID() << " lane=" << lane->getID() << "\n";
            if (*i == lane) {
                return -leftLength;
            } else if (*i == lane->getBidiLane()) {
                return lane->getLength() + leftLength - (calledByGetPosition ? 2 * myType->getLength() : 0);
            }
            ++i;
        }
        //if (DEBUG_COND) std::cout << SIMTIME << " veh=" << getID() << " myShadowFurtherLanes=" << toString(myLaneChangeModel->getShadowFurtherLanes()) << "\n";
        leftLength = myType->getLength() - myState.myPos;
        i = myLaneChangeModel->getShadowFurtherLanes().begin();
        while (leftLength > 0 && i != myLaneChangeModel->getShadowFurtherLanes().end()) {
            leftLength -= (*i)->getLength();
            //if (DEBUG_COND) std::cout << " comparing i=" << (*i)->getID() << " lane=" << lane->getID() << "\n";
            if (*i == lane) {
                return -leftLength;
            }
            ++i;
        }
        //if (DEBUG_COND) std::cout << SIMTIME << " veh=" << getID() << " myFurtherTargetLanes=" << toString(myLaneChangeModel->getFurtherTargetLanes()) << "\n";
        leftLength = myType->getLength() - myState.myPos;
        i = getFurtherLanes().begin();
        const std::vector<MSLane*> furtherTargetLanes = myLaneChangeModel->getFurtherTargetLanes();
        auto j = furtherTargetLanes.begin();
        while (leftLength > 0 && j != furtherTargetLanes.end()) {
            leftLength -= (*i)->getLength();
            // if (DEBUG_COND) std::cout << " comparing i=" << (*i)->getID() << " lane=" << lane->getID() << "\n";
            if (*j == lane) {
                return -leftLength;
            }
            ++i;
            ++j;
        }
        WRITE_WARNING("Request backPos of vehicle '" + getID() + "' for invalid lane '" + Named::getIDSecure(lane)
                      + "' time=" + time2string(MSNet::getInstance()->getCurrentTimeStep()) + ".")
        SOFT_ASSERT(false);
        return  myState.myBackPos;
    }
}


double
MSVehicle::getPositionOnLane(const MSLane* lane) const {
    return getBackPositionOnLane(lane, true) + myType->getLength();
}


bool
MSVehicle::isFrontOnLane(const MSLane* lane) const {
    return lane == myLane || lane == myLaneChangeModel->getShadowLane() || lane == myLane->getBidiLane();
}


void
MSVehicle::checkRewindLinkLanes(const double lengthsInFront, DriveItemVector& lfLinks) const {
    if (MSGlobals::gUsingInternalLanes && !myLane->getEdge().isRoundabout() && !myLaneChangeModel->isOpposite()) {
        double seenSpace = -lengthsInFront;
#ifdef DEBUG_CHECKREWINDLINKLANES
        if (DEBUG_COND) {
            std::cout << "\nCHECK_REWIND_LINKLANES\n" << " veh=" << getID() << " lengthsInFront=" << lengthsInFront << "\n";
        };
#endif
        bool foundStopped = false;
        // compute available space until a stopped vehicle is found
        // this is the sum of non-interal lane length minus in-between vehicle lengths
        for (int i = 0; i < (int)lfLinks.size(); ++i) {
            // skip unset links
            DriveProcessItem& item = lfLinks[i];
#ifdef DEBUG_CHECKREWINDLINKLANES
            if (DEBUG_COND) std::cout << SIMTIME
                                          << " link=" << (item.myLink == 0 ? "NULL" : item.myLink->getViaLaneOrLane()->getID())
                                          << " foundStopped=" << foundStopped;
#endif
            if (item.myLink == nullptr || foundStopped) {
                if (!foundStopped) {
                    item.availableSpace += seenSpace;
                } else {
                    item.availableSpace = seenSpace;
                }
#ifdef DEBUG_CHECKREWINDLINKLANES
                if (DEBUG_COND) {
                    std::cout << " avail=" << item.availableSpace << "\n";
                }
#endif
                continue;
            }
            // get the next lane, determine whether it is an internal lane
            const MSLane* approachedLane = item.myLink->getViaLane();
            if (approachedLane != nullptr) {
                if (keepClear(item.myLink)) {
                    seenSpace = seenSpace - approachedLane->getBruttoVehLenSum();
                    if (approachedLane == myLane) {
                        seenSpace += getVehicleType().getLengthWithGap();
                    }
                } else {
                    seenSpace = seenSpace + approachedLane->getSpaceTillLastStanding(this, foundStopped);// - approachedLane->getBruttoVehLenSum() + approachedLane->getLength();
                }
                item.availableSpace = seenSpace;
#ifdef DEBUG_CHECKREWINDLINKLANES
                if (DEBUG_COND) std::cout
                            << " approached=" << approachedLane->getID()
                            << " approachedBrutto=" << approachedLane->getBruttoVehLenSum()
                            << " avail=" << item.availableSpace
                            << " seenSpace=" << seenSpace
                            << " hadStoppedVehicle=" << item.hadStoppedVehicle
                            << " lengthsInFront=" << lengthsInFront
                            << "\n";
#endif
                continue;
            }
            approachedLane = item.myLink->getLane();
            const MSVehicle* last = approachedLane->getLastAnyVehicle();
            if (last == nullptr || last == this) {
                if (approachedLane->getLength() > getVehicleType().getLength()
                        || keepClear(item.myLink)) {
                    seenSpace += approachedLane->getLength();
                }
                item.availableSpace = seenSpace;
#ifdef DEBUG_CHECKREWINDLINKLANES
                if (DEBUG_COND) {
                    std::cout << " last=" << Named::getIDSecure(last) << " laneLength=" << approachedLane->getLength() << " avail=" << item.availableSpace << "\n";
                }
#endif
            } else {
                bool foundStopped2 = false;
                double spaceTillLastStanding = approachedLane->getSpaceTillLastStanding(this, foundStopped2);
                if (approachedLane->getBidiLane() != nullptr) {
                    const MSVehicle* oncomingVeh = approachedLane->getBidiLane()->getFirstFullVehicle();
                    if (oncomingVeh) {
                        const double oncomingGap = approachedLane->getLength() - oncomingVeh->getPositionOnLane();
                        const double oncomingBGap = oncomingVeh->getBrakeGap(true);
                        // oncoming movement until ego enters the junction
                        const double oncomingMove = STEPS2TIME(item.myArrivalTime - SIMSTEP) * oncomingVeh->getSpeed();
                        const double spaceTillOncoming = oncomingGap - oncomingBGap - oncomingMove;
                        spaceTillLastStanding = MIN2(spaceTillLastStanding, spaceTillOncoming);
                        if (spaceTillOncoming <= getVehicleType().getLengthWithGap()) {
                            foundStopped = true;
                        }
#ifdef DEBUG_CHECKREWINDLINKLANES
                        if (DEBUG_COND) {
                            std::cout << " oVeh=" << oncomingVeh->getID()
                                      << " oGap=" << oncomingGap
                                      << " bGap=" << oncomingBGap
                                      << " mGap=" << oncomingMove
                                      << " sto=" << spaceTillOncoming;
                        }
#endif
                    }
                }
                seenSpace += spaceTillLastStanding;
                if (foundStopped2) {
                    foundStopped = true;
                    item.hadStoppedVehicle = true;
                }
                item.availableSpace = seenSpace;
                if (last->myHaveToWaitOnNextLink || last->isStopped()) {
                    foundStopped = true;
                    item.hadStoppedVehicle = true;
                }
#ifdef DEBUG_CHECKREWINDLINKLANES
                if (DEBUG_COND) std::cout
                            << " approached=" << approachedLane->getID()
                            << " last=" << last->getID()
                            << " lastHasToWait=" << last->myHaveToWaitOnNextLink
                            << " lastBrakeLight=" << last->signalSet(VEH_SIGNAL_BRAKELIGHT)
                            << " lastBrakeGap=" << last->getCarFollowModel().brakeGap(last->getSpeed())
                            << " lastGap=" << (last->getBackPositionOnLane(approachedLane) + last->getCarFollowModel().brakeGap(last->getSpeed()) - last->getSpeed() * last->getCarFollowModel().getHeadwayTime()
                                               // gap of last up to the next intersection
                                               - last->getVehicleType().getMinGap())
                            << " stls=" << spaceTillLastStanding
                            << " avail=" << item.availableSpace
                            << " seenSpace=" << seenSpace
                            << " foundStopped=" << foundStopped
                            << " foundStopped2=" << foundStopped2
                            << "\n";
#endif
            }
        }

        // check which links allow continuation and add pass available to the previous item
        for (int i = ((int)lfLinks.size() - 1); i > 0; --i) {
            DriveProcessItem& item = lfLinks[i - 1];
            DriveProcessItem& nextItem = lfLinks[i];
            const bool canLeaveJunction = item.myLink->getViaLane() == nullptr || nextItem.myLink == nullptr || nextItem.mySetRequest;
            const bool opened = (item.myLink != nullptr
                                 && (canLeaveJunction || (
                                         // indirect bicycle turn
                                         nextItem.myLink != nullptr && nextItem.myLink->isInternalJunctionLink() && nextItem.myLink->haveRed()))
                                 && (
                                     item.myLink->havePriority()
                                     || i == 1 // the upcoming link (item 0) is checked in executeMove anyway. No need to use outdata approachData here
                                     || (myInfluencer != nullptr && !myInfluencer->getRespectJunctionPriority())
                                     || item.myLink->opened(item.myArrivalTime, item.myArrivalSpeed,
                                             item.getLeaveSpeed(), getVehicleType().getLength(),
                                             getImpatience(), getCarFollowModel().getMaxDecel(), getWaitingTime(), getLateralPositionOnLane(), nullptr, false, this)));
            bool allowsContinuation = (item.myLink == nullptr || item.myLink->isCont() || opened) && !item.hadStoppedVehicle;
#ifdef DEBUG_CHECKREWINDLINKLANES
            if (DEBUG_COND) std::cout
                        << "   link=" << (item.myLink == 0 ? "NULL" : item.myLink->getViaLaneOrLane()->getID())
                        << " canLeave=" << canLeaveJunction
                        << " opened=" << opened
                        << " allowsContinuation=" << allowsContinuation
                        << " foundStopped=" << foundStopped
                        << "\n";
#endif
            if (!opened && item.myLink != nullptr) {
                foundStopped = true;
                if (i > 1) {
                    DriveProcessItem& item2 = lfLinks[i - 2];
                    if (item2.myLink != nullptr && item2.myLink->isCont()) {
                        allowsContinuation = true;
                    }
                }
            }
            if (allowsContinuation) {
                item.availableSpace = nextItem.availableSpace;
#ifdef DEBUG_CHECKREWINDLINKLANES
                if (DEBUG_COND) std::cout
                            << "   link=" << (item.myLink == nullptr ? "NULL" : item.myLink->getViaLaneOrLane()->getID())
                            << " copy nextAvail=" << nextItem.availableSpace
                            << "\n";
#endif
            }
        }

        // find removalBegin
        int removalBegin = -1;
        for (int i = 0; foundStopped && i < (int)lfLinks.size() && removalBegin < 0; ++i) {
            // skip unset links
            const DriveProcessItem& item = lfLinks[i];
            if (item.myLink == nullptr) {
                continue;
            }
            /*
            double impatienceCorrection = MAX2(0., double(double(myWaitingTime)));
            if (seenSpace<getVehicleType().getLengthWithGap()-impatienceCorrection/10.&&nextSeenNonInternal!=0) {
                removalBegin = lastLinkToInternal;
            }
            */

            const double leftSpace = item.availableSpace - getVehicleType().getLengthWithGap();
#ifdef DEBUG_CHECKREWINDLINKLANES
            if (DEBUG_COND) std::cout
                        << SIMTIME
                        << " veh=" << getID()
                        << " link=" << (item.myLink == 0 ? "NULL" : item.myLink->getViaLaneOrLane()->getID())
                        << " avail=" << item.availableSpace
                        << " leftSpace=" << leftSpace
                        << "\n";
#endif
            if (leftSpace < 0/* && item.myLink->willHaveBlockedFoe()*/) {
                double impatienceCorrection = 0;
                /*
                if(item.myLink->getState()==LINKSTATE_MINOR) {
                    impatienceCorrection = MAX2(0., STEPS2TIME(myWaitingTime));
                }
                */
                // may ignore keepClear rules
                if (leftSpace < -impatienceCorrection / 10. && keepClear(item.myLink)) {
                    removalBegin = i;
                }
                //removalBegin = i;
            }
        }
        // abort requests
        if (removalBegin != -1 && !(removalBegin == 0 && myLane->getEdge().isInternal())) {
            const double brakeGap = getCarFollowModel().brakeGap(myState.mySpeed, getCarFollowModel().getMaxDecel(), 0.);
            while (removalBegin < (int)(lfLinks.size())) {
                DriveProcessItem& dpi = lfLinks[removalBegin];
                if (dpi.myLink == nullptr) {
                    break;
                }
                dpi.myVLinkPass = dpi.myVLinkWait;
#ifdef DEBUG_CHECKREWINDLINKLANES
                if (DEBUG_COND) {
                    std::cout << " removalBegin=" << removalBegin << " brakeGap=" << brakeGap << " dist=" << dpi.myDistance << " speed=" << myState.mySpeed << " a2s=" << ACCEL2SPEED(getCarFollowModel().getMaxDecel()) << "\n";
                }
#endif
                if (dpi.myDistance >= brakeGap + POSITION_EPS) {
                    // always leave junctions after requesting to enter
                    if (!dpi.myLink->isExitLink() || !lfLinks[removalBegin - 1].mySetRequest) {
                        dpi.mySetRequest = false;
                    }
                }
                ++removalBegin;
            }
        }
    }
}


void
MSVehicle::setApproachingForAllLinks(const SUMOTime t) {
    if (!checkActionStep(t)) {
        return;
    }
    removeApproachingInformation(myLFLinkLanesPrev);
    for (DriveProcessItem& dpi : myLFLinkLanes) {
        if (dpi.myLink != nullptr) {
            if (dpi.myLink->getState() == LINKSTATE_ALLWAY_STOP) {
                dpi.myArrivalTime += (SUMOTime)RandHelper::rand((int)2, getRNG()); // tie braker
            }
            dpi.myLink->setApproaching(this, dpi.myArrivalTime, dpi.myArrivalSpeed, dpi.getLeaveSpeed(),
                                       dpi.mySetRequest, dpi.myArrivalSpeedBraking, getWaitingTime(), dpi.myDistance, getLateralPositionOnLane());
        }
    }
    if (isRailway(getVClass())) {
        for (DriveProcessItem& dpi : myLFLinkLanes) {
            if (dpi.myLink != nullptr && dpi.myLink->getTLLogic() != nullptr && dpi.myLink->getTLLogic()->getLogicType() == TrafficLightType::RAIL_SIGNAL) {
                MSRailSignalControl::getInstance().notifyApproach(dpi.myLink);
            }
        }
    }
    if (myLaneChangeModel->getShadowLane() != nullptr) {
        // register on all shadow links
        for (const DriveProcessItem& dpi : myLFLinkLanes) {
            if (dpi.myLink != nullptr) {
                MSLink* parallelLink = dpi.myLink->getParallelLink(myLaneChangeModel->getShadowDirection());
                if (parallelLink == nullptr && getLaneChangeModel().isOpposite() && dpi.myLink->isEntryLink()) {
                    // register on opposite direction entry link to warn foes at minor side road
                    parallelLink = dpi.myLink->getOppositeDirectionLink();
                }
                if (parallelLink != nullptr) {
                    const double latOffset = getLane()->getRightSideOnEdge() - myLaneChangeModel->getShadowLane()->getRightSideOnEdge();
                    parallelLink->setApproaching(this, dpi.myArrivalTime, dpi.myArrivalSpeed, dpi.getLeaveSpeed(),
                                                 dpi.mySetRequest, dpi.myArrivalSpeedBraking, getWaitingTime(), dpi.myDistance,
                                                 latOffset);
                    myLaneChangeModel->setShadowApproachingInformation(parallelLink);
                }
            }
        }
    }
#ifdef DEBUG_PLAN_MOVE
    if (DEBUG_COND) {
        std::cout << SIMTIME
                  << " veh=" << getID()
                  << " after checkRewindLinkLanes\n";
        for (DriveProcessItem& dpi : myLFLinkLanes) {
            std::cout
                    << " vPass=" << dpi.myVLinkPass
                    << " vWait=" << dpi.myVLinkWait
                    << " linkLane=" << (dpi.myLink == 0 ? "NULL" : dpi.myLink->getViaLaneOrLane()->getID())
                    << " request=" << dpi.mySetRequest
                    << " atime=" << dpi.myArrivalTime
                    << "\n";
        }
    }
#endif
}


void
MSVehicle::registerInsertionApproach(MSLink* link, double dist) {
    DriveProcessItem dpi(0, dist);
    dpi.myLink = link;
    const double arrivalSpeedBraking = getCarFollowModel().getMinimalArrivalSpeedEuler(dist, getSpeed());
    link->setApproaching(this, SUMOTime_MAX, 0, 0, false, arrivalSpeedBraking, 0, dpi.myDistance, 0);
    // ensure cleanup in the next step
    myLFLinkLanes.push_back(dpi);
    MSRailSignalControl::getInstance().notifyApproach(link);
}


void
MSVehicle::enterLaneAtMove(MSLane* enteredLane, bool onTeleporting) {
    myAmOnNet = !onTeleporting;
    // vaporizing edge?
    /*
    if (enteredLane->getEdge().isVaporizing()) {
        // yep, let's do the vaporization...
        myLane = enteredLane;
        return true;
    }
    */
    // Adjust MoveReminder offset to the next lane
    adaptLaneEntering2MoveReminder(*enteredLane);
    // set the entered lane as the current lane
    MSLane* oldLane = myLane;
    myLane = enteredLane;
    myLastBestLanesEdge = nullptr;

    // internal edges are not a part of the route...
    if (!enteredLane->getEdge().isInternal()) {
        ++myCurrEdge;
        assert(myLaneChangeModel->isOpposite() || haveValidStopEdges());
    }
    if (myInfluencer != nullptr) {
        myInfluencer->adaptLaneTimeLine(myLane->getIndex() - oldLane->getIndex());
    }
    if (!onTeleporting) {
        activateReminders(MSMoveReminder::NOTIFICATION_JUNCTION, enteredLane);
        if (MSGlobals::gLateralResolution > 0) {
            // transform lateral position when the lane width changes
            assert(oldLane != nullptr);
            const MSLink* const link = oldLane->getLinkTo(myLane);
            if (link != nullptr) {
                myFurtherLanesPosLat.push_back(myState.myPosLat);
                myState.myPosLat += link->getLateralShift();
            }
        } else if (fabs(myState.myPosLat) > NUMERICAL_EPS) {
            const double overlap = MAX2(0.0, getLateralOverlap(myState.myPosLat, oldLane));
            const double range = (oldLane->getWidth() - getVehicleType().getWidth()) * 0.5 + overlap;
            const double range2 = (myLane->getWidth() - getVehicleType().getWidth()) * 0.5 + overlap;
            myState.myPosLat *= range2 / range;
        }
        if (myLane->getBidiLane() != nullptr && (!isRailway(getVClass()) || (myLane->getPermissions() & ~SVC_RAIL_CLASSES) != 0)) {
            // railways don't need to "see" each other when moving in opposite directions on the same track (efficiency)
            // (unless the lane is shared with cars)
            myLane->getBidiLane()->setPartialOccupation(this);
        }
    } else {
        // normal move() isn't called so reset position here. must be done
        // before calling reminders
        myState.myPos = 0;
        myCachedPosition = Position::INVALID;
        activateReminders(MSMoveReminder::NOTIFICATION_TELEPORT, enteredLane);
    }
    // update via
    if (myParameter->via.size() > 0 &&  myLane->getEdge().getID() == myParameter->via.front()) {
        myParameter->via.erase(myParameter->via.begin());
    }
}


void
MSVehicle::enterLaneAtLaneChange(MSLane* enteredLane) {
    myAmOnNet = true;
    myLane = enteredLane;
    myCachedPosition = Position::INVALID;
    // need to update myCurrentLaneInBestLanes
    updateBestLanes();
    // switch to and activate the new lane's reminders
    // keep OldLaneReminders
    for (std::vector< MSMoveReminder* >::const_iterator rem = enteredLane->getMoveReminders().begin(); rem != enteredLane->getMoveReminders().end(); ++rem) {
        addReminder(*rem);
    }
    activateReminders(MSMoveReminder::NOTIFICATION_LANE_CHANGE, enteredLane);
    MSLane* lane = myLane;
    double leftLength = getVehicleType().getLength() - myState.myPos;
    int deleteFurther = 0;
#ifdef DEBUG_SETFURTHER
    if (DEBUG_COND) {
        std::cout << SIMTIME << " enterLaneAtLaneChange entered=" << Named::getIDSecure(enteredLane) << " oldFurther=" << toString(myFurtherLanes) << "\n";
    }
#endif
    if (myLane->getBidiLane() != nullptr && (!isRailway(getVClass()) || (myLane->getPermissions() & ~SVC_RAIL_CLASSES) != 0)) {
        // railways don't need to "see" each other when moving in opposite directions on the same track (efficiency)
        // (unless the lane is shared with cars)
        myLane->getBidiLane()->setPartialOccupation(this);
    }
    for (int i = 0; i < (int)myFurtherLanes.size(); i++) {
        if (lane != nullptr) {
            lane = lane->getLogicalPredecessorLane(myFurtherLanes[i]->getEdge());
        }
#ifdef DEBUG_SETFURTHER
        if (DEBUG_COND) {
            std::cout << "  enterLaneAtLaneChange i=" << i << " lane=" << Named::getIDSecure(lane) << " leftLength=" << leftLength << "\n";
        }
#endif
        if (leftLength > 0) {
            if (lane != nullptr) {
                myFurtherLanes[i]->resetPartialOccupation(this);
                if (myFurtherLanes[i]->getBidiLane() != nullptr
                        && (!isRailway(getVClass()) || (myFurtherLanes[i]->getPermissions() & ~SVC_RAIL_CLASSES) != 0)) {
                    myFurtherLanes[i]->getBidiLane()->resetPartialOccupation(this);
                }
                // lane changing onto longer lanes may reduce the number of
                // remaining further lanes
                myFurtherLanes[i] = lane;
                myFurtherLanesPosLat[i] = myState.myPosLat;
                leftLength -= lane->setPartialOccupation(this);
                if (lane->getBidiLane() != nullptr
                        && (!isRailway(getVClass()) || (lane->getPermissions() & ~SVC_RAIL_CLASSES) != 0)) {
                    lane->getBidiLane()->setPartialOccupation(this);
                }
                myState.myBackPos = -leftLength;
#ifdef DEBUG_SETFURTHER
                if (DEBUG_COND) {
                    std::cout << SIMTIME << "   newBackPos=" << myState.myBackPos << "\n";
                }
#endif
            } else {
                // keep the old values, but ensure there is no shadow
                if (myLaneChangeModel->isChangingLanes()) {
                    myLaneChangeModel->setNoShadowPartialOccupator(myFurtherLanes[i]);
                }
                if (myState.myBackPos < 0) {
                    myState.myBackPos += myFurtherLanes[i]->getLength();
                }
#ifdef DEBUG_SETFURTHER
                if (DEBUG_COND) {
                    std::cout << SIMTIME << "   i=" << i << " further=" << myFurtherLanes[i]->getID() << " newBackPos=" << myState.myBackPos << "\n";
                }
#endif
            }
        } else {
            myFurtherLanes[i]->resetPartialOccupation(this);
            if (myFurtherLanes[i]->getBidiLane() != nullptr
                    && (!isRailway(getVClass()) || (myFurtherLanes[i]->getPermissions() & ~SVC_RAIL_CLASSES) != 0)) {
                myFurtherLanes[i]->getBidiLane()->resetPartialOccupation(this);
            }
            deleteFurther++;
        }
    }
    if (deleteFurther > 0) {
#ifdef DEBUG_SETFURTHER
        if (DEBUG_COND) {
            std::cout << SIMTIME << " veh=" << getID() << " shortening myFurtherLanes by " << deleteFurther << "\n";
        }
#endif
        myFurtherLanes.erase(myFurtherLanes.end() - deleteFurther, myFurtherLanes.end());
        myFurtherLanesPosLat.erase(myFurtherLanesPosLat.end() - deleteFurther, myFurtherLanesPosLat.end());
    }
#ifdef DEBUG_SETFURTHER
    if (DEBUG_COND) {
        std::cout << SIMTIME << " enterLaneAtLaneChange new furtherLanes=" << toString(myFurtherLanes)
                  << " furterLanesPosLat=" << toString(myFurtherLanesPosLat) << "\n";
    }
#endif
    myAngle = computeAngle();
}


void
MSVehicle::computeFurtherLanes(MSLane* enteredLane, double pos, bool collision) {
    // build the list of lanes the vehicle is lapping into
    if (!myLaneChangeModel->isOpposite()) {
        double leftLength = myType->getLength() - pos;
        MSLane* clane = enteredLane;
        int routeIndex = getRoutePosition();
        while (leftLength > 0) {
            if (routeIndex > 0 && clane->getEdge().isNormal()) {
                // get predecessor lane that corresponds to prior route
                routeIndex--;
                const MSEdge* fromRouteEdge = myRoute->getEdges()[routeIndex];
                MSLane* target = clane;
                clane = nullptr;
                for (auto ili : target->getIncomingLanes()) {
                    if (ili.lane->getEdge().getNormalBefore() == fromRouteEdge) {
                        clane = ili.lane;
                        break;
                    }
                }
            } else {
                clane = clane->getLogicalPredecessorLane();
            }
            if (clane == nullptr || clane == myLane || clane == myLane->getBidiLane()
                    || (clane->isInternal() && (
                            clane->getLinkCont()[0]->getDirection() == LinkDirection::TURN
                            || clane->getLinkCont()[0]->getDirection() == LinkDirection::TURN_LEFTHAND))) {
                break;
            }
            if (!collision || std::find(myFurtherLanes.begin(), myFurtherLanes.end(), clane) == myFurtherLanes.end()) {
                myFurtherLanes.push_back(clane);
                myFurtherLanesPosLat.push_back(myState.myPosLat);
                clane->setPartialOccupation(this);
                if (clane->getBidiLane() != nullptr
                        && (!isRailway(getVClass()) || (clane->getPermissions() & ~SVC_RAIL_CLASSES) != 0)) {
                    clane->getBidiLane()->setPartialOccupation(this);
                }
            }
            leftLength -= clane->getLength();
        }
        myState.myBackPos = -leftLength;
#ifdef DEBUG_SETFURTHER
        if (DEBUG_COND) {
            std::cout << SIMTIME << " computeFurtherLanes veh=" << getID() << " pos=" << pos << " myFurtherLanes=" << toString(myFurtherLanes) << " backPos=" << myState.myBackPos << "\n";
        }
#endif
    } else {
        // clear partial occupation
        for (MSLane* further : myFurtherLanes) {
#ifdef DEBUG_SETFURTHER
            if (DEBUG_COND) {
                std::cout << SIMTIME << " opposite: resetPartialOccupation " << further->getID() << " \n";
            }
#endif
            further->resetPartialOccupation(this);
            if (further->getBidiLane() != nullptr
                    && (!isRailway(getVClass()) || (further->getPermissions() & ~SVC_RAIL_CLASSES) != 0)) {
                further->getBidiLane()->resetPartialOccupation(this);
            }
        }
        myFurtherLanes.clear();
        myFurtherLanesPosLat.clear();
    }
}


void
MSVehicle::enterLaneAtInsertion(MSLane* enteredLane, double pos, double speed, double posLat, MSMoveReminder::Notification notification) {
    myState = State(pos, speed, posLat, pos - getVehicleType().getLength(), hasDeparted() ? myState.myPreviousSpeed : speed);
    if (myDeparture == NOT_YET_DEPARTED) {
        onDepart();
    }
    myCachedPosition = Position::INVALID;
    assert(myState.myPos >= 0);
    assert(myState.mySpeed >= 0);
    myLane = enteredLane;
    myAmOnNet = true;
    // schedule action for the next timestep
    myLastActionTime = MSNet::getInstance()->getCurrentTimeStep() + DELTA_T;
    if (notification != MSMoveReminder::NOTIFICATION_TELEPORT) {
        // set and activate the new lane's reminders, teleports already did that at enterLaneAtMove
        for (std::vector< MSMoveReminder* >::const_iterator rem = enteredLane->getMoveReminders().begin(); rem != enteredLane->getMoveReminders().end(); ++rem) {
            addReminder(*rem);
        }
        activateReminders(notification, enteredLane);
    } else {
        myLastBestLanesEdge = nullptr;
        myLastBestLanesInternalLane = nullptr;
        myLaneChangeModel->resetState();
    }
    computeFurtherLanes(enteredLane, pos);
    if (MSGlobals::gLateralResolution > 0) {
        myLaneChangeModel->updateShadowLane();
        myLaneChangeModel->updateTargetLane();
    } else if (MSGlobals::gLaneChangeDuration > 0) {
        myLaneChangeModel->updateShadowLane();
    }
    if (notification != MSMoveReminder::NOTIFICATION_LOAD_STATE) {
        myAngle = computeAngle();
        if (myLaneChangeModel->isOpposite()) {
            myAngle += M_PI;
        }
    }
}


void
MSVehicle::leaveLane(const MSMoveReminder::Notification reason, const MSLane* approachedLane) {
    for (MoveReminderCont::iterator rem = myMoveReminders.begin(); rem != myMoveReminders.end();) {
        if (rem->first->notifyLeave(*this, myState.myPos + rem->second, reason, approachedLane)) {
#ifdef _DEBUG
            if (myTraceMoveReminders) {
                traceMoveReminder("notifyLeave", rem->first, rem->second, true);
            }
#endif
            ++rem;
        } else {
#ifdef _DEBUG
            if (myTraceMoveReminders) {
                traceMoveReminder("notifyLeave", rem->first, rem->second, false);
            }
#endif
            rem = myMoveReminders.erase(rem);
        }
    }
    if ((reason == MSMoveReminder::NOTIFICATION_JUNCTION
            || reason == MSMoveReminder::NOTIFICATION_TELEPORT
            || reason == MSMoveReminder::NOTIFICATION_TELEPORT_CONTINUATION)
            && myLane != nullptr) {
        myOdometer += getLane()->getLength();
    }
    if (myLane != nullptr && myLane->getBidiLane() != nullptr && myAmOnNet
            && (!isRailway(getVClass()) || (myLane->getPermissions() & ~SVC_RAIL_CLASSES) != 0)) {
        myLane->getBidiLane()->resetPartialOccupation(this);
    }
    if (reason != MSMoveReminder::NOTIFICATION_JUNCTION && reason != MSMoveReminder::NOTIFICATION_LANE_CHANGE) {
        // @note. In case of lane change, myFurtherLanes and partial occupation
        // are handled in enterLaneAtLaneChange()
        for (MSLane* further : myFurtherLanes) {
#ifdef DEBUG_FURTHER
            if (DEBUG_COND) {
                std::cout << SIMTIME << " leaveLane \n";
            }
#endif
            further->resetPartialOccupation(this);
            if (further->getBidiLane() != nullptr
                    && (!isRailway(getVClass()) || (further->getPermissions() & ~SVC_RAIL_CLASSES) != 0)) {
                further->getBidiLane()->resetPartialOccupation(this);
            }
        }
        myFurtherLanes.clear();
        myFurtherLanesPosLat.clear();
    }
    if (reason >= MSMoveReminder::NOTIFICATION_TELEPORT) {
        myAmOnNet = false;
        myWaitingTime = 0;
    }
    if (reason != MSMoveReminder::NOTIFICATION_PARKING && resumeFromStopping()) {
        myStopDist = std::numeric_limits<double>::max();
        if (myPastStops.back().speed <= 0) {
            WRITE_WARNINGF(TL("Vehicle '%' aborts stop."), getID());
        }
    }
    if (reason != MSMoveReminder::NOTIFICATION_PARKING && reason != MSMoveReminder::NOTIFICATION_LANE_CHANGE) {
        while (!myStops.empty() && myStops.front().edge == myCurrEdge && &myStops.front().lane->getEdge() == &myLane->getEdge()) {
            if (myStops.front().getSpeed() <= 0) {
                WRITE_WARNINGF(TL("Vehicle '%' skips stop on lane '%' time=%."), getID(), myStops.front().lane->getID(),
                               time2string(MSNet::getInstance()->getCurrentTimeStep()))
                myStops.pop_front();
            } else {
                MSStop& stop = myStops.front();
                // passed waypoint at the end of the lane
                if (!stop.reached) {
                    if (MSStopOut::active()) {
                        MSStopOut::getInstance()->stopStarted(this, getPersonNumber(), getContainerNumber(), MSNet::getInstance()->getCurrentTimeStep());
                    }
                    stop.reached = true;
                    // enter stopping place so leaveFrom works as expected
                    if (stop.busstop != nullptr) {
                        // let the bus stop know the vehicle
                        stop.busstop->enter(this, stop.pars.parking == ParkingType::OFFROAD);
                    }
                    if (stop.containerstop != nullptr) {
                        // let the container stop know the vehicle
                        stop.containerstop->enter(this, stop.pars.parking == ParkingType::OFFROAD);
                    }
                    // do not enter parkingarea!
                    if (stop.chargingStation != nullptr) {
                        // let the container stop know the vehicle
                        stop.chargingStation->enter(this, stop.pars.parking == ParkingType::OFFROAD);
                    }
                }
                resumeFromStopping();
            }
            myStopDist = std::numeric_limits<double>::max();
        }
    }
}


void
MSVehicle::leaveLaneBack(const MSMoveReminder::Notification reason, const MSLane* leftLane) {
    for (MoveReminderCont::iterator rem = myMoveReminders.begin(); rem != myMoveReminders.end();) {
        if (rem->first->notifyLeaveBack(*this, reason, leftLane)) {
#ifdef _DEBUG
            if (myTraceMoveReminders) {
                traceMoveReminder("notifyLeaveBack", rem->first, rem->second, true);
            }
#endif
            ++rem;
        } else {
#ifdef _DEBUG
            if (myTraceMoveReminders) {
                traceMoveReminder("notifyLeaveBack", rem->first, rem->second, false);
            }
#endif
            rem = myMoveReminders.erase(rem);
        }
    }
#ifdef DEBUG_MOVEREMINDERS
    if (DEBUG_COND) {
        std::cout << SIMTIME << " veh=" << getID() << " myReminders:";
        for (auto rem : myMoveReminders) {
            std::cout << rem.first->getDescription() << " ";
        }
        std::cout << "\n";
    }
#endif
}


MSAbstractLaneChangeModel&
MSVehicle::getLaneChangeModel() {
    return *myLaneChangeModel;
}


const MSAbstractLaneChangeModel&
MSVehicle::getLaneChangeModel() const {
    return *myLaneChangeModel;
}

bool
MSVehicle::isOppositeLane(const MSLane* lane) const {
    return (lane->isInternal()
            ? & (lane->getLinkCont()[0]->getLane()->getEdge()) != *(myCurrEdge + 1)
            : &lane->getEdge() != *myCurrEdge);
}

const std::vector<MSVehicle::LaneQ>&
MSVehicle::getBestLanes() const {
    return *myBestLanes.begin();
}


void
MSVehicle::updateBestLanes(bool forceRebuild, const MSLane* startLane) {
#ifdef DEBUG_BESTLANES
    if (DEBUG_COND) {
        std::cout << SIMTIME << " updateBestLanes veh=" << getID() << " force=" << forceRebuild << " startLane1=" << Named::getIDSecure(startLane) << " myLane=" << Named::getIDSecure(myLane) << "\n";
    }
#endif
    if (startLane == nullptr) {
        startLane = myLane;
    }
    assert(startLane != 0);
    if (myLaneChangeModel->isOpposite()) {
        // depending on the calling context, startLane might be the forward lane
        // or the reverse-direction lane. In the latter case we need to
        // transform it to the forward lane.
        if (isOppositeLane(startLane)) {
            // use leftmost lane of forward edge
            startLane = startLane->getEdge().getOppositeEdge()->getLanes().back();
            assert(startLane != 0);
#ifdef DEBUG_BESTLANES
            if (DEBUG_COND) {
                std::cout << "   startLaneIsOpposite newStartLane=" << startLane->getID() << "\n";
            }
#endif
        }
    }
    if (forceRebuild) {
        myLastBestLanesEdge = nullptr;
        myLastBestLanesInternalLane = nullptr;
    }
    if (myBestLanes.size() > 0 && !forceRebuild && myLastBestLanesEdge == &startLane->getEdge()) {
        updateOccupancyAndCurrentBestLane(startLane);
#ifdef DEBUG_BESTLANES
        if (DEBUG_COND) {
            std::cout << "  only updateOccupancyAndCurrentBestLane\n";
        }
#endif
        return;
    }
    if (startLane->getEdge().isInternal()) {
        if (myBestLanes.size() == 0 || forceRebuild) {
            // rebuilt from previous non-internal lane (may backtrack twice if behind an internal junction)
            updateBestLanes(true, startLane->getLogicalPredecessorLane());
        }
        if (myLastBestLanesInternalLane == startLane && !forceRebuild) {
#ifdef DEBUG_BESTLANES
            if (DEBUG_COND) {
                std::cout << "  nothing to do on internal\n";
            }
#endif
            return;
        }
        // adapt best lanes to fit the current internal edge:
        // keep the entries that are reachable from this edge
        const MSEdge* nextEdge = startLane->getNextNormal();
        assert(!nextEdge->isInternal());
        for (std::vector<std::vector<LaneQ> >::iterator it = myBestLanes.begin(); it != myBestLanes.end();) {
            std::vector<LaneQ>& lanes = *it;
            assert(lanes.size() > 0);
            if (&(lanes[0].lane->getEdge()) == nextEdge) {
                // keep those lanes which are successors of internal lanes from the edge of startLane
                std::vector<LaneQ> oldLanes = lanes;
                lanes.clear();
                const std::vector<MSLane*>& sourceLanes = startLane->getEdge().getLanes();
                for (std::vector<MSLane*>::const_iterator it_source = sourceLanes.begin(); it_source != sourceLanes.end(); ++it_source) {
                    for (std::vector<LaneQ>::iterator it_lane = oldLanes.begin(); it_lane != oldLanes.end(); ++it_lane) {
                        if ((*it_source)->getLinkCont()[0]->getLane() == (*it_lane).lane) {
                            lanes.push_back(*it_lane);
                            break;
                        }
                    }
                }
                assert(lanes.size() == startLane->getEdge().getLanes().size());
                // patch invalid bestLaneOffset and updated myCurrentLaneInBestLanes
                for (int i = 0; i < (int)lanes.size(); ++i) {
                    if (i + lanes[i].bestLaneOffset < 0) {
                        lanes[i].bestLaneOffset = -i;
                    }
                    if (i + lanes[i].bestLaneOffset >= (int)lanes.size()) {
                        lanes[i].bestLaneOffset = (int)lanes.size() - i - 1;
                    }
                    assert(i + lanes[i].bestLaneOffset >= 0);
                    assert(i + lanes[i].bestLaneOffset < (int)lanes.size());
                    if (lanes[i].bestContinuations[0] != 0) {
                        // patch length of bestContinuation to match expectations (only once)
                        lanes[i].bestContinuations.insert(lanes[i].bestContinuations.begin(), (MSLane*)nullptr);
                    }
                    if (startLane->getLinkCont()[0]->getLane() == lanes[i].lane) {
                        myCurrentLaneInBestLanes = lanes.begin() + i;
                    }
                    assert(&(lanes[i].lane->getEdge()) == nextEdge);
                }
                myLastBestLanesInternalLane = startLane;
                updateOccupancyAndCurrentBestLane(startLane);
#ifdef DEBUG_BESTLANES
                if (DEBUG_COND) {
                    std::cout << "  updated for internal\n";
                }
#endif
                return;
            } else {
                // remove passed edges
                it = myBestLanes.erase(it);
            }
        }
        assert(false); // should always find the next edge
    }
    // start rebuilding
    myLastBestLanesInternalLane = nullptr;
    myLastBestLanesEdge = &startLane->getEdge();
    myBestLanes.clear();

    // get information about the next stop
    MSRouteIterator nextStopEdge = myRoute->end();
    const MSLane* nextStopLane = nullptr;
    double nextStopPos = 0;
    bool nextStopIsWaypoint = false;
    if (!myStops.empty()) {
        const MSStop& nextStop = myStops.front();
        nextStopLane = nextStop.lane;
        if (nextStop.isOpposite) {
            // target leftmost lane in forward direction
            nextStopLane = nextStopLane->getEdge().getOppositeEdge()->getLanes().back();
        }
        nextStopEdge = nextStop.edge;
        nextStopPos = nextStop.pars.startPos;
        nextStopIsWaypoint = nextStop.getSpeed() > 0;
    }
    // myArrivalTime = -1 in the context of validating departSpeed with departLane=best
    if (myParameter->arrivalLaneProcedure >= ArrivalLaneDefinition::GIVEN && nextStopEdge == myRoute->end() && myArrivalLane >= 0) {
        nextStopEdge = (myRoute->end() - 1);
        nextStopLane = (*nextStopEdge)->getLanes()[myArrivalLane];
        nextStopPos = myArrivalPos;
    }
    if (nextStopEdge != myRoute->end()) {
        // make sure that the "wrong" lanes get a penalty. (penalty needs to be
        // large enough to overcome a magic threshold in MSLaneChangeModel::DK2004.cpp:383)
        nextStopPos = MAX2(POSITION_EPS, MIN2((double)nextStopPos, (double)(nextStopLane->getLength() - 2 * POSITION_EPS)));
        if (nextStopLane->isInternal()) {
            // switch to the correct lane before entering the intersection
            nextStopPos = (*nextStopEdge)->getLength();
        }
    }

    // go forward along the next lanes;
    // trains do not have to deal with lane-changing for stops but their best
    // lanes lookahead is needed for rail signal control
    const bool continueAfterStop = nextStopIsWaypoint || isRailway(getVClass());
    int seen = 0;
    double seenLength = 0;
    bool progress = true;
    // bestLanes must cover the braking distance even when at the very end of the current lane to avoid unecessary slow down
    const double maxBrakeDist = startLane->getLength() + getCarFollowModel().getHeadwayTime() * getMaxSpeed() + getCarFollowModel().brakeGap(getMaxSpeed()) + getVehicleType().getMinGap();
    for (MSRouteIterator ce = myCurrEdge; progress;) {
        std::vector<LaneQ> currentLanes;
        const std::vector<MSLane*>* allowed = nullptr;
        const MSEdge* nextEdge = nullptr;
        if (ce != myRoute->end() && ce + 1 != myRoute->end()) {
            nextEdge = *(ce + 1);
            allowed = (*ce)->allowedLanes(*nextEdge, myType->getVehicleClass());
        }
        const std::vector<MSLane*>& lanes = (*ce)->getLanes();
        for (std::vector<MSLane*>::const_iterator i = lanes.begin(); i != lanes.end(); ++i) {
            LaneQ q;
            MSLane* cl = *i;
            q.lane = cl;
            q.bestContinuations.push_back(cl);
            q.bestLaneOffset = 0;
            q.length = cl->allowsVehicleClass(myType->getVehicleClass()) ? (*ce)->getLength() : 0;
            q.currentLength = q.length;
            // if all lanes are forbidden (i.e. due to a dynamic closing) we want to express no preference
            q.allowsContinuation = allowed == nullptr || std::find(allowed->begin(), allowed->end(), cl) != allowed->end();
            q.occupation = 0;
            q.nextOccupation = 0;
            currentLanes.push_back(q);
        }
        //
        if (nextStopEdge == ce
                // already past the stop edge
                && !(ce == myCurrEdge && myLane != nullptr && myLane->isInternal())) {
            if (!nextStopLane->isInternal() && !continueAfterStop) {
                progress = false;
            }
            const MSLane* normalStopLane = nextStopLane->getNormalPredecessorLane();
            for (std::vector<LaneQ>::iterator q = currentLanes.begin(); q != currentLanes.end(); ++q) {
                if (nextStopLane != nullptr && normalStopLane != (*q).lane) {
                    (*q).allowsContinuation = false;
                    (*q).length = nextStopPos;
                    (*q).currentLength = (*q).length;
                }
            }
        }

        myBestLanes.push_back(currentLanes);
        ++seen;
        seenLength += currentLanes[0].lane->getLength();
        ++ce;
        progress &= (seen <= 4 || seenLength < MAX2(maxBrakeDist, 3000.0)); // motorway
        progress &= (seen <= 8 || seenLength < MAX2(maxBrakeDist, 200.0) || isRailway(getVClass()));  // urban
        progress &= ce != myRoute->end();
        /*
        if(progress) {
          progress &= (currentLanes.size()!=1||(*ce)->getLanes().size()!=1);
        }
        */
    }

    // we are examining the last lane explicitly
    if (myBestLanes.size() != 0) {
        double bestLength = -1;
        // minimum and maximum lane index with best length
        int bestThisIndex = 0;
        int bestThisMaxIndex = 0;
        int index = 0;
        std::vector<LaneQ>& last = myBestLanes.back();
        for (std::vector<LaneQ>::iterator j = last.begin(); j != last.end(); ++j, ++index) {
            if ((*j).length > bestLength) {
                bestLength = (*j).length;
                bestThisIndex = index;
                bestThisMaxIndex = index;
            } else if ((*j).length == bestLength) {
                bestThisMaxIndex = index;
            }
        }
        index = 0;
        bool requiredChangeRightForbidden = false;
        int requireChangeToLeftForbidden = -1;
        for (std::vector<LaneQ>::iterator j = last.begin(); j != last.end(); ++j, ++index) {
            if ((*j).length < bestLength) {
                if (abs(bestThisIndex - index) < abs(bestThisMaxIndex - index)) {
                    (*j).bestLaneOffset = bestThisIndex - index;
                } else {
                    (*j).bestLaneOffset = bestThisMaxIndex - index;
                }
                if ((*j).bestLaneOffset < 0 && (!(*j).lane->allowsChangingRight(getVClass())
                                                || !(*j).lane->getParallelLane(-1, false)->allowsVehicleClass(getVClass())
                                                || requiredChangeRightForbidden)) {
                    // this lane and all further lanes to the left cannot be used
                    requiredChangeRightForbidden = true;
                    (*j).length = 0;
                } else if ((*j).bestLaneOffset > 0 && (!(*j).lane->allowsChangingLeft(getVClass())
                                                       || !(*j).lane->getParallelLane(1, false)->allowsVehicleClass(getVClass()))) {
                    // this lane and all previous lanes to the right cannot be used
                    requireChangeToLeftForbidden = (*j).lane->getIndex();
                }
            }
        }
        for (int i = requireChangeToLeftForbidden; i >= 0; i--) {
            last[i].length = 0;
        }
#ifdef DEBUG_BESTLANES
        if (DEBUG_COND) {
            std::cout << "   last edge=" << last.front().lane->getEdge().getID() << " (bestIndex=" << bestThisIndex << " bestMaxIndex=" << bestThisMaxIndex << "):\n";
            std::vector<LaneQ>& laneQs = myBestLanes.back();
            for (std::vector<LaneQ>::iterator j = laneQs.begin(); j != laneQs.end(); ++j) {
                std::cout << "     lane=" << (*j).lane->getID() << " length=" << (*j).length << " bestOffset=" << (*j).bestLaneOffset << "\n";
            }
        }
#endif
    }
    // go backward through the lanes
    // track back best lane and compute the best prior lane(s)
    for (std::vector<std::vector<LaneQ> >::reverse_iterator i = myBestLanes.rbegin() + 1; i != myBestLanes.rend(); ++i) {
        std::vector<LaneQ>& nextLanes = (*(i - 1));
        std::vector<LaneQ>& clanes = (*i);
        MSEdge* const cE = &clanes[0].lane->getEdge();
        int index = 0;
        double bestConnectedLength = -1;
        double bestLength = -1;
        for (const LaneQ& j : nextLanes) {
            if (j.lane->isApproachedFrom(cE) && bestConnectedLength < j.length) {
                bestConnectedLength = j.length;
            }
            if (bestLength < j.length) {
                bestLength = j.length;
            }
        }
        // compute index of the best lane (highest length and least offset from the best next lane)
        int bestThisIndex = 0;
        int bestThisMaxIndex = 0;
        if (bestConnectedLength > 0) {
            index = 0;
            for (LaneQ& j : clanes) {
                const LaneQ* bestConnectedNext = nullptr;
                if (j.allowsContinuation) {
                    for (const LaneQ& m : nextLanes) {
                        if ((m.lane->allowsVehicleClass(getVClass()) || m.lane->hadPermissionChanges())
                                && m.lane->isApproachedFrom(cE, j.lane)) {
                            if (betterContinuation(bestConnectedNext, m)) {
                                bestConnectedNext = &m;
                            }
                        }
                    }
                    if (bestConnectedNext != nullptr) {
                        if (bestConnectedNext->length == bestConnectedLength && abs(bestConnectedNext->bestLaneOffset) < 2) {
                            j.length += bestLength;
                        } else {
                            j.length += bestConnectedNext->length;
                        }
                        j.bestLaneOffset = bestConnectedNext->bestLaneOffset;
                    }
                }
                if (bestConnectedNext != nullptr && (bestConnectedNext->allowsContinuation || bestConnectedNext->length > 0)) {
                    copy(bestConnectedNext->bestContinuations.begin(), bestConnectedNext->bestContinuations.end(), back_inserter(j.bestContinuations));
                } else {
                    j.allowsContinuation = false;
                }
                if (clanes[bestThisIndex].length < j.length
                        || (clanes[bestThisIndex].length == j.length && abs(clanes[bestThisIndex].bestLaneOffset) > abs(j.bestLaneOffset))
                        || (clanes[bestThisIndex].length == j.length && abs(clanes[bestThisIndex].bestLaneOffset) == abs(j.bestLaneOffset) &&
                            nextLinkPriority(clanes[bestThisIndex].bestContinuations) < nextLinkPriority(j.bestContinuations))
                   ) {
                    bestThisIndex = index;
                    bestThisMaxIndex = index;
                } else if (clanes[bestThisIndex].length == j.length
                           && abs(clanes[bestThisIndex].bestLaneOffset) == abs(j.bestLaneOffset)
                           && nextLinkPriority(clanes[bestThisIndex].bestContinuations) == nextLinkPriority(j.bestContinuations)) {
                    bestThisMaxIndex = index;
                }
                index++;
            }

            //vehicle with elecHybrid device prefers running under an overhead wire
            if (getDevice(typeid(MSDevice_ElecHybrid)) != nullptr) {
                index = 0;
                for (const LaneQ& j : clanes) {
                    std::string overheadWireSegmentID = MSNet::getInstance()->getStoppingPlaceID(j.lane, j.currentLength / 2., SUMO_TAG_OVERHEAD_WIRE_SEGMENT);
                    if (overheadWireSegmentID != "") {
                        bestThisIndex = index;
                        bestThisMaxIndex = index;
                    }
                    index++;
                }
            }

        } else {
            // only needed in case of disconnected routes
            int bestNextIndex = 0;
            int bestDistToNeeded = (int) clanes.size();
            index = 0;
            for (std::vector<LaneQ>::iterator j = clanes.begin(); j != clanes.end(); ++j, ++index) {
                if ((*j).allowsContinuation) {
                    int nextIndex = 0;
                    for (std::vector<LaneQ>::const_iterator m = nextLanes.begin(); m != nextLanes.end(); ++m, ++nextIndex) {
                        if ((*m).lane->isApproachedFrom(cE, (*j).lane)) {
                            if (bestDistToNeeded > abs((*m).bestLaneOffset)) {
                                bestDistToNeeded = abs((*m).bestLaneOffset);
                                bestThisIndex = index;
                                bestThisMaxIndex = index;
                                bestNextIndex = nextIndex;
                            }
                        }
                    }
                }
            }
            clanes[bestThisIndex].length += nextLanes[bestNextIndex].length;
            copy(nextLanes[bestNextIndex].bestContinuations.begin(), nextLanes[bestNextIndex].bestContinuations.end(), back_inserter(clanes[bestThisIndex].bestContinuations));

        }
        // set bestLaneOffset for all lanes
        index = 0;
        bool requiredChangeRightForbidden = false;
        int requireChangeToLeftForbidden = -1;
        for (std::vector<LaneQ>::iterator j = clanes.begin(); j != clanes.end(); ++j, ++index) {
            if ((*j).length < clanes[bestThisIndex].length
                    || ((*j).length == clanes[bestThisIndex].length && abs((*j).bestLaneOffset) > abs(clanes[bestThisIndex].bestLaneOffset))
                    || (nextLinkPriority((*j).bestContinuations)) < nextLinkPriority(clanes[bestThisIndex].bestContinuations)
               ) {
                if (abs(bestThisIndex - index) < abs(bestThisMaxIndex - index)) {
                    (*j).bestLaneOffset = bestThisIndex - index;
                } else {
                    (*j).bestLaneOffset = bestThisMaxIndex - index;
                }
                if ((nextLinkPriority((*j).bestContinuations)) < nextLinkPriority(clanes[bestThisIndex].bestContinuations)) {
                    // try to move away from the lower-priority lane before it ends
                    (*j).length = (*j).currentLength;
                }
                if ((*j).bestLaneOffset < 0 && (!(*j).lane->allowsChangingRight(getVClass())
                                                || !(*j).lane->getParallelLane(-1, false)->allowsVehicleClass(getVClass())
                                                || requiredChangeRightForbidden)) {
                    // this lane and all further lanes to the left cannot be used
                    requiredChangeRightForbidden = true;
                    if ((*j).length == (*j).currentLength) {
                        (*j).length = 0;
                    }
                } else if ((*j).bestLaneOffset > 0 && (!(*j).lane->allowsChangingLeft(getVClass())
                                                       || !(*j).lane->getParallelLane(1, false)->allowsVehicleClass(getVClass()))) {
                    // this lane and all previous lanes to the right cannot be used
                    requireChangeToLeftForbidden = (*j).lane->getIndex();
                }
            } else {
                (*j).bestLaneOffset = 0;
            }
        }
        for (int idx = requireChangeToLeftForbidden; idx >= 0; idx--) {
            if (clanes[idx].length == clanes[idx].currentLength) {
                clanes[idx].length = 0;
            };
        }

        //vehicle with elecHybrid device prefers running under an overhead wire
        if (static_cast<MSDevice_ElecHybrid*>(getDevice(typeid(MSDevice_ElecHybrid))) != 0) {
            index = 0;
            std::string overheadWireID = MSNet::getInstance()->getStoppingPlaceID(clanes[bestThisIndex].lane, (clanes[bestThisIndex].currentLength) / 2, SUMO_TAG_OVERHEAD_WIRE_SEGMENT);
            if (overheadWireID != "") {
                for (std::vector<LaneQ>::iterator j = clanes.begin(); j != clanes.end(); ++j, ++index) {
                    (*j).bestLaneOffset = bestThisIndex - index;
                }
            }
        }

#ifdef DEBUG_BESTLANES
        if (DEBUG_COND) {
            std::cout << "   edge=" << cE->getID() << " (bestIndex=" << bestThisIndex << " bestMaxIndex=" << bestThisMaxIndex << "):\n";
            std::vector<LaneQ>& laneQs = clanes;
            for (std::vector<LaneQ>::iterator j = laneQs.begin(); j != laneQs.end(); ++j) {
                std::cout << "     lane=" << (*j).lane->getID() << " length=" << (*j).length << " bestOffset=" << (*j).bestLaneOffset << " allowCont=" << (*j).allowsContinuation << "\n";
            }
        }
#endif

    }
    updateOccupancyAndCurrentBestLane(startLane);
#ifdef DEBUG_BESTLANES
    if (DEBUG_COND) {
        std::cout << SIMTIME << " veh=" << getID() << " bestCont=" << toString(getBestLanesContinuation()) << "\n";
    }
#endif
}

void
MSVehicle::updateLaneBruttoSum() {
    if (myLane != nullptr) {
        myLane->markRecalculateBruttoSum();
    }
}

bool
MSVehicle::betterContinuation(const LaneQ* bestConnectedNext, const LaneQ& m) const {
    if (bestConnectedNext == nullptr) {
        return true;
    } else if (m.lane->getBidiLane() != nullptr && bestConnectedNext->lane->getBidiLane() == nullptr) {
        return false;
    } else if (bestConnectedNext->lane->getBidiLane() != nullptr && m.lane->getBidiLane() == nullptr) {
        return true;
    } else if (bestConnectedNext->length < m.length) {
        return true;
    } else if (bestConnectedNext->length == m.length) {
        if (abs(bestConnectedNext->bestLaneOffset) > abs(m.bestLaneOffset)) {
            return true;
        }
        const double contRight = getVehicleType().getParameter().getLCParam(SUMO_ATTR_LCA_CONTRIGHT, 1);
        if (contRight < 1
                // if we don't check for adjacency, the rightmost line will get
                // multiple chances to be better which leads to an uninituitve distribution
                && (m.lane->getIndex() - bestConnectedNext->lane->getIndex()) == 1
                && RandHelper::rand(getRNG()) > contRight) {
            return true;
        }
    }
    return false;
}


int
MSVehicle::nextLinkPriority(const std::vector<MSLane*>& conts) {
    if (conts.size() < 2) {
        return -1;
    } else {
        const MSLink* const link = conts[0]->getLinkTo(conts[1]);
        if (link != nullptr) {
            return link->havePriority() ? 1 : 0;
        } else {
            // disconnected route
            return -1;
        }
    }
}


void
MSVehicle::updateOccupancyAndCurrentBestLane(const MSLane* startLane) {
    std::vector<LaneQ>& currLanes = *myBestLanes.begin();
    std::vector<LaneQ>::iterator i;
    for (i = currLanes.begin(); i != currLanes.end(); ++i) {
        double nextOccupation = 0;
        for (std::vector<MSLane*>::const_iterator j = (*i).bestContinuations.begin() + 1; j != (*i).bestContinuations.end(); ++j) {
            nextOccupation += (*j)->getBruttoVehLenSum();
        }
        (*i).nextOccupation = nextOccupation;
#ifdef DEBUG_BESTLANES
        if (DEBUG_COND) {
            std::cout << "     lane=" << (*i).lane->getID() << " nextOccupation=" << nextOccupation << "\n";
        }
#endif
        if ((*i).lane == startLane) {
            myCurrentLaneInBestLanes = i;
        }
    }
}


const std::vector<MSLane*>&
MSVehicle::getBestLanesContinuation() const {
    if (myBestLanes.empty() || myBestLanes[0].empty()) {
        return myEmptyLaneVector;
    }
    return (*myCurrentLaneInBestLanes).bestContinuations;
}


const std::vector<MSLane*>&
MSVehicle::getBestLanesContinuation(const MSLane* const l) const {
    const MSLane* lane = l;
    // XXX: shouldn't this be a "while" to cover more than one internal lane? (Leo) Refs. #2575
    if (lane->getEdge().isInternal()) {
        // internal edges are not kept inside the bestLanes structure
        lane = lane->getLinkCont()[0]->getLane();
    }
    if (myBestLanes.size() == 0) {
        return myEmptyLaneVector;
    }
    for (std::vector<LaneQ>::const_iterator i = myBestLanes[0].begin(); i != myBestLanes[0].end(); ++i) {
        if ((*i).lane == lane) {
            return (*i).bestContinuations;
        }
    }
    return myEmptyLaneVector;
}

const std::vector<const MSLane*>
MSVehicle::getUpcomingLanesUntil(double distance) const {
    std::vector<const MSLane*> lanes;

    if (distance <= 0. || hasArrived()) {
        // WRITE_WARNINGF(TL("MSVehicle::getUpcomingLanesUntil(): distance ('%') should be greater than 0."), distance);
        return lanes;
    }

    if (!myLaneChangeModel->isOpposite()) {
        distance += getPositionOnLane();
    } else {
        distance += myLane->getOppositePos(getPositionOnLane());
    }
    MSLane* lane = myLaneChangeModel->isOpposite() ? myLane->getParallelOpposite() : myLane;
    while (lane->isInternal() && (distance > 0.)) {  // include initial internal lanes
        lanes.insert(lanes.end(), lane);
        distance -= lane->getLength();
        lane = lane->getLinkCont().front()->getViaLaneOrLane();
    }

    const std::vector<MSLane*>& contLanes = getBestLanesContinuation();
    if (contLanes.empty()) {
        return lanes;
    }
    auto contLanesIt = contLanes.begin();
    MSRouteIterator routeIt = myCurrEdge;  // keep track of covered edges in myRoute
    while (distance > 0.) {
        MSLane* l = nullptr;
        if (contLanesIt != contLanes.end()) {
            l = *contLanesIt;
            if (l != nullptr) {
                assert(l->getEdge().getID() == (*routeIt)->getLanes().front()->getEdge().getID());
            }
            ++contLanesIt;
            if (l != nullptr || myLane->isInternal()) {
                ++routeIt;
            }
            if (l == nullptr) {
                continue;
            }
        } else if (routeIt != myRoute->end()) {  // bestLanes didn't get us far enough
            // choose left-most lane as default (avoid sidewalks, bike lanes etc)
            l = (*routeIt)->getLanes().back();
            ++routeIt;
        } else {  // the search distance goes beyond our route
            break;
        }

        assert(l != nullptr);

        // insert internal lanes if applicable
        const MSLane* internalLane = lanes.size() > 0 ? lanes.back()->getInternalFollowingLane(l) : nullptr;
        while ((internalLane != nullptr) && internalLane->isInternal() && (distance > 0.)) {
            lanes.insert(lanes.end(), internalLane);
            distance -= internalLane->getLength();
            internalLane = internalLane->getLinkCont().front()->getViaLaneOrLane();
        }
        if (distance <= 0.) {
            break;
        }

        lanes.insert(lanes.end(), l);
        distance -= l->getLength();
    }

    return lanes;
}

const std::vector<const MSLane*>
MSVehicle::getPastLanesUntil(double distance) const {
    std::vector<const MSLane*> lanes;

    if (distance <= 0.) {
        // WRITE_WARNINGF(TL("MSVehicle::getPastLanesUntil(): distance ('%') should be greater than 0."), distance);
        return lanes;
    }

    MSRouteIterator routeIt = myCurrEdge;
    if (!myLaneChangeModel->isOpposite()) {
        distance += myLane->getLength() - getPositionOnLane();
    } else {
        distance += myLane->getParallelOpposite()->getLength() - myLane->getOppositePos(getPositionOnLane());
    }
    MSLane* lane = myLaneChangeModel->isOpposite() ? myLane->getParallelOpposite() : myLane;
    while (lane->isInternal() && (distance > 0.)) {  // include initial internal lanes
        lanes.insert(lanes.end(), lane);
        distance -= lane->getLength();
        lane = lane->getLogicalPredecessorLane();
    }

    while (distance > 0.) {
        // choose left-most lane as default (avoid sidewalks, bike lanes etc)
        MSLane* l = (*routeIt)->getLanes().back();

        // insert internal lanes if applicable
        const MSEdge* internalEdge = lanes.size() > 0 ? (*routeIt)->getInternalFollowingEdge(&(lanes.back()->getEdge()), getVClass()) : nullptr;
        const MSLane* internalLane = internalEdge != nullptr ? internalEdge->getLanes().front() : nullptr;
        std::vector<const MSLane*> internalLanes;
        while ((internalLane != nullptr) && internalLane->isInternal()) {  // collect all internal successor lanes
            internalLanes.insert(internalLanes.begin(), internalLane);
            internalLane = internalLane->getLinkCont().front()->getViaLaneOrLane();
        }
        for (auto it = internalLanes.begin(); (it != internalLanes.end()) && (distance > 0.); ++it) {  // check remaining distance in correct order
            lanes.insert(lanes.end(), *it);
            distance -= (*it)->getLength();
        }
        if (distance <= 0.) {
            break;
        }

        lanes.insert(lanes.end(), l);
        distance -= l->getLength();

        // NOTE: we're going backwards with the (bi-directional) Iterator
        // TODO: consider make reverse_iterator() when moving on to C++14 or later
        if (routeIt != myRoute->begin()) {
            --routeIt;
        } else {  // we went backwards to begin() and already processed the first and final element
            break;
        }
    }

    return lanes;
}


const std::vector<MSLane*>
MSVehicle::getUpstreamOppositeLanes() const {
    const std::vector<const MSLane*> routeLanes = getPastLanesUntil(myLane->getMaximumBrakeDist());
    std::vector<MSLane*> result;
    for (const MSLane* lane : routeLanes) {
        MSLane* opposite = lane->getOpposite();
        if (opposite != nullptr) {
            result.push_back(opposite);
        } else {
            break;
        }
    }
    return result;
}


int
MSVehicle::getBestLaneOffset() const {
    if (myBestLanes.empty() || myBestLanes[0].empty()) {
        return 0;
    } else {
        return (*myCurrentLaneInBestLanes).bestLaneOffset;
    }
}

double
MSVehicle::getBestLaneDist() const {
    if (myBestLanes.empty() || myBestLanes[0].empty()) {
        return -1;
    } else {
        return (*myCurrentLaneInBestLanes).length;
    }
}



void
MSVehicle::adaptBestLanesOccupation(int laneIndex, double density) {
    std::vector<MSVehicle::LaneQ>& preb = myBestLanes.front();
    assert(laneIndex < (int)preb.size());
    preb[laneIndex].occupation = density + preb[laneIndex].nextOccupation;
}


void
MSVehicle::fixPosition() {
    if (MSGlobals::gLaneChangeDuration > 0 && !myLaneChangeModel->isChangingLanes()) {
        myState.myPosLat = 0;
    }
}

std::pair<const MSLane*, double>
MSVehicle::getLanePosAfterDist(double distance) const {
    if (distance == 0) {
        return std::make_pair(myLane, getPositionOnLane());
    }
    const std::vector<const MSLane*> lanes = getUpcomingLanesUntil(distance);
    distance += getPositionOnLane();
    for (const MSLane* lane : lanes) {
        if (lane->getLength() > distance) {
            return std::make_pair(lane, distance);
        }
        distance -= lane->getLength();
    }
    return std::make_pair(nullptr, -1);
}


double
MSVehicle::getDistanceToPosition(double destPos, const MSLane* destLane) const {
    if (isOnRoad() && destLane != nullptr) {
        return myRoute->getDistanceBetween(getPositionOnLane(), destPos, myLane, destLane);
    }
    return std::numeric_limits<double>::max();
}


std::pair<const MSVehicle* const, double>
MSVehicle::getLeader(double dist) const {
    if (myLane == nullptr) {
        return std::make_pair(static_cast<const MSVehicle*>(nullptr), -1);
    }
    if (dist == 0) {
        dist = getCarFollowModel().brakeGap(getSpeed()) + getVehicleType().getMinGap();
    }
    const MSVehicle* lead = nullptr;
    const MSLane* lane = myLane; // ensure lane does not change between getVehiclesSecure and releaseVehicles;
    const MSLane::VehCont& vehs = lane->getVehiclesSecure();
    // vehicle might be outside the road network
    MSLane::VehCont::const_iterator it = std::find(vehs.begin(), vehs.end(), this);
    if (it != vehs.end() && it + 1 != vehs.end()) {
        lead = *(it + 1);
    }
    if (lead != nullptr) {
        std::pair<const MSVehicle* const, double> result(
            lead, lead->getBackPositionOnLane(myLane) - getPositionOnLane() - getVehicleType().getMinGap());
        lane->releaseVehicles();
        return result;
    }
    const double seen = myLane->getLength() - getPositionOnLane();
    const std::vector<MSLane*>& bestLaneConts = getBestLanesContinuation(myLane);
    std::pair<const MSVehicle* const, double> result = myLane->getLeaderOnConsecutive(dist, seen, getSpeed(), *this, bestLaneConts);
    lane->releaseVehicles();
    return result;
}


std::pair<const MSVehicle* const, double>
MSVehicle::getFollower(double dist) const {
    if (myLane == nullptr) {
        return std::make_pair(static_cast<const MSVehicle*>(nullptr), -1);
    }
    if (dist == 0) {
        dist = getCarFollowModel().brakeGap(myLane->getEdge().getSpeedLimit() * 2, 4.5, 0);
    }
    return myLane->getFollower(this, getPositionOnLane(), dist, MSLane::MinorLinkMode::FOLLOW_NEVER);
}


double
MSVehicle::getTimeGapOnLane() const {
    // calling getLeader with 0 would induce a dist calculation but we only want to look for the leaders on the current lane
    std::pair<const MSVehicle* const, double> leaderInfo = getLeader(-1);
    if (leaderInfo.first == nullptr || getSpeed() == 0) {
        return -1;
    }
    return (leaderInfo.second + getVehicleType().getMinGap()) / getSpeed();
}


void
MSVehicle::addTransportable(MSTransportable* transportable) {
    MSBaseVehicle::addTransportable(transportable);
    if (myStops.size() > 0 && myStops.front().reached) {
        if (transportable->isPerson()) {
            if (myStops.front().triggered && myStops.front().numExpectedPerson > 0) {
                myStops.front().numExpectedPerson -= (int)myStops.front().pars.awaitedPersons.count(transportable->getID());
            }
        } else {
            if (myStops.front().pars.containerTriggered && myStops.front().numExpectedContainer > 0) {
                myStops.front().numExpectedContainer -= (int)myStops.front().pars.awaitedContainers.count(transportable->getID());
            }
        }
    }
}


void
MSVehicle::setBlinkerInformation() {
    switchOffSignal(VEH_SIGNAL_BLINKER_RIGHT | VEH_SIGNAL_BLINKER_LEFT);
    int state = myLaneChangeModel->getOwnState();
    // do not set blinker for sublane changes or when blocked from changing to the right
    const bool blinkerManoeuvre = (((state & LCA_SUBLANE) == 0) && (
                                       (state & LCA_KEEPRIGHT) == 0 || (state & LCA_BLOCKED) == 0));
    Signalling left = VEH_SIGNAL_BLINKER_LEFT;
    Signalling right = VEH_SIGNAL_BLINKER_RIGHT;
    if (MSGlobals::gLefthand) {
        // lane indices increase from left to right
        std::swap(left, right);
    }
    if ((state & LCA_LEFT) != 0 && blinkerManoeuvre) {
        switchOnSignal(left);
    } else if ((state & LCA_RIGHT) != 0 && blinkerManoeuvre) {
        switchOnSignal(right);
    } else if (myLaneChangeModel->isChangingLanes()) {
        if (myLaneChangeModel->getLaneChangeDirection() == 1) {
            switchOnSignal(left);
        } else {
            switchOnSignal(right);
        }
    } else {
        const MSLane* lane = getLane();
        std::vector<MSLink*>::const_iterator link = MSLane::succLinkSec(*this, 1, *lane, getBestLanesContinuation());
        if (link != lane->getLinkCont().end() && lane->getLength() - getPositionOnLane() < lane->getVehicleMaxSpeed(this) * (double) 7.) {
            switch ((*link)->getDirection()) {
                case LinkDirection::TURN:
                case LinkDirection::LEFT:
                case LinkDirection::PARTLEFT:
                    switchOnSignal(VEH_SIGNAL_BLINKER_LEFT);
                    break;
                case LinkDirection::RIGHT:
                case LinkDirection::PARTRIGHT:
                    switchOnSignal(VEH_SIGNAL_BLINKER_RIGHT);
                    break;
                default:
                    break;
            }
        }
    }
    // stopping related signals
    if (hasStops()
            && (myStops.begin()->reached ||
                (myStopDist < (myLane->getLength() - getPositionOnLane())
                 && myStopDist < getCarFollowModel().brakeGap(myLane->getVehicleMaxSpeed(this), getCarFollowModel().getMaxDecel(), 3)))) {
        if (myStops.begin()->lane->getIndex() > 0 && myStops.begin()->lane->getParallelLane(-1)->allowsVehicleClass(getVClass())) {
            // not stopping on the right. Activate emergency blinkers
            switchOnSignal(VEH_SIGNAL_BLINKER_LEFT | VEH_SIGNAL_BLINKER_RIGHT);
        } else if (!myStops.begin()->reached && (myStops.begin()->pars.parking == ParkingType::OFFROAD)) {
            // signal upcoming parking stop on the current lane when within braking distance (~2 seconds before braking)
            switchOnSignal(MSGlobals::gLefthand ? VEH_SIGNAL_BLINKER_LEFT : VEH_SIGNAL_BLINKER_RIGHT);
        }
    }
    if (myInfluencer != nullptr && myInfluencer->getSignals() >= 0) {
        mySignals = myInfluencer->getSignals();
        myInfluencer->setSignals(-1); // overwrite computed signals only once
    }
}

void
MSVehicle::setEmergencyBlueLight(SUMOTime currentTime) {

    //TODO look if timestep ist SIMSTEP
    if (currentTime % 1000 == 0) {
        if (signalSet(VEH_SIGNAL_EMERGENCY_BLUE)) {
            switchOffSignal(VEH_SIGNAL_EMERGENCY_BLUE);
        } else {
            switchOnSignal(VEH_SIGNAL_EMERGENCY_BLUE);
        }
    }
}


int
MSVehicle::getLaneIndex() const {
    return myLane == nullptr ? -1 : myLane->getIndex();
}


void
MSVehicle::setTentativeLaneAndPosition(MSLane* lane, double pos, double posLat) {
    myLane = lane;
    myState.myPos = pos;
    myState.myPosLat = posLat;
    myState.myBackPos = pos - getVehicleType().getLength();
}


double
MSVehicle::getRightSideOnLane() const {
    return myState.myPosLat + 0.5 * myLane->getWidth() - 0.5 * getVehicleType().getWidth();
}


double
MSVehicle::getLeftSideOnLane() const {
    return myState.myPosLat + 0.5 * myLane->getWidth() + 0.5 * getVehicleType().getWidth();
}


double
MSVehicle::getRightSideOnLane(const MSLane* lane) const {
    return myState.myPosLat + 0.5 * lane->getWidth() - 0.5 * getVehicleType().getWidth();
}


double
MSVehicle::getLeftSideOnLane(const MSLane* lane) const {
    return myState.myPosLat + 0.5 * lane->getWidth() + 0.5 * getVehicleType().getWidth();
}


double
MSVehicle::getRightSideOnEdge(const MSLane* lane) const {
    return getCenterOnEdge(lane) - 0.5 * getVehicleType().getWidth();
}


double
MSVehicle::getLeftSideOnEdge(const MSLane* lane) const {
    return getCenterOnEdge(lane) + 0.5 * getVehicleType().getWidth();
}


double
MSVehicle::getCenterOnEdge(const MSLane* lane) const {
    if (lane == nullptr || &lane->getEdge() == &myLane->getEdge()) {
        return myLane->getRightSideOnEdge() + myState.myPosLat + 0.5 * myLane->getWidth();
    } else if (lane == myLaneChangeModel->getShadowLane()) {
        if (myLaneChangeModel->isOpposite() && &lane->getEdge() != &myLane->getEdge()) {
            return lane->getRightSideOnEdge() + lane->getWidth() - myState.myPosLat + 0.5 * myLane->getWidth();
        }
        if (myLaneChangeModel->getShadowDirection() == -1) {
            return lane->getRightSideOnEdge() + lane->getWidth() + myState.myPosLat + 0.5 * myLane->getWidth();
        } else {
            return lane->getRightSideOnEdge() - myLane->getWidth() + myState.myPosLat + 0.5 * myLane->getWidth();
        }
    } else if (lane == myLane->getBidiLane()) {
        return lane->getRightSideOnEdge() - myState.myPosLat + 0.5 * lane->getWidth();
    } else {
        assert(myFurtherLanes.size() == myFurtherLanesPosLat.size());
        for (int i = 0; i < (int)myFurtherLanes.size(); ++i) {
            if (myFurtherLanes[i] == lane) {
#ifdef DEBUG_FURTHER
                if (DEBUG_COND) std::cout << "    getCenterOnEdge veh=" << getID() << " lane=" << lane->getID() << " i=" << i << " furtherLat=" << myFurtherLanesPosLat[i]
                                              << " result=" << lane->getRightSideOnEdge() + myFurtherLanesPosLat[i] + 0.5 * lane->getWidth()
                                              << "\n";
#endif
                return lane->getRightSideOnEdge() + myFurtherLanesPosLat[i] + 0.5 * lane->getWidth();
            } else if (myFurtherLanes[i]->getBidiLane() == lane) {
#ifdef DEBUG_FURTHER
                if (DEBUG_COND) std::cout << "    getCenterOnEdge veh=" << getID() << " lane=" << lane->getID() << " i=" << i << " furtherLat(bidi)=" << myFurtherLanesPosLat[i]
                                              << " result=" << lane->getRightSideOnEdge() + myFurtherLanesPosLat[i] + 0.5 * lane->getWidth()
                                              << "\n";
#endif
                return lane->getRightSideOnEdge() - myFurtherLanesPosLat[i] + 0.5 * lane->getWidth();
            }
        }
        //if (DEBUG_COND) std::cout << SIMTIME << " veh=" << getID() << " myShadowFurtherLanes=" << toString(myLaneChangeModel->getShadowFurtherLanes()) << "\n";
        const std::vector<MSLane*>& shadowFurther = myLaneChangeModel->getShadowFurtherLanes();
        for (int i = 0; i < (int)shadowFurther.size(); ++i) {
            //if (DEBUG_COND) std::cout << " comparing i=" << (*i)->getID() << " lane=" << lane->getID() << "\n";
            if (shadowFurther[i] == lane) {
                assert(myLaneChangeModel->getShadowLane() != 0);
                return (lane->getRightSideOnEdge() + myLaneChangeModel->getShadowFurtherLanesPosLat()[i] + 0.5 * lane->getWidth()
                        + (myLane->getCenterOnEdge() - myLaneChangeModel->getShadowLane()->getCenterOnEdge()));
            }
        }
        assert(false);
        throw ProcessError("Request lateral pos of vehicle '" + getID() + "' for invalid lane '" + Named::getIDSecure(lane) + "'");
    }
}


double
MSVehicle::getLatOffset(const MSLane* lane) const {
    assert(lane != 0);
    if (&lane->getEdge() == &myLane->getEdge()) {
        return myLane->getRightSideOnEdge() - lane->getRightSideOnEdge();
    } else if (myLane->getParallelOpposite() == lane) {
        return (myLane->getWidth() + lane->getWidth()) * 0.5 - 2 * getLateralPositionOnLane();
    } else if (myLane->getBidiLane() == lane) {
        return -2 * getLateralPositionOnLane();
    } else {
        // Check whether the lane is a further lane for the vehicle
        for (int i = 0; i < (int)myFurtherLanes.size(); ++i) {
            if (myFurtherLanes[i] == lane) {
#ifdef DEBUG_FURTHER
                if (DEBUG_COND) {
                    std::cout << "    getLatOffset veh=" << getID() << " lane=" << lane->getID() << " i=" << i << " posLat=" << myState.myPosLat << " furtherLat=" << myFurtherLanesPosLat[i] << "\n";
                }
#endif
                return myFurtherLanesPosLat[i] - myState.myPosLat;
            } else if (myFurtherLanes[i]->getBidiLane() == lane) {
#ifdef DEBUG_FURTHER
                if (DEBUG_COND) {
                    std::cout << "    getLatOffset veh=" << getID() << " lane=" << lane->getID() << " i=" << i << " posLat=" << myState.myPosLat << " furtherBidiLat=" << myFurtherLanesPosLat[i] << "\n";
                }
#endif
                return -2 * (myFurtherLanesPosLat[i] - myState.myPosLat);
            }
        }
#ifdef DEBUG_FURTHER
        if (DEBUG_COND) {
            std::cout << SIMTIME << " veh=" << getID() << " myShadowFurtherLanes=" << toString(myLaneChangeModel->getShadowFurtherLanes()) << "\n";
        }
#endif
        // Check whether the lane is a "shadow further lane" for the vehicle
        const std::vector<MSLane*>& shadowFurther = myLaneChangeModel->getShadowFurtherLanes();
        for (int i = 0; i < (int)shadowFurther.size(); ++i) {
            if (shadowFurther[i] == lane) {
#ifdef DEBUG_FURTHER
                if (DEBUG_COND) std::cout << "    getLatOffset veh=" << getID()
                                              << " shadowLane=" << Named::getIDSecure(myLaneChangeModel->getShadowLane())
                                              << " lane=" << lane->getID()
                                              << " i=" << i
                                              << " posLat=" << myState.myPosLat
                                              << " shadowPosLat=" << getLatOffset(myLaneChangeModel->getShadowLane())
                                              << " shadowFurtherLat=" << myLaneChangeModel->getShadowFurtherLanesPosLat()[i]
                                              <<  "\n";
#endif
                return getLatOffset(myLaneChangeModel->getShadowLane()) + myLaneChangeModel->getShadowFurtherLanesPosLat()[i] - myState.myPosLat;
            }
        }
        // Check whether the vehicle issued a maneuverReservation on the lane.
        const std::vector<MSLane*>& furtherTargets = myLaneChangeModel->getFurtherTargetLanes();
        for (int i = 0; i < (int)myFurtherLanes.size(); ++i) {
            // Further target lanes are just neighboring lanes of the vehicle's further lanes, @see MSAbstractLaneChangeModel::updateTargetLane()
            MSLane* targetLane = furtherTargets[i];
            if (targetLane == lane) {
                const double targetDir = myLaneChangeModel->getManeuverDist() < 0 ? -1. : 1.;
                const double latOffset = myFurtherLanesPosLat[i] - myState.myPosLat + targetDir * 0.5 * (myFurtherLanes[i]->getWidth() + targetLane->getWidth());
#ifdef DEBUG_TARGET_LANE
                if (DEBUG_COND) {
                    std::cout << "    getLatOffset veh=" << getID()
                              << " wrt targetLane=" << Named::getIDSecure(myLaneChangeModel->getTargetLane())
                              << "\n    i=" << i
                              << " posLat=" << myState.myPosLat
                              << " furtherPosLat=" << myFurtherLanesPosLat[i]
                              << " maneuverDist=" << myLaneChangeModel->getManeuverDist()
                              << " targetDir=" << targetDir
                              << " latOffset=" << latOffset
                              <<  std::endl;
                }
#endif
                return latOffset;
            }
        }
        assert(false);
        throw ProcessError("Request lateral offset of vehicle '" + getID() + "' for invalid lane '" + Named::getIDSecure(lane) + "'");
    }
}


double
MSVehicle::lateralDistanceToLane(const int offset) const {
    // compute the distance when changing to the neighboring lane
    // (ensure we do not lap into the line behind neighLane since there might be unseen blockers)
    assert(offset == 0 || offset == 1 || offset == -1);
    assert(myLane != nullptr);
    assert(myLane->getParallelLane(offset) != nullptr || myLane->getParallelOpposite() != nullptr);
    const double halfCurrentLaneWidth = 0.5 * myLane->getWidth();
    const double halfVehWidth = 0.5 * (getWidth() + NUMERICAL_EPS);
    const double latPos = getLateralPositionOnLane();
    const double oppositeSign = getLaneChangeModel().isOpposite() ? -1 : 1;
    double leftLimit = halfCurrentLaneWidth - halfVehWidth - oppositeSign * latPos;
    double rightLimit = -halfCurrentLaneWidth + halfVehWidth - oppositeSign * latPos;
    double latLaneDist = 0;  // minimum distance to move the vehicle fully onto the new lane
    if (offset == 0) {
        if (latPos + halfVehWidth > halfCurrentLaneWidth) {
            // correct overlapping left
            latLaneDist = halfCurrentLaneWidth - latPos - halfVehWidth;
        } else if (latPos - halfVehWidth < -halfCurrentLaneWidth) {
            // correct overlapping right
            latLaneDist = -halfCurrentLaneWidth - latPos + halfVehWidth;
        }
        latLaneDist *= oppositeSign;
    } else if (offset == -1) {
        latLaneDist = rightLimit - (getWidth() + NUMERICAL_EPS);
    } else if (offset == 1) {
        latLaneDist = leftLimit + (getWidth() + NUMERICAL_EPS);
    }
#ifdef DEBUG_ACTIONSTEPS
    if (DEBUG_COND) {
        std::cout << SIMTIME
                  << " veh=" << getID()
                  << " halfCurrentLaneWidth=" << halfCurrentLaneWidth
                  << " halfVehWidth=" << halfVehWidth
                  << " latPos=" << latPos
                  << " latLaneDist=" << latLaneDist
                  << " leftLimit=" << leftLimit
                  << " rightLimit=" << rightLimit
                  << "\n";
    }
#endif
    return latLaneDist;
}


double
MSVehicle::getLateralOverlap(double posLat, const MSLane* lane) const {
    return (fabs(posLat) + 0.5 * getVehicleType().getWidth()
            - 0.5 * lane->getWidth());
}

double
MSVehicle::getLateralOverlap(const MSLane* lane) const {
    return getLateralOverlap(getLateralPositionOnLane(), lane);
}

double
MSVehicle::getLateralOverlap() const {
    return getLateralOverlap(getLateralPositionOnLane(), myLane);
}


void
MSVehicle::removeApproachingInformation(const DriveItemVector& lfLinks) const {
    for (const DriveProcessItem& dpi : lfLinks) {
        if (dpi.myLink != nullptr) {
            dpi.myLink->removeApproaching(this);
        }
    }
    // unregister on all shadow links
    myLaneChangeModel->removeShadowApproachingInformation();
}


bool
MSVehicle::unsafeLinkAhead(const MSLane* lane) const {
    // the following links are unsafe:
    // - zipper links if they are close enough and have approaching vehicles in the relevant time range
    // - unprioritized links if the vehicle is currently approaching a prioritzed link and unable to stop in time
    double seen = myLane->getLength() - getPositionOnLane();
    const double dist = getCarFollowModel().brakeGap(getSpeed(), getCarFollowModel().getMaxDecel(), 0);
    if (seen < dist) {
        const std::vector<MSLane*>& bestLaneConts = getBestLanesContinuation(lane);
        int view = 1;
        std::vector<MSLink*>::const_iterator link = MSLane::succLinkSec(*this, view, *lane, bestLaneConts);
        DriveItemVector::const_iterator di = myLFLinkLanes.begin();
        while (!lane->isLinkEnd(link) && seen <= dist) {
            if (!lane->getEdge().isInternal()
                    && (((*link)->getState() == LINKSTATE_ZIPPER && seen < (*link)->getFoeVisibilityDistance())
                        || !(*link)->havePriority())) {
                // find the drive item corresponding to this link
                bool found = false;
                while (di != myLFLinkLanes.end() && !found) {
                    if ((*di).myLink != nullptr) {
                        const MSLane* diPredLane = (*di).myLink->getLaneBefore();
                        if (diPredLane != nullptr) {
                            if (&diPredLane->getEdge() == &lane->getEdge()) {
                                found = true;
                            }
                        }
                    }
                    if (!found) {
                        di++;
                    }
                }
                if (found) {
                    const SUMOTime leaveTime = (*link)->getLeaveTime((*di).myArrivalTime, (*di).myArrivalSpeed,
                                               (*di).getLeaveSpeed(), getVehicleType().getLength());
                    if ((*link)->hasApproachingFoe((*di).myArrivalTime, leaveTime, (*di).myArrivalSpeed, getCarFollowModel().getMaxDecel())) {
                        //std::cout << SIMTIME << " veh=" << getID() << " aborting changeTo=" << Named::getIDSecure(bestLaneConts.front()) << " linkState=" << toString((*link)->getState()) << " seen=" << seen << " dist=" << dist << "\n";
                        return true;
                    }
                }
                // no drive item is found if the vehicle aborts its request within dist
            }
            lane = (*link)->getViaLaneOrLane();
            if (!lane->getEdge().isInternal()) {
                view++;
            }
            seen += lane->getLength();
            link = MSLane::succLinkSec(*this, view, *lane, bestLaneConts);
        }
    }
    return false;
}


PositionVector
MSVehicle::getBoundingBox(double offset) const {
    PositionVector centerLine;
    Position pos = getPosition();
    centerLine.push_back(pos);
    switch (myType->getGuiShape()) {
        case SUMOVehicleShape::BUS_FLEXIBLE:
        case SUMOVehicleShape::RAIL:
        case SUMOVehicleShape::RAIL_CAR:
        case SUMOVehicleShape::RAIL_CARGO:
        case SUMOVehicleShape::TRUCK_SEMITRAILER:
        case SUMOVehicleShape::TRUCK_1TRAILER: {
            for (MSLane* lane : myFurtherLanes) {
                centerLine.push_back(lane->getShape().back());
            }
            break;
        }
        default:
            break;
    }
    double l = getLength();
    Position backPos = getBackPosition();
    if (pos.distanceTo2D(backPos) > l + NUMERICAL_EPS) {
        // getBackPosition may not match the visual back in networks without internal lanes
        double a = getAngle() + M_PI; // angle pointing backwards
        backPos = pos + Position(l * cos(a), l * sin(a));
    }
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


PositionVector
MSVehicle::getBoundingPoly(double offset) const {
    switch (myType->getGuiShape()) {
        case SUMOVehicleShape::PASSENGER:
        case SUMOVehicleShape::PASSENGER_SEDAN:
        case SUMOVehicleShape::PASSENGER_HATCHBACK:
        case SUMOVehicleShape::PASSENGER_WAGON:
        case SUMOVehicleShape::PASSENGER_VAN: {
            // box with corners cut off
            PositionVector result;
            PositionVector centerLine;
            centerLine.push_back(getPosition());
            centerLine.push_back(getBackPosition());
            if (offset != 0) {
                centerLine.extrapolate2D(offset);
            }
            PositionVector line1 = centerLine;
            PositionVector line2 = centerLine;
            line1.move2side(MAX2(0.0, 0.3 * myType->getWidth() + offset));
            line2.move2side(MAX2(0.0, 0.5 * myType->getWidth() + offset));
            line2.scaleRelative(0.8);
            result.push_back(line1[0]);
            result.push_back(line2[0]);
            result.push_back(line2[1]);
            result.push_back(line1[1]);
            line1.move2side(MIN2(0.0, -0.6 * myType->getWidth() - offset));
            line2.move2side(MIN2(0.0, -1.0 * myType->getWidth() - offset));
            result.push_back(line1[1]);
            result.push_back(line2[1]);
            result.push_back(line2[0]);
            result.push_back(line1[0]);
            return result;
        }
        default:
            return getBoundingBox();
    }
}


bool
MSVehicle::onFurtherEdge(const MSEdge* edge) const {
    for (std::vector<MSLane*>::const_iterator i = myFurtherLanes.begin(); i != myFurtherLanes.end(); ++i) {
        if (&(*i)->getEdge() == edge) {
            return true;
        }
    }
    return false;
}


bool
MSVehicle::isBidiOn(const MSLane* lane) const {
    return lane->getBidiLane() != nullptr && (
               myLane == lane->getBidiLane()
               || onFurtherEdge(&lane->getBidiLane()->getEdge()));
}


bool
MSVehicle::rerouteParkingArea(const std::string& parkingAreaID, std::string& errorMsg) {
    // this function is based on MSTriggeredRerouter::rerouteParkingArea in order to keep
    // consistency in the behaviour.

    // get vehicle params
    MSParkingArea* destParkArea = getNextParkingArea();
    const MSRoute& route = getRoute();
    const MSEdge* lastEdge = route.getLastEdge();

    if (destParkArea == nullptr) {
        // not driving towards a parking area
        errorMsg = "Vehicle " + getID() + " is not driving to a parking area so it cannot be rerouted.";
        return false;
    }

    // if the current route ends at the parking area, the new route will also and at the new area
    bool newDestination = (&destParkArea->getLane().getEdge() == route.getLastEdge()
                           && getArrivalPos() >= destParkArea->getBeginLanePosition()
                           && getArrivalPos() <= destParkArea->getEndLanePosition());

    // retrieve info on the new parking area
    MSParkingArea* newParkingArea = (MSParkingArea*) MSNet::getInstance()->getStoppingPlace(
                                        parkingAreaID, SumoXMLTag::SUMO_TAG_PARKING_AREA);

    if (newParkingArea == nullptr) {
        errorMsg = "Parking area ID " + toString(parkingAreaID) + " not found in the network.";
        return false;
    }

    const MSEdge* newEdge = &(newParkingArea->getLane().getEdge());
    SUMOAbstractRouter<MSEdge, SUMOVehicle>& router = getRouterTT();

    // Compute the route from the current edge to the parking area edge
    ConstMSEdgeVector edgesToPark;
    router.compute(getEdge(), newEdge, this, MSNet::getInstance()->getCurrentTimeStep(), edgesToPark);

    // Compute the route from the parking area edge to the end of the route
    ConstMSEdgeVector edgesFromPark;
    if (!newDestination) {
        router.compute(newEdge, lastEdge, this, MSNet::getInstance()->getCurrentTimeStep(), edgesFromPark);
    } else {
        // adapt plans of any riders
        for (MSTransportable* p : getPersons()) {
            p->rerouteParkingArea(getNextParkingArea(), newParkingArea);
        }
    }

    // we have a new destination, let's replace the vehicle route
    ConstMSEdgeVector edges = edgesToPark;
    if (edgesFromPark.size() > 0) {
        edges.insert(edges.end(), edgesFromPark.begin() + 1, edgesFromPark.end());
    }

    if (newDestination) {
        SUMOVehicleParameter* newParameter = new SUMOVehicleParameter();
        *newParameter = getParameter();
        newParameter->arrivalPosProcedure = ArrivalPosDefinition::GIVEN;
        newParameter->arrivalPos = newParkingArea->getEndLanePosition();
        replaceParameter(newParameter);
    }
    const double routeCost = router.recomputeCosts(edges, this, MSNet::getInstance()->getCurrentTimeStep());
    ConstMSEdgeVector prevEdges(myCurrEdge, myRoute->end());
    const double savings = router.recomputeCosts(prevEdges, this, MSNet::getInstance()->getCurrentTimeStep());
    if (replaceParkingArea(newParkingArea, errorMsg)) {
        const bool onInit = myLane == nullptr;
        replaceRouteEdges(edges, routeCost, savings, "TraCI:" + toString(SUMO_TAG_PARKING_AREA_REROUTE), onInit, false, false);
    } else {
        WRITE_WARNING("Vehicle '" + getID() + "' could not reroute to new parkingArea '" + newParkingArea->getID()
                      + "' reason=" + errorMsg + ", time=" + time2string(MSNet::getInstance()->getCurrentTimeStep()) + ".");
        return false;
    }
    return true;
}


bool
MSVehicle::addTraciStop(SUMOVehicleParameter::Stop stop, std::string& errorMsg) {
    const int numStops = (int)myStops.size();
    const bool result = MSBaseVehicle::addTraciStop(stop, errorMsg);
    if (myLane != nullptr && numStops != (int)myStops.size()) {
        updateBestLanes(true);
    }
    return result;
}


bool
MSVehicle::handleCollisionStop(MSStop& stop, const double distToStop) {
    if (myCurrEdge == stop.edge && distToStop + POSITION_EPS < getCarFollowModel().brakeGap(myState.mySpeed, getCarFollowModel().getMaxDecel(), 0)) {
        if (distToStop < getCarFollowModel().brakeGap(myState.mySpeed, getCarFollowModel().getEmergencyDecel(), 0)) {
            double vNew = getCarFollowModel().maximumSafeStopSpeed(distToStop, getCarFollowModel().getMaxDecel(), getSpeed(), false, 0);
            //std::cout << SIMTIME << " veh=" << getID() << " v=" << myState.mySpeed << " distToStop=" << distToStop
            //    << " vMinNex=" << getCarFollowModel().minNextSpeed(getSpeed(), this)
            //    << " bg1=" << getCarFollowModel().brakeGap(myState.mySpeed)
            //    << " bg2=" << getCarFollowModel().brakeGap(myState.mySpeed, getCarFollowModel().getEmergencyDecel(), 0)
            //    << " vNew=" << vNew
            //    << "\n";
            myState.mySpeed = MIN2(myState.mySpeed, vNew + ACCEL2SPEED(getCarFollowModel().getEmergencyDecel()));
            myState.myPos = MIN2(myState.myPos, stop.pars.endPos);
            myCachedPosition = Position::INVALID;
            if (myState.myPos < myType->getLength()) {
                computeFurtherLanes(myLane, myState.myPos, true);
                myAngle = computeAngle();
                if (myLaneChangeModel->isOpposite()) {
                    myAngle += M_PI;
                }
            }
        }
    }
    return true;
}


bool
MSVehicle::resumeFromStopping() {
    if (isStopped()) {
        if (myAmRegisteredAsWaiting) {
            MSNet::getInstance()->getVehicleControl().unregisterOneWaiting();
            myAmRegisteredAsWaiting = false;
        }
        MSStop& stop = myStops.front();
        // we have waited long enough and fulfilled any passenger-requirements
        if (stop.busstop != nullptr) {
            // inform bus stop about leaving it
            stop.busstop->leaveFrom(this);
        }
        // we have waited long enough and fulfilled any container-requirements
        if (stop.containerstop != nullptr) {
            // inform container stop about leaving it
            stop.containerstop->leaveFrom(this);
        }
        if (stop.parkingarea != nullptr && stop.getSpeed() <= 0) {
            // inform parking area about leaving it
            stop.parkingarea->leaveFrom(this);
        }
        if (stop.chargingStation != nullptr) {
            // inform charging station about leaving it
            stop.chargingStation->leaveFrom(this);
        }
        // the current stop is no longer valid
        myLane->getEdge().removeWaiting(this);
        // MSStopOut needs to know whether the stop had a loaded 'ended' value so we call this before replacing the value
        if (stop.pars.started == -1) {
            // waypoint edge was passed in a single step
            stop.pars.started = MSNet::getInstance()->getCurrentTimeStep();
        }
        if (MSStopOut::active()) {
            MSStopOut::getInstance()->stopEnded(this, stop.pars, stop.lane->getID());
        }
        stop.pars.ended = MSNet::getInstance()->getCurrentTimeStep();
        for (const auto& rem : myMoveReminders) {
            rem.first->notifyStopEnded();
        }
        if (stop.pars.collision && MSLane::getCollisionAction() == MSLane::COLLISION_ACTION_WARN) {
            myCollisionImmunity = TIME2STEPS(5); // leave the conflict area
        }
        if (stop.pars.posLat != INVALID_DOUBLE && MSGlobals::gLateralResolution <= 0) {
            // reset lateral position to default
            myState.myPosLat = 0;
        }
        myPastStops.push_back(stop.pars);
        myPastStops.back().routeIndex = (int)(stop.edge - myRoute->begin());
        myStops.pop_front();
        myStopDist = std::numeric_limits<double>::max();
        // do not count the stopping time towards gridlock time.
        // Other outputs use an independent counter and are not affected.
        myWaitingTime = 0;
        // maybe the next stop is on the same edge; let's rebuild best lanes
        updateBestLanes(true);
        // continue as wished...
        MSNet::getInstance()->informVehicleStateListener(this, MSNet::VehicleState::ENDING_STOP);
        MSNet::getInstance()->getVehicleControl().registerStopEnded();
        return true;
    }
    return false;
}


MSVehicle::Influencer&
MSVehicle::getInfluencer() {
    if (myInfluencer == nullptr) {
        myInfluencer = new Influencer();
    }
    return *myInfluencer;
}

MSVehicle::BaseInfluencer&
MSVehicle::getBaseInfluencer() {
    return getInfluencer();
}


const MSVehicle::Influencer*
MSVehicle::getInfluencer() const {
    return myInfluencer;
}

const MSVehicle::BaseInfluencer*
MSVehicle::getBaseInfluencer() const {
    return myInfluencer;
}


double
MSVehicle::getSpeedWithoutTraciInfluence() const {
    if (myInfluencer != nullptr && myInfluencer->getOriginalSpeed() >= 0) {
        // influencer original speed is -1 on initialization
        return myInfluencer->getOriginalSpeed();
    }
    return myState.mySpeed;
}


int
MSVehicle::influenceChangeDecision(int state) {
    if (hasInfluencer()) {
        state = getInfluencer().influenceChangeDecision(
                    MSNet::getInstance()->getCurrentTimeStep(),
                    myLane->getEdge(),
                    getLaneIndex(),
                    state);
    }
    return state;
}


void
MSVehicle::setRemoteState(Position xyPos) {
    myCachedPosition = xyPos;
}


bool
MSVehicle::isRemoteControlled() const {
    return myInfluencer != nullptr && myInfluencer->isRemoteControlled();
}


bool
MSVehicle::wasRemoteControlled(SUMOTime lookBack) const {
    return myInfluencer != nullptr && myInfluencer->getLastAccessTimeStep() + lookBack >= MSNet::getInstance()->getCurrentTimeStep();
}


bool
MSVehicle::keepClear(const MSLink* link) const {
    if (link->hasFoes() && link->keepClear() /* && item.myLink->willHaveBlockedFoe()*/) {
        const double keepClearTime = getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_IGNORE_KEEPCLEAR_TIME, -1);
        //std::cout << SIMTIME << " veh=" << getID() << " keepClearTime=" << keepClearTime << " accWait=" << getAccumulatedWaitingSeconds() << " keepClear=" << (keepClearTime < 0 || getAccumulatedWaitingSeconds() < keepClearTime) << "\n";
        return keepClearTime < 0 || getAccumulatedWaitingSeconds() < keepClearTime;
    } else {
        return false;
    }
}


bool
MSVehicle::ignoreRed(const MSLink* link, bool canBrake) const {
    if ((myInfluencer != nullptr && !myInfluencer->getEmergencyBrakeRedLight())) {
        return true;
    }
    const double ignoreRedTime = getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_DRIVE_AFTER_RED_TIME, -1);
#ifdef DEBUG_IGNORE_RED
    if (DEBUG_COND) {
        std::cout << SIMTIME << " veh=" << getID() << " link=" << link->getViaLaneOrLane()->getID() << " state=" << toString(link->getState()) << "\n";
    }
#endif
    if (ignoreRedTime < 0) {
        const double ignoreYellowTime = getVehicleType().getParameter().getJMParam(SUMO_ATTR_JM_DRIVE_AFTER_YELLOW_TIME, 0);
        if (ignoreYellowTime > 0 && link->haveYellow()) {
            assert(link->getTLLogic() != 0);
            const double yellowDuration = STEPS2TIME(MSNet::getInstance()->getCurrentTimeStep() - link->getLastStateChange());
            // when activating ignoreYellow behavior, vehicles will drive if they cannot brake
            return !canBrake || ignoreYellowTime > yellowDuration;
        } else {
            return false;
        }
    } else if (link->haveYellow()) {
        // always drive at yellow when ignoring red
        return true;
    } else if (link->haveRed()) {
        assert(link->getTLLogic() != 0);
        const double redDuration = STEPS2TIME(MSNet::getInstance()->getCurrentTimeStep() - link->getLastStateChange());
#ifdef DEBUG_IGNORE_RED
        if (DEBUG_COND) {
            std::cout
            // << SIMTIME << " veh=" << getID() << " link=" << link->getViaLaneOrLane()->getID()
                    << "   ignoreRedTime=" << ignoreRedTime
                    << " spentRed=" << redDuration
                    << " canBrake=" << canBrake << "\n";
        }
#endif
        // when activating ignoreRed behavior, vehicles will always drive if they cannot brake
        return !canBrake || ignoreRedTime > redDuration;
    } else {
        return false;
    }
}

bool
MSVehicle::ignoreFoe(const SUMOTrafficObject* foe) const {
    if (!getParameter().wasSet(VEHPARS_CFMODEL_PARAMS_SET)) {
        return false;
    }
    for (const std::string& typeID : StringTokenizer(getParameter().getParameter(toString(SUMO_ATTR_CF_IGNORE_TYPES), "")).getVector()) {
        if (typeID == foe->getVehicleType().getID()) {
            return true;
        }
    }
    for (const std::string& id : StringTokenizer(getParameter().getParameter(toString(SUMO_ATTR_CF_IGNORE_IDS), "")).getVector()) {
        if (id == foe->getID()) {
            return true;
        }
    }
    return false;
}

bool
MSVehicle::passingMinor() const {
    // either on an internal lane that was entered via minor link
    // or on approach to minor link below visibility distance
    if (myLane == nullptr) {
        return false;
    }
    if (myLane->getEdge().isInternal()) {
        return !myLane->getIncomingLanes().front().viaLink->havePriority();
    } else if (myLFLinkLanes.size() > 0 && myLFLinkLanes.front().myLink != nullptr) {
        MSLink* link = myLFLinkLanes.front().myLink;
        return !link->havePriority() && myLFLinkLanes.front().myDistance <= link->getFoeVisibilityDistance();
    }
    return false;
}

bool
MSVehicle::isLeader(const MSLink* link, const MSVehicle* veh, const double gap) const {
    assert(link->fromInternalLane());
    if (veh == nullptr) {
        return false;
    }
    if (!myLane->isInternal() || myLane->getEdge().getToJunction() != link->getJunction()) {
        // if this vehicle is not yet on the junction, every vehicle is a leader
        return true;
    }
    if (veh->getLaneChangeModel().hasBlueLight()) {
        // blue light device automatically gets right of way
        return true;
    }
    const MSLane* foeLane = veh->getLane();
    if (foeLane->isInternal()) {
        if (foeLane->getEdge().getFromJunction() == link->getJunction()) {
            SUMOTime egoET = myJunctionConflictEntryTime;
            SUMOTime foeET = veh->myJunctionEntryTime;
            // check relationship between link and foeLane
            if (foeLane->getNormalPredecessorLane() == link->getInternalLaneBefore()->getNormalPredecessorLane()) {
                // we are entering the junction from the same lane
                egoET = myJunctionEntryTimeNeverYield;
                foeET = veh->myJunctionEntryTimeNeverYield;
                if (link->isExitLinkAfterInternalJunction() && link->getInternalLaneBefore()->getLogicalPredecessorLane()->getEntryLink()->isIndirect()) {
                    egoET = myJunctionConflictEntryTime;
                }
            } else {
                const MSLink* foeLink = foeLane->getIncomingLanes()[0].viaLink;
                const MSJunctionLogic* logic = link->getJunction()->getLogic();
                assert(logic != nullptr);
                // determine who has right of way
                bool response; // ego response to foe
                bool response2; // foe response to ego
                // attempt 1: tlLinkState
                const MSLink* entry = link->getCorrespondingEntryLink();
                const MSLink* foeEntry = foeLink->getCorrespondingEntryLink();
                if (entry->haveRed() || foeEntry->haveRed()) {
                    // ensure that vehicles which are stuck on the intersection may exit
                    if (!foeEntry->haveRed() && veh->getSpeed() > SUMO_const_haltingSpeed && gap < 0) {
                        // foe might be oncoming, don't drive unless foe can still brake safely
                        const double foeNextSpeed = veh->getSpeed() + ACCEL2SPEED(veh->getCarFollowModel().getMaxAccel());
                        const double foeBrakeGap = veh->getCarFollowModel().brakeGap(
                                                       foeNextSpeed, veh->getCarFollowModel().getMaxDecel(), veh->getCarFollowModel().getHeadwayTime());
                        // the minGap was subtracted from gap in MSLink::getLeaderInfo (enlarging the negative gap)
                        // so the -2* makes it point in the right direction
                        const double foeGap = -gap - veh->getLength() - 2 * getVehicleType().getMinGap();
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
                        if (DEBUG_COND) {
                            std::cout << " foeGap=" << foeGap << " foeBGap=" << foeBrakeGap << "\n";

                        }
#endif
                        if (foeGap < foeBrakeGap) {
                            response = true;
                            response2 = false;
                        } else {
                            response = false;
                            response2 = true;
                        }
                    } else {
                        // brake for stuck foe
                        response = foeEntry->haveRed();
                        response2 = entry->haveRed();
                    }
                } else if (entry->havePriority() != foeEntry->havePriority()) {
                    response = !entry->havePriority();
                    response2 = !foeEntry->havePriority();
                } else if (entry->haveYellow() && foeEntry->haveYellow()) {
                    // let the faster vehicle keep moving
                    response = veh->getSpeed() >= getSpeed();
                    response2 = getSpeed() >= veh->getSpeed();
                } else {
                    // fallback if pedestrian crossings are involved
                    response = logic->getResponseFor(link->getIndex()).test(foeLink->getIndex());
                    response2 = logic->getResponseFor(foeLink->getIndex()).test(link->getIndex());
                }
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
                if (DEBUG_COND) {
                    std::cout << SIMTIME
                              << " foeLane=" << foeLane->getID()
                              << " foeLink=" << foeLink->getViaLaneOrLane()->getID()
                              << " linkIndex=" << link->getIndex()
                              << " foeLinkIndex=" << foeLink->getIndex()
                              << " entryState=" << toString(entry->getState())
                              << " entryState2=" << toString(foeEntry->getState())
                              << " response=" << response
                              << " response2=" << response2
                              << "\n";
                }
#endif
                if (!response) {
                    // if we have right of way over the foe, entryTime does not matter
                    foeET = veh->myJunctionConflictEntryTime;
                    egoET = myJunctionEntryTime;
                } else if (response && response2) {
                    // in a mutual conflict scenario, use entry time to avoid deadlock
                    foeET = veh->myJunctionConflictEntryTime;
                    egoET = myJunctionConflictEntryTime;
                }
            }
            if (egoET == foeET) {
                // try to use speed as tie braker
                if (getSpeed() == veh->getSpeed()) {
                    // use ID as tie braker
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
                    if (DEBUG_COND) {
                        std::cout << SIMTIME << " veh=" << getID() << " equal ET " << egoET << " with foe " << veh->getID()
                                  << " foeIsLeaderByID=" << (getID() < veh->getID()) << "\n";
                    }
#endif
                    return getID() < veh->getID();
                } else {
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
                    if (DEBUG_COND) {
                        std::cout << SIMTIME << " veh=" << getID() << " equal ET " << egoET << " with foe " << veh->getID()
                                  << " foeIsLeaderBySpeed=" << (getSpeed() < veh->getSpeed())
                                  << " v=" << getSpeed() << " foeV=" << veh->getSpeed()
                                  << "\n";
                    }
#endif
                    return getSpeed() < veh->getSpeed();
                }
            } else {
                // leader was on the junction first
#ifdef DEBUG_PLAN_MOVE_LEADERINFO
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " veh=" << getID() << " egoET " << egoET << " with foe " << veh->getID()
                              << " foeET=" << foeET  << " isLeader=" << (egoET > foeET) << "\n";
                }
#endif
                return egoET > foeET;
            }
        } else {
            // vehicle can only be partially on the junction. Must be a leader
            return true;
        }
    } else {
        // vehicle can only be partially on the junction. Must be a leader
        return true;
    }
}

void
MSVehicle::saveState(OutputDevice& out) {
    MSBaseVehicle::saveState(out);
    // here starts the vehicle internal part (see loading)
    std::vector<std::string> internals;
    internals.push_back(toString(myParameter->parametersSet));
    internals.push_back(toString(myDeparture));
    internals.push_back(toString(distance(myRoute->begin(), myCurrEdge)));
    internals.push_back(toString(myDepartPos));
    internals.push_back(toString(myWaitingTime));
    internals.push_back(toString(myTimeLoss));
    internals.push_back(toString(myLastActionTime));
    internals.push_back(toString(isStopped()));
    internals.push_back(toString(myPastStops.size()));
    out.writeAttr(SUMO_ATTR_STATE, internals);
    out.writeAttr(SUMO_ATTR_POSITION, std::vector<double> { myState.myPos, myState.myBackPos, myState.myLastCoveredDist });
    out.writeAttr(SUMO_ATTR_SPEED, std::vector<double> { myState.mySpeed, myState.myPreviousSpeed });
    out.writeAttr(SUMO_ATTR_ANGLE, GeomHelper::naviDegree(myAngle));
    out.writeAttr(SUMO_ATTR_POSITION_LAT, myState.myPosLat);
    out.writeAttr(SUMO_ATTR_WAITINGTIME, myWaitingTimeCollector.getState());
    myLaneChangeModel->saveState(out);
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
        stop.writeParams(out);
        out.closeTag();
    }
    // save upcoming stops
    for (MSStop& stop : myStops) {
        stop.write(out);
    }
    // save parameters and device states
    myParameter->writeParams(out);
    for (MSVehicleDevice* const dev : myDevices) {
        dev->saveState(out);
    }
    out.closeTag();
}

void
MSVehicle::loadState(const SUMOSAXAttributes& attrs, const SUMOTime offset) {
    if (!attrs.hasAttribute(SUMO_ATTR_POSITION)) {
        throw ProcessError(TL("Error: Invalid vehicles in state (may be a meso state)!"));
    }
    int routeOffset;
    bool stopped;
    int pastStops;

    std::istringstream bis(attrs.getString(SUMO_ATTR_STATE));
    bis >> myParameter->parametersSet;
    bis >> myDeparture;
    bis >> routeOffset;
    bis >> myDepartPos;
    bis >> myWaitingTime;
    bis >> myTimeLoss;
    bis >> myLastActionTime;
    bis >> stopped;
    bis >> pastStops;

    // load stops
    myStops.clear();
    addStops(!MSGlobals::gCheckRoutes, &myCurrEdge, false);

    if (hasDeparted()) {
        myCurrEdge = myRoute->begin() + routeOffset;
        myDeparture -= offset;
        // fix stops
        while (pastStops > 0) {
            myPastStops.push_back(myStops.front().pars);
            myPastStops.back().routeIndex = (int)(myStops.front().edge - myRoute->begin());
            myStops.pop_front();
            pastStops--;
        }
        // see MSBaseVehicle constructor
        if (myParameter->wasSet(VEHPARS_FORCE_REROUTE)) {
            calculateArrivalParams(true);
        }
    }
    if (getActionStepLength() == DELTA_T && !isActionStep(SIMSTEP)) {
        myLastActionTime -= (myLastActionTime - SIMSTEP) % DELTA_T;
        WRITE_WARNINGF(TL("Action steps are out of sync for loaded vehicle '%'."), getID());
    }
    std::istringstream pis(attrs.getString(SUMO_ATTR_POSITION));
    pis >> myState.myPos >> myState.myBackPos >> myState.myLastCoveredDist;
    std::istringstream sis(attrs.getString(SUMO_ATTR_SPEED));
    sis >> myState.mySpeed >> myState.myPreviousSpeed;
    myAcceleration = SPEED2ACCEL(myState.mySpeed - myState.myPreviousSpeed);
    myAngle = GeomHelper::fromNaviDegree(attrs.getFloat(SUMO_ATTR_ANGLE));
    myState.myPosLat = attrs.getFloat(SUMO_ATTR_POSITION_LAT);
    std::istringstream dis(attrs.getString(SUMO_ATTR_DISTANCE));
    dis >> myOdometer >> myNumberReroutes;
    myWaitingTimeCollector.setState(attrs.getString(SUMO_ATTR_WAITINGTIME));
    if (stopped) {
        myStops.front().startedFromState = true;
        myStopDist = 0;
    }
    myLaneChangeModel->loadState(attrs);
    // no need to reset myCachedPosition here since state loading happens directly after creation
}

void
MSVehicle::loadPreviousApproaching(MSLink* link, bool setRequest,
                                   SUMOTime arrivalTime, double arrivalSpeed,
                                   double arrivalSpeedBraking,
                                   double dist, double leaveSpeed) {
    // ensure that approach information is reset on the next call to setApproachingForAllLinks
    myLFLinkLanes.push_back(DriveProcessItem(link, 0, 0, setRequest,
                            arrivalTime, arrivalSpeed, arrivalSpeedBraking, dist, leaveSpeed));

}


std::shared_ptr<MSSimpleDriverState>
MSVehicle::getDriverState() const {
    return myDriverState->getDriverState();
}


double
MSVehicle::getFriction() const {
    return myFrictionDevice == nullptr ? 1. : myFrictionDevice->getMeasuredFriction();
}


void
MSVehicle::setPreviousSpeed(double prevSpeed, double prevAcceleration) {
    myState.mySpeed = MAX2(0., prevSpeed);
    // also retcon acceleration
    if (prevAcceleration != std::numeric_limits<double>::min()) {
        myAcceleration = prevAcceleration;
    } else {
        myAcceleration = SPEED2ACCEL(myState.mySpeed - myState.myPreviousSpeed);
    }
}


double
MSVehicle::getCurrentApparentDecel() const {
    //return MAX2(-myAcceleration, getCarFollowModel().getApparentDecel());
    return getCarFollowModel().getApparentDecel();
}

/****************************************************************************/
bool
MSVehicle::setExitManoeuvre() {
    return (myManoeuvre.configureExitManoeuvre(this));
}

/* -------------------------------------------------------------------------
 * methods of MSVehicle::manoeuvre
 * ----------------------------------------------------------------------- */

MSVehicle::Manoeuvre::Manoeuvre() : myManoeuvreStop(""), myManoeuvreStartTime(0), myManoeuvreCompleteTime(0), myManoeuvreType(MSVehicle::MANOEUVRE_NONE), myGUIIncrement(0) {}


MSVehicle::Manoeuvre::Manoeuvre(const Manoeuvre& manoeuvre) {
    myManoeuvreStop = manoeuvre.myManoeuvreStop;
    myManoeuvreStartTime = manoeuvre.myManoeuvreStartTime;
    myManoeuvreCompleteTime = manoeuvre.myManoeuvreCompleteTime;
    myManoeuvreType = manoeuvre.myManoeuvreType;
    myGUIIncrement = manoeuvre.myGUIIncrement;
}


MSVehicle::Manoeuvre&
MSVehicle::Manoeuvre::operator=(const Manoeuvre& manoeuvre) {
    myManoeuvreStop = manoeuvre.myManoeuvreStop;
    myManoeuvreStartTime = manoeuvre.myManoeuvreStartTime;
    myManoeuvreCompleteTime = manoeuvre.myManoeuvreCompleteTime;
    myManoeuvreType = manoeuvre.myManoeuvreType;
    myGUIIncrement = manoeuvre.myGUIIncrement;
    return *this;
}


bool
MSVehicle::Manoeuvre::operator!=(const Manoeuvre& manoeuvre) {
    return (myManoeuvreStop != manoeuvre.myManoeuvreStop ||
            myManoeuvreStartTime != manoeuvre.myManoeuvreStartTime ||
            myManoeuvreCompleteTime != manoeuvre.myManoeuvreCompleteTime ||
            myManoeuvreType != manoeuvre.myManoeuvreType ||
            myGUIIncrement != manoeuvre.myGUIIncrement
           );
}


double
MSVehicle::Manoeuvre::getGUIIncrement() const {
    return (myGUIIncrement);
}


MSVehicle::ManoeuvreType
MSVehicle::Manoeuvre::getManoeuvreType() const {
    return (myManoeuvreType);
}


MSVehicle::ManoeuvreType
MSVehicle::getManoeuvreType() const {
    return (myManoeuvre.getManoeuvreType());
}


void
MSVehicle::setManoeuvreType(const MSVehicle::ManoeuvreType mType) {
    myManoeuvre.setManoeuvreType(mType);
}


void
MSVehicle::Manoeuvre::setManoeuvreType(const MSVehicle::ManoeuvreType mType) {
    myManoeuvreType = mType;
}


bool
MSVehicle::Manoeuvre::configureEntryManoeuvre(MSVehicle* veh) {
    if (!veh->hasStops()) {
        return false;    // should never happen - checked before call
    }

    const SUMOTime currentTime = MSNet::getInstance()->getCurrentTimeStep();
    const MSStop& stop = veh->getNextStop();

    int manoeuverAngle = stop.parkingarea->getLastFreeLotAngle();
    double GUIAngle = stop.parkingarea->getLastFreeLotGUIAngle();
    if (abs(GUIAngle) < 0.1) {
        GUIAngle = -0.1;    // Wiggle vehicle on parallel entry
    }
    myManoeuvreVehicleID = veh->getID();
    myManoeuvreStop = stop.parkingarea->getID();
    myManoeuvreType = MSVehicle::MANOEUVRE_ENTRY;
    myManoeuvreStartTime = currentTime;
    myManoeuvreCompleteTime = currentTime + veh->myType->getEntryManoeuvreTime(manoeuverAngle);
    myGUIIncrement = GUIAngle / (STEPS2TIME(myManoeuvreCompleteTime - myManoeuvreStartTime) / TS);

#ifdef DEBUG_STOPS
    if (veh->isSelected()) {
        std::cout << "ENTRY manoeuvre start: vehicle=" << veh->getID() << " Manoeuvre Angle=" << manoeuverAngle << " Rotation angle=" << RAD2DEG(GUIAngle) << " Road Angle" << RAD2DEG(veh->getAngle()) << " increment=" << RAD2DEG(myGUIIncrement) << " currentTime=" << currentTime <<
                  " endTime=" << myManoeuvreCompleteTime << " manoeuvre time=" << myManoeuvreCompleteTime - currentTime << " parkArea=" << myManoeuvreStop << std::endl;
    }
#endif

    return (true);
}


bool
MSVehicle::Manoeuvre::configureExitManoeuvre(MSVehicle* veh) {
    // At the moment we only want to set for parking areas
    if (!veh->hasStops()) {
        return true;
    }
    if (veh->getNextStop().parkingarea == nullptr) {
        return true;
    }

    if (myManoeuvreType != MSVehicle::MANOEUVRE_NONE) {
        return (false);
    }

    const SUMOTime currentTime = MSNet::getInstance()->getCurrentTimeStep();

    int manoeuverAngle = veh->getCurrentParkingArea()->getManoeuverAngle(*veh);
    double GUIAngle = veh->getCurrentParkingArea()->getGUIAngle(*veh);
    if (abs(GUIAngle) < 0.1) {
        GUIAngle = 0.1;    // Wiggle vehicle on parallel exit
    }

    myManoeuvreVehicleID = veh->getID();
    myManoeuvreStop = veh->getCurrentParkingArea()->getID();
    myManoeuvreType = MSVehicle::MANOEUVRE_EXIT;
    myManoeuvreStartTime = currentTime;
    myManoeuvreCompleteTime = currentTime + veh->myType->getExitManoeuvreTime(manoeuverAngle);
    myGUIIncrement = -GUIAngle / (STEPS2TIME(myManoeuvreCompleteTime - myManoeuvreStartTime) / TS);
    if (veh->remainingStopDuration() > 0) {
        myManoeuvreCompleteTime += veh->remainingStopDuration();
    }

#ifdef DEBUG_STOPS
    if (veh->isSelected()) {
        std::cout << "EXIT manoeuvre start: vehicle=" << veh->getID() << " Manoeuvre Angle=" << manoeuverAngle  << " increment=" << RAD2DEG(myGUIIncrement) << " currentTime=" << currentTime
                  << " endTime=" << myManoeuvreCompleteTime << " manoeuvre time=" << myManoeuvreCompleteTime - currentTime << " parkArea=" << myManoeuvreStop << std::endl;
    }
#endif

    return (true);
}


bool
MSVehicle::Manoeuvre::entryManoeuvreIsComplete(MSVehicle* veh) {
    // At the moment we only want to consider parking areas - need to check because we could be setting up a manoeuvre
    if (!veh->hasStops()) {
        return (true);
    }
    MSStop* currentStop = &veh->myStops.front();
    if (currentStop->parkingarea == nullptr) {
        return true;
    } else if (currentStop->parkingarea->getID() != myManoeuvreStop || MSVehicle::MANOEUVRE_ENTRY != myManoeuvreType) {
        if (configureEntryManoeuvre(veh)) {
            MSNet::getInstance()->informVehicleStateListener(veh, MSNet::VehicleState::MANEUVERING);
            return (false);
        } else { // cannot configure entry so stop trying
            return true;
        }
    } else if (MSNet::getInstance()->getCurrentTimeStep() < myManoeuvreCompleteTime) {
        return false;
    } else { // manoeuvre complete
        myManoeuvreType = MSVehicle::MANOEUVRE_NONE;
        return true;
    }
}


bool
MSVehicle::Manoeuvre::manoeuvreIsComplete(const ManoeuvreType checkType) const {
    if (checkType != myManoeuvreType) {
        return true;    // we're not maneuvering / wrong manoeuvre
    }

    if (MSNet::getInstance()->getCurrentTimeStep() < myManoeuvreCompleteTime) {
        return false;
    } else {
        return true;
    }
}


bool
MSVehicle::Manoeuvre::manoeuvreIsComplete() const {
    return (MSNet::getInstance()->getCurrentTimeStep() >= myManoeuvreCompleteTime);
}


bool
MSVehicle::manoeuvreIsComplete() const {
    return (myManoeuvre.manoeuvreIsComplete());
}


std::pair<double, double>
MSVehicle::estimateTimeToNextStop() const {
    if (hasStops()) {
        MSLane* lane = myLane;
        if (lane == nullptr) {
            // not in network
            lane = getEdge()->getLanes()[0];
        }
        const MSStop& stop = myStops.front();
        auto it = myCurrEdge + 1;
        // drive to end of current edge
        double dist = (lane->getLength() - getPositionOnLane());
        double travelTime = lane->getEdge().getMinimumTravelTime(this) * dist / lane->getLength();
        // drive until stop edge
        while (it != myRoute->end() && it < stop.edge) {
            travelTime += (*it)->getMinimumTravelTime(this);
            dist += (*it)->getLength();
            it++;
        }
        // drive up to the stop position
        const double stopEdgeDist = stop.pars.endPos - (lane == stop.lane ? lane->getLength() : 0);
        dist += stopEdgeDist;
        travelTime += stop.lane->getEdge().getMinimumTravelTime(this) * (stopEdgeDist / stop.lane->getLength());
        // estimate time loss due to acceleration and deceleration
        // maximum speed is limited by available distance:
        const double a = getCarFollowModel().getMaxAccel();
        const double b = getCarFollowModel().getMaxDecel();
        const double c = getSpeed();
        const double d = dist;
        const double len = getVehicleType().getLength();
        const double vs = MIN2(MAX2(stop.getSpeed(), 0.0), stop.lane->getVehicleMaxSpeed(this));
        // distAccel = (v - c)^2 / (2a)
        // distDecel = (v + vs)*(v - vs) / 2b = (v^2 - vs^2) / (2b)
        // distAccel + distDecel < d
        const double maxVD = MAX2(c, ((sqrt(MAX2(0.0, pow(2 * c * b, 2) + (4 * ((b * ((a * (2 * d * (b + a) + (vs * vs) - (c * c))) - (b * (c * c))))
                                            + pow((a * vs), 2))))) * 0.5) + (c * b)) / (b + a));
        it = myCurrEdge;
        double v0 = c;
        bool v0Stable = getAcceleration() == 0 && v0 > 0;
        double timeLossAccel = 0;
        double timeLossDecel = 0;
        double timeLossLength = 0;
        while (it != myRoute->end() && it <= stop.edge) {
            double v = MIN2(maxVD, (*it)->getVehicleMaxSpeed(this));
            double edgeLength = (it == stop.edge ? stop.pars.endPos : (*it)->getLength()) - (it == myCurrEdge ? getPositionOnLane() : 0);
            if (edgeLength <= len && v0Stable && v0 < v) {
                const double lengthDist = MIN2(len, edgeLength);
                const double dTL = lengthDist / v0 - lengthDist / v;
                //std::cout << "   e=" << (*it)->getID() << " v0=" << v0 << " v=" << v << " el=" << edgeLength << " lDist=" << lengthDist << " newTLL=" << dTL<< "\n";
                timeLossLength += dTL;
            }
            if (edgeLength > len) {
                const double dv = v - v0;
                if (dv > 0) {
                    // timeLossAccel = timeAccel - timeMaxspeed = dv / a - distAccel / v
                    const double dTA = dv / a - dv * (v + v0) / (2 * a * v);
                    //std::cout << "   e=" << (*it)->getID() << " v0=" << v0 << " v=" << v << " newTLA=" << dTA << "\n";
                    timeLossAccel += dTA;
                    // time loss from vehicle length
                } else if (dv < 0) {
                    // timeLossDecel = timeDecel - timeMaxspeed = dv / b - distDecel / v
                    const double dTD = -dv / b + dv * (v + v0) / (2 * b * v0);
                    //std::cout << "   e=" << (*it)->getID() << " v0=" << v0 << " v=" << v << " newTLD=" << dTD << "\n";
                    timeLossDecel += dTD;
                }
                v0 = v;
                v0Stable = true;
            }
            it++;
        }
        // final deceleration to stop (may also be acceleration or deceleration to waypoint speed)
        double v = vs;
        const double dv = v - v0;
        if (dv > 0) {
            // timeLossAccel = timeAccel - timeMaxspeed = dv / a - distAccel / v
            const double dTA = dv / a - dv * (v + v0) / (2 * a * v);
            //std::cout << "  final e=" << (*it)->getID() << " v0=" << v0 << " v=" << v << " newTLA=" << dTA << "\n";
            timeLossAccel += dTA;
            // time loss from vehicle length
        } else if (dv < 0) {
            // timeLossDecel = timeDecel - timeMaxspeed = dv / b - distDecel / v
            const double dTD = -dv / b + dv * (v + v0) / (2 * b * v0);
            //std::cout << "  final  e=" << (*it)->getID() << " v0=" << v0 << " v=" << v << " newTLD=" << dTD << "\n";
            timeLossDecel += dTD;
        }
        const double result = travelTime + timeLossAccel + timeLossDecel + timeLossLength;
        //std::cout << SIMTIME << " v=" << c << " a=" << a << " b=" << b << " maxVD=" << maxVD << " tt=" << travelTime
        //    << " ta=" << timeLossAccel << " td=" << timeLossDecel << " tl=" << timeLossLength << " res=" << result << "\n";
        return {MAX2(0.0, result), dist};
    } else {
        return {INVALID_DOUBLE, INVALID_DOUBLE};
    }
}


double
MSVehicle::getStopDelay() const {
    if (hasStops() && myStops.front().pars.until >= 0) {
        const MSStop& stop = myStops.front();
        SUMOTime estimatedDepart = MSNet::getInstance()->getCurrentTimeStep() - DELTA_T;
        if (stop.reached) {
            return STEPS2TIME(estimatedDepart + stop.duration - stop.pars.until);
        }
        if (stop.pars.duration > 0) {
            estimatedDepart += stop.pars.duration;
        }
        estimatedDepart += TIME2STEPS(estimateTimeToNextStop().first);
        const double result = MAX2(0.0, STEPS2TIME(estimatedDepart - stop.pars.until));
        return result;
    } else {
        // vehicles cannot drive before 'until' so stop delay can never be
        // negative and we can use -1 to signal "undefined"
        return -1;
    }
}


double
MSVehicle::getStopArrivalDelay() const {
    if (hasStops() && myStops.front().pars.arrival >= 0) {
        const MSStop& stop = myStops.front();
        if (stop.reached) {
            return STEPS2TIME(stop.pars.started - stop.pars.arrival);
        } else {
            return STEPS2TIME(MSNet::getInstance()->getCurrentTimeStep()) + estimateTimeToNextStop().first - STEPS2TIME(stop.pars.arrival);
        }
    } else {
        // vehicles can arrival earlier than planned so arrival delay can be negative
        return INVALID_DOUBLE;
    }
}


const MSEdge*
MSVehicle::getCurrentEdge() const {
    return myLane != nullptr ? &myLane->getEdge() : getEdge();
}


const MSEdge*
MSVehicle::getNextEdgePtr() const {
    if (myLane == nullptr || (myCurrEdge + 1) == myRoute->end()) {
        return nullptr;
    }
    if (myLane->isInternal()) {
        return &myLane->getCanonicalSuccessorLane()->getEdge();
    } else {
        const MSEdge* nextNormal = succEdge(1);
        const MSEdge* nextInternal = myLane->getEdge().getInternalFollowingEdge(nextNormal, getVClass());
        return nextInternal ? nextInternal : nextNormal;
    }
}


const MSLane*
MSVehicle::getPreviousLane(const MSLane* current, int& furtherIndex) const {
    if (furtherIndex < (int)myFurtherLanes.size()) {
        return myFurtherLanes[furtherIndex++];
    } else {
        // try to use route information
        int routeIndex = getRoutePosition();
        bool resultInternal;
        if (MSGlobals::gUsingInternalLanes && MSNet::getInstance()->hasInternalLinks()) {
            if (myLane->isInternal()) {
                if (furtherIndex % 2 == 0) {
                    routeIndex -= (furtherIndex + 0) / 2;
                    resultInternal = false;
                } else {
                    routeIndex -= (furtherIndex + 1) / 2;
                    resultInternal = false;
                }
            } else {
                if (furtherIndex % 2 != 0) {
                    routeIndex -= (furtherIndex + 1) / 2;
                    resultInternal = false;
                } else {
                    routeIndex -= (furtherIndex + 2) / 2;
                    resultInternal = true;
                }
            }
        } else {
            routeIndex -= furtherIndex;
            resultInternal = false;
        }
        furtherIndex++;
        if (routeIndex >= 0) {
            if (resultInternal) {
                const MSEdge* prevNormal = myRoute->getEdges()[routeIndex];
                for (MSLane* cand : prevNormal->getLanes()) {
                    for (MSLink* link : cand->getLinkCont()) {
                        if (link->getLane() == current) {
                            if (link->getViaLane() != nullptr) {
                                return link->getViaLane();
                            } else {
                                return const_cast<MSLane*>(link->getLaneBefore());
                            }
                        }
                    }
                }
            } else {
                return myRoute->getEdges()[routeIndex]->getLanes()[0];
            }
        }
    }
    return current;
}


/****************************************************************************/
