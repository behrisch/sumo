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
/// @file    MSRailSignal.cpp
/// @author  Melanie Weber
/// @author  Andreas Kendziorra
/// @author  Jakob Erdmann
/// @date    Jan 2015
///
// A rail signal logic
/****************************************************************************/
#include <config.h>

#include <cassert>
#include <utility>
#include <vector>
#include <bitset>
#ifdef HAVE_FOX
#include <utils/foxtools/MFXWorkerThread.h>
#endif
#include <utils/iodevices/OutputDevice_COUT.h>
#include <microsim/MSEventControl.h>
#include <microsim/MSNet.h>
#include <microsim/MSEdge.h>
#include <microsim/MSEdgeControl.h>
#include <microsim/MSLane.h>
#include <microsim/MSLink.h>
#include <microsim/MSVehicle.h>
#include <microsim/devices/MSDevice_Routing.h>
#include <microsim/devices/MSRoutingEngine.h>
#include <microsim/MSLane.h>

#include "MSTLLogicControl.h"
#include "MSTrafficLightLogic.h"
#include "MSPhaseDefinition.h"
#include "MSTLLogicControl.h"
#include "MSRailSignalConstraint.h"
#include "MSRailSignalControl.h"
#include "MSDriveWay.h"
#include "MSRailSignal.h"

//#define DEBUG_SELECT_DRIVEWAY
//#define DEBUG_DRIVEWAY_UPDATE
//#define DEBUG_SIGNALSTATE
//#define DEBUG_REROUTE

#define DEBUG_COND DEBUG_HELPER(this)
#define DEBUG_COND_LINKINFO DEBUG_HELPER(myLink->getTLLogic())
#define DEBUG_HELPER(obj) ((obj)->isSelected())
//#define DEBUG_HELPER(obj) ((obj)->getID() == "")
//#define DEBUG_HELPER(obj) (true)

// ===========================================================================
// static value definitions
// ===========================================================================

bool MSRailSignal::myStoreVehicles(false);
MSRailSignal::VehicleVector MSRailSignal::myBlockingVehicles;
MSRailSignal::VehicleVector MSRailSignal::myRivalVehicles;
MSRailSignal::VehicleVector MSRailSignal::myPriorityVehicles;
std::string MSRailSignal::myConstraintInfo;
int MSRailSignal::myRSIndex(0);
std::vector<const MSDriveWay*> MSRailSignal::myBlockingDriveWays;
std::string MSRailSignal::myRequestedDriveWay;

// ===========================================================================
// method definitions
// ===========================================================================
MSRailSignal::MSRailSignal(MSTLLogicControl& tlcontrol,
                           const std::string& id, const std::string& programID, SUMOTime delay,
                           const Parameterised::Map& parameters) :
    MSTrafficLightLogic(tlcontrol, id, programID, 0, TrafficLightType::RAIL_SIGNAL, delay, parameters),
    myNumericalID(myRSIndex++),
    myCurrentPhase(DELTA_T, std::string(SUMO_MAX_CONNECTIONS, 'X')), // dummy phase
    myPhaseIndex(0),
    myDriveWayIndex(0) {
    myDefaultCycleTime = DELTA_T;
    myMovingBlock = OptionsCont::getOptions().getBool("railsignal-moving-block");
    mySwitchCommand->deschedule(this);
}

void
MSRailSignal::init(NLDetectorBuilder&) {
    if (myLanes.size() == 0) {
        WRITE_WARNINGF(TL("Rail signal at junction '%' does not control any links"), getID());
    }
    SVCPermissions outgoingPermissions = 0;
    for (LinkVector& links : myLinks) { //for every link index
        if (links.size() != 1) {
            throw ProcessError("At railSignal '" + getID() + "' found " + toString(links.size())
                               + " links controlled by index " + toString(links[0]->getTLIndex()));
        }
        myLinkInfos.push_back(LinkInfo(links[0]));
        outgoingPermissions |= links[0]->getPermissions();
    }
    updateCurrentPhase();
    setTrafficLightSignals(MSNet::getInstance()->getCurrentTimeStep());
    myNumLinks = (int)myLinks.size();
    MSRailSignalControl::getInstance().addSignal(this);
    myMovingBlock |= MSRailSignalControl::isMovingBlock(outgoingPermissions);
}


MSRailSignal::~MSRailSignal() {
    removeConstraints();
}


// ----------- Handling of controlled links
void
MSRailSignal::adaptLinkInformationFrom(const MSTrafficLightLogic& logic) {
    MSTrafficLightLogic::adaptLinkInformationFrom(logic);
    updateCurrentPhase();
}


// ------------ Switching and setting current rows
SUMOTime
MSRailSignal::trySwitch() {
    // deschedule regular traffic light event,
    // updateCurrentPhase is instead called from MSRailSignalControl::updateSignals
    return SUMOTime_MAX;
}



bool
MSRailSignal::updateCurrentPhase() {
#ifdef DEBUG_SIGNALSTATE
    gDebugFlag4 = DEBUG_COND;
#endif
    bool keepActive = false;
    // green by default so vehicles can be inserted at the borders of the network
    std::string state(myLinks.size(), 'G');
    for (LinkInfo& li : myLinkInfos) {
        if (li.myLink->getApproaching().size() > 0 && li.myControlled) {
            keepActive = true;
            Approaching closest = li.myLink->getClosest();
            MSDriveWay& driveway = li.getDriveWay(closest.first);
            //std::cout << SIMTIME << " signal=" << getTLLinkID(li.myLink) << " veh=" << closest.first->getID() << " dw:\n";
            //driveway.writeBlocks(*OutputDevice_COUT::getDevice());
            const bool mustWait = !constraintsAllow(closest.first, true);
            MSEdgeVector occupied;
            if (mustWait || !driveway.reserve(closest, occupied)) {
                state[li.myLink->getTLIndex()] = 'r';
                if (occupied.size() > 0) {
                    li.reroute(const_cast<SUMOVehicle*>(closest.first), occupied);
                }
#ifdef DEBUG_SIGNALSTATE
                if (gDebugFlag4) {
                    std::cout << SIMTIME << " rsl=" << li.getID() << " veh=" << closest.first->getID() << " notReserved\n";
                }
#endif
            } else {
                state[li.myLink->getTLIndex()] = 'G';
#ifdef DEBUG_SIGNALSTATE
                if (gDebugFlag4) {
                    std::cout << SIMTIME << " rsl=" << li.getID() << " veh=" << closest.first->getID() << " reserved\n";
                }
#endif
            }
        } else if (li.myControlled) {
            if (li.myDriveways.empty()) {
#ifdef DEBUG_SIGNALSTATE
                if (gDebugFlag4) {
                    std::cout << SIMTIME << " rsl=" << li.getID() << " red for unitialized signal (no driveways yet)\n";
                }
#endif
                state[li.myLink->getTLIndex()] = 'r';
            } else {
                const MSDriveWay& driveway = *li.myDriveways.front();
                MSEdgeVector occupied;
                if (driveway.foeDriveWayOccupied(true, nullptr, occupied) || driveway.foeDriveWayApproached()) {
                    keepActive = true;
#ifdef DEBUG_SIGNALSTATE
                    if (gDebugFlag4) {
                        std::cout << SIMTIME << " rsl=" << li.getID() << " red for default driveway " << driveway.getID() << "\n";
                    }
#endif
                    state[li.myLink->getTLIndex()] = 'r';
                } else {
#ifdef DEBUG_SIGNALSTATE
                    if (gDebugFlag4) {
                        std::cout << SIMTIME << " rsl=" << li.getID() << " green for default driveway " << driveway.getID() << "\n";
                    }
#endif
                }
            }
        } else {
            state[li.myLink->getTLIndex()] = 'O';
        }
    }
    if (myCurrentPhase.getState() != state) {
        myCurrentPhase.setState(state);
        myPhaseIndex = 1 - myPhaseIndex;
        // set link priorities
        setTrafficLightSignals(SIMSTEP);
        // execute switch actions (3D-gui)
        //const MSTLLogicControl::TLSLogicVariants& vars = myTLControl.get(myTLLogic->getID());
        //vars.executeOnSwitchActions();
    }
#ifdef DEBUG_SIGNALSTATE
    gDebugFlag4 = false;
#endif
    return keepActive;
}


bool
MSRailSignal::constraintsAllow(const SUMOVehicle* veh, bool storeWaitRelation) const {
    if (myConstraints.size() == 0) {
        return true;
    } else {
        const std::string tripID = veh->getParameter().getParameter("tripId", veh->getID());
        auto it = myConstraints.find(tripID);
        if (it != myConstraints.end()) {
            for (MSRailSignalConstraint* c : it->second) {
                // ignore insertion constraints here
                if (!c->isInsertionConstraint() && !c->cleared()) {
#ifdef DEBUG_SIGNALSTATE
                    if (gDebugFlag4) {
                        std::cout << "  constraint '" << c->getDescription() << "' not cleared\n";
                    }
#endif
                    if (storeWaitRelation && MSGlobals::gTimeToTeleportRSDeadlock > 0
                            && veh->getWaitingTime() > veh->getVehicleType().getCarFollowModel().getStartupDelay()) {
                        const SUMOVehicle* foe = c->getFoe();
                        if (foe != nullptr) {
                            MSRailSignalControl::getInstance().addWaitRelation(veh, this, foe, c);
                        }
                    }
                    if (myStoreVehicles) {
                        myConstraintInfo = c->getDescription();
                    }
                    return false;
                }
            }
        }
        return true;
    }
}


void
MSRailSignal::addConstraint(const std::string& tripId, MSRailSignalConstraint* constraint) {
    myConstraints[tripId].push_back(constraint);
}


bool
MSRailSignal::removeConstraint(const std::string& tripId, MSRailSignalConstraint* constraint) {
    if (myConstraints.count(tripId) != 0) {
        auto& constraints = myConstraints[tripId];
        auto it = std::find(constraints.begin(), constraints.end(), constraint);
        if (it != constraints.end()) {
            delete *it;
            constraints.erase(it);
            return true;
        }
    }
    return false;
}

void
MSRailSignal::removeConstraints() {
    for (auto item : myConstraints) {
        for (MSRailSignalConstraint* c : item.second) {
            delete c;
        }
    }
    myConstraints.clear();
}


// ------------ Static Information Retrieval
int
MSRailSignal::getPhaseNumber() const {
    return 0;
}

const MSTrafficLightLogic::Phases&
MSRailSignal::getPhases() const {
    return myPhases;
}

const MSPhaseDefinition&
MSRailSignal::getPhase(int) const {
    return myCurrentPhase;
}

// ------------ Dynamic Information Retrieval
int
MSRailSignal::getCurrentPhaseIndex() const {
    return myPhaseIndex;
}

const MSPhaseDefinition&
MSRailSignal::getCurrentPhaseDef() const {
    return myCurrentPhase;
}


void
MSRailSignal::resetLastSwitch(SUMOTime t) {
    myCurrentPhase.myLastSwitch = t;
}


// ------------ Conversion between time and phase
SUMOTime
MSRailSignal::getPhaseIndexAtTime(SUMOTime) const {
    return 0;
}

SUMOTime
MSRailSignal::getOffsetFromIndex(int) const {
    return 0;
}

int
MSRailSignal::getIndexFromOffset(SUMOTime) const {
    return 0;
}


void
MSRailSignal::addLink(MSLink* link, MSLane* lane, int pos) {
    if (pos >= 0) {
        MSTrafficLightLogic::addLink(link, lane, pos);
    } // ignore uncontrolled link
}


std::string
MSRailSignal::describeLinks(std::vector<MSLink*> links) {
    std::string result;
    for (MSLink* link : links) {
        result += link->getDescription() + " ";
    }
    return result;
}


void
MSRailSignal::writeBlocks(OutputDevice& od, bool writeVehicles) const {
    od.openTag("railSignal");
    od.writeAttr(SUMO_ATTR_ID, getID());
    for (const LinkInfo& li : myLinkInfos) {
        MSLink* link = li.myLink;
        od.openTag("link");
        od.writeAttr(SUMO_ATTR_TLLINKINDEX, link->getTLIndex());
        od.writeAttr(SUMO_ATTR_FROM, link->getLaneBefore()->getID());
        od.writeAttr(SUMO_ATTR_TO, link->getViaLaneOrLane()->getID());
        for (const MSDriveWay* dw : li.myDriveways) {
            if (writeVehicles) {
                dw->writeBlockVehicles(od);
            } else {
                dw->writeBlocks(od);
            }
        }
        od.closeTag(); // link
    }
    od.closeTag(); // railSignal
}


void
MSRailSignal::initDriveWays(const SUMOVehicle* ego, bool update) {
    const ConstMSEdgeVector& edges = ego->getRoute().getEdges();
    int endIndex = ego->getParameter().arrivalEdge;
    if (endIndex < 0) {
        endIndex = (int)edges.size() - 1;
    }
    int departIndex = ego->getParameter().departEdge;
    MSDriveWay* prev = nullptr;
    if (update && ego->hasDeparted()) {
        // find last rail signal on the route and obtain the driveway
        const MSEdge* next = ego->getEdge();
        for (int i = ego->getRoutePosition() - 1; i > departIndex; i--) {
            const MSEdge* e = ego->getRoute().getEdges()[i];
            if (e->getToJunction()->getType() == SumoXMLNodeType::RAIL_SIGNAL) {
                const MSLink* link = e->getLanes().front()->getLinkTo(next->getLanes().front());
                //std::cout << SIMTIME << " veh=" << ego->getID() << " rp=" << ego->getRoutePosition()
                //    << " i=" << i << " e=" << e->getID() << " next=" << next->getID() << " link=" << (link == nullptr ? "NUL" : link->getDescription()) << "\n";
                if (link != nullptr && link->isTLSControlled()) {
                    MSRailSignal* rs = const_cast<MSRailSignal*>(dynamic_cast<const MSRailSignal*>(link->getTLLogic()));
                    LinkInfo& li = rs->myLinkInfos[link->getTLIndex()];
                    prev = &li.getDriveWay(ego, i);
                    departIndex = ego->getRoutePosition();
                    break;
                }
            }
            next = e;
        }
    }
    if (prev == nullptr) {
        prev = const_cast<MSDriveWay*>(MSDriveWay::getDepartureDriveway(ego, true));
    }
    if (update && ego->hasDeparted()) {
        MSBaseVehicle* veh = dynamic_cast<MSBaseVehicle*>(const_cast<SUMOVehicle*>(ego));
        if (!prev->hasTrain(veh) && prev->notifyEnter(*veh, prev->NOTIFICATION_REROUTE, nullptr) && !veh->hasReminder(prev)) {
            veh->addReminder(prev, 1);
        }
    }
    for (int i = departIndex; i <= endIndex - 1; i++) {
        const MSEdge* e = edges[i];
        if (e->isNormal() && e->getToJunction()->getType() == SumoXMLNodeType::RAIL_SIGNAL) {
            const MSEdge* e2 = edges[i + 1];
            for (MSLane* lane : e->getLanes()) {
                for (MSLink* link : lane->getLinkCont()) {
                    if (&link->getLane()->getEdge() == e2) {
                        MSRailSignal* rs = const_cast<MSRailSignal*>(dynamic_cast<const MSRailSignal*>(link->getTLLogic()));
                        if (rs != nullptr) {
                            LinkInfo& li = rs->myLinkInfos[link->getTLIndex()];
                            // init driveway
                            MSDriveWay* dw = &li.getDriveWay(ego, i);
                            MSRailSignalControl::getInstance().addDrivewayFollower(prev, dw);
                            MSRailSignalControl::getInstance().addDWDeadlockChecks(rs, prev);
                            MSRailSignalControl::getInstance().notifyApproach(link);
                            prev = dw;
                            if (update && rs->isActive()) {
                                // vehicle may have rerouted its intial trip
                                // after the states have been set
                                // @note: This is a hack because it could lead to invalid tls-output
                                // (it's still an improvement over switching based on default driveways)
                                if (!ego->hasDeparted()) {
                                    rs->updateCurrentPhase();
                                    rs->setTrafficLightSignals(SIMSTEP);
                                } else if (ego->hasDeparted() && i <= ego->getRoutePosition()) {
                                    MSBaseVehicle* veh = dynamic_cast<MSBaseVehicle*>(const_cast<SUMOVehicle*>(ego));
                                    if (!dw->hasTrain(veh) && dw->notifyEnter(*veh, dw->NOTIFICATION_REROUTE, nullptr) && !veh->hasReminder(dw)) {
                                        veh->addReminder(dw, 1);
                                        for (MSDriveWay* sub : dw->getSubDriveWays()) {
                                            if (!sub->hasTrain(veh) && sub->notifyEnter(*veh, dw->NOTIFICATION_REROUTE, nullptr) && !veh->hasReminder(sub)) {
                                                veh->addReminder(sub, 1);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}


bool
MSRailSignal::hasInsertionConstraint(MSLink* link, const MSVehicle* veh, std::string& info, bool& isInsertionOrder) {
    if (link->getJunction() != nullptr && link->getJunction()->getType() == SumoXMLNodeType::RAIL_SIGNAL) {
        const MSRailSignal* rs = dynamic_cast<const MSRailSignal*>(link->getTLLogic());
        if (rs != nullptr && rs->myConstraints.size() > 0) {
            const std::string tripID = veh->getParameter().getParameter("tripId", veh->getID());
            auto it = rs->myConstraints.find(tripID);
            if (it != rs->myConstraints.end()) {
                for (MSRailSignalConstraint* c : it->second) {
                    if (c->isInsertionConstraint() && !c->cleared()) {
#ifdef DEBUG_SIGNALSTATE
                        if (DEBUG_HELPER(rs)) {
                            std::cout << SIMTIME << " rsl=" << rs->getID() << " insertion constraint '" << c->getDescription() << "' for vehicle '" << veh->getID() << "' not cleared\n";
                        }
#endif
                        info = c->getDescription();
                        isInsertionOrder = c->getType() == MSRailSignalConstraint::ConstraintType::INSERTION_ORDER;
                        if (MSGlobals::gTimeToTeleportRSDeadlock > 0) {
                            const SUMOVehicle* foe = c->getFoe();
                            if (foe != nullptr) {
                                MSRailSignalControl::getInstance().addWaitRelation(veh, rs, foe, c);
                            }
                        }
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

// ===========================================================================
// LinkInfo method definitions
// ===========================================================================

MSRailSignal::LinkInfo::LinkInfo(MSLink* link):
    myLink(link) {
    reset();
}

MSRailSignal::LinkInfo::~LinkInfo() {
    for (MSDriveWay* dw : myDriveways) {
        delete dw;
    }
    myDriveways.clear();
}

void
MSRailSignal::LinkInfo::reset() {
    myLastRerouteTime = -1;
    myLastRerouteVehicle = nullptr;
    myDriveways.clear();
    myControlled = isRailwayOrShared(myLink->getViaLaneOrLane()->getPermissions())
                   && isRailwayOrShared(myLink->getLane()->getPermissions());
}


std::string
MSRailSignal::LinkInfo::getID() const {
    return myLink->getTLLogic()->getID() + "_" + toString(myLink->getTLIndex());
}


MSDriveWay&
MSRailSignal::LinkInfo::getDriveWay(const SUMOVehicle* veh, int searchStart) {
    MSEdge* first = &myLink->getLane()->getEdge();
    auto searchStartIt = searchStart < 0 ? veh->getCurrentRouteEdge() : veh->getRoute().begin() + searchStart;
    MSRouteIterator firstIt = std::find(searchStartIt, veh->getRoute().end(), first);
    if (firstIt == veh->getRoute().end()) {
        // possibly the vehicle has already gone past the first edge (i.e.
        // because first is short or the step-length is high)
        // lets look backward along the route
        // give some slack because the vehicle might have been braking from a higher speed and using ballistic integration
        double lookBack = SPEED2DIST(veh->getSpeed() + 10);
        int routeIndex = veh->getRoutePosition() - 1;
        while (lookBack > 0 && routeIndex > 0) {
            const MSEdge* prevEdge = veh->getRoute().getEdges()[routeIndex];
            if (prevEdge == first) {
                firstIt = veh->getRoute().begin() + routeIndex;
                break;
            }
            lookBack -= prevEdge->getLength();
            routeIndex--;
        }
    }
    MSRailSignal* rs = const_cast<MSRailSignal*>(dynamic_cast<const MSRailSignal*>(myLink->getTLLogic()));
    if (firstIt == veh->getRoute().end()) {
        WRITE_WARNING("Invalid approach information to rail signal '" + MSDriveWay::getClickableTLLinkID(myLink) + "' after rerouting for vehicle '" + veh->getID()
                      + "' first driveway edge '" + first->getID() + "' time=" + time2string(MSNet::getInstance()->getCurrentTimeStep()) + ".");
        if (myDriveways.empty()) {
            ConstMSEdgeVector dummyRoute;
            dummyRoute.push_back(&myLink->getLane()->getEdge());
            MSDriveWay* dw = MSDriveWay::buildDriveWay(rs->getNewDrivewayID(), myLink, dummyRoute.begin(), dummyRoute.end());
            myDriveways.push_back(dw);
        }
        return *myDriveways.front();
    }
    //std::cout << SIMTIME << " veh=" << veh->getID() << " rsl=" << getID() << " dws=" << myDriveways.size() << "\n";
    return getDriveWay(firstIt, veh->getRoute().end(), veh->getID());
}


MSDriveWay&
MSRailSignal::LinkInfo::getDriveWay(MSRouteIterator firstIt, MSRouteIterator endIt, const std::string& info) {
    for (MSDriveWay* dw : myDriveways) {
        if (dw->match(firstIt, endIt)) {
            return *dw;
        }
#ifdef DEBUG_SELECT_DRIVEWAY
        std::cout << SIMTIME << " rs=" << getID() << " veh=" << info << " other dwSignal=" << dw->foundSignal() << " dwRoute=" << toString(dw->getRoute()) << "\n";
#else
        UNUSED_PARAMETER(info);
#endif
    }
    MSRailSignal* rs = const_cast<MSRailSignal*>(dynamic_cast<const MSRailSignal*>(myLink->getTLLogic()));
    MSDriveWay* dw = MSDriveWay::buildDriveWay(rs->getNewDrivewayID(), myLink, firstIt, endIt);
    dw->setVehicle(info);
#ifdef DEBUG_SELECT_DRIVEWAY
    std::cout << SIMTIME << " rs=" << getID() << " veh=" << info << " new dwSignal=" << dw->foundSignal() << " dwRoute=" << toString(dw->getRoute()) << "\n";
#endif
    myDriveways.push_back(dw);
    return *myDriveways.back();
}


void
MSRailSignal::LinkInfo::reroute(SUMOVehicle* veh, const MSEdgeVector& occupied) {
    MSDevice_Routing* rDev = static_cast<MSDevice_Routing*>(veh->getDevice(typeid(MSDevice_Routing)));
    const SUMOTime now = MSNet::getInstance()->getCurrentTimeStep();
    if (rDev != nullptr
            && rDev->mayRerouteRailSignal()
            && (myLastRerouteVehicle != veh
                // reroute each vehicle only once if no periodic routing is allowed,
                // otherwise with the specified period
                || (rDev->getPeriod() > 0 && myLastRerouteTime + rDev->getPeriod() <= now))) {
        myLastRerouteVehicle = veh;
        myLastRerouteTime = now;

#ifdef DEBUG_REROUTE
        ConstMSEdgeVector oldRoute = veh->getRoute().getEdges();
        if (DEBUG_COND_LINKINFO) {
            std::cout << SIMTIME << " reroute veh=" << veh->getID() << " rs=" << getID() << " occupied=" << toString(occupied) << "\n";
        }
#endif
        MSRoutingEngine::Prohibitions prohibited;
        for (MSEdge* e : occupied) {
            // indefinite occupation because vehicles might be in deadlock on their current routes
            prohibited[e].end = std::numeric_limits<double>::max();
        }
        MSRoutingEngine::reroute(*veh, now, "railSignal:" + getID(), false, true, prohibited);
#ifdef DEBUG_REROUTE
        // attention this works only if we are not parallel!
        if (DEBUG_COND_LINKINFO) {
            if (veh->getRoute().getEdges() != oldRoute) {
                std::cout << "    rerouting successful\n";
            }
        }
#endif
    }
}

void
MSRailSignal::resetStored() {
    myBlockingVehicles.clear();
    myRivalVehicles.clear();
    myPriorityVehicles.clear();
    myConstraintInfo = "";
    myBlockingDriveWays.clear();
    myRequestedDriveWay = "";
}


void
MSRailSignal::storeTraCIVehicles(const MSDriveWay* dw) {
    resetStored();
    myStoreVehicles = true;
    MSEdgeVector occupied;
    // call for side effects
    dw->foeDriveWayOccupied(true, nullptr, occupied);
    myStoreVehicles = false;
}

void
MSRailSignal::storeTraCIVehicles(int linkIndex) {
    resetStored();
    myStoreVehicles = true;
    LinkInfo& li = myLinkInfos[linkIndex];
    if (li.myLink->getApproaching().size() > 0) {
        Approaching closest = li.myLink->getClosest();
        MSDriveWay& driveway = li.getDriveWay(closest.first);
        MSEdgeVector occupied;
        myRequestedDriveWay = driveway.getID();
        // call for side effects
        driveway.reserve(closest, occupied);
        constraintsAllow(closest.first);
    } else if (li.myDriveways.size() > 0) {
        li.myDriveways.front()->conflictLaneOccupied();
        li.myDriveways.front()->foeDriveWayApproached();
    }
    myStoreVehicles = false;
}

MSRailSignal::VehicleVector
MSRailSignal::getBlockingVehicles(int linkIndex) {
    storeTraCIVehicles(linkIndex);
    return myBlockingVehicles;
}

MSRailSignal::VehicleVector
MSRailSignal::getRivalVehicles(int linkIndex) {
    storeTraCIVehicles(linkIndex);
    return myRivalVehicles;
}

MSRailSignal::VehicleVector
MSRailSignal::getPriorityVehicles(int linkIndex) {
    storeTraCIVehicles(linkIndex);
    return myPriorityVehicles;
}

std::string
MSRailSignal::getConstraintInfo(int linkIndex) {
    storeTraCIVehicles(linkIndex);
    return myConstraintInfo;
}


std::string
MSRailSignal::getRequestedDriveWay(int linkIndex) {
    storeTraCIVehicles(linkIndex);
    return myRequestedDriveWay;
}


std::vector<const MSDriveWay*>
MSRailSignal::getBlockingDriveWays(int linkIndex) {
    storeTraCIVehicles(linkIndex);
    return myBlockingDriveWays;
}


MSRailSignal::VehicleVector
MSRailSignal::getBlockingVehicles(const MSDriveWay* dw) {
    storeTraCIVehicles(dw);
    return myBlockingVehicles;
}


std::vector<const MSDriveWay*>
MSRailSignal::getBlockingDriveWays(const MSDriveWay* dw) {
    storeTraCIVehicles(dw);
    return myBlockingDriveWays;
}

const MSDriveWay&
MSRailSignal::retrieveDriveWay(int numericalID) const {
    for (const LinkInfo& li : myLinkInfos) {
        for (const MSDriveWay* dw : li.myDriveways) {
            if (dw->getNumericalID() == numericalID) {
                return *dw;
            }
        }
    }
    throw ProcessError("Invalid driveway id " + toString(numericalID) + " at railSignal '" + getID() + "'");
}

const MSDriveWay&
MSRailSignal::retrieveDriveWayForVeh(int tlIndex, const SUMOVehicle* veh) {
    return myLinkInfos[tlIndex].getDriveWay(veh);
}

const MSDriveWay&
MSRailSignal::retrieveDriveWayForRoute(int tlIndex, MSRouteIterator first, MSRouteIterator end) {
    return myLinkInfos[tlIndex].getDriveWay(first, end);
}


const std::vector<MSDriveWay*>
MSRailSignal::retrieveDriveWays(int tlIndex) const {
    return myLinkInfos[tlIndex].myDriveways;
}


std::string
MSRailSignal::getBlockingVehicleIDs() const {
    MSRailSignal* rs = const_cast<MSRailSignal*>(this);
    if (myLinkInfos.size() == 1) {
        return toString(rs->getBlockingVehicles(0));
    } else {
        std::string result;
        for (int i = 0; i < (int)myLinkInfos.size(); i++) {
            result += toString(i) + ": " + toString(rs->getBlockingVehicles(i)) + ";";
        }
        return result;
    }
}
std::string
MSRailSignal::getRivalVehicleIDs() const {
    MSRailSignal* rs = const_cast<MSRailSignal*>(this);
    if (myLinkInfos.size() == 1) {
        return toString(rs->getRivalVehicles(0));
    } else {
        std::string result;
        for (int i = 0; i < (int)myLinkInfos.size(); i++) {
            result += toString(i) + ": " + toString(rs->getRivalVehicles(i)) + ";";
        }
        return result;
    }
}
std::string
MSRailSignal::getPriorityVehicleIDs() const {
    MSRailSignal* rs = const_cast<MSRailSignal*>(this);
    if (myLinkInfos.size() == 1) {
        return toString(rs->getPriorityVehicles(0));
    } else {
        std::string result;
        for (int i = 0; i < (int)myLinkInfos.size(); i++) {
            result += toString(i) + ": " + toString(rs->getPriorityVehicles(i)) + ";";
        }
        return result;
    }
}
std::string
MSRailSignal::getConstraintInfo() const {
    MSRailSignal* rs = const_cast<MSRailSignal*>(this);
    if (myLinkInfos.size() == 1) {
        return rs->getConstraintInfo(0);
    } else {
        std::string result;
        for (int i = 0; i < (int)myLinkInfos.size(); i++) {
            result += toString(i) + ": " + rs->getConstraintInfo(i);
        }
        return result;
    }
}
std::string
MSRailSignal::getRequestedDriveWay() const {
    MSRailSignal* rs = const_cast<MSRailSignal*>(this);
    if (myLinkInfos.size() == 1) {
        return toString(rs->getRequestedDriveWay(0));
    } else {
        std::string result;
        for (int i = 0; i < (int)myLinkInfos.size(); i++) {
            result += toString(i) + ": " + toString(rs->getRequestedDriveWay(i)) + ";";
        }
        return result;
    }
}
std::string
MSRailSignal::getBlockingDriveWayIDs() const {
    MSRailSignal* rs = const_cast<MSRailSignal*>(this);
    if (myLinkInfos.size() == 1) {
        return toString(rs->getBlockingDriveWays(0));
    } else {
        std::string result;
        for (int i = 0; i < (int)myLinkInfos.size(); i++) {
            result += toString(i) + ": " + toString(rs->getBlockingDriveWays(i)) + ";";
        }
        return result;
    }
}

void
MSRailSignal::setParameter(const std::string& key, const std::string& value) {
    // some pre-defined parameters can be updated at runtime
    if (key == "moving-block") {
        bool movingBlock = StringUtils::toBool(value);
        if (movingBlock != myMovingBlock) {
            // recompute driveways
            myMovingBlock = movingBlock;
            for (LinkInfo& li : myLinkInfos) {
                li.reset();
            }
            updateCurrentPhase();
            setTrafficLightSignals(MSNet::getInstance()->getCurrentTimeStep());
        }
    }
    Parameterised::setParameter(key, value);
}


std::string
MSRailSignal::getNewDrivewayID() {
    return getID() + "." + toString(myDriveWayIndex++);
}
/****************************************************************************/
