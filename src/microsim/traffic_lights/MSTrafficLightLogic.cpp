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
/// @file    MSTrafficLightLogic.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
///
// The parent class for traffic light logics
/****************************************************************************/
#include <config.h>

#include <cassert>
#include <string>
#include <iostream>
#include <map>
#include <microsim/MSLink.h>
#include <microsim/MSLane.h>
#include <microsim/MSEventControl.h>
#include <microsim/MSJunctionLogic.h>
#include <microsim/MSNet.h>
#include <microsim/MSEdge.h>
#include <microsim/MSGlobals.h>
#include <mesosim/MESegment.h>
#include "MSTLLogicControl.h"
#include "MSTrafficLightLogic.h"


// ===========================================================================
// static value definitions
// ===========================================================================
const MSTrafficLightLogic::LaneVector MSTrafficLightLogic::myEmptyLaneVector;


#define SHORT_EDGE ((SUMOVTypeParameter::getDefault().length + SUMOVTypeParameter::getDefault().minGap) * 2)

// ===========================================================================
// member method definitions
// ===========================================================================
/* -------------------------------------------------------------------------
 * member method definitions
 * ----------------------------------------------------------------------- */
MSTrafficLightLogic::SwitchCommand::SwitchCommand(MSTLLogicControl& tlcontrol,
        MSTrafficLightLogic* tlLogic, SUMOTime nextSwitch) :
    myTLControl(tlcontrol), myTLLogic(tlLogic),
    myAssumedNextSwitch(nextSwitch), myAmValid(true) {
    // higher than default command priority of 0
    priority = std::numeric_limits<int>::max();
}


MSTrafficLightLogic::SwitchCommand::~SwitchCommand() {}



SUMOTime
MSTrafficLightLogic::SwitchCommand::execute(SUMOTime t) {
    // check whether this command has been descheduled
    if (!myAmValid) {
        return 0;
    }
    int step1 = myTLLogic->getCurrentPhaseIndex();
    SUMOTime next = myTLLogic->trySwitch();
    while (next == 0) {
        // skip phase and switch again
        next = myTLLogic->trySwitch();
    }
    int step2 = myTLLogic->getCurrentPhaseIndex();
    if (step1 != step2) {
        if (myTLLogic->isActive()) {
            // execute any action connected to this tls
            const MSTLLogicControl::TLSLogicVariants& vars = myTLControl.get(myTLLogic->getID());
            // set link priorities
            myTLLogic->setTrafficLightSignals(t);
            // execute switch actions
            vars.executeOnSwitchActions();
        }
    }
    myAssumedNextSwitch += next;
    return next;
}


void
MSTrafficLightLogic::SwitchCommand::deschedule(MSTrafficLightLogic* tlLogic) {
    if (tlLogic == myTLLogic) {
        myAmValid = false;
        myAssumedNextSwitch = -1;
    }
}


SUMOTime
MSTrafficLightLogic::SwitchCommand::shiftTime(SUMOTime currentTime, SUMOTime execTime, SUMOTime newTime) {
    if (myTLLogic->getDefaultCycleTime() == DELTA_T) {
        // MSRailSignal
        return newTime;
    } else {
        UNUSED_PARAMETER(currentTime);
        UNUSED_PARAMETER(execTime);
        // XXX changeStepAndDuration (computed as in NLJunctionControlBuilder::closeTrafficLightLogic
        return newTime;
    }
}

/* -------------------------------------------------------------------------
 * member method definitions
 * ----------------------------------------------------------------------- */
MSTrafficLightLogic::MSTrafficLightLogic(MSTLLogicControl& tlcontrol, const std::string& id,
        const std::string& programID, const SUMOTime offset, const TrafficLightType logicType, const SUMOTime delay,
        const Parameterised::Map& parameters) :
    Named(id), Parameterised(parameters),
    myProgramID(programID),
    myOffset(offset),
    myLogicType(logicType),
    myCurrentDurationIncrement(-1),
    myDefaultCycleTime(0),
    myAmActive(true) {
    mySwitchCommand = new SwitchCommand(tlcontrol, this, delay);
    MSNet::getInstance()->getBeginOfTimestepEvents()->addEvent(mySwitchCommand, delay);
}


void
MSTrafficLightLogic::init(NLDetectorBuilder&) {
    const Phases& phases = getPhases();
    if (phases.size() > 0 && (MSGlobals::gUseMesoSim || MSGlobals::gTLSPenalty > 0)) {
        initMesoTLSPenalties();
    }
    if (phases.size() > 1) {
        bool haveWarnedAboutUnusedStates = false;
        for (int i = 0; i < (int)phases.size(); ++i) {
            // warn about unused states
            std::vector<int> nextPhases;
            nextPhases.push_back((i + 1) % phases.size());
            bool iNextDefault = true;
            if (phases[i]->nextPhases.size() > 0) {
                nextPhases = phases[i]->nextPhases;
                iNextDefault = false;
            }
            for (int iNext : nextPhases) {
                if (iNext < 0 || iNext >= (int)phases.size()) {
                    throw ProcessError("Invalid nextPhase " + toString(iNext) + " in tlLogic '" + getID()
                                       + "', program '" + getProgramID() + "' with " + toString(phases.size()) + " phases");
                }
                if (iNext == i && (int)nextPhases.size() == 1 && (int)phases.size() > 1) {
                    WRITE_WARNINGF("Phase % only loops backs to itself in tlLogic '%', program '%'.", i, getID(), getProgramID());
                }
                const std::string optionalFrom = iNextDefault ? "" : " from phase " + toString(i);
                const std::string& state1 = phases[i]->getState();
                const std::string& state2 = phases[iNext]->getState();
                if (state1.size() != state2.size()) {
                    throw ProcessError("Mismatching phase state length in tlLogic '" + getID()
                                       + "', program '" + getProgramID() + "' in phases " + toString(i) + " and " + toString(iNext));
                }
                if (!haveWarnedAboutUnusedStates && state1.size() > myLanes.size() + myIgnoredIndices.size()) {
                    WRITE_WARNINGF(TL("Unused states in tlLogic '%', program '%' in phase % after tl-index %"),
                                   getID(), getProgramID(), i, (int)myLanes.size() - 1);
                    haveWarnedAboutUnusedStates = true;
                }
                // detect illegal states
                const std::string::size_type illegal = state1.find_first_not_of(SUMOXMLDefinitions::ALLOWED_TLS_LINKSTATES);
                if (std::string::npos != illegal) {
                    throw ProcessError("Illegal character '" + toString(state1[illegal]) + "' in tlLogic '" + getID()
                                       + "', program '" + getProgramID() + "' in phase " + toString(i));
                }
                // warn about transitions from green to red without intermediate yellow
                bool haveWarned = false;
                for (int j = 0; j < (int)MIN3(state1.size(), state2.size(), myLanes.size()) && !haveWarned; ++j) {
                    if ((LinkState)state2[j] == LINKSTATE_TL_RED
                            && ((LinkState)state1[j] == LINKSTATE_TL_GREEN_MAJOR
                                || (LinkState)state1[j] == LINKSTATE_TL_GREEN_MINOR)) {
                        for (LaneVector::const_iterator it = myLanes[j].begin(); it != myLanes[j].end(); ++it) {
                            if ((*it)->getPermissions() != SVC_PEDESTRIAN) {
                                if (getLogicType() != TrafficLightType::NEMA) {
                                    WRITE_WARNINGF(TL("Missing yellow phase in tlLogic '%', program '%' for tl-index % when switching% to phase %."),
                                                   getID(), getProgramID(), j, optionalFrom, iNext);
                                    // one warning per program is enough
                                    haveWarned = true;
                                }
                                break;
                            }
                        }
                    }
                }
            }
        }
    }
    // warn about links that never get the green light
    std::vector<bool> foundGreen(phases.front()->getState().size(), false);
    for (int i = 0; i < (int)phases.size(); ++i) {
        const std::string& state = phases[i]->getState();
        for (int j = 0; j < (int)state.size(); ++j) {
            LinkState ls = (LinkState)state[j];
            if (ls == LINKSTATE_TL_GREEN_MAJOR || ls == LINKSTATE_TL_GREEN_MINOR || ls == LINKSTATE_TL_OFF_BLINKING || ls == LINKSTATE_TL_OFF_NOSIGNAL || ls == LINKSTATE_STOP) {
                foundGreen[j] = true;
            }
        }
    }
    std::vector<bool> usedIndices(phases.front()->getState().size(), false);
    for (auto lv : myLinks) {
        for (const MSLink* link : lv) {
            if (link->getTLIndex() >= 0) {
                usedIndices[link->getTLIndex()] = true;
            }
        }
    }
    for (int j = 0; j < (int)foundGreen.size(); ++j) {
        if (!foundGreen[j] && usedIndices[j]) {
            WRITE_WARNINGF(TL("Missing green phase in tlLogic '%', program '%' for tl-index %."), getID(), getProgramID(), j);
            break;
        }
    }
    // check direct conflict (two green links targeting the same lane)
    const int numLinks = (int)myLinks.size();
    std::set<const MSLane*> unsafeGreen;
    int firstUnsafePhase = -1;
    const MSLane* firstUnsafeLane = nullptr;
    int firstUnsafeOrigins = 0;
    int unsafeGreenPhases = 0;
    for (int i = 0; i < (int)phases.size(); ++i) {
        std::map<const MSLane*, int, ComparatorNumericalIdLess> greenLanes;
        const std::string& state = phases[i]->getState();
        for (int j = 0; j < numLinks; ++j) {
            if (state[j] == LINKSTATE_TL_GREEN_MAJOR) {
                for (const MSLink* link : myLinks[j]) {
                    if (link->isInternalJunctionLink()) {
                        // links from an internal junction have implicit priority in case of conflict
                        continue;
                    }
                    greenLanes[link->getLane()] += 1;
                }
            }
        }
        bool unsafe = false;
        for (auto item : greenLanes) {
            if (item.second > 1) {
                if (unsafeGreenPhases == 0 && !unsafe) {
                    firstUnsafePhase = i;
                    firstUnsafeLane = item.first;
                    firstUnsafeOrigins = item.second;
                }
                unsafe = true;
                unsafeGreen.insert(item.first);
            }
        }
        if (unsafe) {
            unsafeGreenPhases++;
        }
    }
    if (unsafeGreenPhases > 0) {
        const std::string furtherAffected = unsafeGreen.size() > 1 || unsafeGreenPhases > 1 ? TLF(" Overall % lanes in % phases are unsafe.", unsafeGreen.size(), unsafeGreenPhases) : "";
        WRITE_WARNINGF(TL("Unsafe green phase % in tlLogic '%', program '%'. Lane '%' is targeted by % 'G'-links. (use 'g' instead)%"),
                firstUnsafePhase, getID(), getProgramID(), firstUnsafeLane->getID(), firstUnsafeOrigins, furtherAffected);
    }

    // check incompatible junction logic
    // this can happen if the network was built with a very different signal
    // plan from the one currently being used.
    // Connections that never had a common green phase during network building may
    // have a symmetric response relation to avoid certain kinds of jam but this
    // can lead to deadlock if a different program gives minor green to both
    // connections at the same time
    // Note: mutual conflict between 'g' and 'G' is expected for traffic_light_right_on_red

    const bool mustCheck = MSNet::getInstance()->hasInternalLinks();
    // The checks only runs for definitions from additional file and this is sufficient.
    // The distinction is implicit because original logics are loaded earlier and at that time hasInternalLinks is always false
    // Also, when the network has no internal links, mutual conflicts are not built by netconvert
    //std::cout << "init tlLogic=" << getID() << " prog=" << getProgramID() << " links=" << myLinks.size() << " internal=" << MSNet::getInstance()->hasInternalLinks() << "\n";
    if (mustCheck && phases.size() > 0) {
        // see NBNode::tlsConflict
        std::set<const MSJunction*> controlledJunctions;
        for (int j = 0; j < numLinks; ++j) {
            for (int k = 0; k < (int)myLinks[j].size(); ++k) {
                MSLink* link = myLinks[j][k];
                assert(link->getJunction() != nullptr);
                controlledJunctions.insert(link->getJunction());
            }
        }
        const std::string minor = "gos";
        for (const MSJunction* junction : controlledJunctions) {
            const MSJunctionLogic* logic = junction->getLogic();
            if (logic != nullptr) {
                // find symmetrical response
                const int logicSize = logic->getLogicSize();
                bool foundProblem = false;
                std::vector<int> tlIndex;
                for (int u = 0; u < logicSize && !foundProblem; u++) {
                    const MSLogicJunction::LinkBits& response = logic->getResponseFor(u);
                    for (int v = 0; v < logicSize && !foundProblem; v++) {
                        if (response.test(v)) {
                            if (logic->getResponseFor(v).test(u)) {
                                // get tls link index for links u and v
                                if (tlIndex.size() == 0) {
                                    // init tlindex for all links  once
                                    tlIndex.resize(logicSize, -1);
                                    for (int j = 0; j < numLinks; ++j) {
                                        for (int k = 0; k < (int)myLinks[j].size(); ++k) {
                                            MSLink* link = myLinks[j][k];
                                            if (link->getJunction() == junction) {
                                                if (link->fromInternalLane()) {
                                                    // internal links may have their own control (i.e. for indirect left turn) but they also have their own conflict matrix
                                                    continue;
                                                }
                                                if (link->isCont() && link->getViaLane() != nullptr && link->getViaLane()->getLinkCont()[0]->getTLIndex() >= 0) {
                                                    // if the internal junction link is controlled, the first-part indices are not deadlock-relevant
                                                    continue;
                                                }
                                                tlIndex[link->getIndex()] = link->getTLIndex();
                                            }
                                        }
                                    }
                                }
                                const int tlu = tlIndex[u];
                                const int tlv = tlIndex[v];
                                if (tlu >= 0 && tlv >= 0) {
                                    int phaseIndex = 0;
                                    for (MSPhaseDefinition* p : phases) {
                                        if (minor.find(p->getState()[tlu]) != std::string::npos
                                                && minor.find(p->getState()[tlv]) != std::string::npos) {
                                            WRITE_WARNING(TLF("Program '%' at tlLogic '%' is incompatible with logic at junction '%' (mutual conflict between link indices %,% tl indices %,% phase %).\n"
                                                              "  To avoid deadlock/collisions, either: rebuild the signal plan with a newer version of netconvert/netedit\n"
                                                              "  or rebuild the network with option '--tls.ignore-internal-junction-jam' or include the program when building.",
                                                              getProgramID(), getID(), junction->getID(), u, v, tlu, tlv, phaseIndex));
                                            // only one warning per program
                                            foundProblem = true;
                                            break;
                                        }
                                        phaseIndex++;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    myNumLinks = (int)myLinks.size();
}


MSTrafficLightLogic::~MSTrafficLightLogic() {
    // no need to do something about mySwitchCommand here,
    // it is handled by the event control
}


// ----------- Handling of controlled links
void
MSTrafficLightLogic::addLink(MSLink* link, MSLane* lane, int pos) {
    // !!! should be done within the loader (checking necessary)
    myLinks.reserve(pos + 1);
    while ((int)myLinks.size() <= pos) {
        myLinks.push_back(LinkVector());
    }
    myLinks[pos].push_back(link);
    //
    myLanes.reserve(pos + 1);
    while ((int)myLanes.size() <= pos) {
        myLanes.push_back(LaneVector());
    }
    myLanes[pos].push_back(lane);
    link->setTLState((LinkState) getCurrentPhaseDef().getState()[pos], MSNet::getInstance()->getCurrentTimeStep());
}


void
MSTrafficLightLogic::adaptLinkInformationFrom(const MSTrafficLightLogic& logic) {
    myLinks = logic.myLinks;
    myLanes = logic.myLanes;
    myIgnoredIndices = logic.myIgnoredIndices;
}


std::map<MSLink*, LinkState>
MSTrafficLightLogic::collectLinkStates() const {
    std::map<MSLink*, LinkState> ret;
    for (LinkVectorVector::const_iterator i1 = myLinks.begin(); i1 != myLinks.end(); ++i1) {
        const LinkVector& l = (*i1);
        for (LinkVector::const_iterator i2 = l.begin(); i2 != l.end(); ++i2) {
            ret[*i2] = (*i2)->getState();
        }
    }
    return ret;
}


bool
MSTrafficLightLogic::setTrafficLightSignals(SUMOTime t) const {
    // get the current traffic light signal combination
    const std::string& state = getCurrentPhaseDef().getState();
    // go through the links
    for (int i = 0; i < (int)myLinks.size(); i++) {
        const LinkVector& currGroup = myLinks[i];
        LinkState ls = (LinkState) state[i];
        for (LinkVector::const_iterator j = currGroup.begin(); j != currGroup.end(); j++) {
            (*j)->setTLState(ls, t);
        }
    }
    return true;
}


void
MSTrafficLightLogic::resetLinkStates(const std::map<MSLink*, LinkState>& vals) const {
    for (LinkVectorVector::const_iterator i1 = myLinks.begin(); i1 != myLinks.end(); ++i1) {
        const LinkVector& l = (*i1);
        for (LinkVector::const_iterator i2 = l.begin(); i2 != l.end(); ++i2) {
            assert(vals.find(*i2) != vals.end());
            (*i2)->setTLState(vals.find(*i2)->second, MSNet::getInstance()->getCurrentTimeStep());
        }
    }
}


// ----------- Static Information Retrieval
int
MSTrafficLightLogic::getLinkIndex(const MSLink* const link) const {
    int index = 0;
    for (LinkVectorVector::const_iterator i1 = myLinks.begin(); i1 != myLinks.end(); ++i1, ++index) {
        const LinkVector& l = (*i1);
        for (LinkVector::const_iterator i2 = l.begin(); i2 != l.end(); ++i2) {
            if ((*i2) == link) {
                return index;
            }
        }
    }
    return -1;
}



// ----------- Dynamic Information Retrieval
SUMOTime
MSTrafficLightLogic::getNextSwitchTime() const {
    return mySwitchCommand != nullptr ? mySwitchCommand->getNextSwitchTime() : -1;
}


SUMOTime
MSTrafficLightLogic::getSpentDuration(SUMOTime simStep) const {
    if (simStep == -1) {
        simStep = SIMSTEP;
    }
    return simStep - getCurrentPhaseDef().myLastSwitch;
}


// ----------- Changing phases and phase durations
void
MSTrafficLightLogic::addOverridingDuration(SUMOTime duration) {
    myOverridingTimes.push_back(duration);
}


void
MSTrafficLightLogic::setCurrentDurationIncrement(SUMOTime delay) {
    myCurrentDurationIncrement = delay;
}


void MSTrafficLightLogic::initMesoTLSPenalties() {
    if (myLogicType == TrafficLightType::RAIL_SIGNAL) {
        return;
    }
    // set mesoscopic time penalties
    const Phases& phases = getPhases();
    const int numLinks = (int)myLinks.size();
    // warning already given if not all states are used
    assert(numLinks <= (int)phases.front()->getState().size());
    SUMOTime duration = 0;
    std::vector<double> firstRedDuration(numLinks, 0);
    std::vector<double> redDuration(numLinks, 0);
    std::vector<double> totalRedDuration(numLinks, 0);
    std::vector<double> penalty(numLinks, 0);
    for (int i = 0; i < (int)phases.size(); ++i) {
        const std::string& state = phases[i]->getState();
        duration += phases[i]->duration;
        // warn about transitions from green to red without intermediate yellow
        for (int j = 0; j < numLinks; ++j) {
            double& red = redDuration[j];
            if ((LinkState)state[j] == LINKSTATE_TL_RED
                    || (LinkState)state[j] == LINKSTATE_TL_REDYELLOW) {
                red += STEPS2TIME(phases[i]->duration);
                totalRedDuration[j] += STEPS2TIME(phases[i]->duration);
            } else if (red > 0) {
                if (firstRedDuration[j] == 0) {
                    // store for handling wrap-around
                    firstRedDuration[j] = red;
                } else {
                    // vehicle may arive in any second or the red duration
                    // compute the sum over [0,red]
                    penalty[j] += 0.5 * (red * red + red);
                }
                red = 0;
            }
        }
    }
    // final phase and wrap-around to first phase
    for (int j = 0; j < numLinks; ++j) {
        double red = redDuration[j] + firstRedDuration[j];
        if (red) {
            penalty[j] += 0.5 * (red * red + red);
        }
    }
    double tlsPenalty = MSGlobals::gTLSPenalty;
    const double durationSeconds = STEPS2TIME(duration);
    std::set<const MSJunction*> controlledJunctions;
    std::set<const MSEdge*> shortEdges;;
    for (int j = 0; j < numLinks; ++j) {
        for (int k = 0; k < (int)myLinks[j].size(); ++k) {
            MSLink* link = myLinks[j][k];
            MSEdge& edge = link->getLaneBefore()->getEdge();
            if (MSGlobals::gUseMesoSim) {
                const MESegment::MesoEdgeType& edgeType = MSNet::getInstance()->getMesoType(edge.getEdgeType());
                tlsPenalty = edgeType.tlsPenalty;
                double greenFraction = (durationSeconds - totalRedDuration[j]) / durationSeconds;
                if (edgeType.tlsFlowPenalty == 0) {
                    greenFraction = 1;
                } else {
                    greenFraction = MAX2(MIN2(greenFraction / edgeType.tlsFlowPenalty, 1.0), 0.01);
                }
                if (greenFraction == 0.01) {
                    WRITE_WARNINGF(TL("Green fraction is only 1% for link % in tlLogic '%', program '%'."), "%", j, getID(), getProgramID());
                }
                link->setGreenFraction(greenFraction);
                if (tlsPenalty > 0 && edge.getLength() < SHORT_EDGE && shortEdges.count(&edge) == 0) {
                    shortEdges.insert(&edge);
                    WRITE_WARNINGF(TL("Edge '%' is shorter than %m (%m) and will cause incorrect flow reduction with option --meso-tls-penalty"), edge.getID(), SHORT_EDGE, edge.getLength());
                }
            }
            link->setMesoTLSPenalty(TIME2STEPS(tlsPenalty * penalty[j] / durationSeconds));
            controlledJunctions.insert(link->getLane()->getEdge().getFromJunction()); // MSLink::myJunction is not yet initialized
            //std::cout << " tls=" << getID() << " i=" << j << " link=" << link->getDescription() << " p=" << penalty[j] << " fr=" << firstRedDuration[j] << " r=" << redDuration[j] << " tr=" << totalRedDuration[j] << " durSecs=" << durationSeconds << " tlsPen=" << STEPS2TIME(link->getMesoTLSPenalty()) << " gF=" << myLinks[j][k]->getGreenFraction() << "\n";
        }
    }
    // initialize empty-net travel times
    // XXX refactor after merging sharps (links know their incoming edge)
    for (std::set<const MSJunction*>::iterator it = controlledJunctions.begin(); it != controlledJunctions.end(); ++it) {
        const ConstMSEdgeVector incoming = (*it)->getIncoming();
        for (ConstMSEdgeVector::const_iterator it_e = incoming.begin(); it_e != incoming.end(); ++it_e) {
            const_cast<MSEdge*>(*it_e)->recalcCache();
        }
    }

}


void
MSTrafficLightLogic::ignoreLinkIndex(int pos) {
    myIgnoredIndices.insert(pos);
}

SUMOTime
MSTrafficLightLogic::getTimeInCycle() const {
    return mapTimeInCycle(SIMSTEP);
}


SUMOTime
MSTrafficLightLogic::mapTimeInCycle(SUMOTime t) const {
    return (t - myOffset) % myDefaultCycleTime;
}


bool
MSTrafficLightLogic::isSelected() const {
    return MSNet::getInstance()->isSelected(this);
}


void
MSTrafficLightLogic::activateProgram() {
    myAmActive = true;
    // updated the traffic light logic stored in the link
    for (const LinkVector& currGroup : myLinks) {
        for (MSLink* link : currGroup) {
            link->setTLLogic(this);
        }
    }
}


void
MSTrafficLightLogic::deactivateProgram() {
    myAmActive = false;
}

bool
MSTrafficLightLogic::getsMajorGreen(int linkIndex) const {
    if (linkIndex >= 0 && linkIndex < getNumLinks()) {
        for (const MSPhaseDefinition* p : getPhases()) {
            const std::string& s = p->getState();
            assert(linkIndex < (int)s.size());
            if (s[linkIndex] == LINKSTATE_TL_GREEN_MAJOR) {
                return true;
            }
        }
    }
    return false;

}


SUMOTime
MSTrafficLightLogic::getMinDur(int step) const {
    const MSPhaseDefinition& p = step < 0 ? getCurrentPhaseDef() : getPhase(step);
    return p.minDuration;
}

SUMOTime
MSTrafficLightLogic::getMaxDur(int step) const {
    const MSPhaseDefinition& p = step < 0 ? getCurrentPhaseDef() : getPhase(step);
    return p.maxDuration;
}

SUMOTime
MSTrafficLightLogic::getEarliestEnd(int step) const {
    const MSPhaseDefinition& p = step < 0 ? getCurrentPhaseDef() : getPhase(step);
    return p.earliestEnd;
}

SUMOTime
MSTrafficLightLogic::getLatestEnd(int step) const {
    const MSPhaseDefinition& p = step < 0 ? getCurrentPhaseDef() : getPhase(step);
    return p.latestEnd;
}


void
MSTrafficLightLogic::loadState(MSTLLogicControl& tlcontrol, SUMOTime t, int step, SUMOTime spentDuration) {
    const SUMOTime remaining = getPhase(step).duration - spentDuration;
    changeStepAndDuration(tlcontrol, t, step, remaining);
    setTrafficLightSignals(t - spentDuration);
}


SUMOTime
MSTrafficLightLogic::computeCycleTime(const Phases& phases) {
    SUMOTime result = 0;
    for (const MSPhaseDefinition* p : phases) {
        result += p->duration;
    }
    return result;
}



/****************************************************************************/
