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
/// @file    MSSimpleTrafficLightLogic.cpp
/// @author  Daniel Krajzewicz
/// @author  Julia Ringel
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Friedemann Wesner
/// @date    Sept 2002
///
// A fixed traffic light logic
/****************************************************************************/
#include <config.h>

#include <cassert>
#include <utility>
#include <vector>
#include <bitset>
#include <sstream>
#include <utils/common/StringUtils.h>
#include <microsim/MSEventControl.h>
#include <microsim/MSNet.h>
#include <microsim/MSEventControl.h>
#include "MSTLLogicControl.h"
#include "MSTrafficLightLogic.h"
#include "MSSimpleTrafficLightLogic.h"

//#define DEBUG_COORDINATION
#define DEBUG_COND (getID()=="C")


// ===========================================================================
// member method definitions
// ===========================================================================
MSSimpleTrafficLightLogic::MSSimpleTrafficLightLogic(MSTLLogicControl& tlcontrol,
        const std::string& id, const std::string& programID, const SUMOTime offset, const TrafficLightType logicType, const Phases& phases,
        int step, SUMOTime delay,
        const Parameterised::Map& parameters) :
    MSTrafficLightLogic(tlcontrol, id, programID, offset, logicType, delay, parameters),
    myPhases(phases),
    myStep(step) {
    myDefaultCycleTime = computeCycleTime(myPhases);
    if (myStep < (int)myPhases.size()) {
        myPhases[myStep]->myLastSwitch = SIMSTEP;
    }
    // the following initializations are only used by 'actuated' and 'delay_based' but do not affect 'static'
    if (hasParameter(toString(SUMO_ATTR_CYCLETIME))) {
        myDefaultCycleTime = TIME2STEPS(StringUtils::toDouble(Parameterised::getParameter(toString(SUMO_ATTR_CYCLETIME), "")));
    }
    myCoordinated = StringUtils::toBool(Parameterised::getParameter("coordinated", "false"));
    if (myPhases.size() > 0) {
        SUMOTime earliest = SIMSTEP + getEarliest(-1);
        if (earliest > getNextSwitchTime()) {
            mySwitchCommand->deschedule(this);
            mySwitchCommand = new SwitchCommand(tlcontrol, this, earliest);
            MSNet::getInstance()->getBeginOfTimestepEvents()->addEvent(mySwitchCommand, earliest);
        }
    }
}


MSSimpleTrafficLightLogic::~MSSimpleTrafficLightLogic() {
    deletePhases();
}


// ------------ Switching and setting current rows
SUMOTime
MSSimpleTrafficLightLogic::trySwitch() {
    // check whether the current duration shall be increased
    if (myCurrentDurationIncrement > 0) {
        SUMOTime delay = myCurrentDurationIncrement;
        myCurrentDurationIncrement = 0;
        return delay;
    }

    // increment the index
    if (myPhases[myStep]->nextPhases.size() > 0 && myPhases[myStep]->nextPhases.front() >= 0) {
        myStep = myPhases[myStep]->nextPhases.front();
    } else {
        myStep++;
    }
    // if the last phase was reached ...
    if (myStep >= (int)myPhases.size()) {
        // ... set the index to the first phase
        myStep = 0;
    }
    assert((int)myPhases.size() > myStep);
    //stores the time the phase started
    myPhases[myStep]->myLastSwitch = MSNet::getInstance()->getCurrentTimeStep();
    // check whether the next duration was overridden
    if (myOverridingTimes.size() > 0) {
        SUMOTime nextDuration = myOverridingTimes[0];
        myOverridingTimes.erase(myOverridingTimes.begin());
        return nextDuration;
    }
    // return offset to the next switch
    return myPhases[myStep]->duration;
}


// ------------ Static Information Retrieval
int
MSSimpleTrafficLightLogic::getPhaseNumber() const {
    return (int) myPhases.size();
}


const MSSimpleTrafficLightLogic::Phases&
MSSimpleTrafficLightLogic::getPhases() const {
    return myPhases;
}


MSSimpleTrafficLightLogic::Phases&
MSSimpleTrafficLightLogic::getPhases() {
    return myPhases;
}


const MSPhaseDefinition&
MSSimpleTrafficLightLogic::getPhase(int givenStep) const {
    assert((int)myPhases.size() > givenStep);
    return *myPhases[givenStep];
}


// ------------ Dynamic Information Retrieval
int
MSSimpleTrafficLightLogic::getCurrentPhaseIndex() const {
    return myStep;
}


const MSPhaseDefinition&
MSSimpleTrafficLightLogic::getCurrentPhaseDef() const {
    return *myPhases[myStep];
}


void
MSSimpleTrafficLightLogic::resetLastSwitch(SUMOTime t) {
    myPhases[myStep]->myLastSwitch = t;
}


// ------------ Conversion between time and phase
SUMOTime
MSSimpleTrafficLightLogic::getPhaseIndexAtTime(SUMOTime simStep) const {
    SUMOTime position = 0;
    if (myStep > 0) {
        for (int i = 0; i < myStep; i++) {
            position = position + getPhase(i).duration;
        }
    }
    position = position + simStep - getPhase(myStep).myLastSwitch;
    position = position % myDefaultCycleTime;
    assert(position <= myDefaultCycleTime);
    return position;
}


SUMOTime
MSSimpleTrafficLightLogic::getOffsetFromIndex(int index) const {
    assert(index < (int)myPhases.size());
    if (index == 0) {
        return 0;
    }
    SUMOTime pos = 0;
    for (int i = 0; i < index; i++) {
        pos += getPhase(i).duration;
    }
    return pos;
}


int
MSSimpleTrafficLightLogic::getIndexFromOffset(SUMOTime offset) const {
    offset = offset % myDefaultCycleTime;
    if (offset == myDefaultCycleTime) {
        return 0;
    }
    SUMOTime testPos = 0;
    for (int i = 0; i < (int)myPhases.size(); i++) {
        testPos = testPos + getPhase(i).duration;
        if (testPos > offset) {
            return i;
        }
        if (testPos == offset) {
            assert((int)myPhases.size() > (i + 1));
            return (i + 1);
        }
    }
    return 0;
}


SUMOTime
MSSimpleTrafficLightLogic::mapTimeInCycle(SUMOTime t) const {
    return (myCoordinated
            ? (t - myOffset) % myDefaultCycleTime
            : (t - myPhases[0]->myLastSwitch) % myDefaultCycleTime);
}




SUMOTime
MSSimpleTrafficLightLogic::getEarliest(SUMOTime prevStart) const {
    SUMOTime earliest = getEarliestEnd();
    if (earliest == MSPhaseDefinition::UNSPECIFIED_DURATION) {
        return 0;
    } else {
        if (prevStart >= SIMSTEP - getTimeInCycle() && prevStart < getCurrentPhaseDef().myLastEnd) {
            // phase was started and ended once already in the current cycle
            // it should not end a second time in the same cycle
            earliest += myDefaultCycleTime;
#ifdef DEBUG_COORDINATION
            if (DEBUG_COND) {
                std::cout << SIMTIME << " tl=" << getID() << " getEarliest phase=" << myStep
                          << " prevStart= " << STEPS2TIME(prevStart)
                          << " prevEnd= " << STEPS2TIME(getCurrentPhaseDef().myLastEnd)
                          << " cycleStart=" << STEPS2TIME(SIMSTEP - getTimeInCycle()) << " started Twice - move into next cycle\n";
            }
#endif
        } else {
            SUMOTime latest = getLatestEnd();
            if (latest != MSPhaseDefinition::UNSPECIFIED_DURATION) {
                const SUMOTime minRemaining = getMinDur() - (SIMSTEP - getCurrentPhaseDef().myLastSwitch);
                const SUMOTime minEnd = getTimeInCycle() + minRemaining;
                if (latest > earliest && latest < minEnd) {
                    // cannot terminate phase between earliest and latest -> move end into next cycle
                    earliest += myDefaultCycleTime;
                } else if (latest < earliest && latest >= minEnd) {
                    // can ignore earliest since it counts from the previous cycle
                    earliest -= myDefaultCycleTime;
                }
#ifdef DEBUG_COORDINATION
                if (DEBUG_COND) {
                    std::cout << SIMTIME << " tl=" << getID() << " getEarliest phase=" << myStep << " latest=" << STEPS2TIME(latest) << " minEnd="
                              << STEPS2TIME(minEnd) << " earliest=" << STEPS2TIME(earliest) << "\n";
                }
#endif
            }
        }
        const SUMOTime maxRemaining = getMaxDur() - (SIMSTEP - getCurrentPhaseDef().myLastSwitch);
        return MIN2(earliest - getTimeInCycle(), maxRemaining);
    }
}


SUMOTime
MSSimpleTrafficLightLogic::getLatest() const {
    const SUMOTime latest = getLatestEnd();
    if (latest == MSPhaseDefinition::UNSPECIFIED_DURATION) {
        return SUMOTime_MAX; // no restriction
    } else {
        if (latest < getEarliestEnd()) {
            const SUMOTime running = SIMSTEP - getCurrentPhaseDef().myLastSwitch;
            if (running < getTimeInCycle()) {
                // phase was started in the current cycle so the restriction does not apply yet
                return SUMOTime_MAX;
            }
        }
#ifdef DEBUG_COORDINATION
        if (DEBUG_COND) {
            std::cout << SIMTIME << " tl=" << getID() << " getLatest phase=" << myStep << " latest=" << STEPS2TIME(latest)
                      << " cycTime=" << STEPS2TIME(getTimeInCycle()) << " res=" << STEPS2TIME(latest - getTimeInCycle()) << "\n";
        }
#endif
        if (latest == myDefaultCycleTime && getTimeInCycle() == 0) {
            // special case: end on cylce time wrap-around
            return 0;
        }
        return MAX2(SUMOTime(0), latest - getTimeInCycle());
    }
}



// ------------ Changing phases and phase durations
void
MSSimpleTrafficLightLogic::changeStepAndDuration(MSTLLogicControl& tlcontrol,
        SUMOTime simStep, int step, SUMOTime stepDuration) {
    mySwitchCommand->deschedule(this);
    mySwitchCommand = new SwitchCommand(tlcontrol, this, stepDuration + simStep);
    if (step >= 0 && step != myStep) {
        myStep = step;
        myPhases[myStep]->myLastSwitch = MSNet::getInstance()->getCurrentTimeStep();
        if (myAmActive) {
            // when loading from state, the last loaded program isn't always the active one
            setTrafficLightSignals(simStep);
        }
        tlcontrol.get(getID()).executeOnSwitchActions();
    }
    MSNet::getInstance()->getBeginOfTimestepEvents()->addEvent(
        mySwitchCommand, stepDuration + simStep);
}


void
MSSimpleTrafficLightLogic::setPhases(const Phases& phases, int step) {
    assert(step < (int)phases.size());
    SUMOTime lastSwitch = myPhases[myStep]->getState() == phases[step]->getState() ? myPhases[myStep]->myLastSwitch : SIMSTEP;
    deletePhases();
    myPhases = phases;
    myStep = step;
    myDefaultCycleTime = computeCycleTime(myPhases);
    myPhases[myStep]->myLastSwitch = lastSwitch;
}


void
MSSimpleTrafficLightLogic::deletePhases() {
    for (int i = 0; i < (int)myPhases.size(); i++) {
        delete myPhases[i];
    }
}

void
MSSimpleTrafficLightLogic::saveState(OutputDevice& out) const {
    out.openTag(SUMO_TAG_TLLOGIC);
    out.writeAttr(SUMO_ATTR_ID, getID());
    out.writeAttr(SUMO_ATTR_PROGRAMID, getProgramID());
    out.writeAttr(SUMO_ATTR_PHASE, getCurrentPhaseIndex());
    out.writeAttr(SUMO_ATTR_DURATION, getSpentDuration());
    out.writeAttr(SUMO_ATTR_ACTIVE, myAmActive);
    out.closeTag();
}

const std::string
MSSimpleTrafficLightLogic::getParameter(const std::string& key, const std::string defaultValue) const {
    if (key == "cycleTime") {
        return toString(STEPS2TIME(myDefaultCycleTime));
    } else if (key == "offset") {
        return toString(STEPS2TIME(myOffset));
    } else if (key == "coordinated") {
        return toString(myCoordinated);
    } else if (key == "cycleSecond") {
        return toString(STEPS2TIME(getTimeInCycle()));
    } else if (key == "typeName") {
        return toString(this->getLogicType());
    }
    return Parameterised::getParameter(key, defaultValue);
}

void
MSSimpleTrafficLightLogic::setParameter(const std::string& key, const std::string& value) {
    if (key == "cycleTime") {
        myDefaultCycleTime = string2time(value);
        Parameterised::setParameter(key, value);
    } else if (key == "cycleSecond" || key == "typeName") {
        throw InvalidArgument(key + " cannot be changed dynamically for traffic light '" + getID() + "'");
    } else if (key == "offset") {
        myOffset = string2time(value);
    } else if (key == "coordinated") {
        myCoordinated = StringUtils::toBool(value);
        Parameterised::setParameter(key, value);
    } else {
        Parameterised::setParameter(key, value);
    }
}

/****************************************************************************/
