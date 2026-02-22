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
/// @file    MSOffTrafficLightLogic.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    08.05.2007
///
// A traffic lights logic which represents a tls in an off-mode
/****************************************************************************/
#include <config.h>

#include <utility>
#include <vector>
#include <bitset>
#include <sstream>
#include <microsim/MSEventControl.h>
#include "MSTrafficLightLogic.h"
#include "MSOffTrafficLightLogic.h"


// ===========================================================================
// member method definitions
// ===========================================================================
MSOffTrafficLightLogic::MSOffTrafficLightLogic(MSTLLogicControl& tlcontrol, const std::string& id) :
    MSTrafficLightLogic(tlcontrol, id, "off", 0, TrafficLightType::OFF, 0, Parameterised::Map()) {
    myDefaultCycleTime = TIME2STEPS(120);
}


MSOffTrafficLightLogic::~MSOffTrafficLightLogic() {
    for (MSTrafficLightLogic::Phases::const_iterator i = myPhaseDefinition.begin(); i != myPhaseDefinition.end(); ++i) {
        delete *i;
    }
}


void
MSOffTrafficLightLogic::init(NLDetectorBuilder&) {
    rebuildPhase();
}


// ----------- Handling of controlled links
void
MSOffTrafficLightLogic::adaptLinkInformationFrom(const MSTrafficLightLogic& logic) {
    MSTrafficLightLogic::adaptLinkInformationFrom(logic);
    rebuildPhase();
}


void
MSOffTrafficLightLogic::rebuildPhase() {
    int no = (int)getLinks().size();
    std::string state;
    for (int i = 0; i < no; ++i) {
        bool foundMajor = false;
        bool foundMinor = false;
        bool foundAllwayStop = false;
        for (const MSLink* l : myLinks[i]) {
            /// @note. all links for the same index should have the same
            if (l->getOffState() == LINKSTATE_TL_OFF_BLINKING) {
                foundMinor = true;
            } else if (l->getOffState() == LINKSTATE_TL_OFF_NOSIGNAL) {
                foundMajor = true;
            } else if (l->getOffState() == LINKSTATE_ALLWAY_STOP) {
                foundAllwayStop = true;
            } else {
                WRITE_WARNINGF(TL("Invalid 'off'-state for link % at junction '%'"), toString(l->getIndex()), l->getJunction()->getID());
            }
        }
        if (foundMajor && foundMinor) {
            WRITE_WARNINGF(TL("Inconsistent 'off'-states for linkIndex % at tlLogic '%'"), toString(i), getID());
        }
        state += toString(foundAllwayStop ? LINKSTATE_ALLWAY_STOP : (foundMinor ? LINKSTATE_TL_OFF_BLINKING : LINKSTATE_TL_OFF_NOSIGNAL));
    }
    for (MSTrafficLightLogic::Phases::const_iterator i = myPhaseDefinition.begin(); i != myPhaseDefinition.end(); ++i) {
        delete *i;
    }
    myPhaseDefinition.clear();
    myPhaseDefinition.push_back(new MSPhaseDefinition(TIME2STEPS(120), state));
}


// ------------ Static Information Retrieval
int
MSOffTrafficLightLogic::getPhaseNumber() const {
    return 0;
}


const MSOffTrafficLightLogic::Phases&
MSOffTrafficLightLogic::getPhases() const {
    return myPhaseDefinition;
}


const MSPhaseDefinition&
MSOffTrafficLightLogic::getPhase(int) const {
    return *myPhaseDefinition[0];
}


// ------------ Dynamic Information Retrieval
int
MSOffTrafficLightLogic::getCurrentPhaseIndex() const {
    return 0;
}


const MSPhaseDefinition&
MSOffTrafficLightLogic::getCurrentPhaseDef() const {
    return *myPhaseDefinition[0];
}


void
MSOffTrafficLightLogic::resetLastSwitch(SUMOTime t) {
    myPhaseDefinition[0]->myLastSwitch = t;
}


// ------------ Conversion between time and phase
SUMOTime
MSOffTrafficLightLogic::getPhaseIndexAtTime(SUMOTime) const {
    return 0;
}


SUMOTime
MSOffTrafficLightLogic::getOffsetFromIndex(int) const {
    return 0;
}


int
MSOffTrafficLightLogic::getIndexFromOffset(SUMOTime) const {
    return 0;
}


/****************************************************************************/
