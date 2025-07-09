/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2013-2025 German Aerospace Center (DLR) and others.
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
/// @file    MSDevice.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    14.08.2013
///
// Abstract in-vehicle device
/****************************************************************************/
#include <config.h>

#include <utils/options/OptionsCont.h>
#include <utils/common/StringUtils.h>
#include <microsim/MSVehicle.h>
#include <microsim/transportables/MSTransportable.h>
#include <microsim/MSVehicleControl.h>
#include <microsim/MSEdge.h>

#include "MSDevice_Vehroutes.h"
#include "MSDevice_Tripinfo.h"
#include "MSDevice_Routing.h"
#include "MSDevice_Emissions.h"
#include "MSDevice_BTreceiver.h"
#include "MSDevice_BTsender.h"
#include "MSDevice_Example.h"
#include "MSDevice_StationFinder.h"
#include "MSDevice_Battery.h"
#include "MSDevice_SSM.h"
#include "MSDevice_ToC.h"
#include "MSDevice_DriverState.h"
#include "MSDevice_Bluelight.h"
#include "MSDevice_FCD.h"
#include "MSDevice_Taxi.h"
#include "MSDevice_GLOSA.h"
#include "MSDevice_ElecHybrid.h"
#include "MSTransportableDevice_Routing.h"
#include "MSTransportableDevice_FCD.h"
#include "MSTransportableDevice_FCDReplay.h"
#include "MSRoutingEngine.h"
#include "MSDevice_Friction.h"
#include "MSDevice_FCDReplay.h"
#include "MSDevice.h"


// ===========================================================================
// static member variables
// ===========================================================================
const std::string MSDevice::LOADSTATE_DEVICENAMES("LOADSTATE_DEVICENAMES");
std::map<std::string, std::set<std::string> > MSDevice::myExplicitIDs;
SumoRNG MSDevice::myEquipmentRNG("deviceEquipment");


// ===========================================================================
// debug flags
// ===========================================================================
//#define DEBUG_DEVICE_PARAMS


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice::insertOptions(OptionsCont& oc) {
    MSDevice_Routing::insertOptions(oc);
    MSDevice_Emissions::insertOptions(oc);
    MSVehicleDevice_BTreceiver::insertOptions(oc);
    MSVehicleDevice_BTsender::insertOptions(oc);
    MSDevice_Example::insertOptions(oc);
    MSDevice_StationFinder::insertOptions(oc);
    MSDevice_Battery::insertOptions(oc);
    MSDevice_SSM::insertOptions(oc);
    MSDevice_ToC::insertOptions(oc);
    MSDevice_DriverState::insertOptions(oc);
    MSDevice_Bluelight::insertOptions(oc);
    MSDevice_FCD::insertOptions(oc);
    MSDevice_ElecHybrid::insertOptions(oc);
    MSDevice_Taxi::insertOptions(oc);
    MSDevice_GLOSA::insertOptions(oc);
    MSDevice_Tripinfo::insertOptions(oc);
    MSDevice_Vehroutes::insertOptions(oc);
    MSDevice_Friction::insertOptions(oc);
    MSDevice_FCDReplay::insertOptions(oc);

    MSTransportableDevice_Routing::insertOptions(oc);
    MSTransportableDevice_FCD::insertOptions(oc);
    MSTransportableDevice_BTsender::insertOptions(oc);
    MSTransportableDevice_BTreceiver::insertOptions(oc);
}


bool
MSDevice::checkOptions(OptionsCont& oc) {
    bool ok = true;
    ok &= MSDevice_Routing::checkOptions(oc);
    return ok;
}


void
MSDevice::buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into) {
    MSDevice_Vehroutes::buildVehicleDevices(v, into);
    MSDevice_Tripinfo::buildVehicleDevices(v, into);
    MSDevice_Routing::buildVehicleDevices(v, into);
    MSDevice_Emissions::buildVehicleDevices(v, into);
    MSVehicleDevice_BTreceiver::buildVehicleDevices(v, into);
    MSVehicleDevice_BTsender::buildVehicleDevices(v, into);
    MSDevice_Example::buildVehicleDevices(v, into);
    const size_t numBefore = into.size();
    MSDevice_StationFinder::buildVehicleDevices(v, into);
    MSDevice_Battery::buildVehicleDevices(v, into, into.size() == numBefore ? nullptr : static_cast<MSDevice_StationFinder*>(into.back()));
    MSDevice_SSM::buildVehicleDevices(v, into);
    MSDevice_ToC::buildVehicleDevices(v, into);
    MSDevice_DriverState::buildVehicleDevices(v, into);
    MSDevice_Bluelight::buildVehicleDevices(v, into);
    MSDevice_FCD::buildVehicleDevices(v, into);
    MSDevice_ElecHybrid::buildVehicleDevices(v, into);
    MSDevice_Taxi::buildVehicleDevices(v, into);
    MSDevice_GLOSA::buildVehicleDevices(v, into);
    MSDevice_Friction::buildVehicleDevices(v, into);
    MSDevice_FCDReplay::buildVehicleDevices(v, into);
}


void
MSDevice::buildTransportableDevices(MSTransportable& p, std::vector<MSTransportableDevice*>& into) {
    MSTransportableDevice_Routing::buildDevices(p, into);
    MSTransportableDevice_FCD::buildDevices(p, into);
    MSTransportableDevice_BTsender::buildDevices(p, into);
    MSTransportableDevice_BTreceiver::buildDevices(p, into);
    MSTransportableDevice_FCDReplay::buildDevices(p, into);
}


void
MSDevice::cleanupAll() {
    MSRoutingEngine::cleanup();
    MSDevice_Tripinfo::cleanup();
    MSDevice_FCD::cleanup();
    MSDevice_Taxi::cleanup();
}

void
MSDevice::insertDefaultAssignmentOptions(const std::string& deviceName, const std::string& optionsTopic, OptionsCont& oc, const bool isPerson) {
    const std::string prefix = (isPerson ? "person-device." : "device.") + deviceName;
    const std::string object = isPerson ? "person" : "vehicle";
    oc.doRegister(prefix + ".probability", new Option_Float(-1.0));// (default: no need to call RNG)
    oc.addDescription(prefix + ".probability", optionsTopic, "The probability for a " + object + " to have a '" + deviceName + "' device");

    oc.doRegister(prefix + ".explicit", new Option_StringVector());
    oc.addSynonyme(prefix + ".explicit", prefix + ".knownveh", true);
    oc.addDescription(prefix + ".explicit", optionsTopic, "Assign a '" + deviceName + "' device to named " + object + "s");

    oc.doRegister(prefix + ".deterministic", new Option_Bool(false));
    oc.addDescription(prefix + ".deterministic", optionsTopic, "The '" + deviceName + "' devices are set deterministic using a fraction of 1000");
}


std::string
MSDevice::getDeviceName(const std::string& id) {
    if (StringUtils::startsWith(id, "routing_")) {
        // special case: renamed in output
        return "rerouting";
    } else if (StringUtils::startsWith(id, "driverstate")) {
        // special case: no underscore
        return "driverstate";
    } else {
        return id.substr(0, id.find('_'));
    }
}


void
MSDevice::saveState(OutputDevice& /* out */) const {
    WRITE_WARNINGF(TL("Device '%' cannot save state"), getID());
}


void
MSDevice::loadState(const SUMOSAXAttributes& /* attrs */) {
}


/****************************************************************************/
