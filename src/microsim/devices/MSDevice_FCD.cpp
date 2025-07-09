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
/// @file    MSDevice_FCD.cpp
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @author  Jakob Erdmann
/// @date    11.06.2013
///
// A device which stands as an implementation FCD and which outputs movereminder calls
/****************************************************************************/
#include <config.h>

#include <bitset>
#include <utils/common/StringUtils.h>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/vehicle/SUMOVehicle.h>
#include <utils/shapes/ShapeContainer.h>
#include <utils/shapes/SUMOPolygon.h>
#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSVehicle.h>
#include "MSDevice_FCD.h"


// ===========================================================================
// static members
// ===========================================================================
SUMOTime MSDevice_FCD::myBegin = SUMOTime_MAX;
SUMOTime MSDevice_FCD::myPeriod = 0;
bool MSDevice_FCD::myUseGeo;
double MSDevice_FCD::myMaxLeaderDistance;
std::vector<std::string> MSDevice_FCD::myParamsToWrite;
double MSDevice_FCD::myRadius;
std::set<const MSEdge*> MSDevice_FCD::myEdgeFilter;
std::vector<PositionVector> MSDevice_FCD::myShape4Filters;
bool MSDevice_FCD::myEdgeFilterInitialized(false);
bool MSDevice_FCD::myShapeFilterInitialized(false);
bool MSDevice_FCD::myShapeFilterDesired(false);
SumoXMLAttrMask MSDevice_FCD::myWrittenAttributes;


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_FCD::insertOptions(OptionsCont& oc) {
    oc.addOptionSubTopic("FCD Device");
    insertDefaultAssignmentOptions("fcd", "FCD Device", oc);

    oc.doRegister("device.fcd.begin", new Option_String("-1"));
    oc.addDescription("device.fcd.begin", "FCD Device", TL("Recording begin time for FCD-data"));

    oc.doRegister("device.fcd.period", new Option_String("0"));
    oc.addDescription("device.fcd.period", "FCD Device", TL("Recording period for FCD-data"));

    oc.doRegister("device.fcd.radius", new Option_Float(0));
    oc.addDescription("device.fcd.radius", "FCD Device", TL("Record objects in a radius around equipped vehicles"));
}


void
MSDevice_FCD::buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into) {
    OptionsCont& oc = OptionsCont::getOptions();
    if (equippedByDefaultAssignmentOptions(oc, "fcd", v, oc.isSet("fcd-output"))) {
        MSDevice_FCD* device = new MSDevice_FCD(v, "fcd_" + v.getID());
        into.push_back(device);
    }
}


// ---------------------------------------------------------------------------
// MSDevice_FCD-methods
// ---------------------------------------------------------------------------
MSDevice_FCD::MSDevice_FCD(SUMOVehicle& holder, const std::string& id) :
    MSVehicleDevice(holder, id) {
}


MSDevice_FCD::~MSDevice_FCD() {
}


SumoXMLAttrMask
MSDevice_FCD::getDefaultMask() {
    SumoXMLAttrMask mask;
    mask.set(SUMO_ATTR_X);
    mask.set(SUMO_ATTR_Y);
    if (MSNet::getInstance()->hasElevation()) {
        mask.set(SUMO_ATTR_Z);
    }
    mask.set(SUMO_ATTR_ANGLE);
    mask.set(SUMO_ATTR_TYPE);
    mask.set(SUMO_ATTR_SPEED);
    mask.set(SUMO_ATTR_POSITION);
    mask.set(SUMO_ATTR_LANE); // for micro vehicles only
    mask.set(SUMO_ATTR_EDGE); // for persons and meso vehicles
    mask.set(SUMO_ATTR_SLOPE);
    if (!MSGlobals::gUseMesoSim && OptionsCont::getOptions().getFloat("fcd-output.max-leader-distance") > 0.) {
        mask.set(SUMO_ATTR_LEADER_ID);
        mask.set(SUMO_ATTR_LEADER_SPEED);
        mask.set(SUMO_ATTR_LEADER_GAP);
    }
    return mask;
}


bool
MSDevice_FCD::shapeFilter(const SUMOTrafficObject* veh) {
    // lazily build the shape filter in the case where route file is loaded as an additional file
    if (!myShapeFilterInitialized) {
        buildShapeFilter();
    }
    const MSVehicle* msVeh = dynamic_cast<const MSVehicle*>(veh);
    for (auto shape : myShape4Filters) {
        if (shape.around(veh->getPosition()) || ((msVeh != nullptr) && shape.around(msVeh->getBackPosition()))) {
            return true;
        }
    }
    return false;
}


void
MSDevice_FCD::buildShapeFilter(void) {
    const OptionsCont& oc = OptionsCont::getOptions();
    if (oc.isSet("fcd-output.filter-shapes")) {
        const ShapeContainer& loadedShapes = MSNet::getInstance()->getShapeContainer();
        if (loadedShapes.getPolygons().size() > 0) {
            for (std::string attrName : oc.getStringVector("fcd-output.filter-shapes")) {
                if (loadedShapes.getPolygons().get(attrName) == 0) {
                    WRITE_ERRORF(TL("Specified shape '%' for filtering fcd-output could not be found."), attrName);
                } else {
                    // store the PositionVector, not reference, as traci can manipulate / detete the polygons
                    myShape4Filters.push_back(loadedShapes.getPolygons().get(attrName)->getShape());
                }
            }
            myShapeFilterInitialized = true;
        }
    } else {
        myShapeFilterInitialized = true;
    }
}


void
MSDevice_FCD::initOnce() {
    if (myEdgeFilterInitialized) {
        return;
    }
    myEdgeFilterInitialized = true;
    const OptionsCont& oc = OptionsCont::getOptions();
    myPeriod = string2time(oc.getString("device.fcd.period"));
    myBegin = string2time(oc.getString("device.fcd.begin"));
    myUseGeo = oc.getBool("fcd-output.geo");
    myMaxLeaderDistance = oc.getFloat("fcd-output.max-leader-distance");
    myParamsToWrite = oc.getStringVector("fcd-output.params");
    myRadius = oc.getFloat("device.fcd.radius");
    if (oc.isSet("fcd-output.filter-edges.input-file")) {
        const std::string file = oc.getString("fcd-output.filter-edges.input-file");
        std::ifstream strm(file.c_str());
        if (!strm.good()) {
            throw ProcessError(TLF("Could not load names of edges for filtering fcd-output from '%'.", file));
        }
        while (strm.good()) {
            std::string name;
            strm >> name;
            // maybe we're loading an edge-selection
            if (StringUtils::startsWith(name, "edge:")) {
                name = name.substr(5);
            }
            myEdgeFilter.insert(MSEdge::dictionary(name));
        }
    }
    const SumoXMLAttrMask traffic = getDefaultMask();
    SumoXMLAttrMask emissions;
    emissions.set(SUMO_ATTR_ECLASS);
    emissions.set(SUMO_ATTR_CO2);
    emissions.set(SUMO_ATTR_CO);
    emissions.set(SUMO_ATTR_HC);
    emissions.set(SUMO_ATTR_PMX);
    emissions.set(SUMO_ATTR_FUEL);
    emissions.set(SUMO_ATTR_ELECTRICITY);
    SumoXMLAttrMask misc;
    misc.set(SUMO_ATTR_SIGNALS);
    misc.set(SUMO_ATTR_ACCELERATION);
    misc.set(SUMO_ATTR_ACCELERATION_LAT);
    misc.set(SUMO_ATTR_DISTANCE);
    misc.set(SUMO_ATTR_ODOMETER);
    misc.set(SUMO_ATTR_POSITION_LAT);
    misc.set(SUMO_ATTR_SPEED_LAT);
    misc.set(SUMO_ATTR_LEADER_ID);
    misc.set(SUMO_ATTR_LEADER_SPEED);
    misc.set(SUMO_ATTR_ARRIVALDELAY);
    misc.set(SUMO_ATTR_SEGMENT);
    misc.set(SUMO_ATTR_QUEUE);
    misc.set(SUMO_ATTR_ENTRYTIME);
    misc.set(SUMO_ATTR_EVENTTIME);
    misc.set(SUMO_ATTR_BLOCKTIME);
    const std::map<std::string, SumoXMLAttrMask> special = {{"traffic", traffic}, {"emissions", emissions}, {"misc", misc}};
    if (oc.isSet("fcd-output.attributes")) {
        myWrittenAttributes = OutputDevice::parseWrittenAttributes(oc.getStringVector("fcd-output.attributes"), "fcd output", special);
    } else {
        myWrittenAttributes = getDefaultMask();
    }
    myWrittenAttributes.set(SUMO_ATTR_ID);
    if (!MSNet::getInstance()->hasElevation()) {
        myWrittenAttributes.reset(SUMO_ATTR_Z);
    }
    if (oc.getBool("fcd-output.signals")) {
        myWrittenAttributes.set(SUMO_ATTR_SIGNALS);
    }
    if (oc.getBool("fcd-output.acceleration")) {
        myWrittenAttributes.set(SUMO_ATTR_ACCELERATION);
    }
    myWrittenAttributes.set(SUMO_ATTR_ACCELERATION_LAT, myWrittenAttributes.test(SUMO_ATTR_ACCELERATION) && MSGlobals::gSublane);
    if (oc.getBool("fcd-output.distance")) {
        myWrittenAttributes.set(SUMO_ATTR_DISTANCE);
    }

    if (oc.isSet("fcd-output.filter-shapes")) {
        // build the shape filter if it is desired
        myShapeFilterDesired = true;
        buildShapeFilter();
    }
    OutputDevice& of = OutputDevice::getDeviceByOption("fcd-output");
    of.setExpectedAttributes(myWrittenAttributes);
}


void
MSDevice_FCD::cleanup() {
    myEdgeFilter.clear();
    myShape4Filters.clear();
    myEdgeFilterInitialized = false;
    myShapeFilterInitialized = false;
    myShapeFilterDesired = false;
    myWrittenAttributes.reset();
}


/****************************************************************************/
