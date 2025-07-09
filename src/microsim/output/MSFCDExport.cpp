/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2012-2025 German Aerospace Center (DLR) and others.
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
/// @file    MSFCDExport.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Mario Krumnow
/// @author  Michael Behrisch
/// @date    2012-04-26
///
// Realises dumping Floating Car Data (FCD) Data
/****************************************************************************/
#include <config.h>

#include <utils/iodevices/OutputDevice.h>
#include <utils/options/OptionsCont.h>
#include <utils/geom/GeoConvHelper.h>
#include <utils/geom/GeomHelper.h>
#include <utils/shapes/SUMOPolygon.h>
#include <libsumo/Helper.h>
#include <microsim/devices/MSDevice_FCD.h>
#include <microsim/devices/MSTransportableDevice_FCD.h>
#include <microsim/MSEdgeControl.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSNet.h>
#include <microsim/MSVehicle.h>
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <microsim/transportables/MSPerson.h>
#include <microsim/transportables/MSTransportableControl.h>
#include <microsim/MSVehicleControl.h>
#include <mesosim/MEVehicle.h>
#include "MSEmissionExport.h"
#include "MSFCDExport.h"


// ===========================================================================
// method definitions
// ===========================================================================
void
MSFCDExport::write(OutputDevice& of, const SUMOTime timestep) {
    MSDevice_FCD::initOnce();
    const SUMOTime period = MSDevice_FCD::getPeriod();
    const SUMOTime begin = MSDevice_FCD::getBegin();
    if ((period > 0 && (timestep - begin) % period != 0) || timestep < begin) {
        return;
    }
    const SumoXMLAttrMask& mask = MSDevice_FCD::getWrittenAttributes();
    const bool useGeo = MSDevice_FCD::useGeo();
    const double maxLeaderDistance = MSDevice_FCD::getMaxLeaderDistance();
    const std::vector<std::string>& params = MSDevice_FCD::getParamsToWrite();
    MSNet* net = MSNet::getInstance();
    MSVehicleControl& vc = net->getVehicleControl();
    const double radius = MSDevice_FCD::getRadius();
    const bool filter = MSDevice_FCD::getEdgeFilter().size() > 0;
    const bool shapeFilter = MSDevice_FCD::hasShapeFilter();
    std::set<const Named*> inRadius;
    if (radius > 0) {
        // collect all vehicles in radius around equipped vehicles
        for (MSVehicleControl::constVehIt it = vc.loadedVehBegin(); it != vc.loadedVehEnd(); ++it) {
            const SUMOVehicle* veh = it->second;
            if (isVisible(veh) && hasOwnOutput(veh, filter, shapeFilter)) {
                PositionVector shape;
                shape.push_back(veh->getPosition());
                libsumo::Helper::collectObjectsInRange(libsumo::CMD_GET_VEHICLE_VARIABLE, shape, radius, inRadius);
                libsumo::Helper::collectObjectsInRange(libsumo::CMD_GET_PERSON_VARIABLE, shape, radius, inRadius);
            }
        }
    }

    of.openTag("timestep").writeTime(SUMO_ATTR_TIME, timestep);
    for (MSVehicleControl::constVehIt it = vc.loadedVehBegin(); it != vc.loadedVehEnd(); ++it) {
        const SUMOVehicle* const veh = it->second;
        if (isVisible(veh)) {
            const bool hasOutput = hasOwnOutput(veh, filter, shapeFilter, (radius > 0 && inRadius.count(veh) > 0));
            if (hasOutput) {
                const MSVehicle* const microVeh = MSGlobals::gUseMesoSim ? nullptr : static_cast<const MSVehicle*>(veh);
                Position pos = veh->getPosition();
                if (useGeo) {
                    of.setPrecision(gPrecisionGeo);
                    GeoConvHelper::getFinal().cartesian2geo(pos);
                }
                of.openTag(SUMO_TAG_VEHICLE);
                of.writeAttr(SUMO_ATTR_ID, veh->getID());
                of.writeOptionalAttr(SUMO_ATTR_X, pos.x(), mask);
                of.writeOptionalAttr(SUMO_ATTR_Y, pos.y(), mask);
                of.setPrecision(gPrecision);
                of.writeOptionalAttr(SUMO_ATTR_Z, pos.z(), mask);
                of.writeFuncAttr(SUMO_ATTR_ANGLE, [ = ]() {
                    return GeomHelper::naviDegree(veh->getAngle());
                }, mask);
                of.writeFuncAttr(SUMO_ATTR_TYPE, [ = ]() {
                    return veh->getVehicleType().getID();
                }, mask);
                of.writeFuncAttr(SUMO_ATTR_SPEED, [ = ]() {
                    return veh->getSpeed();
                }, mask);
                of.writeFuncAttr(SUMO_ATTR_POSITION, [ = ]() {
                    return veh->getPositionOnLane();
                }, mask);
                of.writeFuncAttr(SUMO_ATTR_LANE, [ = ]() {
                    return MSGlobals::gUseMesoSim ? "" : microVeh->getLane()->getID();
                }, mask, MSGlobals::gUseMesoSim);
                of.writeFuncAttr(SUMO_ATTR_EDGE, [ = ]() {
                    return veh->getEdge()->getID();
                }, mask, !MSGlobals::gUseMesoSim);
                of.writeFuncAttr(SUMO_ATTR_SLOPE, [ = ]() {
                    return veh->getSlope();
                }, mask);
                if (!MSGlobals::gUseMesoSim) {
                    of.writeFuncAttr(SUMO_ATTR_SIGNALS, [ = ]() {
                        return microVeh->getSignals();
                    }, mask);
                    of.writeFuncAttr(SUMO_ATTR_ACCELERATION, [ = ]() {
                        return microVeh->getAcceleration();
                    }, mask);
                    of.writeFuncAttr(SUMO_ATTR_ACCELERATION_LAT, [ = ]() {
                        return microVeh->getLaneChangeModel().getAccelerationLat();
                    }, mask);
                }
                of.writeFuncAttr(SUMO_ATTR_DISTANCE, [ = ]() {
                    double lanePos = veh->getPositionOnLane();
                    if (!MSGlobals::gUseMesoSim && microVeh->getLane()->isInternal()) {
                        lanePos = microVeh->getRoute().getDistanceBetween(0., lanePos, microVeh->getEdge()->getLanes()[0], microVeh->getLane(),
                                  microVeh->getRoutePosition());
                    }
                    return veh->getEdge()->getDistanceAt(lanePos);
                }, mask);
                of.writeFuncAttr(SUMO_ATTR_ODOMETER, [ = ]() {
                    return veh->getOdometer();
                }, mask);
                of.writeFuncAttr(SUMO_ATTR_POSITION_LAT, [ = ]() {
                    return veh->getLateralPositionOnLane();
                }, mask);
                if (!MSGlobals::gUseMesoSim) {
                    of.writeFuncAttr(SUMO_ATTR_SPEED_LAT, [ = ]() {
                        return microVeh->getLaneChangeModel().getSpeedLat();
                    }, mask);
                }
                if (maxLeaderDistance >= 0 && !MSGlobals::gUseMesoSim) {
                    const std::pair<const MSVehicle* const, double> leader = microVeh->getLeader(maxLeaderDistance);
                    if (leader.first != nullptr) {
                        of.writeFuncAttr(SUMO_ATTR_LEADER_ID, [ = ]() {
                            return leader.first->getID();
                        }, mask);
                        of.writeFuncAttr(SUMO_ATTR_LEADER_SPEED, [ = ]() {
                            return leader.first->getSpeed();
                        }, mask);
                        of.writeFuncAttr(SUMO_ATTR_LEADER_GAP, [ = ]() {
                            return leader.second + microVeh->getVehicleType().getMinGap();
                        }, mask);
                    } else {
                        of.writeFuncAttr(SUMO_ATTR_LEADER_ID, [ = ]() {
                            return "";
                        }, mask);
                        of.writeFuncAttr(SUMO_ATTR_LEADER_SPEED, [ = ]() {
                            return -1;
                        }, mask);
                        of.writeFuncAttr(SUMO_ATTR_LEADER_GAP, [ = ]() {
                            return -1;
                        }, mask);
                    }
                }
                for (const std::string& key : params) {
                    std::string error;
                    const std::string value = static_cast<const MSBaseVehicle*>(veh)->getPrefixedParameter(key, error);
                    if (value != "") {
                        of.writeAttr(StringUtils::escapeXML(key), StringUtils::escapeXML(value));
                    }
                }
                of.writeFuncAttr(SUMO_ATTR_ARRIVALDELAY, [ = ]() {
                    const double arrivalDelay = static_cast<const MSBaseVehicle*>(veh)->getStopArrivalDelay();
                    if (arrivalDelay == INVALID_DOUBLE) {
                        // no upcoming stop also means that there is no delay
                        return 0.;
                    }
                    return arrivalDelay;
                }, mask);
                if (MSGlobals::gUseMesoSim) {
                    const MEVehicle* mesoVeh = static_cast<const MEVehicle*>(veh);
                    of.writeFuncAttr(SUMO_ATTR_SEGMENT, [ = ]() {
                        return mesoVeh->getSegmentIndex();
                    }, mask);
                    of.writeFuncAttr(SUMO_ATTR_QUEUE, [ = ]() {
                        return mesoVeh->getQueIndex();
                    }, mask);
                    of.writeFuncAttr(SUMO_ATTR_ENTRYTIME, [ = ]() {
                        return mesoVeh->getLastEntryTimeSeconds();
                    }, mask);
                    of.writeFuncAttr(SUMO_ATTR_EVENTTIME, [ = ]() {
                        return mesoVeh->getEventTimeSeconds();
                    }, mask);
                    of.writeFuncAttr(SUMO_ATTR_BLOCKTIME, [ = ]() {
                        return mesoVeh->getBlockTime() == SUMOTime_MAX ? -1.0 : mesoVeh->getBlockTimeSeconds();
                    }, mask);
                }
                of.writeFuncAttr(SUMO_ATTR_TAG, [ = ]() {
                    return toString(SUMO_TAG_VEHICLE);
                }, mask);
                MSEmissionExport::writeEmissions(of, false, static_cast<const MSBaseVehicle*>(veh), false, mask);
                of.closeTag();
            }
            // write persons and containers in the vehicle
            const MSEdge* edge = MSGlobals::gUseMesoSim ? veh->getEdge() : &veh->getLane()->getEdge();
            for (const MSTransportable* const person : veh->getPersons()) {
                writeTransportable(of, edge, person, veh, filter, shapeFilter, inRadius.count(person) > 0, SUMO_TAG_PERSON, useGeo, mask);
            }
            for (const MSTransportable* const container : veh->getContainers()) {
                writeTransportable(of, edge, container, veh, filter, shapeFilter, inRadius.count(container) > 0, SUMO_TAG_CONTAINER, useGeo, mask);
            }
        }
    }
    if (net->hasPersons() && net->getPersonControl().hasTransportables()) {
        // write persons who are not in a vehicle
        for (const MSEdge* const e : net->getEdgeControl().getEdges()) {
            if (filter && MSDevice_FCD::getEdgeFilter().count(e) == 0) {
                continue;
            }
            for (const MSTransportable* const person : e->getSortedPersons(timestep)) {
                writeTransportable(of, e, person, nullptr, filter, shapeFilter, inRadius.count(person) > 0, SUMO_TAG_PERSON, useGeo, mask);
            }
        }
    }
    if (net->hasContainers() && net->getContainerControl().hasTransportables()) {
        // write containers which are not in a vehicle
        for (const MSEdge* const e : net->getEdgeControl().getEdges()) {
            if (filter && MSDevice_FCD::getEdgeFilter().count(e) == 0) {
                continue;
            }
            for (MSTransportable* container : e->getSortedContainers(timestep)) {
                writeTransportable(of, e, container, nullptr, filter, shapeFilter, inRadius.count(container) > 0, SUMO_TAG_CONTAINER, useGeo, mask);
            }
        }
    }
    of.closeTag();
}


bool
MSFCDExport::isVisible(const SUMOVehicle* veh) {
    return veh->isOnRoad() || veh->isParking() || veh->isRemoteControlled();
}


bool
MSFCDExport::hasOwnOutput(const SUMOVehicle* veh, bool filter, bool shapeFilter, bool isInRadius) {
    return ((!filter || MSDevice_FCD::getEdgeFilter().count(veh->getEdge()) > 0)
            && (!shapeFilter || MSDevice_FCD::shapeFilter(veh))
            && ((veh->getDevice(typeid(MSDevice_FCD)) != nullptr) || isInRadius));
}


bool
MSFCDExport::hasOwnOutput(const MSTransportable* p, bool filter, bool shapeFilter, bool isInRadius) {
    return ((!filter || MSDevice_FCD::getEdgeFilter().count(p->getEdge()) > 0)
            && (!shapeFilter || MSDevice_FCD::shapeFilter(p))
            && ((p->getDevice(typeid(MSTransportableDevice_FCD)) != nullptr) || isInRadius));
}


void
MSFCDExport::writeTransportable(OutputDevice& of, const MSEdge* const e, const MSTransportable* const p, const SUMOVehicle* const v,
                                const bool filter, const bool shapeFilter, const bool inRadius,
                                const SumoXMLTag tag, const bool useGeo, const SumoXMLAttrMask mask) {
    if (!hasOwnOutput(p, filter, shapeFilter, inRadius)) {
        return;
    }
    Position pos = p->getPosition();
    if (useGeo) {
        of.setPrecision(gPrecisionGeo);
        GeoConvHelper::getFinal().cartesian2geo(pos);
    }
    of.openTag(tag);
    of.writeAttr(SUMO_ATTR_ID, p->getID());
    of.writeOptionalAttr(SUMO_ATTR_X, pos.x(), mask);
    of.writeOptionalAttr(SUMO_ATTR_Y, pos.y(), mask);
    of.setPrecision(gPrecision);
    of.writeOptionalAttr(SUMO_ATTR_Z, pos.z(), mask);
    of.writeOptionalAttr(SUMO_ATTR_ANGLE, GeomHelper::naviDegree(p->getAngle()), mask);
    of.writeOptionalAttr(SUMO_ATTR_TYPE, "", mask, true);
    of.writeOptionalAttr(SUMO_ATTR_SPEED, p->getSpeed(), mask);
    of.writeOptionalAttr(SUMO_ATTR_POSITION, p->getEdgePos(), mask);
    of.writeOptionalAttr(SUMO_ATTR_LANE, "", mask, true);
    of.writeOptionalAttr(SUMO_ATTR_EDGE, e->getID(), mask);
    of.writeOptionalAttr(SUMO_ATTR_SLOPE, e->getLanes()[0]->getShape().slopeDegreeAtOffset(p->getEdgePos()), mask);
    of.writeOptionalAttr(SUMO_ATTR_VEHICLE, v == nullptr ? "" : v->getID(), mask);
    of.writeOptionalAttr(SUMO_ATTR_TYPE, p->getVehicleType().getID(), mask);
    of.writeOptionalAttr(SUMO_ATTR_TAG, toString(tag), mask);
    of.closeTag();
}


/****************************************************************************/
