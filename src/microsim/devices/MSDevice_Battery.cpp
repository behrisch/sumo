/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2013-2024 German Aerospace Center (DLR) and others.
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
/// @file    MSDevice_Battery.cpp
/// @author  Tamas Kurczveil
/// @author  Pablo Alvarez Lopez
/// @author  Mirko Barthauer
/// @date    20.12.2013
///
// The Battery parameters for the vehicle
/****************************************************************************/
#include <algorithm>

#include <config.h>

#include <utils/common/StringUtils.h>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/common/SUMOTime.h>
#include <utils/geom/GeomHelper.h>
#include <utils/emissions/HelpersEnergy.h>
#include <utils/xml/SUMOSAXAttributes.h>
#include <microsim/MSNet.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/MSVehicle.h>
#include "MSDevice_StationFinder.h"
#include "MSDevice_Emissions.h"
#include "MSDevice_Battery.h"

#define DEFAULT_MAX_CAPACITY 35000
#define DEFAULT_CHARGE_RATIO 0.5


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static initialisation methods
// ---------------------------------------------------------------------------
void
MSDevice_Battery::insertOptions(OptionsCont& oc) {
    insertDefaultAssignmentOptions("battery", "Battery", oc);
    // custom options
    oc.doRegister("device.battery.track-fuel", new Option_Bool(false));
    oc.addDescription("device.battery.track-fuel", "Battery", TL("Track fuel consumption for non-electric vehicles"));
}


void
MSDevice_Battery::buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into, MSDevice_StationFinder* sf) {
    // Check if vehicle should get a battery
    if (sf != nullptr || equippedByDefaultAssignmentOptions(OptionsCont::getOptions(), "battery", v, false)) {
        // obtain parameter values
        const double maximumBatteryCapacity = readParameterValue(v, SUMO_ATTR_MAXIMUMBATTERYCAPACITY, "battery.capacity", DEFAULT_MAX_CAPACITY);
        const double actualBatteryCapacity = readParameterValue(v, SUMO_ATTR_ACTUALBATTERYCAPACITY, "battery.chargeLevel", maximumBatteryCapacity * DEFAULT_CHARGE_RATIO);
        const double stoppingThreshold = readParameterValue(v, SUMO_ATTR_STOPPINGTHRESHOLD, "battery.stoppingThreshold", 0.1);
        const double maximumChargeRate = readParameterValue(v, SUMO_ATTR_MAXIMUMCHARGERATE, "battery.maximumChargeRate", 150000.);
        const std::string chargeLevelTable = v.getStringParam("device.battery.chargeLevelTable");
        const std::string chargeCurveTable = v.getStringParam("device.battery.chargeCurveTable");

        // battery constructor
        MSDevice_Battery* device = new MSDevice_Battery(v, "battery_" + v.getID(),
                actualBatteryCapacity, maximumBatteryCapacity, stoppingThreshold, maximumChargeRate, chargeLevelTable, chargeCurveTable);

        // Add device to vehicle
        into.push_back(device);

        if (sf != nullptr) {
            sf->setBattery(device);
        }
    }
}


double
MSDevice_Battery::readParameterValue(SUMOVehicle& v, const SumoXMLAttr& attr, const std::string& paramName, double defaultVal) {
    const std::string& oldParam = toString(attr);
    const SUMOVTypeParameter& typeParams = v.getVehicleType().getParameter();
    if (v.getParameter().hasParameter(oldParam) || typeParams.hasParameter(oldParam)) {
        WRITE_WARNINGF(TL("Battery device in vehicle '%' still uses old parameter '%'. Please update to 'device.%'."), v.getID(), oldParam, paramName);
        if (v.getParameter().getParameter(oldParam, "-") == "-") {
            return typeParams.getDouble(oldParam, defaultVal);
        }
        return StringUtils::toDouble(v.getParameter().getParameter(oldParam, "0"));
    }
    return v.getFloatParam("device." + paramName, false, defaultVal);
}


// ---------------------------------------------------------------------------
// MSDevice_Battery-methods
// ---------------------------------------------------------------------------
MSDevice_Battery::MSDevice_Battery(SUMOVehicle& holder, const std::string& id, const double actualBatteryCapacity, const double maximumBatteryCapacity,
                                   const double stoppingThreshold, const double maximumChargeRate, const std::string& chargeLevelTable, const std::string& chargeCurveTable) :
    MSVehicleDevice(holder, id),
    myActualBatteryCapacity(0),         // [actualBatteryCapacity <= maximumBatteryCapacity]
    myMaximumBatteryCapacity(0),        // [maximumBatteryCapacity >= 0]
    myStoppingThreshold(0),             // [stoppingThreshold >= 0]
    myMaximumChargeRate(0),
    myChargeLimit(-1),
    myLastAngle(std::numeric_limits<double>::infinity()),
    myChargingStopped(false),           // Initially vehicle don't charge stopped
    myChargingInTransit(false),         // Initially vehicle don't charge in transit
    myChargingStartTime(0),             // Initially charging start time (must be if the vehicle was launched at the charging station)
    myConsum(0),                        // Initially the vehicle is stopped and therefore the consum is zero.
    myTotalConsumption(0.0),
    myTotalRegenerated(0.0),
    myActChargingStation(nullptr),         // Initially the vehicle isn't over a Charging Station
    myPreviousNeighbouringChargingStation(nullptr),    // Initially the vehicle wasn't over a Charging Station
    myEnergyCharged(0),                 // Initially the energy charged is zero
    myVehicleStopped(0),
    myDepletedCount(0) {
    // Initially the vehicle is stopped and the corresponding variable is 0

    if (maximumBatteryCapacity < 0) {
        WRITE_WARNINGF(TL("Battery builder: Vehicle '%' doesn't have a valid value for parameter % (%)."), getID(), toString(SUMO_ATTR_MAXIMUMBATTERYCAPACITY), toString(maximumBatteryCapacity));
    } else {
        myMaximumBatteryCapacity = maximumBatteryCapacity;
    }

    if (actualBatteryCapacity > maximumBatteryCapacity) {
        WRITE_WARNINGF(TL("Battery builder: Vehicle '%' has a % (%) greater than its % (%). A max battery capacity value will be assigned."),
                       getID(), toString(SUMO_ATTR_ACTUALBATTERYCAPACITY), toString(actualBatteryCapacity), toString(SUMO_ATTR_MAXIMUMBATTERYCAPACITY), toString(maximumBatteryCapacity));
        myActualBatteryCapacity = myMaximumBatteryCapacity;
    } else {
        myActualBatteryCapacity = actualBatteryCapacity;
    }

    if (stoppingThreshold < 0) {
        WRITE_WARNINGF(TL("Battery builder: Vehicle '%' doesn't have a valid value for parameter % (%)."), getID(), toString(SUMO_ATTR_STOPPINGTHRESHOLD), toString(stoppingThreshold));
    } else {
        myStoppingThreshold = stoppingThreshold;
    }

    myTrackFuel = PollutantsInterface::getFuel(holder.getVehicleType().getEmissionClass()) != "Electricity" && OptionsCont::getOptions().getBool("device.battery.track-fuel");
    if (!myTrackFuel && !holder.getVehicleType().getParameter().wasSet(VTYPEPARS_EMISSIONCLASS_SET)) {
        WRITE_WARNINGF(TL("The battery device is active for vehicle '%' but no emission class is set. "
                          "Please consider setting an explicit emission class or battery outputs might be inconsistent with emission outputs!"),
                       holder.getID());
    }

    if (maximumChargeRate < 0) {
        WRITE_WARNINGF(TL("Battery builder: Vehicle '%' doesn't have a valid value for parameter % (%)."), getID(), toString(SUMO_ATTR_MAXIMUMCHARGERATE), toString(maximumChargeRate));
    } else {
        if (!chargeLevelTable.empty() && !chargeCurveTable.empty()) {
            LinearApproxHelpers::setPoints(myChargeCurve, chargeLevelTable, chargeCurveTable);
            if (!myTrackFuel) {
                LinearApproxHelpers::scaleValues(myChargeCurve, 1. / 3600);
            }
            myMaximumChargeRate = LinearApproxHelpers::getMaximumValue(myChargeCurve);
        } else {
            myMaximumChargeRate = maximumChargeRate;
            if (!myTrackFuel) {
                myMaximumChargeRate /= 3600.;
            }
        }
    }
}


MSDevice_Battery::~MSDevice_Battery() {
}


bool MSDevice_Battery::notifyMove(SUMOTrafficObject& tObject, double /* oldPos */, double /* newPos */, double /* newSpeed */) {
    if (!tObject.isVehicle()) {
        return false;
    }
    SUMOVehicle& veh = static_cast<SUMOVehicle&>(tObject);
    // Start vehicleStoppedTimer if the vehicle is stopped. In other case reset timer
    if (veh.getSpeed() < myStoppingThreshold) {
        // Increase vehicle stopped timer
        increaseVehicleStoppedTimer();
    } else {
        // Reset vehicle Stopped
        resetVehicleStoppedTimer();
    }

    // Update Energy from the battery
    EnergyParams* const params = myHolder.getEmissionParameters();
    if (getMaximumBatteryCapacity() != 0) {
        params->setDouble(SUMO_ATTR_ANGLE, myLastAngle == std::numeric_limits<double>::infinity() ? 0. : GeomHelper::angleDiff(myLastAngle, veh.getAngle()));
        if (!myTrackFuel && !veh.getVehicleType().getParameter().wasSet(VTYPEPARS_EMISSIONCLASS_SET)) {
            // no explicit emission class, we fall back to the energy model; a warning has been issued on creation
            myConsum = PollutantsInterface::getEnergyHelper().compute(0, PollutantsInterface::ELEC, veh.getSpeed(), veh.getAcceleration(),
                       veh.getSlope(), params) * TS;
        } else {
            myConsum = PollutantsInterface::compute(veh.getVehicleType().getEmissionClass(),
                                                    myTrackFuel ? PollutantsInterface::FUEL : PollutantsInterface::ELEC,
                                                    veh.getSpeed(), veh.getAcceleration(),
                                                    veh.getSlope(), params) * TS;
        }
        if (veh.isParking()) {
            // recuperation from last braking step is ok but further consumption should cease
            myConsum = MIN2(myConsum, 0.0);
        }

        // saturate between 0 and myMaximumBatteryCapacity [Wh]
        if (myConsum > getActualBatteryCapacity() && getActualBatteryCapacity() > 0 && getMaximumBatteryCapacity() > 0) {
            myDepletedCount++;
            if (myDepletedCount == 1) {
                WRITE_WARNINGF(TL("Battery of vehicle '%' is depleted, time=%."), veh.getID(), time2string(SIMSTEP));
            }
        }

        // Energy lost/gained from vehicle movement (via vehicle energy model) [Wh]
        setActualBatteryCapacity(getActualBatteryCapacity() - myConsum);

        // Track total energy consumption and regeneration
        if (myConsum > 0.0) {
            myTotalConsumption += myConsum;
        } else {
            myTotalRegenerated -= myConsum;
        }

        myLastAngle = veh.getAngle();
    }

    // Check if vehicle has under their position one charge Station
    const std::string chargingStationID = MSNet::getInstance()->getStoppingPlaceID(veh.getLane(), veh.getPositionOnLane(), SUMO_TAG_CHARGING_STATION);

    // If vehicle is over a charging station
    if (chargingStationID != "") {
        // if the vehicle is almost stopped, or charge in transit is enabled, then charge vehicle
        MSChargingStation* const cs = static_cast<MSChargingStation*>(MSNet::getInstance()->getStoppingPlace(chargingStationID, SUMO_TAG_CHARGING_STATION));
        const MSParkingArea* pa = cs->getParkingArea();
        if (((veh.getSpeed() < myStoppingThreshold) || cs->getChargeInTransit()) && (pa == nullptr || veh.isParking())) {
            // Set Flags Stopped/intransit to
            if (veh.getSpeed() < myStoppingThreshold) {
                // vehicle ist almost stopped, then is charging stopped
                myChargingStopped = true;

                // therefore isn't charging in transit
                myChargingInTransit = false;
            } else {
                // vehicle is moving, and the Charging station allow charge in transit
                myChargingStopped = false;

                // Therefore charge in transit
                myChargingInTransit = true;
            }

            // get pointer to charging station
            myActChargingStation = cs;

            // Only update charging start time if vehicle allow charge in transit, or in other case
            // if the vehicle not allow charge in transit but it's stopped.
            if ((myActChargingStation->getChargeInTransit()) || (veh.getSpeed() < myStoppingThreshold)) {
                // Update Charging start time
                increaseChargingStartTime();
            }

            // time it takes the vehicle at the station < charging station time delay?
            if (getChargingStartTime() > myActChargingStation->getChargeDelay()) {
                // Enable charging vehicle
                myActChargingStation->setChargingVehicle(true);

                // Calulate energy charged
                myEnergyCharged = MIN2(MIN2(myActChargingStation->getChargingPower(myTrackFuel) * myActChargingStation->getEfficency(), getMaximumChargeRate()) * TS, getMaximumBatteryCapacity() - getActualBatteryCapacity());

                // Update Battery charge
                setActualBatteryCapacity(getActualBatteryCapacity() + myEnergyCharged);
            }
            // add charge value for output to myActChargingStation
            myActChargingStation->addChargeValueForOutput(myEnergyCharged, this);
        }
        // else disable charging vehicle
        else {
            cs->setChargingVehicle(false);
        }
        // disable charging vehicle from previous (not current) ChargingStation (reason: if there is no gap between two different chargingStations = the vehicle switches from used charging station to other one in a single timestap)
        if (myPreviousNeighbouringChargingStation != nullptr && myPreviousNeighbouringChargingStation != cs) {
            myPreviousNeighbouringChargingStation->setChargingVehicle(false);
        }
        myPreviousNeighbouringChargingStation = cs;
    }
    // In other case, vehicle will be not charged
    else {
        // Disable flags
        myChargingInTransit = false;
        myChargingStopped = false;

        // Disable charging vehicle
        if (myActChargingStation != nullptr) {
            myActChargingStation->setChargingVehicle(false);
        }

        // Set charging station pointer to NULL
        myActChargingStation = nullptr;

        // Set energy charged to 0
        myEnergyCharged = 0.00;

        // Reset timer
        resetChargingStartTime();
    }

    // Always return true.
    return true;
}


void
MSDevice_Battery::saveState(OutputDevice& out) const {
    out.openTag(SUMO_TAG_DEVICE);
    out.writeAttr(SUMO_ATTR_ID, getID());
    std::vector<std::string> internals;
    internals.push_back(toString(myActualBatteryCapacity));
    internals.push_back(toString(myLastAngle));
    internals.push_back(toString(myChargingStopped));
    internals.push_back(toString(myChargingInTransit));
    internals.push_back(toString(myChargingStartTime));
    internals.push_back(toString(myTotalConsumption));
    internals.push_back(toString(myTotalRegenerated));
    internals.push_back(toString(myEnergyCharged));
    internals.push_back(toString(myVehicleStopped));
    internals.push_back(getChargingStationID());
    std::string prevChargingID = (myPreviousNeighbouringChargingStation == nullptr) ? "NULL" : myPreviousNeighbouringChargingStation->getID();
    internals.push_back(prevChargingID);
    internals.push_back(toString(myMaximumChargeRate));
    out.writeAttr(SUMO_ATTR_STATE, toString(internals));
    out.closeTag();
}


void
MSDevice_Battery::loadState(const SUMOSAXAttributes& attrs) {
    std::istringstream bis(attrs.getString(SUMO_ATTR_STATE));
    bis >> myActualBatteryCapacity;
    bis >> myLastAngle;
    bis >> myChargingStopped;
    bis >> myChargingInTransit;
    bis >> myChargingStartTime;
    bis >> myTotalConsumption;
    bis >> myTotalRegenerated;
    bis >> myEnergyCharged;
    bis >> myVehicleStopped;
    std::string chargingID;
    bis >> chargingID;
    if (chargingID != "NULL") {
        myActChargingStation = dynamic_cast<MSChargingStation*>(MSNet::getInstance()->getStoppingPlace(chargingID, SUMO_TAG_CHARGING_STATION));
    }
    std::string prevChargingID;
    bis >> prevChargingID;
    if (prevChargingID != "NULL") {
        myPreviousNeighbouringChargingStation = dynamic_cast<MSChargingStation*>(MSNet::getInstance()->getStoppingPlace(prevChargingID, SUMO_TAG_CHARGING_STATION));
    }
    bis >> myMaximumChargeRate;
}


void
MSDevice_Battery::setActualBatteryCapacity(const double actualBatteryCapacity) {
    if (actualBatteryCapacity < 0) {
        myActualBatteryCapacity = 0;
    } else if (actualBatteryCapacity > myMaximumBatteryCapacity) {
        myActualBatteryCapacity = myMaximumBatteryCapacity;
    } else {
        myActualBatteryCapacity = actualBatteryCapacity;
    }
}


void
MSDevice_Battery::setMaximumBatteryCapacity(const double maximumBatteryCapacity) {
    if (myMaximumBatteryCapacity < 0) {
        WRITE_WARNINGF(TL("Trying to set into the battery device of vehicle '%' an invalid % (%)."), getID(), toString(SUMO_ATTR_MAXIMUMBATTERYCAPACITY), toString(maximumBatteryCapacity));
    } else {
        myMaximumBatteryCapacity = maximumBatteryCapacity;
    }
}


void
MSDevice_Battery::setStoppingThreshold(const double stoppingThreshold) {
    if (stoppingThreshold < 0) {
        WRITE_WARNINGF(TL("Trying to set into the battery device of vehicle '%' an invalid % (%)."), getID(), toString(SUMO_ATTR_STOPPINGTHRESHOLD), toString(stoppingThreshold));
    } else {
        myStoppingThreshold = stoppingThreshold;
    }
}


void
MSDevice_Battery::setMaximumChargeRate(const double chargeRate) {
    if (chargeRate < 0) {
        WRITE_WARNINGF(TL("Trying to set into the battery device of vehicle '%' an invalid % (%)."), getID(), toString(SUMO_ATTR_MAXIMUMCHARGERATE), toString(chargeRate));
    } else {
        myMaximumChargeRate = chargeRate;
    }
}


void
MSDevice_Battery::setChargeLimit(const double limit) {
    myChargeLimit = limit;
}


void
MSDevice_Battery::resetChargingStartTime() {
    myChargingStartTime = 0;
}


void
MSDevice_Battery::increaseChargingStartTime() {
    myChargingStartTime += DELTA_T;
}


void
MSDevice_Battery::resetVehicleStoppedTimer() {
    myVehicleStopped = 0;
}


void
MSDevice_Battery::increaseVehicleStoppedTimer() {
    myVehicleStopped++;
}


double
MSDevice_Battery::getActualBatteryCapacity() const {
    return myActualBatteryCapacity;
}


double
MSDevice_Battery::getMaximumBatteryCapacity() const {
    return myMaximumBatteryCapacity;
}


double
MSDevice_Battery::getConsum() const {
    return myConsum;
}

double
MSDevice_Battery::getTotalConsumption() const {
    return myTotalConsumption;
}


double
MSDevice_Battery::getTotalRegenerated() const {
    return myTotalRegenerated;
}


bool
MSDevice_Battery::isChargingStopped() const {
    return myChargingStopped;
}


bool
MSDevice_Battery::isChargingInTransit() const {
    return myChargingInTransit;
}


SUMOTime
MSDevice_Battery::getChargingStartTime() const {
    return myChargingStartTime;
}


SUMOTime
MSDevice_Battery::estimateChargingDuration(const double toCharge, const double csPower) const {
    //if (!myChargeCurve.empty()) {
    //    // TODO: integrate charge curve
    //}
    return TIME2STEPS(toCharge / MIN2(csPower, myMaximumChargeRate));
}


std::string
MSDevice_Battery::getChargingStationID() const {
    if (myActChargingStation != nullptr) {
        return myActChargingStation->getID();
    } else {
        return "NULL";
    }
}

double
MSDevice_Battery::getEnergyCharged() const {
    return myEnergyCharged;
}


int
MSDevice_Battery::getVehicleStopped() const {
    return myVehicleStopped;
}


double
MSDevice_Battery::getStoppingThreshold() const {
    return myStoppingThreshold;
}


double
MSDevice_Battery::getMaximumChargeRate() const {
    double baseVal = (myChargeCurve.empty()) ? myMaximumChargeRate : LinearApproxHelpers::getInterpolatedValue(myChargeCurve, myActualBatteryCapacity / myMaximumBatteryCapacity);
    return (myChargeLimit < 0) ? baseVal : MIN2(myChargeLimit, baseVal);
}


std::string
MSDevice_Battery::getParameter(const std::string& key) const {
    if (key == toString(SUMO_ATTR_ACTUALBATTERYCAPACITY)
            || key == toString(SUMO_ATTR_CHARGELEVEL)) {
        return toString(getActualBatteryCapacity());
    } else if (key == toString(SUMO_ATTR_ENERGYCONSUMED)) {
        return toString(getConsum());
    } else if (key == toString(SUMO_ATTR_TOTALENERGYCONSUMED)) {
        return toString(getTotalConsumption());
    } else if (key == toString(SUMO_ATTR_TOTALENERGYREGENERATED)) {
        return toString(getTotalRegenerated());
    } else if (key == toString(SUMO_ATTR_ENERGYCHARGED)) {
        return toString(getEnergyCharged());
    } else if (key == toString(SUMO_ATTR_MAXIMUMBATTERYCAPACITY) || key == "capacity") {
        return toString(getMaximumBatteryCapacity());
    } else if (key == toString(SUMO_ATTR_MAXIMUMCHARGERATE)) {
        return toString(getMaximumChargeRate());
    } else if (key == toString(SUMO_ATTR_CHARGINGSTATIONID)) {
        return getChargingStationID();
    } else if (key == toString(SUMO_ATTR_VEHICLEMASS)) {
        WRITE_WARNING(TL("Getting the vehicle mass via parameters is deprecated, please use getMass for the vehicle or its type."));
        return toString(myHolder.getEmissionParameters()->getDouble(SUMO_ATTR_MASS));
    }
    throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
}


void
MSDevice_Battery::setParameter(const std::string& key, const std::string& value) {
    double doubleValue;
    try {
        doubleValue = StringUtils::toDouble(value);
    } catch (NumberFormatException&) {
        throw InvalidArgument("Setting parameter '" + key + "' requires a number for device of type '" + deviceName() + "'");
    }
    if (key == toString(SUMO_ATTR_ACTUALBATTERYCAPACITY) || key == toString(SUMO_ATTR_CHARGELEVEL)) {
        setActualBatteryCapacity(doubleValue);
    } else if (key == toString(SUMO_ATTR_MAXIMUMBATTERYCAPACITY) || key == "capacity") {
        setMaximumBatteryCapacity(doubleValue);
    } else if (key == toString(SUMO_ATTR_MAXIMUMCHARGERATE)) {
        setMaximumChargeRate(doubleValue);
    } else if (key == toString(SUMO_ATTR_VEHICLEMASS)) {
        WRITE_WARNING(TL("Setting the vehicle mass via parameters is deprecated, please use setMass for the vehicle or its type."));
        myHolder.getEmissionParameters()->setDouble(SUMO_ATTR_MASS, doubleValue);
    } else {
        throw InvalidArgument("Setting parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
    }
}


void
MSDevice_Battery::notifyParking() {
    // @note: only charing is performed but no energy is consumed
    notifyMove(myHolder, myHolder.getPositionOnLane(), myHolder.getPositionOnLane(), myHolder.getSpeed());
    myConsum = 0;
}


void
MSDevice_Battery::generateOutput(OutputDevice* tripinfoOut) const {
    if (tripinfoOut != nullptr) {
        tripinfoOut->openTag("battery");
        tripinfoOut->writeAttr("depleted", toString(myDepletedCount));
        tripinfoOut->closeTag();
    }
}


/****************************************************************************/
