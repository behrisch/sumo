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
/// @file    SUMOVTypeParameter.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    10.09.2009
///
// Structure representing possible vehicle parameter
/****************************************************************************/
#include <config.h>

#include <algorithm>
#include <utils/vehicle/SUMOVTypeParameter.h>
#include <utils/common/ToString.h>
#include <utils/common/StringUtils.h>
#include <utils/common/MsgHandler.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/options/OptionsCont.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include <utils/emissions/PollutantsInterface.h>

#define EMPREFIX std::string("HBEFA3/")
#define TTT_UNSET SUMOTime_MIN

// ===========================================================================
// static value definitions
// ===========================================================================
std::set<SumoXMLAttr> SUMOVTypeParameter::AllowedJMAttrs({
    SUMO_ATTR_JM_CROSSING_GAP,
    SUMO_ATTR_JM_DRIVE_AFTER_YELLOW_TIME,
    SUMO_ATTR_JM_DRIVE_AFTER_RED_TIME,
    SUMO_ATTR_JM_DRIVE_RED_SPEED,
    SUMO_ATTR_JM_IGNORE_KEEPCLEAR_TIME,
    SUMO_ATTR_JM_IGNORE_FOE_SPEED,
    SUMO_ATTR_JM_IGNORE_FOE_PROB,
    SUMO_ATTR_JM_IGNORE_JUNCTION_FOE_PROB,
    SUMO_ATTR_JM_SIGMA_MINOR,
    SUMO_ATTR_JM_STOPLINE_GAP,
    SUMO_ATTR_JM_TIMEGAP_MINOR,
    SUMO_ATTR_JM_EXTRA_GAP,
    SUMO_ATTR_JM_ADVANCE,
    SUMO_ATTR_JM_STOPSIGN_WAIT,
    SUMO_ATTR_JM_ALLWAYSTOP_WAIT,
});


// ===========================================================================
// member method definitions
// ===========================================================================

SUMOVTypeParameter::VClassDefaultValues::VClassDefaultValues(SUMOVehicleClass vclass) :
    length(getDefaultVehicleLength(vclass)),
    minGap(2.5),
    minGapLat(0.6),
    maxSpeed(200. / 3.6),
    desiredMaxSpeed(10000 / 3.6), // backward-compatibility: do not influence speeds by default
    width(1.8),
    height(1.5),
    shape(SUMOVehicleShape::UNKNOWN),
    emissionClass(PollutantsInterface::getClassByName(EMPREFIX + "PC_G_EU4", vclass)),
    mass(1500.),
    speedFactor("normc", 1.0, 0.0, 0.2, 2.0),
    personCapacity(4),
    containerCapacity(0),
    osgFile("car-normal-citrus.obj"),
    carriageLength(-1),
    locomotiveLength(-1),
    carriageDoors(2),
    latAlignmentProcedure(LatAlignmentDefinition::CENTER) {
    // update default values
    switch (vclass) {
        case SVC_PEDESTRIAN:
            minGap = 0.25;
            maxSpeed = 37.58 / 3.6; // Usain Bolt
            desiredMaxSpeed = DEFAULT_PEDESTRIAN_SPEED;
            width = 0.478;
            height = 1.719;
            shape = SUMOVehicleShape::PEDESTRIAN;
            osgFile = "humanResting.obj";
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "zero", vclass);
            mass = 70.; // https://en.wikipedia.org/wiki/Human_body_weight for Europe
            speedFactor.getParameter()[1] = 0.1;
            break;
        case SVC_WHEELCHAIR:
            minGap = 0.5;
            maxSpeed = 30.0 / 3.6; // https://en.wikipedia.org/wiki/Wheelchair_racing
            desiredMaxSpeed = DEFAULT_PEDESTRIAN_SPEED;
            width = 0.8;
            height = 1.5;
            shape = SUMOVehicleShape::PEDESTRIAN;
            osgFile = "humanResting.obj";
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "zero", vclass);
            mass = 90.; //
            speedFactor.getParameter()[1] = 0.1;
            break;
        case SVC_BICYCLE:
            minGap = 0.5;
            minGapLat = 0.35;
            maxSpeed = 50. / 3.6;
            desiredMaxSpeed = DEFAULT_BICYCLE_SPEED;
            width = 0.65;
            height = 1.7;
            shape = SUMOVehicleShape::BICYCLE;
            personCapacity = 1;
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "zero", vclass);
            mass = 10.;
            speedFactor.getParameter()[1] = 0.1;
            latAlignmentProcedure = LatAlignmentDefinition::RIGHT;
            break;
        case SVC_SCOOTER:
            minGap = 0.5;
            minGapLat = 0.35;
            maxSpeed = 25 / 3.6;
            desiredMaxSpeed = DEFAULT_BICYCLE_SPEED;
            width = 0.5;
            height = 1.7;
            shape = SUMOVehicleShape::SCOOTER;
            personCapacity = 1;
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "zero", vclass);
            mass = 10.;
            speedFactor.getParameter()[1] = 0.1;
            latAlignmentProcedure = LatAlignmentDefinition::RIGHT;
            break;
        case SVC_MOPED:
            maxSpeed = 60. / 3.6;
            width = 0.78;
            height = 1.7;
            shape = SUMOVehicleShape::MOPED;
            personCapacity = 1;
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "LDV_G_EU6", vclass);
            mass = 80.;
            speedFactor.getParameter()[1] = 0.1;
            break;
        case SVC_MOTORCYCLE:
            width = 0.9;
            height = 1.5;
            shape = SUMOVehicleShape::MOTORCYCLE;
            personCapacity = 1;
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "LDV_G_EU6", vclass);
            mass = 200.;
            speedFactor.getParameter()[1] = 0.1;
            break;
        case SVC_TRUCK:
            maxSpeed = 130. / 3.6;
            width = 2.4;
            height = 2.4;
            shape = SUMOVehicleShape::TRUCK;
            osgFile = "car-microcargo-citrus.obj";
            personCapacity = 2;
            containerCapacity = 1;
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "HDV", vclass);
            mass = 12000.;
            speedFactor.getParameter()[1] = 0.05;
            break;
        case SVC_TRAILER:
            maxSpeed = 130. / 3.6;
            width = 2.55;
            height = 4.;
            shape = SUMOVehicleShape::TRUCK_1TRAILER;
            osgFile = "car-microcargo-citrus.obj";
            personCapacity = 2;
            containerCapacity = 2;
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "HDV", vclass);
            mass = 15000.;
            speedFactor.getParameter()[1] = 0.05;
            break;
        case SVC_BUS:
            maxSpeed = 100. / 3.6;
            width = 2.5;
            height = 3.4;
            shape = SUMOVehicleShape::BUS;
            osgFile = "car-minibus-citrus.obj";
            personCapacity = 85;
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "Bus", vclass);
            mass = 7500.;
            break;
        case SVC_COACH:
            maxSpeed = 100. / 3.6;
            width = 2.6;
            height = 4.;
            shape = SUMOVehicleShape::BUS_COACH;
            osgFile = "car-minibus-citrus.obj";
            personCapacity = 70;
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "Coach", vclass);
            mass = 12000.;
            speedFactor.getParameter()[1] = 0.05;
            break;
        case SVC_TRAM:
            maxSpeed = 80. / 3.6;
            width = 2.4;
            height = 3.2;
            shape = SUMOVehicleShape::RAIL_CAR;
            osgFile = "tram.obj";
            personCapacity = 120;
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "zero", vclass);
            mass = 37900.;
            break;
        case SVC_RAIL_URBAN:
        case SVC_SUBWAY:
            maxSpeed = 100. / 3.6;
            minGap = 5;
            width = 3.0;
            height = 3.6;
            shape = SUMOVehicleShape::RAIL_CAR;
            personCapacity = 300;
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "zero", vclass);
            mass = 59000.;
            break;
        case SVC_RAIL:
            maxSpeed = 160. / 3.6;
            minGap = 5;
            width = 2.84;
            height = 3.75;
            shape = SUMOVehicleShape::RAIL;
            personCapacity = 434;
            // slight understatement (-:
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "HDV_D_EU0", vclass);
            mass = 79500.; // only locomotive
            break;
        case SVC_RAIL_ELECTRIC:
            maxSpeed = 220. / 3.6;
            minGap = 5;
            width = 2.95;
            height = 3.89;
            shape = SUMOVehicleShape::RAIL;
            personCapacity = 425;
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "zero", vclass);
            mass = 83000.; // only locomotive
            break;
        case SVC_RAIL_FAST:
            maxSpeed = 330. / 3.6;
            minGap = 5;
            width = 2.95;
            height = 3.89;
            shape = SUMOVehicleShape::RAIL;
            personCapacity = 425;
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "zero", vclass);
            mass = 409000.;
            break;
        case SVC_DELIVERY:
            width = 2.16;
            height = 2.86;
            shape = SUMOVehicleShape::DELIVERY;
            personCapacity = 2;
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "LDV", vclass);
            mass = 5000.;
            speedFactor.getParameter()[1] = 0.05;
            break;
        case SVC_EMERGENCY:
            width = 2.16;
            height = 2.86;
            shape = SUMOVehicleShape::DELIVERY;
            personCapacity = 2;
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "LDV", vclass);
            mass = 5000.;
            break;
        case SVC_PRIVATE:
        case SVC_VIP:
        case SVC_PASSENGER:
        case SVC_HOV:
        case SVC_CUSTOM1:
        case SVC_CUSTOM2:
            shape = SUMOVehicleShape::PASSENGER;
            speedFactor.getParameter()[1] = 0.1;
            break;
        case SVC_TAXI:
            shape = SUMOVehicleShape::TAXI;
            speedFactor.getParameter()[1] = 0.05;
            break;
        case SVC_E_VEHICLE:
            shape = SUMOVehicleShape::E_VEHICLE;
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "zero", vclass);
            speedFactor.getParameter()[1] = 0.1;
            break;
        case SVC_CONTAINER:
            width = 2.5908;
            break;
        case SVC_DRONE:
            width = 0.5;
            break;
        case SVC_AIRCRAFT:
            // Airbus A380
            shape = SUMOVehicleShape::AIRCRAFT;
            width = 79.8;
            break;
        case SVC_SHIP:
            width = 4;
            maxSpeed = 8 / 1.94; // 8 knots
            height = 4;
            shape = SUMOVehicleShape::SHIP;
            // slight understatement (-:
            emissionClass = PollutantsInterface::getClassByName(EMPREFIX + "HDV_D_EU0", vclass);
            mass = 100000.;
            speedFactor.getParameter()[1] = 0.1;
            break;
        default:
            break;
    }
}


SUMOVTypeParameter::VClassDefaultValues::VClassDefaultValues() :
    speedFactor("normc", 1.0, 0.0, 0.2, 2.0) {}

SUMOVTypeParameter::SUMOVTypeParameter(const std::string& vtid, const SUMOVehicleClass vclass)
    : id(vtid),
      actionStepLength(0),
      defaultProbability(DEFAULT_VEH_PROB),
      speedFactor("normc", 1.0, 0.0, 0.2, 2.0),
      emissionClass(PollutantsInterface::getClassByName(EMPREFIX + "PC_G_EU4", vclass)),
      color(RGBColor::DEFAULT_COLOR),
      vehicleClass(vclass),
      impatience(0.0),
      personCapacity(4),
      containerCapacity(0),
      boardingDuration(500),
      loadingDuration(90000),
      scale(1),
      width(1.8),
      height(1.5),
      shape(SUMOVehicleShape::UNKNOWN),
      osgFile("car-normal-citrus.obj"),
      cfModel(SUMO_TAG_CF_KRAUSS),
      lcModel(LaneChangeModel::DEFAULT),
      maxSpeedLat(1.0),
      latAlignmentOffset(0.0),
      latAlignmentProcedure(LatAlignmentDefinition::CENTER),
      carriageLength(-1),
      locomotiveLength(-1),
      carriageGap(1),
      carriageDoors(2),
      timeToTeleport(TTT_UNSET),
      timeToTeleportBidi(TTT_UNSET),
      speedFactorPremature(-1),
      frontSeatPos(1.7),
      seatingWidth(-1),
      boardingFactor(1),
      parametersSet(0),
      saved(false),
      onlyReferenced(false) {
    const OptionsCont& oc = OptionsCont::getOptions();
    if (oc.exists("carfollow.model")) {
        // check for valid value has been performed in MSFrame
        cfModel = SUMOXMLDefinitions::CarFollowModels.get(oc.getString("carfollow.model"));
    }
    // obtain default values depending of vclass
    VClassDefaultValues defaultValues(vclass);
    // overwrite SUMOVTypeParameter with VClassDefaultValues
    length = defaultValues.length;
    minGap = defaultValues.minGap;
    minGapLat = defaultValues.minGapLat;
    maxSpeed = defaultValues.maxSpeed;
    desiredMaxSpeed = defaultValues.desiredMaxSpeed;
    width = defaultValues.width;
    height = defaultValues.height;
    shape = defaultValues.shape;
    emissionClass = defaultValues.emissionClass;
    mass = defaultValues.mass;
    speedFactor = defaultValues.speedFactor;
    personCapacity = defaultValues.personCapacity;
    containerCapacity = defaultValues.containerCapacity;
    osgFile = defaultValues.osgFile;
    carriageLength = defaultValues.carriageLength;
    locomotiveLength = defaultValues.locomotiveLength;
    carriageDoors = defaultValues.carriageDoors;
    latAlignmentProcedure = defaultValues.latAlignmentProcedure;
    // check if default speeddev was defined
    if (oc.exists("default.speeddev")) {
        const double defaultSpeedDev = oc.getFloat("default.speeddev");
        if (defaultSpeedDev >= 0) {
            speedFactor.getParameter()[1] = defaultSpeedDev;
        }
    } else {
        speedFactor.getParameter()[1] = -1;
    }
    setManoeuverAngleTimes(vclass);
}

void
SUMOVTypeParameter::setManoeuverAngleTimes(const SUMOVehicleClass vclass) {

    myManoeuverAngleTimes.clear();
    /**
     * Defaults assume:   approaching at angles between 0-10 and 171-180 (will never be > 180) are approaching a space roughly parallel to the road
     *                    approaching at angles between 11-80 are approaching an acute angled space that is easiest to drive straight in
     *                    approaching at angles between 81-110 are approaching a space at approximately right angles to the road so the driver has a choice
     *                    approaching at angles between 111 and 170 are approaching an obtuse angled space that is easiest to drive past and reverse in
     *              More (or less) granular angle ranges can be used - configurable as a vType parameter
     */
    switch (vclass) {
        case SVC_PASSENGER:
        case SVC_HOV:
        case SVC_TAXI:
        case SVC_E_VEHICLE:
            myManoeuverAngleTimes.insert(std::pair<int, std::pair<SUMOTime, SUMOTime>>(10, std::pair< SUMOTime, SUMOTime>(3000, 4000)));     // straight in but potentially needing parallel parking
            myManoeuverAngleTimes.insert(std::pair<int, std::pair<SUMOTime, SUMOTime>>(80, std::pair< SUMOTime, SUMOTime>(1000, 11000)));    // straight in
            myManoeuverAngleTimes.insert(std::pair<int, std::pair<SUMOTime, SUMOTime>>(110, std::pair< SUMOTime, SUMOTime>(11000, 2000)));   // optional forwards/backwards
            myManoeuverAngleTimes.insert(std::pair<int, std::pair<SUMOTime, SUMOTime>>(170, std::pair< SUMOTime, SUMOTime>(8000, 3000)));    // backwards into obtuse space
            myManoeuverAngleTimes.insert(std::pair<int, std::pair<SUMOTime, SUMOTime>>(181, std::pair< SUMOTime, SUMOTime>(3000, 4000)));    // straight in but potentially needing parallel parking
            break;
        case SVC_TRUCK:
        case SVC_TRAILER:
        case SVC_BUS:
        case SVC_COACH:
        case SVC_DELIVERY:
            myManoeuverAngleTimes.insert(std::pair<int, std::pair<SUMOTime, SUMOTime>>(10, std::pair< SUMOTime, SUMOTime>(6000, 8000)));    // straight in but potentially needing parallel parking
            myManoeuverAngleTimes.insert(std::pair<int, std::pair<SUMOTime, SUMOTime>>(80, std::pair< SUMOTime, SUMOTime>(2000, 21000)));   // straight in
            myManoeuverAngleTimes.insert(std::pair<int, std::pair<SUMOTime, SUMOTime>>(110, std::pair< SUMOTime, SUMOTime>(21000, 2000)));  // optional forwards/backwards
            myManoeuverAngleTimes.insert(std::pair<int, std::pair<SUMOTime, SUMOTime>>(170, std::pair< SUMOTime, SUMOTime>(14000, 5000)));  // backwards into obtuse space
            myManoeuverAngleTimes.insert(std::pair<int, std::pair<SUMOTime, SUMOTime>>(181, std::pair< SUMOTime, SUMOTime>(6000, 8000)));   // straight in but potentially needing parallel parking
            break;
        case SVC_PEDESTRIAN:
        case SVC_MOPED:
        case SVC_BICYCLE:
            myManoeuverAngleTimes.insert(std::pair<int, std::pair<SUMOTime, SUMOTime>>(181, std::pair< SUMOTime, SUMOTime>(1000, 1000)));  // no dependence on angle
            break;
        default:
            myManoeuverAngleTimes.insert(std::pair<int, std::pair<SUMOTime, SUMOTime>>(10, std::pair< SUMOTime, SUMOTime>(3000, 4000)));    // straight in but potentially needing parallel parking
            myManoeuverAngleTimes.insert(std::pair<int, std::pair<SUMOTime, SUMOTime>>(80, std::pair< SUMOTime, SUMOTime>(1000, 11000)));   // straight in
            myManoeuverAngleTimes.insert(std::pair<int, std::pair<SUMOTime, SUMOTime>>(110, std::pair< SUMOTime, SUMOTime>(11000, 2000)));  // optional forwards/backwards
            myManoeuverAngleTimes.insert(std::pair<int, std::pair<SUMOTime, SUMOTime>>(170, std::pair< SUMOTime, SUMOTime>(8000, 3000)));   // backwards into obtuse space
            myManoeuverAngleTimes.insert(std::pair<int, std::pair<SUMOTime, SUMOTime>>(181, std::pair< SUMOTime, SUMOTime>(3000, 4000)));   // straight in but potentially needing parallel parking
            break;
    }
}

void
SUMOVTypeParameter::write(OutputDevice& dev) const {
    // first check if vehicle type can be written
    if (onlyReferenced) {
        return;
    }
    // open vehicle tag
    dev.openTag(SUMO_TAG_VTYPE);
    // write ID (always needed)
    dev.writeAttr(SUMO_ATTR_ID, id);
    // write parameters depending if is set
    if (wasSet(VTYPEPARS_LENGTH_SET)) {
        dev.writeAttr(SUMO_ATTR_LENGTH, length);
    }
    if (wasSet(VTYPEPARS_MINGAP_SET)) {
        dev.writeAttr(SUMO_ATTR_MINGAP, minGap);
    }
    if (wasSet(VTYPEPARS_MAXSPEED_SET)) {
        dev.writeAttr(SUMO_ATTR_MAXSPEED, maxSpeed);
    }
    if (wasSet(VTYPEPARS_DESIRED_MAXSPEED_SET)) {
        dev.writeAttr(SUMO_ATTR_DESIRED_MAXSPEED, desiredMaxSpeed);
    }
    if (wasSet(VTYPEPARS_PROBABILITY_SET)) {
        dev.writeAttr(SUMO_ATTR_PROB, defaultProbability);
    }
    if (wasSet(VTYPEPARS_SPEEDFACTOR_SET)) {
        dev.writeAttr(SUMO_ATTR_SPEEDFACTOR, speedFactor);
    }
    if (wasSet(VTYPEPARS_ACTIONSTEPLENGTH_SET)) {
        // Note: action step length is only exposed in seconds to the user
        dev.writeAttr(SUMO_ATTR_ACTIONSTEPLENGTH, STEPS2TIME(actionStepLength));
    }
    if (wasSet(VTYPEPARS_VEHICLECLASS_SET)) {
        dev.writeAttr(SUMO_ATTR_VCLASS, toString(vehicleClass));
    }
    if (wasSet(VTYPEPARS_EMISSIONCLASS_SET)) {
        dev.writeAttr(SUMO_ATTR_EMISSIONCLASS, PollutantsInterface::getName(emissionClass));
    }
    if (wasSet(VTYPEPARS_MASS_SET)) {
        dev.writeAttr(SUMO_ATTR_MASS, mass);
    }
    if (wasSet(VTYPEPARS_IMPATIENCE_SET)) {
        if (impatience == -std::numeric_limits<double>::max()) {
            dev.writeAttr(SUMO_ATTR_IMPATIENCE, "off");
        } else {
            dev.writeAttr(SUMO_ATTR_IMPATIENCE, impatience);
        }
    }
    if (wasSet(VTYPEPARS_SHAPE_SET)) {
        dev.writeAttr(SUMO_ATTR_GUISHAPE, getVehicleShapeName(shape));
    }
    if (wasSet(VTYPEPARS_WIDTH_SET)) {
        dev.writeAttr(SUMO_ATTR_WIDTH, width);
    }
    if (wasSet(VTYPEPARS_HEIGHT_SET)) {
        dev.writeAttr(SUMO_ATTR_HEIGHT, height);
    }
    if (wasSet(VTYPEPARS_COLOR_SET)) {
        dev.writeAttr(SUMO_ATTR_COLOR, color);
    }
    if (wasSet(VTYPEPARS_OSGFILE_SET)) {
        dev.writeAttr(SUMO_ATTR_OSGFILE, osgFile);
    }
    if (wasSet(VTYPEPARS_IMGFILE_SET)) {
        dev.writeAttr(SUMO_ATTR_IMGFILE, imgFile);
    }
    if (wasSet(VTYPEPARS_PERSON_CAPACITY)) {
        dev.writeAttr(SUMO_ATTR_PERSON_CAPACITY, personCapacity);
    }
    if (wasSet(VTYPEPARS_CONTAINER_CAPACITY)) {
        dev.writeAttr(SUMO_ATTR_CONTAINER_CAPACITY, containerCapacity);
    }
    if (wasSet(VTYPEPARS_BOARDING_DURATION)) {
        dev.writeAttr(SUMO_ATTR_BOARDING_DURATION, time2string(boardingDuration));
    }
    if (wasSet(VTYPEPARS_LOADING_DURATION)) {
        dev.writeAttr(SUMO_ATTR_LOADING_DURATION, time2string(loadingDuration));
    }
    if (wasSet(VTYPEPARS_MAXSPEED_LAT_SET)) {
        dev.writeAttr(SUMO_ATTR_MAXSPEED_LAT, maxSpeedLat);
    }
    if (wasSet(VTYPEPARS_LATALIGNMENT_SET)) {
        switch (latAlignmentProcedure) {
            case LatAlignmentDefinition::GIVEN:
                dev.writeAttr(SUMO_ATTR_LATALIGNMENT, latAlignmentOffset);
                break;
            case LatAlignmentDefinition::RIGHT:
                dev.writeAttr(SUMO_ATTR_LATALIGNMENT, "right");
                break;
            case LatAlignmentDefinition::CENTER:
                dev.writeAttr(SUMO_ATTR_LATALIGNMENT, "center");
                break;
            case LatAlignmentDefinition::ARBITRARY:
                dev.writeAttr(SUMO_ATTR_LATALIGNMENT, "arbitrary");
                break;
            case LatAlignmentDefinition::NICE:
                dev.writeAttr(SUMO_ATTR_LATALIGNMENT, "nice");
                break;
            case LatAlignmentDefinition::COMPACT:
                dev.writeAttr(SUMO_ATTR_LATALIGNMENT, "compact");
                break;
            case LatAlignmentDefinition::LEFT:
                dev.writeAttr(SUMO_ATTR_LATALIGNMENT, "left");
                break;
            case LatAlignmentDefinition::DEFAULT:
            default:
                break;
        }
    }
    if (wasSet(VTYPEPARS_MINGAP_LAT_SET)) {
        dev.writeAttr(SUMO_ATTR_MINGAP_LAT, minGapLat);
    }
    if (wasSet(VTYPEPARS_MANEUVER_ANGLE_TIMES_SET)) {
        dev.writeAttr(SUMO_ATTR_MANEUVER_ANGLE_TIMES, getManoeuverAngleTimesS());
    }
    if (wasSet(VTYPEPARS_SCALE_SET)) {
        dev.writeAttr(SUMO_ATTR_SCALE, scale);
    }
    if (wasSet(VTYPEPARS_TTT_SET)) {
        dev.writeAttr(SUMO_ATTR_TIME_TO_TELEPORT, time2string(timeToTeleport));
    }
    if (wasSet(VTYPEPARS_TTT_BIDI_SET)) {
        dev.writeAttr(SUMO_ATTR_TIME_TO_TELEPORT_BIDI, time2string(timeToTeleportBidi));
    }
    if (wasSet(VTYPEPARS_SPEEDFACTOR_PREMATURE_SET)) {
        dev.writeAttr(SUMO_ATTR_SPEEDFACTOR_PREMATURE, speedFactorPremature);
    }
    if (wasSet(VTYPEPARS_BOARDING_FACTOR_SET)) {
        dev.writeAttr(SUMO_ATTR_BOARDING_FACTOR, boardingFactor);
    }
    if (wasSet(VTYPEPARS_LANE_CHANGE_MODEL_SET)) {
        dev.writeAttr(SUMO_ATTR_LANE_CHANGE_MODEL, lcModel);
    }
    // Write Lane Change Model parameters
    for (const auto& lcParam : lcParameter) {
        dev.writeAttr(lcParam.first, lcParam.second);
    }
    // Write Junction Model parameter
    for (const auto& jmParam : jmParameter) {
        dev.writeAttr(jmParam.first, jmParam.second);
    }
    if (wasSet(VTYPEPARS_CAR_FOLLOW_MODEL)) {
        dev.writeAttr(SUMO_ATTR_CAR_FOLLOW_MODEL, SUMOXMLDefinitions::CarFollowModels.getString(cfModel));
    }
    // Write Car Following Model parameters
    for (const auto& cfParam : cfParameter) {
        dev.writeAttr(cfParam.first, cfParam.second);
    }
    // Write carriage length
    if (wasSet(VTYPEPARS_CARRIAGE_LENGTH_SET)) {
        dev.openTag(SUMO_TAG_PARAM);
        dev.writeAttr(SUMO_ATTR_KEY, toString(SUMO_ATTR_CARRIAGE_LENGTH));
        dev.writeAttr(SUMO_ATTR_VALUE, toString(carriageLength));
        dev.closeTag();
    }
    // Write locomotive length
    if (wasSet(VTYPEPARS_LOCOMOTIVE_LENGTH_SET)) {
        dev.openTag(SUMO_TAG_PARAM);
        dev.writeAttr(SUMO_ATTR_KEY, toString(SUMO_ATTR_LOCOMOTIVE_LENGTH));
        dev.writeAttr(SUMO_ATTR_VALUE, toString(locomotiveLength));
        dev.closeTag();
    }
    // Write carriage gap
    if (wasSet(VTYPEPARS_CARRIAGE_GAP_SET)) {
        dev.openTag(SUMO_TAG_PARAM);
        dev.writeAttr(SUMO_ATTR_KEY, toString(SUMO_ATTR_CARRIAGE_GAP));
        dev.writeAttr(SUMO_ATTR_VALUE, toString(carriageGap));
        dev.closeTag();
    }
    // Write carriage doors
    if (wasSet(VTYPEPARS_CARRIAGE_DOORS_SET)) {
        dev.openTag(SUMO_TAG_PARAM);
        dev.writeAttr(SUMO_ATTR_KEY, toString(SUMO_ATTR_CARRIAGE_DOORS));
        dev.writeAttr(SUMO_ATTR_VALUE, toString(carriageDoors));
        dev.closeTag();
    }
    // Write rest of parameters
    writeParams(dev);
    // close tag
    dev.closeTag();
}


double
SUMOVTypeParameter::getCFParam(const SumoXMLAttr attr, const double defaultValue) const {
    if (cfParameter.count(attr)) {
        return StringUtils::toDouble(cfParameter.find(attr)->second);
    } else {
        return defaultValue;
    }
}


std::string
SUMOVTypeParameter::getCFParamString(const SumoXMLAttr attr, const std::string defaultValue) const {
    if (cfParameter.count(attr)) {
        return cfParameter.find(attr)->second;
    } else {
        return defaultValue;
    }
}


double
SUMOVTypeParameter::getLCParam(const SumoXMLAttr attr, const double defaultValue) const {
    if (lcParameter.count(attr)) {
        return StringUtils::toDouble(lcParameter.find(attr)->second);
    } else {
        return defaultValue;
    }
}


std::string
SUMOVTypeParameter::getLCParamString(const SumoXMLAttr attr, const std::string& defaultValue) const {
    if (lcParameter.count(attr)) {
        return lcParameter.find(attr)->second;
    } else {
        return defaultValue;
    }
}


const SUMOVTypeParameter::SubParams&
SUMOVTypeParameter::getLCParams() const {
    return lcParameter;
}


double
SUMOVTypeParameter::getJMParam(const SumoXMLAttr attr, const double defaultValue) const {
    if (jmParameter.count(attr)) {
        return StringUtils::toDouble(jmParameter.find(attr)->second);
    } else {
        return defaultValue;
    }
}


std::string
SUMOVTypeParameter::getJMParamString(const SumoXMLAttr attr, const std::string defaultValue) const {
    if (jmParameter.count(attr)) {
        return jmParameter.find(attr)->second;
    } else {
        return defaultValue;
    }
}

SUMOTime
SUMOVTypeParameter::getEntryManoeuvreTime(const int angle) const {
    SUMOTime last = 0;
    for (std::pair<int, std::pair<SUMOTime, SUMOTime>> angleTime : myManoeuverAngleTimes) {
        if (angle <= angleTime.first) {
            return (angleTime.second.first);
        } else {
            last = angleTime.second.first;
        }
    }
    return (last);
}

SUMOTime
SUMOVTypeParameter::getExitManoeuvreTime(const int angle) const {
    SUMOTime last = 0;
    for (std::pair<int, std::pair<SUMOTime, SUMOTime>> angleTime : myManoeuverAngleTimes) {
        if (angle <= angleTime.first) {
            return (angleTime.second.second);
        } else {
            last = angleTime.second.second;
        }
    }
    return (last);
}

std::string
SUMOVTypeParameter::getManoeuverAngleTimesS() const {
    std::stringstream stream;

    stream << std::fixed << std::setprecision(1);
    int count = 0;
    for (std::pair<int, std::pair<SUMOTime, SUMOTime>> angleTime : myManoeuverAngleTimes) {
        if (count++ > 0) {
            stream << ",";
        }
        stream << toString(angleTime.first) + " " << STEPS2TIME(angleTime.second.first) << " " << STEPS2TIME(angleTime.second.second);
    }
    std::string triplets = stream.str();
    return triplets;
}


void
SUMOVTypeParameter::cacheParamRestrictions(const std::vector<std::string>& restrictionKeys) {
    for (const std::string& key : restrictionKeys) {
        paramRestrictions.push_back(StringUtils::toDouble(getParameter(key, "0")));
    }
}


void
SUMOVTypeParameter::initRailVisualizationParameters() {
    if (hasParameter("carriageLength")) {
        carriageLength = StringUtils::toDouble(getParameter("carriageLength"));
        parametersSet |= VTYPEPARS_CARRIAGE_LENGTH_SET;
    } else {
        switch (shape) {
            case SUMOVehicleShape::BUS_FLEXIBLE:
                carriageLength = 8.25; // 16.5 overall, 2 modules http://de.wikipedia.org/wiki/Ikarus_180
                carriageGap = 0;
                break;
            case SUMOVehicleShape::RAIL:
                if (vehicleClass == SVC_RAIL_ELECTRIC) {
                    carriageLength = 24.5;
                    locomotiveLength = 19.100; // https://en.wikipedia.org/wiki/DB_Class_101
                } else if (vehicleClass == SVC_RAIL_FAST) {
                    carriageLength = 24.775; // http://de.wikipedia.org/wiki/ICE_3
                    locomotiveLength = 25.835;
                } else {
                    carriageLength = 24.5; // http://de.wikipedia.org/wiki/UIC-Y-Wagen_%28DR%29
                    locomotiveLength = 16.4; // https://en.wikipedia.org/wiki/DB_Class_218
                }
                break;
            case SUMOVehicleShape::RAIL_CAR:
                if (vehicleClass == SVC_TRAM) {
                    carriageLength = 5.71; // http://de.wikipedia.org/wiki/Bombardier_Flexity_Berlin
                    locomotiveLength = 5.71;
                } else if (vehicleClass == SVC_RAIL_URBAN) {
                    carriageLength = 18.4;  // https://en.wikipedia.org/wiki/DBAG_Class_481
                    locomotiveLength = 18.4;
                } else {
                    carriageLength = 16.85;  // 67.4m overall, 4 carriages http://de.wikipedia.org/wiki/DB-Baureihe_423
                }
                break;
            case SUMOVehicleShape::RAIL_CARGO:
                carriageLength = 13.86; // UIC 571-1 http://de.wikipedia.org/wiki/Flachwagen
                break;
            case SUMOVehicleShape::TRUCK_SEMITRAILER:
                carriageLength = 13.5;
                locomotiveLength = 2.5;
                carriageGap = 0.5;
                break;
            case SUMOVehicleShape::TRUCK_1TRAILER:
                carriageLength = 6.75;
                locomotiveLength = 2.5 + 6.75;
                carriageGap = 0.5;
                break;
            default:
                break;
        }
    }
    if (hasParameter("locomotiveLength")) {
        locomotiveLength = StringUtils::toDouble(getParameter("locomotiveLength"));
        parametersSet |= VTYPEPARS_LOCOMOTIVE_LENGTH_SET;
    } else if (locomotiveLength < 0) {
        locomotiveLength = carriageLength;
    }
    if (hasParameter("carriageGap")) {
        carriageGap = StringUtils::toDouble(getParameter("carriageGap"));
        parametersSet |= VTYPEPARS_CARRIAGE_GAP_SET;
    }
    if (hasParameter("carriageDoors")) {
        carriageDoors = StringUtils::toInt(getParameter("carriageDoors"));
        parametersSet |= VTYPEPARS_CARRIAGE_DOORS_SET;
    }
    if (hasParameter("frontSeatPos")) {
        frontSeatPos = StringUtils::toDouble(getParameter("frontSeatPos"));
        parametersSet |= VTYPEPARS_FRONT_SEAT_POS_SET;
    } else {
        switch (shape) {
            case SUMOVehicleShape::SHIP:
                frontSeatPos = 5;
                break;
            case SUMOVehicleShape::DELIVERY:
                frontSeatPos = 1.2;
                break;
            case SUMOVehicleShape::BICYCLE:
                frontSeatPos = 0.6;
                break;
            case SUMOVehicleShape::MOPED:
            case SUMOVehicleShape::MOTORCYCLE:
                frontSeatPos = 0.9;
                break;
            case SUMOVehicleShape::BUS:
            case SUMOVehicleShape::BUS_COACH:
            case SUMOVehicleShape::BUS_FLEXIBLE:
            case SUMOVehicleShape::BUS_TROLLEY:
                frontSeatPos = 0.5;
                break;
            case SUMOVehicleShape::TRUCK:
            case SUMOVehicleShape::TRUCK_1TRAILER:
            case SUMOVehicleShape::TRUCK_SEMITRAILER:
                frontSeatPos = 0.8;
                break;
            default:
                break;
        }
    }

    if (hasParameter("seatingWidth")) {
        seatingWidth = StringUtils::toDouble(getParameter("seatingWidth"));
        parametersSet |= VTYPEPARS_SEATING_WIDTH_SET;
    }
}


double
SUMOVTypeParameter::getDefaultAccel(const SUMOVehicleClass vc) {
    switch (vc) {
        case SVC_PEDESTRIAN:
        case SVC_WHEELCHAIR:
            return 1.5;
        case SVC_BICYCLE:
        case SVC_SCOOTER:
            return 1.2;
        case SVC_MOTORCYCLE:
            return 6.;
        case SVC_MOPED:
            return 1.1;
        case SVC_TRUCK:
            return 1.3;
        case SVC_TRAILER:
            return 1.1;
        case SVC_BUS:
            return 1.2;
        case SVC_COACH:
            return 2.;
        case SVC_TRAM:
            return 1.;
        case SVC_RAIL_URBAN:
            return 1.;
        case SVC_RAIL:
            return 0.25;
        case SVC_RAIL_ELECTRIC:
        case SVC_RAIL_FAST:
            return 0.5;
        case SVC_SHIP:
            return 0.1;
        default:
            return 2.6;//2.9;
    }
}


double
SUMOVTypeParameter::getDefaultDecel(const SUMOVehicleClass vc) {
    switch (vc) {
        case SVC_PEDESTRIAN:
        case SVC_WHEELCHAIR:
            return 2.;
        case SVC_BICYCLE:
        case SVC_SCOOTER:
            return 3.;
        case SVC_MOPED:
            return 7.;
        case SVC_MOTORCYCLE:
            return 10.;
        case SVC_TRUCK:
        case SVC_TRAILER:
        case SVC_BUS:
        case SVC_COACH:
            return 4.;
        case SVC_TRAM:
        case SVC_RAIL_URBAN:
            return 3.;
        case SVC_RAIL:
        case SVC_RAIL_ELECTRIC:
        case SVC_RAIL_FAST:
            return 1.3;
        case SVC_SHIP:
            return 0.15;
        default:
            return 4.5;//7.5;
    }
}


double
SUMOVTypeParameter::getDefaultEmergencyDecel(const SUMOVehicleClass vc, double decel, double defaultOption) {
    if (defaultOption == VTYPEPARS_DEFAULT_EMERGENCYDECEL_DEFAULT) {
        double vcDecel;
        switch (vc) {
            case SVC_PEDESTRIAN:
            case SVC_WHEELCHAIR:
                vcDecel = 5.;
                break;
            case SVC_BICYCLE:
            case SVC_SCOOTER:
                vcDecel = 7.;
                break;
            case SVC_MOPED:
            case SVC_MOTORCYCLE:
                vcDecel = 10.;
                break;
            case SVC_TRUCK:
            case SVC_TRAILER:
            case SVC_BUS:
            case SVC_COACH:
            case SVC_TRAM:
            case SVC_RAIL_URBAN:
                vcDecel = 7.;
                break;
            case SVC_RAIL:
            case SVC_RAIL_ELECTRIC:
            case SVC_RAIL_FAST:
                vcDecel = 5.;
                break;
            case SVC_SHIP:
                vcDecel = 1.;
                break;
            default:
                vcDecel = 9.;
        }
        return MAX2(decel, vcDecel);
    } else if (defaultOption == VTYPEPARS_DEFAULT_EMERGENCYDECEL_DECEL) {
        return decel;
    } else {
        // value already checked in MSFrame::checkOptions
        return MAX2(decel, defaultOption);
    }
}



double
SUMOVTypeParameter::getDefaultImperfection(const SUMOVehicleClass vc) {
    switch (vc) {
        case SVC_TRAM:
        case SVC_RAIL_URBAN:
        case SVC_RAIL:
        case SVC_RAIL_ELECTRIC:
        case SVC_RAIL_FAST:
        case SVC_SHIP:
            return 0.;
        default:
            return 0.5;
    }
}

const SUMOVTypeParameter&
SUMOVTypeParameter::getDefault() {
    static SUMOVTypeParameter defaultParams("");
    return defaultParams;
}

bool
SUMOVTypeParameter::parseLatAlignment(const std::string& val, double& lao, LatAlignmentDefinition& lad) {
    bool ok = true;
    lao = 0.0;
    lad = LatAlignmentDefinition::GIVEN;
    if (val == "right") {
        lad = LatAlignmentDefinition::RIGHT;
    } else if (val == "center") {
        lad = LatAlignmentDefinition::CENTER;
    } else if (val == "arbitrary") {
        lad = LatAlignmentDefinition::ARBITRARY;
    } else if (val == "nice") {
        lad = LatAlignmentDefinition::NICE;
    } else if (val == "compact") {
        lad = LatAlignmentDefinition::COMPACT;
    } else if (val == "left") {
        lad = LatAlignmentDefinition::LEFT;
    } else {
        try {
            lao = StringUtils::toDouble(val);
        } catch (...) {
            ok = false;
        }
    }
    return ok;
}


SUMOTime
SUMOVTypeParameter::getTimeToTeleport(SUMOTime defaultValue) const {
    return timeToTeleport == TTT_UNSET ? defaultValue : timeToTeleport;
}

SUMOTime
SUMOVTypeParameter::getTimeToTeleportBidi(SUMOTime defaultValue) const {
    return timeToTeleportBidi == TTT_UNSET ? defaultValue : timeToTeleportBidi;
}

/****************************************************************************/
