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
/// @file    SUMOVehicleParameter.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Tue, 31.03.2009
///
// Structure representing possible vehicle parameter
/****************************************************************************/
#include <config.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/StringTokenizer.h>
#include <utils/common/StringUtils.h>
#include <utils/common/ToString.h>
#include <utils/common/RandHelper.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/options/OptionsCont.h>

#include "SUMOVehicleParameter.h"

// ===========================================================================
// member method definitions
// ===========================================================================

SUMOVehicleParameter::SUMOVehicleParameter()
    : tag(SUMO_TAG_NOTHING), vtypeid(DEFAULT_VTYPE_ID), color(RGBColor::DEFAULT_COLOR),
      depart(-1), departProcedure(DepartDefinition::GIVEN),
      departLane(0), departLaneProcedure(DepartLaneDefinition::DEFAULT),
      departPos(0), departPosProcedure(DepartPosDefinition::DEFAULT),
      departPosLat(0), departPosLatProcedure(DepartPosLatDefinition::DEFAULT),
      departSpeed(-1), departSpeedProcedure(DepartSpeedDefinition::DEFAULT),
      departEdge(0), departEdgeProcedure(RouteIndexDefinition::DEFAULT),
      arrivalLane(0), arrivalLaneProcedure(ArrivalLaneDefinition::DEFAULT),
      arrivalPos(0), arrivalPosProcedure(ArrivalPosDefinition::DEFAULT),
      arrivalPosLat(0), arrivalPosLatProcedure(ArrivalPosLatDefinition::DEFAULT),
      arrivalSpeed(-1), arrivalSpeedProcedure(ArrivalSpeedDefinition::DEFAULT),
      arrivalEdge(-1), arrivalEdgeProcedure(RouteIndexDefinition::DEFAULT),
      repetitionNumber(-1),
      repetitionsDone(-1),
      repetitionOffset(-1),
      repetitionTotalOffset(0),
      repetitionProbability(-1),
      poissonRate(0),
      repetitionEnd(-1),
      line(), fromTaz(), toTaz(), personNumber(0), containerNumber(0),
      speedFactor(-1),
      calibratorSpeed(-1),
      insertionChecks((int)InsertionCheck::ALL),
      parametersSet(0)
{ }


SUMOVehicleParameter::~SUMOVehicleParameter() {
}


bool
SUMOVehicleParameter::defaultOptionOverrides(const OptionsCont& oc, const std::string& optionName) const {
    return oc.exists(optionName) && oc.isSet(optionName) && oc.getBool("defaults-override");
}


void
SUMOVehicleParameter::write(OutputDevice& dev, const OptionsCont& oc, const SumoXMLTag altTag, const std::string& typeID) const {
    if (!id.empty()) {
        // only used by calibrator flows
        dev.openTag(altTag).writeAttr(SUMO_ATTR_ID, id);
    }
    if (typeID == "") {
        if (wasSet(VEHPARS_VTYPE_SET)) {
            dev.writeAttr(SUMO_ATTR_TYPE, vtypeid);
        }
    } else {
        dev.writeAttr(SUMO_ATTR_TYPE, typeID);
    }
    // write depart depending of tag
    if (altTag == SUMO_TAG_FLOW
            || altTag == SUMO_TAG_PERSONFLOW
            || altTag == SUMO_TAG_CONTAINERFLOW
            || altTag == GNE_TAG_FLOW_ROUTE
            || altTag == GNE_TAG_FLOW_WITHROUTE
            || altTag == SUMO_TAG_FLOWSTATE) {
        dev.writeAttr(SUMO_ATTR_BEGIN, getDepart());
    } else {
        dev.writeAttr(SUMO_ATTR_DEPART, getDepart());
    }
    // optional parameter
    //  departlane
    if (wasSet(VEHPARS_DEPARTLANE_SET) && !defaultOptionOverrides(oc, "departlane")) {
        dev.writeNonEmptyAttr(SUMO_ATTR_DEPARTLANE, getDepartLane());
    } else if (oc.exists("departlane") && oc.isSet("departlane")) {
        dev.writeNonEmptyAttr(SUMO_ATTR_DEPARTLANE, oc.getString("departlane"));
    }
    //  departpos
    if (wasSet(VEHPARS_DEPARTPOS_SET) && !defaultOptionOverrides(oc, "departpos")) {
        dev.writeNonEmptyAttr(SUMO_ATTR_DEPARTPOS, getDepartPos());
    } else if (oc.exists("departpos") && oc.isSet("departpos")) {
        dev.writeNonEmptyAttr(SUMO_ATTR_DEPARTPOS, oc.getString("departpos"));
    }
    //  departPosLat
    if (wasSet(VEHPARS_DEPARTPOSLAT_SET)) {
        dev.writeNonEmptyAttr(SUMO_ATTR_DEPARTPOS_LAT, getDepartPosLat());
    }
    //  departspeed
    if (wasSet(VEHPARS_DEPARTSPEED_SET) && !defaultOptionOverrides(oc, "departspeed")) {
        dev.writeNonEmptyAttr(SUMO_ATTR_DEPARTSPEED, getDepartSpeed());
    } else if (oc.exists("departspeed") && oc.isSet("departspeed")) {
        dev.writeNonEmptyAttr(SUMO_ATTR_DEPARTSPEED, oc.getString("departspeed"));
    }
    //  departedge
    if (wasSet(VEHPARS_DEPARTEDGE_SET) && !defaultOptionOverrides(oc, "departedge")) {
        dev.writeNonEmptyAttr(SUMO_ATTR_DEPARTEDGE, getDepartEdge());
    } else if (oc.exists("departedge") && oc.isSet("departedge")) {
        dev.writeNonEmptyAttr(SUMO_ATTR_DEPARTEDGE, oc.getString("departedge"));
    }
    //  arrivallane
    if (wasSet(VEHPARS_ARRIVALLANE_SET) && !defaultOptionOverrides(oc, "arrivallane")) {
        dev.writeNonEmptyAttr(SUMO_ATTR_ARRIVALLANE, getArrivalLane());
    } else if (oc.exists("arrivallane") && oc.isSet("arrivallane")) {
        dev.writeNonEmptyAttr(SUMO_ATTR_ARRIVALLANE, oc.getString("arrivallane"));
    }
    //  arrivalpos
    if (wasSet(VEHPARS_ARRIVALPOS_SET) && !defaultOptionOverrides(oc, "arrivalpos")) {
        dev.writeNonEmptyAttr(SUMO_ATTR_ARRIVALPOS, getArrivalPos());
    } else if (oc.exists("arrivalpos") && oc.isSet("arrivalpos")) {
        dev.writeNonEmptyAttr(SUMO_ATTR_ARRIVALPOS, oc.getString("arrivalpos"));
    }
    //  arrivalPosLat
    if (wasSet(VEHPARS_ARRIVALPOSLAT_SET)) {
        dev.writeNonEmptyAttr(SUMO_ATTR_ARRIVALPOS_LAT, getArrivalPosLat());
    }
    //  arrivalspeed
    if (wasSet(VEHPARS_ARRIVALSPEED_SET) && !defaultOptionOverrides(oc, "arrivalspeed")) {
        dev.writeNonEmptyAttr(SUMO_ATTR_ARRIVALSPEED, getArrivalSpeed());
    } else if (oc.exists("arrivalspeed") && oc.isSet("arrivalspeed")) {
        dev.writeNonEmptyAttr(SUMO_ATTR_ARRIVALSPEED, oc.getString("arrivalspeed"));
    }
    //  arrivalEdge
    if (wasSet(VEHPARS_ARRIVALEDGE_SET) && !defaultOptionOverrides(oc, "arrivaledge") && arrivalEdge >= 0) {
        dev.writeNonEmptyAttr(SUMO_ATTR_ARRIVALEDGE, getArrivalEdge());
    } else if (oc.exists("arrivaledge") && oc.isSet("arrivaledge")) {
        dev.writeNonEmptyAttr(SUMO_ATTR_ARRIVALEDGE, oc.getString("arrivaledge"));
    }
    // color
    if (wasSet(VEHPARS_COLOR_SET)) {
        dev.writeAttr(SUMO_ATTR_COLOR, color);
    }
    // line
    if (wasSet(VEHPARS_LINE_SET)) {
        dev.writeAttr(SUMO_ATTR_LINE, line);
    }
    // from TAZ
    if (wasSet(VEHPARS_FROM_TAZ_SET)) {
        dev.writeAttr(SUMO_ATTR_FROM_TAZ, fromTaz);
    }
    // to TAZ
    if (wasSet(VEHPARS_TO_TAZ_SET)) {
        dev.writeAttr(SUMO_ATTR_TO_TAZ, toTaz);
    }
    // person number
    if (wasSet(VEHPARS_PERSON_NUMBER_SET)) {
        dev.writeAttr(SUMO_ATTR_PERSON_NUMBER, personNumber);
    }
    // container number
    if (wasSet(VEHPARS_CONTAINER_NUMBER_SET)) {
        dev.writeAttr(SUMO_ATTR_CONTAINER_NUMBER, containerNumber);
    }
    // individual speedFactor
    if (wasSet(VEHPARS_SPEEDFACTOR_SET)) {
        // might be saving state with custom precision
        const int precision = dev.precision();
        dev.setPrecision(MAX2(gPrecisionRandom, precision));
        dev.writeAttr(SUMO_ATTR_SPEEDFACTOR, speedFactor);
        dev.setPrecision(precision);
    }
    // speed (only used by calibrators)
    if (wasSet(VEHPARS_CALIBRATORSPEED_SET)) {
        dev.writeAttr(SUMO_ATTR_SPEED, calibratorSpeed);
    }
    // insertionChecks
    if (wasSet(VEHPARS_INSERTION_CHECKS_SET) && insertionChecks != (int)InsertionCheck::ALL) {
        std::vector<std::string> checks;
        if (insertionChecks == (int)InsertionCheck::NONE) {
            checks.push_back(toString(InsertionCheck::NONE));
        } else {
            for (auto it : SUMOXMLDefinitions::InsertionChecks.getValues()) {
                if (((int)it & insertionChecks) != 0) {
                    checks.push_back(toString(it));
                }
            }
        }
        dev.writeAttr(SUMO_ATTR_INSERTIONCHECKS, checks);
    }
    // parking access rights
    if (wasSet(VEHPARS_PARKING_BADGES_SET)) {
        dev.writeNonEmptyAttr(SUMO_ATTR_PARKING_BADGES, joinToString(parkingBadges, " "));
    }
}


void
SUMOVehicleParameter::Stop::write(OutputDevice& dev, const bool close, const bool writeTagAndParents) const {
    if (writeTagAndParents) {
        dev.openTag(SUMO_TAG_STOP);
        if (busstop != "") {
            dev.writeAttr(SUMO_ATTR_BUS_STOP, busstop);
        }
        if (containerstop != "") {
            dev.writeAttr(SUMO_ATTR_CONTAINER_STOP, containerstop);
        }
        if (chargingStation != "") {
            dev.writeAttr(SUMO_ATTR_CHARGING_STATION, chargingStation);
        }
        if (parkingarea != "") {
            dev.writeAttr(SUMO_ATTR_PARKING_AREA, parkingarea);
        }
        if ((busstop == "") && (containerstop == "") && (parkingarea == "") && (chargingStation == "")) {
            if (lane != "") {
                dev.writeAttr(SUMO_ATTR_LANE, lane);
            } else {
                dev.writeAttr(SUMO_ATTR_EDGE, edge);
            }
            if ((parametersSet & STOP_START_SET) != 0) {
                dev.writeAttr(SUMO_ATTR_STARTPOS, startPos);
            }
            if ((parametersSet & STOP_END_SET) != 0) {
                dev.writeAttr(SUMO_ATTR_ENDPOS, endPos);
            }
        }
    }
    if (index > 0) {
        dev.writeAttr(SUMO_ATTR_INDEX, index);
    }
    if ((parametersSet & STOP_POSLAT_SET) != 0 && posLat != INVALID_DOUBLE) {
        dev.writeAttr(SUMO_ATTR_POSITION_LAT, posLat);
    }
    if ((parametersSet & STOP_ARRIVAL_SET) && (arrival >= 0)) {
        dev.writeAttr(SUMO_ATTR_ARRIVAL, time2string(arrival));
    }
    if ((parametersSet & STOP_DURATION_SET) && (duration >= 0)) {
        dev.writeAttr(SUMO_ATTR_DURATION, time2string(duration));
    }
    if ((parametersSet & STOP_UNTIL_SET) && (until >= 0)) {
        dev.writeAttr(SUMO_ATTR_UNTIL, time2string(until));
    }
    if ((parametersSet & STOP_STARTED_SET) && (started >= 0)) {
        dev.writeAttr(SUMO_ATTR_STARTED, time2string(started));
    }
    if ((parametersSet & STOP_ENDED_SET) && (ended >= 0)) {
        dev.writeAttr(SUMO_ATTR_ENDED, time2string(ended));
    }
    if ((parametersSet & STOP_EXTENSION_SET) && (extension >= 0)) {
        dev.writeAttr(SUMO_ATTR_EXTENSION, time2string(extension));
    }
    if ((parametersSet & STOP_TRIGGER_SET) != 0) {
        const std::vector<std::string> triggers = getTriggers();
        if (triggers.size() > 0) {
            dev.writeAttr(SUMO_ATTR_TRIGGERED, triggers);
        }
    }
    if ((parametersSet & STOP_PARKING_SET) != 0) {
        dev.writeAttr(SUMO_ATTR_PARKING, parking);
    }
    if ((parametersSet & STOP_EXPECTED_SET) != 0 && awaitedPersons.size() > 0) {
        dev.writeAttr(SUMO_ATTR_EXPECTED, awaitedPersons);
    }
    if ((parametersSet & STOP_PERMITTED_SET) != 0 && permitted.size() > 0) {
        dev.writeAttr(SUMO_ATTR_PERMITTED, permitted);
    }
    if ((parametersSet & STOP_EXPECTED_CONTAINERS_SET) != 0 && awaitedContainers.size() > 0) {
        dev.writeAttr(SUMO_ATTR_EXPECTED_CONTAINERS, awaitedContainers);
    }
    if ((parametersSet & STOP_TRIP_ID_SET) != 0) {
        dev.writeAttr(SUMO_ATTR_TRIP_ID, tripId);
    }
    if ((parametersSet & STOP_LINE_SET) != 0) {
        dev.writeAttr(SUMO_ATTR_LINE, line);
    }
    if ((parametersSet & STOP_SPLIT_SET) != 0) {
        dev.writeAttr(SUMO_ATTR_SPLIT, split);
    }
    if ((parametersSet & STOP_JOIN_SET) != 0) {
        dev.writeAttr(SUMO_ATTR_JOIN, join);
    }
    if ((parametersSet & STOP_SPEED_SET) != 0) {
        dev.writeAttr(SUMO_ATTR_SPEED, speed);
    }
    if ((parametersSet & STOP_ONDEMAND_SET) != 0) {
        dev.writeAttr(SUMO_ATTR_ONDEMAND, onDemand);
    }
    if ((parametersSet & STOP_JUMP_SET) != 0 && jump >= 0) {
        dev.writeAttr(SUMO_ATTR_JUMP, time2string(jump));
    }
    if (collision) {
        dev.writeAttr(SUMO_ATTR_COLLISION, collision);
    }
    // only write friendly position if is true
    if (friendlyPos) {
        dev.writeAttr(SUMO_ATTR_FRIENDLY_POS, friendlyPos);
    }
    // only write act type if isn't empty
    if (!actType.empty()) {
        dev.writeAttr(SUMO_ATTR_ACTTYPE, actType);
    }
    if (close) {
        // the user is closing the stop it is responsible for writing params
        writeParams(dev);
        dev.closeTag();
    }
}

std::vector<std::string>
SUMOVehicleParameter::Stop::getStoppingPlaceIDs() const {
    std::vector<std::string> result;
    if (busstop != "") {
        result.push_back(busstop);
    }
    if (containerstop != "") {
        result.push_back(containerstop);
    }
    if (chargingStation != "") {
        result.push_back(chargingStation);
    }
    if (parkingarea != "") {
        result.push_back(parkingarea);
    }
    return result;
}

bool
SUMOVehicleParameter::parseDepart(const std::string& val, const std::string& element, const std::string& id,
                                  SUMOTime& depart, DepartDefinition& dd, std::string& error, const std::string& attr) {
    if (val == "triggered") {
        dd = DepartDefinition::TRIGGERED;
    } else if (val == "containerTriggered") {
        dd = DepartDefinition::CONTAINER_TRIGGERED;
    } else if (val == "now") {
        // only used via TraCI. depart must be set by the calling code
        dd = DepartDefinition::NOW;
    } else if (val == "split") {
        dd = DepartDefinition::SPLIT;
    } else if (val == "begin") {
        dd = DepartDefinition::BEGIN;
    } else {
        try {
            depart = string2time(val);
            dd = DepartDefinition::GIVEN;
            if (depart < 0) {
                error = "Negative " + attr + " time in the definition of " + element + " '" + id + "'.";
                return false;
            }
        } catch (...) {
            if (id.empty()) {
                error = "Invalid " + attr + " time for " + element + ". Must be one of (\"triggered\", \"containerTriggered\", \"now\", or a float >= 0)";
            } else {
                error = "Invalid " + attr + " time for " + element + " '" + id + "';\n must be one of (\"triggered\", \"containerTriggered\", \"now\", or a float >= 0)";
            }
            return false;
        }
    }
    return true;
}


bool
SUMOVehicleParameter::parseDepartLane(const std::string& val, const std::string& element, const std::string& id,
                                      int& lane, DepartLaneDefinition& dld, std::string& error) {
    bool ok = true;
    lane = 0;
    dld = DepartLaneDefinition::GIVEN;
    if (val == "random") {
        dld = DepartLaneDefinition::RANDOM;
    } else if (val == "free") {
        dld = DepartLaneDefinition::FREE;
    } else if (val == "allowed") {
        dld = DepartLaneDefinition::ALLOWED_FREE;
    } else if (val == "best") {
        dld = DepartLaneDefinition::BEST_FREE;
    } else if (val == "best_prob") {
        dld = DepartLaneDefinition::BEST_PROB;
    } else if (val == "first") {
        dld = DepartLaneDefinition::FIRST_ALLOWED;
    } else {
        try {
            lane = StringUtils::toInt(val);
            if (lane < 0) {
                ok = false;
            }
        } catch (...) {
            ok = false;
        }
    }
    if (!ok) {
        if (id.empty()) {
            error = "Invalid departLane definition for " + element + ". Must be one of (\"random\", \"free\", \"allowed\", \"best\", \"best_prob\", \"first\", or an int>=0)";
        } else {
            error = "Invalid departLane definition for " + element + " '" + id + "';\n must be one of (\"random\", \"free\", \"allowed\", \"best\", \"best_prob\", \"first\", or an int>=0)";
        }
    }
    return ok;
}


bool
SUMOVehicleParameter::parseDepartPos(const std::string& val, const std::string& element, const std::string& id,
                                     double& pos, DepartPosDefinition& dpd, std::string& error) {
    bool ok = true;
    pos = 0.;
    dpd = DepartPosDefinition::GIVEN;
    if (val == "random") {
        dpd = DepartPosDefinition::RANDOM;
    } else if (val == "random_free") {
        dpd = DepartPosDefinition::RANDOM_FREE;
    } else if (val == "random_location") {
        dpd = DepartPosDefinition::RANDOM_LOCATION;
    } else if (val == "free") {
        dpd = DepartPosDefinition::FREE;
    } else if (val == "base") {
        dpd = DepartPosDefinition::BASE;
    } else if (val == "last") {
        dpd = DepartPosDefinition::LAST;
    } else if (val == "splitFront") {
        dpd = DepartPosDefinition::SPLIT_FRONT;
    } else if (val == "stop") {
        dpd = DepartPosDefinition::STOP;
    } else {
        try {
            pos = StringUtils::toDouble(val);
        } catch (...) {
            ok = false;
        }
    }
    if (!ok) {
        if (id.empty()) {
            error = "Invalid departPos definition for " + element + ". Must be one of (\"random\", \"random_free\", \"free\", \"base\", \"last\" or a float)";
        } else {
            error = "Invalid departPos definition for " + element + " '" + id + "';\n must be one of (\"random\", \"random_free\", \"free\", \"base\", \"last\" or a float)";
        }
    }
    return ok;
}


bool
SUMOVehicleParameter::parseDepartPosLat(const std::string& val, const std::string& element, const std::string& id,
                                        double& pos, DepartPosLatDefinition& dpd, std::string& error) {
    bool ok = true;
    pos = 0.;
    dpd = DepartPosLatDefinition::GIVEN;
    if (val == "random") {
        dpd = DepartPosLatDefinition::RANDOM;
    } else if (val == "random_free") {
        dpd = DepartPosLatDefinition::RANDOM_FREE;
    } else if (val == "free") {
        dpd = DepartPosLatDefinition::FREE;
    } else if (val == "right") {
        dpd = DepartPosLatDefinition::RIGHT;
    } else if (val == "center") {
        dpd = DepartPosLatDefinition::CENTER;
    } else if (val == "left") {
        dpd = DepartPosLatDefinition::LEFT;
    } else {
        try {
            pos = StringUtils::toDouble(val);
        } catch (...) {
            ok = false;
        }
    }
    if (!ok) {
        if (id.empty()) {
            error = "Invalid departPosLat definition for " + element + ". Must be one of (\"random\", \"random_free\", \"free\", \"right\", \"center\", \"left\", or a float)";
        } else {
            error = "Invalid departPosLat definition for " + element + " '" + id + "';\n must be one of (\"random\", \"random_free\", \"free\", \"right\", \"center\", \"left\", or a float)";
        }
    }
    return ok;
}


bool
SUMOVehicleParameter::parseDepartSpeed(const std::string& val, const std::string& element, const std::string& id,
                                       double& speed, DepartSpeedDefinition& dsd, std::string& error) {
    bool ok = true;
    speed = -1.;
    dsd = DepartSpeedDefinition::GIVEN;
    if (val == "random") {
        dsd = DepartSpeedDefinition::RANDOM;
    } else if (val == "max") {
        dsd = DepartSpeedDefinition::MAX;
    } else if (val == "desired") {
        dsd = DepartSpeedDefinition::DESIRED;
    } else if (val == "speedLimit") {
        dsd = DepartSpeedDefinition::LIMIT;
    } else if (val == "last") {
        dsd = DepartSpeedDefinition::LAST;
    } else if (val == "avg") {
        dsd = DepartSpeedDefinition::AVG;
    } else {
        try {
            speed = StringUtils::toDouble(val);
            if (speed < 0) {
                ok = false;
            }
        } catch (...) {
            ok = false;
        }
    }
    if (!ok) {
        if (id.empty()) {
            error = "Invalid departSpeed definition for " + element + ". Must be one of (\"random\", \"max\", or a float>=0)";
        } else {
            error = "Invalid departSpeed definition for " + element + " '" + id + "';\n must be one of (\"random\", \"max\", or a float>=0)";
        }
    }
    return ok;
}


bool
SUMOVehicleParameter::parseRouteIndex(const std::string& val, const std::string& element, const std::string& id,
                                      const SumoXMLAttr attr, int& edgeIndex, RouteIndexDefinition& rid, std::string& error) {
    bool ok = true;
    edgeIndex = -1;
    rid = RouteIndexDefinition::GIVEN;
    if (val == "random") {
        rid = RouteIndexDefinition::RANDOM;
    } else {
        try {
            edgeIndex = StringUtils::toInt(val);
            if (edgeIndex < 0) {
                ok = false;
            }
        } catch (...) {
            ok = false;
        }
    }
    if (!ok) {
        if (id.empty()) {
            error = "Invalid " + toString(attr) + " definition for " + element + ". Must be one of (\"random\", \"free\", or an int>=0)";
        } else {
            error = "Invalid " + toString(attr) + " definition for " + element + " '" + id + "';\n must be one of (\"random\", \"free\", or an int>=0)";
        }
    }
    return ok;
}


bool
SUMOVehicleParameter::parseArrivalLane(const std::string& val, const std::string& element, const std::string& id,
                                       int& lane, ArrivalLaneDefinition& ald, std::string& error) {
    bool ok = true;
    lane = 0;
    ald = ArrivalLaneDefinition::GIVEN;
    if (val == "current") {
        ald = ArrivalLaneDefinition::CURRENT;
    } else if (val == "random") {
        ald = ArrivalLaneDefinition::RANDOM;
    } else if (val == "first") {
        ald = ArrivalLaneDefinition::FIRST_ALLOWED;
    } else {
        try {
            lane = StringUtils::toInt(val);
            if (lane < 0) {
                ok = false;
            }
        } catch (...) {
            ok = false;
        }
    }
    if (!ok) {
        if (id.empty()) {
            error = "Invalid arrivalLane definition for " + element + ". Must be one of (\"current\", or an int>=0)";
        } else {
            error = "Invalid arrivalLane definition for " + element + " '" + id + "';\n must be one of (\"current\", or an int>=0)";
        }
    }
    return ok;
}


bool
SUMOVehicleParameter::parseArrivalPos(const std::string& val, const std::string& element, const std::string& id,
                                      double& pos, ArrivalPosDefinition& apd, std::string& error) {
    bool ok = true;
    pos = 0.;
    apd = ArrivalPosDefinition::GIVEN;
    if (val == "random") {
        apd = ArrivalPosDefinition::RANDOM;
    } else if (val == "center") {
        apd = ArrivalPosDefinition::CENTER;
    } else if (val == "max") {
        apd = ArrivalPosDefinition::MAX;
    } else {
        try {
            pos = StringUtils::toDouble(val);
        } catch (...) {
            ok = false;
        }
    }
    if (!ok) {
        if (id.empty()) {
            error = "Invalid arrivalPos definition for " + element + ". Must be one of (\"random\", \"max\", or a float)";
        } else {
            error = "Invalid arrivalPos definition for " + element + " '" + id + "';\n must be one of (\"random\", \"max\", or a float)";
        }
    }
    return ok;
}


bool
SUMOVehicleParameter::parseArrivalPosLat(const std::string& val, const std::string& element, const std::string& id,
        double& pos, ArrivalPosLatDefinition& apd, std::string& error) {
    bool ok = true;
    pos = 0.;
    apd = ArrivalPosLatDefinition::GIVEN;
    if (val == "right") {
        apd = ArrivalPosLatDefinition::RIGHT;
    } else if (val == "center") {
        apd = ArrivalPosLatDefinition::CENTER;
    } else if (val == "left") {
        apd = ArrivalPosLatDefinition::LEFT;
    } else {
        try {
            pos = StringUtils::toDouble(val);
        } catch (...) {
            ok = false;
        }
    }
    if (!ok) {
        if (id.empty()) {
            error = "Invalid arrivalPosLat definition for " + element + ". Must be one of (\"right\", \"center\", \"left\", or a float)";
        } else {
            error = "Invalid arrivalPosLat definition for " + element + " '" + id + "';\n must be one of (\"right\", \"center\", \"left\", or a float)";
        }
    }
    return ok;
}


bool
SUMOVehicleParameter::parseArrivalSpeed(const std::string& val, const std::string& element, const std::string& id,
                                        double& speed, ArrivalSpeedDefinition& asd, std::string& error) {
    bool ok = true;
    speed = -1.;
    asd = ArrivalSpeedDefinition::GIVEN;
    if (val == "current") {
        asd = ArrivalSpeedDefinition::CURRENT;
    } else {
        try {
            speed = StringUtils::toDouble(val);
            if (speed < 0) {
                ok = false;
            }
        } catch (...) {
            ok = false;
        }
    }
    if (!ok) {
        if (id.empty()) {
            error = "Invalid arrivalSpeed definition for " + element + ". Must be one of (\"current\", or a float>=0)";
        } else {
            error = "Invalid arrivalSpeed definition for " + element + " '" + id + "';\n must be one of (\"current\", or a float>=0)";
        }
    }
    return ok;
}


double
SUMOVehicleParameter::interpretEdgePos(double pos, double maximumValue, SumoXMLAttr attr, const std::string& id, bool silent) {
    if (pos < 0) {
        pos = maximumValue + pos;
    }
    if (pos > maximumValue && pos != std::numeric_limits<double>::infinity()) {
        if (!silent) {
            WRITE_WARNINGF(TL("Invalid % % given for %. Using edge end instead."), toString(attr), toString(pos), id);
        }
        pos = maximumValue;
    }
    return pos;
}


bool
SUMOVehicleParameter::parsePersonModes(const std::string& modes, const std::string& element, const std::string& id, SVCPermissions& modeSet, std::string& error) {
    // separte modes in different strings, and check if modes are valid
    for (StringTokenizer st(modes); st.hasNext();) {
        const std::string mode = st.next();
        if (mode == "car") {
            modeSet |= SVC_PASSENGER;
        } else if (mode == "taxi") {
            modeSet |= SVC_TAXI;
        } else if (mode == "bicycle") {
            modeSet |= SVC_BICYCLE;
        } else if (mode == "public") {
            modeSet |= SVC_BUS;
        } else {
            if (id.empty()) {
                error = "Unknown person mode '" + mode + "'. Must be a combination of (\"car\", \"taxi\", \"bicycle\" or \"public\")";
            } else {
                error = "Unknown person mode '" + mode + "' for " + element + " '" + id + "';\n must be a combination of (\"car\", \"taxi\", \"bicycle\" or \"public\")";
            }
            return false;
        }
    }
    return true;
}


void
SUMOVehicleParameter::parseStopTriggers(const std::vector<std::string>& triggers, bool expectTrigger, Stop& stop) {
    if (triggers.size() == 0 && expectTrigger) {
        stop.triggered = true;
    }
    for (std::string val : triggers) {
        if (val == toString(SUMO_TAG_PERSON)) {
            stop.triggered = true;
        } else if (val == toString(SUMO_TAG_CONTAINER)) {
            stop.containerTriggered = true;
        } else if (val == toString(SUMO_ATTR_JOIN)) {
            stop.joinTriggered = true;
        } else {
            try {
                stop.triggered = StringUtils::toBool(val);
            } catch (BoolFormatException&) {
                WRITE_ERROR(TL("Value of stop attribute 'trigger' must be 'person', 'container', 'join' or a boolean"));
            }
        }
    }
}


ParkingType
SUMOVehicleParameter::parseParkingType(const std::string& value) {
    if (value == toString(ParkingType::OPPORTUNISTIC)) {
        return ParkingType::OPPORTUNISTIC;
    } else {
        return StringUtils::toBool(value) ? ParkingType::OFFROAD : ParkingType::ONROAD;
    }
}


std::vector<std::string>
SUMOVehicleParameter::Stop::getTriggers() const {
    std::vector<std::string> triggers;
    if (triggered) {
        triggers.push_back(toString(SUMO_TAG_PERSON));
    }
    if (containerTriggered) {
        triggers.push_back(toString(SUMO_TAG_CONTAINER));
    }
    if (joinTriggered) {
        triggers.push_back(toString(SUMO_ATTR_JOIN));
    }
    return triggers;
}

int
SUMOVehicleParameter::Stop::getFlags() const {
    return (((parking == ParkingType::OFFROAD) ? 1 : 0) +
            (triggered ? 2 : 0) +
            (containerTriggered ? 4 : 0) +
            (busstop != "" ? 8 : 0) +
            (containerstop != "" ? 16 : 0) +
            (chargingStation != "" ? 32 : 0) +
            (parkingarea != "" ? 64 : 0) +
            (overheadWireSegment != "" ? 128 : 0));
}


std::string
SUMOVehicleParameter::getDepart() const {
    if (departProcedure == DepartDefinition::TRIGGERED) {
        return "triggered";
    } else if (departProcedure == DepartDefinition::CONTAINER_TRIGGERED) {
        return "containerTriggered";
//    } else if (departProcedure == DepartDefinition::NOW) {  // TODO check whether this is useful in XML input (currently TraCI only)
//        return "now";
    } else if (departProcedure == DepartDefinition::SPLIT) {
        return "split";
    } else if (departProcedure == DepartDefinition::BEGIN) {
        return "begin";
    } else {
        return time2string(depart);
    }
}


std::string
SUMOVehicleParameter::getDepartLane() const {
    std::string val;
    switch (departLaneProcedure) {
        case DepartLaneDefinition::GIVEN:
            val = toString(departLane);
            break;
        case DepartLaneDefinition::RANDOM:
            val = "random";
            break;
        case DepartLaneDefinition::FREE:
            val = "free";
            break;
        case DepartLaneDefinition::ALLOWED_FREE:
            val = "allowed";
            break;
        case DepartLaneDefinition::BEST_FREE:
            val = "best";
            break;
        case DepartLaneDefinition::BEST_PROB:
            val = "best_prob";
            break;
        case DepartLaneDefinition::FIRST_ALLOWED:
            val = "first";
            break;
        case DepartLaneDefinition::DEFAULT:
        default:
            break;
    }
    return val;
}


std::string
SUMOVehicleParameter::getDepartPos() const {
    std::string val;
    switch (departPosProcedure) {
        case DepartPosDefinition::GIVEN:
            val = toString(departPos);
            break;
        case DepartPosDefinition::GIVEN_VEHROUTE:
            val = StringUtils::pruneZeros(toString(departPos, MAX2(gPrecisionRandom, gPrecision)), 2);
            break;
        case DepartPosDefinition::RANDOM:
            val = "random";
            break;
        case DepartPosDefinition::RANDOM_FREE:
            val = "random_free";
            break;
        case DepartPosDefinition::RANDOM_LOCATION:
            val = "random_location";
            break;
        case DepartPosDefinition::FREE:
            val = "free";
            break;
        case DepartPosDefinition::LAST:
            val = "last";
            break;
        case DepartPosDefinition::BASE:
            val = "base";
            break;
        case DepartPosDefinition::SPLIT_FRONT:
            val = "splitFront";
            break;
        case DepartPosDefinition::STOP:
            val = "stop";
            break;
        case DepartPosDefinition::DEFAULT:
        default:
            break;
    }
    return val;
}


std::string
SUMOVehicleParameter::getDepartPosLat() const {
    std::string val;
    switch (departPosLatProcedure) {
        case DepartPosLatDefinition::GIVEN:
            val = toString(departPosLat);
            break;
        case DepartPosLatDefinition::GIVEN_VEHROUTE:
            val = StringUtils::pruneZeros(toString(departPosLat, MAX2(gPrecisionRandom, gPrecision)), 2);
            break;
        case DepartPosLatDefinition::RANDOM:
            val = "random";
            break;
        case DepartPosLatDefinition::RANDOM_FREE:
            val = "random_free";
            break;
        case DepartPosLatDefinition::FREE:
            val = "free";
            break;
        case DepartPosLatDefinition::RIGHT:
            val = "right";
            break;
        case DepartPosLatDefinition::CENTER:
            val = "center";
            break;
        case DepartPosLatDefinition::LEFT:
            val = "left";
            break;
        case DepartPosLatDefinition::DEFAULT:
        default:
            break;
    }
    return val;
}


std::string
SUMOVehicleParameter::getDepartSpeed() const {
    std::string val;
    switch (departSpeedProcedure) {
        case DepartSpeedDefinition::GIVEN:
            val = toString(departSpeed);
            break;
        case DepartSpeedDefinition::GIVEN_VEHROUTE:
            val = StringUtils::pruneZeros(toString(departSpeed, MAX2(gPrecisionRandom, gPrecision)), 2);
            break;
        case DepartSpeedDefinition::RANDOM:
            val = "random";
            break;
        case DepartSpeedDefinition::MAX:
            val = "max";
            break;
        case DepartSpeedDefinition::DESIRED:
            val = "desired";
            break;
        case DepartSpeedDefinition::LIMIT:
            val = "speedLimit";
            break;
        case DepartSpeedDefinition::LAST:
            val = "last";
            break;
        case DepartSpeedDefinition::AVG:
            val = "avg";
            break;
        case DepartSpeedDefinition::DEFAULT:
        default:
            break;
    }
    return val;
}


std::string
SUMOVehicleParameter::getDepartEdge() const {
    std::string val;
    switch (departEdgeProcedure) {
        case RouteIndexDefinition::GIVEN:
            val = toString(departEdge);
            break;
        case RouteIndexDefinition::RANDOM:
            val = "random";
            break;
        case RouteIndexDefinition::DEFAULT:
        default:
            break;
    }
    return val;
}

std::string
SUMOVehicleParameter::getArrivalEdge() const {
    std::string val;
    switch (arrivalEdgeProcedure) {
        case RouteIndexDefinition::GIVEN:
            val = toString(arrivalEdge);
            break;
        case RouteIndexDefinition::RANDOM:
            val = "random";
            break;
        case RouteIndexDefinition::DEFAULT:
        default:
            break;
    }
    return val;
}




std::string
SUMOVehicleParameter::getArrivalLane() const {
    std::string val;
    switch (arrivalLaneProcedure) {
        case ArrivalLaneDefinition::GIVEN:
            val = toString(arrivalLane);
            break;
        case ArrivalLaneDefinition::CURRENT:
            val = "current";
            break;
        case ArrivalLaneDefinition::RANDOM:
            val = "random";
            break;
        case ArrivalLaneDefinition::FIRST_ALLOWED:
            val = "first";
            break;
        case ArrivalLaneDefinition::DEFAULT:
        default:
            break;
    }
    return val;
}


std::string
SUMOVehicleParameter::getArrivalPos() const {
    std::string val;
    switch (arrivalPosProcedure) {
        case ArrivalPosDefinition::GIVEN:
            val = toString(arrivalPos);
            break;
        case ArrivalPosDefinition::RANDOM:
            val = "random";
            break;
        case ArrivalPosDefinition::CENTER:
            val = "center";
            break;
        case ArrivalPosDefinition::MAX:
            val = "max";
            break;
        case ArrivalPosDefinition::DEFAULT:
        default:
            break;
    }
    return val;
}


std::string
SUMOVehicleParameter::getArrivalPosLat() const {
    std::string val;
    switch (arrivalPosLatProcedure) {
        case ArrivalPosLatDefinition::GIVEN:
            val = toString(arrivalPosLat);
            break;
        case ArrivalPosLatDefinition::RIGHT:
            val = "right";
            break;
        case ArrivalPosLatDefinition::CENTER:
            val = "center";
            break;
        case ArrivalPosLatDefinition::LEFT:
            val = "left";
            break;
        case ArrivalPosLatDefinition::DEFAULT:
        default:
            break;
    }
    return val;
}


std::string
SUMOVehicleParameter::getArrivalSpeed() const {
    std::string val;
    switch (arrivalSpeedProcedure) {
        case ArrivalSpeedDefinition::GIVEN:
            val = toString(arrivalSpeed);
            break;
        case ArrivalSpeedDefinition::CURRENT:
            val = "current";
            break;
        case ArrivalSpeedDefinition::DEFAULT:
        default:
            break;
    }
    return val;
}


void
SUMOVehicleParameter::incrementFlow(double scale, SumoRNG* rng) {
    repetitionsDone++;
    // equidistant or exponential offset (for poisson distributed arrivals)
    if (repetitionProbability < 0) {
        if (repetitionOffset >= 0) {
            repetitionTotalOffset += (SUMOTime)((double)repetitionOffset / scale);
        } else {
            assert(poissonRate > 0);
            // we need to cache this do avoid double generation of the rng in the TIME2STEPS macro
            const double r = RandHelper::randExp(poissonRate, rng);
            repetitionTotalOffset += TIME2STEPS(r / scale);
        }
    }
}


std::string
SUMOVehicleParameter::getInsertionChecks() const {
    if ((insertionChecks == 0) || (insertionChecks == (int)InsertionCheck::ALL)) {
        return SUMOXMLDefinitions::InsertionChecks.getString(InsertionCheck::ALL);
    } else {
        std::vector<std::string> insertionChecksStrs;
        const auto insertionCheckValues = SUMOXMLDefinitions::InsertionChecks.getValues();
        // iterate over values
        for (const auto insertionCheckValue : insertionCheckValues) {
            if ((insertionCheckValue != InsertionCheck::ALL) && (insertionChecks & (int)insertionCheckValue) != 0) {
                insertionChecksStrs.push_back(SUMOXMLDefinitions::InsertionChecks.getString(insertionCheckValue));
            }
        }
        return toString(insertionChecksStrs);
    }
}


bool
SUMOVehicleParameter::areInsertionChecksValid(const std::string& value) const {
    if (value.empty()) {
        return true;
    } else {
        // split value in substrinsg
        StringTokenizer valueStrs(value, " ");
        // iterate over values
        while (valueStrs.hasNext()) {
            if (!SUMOXMLDefinitions::InsertionChecks.hasString(valueStrs.next())) {
                return false;
            }
        }
        return true;
    }
}


int
SUMOVehicleParameter::parseInsertionChecks(const std::string& value) {
    // first reset insertionChecks
    int result = 0;
    if (value.empty()) {
        return (int)InsertionCheck::ALL;
    } else {
        // split value in substrinsg
        StringTokenizer insertionCheckStrs(value, " ");
        while (insertionCheckStrs.hasNext()) {
            std::string val = insertionCheckStrs.next();
            if (SUMOXMLDefinitions::InsertionChecks.hasString(val)) {
                result |= (int)SUMOXMLDefinitions::InsertionChecks.get(val);
            } else {
                throw InvalidArgument("Unknown value '" + val + "' in " + toString(SUMO_ATTR_INSERTIONCHECKS) + ".");
            }
        }
    }
    return result;
}

/****************************************************************************/
