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
/// @file    SUMOVehicleClass.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Walter Bamberger
/// @author  Laura Bieker-Walz
/// @date    2006-01-24
///
// Definitions of SUMO vehicle classes and helper functions
/****************************************************************************/
#include <config.h>

#include <string>
#include <map>
#include "SUMOVehicleClass.h"
#include <utils/common/StringUtils.h>
#include <utils/common/ToString.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/StringTokenizer.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/xml/SUMOSAXAttributes.h>


// ===========================================================================
// static members
// ===========================================================================

static StringBijection<SUMOVehicleClass>::Entry sumoVehicleClassStringInitializer[] = {
    {"ignoring",          SVC_IGNORING},
    {"private",           SVC_PRIVATE},
    {"public_emergency",  SVC_EMERGENCY}, // !!! deprecated
    {"emergency",         SVC_EMERGENCY},
    {"public_authority",  SVC_AUTHORITY}, // !!! deprecated
    {"authority",         SVC_AUTHORITY},
    {"public_army",       SVC_ARMY}, // !!! deprecated
    {"army",              SVC_ARMY},
    {"vip",               SVC_VIP},
    {"pedestrian",        SVC_PEDESTRIAN},
    {"passenger",         SVC_PASSENGER},
    {"hov",               SVC_HOV},
    {"taxi",              SVC_TAXI},
    {"public_transport",  SVC_BUS}, // !!! deprecated
    {"bus",               SVC_BUS},
    {"coach",             SVC_COACH},
    {"delivery",          SVC_DELIVERY},
    {"transport",         SVC_TRUCK}, // !!! deprecated
    {"truck",             SVC_TRUCK},
    {"trailer",           SVC_TRAILER},
    {"motorcycle",        SVC_MOTORCYCLE},
    {"moped",             SVC_MOPED},
    {"bicycle",           SVC_BICYCLE},
    {"evehicle",          SVC_E_VEHICLE},
    {"lightrail",         SVC_TRAM}, // !!! deprecated
    {"tram",              SVC_TRAM},
    {"cityrail",          SVC_RAIL_URBAN}, // !!! deprecated
    {"rail_urban",        SVC_RAIL_URBAN},
    {"rail_slow",         SVC_RAIL}, // !!! deprecated
    {"rail",              SVC_RAIL},
    {"rail_electric",     SVC_RAIL_ELECTRIC},
    {"rail_fast",         SVC_RAIL_FAST},
    {"ship",              SVC_SHIP},
    {"container",         SVC_CONTAINER},
    {"cable_car",         SVC_CABLE_CAR},
    {"subway",            SVC_SUBWAY},
    {"aircraft",          SVC_AIRCRAFT},
    {"wheelchair",        SVC_WHEELCHAIR},
    {"scooter",           SVC_SCOOTER},
    {"drone",             SVC_DRONE},
    {"custom1",           SVC_CUSTOM1},
    {"custom2",           SVC_CUSTOM2}
};


StringBijection<SUMOVehicleClass> SumoVehicleClassStrings(
    sumoVehicleClassStringInitializer, SVC_CUSTOM2, false);

// count only non-deprecated classes
const int NUM_VCLASSES = SumoVehicleClassStrings.size() - 8;

std::set<std::string> deprecatedVehicleClassesSeen;


static StringBijection<SUMOVehicleShape>::Entry sumoVehicleShapeStringInitializer[] = {
    {"pedestrian",            SUMOVehicleShape::PEDESTRIAN},
    {"bicycle",               SUMOVehicleShape::BICYCLE},
    {"moped",                 SUMOVehicleShape::MOPED},
    {"motorcycle",            SUMOVehicleShape::MOTORCYCLE},
    {"passenger",             SUMOVehicleShape::PASSENGER},
    {"passenger/sedan",       SUMOVehicleShape::PASSENGER_SEDAN},
    {"passenger/hatchback",   SUMOVehicleShape::PASSENGER_HATCHBACK},
    {"passenger/wagon",       SUMOVehicleShape::PASSENGER_WAGON},
    {"passenger/van",         SUMOVehicleShape::PASSENGER_VAN},
    {"taxi",                  SUMOVehicleShape::TAXI},
    {"delivery",              SUMOVehicleShape::DELIVERY},
    {"transport",             SUMOVehicleShape::TRUCK}, // !!! deprecated
    {"truck",                 SUMOVehicleShape::TRUCK},
    {"transport/semitrailer", SUMOVehicleShape::TRUCK_SEMITRAILER}, // !!! deprecated
    {"truck/semitrailer",     SUMOVehicleShape::TRUCK_SEMITRAILER},
    {"transport/trailer",     SUMOVehicleShape::TRUCK_1TRAILER}, // !!! deprecated
    {"truck/trailer",         SUMOVehicleShape::TRUCK_1TRAILER},
    {"bus/city",              SUMOVehicleShape::BUS}, // !!! deprecated
    {"bus",                   SUMOVehicleShape::BUS},
    {"bus/overland",          SUMOVehicleShape::BUS_COACH}, // !!! deprecated
    {"bus/coach",             SUMOVehicleShape::BUS_COACH},
    {"bus/flexible",          SUMOVehicleShape::BUS_FLEXIBLE},
    {"bus/trolley",           SUMOVehicleShape::BUS_TROLLEY},
    {"rail/slow",             SUMOVehicleShape::RAIL}, // !!! deprecated
    {"rail/fast",             SUMOVehicleShape::RAIL}, // !!! deprecated
    {"rail",                  SUMOVehicleShape::RAIL},
    {"rail/light",            SUMOVehicleShape::RAIL_CAR}, // !!! deprecated
    {"rail/city",             SUMOVehicleShape::RAIL_CAR}, // !!! deprecated
    {"rail/railcar",          SUMOVehicleShape::RAIL_CAR},
    {"rail/cargo",            SUMOVehicleShape::RAIL_CARGO},
    {"evehicle",              SUMOVehicleShape::E_VEHICLE},
    {"ant",                   SUMOVehicleShape::ANT},
    {"ship",                  SUMOVehicleShape::SHIP},
    {"emergency",             SUMOVehicleShape::EMERGENCY},
    {"firebrigade",           SUMOVehicleShape::FIREBRIGADE},
    {"police",                SUMOVehicleShape::POLICE},
    {"rickshaw",              SUMOVehicleShape::RICKSHAW },
    {"scooter",               SUMOVehicleShape::SCOOTER},
    {"aircraft",              SUMOVehicleShape::AIRCRAFT},
    {"",                      SUMOVehicleShape::UNKNOWN}
};


StringBijection<SUMOVehicleShape> SumoVehicleShapeStrings(
    sumoVehicleShapeStringInitializer, SUMOVehicleShape::UNKNOWN, false);

// ===========================================================================
// static values used for cached
// ===========================================================================

static std::map<SVCPermissions, std::vector<std::string> > vehicleClassNamesListCached;
static std::map<std::string, SVCPermissions> parseVehicleClassesCached;
static std::map<SVCPermissions, std::string> getVehicleClassNamesCached;
static std::string VehicleClassNameAll = "all";

// ===========================================================================
// additional constants
// ===========================================================================

const SUMOVehicleClass SUMOVehicleClass_MAX = SVC_CUSTOM2;

const SVCPermissions SVCAll = 2 * (long long int)SUMOVehicleClass_MAX - 1; // all relevant bits set to 1

const SVCPermissions SVC_UNSPECIFIED = -1;

const SUMOEmissionClass EMISSION_CLASS_UNSPECIFIED = -1;

const std::string DEFAULT_VTYPE_ID("DEFAULT_VEHTYPE");
const std::string DEFAULT_PEDTYPE_ID("DEFAULT_PEDTYPE");
const std::string DEFAULT_BIKETYPE_ID("DEFAULT_BIKETYPE");
const std::string DEFAULT_CONTAINERTYPE_ID("DEFAULT_CONTAINERTYPE");
const std::string DEFAULT_TAXITYPE_ID("DEFAULT_TAXITYPE");
const std::string DEFAULT_RAILTYPE_ID("DEFAULT_RAILTYPE");

const std::set<std::string> DEFAULT_VTYPES({DEFAULT_VTYPE_ID, DEFAULT_PEDTYPE_ID, DEFAULT_BIKETYPE_ID, DEFAULT_CONTAINERTYPE_ID, DEFAULT_TAXITYPE_ID, DEFAULT_RAILTYPE_ID});

const double DEFAULT_VEH_PROB(1.);
const double DEFAULT_VEH_MASS(1500.);
const double DEFAULT_VEH_WIDTH(1.8);
const double DEFAULT_VEH_HEIGHT(1.5);
const double DEFAULT_VEH_SHUT_OFF_STOP(300.);

const double DEFAULT_PEDESTRIAN_SPEED(5. / 3.6);

const double DEFAULT_BICYCLE_SPEED(20. / 3.6);

const double DEFAULT_CONTAINER_TRANSHIP_SPEED(5. / 3.6);


// ===========================================================================
// method definitions
// ===========================================================================

// Stop Offset

StopOffset::StopOffset() :
    myPermissions(SVCAll),
    myOffset(0) {
}


StopOffset::StopOffset(const SUMOSAXAttributes& attrs, bool& ok) :
    myPermissions(SVC_IGNORING),
    myOffset(0) {
    // first check conditions
    if (attrs.hasAttribute(SUMO_ATTR_VCLASSES) && attrs.hasAttribute(SUMO_ATTR_EXCEPTIONS)) {
        WRITE_ERROR(TL("Simultaneous specification of vClasses and exceptions is not allowed"));
        ok = false;
    }
    if (!attrs.hasAttribute(SUMO_ATTR_VALUE)) {
        WRITE_ERROR(TL("StopOffset requires an offset value"));
        ok = false;
    }
    // parse elements
    const std::string vClasses = attrs.getOpt<std::string>(SUMO_ATTR_VCLASSES, nullptr, ok, "");
    const std::string exceptions = attrs.getOpt<std::string>(SUMO_ATTR_EXCEPTIONS, nullptr, ok, "");
    // parse permissions
    if (attrs.hasAttribute(SUMO_ATTR_VCLASSES)) {
        myPermissions = parseVehicleClasses(vClasses);
    } else if (attrs.hasAttribute(SUMO_ATTR_EXCEPTIONS)) {
        myPermissions = ~parseVehicleClasses(exceptions);
    } else {
        // no vClasses specified, thus apply to all
        myPermissions = parseVehicleClasses("all");
    }
    // parse offset
    myOffset = attrs.getOpt<double>(SUMO_ATTR_VALUE, nullptr, ok, 0);
}


bool
StopOffset::isDefined() const {
    return myOffset != 0;
}


void
StopOffset::reset() {
    myPermissions = SVC_IGNORING;
    myOffset = 0;
}


SVCPermissions
StopOffset::getPermissions() const {
    return myPermissions;
}


std::string
StopOffset::getExceptions() const {
    return getVehicleClassNames(~myPermissions);
}


double
StopOffset::getOffset() const {
    return myOffset;
}


void
StopOffset::setPermissions(const SVCPermissions permissions) {
    myPermissions = permissions;
}


void
StopOffset::setExceptions(const std::string permissions) {
    myPermissions = ~parseVehicleClasses(permissions);
}


void
StopOffset::setOffset(const double offset) {
    myOffset = offset;
}


bool
StopOffset::operator==(StopOffset const& other) const {
    return ((myPermissions == other.myPermissions) &&
            (myOffset == other.myOffset));
}


bool
StopOffset::operator!=(StopOffset const& other) const {
    return ((myPermissions != other.myPermissions) ||
            (myOffset != other.myOffset));
}

// Conversion of SUMOVehicleClass

const std::string&
getVehicleClassNames(SVCPermissions permissions, bool expand) {
    if ((permissions & SVCAll) == SVCAll && !expand) {
        return VehicleClassNameAll;
    }
    // check if previously was cached
    if (getVehicleClassNamesCached.count(permissions) == 0) {
        getVehicleClassNamesCached[permissions] = joinToString(getVehicleClassNamesList(permissions), ' ');
    }
    return getVehicleClassNamesCached.at(permissions);
}


const std::vector<std::string>&
getVehicleClassNamesList(SVCPermissions permissions) {
    // first check if it's cached
    if (vehicleClassNamesListCached.count(permissions) == 0) {
        const std::vector<std::string> classNames = SumoVehicleClassStrings.getStrings();
        std::vector<std::string> result;
        for (std::vector<std::string>::const_iterator it = classNames.begin(); it != classNames.end(); it++) {
            const SVCPermissions svc = (SVCPermissions)SumoVehicleClassStrings.get(*it);
            if ((svc & permissions) == svc && svc != SVC_IGNORING) {
                result.push_back(*it);
            }
        }
        // add it into vehicleClassNamesListCached
        vehicleClassNamesListCached[permissions] = result;
    }
    return vehicleClassNamesListCached.at(permissions);
}


SUMOVehicleClass
getVehicleClassID(const std::string& name) {
    if (SumoVehicleClassStrings.hasString(name)) {
        return SumoVehicleClassStrings.get(name);
    }
    throw InvalidArgument("Unknown vehicle class '" + name + "'.");
}


SVCPermissions
getVehicleClassCompoundID(const std::string& name) {
    SVCPermissions ret = SVC_IGNORING;
    const std::vector<std::string> names = SumoVehicleClassStrings.getStrings();
    for (std::vector<std::string>::const_iterator it = names.begin(); it != names.end(); it++) {
        if (name.find(*it) != std::string::npos) {
            ret = ret | (SVCPermissions) SumoVehicleClassStrings.get(*it);
        }
    }
    return ret;
}


SVCPermissions
parseVehicleClasses(const std::string& allowedS) {
    if (allowedS == VehicleClassNameAll) {
        return SVCAll;
    }
    // check  if allowedS was previously cached
    if (parseVehicleClassesCached.count(allowedS) == 0) {
        SVCPermissions result = 0;
        StringTokenizer sta(allowedS, " ");
        while (sta.hasNext()) {
            const std::string s = sta.next();
            if (!SumoVehicleClassStrings.hasString(s)) {
                if (gIgnoreUnknownVClass) {
                    WRITE_WARNINGF(TL("Unknown vehicle class '%' ignored."), s);
                } else {
                    WRITE_ERRORF(TL("Unknown vehicle class '%' encountered."), s);
                }
            } else {
                const SUMOVehicleClass vc = getVehicleClassID(s);
                const std::string& realName = SumoVehicleClassStrings.getString(vc);
                if (realName != s) {
                    deprecatedVehicleClassesSeen.insert(s);
                }
                result |= vc;
            }
        }
        // save parsed vehicle class cached
        parseVehicleClassesCached[allowedS] = result;
    }
    return parseVehicleClassesCached.at(allowedS);
}


bool
canParseVehicleClasses(const std::string& classes) {
    if (classes == VehicleClassNameAll) {
        return true;
    }
    // check if was previously cached
    if (parseVehicleClassesCached.count(classes) != 0) {
        return true;
    }
    StringTokenizer sta(classes, " ");
    while (sta.hasNext()) {
        if (!SumoVehicleClassStrings.hasString(sta.next())) {
            return false;
        }
    }
    return true;
}


SVCPermissions
parseVehicleClasses(const std::string& allowedS, const std::string& disallowedS, const MMVersion& networkVersion) {
    if (allowedS.size() == 0 && disallowedS.size() == 0) {
        return SVCAll;
    } else if (allowedS.size() > 0 && disallowedS.size() > 0) {
        WRITE_WARNING(TL("SVCPermissions must be specified either via 'allow' or 'disallow'. Ignoring 'disallow'"));
        return parseVehicleClasses(allowedS);
    } else if (allowedS.size() > 0) {
        return parseVehicleClasses(allowedS);
    } else {
        return invertPermissions(extraDisallowed(parseVehicleClasses(disallowedS), networkVersion));
    }
}

SVCPermissions
extraDisallowed(SVCPermissions disallowed, const MMVersion& networkVersion) {
    if (networkVersion < MMVersion(1, 3)) {
        disallowed |= SVC_RAIL_FAST;
    }
    if (networkVersion < MMVersion(1, 20)) {
        if ((disallowed & SVC_RAIL_URBAN) != 0) {
            disallowed |= SVC_SUBWAY;
            disallowed |= SVC_CABLE_CAR;
        }
    }
    return disallowed;
}


SVCPermissions
invertPermissions(SVCPermissions permissions) {
    return SVCAll & ~permissions;
}


SVCPermissions
parseVehicleClasses(const std::vector<std::string>& allowedS) {
    SVCPermissions result = 0;
    if (std::find(allowedS.begin(), allowedS.end(), VehicleClassNameAll) != allowedS.end()) {
        return SVCAll;
    }
    for (std::vector<std::string>::const_iterator i = allowedS.begin(); i != allowedS.end(); ++i) {
        const SUMOVehicleClass vc = getVehicleClassID(*i);
        const std::string& realName = SumoVehicleClassStrings.getString(vc);
        if (realName != *i) {
            WRITE_WARNINGF(TL("The vehicle class '%' is deprecated, use '%' instead."), (*i), realName);
        }
        result |= getVehicleClassID(*i);
    }
    return result;
}


void
writePermissions(OutputDevice& into, SVCPermissions permissions) {
    if (permissions == SVCAll) {
        return;
    } else if (permissions == 0) {
        into.writeAttr(SUMO_ATTR_DISALLOW, VehicleClassNameAll);
        return;
    } else {
        int num_allowed = 0;
        for (SVCPermissions mask = 1; mask <= SUMOVehicleClass_MAX; mask = mask << 1) {
            if ((mask & permissions) == mask) {
                ++num_allowed;
            }
        }
        if (num_allowed <= (NUM_VCLASSES - num_allowed) && num_allowed > 0) {
            into.writeAttr(SUMO_ATTR_ALLOW, getVehicleClassNames(permissions));
        } else {
            into.writeAttr(SUMO_ATTR_DISALLOW, getVehicleClassNames(~permissions));
        }
    }
}


void
writePreferences(OutputDevice& into, SVCPermissions preferred) {
    if (preferred == SVCAll || preferred == 0) {
        return;
    } else {
        into.writeAttr(SUMO_ATTR_PREFER, getVehicleClassNames(preferred));
    }
}


SUMOVehicleShape
getVehicleShapeID(const std::string& name) {
    if (SumoVehicleShapeStrings.hasString(name)) {
        return SumoVehicleShapeStrings.get(name);
    } else {
        throw InvalidArgument("Unknown vehicle shape '" + name + "'.");
    }
}


bool
canParseVehicleShape(const std::string& shape) {
    return SumoVehicleShapeStrings.hasString(shape);
}


std::string
getVehicleShapeName(SUMOVehicleShape id) {
    return SumoVehicleShapeStrings.getString(id);
}


bool isRailway(SVCPermissions permissions) {
    return (permissions & SVC_RAIL_CLASSES) != 0 && (permissions & ~(SVC_RAIL_CLASSES | SVC_TAXI)) == 0;
}

bool isRailwayOrShared(SVCPermissions permissions) {
    // basically check that it isn't SVC_ALL
    return (permissions & SVC_RAIL_CLASSES) != 0 && (permissions & SVC_NON_ROAD_RAIL) == 0;
}

bool isTram(SVCPermissions permissions) {
    return (permissions & SVC_RAIL_CLASSES) == SVC_TRAM && (permissions & SVC_PASSENGER) == 0;
}

bool isBikepath(SVCPermissions permissions) {
    return (permissions & SVC_BICYCLE) == SVC_BICYCLE && (permissions & SVC_ROAD_MOTOR_CLASSES) == 0;
}


bool
isWaterway(SVCPermissions permissions) {
    return permissions == SVC_SHIP;
}

bool
isAirway(SVCPermissions permissions) {
    return permissions == SVC_AIRCRAFT || permissions == SVC_DRONE;
}

bool
isForbidden(SVCPermissions permissions) {
    return (permissions & SVCAll) == 0;
}


bool
isSidewalk(SVCPermissions permissions) {
    return (permissions & SVCAll) == SVC_PEDESTRIAN;
}


bool
isForVulnerableModes(SVCPermissions permissions) {
    return ((permissions & SVC_VULNERABLE) != 0 && (permissions & ~SVC_VULNERABLE) == 0);
}


bool
noVehicles(SVCPermissions permissions) {
    return isForbidden(permissions) || isSidewalk(permissions);
}


double
getDefaultVehicleLength(const SUMOVehicleClass vc) {
    switch (vc) {
        case SVC_PEDESTRIAN:
            return 0.215;
        case SVC_WHEELCHAIR:
            return 0.5;
        case SVC_BICYCLE:
            return 1.6;
        case SVC_SCOOTER:
            return 1.2;
        case SVC_MOPED:
            return 2.1;
        case SVC_MOTORCYCLE:
            return 2.2;
        case SVC_TRUCK:
            return 7.1;
        case SVC_TRAILER:
            return 16.5;
        case SVC_BUS:
            return 12.;
        case SVC_COACH:
            return 14.;
        case SVC_TRAM:
            return 22.;
        case SVC_RAIL_URBAN:
        case SVC_SUBWAY:
            return 36.5 * 3;
        case SVC_RAIL:
            return 67.5 * 2;
        case SVC_RAIL_ELECTRIC:
        case SVC_RAIL_FAST:
            return 25. * 8;
        case SVC_DELIVERY:
        case SVC_EMERGENCY:
            return 6.5;
        case SVC_SHIP:
            return 17;
        case SVC_CONTAINER:
            return 6.096;
        case SVC_DRONE:
            return 0.5;
        case SVC_AIRCRAFT:
            return 72.7;
        default:
            return 5; /*4.3*/
    }
}



/****************************************************************************/
