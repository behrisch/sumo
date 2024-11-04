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
/// @file    GNEAttributeCarrier.cpp
/// @author  Jakob Erdmann
/// @date    Feb 2011
///
// Abstract Base class for gui objects which carry attributes
/****************************************************************************/
#include <netedit/GNENet.h>
#include <netedit/GNEViewNet.h>
#include <utils/common/StringTokenizer.h>
#include <utils/common/ToString.h>
#include <utils/emissions/PollutantsInterface.h>
#include <utils/geom/GeomConvHelper.h>
#include <utils/gui/div/GUIGlobalSelection.h>
#include <utils/gui/images/VClassIcons.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/options/OptionsCont.h>
#include <utils/shapes/PointOfInterest.h>

#include "GNEAttributeCarrier.h"


// ===========================================================================
// static members
// ===========================================================================

std::map<SumoXMLTag, GNETagProperties> GNEAttributeCarrier::myTagProperties;
std::map<SumoXMLTag, GNETagProperties> GNEAttributeCarrier::myMergedPlanTagProperties;
const std::string GNEAttributeCarrier::FEATURE_LOADED = "loaded";
const std::string GNEAttributeCarrier::FEATURE_GUESSED = "guessed";
const std::string GNEAttributeCarrier::FEATURE_MODIFIED = "modified";
const std::string GNEAttributeCarrier::FEATURE_APPROVED = "approved";
const size_t GNEAttributeCarrier::MAXNUMBEROFATTRIBUTES = 128;
const Parameterised::Map GNEAttributeCarrier::PARAMETERS_EMPTY;
const std::string GNEAttributeCarrier::True = toString(true);
const std::string GNEAttributeCarrier::False = toString(false);


// ===========================================================================
// method definitions
// ===========================================================================

GNEAttributeCarrier::GNEAttributeCarrier(const SumoXMLTag tag, GNENet* net) :
    myTagProperty(getTagProperty(tag)),
    myNet(net) {
}


GNEAttributeCarrier::~GNEAttributeCarrier() {}


const std::string
GNEAttributeCarrier::getID() const {
    return getAttribute(SUMO_ATTR_ID);
}


GNENet*
GNEAttributeCarrier::getNet() const {
    return myNet;
}


void
GNEAttributeCarrier::selectAttributeCarrier(const bool changeFlag) {
    if (getGUIGlObject() && myTagProperty.isSelectable()) {
        gSelected.select(getGUIGlObject()->getGlID());
        if (changeFlag) {
            mySelected = true;
        }
    }
}


void
GNEAttributeCarrier::unselectAttributeCarrier(const bool changeFlag) {
    if (getGUIGlObject() && myTagProperty.isSelectable()) {
        gSelected.deselect(getGUIGlObject()->getGlID());
        if (changeFlag) {
            mySelected = false;
        }
    }
}


bool
GNEAttributeCarrier::isAttributeCarrierSelected() const {
    return mySelected;
}


bool
GNEAttributeCarrier::drawUsingSelectColor() const {
    // first check if element is selected
    if (mySelected) {
        // get flag for network element
        const bool networkElement = myTagProperty.isNetworkElement() || myTagProperty.isAdditionalElement();
        // check current supermode
        if (networkElement && myNet->getViewNet()->getEditModes().isCurrentSupermodeNetwork()) {
            return true;
        } else if (myTagProperty.isDemandElement() && myNet->getViewNet()->getEditModes().isCurrentSupermodeDemand()) {
            return true;
        } else if (myTagProperty.isGenericData() && myNet->getViewNet()->getEditModes().isCurrentSupermodeData()) {
            return true;
        } else {
            return false;
        }
    } else {
        return false;
    }
}


void
GNEAttributeCarrier::setInGrid(bool value) {
    myInGrid = value;
}


bool
GNEAttributeCarrier::inGrid() const {
    return myInGrid;
}


bool
GNEAttributeCarrier::checkDrawInspectContour() const {
    return myNet->getViewNet()->isAttributeCarrierInspected(this);
}


bool
GNEAttributeCarrier::checkDrawFrontContour() const {
    return (myNet->getViewNet()->getFrontAttributeCarrier() == this);
}


void
GNEAttributeCarrier::resetDefaultValues() {
    for (const auto& attrProperty : myTagProperty) {
        if (attrProperty.hasDefaultValue()) {
            setAttribute(attrProperty.getAttr(), attrProperty.getDefaultValue());
            if (attrProperty.isActivatable()) {
                toggleAttribute(attrProperty.getAttr(), attrProperty.getDefaultActivated());
            }
        }
    }
}


void
GNEAttributeCarrier::enableAttribute(SumoXMLAttr /*key*/, GNEUndoList* /*undoList*/) {
    throw ProcessError(TL("Nothing to enable, implement in Children"));

}


void
GNEAttributeCarrier::disableAttribute(SumoXMLAttr /*key*/, GNEUndoList* /*undoList*/) {
    throw ProcessError(TL("Nothing to disable, implement in Children"));
}


bool
GNEAttributeCarrier::isAttributeEnabled(SumoXMLAttr /*key*/) const {
    // by default, all attributes are enabled
    return true;
}


bool
GNEAttributeCarrier::isAttributeComputed(SumoXMLAttr /*key*/) const {
    // by default, all attributes aren't computed
    return false;
}


template<> int
GNEAttributeCarrier::parse(const std::string& string) {
    return StringUtils::toInt(string);
}


template<> double
GNEAttributeCarrier::parse(const std::string& string) {
    return StringUtils::toDouble(string);
}


template<> SUMOTime
GNEAttributeCarrier::parse(const std::string& string) {
    return string2time(string);
}


template<> bool
GNEAttributeCarrier::parse(const std::string& string) {
    return StringUtils::toBool(string);
}


template<> std::string
GNEAttributeCarrier::parse(const std::string& string) {
    return string;
}


template<> SUMOVehicleClass
GNEAttributeCarrier::parse(const std::string& string) {
    if (string.size() == 0) {
        throw EmptyData();
    } else if (!SumoVehicleClassStrings.hasString(string)) {
        return SVC_IGNORING;
    } else {
        return SumoVehicleClassStrings.get(string);
    }
}


template<> RGBColor
GNEAttributeCarrier::parse(const std::string& string) {
    if (string.empty()) {
        return RGBColor::INVISIBLE;
    } else {
        return RGBColor::parseColor(string);
    }
}


template<> Position
GNEAttributeCarrier::parse(const std::string& string) {
    if (string.size() == 0) {
        throw EmptyData();
    } else {
        bool ok = true;
        PositionVector pos = GeomConvHelper::parseShapeReporting(string, "user-supplied position", 0, ok, false, false);
        if (!ok || (pos.size() != 1)) {
            throw NumberFormatException("(Position) " + string);
        } else {
            return pos[0];
        }
    }
}


template<> PositionVector
GNEAttributeCarrier::parse(const std::string& string) {
    PositionVector posVector;
    // empty string are allowed (It means empty position vector)
    if (string.empty()) {
        return posVector;
    } else {
        bool ok = true;
        posVector = GeomConvHelper::parseShapeReporting(string, "user-supplied shape", 0, ok, false, true);
        if (!ok) {
            throw NumberFormatException("(Position List) " + string);
        } else {
            return posVector;
        }
    }
}


template<> SUMOVehicleShape
GNEAttributeCarrier::parse(const std::string& string) {
    if ((string == "unknown") || (!SumoVehicleShapeStrings.hasString(string))) {
        return SUMOVehicleShape::UNKNOWN;
    } else {
        return SumoVehicleShapeStrings.get(string);
    }
}


template<> std::vector<std::string>
GNEAttributeCarrier::parse(const std::string& string) {
    return StringTokenizer(string).getVector();
}


template<> std::set<std::string>
GNEAttributeCarrier::parse(const std::string& string) {
    std::vector<std::string> vectorString = StringTokenizer(string).getVector();
    std::set<std::string> solution;
    for (const auto& i : vectorString) {
        solution.insert(i);
    }
    return solution;
}


template<> std::vector<int>
GNEAttributeCarrier::parse(const std::string& string) {
    std::vector<std::string> parsedValues = parse<std::vector<std::string> >(string);
    std::vector<int> parsedIntValues;
    for (const auto& i : parsedValues) {
        parsedIntValues.push_back(parse<int>(i));
    }
    return parsedIntValues;
}


template<> std::vector<double>
GNEAttributeCarrier::parse(const std::string& string) {
    std::vector<std::string> parsedValues = parse<std::vector<std::string> >(string);
    std::vector<double> parsedDoubleValues;
    for (const auto& i : parsedValues) {
        parsedDoubleValues.push_back(parse<double>(i));
    }
    return parsedDoubleValues;
}


template<> std::vector<bool>
GNEAttributeCarrier::parse(const std::string& string) {
    std::vector<std::string> parsedValues = parse<std::vector<std::string> >(string);
    std::vector<bool> parsedBoolValues;
    for (const auto& i : parsedValues) {
        parsedBoolValues.push_back(parse<bool>(i));
    }
    return parsedBoolValues;
}


template<> std::vector<SumoXMLAttr>
GNEAttributeCarrier::parse(const std::string& value) {
    // Declare string vector
    std::vector<std::string> attributesStr = GNEAttributeCarrier::parse<std::vector<std::string> > (value);
    std::vector<SumoXMLAttr> attributes;
    // Iterate over lanes IDs, retrieve Lanes and add it into parsedLanes
    for (const auto& attributeStr : attributesStr) {
        if (SUMOXMLDefinitions::Tags.hasString(attributeStr)) {
            attributes.push_back(static_cast<SumoXMLAttr>(SUMOXMLDefinitions::Attrs.get(attributeStr)));
        } else {
            throw InvalidArgument("Error parsing attributes. Attribute '" + attributeStr + "'  doesn't exist");
        }
    }
    return attributes;
}


template<> std::vector<GNEEdge*>
GNEAttributeCarrier::parse(GNENet* net, const std::string& value) {
    // Declare string vector
    std::vector<std::string> edgeIds = GNEAttributeCarrier::parse<std::vector<std::string> > (value);
    std::vector<GNEEdge*> parsedEdges;
    // Iterate over edges IDs, retrieve Edges and add it into parsedEdges
    for (const auto& edgeID : edgeIds) {
        GNEEdge* retrievedEdge = net->getAttributeCarriers()->retrieveEdge(edgeID, false);
        if (retrievedEdge) {
            parsedEdges.push_back(net->getAttributeCarriers()->retrieveEdge(edgeID));
        } else {
            throw FormatException("Error parsing parameter " + toString(SUMO_ATTR_EDGES) + ". " +
                                  toString(SUMO_TAG_EDGE) + " '" + edgeID + "' doesn't exist");
        }
    }
    return parsedEdges;
}


template<> std::vector<GNELane*>
GNEAttributeCarrier::parse(GNENet* net, const std::string& value) {
    // Declare string vector
    std::vector<std::string> laneIds = GNEAttributeCarrier::parse<std::vector<std::string> > (value);
    std::vector<GNELane*> parsedLanes;
    // Iterate over lanes IDs, retrieve Lanes and add it into parsedLanes
    for (const auto& laneID : laneIds) {
        GNELane* retrievedLane = net->getAttributeCarriers()->retrieveLane(laneID, false);
        if (retrievedLane) {
            parsedLanes.push_back(net->getAttributeCarriers()->retrieveLane(laneID));
        } else {
            throw FormatException("Error parsing parameter " + toString(SUMO_ATTR_LANES) + ". " +
                                  toString(SUMO_TAG_LANE) + " '" + laneID + "'  doesn't exist");
        }
    }
    return parsedLanes;
}


template<> std::string
GNEAttributeCarrier::parseIDs(const std::vector<GNEEdge*>& ACs) {
    // obtain ID's of edges and return their join
    std::vector<std::string> edgeIDs;
    for (const auto& i : ACs) {
        edgeIDs.push_back(i->getID());
    }
    return joinToString(edgeIDs, " ");
}


template<> std::string
GNEAttributeCarrier::parseIDs(const std::vector<GNELane*>& ACs) {
    // obtain ID's of lanes and return their join
    std::vector<std::string> laneIDs;
    for (const auto& i : ACs) {
        laneIDs.push_back(i->getID());
    }
    return joinToString(laneIDs, " ");
}


bool
GNEAttributeCarrier::lanesConsecutives(const std::vector<GNELane*>& lanes) {
    // we need at least two lanes
    if (lanes.size() > 1) {
        // now check that lanes are consecutive (not necessary connected)
        int currentLane = 0;
        while (currentLane < ((int)lanes.size() - 1)) {
            int nextLane = -1;
            // iterate over outgoing edges of destination junction of edge's lane
            for (int i = 0; (i < (int)lanes.at(currentLane)->getParentEdge()->getToJunction()->getGNEOutgoingEdges().size()) && (nextLane == -1); i++) {
                // iterate over lanes of outgoing edges of destination junction of edge's lane
                for (int j = 0; (j < (int)lanes.at(currentLane)->getParentEdge()->getToJunction()->getGNEOutgoingEdges().at(i)->getLanes().size()) && (nextLane == -1); j++) {
                    // check if lane correspond to the next lane of "lanes"
                    if (lanes.at(currentLane)->getParentEdge()->getToJunction()->getGNEOutgoingEdges().at(i)->getLanes().at(j) == lanes.at(currentLane + 1)) {
                        nextLane = currentLane;
                    }
                }
            }
            if (nextLane == -1) {
                return false;
            } else {
                currentLane++;
            }
        }
        return true;
    } else {
        return false;
    }
}


template<> std::string
GNEAttributeCarrier::getACParameters() const {
    std::string result;
    // Generate an string using the following structure: "key1=value1|key2=value2|...
    for (const auto& parameter : getACParametersMap()) {
        result += parameter.first + "=" + parameter.second + "|";
    }
    // remove the last "|"
    if (!result.empty()) {
        result.pop_back();
    }
    return result;
}


template<> std::vector<std::pair<std::string, std::string> >
GNEAttributeCarrier::getACParameters() const {
    std::vector<std::pair<std::string, std::string> > result;
    // Generate a vector string using the following structure: "<key1,value1>, <key2, value2>,...
    for (const auto& parameter : getACParametersMap()) {
        result.push_back(std::make_pair(parameter.first, parameter.second));
    }
    return result;
}


void
GNEAttributeCarrier::setACParameters(const std::string& parameters, GNEUndoList* undoList) {
    // declare map
    Parameterised::Map parametersMap;
    // separate value in a vector of string using | as separator
    StringTokenizer parametersTokenizer(parameters, "|", true);
    // iterate over all values
    while (parametersTokenizer.hasNext()) {
        // obtain key and value and save it in myParameters
        const std::vector<std::string> keyValue = StringTokenizer(parametersTokenizer.next(), "=", true).getVector();
        if (keyValue.size() == 2) {
            parametersMap[keyValue.front()] = keyValue.back();
        }
    }
    // set setACParameters map
    setACParameters(parametersMap, undoList);
}


void
GNEAttributeCarrier::setACParameters(const std::vector<std::pair<std::string, std::string> >& parameters, GNEUndoList* undoList) {
    // declare parametersMap
    Parameterised::Map parametersMap;
    // Generate an string using the following structure: "key1=value1|key2=value2|...
    for (const auto& parameter : parameters) {
        parametersMap[parameter.first] = parameter.second;
    }
    // set setACParameters map
    setACParameters(parametersMap, undoList);
}


void
GNEAttributeCarrier::setACParameters(const Parameterised::Map& parameters, GNEUndoList* undoList) {
    // declare result string
    std::string paramsStr;
    // Generate an string using the following structure: "key1=value1|key2=value2|...
    for (const auto& parameter : parameters) {
        paramsStr += parameter.first + "=" + parameter.second + "|";
    }
    // remove the last "|"
    if (!paramsStr.empty()) {
        paramsStr.pop_back();
    }
    // set parameters
    setAttribute(GNE_ATTR_PARAMETERS, paramsStr, undoList);
}


void
GNEAttributeCarrier::addACParameters(const std::string& key, const std::string& attribute, GNEUndoList* undoList) {
    // get parametersMap
    Parameterised::Map parametersMap = getACParametersMap();
    // add (or update) attribute
    parametersMap[key] = attribute;
    // set attribute
    setACParameters(parametersMap, undoList);
}


void
GNEAttributeCarrier::removeACParametersKeys(const std::vector<std::string>& keepKeys, GNEUndoList* undoList) {
    // declare parametersMap
    Parameterised::Map newParametersMap;
    // iterate over parameters map
    for (const auto& parameter : getACParametersMap()) {
        // copy to newParametersMap if key is in keepKeys
        if (std::find(keepKeys.begin(), keepKeys.end(), parameter.first) != keepKeys.end()) {
            newParametersMap.insert(parameter);
        }
    }
    // set newParametersMap map
    setACParameters(newParametersMap, undoList);
}


std::string
GNEAttributeCarrier::getAlternativeValueForDisabledAttributes(SumoXMLAttr key) const {
    switch (key) {
        // Crossings
        case SUMO_ATTR_TLLINKINDEX:
        case SUMO_ATTR_TLLINKINDEX2:
            return "No TLS";
        // connections
        case SUMO_ATTR_DIR: {
            // special case for connection directions
            std::string direction = getAttribute(key);
            if (direction == "s") {
                return "Straight (s)";
            } else if (direction ==  "t") {
                return "Turn (t))";
            } else if (direction ==  "l") {
                return "Left (l)";
            } else if (direction ==  "r") {
                return "Right (r)";
            } else if (direction ==  "L") {
                return "Partially left (L)";
            } else if (direction ==  "R") {
                return "Partially right (R)";
            } else if (direction ==  "invalid") {
                return "No direction (Invalid))";
            } else {
                return "undefined";
            }
        }
        case SUMO_ATTR_STATE: {
            // special case for connection states
            std::string state = getAttribute(key);
            if (state == "-") {
                return "Dead end (-)";
            } else if (state == "=") {
                return "equal (=)";
            } else if (state == "m") {
                return "Minor link (m)";
            } else if (state == "M") {
                return "Major link (M)";
            } else if (state == "O") {
                return "TLS controller off (O)";
            } else if (state == "o") {
                return "TLS yellow flashing (o)";
            } else if (state == "y") {
                return "TLS yellow minor link (y)";
            } else if (state == "Y") {
                return "TLS yellow major link (Y)";
            } else if (state == "r") {
                return "TLS red (r)";
            } else if (state == "g") {
                return "TLS green minor (g)";
            } else if (state == "G") {
                return "TLS green major (G)";
            } else if (state == "Z") {
                return "Zipper (Z)";
            } else {
                return "undefined";
            }
        }
        default:
            return getAttribute(key);
    }
}


std::string
GNEAttributeCarrier::getAttributeForSelection(SumoXMLAttr key) const {
    return getAttribute(key);
}


const std::string&
GNEAttributeCarrier::getTagStr() const {
    return myTagProperty.getTagStr();
}


FXIcon*
GNEAttributeCarrier::getACIcon() const {
    // define on first access
    if (myTagProperties.size() == 0) {
        fillAttributeCarriers();
    }
    // special case for vClass icons
    if (myTagProperty.vClassIcon()) {
        return VClassIcons::getVClassIcon(SumoVehicleClassStrings.get(getAttribute(SUMO_ATTR_VCLASS)));
    } else {
        return GUIIconSubSys::getIcon(myTagProperty.getGUIIcon());
    }
}


bool
GNEAttributeCarrier::isTemplate() const {
    return myIsTemplate;
}


const GNETagProperties&
GNEAttributeCarrier::getTagProperty() const {
    return myTagProperty;
}

// ===========================================================================
// static methods
// ===========================================================================

const GNETagProperties&
GNEAttributeCarrier::getTagProperty(SumoXMLTag tag) {
    // define on first access
    if (myTagProperties.size() == 0) {
        fillAttributeCarriers();
    }
    // check that tag is defined
    if (myTagProperties.count(tag) == 0) {
        if (myMergedPlanTagProperties.count(tag) == 0) {
            throw ProcessError(TLF("TagProperty for tag '%' not defined", toString(tag)));
        } else {
            return myMergedPlanTagProperties.at(tag);
        }
    } else {
        return myTagProperties.at(tag);
    }
}


const std::vector<GNETagProperties>
GNEAttributeCarrier::getTagPropertiesByType(const int tagPropertyCategory, const bool mergeCommonPlans) {
    std::vector<GNETagProperties> allowedTags;
    // define on first access
    if (myTagProperties.size() == 0) {
        fillAttributeCarriers();
    }
    if (tagPropertyCategory & GNETagProperties::TagType::NETWORKELEMENT) {
        // fill networkElements tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second.isNetworkElement()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::ADDITIONALELEMENT) {
        // fill additional tags (only with pure additionals)
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second.isAdditionalPureElement()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::SHAPE) {
        // fill shape tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second.isShapeElement()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::TAZELEMENT) {
        // fill taz tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second.isTAZElement()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::WIRE) {
        // fill wire tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second.isWireElement()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::DEMANDELEMENT) {
        // fill demand tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second.isDemandElement()) {
                if (!mergeCommonPlans || !tagProperty.second.isPlan()) {
                    allowedTags.push_back(tagProperty.second);
                }
            }
        }
        if (mergeCommonPlans) {
            for (const auto& mergedPlanTagProperty : myMergedPlanTagProperties) {
                allowedTags.push_back(mergedPlanTagProperty.second);
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::ROUTE) {
        // fill route tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second.isRoute()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::VEHICLE) {
        // fill vehicle tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second.isVehicle()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::VEHICLESTOP) {
        // fill stop (and waypoints) tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second.isVehicleStop()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::PERSON) {
        // fill person tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second.isPerson()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::PERSONPLAN) {
        // fill person plan tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second.isPlanPerson()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::PERSONTRIP) {
        if (mergeCommonPlans) {
            allowedTags.push_back(myMergedPlanTagProperties.at(SUMO_TAG_PERSONTRIP));
        } else {
            // fill demand tags
            for (const auto& tagProperty : myTagProperties) {
                if (tagProperty.second.isPlanPersonTrip()) {
                    allowedTags.push_back(tagProperty.second);
                }
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::WALK) {
        if (mergeCommonPlans) {
            allowedTags.push_back(myMergedPlanTagProperties.at(SUMO_TAG_WALK));
        } else {
            // fill demand tags
            for (const auto& tagProperty : myTagProperties) {
                if (tagProperty.second.isPlanWalk()) {
                    allowedTags.push_back(tagProperty.second);
                }
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::RIDE) {
        if (mergeCommonPlans) {
            allowedTags.push_back(myMergedPlanTagProperties.at(SUMO_TAG_RIDE));
        } else {
            // fill demand tags
            for (const auto& tagProperty : myTagProperties) {
                if (tagProperty.second.isPlanRide()) {
                    allowedTags.push_back(tagProperty.second);
                }
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::STOPPERSON) {
        if (mergeCommonPlans) {
            allowedTags.push_back(myMergedPlanTagProperties.at(GNE_TAG_STOPPERSON));
        } else {
            // fill demand tags
            for (const auto& tagProperty : myTagProperties) {
                if (tagProperty.second.isPlanStopPerson()) {
                    allowedTags.push_back(tagProperty.second);
                }
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::GENERICDATA) {
        // fill generic data tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second.isGenericData()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::MEANDATA) {
        // fill generic data tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second.isMeanData()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::CONTAINER) {
        // fill container tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second.isContainer()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::CONTAINERPLAN) {
        // fill container plan tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second.isPlanContainer()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::TRANSPORT) {
        if (mergeCommonPlans) {
            allowedTags.push_back(myMergedPlanTagProperties.at(SUMO_TAG_TRANSPORT));
        } else {
            // fill demand tags
            for (const auto& tagProperty : myTagProperties) {
                if (tagProperty.second.isPlanTransport()) {
                    allowedTags.push_back(tagProperty.second);
                }
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::TRANSHIP) {
        if (mergeCommonPlans) {
            allowedTags.push_back(myMergedPlanTagProperties.at(SUMO_TAG_TRANSHIP));
        } else {
            // fill demand tags
            for (const auto& tagProperty : myTagProperties) {
                if (tagProperty.second.isPlanTranship()) {
                    allowedTags.push_back(tagProperty.second);
                }
            }
        }
    }
    if (tagPropertyCategory & GNETagProperties::TagType::STOPCONTAINER) {
        if (mergeCommonPlans) {
            allowedTags.push_back(myMergedPlanTagProperties.at(GNE_TAG_STOPCONTAINER));
        } else {
            // fill demand tags
            for (const auto& tagProperty : myTagProperties) {
                if (tagProperty.second.isPlanStopContainer()) {
                    allowedTags.push_back(tagProperty.second);
                }
            }
        }
    }
    return allowedTags;
}


const std::vector<GNETagProperties>
GNEAttributeCarrier::getTagPropertiesByMergingTag(SumoXMLTag mergingTag) {
    std::vector<GNETagProperties> result;
    // fill tags
    for (const auto& tagProperty : myTagProperties) {
        if ((mergingTag == SUMO_TAG_PERSONTRIP) && tagProperty.second.isPlanPerson()) {
            result.push_back(tagProperty.second);
        } else if ((mergingTag == SUMO_TAG_RIDE) && tagProperty.second.isPlanRide()) {
            result.push_back(tagProperty.second);
        } else if ((mergingTag == SUMO_TAG_WALK) && tagProperty.second.isPlanWalk()) {
            result.push_back(tagProperty.second);
        } else if ((mergingTag == GNE_TAG_STOPPERSON) && tagProperty.second.isPlanStopPerson()) {
            result.push_back(tagProperty.second);
        } else if ((mergingTag == SUMO_TAG_TRANSPORT) && tagProperty.second.isPlanTransport()) {
            result.push_back(tagProperty.second);
        } else if ((mergingTag == SUMO_TAG_TRANSHIP) && tagProperty.second.isPlanTranship()) {
            result.push_back(tagProperty.second);
        } else if ((mergingTag == GNE_TAG_STOPCONTAINER) && tagProperty.second.isPlanStopContainer()) {
            result.push_back(tagProperty.second);
        }
    }
    return result;
}

// ===========================================================================
// private
// ===========================================================================

void
GNEAttributeCarrier::resetAttributes() {
    for (const auto& attrProperty : myTagProperty) {
        if (attrProperty.hasDefaultValue()) {
            setAttribute(attrProperty.getAttr(), attrProperty.getDefaultValue());
        }
    }
}


void
GNEAttributeCarrier::toggleAttribute(SumoXMLAttr /*key*/, const bool /*value*/) {
    throw ProcessError(TL("Nothing to toggle, implement in Children"));
}


void
GNEAttributeCarrier::fillAttributeCarriers() {
    // fill all groups of ACs
    fillNetworkElements();
    fillAdditionalElements();
    fillShapeElements();
    fillTAZElements();
    fillWireElements();
    fillJuPedSimElements();
    // demand
    fillDemandElements();
    fillVehicleElements();
    fillStopElements();
    fillWaypointElements();
    // persons
    fillPersonElements();
    fillPersonPlanTrips();
    fillPersonPlanWalks();
    fillPersonPlanRides();
    fillPersonStopElements();
    // containers
    fillContainerElements();
    fillContainerTransportElements();
    fillContainerTranshipElements();
    fillContainerStopElements();
    //data
    fillDataElements();
    // check integrity of all Tags (function checkTagIntegrity() throws an exception if there is an inconsistency)
    for (const auto& tagProperty : myTagProperties) {
        tagProperty.second.checkTagIntegrity();
    }
}


void
GNEAttributeCarrier::fillNetworkElements() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    // obtain Node Types except SumoXMLNodeType::DEAD_END_DEPRECATED
    const auto& neteditOptions = OptionsCont::getOptions();
    std::vector<std::string> nodeTypes = SUMOXMLDefinitions::NodeTypes.getStrings();
    nodeTypes.erase(std::find(nodeTypes.begin(), nodeTypes.end(), toString(SumoXMLNodeType::DEAD_END_DEPRECATED)));
    nodeTypes.erase(std::find(nodeTypes.begin(), nodeTypes.end(), toString(SumoXMLNodeType::DEAD_END)));
    nodeTypes.erase(std::find(nodeTypes.begin(), nodeTypes.end(), toString(SumoXMLNodeType::INTERNAL)));
    // obtain TLTypes (note: avoid insert all TLTypes because some of them are experimental and not documented)
    std::vector<std::string> TLTypes;
    TLTypes.push_back(toString(TrafficLightType::STATIC));
    TLTypes.push_back(toString(TrafficLightType::ACTUATED));
    TLTypes.push_back(toString(TrafficLightType::DELAYBASED));
    TLTypes.push_back(toString(TrafficLightType::NEMA));
    // fill networkElement ACs
    SumoXMLTag currentTag = SUMO_TAG_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::NETWORKELEMENT,
                                      GNETagProperties::TagProperty::RTREE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::JUNCTION, currentTag, TL("Junction"));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of the node"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::POSITION | GNEAttributeProperties::UPDATEGEOMETRY, // virtual attribute from the combination of the actually attributes SUMO_ATTR_X, SUMO_ATTR_Y
                                              TL("The x-y-z position of the node on the plane in meters"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("An optional type for the node"));
        attrProperty.setDiscreteValues(nodeTypes);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SHAPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::POSITION | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("A custom shape for that node"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_RADIUS,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("Optional turning radius (for all corners) for that node in meters"),
                                              "1.5");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_KEEP_CLEAR,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Whether the junction-blocking-heuristic should be activated at this node"),
                                              "1");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_RIGHT_OF_WAY,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("How to compute right of way rules at this node"),
                                              SUMOXMLDefinitions::RightOfWayValues.getString(RightOfWay::DEFAULT));
        attrProperty.setDiscreteValues(SUMOXMLDefinitions::RightOfWayValues.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FRINGE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Whether this junction is at the fringe of the network"),
                                              SUMOXMLDefinitions::FringeTypeValues.getString(FringeType::DEFAULT));
        attrProperty.setDiscreteValues(SUMOXMLDefinitions::FringeTypeValues.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Optional name of junction"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TLTYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("An optional type for the traffic light algorithm"));
        attrProperty.setDiscreteValues(TLTypes);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TLLAYOUT,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("An optional layout for the traffic light plan"));
        attrProperty.setDiscreteValues({toString(TrafficLightLayout::DEFAULT),
                                        toString(TrafficLightLayout::OPPOSITES),
                                        toString(TrafficLightLayout::INCOMING),
                                        toString(TrafficLightLayout::ALTERNATE_ONEWAY)});
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TLID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("An optional id for the traffic light program"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(GNE_ATTR_IS_ROUNDABOUT,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Whether this junction is part of a roundabout"), "false");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_TYPE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::NETWORKELEMENT,
                                      GNETagProperties::TagProperty::NOTDRAWABLE | GNETagProperties::TagProperty::NOTSELECTABLE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::EDGETYPE, currentTag, TL("EdgeType"));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("The id of the edge"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NUMLANES,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("The number of lanes of the edge"),
                                              toString(neteditOptions.getInt("default.lanenumber")));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SPEED,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("The maximum speed allowed on the edge in m/s"),
                                              toString(neteditOptions.getFloat("default.speed")));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ALLOW,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("Explicitly allows the given vehicle classes (not given will be not allowed)"),
                                              "all");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DISALLOW,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("Explicitly disallows the given vehicle classes (not given will be allowed)"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SPREADTYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("The spreadType defines how to compute the lane geometry from the edge geometry (used for visualization)"),
                                              SUMOXMLDefinitions::LaneSpreadFunctions.getString(LaneSpreadFunction::RIGHT));
        attrProperty.setDiscreteValues(SUMOXMLDefinitions::LaneSpreadFunctions.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PRIORITY,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("The priority of the edge"),
                                              toString(neteditOptions.getInt("default.priority")));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_WIDTH,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("Lane width for all lanes of this edge in meters (used for visualization)"),
                                              "default");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SIDEWALKWIDTH,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("The width of the sidewalk that should be added as an additional lane"),
                                              "default");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_BIKELANEWIDTH,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("The width of the bike lane that should be added as an additional lane"),
                                              "default");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_LANETYPE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::NETWORKELEMENT,
                                      GNETagProperties::TagProperty::NOTDRAWABLE | GNETagProperties::TagProperty::NOTSELECTABLE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::LANETYPE, currentTag, TL("LaneType"));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_SPEED,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("The maximum speed allowed on the lane in m/s"),
                                              toString(neteditOptions.getFloat("default.speed")));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ALLOW,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("Explicitly allows the given vehicle classes (not given will be not allowed)"),
                                              "all");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DISALLOW,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("Explicitly disallows the given vehicle classes (not given will be allowed)"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_WIDTH,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::COPYABLE,
                                              TL("Lane width for all lanes of this type in meters (used for visualization)"),
                                              "default");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::NETWORKELEMENT,
                                      GNETagProperties::TagProperty::RTREE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::EDGE, currentTag, TL("Edge"));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("Edge ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FROM,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of a node within the nodes-file the edge shall start at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TO,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of a node within the nodes-file the edge shall end at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SPEED,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("The maximum speed allowed on the edge in m/s"),
                                              toString(neteditOptions.getFloat("default.speed")));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PRIORITY,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("The priority of the edge"),
                                              toString(neteditOptions.getInt("default.priority")));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NUMLANES,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("The number of lanes of the edge"),
                                              toString(neteditOptions.getInt("default.lanenumber")));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("The name of a type within the SUMO edge type file"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ALLOW,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("Explicitly allows the given vehicle classes (not given will be not allowed)"),
                                              "all");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DISALLOW,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("Explicitly disallows the given vehicle classes (not given will be allowed)"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SHAPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::POSITION | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("If the shape is given it should start and end with the positions of the from-node and to-node"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LENGTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The length of the edge in meter"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SPREADTYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("The spreadType defines how to compute the lane geometry from the edge geometry (used for visualization)"),
                                              SUMOXMLDefinitions::LaneSpreadFunctions.getString(LaneSpreadFunction::RIGHT));
        attrProperty.setDiscreteValues(SUMOXMLDefinitions::LaneSpreadFunctions.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("street name (does not need to be unique, used for visualization)"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_WIDTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::COPYABLE,
                                              TL("Lane width for all lanes of this edge in meters (used for visualization)"),
                                              "-1");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ENDOFFSET,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("Move the stop line back from the intersection by the given amount"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(GNE_ATTR_SHAPE_START,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::POSITION | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY, // virtual attribute used to define an endPoint
                                              TL("Custom position in which shape start (by default position of junction from)"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(GNE_ATTR_SHAPE_END,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::POSITION | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY, // virtual attribute from to define an endPoint
                                              TL("Custom position in which shape end (by default position of junction from)"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(GNE_ATTR_BIDIR,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UNIQUE, // virtual attribute to check of this edge is part of a bidirectional railway (cannot be edited)
                                              TL("Show if edge is bidirectional"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DISTANCE,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UNIQUE,
                                              TL("Distance"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(GNE_ATTR_STOPOFFSET,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The stop offset as positive value in meters"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(GNE_ATTR_STOPOEXCEPTION,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Specifies, for which vehicle classes the stopOffset does NOT apply."));
        attrProperty.setDiscreteValues(SumoVehicleClassStrings.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(GNE_ATTR_IS_ROUNDABOUT,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UNIQUE, // cannot be edited
                                              TL("Whether this edge is part of a roundabout"), "false");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_LANE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::NETWORKELEMENT,
                                      GNETagProperties::TagProperty::NO_PROPERTY,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::LANE, currentTag, TL("Lane"));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("Lane ID (Automatic, non editable)"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_INDEX,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The enumeration index of the lane (0 is the rightmost lane, <NUMBER_LANES>-1 is the leftmost one)"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SPEED,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("Speed in meters per second"),
                                              "13.89");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ALLOW,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("Explicitly allows the given vehicle classes (not given will be not allowed)"),
                                              "all");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DISALLOW,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("Explicitly disallows the given vehicle classes (not given will be allowed)"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_WIDTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::COPYABLE,
                                              TL("Width in meters (used for visualization)"),
                                              "-1");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ENDOFFSET,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("Move the stop line back from the intersection by the given amount"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ACCELERATION,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("Enable or disable lane as acceleration lane"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CUSTOMSHAPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::POSITION | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("If the shape is given it overrides the computation based on edge shape"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(GNE_ATTR_OPPOSITE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UNIQUE,
                                              TL("If given, this defines the opposite direction lane"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CHANGE_LEFT,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("Permit changing left only for to the given vehicle classes"),
                                              "all");
        attrProperty.setDiscreteValues(SumoVehicleClassStrings.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CHANGE_RIGHT,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("Permit changing right only for to the given vehicle classes"),
                                              "all");
        attrProperty.setDiscreteValues(SumoVehicleClassStrings.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::COPYABLE,
                                              TL("Lane type description (optional)"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(GNE_ATTR_STOPOFFSET,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The stop offset as positive value in meters"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(GNE_ATTR_STOPOEXCEPTION,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Specifies, for which vehicle classes the stopOffset does NOT apply."));
        attrProperty.setDiscreteValues(SumoVehicleClassStrings.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_CROSSING;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::NETWORKELEMENT,
                                      GNETagProperties::TagProperty::NO_PROPERTY,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::CROSSING, currentTag, TL("Crossing"));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("Crossing ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_EDGES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The (road) edges which are crossed"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PRIORITY,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Whether the pedestrians have priority over the vehicles (automatically set to true at tls-controlled intersections)"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_WIDTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The width of the crossings"),
                                              toString(OptionsCont::getOptions().getFloat("default.crossing-width")));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TLLINKINDEX,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("sets the tls-index for this crossing"),
                                              "-1");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TLLINKINDEX2,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("sets the opposite-direction tls-index for this crossing"),
                                              "-1");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CUSTOMSHAPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::POSITION | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("Overrides default shape of pedestrian crossing"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_WALKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::NETWORKELEMENT,
                                      GNETagProperties::TagProperty::NOPARAMETERS,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::WALKINGAREA, currentTag, TL("WalkingArea"));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("Walking Area ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_WIDTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The width of the WalkingArea"),
                                              toString(OptionsCont::getOptions().getFloat("default.sidewalk-width")));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LENGTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The length of the WalkingArea in meter"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SHAPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::POSITION | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("Overrides default shape of pedestrian sidewalk"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_CONNECTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::NETWORKELEMENT,
                                      GNETagProperties::TagProperty::NO_PROPERTY,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::CONNECTION, currentTag, TL("Connection"));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_FROM,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The ID of the edge the vehicles leave"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TO,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The ID of the edge the vehicles may reach when leaving 'from'"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FROM_LANE,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("the lane index of the incoming lane (numbers starting with 0)"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TO_LANE,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("the lane index of the outgoing lane (numbers starting with 0)"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PASS,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("if set, vehicles which pass this (lane-2-lane) connection) will not wait"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_KEEP_CLEAR,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("if set to false, vehicles which pass this (lane-2-lane) connection) will not worry about blocking the intersection"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CONTPOS,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set to a more than 0 value, an internal junction will be built at this position (in m)/n from the start of the internal lane for this connection"),
                                              toString(NBEdge::UNSPECIFIED_CONTPOS));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_UNCONTROLLED,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set to true, This connection will not be TLS-controlled despite its node being controlled"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_VISIBILITY_DISTANCE,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Vision distance between vehicles"),
                                              toString(NBEdge::UNSPECIFIED_VISIBILITY_DISTANCE));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TLLINKINDEX,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("sets index of this connection within the controlling traffic light"),
                                              "-1");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TLLINKINDEX2,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("sets index for the internal junction of this connection within the controlling traffic light"),
                                              "-1");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ALLOW,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Explicitly allows the given vehicle classes (not given will be not allowed)"),
                                              "all");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DISALLOW,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Explicitly disallows the given vehicle classes (not given will be allowed)"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SPEED,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("sets custom speed limit for the connection"),
                                              toString(NBEdge::UNSPECIFIED_SPEED));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LENGTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("sets custom length for the connection"),
                                              toString(NBEdge::UNSPECIFIED_LOADED_LENGTH));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CUSTOMSHAPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::POSITION | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("sets custom shape for the connection"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CHANGE_LEFT,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Permit changing left only for to the given vehicle classes"),
                                              "all");
        attrProperty.setDiscreteValues(SumoVehicleClassStrings.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CHANGE_RIGHT,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Permit changing right only for to the given vehicle classes"),
                                              "all");
        attrProperty.setDiscreteValues(SumoVehicleClassStrings.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_INDIRECT,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("if set to true, vehicles will make a turn in 2 steps"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("set a custom edge type (for applying vClass-specific speed restrictions)"));
        myTagProperties[currentTag].addAttribute(attrProperty);


        attrProperty = GNEAttributeProperties(SUMO_ATTR_DIR,
                                              GNEAttributeProperties::STRING,
                                              TL("turning direction for this connection (computed)"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_STATE,
                                              GNEAttributeProperties::STRING,
                                              TL("link state for this connection (computed)"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = GNE_TAG_INTERNAL_LANE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::INTERNALLANE,
                                      GNETagProperties::TagProperty::NO_PROPERTY,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::JUNCTION, currentTag, TL("InternalLanes"));
        //  internal lanes does't have attributes
    }
}


void
GNEAttributeCarrier::fillAdditionalElements() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    // fill additional elements
    SumoXMLTag currentTag = SUMO_TAG_BUS_STOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::STOPPINGPLACE,
                                      GNETagProperties::TagProperty::MASKSTARTENDPOS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GNETagProperties::Conflicts::POS_LANE_START | GNETagProperties::Conflicts::POS_LANE_END,
                                      GUIIcon::BUSSTOP, currentTag, TL("BusStop"),
                                      {}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of bus stop"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of the lane the bus stop shall be located at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_STARTPOS,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The begin position on the lane (the lower position on the lane) in meters"));

        myTagProperties[currentTag].addAttribute(attrProperty);
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ENDPOS,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The end position on the lane (the higher position on the lane) in meters, must be larger than startPos by more than 0.1m"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Name of busStop"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FRIENDLY_POS,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set, no error will be reported if element is placed behind the lane.") + std::string("\n") +
                                              TL("Instead, it will be placed 0.1 meters from the lanes end or at position 0.1") + std::string("\n") +
                                              TL("if the position was negative and larger than the lanes length after multiplication with - 1"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LINES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Meant to be the names of the bus lines that stop at this bus stop. This is only used for visualization purposes"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PERSON_CAPACITY,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Larger numbers of persons trying to enter will create an upstream jam on the sidewalk"),
                                              "6");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PARKING_LENGTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Optional space definition for vehicles that park at this stop"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_COLOR,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::COLOR | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The RGBA color with which the busStop shall be displayed"));
        myTagProperties[currentTag].addAttribute(attrProperty);

    }
    currentTag = SUMO_TAG_TRAIN_STOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::STOPPINGPLACE,
                                      GNETagProperties::TagProperty::MASKSTARTENDPOS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GNETagProperties::Conflicts::POS_LANE_START | GNETagProperties::Conflicts::POS_LANE_END,
                                      GUIIcon::TRAINSTOP, currentTag, TL("TrainStop"),
                                      {}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of train stop"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of the lane the train stop shall be located at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_STARTPOS,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The begin position on the lane (the lower position on the lane) in meters"));

        myTagProperties[currentTag].addAttribute(attrProperty);
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ENDPOS,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The end position on the lane (the higher position on the lane) in meters, must be larger than startPos by more than 0.1m"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Name of trainStop"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FRIENDLY_POS,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set, no error will be reported if element is placed behind the lane.") + std::string("\n") +
                                              TL("Instead, it will be placed 0.1 meters from the lanes end or at position 0.1,") + std::string("\n") +
                                              TL("if the position was negative and larger than the lanes length after multiplication with - 1"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LINES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Meant to be the names of the train lines that stop at this train stop. This is only used for visualization purposes"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PERSON_CAPACITY,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Larger numbers of persons trying to enter will create an upstream jam on the sidewalk"),
                                              "6");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PARKING_LENGTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Optional space definition for vehicles that park at this stop"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_COLOR,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::COLOR | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The RGBA color with which the trainStop shall be displayed"));
        myTagProperties[currentTag].addAttribute(attrProperty);

    }
    currentTag = SUMO_TAG_ACCESS;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT,
                                      GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::REPARENT,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GNETagProperties::Conflicts::POS_LANE,
                                      GUIIcon::ACCESS, currentTag, TL("Access"),
        {SUMO_TAG_BUS_STOP, SUMO_TAG_TRAIN_STOP}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of the lane the stop access shall be located at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The position on the lane (the lower position on the lane) in meters"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LENGTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The walking length of the access in meters"),
                                              "-1.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FRIENDLY_POS,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set, no error will be reported if element is placed behind the lane.") + std::string("\n") +
                                              TL("Instead, it will be placed 0.1 meters from the lanes end or at position 0.1,") + std::string("\n") +
                                              TL("if the position was negative and larger than the lanes length after multiplication with - 1"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

    }
    currentTag = SUMO_TAG_CONTAINER_STOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::STOPPINGPLACE,
                                      GNETagProperties::TagProperty::MASKSTARTENDPOS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GNETagProperties::Conflicts::POS_LANE_START | GNETagProperties::Conflicts::POS_LANE_END,
                                      GUIIcon::CONTAINERSTOP, currentTag, TL("ContainerStop"),
                                      {}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of container stop"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of the lane the container stop shall be located at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_STARTPOS,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The begin position on the lane (the lower position on the lane) in meters"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ENDPOS,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The end position on the lane (the higher position on the lane) in meters, must be larger than startPos by more than 0.1m"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Name of containerStop"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FRIENDLY_POS,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set, no error will be reported if element is placed behind the lane.") + std::string("\n") +
                                              TL("Instead, it will be placed 0.1 meters from the lanes end or at position 0.1,") + std::string("\n") +
                                              TL("if the position was negative and larger than the lanes length after multiplication with - 1"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LINES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("meant to be the names of the bus lines that stop at this container stop. This is only used for visualization purposes"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CONTAINER_CAPACITY,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Larger numbers of container trying to enter will create an upstream jam on the sidewalk"),
                                              "6");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PARKING_LENGTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Optional space definition for vehicles that park at this stop"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_COLOR,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::COLOR | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The RGBA color with which the containerStop shall be displayed"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_CHARGING_STATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::STOPPINGPLACE,
                                      GNETagProperties::TagProperty::MASKSTARTENDPOS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GNETagProperties::Conflicts::POS_LANE_START | GNETagProperties::Conflicts::POS_LANE_END,
                                      GUIIcon::CHARGINGSTATION, currentTag, TL("ChargingStation"),
                                      {}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of charging station"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("Lane of the charging station location"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_STARTPOS,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("Begin position in the specified lane"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ENDPOS,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("End position in the specified lane"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Name of chargingStation"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FRIENDLY_POS,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set, no error will be reported if element is placed behind the lane.") + std::string("\n") +
                                              TL("Instead, it will be placed 0.1 meters from the lanes end or at position 0.1,") + std::string("\n") +
                                              TL("if the position was negative and larger than the lanes length after multiplication with - 1"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CHARGINGPOWER,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Charging power in W"),
                                              "22000.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_EFFICIENCY,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::RANGE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Charging efficiency [0,1]"),
                                              "0.95");
        attrProperty.setRange(0, 1);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CHARGEINTRANSIT,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Enable or disable charge in transit, i.e. vehicle must or must not to stop for charging"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CHARGEDELAY,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Time delay after the vehicles has reached / stopped on the charging station, before the energy transfer (charging) begins"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CHARGETYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Battery charging type"),
                                              "normal");
        attrProperty.setDiscreteValues({"normal", "electric", "fuel"});
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_WAITINGTIME,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Waiting time before start charging"),
                                              "900.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PARKING_AREA,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Parking area the charging station is located"),
                                              "");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_PARKING_AREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::STOPPINGPLACE,
                                      GNETagProperties::TagProperty::MASKSTARTENDPOS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GNETagProperties::Conflicts::POS_LANE_START | GNETagProperties::Conflicts::POS_LANE_END,
                                      GUIIcon::PARKINGAREA, currentTag, TL("ParkingArea"),
                                      {}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of ParkingArea"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of the lane the Parking Area shall be located at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_STARTPOS,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The begin position on the lane (the lower position on the lane) in meters"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ENDPOS,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The end position on the lane (the higher position on the lane) in meters, must be larger than startPos by more than 0.1m"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPARTPOS,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("Lane position in that vehicle must depart when leaves parkingArea"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Name of parkingArea"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ACCEPTED_BADGES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Accepted badges to access this parkingArea"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ROADSIDE_CAPACITY,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL(" The number of parking spaces for road-side parking"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ONROAD,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set, vehicles will park on the road lane and thereby reducing capacity"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FRIENDLY_POS,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set, no error will be reported if element is placed behind the lane.") + std::string("\n") +
                                              TL("Instead, it will be placed 0.1 meters from the lanes end or at position 0.1,") + std::string("\n") +
                                              TL("if the position was negative and larger than the lanes length after multiplication with - 1"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_WIDTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The width of the road-side parking spaces"),
                                              toString(SUMO_const_laneWidth));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LENGTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The length of the road-side parking spaces. By default (endPos - startPos) / roadsideCapacity"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ANGLE,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::ANGLE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The angle of the road-side parking spaces relative to the lane angle, positive means clockwise"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LEFTHAND,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Enable or disable lefthand position"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

    }
    currentTag = SUMO_TAG_PARKING_SPACE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT,
                                      GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::REPARENT | GNETagProperties::TagProperty::RTREE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::PARKINGSPACE, currentTag, TL("ParkingSpace"),
        {SUMO_TAG_PARKING_AREA}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::POSITION | GNEAttributeProperties::UPDATEGEOMETRY, // virtual attribute from the combination of the actually attributes SUMO_ATTR_X, SUMO_ATTR_Y
                                              TL("The x-y-z position of the node on the plane in meters"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Name of parking space"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_WIDTH,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The width of the road-side parking spaces"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LENGTH,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The length of the road-side parking spaces"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ANGLE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The angle of the road-side parking spaces relative to the lane angle, positive means clockwise"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SLOPE,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::ANGLE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The slope of the road-side parking spaces"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

    }
    currentTag = SUMO_TAG_INDUCTION_LOOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::DETECTOR,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GNETagProperties::Conflicts::POS_LANE,
                                      GUIIcon::E1, currentTag, TL("E1 InductionLoop"),
                                      {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of E1"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The id of the lane the detector shall be laid on. The lane must be a part of the network used"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The position on the lane the detector shall be laid on in meters. The position must be a value between -1*lane's length and the lane's length"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PERIOD,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The aggregation period the values the detector collects shall be summed up"),
                                              "300.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Name of induction loop"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FILE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::FILENAME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The path to the output file"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_VTYPES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Space separated list of vehicle type ids to consider"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NEXT_EDGES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("List of edge ids that must all be part of the future route of the vehicle to qualify for detection"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DETECT_PERSONS,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Detect persons instead of vehicles (pedestrians or passengers)"),
                                              SUMOXMLDefinitions::PersonModeValues.getString(PersonMode::NONE));
        attrProperty.setDiscreteValues(SUMOXMLDefinitions::PersonModeValues.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FRIENDLY_POS,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set, no error will be reported if element is placed behind the lane.") + std::string("\n") +
                                              TL("Instead, it will be placed 0.1 meters from the lanes end or at position 0.1,") + std::string("\n") +
                                              TL("if the position was negative and larger than the lanes length after multiplication with - 1"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_LANE_AREA_DETECTOR;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::DETECTOR,
                                      GNETagProperties::TagProperty::NO_PROPERTY,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::E2, currentTag, TL("E2 LaneAreaDetector"),
                                      {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of E2"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The id of the lane the detector shall be laid on. The lane must be a part of the network used"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The position on the lane the detector shall be laid on in meters"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LENGTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The length of the detector in meters"),
                                              "10.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PERIOD,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The aggregation period the values the detector collects shall be summed up"),
                                              "300.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TLID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The traffic light that triggers aggregation when switching"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Name of lane area detector"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FILE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::FILENAME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The path to the output file"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_VTYPES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Space separated list of vehicle type ids to consider"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NEXT_EDGES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("List of edge ids that must all be part of the future route of the vehicle to qualify for detection"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DETECT_PERSONS,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Detect persons instead of vehicles (pedestrians or passengers)"),
                                              SUMOXMLDefinitions::PersonModeValues.getString(PersonMode::NONE));
        attrProperty.setDiscreteValues(SUMOXMLDefinitions::PersonModeValues.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_HALTING_TIME_THRESHOLD,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The time-based threshold that describes how much time has to pass until a vehicle is recognized as halting)"),
                                              "1.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_HALTING_SPEED_THRESHOLD,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The speed-based threshold that describes how slow a vehicle has to be to be recognized as halting) in m/s"),
                                              "1.39");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_JAM_DIST_THRESHOLD,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The maximum distance to the next standing vehicle in order to make this vehicle count as a participant to the jam in m"),
                                              "10.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FRIENDLY_POS,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set, no error will be reported if element is placed behind the lane.") + std::string("\n") +
                                              TL("Instead, it will be placed 0.1 meters from the lanes end or at position 0.1,") + std::string("\n") +
                                              TL("if the position was negative and larger than the lanes length after multiplication with - 1"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SHOW_DETECTOR,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Show detector in sumo-gui"),
                                              "1");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = GNE_TAG_MULTI_LANE_AREA_DETECTOR;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::DETECTOR,
                                      GNETagProperties::TagProperty::NO_PROPERTY,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::E2, SUMO_TAG_LANE_AREA_DETECTOR, TL("E2 MultiLaneAreaDetector"),
                                      {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of Multilane E2"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::SECUENCIAL | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The sequence of lane ids in which the detector shall be laid on"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The position on the lane the detector shall be laid on in meters"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ENDPOS,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The end position on the lane the detector shall be laid on in meters"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PERIOD,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The aggregation period the values the detector collects shall be summed up"),
                                              "300.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TLID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The traffic light that triggers aggregation when switching"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Name of Multilane E2 detector"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FILE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::FILENAME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The path to the output file"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_VTYPES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Space separated list of vehicle type ids to consider"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NEXT_EDGES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("List of edge ids that must all be part of the future route of the vehicle to qualify for detection"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DETECT_PERSONS,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Detect persons instead of vehicles (pedestrians or passengers)"),
                                              SUMOXMLDefinitions::PersonModeValues.getString(PersonMode::NONE));
        attrProperty.setDiscreteValues(SUMOXMLDefinitions::PersonModeValues.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_HALTING_TIME_THRESHOLD,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The time-based threshold that describes how much time has to pass until a vehicle is recognized as halting)"),
                                              "1.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_HALTING_SPEED_THRESHOLD,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The speed-based threshold that describes how slow a vehicle has to be to be recognized as halting) in m/s"),
                                              "1.39");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_JAM_DIST_THRESHOLD,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The maximum distance to the next standing vehicle in order to make this vehicle count as a participant to the jam in m"),
                                              "10.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FRIENDLY_POS,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set, no error will be reported if element is placed behind the lane.") + std::string("\n") +
                                              TL("Instead, it will be placed 0.1 meters from the lanes end or at position 0.1,") + std::string("\n") +
                                              TL("if the position was negative and larger than the lanes length after multiplication with - 1"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SHOW_DETECTOR,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Show detector in sumo-gui"),
                                              "1");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_ENTRY_EXIT_DETECTOR;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::DETECTOR,
                                      GNETagProperties::TagProperty::RTREE,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GNETagProperties::Conflicts::NO_ADDITIONAL_CHILDREN,
                                      GUIIcon::E3, currentTag, TL("E3 EntryExitDetector"),
                                      {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of E3"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::POSITION | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("X-Y position of detector in editor (Only used in netedit)"),
                                              "0,0"); // virtual attribute from the combination of the actually attributes SUMO_ATTR_X, SUMO_ATTR_Y
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PERIOD,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The aggregation period the values the detector collects shall be summed up"),
                                              "300.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Name of Entry Exit detector"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FILE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::FILENAME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The path to the output file"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_VTYPES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Space separated list of vehicle type ids to consider"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NEXT_EDGES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("List of edge ids that must all be part of the future route of the vehicle to qualify for detection"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DETECT_PERSONS,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Detect persons instead of vehicles (pedestrians or passengers)"),
                                              SUMOXMLDefinitions::PersonModeValues.getString(PersonMode::NONE));
        attrProperty.setDiscreteValues(SUMOXMLDefinitions::PersonModeValues.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_OPEN_ENTRY,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set to true, no error will be reported if vehicles leave the detector without first entering it"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_HALTING_TIME_THRESHOLD,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The time-based threshold that describes how much time has to pass until a vehicle is recognized as halting) in s"),
                                              "1.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_HALTING_SPEED_THRESHOLD,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The speed-based threshold that describes how slow a vehicle has to be to be recognized as halting) in m/s"),
                                              "1.39");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_EXPECT_ARRIVAL,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Whether no warning should be issued when a vehicle arrives within the detector area."),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_DET_ENTRY;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::DETECTOR,
                                      GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::REPARENT,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::E3ENTRY, currentTag, TL("E3 DetEntry"),
        {SUMO_TAG_ENTRY_EXIT_DETECTOR}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("The id of the lane the detector shall be laid on. The lane must be a part of the network used"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The position on the lane the detector shall be laid on in meters"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FRIENDLY_POS,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set, no error will be reported if element is placed behind the lane.") + std::string("\n") +
                                              TL("Instead, it will be placed 0.1 meters from the lanes end or at position 0.1,") + std::string("\n") +
                                              TL("if the position was negative and larger than the lanes length after multiplication with - 1"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

    }
    currentTag = SUMO_TAG_DET_EXIT;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::DETECTOR,
                                      GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::REPARENT,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::E3EXIT, currentTag, TL("E3 DetExit"),
        {SUMO_TAG_ENTRY_EXIT_DETECTOR}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The id of the lane the detector shall be laid on. The lane must be a part of the network used"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The position on the lane the detector shall be laid on in meters"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FRIENDLY_POS,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set, no error will be reported if element is placed behind the lane.") + std::string("\n") +
                                              TL("Instead, it will be placed 0.1 meters from the lanes end or at position 0.1,") + std::string("\n") +
                                              TL("if the position was negative and larger than the lanes length after multiplication with - 1"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

    }
    currentTag = SUMO_TAG_INSTANT_INDUCTION_LOOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::DETECTOR,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GNETagProperties::Conflicts::POS_LANE,
                                      GUIIcon::E1INSTANT, currentTag, TL("E3 DetExit"),
                                      {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of Instant Induction Loop (E1Instant)"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The id of the lane the detector shall be laid on. The lane must be a part of the network used"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The position on the lane the detector shall be laid on in meters. The position must be a value between -1*lane's length and the lane's length"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Name of instant induction loop"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FILE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::FILENAME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The path to the output file"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_VTYPES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Space separated list of vehicle type ids to consider"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NEXT_EDGES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("List of edge ids that must all be part of the future route of the vehicle to qualify for detection"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DETECT_PERSONS,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Detect persons instead of vehicles (pedestrians or passengers)"),
                                              SUMOXMLDefinitions::PersonModeValues.getString(PersonMode::NONE));
        attrProperty.setDiscreteValues(SUMOXMLDefinitions::PersonModeValues.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FRIENDLY_POS,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set, no error will be reported if element is placed behind the lane.") + std::string("\n") +
                                              TL("Instead, it will be placed 0.1 meters from the lanes end or at position 0.1,") + std::string("\n") +
                                              TL("if the position was negative and larger than the lanes length after multiplication with - 1"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_ROUTEPROBE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT,
                                      GNETagProperties::TagProperty::CENTERAFTERCREATION,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::ROUTEPROBE, currentTag, TL("RouteProbe"),
                                      {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of RouteProbe"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_EDGE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The id of an edge in the simulation network"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PERIOD,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The frequency in which to report the distribution"),
                                              "3600.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Name of route probe"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FILE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::FILENAME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The file for generated output"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_BEGIN,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The time at which to start generating output"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_VTYPES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Space separated list of vehicle type ids to consider (empty to affect all types)"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_VSS;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT,
                                      GNETagProperties::TagProperty::RTREE | GNETagProperties::TagProperty::DIALOG,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::VARIABLESPEEDSIGN, currentTag, TL("VariableSpeedSign"),
                                      {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of Variable Speed Signal"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::POSITION | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("X-Y position of detector in editor (Only used in netedit)"),
                                              "0,0"); // virtual attribute from the combination of the actually attributes SUMO_ATTR_X, SUMO_ATTR_Y
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("List of Variable Speed Sign lanes"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Name of Variable Speed Signal"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_VTYPES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Space separated list of vehicle type ids to consider (empty to affect all types)"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = GNE_TAG_VSS_SYMBOL;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT,
                                      GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOPARAMETERS | GNETagProperties::TagProperty::NOTSELECTABLE | GNETagProperties::TagProperty::SYMBOL,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::LANE, currentTag, TL("VariableSpeedSign (lane)"),
        {SUMO_TAG_VSS}, FXRGBA(210, 233, 255, 255));
    }
    currentTag = SUMO_TAG_STEP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT,
                                      GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOPARAMETERS,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::VSSSTEP, currentTag, TL("VariableSpeedSign Step"),
        {SUMO_TAG_VSS}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_TIME,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::UNIQUE,
                                              TL("Time"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SPEED,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Speed"),
                                              "13.89");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_CALIBRATOR;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::CALIBRATOR,
                                      GNETagProperties::TagProperty::DIALOG | GNETagProperties::TagProperty::CENTERAFTERCREATION,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::CALIBRATOR, currentTag, TL("Calibrator"),
                                      {}, FXRGBA(253, 255, 206, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of Calibrator"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_EDGE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The id of edge in the simulation network"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The position of the calibrator on the specified lane"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PERIOD,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The aggregation interval in which to calibrate the flows. Default is step-length"),
                                              "1.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Name of Calibrator"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ROUTEPROBE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The id of the routeProbe element from which to determine the route distribution for generated vehicles"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_OUTPUT,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::FILENAME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The output file for writing calibrator information or NULL"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_JAM_DIST_THRESHOLD,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("A threshold value to detect and clear unexpected jamming"),
                                              "0.50");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_VTYPES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("space separated list of vehicle type ids to consider (empty to affect all types)"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = GNE_TAG_CALIBRATOR_LANE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::CALIBRATOR,
                                      GNETagProperties::TagProperty::DIALOG | GNETagProperties::TagProperty::CENTERAFTERCREATION,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::CALIBRATOR, SUMO_TAG_CALIBRATOR, TL("CalibratorLane"),
                                      {}, FXRGBA(253, 255, 206, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of Calibrator"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The id of lane in the simulation network"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The position of the calibrator on the specified lane"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PERIOD,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The aggregation interval in which to calibrate the flows. Default is step-length"),
                                              "1.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Name of calibrator lane"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ROUTEPROBE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The id of the routeProbe element from which to determine the route distribution for generated vehicles"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_OUTPUT,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::FILENAME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The output file for writing calibrator information or NULL"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_JAM_DIST_THRESHOLD,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("A threshold value to detect and clear unexpected jamming"),
                                              "0.50");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_VTYPES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("space separated list of vehicle type ids to consider (empty to affect all types)"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = GNE_TAG_CALIBRATOR_FLOW;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::CALIBRATOR,
                                      GNETagProperties::TagProperty::CHILD,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::FLOW, SUMO_TAG_FLOW, TL("CalibratorFlow"),
        {SUMO_TAG_CALIBRATOR}, FXRGBA(253, 255, 206, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ROUTE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The id of the route the vehicle shall drive along"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_BEGIN,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("First calibrator flow departure time"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_END,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("End of departure interval"),
                                              "3600");
        myTagProperties[currentTag].addAttribute(attrProperty);

        // fill common vehicle attributes
        fillCommonVehicleAttributes(currentTag);

        // optional attributes (at least one must be defined)
        attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::ACTIVATABLE | GNEAttributeProperties::VTYPE,
                                              TL("The id of the vehicle type to use for this calibrator flow"),
                                              DEFAULT_VTYPE_ID);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_VEHSPERHOUR,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::ACTIVATABLE,
                                              TL("Number of vehicles per hour, equally spaced"),
                                              "1800");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SPEED,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::ACTIVATABLE,
                                              TL("Vehicle's speed"),
                                              "15.0");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_REROUTER;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT,
                                      GNETagProperties::TagProperty::RTREE | GNETagProperties::TagProperty::DIALOG,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::REROUTER, currentTag, TL("Rerouter"),
                                      {}, FXRGBA(255, 213, 213, 255));

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of Rerouter"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_EDGES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("An edge id or a list of edge ids where vehicles shall be rerouted"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::POSITION | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("X,Y position in editor (Only used in netedit)"),
                                              "0,0"); // virtual attribute from the combination of the actually attributes SUMO_ATTR_X, SUMO_ATTR_Y
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Name of Rerouter"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PROB,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::PROBABILITY | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The probability for vehicle rerouting (0-1)"),
                                              "1.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_HALTING_TIME_THRESHOLD,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The waiting time threshold (in s) that must be reached to activate rerouting (default -1 which disables the threshold)"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_VTYPES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The list of vehicle types that shall be affected by this rerouter (empty to affect all types)"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_OFF,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Whether the router should be inactive initially (and switched on in the gui)"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_OPTIONAL,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If rerouter is optional"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = GNE_TAG_REROUTER_SYMBOL;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT,
                                      GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOPARAMETERS | GNETagProperties::TagProperty::NOTSELECTABLE | GNETagProperties::TagProperty::SYMBOL,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::EDGE, currentTag, TL("Rerouter (Edge)"),
        {GNE_TAG_REROUTER_SYMBOL}, FXRGBA(255, 213, 213, 255));
    }
    currentTag = SUMO_TAG_INTERVAL;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT,
                                      GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOPARAMETERS,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::REROUTERINTERVAL, currentTag, TL("Rerouter Interval"),
        {SUMO_TAG_REROUTER}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_BEGIN,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Begin"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_END,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("End"),
                                              "3600");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_CLOSING_REROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT,
                                      GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOPARAMETERS,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::CLOSINGREROUTE, currentTag, TL("ClosingReroute"),
        {SUMO_TAG_INTERVAL}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_EDGE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::SYNONYM | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("Edge ID"));
        attrProperty.setSynonym(SUMO_ATTR_ID);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ALLOW,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("allowed vehicles"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DISALLOW,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("disallowed vehicles"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_CLOSING_LANE_REROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT,
                                      GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOPARAMETERS,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::CLOSINGLANEREROUTE, currentTag, TL("ClosingLaneReroute"),
        {SUMO_TAG_INTERVAL}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::SYNONYM | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("Lane ID"));
        attrProperty.setSynonym(SUMO_ATTR_ID);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ALLOW,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("allowed vehicles"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DISALLOW,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("disallowed vehicles"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_DEST_PROB_REROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT,
                                      GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOPARAMETERS,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::DESTPROBREROUTE, currentTag, TL("DestinationProbabilityReroute"),
        {SUMO_TAG_INTERVAL}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_EDGE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::SYNONYM | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("Edge ID"));
        attrProperty.setSynonym(SUMO_ATTR_ID);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PROB,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("SUMO Probability"),
                                              "1.00");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_PARKING_AREA_REROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT,
                                      GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOPARAMETERS,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::PARKINGZONEREROUTE, currentTag, TL("ParkingAreaReroute"),
        {SUMO_TAG_INTERVAL}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_PARKING,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::SYNONYM,
                                              TL("ParkingArea ID"));
        attrProperty.setSynonym(SUMO_ATTR_ID);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PROB,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("SUMO Probability"),
                                              "1.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_VISIBLE,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Enable or disable visibility for parking area reroutes"),
                                              "1");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_ROUTE_PROB_REROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT,
                                      GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOPARAMETERS,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::ROUTEPROBREROUTE, currentTag, TL("RouteProbabilityReroute"),
        {SUMO_TAG_INTERVAL}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ROUTE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::SYNONYM | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("Route"));
        attrProperty.setSynonym(SUMO_ATTR_ID);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PROB,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("SUMO Probability"),
                                              "1.00");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_VAPORIZER;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT,
                                      GNETagProperties::TagProperty::CENTERAFTERCREATION,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::VAPORIZER, currentTag, TL("Vaporizer"),
                                      {}, FXRGBA(253, 255, 206, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::AUTOMATICID,
                                              TL("Edge in which vaporizer is placed"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_BEGIN,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Start Time"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_END,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("End Time"),
                                              "3600");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Name of vaporizer"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
}


void
GNEAttributeCarrier::fillShapeElements() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    // fill shape ACs
    SumoXMLTag currentTag = SUMO_TAG_POLY;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::SHAPE,
                                      GNETagProperties::TagProperty::RTREE | GNETagProperties::TagProperty::CLOSESHAPE | GNETagProperties::TagProperty::GEOSHAPE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::POLY, currentTag, TL("Polygon"),
                                      {}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of the polygon"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SHAPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::POSITION | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE,
                                              TL("The shape of the polygon"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_COLOR,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::COLOR | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The RGBA color with which the polygon shall be displayed"),
                                              "red");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FILL,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("An information whether the polygon shall be filled"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LINEWIDTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The default line width for drawing an unfilled polygon"),
                                              "1");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LAYER,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The layer in which the polygon lies"),
                                              toString(Shape::DEFAULT_LAYER));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("A typename for the polygon"),
                                              toString(Shape::DEFAULT_TYPE));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Polygon's name"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_IMGFILE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::FILENAME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("A bitmap to use for rendering this polygon"),
                                              toString(Shape::DEFAULT_IMG_FILE));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_RELATIVEPATH,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Enable or disable use image file as a relative path"),
                                              toString(Shape::DEFAULT_RELATIVEPATH));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ANGLE,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::ANGLE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Angle of rendered image in degree"),
                                              toString(Shape::DEFAULT_ANGLE));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_POI;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::SHAPE,
                                      GNETagProperties::TagProperty::RTREE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::POI, currentTag, TL("PointOfInterest"),
                                      {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of the POI"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::POSITION | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY, // virtual attribute from the combination of the actually attributes SUMO_ATTR_X, SUMO_ATTR_Y
                                              TL("The position in view"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // fill Poi attributes
        fillPOIAttributes(currentTag);
    }
    currentTag = GNE_TAG_POILANE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::SHAPE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GNETagProperties::Conflicts::POS_LANE,
                                      GUIIcon::POILANE, SUMO_TAG_POI, TL("PointOfInterestLane"),
                                      {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of the POI"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("The name of the lane at which the POI is located at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The position on the named lane or in the net in meters at which the POI is located at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FRIENDLY_POS,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set, no error will be reported if element is placed behind the lane.") + std::string("\n") +
                                              TL("Instead, it will be placed 0.1 meters from the lanes end or at position 0.1,") + std::string("\n") +
                                              TL("if the position was negative and larger than the lanes length after multiplication with - 1"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION_LAT,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The lateral offset on the named lane at which the POI is located at"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        // fill Poi attributes
        fillPOIAttributes(currentTag);
    }
    currentTag = GNE_TAG_POIGEO;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::SHAPE,
                                      GNETagProperties::TagProperty::RTREE | GNETagProperties::TagProperty::REQUIRE_PROJ,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::POIGEO, SUMO_TAG_POI, TL("PointOfInterestGeo"),
                                      {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of the POI"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_LON,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The longitude position of the parking vehicle on the view"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LAT,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The latitude position of the parking vehicle on the view"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // fill Poi attributes
        fillPOIAttributes(currentTag);
    }
}


void
GNEAttributeCarrier::fillTAZElements() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    // fill TAZ ACs
    SumoXMLTag currentTag = SUMO_TAG_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::TAZELEMENT,
                                      GNETagProperties::TagProperty::RTREE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::TAZ, currentTag, TL("TrafficAssignmentZones"));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of the TAZ"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SHAPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::POSITION | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The shape of the TAZ"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CENTER,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::POSITION | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("TAZ center"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FILL,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("An information whether the TAZ shall be filled"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_COLOR,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::COLOR | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The RGBA color with which the TAZ shall be displayed"),
                                              "red");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Name of POI"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_TAZSOURCE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::TAZELEMENT,
                                      GNETagProperties::TagProperty::CHILD,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::TAZEDGE, currentTag, TL("TAZ Source"),
        {SUMO_TAG_TAZ});
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_EDGE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::SYNONYM | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The id of edge in the simulation network"));
        attrProperty.setSynonym(SUMO_ATTR_ID);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_WEIGHT,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Depart weight associated to this Edge"),
                                              "1");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_TAZSINK;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::TAZELEMENT,
                                      GNETagProperties::TagProperty::CHILD,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::TAZEDGE, currentTag, TL("TAZ Sink"),
        {SUMO_TAG_TAZ});
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_EDGE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::SYNONYM | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The id of edge in the simulation network"));
        attrProperty.setSynonym(SUMO_ATTR_ID);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_WEIGHT,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Arrival weight associated to this Edge"),
                                              "1");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
}


void
GNEAttributeCarrier::fillWireElements() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    // fill wire elements
    SumoXMLTag currentTag = SUMO_TAG_TRACTION_SUBSTATION;
    {
        // set tag properties
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::WIRE,
                                      GNETagProperties::TagProperty::RTREE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::TRACTION_SUBSTATION, currentTag, TL("TractionSubstation"));
        // set attribute properties
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("Traction substation ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::POSITION | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("X-Y position of detector in editor (Only used in netedit)"),
                                              "0,0"); // virtual attribute from the combination of the actually attributes SUMO_ATTR_X, SUMO_ATTR_Y
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_VOLTAGE,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Voltage of at connection point for the overhead wire"),
                                              "600");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CURRENTLIMIT,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Current limit of the feeder line"),
                                              "400");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_OVERHEAD_WIRE_SECTION;
    {
        // set tag properties
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::WIRE,
                                      GNETagProperties::TagProperty::NO_PROPERTY,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::OVERHEADWIRE, currentTag, TL("WireSection"));
        // set attribute properties
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("Overhead wire segment ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SUBSTATIONID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("Substation to which the circuit is connected"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE,
                                              TL("List of consecutive lanes of the circuit"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_STARTPOS,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Starting position in the specified lane"),
                                              "0.0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ENDPOS,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Ending position in the specified lane"),
                                              toString(INVALID_DOUBLE));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FRIENDLY_POS,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set, no error will be reported if element is placed behind the lane.") + std::string("\n") +
                                              TL("Instead, it will be placed 0.1 meters from the lanes end or at position 0.1,") + std::string("\n") +
                                              TL("if the position was negative and larger than the lanes length after multiplication with - 1"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_OVERHEAD_WIRE_FORBIDDEN,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST,
                                              TL("Inner lanes, where placing of overhead wire is restricted"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_OVERHEAD_WIRE_CLAMP;
    {
        // set tag properties
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::WIRE,
                                      GNETagProperties::TagProperty::NO_PROPERTY,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::OVERHEADWIRE_CLAMP, currentTag, TL("OverheadWireClamp"));
        // set attribute properties
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("Overhead wire clamp ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_OVERHEAD_WIRECLAMP_START,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("ID of the overhead wire segment, to the start of which the overhead wire clamp is connected"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_OVERHEAD_WIRECLAMP_LANESTART,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("ID of the overhead wire segment lane of overheadWireIDStartClamp"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_OVERHEAD_WIRECLAMP_END,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("ID of the overhead wire segment, to the end of which the overhead wire clamp is connected"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_OVERHEAD_WIRECLAMP_LANEEND,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("ID of the overhead wire segment lane of overheadWireIDEndClamp"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
}


void
GNEAttributeCarrier::fillJuPedSimElements() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    // fill shape ACs
    SumoXMLTag currentTag = GNE_TAG_JPS_WALKABLEAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::SHAPE | GNETagProperties::TagType::JUPEDSIM,
                                      GNETagProperties::TagProperty::RTREE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::JPS_WALKABLEAREA, SUMO_TAG_POLY, TL("JuPedSim WalkableArea"),
                                      {}, FXRGBA(253, 255, 206, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of the walkable area"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SHAPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::POSITION | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE,
                                              TL("The shape of the walkable area"));
        myTagProperties[currentTag].addAttribute(attrProperty);


        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Walkable area's name"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = GNE_TAG_JPS_OBSTACLE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::ADDITIONALELEMENT | GNETagProperties::TagType::SHAPE | GNETagProperties::TagType::JUPEDSIM,
                                      GNETagProperties::TagProperty::RTREE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::JPS_OBSTACLE, SUMO_TAG_POLY, TL("JuPedSim Obstacle"),
                                      {}, FXRGBA(253, 255, 206, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of the obstacle"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SHAPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::POSITION | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE,
                                              TL("The shape of the obstacle"));
        myTagProperties[currentTag].addAttribute(attrProperty);


        attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Obstacle's name"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
}


void
GNEAttributeCarrier::fillDemandElements() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    // fill demand elements
    SumoXMLTag currentTag = SUMO_TAG_ROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::ROUTE,
                                      GNETagProperties::TagProperty::NO_PROPERTY,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::ROUTE, currentTag, TL("Route"));

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of Route"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(GNE_ATTR_ROUTE_DISTRIBUTION,
                                              GNEAttributeProperties::STRING,
                                              TL("Route distribution"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_EDGES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The edges the vehicle shall drive along, given as their ids, separated using spaces"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_COLOR,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::COLOR | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("This route's color"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_REPEAT,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The number of times that the edges of this route shall be repeated"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CYCLETIME,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("When defining a repeating route with stops and those stops use the until attribute,") + std::string("\n") +
                                              TL("the times will be shifted forward by 'cycleTime' on each repeat"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_ROUTE_DISTRIBUTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT,
                                      GNETagProperties::TagProperty::NOTDRAWABLE | GNETagProperties::TagProperty::NOTSELECTABLE | GNETagProperties::TagProperty::NOPARAMETERS,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::ROUTEDISTRIBUTION, currentTag, TL("RouteDistribution"));

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("The id of route distribution"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = GNE_TAG_ROUTE_EMBEDDED;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::ROUTE,
                                      GNETagProperties::TagProperty::CHILD,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::ROUTE, SUMO_TAG_ROUTE, TL("RouteEmbedded"),
        {GNE_TAG_VEHICLE_WITHROUTE, GNE_TAG_FLOW_WITHROUTE});

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_EDGES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The edges the vehicle shall drive along, given as their ids, separated using spaces"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_COLOR,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::COLOR | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("This route's color"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_REPEAT,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The number of times that the edges of this route shall be repeated"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CYCLETIME,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("When defining a repeating route with stops and those stops use the until attribute,") + std::string("\n") +
                                              TL("the times will be shifted forward by 'cycleTime' on each repeat"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_VTYPE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VTYPE,
                                      GNETagProperties::TagProperty::NOTDRAWABLE | GNETagProperties::TagProperty::NOTSELECTABLE | GNETagProperties::TagProperty::VCLASS_ICON,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::VTYPE, currentTag, TL("VehicleType"));

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("type ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(GNE_ATTR_VTYPE_DISTRIBUTION,
                                              GNEAttributeProperties::STRING,
                                              TL("Type distribution"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_VCLASS,
                                              GNEAttributeProperties::VCLASS | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("An abstract vehicle class"),
                                              "passenger");
        attrProperty.setDiscreteValues(SumoVehicleClassStrings.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_COLOR,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::COLOR | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("This type's color"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LENGTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE,
                                              TL("The vehicle's netto-length (length) [m]"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_MINGAP,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE,
                                              TL("Empty space after leader [m]"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_MAXSPEED,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE,
                                              TL("The vehicle's maximum velocity [m/s]"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_SPEEDFACTOR,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::EXTENDED,
                                              TL("The vehicle's expected multiplicator for lane speed limits (or a distribution specifier)"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DESIRED_MAXSPEED,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE,
                                              TL("The vehicle's desired maximum velocity (interacts with speedFactor).") + std::string("\n") +
                                              TL("Applicable when no speed limit applies (bicycles, some motorways) [m/s]"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_EMISSIONCLASS,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::EXTENDED,
                                              TL("An abstract emission class"));
        attrProperty.setDiscreteValues(PollutantsInterface::getAllClassesStr());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_GUISHAPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::EXTENDED,
                                              TL("How this vehicle is rendered"));
        attrProperty.setDiscreteValues(SumoVehicleShapeStrings.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_WIDTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                              TL("The vehicle's width [m] (only used for drawing)"),
                                              "1.8");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_HEIGHT,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                              TL("The vehicle's height [m] (only used for drawing)"),
                                              "1.5");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PARKING_BADGES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The parking badges assigned to the vehicle"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_IMGFILE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::FILENAME | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                              TL("Image file for rendering vehicles of this type (should be grayscale to allow functional coloring)"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANE_CHANGE_MODEL,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                              TL("The model used for changing lanes"),
                                              "default");
        attrProperty.setDiscreteValues(SUMOXMLDefinitions::LaneChangeModels.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CAR_FOLLOW_MODEL,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                              TL("The model used for car-following"),
                                              "Krauss");
        attrProperty.setDiscreteValues(SUMOXMLDefinitions::CarFollowModels.getStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PERSON_CAPACITY,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::EXTENDED,
                                              TL("The number of persons (excluding an autonomous driver) the vehicle can transport"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CONTAINER_CAPACITY,
                                              GNEAttributeProperties::INT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::EXTENDED,
                                              TL("The number of containers the vehicle can transport"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_BOARDING_DURATION,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                              TL("The time required by a person to board the vehicle"),
                                              "0.50");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LOADING_DURATION,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                              TL("The time required to load a container onto the vehicle"),
                                              "90.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LATALIGNMENT,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                              TL("The preferred lateral alignment when using the sublane-model"),
                                              "center");
        attrProperty.setDiscreteValues(SUMOVTypeParameter::getLatAlignmentStrings());
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_MINGAP_LAT,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                              TL("The minimum lateral gap at a speed difference of 50km/h when using the sublane-model"),
                                              "0.12");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_MAXSPEED_LAT,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                              TL("The maximum lateral speed when using the sublane-model"),
                                              "1.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ACTIONSTEPLENGTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                              TL("The interval length for which vehicle performs its decision logic (acceleration and lane-changing)"),
                                              toString(OptionsCont::getOptions().getFloat("default.action-step-length")));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_PROB,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                              TL("The probability when being added to a distribution without an explicit probability"),
                                              toString(DEFAULT_VEH_PROB));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_OSGFILE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                              TL("3D model file for this class"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CARRIAGE_LENGTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::EXTENDED,
                                              TL("Carriage lengths"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_LOCOMOTIVE_LENGTH,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::EXTENDED,
                                              TL("Locomotive lengths"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_CARRIAGE_GAP,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                              TL("Gap between carriages"),
                                              "1");
        myTagProperties[currentTag].addAttribute(attrProperty);

        // fill VType Car Following Model Values (implemented in a separated function to improve code legibility)
        fillCarFollowingModelAttributes(currentTag);

        // fill VType Junction Model Parameters (implemented in a separated function to improve code legibility)
        fillJunctionModelAttributes(currentTag);

        // fill VType Lane Change Model Parameters (implemented in a separated function to improve code legibility)
        fillLaneChangingModelAttributes(currentTag);
    }
    currentTag = SUMO_TAG_VTYPE_DISTRIBUTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT,
                                      GNETagProperties::TagProperty::NOTDRAWABLE | GNETagProperties::TagProperty::NOTSELECTABLE | GNETagProperties::TagProperty::NOPARAMETERS,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::VTYPEDISTRIBUTION, currentTag, TL("VehicleTypeDistribution"));

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("The id of VehicleType distribution"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
}


void
GNEAttributeCarrier::fillVehicleElements() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    // fill vehicle ACs
    SumoXMLTag currentTag = SUMO_TAG_TRIP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::TagParents::VEHICLE_EDGES,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::TRIP, currentTag, TL("TripEdges"),
                                      {}, FXRGBA(253, 255, 206, 255), "trip (from-to edges)");

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The ID of trip"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::VTYPE,
                                              TL("The id of the vehicle type to use for this trip"),
                                              DEFAULT_VTYPE_ID);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FROM,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The ID of the edge the trip starts at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TO,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The ID of the edge the trip ends at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_VIA,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::LIST,
                                              TL("List of intermediate edge ids which shall be part of the trip"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // add common attributes
        fillCommonVehicleAttributes(currentTag);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPART,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The departure time of the (first) trip which is generated using this trip definition"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = GNE_TAG_TRIP_JUNCTIONS;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::TagParents::VEHICLE_JUNCTIONS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::TRIP_JUNCTIONS, SUMO_TAG_TRIP, TL("TripJunctions"),
                                      {}, FXRGBA(255, 213, 213, 255), "trip (from-to junctions)");

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of trip"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::VTYPE,
                                              TL("The id of the vehicle type to use for this trip"),
                                              DEFAULT_VTYPE_ID);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FROM_JUNCTION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of the junction the trip starts at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TO_JUNCTION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of the junction the trip ends at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // add common attributes
        fillCommonVehicleAttributes(currentTag);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPART,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The departure time of the (first) trip which is generated using this trip definition"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = GNE_TAG_TRIP_TAZS;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::TagParents::VEHICLE_TAZS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::TRIP_TAZS, SUMO_TAG_TRIP, TL("TripTAZs"),
                                      {}, FXRGBA(240, 255, 205, 255), "trip (from-to TAZs)");

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of trip"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::VTYPE,
                                              TL("The id of the vehicle type to use for this trip"),
                                              DEFAULT_VTYPE_ID);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FROM_TAZ,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of the TAZ the trip starts at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TO_TAZ,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of the TAZ the trip ends at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // add common attributes
        fillCommonVehicleAttributes(currentTag);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPART,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The departure time of the (first) trip which is generated using this trip definition"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_VEHICLE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::TagParents::VEHICLE_ROUTE,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::VEHICLE, currentTag, TL("VehicleRoute"),
                                      {}, FXRGBA(210, 233, 255, 255), "vehicle (over route)");

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The ID of the vehicle"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::VTYPE,
                                              TL("The id of the vehicle type to use for this vehicle"),
                                              DEFAULT_VTYPE_ID);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ROUTE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The id of the route the vehicle shall drive along"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPARTEDGE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The index of the edge within route the vehicle starts at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ARRIVALEDGE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The index of the edge within route the vehicle ends at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // add common attributes
        fillCommonVehicleAttributes(currentTag);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPART,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The time step at which the vehicle shall enter the network"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = GNE_TAG_VEHICLE_WITHROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::TagParents::VEHICLE_ROUTE_EMBEDDED,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::VEHICLE, SUMO_TAG_VEHICLE, TL("VehicleEmbeddedRoute"),
                                      {}, FXRGBA(210, 233, 255, 255), "vehicle (embedded route)");

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The ID of the vehicle"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::VTYPE,
                                              TL("The id of the vehicle type to use for this vehicle"),
                                              DEFAULT_VTYPE_ID);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPARTEDGE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The index of the edge within route the vehicle starts at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ARRIVALEDGE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The index of the edge within route the vehicle ends at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // add common attributes
        fillCommonVehicleAttributes(currentTag);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPART,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The time step at which the vehicle shall enter the network"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_FLOW;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLE | GNETagProperties::TagType::FLOW,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::TagParents::VEHICLE_EDGES,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::FLOW, currentTag, TL("FlowEdges"),
                                      {}, FXRGBA(253, 255, 206, 255), "flow (from-to edges)");

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The ID of the flow"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::VTYPE,
                                              TL("The id of the flow type to use for this flow"),
                                              DEFAULT_VTYPE_ID);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FROM,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The ID of the edge the flow starts at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TO,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The ID of the edge the flow ends at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_VIA,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::LIST,
                                              TL("List of intermediate edge ids which shall be part of the flow"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // add common attributes
        fillCommonVehicleAttributes(currentTag);

        // add flow attributes
        fillCommonFlowAttributes(currentTag, SUMO_ATTR_VEHSPERHOUR);
    }
    currentTag = GNE_TAG_FLOW_JUNCTIONS;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLE | GNETagProperties::TagType::FLOW,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::TagParents::VEHICLE_JUNCTIONS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::FLOW_JUNCTIONS, SUMO_TAG_FLOW, TL("FlowJunctions"),
                                      {}, FXRGBA(255, 213, 213, 255), "flow (from-to junctions)");

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of the flow"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::VTYPE,
                                              TL("The id of the flow type to use for this flow"),
                                              DEFAULT_VTYPE_ID);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FROM_JUNCTION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of the junction the flow starts at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TO_JUNCTION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of the junction the flow ends at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // add common attributes
        fillCommonVehicleAttributes(currentTag);

        // add flow attributes
        fillCommonFlowAttributes(currentTag, SUMO_ATTR_VEHSPERHOUR);
    }
    currentTag = GNE_TAG_FLOW_TAZS;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLE | GNETagProperties::TagType::FLOW,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::TagParents::VEHICLE_TAZS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::FLOW_TAZS, SUMO_TAG_FLOW, TL("FlowTAZs"),
                                      {}, FXRGBA(240, 255, 205, 255), "flow (from-to TAZs)");

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of the flow"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::VTYPE,
                                              TL("The id of the flow type to use for this flow"),
                                              DEFAULT_VTYPE_ID);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FROM_TAZ,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of the TAZ the flow starts at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TO_TAZ,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of the TAZ the flow ends at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // add common attributes
        fillCommonVehicleAttributes(currentTag);

        // add flow attributes
        fillCommonFlowAttributes(currentTag, SUMO_ATTR_VEHSPERHOUR);
    }
    currentTag = GNE_TAG_FLOW_ROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLE | GNETagProperties::TagType::FLOW,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::TagParents::VEHICLE_ROUTE,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::ROUTEFLOW, SUMO_TAG_FLOW, TL("FlowRoute"),
                                      {}, FXRGBA(210, 233, 255, 255), "flow (over route)");

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The id of the flow"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::VTYPE,
                                              TL("The id of the flow type to use for this flow"),
                                              DEFAULT_VTYPE_ID);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ROUTE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The id of the route the flow shall drive along"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPARTEDGE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The index of the edge within route the flow starts at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ARRIVALEDGE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The index of the edge within route the flow ends at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // add common attributes
        fillCommonVehicleAttributes(currentTag);

        // add flow attributes
        fillCommonFlowAttributes(currentTag, SUMO_ATTR_VEHSPERHOUR);
    }
    currentTag = GNE_TAG_FLOW_WITHROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLE | GNETagProperties::TagType::FLOW,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::TagParents::VEHICLE_ROUTE_EMBEDDED,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::ROUTEFLOW, SUMO_TAG_FLOW, TL("FlowEmbeddedRoute"),
                                      {}, FXRGBA(210, 233, 255, 255), "flow (embedded route)");

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("The name of the flow"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::VTYPE,
                                              TL("The id of the flow type to use for this flow"),
                                              DEFAULT_VTYPE_ID);
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPARTEDGE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The index of the edge within route the flow starts at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ARRIVALEDGE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The index of the edge within route the flow ends at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // add common attributes
        fillCommonVehicleAttributes(currentTag);

        // add flow attributes
        fillCommonFlowAttributes(currentTag, SUMO_ATTR_VEHSPERHOUR);
    }
}


void
GNEAttributeCarrier::fillStopElements() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    // fill stops ACs
    SumoXMLTag currentTag = GNE_TAG_STOP_LANE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLESTOP,
                                      GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::MASKSTARTENDPOS,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::STOPELEMENT, SUMO_TAG_STOP, TL("StopLane"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of the lane the stop shall be located at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_STARTPOS,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The begin position on the lane (the lower position on the lane) in meters"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ENDPOS,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The end position on the lane (the higher position on the lane) in meters, must be larger than startPos by more than 0.1m"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FRIENDLY_POS,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set, no error will be reported if element is placed behind the lane.") + std::string("\n") +
                                              TL("Instead, it will be placed 0.1 meters from the lanes end or at position 0.1,") + std::string("\n") +
                                              TL("if the position was negative and larger than the lanes length after multiplication with - 1"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION_LAT,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The lateral offset on the named lane at which the vehicle must stop"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // fill common stop attributes
        fillCommonStopAttributes(currentTag, false);
    }
    currentTag = GNE_TAG_STOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLESTOP,
                                      GNETagProperties::TagProperty::CHILD,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::STOPELEMENT, SUMO_TAG_STOP, TL("StopBusStop"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_BUS_STOP,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("BusStop associated with this stop"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // fill common stop attributes
        fillCommonStopAttributes(currentTag, false);
    }
    currentTag = GNE_TAG_STOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLESTOP,
                                      GNETagProperties::TagProperty::CHILD,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::STOPELEMENT, SUMO_TAG_STOP, TL("StopTrainStop"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_TRAIN_STOP,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("TrainStop associated with this stop"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // fill common stop attributes
        fillCommonStopAttributes(currentTag, false);
    }
    currentTag = GNE_TAG_STOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLESTOP,
                                      GNETagProperties::TagProperty::CHILD,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::STOPELEMENT, SUMO_TAG_STOP, TL("StopContainerStop"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_CONTAINER_STOP,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("ContainerStop associated with this stop"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // fill common stop attributes
        fillCommonStopAttributes(currentTag, false);
    }
    currentTag = GNE_TAG_STOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLESTOP,
                                      GNETagProperties::TagProperty::CHILD,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::STOPELEMENT, SUMO_TAG_STOP, TL("StopChargingStation"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_CHARGING_STATION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("ChargingStation associated with this stop"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // fill common stop attributes
        fillCommonStopAttributes(currentTag, false);
    }
    currentTag = GNE_TAG_STOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLESTOP,
                                      GNETagProperties::TagProperty::CHILD,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::STOPELEMENT, SUMO_TAG_STOP, TL("StopParkingArea"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_PARKING_AREA,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("ParkingArea associated with this stop"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // fill common stop attributes (no parking)
        fillCommonStopAttributes(currentTag, false);
    }
}


void
GNEAttributeCarrier::fillWaypointElements() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    // fill waypoints ACs
    SumoXMLTag currentTag = GNE_TAG_WAYPOINT_LANE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLESTOP | GNETagProperties::TagType::VEHICLEWAYPOINT,
                                      GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::MASKSTARTENDPOS,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::WAYPOINT, SUMO_TAG_STOP, TL("WaypointLane"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_LANE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of the lane the waypoint shall be located at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_STARTPOS,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The begin position on the lane (the lower position on the lane) in meters"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ENDPOS,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The end position on the lane (the higher position on the lane) in meters, must be larger than startPos by more than 0.1m"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_FRIENDLY_POS,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set, no error will be reported if element is placed behind the lane.") + std::string("\n") +
                                              TL("Instead, it will be placed 0.1 meters from the lanes end or at position 0.1,") + std::string("\n") +
                                              TL("if the position was negative and larger than the lanes length after multiplication with - 1"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_POSITION_LAT,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The lateral offset on the named lane at which the vehicle must waypoint"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // fill common waypoint (stop) attributes
        fillCommonStopAttributes(currentTag, true);
    }
    currentTag = GNE_TAG_WAYPOINT_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLESTOP | GNETagProperties::TagType::VEHICLEWAYPOINT,
                                      GNETagProperties::TagProperty::CHILD,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::WAYPOINT, SUMO_TAG_STOP, TL("WaypointBusStop"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_BUS_STOP,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("BusWaypoint associated with this waypoint"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // fill common waypoint (stop) attributes
        fillCommonStopAttributes(currentTag, true);
    }
    currentTag = GNE_TAG_WAYPOINT_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLESTOP | GNETagProperties::TagType::VEHICLEWAYPOINT,
                                      GNETagProperties::TagProperty::CHILD,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::WAYPOINT, SUMO_TAG_STOP, TL("WaypointTrainStop"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_TRAIN_STOP,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("TrainWaypoint associated with this waypoint"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // fill common waypoint (stop) attributes
        fillCommonStopAttributes(currentTag, true);
    }
    currentTag = GNE_TAG_WAYPOINT_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLESTOP | GNETagProperties::TagType::VEHICLEWAYPOINT,
                                      GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOPARAMETERS,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::WAYPOINT, SUMO_TAG_STOP, TL("WaypointContainerStop"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_CONTAINER_STOP,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("ContainerWaypoint associated with this waypoint"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // fill common waypoint (stop) attributes
        fillCommonStopAttributes(currentTag, true);
    }
    currentTag = GNE_TAG_WAYPOINT_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLESTOP | GNETagProperties::TagType::VEHICLEWAYPOINT,
                                      GNETagProperties::TagProperty::CHILD,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::WAYPOINT, SUMO_TAG_STOP, TL("WaypointChargingStation"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_CHARGING_STATION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("ChargingStation associated with this waypoint"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // fill common waypoint (stop) attributes
        fillCommonStopAttributes(currentTag, true);
    }
    currentTag = GNE_TAG_WAYPOINT_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::VEHICLESTOP | GNETagProperties::TagType::VEHICLEWAYPOINT,
                                      GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOPARAMETERS,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::WAYPOINT, SUMO_TAG_STOP, TL("WaypointParkingArea"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_PARKING_AREA,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("ParkingArea associated with this waypoint"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // fill common waypoint (stop) attributes
        fillCommonStopAttributes(currentTag, true);
    }
}


void
GNEAttributeCarrier::fillPersonElements() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    // fill vehicle ACs
    SumoXMLTag currentTag = SUMO_TAG_PERSON;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::PERSON,
                                      GNETagProperties::TagProperty::NO_PROPERTY,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::PERSON, currentTag, TL("Person"));

        // add flow attributes
        fillCommonPersonAttributes(currentTag);

        // set specific attribute depart (note: Persons doesn't support triggered and containerTriggered values)
        attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPART,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The time step at which the person shall enter the network"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);

    }
    currentTag = SUMO_TAG_PERSONFLOW;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::PERSON | GNETagProperties::TagType::FLOW,
                                      GNETagProperties::TagProperty::NO_PROPERTY,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::PERSONFLOW, currentTag, TL("PersonFlow"));

        // add flow attributes
        fillCommonPersonAttributes(currentTag);

        // add flow attributes
        fillCommonFlowAttributes(currentTag, SUMO_ATTR_PERSONSPERHOUR);
    }
}


void
GNEAttributeCarrier::fillContainerElements() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    // fill vehicle ACs
    SumoXMLTag currentTag = SUMO_TAG_CONTAINER;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::CONTAINER,
                                      GNETagProperties::TagProperty::NO_PROPERTY,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::CONTAINER, currentTag, TL("Container"));

        // add flow attributes
        fillCommonContainerAttributes(currentTag);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPART,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("The time step at which the container shall enter the network"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_CONTAINERFLOW;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::CONTAINER | GNETagProperties::TagType::FLOW,
                                      GNETagProperties::TagProperty::NO_PROPERTY,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::CONTAINERFLOW, currentTag, TL("ContainerFlow"));

        // add common container attribute
        fillCommonContainerAttributes(currentTag);

        // add flow attributes
        fillCommonFlowAttributes(currentTag, SUMO_ATTR_CONTAINERSPERHOUR);
    }
}


void
GNEAttributeCarrier::fillContainerTransportElements() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;
    // declare common tag types and properties
    const int tagType = GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::CONTAINERPLAN | GNETagProperties::TagType::TRANSPORT;
    const int tagProperty = GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOPARAMETERS;
    const int tagPropertyTAZ = GNETagProperties::TagProperty::RTREE | tagProperty;
    const int conflicts = GNETagProperties::Conflicts::NO_CONFLICTS;
    const std::vector<SumoXMLTag> parents({SUMO_TAG_CONTAINER, SUMO_TAG_CONTAINERFLOW});
    const unsigned int color = FXRGBA(240, 255, 205, 255);
    const GUIIcon icon = GUIIcon::TRANSPORT_EDGE;
    const SumoXMLTag xmlTag = SUMO_TAG_TRANSPORT;
    // fill merged tag
    myMergedPlanTagProperties[xmlTag] = GNETagProperties(xmlTag, tagType, tagProperty,
                                        0,
                                        conflicts, icon, xmlTag, TL("Container"), parents, color);
    // set values of attributes
    fillTransportCommonAttributes(myMergedPlanTagProperties[xmlTag]);
    // from edge
    SumoXMLTag currentTag = GNE_TAG_TRANSPORT_EDGE_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("edge"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_EDGE_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("edge"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_EDGE_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("edge"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_EDGE_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("edge"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_EDGE_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("edge"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_EDGE_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("edge"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_EDGE_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("edge"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_EDGE_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("edge"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    // from taz
    currentTag = GNE_TAG_TRANSPORT_TAZ_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("taz"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TAZ_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("taz"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TAZ_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("taz"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TAZ_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("taz"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TAZ_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("taz"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TAZ_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("taz"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TAZ_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("taz"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TAZ_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("taz"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    // from junction
    currentTag = GNE_TAG_TRANSPORT_JUNCTION_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("junction"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_JUNCTION_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("junction"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_JUNCTION_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("junction"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_JUNCTION_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("junction"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_JUNCTION_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("junction"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_JUNCTION_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("junction"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_JUNCTION_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("junction"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_JUNCTION_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("junction"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    // from busStop
    currentTag = GNE_TAG_TRANSPORT_BUSSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("busStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_BUSSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("busStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_BUSSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("busStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_BUSSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("busStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_BUSSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("busStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_BUSSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("busStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_BUSSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("busStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_BUSSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("busStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    // from trainStop
    currentTag = GNE_TAG_TRANSPORT_TRAINSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("trainStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TRAINSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("trainStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TRAINSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("trainStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TRAINSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("trainStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TRAINSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("trainStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TRAINSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("trainStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TRAINSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("trainStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TRAINSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("trainStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    // from containerStop
    currentTag = GNE_TAG_TRANSPORT_CONTAINERSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("containerStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CONTAINERSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("containerStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CONTAINERSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("containerStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CONTAINERSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("containerStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CONTAINERSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("containerStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CONTAINERSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("containerStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CONTAINERSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("containerStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CONTAINERSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("containerStop"), TL("parkingArea")), parents, color);

        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    // from chargingStation
    currentTag = GNE_TAG_TRANSPORT_CHARGINGSTATION_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("chargingStation"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CHARGINGSTATION_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("chargingStation"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CHARGINGSTATION_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("chargingStation"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CHARGINGSTATION_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("chargingStation"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CHARGINGSTATION_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("chargingStation"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CHARGINGSTATION_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("chargingStation"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CHARGINGSTATION_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("chargingStation"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CHARGINGSTATION_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("chargingStation"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    // from parkingArea
    currentTag = GNE_TAG_TRANSPORT_PARKINGAREA_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("parkingArea"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_PARKINGAREA_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("parkingArea"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_PARKINGAREA_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("parkingArea"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_PARKINGAREA_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("parkingArea"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_PARKINGAREA_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("parkingArea"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_PARKINGAREA_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("parkingArea"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_PARKINGAREA_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("parkingArea"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_PARKINGAREA_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("parkingArea"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
}


void
GNEAttributeCarrier::fillContainerTranshipElements() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;
    // declare common tag types and properties
    const int tagType = GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::CONTAINERPLAN | GNETagProperties::TagType::TRANSHIP;
    const int tagProperty = GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOPARAMETERS;
    const int tagPropertyTAZ = GNETagProperties::TagProperty::RTREE | tagProperty;
    const int conflicts = GNETagProperties::Conflicts::NO_CONFLICTS;
    const std::vector<SumoXMLTag> parents({SUMO_TAG_CONTAINER, SUMO_TAG_CONTAINERFLOW});
    const unsigned int color = FXRGBA(210, 233, 255, 255);
    const GUIIcon icon = GUIIcon::TRANSHIP_EDGES;
    const SumoXMLTag xmlTag = SUMO_TAG_TRANSHIP;
    // fill merged tag
    myMergedPlanTagProperties[xmlTag] = GNETagProperties(xmlTag, tagType, tagProperty,
                                        0,
                                        conflicts, icon, xmlTag, TL("Tranship"), parents, color);
    // set values of attributes
    fillTranshipCommonAttributes(myMergedPlanTagProperties[xmlTag]);
    // fill tags
    SumoXMLTag currentTag = GNE_TAG_TRANSHIP_EDGES;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_CONSECUTIVE_EDGES,
                                      conflicts, icon, xmlTag, TL("Tranship: edges"), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    // from edge
    currentTag = GNE_TAG_TRANSHIP_EDGE_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("edge"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_EDGE_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("edge"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_EDGE_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("edge"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_EDGE_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("edge"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_EDGE_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("edge"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_EDGE_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("edge"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_EDGE_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("edge"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_EDGE_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("edge"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    // from taz
    currentTag = GNE_TAG_TRANSHIP_TAZ_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("taz"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TAZ_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("taz"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TAZ_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("taz"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TAZ_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("taz"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TAZ_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("taz"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TAZ_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("taz"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TAZ_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("taz"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TAZ_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("taz"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    // from junction
    currentTag = GNE_TAG_TRANSHIP_JUNCTION_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("junction"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_JUNCTION_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("junction"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_JUNCTION_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("junction"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_JUNCTION_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("junction"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_JUNCTION_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("junction"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_JUNCTION_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("junction"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_JUNCTION_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("junction"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_JUNCTION_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("junction"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    // from busStop
    currentTag = GNE_TAG_TRANSHIP_BUSSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("busStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_BUSSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("busStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_BUSSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("busStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_BUSSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("busStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_BUSSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("busStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_BUSSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("busStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_BUSSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("busStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_BUSSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("busStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    // from trainStop
    currentTag = GNE_TAG_TRANSHIP_TRAINSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("trainStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TRAINSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("trainStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TRAINSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("trainStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TRAINSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("trainStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TRAINSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("trainStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TRAINSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("trainStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TRAINSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("trainStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TRAINSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("trainStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    // from containerStop
    currentTag = GNE_TAG_TRANSHIP_CONTAINERSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("containerStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CONTAINERSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("containerStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CONTAINERSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("containerStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CONTAINERSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("containerStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CONTAINERSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("containerStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CONTAINERSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("containerStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CONTAINERSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("containerStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CONTAINERSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("containerStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    // from chargingStation
    currentTag = GNE_TAG_TRANSHIP_CHARGINGSTATION_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("chargingStation"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CHARGINGSTATION_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("chargingStation"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CHARGINGSTATION_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("chargingStation"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CHARGINGSTATION_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("chargingStation"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CHARGINGSTATION_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("chargingStation"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CHARGINGSTATION_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("chargingStation"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CHARGINGSTATION_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("chargingStation"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CHARGINGSTATION_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("chargingStation"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    // from parkingArea
    currentTag = GNE_TAG_TRANSHIP_PARKINGAREA_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("parkingArea"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_PARKINGAREA_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("parkingArea"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_PARKINGAREA_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("parkingArea"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_PARKINGAREA_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("parkingArea"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_PARKINGAREA_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("parkingArea"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_PARKINGAREA_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("parkingArea"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_PARKINGAREA_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("parkingArea"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_PARKINGAREA_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("parkingArea"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
}


void
GNEAttributeCarrier::fillContainerStopElements() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;
    // declare common tag types and properties
    const int tagType = GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::CONTAINERPLAN | GNETagProperties::TagType::STOPCONTAINER;
    const int tagProperty = GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOPARAMETERS;
    const int conflicts = GNETagProperties::Conflicts::NO_CONFLICTS;
    const std::vector<SumoXMLTag> parents({SUMO_TAG_CONTAINER, SUMO_TAG_CONTAINERFLOW});
    const unsigned int color = FXRGBA(255, 213, 213, 255);
    const GUIIcon icon = GUIIcon::STOPELEMENT;
    const SumoXMLTag xmlTag = SUMO_TAG_STOP;
    // fill merged tag
    myMergedPlanTagProperties[GNE_TAG_STOPCONTAINER] = GNETagProperties(GNE_TAG_STOPCONTAINER, tagType, tagProperty,
            0,
            conflicts, icon, xmlTag, TL("ContainerStop"), parents, color);
    // set values of attributes
    fillPlanStopCommonAttributes(myMergedPlanTagProperties[GNE_TAG_STOPCONTAINER]);
    // fill tags
    SumoXMLTag currentTag = GNE_TAG_STOPCONTAINER_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_EDGE,
                                      conflicts, icon, xmlTag, TL("ContainerStop: edge"), parents, color);

        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPCONTAINER_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_BUSSTOP,
                                      conflicts, icon, xmlTag, TL("ContainerStop: busStop"), parents, color);

        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPCONTAINER_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_TRAINSTOP,
                                      conflicts, icon, xmlTag, TL("ContainerStop: trainStop"), parents, color);

        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPCONTAINER_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, TL("ContainerStop: containerStop"), parents, color);

        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPCONTAINER_CHARGINGSTATION;
    {
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, TL("ContainerStop: chargingStation"), parents, color);

        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPCONTAINER_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_PARKINGAREA,
                                      conflicts, icon, xmlTag, TL("ContainerStop: parkingArea"), parents, color);

        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
}


void
GNEAttributeCarrier::fillPersonPlanTrips() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;
    // declare common tag types and properties
    const int tagType = GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::PERSONPLAN | GNETagProperties::TagType::PERSONTRIP;
    const int tagProperty = GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOPARAMETERS;
    const int tagPropertyTAZ = GNETagProperties::TagProperty::RTREE | tagProperty;
    const int conflicts = GNETagProperties::Conflicts::NO_CONFLICTS;
    const std::vector<SumoXMLTag> parents({SUMO_TAG_PERSON, SUMO_TAG_PERSONFLOW});
    const unsigned int color = FXRGBA(253, 255, 206, 255);
    const GUIIcon icon = GUIIcon::PERSONTRIP_EDGE;
    const SumoXMLTag xmlTag = SUMO_TAG_PERSONTRIP;
    // fill merged tag
    myMergedPlanTagProperties[xmlTag] = GNETagProperties(xmlTag, tagType, tagProperty,
                                        0,
                                        conflicts, icon, xmlTag, TL("PersonTrip"), parents, color);
    // set values of attributes
    fillPersonTripCommonAttributes(myMergedPlanTagProperties[xmlTag]);
    // from edge
    SumoXMLTag currentTag = GNE_TAG_PERSONTRIP_EDGE_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("edge"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_EDGE_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("edge"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_EDGE_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("edge"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_EDGE_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("edge"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_EDGE_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("edge"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_EDGE_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("edge"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_EDGE_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("edge"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_EDGE_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("edge"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    // from taz
    currentTag = GNE_TAG_PERSONTRIP_TAZ_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("taz"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TAZ_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("taz"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TAZ_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("taz"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TAZ_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("taz"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TAZ_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("taz"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TAZ_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("taz"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TAZ_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("taz"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TAZ_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("taz"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    // from junction
    currentTag = GNE_TAG_PERSONTRIP_JUNCTION_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("junction"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_JUNCTION_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("junction"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_JUNCTION_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("junction"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_JUNCTION_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("junction"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_JUNCTION_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("junction"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_JUNCTION_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("junction"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_JUNCTION_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("junction"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_JUNCTION_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("junction"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    // from busStop
    currentTag = GNE_TAG_PERSONTRIP_BUSSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("busStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_BUSSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("busStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_BUSSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("busStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_BUSSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("busStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_BUSSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("busStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_BUSSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("busStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_BUSSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("busStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_BUSSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("busStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    // from trainStop
    currentTag = GNE_TAG_PERSONTRIP_TRAINSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("trainStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TRAINSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("trainStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TRAINSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("trainStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TRAINSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("trainStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TRAINSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("trainStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TRAINSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("trainStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TRAINSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("trainStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TRAINSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("trainStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    // from containerStop
    currentTag = GNE_TAG_PERSONTRIP_CONTAINERSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("containerStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CONTAINERSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("containerStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CONTAINERSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("containerStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CONTAINERSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("containerStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CONTAINERSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("containerStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CONTAINERSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("containerStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CONTAINERSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("containerStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CONTAINERSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("containerStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    // from chargingStation
    currentTag = GNE_TAG_PERSONTRIP_CHARGINGSTATION_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("chargingStation"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CHARGINGSTATION_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("chargingStation"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CHARGINGSTATION_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("chargingStation"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CHARGINGSTATION_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("chargingStation"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CHARGINGSTATION_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("chargingStation"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CHARGINGSTATION_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("chargingStation"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CHARGINGSTATION_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("chargingStation"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CHARGINGSTATION_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("chargingStation"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    // from parkingArea
    currentTag = GNE_TAG_PERSONTRIP_PARKINGAREA_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("parkingArea"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_PARKINGAREA_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("parkingArea"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_PARKINGAREA_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("parkingArea"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_PARKINGAREA_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("parkingArea"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_PARKINGAREA_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("parkingArea"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_PARKINGAREA_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("parkingArea"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_PARKINGAREA_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("parkingArea"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_PARKINGAREA_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("parkingArea"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
}


void
GNEAttributeCarrier::fillPersonPlanWalks() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;
    // declare common tag types and properties
    const int tagType = GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::PERSONPLAN | GNETagProperties::TagType::WALK;
    const int tagProperty = GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOPARAMETERS;
    const int tagPropertyTAZ = GNETagProperties::TagProperty::RTREE | tagProperty;
    const int conflicts = GNETagProperties::Conflicts::NO_CONFLICTS;
    const std::vector<SumoXMLTag> parents({SUMO_TAG_PERSON, SUMO_TAG_PERSONFLOW});
    const unsigned int color = FXRGBA(240, 255, 205, 255);
    const GUIIcon icon = GUIIcon::WALK_EDGES;
    const SumoXMLTag xmlTag = SUMO_TAG_WALK;
    // fill merged tag
    myMergedPlanTagProperties[xmlTag] = GNETagProperties(xmlTag, tagType, tagProperty,
                                        0,
                                        conflicts, icon, xmlTag, TL("Walk"), parents, color);
    // set values of attributes
    fillWalkCommonAttributes(myMergedPlanTagProperties[xmlTag]);
    // fill tags
    SumoXMLTag currentTag = GNE_TAG_WALK_EDGES;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_CONSECUTIVE_EDGES,
                                      conflicts, icon, xmlTag, TL("Walk: edges"), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_ROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_ROUTE,
                                      conflicts, icon, xmlTag, TL("Walk: route"), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    // from edge
    currentTag = GNE_TAG_WALK_EDGE_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("edge"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_EDGE_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("edge"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_EDGE_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("edge"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_EDGE_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("edge"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_EDGE_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("edge"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_EDGE_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("edge"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_EDGE_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("edge"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_EDGE_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("edge"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    // from taz
    currentTag = GNE_TAG_WALK_TAZ_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("taz"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TAZ_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("taz"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TAZ_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("taz"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TAZ_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("taz"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TAZ_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("taz"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TAZ_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("taz"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TAZ_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("taz"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TAZ_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("taz"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    // from junction
    currentTag = GNE_TAG_WALK_JUNCTION_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("junction"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_JUNCTION_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("junction"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_JUNCTION_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("junction"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_JUNCTION_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("junction"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_JUNCTION_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("junction"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_JUNCTION_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("junction"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_JUNCTION_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("junction"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_JUNCTION_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("junction"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    // from busStop
    currentTag = GNE_TAG_WALK_BUSSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("busStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_BUSSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("busStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_BUSSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("busStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_BUSSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("busStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_BUSSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("busStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_BUSSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("busStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_BUSSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("busStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_BUSSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("busStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    // from trainStop
    currentTag = GNE_TAG_WALK_TRAINSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("trainStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TRAINSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("trainStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TRAINSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("trainStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TRAINSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("trainStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TRAINSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("trainStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TRAINSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("trainStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TRAINSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("trainStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TRAINSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("trainStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    // from containerStop
    currentTag = GNE_TAG_WALK_CONTAINERSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("containerStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CONTAINERSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("containerStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CONTAINERSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("containerStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CONTAINERSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("containerStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CONTAINERSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("containerStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CONTAINERSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("containerStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CONTAINERSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("containerStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CONTAINERSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("containerStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    // from chargingStation
    currentTag = GNE_TAG_WALK_CHARGINGSTATION_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("chargingStation"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CHARGINGSTATION_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("chargingStation"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CHARGINGSTATION_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("chargingStation"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CHARGINGSTATION_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("chargingStation"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CHARGINGSTATION_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("chargingStation"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CHARGINGSTATION_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("chargingStation"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CHARGINGSTATION_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("chargingStation"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CHARGINGSTATION_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("chargingStation"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    // from parkingArea
    currentTag = GNE_TAG_WALK_PARKINGAREA_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("parkingArea"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_PARKINGAREA_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("parkingArea"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_PARKINGAREA_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("parkingArea"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_PARKINGAREA_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("parkingArea"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_PARKINGAREA_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("parkingArea"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_PARKINGAREA_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("parkingArea"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_PARKINGAREA_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("parkingArea"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_PARKINGAREA_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("parkingArea"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
}


void
GNEAttributeCarrier::fillPersonPlanRides() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;
    // declare common tag types and properties
    const int tagType = GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::PERSONPLAN | GNETagProperties::TagType::RIDE;
    const int tagProperty = GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOPARAMETERS;
    const int tagPropertyTAZ = GNETagProperties::TagProperty::RTREE | tagProperty;
    const int conflicts = GNETagProperties::Conflicts::NO_CONFLICTS;
    const std::vector<SumoXMLTag> parents({SUMO_TAG_PERSON, SUMO_TAG_PERSONFLOW});
    const unsigned int color = FXRGBA(253, 255, 206, 255);
    const GUIIcon icon = GUIIcon::RIDE_EDGE;
    const SumoXMLTag xmlTag = SUMO_TAG_RIDE;
    // fill merged tag
    myMergedPlanTagProperties[xmlTag] = GNETagProperties(xmlTag, tagType, tagProperty,
                                        0,
                                        conflicts, icon, xmlTag, TL("PersonTrip"), parents, color);
    // set values of attributes
    fillRideCommonAttributes(myMergedPlanTagProperties[xmlTag]);
    // from edge
    SumoXMLTag currentTag = GNE_TAG_RIDE_EDGE_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("edge"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_EDGE_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("edge"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_EDGE_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("edge"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_EDGE_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("edge"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_EDGE_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("edge"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_EDGE_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("edge"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_EDGE_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("edge"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_EDGE_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_EDGE | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("edge"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    // from taz
    currentTag = GNE_TAG_RIDE_TAZ_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("taz"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TAZ_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("taz"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TAZ_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("taz"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TAZ_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("taz"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TAZ_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("taz"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TAZ_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("taz"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TAZ_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("taz"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TAZ_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TAZ | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("taz"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    // from junction
    currentTag = GNE_TAG_RIDE_JUNCTION_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("junction"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_JUNCTION_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("junction"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_JUNCTION_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("junction"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_JUNCTION_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("junction"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_JUNCTION_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("junction"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_JUNCTION_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("junction"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_JUNCTION_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("junction"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_JUNCTION_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_JUNCTION | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("junction"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    // from busStop
    currentTag = GNE_TAG_RIDE_BUSSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("busStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_BUSSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("busStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_BUSSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("busStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_BUSSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("busStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_BUSSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("busStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_BUSSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("busStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_BUSSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("busStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_BUSSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_BUSSTOP | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("busStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    // from trainStop
    currentTag = GNE_TAG_RIDE_TRAINSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("trainStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TRAINSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("trainStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TRAINSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("trainStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TRAINSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("trainStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TRAINSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("trainStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TRAINSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("trainStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TRAINSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("trainStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TRAINSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_TRAINSTOP | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("trainStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    // from containerStop
    currentTag = GNE_TAG_RIDE_CONTAINERSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("containerStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CONTAINERSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("containerStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CONTAINERSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("containerStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CONTAINERSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("containerStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CONTAINERSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("containerStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CONTAINERSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("containerStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CONTAINERSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("containerStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CONTAINERSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CONTAINERSTOP | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("containerStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    // from chargingStation
    currentTag = GNE_TAG_RIDE_CHARGINGSTATION_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("chargingStation"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CHARGINGSTATION_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("chargingStation"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CHARGINGSTATION_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("chargingStation"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CHARGINGSTATION_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("chargingStation"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CHARGINGSTATION_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("chargingStation"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CHARGINGSTATION_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("chargingStation"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CHARGINGSTATION_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("chargingStation"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CHARGINGSTATION_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_CHARGINGSTATION | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("chargingStation"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    // from parkingArea
    currentTag = GNE_TAG_RIDE_PARKINGAREA_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_EDGE,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("parkingArea"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_PARKINGAREA_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagPropertyTAZ,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_TAZ,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("parkingArea"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_PARKINGAREA_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_JUNCTION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("parkingArea"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_PARKINGAREA_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_BUSSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("parkingArea"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_PARKINGAREA_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_TRAINSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("parkingArea"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_PARKINGAREA_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("parkingArea"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_PARKINGAREA_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("parkingArea"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_PARKINGAREA_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_FROM_PARKINGAREA | GNETagProperties::TagParents::PLAN_TO_PARKINGAREA,
                                      conflicts, icon, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("parkingArea"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
}


void
GNEAttributeCarrier::fillPersonStopElements() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;
    // declare common tag types and properties
    const int tagType = GNETagProperties::TagType::DEMANDELEMENT | GNETagProperties::TagType::PERSONPLAN | GNETagProperties::TagType::STOPPERSON;
    const int tagProperty = GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOPARAMETERS;
    const int conflicts = GNETagProperties::Conflicts::NO_CONFLICTS;
    const std::vector<SumoXMLTag> parents({SUMO_TAG_PERSON, SUMO_TAG_PERSONFLOW});
    const unsigned int color = FXRGBA(255, 213, 213, 255);
    const GUIIcon icon = GUIIcon::STOPELEMENT;
    const SumoXMLTag xmlTag = SUMO_TAG_STOP;
    // fill merged tag
    myMergedPlanTagProperties[GNE_TAG_STOPPERSON] = GNETagProperties(GNE_TAG_STOPPERSON, tagType, tagProperty,
            0,
            conflicts, icon, xmlTag, TL("PersonStop"), parents, color);
    // set values of attributes
    fillPlanStopCommonAttributes(myMergedPlanTagProperties[GNE_TAG_STOPPERSON]);
    // fill tags
    SumoXMLTag currentTag = GNE_TAG_STOPPERSON_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_EDGE,
                                      conflicts, icon, xmlTag, TL("PersonStop: edge"), parents, color);

        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPPERSON_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_BUSSTOP,
                                      conflicts, icon, xmlTag, TL("PersonStop: busStop"), parents, color);

        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPPERSON_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_TRAINSTOP,
                                      conflicts, icon, xmlTag, TL("PersonStop: trainStop"), parents, color);

        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPPERSON_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_CONTAINERSTOP,
                                      conflicts, icon, xmlTag, TL("PersonStop: containerStop"), parents, color);

        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPPERSON_CHARGINGSTATION;
    {
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_CHARGINGSTATION,
                                      conflicts, icon, xmlTag, TL("PersonStop: chargingStation"), parents, color);

        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPPERSON_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag, tagType, tagProperty,
                                      GNETagProperties::TagParents::PLAN_PARKINGAREA,
                                      conflicts, icon, xmlTag, TL("PersonStop: parkingArea"), parents, color);

        // set values of attributes
        fillPlanParentAttributes(currentTag);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
}


void
GNEAttributeCarrier::fillPOIAttributes(SumoXMLTag currentTag) {

    GNEAttributeProperties attrProperty = GNEAttributeProperties(SUMO_ATTR_COLOR,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::COLOR | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("The color with which the POI shall be displayed"),
                                          "red");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("A typename for the POI"),
                                          toString(Shape::DEFAULT_TYPE));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_ICON,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("POI Icon"),
                                          SUMOXMLDefinitions::POIIcons.getString(POIIcon::NONE));
    attrProperty.setDiscreteValues(SUMOXMLDefinitions::POIIcons.getStrings());
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_NAME,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Name of POI"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LAYER,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("The layer of the POI for drawing and selecting"),
                                          toString(Shape::DEFAULT_LAYER_POI));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_WIDTH,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Width of rendered image in meters"),
                                          toString(Shape::DEFAULT_IMG_WIDTH));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_HEIGHT,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Height of rendered image in meters"),
                                          toString(Shape::DEFAULT_IMG_HEIGHT));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_IMGFILE,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::FILENAME | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("A bitmap to use for rendering this POI"),
                                          toString(Shape::DEFAULT_IMG_FILE));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_RELATIVEPATH,
                                          GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Enable or disable use image file as a relative path"),
                                          toString(Shape::DEFAULT_RELATIVEPATH));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_ANGLE,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::ANGLE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Angle of rendered image in degree"),
                                          toString(Shape::DEFAULT_ANGLE));
    myTagProperties[currentTag].addAttribute(attrProperty);
}


void
GNEAttributeCarrier::fillCommonVehicleAttributes(SumoXMLTag currentTag) {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    attrProperty = GNEAttributeProperties(SUMO_ATTR_COLOR,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::COLOR | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("This vehicle's color"),
                                          "yellow");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPARTLANE,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE |  GNEAttributeProperties::UPDATEGEOMETRY,
                                          TL("The lane on which the vehicle shall be inserted"),
                                          "first");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPARTPOS,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE |  GNEAttributeProperties::UPDATEGEOMETRY,
                                          TL("The position at which the vehicle shall enter the net"),
                                          "base");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPARTSPEED,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("The speed with which the vehicle shall enter the network"),
                                          "0.00");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_ARRIVALLANE,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE |  GNEAttributeProperties::UPDATEGEOMETRY,
                                          TL("The lane at which the vehicle shall leave the network"),
                                          "current");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_ARRIVALPOS,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE |  GNEAttributeProperties::UPDATEGEOMETRY,
                                          TL("The position at which the vehicle shall leave the network"),
                                          "max");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_ARRIVALSPEED,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("The speed with which the vehicle shall leave the network"),
                                          "current");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LINE,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("A string specifying the id of a public transport line which can be used when specifying person rides"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_PERSON_NUMBER,
                                          GNEAttributeProperties::INT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("The number of occupied seats when the vehicle is inserted"),
                                          "0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CONTAINER_NUMBER,
                                          GNEAttributeProperties::INT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("The number of occupied container places when the vehicle is inserted"),
                                          "0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPARTPOS_LAT,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE |  GNEAttributeProperties::UPDATEGEOMETRY,
                                          TL("The lateral position on the departure lane at which the vehicle shall enter the net"),
                                          "center");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_ARRIVALPOS_LAT,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE |  GNEAttributeProperties::UPDATEGEOMETRY,
                                          TL("The lateral position on the arrival lane at which the vehicle shall arrive"),
                                          "center");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_INSERTIONCHECKS,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Insertion checks"),
                                          SUMOXMLDefinitions::InsertionChecks.getString(InsertionCheck::ALL));
    myTagProperties[currentTag].addAttribute(attrProperty);
}


void
GNEAttributeCarrier::fillCommonFlowAttributes(SumoXMLTag currentTag, SumoXMLAttr perHour) {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    attrProperty = GNEAttributeProperties(SUMO_ATTR_BEGIN,
                                          GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("First flow departure time"),
                                          "0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_END,
                                          GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::FLOWDEFINITION,
                                          TL("End of departure interval"),
                                          "3600");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_NUMBER,
                                          GNEAttributeProperties::INT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::FLOWDEFINITION,
                                          TL("probability for emitting a flow each second") + std::string("\n") +
                                          TL("(not together with vehsPerHour or period)"),
                                          "1800");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(perHour,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::FLOWDEFINITION,
                                          TL("Number of flows per hour, equally spaced") + std::string("\n") +
                                          TL("(not together with period or probability or poisson)"),
                                          "1800");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_PERIOD,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::FLOWDEFINITION,
                                          TL("Insert equally spaced flows at that period") + std::string("\n") +
                                          TL("(not together with vehsPerHour or probability or poisson)"),
                                          "2");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_PROB,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::FLOWDEFINITION,
                                          TL("probability for emitting a flow each second") + std::string("\n") +
                                          TL("(not together with vehsPerHour or period or poisson)"),
                                          "0.5");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(GNE_ATTR_POISSON,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::FLOWDEFINITION,
                                          TL("Insert flow expected vehicles per second with poisson distributed insertion rate") + std::string("\n") +
                                          TL("(not together with period or vehsPerHour or probability)"),
                                          "0.5");
    myTagProperties[currentTag].addAttribute(attrProperty);
}


void
GNEAttributeCarrier::fillCarFollowingModelAttributes(SumoXMLTag currentTag) {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    attrProperty = GNEAttributeProperties(SUMO_ATTR_ACCEL,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("The acceleration ability of vehicles of this type [m/s^2]"),
                                          "2.60");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_DECEL,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("The deceleration ability of vehicles of this type [m/s^2]"),
                                          "4.50");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_APPARENTDECEL,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("The apparent deceleration of the vehicle as used by the standard model [m/s^2]"),
                                          "4.50");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_EMERGENCYDECEL,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("The maximal physically possible deceleration for the vehicle [m/s^2]"),
                                          "4.50");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_SIGMA,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::RANGE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Car-following model parameter"),
                                          "0.50");
    attrProperty.setRange(0, 1);
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_TAU,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Car-following model parameter"),
                                          "1.00");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_TMP1,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("SKRAUSSX parameter 1"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_TMP2,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("SKRAUSSX parameter 2"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_TMP3,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("SKRAUSSX parameter 3"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_TMP4,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("SKRAUSSX parameter 4"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_TMP5,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("SKRAUSSX parameter 5"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_EIDM_T_LOOK_AHEAD,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("EIDM Look ahead / preview parameter [s]"),
                                          "4.00");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_EIDM_T_REACTION,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("EIDM AP Reaction Time parameter [s]"),
                                          "0.50");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_EIDM_T_PERSISTENCE_DRIVE,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("EIDM Wiener Process parameter for the Driving Error [s]"),
                                          "3.00");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_EIDM_T_PERSISTENCE_ESTIMATE,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("EIDM Wiener Process parameter for the Estimation Error [s]"),
                                          "10.00");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_EIDM_C_COOLNESS,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::RANGE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("EIDM Coolness parameter of the Enhanced IDM [-]"),
                                          "0.99");
    attrProperty.setRange(0, 1);
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_EIDM_SIG_LEADER,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("EIDM leader speed estimation error parameter [-]"),
                                          "0.02");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_EIDM_SIG_GAP,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("EIDM gap estimation error parameter [-]"),
                                          "0.10");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_EIDM_SIG_ERROR,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("EIDM driving error parameter [-]"),
                                          "0.04");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_EIDM_JERK_MAX,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("EIDM maximal jerk parameter [m/s^3]"),
                                          "3.00");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_EIDM_EPSILON_ACC,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("EIDM maximal negative acceleration between two Action Points (threshold) [m/s^2]"),
                                          "1.00");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_EIDM_T_ACC_MAX,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("EIDM Time parameter until vehicle reaches amax after startup/driveoff [s]"),
                                          "1.20");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_EIDM_M_FLATNESS,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("EIDM Flatness parameter of startup/driveoff curve [-]"),
                                          "2.00");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_EIDM_M_BEGIN,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("EIDM Shift parameter of startup/driveoff curve [-]"),
                                          "0.70");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_EIDM_USEVEHDYNAMICS,
                                          GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("EIDM parameter if model shall include vehicle dynamics into the acceleration calculation [0/1]"),
                                          "0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_EIDM_MAX_VEH_PREVIEW,
                                          GNEAttributeProperties::INT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("EIDM parameter how many vehicles are taken into the preview calculation of the driver (at least always 1!) [-]"),
                                          "0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_PWAGNER2009_TAULAST,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Peter Wagner 2009 parameter"),
                                          "0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_PWAGNER2009_APPROB,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Peter Wagner 2009 parameter"),
                                          "0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_IDMM_ADAPT_FACTOR,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("IDMM parameter"),
                                          "0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_IDMM_ADAPT_TIME,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("IDMM parameter"),
                                          "0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_W99_CC1,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("W99 parameter"),
                                          "1.3");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_W99_CC2,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("W99 parameter"),
                                          "8.0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_W99_CC3,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("W99 parameter"),
                                          "-12.0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_W99_CC4,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("W99 parameter"),
                                          "-0.25");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_W99_CC5,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("W99 parameter"),
                                          "0.35");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_W99_CC6,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("W99 parameter"),
                                          "6.0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_W99_CC7,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("W99 parameter"),
                                          "0.25");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_W99_CC8,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("W99 parameter"),
                                          "2.0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_W99_CC9,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("W99 parameter"),
                                          "1.5");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_WIEDEMANN_SECURITY,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Wiedemann parameter"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_WIEDEMANN_ESTIMATION,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Wiedemann parameter"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_COLLISION_MINGAP_FACTOR,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("MinGap factor parameter"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_K,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("K parameter"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_KERNER_PHI,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Kerner Phi parameter"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_IDM_DELTA,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("IDM Delta parameter"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_CF_IDM_STEPPING,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("IDM Stepping parameter"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_TRAIN_TYPE,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Train Types"),
                                          "NGT400");
    attrProperty.setDiscreteValues(SUMOXMLDefinitions::TrainTypes.getStrings());
    myTagProperties[currentTag].addAttribute(attrProperty);
}


void
GNEAttributeCarrier::fillJunctionModelAttributes(SumoXMLTag currentTag) {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    attrProperty = GNEAttributeProperties(SUMO_ATTR_JM_CROSSING_GAP,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Minimum distance to pedestrians that are walking towards the conflict point with the ego vehicle."),
                                          "10");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_JM_IGNORE_KEEPCLEAR_TIME,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("The accumulated waiting time after which a vehicle will drive onto an intersection even though this might cause jamming."),
                                          "-1");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_JM_DRIVE_AFTER_YELLOW_TIME,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("This value causes vehicles to violate a yellow light if the duration of the yellow phase is lower than the given threshold."),
                                          "-1");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_JM_DRIVE_AFTER_RED_TIME,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("This value causes vehicles to violate a red light if the duration of the red phase is lower than the given threshold."),
                                          "-1");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_JM_DRIVE_RED_SPEED,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("This value causes vehicles affected by jmDriveAfterRedTime to slow down when violating a red light."),
                                          "0.0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_JM_IGNORE_FOE_PROB,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("This value causes vehicles to ignore foe vehicles that have right-of-way with the given probability."),
                                          "0.0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_JM_IGNORE_FOE_SPEED,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("This value is used in conjunction with jmIgnoreFoeProb.") + std::string("\n") +
                                          TL("Only vehicles with a speed below or equal to the given value may be ignored."),
                                          "0.0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_JM_SIGMA_MINOR,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("This value configures driving imperfection (dawdling) while passing a minor link."),
                                          "0.0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_JM_TIMEGAP_MINOR,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("This value defines the minimum time gap when passing ahead of a prioritized vehicle. "),
                                          "1");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_IMPATIENCE,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Willingess of drivers to impede vehicles with higher priority"),
                                          "0.0");
    myTagProperties[currentTag].addAttribute(attrProperty);
}


void
GNEAttributeCarrier::fillLaneChangingModelAttributes(SumoXMLTag currentTag) {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_STRATEGIC_PARAM,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("The eagerness for performing strategic lane changing. Higher values result in earlier lane-changing."),
                                          "1.0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_COOPERATIVE_PARAM,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("The willingness for performing cooperative lane changing. Lower values result in reduced cooperation."),
                                          "1.0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_SPEEDGAIN_PARAM,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("The eagerness for performing lane changing to gain speed. Higher values result in more lane-changing."),
                                          "1.0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_KEEPRIGHT_PARAM,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("The eagerness for following the obligation to keep right. Higher values result in earlier lane-changing."),
                                          "1.0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_SUBLANE_PARAM,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("The eagerness for using the configured lateral alignment within the lane.") + std::string("\n") +
                                          TL("Higher values result in increased willingness to sacrifice speed for alignment."),
                                          "1.0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_OPPOSITE_PARAM,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("The eagerness for overtaking through the opposite-direction lane. Higher values result in more lane-changing."),
                                          "1.0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_PUSHY,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Willingness to encroach laterally on other drivers."),
                                          "0.00");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_PUSHYGAP,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Minimum lateral gap when encroaching laterally on other drives (alternative way to define lcPushy)"),
                                          "0.00");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_ASSERTIVE,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Willingness to accept lower front and rear gaps on the target lane."),
                                          "1.0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_IMPATIENCE,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Dynamic factor for modifying lcAssertive and lcPushy."),
                                          "0.00");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_TIME_TO_IMPATIENCE,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Time to reach maximum impatience (of 1). Impatience grows whenever a lane-change manoeuvre is blocked."),
                                          "infinity");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_ACCEL_LAT,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Maximum lateral acceleration per second."),
                                          "1.0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_LOOKAHEADLEFT,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Factor for configuring the strategic lookahead distance when a change to the left is necessary (relative to right lookahead)."),
                                          "2.0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_SPEEDGAINRIGHT,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Factor for configuring the threshold asymmetry when changing to the left or to the right for speed gain."),
                                          "0.1");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_MAXSPEEDLATSTANDING,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Upper bound on lateral speed when standing."),
                                          "0.00");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_MAXSPEEDLATFACTOR,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Upper bound on lateral speed while moving computed as lcMaxSpeedLatStanding + lcMaxSpeedLatFactor * getSpeed()"),
                                          "1.00");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_TURN_ALIGNMENT_DISTANCE,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Distance to an upcoming turn on the vehicles route, below which the alignment") + std::string("\n") +
                                          TL("should be dynamically adapted to match the turn direction."),
                                          "0.00");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_OVERTAKE_RIGHT,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("The probability for violating rules gainst overtaking on the right."),
                                          "0.00");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_KEEPRIGHT_ACCEPTANCE_TIME,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Time threshold for the willingness to change right."),
                                          "-1");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LCA_OVERTAKE_DELTASPEED_FACTOR,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::RANGE | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::EXTENDED,
                                          TL("Speed difference factor for the eagerness of overtaking a neighbor vehicle before changing lanes (threshold = factor*speedlimit)."),
                                          "0.00");
    attrProperty.setRange(-1, 1);
    myTagProperties[currentTag].addAttribute(attrProperty);

}


void
GNEAttributeCarrier::fillCommonPersonAttributes(SumoXMLTag currentTag) {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                          TL("The name of the person"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::VTYPE,
                                          TL("The id of the person type to use for this person"),
                                          DEFAULT_VTYPE_ID);
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_COLOR,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::COLOR | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("This person's color"),
                                          "yellow");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPARTPOS,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE |  GNEAttributeProperties::UPDATEGEOMETRY,
                                          TL("The position at which the person shall enter the net"),
                                          "base");
    myTagProperties[currentTag].addAttribute(attrProperty);
}


void
GNEAttributeCarrier::fillCommonContainerAttributes(SumoXMLTag currentTag) {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                          TL("The name of the container"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_TYPE,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::VTYPE,
                                          TL("The id of the container type to use for this container"),
                                          DEFAULT_CONTAINERTYPE_ID);
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_COLOR,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::COLOR | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("This container's color"),
                                          "yellow");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPARTPOS,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE |  GNEAttributeProperties::UPDATEGEOMETRY,
                                          TL("The position at which the container shall enter the net"),
                                          "base");
    myTagProperties[currentTag].addAttribute(attrProperty);
}


void
GNEAttributeCarrier::fillCommonStopAttributes(SumoXMLTag currentTag, const bool waypoint) {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    attrProperty = GNEAttributeProperties(SUMO_ATTR_DURATION,
                                          GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::ACTIVATABLE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Minimum duration for stopping"),
                                          "60");
    attrProperty.setDefaultActivated(true);
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_UNTIL,
                                          GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::ACTIVATABLE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("The time step at which the route continues"),
                                          "0.00");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_EXTENSION,
                                          GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::ACTIVATABLE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("If set to a non-negative time value, then the stop duration can be extended at most by the extension value in seconds"),
                                          "0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    if (!waypoint) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_TRIGGERED,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Whether a person or container or both may end the stop"),
                                              "false");
        attrProperty.setDiscreteValues({"false", "person", "container", "join"});
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_EXPECTED,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("List of elements that must board the vehicle before it may continue"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_JOIN,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Joins this train to another upon reaching the stop"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }

    attrProperty = GNEAttributeProperties(SUMO_ATTR_PERMITTED,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("List of elements that can board the vehicle before it may continue"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_PARKING,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Whether the vehicle stops on the road or beside"),
                                          "false");
    attrProperty.setDiscreteValues({"true", "false", "opportunistic"});
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_ACTTYPE,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Activity displayed for stopped person in GUI and output files"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_TRIP_ID,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Parameter to be applied to the vehicle to track the trip id within a cyclical public transport route"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LINE,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("New line attribute to be set on the vehicle when reaching this stop (for cyclical public transport route)"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    if (waypoint) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_SPEED,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Speed to be kept while driving between startPos and endPos"),
                                              "0.00");
        myTagProperties[currentTag].addAttribute(attrProperty);
    } else {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ONDEMAND,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Whether the stop may be skipped if no passengers wants to embark or disembark"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }

    attrProperty = GNEAttributeProperties(SUMO_ATTR_JUMP,
                                          GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("transfer time if there shall be a jump from this stop to the next route edge"),
                                          "-1");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_SPLIT,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Splits the train upon reaching the stop"));
    myTagProperties[currentTag].addAttribute(attrProperty);
}


void
GNEAttributeCarrier::fillPlanParentAttributes(SumoXMLTag currentTag) {
    // get rag property
    auto& tagProperty = myTagProperties[currentTag];
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    // basic parents
    if (tagProperty.planConsecutiveEdges()) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_EDGES,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("list of consecutive edges"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ARRIVALPOS,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Arrival position on the last edge"),
                                              "-1");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planRoute()) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ROUTE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("Route ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ARRIVALPOS,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Arrival position on the destination edge"),
                                              "-1");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planEdge()) {

        attrProperty = GNEAttributeProperties(SUMO_ATTR_EDGE,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("Edge ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_ENDPOS,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The end position on the lane (the higher position on the lane) in meters, must be larger than startPos by more than 0.1m"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planBusStop()) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_BUS_STOP,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("Bus stop ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planTrainStop()) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_TRAIN_STOP,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("Train stop ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planContainerStop()) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_CONTAINER_STOP,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("Container stop ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planChargingStation()) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_CHARGING_STATION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("Charging station ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planParkingArea()) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_PARKING_AREA,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("Parking area ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    // from parents
    if (tagProperty.planFromEdge()) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_FROM,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("Edge start ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planFromTAZ()) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_FROM_TAZ,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("TAZ start ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planFromJunction()) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_FROM_JUNCTION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("Junction start ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planFromBusStop()) {
        attrProperty = GNEAttributeProperties(GNE_ATTR_FROM_BUSSTOP,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("BuStop start ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planFromTrainStop()) {
        attrProperty = GNEAttributeProperties(GNE_ATTR_FROM_TRAINSTOP,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("TrainStop start ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planFromContainerStop()) {
        attrProperty = GNEAttributeProperties(GNE_ATTR_FROM_CONTAINERSTOP,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("ContainerStop start ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planFromChargingStation()) {
        attrProperty = GNEAttributeProperties(GNE_ATTR_FROM_CHARGINGSTATION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("ChargingStation start ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planFromParkingArea()) {
        attrProperty = GNEAttributeProperties(GNE_ATTR_FROM_CHARGINGSTATION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("ParkingArea start ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    // to parents
    if (tagProperty.planToEdge()) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_TO,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("Edge end ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
        // departPos only for tranships
        if (tagProperty.isPlanTranship()) {
            // depart pos
            attrProperty = GNEAttributeProperties(SUMO_ATTR_DEPARTPOS,
                                                  GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                                  TL("The position at which the tranship shall enter the net"),
                                                  "0");
            myTagProperties[currentTag].addAttribute(attrProperty);
        }
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ARRIVALPOS,
                                              GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("arrival position on the destination edge"),
                                              "-1");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planToTAZ()) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_TO_TAZ,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("TAZ end ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planToJunction()) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_TO_JUNCTION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("Junction end ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planToBusStop()) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_BUS_STOP,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("BuStop end ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planToTrainStop()) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_TRAIN_STOP,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("TrainStop end ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planToContainerStop()) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_CONTAINER_STOP,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("ContainerStop end ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planToChargingStation()) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_CHARGING_STATION,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("ChargingStation end ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    if (tagProperty.planToParkingArea()) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_PARKING_AREA,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                              TL("ParkingArea end ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
}


void
GNEAttributeCarrier::fillPersonTripCommonAttributes(GNETagProperties& tagProperties) {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    attrProperty = GNEAttributeProperties(SUMO_ATTR_VTYPES,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("List of possible vehicle types to take"));
    tagProperties.addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_MODES,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("List of possible traffic modes. Walking is always possible regardless of this value"));
    tagProperties.addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LINES,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("list of vehicle alternatives to take for the person trip"));
    tagProperties.addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_WALKFACTOR,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Walk factor"),
                                          "0.00");
    tagProperties.addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_GROUP,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("id of the travel group. Persons with the same group may share a taxi ride"));
    tagProperties.addAttribute(attrProperty);
}


void
GNEAttributeCarrier::fillWalkCommonAttributes(GNETagProperties& tagProperties) {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    attrProperty = GNEAttributeProperties(SUMO_ATTR_SPEED,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("speed of the person for this tranship in m/s (not together with duration)"),
                                          "1.39");
    tagProperties.addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_DURATION,
                                          GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("duration of the plan in second (not together with speed)"),
                                          "0");
    tagProperties.addAttribute(attrProperty);
}


void
GNEAttributeCarrier::fillRideCommonAttributes(GNETagProperties& tagProperties) {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LINES,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("list of vehicle alternatives to take for the ride"));
    tagProperties.addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_GROUP,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("id of the travel group. Persons with the same group may share a taxi ride"));
    tagProperties.addAttribute(attrProperty);
}


void
GNEAttributeCarrier::fillTransportCommonAttributes(GNETagProperties& tagProperties) {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    attrProperty = GNEAttributeProperties(SUMO_ATTR_LINES,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("list of vehicle alternatives to take for the transport"));
    tagProperties.addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_GROUP,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("id of the travel group. Persons with the same group may share a taxi ride"));
    tagProperties.addAttribute(attrProperty);
}


void
GNEAttributeCarrier::fillTranshipCommonAttributes(GNETagProperties& tagProperties) {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    attrProperty = GNEAttributeProperties(SUMO_ATTR_SPEED,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("speed of the person for this tranship in m/s (not together with duration)"),
                                          "1.39");
    tagProperties.addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_DURATION,
                                          GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("duration of the plan in second (not together with speed)"),
                                          "0");
    tagProperties.addAttribute(attrProperty);
}


void
GNEAttributeCarrier::fillPlanStopCommonAttributes(GNETagProperties& tagProperties) {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    attrProperty = GNEAttributeProperties(SUMO_ATTR_DURATION,
                                          GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::ACTIVATABLE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Minimum duration for stopping"),
                                          "60");
    attrProperty.setDefaultActivated(true);
    tagProperties.addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_UNTIL,
                                          GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::POSITIVE | GNEAttributeProperties::ACTIVATABLE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("The time step at which the route continues"),
                                          "0.00");
    tagProperties.addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_ACTTYPE,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Activity displayed for stopped person in GUI and output files "));
    tagProperties.addAttribute(attrProperty);

    // friendlyPos attribute only for stops over edges
    if (tagProperties.hasAttribute(SUMO_ATTR_EDGE)) {
        attrProperty = GNEAttributeProperties(SUMO_ATTR_FRIENDLY_POS,
                                              GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("If set, no error will be reported if element is placed behind the lane.") + std::string("\n") +
                                              TL("Instead, it will be placed 0.1 meters from the lanes end or at position 0.1,") + std::string("\n") +
                                              TL("if the position was negative and larger than the lanes length after multiplication with - 1"),
                                              "0");
        tagProperties.addAttribute(attrProperty);
    }
}


void
GNEAttributeCarrier::fillDataElements() {
    // declare empty GNEAttributeProperties
    GNEAttributeProperties attrProperty;

    // fill data set element
    SumoXMLTag currentTag = SUMO_TAG_DATASET;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DATAELEMENT,
                                      GNETagProperties::TagProperty::NOTDRAWABLE | GNETagProperties::TagProperty::NOPARAMETERS | GNETagProperties::TagProperty::NOTSELECTABLE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::DATASET, currentTag, TL("DataSet"));

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("Data set ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);

    }
    // fill data interval element
    currentTag = SUMO_TAG_DATAINTERVAL;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DATAELEMENT,
                                      GNETagProperties::TagProperty::NOTDRAWABLE | GNETagProperties::TagProperty::NOPARAMETERS | GNETagProperties::TagProperty::CHILD | GNETagProperties::TagProperty::NOTSELECTABLE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::DATAINTERVAL, currentTag, TL("DataInterval"),
        {SUMO_TAG_DATASET});

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::AUTOMATICID,
                                              TL("Interval ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_BEGIN,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Data interval begin time"),
                                              "0");
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_END,
                                              GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                              TL("Data interval end time"),
                                              "3600");
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    // fill edge data element
    currentTag = GNE_TAG_EDGEREL_SINGLE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DATAELEMENT | GNETagProperties::TagType::GENERICDATA,
                                      GNETagProperties::TagProperty::NO_PROPERTY,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::EDGEDATA, SUMO_TAG_EDGE, TL("EdgeRelationSingle"));

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY | GNEAttributeProperties::AUTOMATICID,
                                              TL("edge ID"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_EDGEREL;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DATAELEMENT | GNETagProperties::TagType::GENERICDATA,
                                      GNETagProperties::TagProperty::NO_PROPERTY,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::EDGERELDATA, currentTag, TL("EdgeRelation"));

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_FROM,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The ID of the edge the edgeRel starts at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TO,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The ID of the edge the edgeRel ends at"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_TAZREL;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::DATAELEMENT | GNETagProperties::TagType::GENERICDATA,
                                      GNETagProperties::TagProperty::RTREE | GNETagProperties::TagProperty::CHILD,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::TAZRELDATA, currentTag, TL("TAZRelation"),
        {SUMO_TAG_DATAINTERVAL});

        // set values of attributes
        attrProperty = GNEAttributeProperties(SUMO_ATTR_FROM,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of the TAZ the TAZRel starts at"));
        myTagProperties[currentTag].addAttribute(attrProperty);

        attrProperty = GNEAttributeProperties(SUMO_ATTR_TO,
                                              GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE | GNEAttributeProperties::UPDATEGEOMETRY,
                                              TL("The name of the TAZ the TAZRel ends at"));
        myTagProperties[currentTag].addAttribute(attrProperty);
    }
    currentTag = SUMO_TAG_MEANDATA_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::MEANDATA,
                                      GNETagProperties::TagProperty::NOTDRAWABLE | GNETagProperties::TagProperty::NOPARAMETERS | GNETagProperties::TagProperty::NOTSELECTABLE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::MEANDATAEDGE, currentTag, TL("MeanDataEdge"));

        // set values of attributes
        fillCommonMeanDataAttributes(currentTag);

    }
    currentTag = SUMO_TAG_MEANDATA_LANE;
    {
        // set values of tag
        myTagProperties[currentTag] = GNETagProperties(currentTag,
                                      GNETagProperties::TagType::MEANDATA,
                                      GNETagProperties::TagProperty::NOTDRAWABLE | GNETagProperties::TagProperty::NOPARAMETERS | GNETagProperties::TagProperty::NOTSELECTABLE,
                                      GNETagProperties::TagParents::NO_PARENTS,
                                      GNETagProperties::Conflicts::NO_CONFLICTS,
                                      GUIIcon::MEANDATALANE, currentTag, TL("MeanDataLane"));

        // set values of attributes
        fillCommonMeanDataAttributes(currentTag);
    }
}


void
GNEAttributeCarrier::fillCommonMeanDataAttributes(SumoXMLTag currentTag) {
    GNEAttributeProperties attrProperty;

    // fill all meanData attributes
    attrProperty = GNEAttributeProperties(SUMO_ATTR_ID,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::UNIQUE,
                                          TL("The id of this set of measurements"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_FILE,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::FILENAME | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("The path to the output file. The path may be relative"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_PERIOD,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("The aggregation period the values the detector collects shall be summed up"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_BEGIN,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("The time to start writing. If not given, the simulation's begin is used."));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_END,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("The time to end writing. If not given the simulation's end is used."));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_EXCLUDE_EMPTY,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::DISCRETE | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("If set to true, edges/lanes which were not used by a vehicle during this period will not be written"),
                                          "default");
    attrProperty.setDiscreteValues({"1", "0", "default"});
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_WITH_INTERNAL,
                                          GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("If set, junction internal edges/lanes will be written as well"),
                                          "0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_MAX_TRAVELTIME,
                                          GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("The maximum travel time in seconds to write if only very small movements occur"),
                                          "100000");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_MIN_SAMPLES,
                                          GNEAttributeProperties::SUMOTIME | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Consider an edge/lane unused if it has at most this many sampled seconds"),
                                          "0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_HALTING_SPEED_THRESHOLD,
                                          GNEAttributeProperties::FLOAT | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("The maximum speed to consider a vehicle halting;"),
                                          "0.1");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_VTYPES,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("space separated list of vehicle type ids to consider"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_TRACK_VEHICLES,
                                          GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("whether aggregation should be performed over all vehicles that entered the edge/lane in the aggregation interval"),
                                          "0");
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_DETECT_PERSONS,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Whether pedestrians shall be recorded instead of vehicles. Allowed value is walk"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_WRITE_ATTRIBUTES,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("List of attribute names that shall be written"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_EDGES,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::LIST | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Restrict output to the given list of edge ids"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_EDGESFILE,
                                          GNEAttributeProperties::STRING | GNEAttributeProperties::FILENAME | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Restrict output to the given list of edges given in file"));
    myTagProperties[currentTag].addAttribute(attrProperty);

    attrProperty = GNEAttributeProperties(SUMO_ATTR_AGGREGATE,
                                          GNEAttributeProperties::BOOL | GNEAttributeProperties::DEFAULTVALUE,
                                          TL("Whether the traffic statistic of all edges shall be aggregated into a single value"),
                                          "0");
    myTagProperties[currentTag].addAttribute(attrProperty);
}


void
GNEAttributeCarrier::writeAttributeHelp() {
    if (myTagProperties.size() == 0) {
        fillAttributeCarriers();
    }
    // merge "virtual" netedit tags like  '<walk: edge->edge'
    static std::map<SumoXMLTag, GNETagProperties> xmlTagProperties;
    for (const auto& item : myTagProperties) {
        if (xmlTagProperties.count(item.second.getXMLTag()) == 0) {
            xmlTagProperties[item.second.getXMLTag()] = item.second;
        } else {
            std::set<SumoXMLAttr> attrs;
            auto& old = xmlTagProperties[item.second.getXMLTag()];
            for (auto it = old.begin(); it != old.end(); it++) {
                attrs.insert(it->getAttr());
            }
            for (auto it = item.second.begin(); it != item.second.end(); it++) {
                if (attrs.count(it->getAttr()) == 0) {
                    old.addAttribute(*it);
                }
            }
        }
    }
    const std::string opt = "attribute-help-output";
    OutputDevice::createDeviceByOption(opt);
    OutputDevice& dev = OutputDevice::getDeviceByOption(opt);
    dev << "# Netedit attribute help\n";
    for (const auto& item : xmlTagProperties) {
        if (item.second.begin() == item.second.end()) {
            // don't write elements without attributes, they are only used for internal purposes
            continue;
        }
        if (item.second.getParentTags().empty()) {
            dev << "\n## " << toString(item.first) << "\n";
        } else {
            if (item.first == SUMO_TAG_FLOW) {
                dev << "\n## " << toString(item.first) << "\n";
                dev << "also child element of ";
            } else {
                dev << "\n### " << toString(item.first) << "\n";
                dev << "child element of ";
            }
            bool sep = false;
            for (const auto& pTag : item.second.getParentTags()) {
                if (sep) {
                    dev << ", ";
                } else {
                    sep = true;
                }
                dev << "[" << toString(pTag) << "](#" << StringUtils::to_lower_case(toString(pTag)) << ")";
            }
            dev << "\n\n";
        }
        dev << "| Attribute | Type | Description |\n";
        dev << "|-----------|------|-------------|\n";
        for (const auto& attr : item.second) {
            dev << "|" << toString(attr.getAttr()) << "|"
                << attr.getDescription() << "|"
                << StringUtils::replace(attr.getDefinition(), "\n", " ");
            if (attr.getDefaultValue() != "") {
                dev << " *default:* **" << attr.getDefaultValue() << "**";
            }
            dev << "|\n";
        }
    }
}


/****************************************************************************/
