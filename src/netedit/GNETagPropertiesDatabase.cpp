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
/// @file    GNETagPropertiesDatabase.cpp
/// @author  Pablo Alvarez lopez
/// @date    Feb 2025
///
// Database with all information about netedit elements
/****************************************************************************/

#include <netedit/elements/moving/GNEMoveElementLaneDouble.h>
#include <netedit/elements/moving/GNEMoveElementLaneSingle.h>
#include <netedit/GNENet.h>
#include <utils/emissions/PollutantsInterface.h>
#include <utils/options/OptionsCont.h>

#include "GNETagPropertiesDatabase.h"

// ===========================================================================
// method definitions
// ===========================================================================

GNETagPropertiesDatabase::GNETagPropertiesDatabase() {
    // fill all groups of ACs
    fillHierarchy();
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
    // add common attributes to all elements
    for (auto& tagProperties : myTagProperties) {
        fillCommonAttributes(tagProperties.second);
    }
    // update max number of editable attributes
    updateMaxNumberOfAttributesEditorRows();
    // calculate hierarchy dept
    updateMaxHierarchyDepth();
    // check integrity of all Tags (function checkTagIntegrity() throws an exception if there is an inconsistency)
    for (const auto& tagProperties : myTagProperties) {
        tagProperties.second->checkTagIntegrity();
    }
}


GNETagPropertiesDatabase::~GNETagPropertiesDatabase() {
    // delete all tag properties (this also delete all attributeProperties)
    for (auto& tagProperties : myTagProperties) {
        delete tagProperties.second;
    }
}


const GNETagProperties*
GNETagPropertiesDatabase::getTagProperty(SumoXMLTag tag, const bool hardFail) const {
    // check that tag is defined in tagProperties or in tagPropertiesSet
    if (myTagProperties.count(tag) > 0) {
        return myTagProperties.at(tag);
    } else if (mySetTagProperties.count(tag) > 0) {
        return mySetTagProperties.at(tag);
    } else if (hardFail) {
        throw ProcessError(TLF("Property for tag '%' not defined", toString(tag)));
    } else {
        return nullptr;
    }
}


const std::vector<const GNETagProperties*>
GNETagPropertiesDatabase::getTagPropertiesSet(const SumoXMLTag tag, const bool hardFail) const {
    // check that tag is defined in tagProperties or in tagPropertiesSet
    if (mySetTagProperties.count(tag) > 0) {
        return mySetTagProperties.at(tag)->getHierarchicalChildren();
    } else if (hardFail) {
        throw ProcessError(TLF("TagPropertySet for tag '%' not defined", toString(tag)));
    } else {
        return {};
    }
}


const std::vector<const GNETagProperties*>
GNETagPropertiesDatabase::getTagPropertiesByType(const GNETagProperties::Type type) const {
    std::vector<const GNETagProperties*> allowedTags;
    if (type & GNETagProperties::Type::NETWORKELEMENT) {
        // fill networkElements tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isNetworkElement()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::ADDITIONALELEMENT) {
        // fill additional tags (only with pure additionals)
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isAdditionalPureElement()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::SHAPE) {
        // fill shape tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isShapeElement()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::TAZELEMENT) {
        // fill taz tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isTAZElement()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::WIRE) {
        // fill wire tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isWireElement()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::DEMANDELEMENT) {
        // fill demand tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isDemandElement()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::ROUTE) {
        // fill route tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isRoute()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::VEHICLE) {
        // fill vehicle tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isVehicle()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::STOP_VEHICLE) {
        // fill stop (and waypoints) tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isVehicleStop()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::PERSON) {
        // fill person tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isPerson()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::PERSONPLAN) {
        // fill person plan tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isPlanPerson()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::PERSONTRIP) {
        // fill demand tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isPlanPersonTrip()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::WALK) {
        // fill demand tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isPlanWalk()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::RIDE) {
        // fill demand tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isPlanRide()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::STOP_PERSON) {
        // fill demand tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isPlanStopPerson()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::GENERICDATA) {
        // fill generic data tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isGenericData()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::MEANDATA) {
        // fill generic data tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isMeanData()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::CONTAINER) {
        // fill container tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isContainer()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::CONTAINERPLAN) {
        // fill container plan tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isPlanContainer()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::TRANSPORT) {
        // fill demand tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isPlanTransport()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::TRANSHIP) {
        // fill demand tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isPlanTranship()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    if (type & GNETagProperties::Type::STOP_CONTAINER) {
        // fill demand tags
        for (const auto& tagProperty : myTagProperties) {
            if (tagProperty.second->isPlanStopContainer()) {
                allowedTags.push_back(tagProperty.second);
            }
        }
    }
    return allowedTags;
}


int
GNETagPropertiesDatabase::getMaxNumberOfEditableAttributeRows() const {
    return myMaxNumberOfEditableAttributeRows;
}


int
GNETagPropertiesDatabase::getMaxNumberOfGeoAttributeRows() const {
    return myMaxNumberOfGeoAttributeRows;
}


int
GNETagPropertiesDatabase::getMaxNumberOfFlowAttributeRows() const {
    return myMaxNumberOfFlowAttributeRows;
}


int
GNETagPropertiesDatabase::getMaxNumberOfNeteditAttributesRows() const {
    return myMaxNumberOfNeteditAttributeRows;
}


int
GNETagPropertiesDatabase::getHierarchyDepth() const {
    return myHierarchyDepth;
}


void
GNETagPropertiesDatabase::writeAttributeHelp() const {
    const std::string opt = "attribute-help-output";
    OutputDevice::createDeviceByOption(opt);
    OutputDevice& dev = OutputDevice::getDeviceByOption(opt);
    dev << "# Netedit attribute help\n";
    for (const auto& tagProperty : myTagProperties) {
        if (tagProperty.second->getAttributeProperties().begin() == tagProperty.second->getAttributeProperties().end()) {
            // don't write elements without attributes, they are only used for internal purposes
            continue;
        }
        if (tagProperty.second->getXMLTag() != tagProperty.first) {
            dev << "\n## " << toString(tagProperty.second->getXMLTag()) << " (" << toString(tagProperty.first) << ")\n";
        } else if (tagProperty.second->getXMLParentTags().empty()) {
            dev << "\n## " << toString(tagProperty.first) << "\n";
        } else {
            if (tagProperty.first == SUMO_TAG_FLOW) {
                dev << "\n## " << toString(tagProperty.first) << "\n";
                dev << "also child element of ";
            } else {
                dev << "\n### " << toString(tagProperty.first) << "\n";
                dev << "child element of ";
            }
            bool sep = false;
            for (const auto& pTag : tagProperty.second->getXMLParentTags()) {
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
        for (const auto& attr : tagProperty.second->getAttributeProperties()) {
            // ignore netedit attributes (front, selected, etc.)
            if (!attr->isNeteditEditor() && (attr->getAttr() != GNE_ATTR_PARAMETERS)) {
                dev << "|" << toString(attr->getAttr()) << "|"
                    << attr->getDescription() << "|"
                    << StringUtils::replace(attr->getDefinition(), "\n", " ");
                if (attr->hasDefaultValue() && attr->getDefaultStringValue() != "") {
                    dev << " *default:* **" << attr->getDefaultStringValue() << "**";
                }
                dev << "|\n";
            }
        }
    }
}


void
GNETagPropertiesDatabase::fillHierarchy() {
    // root - level 0
    mySetTagProperties[SUMO_TAG_ROOTFILE] = new GNETagProperties(SUMO_TAG_ROOTFILE,
            nullptr,
            GUIIcon::NETEDIT_MINI,
            TL("Root"));
    // supermodes - level 1
    mySetTagProperties[GNE_TAG_SUPERMODE_NETWORK] = new GNETagProperties(GNE_TAG_SUPERMODE_NETWORK,
            mySetTagProperties.at(SUMO_TAG_ROOTFILE),
            GUIIcon::SUPERMODENETWORK,
            TL("Supermode network"),
            FXRGBA(255, 255, 255, 255),
            TL("Supermode network"));
    mySetTagProperties[GNE_TAG_SUPERMODE_DEMAND] = new GNETagProperties(GNE_TAG_SUPERMODE_DEMAND,
            mySetTagProperties.at(SUMO_TAG_ROOTFILE),
            GUIIcon::SUPERMODEDEMAND,
            TL("Supermode demand"),
            FXRGBA(255, 255, 255, 255),
            TL("Supermode demand"));
    mySetTagProperties[GNE_TAG_SUPERMODE_DATA] = new GNETagProperties(GNE_TAG_SUPERMODE_DATA,
            mySetTagProperties.at(SUMO_TAG_ROOTFILE),
            GUIIcon::SUPERMODEDATA,
            TL("Supermode data"),
            FXRGBA(255, 255, 255, 255),
            TL("Supermode data"));
    // net - level 2
    mySetTagProperties[SUMO_TAG_NET] = new GNETagProperties(SUMO_TAG_NET,
            mySetTagProperties.at(GNE_TAG_SUPERMODE_NETWORK),
            GUIIcon::MODECREATEEDGE,
            TL("Network elements"),
            FXRGBA(255, 255, 255, 255),
            TL("Network elements"));
    // additionals - level 2
    mySetTagProperties[SUMO_TAG_VIEWSETTINGS_ADDITIONALS] = new GNETagProperties(SUMO_TAG_VIEWSETTINGS_ADDITIONALS,
            mySetTagProperties.at(GNE_TAG_SUPERMODE_NETWORK),
            GUIIcon::MODEADDITIONAL,
            TL("Additional elements"),
            FXRGBA(255, 255, 255, 255),
            TL("Additional elements"));
    // stoppingPlaces - level 3
    mySetTagProperties[GNE_TAG_STOPPINGPLACES] = new GNETagProperties(GNE_TAG_STOPPINGPLACES,
            mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
            GUIIcon::BUSSTOP,
            TL("Stopping places"),
            FXRGBA(255, 255, 255, 255),
            TL("Stopping places"));
    // detectors - level 3
    mySetTagProperties[GNE_TAG_DETECTORS] = new GNETagProperties(GNE_TAG_DETECTORS,
            mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
            GUIIcon::E1,
            TL("Detectors"),
            FXRGBA(255, 255, 255, 255),
            TL("Detectors"));
    // wires - level 2
    mySetTagProperties[GNE_TAG_WIRES] = new GNETagProperties(GNE_TAG_WIRES,
            mySetTagProperties.at(GNE_TAG_SUPERMODE_NETWORK),
            GUIIcon::MODEWIRE,
            TL("Wire elements"),
            FXRGBA(255, 255, 255, 255),
            TL("Wire elements"));
    // shapes - level 2
    mySetTagProperties[GNE_TAG_SHAPES] = new GNETagProperties(GNE_TAG_SHAPES,
            mySetTagProperties.at(GNE_TAG_SUPERMODE_NETWORK),
            GUIIcon::MODESHAPE,
            TL("Shape elements"),
            FXRGBA(255, 255, 255, 255),
            TL("Shape elements"));
    mySetTagProperties[GNE_TAG_JUPEDSIM] = new GNETagProperties(GNE_TAG_JUPEDSIM,
            mySetTagProperties.at(GNE_TAG_SHAPES),
            GUIIcon::JPS_WALKABLEAREA,
            TL("JuPedSim elements"),
            FXRGBA(255, 255, 255, 255),
            TL("JuPedSim elements"));
    // TAZs - level 2
    mySetTagProperties[GNE_TAG_TAZS] = new GNETagProperties(GNE_TAG_TAZS,
            mySetTagProperties.at(GNE_TAG_SUPERMODE_NETWORK),
            GUIIcon::MODEADDITIONAL,
            TL("TAZ elements"),
            FXRGBA(255, 255, 255, 255),
            TL("TAZ elements"));
    // vehicles - level 2
    mySetTagProperties[SUMO_TAG_VIEWSETTINGS_VEHICLES] = new GNETagProperties(SUMO_TAG_VIEWSETTINGS_VEHICLES,
            mySetTagProperties.at(GNE_TAG_SUPERMODE_DEMAND),
            GUIIcon::VEHICLE,
            TL("Vehicles"),
            FXRGBA(255, 255, 255, 255),
            TL("Vehicles"));
    // flows - level 2
    mySetTagProperties[GNE_TAG_FLOWS] = new GNETagProperties(GNE_TAG_FLOWS,
            mySetTagProperties.at(GNE_TAG_SUPERMODE_DEMAND),
            GUIIcon::FLOW,
            TL("Flows"),
            FXRGBA(255, 255, 255, 255),
            TL("Vehicle flows"));
    // stops - level 2
    mySetTagProperties[GNE_TAG_STOPS] = new GNETagProperties(GNE_TAG_STOPS,
            mySetTagProperties.at(GNE_TAG_SUPERMODE_DEMAND),
            GUIIcon::MODESTOP,
            TL("Vehicle stops"),
            FXRGBA(255, 255, 255, 255),
            TL("Vehicle stops"));
    // personPlans - level 2
    mySetTagProperties[GNE_TAG_PERSONPLANS] = new GNETagProperties(GNE_TAG_PERSONPLANS,
            mySetTagProperties.at(GNE_TAG_SUPERMODE_DEMAND),
            GUIIcon::MODEPERSONPLAN,
            TL("Person plans"),
            FXRGBA(255, 255, 255, 255),
            TL("Person plans"));
    // personTrips - level 3
    mySetTagProperties[GNE_TAG_PERSONTRIPS] = new GNETagProperties(GNE_TAG_PERSONTRIPS,
            mySetTagProperties.at(GNE_TAG_PERSONPLANS),
            GUIIcon::PERSONTRIP_BUSSTOP,
            TL("Person trips"),
            FXRGBA(255, 255, 255, 255),
            TL("Person trips"));
    // rides - level 3
    mySetTagProperties[GNE_TAG_RIDES] = new GNETagProperties(GNE_TAG_RIDES,
            mySetTagProperties.at(GNE_TAG_PERSONPLANS),
            GUIIcon::RIDE_BUSSTOP,
            TL("Person rides"),
            FXRGBA(255, 255, 255, 255),
            TL("Person rides"));
    // walks - level 3
    mySetTagProperties[GNE_TAG_WALKS] = new GNETagProperties(GNE_TAG_WALKS,
            mySetTagProperties.at(GNE_TAG_PERSONPLANS),
            GUIIcon::WALK_BUSSTOP,
            TL("Person walks"),
            FXRGBA(255, 255, 255, 255),
            TL("Person walks"));
    // personStops - level 3
    mySetTagProperties[GNE_TAG_PERSONSTOPS] = new GNETagProperties(GNE_TAG_PERSONSTOPS,
            mySetTagProperties.at(GNE_TAG_PERSONPLANS),
            GUIIcon::STOP,
            TL("Person stop"),
            FXRGBA(255, 255, 255, 255),
            TL("Person stop"));
    // containerPlans - level 2
    mySetTagProperties[GNE_TAG_CONTAINERPLANS] = new GNETagProperties(GNE_TAG_CONTAINERPLANS,
            mySetTagProperties.at(GNE_TAG_SUPERMODE_DEMAND),
            GUIIcon::MODECONTAINERPLAN,
            TL("Container plans"),
            FXRGBA(255, 255, 255, 255),
            TL("Container plans"));
    // transports - level 3
    mySetTagProperties[GNE_TAG_TRANSPORTS] = new GNETagProperties(GNE_TAG_TRANSPORTS,
            mySetTagProperties.at(GNE_TAG_CONTAINERPLANS),
            GUIIcon::TRANSPORT_BUSSTOP,
            TL("Container transports"),
            FXRGBA(255, 255, 255, 255),
            TL("Container transports"));
    // tranships - level 3
    mySetTagProperties[GNE_TAG_TRANSHIPS] = new GNETagProperties(GNE_TAG_TRANSHIPS,
            mySetTagProperties.at(GNE_TAG_CONTAINERPLANS),
            GUIIcon::TRANSHIP_BUSSTOP,
            TL("Container tranships"),
            FXRGBA(255, 255, 255, 255),
            TL("Container tranships"));
    // containerStops - level 3
    mySetTagProperties[GNE_TAG_CONTAINERSTOPS] = new GNETagProperties(GNE_TAG_CONTAINERSTOPS,
            mySetTagProperties.at(GNE_TAG_CONTAINERPLANS),
            GUIIcon::STOP,
            TL("Container stops"),
            FXRGBA(255, 255, 255, 255),
            TL("Container stops"));
    // datas - level 2
    mySetTagProperties[GNE_TAG_DATAS] = new GNETagProperties(GNE_TAG_DATAS,
            mySetTagProperties.at(GNE_TAG_SUPERMODE_DATA),
            GUIIcon::EDGEDATA,
            TL("Datas"),
            FXRGBA(255, 255, 255, 255),
            TL("Datas"));
}


void
GNETagPropertiesDatabase::fillNetworkElements() {
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
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_NET),
                GNETagProperties::Type::NETWORKELEMENT,
                GNETagProperties::Property::RTREE,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::NETWORK,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::JUNCTION, GUIGlObjectType::GLO_JUNCTION, currentTag, TL("Junction"));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_POSITION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::UPDATEGEOMETRY, // virtual attribute from the combination of the actually attributes SUMO_ATTR_X, SUMO_ATTR_Y
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The x-y-z position of the node on the plane in meters"));

        auto type = new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TYPE,
                                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                               TL("An optional type for the node"));
        type->setDiscreteValues(nodeTypes);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SHAPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("A custom shape for that node"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_RADIUS,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Optional turning radius (for all corners) for that node in meters"),
                                   "4");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_KEEP_CLEAR,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Whether the junction-blocking-heuristic should be activated at this node"),
                                   GNEAttributeCarrier::TRUE_STR);

        auto rightOfWay = new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_RIGHT_OF_WAY,
                GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE,
                GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                TL("How to compute right of way rules at this node"),
                SUMOXMLDefinitions::RightOfWayValues.getString(RightOfWay::DEFAULT));
        rightOfWay->setDiscreteValues(SUMOXMLDefinitions::RightOfWayValues.getStrings());

        auto fringe = new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_FRINGE,
                GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE,
                GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                TL("Whether this junction is at the fringe of the network"),
                SUMOXMLDefinitions::FringeTypeValues.getString(FringeType::DEFAULT));
        fringe->setDiscreteValues(SUMOXMLDefinitions::FringeTypeValues.getStrings());

        auto roundabout = new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ROUNDABOUT,
                GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE,
                GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                TL("Whether this junction may be considered when guessing roundabouts"),
                SUMOXMLDefinitions::FringeTypeValues.getString(FringeType::DEFAULT));
        roundabout->setDiscreteValues(SUMOXMLDefinitions::RoundaboutTypeValues.getStrings());

        fillNameAttribute(myTagProperties[currentTag]);

        auto tlType = new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TLTYPE,
                GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE,
                GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                TL("An optional type for the traffic light algorithm"));
        tlType->setDiscreteValues(TLTypes);

        auto tlLayout = new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TLLAYOUT,
                GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE,
                GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                TL("An optional layout for the traffic light plan"),
                toString(TrafficLightLayout::DEFAULT));
        tlLayout->setDiscreteValues({toString(TrafficLightLayout::DEFAULT),
                                     toString(TrafficLightLayout::OPPOSITES),
                                     toString(TrafficLightLayout::INCOMING),
                                     toString(TrafficLightLayout::ALTERNATE_ONEWAY)});

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TLID,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("An optional id for the traffic light program"));

        new GNEAttributeProperties(myTagProperties[currentTag], GNE_ATTR_IS_ROUNDABOUT,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Whether this junction is part of a roundabout"),
                                   GNEAttributeCarrier::FALSE_STR);
    }
    currentTag = SUMO_TAG_TYPE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_NET),
                GNETagProperties::Type::NETWORKELEMENT,
                GNETagProperties::Property::NOTDRAWABLE | GNETagProperties::Property::NOTSELECTABLE,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::NETWORK,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::EDGETYPE, GUIGlObjectType::GLO_EDGETYPE, currentTag, TL("EdgeType"));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], false);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_NUMLANES,
                                   GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The number of lanes of the edge"),
                                   toString(neteditOptions.getInt("default.lanenumber")));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SPEED,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The maximum speed allowed on the edge in m/s"),
                                   toString(neteditOptions.getFloat("default.speed")));

        fillAllowDisallowAttributes(myTagProperties[currentTag]);

        auto spreadType = new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SPREADTYPE,
                GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                TL("The spreadType defines how to compute the lane geometry from the edge geometry (used for visualization)"),
                SUMOXMLDefinitions::LaneSpreadFunctions.getString(LaneSpreadFunction::RIGHT));
        spreadType->setDiscreteValues(SUMOXMLDefinitions::LaneSpreadFunctions.getStrings());

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PRIORITY,
                                   GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The priority of the edge"),
                                   toString(neteditOptions.getInt("default.priority")));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_WIDTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Lane width for all lanes of this edge in meters (used for visualization)"),
                                   "", toString(SUMO_const_laneWidth));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SIDEWALKWIDTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The width of the sidewalk that should be added as an additional lane"),
                                   "", "2");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_BIKELANEWIDTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The width of the bike lane that should be added as an additional lane"),
                                   "", "1");
    }
    currentTag = SUMO_TAG_LANETYPE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_NET),
                GNETagProperties::Type::NETWORKELEMENT,
                GNETagProperties::Property::NOTDRAWABLE | GNETagProperties::Property::NOTSELECTABLE,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::NETWORK,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::LANETYPE, GUIGlObjectType::GLO_LANETYPE, currentTag, TL("LaneType"));
        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SPEED,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The maximum speed allowed on the lane in m/s"),
                                   toString(neteditOptions.getFloat("default.speed")));

        fillAllowDisallowAttributes(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_WIDTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Lane width for all lanes of this type in meters (used for visualization)"),
                                   "3.2");
    }
    currentTag = SUMO_TAG_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_NET),
                GNETagProperties::Type::NETWORKELEMENT,
                GNETagProperties::Property::RTREE,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::NETWORK,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::EDGE, GUIGlObjectType::GLO_EDGE, currentTag, TL("Edge"));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_FROM,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The name of a node within the nodes-file the edge shall start at"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TO,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The name of a node within the nodes-file the edge shall end at"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SPEED,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The maximum speed allowed on the edge in m/s"),
                                   toString(neteditOptions.getFloat("default.speed")));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PRIORITY,
                                   GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The priority of the edge"),
                                   toString(neteditOptions.getInt("default.priority")));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_NUMLANES,
                                   GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The number of lanes of the edge"),
                                   toString(neteditOptions.getInt("default.lanenumber")));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TYPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The name of a type within the SUMO edge type file"));

        fillAllowDisallowAttributes(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SHAPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("If the shape is given it should start and end with the positions of the from-node and to-node"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_LENGTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The length of the edge in meter"));

        auto spreadType = new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SPREADTYPE,
                GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                TL("The spreadType defines how to compute the lane geometry from the edge geometry (used for visualization)"),
                SUMOXMLDefinitions::LaneSpreadFunctions.getString(LaneSpreadFunction::RIGHT));
        spreadType->setDiscreteValues(SUMOXMLDefinitions::LaneSpreadFunctions.getStrings());

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_NAME,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("street name (does not need to be unique, used for visualization)"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_WIDTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Lane width for all lanes of this edge in meters (used for visualization)"),
                                   "3.2");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ENDOFFSET,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Move the stop line back from the intersection by the given amount"),
                                   "0");

        new GNEAttributeProperties(myTagProperties[currentTag], GNE_ATTR_SHAPE_START,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY, // virtual attribute used to define an endPoint
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Custom position in which shape start (by default position of junction from)"));

        new GNEAttributeProperties(myTagProperties[currentTag], GNE_ATTR_SHAPE_END,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY, // virtual attribute from to define an endPoint
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Custom position in which shape end (by default position of junction from)"));

        new GNEAttributeProperties(myTagProperties[currentTag], GNE_ATTR_BIDIR,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UNIQUE, // virtual attribute to check of this edge is part of a bidirectional railway (cannot be edited)
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Show if edge is bidirectional"),
                                   GNEAttributeCarrier::FALSE_STR);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_DISTANCE,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Distance"),
                                   "0");

        new GNEAttributeProperties(myTagProperties[currentTag], GNE_ATTR_STOPOFFSET,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The stop offset as positive value in meters"),
                                   "0");

        new GNEAttributeProperties(myTagProperties[currentTag], GNE_ATTR_STOPOEXCEPTION,
                                   GNEAttributeProperties::Property::VCLASS | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Specifies, for which vehicle classes the stopOffset does NOT apply."));

        new GNEAttributeProperties(myTagProperties[currentTag], GNE_ATTR_IS_ROUNDABOUT,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UNIQUE, // cannot be edited
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Whether this edge is part of a roundabout"),
                                   GNEAttributeCarrier::FALSE_STR);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ROUTINGTYPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The name of a routingType"));

    }
    currentTag = SUMO_TAG_LANE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_NET),
                GNETagProperties::Type::NETWORKELEMENT,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::NETWORK,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::LANE, GUIGlObjectType::GLO_LANE, currentTag, TL("Lane"));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_INDEX,
                                   GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The enumeration index of the lane (0 is the rightmost lane, <NUMBER_LANES>-1 is the leftmost one)"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SPEED,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Speed in meters per second"),
                                   toString(neteditOptions.getFloat("default.speed")));

        fillAllowDisallowAttributes(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_WIDTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Width in meters (used for visualization)"),
                                   "", "-1");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ENDOFFSET,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Move the stop line back from the intersection by the given amount"),
                                   "0");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ACCELERATION,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Enable or disable lane as acceleration lane"),
                                   GNEAttributeCarrier::FALSE_STR);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CUSTOMSHAPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("If the shape is given it overrides the computation based on edge shape"));

        new GNEAttributeProperties(myTagProperties[currentTag], GNE_ATTR_OPPOSITE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("If given, this defines the opposite direction lane"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CHANGE_LEFT,
                                   GNEAttributeProperties::Property::VCLASS | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Permit changing left only for to the given vehicle classes"),
                                   "all");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CHANGE_RIGHT,
                                   GNEAttributeProperties::Property::VCLASS | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Permit changing right only for to the given vehicle classes"),
                                   "all");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TYPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Lane type description (optional)"));

        new GNEAttributeProperties(myTagProperties[currentTag], GNE_ATTR_STOPOFFSET,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The stop offset as positive value in meters"),
                                   "0");

        new GNEAttributeProperties(myTagProperties[currentTag], GNE_ATTR_STOPOEXCEPTION,
                                   GNEAttributeProperties::Property::VCLASS | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Specifies, for which vehicle classes the stopOffset does NOT apply."));
    }
    currentTag = SUMO_TAG_CROSSING;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_NET),
                GNETagProperties::Type::NETWORKELEMENT,
                GNETagProperties::Property::NOPARAMETERS,
                GNETagProperties::Over::JUNCTION,
                FileBucket::Type::NETWORK,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::CROSSING, GUIGlObjectType::GLO_CROSSING, currentTag, TL("Crossing"));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_EDGES,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The (road) edges which are crossed"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PRIORITY,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Whether the pedestrians have priority over the vehicles (automatically set to true at tls-controlled intersections)"),
                                   GNEAttributeCarrier::FALSE_STR);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_WIDTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The width of the crossings"),
                                   toString(OptionsCont::getOptions().getFloat("default.crossing-width")));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TLLINKINDEX,
                                   GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("sets the tls-index for this crossing (-1 means automatic assignment)"),
                                   "-1");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TLLINKINDEX2,
                                   GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("sets the opposite-direction tls-index for this crossing (-1 means not assigned)"),
                                   "-1");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CUSTOMSHAPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Overrides default shape of pedestrian crossing"));
    }
    currentTag = SUMO_TAG_WALKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_NET),
                GNETagProperties::Type::NETWORKELEMENT,
                GNETagProperties::Property::NOPARAMETERS,
                GNETagProperties::Over::JUNCTION,
                FileBucket::Type::NETWORK,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::WALKINGAREA, GUIGlObjectType::GLO_WALKINGAREA, currentTag, TL("WalkingArea"));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_WIDTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The width of the WalkingArea"),
                                   toString(OptionsCont::getOptions().getFloat("default.sidewalk-width")));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_LENGTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The length of the WalkingArea in meter"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SHAPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Overrides default shape of pedestrian sidewalk"));

    }
    currentTag = SUMO_TAG_CONNECTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_NET),
                GNETagProperties::Type::NETWORKELEMENT,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::JUNCTION,
                FileBucket::Type::NETWORK,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::CONNECTION, GUIGlObjectType::GLO_CONNECTION, currentTag, TL("Connection"));
        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_FROM,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The ID of the edge the vehicles leave"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TO,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The ID of the edge the vehicles may reach when leaving 'from'"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_FROM_LANE,
                                   GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("the lane index of the incoming lane (numbers starting with 0)"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TO_LANE,
                                   GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("the lane index of the outgoing lane (numbers starting with 0)"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PASS,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("if set, vehicles which pass this (lane-2-lane) connection) will not wait"),
                                   GNEAttributeCarrier::FALSE_STR);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_KEEP_CLEAR,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("if set to false, vehicles which pass this (lane-2-lane) connection) will not worry about blocking the intersection"),
                                   GNEAttributeCarrier::FALSE_STR);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CONTPOS,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("If set to a more than 0 value, an internal junction will be built at this position (in m)/n from the start of the internal lane for this connection"),
                                   "", toString(NBEdge::UNSPECIFIED_CONTPOS));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_UNCONTROLLED,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("If set to true, This connection will not be TLS-controlled despite its node being controlled"),
                                   GNEAttributeCarrier::FALSE_STR);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_VISIBILITY_DISTANCE,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Vision distance between vehicles"),
                                   "", toString(NBEdge::UNSPECIFIED_VISIBILITY_DISTANCE));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TLLINKINDEX,
                                   GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("sets index of this connection within the controlling traffic light (-1 means automatic assignment)"),
                                   "-1");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TLLINKINDEX2,
                                   GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("sets index for the internal junction of this connection within the controlling traffic light (-1 means internal junction not controlled)"),
                                   "-1");

        fillAllowDisallowAttributes(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SPEED,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("sets custom speed limit for the connection"),
                                   "", toString(NBEdge::UNSPECIFIED_SPEED));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_LENGTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("sets custom length for the connection"),
                                   "", toString(NBEdge::UNSPECIFIED_LOADED_LENGTH));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CUSTOMSHAPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("sets custom shape for the connection"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CHANGE_LEFT,
                                   GNEAttributeProperties::Property::VCLASS | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Permit changing left only for to the given vehicle classes"),
                                   "all");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CHANGE_RIGHT,
                                   GNEAttributeProperties::Property::VCLASS | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Permit changing right only for to the given vehicle classes"),
                                   "all");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_INDIRECT,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("if set to true, vehicles will make a turn in 2 steps"),
                                   GNEAttributeCarrier::FALSE_STR);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TYPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("set a custom edge type (for applying vClass-specific speed restrictions)"));


        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_DIR,
                                   GNEAttributeProperties::Property::STRING,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("turning direction for this connection (computed)"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_STATE,
                                   GNEAttributeProperties::Property::STRING,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("link state for this connection (computed)"));
    }
    currentTag = GNE_TAG_INTERNAL_LANE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_NET),
                GNETagProperties::Type::INTERNALLANE,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::JUNCTION,
                FileBucket::Type::NETWORK,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::JUNCTION, GUIGlObjectType::GLO_TLLOGIC, currentTag, TL("InternalLanes"));
        //  internal lanes does't have attributes
    }
}


void
GNETagPropertiesDatabase::fillAdditionalElements() {
    // fill additional elements
    SumoXMLTag currentTag = SUMO_TAG_BUS_STOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_STOPPINGPLACES),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::STOPPINGPLACE,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::LANE,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::POS_LANE_START | GNETagProperties::Conflicts::POS_LANE_END,
                GUIIcon::BUSSTOP, GUIGlObjectType::GLO_BUS_STOP, currentTag, TL("BusStop"),
                {}, FXRGBA(240, 255, 205, 255));
        // set common attributes
        fillCommonStoppingPlaceAttributes(myTagProperties[currentTag], true, false);

        // set specific attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_LINES,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Meant to be the names of the bus lines that stop at this bus stop. This is only used for visualization purposes"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PERSON_CAPACITY,
                                   GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Larger numbers of persons trying to enter will create an upstream jam on the sidewalk"),
                                   "6");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PARKING_LENGTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Optional space definition for vehicles that park at this stop"),
                                   "0");
    }
    currentTag = SUMO_TAG_TRAIN_STOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_STOPPINGPLACES),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::STOPPINGPLACE,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::LANE,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::POS_LANE_START | GNETagProperties::Conflicts::POS_LANE_END,
                GUIIcon::TRAINSTOP, GUIGlObjectType::GLO_TRAIN_STOP, currentTag, TL("TrainStop"),
                {}, FXRGBA(240, 255, 205, 255));
        // set common attributes
        fillCommonStoppingPlaceAttributes(myTagProperties[currentTag], true, false);

        // set specific attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_LINES,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Meant to be the names of the train lines that stop at this train stop. This is only used for visualization purposes"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PERSON_CAPACITY,
                                   GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Larger numbers of persons trying to enter will create an upstream jam on the sidewalk"),
                                   "6");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PARKING_LENGTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Optional space definition for vehicles that park at this stop"),
                                   "0");

    }
    currentTag = SUMO_TAG_ACCESS;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
                GNETagProperties::Type::ADDITIONALELEMENT,
                GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::REPARENT,
                GNETagProperties::Over::LANE,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::POS_LANE,
                GUIIcon::ACCESS, GUIGlObjectType::GLO_ACCESS, currentTag, TL("Access"),
        {SUMO_TAG_BUS_STOP, SUMO_TAG_TRAIN_STOP, SUMO_TAG_CONTAINER_STOP}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        fillLaneAttribute(myTagProperties[currentTag], false);

        fillPosOverLaneAttribute(myTagProperties[currentTag]);

        fillFriendlyPosAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_LENGTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The walking length of the access in meters (default is geometric length)"),
                                   "", "-1");
    }
    currentTag = SUMO_TAG_CONTAINER_STOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_STOPPINGPLACES),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::STOPPINGPLACE,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::LANE,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::POS_LANE_START | GNETagProperties::Conflicts::POS_LANE_END,
                GUIIcon::CONTAINERSTOP, GUIGlObjectType::GLO_CONTAINER_STOP, currentTag, TL("ContainerStop"),
                {}, FXRGBA(240, 255, 205, 255));
        // set common attributes
        fillCommonStoppingPlaceAttributes(myTagProperties[currentTag], true, false);

        // set specific attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_LINES,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("meant to be the names of the bus lines that stop at this container stop. This is only used for visualization purposes"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CONTAINER_CAPACITY,
                                   GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Larger numbers of container trying to enter will create an upstream jam on the sidewalk"),
                                   "6");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PARKING_LENGTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Optional space definition for vehicles that park at this stop"),
                                   "", "0");
    }
    currentTag = SUMO_TAG_CHARGING_STATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_STOPPINGPLACES),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::STOPPINGPLACE,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::LANE,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::POS_LANE_START | GNETagProperties::Conflicts::POS_LANE_END,
                GUIIcon::CHARGINGSTATION, GUIGlObjectType::GLO_CHARGING_STATION, currentTag, TL("ChargingStation"),
                {}, FXRGBA(240, 255, 205, 255));
        // set common attributes
        fillCommonStoppingPlaceAttributes(myTagProperties[currentTag], false, false);

        // set specific attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CHARGINGPOWER,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Charging power in W"),
                                   "22000");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TOTALPOWER,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Total power in W"),
                                   "0");

        auto efficiency = new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_EFFICIENCY,
                GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::RANGE | GNEAttributeProperties::Property::DEFAULTVALUE,
                GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                TL("Charging efficiency [0,1]"),
                "0.95");
        efficiency->setRange(0, 1);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CHARGEINTRANSIT,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Enable or disable charge in transit, i.e. vehicle must or must not to stop for charging"),
                                   GNEAttributeCarrier::FALSE_STR);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CHARGEDELAY,
                                   GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Time delay after the vehicles has reached / stopped on the charging station, before the energy transfer (charging) begins"),
                                   "0");

        auto chargeType = new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CHARGETYPE,
                GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE,
                GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                TL("Battery charging type"),
                SUMOXMLDefinitions::ChargeTypes.getString(ChargeType::NORMAL));
        chargeType->setDiscreteValues(SUMOXMLDefinitions::ChargeTypes.getStrings());

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_WAITINGTIME,
                                   GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Waiting time before start charging"),
                                   "900");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PARKING_AREA,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Parking area the charging station is located"));
    }
    currentTag = SUMO_TAG_PARKING_AREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_STOPPINGPLACES),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::STOPPINGPLACE,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::LANE,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::POS_LANE_START | GNETagProperties::Conflicts::POS_LANE_END,
                GUIIcon::PARKINGAREA, GUIGlObjectType::GLO_PARKING_AREA, currentTag, TL("ParkingArea"),
                {}, FXRGBA(240, 255, 205, 255));
        // set common attributes
        fillCommonStoppingPlaceAttributes(myTagProperties[currentTag], false, true);

        // set specific attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_DEPARTPOS,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Lane position in that vehicle must depart when leaves parkingArea"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ACCEPTED_BADGES,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Accepted badges to access this parkingArea"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ROADSIDE_CAPACITY,
                                   GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL(" The number of parking spaces for road-side parking"),
                                   "0");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ONROAD,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("If set, vehicles will park on the road lane and thereby reducing capacity"),
                                   "0");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_WIDTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The width of the road-side parking spaces"),
                                   toString(SUMO_const_laneWidth));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_LENGTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The length of the road-side parking spaces. By default (endPos - startPos) / roadsideCapacity"),
                                   "", "0");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_LEFTHAND,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Visual placement to the left of the lane (in righthand networks)"),
                                   GNEAttributeCarrier::FALSE_STR);

    }
    currentTag = SUMO_TAG_PARKING_SPACE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
                GNETagProperties::Type::ADDITIONALELEMENT,
                GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::REPARENT | GNETagProperties::Property::RTREE,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::PARKINGSPACE, GUIGlObjectType::GLO_PARKING_SPACE, currentTag, TL("ParkingSpace"),
        {SUMO_TAG_PARKING_AREA}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_POSITION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::UPDATEGEOMETRY, // virtual attribute from the combination of the actually attributes SUMO_ATTR_X, SUMO_ATTR_Y
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The x-y-z position of the node on the plane in meters"));

        fillNameAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_WIDTH,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The width of the road-side parking spaces"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_LENGTH,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The length of the road-side parking spaces"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ANGLE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The angle of the road-side parking spaces relative to the lane angle, positive means clockwise"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SLOPE,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::ANGLE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The slope of the road-side parking spaces"),
                                   "0");

    }
    currentTag = SUMO_TAG_INDUCTION_LOOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_DETECTORS),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::DETECTOR,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::LANE,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::POS_LANE,
                GUIIcon::E1, GUIGlObjectType::GLO_E1DETECTOR, currentTag, TL("E1 InductionLoop"),
                {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        fillLaneAttribute(myTagProperties[currentTag], false);

        fillPosOverLaneAttribute(myTagProperties[currentTag]);

        fillFriendlyPosAttribute(myTagProperties[currentTag]);

        fillNameAttribute(myTagProperties[currentTag]);

        fillDetectorPeriodAttribute(myTagProperties[currentTag]);

        fillFileAttribute(myTagProperties[currentTag]);

        fillVTypesAttribute(myTagProperties[currentTag]);

        fillDetectorNextEdgesAttribute(myTagProperties[currentTag]);

        fillDetectPersonsAttribute(myTagProperties[currentTag]);
    }
    currentTag = SUMO_TAG_LANE_AREA_DETECTOR;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_DETECTORS),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::DETECTOR,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::LANE,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::E2, GUIGlObjectType::GLO_E2DETECTOR, currentTag, TL("E2 LaneAreaDetector"),
                {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        fillLaneAttribute(myTagProperties[currentTag], false);

        fillPosOverLaneAttribute(myTagProperties[currentTag]);

        // this is a "virtual attribute", only used for movement
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ENDPOS,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::NO_EDIT,
                                   TL("The end position on the lane the detector shall be laid on in meters"));

        fillFriendlyPosAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_LENGTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The length of the detector in meters"),
                                   toString(GNEMoveElementLaneDouble::defaultSize));

        fillNameAttribute(myTagProperties[currentTag]);

        fillDetectorPeriodAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TLID,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The traffic light that triggers aggregation when switching"));

        fillFileAttribute(myTagProperties[currentTag]);

        fillVTypesAttribute(myTagProperties[currentTag]);

        fillDetectorNextEdgesAttribute(myTagProperties[currentTag]);

        fillDetectPersonsAttribute(myTagProperties[currentTag]);

        fillDetectorThresholdAttributes(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SHOW_DETECTOR,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Show detector in sumo-gui"),
                                   GNEAttributeCarrier::TRUE_STR);
    }
    currentTag = GNE_TAG_MULTI_LANE_AREA_DETECTOR;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_DETECTORS),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::DETECTOR,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::CONSECUTIVE_LANES,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::E2, GUIGlObjectType::GLO_E2DETECTOR, SUMO_TAG_LANE_AREA_DETECTOR, TL("E2 MultiLaneAreaDetector"),
                {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_LANES,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::SECUENCIAL | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The sequence of lane ids in which the detector shall be laid on"));

        fillPosOverLaneAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ENDPOS,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The end position on the lane the detector shall be laid on in meters"));

        fillFriendlyPosAttribute(myTagProperties[currentTag]);

        fillDetectorPeriodAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TLID,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The traffic light that triggers aggregation when switching"));

        fillNameAttribute(myTagProperties[currentTag]);

        fillFileAttribute(myTagProperties[currentTag]);

        fillVTypesAttribute(myTagProperties[currentTag]);

        fillDetectorNextEdgesAttribute(myTagProperties[currentTag]);

        fillDetectPersonsAttribute(myTagProperties[currentTag]);

        fillDetectorThresholdAttributes(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SHOW_DETECTOR,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Show detector in sumo-gui"),
                                   GNEAttributeCarrier::TRUE_STR);
    }
    currentTag = SUMO_TAG_ENTRY_EXIT_DETECTOR;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_DETECTORS),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::DETECTOR,
                GNETagProperties::Property::RTREE,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_ADDITIONAL_CHILDREN,
                GUIIcon::E3, GUIGlObjectType::GLO_DET_ENTRYEXIT, currentTag, TL("E3 EntryExitDetector"),
                {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_POSITION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("X-Y position of detector in editor (Only used in netedit)"),
                                   "0,0"); // virtual attribute from the combination of the actually attributes SUMO_ATTR_X, SUMO_ATTR_Y

        fillNameAttribute(myTagProperties[currentTag]);

        fillDetectorPeriodAttribute(myTagProperties[currentTag]);

        fillFileAttribute(myTagProperties[currentTag]);

        fillVTypesAttribute(myTagProperties[currentTag]);

        fillDetectorNextEdgesAttribute(myTagProperties[currentTag]);

        fillDetectPersonsAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_OPEN_ENTRY,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("If set to true, no error will be reported if vehicles leave the detector without first entering it"),
                                   GNEAttributeCarrier::FALSE_STR);

        fillDetectorThresholdAttributes(myTagProperties[currentTag], false);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_EXPECT_ARRIVAL,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Whether no warning should be issued when a vehicle arrives within the detector area."),
                                   GNEAttributeCarrier::FALSE_STR);
    }
    currentTag = SUMO_TAG_DET_ENTRY;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_DETECTORS),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::DETECTOR,
                GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::REPARENT,
                GNETagProperties::Over::LANE,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::POS_LANE,
                GUIIcon::E3ENTRY, GUIGlObjectType::GLO_DET_ENTRY, currentTag, TL("E3 DetEntry"),
        {SUMO_TAG_ENTRY_EXIT_DETECTOR}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        fillLaneAttribute(myTagProperties[currentTag], false);

        fillPosOverLaneAttribute(myTagProperties[currentTag]);

        fillFriendlyPosAttribute(myTagProperties[currentTag]);

    }
    currentTag = SUMO_TAG_DET_EXIT;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_DETECTORS),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::DETECTOR,
                GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::REPARENT,
                GNETagProperties::Over::LANE,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::POS_LANE,
                GUIIcon::E3EXIT, GUIGlObjectType::GLO_DET_EXIT, currentTag, TL("E3 DetExit"),
        {SUMO_TAG_ENTRY_EXIT_DETECTOR}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        fillLaneAttribute(myTagProperties[currentTag], false);

        fillPosOverLaneAttribute(myTagProperties[currentTag]);

        fillFriendlyPosAttribute(myTagProperties[currentTag]);

    }
    currentTag = SUMO_TAG_INSTANT_INDUCTION_LOOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_DETECTORS),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::DETECTOR,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::LANE,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::POS_LANE,
                GUIIcon::E1INSTANT, GUIGlObjectType::GLO_E1DETECTOR_INSTANT, currentTag, TL("E3 DetExit"),
                {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        fillLaneAttribute(myTagProperties[currentTag], false);

        fillPosOverLaneAttribute(myTagProperties[currentTag]);

        fillFriendlyPosAttribute(myTagProperties[currentTag]);

        fillNameAttribute(myTagProperties[currentTag]);

        fillFileAttribute(myTagProperties[currentTag]);

        fillVTypesAttribute(myTagProperties[currentTag]);

        fillDetectorNextEdgesAttribute(myTagProperties[currentTag]);

        fillDetectPersonsAttribute(myTagProperties[currentTag]);
    }
    currentTag = SUMO_TAG_ROUTEPROBE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
                GNETagProperties::Type::ADDITIONALELEMENT,
                GNETagProperties::Property::CENTERAFTERCREATION,
                GNETagProperties::Over::EDGE,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::ROUTEPROBE, GUIGlObjectType::GLO_ROUTEPROBE, currentTag, TL("RouteProbe"),
                {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        fillEdgeAttribute(myTagProperties[currentTag], false);

        fillNameAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PERIOD,
                                   GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The frequency in which to report the distribution"),
                                   "3600");

        fillFileAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_BEGIN,
                                   GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The time at which to start generating output"),
                                   "0");

        fillVTypesAttribute(myTagProperties[currentTag]);
    }
    currentTag = SUMO_TAG_VSS;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
                GNETagProperties::Type::ADDITIONALELEMENT,
                GNETagProperties::Property::RTREE | GNETagProperties::Property::DIALOG,
                GNETagProperties::Over::LANES,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::VARIABLESPEEDSIGN, GUIGlObjectType::GLO_VSS, currentTag, TL("VariableSpeedSign"),
                {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_LANES,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("List of Variable Speed Sign lanes"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_POSITION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("X-Y position of detector in editor (Only used in netedit)"),
                                   "0,0"); // virtual attribute from the combination of the actually attributes SUMO_ATTR_X, SUMO_ATTR_Y

        fillNameAttribute(myTagProperties[currentTag]);

        fillVTypesAttribute(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_VSS_SYMBOL;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
                GNETagProperties::Type::ADDITIONALELEMENT,
                GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS | GNETagProperties::Property::NOTSELECTABLE | GNETagProperties::Property::SYMBOL,
                GNETagProperties::Over::LANE,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::LANE, GUIGlObjectType::GLO_VSS, currentTag, TL("VariableSpeedSign (lane)"),
        {SUMO_TAG_VSS}, FXRGBA(210, 233, 255, 255));
    }
    currentTag = SUMO_TAG_STEP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
                GNETagProperties::Type::ADDITIONALELEMENT,
                GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS | GNETagProperties::Property::LISTED,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::VSSSTEP, GUIGlObjectType::GLO_VSS_STEP, currentTag, TL("VariableSpeedSign Step"),
        {SUMO_TAG_VSS}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TIME,
                                   GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::SORTABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                   TL("Time"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SPEED,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::SORTABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                   TL("Speed"),
                                   toString(OptionsCont::getOptions().getFloat("default.speed")));
    }
    currentTag = SUMO_TAG_CALIBRATOR;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::CALIBRATOR,
                GNETagProperties::Property::DIALOG | GNETagProperties::Property::CENTERAFTERCREATION,
                GNETagProperties::Over::EDGE,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::CALIBRATOR, GUIGlObjectType::GLO_CALIBRATOR, currentTag, TL("Calibrator"),
                {}, FXRGBA(253, 255, 206, 255));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        fillEdgeAttribute(myTagProperties[currentTag], false);

        fillPosOverLaneAttribute(myTagProperties[currentTag]);

        fillNameAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PERIOD,
                                   GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The aggregation interval in which to calibrate the flows. Default is step-length"),
                                   "1");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ROUTEPROBE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The id of the routeProbe element from which to determine the route distribution for generated vehicles"));

        fillOutputAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_JAM_DIST_THRESHOLD,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("A threshold value to detect and clear unexpected jamming"),
                                   "0.50");

        fillVTypesAttribute(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_CALIBRATOR_LANE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::CALIBRATOR,
                GNETagProperties::Property::DIALOG | GNETagProperties::Property::CENTERAFTERCREATION,
                GNETagProperties::Over::LANE,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::CALIBRATOR, GUIGlObjectType::GLO_CALIBRATOR, SUMO_TAG_CALIBRATOR, TL("CalibratorLane"),
                {}, FXRGBA(253, 255, 206, 255));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        fillLaneAttribute(myTagProperties[currentTag], false);

        fillPosOverLaneAttribute(myTagProperties[currentTag]);

        fillNameAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PERIOD,
                                   GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The aggregation interval in which to calibrate the flows. Default is step-length"),
                                   "1");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ROUTEPROBE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The id of the routeProbe element from which to determine the route distribution for generated vehicles"));

        fillOutputAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_JAM_DIST_THRESHOLD,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("A threshold value to detect and clear unexpected jamming"),
                                   "0.50");

        fillVTypesAttribute(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_CALIBRATOR_FLOW;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::CALIBRATOR,
                GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::LISTED,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::FLOW, GUIGlObjectType::GLO_CALIBRATOR_FLOW, SUMO_TAG_FLOW, TL("CalibratorFlow"),
        {SUMO_TAG_CALIBRATOR}, FXRGBA(253, 255, 206, 255));
        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_BEGIN,
                                   GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::SORTABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                   TL("First calibrator flow departure time"),
                                   "0");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_END,
                                   GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::SORTABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                   TL("End of departure interval"),
                                   "3600");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TYPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::ACTIVATABLE | GNEAttributeProperties::Property::VTYPE,
                                   GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                   TL("The id of the vehicle type to use for this calibrator flow"),
                                   DEFAULT_VTYPE_ID);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ROUTE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                   TL("The id of the route the vehicle shall drive along"));

        // at least one of the following attributes must be defined

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_VEHSPERHOUR,
                                   GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::ACTIVATABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                   TL("Number of vehicles per hour, equally spaced"),
                                   "1800");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SPEED,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::ACTIVATABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                   TL("Vehicle's speed"),
                                   "15");

        // fill common vehicle attributes
        fillCommonVehicleAttributes(myTagProperties[currentTag]);
    }
    currentTag = SUMO_TAG_REROUTER;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
                GNETagProperties::Type::ADDITIONALELEMENT,
                GNETagProperties::Property::RTREE | GNETagProperties::Property::DIALOG,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::REROUTER, GUIGlObjectType::GLO_REROUTER, currentTag, TL("Rerouter"),
                {}, FXRGBA(255, 213, 213, 255));

        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_POSITION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("X,Y position in editor (Only used in netedit)"),
                                   "0,0"); // virtual attribute from the combination of the actually attributes SUMO_ATTR_X, SUMO_ATTR_Y

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_EDGES,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("An edge id or a list of edge ids where vehicles shall be rerouted"));

        fillNameAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PROB,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::PROBABILITY | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The probability for vehicle rerouting (0-1)"),
                                   "1");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_HALTING_TIME_THRESHOLD,
                                   GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The waiting time threshold (in s) that must be reached to activate rerouting (default -1 which disables the threshold)"),
                                   "0");

        fillVTypesAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_OFF,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Whether the router should be inactive initially (and switched on in the gui)"),
                                   GNEAttributeCarrier::FALSE_STR);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_OPTIONAL,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("If rerouter is optional"),
                                   GNEAttributeCarrier::FALSE_STR);
    }
    currentTag = GNE_TAG_REROUTER_SYMBOL;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
                GNETagProperties::Type::ADDITIONALELEMENT,
                GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS | GNETagProperties::Property::NOTSELECTABLE | GNETagProperties::Property::SYMBOL,
                GNETagProperties::Over::EDGE,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::EDGE, GUIGlObjectType::GLO_REROUTER, currentTag, TL("Rerouter (Edge)"),
        {GNE_TAG_REROUTER_SYMBOL}, FXRGBA(255, 213, 213, 255));
    }
    currentTag = SUMO_TAG_INTERVAL;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
                GNETagProperties::Type::ADDITIONALELEMENT,
                GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::REROUTERINTERVAL, GUIGlObjectType::GLO_REROUTER_INTERVAL, currentTag, TL("Rerouter Interval"),
        {SUMO_TAG_REROUTER}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_BEGIN,
                                   GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::SORTABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                   TL("Begin"),
                                   "0");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_END,
                                   GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::SORTABLE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                   TL("End"),
                                   "3600");
    }
    currentTag = SUMO_TAG_CLOSING_REROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
                GNETagProperties::Type::ADDITIONALELEMENT,
                GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS | GNETagProperties::Property::LISTED,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::CLOSINGREROUTE, GUIGlObjectType::GLO_REROUTER_CLOSINGREROUTE, currentTag, TL("ClosingReroute"),
        {SUMO_TAG_INTERVAL}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        fillEdgeAttribute(myTagProperties[currentTag], true);

        fillAllowDisallowAttributes(myTagProperties[currentTag]);
    }
    currentTag = SUMO_TAG_CLOSING_LANE_REROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
                GNETagProperties::Type::ADDITIONALELEMENT,
                GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS | GNETagProperties::Property::LISTED,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::CLOSINGLANEREROUTE, GUIGlObjectType::GLO_REROUTER_CLOSINGLANEREROUTE, currentTag, TL("ClosingLaneReroute"),
        {SUMO_TAG_INTERVAL}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        fillLaneAttribute(myTagProperties[currentTag], true);

        fillAllowDisallowAttributes(myTagProperties[currentTag]);
    }
    currentTag = SUMO_TAG_DEST_PROB_REROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
                GNETagProperties::Type::ADDITIONALELEMENT,
                GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS | GNETagProperties::Property::LISTED,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::DESTPROBREROUTE, GUIGlObjectType::GLO_REROUTER_DESTPROBREROUTE, currentTag, TL("DestinationProbabilityReroute"),
        {SUMO_TAG_INTERVAL}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        fillEdgeAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PROB,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                   TL("SUMO Probability"),
                                   "1");
    }
    currentTag = SUMO_TAG_PARKING_AREA_REROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
                GNETagProperties::Type::ADDITIONALELEMENT,
                GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS | GNETagProperties::Property::LISTED,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::PARKINGZONEREROUTE, GUIGlObjectType::GLO_REROUTER_PARKINGAREAREROUTE, currentTag, TL("ParkingAreaReroute"),
        {SUMO_TAG_INTERVAL}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        auto parking = new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PARKING,
                GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::SYNONYM,
                GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                TL("ParkingArea ID"));
        parking->setSynonym(SUMO_ATTR_ID);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PROB,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                   TL("SUMO Probability"),
                                   "1");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_VISIBLE,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                   TL("Enable or disable visibility for parking area reroutes"),
                                   GNEAttributeCarrier::TRUE_STR);
    }
    currentTag = SUMO_TAG_ROUTE_PROB_REROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
                GNETagProperties::Type::ADDITIONALELEMENT,
                GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS | GNETagProperties::Property::LISTED,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::ROUTEPROBREROUTE, GUIGlObjectType::GLO_REROUTER_ROUTEPROBREROUTE, currentTag, TL("RouteProbabilityReroute"),
        {SUMO_TAG_INTERVAL}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        auto route = new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ROUTE,
                                                GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::SYNONYM | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                                GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                                TL("Route"));
        route->setSynonym(SUMO_ATTR_ID);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PROB,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                   TL("SUMO Probability"),
                                   "1");
    }
    currentTag = SUMO_TAG_VAPORIZER;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_ADDITIONALS),
                GNETagProperties::Type::ADDITIONALELEMENT,
                GNETagProperties::Property::CENTERAFTERCREATION,
                GNETagProperties::Over::EDGE,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::VAPORIZER, GUIGlObjectType::GLO_VAPORIZER, currentTag, TL("Vaporizer"),
                {}, FXRGBA(253, 255, 206, 255));
        // set values of attributes
        fillEdgeAttribute(myTagProperties[currentTag], true);

        fillNameAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_BEGIN,
                                   GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Start Time"),
                                   "0");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_END,
                                   GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("End Time"),
                                   "3600");
    }
}


void
GNETagPropertiesDatabase::fillShapeElements() {
    // fill shape ACs
    SumoXMLTag currentTag = SUMO_TAG_POLY;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_SHAPES),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::SHAPE,
                GNETagProperties::Property::RTREE | GNETagProperties::Property::GEOSHAPE,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::POLY, GUIGlObjectType::GLO_POLYGON, currentTag, TL("Polygon"),
                {}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SHAPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The shape of the polygon"));

        fillNameAttribute(myTagProperties[currentTag]);

        fillColorAttribute(myTagProperties[currentTag], "red");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_FILL,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("An information whether the polygon shall be filled"),
                                   GNEAttributeCarrier::FALSE_STR);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_LINEWIDTH,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The default line width for drawing an unfilled polygon"),
                                   "1");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_LAYER,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The layer in which the polygon lies"),
                                   toString(Shape::DEFAULT_LAYER));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TYPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("A typename for the polygon"),
                                   toString(Shape::DEFAULT_TYPE));

        fillImgFileAttribute(myTagProperties[currentTag], false);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ANGLE,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::ANGLE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Angle of rendered image in degree"),
                                   toString(Shape::DEFAULT_ANGLE));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_GEO,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::GEOEDITOR,
                                   TL("Enable or disable GEO attributes"),
                                   GNEAttributeCarrier::FALSE_STR);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_GEOSHAPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::GEOEDITOR,
                                   TL("A custom geo shape for this polygon"));

        new GNEAttributeProperties(myTagProperties[currentTag], GNE_ATTR_CLOSE_SHAPE,
                                   GNEAttributeProperties::Property::BOOL,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::NETEDITEDITOR,
                                   TL("Toggle close or open shape"));
    }
    currentTag = SUMO_TAG_POI;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_SHAPES),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::SHAPE,
                GNETagProperties::Property::RTREE,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::POI, GUIGlObjectType::GLO_POI, currentTag, TL("PointOfInterest"),
                {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_POSITION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY, // virtual attribute from the combination of the actually attributes SUMO_ATTR_X, SUMO_ATTR_Y
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The position in view"));

        // fill Poi attributes
        fillCommonPOIAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_POILANE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_SHAPES),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::SHAPE,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::LANE,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::POS_LANE,
                GUIIcon::POILANE, GUIGlObjectType::GLO_POI, SUMO_TAG_POI, TL("PointOfInterestLane"),
                {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        fillLaneAttribute(myTagProperties[currentTag], false);

        fillPosOverLaneAttribute(myTagProperties[currentTag]);

        fillFriendlyPosAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_POSITION_LAT,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The lateral offset on the named lane at which the POI is located at"),
                                   "0");

        // fill Poi attributes
        fillCommonPOIAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_POIGEO;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_SHAPES),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::SHAPE,
                GNETagProperties::Property::RTREE | GNETagProperties::Property::REQUIRE_PROJ,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::POIGEO, GUIGlObjectType::GLO_POI, SUMO_TAG_POI, TL("PointOfInterestGeo"),
                {}, FXRGBA(210, 233, 255, 255));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_LON,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::GEOEDITOR,
                                   TL("The longitude position of the parking vehicle on the view"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_LAT,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::GEOEDITOR,
                                   TL("The latitude position of the parking vehicle on the view"));

        // fill Poi attributes
        fillCommonPOIAttributes(myTagProperties[currentTag]);
    }
}


void
GNETagPropertiesDatabase::fillTAZElements() {
    // fill TAZ ACs
    SumoXMLTag currentTag = SUMO_TAG_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TAZS),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::TAZELEMENT,
                GNETagProperties::Property::RTREE,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::TAZ, GUIGlObjectType::GLO_TAZ, currentTag, TL("TrafficAssignmentZones"));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SHAPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The shape of the TAZ"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CENTER,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("TAZ center"));

        fillNameAttribute(myTagProperties[currentTag]);

        fillColorAttribute(myTagProperties[currentTag], "red");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_FILL,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("An information whether the TAZ shall be filled"),
                                   GNEAttributeCarrier::FALSE_STR);

        auto edgesWithin = new GNEAttributeProperties(myTagProperties[currentTag], GNE_ATTR_EDGES_WITHIN,
                GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::NETEDITEDITOR,
                TL("Use the edges within the shape"),
                GNEAttributeCarrier::TRUE_STR);
        edgesWithin->setAlternativeName(TL("edges within"));
    }
    currentTag = SUMO_TAG_TAZSOURCE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TAZS),
                GNETagProperties::Type::OTHER,
                GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS,
                GNETagProperties::Over::EDGE,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::TAZEDGE, GUIGlObjectType::GLO_TAZ, currentTag, TL("TAZ Source"),
        {SUMO_TAG_TAZ});
        // set values of attributes
        fillEdgeAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_WEIGHT,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Depart weight associated to this Edge"),
                                   "1");
    }
    currentTag = SUMO_TAG_TAZSINK;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TAZS),
                GNETagProperties::Type::OTHER,
                GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS,
                GNETagProperties::Over::EDGE,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::TAZEDGE, GUIGlObjectType::GLO_TAZ, currentTag, TL("TAZ Sink"),
        {SUMO_TAG_TAZ});
        // set values of attributes
        fillEdgeAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_WEIGHT,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Arrival weight associated to this Edge"),
                                   "1");
    }
}


void
GNETagPropertiesDatabase::fillWireElements() {
    // fill wire elements
    SumoXMLTag currentTag = SUMO_TAG_TRACTION_SUBSTATION;
    {
        // set tag properties
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WIRES),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::WIRE,
                GNETagProperties::Property::RTREE,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::TRACTION_SUBSTATION, GUIGlObjectType::GLO_TRACTIONSUBSTATION, currentTag, TL("TractionSubstation"));
        // set attribute properties
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_POSITION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("X-Y position of detector in editor (Only used in netedit)"),
                                   "0,0"); // virtual attribute from the combination of the actually attributes SUMO_ATTR_X, SUMO_ATTR_Y

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_VOLTAGE,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Voltage of at connection point for the overhead wire"),
                                   "600");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CURRENTLIMIT,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Current limit of the feeder line"),
                                   "400");
    }
    currentTag = SUMO_TAG_OVERHEAD_WIRE_SECTION;
    {
        // set tag properties
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WIRES),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::WIRE,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::CONSECUTIVE_LANES,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::OVERHEADWIRE, GUIGlObjectType::GLO_OVERHEAD_WIRE_SEGMENT, currentTag, TL("WireSection"));
        // set attribute properties
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SUBSTATIONID,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Substation to which the circuit is connected"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_LANES,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("List of consecutive lanes of the circuit"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_STARTPOS,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Starting position in the specified lane"),
                                   "", "0");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ENDPOS,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Ending position in the specified lane"),
                                   "", "INVALID_DOUBLE");

        fillFriendlyPosAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_OVERHEAD_WIRE_FORBIDDEN,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Inner lanes, where placing of overhead wire is restricted"));
    }
    currentTag = SUMO_TAG_OVERHEAD_WIRE_CLAMP;
    {
        // set tag properties
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WIRES),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::WIRE,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::OVERHEADWIRE_CLAMP, GUIGlObjectType::GLO_OVERHEAD_WIRE_SEGMENT, currentTag, TL("OverheadWireClamp"));
        // set attribute properties
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_OVERHEAD_WIRECLAMP_START,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("ID of the overhead wire segment, to the start of which the overhead wire clamp is connected"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_OVERHEAD_WIRECLAMP_LANESTART,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("ID of the overhead wire segment lane of overheadWireIDStartClamp"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_OVERHEAD_WIRECLAMP_END,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("ID of the overhead wire segment, to the end of which the overhead wire clamp is connected"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_OVERHEAD_WIRECLAMP_LANEEND,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("ID of the overhead wire segment lane of overheadWireIDEndClamp"));
    }
}


void
GNETagPropertiesDatabase::fillJuPedSimElements() {
    // fill shape ACs
    SumoXMLTag currentTag = GNE_TAG_JPS_WALKABLEAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_JUPEDSIM),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::SHAPE | GNETagProperties::Type::JUPEDSIM,
                GNETagProperties::Property::RTREE,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::JPS_WALKABLEAREA, GUIGlObjectType::GLO_JPS_WALKABLEAREA, SUMO_TAG_POLY, TL("JuPedSim WalkableArea"),
                {}, FXRGBA(253, 255, 206, 255));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SHAPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The shape of the walkable area"));

        fillNameAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_GEO,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::GEOEDITOR,
                                   TL("Enable or disable GEO attributes"),
                                   GNEAttributeCarrier::FALSE_STR);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_GEOSHAPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::GEOEDITOR,
                                   TL("A custom geo shape for this walkable area"));
    }
    currentTag = GNE_TAG_JPS_OBSTACLE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_JUPEDSIM),
                GNETagProperties::Type::ADDITIONALELEMENT | GNETagProperties::Type::SHAPE | GNETagProperties::Type::JUPEDSIM,
                GNETagProperties::Property::RTREE,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::JPS_OBSTACLE, GUIGlObjectType::GLO_JPS_OBSTACLE, SUMO_TAG_POLY, TL("JuPedSim Obstacle"),
                {}, FXRGBA(253, 255, 206, 255));
        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_SHAPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The shape of the obstacle"));

        fillNameAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_GEO,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::GEOEDITOR,
                                   TL("Enable or disable GEO attributes"),
                                   GNEAttributeCarrier::FALSE_STR);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_GEOSHAPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::POSITION | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::GEOEDITOR,
                                   TL("A custom geo shape for this obstacle"));
    }
}


void
GNETagPropertiesDatabase::fillDemandElements() {
    // fill demand elements
    SumoXMLTag currentTag = SUMO_TAG_ROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties[GNE_TAG_SUPERMODE_DEMAND],
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::ROUTE,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::CONSECUTIVE_EDGES,
                FileBucket::Type::DEMAND | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::ROUTE, GUIGlObjectType::GLO_ROUTE, currentTag, TL("Route"));

        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        // add common route attributes
        fillCommonRouteAttributes(myTagProperties[currentTag]);

        // add distribution probability
        fillDistributionProbability(myTagProperties[currentTag], true);
    }
    currentTag = GNE_TAG_ROUTE_EMBEDDED;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties[GNE_TAG_SUPERMODE_DEMAND],
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::ROUTE,
                GNETagProperties::Property::XMLCHILD,
                GNETagProperties::Over::CONSECUTIVE_EDGES,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::ROUTE, GUIGlObjectType::GLO_ROUTE_EMBEDDED, SUMO_TAG_ROUTE, TL("Route (embedded)"),
        {GNE_TAG_VEHICLE_WITHROUTE, GNE_TAG_FLOW_WITHROUTE});

        // add common route attributes
        fillCommonRouteAttributes(myTagProperties[currentTag]);
    }
    currentTag = SUMO_TAG_ROUTE_DISTRIBUTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties[GNE_TAG_SUPERMODE_DEMAND],
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::ROUTE | GNETagProperties::Type::DISTRIBUTION,
                GNETagProperties::Property::NOTDRAWABLE | GNETagProperties::Property::NOTSELECTABLE | GNETagProperties::Property::NOPARAMETERS,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::DEMAND | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::ROUTEDISTRIBUTION, GUIGlObjectType::GLO_ROUTE_DISTRIBUTION, currentTag, TL("RouteDistribution"));

        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);
    }
    currentTag = GNE_TAG_ROUTEREF;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties[GNE_TAG_SUPERMODE_DEMAND],
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::DISTRIBUTIONREF,
                GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::ROUTEREF, GUIGlObjectType::GLO_ROUTE_REF, currentTag, TL("Route (Ref)"),
        {SUMO_TAG_ROUTE_DISTRIBUTION});

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_REFID,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Reference ID of route"));

        // add distribution probability
        fillDistributionProbability(myTagProperties[currentTag], false);
    }
    currentTag = SUMO_TAG_VTYPE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties[GNE_TAG_SUPERMODE_DEMAND],
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::VTYPE,
                GNETagProperties::Property::NOTDRAWABLE | GNETagProperties::Property::NOTSELECTABLE | GNETagProperties::Property::VCLASS_ICON | GNETagProperties::Property::EXTENDED,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::DEMAND | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::VTYPE, GUIGlObjectType::GLO_VTYPE, currentTag, TL("VehicleType"));

        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        // add common vType attributes
        fillCommonVTypeAttributes(myTagProperties[currentTag]);
    }
    currentTag = SUMO_TAG_VTYPE_DISTRIBUTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties[GNE_TAG_SUPERMODE_DEMAND],
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::VTYPE | GNETagProperties::Type::DISTRIBUTION,
                GNETagProperties::Property::NOTDRAWABLE | GNETagProperties::Property::NOTSELECTABLE | GNETagProperties::Property::NOPARAMETERS,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::DEMAND | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::VTYPEDISTRIBUTION, GUIGlObjectType::GLO_VTYPE_DISTRIBUTION, currentTag, TL("TypeDistribution"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_DETERMINISTIC,
                                   GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Deterministic distribution"),
                                   "-1");

        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);
    }
    currentTag = GNE_TAG_VTYPEREF;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties[GNE_TAG_SUPERMODE_DEMAND],
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::DISTRIBUTIONREF,
                GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::VTYPEREF, GUIGlObjectType::GLO_VTYPE_REF, currentTag, TL("VType (Ref)"),
        {SUMO_TAG_VTYPE_DISTRIBUTION});

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_REFID,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Reference ID of vType"));

        // add distribution probability
        fillDistributionProbability(myTagProperties[currentTag], false);
    }
}


void
GNETagPropertiesDatabase::fillVehicleElements() {
    // fill vehicle ACs
    SumoXMLTag currentTag = SUMO_TAG_TRIP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_VEHICLES),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::VEHICLE,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_EDGE,
                FileBucket::Type::DEMAND | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::TRIP, GUIGlObjectType::GLO_TRIP, currentTag, TL("TripEdges"),
                {}, FXRGBA(253, 255, 206, 255), "trip (from-to edges)");

        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TYPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::VTYPE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The id of the vehicle type to use for this trip"),
                                   DEFAULT_VTYPE_ID);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_FROM,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The ID of the edge the trip starts at"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TO,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The ID of the edge the trip ends at"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_VIA,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::LIST,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("List of intermediate edge ids which shall be part of the trip"));

        // add common attributes
        fillCommonVehicleAttributes(myTagProperties[currentTag]);

        fillDepartAttribute(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRIP_JUNCTIONS;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_VEHICLES),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::VEHICLE,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::FROM_JUNCTION | GNETagProperties::Over::TO_JUNCTION,
                FileBucket::Type::DEMAND | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::TRIP_JUNCTIONS, GUIGlObjectType::GLO_TRIP, SUMO_TAG_TRIP, TL("TripJunctions"),
                {}, FXRGBA(255, 213, 213, 255), "trip (from-to junctions)");

        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TYPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::VTYPE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The id of the vehicle type to use for this trip"),
                                   DEFAULT_VTYPE_ID);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_FROM_JUNCTION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The name of the junction the trip starts at"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TO_JUNCTION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The name of the junction the trip ends at"));

        // add common attributes
        fillCommonVehicleAttributes(myTagProperties[currentTag]);

        fillDepartAttribute(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRIP_TAZS;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_VEHICLES),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::VEHICLE,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_TAZ,
                FileBucket::Type::DEMAND | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::TRIP_TAZS, GUIGlObjectType::GLO_TRIP, SUMO_TAG_TRIP, TL("TripTAZs"),
                {}, FXRGBA(240, 255, 205, 255), "trip (from-to TAZs)");

        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TYPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::VTYPE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The id of the vehicle type to use for this trip"),
                                   DEFAULT_VTYPE_ID);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_FROM_TAZ,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The name of the TAZ the trip starts at"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TO_TAZ,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The name of the TAZ the trip ends at"));

        // add common attributes
        fillCommonVehicleAttributes(myTagProperties[currentTag]);

        fillDepartAttribute(myTagProperties[currentTag]);
    }
    currentTag = SUMO_TAG_VEHICLE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_VEHICLES),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::VEHICLE,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::ROUTE,
                FileBucket::Type::DEMAND | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::VEHICLE, GUIGlObjectType::GLO_VEHICLE, SUMO_TAG_VEHICLE, TL("VehicleRoute"),
                {}, FXRGBA(210, 233, 255, 255), "vehicle (over route)");

        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TYPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::VTYPE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The id of the vehicle type to use for this vehicle"),
                                   DEFAULT_VTYPE_ID);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ROUTE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The id of the route the vehicle shall drive along"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_DEPARTEDGE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The index of the edge within route the vehicle starts at"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ARRIVALEDGE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The index of the edge within route the vehicle ends at"));

        // add common attributes
        fillCommonVehicleAttributes(myTagProperties[currentTag]);

        fillDepartAttribute(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_VEHICLE_WITHROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_VEHICLES),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::VEHICLE,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::ROUTE_EMBEDDED,
                FileBucket::Type::DEMAND | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::VEHICLE, GUIGlObjectType::GLO_VEHICLE, SUMO_TAG_VEHICLE, TL("VehicleEmbeddedRoute"),
                {}, FXRGBA(210, 233, 255, 255), "vehicle (embedded route)");

        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TYPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::VTYPE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The id of the vehicle type to use for this vehicle"),
                                   DEFAULT_VTYPE_ID);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_DEPARTEDGE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The index of the edge within route the vehicle starts at"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ARRIVALEDGE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The index of the edge within route the vehicle ends at"));

        // add common attributes
        fillCommonVehicleAttributes(myTagProperties[currentTag]);

        fillDepartAttribute(myTagProperties[currentTag]);
    }
    currentTag = SUMO_TAG_FLOW;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_VEHICLES),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::VEHICLE | GNETagProperties::Type::FLOW,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_EDGE,
                FileBucket::Type::DEMAND | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::FLOW, GUIGlObjectType::GLO_FLOW, currentTag, TL("FlowEdges"),
                {}, FXRGBA(253, 255, 206, 255), "flow (from-to edges)");

        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TYPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::VTYPE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The id of the flow type to use for this flow"),
                                   DEFAULT_VTYPE_ID);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_FROM,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The ID of the edge the flow starts at"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TO,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The ID of the edge the flow ends at"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_VIA,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::LIST,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("List of intermediate edge ids which shall be part of the flow"));

        // add common attributes
        fillCommonVehicleAttributes(myTagProperties[currentTag]);

        // add flow attributes
        fillCommonFlowAttributes(myTagProperties[currentTag], SUMO_ATTR_VEHSPERHOUR);
    }
    currentTag = GNE_TAG_FLOW_JUNCTIONS;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_VEHICLES),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::VEHICLE | GNETagProperties::Type::FLOW,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::FROM_JUNCTION | GNETagProperties::Over::TO_JUNCTION,
                FileBucket::Type::DEMAND | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::FLOW_JUNCTIONS, GUIGlObjectType::GLO_FLOW, SUMO_TAG_FLOW, TL("FlowJunctions"),
                {}, FXRGBA(255, 213, 213, 255), "flow (from-to junctions)");

        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TYPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::VTYPE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The id of the flow type to use for this flow"),
                                   DEFAULT_VTYPE_ID);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_FROM_JUNCTION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The name of the junction the flow starts at"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TO_JUNCTION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The name of the junction the flow ends at"));

        // add common attributes
        fillCommonVehicleAttributes(myTagProperties[currentTag]);

        // add flow attributes
        fillCommonFlowAttributes(myTagProperties[currentTag], SUMO_ATTR_VEHSPERHOUR);
    }
    currentTag = GNE_TAG_FLOW_TAZS;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_VEHICLES),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::VEHICLE | GNETagProperties::Type::FLOW,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_TAZ,
                FileBucket::Type::DEMAND | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::FLOW_TAZS, GUIGlObjectType::GLO_FLOW, SUMO_TAG_FLOW, TL("FlowTAZs"),
                {}, FXRGBA(240, 255, 205, 255), "flow (from-to TAZs)");

        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TYPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::VTYPE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The id of the flow type to use for this flow"),
                                   DEFAULT_VTYPE_ID);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_FROM_TAZ,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The name of the TAZ the flow starts at"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TO_TAZ,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The name of the TAZ the flow ends at"));

        // add common attributes
        fillCommonVehicleAttributes(myTagProperties[currentTag]);

        // add flow attributes
        fillCommonFlowAttributes(myTagProperties[currentTag], SUMO_ATTR_VEHSPERHOUR);
    }
    currentTag = GNE_TAG_FLOW_ROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_VEHICLES),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::VEHICLE | GNETagProperties::Type::FLOW,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::ROUTE,
                FileBucket::Type::DEMAND | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::ROUTEFLOW, GUIGlObjectType::GLO_ROUTEFLOW, SUMO_TAG_FLOW, TL("FlowRoute"),
                {}, FXRGBA(210, 233, 255, 255), "flow (over route)");

        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TYPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::VTYPE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The id of the flow type to use for this flow"),
                                   DEFAULT_VTYPE_ID);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ROUTE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The id of the route the flow shall drive along"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_DEPARTEDGE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The index of the edge within route the flow starts at"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ARRIVALEDGE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The index of the edge within route the flow ends at"));

        // add common attributes
        fillCommonVehicleAttributes(myTagProperties[currentTag]);

        // add flow attributes
        fillCommonFlowAttributes(myTagProperties[currentTag], SUMO_ATTR_VEHSPERHOUR);
    }
    currentTag = GNE_TAG_FLOW_WITHROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(SUMO_TAG_VIEWSETTINGS_VEHICLES),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::VEHICLE | GNETagProperties::Type::FLOW,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::ROUTE_EMBEDDED,
                FileBucket::Type::DEMAND | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::ROUTEFLOW, GUIGlObjectType::GLO_ROUTEFLOW, SUMO_TAG_FLOW, TL("FlowEmbeddedRoute"),
                {}, FXRGBA(210, 233, 255, 255), "flow (embedded route)");

        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TYPE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::VTYPE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The id of the flow type to use for this flow"),
                                   DEFAULT_VTYPE_ID);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_DEPARTEDGE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The index of the edge within route the flow starts at"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ARRIVALEDGE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The index of the edge within route the flow ends at"));

        // add common attributes
        fillCommonVehicleAttributes(myTagProperties[currentTag]);

        // add flow attributes
        fillCommonFlowAttributes(myTagProperties[currentTag], SUMO_ATTR_VEHSPERHOUR);
    }
}


void
GNETagPropertiesDatabase::fillStopElements() {
    // fill stops ACs
    SumoXMLTag currentTag = GNE_TAG_STOP_LANE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_STOPS),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::STOP_VEHICLE,
                GNETagProperties::Property::XMLCHILD,
                GNETagProperties::Over::LANE,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::POS_LANE_START | GNETagProperties::Conflicts::POS_LANE_END,
                GUIIcon::STOPELEMENT, GUIGlObjectType::GLO_STOP, SUMO_TAG_STOP, TL("StopLane"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        fillLaneAttribute(myTagProperties[currentTag], false);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_STARTPOS,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The begin position on the lane (the lower position on the lane) in meters"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ENDPOS,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The end position on the lane (the higher position on the lane) in meters, must be larger than startPos by more than 0.1m"));

        fillFriendlyPosAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_POSITION_LAT,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The lateral offset on the named lane at which the vehicle must stop"));

        // fill common stop attributes
        fillCommonStopAttributes(myTagProperties[currentTag], false);
        /*
            // netedit attributes
            new GNEAttributeProperties(myTagProperties[currentTag], GNE_ATTR_SIZE,
                    GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Edit::NETEDITEDITOR,
                    TLF("Length of %", myTagProperties[currentTag]->getTagStr()));

            new GNEAttributeProperties(myTagProperties[currentTag], GNE_ATTR_FORCESIZE,
                    GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Edit::NETEDITEDITOR,
                    GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                    TL("Force size during creation"),
                    GNEAttributeCarrier::FALSE_STR);

            new GNEAttributeProperties(myTagProperties[currentTag], GNE_ATTR_REFERENCE,
                    GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Edit::NETEDITEDITOR,
                    TLF("Reference position used for creating %", myTagProperties[currentTag]->getTagStr()));
            attrProperty->setDiscreteValues(SUMOXMLDefinitions::ReferencePositions.getStrings());
            */
    }
    currentTag = GNE_TAG_STOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_STOPS),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::STOP_VEHICLE,
                GNETagProperties::Property::XMLCHILD,
                GNETagProperties::Over::BUSSTOP,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::STOPELEMENT, GUIGlObjectType::GLO_STOP, SUMO_TAG_STOP, TL("StopBusStop"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_BUS_STOP,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("BusStop associated with this stop"));

        // fill common stop attributes
        fillCommonStopAttributes(myTagProperties[currentTag], false);
    }
    currentTag = GNE_TAG_STOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_STOPS),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::STOP_VEHICLE,
                GNETagProperties::Property::XMLCHILD,
                GNETagProperties::Over::TRAINSTOP,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::STOPELEMENT, GUIGlObjectType::GLO_STOP, SUMO_TAG_STOP, TL("StopTrainStop"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TRAIN_STOP,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("TrainStop associated with this stop"));

        // fill common stop attributes
        fillCommonStopAttributes(myTagProperties[currentTag], false);
    }
    currentTag = GNE_TAG_STOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_STOPS),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::STOP_VEHICLE,
                GNETagProperties::Property::XMLCHILD,
                GNETagProperties::Over::CONTAINERSTOP,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::STOPELEMENT, GUIGlObjectType::GLO_STOP, SUMO_TAG_STOP, TL("StopContainerStop"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CONTAINER_STOP,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("ContainerStop associated with this stop"));

        // fill common stop attributes
        fillCommonStopAttributes(myTagProperties[currentTag], false);
    }
    currentTag = GNE_TAG_STOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_STOPS),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::STOP_VEHICLE,
                GNETagProperties::Property::XMLCHILD,
                GNETagProperties::Over::CHARGINGSTATION,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::STOPELEMENT, GUIGlObjectType::GLO_STOP, SUMO_TAG_STOP, TL("StopChargingStation"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CHARGING_STATION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("ChargingStation associated with this stop"));

        // fill common stop attributes
        fillCommonStopAttributes(myTagProperties[currentTag], false);
    }
    currentTag = GNE_TAG_STOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_STOPS),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::STOP_VEHICLE,
                GNETagProperties::Property::XMLCHILD,
                GNETagProperties::Over::PARKINGAREA,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::STOPELEMENT, GUIGlObjectType::GLO_STOP, SUMO_TAG_STOP, TL("StopParkingArea"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(255, 213, 213, 255));
        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PARKING_AREA,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("ParkingArea associated with this stop"));

        // fill common stop attributes (no parking)
        fillCommonStopAttributes(myTagProperties[currentTag], false);
    }
}


void
GNETagPropertiesDatabase::fillWaypointElements() {
    // fill waypoints ACs
    SumoXMLTag currentTag = GNE_TAG_WAYPOINT_LANE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_STOPS),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::STOP_VEHICLE | GNETagProperties::Type::WAYPOINT_VEHICLE,
                GNETagProperties::Property::XMLCHILD,
                GNETagProperties::Over::LANE,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::POS_LANE_START | GNETagProperties::Conflicts::POS_LANE_END,
                GUIIcon::WAYPOINT, GUIGlObjectType::GLO_STOP, SUMO_TAG_STOP, TL("WaypointLane"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        fillLaneAttribute(myTagProperties[currentTag], false);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_STARTPOS,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The begin position on the lane (the lower position on the lane) in meters"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_ENDPOS,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The end position on the lane (the higher position on the lane) in meters, must be larger than startPos by more than 0.1m"));

        fillFriendlyPosAttribute(myTagProperties[currentTag]);

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_POSITION_LAT,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The lateral offset on the named lane at which the vehicle must waypoint"));

        // fill common waypoint (stop) attributes
        fillCommonStopAttributes(myTagProperties[currentTag], true);
        /*
            // netedit attributes
            new GNEAttributeProperties(myTagProperties[currentTag], GNE_ATTR_SIZE,
                    GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Edit::NETEDITEDITOR,
                    TLF("Length of %", myTagProperties[currentTag]->getTagStr()));

            new GNEAttributeProperties(myTagProperties[currentTag], GNE_ATTR_FORCESIZE,
                    GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Edit::NETEDITEDITOR,
                    GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                    TL("Force size during creation"),
                    GNEAttributeCarrier::FALSE_STR);

            new GNEAttributeProperties(myTagProperties[currentTag], GNE_ATTR_REFERENCE,
                    GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Edit::NETEDITEDITOR,
                    TLF("Reference position used for creating %", myTagProperties[currentTag]->getTagStr()));
            attrProperty->setDiscreteValues(SUMOXMLDefinitions::ReferencePositions.getStrings());
            */
    }
    currentTag = GNE_TAG_WAYPOINT_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_STOPS),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::STOP_VEHICLE | GNETagProperties::Type::WAYPOINT_VEHICLE,
                GNETagProperties::Property::XMLCHILD,
                GNETagProperties::Over::BUSSTOP,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::WAYPOINT, GUIGlObjectType::GLO_STOP, SUMO_TAG_STOP, TL("WaypointBusStop"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_BUS_STOP,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("BusWaypoint associated with this waypoint"));

        // fill common waypoint (stop) attributes
        fillCommonStopAttributes(myTagProperties[currentTag], true);
    }
    currentTag = GNE_TAG_WAYPOINT_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_STOPS),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::STOP_VEHICLE | GNETagProperties::Type::WAYPOINT_VEHICLE,
                GNETagProperties::Property::XMLCHILD,
                GNETagProperties::Over::TRAINSTOP,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::WAYPOINT, GUIGlObjectType::GLO_STOP, SUMO_TAG_STOP, TL("WaypointTrainStop"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TRAIN_STOP,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("TrainWaypoint associated with this waypoint"));

        // fill common waypoint (stop) attributes
        fillCommonStopAttributes(myTagProperties[currentTag], true);
    }
    currentTag = GNE_TAG_WAYPOINT_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_STOPS),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::STOP_VEHICLE | GNETagProperties::Type::WAYPOINT_VEHICLE,
                GNETagProperties::Property::XMLCHILD,
                GNETagProperties::Over::CONTAINERSTOP,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::WAYPOINT, GUIGlObjectType::GLO_STOP, SUMO_TAG_STOP, TL("WaypointContainerStop"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CONTAINER_STOP,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("ContainerWaypoint associated with this waypoint"));

        // fill common waypoint (stop) attributes
        fillCommonStopAttributes(myTagProperties[currentTag], true);
    }
    currentTag = GNE_TAG_WAYPOINT_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_STOPS),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::STOP_VEHICLE | GNETagProperties::Type::WAYPOINT_VEHICLE,
                GNETagProperties::Property::XMLCHILD,
                GNETagProperties::Over::CHARGINGSTATION,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::WAYPOINT, GUIGlObjectType::GLO_STOP, SUMO_TAG_STOP, TL("WaypointChargingStation"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_CHARGING_STATION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("ChargingStation associated with this waypoint"));

        // fill common waypoint (stop) attributes
        fillCommonStopAttributes(myTagProperties[currentTag], true);
    }
    currentTag = GNE_TAG_WAYPOINT_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_STOPS),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::STOP_VEHICLE | GNETagProperties::Type::WAYPOINT_VEHICLE,
                GNETagProperties::Property::XMLCHILD,
                GNETagProperties::Over::PARKINGAREA,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::WAYPOINT, GUIGlObjectType::GLO_STOP, SUMO_TAG_STOP, TL("WaypointParkingArea"),
        {SUMO_TAG_ROUTE, SUMO_TAG_TRIP, SUMO_TAG_FLOW}, FXRGBA(240, 255, 205, 255));
        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_PARKING_AREA,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("ParkingArea associated with this waypoint"));

        // fill common waypoint (stop) attributes
        fillCommonStopAttributes(myTagProperties[currentTag], true);
    }
}


void
GNETagPropertiesDatabase::fillPersonElements() {
    // fill vehicle ACs
    SumoXMLTag currentTag = SUMO_TAG_PERSON;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_SUPERMODE_DEMAND),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::PERSON,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::DEMAND | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::PERSON, GUIGlObjectType::GLO_PERSON, currentTag, TL("Person"));

        // add flow attributes
        fillCommonPersonAttributes(myTagProperties[currentTag]);

        fillDepartAttribute(myTagProperties[currentTag]);

    }
    currentTag = SUMO_TAG_PERSONFLOW;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_SUPERMODE_DEMAND),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::PERSON | GNETagProperties::Type::FLOW,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::DEMAND | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::PERSONFLOW, GUIGlObjectType::GLO_PERSONFLOW, currentTag, TL("PersonFlow"));

        // add flow attributes
        fillCommonPersonAttributes(myTagProperties[currentTag]);

        // add flow attributes
        fillCommonFlowAttributes(myTagProperties[currentTag], SUMO_ATTR_PERSONSPERHOUR);
    }
}


void
GNETagPropertiesDatabase::fillContainerElements() {
    // fill vehicle ACs
    SumoXMLTag currentTag = SUMO_TAG_CONTAINER;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_SUPERMODE_DEMAND),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::CONTAINER,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::DEMAND | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::CONTAINER, GUIGlObjectType::GLO_CONTAINER, currentTag, TL("Container"));

        // add flow attributes
        fillCommonContainerAttributes(myTagProperties[currentTag]);

        fillDepartAttribute(myTagProperties[currentTag]);
    }
    currentTag = SUMO_TAG_CONTAINERFLOW;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_SUPERMODE_DEMAND),
                GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::CONTAINER | GNETagProperties::Type::FLOW,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::DEMAND | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::CONTAINERFLOW, GUIGlObjectType::GLO_CONTAINERFLOW, currentTag, TL("ContainerFlow"));

        // add common container attribute
        fillCommonContainerAttributes(myTagProperties[currentTag]);

        // add flow attributes
        fillCommonFlowAttributes(myTagProperties[currentTag], SUMO_ATTR_CONTAINERSPERHOUR);
    }
}


void
GNETagPropertiesDatabase::fillContainerTransportElements() {
    // declare common tag types and properties
    const auto tagType = GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::CONTAINERPLAN | GNETagProperties::Type::TRANSPORT;
    const auto tagProperty = GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS;
    const auto files = FileBucket::Type::NOTHING;
    const auto conflicts = GNETagProperties::Conflicts::NO_CONFLICTS;
    const std::vector<SumoXMLTag> parents({SUMO_TAG_CONTAINER, SUMO_TAG_CONTAINERFLOW});
    const unsigned int color = FXRGBA(240, 255, 205, 255);
    const GUIIcon icon = GUIIcon::TRANSPORT_EDGE;
    const GUIGlObjectType GLType = GUIGlObjectType::GLO_TRANSPORT;
    const SumoXMLTag xmlTag = SUMO_TAG_TRANSPORT;
    // from edge
    SumoXMLTag currentTag = GNE_TAG_TRANSPORT_EDGE_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("edge"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_EDGE_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("edge"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_EDGE_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("edge"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_EDGE_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("edge"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_EDGE_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("edge"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_EDGE_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("edge"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    // from busStop
    currentTag = GNE_TAG_TRANSPORT_BUSSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("busStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_BUSSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("busStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_BUSSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("busStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_BUSSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("busStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_BUSSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("busStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_BUSSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("busStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    // from trainStop
    currentTag = GNE_TAG_TRANSPORT_TRAINSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("trainStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TRAINSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("trainStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TRAINSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("trainStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TRAINSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("trainStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TRAINSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("trainStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_TRAINSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("trainStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    // from containerStop
    currentTag = GNE_TAG_TRANSPORT_CONTAINERSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("containerStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CONTAINERSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("containerStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CONTAINERSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("containerStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CONTAINERSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("containerStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CONTAINERSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("containerStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CONTAINERSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("containerStop"), TL("parkingArea")), parents, color);

        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    // from chargingStation
    currentTag = GNE_TAG_TRANSPORT_CHARGINGSTATION_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("chargingStation"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CHARGINGSTATION_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("chargingStation"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CHARGINGSTATION_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("chargingStation"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CHARGINGSTATION_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("chargingStation"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CHARGINGSTATION_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("chargingStation"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_CHARGINGSTATION_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("chargingStation"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    // from parkingArea
    currentTag = GNE_TAG_TRANSPORT_PARKINGAREA_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("parkingArea"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_PARKINGAREA_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("parkingArea"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_PARKINGAREA_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("parkingArea"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_PARKINGAREA_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("parkingArea"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_PARKINGAREA_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("parkingArea"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSPORT_PARKINGAREA_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSPORTS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Transport"), TL("parkingArea"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTransportCommonAttributes(myTagProperties[currentTag]);
    }
}


void
GNETagPropertiesDatabase::fillContainerTranshipElements() {
    // declare common tag types and properties
    const auto tagType = GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::CONTAINERPLAN | GNETagProperties::Type::TRANSHIP;
    const auto tagProperty = GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS;
    const auto files = FileBucket::Type::NOTHING;
    const auto conflicts = GNETagProperties::Conflicts::NO_CONFLICTS;
    const std::vector<SumoXMLTag> parents({SUMO_TAG_CONTAINER, SUMO_TAG_CONTAINERFLOW});
    const unsigned int color = FXRGBA(210, 233, 255, 255);
    const GUIIcon icon = GUIIcon::TRANSHIP_EDGES;
    const GUIGlObjectType GLType = GUIGlObjectType::GLO_TRANSHIP;
    const SumoXMLTag xmlTag = SUMO_TAG_TRANSHIP;
    // fill tags
    SumoXMLTag currentTag = GNE_TAG_TRANSHIP_EDGES;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::CONSECUTIVE_EDGES,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %", TL("Tranship"), TL("edges")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    // from edge
    currentTag = GNE_TAG_TRANSHIP_EDGE_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("edge"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_EDGE_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("edge"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_EDGE_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("edge"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_EDGE_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("edge"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_EDGE_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("edge"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_EDGE_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("edge"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    // from busStop
    currentTag = GNE_TAG_TRANSHIP_BUSSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("busStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_BUSSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("busStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_BUSSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("busStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_BUSSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("busStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_BUSSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("busStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_BUSSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("busStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    // from trainStop
    currentTag = GNE_TAG_TRANSHIP_TRAINSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("trainStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TRAINSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("trainStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TRAINSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("trainStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TRAINSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("trainStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TRAINSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("trainStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_TRAINSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("trainStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    // from containerStop
    currentTag = GNE_TAG_TRANSHIP_CONTAINERSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("containerStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CONTAINERSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("containerStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CONTAINERSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("containerStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CONTAINERSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("containerStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CONTAINERSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("containerStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CONTAINERSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("containerStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    // from chargingStation
    currentTag = GNE_TAG_TRANSHIP_CHARGINGSTATION_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("chargingStation"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CHARGINGSTATION_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("chargingStation"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CHARGINGSTATION_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("chargingStation"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CHARGINGSTATION_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("chargingStation"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CHARGINGSTATION_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("chargingStation"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_CHARGINGSTATION_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("chargingStation"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    // from parkingArea
    currentTag = GNE_TAG_TRANSHIP_PARKINGAREA_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("parkingArea"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_PARKINGAREA_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("parkingArea"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_PARKINGAREA_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("parkingArea"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_PARKINGAREA_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("parkingArea"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_PARKINGAREA_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("parkingArea"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_TRANSHIP_PARKINGAREA_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_TRANSHIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Tranship"), TL("parkingArea"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillTranshipCommonAttributes(myTagProperties[currentTag]);
    }
}


void
GNETagPropertiesDatabase::fillContainerStopElements() {
    // declare common tag types and properties
    const auto tagType = GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::CONTAINERPLAN | GNETagProperties::Type::STOP_CONTAINER;
    const auto tagProperty = GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS;
    const auto conflicts = GNETagProperties::Conflicts::NO_CONFLICTS;
    const auto files = FileBucket::Type::NOTHING;
    const std::vector<SumoXMLTag> parents({SUMO_TAG_CONTAINER, SUMO_TAG_CONTAINERFLOW});
    const unsigned int color = FXRGBA(255, 213, 213, 255);
    const GUIIcon icon = GUIIcon::STOPELEMENT;
    const GUIGlObjectType GLType = GUIGlObjectType::GLO_STOP_PLAN;
    const SumoXMLTag xmlTag = SUMO_TAG_STOP;
    // fill tags
    SumoXMLTag currentTag = GNE_TAG_STOPCONTAINER_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_CONTAINERSTOPS), tagType, tagProperty,
                GNETagProperties::Over::EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %", TL("ContainerStop"), TL("edge")), parents, color);

        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPCONTAINER_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_CONTAINERSTOPS), tagType, tagProperty,
                GNETagProperties::Over::BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %", TL("ContainerStop"), TL("busStop")), parents, color);

        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPCONTAINER_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_CONTAINERSTOPS), tagType, tagProperty,
                GNETagProperties::Over::TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %", TL("ContainerStop"), TL("trainStop")), parents, color);

        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPCONTAINER_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_CONTAINERSTOPS), tagType, tagProperty,
                GNETagProperties::Over::CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %", TL("ContainerStop"), TL("containerStop")), parents, color);

        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPCONTAINER_CHARGINGSTATION;
    {
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_CONTAINERSTOPS), tagType, tagProperty,
                GNETagProperties::Over::CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %", TL("ContainerStop"), TL("chargingStation")), parents, color);

        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPCONTAINER_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_CONTAINERSTOPS), tagType, tagProperty,
                GNETagProperties::Over::PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %", TL("ContainerStop"), TL("parkingArea")), parents, color);

        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
}


void
GNETagPropertiesDatabase::fillPersonPlanTrips() {
    // declare common tag types and properties
    const auto tagType = GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::PERSONPLAN | GNETagProperties::Type::PERSONTRIP;
    const auto tagProperty = GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS;
    const auto tagPropertyTAZ = GNETagProperties::Property::RTREE | tagProperty;
    const auto files = FileBucket::Type::NOTHING;
    const auto conflicts = GNETagProperties::Conflicts::NO_CONFLICTS;
    const std::vector<SumoXMLTag> parents({SUMO_TAG_PERSON, SUMO_TAG_PERSONFLOW});
    const unsigned int color = FXRGBA(253, 255, 206, 255);
    const GUIIcon icon = GUIIcon::PERSONTRIP_EDGE;
    const GUIGlObjectType GLType = GUIGlObjectType::GLO_PERSONTRIP;
    const SumoXMLTag xmlTag = SUMO_TAG_PERSONTRIP;
    // from edge
    SumoXMLTag currentTag = GNE_TAG_PERSONTRIP_EDGE_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("edge"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_EDGE_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_TAZ,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("edge"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_EDGE_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_JUNCTION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("edge"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_EDGE_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("edge"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_EDGE_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("edge"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_EDGE_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("edge"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_EDGE_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("edge"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_EDGE_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("edge"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    // from taz
    currentTag = GNE_TAG_PERSONTRIP_TAZ_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("taz"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TAZ_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_TAZ,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("taz"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TAZ_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_JUNCTION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("taz"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TAZ_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("taz"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TAZ_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("taz"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TAZ_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("taz"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TAZ_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("taz"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TAZ_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("taz"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    // from junction
    currentTag = GNE_TAG_PERSONTRIP_JUNCTION_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_JUNCTION | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("junction"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_JUNCTION_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_JUNCTION | GNETagProperties::Over::TO_TAZ,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("junction"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_JUNCTION_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_JUNCTION | GNETagProperties::Over::TO_JUNCTION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("junction"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_JUNCTION_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_JUNCTION | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("junction"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_JUNCTION_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_JUNCTION | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("junction"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_JUNCTION_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_JUNCTION | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("junction"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_JUNCTION_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_JUNCTION | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("junction"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_JUNCTION_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_JUNCTION | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("junction"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    // from busStop
    currentTag = GNE_TAG_PERSONTRIP_BUSSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("busStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_BUSSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_TAZ,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("busStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_BUSSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_JUNCTION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("busStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_BUSSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("busStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_BUSSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("busStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_BUSSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("busStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_BUSSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("busStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_BUSSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("busStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    // from trainStop
    currentTag = GNE_TAG_PERSONTRIP_TRAINSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("trainStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TRAINSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_TAZ,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("trainStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TRAINSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_JUNCTION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("trainStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TRAINSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("trainStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TRAINSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("trainStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TRAINSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("trainStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TRAINSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("trainStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_TRAINSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("trainStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    // from containerStop
    currentTag = GNE_TAG_PERSONTRIP_CONTAINERSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("containerStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CONTAINERSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_TAZ,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("containerStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CONTAINERSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_JUNCTION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("containerStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CONTAINERSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("containerStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CONTAINERSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("containerStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CONTAINERSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("containerStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CONTAINERSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("containerStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CONTAINERSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("containerStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    // from chargingStation
    currentTag = GNE_TAG_PERSONTRIP_CHARGINGSTATION_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("chargingStation"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CHARGINGSTATION_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_TAZ,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("chargingStation"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CHARGINGSTATION_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_JUNCTION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("chargingStation"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CHARGINGSTATION_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("chargingStation"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CHARGINGSTATION_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("chargingStation"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CHARGINGSTATION_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("chargingStation"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CHARGINGSTATION_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("chargingStation"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_CHARGINGSTATION_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("chargingStation"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    // from parkingArea
    currentTag = GNE_TAG_PERSONTRIP_PARKINGAREA_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("parkingArea"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_PARKINGAREA_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_TAZ,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("parkingArea"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_PARKINGAREA_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_JUNCTION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("parkingArea"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_PARKINGAREA_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("parkingArea"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_PARKINGAREA_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("parkingArea"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_PARKINGAREA_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("parkingArea"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_PARKINGAREA_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("parkingArea"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_PERSONTRIP_PARKINGAREA_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONTRIPS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("PersonTrip"), TL("parkingArea"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPersonTripCommonAttributes(myTagProperties[currentTag]);
    }
}


void
GNETagPropertiesDatabase::fillPersonPlanWalks() {
    // declare common tag types and properties
    const auto tagType = GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::PERSONPLAN | GNETagProperties::Type::WALK;
    const auto tagProperty = GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS;
    const auto tagPropertyTAZ = GNETagProperties::Property::RTREE | tagProperty;
    const auto files = FileBucket::Type::NOTHING;
    const auto conflicts = GNETagProperties::Conflicts::NO_CONFLICTS;
    const std::vector<SumoXMLTag> parents({SUMO_TAG_PERSON, SUMO_TAG_PERSONFLOW});
    const unsigned int color = FXRGBA(240, 255, 205, 255);
    const GUIIcon icon = GUIIcon::WALK_EDGES;
    const GUIGlObjectType GLType = GUIGlObjectType::GLO_WALK;
    const SumoXMLTag xmlTag = SUMO_TAG_WALK;
    // fill tags
    SumoXMLTag currentTag = GNE_TAG_WALK_EDGES;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::CONSECUTIVE_EDGES,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %", TL("Walk"), TL("edges")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_ROUTE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::ROUTE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %", TL("Walk"), TL("route")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    // from edge
    currentTag = GNE_TAG_WALK_EDGE_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("edge"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_EDGE_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_TAZ,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("edge"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_EDGE_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_JUNCTION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("edge"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_EDGE_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("edge"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_EDGE_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("edge"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_EDGE_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("edge"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_EDGE_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("edge"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_EDGE_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("edge"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    // from taz
    currentTag = GNE_TAG_WALK_TAZ_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("taz"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TAZ_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_TAZ,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("taz"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TAZ_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_JUNCTION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("taz"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TAZ_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("taz"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TAZ_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("taz"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TAZ_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("taz"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TAZ_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("taz"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TAZ_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("taz"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    // from junction
    currentTag = GNE_TAG_WALK_JUNCTION_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_JUNCTION | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("junction"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_JUNCTION_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_JUNCTION | GNETagProperties::Over::TO_TAZ,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("junction"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_JUNCTION_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_JUNCTION | GNETagProperties::Over::TO_JUNCTION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("junction"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_JUNCTION_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_JUNCTION | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("junction"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_JUNCTION_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_JUNCTION | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("junction"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_JUNCTION_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_JUNCTION | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("junction"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_JUNCTION_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_JUNCTION | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("junction"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_JUNCTION_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_JUNCTION | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("junction"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    // from busStop
    currentTag = GNE_TAG_WALK_BUSSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("busStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_BUSSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_TAZ,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("busStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_BUSSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_JUNCTION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("busStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_BUSSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("busStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_BUSSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("busStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_BUSSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("busStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_BUSSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("busStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_BUSSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("busStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    // from trainStop
    currentTag = GNE_TAG_WALK_TRAINSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("trainStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TRAINSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_TAZ,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("trainStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TRAINSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_JUNCTION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("trainStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TRAINSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("trainStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TRAINSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("trainStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TRAINSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("trainStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TRAINSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("trainStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_TRAINSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("trainStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    // from containerStop
    currentTag = GNE_TAG_WALK_CONTAINERSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("containerStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CONTAINERSTOP_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_TAZ,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("containerStop"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CONTAINERSTOP_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_JUNCTION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("containerStop"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CONTAINERSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("containerStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CONTAINERSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("containerStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CONTAINERSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("containerStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CONTAINERSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("containerStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CONTAINERSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("containerStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    // from chargingStation
    currentTag = GNE_TAG_WALK_CHARGINGSTATION_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("chargingStation"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CHARGINGSTATION_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_TAZ,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("chargingStation"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CHARGINGSTATION_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_JUNCTION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("chargingStation"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CHARGINGSTATION_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("chargingStation"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CHARGINGSTATION_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("chargingStation"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CHARGINGSTATION_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("chargingStation"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CHARGINGSTATION_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("chargingStation"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_CHARGINGSTATION_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("chargingStation"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    // from parkingArea
    currentTag = GNE_TAG_WALK_PARKINGAREA_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("parkingArea"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_PARKINGAREA_TAZ;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagPropertyTAZ,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_TAZ,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("parkingArea"), TL("taz")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_PARKINGAREA_JUNCTION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_JUNCTION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("parkingArea"), TL("junction")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_PARKINGAREA_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("parkingArea"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_PARKINGAREA_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("parkingArea"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_PARKINGAREA_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("parkingArea"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_PARKINGAREA_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("parkingArea"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_WALK_PARKINGAREA_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_WALKS), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Walk"), TL("parkingArea"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillWalkCommonAttributes(myTagProperties[currentTag]);
    }
}


void
GNETagPropertiesDatabase::fillPersonPlanRides() {
    // declare common tag types and properties
    const auto tagType = GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::PERSONPLAN | GNETagProperties::Type::RIDE;
    const auto tagProperty = GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS;
    const auto files = FileBucket::Type::NOTHING;
    const auto conflicts = GNETagProperties::Conflicts::NO_CONFLICTS;
    const std::vector<SumoXMLTag> parents({SUMO_TAG_PERSON, SUMO_TAG_PERSONFLOW});
    const unsigned int color = FXRGBA(253, 255, 206, 255);
    const GUIIcon icon = GUIIcon::RIDE_EDGE;
    const GUIGlObjectType GLType = GUIGlObjectType::GLO_RIDE;
    const SumoXMLTag xmlTag = SUMO_TAG_RIDE;
    // from edge
    SumoXMLTag currentTag = GNE_TAG_RIDE_EDGE_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("edge"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_EDGE_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("edge"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_EDGE_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("edge"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_EDGE_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("edge"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_EDGE_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("edge"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_EDGE_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("edge"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    // from busStop
    currentTag = GNE_TAG_RIDE_BUSSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("busStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_BUSSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("busStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_BUSSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("busStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_BUSSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("busStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_BUSSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("busStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_BUSSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_BUSSTOP | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("busStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    // from trainStop
    currentTag = GNE_TAG_RIDE_TRAINSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("trainStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TRAINSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("trainStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TRAINSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("trainStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TRAINSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("trainStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TRAINSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("trainStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_TRAINSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_TRAINSTOP | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("trainStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    // from containerStop
    currentTag = GNE_TAG_RIDE_CONTAINERSTOP_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("containerStop"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CONTAINERSTOP_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("containerStop"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CONTAINERSTOP_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("containerStop"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CONTAINERSTOP_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("containerStop"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CONTAINERSTOP_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("containerStop"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CONTAINERSTOP_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_CONTAINERSTOP | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("containerStop"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    // from chargingStation
    currentTag = GNE_TAG_RIDE_CHARGINGSTATION_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("chargingStation"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CHARGINGSTATION_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("chargingStation"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CHARGINGSTATION_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("chargingStation"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CHARGINGSTATION_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("chargingStation"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CHARGINGSTATION_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("chargingStation"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_CHARGINGSTATION_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_CHARGINGSTATION | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("chargingStation"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    // from parkingArea
    currentTag = GNE_TAG_RIDE_PARKINGAREA_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("parkingArea"), TL("edge")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_PARKINGAREA_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("parkingArea"), TL("busStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_PARKINGAREA_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("parkingArea"), TL("trainStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_PARKINGAREA_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("parkingArea"), TL("containerStop")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_PARKINGAREA_CHARGINGSTATION;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("parkingArea"), TL("chargingStation")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_RIDE_PARKINGAREA_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_RIDES), tagType, tagProperty,
                GNETagProperties::Over::FROM_PARKINGAREA | GNETagProperties::Over::TO_PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %->%", TL("Ride"), TL("parkingArea"), TL("parkingArea")), parents, color);
        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillRideCommonAttributes(myTagProperties[currentTag]);
    }
}


void
GNETagPropertiesDatabase::fillPersonStopElements() {
    // declare common tag types and properties
    const auto tagType = GNETagProperties::Type::DEMANDELEMENT | GNETagProperties::Type::PERSONPLAN | GNETagProperties::Type::STOP_PERSON;
    const auto tagProperty = GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOPARAMETERS;
    const auto files = FileBucket::Type::NOTHING;
    const auto conflicts = GNETagProperties::Conflicts::NO_CONFLICTS;
    const std::vector<SumoXMLTag> parents({SUMO_TAG_PERSON, SUMO_TAG_PERSONFLOW});
    const unsigned int color = FXRGBA(255, 213, 213, 255);
    const GUIIcon icon = GUIIcon::STOPELEMENT;
    const GUIGlObjectType GLType = GUIGlObjectType::GLO_STOP_PLAN;
    const SumoXMLTag xmlTag = SUMO_TAG_STOP;
    // fill tags
    SumoXMLTag currentTag = GNE_TAG_STOPPERSON_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONSTOPS), tagType, tagProperty,
                GNETagProperties::Over::EDGE,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %", TL("PersonStop"), TL("edge")), parents, color);

        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPPERSON_BUSSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONSTOPS), tagType, tagProperty,
                GNETagProperties::Over::BUSSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %", TL("PersonStop"), TL("busStop")), parents, color);

        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPPERSON_TRAINSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONSTOPS), tagType, tagProperty,
                GNETagProperties::Over::TRAINSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %", TL("PersonStop"), TL("trainStop")), parents, color);

        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPPERSON_CONTAINERSTOP;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONSTOPS), tagType, tagProperty,
                GNETagProperties::Over::CONTAINERSTOP,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %", TL("PersonStop"), TL("containerStop")), parents, color);

        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPPERSON_CHARGINGSTATION;
    {
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONSTOPS), tagType, tagProperty,
                GNETagProperties::Over::CHARGINGSTATION,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %", TL("PersonStop"), TL("chargingStation")), parents, color);

        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
    currentTag = GNE_TAG_STOPPERSON_PARKINGAREA;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_PERSONSTOPS), tagType, tagProperty,
                GNETagProperties::Over::PARKINGAREA,
                files, conflicts, icon, GLType, xmlTag, StringUtils::format("%: %", TL("PersonStop"), TL("parkingArea")), parents, color);

        // set values of attributes
        fillPlanParentAttributes(myTagProperties[currentTag]);
        fillPlanStopCommonAttributes(myTagProperties[currentTag]);
    }
}


void
GNETagPropertiesDatabase::fillCommonAttributes(GNETagProperties* tagProperties) {
    GNEAttributeProperties* commonAttribute = nullptr;
    // check if element can be reparent
    if (tagProperties->canCenterCameraAfterCreation()) {
        commonAttribute = new GNEAttributeProperties(tagProperties, GNE_ATTR_CENTER_AFTER_CREATION,
                GNEAttributeProperties::Property::BOOL,
                GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::NETEDITEDITOR,
                TLF("Center view over element % after creation", tagProperties->getTagStr()));
        commonAttribute->setAlternativeName(TL("center view"));
    }
    // fill save file attributes
    if (tagProperties->saveInNetworkFile()) {
        commonAttribute = new GNEAttributeProperties(tagProperties, GNE_ATTR_SAVEFILE,
                GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::FILESAVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                GNEAttributeProperties::Edit::NETEDITEDITOR,
                TL("The path to the file to save this element (not editable for network elements)"));
        commonAttribute->setFilenameExtensions(SUMOXMLDefinitions::AdditionalFileExtensions.getStrings());
        commonAttribute->setAlternativeName(TL("File"));
    } else if (tagProperties->saveInParentFile()) {
        commonAttribute = new GNEAttributeProperties(tagProperties, GNE_ATTR_SAVEFILE,
                GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::FILESAVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                GNEAttributeProperties::Edit::NETEDITEDITOR,
                TL("The path to the file to save this element (the same of their parent)"));
        commonAttribute->setFilenameExtensions(SUMOXMLDefinitions::AdditionalFileExtensions.getStrings());
        commonAttribute->setAlternativeName(TL("File"));
    } else {
        commonAttribute = new GNEAttributeProperties(tagProperties, GNE_ATTR_SAVEFILE,
                GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::FILESAVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::NETEDITEDITOR,
                TL("The path to the file to save this element"));
        commonAttribute->setFilenameExtensions(SUMOXMLDefinitions::AdditionalFileExtensions.getStrings());
        commonAttribute->setAlternativeName(TL("File"));
    }

    // if this is a drawable element, add front and select attributes
    if (tagProperties->isDrawable()) {
        commonAttribute = new GNEAttributeProperties(tagProperties, GNE_ATTR_FRONTELEMENT,
                GNEAttributeProperties::Property::BOOL,
                GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::NETEDITEDITOR,
                TL("Toggle front element"));

        commonAttribute = new GNEAttributeProperties(tagProperties, GNE_ATTR_SELECTED,
                GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                GNEAttributeProperties::Edit::NETEDITEDITOR,
                TL("Toggle select element"),
                GNEAttributeCarrier::FALSE_STR);
    }
    // check if element can be reparent
    if (tagProperties->canBeReparent()) {
        commonAttribute = new GNEAttributeProperties(tagProperties, GNE_ATTR_PARENT,
                GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::NETEDITEDITOR,
                TL("Change element parent"));
    }
    // check if element has parameters
    if (tagProperties->hasParameters()) {
        commonAttribute = new GNEAttributeProperties(tagProperties, GNE_ATTR_PARAMETERS,
                GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                TL("Generic parameters (Format: key1=value1|key2=value2|..."));
        commonAttribute->setAlternativeName(TL("parameters"));
    }
}


void
GNETagPropertiesDatabase::fillCommonStoppingPlaceAttributes(GNETagProperties* tagProperties, const bool includeColor, const bool parkingAreaAngle) {
    // set values of attributes
    fillIDAttribute(tagProperties, true);

    fillLaneAttribute(tagProperties, false);

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_STARTPOS,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                               GNEAttributeProperties::Edit::EDITMODE,
                               TL("The begin position on the lane (the lower position on the lane) in meters"),
                               GNEMoveElementLaneSingle::PositionType::STARPOS, "INVALID_DOUBLE");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_ENDPOS,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                               GNEAttributeProperties::Edit::EDITMODE,
                               TL("The end position on the lane (the higher position on the lane) in meters, must be larger than startPos by more than 0.1m"),
                               GNEMoveElementLaneSingle::PositionType::ENDPOS, "INVALID_DOUBLE");

    fillFriendlyPosAttribute(tagProperties);

    fillNameAttribute(tagProperties);

    if (includeColor) {
        fillColorAttribute(tagProperties, "");
    }

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_ANGLE,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::ANGLE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               parkingAreaAngle ? TL("The angle of the road-side parking spaces relative to the lane angle, positive means clockwise") :
                               TLF("Angle of waiting %s relative to lane angle", tagProperties->getTag() == SUMO_TAG_CONTAINER_STOP ? toString(SUMO_TAG_CONTAINER) : toString(SUMO_TAG_PERSON)),
                               "0");

    // netedit attributes
    new GNEAttributeProperties(tagProperties, GNE_ATTR_SIZE,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::NETEDITEDITOR,
                               TLF("Length of %", tagProperties->getTagStr()),
                               toString(GNEMoveElementLaneDouble::defaultSize));

    auto forceSize = new GNEAttributeProperties(tagProperties, GNE_ATTR_FORCESIZE,
            GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::NETEDITEDITOR,
            TL("Force size during creation"),
            GNEAttributeCarrier::FALSE_STR);
    forceSize->setAlternativeName(TL("force size"));

    auto reference = new GNEAttributeProperties(tagProperties, GNE_ATTR_REFERENCE,
            GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::NETEDITEDITOR,
            TLF("Longitudinal reference position for creating %", tagProperties->getTagStr()));
    reference->setDiscreteValues(SUMOXMLDefinitions::ReferencePositions.getStrings());
}


void
GNETagPropertiesDatabase::fillCommonPOIAttributes(GNETagProperties* tagProperties) {
    // fill POI attributes
    fillNameAttribute(tagProperties);

    fillColorAttribute(tagProperties, "red");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_TYPE,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("A typename for the POI"),
                               toString(Shape::DEFAULT_TYPE));

    auto icon = new GNEAttributeProperties(tagProperties, SUMO_ATTR_ICON,
                                           GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                           GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                           TL("POI Icon"),
                                           SUMOXMLDefinitions::POIIcons.getString(POIIcon::NONE));
    icon->setDiscreteValues(SUMOXMLDefinitions::POIIcons.getStrings());

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LAYER,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The layer of the POI for drawing and selecting"),
                               toString(Shape::DEFAULT_LAYER_POI));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_WIDTH,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("Width of rendered image in meters"),
                               toString(Shape::DEFAULT_IMG_WIDTH));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_HEIGHT,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("Height of rendered image in meters"),
                               toString(Shape::DEFAULT_IMG_HEIGHT));

    fillImgFileAttribute(tagProperties, false);

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_ANGLE,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::ANGLE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("Angle of rendered image in degree"),
                               toString(Shape::DEFAULT_ANGLE));
}


void
GNETagPropertiesDatabase::fillCommonRouteAttributes(GNETagProperties* tagProperties) {
    // fill route attributes
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_EDGES,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                               GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                               TL("The edges the vehicle shall drive along, given as their ids, separated using spaces"));

    fillColorAttribute(tagProperties, "");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_REPEAT,
                               GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The number of times that the edges of this route shall be repeated"),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CYCLETIME,
                               GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("When defining a repeating route with stops and those stops use the until attribute,") + std::string("\n") +
                               TL("the times will be shifted forward by 'cycleTime' on each repeat"),
                               "0");
}


void
GNETagPropertiesDatabase::fillCommonVTypeAttributes(GNETagProperties* tagProperties) {
    // fill vType attributes
    auto vClass = new GNEAttributeProperties(tagProperties, SUMO_ATTR_VCLASS,
            GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
            TL("An abstract vehicle class"),
            "passenger");
    vClass->setDiscreteValues(SumoVehicleClassStrings.getStrings());

    fillColorAttribute(tagProperties, "");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LENGTH,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::ALWAYSENABLED,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The vehicle's netto-length (length) [m]"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_MINGAP,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::ALWAYSENABLED,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("Empty space after leader [m]"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_MAXSPEED,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::ALWAYSENABLED,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The vehicle's maximum velocity [m/s]"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_SPEEDFACTOR,
                               GNEAttributeProperties::Property::STRING,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The vehicle's expected multiplicator for lane speed limits (or a distribution specifier)"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_DESIRED_MAXSPEED,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::ALWAYSENABLED,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The vehicle's desired maximum velocity (interacts with speedFactor).") + std::string("\n") +
                               TL("Applicable when no speed limit applies (bicycles, some motorways) [m/s]"));

    auto emissionClass = new GNEAttributeProperties(tagProperties, SUMO_ATTR_EMISSIONCLASS,
            GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
            TL("An abstract emission class"));
    emissionClass->setDiscreteValues(PollutantsInterface::getAllClassesStr());

    auto GUIShape = new GNEAttributeProperties(tagProperties, SUMO_ATTR_GUISHAPE,
            GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
            TL("How this vehicle is rendered"));
    GUIShape->setDiscreteValues(SumoVehicleShapeStrings.getStrings());

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_WIDTH,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The vehicle's width [m] (only used for drawing)"),
                               "1.8");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_HEIGHT,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The vehicle's height [m] (only used for drawing)"),
                               "1.5");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_PARKING_BADGES,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The parking badges assigned to the vehicle"));

    fillImgFileAttribute(tagProperties, true);

    auto laneChangeModel = new GNEAttributeProperties(tagProperties, SUMO_ATTR_LANE_CHANGE_MODEL,
            GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
            TL("The model used for changing lanes"),
            SUMOXMLDefinitions::LaneChangeModels.getString(LaneChangeModel::DEFAULT));
    laneChangeModel->setDiscreteValues(SUMOXMLDefinitions::LaneChangeModels.getStrings());

    auto carFollowModel = new GNEAttributeProperties(tagProperties, SUMO_ATTR_CAR_FOLLOW_MODEL,
            GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
            TL("The model used for car-following"),
            SUMOXMLDefinitions::CarFollowModels.getString(SUMO_TAG_CF_KRAUSS));
    carFollowModel->setDiscreteValues(SUMOXMLDefinitions::CarFollowModels.getStrings());

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_PERSON_CAPACITY,
                               GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::POSITIVE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The number of persons (excluding an autonomous driver) the vehicle can transport"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CONTAINER_CAPACITY,
                               GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::POSITIVE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The number of containers the vehicle can transport"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_BOARDING_DURATION,
                               GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The time required by a person to board the vehicle"),
                               "0.50");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LOADING_DURATION,
                               GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The time required to load a container onto the vehicle"),
                               "90");

    auto latAlignment = new GNEAttributeProperties(tagProperties, SUMO_ATTR_LATALIGNMENT,
            GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
            TL("The preferred lateral alignment when using the sublane-model"),
            "center");
    latAlignment->setDiscreteValues(SUMOVTypeParameter::getLatAlignmentStrings());

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_MINGAP_LAT,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The minimum lateral gap at a speed difference of 50km/h when using the sublane-model"),
                               "0.12");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_MAXSPEED_LAT,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The maximum lateral speed when using the sublane-model"),
                               "1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_ACTIONSTEPLENGTH,
                               GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The interval length for which vehicle performs its decision logic (acceleration and lane-changing)"),
                               toString(OptionsCont::getOptions().getFloat("default.action-step-length")));

    // add distribution probability
    fillDistributionProbability(tagProperties, true);

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_OSGFILE,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("3D model file for this class"));
    /*
            Waiting for #16343
            new GNEAttributeProperties(tagProperties, SUMO_ATTR_CARRIAGE_LENGTH,
                                        GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE,
                                        GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                                        TL("Carriage lengths"));

            new GNEAttributeProperties(tagProperties, SUMO_ATTR_LOCOMOTIVE_LENGTH,
                                        GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE,
                                        GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                                        TL("Locomotive lengths"));

            new GNEAttributeProperties(tagProperties, SUMO_ATTR_CARRIAGE_GAP,
                                        GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                        GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                                        TL("Gap between carriages"),
                                        "1");
    */
    // fill VType Car Following Model Values (implemented in a separated function to improve code legibility)
    fillCarFollowingModelAttributes(tagProperties);

    // fill VType Junction Model Parameters (implemented in a separated function to improve code legibility)
    fillJunctionModelAttributes(tagProperties);

    // fill VType Lane Change Model Parameters (implemented in a separated function to improve code legibility)
    fillLaneChangingModelAttributes(tagProperties);
}


void
GNETagPropertiesDatabase::fillCommonVehicleAttributes(GNETagProperties* tagProperties) {
    // fill vehicle attributes
    fillColorAttribute(tagProperties, "yellow");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_DEPARTLANE,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE |  GNEAttributeProperties::Property::UPDATEGEOMETRY,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The lane on which the vehicle shall be inserted"),
                               "first");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_DEPARTPOS,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE |  GNEAttributeProperties::Property::UPDATEGEOMETRY,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The position at which the vehicle shall enter the net"),
                               "base");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_DEPARTSPEED,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The speed with which the vehicle shall enter the network"),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_ARRIVALLANE,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE |  GNEAttributeProperties::Property::UPDATEGEOMETRY,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The lane at which the vehicle shall leave the network"),
                               "current");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_ARRIVALPOS,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE |  GNEAttributeProperties::Property::UPDATEGEOMETRY,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The position at which the vehicle shall leave the network"),
                               "max");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_ARRIVALSPEED,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The speed with which the vehicle shall leave the network"),
                               "current");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LINE,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("A string specifying the id of a public transport line which can be used when specifying person rides"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_PERSON_NUMBER,
                               GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The number of occupied seats when the vehicle is inserted"),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CONTAINER_NUMBER,
                               GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The number of occupied container places when the vehicle is inserted"),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_DEPARTPOS_LAT,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE |  GNEAttributeProperties::Property::UPDATEGEOMETRY,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The lateral position on the departure lane at which the vehicle shall enter the net"),
                               "center");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_ARRIVALPOS_LAT,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE |  GNEAttributeProperties::Property::UPDATEGEOMETRY,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The lateral position on the arrival lane at which the vehicle shall arrive"),
                               "center");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_INSERTIONCHECKS,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("Insertion checks"),
                               SUMOXMLDefinitions::InsertionChecks.getString(InsertionCheck::ALL));
}


void
GNETagPropertiesDatabase::fillCommonFlowAttributes(GNETagProperties* tagProperties, SumoXMLAttr perHour) {
    // fill common flow attributes
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_BEGIN,
                               GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("First flow departure time"),
                               "0");

    auto flowTerminate = new GNEAttributeProperties(tagProperties, GNE_ATTR_FLOW_TERMINATE,
            GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::FLOW,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::FLOWEDITOR,
            TL("Criterion for flow termination"),
            toString(SUMO_ATTR_END));
    flowTerminate->setDiscreteValues({toString(SUMO_ATTR_END), toString(SUMO_ATTR_NUMBER), toString(SUMO_ATTR_END) + "-" + toString(SUMO_ATTR_NUMBER)});

    auto flowSpacing = new GNEAttributeProperties(tagProperties, GNE_ATTR_FLOW_SPACING,
            GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::FLOW,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::FLOWEDITOR,
            TL("Criterion for flow spacing"),
            toString(perHour));
    flowSpacing->setDiscreteValues({toString(perHour), toString(SUMO_ATTR_PERIOD), toString(SUMO_ATTR_PROB), toString(GNE_ATTR_POISSON)});

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_END,
                               GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::FLOW,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::FLOWEDITOR,
                               TL("End of departure interval"),
                               "3600");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_NUMBER,
                               GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::FLOW,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::FLOWEDITOR,
                               TL("probability for emitting a flow each second") + std::string("\n") +
                               TL("(not together with vehsPerHour or period)"),
                               "1800");

    new GNEAttributeProperties(tagProperties, perHour,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::FLOW,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::FLOWEDITOR,
                               TL("Number of flows per hour, equally spaced") + std::string("\n") +
                               TL("(not together with period or probability or poisson)"),
                               "1800");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_PERIOD,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::FLOW,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::FLOWEDITOR,
                               TL("Insert equally spaced flows at that period") + std::string("\n") +
                               TL("(not together with vehsPerHour or probability or poisson)"),
                               "2");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_PROB,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::FLOW,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::FLOWEDITOR,
                               TL("probability for emitting a flow each second") + std::string("\n") +
                               TL("(not together with vehsPerHour or period or poisson)"),
                               "0.5");

    new GNEAttributeProperties(tagProperties, GNE_ATTR_POISSON,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::FLOW,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::FLOWEDITOR,
                               TL("Insert flow expected vehicles per second with poisson distributed insertion rate") + std::string("\n") +
                               TL("(not together with period or vehsPerHour or probability)"),
                               "0.5");
}


void
GNETagPropertiesDatabase::fillCarFollowingModelAttributes(GNETagProperties* tagProperties) {
    // fill CFM attributes
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_ACCEL,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The acceleration ability of vehicles of this type [m/s^2]"),
                               toString(SUMOVTypeParameter::getDefaultAccel()));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_DECEL,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The deceleration ability of vehicles of this type [m/s^2]"),
                               toString(SUMOVTypeParameter::getDefaultDecel()));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_APPARENTDECEL,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The apparent deceleration of the vehicle as used by the standard model [m/s^2]"),
                               toString(SUMOVTypeParameter::getDefaultDecel()));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_EMERGENCYDECEL,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The maximal physically possible deceleration for the vehicle [m/s^2]"),
                               toString(SUMOVTypeParameter::getDefaultEmergencyDecel(SVC_IGNORING,
                                        SUMOVTypeParameter::getDefaultDecel(),
                                        VTYPEPARS_DEFAULT_EMERGENCYDECEL_DEFAULT)));

    auto sigma = new GNEAttributeProperties(tagProperties, SUMO_ATTR_SIGMA,
                                            GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::RANGE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                            TL("Car-following model parameter"),
                                            toString(SUMOVTypeParameter::getDefaultImperfection()));
    sigma->setRange(0, 1);

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_TAU,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("Car-following model parameter"),
                               "1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_TMP1,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("SKRAUSSX parameter 1"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_TMP2,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("SKRAUSSX parameter 2"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_TMP3,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("SKRAUSSX parameter 3"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_TMP4,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("SKRAUSSX parameter 4"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_TMP5,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("SKRAUSSX parameter 5"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_EIDM_T_LOOK_AHEAD,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("EIDM Look ahead / preview parameter [s]"),
                               "4");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_EIDM_T_REACTION,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("EIDM AP Reaction Time parameter [s]"),
                               "0.50");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_EIDM_T_PERSISTENCE_DRIVE,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("EIDM Wiener Process parameter for the Driving Error [s]"),
                               "3");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_EIDM_T_PERSISTENCE_ESTIMATE,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("EIDM Wiener Process parameter for the Estimation Error [s]"),
                               "10");

    auto coolness = new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_EIDM_C_COOLNESS,
            GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::RANGE | GNEAttributeProperties::Property::DEFAULTVALUE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
            TL("EIDM Coolness parameter of the Enhanced IDM [-]"),
            "0.99");
    coolness->setRange(0, 1);

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_EIDM_SIG_LEADER,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("EIDM leader speed estimation error parameter [-]"),
                               "0.02");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_EIDM_SIG_GAP,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("EIDM gap estimation error parameter [-]"),
                               "0.10");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_EIDM_SIG_ERROR,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("EIDM driving error parameter [-]"),
                               "0.04");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_EIDM_JERK_MAX,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("EIDM maximal jerk parameter [m/s^3]"),
                               "3");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_EIDM_EPSILON_ACC,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("EIDM maximal negative acceleration between two Action Points (threshold) [m/s^2]"),
                               "1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_EIDM_T_ACC_MAX,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("EIDM Time parameter until vehicle reaches amax after startup/driveoff [s]"),
                               "1.20");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_EIDM_M_FLATNESS,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("EIDM Flatness parameter of startup/driveoff curve [-]"),
                               "2");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_EIDM_M_BEGIN,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("EIDM Shift parameter of startup/driveoff curve [-]"),
                               "0.70");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_EIDM_USEVEHDYNAMICS,
                               GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("EIDM parameter if model shall include vehicle dynamics into the acceleration calculation [0/1]"),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_EIDM_MAX_VEH_PREVIEW,
                               GNEAttributeProperties::Property::INT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("EIDM parameter how many vehicles are taken into the preview calculation of the driver (at least always 1!) [-]"),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_PWAGNER2009_TAULAST,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Peter Wagner 2009 parameter"),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_PWAGNER2009_APPROB,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Peter Wagner 2009 parameter"),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_IDMM_ADAPT_FACTOR,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("IDMM parameter"),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_IDMM_ADAPT_TIME,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("IDMM parameter"),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_W99_CC1,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("W99 parameter"),
                               "1.3");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_W99_CC2,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("W99 parameter"),
                               "8");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_W99_CC3,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("W99 parameter"),
                               "-12");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_W99_CC4,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("W99 parameter"),
                               "-0.25");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_W99_CC5,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("W99 parameter"),
                               "0.35");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_W99_CC6,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("W99 parameter"),
                               "6");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_W99_CC7,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("W99 parameter"),
                               "0.25");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_W99_CC8,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("W99 parameter"),
                               "2");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_W99_CC9,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("W99 parameter"),
                               "1.5");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_WIEDEMANN_SECURITY,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Wiedemann parameter"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_WIEDEMANN_ESTIMATION,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Wiedemann parameter"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_COLLISION_MINGAP_FACTOR,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("MinGap factor parameter"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_K,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("K parameter"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_KERNER_PHI,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Kerner Phi parameter"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_IDM_DELTA,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("IDM Delta parameter"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_CF_IDM_STEPPING,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("IDM Stepping parameter"));

    auto trainType = new GNEAttributeProperties(tagProperties, SUMO_ATTR_TRAIN_TYPE,
            GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
            TL("Train Types"),
            SUMOXMLDefinitions::TrainTypes.getString(TrainType::NGT400));
    trainType->setDiscreteValues(SUMOXMLDefinitions::TrainTypes.getStrings());
}


void
GNETagPropertiesDatabase::fillJunctionModelAttributes(GNETagProperties* tagProperties) {
    // fill junction model attributes
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_JM_CROSSING_GAP,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Minimum distance to pedestrians that are walking towards the conflict point with the ego vehicle."),
                               "10");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_JM_IGNORE_KEEPCLEAR_TIME,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The accumulated waiting time after which a vehicle will drive onto an intersection even though this might cause jamming."),
                               "", "-1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_JM_DRIVE_AFTER_YELLOW_TIME,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("This value causes vehicles to violate a yellow light if the duration of the yellow phase is lower than the given threshold."),
                               "", "-1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_JM_DRIVE_AFTER_RED_TIME,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("This value causes vehicles to violate a red light if the duration of the red phase is lower than the given threshold."),
                               "", "-1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_JM_DRIVE_RED_SPEED,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("This value causes vehicles affected by jmDriveAfterRedTime to slow down when violating a red light."),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_JM_IGNORE_FOE_PROB,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("This value causes vehicles to ignore foe vehicles that have right-of-way with the given probability."),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_JM_IGNORE_FOE_SPEED,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("This value is used in conjunction with jmIgnoreFoeProb.") + std::string("\n") +
                               TL("Only vehicles with a speed below or equal to the given value may be ignored."),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_JM_SIGMA_MINOR,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("This value configures driving imperfection (dawdling) while passing a minor link."),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_JM_TIMEGAP_MINOR,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("This value defines the minimum time gap when passing ahead of a prioritized vehicle. "),
                               "1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_IMPATIENCE,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Willingess of drivers to impede vehicles with higher priority"),
                               "0");
}


void
GNETagPropertiesDatabase::fillLaneChangingModelAttributes(GNETagProperties* tagProperties) {
    // fill lane changing model
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_STRATEGIC_PARAM,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The eagerness for performing strategic lane changing. Higher values result in earlier lane-changing."),
                               "1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_COOPERATIVE_PARAM,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The willingness for performing cooperative lane changing. Lower values result in reduced cooperation."),
                               "1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_SPEEDGAIN_PARAM,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The eagerness for performing lane changing to gain speed. Higher values result in more lane-changing."),
                               "1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_KEEPRIGHT_PARAM,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The eagerness for following the obligation to keep right. Higher values result in earlier lane-changing."),
                               "1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_SUBLANE_PARAM,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The eagerness for using the configured lateral alignment within the lane.") + std::string("\n") +
                               TL("Higher values result in increased willingness to sacrifice speed for alignment."),
                               "1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_OPPOSITE_PARAM,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The eagerness for overtaking through the opposite-direction lane. Higher values result in more lane-changing."),
                               "1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_PUSHY,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Willingness to encroach laterally on other drivers."),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_PUSHYGAP,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Minimum lateral gap when encroaching laterally on other drives (alternative way to define lcPushy)"),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_ASSERTIVE,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Willingness to accept lower front and rear gaps on the target lane."),
                               "1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_IMPATIENCE,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Dynamic factor for modifying lcAssertive and lcPushy."),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_TIME_TO_IMPATIENCE,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Time to reach maximum impatience (of 1). Impatience grows whenever a lane-change manoeuvre is blocked."),
                               "infinity");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_ACCEL_LAT,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Maximum lateral acceleration per second."),
                               "1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_LOOKAHEADLEFT,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Factor for configuring the strategic lookahead distance when a change to the left is necessary (relative to right lookahead)."),
                               "2");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_SPEEDGAINRIGHT,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Factor for configuring the threshold asymmetry when changing to the left or to the right for speed gain."),
                               "0.1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_MAXSPEEDLATSTANDING,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Upper bound on lateral speed when standing."),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_MAXSPEEDLATFACTOR,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Upper bound on lateral speed while moving computed as lcMaxSpeedLatStanding + lcMaxSpeedLatFactor * getSpeed()"),
                               "1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_TURN_ALIGNMENT_DISTANCE,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Distance to an upcoming turn on the vehicles route, below which the alignment") + std::string("\n") +
                               TL("should be dynamically adapted to match the turn direction."),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_OVERTAKE_RIGHT,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("The probability for violating rules gainst overtaking on the right."),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_KEEPRIGHT_ACCEPTANCE_TIME,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                               TL("Time threshold for the willingness to change right."),
                               "", "-1");

    auto factor = new GNEAttributeProperties(tagProperties, SUMO_ATTR_LCA_OVERTAKE_DELTASPEED_FACTOR,
            GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::RANGE | GNEAttributeProperties::Property::DEFAULTVALUE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE  | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
            TL("Speed difference factor for the eagerness of overtaking a neighbor vehicle before changing lanes (threshold = factor*speedlimit)."),
            "0");
    factor->setRange(-1, 1);

}


void
GNETagPropertiesDatabase::fillCommonPersonAttributes(GNETagProperties* tagProperties) {
    // fill person attributes
    fillIDAttribute(tagProperties, true);

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_TYPE,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::VTYPE,
                               GNEAttributeProperties::Edit::EDITMODE,
                               TL("The id of the person type to use for this person"),
                               DEFAULT_VTYPE_ID);

    fillColorAttribute(tagProperties, "yellow");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_DEPARTPOS,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE |  GNEAttributeProperties::Property::UPDATEGEOMETRY,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The position at which the person shall enter the net"),
                               "base");
}


void
GNETagPropertiesDatabase::fillCommonContainerAttributes(GNETagProperties* tagProperties) {
    // fill common container attributes
    fillIDAttribute(tagProperties, true);

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_TYPE,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY | GNEAttributeProperties::Property::VTYPE,
                               GNEAttributeProperties::Edit::EDITMODE,
                               TL("The id of the container type to use for this container"),
                               DEFAULT_CONTAINERTYPE_ID);

    fillColorAttribute(tagProperties, "yellow");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_DEPARTPOS,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE |  GNEAttributeProperties::Property::UPDATEGEOMETRY,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The position at which the container shall enter the net"),
                               "base");
}


void
GNETagPropertiesDatabase::fillCommonStopAttributes(GNETagProperties* tagProperties, const bool waypoint) {
    // fill common stop attributes
    auto duration = new GNEAttributeProperties(tagProperties, SUMO_ATTR_DURATION,
            GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::ACTIVATABLE | GNEAttributeProperties::Property::DEFAULTVALUE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
            TL("Minimum duration for stopping"),
            "60");
    duration->setDefaultActivated(true);

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_UNTIL,
                               GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::ACTIVATABLE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The time step at which the route continues"),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_EXTENSION,
                               GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::ACTIVATABLE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("If set to a non-negative time value, then the stop duration can be extended at most by the extension value in seconds"),
                               "0");

    if (!waypoint) {
        auto triggered = new GNEAttributeProperties(tagProperties, SUMO_ATTR_TRIGGERED,
                GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE,
                GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                TL("Whether a person or container or both may end the stop"),
                "false");
        triggered->setDiscreteValues({"false", "person", "container", "join"});

        new GNEAttributeProperties(tagProperties, SUMO_ATTR_EXPECTED,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("List of elements that must board the vehicle before it may continue"));

        new GNEAttributeProperties(tagProperties, SUMO_ATTR_JOIN,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Joins this train to another upon reaching the stop"));
    }

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_PERMITTED,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("List of elements that can board the vehicle before it may continue"));

    auto parking = new GNEAttributeProperties(tagProperties, SUMO_ATTR_PARKING,
            GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
            TL("Whether the vehicle stops on the road or beside"),
            "false");
    parking->setDiscreteValues({"true", "false", "opportunistic"});

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_ACTTYPE,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("Activity displayed for stopped person in GUI and output files"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_TRIP_ID,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("Parameter to be applied to the vehicle to track the trip id within a cyclical public transport route"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LINE,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("New line attribute to be set on the vehicle when reaching this stop (for cyclical public transport route)"));

    if (waypoint) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_SPEED,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Speed to be kept while driving between startPos and endPos"),
                                   "0");
    } else {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_ONDEMAND,
                                   GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Whether the stop may be skipped if no passengers wants to embark or disembark"),
                                   GNEAttributeCarrier::FALSE_STR);
    }

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_JUMP,
                               GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("transfer time if there shall be a jump from this stop to the next route edge"),
                               "", "-1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_SPLIT,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("Splits the train upon reaching the stop"));
}


void
GNETagPropertiesDatabase::fillPlanParentAttributes(GNETagProperties* tagProperties) {
    // fill plan parents
    // basic parents
    if (tagProperties->planConsecutiveEdges()) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_EDGES,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("list of consecutive edges"));

        new GNEAttributeProperties(tagProperties, SUMO_ATTR_ARRIVALPOS,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Arrival position on the last edge"),
                                   "", "-1");
    }
    if (tagProperties->planRoute()) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_ROUTE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Route ID"));

        new GNEAttributeProperties(tagProperties, SUMO_ATTR_ARRIVALPOS,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Arrival position on the destination edge"),
                                   "", "-1");
    }
    if (tagProperties->planEdge()) {

        fillEdgeAttribute(tagProperties, false);

        new GNEAttributeProperties(tagProperties, SUMO_ATTR_ENDPOS,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The end position on the lane (the higher position on the lane) in meters, must be larger than startPos by more than 0.1m"));
    }
    if (tagProperties->planBusStop()) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_BUS_STOP,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Bus stop ID"));
    }
    if (tagProperties->planTrainStop()) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_TRAIN_STOP,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Train stop ID"));
    }
    if (tagProperties->planContainerStop()) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_CONTAINER_STOP,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Container stop ID"));
    }
    if (tagProperties->planChargingStation()) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_CHARGING_STATION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Charging station ID"));
    }
    if (tagProperties->planParkingArea()) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_PARKING_AREA,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Parking area ID"));
    }
    // from parents
    if (tagProperties->planFromEdge()) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_FROM,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Edge start ID"));
    }
    if (tagProperties->planFromTAZ()) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_FROM_TAZ,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("TAZ start ID"));
    }
    if (tagProperties->planFromJunction()) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_FROM_JUNCTION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Junction start ID"));
    }
    if (tagProperties->planFromBusStop()) {
        new GNEAttributeProperties(tagProperties, GNE_ATTR_FROM_BUSSTOP,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("BusStop start ID"));
    }
    if (tagProperties->planFromTrainStop()) {
        new GNEAttributeProperties(tagProperties, GNE_ATTR_FROM_TRAINSTOP,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("TrainStop start ID"));
    }
    if (tagProperties->planFromContainerStop()) {
        new GNEAttributeProperties(tagProperties, GNE_ATTR_FROM_CONTAINERSTOP,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("ContainerStop start ID"));
    }
    if (tagProperties->planFromChargingStation()) {
        new GNEAttributeProperties(tagProperties, GNE_ATTR_FROM_CHARGINGSTATION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("ChargingStation start ID"));
    }
    if (tagProperties->planFromParkingArea()) {
        new GNEAttributeProperties(tagProperties, GNE_ATTR_FROM_CHARGINGSTATION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("ParkingArea start ID"));
    }
    // to parents
    if (tagProperties->planToEdge()) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_TO,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Edge end ID"));
        // departPos only for tranships
        if (tagProperties->isPlanTranship()) {
            // depart pos
            new GNEAttributeProperties(tagProperties, SUMO_ATTR_DEPARTPOS,
                                       GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                       GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                       TL("The position at which the tranship shall enter the net"),
                                       "0");
        }
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_ARRIVALPOS,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("arrival position on the destination edge"),
                                   "", "-1");
    }
    if (tagProperties->planToTAZ()) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_TO_TAZ,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("TAZ end ID"));
    }
    if (tagProperties->planToJunction()) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_TO_JUNCTION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Junction end ID"));
    }
    if (tagProperties->planToBusStop()) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_BUS_STOP,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("BusStop end ID"));
    }
    if (tagProperties->planToTrainStop()) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_TRAIN_STOP,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("TrainStop end ID"));
    }
    if (tagProperties->planToContainerStop()) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_CONTAINER_STOP,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("ContainerStop end ID"));
    }
    if (tagProperties->planToChargingStation()) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_CHARGING_STATION,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("ChargingStation end ID"));
    }
    if (tagProperties->planToParkingArea()) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_PARKING_AREA,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("ParkingArea end ID"));
    }
}


void
GNETagPropertiesDatabase::fillPersonTripCommonAttributes(GNETagProperties* tagProperties) {
    // fill person trip common attributes
    fillVTypesAttribute(tagProperties);

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_MODES,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("List of possible traffic modes. Walking is always possible regardless of this value"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LINES,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("list of vehicle alternatives to take for the person trip"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_WALKFACTOR,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("Walk factor"),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_GROUP,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("id of the travel group. Persons with the same group may share a taxi ride"));
}


void
GNETagPropertiesDatabase::fillWalkCommonAttributes(GNETagProperties* tagProperties) {
    // fill walk common attributes
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_SPEED,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TLF("speed of the person for this % in m/s (not together with duration)", tagProperties->getTagStr()),
                               "1.39");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_DURATION,
                               GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("duration of the plan in second (not together with speed)"),
                               "0");
}


void
GNETagPropertiesDatabase::fillRideCommonAttributes(GNETagProperties* tagProperties) {
    // fill ride common attributes
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LINES,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("list of vehicle alternatives to take for the ride"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_GROUP,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("id of the travel group. Persons with the same group may share a taxi ride"));
}


void
GNETagPropertiesDatabase::fillTransportCommonAttributes(GNETagProperties* tagProperties) {
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_LINES,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("list of vehicle alternatives to take for the transport"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_GROUP,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("id of the travel group. Persons with the same group may share a taxi ride"));
}


void
GNETagPropertiesDatabase::fillTranshipCommonAttributes(GNETagProperties* tagProperties) {
    // fill tranship attributes
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_SPEED,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("speed of the person for this tranship in m/s (not together with duration)"),
                               "1.39");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_DURATION,
                               GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("duration of the plan in second (not together with speed)"),
                               "0");
}


void
GNETagPropertiesDatabase::fillPlanStopCommonAttributes(GNETagProperties* tagProperties) {
    // fill plan stop common attributes
    auto duration = new GNEAttributeProperties(tagProperties, SUMO_ATTR_DURATION,
            GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::ACTIVATABLE | GNEAttributeProperties::Property::DEFAULTVALUE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
            TL("Minimum duration for stopping"),
            "60");
    duration->setDefaultActivated(true);

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_UNTIL,
                               GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::ACTIVATABLE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The time step at which the route continues"),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_ACTTYPE,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("Activity displayed for stopped person in GUI and output files "));

    // friendlyPos attribute only for stops over edges
    if (tagProperties->hasAttribute(SUMO_ATTR_EDGE)) {
        fillFriendlyPosAttribute(tagProperties);
    }
}


void
GNETagPropertiesDatabase::fillDataElements() {
    // fill data set element
    SumoXMLTag currentTag = SUMO_TAG_DATASET;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties[GNE_TAG_SUPERMODE_DATA],
                GNETagProperties::Type::DATAELEMENT,
                GNETagProperties::Property::NOTDRAWABLE | GNETagProperties::Property::NOPARAMETERS | GNETagProperties::Property::NOTSELECTABLE,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::DATA,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::DATASET, GUIGlObjectType::GLO_DATASET, currentTag, TL("DataSet"));

        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);
    }
    // fill data interval element
    currentTag = SUMO_TAG_DATAINTERVAL;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties[GNE_TAG_SUPERMODE_DATA],
                GNETagProperties::Type::DATAELEMENT,
                GNETagProperties::Property::NOTDRAWABLE | GNETagProperties::Property::NOPARAMETERS | GNETagProperties::Property::XMLCHILD | GNETagProperties::Property::NOTSELECTABLE,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::DATAINTERVAL, GUIGlObjectType::GLO_DATAINTERVAL, currentTag, TL("DataInterval"),
        {SUMO_TAG_DATASET});

        // set values of attributes
        fillIDAttribute(myTagProperties[currentTag], true);

        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_BEGIN,
                                   GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Data interval begin time"),
                                   "0");

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_END,
                                   GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("Data interval end time"),
                                   "3600");
    }
    // fill edge data element
    currentTag = GNE_TAG_EDGEREL_SINGLE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_DATAS),
                GNETagProperties::Type::DATAELEMENT | GNETagProperties::Type::GENERICDATA,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::EDGE,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::EDGEDATA, GUIGlObjectType::GLO_EDGEDATA, SUMO_TAG_EDGE, TL("EdgeRelationSingle"));
    }
    currentTag = SUMO_TAG_EDGEREL;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_DATAS),
                GNETagProperties::Type::DATAELEMENT | GNETagProperties::Type::GENERICDATA,
                GNETagProperties::Property::NO_PROPERTY,
                GNETagProperties::Over::FROM_EDGE | GNETagProperties::Over::TO_EDGE,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::EDGERELDATA, GUIGlObjectType::GLO_EDGERELDATA, currentTag, TL("EdgeRelation"));

        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_FROM,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The ID of the edge the edgeRel starts at"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TO,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The ID of the edge the edgeRel ends at"));
    }
    currentTag = SUMO_TAG_TAZREL;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_DATAS),
                GNETagProperties::Type::DATAELEMENT | GNETagProperties::Type::GENERICDATA,
                GNETagProperties::Property::RTREE | GNETagProperties::Property::XMLCHILD,
                GNETagProperties::Over::FROM_TAZ | GNETagProperties::Over::TO_TAZ,
                FileBucket::Type::NOTHING,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::TAZRELDATA, GUIGlObjectType::GLO_TAZRELDATA, currentTag, TL("TAZRelation"),
        {SUMO_TAG_DATAINTERVAL});

        // set values of attributes
        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_FROM,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The name of the TAZ the TAZRel starts at"));

        new GNEAttributeProperties(myTagProperties[currentTag], SUMO_ATTR_TO,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The name of the TAZ the TAZRel ends at"));
    }
    currentTag = SUMO_TAG_MEANDATA_EDGE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_SUPERMODE_DATA),
                GNETagProperties::Type::MEANDATA,
                GNETagProperties::Property::NOTDRAWABLE | GNETagProperties::Property::NOPARAMETERS | GNETagProperties::Property::NOTSELECTABLE,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::MEANDATA | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::MEANDATAEDGE, GUIGlObjectType::GLO_MEANDATA, currentTag, TL("MeanDataEdge"));

        // set values of attributes
        fillCommonMeanDataAttributes(myTagProperties[currentTag]);
    }
    currentTag = SUMO_TAG_MEANDATA_LANE;
    {
        // set values of tag
        myTagProperties[currentTag] = new GNETagProperties(currentTag, mySetTagProperties.at(GNE_TAG_SUPERMODE_DATA),
                GNETagProperties::Type::MEANDATA,
                GNETagProperties::Property::NOTDRAWABLE | GNETagProperties::Property::NOPARAMETERS | GNETagProperties::Property::NOTSELECTABLE,
                GNETagProperties::Over::VIEW,
                FileBucket::Type::MEANDATA | FileBucket::Type::ADDITIONAL,
                GNETagProperties::Conflicts::NO_CONFLICTS,
                GUIIcon::MEANDATALANE, GUIGlObjectType::GLO_MEANDATA, currentTag, TL("MeanDataLane"));

        // set values of attributes
        fillCommonMeanDataAttributes(myTagProperties[currentTag]);
    }
}


void
GNETagPropertiesDatabase::fillCommonMeanDataAttributes(GNETagProperties* tagProperties) {
    // fill all meanData attributes
    fillIDAttribute(tagProperties, true);

    fillFileAttribute(tagProperties);

    auto meanDataType = new GNEAttributeProperties(tagProperties, SUMO_ATTR_TYPE,
            GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
            TL("Type of data generated by this mean data"),
            SUMOXMLDefinitions::MeanDataTypes.getString(MeanDataType::DEFAULT));
    meanDataType->setDiscreteValues(SUMOXMLDefinitions::MeanDataTypes.getStrings());

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_PERIOD,
                               GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The aggregation period the values the detector collects shall be summed up"),
                               "-1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_BEGIN,
                               GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The time to start writing. If not given, the simulation's begin is used."),
                               "-1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_END,
                               GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The time to end writing. If not given the simulation's end is used."),
                               "-1");

    auto excludeEmpty = new GNEAttributeProperties(tagProperties, SUMO_ATTR_EXCLUDE_EMPTY,
            GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
            TL("If set to true, edges/lanes which were not used by a vehicle during this period will not be written"),
            SUMOXMLDefinitions::ExcludeEmptys.getString(ExcludeEmpty::FALSES));
    excludeEmpty->setDiscreteValues(SUMOXMLDefinitions::ExcludeEmptys.getStrings());

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_WITH_INTERNAL,
                               GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("If set, junction internal edges/lanes will be written as well"),
                               GNEAttributeCarrier::FALSE_STR);

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_MAX_TRAVELTIME,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The maximum travel time in seconds to write if only very small movements occur"),
                               toString(100000));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_MIN_SAMPLES,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("Consider an edge/lane unused if it has at most this many sampled seconds"),
                               "0");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_HALTING_SPEED_THRESHOLD,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The maximum speed to consider a vehicle halting;"),
                               "0.1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_VTYPES,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("space separated list of vehicle type ids to consider"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_TRACK_VEHICLES,
                               GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("whether aggregation should be performed over all vehicles that entered the edge/lane in the aggregation interval"),
                               GNEAttributeCarrier::FALSE_STR);

    fillDetectPersonsAttribute(tagProperties);

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_WRITE_ATTRIBUTES,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("List of attribute names that shall be written"));

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_EDGES,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("Restrict output to the given list of edge ids"));

    auto edgesFile = new GNEAttributeProperties(tagProperties, SUMO_ATTR_EDGESFILE,
            GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::FILEOPEN | GNEAttributeProperties::Property::DEFAULTVALUE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
            TL("Restrict output to the given list of edges given in file"));
    edgesFile->setFilenameExtensions(SUMOXMLDefinitions::OutputFileExtensions.getStrings());

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_AGGREGATE,
                               GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("Whether the traffic statistic of all edges shall be aggregated into a single value"),
                               GNEAttributeCarrier::FALSE_STR);
}


void
GNETagPropertiesDatabase::fillIDAttribute(GNETagProperties* tagProperties, const bool createMode) {
    if (createMode) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_ID,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                   TLF("ID of %", tagProperties->getTagStr()));
    } else {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_ID,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE,
                                   GNEAttributeProperties::Edit::EDITMODE,
                                   TLF("ID of %", tagProperties->getTagStr()));
    }
}


void
GNETagPropertiesDatabase::fillNameAttribute(GNETagProperties* tagProperties) {
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_NAME,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TLF("Optional name for %", tagProperties->getTagStr()));
}


void
GNETagPropertiesDatabase::fillEdgeAttribute(GNETagProperties* tagProperties, const bool synonymID) {
    if (synonymID) {
        // set values of attributes
        auto edge = new GNEAttributeProperties(tagProperties, SUMO_ATTR_EDGE,
                                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::SYNONYM | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                               GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                               TL("The id of an edge in the simulation network"));
        edge->setSynonym(SUMO_ATTR_ID);
    } else {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_EDGE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                   TL("The id of an edge in the simulation network"));
    }
}


void
GNETagPropertiesDatabase::fillLaneAttribute(GNETagProperties* tagProperties, const bool synonymID) {
    if (synonymID) {
        auto lane = new GNEAttributeProperties(tagProperties, SUMO_ATTR_LANE,
                                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::SYNONYM | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                               GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                               TLF("The name of the lane the % shall be located at", tagProperties->getTagStr()));
        lane->setSynonym(SUMO_ATTR_ID);
    } else {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_LANE,
                                   GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                                   GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                                   TLF("The name of the lane the % shall be located at", tagProperties->getTagStr()));
    }
}


void
GNETagPropertiesDatabase::fillFriendlyPosAttribute(GNETagProperties* tagProperties) {
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_FRIENDLY_POS,
                               GNEAttributeProperties::Property::BOOL | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("If set, no error will be reported if element is placed behind the lane.") + std::string("\n") +
                               TL("Instead, it will be placed 0.1 meters from the lanes end or at position 0.1,") + std::string("\n") +
                               TL("if the position was negative and larger than the lanes length after multiplication with - 1"),
                               GNEAttributeCarrier::FALSE_STR);
}


void
GNETagPropertiesDatabase::fillVTypesAttribute(GNETagProperties* tagProperties) {
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_VTYPES,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("Space separated list of vehicle type ids to consider"));
}


void
GNETagPropertiesDatabase::fillFileAttribute(GNETagProperties* tagProperties) {
    auto file = new GNEAttributeProperties(tagProperties, SUMO_ATTR_FILE,
                                           GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::FILESAVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                           GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                           TL("The path to the output file"));
    file->setFilenameExtensions(SUMOXMLDefinitions::OutputFileExtensions.getStrings());
}


void
GNETagPropertiesDatabase::fillOutputAttribute(GNETagProperties* tagProperties) {
    auto output = new GNEAttributeProperties(tagProperties, SUMO_ATTR_OUTPUT,
            GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::FILESAVE | GNEAttributeProperties::Property::DEFAULTVALUE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
            TL("Path to the output file for writing information"));
    output->setFilenameExtensions(SUMOXMLDefinitions::OutputFileExtensions.getStrings());
}


void
GNETagPropertiesDatabase::fillImgFileAttribute(GNETagProperties* tagProperties, const bool isExtended) {
    GNEAttributeProperties* imgFile = nullptr;
    if (isExtended) {
        imgFile = new GNEAttributeProperties(tagProperties, SUMO_ATTR_IMGFILE,
                                             GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::FILEOPEN | GNEAttributeProperties::Property::DEFAULTVALUE,
                                             GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::EXTENDEDEDITOR,
                                             TLF("A bitmap to use for rendering this %", tagProperties->getTagStr()));
    } else {
        imgFile = new GNEAttributeProperties(tagProperties, SUMO_ATTR_IMGFILE,
                                             GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::FILEOPEN | GNEAttributeProperties::Property::DEFAULTVALUE,
                                             GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                             TLF("A bitmap to use for rendering this %", tagProperties->getTagStr()));
    }
    imgFile->setFilenameExtensions(SUMOXMLDefinitions::ImageFileExtensions.getStrings());
}


void
GNETagPropertiesDatabase::fillDepartAttribute(GNETagProperties* tagProperties) {
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_DEPART,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TLF("The departure time of the (first) % which is generated using this trip definition", tagProperties->getTagStr()),
                               "0");
}


void
GNETagPropertiesDatabase::fillAllowDisallowAttributes(GNETagProperties* tagProperties) {
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_ALLOW,
                               GNEAttributeProperties::Property::VCLASS | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                               TL("Explicitly allows the given vehicle classes (not given will be not allowed)"),
                               "all");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_DISALLOW,
                               GNEAttributeProperties::Property::VCLASS | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::COPYABLE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                               TL("Explicitly disallows the given vehicle classes (not given will be allowed)"));
}


void
GNETagPropertiesDatabase::fillPosOverLaneAttribute(GNETagProperties* tagProperties) {
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_POSITION,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::UNIQUE | GNEAttributeProperties::Property::DEFAULTVALUE | GNEAttributeProperties::Property::UPDATEGEOMETRY,
                               GNEAttributeProperties::Edit::EDITMODE | GNEAttributeProperties::Edit::DIALOGEDITOR,
                               TLF("The position on the lane the % shall be laid on in meters", tagProperties->getTagStr()),
                               "0");
}


void
GNETagPropertiesDatabase::fillDetectPersonsAttribute(GNETagProperties* tagProperties) {
    auto detectPersons = new GNEAttributeProperties(tagProperties, SUMO_ATTR_DETECT_PERSONS,
            GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::DISCRETE | GNEAttributeProperties::Property::DEFAULTVALUE,
            GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
            TL("Detect persons instead of vehicles (pedestrians or passengers)"),
            SUMOXMLDefinitions::PersonModeValues.getString(PersonMode::NONE));
    detectPersons->setDiscreteValues(SUMOXMLDefinitions::PersonModeValues.getStrings());
}


void
GNETagPropertiesDatabase::fillColorAttribute(GNETagProperties* tagProperties, const std::string& defaultColor) {
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_COLOR,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::COLOR | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TLF("The RGBA color with which the % shall be displayed", tagProperties->getTagStr()),
                               defaultColor);
}


void
GNETagPropertiesDatabase::fillDetectorPeriodAttribute(GNETagProperties* tagProperties) {
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_PERIOD,
                               GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TLF("The aggregation period the values the % detector collects shall be summed up", tagProperties->getTagStr()),
                               "300");
}


void
GNETagPropertiesDatabase::fillDetectorNextEdgesAttribute(GNETagProperties* tagProperties) {
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_NEXT_EDGES,
                               GNEAttributeProperties::Property::STRING | GNEAttributeProperties::Property::LIST | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("List of edge ids that must all be part of the future route of the vehicle to qualify for detection"));
}


void
GNETagPropertiesDatabase::fillDetectorThresholdAttributes(GNETagProperties* tagProperties, const bool includingJam) {
    new GNEAttributeProperties(tagProperties, SUMO_ATTR_HALTING_TIME_THRESHOLD,
                               GNEAttributeProperties::Property::SUMOTIME | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The time-based threshold that describes how much time has to pass until a vehicle is recognized as halting)"),
                               "1");

    new GNEAttributeProperties(tagProperties, SUMO_ATTR_HALTING_SPEED_THRESHOLD,
                               GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                               GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                               TL("The speed-based threshold that describes how slow a vehicle has to be to be recognized as halting) in m/s"),
                               "1.39");
    if (includingJam) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_JAM_DIST_THRESHOLD,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The maximum distance to the next standing vehicle in order to make this vehicle count as a participant to the jam in m"),
                                   "10");
    }
}


void
GNETagPropertiesDatabase::fillDistributionProbability(GNETagProperties* tagProperties, const bool defaultValue) {
    if (defaultValue) {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_PROB,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE | GNEAttributeProperties::Property::DEFAULTVALUE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The probability when being added to a distribution"),
                                   toString(DEFAULT_VEH_PROB));
    } else {
        new GNEAttributeProperties(tagProperties, SUMO_ATTR_PROB,
                                   GNEAttributeProperties::Property::FLOAT | GNEAttributeProperties::Property::POSITIVE,
                                   GNEAttributeProperties::Edit::CREATEMODE | GNEAttributeProperties::Edit::EDITMODE,
                                   TL("The probability when being added to a distribution"));
    }
}


void
GNETagPropertiesDatabase::updateMaxNumberOfAttributesEditorRows() {
    for (const auto& tagPropertyItem : myTagProperties) {
        int basicEditableAttributes = 0;
        int geoAttributes = 0;
        int flowAttributes = 0;
        int neteditAttributes = 0;
        for (const auto& attributeProperty : tagPropertyItem.second->getAttributeProperties()) {
            if (attributeProperty->isCreateMode() || attributeProperty->isEditMode()) {
                if (attributeProperty->isBasicEditor()) {
                    basicEditableAttributes++;
                }
                if (attributeProperty->isGeoEditor()) {
                    geoAttributes++;
                }
                if (attributeProperty->isFlowEditor()) {
                    flowAttributes++;
                }
                if (attributeProperty->isNeteditEditor()) {
                    neteditAttributes++;
                }
            }
        }
        if (myMaxNumberOfEditableAttributeRows < basicEditableAttributes) {
            myMaxNumberOfEditableAttributeRows = basicEditableAttributes;
        }
        if (myMaxNumberOfGeoAttributeRows < geoAttributes) {
            myMaxNumberOfGeoAttributeRows = geoAttributes;
        }
        if (myMaxNumberOfFlowAttributeRows < flowAttributes) {
            myMaxNumberOfFlowAttributeRows = flowAttributes;
        }
        if (myMaxNumberOfNeteditAttributeRows < neteditAttributes) {
            myMaxNumberOfNeteditAttributeRows = neteditAttributes;
        }
    }
}


void
GNETagPropertiesDatabase::updateMaxHierarchyDepth() {
    for (const auto& tagPropertyItem : myTagProperties) {
        const int hierarchySize = (int)tagPropertyItem.second->getHierarchicalParentsRecuersively().size();
        if (hierarchySize > myHierarchyDepth) {
            myHierarchyDepth = hierarchySize;
        }
    }
}

/****************************************************************************/
