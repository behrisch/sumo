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
/// @file    GNERouteProbe.cpp
/// @author  Pablo Alvarez Lopez
/// @date    May 2016
///
//
/****************************************************************************/
#include <config.h>

#include <utils/gui/div/GLHelper.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNEUndoList.h>
#include <netedit/GNENet.h>
#include <netedit/changes/GNEChange_Attribute.h>
#include <utils/gui/globjects/GLIncludes.h>
#include <utils/gui/div/GUIGlobalViewObjectsHandler.h>

#include "GNERouteProbe.h"


// ===========================================================================
// member method definitions
// ===========================================================================

GNERouteProbe::GNERouteProbe(GNENet* net) :
    GNEAdditional("", net, GLO_ROUTEPROBE, SUMO_TAG_ROUTEPROBE,
                  GUIIconSubSys::getIcon(GUIIcon::ROUTEPROBE), "", {}, {}, {}, {}, {}, {}),
                            myPeriod(SUMOTime_MAX_PERIOD),
    myBegin(0) {
    // reset default values
    resetDefaultValues();
    // update centering boundary without updating grid
    updateCenteringBoundary(false);
}


GNERouteProbe::GNERouteProbe(const std::string& id, GNENet* net, GNEEdge* edge, const SUMOTime period, const std::string& name,
                             const std::string& filename, SUMOTime begin, const std::vector<std::string>& vehicleTypes,
                             const Parameterised::Map& parameters) :
    GNEAdditional(id, net, GLO_ROUTEPROBE, SUMO_TAG_ROUTEPROBE,
                  GUIIconSubSys::getIcon(GUIIcon::ROUTEPROBE), name, {}, {edge}, {}, {}, {}, {}),
    Parameterised(parameters),
    myPeriod(period),
    myFilename(filename),
    myBegin(begin),
    myVehicleTypes(vehicleTypes) {
    // update centering boundary without updating grid
    updateCenteringBoundary(false);
}


GNERouteProbe::~GNERouteProbe() {
}


void
GNERouteProbe::writeAdditional(OutputDevice& device) const {
    // open tag
    device.openTag(SUMO_TAG_ROUTEPROBE);
    // write parameters
    device.writeAttr(SUMO_ATTR_ID, getID());
    device.writeAttr(SUMO_ATTR_BEGIN, time2string(myBegin));
    if (getAttribute(SUMO_ATTR_PERIOD).size() > 0) {
        device.writeAttr(SUMO_ATTR_PERIOD, time2string(myPeriod));
    }
    device.writeAttr(SUMO_ATTR_EDGE, getParentEdges().front()->getID());
    if (!myFilename.empty()) {
        device.writeAttr(SUMO_ATTR_FILE, myFilename);
    }
    if (!myAdditionalName.empty()) {
        device.writeAttr(SUMO_ATTR_NAME, myAdditionalName);
    }
    if (!myVehicleTypes.empty()) {
        device.writeAttr(SUMO_ATTR_VTYPES, myVehicleTypes);
    }
    // write parameters (Always after children to avoid problems with additionals.xsd)
    writeParams(device);
    device.closeTag();
}


bool
GNERouteProbe::isAdditionalValid() const {
    return true;
}


std::string
GNERouteProbe::getAdditionalProblem() const {
    return "";
}


void
GNERouteProbe::fixAdditionalProblem() {
    // nothing to fix
}


void
GNERouteProbe::updateGeometry() {
    // calculate perpendicular line
    calculatePerpendicularLine(1);
}


Position
GNERouteProbe::getPositionInView() const {
    return getAdditionalGeometry().getShape().getPolygonCenter();
}


void
GNERouteProbe::updateCenteringBoundary(const bool /*pdateGrid*/) {
    // nothing to do
}


void
GNERouteProbe::splitEdgeGeometry(const double /*splitPosition*/, const GNENetworkElement* /*originalElement*/, const GNENetworkElement* /*newElement*/, GNEUndoList* /*undoList*/) {
    // geometry of this element cannot be splitted
}


bool
GNERouteProbe::checkDrawMoveContour() const {
    return false;
}


GNEMoveOperation*
GNERouteProbe::getMoveOperation() {
    // routeprobes cannot be moved
    return nullptr;
}


std::string
GNERouteProbe::getParentName() const {
    return getParentEdges().front()->getID();
}


void
GNERouteProbe::drawGL(const GUIVisualizationSettings& s) const {
    // first check if additional has to be drawn
    if (myNet->getViewNet()->getDataViewOptions().showAdditionals()) {
        // Obtain exaggeration of the draw
        const double routeProbeExaggeration = getExaggeration(s);
        // declare colors
        RGBColor routeProbeColor, centralLineColor;
        // set colors
        if (drawUsingSelectColor()) {
            routeProbeColor = s.colorSettings.selectedAdditionalColor;
            centralLineColor = routeProbeColor.changedBrightness(-32);
        } else {
            routeProbeColor = s.additionalSettings.routeProbeColor;
            centralLineColor = RGBColor::WHITE;
        }
        // get detail level
        const auto d = s.getDetailLevel(routeProbeExaggeration);
        // draw geometry only if we'rent in drawForObjectUnderCursor mode
        if (s.checkDrawAdditional(d, isAttributeCarrierSelected())) {
            // draw parent and child lines
            drawParentChildLines(s, s.additionalSettings.connectionColor);
            // Add layer matrix matrix
            GLHelper::pushMatrix();
            // translate to front
            myNet->getViewNet()->drawTranslateFrontAttributeCarrier(this, GLO_ROUTEPROBE);
            // set base color
            GLHelper::setColor(routeProbeColor);
            // Draw the area using shape, shapeRotations, shapeLengths and value of exaggeration
            GUIGeometry::drawGeometry(d, myAdditionalGeometry, 0.3 * routeProbeExaggeration);
            // move to front
            glTranslated(0, 0, .1);
            // set central color
            GLHelper::setColor(centralLineColor);
            // Draw the area using shape, shapeRotations, shapeLengths and value of exaggeration
            GUIGeometry::drawGeometry(d, myAdditionalGeometry, 0.05 * routeProbeExaggeration);
            // move to icon position and front
            glTranslated(myAdditionalGeometry.getShape().front().x(), myAdditionalGeometry.getShape().front().y(), .1);
            // rotate over lane
            GUIGeometry::rotateOverLane(myAdditionalGeometry.getShapeRotations().front() * -1);
            // Draw icon depending of level of detail
            if (d <= GUIVisualizationSettings::Detail::AdditionalDetails) {
                // set color
                glColor3d(1, 1, 1);
                // rotate texture
                glRotated(90, 0, 0, 1);
                // draw texture
                if (drawUsingSelectColor()) {
                    GUITexturesHelper::drawTexturedBox(GUITextureSubSys::getTexture(GUITexture::ROUTEPROBE_SELECTED), s.additionalSettings.routeProbeSize * routeProbeExaggeration);
                } else {
                    GUITexturesHelper::drawTexturedBox(GUITextureSubSys::getTexture(GUITexture::ROUTEPROBE), s.additionalSettings.routeProbeSize * routeProbeExaggeration);
                }
            } else {
                // set route probe color
                GLHelper::setColor(routeProbeColor);
                // just drawn a box
                GLHelper::drawBoxLine(Position(0, 0), 0, 2 * s.additionalSettings.routeProbeSize, s.additionalSettings.routeProbeSize * routeProbeExaggeration);
            }
            // pop layer matrix
            GLHelper::popMatrix();
            // draw additional name
            drawAdditionalName(s);
            // draw dotted contour
            myAdditionalContour.drawDottedContours(s, d, this, s.dottedContourSettings.segmentWidth, true);
            mySymbolBaseContour.drawDottedContours(s, d, this, s.dottedContourSettings.segmentWidthSmall, true);
        }
        // calculate contour and draw dotted geometry
        myAdditionalContour.calculateContourRectangleShape(s, d, this, myAdditionalGeometry.getShape().front(), s.additionalSettings.routeProbeSize,
                s.additionalSettings.routeProbeSize, getType(), 0, 0, 0, routeProbeExaggeration);
        mySymbolBaseContour.calculateContourExtrudedShape(s, d, this, myAdditionalGeometry.getShape(), getType(), 0.3, routeProbeExaggeration, true, true, 0);
    }
}


std::string
GNERouteProbe::getAttribute(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_ID:
            return getMicrosimID();
        case SUMO_ATTR_EDGE:
            return getParentEdges().front()->getID();
        case SUMO_ATTR_NAME:
            return myAdditionalName;
        case SUMO_ATTR_FILE:
            return myFilename;
        case SUMO_ATTR_PERIOD:
        case SUMO_ATTR_FREQUENCY:
            if (myPeriod == SUMOTime_MAX_PERIOD) {
                return "";
            } else {
                return time2string(myPeriod);
            }
        case SUMO_ATTR_BEGIN:
            return time2string(myBegin);
        case SUMO_ATTR_VTYPES:
            return toString(myVehicleTypes);
        case GNE_ATTR_SELECTED:
            return toString(isAttributeCarrierSelected());
        case GNE_ATTR_PARAMETERS:
            return getParametersStr();
        default:
            throw InvalidArgument(getTagStr() + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


double
GNERouteProbe::getAttributeDouble(SumoXMLAttr key) const {
    switch (key) {
        case SUMO_ATTR_BEGIN:
            return STEPS2TIME(myBegin);
        default:
            throw InvalidArgument(getTagStr() + " doesn't have a double attribute of type '" + toString(key) + "'");
    }
}


const Parameterised::Map&
GNERouteProbe::getACParametersMap() const {
    return getParametersMap();
}


void
GNERouteProbe::setAttribute(SumoXMLAttr key, const std::string& value, GNEUndoList* undoList) {
    if (value == getAttribute(key)) {
        return; //avoid needless changes, later logic relies on the fact that attributes have changed
    }
    switch (key) {
        case SUMO_ATTR_ID:
        case SUMO_ATTR_EDGE:
        case SUMO_ATTR_NAME:
        case SUMO_ATTR_FILE:
        case SUMO_ATTR_PERIOD:
        case SUMO_ATTR_FREQUENCY:
        case SUMO_ATTR_BEGIN:
        case SUMO_ATTR_VTYPES:
        case GNE_ATTR_SELECTED:
        case GNE_ATTR_PARAMETERS:
            GNEChange_Attribute::changeAttribute(this, key, value, undoList);
            break;
        default:
            throw InvalidArgument(getTagStr() + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


std::string
GNERouteProbe::getPopUpID() const {
    return getTagStr();
}


std::string
GNERouteProbe::getHierarchyName() const {
    return getTagStr() + ": " + getAttribute(SUMO_ATTR_BEGIN);
}

// ===========================================================================
// private
// ===========================================================================

bool
GNERouteProbe::isValid(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            return isValidAdditionalID(value);
        case SUMO_ATTR_EDGE:
            if (myNet->getAttributeCarriers()->retrieveEdge(value, false) != nullptr) {
                return true;
            } else {
                return false;
            }
        case SUMO_ATTR_NAME:
            return SUMOXMLDefinitions::isValidAttribute(value);
        case SUMO_ATTR_FILE:
            return SUMOXMLDefinitions::isValidFilename(value);
        case SUMO_ATTR_PERIOD:
        case SUMO_ATTR_FREQUENCY:
            if (value.empty()) {
                return true;
            } else {
                return (canParse<double>(value) && (parse<double>(value) >= 0));
            }
        case SUMO_ATTR_BEGIN:
            return canParse<SUMOTime>(value);
        case SUMO_ATTR_VTYPES:
            if (value.empty()) {
                return true;
            } else {
                return SUMOXMLDefinitions::isValidListOfTypeID(value);
            }
        case GNE_ATTR_SELECTED:
            return canParse<bool>(value);
        case GNE_ATTR_PARAMETERS:
            return areParametersValid(value);
        default:
            throw InvalidArgument(getTagStr() + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNERouteProbe::setAttribute(SumoXMLAttr key, const std::string& value) {
    switch (key) {
        case SUMO_ATTR_ID:
            // update microsimID
            setAdditionalID(value);
            break;
        case SUMO_ATTR_EDGE:
            replaceAdditionalParentEdges(value);
            break;
        case SUMO_ATTR_NAME:
            myAdditionalName = value;
            break;
        case SUMO_ATTR_FILE:
            myFilename = value;
            break;
        case SUMO_ATTR_PERIOD:
        case SUMO_ATTR_FREQUENCY:
            if (value.empty()) {
                myPeriod = SUMOTime_MAX_PERIOD;
            } else {
                myPeriod = string2time(value);
            }
            break;
        case SUMO_ATTR_BEGIN:
            myBegin = parse<SUMOTime>(value);
            break;
        case SUMO_ATTR_VTYPES:
            myVehicleTypes = parse<std::vector<std::string> >(value);
            break;
        case GNE_ATTR_SELECTED:
            if (parse<bool>(value)) {
                selectAttributeCarrier();
            } else {
                unselectAttributeCarrier();
            }
            break;
        case GNE_ATTR_PARAMETERS:
            setParametersStr(value);
            break;
        default:
            throw InvalidArgument(getTagStr() + " doesn't have an attribute of type '" + toString(key) + "'");
    }
}


void
GNERouteProbe::setMoveShape(const GNEMoveResult& /*moveResult*/) {
    // nothing to do
}


void
GNERouteProbe::commitMoveShape(const GNEMoveResult& /*moveResult*/, GNEUndoList* /*undoList*/) {
    // nothing to do
}


/****************************************************************************/
