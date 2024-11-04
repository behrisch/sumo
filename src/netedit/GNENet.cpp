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
/// @file    GNENet.cpp
/// @author  Jakob Erdmann
/// @date    Feb 2011
///
// A visual container for GNE-network-components such as GNEEdge and GNEJunction.
// GNE components wrap netbuild-components and supply visualisation and editing
// capabilities (adapted from GUINet)
//
// WorkrouteFlow (rough draft)
//   use NILoader to fill
//   do netedit stuff
//   call compute to save results
//
/****************************************************************************/
#include <netbuild/NBAlgorithms.h>
#include <netbuild/NBNetBuilder.h>
#include <netedit/changes/GNEChange_Additional.h>
#include <netedit/changes/GNEChange_Attribute.h>
#include <netedit/changes/GNEChange_Connection.h>
#include <netedit/changes/GNEChange_Crossing.h>
#include <netedit/changes/GNEChange_DataSet.h>
#include <netedit/changes/GNEChange_DataInterval.h>
#include <netedit/changes/GNEChange_GenericData.h>
#include <netedit/changes/GNEChange_DemandElement.h>
#include <netedit/changes/GNEChange_MeanData.h>
#include <netedit/changes/GNEChange_Edge.h>
#include <netedit/changes/GNEChange_Junction.h>
#include <netedit/changes/GNEChange_Lane.h>
#include <netedit/changes/GNEChange_RegisterJoin.h>
#include <netedit/dialogs/GNEFixAdditionalElements.h>
#include <netedit/dialogs/GNEFixDemandElements.h>
#include <netedit/elements/GNEGeneralHandler.h>
#include <netedit/elements/data/GNEDataHandler.h>
#include <netedit/elements/data/GNEDataInterval.h>
#include <netedit/elements/network/GNEConnection.h>
#include <netedit/elements/network/GNECrossing.h>
#include <netedit/elements/network/GNEEdgeType.h>
#include <netedit/elements/network/GNEEdgeTemplate.h>
#include <netedit/elements/network/GNELaneType.h>
#include <netedit/elements/data/GNEMeanData.h>
#include <netedit/frames/common/GNEInspectorFrame.h>
#include <netwrite/NWFrame.h>
#include <netwrite/NWWriter_SUMO.h>
#include <netwrite/NWWriter_XML.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/gui/div/GUIParameterTableWindow.h>
#include <utils/gui/globjects/GUIGLObjectPopupMenu.h>
#include <utils/gui/globjects/GUIGlObjectStorage.h>
#include <utils/options/OptionsCont.h>
#include <utils/gui/div/GLHelper.h>

#include "GNEApplicationWindow.h"
#include "GNENet.h"
#include "GNEViewNet.h"
#include "GNEUndoList.h"
#include "GNEViewParent.h"


// ===========================================================================
// FOX callback mapping
// ===========================================================================

FXIMPLEMENT_ABSTRACT(GNENetHelper::GNEChange_ReplaceEdgeInTLS, GNEChange, nullptr, 0)

// ===========================================================================
// static members
// ===========================================================================

const double GNENet::Z_INITIALIZED = 1;
const std::map<SumoXMLAttr, std::string> GNENet::EMPTY_HEADER;

// ===========================================================================
// member method definitions
// ===========================================================================

GNENet::GNENet(NBNetBuilder* netBuilder) :
    GUIGlObject(GLO_NETWORK, "", nullptr),
    myNetBuilder(netBuilder),
    myAttributeCarriers(new GNENetHelper::AttributeCarriers(this)),
    mySavingStatus(new GNENetHelper::SavingStatus()),
    myPathManager(new GNEPathManager(this)) { // TODO a little dangerous to use "this" here, it makes access to the net and the netBuilder
    // set net in gIDStorage
    GUIGlObjectStorage::gIDStorage.setNetObject(this);
    // Write GL debug information
    WRITE_GLDEBUG("initJunctionsAndEdges function called in GNENet constructor");
    // init junction and edges
    initJunctionsAndEdges();
    // check Z boundary
    if (myZBoundary.ymin() != Z_INITIALIZED) {
        myZBoundary.add(0, 0);
    }

}


GNENet::~GNENet() {
    // delete path manager
    delete myPathManager;
    // delete AttributeCarriers
    delete myAttributeCarriers;
    // delete saving status
    delete mySavingStatus;
    // show extra information for tests
    WRITE_DEBUG("Deleting net builder in GNENet destructor");
    delete myNetBuilder;
}


GNENetHelper::AttributeCarriers*
GNENet::getAttributeCarriers() const {
    return myAttributeCarriers;
}


GNENetHelper::SavingStatus*
GNENet::getSavingStatus() const {
    return mySavingStatus;
}


GNEPathManager*
GNENet::getPathManager() {
    return myPathManager;
}


const Boundary&
GNENet::getBoundary() const {
    // SUMORTree is also a Boundary
    return myGrid;
}


SUMORTree&
GNENet::getGrid() {
    return myGrid;
}

const std::map<std::string, int>&
GNENet::getEdgesAndNumberOfLanes() const {
    return myEdgesAndNumberOfLanes;
}


GUIGLObjectPopupMenu*
GNENet::getPopUpMenu(GUIMainWindow& app, GUISUMOAbstractView& parent) {
    GUIGLObjectPopupMenu* ret = new GUIGLObjectPopupMenu(app, parent, *this);
    buildPopupHeader(ret, app);
    buildCenterPopupEntry(ret);
    buildPositionCopyEntry(ret, app);
    if (GeoConvHelper::getFinal().usingGeoProjection()) {
        GUIDesigns::buildFXMenuCommand(ret, TL("Copy view geo-boundary to clipboard"), nullptr, ret, MID_COPY_VIEW_GEOBOUNDARY);
    }
    return ret;
}


GUIParameterTableWindow*
GNENet::getParameterWindow(GUIMainWindow& app, GUISUMOAbstractView&) {
    // Nets lanes don't have attributes
    GUIParameterTableWindow* ret = new GUIParameterTableWindow(app, *this);
    // close building
    ret->closeBuilding();
    return ret;
}


void
GNENet::drawGL(const GUIVisualizationSettings& s) const {
    // draw boundaries
    GLHelper::drawBoundary(s, getCenteringBoundary());
}


Boundary
GNENet::getCenteringBoundary() const {
    return getBoundary();
}


void
GNENet::expandBoundary(const Boundary& newBoundary) {
    myGrid.add(newBoundary);
}


const Boundary&
GNENet::getZBoundary() const {
    return myZBoundary;
}


void
GNENet::addZValueInBoundary(const double z) {
    // @todo let Boundary class track z-coordinate natively
    if (z != 0) {
        myZBoundary.add(z, Z_INITIALIZED);
    }
}


GNEJunction*
GNENet::createJunction(const Position& pos, GNEUndoList* undoList) {
    // get junction prefix
    const std::string junctionPrefix = OptionsCont::getOptions().getString("prefix") + OptionsCont::getOptions().getString("node-prefix");
    // generate new ID
    while (myAttributeCarriers->getJunctions().count(junctionPrefix + toString(myJunctionIDCounter)) != 0) {
        myJunctionIDCounter++;
    }
    // create new NBNode
    NBNode* nbn = new NBNode(junctionPrefix + toString(myJunctionIDCounter), pos);
    GNEJunction* junction = new GNEJunction(this, nbn);
    undoList->add(new GNEChange_Junction(junction, true), true);
    return junction;
}


GNEEdge*
GNENet::createEdge(GNEJunction* src, GNEJunction* dest, GNEEdge* edgeTemplate, GNEUndoList* undoList,
                   const std::string& suggestedName, bool wasSplit, bool allowDuplicateGeom, bool recomputeConnections) {
    // get edge prefix
    const std::string edgePrefix = OptionsCont::getOptions().getString("prefix") + OptionsCont::getOptions().getString("edge-prefix");
    // get edge infix
    std::string edgeInfix = OptionsCont::getOptions().getString("edge-infix");
    // prevent duplicate edge (same geometry)
    for (const auto& outgoingEdge : src->getNBNode()->getOutgoingEdges()) {
        if (outgoingEdge->getToNode() == dest->getNBNode() && outgoingEdge->getGeometry().size() == 2) {
            if (!allowDuplicateGeom) {
                return nullptr;
            }
        }
    }
    // check if exist opposite edge
    const auto oppositeEdges = myAttributeCarriers->retrieveEdges(dest, src);
    // declare edge id
    std::string edgeID;
    // update id
    if (oppositeEdges.size() > 0) {
        // avoid ids with "--..."
        if ((oppositeEdges.front()->getID().size() > 1) && (oppositeEdges.front()->getID().front() == '-')) {
            edgeID = oppositeEdges.front()->getID().substr(1);
        } else {
            edgeID = "-" + oppositeEdges.front()->getID();
        }
        // check if already exist an edge with edgeID
        if (myAttributeCarriers->getEdges().count(edgeID) > 0) {
            int counter = 0;
            // generate new ID using edgeID and counter
            while (myAttributeCarriers->getEdges().count(edgeID + toString(counter)) > 0) {
                counter++;
            }
            edgeID = edgeID + toString(counter);
        }
    } else if ((suggestedName.size() > 0) && (myAttributeCarriers->retrieveEdge(suggestedName, false) == nullptr)) {
        edgeID = suggestedName;
    } else if (edgeInfix.size() > 0) {
        // permit empty infix by setting it to <SPACE>
        edgeInfix = StringUtils::trim(edgeInfix);
        // check if exist edge with id <fromNodeID><infix><toNodeID>
        if (myAttributeCarriers->getEdges().count(src->getID() + edgeInfix + dest->getID()) == 0) {
            edgeID = src->getID() + edgeInfix + dest->getID();
        } else {
            int counter = 0;
            // generate new ID using edgeInfix and counter
            while (myAttributeCarriers->getEdges().count(src->getID() + edgeInfix + toString(counter) + dest->getID()) != 0) {
                myEdgeIDCounter++;
            }
            edgeID = src->getID() + edgeInfix + toString(counter) + dest->getID();
        }
    } else {
        // generate new ID
        while (myAttributeCarriers->getEdges().count(edgePrefix + toString(myEdgeIDCounter)) != 0) {
            myEdgeIDCounter++;
        }
        edgeID = edgePrefix + toString(myEdgeIDCounter);
    }
    GNEEdge* edge;
    // check if there is a template edge
    if (edgeTemplate) {
        // create NBEdgeTemplate
        NBEdge* nbe = new NBEdge(edgeID, src->getNBNode(), dest->getNBNode(), edgeTemplate->getNBEdge());
        edge = new GNEEdge(this, nbe, wasSplit);
    } else {
        // default if no template is given
        const auto& neteditOptions = OptionsCont::getOptions();
        double defaultSpeed = neteditOptions.getFloat("default.speed");
        const std::string defaultType = neteditOptions.getString("default.type");
        const int defaultNrLanes = neteditOptions.getInt("default.lanenumber");
        const int defaultPriority = neteditOptions.getInt("default.priority");
        const double defaultWidth = NBEdge::UNSPECIFIED_WIDTH;
        const double defaultOffset = NBEdge::UNSPECIFIED_OFFSET;
        const LaneSpreadFunction spread = LaneSpreadFunction::RIGHT;
        // build NBEdge
        NBEdge* nbe = new NBEdge(edgeID, src->getNBNode(), dest->getNBNode(),
                                 defaultType, defaultSpeed, NBEdge::UNSPECIFIED_FRICTION,
                                 defaultNrLanes, defaultPriority,
                                 defaultWidth, defaultOffset, spread);
        // create edge
        edge = new GNEEdge(this, nbe, wasSplit);
    }
    // add edge using undo list
    undoList->begin(edge, TL("create edge"));
    undoList->add(new GNEChange_Edge(edge, true), true);
    // recompute connection
    if (recomputeConnections) {
        src->setLogicValid(false, undoList);
        dest->setLogicValid(false, undoList);
    }
    requireRecompute();
    undoList->end();
    return edge;
}


void
GNENet::deleteNetworkElement(GNENetworkElement* networkElement, GNEUndoList* undoList) {
    if (networkElement->getTagProperty().getTag() == SUMO_TAG_JUNCTION) {
        // get junction (note: could be already removed if is a child, then hardfail=false)
        GNEJunction* junction = myAttributeCarriers->retrieveJunction(networkElement->getID(), false);
        // if exist, remove it
        if (junction) {
            deleteJunction(junction, undoList);
        }
    } else if (networkElement->getTagProperty().getTag() == SUMO_TAG_CROSSING) {
        // get crossing (note: could be already removed if is a child, then hardfail=false)
        GNECrossing* crossing = myAttributeCarriers->retrieveCrossing(networkElement->getGUIGlObject(), false);
        // if exist, remove it
        if (crossing) {
            deleteCrossing(crossing, undoList);
        }
    } else if (networkElement->getTagProperty().getTag() == SUMO_TAG_EDGE) {
        // get edge (note: could be already removed if is a child, then hardfail=false)
        GNEEdge* edge = myAttributeCarriers->retrieveEdge(networkElement->getID(), false);
        // if exist, remove it
        if (edge) {
            deleteEdge(edge, undoList, false);
        }
    } else if (networkElement->getTagProperty().getTag() == SUMO_TAG_LANE) {
        // get lane (note: could be already removed if is a child, then hardfail=false)
        GNELane* lane = myAttributeCarriers->retrieveLane(networkElement->getGUIGlObject(), false);
        // if exist, remove it
        if (lane) {
            deleteLane(lane, undoList, false);
        }
    } else if (networkElement->getTagProperty().getTag() == SUMO_TAG_CONNECTION) {
        // get connection (note: could be already removed if is a child, then hardfail=false)
        GNEConnection* connection = myAttributeCarriers->retrieveConnection(networkElement->getGUIGlObject(), false);
        // if exist, remove it
        if (connection) {
            deleteConnection(connection, undoList);
        }
    }
}


void
GNENet::deleteJunction(GNEJunction* junction, GNEUndoList* undoList) {
    // we have to delete all incident edges because they cannot exist without that junction
    // all deletions must be undone/redone together so we start a new command group
    // @todo if any of those edges are dead-ends should we remove their orphan junctions as well?
    undoList->begin(GUIIcon::MODEDELETE, TL("delete junction"));
    // invalidate junction path elements
    myPathManager->invalidateJunctionPath(junction);
    // delete junction child demand elements
    while (junction->getChildDemandElements().size() > 0) {
        deleteDemandElement(junction->getChildDemandElements().front(), undoList);
    }
    // delete all crossings vinculated with junction
    while (junction->getGNECrossings().size() > 0) {
        deleteCrossing(junction->getGNECrossings().front(), undoList);
    }
    // find all crossings of neighbour junctions that shares an edge of this junction
    std::vector<GNECrossing*> crossingsToRemove;
    std::vector<GNEJunction*> junctionNeighbours = junction->getJunctionNeighbours();
    for (const auto& junctionNeighbour : junctionNeighbours) {
        // iterate over crossing of neighbour junction
        for (const auto& crossing : junctionNeighbour->getGNECrossings()) {
            // if at least one of the edges of junction to remove belongs to a crossing of the neighbour junction, delete it
            if (crossing->checkEdgeBelong(junctionNeighbour->getChildEdges())) {
                crossingsToRemove.push_back(crossing);
            }
        }
    }
    // delete collected crossings
    for (const auto& crossing : crossingsToRemove) {
        deleteCrossing(crossing, undoList);
    }
    // deleting edges changes in the underlying EdgeVector so we have to make a copy
    const EdgeVector incidentEdges = junction->getNBNode()->getEdges();
    for (const auto& edge : incidentEdges) {
        deleteEdge(myAttributeCarriers->getEdges().at(edge->getID()), undoList, true);
    }
    // remove any traffic lights from the traffic light container (avoids lots of warnings)
    junction->setAttribute(SUMO_ATTR_TYPE, toString(SumoXMLNodeType::PRIORITY), undoList);
    // delete edge
    undoList->add(new GNEChange_Junction(junction, false), true);
    undoList->end();
}


void
GNENet::deleteEdge(GNEEdge* edge, GNEUndoList* undoList, bool recomputeConnections) {
    undoList->begin(GUIIcon::MODEDELETE, TL("delete edge"));
    // iterate over lanes
    for (const auto& lane : edge->getLanes()) {
        // invalidate lane path elements
        myPathManager->invalidateLanePath(lane);
        // delete lane additionals
        while (lane->getChildAdditionals().size() > 0) {
            deleteAdditional(lane->getChildAdditionals().front(), undoList);
        }
        // delete lane demand elements
        while (lane->getChildDemandElements().size() > 0) {
            deleteDemandElement(lane->getChildDemandElements().front(), undoList);
        }
        // delete lane generic data elements
        while (lane->getChildGenericDatas().size() > 0) {
            deleteGenericData(lane->getChildGenericDatas().front(), undoList);
        }
    }
    // delete edge child additionals
    while (edge->getChildAdditionals().size() > 0) {
        deleteAdditional(edge->getChildAdditionals().front(), undoList);
    }
    // delete edge child demand elements
    while (edge->getChildDemandElements().size() > 0) {
        // special case for embedded routes
        if (edge->getChildDemandElements().front()->getTagProperty().getTag() == GNE_TAG_ROUTE_EMBEDDED) {
            deleteDemandElement(edge->getChildDemandElements().front()->getParentDemandElements().front(), undoList);
        } else if (edge->getChildDemandElements().front()->getTagProperty().isPlan()) {
            const auto planParent = edge->getChildDemandElements().front()->getParentDemandElements().front();
            // if this is the last person child, remove plan parent (person/container) instead plan element
            if (planParent->getChildDemandElements().size() == 1) {
                deleteDemandElement(planParent, undoList);
            } else {
                deleteDemandElement(edge->getChildDemandElements().front(), undoList);
            }
        } else {
            deleteDemandElement(edge->getChildDemandElements().front(), undoList);
        }
    }
    // delete edge child generic datas
    while (edge->getChildGenericDatas().size() > 0) {
        deleteGenericData(edge->getChildGenericDatas().front(), undoList);
    }
    // remove edge from crossings related with this edge
    edge->getFromJunction()->removeEdgeFromCrossings(edge, undoList);
    edge->getToJunction()->removeEdgeFromCrossings(edge, undoList);
    // update affected connections
    if (recomputeConnections) {
        edge->getFromJunction()->setLogicValid(false, undoList);
        edge->getToJunction()->setLogicValid(false, undoList);
    } else {
        edge->getFromJunction()->removeConnectionsTo(edge, undoList, true);
        edge->getToJunction()->removeConnectionsFrom(edge, undoList, true);
    }
    // if junction source is a TLS and after deletion will have only an edge, remove TLS
    if (edge->getFromJunction()->getNBNode()->isTLControlled() && (edge->getFromJunction()->getGNEOutgoingEdges().size() <= 1)) {
        edge->getFromJunction()->setAttribute(SUMO_ATTR_TYPE, toString(SumoXMLNodeType::PRIORITY), undoList);
    }
    // if junction destination is a TLS and after deletion will have only an edge, remove TLS
    if (edge->getToJunction()->getNBNode()->isTLControlled() && (edge->getToJunction()->getGNEIncomingEdges().size() <= 1)) {
        edge->getToJunction()->setAttribute(SUMO_ATTR_TYPE, toString(SumoXMLNodeType::PRIORITY), undoList);
    }
    // Delete edge
    undoList->add(new GNEChange_Edge(edge, false), true);
    // remove edge requires always a recompute (due geometry and connections)
    requireRecompute();
    // finish delete edge
    undoList->end();
}


void
GNENet::replaceIncomingEdge(GNEEdge* which, GNEEdge* by, GNEUndoList* undoList) {
    undoList->begin(which, TL("replace edge"));
    GNEChange_Attribute::changeAttribute(by, SUMO_ATTR_TO, which->getAttribute(SUMO_ATTR_TO), undoList);
    // iterate over lane
    for (const auto& lane : which->getLanes()) {
        // replace in additionals
        std::vector<GNEAdditional*> copyOfLaneAdditionals = lane->getChildAdditionals();
        for (const auto& additional : copyOfLaneAdditionals) {
            GNEChange_Attribute::changeAttribute(additional, SUMO_ATTR_LANE, by->getNBEdge()->getLaneID(lane->getIndex()), undoList);
            if (additional->hasAttribute(SUMO_ATTR_STARTPOS)) {
                GNEChange_Attribute::changeAttribute(additional, SUMO_ATTR_STARTPOS,
                                                     toString(StringUtils::toDouble(additional->getAttribute(SUMO_ATTR_STARTPOS)) + which->getNBEdge()->getFinalLength()),
                                                     undoList);
            }
            if (additional->hasAttribute(SUMO_ATTR_ENDPOS)) {
                GNEChange_Attribute::changeAttribute(additional, SUMO_ATTR_ENDPOS,
                                                     toString(StringUtils::toDouble(additional->getAttribute(SUMO_ATTR_ENDPOS)) + which->getNBEdge()->getFinalLength()),
                                                     undoList);
            }
        }
        // replace in demand elements
        std::vector<GNEDemandElement*> copyOfLaneDemandElements = lane->getChildDemandElements();
        for (const auto& demandElement : copyOfLaneDemandElements) {
            GNEChange_Attribute::changeAttribute(demandElement, SUMO_ATTR_LANE, by->getNBEdge()->getLaneID(lane->getIndex()), undoList);
        }
        // replace in generic datas
        std::vector<GNEGenericData*> copyOfLaneGenericDatas = lane->getChildGenericDatas();
        for (const auto& demandElement : copyOfLaneGenericDatas) {
            GNEChange_Attribute::changeAttribute(demandElement, SUMO_ATTR_LANE, by->getNBEdge()->getLaneID(lane->getIndex()), undoList);
        }
    }
    // replace in edge additionals children
    std::vector<GNEAdditional*> addElements = which->getChildAdditionals();
    for (GNEAdditional* add : addElements) {
        if (add->hasAttribute(SUMO_ATTR_EDGE)) {
            GNEChange_Attribute::changeAttribute(add, SUMO_ATTR_EDGE, by->getID(), undoList);
        }
    }
    // replace in edge demand elements children
    const std::vector<GNEDemandElement*> demandElements = which->getChildDemandElements();
    for (GNEDemandElement* demandElement : demandElements) {
        if (demandElement->hasAttribute(SUMO_ATTR_EDGE)) {
            GNEChange_Attribute::changeAttribute(demandElement, SUMO_ATTR_EDGE, by->getID(), undoList);
        }
        if (demandElement->hasAttribute(SUMO_ATTR_EDGES)) {
            replaceInListAttribute(demandElement, SUMO_ATTR_EDGES, which->getID(), by->getID(), undoList);
        }
        if (demandElement->hasAttribute(SUMO_ATTR_VIA)) {
            replaceInListAttribute(demandElement, SUMO_ATTR_VIA, which->getID(), by->getID(), undoList);
        }
        if (demandElement->hasAttribute(SUMO_ATTR_FROM) && demandElement->getAttribute(SUMO_ATTR_FROM) == which->getID()) {
            GNEChange_Attribute::changeAttribute(demandElement, SUMO_ATTR_FROM, by->getID(), undoList);
        }
        if (demandElement->hasAttribute(SUMO_ATTR_TO) && demandElement->getAttribute(SUMO_ATTR_TO) == which->getID()) {
            GNEChange_Attribute::changeAttribute(demandElement, SUMO_ATTR_TO, by->getID(), undoList);
        }
    }
    // replace in data
    const std::vector<GNEGenericData*> dataElements = which->getChildGenericDatas();
    for (GNEGenericData* dataElement : dataElements) {
        if (dataElement->hasAttribute(SUMO_ATTR_EDGE)) {
            GNEChange_Attribute::changeAttribute(dataElement, SUMO_ATTR_EDGE, by->getID(), undoList);
        }
    }
    // replace in rerouters
    addElements = which->getParentAdditionals();
    for (GNEAdditional* add : addElements) {
        if (add->hasAttribute(SUMO_ATTR_EDGES)) {
            replaceInListAttribute(add, SUMO_ATTR_EDGES, which->getID(), by->getID(), undoList);
        }
    }
    // replace in crossings
    for (const auto& crossing : which->getToJunction()->getGNECrossings()) {
        // if at least one of the edges of junction to remove belongs to a crossing of the source junction, delete it
        replaceInListAttribute(crossing, SUMO_ATTR_EDGES, which->getID(), by->getID(), undoList);
    }
    // fix connections (make a copy because they will be modified
    std::vector<NBEdge::Connection> NBConnections = which->getNBEdge()->getConnections();
    for (const auto& NBConnection : NBConnections) {
        if (NBConnection.toEdge != nullptr) {
            undoList->add(new GNEChange_Connection(which, NBConnection, false, false), true);
            undoList->add(new GNEChange_Connection(by, NBConnection, false, true), true);
        }
    }
    undoList->add(new GNENetHelper::GNEChange_ReplaceEdgeInTLS(getTLLogicCont(), which->getNBEdge(), by->getNBEdge()), true);
    // Delete edge
    undoList->add(new GNEChange_Edge(which, false), true);
    // finish replace edge
    undoList->end();
}


void
GNENet::deleteLane(GNELane* lane, GNEUndoList* undoList, bool recomputeConnections) {
    GNEEdge* edge = lane->getParentEdge();
    if (edge->getNBEdge()->getNumLanes() == 1) {
        // remove the whole edge instead
        deleteEdge(edge, undoList, recomputeConnections);
    } else {
        undoList->begin(GUIIcon::MODEDELETE, TL("delete lane"));
        // invalidate lane path elements
        myPathManager->invalidateLanePath(lane);
        // delete lane additional children
        while (lane->getChildAdditionals().size() > 0) {
            deleteAdditional(lane->getChildAdditionals().front(), undoList);
        }
        // delete lane demand element children
        while (lane->getChildDemandElements().size() > 0) {
            deleteDemandElement(lane->getChildDemandElements().front(), undoList);
        }
        // delete lane generic data children
        while (lane->getChildGenericDatas().size() > 0) {
            deleteGenericData(lane->getChildGenericDatas().front(), undoList);
        }
        // update affected connections
        if (recomputeConnections) {
            edge->getFromJunction()->setLogicValid(false, undoList);
            edge->getToJunction()->setLogicValid(false, undoList);
        } else {
            edge->getFromJunction()->removeConnectionsTo(edge, undoList, true, lane->getIndex());
            edge->getToJunction()->removeConnectionsFrom(edge, undoList, true, lane->getIndex());
        }
        // delete lane
        const NBEdge::Lane& laneAttrs = edge->getNBEdge()->getLaneStruct(lane->getIndex());
        undoList->add(new GNEChange_Lane(edge, lane, laneAttrs, false, recomputeConnections), true);
        // remove lane requires always a recompute (due geometry and connections)
        requireRecompute();
        undoList->end();
    }
}


void
GNENet::deleteConnection(GNEConnection* connection, GNEUndoList* undoList) {
    undoList->begin(GUIIcon::MODEDELETE, TL("delete connection"));
    // obtain NBConnection to remove
    NBConnection deleted = connection->getNBConnection();
    GNEJunction* junctionDestination = connection->getEdgeFrom()->getToJunction();
    junctionDestination->markAsModified(undoList);
    undoList->add(new GNEChange_Connection(connection->getEdgeFrom(), connection->getNBEdgeConnection(), connection->isAttributeCarrierSelected(), false), true);
    junctionDestination->invalidateTLS(undoList, deleted);
    // remove connection requires always a recompute (due geometry and connections)
    requireRecompute();
    undoList->end();
}


void
GNENet::deleteCrossing(GNECrossing* crossing, GNEUndoList* undoList) {
    undoList->begin(GUIIcon::MODEDELETE, TL("delete crossing"));
    // remove it using GNEChange_Crossing
    undoList->add(new GNEChange_Crossing(
                      crossing->getParentJunction(), crossing->getNBCrossing()->edges,
                      crossing->getNBCrossing()->width, crossing->getNBCrossing()->priority,
                      crossing->getNBCrossing()->customTLIndex,
                      crossing->getNBCrossing()->customTLIndex2,
                      crossing->getNBCrossing()->customShape,
                      crossing->isAttributeCarrierSelected(),
                      false), true);
    // remove crossing requires always a recompute (due geometry and connections)
    requireRecompute();
    undoList->end();
}


void
GNENet::deleteAdditional(GNEAdditional* additional, GNEUndoList* undoList) {
    undoList->begin(GUIIcon::MODEDELETE, TL("delete ") + additional->getTagStr());
    // remove all demand element children of this additional deleteDemandElement this function recursively
    while (additional->getChildDemandElements().size() > 0) {
        deleteDemandElement(additional->getChildDemandElements().front(), undoList);
    }
    // remove all generic data children of this additional deleteGenericData this function recursively
    while (additional->getChildGenericDatas().size() > 0) {
        deleteGenericData(additional->getChildGenericDatas().front(), undoList);
    }
    // remove all additional children of this additional calling this function recursively
    while (additional->getChildAdditionals().size() > 0) {
        deleteAdditional(additional->getChildAdditionals().front(), undoList);
    }
    // remove additional
    undoList->add(new GNEChange_Additional(additional, false), true);
    undoList->end();
}


void
GNENet::deleteDemandElement(GNEDemandElement* demandElement, GNEUndoList* undoList) {
    // check that default VTypes aren't removed
    if ((demandElement->getTagProperty().getTag() == SUMO_TAG_VTYPE) && (GNEAttributeCarrier::parse<bool>(demandElement->getAttribute(GNE_ATTR_DEFAULT_VTYPE)))) {
        throw ProcessError(TL("Trying to delete a default Vehicle Type"));
    } else {
        // check if currently is being inspected
        if (myViewNet->isAttributeCarrierInspected(demandElement)) {
            myViewNet->getViewParent()->getInspectorFrame()->clearInspectedAC();
        }
        undoList->begin(GUIIcon::MODEDELETE, TL("delete ") + demandElement->getTagStr());
        // remove all child additional elements of this demandElement calling this function recursively
        while (demandElement->getChildAdditionals().size() > 0) {
            deleteAdditional(demandElement->getChildAdditionals().front(), undoList);
        }
        // remove all child demand elements of this demandElement calling this function recursively
        while (demandElement->getChildDemandElements().size() > 0) {
            deleteDemandElement(demandElement->getChildDemandElements().front(), undoList);
        }
        // remove all generic data children of this additional deleteGenericData this function recursively
        while (demandElement->getChildGenericDatas().size() > 0) {
            deleteGenericData(demandElement->getChildGenericDatas().front(), undoList);
        }
        // remove demandElement
        undoList->add(new GNEChange_DemandElement(demandElement, false), true);
        undoList->end();
    }
}


void
GNENet::deleteDataSet(GNEDataSet* dataSet, GNEUndoList* undoList) {
    undoList->begin(GUIIcon::MODEDELETE, TL("delete ") + dataSet->getTagStr());
    // make a copy of all generic data children
    auto copyOfDataIntervalChildren = dataSet->getDataIntervalChildren();
    // clear all data intervals (this will be delete also the dataSet)
    for (const auto& dataInterval : copyOfDataIntervalChildren) {
        deleteDataInterval(dataInterval.second, undoList);
    }
    undoList->end();
}


void
GNENet::deleteDataInterval(GNEDataInterval* dataInterval, GNEUndoList* undoList) {
    undoList->begin(GUIIcon::MODEDELETE, TL("delete ") + dataInterval->getTagStr());
    // make a copy of all generic data children
    auto copyOfGenericDataChildren = dataInterval->getGenericDataChildren();
    // clear all generic datas (this will be delete also the data intervals)
    for (const auto& genericData : copyOfGenericDataChildren) {
        deleteGenericData(genericData, undoList);
    }
    undoList->end();
}


void
GNENet::deleteGenericData(GNEGenericData* genericData, GNEUndoList* undoList) {
    undoList->begin(GUIIcon::MODEDELETE, TL("delete ") + genericData->getTagStr());
    // remove all child demand elements of this demandElement calling this function recursively
    while (genericData->getChildDemandElements().size() > 0) {
        deleteDemandElement(genericData->getChildDemandElements().front(), undoList);
    }
    // remove all generic data children of this additional deleteGenericData this function recursively
    while (genericData->getChildGenericDatas().size() > 0) {
        deleteGenericData(genericData->getChildGenericDatas().front(), undoList);
    }
    // get pointer to dataInterval and dataSet
    GNEDataInterval* dataInterval = genericData->getDataIntervalParent();
    GNEDataSet* dataSet = dataInterval->getDataSetParent();
    // remove generic data
    undoList->add(new GNEChange_GenericData(genericData, false), true);
    // check if data interval is empty
    if (dataInterval->getGenericDataChildren().empty()) {
        // remove data interval
        undoList->add(new GNEChange_DataInterval(genericData->getDataIntervalParent(), false), true);
        // now check if data set is empty
        if (dataSet->getDataIntervalChildren().empty()) {
            // remove data set
            undoList->add(new GNEChange_DataSet(genericData->getDataIntervalParent()->getDataSetParent(), false), true);
        }
    }
    undoList->end();
}


void
GNENet::deleteMeanData(GNEMeanData* meanData, GNEUndoList* undoList) {
    undoList->begin(GUIIcon::MODEDELETE, TL("delete ") + meanData->getTagStr());
    // remove mean data
    undoList->add(new GNEChange_MeanData(meanData, false), true);
    undoList->end();
}


void
GNENet::duplicateLane(GNELane* lane, GNEUndoList* undoList, bool recomputeConnections) {
    undoList->begin(lane, TL("duplicate lane"));
    GNEEdge* edge = lane->getParentEdge();
    const NBEdge::Lane& laneAttrs = edge->getNBEdge()->getLaneStruct(lane->getIndex());
    if (recomputeConnections) {
        edge->getFromJunction()->setLogicValid(false, undoList);
        edge->getToJunction()->setLogicValid(false, undoList);
    }
    GNELane* newLane = new GNELane(edge, lane->getIndex());
    undoList->add(new GNEChange_Lane(edge, newLane, laneAttrs, true, recomputeConnections), true);
    requireRecompute();
    undoList->end();
}


bool
GNENet::restrictLane(SUMOVehicleClass vclass, GNELane* lane, GNEUndoList* undoList) {
    bool addRestriction = true;
    if (vclass == SVC_PEDESTRIAN) {
        GNEEdge* edge = lane->getParentEdge();
        for (const auto& edgeLane : edge->getLanes()) {
            if (edgeLane->isRestricted(SVC_PEDESTRIAN)) {
                // prevent adding a 2nd sidewalk
                addRestriction = false;
            } else {
                // ensure that the sidewalk is used exclusively
                const SVCPermissions allOldWithoutPeds = edge->getNBEdge()->getPermissions(edgeLane->getIndex()) & ~SVC_PEDESTRIAN;
                edgeLane->setAttribute(SUMO_ATTR_ALLOW, getVehicleClassNames(allOldWithoutPeds), undoList);
            }
        }
    }
    // restrict the lane
    if (addRestriction) {
        double width;
        if (vclass == SVC_PEDESTRIAN) {
            width = OptionsCont::getOptions().getFloat("default.sidewalk-width");
        } else if (vclass == SVC_BICYCLE) {
            width = OptionsCont::getOptions().getFloat("default.bikelane-width");
        } else {
            width = OptionsCont::getOptions().getFloat("default.lanewidth");
        }
        lane->setAttribute(SUMO_ATTR_ALLOW, toString(vclass), undoList);
        lane->setAttribute(SUMO_ATTR_WIDTH, toString(width), undoList);
        if ((vclass & ~SVC_PEDESTRIAN) == 0) {
            std::vector<GNEConnection*> cons;
            const bool reguess = lane->getParentEdge()->getNBEdge()->getStep() <= NBEdge::EdgeBuildingStep::LANES2LANES_RECHECK;
            if (reguess) {
                // remove all connections and rebuild from scratch
                cons = lane->getParentEdge()->getGNEConnections();
            } else {
                // remove connections that have become invalid (pedestrians are
                // connected via walkingareas somewhere else)
                cons = lane->getGNEOutcomingConnections();
            }
            for (auto c : cons) {
                undoList->add(new GNEChange_Connection(lane->getParentEdge(), c->getNBEdgeConnection(), false, false), true);
            }
            if (reguess) {
                GNEChange_Attribute::changeAttribute(lane->getParentEdge(), GNE_ATTR_MODIFICATION_STATUS, GNEAttributeCarrier::FEATURE_GUESSED, undoList, true);
            }
        }
        return true;
    } else {
        return false;
    }
}


bool
GNENet::addRestrictedLane(SUMOVehicleClass vclass, GNEEdge* edge, int index, GNEUndoList* undoList) {
    // First check that edge don't have a restricted lane of the given vclass
    for (const auto& lane : edge->getLanes()) {
        if (lane->isRestricted(vclass)) {
            return false;
        }
    }
    // check that index is correct (index == size adds to the left of the leftmost lane)
    const int numLanes = (int)edge->getLanes().size();
    if (index > numLanes) {
        return false;
    }
    if (index < 0) {
        // for pedestrians and greenVerge, always index 0
        index = 0;
        // guess index from vclass
        if (vclass == SVC_BICYCLE) {
            // add bikelanes to the left of an existing sidewalk
            index = edge->getLanes()[0]->isRestricted(SVC_PEDESTRIAN) ? 1 : 0;
        } else if (vclass == SVC_BUS) {
            // add greenVerge to the left of an existing sidewalk or bikeLane
            // add busLane to the left of an existing sidewalk, bikeLane or greenVerge
            index = 0;
            while (index < numLanes && (edge->getNBEdge()->getPermissions(index) & ~(SVC_PEDESTRIAN | SVC_BICYCLE)) == 0) {
                index++;
            }
        }
    }
    // duplicate selected lane
    duplicateLane(edge->getLanes().at(MIN2(index, numLanes - 1)), undoList, true);
    // transform the created lane
    return restrictLane(vclass, edge->getLanes().at(index), undoList);
}


bool
GNENet::addGreenVergeLane(GNEEdge* edge, int index, GNEUndoList* undoList) {
    // check that index is correct (index == size adds to the left of the leftmost lane)
    const int numLanes = (int)edge->getLanes().size();
    if (index > numLanes) {
        index = numLanes;
    }
    if (index < 0) {
        index = 0;
    }
    // duplicate selected lane
    duplicateLane(edge->getLanes().at(MIN2(index, numLanes - 1)), undoList, true);
    // transform the created lane
    return restrictLane(SVC_IGNORING, edge->getLanes().at(index), undoList);
}


bool
GNENet::removeRestrictedLane(SUMOVehicleClass vclass, GNEEdge* edge, GNEUndoList* undoList) {
    // iterate over lanes of edge
    for (const auto& lane : edge->getLanes()) {
        if (lane->isRestricted(vclass)) {
            // Delete lane
            deleteLane(lane, undoList, true);
            return true;
        }
    }
    return false;
}


std::pair<GNEJunction*, GNEEdge*>
GNENet::splitEdge(GNEEdge* edge, const Position& pos, GNEUndoList* undoList, GNEJunction* newJunction) {
    // begin undo list
    undoList->begin(edge, TL("split edge"));
    // check if we have to create a new edge
    if (newJunction == nullptr) {
        newJunction = createJunction(pos, undoList);
    }
    // obtain edge geometry and split position
    const PositionVector& oldEdgeGeometry = edge->getNBEdge()->getGeometry();
    const double edgeSplitPosition = oldEdgeGeometry.nearest_offset_to_point2D(pos, false);
    // obtain lane geometry and split position (needed for adjust additional and demand childs)
    const PositionVector& oldLaneGeometry = edge->getLanes().front()->getLaneShape();
    const double laneSplitPosition = oldLaneGeometry.nearest_offset_to_point2D(pos, false);
    // split edge geometry in two new geometries using edgeSplitPosition
    std::pair<PositionVector, PositionVector> newGeoms = oldEdgeGeometry.splitAt(edgeSplitPosition);
    const double oldLength = oldEdgeGeometry.length();
    const double relativeLength1 = oldLength != 0 ? newGeoms.first.length() / oldLength : 1;
    const double relativeLength2 = oldLength != 0 ? newGeoms.second.length() / oldLength : 1;
    // get shape end
    const std::string shapeEnd = edge->getAttribute(GNE_ATTR_SHAPE_END);
    // figure out the new name
    int posBase = 0;
    // set baseName
    std::string baseName = edge->getMicrosimID();
    if (edge->wasSplit()) {
        const std::string::size_type sep_index = baseName.rfind('.');
        // edge may have been renamed in between
        if (sep_index != std::string::npos) {
            std::string posString = baseName.substr(sep_index + 1);
            if (GNEAttributeCarrier::canParse<int>(posString.c_str())) {
                ;
                posBase = GNEAttributeCarrier::parse<int>(posString.c_str());
                baseName = baseName.substr(0, sep_index); // includes the .
            }
        }
    }
    baseName += '.';
    // create a new edge from the new junction to the previous destination
    GNEEdge* secondPart = createEdge(newJunction, edge->getToJunction(), edge,
                                     undoList, baseName + toString(posBase + (int)edgeSplitPosition), true, false, false);
    // fix connections from the split edge (must happen before changing SUMO_ATTR_TO)
    edge->getToJunction()->replaceIncomingConnections(edge, secondPart, undoList);
    // remove affected crossings from junction (must happen before changing SUMO_ATTR_TO)
    std::vector<NBNode::Crossing> affectedCrossings;
    for (GNECrossing* crossing : edge->getToJunction()->getGNECrossings()) {
        if (crossing->checkEdgeBelong(edge)) {
            NBNode::Crossing nbC = *crossing->getNBCrossing();
            undoList->add(new GNEChange_Crossing(edge->getToJunction(), nbC, false), true);
            EdgeVector newEdges;
            for (NBEdge* nbEdge : nbC.edges) {
                if (nbEdge == edge->getNBEdge()) {
                    newEdges.push_back(secondPart->getNBEdge());
                } else {
                    newEdges.push_back(nbEdge);
                }
            }
            nbC.edges = newEdges;
            affectedCrossings.push_back(nbC);
        }
    }
    // modify the edge so that it ends at the new junction (and all incoming connections are preserved
    GNEChange_Attribute::changeAttribute(edge, SUMO_ATTR_TO, newJunction->getID(), undoList);
    // set first part of geometry
    newGeoms.first.pop_back();
    newGeoms.first.erase(newGeoms.first.begin());
    edge->setAttribute(GNE_ATTR_SHAPE_END, "", undoList);
    edge->setAttribute(SUMO_ATTR_SHAPE, toString(newGeoms.first), undoList);
    // set second part of geometry
    secondPart->setAttribute(GNE_ATTR_SHAPE_END, shapeEnd, undoList);
    newGeoms.second.pop_back();
    newGeoms.second.erase(newGeoms.second.begin());
    secondPart->setAttribute(SUMO_ATTR_SHAPE, toString(newGeoms.second), undoList);
    // fix custom length
    if (edge->getNBEdge()->hasLoadedLength()) {
        // split in proportion to geometry lengths
        const double loadedLength = edge->getNBEdge()->getLoadedLength();
        edge->setAttribute(SUMO_ATTR_LENGTH, toString(relativeLength1 * loadedLength), undoList);
        secondPart->setAttribute(SUMO_ATTR_LENGTH, toString(relativeLength2 * loadedLength), undoList);
    }
    // reconnect across the split
    for (int i = 0; i < (int)edge->getLanes().size(); ++i) {
        undoList->add(new GNEChange_Connection(edge, NBEdge::Connection(i, secondPart->getNBEdge(), i), false, true), true);
    }
    // re-add modified crossings
    for (const auto& nbC : affectedCrossings) {
        undoList->add(new GNEChange_Crossing(secondPart->getToJunction(), nbC, true), true);
    }
    // Split geometry of all child additional
    auto childAdditionals = edge->getChildAdditionals();
    for (const auto& additional : childAdditionals) {
        additional->splitEdgeGeometry(edgeSplitPosition, edge, secondPart, undoList);
    }
    // Split geometry of all child lane additional
    for (int i = 0; i < (int)edge->getLanes().size(); i++) {
        for (const auto& additional : edge->getLanes().at(i)->getChildAdditionals()) {
            additional->splitEdgeGeometry(laneSplitPosition, edge->getLanes().at(i), secondPart->getLanes().at(i), undoList);
        }
    }
    // Split geometry of all child demand elements
    auto childDemandElements = edge->getChildDemandElements();
    for (const auto& demandElement : childDemandElements) {
        demandElement->splitEdgeGeometry(edgeSplitPosition, edge, secondPart, undoList);
    }
    // Split geometry of all child lane demand elements
    for (int i = 0; i < (int)edge->getLanes().size(); i++) {
        for (const auto& demandElement : edge->getLanes().at(i)->getChildDemandElements()) {
            demandElement->splitEdgeGeometry(laneSplitPosition, edge->getLanes().at(i), secondPart->getLanes().at(i), undoList);
        }
    }
    // finish undo list
    undoList->end();
    // return new junction
    return std::make_pair(newJunction, secondPart);
}


void
GNENet::splitEdgesBidi(GNEEdge* edge, GNEEdge* oppositeEdge, const Position& pos, GNEUndoList* undoList) {
    GNEJunction* newJunction = nullptr;
    undoList->begin(edge, TL("split edges"));
    // split edge and save created junction
    auto newStuff = splitEdge(edge, pos, undoList, newJunction);
    newJunction = newStuff.first;
    // split second edge
    splitEdge(oppositeEdge, pos, undoList, newJunction);
    if (edge->getLanes().back()->getAttribute(GNE_ATTR_OPPOSITE) != "") {
        // restore opposite lane information
        for (const auto& nbEdge : newJunction->getNBNode()->getEdges()) {
            GNEEdge* e = myAttributeCarriers->retrieveEdge(nbEdge->getID());
            // store old attribute before it's changed by guess opposite
            e->getLanes().back()->setAttribute(GNE_ATTR_OPPOSITE, "", undoList);
            if (nbEdge->guessOpposite(true)) {
                e->getLanes().back()->setAttribute(GNE_ATTR_OPPOSITE, nbEdge->getLanes().back().oppositeID, undoList);
            }
        }
    }
    undoList->end();
}


void
GNENet::reverseEdge(GNEEdge* edge, GNEUndoList* undoList) {
    undoList->begin(edge, TL("reverse edge"));
    deleteEdge(edge, undoList, false); // still exists. we delete it so we can reuse the name in case of resplit
    GNEEdge* reversed = createEdge(edge->getToJunction(), edge->getFromJunction(), edge, undoList, edge->getID(), false, true);
    assert(reversed != 0);
    reversed->setAttribute(SUMO_ATTR_SHAPE, toString(edge->getNBEdge()->getInnerGeometry().reverse()), undoList);
    reversed->setAttribute(GNE_ATTR_SHAPE_START, edge->getAttribute(GNE_ATTR_SHAPE_END), undoList);
    reversed->setAttribute(GNE_ATTR_SHAPE_END, edge->getAttribute(GNE_ATTR_SHAPE_START), undoList);
    undoList->end();
}


GNEEdge*
GNENet::addReversedEdge(GNEEdge* edge, const bool disconnected, GNEUndoList* undoList) {
    GNEEdge* reversed = edge->getReverseEdge();
    if (reversed != nullptr) {
        return reversed;
    }
    undoList->begin(edge, TL("add reversed edge"));
    if (!disconnected) {
        // for rail edges, we assume bi-directional tracks are wanted
        reversed = createEdge(edge->getToJunction(), edge->getFromJunction(), edge, undoList, "-" + edge->getID(), false, true);
        assert(reversed != 0);
        reversed->setAttribute(SUMO_ATTR_SHAPE, toString(edge->getNBEdge()->getInnerGeometry().reverse()), undoList);
        reversed->setAttribute(GNE_ATTR_SHAPE_START, edge->getAttribute(GNE_ATTR_SHAPE_END), undoList);
        reversed->setAttribute(GNE_ATTR_SHAPE_END, edge->getAttribute(GNE_ATTR_SHAPE_START), undoList);
    } else {
        // if the edge is centered it should probably connect somewhere else
        // make it easy to move and reconnect it
        PositionVector orig = edge->getNBEdge()->getGeometry();
        PositionVector origInner = edge->getNBEdge()->getInnerGeometry();
        const double tentativeShift = edge->getNBEdge()->getTotalWidth() + 2;
        orig.move2side(-tentativeShift);
        origInner.move2side(-tentativeShift);
        GNEJunction* src = createJunction(orig.back(), undoList);
        GNEJunction* dest = createJunction(orig.front(), undoList);
        reversed = createEdge(src, dest, edge, undoList, "-" + edge->getID(), false, true);
        assert(reversed != 0);
        reversed->setAttribute(SUMO_ATTR_SHAPE, toString(origInner.reverse()), undoList);
        reversed->setAttribute(SUMO_ATTR_SHAPE, toString(origInner.reverse()), undoList);
        // select the new edge and its nodes
        reversed->setAttribute(GNE_ATTR_SELECTED, "true", undoList);
        src->setAttribute(GNE_ATTR_SELECTED, "true", undoList);
        dest->setAttribute(GNE_ATTR_SELECTED, "true", undoList);
    }
    undoList->end();
    return reversed;
}


void
GNENet::mergeJunctions(GNEJunction* moved, GNEJunction* target, GNEUndoList* undoList) {
    undoList->begin(moved, TL("merge junctions"));
    // place moved junction in the same position of target junction
    moved->setAttribute(SUMO_ATTR_POSITION, target->getAttribute(SUMO_ATTR_POSITION), undoList);
    // deleting edges changes in the underlying EdgeVector so we have to make a copy
    const EdgeVector incomingNBEdges = moved->getNBNode()->getIncomingEdges();
    for (const auto& incomingNBEdge : incomingNBEdges) {
        // delete edges between the merged junctions
        GNEEdge* edge = myAttributeCarriers->getEdges().at(incomingNBEdge->getID());
        if (edge->getFromJunction() == target) {
            deleteEdge(edge, undoList, false);
        } else {
            GNEChange_Attribute::changeAttribute(edge, SUMO_ATTR_TO, target->getID(), undoList);
        }
    }
    // deleting edges changes in the underlying EdgeVector so we have to make a copy
    const EdgeVector outgoingNBEdges = moved->getNBNode()->getOutgoingEdges();
    for (const auto& outgoingNBEdge : outgoingNBEdges) {
        // delete edges between the merged junctions
        GNEEdge* edge = myAttributeCarriers->getEdges().at(outgoingNBEdge->getID());
        if (edge->getToJunction() == target) {
            deleteEdge(edge, undoList, false);
        } else {
            GNEChange_Attribute::changeAttribute(edge, SUMO_ATTR_FROM, target->getID(), undoList);
        }
    }
    // deleted moved junction
    deleteJunction(moved, undoList);
    undoList->end();
}


void
GNENet::selectRoundabout(GNEJunction* junction, GNEUndoList* undoList) {
    for (const EdgeSet& roundabout : myNetBuilder->getEdgeCont().getRoundabouts()) {
        for (NBEdge* edge : roundabout) {
            if (edge->getFromNode() == junction->getNBNode()) {
                undoList->begin(junction, TL("select roundabout"));
                for (const auto& roundaboutEdge : roundabout) {
                    GNEEdge* e = myAttributeCarriers->retrieveEdge(roundaboutEdge->getID());
                    e->setAttribute(GNE_ATTR_SELECTED, "true", undoList);
                    e->getToJunction()->setAttribute(GNE_ATTR_SELECTED, "true", undoList);
                }
                undoList->end();
                return;
            }
        }
    }
}


void
GNENet::createRoundabout(GNEJunction* junction, GNEUndoList* undoList) {
    undoList->begin(junction, TL("create roundabout"));
    // reset shape end from incoming edges
    for (const auto& incomingEdge : junction->getGNEIncomingEdges()) {
        incomingEdge->setAttribute(GNE_ATTR_SHAPE_END, "", undoList);
    }
    // reset shape start from outgoing edges
    for (const auto& outgoingEdge : junction->getGNEOutgoingEdges()) {
        outgoingEdge->setAttribute(GNE_ATTR_SHAPE_START, "", undoList);
    }
    junction->getNBNode()->updateSurroundingGeometry();
    double radius = junction->getNBNode()->getRadius();
    if (radius == NBNode::UNSPECIFIED_RADIUS) {
        radius = OptionsCont::getOptions().getFloat("default.junctions.radius");
    }
    std::vector<GNEEdge*> edges;
    // use clockwise sorting
    for (const auto& nbEdge : junction->getNBNode()->getEdges()) {
        edges.push_back(myAttributeCarriers->retrieveEdge(nbEdge->getID()));
    }
    const bool lefthand = OptionsCont::getOptions().getBool("lefthand");
    if (lefthand) {
        std::reverse(edges.begin(), edges.end());
    }
    const double lefthandSign = lefthand ? -1 : 1;
    std::vector<GNEJunction*> newJunctions;
    GNEEdge* templateEdge = nullptr;
    GNEEdge* prevOpposite = nullptr;
    // store old crossings edges as ids because edges are renamed and pointers will be invalid
    std::vector<std::pair<NBNode::Crossing, std::vector<std::string>>> oldCrossings;
    for (const auto& crossing : junction->getGNECrossings()) {
        std::vector<std::string> edgeIDs;
        for (auto e : crossing->getCrossingEdges()) {
            edgeIDs.push_back(e->getID());
        }
        oldCrossings.push_back(std::make_pair(*crossing->getNBCrossing(), edgeIDs));
    }
    std::map<std::string, std::string> edgeRename;
    // split incoming/outgoing edges
    for (GNEEdge* edge : edges) {
        GNEJunction* newJunction = nullptr;
        if (edge == prevOpposite) {
            newJunction = newJunctions.back();
        }
        // update template (most "important" incoming edge)
        if (edge->getToJunction() == junction) {
            if (templateEdge == nullptr) {
                templateEdge = edge;
            } else {
                NBEdge* tpl = templateEdge->getNBEdge();
                NBEdge* e = edge->getNBEdge();
                if (tpl->getNumLanes() < e->getNumLanes()
                        || (tpl->getNumLanes() == e->getNumLanes()
                            && tpl->getPriority() < e->getPriority())) {
                    templateEdge = edge;
                }
            }
        }
        //std::cout << " edge=" << edge->getID() << " prevOpposite=" << Named::getIDSecure(prevOpposite) << " newJunction=" << Named::getIDSecure(newJunction) << "\n";
        prevOpposite = edge->getOppositeEdges().size() > 0 ? edge->getOppositeEdges().front() : nullptr;
        const double geomLength = edge->getNBEdge()->getGeometry().length2D();
        const double splitOffset = (edge->getToJunction() == junction
                                    ? MAX2(POSITION_EPS, geomLength - radius)
                                    : MIN2(geomLength - POSITION_EPS, radius));
        Position pos = edge->getNBEdge()->getGeometry().positionAtOffset2D(splitOffset);
        auto newStuff = splitEdge(edge, pos, undoList, newJunction);
        newJunction = newStuff.first;
        if (edge->getFromJunction() == junction) {
            // edge will be deleted and the new part after the split kept. Preserve edge id
            edgeRename[newStuff.second->getID()] = edge->getID();
        }
        if (newJunctions.empty() || newJunction != newJunctions.back()) {
            newJunctions.push_back(newJunction);
        }
    }

    Position center = junction->getPositionInView();
    deleteJunction(junction, undoList);
    // rename edges after the originals have been deleted
    for (auto item : edgeRename) {
        GNEEdge* outgoing = myAttributeCarriers->retrieveEdge(item.first);
        outgoing->setAttribute(SUMO_ATTR_ID, item.second, undoList);
    }
    // restore crossings (after rename)
    for (auto nbCItem : oldCrossings) {
        for (GNEJunction* nj : newJunctions) {
            nbCItem.first.edges.clear();
            // check whether this junction has all the edges of the crossing
            for (const std::string& ce : nbCItem.second) {
                bool foundCE = false;
                for (NBEdge* je : nj->getNBNode()->getEdges()) {
                    if (je->getID() == ce) {
                        foundCE = true;
                        nbCItem.first.edges.push_back(je);
                        break;
                    }
                }
                if (!foundCE) {
                    break;
                }
            }
            if (nbCItem.first.edges.size() == nbCItem.second.size()) {
                undoList->add(new GNEChange_Crossing(nj, nbCItem.first, true), true);
                break;
            }
        }
    }
    // create new edges to connect roundabout junctions (counter-clockwise)
    const double resolution = OptionsCont::getOptions().getFloat("opendrive.curve-resolution") * 3;
    for (int i = 0; i < (int)newJunctions.size(); i++) {
        GNEJunction* from = newJunctions[(i + 1) % newJunctions.size()];
        GNEJunction* to = newJunctions[i];
        GNEEdge* newEdge = createEdge(from, to, nullptr, undoList);
        const double angle1 = center.angleTo2D(from->getPositionInView());
        const double angle2 = center.angleTo2D(to->getPositionInView());
        // insert geometry points every resolution meters
        const double angleDiff = fabs(GeomHelper::angleDiff(angle2, angle1));
        // circumference = 2 * M_PI * radius, angularFraction = angleDiff / 2 * M_PI
        int numSegments = MAX2(2, (int)ceil(angleDiff * radius / resolution));
        PositionVector innerGeom;
        for (int j = 1; j < numSegments; j++) {
            const double angle = angle1 + lefthandSign * j * angleDiff / numSegments;
            innerGeom.push_back(center + Position(cos(angle) * radius, sin(angle) * radius));
        }
        //std::cout << " newEdge=" << newEdge->getID() << " angle1=" << angle1 << " angle2=" << angle2 << " angleDiff=" << angleDiff
        //    << " numSegments=" << numSegments << " innerGeom=" << innerGeom << "\n";
        newEdge->setAttribute(SUMO_ATTR_SHAPE, toString(innerGeom), undoList);
        if (templateEdge) {
            GNEEdgeTemplate t(templateEdge);
            newEdge->copyTemplate(&t, undoList);
        }
    }
    undoList->end();
}


bool
GNENet::checkJunctionPosition(const Position& pos) {
    // Check that there isn't another junction in the same position as Pos
    for (auto& junction : myAttributeCarriers->getJunctions()) {
        if (junction.second->getPositionInView() == pos) {
            return false;
        }
    }
    return true;
}


void
GNENet::saveNetwork() {
    auto& neteditOptions = OptionsCont::getOptions();
    auto& sumoOptions = myViewNet->getViewParent()->getGNEAppWindows()->getSumoOptions();
    // set output file in SUMO and netedit options
    neteditOptions.resetWritable();
    neteditOptions.set("output-file", neteditOptions.getString("net-file"));
    sumoOptions.resetWritable();
    sumoOptions.set("net-file", neteditOptions.getString("net-file"));
    // compute without volatile options and update network
    computeAndUpdate(neteditOptions, false);
    // clear typeContainer
    myNetBuilder->getTypeCont().clearTypes();
    // now update typeContainer with edgeTypes
    for (const auto& edgeType : myAttributeCarriers->getEdgeTypes()) {
        myNetBuilder->getTypeCont().insertEdgeType(edgeType.first, edgeType.second);
        for (int i = 0; i < (int)edgeType.second->getLaneTypes().size(); i++) {
            myNetBuilder->getTypeCont().insertLaneType(edgeType.first, i,
                    edgeType.second->getLaneTypes().at(i)->speed,
                    edgeType.second->getLaneTypes().at(i)->permissions,
                    edgeType.second->getLaneTypes().at(i)->width,
                    edgeType.second->getLaneTypes().at(i)->attrs);
        }
    }
    // write network
    NWFrame::writeNetwork(neteditOptions, *myNetBuilder);
    // reset output file
    sumoOptions.resetWritable();
    neteditOptions.resetDefault("output-file");
    // mark network as saved
    mySavingStatus->networkSaved();
}


void
GNENet::savePlain(const std::string& prefix) {
    auto& neteditOptions = OptionsCont::getOptions();
    // compute without volatile options
    computeAndUpdate(neteditOptions, false);
    NWWriter_XML::writeNetwork(neteditOptions, prefix, *myNetBuilder);
}


void
GNENet::saveJoined(const std::string& filename) {
    // compute without volatile options
    computeAndUpdate(OptionsCont::getOptions(), false);
    NWWriter_XML::writeJoinedJunctions(filename, myNetBuilder->getNodeCont());
}


void
GNENet::setViewNet(GNEViewNet* viewNet) {
    // set view net
    myViewNet = viewNet;
    // add default vTypes
    myAttributeCarriers->addDefaultVTypes();
    // update all edge geometries
    for (const auto& edge : myAttributeCarriers->getEdges()) {
        edge.second->updateGeometry();
    }
    // update view Net and recalculate edge boundaries)
    myViewNet->update();
    for (const auto& edge : myAttributeCarriers->getEdges()) {
        edge.second->updateCenteringBoundary(true);
    }
}


void
GNENet::addGLObjectIntoGrid(GNEAttributeCarrier* AC) {
    // check if object must be inserted in RTREE
    if (AC->getTagProperty().isPlacedInRTree() && !AC->inGrid()) {
        myGrid.addAdditionalGLObject(AC->getGUIGlObject());
        AC->setInGrid(true);
    }
}


void
GNENet::removeGLObjectFromGrid(GNEAttributeCarrier* AC) {
    // check if object must be inserted in RTREE
    if (AC->getTagProperty().isPlacedInRTree() && AC->inGrid()) {
        myGrid.removeAdditionalGLObject(AC->getGUIGlObject());
        AC->setInGrid(false);
    }
}


void
GNENet::computeNetwork(GNEApplicationWindow* window, bool force, bool volatileOptions) {
    if (!myNeedRecompute) {
        if (force) {
            if (volatileOptions) {
                window->setStatusBarText(TL("Forced computing junctions with volatile options ..."));
            } else {
                window->setStatusBarText(TL("Forced computing junctions ..."));
            }
        } else {
            return;
        }
    } else {
        if (volatileOptions) {
            window->setStatusBarText(TL("Computing junctions with volatile options ..."));
        } else {
            window->setStatusBarText(TL("Computing junctions ..."));
        }
    }
    // start recomputing
    window->getApp()->beginWaitCursor();
    // save current number of lanes for every edge if recomputing is with volatile options
    if (volatileOptions) {
        for (const auto& edge : myAttributeCarriers->getEdges()) {
            myEdgesAndNumberOfLanes[edge.second->getID()] = (int)edge.second->getLanes().size();
        }
    }
    // compute and update
    auto& neteditOptions = OptionsCont::getOptions();
    computeAndUpdate(neteditOptions, volatileOptions);
    // load additionals if was recomputed with volatile options
    if (volatileOptions && OptionsCont::getOptions().getString("additional-files").size() > 0) {
        // Create additional handler
        GNEGeneralHandler generalHandler(this, OptionsCont::getOptions().getString("additional-files"), false, true);
        // Run parser
        if (!generalHandler.parse()) {
            WRITE_ERROR(TL("Loading of additional file failed: ") + OptionsCont::getOptions().getString("additional-files"));
        } else {
            WRITE_MESSAGE(TL("Loading of additional file successfully: ") + OptionsCont::getOptions().getString("additional-files"));
        }
    }
    // load demand elements if was recomputed with volatile options
    if (volatileOptions && OptionsCont::getOptions().getString("route-files").size() > 0) {
        // Create general handler
        GNEGeneralHandler generalHandler(this, OptionsCont::getOptions().getString("route-files"), false, true);
        // Run parser
        if (!generalHandler.parse()) {
            WRITE_ERROR(TL("Loading of route file failed: ") + OptionsCont::getOptions().getString("route-files"));
        } else {
            WRITE_MESSAGE(TL("Loading of route file successfully: ") + OptionsCont::getOptions().getString("route-files"));
        }
    }
    // load datas if was recomputed with volatile options
    if (volatileOptions && OptionsCont::getOptions().getString("data-files").size() > 0) {
        // Create data handler
        GNEDataHandler dataHandler(this, OptionsCont::getOptions().getString("data-files"), false, true);
        // Run parser
        if (!dataHandler.parse()) {
            WRITE_ERROR(TL("Loading of data file failed: ") + OptionsCont::getOptions().getString("data-files"));
        } else {
            WRITE_MESSAGE(TL("Loading of data file successfully: ") + OptionsCont::getOptions().getString("data-files"));
        }
    }
    // load meanDatas if was recomputed with volatile options
    if (volatileOptions && OptionsCont::getOptions().getString("meandata-files").size() > 0) {
        // Create meanData handler
        GNEGeneralHandler generalHandler(this, OptionsCont::getOptions().getString("meandata-files"), false, true);
        // Run parser
        if (!generalHandler.parse()) {
            WRITE_ERROR(TL("Loading of meandata file failed: ") + OptionsCont::getOptions().getString("meandata-files"));
        } else {
            WRITE_MESSAGE(TL("Loading of meandata file successfully: ") + OptionsCont::getOptions().getString("meandata-files"));
        }
    }
    // clear myEdgesAndNumberOfLanes after reload additionals
    myEdgesAndNumberOfLanes.clear();
    // end recomputing
    window->getApp()->endWaitCursor();
    // update status bar
    window->setStatusBarText(TL("Finished computing junctions."));
}


void
GNENet::computeDemandElements(GNEApplicationWindow* window) {
    window->setStatusBarText(TL("Computing demand elements ..."));
    // if we aren't in Demand mode, update path calculator
    if (!myViewNet->getEditModes().isCurrentSupermodeDemand() &&
            !myPathManager->getPathCalculator()->isPathCalculatorUpdated())  {
        myPathManager->getPathCalculator()->updatePathCalculator();
    }
    // clear demand paths
    myPathManager->clearDemandPaths();
    // iterate over all demand elements and compute
    for (const auto& demandElements : myAttributeCarriers->getDemandElements()) {
        for (const auto& demandElement : demandElements.second) {
            demandElement.second->computePathElement();
        }
    }
    window->setStatusBarText(TL("Finished computing demand elements."));
}


void
GNENet::computeDataElements(GNEApplicationWindow* window) {
    window->setStatusBarText(TL("Computing data elements ..."));
    // iterate over all demand elements and compute
    for (const auto& genericDataTag : myAttributeCarriers->getGenericDatas()) {
        for (const auto& genericData : genericDataTag.second) {
            genericData.second->computePathElement();
        }
    }
    window->setStatusBarText(TL("Finished computing data elements."));
}


void
GNENet::computeJunction(GNEJunction* junction) {
    // recompute tl-logics
    auto& neteditOptions = OptionsCont::getOptions();
    NBTrafficLightLogicCont& tllCont = getTLLogicCont();
    // iterate over traffic lights definitions. Make a copy because invalid
    // definitions will be removed (and would otherwise destroy the iterator)
    const std::set<NBTrafficLightDefinition*> tlsDefs = junction->getNBNode()->getControllingTLS();
    for (auto it : tlsDefs) {
        it->setParticipantsInformation();
        it->setTLControllingInformation();
        tllCont.computeSingleLogic(neteditOptions, it);
    }

    // @todo compute connections etc...
}


void
GNENet::requireRecompute() {
    myNeedRecompute = true;
}


bool
GNENet::isNetRecomputed() const {
    return !myNeedRecompute;
}


FXApp*
GNENet::getApp() {
    return myViewNet->getApp();
}


NBNetBuilder*
GNENet::getNetBuilder() const {
    return myNetBuilder;
}


bool
GNENet::joinSelectedJunctions(GNEUndoList* undoList) {
    const auto selectedJunctions = myAttributeCarriers->getSelectedJunctions();
    if (selectedJunctions.size() < 2) {
        return false;
    }
    EdgeVector allIncoming;
    EdgeVector allOutgoing;
    std::set<NBNode*, ComparatorIdLess> cluster;
    for (const auto& selectedJunction : selectedJunctions) {
        cluster.insert(selectedJunction->getNBNode());
        const EdgeVector& incoming = selectedJunction->getNBNode()->getIncomingEdges();
        allIncoming.insert(allIncoming.end(), incoming.begin(), incoming.end());
        const EdgeVector& outgoing = selectedJunction->getNBNode()->getOutgoingEdges();
        allOutgoing.insert(allOutgoing.end(), outgoing.begin(), outgoing.end());
    }
    // create new junction
    Position pos;
    Position oldPos;
    bool setTL = false;
    std::string id = "cluster";
    TrafficLightType type;
    SumoXMLNodeType nodeType = SumoXMLNodeType::UNKNOWN;
    myNetBuilder->getNodeCont().analyzeCluster(cluster, id, pos, setTL, type, nodeType);
    // save position
    oldPos = pos;
    // Check that there isn't another junction in the same position as Pos but doesn't belong to cluster
    for (const auto& junction : myAttributeCarriers->getJunctions()) {
        if ((junction.second->getPositionInView() == pos) && (cluster.find(junction.second->getNBNode()) == cluster.end())) {
            // show warning in gui testing debug mode
            WRITE_DEBUG("Opening FXMessageBox 'Join non-selected junction'");
            // Ask confirmation to user
            const std::string header = TL("Position of joined junction");
            const std::string bodyA = TL("There is another unselected junction in the same position of joined junction.");
            const std::string bodyB = TL("It will be joined with the other selected junctions. Continue?");
            const auto answer = FXMessageBox::question(getApp(), MBOX_YES_NO, header.c_str(), "%s", (bodyA + std::string("\n") + bodyB).c_str());
            if (answer != 1) { // 1:yes, 2:no, 4:esc
                // write warning if netedit is running in testing mode
                if (answer == 2) {
                    WRITE_DEBUG("Closed FXMessageBox 'Join non-selected junction' with 'No'");
                } else if (answer == 4) {
                    WRITE_DEBUG("Closed FXMessageBox 'Join non-selected junction' with 'ESC'");
                }
                return false;
            } else {
                // write warning if netedit is running in testing mode
                WRITE_DEBUG("Closed FXMessageBox 'Join non-selected junction' with 'Yes'");
                // select conflicted junction an join all again
                junction.second->setAttribute(GNE_ATTR_SELECTED, "true", undoList);
                return joinSelectedJunctions(undoList);
            }
        }
    }
    // use checkJunctionPosition to avoid conflicts with junction in the same position as others
    while (!checkJunctionPosition(pos)) {
        pos.setx(pos.x() + 0.1);
        pos.sety(pos.y() + 0.1);
    }
    // start with the join selected junctions
    undoList->begin(GUIIcon::JUNCTION, "Join selected " + toString(SUMO_TAG_JUNCTION) + "s");
    GNEJunction* joined = createJunction(pos, undoList);
    joined->setAttribute(SUMO_ATTR_TYPE, toString(nodeType), undoList); // i.e. rail crossing
    if (setTL) {
        joined->setAttribute(SUMO_ATTR_TLTYPE, toString(type), undoList);
    }
    GNEChange_RegisterJoin::registerJoin(cluster, myNetBuilder->getNodeCont(), undoList);
    // first remove all crossing of the involved junctions and edges
    // (otherwise edge removal will trigger discarding)
    std::vector<NBNode::Crossing> oldCrossings;
    for (const auto& selectedJunction : selectedJunctions) {
        const auto crossings = selectedJunction->getGNECrossings();
        for (auto crossing : crossings) {
            deleteCrossing(crossing, undoList);
        }
    }
    // preserve old connections
    for (const auto& selectedJunction : selectedJunctions) {
        selectedJunction->setLogicValid(false, undoList);
    }
    // remap edges
    for (const auto& incomingEdge : allIncoming) {
        if (std::find(allOutgoing.begin(), allOutgoing.end(), incomingEdge) == allOutgoing.end()) {
            GNEChange_Attribute::changeAttribute(myAttributeCarriers->getEdges().at(incomingEdge->getID()), SUMO_ATTR_TO, joined->getID(), undoList);
        }
    }
    EdgeSet edgesWithin;
    for (const auto& outgoingEdge : allOutgoing) {
        // delete edges within the cluster
        GNEEdge* edge = myAttributeCarriers->getEdges().at(outgoingEdge->getID());
        if (edge->getToJunction() == joined) {
            edgesWithin.insert(outgoingEdge);
            deleteEdge(edge, undoList, false);
        } else {
            GNEChange_Attribute::changeAttribute(myAttributeCarriers->getEdges().at(outgoingEdge->getID()), SUMO_ATTR_FROM, joined->getID(), undoList);
        }
    }
    // remap all crossing of the involved junctions and edges
    for (const auto& nbc : oldCrossings) {
        bool keep = true;
        for (NBEdge* e : nbc.edges) {
            if (edgesWithin.count(e) != 0) {
                keep = false;
                break;
            }
        };
        if (keep) {
            undoList->add(new GNEChange_Crossing(joined, nbc.edges, nbc.width,
                                                 nbc.priority || joined->getNBNode()->isTLControlled(),
                                                 nbc.customTLIndex, nbc.customTLIndex2, nbc.customShape,
                                                 false, true), true);
        }
    }
    // delete original junctions
    for (const auto& selectedJunction : selectedJunctions) {
        deleteJunction(selectedJunction, undoList);
    }
    joined->setAttribute(SUMO_ATTR_ID, id, undoList);

    // check if joined junction had to change their original position to avoid errors
    if (pos != oldPos) {
        joined->setAttribute(SUMO_ATTR_POSITION, toString(oldPos), undoList);
    }
    undoList->end();
    return true;
}


bool
GNENet::cleanInvalidCrossings(GNEUndoList* undoList) {
    // obtain current net's crossings
    std::vector<GNECrossing*> myNetCrossings;
    for (const auto& junction : myAttributeCarriers->getJunctions()) {
        myNetCrossings.reserve(myNetCrossings.size() + junction.second->getGNECrossings().size());
        myNetCrossings.insert(myNetCrossings.end(), junction.second->getGNECrossings().begin(), junction.second->getGNECrossings().end());
    }
    // obtain invalid crossings
    std::vector<GNECrossing*> myInvalidCrossings;
    for (auto i = myNetCrossings.begin(); i != myNetCrossings.end(); i++) {
        if (!(*i)->getNBCrossing()->valid) {
            myInvalidCrossings.push_back(*i);
        }
    }

    if (myInvalidCrossings.empty()) {
        // show warning in gui testing debug mode
        WRITE_DEBUG("Opening FXMessageBox 'No crossing to remove'");
        // open a dialog informing that there isn't crossing to remove
        const std::string header = TL("Clear crossings");
        const std::string body = TL("There are no invalid crossings to remove.");
        FXMessageBox::warning(getApp(), MBOX_OK, (header).c_str(), "%s", (body).c_str());
        // show warning in gui testing debug mode
        WRITE_DEBUG("Closed FXMessageBox 'No crossing to remove' with 'OK'");
    } else {
        std::string plural = myInvalidCrossings.size() == 1 ? ("") : ("s");
        // show warning in gui testing debug mode
        WRITE_DEBUG("Opening FXMessageBox 'clear crossings'");
        // Ask confirmation to user
        const std::string header = TL("Clear crossings");
        const std::string body = TL("Crossings will be cleared. Continue?");
        const auto answer = FXMessageBox::question(getApp(), MBOX_YES_NO, header.c_str(), "%s", body.c_str());
        // 1:yes, 2:no, 4:esc
        if (answer != 1) {
            // write warning if netedit is running in testing mode
            if (answer == 2) {
                WRITE_DEBUG("Closed FXMessageBox 'clear crossings' with 'No'");
            } else if (answer == 4) {
                WRITE_DEBUG("Closed FXMessageBox 'clear crossings' with 'ESC'");
            }
        } else {
            undoList->begin(GUIIcon::MODEDELETE, TL("clear crossings"));
            for (auto i = myInvalidCrossings.begin(); i != myInvalidCrossings.end(); i++) {
                deleteCrossing((*i), undoList);
            }
            undoList->end();
        }
    }
    return 1;
}


void
GNENet::removeSolitaryJunctions(GNEUndoList* undoList) {
    undoList->begin(GUIIcon::MODEDELETE, TL("clear junctions"));
    std::vector<GNEJunction*> toRemove;
    for (auto& junction : myAttributeCarriers->getJunctions()) {
        if (junction.second->getNBNode()->getEdges().size() == 0) {
            toRemove.push_back(junction.second);
        }
    }
    for (auto junction : toRemove) {
        deleteJunction(junction, undoList);
    }
    undoList->end();
}


void
GNENet::cleanUnusedRoutes(GNEUndoList* undoList) {
    // first declare a vector to save all routes without children
    std::vector<GNEDemandElement*> routesWithoutChildren;
    routesWithoutChildren.reserve(myAttributeCarriers->getDemandElements().at(SUMO_TAG_ROUTE).size());
    // iterate over routes
    for (const auto& route : myAttributeCarriers->getDemandElements().at(SUMO_TAG_ROUTE)) {
        if (route.second->getChildDemandElements().empty()) {
            routesWithoutChildren.push_back(route.second);
        }
    }
    // finally remove all routesWithoutChildren
    if (routesWithoutChildren.size() > 0) {
        // begin undo list
        undoList->begin(GUIIcon::MODEDELETE, TL("clear unused routes"));
        // iterate over routesWithoutChildren
        for (const auto& i : routesWithoutChildren) {
            // due route doesn't have children, simply call GNEChange_DemandElement
            undoList->add(new GNEChange_DemandElement(i, false), true);
        }
        // end undo list
        undoList->end();
    }
}


void
GNENet::joinRoutes(GNEUndoList* undoList) {
    // first declare a sorted set of sorted route's edges in string format
    std::set<std::pair<std::string, GNEDemandElement*> > mySortedRoutes;
    // iterate over routes and save it in mySortedRoutes  (only if it doesn't have Stop Children)
    for (const auto& route : myAttributeCarriers->getDemandElements().at(SUMO_TAG_ROUTE)) {
        // first check route has stops
        bool hasStops = false;
        for (const auto& stop : route.second->getChildDemandElements()) {
            if (stop->getTagProperty().isVehicleStop()) {
                hasStops = true;
            }
        }
        if (!hasStops) {
            mySortedRoutes.insert(std::make_pair(GNEAttributeCarrier::parseIDs(route.second->getParentEdges()), route.second));
        }
    }
    // now declare a matrix in which organize routes to be merged
    std::vector<std::vector<GNEDemandElement*> > routesToMerge;
    auto index = mySortedRoutes.begin();
    // iterate over mySortedRoutes
    for (auto i = mySortedRoutes.begin(); i != mySortedRoutes.end(); i++) {
        if (routesToMerge.empty()) {
            routesToMerge.push_back({i->second});
        } else {
            if (index->first == i->first) {
                routesToMerge.back().push_back(i->second);
            } else {
                routesToMerge.push_back({i->second});
                index = i;
            }
        }
    }
    // now check if there is routes to merge
    bool thereIsRoutesToMerge = false;
    for (const auto& i : routesToMerge) {
        if (i.size() > 1) {
            thereIsRoutesToMerge = true;
        }
    }
    // if exist
    if (thereIsRoutesToMerge) {
        // begin undo list
        undoList->begin(GUIIcon::ROUTE, TL("merge routes"));
        // iterate over route to edges
        for (const auto& routes : routesToMerge) {
            if (routes.size() > 1) {
                // iterate over duplicated routes
                for (int i = 1; i < (int)routes.size(); i++) {
                    // move all vehicles of every duplicated route
                    while (routes.at(i)->getChildDemandElements().size() > 0) {
                        routes.at(i)->getChildDemandElements().front()->setAttribute(SUMO_ATTR_ROUTE, routes.at(0)->getID(), undoList);
                    }
                    // finally remove route
                    undoList->add(new GNEChange_DemandElement(routes.at(i), false), true);
                }
            }
        }
        // end undo list
        undoList->end();
    }
}


void
GNENet::adjustPersonPlans(GNEUndoList* undoList) {
    // declare personPlan-pos map
    std::map<GNEDemandElement*, std::string> personPlanMap;
    // iterate over persons
    for (const auto& persontag : {
                SUMO_TAG_PERSON, SUMO_TAG_PERSONFLOW
            }) {
        for (const auto& person : myAttributeCarriers->getDemandElements().at(persontag)) {
            if (person.second->getChildDemandElements().size() > 0) {
                // get person plan
                GNEDemandElement* personPlan = person.second->getChildDemandElements().front();
                // iterate over all personPlans
                while (personPlan) {
                    // check if personPlan is a stopPerson over edge
                    if (personPlan->getTagProperty().getTag() == GNE_TAG_STOPPERSON_EDGE) {
                        // get previous person plan
                        GNEDemandElement* previousPersonPlan = person.second->getPreviousChildDemandElement(personPlan);
                        // check if arrivalPos of previous personPlan is different of endPos of stopPerson
                        if (previousPersonPlan && previousPersonPlan->getTagProperty().hasAttribute(SUMO_ATTR_ARRIVALPOS) &&
                                (previousPersonPlan->getAttribute(SUMO_ATTR_ARRIVALPOS) != personPlan->getAttribute(SUMO_ATTR_ENDPOS))) {
                            personPlanMap[previousPersonPlan] = personPlan->getAttribute(SUMO_ATTR_ENDPOS);
                        }
                    }
                    // go to next person plan
                    personPlan = person.second->getNextChildDemandElement(personPlan);
                }
            }
        }
    }
    // continue if there is personPlanMap to adjust
    if (personPlanMap.size() > 0) {
        // begin undo list
        undoList->begin(GUIIcon::MODEPERSONPLAN, TL("adjust person plans"));
        // iterate over invalidDemandElements
        for (const auto& personPlan : personPlanMap) {
            // set arrivalPos attribute
            personPlan.first->setAttribute(SUMO_ATTR_ARRIVALPOS, personPlan.second, undoList);
        }
        // end undo list
        undoList->end();
    }
}


void
GNENet::cleanInvalidDemandElements(GNEUndoList* undoList) {
    // first declare a vector to save all invalid demand elements
    std::vector<GNEDemandElement*> invalidDemandElements;
    invalidDemandElements.reserve(myAttributeCarriers->getDemandElements().at(SUMO_TAG_ROUTE).size() +
                                  myAttributeCarriers->getDemandElements().at(SUMO_TAG_FLOW).size() +
                                  myAttributeCarriers->getDemandElements().at(SUMO_TAG_TRIP).size());
    // iterate over routes
    for (const auto& route : myAttributeCarriers->getDemandElements().at(SUMO_TAG_ROUTE)) {
        if (route.second->isDemandElementValid() != GNEDemandElement::Problem::OK) {
            invalidDemandElements.push_back(route.second);
        }
    }
    // iterate over flows
    for (const auto& flow : myAttributeCarriers->getDemandElements().at(SUMO_TAG_FLOW)) {
        if (flow.second->isDemandElementValid() != GNEDemandElement::Problem::OK) {
            invalidDemandElements.push_back(flow.second);
        }
    }
    // iterate over trip
    for (const auto& trip : myAttributeCarriers->getDemandElements().at(SUMO_TAG_TRIP)) {
        if (trip.second->isDemandElementValid() != GNEDemandElement::Problem::OK) {
            invalidDemandElements.push_back(trip.second);
        }
    }
    // continue if there is invalidDemandElements to remove
    if (invalidDemandElements.size() > 0) {
        // begin undo list
        undoList->begin(GUIIcon::MODEDELETE, TL("remove invalid demand elements"));
        // iterate over invalidDemandElements
        for (const auto& invalidDemandElement : invalidDemandElements) {
            // simply call GNEChange_DemandElement
            undoList->add(new GNEChange_DemandElement(invalidDemandElement, false), true);
        }
        // end undo list
        undoList->end();
    }
}

void
GNENet::replaceJunctionByGeometry(GNEJunction* junction, GNEUndoList* undoList) {
    if (junction->getNBNode()->checkIsRemovable()) {
        // start operation
        undoList->begin(junction, TL("replace junction by geometry"));
        // obtain Edges to join
        std::vector<std::pair<NBEdge*, NBEdge*> > toJoin = junction->getNBNode()->getEdgesToJoin();
        // clear connections of junction to replace
        clearJunctionConnections(junction, undoList);
        // iterate over NBEdges to join
        for (auto j : toJoin) {
            // obtain GNEEdges
            GNEEdge* begin = myAttributeCarriers->getEdges().at(j.first->getID());
            GNEEdge* continuation = myAttributeCarriers->getEdges().at(j.second->getID());
            // remove connections between the edges
            std::vector<NBEdge::Connection> connections = begin->getNBEdge()->getConnections();
            for (auto con : connections) {
                undoList->add(new GNEChange_Connection(begin, con, false, false), true);
            }
            // fix shape of replaced edge
            PositionVector newShape = begin->getNBEdge()->getInnerGeometry();
            if (begin->getNBEdge()->hasDefaultGeometryEndpointAtNode(begin->getNBEdge()->getToNode())) {
                newShape.push_back(junction->getNBNode()->getPosition());
            } else {
                newShape.push_back(begin->getNBEdge()->getGeometry()[-1]);
            }
            if (continuation->getNBEdge()->hasDefaultGeometryEndpointAtNode(begin->getNBEdge()->getToNode())) {
                newShape.push_back_noDoublePos(junction->getNBNode()->getPosition());
            } else {
                newShape.push_back_noDoublePos(continuation->getNBEdge()->getGeometry()[0]);
            }
            // replace incoming edge
            replaceIncomingEdge(continuation, begin, undoList);

            newShape.append(continuation->getNBEdge()->getInnerGeometry());
            begin->setAttribute(GNE_ATTR_SHAPE_END, continuation->getAttribute(GNE_ATTR_SHAPE_END), undoList);
            begin->setAttribute(SUMO_ATTR_ENDOFFSET, continuation->getAttribute(SUMO_ATTR_ENDOFFSET), undoList);
            begin->setAttribute(SUMO_ATTR_SHAPE, toString(newShape), undoList);
            begin->getNBEdge()->resetNodeBorder(begin->getNBEdge()->getToNode());
            // fix loaded lengths
            if (begin->getNBEdge()->hasLoadedLength() || continuation->getNBEdge()->hasLoadedLength()) {
                begin->setAttribute(SUMO_ATTR_LENGTH, toString(begin->getNBEdge()->getLoadedLength() + continuation->getNBEdge()->getLoadedLength()), undoList);
            }
        }
        //delete replaced junction
        deleteJunction(junction, undoList);
        // finish operation
        undoList->end();
    } else {
        throw ProcessError(TL("Junction isn't removable"));
    }
}


void
GNENet::splitJunction(GNEJunction* junction, bool reconnect, GNEUndoList* undoList) {
    std::vector<std::pair<Position, std::string> > endpoints = junction->getNBNode()->getEndPoints();
    if (endpoints.size() < 2) {
        return;
    }
    // start operation
    undoList->begin(junction, TL("split junction"));
    // record connections
    std::map<GNEEdge*, std::vector<NBEdge::Connection>> straightConnections;
    for (GNEEdge* e : junction->getGNEIncomingEdges()) {
        for (const auto& c : e->getNBEdge()->getConnections()) {
            if (c.fromLane >= 0 && junction->getNBNode()->getDirection(e->getNBEdge(), c.toEdge) == LinkDirection::STRAIGHT) {
                straightConnections[e].push_back(c);
            }
        };
    }
    //std::cout << "split junction at endpoints:\n";

    junction->setLogicValid(false, undoList);
    for (const auto& pair : endpoints) {
        const Position& pos = pair.first;
        const std::string& origID = pair.second;
        GNEJunction* newJunction = createJunction(pos, undoList);
        std::string newID = origID != "" ? origID : newJunction->getID();
        // make a copy because the original vectors are modified during iteration
        const std::vector<GNEEdge*> incoming = junction->getGNEIncomingEdges();
        const std::vector<GNEEdge*> outgoing = junction->getGNEOutgoingEdges();
        //std::cout << "  checkEndpoint " << pair.first << " " << pair.second << " newID=" << newID << "\n";
        for (GNEEdge* e : incoming) {
            //std::cout << "   incoming " << e->getID() << " pos=" << pos << " origTo=" << e->getNBEdge()->getParameter("origTo") << " newID=" << newID << "\n";
            if (e->getNBEdge()->getGeometry().back().almostSame(pos) || e->getNBEdge()->getParameter("origTo") == newID) {
                //std::cout << "     match\n";
                GNEChange_Attribute::changeAttribute(e, SUMO_ATTR_TO, newJunction->getID(), undoList);
            }
        }
        for (GNEEdge* e : outgoing) {
            //std::cout << "   outgoing " << e->getID() << " pos=" << pos << " origFrom=" << e->getNBEdge()->getParameter("origFrom") << " newID=" << newID << "\n";
            if (e->getNBEdge()->getGeometry().front().almostSame(pos) || e->getNBEdge()->getParameter("origFrom") == newID) {
                //std::cout << "     match\n";
                GNEChange_Attribute::changeAttribute(e, SUMO_ATTR_FROM, newJunction->getID(), undoList);
            }
        }
        if (newID != newJunction->getID()) {
            if (newJunction->isValid(SUMO_ATTR_ID, newID)) {
                GNEChange_Attribute::changeAttribute(newJunction, SUMO_ATTR_ID, newID, undoList);
            } else {
                WRITE_WARNINGF(TL("Could not rename split node to '%'"), newID);
            }
        }
    }
    // recreate edges from straightConnections
    if (reconnect) {
        for (const auto& item : straightConnections) {
            GNEEdge* in = item.first;
            std::map<NBEdge*, GNEEdge*> newEdges;
            for (auto& c : item.second) {
                GNEEdge* out = myAttributeCarriers->retrieveEdge(c.toEdge->getID());
                GNEEdge* newEdge = nullptr;
                if (in->getToJunction() == out->getFromJunction()) {
                    continue;
                }
                if (newEdges.count(c.toEdge) == 0) {
                    newEdge = createEdge(in->getToJunction(), out->getFromJunction(), in, undoList);
                    newEdges[c.toEdge] = newEdge;
                    newEdge->setAttribute(SUMO_ATTR_NUMLANES, "1", undoList);
                } else {
                    newEdge = newEdges[c.toEdge];
                    duplicateLane(newEdge->getLanes().back(), undoList, true);
                }
                // copy permissions
                newEdge->getLanes().back()->setAttribute(SUMO_ATTR_ALLOW,
                        in->getLanes()[c.fromLane]-> getAttribute(SUMO_ATTR_ALLOW), undoList);
            }
        }
    }

    deleteJunction(junction, undoList);
    // finish operation
    undoList->end();
}



void
GNENet::clearJunctionConnections(GNEJunction* junction, GNEUndoList* undoList) {
    undoList->begin(GUIIcon::MODEDELETE, TL("clear junction connections"));
    std::vector<GNEConnection*> connections = junction->getGNEConnections();
    // Iterate over all connections and clear it
    for (auto i : connections) {
        deleteConnection(i, undoList);
    }
    undoList->end();
}


void
GNENet::resetJunctionConnections(GNEJunction* junction, GNEUndoList* undoList) {
    undoList->begin(junction, TL("reset junction connections"));
    // first clear connections
    clearJunctionConnections(junction, undoList);
    // invalidate logic to create new connections in the next recomputing
    junction->setLogicValid(false, undoList);
    undoList->end();
}


void
GNENet::clearAdditionalElements(GNEUndoList* undoList) {
    undoList->begin(GUIIcon::MODEDELETE, TL("clear additional elements"));
    // clear additionals
    for (const auto& additionalMap : myAttributeCarriers->getAdditionals()) {
        while (additionalMap.second.size() > 0) {
            deleteAdditional(additionalMap.second.begin()->second, undoList);
        }
    }
    undoList->end();
}


void
GNENet::clearDemandElements(GNEUndoList* undoList) {
    undoList->begin(GUIIcon::MODEDELETE, TL("clear demand elements"));
    // clear demand elements
    for (const auto& demandElementsMap : myAttributeCarriers->getDemandElements()) {
        while (demandElementsMap.second.size() > 0) {
            deleteDemandElement(demandElementsMap.second.begin()->second, undoList);
        }
    }
    undoList->end();
}


void
GNENet::clearDataElements(GNEUndoList* undoList) {
    undoList->begin(GUIIcon::MODEDELETE, TL("clear data elements"));
    // clear data sets
    for (const auto& dataSet : myAttributeCarriers->getDataSets()) {
        deleteDataSet(dataSet.second, undoList);
    }
    undoList->end();
}


void
GNENet::clearMeanDataElements(GNEUndoList* undoList) {
    undoList->begin(GUIIcon::MODEDELETE, TL("clear meanData elements"));
    // clear meanDatas
    for (const auto& meanDataMap : myAttributeCarriers->getMeanDatas()) {
        while (meanDataMap.second.size() > 0) {
            deleteMeanData(meanDataMap.second.begin()->second, undoList);
        }
    }
    undoList->end();
}


void
GNENet::changeEdgeEndpoints(GNEEdge* edge, const std::string& newSource, const std::string& newDest) {
    NBNode* from = myAttributeCarriers->retrieveJunction(newSource)->getNBNode();
    NBNode* to = myAttributeCarriers->retrieveJunction(newDest)->getNBNode();
    edge->getNBEdge()->reinitNodes(from, to);
    requireRecompute();
}


GNEViewNet*
GNENet::getViewNet() const {
    return myViewNet;
}


NBTrafficLightLogicCont&
GNENet::getTLLogicCont() {
    return myNetBuilder->getTLLogicCont();
}


NBEdgeCont&
GNENet::getEdgeCont() {
    return myNetBuilder->getEdgeCont();
}


void
GNENet::addExplicitTurnaround(std::string id) {
    myExplicitTurnarounds.insert(id);
}


void
GNENet::removeExplicitTurnaround(std::string id) {
    myExplicitTurnarounds.erase(id);
}


void
GNENet::saveAdditionals() {
    // obtain invalid additionals depending of number of their parent lanes
    std::vector<GNEAdditional*> invalidSingleLaneAdditionals;
    std::vector<GNEAdditional*> invalidMultiLaneAdditionals;
    // iterate over additionals and obtain invalids
    for (const auto& additionalPair : myAttributeCarriers->getAdditionals()) {
        for (const auto& addditional : additionalPair.second) {
            // check if has to be fixed
            if (addditional.second->getTagProperty().hasAttribute(SUMO_ATTR_LANE) && !addditional.second->isAdditionalValid()) {
                invalidSingleLaneAdditionals.push_back(addditional.second);
            } else if (addditional.second->getTagProperty().hasAttribute(SUMO_ATTR_LANES) && !addditional.second->isAdditionalValid()) {
                invalidMultiLaneAdditionals.push_back(addditional.second);
            }
        }
    }
    // if there are invalid StoppingPlaces or detectors, open GNEFixAdditionalElements
    if (invalidSingleLaneAdditionals.size() > 0 || invalidMultiLaneAdditionals.size() > 0) {
        // 0 -> Canceled Saving, with or without selecting invalid stopping places and E2
        // 1 -> Invalid stoppingPlaces and E2 fixed, friendlyPos enabled, or saved with invalid positions
        GNEFixAdditionalElements fixAdditionalElementsDialog(myViewNet, invalidSingleLaneAdditionals, invalidMultiLaneAdditionals);
        if (fixAdditionalElementsDialog.execute() == 0) {
            // show debug information
            WRITE_DEBUG("Additionals saving aborted");
        } else {
            saveAdditionalsConfirmed();
            // show debug information
            WRITE_DEBUG("Additionals saved after dialog");
        }
        // set focus again in net
        myViewNet->setFocus();
    } else {
        saveAdditionalsConfirmed();
        // show debug information
        WRITE_DEBUG("Additionals saved");
    }
}


void
GNENet::saveJuPedSimElements(const std::string& file) {
    OutputDevice& device = OutputDevice::getDevice(file);
    // open header
    device.writeXMLHeader("additional", "additional_file.xsd", EMPTY_HEADER, false);
    // juPedSim elements
    writeJuPedSimComment(device);
    writeAdditionalByType(device, {GNE_TAG_JPS_WALKABLEAREA});
    writeAdditionalByType(device, {GNE_TAG_JPS_OBSTACLE});
    // close device
    device.close();
}


void
GNENet::saveDemandElements() {
    // first recompute demand elements
    computeDemandElements(myViewNet->getViewParent()->getGNEAppWindows());
    // obtain invalid demandElements depending of number of their parent lanes
    std::vector<GNEDemandElement*> invalidSingleLaneDemandElements;
    // iterate over demandElements and obtain invalids
    for (const auto& demandElementSet : myAttributeCarriers->getDemandElements()) {
        for (const auto& demandElement : demandElementSet.second) {
            // compute before check if demand element is valid
            demandElement.second->computePathElement();
            // check if has to be fixed
            if (demandElement.second->isDemandElementValid() != GNEDemandElement::Problem::OK) {
                invalidSingleLaneDemandElements.push_back(demandElement.second);
            }
        }
    }
    // if there are invalid demand elements, open GNEFixDemandElements
    if (invalidSingleLaneDemandElements.size() > 0) {
        // 0 -> Canceled Saving, with or without selecting invalid demand elements
        // 1 -> Invalid demand elements fixed, friendlyPos enabled, or saved with invalid positions
        GNEFixDemandElements fixDemandElementsDialog(myViewNet, invalidSingleLaneDemandElements);
        if (fixDemandElementsDialog.execute() == 0) {
            // show debug information
            WRITE_DEBUG("demand elements saving aborted");
        } else {
            saveDemandElementsConfirmed();
            // show debug information
            WRITE_DEBUG("demand elements saved after dialog");
        }
        // set focus again in net
        myViewNet->setFocus();
    } else {
        saveDemandElementsConfirmed();
        // show debug information
        WRITE_DEBUG("demand elements saved");
    }
}


void
GNENet::saveDataElements() {
    // first recompute data sets
    computeDataElements(myViewNet->getViewParent()->getGNEAppWindows());
    // save data elements
    saveDataElementsConfirmed();
    // show debug information
    WRITE_DEBUG("data sets saved");
}


double
GNENet::getDataSetIntervalMinimumBegin() const {
    double minimumBegin = 0;
    // update with first minimum (if exist)
    if (myAttributeCarriers->getDataIntervals().size() > 0) {
        minimumBegin = myAttributeCarriers->getDataIntervals().begin()->second->getAttributeDouble(SUMO_ATTR_BEGIN);
    }
    // iterate over interval
    for (const auto& interval : myAttributeCarriers->getDataIntervals()) {
        if (interval.second->getAttributeDouble(SUMO_ATTR_BEGIN) < minimumBegin) {
            minimumBegin = interval.second->getAttributeDouble(SUMO_ATTR_BEGIN);
        }
    }
    return minimumBegin;
}


double
GNENet::getDataSetIntervalMaximumEnd() const {
    double maximumEnd = 0;
    // update with first maximum (if exist)
    if (myAttributeCarriers->getDataIntervals().size() > 0) {
        maximumEnd = myAttributeCarriers->getDataIntervals().begin()->second->getAttributeDouble(SUMO_ATTR_END);
    }
    // iterate over intervals
    for (const auto& interval : myAttributeCarriers->getDataIntervals()) {
        if (interval.second->getAttributeDouble(SUMO_ATTR_END) > maximumEnd) {
            maximumEnd = interval.second->getAttributeDouble(SUMO_ATTR_END);
        }
    }
    return maximumEnd;
}


void
GNENet::saveMeanDatas() {
    saveMeanDatasConfirmed();
    // show debug information
    WRITE_DEBUG("MeanDatas saved");
}


void
GNENet::saveAdditionalsConfirmed() {
    OutputDevice& device = OutputDevice::getDevice(OptionsCont::getOptions().getString("additional-files"));
    // open header
    device.writeXMLHeader("additional", "additional_file.xsd", EMPTY_HEADER, false);
    // write vTypes with additional childrens (due calibrators)
    writeVTypeComment(device, true);
    writeVTypeDistributions(device, true);
    writeVTypes(device, true);
    // write routes with additional children (due route prob reroutes)
    writeRouteComment(device, true);
    writeRouteDistributions(device, true);
    writeRoutes(device, true);
    // routeProbes
    writeRouteProbeComment(device);
    writeAdditionalByType(device, {SUMO_TAG_ROUTEPROBE});
    // calibrator
    writeCalibratorComment(device);
    writeAdditionalByType(device, {SUMO_TAG_CALIBRATOR, GNE_TAG_CALIBRATOR_LANE});
    // stoppingPlaces
    writeStoppingPlaceComment(device);
    writeAdditionalByType(device, {SUMO_TAG_BUS_STOP});
    writeAdditionalByType(device, {SUMO_TAG_TRAIN_STOP});
    writeAdditionalByType(device, {SUMO_TAG_CONTAINER_STOP});
    writeAdditionalByType(device, {SUMO_TAG_PARKING_AREA});
    writeAdditionalByType(device, {SUMO_TAG_CHARGING_STATION});
    // detectors
    writeDetectorComment(device);
    writeAdditionalByType(device, {SUMO_TAG_INDUCTION_LOOP});
    writeAdditionalByType(device, {SUMO_TAG_INSTANT_INDUCTION_LOOP});
    writeAdditionalByType(device, {SUMO_TAG_LANE_AREA_DETECTOR, GNE_TAG_MULTI_LANE_AREA_DETECTOR});
    writeAdditionalByType(device, {SUMO_TAG_ENTRY_EXIT_DETECTOR});
    // Other additionals
    writeOtherAdditionalsComment(device);
    writeAdditionalByType(device, {SUMO_TAG_REROUTER});
    writeAdditionalByType(device, {SUMO_TAG_VSS});
    writeAdditionalByType(device, {SUMO_TAG_VAPORIZER});
    // shapes
    writeShapesComment(device);
    writeAdditionalByType(device, {SUMO_TAG_POLY});
    writeAdditionalByType(device, {SUMO_TAG_POI, GNE_TAG_POILANE, GNE_TAG_POIGEO});
    // TAZs
    writeTAZComment(device);
    writeAdditionalByType(device, {SUMO_TAG_TAZ});
    // Wire element
    writeWireComment(device);
    writeAdditionalByType(device, {SUMO_TAG_TRACTION_SUBSTATION});
    writeAdditionalByType(device, {SUMO_TAG_OVERHEAD_WIRE_SECTION});
    writeAdditionalByType(device, {SUMO_TAG_OVERHEAD_WIRE_CLAMP});
    // juPedSim elements
    writeJuPedSimComment(device);
    writeAdditionalByType(device, {GNE_TAG_JPS_WALKABLEAREA});
    writeAdditionalByType(device, {GNE_TAG_JPS_OBSTACLE});
    // close device
    device.close();
    // mark additionals as saved
    mySavingStatus->additionalsSaved();
}


void
GNENet::saveDemandElementsConfirmed() {
    OutputDevice& device = OutputDevice::getDevice(OptionsCont::getOptions().getString("route-files"));
    // open header
    device.writeXMLHeader("routes", "routes_file.xsd", EMPTY_HEADER, false);
    // first  write all vTypeDistributions (and their vTypes)
    writeVTypeComment(device, false);
    writeVTypes(device, false);
    writeVTypeDistributions(device, false);
    // now write all routes (and their associated stops), except routes with additional children (due routeProbReroutes)
    writeRouteComment(device, false);
    writeRoutes(device, false);
    writeRouteDistributions(device, false);
    // sort vehicles/persons by depart
    std::map<double, std::map<std::pair<SumoXMLTag, std::string>, GNEDemandElement*> > vehiclesSortedByDepart;
    for (const auto& demandElementTag : myAttributeCarriers->getDemandElements()) {
        for (const auto& demandElement : demandElementTag.second) {
            if (demandElement.second->getTagProperty().isVehicle() || demandElement.second->getTagProperty().isPerson() || demandElement.second->getTagProperty().isContainer()) {
                vehiclesSortedByDepart[demandElement.second->getAttributeDouble(SUMO_ATTR_DEPART)][std::make_pair(demandElement.second->getTagProperty().getTag(), demandElement.second->getID())] = demandElement.second;
            }
        }
    }
    // finally write all vehicles, persons and containers sorted by depart time (and their associated stops, personPlans, etc.)
    if (vehiclesSortedByDepart.size() > 0) {
        device << ("    <!-- Vehicles, persons and containers (sorted by depart) -->\n");
        for (const auto& vehicleTag : vehiclesSortedByDepart) {
            for (const auto& vehicle : vehicleTag.second) {
                vehicle.second->writeDemandElement(device);
            }
        }
    }
    // close device
    device.close();
    // mark demand elements as saved
    mySavingStatus->demandElementsSaved();
}


void
GNENet::saveDataElementsConfirmed() {
    OutputDevice& device = OutputDevice::getDevice(OptionsCont::getOptions().getString("data-files"));
    device.writeXMLHeader("data", "datamode_file.xsd", EMPTY_HEADER, false);
    // write all data sets
    for (const auto& dataSet : myAttributeCarriers->getDataSets()) {
        dataSet.second->writeDataSet(device);
    }
    // close device
    device.close();
    // mark data element as saved
    mySavingStatus->dataElementsSaved();
}


void
GNENet::saveMeanDatasConfirmed() {
    OutputDevice& device = OutputDevice::getDevice(OptionsCont::getOptions().getString("meandata-files"));
    // open header
    device.writeXMLHeader("additional", "additional_file.xsd", EMPTY_HEADER, false);
    // MeanDataEdges
    writeMeanDataEdgeComment(device);
    writeMeanDatas(device, SUMO_TAG_MEANDATA_EDGE);
    // MeanDataLanes
    writeMeanDataLaneComment(device);
    writeMeanDatas(device, SUMO_TAG_MEANDATA_LANE);
    // close device
    device.close();
    // mark mean datas as saved
    mySavingStatus->meanDatasSaved();
}


void
GNENet::writeAdditionalByType(OutputDevice& device, const std::vector<SumoXMLTag> tags) const {
    std::map<std::string, GNEAdditional*> sortedAdditionals;
    for (const auto& tag : tags) {
        for (const auto& additional : myAttributeCarriers->getAdditionals().at(tag)) {
            if (sortedAdditionals.count(additional.second->getID()) == 0) {
                sortedAdditionals[additional.second->getID()] = additional.second;
            } else {
                throw ProcessError(TL("Duplicated ID"));
            }
        }
    }
    for (const auto& additional : sortedAdditionals) {
        additional.second->writeAdditional(device);
    }
}


void
GNENet::writeDemandByType(OutputDevice& device, SumoXMLTag tag) const {
    std::map<std::string, GNEDemandElement*> sortedDemandElements;
    for (const auto& demandElement : myAttributeCarriers->getDemandElements().at(tag)) {
        sortedDemandElements[demandElement.second->getID()] = demandElement.second;
    }
    for (const auto& demandElement : sortedDemandElements) {
        demandElement.second->writeDemandElement(device);
    }
}


void
GNENet::writeRouteDistributions(OutputDevice& device, const bool additionalFile) const {
    std::map<std::string, GNEDemandElement*> sortedElements;
    // first write route Distributions
    for (const auto& routeDistribution : myAttributeCarriers->getDemandElements().at(SUMO_TAG_ROUTE_DISTRIBUTION)) {
        // get number of additional children
        const auto numChildren = routeDistribution.second->getAttributeDouble(GNE_ATTR_ADDITIONALCHILDREN);
        if ((additionalFile && (numChildren != 0)) || (!additionalFile && (numChildren == 0))) {
            sortedElements[routeDistribution.second->getID()] = routeDistribution.second;
        }
    }
    for (const auto& element : sortedElements) {
        element.second->writeDemandElement(device);
    }
    sortedElements.clear();
}


void
GNENet::writeRoutes(OutputDevice& device, const bool additionalFile) const {
    std::map<std::string, GNEDemandElement*> sortedRoutes;
    for (const auto& route : myAttributeCarriers->getDemandElements().at(SUMO_TAG_ROUTE)) {
        if ((additionalFile && (route.second->getChildAdditionals().size() > 0)) ||
                (!additionalFile && (route.second->getChildAdditionals().size() == 0))) {
            sortedRoutes[route.second->getID()] = route.second;
        }
    }
    for (const auto& route : sortedRoutes) {
        route.second->writeDemandElement(device);
    }
}


void
GNENet::writeVTypeDistributions(OutputDevice& device, const bool additionalFile) const {
    std::map<std::string, GNEDemandElement*> sortedElements;
    // first write vType Distributions
    for (const auto& vTypeDistribution : myAttributeCarriers->getDemandElements().at(SUMO_TAG_VTYPE_DISTRIBUTION)) {
        // get number of additional children
        const auto numChildren = vTypeDistribution.second->getAttributeDouble(GNE_ATTR_ADDITIONALCHILDREN);
        if ((additionalFile && (numChildren != 0)) || (!additionalFile && (numChildren == 0))) {
            sortedElements[vTypeDistribution.second->getID()] = vTypeDistribution.second;
        }
    }
    for (const auto& element : sortedElements) {
        element.second->writeDemandElement(device);
    }
    sortedElements.clear();
}


void
GNENet::writeVTypes(OutputDevice& device, const bool additionalFile) const {
    std::map<std::string, GNEDemandElement*> sortedElements;
    // write vTypes
    for (const auto& vType : myAttributeCarriers->getDemandElements().at(SUMO_TAG_VTYPE)) {
        // get number of additional children
        const auto numChildren = vType.second->getChildAdditionals().size();
        if ((additionalFile && (numChildren != 0)) || (!additionalFile && (numChildren == 0))) {
            sortedElements[vType.second->getID()] = vType.second;
        }
    }
    for (const auto& element : sortedElements) {
        element.second->writeDemandElement(device);
    }
}


void
GNENet::writeMeanDatas(OutputDevice& device, SumoXMLTag tag) const {
    std::map<std::string, GNEMeanData*> sortedMeanDatas;
    for (const auto& meanData : myAttributeCarriers->getMeanDatas().at(tag)) {
        if (sortedMeanDatas.count(meanData.second->getID()) == 0) {
            sortedMeanDatas[meanData.second->getID()] = meanData.second;
        } else {
            throw ProcessError(TL("Duplicated ID"));
        }
    }
    for (const auto& additional : sortedMeanDatas) {
        additional.second->writeMeanData(device);
    }
}

bool
GNENet::writeVTypeComment(OutputDevice& device, const bool additionalFile) const {
    // vTypes
    for (const auto& vType : myAttributeCarriers->getDemandElements().at(SUMO_TAG_VTYPE)) {
        // special case for default vTypes
        const bool defaultVType = GNEAttributeCarrier::parse<bool>(vType.second->getAttribute(GNE_ATTR_DEFAULT_VTYPE));
        const bool defaultVTypeModified = GNEAttributeCarrier::parse<bool>(vType.second->getAttribute(GNE_ATTR_DEFAULT_VTYPE_MODIFIED));
        // only write default vType modified
        if ((vType.second->getParentDemandElements().size() == 0) && (!defaultVType || (defaultVType && defaultVTypeModified))) {
            if (additionalFile && (vType.second->getChildAdditionals().size() != 0)) {
                device << ("    <!-- VTypes (used in calibratorFlows) -->\n");
                return true;
            } else if (!additionalFile && (vType.second->getChildAdditionals().size() == 0)) {
                device << ("    <!-- VTypes -->\n");
                return true;
            }
        }
    }
    return false;
}


bool
GNENet::writeRouteComment(OutputDevice& device, const bool additionalFile) const {
    for (const auto& route : myAttributeCarriers->getDemandElements().at(SUMO_TAG_ROUTE)) {
        if (additionalFile && (route.second->getChildAdditionals().size() != 0)) {
            device << ("    <!-- Routes (used in RouteProbReroutes and calibratorFlows) -->\n");
            return true;
        } else if (!additionalFile && (route.second->getChildAdditionals().size() == 0)) {
            device << ("    <!-- Routes -->\n");
            return true;
        }
    }
    return false;
}


bool
GNENet::writeRouteProbeComment(OutputDevice& device) const {
    if (myAttributeCarriers->getAdditionals().at(SUMO_TAG_ROUTEPROBE).size() > 0) {
        device << ("    <!-- RouteProbes -->\n");
        return true;
    }
    return false;
}


bool
GNENet::writeCalibratorComment(OutputDevice& device) const {
    for (const auto& additionals : myAttributeCarriers->getAdditionals()) {
        if (GNEAttributeCarrier::getTagProperty(additionals.first).isCalibrator() && (additionals.second.size() > 0)) {
            device << ("    <!-- Calibrators -->\n");
            return true;
        }
    }
    return false;
}


bool
GNENet::writeStoppingPlaceComment(OutputDevice& device) const {
    for (const auto& additionals : myAttributeCarriers->getAdditionals()) {
        if (GNEAttributeCarrier::getTagProperty(additionals.first).isStoppingPlace() && (additionals.second.size() > 0)) {
            device << ("    <!-- StoppingPlaces -->\n");
            return true;
        }
    }
    return false;
}


bool
GNENet::writeDetectorComment(OutputDevice& device) const {
    for (const auto& additionals : myAttributeCarriers->getAdditionals()) {
        if (GNEAttributeCarrier::getTagProperty(additionals.first).isDetector() && (additionals.second.size() > 0)) {
            device << ("    <!-- Detectors -->\n");
            return true;
        }
    }
    return false;
}


bool
GNENet::writeOtherAdditionalsComment(OutputDevice& device) const {
    for (const auto& additionals : myAttributeCarriers->getAdditionals()) {
        if (GNEAttributeCarrier::getTagProperty(additionals.first).isAdditionalPureElement() &&
                !GNEAttributeCarrier::getTagProperty(additionals.first).isStoppingPlace() &&
                !GNEAttributeCarrier::getTagProperty(additionals.first).isDetector() &&
                !GNEAttributeCarrier::getTagProperty(additionals.first).isCalibrator() &&
                (additionals.first != SUMO_TAG_ROUTEPROBE) && (additionals.first != SUMO_TAG_ACCESS) &&
                (additionals.first != SUMO_TAG_PARKING_SPACE) && (additionals.second.size() > 0)) {
            device << ("    <!-- Other additionals -->\n");
            return true;
        }
    }
    return false;
}


bool
GNENet::writeShapesComment(OutputDevice& device) const {
    for (const auto& additionals : myAttributeCarriers->getAdditionals()) {
        if (GNEAttributeCarrier::getTagProperty(additionals.first).isShapeElement() &&
                !GNEAttributeCarrier::getTagProperty(additionals.first).isJuPedSimElement() &&
                (additionals.second.size() > 0)) {
            device << ("    <!-- Shapes -->\n");
            return true;
        }
    }
    return false;
}


bool
GNENet::writeJuPedSimComment(OutputDevice& device) const {
    for (const auto& additionals : myAttributeCarriers->getAdditionals()) {
        if (GNEAttributeCarrier::getTagProperty(additionals.first).isJuPedSimElement() && (additionals.second.size() > 0)) {
            device << ("    <!-- JuPedSim elements -->\n");
            return true;
        }
    }
    return false;
}


bool
GNENet::writeTAZComment(OutputDevice& device) const {
    if (myAttributeCarriers->getAdditionals().at(SUMO_TAG_TAZ).size() > 0) {
        device << ("    <!-- TAZs -->\n");
        return true;
    }
    return false;
}


bool
GNENet::writeWireComment(OutputDevice& device) const {
    if (myAttributeCarriers->getAdditionals().at(SUMO_TAG_TRACTION_SUBSTATION).size() > 0) {
        device << ("    <!-- Wires -->\n");
        return true;
    }
    return false;
}


bool
GNENet::writeMeanDataEdgeComment(OutputDevice& device) const {
    if (myAttributeCarriers->getMeanDatas().at(SUMO_TAG_MEANDATA_EDGE).size() > 0) {
        device << ("    <!-- MeanDataEdges -->\n");
        return true;
    }
    return false;
}


bool
GNENet::writeMeanDataLaneComment(OutputDevice& device) const {
    if (myAttributeCarriers->getMeanDatas().at(SUMO_TAG_MEANDATA_LANE).size() > 0) {
        device << ("    <!-- MeanDataLanes -->\n");
        return true;
    }
    return false;
}


void
GNENet::saveTLSPrograms(const std::string& filename) {
    // open output device
    OutputDevice& device = OutputDevice::getDevice(filename);
    device.openTag("additionals");
    // write traffic lights using NWWriter
    NWWriter_SUMO::writeTrafficLights(device, getTLLogicCont());
    device.close();
    // change save status
    mySavingStatus->TLSSaved();
    // show debug information
    WRITE_DEBUG("TLSPrograms saved");
}


int
GNENet::getNumberOfTLSPrograms() const {
    return -1;
}


void
GNENet::saveEdgeTypes(const std::string& filename) {
    // first clear typeContainer
    myNetBuilder->getTypeCont().clearTypes();
    // now update typeContainer with edgeTypes
    for (const auto& edgeType : myAttributeCarriers->getEdgeTypes()) {
        myNetBuilder->getTypeCont().insertEdgeType(edgeType.first, edgeType.second);
        for (int i = 0; i < (int)edgeType.second->getLaneTypes().size(); i++) {
            myNetBuilder->getTypeCont().insertLaneType(edgeType.first, i,
                    edgeType.second->getLaneTypes().at(i)->speed,
                    edgeType.second->getLaneTypes().at(i)->permissions,
                    edgeType.second->getLaneTypes().at(i)->width,
                    edgeType.second->getLaneTypes().at(i)->attrs);
        }
    }
    // open device
    OutputDevice& device = OutputDevice::getDevice(filename);
    // open tag
    device.openTag(SUMO_TAG_TYPE);
    // write edge types
    myNetBuilder->getTypeCont().writeEdgeTypes(device);
    // close tag
    device.closeTag();
    // close device
    device.close();
}


void
GNENet::enableUpdateGeometry() {
    myUpdateGeometryEnabled = true;
}


void
GNENet::disableUpdateGeometry() {
    myUpdateGeometryEnabled = false;
}


bool
GNENet::isUpdateGeometryEnabled() const {
    return myUpdateGeometryEnabled;
}


void
GNENet::enableUpdateData() {
    myUpdateDataEnabled = true;
    // update data elements
    for (const auto& dataInterval : myAttributeCarriers->getDataIntervals()) {
        dataInterval.second->updateGenericDataIDs();
        dataInterval.second->updateAttributeColors();
    }
}


void
GNENet::disableUpdateData() {
    myUpdateDataEnabled = false;
}


bool
GNENet::isUpdateDataEnabled() const {
    return myUpdateDataEnabled;
}

// ===========================================================================
// private
// ===========================================================================

void
GNENet::initJunctionsAndEdges() {
    // init edge types
    for (const auto& edgeType : myNetBuilder->getTypeCont()) {
        // register edge type
        myAttributeCarriers->registerEdgeType(new GNEEdgeType(this, edgeType.first, edgeType.second));
    }
    // init junctions (by default Crossing and walking areas aren't created)
    for (const auto& nodeName : myNetBuilder->getNodeCont().getAllNames()) {
        // create and register junction
        myAttributeCarriers->registerJunction(new GNEJunction(this, myNetBuilder->getNodeCont().retrieve(nodeName), true));
    }
    // init edges
    for (const auto& edgeName : myNetBuilder->getEdgeCont().getAllNames()) {
        // create edge using NBEdge
        GNEEdge* edge = new GNEEdge(this, myNetBuilder->getEdgeCont().retrieve(edgeName), false, true);
        // register edge
        myAttributeCarriers->registerEdge(edge);
        // add manually child references due initJunctionsAndEdges doesn't use undo-redo
        edge->getFromJunction()->addChildElement(edge);
        edge->getToJunction()->addChildElement(edge);
        // check grid
        if (myGrid.getWidth() > 10e16 || myGrid.getHeight() > 10e16) {
            throw ProcessError(TL("Network size exceeds 1 Lightyear. Please reconsider your inputs.") + std::string("\n"));
        }
    }
    // make sure myGrid is initialized even for an empty net. This ensure that the network starts with a zoom of 100
    if (myAttributeCarriers->getEdges().size() == 0) {
        myGrid.add(Boundary(-50, -50, 50, 50));
    }
    // recalculate all lane2lane connections
    for (const auto& edge : myAttributeCarriers->getEdges()) {
        for (const auto& lane : edge.second->getLanes()) {
            lane->updateGeometry();
        }
    }
    // sort nodes edges so that arrows can be drawn correctly
    NBNodesEdgesSorter::sortNodesEdges(myNetBuilder->getNodeCont());
}


void
GNENet::initGNEConnections() {
    for (const auto& edge : myAttributeCarriers->getEdges()) {
        // remake connections
        edge.second->remakeGNEConnections();
        // update geometry of connections
        for (const auto& connection : edge.second->getGNEConnections()) {
            connection->updateGeometry();
        }
    }
}


void
GNENet::computeAndUpdate(OptionsCont& neteditOptions, bool volatileOptions) {
    // make sure we only add turn arounds to edges which currently exist within the network
    std::set<std::string> liveExplicitTurnarounds;
    for (const auto& explicitTurnarounds : myExplicitTurnarounds) {
        if (myAttributeCarriers->getEdges().count(explicitTurnarounds) > 0) {
            liveExplicitTurnarounds.insert(explicitTurnarounds);
        }
    }
    // removes all junctions of grid
    WRITE_GLDEBUG("Removing junctions during recomputing");
    for (const auto& junction : myAttributeCarriers->getJunctions()) {
        removeGLObjectFromGrid(junction.second);
    }
    // remove all edges from grid
    WRITE_GLDEBUG("Removing edges during recomputing");
    for (const auto& edge : myAttributeCarriers->getEdges()) {
        removeGLObjectFromGrid(edge.second);
    }
    // compute using NetBuilder
    myNetBuilder->compute(neteditOptions, liveExplicitTurnarounds, volatileOptions);
    // remap ids if necessary
    if (neteditOptions.getBool("numerical-ids") || neteditOptions.isSet("reserved-ids")) {
        myAttributeCarriers->remapJunctionAndEdgeIds();
    }
    // update rtree if necessary
    if (!neteditOptions.getBool("offset.disable-normalization")) {
        for (const auto& edge : myAttributeCarriers->getEdges()) {
            // refresh edge geometry
            edge.second->updateGeometry();
        }
    }
    // Clear current inspected ACs in inspectorFrame if a previous net was loaded
    if (myViewNet != nullptr) {
        myViewNet->getViewParent()->getInspectorFrame()->clearInspectedAC();
    }
    // Reset Grid
    myGrid.reset();
    myGrid.add(GeoConvHelper::getFinal().getConvBoundary());
    // if volatile options are true
    if (volatileOptions) {
        // check that net exist
        if (myViewNet == nullptr) {
            throw ProcessError("ViewNet doesn't exist");
        }
        // disable update geometry before clear undo list
        myUpdateGeometryEnabled = false;
        // destroy Popup
        myViewNet->destroyPopup();
        // clear undo list (This will be remove additionals and shapes)
        myViewNet->getUndoList()->clear();
        // clear all elements (it will also removed from grid)
        myAttributeCarriers->clearJunctions();
        myAttributeCarriers->clearEdges();
        myAttributeCarriers->clearAdditionals();
        myAttributeCarriers->clearDemandElements();
        // enable update geometry again
        myUpdateGeometryEnabled = true;
        // Write GL debug information
        WRITE_GLDEBUG("initJunctionsAndEdges function called in computeAndUpdate(...) due recomputing with volatile options");
        // init again junction an edges (Additionals and shapes will be loaded after the end of this function)
        initJunctionsAndEdges();
        // init default vTypes again
        myAttributeCarriers->addDefaultVTypes();
    } else {
        // insert all junctions of grid again
        WRITE_GLDEBUG("Add junctions during recomputing after calling myNetBuilder->compute(...)");
        for (const auto& junction : myAttributeCarriers->getJunctions()) {
            // update centering boundary
            junction.second->updateCenteringBoundary(false);
            // add junction in grid again
            addGLObjectIntoGrid(junction.second);
        }
        // insert all edges from grid again
        WRITE_GLDEBUG("Add edges during recomputing after calling myNetBuilder->compute(...)");
        for (const auto& edge : myAttributeCarriers->getEdges()) {
            // update centeting boundary
            edge.second->updateCenteringBoundary(false);
            // add edge in grid again
            addGLObjectIntoGrid(edge.second);
        }
        // remake connections
        for (const auto& edge : myAttributeCarriers->getEdges()) {
            edge.second->remakeGNEConnections(true);
        }
        // iterate over junctions of net
        for (const auto& junction : myAttributeCarriers->getJunctions()) {
            // undolist may not yet exist but is also not needed when just marking junctions as valid
            junction.second->setLogicValid(true, nullptr);
            // updated geometry
            junction.second->updateGeometryAfterNetbuild();
            // rebuild walking areas
            junction.second->rebuildGNEWalkingAreas();
        }
        // iterate over all edges of net
        for (const auto& edge : myAttributeCarriers->getEdges()) {
            // update geometry
            edge.second->updateGeometry();
        }
    }
    // net recomputed, then return false;
    myNeedRecompute = false;
}


void
GNENet::replaceInListAttribute(GNEAttributeCarrier* ac, SumoXMLAttr key, const std::string& which, const std::string& by, GNEUndoList* undoList) {
    assert(ac->getTagProperty().getAttributeProperties(key).isList());
    std::vector<std::string> values = GNEAttributeCarrier::parse<std::vector<std::string> >(ac->getAttribute(key));
    std::vector<std::string> newValues;
    bool lastBy = false;
    for (auto v : values) {
        if (v == which && !lastBy) {
            // avoid duplicate occurence of the 'by' edge (i.e. in routes)
            newValues.push_back(by);
        } else {
            newValues.push_back(v);
        }
        lastBy = v == by;
    }
    ac->setAttribute(key, toString(newValues), undoList);
}


/****************************************************************************/
