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
/// @file    GNEViewNetHelper.cpp
/// @author  Jakob Erdmann
/// @author  Pablo Alvarez Lopez
/// @date    Feb 2019
///
// A file used to reduce the size of GNEViewNet.h grouping structs and classes
/****************************************************************************/
#include <netedit/elements/additional/GNEPOI.h>
#include <netedit/elements/additional/GNEPoly.h>
#include <netedit/elements/additional/GNETAZ.h>
#include <netedit/elements/data/GNEDataInterval.h>
#include <netedit/elements/data/GNEEdgeData.h>
#include <netedit/elements/data/GNEEdgeRelData.h>
#include <netedit/elements/data/GNETAZRelData.h>
#include <netedit/elements/network/GNEConnection.h>
#include <netedit/elements/network/GNECrossing.h>
#include <netedit/elements/network/GNEWalkingArea.h>
#include <netedit/elements/network/GNEInternalLane.h>
#include <netedit/frames/common/GNESelectorFrame.h>
#include <netedit/frames/network/GNETLSEditorFrame.h>
#include <utils/foxtools/MFXMenuCheckIcon.h>
#include <utils/gui/div/GLHelper.h>
#include <utils/gui/div/GUIDesigns.h>
#include <utils/gui/div/GUIGlobalViewObjectsHandler.h>
#include <utils/options/OptionsCont.h>

#include "GNEViewNetHelper.h"
#include "GNEViewNet.h"
#include "GNENet.h"
#include "GNEUndoList.h"
#include "GNEViewParent.h"
#include "GNEApplicationWindow.h"


// ===========================================================================
// static members
// ===========================================================================

std::vector<RGBColor> GNEViewNetHelper::myRainbowScaledColors;

// ---------------------------------------------------------------------------
// GNEViewNetHelper::LockManager - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::LockManager::LockManager(GNEViewNet* viewNet) :
    myViewNet(viewNet) {
    // fill myLockedElements objects
    myLockedElements[GLO_JUNCTION] = OperationLocked(Supermode::NETWORK);
    myLockedElements[GLO_EDGE] = OperationLocked(Supermode::NETWORK);
    myLockedElements[GLO_LANE] = OperationLocked(Supermode::NETWORK);
    myLockedElements[GLO_CONNECTION] = OperationLocked(Supermode::NETWORK);
    myLockedElements[GLO_CROSSING] = OperationLocked(Supermode::NETWORK);
    myLockedElements[GLO_WALKINGAREA] = OperationLocked(Supermode::NETWORK);
    myLockedElements[GLO_ADDITIONALELEMENT] = OperationLocked(Supermode::NETWORK);
    myLockedElements[GLO_WIRE] = OperationLocked(Supermode::NETWORK);
    myLockedElements[GLO_TAZ] = OperationLocked(Supermode::NETWORK);
    myLockedElements[GLO_WIRE] = OperationLocked(Supermode::NETWORK);
    myLockedElements[GLO_POLYGON] = OperationLocked(Supermode::NETWORK);
    myLockedElements[GLO_POI] = OperationLocked(Supermode::NETWORK);
    myLockedElements[GLO_JPS_WALKABLEAREA] = OperationLocked(Supermode::NETWORK);
    myLockedElements[GLO_JPS_OBSTACLE] = OperationLocked(Supermode::NETWORK);
    // fill myLockedElements objects
    myLockedElements[GLO_ROUTE] = OperationLocked(Supermode::DEMAND);
    myLockedElements[GLO_VEHICLE] = OperationLocked(Supermode::DEMAND);
    myLockedElements[GLO_PERSON] = OperationLocked(Supermode::DEMAND);
    myLockedElements[GLO_PERSONTRIP] = OperationLocked(Supermode::DEMAND);
    myLockedElements[GLO_RIDE] = OperationLocked(Supermode::DEMAND);
    myLockedElements[GLO_WALK] = OperationLocked(Supermode::DEMAND);
    myLockedElements[GLO_CONTAINER] = OperationLocked(Supermode::DEMAND);
    myLockedElements[GLO_TRANSPORT] = OperationLocked(Supermode::DEMAND);
    myLockedElements[GLO_TRANSHIP] = OperationLocked(Supermode::DEMAND);
    myLockedElements[GLO_STOP] = OperationLocked(Supermode::DEMAND);
    // fill myLockedElements objects
    myLockedElements[GLO_EDGEDATA] = OperationLocked(Supermode::DATA);
    myLockedElements[GLO_EDGERELDATA] = OperationLocked(Supermode::DATA);
    myLockedElements[GLO_TAZRELDATA] = OperationLocked(Supermode::DATA);
}


GNEViewNetHelper::LockManager::~LockManager() {}


bool
GNEViewNetHelper::LockManager::isObjectLocked(GUIGlObjectType objectType, const bool selected) const {
    if (selected && (myViewNet->getViewParent()->getGNEAppWindows()->getLockMenuCommands().menuCheckLockSelectedElements->getCheck() == TRUE)) {
        return true;
    } else if ((objectType >= GLO_ADDITIONALELEMENT) && (objectType <= GLO_ACCESS)) {
        // additionals
        return myLockedElements.at(GLO_ADDITIONALELEMENT).lock;
    } else if ((objectType >= GLO_WIRE) && (objectType <= GLO_TRACTIONSUBSTATION)) {
        // wires
        return myLockedElements.at(GLO_WIRE).lock;
    } else if ((objectType == GLO_ROUTE) || (objectType == GLO_ROUTE_EMBEDDED)) {
        // routes
        return myLockedElements.at(GLO_ROUTE).lock;
    } else if ((objectType >= GLO_VEHICLE) && (objectType <= GLO_ROUTEFLOW)) {
        // vehicles
        return myLockedElements.at(GLO_VEHICLE).lock;
    } else if ((objectType == GLO_PERSON) || (objectType == GLO_PERSONFLOW)) {
        // persons
        return myLockedElements.at(GLO_PERSON).lock;
    } else if ((objectType == GLO_CONTAINER) || (objectType == GLO_CONTAINERFLOW)) {
        // containers
        return myLockedElements.at(GLO_CONTAINER).lock;
    } else if ((objectType >= GLO_STOP) && (objectType <= GLO_STOP_PLAN)) {
        // stops
        return myLockedElements.at(GLO_STOP).lock;
    } else {
        return myLockedElements.at(objectType).lock;
    }
}


void
GNEViewNetHelper::LockManager::updateFlags() {
    // get lock menu commands
    GNEApplicationWindowHelper::LockMenuCommands& lockMenuCommands = myViewNet->getViewParent()->getGNEAppWindows()->getLockMenuCommands();
    // network
    myLockedElements[GLO_JUNCTION].lock = lockMenuCommands.menuCheckLockJunction->getCheck() == TRUE;
    myLockedElements[GLO_EDGE].lock = lockMenuCommands.menuCheckLockEdges->getCheck() == TRUE;
    myLockedElements[GLO_LANE].lock = lockMenuCommands.menuCheckLockLanes->getCheck() == TRUE;
    myLockedElements[GLO_CONNECTION].lock = lockMenuCommands.menuCheckLockConnections->getCheck() == TRUE;
    myLockedElements[GLO_CROSSING].lock = lockMenuCommands.menuCheckLockCrossings->getCheck() == TRUE;
    myLockedElements[GLO_WALKINGAREA].lock = lockMenuCommands.menuCheckLockWalkingAreas->getCheck() == TRUE;
    myLockedElements[GLO_ADDITIONALELEMENT].lock = lockMenuCommands.menuCheckLockAdditionals->getCheck() == TRUE;
    myLockedElements[GLO_WIRE].lock = lockMenuCommands.menuCheckLockWires->getCheck() == TRUE;
    myLockedElements[GLO_TAZ].lock = lockMenuCommands.menuCheckLockTAZs->getCheck() == TRUE;
    myLockedElements[GLO_POLYGON].lock = lockMenuCommands.menuCheckLockPolygons->getCheck() == TRUE;
    myLockedElements[GLO_POI].lock = lockMenuCommands.menuCheckLockPOIs->getCheck() == TRUE;
    myLockedElements[GLO_JPS_WALKABLEAREA].lock = lockMenuCommands.menuCheckLockJpsWalkableAreas->getCheck() == TRUE;
    myLockedElements[GLO_JPS_OBSTACLE].lock = lockMenuCommands.menuCheckLockJpsObstacles->getCheck() == TRUE;
    // demand
    myLockedElements[GLO_ROUTE].lock = lockMenuCommands.menuCheckLockRoutes->getCheck() == TRUE;
    myLockedElements[GLO_VEHICLE].lock = lockMenuCommands.menuCheckLockVehicles->getCheck() == TRUE;
    myLockedElements[GLO_PERSON].lock = lockMenuCommands.menuCheckLockPersons->getCheck() == TRUE;
    myLockedElements[GLO_PERSONTRIP].lock = lockMenuCommands.menuCheckLockPersonTrip->getCheck() == TRUE;
    myLockedElements[GLO_WALK].lock = lockMenuCommands.menuCheckLockWalk->getCheck() == TRUE;
    myLockedElements[GLO_RIDE].lock = lockMenuCommands.menuCheckLockRides->getCheck() == TRUE;
    myLockedElements[GLO_CONTAINER].lock = lockMenuCommands.menuCheckLockContainers->getCheck() == TRUE;
    myLockedElements[GLO_TRANSPORT].lock = lockMenuCommands.menuCheckLockTransports->getCheck() == TRUE;
    myLockedElements[GLO_TRANSHIP].lock = lockMenuCommands.menuCheckLockTranships->getCheck() == TRUE;
    myLockedElements[GLO_STOP].lock = lockMenuCommands.menuCheckLockStops->getCheck() == TRUE;
    // data
    myLockedElements[GLO_EDGEDATA].lock = lockMenuCommands.menuCheckLockEdgeDatas->getCheck() == TRUE;
    myLockedElements[GLO_EDGERELDATA].lock = lockMenuCommands.menuCheckLockEdgeRelDatas->getCheck() == TRUE;
    myLockedElements[GLO_TAZRELDATA].lock = lockMenuCommands.menuCheckLockEdgeTAZRels->getCheck() == TRUE;
}


void
GNEViewNetHelper::LockManager::updateLockMenuBar() {
    // get lock menu commands
    GNEApplicationWindowHelper::LockMenuCommands& lockMenuCommands = myViewNet->getViewParent()->getGNEAppWindows()->getLockMenuCommands();
    // network
    lockMenuCommands.menuCheckLockJunction->setCheck(myLockedElements[GLO_JUNCTION].lock);
    lockMenuCommands.menuCheckLockEdges->setCheck(myLockedElements[GLO_EDGE].lock);
    lockMenuCommands.menuCheckLockLanes->setCheck(myLockedElements[GLO_LANE].lock);
    lockMenuCommands.menuCheckLockConnections->setCheck(myLockedElements[GLO_CONNECTION].lock);
    lockMenuCommands.menuCheckLockCrossings->setCheck(myLockedElements[GLO_CROSSING].lock);
    lockMenuCommands.menuCheckLockWalkingAreas->setCheck(myLockedElements[GLO_WALKINGAREA].lock);
    lockMenuCommands.menuCheckLockAdditionals->setCheck(myLockedElements[GLO_ADDITIONALELEMENT].lock);
    lockMenuCommands.menuCheckLockWires->setCheck(myLockedElements[GLO_WIRE].lock);
    lockMenuCommands.menuCheckLockTAZs->setCheck(myLockedElements[GLO_TAZ].lock);
    lockMenuCommands.menuCheckLockPolygons->setCheck(myLockedElements[GLO_POLYGON].lock);
    lockMenuCommands.menuCheckLockPOIs->setCheck(myLockedElements[GLO_POI].lock);
    lockMenuCommands.menuCheckLockJpsWalkableAreas->setCheck(myLockedElements[GLO_JPS_WALKABLEAREA].lock);
    lockMenuCommands.menuCheckLockJpsObstacles->setCheck(myLockedElements[GLO_JPS_OBSTACLE].lock);
    // demand
    lockMenuCommands.menuCheckLockRoutes->setCheck(myLockedElements[GLO_ROUTE].lock);
    lockMenuCommands.menuCheckLockVehicles->setCheck(myLockedElements[GLO_VEHICLE].lock);
    lockMenuCommands.menuCheckLockPersons->setCheck(myLockedElements[GLO_PERSON].lock);
    lockMenuCommands.menuCheckLockPersonTrip->setCheck(myLockedElements[GLO_PERSONTRIP].lock);
    lockMenuCommands.menuCheckLockWalk->setCheck(myLockedElements[GLO_WALK].lock);
    lockMenuCommands.menuCheckLockRides->setCheck(myLockedElements[GLO_RIDE].lock);
    lockMenuCommands.menuCheckLockContainers->setCheck(myLockedElements[GLO_CONTAINER].lock);
    lockMenuCommands.menuCheckLockTransports->setCheck(myLockedElements[GLO_TRANSPORT].lock);
    lockMenuCommands.menuCheckLockTranships->setCheck(myLockedElements[GLO_TRANSHIP].lock);
    lockMenuCommands.menuCheckLockStops->setCheck(myLockedElements[GLO_STOP].lock);
    // data
    lockMenuCommands.menuCheckLockEdgeDatas->setCheck(myLockedElements[GLO_EDGEDATA].lock);
    lockMenuCommands.menuCheckLockEdgeRelDatas->setCheck(myLockedElements[GLO_EDGERELDATA].lock);
    lockMenuCommands.menuCheckLockEdgeTAZRels->setCheck(myLockedElements[GLO_TAZRELDATA].lock);
}


GNEViewNetHelper::LockManager::OperationLocked::OperationLocked():
    mySupermode(Supermode::NETWORK) {
}


GNEViewNetHelper::LockManager::OperationLocked::OperationLocked(Supermode supermode) :
    mySupermode(supermode) {
}


GNEViewNetHelper::LockManager::OperationLocked::~OperationLocked() {}


Supermode
GNEViewNetHelper::LockManager::OperationLocked::getSupermode() const {
    return mySupermode;
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::ViewObjectsSelector - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::ViewObjectsSelector::ViewObjectsSelector(GNEViewNet* viewNet) :
    myViewNet(viewNet) {
}


void
GNEViewNetHelper::ViewObjectsSelector::updateObjects() {
    // clear elements
    myViewObjects.clearElements();
    // process GUIGLObjects using elements under cursor
    processGUIGlObjects(gViewObjectsHandler.getSelectedObjects());
}


void
GNEViewNetHelper::ViewObjectsSelector::filterEdges() {
    // get all edges to filter
    std::vector<const GUIGlObject*> edges;
    for (const auto& edge : myViewObjects.edges) {
        edges.push_back(edge);
    }
    myViewObjects.filterElements(edges);
}


void
GNEViewNetHelper::ViewObjectsSelector::filterLanes() {
    // get all lanes to filter
    std::vector<const GUIGlObject*> lanes;
    for (const auto& lane : myViewObjects.lanes) {
        lanes.push_back(lane);
    }
    myViewObjects.filterElements(lanes);
}


void
GNEViewNetHelper::ViewObjectsSelector::filterShapes() {
    // get all elements to filter
    std::vector<const GUIGlObject*> shapes;
    for (const auto& poly : myViewObjects.polys) {
        shapes.push_back(poly);
    }
    for (const auto& POI : myViewObjects.POIs) {
        shapes.push_back(POI);
    }
    myViewObjects.filterElements(shapes);
}


void
GNEViewNetHelper::ViewObjectsSelector::filterAdditionals(const bool includeStoppigPlaces, const bool includeTAZs) {
    // get all elements to filter
    std::vector<const GUIGlObject*> additionals;
    for (const auto& additional : myViewObjects.additionals) {
        if (!includeStoppigPlaces && (additional->getType() > GLO_STOPPING_PLACE) && (additional->getType() < GLO_STOPPING_PLACE_LAST)) {
            continue;
        } else if (!includeTAZs && (additional->getType() == GLO_TAZ)) {
            continue;
        } else {
            additionals.push_back(additional);
        }
    }
    myViewObjects.filterElements(additionals);
}


void
GNEViewNetHelper::ViewObjectsSelector::filterDemandElements(const bool includeRoutes) {
    // get all elements to filter
    std::vector<const GUIGlObject*> demandElements;
    for (const auto& demandElement : myViewObjects.demandElements) {
        if (!includeRoutes && (demandElement->getType() == GLO_ROUTE)) {
            continue;
        } else {
            demandElements.push_back(demandElement);
        }
    }
    myViewObjects.filterElements(demandElements);
}


void
GNEViewNetHelper::ViewObjectsSelector::filterLockedElements(const std::vector<GUIGlObjectType> ignoreFilter) {
    std::vector<const GUIGlObject*> GUIGlObjects;
    // get all locked elements
    for (const auto& GUIGlObject : myViewObjects.GUIGlObjects) {
        if (GUIGlObject->isGLObjectLocked()) {
            GUIGlObjects.push_back(GUIGlObject);
        }
    }
    // apply ignore filter
    for (const auto& ignoredType : ignoreFilter) {
        auto it = GUIGlObjects.begin();
        while (it != GUIGlObjects.end()) {
            if ((*it)->getType() == ignoredType) {
                it = GUIGlObjects.erase(it);
            } else {
                it++;
            }
        }
    }
    // filter objects
    myViewObjects.filterElements(GUIGlObjects);
}


const GUIGlObject*
GNEViewNetHelper::ViewObjectsSelector::getGUIGlObjectFront() const {
    if (myViewObjects.GUIGlObjects.size() > 0) {
        return myViewObjects.GUIGlObjects.front();
    } else {
        return nullptr;
    }
}


GNEAttributeCarrier*
GNEViewNetHelper::ViewObjectsSelector::getAttributeCarrierFront() const {
    if (myViewObjects.attributeCarriers.size() > 0) {
        return myViewObjects.attributeCarriers.front();
    } else {
        return nullptr;
    }
}


GNENetworkElement*
GNEViewNetHelper::ViewObjectsSelector::getNetworkElementFront() const {
    if (myViewObjects.networkElements.size() > 0) {
        return myViewObjects.networkElements.front();
    } else {
        return nullptr;
    }
}


GNEAdditional*
GNEViewNetHelper::ViewObjectsSelector::getAdditionalFront() const {
    if (myViewObjects.additionals.size() > 0) {
        return myViewObjects.additionals.front();
    } else {
        return nullptr;
    }
}


GNEDemandElement*
GNEViewNetHelper::ViewObjectsSelector::getDemandElementFront() const {
    if (myViewObjects.demandElements.size() > 0) {
        return myViewObjects.demandElements.front();
    } else {
        return nullptr;
    }
}


GNEGenericData*
GNEViewNetHelper::ViewObjectsSelector::getGenericDataElementFront() const {
    if (myViewObjects.genericDatas.size() > 0) {
        return myViewObjects.genericDatas.front();
    } else {
        return nullptr;
    }
}


GNEJunction*
GNEViewNetHelper::ViewObjectsSelector::getJunctionFront() const {
    if (myViewObjects.junctions.size() > 0) {
        return myViewObjects.junctions.front();
    } else {
        return nullptr;
    }
}


GNEEdge*
GNEViewNetHelper::ViewObjectsSelector::getEdgeFront() const {
    if (myViewObjects.edges.size() > 0) {
        return myViewObjects.edges.front();
    } else {
        return nullptr;
    }
}


GNELane*
GNEViewNetHelper::ViewObjectsSelector::getLaneFront() const {
    if (myViewObjects.lanes.size() > 0) {
        return myViewObjects.lanes.front();
    } else {
        return nullptr;
    }
}


GNELane*
GNEViewNetHelper::ViewObjectsSelector::getLaneFrontNonLocked() const {
    if (myViewObjects.lanes.size() > 0) {
        for (auto& lane : myViewObjects.lanes) {
            if (!(lane->isAttributeCarrierSelected() || lane->getParentEdge()->isAttributeCarrierSelected()) ||
                    !myViewNet->getViewParent()->getGNEAppWindows()->getLockMenuCommands().menuCheckLockSelectedElements->getCheck()) {
                return lane;
            }
        }
        // all locked, then return nullptr
        return nullptr;
    } else {
        return nullptr;
    }
}


const std::vector<GNELane*>&
GNEViewNetHelper::ViewObjectsSelector::getLanes() const {
    return myViewObjects.lanes;
}


GNECrossing*
GNEViewNetHelper::ViewObjectsSelector::getCrossingFront() const {
    if (myViewObjects.crossings.size() > 0) {
        return myViewObjects.crossings.front();
    } else {
        return nullptr;
    }
}


GNEWalkingArea*
GNEViewNetHelper::ViewObjectsSelector::getWalkingAreaFront() const {
    if (myViewObjects.walkingAreas.size() > 0) {
        return myViewObjects.walkingAreas.front();
    } else {
        return nullptr;
    }
}


GNEConnection*
GNEViewNetHelper::ViewObjectsSelector::getConnectionFront() const {
    if (myViewObjects.connections.size() > 0) {
        return myViewObjects.connections.front();
    } else {
        return nullptr;
    }
}


GNEInternalLane*
GNEViewNetHelper::ViewObjectsSelector::getInternalLaneFront() const {
    if (myViewObjects.internalLanes.size() > 0) {
        return myViewObjects.internalLanes.front();
    } else {
        return nullptr;
    }
}


GNEPOI*
GNEViewNetHelper::ViewObjectsSelector::getPOIFront() const {
    if (myViewObjects.POIs.size() > 0) {
        return myViewObjects.POIs.front();
    } else {
        return nullptr;
    }
}


GNEPoly*
GNEViewNetHelper::ViewObjectsSelector::getPolyFront() const {
    if (myViewObjects.polys.size() > 0) {
        return myViewObjects.polys.front();
    } else {
        return nullptr;
    }
}


GNETAZ*
GNEViewNetHelper::ViewObjectsSelector::getTAZFront() const {
    if (myViewObjects.TAZs.size() > 0) {
        return myViewObjects.TAZs.front();
    } else {
        return nullptr;
    }
}


GNEEdgeData*
GNEViewNetHelper::ViewObjectsSelector::getEdgeDataElementFront() const {
    if (myViewObjects.edgeDatas.size() > 0) {
        return myViewObjects.edgeDatas.front();
    } else {
        return nullptr;
    }
}


GNEEdgeRelData*
GNEViewNetHelper::ViewObjectsSelector::getEdgeRelDataElementFront() const {
    if (myViewObjects.edgeRelDatas.size() > 0) {
        return myViewObjects.edgeRelDatas.front();
    } else {
        return nullptr;
    }
}


GNETAZRelData*
GNEViewNetHelper::ViewObjectsSelector::getTAZRelDataElementFront() const {
    if (myViewObjects.TAZRelDatas.size() > 0) {
        return myViewObjects.TAZRelDatas.front();
    } else {
        return nullptr;
    }
}


const std::vector<GUIGlObject*>&
GNEViewNetHelper::ViewObjectsSelector::getGLObjects() const {
    return myViewObjects.GUIGlObjects;
}


const std::vector<GNEAttributeCarrier*>&
GNEViewNetHelper::ViewObjectsSelector::getAttributeCarriers() const {
    return myViewObjects.attributeCarriers;
}


const std::vector<GNEJunction*>&
GNEViewNetHelper::ViewObjectsSelector::getJunctions() const {
    return myViewObjects.junctions;
}


const std::vector<GNEEdge*>&
GNEViewNetHelper::ViewObjectsSelector::getEdges() const {
    return myViewObjects.edges;
}

const std::vector<GNETAZ*>&
GNEViewNetHelper::ViewObjectsSelector::getTAZs() const {
    return myViewObjects.TAZs;
}

const std::vector<GNEAdditional*>&
GNEViewNetHelper::ViewObjectsSelector::getAdditionals() const {
    return myViewObjects.additionals;
}

const std::vector<GNEDemandElement*>&
GNEViewNetHelper::ViewObjectsSelector::getDemandElements() const {
    return myViewObjects.demandElements;
}


GNEViewNetHelper::ViewObjectsSelector::ViewObjectsContainer::ViewObjectsContainer() {}


void
GNEViewNetHelper::ViewObjectsSelector::ViewObjectsContainer::clearElements() {
    // just clear all containers
    GUIGlObjects.clear();
    attributeCarriers.clear();
    networkElements.clear();
    additionals.clear();
    demandElements.clear();
    junctions.clear();
    edges.clear();
    lanes.clear();
    crossings.clear();
    walkingAreas.clear();
    connections.clear();
    internalLanes.clear();
    TAZs.clear();
    POIs.clear();
    polys.clear();
    genericDatas.clear();
    edgeDatas.clear();
    edgeRelDatas.clear();
    TAZRelDatas.clear();
}


void
GNEViewNetHelper::ViewObjectsSelector::ViewObjectsContainer::filterElements(const std::vector<const GUIGlObject*>& objects) {
    for (const auto& object : objects) {
        // remove from GUIGlObjects
        auto itGlObjects = GUIGlObjects.begin();
        while (itGlObjects != GUIGlObjects.end()) {
            if (*itGlObjects == object) {
                itGlObjects = GUIGlObjects.erase(itGlObjects);
            } else {
                itGlObjects++;
            }
        }
        // remove from attributeCarriers
        auto itACs = attributeCarriers.begin();
        while (itACs != attributeCarriers.end()) {
            if ((*itACs)->getGUIGlObject() == object) {
                itACs = attributeCarriers.erase(itACs);
            } else {
                itACs++;
            }
        }
        // remove from networkElements
        auto itNetworkElements = networkElements.begin();
        while (itNetworkElements != networkElements.end()) {
            if ((*itNetworkElements)->getGUIGlObject() == object) {
                itNetworkElements = networkElements.erase(itNetworkElements);
            } else {
                itNetworkElements++;
            }
        }
        // remove from additionals
        auto itAdditionals = additionals.begin();
        while (itAdditionals != additionals.end()) {
            if ((*itAdditionals)->getGUIGlObject() == object) {
                itAdditionals = additionals.erase(itAdditionals);
            } else {
                itAdditionals++;
            }
        }
        // remove from demandElements
        auto itDemandElements = demandElements.begin();
        while (itDemandElements != demandElements.end()) {
            if ((*itDemandElements)->getGUIGlObject() == object) {
                itDemandElements = demandElements.erase(itDemandElements);
            } else {
                itDemandElements++;
            }
        }
        // remove from genericDatas
        auto itGenericDatas = genericDatas.begin();
        while (itGenericDatas != genericDatas.end()) {
            if ((*itGenericDatas)->getGUIGlObject() == object) {
                itGenericDatas = genericDatas.erase(itGenericDatas);
            } else {
                itGenericDatas++;
            }
        }
        // remove from junctions
        auto itJunctions = junctions.begin();
        while (itJunctions != junctions.end()) {
            if ((*itJunctions)->getGUIGlObject() == object) {
                itJunctions = junctions.erase(itJunctions);
            } else {
                itJunctions++;
            }
        }
        // remove from edges
        auto itEdges = edges.begin();
        while (itEdges != edges.end()) {
            if ((*itEdges)->getGUIGlObject() == object) {
                itEdges = edges.erase(itEdges);
            } else {
                itEdges++;
            }
        }
        // remove from lanes
        auto itLanes = lanes.begin();
        while (itLanes != lanes.end()) {
            if ((*itLanes)->getGUIGlObject() == object) {
                itLanes = lanes.erase(itLanes);
            } else {
                itLanes++;
            }
        }
        // remove from crossings
        auto itCrossings = crossings.begin();
        while (itCrossings != crossings.end()) {
            if ((*itCrossings)->getGUIGlObject() == object) {
                itCrossings = crossings.erase(itCrossings);
            } else {
                itCrossings++;
            }
        }
        // remove from walkingAreas
        auto itWalkingAreas = walkingAreas.begin();
        while (itWalkingAreas != walkingAreas.end()) {
            if ((*itWalkingAreas)->getGUIGlObject() == object) {
                itWalkingAreas = walkingAreas.erase(itWalkingAreas);
            } else {
                itWalkingAreas++;
            }
        }
        // remove from connections
        auto itConnections = connections.begin();
        while (itConnections != connections.end()) {
            if ((*itConnections)->getGUIGlObject() == object) {
                itConnections = connections.erase(itConnections);
            } else {
                itConnections++;
            }
        }
        // remove from internalLanes
        auto itInternalLanes = internalLanes.begin();
        while (itInternalLanes != internalLanes.end()) {
            if ((*itInternalLanes)->getGUIGlObject() == object) {
                itInternalLanes = internalLanes.erase(itInternalLanes);
            } else {
                itInternalLanes++;
            }
        }
        // remove from TAZs
        auto itTAZs = TAZs.begin();
        while (itTAZs != TAZs.end()) {
            if ((*itTAZs)->getGUIGlObject() == object) {
                itTAZs = TAZs.erase(itTAZs);
            } else {
                itTAZs++;
            }
        }
        // remove from POIs
        auto itPOIs = POIs.begin();
        while (itPOIs != POIs.end()) {
            if ((*itPOIs)->getGUIGlObject() == object) {
                itPOIs = POIs.erase(itPOIs);
            } else {
                itPOIs++;
            }
        }
        // remove from polys
        auto itPolys = polys.begin();
        while (itPolys != polys.end()) {
            if ((*itPolys)->getGUIGlObject() == object) {
                itPolys = polys.erase(itPolys);
            } else {
                itPolys++;
            }
        }
        // remove from edgeDatas
        auto itEdgeDatas = edgeDatas.begin();
        while (itEdgeDatas != edgeDatas.end()) {
            if ((*itEdgeDatas)->getGUIGlObject() == object) {
                itEdgeDatas = edgeDatas.erase(itEdgeDatas);
            } else {
                itEdgeDatas++;
            }
        }
        // remove from edgeRelDatas
        auto itEdgeRelDatas = edgeRelDatas.begin();
        while (itEdgeRelDatas != edgeRelDatas.end()) {
            if ((*itEdgeRelDatas)->getGUIGlObject() == object) {
                itEdgeRelDatas = edgeRelDatas.erase(itEdgeRelDatas);
            } else {
                itEdgeRelDatas++;
            }
        }
        // remove from TAZRelDatas
        auto itTAZRelDatas = TAZRelDatas.begin();
        while (itTAZRelDatas != TAZRelDatas.end()) {
            if ((*itTAZRelDatas)->getGUIGlObject() == object) {
                itTAZRelDatas = TAZRelDatas.erase(itTAZRelDatas);
            } else {
                itTAZRelDatas++;
            }
        }
    }
}


void
GNEViewNetHelper::ViewObjectsSelector::updateNetworkElements(ViewObjectsContainer& container, const GUIGlObject* glObject) {
    // get front AC
    const auto frontGLObject = myViewNet->getFrontGLObject();
    // cast specific network element
    switch (glObject->getType()) {
        case GLO_JUNCTION: {
            // get junction
            auto junction = myViewNet->getNet()->getAttributeCarriers()->retrieveJunction(glObject->getMicrosimID());
            // check front element
            if (glObject == frontGLObject) {
                // insert at front
                container.junctions.insert(container.junctions.begin(), junction);
                container.networkElements.insert(container.networkElements.begin(), junction);
                container.attributeCarriers.insert(container.attributeCarriers.begin(), junction);
                container.GUIGlObjects.insert(container.GUIGlObjects.begin(), junction);
            } else {
                // insert at back
                container.junctions.push_back(junction);
                container.networkElements.push_back(junction);
                container.attributeCarriers.push_back(junction);
                container.GUIGlObjects.push_back(junction);
            }
            break;
        }
        case GLO_EDGE: {
            // get edge
            auto edge = myViewNet->getNet()->getAttributeCarriers()->retrieveEdge(glObject->getMicrosimID());
            // check front element
            if (glObject == frontGLObject) {
                // insert at front
                container.edges.insert(container.edges.begin(), edge);
                container.networkElements.insert(container.networkElements.begin(), edge);
                container.attributeCarriers.insert(container.attributeCarriers.begin(), edge);
                container.GUIGlObjects.insert(container.GUIGlObjects.begin(), edge);
            } else {
                // insert at back
                container.edges.push_back(edge);
                container.networkElements.push_back(edge);
                container.attributeCarriers.push_back(edge);
                container.GUIGlObjects.push_back(edge);
            }
            break;
        }
        case GLO_LANE: {
            // get lane
            auto lane = myViewNet->getNet()->getAttributeCarriers()->retrieveLane(glObject);
            // check front element
            if (glObject == frontGLObject) {
                // insert at front
                container.lanes.insert(container.lanes.begin(), lane);
                container.networkElements.insert(container.networkElements.begin(), lane);
                container.attributeCarriers.insert(container.attributeCarriers.begin(), lane);
                container.GUIGlObjects.insert(container.GUIGlObjects.begin(), lane);
            } else {
                // insert at back
                container.lanes.push_back(lane);
                container.networkElements.push_back(lane);
                container.attributeCarriers.push_back(lane);
                container.GUIGlObjects.push_back(lane);
            }
            break;
        }
        case GLO_CROSSING: {
            // get crossing
            auto crossing = myViewNet->getNet()->getAttributeCarriers()->retrieveCrossing(glObject);
            // check front element
            if (glObject == frontGLObject) {
                // insert at front
                container.crossings.insert(container.crossings.begin(), crossing);
                container.networkElements.insert(container.networkElements.begin(), crossing);
                container.attributeCarriers.insert(container.attributeCarriers.begin(), crossing);
                container.GUIGlObjects.insert(container.GUIGlObjects.begin(), crossing);
            } else {
                // insert at back
                container.crossings.push_back(crossing);
                container.networkElements.push_back(crossing);
                container.attributeCarriers.push_back(crossing);
                container.GUIGlObjects.push_back(crossing);
            }
            break;
        }
        case GLO_WALKINGAREA: {
            // get walkingArea
            auto walkingArea = myViewNet->getNet()->getAttributeCarriers()->retrieveWalkingArea(glObject);
            // check front element
            if (glObject == frontGLObject) {
                // insert at front
                container.walkingAreas.insert(container.walkingAreas.begin(), walkingArea);
                container.networkElements.insert(container.networkElements.begin(), walkingArea);
                container.attributeCarriers.insert(container.attributeCarriers.begin(), walkingArea);
                container.GUIGlObjects.insert(container.GUIGlObjects.begin(), walkingArea);
            } else {
                // insert at back
                container.walkingAreas.push_back(walkingArea);
                container.networkElements.push_back(walkingArea);
                container.attributeCarriers.push_back(walkingArea);
                container.GUIGlObjects.push_back(walkingArea);
            }
            break;
        }
        case GLO_CONNECTION: {
            // get connection
            auto connection = myViewNet->getNet()->getAttributeCarriers()->retrieveConnection(glObject);
            // check front element
            if (glObject == frontGLObject) {
                // insert at front
                container.connections.insert(container.connections.begin(), connection);
                container.networkElements.insert(container.networkElements.begin(), connection);
                container.attributeCarriers.insert(container.attributeCarriers.begin(), connection);
                container.GUIGlObjects.insert(container.GUIGlObjects.begin(), connection);
            } else {
                // insert at back
                container.connections.push_back(connection);
                container.networkElements.push_back(connection);
                container.attributeCarriers.push_back(connection);
                container.GUIGlObjects.push_back(connection);
            }
            break;
        }
        case GLO_TLLOGIC: {
            // get internal lane
            auto internalLane = myViewNet->getNet()->getAttributeCarriers()->retrieveInternalLane(glObject);
            // check front element
            if (glObject == frontGLObject) {
                // insert at front
                container.internalLanes.insert(container.internalLanes.begin(), internalLane);
                container.attributeCarriers.insert(container.attributeCarriers.begin(), internalLane);
                container.GUIGlObjects.insert(container.GUIGlObjects.begin(), internalLane);
            } else {
                // insert at back
                container.internalLanes.push_back(internalLane);
                container.attributeCarriers.push_back(internalLane);
                container.GUIGlObjects.push_back(internalLane);
            }
            break;
        }
        default:
            break;
    }
}


void
GNEViewNetHelper::ViewObjectsSelector::updateAdditionalElements(ViewObjectsContainer& container, const GUIGlObject* glObject) {
    // get additional element
    auto additionalElement = myViewNet->getNet()->getAttributeCarriers()->retrieveAdditional(glObject, false);
    if (additionalElement) {
        // insert depending if is the front attribute carrier
        if (additionalElement == myViewNet->getFrontAttributeCarrier()) {
            // insert at front
            container.additionals.insert(container.additionals.begin(), additionalElement);
            container.attributeCarriers.insert(container.attributeCarriers.begin(), additionalElement);
            container.GUIGlObjects.insert(container.GUIGlObjects.begin(), additionalElement);
        } else {
            // insert at back
            container.additionals.push_back(additionalElement);
            container.attributeCarriers.push_back(additionalElement);
            container.GUIGlObjects.push_back(additionalElement);
        }
    }
}


void
GNEViewNetHelper::ViewObjectsSelector::updateShapeElements(ViewObjectsContainer& container, const GUIGlObject* glObject) {
    // cast specific shape
    if (glObject->getType() == GLO_POI) {
        // cast POI
        auto POI = dynamic_cast<GNEPOI*>(myViewNet->getNet()->getAttributeCarriers()->retrieveAdditional(glObject));
        // check front element
        if (glObject == myViewNet->getFrontGLObject()) {
            // insert at front
            container.POIs.insert(container.POIs.begin(), POI);
        } else {
            // insert at back
            container.POIs.push_back(POI);
        }
    } else if ((glObject->getType() == GLO_POLYGON) || (glObject->getType() == GLO_JPS_WALKABLEAREA) || (glObject->getType() == GLO_JPS_OBSTACLE)) {
        // cast poly
        auto poly = dynamic_cast<GNEPoly*>(myViewNet->getNet()->getAttributeCarriers()->retrieveAdditional(glObject));
        // check front element
        if (glObject == myViewNet->getFrontGLObject()) {
            // insert at front
            container.polys.insert(container.polys.begin(), poly);
        } else {
            // insert at back
            container.polys.push_back(poly);
        }
    }
}


void
GNEViewNetHelper::ViewObjectsSelector::updateTAZElements(ViewObjectsContainer& container, const GUIGlObject* glObject) {
    // cast specific TAZ
    if (glObject->getType() == GLO_TAZ) {
        // cast TAZ
        auto TAZ = dynamic_cast<GNETAZ*>(myViewNet->getNet()->getAttributeCarriers()->retrieveAdditional(glObject));
        // check front element
        if (glObject == myViewNet->getFrontGLObject()) {
            // insert at front
            container.TAZs.insert(container.TAZs.begin(), TAZ);
        } else {
            // insert at back
            container.TAZs.push_back(TAZ);
        }
    }
}


void
GNEViewNetHelper::ViewObjectsSelector::updateDemandElements(ViewObjectsContainer& container, const GUIGlObject* glObject) {
    // get demandElement
    GNEDemandElement* demandElement = myViewNet->getNet()->getAttributeCarriers()->retrieveDemandElement(glObject, false);
    if (demandElement) {
        // insert depending if is the front attribute carrier
        if (demandElement == myViewNet->getFrontAttributeCarrier()) {
            // insert at front
            container.demandElements.insert(container.demandElements.begin(), demandElement);
            container.attributeCarriers.insert(container.attributeCarriers.begin(), demandElement);
            container.GUIGlObjects.insert(container.GUIGlObjects.begin(), demandElement);
        } else {
            // insert at back
            container.demandElements.push_back(demandElement);
            container.attributeCarriers.push_back(demandElement);
            container.GUIGlObjects.push_back(demandElement);
        }
    }
}


void
GNEViewNetHelper::ViewObjectsSelector::updateGenericDataElements(ViewObjectsContainer& container, const GUIGlObject* glObject) {
    // cast specific generic data
    switch (glObject->getType()) {
        case GLO_EDGEDATA: {
            // cast EdgeData
            auto edgeData = dynamic_cast<GNEEdgeData*>(myViewNet->getNet()->getAttributeCarriers()->retrieveGenericData(glObject));
            // check front element
            if (glObject == myViewNet->getFrontGLObject()) {
                // insert at front
                container.edgeDatas.insert(container.edgeDatas.begin(), edgeData);
                container.genericDatas.insert(container.genericDatas.begin(), edgeData);
                container.attributeCarriers.insert(container.attributeCarriers.begin(), edgeData);
                container.GUIGlObjects.insert(container.GUIGlObjects.begin(), edgeData);
            } else {
                // insert at back
                container.edgeDatas.push_back(edgeData);
                container.genericDatas.push_back(edgeData);
                container.attributeCarriers.push_back(edgeData);
                container.GUIGlObjects.push_back(edgeData);
            }
            break;
        }
        case GLO_EDGERELDATA: {
            // cast EdgeData
            auto edgeRelData = dynamic_cast<GNEEdgeRelData*>(myViewNet->getNet()->getAttributeCarriers()->retrieveGenericData(glObject));
            // check front element
            if (glObject == myViewNet->getFrontGLObject()) {
                // insert at front
                container.edgeRelDatas.insert(container.edgeRelDatas.begin(), edgeRelData);
                container.genericDatas.insert(container.genericDatas.begin(), edgeRelData);
                container.attributeCarriers.insert(container.attributeCarriers.begin(), edgeRelData);
                container.GUIGlObjects.insert(container.GUIGlObjects.begin(), edgeRelData);
            } else {
                // insert at back
                container.edgeRelDatas.push_back(edgeRelData);
                container.genericDatas.push_back(edgeRelData);
                container.attributeCarriers.push_back(edgeRelData);
                container.GUIGlObjects.push_back(edgeRelData);
            }
            break;
        }
        case GLO_TAZRELDATA: {
            // cast TAZRelData
            auto TAZRelData = dynamic_cast<GNETAZRelData*>(myViewNet->getNet()->getAttributeCarriers()->retrieveGenericData(glObject));
            // check front element
            if (glObject == myViewNet->getFrontGLObject()) {
                // insert at front
                container.TAZRelDatas.insert(container.TAZRelDatas.begin(), TAZRelData);
                container.genericDatas.insert(container.genericDatas.begin(), TAZRelData);
                container.attributeCarriers.insert(container.attributeCarriers.begin(), TAZRelData);
                container.GUIGlObjects.insert(container.GUIGlObjects.begin(), TAZRelData);
            } else {
                // insert at back
                container.TAZRelDatas.push_back(TAZRelData);
                container.genericDatas.push_back(TAZRelData);
                container.attributeCarriers.push_back(TAZRelData);
                container.GUIGlObjects.push_back(TAZRelData);
            }
            break;
        }
        default:
            break;
    }
}


void
GNEViewNetHelper::ViewObjectsSelector::processGUIGlObjects(const GUIViewObjectsHandler::GLObjectsSortedContainer& objectsContainer) {
    // iterate over filtered edge objects
    for (const auto& glObjectLayer : objectsContainer) {
        for (const auto& glObject : glObjectLayer.second) {
            // update all elements by categories
            updateNetworkElements(myViewObjects, glObject.object);
            updateAdditionalElements(myViewObjects, glObject.object);
            updateShapeElements(myViewObjects, glObject.object);
            updateTAZElements(myViewObjects, glObject.object);
            updateDemandElements(myViewObjects, glObject.object);
            updateGenericDataElements(myViewObjects, glObject.object);
        }
    }
}


GNEViewNetHelper::ViewObjectsSelector::ViewObjectsSelector() :
    myViewNet(nullptr) {
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::MouseButtonKeyPressed - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::MouseButtonKeyPressed::MouseButtonKeyPressed() :
    myEventInfo(nullptr) {
}


void
GNEViewNetHelper::MouseButtonKeyPressed::update(void* eventData) {
    myEventInfo = (FXEvent*) eventData;
}


bool
GNEViewNetHelper::MouseButtonKeyPressed::shiftKeyPressed() const {
    if (myEventInfo) {
        return (myEventInfo->state & SHIFTMASK) != 0;
    } else {
        return false;
    }
}


bool
GNEViewNetHelper::MouseButtonKeyPressed::controlKeyPressed() const {
    if (myEventInfo) {
        return (myEventInfo->state & CONTROLMASK) != 0;
    } else {
        return false;
    }
}


bool
GNEViewNetHelper::MouseButtonKeyPressed::altKeyPressed() const {
    if (myEventInfo) {
        return (myEventInfo->state & ALTMASK) != 0;
    } else {
        return false;
    }
}


bool
GNEViewNetHelper::MouseButtonKeyPressed::mouseLeftButtonPressed() const {
    if (myEventInfo) {
        return (myEventInfo->state & LEFTBUTTONMASK) != 0;
    } else {
        return false;
    }
}


bool
GNEViewNetHelper::MouseButtonKeyPressed::mouseRightButtonPressed() const {
    if (myEventInfo) {
        return (myEventInfo->state & RIGHTBUTTONMASK) != 0;
    } else {
        return false;
    }
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::MoveSingleElementValues - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::MoveSingleElementModul::MoveSingleElementModul(GNEViewNet* viewNet) :
    myViewNet(viewNet) {
}


bool
GNEViewNetHelper::MoveSingleElementModul::beginMoveNetworkElementShape() {
    // first obtain moving reference (common for all)
    myRelativeClickedPosition = myViewNet->getPositionInformation();
    // get edited element
    const GNENetworkElement* editedElement = myViewNet->myEditNetworkElementShapes.getEditedNetworkElement();
    // check what type of AC will be moved
    if (myViewNet->myViewObjectsSelector.getJunctionFront() && (myViewNet->myViewObjectsSelector.getJunctionFront() == editedElement)) {
        // get move operation
        GNEMoveOperation* moveOperation = myViewNet->myViewObjectsSelector.getJunctionFront()->getMoveOperation();
        // continue if move operation is valid
        if (moveOperation) {
            myMoveOperation = moveOperation;
            return true;
        } else {
            return false;
        }
    } else if (myViewNet->myViewObjectsSelector.getLaneFront() && (myViewNet->myViewObjectsSelector.getLaneFront() == editedElement)) {
        // get move operation
        GNEMoveOperation* moveOperation = myViewNet->myViewObjectsSelector.getLaneFront()->getMoveOperation();
        // continue if move operation is valid
        if (moveOperation) {
            myMoveOperation = moveOperation;
            return true;
        } else {
            return false;
        }
    } else if (myViewNet->myViewObjectsSelector.getCrossingFront() && (myViewNet->myViewObjectsSelector.getCrossingFront() == editedElement)) {
        // get move operation
        GNEMoveOperation* moveOperation = myViewNet->myViewObjectsSelector.getCrossingFront()->getMoveOperation();
        // continue if move operation is valid
        if (moveOperation) {
            myMoveOperation = moveOperation;
            return true;
        } else {
            return false;
        }
    } else if (myViewNet->myViewObjectsSelector.getConnectionFront() && (myViewNet->myViewObjectsSelector.getConnectionFront() == editedElement)) {
        // get move operation
        GNEMoveOperation* moveOperation = myViewNet->myViewObjectsSelector.getConnectionFront()->getMoveOperation();
        // continue if move operation is valid
        if (moveOperation) {
            myMoveOperation = moveOperation;
            return true;
        } else {
            return false;
        }
    } else if (myViewNet->myViewObjectsSelector.getWalkingAreaFront() && (myViewNet->myViewObjectsSelector.getWalkingAreaFront() == editedElement)) {
        // get move operation
        GNEMoveOperation* moveOperation = myViewNet->myViewObjectsSelector.getWalkingAreaFront()->getMoveOperation();
        // continue if move operation is valid
        if (moveOperation) {
            myMoveOperation = moveOperation;
            return true;
        } else {
            return false;
        }
    } else {
        // there isn't moved items, then return false
        return false;
    }
}


bool
GNEViewNetHelper::MoveSingleElementModul::beginMoveSingleElementNetworkMode() {
    // first obtain moving reference (common for all)
    myRelativeClickedPosition = myViewNet->getPositionInformation();
    // get front AC
    const GNEAttributeCarrier* frontAC = myViewNet->myViewObjectsSelector.getAttributeCarrierFront();
    // check what type of AC will be moved
    if (myViewNet->myViewObjectsSelector.getPolyFront() && (frontAC == myViewNet->myViewObjectsSelector.getPolyFront())) {
        // get move operation
        GNEMoveOperation* moveOperation = myViewNet->myViewObjectsSelector.getPolyFront()->getMoveOperation();
        // continue if move operation is valid
        if (moveOperation) {
            myMoveOperation = moveOperation;
            return true;
        } else {
            return false;
        }
    } else if (myViewNet->myViewObjectsSelector.getPOIFront() && (frontAC == myViewNet->myViewObjectsSelector.getPOIFront())) {
        // get move operation
        GNEMoveOperation* moveOperation = myViewNet->myViewObjectsSelector.getPOIFront()->getMoveOperation();
        // continue if move operation is valid
        if (moveOperation) {
            myMoveOperation = moveOperation;
            return true;
        } else {
            return false;
        }
    } else if (myViewNet->myViewObjectsSelector.getAdditionalFront() && (frontAC == myViewNet->myViewObjectsSelector.getAdditionalFront())) {
        // get move operation
        GNEMoveOperation* moveOperation = myViewNet->myViewObjectsSelector.getAdditionalFront()->getMoveOperation();
        // continue if move operation is valid
        if (moveOperation) {
            myMoveOperation = moveOperation;
            return true;
        } else {
            return false;
        }
    } else if (myViewNet->myViewObjectsSelector.getJunctionFront() && (frontAC == myViewNet->myViewObjectsSelector.getJunctionFront())) {
        // check if over junction there is a geometry point
        if (myViewNet->myViewObjectsSelector.getEdgeFront() && (myViewNet->myViewObjectsSelector.getEdgeFront()->clickedOverGeometryPoint(myRelativeClickedPosition))) {
            // get move operation
            GNEMoveOperation* moveOperation = myViewNet->myViewObjectsSelector.getEdgeFront()->getMoveOperation();
            // continue if move operation is valid
            if (moveOperation) {
                myMoveOperation = moveOperation;
                return true;
            } else {
                return false;
            }
        } else {
            // get move operation
            GNEMoveOperation* moveOperation = myViewNet->myViewObjectsSelector.getJunctionFront()->getMoveOperation();
            // continue if move operation is valid
            if (moveOperation) {
                myMoveOperation = moveOperation;
                return true;
            } else {
                return false;
            }
        }
    } else if (myViewNet->myViewObjectsSelector.getEdgeFront() && (frontAC == myViewNet->myViewObjectsSelector.getEdgeFront())) {
        // calculate Edge movement values (can be entire shape, single geometry points, altitude, etc.)
        if (myViewNet->myMouseButtonKeyPressed.shiftKeyPressed()) {
            // edit end point
            myViewNet->myViewObjectsSelector.getEdgeFront()->editEndpoint(myViewNet->getPositionInformation(), myViewNet->myUndoList);
            // edge values wasn't calculated, then return false
            return false;
        } else {
            // get move operation
            GNEMoveOperation* moveOperation = myViewNet->myViewObjectsSelector.getEdgeFront()->getMoveOperation();
            // continue if move operation is valid
            if (moveOperation) {
                myMoveOperation = moveOperation;
                return true;
            } else {
                return false;
            }
        }
    } else if (myViewNet->myViewObjectsSelector.getLaneFront() && (frontAC == myViewNet->myViewObjectsSelector.getLaneFront())) {
        // get move operation
        GNEMoveOperation* moveOperation = myViewNet->myViewObjectsSelector.getLaneFront()->getMoveOperation();
        // continue if move operation is valid
        if (moveOperation) {
            myMoveOperation = moveOperation;
            return true;
        } else {
            return false;
        }
    } else {
        // there isn't moved items, then return false
        return false;
    }
}


bool
GNEViewNetHelper::MoveSingleElementModul::beginMoveSingleElementDemandMode() {
    // first obtain moving reference (common for all)
    myRelativeClickedPosition = myViewNet->getPositionInformation();
    // get front AC
    const GNEAttributeCarrier* frontAC = myViewNet->myViewObjectsSelector.getAttributeCarrierFront();
    // check demand element
    if (myViewNet->myViewObjectsSelector.getDemandElementFront() && (frontAC == myViewNet->myViewObjectsSelector.getDemandElementFront())) {
        // get move operation
        GNEMoveOperation* moveOperation = myViewNet->myViewObjectsSelector.getDemandElementFront()->getMoveOperation();
        // continue if move operation is valid
        if (moveOperation) {
            myMoveOperation = moveOperation;
            return true;
        } else {
            return false;
        }

    } else {
        // there isn't moved items, then return false
        return false;
    }
}


void
GNEViewNetHelper::MoveSingleElementModul::moveSingleElement(const bool mouseLeftButtonPressed) {
    if (myMoveOperation) {
        // calculate moveOffset
        const GNEMoveOffset moveOffset = calculateMoveOffset();
        // check if mouse button is pressed
        if (mouseLeftButtonPressed) {
            // move elements
            GNEMoveElement::moveElement(myViewNet, myMoveOperation, moveOffset);
        } else {
            // commit move
            GNEMoveElement::commitMove(myViewNet, myMoveOperation, moveOffset, myViewNet->getUndoList());
            // don't forget delete move operation
            delete myMoveOperation;
            myMoveOperation = nullptr;
        }
    }
}


void
GNEViewNetHelper::MoveSingleElementModul::finishMoveSingleElement() {
    if (myMoveOperation) {
        // calculate moveOffset
        const GNEMoveOffset moveOffset = calculateMoveOffset();
        GNEMoveElement::commitMove(myViewNet, myMoveOperation, moveOffset, myViewNet->getUndoList());
        // don't forget delete move operation
        delete myMoveOperation;
        myMoveOperation = nullptr;
    }
}


bool
GNEViewNetHelper::MoveSingleElementModul::isCurrentlyMovingSingleElement() const {
    return myMoveOperation != nullptr;
}


GNEMoveElement*
GNEViewNetHelper::MoveSingleElementModul::getMovedElement() const {
    if (myMoveOperation) {
        return myMoveOperation->moveElement;
    } else {
        return nullptr;
    }
}


const GNEMoveOffset
GNEViewNetHelper::MoveSingleElementModul::calculateMoveOffset() const {
    // calculate moveOffset depending of current mouse position and relative clicked position
    // @note  #3521: Add checkBox to allow moving elements... has to be implemented and used here
    Position moveOffset = (myViewNet->getPositionInformation() - myViewNet->myMoveSingleElement.myRelativeClickedPosition);
    // calculate Z depending of moveElevation
    if (myViewNet->myNetworkViewOptions.menuCheckMoveElevation->shown() && myViewNet->myNetworkViewOptions.menuCheckMoveElevation->amChecked() == TRUE) {
        // use Y as Z value and return Z move offset
        return GNEMoveOffset(moveOffset.y());
    } else {
        // return X-Y move offset
        return GNEMoveOffset(moveOffset.x(), moveOffset.y());
    }
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::MoveMultipleElementValues - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::MoveMultipleElementModul::MoveMultipleElementModul(GNEViewNet* viewNet) :
    myViewNet(viewNet),
    myMovingSelectedEdge(false),
    myEdgeOffset(0) {
}


void
GNEViewNetHelper::MoveMultipleElementModul::beginMoveSelection() {
    // save clicked position (to calculate offset)
    myClickedPosition = myViewNet->getPositionInformation();
    // continue depending of clicked element
    if (myViewNet->myViewObjectsSelector.getJunctionFront()) {
        calculateJunctionSelection();
    } else if (myViewNet->myViewObjectsSelector.getEdgeFront()) {
        calculateEdgeSelection(myViewNet->myViewObjectsSelector.getEdgeFront());
    }
    myViewNet->updateViewNet();
}


void
GNEViewNetHelper::MoveMultipleElementModul::moveSelection(const bool mouseLeftButtonPressed) {
    // calculate moveOffset
    const GNEMoveOffset moveOffset = calculateMoveOffset();
    // check if mouse button is pressed
    if (mouseLeftButtonPressed) {
        // iterate over all operations
        for (const auto& moveOperation : myMoveOperations) {
            // move elements
            GNEMoveElement::moveElement(myViewNet, moveOperation, moveOffset);
        }
    } else if (myMoveOperations.size() > 0) {
        // begin undo list
        myViewNet->getUndoList()->begin(GUIIcon::MODEMOVE, TL("moving selection"));
        // iterate over all operations
        for (const auto& moveOperation : myMoveOperations) {
            // commit move
            GNEMoveElement::commitMove(myViewNet, moveOperation, moveOffset, myViewNet->getUndoList());
            // don't forget delete move operation
            delete moveOperation;
        }
        // end undo list
        myViewNet->getUndoList()->end();
        // clear move operations
        myMoveOperations.clear();
    }
}


void
GNEViewNetHelper::MoveMultipleElementModul::finishMoveSelection() {
    // calculate moveOffset
    const GNEMoveOffset moveOffset = calculateMoveOffset();
    // begin undo list
    myViewNet->getUndoList()->begin(GUIIcon::MODEMOVE, TL("moving selection"));
    // finish all move operations
    for (const auto& moveOperation : myMoveOperations) {
        GNEMoveElement::commitMove(myViewNet, moveOperation, moveOffset, myViewNet->getUndoList());
        // don't forget delete move operation
        delete moveOperation;
    }
    // end undo list
    myViewNet->getUndoList()->end();
    // clear move operations
    myMoveOperations.clear();
}


bool
GNEViewNetHelper::MoveMultipleElementModul::isMovingSelection() const {
    return (myMoveOperations.size() > 0);
}


bool
GNEViewNetHelper::MoveMultipleElementModul::isMovingSelectedEdge() const {
    return myMovingSelectedEdge;
}


void
GNEViewNetHelper::MoveMultipleElementModul::resetMovingSelectedEdge() {
    myMovingSelectedEdge = false;
}


double
GNEViewNetHelper::MoveMultipleElementModul::getEdgeOffset() const {
    return myEdgeOffset;
}


bool
GNEViewNetHelper::MoveMultipleElementModul::isCurrentlyMovingMultipleElements() const {
    return myMoveOperations.size() > 0;
}


const GNEMoveOffset
GNEViewNetHelper::MoveMultipleElementModul::calculateMoveOffset() const {
    // calculate moveOffset depending of current mouse position and relative clicked position
    // @note  #3521: Add checkBox to allow moving elements... has to be implemented and used here
    Position moveOffset = (myViewNet->getPositionInformation() - myClickedPosition);
    // calculate Z depending of moveElevation
    if (myViewNet->myNetworkViewOptions.menuCheckMoveElevation->shown() && myViewNet->myNetworkViewOptions.menuCheckMoveElevation->amChecked() == TRUE) {
        // use Y for Z and return X move offset
        return GNEMoveOffset(moveOffset.y());
    } else {
        // return X-Y move offset
        return GNEMoveOffset(moveOffset.x(), moveOffset.y());
    }
}


void
GNEViewNetHelper::MoveMultipleElementModul::calculateJunctionSelection() {
    // declare move operation
    GNEMoveOperation* moveOperation = nullptr;
    // first move all selected junctions
    const auto selectedJunctions = myViewNet->getNet()->getAttributeCarriers()->getSelectedJunctions();
    // iterate over selected junctions
    for (const auto& junction : selectedJunctions) {
        moveOperation = junction->getMoveOperation();
        if (moveOperation) {
            myMoveOperations.push_back(moveOperation);
        }
    }
    // now move all selected edges
    const auto selectedEdges = myViewNet->getNet()->getAttributeCarriers()->getSelectedEdges();
    // iterate over selected edges
    for (const auto& edge : selectedEdges) {
        moveOperation = edge->getMoveOperation();
        if (moveOperation) {
            myMoveOperations.push_back(moveOperation);
        }
    }
}


void
GNEViewNetHelper::MoveMultipleElementModul::calculateEdgeSelection(const GNEEdge* clickedEdge) {
    // first move all selected junctions
    const auto selectedJunctions = myViewNet->getNet()->getAttributeCarriers()->getSelectedJunctions();
    // iterate over selected junctions
    for (const auto& junction : selectedJunctions) {
        GNEMoveOperation* moveOperation = junction->getMoveOperation();
        if (moveOperation) {
            myMoveOperations.push_back(moveOperation);
        }
    }
    // enable moving selected edge flag
    myMovingSelectedEdge = true;
    // get edge shape
    const auto& shape = clickedEdge->getNBEdge()->getGeometry();
    // calculate offset based on the clicked edge shape and convex angle
    if (clickedEdge->isConvexAngle()) {
        myEdgeOffset = shape.nearest_offset_to_point2D(myViewNet->getPositionInformation());
    } else {
        myEdgeOffset = shape.length2D() - shape.nearest_offset_to_point2D(myViewNet->getPositionInformation());
    }
    // now move all selected edges
    const auto selectedEdges = myViewNet->getNet()->getAttributeCarriers()->getSelectedEdges();
    // iterate over edges between 0 and 180 degrees
    for (const auto& edge : selectedEdges) {
        GNEMoveOperation* moveOperation = edge->getMoveOperation();
        // continue if move operation is valid
        if (moveOperation) {
            myMoveOperations.push_back(moveOperation);
        }
    }
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::VehicleOptions - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::VehicleOptions::VehicleOptions(GNEViewNet* viewNet) :
    myViewNet(viewNet) {
}


void
GNEViewNetHelper::VehicleOptions::buildVehicleOptionsMenuChecks() {
    UNUSED_PARAMETER(myViewNet);
    // currently unused
}


void
GNEViewNetHelper::VehicleOptions::hideVehicleOptionsMenuChecks() {
    // currently unused
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::VehicleTypeOptions - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::VehicleTypeOptions::VehicleTypeOptions(GNEViewNet* viewNet) :
    myViewNet(viewNet) {
}


void
GNEViewNetHelper::VehicleTypeOptions::buildVehicleTypeOptionsMenuChecks() {
    UNUSED_PARAMETER(myViewNet);
    // currently unused
}


void
GNEViewNetHelper::VehicleTypeOptions::hideVehicleTypeOptionsMenuChecks() {
    // currently unused
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::SelectingArea - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::SelectingArea::SelectingArea(GNEViewNet* viewNet) :
    selectingUsingRectangle(false),
    startDrawing(false),
    myViewNet(viewNet) {
}


void
GNEViewNetHelper::SelectingArea::beginRectangleSelection() {
    selectingUsingRectangle = true;
    selectionCorner1 = myViewNet->getPositionInformation();
    selectionCorner2 = selectionCorner1;
}


void
GNEViewNetHelper::SelectingArea::moveRectangleSelection() {
    // start drawing
    startDrawing = true;
    // only update selection corner 2
    selectionCorner2 = myViewNet->getPositionInformation();
    // update status bar
    myViewNet->setStatusBarText(TL("Selection width:") + toString(fabs(selectionCorner1.x() - selectionCorner2.x()))
                                + TL(" height:") + toString(fabs(selectionCorner1.y() - selectionCorner2.y()))
                                + TL(" diagonal:") + toString(selectionCorner1.distanceTo2D(selectionCorner2)));
}


void
GNEViewNetHelper::SelectingArea::finishRectangleSelection() {
    // finish rectangle selection
    selectingUsingRectangle = false;
    startDrawing = false;
    // clear status bar
    myViewNet->setStatusBarText("");
}


void
GNEViewNetHelper::SelectingArea::processRectangleSelection() {
    // shift held down on mouse-down and mouse-up and check that rectangle exist
    if ((abs(selectionCorner1.x() - selectionCorner2.x()) > 0.01) &&
            (abs(selectionCorner1.y() - selectionCorner2.y()) > 0.01) &&
            myViewNet->myMouseButtonKeyPressed.shiftKeyPressed()) {
        // create boundary between two corners
        Boundary rectangleBoundary;
        rectangleBoundary.add(selectionCorner1);
        rectangleBoundary.add(selectionCorner2);
        // process selection within boundary
        processBoundarySelection(rectangleBoundary);
    }
}


std::vector<GNEEdge*>
GNEViewNetHelper::SelectingArea::processEdgeRectangleSelection() {
    // shift held down on mouse-down and mouse-up and check that rectangle exist
    if ((abs(selectionCorner1.x() - selectionCorner2.x()) > 0.01) &&
            (abs(selectionCorner1.y() - selectionCorner2.y()) > 0.01) &&
            myViewNet->myMouseButtonKeyPressed.shiftKeyPressed()) {
        // create boundary between two corners
        Boundary rectangleBoundary;
        rectangleBoundary.add(selectionCorner1);
        rectangleBoundary.add(selectionCorner2);
        // obtain all elements in boundary
        myViewNet->updateObjectsInBoundary(rectangleBoundary);
        // return all edges
        return myViewNet->getViewObjectsSelector().getEdges();
    } else {
        return {};
    }
}


void
GNEViewNetHelper::SelectingArea::drawRectangleSelection(const RGBColor& color) const {
    if (selectingUsingRectangle) {
        GLHelper::pushMatrix();
        glTranslated(0, 0, GLO_RECTANGLESELECTION);
        GLHelper::setColor(color);
        glLineWidth(2);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glBegin(GL_QUADS);
        glVertex2d(selectionCorner1.x(), selectionCorner1.y());
        glVertex2d(selectionCorner1.x(), selectionCorner2.y());
        glVertex2d(selectionCorner2.x(), selectionCorner2.y());
        glVertex2d(selectionCorner2.x(), selectionCorner1.y());
        glEnd();
        GLHelper::popMatrix();
    }
}


void
GNEViewNetHelper::SelectingArea::processBoundarySelection(const Boundary& boundary) {
    const bool selEdges = myViewNet->myNetworkViewOptions.selectEdges();
    // obtain all elements in boundary
    myViewNet->updateObjectsInBoundary(boundary);
    // filter ACsInBoundary depending of current supermode
    std::set<GNEAttributeCarrier*> ACsFiltered;
    for (const auto& AC : myViewNet->getViewObjectsSelector().getAttributeCarriers()) {
        if (!AC->getGUIGlObject()->isGLObjectLocked()) {
            if (myViewNet->myEditModes.isCurrentSupermodeNetwork()) {
                if (AC->getTagProperty().isNetworkElement() || AC->getTagProperty().isAdditionalElement()) {
                    if ((AC->getTagProperty().getTag() == SUMO_TAG_EDGE && !selEdges)
                            || (AC->getTagProperty().getTag() == SUMO_TAG_LANE && selEdges)) {
                        continue;
                    }
                    ACsFiltered.insert(AC);
                }
            } else if (myViewNet->myEditModes.isCurrentSupermodeDemand() && AC->getTagProperty().isDemandElement()) {
                ACsFiltered.insert(AC);
            } else if (myViewNet->myEditModes.isCurrentSupermodeData() && AC->getTagProperty().isGenericData()) {
                ACsFiltered.insert(AC);
            }
        }
    }
    // declare two sets of attribute carriers, one for select and another for unselect
    std::vector<GNEAttributeCarrier*> ACToSelect;
    std::vector<GNEAttributeCarrier*> ACToUnselect;
    // reserve memory (we assume that in the worst case we're going to insert all elements of ACsInBoundaryFiltered
    ACToSelect.reserve(ACsFiltered.size());
    ACToUnselect.reserve(ACsFiltered.size());
    // in restrict AND replace mode all current selected attribute carriers will be unselected
    if ((myViewNet->myViewParent->getSelectorFrame()->getModificationModeModul()->getModificationMode() == GNESelectorFrame::ModificationMode::Operation::RESTRICT) ||
            (myViewNet->myViewParent->getSelectorFrame()->getModificationModeModul()->getModificationMode() == GNESelectorFrame::ModificationMode::Operation::REPLACE)) {
        // obtain selected ACs depending of current supermode
        const auto selectedAC = myViewNet->getNet()->getAttributeCarriers()->getSelectedAttributeCarriers(false);
        // add id into ACs to unselect
        for (const auto& AC : selectedAC) {
            ACToUnselect.push_back(AC);
        }
    }
    // iterate over AttributeCarriers obtained of boundary an place it in ACToSelect or ACToUnselect
    for (const auto& AC : ACsFiltered) {
        switch (myViewNet->myViewParent->getSelectorFrame()->getModificationModeModul()->getModificationMode()) {
            case GNESelectorFrame::ModificationMode::Operation::SUB:
                ACToUnselect.push_back(AC);
                break;
            case GNESelectorFrame::ModificationMode::Operation::RESTRICT:
                if (std::find(ACToUnselect.begin(), ACToUnselect.end(), AC) != ACToUnselect.end()) {
                    ACToSelect.push_back(AC);
                }
                break;
            default:
                ACToSelect.push_back(AC);
                break;
        }
    }
    // select junctions and their connections and crossings if Auto select junctions is enabled (note: only for "add mode")
    if (myViewNet->autoSelectNodes() && (myViewNet->myViewParent->getSelectorFrame()->getModificationModeModul()->getModificationMode() == GNESelectorFrame::ModificationMode::Operation::ADD)) {
        std::vector<GNEEdge*> edgesToSelect;
        // iterate over ACToSelect and extract edges
        for (const auto& AC : ACToSelect) {
            if (AC->getTagProperty().getTag() == SUMO_TAG_EDGE) {
                edgesToSelect.push_back(dynamic_cast<GNEEdge*>(AC));
            }
        }
        // iterate over extracted edges
        for (const auto& edge : edgesToSelect) {
            // select junction source and all their connections and crossings
            ACToSelect.push_back(edge->getFromJunction());
            for (const auto& connection : edge->getFromJunction()->getGNEConnections()) {
                ACToSelect.push_back(connection);
            }
            for (const auto& crossing : edge->getFromJunction()->getGNECrossings()) {
                ACToSelect.push_back(crossing);
            }
            // select junction destination and all their connections crossings
            ACToSelect.push_back(edge->getToJunction());
            for (const auto& connection : edge->getToJunction()->getGNEConnections()) {
                ACToSelect.push_back(connection);
            }
            for (const auto& crossing : edge->getToJunction()->getGNECrossings()) {
                ACToSelect.push_back(crossing);
            }
        }
    }
    // only continue if there is ACs to select or unselect
    if ((ACToSelect.size() + ACToUnselect.size()) > 0) {
        // first unselect AC of ACToUnselect and then selects AC of ACToSelect
        myViewNet->myUndoList->begin(GUIIcon::MODESELECT, TL("selection using rectangle"));
        for (const auto& AC : ACToUnselect) {
            AC->setAttribute(GNE_ATTR_SELECTED, "0", myViewNet->myUndoList);
        }
        for (const auto& AC : ACToSelect) {
            if (AC->getTagProperty().isSelectable()) {
                AC->setAttribute(GNE_ATTR_SELECTED, "1", myViewNet->myUndoList);
            }
        }
        myViewNet->myUndoList->end();
    }
    myViewNet->makeNonCurrent();
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::TestingMode - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::TestingMode::TestingMode(GNEViewNet* viewNet) :
    myViewNet(viewNet) {
}


void
GNEViewNetHelper::TestingMode::initTestingMode() {
    // first check if testing mode is enabled and window size is correct
    if (OptionsCont::getOptions().getBool("gui-testing") && OptionsCont::getOptions().isSet("window-size")) {
        std::vector<std::string> windowSize = OptionsCont::getOptions().getStringVector("window-size");
        // make sure that given windows size has exactly two valid int values
        if ((windowSize.size() == 2) && GNEAttributeCarrier::canParse<int>(windowSize[0]) && GNEAttributeCarrier::canParse<int>(windowSize[1])) {
            myTestingWidth = GNEAttributeCarrier::parse<int>(windowSize[0]);
            myTestingHeight = GNEAttributeCarrier::parse<int>(windowSize[1]);
        } else {
            WRITE_ERRORF(TL("Invalid windows size-format: %for option 'window-size'"), toString(windowSize));
        }
    }
}


void
GNEViewNetHelper::TestingMode::drawTestingElements(GUIMainWindow* mainWindow) {
    // first check if testing mode is enabled
    if (OptionsCont::getOptions().getBool("gui-testing")) {
        // check if main windows has to be resized
        if (myTestingWidth > 0) {
            mainWindow->resize(myTestingWidth, myTestingHeight);
        }
        //std::cout << " fixed: view=" << getWidth() << ", " << getHeight() << " app=" << mainWindow->getWidth() << ", " << mainWindow->getHeight() << "\n";
        // draw pink square in the upper left corner on top of everything
        const double size = myViewNet->p2m(32);
        Position center = myViewNet->screenPos2NetPos(8, 8);
        // magenta
        GLHelper::pushMatrix();
        GLHelper::setColor(RGBColor::MAGENTA);
        glTranslated(center.x(), center.y(), GLO_TESTELEMENT);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glBegin(GL_QUADS);
        glVertex2d(0, 0);
        glVertex2d(0, -size);
        glVertex2d(size, -size);
        glVertex2d(size, 0);
        glEnd();
        GLHelper::popMatrix();
        // blue
        GLHelper::pushMatrix();
        GLHelper::setColor(RGBColor::BLUE);
        glTranslated(center.x(), center.y(), GLO_TESTELEMENT + 1);
        glScaled(0.7, 0.7, 0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glBegin(GL_QUADS);
        glVertex2d(0, 0);
        glVertex2d(0, -size);
        glVertex2d(size, -size);
        glVertex2d(size, 0);
        glEnd();
        GLHelper::popMatrix();
        // yellow
        GLHelper::pushMatrix();
        GLHelper::setColor(RGBColor::YELLOW);
        glTranslated(center.x(), center.y(), GLO_TESTELEMENT + 2);
        glScaled(0.4, 0.4, 0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glBegin(GL_QUADS);
        glVertex2d(0, 0);
        glVertex2d(0, -size);
        glVertex2d(size, -size);
        glVertex2d(size, 0);
        glEnd();
        GLHelper::popMatrix();
    }
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::SaveElements - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::SaveElements::SaveElements(GNEViewNet* viewNet) :
    myViewNet(viewNet) {
}


GNEViewNetHelper::SaveElements::~SaveElements() {
    delete mySaveIndividualFilesPopup;
}


void
GNEViewNetHelper::SaveElements::buildSaveElementsButtons() {
    // get save element grip
    auto gripSaveElements = myViewNet->getViewParent()->getGNEAppWindows()->getToolbarsGrip().saveElements;
    // get tooltip menu
    auto tooltipMenu = myViewNet->myViewParent->getGNEAppWindows()->getStaticTooltipMenu();
    // create save sumo config button
    mySaveNeteditConfig = new MFXButtonTooltip(gripSaveElements, tooltipMenu,
            std::string("\t") + TL("Save Netedit Config") + std::string("\t") + TL("Save Netedit Config. (Ctrl+Shift+E)"), GUIIconSubSys::getIcon(GUIIcon::SAVE_NETEDITCONFIG),
            myViewNet->getViewParent()->getGNEAppWindows(), MID_HOTKEY_CTRL_SHIFT_E_SAVENETEDITCONFIG, GUIDesignButtonToolbar);
    mySaveNeteditConfig->create();
    // create save sumo config button
    mySaveSumoConfig = new MFXButtonTooltip(gripSaveElements, tooltipMenu,
                                            std::string("\t") + TL("Save Sumo Config") + std::string("\t") + TL("Save Sumo Config. (Ctrl+Shift+S)"), GUIIconSubSys::getIcon(GUIIcon::SAVE_SUMOCONFIG),
                                            myViewNet->getViewParent()->getGNEAppWindows(), MID_HOTKEY_CTRL_SHIFT_S_SAVESUMOCONFIG, GUIDesignButtonToolbar);
    mySaveSumoConfig->create();
    // create save network button
    mySaveNetwork = new MFXButtonTooltip(gripSaveElements, tooltipMenu,
                                         std::string("\t") + TL("Save network") + std::string("\t") + TL("Save network. (Ctrl+S)"), GUIIconSubSys::getIcon(GUIIcon::SAVE_NETWORKELEMENTS),
                                         myViewNet->getViewParent()->getGNEAppWindows(), MID_HOTKEY_CTRL_S_STOPSIMULATION_SAVENETWORK, GUIDesignButtonToolbar);
    mySaveNetwork->create();
    // create popup for save individual files
    mySaveIndividualFilesPopup = new FXPopup(gripSaveElements, POPUP_VERTICAL);
    mySaveIndividualFilesPopup->create();
    // create save individual files button
    mySaveIndividualFiles = new MFXMenuButtonTooltip(gripSaveElements, tooltipMenu,
            std::string("\t") + TL("Save individual files") + std::string("\t") + TL("Save individual files."),
            GUIIconSubSys::getIcon(GUIIcon::SAVE_MULTIPLE), mySaveIndividualFilesPopup, nullptr, GUIDesignButtonToolbarLocator);
    mySaveIndividualFiles->create();
    // create save additional elements button
    mySaveAdditionalElements = new MFXButtonTooltip(mySaveIndividualFilesPopup, tooltipMenu,
            std::string("\t") + TL("Save additional elements") + std::string("\t") + TL("Save additional elements. (Ctrl+Shift+A)"), GUIIconSubSys::getIcon(GUIIcon::SAVE_ADDITIONALELEMENTS),
            myViewNet->getViewParent()->getGNEAppWindows(), MID_HOTKEY_CTRL_SHIFT_A_SAVEADDITIONALS, GUIDesignButtonPopup);
    mySaveAdditionalElements->create();
    // create save demand elements button
    mySaveDemandElements = new MFXButtonTooltip(mySaveIndividualFilesPopup, tooltipMenu,
            std::string("\t") + TL("Save demand elements") + std::string("\t") + TL("Save demand elements. (Ctrl+Shift+D)"), GUIIconSubSys::getIcon(GUIIcon::SAVE_DEMANDELEMENTS),
            myViewNet->getViewParent()->getGNEAppWindows(), MID_HOTKEY_CTRL_SHIFT_D_SAVEDEMANDELEMENTS, GUIDesignButtonPopup);
    mySaveDemandElements->create();
    // create save data elements button
    mySaveDataElements = new MFXButtonTooltip(mySaveIndividualFilesPopup, tooltipMenu,
            std::string("\t") + TL("Save data elements") + std::string("\t") + TL("Save data elements. (Ctrl+Shift+B)"), GUIIconSubSys::getIcon(GUIIcon::SAVE_DATAELEMENTS),
            myViewNet->getViewParent()->getGNEAppWindows(), MID_HOTKEY_CTRL_SHIFT_B_SAVEDATAELEMENTS, GUIDesignButtonPopup);
    mySaveDataElements->create();
    // create save mean datas elements button
    mySaveMeanDataElements = new MFXButtonTooltip(mySaveIndividualFilesPopup, tooltipMenu,
            std::string("\t") + TL("Save mean data elements") + std::string("\t") + TL("Save mean data elements. (Ctrl+Shift+M)"), GUIIconSubSys::getIcon(GUIIcon::SAVE_MEANDATAELEMENTS),
            myViewNet->getViewParent()->getGNEAppWindows(), MID_HOTKEY_CTRL_SHIFT_M_SAVEMEANDATAS, GUIDesignButtonPopup);
    mySaveMeanDataElements->create();
    // recalc menu bar because there is new elements
    gripSaveElements->recalc();
    // show menu bar modes
    gripSaveElements->show();
}


void
GNEViewNetHelper::SaveElements::setSaveIndividualFiles(bool value) {
    if (value) {
        mySaveIndividualFiles->enable();
    } else {
        mySaveIndividualFiles->disable();
    }
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::TimeFormat - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::TimeFormat::TimeFormat(GNEViewNet* viewNet) :
    myViewNet(viewNet) {
}


void
GNEViewNetHelper::TimeFormat::buildTimeFormatButtons() {
    // create save sumo config button
    mySwitchButton = new MFXButtonTooltip(myViewNet->getViewParent()->getGNEAppWindows()->getToolbarsGrip().timeSwitch,
                                          myViewNet->myViewParent->getGNEAppWindows()->getStaticTooltipMenu(),
                                          gHumanReadableTime ? "H" : "S" + std::string("\t") + TL("Switch between seconds and HH:MM:SS") + std::string("\t") + TL("Switch between seconds and HH:MM:SS"), nullptr,
                                          myViewNet->getViewParent()->getGNEAppWindows(), MID_GNE_TOGGLE_TIMEFORMAT, GUIDesignButtonToolbar);
    mySwitchButton->create();
}


void
GNEViewNetHelper::TimeFormat::switchTimeFormat() {
    if (gHumanReadableTime) {
        gHumanReadableTime = false;
    } else {
        gHumanReadableTime = true;
    }
    OptionsCont::getOptions().resetWritable();
    OptionsCont::getOptions().set("human-readable-time", toString(gHumanReadableTime));
}


void
GNEViewNetHelper::TimeFormat::updateButtonLabel() {
    if (gHumanReadableTime) {
        mySwitchButton->setText("H");
    } else {
        mySwitchButton->setText("S");
    }
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::EditModes - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::EditModes::EditModes(GNEViewNet* viewNet) :
    currentSupermode(Supermode::NETWORK),
    networkEditMode(NetworkEditMode::NETWORK_INSPECT),
    demandEditMode(DemandEditMode::DEMAND_INSPECT),
    dataEditMode(DataEditMode::DATA_INSPECT),
    myViewNet(viewNet) {
    auto& neteditOptions = OptionsCont::getOptions();
    // if new option is enabled, start in create edge mode
    if (neteditOptions.getBool("new")) {
        networkEditMode = NetworkEditMode::NETWORK_CREATE_EDGE;
        neteditOptions.resetWritable();
        neteditOptions.set("new", "false");
    }
}


GNEViewNetHelper::EditModes::~EditModes() {
    delete myNeteditViewsPopup;
}


void
GNEViewNetHelper::EditModes::buildSuperModeButtons() {
    // grip supermodes
    auto gripSupermodes = myViewNet->getViewParent()->getGNEAppWindows()->getToolbarsGrip().superModes;
    // static toolTip menu
    auto toolTipMenu = myViewNet->myViewParent->getGNEAppWindows()->getStaticTooltipMenu();
    // create network button
    networkButton = new MFXCheckableButton(false, gripSupermodes, toolTipMenu,
                                           TL("Network") + std::string("\t") + TL("Edit network elements") + std::string("\t") + TL("Set mode for editing network elements. (F2)"),
                                           GUIIconSubSys::getIcon(GUIIcon::SUPERMODENETWORK), myViewNet, MID_HOTKEY_F2_SUPERMODE_NETWORK, GUIDesignMFXCheckableButtonSupermode);
    networkButton->create();
    // create demand button
    demandButton = new MFXCheckableButton(false, gripSupermodes, toolTipMenu,
                                          TL("Demand") + std::string("\t") + TL("Edit traffic demand elements") + std::string("\t") + TL("Set mode for editing traffic demand. (F3)"),
                                          GUIIconSubSys::getIcon(GUIIcon::SUPERMODEDEMAND), myViewNet, MID_HOTKEY_F3_SUPERMODE_DEMAND, GUIDesignMFXCheckableButtonSupermode);
    demandButton->create();
    // create data button
    dataButton = new MFXCheckableButton(false, gripSupermodes, toolTipMenu,
                                        TL("Data") + std::string("\t") + TL("Edit data elements") + std::string("\t") + TL("Set mode for editing data demand. (F4)"),
                                        GUIIconSubSys::getIcon(GUIIcon::SUPERMODEDATA), myViewNet, MID_HOTKEY_F4_SUPERMODE_DATA, GUIDesignMFXCheckableButtonSupermode);
    dataButton->create();
    // build the views popup
    myNeteditViewsPopup = new FXPopup(gripSupermodes, POPUP_VERTICAL);
    myNeteditViewsPopup->create();
    // build views button
    myNeteditViewsButton = new MFXMenuButtonTooltip(gripSupermodes, toolTipMenu,
            (std::string("\t") + TL("Change netedit view") + std::string("\t") + TL("Change netedit view.")).c_str(),
            GUIIconSubSys::getIcon(GUIIcon::VIEWDEFAULT), myNeteditViewsPopup, nullptr, GUIDesignTLSTableCheckableButtonIcon);
    myNeteditViewsButton->create();
    // build default view button
    myDefaultViewButton = new MFXButtonTooltip(myNeteditViewsPopup, toolTipMenu,
            (std::string("Default\t") + TL("Set default view") + std::string("\t") + TL("Set default view.")).c_str(),
            GUIIconSubSys::getIcon(GUIIcon::VIEWDEFAULT), myViewNet, MID_GNE_VIEW_DEFAULT, GUIDesignButtonLeft);
    myDefaultViewButton->create();
    // build jupedsim view button
    myJuPedSimViewButton = new MFXButtonTooltip(myNeteditViewsPopup, toolTipMenu,
            (std::string("JuPedSim\t") + TL("Set JuPedSim view") + std::string("\t") + TL("Set JuPedSim view.")).c_str(),
            GUIIconSubSys::getIcon(GUIIcon::VIEWJUPEDSIM), myViewNet, MID_GNE_VIEW_JUPEDSIM, GUIDesignButtonLeft);
    myJuPedSimViewButton->create();
    myNeteditViewsPopup->recalc();
    // set width (grip + 3 large buttons + icon button)
    gripSupermodes->setWidth(353);
    // show menu bar modes
    gripSupermodes->show();
}


void
GNEViewNetHelper::EditModes::setSupermode(Supermode supermode, const bool force) {
    if (!force && (supermode == currentSupermode)) {
        myViewNet->setStatusBarText(TL("Mode already selected"));
        if (myViewNet->myCurrentFrame != nullptr) {
            myViewNet->myCurrentFrame->focusUpperElement();
        }
    } else {
        myViewNet->setStatusBarText("");
        // abort current operation
        myViewNet->abortOperation(false);
        // set super mode
        currentSupermode = supermode;
        // set supermodes
        if (supermode == Supermode::NETWORK) {
            // change buttons
            networkButton->setChecked(true);
            demandButton->setChecked(false);
            dataButton->setChecked(false);
            // show network buttons
            myViewNet->myNetworkCheckableButtons.showNetworkCheckableButtons();
            // hide demand buttons
            myViewNet->myDemandCheckableButtons.hideDemandCheckableButtons();
            // hide data buttons
            myViewNet->myDataCheckableButtons.hideDataCheckableButtons();
            // force update network mode
            setNetworkEditMode(networkEditMode, true);
        } else if (supermode == Supermode::DEMAND) {
            // change buttons
            networkButton->setChecked(false);
            demandButton->setChecked(true);
            dataButton->setChecked(false);
            // hide network buttons
            myViewNet->myNetworkCheckableButtons.hideNetworkCheckableButtons();
            // show demand buttons
            myViewNet->myDemandCheckableButtons.showDemandCheckableButtons();
            // hide data buttons
            myViewNet->myDataCheckableButtons.hideDataCheckableButtons();
            // force update demand mode
            setDemandEditMode(demandEditMode, true);
            // demand modes require ALWAYS a recomputing
            myViewNet->myNet->computeNetwork(myViewNet->myViewParent->getGNEAppWindows());
            // check if update path calculator
            if (!myViewNet->myNet->getPathManager()->getPathCalculator()->isPathCalculatorUpdated()) {
                // update DijkstraRouter of RouteCalculatorInstance
                myViewNet->myNet->getPathManager()->getPathCalculator()->updatePathCalculator();
                // compute all demand elements
                myViewNet->myNet->computeDemandElements(myViewNet->myViewParent->getGNEAppWindows());
            }
        } else if (supermode == Supermode::DATA) {
            // change buttons
            networkButton->setChecked(false);
            demandButton->setChecked(false);
            dataButton->setChecked(true);
            // hide network buttons
            myViewNet->myNetworkCheckableButtons.hideNetworkCheckableButtons();
            // hide demand buttons
            myViewNet->myDemandCheckableButtons.hideDemandCheckableButtons();
            // show data buttons
            myViewNet->myDataCheckableButtons.showDataCheckableButtons();
            // force update data mode
            setDataEditMode(dataEditMode, true);
            // check if recompute
            if (myViewNet->getViewParent()->getGNEAppWindows()->getProcessingMenuCommands().menuCheckRecomputeDataMode->getCheck() == TRUE) {
                // demand modes require ALWAYS a recomputing
                myViewNet->myNet->computeNetwork(myViewNet->myViewParent->getGNEAppWindows());
            }
            // check if update path calculator
            if (!myViewNet->myNet->getPathManager()->getPathCalculator()->isPathCalculatorUpdated()) {
                // update DijkstraRouter of RouteCalculatorInstance
                myViewNet->myNet->getPathManager()->getPathCalculator()->updatePathCalculator();
                // compute all demand elements
                myViewNet->myNet->computeDemandElements(myViewNet->myViewParent->getGNEAppWindows());
            }
            // reset TAZ contours (due filling)
            for (const auto& TAZ : myViewNet->getNet()->getAttributeCarriers()->getAdditionals().at(SUMO_TAG_TAZ)) {
                TAZ.second->resetAdditionalContour();
            }
        }
        // update buttons
        networkButton->update();
        demandButton->update();
        dataButton->update();
        // update Supermode CommandButtons in GNEAppWindows
        myViewNet->myViewParent->getGNEAppWindows()->updateSuperModeMenuCommands(currentSupermode);
    }
}


void
GNEViewNetHelper::EditModes::setNetworkEditMode(NetworkEditMode mode, const bool force) {
    if ((mode == networkEditMode) && !force) {
        myViewNet->setStatusBarText(TL("Network mode already selected"));
        if (myViewNet->myCurrentFrame != nullptr) {
            myViewNet->myCurrentFrame->focusUpperElement();
        }
    } else if (networkEditMode == NetworkEditMode::NETWORK_TLS && !myViewNet->myViewParent->getTLSEditorFrame()->isTLSSaved()) {
        myViewNet->setStatusBarText(TL("Save modifications in TLS before change mode"));
        myViewNet->myCurrentFrame->focusUpperElement();
    } else {
        myViewNet->setStatusBarText("");
        myViewNet->abortOperation(false);
        // stop editing of custom shapes
        myViewNet->myEditNetworkElementShapes.stopEditCustomShape();
        // set new Network mode
        networkEditMode = mode;
        // for common modes (Inspect/Delete/Select/move) change also the other supermode
        if (networkEditMode == NetworkEditMode::NETWORK_INSPECT) {
            demandEditMode = DemandEditMode::DEMAND_INSPECT;
            dataEditMode = DataEditMode::DATA_INSPECT;
        } else if (networkEditMode == NetworkEditMode::NETWORK_DELETE) {
            demandEditMode = DemandEditMode::DEMAND_DELETE;
            dataEditMode = DataEditMode::DATA_DELETE;
        } else if (networkEditMode == NetworkEditMode::NETWORK_SELECT) {
            demandEditMode = DemandEditMode::DEMAND_SELECT;
            dataEditMode = DataEditMode::DATA_SELECT;
        } else if (networkEditMode == NetworkEditMode::NETWORK_MOVE) {
            demandEditMode = DemandEditMode::DEMAND_MOVE;
        }
        // certain modes require a recomputing
        switch (mode) {
            case NetworkEditMode::NETWORK_PROHIBITION:
            case NetworkEditMode::NETWORK_CONNECT:
            case NetworkEditMode::NETWORK_TLS:
            case NetworkEditMode::NETWORK_WIRE:
                // modes which depend on computed data
                myViewNet->myNet->computeNetwork(myViewNet->myViewParent->getGNEAppWindows());
                break;
            default:
                break;
        }
        // update cursors
        myViewNet->updateCursor();
        // update network mode specific controls
        myViewNet->updateNetworkModeSpecificControls();
    }
}


void
GNEViewNetHelper::EditModes::setDemandEditMode(DemandEditMode mode, const bool force) {
    if ((mode == demandEditMode) && !force) {
        myViewNet->setStatusBarText(TL("Demand mode already selected"));
        if (myViewNet->myCurrentFrame != nullptr) {
            myViewNet->myCurrentFrame->focusUpperElement();
        }
    } else {
        myViewNet->setStatusBarText("");
        myViewNet->abortOperation(false);
        // stop editing of custom shapes
        myViewNet->myEditNetworkElementShapes.stopEditCustomShape();
        // set new Demand mode
        demandEditMode = mode;
        // for common modes (Inspect/Delete/Select/Move) change also the other supermode
        if (demandEditMode == DemandEditMode::DEMAND_INSPECT) {
            networkEditMode = NetworkEditMode::NETWORK_INSPECT;
            dataEditMode = DataEditMode::DATA_INSPECT;
        } else if (demandEditMode == DemandEditMode::DEMAND_DELETE) {
            networkEditMode = NetworkEditMode::NETWORK_DELETE;
            dataEditMode = DataEditMode::DATA_DELETE;
        } else if (demandEditMode == DemandEditMode::DEMAND_SELECT) {
            networkEditMode = NetworkEditMode::NETWORK_SELECT;
            dataEditMode = DataEditMode::DATA_SELECT;
        } else if (demandEditMode == DemandEditMode::DEMAND_MOVE) {
            networkEditMode = NetworkEditMode::NETWORK_MOVE;
        }
        // update cursors
        myViewNet->updateCursor();
        // update network mode specific controls
        myViewNet->updateDemandModeSpecificControls();
    }
}


void
GNEViewNetHelper::EditModes::setDataEditMode(DataEditMode mode, const bool force) {
    if ((mode == dataEditMode) && !force) {
        myViewNet->setStatusBarText(TL("Data mode already selected"));
        if (myViewNet->myCurrentFrame != nullptr) {
            myViewNet->myCurrentFrame->focusUpperElement();
        }
    } else {
        myViewNet->setStatusBarText("");
        myViewNet->abortOperation(false);
        // stop editing of custom shapes
        myViewNet->myEditNetworkElementShapes.stopEditCustomShape();
        // set new Data mode
        dataEditMode = mode;
        // for common modes (Inspect/Delete/Select/Move) change also the other supermode
        if (dataEditMode == DataEditMode::DATA_INSPECT) {
            networkEditMode = NetworkEditMode::NETWORK_INSPECT;
            demandEditMode = DemandEditMode::DEMAND_INSPECT;
        } else if (dataEditMode == DataEditMode::DATA_DELETE) {
            networkEditMode = NetworkEditMode::NETWORK_DELETE;
            demandEditMode = DemandEditMode::DEMAND_DELETE;
        } else if (dataEditMode == DataEditMode::DATA_SELECT) {
            networkEditMode = NetworkEditMode::NETWORK_SELECT;
            demandEditMode = DemandEditMode::DEMAND_SELECT;
        }
        // update all datasets
        for (const auto& dataSet : myViewNet->getNet()->getAttributeCarriers()->getDataSets()) {
            dataSet.second->updateAttributeColors();
        }
        // update cursors
        myViewNet->updateCursor();
        // update network mode specific controls
        myViewNet->updateDataModeSpecificControls();
    }
}


bool
GNEViewNetHelper::EditModes::isCurrentSupermodeNetwork() const {
    return (currentSupermode == Supermode::NETWORK);
}


bool
GNEViewNetHelper::EditModes::isCurrentSupermodeDemand() const {
    return (currentSupermode == Supermode::DEMAND);
}


bool
GNEViewNetHelper::EditModes::isCurrentSupermodeData() const {
    return (currentSupermode == Supermode::DATA);
}


void
GNEViewNetHelper::EditModes::setView(FXSelector sel) {
    // grip supermodes
    auto gripSupermodes = myViewNet->getViewParent()->getGNEAppWindows()->getToolbarsGrip().superModes;
    // file menu commands
    auto& fileMenuCommands = myViewNet->getViewParent()->getGNEAppWindows()->getFileMenuCommands();
    // continue depending of selector
    if (sel == MID_GNE_VIEW_DEFAULT) {
        myNeteditViewsButton->setIcon(GUIIconSubSys::getIcon(GUIIcon::VIEWDEFAULT));
        gripSupermodes->setWidth(353);
        // show menu commands
        fileMenuCommands.setDefaultView();
    } else if (sel == MID_GNE_VIEW_JUPEDSIM) {
        myNeteditViewsButton->setIcon(GUIIconSubSys::getIcon(GUIIcon::VIEWJUPEDSIM));
        gripSupermodes->setWidth(250);
        // hide menu commands
        fileMenuCommands.setJuPedSimView();
    }
    // update viewNet
    myViewNet->viewUpdated();
}


bool
GNEViewNetHelper::EditModes::isDefaultView() const {
    return myNeteditViewsButton->getIcon() == GUIIconSubSys::getIcon(GUIIcon::VIEWDEFAULT);
}


bool
GNEViewNetHelper::EditModes::isJuPedSimView() const {
    return myNeteditViewsButton->getIcon() == GUIIconSubSys::getIcon(GUIIcon::VIEWJUPEDSIM);
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::NetworkViewOptions - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::NetworkViewOptions::NetworkViewOptions(GNEViewNet* viewNet) :
    myViewNet(viewNet) {
}


void
GNEViewNetHelper::NetworkViewOptions::buildNetworkViewOptionsMenuChecks() {
    // get grip modes
    auto gripModes = myViewNet->myViewParent->getGNEAppWindows()->getToolbarsGrip().modes;
    // static toolTip menu
    auto toolTipMenu = myViewNet->myViewParent->getGNEAppWindows()->getStaticTooltipMenu();
    // create menu checks
    menuCheckToggleGrid = new MFXCheckableButton(false, gripModes, toolTipMenu,
            (std::string("\t") + TL("Show grid") + std::string("\t") + TL("Show grid and restrict movement to the grid - define grid size in visualization options. (Ctrl+G)")),
            GUIIconSubSys::getIcon(GUIIcon::COMMONMODE_CHECKBOX_TOGGLEGRID),
            myViewNet, MID_GNE_NETWORKVIEWOPTIONS_TOGGLEGRID, GUIDesignMFXCheckableButtonSquare);
    menuCheckToggleGrid->setChecked(false);
    menuCheckToggleGrid->create();

    menuCheckToggleDrawJunctionShape = new MFXCheckableButton(false, gripModes, toolTipMenu,
            (std::string("\t") + TL("Hide junction shape") + std::string("\t") + TL("Toggle hiding junction shape. (Ctrl+J)")),
            GUIIconSubSys::getIcon(GUIIcon::COMMONMODE_CHECKBOX_TOGGLEDRAWJUNCTIONSHAPE),
            myViewNet, MID_GNE_NETWORKVIEWOPTIONS_TOGGLEDRAWJUNCTIONSHAPE, GUIDesignMFXCheckableButtonSquare);
    menuCheckToggleDrawJunctionShape->setChecked(false);
    menuCheckToggleDrawJunctionShape->create();

    menuCheckDrawSpreadVehicles = new MFXCheckableButton(false, gripModes, toolTipMenu,
            (std::string("\t") + TL("Draw vehicles spread in lane or depart position") + std::string("\t") + TL("Draw vehicles spread in lane or in depart position.")),
            GUIIconSubSys::getIcon(GUIIcon::COMMONMODE_CHECKBOX_SPREADVEHICLE),
            myViewNet, MID_GNE_NETWORKVIEWOPTIONS_DRAWSPREADVEHICLES, GUIDesignMFXCheckableButtonSquare);
    menuCheckDrawSpreadVehicles->setChecked(false);
    menuCheckDrawSpreadVehicles->create();

    menuCheckShowDemandElements = new MFXCheckableButton(false, gripModes, toolTipMenu,
            (std::string("\t") + TL("Show demand elements") + std::string("\t") + TL("Toggle show demand elements.")),
            GUIIconSubSys::getIcon(GUIIcon::COMMONMODE_CHECKBOX_SHOWDEMANDELEMENTS),
            myViewNet, MID_GNE_NETWORKVIEWOPTIONS_SHOWDEMANDELEMENTS, GUIDesignMFXCheckableButtonSquare);
    menuCheckShowDemandElements->setChecked(false);
    menuCheckShowDemandElements->create();

    menuCheckSelectEdges = new MFXCheckableButton(false, gripModes, toolTipMenu,
            (std::string("\t") + TL("Clicks target lanes") + std::string("\t") + TL("Toggle whether clicking should inspect/select/delete lanes instead of edges.")),
            GUIIconSubSys::getIcon(GUIIcon::NETWORKMODE_CHECKBOX_SELECTEDGES),
            myViewNet, MID_GNE_NETWORKVIEWOPTIONS_SELECTEDGES, GUIDesignMFXCheckableButtonSquare);
    menuCheckSelectEdges->create();

    menuCheckShowConnections = new MFXCheckableButton(false, gripModes, toolTipMenu,
            (std::string("\t") + TL("Show connections over junctions") + std::string("\t") + TL("Toggle show connections over junctions.")),
            GUIIconSubSys::getIcon(GUIIcon::NETWORKMODE_CHECKBOX_SHOWCONNECTIONS),
            myViewNet, MID_GNE_NETWORKVIEWOPTIONS_SHOWCONNECTIONS, GUIDesignMFXCheckableButtonSquare);
    menuCheckShowConnections->setChecked(myViewNet->getVisualisationSettings().showLane2Lane);
    menuCheckShowConnections->create();

    menuCheckHideConnections = new MFXCheckableButton(false, gripModes, toolTipMenu,
            (std::string("\t") + TL("Hide connections") + std::string("\t") + TL("Toggle hide connections.")),
            GUIIconSubSys::getIcon(GUIIcon::NETWORKMODE_CHECKBOX_HIDECONNECTIONS),
            myViewNet, MID_GNE_NETWORKVIEWOPTIONS_HIDECONNECTIONS, GUIDesignMFXCheckableButtonSquare);
    menuCheckHideConnections->setChecked(false);
    menuCheckHideConnections->create();

    menuCheckShowAdditionalSubElements = new MFXCheckableButton(false, gripModes, toolTipMenu,
            (std::string("\t") + TL("Show additional sub-elements") + std::string("\t") + TL("Toggle show additional sub-elements.")),
            GUIIconSubSys::getIcon(GUIIcon::NETWORKMODE_CHECKBOX_SHOWSUBADDITIONALS),
            myViewNet, MID_GNE_NETWORKVIEWOPTIONS_SHOWSUBADDITIONALS, GUIDesignMFXCheckableButtonSquare);
    menuCheckShowAdditionalSubElements->setChecked(false);
    menuCheckShowAdditionalSubElements->create();

    menuCheckShowTAZElements = new MFXCheckableButton(false, gripModes, toolTipMenu,
            (std::string("\t") + TL("Show TAZ elements") + std::string("\t") + TL("Toggle show TAZ elements.")),
            GUIIconSubSys::getIcon(GUIIcon::NETWORKMODE_CHECKBOX_SHOWTAZELEMENTS),
            myViewNet, MID_GNE_NETWORKVIEWOPTIONS_SHOWTAZELEMENTS, GUIDesignMFXCheckableButtonSquare);
    menuCheckShowTAZElements->setChecked(false);
    menuCheckShowTAZElements->create();

    menuCheckExtendSelection = new MFXCheckableButton(false, gripModes, toolTipMenu,
            (std::string("\t") + TL("Automatic select junctions") + std::string("\t") + TL("Toggle whether selecting multiple edges should automatically select their junctions.")),
            GUIIconSubSys::getIcon(GUIIcon::NETWORKMODE_CHECKBOX_AUTOSELECTJUNCTIONS),
            myViewNet, MID_GNE_NETWORKVIEWOPTIONS_EXTENDSELECTION, GUIDesignMFXCheckableButtonSquare);
    menuCheckExtendSelection->setChecked(true);
    menuCheckExtendSelection->create();

    menuCheckChangeAllPhases = new MFXCheckableButton(false, gripModes, toolTipMenu,
            (std::string("\t") + TL("Apply state to all phases") + std::string("\t") + TL("Toggle whether clicking should apply state changes to all phases of the current TLS plan.")),
            GUIIconSubSys::getIcon(GUIIcon::NETWORKMODE_CHECKBOX_APPLYTOALLPHASES),
            myViewNet, MID_GNE_NETWORKVIEWOPTIONS_CHANGEALLPHASES, GUIDesignMFXCheckableButtonSquare);
    menuCheckChangeAllPhases->setChecked(false);
    menuCheckChangeAllPhases->create();

    menuCheckWarnAboutMerge = new MFXCheckableButton(false, gripModes, toolTipMenu,
            (std::string("\t") + TL("Automatic merging junction") + std::string("\t") + TL("Toggle ask for confirmation before merging junction.")),
            GUIIconSubSys::getIcon(GUIIcon::NETWORKMODE_CHECKBOX_ASKFORMERGE),
            myViewNet, MID_GNE_NETWORKVIEWOPTIONS_ASKFORMERGE, GUIDesignMFXCheckableButtonSquare);
    menuCheckWarnAboutMerge->create();

    menuCheckShowJunctionBubble = new MFXCheckableButton(false, gripModes, toolTipMenu,
            (std::string("\t") + TL("Show bubbles") + std::string("\t") + TL("Toggle show bubbles over junctions shapes.")),
            GUIIconSubSys::getIcon(GUIIcon::NETWORKMODE_CHECKBOX_BUBBLES),
            myViewNet, MID_GNE_NETWORKVIEWOPTIONS_SHOWBUBBLES, GUIDesignMFXCheckableButtonSquare);
    menuCheckShowJunctionBubble->setChecked(false);
    menuCheckShowJunctionBubble->create();

    menuCheckMoveElevation = new MFXCheckableButton(false, gripModes, toolTipMenu,
            (std::string("\t") + TL("Move elevation") + std::string("\t") + TL("Apply mouse movement to elevation instead of x,y position.")),
            GUIIconSubSys::getIcon(GUIIcon::NETWORKMODE_CHECKBOX_ELEVATION),
            myViewNet, MID_GNE_NETWORKVIEWOPTIONS_MOVEELEVATION, GUIDesignMFXCheckableButtonSquare);
    menuCheckMoveElevation->setChecked(false);
    menuCheckMoveElevation->create();

    menuCheckChainEdges = new MFXCheckableButton(false, gripModes, toolTipMenu,
            (std::string("\t") + TL("Edge chain mode") + std::string("\t") + TL("Create consecutive edges with a single click (hit ESC to cancel chain).")),
            GUIIconSubSys::getIcon(GUIIcon::NETWORKMODE_CHECKBOX_CHAIN),
            myViewNet, MID_GNE_NETWORKVIEWOPTIONS_CHAINEDGES, GUIDesignMFXCheckableButtonSquare);
    menuCheckChainEdges->setChecked(false);
    menuCheckChainEdges->create();

    menuCheckAutoOppositeEdge = new MFXCheckableButton(false, gripModes, toolTipMenu,
            (std::string("\t") + TL("Edge opposite direction") + std::string("\t") + TL("Automatically create an edge in the opposite direction.")),
            GUIIconSubSys::getIcon(GUIIcon::NETWORKMODE_CHECKBOX_TWOWAY),
            myViewNet, MID_GNE_NETWORKVIEWOPTIONS_AUTOOPPOSITEEDGES, GUIDesignMFXCheckableButtonSquare);
    menuCheckAutoOppositeEdge->setChecked(false);
    menuCheckAutoOppositeEdge->create();

    // always recalc after creating new elements
    gripModes->recalc();
}


void
GNEViewNetHelper::NetworkViewOptions::hideNetworkViewOptionsMenuChecks() {
    menuCheckToggleGrid->hide();
    menuCheckToggleDrawJunctionShape->hide();
    menuCheckDrawSpreadVehicles->hide();
    menuCheckShowDemandElements->hide();
    menuCheckSelectEdges->hide();
    menuCheckShowConnections->hide();
    menuCheckHideConnections->hide();
    menuCheckShowAdditionalSubElements->hide();
    menuCheckShowTAZElements->hide();
    menuCheckExtendSelection->hide();
    menuCheckChangeAllPhases->hide();
    menuCheckWarnAboutMerge->hide();
    menuCheckShowJunctionBubble->hide();
    menuCheckMoveElevation->hide();
    menuCheckChainEdges->hide();
    menuCheckAutoOppositeEdge->hide();
}


void
GNEViewNetHelper::NetworkViewOptions::getVisibleNetworkMenuCommands(std::vector<MFXCheckableButton*>& commands) const {
    // save visible menu commands in commands vector
    if (menuCheckToggleGrid->shown()) {
        commands.push_back(menuCheckToggleGrid);
    }
    if (menuCheckToggleDrawJunctionShape->shown()) {
        commands.push_back(menuCheckToggleDrawJunctionShape);
    }
    if (menuCheckDrawSpreadVehicles->shown()) {
        commands.push_back(menuCheckDrawSpreadVehicles);
    }
    if (menuCheckShowDemandElements->shown()) {
        commands.push_back(menuCheckShowDemandElements);
    }
    if (menuCheckSelectEdges->shown()) {
        commands.push_back(menuCheckSelectEdges);
    }
    if (menuCheckShowConnections->shown()) {
        commands.push_back(menuCheckShowConnections);
    }
    if (menuCheckHideConnections->shown()) {
        commands.push_back(menuCheckHideConnections);
    }
    if (menuCheckShowAdditionalSubElements->shown()) {
        commands.push_back(menuCheckShowAdditionalSubElements);
    }
    if (menuCheckShowTAZElements->shown()) {
        commands.push_back(menuCheckShowTAZElements);
    }
    if (menuCheckExtendSelection->shown()) {
        commands.push_back(menuCheckExtendSelection);
    }
    if (menuCheckChangeAllPhases->shown()) {
        commands.push_back(menuCheckChangeAllPhases);
    }
    if (menuCheckWarnAboutMerge->shown()) {
        commands.push_back(menuCheckWarnAboutMerge);
    }
    if (menuCheckShowJunctionBubble->shown()) {
        commands.push_back(menuCheckShowJunctionBubble);
    }
    if (menuCheckMoveElevation->shown()) {
        commands.push_back(menuCheckMoveElevation);
    }
    if (menuCheckChainEdges->shown()) {
        commands.push_back(menuCheckChainEdges);
    }
    if (menuCheckAutoOppositeEdge->shown()) {
        commands.push_back(menuCheckAutoOppositeEdge);
    }
}


bool
GNEViewNetHelper::NetworkViewOptions::drawSpreadVehicles() const {
    return (menuCheckDrawSpreadVehicles->amChecked() == TRUE);
}


bool
GNEViewNetHelper::NetworkViewOptions::showDemandElements() const {
    if (menuCheckShowDemandElements->shown()) {
        return (menuCheckShowDemandElements->amChecked() == TRUE);
    } else {
        // by default, if menuCheckShowDemandElements isn't shown, always show demand elements
        return true;
    }
}


bool
GNEViewNetHelper::NetworkViewOptions::selectEdges() const {
    if (menuCheckSelectEdges->shown()) {
        return (menuCheckSelectEdges->amChecked() != TRUE);
    } else {
        // by default, if menuCheckSelectEdges isn't shown, always select edges
        return true;
    }
}


bool
GNEViewNetHelper::NetworkViewOptions::showConnections() const {
    if (myViewNet->myEditModes.isCurrentSupermodeData()) {
        return false;
    } else if (myViewNet->myEditModes.networkEditMode == NetworkEditMode::NETWORK_CONNECT) {
        // check if menu check hide connections ins shown
        return (menuCheckHideConnections->amChecked() == FALSE);
    } else if (myViewNet->myEditModes.networkEditMode == NetworkEditMode::NETWORK_PROHIBITION) {
        return true;
    } else if (myViewNet->myEditModes.isCurrentSupermodeNetwork() && menuCheckShowConnections->shown() == false) {
        return false;
    } else {
        return (myViewNet->getVisualisationSettings().showLane2Lane);
    }
}


bool
GNEViewNetHelper::NetworkViewOptions::showSubAdditionals() const {
    if (!myViewNet->myEditModes.isCurrentSupermodeNetwork()) {
        return false;
    } else if (menuCheckShowAdditionalSubElements->shown() == false) {
        return false;
    } else {
        return menuCheckShowAdditionalSubElements->amChecked();
    }
}


bool
GNEViewNetHelper::NetworkViewOptions::showTAZElements() const {
    if (!myViewNet->myEditModes.isCurrentSupermodeNetwork()) {
        return false;
    } else if (menuCheckShowTAZElements->shown() == false) {
        return false;
    } else {
        return menuCheckShowTAZElements->amChecked();
    }
}


bool
GNEViewNetHelper::NetworkViewOptions::editingElevation() const {
    if (menuCheckMoveElevation->shown()) {
        return (menuCheckMoveElevation->amChecked() == TRUE);
    } else {
        return false;
    }
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::DemandViewOptions - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::DemandViewOptions::DemandViewOptions(GNEViewNet* viewNet) :
    myViewNet(viewNet) {
}


void
GNEViewNetHelper::DemandViewOptions::buildDemandViewOptionsMenuChecks() {
    // get grip modes
    auto gripModes = myViewNet->getViewParent()->getGNEAppWindows()->getToolbarsGrip().modes;
    // get tooltip menu
    auto tooltipMenu = myViewNet->myViewParent->getGNEAppWindows()->getStaticTooltipMenu();
    // create menu checks
    menuCheckToggleGrid = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Show grid") + std::string("\t") + TL("Show grid and restrict movement to the grid - define grid size in visualization options. (Ctrl+G)")),
            GUIIconSubSys::getIcon(GUIIcon::COMMONMODE_CHECKBOX_TOGGLEGRID),
            myViewNet, MID_GNE_DEMANDVIEWOPTIONS_SHOWGRID, GUIDesignMFXCheckableButtonSquare);
    menuCheckToggleGrid->setChecked(false);
    menuCheckToggleGrid->create();

    menuCheckToggleDrawJunctionShape = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Hide junction shape") + std::string("\t") + TL("Toggle hiding junction shape. (Ctrl+J)")),
            GUIIconSubSys::getIcon(GUIIcon::COMMONMODE_CHECKBOX_TOGGLEDRAWJUNCTIONSHAPE),
            myViewNet, MID_GNE_DEMANDVIEWOPTIONS_TOGGLEDRAWJUNCTIONSHAPE, GUIDesignMFXCheckableButtonSquare);
    menuCheckToggleDrawJunctionShape->setChecked(false);
    menuCheckToggleDrawJunctionShape->create();

    menuCheckDrawSpreadVehicles = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Draw vehicles spread/depart position") + std::string("\t") + TL("Draw vehicles spread in lane or in depart position.")),
            GUIIconSubSys::getIcon(GUIIcon::COMMONMODE_CHECKBOX_SPREADVEHICLE),
            myViewNet, MID_GNE_DEMANDVIEWOPTIONS_DRAWSPREADVEHICLES, GUIDesignMFXCheckableButtonSquare);
    menuCheckDrawSpreadVehicles->setChecked(false);
    menuCheckDrawSpreadVehicles->create();

    menuCheckHideShapes = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Show shapes") + std::string("\t") + TL("Toggle show shapes (Polygons and POIs).")),
            GUIIconSubSys::getIcon(GUIIcon::DEMANDMODE_CHECKBOX_HIDESHAPES),
            myViewNet, MID_GNE_DEMANDVIEWOPTIONS_HIDESHAPES, GUIDesignMFXCheckableButtonSquare);
    menuCheckHideShapes->setChecked(false);
    menuCheckHideShapes->create();

    menuCheckShowAllTrips = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Show all trips") + std::string("\t") + TL("Toggle show all trips (requires updated demand - F5).")),
            GUIIconSubSys::getIcon(GUIIcon::DEMANDMODE_CHECKBOX_SHOWTRIPS),
            myViewNet, MID_GNE_DEMANDVIEWOPTIONS_SHOWTRIPS, GUIDesignMFXCheckableButtonSquare);
    menuCheckShowAllTrips->setChecked(false);
    menuCheckShowAllTrips->create();

    menuCheckShowAllPersonPlans = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Show all person plans") + std::string("\t") + TL("Toggle show all person plans.")),
            GUIIconSubSys::getIcon(GUIIcon::DEMANDMODE_CHECKBOX_SHOWPERSONPLANS),
            myViewNet, MID_GNE_DEMANDVIEWOPTIONS_SHOWALLPERSONPLANS, GUIDesignMFXCheckableButtonSquare);
    menuCheckShowAllPersonPlans->setChecked(false);
    menuCheckShowAllPersonPlans->create();

    menuCheckLockPerson = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Lock selected person") + std::string("\t") + TL("Toggle lock selected person.")),
            GUIIconSubSys::getIcon(GUIIcon::DEMANDMODE_CHECKBOX_LOCKPERSON),
            myViewNet, MID_GNE_DEMANDVIEWOPTIONS_LOCKPERSON, GUIDesignMFXCheckableButtonSquare);
    menuCheckLockPerson->setChecked(false);
    menuCheckLockPerson->create();

    menuCheckShowAllContainerPlans = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Show all containers plans") + std::string("\t") + TL("Toggle show all container plans.")),
            GUIIconSubSys::getIcon(GUIIcon::DEMANDMODE_CHECKBOX_SHOWCONTAINERPLANS),
            myViewNet, MID_GNE_DEMANDVIEWOPTIONS_SHOWALLCONTAINERPLANS, GUIDesignMFXCheckableButtonSquare);
    menuCheckShowAllContainerPlans->setChecked(false);
    menuCheckShowAllContainerPlans->create();

    menuCheckLockContainer = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Lock selected container") + std::string("\t") + TL("Toggle lock selected container.")),
            GUIIconSubSys::getIcon(GUIIcon::DEMANDMODE_CHECKBOX_LOCKCONTAINER),
            myViewNet, MID_GNE_DEMANDVIEWOPTIONS_LOCKCONTAINER, GUIDesignMFXCheckableButtonSquare);
    menuCheckLockContainer->setChecked(false);
    menuCheckLockContainer->create();

    menuCheckHideNonInspectedDemandElements = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Show non-inspected demand elements") + std::string("\t") + TL("Toggle show non-inspected demand elements.")),
            GUIIconSubSys::getIcon(GUIIcon::DEMANDMODE_CHECKBOX_HIDENONINSPECTEDDEMANDELEMENTS),
            myViewNet, MID_GNE_DEMANDVIEWOPTIONS_HIDENONINSPECTED, GUIDesignMFXCheckableButtonSquare);
    menuCheckHideNonInspectedDemandElements->setChecked(false);
    menuCheckHideNonInspectedDemandElements->create();

    menuCheckShowOverlappedRoutes = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Show number of overlapped routes") + std::string("\t") + TL("Toggle show number of overlapped routes.")),
            GUIIconSubSys::getIcon(GUIIcon::DEMANDMODE_CHECKBOX_SHOWOVERLAPPEDROUTES),
            myViewNet, MID_GNE_DEMANDVIEWOPTIONS_SHOWOVERLAPPEDROUTES, GUIDesignMFXCheckableButtonSquare);
    menuCheckShowOverlappedRoutes->setChecked(false);
    menuCheckShowOverlappedRoutes->create();

    // always recalc after creating new elements
    gripModes->recalc();
}


void
GNEViewNetHelper::DemandViewOptions::hideDemandViewOptionsMenuChecks() {
    menuCheckToggleGrid->hide();
    menuCheckToggleDrawJunctionShape->hide();
    menuCheckDrawSpreadVehicles->hide();
    menuCheckHideShapes->hide();
    menuCheckShowAllTrips->hide();
    menuCheckShowAllPersonPlans->hide();
    menuCheckLockPerson->hide();
    menuCheckShowAllContainerPlans->hide();
    menuCheckLockContainer->hide();
    menuCheckHideNonInspectedDemandElements->hide();
    menuCheckShowOverlappedRoutes->hide();
}


void
GNEViewNetHelper::DemandViewOptions::getVisibleDemandMenuCommands(std::vector<MFXCheckableButton*>& commands) const {
    // save visible menu commands in commands vector
    if (menuCheckToggleGrid->shown()) {
        commands.push_back(menuCheckToggleGrid);
    }
    if (menuCheckToggleDrawJunctionShape->shown()) {
        commands.push_back(menuCheckToggleDrawJunctionShape);
    }
    if (menuCheckDrawSpreadVehicles->shown()) {
        commands.push_back(menuCheckDrawSpreadVehicles);
    }
    if (menuCheckHideShapes->shown()) {
        commands.push_back(menuCheckHideShapes);
    }
    if (menuCheckShowAllTrips->shown()) {
        commands.push_back(menuCheckShowAllTrips);
    }
    if (menuCheckShowAllPersonPlans->shown() && menuCheckShowAllPersonPlans->isEnabled()) {
        commands.push_back(menuCheckShowAllPersonPlans);
    }
    if (menuCheckLockPerson->shown() && menuCheckLockPerson->isEnabled()) {
        commands.push_back(menuCheckLockPerson);
    }
    if (menuCheckShowAllContainerPlans->shown() && menuCheckShowAllContainerPlans->isEnabled()) {
        commands.push_back(menuCheckShowAllContainerPlans);
    }
    if (menuCheckLockContainer->shown() && menuCheckLockContainer->isEnabled()) {
        commands.push_back(menuCheckLockContainer);
    }
    if (menuCheckHideNonInspectedDemandElements->shown()) {
        commands.push_back(menuCheckHideNonInspectedDemandElements);
    }
    if (menuCheckShowOverlappedRoutes->shown()) {
        commands.push_back(menuCheckShowOverlappedRoutes);
    }
}


bool
GNEViewNetHelper::DemandViewOptions::drawSpreadVehicles() const {
    return (menuCheckDrawSpreadVehicles->amChecked() == TRUE);
}


bool
GNEViewNetHelper::DemandViewOptions::showNonInspectedDemandElements(const GNEDemandElement* demandElement) const {
    if (menuCheckHideNonInspectedDemandElements->shown()) {
        // check conditions
        if ((menuCheckHideNonInspectedDemandElements->amChecked() == FALSE) || (myViewNet->getInspectedAttributeCarriers().empty())) {
            // if checkbox is disabled or there isn't insepected element, then return true
            return true;
        } else if (myViewNet->getInspectedAttributeCarriers().front()->getTagProperty().isDemandElement()) {
            if (myViewNet->isAttributeCarrierInspected(demandElement)) {
                // if inspected element correspond to demandElement, return true
                return true;
            } else {
                // if demandElement is a route, check if dottedAC is one of their children (Vehicle or Stop)
                for (const auto& i : demandElement->getChildDemandElements()) {
                    if (myViewNet->isAttributeCarrierInspected(i)) {
                        return true;
                    }
                }
                // if demandElement is a vehicle, check if dottedAC is one of his route Parent
                for (const auto& i : demandElement->getParentDemandElements()) {
                    if (myViewNet->isAttributeCarrierInspected(i)) {
                        return true;
                    }
                }
                // dottedAC isn't one of their parent, then return false
                return false;
            }
        } else {
            // we're inspecting a demand element, then return true
            return true;
        }
    } else {
        // we're inspecting a demand element, then return true
        return true;
    }
}


bool
GNEViewNetHelper::DemandViewOptions::showShapes() const {
    if (menuCheckHideShapes->shown()) {
        return (menuCheckHideShapes->amChecked() == FALSE);
    } else {
        return true;
    }
}


bool
GNEViewNetHelper::DemandViewOptions::showAllTrips() const {
    return (menuCheckShowAllTrips->amChecked() == TRUE);
}


bool
GNEViewNetHelper::DemandViewOptions::showAllPersonPlans() const {
    if (menuCheckShowAllPersonPlans->isEnabled()) {
        return (menuCheckShowAllPersonPlans->amChecked() == TRUE);
    } else {
        return false;
    }
}


void
GNEViewNetHelper::DemandViewOptions::lockPerson(const GNEDemandElement* person) {
    myLockedPerson = person;
}


void
GNEViewNetHelper::DemandViewOptions::unlockPerson() {
    myLockedPerson = nullptr;
}


const GNEDemandElement*
GNEViewNetHelper::DemandViewOptions::getLockedPerson() const {
    return myLockedPerson;
}


bool
GNEViewNetHelper::DemandViewOptions::showAllContainerPlans() const {
    if (menuCheckShowAllContainerPlans->isEnabled()) {
        return (menuCheckShowAllContainerPlans->amChecked() == TRUE);
    } else {
        return false;
    }
}


void
GNEViewNetHelper::DemandViewOptions::lockContainer(const GNEDemandElement* container) {
    myLockedContainer = container;
}


void
GNEViewNetHelper::DemandViewOptions::unlockContainer() {
    myLockedContainer = nullptr;
}


bool
GNEViewNetHelper::DemandViewOptions::showOverlappedRoutes() const {
    if (menuCheckShowOverlappedRoutes->isEnabled()) {
        return (menuCheckShowOverlappedRoutes->amChecked() == TRUE);
    } else {
        return false;
    }
}


const GNEDemandElement*
GNEViewNetHelper::DemandViewOptions::getLockedContainer() const {
    return myLockedContainer;
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::DataViewOptions - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::DataViewOptions::DataViewOptions(GNEViewNet* viewNet) :
    myViewNet(viewNet) {
}


void
GNEViewNetHelper::DataViewOptions::buildDataViewOptionsMenuChecks() {
    // get grip modes
    auto gripModes = myViewNet->getViewParent()->getGNEAppWindows()->getToolbarsGrip().modes;
    // get tooltip menu
    auto tooltipMenu = myViewNet->myViewParent->getGNEAppWindows()->getStaticTooltipMenu();
    // create menu checks
    menuCheckToggleDrawJunctionShape = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Hide junction shape") + std::string("\t") + TL("Toggle hiding junction shape. (Ctrl+J)")),
            GUIIconSubSys::getIcon(GUIIcon::COMMONMODE_CHECKBOX_TOGGLEDRAWJUNCTIONSHAPE),
            myViewNet, MID_GNE_DATAVIEWOPTIONS_TOGGLEDRAWJUNCTIONSHAPE, GUIDesignMFXCheckableButtonSquare);
    menuCheckToggleDrawJunctionShape->setChecked(false);
    menuCheckToggleDrawJunctionShape->create();

    menuCheckShowAdditionals = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Show additionals") + std::string("\t") + TL("Toggle show additionals.")),
            GUIIconSubSys::getIcon(GUIIcon::DATAMODE_CHECKBOX_SHOWADDITIONALS),
            myViewNet, MID_GNE_DATAVIEWOPTIONS_SHOWADDITIONALS, GUIDesignMFXCheckableButtonSquare);
    menuCheckShowAdditionals->create();

    menuCheckShowShapes = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Show shapes") + std::string("\t") + TL("Toggle show shapes (Polygons and POIs).")),
            GUIIconSubSys::getIcon(GUIIcon::DATAMODE_CHECKBOX_SHOWSHAPES),
            myViewNet, MID_GNE_DATAVIEWOPTIONS_SHOWSHAPES, GUIDesignMFXCheckableButtonSquare);
    menuCheckShowShapes->create();

    menuCheckShowDemandElements = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Show demand elements") + std::string("\t") + TL("Toggle show demand elements.")),
            GUIIconSubSys::getIcon(GUIIcon::COMMONMODE_CHECKBOX_SHOWDEMANDELEMENTS),
            myViewNet, MID_GNE_DATAVIEWOPTIONS_SHOWDEMANDELEMENTS, GUIDesignMFXCheckableButtonSquare);
    menuCheckShowDemandElements->create();

    menuCheckToggleTAZRelDrawing = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Draw TAZREL drawing mode") + std::string("\t") + TL("Toggle draw TAZREL drawing mode.")),
            GUIIconSubSys::getIcon(GUIIcon::DATAMODE_CHECKBOX_TAZRELDRAWING),
            myViewNet, MID_GNE_DATAVIEWOPTIONS_TAZRELDRAWING, GUIDesignMFXCheckableButtonSquare);
    menuCheckToggleTAZRelDrawing->create();

    menuCheckToggleTAZDrawFill = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Toggle draw TAZ fill") + std::string("\t") + TL("Toggle draw TAZ fill")),
            GUIIconSubSys::getIcon(GUIIcon::DATAMODE_CHECKBOX_TAZDRAWFILL),
            myViewNet, MID_GNE_DATAVIEWOPTIONS_TAZDRAWFILL, GUIDesignMFXCheckableButtonSquare);
    menuCheckToggleTAZDrawFill->create();

    menuCheckToggleTAZRelOnlyFrom = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Toggle draw TAZRel only from") + std::string("\t") + TL("Toggle draw TAZRel only from")),
            GUIIconSubSys::getIcon(GUIIcon::DATAMODE_CHECKBOX_TAZRELONLYFROM),
            myViewNet, MID_GNE_DATAVIEWOPTIONS_TAZRELONLYFROM, GUIDesignMFXCheckableButtonSquare);
    menuCheckToggleTAZRelOnlyFrom->create();

    menuCheckToggleTAZRelOnlyTo = new MFXCheckableButton(false, gripModes, tooltipMenu,
            (std::string("\t") + TL("Toggle draw TAZRel only to") + std::string("\t") + TL("Toggle draw TAZRel only to")),
            GUIIconSubSys::getIcon(GUIIcon::DATAMODE_CHECKBOX_TAZRELONLYTO),
            myViewNet, MID_GNE_DATAVIEWOPTIONS_TAZRELONLYTO, GUIDesignMFXCheckableButtonSquare);
    menuCheckToggleTAZRelOnlyTo->create();

    // always recalc after creating new elements
    gripModes->recalc();
}


void
GNEViewNetHelper::DataViewOptions::hideDataViewOptionsMenuChecks() {
    menuCheckToggleDrawJunctionShape->hide();
    menuCheckShowAdditionals->hide();
    menuCheckShowShapes->hide();
    menuCheckShowDemandElements->hide();
    menuCheckToggleTAZRelDrawing->hide();
    menuCheckToggleTAZDrawFill->hide();
    menuCheckToggleTAZRelOnlyFrom->hide();
    menuCheckToggleTAZRelOnlyTo->hide();
}


void
GNEViewNetHelper::DataViewOptions::getVisibleDataMenuCommands(std::vector<MFXCheckableButton*>& commands) const {
    // save visible menu commands in commands vector
    if (menuCheckToggleDrawJunctionShape->shown()) {
        commands.push_back(menuCheckToggleDrawJunctionShape);
    }
    if (menuCheckShowAdditionals->shown()) {
        commands.push_back(menuCheckShowAdditionals);
    }
    if (menuCheckShowShapes->shown()) {
        commands.push_back(menuCheckShowShapes);
    }
    if (menuCheckShowDemandElements->shown()) {
        commands.push_back(menuCheckShowDemandElements);
    }
    if (menuCheckToggleTAZRelDrawing->shown()) {
        commands.push_back(menuCheckToggleTAZRelDrawing);
    }
    if (menuCheckToggleTAZDrawFill->shown()) {
        commands.push_back(menuCheckToggleTAZDrawFill);
    }
    if (menuCheckToggleTAZRelOnlyFrom->shown()) {
        commands.push_back(menuCheckToggleTAZRelOnlyFrom);
    }
    if (menuCheckToggleTAZRelOnlyTo->shown()) {
        commands.push_back(menuCheckToggleTAZRelOnlyTo);
    }
}


bool
GNEViewNetHelper::DataViewOptions::showAdditionals() const {
    if (menuCheckShowAdditionals->shown()) {
        return (menuCheckShowAdditionals->amChecked() == TRUE);
    } else {
        return true;
    }
}


bool
GNEViewNetHelper::DataViewOptions::showShapes() const {
    if (menuCheckShowShapes->shown()) {
        return (menuCheckShowShapes->amChecked() == TRUE);
    } else {
        return true;
    }
}


bool
GNEViewNetHelper::DataViewOptions::showDemandElements() const {
    if (menuCheckShowDemandElements->shown()) {
        return (menuCheckShowDemandElements->amChecked() == TRUE);
    } else {
        return true;
    }
}


bool
GNEViewNetHelper::DataViewOptions::TAZRelDrawing() const {
    return (menuCheckToggleTAZRelDrawing->amChecked() != TRUE);
}


bool
GNEViewNetHelper::DataViewOptions::TAZDrawFill() const {
    if (menuCheckToggleTAZDrawFill->shown()) {
        return (menuCheckToggleTAZDrawFill->amChecked() != TRUE);
    } else {
        return false;
    }
}


bool
GNEViewNetHelper::DataViewOptions::TAZRelOnlyFrom() const {
    if (menuCheckToggleTAZRelOnlyFrom->shown()) {
        return (menuCheckToggleTAZRelOnlyFrom->amChecked() != TRUE);
    } else {
        return false;
    }
}


bool
GNEViewNetHelper::DataViewOptions::TAZRelOnlyTo() const {
    if (menuCheckToggleTAZRelOnlyTo->shown()) {
        return (menuCheckToggleTAZRelOnlyTo->amChecked() != TRUE);
    } else {
        return false;
    }
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::IntervalBar - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::IntervalBar::IntervalBar(GNEViewNet* viewNet) :
    myViewNet(viewNet) {
}


void
GNEViewNetHelper::IntervalBar::buildIntervalBarElements() {
    // create interval label
    FXLabel* genericDataLabel = new FXLabel(myViewNet->myViewParent->getGNEAppWindows()->getToolbarsGrip().intervalBar,
                                            "Data type", 0, GUIDesignLabelThickedFixed(100));
    genericDataLabel->create();
    // create combo box for generic datas
    myGenericDataTypesComboBox = new MFXComboBoxIcon(myViewNet->myViewParent->getGNEAppWindows()->getToolbarsGrip().intervalBar,
            GUIDesignComboBoxNCol, false, GUIDesignComboBoxVisibleItemsFixed, myViewNet,
            MID_GNE_INTERVALBAR_GENERICDATATYPE, GUIDesignComboBoxWidth180);
    myGenericDataTypesComboBox->create();
    // fill combo box
    myGenericDataTypesComboBox->appendIconItem("<all>");
    myGenericDataTypesComboBox->appendIconItem(toString(GNE_TAG_EDGEREL_SINGLE).c_str());
    myGenericDataTypesComboBox->appendIconItem(toString(SUMO_TAG_EDGEREL).c_str());
    myGenericDataTypesComboBox->appendIconItem(toString(SUMO_TAG_TAZREL).c_str());
    // create dataSet label
    FXLabel* dataSetLabel = new FXLabel(myViewNet->myViewParent->getGNEAppWindows()->getToolbarsGrip().intervalBar,
                                        "Data sets", 0, GUIDesignLabelThickedFixed(100));
    dataSetLabel->create();
    // create combo box for sets
    myDataSetsComboBox = new MFXComboBoxIcon(myViewNet->myViewParent->getGNEAppWindows()->getToolbarsGrip().intervalBar,
            GUIDesignComboBoxNCol, false, GUIDesignComboBoxVisibleItemsFixed,
            myViewNet, MID_GNE_INTERVALBAR_DATASET, GUIDesignComboBoxWidth180);
    myDataSetsComboBox->create();
    // create checkbutton for myLimitByInterval
    myIntervalCheckBox = new FXCheckButton(myViewNet->myViewParent->getGNEAppWindows()->getToolbarsGrip().intervalBar,
                                           "Interval", myViewNet, MID_GNE_INTERVALBAR_LIMITED, GUIDesignCheckButtonAttribute);
    myIntervalCheckBox->create();
    // create textfield for begin
    myBeginTextField = new FXTextField(myViewNet->myViewParent->getGNEAppWindows()->getToolbarsGrip().intervalBar,
                                       GUIDesignTextFieldNCol, myViewNet, MID_GNE_INTERVALBAR_BEGIN, GUIDesignTextFieldFixedRestricted(50, TEXTFIELD_REAL));
    myBeginTextField->create();
    // create text field for end
    myEndTextField = new FXTextField(myViewNet->myViewParent->getGNEAppWindows()->getToolbarsGrip().intervalBar,
                                     GUIDesignTextFieldNCol, myViewNet, MID_GNE_INTERVALBAR_END, GUIDesignTextFieldFixedRestricted(50, TEXTFIELD_REAL));
    myEndTextField->create();
    // create parameter label
    FXLabel* parameterLabel = new FXLabel(myViewNet->myViewParent->getGNEAppWindows()->getToolbarsGrip().intervalBar,
                                          "Parameter", 0, GUIDesignLabelThickedFixed(100));
    parameterLabel->create();
    // create combo box for attributes
    myParametersComboBox = new MFXComboBoxIcon(myViewNet->myViewParent->getGNEAppWindows()->getToolbarsGrip().intervalBar,
            GUIDesignComboBoxNCol, false, GUIDesignComboBoxVisibleItemsFixed,
            myViewNet, MID_GNE_INTERVALBAR_PARAMETER, GUIDesignComboBoxWidth180);
    myParametersComboBox->create();
    // always recalc after creating new elements
    myViewNet->myViewParent->getGNEAppWindows()->getToolbarsGrip().intervalBar->recalc();
}


void
GNEViewNetHelper::IntervalBar::showIntervalBar() {
    // first update interval bar
    updateIntervalBar();
    // show toolbar grip
    myViewNet->myViewParent->getGNEAppWindows()->getToolbarsGrip().intervalBar->show();
}


void
GNEViewNetHelper::IntervalBar::hideIntervalBar() {
    // hide toolbar grip
    myViewNet->myViewParent->getGNEAppWindows()->getToolbarsGrip().intervalBar->hide();
}


void
GNEViewNetHelper::IntervalBar::updateIntervalBar() {
    // check if intervalBar has to be updated
    if (myUpdateInterval && myViewNet->getNet()) {
        // clear data sets
        myDataSets.clear();
        // declare intervals
        double begin = INVALID_DOUBLE;
        double end = INVALID_DOUBLE;
        // clear parameters
        myParameters.clear();
        // iterate over all data elements
        for (const auto& dataSet : myViewNet->getNet()->getAttributeCarriers()->getDataSets()) {
            // add data set ID
            myDataSets.push_back(dataSet.second->getID());
            // iterate over all intervals
            for (const auto& interval : dataSet.second->getDataIntervalChildren()) {
                // set intervals
                if ((begin == INVALID_DOUBLE) || (interval.first < begin)) {
                    begin = interval.first;
                }
                if ((end == INVALID_DOUBLE) || (interval.first > end)) {
                    end = interval.first;
                }
                // iterate over all generic datas
                for (const auto& genericData : interval.second->getGenericDataChildren()) {
                    // iterate over parameters
                    for (const auto& parameter : genericData->getParametersMap()) {
                        myParameters.insert(parameter.first);
                    }
                }
            }
        }
        // get previous dataSet
        const std::string previousDataSet = myDataSetsComboBox->getText().text();
        // get previous interval
        const std::string previousBegin = (myIntervalCheckBox->getCheck() == TRUE) ? myBeginTextField->getText().text() : "";
        const std::string previousEnd = (myIntervalCheckBox->getCheck() == TRUE) ? myEndTextField->getText().text() : "";
        // get previous parameter
        const std::string previousParameter = myParametersComboBox->getText().text();
        // clear comboBoxes
        myDataSetsComboBox->clearItems();
        myParametersComboBox->clearItems();
        // add first item (all)
        myDataSetsComboBox->appendIconItem("<all>");
        myParametersComboBox->appendIconItem("<all>");
        // fill dataSet comboBox
        for (const auto& dataSet : myDataSets) {
            myDataSetsComboBox->appendIconItem(dataSet.c_str());
        }
        // set begin/end
        myBeginTextField->setText(toString(begin).c_str());
        myEndTextField->setText(toString(end).c_str());
        // fill parameter comboBox
        for (const auto& parameter : myParameters) {
            myParametersComboBox->appendIconItem(parameter.c_str());
        }
        // check previous dataSet
        myDataSetsComboBox->setCurrentItem(0, FALSE);
        for (int i = 0; i < myDataSetsComboBox->getNumItems(); i++) {
            if (myDataSetsComboBox->getItemText(i) == previousDataSet) {
                myDataSetsComboBox->setCurrentItem(i);
            }
        }
        // set previous interval
        if (myIntervalCheckBox->getCheck() == TRUE) {
            myBeginTextField->setText(previousBegin.c_str());
            myEndTextField->setText(previousEnd.c_str());
        }
        // check previous parameter
        myParametersComboBox->setCurrentItem(0, FALSE);
        for (int i = 0; i < myParametersComboBox->getNumItems(); i++) {
            if (myParametersComboBox->getItemText(i) == previousParameter) {
                myParametersComboBox->setCurrentItem(i);
            }
        }
        // check if enable or disable
        if ((myViewNet->getEditModes().dataEditMode == DataEditMode::DATA_INSPECT) ||
                (myViewNet->getEditModes().dataEditMode == DataEditMode::DATA_SELECT) ||
                (myViewNet->getEditModes().dataEditMode == DataEditMode::DATA_DELETE)) {
            enableIntervalBar();
        } else {
            disableIntervalBar();
        }
        // intervalBar updated, then change flag
        myUpdateInterval = false;
    }
}


void
GNEViewNetHelper::IntervalBar::markForUpdate() {
    myUpdateInterval = true;
}


SumoXMLTag
GNEViewNetHelper::IntervalBar::getGenericDataType() const {
    if (myGenericDataTypesComboBox->isEnabled() && (myGenericDataTypesComboBox->getTextColor() == FXRGB(0, 0, 0))) {
        if (myGenericDataTypesComboBox->getText() == toString(GNE_TAG_EDGEREL_SINGLE).c_str()) {
            return GNE_TAG_EDGEREL_SINGLE;
        } else if (myGenericDataTypesComboBox->getText() == toString(SUMO_TAG_EDGEREL).c_str()) {
            return SUMO_TAG_EDGEREL;
        } else if (myGenericDataTypesComboBox->getText() == toString(SUMO_TAG_TAZREL).c_str()) {
            return SUMO_TAG_TAZREL;
        }
    }
    return SUMO_TAG_NOTHING;
}


GNEDataSet*
GNEViewNetHelper::IntervalBar::getDataSet() const {
    if (!myDataSetsComboBox->isEnabled() ||
            (myDataSetsComboBox->getCurrentItem() == 0) ||
            (myDataSetsComboBox->getTextColor() == FXRGB(255, 0, 0))) {
        return nullptr;
    } else {
        return myViewNet->getNet()->getAttributeCarriers()->retrieveDataSet(myDataSetsComboBox->getText().text());
    }
}


double
GNEViewNetHelper::IntervalBar::getBegin() const {
    if (!myIntervalCheckBox->isEnabled() || (myIntervalCheckBox->getCheck() == FALSE)) {
        return INVALID_DOUBLE;
    } else {
        return GNEAttributeCarrier::parse<double>(myBeginTextField->getText().text());
    }
}


double
GNEViewNetHelper::IntervalBar::getEnd() const {
    if (!myIntervalCheckBox->isEnabled() || (myIntervalCheckBox->getCheck() == FALSE)) {
        return INVALID_DOUBLE;
    } else {
        return GNEAttributeCarrier::parse<double>(myEndTextField->getText().text());
    }
}


std::string
GNEViewNetHelper::IntervalBar::getParameter() const {
    if (!myParametersComboBox->isEnabled() ||
            (myParametersComboBox->getCurrentItem() == 0) ||
            (myParametersComboBox->getTextColor() == FXRGB(255, 0, 0))) {
        return "";
    } else {
        return myParametersComboBox->getText().text();
    }
}


void
GNEViewNetHelper::IntervalBar::setGenericDataType() {
    if ((myGenericDataTypesComboBox->getText() == "<all>") ||
            (myGenericDataTypesComboBox->getText() == toString(GNE_TAG_EDGEREL_SINGLE).c_str()) ||
            (myGenericDataTypesComboBox->getText() == toString(SUMO_TAG_EDGEREL).c_str()) ||
            (myGenericDataTypesComboBox->getText() == toString(SUMO_TAG_TAZREL).c_str())) {
        myGenericDataTypesComboBox->setTextColor(FXRGB(0, 0, 0));
    } else {
        myGenericDataTypesComboBox->setTextColor(FXRGB(255, 0, 0));
    }
    // update view net
    myViewNet->updateViewNet();
}


void
GNEViewNetHelper::IntervalBar::setDataSet() {
    // check if exist
    if (std::find(myDataSets.begin(), myDataSets.end(), myDataSetsComboBox->getText().text()) != myDataSets.end()) {
        myDataSetsComboBox->setTextColor(FXRGB(0, 0, 0));
    } else {
        myDataSetsComboBox->setTextColor(FXRGB(255, 0, 0));
    }
    // update view net
    myViewNet->updateViewNet();
}


void
GNEViewNetHelper::IntervalBar::setInterval() {
    // enable or disable text fields
    if (myIntervalCheckBox->isEnabled() && (myIntervalCheckBox->getCheck() == TRUE)) {
        myBeginTextField->enable();
        myEndTextField->enable();
    } else {
        myBeginTextField->disable();
        myEndTextField->disable();
    }
    // update view net
    myViewNet->updateViewNet();
}


void
GNEViewNetHelper::IntervalBar::setBegin() {
    if (myBeginTextField->getText().empty()) {
        myBeginTextField->setText(toString(myViewNet->getNet()->getDataSetIntervalMinimumBegin()).c_str());
        myBeginTextField->setTextColor(FXRGB(0, 0, 0));
    } else if (GNEAttributeCarrier::canParse<double>(myBeginTextField->getText().text())) {
        myBeginTextField->setTextColor(FXRGB(0, 0, 0));
    } else {
        myBeginTextField->setTextColor(FXRGB(255, 0, 0));
    }
    // update view net
    myViewNet->updateViewNet();
}


void
GNEViewNetHelper::IntervalBar::setEnd() {
    if (myEndTextField->getText().empty()) {
        myEndTextField->setText(toString(myViewNet->getNet()->getDataSetIntervalMaximumEnd()).c_str());
        myEndTextField->setTextColor(FXRGB(0, 0, 0));
    } else if (GNEAttributeCarrier::canParse<double>(myEndTextField->getText().text())) {
        myEndTextField->setTextColor(FXRGB(0, 0, 0));
    } else {
        myEndTextField->setTextColor(FXRGB(255, 0, 0));
    }
    // update view net
    myViewNet->updateViewNet();
}


void
GNEViewNetHelper::IntervalBar::setParameter() {
    // check if exist
    if (myParameters.count(myParametersComboBox->getText().text()) > 0) {
        myParametersComboBox->setTextColor(FXRGB(0, 0, 0));
    } else {
        myParametersComboBox->setTextColor(FXRGB(255, 0, 0));
    }
    // update view net
    myViewNet->updateViewNet();
}



void
GNEViewNetHelper::IntervalBar::enableIntervalBar() {
    // enable elements
    myGenericDataTypesComboBox->enable();
    myDataSetsComboBox->enable();
    myIntervalCheckBox->enable();
    if (myIntervalCheckBox->getCheck() == TRUE) {
        myBeginTextField->enable();
        myEndTextField->enable();
    } else {
        myBeginTextField->disable();
        myEndTextField->disable();
    }
    myParametersComboBox->enable();
}


void
GNEViewNetHelper::IntervalBar::disableIntervalBar() {
    // disable all elements
    myGenericDataTypesComboBox->disable();
    myDataSetsComboBox->disable();
    myIntervalCheckBox->disable();
    myBeginTextField->disable();
    myEndTextField->disable();
    myParametersComboBox->disable();
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::CommonCheckableButtons - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::CommonCheckableButtons::CommonCheckableButtons(GNEViewNet* viewNet) :
    myViewNet(viewNet) {
}


void
GNEViewNetHelper::CommonCheckableButtons::buildCommonCheckableButtons() {
    // get grip modes
    auto gripModes = myViewNet->getViewParent()->getGNEAppWindows()->getToolbarsGrip().modes;
    // get tooltip menu
    auto tooltipMenu = myViewNet->myViewParent->getGNEAppWindows()->getStaticTooltipMenu();
    // inspect button
    inspectButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
                                           std::string("\t") + TL("Inspect mode") + std::string("\t") + TL("Mode for inspect elements and change their attributes. (I)"),
                                           GUIIconSubSys::getIcon(GUIIcon::MODEINSPECT), myViewNet, MID_HOTKEY_I_MODE_INSPECT, GUIDesignMFXCheckableButtonSquare);
    inspectButton->create();
    // delete button
    deleteButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
                                          std::string("\t") + TL("Delete mode") + std::string("\t") + TL("Mode for deleting elements. (D)"),
                                          GUIIconSubSys::getIcon(GUIIcon::MODEDELETE), myViewNet, MID_HOTKEY_D_MODE_SINGLESIMULATIONSTEP_DELETE, GUIDesignMFXCheckableButtonSquare);
    deleteButton->create();
    // select button
    selectButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
                                          std::string("\t") + TL("Select mode") + std::string("\t") + TL("Mode for selecting elements. (S)"),
                                          GUIIconSubSys::getIcon(GUIIcon::MODESELECT), myViewNet, MID_HOTKEY_S_MODE_STOPSIMULATION_SELECT, GUIDesignMFXCheckableButtonSquare);
    selectButton->create();
    // always recalc menu bar after creating new elements
    gripModes->recalc();
}


void
GNEViewNetHelper::CommonCheckableButtons::showCommonCheckableButtons() {
    inspectButton->show();
    deleteButton->show();
    selectButton->show();
}


void
GNEViewNetHelper::CommonCheckableButtons::hideCommonCheckableButtons() {
    inspectButton->hide();
    deleteButton->hide();
    selectButton->hide();
}


void
GNEViewNetHelper::CommonCheckableButtons::disableCommonCheckableButtons() {
    inspectButton->setChecked(false);
    deleteButton->setChecked(false);
    selectButton->setChecked(false);
}


void
GNEViewNetHelper::CommonCheckableButtons::updateCommonCheckableButtons() {
    inspectButton->update();
    deleteButton->update();
    selectButton->update();
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::NetworkCheckableButtons - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::NetworkCheckableButtons::NetworkCheckableButtons(GNEViewNet* viewNet) :
    myViewNet(viewNet) {
}


void
GNEViewNetHelper::NetworkCheckableButtons::buildNetworkCheckableButtons() {
    // get grip modes
    auto gripModes = myViewNet->getViewParent()->getGNEAppWindows()->getToolbarsGrip().modes;
    // get tooltip menu
    auto tooltipMenu = myViewNet->myViewParent->getGNEAppWindows()->getStaticTooltipMenu();
    // move button
    moveNetworkElementsButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
            std::string("\t") + TL("Move mode") + std::string("\t") + TL("Mode for moving elements. (M)"),
            GUIIconSubSys::getIcon(GUIIcon::MODEMOVE), myViewNet, MID_HOTKEY_M_MODE_MOVE_MEANDATA, GUIDesignMFXCheckableButtonSquare);
    moveNetworkElementsButton->create();
    // create edge
    createEdgeButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
            std::string("\t") + TL("Edge mode") + std::string("\t") + TL("Mode for creating junctions and edges. (E)"),
            GUIIconSubSys::getIcon(GUIIcon::MODECREATEEDGE), myViewNet, MID_HOTKEY_E_MODE_EDGE_EDGEDATA, GUIDesignMFXCheckableButtonSquare);
    createEdgeButton->create();
    // traffic light mode
    trafficLightButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
            std::string("\t") + TL("Traffic light mode") + std::string("\t") + TL("Mode for editing traffic lights over junctions. (T)"),
            GUIIconSubSys::getIcon(GUIIcon::MODETLS), myViewNet, MID_HOTKEY_T_MODE_TLS_TYPE, GUIDesignMFXCheckableButtonSquare);
    trafficLightButton->create();
    // connection mode
    connectionButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
            std::string("\t") + TL("Connection mode") + std::string("\t") + TL("Mode for edit connections between lanes. (C)"),
            GUIIconSubSys::getIcon(GUIIcon::MODECONNECTION), myViewNet, MID_HOTKEY_C_MODE_CONNECT_CONTAINER, GUIDesignMFXCheckableButtonSquare);
    connectionButton->create();
    // prohibition mode
    prohibitionButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
            std::string("\t") + TL("Prohibition mode") + std::string("\t") + TL("Mode for editing connection prohibitions. (H)"),
            GUIIconSubSys::getIcon(GUIIcon::MODEPROHIBITION), myViewNet, MID_HOTKEY_H_MODE_PROHIBITION_CONTAINERPLAN, GUIDesignMFXCheckableButtonSquare);
    prohibitionButton->create();
    // crossing mode
    crossingButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
                                            std::string("\t") + TL("Crossing mode") + std::string("\t") + TL("Mode for creating crossings between edges. (R)"),
                                            GUIIconSubSys::getIcon(GUIIcon::MODECROSSING), myViewNet, MID_HOTKEY_R_MODE_CROSSING_ROUTE_EDGERELDATA, GUIDesignMFXCheckableButtonSquare);
    crossingButton->create();
    // additional mode
    additionalButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
            std::string("\t") + TL("Additional mode") + std::string("\t") + TL("Mode for creating additional elements. (A)"),
            GUIIconSubSys::getIcon(GUIIcon::MODEADDITIONAL), myViewNet, MID_HOTKEY_A_MODE_STARTSIMULATION_ADDITIONALS_STOPS, GUIDesignMFXCheckableButtonSquare);
    additionalButton->create();
    // wire mode
    wireButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
                                        std::string("\t") + TL("Wire mode") + std::string("\t") + TL("Mode for editing overhead wires. (W)"),
                                        GUIIconSubSys::getIcon(GUIIcon::MODEWIRE), myViewNet, MID_HOTKEY_W_MODE_WIRE_ROUTEDISTRIBUTION, GUIDesignMFXCheckableButtonSquare);
    wireButton->create();
    // TAZ Mode
    TAZButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
                                       std::string("\t") + TL("TAZ mode") + std::string("\t") + TL("Mode for creating Traffic Assignment Zones. (Z)"),
                                       GUIIconSubSys::getIcon(GUIIcon::MODETAZ), myViewNet, MID_HOTKEY_Z_MODE_TAZ_TAZREL, GUIDesignMFXCheckableButtonSquare);
    TAZButton->create();
    // shape mode
    shapeButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
                                         std::string("\t") + TL("Polygon mode") + std::string("\t") + TL("Mode for creating polygons and POIs. (P)"),
                                         GUIIconSubSys::getIcon(GUIIcon::MODESHAPE), myViewNet, MID_HOTKEY_P_MODE_POLYGON_PERSON, GUIDesignMFXCheckableButtonSquare);
    shapeButton->create();
    // decal mode
    decalButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
                                         std::string("\t") + TL("Decal mode") + std::string("\t") + TL("Mode for editing decals. (U)"),
                                         GUIIconSubSys::getIcon(GUIIcon::MODEDECAL), myViewNet, MID_HOTKEY_U_MODE_DECAL_TYPEDISTRIBUTION, GUIDesignMFXCheckableButtonSquare);
    decalButton->create();
    // always recalc after creating new elements
    gripModes->recalc();
}


void
GNEViewNetHelper::NetworkCheckableButtons::showNetworkCheckableButtons() {
    // continue depending of view
    if (myViewNet->getEditModes().isJuPedSimView()) {
        moveNetworkElementsButton->show();
        createEdgeButton->show();
        connectionButton->hide();
        trafficLightButton->hide();
        additionalButton->show();
        crossingButton->show();
        TAZButton->show();
        shapeButton->show();
        prohibitionButton->hide();
        wireButton->hide();
        decalButton->hide();
    } else {
        moveNetworkElementsButton->show();
        createEdgeButton->show();
        connectionButton->show();
        trafficLightButton->show();
        additionalButton->show();
        crossingButton->show();
        TAZButton->show();
        shapeButton->show();
        prohibitionButton->show();
        wireButton->show();
        decalButton->show();
    }
}


void
GNEViewNetHelper::NetworkCheckableButtons::hideNetworkCheckableButtons() {
    moveNetworkElementsButton->hide();
    createEdgeButton->hide();
    connectionButton->hide();
    trafficLightButton->hide();
    additionalButton->hide();
    crossingButton->hide();
    TAZButton->hide();
    shapeButton->hide();
    prohibitionButton->hide();
    wireButton->hide();
    decalButton->hide();
}


void
GNEViewNetHelper::NetworkCheckableButtons::disableNetworkCheckableButtons() {
    moveNetworkElementsButton->setChecked(false);
    createEdgeButton->setChecked(false);
    connectionButton->setChecked(false);
    trafficLightButton->setChecked(false);
    additionalButton->setChecked(false);
    crossingButton->setChecked(false);
    TAZButton->setChecked(false);
    shapeButton->setChecked(false);
    prohibitionButton->setChecked(false);
    wireButton->setChecked(false);
    decalButton->setChecked(false);
}


void
GNEViewNetHelper::NetworkCheckableButtons::updateNetworkCheckableButtons() {
    moveNetworkElementsButton->update();
    createEdgeButton->update();
    connectionButton->update();
    trafficLightButton->update();
    additionalButton->update();
    crossingButton->update();
    TAZButton->update();
    shapeButton->update();
    prohibitionButton->update();
    wireButton->update();
    decalButton->update();
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::DemandCheckableButtons - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::DemandCheckableButtons::DemandCheckableButtons(GNEViewNet* viewNet) :
    myViewNet(viewNet) {
}


void
GNEViewNetHelper::DemandCheckableButtons::buildDemandCheckableButtons() {
    // get grip modes
    auto gripModes = myViewNet->getViewParent()->getGNEAppWindows()->getToolbarsGrip().modes;
    // get tooltip menu
    auto tooltipMenu = myViewNet->myViewParent->getGNEAppWindows()->getStaticTooltipMenu();
    // move button
    moveDemandElementsButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
            std::string("\t") + TL("Move mode") + std::string("\t") + TL("Mode for moving elements. (M)"),
            GUIIconSubSys::getIcon(GUIIcon::MODEMOVE), myViewNet, MID_HOTKEY_M_MODE_MOVE_MEANDATA, GUIDesignMFXCheckableButtonSquare);
    moveDemandElementsButton->create();
    // route mode
    routeButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
                                         std::string("\t") + TL("Route mode") + std::string("\t") + TL("Mode for creating routes. (R)"),
                                         GUIIconSubSys::getIcon(GUIIcon::MODEROUTE), myViewNet, MID_HOTKEY_R_MODE_CROSSING_ROUTE_EDGERELDATA, GUIDesignMFXCheckableButtonSquare);
    routeButton->create();
    // rout distribution mode
    routeDistributionButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
            std::string("\t") + TL("Route distribution mode") + std::string("\t") + TL("Mode for creating and editing rout distributions. (W)"),
            GUIIconSubSys::getIcon(GUIIcon::MODEROUTEDISTRIBUTION), myViewNet, MID_HOTKEY_W_MODE_WIRE_ROUTEDISTRIBUTION, GUIDesignMFXCheckableButtonSquare);
    routeDistributionButton->create();
    // vehicle mode
    vehicleButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
                                           std::string("\t") + TL("Vehicle mode") + std::string("\t") + TL("Mode for creating vehicles. (V)"),
                                           GUIIconSubSys::getIcon(GUIIcon::MODEVEHICLE), myViewNet, MID_HOTKEY_V_MODE_VEHICLE, GUIDesignMFXCheckableButtonSquare);
    vehicleButton->create();
    // type mode
    typeButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
                                        std::string("\t") + TL("Type mode") + std::string("\t") + TL("Mode for creating types (of vehicles, persons and containers). (T)"),
                                        GUIIconSubSys::getIcon(GUIIcon::MODETYPE), myViewNet, MID_HOTKEY_T_MODE_TLS_TYPE, GUIDesignMFXCheckableButtonSquare);
    typeButton->create();
    // type distribution mode
    typeDistributionButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
            std::string("\t") + TL("Type distribution mode") + std::string("\t") + TL("Mode for creating and editing type distribution. (U)"),
            GUIIconSubSys::getIcon(GUIIcon::MODETYPEDISTRIBUTION), myViewNet, MID_HOTKEY_U_MODE_DECAL_TYPEDISTRIBUTION, GUIDesignMFXCheckableButtonSquare);
    typeDistributionButton->create();
    // stop mode
    stopButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
                                        std::string("\t") + TL("Stop mode") + std::string("\t") + TL("Mode for creating stops. (A)"),
                                        GUIIconSubSys::getIcon(GUIIcon::MODESTOP), myViewNet, MID_HOTKEY_A_MODE_STARTSIMULATION_ADDITIONALS_STOPS, GUIDesignMFXCheckableButtonSquare);
    stopButton->create();
    // person mode
    personButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
                                          std::string("\t") + TL("Person mode") + std::string("\t") + TL("Mode for creating persons. (P)"),
                                          GUIIconSubSys::getIcon(GUIIcon::MODEPERSON), myViewNet, MID_HOTKEY_P_MODE_POLYGON_PERSON, GUIDesignMFXCheckableButtonSquare);
    personButton->create();
    // person plan mode
    personPlanButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
            std::string("\t") + TL("Person plan mode") + std::string("\t") + TL("Mode for creating person plans. (L)"),
            GUIIconSubSys::getIcon(GUIIcon::MODEPERSONPLAN), myViewNet, MID_HOTKEY_L_MODE_PERSONPLAN, GUIDesignMFXCheckableButtonSquare);
    personPlanButton->create();
    // container mode
    containerButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
            std::string("\t") + TL("Container mode") + std::string("\t") + TL("Mode for creating containers. (C)"),
            GUIIconSubSys::getIcon(GUIIcon::MODECONTAINER), myViewNet, MID_HOTKEY_C_MODE_CONNECT_CONTAINER, GUIDesignMFXCheckableButtonSquare);
    containerButton->create();
    // container plan mode
    containerPlanButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
            std::string("\t") + TL("Container plan mode") + std::string("\t") + TL("Mode for creating container plans. (H)"),
            GUIIconSubSys::getIcon(GUIIcon::MODECONTAINERPLAN), myViewNet, MID_HOTKEY_H_MODE_PROHIBITION_CONTAINERPLAN, GUIDesignMFXCheckableButtonSquare);
    containerPlanButton->create();
    // always recalc after creating new elements
    gripModes->recalc();
}


void
GNEViewNetHelper::DemandCheckableButtons::showDemandCheckableButtons() {
    if (myViewNet->getEditModes().isJuPedSimView()) {
        moveDemandElementsButton->hide();
        routeButton->show();
        routeDistributionButton->show();
        vehicleButton->hide();
        typeButton->show();
        typeDistributionButton->show();
        stopButton->hide();
        personButton->show();
        personPlanButton->show();
        containerButton->hide();
        containerPlanButton->hide();
    } else {
        moveDemandElementsButton->show();
        routeButton->show();
        routeDistributionButton->show();
        vehicleButton->show();
        typeButton->show();
        typeDistributionButton->show();
        stopButton->show();
        personButton->show();
        personPlanButton->show();
        containerButton->show();
        containerPlanButton->show();
    }
}


void
GNEViewNetHelper::DemandCheckableButtons::hideDemandCheckableButtons() {
    moveDemandElementsButton->hide();
    routeButton->hide();
    routeDistributionButton->hide();
    vehicleButton->hide();
    typeButton->hide();
    typeDistributionButton->hide();
    stopButton->hide();
    personButton->hide();
    personPlanButton->hide();
    containerButton->hide();
    containerPlanButton->hide();
}


void
GNEViewNetHelper::DemandCheckableButtons::disableDemandCheckableButtons() {
    moveDemandElementsButton->setChecked(false);
    routeButton->setChecked(false);
    routeDistributionButton->setChecked(false);
    vehicleButton->setChecked(false);
    typeButton->setChecked(false);
    typeDistributionButton->setChecked(false);
    stopButton->setChecked(false);
    personButton->setChecked(false);
    personPlanButton->setChecked(false);
    containerButton->setChecked(false);
    containerPlanButton->setChecked(false);
}


void
GNEViewNetHelper::DemandCheckableButtons::updateDemandCheckableButtons() {
    moveDemandElementsButton->update();
    routeButton->update();
    routeDistributionButton->update();
    vehicleButton->update();
    typeButton->update();
    typeDistributionButton->update();
    stopButton->update();
    personButton->update();
    personPlanButton->update();
    containerButton->update();
    containerPlanButton->update();
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::DataCheckableButtons - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::DataCheckableButtons::DataCheckableButtons(GNEViewNet* viewNet) :
    myViewNet(viewNet) {
}


void
GNEViewNetHelper::DataCheckableButtons::buildDataCheckableButtons() {
    // get grip modes
    auto gripModes = myViewNet->getViewParent()->getGNEAppWindows()->getToolbarsGrip().modes;
    // get tooltip menu
    auto tooltipMenu = myViewNet->myViewParent->getGNEAppWindows()->getStaticTooltipMenu();
    // edgeData mode
    edgeDataButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
                                            std::string("\t") + TL("Edge data mode") + std::string("\t") + TL("Mode for creating edge datas. (E)"),
                                            GUIIconSubSys::getIcon(GUIIcon::MODEEDGEDATA), myViewNet, MID_HOTKEY_E_MODE_EDGE_EDGEDATA, GUIDesignMFXCheckableButtonSquare);
    edgeDataButton->create();
    // edgeRelData mode
    edgeRelDataButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
            std::string("\t") + TL("Edge relation data mode") + std::string("\t") + TL("Mode for creating edge relation datas. (R)"),
            GUIIconSubSys::getIcon(GUIIcon::MODEEDGERELDATA), myViewNet, MID_HOTKEY_R_MODE_CROSSING_ROUTE_EDGERELDATA, GUIDesignMFXCheckableButtonSquare);
    edgeRelDataButton->create();
    // TAZRelData mode
    TAZRelDataButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
            std::string("\t") + TL("TAZ relation data mode") + std::string("\t") + TL("Mode for creating TAZ relation datas. (Z)"),
            GUIIconSubSys::getIcon(GUIIcon::MODETAZRELDATA), myViewNet, MID_HOTKEY_Z_MODE_TAZ_TAZREL, GUIDesignMFXCheckableButtonSquare);
    TAZRelDataButton->create();
    // meanData button
    meanDataButton = new MFXCheckableButton(false, gripModes, tooltipMenu,
                                            std::string("\t") + TL("MeanData mode") + std::string("\t") + TL("Mode for MeanData elements. (M)"),
                                            GUIIconSubSys::getIcon(GUIIcon::MODEMEANDATA), myViewNet, MID_HOTKEY_M_MODE_MOVE_MEANDATA, GUIDesignMFXCheckableButtonSquare);
    meanDataButton->create();
    // always recalc after creating new elements
    gripModes->recalc();
}


void
GNEViewNetHelper::DataCheckableButtons::showDataCheckableButtons() {
    edgeDataButton->show();
    edgeRelDataButton->show();
    TAZRelDataButton->show();
    meanDataButton->show();
}


void
GNEViewNetHelper::DataCheckableButtons::hideDataCheckableButtons() {
    edgeDataButton->hide();
    edgeRelDataButton->hide();
    TAZRelDataButton->hide();
    meanDataButton->hide();
}


void
GNEViewNetHelper::DataCheckableButtons::disableDataCheckableButtons() {
    edgeDataButton->setChecked(false);
    edgeRelDataButton->setChecked(false);
    TAZRelDataButton->setChecked(false);
    meanDataButton->setChecked(false);
}


void
GNEViewNetHelper::DataCheckableButtons::updateDataCheckableButtons() {
    edgeDataButton->update();
    edgeRelDataButton->update();
    TAZRelDataButton->update();
    meanDataButton->update();
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::EditNetworkElementShapes - methods
// ---------------------------------------------------------------------------

GNEViewNetHelper::EditNetworkElementShapes::EditNetworkElementShapes(GNEViewNet* viewNet) :
    myViewNet(viewNet),
    myPreviousNetworkEditMode(NetworkEditMode::NETWORK_NONE) {
}


void
GNEViewNetHelper::EditNetworkElementShapes::startEditCustomShape(GNENetworkElement* element) {
    if (element && (myEditedNetworkElement == nullptr)) {
        // save current edit mode before starting
        myPreviousNetworkEditMode = myViewNet->myEditModes.networkEditMode;
        // set move mode
        myViewNet->myEditModes.setNetworkEditMode(NetworkEditMode::NETWORK_MOVE);
        //set editedNetworkElement
        myEditedNetworkElement = element;
        // enable shape edited flag
        myEditedNetworkElement->setShapeEdited(true);
        // update view net to show the new editedShapePoly
        myViewNet->updateViewNet();
    }
}


void
GNEViewNetHelper::EditNetworkElementShapes::stopEditCustomShape() {
    // stop edit shape junction deleting editedShapePoly
    if (myEditedNetworkElement != nullptr) {
        // disable shape edited flag
        myEditedNetworkElement->setShapeEdited(false);
        // reset editedNetworkElement
        myEditedNetworkElement = nullptr;
        // restore previous edit mode
        if (myViewNet->myEditModes.networkEditMode != myPreviousNetworkEditMode) {
            myViewNet->myEditModes.setNetworkEditMode(myPreviousNetworkEditMode);
        }
    }
}


void
GNEViewNetHelper::EditNetworkElementShapes::commitShapeEdited() {
    // save edited junction's shape
    if (myEditedNetworkElement != nullptr) {
        // stop edit custom shape
        stopEditCustomShape();
    }
}


GNENetworkElement*
GNEViewNetHelper::EditNetworkElementShapes::getEditedNetworkElement() const {
    return myEditedNetworkElement;
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper::BlockIcon - methods
// ---------------------------------------------------------------------------

void
GNEViewNetHelper::LockIcon::drawLockIcon(const GUIVisualizationSettings::Detail d,
        const GNEAttributeCarrier* AC, GUIGlObjectType type, const Position position, const double exaggeration,
        const double size, const double offsetx, const double offsety) {
    // first check if icon can be drawn
    if (checkDrawing(d, AC, type, exaggeration)) {
        // Start pushing matrix
        GLHelper::pushMatrix();
        // Traslate to position
        glTranslated(position.x(), position.y(), GLO_LOCKICON);
        // Traslate depending of the offset
        glTranslated(offsetx, offsety, 0);
        // rotate to avoid draw invert
        glRotated(180, 0, 0, 1);
        // Set draw color
        glColor3d(1, 1, 1);
        // Draw lock icon
        GUITexturesHelper::drawTexturedBox(GUITextureSubSys::getTexture(GUITexture::LOCK), size);
        // Pop matrix
        GLHelper::popMatrix();
    }
}


GNEViewNetHelper::LockIcon::LockIcon() {}


bool
GNEViewNetHelper::LockIcon::checkDrawing(const GUIVisualizationSettings::Detail d, const GNEAttributeCarrier* AC,
        GUIGlObjectType type, const double exaggeration) {
    // check detail
    if (d > GUIVisualizationSettings::Detail::LockedIcons) {
        return false;
    }
    // get view net
    const auto viewNet = AC->getNet()->getViewNet();
    // check exaggeration
    if (exaggeration == 0) {
        return false;
    }
    // check supermodes
    if (viewNet->getEditModes().isCurrentSupermodeNetwork() &&
            !(AC->getTagProperty().isNetworkElement() || AC->getTagProperty().isAdditionalElement())) {
        return false;
    }
    if (viewNet->getEditModes().isCurrentSupermodeDemand() && (!AC->getTagProperty().isDemandElement())) {
        return false;
    }
    if (viewNet->getEditModes().isCurrentSupermodeData() && (!AC->getTagProperty().isDataElement())) {
        return false;
    }
    // check if is locked
    if (!viewNet->getLockManager().isObjectLocked(type, AC->isAttributeCarrierSelected())) {
        return false;
    }
    // all ok, then draw
    return true;
}

// ---------------------------------------------------------------------------
// GNEViewNetHelper - methods
// ---------------------------------------------------------------------------

const std::vector<RGBColor>&
GNEViewNetHelper::getRainbowScaledColors() {
    // if is empty, fill it
    if (myRainbowScaledColors.empty()) {
        // fill scale colors (10)
        myRainbowScaledColors.push_back(RGBColor(232, 35,  0,   255));
        myRainbowScaledColors.push_back(RGBColor(255, 165, 0,   255));
        myRainbowScaledColors.push_back(RGBColor(255, 255, 0,   255));
        myRainbowScaledColors.push_back(RGBColor(28,  215, 0,   255));
        myRainbowScaledColors.push_back(RGBColor(0,   181, 100, 255));
        myRainbowScaledColors.push_back(RGBColor(0,   255, 191, 255));
        myRainbowScaledColors.push_back(RGBColor(178, 255, 255, 255));
        myRainbowScaledColors.push_back(RGBColor(0,   112, 184, 255));
        myRainbowScaledColors.push_back(RGBColor(56,  41,  131, 255));
        myRainbowScaledColors.push_back(RGBColor(127, 0,   255, 255));
    }
    return myRainbowScaledColors;
}


const RGBColor&
GNEViewNetHelper::getRainbowScaledColor(const double min, const double max, const double value) {
    // check extremes
    if (value <= min) {
        return getRainbowScaledColors().front();
    } else if (value >= max) {
        return getRainbowScaledColors().back();
    } else {
        // calculate value procent between [min, max]
        const double procent = ((value - min) * 100) / (max - min);
        // check if is valid
        if (procent <= 0) {
            return getRainbowScaledColors().front();
        } else if (procent >= 100) {
            return getRainbowScaledColors().back();
        } else {
            // return scaled color
            return getRainbowScaledColors().at((int)(procent / 10.0));
        }
    }
}


std::vector<GUIGlObject*>
GNEViewNetHelper::filterElementsByLayer(const std::vector<GUIGlObject*>& GLObjects) {
    std::vector<GUIGlObject*> filteredGLObjects;
    if (GLObjects.size() > 0) {
        const auto firstLayer = GLObjects.front()->getType();
        for (const auto& GLObject : GLObjects) {
            if ((GLO_RIDE <= firstLayer) && (firstLayer <= GLO_TRANSHIP) &&
                    (GLO_RIDE <= GLObject->getType()) && (GLObject->getType() <= GLO_TRANSHIP)) {
                filteredGLObjects.push_back(GLObject);
            } else if ((GLO_STOP <= firstLayer) && (firstLayer <= GLO_STOP_PLAN) &&
                       (GLO_STOP <= GLObject->getType()) && (GLObject->getType() <= GLO_STOP_PLAN)) {
                filteredGLObjects.push_back(GLObject);
            } else if ((GLO_VEHICLE <= firstLayer) && (firstLayer <= GLO_ROUTEFLOW) &&
                       (GLO_VEHICLE <= GLObject->getType()) && (GLObject->getType() <= GLO_ROUTEFLOW)) {
                filteredGLObjects.push_back(GLObject);
            } else if ((GLO_PERSON <= firstLayer) && (firstLayer <= GLO_PERSONFLOW) &&
                       (GLO_PERSON <= GLObject->getType()) && (GLObject->getType() <= GLO_PERSONFLOW)) {
                filteredGLObjects.push_back(GLObject);
            } else if ((GLO_CONTAINER <= firstLayer) && (firstLayer <= GLO_CONTAINERFLOW) &&
                       (GLO_CONTAINER <= GLObject->getType()) && (GLObject->getType() <= GLO_CONTAINERFLOW)) {
                filteredGLObjects.push_back(GLObject);
            } else if (GLObject->getType() == firstLayer) {
                filteredGLObjects.push_back(GLObject);
            }
        }
    }
    return filteredGLObjects;
}

/****************************************************************************/
