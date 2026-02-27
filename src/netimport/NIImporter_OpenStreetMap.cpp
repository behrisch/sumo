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
/// @file    NIImporter_OpenStreetMap.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Walter Bamberger
/// @author  Gregor Laemmel
/// @author  Mirko Barthauer
/// @date    Mon, 14.04.2008
///
// Importer for networks stored in OpenStreetMap format
/****************************************************************************/
#include <config.h>
#include <algorithm>
#include <set>
#include <functional>
#include <sstream>
#include <limits>
#include <utils/common/UtilExceptions.h>
#include <utils/common/StringUtils.h>
#include <utils/common/ToString.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/StringUtils.h>
#include <utils/common/StringTokenizer.h>
#include <utils/common/FileHelpers.h>
#include <utils/geom/GeoConvHelper.h>
#include <utils/geom/GeomConvHelper.h>
#include <utils/options/OptionsCont.h>
#include <utils/xml/SUMOSAXHandler.h>
#include <utils/xml/SUMOSAXReader.h>
#include <utils/xml/SUMOXMLDefinitions.h>
#include <utils/xml/XMLSubSys.h>
#include <netbuild/NBEdge.h>
#include <netbuild/NBEdgeCont.h>
#include <netbuild/NBNode.h>
#include <netbuild/NBNodeCont.h>
#include <netbuild/NBNetBuilder.h>
#include <netbuild/NBOwnTLDef.h>
#include <netbuild/NBPTLine.h>
#include <netbuild/NBPTLineCont.h>
#include <netbuild/NBPTPlatform.h>
#include <netbuild/NBPTStop.h>
#include "NILoader.h"
#include "NIImporter_OpenStreetMap.h"

//#define DEBUG_LAYER_ELEVATION
//#define DEBUG_RAIL_DIRECTION

// ---------------------------------------------------------------------------
// static members
// ---------------------------------------------------------------------------
const double NIImporter_OpenStreetMap::MAXSPEED_UNGIVEN = -1;

const long long int NIImporter_OpenStreetMap::INVALID_ID = std::numeric_limits<long long int>::max();
bool NIImporter_OpenStreetMap::myAllAttributes(false);
std::set<std::string> NIImporter_OpenStreetMap::myExtraAttributes;

// ===========================================================================
// Private classes
// ===========================================================================

/** @brief Functor which compares two Edges
 */
class NIImporter_OpenStreetMap::CompareEdges {
public:
    bool operator()(const Edge* e1, const Edge* e2) const {
        if (e1->myHighWayType != e2->myHighWayType) {
            return e1->myHighWayType > e2->myHighWayType;
        }
        if (e1->myNoLanes != e2->myNoLanes) {
            return e1->myNoLanes > e2->myNoLanes;
        }
        if (e1->myNoLanesForward != e2->myNoLanesForward) {
            return e1->myNoLanesForward > e2->myNoLanesForward;
        }
        if (e1->myMaxSpeed != e2->myMaxSpeed) {
            return e1->myMaxSpeed > e2->myMaxSpeed;
        }
        if (e1->myIsOneWay != e2->myIsOneWay) {
            return e1->myIsOneWay > e2->myIsOneWay;
        }
        return e1->myCurrentNodes > e2->myCurrentNodes;
    }
};

// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// static methods
// ---------------------------------------------------------------------------
const std::string NIImporter_OpenStreetMap::compoundTypeSeparator("|"); //clang-tidy says: "compundTypeSeparator with
// static storage duration my throw an exception that cannot be caught

void
NIImporter_OpenStreetMap::loadNetwork(const OptionsCont& oc, NBNetBuilder& nb) {
    NIImporter_OpenStreetMap importer;
    importer.load(oc, nb);
}

NIImporter_OpenStreetMap::NIImporter_OpenStreetMap() = default;

NIImporter_OpenStreetMap::~NIImporter_OpenStreetMap() {
    // delete nodes
    for (auto myUniqueNode : myUniqueNodes) {
        delete myUniqueNode;
    }
    // delete edges
    for (auto& myEdge : myEdges) {
        delete myEdge.second;
    }
    // delete platform shapes
    for (auto& myPlatformShape : myPlatformShapes) {
        delete myPlatformShape.second;
    }
}

void
NIImporter_OpenStreetMap::load(const OptionsCont& oc, NBNetBuilder& nb) {
    if (!oc.isSet("osm-files")) {
        return;
    }
    const std::vector<std::string> files = oc.getStringVector("osm-files");
    std::vector<SUMOSAXReader*> readers;

    myImportLaneAccess = oc.getBool("osm.lane-access");
    myImportTurnSigns = oc.getBool("osm.turn-lanes");
    myImportSidewalks = oc.getBool("osm.sidewalks");
    myImportBikeAccess = oc.getBool("osm.bike-access");
    myImportCrossings = oc.getBool("osm.crossings");
    myOnewayDualSidewalk = oc.getBool("osm.oneway-reverse-sidewalk");
    myAnnotateDefaults = oc.getBool("osm.annotate-defaults");

    myAllAttributes = OptionsCont::getOptions().getBool("osm.all-attributes");
    std::vector<std::string> extra = OptionsCont::getOptions().getStringVector("osm.extra-attributes");
    myExtraAttributes.insert(extra.begin(), extra.end());
    if (myExtraAttributes.count("all") != 0) {
        // import all
        myExtraAttributes.clear();
    }

    // load nodes, first
    NodesHandler nodesHandler(myOSMNodes, myUniqueNodes, oc);
    for (const std::string& file : files) {
        if (!FileHelpers::isReadable(file)) {
            WRITE_ERRORF(TL("Could not open osm-file '%'."), file);
            return;
        }
        nodesHandler.setFileName(file);
        nodesHandler.resetHierarchy();
        const long before = PROGRESS_BEGIN_TIME_MESSAGE("Parsing nodes from osm-file '" + file + "'");
        readers.push_back(XMLSubSys::getSAXReader(nodesHandler));
        if (!readers.back()->parseFirst(file) || !readers.back()->parseSection(SUMO_TAG_NODE) ||
                MsgHandler::getErrorInstance()->wasInformed()) {
            return;
        }
        if (nodesHandler.getDuplicateNodes() > 0) {
            WRITE_MESSAGEF(TL("Found and substituted % osm nodes."), toString(nodesHandler.getDuplicateNodes()));
        }
        PROGRESS_TIME_MESSAGE(before);
    }

    // load edges, then
    EdgesHandler edgesHandler(myOSMNodes, myEdges, myPlatformShapes, nb.getTypeCont());
    int idx = 0;
    for (const std::string& file : files) {
        edgesHandler.setFileName(file);
        readers[idx]->setHandler(edgesHandler);
        const long before = PROGRESS_BEGIN_TIME_MESSAGE("Parsing edges from osm-file '" + file + "'");
        if (!readers[idx]->parseSection(SUMO_TAG_WAY)) {
            // eof already reached, no relations
            delete readers[idx];
            readers[idx] = nullptr;
        }
        PROGRESS_TIME_MESSAGE(before);
        idx++;
    }

    /* Remove duplicate edges with the same shape and attributes */
    if (!oc.getBool("osm.skip-duplicates-check")) {
        int numRemoved = 0;
        PROGRESS_BEGIN_MESSAGE(TL("Removing duplicate edges"));
        if (myEdges.size() > 1) {
            std::set<const Edge*, CompareEdges> dupsFinder;
            for (auto it = myEdges.begin(); it != myEdges.end();) {
                if (dupsFinder.count(it->second) > 0) {
                    numRemoved++;
                    delete it->second;
                    myEdges.erase(it++);
                } else {
                    dupsFinder.insert(it->second);
                    it++;
                }
            }
        }
        if (numRemoved > 0) {
            WRITE_MESSAGEF(TL("Removed % duplicate osm edges."), toString(numRemoved));
        }
        PROGRESS_DONE_MESSAGE();
    }

    /* Mark which nodes are used (by edges or traffic lights).
     * This is necessary to detect which OpenStreetMap nodes are for
     * geometry only */
    std::map<long long int, int> nodeUsage;
    // Mark which nodes are used by edges (begin and end)
    for (const auto& edgeIt : myEdges) {
        assert(edgeIt.second->myCurrentIsRoad);
        for (const long long int node : edgeIt.second->myCurrentNodes) {
            nodeUsage[node]++;
        }
    }
    // Mark which nodes are used by traffic lights or are pedestrian crossings
    for (const auto& nodesIt : myOSMNodes) {
        if (nodesIt.second->tlsControlled || nodesIt.second->railwaySignal || (nodesIt.second->pedestrianCrossing && myImportCrossings) /* || nodesIt->second->railwayCrossing*/) {
            // If the key is not found in the map, the value is automatically
            // initialized with 0.
            nodeUsage[nodesIt.first]++;
        }
    }

    /* Instantiate edges
     * Only those nodes in the middle of an edge which are used by more than
     * one edge are instantiated. Other nodes are considered as geometry nodes. */
    NBNodeCont& nc = nb.getNodeCont();
    NBTrafficLightLogicCont& tlsc = nb.getTLLogicCont();
    for (const auto& edgeIt : myEdges) {
        Edge* const e = edgeIt.second;
        if (!e->myCurrentIsRoad) {
            continue;
        }
        if (e->myCurrentNodes.size() < 2) {
            WRITE_WARNINGF(TL("Discarding way '%' because it has only % node(s)"), e->id, e->myCurrentNodes.size());
            continue;
        }
        extendRailwayDistances(e, nb.getTypeCont());
        // build nodes;
        //  - the from- and to-nodes must be built in any case
        //  - the in-between nodes are only built if more than one edge references them
        NBNode* first = insertNodeChecking(e->myCurrentNodes.front(), nc, tlsc);
        NBNode* last = insertNodeChecking(e->myCurrentNodes.back(), nc, tlsc);
        NBNode* currentFrom = first;
        int running = 0;
        std::vector<long long int> passed;
        for (auto j = e->myCurrentNodes.begin(); j != e->myCurrentNodes.end(); ++j) {
            passed.push_back(*j);
            if (nodeUsage[*j] > 1 && j != e->myCurrentNodes.end() - 1 && j != e->myCurrentNodes.begin()) {
                NBNode* currentTo = insertNodeChecking(*j, nc, tlsc);
                running = insertEdge(e, running, currentFrom, currentTo, passed, nb, first, last);
                currentFrom = currentTo;
                passed.clear();
                passed.push_back(*j);
            }
        }
        if (running == 0) {
            running = -1;
        }
        insertEdge(e, running, currentFrom, last, passed, nb, first, last);
    }

    /* Collect edges which explicitly are part of a roundabout and store the edges of each
     * detected roundabout */
    nb.getEdgeCont().extractRoundabouts();

    if (myImportCrossings) {
        /* After edges are instantiated
         * nodes are parsed again to add pedestrian crossings to them
         * This is only executed if crossings are imported and not guessed */
        const double crossingWidth = OptionsCont::getOptions().getFloat("default.crossing-width");

        for (auto item : nodeUsage) {
            NIOSMNode* osmNode = myOSMNodes.find(item.first)->second;
            if (osmNode->pedestrianCrossing) {
                NBNode* n = osmNode->node;
                EdgeVector incomingEdges = n->getIncomingEdges();
                EdgeVector outgoingEdges = n->getOutgoingEdges();
                size_t incomingEdgesNo = incomingEdges.size();
                size_t outgoingEdgesNo = outgoingEdges.size();

                for (size_t i = 0; i < incomingEdgesNo; i++) {
                    /* Check if incoming edge has driving lanes(and sidewalks)
                     * if not, ignore
                     * if yes, check if there is a corresponding outgoing edge for the opposite direction
                     *   -> if yes, check if it has driving lanes
                     *          --> if yes, do the crossing
                     *          --> if no, only do the crossing with the incoming edge (usually one lane roads with two sidewalks)
                     *   -> if not, do nothing as we don't have a sidewalk in the opposite direction */
                    auto const iEdge = incomingEdges[i];

                    if (iEdge->getFirstNonPedestrianLaneIndex(NBNode::FORWARD) > -1
                            && iEdge->getSpecialLane(SVC_PEDESTRIAN) > -1) {
                        std::string const& iEdgeId = iEdge->getID();
                        std::size_t const m = iEdgeId.find_first_of("#");
                        std::string const& iWayId = iEdgeId.substr(0, m);
                        for (size_t j = 0; j < outgoingEdgesNo; j++) {
                            auto const oEdge = outgoingEdges[j];
                            // Searching for a corresponding outgoing edge (based on OSM way identifier)
                            // with at least a pedestrian lane, going in the opposite direction
                            if (oEdge->getID().find(iWayId) != std::string::npos
                                    && oEdge->getSpecialLane(SVC_PEDESTRIAN) > -1
                                    && oEdge->getID().rfind(iWayId, 0) != 0) {
                                EdgeVector edgeVector = EdgeVector{ iEdge };
                                if (oEdge->getFirstNonPedestrianLaneIndex(NBNode::FORWARD) > -1) {
                                    edgeVector.push_back(oEdge);
                                }

                                if (!n->checkCrossingDuplicated(edgeVector)) {
                                    n->addCrossing(edgeVector, crossingWidth, false);
                                }
                            }
                        }
                    }
                }
                for (size_t i = 0; i < outgoingEdgesNo; i++) {
                    // Same checks as above for loop, but for outgoing edges
                    auto const oEdge = outgoingEdges[i];

                    if (oEdge->getFirstNonPedestrianLaneIndex(NBNode::FORWARD) > -1
                            && oEdge->getSpecialLane(SVC_PEDESTRIAN) > -1) {
                        std::string const& oEdgeId = oEdge->getID();
                        std::size_t const m = oEdgeId.find_first_of("#");
                        std::string const& iWayId = oEdgeId.substr(0, m);
                        for (size_t j = 0; j < incomingEdgesNo; j++) {
                            auto const iEdge = incomingEdges[j];
                            if (iEdge->getID().find(iWayId) != std::string::npos
                                    && iEdge->getSpecialLane(SVC_PEDESTRIAN) > -1
                                    && iEdge->getID().rfind(iWayId, 0) != 0) {
                                EdgeVector edgeVector = EdgeVector{ oEdge };
                                if (iEdge->getFirstNonPedestrianLaneIndex(NBNode::FORWARD) > -1) {
                                    edgeVector.push_back(iEdge);
                                }

                                if (!n->checkCrossingDuplicated(edgeVector)) {
                                    n->addCrossing(edgeVector, crossingWidth, false);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    const double layerElevation = oc.getFloat("osm.layer-elevation");
    if (layerElevation > 0) {
        reconstructLayerElevation(layerElevation, nb);
    }

    // revise pt stops; remove stops on deleted edges
    nb.getPTStopCont().cleanupDeleted(nb.getEdgeCont());

    // load relations (after edges are built since we want to apply
    // turn-restrictions directly to NBEdges)
    RelationHandler relationHandler(myOSMNodes, myEdges, &(nb.getPTStopCont()), myPlatformShapes,
                                    &nb.getPTLineCont(), oc);
    idx = 0;
    for (const std::string& file : files) {
        if (readers[idx] != nullptr) {
            relationHandler.setFileName(file);
            readers[idx]->setHandler(relationHandler);
            const long before = PROGRESS_BEGIN_TIME_MESSAGE("Parsing relations from osm-file '" + file + "'");
            readers[idx]->parseSection(SUMO_TAG_RELATION);
            PROGRESS_TIME_MESSAGE(before);
            delete readers[idx];
        }
        idx++;
    }

    // declare additional stops that are not anchored to a (road)-way or route relation
    std::set<std::string> stopNames;
    for (const auto& item : nb.getPTStopCont().getStops()) {
        stopNames.insert(item.second->getName());
    }
    for (const auto& item : myOSMNodes) {
        const NIOSMNode* n = item.second;
        if (n->ptStopPosition && stopNames.count(n->name) == 0) {
            Position ptPos(n->lon, n->lat, n->ele);
            if (!NBNetBuilder::transformCoordinate(ptPos)) {
                WRITE_ERRORF("Unable to project coordinates for node '%'.", n->id);
            }
            SumoXMLTag element = isRailway(n->permissions) ? SUMO_TAG_TRAIN_STOP : SUMO_TAG_BUS_STOP;
            std::shared_ptr<NBPTStop> ptStop = std::make_shared<NBPTStop>(element, toString(n->id), ptPos, "", "", n->ptStopLength, n->name, n->permissions);
            nb.getPTStopCont().insert(ptStop, true);
        }
    }
}

// ---------------------------------------------------------------------------
// definitions of NIImporter_OpenStreetMap-methods
// ---------------------------------------------------------------------------

NBNode*
NIImporter_OpenStreetMap::insertNodeChecking(long long int id, NBNodeCont& nc, NBTrafficLightLogicCont& tlsc) {
    NBNode* node = nc.retrieve(toString(id));
    if (node == nullptr) {
        NIOSMNode* n = myOSMNodes.find(id)->second;
        Position pos(n->lon, n->lat, n->ele);
        if (!NBNetBuilder::transformCoordinate(pos, true)) {
            WRITE_ERRORF("Unable to project coordinates for junction '%'.", id);
            return nullptr;
        }
        node = new NBNode(toString(id), pos);
        if (!nc.insert(node)) {
            WRITE_ERRORF(TL("Could not insert junction '%'."), toString(id));
            delete node;
            return nullptr;
        }
        n->node = node;
        if (n->railwayCrossing) {
            if (n->getParameter("crossing:barrier") != "no") {
                node->reinit(pos, SumoXMLNodeType::RAIL_CROSSING);
            } else if (n->getParameter("crossing.light") == "yes") {
                node->reinit(pos, SumoXMLNodeType::TRAFFIC_LIGHT);
            }
        } else if (n->railwaySignal) {
            node->reinit(pos, SumoXMLNodeType::RAIL_SIGNAL);
        } else if (n->tlsControlled) {
            // ok, this node is a traffic light node where no other nodes
            //  participate
            // @note: The OSM-community has not settled on a schema for differentiating between fixed and actuated lights
            TrafficLightType type = SUMOXMLDefinitions::TrafficLightTypes.get(
                                        OptionsCont::getOptions().getString("tls.default-type"));
            NBOwnTLDef* tlDef = new NBOwnTLDef(toString(id), node, 0, type);
            if (!tlsc.insert(tlDef)) {
                // actually, nothing should fail here
                delete tlDef;
                throw ProcessError(TLF("Could not allocate tls '%'.", toString(id)));
            }
        }
        if (n->railwayBufferStop) {
            node->setParameter("buffer_stop", "true");
            node->setFringeType(FringeType::INNER);
        }
        if (n->railwaySignal) {
            if (n->myRailDirection == WAY_FORWARD) {
                node->setParameter(NBTrafficLightDefinition::OSM_SIGNAL_DIRECTION, "forward");
            } else if (n->myRailDirection == WAY_BACKWARD) {
                node->setParameter(NBTrafficLightDefinition::OSM_SIGNAL_DIRECTION, "backward");
            }
        }
        node->updateParameters(n->getParametersMap());
    }
    return node;
}


int
NIImporter_OpenStreetMap::insertEdge(Edge* e, int index, NBNode* from, NBNode* to,
                                     const std::vector<long long int>& passed, NBNetBuilder& nb,
                                     const NBNode* first, const NBNode* last) {
    NBNodeCont& nc = nb.getNodeCont();
    NBEdgeCont& ec = nb.getEdgeCont();
    NBTypeCont& tc = nb.getTypeCont();
    NBPTStopCont& sc = nb.getPTStopCont();

    NBTrafficLightLogicCont& tlsc = nb.getTLLogicCont();
    // patch the id
    std::string id = toString(e->id);
    if (from == nullptr || to == nullptr) {
        WRITE_ERRORF("Discarding edge '%' because the nodes could not be built.", id);
        return index;
    }
    if (index >= 0) {
        id = id + "#" + toString(index);
    } else {
        index = 0;
    }
    if (from == to) {
        assert(passed.size() >= 2);
        if (passed.size() == 2) {
            WRITE_WARNINGF(TL("Discarding edge '%' which connects two identical nodes without geometry."), id);
            return index;
        }
        // in the special case of a looped way split again using passed
        int intermediateIndex = (int) passed.size() / 2;
        NBNode* intermediate = insertNodeChecking(passed[intermediateIndex], nc, tlsc);
        std::vector<long long int> part1(passed.begin(), passed.begin() + intermediateIndex + 1);
        std::vector<long long int> part2(passed.begin() + intermediateIndex, passed.end());
        index = insertEdge(e, index, from, intermediate, part1, nb, first, last);
        return insertEdge(e, index, intermediate, to, part2, nb, first, last);
    }
    const int newIndex = index + 1;
    const std::string type = usableType(e->myHighWayType, id, tc);
    if (type == "") {  // we do not want to import it
        return newIndex;
    }
    std::string routingType = "";
    int numLanesForward = tc.getEdgeTypeNumLanes(type);
    int numLanesBackward = tc.getEdgeTypeNumLanes(type);
    double speed = tc.getEdgeTypeSpeed(type);
    bool defaultsToOneWay = tc.getEdgeTypeIsOneWay(type);
    const SVCPermissions defaultPermissions = tc.getEdgeTypePermissions(type);
    SVCPermissions extra = myImportBikeAccess ? e->myExtraAllowed : (e->myExtraAllowed & ~SVC_BICYCLE);
    const SVCPermissions extraDis = myImportBikeAccess ? e->myExtraDisallowed : (e->myExtraDisallowed & ~SVC_BICYCLE);
    std::vector<SumoXMLAttr> defaults;
    // extra permissions are more specific than extra prohibitions except for buses (which come from the less specific psv tag)
    if ((extraDis & SVC_BUS) && (extra & SVC_BUS)) {
        extra = extra & ~SVC_BUS;
    }
    SVCPermissions permissions = (defaultPermissions & ~extraDis) | extra;
    if (defaultPermissions == SVC_SHIP) {
        // extra permission apply to the ships operating on the route rather than the waterway
        permissions = defaultPermissions;
    }
    if (defaultsToOneWay && defaultPermissions == SVC_PEDESTRIAN && (permissions & (~SVC_PEDESTRIAN)) != 0) {
        defaultsToOneWay = false;
    }
    if ((permissions & SVC_RAIL) != 0 && e->myExtraTags.count("electrified") != 0) {
        permissions |= (SVC_RAIL_ELECTRIC | SVC_RAIL_FAST);
    }

    // convert the shape
    PositionVector shape;
    double distanceStart = myOSMNodes[passed.front()]->positionMeters;
    double distanceEnd = myOSMNodes[passed.back()]->positionMeters;
    const bool useDistance = distanceStart != std::numeric_limits<double>::max() && distanceEnd != std::numeric_limits<double>::max();
    if (useDistance) {
        // negative sign denotes counting in the other direction
        if (distanceStart < distanceEnd) {
            distanceStart *= -1;
        } else {
            distanceEnd *= -1;
        }
    } else {
        distanceStart = 0;
        distanceEnd = 0;
    }
    // get additional direction information
    int nodeDirection = myOSMNodes.find(StringUtils::toLong(from->getID()))->second->myRailDirection |
                        myOSMNodes.find(StringUtils::toLong(to->getID()))->second->myRailDirection;

    std::vector<std::shared_ptr<NBPTStop> > ptStops;
    for (long long i : passed) {
        NIOSMNode* n = myOSMNodes.find(i)->second;
        // recheck permissions, maybe they got assigned to a strange edge, see #11656
        if (n->ptStopPosition && (n->permissions == 0 || (permissions & n->permissions) != 0)) {
            std::shared_ptr<NBPTStop> existingPtStop = sc.get(toString(n->id));
            if (existingPtStop != nullptr) {
                existingPtStop->registerAdditionalEdge(toString(e->id), id);
            } else {
                Position ptPos(n->lon, n->lat, n->ele);
                if (!NBNetBuilder::transformCoordinate(ptPos)) {
                    WRITE_ERRORF("Unable to project coordinates for node '%'.", n->id);
                }
                SumoXMLTag element = isRailway(n->permissions) ? SUMO_TAG_TRAIN_STOP : SUMO_TAG_BUS_STOP;
                ptStops.push_back(std::make_shared<NBPTStop>(element, toString(n->id), ptPos, id, toString(e->id), n->ptStopLength, n->name, n->permissions));
                sc.insert(ptStops.back());
            }
        }
        if (n->railwaySignal) {
            nodeDirection |= n->myRailDirection;
        }
        Position pos(n->lon, n->lat, n->ele);
        shape.push_back(pos);
    }
#ifdef DEBUG_LAYER_ELEVATION
    if (e->id == "DEBUGID") {
        std::cout
                << " id=" << id << " from=" << from->getID() << " fromRailDirection=" << myOSMNodes.find(StringUtils::toLong(from->getID()))->second->myRailDirection
                << " to=" << to->getID() << " toRailDirection=" << myOSMNodes.find(StringUtils::toLong(to->getID()))->second->myRailDirection
                << " origRailDirection=" << e->myRailDirection
                << " nodeDirection=" << nodeDirection
                << "\n";
    }
#endif
    if (e->myRailDirection == WAY_UNKNOWN && nodeDirection != WAY_UNKNOWN && nodeDirection != WAY_FORWARD
            && nodeDirection != (WAY_FORWARD | WAY_UNKNOWN)) {
        //std::cout << "way " << e->id << " nodeDirection=" << nodeDirection << " origDirection=" << e->myRailDirection << "\n";
        // heuristic: assume that the mapped way direction indicates
        // potential driving direction
        e->myRailDirection = WAY_BOTH;
    }
    if (!NBNetBuilder::transformCoordinates(shape)) {
        WRITE_ERRORF("Unable to project coordinates for edge '%'.", id);
    }

    SVCPermissions forwardPermissions = permissions;
    SVCPermissions backwardPermissions = permissions;
    const std::string streetName = isRailway(permissions) && e->ref != "" ? e->ref : e->streetName;
    if (streetName == e->ref) {
        e->unsetParameter("ref"); // avoid superfluous param for railways
    }
    double forwardWidth = tc.getEdgeTypeWidth(type);
    double backwardWidth = tc.getEdgeTypeWidth(type);
    double sidewalkWidth = tc.getEdgeTypeSidewalkWidth(type);
    bool addSidewalk = sidewalkWidth != NBEdge::UNSPECIFIED_WIDTH;
    if (myImportSidewalks) {
        if (addSidewalk) {
            // only use sidewalk width from typemap but don't add sidewalks
            // unless OSM specifies them
            addSidewalk = false;
        } else {
            sidewalkWidth = OptionsCont::getOptions().getFloat("default.sidewalk-width");
        }
    }
    double bikeLaneWidth = tc.getEdgeTypeBikeLaneWidth(type);
    const std::string& onewayBike = e->myExtraTags["oneway:bicycle"];
    if (onewayBike == "false" || onewayBike == "no" || onewayBike == "0") {
        e->myCyclewayType = e->myCyclewayType == WAY_UNKNOWN ? WAY_BACKWARD : (WayType)(e->myCyclewayType | WAY_BACKWARD);
    }

    const bool addBikeLane = bikeLaneWidth != NBEdge::UNSPECIFIED_WIDTH ||
                             (myImportBikeAccess && (((e->myCyclewayType & WAY_BOTH) != 0 || e->myExtraTags.count("segregated") != 0) &&
                                     !(e->myCyclewayType == WAY_BACKWARD && (e->myBuswayType & WAY_BOTH) != 0)));
    if (addBikeLane && bikeLaneWidth == NBEdge::UNSPECIFIED_WIDTH) {
        bikeLaneWidth = OptionsCont::getOptions().getFloat("default.bikelane-width");
    }
    // check directions
    bool addForward = true;
    bool addBackward = true;
    const bool explicitTwoWay = e->myIsOneWay == "no";
    if ((e->myIsOneWay == "true" || e->myIsOneWay == "yes" || e->myIsOneWay == "1"
            || (defaultsToOneWay && e->myIsOneWay != "no" && e->myIsOneWay != "false" && e->myIsOneWay != "0"))
            && e->myRailDirection != WAY_BOTH) {
        addBackward = false;
    }
    if (e->myIsOneWay == "-1" || e->myIsOneWay == "reverse" || e->myRailDirection == WAY_BACKWARD) {
        // one-way in reversed direction of way
        addForward = false;
        addBackward = true;
    }
    if (!e->myIsOneWay.empty() && e->myIsOneWay != "false" && e->myIsOneWay != "no" && e->myIsOneWay != "true"
            && e->myIsOneWay != "yes" && e->myIsOneWay != "-1" && e->myIsOneWay != "1" && e->myIsOneWay != "reverse") {
        WRITE_WARNINGF(TL("New value for oneway found: %"), e->myIsOneWay);
    }
    if ((permissions == SVC_BICYCLE || permissions == (SVC_BICYCLE | SVC_PEDESTRIAN) || permissions == SVC_PEDESTRIAN)) {
        if (addBackward && (onewayBike == "true" || onewayBike == "yes" || onewayBike == "1")) {
            addBackward = false;
        }
        if (addForward && (onewayBike == "reverse" || onewayBike == "-1")) {
            addForward = false;
        }
        if (!addBackward && (onewayBike == "false" || onewayBike == "no" || onewayBike == "0")) {
            addBackward = true;
        }
    }
    bool ok = true;
    // if we had been able to extract the number of lanes, override the highway type default
    if (e->myNoLanes > 0) {
        if (addForward && !addBackward) {
            numLanesForward = e->myNoLanesForward > 0 ? e->myNoLanesForward : e->myNoLanes;
        } else if (!addForward && addBackward) {
            numLanesBackward = e->myNoLanesForward < 0 ? -e->myNoLanesForward : e->myNoLanes;
        } else {
            if (e->myNoLanesForward > 0) {
                numLanesForward = e->myNoLanesForward;
            } else if (e->myNoLanesForward < 0) {
                numLanesForward = e->myNoLanes + e->myNoLanesForward;
            } else {
                numLanesForward = (int) std::ceil(e->myNoLanes / 2.0);
            }
            numLanesBackward = e->myNoLanes - numLanesForward;
            // sometimes ways are tagged according to their physical width of a single
            // lane but they are intended for traffic in both directions
            numLanesForward = MAX2(1, numLanesForward);
            numLanesBackward = MAX2(1, numLanesBackward);
        }
    } else if (e->myNoLanes == 0) {
        WRITE_WARNINGF(TL("Skipping edge '%' because it has zero lanes."), id);
        ok = false;
    } else {
        // the total number of lanes is not known but at least one direction
        if (e->myNoLanesForward > 0) {
            numLanesForward = e->myNoLanesForward;
        } else if ((e->myBuswayType & WAY_FORWARD) != 0 && (extraDis & SVC_PASSENGER) == 0) {
            // if we have a busway lane, yet cars may drive this implies at least two lanes
            numLanesForward = MAX2(numLanesForward, 2);
        }
        if (e->myNoLanesForward < 0) {
            numLanesBackward = -e->myNoLanesForward;
        } else if ((e->myBuswayType & WAY_BACKWARD) != 0 && (extraDis & SVC_PASSENGER) == 0) {
            // if we have a busway lane, yet cars may drive this implies at least two lanes
            numLanesBackward = MAX2(numLanesForward, 2);
        }
        if (myAnnotateDefaults && e->myNoLanesForward == 0) {
            defaults.push_back(SUMO_ATTR_NUMLANES);
        }
    }
    // deal with busways that run in the opposite direction of a one-way street
    if (!addForward && (e->myBuswayType & WAY_FORWARD) != 0) {
        addForward = true;
        forwardPermissions = SVC_BUS;
        numLanesForward = 1;
    }
    if (!addBackward && (e->myBuswayType & WAY_BACKWARD) != 0) {
        addBackward = true;
        backwardPermissions = SVC_BUS;
        numLanesBackward = 1;
    }
    // width is meant for raw lane count before adding sidewalks or cycleways
    const int taggedLanes = (addForward ? numLanesForward : 0) + (addBackward ? numLanesBackward : 0);
    if (e->myWidth > 0 && e->myWidthLanesForward.size() == 0 && e->myWidthLanesBackward.size() == 0 && taggedLanes != 0
            && !OptionsCont::getOptions().getBool("ignore-widths")) {
        // width is tagged excluding sidewalks and cycleways
        forwardWidth = e->myWidth / taggedLanes;
        backwardWidth = forwardWidth;
    }

    // if we had been able to extract the maximum speed, override the type's default
    if (e->myMaxSpeed != MAXSPEED_UNGIVEN) {
        speed = e->myMaxSpeed;
    } else if (myAnnotateDefaults) {
        defaults.push_back(SUMO_ATTR_SPEED);
    }
    double speedBackward = speed;
    if (e->myMaxSpeedBackward != MAXSPEED_UNGIVEN) {
        speedBackward = e->myMaxSpeedBackward;
    }
    if (speed <= 0 || speedBackward <= 0) {
        WRITE_WARNINGF(TL("Skipping edge '%' because it has speed %."), id, speed);
        ok = false;
    }
    if (e->myNoLanes == 1 && addForward && addBackward) {
        // narrow road which now receives a total of 2 lanes but has less capacity than implied
        if (e->myWidth < 0 && e->myWidthLanesForward.size() == 0 && e->myWidthLanesBackward.size() == 0) {
            if (forwardWidth == NBEdge::UNSPECIFIED_WIDTH) {
                forwardWidth = SUMO_const_laneWidth;
            }
            if (backwardWidth == NBEdge::UNSPECIFIED_WIDTH) {
                backwardWidth = SUMO_const_laneWidth;
            }
            forwardWidth /= 2;
            backwardWidth /= 2;
        }
        if (e->myWidth < 5) {
            routingType = "narrow";
        }
    }
    // deal with cycleways that run in the opposite direction of a one-way street
    WayType cyclewayType = e->myCyclewayType; // make a copy because we do some temporary modifications
    if (addBikeLane) {
        if (!addForward && (cyclewayType & WAY_FORWARD) != 0) {
            addForward = true;
            forwardPermissions = SVC_BICYCLE;
            forwardWidth = bikeLaneWidth;
            numLanesForward = 1;
            // do not add an additional cycle lane
            cyclewayType = (WayType)(cyclewayType & ~WAY_FORWARD);
        }
        if (!addBackward && (cyclewayType & WAY_BACKWARD) != 0) {
            addBackward = true;
            backwardPermissions = SVC_BICYCLE;
            backwardWidth = bikeLaneWidth;
            numLanesBackward = 1;
            // do not add an additional cycle lane
            cyclewayType = (WayType)(cyclewayType & ~WAY_BACKWARD);
        }
    }
    // deal with sidewalks that run in the opposite direction of a one-way street
    WayType sidewalkType = e->mySidewalkType; // make a copy because we do some temporary modifications
    if (sidewalkType == WAY_UNKNOWN && (e->myExtraAllowed & SVC_PEDESTRIAN) != 0 && (permissions & SVC_PASSENGER) != 0) {
        // do not assume shared space unless sidewalk is actively disabled
        if (myOnewayDualSidewalk) {
            sidewalkType = WAY_BOTH;
        }
    }
    if (addSidewalk || (myImportSidewalks && (permissions & SVC_ROAD_CLASSES) != 0 && defaultPermissions != SVC_PEDESTRIAN)) {
        if (!addForward && (sidewalkType & WAY_FORWARD) != 0) {
            addForward = true;
            forwardPermissions = SVC_PEDESTRIAN;
            forwardWidth = tc.getEdgeTypeSidewalkWidth(type);
            numLanesForward = 1;
            // do not add an additional sidewalk
            sidewalkType = (WayType)(sidewalkType & ~WAY_FORWARD);  //clang tidy thinks "!WAY_FORWARD" is always false
        } else if (addSidewalk && addForward && (sidewalkType & WAY_BOTH) == 0
                   && numLanesForward == 1 && numLanesBackward <= 1
                   && (e->myExtraDisallowed & SVC_PEDESTRIAN) == 0) {
            // our typemap says pedestrians should walk here but the data says
            // there is no sidewalk at all. If the road is small, pedestrians can just walk
            // on the road
            forwardPermissions |= SVC_PEDESTRIAN;
        }
        if (!addBackward && (sidewalkType & WAY_BACKWARD) != 0) {
            addBackward = true;
            backwardPermissions = SVC_PEDESTRIAN;
            backwardWidth = tc.getEdgeTypeSidewalkWidth(type);
            numLanesBackward = 1;
            // do not add an additional cycle lane
            sidewalkType = (WayType)(sidewalkType & ~WAY_BACKWARD); //clang tidy thinks "!WAY_BACKWARD" is always false
        } else if (addSidewalk && addBackward && (sidewalkType & WAY_BOTH) == 0
                   && numLanesBackward == 1 && numLanesForward <= 1
                   && (e->myExtraDisallowed & SVC_PEDESTRIAN) == 0) {
            // our typemap says pedestrians should walk here but the data says
            // there is no sidewalk at all. If the road is small, pedestrians can just walk
            // on the road
            backwardPermissions |= SVC_PEDESTRIAN;
        }
    }

    const std::string origID = OptionsCont::getOptions().getBool("output.original-names") ? toString(e->id) : "";
    if (ok) {
        const bool lefthand = OptionsCont::getOptions().getBool("lefthand");
        const int offsetFactor = lefthand ? -1 : 1;
        LaneSpreadFunction lsf = (addBackward || OptionsCont::getOptions().getBool("osm.oneway-spread-right")) &&
                                 (e->myRailDirection == WAY_UNKNOWN || explicitTwoWay)  ? LaneSpreadFunction::RIGHT : LaneSpreadFunction::CENTER;
        if (addBackward && lsf == LaneSpreadFunction::RIGHT && OptionsCont::getOptions().getString("default.spreadtype") == toString(LaneSpreadFunction::ROADCENTER)) {
            lsf = LaneSpreadFunction::ROADCENTER;
        }
        if (tc.getEdgeTypeSpreadType(type) != LaneSpreadFunction::RIGHT) {
            // user defined value overrides defaults
            lsf = tc.getEdgeTypeSpreadType(type);
        }
        if (defaults.size() > 0) {
            e->setParameter("osmDefaults", joinToString(defaults, " "));
        }

        id = StringUtils::escapeXML(id);
        const std::string reverseID = "-" + id;
        const bool markOSMDirection =  from->getType() == SumoXMLNodeType::RAIL_SIGNAL || to->getType() == SumoXMLNodeType::RAIL_SIGNAL;
        if (addForward) {
            assert(numLanesForward > 0);
            NBEdge* nbe = new NBEdge(id, from, to, type, speed, NBEdge::UNSPECIFIED_FRICTION, numLanesForward, tc.getEdgeTypePriority(type),
                                     forwardWidth, NBEdge::UNSPECIFIED_OFFSET, shape, lsf,
                                     StringUtils::escapeXML(streetName), origID, true);
            if (markOSMDirection) {
                nbe->setParameter(NBTrafficLightDefinition::OSM_DIRECTION, "forward");
            }
            nbe->setPermissions(forwardPermissions, -1);
            if ((e->myBuswayType & WAY_FORWARD) != 0) {
                nbe->setPermissions(SVC_BUS, 0);
            }
            applyChangeProhibition(nbe, e->myChangeForward);
            applyLaneUse(nbe, e, true);
            applyTurnSigns(nbe, e->myTurnSignsForward);
            nbe->setTurnSignTarget(last->getID());
            if (addBikeLane && (cyclewayType == WAY_UNKNOWN || (cyclewayType & WAY_FORWARD) != 0)) {
                nbe->addBikeLane(bikeLaneWidth * offsetFactor);
            } else if (nbe->getPermissions(0) == SVC_BUS) {
                // bikes drive on buslanes if no separate cycle lane is available
                nbe->setPermissions(SVC_BUS | SVC_BICYCLE, 0);
            }
            if ((addSidewalk && (sidewalkType == WAY_UNKNOWN || (sidewalkType & WAY_FORWARD) != 0))
                    || (myImportSidewalks && (sidewalkType & WAY_FORWARD) != 0 && defaultPermissions != SVC_PEDESTRIAN)) {
                nbe->addSidewalk(sidewalkWidth * offsetFactor);
            }
            if (!addBackward && (e->myExtraAllowed & SVC_PEDESTRIAN) != 0 && (nbe->getPermissions(0) & SVC_PEDESTRIAN) == 0) {
                // Pedestrians are explicitly allowed (maybe through foot="yes") but did not get a sidewalk (maybe through sidewalk="no").
                // Since we do not have a backward edge, we need to make sure they can at least walk somewhere, see #14124
                nbe->setPermissions(nbe->getPermissions(0) | SVC_PEDESTRIAN, 0);
            }
            nbe->updateParameters(e->getParametersMap());
            nbe->setDistance(distanceStart);
            if (e->myAmInRoundabout) {
                // ensure roundabout edges have the precedence
                nbe->setJunctionPriority(to, NBEdge::JunctionPriority::ROUNDABOUT);
                nbe->setJunctionPriority(from, NBEdge::JunctionPriority::ROUNDABOUT);
            }

            // process forward lanes width
            const int numForwardLanesFromWidthKey = (int)e->myWidthLanesForward.size();
            if (numForwardLanesFromWidthKey > 0 && !OptionsCont::getOptions().getBool("ignore-widths")) {
                if ((int)nbe->getLanes().size() != numForwardLanesFromWidthKey) {
                    WRITE_WARNINGF(TL("Forward lanes count for edge '%' ('%') is not matching the number of lanes defined in width:lanes:forward key ('%'). Using default width values."),
                                   id, nbe->getLanes().size(), numForwardLanesFromWidthKey);
                } else {
                    for (int i = 0; i < numForwardLanesFromWidthKey; i++) {
                        const double actualWidth = e->myWidthLanesForward[i] <= 0 ? forwardWidth : e->myWidthLanesForward[i];
                        const int laneIndex = lefthand ? i : numForwardLanesFromWidthKey - i - 1;
                        nbe->setLaneWidth(laneIndex, actualWidth);
                    }
                }
            }
            nbe->setRoutingType(routingType);

            if (!ec.insert(nbe)) {
                delete nbe;
                throw ProcessError(TLF("Could not add edge '%'.", id));
            }
        }
        if (addBackward) {
            assert(numLanesBackward > 0);
            NBEdge* nbe = new NBEdge(reverseID, to, from, type, speedBackward, NBEdge::UNSPECIFIED_FRICTION, numLanesBackward, tc.getEdgeTypePriority(type),
                                     backwardWidth, NBEdge::UNSPECIFIED_OFFSET, shape.reverse(), lsf,
                                     StringUtils::escapeXML(streetName), origID, true);
            if (markOSMDirection) {
                nbe->setParameter(NBTrafficLightDefinition::OSM_DIRECTION, "backward");
            }
            nbe->setPermissions(backwardPermissions);
            if ((e->myBuswayType & WAY_BACKWARD) != 0) {
                nbe->setPermissions(SVC_BUS, 0);
            }
            applyChangeProhibition(nbe, e->myChangeBackward);
            applyLaneUse(nbe, e, false);
            applyTurnSigns(nbe, e->myTurnSignsBackward);
            nbe->setTurnSignTarget(first->getID());
            if (addBikeLane && (cyclewayType == WAY_UNKNOWN || (cyclewayType & WAY_BACKWARD) != 0)) {
                nbe->addBikeLane(bikeLaneWidth * offsetFactor);
            } else if (nbe->getPermissions(0) == SVC_BUS) {
                // bikes drive on buslanes if no separate cycle lane is available
                nbe->setPermissions(SVC_BUS | SVC_BICYCLE, 0);
            }
            if ((addSidewalk && (sidewalkType == WAY_UNKNOWN || (sidewalkType & WAY_BACKWARD) != 0))
                    || (myImportSidewalks && (sidewalkType & WAY_BACKWARD) != 0 && defaultPermissions != SVC_PEDESTRIAN)) {
                nbe->addSidewalk(sidewalkWidth * offsetFactor);
            }
            nbe->updateParameters(e->getParametersMap());
            nbe->setDistance(distanceEnd);
            if (e->myAmInRoundabout) {
                // ensure roundabout edges have the precedence
                nbe->setJunctionPriority(from, NBEdge::JunctionPriority::ROUNDABOUT);
                nbe->setJunctionPriority(to, NBEdge::JunctionPriority::ROUNDABOUT);
            }
            // process backward lanes width
            const int numBackwardLanesFromWidthKey = (int)e->myWidthLanesBackward.size();
            if (numBackwardLanesFromWidthKey > 0 && !OptionsCont::getOptions().getBool("ignore-widths")) {
                if ((int)nbe->getLanes().size() != numBackwardLanesFromWidthKey) {
                    WRITE_WARNINGF(TL("Backward lanes count for edge '%' ('%') is not matching the number of lanes defined in width:lanes:backward key ('%'). Using default width values."),
                                   id, nbe->getLanes().size(), numBackwardLanesFromWidthKey);
                } else {
                    for (int i = 0; i < numBackwardLanesFromWidthKey; i++) {
                        const double actualWidth = e->myWidthLanesBackward[i] <= 0 ? backwardWidth : e->myWidthLanesBackward[i];
                        const int laneIndex = lefthand ? i : numBackwardLanesFromWidthKey - i - 1;
                        nbe->setLaneWidth(laneIndex, actualWidth);
                    }
                }
            }
            nbe->setRoutingType(routingType);

            if (!ec.insert(nbe)) {
                delete nbe;
                throw ProcessError(TLF("Could not add edge '-%'.", id));
            }
        }
        if ((e->myParkingType & PARKING_BOTH) != 0 && OptionsCont::getOptions().isSet("parking-output")) {
            if ((e->myParkingType & PARKING_RIGHT) != 0) {
                if (addForward) {
                    nb.getParkingCont().push_back(NBParking(id, id));
                } else {
                    /// XXX parking area should be added on the left side of a reverse one-way street
                    if ((e->myParkingType & PARKING_LEFT) == 0 && !addBackward) {
                        /// put it on the wrong side (better than nothing)
                        nb.getParkingCont().push_back(NBParking(reverseID, reverseID));
                    }
                }
            }
            if ((e->myParkingType & PARKING_LEFT) != 0) {
                if (addBackward) {
                    nb.getParkingCont().push_back(NBParking(reverseID, reverseID));
                } else {
                    /// XXX parking area should be added on the left side of an one-way street
                    if ((e->myParkingType & PARKING_RIGHT) == 0 && !addForward) {
                        /// put it on the wrong side (better than nothing)
                        nb.getParkingCont().push_back(NBParking(id, id));
                    }
                }
            }
        }
    }
    return newIndex;
}


void
NIImporter_OpenStreetMap::reconstructLayerElevation(const double layerElevation, NBNetBuilder& nb) {
    NBNodeCont& nc = nb.getNodeCont();
    NBEdgeCont& ec = nb.getEdgeCont();
    // reconstruct elevation from layer info
    // build a map of raising and lowering forces (attractor and distance)
    // for all nodes unknownElevation
    std::map<NBNode*, std::vector<std::pair<double, double> > > layerForces;

    // collect all nodes that belong to a way with layer information
    std::set<NBNode*> knownElevation;
    for (auto& myEdge : myEdges) {
        Edge* e = myEdge.second;
        if (e->myLayer != 0) {
            for (auto j = e->myCurrentNodes.begin(); j != e->myCurrentNodes.end(); ++j) {
                NBNode* node = nc.retrieve(toString(*j));
                if (node != nullptr) {
                    knownElevation.insert(node);
                    layerForces[node].emplace_back(e->myLayer * layerElevation, POSITION_EPS);
                }
            }
        }
    }
#ifdef DEBUG_LAYER_ELEVATION
    std::cout << "known elevations:\n";
    for (std::set<NBNode*>::iterator it = knownElevation.begin(); it != knownElevation.end(); ++it) {
        const std::vector<std::pair<double, double> >& primaryLayers = layerForces[*it];
        std::cout << "  node=" << (*it)->getID() << " ele=";
        for (std::vector<std::pair<double, double> >::const_iterator it_ele = primaryLayers.begin(); it_ele != primaryLayers.end(); ++it_ele) {
            std::cout << it_ele->first << " ";
        }
        std::cout << "\n";
    }
#endif
    // layer data only provides a lower bound on elevation since it is used to
    // resolve the relation among overlapping ways.
    // Perform a sanity check for steep inclines and raise the knownElevation if necessary
    std::map<NBNode*, double> knownEleMax;
    for (auto it : knownElevation) {
        double eleMax = -std::numeric_limits<double>::max();
        const std::vector<std::pair<double, double> >& primaryLayers = layerForces[it];
        for (const auto& primaryLayer : primaryLayers) {
            eleMax = MAX2(eleMax, primaryLayer.first);
        }
        knownEleMax[it] = eleMax;
    }
    const double gradeThreshold = OptionsCont::getOptions().getFloat("osm.layer-elevation.max-grade") / 100;
    bool changed = true;
    while (changed) {
        changed = false;
        for (auto it = knownElevation.begin(); it != knownElevation.end(); ++it) {
            std::map<NBNode*, std::pair<double, double> > neighbors = getNeighboringNodes(*it,
                    knownEleMax[*it]
                    / gradeThreshold * 3,
                    knownElevation);
            for (auto& neighbor : neighbors) {
                if (knownElevation.count(neighbor.first) != 0) {
                    const double grade = fabs(knownEleMax[*it] - knownEleMax[neighbor.first])
                                         / MAX2(POSITION_EPS, neighbor.second.first);
#ifdef DEBUG_LAYER_ELEVATION
                    std::cout << "   grade at node=" << (*it)->getID() << " ele=" << knownEleMax[*it] << " neigh=" << it_neigh->first->getID() << " neighEle=" << knownEleMax[it_neigh->first] << " grade=" << grade << " dist=" << it_neigh->second.first << " speed=" << it_neigh->second.second << "\n";
#endif
                    if (grade > gradeThreshold * 50 / 3.6 / neighbor.second.second) {
                        // raise the lower node to the higher level
                        const double eleMax = MAX2(knownEleMax[*it], knownEleMax[neighbor.first]);
                        if (knownEleMax[*it] < eleMax) {
                            knownEleMax[*it] = eleMax;
                        } else {
                            knownEleMax[neighbor.first] = eleMax;
                        }
                        changed = true;
                    }
                }
            }
        }
    }

    // collect all nodes within a grade-dependent range around knownElevation-nodes and apply knowElevation forces
    std::set<NBNode*> unknownElevation;
    for (auto it = knownElevation.begin(); it != knownElevation.end(); ++it) {
        const double eleMax = knownEleMax[*it];
        const double maxDist = fabs(eleMax) * 100 / layerElevation;
        std::map<NBNode*, std::pair<double, double> > neighbors = getNeighboringNodes(*it, maxDist, knownElevation);
        for (auto& neighbor : neighbors) {
            if (knownElevation.count(neighbor.first) == 0) {
                unknownElevation.insert(neighbor.first);
                layerForces[neighbor.first].emplace_back(eleMax, neighbor.second.first);
            }
        }
    }

    // apply forces to ground-level nodes (neither in knownElevation nor unknownElevation)
    for (auto it = unknownElevation.begin(); it != unknownElevation.end(); ++it) {
        double eleMax = -std::numeric_limits<double>::max();
        const std::vector<std::pair<double, double> >& primaryLayers = layerForces[*it];
        for (const auto& primaryLayer : primaryLayers) {
            eleMax = MAX2(eleMax, primaryLayer.first);
        }
        const double maxDist = fabs(eleMax) * 100 / layerElevation;
        std::map<NBNode*, std::pair<double, double> > neighbors = getNeighboringNodes(*it, maxDist, knownElevation);
        for (auto& neighbor : neighbors) {
            if (knownElevation.count(neighbor.first) == 0 && unknownElevation.count(neighbor.first) == 0) {
                layerForces[*it].emplace_back(0, neighbor.second.first);
            }
        }
    }
    // compute the elevation for each node as the weighted average of all forces
#ifdef DEBUG_LAYER_ELEVATION
    std::cout << "summation of forces\n";
#endif
    std::map<NBNode*, double> nodeElevation;
    for (auto& layerForce : layerForces) {
        const std::vector<std::pair<double, double> >& forces = layerForce.second;
        if (knownElevation.count(layerForce.first) != 0) {
            // use the maximum value
            /*
            double eleMax = -std::numeric_limits<double>::max();
            for (std::vector<std::pair<double, double> >::const_iterator it_force = forces.begin(); it_force != forces.end(); ++it_force) {
                eleMax = MAX2(eleMax, it_force->first);
            }
            */
#ifdef DEBUG_LAYER_ELEVATION
            std::cout << "   node=" << it->first->getID() << " knownElevation=" << knownEleMax[it->first] << "\n";
#endif
            nodeElevation[layerForce.first] = knownEleMax[layerForce.first];
        } else if (forces.size() == 1) {
            nodeElevation[layerForce.first] = forces.front().first;
        } else {
            // use the weighted sum
            double distSum = 0;
            for (const auto& force : forces) {
                distSum += force.second;
            }
            double weightSum = 0;
            double elevation = 0;
#ifdef DEBUG_LAYER_ELEVATION
            std::cout << "   node=" << it->first->getID() << "  distSum=" << distSum << "\n";
#endif
            for (const auto& force : forces) {
                const double weight = (distSum - force.second) / distSum;
                weightSum += weight;
                elevation += force.first * weight;

#ifdef DEBUG_LAYER_ELEVATION
                std::cout << "       force=" << it_force->first << " dist=" << it_force->second << "  weight=" << weight << " ele=" << elevation << "\n";
#endif
            }
            nodeElevation[layerForce.first] = elevation / weightSum;
        }
    }
#ifdef DEBUG_LAYER_ELEVATION
    std::cout << "final elevations:\n";
    for (std::map<NBNode*, double>::iterator it = nodeElevation.begin(); it != nodeElevation.end(); ++it) {
        std::cout << "  node=" << (it->first)->getID() << " ele=" << it->second << "\n";
    }
#endif
    // apply node elevations
    for (auto& it : nodeElevation) {
        NBNode* n = it.first;
        n->reinit(n->getPosition() + Position(0, 0, it.second), n->getType());
    }

    // apply way elevation to all edges that had layer information
    for (const auto& it : ec) {
        NBEdge* edge = it.second;
        const PositionVector& geom = edge->getGeometry();
        const double length = geom.length2D();
        const double zFrom = nodeElevation[edge->getFromNode()];
        const double zTo = nodeElevation[edge->getToNode()];
        // XXX if the from- or to-node was part of multiple ways with
        // different layers, reconstruct the layer value from origID
        double dist = 0;
        PositionVector newGeom;
        for (auto it_pos = geom.begin(); it_pos != geom.end(); ++it_pos) {
            if (it_pos != geom.begin()) {
                dist += (*it_pos).distanceTo2D(*(it_pos - 1));
            }
            newGeom.push_back((*it_pos) + Position(0, 0, zFrom + (zTo - zFrom) * dist / length));
        }
        edge->setGeometry(newGeom);
    }
}

std::map<NBNode*, std::pair<double, double> >
NIImporter_OpenStreetMap::getNeighboringNodes(NBNode* node, double maxDist, const std::set<NBNode*>& knownElevation) {
    std::map<NBNode*, std::pair<double, double> > result;
    std::set<NBNode*> visited;
    std::vector<NBNode*> open;
    open.push_back(node);
    result[node] = std::make_pair(0, 0);
    while (!open.empty()) {
        NBNode* n = open.back();
        open.pop_back();
        if (visited.count(n) != 0) {
            continue;
        }
        visited.insert(n);
        const EdgeVector& edges = n->getEdges();
        for (auto e : edges) {
            NBNode* s = nullptr;
            if (n->hasIncoming(e)) {
                s = e->getFromNode();
            } else {
                s = e->getToNode();
            }
            const double dist = result[n].first + e->getGeometry().length2D();
            const double speed = MAX2(e->getSpeed(), result[n].second);
            if (result.count(s) == 0) {
                result[s] = std::make_pair(dist, speed);
            } else {
                result[s] = std::make_pair(MIN2(dist, result[s].first), MAX2(speed, result[s].second));
            }
            if (dist < maxDist && knownElevation.count(s) == 0) {
                open.push_back(s);
            }
        }
    }
    result.erase(node);
    return result;
}


std::string
NIImporter_OpenStreetMap::usableType(const std::string& type, const std::string& id, NBTypeCont& tc) {
    if (tc.knows(type)) {
        return type;
    }
    if (myUnusableTypes.count(type) > 0) {
        return "";
    }
    if (myKnownCompoundTypes.count(type) > 0) {
        return myKnownCompoundTypes[type];
    }
    // this edge has a type which does not yet exist in the TypeContainer
    StringTokenizer tok = StringTokenizer(type, compoundTypeSeparator);
    std::vector<std::string> types;
    while (tok.hasNext()) {
        std::string t = tok.next();
        if (tc.knows(t)) {
            if (std::find(types.begin(), types.end(), t) == types.end()) {
                types.push_back(t);
            }
        } else if (tok.size() > 1) {
            if (!StringUtils::startsWith(t, "service.")) {
                WRITE_WARNINGF(TL("Discarding unknown compound '%' in type '%' (first occurrence for edge '%')."), t, type, id);
            }
        }
    }
    if (types.empty()) {
        if (!StringUtils::startsWith(type, "service.")) {
            WRITE_WARNINGF(TL("Discarding unusable type '%' (first occurrence for edge '%')."), type, id);
        }
        myUnusableTypes.insert(type);
        return "";
    }
    const std::string newType = joinToString(types, "|");
    if (tc.knows(newType)) {
        myKnownCompoundTypes[type] = newType;
        return newType;
    } else if (myKnownCompoundTypes.count(newType) > 0) {
        return myKnownCompoundTypes[newType];
    } else {
        // build a new type by merging all values
        int numLanes = 0;
        double maxSpeed = 0;
        int prio = 0;
        double width = NBEdge::UNSPECIFIED_WIDTH;
        double sidewalkWidth = NBEdge::UNSPECIFIED_WIDTH;
        double bikelaneWidth = NBEdge::UNSPECIFIED_WIDTH;
        bool defaultIsOneWay = true;
        SVCPermissions permissions = 0;
        LaneSpreadFunction spreadType = LaneSpreadFunction::RIGHT;
        bool discard = true;
        bool hadDiscard = false;
        for (auto& type2 : types) {
            if (!tc.getEdgeTypeShallBeDiscarded(type2)) {
                numLanes = MAX2(numLanes, tc.getEdgeTypeNumLanes(type2));
                maxSpeed = MAX2(maxSpeed, tc.getEdgeTypeSpeed(type2));
                prio = MAX2(prio, tc.getEdgeTypePriority(type2));
                defaultIsOneWay &= tc.getEdgeTypeIsOneWay(type2);
                //std::cout << "merging component " << type2 << " into type " << newType << " allows=" << getVehicleClassNames(tc.getPermissions(type2)) << " oneway=" << defaultIsOneWay << "\n";
                permissions |= tc.getEdgeTypePermissions(type2);
                spreadType = tc.getEdgeTypeSpreadType(type2);
                width = MAX2(width, tc.getEdgeTypeWidth(type2));
                sidewalkWidth = MAX2(sidewalkWidth, tc.getEdgeTypeSidewalkWidth(type2));
                bikelaneWidth = MAX2(bikelaneWidth, tc.getEdgeTypeBikeLaneWidth(type2));
                discard = false;
            } else {
                hadDiscard = true;
            }
        }
        if (hadDiscard && permissions == 0) {
            discard = true;
        }
        if (discard) {
            WRITE_WARNINGF(TL("Discarding compound type '%' (first occurrence for edge '%')."), newType, id);
            myUnusableTypes.insert(newType);
            return "";
        }
        if (width != NBEdge::UNSPECIFIED_WIDTH) {
            width = MAX2(width, SUMO_const_laneWidth);
        }
        // ensure pedestrians don't run into trains
        if (sidewalkWidth == NBEdge::UNSPECIFIED_WIDTH
                && (permissions & SVC_PEDESTRIAN) != 0
                && (permissions & SVC_RAIL_CLASSES) != 0) {
            //std::cout << "patching sidewalk for type '" << newType << "' which allows=" << getVehicleClassNames(permissions) << "\n";
            sidewalkWidth = OptionsCont::getOptions().getFloat("default.sidewalk-width");
        }

        WRITE_MESSAGEF(TL("Adding new type '%' (first occurrence for edge '%')."), type, id);
        tc.insertEdgeType(newType, numLanes, maxSpeed, prio, permissions, spreadType, width,
                          defaultIsOneWay, sidewalkWidth, bikelaneWidth, 0, 0, 0);
        for (auto& type3 : types) {
            if (!tc.getEdgeTypeShallBeDiscarded(type3)) {
                tc.copyEdgeTypeRestrictionsAndAttrs(type3, newType);
            }
        }
        myKnownCompoundTypes[type] = newType;
        return newType;
    }
}

void
NIImporter_OpenStreetMap::extendRailwayDistances(Edge* e, NBTypeCont& tc) {
    const std::string id = toString(e->id);
    std::string type = usableType(e->myHighWayType, id, tc);
    if (type != "" && isRailway(tc.getEdgeTypePermissions(type))) {
        std::vector<NIOSMNode*> nodes;
        std::vector<double> usablePositions;
        std::vector<int> usableIndex;
        for (long long int n : e->myCurrentNodes) {
            NIOSMNode* node = myOSMNodes[n];
            node->positionMeters = interpretDistance(node);
            if (node->positionMeters != std::numeric_limits<double>::max()) {
                usablePositions.push_back(node->positionMeters);
                usableIndex.push_back((int)nodes.size());
            }
            nodes.push_back(node);
        }
        if (usablePositions.size() == 0) {
            return;
        } else {
            bool forward = true;
            if (usablePositions.size() == 1) {
                WRITE_WARNINGF(TL("Ambiguous railway kilometrage direction for way '%' (assuming forward)"), id);
            } else {
                forward = usablePositions.front() < usablePositions.back();
            }
            // check for consistency
            for (int i = 1; i < (int)usablePositions.size(); i++) {
                if ((usablePositions[i - 1] < usablePositions[i]) != forward) {
                    WRITE_WARNINGF(TL("Inconsistent railway kilometrage direction for way '%': % (skipping)"), id, toString(usablePositions));
                    return;
                }
            }
            if (nodes.size() > usablePositions.size()) {
                // complete missing values
                PositionVector shape;
                for (NIOSMNode* node : nodes) {
                    shape.push_back(Position(node->lon, node->lat, 0));
                }
                if (!NBNetBuilder::transformCoordinates(shape)) {
                    return; // error will be given later
                }
                double sign = forward ? 1 : -1;
                // extend backward before first usable value
                for (int i = usableIndex.front() - 1; i >= 0; i--) {
                    nodes[i]->positionMeters = nodes[i + 1]->positionMeters - sign * shape[i].distanceTo2D(shape[i + 1]);
                }
                // extend forward
                for (int i = usableIndex.front() + 1; i < (int)nodes.size(); i++) {
                    if (nodes[i]->positionMeters == std::numeric_limits<double>::max()) {
                        nodes[i]->positionMeters = nodes[i - 1]->positionMeters + sign * shape[i].distanceTo2D(shape[i - 1]);
                    }
                }
                //std::cout << " way=" << id << " usable=" << toString(usablePositions) << "\n indices=" << toString(usableIndex)
                //    << " final:\n";
                //for (auto n : nodes) {
                //    std::cout << "    " << n->id << " " << n->positionMeters << " " << n->position<< "\n";
                //}
            }
        }
    }
}


double
NIImporter_OpenStreetMap::interpretDistance(NIOSMNode* node) {
    if (node->position.size() > 0) {
        try {
            if (StringUtils::startsWith(node->position, "mi:")) {
                return StringUtils::toDouble(node->position.substr(3)) * 1609.344; // meters per mile
            } else {
                return StringUtils::toDouble(node->position) * 1000;
            }
        } catch (...) {
            WRITE_WARNINGF(TL("Value of railway:position is not numeric ('%') in node '%'."), node->position, toString(node->id));
        }
    }
    return std::numeric_limits<double>::max();
}

SUMOVehicleClass
NIImporter_OpenStreetMap::interpretTransportType(const std::string& type, NIOSMNode* toSet) {
    SUMOVehicleClass result = SVC_IGNORING;
    if (type == "train") {
        result = SVC_RAIL;
    } else if (type == "subway") {
        result = SVC_SUBWAY;
    } else if (type == "aerialway") {
        result = SVC_CABLE_CAR;
    } else if (type == "light_rail" || type == "monorail") {
        result = SVC_RAIL_URBAN;
    } else if (type == "share_taxi") {
        result = SVC_TAXI;
    } else if (type == "minibus") {
        result = SVC_BUS;
    } else if (type == "trolleybus") {
        result = SVC_BUS;
    } else if (SumoVehicleClassStrings.hasString(type)) {
        result = SumoVehicleClassStrings.get(type);
    }
    std::string stop = "";
    if (result == SVC_TRAM) {
        stop = ".tram";
    } else if (result == SVC_BUS) {
        stop = ".bus";
    } else if (isRailway(result)) {
        stop = ".train";
    }
    if (toSet != nullptr && result != SVC_IGNORING) {
        toSet->permissions |= result;
        toSet->ptStopLength = OptionsCont::getOptions().getFloat("osm.stop-output.length" + stop);
    }
    return result;
}


void
NIImporter_OpenStreetMap::applyChangeProhibition(NBEdge* e, int changeProhibition) {
    bool multiLane = changeProhibition > 3;
    //std::cout << "applyChangeProhibition e=" << e->getID() << " changeProhibition=" << std::bitset<32>(changeProhibition) << " val=" << changeProhibition << "\n";
    for (int lane = 0; changeProhibition > 0 && lane < e->getNumLanes(); lane++) {
        int code = changeProhibition % 4; // only look at the last 2 bits
        SVCPermissions changeLeft = (code & CHANGE_NO_LEFT) == 0 ? SVCAll : (SVCPermissions)SVC_AUTHORITY;
        SVCPermissions changeRight = (code & CHANGE_NO_RIGHT) == 0 ? SVCAll : (SVCPermissions)SVC_AUTHORITY;
        e->setPermittedChanging(lane, changeLeft, changeRight);
        if (multiLane) {
            changeProhibition = changeProhibition >> 2;
        }
    }
}


void
NIImporter_OpenStreetMap::applyLaneUse(NBEdge* e, NIImporter_OpenStreetMap::Edge* nie, const bool forward) {
    if (myImportLaneAccess) {
        const int numLanes = e->getNumLanes();
        const bool lefthand = OptionsCont::getOptions().getBool("lefthand");
        const std::vector<bool>& designated = forward ? nie->myDesignatedLaneForward : nie->myDesignatedLaneBackward;
        const std::vector<SVCPermissions>& allowed = forward ? nie->myAllowedLaneForward : nie->myAllowedLaneBackward;
        const std::vector<SVCPermissions>& disallowed = forward ? nie->myDisallowedLaneForward : nie->myDisallowedLaneBackward;
        for (int lane = 0; lane < numLanes; lane++) {
            // laneUse stores from left to right
            const int i = lefthand ? lane : numLanes - 1 - lane;
            // Extra allowed SVCs for this lane or none if no info was present for the lane
            const SVCPermissions extraAllowed = i < (int)allowed.size() ? allowed[i] : (SVCPermissions)SVC_IGNORING;
            // Extra disallowed SVCs for this lane or none if no info was present for the lane
            const SVCPermissions extraDisallowed = i < (int)disallowed.size() ? disallowed[i] : (SVCPermissions)SVC_IGNORING;
            if (i < (int)designated.size() && designated[i]) {
                // if designated, delete all permissions
                e->setPermissions(SVC_IGNORING, lane);
                e->preferVehicleClass(lane, extraAllowed);
            }
            e->setPermissions((e->getPermissions(lane) | extraAllowed) & (~extraDisallowed), lane);
        }
    }
}

void
NIImporter_OpenStreetMap::mergeTurnSigns(std::vector<int>& signs, std::vector<int> signs2) {
    if (signs.empty()) {
        signs.insert(signs.begin(), signs2.begin(), signs2.end());
    } else {
        for (int i = 0; i < (int)MIN2(signs.size(), signs2.size()); i++) {
            signs[i] |= signs2[i];
        }
    }
}


void
NIImporter_OpenStreetMap::applyTurnSigns(NBEdge* e, const std::vector<int>& turnSigns) {
    if (myImportTurnSigns && turnSigns.size() > 0) {
        // no sidewalks and bike lanes have been added yet
        if ((int)turnSigns.size() == e->getNumLanes()) {
            //std::cout << "apply turnSigns for " << e->getID() << " turnSigns=" << toString(turnSigns) << "\n";
            for (int i = 0; i < (int)turnSigns.size(); i++) {
                // laneUse stores from left to right
                const int laneIndex = e->getNumLanes() - 1 - i;
                NBEdge::Lane& lane = e->getLaneStruct(laneIndex);
                lane.turnSigns = turnSigns[i];
            }
        } else {
            WRITE_WARNINGF(TL("Ignoring turn sign information for % lanes on edge % with % driving lanes"), turnSigns.size(), e->getID(), e->getNumLanes());
        }
    }
}


// ---------------------------------------------------------------------------
// definitions of NIImporter_OpenStreetMap::NodesHandler-methods
// ---------------------------------------------------------------------------
NIImporter_OpenStreetMap::NodesHandler::NodesHandler(std::map<long long int, NIOSMNode*>& toFill,
        std::set<NIOSMNode*, CompareNodes>& uniqueNodes, const OptionsCont& oc) :
    SUMOSAXHandler("osm - file"),
    myToFill(toFill),
    myCurrentNode(nullptr),
    myIsStation(false),
    myHierarchyLevel(0),
    myUniqueNodes(uniqueNodes),
    myImportElevation(oc.getBool("osm.elevation")),
    myDuplicateNodes(0),
    myOptionsCont(oc) {
    // init rail signal rules
    for (std::string kv : oc.getStringVector("osm.railsignals")) {
        if (kv == "DEFAULT") {
            myRailSignalRules.push_back("railway:signal:main=");
            myRailSignalRules.push_back("railway:signal:combined=");
        } else if (kv == "ALL") {
            myRailSignalRules.push_back("railway=signal");
        } else {
            myRailSignalRules.push_back("railway:signal:" + kv);
        }
    }
}


NIImporter_OpenStreetMap::NodesHandler::~NodesHandler() = default;

void
NIImporter_OpenStreetMap::NodesHandler::myStartElement(int element, const SUMOSAXAttributes& attrs) {
    ++myHierarchyLevel;
    if (element == SUMO_TAG_NODE) {
        bool ok = true;
        myLastNodeID = attrs.get<std::string>(SUMO_ATTR_ID, nullptr, ok);
        if (myHierarchyLevel != 2) {
            WRITE_ERROR("Node element on wrong XML hierarchy level (id='" + myLastNodeID +
                        "', level='" + toString(myHierarchyLevel) + "').");
            return;
        }
        const std::string& action = attrs.getOpt<std::string>(SUMO_ATTR_ACTION, myLastNodeID.c_str(), ok);
        if (action == "delete" || !ok) {
            return;
        }
        try {
            // we do not use attrs.get here to save some time on parsing
            const long long int id = StringUtils::toLong(myLastNodeID);
            myCurrentNode = nullptr;
            const auto insertionIt = myToFill.lower_bound(id);
            if (insertionIt == myToFill.end() || insertionIt->first != id) {
                // assume we are loading multiple files, so we won't report duplicate nodes
                const double tlon = attrs.get<double>(SUMO_ATTR_LON, myLastNodeID.c_str(), ok);
                const double tlat = attrs.get<double>(SUMO_ATTR_LAT, myLastNodeID.c_str(), ok);
                if (!ok) {
                    return;
                }
                myCurrentNode = new NIOSMNode(id, tlon, tlat);
                auto similarNode = myUniqueNodes.find(myCurrentNode);
                if (similarNode == myUniqueNodes.end()) {
                    myUniqueNodes.insert(myCurrentNode);
                } else {
                    delete myCurrentNode;
                    myCurrentNode = *similarNode;
                    myDuplicateNodes++;
                }
                myToFill.emplace_hint(insertionIt, id, myCurrentNode);
            }
        } catch (FormatException&) {
            WRITE_ERROR(TL("Attribute 'id' in the definition of a node is not of type long long int."));
            return;
        }
    }
    if (element == SUMO_TAG_TAG && myCurrentNode != nullptr) {
        if (myHierarchyLevel != 3) {
            WRITE_ERROR(TL("Tag element on wrong XML hierarchy level."));
            return;
        }
        bool ok = true;
        const std::string& key = attrs.get<std::string>(SUMO_ATTR_K, myLastNodeID.c_str(), ok, false);
        // we check whether the key is relevant (and we really need to transcode the value) to avoid hitting #1636
        if (key == "highway" || key == "ele" || key == "crossing" || key == "railway" || key == "public_transport"
                || key == "name" || key == "train" || key == "bus" || key == "tram" || key == "light_rail" || key == "subway" || key == "station" || key == "noexit"
                || key == "crossing:barrier"
                || key == "crossing:light"
                || key == "railway:ref"
                || StringUtils::startsWith(key, "railway:signal")
                || StringUtils::startsWith(key, "railway:position")
           ) {
            const std::string& value = attrs.get<std::string>(SUMO_ATTR_V, myLastNodeID.c_str(), ok, false);
            if (key == "highway" && value.find("traffic_signal") != std::string::npos) {
                myCurrentNode->tlsControlled = true;
            } else if (key == "crossing" && value.find("traffic_signals") != std::string::npos) {
                myCurrentNode->tlsControlled = true;
            } else if (key == "highway" && value.find("crossing") != std::string::npos) {
                myCurrentNode->pedestrianCrossing = true;
            } else if ((key == "noexit" && value == "yes")
                       || (key == "railway" && value == "buffer_stop")) {
                myCurrentNode->railwayBufferStop = true;
            } else if (key == "railway" && value.find("crossing") != std::string::npos) {
                myCurrentNode->railwayCrossing = true;
            } else if (key == "crossing:barrier") {
                myCurrentNode->setParameter("crossing:barrier", value);
            } else if (key == "crossing:light") {
                myCurrentNode->setParameter("crossing:light", value);
            } else if (key == "railway:signal:direction") {
                if (value == "both") {
                    myCurrentNode->myRailDirection = WAY_BOTH;
                } else if (value == "backward") {
                    myCurrentNode->myRailDirection = WAY_BACKWARD;
                } else if (value == "forward") {
                    myCurrentNode->myRailDirection = WAY_FORWARD;
                }
            } else if (StringUtils::startsWith(key, "railway:signal") || (key == "railway" && value == "signal")) {
                std::string kv = key + "=" + value;
                std::string kglob = key + "=";
                if ((std::find(myRailSignalRules.begin(), myRailSignalRules.end(), kv) != myRailSignalRules.end())
                        || (std::find(myRailSignalRules.begin(), myRailSignalRules.end(), kglob) != myRailSignalRules.end())) {
                    myCurrentNode->railwaySignal = true;
                }
            } else if (StringUtils::startsWith(key, "railway:position") && value.size() > myCurrentNode->position.size()) {
                // use the entry with the highest precision (more digits)
                myCurrentNode->position = value;
            } else if ((key == "public_transport" && value == "stop_position") ||
                       (key == "highway" && value == "bus_stop")) {
                myCurrentNode->ptStopPosition = true;
                if (myCurrentNode->ptStopLength == 0) {
                    // default length
                    myCurrentNode->ptStopLength = myOptionsCont.getFloat("osm.stop-output.length");
                }
            } else if (key == "name") {
                myCurrentNode->name = value;
            } else if (myImportElevation && key == "ele") {
                try {
                    const double elevation = StringUtils::parseDist(value);
                    if (std::isnan(elevation)) {
                        WRITE_WARNINGF(TL("Value of key '%' is invalid ('%') in node '%'."), key, value, myLastNodeID);
                    } else {
                        myCurrentNode->ele = elevation;
                    }
                } catch (...) {
                    WRITE_WARNINGF(TL("Value of key '%' is not numeric ('%') in node '%'."), key, value, myLastNodeID);
                }
            } else if (key == "station") {
                interpretTransportType(value, myCurrentNode);
                myIsStation = true;
            } else if (key == "railway:ref") {
                myRailwayRef = value;
            } else {
                // v="yes"
                interpretTransportType(key, myCurrentNode);
            }
        }
        if (myAllAttributes && (myExtraAttributes.count(key) != 0 || myExtraAttributes.size() == 0)) {
            const std::string info = "node=" + toString(myCurrentNode->id) + ", k=" + key;
            myCurrentNode->setParameter(key, attrs.get<std::string>(SUMO_ATTR_V, info.c_str(), ok, false));
        }
    }
}


void
NIImporter_OpenStreetMap::NodesHandler::myEndElement(int element) {
    if (element == SUMO_TAG_NODE && myHierarchyLevel == 2) {
        if (myIsStation && myRailwayRef != "") {
            myCurrentNode->setParameter("railway:ref", myRailwayRef);
        }
        myCurrentNode = nullptr;
        myIsStation = false;
        myRailwayRef = "";
    }
    --myHierarchyLevel;
}


// ---------------------------------------------------------------------------
// definitions of NIImporter_OpenStreetMap::EdgesHandler-methods
// ---------------------------------------------------------------------------
NIImporter_OpenStreetMap::EdgesHandler::EdgesHandler(
    const std::map<long long int, NIOSMNode*>& osmNodes,
    std::map<long long int, Edge*>& toFill, std::map<long long int, Edge*>& platformShapes,
    const NBTypeCont& tc):
    SUMOSAXHandler("osm - file"),
    myOSMNodes(osmNodes),
    myEdgeMap(toFill),
    myPlatformShapesMap(platformShapes),
    myTypeCont(tc) {

    const double unlimitedSpeed = OptionsCont::getOptions().getFloat("osm.speedlimit-none");

    mySpeedMap["nan"] = MAXSPEED_UNGIVEN;
    mySpeedMap["sign"] = MAXSPEED_UNGIVEN;
    mySpeedMap["signals"] = MAXSPEED_UNGIVEN;
    mySpeedMap["none"] = unlimitedSpeed;
    mySpeedMap["no"] = unlimitedSpeed;
    mySpeedMap["walk"] = 5. / 3.6;
    // https://wiki.openstreetmap.org/wiki/Key:source:maxspeed#Commonly_used_values
    mySpeedMap["AT:urban"] = 50. / 3.6;
    mySpeedMap["AT:rural"] = 100. / 3.6;
    mySpeedMap["AT:trunk"] = 100. / 3.6;
    mySpeedMap["AT:motorway"] = 130. / 3.6;
    mySpeedMap["AU:urban"] = 50. / 3.6;
    mySpeedMap["BE:urban"] = 50. / 3.6;
    mySpeedMap["BE:zone"] = 30. / 3.6;
    mySpeedMap["BE:motorway"] = 120. / 3.6;
    mySpeedMap["BE:zone30"] = 30. / 3.6;
    mySpeedMap["BE-VLG:rural"] = 70. / 3.6;
    mySpeedMap["BE-WAL:rural"] = 90. / 3.6;
    mySpeedMap["BE:school"] = 30. / 3.6;
    mySpeedMap["CZ:motorway"] = 130. / 3.6;
    mySpeedMap["CZ:trunk"] = 110. / 3.6;
    mySpeedMap["CZ:rural"] = 90. / 3.6;
    mySpeedMap["CZ:urban_motorway"] = 80. / 3.6;
    mySpeedMap["CZ:urban_trunk"] = 80. / 3.6;
    mySpeedMap["CZ:urban"] = 50. / 3.6;
    mySpeedMap["DE:motorway"] = unlimitedSpeed;
    mySpeedMap["DE:rural"] = 100. / 3.6;
    mySpeedMap["DE:urban"] = 50. / 3.6;
    mySpeedMap["DE:bicycle_road"] = 30. / 3.6;
    mySpeedMap["DK:motorway"] = 130. / 3.6;
    mySpeedMap["DK:rural"] = 80. / 3.6;
    mySpeedMap["DK:urban"] = 50. / 3.6;
    mySpeedMap["EE:urban"] = 50. / 3.6;
    mySpeedMap["EE:rural"] = 90. / 3.6;
    mySpeedMap["ES:urban"] = 50. / 3.6;
    mySpeedMap["ES:zone30"] = 30. / 3.6;
    mySpeedMap["FR:motorway"] = 130. / 3.6; // 110 (raining)
    mySpeedMap["FR:rural"] = 80. / 3.6;
    mySpeedMap["FR:urban"] = 50. / 3.6;
    mySpeedMap["FR:zone30"] = 30. / 3.6;
    mySpeedMap["HU:living_street"] = 20. / 3.6;
    mySpeedMap["HU:motorway"] = 130. / 3.6;
    mySpeedMap["HU:rural"] = 90. / 3.6;
    mySpeedMap["HU:trunk"] = 110. / 3.6;
    mySpeedMap["HU:urban"] = 50. / 3.6;
    mySpeedMap["IT:rural"] = 90. / 3.6;
    mySpeedMap["IT:motorway"] = 130. / 3.6;
    mySpeedMap["IT:urban"] = 50. / 3.6;
    mySpeedMap["JP:nsl"] = 60. / 3.6;
    mySpeedMap["JP:express"] = 100. / 3.6;
    mySpeedMap["LT:rural"] = 90. / 3.6;
    mySpeedMap["LT:urban"] = 50. / 3.6;
    mySpeedMap["NO:rural"] = 80. / 3.6;
    mySpeedMap["NO:urban"] = 50. / 3.6;
    mySpeedMap["ON:urban"] = 50. / 3.6;
    mySpeedMap["ON:rural"] = 80. / 3.6;
    mySpeedMap["PT:motorway"] = 120. / 3.6;
    mySpeedMap["PT:rural"] = 90. / 3.6;
    mySpeedMap["PT:trunk"] = 100. / 3.6;
    mySpeedMap["PT:urban"] = 50. / 3.6;
    mySpeedMap["RO:motorway"] = 130. / 3.6;
    mySpeedMap["RO:rural"] = 90. / 3.6;
    mySpeedMap["RO:trunk"] = 100. / 3.6;
    mySpeedMap["RO:urban"] = 50. / 3.6;
    mySpeedMap["RS:living_street"] = 30. / 3.6;
    mySpeedMap["RS:motorway"] = 130. / 3.6;
    mySpeedMap["RS:rural"] = 80. / 3.6;
    mySpeedMap["RS:trunk"] = 100. / 3.6;
    mySpeedMap["RS:urban"] = 50. / 3.6;
    mySpeedMap["RU:living_street"] = 20. / 3.6;
    mySpeedMap["RU:urban"] = 60. / 3.6;
    mySpeedMap["RU:rural"] = 90. / 3.6;
    mySpeedMap["RU:motorway"] = 110. / 3.6;
    const double seventy = StringUtils::parseSpeed("70mph");
    const double sixty = StringUtils::parseSpeed("60mph");
    mySpeedMap["GB:motorway"] = seventy;
    mySpeedMap["GB:nsl_dual"] = seventy;
    mySpeedMap["GB:nsl_single"] = sixty;
    mySpeedMap["UK:motorway"] = seventy;
    mySpeedMap["UK:nsl_dual"] = seventy;
    mySpeedMap["UK:nsl_single"] = sixty;
    mySpeedMap["UZ:living_street"] = 30. / 3.6;
    mySpeedMap["UZ:urban"] = 70. / 3.6;
    mySpeedMap["UZ:rural"] = 100. / 3.6;
    mySpeedMap["UZ:motorway"] = 110. / 3.6;
}

NIImporter_OpenStreetMap::EdgesHandler::~EdgesHandler() = default;

void
NIImporter_OpenStreetMap::EdgesHandler::myStartElement(int element, const SUMOSAXAttributes& attrs) {
    if (element == SUMO_TAG_WAY) {
        bool ok = true;
        const long long int id = attrs.get<long long int>(SUMO_ATTR_ID, nullptr, ok);
        const std::string& action = attrs.getOpt<std::string>(SUMO_ATTR_ACTION, nullptr, ok);
        if (action == "delete" || !ok) {
            myCurrentEdge = nullptr;
            return;
        }
        myCurrentEdge = new Edge(id);
    }
    // parse "nd" (node) elements
    if (element == SUMO_TAG_ND && myCurrentEdge != nullptr) {
        bool ok = true;
        long long int ref = attrs.get<long long int>(SUMO_ATTR_REF, nullptr, ok);
        if (ok) {
            auto node = myOSMNodes.find(ref);
            if (node == myOSMNodes.end()) {
                WRITE_WARNINGF(TL("The referenced geometry information (ref='%') is not known"), toString(ref));
                return;
            }

            ref = node->second->id; // node may have been substituted
            if (myCurrentEdge->myCurrentNodes.empty() ||
                    myCurrentEdge->myCurrentNodes.back() != ref) { // avoid consecutive duplicates
                myCurrentEdge->myCurrentNodes.push_back(ref);
            }

        }
    }
    if (element == SUMO_TAG_TAG && myCurrentEdge != nullptr) {
        bool ok = true;
        std::string key = attrs.get<std::string>(SUMO_ATTR_K, toString(myCurrentEdge->id).c_str(), ok, false);
        if (key.size() > 6 && StringUtils::startsWith(key, "busway:")) {
            // handle special busway keys
            const std::string buswaySpec = key.substr(7);
            key = "busway";
            if (buswaySpec == "right") {
                myCurrentEdge->myBuswayType = (WayType)(myCurrentEdge->myBuswayType | WAY_FORWARD);
            } else if (buswaySpec == "left") {
                myCurrentEdge->myBuswayType = (WayType)(myCurrentEdge->myBuswayType | WAY_BACKWARD);
            } else if (buswaySpec == "both") {
                myCurrentEdge->myBuswayType = (WayType)(myCurrentEdge->myBuswayType | WAY_BOTH);
            } else {
                key = "ignore";
            }
        }
        if (myAllAttributes && (myExtraAttributes.count(key) != 0 || myExtraAttributes.size() == 0)) {
            const std::string info = "way=" + toString(myCurrentEdge->id) + ", k=" + key;
            myCurrentEdge->setParameter(key, attrs.get<std::string>(SUMO_ATTR_V, info.c_str(), ok, false));
        }
        // we check whether the key is relevant (and we really need to transcode the value) to avoid hitting #1636
        if (!StringUtils::endsWith(key, "way")
                && !StringUtils::startsWith(key, "lanes")
                && key != "maxspeed" && key != "maxspeed:type"
                && key != "zone:maxspeed"
                && key != "maxspeed:forward" && key != "maxspeed:backward"
                && key != "junction" && key != "name" && key != "tracks" && key != "layer"
                && key != "route"
                && !StringUtils::startsWith(key, "cycleway")
                && !StringUtils::startsWith(key, "sidewalk")
                && key != "ref"
                && key != "highspeed"
                && !StringUtils::startsWith(key, "parking")
                && !StringUtils::startsWith(key, "change")
                && !StringUtils::startsWith(key, "vehicle:lanes")
                && key != "postal_code"
                && key != "railway:preferred_direction"
                && key != "railway:bidirectional"
                && key != "railway:track_ref"
                && key != "usage"
                && key != "access"
                && key != "emergency"
                && key != "service"
                && key != "electrified"
                && key != "segregated"
                && key != "bus"
                && key != "psv"
                && key != "foot"
                && key != "bicycle"
                && key != "oneway:bicycle"
                && key != "oneway:bus"
                && key != "oneway:psv"
                && key != "bus:lanes"
                && key != "bus:lanes:forward"
                && key != "bus:lanes:backward"
                && key != "psv:lanes"
                && key != "psv:lanes:forward"
                && key != "psv:lanes:backward"
                && key != "bicycle:lanes"
                && key != "bicycle:lanes:forward"
                && key != "bicycle:lanes:backward"
                && !StringUtils::startsWith(key, "width")
                && !(StringUtils::startsWith(key, "turn:") && key.find(":lanes") != std::string::npos)
                && key != "public_transport") {
            return;
        }
        const std::string value = attrs.get<std::string>(SUMO_ATTR_V, toString(myCurrentEdge->id).c_str(), ok, false);

        if (key == "highway" || key == "railway" || key == "waterway" || StringUtils::startsWith(key, "cycleway")
                || key == "busway" || key == "route" || StringUtils::startsWith(key, "sidewalk") || key == "highspeed"
                || key == "aeroway" || key == "aerialway" || key == "usage" || key == "service") {
            // build type id
            if (key != "highway" || myTypeCont.knows(key + "." + value)) {
                myCurrentEdge->myCurrentIsRoad = true;
            }
            // special cycleway stuff https://wiki.openstreetmap.org/wiki/Key:cycleway
            if (key == "cycleway") {
                if (value == "no" || value == "none" || value == "separate") {
                    myCurrentEdge->myCyclewayType = WAY_NONE;
                } else if (value == "both") {
                    myCurrentEdge->myCyclewayType = WAY_BOTH;
                } else if (value == "right") {
                    myCurrentEdge->myCyclewayType = WAY_FORWARD;
                } else if (value == "left") {
                    myCurrentEdge->myCyclewayType = WAY_BACKWARD;
                } else if (value == "opposite_track") {
                    myCurrentEdge->myCyclewayType = WAY_BACKWARD;
                } else if (value == "opposite_lane") {
                    myCurrentEdge->myCyclewayType = WAY_BACKWARD;
                } else if (value == "opposite") {
                    // according to the wiki ref above, this should rather be a bidi lane, see #13438
                    myCurrentEdge->myCyclewayType = WAY_BACKWARD;
                }
            }
            if (key == "cycleway:left") {
                if (myCurrentEdge->myCyclewayType == WAY_UNKNOWN) {
                    myCurrentEdge->myCyclewayType = WAY_NONE;
                }
                if (value == "yes" || value == "lane" || value == "track") {
                    myCurrentEdge->myCyclewayType = (WayType)(myCurrentEdge->myCyclewayType | WAY_BACKWARD);
                }
                key = "cycleway"; // for type adaption
            }
            if (key == "cycleway:right") {
                if (myCurrentEdge->myCyclewayType == WAY_UNKNOWN) {
                    myCurrentEdge->myCyclewayType = WAY_NONE;
                }
                if (value == "yes" || value == "lane" || value == "track") {
                    myCurrentEdge->myCyclewayType = (WayType)(myCurrentEdge->myCyclewayType | WAY_FORWARD);
                }
                key = "cycleway"; // for type adaption
            }
            if (key == "cycleway:both") {
                if (myCurrentEdge->myCyclewayType == WAY_UNKNOWN) {
                    if (value == "no" || value == "none" || value == "separate") {
                        myCurrentEdge->myCyclewayType = WAY_NONE;
                    }
                    if (value == "yes" || value == "lane" || value == "track") {
                        myCurrentEdge->myCyclewayType = WAY_BOTH;
                    }
                }
                key = "cycleway"; // for type adaption
            }
            if (key == "cycleway" && value != "lane" && value != "track" && value != "opposite_track" && value != "opposite_lane") {
                // typemap covers only the lane and track cases
                return;
            }
            if (StringUtils::startsWith(key, "cycleway:")) {
                // no need to extend the type id for other cycleway sub tags
                return;
            }
            // special sidewalk stuff
            if (key == "sidewalk") {
                if (value == "no" || value == "none" || value == "separate") {
                    myCurrentEdge->mySidewalkType = WAY_NONE;
                } else if (value == "both") {
                    myCurrentEdge->mySidewalkType = WAY_BOTH;
                } else if (value == "right") {
                    myCurrentEdge->mySidewalkType = WAY_FORWARD;
                } else if (value == "left") {
                    myCurrentEdge->mySidewalkType = WAY_BACKWARD;
                }
            }
            if (key == "sidewalk:left") {
                if (myCurrentEdge->mySidewalkType == WAY_UNKNOWN) {
                    myCurrentEdge->mySidewalkType = WAY_NONE;
                }
                if (value == "yes") {
                    myCurrentEdge->mySidewalkType = (WayType)(myCurrentEdge->mySidewalkType | WAY_BACKWARD);
                }
            }
            if (key == "sidewalk:right") {
                if (myCurrentEdge->mySidewalkType == WAY_UNKNOWN) {
                    myCurrentEdge->mySidewalkType = WAY_NONE;
                }
                if (value == "yes") {
                    myCurrentEdge->mySidewalkType = (WayType)(myCurrentEdge->mySidewalkType | WAY_FORWARD);
                }
            }
            if (key == "sidewalk:both") {
                if (myCurrentEdge->mySidewalkType == WAY_UNKNOWN) {
                    if (value == "no" || value == "none" || value == "separate") {
                        myCurrentEdge->mySidewalkType = WAY_NONE;
                    }
                    if (value == "yes") {
                        myCurrentEdge->mySidewalkType = WAY_BOTH;
                    }
                }
            }
            if (StringUtils::startsWith(key, "sidewalk")) {
                // no need to extend the type id
                return;
            }
            // special busway stuff
            if (key == "busway") {
                if (value == "no") {
                    return;
                }
                if (value == "opposite_track") {
                    myCurrentEdge->myBuswayType = WAY_BACKWARD;
                } else if (value == "opposite_lane") {
                    myCurrentEdge->myBuswayType = WAY_BACKWARD;
                }
                // no need to extend the type id
                return;
            }
            std::string singleTypeID = key + "." + value;
            if (key == "highspeed") {
                if (value == "no") {
                    return;
                }
                singleTypeID = "railway.highspeed";
            }
            addType(singleTypeID);

        } else if (key == "bus" || key == "psv") {
            // 'psv' includes taxi in the UK but not in germany
            try {
                if (StringUtils::toBool(value)) {
                    myCurrentEdge->myExtraAllowed |= SVC_BUS;
                    addType(key);
                } else {
                    myCurrentEdge->myExtraDisallowed |= SVC_BUS;
                }
            } catch (const BoolFormatException&) {
                myCurrentEdge->myExtraAllowed |= SVC_BUS;
                addType(key);
            }
        } else if (key == "emergency") {
            try {
                if (StringUtils::toBool(value)) {
                    myCurrentEdge->myExtraAllowed |= SVC_AUTHORITY | SVC_EMERGENCY;
                }
            } catch (const BoolFormatException&) {
                myCurrentEdge->myExtraAllowed |= SVC_AUTHORITY | SVC_EMERGENCY;
            }
        } else if (key == "access") {
            if (value == "no") {
                myCurrentEdge->myExtraDisallowed |= ~(SVC_PUBLIC_CLASSES | SVC_EMERGENCY | SVC_AUTHORITY);
            }
        } else if (StringUtils::startsWith(key, "width:lanes")) {
            try {
                const std::vector<std::string> values = StringTokenizer(value, "|").getVector();
                std::vector<double> widthLanes;
                for (std::string width : values) {
                    const double parsedWidth = width == "" ? -1 : StringUtils::parseDist(width);
                    widthLanes.push_back(parsedWidth);
                }

                if (key == "width:lanes" || key == "width:lanes:forward") {
                    myCurrentEdge->myWidthLanesForward = widthLanes;
                } else if (key == "width:lanes:backward") {
                    myCurrentEdge->myWidthLanesBackward = widthLanes;
                } else {
                    WRITE_WARNINGF(TL("Using default lane width for edge '%' as key '%' could not be parsed."), toString(myCurrentEdge->id), key);
                }
            } catch (const NumberFormatException&) {
                WRITE_WARNINGF(TL("Using default lane width for edge '%' as value '%' could not be parsed."), toString(myCurrentEdge->id), value);
            }
        } else if (key == "width") {
            try {
                myCurrentEdge->myWidth = StringUtils::parseDist(value);
            } catch (const NumberFormatException&) {
                WRITE_WARNINGF(TL("Using default width for edge '%' as value '%' could not be parsed."), toString(myCurrentEdge->id), value);
            }
        } else if (key == "foot") {
            if (value == "use_sidepath" || value == "no") {
                myCurrentEdge->myExtraDisallowed |= SVC_PEDESTRIAN;
            } else if (value == "yes" || value == "designated" || value == "permissive") {
                myCurrentEdge->myExtraAllowed |= SVC_PEDESTRIAN;
            }
        } else if (key == "bicycle") {
            if (value == "use_sidepath" || value == "no") {
                myCurrentEdge->myExtraDisallowed |= SVC_BICYCLE;
            } else if (value == "yes" || value == "designated" || value == "permissive") {
                myCurrentEdge->myExtraAllowed |= SVC_BICYCLE;
            }
        } else if (key == "oneway:bicycle") {
            myCurrentEdge->myExtraTags["oneway:bicycle"] = value;
        } else if (key == "oneway:bus" || key == "oneway:psv") {
            if (value == "no") {
                // need to add a bus way in reversed direction of way
                myCurrentEdge->myBuswayType = WAY_BACKWARD;
            }
        } else if (key == "lanes") {
            try {
                myCurrentEdge->myNoLanes = StringUtils::toInt(value);
            } catch (NumberFormatException&) {
                // might be a list of values
                StringTokenizer st(value, ";", true);
                std::vector<std::string> list = st.getVector();
                if (list.size() >= 2) {
                    int minLanes = std::numeric_limits<int>::max();
                    try {
                        for (auto& i : list) {
                            const int numLanes = StringUtils::toInt(StringUtils::prune(i));
                            minLanes = MIN2(minLanes, numLanes);
                        }
                        myCurrentEdge->myNoLanes = minLanes;
                        WRITE_WARNINGF(TL("Using minimum lane number from list (%) for edge '%'."), value, toString(myCurrentEdge->id));
                    } catch (NumberFormatException&) {
                        WRITE_WARNINGF(TL("Value of key '%' is not numeric ('%') in edge '%'."), key, value, myCurrentEdge->id);
                    }
                }
            } catch (EmptyData&) {
                WRITE_WARNINGF(TL("Value of key '%' is not numeric ('%') in edge '%'."), key, value, myCurrentEdge->id);
            }
        } else if (key == "lanes:forward") {
            try {
                const int numLanes = StringUtils::toInt(value);
                if (myCurrentEdge->myNoLanesForward < 0 && myCurrentEdge->myNoLanes < 0) {
                    // fix lane count in case only lanes:forward and lanes:backward are set
                    myCurrentEdge->myNoLanes = numLanes - myCurrentEdge->myNoLanesForward;
                }
                myCurrentEdge->myNoLanesForward = numLanes;
            } catch (...) {
                WRITE_WARNINGF(TL("Value of key '%' is not numeric ('%') in edge '%'."), key, value, myCurrentEdge->id);
            }
        } else if (key == "lanes:backward") {
            try {
                const int numLanes = StringUtils::toInt(value);
                if (myCurrentEdge->myNoLanesForward > 0 && myCurrentEdge->myNoLanes < 0) {
                    // fix lane count in case only lanes:forward and lanes:backward are set
                    myCurrentEdge->myNoLanes = numLanes + myCurrentEdge->myNoLanesForward;
                }
                // denote backwards count with a negative sign
                myCurrentEdge->myNoLanesForward = -numLanes;
            } catch (...) {
                WRITE_WARNINGF(TL("Value of key '%' is not numeric ('%') in edge '%'."), key, value, myCurrentEdge->id);
            }
        } else if (myCurrentEdge->myMaxSpeed == MAXSPEED_UNGIVEN &&
                   (key == "maxspeed" || key == "maxspeed:type" || key == "maxspeed:forward" || key == "zone:maxspeed")) {
            // both 'maxspeed' and 'maxspeed:type' may be given so we must take care not to overwrite an already seen value
            myCurrentEdge->myMaxSpeed = interpretSpeed(key, value);
        } else if (key == "maxspeed:backward" && myCurrentEdge->myMaxSpeedBackward == MAXSPEED_UNGIVEN) {
            myCurrentEdge->myMaxSpeedBackward = interpretSpeed(key, value);
        } else if (key == "junction") {
            if ((value == "roundabout" || value == "circular") && myCurrentEdge->myIsOneWay.empty()) {
                myCurrentEdge->myIsOneWay = "yes";
            }
            if (value == "roundabout") {
                myCurrentEdge->myAmInRoundabout = true;
            }
        } else if (key == "oneway") {
            myCurrentEdge->myIsOneWay = value;
        } else if (key == "name") {
            myCurrentEdge->streetName = value;
        } else if (key == "ref") {
            myCurrentEdge->ref = value;
            myCurrentEdge->setParameter("ref", value);
        } else if (key == "layer") {
            try {
                myCurrentEdge->myLayer = StringUtils::toInt(value);
            } catch (...) {
                WRITE_WARNINGF(TL("Value of key '%' is not numeric ('%') in edge '%'."), key, value, myCurrentEdge->id);
            }
        } else if (key == "tracks") {
            try {
                if (StringUtils::toInt(value) == 1) {
                    myCurrentEdge->myIsOneWay = "true";
                } else {
                    WRITE_WARNINGF(TL("Ignoring track count % for edge '%'."), value, myCurrentEdge->id);
                }
            } catch (...) {
                WRITE_WARNINGF(TL("Value of key '%' is not numeric ('%') in edge '%'."), key, value, myCurrentEdge->id);
            }
        } else if (key == "railway:preferred_direction") {
            if (value == "both") {
                myCurrentEdge->myRailDirection = WAY_BOTH;
            } else if (myCurrentEdge->myRailDirection == WAY_UNKNOWN) {
                if (value == "backward") {
                    myCurrentEdge->myRailDirection = WAY_BACKWARD;
                } else if (value == "forward") {
                    myCurrentEdge->myRailDirection = WAY_FORWARD;
                }
            }
        } else if (key == "railway:bidirectional") {
            if (value == "regular") {
                myCurrentEdge->myRailDirection = WAY_BOTH;
            }
        } else if (key == "electrified" || key == "segregated") {
            if (value != "no") {
                myCurrentEdge->myExtraTags[key] = value;
            }
        } else if (key == "railway:track_ref") {
            myCurrentEdge->setParameter(key, value);
        } else if (key == "public_transport" && value == "platform") {
            myCurrentEdge->myExtraTags["platform"] = "yes";
        } else if ((key == "parking:both" || key == "parking:lane:both") && !StringUtils::startsWith(value, "no")) {
            myCurrentEdge->myParkingType |= PARKING_BOTH;
        } else if ((key == "parking:left" || key == "parking:lane:left") && !StringUtils::startsWith(value, "no")) {
            myCurrentEdge->myParkingType |= PARKING_LEFT;
        } else if ((key == "parking:right" || key == "parking:lane:right") && !StringUtils::startsWith(value, "no")) {
            myCurrentEdge->myParkingType |= PARKING_RIGHT;
        } else if (key == "change" || key == "change:lanes") {
            myCurrentEdge->myChangeForward = myCurrentEdge->myChangeBackward = interpretChangeType(value);
        } else if (key == "change:forward" || key == "change:lanes:forward") {
            myCurrentEdge->myChangeForward = interpretChangeType(value);
        } else if (key == "change:backward" || key == "change:lanes:backward") {
            myCurrentEdge->myChangeBackward = interpretChangeType(value);
        } else if (key == "vehicle:lanes" || key == "vehicle:lanes:forward") {
            interpretLaneUse(value, SVC_PASSENGER, true);
            interpretLaneUse(value, SVC_PRIVATE, true);
        } else if (key == "vehicle:lanes:backward") {
            interpretLaneUse(value, SVC_PASSENGER, false);
            interpretLaneUse(value, SVC_PRIVATE, false);
        } else if (key == "bus:lanes" || key == "bus:lanes:forward") {
            interpretLaneUse(value, SVC_BUS, true);
        } else if (key == "bus:lanes:backward") {
            interpretLaneUse(value, SVC_BUS, false);
        } else if (key == "psv:lanes" || key == "psv:lanes:forward") {
            interpretLaneUse(value, SVC_BUS, true);
            interpretLaneUse(value, SVC_TAXI, true);
        } else if (key == "psv:lanes:backward") {
            interpretLaneUse(value, SVC_BUS, false);
            interpretLaneUse(value, SVC_TAXI, false);
        } else if (key == "bicycle:lanes" || key == "bicycle:lanes:forward") {
            interpretLaneUse(value, SVC_BICYCLE, true);
        } else if (key == "bicycle:lanes:backward") {
            interpretLaneUse(value, SVC_BICYCLE, false);
        } else if (StringUtils::startsWith(key, "turn:") && key.find(":lanes") != std::string::npos) {
            int shift = 0;
            // use the first 8 bit to encode permitted directions for all classes
            // and the successive 8 bit blocks for selected classes
            if (StringUtils::startsWith(key, "turn:bus") || StringUtils::startsWith(key, "turn:psv:")) {
                shift = NBEdge::TURN_SIGN_SHIFT_BUS;
            } else if (StringUtils::startsWith(key, "turn:taxi")) {
                shift = NBEdge::TURN_SIGN_SHIFT_TAXI;
            } else if (StringUtils::startsWith(key, "turn:bicycle")) {
                shift = NBEdge::TURN_SIGN_SHIFT_BICYCLE;
            }
            const std::vector<std::string> values = StringTokenizer(value, "|").getVector();
            std::vector<int> turnCodes;
            for (std::string codeList : values) {
                const std::vector<std::string> codes = StringTokenizer(codeList, ";").getVector();
                int turnCode = 0;
                if (codes.size() == 0) {
                    turnCode = (int)LinkDirection::STRAIGHT;
                }
                for (std::string code : codes) {
                    if (code == "" || code == "none" || code == "through") {
                        turnCode |= (int)LinkDirection::STRAIGHT << shift ;
                    } else if (code == "left" || code == "sharp_left") {
                        turnCode |= (int)LinkDirection::LEFT << shift;
                    } else if (code == "right" || code == "sharp_right") {
                        turnCode |= (int)LinkDirection::RIGHT << shift;
                    } else if (code == "slight_left") {
                        turnCode |= (int)LinkDirection::PARTLEFT << shift;
                    } else if (code == "slight_right") {
                        turnCode |= (int)LinkDirection::PARTRIGHT << shift;
                    } else if (code == "reverse") {
                        turnCode |= (int)LinkDirection::TURN << shift;
                    } else if (code == "merge_to_left" || code == "merge_to_right") {
                        turnCode |= (int)LinkDirection::NODIR << shift;
                    }
                }
                turnCodes.push_back(turnCode);
            }
            if (StringUtils::endsWith(key, "lanes") || StringUtils::endsWith(key, "lanes:forward")) {
                mergeTurnSigns(myCurrentEdge->myTurnSignsForward, turnCodes);
            } else if (StringUtils::endsWith(key, "lanes:backward")) {
                mergeTurnSigns(myCurrentEdge->myTurnSignsBackward, turnCodes);
            } else if (StringUtils::endsWith(key, "lanes:both_ways")) {
                mergeTurnSigns(myCurrentEdge->myTurnSignsForward, turnCodes);
                mergeTurnSigns(myCurrentEdge->myTurnSignsBackward, turnCodes);
            }
        }
    }
}


void
NIImporter_OpenStreetMap::EdgesHandler::addType(const std::string& singleTypeID) {
    // special case: never build compound type for highspeed rail
    if (!myCurrentEdge->myHighWayType.empty() && singleTypeID != "railway.highspeed") {
        if (myCurrentEdge->myHighWayType == "railway.highspeed") {
            return;
        }
        // osm-ways may be used by more than one mode (eg railway.tram + highway.residential. this is relevant for multimodal traffic)
        // we create a new type for this kind of situation which must then be resolved in insertEdge()
        std::vector<std::string> types = StringTokenizer(myCurrentEdge->myHighWayType,
                                         compoundTypeSeparator).getVector();
        types.push_back(singleTypeID);
        myCurrentEdge->myHighWayType = joinToStringSorting(types, compoundTypeSeparator);
    } else {
        myCurrentEdge->myHighWayType = singleTypeID;
    }
}


double
NIImporter_OpenStreetMap::EdgesHandler::interpretSpeed(const std::string& key, std::string value) {
    if (mySpeedMap.find(value) != mySpeedMap.end()) {
        return mySpeedMap[value];
    } else {
        // handle symbolic names of the form DE:30 / DE:zone30
        if (value.size() > 3 && value[2] == ':') {
            if (value.substr(3, 4) == "zone") {
                value = value.substr(7);
            } else {
                value = value.substr(3);
            }
        }
        try {
            return StringUtils::parseSpeed(value);
        } catch (...) {
            WRITE_WARNING("Value of key '" + key + "' is not numeric ('" + value + "') in edge '" +
                          toString(myCurrentEdge->id) + "'.");
            return MAXSPEED_UNGIVEN;
        }
    }
}


int
NIImporter_OpenStreetMap::EdgesHandler::interpretChangeType(const std::string& value) const {
    int result = 0;
    const std::vector<std::string> values = StringTokenizer(value, "|").getVector();
    for (const std::string& val : values) {
        if (val == "no") {
            result += CHANGE_NO;
        } else if (val == "not_left") {
            result += CHANGE_NO_LEFT;
        } else if (val == "not_right") {
            result += CHANGE_NO_RIGHT;
        }
        result = result << 2;
    }
    // last shift was superfluous
    result = result >> 2;

    if (values.size() > 1) {
        result += 2 << 29; // mark multi-value input
    }
    //std::cout << " way=" << myCurrentEdge->id << " value=" << value << " result=" << std::bitset<32>(result) << "\n";
    return result;
}


void
NIImporter_OpenStreetMap::EdgesHandler::interpretLaneUse(const std::string& value, SUMOVehicleClass svc, const bool forward) const {
    const std::vector<std::string> values = StringTokenizer(value, "|").getVector();
    std::vector<bool>& designated = forward ? myCurrentEdge->myDesignatedLaneForward : myCurrentEdge->myDesignatedLaneBackward;
    std::vector<SVCPermissions>& allowed = forward ? myCurrentEdge->myAllowedLaneForward : myCurrentEdge->myAllowedLaneBackward;
    std::vector<SVCPermissions>& disallowed = forward ? myCurrentEdge->myDisallowedLaneForward : myCurrentEdge->myDisallowedLaneBackward;
    designated.resize(MAX2(designated.size(), values.size()), false);
    allowed.resize(MAX2(allowed.size(), values.size()), SVC_IGNORING);
    disallowed.resize(MAX2(disallowed.size(), values.size()), SVC_IGNORING);
    int i = 0;
    for (const std::string& val : values) {
        if (val == "yes" || val == "permissive") {
            allowed[i] |= svc;
        } else if (val == "lane" || val == "designated") {
            allowed[i] |= svc;
            designated[i] = true;
        } else if (val == "no") {
            disallowed[i] |= svc;
        } else {
            WRITE_WARNINGF(TL("Unknown lane use specifier '%' ignored for way '%'"), val, myCurrentEdge->id);
        }
        i++;
    }
}


void
NIImporter_OpenStreetMap::EdgesHandler::myEndElement(int element) {
    if (element == SUMO_TAG_WAY && myCurrentEdge != nullptr) {
        if (myCurrentEdge->myCurrentIsRoad) {
            const auto insertionIt = myEdgeMap.lower_bound(myCurrentEdge->id);
            if (insertionIt == myEdgeMap.end() || insertionIt->first != myCurrentEdge->id) {
                // assume we are loading multiple files, so we won't report duplicate edges
                myEdgeMap.emplace_hint(insertionIt, myCurrentEdge->id, myCurrentEdge);
            } else {
                delete myCurrentEdge;
            }
        } else if (myCurrentEdge->myExtraTags.count("platform") != 0) {
            const auto insertionIt = myPlatformShapesMap.lower_bound(myCurrentEdge->id);
            if (insertionIt == myPlatformShapesMap.end() || insertionIt->first != myCurrentEdge->id) {
                // assume we are loading multiple files, so we won't report duplicate platforms
                myPlatformShapesMap.emplace_hint(insertionIt, myCurrentEdge->id, myCurrentEdge);
            } else {
                delete myCurrentEdge;
            }
        } else {
            delete myCurrentEdge;
        }
        myCurrentEdge = nullptr;
    }
}


// ---------------------------------------------------------------------------
// definitions of NIImporter_OpenStreetMap::RelationHandler-methods
// ---------------------------------------------------------------------------
NIImporter_OpenStreetMap::RelationHandler::RelationHandler(
    const std::map<long long int, NIOSMNode*>& osmNodes,
    const std::map<long long int, Edge*>& osmEdges, NBPTStopCont* nbptStopCont,
    const std::map<long long int, Edge*>& platformShapes,
    NBPTLineCont* nbptLineCont,
    const OptionsCont& oc) :
    SUMOSAXHandler("osm - file"),
    myOSMNodes(osmNodes),
    myOSMEdges(osmEdges),
    myPlatformShapes(platformShapes),
    myNBPTStopCont(nbptStopCont),
    myNBPTLineCont(nbptLineCont),
    myOptionsCont(oc) {
    resetValues();
}


NIImporter_OpenStreetMap::RelationHandler::~RelationHandler() = default;


void
NIImporter_OpenStreetMap::RelationHandler::resetValues() {
    myCurrentRelation = INVALID_ID;
    myIsRestriction = false;
    myRestrictionException = SVC_IGNORING;
    myFromWay = INVALID_ID;
    myToWay = INVALID_ID;
    myViaNode = INVALID_ID;
    myViaWay = INVALID_ID;
    myStation = INVALID_ID;
    myRestrictionType = RestrictionType::UNKNOWN;
    myPlatforms.clear();
    myStops.clear();
    myPlatformStops.clear();
    myWays.clear();
    myIsStopArea = false;
    myIsRoute = false;
    myPTRouteType = "";
    myRouteColor.setValid(false);
}


void
NIImporter_OpenStreetMap::RelationHandler::myStartElement(int element, const SUMOSAXAttributes& attrs) {
    if (element == SUMO_TAG_RELATION) {
        bool ok = true;
        myCurrentRelation = attrs.get<long long int>(SUMO_ATTR_ID, nullptr, ok);
        const std::string& action = attrs.getOpt<std::string>(SUMO_ATTR_ACTION, nullptr, ok);
        if (action == "delete" || !ok) {
            myCurrentRelation = INVALID_ID;
        }
        myName = "";
        myRef = "";
        myInterval = -1;
        myNightService = "";
        return;
    }
    if (myCurrentRelation == INVALID_ID) {
        return;
    }
    if (element == SUMO_TAG_MEMBER) {
        bool ok = true;
        std::string role = attrs.hasAttribute("role") ? attrs.getStringSecure("role", "") : "";
        const long long int ref = attrs.get<long long int>(SUMO_ATTR_REF, nullptr, ok);
        if (role == "via") {
            // u-turns for divided ways may be given with 2 via-nodes or 1 via-way
            std::string memberType = attrs.get<std::string>(SUMO_ATTR_TYPE, nullptr, ok);
            if (memberType == "way" && checkEdgeRef(ref)) {
                myViaWay = ref;
            } else if (memberType == "node") {
                if (myOSMNodes.find(ref) != myOSMNodes.end()) {
                    myViaNode = ref;
                } else {
                    WRITE_WARNINGF(TL("No node found for reference '%' in relation '%'."), toString(ref), toString(myCurrentRelation));
                }
            }
        } else if (role == "from" && checkEdgeRef(ref)) {
            myFromWay = ref;
        } else if (role == "to" && checkEdgeRef(ref)) {
            myToWay = ref;
        } else if (StringUtils::startsWith(role, "stop")) {
            // permit _entry_only and _exit_only variants
            myStops.push_back(ref);
        } else if (StringUtils::startsWith(role, "platform")) {
            // permit _entry_only and _exit_only variants
            std::string memberType = attrs.get<std::string>(SUMO_ATTR_TYPE, nullptr, ok);
            if (memberType == "way") {
                const std::map<long long int, NIImporter_OpenStreetMap::Edge*>::const_iterator& wayIt = myPlatformShapes.find(ref);
                if (wayIt != myPlatformShapes.end()) {
                    NIIPTPlatform platform;
                    platform.isWay = true;
                    platform.ref = ref;
                    myPlatforms.push_back(platform);
                }
            } else if (memberType == "node") {
                // myIsStopArea may not be set yet
                myStops.push_back(ref);
                myPlatformStops.insert(ref);
                NIIPTPlatform platform;
                platform.isWay = false;
                platform.ref = ref;
                myPlatforms.push_back(platform);
            }

        } else if (role == "station") {
            myStation = ref;
        } else if (role.empty()) {
            std::string memberType = attrs.get<std::string>(SUMO_ATTR_TYPE, nullptr, ok);
            if (memberType == "way") {
                myWays.push_back(ref);
            } else if (memberType == "node") {
                auto it = myOSMNodes.find(ref);
                if (it != myOSMNodes.end() && it->second->hasParameter("railway:ref")) {
                    myStation = ref;
                } else {
                    myStops.push_back(ref);
                }
            }
        }
        return;
    }
    // parse values
    if (element == SUMO_TAG_TAG) {
        bool ok = true;
        std::string key = attrs.get<std::string>(SUMO_ATTR_K, toString(myCurrentRelation).c_str(), ok, false);
        // we check whether the key is relevant (and we really need to transcode the value) to avoid hitting #1636
        if (key == "type" || key == "restriction") {
            std::string value = attrs.get<std::string>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, false);
            if (key == "type" && value == "restriction") {
                myIsRestriction = true;
                return;
            }
            if (key == "type" && value == "route") {
                myIsRoute = true;
                return;
            }
            if (key == "restriction") {
                // @note: the 'right/left/straight' part is ignored since the information is
                // redundantly encoded in the 'from', 'to' and 'via' members
                if (value.substr(0, 5) == "only_") {
                    myRestrictionType = RestrictionType::ONLY;
                } else if (value.substr(0, 3) == "no_") {
                    myRestrictionType = RestrictionType::NO;
                } else {
                    WRITE_WARNINGF(TL("Found unknown restriction type '%' in relation '%'"), value, toString(myCurrentRelation));
                }
                return;
            }
        } else if (key == "except") {
            std::string value = attrs.get<std::string>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, false);
            for (const std::string& v : StringTokenizer(value, ";").getVector()) {
                if (v == "psv") {
                    myRestrictionException |= SVC_BUS;
                } else if (v == "bicycle") {
                    myRestrictionException |= SVC_BICYCLE;
                } else if (v == "hgv") {
                    myRestrictionException |= SVC_TRUCK | SVC_TRAILER;
                } else if (v == "motorcar") {
                    myRestrictionException |= SVC_PASSENGER | SVC_TAXI;
                } else if (v == "emergency") {
                    myRestrictionException |= SVC_EMERGENCY;
                }
            }
        } else if (key == "public_transport") {
            std::string value = attrs.get<std::string>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, false);
            if (value == "stop_area") {
                myIsStopArea = true;
            }
        } else if (key == "route") {
            std::string value = attrs.get<std::string>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, false);
            if (value == "train" || value == "subway" || value == "light_rail" || value == "monorail" || value == "tram" || value == "bus"
                    || value == "trolleybus" || value == "aerialway" || value == "ferry" || value == "share_taxi" || value == "minibus") {
                myPTRouteType = value;
            }

        } else if (key == "name") {
            myName = attrs.get<std::string>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, false);
        } else if (key == "colour") {
            std::string value = attrs.get<std::string>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, false);
            try {
                myRouteColor = RGBColor::parseColor(value);
            } catch (...) {
                WRITE_WARNINGF(TL("Invalid color value '%' in relation %"), value, myCurrentRelation);
            }
        } else if (key == "ref") {
            myRef = attrs.get<std::string>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, false);
        } else if (key == "interval" || key == "headway") {
            myInterval = attrs.get<int>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, false);
        } else if (key == "by_night") {
            myNightService = attrs.get<std::string>(SUMO_ATTR_V, toString(myCurrentRelation).c_str(), ok, false);
        }
    }
}


bool
NIImporter_OpenStreetMap::RelationHandler::checkEdgeRef(long long int ref) const {
    if (myOSMEdges.find(ref) != myOSMEdges.end()) {
        return true;
    }
    WRITE_WARNINGF(TL("No way found for reference '%' in relation '%'"), toString(ref), toString(myCurrentRelation));
    return false;
}


void
NIImporter_OpenStreetMap::RelationHandler::myEndElement(int element) {
    if (element == SUMO_TAG_RELATION) {
        if (myIsRestriction) {
            assert(myCurrentRelation != INVALID_ID);
            bool ok = true;
            if (myRestrictionType == RestrictionType::UNKNOWN) {
                WRITE_WARNINGF(TL("Ignoring restriction relation '%' with unknown type."), toString(myCurrentRelation));
                ok = false;
            }
            if (myFromWay == INVALID_ID) {
                WRITE_WARNINGF(TL("Ignoring restriction relation '%' with unknown from-way."), toString(myCurrentRelation));
                ok = false;
            }
            if (myToWay == INVALID_ID) {
                WRITE_WARNINGF(TL("Ignoring restriction relation '%' with unknown to-way."), toString(myCurrentRelation));
                ok = false;
            }
            if (myViaNode == INVALID_ID && myViaWay == INVALID_ID) {
                WRITE_WARNINGF(TL("Ignoring restriction relation '%' with unknown via."), toString(myCurrentRelation));
                ok = false;
            }
            if (ok && !applyRestriction()) {
                WRITE_WARNINGF(TL("Ignoring restriction relation '%'."), toString(myCurrentRelation));
            }
        } else if (myIsStopArea) {
            for (long long ref : myStops) {
                myStopAreas[ref] = myCurrentRelation;
                if (myOSMNodes.find(ref) == myOSMNodes.end()) {
                    //WRITE_WARNING(
                    //    "Referenced node: '" + toString(ref) + "' in relation: '" + toString(myCurrentRelation)
                    //    + "' does not exist. Probably OSM file is incomplete.");
                    continue;
                }

                NIOSMNode* n = myOSMNodes.find(ref)->second;
                std::shared_ptr<NBPTStop> ptStop = myNBPTStopCont->get(toString(n->id));
                if (ptStop == nullptr) {
                    //WRITE_WARNING(
                    //    "Relation '" + toString(myCurrentRelation) + "' refers to a non existing pt stop at node: '"
                    //    + toString(n->id) + "'. Probably OSM file is incomplete.");
                    continue;
                }
                for (NIIPTPlatform& myPlatform : myPlatforms) {
                    if (myPlatform.isWay) {
                        assert(myPlatformShapes.find(myPlatform.ref) != myPlatformShapes.end()); //already tested earlier
                        Edge* edge = (*myPlatformShapes.find(myPlatform.ref)).second;
                        if (edge->myCurrentNodes.size() > 1 && edge->myCurrentNodes[0] == *(edge->myCurrentNodes.end() - 1)) {
                            WRITE_WARNINGF(TL("Platform '%' in relation: '%' is given as polygon, which currently is not supported."), myPlatform.ref, myCurrentRelation);
                            continue;

                        }
                        PositionVector p;
                        for (auto nodeRef : edge->myCurrentNodes) {
                            if (myOSMNodes.find(nodeRef) == myOSMNodes.end()) {
                                //WRITE_WARNING(
                                //    "Referenced node: '" + toString(ref) + "' in relation: '" + toString(myCurrentRelation)
                                //    + "' does not exist. Probably OSM file is incomplete.");
                                continue;
                            }
                            NIOSMNode* pNode = myOSMNodes.find(nodeRef)->second;
                            Position pNodePos(pNode->lon, pNode->lat, pNode->ele);
                            if (!NBNetBuilder::transformCoordinate(pNodePos)) {
                                WRITE_ERRORF("Unable to project coordinates for node '%'.", pNode->id);
                                continue;
                            }
                            p.push_back(pNodePos);
                        }
                        if (p.size() == 0) {
                            WRITE_WARNINGF(TL("Referenced platform: '%' in relation: '%' is corrupt. Probably OSM file is incomplete."),
                                           toString(myPlatform.ref), toString(myCurrentRelation));
                            continue;
                        }
                        NBPTPlatform platform(p[(int)p.size() / 2], p.length());
                        ptStop->addPlatformCand(platform);
                    } else {
                        if (myOSMNodes.find(myPlatform.ref) == myOSMNodes.end()) {
                            //WRITE_WARNING(
                            //    "Referenced node: '" + toString(ref) + "' in relation: '" + toString(myCurrentRelation)
                            //    + "' does not exist. Probably OSM file is incomplete.");
                            continue;
                        }
                        NIOSMNode* pNode = myOSMNodes.find(myPlatform.ref)->second;
                        Position platformPos(pNode->lon, pNode->lat, pNode->ele);
                        if (!NBNetBuilder::transformCoordinate(platformPos)) {
                            WRITE_ERRORF("Unable to project coordinates for node '%'.", pNode->id);
                        }
                        NBPTPlatform platform(platformPos, myOptionsCont.getFloat("osm.stop-output.length"));
                        ptStop->addPlatformCand(platform);

                    }
                }
                ptStop->setIsMultipleStopPositions(myStops.size() > 1, myCurrentRelation);
                if (myStation != INVALID_ID) {
                    const auto& nodeIt = myOSMNodes.find(myStation);
                    if (nodeIt != myOSMNodes.end()) {
                        NIOSMNode* station = nodeIt->second;
                        if (station != nullptr) {
                            if (station->hasParameter("railway:ref")) {
                                ptStop->setParameter("stationRef", station->getParameter("railway:ref"));
                            }
                        }
                    }
                }
            }
        } else if (myPTRouteType != "" && myIsRoute) {
            NBPTLine* ptLine = new NBPTLine(toString(myCurrentRelation), myName, myPTRouteType, myRef, myInterval, myNightService,
                                            interpretTransportType(myPTRouteType), myRouteColor);
            int consecutiveGap = false;
            int missingBefore = 0;
            int missingAfter = 0;
            for (long long ref : myStops) {
                const auto& nodeIt = myOSMNodes.find(ref);
                if (nodeIt == myOSMNodes.end()) {
                    if (ptLine->getStops().empty()) {
                        missingBefore++;
                    } else {
                        missingAfter++;
                        consecutiveGap++;
                    }
                    continue;
                }
                // give some slack for single missing stops
                if (consecutiveGap > 1) {
                    WRITE_WARNINGF(TL("PT line '%' in relation % has a gap of % stops, only keeping first part."), myName, myCurrentRelation, consecutiveGap);
                    missingAfter = (int)myStops.size() - missingBefore - (int)ptLine->getStops().size();
                    break;
                }
                // reset gap
                consecutiveGap = 0;

                const NIOSMNode* const n = nodeIt->second;
                std::shared_ptr<NBPTStop> ptStop = myNBPTStopCont->get(toString(n->id));
                if (ptStop == nullptr) {
                    // loose stop, which must later be mapped onto a line way
                    Position ptPos(n->lon, n->lat, n->ele);
                    if (!NBNetBuilder::transformCoordinate(ptPos)) {
                        WRITE_ERRORF("Unable to project coordinates for node '%'.", n->id);
                    }
                    const SumoXMLTag stopElement = isRailway(n->permissions) ? SUMO_TAG_TRAIN_STOP : SUMO_TAG_BUS_STOP;
                    ptStop = std::make_shared<NBPTStop>(stopElement, toString(n->id), ptPos, "", "", n->ptStopLength, n->name, n->permissions);
                    myNBPTStopCont->insert(ptStop);
                    if (myStopAreas.count(n->id)) {
                        ptStop->setIsMultipleStopPositions(false, myStopAreas[n->id]);
                    }
                    if (myPlatformStops.count(n->id) > 0) {
                        ptStop->setIsPlatform();
                    }
                }
                ptLine->addPTStop(ptStop);
            }
            for (long long& myWay : myWays) {
                auto entr = myOSMEdges.find(myWay);
                if (entr != myOSMEdges.end()) {
                    Edge* edge = entr->second;
                    for (long long& myCurrentNode : edge->myCurrentNodes) {
                        ptLine->addWayNode(myWay, myCurrentNode);
                    }
                }
            }
            ptLine->setNumOfStops((int)myStops.size(), missingBefore, missingAfter);
            if (ptLine->getStops().empty()) {
                WRITE_WARNINGF(TL("PT line in relation % with no stops ignored. Probably OSM file is incomplete."), myCurrentRelation);
                delete ptLine;
                resetValues();
                return;
            }
            if (!myNBPTLineCont->insert(ptLine)) {
                WRITE_WARNINGF(TL("Ignoring duplicate PT line '%'."), myCurrentRelation);
                delete ptLine;
            }
        }
        // other relations might use similar subelements so reset in any case
        resetValues();
    }
}

bool
NIImporter_OpenStreetMap::RelationHandler::applyRestriction() const {
    // since OSM ways are bidirectional we need the via to figure out which direction was meant
    if (myViaNode != INVALID_ID) {
        NBNode* viaNode = myOSMNodes.find(myViaNode)->second->node;
        if (viaNode == nullptr) {
            WRITE_WARNINGF(TL("Via-node '%' was not instantiated"), toString(myViaNode));
            return false;
        }
        NBEdge* from = findEdgeRef(myFromWay, viaNode->getIncomingEdges());
        NBEdge* to = findEdgeRef(myToWay, viaNode->getOutgoingEdges());
        if (from == nullptr) {
            WRITE_WARNINGF(TL("from-edge '%' of restriction relation could not be determined"), toString(myFromWay));
            return false;
        }
        if (to == nullptr) {
            WRITE_WARNINGF(TL("to-edge '%' of restriction relation could not be determined"), toString(myToWay));
            return false;
        }
        if (myRestrictionType == RestrictionType::ONLY) {
            from->addEdge2EdgeConnection(to, true);
            // make sure that these connections remain disabled even if network
            // modifications (ramps.guess) reset existing connections
            for (NBEdge* cand : from->getToNode()->getOutgoingEdges()) {
                if (!from->isConnectedTo(cand)) {
                    if (myRestrictionException == SVC_IGNORING) {
                        from->removeFromConnections(cand, -1, -1, true);
                    } else {
                        from->addEdge2EdgeConnection(cand, true, myRestrictionException);
                    }
                }
            }
        } else {
            if (myRestrictionException == SVC_IGNORING) {
                from->removeFromConnections(to, -1, -1, true);
            } else {
                from->addEdge2EdgeConnection(to, true, myRestrictionException);
                for (NBEdge* cand : from->getToNode()->getOutgoingEdges()) {
                    if (!from->isConnectedTo(cand)) {
                        from->addEdge2EdgeConnection(cand, true);
                    }
                }
            }
        }
    } else {
        // XXX interpreting via-ways or via-node lists not yet implemented
        WRITE_WARNINGF(TL("direction of restriction relation could not be determined%"), "");
        return false;
    }
    return true;
}

NBEdge*
NIImporter_OpenStreetMap::RelationHandler::findEdgeRef(long long int wayRef,
        const std::vector<NBEdge*>& candidates) const {
    const std::string prefix = toString(wayRef);
    const std::string backPrefix = "-" + prefix;
    NBEdge* result = nullptr;
    int found = 0;
    for (auto candidate : candidates) {
        if ((candidate->getID().substr(0, prefix.size()) == prefix) ||
                (candidate->getID().substr(0, backPrefix.size()) == backPrefix)) {
            result = candidate;
            found++;
        }
    }
    if (found > 1) {
        WRITE_WARNINGF(TL("Ambiguous way reference '%' in restriction relation"), prefix);
        result = nullptr;
    }
    return result;
}


/****************************************************************************/
