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
/// @file    RONet.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Sept 2002
///
// The router's network representation
/****************************************************************************/
#include <config.h>

#include <algorithm>
#include <utils/router/RouteCostCalculator.h>
#include <utils/vehicle/SUMOVTypeParameter.h>
#include <utils/router/SUMOAbstractRouter.h>
#include <utils/options/OptionsCont.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/ToString.h>
#include <utils/common/StringUtils.h>
#include <utils/common/RandHelper.h>
#include <utils/common/SUMOVehicleClass.h>
#include <utils/iodevices/OutputDevice.h>
#include "ROEdge.h"
#include "ROLane.h"
#include "RONode.h"
#include "ROPerson.h"
#include "RORoute.h"
#include "RORouteDef.h"
#include "ROVehicle.h"
#include "ROAbstractEdgeBuilder.h"
#include "RONet.h"


// ===========================================================================
// static member definitions
// ===========================================================================
RONet* RONet::myInstance = nullptr;


// ===========================================================================
// method definitions
// ===========================================================================
RONet*
RONet::getInstance(void) {
    if (myInstance != nullptr) {
        return myInstance;
    }
    throw ProcessError(TL("A network was not yet constructed."));
}


RONet::RONet() :
    myVehicleTypes(), myDefaultVTypeMayBeDeleted(true),
    myDefaultPedTypeMayBeDeleted(true),
    myDefaultBikeTypeMayBeDeleted(true),
    myDefaultTaxiTypeMayBeDeleted(true),
    myDefaultRailTypeMayBeDeleted(true),
    myHaveActiveFlows(true),
    myRoutesOutput(nullptr), myRouteAlternativesOutput(nullptr), myTypesOutput(nullptr),
    myReadRouteNo(0), myDiscardedRouteNo(0), myWrittenRouteNo(0),
    myHavePermissions(false),
    myHaveParamRestrictions(false),
    myNumInternalEdges(0),
    myErrorHandler(OptionsCont::getOptions().exists("ignore-errors")
                   && OptionsCont::getOptions().getBool("ignore-errors") ? MsgHandler::getWarningInstance() : MsgHandler::getErrorInstance()),
    myKeepVTypeDist(OptionsCont::getOptions().exists("keep-vtype-distributions")
                    && OptionsCont::getOptions().getBool("keep-vtype-distributions")),
    myDoPTRouting(!OptionsCont::getOptions().exists("ptline-routing")
                  || OptionsCont::getOptions().getBool("ptline-routing")),
    myKeepFlows(OptionsCont::getOptions().exists("keep-flows")
                && OptionsCont::getOptions().getBool("keep-flows")),
    myHasBidiEdges(false),
    myMaxTraveltime(OptionsCont::getOptions().exists("max-traveltime") ? STEPS2TIME(string2time(OptionsCont::getOptions().getString("max-traveltime"))) : -1) {
    if (myInstance != nullptr) {
        throw ProcessError(TL("A network was already constructed."));
    }
    SUMOVTypeParameter* type = new SUMOVTypeParameter(DEFAULT_VTYPE_ID, SVC_PASSENGER);
    type->onlyReferenced = true;
    myVehicleTypes.add(type->id, type);

    SUMOVTypeParameter* defPedType = new SUMOVTypeParameter(DEFAULT_PEDTYPE_ID, SVC_PEDESTRIAN);
    defPedType->onlyReferenced = true;
    defPedType->parametersSet |= VTYPEPARS_VEHICLECLASS_SET;
    myVehicleTypes.add(defPedType->id, defPedType);

    SUMOVTypeParameter* defBikeType = new SUMOVTypeParameter(DEFAULT_BIKETYPE_ID, SVC_BICYCLE);
    defBikeType->onlyReferenced = true;
    defBikeType->parametersSet |= VTYPEPARS_VEHICLECLASS_SET;
    myVehicleTypes.add(defBikeType->id, defBikeType);

    SUMOVTypeParameter* defTaxiType = new SUMOVTypeParameter(DEFAULT_TAXITYPE_ID, SVC_TAXI);
    defTaxiType->onlyReferenced = true;
    defTaxiType->parametersSet |= VTYPEPARS_VEHICLECLASS_SET;
    myVehicleTypes.add(defTaxiType->id, defTaxiType);

    SUMOVTypeParameter* defRailType = new SUMOVTypeParameter(DEFAULT_RAILTYPE_ID, SVC_RAIL);
    defRailType->onlyReferenced = true;
    defRailType->parametersSet |= VTYPEPARS_VEHICLECLASS_SET;
    myVehicleTypes.add(defRailType->id, defRailType);

    myInstance = this;
}


RONet::~RONet() {
    for (const auto& routables : myRoutables) {
        for (RORoutable* const r : routables.second) {
            const ROVehicle* const veh = dynamic_cast<const ROVehicle*>(r);
            // delete routes and the vehicle
            if (veh != nullptr && veh->getRouteDefinition()->getID()[0] == '!') {
                if (!myRoutes.remove(veh->getRouteDefinition()->getID())) {
                    delete veh->getRouteDefinition();
                }
            }
            delete r;
        }
    }
    for (const RORoutable* const r : myPTVehicles) {
        const ROVehicle* const veh = dynamic_cast<const ROVehicle*>(r);
        // delete routes and the vehicle
        if (veh != nullptr && veh->getRouteDefinition()->getID()[0] == '!') {
            if (!myRoutes.remove(veh->getRouteDefinition()->getID())) {
                delete veh->getRouteDefinition();
            }
        }
        delete r;
    }
    myRoutables.clear();
    for (const auto& vTypeDist : myVTypeDistDict) {
        delete vTypeDist.second;
    }
}


void
RONet::addSpeedRestriction(const std::string& id, const SUMOVehicleClass svc, const double speed) {
    mySpeedRestrictions[id][svc] = speed;
}


double
RONet::getPreference(const std::string& routingType, const SUMOVTypeParameter& pars) const {
    if (gRoutingPreferences) {
        auto it = myVTypePreferences.find(pars.id);
        if (it != myVTypePreferences.end()) {
            auto it2 = it->second.find(routingType);
            if (it2 != it->second.end()) {
                return it2->second;
            }
        }
        auto it3 = myVClassPreferences.find(pars.vehicleClass);
        if (it3 != myVClassPreferences.end()) {
            auto it4 = it3->second.find(routingType);
            if (it4 != it3->second.end()) {
                return it4->second;
            }
        }
        // fallback to generel preferences
        it = myVTypePreferences.find("");
        if (it != myVTypePreferences.end()) {
            auto it2 = it->second.find(routingType);
            if (it2 != it->second.end()) {
                return it2->second;
            }
        }
    }
    return 1;
}


void
RONet::addPreference(const std::string& routingType, SUMOVehicleClass svc, double prio) {
    myVClassPreferences[svc][routingType] = prio;
    gRoutingPreferences = true;
}


void
RONet::addPreference(const std::string& routingType, std::string vType, double prio) {
    myVTypePreferences[vType][routingType] = prio;
    gRoutingPreferences = true;
}



const std::map<SUMOVehicleClass, double>*
RONet::getRestrictions(const std::string& id) const {
    std::map<std::string, std::map<SUMOVehicleClass, double> >::const_iterator i = mySpeedRestrictions.find(id);
    if (i == mySpeedRestrictions.end()) {
        return nullptr;
    }
    return &i->second;
}


bool
RONet::addEdge(ROEdge* edge) {
    if (!myEdges.add(edge->getID(), edge)) {
        WRITE_ERRORF(TL("The edge '%' occurs at least twice."), edge->getID());
        delete edge;
        return false;
    }
    if (edge->isInternal()) {
        myNumInternalEdges += 1;
    }
    return true;
}


bool
RONet::addDistrict(const std::string id, ROEdge* source, ROEdge* sink) {
    if (myDistricts.count(id) > 0) {
        WRITE_ERRORF(TL("The TAZ '%' occurs at least twice."), id);
        delete source;
        delete sink;
        return false;
    }
    sink->setFunction(SumoXMLEdgeFunc::CONNECTOR);
    if (!addEdge(sink)) {
        return false;
    }
    source->setFunction(SumoXMLEdgeFunc::CONNECTOR);
    if (!addEdge(source)) {
        return false;
    }
    sink->setOtherTazConnector(source);
    source->setOtherTazConnector(sink);
    myDistricts[id] = std::make_pair(std::vector<std::string>(), std::vector<std::string>());
    return true;
}


bool
RONet::addDistrictEdge(const std::string tazID, const std::string edgeID, const bool isSource) {
    if (myDistricts.count(tazID) == 0) {
        WRITE_ERRORF(TL("The TAZ '%' is unknown."), tazID);
        return false;
    }
    ROEdge* edge = getEdge(edgeID);
    if (edge == nullptr) {
        WRITE_ERRORF(TL("The edge '%' for TAZ '%' is unknown."), edgeID, tazID);
        return false;
    }
    if (isSource) {
        getEdge(tazID + "-source")->addSuccessor(edge);
        myDistricts[tazID].first.push_back(edgeID);
    } else {
        edge->addSuccessor(getEdge(tazID + "-sink"));
        myDistricts[tazID].second.push_back(edgeID);
    }
    return true;
}


void
RONet::addJunctionTaz(ROAbstractEdgeBuilder& eb) {
    for (auto item : myNodes) {
        const std::string tazID = item.first;
        if (myDistricts.count(tazID) != 0) {
            WRITE_WARNINGF(TL("A TAZ with id '%' already exists. Not building junction TAZ."), tazID);
            continue;
        }
        const std::string sourceID = tazID + "-source";
        const std::string sinkID = tazID + "-sink";
        // sink must be added before source
        ROEdge* sink = eb.buildEdge(sinkID, nullptr, nullptr, 0, "", "");
        ROEdge* source = eb.buildEdge(sourceID, nullptr, nullptr, 0, "", "");
        sink->setOtherTazConnector(source);
        source->setOtherTazConnector(sink);
        if (!addDistrict(tazID, source, sink)) {
            continue;
        }
        auto& district = myDistricts[tazID];
        const RONode* junction = item.second;
        for (const ROEdge* edge : junction->getIncoming()) {
            if (!edge->isInternal()) {
                const_cast<ROEdge*>(edge)->addSuccessor(sink);
                district.second.push_back(edge->getID());
            }
        }
        for (const ROEdge* edge : junction->getOutgoing()) {
            if (!edge->isInternal()) {
                source->addSuccessor(const_cast<ROEdge*>(edge));
                district.first.push_back(edge->getID());
            }
        }
    }
}


void
RONet::setBidiEdges(const std::map<ROEdge*, std::string>& bidiMap) {
    for (const auto& item : bidiMap) {
        ROEdge* bidi = myEdges.get(item.second);
        if (bidi == nullptr) {
            WRITE_ERRORF(TL("The bidi edge '%' is not known."), item.second);
        }
        item.first->setBidiEdge(bidi);
        myHasBidiEdges = true;
    }
}


void
RONet::addNode(RONode* node) {
    if (!myNodes.add(node->getID(), node)) {
        WRITE_ERRORF(TL("The node '%' occurs at least twice."), node->getID());
        delete node;
    }
}


void
RONet::addStoppingPlace(const std::string& id, const SumoXMLTag category, SUMOVehicleParameter::Stop* stop) {
    if (!myStoppingPlaces[category == SUMO_TAG_TRAIN_STOP ? SUMO_TAG_BUS_STOP : category].add(id, stop)) {
        WRITE_ERRORF(TL("The % '%' occurs at least twice."), toString(category), id);
        delete stop;
    }
}


bool
RONet::addRouteDef(RORouteDef* def) {
    return myRoutes.add(def->getID(), def);
}


void
RONet::openOutput(const OptionsCont& options) {
    if (options.isSet("output-file") && options.getString("output-file") != "") {
        myRoutesOutput = &OutputDevice::getDevice(options.getString("output-file"));
        if (myRoutesOutput->isNull()) {
            myRoutesOutput = nullptr;
        } else {
            myRoutesOutput->writeXMLHeader("routes", "routes_file.xsd");
        }
    }
    if (options.exists("alternatives-output") && options.isSet("alternatives-output")
            && !(options.exists("write-trips") && options.getBool("write-trips"))) {
        myRouteAlternativesOutput = &OutputDevice::getDevice(options.getString("alternatives-output"));
        if (myRouteAlternativesOutput->isNull()) {
            myRouteAlternativesOutput = nullptr;
        } else {
            myRouteAlternativesOutput->writeXMLHeader("routes", "routes_file.xsd");
        }
    }
    if (options.isSet("vtype-output")) {
        myTypesOutput = &OutputDevice::getDevice(options.getString("vtype-output"));
        myTypesOutput->writeXMLHeader("routes", "routes_file.xsd");
    }
}


void
RONet::writeIntermodal(const OptionsCont& options, ROIntermodalRouter& router) const {
    if (options.exists("intermodal-network-output") && options.isSet("intermodal-network-output")) {
        OutputDevice::createDeviceByOption("intermodal-network-output", "intermodal");
        router.writeNetwork(OutputDevice::getDevice(options.getString("intermodal-network-output")));
    }
    if (options.exists("intermodal-weight-output") && options.isSet("intermodal-weight-output")) {
        OutputDevice::createDeviceByOption("intermodal-weight-output", "weights", "meandata_file.xsd");
        OutputDevice& dev = OutputDevice::getDeviceByOption("intermodal-weight-output");
        dev.openTag(SUMO_TAG_INTERVAL);
        dev.writeAttr(SUMO_ATTR_ID, "intermodalweights");
        dev.writeAttr(SUMO_ATTR_BEGIN, 0);
        dev.writeAttr(SUMO_ATTR_END, SUMOTime_MAX);
        router.writeWeights(dev);
        dev.closeTag();
    }
}


void
RONet::cleanup() {
    // end writing
    if (myRoutesOutput != nullptr) {
        myRoutesOutput->close();
    }
    // only if opened
    if (myRouteAlternativesOutput != nullptr) {
        myRouteAlternativesOutput->close();
    }
    // only if opened
    if (myTypesOutput != nullptr) {
        myTypesOutput->close();
    }
    RouteCostCalculator<RORoute, ROEdge, ROVehicle>::cleanup();
#ifdef HAVE_FOX
    if (myThreadPool.size() > 0) {
        myThreadPool.clear();
    }
#endif
}



SUMOVTypeParameter*
RONet::getVehicleTypeSecure(const std::string& id) {
    // check whether the type was already known
    SUMOVTypeParameter* type = myVehicleTypes.get(id);
    if (id == DEFAULT_VTYPE_ID) {
        myDefaultVTypeMayBeDeleted = false;
    } else if (id == DEFAULT_PEDTYPE_ID) {
        myDefaultPedTypeMayBeDeleted = false;
    } else if (id == DEFAULT_BIKETYPE_ID) {
        myDefaultBikeTypeMayBeDeleted = false;
    } else if (id == DEFAULT_TAXITYPE_ID) {
        myDefaultTaxiTypeMayBeDeleted = false;
    } else if (id == DEFAULT_RAILTYPE_ID) {
        myDefaultRailTypeMayBeDeleted = false;
    }
    if (type != nullptr) {
        return type;
    }
    VTypeDistDictType::iterator it2 = myVTypeDistDict.find(id);
    if (it2 != myVTypeDistDict.end()) {
        return it2->second->get();
    }
    if (id == "") {
        // ok, no vehicle type or an unknown type was given within the user input
        //  return the default type
        myDefaultVTypeMayBeDeleted = false;
        return myVehicleTypes.get(DEFAULT_VTYPE_ID);
    }
    return type;
}


bool
RONet::checkVType(const std::string& id) {
    if (id == DEFAULT_VTYPE_ID) {
        if (myDefaultVTypeMayBeDeleted) {
            myVehicleTypes.remove(id);
            myDefaultVTypeMayBeDeleted = false;
        } else {
            return false;
        }
    } else if (id == DEFAULT_PEDTYPE_ID) {
        if (myDefaultPedTypeMayBeDeleted) {
            myVehicleTypes.remove(id);
            myDefaultPedTypeMayBeDeleted = false;
        } else {
            return false;
        }
    } else if (id == DEFAULT_BIKETYPE_ID) {
        if (myDefaultBikeTypeMayBeDeleted) {
            myVehicleTypes.remove(id);
            myDefaultBikeTypeMayBeDeleted = false;
        } else {
            return false;
        }
    } else if (id == DEFAULT_TAXITYPE_ID) {
        if (myDefaultTaxiTypeMayBeDeleted) {
            myVehicleTypes.remove(id);
            myDefaultTaxiTypeMayBeDeleted = false;
        } else {
            return false;
        }
    } else if (id == DEFAULT_RAILTYPE_ID) {
        if (myDefaultRailTypeMayBeDeleted) {
            myVehicleTypes.remove(id);
            myDefaultRailTypeMayBeDeleted = false;
        } else {
            return false;
        }
    } else {
        if (myVehicleTypes.get(id) != 0 || myVTypeDistDict.find(id) != myVTypeDistDict.end()) {
            return false;
        }
    }
    return true;
}


bool
RONet::addVehicleType(SUMOVTypeParameter* type) {
    if (checkVType(type->id)) {
        myVehicleTypes.add(type->id, type);
    } else {
        WRITE_ERRORF(TL("The vehicle type '%' occurs at least twice."), type->id);
        delete type;
        return false;
    }
    return true;
}


bool
RONet::addVTypeDistribution(const std::string& id, RandomDistributor<SUMOVTypeParameter*>* vehTypeDistribution) {
    if (checkVType(id)) {
        myVTypeDistDict[id] = vehTypeDistribution;
        return true;
    }
    delete vehTypeDistribution;
    return false;
}


bool
RONet::addVehicle(const std::string& id, ROVehicle* veh) {
    if (myVehIDs.find(id) == myVehIDs.end()) {
        myVehIDs[id] = veh->getParameter().departProcedure == DepartDefinition::TRIGGERED ? -1 : veh->getDepartureTime();

        if (veh->isPublicTransport()) {
            if (!veh->isPartOfFlow()) {
                myPTVehicles.push_back(veh);
            }
            if (!myDoPTRouting) {
                return true;
            }
        }
        myRoutables[veh->getDepart()].push_back(veh);
        return true;
    }
    WRITE_ERRORF(TL("Another vehicle with the id '%' exists."), id);
    delete veh;
    return false;
}


bool
RONet::knowsVehicle(const std::string& id) const {
    return myVehIDs.find(id) != myVehIDs.end();
}

SUMOTime
RONet::getDeparture(const std::string& vehID) const {
    auto it = myVehIDs.find(vehID);
    if (it != myVehIDs.end()) {
        return it->second;
    } else {
        throw ProcessError(TLF("Requesting departure time for unknown vehicle '%'", vehID));
    }
}


bool
RONet::addFlow(SUMOVehicleParameter* flow, const bool randomize) {
    if (randomize && flow->repetitionOffset >= 0) {
        myDepartures[flow->id].reserve(flow->repetitionNumber);
        for (int i = 0; i < flow->repetitionNumber; ++i) {
            myDepartures[flow->id].push_back(flow->depart + RandHelper::rand(flow->repetitionNumber * flow->repetitionOffset));
        }
        std::sort(myDepartures[flow->id].begin(), myDepartures[flow->id].end());
        std::reverse(myDepartures[flow->id].begin(), myDepartures[flow->id].end());
    } else if (flow->repetitionOffset < 0) {
        // init poisson flow (but only the timing)
        flow->incrementFlow(1);
        flow->repetitionsDone--;
    }
    const bool added = myFlows.add(flow->id, flow);
    if (added) {
        myHaveActiveFlows = true;
    }
    return added;
}


bool
RONet::addPerson(ROPerson* person) {
    if (myPersonIDs.count(person->getID()) == 0) {
        myPersonIDs.insert(person->getID());
        myRoutables[person->getDepart()].push_back(person);
        return true;
    }
    WRITE_ERRORF(TL("Another person with the id '%' exists."), person->getID());
    return false;
}


void
RONet::addContainer(const SUMOTime depart, const std::string desc) {
    myContainers.insert(std::pair<const SUMOTime, const std::string>(depart, desc));
}


void
RONet::checkFlows(SUMOTime time, MsgHandler* errorHandler) {
    myHaveActiveFlows = false;
    for (const auto& i : myFlows) {
        SUMOVehicleParameter* const pars = i.second;
        if (pars->line != "" && !myDoPTRouting) {
            continue;
        }
        if (myKeepFlows) {
            if (pars->repetitionsDone < pars->repetitionNumber) {
                // each each flow only once
                pars->repetitionsDone = pars->repetitionNumber;
                const SUMOVTypeParameter* type = getVehicleTypeSecure(pars->vtypeid);
                if (type == nullptr) {
                    type = getVehicleTypeSecure(DEFAULT_VTYPE_ID);
                } else {
                    auto dist = getVTypeDistribution(pars->vtypeid);
                    if (dist != nullptr) {
                        WRITE_WARNINGF("Keeping flow '%' with a vTypeDistribution can lead to invalid routes if the distribution contains different vClasses", pars->id);
                    }
                }
                RORouteDef* route = getRouteDef(pars->routeid)->copy(pars->routeid, pars->depart);
                ROVehicle* veh = new ROVehicle(*pars, route, type, this, errorHandler);
                addVehicle(pars->id, veh);
            }
        } else if (pars->repetitionProbability > 0) {
            if (pars->repetitionEnd > pars->depart && pars->repetitionsDone < pars->repetitionNumber) {
                myHaveActiveFlows = true;
            }
            const SUMOTime origDepart = pars->depart;
            while (pars->depart < time && pars->repetitionsDone < pars->repetitionNumber) {
                if (pars->repetitionEnd <= pars->depart) {
                    break;
                }
                // only call rand if all other conditions are met
                if (RandHelper::rand() < (pars->repetitionProbability * TS)) {
                    SUMOVehicleParameter* newPars = new SUMOVehicleParameter(*pars);
                    newPars->id = pars->id + "." + toString(pars->repetitionsDone);
                    newPars->depart = pars->depart;
                    for (StopParVector::iterator stop = newPars->stops.begin(); stop != newPars->stops.end(); ++stop) {
                        if (stop->until >= 0) {
                            stop->until += pars->depart - origDepart;
                        }
                    }
                    pars->repetitionsDone++;
                    // try to build the vehicle
                    const SUMOVTypeParameter* type = getVehicleTypeSecure(pars->vtypeid);
                    if (type == nullptr) {
                        type = getVehicleTypeSecure(DEFAULT_VTYPE_ID);
                    } else if (!myKeepVTypeDist) {
                        // fix the type id in case we used a distribution
                        newPars->vtypeid = type->id;
                    }
                    const SUMOTime stopOffset = pars->routeid[0] == '!' ? pars->depart - origDepart : pars->depart;
                    RORouteDef* route = getRouteDef(pars->routeid)->copy("!" + newPars->id, stopOffset);
                    ROVehicle* veh = new ROVehicle(*newPars, route, type, this, errorHandler);
                    addVehicle(newPars->id, veh);
                    delete newPars;
                }
                pars->depart += DELTA_T;
            }
        } else {
            SUMOTime depart = static_cast<SUMOTime>(pars->depart + pars->repetitionTotalOffset);
            while (pars->repetitionsDone < pars->repetitionNumber && pars->repetitionEnd >= depart) {
                myHaveActiveFlows = true;
                depart = static_cast<SUMOTime>(pars->depart + pars->repetitionTotalOffset);
                if (myDepartures.find(pars->id) != myDepartures.end()) {
                    depart = myDepartures[pars->id].back();
                }
                if (depart >= time + DELTA_T) {
                    break;
                }
                if (myDepartures.find(pars->id) != myDepartures.end()) {
                    myDepartures[pars->id].pop_back();
                }
                SUMOVehicleParameter* newPars = new SUMOVehicleParameter(*pars);
                newPars->id = pars->id + "." + toString(pars->repetitionsDone);
                newPars->depart = depart;
                for (StopParVector::iterator stop = newPars->stops.begin(); stop != newPars->stops.end(); ++stop) {
                    if (stop->until >= 0) {
                        stop->until += depart - pars->depart;
                    }
                    if (stop->arrival >= 0) {
                        stop->arrival += depart - pars->depart;
                    }
                }
                pars->incrementFlow(1);
                // try to build the vehicle
                const SUMOVTypeParameter* type = getVehicleTypeSecure(pars->vtypeid);
                if (type == nullptr) {
                    type = getVehicleTypeSecure(DEFAULT_VTYPE_ID);
                } else if (!myKeepVTypeDist) {
                    // fix the type id in case we used a distribution
                    newPars->vtypeid = type->id;
                }
                const SUMOTime stopOffset = pars->routeid[0] == '!' ? depart - pars->depart : depart;
                RORouteDef* route = getRouteDef(pars->routeid)->copy("!" + newPars->id, stopOffset);
                ROVehicle* veh = new ROVehicle(*newPars, route, type, this, errorHandler);
                addVehicle(newPars->id, veh);
                delete newPars;
            }
        }
    }
}


void
RONet::createBulkRouteRequests(const RORouterProvider& provider, const SUMOTime time, const bool removeLoops) {
    std::map<const int, std::vector<RORoutable*> > bulkVehs;
    int numBulked = 0;
    for (RoutablesMap::const_iterator i = myRoutables.begin(); i != myRoutables.end(); ++i) {
        if (i->first >= time) {
            break;
        }
        for (RORoutable* const routable : i->second) {
            const ROEdge* const depEdge = routable->getDepartEdge();
            bulkVehs[depEdge->getNumericalID()].push_back(routable);
            RORoutable* const first = bulkVehs[depEdge->getNumericalID()].front();
            numBulked++;
            if (first->getMaxSpeed() != routable->getMaxSpeed()) {
                WRITE_WARNINGF(TL("Bulking different maximum speeds ('%' and '%') may lead to suboptimal routes."), first->getID(), routable->getID());
            }
            if (first->getVClass() != routable->getVClass()) {
                WRITE_WARNINGF(TL("Bulking different vehicle classes ('%' and '%') may lead to invalid routes."), first->getID(), routable->getID());
            }
        }
    }
#ifdef HAVE_FOX
    int workerIndex = 0;
#endif
    if ((int)bulkVehs.size() < numBulked) {
        WRITE_MESSAGE(TLF("Using bulk-mode for % entities from % origins", numBulked, bulkVehs.size()));
    }
    for (std::map<const int, std::vector<RORoutable*> >::const_iterator i = bulkVehs.begin(); i != bulkVehs.end(); ++i) {
#ifdef HAVE_FOX
        if (myThreadPool.size() > 0) {
            bool bulk = true;
            for (RORoutable* const r : i->second) {
                myThreadPool.add(new RoutingTask(r, removeLoops, myErrorHandler), workerIndex);
                if (bulk) {
                    myThreadPool.add(new BulkmodeTask(true), workerIndex);
                    bulk = false;
                }
            }
            myThreadPool.add(new BulkmodeTask(false), workerIndex);
            workerIndex++;
            if (workerIndex == (int)myThreadPool.size()) {
                workerIndex = 0;
            }
            continue;
        }
#endif
        for (RORoutable* const r : i->second) {
            r->computeRoute(provider, removeLoops, myErrorHandler);
            provider.setBulkMode(true);
        }
        provider.setBulkMode(false);
    }
}


SUMOTime
RONet::saveAndRemoveRoutesUntil(OptionsCont& options, const RORouterProvider& provider,
                                SUMOTime time) {
    MsgHandler* mh = (options.getBool("ignore-errors") ?
                      MsgHandler::getWarningInstance() : MsgHandler::getErrorInstance());
    if (myHaveActiveFlows) {
        checkFlows(time, mh);
    }
    SUMOTime lastTime = -1;
    const bool removeLoops = options.getBool("remove-loops");
#ifdef HAVE_FOX
    const int maxNumThreads = options.getInt("routing-threads");
#endif
    if (myRoutables.size() != 0) {
        if (options.getBool("bulk-routing")) {
#ifdef HAVE_FOX
            while ((int)myThreadPool.size() < maxNumThreads) {
                new WorkerThread(myThreadPool, provider);
            }
#endif
            createBulkRouteRequests(provider, time, removeLoops);
        } else {
            for (RoutablesMap::const_iterator i = myRoutables.begin(); i != myRoutables.end(); ++i) {
                if (i->first >= time) {
                    break;
                }
                for (RORoutable* const routable : i->second) {
#ifdef HAVE_FOX
                    // add task
                    if (maxNumThreads > 0) {
                        const int numThreads = (int)myThreadPool.size();
                        if (numThreads == 0) {
                            // This is the very first routing. Since at least the CHRouter needs initialization
                            // before it gets cloned, we do not do this in parallel
                            routable->computeRoute(provider, removeLoops, myErrorHandler);
                            new WorkerThread(myThreadPool, provider);
                        } else {
                            // add thread if necessary
                            if (numThreads < maxNumThreads && myThreadPool.isFull()) {
                                new WorkerThread(myThreadPool, provider);
                            }
                            myThreadPool.add(new RoutingTask(routable, removeLoops, myErrorHandler));
                        }
                        continue;
                    }
#endif
                    routable->computeRoute(provider, removeLoops, myErrorHandler);
                }
            }
        }
#ifdef HAVE_FOX
        myThreadPool.waitAll();
#endif
    }
    const double scale = options.exists("scale-suffix") ? options.getFloat("scale") : 1;
    // write all vehicles (and additional structures)
    while (myRoutables.size() != 0 || myContainers.size() != 0) {
        // get the next vehicle, person or container
        RoutablesMap::iterator routables = myRoutables.begin();
        const SUMOTime routableTime = routables == myRoutables.end() ? SUMOTime_MAX : routables->first;
        ContainerMap::iterator container = myContainers.begin();
        const SUMOTime containerTime = container == myContainers.end() ? SUMOTime_MAX : container->first;
        // check whether it shall not yet be computed
        if (routableTime >= time && containerTime >= time) {
            lastTime = MIN2(routableTime, containerTime);
            break;
        }
        const SUMOTime minTime = MIN2(routableTime, containerTime);
        if (routableTime == minTime) {
            // check whether to print the output
            if (lastTime != routableTime && lastTime != -1) {
                // report writing progress
                if (options.getInt("stats-period") >= 0 && ((int)routableTime % options.getInt("stats-period")) == 0) {
                    WRITE_MESSAGE("Read: " + toString(myVehIDs.size()) + ",  Discarded: " + toString(myDiscardedRouteNo) + ",  Written: " + toString(myWrittenRouteNo));
                }
            }
            lastTime = routableTime;
            for (const RORoutable* const r : routables->second) {
                // ok, check whether it has been routed
                if (r->getRoutingSuccess()) {
                    // write the route
                    int quota = getScalingQuota(scale, myWrittenRouteNo);
                    r->write(myRoutesOutput, myRouteAlternativesOutput, myTypesOutput, options, quota);
                    myWrittenRouteNo++;
                } else {
                    myDiscardedRouteNo++;
                }
                // we need to keep individual public transport vehicles but not the flows
                if (!r->isPublicTransport() || r->isPartOfFlow()) {
                    // delete routes and the vehicle
                    const ROVehicle* const veh = dynamic_cast<const ROVehicle*>(r);
                    if (veh != nullptr && veh->getRouteDefinition()->getID()[0] == '!') {
                        if (r->isPartOfFlow() || !myRoutes.remove(veh->getRouteDefinition()->getID())) {
                            delete veh->getRouteDefinition();
                        }
                    }
                    delete r;
                }
            }
            myRoutables.erase(routables);
        }
        if (containerTime == minTime) {
            myRoutesOutput->writePreformattedTag(container->second);
            if (myRouteAlternativesOutput != nullptr) {
                myRouteAlternativesOutput->writePreformattedTag(container->second);
            }
            myContainers.erase(container);
        }
    }
    return lastTime;
}


bool
RONet::furtherStored() {
    return myRoutables.size() > 0 || (myFlows.size() > 0 && myHaveActiveFlows) || myContainers.size() > 0;
}


int
RONet::getEdgeNumber() const {
    return myEdges.size();
}


int
RONet::getInternalEdgeNumber() const {
    return myNumInternalEdges;
}


ROEdge*
RONet::getEdgeForLaneID(const std::string& laneID) const {
    return getEdge(SUMOXMLDefinitions::getEdgeIDFromLane(laneID));
}


ROLane*
RONet::getLane(const std::string& laneID) const {
    int laneIndex = SUMOXMLDefinitions::getIndexFromLane(laneID);
    return getEdgeForLaneID(laneID)->getLanes()[laneIndex];
}


void
RONet::adaptIntermodalRouter(ROIntermodalRouter& router) {
    double taxiWait = STEPS2TIME(string2time(OptionsCont::getOptions().getString("persontrip.taxi.waiting-time")));
    for (const auto& stopType : myInstance->myStoppingPlaces) {
        // add access to all stopping places
        const SumoXMLTag element = stopType.first;
        for (const auto& stop : stopType.second) {
            router.getNetwork()->addAccess(stop.first, myInstance->getEdgeForLaneID(stop.second->lane),
                                           stop.second->startPos, stop.second->endPos, 0., element, false, taxiWait);
            // add access to all public transport stops
            if (element == SUMO_TAG_BUS_STOP) {
                for (const auto& a : stop.second->accessPos) {
                    router.getNetwork()->addAccess(stop.first, myInstance->getEdgeForLaneID(std::get<0>(a)),
                                                   std::get<1>(a), std::get<1>(a), std::get<2>(a), SUMO_TAG_BUS_STOP, true, taxiWait);
                }
            }
        }
    }
    // fill the public transport router with pre-parsed public transport lines
    for (const auto& i : myInstance->myFlows) {
        if (i.second->line != "") {
            const RORouteDef* const route = myInstance->getRouteDef(i.second->routeid);
            const StopParVector* addStops = nullptr;
            if (route != nullptr && route->getFirstRoute() != nullptr) {
                addStops = &route->getFirstRoute()->getStops();
            }
            router.getNetwork()->addSchedule(*i.second, addStops);
        }
    }
    for (const RORoutable* const veh : myInstance->myPTVehicles) {
        // add single vehicles with line attribute which are not part of a flow
        // no need to add route stops here, they have been added to the vehicle before
        router.getNetwork()->addSchedule(veh->getParameter());
    }
    // add access to transfer from walking to taxi-use
    if ((router.getCarWalkTransfer() & ModeChangeOptions::TAXI_PICKUP_ANYWHERE) != 0) {
        for (const ROEdge* edge : ROEdge::getAllEdges()) {
            if (!edge->isTazConnector() && (edge->getPermissions() & SVC_PEDESTRIAN) != 0 && (edge->getPermissions() & SVC_TAXI) != 0) {
                router.getNetwork()->addCarAccess(edge, SVC_TAXI, taxiWait);
            }
        }
    }
}


bool
RONet::hasPermissions() const {
    return myHavePermissions;
}


void
RONet::setPermissionsFound() {
    myHavePermissions = true;
}

bool
RONet::hasLoadedEffort() const {
    for (const auto& item : myEdges) {
        if (item.second->hasStoredEffort()) {
            return true;
        }
    }
    return false;
}

const std::string
RONet::getStoppingPlaceName(const std::string& id) const {
    for (const auto& mapItem : myStoppingPlaces) {
        SUMOVehicleParameter::Stop* stop = mapItem.second.get(id);
        if (stop != nullptr) {
            // see RONetHandler::parseStoppingPlace
            return stop->busstop;
        }
    }
    return "";
}

const std::string
RONet::getStoppingPlaceElement(const std::string& id) const {
    for (const auto& mapItem : myStoppingPlaces) {
        SUMOVehicleParameter::Stop* stop = mapItem.second.get(id);
        if (stop != nullptr) {
            // see RONetHandler::parseStoppingPlace
            return stop->actType;
        }
    }
    return toString(SUMO_TAG_BUS_STOP);
}


void
RONet::addProhibition(const ROEdge* edge, const RouterProhibition& prohibition) {
    if (myProhibitions.count(edge) != 0) {
        throw ProcessError(TLF("Already loaded prohibition for edge '%'. (Only one prohibition per edge is supported)", edge->getID()));
    }
    myProhibitions[edge] = prohibition;
    myHavePermissions = true;
}


void
RONet::addLaneProhibition(const ROLane* lane, const RouterProhibition& prohibition) {
    if (myLaneProhibitions.count(lane) != 0) {
        throw ProcessError(TLF("Already loaded prohibition for lane '%'. (Only one prohibition per lane is supported)", lane->getID()));
    }
    assert(prohibition.end > prohibition.begin);
    myLaneProhibitions[lane] = prohibition;
    myLaneProhibitionTimes[prohibition.begin].insert(lane);
    myHavePermissions = true;
}


void
RONet::updateLaneProhibitions(SUMOTime begin) {
    const double beginS = STEPS2TIME(begin);
    while (myLaneProhibitionTimes.size() > 0 && myLaneProhibitionTimes.begin()->first <= beginS) {
        const double t = myLaneProhibitionTimes.begin()->first;
        for (const ROLane* const lane : myLaneProhibitionTimes.begin()->second) {
            const SVCPermissions orig = lane->getPermissions();
            assert(myLaneProhibitions.count(lane) != 0);
            RouterProhibition& rp = myLaneProhibitions[lane];
            const_cast<ROLane*>(lane)->setPermissions(rp.permissions);
            lane->getEdge().resetSuccessors();
            for (ROEdge* pred : lane->getEdge().getPredecessors()) {
                pred->resetSuccessors();
            }
            if (t == rp.begin) {
                // schedule restoration of original permissions. This works
                // without a stack because there is at most one prohibition per lane
                myLaneProhibitionTimes[rp.end].insert(lane);
                rp.permissions = orig;
            }
        }
        myLaneProhibitionTimes.erase(myLaneProhibitionTimes.begin());
    }
}


#ifdef HAVE_FOX
// ---------------------------------------------------------------------------
// RONet::RoutingTask-methods
// ---------------------------------------------------------------------------
void
RONet::RoutingTask::run(MFXWorkerThread* context) {
    myRoutable->computeRoute(*static_cast<WorkerThread*>(context), myRemoveLoops, myErrorHandler);
}
#endif


/****************************************************************************/
