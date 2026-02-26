/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2017-2026 German Aerospace Center (DLR) and others.
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
/// @file    Simulation.cpp
/// @author  Laura Bieker-Walz
/// @author  Robert Hilbrich
/// @author  Mirko Barthauer
/// @date    15.09.2017
///
// C++ TraCI client API implementation
/****************************************************************************/
#include <config.h>
#ifdef HAVE_VERSION_H
#include <version.h>
#endif
#include <utils/options/OptionsCont.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/StdDefs.h>
#include <utils/common/StringTokenizer.h>
#include <utils/common/StringUtils.h>
#include <utils/common/SystemFrame.h>
#include <utils/geom/GeoConvHelper.h>
#include <utils/options/OptionsIO.h>
#include <utils/router/IntermodalRouter.h>
#include <utils/router/PedestrianRouter.h>
#include <utils/xml/XMLSubSys.h>
#include <microsim/MSNet.h>
#include <microsim/MSEdgeControl.h>
#include <microsim/MSInsertionControl.h>
#include <microsim/MSEdge.h>
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSVehicleControl.h>
#include <microsim/MSVehicleTransfer.h>
#include <microsim/transportables/MSTransportableControl.h>
#include <microsim/MSStateHandler.h>
#include <microsim/MSStoppingPlace.h>
#include <microsim/MSParkingArea.h>
#include <microsim/devices/MSRoutingEngine.h>
#include <microsim/devices/MSDevice_Taxi.h>
#include <microsim/trigger/MSChargingStation.h>
#include <microsim/trigger/MSOverheadWire.h>
#include <microsim/devices/MSDevice_Tripinfo.h>
#include <mesosim/MELoop.h>
#include <mesosim/MESegment.h>
#include <netload/NLBuilder.h>
#include <libsumo/Helper.h>
#include <libsumo/StorageHelper.h>
#include <libsumo/TraCIConstants.h>
#ifdef HAVE_LIBSUMOGUI
#include "GUI.h"
#endif
#include "Simulation.h"
#include <libsumo/TraCIDefs.h>


namespace libsumo {
// ===========================================================================
// static member initializations
// ===========================================================================
SubscriptionResults Simulation::mySubscriptionResults;
ContextSubscriptionResults Simulation::myContextSubscriptionResults;
#ifdef HAVE_FOX
FXMutex Simulation::myStepMutex;
#endif


// ===========================================================================
// static member definitions
// ===========================================================================
std::pair<int, std::string>
Simulation::init(int /* port */, int /* numRetries */, const std::string& /* host */, const std::string& /* label */, FILE* const /* pipe */) {
    throw TraCIException("Multi client support (including connection switching) is not implemented in libsumo.");
}


std::pair<int, std::string>
Simulation::start(const std::vector<std::string>& cmd, int /* port */, int /* numRetries */, const std::string& /* label */, const bool /* verbose */,
                  const std::string& /* traceFile */, bool /* traceGetters */, void* /* _stdout */) {
#ifdef HAVE_LIBSUMOGUI
    if (GUI::start(cmd)) {
        return getVersion();
    }
#endif
    load(std::vector<std::string>(cmd.begin() + 1, cmd.end()));
    return getVersion();
}


bool
Simulation::isLibsumo() {
    return true;
}


void
Simulation::switchConnection(const std::string& /* label */) {
    throw TraCIException("Multi client support (including connection switching) is not implemented in libsumo.");
}


const std::string&
Simulation::getLabel() {
    throw TraCIException("Multi client support (including connection switching) is not implemented in libsumo.");
}


void
Simulation::setOrder(int /* order */) {
    throw TraCIException("Multi client support (including connection switching) is not implemented in libsumo.");
}


void
Simulation::load(const std::vector<std::string>& args) {
#ifdef HAVE_LIBSUMOGUI
    if (GUI::load(args)) {
        return;
    }
#endif
    close("Libsumo issued load command.");
    try {
        OptionsCont::getOptions().setApplicationName("libsumo", "Eclipse SUMO libsumo " VERSION_STRING);
        gSimulation = true;
        XMLSubSys::init();
        OptionsIO::setArgs(args);
        if (NLBuilder::init(true) != nullptr) {
            const SUMOTime begin = string2time(OptionsCont::getOptions().getString("begin"));
            MSNet::getInstance()->setCurrentTimeStep(begin); // needed for state loading
            WRITE_MESSAGEF(TL("Simulation version % started via libsumo with time: %."), VERSION_STRING, time2string(begin));
        }
    } catch (ProcessError& e) {
        throw TraCIException(e.what());
    }
}


bool
Simulation::hasGUI() {
#ifdef HAVE_LIBSUMOGUI
    return GUI::hasInstance();
#else
    return false;
#endif
}


bool
Simulation::isLoaded() {
    return MSNet::hasInstance();
}


void
Simulation::step(const double time) {
#ifdef HAVE_FOX
    FXMutexLock lock(myStepMutex);
#endif
    Helper::clearStateChanges();
    const SUMOTime t = TIME2STEPS(time);
#ifdef HAVE_LIBSUMOGUI
    if (!GUI::step(t)) {
#endif
        if (t == 0) {
            MSNet::getInstance()->simulationStep();
        } else {
            while (SIMSTEP < t) {
                MSNet::getInstance()->simulationStep();
            }
        }
#ifdef HAVE_LIBSUMOGUI
    }
#endif
    Helper::handleSubscriptions(SIMSTEP);
}


void
Simulation::executeMove() {
    MSNet::getInstance()->simulationStep(true);
}


void
Simulation::close(const std::string& reason) {
    Helper::clearSubscriptions();
    if (
#ifdef HAVE_LIBSUMOGUI
        !GUI::close(reason) &&
#endif
        MSNet::hasInstance()) {
        MSNet::getInstance()->closeSimulation(0, reason);
        delete MSNet::getInstance();
        SystemFrame::close();
    }
}


void
Simulation::subscribe(const std::vector<int>& varIDs, double begin, double end, const libsumo::TraCIResults& parameters) {
    libsumo::Helper::subscribe(CMD_SUBSCRIBE_SIM_VARIABLE, "", varIDs, begin, end, parameters);
}


const TraCIResults
Simulation::getSubscriptionResults() {
    return mySubscriptionResults[""];
}


LIBSUMO_SUBSCRIPTION_IMPLEMENTATION(Simulation, SIM)


std::pair<int, std::string>
Simulation::getVersion() {
    return std::make_pair(libsumo::TRACI_VERSION, "SUMO " VERSION_STRING);
}


std::string
Simulation::getOption(const std::string& option) {
    OptionsCont& oc = OptionsCont::getOptions();
    if (!oc.exists(option)) {
        throw TraCIException("The option " + option + " is unknown.");
    }
    return oc.getValueString(option);
}


int
Simulation::getCurrentTime() {
    return (int)MSNet::getInstance()->getCurrentTimeStep();
}


double
Simulation::getTime() {
    return SIMTIME;
}

double
Simulation::getEndTime() {
    return STEPS2TIME(string2time(OptionsCont::getOptions().getString("end")));
}


int
Simulation::getLoadedNumber() {
    return (int)Helper::getVehicleStateChanges(MSNet::VehicleState::BUILT).size();
}


std::vector<std::string>
Simulation::getLoadedIDList() {
    return Helper::getVehicleStateChanges(MSNet::VehicleState::BUILT);
}


int
Simulation::getDepartedNumber() {
    return (int)Helper::getVehicleStateChanges(MSNet::VehicleState::DEPARTED).size();
}


std::vector<std::string>
Simulation::getDepartedIDList() {
    return Helper::getVehicleStateChanges(MSNet::VehicleState::DEPARTED);
}


int
Simulation::getArrivedNumber() {
    return (int)Helper::getVehicleStateChanges(MSNet::VehicleState::ARRIVED).size();
}


std::vector<std::string>
Simulation::getArrivedIDList() {
    return Helper::getVehicleStateChanges(MSNet::VehicleState::ARRIVED);
}


int
Simulation::getParkingStartingVehiclesNumber() {
    return (int)Helper::getVehicleStateChanges(MSNet::VehicleState::STARTING_PARKING).size();
}


std::vector<std::string>
Simulation::getParkingStartingVehiclesIDList() {
    return Helper::getVehicleStateChanges(MSNet::VehicleState::STARTING_PARKING);
}


int
Simulation::getParkingEndingVehiclesNumber() {
    return (int)Helper::getVehicleStateChanges(MSNet::VehicleState::ENDING_PARKING).size();
}


std::vector<std::string>
Simulation::getParkingEndingVehiclesIDList() {
    return Helper::getVehicleStateChanges(MSNet::VehicleState::ENDING_PARKING);
}


int
Simulation::getStopStartingVehiclesNumber() {
    return (int)Helper::getVehicleStateChanges(MSNet::VehicleState::STARTING_STOP).size();
}


std::vector<std::string>
Simulation::getStopStartingVehiclesIDList() {
    return Helper::getVehicleStateChanges(MSNet::VehicleState::STARTING_STOP);
}


int
Simulation::getStopEndingVehiclesNumber() {
    return (int)Helper::getVehicleStateChanges(MSNet::VehicleState::ENDING_STOP).size();
}


std::vector<std::string>
Simulation::getStopEndingVehiclesIDList() {
    return Helper::getVehicleStateChanges(MSNet::VehicleState::ENDING_STOP);
}


int
Simulation::getCollidingVehiclesNumber() {
    return (int)Helper::getVehicleStateChanges(MSNet::VehicleState::COLLISION).size();
}


std::vector<std::string>
Simulation::getCollidingVehiclesIDList() {
    return Helper::getVehicleStateChanges(MSNet::VehicleState::COLLISION);
}


int
Simulation::getEmergencyStoppingVehiclesNumber() {
    return (int)Helper::getVehicleStateChanges(MSNet::VehicleState::EMERGENCYSTOP).size();
}


std::vector<std::string>
Simulation::getEmergencyStoppingVehiclesIDList() {
    return Helper::getVehicleStateChanges(MSNet::VehicleState::EMERGENCYSTOP);
}


int
Simulation::getStartingTeleportNumber() {
    return (int)Helper::getVehicleStateChanges(MSNet::VehicleState::STARTING_TELEPORT).size();
}


std::vector<std::string>
Simulation::getStartingTeleportIDList() {
    return Helper::getVehicleStateChanges(MSNet::VehicleState::STARTING_TELEPORT);
}


int
Simulation::getEndingTeleportNumber() {
    return (int)Helper::getVehicleStateChanges(MSNet::VehicleState::ENDING_TELEPORT).size();
}


std::vector<std::string>
Simulation::getEndingTeleportIDList() {
    return Helper::getVehicleStateChanges(MSNet::VehicleState::ENDING_TELEPORT);
}

int
Simulation::getDepartedPersonNumber() {
    return (int)Helper::getTransportableStateChanges(MSNet::TransportableState::PERSON_DEPARTED).size();
}


std::vector<std::string>
Simulation::getDepartedPersonIDList() {
    return Helper::getTransportableStateChanges(MSNet::TransportableState::PERSON_DEPARTED);
}


int
Simulation::getArrivedPersonNumber() {
    return (int)Helper::getTransportableStateChanges(MSNet::TransportableState::PERSON_ARRIVED).size();
}


std::vector<std::string>
Simulation::getArrivedPersonIDList() {
    return Helper::getTransportableStateChanges(MSNet::TransportableState::PERSON_ARRIVED);
}

std::vector<std::string>
Simulation::getBusStopIDList() {
    std::vector<std::string> result;
    for (const auto& pair : MSNet::getInstance()->getStoppingPlaces(SUMO_TAG_BUS_STOP)) {
        result.push_back(pair.first);
    }
    return result;
}

int
Simulation::getBusStopWaiting(const std::string& stopID) {
    MSStoppingPlace* s = MSNet::getInstance()->getStoppingPlace(stopID, SUMO_TAG_BUS_STOP);
    if (s == nullptr) {
        throw TraCIException("Unknown bus stop '" + stopID + "'.");
    }
    return s->getTransportableNumber();
}

std::vector<std::string>
Simulation::getBusStopWaitingIDList(const std::string& stopID) {
    MSStoppingPlace* s = MSNet::getInstance()->getStoppingPlace(stopID, SUMO_TAG_BUS_STOP);
    if (s == nullptr) {
        throw TraCIException("Unknown bus stop '" + stopID + "'.");
    }
    std::vector<std::string> result;
    for (const MSTransportable* t : s->getTransportables()) {
        result.push_back(t->getID());
    }
    return result;
}


std::vector<std::string>
Simulation::getPendingVehicles() {
    std::vector<std::string> result;
    for (const SUMOVehicle* veh : MSNet::getInstance()->getInsertionControl().getPendingVehicles()) {
        result.push_back(veh->getID());
    }
    return result;
}


std::vector<libsumo::TraCICollision>
Simulation::getCollisions() {
    std::vector<libsumo::TraCICollision> result;
    for (auto item : MSNet::getInstance()->getCollisions()) {
        for (const MSNet::Collision& c : item.second) {
            libsumo::TraCICollision c2;
            c2.collider = item.first;
            c2.victim = c.victim;
            c2.colliderType = c.colliderType;
            c2.victimType = c.victimType;
            c2.colliderSpeed = c.colliderSpeed;
            c2.victimSpeed = c.victimSpeed;
            c2.type = c.type;
            c2.lane = c.lane->getID();
            c2.pos = c.pos;
            result.push_back(c2);
        }
    }
    return result;
}

double
Simulation::getScale() {
    return MSNet::getInstance()->getVehicleControl().getScale();
}

double
Simulation::getDeltaT() {
    return TS;
}


TraCIPositionVector
Simulation::getNetBoundary() {
    Boundary b = GeoConvHelper::getFinal().getConvBoundary();
    TraCIPositionVector tb;
    TraCIPosition minV;
    TraCIPosition maxV;
    minV.x = b.xmin();
    maxV.x = b.xmax();
    minV.y = b.ymin();
    maxV.y = b.ymax();
    minV.z = b.zmin();
    maxV.z = b.zmax();
    tb.value.push_back(minV);
    tb.value.push_back(maxV);
    return tb;
}


int
Simulation::getMinExpectedNumber() {
    MSNet* net = MSNet::getInstance();
    return (net->getVehicleControl().getActiveVehicleCount()
            + net->getInsertionControl().getPendingFlowCount()
            + (net->hasPersons() ? net->getPersonControl().getActiveCount() : 0)
            + (net->hasContainers() ? net->getContainerControl().getActiveCount() : 0)
            + (MSDevice_Taxi::hasServableReservations() ? 1 : 0));
}


TraCIPosition
Simulation::convert2D(const std::string& edgeID, double pos, int laneIndex, bool toGeo) {
    Position result = Helper::getLaneChecking(edgeID, laneIndex, pos)->geometryPositionAtOffset(pos);
    if (toGeo) {
        GeoConvHelper::getFinal().cartesian2geo(result);
    }
    result.setz(0.);
    return Helper::makeTraCIPosition(result);
}


TraCIPosition
Simulation::convert3D(const std::string& edgeID, double pos, int laneIndex, bool toGeo) {
    Position result = Helper::getLaneChecking(edgeID, laneIndex, pos)->geometryPositionAtOffset(pos);
    if (toGeo) {
        GeoConvHelper::getFinal().cartesian2geo(result);
    }
    return Helper::makeTraCIPosition(result, true);
}


TraCIRoadPosition
Simulation::convertRoad(double x, double y, bool isGeo, const std::string& vClass) {
    Position pos(x, y);
    if (isGeo) {
        GeoConvHelper::getFinal().x2cartesian_const(pos);
    }
    if (!SumoVehicleClassStrings.hasString(vClass)) {
        throw TraCIException("Unknown vehicle class '" + vClass + "'.");
    }
    const SUMOVehicleClass vc = SumoVehicleClassStrings.get(vClass);
    std::pair<MSLane*, double> roadPos = libsumo::Helper::convertCartesianToRoadMap(pos, vc);
    if (roadPos.first == nullptr) {
        throw TraCIException("Cannot convert position to road.");
    }
    TraCIRoadPosition result;
    result.edgeID = roadPos.first->getEdge().getID();
    result.laneIndex = roadPos.first->getIndex();
    result.pos = roadPos.second;
    return result;
}


TraCIPosition
Simulation::convertGeo(double x, double y, bool fromGeo) {
    Position pos(x, y);
    if (fromGeo) {
        GeoConvHelper::getFinal().x2cartesian_const(pos);
    } else {
        GeoConvHelper::getFinal().cartesian2geo(pos);
    }
    return Helper::makeTraCIPosition(pos);
}


double
Simulation::getDistance2D(double x1, double y1, double x2, double y2, bool isGeo, bool isDriving) {
    Position pos1(x1, y1);
    Position pos2(x2, y2);
    if (isGeo) {
        GeoConvHelper::getFinal().x2cartesian_const(pos1);
        GeoConvHelper::getFinal().x2cartesian_const(pos2);
    }
    if (isDriving) {
        std::pair<const MSLane*, double> roadPos1 = libsumo::Helper::convertCartesianToRoadMap(pos1, SVC_IGNORING);
        std::pair<const MSLane*, double> roadPos2 = libsumo::Helper::convertCartesianToRoadMap(pos2, SVC_IGNORING);
        return Helper::getDrivingDistance(roadPos1, roadPos2);
    } else {
        return pos1.distanceTo(pos2);
    }
}


double
Simulation::getDistanceRoad(const std::string& edgeID1, double pos1, const std::string& edgeID2, double pos2, bool isDriving) {
    std::pair<const MSLane*, double> roadPos1 = std::make_pair(libsumo::Helper::getLaneChecking(edgeID1, 0, pos1), pos1);
    std::pair<const MSLane*, double> roadPos2 = std::make_pair(libsumo::Helper::getLaneChecking(edgeID2, 0, pos2), pos2);
    if (isDriving) {
        return Helper::getDrivingDistance(roadPos1, roadPos2);
    } else {
        const Position p1 = roadPos1.first->geometryPositionAtOffset(roadPos1.second);
        const Position p2 = roadPos2.first->geometryPositionAtOffset(roadPos2.second);
        return p1.distanceTo(p2);
    }
}


TraCIStage
Simulation::findRoute(const std::string& from, const std::string& to, const std::string& typeID,
                      double depart, int routingMode, double departPos, double arrivalPos) {
    TraCIStage result(STAGE_DRIVING);
    const MSEdge* const fromEdge = MSEdge::dictionary(from);
    if (fromEdge == nullptr) {
        throw TraCIException("Unknown from edge '" + from + "'.");
    }
    const MSEdge* const toEdge = MSEdge::dictionary(to);
    if (toEdge == nullptr) {
        throw TraCIException("Unknown to edge '" + to + "'.");
    }
    MSBaseVehicle* vehicle = nullptr;
    MSVehicleType* type = MSNet::getInstance()->getVehicleControl().getVType(typeID == "" ? DEFAULT_VTYPE_ID : typeID);
    if (type == nullptr) {
        throw TraCIException("The vehicle type '" + typeID + "' is not known.");
    }
    SUMOVehicleParameter* pars = new SUMOVehicleParameter();
    pars->id = "simulation.findRoute";
    try {
        ConstMSRoutePtr const routeDummy = std::make_shared<MSRoute>("", ConstMSEdgeVector({ fromEdge }), false, nullptr, StopParVector());
        vehicle = dynamic_cast<MSBaseVehicle*>(MSNet::getInstance()->getVehicleControl().buildVehicle(pars, routeDummy, type, false));
        std::string msg;
        if (!vehicle->hasValidRouteStart(msg)) {
            MSNet::getInstance()->getVehicleControl().deleteVehicle(vehicle, true);
            MSNet::getInstance()->getVehicleControl().discountRoutingVehicle();
            throw TraCIException("Invalid departure edge for vehicle type '" + type->getID() + "' (" + msg + ")");
        }
        // we need to fix the speed factor here for deterministic results
        vehicle->setChosenSpeedFactor(type->getSpeedFactor().getParameter(0));
        vehicle->setRoutingMode(routingMode);
    } catch (ProcessError& e) {
        throw TraCIException("Invalid departure edge for vehicle type '" + type->getID() + "' (" + e.what() + ")");
    }
    if (abs(departPos) > fromEdge->getLength()) {
        throw TraCIException("Invalid departPos " + toString(departPos) + " on edge '" + fromEdge->getID() + " (length " + toString(fromEdge->getLength()) + ")");
    }
    if (departPos < 0) {
        departPos += fromEdge->getLength();
    }
    if (arrivalPos == INVALID_DOUBLE_VALUE) {
        arrivalPos = toEdge->getLength();
    }
    if (abs(arrivalPos) > toEdge->getLength()) {
        throw TraCIException("Invalid arrivalPos " + toString(arrivalPos) + " on edge '" + toEdge->getID() + " (length " + toString(toEdge->getLength()) + ")");
    }
    if (arrivalPos < 0) {
        arrivalPos += toEdge->getLength();
    }
    ConstMSEdgeVector edges;
    const SUMOTime dep = depart < 0 ? MSNet::getInstance()->getCurrentTimeStep() : TIME2STEPS(depart);
    SUMOAbstractRouter<MSEdge, SUMOVehicle>& router = routingMode == ROUTING_MODE_AGGREGATED ? MSRoutingEngine::getRouterTT(0, vehicle->getVClass()) : MSNet::getInstance()->getRouterTT(0);
    router.compute(fromEdge, departPos, toEdge, arrivalPos, vehicle, dep, edges);
    for (const MSEdge* e : edges) {
        result.edges.push_back(e->getID());
    }
    result.travelTime = result.cost = router.recomputeCostsPos(edges, vehicle, departPos, arrivalPos, dep, &result.length);
    result.arrivalPos = arrivalPos;
    result.departPos = departPos;
    if (vehicle != nullptr) {
        MSNet::getInstance()->getVehicleControl().deleteVehicle(vehicle, true);
        MSNet::getInstance()->getVehicleControl().discountRoutingVehicle();
    }
    return result;
}


std::vector<TraCIStage>
Simulation::findIntermodalRoute(const std::string& from, const std::string& to,
                                const std::string& modes, double depart, const int routingMode, double speed, double walkFactor,
                                double departPos, double arrivalPos, const double departPosLat,
                                const std::string& pType, const std::string& vType, const std::string& destStop) {
    UNUSED_PARAMETER(departPosLat);
    std::vector<TraCIStage> result;
    const MSEdge* const fromEdge = MSEdge::dictionary(from);
    if (fromEdge == nullptr) {
        throw TraCIException("Unknown from edge '" + from + "'.");
    }
    const MSEdge* const toEdge = MSEdge::dictionary(to);
    if (toEdge == nullptr) {
        throw TraCIException("Unknown to edge '" + to + "'.");
    }
    MSVehicleControl& vehControl = MSNet::getInstance()->getVehicleControl();
    SVCPermissions modeSet = 0;
    std::vector<SUMOVehicleParameter*> pars;
    if (vType != "") {
        pars.push_back(new SUMOVehicleParameter());
        pars.back()->vtypeid = vType;
        pars.back()->id = vType;
        modeSet |= SVC_PASSENGER;
    }
    for (StringTokenizer st(modes); st.hasNext();) {
        const std::string mode = st.next();
        if (mode == toString(PersonMode::CAR)) {
            pars.push_back(new SUMOVehicleParameter());
            pars.back()->vtypeid = DEFAULT_VTYPE_ID;
            pars.back()->id = mode;
            modeSet |= SVC_PASSENGER;
        } else if (mode == toString(PersonMode::BICYCLE)) {
            pars.push_back(new SUMOVehicleParameter());
            pars.back()->vtypeid = DEFAULT_BIKETYPE_ID;
            pars.back()->id = mode;
            modeSet |= SVC_BICYCLE;
        } else if (mode == toString(PersonMode::TAXI)) {
            pars.push_back(new SUMOVehicleParameter());
            pars.back()->vtypeid = DEFAULT_TAXITYPE_ID;
            pars.back()->id = mode;
            pars.back()->line = mode;
            modeSet |= SVC_TAXI;
        } else if (mode == toString(PersonMode::PUBLIC)) {
            pars.push_back(nullptr);
            modeSet |= SVC_BUS;
        } else if (mode == toString(PersonMode::WALK)) {
            // do nothing
        } else {
            throw TraCIException("Unknown person mode '" + mode + "'.");
        }
    }
    if (pars.empty()) {
        pars.push_back(nullptr);
    }
    // interpret default arguments
    const MSVehicleType* pedType = vehControl.hasVType(pType) ? vehControl.getVType(pType) : vehControl.getVType(DEFAULT_PEDTYPE_ID);
    SUMOTime departStep = TIME2STEPS(depart);
    if (depart < 0) {
        departStep = MSNet::getInstance()->getCurrentTimeStep();
    }
    if (speed < 0) {
        speed =  MIN2(pedType->getMaxSpeed(), pedType->getDesiredMaxSpeed());
    }
    if (walkFactor < 0) {
        walkFactor = OptionsCont::getOptions().getFloat("persontrip.walkfactor");
    }
    const double externalFactor = StringUtils::toDouble(pedType->getParameter().getParameter("externalEffortFactor", "100"));
    if (departPos < 0) {
        departPos += fromEdge->getLength();
    }
    if (arrivalPos == INVALID_DOUBLE_VALUE) {
        arrivalPos = toEdge->getLength() / 2;
    } else if (arrivalPos < 0) {
        arrivalPos += toEdge->getLength();
    }
    if (departPos < 0 || departPos >= fromEdge->getLength()) {
        throw TraCIException("Invalid depart position " + toString(departPos) + " for edge '" + from + "'.");
    }
    if (arrivalPos < 0 || arrivalPos >= toEdge->getLength()) {
        throw TraCIException("Invalid arrival position " + toString(arrivalPos) + " for edge '" + to + "'.");
    }
    double minCost = std::numeric_limits<double>::max();
    MSTransportableRouter& router = MSNet::getInstance()->getIntermodalRouter(0, routingMode);
    for (SUMOVehicleParameter* vehPar : pars) {
        std::vector<TraCIStage> resultCand;
        SUMOVehicle* vehicle = nullptr;
        if (vehPar != nullptr) {
            MSVehicleType* type = MSNet::getInstance()->getVehicleControl().getVType(vehPar->vtypeid);
            const bool isTaxi = type != nullptr && type->getID() == DEFAULT_TAXITYPE_ID && vehPar->line == "taxi";
            if (type == nullptr) {
                throw TraCIException("Unknown vehicle type '" + vehPar->vtypeid + "'.");
            }
            if (type->getVehicleClass() != SVC_IGNORING && (fromEdge->getPermissions() & type->getVehicleClass()) == 0 && !isTaxi) {
                WRITE_WARNINGF(TL("Ignoring vehicle type '%' when performing intermodal routing because it is not allowed on the start edge '%'."), type->getID(), from);
            } else {
                ConstMSRoutePtr const routeDummy = std::make_shared<MSRoute>(vehPar->id, ConstMSEdgeVector({ fromEdge }), false, nullptr, StopParVector());
                vehicle = vehControl.buildVehicle(vehPar, routeDummy, type, !MSGlobals::gCheckRoutes);
                // we need to fix the speed factor here for deterministic results
                vehicle->setChosenSpeedFactor(type->getSpeedFactor().getParameter(0));
            }
        }
        std::vector<MSTransportableRouter::TripItem> items;
        if (router.compute(fromEdge, toEdge, departPos, "", arrivalPos, destStop,
                           speed * walkFactor, vehicle, pedType->getParameter(), modeSet, departStep, items, externalFactor)) {
            double cost = 0;
            for (std::vector<MSTransportableRouter::TripItem>::iterator it = items.begin(); it != items.end(); ++it) {
                if (!it->edges.empty()) {
                    resultCand.push_back(TraCIStage((it->line == "" ? STAGE_WALKING : STAGE_DRIVING), it->vType, it->line, it->destStop));
                    for (const MSEdge* e : it->edges) {
                        resultCand.back().edges.push_back(e->getID());
                    }
                    resultCand.back().travelTime = it->traveltime;
                    resultCand.back().cost = it->cost;
                    resultCand.back().length = it->length;
                    resultCand.back().intended = it->intended;
                    resultCand.back().depart = it->depart;
                    resultCand.back().departPos = it->departPos;
                    resultCand.back().arrivalPos = it->arrivalPos;
                    resultCand.back().description = it->description;
                }
                cost += it->cost;
            }
            if (cost < minCost) {
                minCost = cost;
                result = resultCand;
            }
        }
        if (vehicle != nullptr) {
            vehControl.deleteVehicle(vehicle, true);
            vehControl.discountRoutingVehicle();
        }
    }
    return result;
}


std::string
Simulation::getParameter(const std::string& objectID, const std::string& key) {
    if (StringUtils::startsWith(key, "chargingStation.")) {
        WRITE_WARNING("Retrieving chargingStation parameters via the simulation API is deprecated, please use traci.chargingstation.");
        const std::string attrName = key.substr(16);
        MSChargingStation* cs = static_cast<MSChargingStation*>(MSNet::getInstance()->getStoppingPlace(objectID, SUMO_TAG_CHARGING_STATION));
        if (cs == nullptr) {
            throw TraCIException("Invalid chargingStation '" + objectID + "'");
        }
        if (attrName == toString(SUMO_ATTR_TOTALENERGYCHARGED)) {
            return toString(cs->getTotalCharged());
        } else if (attrName == toString(SUMO_ATTR_NAME)) {
            return toString(cs->getMyName());
        } else if (attrName == "lane") {
            return cs->getLane().getID();
        } else if (cs->hasParameter(attrName)) {
            return cs->getParameter(attrName);
        } else {
            throw TraCIException("Invalid chargingStation parameter '" + attrName + "'");
        }
    } else if (StringUtils::startsWith(key, "overheadWire.")) {
        WRITE_WARNING("Retrieving overheadWire parameters via the simulation API is deprecated, please use traci.overheadwire.");
        const std::string attrName = key.substr(16);
        MSOverheadWire* cs = static_cast<MSOverheadWire*>(MSNet::getInstance()->getStoppingPlace(objectID, SUMO_TAG_OVERHEAD_WIRE_SEGMENT));
        if (cs == 0) {
            throw TraCIException("Invalid overhead wire '" + objectID + "'");
        }
        if (attrName == toString(SUMO_ATTR_TOTALENERGYCHARGED)) {
            return toString(cs->getTotalCharged());
        } else if (attrName == toString(SUMO_ATTR_NAME)) {
            return toString(cs->getMyName());
        } else {
            throw TraCIException("Invalid overhead wire parameter '" + attrName + "'");
        }
    } else if (StringUtils::startsWith(key, "net.")) {
        const std::string attrName = key.substr(4);
        if (attrName == toString(SUMO_ATTR_NET_OFFSET)) {
            return toString(GeoConvHelper::getFinal().getOffsetBase());
        } else {
            throw TraCIException("Invalid net parameter '" + attrName + "'");
        }
    } else if (StringUtils::startsWith(key, "stats.")) {
        if (objectID != "") {
            throw TraCIException("Simulation parameter '" + key + "' is not supported for object id '" + objectID + "'. Use empty id for stats");
        }
        const std::string attrName = key.substr(6);
        const MSVehicleControl& vc = MSNet::getInstance()->getVehicleControl();
        const MSTransportableControl* pc = MSNet::getInstance()->hasPersons() ? &MSNet::getInstance()->getPersonControl() : nullptr;
        if (attrName == "vehicles.loaded") {
            return toString(vc.getLoadedVehicleNo());
        } else if (attrName == "vehicles.inserted") {
            return toString(vc.getDepartedVehicleNo());
        } else if (attrName == "vehicles.running") {
            return toString(vc.getRunningVehicleNo());
        } else if (attrName == "vehicles.waiting") {
            return toString(MSNet::getInstance()->getInsertionControl().getWaitingVehicleNo());
        } else if (attrName == "teleports.total") {
            return toString(vc.getTeleportCount());
        } else if (attrName == "teleports.jam") {
            return toString(vc.getTeleportsJam());
        } else if (attrName == "teleports.yield") {
            return toString(vc.getTeleportsYield());
        } else if (attrName == "teleports.wrongLane") {
            return toString(vc.getTeleportsWrongLane());
        } else if (attrName == "safety.collisions") {
            return toString(vc.getCollisionCount());
        } else if (attrName == "safety.emergencyStops") {
            return toString(vc.getEmergencyStops());
        } else if (attrName == "safety.emergencyBraking") {
            return toString(vc.getEmergencyBrakingCount());
        } else if (attrName == "persons.loaded") {
            return toString(pc != nullptr ? pc->getLoadedNumber() : 0);
        } else if (attrName == "persons.running") {
            return toString(pc != nullptr ? pc->getRunningNumber() : 0);
        } else if (attrName == "persons.jammed") {
            return toString(pc != nullptr ? pc->getJammedNumber() : 0);
        } else if (attrName == "personTeleports.total") {
            return toString(pc != nullptr ? pc->getTeleportCount() : 0);
        } else if (attrName == "personTeleports.abortWait") {
            return toString(pc != nullptr ? pc->getTeleportsAbortWait() : 0);
        } else if (attrName == "personTeleports.wrongDest") {
            return toString(pc != nullptr ? pc->getTeleportsWrongDest() : 0);
        } else {
            throw TraCIException("Invalid stats parameter '" + attrName + "'");
        }
    } else if (StringUtils::startsWith(key, "parkingArea.")) {
        WRITE_WARNING("Retrieving parkingArea parameters via the simulation API is deprecated, please use traci.parkingarea.");
        const std::string attrName = key.substr(12);
        MSParkingArea* pa = static_cast<MSParkingArea*>(MSNet::getInstance()->getStoppingPlace(objectID, SUMO_TAG_PARKING_AREA));
        if (pa == nullptr) {
            throw TraCIException("Invalid parkingArea '" + objectID + "'");
        }
        if (attrName == "capacity") {
            return toString(pa->getCapacity());
        } else if (attrName == "occupancy") {
            return toString(pa->getOccupancyIncludingBlocked());
        } else if (attrName == toString(SUMO_ATTR_NAME)) {
            return toString(pa->getMyName());
        } else if (attrName == "lane") {
            return pa->getLane().getID();
        } else if (pa->hasParameter(attrName)) {
            return pa->getParameter(attrName);
        } else {
            throw TraCIException("Invalid parkingArea parameter '" + attrName + "'");
        }
    } else if (StringUtils::startsWith(key, "busStop.")) {
        WRITE_WARNING("Retrieving busStop parameters via the simulation API is deprecated, please use traci.busstop.");
        const std::string attrName = key.substr(8);
        MSStoppingPlace* bs = static_cast<MSStoppingPlace*>(MSNet::getInstance()->getStoppingPlace(objectID, SUMO_TAG_BUS_STOP));
        if (bs == nullptr) {
            throw TraCIException("Invalid busStop '" + objectID + "'");
        }
        if (attrName == toString(SUMO_ATTR_NAME)) {
            return toString(bs->getMyName());
        } else if (attrName == "lane") {
            return bs->getLane().getID();
        } else if (bs->hasParameter(attrName)) {
            return bs->getParameter(attrName);
        } else {
            throw TraCIException("Invalid busStop parameter '" + attrName + "'");
        }
    } else if (StringUtils::startsWith(key, "device.tripinfo.")) {
        if (objectID != "") {
            throw TraCIException("Simulation parameter '" + key + "' is not supported for object id '" + objectID
                                 + "'. Use empty id for global device parameters or vehicle domain for vehicle specific parameters");
        }
        const std::string attrName = key.substr(16);
        return MSDevice_Tripinfo::getGlobalParameter(attrName);
    } else if (StringUtils::startsWith(key, "buildConfig.")) {
        if (objectID != "") {
            throw TraCIException("Simulation parameter '" + key + "' is not supported for object id '" + objectID
                                 + "'. Use an empty id for buildConfig parameters");
        }
        if (key == "buildConfig.ARROW_SO_VERSION") {
#ifdef ARROW_SO_VERSION
            return ARROW_SO_VERSION;
#else
            return "";
#endif
        } else if (key == "buildConfig.JPS_VERSION") {
#ifdef JPS_VERSION
            return toString(JPS_VERSION);
#else
            return "";
#endif
        } else if (key == "buildConfig.ENABLED") {
            return HAVE_ENABLED;
        }
        throw TraCIException("Unknown parameter '" + key + "'");
    } else if (objectID == "") {
        return MSNet::getInstance()->getParameter(key, "");
    } else {
        throw TraCIException("Simulation parameter '" + key + "' is not supported for object id '" + objectID + "'. Use empty id for generic network parameters");
    }
}

LIBSUMO_GET_PARAMETER_WITH_KEY_IMPLEMENTATION(Simulation)

void
Simulation::setParameter(const std::string& objectID, const std::string& key, const std::string& value) {
    if (objectID == "") {
        MSNet::getInstance()->setParameter(key, value);
    } else {
        throw TraCIException("Setting simulation parameter '" + key + "' is not supported for object id '" + objectID + "'. Use empty id for generic network parameters");
    }
}

void
Simulation::setScale(double value) {
    MSNet::getInstance()->getVehicleControl().setScale(value);
}

void
Simulation::clearPending(const std::string& routeID) {
    MSNet::getInstance()->getInsertionControl().clearPendingVehicles(routeID);
}


void
Simulation::saveState(const std::string& fileName) {
    MSStateHandler::saveState(fileName, MSNet::getInstance()->getCurrentTimeStep());
}

double
Simulation::loadState(const std::string& fileName) {
    long before = PROGRESS_BEGIN_TIME_MESSAGE("Loading state from '" + fileName + "'");
    try {
        const SUMOTime newTime = MSNet::getInstance()->loadState(fileName, false);
        Helper::clearStateChanges();
        Helper::clearSubscriptions();
        PROGRESS_TIME_MESSAGE(before);
        return STEPS2TIME(newTime);
    } catch (const IOError& e) {
        throw TraCIException("Loading state from '" + fileName + "' failed. " + e.what());
    } catch (const ProcessError& e) {
        throw TraCIException("Loading state from '" + fileName + "' failed, check whether SUMO versions match. " + e.what());
    }
}

void
Simulation::writeMessage(const std::string& msg) {
    WRITE_MESSAGE(msg);
}


void
Simulation::storeShape(PositionVector& shape) {
    shape = GeoConvHelper::getFinal().getConvBoundary().getShape(true);
}


std::shared_ptr<VariableWrapper>
Simulation::makeWrapper() {
    return std::make_shared<Helper::SubscriptionWrapper>(handleVariable, mySubscriptionResults, myContextSubscriptionResults);
}


bool
Simulation::handleVariable(const std::string& objID, const int variable, VariableWrapper* wrapper, tcpip::Storage* paramData) {
    switch (variable) {
        case VAR_TIME:
            return wrapper->wrapDouble(objID, variable, getTime());
        case VAR_TIME_STEP:
            return wrapper->wrapInt(objID, variable, (int)getCurrentTime());
        case VAR_END:
            return wrapper->wrapDouble(objID, variable, getEndTime());
        case VAR_LOADED_VEHICLES_NUMBER:
            return wrapper->wrapInt(objID, variable, getLoadedNumber());
        case VAR_LOADED_VEHICLES_IDS:
            return wrapper->wrapStringList(objID, variable, getLoadedIDList());
        case VAR_DEPARTED_VEHICLES_NUMBER:
            return wrapper->wrapInt(objID, variable, getDepartedNumber());
        case VAR_DEPARTED_VEHICLES_IDS:
            return wrapper->wrapStringList(objID, variable, getDepartedIDList());
        case VAR_TELEPORT_STARTING_VEHICLES_NUMBER:
            return wrapper->wrapInt(objID, variable, getStartingTeleportNumber());
        case VAR_TELEPORT_STARTING_VEHICLES_IDS:
            return wrapper->wrapStringList(objID, variable, getStartingTeleportIDList());
        case VAR_TELEPORT_ENDING_VEHICLES_NUMBER:
            return wrapper->wrapInt(objID, variable, getEndingTeleportNumber());
        case VAR_TELEPORT_ENDING_VEHICLES_IDS:
            return wrapper->wrapStringList(objID, variable, getEndingTeleportIDList());
        case VAR_ARRIVED_VEHICLES_NUMBER:
            return wrapper->wrapInt(objID, variable, getArrivedNumber());
        case VAR_ARRIVED_VEHICLES_IDS:
            return wrapper->wrapStringList(objID, variable, getArrivedIDList());
        case VAR_PARKING_STARTING_VEHICLES_NUMBER:
            return wrapper->wrapInt(objID, variable, getParkingStartingVehiclesNumber());
        case VAR_PARKING_STARTING_VEHICLES_IDS:
            return wrapper->wrapStringList(objID, variable, getParkingStartingVehiclesIDList());
        case VAR_PARKING_ENDING_VEHICLES_NUMBER:
            return wrapper->wrapInt(objID, variable, getParkingEndingVehiclesNumber());
        case VAR_PARKING_ENDING_VEHICLES_IDS:
            return wrapper->wrapStringList(objID, variable, getParkingEndingVehiclesIDList());
        case VAR_STOP_STARTING_VEHICLES_NUMBER:
            return wrapper->wrapInt(objID, variable, getStopStartingVehiclesNumber());
        case VAR_STOP_STARTING_VEHICLES_IDS:
            return wrapper->wrapStringList(objID, variable, getStopStartingVehiclesIDList());
        case VAR_STOP_ENDING_VEHICLES_NUMBER:
            return wrapper->wrapInt(objID, variable, getStopEndingVehiclesNumber());
        case VAR_STOP_ENDING_VEHICLES_IDS:
            return wrapper->wrapStringList(objID, variable, getStopEndingVehiclesIDList());
        case VAR_COLLIDING_VEHICLES_NUMBER:
            return wrapper->wrapInt(objID, variable, getCollidingVehiclesNumber());
        case VAR_COLLIDING_VEHICLES_IDS:
            return wrapper->wrapStringList(objID, variable, getCollidingVehiclesIDList());
        case VAR_EMERGENCYSTOPPING_VEHICLES_NUMBER:
            return wrapper->wrapInt(objID, variable, getEmergencyStoppingVehiclesNumber());
        case VAR_EMERGENCYSTOPPING_VEHICLES_IDS:
            return wrapper->wrapStringList(objID, variable, getEmergencyStoppingVehiclesIDList());
        case VAR_DEPARTED_PERSONS_NUMBER:
            return wrapper->wrapInt(objID, variable, getDepartedPersonNumber());
        case VAR_DEPARTED_PERSONS_IDS:
            return wrapper->wrapStringList(objID, variable, getDepartedPersonIDList());
        case VAR_ARRIVED_PERSONS_NUMBER:
            return wrapper->wrapInt(objID, variable, getArrivedPersonNumber());
        case VAR_ARRIVED_PERSONS_IDS:
            return wrapper->wrapStringList(objID, variable, getArrivedPersonIDList());
        case VAR_SCALE:
            return wrapper->wrapDouble(objID, variable, getScale());
        case VAR_DELTA_T:
            return wrapper->wrapDouble(objID, variable, getDeltaT());
        case VAR_OPTION:
            return wrapper->wrapString(objID, variable, getOption(objID));
        case VAR_MIN_EXPECTED_VEHICLES:
            return wrapper->wrapInt(objID, variable, getMinExpectedNumber());
        case VAR_BUS_STOP_ID_LIST:
            return wrapper->wrapStringList(objID, variable, getBusStopIDList());
        case VAR_BUS_STOP_WAITING:
            return wrapper->wrapInt(objID, variable, getBusStopWaiting(objID));
        case VAR_BUS_STOP_WAITING_IDS:
            return wrapper->wrapStringList(objID, variable, getBusStopWaitingIDList(objID));
        case VAR_PENDING_VEHICLES:
            return wrapper->wrapStringList(objID, variable, getPendingVehicles());
        case VAR_PARAMETER:
            return wrapper->wrapString(objID, variable, getParameter(objID, StoHelp::readTypedString(*paramData)));
        case VAR_PARAMETER_WITH_KEY:
            return wrapper->wrapStringPair(objID, variable, getParameterWithKey(objID, StoHelp::readTypedString(*paramData)));
        default:
            return false;
    }
}
}


/****************************************************************************/
