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
/// @file    NLBuilder.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 9 Jul 2001
///
// The main interface for loading a microsim
/****************************************************************************/
#include <config.h>

#include <iostream>
#include <vector>
#include <string>
#include <map>

#include <utils/common/MsgHandler.h>
#include <utils/common/StringTokenizer.h>
#include <utils/common/SystemFrame.h>
#include <utils/options/Option.h>
#include <utils/options/OptionsCont.h>
#include <utils/options/OptionsIO.h>
#include <utils/common/StringUtils.h>
#include <utils/common/FileHelpers.h>
#include <utils/common/SysUtils.h>
#include <utils/common/ToString.h>
#include <utils/vehicle/SUMORouteLoaderControl.h>
#include <utils/vehicle/SUMORouteLoader.h>
#include <utils/xml/XMLSubSys.h>
#ifdef HAVE_FOX
#include <utils/foxtools/MsgHandlerSynchronized.h>
#endif
#include <libsumo/Helper.h>
#include <mesosim/MEVehicleControl.h>
#include <mesosim/METypeHandler.h>
#include <microsim/MSVehicleControl.h>
#include <microsim/MSVehicleTransfer.h>
#include <microsim/MSNet.h>
#include <microsim/devices/MSDevice.h>
#include <microsim/devices/MSDevice_ToC.h>
#include <microsim/devices/MSDevice_BTreceiver.h>
#include <microsim/devices/MSDevice_FCDReplay.h>
#include <microsim/MSEdgeControl.h>
#include <microsim/MSGlobals.h>
#include <microsim/output/MSDetectorControl.h>
#include <microsim/MSFrame.h>
#include <microsim/MSEdgeWeightsStorage.h>
#include <microsim/MSStateHandler.h>
#include <microsim/MSDriverState.h>
#include <microsim/trigger/MSTriggeredRerouter.h>
#include <traci-server/TraCIServer.h>

#include "NLHandler.h"
#include "NLNetShapeHandler.h"
#include "NLEdgeControlBuilder.h"
#include "NLJunctionControlBuilder.h"
#include "NLDetectorBuilder.h"
#include "NLTriggerBuilder.h"
#include "NLBuilder.h"


// ===========================================================================
// method definitions
// ===========================================================================
// ---------------------------------------------------------------------------
// NLBuilder::EdgeFloatTimeLineRetriever_EdgeWeight - methods
// ---------------------------------------------------------------------------
void
NLBuilder::EdgeFloatTimeLineRetriever_EdgeEffort::addEdgeWeight(const std::string& id,
        double value, double begTime, double endTime) const {
    MSEdge* edge = MSEdge::dictionary(id);
    if (edge != nullptr) {
        myNet.getWeightsStorage().addEffort(edge, begTime, endTime, value);
    } else {
        WRITE_ERRORF(TL("Trying to set the effort for the unknown edge '%'."), id);
    }
}


// ---------------------------------------------------------------------------
// NLBuilder::EdgeFloatTimeLineRetriever_EdgeTravelTime - methods
// ---------------------------------------------------------------------------
void
NLBuilder::EdgeFloatTimeLineRetriever_EdgeTravelTime::addEdgeWeight(const std::string& id,
        double value, double begTime, double endTime) const {
    MSEdge* edge = MSEdge::dictionary(id);
    if (edge != nullptr) {
        myNet.getWeightsStorage().addTravelTime(edge, begTime, endTime, value);
    } else {
        WRITE_ERRORF(TL("Trying to set the travel time for the unknown edge '%'."), id);
    }
}


// ---------------------------------------------------------------------------
// NLBuilder - methods
// ---------------------------------------------------------------------------
NLBuilder::NLBuilder(OptionsCont& oc,
                     MSNet& net,
                     NLEdgeControlBuilder& eb,
                     NLJunctionControlBuilder& jb,
                     NLDetectorBuilder& db,
                     NLHandler& xmlHandler)
    : myOptions(oc), myEdgeBuilder(eb), myJunctionBuilder(jb),
      myDetectorBuilder(db),
      myNet(net), myXMLHandler(xmlHandler) {}


NLBuilder::~NLBuilder() {}


bool
NLBuilder::build() {
    // try to build the net
    if (!load("net-file", true)) {
        return false;
    }
    if (myXMLHandler.networkVersion() == MMVersion(0, 0)) {
        throw ProcessError(TL("Invalid network, no network version declared."));
    }
    // check whether the loaded net agrees with the simulation options
    if ((myOptions.getBool("no-internal-links") || myOptions.getBool("mesosim")) && myXMLHandler.haveSeenInternalEdge() && myXMLHandler.haveSeenDefaultLength()) {
        WRITE_WARNING(TL("Network contains internal links which are ignored. Vehicles will 'jump' across junctions and thus underestimate route lengths and travel times."));
    }
    buildNet();
    if (myOptions.isSet("alternative-net-file")) {
        for (std::string fname : myOptions.getStringVector("alternative-net-file")) {
            const long before = PROGRESS_BEGIN_TIME_MESSAGE("Loading alternative net from '" + fname + "'");
            NLNetShapeHandler nsh(fname, myNet);
            if (!XMLSubSys::runParser(nsh, fname, true)) {
                WRITE_MESSAGE("Loading of alternative net failed.");
                return false;
            }
            nsh.sortInternalShapes();
            PROGRESS_TIME_MESSAGE(before);
        }
    }
    // @note on loading order constraints:
    // - additional-files before route-files and state-files due to referencing
    // - additional-files before weight-files since the latter might contain intermodal edge data and the intermodal net depends on the stops and public transport from the additionals

    bool stateBeginMismatch = false;
    if (myOptions.isSet("load-state")) {
        // first, load only the time
        const SUMOTime stateTime = MSStateHandler::MSStateTimeHandler::getTime(myOptions.getString("load-state"));
        if (myOptions.isDefault("begin")) {
            myOptions.set("begin", time2string(stateTime));
            if (TraCIServer::getInstance() != nullptr) {
                TraCIServer::getInstance()->stateLoaded(stateTime);
            }
        } else {
            if (stateTime != string2time(myOptions.getString("begin"))) {
                WRITE_WARNINGF(TL("State was written at a different time=% than the begin time %!"), time2string(stateTime), myOptions.getString("begin"));
                stateBeginMismatch = true;
            }
        }
        myNet.setCurrentTimeStep(string2time(myOptions.getString("begin")));
    }

    if (myOptions.getBool("junction-taz")) {
        // create a TAZ for every junction
        const MSJunctionControl& junctions = myNet.getJunctionControl();
        for (auto it = junctions.begin(); it != junctions.end(); it++) {
            const std::string sinkID = it->first + "-sink";
            const std::string sourceID = it->first + "-source";
            if (MSEdge::dictionary(sinkID) == nullptr && MSEdge::dictionary(sourceID) == nullptr) {
                // sink must be built and added before source
                MSEdge* sink = myEdgeBuilder.buildEdge(sinkID, SumoXMLEdgeFunc::CONNECTOR, "", "", "", -1, 0);
                MSEdge* source = myEdgeBuilder.buildEdge(sourceID, SumoXMLEdgeFunc::CONNECTOR, "", "", "", -1, 0);
                sink->setOtherTazConnector(source);
                source->setOtherTazConnector(sink);
                MSEdge::dictionary(sinkID, sink);
                MSEdge::dictionary(sourceID, source);
                sink->initialize(new std::vector<MSLane*>());
                source->initialize(new std::vector<MSLane*>());
                const MSJunction* junction = it->second;
                for (const MSEdge* edge : junction->getIncoming()) {
                    if (!edge->isInternal()) {
                        const_cast<MSEdge*>(edge)->addSuccessor(sink);
                    }
                }
                for (const MSEdge* edge : junction->getOutgoing()) {
                    if (!edge->isInternal()) {
                        source->addSuccessor(const_cast<MSEdge*>(edge));
                    }
                }
            } else {
                WRITE_WARNINGF(TL("A TAZ with id '%' already exists. Not building junction TAZ."), it->first)
            }
        }
    }
    // load meso edge types
    bool haveSeenMesoEdgeType = false;
    if (MSGlobals::gUseMesoSim) {
        if (myOptions.isSet("additional-files")) {
            haveSeenMesoEdgeType = loadMesoEdgeTypes("additional-files");
        }
        // meso segment building must be delayed until meso edge types have been read from additional files
        myNet.getEdgeControl().buildMesoSegments();
    }
    // load additional net elements (sources, detectors, ...)
    if (myOptions.isSet("additional-files")) {
        if (!load("additional-files")) {
            return false;
        }
        // load shapes with separate handler
        NLShapeHandler sh("", myNet.getShapeContainer());
        if (!ShapeHandler::loadFiles(myOptions.getStringVector("additional-files"), sh)) {
            return false;
        }
        if (myXMLHandler.haveSeenAdditionalSpeedRestrictions()) {
            myNet.getEdgeControl().setAdditionalRestrictions();
        }
        MSTriggeredRerouter::checkParkingRerouteConsistency();
    }
    if (MSGlobals::gUseMesoSim) {
        if (haveSeenMesoEdgeType || myXMLHandler.haveSeenTLSParams()) {
            for (MSTrafficLightLogic* tll : myNet.getTLSControl().getAllLogics()) {
                tll->initMesoTLSPenalties();
            }
        }
    }
    // init tls after all detectors have been loaded
    myJunctionBuilder.postLoadInitialization();
    // declare meandata set by options
    buildDefaultMeanData("edgedata-output", "DEFAULT_EDGEDATA", false);
    buildDefaultMeanData("lanedata-output", "DEFAULT_LANEDATA", true);

    if (stateBeginMismatch && myNet.getVehicleControl().getLoadedVehicleNo() > 0) {
        throw ProcessError(TL("Loading vehicles ahead of a state file is not supported. Correct --begin option or load vehicles with option --route-files"));
    }

    // load weights if wished
    if (myOptions.isSet("weight-files")) {
        if (!myOptions.isUsableFileList("weight-files")) {
            return false;
        }
        // build and prepare the weights handler
        std::vector<SAXWeightsHandler::ToRetrieveDefinition*> retrieverDefs;
        //  travel time, first (always used)
        EdgeFloatTimeLineRetriever_EdgeTravelTime ttRetriever(myNet);
        retrieverDefs.push_back(new SAXWeightsHandler::ToRetrieveDefinition("traveltime", true, ttRetriever));
        //  the measure to use, then
        EdgeFloatTimeLineRetriever_EdgeEffort eRetriever(myNet);
        std::string measure = myOptions.getString("weight-attribute");
        if (!myOptions.isDefault("weight-attribute")) {
            if (measure == "CO" || measure == "CO2" || measure == "HC" || measure == "PMx" || measure == "NOx" || measure == "fuel" || measure == "electricity") {
                measure += "_perVeh";
            }
            retrieverDefs.push_back(new SAXWeightsHandler::ToRetrieveDefinition(measure, true, eRetriever));
        }
        //  set up handler
        SAXWeightsHandler handler(retrieverDefs, "");
        // start parsing; for each file in the list
        std::vector<std::string> files = myOptions.getStringVector("weight-files");
        for (std::vector<std::string>::iterator i = files.begin(); i != files.end(); ++i) {
            // report about loading when wished
            WRITE_MESSAGEF(TL("Loading weights from '%'..."), *i);
            // parse the file
            if (!XMLSubSys::runParser(handler, *i)) {
                return false;
            }
        }
    }
    // load the previous state if wished
    if (myOptions.isSet("load-state")) {
        const std::string& f = myOptions.getString("load-state");
        long before = PROGRESS_BEGIN_TIME_MESSAGE(TLF("Loading state from '%'", f));
        MSStateHandler h(f, string2time(myOptions.getString("load-state.offset")));
        XMLSubSys::runParser(h, f);
        if (MsgHandler::getErrorInstance()->wasInformed()) {
            return false;
        }
        PROGRESS_TIME_MESSAGE(before);
    }
    // routes from FCD files
    MSDevice_FCDReplay::init();
    // load routes
    if (myOptions.isSet("route-files")) {
        if (string2time(myOptions.getString("route-steps")) <= 0) {
            // incremental loading is disabled. Load route files fully
            if (!load("route-files")) {
                return false;
            }
        } else {
            // message must come after additional-files have been loaded (but buildRouteLoaderControl was called earlier)
            for (std::string file : myOptions.getStringVector("route-files")) {
                WRITE_MESSAGE(TLF("Loading route-files incrementally from '%'", file));
            }
        }
    }
    // optionally switch off traffic lights
    if (myOptions.getBool("tls.all-off")) {
        myNet.getTLSControl().switchOffAll();
    }
    WRITE_MESSAGE(TL("Loading done."));
    return true;
}


MSNet*
NLBuilder::init(const bool isLibsumo) {
    OptionsCont& oc = OptionsCont::getOptions();
    oc.clear();
    MSFrame::fillOptions();
    OptionsIO::getOptions();
    if (oc.processMetaOptions(OptionsIO::getArgC() < 2)) {
        SystemFrame::close();
        return nullptr;
    }
    SystemFrame::checkOptions(oc);
    std::string validation = oc.getString("xml-validation");
    std::string routeValidation = oc.getString("xml-validation.routes");
    if (isLibsumo) {
        if (oc.isDefault("xml-validation")) {
            validation = "never";
        }
        if (oc.isDefault("xml-validation.routes")) {
            routeValidation = "never";
        }
    }
    XMLSubSys::setValidation(validation, oc.getString("xml-validation.net"), routeValidation);
    if (!MSFrame::checkOptions()) {
        throw ProcessError();
    }
#ifdef HAVE_FOX
    if (oc.getInt("threads") > 1) {
        // make the output aware of threading
        MsgHandler::setFactory(&MsgHandlerSynchronized::create);
    }
#endif
    MsgHandler::initOutputOptions();
    initRandomness();
    MSFrame::setMSGlobals(oc);
    MSVehicleControl* vc = nullptr;
    if (MSGlobals::gUseMesoSim) {
        vc = new MEVehicleControl();
    } else {
        vc = new MSVehicleControl();
    }
    MSNet* net = new MSNet(vc, new MSEventControl(), new MSEventControl(), new MSEventControl());
    // need to init TraCI-Server before loading routes to catch VehicleState::BUILT
    TraCIServer::openSocket(std::map<int, TraCIServer::CmdExecutor>());
    if (isLibsumo) {
        libsumo::Helper::registerStateListener();
    }

    NLEdgeControlBuilder eb;
    NLDetectorBuilder db(*net);
    NLJunctionControlBuilder jb(*net, db);
    NLTriggerBuilder tb;
    NLHandler handler("", *net, db, tb, eb, jb);
    tb.setHandler(&handler);
    NLBuilder builder(oc, *net, eb, jb, db, handler);
    MsgHandler::getErrorInstance()->clear();
    MsgHandler::getWarningInstance()->clear();
    MsgHandler::getMessageInstance()->clear();
    if (builder.build()) {
        // preload the routes especially for TraCI
        net->loadRoutes();
        return net;
    }
    delete net;
    throw ProcessError();
}


void
NLBuilder::initRandomness() {
    RandHelper::initRandGlobal();
    RandHelper::initRandGlobal(MSRouteHandler::getParsingRNG());
    RandHelper::initRandGlobal(MSDevice::getEquipmentRNG());
    RandHelper::initRandGlobal(OUProcess::getRNG());
    RandHelper::initRandGlobal(MSDevice_ToC::getResponseTimeRNG());
    RandHelper::initRandGlobal(MSDevice_BTreceiver::getRecognitionRNG());
    MSLane::initRNGs(OptionsCont::getOptions());
}


void
NLBuilder::buildNet() {
    MSEdgeControl* edges = nullptr;
    MSJunctionControl* junctions = nullptr;
    SUMORouteLoaderControl* routeLoaders = nullptr;
    MSTLLogicControl* tlc = nullptr;
    std::vector<SUMOTime> stateDumpTimes;
    std::vector<std::string> stateDumpFiles;
    try {
        MSFrame::buildStreams(); // ensure streams are ready for output during building
        edges = myEdgeBuilder.build(myXMLHandler.networkVersion());
        junctions = myJunctionBuilder.build();
        junctions->postloadInitContainer();
        for (MSEdge* e : edges->getEdges()) {
            e->postLoadInitLaneChanger();
        }
        routeLoaders = buildRouteLoaderControl(myOptions);
        tlc = myJunctionBuilder.buildTLLogics();
        for (std::string timeStr : myOptions.getStringVector("save-state.times")) {
            stateDumpTimes.push_back(string2time(timeStr));
        }
        if (myOptions.isSet("save-state.files")) {
            stateDumpFiles = myOptions.getStringVector("save-state.files");
            if (stateDumpFiles.size() != stateDumpTimes.size()) {
                throw ProcessError(TL("Wrong number of state file names!"));
            }
        } else {
            const std::string prefix = myOptions.getString("save-state.prefix");
            const std::string suffix = myOptions.getString("save-state.suffix");
            for (std::vector<SUMOTime>::iterator i = stateDumpTimes.begin(); i != stateDumpTimes.end(); ++i) {
                std::string timeStamp = time2string(*i);
                std::replace(timeStamp.begin(), timeStamp.end(), ':', '-');
                stateDumpFiles.push_back(prefix + "_" + timeStamp + suffix);
            }
        }
    } catch (ProcessError&) {
        MSEdge::clear();
        MSLane::clear();
        delete edges;
        delete junctions;
        delete routeLoaders;
        delete tlc;
        throw;
    }
    // if anthing goes wrong after this point, the net is responsible for cleaning up
    myNet.closeBuilding(myOptions, edges, junctions, routeLoaders, tlc, stateDumpTimes, stateDumpFiles,
                        myXMLHandler.haveSeenInternalEdge(),
                        myXMLHandler.hasJunctionHigherSpeeds(),
                        myXMLHandler.networkVersion());
}


bool
NLBuilder::load(const std::string& mmlWhat, const bool isNet) {
    if (!myOptions.isUsableFileList(mmlWhat)) {
        return false;
    }
    std::vector<std::string> files = myOptions.getStringVector(mmlWhat);
    for (std::vector<std::string>::const_iterator fileIt = files.begin(); fileIt != files.end(); ++fileIt) {
        const long before = PROGRESS_BEGIN_TIME_MESSAGE(TLF("Loading % from '%'", mmlWhat, *fileIt));
        if (!XMLSubSys::runParser(myXMLHandler, *fileIt, isNet)) {
            WRITE_MESSAGEF(TL("Loading of % failed."), mmlWhat);
            return false;
        }
        PROGRESS_TIME_MESSAGE(before);
    }
    return true;
}


bool
NLBuilder::loadMesoEdgeTypes(const std::string& mmlWhat) {
    if (!myOptions.isUsableFileList(mmlWhat)) {
        return false;
    }
    METypeHandler meTypeHandler("", myNet);
    for (const std::string& file : myOptions.getStringVector(mmlWhat)) {
        if (!XMLSubSys::runParser(meTypeHandler, file)) {
            continue;
        }
    }
    return meTypeHandler.haveSeenMesoEdgeType();
}


SUMORouteLoaderControl*
NLBuilder::buildRouteLoaderControl(const OptionsCont& oc) {
    // build the loaders
    SUMORouteLoaderControl* loaders =  new SUMORouteLoaderControl(string2time(oc.getString("route-steps")));
    // check whether a list is existing
    if (oc.isSet("route-files") && string2time(oc.getString("route-steps")) > 0) {
        std::vector<std::string> files = oc.getStringVector("route-files");
        for (std::vector<std::string>::const_iterator fileIt = files.begin(); fileIt != files.end(); ++fileIt) {
            if (!FileHelpers::isReadable(*fileIt)) {
                throw ProcessError(TLF("The route file '%' is not accessible.", *fileIt));
            }
        }
        // open files for reading
        for (std::vector<std::string>::const_iterator fileIt = files.begin(); fileIt != files.end(); ++fileIt) {
            loaders->add(new SUMORouteLoader(new MSRouteHandler(*fileIt, false)));
        }
    }
    return loaders;
}


void
NLBuilder::buildDefaultMeanData(const std::string& optionName, const std::string& id, bool useLanes) {
    if (OptionsCont::getOptions().isSet(optionName)) {
        if (useLanes && MSGlobals::gUseMesoSim && !OptionsCont::getOptions().getBool("meso-lane-queue")) {
            WRITE_WARNING(TL("LaneData requested for mesoscopic simulation but --meso-lane-queue is not active. Falling back to edgeData."));
            useLanes = false;
        }
        try {
            SUMOTime begin = string2time(OptionsCont::getOptions().getString("begin"));
            myDetectorBuilder.createEdgeLaneMeanData(id, -1, begin, -1, "traffic", useLanes, "true",
                    false, false, 0, 100000, 0, SUMO_const_haltingSpeed, "", "", std::vector<MSEdge*>(), AggregateType::NO,
                    OptionsCont::getOptions().getString(optionName));
        } catch (InvalidArgument& e) {
            WRITE_ERROR(e.what());
        } catch (IOError& e) {
            WRITE_ERROR(e.what());
        }
    }
}

/****************************************************************************/
