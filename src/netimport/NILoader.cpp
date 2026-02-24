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
/// @file    NILoader.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Sascha Krieg
/// @author  Michael Behrisch
/// @author  Robert Hilbrich
/// @date    Tue, 20 Nov 2001
///
// Perfoms network import
/****************************************************************************/
#include <config.h>

#include <string>
#include <utils/common/UtilExceptions.h>
#include <utils/common/MsgHandler.h>
#include <utils/options/OptionsCont.h>
#include <utils/options/Option.h>
#include <utils/common/FileHelpers.h>
#include <utils/common/StringUtils.h>
#include <utils/common/ToString.h>
#include <utils/common/StringUtils.h>
#include <utils/geom/GeoConvHelper.h>
#include <netbuild/NBTypeCont.h>
#include <netbuild/NBNodeCont.h>
#include <netbuild/NBEdgeCont.h>
#include <netbuild/NBHeightMapper.h>
#include <netbuild/NBNetBuilder.h>
#include <netimport/NIXMLEdgesHandler.h>
#include <netimport/NIXMLNodesHandler.h>
#include <netimport/NIXMLTrafficLightsHandler.h>
#include <netimport/NIXMLTypesHandler.h>
#include <netimport/NIXMLPTHandler.h>
#include <netimport/NIXMLShapeHandler.h>
#include <netimport/NIXMLConnectionsHandler.h>
#include <netimport/NIImporter_DlrNavteq.h>
#include <netimport/NIImporter_VISUM.h>
#include <netimport/vissim/NIImporter_Vissim.h>
#include <netimport/NIImporter_ArcView.h>
#include <netimport/NIImporter_SUMO.h>
#include <netimport/NIImporter_OpenStreetMap.h>
#include <netimport/NIImporter_OpenDrive.h>
#include <netimport/NIImporter_MATSim.h>
#include <netimport/NIImporter_ITSUMO.h>
#include <netimport/typemap.h>
#include "NILoader.h"
#include "NITypeLoader.h"

// ===========================================================================
// method definitions
// ===========================================================================
NILoader::NILoader(NBNetBuilder& nb)
    : myNetBuilder(nb) {}

NILoader::~NILoader() {}

void
NILoader::load(OptionsCont& oc) {
    bool ok = true;
    // load types first
    NIXMLTypesHandler handler(myNetBuilder.getTypeCont());
    if (!oc.isSet("type-files")) {
        std::vector<std::string> files;
        if (oc.isSet("osm-files")) {
            files.push_back(osmTypemap);
        }
        if (oc.isSet("opendrive-files")) {
            files.push_back(opendriveTypemap);
        }
        if (oc.isSet("visum-file")) {
            files.push_back(visumTypemap);
        }
        ok &= NITypeLoader::load(handler, files, toString(SUMO_TAG_TYPES), true);
    } else {
        ok &= NITypeLoader::load(handler, oc.getStringVector("type-files"), toString(SUMO_TAG_TYPES));
    }
    // try to load height data so it is ready for use by other importers
    NBHeightMapper::loadIfSet(oc);
    // try to load using different methods
    NIImporter_SUMO::loadNetwork(oc, myNetBuilder);
    NIImporter_OpenStreetMap::loadNetwork(oc, myNetBuilder);
    NIImporter_VISUM::loadNetwork(oc, myNetBuilder);
    NIImporter_ArcView::loadNetwork(oc, myNetBuilder);
    NIImporter_Vissim::loadNetwork(oc, myNetBuilder);
    NIImporter_DlrNavteq::loadNetwork(oc, myNetBuilder);
    NIImporter_OpenDrive::loadNetwork(oc, myNetBuilder);
    NIImporter_MATSim::loadNetwork(oc, myNetBuilder);
    NIImporter_ITSUMO::loadNetwork(oc, myNetBuilder);
    if (oc.getBool("tls.discard-loaded") || oc.getBool("tls.discard-simple")) {
        myNetBuilder.getNodeCont().discardTrafficLights(myNetBuilder.getTLLogicCont(), oc.getBool("tls.discard-simple"));
        int removed = myNetBuilder.getTLLogicCont().getNumExtracted();
        if (removed > 0) {
            WRITE_MESSAGEF(TL(" Removed % traffic lights before loading plain-XML"), toString(removed));
        }
    }
    if (oc.getBool("railway.signals.discard")) {
        myNetBuilder.getNodeCont().discardRailSignals();
    }
    ok &= loadXML(oc);
    // check the loaded structures
    if (myNetBuilder.getNodeCont().size() == 0) {
        throw ProcessError(TL("No nodes loaded."));
    }
    if (myNetBuilder.getEdgeCont().size() == 0) {
        throw ProcessError(TL("No edges loaded."));
    }
    if (!myNetBuilder.getEdgeCont().checkConsistency(myNetBuilder.getNodeCont())) {
        throw ProcessError();
    }
    if (!ok && !oc.getBool("ignore-errors")) {
        throw ProcessError();
    }
    // configure default values that depend on other values
    myNetBuilder.getNodeCont().applyConditionalDefaults();
    // report loaded structures
    WRITE_MESSAGE(TL(" Import done:"));
    if (myNetBuilder.getDistrictCont().size() > 0) {
        WRITE_MESSAGEF(TL("   % districts loaded."), toString(myNetBuilder.getDistrictCont().size()));
    }
    WRITE_MESSAGEF(TL("   % nodes loaded."), toString(myNetBuilder.getNodeCont().size()));
    if (myNetBuilder.getTypeCont().size() > 0) {
        WRITE_MESSAGEF(TL("   % types loaded."), toString(myNetBuilder.getTypeCont().size()));
    }
    WRITE_MESSAGEF(TL("   % edges loaded."), toString(myNetBuilder.getEdgeCont().size()));
    if (myNetBuilder.getEdgeCont().getNumEdgeSplits() > 0) {
        WRITE_MESSAGEF(TL("The split of edges was performed % times."), toString(myNetBuilder.getEdgeCont().getNumEdgeSplits()));
    }

    //TODO: uncomment the following lines + adapt tests! [Gregor March '17]
//  if (myNetBuilder.getPTStopCont().size() > 0) {
//    WRITE_MESSAGEF(TL("   % pt stops loaded."), toString(myNetBuilder.getPTStopCont().size()));
//  }
    if (GeoConvHelper::getProcessing().usingGeoProjection()) {
        WRITE_MESSAGEF(TL("Proj projection parameters used: '%'."), GeoConvHelper::getProcessing().getProjString());
    }
}

/* -------------------------------------------------------------------------
 * file loading methods
 * ----------------------------------------------------------------------- */
bool
NILoader::loadXML(OptionsCont& oc) {
    // load nodes
    NIXMLNodesHandler nodesHandler(myNetBuilder.getNodeCont(), myNetBuilder.getEdgeCont(),
                                   myNetBuilder.getTLLogicCont(), oc);
    bool ok = NITypeLoader::load(nodesHandler, oc.getStringVector("node-files"), "nodes");
    // load the edges
    if (ok) {
        NIXMLEdgesHandler edgesHandler(myNetBuilder.getNodeCont(), myNetBuilder.getEdgeCont(),
                                       myNetBuilder.getTypeCont(), myNetBuilder.getDistrictCont(),
                                       myNetBuilder.getTLLogicCont(), oc);
        ok = NITypeLoader::load(edgesHandler, oc.getStringVector("edge-files"), "edges");
    }
    if (!deprecatedVehicleClassesSeen.empty()) {
        WRITE_WARNINGF(TL("Deprecated vehicle class(es) '%' in input edge files."), toString(deprecatedVehicleClassesSeen));
    }
    // load the connections
    if (ok) {
        NIXMLConnectionsHandler connectionsHandler(myNetBuilder.getEdgeCont(),
                myNetBuilder.getNodeCont(), myNetBuilder.getTLLogicCont());
        ok = NITypeLoader::load(connectionsHandler, oc.getStringVector("connection-files"), "connections");
    }
    // load traffic lights (needs to come last, references loaded edges and connections)
    if (ok) {
        NIXMLTrafficLightsHandler tlHandler(myNetBuilder.getTLLogicCont(), myNetBuilder.getEdgeCont());
        ok = NITypeLoader::load(tlHandler, oc.getStringVector("tllogic-files"), "traffic lights");
    }

    // load public transport stops (used for restricting edge removal and as input when repairing railroad topology)
    if (ok && oc.exists("ptstop-files")) {
        NIXMLPTHandler ptHandler(myNetBuilder.getEdgeCont(),
                                 myNetBuilder.getPTStopCont(), myNetBuilder.getPTLineCont());
        ok = NITypeLoader::load(ptHandler, oc.getStringVector("ptstop-files"), "public transport stops");
    }

    // load public transport lines (used as input when repairing railroad topology)
    if (ok && oc.exists("ptline-files")) {
        NIXMLPTHandler ptHandler(myNetBuilder.getEdgeCont(),
                                 myNetBuilder.getPTStopCont(), myNetBuilder.getPTLineCont());
        ok = NITypeLoader::load(ptHandler, oc.getStringVector("ptline-files"), "public transport lines");
    }

    // load shapes for output formats that embed shape data
    if (ok && oc.exists("polygon-files")) {
        NIXMLShapeHandler shapeHandler(myNetBuilder.getShapeCont(), myNetBuilder.getEdgeCont());
        ok = NITypeLoader::load(shapeHandler, oc.getStringVector("polygon-files"), "polygon data");
    }
    return ok;
}


/****************************************************************************/
