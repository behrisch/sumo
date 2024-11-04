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
/// @file    NIFrame.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @author  Gregor Laemmel
/// @date    Tue, 20 Nov 2001
///
// Sets and checks options for netimport
/****************************************************************************/
#include <config.h>

#include <string>
#include <iostream>
#include <fstream>
#include <utils/options/Option.h>
#include <utils/options/OptionsCont.h>
#include <utils/options/OptionsIO.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/ToString.h>
#include <utils/common/FileHelpers.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/RandHelper.h>
#include <netbuild/NBNetBuilder.h>
#include <netwrite/NWFrame.h>
#include <utils/common/SystemFrame.h>
#include "NIImporter_DlrNavteq.h"
#include "NIFrame.h"


// ===========================================================================
// method definitions
// ===========================================================================

void
NIFrame::fillOptions(OptionsCont& oc, bool forNetedit) {
    // register input formats
    oc.doRegister("sumo-net-file", 's', new Option_FileName());
    oc.addSynonyme("sumo-net-file", "sumo-net", true);
    oc.addSynonyme("sumo-net-file", "net-file");
    oc.addDescription("sumo-net-file", "Input", TL("Read SUMO-net from FILE"));
    oc.addXMLDefault("sumo-net-file", "net");

    oc.doRegister("node-files", 'n', new Option_FileName());
    oc.addSynonyme("node-files", "xml-node-files", true);
    oc.addSynonyme("node-files", "xml-nodes", true);
    oc.addSynonyme("node-files", "nodes");
    oc.addDescription("node-files", "Input", TL("Read XML-node defs from FILE"));

    oc.doRegister("edge-files", 'e', new Option_FileName());
    oc.addSynonyme("edge-files", "xml-edge-files", true);
    oc.addSynonyme("edge-files", "xml-edges", true);
    oc.addSynonyme("edge-files", "edges");
    oc.addDescription("edge-files", "Input", TL("Read XML-edge defs from FILE"));

    oc.doRegister("connection-files", 'x', new Option_FileName());
    oc.addSynonyme("connection-files", "xml-connection-files", true);
    oc.addSynonyme("connection-files", "xml-connections", true);
    oc.addSynonyme("connection-files", "connections");
    oc.addDescription("connection-files", "Input", TL("Read XML-connection defs from FILE"));

    oc.doRegister("tllogic-files", 'i', new Option_FileName());
    oc.addDescription("tllogic-files", "Input", TL("Read XML-traffic light defs from FILE"));

    oc.doRegister("type-files", 't', new Option_FileName());
    oc.addSynonyme("type-files", "xml-type-files", true);
    oc.addSynonyme("type-files", "xml-types", true);
    oc.addSynonyme("type-files", "types");
    oc.addDescription("type-files", "Input", TL("Read XML-type defs from FILE"));

    if (!forNetedit) {
        // would cause confusion because netedit loads stops and shapes using option --additional-files
        oc.doRegister("ptstop-files", new Option_FileName());
        oc.addDescription("ptstop-files", "Input", TL("Reads public transport stops from FILE"));
        oc.doRegister("ptline-files", new Option_FileName());
        oc.addDescription("ptline-files", "Input", TL("Reads public transport lines from FILE"));
        oc.doRegister("polygon-files", new Option_FileName());
        oc.addDescription("polygon-files", "Input", TL("Reads polygons from FILE for embedding in network where applicable"));
    }

    oc.doRegister("shapefile-prefix", new Option_FileName());
    oc.addSynonyme("shapefile-prefix", "shapefile");
    oc.addSynonyme("shapefile-prefix", "arcview", true);
    oc.addSynonyme("shapefile-prefix", "tiger", true);
    oc.addDescription("shapefile-prefix", "Input", TL("Read shapefiles (ArcView, Tiger, ...) from files starting with 'FILE'"));

    oc.doRegister("dlr-navteq-prefix", new Option_FileName());
    oc.addSynonyme("dlr-navteq-prefix", "dlr-navteq");
    oc.addSynonyme("dlr-navteq-prefix", "elmar2", true);
    oc.addDescription("dlr-navteq-prefix", "Input", TL("Read converted Navteq GDF data (unsplitted Elmar-network) from path 'FILE'"));

    oc.doRegister("osm-files", new Option_FileName());
    oc.addSynonyme("osm-files", "osm");
    oc.addDescription("osm-files", "Input", TL("Read OSM-network from path 'FILE(s)'"));

    oc.doRegister("opendrive-files", new Option_FileName());
    oc.addSynonyme("opendrive-files", "opendrive");
    oc.addDescription("opendrive-files", "Input", TL("Read OpenDRIVE-network from FILE"));

    oc.doRegister("visum-file", new Option_FileName());
    oc.addSynonyme("visum-file", "visum");
    oc.addDescription("visum-file", "Input", TL("Read VISUM-net from FILE"));

    oc.doRegister("vissim-file", new Option_FileName());
    oc.addSynonyme("vissim-file", "vissim");
    oc.addDescription("vissim-file", "Input", TL("Read VISSIM-net from FILE"));

    oc.doRegister("robocup-dir", new Option_FileName());
    oc.addSynonyme("robocup-dir", "robocup-net", true);
    oc.addSynonyme("robocup-dir", "robocup");
    oc.addDescription("robocup-dir", "Input", TL("Read RoboCup-net from DIR"));

    oc.doRegister("matsim-files", new Option_FileName());
    oc.addSynonyme("matsim-files", "matsim");
    oc.addDescription("matsim-files", "Input", TL("Read MATSim-net from FILE"));

    oc.doRegister("itsumo-files", new Option_FileName());
    oc.addSynonyme("itsumo-files", "itsumo");
    oc.addDescription("itsumo-files", "Input", TL("Read ITSUMO-net from FILE"));

    oc.doRegister("heightmap.shapefiles", new Option_FileName());
    oc.addDescription("heightmap.shapefiles", "Input", TL("Read heightmap from ArcGIS shapefile"));

    oc.doRegister("heightmap.geotiff", new Option_FileName());
    oc.addDescription("heightmap.geotiff", "Input", TL("Read heightmap from GeoTIFF"));

    // need to do this here to be able to check for network and route input options
    SystemFrame::addReportOptions(oc);

    // register basic processing options
    oc.doRegister("ignore-errors", new Option_Bool(false));
    oc.addSynonyme("ignore-errors", "dismiss-loading-errors", true);
    oc.addDescription("ignore-errors", "Report", TL("Continue on broken input"));

    oc.doRegister("ignore-errors.connections", new Option_Bool(false));
    oc.addDescription("ignore-errors.connections", "Report", TL("Continue on invalid connections"));

    oc.doRegister("show-errors.connections-first-try", new Option_Bool(false));
    oc.addDescription("show-errors.connections-first-try", "Report", TL("Show errors in connections at parsing"));

    oc.doRegister("ignore-errors.edge-type", new Option_Bool(false));
    oc.addDescription("ignore-errors.edge-type", "Report", TL("Continue on unknown edge types"));

    oc.doRegister("speed-in-kmh", new Option_Bool(false));
    oc.addDescription("speed-in-kmh", "Processing", TL("vmax is parsed as given in km/h"));

    oc.doRegister("construction-date", new Option_String());
    oc.addDescription("construction-date", "Processing", TL("Use YYYY-MM-DD date to determine the readiness of features under construction"));

    oc.doRegister("flatten", new Option_Bool(false));
    oc.addDescription("flatten", "Processing", TL("Remove all z-data"));

    oc.doRegister("discard-params", new Option_StringVector());
    oc.addDescription("discard-params", "Formats", TL("Remove the list of keys from all params"));

    oc.doRegister("ignore-change-restrictions", new Option_StringVector(StringVector({"authority"})));
    oc.addDescription("ignore-change-restrictions", "Formats", TL("List vehicle classes that may ignore lane changing restrictions ('all' discards all restrictions)"));

    oc.doRegister("ignore-widths", new Option_Bool(false));
    oc.addSynonyme("ignore-widths", "opendrive.ignore-widths", false);
    oc.addDescription("ignore-widths", "Formats", TL("Whether lane widths shall be ignored."));

    // register xml options
    oc.doRegister("plain.extend-edge-shape", new Option_Bool(false));
    oc.addSynonyme("plain.extend-edge-shape", "xml.keep-shape", true);
    oc.addDescription("plain.extend-edge-shape", "Processing", TL("If edge shapes do not end at the node positions, extend them"));

    // register osm options
    oc.doRegister("osm.skip-duplicates-check", new Option_Bool(false));
    oc.addDescription("osm.skip-duplicates-check", "Formats", TL("Skips the check for duplicate nodes and edges"));

    oc.doRegister("osm.elevation", new Option_Bool(false));
    oc.addDescription("osm.elevation", "Formats", TL("Imports elevation data"));

    oc.doRegister("osm.layer-elevation", new Option_Float(0));
    oc.addDescription("osm.layer-elevation", "Formats", TL("Reconstruct (relative) elevation based on layer data. Each layer is raised by FLOAT m"));

    oc.doRegister("osm.layer-elevation.max-grade", new Option_Float(10));
    oc.addDescription("osm.layer-elevation.max-grade", "Formats", TL("Maximum grade threshold in % at 50km/h when reconstructing elevation based on layer data. The value is scaled according to road speed."));

    oc.doRegister("osm.oneway-spread-right", new Option_Bool(false));
    oc.addDescription("osm.oneway-spread-right", "Formats", TL("Whether one-way roads should be spread to the side instead of centered"));

    oc.doRegister("osm.lane-access", new Option_Bool(false));
    oc.addDescription("osm.lane-access", "Formats", TL("Import lane-specific access restrictions"));

    oc.doRegister("osm.bike-access", new Option_Bool(false));
    oc.addSynonyme("osm.bike-access", "osm.bike-lanes");
    oc.addDescription("osm.bike-access", "Formats", TL("Import bike lanes and fix directions and permissions on bike paths"));

    oc.doRegister("osm.sidewalks", new Option_Bool(false));
    oc.addDescription("osm.sidewalks", "Formats", TL("Import sidewalks"));

    oc.doRegister("osm.crossings", new Option_Bool(false));
    oc.addDescription("osm.crossings", "Formats", TL("Import crossings"));

    oc.doRegister("osm.turn-lanes", new Option_Bool(false));
    oc.addDescription("osm.turn-lanes", "Formats", TL("Import turning arrows from OSM to help with connection building"));

    oc.doRegister("osm.stop-output.length", new Option_Float(25));
    oc.addDescription("osm.stop-output.length", "Formats", TL("The default length of a public transport stop in FLOAT m"));
    oc.doRegister("osm.stop-output.length.bus", new Option_Float(15));
    oc.addDescription("osm.stop-output.length.bus", "Formats", TL("The default length of a bus stop in FLOAT m"));
    oc.doRegister("osm.stop-output.length.tram", new Option_Float(25));
    oc.addDescription("osm.stop-output.length.tram", "Formats", TL("The default length of a tram stop in FLOAT m"));
    oc.doRegister("osm.stop-output.length.train", new Option_Float(200));
    oc.addDescription("osm.stop-output.length.train", "Formats", TL("The default length of a train stop in FLOAT m"));

    oc.doRegister("osm.railsignals", new Option_StringVector(StringVector({ "DEFAULT"})));
    oc.addDescription("osm.railsignals", "Formats", TL("Specify custom rules for importing railway signals"));

    oc.doRegister("osm.all-attributes", new Option_Bool(false));
    oc.addSynonyme("osm.all-attributes", "osm.all-tags");
    oc.addDescription("osm.all-attributes", "Formats", TL("Whether additional attributes shall be imported"));

    oc.doRegister("osm.extra-attributes", new Option_StringVector(StringVector({ "all" })));
    oc.addSynonyme("osm.extra-attributes", "osm.extra-tags");
    oc.addDescription("osm.extra-attributes", "Formats", TL("List of additional attributes that shall be imported from OSM via osm.all-attributes (set 'all' to import all)"));

    oc.doRegister("osm.speedlimit-none", new Option_Float(39.4444));
    oc.addDescription("osm.speedlimit-none", "Formats", TL("The speed limit to be set when there is no actual speed limit in reality"));

    // register matsim options
    oc.doRegister("matsim.keep-length", new Option_Bool(false));
    oc.addDescription("matsim.keep-length", "Formats", TL("The edge lengths given in the MATSim-file will be kept"));

    oc.doRegister("matsim.lanes-from-capacity", new Option_Bool(false));
    oc.addDescription("matsim.lanes-from-capacity", "Formats", TL("The lane number will be computed from the capacity"));


    // register shapefile options
    oc.doRegister("shapefile.street-id", new Option_String());
    oc.addSynonyme("shapefile.street-id", "arcview.street-id", true);
    oc.addDescription("shapefile.street-id", "Formats", TL("Read edge ids from column STR"));

    oc.doRegister("shapefile.from-id", new Option_String());
    oc.addSynonyme("shapefile.from-id", "arcview.from-id", true);
    oc.addDescription("shapefile.from-id", "Formats", TL("Read from-node ids from column STR"));

    oc.doRegister("shapefile.to-id", new Option_String());
    oc.addSynonyme("shapefile.to-id", "arcview.to-id", true);
    oc.addDescription("shapefile.to-id", "Formats", TL("Read to-node ids from column STR"));

    oc.doRegister("shapefile.type-id", new Option_String());
    oc.addSynonyme("shapefile.type-id", "arcview.type-id", true);
    oc.addDescription("shapefile.type-id", "Formats", TL("Read type ids from column STR"));

    oc.doRegister("shapefile.laneNumber", new Option_String());
    oc.addDescription("shapefile.laneNumber", "Formats", TL("Read lane number from column STR"));

    oc.doRegister("shapefile.speed", new Option_String());
    oc.addDescription("shapefile.speed", "Formats", TL("Read speed from column STR"));

    oc.doRegister("shapefile.length", new Option_String());
    oc.addDescription("shapefile.length", "Formats", TL("Read custom edge length from column STR"));

    oc.doRegister("shapefile.width", new Option_String());
    oc.addDescription("shapefile.width", "Formats", TL("Read total edge width from column STR"));

    oc.doRegister("shapefile.name", new Option_String());
    oc.addDescription("shapefile.name", "Formats", TL("Read (non-unique) name from column STR"));

    oc.doRegister("shapefile.node-join-dist", new Option_Float(0));
    oc.addDescription("shapefile.node-join-dist", "Formats", TL("Distance threshold for determining whether distinct shapes are connected (used when from-id and to-id are not available)"));

    oc.doRegister("shapefile.add-params", new Option_StringVector());
    oc.addDescription("shapefile.add-params", "Formats", TL("Add the list of field names as edge params"));

    oc.doRegister("shapefile.use-defaults-on-failure", new Option_Bool(false));
    oc.addSynonyme("shapefile.use-defaults-on-failure", "arcview.use-defaults-on-failure", true);
    oc.addDescription("shapefile.use-defaults-on-failure", "Formats", TL("Uses edge type defaults on problems"));

    oc.doRegister("shapefile.all-bidirectional", new Option_Bool(false));
    oc.addSynonyme("shapefile.all-bidirectional", "shapefile.all-bidi");
    oc.addSynonyme("shapefile.all-bidirectional", "arcview.all-bidi", true);
    oc.addDescription("shapefile.all-bidirectional", "Formats", TL("Insert edges in both directions"));

    oc.doRegister("shapefile.guess-projection", new Option_Bool(false));
    oc.addSynonyme("shapefile.guess-projection", "arcview.guess-projection", true);
    oc.addDescription("shapefile.guess-projection", "Formats", TL("Guess the proper projection"));

    oc.doRegister("shapefile.traditional-axis-mapping", new Option_Bool(false));
    oc.addDescription("shapefile.traditional-axis-mapping", "Formats", TL("Use traditional axis order (lon, lat)"));


    // register dlr-navteq options
    oc.doRegister("dlr-navteq.tolerant-permissions", new Option_Bool(false));
    oc.addDescription("dlr-navteq.tolerant-permissions", "Formats", TL("Allow more vehicle classes by default"));

    oc.doRegister("dlr-navteq.keep-length", new Option_Bool(false));
    oc.addDescription("dlr-navteq.keep-length", "Formats", TL("The edge lengths given in the DLR Navteq-file will be kept"));


    // register vissim options
    oc.doRegister("vissim.join-distance", new Option_Float(5.0f));
    oc.addSynonyme("vissim.join-distance", "vissim.offset", true);
    oc.addDescription("vissim.join-distance", "Formats", TL("Structure join offset"));

    oc.doRegister("vissim.default-speed", new Option_Float(50.0f));
    oc.addDescription("vissim.default-speed", "Formats", TL("Use FLOAT as default speed"));

    oc.doRegister("vissim.speed-norm", new Option_Float(1.0f));
    oc.addDescription("vissim.speed-norm", "Formats", TL("Factor for edge velocity"));

    oc.doRegister("vissim.report-unset-speeds", new Option_Bool(false));
    oc.addDescription("vissim.report-unset-speeds", "Formats", TL("Writes lanes without an explicit speed set"));


    // register visum options
    oc.doRegister("visum.language-file", new Option_FileName());
    oc.addDescription("visum.language-file", "Formats", TL("Load language mappings from FILE"));

    oc.doRegister("visum.use-type-priority", new Option_Bool(false));
    oc.addDescription("visum.use-type-priority", "Formats", TL("Uses priorities from types"));

    oc.doRegister("visum.use-type-laneno", new Option_Bool(false));
    oc.addDescription("visum.use-type-laneno", "Formats", TL("Uses lane numbers from types"));

    oc.doRegister("visum.use-type-speed", new Option_Bool(false));
    oc.addDescription("visum.use-type-speed", "Formats", TL("Uses speeds from types"));

    oc.doRegister("visum.connector-speeds", new Option_Float(100.));
    oc.addDescription("visum.connector-speeds", "Formats", TL("Sets connector speed"));

    oc.doRegister("visum.connectors-lane-number", new Option_Integer(3));
    oc.addSynonyme("visum.connectors-lane-number", "visum.connector-laneno", true);
    oc.addDescription("visum.connectors-lane-number", "Formats", TL("Sets connector lane number"));

    oc.doRegister("visum.no-connectors", new Option_Bool(true));
    oc.addDescription("visum.no-connectors", "Formats", TL("Excludes connectors"));

    oc.doRegister("visum.recompute-lane-number", new Option_Bool(false));
    oc.addSynonyme("visum.recompute-lane-number", "visum.recompute-laneno", true);
    oc.addDescription("visum.recompute-lane-number", "Formats", TL("Computes the number of lanes from the edges' capacities"));

    oc.doRegister("visum.verbose-warnings", new Option_Bool(false));
    oc.addDescription("visum.verbose-warnings", "Formats", TL("Prints all warnings, some of which are due to VISUM misbehaviour"));

    oc.doRegister("visum.lanes-from-capacity.norm", new Option_Float(1800.));
    oc.addSynonyme("visum.lanes-from-capacity.norm", "capacity-norm", true);
    oc.addSynonyme("visum.lanes-from-capacity.norm", "lanes-from-capacity.norm");
    oc.addDescription("visum.lanes-from-capacity.norm", "Formats", TL("The factor for flow to no. lanes conversion"));


    // register opendrive options
    oc.doRegister("opendrive.import-all-lanes", new Option_Bool(false));
    oc.addDescription("opendrive.import-all-lanes", "Formats", TL("Imports all lane types"));
    oc.doRegister("opendrive.curve-resolution", new Option_Float(2.0));
    oc.addDescription("opendrive.curve-resolution", "Formats", TL("The geometry resolution in m when importing curved geometries as line segments."));
    oc.doRegister("opendrive.advance-stopline", new Option_Float(0.0));
    oc.addDescription("opendrive.advance-stopline", "Formats", TL("Allow stop lines to be built beyond the start of the junction if the geometries allow so"));
    oc.doRegister("opendrive.min-width", new Option_Float(1.8));
    oc.addDescription("opendrive.min-width", "Formats", TL("The minimum lane width for determining start or end of variable-width lanes"));
    oc.doRegister("opendrive.internal-shapes", new Option_Bool(false));
    oc.addDescription("opendrive.internal-shapes", "Formats", TL("Import internal lane shapes"));
    oc.doRegister("opendrive.position-ids", new Option_Bool(false));
    oc.addDescription("opendrive.position-ids", "Formats", TL("Sets edge-id based on road-id and offset in m (legacy)"));
    oc.doRegister("opendrive.lane-shapes", new Option_Bool(false));
    oc.addDescription("opendrive.lane-shapes", "Formats", TL("Use custom lane shapes to compensate discarded lane types"));
    oc.doRegister("opendrive.signal-groups", new Option_Bool(false));
    oc.addDescription("opendrive.signal-groups", "Formats", TL("Use the OpenDRIVE controller information for the generated signal program"));
    oc.doRegister("opendrive.ignore-misplaced-signals", new Option_Bool(false));
    oc.addDescription("opendrive.ignore-misplaced-signals", "Formats", TL("Ignore traffic signals which do not control any driving lane"));

    // register some additional options
    oc.doRegister("tls.discard-loaded", new Option_Bool(false));
    oc.addDescription("tls.discard-loaded", "TLS Building", "Does not instantiate traffic lights loaded from other formats than plain-XML");

    oc.doRegister("tls.discard-simple", new Option_Bool(false));
    oc.addDescription("tls.discard-simple", "TLS Building", "Does not instantiate traffic lights at geometry-like nodes loaded from other formats than plain-XML");

    // register railway options
    oc.doRegister("railway.signals.discard", new Option_Bool(false));
    oc.addDescription("railway.signals.discard", "Railway", TL("Discard all railway signal information loaded from other formats than plain-xml"));
}


bool
NIFrame::checkOptions(OptionsCont& oc) {
    bool ok = oc.checkDependingSuboptions("shapefile", "shapefile.");
    ok &= oc.checkDependingSuboptions("visum-file", "visum.");
    ok &= oc.checkDependingSuboptions("vissim-file", "vissim.");
#ifdef PROJ_API_FILE
    int numProjections = oc.getBool("simple-projection") + oc.getBool("proj.utm") + oc.getBool("proj.dhdn") + (oc.getString("proj").length() > 1);
    if ((oc.isSet("osm-files") || oc.isSet("dlr-navteq-prefix") || oc.isSet("shapefile-prefix")) && numProjections == 0) {
        if (oc.isDefault("proj")) {
            oc.set("proj.utm", "true");
        }
    }
    if (oc.isSet("dlr-navteq-prefix") && oc.isDefault("proj.scale")) {
        oc.setDefault("proj.scale", NIImporter_DlrNavteq::GEO_SCALE);
    }
#else
    if ((oc.isSet("osm-files") || oc.isSet("dlr-navteq-prefix") || oc.isSet("shapefile-prefix")) && !oc.getBool("simple-projection")) {
        WRITE_ERROR(TL("Cannot import network data without PROJ-Library. Please install package proj before building sumo"));
        ok = false;
    }
#endif
    if (oc.isSet("sumo-net-file")) {
        if (oc.isWriteable("no-turnarounds")) {
            // changed default since turnarounds are loaded from the net file.
            oc.set("no-turnarounds", "true");
        }
        if (oc.isWriteable("offset.disable-normalization")) {
            // changed default since we wish to preserve the network as far as possible
            oc.set("offset.disable-normalization", "true");
        }
        if (oc.isWriteable("geometry.max-grade.fix")) {
            // changed default since we wish to preserve the network as far as possible
            oc.set("geometry.max-grade.fix", "false");
        }
        if (oc.isWriteable("geometry.min-radius.fix.railways")) {
            // changed default since we wish to preserve the network as far as possible
            oc.set("geometry.min-radius.fix.railways", "false");
        }
    }
    if (!oc.isSet("type-files")) {
        const char* sumoPath = std::getenv("SUMO_HOME");
        if (sumoPath == nullptr) {
            WRITE_WARNING(TL("Environment variable SUMO_HOME is not set, using built in type maps."));
        } else {
            const std::string path = sumoPath + std::string("/data/typemap/");
            if (oc.isSet("osm-files")) {
                oc.setDefault("type-files", path + "osmNetconvert.typ.xml");
            }
            if (oc.isSet("opendrive-files")) {
                oc.setDefault("type-files", path + "opendriveNetconvert.typ.xml");
            }
        }
    }
    if (oc.isSet("opendrive-files")) {
        if (oc.isDefault("tls.left-green.time")) {
            // legacy behavior. see #2114
            oc.setDefault("tls.left-green.time", "0");
        }
        if (oc.isDefault("rectangular-lane-cut")) {
            // a better interpretation of imported geometries
            oc.setDefault("rectangular-lane-cut", "true");
        }
        if (oc.isDefault("geometry.max-grade.fix")) {
            // a better interpretation of imported geometries
            oc.setDefault("geometry.max-grade.fix", "false");
        }
        if (oc.isDefault("no-turnarounds") && oc.isDefault("no-turnarounds.except-deadend")) {
            // changed default since all connections (connecting roads) are loaded from the input.
            oc.set("no-turnarounds.except-deadend", "true");
        }
    }
    if (!oc.isDefault("osm.extra-attributes") && oc.isDefault("osm.all-attributes")) {
        oc.setDefault("osm.all-attributes", "true");
    }
    if (oc.getBool("osm.crossings") && !oc.getBool("osm.sidewalks")) {
        WRITE_WARNING(TL("It is recommended to use option osm.crossings with osm.sidewalks"));
    }
    if (oc.isSet("shapefile-prefix") && !oc.isDefault("shapefile.name")) {
        oc.setDefault("output.street-names", "true");
    }

    return ok;
}


/****************************************************************************/
