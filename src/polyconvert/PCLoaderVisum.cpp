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
/// @file    PCLoaderVisum.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Christoph Sommer
/// @author  Michael Behrisch
/// @date    Thu, 02.11.2006
///
// A reader of pois and polygons stored in VISUM-format
/****************************************************************************/
#include <config.h>

#include <string>
#include <map>
#include <fstream>
#include <utils/common/StringTokenizer.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/StringUtils.h>
#include <utils/common/StringUtils.h>
#include <utils/common/ToString.h>
#include <utils/common/FileHelpers.h>
#include <utils/options/OptionsCont.h>
#include <utils/options/Option.h>
#include <utils/importio/LineReader.h>
#include <utils/common/StdDefs.h>
#include <polyconvert/PCPolyContainer.h>
#include "PCLoaderVisum.h"
#include <utils/common/RGBColor.h>
#include <utils/geom/GeomHelper.h>
#include <utils/geom/Boundary.h>
#include <utils/geom/Position.h>
#include <utils/geom/GeoConvHelper.h>
#include <utils/importio/NamedColumnsParser.h>

StringBijection<PCLoaderVisum::VISUM_KEY>::Entry PCLoaderVisum::KEYS_DE[] = {
    // duplicates NIImporter_VISUM::KEYS_DE due to lack of suitable common location
    { "VSYS", VISUM_SYS },
    { "STRECKENTYP", VISUM_LINKTYPE },
    { "KNOTEN", VISUM_NODE },
    { "BEZIRK", VISUM_DISTRICT },
    { "PUNKT", VISUM_POINT },
    { "STRECKE", VISUM_LINK },
    { "V0IV", VISUM_V0 },
    { "VSYSSET", VISUM_TYPES },
    { "RANG", VISUM_RANK },
    { "KAPIV", VISUM_CAPACITY },
    { "XKOORD", VISUM_XCOORD },
    { "YKOORD", VISUM_YCOORD },
    { "ID", VISUM_ID },
    { "CODE", VISUM_CODE },
    { "VONKNOTNR", VISUM_FROMNODE },
    { "NACHKNOTNR", VISUM_TONODE },
    { "TYPNR", VISUM_TYPE },
    { "TYP", VISUM_TYP },
    { "ANBINDUNG", VISUM_DISTRICT_CONNECTION },
    { "BEZNR", VISUM_SOURCE_DISTRICT },
    { "KNOTNR",  VISUM_FROMNODENO },
    { "RICHTUNG",  VISUM_DIRECTION },
    { "FLAECHEID",  VISUM_SURFACEID },
    { "TFLAECHEID",  VISUM_FACEID },
    { "VONPUNKTID",  VISUM_FROMPOINTID },
    { "NACHPUNKTID",  VISUM_TOPOINTID },
    { "KANTE",  VISUM_EDGE },
    { "ABBIEGER",  VISUM_TURN },
    { "UEBERKNOTNR",  VISUM_VIANODENO },
    { "ANZFAHRSTREIFEN",  VISUM_NUMLANES },
    { "INDEX",  VISUM_INDEX },
    { "STRECKENPOLY",  VISUM_LINKPOLY },
    { "FLAECHENELEMENT",  VISUM_SURFACEITEM },
    { "TEILFLAECHENELEMENT",  VISUM_FACEITEM },
    { "KANTEID",  VISUM_EDGEID },
    { "Q",  VISUM_ORIGIN },
    { "Z",  VISUM_DESTINATION },
    { "KATNR", VISUM_CATID },
    { "ZWISCHENPUNKT", VISUM_EDGEITEM },
    { "POIKATEGORIE", VISUM_POICATEGORY },
    { "NETZ", VISUM_NETWORK },
    { "NR", VISUM_NO } // must be the last one
};


StringBijection<PCLoaderVisum::VISUM_KEY> PCLoaderVisum::KEYS(PCLoaderVisum::KEYS_DE, VISUM_NO);


// ===========================================================================
// method definitions
// ===========================================================================
void
PCLoaderVisum::loadIfSet(OptionsCont& oc, PCPolyContainer& toFill,
                         PCTypeMap& tm) {
    if (!oc.isSet("visum-files")) {
        return;
    }
    const std::string languageFile = oc.getString("visum.language-file");
    if (languageFile != "") {
        loadLanguage(languageFile);
    }
    // parse file(s)
    std::vector<std::string> files = oc.getStringVector("visum-files");
    for (std::vector<std::string>::const_iterator file = files.begin(); file != files.end(); ++file) {
        if (!FileHelpers::isReadable(*file)) {
            throw ProcessError(TLF("Could not open visum-file '%'.", *file));
        }
        PROGRESS_BEGIN_MESSAGE("Parsing from visum-file '" + *file + "'");
        load(*file, oc, toFill, tm);
        PROGRESS_DONE_MESSAGE();
    }
}



void
PCLoaderVisum::load(const std::string& file, OptionsCont& oc, PCPolyContainer& toFill,
                    PCTypeMap& tm) {
    GeoConvHelper& geoConvHelper = GeoConvHelper::getProcessing();
    std::string what;
    std::map<long long int, Position> punkte;
    std::map<long long int, PositionVector> kanten;
    std::map<long long int, PositionVector> teilflaechen;
    std::map<long long int, long long int> flaechenelemente;
    NamedColumnsParser lineParser;
    LineReader lr(file);
    while (lr.hasMore()) {
        std::string line = lr.readLine();
        // reset if current is over
        if (line.length() == 0 || line[0] == '*' || line[0] == '$') {
            what = "";
        }
        // read items
        if (what == "$" + KEYS.getString(VISUM_POINT)) {
            lineParser.parseLine(line);
            long long int id = StringUtils::toLong(lineParser.get(KEYS.getString(VISUM_ID)));
            double x = StringUtils::toDouble(lineParser.get(KEYS.getString(VISUM_XCOORD)));
            double y = StringUtils::toDouble(lineParser.get(KEYS.getString(VISUM_YCOORD)));
            Position pos(x, y);
            if (!geoConvHelper.x2cartesian(pos)) {
                WRITE_WARNINGF(TL("Unable to project coordinates for point '%'."), toString(id));
            }
            punkte[id] = pos;
            continue;
        } else if (what == "$" + KEYS.getString(VISUM_EDGE)) {
            lineParser.parseLine(line);
            long long int id = StringUtils::toLong(lineParser.get(KEYS.getString(VISUM_ID)));
            long long int fromID = StringUtils::toLong(lineParser.get(KEYS.getString(VISUM_FROMPOINTID)));
            long long int toID = StringUtils::toLong(lineParser.get(KEYS.getString(VISUM_TOPOINTID)));
            PositionVector vec;
            vec.push_back(punkte[fromID]);
            vec.push_back(punkte[toID]);
            kanten[id] = vec;
            continue;
        } else if (what == "$" + KEYS.getString(VISUM_EDGEITEM)) {
            lineParser.parseLine(line);
            long long int id = StringUtils::toLong(lineParser.get(KEYS.getString(VISUM_EDGEID)));
            int index = StringUtils::toInt(lineParser.get(KEYS.getString(VISUM_INDEX)));
            double x = StringUtils::toDouble(lineParser.get(KEYS.getString(VISUM_XCOORD)));
            double y = StringUtils::toDouble(lineParser.get(KEYS.getString(VISUM_YCOORD)));
            Position pos(x, y);
            if (!geoConvHelper.x2cartesian(pos)) {
                WRITE_WARNINGF(TL("Unable to project coordinates for edge '%'."), toString(id));
            }
            kanten[id].insert(kanten[id].begin() + index, pos);
            continue;
        } else if (what == "$" + KEYS.getString(VISUM_FACEITEM)) {
            lineParser.parseLine(line);
            long long int id = StringUtils::toLong(lineParser.get(KEYS.getString(VISUM_FACEID)));
            //int index = StringUtils::toInt(lineParser.get("INDEX"));
            //index = 0; /// hmmmm - assume it's sorted...
            long long int kid = StringUtils::toLong(lineParser.get(KEYS.getString(VISUM_EDGEID)));
            int dir = StringUtils::toInt(lineParser.get(KEYS.getString(VISUM_DIRECTION)));
            if (teilflaechen.find(id) == teilflaechen.end()) {
                teilflaechen[id] = PositionVector();
            }
            if (dir == 0) {
                for (int i = 0; i < (int) kanten[kid].size(); ++i) {
                    teilflaechen[id].push_back_noDoublePos(kanten[kid][i]);
                }
            } else {
                for (int i = (int) kanten[kid].size() - 1; i >= 0; --i) {
                    teilflaechen[id].push_back_noDoublePos(kanten[kid][i]);
                }
            }
            continue;
        } else if (what == "$" + KEYS.getString(VISUM_SURFACEITEM)) {
            lineParser.parseLine(line);
            long long int id = StringUtils::toLong(lineParser.get(KEYS.getString(VISUM_SURFACEID)));
            long long int tid = StringUtils::toLong(lineParser.get(KEYS.getString(VISUM_FACEID)));
            flaechenelemente[id] = tid;
            continue;
        }
        // set if read
        if (line[0] == '$') {
            what = "";
            if (line.find("$" + KEYS.getString(VISUM_POINT) + ":") == 0) {
                what = "$" + KEYS.getString(VISUM_POINT);
            } else if (line.find("$" + KEYS.getString(VISUM_EDGE) + ":") == 0) {
                what = "$" + KEYS.getString(VISUM_EDGE);
            } else if (line.find("$" + KEYS.getString(VISUM_EDGEITEM) + ":") == 0) {
                what = "$" + KEYS.getString(VISUM_EDGEITEM);
            } else if (line.find("$" + KEYS.getString(VISUM_FACEITEM) + ":") == 0) {
                what = "$" + KEYS.getString(VISUM_FACEITEM);
            } else if (line.find("$" + KEYS.getString(VISUM_SURFACEITEM) + ":") == 0) {
                what = "$" + KEYS.getString(VISUM_SURFACEITEM);
            }
            if (what != "") {
                lineParser.reinit(line.substr(what.length() + 1));
            }
        }
    }

    // do some more sane job...
    RGBColor c = RGBColor::parseColor(oc.getString("color"));
    std::map<std::string, std::string> typemap;
    // load the pois/polys
    lr.reinit();
    bool parsingCategories = false;
    bool parsingPOIs = false;
    bool parsingDistrictsDirectly = false;
    PositionVector vec;
    std::string polyType, lastID;
    bool first = true;
    while (lr.hasMore()) {
        std::string line = lr.readLine();
        // do not parse empty lines
        if (line.length() == 0) {
            continue;
        }
        // do not parse comment lines
        if (line[0] == '*') {
            continue;
        }

        if (line[0] == '$') {
            // reset parsing on new entry type
            parsingCategories = false;
            parsingPOIs = false;
            parsingDistrictsDirectly = false;
            polyType = "";
        }

        if (parsingCategories) {
            // parse the category
            StringTokenizer st(line, ";");
            std::string catid = st.next();
            std::string catname = st.next();
            typemap[catid] = catname;
        }
        if (parsingPOIs) {
            // parse the poi
            // $POI:Nr;CATID;CODE;NAME;Kommentar;XKoord;YKoord;
            lineParser.parseLine(line);
            long long int idL = StringUtils::toLong(lineParser.get(KEYS.getString(VISUM_NO)));
            std::string id = toString(idL);
            std::string catid = lineParser.get(KEYS.getString(VISUM_CATID));
            // process read values
            double x = StringUtils::toDouble(lineParser.get(KEYS.getString(VISUM_XCOORD)));
            double y = StringUtils::toDouble(lineParser.get(KEYS.getString(VISUM_YCOORD)));
            Position pos(x, y);
            if (!geoConvHelper.x2cartesian(pos)) {
                WRITE_WARNINGF(TL("Unable to project coordinates for POI '%'."), id);
            }
            std::string type = typemap[catid];
            // patch the values
            bool discard = oc.getBool("discard");
            std::string icon = oc.getString("icon");
            double layer = oc.getFloat("layer");
            RGBColor color;
            if (tm.has(type)) {
                const PCTypeMap::TypeDef& def = tm.get(type);
                id = def.prefix + id;
                type = def.id;
                color = def.color;
                discard = def.discard;
                icon = def.icon;
                layer = def.layer;
            } else {
                id = oc.getString("prefix") + id;
                color = c;
            }
            if (!discard) {
                const std::string origId = id;
                int index = 1;
                while (toFill.getPOIs().get(id) != nullptr) {
                    id = origId + "#" + toString(index++);
                }
                PointOfInterest* poi = new PointOfInterest(id, type, color, pos, false, "", 0, false, 0, icon, layer);
                toFill.add(poi);
            }
        }

        // poly
        if (polyType != "") {
            StringTokenizer st(line, ";");
            std::string id = st.next();
            std::string type;
            if (!first && lastID != id) {
                // we have parsed a polygon completely
                RGBColor color;
                double layer = oc.getFloat("layer");
                bool discard = oc.getBool("discard");
                if (tm.has(polyType)) {
                    const PCTypeMap::TypeDef& def = tm.get(polyType);
                    id = def.prefix + id;
                    type = def.id;
                    color = def.color;
                    discard = def.discard;
                    layer = def.layer;
                } else {
                    id = oc.getString("prefix") + id;
                    type = oc.getString("type");
                    color = c;
                }
                if (!discard) {
                    const std::string origId = id;
                    int index = 1;
                    while (toFill.getPolygons().get(id) != nullptr) {
                        id = origId + "#" + toString(index++);
                    }
                    SUMOPolygon* poly = new SUMOPolygon(id, type, color, vec, false, false, 1, layer);
                    toFill.add(poly);
                }
                vec.clear();
            }
            lastID = id;
            first = false;
            // parse current poly
            std::string index = st.next();
            std::string xpos = st.next();
            std::string ypos = st.next();
            Position pos2D((double) atof(xpos.c_str()), (double) atof(ypos.c_str()));
            if (!geoConvHelper.x2cartesian(pos2D)) {
                WRITE_WARNINGF(TL("Unable to project coordinates for polygon '%'."), id);
            }
            vec.push_back(pos2D);
        }

        // district refering a shape
        if (parsingDistrictsDirectly) {
            //$BEZIRK:NR	CODE	NAME	TYPNR	XKOORD	YKOORD	FLAECHEID	BEZART	IVANTEIL_Q	IVANTEIL_Z	OEVANTEIL	METHODEANBANTEILE	ZWERT1	ZWERT2	ZWERT3	ISTINAUSWAHL	OBEZNR	NOM_COM	COD_COM
            lineParser.parseLine(line);
            long long int idL = StringUtils::toLong(lineParser.get(KEYS.getString(VISUM_NO)));
            std::string id = toString(idL);
            long long int area = StringUtils::toLong(lineParser.get(KEYS.getString(VISUM_SURFACEID)));
            double x = StringUtils::toDouble(lineParser.get(KEYS.getString(VISUM_XCOORD)));
            double y = StringUtils::toDouble(lineParser.get(KEYS.getString(VISUM_YCOORD)));
            // patch the values
            std::string type = "district";
            bool discard = oc.getBool("discard");
            std::string icon = oc.getString("icon");
            double layer = oc.getFloat("layer");
            RGBColor color;
            if (tm.has(type)) {
                const PCTypeMap::TypeDef& def = tm.get(type);
                id = def.prefix + id;
                type = def.id;
                color = def.color;
                discard = def.discard;
                icon = def.icon;
                layer = def.layer;
            } else {
                id = oc.getString("prefix") + id;
                type = oc.getString("type");
                color = c;
            }
            if (!discard) {
                if (teilflaechen[flaechenelemente[area]].size() > 0) {
                    const std::string origId = id;
                    int index = 1;
                    while (toFill.getPolygons().get(id) != nullptr) {
                        id = origId + "#" + toString(index++);
                    }
                    const auto shape = teilflaechen[flaechenelemente[area]];
                    SUMOPolygon* poly = new SUMOPolygon(id, type, color, shape, false, false, 1, layer);
                    toFill.add(poly);
                } else {
                    Position pos(x, y);
                    if (!geoConvHelper.x2cartesian(pos)) {
                        WRITE_WARNINGF(TL("Unable to project coordinates for POI '%'."), id);
                    }
                    const std::string origId = id;
                    int index = 1;
                    while (toFill.getPOIs().get(id) != nullptr) {
                        id = origId + "#" + toString(index++);
                    }
                    PointOfInterest* poi = new PointOfInterest(id, type, color, pos, false, "", 0, false, 0, icon, layer);
                    toFill.add(poi);
                }
            }
        }


        if (line.find("$POIKATEGORIEDEF:") == 0 || line.find("$" + KEYS.getString(VISUM_POICATEGORY) + ":") == 0) {
            // ok, got categories, begin parsing from next line
            parsingCategories = true;
            lineParser.reinit(line.substr(line.find(":") + 1));
        }
        if ((line.find("$POI:") == 0) || line.find("$POIOFCAT") != std::string::npos) {
            // ok, got pois, begin parsing from next line
            parsingPOIs = true;
            lineParser.reinit(line.substr(line.find(":") + 1));
        }
        if (line.find("$" + KEYS.getString(VISUM_DISTRICT)) == 0 && line.find(KEYS.getString(VISUM_SURFACEID)) != std::string::npos) {
            // ok, have a district header, and it seems like districts would reference shapes...
            parsingDistrictsDirectly = true;
            lineParser.reinit(line.substr(line.find(":") + 1));
        }


        if (line.find("$BEZIRKPOLY") != std::string::npos) {
            polyType = "district";
        }
        if (line.find("$GEBIETPOLY") != std::string::npos) {
            polyType = "area";
        }

    }
}


void
PCLoaderVisum::loadLanguage(const std::string& file) {
    std::ifstream strm(file.c_str());
    if (!strm.good()) {
        throw ProcessError(TLF("Could not load VISUM language map from '%'.", file));
    }
    while (strm.good()) {
        std::string keyDE;
        std::string keyNew;
        strm >> keyDE;
        strm >> keyNew;
        if (KEYS.hasString(keyDE)) {
            VISUM_KEY key = KEYS.get(keyDE);
            KEYS.remove(keyDE, key);
            KEYS.insert(keyNew, key);
        } else if (keyDE != "") {
            // do not warn about network-related keys (NIImporter_VISUM)
            //WRITE_WARNINGF(TL("Unknown entry '%' in VISUM language map"), keyDE);
        }
    }

}

/****************************************************************************/
