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
/// @file    PCLoaderVisum.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    Thu, 02.11.2006
///
// A reader of pois and polygons stored in VISUM-format
/****************************************************************************/
#pragma once
#include <config.h>

#include <string>
#include "PCPolyContainer.h"
#include "PCTypeMap.h"


// ===========================================================================
// class definitions
// ===========================================================================
class OptionsCont;


// ===========================================================================
// class declarations
// ===========================================================================
/**
 * @class PCLoaderVisum
 * @brief A reader of pois and polygons stored in VISUM-format
 */
class PCLoaderVisum {
public:
    /** @brief Loads pois/polygons assumed to be stored using VISUM-format
     *
     * If the option "visum-files" is set within the given options container,
     *  the files stored herein are parsed using "load", assuming this
     *  option contains file paths to files containing pois and polygons stored
     *  in VISUM ".net"-format.
     *
     * @param[in] oc The options container to get further options from
     * @param[in] toFill The poly/pois container to add loaded polys/pois to
     * @param[in] tm The type map to use for setting values of loaded polys/pois
     * @exception ProcessError if something fails
     */
    static void loadIfSet(OptionsCont& oc, PCPolyContainer& toFill,
                          PCTypeMap& tm);


protected:
    /** @brief Parses pois/polys stored within the given file
     * @param[in] oc The options container to get further options from
     * @param[in] toFill The poly/pois container to add loaded polys/pois to
     * @param[in] tm The type map to use for setting values of loaded polys/pois
     * @exception ProcessError if something fails
     */
    static void load(const std::string& file, OptionsCont& oc, PCPolyContainer& toFill,
                     PCTypeMap& tm);


    /**
     * @enum VISUM keys
     * @brief Numbers representing VISUM keywords
     */
    enum VISUM_KEY {
        VISUM_SYS,
        VISUM_LINKTYPE,
        VISUM_NODE,
        VISUM_DISTRICT,
        VISUM_POINT,
        VISUM_LINK,
        VISUM_V0,
        VISUM_TYPES,
        VISUM_RANK,
        VISUM_CAPACITY,
        VISUM_XCOORD,
        VISUM_YCOORD,
        VISUM_FROMNODE,
        VISUM_TONODE,
        VISUM_TYPE,
        VISUM_TYP,
        VISUM_ID,
        VISUM_CODE,
        VISUM_DISTRICT_CONNECTION,
        VISUM_SOURCE_DISTRICT,
        VISUM_FROMNODENO,
        VISUM_DIRECTION,
        VISUM_SURFACEID,
        VISUM_FACEID,
        VISUM_FROMPOINTID,
        VISUM_TOPOINTID,
        VISUM_EDGE,
        VISUM_VIANODENO,
        VISUM_NUMLANES,
        VISUM_TURN,
        VISUM_INDEX,
        VISUM_LINKPOLY,
        VISUM_SURFACEITEM,
        VISUM_FACEITEM,
        VISUM_EDGEID,
        VISUM_ORIGIN,
        VISUM_DESTINATION,
        // polyconvert keys added to avoid warnings
        VISUM_CATID,
        VISUM_EDGEITEM,
        VISUM_POICATEGORY,
        VISUM_NETWORK,
        VISUM_PROJECTIONDEFINITION,
        VISUM_NO // must be the last one
    };

    /// Strings for the keywords
    static StringBijection<VISUM_KEY>::Entry KEYS_DE[];

    /// @brief link directions
    static StringBijection<VISUM_KEY> KEYS;

    static void loadLanguage(const std::string& file);
};
