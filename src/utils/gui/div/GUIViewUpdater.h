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
/// @file    GUIViewUpdater.h
/// @author  Pablo Alvarez Lopez
/// @date    Nov 2024
///
// class used for enable/disable updating view
/****************************************************************************/
#pragma once
#include <config.h>

// variables used for debug view updater
//#define DISABLE_VIEWUPDATER
//#define DEBUG_VIEWUPDATER

// ===========================================================================
// class definitions
// ===========================================================================

class GUIViewUpdater {

public:
    /// @brief constructor
    GUIViewUpdater();

    /// @brief allow update
    bool allowUpdate();

    /// @brief enable update
    void enableUpdate();

    /// @brief disable update
    void disableUpdate();

private:
    /// @brief allow update
    int myAllowUpdate = 0;

#ifdef DEBUG_VIEWUPDATER
    /// @brief counters for number of updates (only for debugging)
    long int numAllowedUpdates = 0;
    long int numDisallowedUpdates = 0;
#endif

    /// @brief set copy constructor private
    GUIViewUpdater(const GUIViewUpdater&) = default;

    /// @brief set assignment operator private
    GUIViewUpdater& operator=(const GUIViewUpdater&) = default;
};
