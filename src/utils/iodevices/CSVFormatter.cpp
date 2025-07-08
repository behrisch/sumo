/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2012-2025 German Aerospace Center (DLR) and others.
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
/// @file    CSVFormatter.cpp
/// @author  Michael Behrisch
/// @date    2025-06-12
///
// An output formatter for CSV files
/****************************************************************************/
#include <config.h>

#include <utils/common/MsgHandler.h>
#include <utils/common/ToString.h>
#include "CSVFormatter.h"


// ===========================================================================
// member method definitions
// ===========================================================================
CSVFormatter::CSVFormatter(const std::string& columnNames, const char separator)
    : OutputFormatter(OutputFormatterType::CSV), myHeaderFormat(columnNames), mySeparator(separator) {
    if (myHeaderFormat == "none") {
        myWroteHeader = true;
    }
}


void
CSVFormatter::openTag(std::ostream& /* into */, const std::string& xmlElement) {
    myCurrentDepth++;
    if (myCurrentDepth > (int)myXMLStack.size()) {
        myXMLStack.emplace_back(std::unique_ptr<std::ostringstream>(new std::ostringstream()));
    }
    if (!myWroteHeader) {
        myCurrentTag = xmlElement;
    }
    if (myMaxDepth == myCurrentDepth && myWroteHeader && myCurrentTag != xmlElement) {
        WRITE_WARNINGF("Encountered mismatch in XML tags (expected % but got %). Column names may be incorrect.", myCurrentTag, xmlElement);
    }
}


void
CSVFormatter::openTag(std::ostream& /* into */, const SumoXMLTag& xmlElement) {
    myCurrentDepth++;
    if (myCurrentDepth > (int)myXMLStack.size()) {
        myXMLStack.emplace_back(std::unique_ptr<std::ostringstream>(new std::ostringstream()));
    }
    if (!myWroteHeader) {
        myCurrentTag = toString(xmlElement);
    }
    if (myMaxDepth == myCurrentDepth && myWroteHeader && myCurrentTag != toString(xmlElement)) {
        WRITE_WARNINGF("Encountered mismatch in XML tags (expected % but got %). Column names may be incorrect.", myCurrentTag, toString(xmlElement));
    }
}


bool
CSVFormatter::closeTag(std::ostream& into, const std::string& /* comment */) {
    if (myMaxDepth == 0) {
        WRITE_WARNING("Column based formats are still experimental. Autodetection only works for homogeneous output.");
        myMaxDepth = myCurrentDepth;
    }
    if (myMaxDepth == myCurrentDepth && !myWroteHeader) {
        into << joinToString(myHeader, mySeparator) << "\n";
        myWroteHeader = true;
    }
    if (myCurrentDepth == myMaxDepth) {
        if (myCheckColumns && myExpectedAttrs != mySeenAttrs) {
            WRITE_ERRORF("Incomplete attribute set '%', this file format does not support CSV output yet.", toString(mySeenAttrs));
        }
        for (auto it = myXMLStack.begin(); it != myXMLStack.end() - 1; ++it) {
            into << (*it)->str();
        }
        // remove the final separator
        std::string final = myXMLStack[myCurrentDepth - 1]->str();
        final[final.size() - 1] = '\n';
        into << final;
        mySeenAttrs.reset();
    }
    if (myCurrentDepth > 0) {
        if (!myWroteHeader) {
            const std::string text = myXMLStack[myCurrentDepth - 1]->str();
            const int count = (int)std::count(text.begin(), text.end(), mySeparator);
            myHeader.resize(myHeader.size() - count);
        }
        myXMLStack[myCurrentDepth - 1]->str("");
        myXMLStack[myCurrentDepth - 1]->clear();
        myCurrentDepth--;
    }
    return false;
}


/****************************************************************************/
