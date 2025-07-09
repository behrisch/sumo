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
/// @file    CSVFormatter.h
/// @author  Michael Behrisch
/// @date    2025-06-12
///
// Output formatter for CSV output
/****************************************************************************/
#pragma once
#include <config.h>

#include <memory>
#ifdef HAVE_FMT
#include <fmt/ostream.h>
#endif

#include "OutputFormatter.h"


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class CSVFormatter
 * @brief Output formatter for CSV output
 */
class CSVFormatter : public OutputFormatter {
public:
    /// @brief Constructor
    CSVFormatter(const std::string& columnNames, const char separator = ';');

    /// @brief Destructor
    virtual ~CSVFormatter() { }

    /** @brief Keeps track of an open XML tag by adding a new element to the stack
     *
     * @param[in] into The output stream to use (unused)
     * @param[in] xmlElement Name of element to open (unused)
     * @return The OutputDevice for further processing
     */
    void openTag(std::ostream& into, const std::string& xmlElement);

    /** @brief Keeps track of an open XML tag by adding a new element to the stack
     *
     * @param[in] into The output stream to use (unused)
     * @param[in] xmlElement Name of element to open (unused)
     */
    void openTag(std::ostream& into, const SumoXMLTag& xmlElement);

    /** @brief Closes the most recently opened tag
     *
     * @param[in] into The output stream to use
     * @return Whether a further element existed in the stack and could be closed
     * @todo it is not verified that the topmost element was closed
     */
    bool closeTag(std::ostream& into, const std::string& comment = "");

    /** @brief writes a named attribute
     *
     * @param[in] into The output stream to use
     * @param[in] attr The attribute (name)
     * @param[in] val The attribute value
     */
    template <class T>
    void writeAttr(std::ostream& into, const SumoXMLAttr attr, const T& val) {
        checkAttr(attr);
        *myXMLStack[myCurrentDepth - 1] << toString(val, into.precision()) << mySeparator;
    }

    template <class T>
    void writeAttr(std::ostream& into, const std::string& attr, const T& val) {
        assert(!myCheckColumns);
        if (!myWroteHeader) {
            if (std::find(myHeader.begin(), myHeader.end(), attr) != myHeader.end()) {
                myHeader.push_back(myCurrentTag + "_" + attr);
            } else {
                myHeader.push_back(attr);
            }
        }
        *myXMLStack[myCurrentDepth - 1] << toString(val, into.precision()) << mySeparator;
    }

    void writeNull(std::ostream& /* into */, const SumoXMLAttr attr) {
        checkAttr(attr);
        *myXMLStack[myCurrentDepth - 1] << mySeparator;
    }

    void writeTime(std::ostream& /* into */, const SumoXMLAttr attr, const SUMOTime val) {
        checkAttr(attr);
        *myXMLStack[myCurrentDepth - 1] << time2string(val) << mySeparator;
    }

    bool wroteHeader() const {
        return myWroteHeader;
    }

    void setExpectedAttributes(const SumoXMLAttrMask& expected, const int depth = 2) {
        myExpectedAttrs = expected;
        myMaxDepth = depth;
        myCheckColumns = expected.any();
    }

private:
    /** @brief Helper function to keep track of the written attributes and accumulate the header.
     * It checks whether the written attribute is expected in the column based format.
     * The check does only apply to the deepest level of the XML hierarchy and not to the order of the columns just to the presence.
     *
     * @param[in] attr The attribute (name)
     */
    inline void checkAttr(const SumoXMLAttr attr) {
        if (myCheckColumns && myMaxDepth == myCurrentDepth) {
            mySeenAttrs.set(attr);
            if (!myExpectedAttrs.test(attr)) {
                throw ProcessError(TLF("Unexpected attribute '%', this file format does not support CSV output yet.", toString(attr)));
            }
        }
        if (!myWroteHeader) {
            const std::string attrString = toString(attr);
            if (myHeaderFormat == "plain" || (myHeaderFormat == "auto" && std::find(myHeader.begin(), myHeader.end(), attrString) == myHeader.end())) {
                myHeader.push_back(attrString);
            } else {
                myHeader.push_back(myCurrentTag + "_" + attrString);
            }
        }
    }

    /// @brief the format to use for the column names
    const std::string myHeaderFormat;

    /// @brief The value separator
    const char mySeparator;

    /// @brief the CSV header
    std::vector<std::string> myHeader;

    /// @brief the currently read tag (only valid when generating the header)
    std::string myCurrentTag;

    /// @brief The attributes to write for each begun xml element (excluding the root element)
    std::vector<std::unique_ptr<std::ostringstream>> myXMLStack;

    /// @brief the maximum depth of the XML hierarchy (excluding the root element)
    int myMaxDepth = 0;

    /// @brief the current depth of the XML hierarchy (excluding the root element)
    int myCurrentDepth = 0;

    /// @brief whether the CSV header line has been written
    bool myWroteHeader = false;

    /// @brief whether the columns should be checked for completeness
    bool myCheckColumns = false;

    /// @brief which CSV columns are expected (just for checking completeness)
    SumoXMLAttrMask myExpectedAttrs;

    /// @brief which CSV columns have been set (just for checking completeness)
    SumoXMLAttrMask mySeenAttrs;
};


// ===========================================================================
// specialized template implementations (for speedup)
// ===========================================================================
template <>
inline void CSVFormatter::writeAttr(std::ostream& into, const SumoXMLAttr attr, const double& val) {
    checkAttr(attr);
#ifdef HAVE_FMT
    fmt::print(*myXMLStack[myCurrentDepth - 1], "{:.{}f}{}", val, into.precision(), mySeparator);
#else
    *myXMLStack[myCurrentDepth - 1] << toString(val, into.precision()) << mySeparator;
#endif
}


template <>
inline void CSVFormatter::writeAttr(std::ostream& /* into */, const SumoXMLAttr attr, const std::string& val) {
    checkAttr(attr);
    *myXMLStack[myCurrentDepth - 1] << val << mySeparator;
}
