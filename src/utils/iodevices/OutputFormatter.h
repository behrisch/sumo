/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2012-2026 German Aerospace Center (DLR) and others.
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
/// @file    OutputFormatter.h
/// @author  Daniel Krajzewicz
/// @author  Michael Behrisch
/// @date    2012
///
// Abstract base class for output formatters
/****************************************************************************/
#pragma once
#include <config.h>

#include <string>
#include <vector>
#include <utils/xml/SUMOXMLDefinitions.h>


// ===========================================================================
// class declarations
// ===========================================================================
class Boundary;
class Position;
class PositionVector;
class RGBColor;


// ===========================================================================
// class definitions
// ===========================================================================
enum class OutputFormatterType {
    XML,
#ifdef ARROW_SO_VERSION
    PARQUET,
#endif
    CSV
};

/**
 * @class OutputFormatter
 * @brief Abstract base class for output formatters
 *
 * OutputFormatter format XML like output into the output stream.
 *  There are only two implementation at the moment, "normal" XML
 *  and binary XML.
 */
class OutputFormatter {
public:
    /// @brief Constructor
    OutputFormatter(OutputFormatterType t) : myType(t) { }

    /// @brief Destructor
    virtual ~OutputFormatter() { }

    /** @brief Writes an XML header with optional configuration
     *
     * If something has been written (myXMLStack is not empty), nothing
     *  is written and false returned.
     * The default implementation does nothing and returns false.
     *
     * @param[in] into The output stream to use
     * @param[in] rootElement The root element to use
     * @param[in] attrs Additional attributes to save within the rootElement
     * @param[in] includeConfig whether the current config should be included as XML comment
     * @return whether something has been written
     */
    virtual bool writeXMLHeader(std::ostream& into, const std::string& rootElement,
                                const std::map<SumoXMLAttr, std::string>& attrs, bool writeMetadata,
                                bool includeConfig) {
        UNUSED_PARAMETER(into);
        UNUSED_PARAMETER(rootElement);
        UNUSED_PARAMETER(attrs);
        UNUSED_PARAMETER(writeMetadata);
        UNUSED_PARAMETER(includeConfig);
        return false;
    }

    /** @brief Opens an XML tag
     *
     * An indentation, depending on the current xml-element-stack size, is written followed
     *  by the given xml element ("<" + xmlElement)
     * The xml element is added to the stack, then.
     *
     * @param[in] into The output stream to use
     * @param[in] xmlElement Name of element to open
     * @return The OutputDevice for further processing
     */
    virtual void openTag(std::ostream& into, const std::string& xmlElement) = 0;

    /** @brief Opens an XML tag
     *
     * Helper method which finds the correct string before calling openTag.
     *
     * @param[in] into The output stream to use
     * @param[in] xmlElement Id of the element to open
     */
    virtual void openTag(std::ostream& into, const SumoXMLTag& xmlElement) = 0;

    virtual void writeTime(std::ostream& into, const SumoXMLAttr attr, const SUMOTime val) = 0;

    /** @brief Closes the most recently opened tag and optinally add a comment
     *
     * @param[in] into The output stream to use
     * @return Whether a further element existed in the stack and could be closed
     * @todo it is not verified that the topmost element was closed
     */
    virtual bool closeTag(std::ostream& into, const std::string& comment = "") = 0;

    /** @brief Writes a preformatted tag to the device but ensures that any
     * pending tags are closed.
     * This method is only implemented for XML output.
     * @param[in] into The output stream to use
     * @param[in] val The preformatted data
     */
    virtual void writePreformattedTag(std::ostream& into, const std::string& val) {
        UNUSED_PARAMETER(into);
        UNUSED_PARAMETER(val);
        throw ProcessError("The selected file format does not support preformatted tags.");
    }

    /** @brief Writes some whitespace to format the output.
     * This method is only implemented for XML output.
     * @param[in] into The output stream to use
     * @param[in] val The whitespace
     */
    virtual void writePadding(std::ostream& into, const std::string& val) {
        UNUSED_PARAMETER(into);
        UNUSED_PARAMETER(val);
    }

    /** @brief Returns whether a header has been written.
     * Useful to detect whether a file is being used by multiple sources.
     * @return Whether a header has been written
     */
    virtual bool wroteHeader() const = 0;

    /** @brief Returns the type of formatter being used.
     * @return the formatter type
     */
    OutputFormatterType getType() {
        return myType;
    }

    /** @brief Set the expected attributes to write.
     * This is used for tracking which attributes are expected in table like outputs.
     * This should be not necessary but at least in the initial phase of implementing CSV and Parquet
     * it helps a lot to track errors.
     * @param[in] expected which attributes are to be written (at the deepest XML level)
     * @param[in] depth the maximum XML hierarchy depth (excluding the root)
     */
    virtual void setExpectedAttributes(const SumoXMLAttrMask& expected, const int depth = 2) {
        UNUSED_PARAMETER(expected);
        UNUSED_PARAMETER(depth);
    }

private:
    /// @brief the type of formatter being used (XML, CSV, Parquet, etc.)
    const OutputFormatterType myType;
};
