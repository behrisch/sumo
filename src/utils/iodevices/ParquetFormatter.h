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
/// @file    ParquetFormatter.h
/// @author  Michael Behrisch
/// @date    2025-06-17
///
// Output formatter for Parquet output
/****************************************************************************/
#pragma once
#include <config.h>

#include <ostream>

#ifdef _MSC_VER
/* Disable warning about unmatched push / pop.
   TODO Re-enable this once it has been solved upstream, see https://github.com/apache/arrow/issues/47099 */
#pragma warning(suppress: 5032)
#pragma warning(push)
/* Disable warning about unused parameters */
#pragma warning(disable: 4100)
/* Disable warning about hidden function arrow::io::Writable::Write */
#pragma warning(disable: 4266)
/* Disable warning about padded memory layout */
#pragma warning(disable: 4324)
/* Disable warning about this in initializers */
#pragma warning(disable: 4355)
/* Disable warning about changed memory layout due to virtual base class */
#pragma warning(disable: 4435)
/* Disable warning about declaration hiding class member */
#pragma warning(disable: 4458)
/* Disable warning about implicit conversion of int to bool */
#pragma warning(disable: 4800)
#endif
#include <arrow/api.h>
#include <parquet/arrow/writer.h>
#ifdef _MSC_VER
/* Disable warning about unmatched push / pop.
   TODO Re-enable this once it has been solved upstream, see https://github.com/apache/arrow/issues/47099 */
#pragma warning(suppress: 5031)
#pragma warning(pop)
#endif

#include <utils/common/ToString.h>
#include "OutputFormatter.h"


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class ParquetFormatter
 * @brief Output formatter for Parquet output
 */
class ParquetFormatter : public OutputFormatter {
public:
    /// @brief Constructor
    // for some motivation on the default batch size see https://stackoverflow.com/questions/76782018/what-is-actually-meant-when-referring-to-parquet-row-group-size
    ParquetFormatter(const std::string& columnNames, const std::string& compression = "", const int batchSize = 1000000);

    /// @brief Destructor
    virtual ~ParquetFormatter() { }

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
     * @param[in] attr The attribute (name)
     * @param[in] val The attribute value
     * @param[in] isNull The given value is not set
     */
    template <class T>
    void writeAttr(std::ostream& /* into */, const SumoXMLAttr attr, const T& val, const bool isNull = false) {
        checkAttr(attr);
        checkBuilder<SumoXMLAttr, arrow::StringBuilder>(attr, arrow::utf8);
        myValues.push_back(isNull ? nullptr : std::make_shared<arrow::StringScalar>(toString(val)));
    }

    template <class T>
    void writeAttr(std::ostream& /* into */, const std::string& attr, const T& val) {
        assert(!myCheckColumns);
        checkBuilder<std::string, arrow::StringBuilder>(attr, arrow::utf8);
        myValues.push_back(std::make_shared<arrow::StringScalar>(toString(val)));
    }

    void writeTime(std::ostream& into, const SumoXMLAttr attr, const SUMOTime val) {
        if (!gHumanReadableTime) {
            checkBuilder<SumoXMLAttr, arrow::DoubleBuilder>(attr, arrow::float64);
            myValues.push_back(std::make_shared<arrow::DoubleScalar>(STEPS2TIME(val)));
            return;
        }
        writeAttr(into, attr, time2string(val));
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
    inline const std::string getAttrString(const std::string& attrString) {
        if (myHeaderFormat == "plain") {
            return attrString;
        }
        if (myHeaderFormat == "auto") {
            for (const auto& field : mySchema->fields()) {
                if (field->name() == attrString) {
                    return myCurrentTag + "_" + attrString;
                }
            }
            return attrString;
        }
        return myCurrentTag + "_" + attrString;
    }

    inline void checkAttr(const SumoXMLAttr attr) {
        if (myCheckColumns && myMaxDepth == (int)myXMLStack.size()) {
            mySeenAttrs.set(attr);
            if (!myExpectedAttrs.test(attr)) {
                throw ProcessError(TLF("Unexpected attribute '%', this file format does not support Parquet output yet.", toString(attr)));
            }
        }
    }

    template <class ATTR_TYPE, class BUILDER>
    inline void checkBuilder(const ATTR_TYPE& attr, const std::shared_ptr<arrow::DataType>& (*dataType)()) {
        myNeedsWrite = true;
        if (!myWroteHeader) {
            const std::string fieldName = getAttrString(toString(attr));
            for (const auto& field : mySchema->fields()) {
                if (field->name() == fieldName) {
                    return;
                }
            }
            mySchema = *mySchema->AddField(mySchema->num_fields(), arrow::field(fieldName, dataType()));
            auto builder = std::make_shared<BUILDER>();
            if (!myBuilders.empty()) {
                if (myBuilders.back()->length() > 0) {
                    PARQUET_THROW_NOT_OK(builder->AppendNulls(myBuilders.back()->length()));
                }
                while (myValues.size() < myBuilders.size()) {
                    myValues.push_back(nullptr);
                }
            }
            myBuilders.push_back(builder);
        }
    }

    /// @brief the format to use for the column names
    const std::string myHeaderFormat;

    /// @brief the compression to use
    parquet::Compression::type myCompression = parquet::Compression::UNCOMPRESSED;

    /// @brief the number of rows to write per batch
    const int myBatchSize;

    /// @brief the currently read tag (only valid when generating the header)
    std::string myCurrentTag;

    /// @brief the table schema
    std::shared_ptr<arrow::Schema> mySchema = arrow::schema({});

    /// @brief the output stream writer
    std::unique_ptr<parquet::arrow::FileWriter> myParquetWriter;

    /// @brief the content array builders for the table
    std::vector<std::shared_ptr<arrow::ArrayBuilder> > myBuilders;

    /// @brief The number of attributes in the currently open XML elements
    std::vector<int> myXMLStack;

    /// @brief the current attribute / column values
    std::vector<std::shared_ptr<arrow::Scalar> > myValues;

    /// @brief the maximum depth of the XML hierarchy
    int myMaxDepth = 0;

    /// @brief whether the schema has been constructed completely
    bool myWroteHeader = false;

    /// @brief whether the columns should be checked for completeness
    bool myCheckColumns = false;

    /// @brief whether there is still unwritten data
    bool myNeedsWrite = false;

    /// @brief the attributes which are expected for a complete row (including null values)
    SumoXMLAttrMask myExpectedAttrs;

    /// @brief the attributes already seen (including null values)
    SumoXMLAttrMask mySeenAttrs;
};


// ===========================================================================
// specialized template implementations
// ===========================================================================
template <>
inline void ParquetFormatter::writeAttr(std::ostream& into, const SumoXMLAttr attr, const double& val, const bool isNull) {
    checkAttr(attr);
    if (attr == SUMO_ATTR_X || attr == SUMO_ATTR_Y || into.precision() > 2) {
        checkBuilder<SumoXMLAttr, arrow::DoubleBuilder>(attr, arrow::float64);
        myValues.push_back(isNull ? nullptr : std::make_shared<arrow::DoubleScalar>(val));
    } else {
        checkBuilder<SumoXMLAttr, arrow::FloatBuilder>(attr, arrow::float32);
        myValues.push_back(isNull ? nullptr : std::make_shared<arrow::FloatScalar>((float)val));
    }
}

template <>
inline void ParquetFormatter::writeAttr(std::ostream& /* into */, const SumoXMLAttr attr, const int& val, const bool isNull) {
    checkAttr(attr);
    checkBuilder<SumoXMLAttr, arrow::Int32Builder>(attr, arrow::int32);
    myValues.push_back(isNull ? nullptr : std::make_shared<arrow::Int32Scalar>(val));
}

template <>
inline void ParquetFormatter::writeAttr(std::ostream& into, const std::string& attr, const double& val) {
    assert(!myCheckColumns);
    if (into.precision() > 2) {
        checkBuilder<std::string, arrow::DoubleBuilder>(attr, arrow::float64);
        myValues.push_back(std::make_shared<arrow::DoubleScalar>(val));
    } else {
        checkBuilder<std::string, arrow::FloatBuilder>(attr, arrow::float32);
        myValues.push_back(std::make_shared<arrow::FloatScalar>((float)val));
    }
}

template <>
inline void ParquetFormatter::writeAttr(std::ostream& /* into */, const std::string& attr, const int& val) {
    assert(!myCheckColumns);
    checkBuilder<std::string, arrow::Int32Builder>(attr, arrow::int32);
    myValues.push_back(std::make_shared<arrow::Int32Scalar>(val));
}
