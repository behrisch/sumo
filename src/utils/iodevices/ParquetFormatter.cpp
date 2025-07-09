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
/// @file    ParquetFormatter.cpp
/// @author  Michael Behrisch
/// @date    2025-06-17
///
// An output formatter for Parquet files
/****************************************************************************/
#include <config.h>

#include <arrow/io/api.h>

#include <utils/common/MsgHandler.h>
#include <utils/common/ToString.h>
#include "ParquetFormatter.h"


// ===========================================================================
// member method definitions
// ===========================================================================
ParquetFormatter::ParquetFormatter(const std::string& columnNames, const std::string& compression, const int batchSize)
    : OutputFormatter(OutputFormatterType::PARQUET), myHeaderFormat(columnNames), myBatchSize(batchSize) {
    if (compression == "snappy") {
        myCompression = parquet::Compression::SNAPPY;
    } else if (compression == "gzip") {
        myCompression = parquet::Compression::GZIP;
    } else if (compression == "brotli") {
        myCompression = parquet::Compression::BROTLI;
    } else if (compression == "zstd") {
        myCompression = parquet::Compression::ZSTD;
    } else if (compression == "lz4") {
        myCompression = parquet::Compression::LZ4;
    } else if (compression == "bz2") {
        myCompression = parquet::Compression::BZ2;
    } else if (compression != "" && compression != "uncompressed") {
        WRITE_ERRORF("Unknown compression: %", compression);
    }
    if (!arrow::util::Codec::IsAvailable(myCompression)) {
        WRITE_WARNINGF("Compression '%' not available, falling back to uncompressed.", compression);
        myCompression = parquet::Compression::UNCOMPRESSED;
    }
}

void
ParquetFormatter::openTag(std::ostream& /* into */, const std::string& xmlElement) {
    myXMLStack.push_back(myValues.size());
    if (!myWroteHeader) {
        myCurrentTag = xmlElement;
    }
    if (myMaxDepth == (int)myXMLStack.size() && myWroteHeader && myCurrentTag != xmlElement) {
        WRITE_WARNINGF("Encountered mismatch in XML tags (expected % but got %). Column names may be incorrect.", myCurrentTag, xmlElement);
    }
}


void
ParquetFormatter::openTag(std::ostream& /* into */, const SumoXMLTag& xmlElement) {
    myXMLStack.push_back(myValues.size());
    if (!myWroteHeader) {
        myCurrentTag = toString(xmlElement);
    }
    if (myMaxDepth == (int)myXMLStack.size() && myWroteHeader && myCurrentTag != toString(xmlElement)) {
        WRITE_WARNINGF("Encountered mismatch in XML tags (expected % but got %). Column names may be incorrect.", myCurrentTag, toString(xmlElement));
    }
}


bool
ParquetFormatter::closeTag(std::ostream& into, const std::string& /* comment */) {
    if (myMaxDepth == 0) {
        WRITE_WARNING("Column based formats are still experimental. Autodetection only works for homogeneous output.");
        myMaxDepth = (int)myXMLStack.size();
    }
    if (myMaxDepth == (int)myXMLStack.size() && !myWroteHeader) {
        auto arrow_stream = std::make_shared<ArrowOStreamWrapper>(into);
        std::shared_ptr<parquet::WriterProperties> props = parquet::WriterProperties::Builder().compression(myCompression)->build();
        myParquetWriter = *parquet::arrow::FileWriter::Open(*mySchema, arrow::default_memory_pool(), arrow_stream, props);
        myWroteHeader = true;
    }
    bool writeBatch = false;
    if ((int)myXMLStack.size() == myMaxDepth) {
        if (myCheckColumns && myExpectedAttrs != mySeenAttrs) {
            for (int i = 0; i < (int)myExpectedAttrs.size(); ++i) {
                if (myExpectedAttrs.test(i) && !mySeenAttrs.test(i)) {
                    WRITE_ERRORF("Incomplete attribute set, '%' is missing. This file format does not support Parquet output yet.",
                                 toString((SumoXMLAttr)i));
                }
            }
        }
        int index = 0;
        for (auto& builder : myBuilders) {
            const auto val = myValues[index++];
            PARQUET_THROW_NOT_OK(val == nullptr ? builder->AppendNull() : builder->AppendScalar(*val));
        }
        writeBatch = myBuilders.back()->length() == myBatchSize;
        mySeenAttrs.reset();
    }
    if (writeBatch || myXMLStack.empty()) {
        std::vector<std::shared_ptr<arrow::Array> > data;
        for (auto& builder : myBuilders) {
            std::shared_ptr<arrow::Array> column;
            PARQUET_THROW_NOT_OK(builder->Finish(&column));
            data.push_back(column);
            // builder.reset();
        }
        auto batch = arrow::RecordBatch::Make(mySchema, data.back()->length(), data);
        PARQUET_THROW_NOT_OK(myParquetWriter->WriteRecordBatch(*batch));
    }
    if (!myXMLStack.empty()) {
        while ((int)myValues.size() > myXMLStack.back()) {
            if (!myWroteHeader) {
                mySchema = *mySchema->RemoveField(mySchema->num_fields() - 1);
                myBuilders.pop_back();
            }
            myValues.pop_back();
        }
        myXMLStack.pop_back();
    }
    return false;
}


/****************************************************************************/
