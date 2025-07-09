/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2004-2025 German Aerospace Center (DLR) and others.
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
/// @file    OutputDevice.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    2004
///
// Static storage of an output device and its base (abstract) implementation
/****************************************************************************/
#include <config.h>

#include <map>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>
#ifdef WIN32
#define NOMINMAX
#include <windows.h>
#undef NOMINMAX
#endif
#include "OutputDevice.h"
#include "OutputDevice_File.h"
#include "OutputDevice_COUT.h"
#include "OutputDevice_CERR.h"
#include "OutputDevice_Network.h"
#include "PlainXMLFormatter.h"
#include <utils/common/StringUtils.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/FileHelpers.h>
#include <utils/common/ToString.h>
#include <utils/common/MsgHandler.h>
#include <utils/options/OptionsCont.h>
#include <utils/options/OptionsIO.h>


// ===========================================================================
// static member definitions
// ===========================================================================
std::map<std::string, OutputDevice*> OutputDevice::myOutputDevices;
int OutputDevice::myPrevConsoleCP = -1;


// ===========================================================================
// static method definitions
// ===========================================================================
OutputDevice&
OutputDevice::getDevice(const std::string& name, bool usePrefix) {
#ifdef WIN32
    // fix the windows console output on first call
    if (myPrevConsoleCP == -1) {
        myPrevConsoleCP = GetConsoleOutputCP();
        SetConsoleOutputCP(CP_UTF8);
    }
#endif
    // check whether the device has already been aqcuired
    if (myOutputDevices.find(name) != myOutputDevices.end()) {
        return *myOutputDevices[name];
    }
    // build the device
    const OptionsCont& oc = OptionsCont::getOptions();
    const int len = (int)name.length();
#ifdef HAVE_PARQUET
    const bool isParquet = (oc.exists("output.format") && oc.getString("output.format") == "parquet") || (len > 8 && name.substr(len - 8) == ".parquet");
#else
    const bool isParquet = false;
#endif
    OutputDevice* dev = nullptr;
    // check whether the device shall print to stdout
    if (name == "stdout") {
        dev = OutputDevice_COUT::getDevice();
    } else if (name == "stderr") {
        dev = OutputDevice_CERR::getDevice();
    } else if (FileHelpers::isSocket(name)) {
        try {
            const bool ipv6 = name[0] == '[';  // IPv6 adresses may be written like '[::1]:8000'
            const size_t sepIndex = name.find(":", ipv6 ? name.find("]") : 0);
            const int port = StringUtils::toInt(name.substr(sepIndex + 1));
            dev = new OutputDevice_Network(ipv6 ? name.substr(1, sepIndex - 2) : name.substr(0, sepIndex), port);
        } catch (NumberFormatException&) {
            throw IOError("Given port number '" + name.substr(name.find(":") + 1) + "' is not numeric.");
        } catch (EmptyData&) {
            throw IOError(TL("No port number given."));
        }
    } else {
        std::string name2 = (name == "nul" || name == "NUL") ? "/dev/null" : name;
        if (usePrefix && oc.isSet("output-prefix") && name2 != "/dev/null") {
            std::string prefix = oc.getString("output-prefix");
            const std::string::size_type metaTimeIndex = prefix.find("TIME");
            if (metaTimeIndex != std::string::npos) {
                const time_t rawtime = std::chrono::system_clock::to_time_t(OptionsIO::getLoadTime());
                char buffer [80];
                struct tm* timeinfo = localtime(&rawtime);
                strftime(buffer, 80, "%Y-%m-%d-%H-%M-%S", timeinfo);
                prefix.replace(metaTimeIndex, 4, buffer);
            }
            name2 = FileHelpers::prependToLastPathComponent(prefix, name);
        }
        name2 = StringUtils::substituteEnvironment(name2, &OptionsIO::getLoadTime());
        dev = new OutputDevice_File(name2, isParquet);
    }
    if ((oc.exists("output.format") && oc.getString("output.format") == "csv") || (len > 4 && name.substr(len - 4) == ".csv") || (len > 7 && name.substr(len - 7) == ".csv.gz")) {
        dev->setFormatter(new CSVFormatter(oc.getString("output.column-header"), oc.getString("output.column-separator")[0]));
    }
#ifdef HAVE_PARQUET
    if (isParquet) {
        dev->setFormatter(new ParquetFormatter(oc.getString("output.column-header"), oc.getString("output.compression")));
    }
#endif
    dev->setPrecision();
    dev->getOStream() << std::setiosflags(std::ios::fixed);
    myOutputDevices[name] = dev;
    return *dev;
}


bool
OutputDevice::createDeviceByOption(const std::string& optionName,
                                   const std::string& rootElement,
                                   const std::string& schemaFile) {
    if (!OptionsCont::getOptions().isSet(optionName)) {
        return false;
    }
    OutputDevice& dev = OutputDevice::getDevice(OptionsCont::getOptions().getString(optionName));
    if (rootElement != "") {
        dev.writeXMLHeader(rootElement, schemaFile);
    }
    return true;
}


OutputDevice&
OutputDevice::getDeviceByOption(const std::string& optionName) {
    std::string devName = OptionsCont::getOptions().getString(optionName);
    if (myOutputDevices.find(devName) == myOutputDevices.end()) {
        throw InvalidArgument("Output device '" + devName + "' for option '" + optionName + "' has not been created.");
    }
    return OutputDevice::getDevice(devName);
}


void
OutputDevice::flushAll() {
    for (auto item : myOutputDevices) {
        item.second->flush();
    }
}


void
OutputDevice::closeAll(bool keepErrorRetrievers) {
    std::vector<OutputDevice*> errorDevices;
    std::vector<OutputDevice*> nonErrorDevices;
    for (std::map<std::string, OutputDevice*>::iterator i = myOutputDevices.begin(); i != myOutputDevices.end(); ++i) {
        if (MsgHandler::getErrorInstance()->isRetriever(i->second)) {
            errorDevices.push_back(i->second);
        } else {
            nonErrorDevices.push_back(i->second);
        }
    }
    for (OutputDevice* const dev : nonErrorDevices) {
        try {
            dev->close();
        } catch (const IOError& e) {
            WRITE_ERROR(TL("Error on closing output devices."));
            WRITE_ERROR(e.what());
        }
    }
    if (!keepErrorRetrievers) {
        for (OutputDevice* const dev : errorDevices) {
            try {
                dev->close();
            } catch (const IOError& e) {
                std::cerr << "Error on closing error output devices." << std::endl;
                std::cerr << e.what() << std::endl;
            }
        }
#ifdef WIN32
        if (myPrevConsoleCP != -1) {
            SetConsoleOutputCP(myPrevConsoleCP);
        }
#endif
    }
}


// ===========================================================================
// member method definitions
// ===========================================================================
OutputDevice::OutputDevice(const int defaultIndentation, const std::string& filename) :
    myFilename(filename), myFormatter(new PlainXMLFormatter(defaultIndentation)) {
}


OutputDevice::~OutputDevice() {
    delete myFormatter;
}


bool
OutputDevice::ok() {
    return getOStream().good();
}


const std::string&
OutputDevice::getFilename() {
    return myFilename;
}

void
OutputDevice::close() {
    while (closeTag()) {}
    for (std::map<std::string, OutputDevice*>::iterator i = myOutputDevices.begin(); i != myOutputDevices.end(); ++i) {
        if (i->second == this) {
            myOutputDevices.erase(i);
            break;
        }
    }
    MsgHandler::removeRetrieverFromAllInstances(this);
    delete this;
}


void
OutputDevice::setPrecision(int precision) {
    getOStream() << std::setprecision(precision);
}


bool
OutputDevice::writeXMLHeader(const std::string& rootElement,
                             const std::string& schemaFile,
                             std::map<SumoXMLAttr, std::string> attrs,
                             bool includeConfig) {
    if (schemaFile != "") {
        attrs[SUMO_ATTR_XMLNS] = "http://www.w3.org/2001/XMLSchema-instance";
        attrs[SUMO_ATTR_SCHEMA_LOCATION] = "http://sumo.dlr.de/xsd/" + schemaFile;
    }
    return myFormatter->writeXMLHeader(getOStream(), rootElement, attrs, includeConfig);
}


OutputDevice&
OutputDevice::openTag(const std::string& xmlElement) {
    myFormatter->openTag(getOStream(), xmlElement);
    return *this;
}


OutputDevice&
OutputDevice::openTag(const SumoXMLTag& xmlElement) {
    myFormatter->openTag(getOStream(), xmlElement);
    return *this;
}


bool
OutputDevice::closeTag(const std::string& comment) {
    if (myFormatter->closeTag(getOStream(), comment)) {
        postWriteHook();
        return true;
    }
    return false;
}


void
OutputDevice::postWriteHook() {}


void
OutputDevice::inform(const std::string& msg, const bool progress) {
    if (progress) {
        getOStream() << msg;
    } else {
        getOStream() << msg << '\n';
    }
    postWriteHook();
}


const SumoXMLAttrMask
OutputDevice::parseWrittenAttributes(const std::vector<std::string>& attrList, const std::string& desc, const std::map<std::string, SumoXMLAttrMask>& special) {
    SumoXMLAttrMask result;
    for (std::string attrName : attrList) {
        if (attrName == "all") {
            result.set();
        } else if (special.count(attrName) > 0) {
            result |= special.find(attrName)->second;
        } else {
            if (SUMOXMLDefinitions::Attrs.hasString(attrName)) {
                result.set(SUMOXMLDefinitions::Attrs.get(attrName));
            } else {
                WRITE_ERRORF(TL("Unknown attribute '%' to write in %."), attrName, desc);
            }
        }
    }
    return result;
}


/****************************************************************************/
