/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2007-2026 German Aerospace Center (DLR) and others.
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
/// @file    SUMOSAXAttributes.h
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Fri, 30 Mar 2007
///
// Encapsulated SAX-Attributes
/****************************************************************************/
#pragma once
#include <config.h>

#include <string>
#include <vector>
#include <set>

#include <utils/common/StringUtils.h>
#include <utils/common/SUMOTime.h>
#include <utils/common/ToString.h>
#include <utils/common/UtilExceptions.h>

#include "SUMOXMLDefinitions.h"

// ===========================================================================
// class declarations
// ===========================================================================

class Position;
class PositionVector;
class Boundary;
class RGBColor;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class SUMOSAXAttributes
 * @brief Encapsulated SAX-Attributes
 *
 * This class is an interface for using encapsulated SAX-attributes.
 * Encapsulation is done to allow a common access without the need to
 *  import all the Xerces-definitions.
 */
class SUMOSAXAttributes {
public:
    /* @brief Constructor
     * @param[in] tagName The name of the parsed object type; used for error message generation
     */
    SUMOSAXAttributes(const std::string& objectType);


    /// @brief Destructor
    virtual ~SUMOSAXAttributes() { }


    /** @brief Tries to read given attribute assuming it is an int
     *
     * If an error occurs (the attribute is not there, it is not numeric), "ok" is
     *  set to false. If report is true an error message is written to MsgHandler::getErrorInstance.
     *
     * If the value could be read, "ok" is not changed, and the value is returned.
     *
     * @param[in] attr The id of the attribute to read
     * @param[in] objectid The name of the parsed object; used for error message generation
     * @param[out] ok Whether the value could be read
     * @param[in] report Whether errors shall be written to msg handler's error instance
     * @return The read value if given and correct; -1 if an error occurred
     */
    template <typename T>
    T get(int attr, const char* objectid, bool& ok, bool report = true) const;


    /** @brief Tries to read given attribute assuming it is an int
     *
     * If the attribute is not existing in the current element, the default value is returned.
     * If an error occurs on parsing (the attribute is empty, it is not numeric), "ok" is
     *  set to false. If report is true an error message is written to MsgHandler::getErrorInstance.
     *
     * If the value could be read, "ok" is not changed, and the value is returned.
     *
     * @param[in] attr The id of the attribute to read
     * @param[in] objectid The name of the parsed object; used for error message generation
     * @param[out] ok Whether the value could be read
     * @param[in] defaultValue The value to return if the attribute is not within the element
     * @param[in] report Whether errors shall be written to msg handler's error instance
     * @return The read value if given and correct; the default value if the attribute does not exist;  -1 if an error occurred
     */
    template <typename T>
    T getOpt(int attr, const char* objectid, bool& ok, T defaultValue = T(), bool report = true) const;


    /** @brief Tries to read given attribute assuming it is a SUMOTime
     *
     * If an error occurs (the attribute is not there, it is not numeric), "ok" is
     *  set to false and an error message is written to MsgHandler::getErrorInstance.
     *
     * Otherwise, "ok" is not changed.
     *
     * In dependence to the used time representation, either get<int> or get<double>
     *  is used.
     *
     * @param[in] attr The id of the attribute to read
     * @param[in] objectid The name of the parsed object; used for error message generation
     * @param[out] ok Whether the value could be read
     * @param[in] report Whether errors shall be written to msg handler's error instance
     * @return The read value if given and correct; -1 if an error occurred
     */
    SUMOTime getSUMOTimeReporting(int attr, const char* objectid, bool& ok,
                                  bool report = true) const;


    /** @brief Tries to read the SUMOTime 'period' attribute
     *
     * If 'period' cannot be found, tries 'freq' as an alias.
     *
     * If an error occurs (the attribute is not there, it is not numeric), "ok" is
     *  set to false and an error message is written to MsgHandler::getErrorInstance.
     *
     * Otherwise, "ok" is not changed.
     *
     * In dependence to the used time representation, either get<int> or get<double>
     *  is used.
     *
     * @param[in] objectid The name of the parsed object; used for error message generation
     * @param[out] ok Whether the value could be read
     * @param[in] report Whether errors shall be written to msg handler's error instance
     * @return The read value if given and correct; -1 if an error occurred
     */
    SUMOTime getPeriod(const char* objectid, bool& ok, bool report = true) const;


    /** @brief Tries to read given attribute assuming it is a SUMOTime
     *
     * If the attribute is not existing in the current element, the default value is returned.
     * If an error occurs on parsing (the attribute is empty, it is not numeric), "ok" is
     *  set to false. If report is true an error message is written to MsgHandler::getErrorInstance.
     *
     * Otherwise, "ok" is not changed.
     *
     * In dependence to the used time representation, either get<int> or get<double>
     *  is used.
     *
     * @param[in] attr The id of the attribute to read
     * @param[in] objectid The name of the parsed object; used for error message generation
     * @param[out] ok Whether the value could be read
     * @param[in] defaultValue The value to return if the attribute is not within the element
     * @param[in] report Whether errors shall be written to msg handler's error instance
     * @return The read value if given and correct; the default value if the attribute does not exist;  -1 if an error occurred
     */
    SUMOTime getOptSUMOTimeReporting(int attr, const char* objectid, bool& ok,
                                     SUMOTime defaultValue, bool report = true) const;

    /** @brief Tries to read given attribute assuming it is a tls offset
     * (SUMOTime or "begin")
     *
     * If the attribute is not existing in the current element, the default value is returned.
     * If an error occurs on parsing (the attribute is empty, it is not numeric), "ok" is
     *  set to false. If report is true an error message is written to MsgHandler::getErrorInstance.
     *
     * Otherwise, "ok" is not changed.
     *
     * In dependence to the used time representation, either get<int> or get<double>
     *  is used.
     *
     * @param[in] attr The id of the attribute to read
     * @param[in] objectid The name of the parsed object; used for error message generation
     * @param[out] ok Whether the value could be read
     * @param[in] defaultValue The value to return if the attribute is not within the element
     * @param[in] report Whether errors shall be written to msg handler's error instance
     * @return The read value if given and correct; the default value if the attribute does not exist;  -1 if an error occurred
     */
    SUMOTime getOptOffsetReporting(int attr, const char* objectid, bool& ok,
                                   SUMOTime defaultValue, bool report = true) const;

    /** @brief Tries to read the SUMOTime 'period' attribute
     *
     * If 'period' cannot be found, tries 'freq' as an alias.
     *
     * If both attributes do not exist in the current element, the default value is returned.
     * If an error occurs on parsing (the attribute is empty, it is not numeric), "ok" is
     *  set to false. If report is true an error message is written to MsgHandler::getErrorInstance.
     *
     * Otherwise, "ok" is not changed.
     *
     * In dependence to the used time representation, either get<int> or get<double>
     *  is used.
     *
     * @param[in] objectid The name of the parsed object; used for error message generation
     * @param[out] ok Whether the value could be read
     * @param[in] defaultValue The value to return if the attribute is not within the element
     * @param[in] report Whether errors shall be written to msg handler's error instance
     * @return The read value if given and correct; the default value if the attribute does not exist;  -1 if an error occurred
     */
    SUMOTime getOptPeriod(const char* objectid, bool& ok, SUMOTime defaultValue, bool report = true) const;



    /// @name virtual methods for retrieving attribute values
    /// @{

    /** @brief Returns the information whether the named (by its enum-value) attribute is within the current list
     *
     * @param[in] id The id of the attribute to search for
     * @return Whether the attribute is within the attributes
     */
    virtual bool hasAttribute(int id) const = 0;


    /** @brief Returns the information whether the named attribute is within the current list
     *
     * @param[in] id The name of the attribute to search for
     * @return Whether the named attribute is within the attributes
     */
    virtual bool hasAttribute(const std::string& id) const = 0;


    /**
     * @brief Returns the bool-value of the named (by its enum-value) attribute
     *
     * Tries to retrieve the attribute from the attribute list. The retrieved
     *  attribute  (which may be 0) is then parsed using StringUtils::toBool.
     *  If the attribute is not set or the empty string, StringUtils::toBool throws an
     *  EmptyData exception which is passed.
     * If the value can not be parsed to a bool, StringUtils::toBool throws a
     *  BoolFormatException which is passed.
     *
     * @param[in] id The id of the attribute to return the value of
     * @return The attribute's value as a bool, if it could be read and parsed
     * @exception EmptyData If the attribute is not known or the attribute value is an empty string
     * @exception BoolFormatException If the attribute value can not be parsed to a bool
     */
    inline bool getBool(int id) const {
        return StringUtils::toBool(getString(id));
    }

    /**
     * @brief Returns the int-value of the named (by its enum-value) attribute
     *
     * Tries to retrieve the attribute from the attribute list. The retrieved
     *  attribute  (which may be 0) is then parsed using StringUtils::toInt.
     *  If the attribute is the empty string or not set, StringUtils::toInt throws an
     *  EmptyData exception which is passed.
     * If the value can not be parsed to an int, StringUtils::toInt throws a
     *  NumberFormatException which is passed.
     *
     * @param[in] id The id of the attribute to return the value of
     * @return The attribute's value as an int, if it could be read and parsed
     * @exception EmptyData If the attribute is not known or the attribute value is an empty string
     * @exception NumberFormatException If the attribute value can not be parsed to an int
     */
    inline int getInt(int id) const {
        return StringUtils::toInt(getString(id));
    }


    /**
     * @brief Returns the long-value of the named (by its enum-value) attribute
     *
     * Tries to retrieve the attribute from the attribute list. The retrieved
     *  attribute  (which may be 0) is then parsed using TplConvert<XMLCh>::_2long.
     *  If the attribute is empty or ==0, TplConvert<XMLCh>::_2long throws an
     *  EmptyData-exception which is passed.
     * If the value can not be parsed to a long, TplConvert<XMLCh>::_2long throws a
     *  NumberFormatException-exception which is passed.
     *
     * @param[in] id The id of the attribute to return the value of
     * @return The attribute's value as an int, if it could be read and parsed
     * @exception EmptyData If the attribute is not known or the attribute value is an empty string
     * @exception NumberFormatException If the attribute value can not be parsed to an int
     */
    virtual long long int getLong(int id) const {
        return StringUtils::toLong(getString(id));
    }


    /**
     * @brief Returns the string-value of the named (by its enum-value) attribute
     *
     * Tries to retrieve the attribute from the attribute list. The retrieved
     *  attribute  (which may be 0) is then parsed using TplConvert<XMLCh>::_2str.
     *  If the attribute is ==0, TplConvert<XMLCh>::_2str throws an
     *  EmptyData-exception which is passed.
     *
     * @param[in] id The id of the attribute to return the value of
     * @return The attribute's value as a string, if it could be read and parsed
     * @exception EmptyData If the attribute is not known or the attribute value is an empty string
     */
    virtual std::string getString(int id, bool* isPresent = nullptr) const = 0;


    /**
     * @brief Returns the string-value of the named (by its enum-value) attribute
     *
     * Tries to retrieve the attribute from the attribute list. The retrieved
     *  attribute  (which may be 0) is then parsed using TplConvert<XMLCh>::_2strSec.
     *  If the attribute is ==0, TplConvert<XMLCh>::_2strSec returns the default value.
     *
     * @param[in] id The id of the attribute to return the value of
     * @param[in] def The default value to return if the attribute is not in attributes
     * @return The attribute's value as a string, if it could be read and parsed
     * @exception EmptyData If the attribute is not known or the attribute value is an empty string
     */
    virtual std::string getStringSecure(int id, const std::string& def) const = 0;


    /**
     * @brief Returns the double-value of the named (by its enum-value) attribute
     *
     * Tries to retrieve the attribute from the attribute list. The retrieved
     *  attribute  (which may be 0) is then parsed using TplConvert<XMLCh>::_2double.
     *  If the attribute is empty or ==0, TplConvert<XMLCh>::_2double throws an
     *  EmptyData-exception which is passed.
     * If the value can not be parsed to a double, TplConvert<XMLCh>::_2double throws a
     *  NumberFormatException-exception which is passed.
     *
     * @param[in] id The id of the attribute to return the value of
     * @return The attribute's value as a float, if it could be read and parsed
     * @exception EmptyData If the attribute is not known or the attribute value is an empty string
     * @exception NumberFormatException If the attribute value can not be parsed to an double
     */
    inline double getFloat(int id) const {
        return StringUtils::toDouble(getString(id));
    }


    /**
     * @brief Returns the double-value of the named attribute
     *
     * Tries to retrieve the attribute from the attribute list. The retrieved
     *  attribute  (which may be 0) is then parsed using TplConvert<XMLCh>::_2double.
     *  If the attribute is empty or ==0, TplConvert<XMLCh>::_2double throws an
     *  EmptyData-exception which is passed.
     * If the value can not be parsed to a double, TplConvert<XMLCh>::_2double throws a
     *  NumberFormatException-exception which is passed.
     *
     * @param[in] id The name of the attribute to return the value of
     * @return The attribute's value as a float, if it could be read and parsed
     * @exception EmptyData If the attribute is not known or the attribute value is an empty string
     * @exception NumberFormatException If the attribute value can not be parsed to an double
     */
    virtual double getFloat(const std::string& id) const = 0;


    /**
     * @brief Returns the string-value of the named (by its enum-value) attribute
     *
     * Tries to retrieve the attribute from the attribute list.
     *  If the attribute is ==0, TplConvert<XMLCh>::_2strSec returns the default value.
     * @param[in] id The name of the attribute to return the value of
     * @param[in] def The default value to return if the attribute is not in attributes
     * @return The attribute's value as a string, if it could be read and parsed
     */
    virtual std::string getStringSecure(const std::string& id,
                                        const std::string& def) const = 0;
    //}


    /** @brief Converts the given attribute id into a man readable string
     *
     * @param[in] attr The id of the attribute to return the name of
     * @return The name of the described attribute
     */
    virtual std::string getName(int attr) const = 0;

    /** @brief Prints all attribute names and values into the given stream
     *
     * @param[in] os The stream to use
     */
    virtual void serialize(std::ostream& os) const = 0;

    /** @brief Retrieves all attribute names
     */
    virtual std::vector<std::string> getAttributeNames() const = 0;

    /// @brief return the objecttype to which these attributes belong
    const std::string& getObjectType() const {
        return myObjectType;
    }

    friend std::ostream& operator<<(std::ostream& os, const SUMOSAXAttributes& src);

    /// @brief return a new deep-copy attributes object
    virtual SUMOSAXAttributes* clone() const = 0;

    /** @brief The encoding of parsed strings */
    static const std::string ENCODING;


protected:
    template <typename T> T fromString(const std::string& value) const;
    void emitUngivenError(const std::string& attrname, const char* objectid) const;
    void emitEmptyError(const std::string& attrname, const char* objectid) const;
    void emitFormatError(const std::string& attrname, const std::string& type, const char* objectid) const;

private:
    /// @brief Invalidated copy constructor.
    SUMOSAXAttributes(const SUMOSAXAttributes& src) = delete;

    /// @brief Invalidated assignment operator.
    SUMOSAXAttributes& operator=(const SUMOSAXAttributes& src) = delete;

    /// @brief the object type to use in error reporting
    std::string myObjectType;

};


inline std::ostream& operator<<(std::ostream& os, const SUMOSAXAttributes& src) {
    src.serialize(os);
    return os;
}


template<typename X> struct invalid_return {
    static const X value;
};

#define INVALID_RETURN(TYPE) \
template<> struct invalid_return<TYPE> { \
    static const TYPE value; \
}
INVALID_RETURN(std::string);
INVALID_RETURN(int);
INVALID_RETURN(long long int);
INVALID_RETURN(double);
INVALID_RETURN(bool);
INVALID_RETURN(RGBColor);
INVALID_RETURN(Position);
INVALID_RETURN(PositionVector);
INVALID_RETURN(Boundary);
INVALID_RETURN(SumoXMLEdgeFunc);
INVALID_RETURN(SumoXMLNodeType);
INVALID_RETURN(RightOfWay);
INVALID_RETURN(FringeType);
INVALID_RETURN(RoundaboutType);
INVALID_RETURN(ParkingType);
INVALID_RETURN(std::vector<std::string>);
INVALID_RETURN(std::vector<int>);
INVALID_RETURN(std::vector<double>);


template <typename T>
T SUMOSAXAttributes::get(int attr, const char* objectid,
                         bool& ok, bool report) const {
    try {
        bool isPresent = true;
        const std::string& strAttr = getString(attr, &isPresent);
        if (isPresent) {
            return fromString<T>(strAttr);
        }
        if (report) {
            emitUngivenError(getName(attr), objectid);
        }
    } catch (const FormatException& e) {
        if (report) {
            emitFormatError(getName(attr), e.what(), objectid);
        }
    } catch (EmptyData&) {
        if (report) {
            emitEmptyError(getName(attr), objectid);
        }
    }
    ok = false;
    return invalid_return<T>::value;
}


template <typename T>
T SUMOSAXAttributes::getOpt(int attr, const char* objectid,
                            bool& ok, T defaultValue, bool report) const {
    try {
        bool isPresent = true;
        const std::string& strAttr = getString(attr, &isPresent);
        if (isPresent) {
            return fromString<T>(strAttr);
        }
        return defaultValue;
    } catch (const FormatException& e) {
        if (report) {
            emitFormatError(getName(attr), e.what(), objectid);
        }
    } catch (EmptyData&) {
        if (report) {
            emitEmptyError(getName(attr), objectid);
        }
    }
    ok = false;
    return invalid_return<T>::value;
}
