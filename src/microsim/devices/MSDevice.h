/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2007-2025 German Aerospace Center (DLR) and others.
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
/// @file    MSDevice.h
/// @author  Michael Behrisch
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @date    Tue, 04 Dec 2007
///
// Abstract in-vehicle device
/****************************************************************************/
#pragma once
#include <config.h>

#include <string>
#include <vector>
#include <map>
#include <set>
#include <random>
#include <microsim/MSMoveReminder.h>
#include <microsim/MSNet.h>
#include <microsim/MSVehicleType.h>
#include <microsim/MSVehicleControl.h>
#include <utils/common/Named.h>
#include <utils/common/StringUtils.h>
#include <utils/common/UtilExceptions.h>
#include <utils/options/OptionsCont.h>


// ===========================================================================
// class declarations
// ===========================================================================
class OutputDevice;
class SUMOVehicle;
class MSTransportable;
class SUMOSAXAttributes;
class MSVehicleDevice;
class MSTransportableDevice;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSDevice
 * @brief Abstract in-vehicle / in-person device
 *
 * The MSDevice-interface brings the following interfaces to a vehicle /person that
 *  may be overwritten by real devices:
 * @arg Building and retrieval of a device id
 */
class MSDevice : public Named {
public:
    /** @brief Inserts options for building devices
     * @param[filled] oc The options container to add the options to
     */
    static void insertOptions(OptionsCont& oc);

    /** @brief check device-specific options
     * @param[filled] oc The options container with the user-defined options
     */
    static bool checkOptions(OptionsCont& oc);


    /** @brief Build devices for the given vehicle, if needed
    *
    * @param[in] v The vehicle for which a device may be built
    * @param[filled] into The vector to store the built device in
    */
    static void buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into);

    /** @brief Build devices for the given person, if needed
    *
    * @param[in] p The person for which a device may be built
    * @param[filled] into The vector to store the built device in
    */
    static void buildTransportableDevices(MSTransportable& p, std::vector<MSTransportableDevice*>& into);

    /// @brief extracts the deviceName from the id (which includes holder id) and is subject to special cases
    static std::string getDeviceName(const std::string& id);

    static SumoRNG* getEquipmentRNG() {
        return &myEquipmentRNG;
    }

    /// @brief return the name for this type of device
    virtual const std::string deviceName() const = 0;

    /// @brief perform cleanup for all devices
    static void cleanupAll();

    static const std::string LOADSTATE_DEVICENAMES;

public:
    /** @brief Constructor
     *
     * @param[in] id The ID of the device
     */
    MSDevice(const std::string& id) : Named(id) {
    }


    /// @brief Destructor
    virtual ~MSDevice() { }


    /** @brief Called on vehicle deletion to extend tripinfo and other outputs
     *
     * The device may write some statistics into the tripinfo output and may
     *  choose to finalize its own outputs. It is assumed that the
     *  information written to tripinfoOut is a valid xml-snipplet, which
     *  will be embedded within the vehicle's tripinfo information.
     *
     * The device should use the openTag / closeTag methods of the OutputDevice
     *  for correct indentation.
     *
     * @exception IOError not yet implemented
     */
    virtual void generateOutput(OutputDevice* /*tripinfoOut*/) const {
    }

    /** @brief Saves the state of the device
     *
     * The default implementation writes a warning and does nothing.
     * @param[in] out The OutputDevice to write the information into
     */
    virtual void saveState(OutputDevice& out) const;

    /** @brief Loads the state of the device from the given description
     *
     * The default implementation does nothing.
     * @param[in] attrs XML attributes describing the current state
     */
    virtual void loadState(const SUMOSAXAttributes& attrs);

    /// @brief try to retrieve the given parameter from this device. Throw exception for unsupported key
    virtual std::string getParameter(const std::string& key) const {
        throw InvalidArgument("Parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
    }

    /// @brief try to set the given parameter for this device. Throw exception for unsupported key
    virtual void setParameter(const std::string& key, const std::string& value) {
        UNUSED_PARAMETER(value);
        throw InvalidArgument("Setting parameter '" + key + "' is not supported for device of type '" + deviceName() + "'");
    }

protected:
    /// @name Helper methods for device assignment
    /// @{

    /** @brief Adds common command options that allow to assign devices to vehicles
     *
     * @param[in] deviceName The name of the device type
     * @param[in] optionsTopic The options topic into which the options shall be added
     * @param[filled] oc The options container to add the options to
     */
    static void insertDefaultAssignmentOptions(const std::string& deviceName, const std::string& optionsTopic, OptionsCont& oc, const bool isPerson = false);


    /** @brief Determines whether a vehicle should get a certain device
     *
     * @param[in] oc The options container to get the information about assignment from
     * @param[in] deviceName The name of the device type
     * @param[in] v The vehicle to determine whether it shall be equipped or not
     */
    template<class DEVICEHOLDER>
    static bool equippedByDefaultAssignmentOptions(const OptionsCont& oc, const std::string& deviceName, DEVICEHOLDER& v, bool outputOptionSet, const bool isPerson = false);
    /// @}


private:
    /// @brief vehicles which explicitly carry a device, sorted by device, first
    static std::map<std::string, std::set<std::string> > myExplicitIDs;

    /// @brief A random number generator used to choose from vtype/route distributions and computing the speed factors
    static SumoRNG myEquipmentRNG;


private:
    /// @brief Invalidated copy constructor.
    MSDevice(const MSDevice&);

    /// @brief Invalidated assignment operator.
    MSDevice& operator=(const MSDevice&);

};


template<class DEVICEHOLDER> bool
MSDevice::equippedByDefaultAssignmentOptions(const OptionsCont& oc, const std::string& deviceName, DEVICEHOLDER& v, bool outputOptionSet, const bool isPerson) {
    const std::string prefix = (isPerson ? "person-device." : "device.") + deviceName;
    // assignment by number
    bool numberGiven = ((oc.exists(prefix + ".deterministic") && oc.getBool(prefix + ".deterministic"))
                        || (oc.exists(prefix + ".probability") && oc.getFloat(prefix + ".probability") >= 0.));
    double probability = numberGiven ? oc.getFloat(prefix + ".probability") : 0;
    // assignment by name
    bool haveByName = false;
    bool nameGiven = false;
    if (oc.exists(prefix + ".explicit") && oc.isSet(prefix + ".explicit")) {
        nameGiven = true;
        if (myExplicitIDs.find(deviceName) == myExplicitIDs.end()) {
            myExplicitIDs[deviceName] = std::set<std::string>();
            const std::vector<std::string> idList = OptionsCont::getOptions().getStringVector(prefix + ".explicit");
            myExplicitIDs[deviceName].insert(idList.begin(), idList.end());
        }
        haveByName = myExplicitIDs[deviceName].count(v.getID()) > 0;
    }
    // assignment by abstract parameters
    bool haveByParameter = false;
    bool parameterGiven = false;
    const std::string key = "has." + deviceName + ".device";
    if (v.getParameter().hasParameter(key)) {
        parameterGiven = true;
        haveByParameter = StringUtils::toBool(v.getParameter().getParameter(key, "false"));
    } else if (v.getVehicleType().getParameter().hasParameter(key)) {
        parameterGiven = true;
        haveByParameter = StringUtils::toBool(v.getVehicleType().getParameter().getParameter(key, "false"));
    } else if (v.getVehicleType().getParameter().hasParameter(prefix + ".probability")) {
        // override global options
        numberGiven = true;
        probability = StringUtils::toDouble(v.getVehicleType().getParameter().getParameter(prefix + ".probability", "0"));
    }
    //std::cout << " deviceName=" << deviceName << " holder=" << v.getID()
    //    << " nameGiven=" << nameGiven << " haveByName=" << haveByName
    //    << " parameterGiven=" << parameterGiven << " haveByParameter=" << haveByParameter
    //    << " numberGiven=" << numberGiven << " haveByNumber=" << haveByNumber
    //    << " outputOptionSet=" << outputOptionSet << "\n";
    if (haveByName) {
        return true;
    } else if (parameterGiven) {
        return haveByParameter;
    } else if (numberGiven) {
        if (oc.exists(prefix + ".deterministic") && oc.getBool(prefix + ".deterministic")) {
            return MSNet::getInstance()->getVehicleControl().getQuota(probability) == 1;
        } else if (probability > 0) {
            if (v.getParameter().hasParameter(LOADSTATE_DEVICENAMES)) {
                // replicate probabilistic assignment
                const std::vector<std::string> lsdn = StringTokenizer(v.getParameter().getParameter(LOADSTATE_DEVICENAMES)).getVector();
                return std::find(lsdn.begin(), lsdn.end(), deviceName) != lsdn.end();
            } else {
                return RandHelper::rand(&myEquipmentRNG) < probability;
            }
        } else {
            return false;
        }
    } else {
        return !nameGiven && outputOptionSet;
    }
}
