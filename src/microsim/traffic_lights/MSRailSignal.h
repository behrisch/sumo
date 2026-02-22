/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2002-2026 German Aerospace Center (DLR) and others.
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
/// @file    MSRailSignal.h
/// @author  Melanie Weber
/// @author  Andreas Kendziorra
/// @author  Jakob Erdmann
/// @date    Jan 2015
///
// A rail signal logic
/****************************************************************************/
#pragma once
#include <config.h>

#include <vector>
#include <microsim/MSRoute.h>
#include <microsim/traffic_lights/MSTrafficLightLogic.h>
#include <microsim/traffic_lights/MSTLLogicControl.h>


// ===========================================================================
// class declarations
// ===========================================================================
class MSLink;
class MSPhaseDefinition;
class MSRailSignalConstraint;
class MSDriveWay;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSRailSignal
 * @brief A signal for rails
 */
class MSRailSignal : public MSTrafficLightLogic {
public:
    /** @brief Constructor
     * @param[in] tlcontrol The tls control responsible for this tls
     * @param[in] id This tls' id
     * @param[in] programID This tls' sub-id (program id)
     * @param[in] parameters This tls' parameters
     * @param[in] delay The time to wait before the first switch
     */
    MSRailSignal(MSTLLogicControl& tlcontrol,
                 const std::string& id, const std::string& programID, SUMOTime delay,
                 const Parameterised::Map& parameters);


    /** @brief Initialises the rail signal with information about adjacent rail signals
     * @param[in] nb The detector builder
     * @exception ProcessError If something fails on initialisation
     */
    void init(NLDetectorBuilder& nb) override;


    /// @brief Destructor
    ~MSRailSignal();

    inline int getNumericalID() const {
        return myNumericalID;
    }

    /**@brief Sets a parameter and updates internal constants */
    void setParameter(const std::string& key, const std::string& value) override;

    /** @brief Adds a link on building
     * @param[in] link The controlled link
     * @param[in] lane The lane this link starts at
     * @param[in] pos The link's index (signal group) within this program
     */
    void addLink(MSLink* link, MSLane* lane, int pos) override;

    /// @name Handling of controlled links
    /// @{

    /** @brief Applies information about controlled links and lanes from the given logic
     * @param[in] logic The logic to use the information about controlled links/lanes from
     * @see MSTrafficLightLogic::adaptLinkInformationFrom
     */
    void adaptLinkInformationFrom(const MSTrafficLightLogic& logic) override;
    /// @}


    /// @name Switching and setting current rows
    /// @{


    /** @brief returns the state of the signal that actually required
     *
     * Returns the state of the rail signal that is actually required based on the
     *  occupation of the adjoining blocks.
     *
     * @return The state actually required for this signal.
     */

    /// @brief updates the current phase of the signal and return whether it should be kept active
    bool updateCurrentPhase();

    /** @brief Switches to the next phase
    * @return The time of the next switch (always the next step)
    * @see MSTrafficLightLogic::trySwitch
    */
    SUMOTime trySwitch() override;

    /// @}


    /// @name Static Information Retrieval
    /// @{

    /** @brief Returns the number of phases
     * @return The number of this tls program's phases (always zero)
     * @see MSTrafficLightLogic::getPhaseNumber
     */
    int getPhaseNumber() const override;

    /** @brief Returns the phases of this tls program
     * @return The phases of this tls program
     * @see MSTrafficLightLogic::getPhases
     */
    const Phases& getPhases() const override;

    /** @brief Returns the definition of the phase from the given position within the plan
    *
    * Returns the current phase as there does not exist a plan of the phases.
    *
    * @param[in] givenstep The index of the phase within the plan
    * @return The definition of the phase at the given position
    * @see MSTrafficLightLogic::getPhase
    */
    const MSPhaseDefinition& getPhase(int givenstep) const override;

    /// @brief whether the given link index ever turns 'G'
    bool getsMajorGreen(int /*linkIndex*/) const override {
        return true;
    }
    /// @}


    /// @name Dynamic Information Retrieval
    /// @{

    /** @brief Returns the current index within the program
     * @return The index of the current phase within the tls (here, always zero will be returned)
     * @see MSTrafficLightLogic::getCurrentPhaseIndex
     */
    int getCurrentPhaseIndex() const override;

    /** @brief Returns the definition of the current phase
    * @return The current phase
    */
    const MSPhaseDefinition& getCurrentPhaseDef() const override;
    /// @}

    void resetLastSwitch(SUMOTime t); 

    /// @name Conversion between time and phase
    /// @{

    /** @brief Returns the index of the logic at the given simulation step
     * @return The (estimated) index of the tls at the given simulation time step (here, always zero will be returned)
     * @see MSTrafficLightLogic::getPhaseIndexAtTime
     */
    SUMOTime getPhaseIndexAtTime(SUMOTime simStep) const override;

    /** @brief Returns the position (start of a phase during a cycle) from of a given step
     * @param[in] index The index of the phase to return the begin of
     * @return The begin time of the phase (here, always zero will be returned)
     * @see MSTrafficLightLogic::getOffsetFromIndex
     */
    SUMOTime getOffsetFromIndex(int index) const override;

    /** @brief Returns the step (the phasenumber) of a given position of the cycle
    * @param[in] offset The offset (time) for which the according phase shall be returned
    * @return The according phase (here, always zero will be returned)
    * @see MSTrafficLightLogic::getIndexFromOffset
    */
    int getIndexFromOffset(SUMOTime offset) const override;
    /// @}


    /// @name Changing phases and phase durations
    /// @{

    /** @brief Changes the current phase and her duration
     * @param[in] tlcontrol The responsible traffic lights control
     * @param[in] simStep The current simulation step
     * @param[in] step Index of the phase to use
     * @param[in] stepDuration The left duration of the phase
     * @see MSTrafficLightLogic::changeStepAndDuration
     */
    void changeStepAndDuration(MSTLLogicControl& tlcontrol, SUMOTime simStep, int step, SUMOTime stepDuration) override {
        UNUSED_PARAMETER(tlcontrol);
        UNUSED_PARAMETER(simStep);
        UNUSED_PARAMETER(step);
        UNUSED_PARAMETER(stepDuration);
    }
    /// @}

    /// @brief return vehicles that block the intersection/rail signal for vehicles that wish to pass the given linkIndex
    VehicleVector getBlockingVehicles(int linkIndex) override;
    std::string getBlockingVehicleIDs() const;
    /// @brief return vehicles that block the given driveWay
    static VehicleVector getBlockingVehicles(const MSDriveWay* dw);

    /// @brief return vehicles that approach the intersection/rail signal and are in conflict with vehicles that wish to pass the given linkIndex
    VehicleVector getRivalVehicles(int linkIndex) override;
    std::string getRivalVehicleIDs() const;

    /// @brief return vehicles that approach the intersection/rail signal and have priority over vehicles that wish to pass the given linkIndex
    VehicleVector getPriorityVehicles(int linkIndex) override;
    std::string getPriorityVehicleIDs() const;

    /// @brief return information regarding active rail signal constraints for the closest approaching vehicle
    std::string getConstraintInfo(int linkIndex);
    std::string getConstraintInfo() const;

    std::vector<const MSDriveWay*> getBlockingDriveWays(int linkIndex) override;
    std::string getBlockingDriveWayIDs() const;

    /// @brief return driveways that block the given driveWay
    static std::vector<const MSDriveWay*> getBlockingDriveWays(const MSDriveWay* dw);

    std::string getRequestedDriveWay(int linkIndex) override;
    std::string getRequestedDriveWay() const;

    /// @brief write rail signal block output for all links and driveways
    void writeBlocks(OutputDevice& od, bool writeVehicles) const;

    /// @brief register constraint for signal switching
    void addConstraint(const std::string& tripId, MSRailSignalConstraint* constraint);

    /// @name TraCI access to constraints
    /// @{
    const std::map<std::string, std::vector<MSRailSignalConstraint*> >&  getConstraints() const {
        return myConstraints;
    }

    /// @brief remove constraint for signal switching
    bool removeConstraint(const std::string& tripId, MSRailSignalConstraint* constraint);
    void removeConstraints();
    /// @}

    /// update driveway for extended deadlock protection
    void updateDriveway(int numericalID);

    /* @brief return whether vehicle insertion must be delayed for an oncoming train
     * @param[in] link The rail signal link before which the vehicle is being inserted
     * @param[in] veh The vehicle being inserted
     * @param[in] brakeBeforeSignal Whether the vehicle may brake before the signal,
     *                              Returns true if the vehicle has to brake before the signal
     */
    static bool hasInsertionConstraint(MSLink* link, const MSVehicle* veh, std::string& info, bool& isInsertionOrder);

    static void initDriveWays(const SUMOVehicle* ego, bool update);

    typedef std::pair<const SUMOVehicle* const, const MSLink::ApproachingVehicleInformation> Approaching;
    typedef std::map<const MSLane*, int, ComparatorNumericalIdLess> LaneVisitedMap;

    /* @brief retrieve driveway with the given numerical id
     * @note: throws exception if the driveway does not exist at this rail signal */
    const MSDriveWay& retrieveDriveWay(int numericalID) const;

    const MSDriveWay& retrieveDriveWayForVeh(int tlIndex, const SUMOVehicle* veh);
    const MSDriveWay& retrieveDriveWayForRoute(int tlIndex, MSRouteIterator first, MSRouteIterator end);

    const std::vector<MSDriveWay*> retrieveDriveWays(int tlIndex) const;

    /// @brief whether the given vehicle is free to drive
    bool constraintsAllow(const SUMOVehicle* veh, bool storeWaitRelation = false) const;

    bool isMovingBlock() const {
        return myMovingBlock;
    }

    static bool storeVehicles() {
        return myStoreVehicles;
    }

    static VehicleVector& blockingVehicles() {
        return myBlockingVehicles;
    }

    static std::vector<const MSDriveWay*>& blockingDriveWays() {
        return myBlockingDriveWays;
    }

    static VehicleVector& rivalVehicles() {
        return myRivalVehicles;
    }

    static VehicleVector& priorityVehicles() {
        return myPriorityVehicles;
    }

    /// @brief print link descriptions
    static std::string describeLinks(std::vector<MSLink*> links);

protected:

    const int myNumericalID;

    std::string getNewDrivewayID();

    /* The driveways for each link
     */
    struct LinkInfo {
        /// @brief constructor
        LinkInfo(MSLink* link);

        /// @brief Destructor
        ~LinkInfo();

        MSLink* myLink;

        /// @brief all driveways immediately following this link
        std::vector<MSDriveWay*> myDriveways;

        /// @brief return id for this railsignal-link
        std::string getID() const;

        /// @brief retrieve an existing Driveway or construct a new driveway based on the vehicles route
        MSDriveWay& getDriveWay(const SUMOVehicle*, int searchStart = -1);
        MSDriveWay& getDriveWay(MSRouteIterator firstIt, MSRouteIterator endIt, const std::string& info = "");

        /// @brief try rerouting vehicle if reservation failed
        void reroute(SUMOVehicle* veh, const MSEdgeVector& occupied);

        /// @brief init LinkInfo
        void reset();

        SUMOTime myLastRerouteTime;
        SUMOVehicle* myLastRerouteVehicle;

        /// @brief whether this link is used for railway operations
        bool myControlled;
    };

    /// @brief data storage for every link at this node (more than one when directly guarding a switch)
    std::vector<LinkInfo> myLinkInfos;

protected:

    /** @brief The list of phases this logic uses
    *
    *   This vector is always empty and only constructed because the superclass MSTrafficLightLogic requires it.
    */
    Phases myPhases;

    /// @brief The current phase
    MSPhaseDefinition myCurrentPhase;

    /// @brief MSTrafficLightLogic requires that the phase index changes whenever signals change their state
    int myPhaseIndex;

    /// @brief whether the signal is in moving block mode (only protects from oncoming and flanking trains)
    bool myMovingBlock;

    /// @brief map from tripId to constraint list
    std::map<std::string, std::vector<MSRailSignalConstraint*> > myConstraints;

    /// @brief running number of driveways created for this signal
    int myDriveWayIndex;

    static int myRSIndex;

protected:
    /// @brief update vehicle lists for traci calls
    void storeTraCIVehicles(int linkIndex);
    static void storeTraCIVehicles(const MSDriveWay* dw);

    /// @brief reset temporary storage for injected conflict output
    static void resetStored();

    /// @name traci result storage
    //@{
    static bool myStoreVehicles;
    static VehicleVector myBlockingVehicles;
    static VehicleVector myRivalVehicles;
    static VehicleVector myPriorityVehicles;
    static std::string myConstraintInfo;
    static std::vector<const MSDriveWay*> myBlockingDriveWays;
    static std::string myRequestedDriveWay;
    //@}


};
