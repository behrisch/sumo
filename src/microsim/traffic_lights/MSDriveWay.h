/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2002-2025 German Aerospace Center (DLR) and others.
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
/// @file    MSDriveWay.h
/// @author  Jakob Erdmann
/// @date    December 2021
///
// A sequende of rail tracks (lanes) that may be used as a "set route" (Fahrstraße)
/****************************************************************************/
#pragma once
#include <config.h>

#include <utils/common/Named.h>
#include <microsim/MSMoveReminder.h>
#include <microsim/MSRoute.h>

// ===========================================================================
// class declarations
// ===========================================================================
class SUMOVehicle;
class MSLane;
class MSLink;
class MSRailSignal;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSDriveWay
 */
class MSDriveWay : public MSMoveReminder, public Named {
public:
    typedef std::pair<const SUMOVehicle* const, const MSLink::ApproachingVehicleInformation> Approaching;
    typedef std::set<const MSLane*, ComparatorNumericalIdLess> LaneSet;
    typedef std::map<const MSLane*, int, ComparatorNumericalIdLess> LaneVisitedMap;

    struct VehicleEvent {
        VehicleEvent(SUMOTime _time, bool _isEntry, const std::string& _id, Notification _reason):
            time(_time), isEntry(_isEntry), id(_id), reason(_reason) {}
        SUMOTime time;
        bool isEntry;
        std::string id;
        Notification reason;
    };

    /*  The driveways (Fahrstrassen) for each link index of MSRailSignal
     *  Each link index has at least one driveway
     *  A driveway describes one possible route that passes the signal up to
     *  the next secure point
     *  When a signal guards a switch (indirect guard) that signal stores two
     *  or more driveways
     */
    MSDriveWay(const MSLink* origin, const std::string& id, bool temporary = false);

    /// @brief Destructor
    virtual ~MSDriveWay();

    static void cleanup();

    static void clearState();

    static bool haveDriveWays() {
        return myGlobalDriveWayIndex > 0;
    }

    static const MSDriveWay* retrieveDepartDriveWay(const MSEdge* edge, const std::string& id);

    bool notifyEnter(SUMOTrafficObject& veh, Notification reason, const MSLane* enteredLane);
    bool notifyLeave(SUMOTrafficObject& veh, double lastPos, Notification reason, const MSLane* enteredLane = 0);
    bool notifyLeaveBack(SUMOTrafficObject& veh, Notification reason, const MSLane* leftLane);
    bool notifyReroute(SUMOTrafficObject& veh);

    /// @brief Wether there is a flank conflict with the given driveway
    bool flankConflict(const MSDriveWay& other) const;

    /// @brief Wether there is a crossing conflict with the given driveway
    bool crossingConflict(const MSDriveWay& other) const;

    /// @brief Wether there is a bidi conflict with the given driveway
    bool bidiBlockedBy(const MSDriveWay& other) const;

    /// @brief Wether there is a bidi conflict with the end of the given driveway
    bool bidiBlockedByEnd(const MSDriveWay& other) const;

    /// @brief Wether the route of other passes into the forward section of this driveway
    bool forwardRouteConflict(std::set<const MSEdge*> forward, const MSDriveWay& other, bool secondCheck = false);

    /// @brief whether any of myConflictLanes is occupied (vehicles that are the target of a join must be ignored)
    bool conflictLaneOccupied(bool store = true, const SUMOVehicle* ego = nullptr) const;

    /// @brief whether any of myFoes is occupied (vehicles that are the target of a join must be ignored)
    bool foeDriveWayOccupied(bool store, const SUMOVehicle* ego, MSEdgeVector& occupied) const;

    /// @brief whether any of my Foes is being approached
    bool foeDriveWayApproached() const;

    /// @brief attempt reserve this driveway for the given vehicle
    bool reserve(const Approaching& closest, MSEdgeVector& occupied);

    /// @brief Write block items for this driveway
    void writeBlocks(OutputDevice& od) const;
    void writeBlockVehicles(OutputDevice& od) const;

    const std::vector<const MSEdge*>& getRoute() const {
        return myRoute;
    }

    const std::vector<const MSLane*>& getFlank() const {
        return myFlank;
    }

    const std::vector<const MSLane*>& getBidi() const {
        return myBidi;
    }

    const std::vector<MSLink*>& getConflictLinks() const {
        return myConflictLinks;
    }

    int getNumericalID() const {
        return myNumericalID;
    }

    const std::vector<VehicleEvent>& getEvents() const {
        return myVehicleEvents;
    }

    void setEvents(const std::vector<VehicleEvent>& events) {
        myVehicleEvents = events;
    }

    void setVehicle(const std::string& vehID) {
        myFirstVehicle = vehID;
    }

    const std::vector<MSDriveWay*>& getFoes() const {
        return myFoes;
    }

    const std::vector<const MSLane*>& getForward() const {
        return myForward;
    }

    /// @brief whether the give route matches this driveway
    bool match(MSRouteIterator firstIt, MSRouteIterator endIt) const;

    void addDWDeadlock(const std::vector<const MSDriveWay*>& deadlockFoes);

    bool isDepartDriveway() const {
        return myOrigin == nullptr;
    };

    const MSLink* getOrigin() const {
        return myOrigin;
    }

    /// @brief whether the given train is on this driveway
    bool hasTrain(SUMOVehicle* veh) const;

    const std::vector<MSDriveWay*>& getSubDriveWays() const {
        return mySubDriveWays;
    }

    static void init();

    static bool hasRS(const MSEdge* cur, const MSEdge* next);

    /// @brief Whether veh must yield to the foe train
    static bool mustYield(const Approaching& veh, const Approaching& foe);

    /// @brief construct a new driveway by searching along the given route until all block structures are found
    static MSDriveWay* buildDriveWay(const std::string& id, const MSLink* link, MSRouteIterator first, MSRouteIterator end);

    /// @brief return logicID_linkIndex in a way that allows clicking in sumo-gui
    static std::string getClickableTLLinkID(const MSLink* link);

    static const MSDriveWay* getDepartureDriveway(const SUMOVehicle* veh, bool init = false);

    static void writeDepatureBlocks(OutputDevice& od, bool writeVehicles);

    /** @brief Save driveway occupancy into the given stream */
    static void saveState(OutputDevice& out);
    static void loadState(const SUMOSAXAttributes& attrs, int tag);

protected:

    /// @brief global driveway index
    int myNumericalID;

    /// @brief the link that enters this driveway or nullptr for a departure driveWay
    const MSLink* myOrigin;

    /// @brief whether the current signal is switched green for a train approaching this block
    const SUMOVehicle* myActive;

    /// @brief list of edges for matching against train routes
    std::vector<const MSEdge*> myRoute;

    /// @brief number of edges in myRoute where overlap with other driveways is forbidden
    int myCoreSize;

    /// @brief number of normal edges in myForward
    int myForwardEdgeCount;

    /// @brief whether this driveway ends its forward section with a rail signal (and thus comprises a full block)
    bool myFoundSignal;
    bool myFoundJump;
    bool myTerminateRoute;
    /// @brief whether driveway building was aborted due to MAX_BLOCK_LENGTH
    bool myAbortedBuild;
    /// @brief whether driveway building was aborted when no further bidi edge was found
    bool myBidiEnded;
    bool myIsSubDriveway;

    /* @brief the actual driveway part up to the next railsignal (halting position)
     * This must be free of other trains */
    std::vector<const MSLane*> myForward;

    /* @brief the list of bidirectional edges that can enter the forward
     * section and which must also be free of traffic
     * (up to the first element that could give protection) */
    std::vector<const MSLane*> myBidi;

    /* @brief the list of bidirectional edges that can enter the forward
     * section and which might contain deadlock-relevant traffic */
    std::vector<const MSLane*> myBidiExtended;

    /* @brief the list of edges that merge with the forward section
     * (found via backward search, up to the first element that could give protection) */
    std::vector<const MSLane*> myFlank;

    /// @brief the lanes that must be clear of trains before this signal can switch to green
    std::vector<const MSLane*> myConflictLanes;

    /* The conflict links for this block
     * Conflict resolution must be performed if vehicles are approaching the
     * current link and any of the conflict links */
    std::vector<MSLink*> myConflictLinks;

    /// @brief Whether the approaching vehicle is prevent from driving by another vehicle approaching the given link
    bool hasLinkConflict(const Approaching& closest, const MSLink* foeLink) const;

    /// @brief Wether this driveway (route) overlaps with the given one
    bool overlap(const MSDriveWay& other) const;

    /* @brief determine route that identifies this driveway (a subset of the
     * vehicle route)
     * collects:
     *   myRoute
     *   myForward
     *   myBidi
     *   myProtectedBidi
     *
     * returns edge that is assumed to safe from oncoming-deadlock or nullptr
     */
    void buildRoute(const MSLink* origin, MSRouteIterator next, MSRouteIterator end, LaneVisitedMap& visited, std::set<MSLink*>&);

    /* @brief find switches that threaten this driveway
     * @param[out] flankSwitches collect the switches
     */
    void checkFlanks(const MSLink* originLink, const std::vector<const MSLane*>& lanes, const LaneVisitedMap& visited, bool allFoes, std::set<MSLink*>& flankSwitches) const;

    /* @brief find links that cross the driveway without entering it
     * @param[out] flankSwitches collect the switches
     */
    void checkCrossingFlanks(MSLink* dwLink, const LaneVisitedMap& visited, std::set<MSLink*>& flankSwitches) const;

    /* @brief find upstream protection from the given link
     * @param[out] flank: the stored flank lanes
     */
    void findFlankProtection(MSLink* link, MSLink* origLink, std::vector<const MSLane*>& flank);

    /// @brief add all driveWays that start at the given link as foes
    void addFoes(const MSLink* link);

    /// @brief add sidings for the given foe
    void addSidings(MSDriveWay* foe, bool addToFoe = false);

    /// @brief derive foe driveways based on myBidi or myBidiExtended
    void addBidiFoes(const MSRailSignal* ownSignal, bool extended);

    /// @brief derive foe driveways that start at the same signal
    void addParallelFoes(const MSLink* link, const MSEdge* first);

    /// @brief derive foe driveways that enter the bidi section by reversing
    void addReversalFoes(bool movingBlock);

    /* @brief build shortened driveway that ends where the foe train leaves the conflict zone of this driveway
     * @return whether the foe has received a new entry in myFoes
     */
    bool buildSubFoe(MSDriveWay* foe, bool movingBlock);

    /// @brief add symmetical conflict link for foes when building a new driveway
    void addConflictLink(const MSLink* link);

    /// @brief return whether a siding can be used. If a siding exist but is occupied, also return the occupied driveway in the siding
    std::pair<bool, const MSDriveWay*> canUseSiding(const SUMOVehicle* ego, const MSDriveWay* foe, bool recurse = true) const;

    bool isFoeOrSubFoe(const MSDriveWay* foe) const;

    bool forwardEndOnRoute(const MSDriveWay* foe) const;

    void addSwitchFoes(MSLink* link);

    bool haveSubTrains() const;

    /* @brief whether the train would have matched this driveway in it's past
     * @return If matching, returns the number of edges the vehicle has gone past the start of the driveway,
     *         Indicate no-match by returning a negative value */
    int matchesPastRoute(SUMOVehicle& sveh) const;

    /// @brief helper method for notifyEnter
    void enterDriveWay(SUMOVehicle& sveh, Notification reason);

    static bool hasJoin(const SUMOVehicle* ego, const SUMOVehicle* foe);

    static bool isSwitch(const MSLink* link);

    void _saveState(OutputDevice& out) const;

    /// @brief return logicID_linkIndex
    static std::string getTLLinkID(const MSLink* link);

    /// @brief return junctionID_junctionLinkIndex
    static std::string getJunctionLinkID(const MSLink* link);

    /// @brief print link descriptions
    static std::string formatVisitedMap(const LaneVisitedMap& visited);

    /// @brief append to map by map index and avoid undefined behavior
    static void appendMapIndex(LaneVisitedMap& map, const MSLane* lane);

private:

    struct Siding {
        Siding(int s, int e, double l) : start(s), end(e), length(l) {}
        // indices along route
        int start;
        int end;
        double length;
    };

    std::set<SUMOVehicle*> myTrains;

    std::vector<VehicleEvent> myVehicleEvents;
    std::vector<MSDriveWay*> myFoes;
    std::map<const MSDriveWay*, std::vector<Siding>, ComparatorIdLess> mySidings;
    std::vector<std::set <const MSDriveWay*> > myDeadlocks;

    /* @brief shortened versions of this driveway to be used as foes instead of the long original
     * (ends as soon as the train has left a particular conflict section)
     * they are never attached to a LinkInfo and thus never the target of the match() function */
    std::vector<MSDriveWay*> mySubDriveWays;

    /// @brief track own occurences in myReversalDriveWays for cleanup in destructor
    std::vector<const MSEdge*> myReversals;

    /// @brief the first vehicle using this driveway
    std::string myFirstVehicle;

    static int myGlobalDriveWayIndex;
    static bool myWriteVehicles;
    static std::set<const MSEdge*> myBlockLengthWarnings;

    /// @brief all driveways passing the given switch (used to look up flank foes)
    static std::map<const MSLink*, std::vector<MSDriveWay*> > mySwitchDriveWays;

    /// @brief all driveways reversing on the given switch (used to look up flank foes)
    static std::map<const MSEdge*, std::vector<MSDriveWay*> > myReversalDriveWays;

    /// @brief all driveways that do not start at a rail signal (and are only used at departure)
    static std::map<const MSEdge*, std::vector<MSDriveWay*>, ComparatorNumericalIdLess > myDepartureDriveways;
    static std::map<const MSJunction*, int> myDepartDrivewayIndex;
    /// @brief all driveways that do not start at a rail signal (and are only used at departure) by end edge
    static std::map<const MSEdge*, std::vector<MSDriveWay*> > myDepartureDrivewaysEnds;

    /// @brief all driveways that end on the given edge
    static std::map<const MSEdge*, std::vector<MSDriveWay*>, ComparatorNumericalIdLess> myEndingDriveways;

    /// @brief lookup table for state loading
    static std::map<ConstMSEdgeVector, MSDriveWay*> myDriveWayRouteLookup;
    static std::map<std::string, MSDriveWay*> myDriveWayLookup;

};
