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
/// @file    MSLeaderInfo.h
/// @author  Jakob Erdmann
/// @date    Oct 2015
///
// Information about vehicles ahead (may be multiple vehicles if
// lateral-resolution is active)
/****************************************************************************/
#pragma once
#include <config.h>

#include <string>
#include <vector>
#include <utils/common/StdDefs.h>

// ===========================================================================
// class declarations
// ===========================================================================
class MSVehicle;
class MSLane;


// ===========================================================================
// type definitions
// ===========================================================================
typedef std::pair<const MSVehicle*, double> CLeaderDist;
typedef std::pair<MSVehicle*, double> LeaderDist;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSLeaderInfo
 */
class MSLeaderInfo {
public:
    /// Constructor
    MSLeaderInfo(const double laneWidth, const MSVehicle* ego = nullptr, const double latOffset = 0.);

    /// Destructor
    virtual ~MSLeaderInfo();

    /* @brief adds this vehicle as a leader in the appropriate sublanes
     * @param[in] veh The vehicle to add
     * @param[in] beyond Whether the vehicle is beyond the existing leaders (and thus may be shadowed by them)
     * @param[in] latOffset The lateral offset that must be added to the position of veh
     * @return The number of free sublanes
     */
    virtual int addLeader(const MSVehicle* veh, bool beyond, double latOffset = 0.);

    /// @brief discard all information
    virtual void clear();

    /* @brief returns sublanes occupied by veh
     * @param[in] veh The vehicle to check
     * @param[in] latOffset The offset value to add to the vehicle position
     * @param[out] rightmost The rightmost sublane occupied by veh
     * @param[out] leftmost The rightmost sublane occupied by veh
     */
    void getSubLanes(const MSVehicle* veh, double latOffset, int& rightmost, int& leftmost) const;

    /* @brief returns the sublane boundaries of the ith sublane
     * @param[in] sublane The sublane to check
     * @param[in] latOffset The offset value to add to the result
     * @param[out] rightSide The right border of the given sublane
     * @param[out] leftSide The left border of the given sublane
     */
    void getSublaneBorders(int sublane, double latOffset, double& rightSide, double& leftSide) const;

    /// @brief return the vehicle for the given sublane
    const MSVehicle* operator[](int sublane) const;

    int numSublanes() const {
        return (int)myVehicles.size();
    }

    int numFreeSublanes() const {
        return myFreeSublanes;
    }

    bool hasVehicles() const {
        return myHasVehicles;
    }

    const std::vector<const MSVehicle*>& getVehicles() const {
        return myVehicles;
    }

    int getSublaneOffset() const {
        return myOffset;
    }

    /// @brief set number of sublanes by which to shift positions
    void setSublaneOffset(int offset);

    /// @brief whether a stopped vehicle is leader
    bool hasStoppedVehicle() const;

    /// @brief whether the given vehicle is part of this leaderInfo
    bool hasVehicle(const MSVehicle* veh) const;

    /// @brief remove vehicles that are driving in the opposite direction (fully or partially) on the given lane
    void removeOpposite(const MSLane* lane);

    /// @brief print a debugging representation
    virtual std::string toString() const;

protected:

    /// @brief the width of the lane to which this instance applies
    // @note: not const to simplify assignment
    double myWidth;

    /// @brief an extra offset for shifting the interpretation of sublane borders (default [0,myWidth])
    int myOffset;

    std::vector<const MSVehicle*> myVehicles;

    /// @brief the number of free sublanes
    // if an ego vehicle is given in the constructor, the number of free
    // sublanes of those covered by ego
    int myFreeSublanes;

    /// @brief borders of the ego vehicle for filtering of free sublanes
    int egoRightMost;
    int egoLeftMost;

    bool myHasVehicles;

};


/// @brief saves leader/follower vehicles and their distances relative to an ego vehicle
class MSLeaderDistanceInfo : public MSLeaderInfo {
public:
    /// Constructor
    MSLeaderDistanceInfo(const double laneWidth, const MSVehicle* ego, const double latOffset);

    /// @brief Construct for the non-sublane-case
    MSLeaderDistanceInfo(const CLeaderDist& cLeaderDist, const double laneWidth);

    /// Destructor
    virtual ~MSLeaderDistanceInfo();

    /* @brief adds this vehicle as a leader in the appropriate sublanes
     * @param[in] veh The vehicle to add
     * @param[in] gap The gap between the egoFront+minGap to the back of veh
     *   or from the back of ego to the front+minGap of veh
     * @param[in] latOffset The lateral offset that must be added to the position of veh
     * @param[in] sublane The single sublane to which this leader shall be checked (-1 means: check for all)
     * @return The number of free sublanes
     */
    virtual int addLeader(const MSVehicle* veh, double gap, double latOffset = 0, int sublane = -1);

    virtual int addLeader(const MSVehicle* veh, bool beyond, double latOffset = 0) {
        UNUSED_PARAMETER(veh);
        UNUSED_PARAMETER(beyond);
        UNUSED_PARAMETER(latOffset);
        throw ProcessError(TL("Method not supported"));
    }

    /// @brief updatd empty sublanes with vehicles and gaps from other
    virtual void addLeaders(MSLeaderDistanceInfo& other);

    /// @brief discard all information
    virtual void clear();

    /// @brief return the vehicle and its distance for the given sublane
    CLeaderDist operator[](int sublane) const;

    /// @brief print a debugging representation
    virtual std::string toString() const;

    const std::vector<double>& getDistances() const {
        return myDistances;
    }

    /// @brief subtract vehicle length from all gaps if the leader vehicle is driving in the opposite direction
    void fixOppositeGaps(bool isFollower);

    /// @brief add given value to all gaps
    void patchGaps(double amount);

    /// @brief return vehicle with the smalles gap
    CLeaderDist getClosest() const;

    void moveSamePosTo(const MSVehicle* ego, MSLeaderDistanceInfo& other);

    /// @brief return minimum distance to a stopped vehicle or max double
    double getMinDistToStopped() const;


protected:

    std::vector<double> myDistances;

};


/* @brief saves follower vehicles and their distances as well as their required gap relative to an ego vehicle
 * when adding new followers, the one with the largest required gap is recored
 * (rather than the one with the smallest gap) */
class MSCriticalFollowerDistanceInfo : public MSLeaderDistanceInfo {
public:
    /// Constructor
    MSCriticalFollowerDistanceInfo(const double laneWidth, const MSVehicle* ego, const double latOffset, const bool haveOppositeLeaders = false);

    /// Destructor
    virtual ~MSCriticalFollowerDistanceInfo();

    /* @brief adds this vehicle as a follower in the appropriate sublanes
     * @param[in] veh The vehicle to add
     * @param[in] ego The vehicle which is being followed
     * @param[in] gap The distance from the back of ego to the follower
     * @param[in] latOffset The lateral offset that must be added to the position of veh
     * @param[in] sublane The single sublane to which this leader shall be checked (-1 means: check for all)
     * @return The number of free sublanes
     */
    int addFollower(const MSVehicle* veh, const MSVehicle* ego, double gap, double latOffset = 0, int sublane = -1);

    virtual int addLeader(const MSVehicle* veh, double gap, double latOffset = 0, int sublane = -1) {
        UNUSED_PARAMETER(veh);
        UNUSED_PARAMETER(gap);
        UNUSED_PARAMETER(latOffset);
        UNUSED_PARAMETER(sublane);
        throw ProcessError(TL("Method not supported"));
    }

    virtual int addLeader(const MSVehicle* veh, bool beyond, double latOffset = 0) {
        UNUSED_PARAMETER(veh);
        UNUSED_PARAMETER(beyond);
        UNUSED_PARAMETER(latOffset);
        throw ProcessError(TL("Method not supported"));
    }

    /// @brief discard all information
    void clear();

    /// @brief print a debugging representation
    std::string toString() const;

protected:

    // @brief the differences between requriedGap and actual gap for each of the followers
    std::vector<double> myMissingGaps;

    // @brief whether this Info objects tracks leaders instead of followers
    bool myHaveOppositeLeaders;

};
