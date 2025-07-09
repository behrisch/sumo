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
/// @file    MSRoutingEngine.h
/// @author  Michael Behrisch
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @date    Tue, 04 Dec 2007
///
// A device that performs vehicle rerouting based on current edge speeds
/****************************************************************************/
#pragma once
#include <config.h>

#include <set>
#include <vector>
#include <map>
#include <thread>
#include <utils/common/SUMOTime.h>
#include <utils/common/WrappingCommand.h>
#include <utils/router/AStarRouter.h>
#include <microsim/MSRouterDefs.h>

#ifdef HAVE_FOX
#include <utils/foxtools/MFXWorkerThread.h>
#endif


// ===========================================================================
// class declarations
// ===========================================================================
class MSTransportable;
class SUMOSAXAttributes;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSRoutingEngine
 * @brief A device that performs vehicle rerouting based on current edge speeds
 *
 * The routing-device system consists of in-vehicle devices that perform a routing
 *  and a simulation-wide (static) methods for colecting edge weights.
 *
 * The edge weights container "myEdgeSpeeds" is pre-initialised as soon as one
 *  device is built and is kept updated via an event that adapts it to the current
 *  mean speed on the simulated network's edges.
 *
 * A device is assigned to a vehicle using the common explicit/probability - procedure.
 *
 * A device computes a new route for a vehicle as soon as the vehicle is inserted
 *  (within "enterLaneAtInsertion") - and, if the given period is larger than 0 - each
 *  x time steps where x is the period. This is triggered by an event that executes
 *  "wrappedRerouteCommandExecute".
 */
class MSRoutingEngine {
public:
    typedef std::map<const MSEdge*, double> Prohibitions;

    /// @brief intialize period edge weight update
    static void initWeightUpdate();

    /// @brief initialize the edge weights if not done before
    static void initEdgeWeights(SUMOVehicleClass svc, SUMOTime lastAdaption = -1);

    /// @brief returns whether any edge weight updates will take place
    static bool hasEdgeUpdates() {
        return myEdgeWeightSettingCommand != nullptr;
    }

    /// @brief Information when the last edge weight adaptation occurred
    static SUMOTime getLastAdaptation() {
        return myLastAdaptation;
    }

    /// @brief return the cached route or nullptr on miss
    static ConstMSRoutePtr getCachedRoute(const std::pair<const MSEdge*, const MSEdge*>& key);

    static void initRouter(SUMOVehicle* vehicle = nullptr);

    /// @brief initiate the rerouting, create router / thread pool on first use
    static void reroute(SUMOVehicle& vehicle, const SUMOTime currentTime, const std::string& info,
                        const bool onInit = false, const bool silent = false, const Prohibitions& prohibited = {});

    /// @brief initiate the person rerouting, create router / thread pool on first use
    static void reroute(MSTransportable& t, const SUMOTime currentTime, const std::string& info,
                        const bool onInit = false, const bool silent = false, const Prohibitions& prohibited = {});

    /// @brief adapt the known travel time for an edge
    static void setEdgeTravelTime(const MSEdge* const edge, const double travelTime);

    /// @brief deletes the router instance
    static void cleanup();

    /// @brief returns whether any routing actions take place
    static bool isEnabled() {
        return !myWithTaz && myAdaptationInterval >= 0;
    }

    /// @brief return the vehicle router instance
    static MSVehicleRouter& getRouterTT(const int rngIndex,
                                        SUMOVehicleClass svc,
                                        const Prohibitions& prohibited = {});

    /// @brief return the person router instance
    static MSTransportableRouter& getIntermodalRouterTT(const int rngIndex,
            const Prohibitions& prohibited = {});

    /** @brief Returns the effort to pass an edge
    *
    * This method is given to the used router in order to obtain the efforts
    *  to pass an edge from the internal edge weights container.
    *
    * The time is not used, here, as the current simulation state is
    *  used in an aggregated way.
    *
    * @param[in] e The edge for which the effort to be passed shall be returned
    * @param[in] v The vehicle that is rerouted
    * @param[in] t The time for which the effort shall be returned
    * @return The effort (time to pass in this case) for an edge
    * @see DijkstraRouter_ByProxi
    */
    static double getEffort(const MSEdge* const e, const SUMOVehicle* const v, double t);
    static double getEffortBike(const MSEdge* const e, const SUMOVehicle* const v, double t);
    static double getEffortExtra(const MSEdge* const e, const SUMOVehicle* const v, double t);
    static SUMOAbstractRouter<MSEdge, SUMOVehicle>::Operation myEffortFunc;

    /// @brief return current travel speed assumption
    static double getAssumedSpeed(const MSEdge* edge, const SUMOVehicle* veh);

    /// @brief whether taz-routing is enabled
    static bool withTaz() {
        return myWithTaz;
    }

    /// @brief record actual travel time for an edge
    static void addEdgeTravelTime(const MSEdge& edge, const SUMOTime travelTime);

    /** @brief Saves the state (i.e. recorded speeds)
     *
     * @param[in] out The OutputDevice to write the information into
     */
    static void saveState(OutputDevice& out);

    /** @brief Loads the state
     *
     * @param[in] attrs XML attributes describing the current state
     */
    static void loadState(const SUMOSAXAttributes& attrs);

#ifdef HAVE_FOX
    static void waitForAll();
#endif


private:
#ifdef HAVE_FOX
    /**
     * @class RoutingTask
     * @brief the routing task which mainly calls reroute of the vehicle
     */
    class RoutingTask : public MFXWorkerThread::Task {
    public:
        RoutingTask(SUMOVehicle& v, const SUMOTime time, const std::string& info,
                    const bool onInit, const bool silent, const Prohibitions& prohibited)
            : myVehicle(v), myTime(time), myInfo(info), myOnInit(onInit), mySilent(silent), myProhibited(prohibited) {}
        void run(MFXWorkerThread* context);
    private:
        SUMOVehicle& myVehicle;
        const SUMOTime myTime;
        const std::string myInfo;
        const bool myOnInit;
        const bool mySilent;
        const Prohibitions myProhibited;
    private:
        /// @brief Invalidated assignment operator.
        RoutingTask& operator=(const RoutingTask&) = delete;
    };

    /**
     * @class InitTask
     * @brief setup RNGs for each thread (with proper locking so we don't need
     * locking later */
    class InitTask : public MFXWorkerThread::Task {
    public:
        InitTask() {}
        void run(MFXWorkerThread* context);
    private:
        /// @brief Invalidated assignment operator.
        RoutingTask& operator=(const RoutingTask&) = delete;
    };
#endif

    /// @name Network state adaptation
    /// @{

    /** @brief Adapt edge efforts by the current edge states
     *
     * This method is called by the event handler at the end of a simulation
     *  step. The current edge weights are combined with the previously stored.
     *
     * @param[in] currentTime The current simulation time
     * @return The offset to the next call (always 1 in this case - edge weights are updated each time step)
     * @todo Describe how the weights are adapted
     * @see MSEventHandler
     * @see StaticCommand
     */
    static SUMOTime adaptEdgeEfforts(SUMOTime currentTime);

    static double patchSpeedForTurns(const MSEdge* edge, double currSpeed);
    /// @}

    /// @brief initialized edge speed storage into the given containers
    static void _initEdgeWeights(std::vector<double>& edgeSpeeds, std::vector<std::vector<double> >& pastEdgeSpeeds);

    /// @brief returns RNG associated with the current thread
    static SumoRNG* getThreadRNG();

private:
    /// @brief The weights adaptation/overwriting command
    static Command* myEdgeWeightSettingCommand;

    /// @brief Information which weight prior edge efforts have
    static double myAdaptationWeight;

    /// @brief At which time interval the edge weights get updated
    static SUMOTime myAdaptationInterval;

    /// @brief Information when the last edge weight adaptation occurred
    static SUMOTime myLastAdaptation;

    /// @brief The number of steps for averaging edge speeds (ring-buffer)
    static int myAdaptationSteps;

    /// @brief The current index in the pastEdgeSpeed ring-buffer
    static int myAdaptationStepsIndex;

    typedef std::pair<SUMOTime, int> TimeAndCount;

    /// @brief The container of edge speeds
    static std::vector<double> myEdgeSpeeds;
    static std::vector<double> myEdgeBikeSpeeds;

    /// @brief Sum of travel times experienced by equipped vehicles for each edge
    static std::vector<TimeAndCount> myEdgeTravelTimes;

    /// @brief The container of past edge speeds (when using a simple moving average)
    static std::vector<std::vector<double> > myPastEdgeSpeeds;
    static std::vector<std::vector<double> > myPastEdgeBikeSpeeds;

    /// @brief whether taz shall be used at initial rerouting
    static bool myWithTaz;

    /// @brief whether separate speeds for bicycles shall be tracked
    static bool myBikeSpeeds;

    /// @brief The router to use
    static MSRouterProvider* myRouterProvider;

    static std::map<std::thread::id, SumoRNG*> myThreadRNGs;
    static bool myHaveRoutingThreads;

    /// @brief The container of pre-calculated routes
    static std::map<std::pair<const MSEdge*, const MSEdge*>, ConstMSRoutePtr> myCachedRoutes;

    /// @brief Coefficient for factoring edge priority into routing weight
    static double myPriorityFactor;

    /// @brief Minimum priority for all edges
    static double myMinEdgePriority;
    /// @brief the difference between maximum and minimum priority for all edges
    static double myEdgePriorityRange;

#ifdef HAVE_FOX
    /// @brief Mutex for accessing the route cache
    static FXMutex myRouteCacheMutex;
#endif

private:
    /// @brief Invalidated copy constructor.
    MSRoutingEngine(const MSRoutingEngine&);

    /// @brief Invalidated assignment operator.
    MSRoutingEngine& operator=(const MSRoutingEngine&);


};
