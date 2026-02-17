/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2001-2026 German Aerospace Center (DLR) and others.
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
/// @file    MSNet.h
/// @author  Christian Roessel
/// @author  Jakob Erdmann
/// @author  Daniel Krajzewicz
/// @author  Thimor Bohn
/// @author  Eric Nicolay
/// @author  Clemens Honomichl
/// @author  Michael Behrisch
/// @author  Leonhard Luecken
/// @date    Mon, 12 Mar 2001
///
// The simulated network and simulation performer
/****************************************************************************/
#pragma once
#include <config.h>

#include <typeinfo>
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>
#include <iomanip>
#include <memory>
#include <utils/common/SUMOTime.h>
#include <utils/common/UtilExceptions.h>
#include <utils/common/NamedObjectCont.h>
#include <utils/common/NamedRTree.h>
#include <utils/router/SUMOAbstractRouter.h>
#include <mesosim/MESegment.h>
#include "MSRouterDefs.h"
#include "MSJunction.h"


// ===========================================================================
// class declarations
// ===========================================================================
class MSEdge;
class MSEdgeControl;
class MSEventControl;
class MSVehicleControl;
class MSJunctionControl;
class MSInsertionControl;
class SUMORouteLoaderControl;
class MSTransportableControl;
class MSTransportable;
class MSVehicle;
class MSRoute;
class MSLane;
class MSTLLogicControl;
class MSTrafficLightLogic;
class MSDetectorControl;
class ShapeContainer;
class MSDynamicShapeUpdater;
class PolygonDynamics;
class MSEdgeWeightsStorage;
class SUMOVehicle;
class SUMOTrafficObject;
class MSTractionSubstation;
class MSStoppingPlace;
template<class E, class L, class N, class V>
class IntermodalRouter;
template<class E, class L, class N, class V>
class PedestrianRouter;
class OptionsCont;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSNet
 * @brief The simulated network and simulation perfomer
 */
class MSNet : public Parameterised {
public:
    /** @enum SimulationState
     * @brief Possible states of a simulation - running or stopped with different reasons
     */
    enum SimulationState {
        /// @brief The simulation is loading
        SIMSTATE_LOADING,
        /// @brief The simulation is running
        SIMSTATE_RUNNING,
        /// @brief The final simulation step has been performed
        SIMSTATE_END_STEP_REACHED,
        /// @brief The simulation does not contain further vehicles
        SIMSTATE_NO_FURTHER_VEHICLES,
        /// @brief The connection to a client was closed by the client
        SIMSTATE_CONNECTION_CLOSED,
        /// @brief An error occurred during the simulation step
        SIMSTATE_ERROR_IN_SIM,
        /// @brief An external interrupt occurred
        SIMSTATE_INTERRUPTED,
        /// @brief The simulation had too many teleports
        SIMSTATE_TOO_MANY_TELEPORTS
    };

    /// @brief collision tracking
    struct Collision {
        std::string victim;
        std::string colliderType;
        std::string victimType;
        double colliderSpeed;
        double victimSpeed;
        Position colliderFront;
        Position victimFront;
        Position colliderBack;
        Position victimBack;
        std::string type;
        const MSLane* lane;
        double pos;
        SUMOTime time;
        SUMOTime continuationTime;
    };

    typedef std::map<std::string, std::vector<Collision> > CollisionMap;
    typedef std::map<const MSEdge*, RouterProhibition> Prohibitions;

public:
    /** @brief Returns the pointer to the unique instance of MSNet (singleton).
     * @return Pointer to the unique MSNet-instance
     * @exception ProcessError If a network was not yet constructed
     */
    static MSNet* getInstance();

    /**
     * @brief Returns whether this is a GUI Net
     */
    virtual bool isGUINet() const {
        return false;
    }

    /// @brief Place for static initializations of simulation components (called after successful net build)
    static void initStatic();

    /// @brief Place for static initializations of simulation components (called after successful net build)
    static void cleanupStatic();


    /** @brief Returns whether the network was already constructed
    * @return whether the network was already constructed
    */
    static bool hasInstance() {
        return myInstance != nullptr;
    }


    /** @brief Constructor
     *
     * This constructor builds a net of which only some basic structures are initialised.
     * It prepares the network for being filled while loading.
     * As soon as all edge/junction/traffic lights and other containers are build, they
     *  must be initialised using "closeBuilding".
     * @param[in] vc The vehicle control to use
     * @param[in] beginOfTimestepEvents The event control to use for simulation step begin events
     * @param[in] endOfTimestepEvents The event control to use for simulation step end events
     * @param[in] insertionEvents The event control to use for insertion events
     * @param[in] shapeCont The shape container to use
     * @exception ProcessError If a network was already constructed
     * @see closeBuilding
     */
    MSNet(MSVehicleControl* vc, MSEventControl* beginOfTimestepEvents,
          MSEventControl* endOfTimestepEvents,
          MSEventControl* insertionEvents,
          ShapeContainer* shapeCont = nullptr);


    /// @brief Destructor
    virtual ~MSNet();


    /** @brief Closes the network's building process
     *
     * Assigns the structures built while loading to this network.
     * @param[in] oc The options to use
     * @param[in] edges The control of edges which belong to this network
     * @param[in] junctions The control of junctions which belong to this network
     * @param[in] routeLoaders The route loaders used
     * @param[in] tlc The control of traffic lights which belong to this network
     * @param[in] stateDumpTimes List of time steps at which state shall be written
     * @param[in] stateDumpFiles Filenames for states
     * @param[in] hasInternalLinks Whether the network actually contains internal links
     * @param[in] junctionHigherSpeeds Whether the network was built with higher junction speeds
     * @param[in] version The network version
     * @todo Try to move all this to the constructor?
     */
    void closeBuilding(const OptionsCont& oc, MSEdgeControl* edges, MSJunctionControl* junctions,
                       SUMORouteLoaderControl* routeLoaders, MSTLLogicControl* tlc,
                       std::vector<SUMOTime> stateDumpTimes, std::vector<std::string> stateDumpFiles,
                       bool hasInternalLinks,
                       bool junctionHigherSpeeds,
                       const MMVersion& version);


    /** @brief Returns whether the network has specific vehicle class permissions
     * @return whether permissions are present
     */
    bool hasPermissions() const {
        return myHavePermissions;
    }


    /// @brief Labels the network to contain vehicle class permissions
    void setPermissionsFound() {
        myHavePermissions = true;
    }


    /** @brief Adds a restriction for an edge type
     * @param[in] id The id of the type
     * @param[in] svc The vehicle class the restriction refers to
     * @param[in] speed The restricted speed
     */
    void addRestriction(const std::string& id, const SUMOVehicleClass svc, const double speed);


    /** @brief Returns the restrictions for an edge type
     * If no restrictions are present, 0 is returned.
     * @param[in] id The id of the type
     * @return The mapping of vehicle classes to maximum speeds
     */
    const std::map<SUMOVehicleClass, double>* getRestrictions(const std::string& id) const;

    /// @brief retriefe edge type specific routing preference
    double getPreference(const std::string& routingType, const SUMOVTypeParameter& pars) const;

    /// @brief add edge type specific routing preference
    void addPreference(const std::string& routingType, SUMOVehicleClass svc, double prio);
    /// @brief add edge type specific routing preference
    void addPreference(const std::string& routingType, std::string vType, double prio);

    /** @brief Adds edge type specific meso parameters
     * @param[in] id The id of the type
     * @param[in] edgeType The parameter object
     */
    void addMesoType(const std::string& typeID, const MESegment::MesoEdgeType& edgeType);

    /** @brief Returns edge type specific meso parameters
     * if no type specific parameters have been loaded, default values are returned
     */
    const MESegment::MesoEdgeType& getMesoType(const std::string& typeID);

    /** @brief Clears all dictionaries
     * @todo Try to move all this to the destructor
     */
    static void clearAll();

    /// @brief return whether the given flow is known
    bool hasFlow(const std::string& id) const;

    /** @brief Simulates from timestep start to stop
     * @param[in] start The begin time step of the simulation
     * @param[in] stop The end time step of the simulation
     * @return Returns always 0
     * @todo Recheck return value
     * @todo What exceptions may occure?
     */
    SimulationState simulate(SUMOTime start, SUMOTime stop);


    /** @brief Performs a single simulation step
     * @todo Which exceptions may occur?
     */
    void simulationStep(const bool onlyMove = false);

    /** @brief loads routes for the next few steps */
    void loadRoutes();


    /** @brief Writes performance output and running vehicle stats
     *
     * @param[in] start The step the simulation was started with
     */
    const std::string generateStatistics(const SUMOTime start, const long now);

    /// @brief write collision output to (xml) file
    void writeCollisions() const;

    /// @brief write statistic output to (xml) file
    void writeStatistics(const SUMOTime start, const long now) const;

    /// @brief write summary-output to (xml) file
    void writeSummaryOutput(bool finalStep = false);

    /** @brief Closes the simulation (all files, connections, etc.)
     *
     * Writes also performance output
     *
     * @param[in] start The step the simulation was started with
     */
    void closeSimulation(SUMOTime start, const std::string& reason = "");


    /** @brief This method returns the current simulation state. It should not modify status.
     * @param[in] stopTime The time the simulation shall stop at
     * @return The current simulation state
     * @see SimulationState
     */
    SimulationState simulationState(SUMOTime stopTime) const;


    /** @brief Called after a simulation step, this method adapts the current simulation state if necessary
     * @param[in] state The current simulation state
     * @return The new simulation state
     * @see SimulationState
     */
    SimulationState adaptToState(const SimulationState state, const bool isLibsumo = false) const;


    /** @brief Returns the message to show if a certain state occurs
     * @return Readable description of the state
     */
    static std::string getStateMessage(SimulationState state);


    /** @brief Returns the current simulation step
     * @return the current simulation step
     */
    inline SUMOTime getCurrentTimeStep() const {
        return myStep;
    }


    /** @brief Sets the current simulation step (used by state loading)
     * @param step the current simulation step
     */
    inline void setCurrentTimeStep(const SUMOTime step) {
        myStep = step;
    }


    /** @brief Resets events when quick-loading state
     * @param step The new simulation step
     */
    void clearState(const SUMOTime step, bool quickReload = false);

    /** @brief Write netstate, summary and detector output
     * @todo Which exceptions may occur?
     */
    void writeOutput();


    /** @brief Returns whether duration shall be logged
     * @return Whether duration shall be logged
     */
    bool logSimulationDuration() const;



    /// @name Output during the simulation
    //@{

    /** @brief Prints the current step number
     *
     * Called on the begin of a simulation step
     */
    void preSimStepOutput() const;


    /** @brief Prints the statistics of the step at its end
     *
     * Called on the end of a simulation step
     */
    void postSimStepOutput() const;
    //}



    /// @name Retrieval of references to substructures
    /// @{

    /** @brief Returns the vehicle control
     * @return The vehicle control
     * @see MSVehicleControl
     * @see myVehicleControl
     */
    MSVehicleControl& getVehicleControl() {
        return *myVehicleControl;
    }


    /** @brief Returns the person control
     *
     * If the person control does not exist, yet, it is created.
     *
     * @return The person control
     * @see MSPersonControl
     * @see myPersonControl
     */
    virtual MSTransportableControl& getPersonControl();

    /** @brief Returns whether persons are simulated
     */
    bool hasPersons() const {
        return myPersonControl != nullptr;
    }

    /** @brief Returns the container control
     *
     * If the container control does not exist, yet, it is created.
     *
     * @return The container control
     * @see MSContainerControl
     * @see myContainerControl
     */
    virtual MSTransportableControl& getContainerControl();

    /** @brief Returns whether containers are simulated
    */
    bool hasContainers() const {
        return myContainerControl != nullptr;
    }


    /** @brief Returns the edge control
     * @return The edge control
     * @see MSEdgeControl
     * @see myEdges
     */
    MSEdgeControl& getEdgeControl() {
        return *myEdges;
    }


    /** @brief Returns the insertion control
     * @return The insertion control
     * @see MSInsertionControl
     * @see myInserter
     */
    MSInsertionControl& getInsertionControl() {
        return *myInserter;
    }


    /** @brief Returns the detector control
     * @return The detector control
     * @see MSDetectorControl
     * @see myDetectorControl
     */
    MSDetectorControl& getDetectorControl() {
        return *myDetectorControl;
    }


    /** @brief Returns the tls logics control
     * @return The tls logics control
     * @see MSTLLogicControl
     * @see myLogics
     */
    MSTLLogicControl& getTLSControl() {
        return *myLogics;
    }


    /** @brief Returns the junctions control
     * @return The junctions control
     * @see MSJunctionControl
     * @see myJunctions
     */
    MSJunctionControl& getJunctionControl() {
        return *myJunctions;
    }


    /** @brief Returns the event control for events executed at the begin of a time step
     * @return The control responsible for events that are executed at the begin of a time step
     * @see MSEventControl
     * @see myBeginOfTimestepEvents
     */
    MSEventControl* getBeginOfTimestepEvents() {
        return myBeginOfTimestepEvents;
    }


    /** @brief Returns the event control for events executed at the end of a time step
     * @return The control responsible for events that are executed at the end of a time step
     * @see MSEventControl
     * @see myEndOfTimestepEvents
     */
    MSEventControl* getEndOfTimestepEvents() {
        return myEndOfTimestepEvents;
    }


    /** @brief Returns the event control for insertion events
     * @return The control responsible for insertion events
     * @see MSEventControl
     * @see myInsertionEvents
     */
    MSEventControl* getInsertionEvents() {
        return myInsertionEvents;
    }


    /** @brief Returns the shapes container
     * @return The shapes container
     * @see ShapeContainer
     * @see myShapeContainer
     */
    ShapeContainer& getShapeContainer() {
        return *myShapeContainer;
    }

    /** @brief Returns the dynamic shapes updater
     * @see PolygonDynamics
     */
    MSDynamicShapeUpdater* getDynamicShapeUpdater() {
        return myDynamicShapeUpdater.get();
    }

    /** @brief Creates and returns a dynamic shapes updater
     * @see PolygonDynamics
     */
    MSDynamicShapeUpdater* makeDynamicShapeUpdater();

    /** @brief Returns the net's internal edge travel times/efforts container
     *
     * If the net does not have such a container, it is built.
     * @return The net's knowledge about edge weights
     */
    MSEdgeWeightsStorage& getWeightsStorage();
    /// @}

    /// @name Insertion and retrieval of stopping places
    /// @{

    /** @brief Adds a stopping place
     *
     * If another stop with the same id and category exists, false is returned.
     *  Otherwise, the stop is added to the internal stopping place container.
     *
     * This control gets responsible for deletion of the added stop.
     *
     * @param[in] stop The stop to add
     * @return Whether the stop could be added
     */
    bool addStoppingPlace(SumoXMLTag category, MSStoppingPlace* stop);


    /** @brief Adds a traction substation
     *
     * If another traction substation with the same id and category exists, false is returned.
     *  Otherwise, the traction substation is added to the internal substations container.
     *
     * @param[in] substation The traction substation to add
     * @return Whether the stop could be added
     */
    bool addTractionSubstation(MSTractionSubstation* substation);


    /** @brief Returns the named stopping place of the given category
     * @param[in] id The id of the stop to return.
     * @param[in] category The type of stop
     * @return The named stop, or 0 if no such stop exists
     */
    MSStoppingPlace* getStoppingPlace(const std::string& id, const SumoXMLTag category) const;

    /** @brief Returns the named stopping place by looking through all categories
     * @param[in] id The id of the stop to return.
     * @return The named stop, or 0 if no such stop exists
     */
    MSStoppingPlace* getStoppingPlace(const std::string& id) const;

    /** @brief Returns the stop of the given category close to the given position
     * @param[in] lane the lane of the stop to return.
     * @param[in] pos the position of the stop to return.
     * @param[in] category The type of stop
     * @return The stop id on the location, or "" if no such stop exists
     */
    std::string getStoppingPlaceID(const MSLane* lane, const double pos, const SumoXMLTag category) const;

    /* @brief returns all stopping places of that category with the same (non-empty) name attribute
     */
    const std::vector<MSStoppingPlace*>& getStoppingPlaceAlternatives(const std::string& name, SumoXMLTag category) const;
    /// @}

    const NamedObjectCont<MSStoppingPlace*>& getStoppingPlaces(SumoXMLTag category) const;

    /// @brief write charging station output
    void writeChargingStationOutput() const;

    /// @brief write rail signal block output
    void writeRailSignalBlocks() const;

    /// @brief creates a wrapper for the given logic (see GUINet)
    virtual void createTLWrapper(MSTrafficLightLogic*) {};

    /// @brief write the output generated by an overhead wire segment
    void writeOverheadWireSegmentOutput() const;

    /// @brief write electrical substation output
    void writeSubstationOutput() const;

    /// @brief return wheter the given logic (or rather its wrapper) is selected in the GUI
    virtual bool isSelected(const MSTrafficLightLogic*) const {
        return false;
    }
    /// @brief update view after simulation.loadState
    virtual void updateGUI() const { }

    /// @brief load state from file and return new time
    SUMOTime loadState(const std::string& fileName, const bool catchExceptions);

    /// @brief reset state to the beginning without reloading the network
    void quickReload();

    /// @name Notification about vehicle state changes
    /// @{

    /// @brief Definition of a vehicle state
    enum class VehicleState {
        /// @brief The vehicle was built, but has not yet departed
        BUILT,
        /// @brief The vehicle has departed (was inserted into the network)
        DEPARTED,
        /// @brief The vehicle started to teleport
        STARTING_TELEPORT,
        /// @brief The vehicle ended being teleported
        ENDING_TELEPORT,
        /// @brief The vehicle arrived at his destination (is deleted)
        ARRIVED,
        /// @brief The vehicle got a new route
        NEWROUTE,
        /// @brief The vehicles starts to park
        STARTING_PARKING,
        /// @brief The vehicle ends to park
        ENDING_PARKING,
        /// @brief The vehicles starts to stop
        STARTING_STOP,
        /// @brief The vehicle ends to stop
        ENDING_STOP,
        /// @brief The vehicle is involved in a collision
        COLLISION,
        /// @brief The vehicle had to brake harder than permitted
        EMERGENCYSTOP,
        /// @brief Vehicle maneuvering either entering or exiting a parking space
        MANEUVERING
    };


    /** @class VehicleStateListener
     * @brief Interface for objects listening to vehicle state changes
     */
    class VehicleStateListener {
    public:
        /// @brief Constructor
        VehicleStateListener() { }

        /// @brief Destructor
        virtual ~VehicleStateListener() { }

        /** @brief Called if a vehicle changes its state
         * @param[in] vehicle The vehicle which changed its state
         * @param[in] to The state the vehicle has changed to
         * @param[in] info Additional information on the state change
         */
        virtual void vehicleStateChanged(const SUMOVehicle* const vehicle, VehicleState to, const std::string& info = "") = 0;

    };


    /** @brief Adds a vehicle states listener
     * @param[in] listener The listener to add
     */
    void addVehicleStateListener(VehicleStateListener* listener);


    /** @brief Removes a vehicle states listener
     * @param[in] listener The listener to remove
     */
    void removeVehicleStateListener(VehicleStateListener* listener);


    /** @brief Informs all added listeners about a vehicle's state change
     * @param[in] vehicle The vehicle which changed its state
     * @param[in] to The state the vehicle has changed to
     * @param[in] info Information regarding the replacement
     * @see VehicleStateListener:vehicleStateChanged
     */
    void informVehicleStateListener(const SUMOVehicle* const vehicle, VehicleState to, const std::string& info = "");
    /// @}


    /// @name Notification about transportable state changes
    /// @{

    /// @brief Definition of a transportable state
    enum class TransportableState {
        /// @brief The transportable person has departed (was inserted into the network)
        PERSON_DEPARTED,
        /// @brief The transportable person arrived at his destination (is deleted)
        PERSON_ARRIVED,
        /// @brief The transportable container has departed (was inserted into the network)
        CONTAINER_DEPARTED,
        /// @brief The transportable container arrived at his destination (is deleted)
        CONTAINER_ARRIVED
    };


    /** @class TransportableStateListener
     * @brief Interface for objects listening to transportable state changes
     */
    class TransportableStateListener {
    public:
        /// @brief Constructor
        TransportableStateListener() { }

        /// @brief Destructor
        virtual ~TransportableStateListener() { }

        /** @brief Called if a transportable changes its state
         * @param[in] transportable The transportable which changed its state
         * @param[in] to The state the transportable has changed to
         * @param[in] info Additional information on the state change
         */
        virtual void transportableStateChanged(const MSTransportable* const transportable, TransportableState to, const std::string& info = "") = 0;

    };


    /** @brief Adds a transportable states listener
     * @param[in] listener The listener to add
     */
    void addTransportableStateListener(TransportableStateListener* listener);


    /** @brief Removes a transportable states listener
     * @param[in] listener The listener to remove
     */
    void removeTransportableStateListener(TransportableStateListener* listener);


    /** @brief Informs all added listeners about a transportable's state change
     * @param[in] transportable The transportable which changed its state
     * @param[in] to The state the transportable has changed to
     * @param[in] info Information regarding the replacement
     * @see TransportableStateListener:TransportableStateChanged
     */
    void informTransportableStateListener(const MSTransportable* const transportable, TransportableState to, const std::string& info = "");
    /// @}


    /// @brief register collision and return whether it was the first one involving these vehicles
    bool registerCollision(const SUMOTrafficObject* collider, const SUMOTrafficObject* victim, const std::string& collisionType, const MSLane* lane, double pos);

    const CollisionMap& getCollisions() const {
        return myCollisions;
    }


    /** @brief Returns the travel time to pass an edge
     * @param[in] e The edge for which the travel time to be passed shall be returned
     * @param[in] v The vehicle that is rerouted
     * @param[in] t The time for which the travel time shall be returned [s]
     * @return The travel time for an edge
     * @see DijkstraRouter_ByProxi
     */
    static double getTravelTime(const MSEdge* const e, const SUMOVehicle* const v, double t);


    /** @brief Returns the effort to pass an edge
     * @param[in] e The edge for which the effort to be passed shall be returned
     * @param[in] v The vehicle that is rerouted
     * @param[in] t The time for which the effort shall be returned [s]
     * @return The effort (abstract) for an edge
     * @see DijkstraRouter_ByProxi
     */
    static double getEffort(const MSEdge* const e, const SUMOVehicle* const v, double t);


    /* @brief get the router, initialize on first use
     * @param[in] prohibited The vector of forbidden edges (optional)
     */
    MSVehicleRouter& getRouterTT(int rngIndex, const Prohibitions& prohibited = {}) const;
    MSVehicleRouter& getRouterEffort(int rngIndex, const Prohibitions& prohibited = {}) const;
    MSPedestrianRouter& getPedestrianRouter(int rngIndex, const Prohibitions& prohibited = {}) const;
    MSTransportableRouter& getIntermodalRouter(int rngIndex, const int routingMode = 0, const Prohibitions& prohibited = {}) const;

    static void adaptIntermodalRouter(MSTransportableRouter& router);


    /// @brief return whether the network contains internal links
    bool hasInternalLinks() const {
        return myHasInternalLinks;
    }

    /// @brief return whether the network was built with higher junction speeds
    bool hasJunctionHigherSpeeds() const {
        return myJunctionHigherSpeeds;
    }

    /// @brief return whether the network contains elevation data
    bool hasElevation() const {
        return myHasElevation;
    }

    /// @brief return whether the network contains walkingareas and crossings
    bool hasPedestrianNetwork() const {
        return myHasPedestrianNetwork;

    }
    /// @brief return whether the network contains bidirectional rail edges
    bool hasBidiEdges() const {
        return myHasBidiEdges;
    }

    /// @brief return the network version
    MMVersion getNetworkVersion() const {
        return myVersion;
    }

    /// @brief return whether a warning regarding the given object shall be issued
    bool warnOnce(const std::string& typeAndID);

    void interrupt() {
        myAmInterrupted = true;
    }

    bool isInterrupted() const {
        return myAmInterrupted;
    }

    /// @brief gui may prevent final meanData reset to keep live data visible
    virtual bool skipFinalReset() const {
        return false;
    }

    MSMapMatcher* getMapMatcher() const;

    /// @brief find electrical substation by its id
    MSTractionSubstation* findTractionSubstation(const std::string& substationId);

    /// @brief return whether given electrical substation exists in the network
    bool existTractionSubstation(const std::string& substationId);

    /// @brief string constants for simstep stages
    static const std::string STAGE_EVENTS;
    static const std::string STAGE_MOVEMENTS;
    static const std::string STAGE_LANECHANGE;
    static const std::string STAGE_INSERTIONS;
    static const std::string STAGE_REMOTECONTROL;

protected:
    /// @brief check all lanes for elevation data
    bool checkElevation();

    /// @brief check all lanes for type walkingArea
    bool checkWalkingarea();

    /// @brief check wether bidirectional edges occur in the network
    bool checkBidiEdges();

    /// @brief remove collisions from the previous simulation step
    void removeOutdatedCollisions();

    /** @brief Performs the parts of the simulation step which happen after the move
     */
    void postMoveStep();

protected:
    /// @brief Unique instance of MSNet
    static MSNet* myInstance;

    /// @brief Route loader for dynamic loading of routes
    SUMORouteLoaderControl* myRouteLoaders;

    /// @brief Current time step
    SUMOTime myStep;

    /// @brief whether libsumo triggered a partial step (executeMove)
    bool myStepCompletionMissing = false;

    /// @brief Maximum number of teleports.
    int myMaxTeleports;

    /// @brief whether an interrupt occurred
    bool myAmInterrupted;



    /// @name Substructures
    /// @{

    /// @brief Controls vehicle building and deletion; @see MSVehicleControl
    MSVehicleControl* myVehicleControl;
    /// @brief Controls person building and deletion; @see MSTransportableControl
    MSTransportableControl* myPersonControl;
    /// @brief Controls container building and deletion; @see MSTransportableControl
    MSTransportableControl* myContainerControl;
    /// @brief Controls edges, performs vehicle movement; @see MSEdgeControl
    MSEdgeControl* myEdges;
    /// @brief Controls junctions, realizes right-of-way rules; @see MSJunctionControl
    MSJunctionControl* myJunctions;
    /// @brief Controls tls logics, realizes waiting on tls rules; @see MSJunctionControl
    MSTLLogicControl* myLogics;
    /// @brief Controls vehicle insertion; @see MSInsertionControl
    MSInsertionControl* myInserter;
    /// @brief Controls detectors; @see MSDetectorControl
    MSDetectorControl* myDetectorControl;
    /// @brief Controls events executed at the begin of a time step; @see MSEventControl
    MSEventControl* myBeginOfTimestepEvents;
    /// @brief Controls events executed at the end of a time step; @see MSEventControl
    MSEventControl* myEndOfTimestepEvents;
    /// @brief Controls insertion events; @see MSEventControl
    MSEventControl* myInsertionEvents;
    /// @brief A container for geometrical shapes; @see ShapeContainer
    ShapeContainer* myShapeContainer;
    /// @brief The net's knowledge about edge efforts/travel times; @see MSEdgeWeightsStorage
    MSEdgeWeightsStorage* myEdgeWeights;
    /// @}



    /// @name data needed for computing performance values
    /// @{

    /// @brief Information whether the simulation duration shall be logged
    bool myLogExecutionTime;

    /// @brief Information whether the number of the simulation step shall be logged
    bool myLogStepNumber;
    /// @brief Period between successive step-log outputs
    int myLogStepPeriod;

    /// @brief The last simulation step duration
    long myTraCIStepDuration = 0, mySimStepDuration = 0;

    /// @brief The overall simulation duration
    long mySimBeginMillis;

    /// @brief The overall time spent waiting for traci operations including
    long myTraCIMillis;

    /// @brief The overall number of vehicle movements
    long long int myVehiclesMoved;
    long long int myPersonsMoved;
    //}



    /// @name State output variables
    /// @{

    /// @brief Times at which a state shall be written
    std::vector<SUMOTime> myStateDumpTimes;
    /// @brief The names for the state files
    std::vector<std::string> myStateDumpFiles;
    /// @brief The names of the last K periodic state files (only only K shall be kept)
    std::vector<std::string> myPeriodicStateFiles;
    /// @brief The period for writing state
    SUMOTime myStateDumpPeriod;
    /// @brief name components for periodic state
    std::string myStateDumpPrefix;
    std::string myStateDumpSuffix;
    /// @}



    /// @brief Whether the network contains edges which not all vehicles may pass
    bool myHavePermissions;

    /// @brief The vehicle class specific speed restrictions
    std::map<std::string, std::map<SUMOVehicleClass, double> > myRestrictions;

    /// @brief Preferences for routing
    std::map<SUMOVehicleClass, std::map<std::string, double> > myVClassPreferences;
    std::map<std::string, std::map<std::string, double> > myVTypePreferences;

    /// @brief The edge type specific meso parameters
    std::map<std::string, MESegment::MesoEdgeType> myMesoEdgeTypes;

    /// @brief Whether the network contains internal links/lanes/edges
    bool myHasInternalLinks;

    /// @brief Whether the network was built with higher speed on junctions
    bool myJunctionHigherSpeeds;

    /// @brief Whether the network contains elevation data
    bool myHasElevation;

    /// @brief Whether the network contains pedestrian network elements
    bool myHasPedestrianNetwork;

    /// @brief Whether the network contains bidirectional rail edges
    bool myHasBidiEdges;

    /// @brief the network version
    MMVersion myVersion;

    /// @brief end of loaded edgeData
    SUMOTime myEdgeDataEndTime;

    /// @brief Dictionary of bus / container stops
    std::map<SumoXMLTag, NamedObjectCont<MSStoppingPlace*> > myStoppingPlaces;

    /// @brief dictionary of named stopping places
    std::map<SumoXMLTag, std::map<std::string, std::vector<MSStoppingPlace*> > > myNamedStoppingPlaces;

    /// @brief Dictionary of traction substations
    std::vector<MSTractionSubstation*> myTractionSubstations;

    /// @brief Container for vehicle state listener
    std::vector<VehicleStateListener*> myVehicleStateListeners;

    /// @brief Container for transportable state listener
    std::vector<TransportableStateListener*> myTransportableStateListeners;

    /// @brief collisions in the current time step
    CollisionMap myCollisions;

#ifdef HAVE_FOX
    /// @brief to avoid concurrent access to the state update function
    FXMutex myVehicleStateListenerMutex;

    /// @brief to avoid concurrent access to the state update function
    FXMutex myTransportableStateListenerMutex;
#endif
    static const NamedObjectCont<MSStoppingPlace*> myEmptyStoppingPlaceCont;
    static const std::vector<MSStoppingPlace*> myEmptyStoppingPlaceVector;

    /// @brief container to record warnings that shall only be issued once
    std::map<std::string, bool> myWarnedOnce;

    /* @brief The router instance for routing by trigger and by traci
     * @note MSDevice_Routing has its own instance since it uses a different weight function
     * @note we provide one member for every switchable router type
     * because the class structure makes it inconvenient to use a superclass
     */
    mutable std::map<int, MSVehicleRouter*> myRouterTT;
    mutable std::map<int, MSVehicleRouter*> myRouterEffort;
    mutable std::map<int, MSPedestrianRouter*> myPedestrianRouter;
    mutable std::map<int, MSTransportableRouter*> myIntermodalRouter;

    /// @brief An RTree structure holding lane IDs
    mutable std::pair<bool, NamedRTree> myLanesRTree;

    /// @brief Updater for dynamic shapes that are tracking traffic objects
    ///        (ensures removal of shape dynamics when the objects are removed)
    /// @see utils/shapes/PolygonDynamics
    std::unique_ptr<MSDynamicShapeUpdater> myDynamicShapeUpdater;

private:
    /// @brief Invalidated copy constructor.
    MSNet(const MSNet&);

    /// @brief Invalidated assignment operator.
    MSNet& operator=(const MSNet&);


};
