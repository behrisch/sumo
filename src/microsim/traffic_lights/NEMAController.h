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
/// @file    NEMAController.h
/// @author  Tianxin Li
/// @author  Qichao Wang
/// @date    August 2020
///
// An actuated NEMA-phase-compliant traffic light logic
/****************************************************************************/
#pragma once
#include <config.h>

#include <utility>
#include <vector>
#include <bitset>
#include <map>
#include <set>
#include <microsim/MSEventControl.h>
#include <microsim/traffic_lights/MSTrafficLightLogic.h>
#include "MSSimpleTrafficLightLogic.h"
#include "microsim/output/MSE2Collector.h"
#include "MSPhaseDefinition.h"


// ===========================================================================
// class declarations
// ===========================================================================
class NLDetectorBuilder;
class MSE2Collector;
class NEMAPhase;
class PhaseTransitionLogic;

// ===========================================================================
// Enumeration
// ===========================================================================
enum class LightState {
    RedXfer,
    Red,
    Yellow,
    Green,
    GreenXfer,
    GreenRest,
};


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class NEMALogic
 * @brief A NEMA (adaptive) traffic light logic based on E2Detector
 */
class NEMALogic : public MSSimpleTrafficLightLogic {
public:

    typedef NEMAPhase* PhasePtr;

    typedef std::map<MSLane*, MSE2Collector*> LaneDetectorMap;

    typedef std::map<MSE2Collector*, MSLane*, ComparatorIdLess> DetectorLaneMap;

    // Small structure for storing two ring transitions and the average distance
    struct transitionInfo {
        PhaseTransitionLogic* p1;
        PhaseTransitionLogic* p2;
        float distance;
    };

    enum controllerType {
        Type170,
        TS2
    };

    /// @brief constant for storing the priority order for light heads. Iterates left to right and stops when finds a match.
    const std::string lightHeadPriority = "GgyuOs";

    typedef std::vector<transitionInfo> TransitionPairs;

    /** @brief Constructor
     * @param[in] tlcontrol The tls control responsible for this tls
     * @param[in] id This tls' id
     * @param[in] programID This tls' sub-id (program id)
     * @param[in] phases Definitions of the phases
     * @param[in] step The initial phase index
     * @param[in] delay The time to wait before the first switch
     * @param[in] parameter The parameter to use for tls set-up
     */
    NEMALogic(MSTLLogicControl& tlcontrol,
              const std::string& id, const std::string& programID,
              const SUMOTime offset,
              const MSSimpleTrafficLightLogic::Phases& phases,
              int step, SUMOTime delay,
              const std::map<std::string, std::string>& parameter,
              const std::string& basePath);


    /** @brief Initialises the tls with information about incoming lanes
     * @param[in] nb The detector builder
     * @exception ProcessError If something fails on initialisation
     */
    void init(NLDetectorBuilder& nb) override;

    /// @brief Destructor
    ~NEMALogic();

    /// @brief overrides the MSSimpleTrafficLightLogic trySwitch method
    SUMOTime trySwitch() override;

    /// @name Dynamic Information Retrieval
    /// @{

    /** @brief Returns myPhase, which doesn't correspond to a NEMA phase, but rather the composite light string
     * @return The current phase (NEMA controller really uses )
     */
    const MSPhaseDefinition& getCurrentPhaseDef() const override;
    /// @}

    void resetLastSwitch(SUMOTime t); 

    void activateProgram() override;
    void deactivateProgram() override;

    bool showDetectors() const {
        return myShowDetectors;
    }

    void setShowDetectors(bool show);

    /// @brief retrieve all detectors used by this program
    std::map<std::string, double> getDetectorStates() const override;

    /**
     * @brief extends the transitions vector with valid Transitions given the current traffic light state
     *
     * @param[out] transitions a reference to the TransitionPairs vector
     */
    void getNextPhases(TransitionPairs& transitions);

    /**
     * @brief Calculates the modulus a / b, normally used to calculate the cycle time between
     * two times. Usage example:  ModeCycle(t1 - t2, cycleLength)
     *
     * @param a time 1
     * @param b time 2
     * @return SUMOTime
     */
    SUMOTime ModeCycle(SUMOTime a, SUMOTime b);


    /**
     * @brief returns the IDs of the phase's controlled lanes.
     * Found by looking for the "G" in the light state string
     *
     * @param state the light state string
     * @return std::set<std::string>
     */
    void getLaneInfoFromNEMAState(std::string state, StringVector& laneIDs, IntVector& stateIndex);

    /**
     * @brief Set the max green of all phases.
     *
     * @param newMaxGreens a vector of new max green times. Must be length 8
     */
    void setNewMaxGreens(std::vector<double> newMaxGreens);

    /**
     * @brief Set the new splits of all phases
     *
     * @param newSplits a vector of new splits. Must be length 8
     */
    void setNewSplits(std::vector<double> newSplits);

    /**
     * @brief set the new cycle length for the controller
     *
     * @param newCycleLength
     */
    void setNewCycleLength(double newCycleLength);

    /**
     * @brief Set the new offset for the controller
     *
     * @param newOffset
     */
    void setNewOffset(double newOffset);

    /**
     * @brief Get the current cycle length
     *
     * @return SUMOTime
     */
    SUMOTime getCurrentCycleLength() {
        return myCycleLength;
    }

    /// @brief try to set the given parameter. Parameters prefixed with 'NEMA.' control functionality
    void setParameter(const std::string& key, const std::string& value) override;

    /// @brief try to get the value of the given parameter. Parameters prefixed with 'NEMA.' control functionality
    const std::string getParameter(const std::string& key, const std::string defaultValue = "") const override;

    /// @brief Wrapper Function to Simplify Accessing Time
    inline SUMOTime getCurrentTime(void) const {
        return simTime;
    }

    // /// @brief Wrapper Function to Simplify Accessing Offset Cycle Time
    // inline SUMOTime getCurrentOffsetTime(void) const {return simTime - cycleRefPoint - offset; };

    /// @brief override Function to Simplify Accessing Offset Cycle Time
    inline SUMOTime getTimeInCycle() const {
        return (simTime - cycleRefPoint - offset) % myCycleLength;
    }


    /// @brief set the active phase
    void setActivePhase(PhasePtr phase);

    /**
     * @brief Get the Active Phase object for a specified ring
     *
     * @param ringNum
     * @return PhasePtr
     */
    inline PhasePtr getActivePhase(int ringNum) {
        return myActivePhaseObjs[ringNum];
    }

    /**
     * @brief get all phases for a given ring
     *
     * @param ringNum
     * @return std::vector<PhasePtr>
     */
    std::vector<PhasePtr> getPhasesByRing(int ringNum);

    /**
     * @brief get the phase object matching the phaseNum
     * If ringNum is passed, it will only search for the phase in the given ring
     *
     * @param phaseNum an integer corresponding to the phase
     * @param ringNum the ring to search for the phase in. Defaults to -1, meaning both rings will be searched
     * @return PhasePtr (NEMAPhase*)
     */
    PhasePtr getPhaseObj(int phaseNum, int ringNum = -1);

    /**
     * @brief get a vector of all phase objects
     *
     * @return std::vector<PhasePtr>
     */
    inline std::vector<PhasePtr> getPhaseObjs(void) {
        return myPhaseObjs;
    }

    /**
     * @brief return the ring distance between two phases
     *
     * @param p1 phase 1
     * @param p2 phase 2
     * @param ringNum the ring on which to measure the phase distance
     * @return int
     */
    int measureRingDistance(int p1, int p2, int ringNum);

    /**
     * @brief checks if the controller is of type170
     *
     * @return true if myControllerType == Type170
     * @return false
     */
    inline bool isType170(void) const {
        return myControllerType == Type170;
    }

    /**
     * @brief Get the opposite active phase
     *
     * @param p a pointer to the known phase
     * @return PhasePtr
     */
    PhasePtr getOtherPhase(PhasePtr p);

    /// @brief whether the controller is in coordinated mode or not
    bool coordinateMode;

    /**
     * @brief implement any pending traci changes
     * This function is called once per cycle
     */
    void implementTraciChanges(void);

    /// @brief a store of the coordinated phase objects. Only used meaningfully when the controller is
    /// in coordinated mode
    PhasePtr coordinatePhaseObjs[2];

protected:

    /// @brief flag to keep track of whether a timing change has been requested via traci
    bool queuedTraciChanges;

    /// @brief the controller's offset
    SUMOTime offset;
    /// @brief the next offset to implement
    SUMOTime myNextOffset;

    /// @brief the coordinated cycle length
    SUMOTime myCycleLength;
    /// @brief the next cycle length (set by traci)
    SUMOTime myNextCycleLength;

    /// @brief stores the simulation time to make it easily accessible
    SUMOTime simTime = 0;

    /// @brief stores controllers # of rings
    int myNumberRings;

    /// @brief stores the length of phase string for the controller "GGrrrrs" = 6. Must be the same length for all phases
    int myPhaseStrLen = -1;

    /// @brief Set the simTime
    inline void setCurrentTime(void) {
        simTime = MSNet::getInstance()->getCurrentTimeStep();
    }

    /// @brief variable to store the active phases
    PhasePtr myActivePhaseObjs[2] = { nullptr, nullptr };

    /// @brief a vector that stores a pointer to the instantiated NEMAPhase objects
    std::vector<PhasePtr > myPhaseObjs;

    /// @brief an array to store the phases located at a barrier for each ring
    PhasePtr defaultBarrierPhases[2][2];

    /**
     * Construct Timing and Phase Defs
     * @brief constructs phase using the configuration file
     * @param barriers a string of barrier phases ("4,8")
     * @param coordinates a string of coordinated phases ("2,6")
     * @param ring1 a string of phases in ring 1 ("1,2,3,4")
     * @param ring2 a string of phases in ring 2 ("5,6,7,8")
     */
    void constructTimingAndPhaseDefs(std::string& barriers, std::string& coordinates,
                                     std::string& ring1, std::string& ring2);

    /** @brief iterates over the two active phases (myActivePhaseObjs) and merges the two active phases
     * @return std::string the light string to implement (GGGrrrGGGrrr)
     */
    std::string composeLightString();

    /** @brief check if a vector contains an element
     * @param v the vector of phase numbers
     * @param phaseNum the phase number
     * @return bool
     */
    bool vectorContainsPhase(std::vector<int> v, int phaseNum);

    // create a small datatype for mapping detector to phase index
    // This is the one copied from MSActuatedTrafficLightLogic
    // not used in our controller, but it is here for meeting the SUMO default traffic logic light check
    // this one and related could be removed with extra efforts
    struct DetectorInfo {
        DetectorInfo(MSE2Collector* _det, int numPhases) :
            det(_det),
            servedPhase(numPhases, false)
        {}
        MSE2Collector* det;
        SUMOTime lastGreenTime = 0;
        std::vector<bool> servedPhase;
    };
    typedef std::vector<std::vector<DetectorInfo*>> detectorMap;
    detectorMap myDetectorForPhase;
    /// @brief storing the detector info in a vector
    std::vector<DetectorInfo> myDetectorInfoVector;


    /// @brief return whether there is a major link from the given lane in the given phase
    bool hasMajor(const std::string& state, const LaneVector& lanes) const;

    /**
     * @brief converts a comma separated string into a integer vector
     * "1,2,3,4" -> {1,2,3,4}
     *
     * @param s the string of comma separated integers
     * @return std::vector<int>
     */
    std::vector<int> readParaFromString(std::string s);

    /**
     * @brief decide whether the detector is for left turn lane
     * if it is, use the detector length for left turn lane
     *
     * @param lane a pointer to the lane
     * @return whether a lane is a left turn or not
     */
    bool isLeftTurnLane(const MSLane* const lane) const;

    /// @brief convert a string to an integer
    int string2int(std::string s);

    /// @brief A map from lanes to detectors
    LaneDetectorMap myLaneDetectorMap;

    /// @brief A map from lanes names to phases
    std::map<std::string, int> myLanePhaseMap;

    /// @brief A map from detectors to lanes
    DetectorLaneMap myDetectorLaneMap;

    /// @brief store the generated detector length
    double myDetectorLength;

    /// @brief store the left turn lane detestor length
    double myDetectorLengthLeftTurnLane;

    /// Whether the detectors shall be shown in the GUI
    bool myShowDetectors;

    /// The output file for generated detectors
    std::string myFile;

    /// The frequency for aggregating detector output
    SUMOTime myFreq;

    /// Whether detector output separates by vType
    std::string myVehicleTypes;

    /*
    {
        {3,4,1,2},
        {7,8,5,6}
    }
    */
    std::vector<std::vector<int>> rings;

    /*
    {
        {1 : PhaseDetectorInfo{
                detectors: {det1, det2, ...},
                crossPhaseDetector: 6
            },
        },
        {2 : ...
    }
    */
    // std::map<int, PhaseDetectorInfo> phase2DetectorMap;
    std::map<int, std::vector<std::string>> phase2ControllerLanesMap;

    bool fixForceOff;
    SUMOTime cycleRefPoint;// missing update
    bool whetherOutputState;
    bool ignoreErrors;

    /**
     * @brief return the default transition for t give its and the ot's state
     *
     * @param t the target phase
     * @param ot the other active phase
     * @return PhaseTransitionLogic* the transition logic describing this transition
     */
    PhaseTransitionLogic* getDefaultTransition(PhaseTransitionLogic* t, PhaseTransitionLogic* ot);

    // Store the cabinet type
    controllerType myControllerType;

    /**
     * @brief parse the controllerType from the tllogic description
     *
     * @param inputType
     * @return controllerType
     */
    controllerType parseControllerType(std::string inputType);

    /// @brief virtual phase that holds the current state
    MSPhaseDefinition myPhase;

    /**
     * @brief throw an InvalidArgument error if the param_name is not set
     *
     * @param param_variable the value of param_name
     * @param param_name  the name of the parameter
     */
    void error_handle_not_set(std::string param_variable, std::string param_name);

    /**
     * @brief validates the NEMA timing.
     * Writes warnings if ignoreError set to true else throws ProcessError
     *
     */
    void validate_timing();

    /**
     * @brief calculate the forceOffs for a TS2 style offset
     * From https://ops.fhwa.dot.gov/publications/fhwahop08024/chapter6.htm#6.3
     *
     */
    void calculateForceOffsTS2();
    /**
     * @brief calculate the forceOffs for a Type 170 style offset
     * From https://ops.fhwa.dot.gov/publications/fhwahop08024/chapter6.htm#6.3
     */
    void calculateForceOffs170();

    /// @brief directs the code to the correct force off function accorifing to its cabinet type
    void calculateForceOffs() {
        switch (myControllerType) {
            case Type170:
                return calculateForceOffs170();
            case TS2:
                return calculateForceOffsTS2();
            default:
                return calculateForceOffs170();
        }
    }


    /// @brief calculate the initial phases for the TS2 style controller to start in
    void calculateInitialPhasesTS2();
    /// @brief calculate the initial phases for Type 170
    void calculateInitialPhases170();
    /// @brief directs the controller to the correct calculate phases function
    void calculateInitialPhases() {
        switch (myControllerType) {
            case Type170:
                return calculateInitialPhases170();
            case TS2:
                return calculateInitialPhasesTS2();
            default:
                // Default to Type170
                return calculateInitialPhases170();
        }
    }
};


/**
 * @class NEMAPhase
 * @brief One phase in the NEMAController
 *
 * This represents one phase and all its parameters in a NEMA traffic light
 * The phase ultimately controls its transition to the next phase,
 * and is resbonisble for determining the valid transitions given its current state
 */
class NEMAPhase {
public:
    /// @brief Typedef for commonly used phase pointer
    typedef NEMAPhase* PhasePtr;

    /// @struct PhaseDetectorInfo
    /// @brief stores information about the phase's detector(s)
    struct PhaseDetectorInfo {
        PhaseDetectorInfo() :
            detectors(),
            cpdTarget(),
            cpdSource(),
            detectActive(),
            latching()
        {}
        PhaseDetectorInfo(bool latching, PhasePtr cpdSource, PhasePtr cpdTarget) :
            cpdTarget(cpdTarget),
            cpdSource(cpdSource),
            detectActive(false),
            latching(latching)
        {}
        ///@brief a vector of pointers to the phase's detectors
        std::vector<MSE2Collector*> detectors;
        /// @brief the cross-phase switching target for myself (6 if 6 should check 1 if 6 is green and I am phase 1)
        PhasePtr cpdTarget;
        /// @brief the cross-phase switching source for myself  (1 if 6 should check 1 if 6 is green and I am phase 6)
        PhasePtr cpdSource;
        /// @brief where any of my detectors are active or not
        bool detectActive;
        /// @brief whether the detectors are latching or not
        bool latching;
    };

    // create a PhaseDetectorInfo type
    typedef PhaseDetectorInfo PhaseDetectorInfo;

    /**
     * @brief Construct a new NEMAPhase object
     *
     * @param phaseName the "name" of the phase as an integer
     * @param isBarrier if the phase is located at a barrier or not
     * @param isGreenRest if it is a phase in which the traffic signal can green rest
     * @param isCoordinated if it is a coordinated phase
     * @param minRecall whether or not the phase has minimum recall or not
     * @param maxRecall whether or not the phase has maximum recall or not
     * @param fixForceOff if the phase has a force off or not
     * @param barrierNum the barrier to which the phase belongs (0 or 1)
     * @param ringNum the ring to which the phase belongs (0 or 1)
     * @param phaseStringInds the indexes of lanes that I control, ie. "srrrrGG" is {5, 6}
     * @param phase the MSPhaseDefinition base class
     */
    NEMAPhase(int phaseName,
              bool isBarrier,
              bool isGreenRest,
              bool isCoordinated,
              bool minRecall,
              bool maxRecall,
              bool fixForceOff,
              int barrierNum,
              int ringNum,
              IntVector phaseStringInds,
              MSPhaseDefinition* phase);

    /// @brief Destructor
    ~NEMAPhase();

    /// @brief gets the current light state
    inline LightState getCurrentState() const {
        return myLightState;
    }
    /// @brief returns a vector of the phases detectors
    inline std::vector<MSE2Collector*> getDetectors() const {
        return myDetectorInfo.detectors;
    }


    /// @brief sets the detectors for the phase
    inline void setDetectors(std::vector<MSE2Collector*> detectors) {
        myDetectorInfo.detectors = detectors;
    }

    /// @brief check if a transition is active
    inline bool isTransitionActive() const {
        return myLightState < LightState::Green;
    }

    // Build a Map of Valid Transitions and store the detector-based information
    /**
     * @brief initializes the object
     *
     * @param controller a pointer to the controller object
     * @param crossPhaseTarget the cross phase switching target
     * @param crossPhaseSource the cross phase switching source
     * @param latching whether the phase has latching detectors or not
     */
    void init(NEMALogic* controller, int crossPhaseTarget, int crossPhaseSource, bool latching);

    /**
     * @brief update is called on the active phases by the NEMAController at every time step
     *
     * @param controller a reference to the controller
     */
    void update(NEMALogic* controller);

    /**
     * @brief handles the transition out of a phase into the next (puts the phase through (G -> Y -> R) transition
     *
     * @param controller a reference to the NEMAController
     * @param nextPhases the next phases that the controller wants to transition to
     */
    void exit(NEMALogic* controller, PhaseTransitionLogic* nextPhases[2]);

    /**
     * @brief handles the transition into a green rest state
     *
     * @param controller a reference to the NEMAController
     * @param nextPhases the next phases that the controller wants to transition to
     */
    void handleGreenRestOrTransfer(NEMALogic* controller, PhaseTransitionLogic* nextPhases[2]);

    /**
     * @brief handles the transition into yellow
     *
     * @param controller a reference to the NEMAController
     */
    void enterYellow(NEMALogic* controller);

    /**
     * @brief handles the transition into a red xfer state, which is roughly the same as green rest
     *
     * @param controller a reference to the NEMAController
     */
    void handleRedXferOrNextPhase(NEMALogic* controller, PhaseTransitionLogic* nextPhases[2]);

    /// @brief simple method to check if there is a recall on the phase.
    inline bool hasRecall(void) {
        return minRecall || maxRecall;
    }

    /// @brief simple method to check if there is either a recall or an active detector
    inline bool callActive(void) {
        return minRecall || maxRecall || myDetectorInfo.detectActive;
    }

    /// @brief simple method to check if a detector is active
    inline bool detectActive(void) {
        return myDetectorInfo.detectActive;
    }

    /// @brief Check Detectors. Called on all phases at every step
    void checkMyDetectors(void);

    /// @brief Clear My Detectors. Called on all phases at every step
    void clearMyDetectors(void);

    // Need-to-know Phase Settings
    int phaseName;
    bool isAtBarrier;
    bool isGreenRest;
    int barrierNum;
    bool coordinatePhase;
    bool minRecall;
    bool maxRecall;
    bool fixForceOff;
    int ringNum;

    /// @brief store the last detect check for traci purposes
    bool lastDetectActive;

    /// @brief a count down timer to track green rest transition time
    SUMOTime greenRestTimer;
    SUMOTime greatestStartTime;
    /// @brief stores the force off time in coordinated mode
    SUMOTime forceOffTime;

    /// @brief flag to for the supervisory controller to denote whether phase is ready to switch or not.
    bool readyToSwitch;

    /**
     * @brief Get the Transition Time
     *
     * @param controller
     * @return SUMOTime
     */
    SUMOTime getTransitionTime(NEMALogic* controller);

    /**
     * @brief Get the Transition time given
     *
     * @param controller
     * @return SUMOTime
     */
    inline SUMOTime getTransitionTimeStateless(void) {
        return yellow + red;
    }

    /// @brief get the prior phase
    inline PhasePtr getSequentialPriorPhase(void) {
        return sequentialPriorPhase;
    }

    /// @brief set the prior phase
    inline void setSequentialPriorPhase(PhasePtr priorPhase) {
        sequentialPriorPhase = priorPhase;
    }

    /**
     * @brief calculate a vector of potention next phases
     *
     * @param controller
     * @return std::vector<PhaseTransitionLogic*>
     */
    std::vector<PhaseTransitionLogic*> trySwitch(NEMALogic* controller);

    /**
     * @brief return the PhaseTransitionLogic matching the toPhase
     *
     * @param toPhase a integer representing the target phase
     * @return PhaseTransitionLogic*
     */
    PhaseTransitionLogic* getTransition(int toPhase);

    /// @brief Return the ryg light string for the phase
    char getNEMAChar(int i);

    /// @brief accessory function to recalculate timing
    void recalculateTiming(void);

    /// @brief Force Enter. This Should only be called at initialization time
    inline void forceEnter(NEMALogic* controller) {
        enter(controller, sequentialPriorPhase);
    }

    /// @brief Return whether or not the phase index is controlled by me
    inline bool controlledIndex(int i) {
        return std::count(myPhaseStringInds.begin(), myPhaseStringInds.end(), i) > 0;
    }

    ///  @name Basic Phase Timing Parameters
    /// @{
    SUMOTime yellow;
    SUMOTime red;
    SUMOTime minDuration;
    SUMOTime maxDuration;
    SUMOTime nextMaxDuration;
    SUMOTime vehExt;
    /// @}

    /// @brief public method to set whether phase is active or not
    inline void cleanupExit(void) {
        transitionActive = false;
        readyToSwitch = false;
        myLightState = LightState::Red;
    }

    /// @brief simple internal check to see if done okay to transition
    inline bool okay2ForceSwitch(NEMALogic* controller) {
        return readyToSwitch && !transitionActive && (getTransitionTime(controller) <= TIME2STEPS(0));
    }

private:
    /// @brief A reference to the core phase of which NEMAPhase wraps
    MSPhaseDefinition* myCorePhase = nullptr;

    /// @name store references to myself, the last phase I was in, and the sequentially next phase
    /// @{
    PhasePtr myInstance = nullptr;
    PhasePtr myLastPhaseInstance = nullptr;
    PhasePtr sequentialPriorPhase = nullptr;
    /// @}

    // Phase Knowledge Space
    LightState myLightState;
    PhaseDetectorInfo myDetectorInfo;

    /// @name Timing Parameters
    /// @{
    SUMOTime maxGreenDynamic;
    SUMOTime myStartTime;
    SUMOTime myExpectedDuration;
    SUMOTime myLastEnd;
    /// @}

    /// @name Light String Parameters
    /// @{
    IntVector myPhaseStringInds;
    std::string myGreenString;
    std::string myYellowString;
    std::string myRedString;
    /// }

    /**
     * @brief Applies the vehicle extension timer if appropriate
     *
     * @param duration the current phase duration
     * @return SUMOTime
     */
    SUMOTime calcVehicleExtension(SUMOTime duration);

    /// @brief stores a sorted list of potential transitions
    std::vector<PhaseTransitionLogic*> myTransitions;

    /**
     * @brief handles entry to the phase during simulation
     * Sets the color to green and determines maximum duration
     *
     * @param controller a reference to the controller
     * @param lastPhase a reference to the last phase
     */
    void enter(NEMALogic* controller, PhasePtr lastPhase);

    /**
     * @brief this function replaces getNEMAStates calculation at every call
     * It sets my myGreenString, myYellowString, and myRedString on class creation
    */
    void setMyNEMAStates(void);

    /// @brief variable to store whether a transition is active or not
    bool transitionActive;

    /// @brief pointer to save the last transition
    PhaseTransitionLogic* lastTransitionDecision;

};


/**
 * @class PhaseTransitionLogic
 * @brief This class handles the transition logic between two phases
 *
 * This is intended to be extensible in the future. Each phase stores some
 * number of PhaseTransitionLogics, equal to the number of non-zero phases in a ring
 *
 */
class PhaseTransitionLogic {
public:
    /// @brief Typedef for commonly used phase pointer
    typedef NEMAPhase* PhasePtr;

    /**
     * @brief Construct a new Phase Transition Logic object
     *
     * @param fromPhase the phase who "owns" this transition
     * @param toPhase the phase to which I represent a transition to
     */
    PhaseTransitionLogic(
        PhasePtr fromPhase,
        PhasePtr toPhase
    );

    /**
     * @brief This function is the main PhaseTransitionLogic function
     * It is called by the fromPhase to check if a transition to the toPhase is acceptable
     *
     * @param controller a reference to the controller
     * @return true
     * @return false
     */
    bool okay(NEMALogic* controller);

    /**
     * @brief return the ring distance that this transition represents
     *
     * @param otherTrans the other PhaseTransitionLogic
     * @return int
     */
    int getDistance(PhaseTransitionLogic* otherTrans);
    /// @brief set the transition distance
    inline void setDistance(int d) {
        distance = d;
    }
    int distance;

    /// @brief deconstructor
    ~PhaseTransitionLogic() {};

    /// @brief get the to phase
    inline PhasePtr getToPhase(void) const {
        return toPhase;
    }

    /// @brief get the from phase
    inline PhasePtr getFromPhase(void) const {
        return fromPhase;
    }

private:
    PhasePtr fromPhase;
    PhasePtr toPhase;

    /// @brief build the transition logic based on the from and to phase
    void buildLogic(void);

    /**
     * @brief If the fromPhase is at a barrier, then this function
     * will be called to check whether the transition is valid
     *
     * @param controller a reference to the controller
     * @return true
     * @return false
     */
    bool fromBarrier(NEMALogic* controller);

    /**
     * @brief if the fromPhase is a coordinated phase, then
     * this logic will be checked
     *
     * @param controller
     * @return true
     * @return false
     */
    bool fromCoord(NEMALogic* controller);

    /**
     * @brief this represents the bare minimum logic,
     * that the toPhase has an active detector and that the fromPhase is ready to switch
     *
     * @param controller
     * @return true
     * @return false
     */
    bool freeBase(NEMALogic* controller);

    /**
     * @brief represents the bare minimum coordinate mode logic.
     * Requires that the toPhase can fit its minimum green time before the force off
     *
     * @param controller
     * @return true
     * @return false
     */
    bool coordBase(NEMALogic* controller);
};
