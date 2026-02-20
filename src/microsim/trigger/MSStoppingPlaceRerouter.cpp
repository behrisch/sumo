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
/// @file    MSStoppingPlaceRerouter.cpp
/// @author  Mirko Barthauer
/// @date    Mon, 17 June 2024
///
// The StoppingPlaceRerouter provides an interface to structure the rerouting
// to the best StoppingPlace according to the evaluation components and
// associated weights.
/****************************************************************************/
#include <utils/vehicle/SUMOVehicle.h>
#include <microsim/MSEdge.h>
#include <microsim/MSGlobals.h>
#include <microsim/MSLane.h>
#include <microsim/MSRoute.h>
#include <microsim/MSParkingArea.h>
#include <microsim/MSStoppingPlace.h>
#include <microsim/MSVehicleType.h>
#include <microsim/trigger/MSChargingStation.h>
#include "MSStoppingPlaceRerouter.h"

//#define DEBUG_STOPPINGPLACE
#define DEBUGCOND (veh.isSelected())
//#define DEBUGCOND (true)


///@brief Constructor
MSStoppingPlaceRerouter::MSStoppingPlaceRerouter(std::string paramPrefix, bool checkValidity, StoppingPlaceParamMap_t addEvalParams, StoppingPlaceParamSwitchMap_t addInvertParams) :
    myParamPrefix(paramPrefix), myCheckValidity(checkValidity) {
    myEvalParams = { {"probability", 0.}, {"capacity", 0.}, {"timefrom", 0.}, {"timeto", 0.}, {"distancefrom", 0.}, {"distanceto", 1.}, {"absfreespace", 0.}, {"relfreespace", 0.}, };
    myInvertParams = { {"probability", false}, { "capacity", true }, { "timefrom", false }, { "timeto", false }, { "distancefrom", false }, { "distanceto", false }, { "absfreespace", true }, { "relfreespace", true } };
    for (auto param : addEvalParams) {
        myEvalParams[param.first] = param.second;
        myInvertParams[param.first] = (addInvertParams.count(param.first) > 0) ? addInvertParams[param.first] : false;
    }
    for (auto param : myEvalParams) {
        myNormParams.insert({param.first, param.first != "probability"});
    }
}

MSStoppingPlace*
MSStoppingPlaceRerouter::rerouteStoppingPlace(MSStoppingPlace* destStoppingPlace, const std::vector<StoppingPlaceVisible>& stoppingPlaceCandidates, const std::vector<double>& probs, SUMOVehicle& veh, bool& newDestination, ConstMSEdgeVector& newRoute, StoppingPlaceParamMap_t& scores,
        const Prohibitions& closedEdges, const int insertStopIndex, const bool keepCurrentStop) {
    // Reroute destination from initial stopping place to an alternative stopping place
    // if the following conditions are met:
    // - next stop target is a stopping place of the right type
    // - target is included in the current alternative set
    // - target is visibly full
    // Any stopping places that are visibly full at the current location are
    // committed to the stopping place memory corresponding to their type

    MSStoppingPlace* nearStoppingPlace = nullptr;

    // get vehicle params
    bool destVisible = false;
    if (destStoppingPlace != nullptr) {
        destVisible = (&destStoppingPlace->getLane().getEdge() == veh.getEdge());
        // if the vehicle is on the destination stop edge it is always visible
        for (auto stoppingPlace : stoppingPlaceCandidates) {
            if (stoppingPlace.first == destStoppingPlace && stoppingPlace.second) {
                destVisible = true;
                break;
            }
        }
    }
    const MSRoute& route = veh.getRoute();

    MSStoppingPlace* onTheWay = nullptr;
    const int stopAnywhere = (int)getWeight(veh, "anywhere", -1);
    const bool ignoreDest = getWeight(veh, "ignoreDest", destStoppingPlace != nullptr ? 0 : 1) != 0;
    // check whether we are ready to accept any free stopping place along the
    // way to our destination
    if (stopAnywhere < 0 || stopAnywhere > getNumberStoppingPlaceReroutes(veh)) {
        if (!destVisible) {
            // cannot determine destination occupancy, only register visibly full
            for (const StoppingPlaceVisible& stoppingPlace : stoppingPlaceCandidates) {
                if (stoppingPlace.second && getLastStepStoppingPlaceOccupancy(stoppingPlace.first, &veh) >= getStoppingPlaceCapacity(stoppingPlace.first)) {
                    rememberStoppingPlaceScore(veh, stoppingPlace.first, "occupied");
                    rememberBlockedStoppingPlace(veh, stoppingPlace.first, &stoppingPlace.first->getLane().getEdge() == veh.getEdge());
                }
            }
#ifdef DEBUG_STOPPINGPLACE
            if (DEBUGCOND) {
                //std::cout << SIMTIME << " veh=" << veh.getID() << " dest=" << ((destStoppingPlace == nullptr)? "null" : destStoppingPlace->getID()) << " stopAnywhere=" << stopAnywhere << " reroutes=" << getNumberStoppingPlaceReroutes(veh) << " stay on original route\n";
            }
#endif
        }
    } else {
        double bestDist = std::numeric_limits<double>::max();
        const double brakeGap = veh.getBrakeGap(true);
        for (const StoppingPlaceVisible& item : stoppingPlaceCandidates) {
            if (item.second) {
                if (&item.first->getLane().getEdge() == veh.getEdge()
                        && getLastStepStoppingPlaceOccupancy(item.first, &veh) < getStoppingPlaceCapacity(item.first)) {
                    const double distToStart = item.first->getBeginLanePosition() - veh.getPositionOnLane();
                    const double distToEnd = item.first->getEndLanePosition() - veh.getPositionOnLane();
                    if (distToEnd > brakeGap) {
                        rememberStoppingPlaceScore(veh, item.first, "dist=" + toString(distToStart));
                        if (distToStart < bestDist) {
                            bestDist = distToStart;
                            onTheWay = item.first;
                        }
                    } else {
                        rememberStoppingPlaceScore(veh, item.first, "tooClose");
                    }
                }
            }
        }
#ifdef DEBUG_STOPPINGPLACE
        if (DEBUGCOND) {
            std::cout << SIMTIME << " veh=" << veh.getID()
                      << " dest=" << ((destStoppingPlace == nullptr) ? "null" : destStoppingPlace->getID()) << " stopAnywhere=" << stopAnywhere << " reroutes=" << getNumberStoppingPlaceReroutes(veh) << " alongTheWay=" << Named::getIDSecure(onTheWay) << "\n";
        }
#endif
    }
    if (!ignoreDest && !destVisible && onTheWay == nullptr) {
        return nullptr;
    }

    const bool destIsFull = destStoppingPlace != nullptr && getLastStepStoppingPlaceOccupancy(destStoppingPlace, &veh) >= getStoppingPlaceCapacity(destStoppingPlace);
    if (ignoreDest || destIsFull || onTheWay != nullptr) {
        // if the current route ends at the stopping place, the new route will
        // also end at the new stopping place
        newDestination = (destStoppingPlace != nullptr && &destStoppingPlace->getLane().getEdge() == route.getLastEdge()
                          && veh.getArrivalPos() >= destStoppingPlace->getBeginLanePosition()
                          && veh.getArrivalPos() <= destStoppingPlace->getEndLanePosition()
                          && veh.getStops().size() == 1);

#ifdef DEBUG_STOPPINGPLACE
        if (DEBUGCOND) {
            std::cout << SIMTIME << " veh=" << veh.getID()
                      << " newDest=" << newDestination
                      << " destIsFull=" << destIsFull
                      << " onTheWay=" << Named::getIDSecure(onTheWay)
                      << "\n";
        }
#endif
        std::map<MSStoppingPlace*, ConstMSEdgeVector> newRoutes;
        std::map<MSStoppingPlace*, ConstMSEdgeVector> stopApproaches;
        StoppingPlaceParamMap_t weights = collectWeights(veh); // add option to patch values for interdependent values
        StoppingPlaceParamMap_t maxValues;
        for (auto param : weights) {
            maxValues[param.first] = 0.;
        }

        // a map stores elegible stopping places
        StoppingPlaceMap_t stoppingPlaces;
        SUMOAbstractRouter<MSEdge, SUMOVehicle>& router = getRouter(veh, closedEdges);
        const double brakeGap = veh.getBrakeGap(true);

        if (onTheWay != nullptr) {
            // compute new route
            if (newDestination) {
                newRoute.push_back(veh.getEdge());
            } else {
                bool valid = evaluateDestination(veh, brakeGap, newDestination, onTheWay, getLastStepStoppingPlaceOccupancy(onTheWay, &veh), 1, router, stoppingPlaces, newRoutes, stopApproaches, maxValues, scores, insertStopIndex, keepCurrentStop);
                if (!valid) {
                    WRITE_WARNINGF(TL("Stopping place '%' along the way cannot be used by vehicle '%' for unknown reason"), onTheWay->getID(), veh.getID());
                    return nullptr;
                }
                newRoute = newRoutes[onTheWay];
            }
            return onTheWay;
        }
        int numAlternatives = 0;
        std::vector<std::tuple<SUMOTime, MSStoppingPlace*, int>> blockedTimes;
        resetStoppingPlaceScores(veh);

        if (destStoppingPlace != nullptr && destIsFull) {
            rememberStoppingPlaceScore(veh, destStoppingPlace, "occupied");
            rememberBlockedStoppingPlace(veh, destStoppingPlace, &destStoppingPlace->getLane().getEdge() == veh.getEdge());
        }
        const SUMOTime stoppingPlaceMemory = TIME2STEPS(getWeight(veh, "memory", 600));
        const double stoppingPlaceFrustration = getWeight(veh, "frustration", 100);
        const double stoppingPlaceKnowledge = getWeight(veh, "knowledge", 0);

        for (int i = 0; i < (int)stoppingPlaceCandidates.size(); ++i) {
            // alternative occupancy is randomized (but never full) if invisible
            // current destination must be visible at this point
            if (!useStoppingPlace(stoppingPlaceCandidates[i].first)) {
                continue;
            }
            const bool visible = stoppingPlaceCandidates[i].second || (stoppingPlaceCandidates[i].first == destStoppingPlace && destVisible);
            double occupancy = getStoppingPlaceOccupancy(stoppingPlaceCandidates[i].first, &veh);
            if (!visible && (stoppingPlaceKnowledge == 0 || stoppingPlaceKnowledge < RandHelper::rand(veh.getRNG()))) {
                double capacity = getStoppingPlaceCapacity(stoppingPlaceCandidates[i].first);
                const double minOccupancy = MIN2(capacity - NUMERICAL_EPS, (getNumberStoppingPlaceReroutes(veh) * capacity / stoppingPlaceFrustration));
                occupancy = RandHelper::rand(minOccupancy, capacity);
                // previously visited?
                SUMOTime blockedTime = sawBlockedStoppingPlace(veh, stoppingPlaceCandidates[i].first, false);
                if (blockedTime >= 0 && SIMSTEP - blockedTime < stoppingPlaceMemory) {
                    // assume it's still occupied
                    occupancy = capacity;
                    blockedTimes.push_back(std::make_tuple(blockedTime, stoppingPlaceCandidates[i].first, i));
#ifdef DEBUG_STOPPINGPLACE
                    if (DEBUGCOND) {
                        std::cout << "    altStoppingPlace=" << stoppingPlaceCandidates[i].first->getID() << " was blocked at " << time2string(blockedTime) << "\n";
                    }
#endif
                }
            }
            if (occupancy < getStoppingPlaceCapacity(stoppingPlaceCandidates[i].first)) {
                if (evaluateDestination(veh, brakeGap, newDestination, stoppingPlaceCandidates[i].first, occupancy, probs[i], router, stoppingPlaces, newRoutes, stopApproaches, maxValues, scores, insertStopIndex, keepCurrentStop)) {
                    numAlternatives++;
                }
            } else if (visible) {
                // might only be visible now (i.e. because it's on the other
                // side of the street), so we should remember this for later.
                rememberStoppingPlaceScore(veh, stoppingPlaceCandidates[i].first, "occupied");
                rememberBlockedStoppingPlace(veh, stoppingPlaceCandidates[i].first, &stoppingPlaceCandidates[i].first->getLane().getEdge() == veh.getEdge());
            }
        }

        if (numAlternatives == 0) {
            // use parkingArea with lowest blockedTime
            std::sort(blockedTimes.begin(), blockedTimes.end(),
            [](std::tuple<SUMOTime, MSStoppingPlace*, int> const & t1, std::tuple<SUMOTime, MSStoppingPlace*, int> const & t2) {
                if (std::get<0>(t1) < std::get<0>(t2)) {
                    return true;
                }
                if (std::get<0>(t1) == std::get<0>(t2)) {
                    if (std::get<1>(t1)->getID() < std::get<1>(t2)->getID()) {
                        return true;
                    }
                    if (std::get<1>(t1)->getID() == std::get<1>(t2)->getID()) {
                        return std::get<2>(t1) < std::get<2>(t2);
                    }
                }
                return false;
            }
                     );
            for (auto item : blockedTimes) {
                MSStoppingPlace* sp = std::get<1>(item);
                double prob = probs[std::get<2>(item)];
                // all stopping places are occupied. We have no good basis for
                // prefering one or the other based on estimated occupancy
                double occupancy = RandHelper::rand(getStoppingPlaceCapacity(sp));
                if (evaluateDestination(veh, brakeGap, newDestination, sp, occupancy, prob, router, stoppingPlaces, newRoutes, stopApproaches, maxValues, scores, insertStopIndex, keepCurrentStop)) {
#ifdef DEBUG_STOPPINGPLACE
                    if (DEBUGCOND) {
                        std::cout << "    altStoppingPlace=" << sp->getID() << " targeting occupied stopping place based on blockTime " << STEPS2TIME(std::get<0>(item)) << " among " << blockedTimes.size() << " alternatives\n";
                    }
#endif
                    numAlternatives = 1;
                    break;
                }
                //std::cout << "  candidate=" << item.second->getID() << " observed=" << time2string(item.first) << "\n";
            }
            if (numAlternatives == 0) {
                // take any random target but prefer one that hasn't been visited yet
                std::vector<std::pair<SUMOTime, MSStoppingPlace*>> candidates;
                for (const StoppingPlaceVisible& stoppingPlaceCandidate : stoppingPlaceCandidates) {
                    if (stoppingPlaceCandidate.first == destStoppingPlace) {
                        continue;
                    }
                    SUMOTime dummy = sawBlockedStoppingPlace(veh, stoppingPlaceCandidate.first, true);
                    if (dummy < 0) {
                        // randomize among the unvisited
                        dummy = -RandHelper::rand(1000000);
                    }
                    candidates.push_back(std::make_pair(dummy, stoppingPlaceCandidate.first));
                }
                std::sort(candidates.begin(), candidates.end(),
                [](std::tuple<SUMOTime, MSStoppingPlace*> const & t1, std::tuple<SUMOTime, MSStoppingPlace*> const & t2) {
                    return std::get<0>(t1) < std::get<0>(t2) || (std::get<0>(t1) == std::get<0>(t2) && std::get<1>(t1)->getID() < std::get<1>(t2)->getID());
                }
                         );
                for (auto item : candidates) {
                    if (evaluateDestination(veh, brakeGap, newDestination, item.second, 0, 1, router, stoppingPlaces, newRoutes, stopApproaches, maxValues, scores, insertStopIndex, keepCurrentStop)) {
#ifdef DEBUG_STOPPINGPLACE
                        if (DEBUGCOND) {
                            std::cout << "    altStoppingPlace=" << item.second->getID() << " targeting occupied stopping place (based on pure randomness) among " << candidates.size() << " alternatives\n";
                        }
#endif
                        numAlternatives = 1;
                        break;
                    }
                }
            }
        }
        getRouter(veh); // reset closed edges

#ifdef DEBUG_STOPPINGPLACE
        if (DEBUGCOND) {
            std::cout << "  maxValues=" << joinToString(maxValues, " ", ":") << "\n";
        }
#endif

        // minimum cost to get the parking area
        double minStoppingPlaceCost = 0.0;

        for (StoppingPlaceMap_t::iterator it = stoppingPlaces.begin(); it != stoppingPlaces.end(); ++it) {
            // get the parking values
            StoppingPlaceParamMap_t stoppingPlaceValues = it->second;

            if (weights["probability"] > 0. && maxValues["probability"] > 0.) {
                // random search should not drive past a usable parking area
                bool dominated = false;
                double endPos = it->first->getEndLanePosition();
                const ConstMSEdgeVector& to1 = stopApproaches[it->first];
                assert(to1.size() > 0);
                for (auto altSp : stoppingPlaces) {
                    if (altSp.first == it->first) {
                        continue;
                    }
                    const ConstMSEdgeVector& to2 = stopApproaches[altSp.first];
                    assert(to2.size() > 0);
                    if (to1.size() > to2.size()) {
                        if (std::equal(to2.begin(), to2.end(), to1.begin())) {
                            // other target lies on the route to the current candidate
                            dominated = true;
                            //std::cout << SIMTIME << " rrP veh=" << veh.getID() << " full=" << destParkArea->getID() << " cand=" << it->first->getID() << " onTheWay=" << altPa.first->getID() << "\n";
                            break;
                        }
                    } else if (to1 == to2 && endPos > altSp.first->getEndLanePosition()) {
                        // other target is on the same edge but ahead of the current candidate
                        dominated = true;
                        //std::cout << SIMTIME << " rrP veh=" << veh.getID() << " full=" << destParkArea->getID() << " cand=" << it->first->getID() << " sameEdge=" << altPa.first->getID() << "\n";
                        break;
                    }
                }
                double prob = 0;
                if (!dominated) {
                    prob = RandHelper::rand(stoppingPlaceValues["probability"], veh.getRNG());
                    stoppingPlaceValues["probability"] = 1.0 - prob / maxValues["probability"];
                } else {
                    // worst probability score
                    stoppingPlaceValues["probability"] = 1.0;
                }
            } else {
                // value takes no effect due to weight=0
                stoppingPlaceValues["probability"] = 0;
            }

            // get the parking area cost
            double stoppingPlaceCost = getTargetValue(stoppingPlaceValues, maxValues, weights, myNormParams, myInvertParams);
            rememberStoppingPlaceScore(veh, it->first, toString(stoppingPlaceCost));

            // get the parking area with minimum cost
            if (nearStoppingPlace == nullptr || stoppingPlaceCost < minStoppingPlaceCost) {
                minStoppingPlaceCost = stoppingPlaceCost;
                nearStoppingPlace = it->first;
                newRoute = newRoutes[nearStoppingPlace];
            }
#ifdef DEBUG_STOPPINGPLACE
            if (DEBUGCOND) {
                std::cout << "    altStoppingPlace=" << it->first->getID() << " score=" << stoppingPlaceCost << " vals=" << joinToString(stoppingPlaceValues, " ", ":") << "\n";
            }
#endif
        }
        // expose the scores of the best solution
        if (nearStoppingPlace != nullptr) {
            for (auto component : stoppingPlaces[nearStoppingPlace]) {
                scores[component.first] = component.second;
            }
        }
        setNumberStoppingPlaceReroutes(veh, getNumberStoppingPlaceReroutes(veh) + 1);
    } else {
#ifdef DEBUG_STOPPINGPLACE
        if (DEBUGCOND) {
            std::cout << SIMTIME << " veh=" << veh.getID() << " dest=" << destStoppingPlace->getID() << " sufficient space\n";
        }
#endif
    }

#ifdef DEBUG_STOPPINGPLACE
    if (DEBUGCOND) {
        std::cout << "  stoppingPlaceResult=" << Named::getIDSecure(nearStoppingPlace) << "\n";
    }
#endif
    return nearStoppingPlace;
}


bool
MSStoppingPlaceRerouter::evaluateDestination(SUMOVehicle& veh, double brakeGap, bool newDestination, MSStoppingPlace* alternative,
        double occupancy, double prob, SUMOAbstractRouter<MSEdge, SUMOVehicle>& router, StoppingPlaceMap_t& stoppingPlaces,
        std::map<MSStoppingPlace*, ConstMSEdgeVector>& newRoutes, std::map<MSStoppingPlace*, ConstMSEdgeVector>& stoppingPlaceApproaches,
        StoppingPlaceParamMap_t& maxValues, StoppingPlaceParamMap_t& addInput, const int insertStopIndex, const bool keepCurrentStop) {

    // a map stores the stopping place values
    StoppingPlaceParamMap_t stoppingPlaceValues;
    const SUMOTime now = SIMSTEP;

    const MSRoute& route = veh.getRoute();
    const RGBColor& c = route.getColor();
    const MSEdge* stoppingPlaceEdge = &(alternative->getLane().getEdge());

    const bool includeInternalLengths = MSGlobals::gUsingInternalLanes && MSNet::getInstance()->hasInternalLinks();

    // Compute the route from the current edge to the stopping place edge
    ConstMSEdgeVector edgesToStop;
    ConstMSEdgeVector edgesUpstream;
    const double targetPos = alternative->getLastFreePos(veh);
    MSRouteIterator rerouteOriginIt = determineRerouteOrigin(veh, insertStopIndex);
    double posOnLane = veh.getPositionOnLane();
    if (insertStopIndex > 0) {
        posOnLane = 0.;
        // determine preceding edges
        for (MSRouteIterator it = veh.getCurrentRouteEdge(); it != rerouteOriginIt; ++it) {
            if (it != rerouteOriginIt) {
                edgesUpstream.push_back(*it);
            }
        }
    }
    const MSEdge* rerouteOrigin = *rerouteOriginIt;
    router.compute(rerouteOrigin, posOnLane, stoppingPlaceEdge, targetPos, &veh, now, edgesToStop, true);
    if (edgesToStop.size() > 0) {
        // Compute the route from the stopping place edge to the end of the route
        if (insertStopIndex == 0 && rerouteOrigin != veh.getEdge()) {
            edgesToStop.insert(edgesToStop.begin(), veh.getEdge());
        }
        // prepend preceding edges
        std::reverse(edgesUpstream.begin(), edgesUpstream.end());
        for (auto edge : edgesUpstream) {
            edgesToStop.insert(edgesToStop.begin(), edge);
        }
        ConstMSEdgeVector edgesFromStop;
        stoppingPlaceApproaches[alternative] = edgesToStop;

        const MSEdge* nextDestination = route.getLastEdge();
        double nextPos = veh.getArrivalPos();
        int nextDestinationIndex = route.size() - 1;
        if (!newDestination) {
            std::vector<std::pair<int, double> > stopIndices = veh.getStopIndices();
            int nextDestStopIndex = 1 + insertStopIndex;
            if (!keepCurrentStop) {
                nextDestStopIndex++;
            }
            if ((int)stopIndices.size() > nextDestStopIndex) {
                nextDestinationIndex = stopIndices[nextDestStopIndex].first;
                nextDestination = route.getEdges()[nextDestinationIndex];
                nextPos = stopIndices[nextDestStopIndex].second;
            }
            router.compute(stoppingPlaceEdge, targetPos, nextDestination, nextPos, &veh, now, edgesFromStop, true);
        }
        if (edgesFromStop.size() > 0 || newDestination) {
            stoppingPlaceValues["probability"] = prob;
            if (stoppingPlaceValues["probability"] > maxValues["probability"]) {
                maxValues["probability"] = stoppingPlaceValues["probability"];
            }
            stoppingPlaceValues["capacity"] = getStoppingPlaceCapacity(alternative);
            stoppingPlaceValues["absfreespace"] = stoppingPlaceValues["capacity"] - occupancy;
            // if capacity = 0 then absfreespace and relfreespace are also 0
            stoppingPlaceValues["relfreespace"] = stoppingPlaceValues["absfreespace"] / MAX2(1.0, stoppingPlaceValues["capacity"]);
            MSRoute routeToPark(route.getID() + "!to" + myParamPrefix + "#1", edgesToStop, false,
                                &c == &RGBColor::DEFAULT_COLOR ? nullptr : new RGBColor(c), route.getStops());

            // The distance from the current edge to the new parking area
            double toPos = alternative->getBeginLanePosition();
            if (&alternative->getLane().getEdge() == veh.getEdge()) {
                toPos = MAX2(veh.getPositionOnLane(), toPos);
            }
            stoppingPlaceValues["distanceto"] = routeToPark.getDistanceBetween(veh.getPositionOnLane(), toPos,
                                                routeToPark.begin(), routeToPark.end() - 1, includeInternalLengths);

            if (stoppingPlaceValues["distanceto"] == std::numeric_limits<double>::max()) {
                WRITE_WARNINGF(TL("Invalid distance computation for vehicle '%' to stopping place '%' at time=%."),
                               veh.getID(), alternative->getID(), time2string(now));
            }
            const double endPos = alternative->getLastFreePos(veh, veh.getPositionOnLane() + brakeGap);
            const double distToEnd = stoppingPlaceValues["distanceto"] - toPos + endPos;

            if (distToEnd < brakeGap) {
                rememberStoppingPlaceScore(veh, alternative, "tooClose");
                return false;
            }

            // The time to reach the new stopping place
            const double correctionLastEdge = ((alternative->getLane().getLength() - alternative->getEndLanePosition()) / alternative->getLane().getVehicleMaxSpeed(&veh));
            const double correctionFirstEdge = veh.getPositionOnLane() / edgesToStop.front()->getVehicleMaxSpeed(&veh);

            stoppingPlaceValues["timeto"] = router.recomputeCosts(edgesToStop, &veh, SIMSTEP) - correctionLastEdge - correctionFirstEdge;
            ConstMSEdgeVector newEdges = edgesToStop;
            if (newDestination) {
                stoppingPlaceValues["distancefrom"] = 0;
                stoppingPlaceValues["timefrom"] = 0;
            } else {
                MSRoute routeFromPark(route.getID() + "!from" + myParamPrefix + "#1", edgesFromStop, false,
                                      &c == &RGBColor::DEFAULT_COLOR ? nullptr : new RGBColor(c), route.getStops());
                // The distance from the new parking area to the end of the route
                stoppingPlaceValues["distancefrom"] = routeFromPark.getDistanceBetween(alternative->getBeginLanePosition(), routeFromPark.getLastEdge()->getLength(),
                                                      routeFromPark.begin(), routeFromPark.end() - 1, includeInternalLengths);
                if (stoppingPlaceValues["distancefrom"] == std::numeric_limits<double>::max()) {
                    WRITE_WARNINGF(TL("Invalid distance computation for vehicle '%' from stopping place '%' at time=%."),
                                   veh.getID(), alternative->getID(), time2string(SIMSTEP));
                }
                // The time to reach this area
                stoppingPlaceValues["timefrom"] = router.recomputeCosts(edgesFromStop, &veh, SIMSTEP) - (alternative->getEndLanePosition() / alternative->getLane().getSpeedLimit());
                newEdges.insert(newEdges.end(), edgesFromStop.begin() + 1, edgesFromStop.end());
                newEdges.insert(newEdges.end(), route.begin() + nextDestinationIndex + 1, route.end());
            }

            // add some additional/custom target function components
            if (!evaluateCustomComponents(veh, brakeGap, newDestination, alternative, occupancy, prob, router, stoppingPlaceValues, stoppingPlaceApproaches[alternative], newEdges, maxValues, addInput)) {
                return false;
            }
            if (!myCheckValidity || validComponentValues(stoppingPlaceValues)) {
                updateMaxValues(stoppingPlaceValues, maxValues);
                stoppingPlaces[alternative] = stoppingPlaceValues;
                newRoutes[alternative] = newEdges;
                return true;
            } else {
                return false;
            }
        } else {
            rememberStoppingPlaceScore(veh, alternative, "unreachable");
        }
    } else {
        rememberStoppingPlaceScore(veh, alternative, "unreachable");
    }
    // unreachable
    return false;
}


bool
MSStoppingPlaceRerouter::evaluateCustomComponents(SUMOVehicle& /*veh*/, double /*brakeGap*/, bool /*newDestination*/,
        MSStoppingPlace* /*alternative*/, double /*occupancy*/, double /*prob*/, SUMOAbstractRouter<MSEdge, SUMOVehicle>& /*router*/,
        StoppingPlaceParamMap_t& /*stoppingPlaceValues*/, ConstMSEdgeVector& /*newRoute*/, ConstMSEdgeVector& /*stoppingPlaceApproach*/,
        StoppingPlaceParamMap_t& /*maxValues*/, StoppingPlaceParamMap_t& /*addInput*/) {
    return true;
}


bool
MSStoppingPlaceRerouter::validComponentValues(StoppingPlaceParamMap_t& /* stoppingPlaceValues */) {
    return true;
}


bool
MSStoppingPlaceRerouter::useStoppingPlace(MSStoppingPlace* /* stoppingPlace */) {
    return true;
}


SUMOAbstractRouter<MSEdge, SUMOVehicle>&
MSStoppingPlaceRerouter::getRouter(SUMOVehicle& veh, const Prohibitions& prohibited) {
    return MSNet::getInstance()->getRouterTT(veh.getRNGIndex(), prohibited);
}


MSStoppingPlaceRerouter::StoppingPlaceParamMap_t
MSStoppingPlaceRerouter::collectWeights(SUMOVehicle& veh) {
    MSStoppingPlaceRerouter::StoppingPlaceParamMap_t result;
    myEvalParams["distanceto"] = getWeight(veh, "distance.weight", myEvalParams["distanceto"]);
    for (auto evalParam : myEvalParams) {
        result[evalParam.first] = getWeight(veh, evalParam.first + ".weight", evalParam.second);
    }
    result["probability"] = getWeight(veh, "probability.weight", 0.);
    return result;
}


double
MSStoppingPlaceRerouter::getWeight(SUMOVehicle& veh, const std::string param, const double defaultWeight, const bool warn) {
    // get custom vehicle parameter
    const std::string key = myParamPrefix + "." + param;
    if (veh.getParameter().hasParameter(key)) {
        try {
            return StringUtils::toDouble(veh.getParameter().getParameter(key, "-1"));
        } catch (...) {
            WRITE_WARNINGF(TL("Invalid value '%' for vehicle parameter '%'"), veh.getParameter().getParameter(key, "-1"), key);
        }
    } else {
        // get custom vType parameter
        if (veh.getVehicleType().getParameter().hasParameter(key)) {
            try {
                return StringUtils::toDouble(veh.getVehicleType().getParameter().getParameter(key, "-1"));
            } catch (...) {
                WRITE_WARNINGF(TL("Invalid value '%' for vType parameter '%'"), veh.getVehicleType().getParameter().getParameter(key, "-1"), key);
            }
        }
    }
    if (warn) {
        WRITE_MESSAGEF("Vehicle '%' does not supply vehicle parameter '%'. Using default of %\n", veh.getID(), key, toString(defaultWeight));
    }
    return defaultWeight;
}


void
MSStoppingPlaceRerouter::updateMaxValues(StoppingPlaceParamMap_t& stoppingPlaceValues, StoppingPlaceParamMap_t& maxValues) {
    for (auto it = maxValues.begin(); it != maxValues.end(); ++it) {
        if (stoppingPlaceValues[it->first] > it->second) {
            it->second = stoppingPlaceValues[it->first];
        }
    }
}


double
MSStoppingPlaceRerouter::getTargetValue(const StoppingPlaceParamMap_t& absValues, const StoppingPlaceParamMap_t& maxValues, const StoppingPlaceParamMap_t& weights, const StoppingPlaceParamSwitchMap_t& norm, const StoppingPlaceParamSwitchMap_t& invert) {
    double cost = 0.;
    for (StoppingPlaceParamMap_t::const_iterator sc = absValues.begin(); sc != absValues.end(); ++sc) {
        double weight = weights.at(sc->first);
        double val = sc->second;
        if (norm.at(sc->first) && maxValues.at(sc->first) > 0.) {
            val /= maxValues.at(sc->first);
        }
        cost += (invert.at(sc->first)) ? weight * (1. - val) : weight * val;
    }
    return cost;
}


/****************************************************************************/
