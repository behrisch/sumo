/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2001-2025 German Aerospace Center (DLR) and others.
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
/// @file    GNEMoveElement.cpp
/// @author  Pablo Alvarez Lopez
/// @date    Mar 2020
///
// Class used for move shape elements
/****************************************************************************/

#include <netedit/changes/GNEChange_Attribute.h>
#include <netedit/GNEViewNet.h>
#include <netedit/GNEViewParent.h>
#include <netedit/frames/common/GNEMoveFrame.h>

#include "GNEMoveElement.h"

// ===========================================================================
// GNEMoveOperation method definitions
// ===========================================================================

GNEMoveOperation::GNEMoveOperation(GNEMoveElement* _moveElement,
                                   const Position _originalPosition) :
    moveElement(_moveElement),
    originalShape({_originalPosition}),
              shapeToMove({_originalPosition}),
              allowChangeLane(false),
              firstGeometryPoint(false),
operationType(OperationType::POSITION) {
}


GNEMoveOperation::GNEMoveOperation(GNEMoveElement* _moveElement,
                                   const PositionVector _originalShape) :
    moveElement(_moveElement),
    originalShape(_originalShape),
    shapeToMove(_originalShape),
    allowChangeLane(false),
    firstGeometryPoint(false),
    operationType(OperationType::ENTIRE_SHAPE) {
}

GNEMoveOperation::GNEMoveOperation(GNEMoveElement* _moveElement,
                                   const PositionVector _originalShape,
                                   const bool _firstGeometryPoint,
                                   const OperationType _operationType) :
    moveElement(_moveElement),
    originalShape(_originalShape),
    shapeToMove(_originalShape),
    allowChangeLane(false),
    firstGeometryPoint(_firstGeometryPoint),
    operationType(_operationType) {
}

GNEMoveOperation::GNEMoveOperation(GNEMoveElement* _moveElement,
                                   const PositionVector _originalShape,
                                   const std::vector<int> _originalgeometryPoints,
                                   const PositionVector _shapeToMove,
                                   const std::vector<int> _geometryPointsToMove) :
    moveElement(_moveElement),
    originalShape(_originalShape),
    originalGeometryPoints(_originalgeometryPoints),
    shapeToMove(_shapeToMove),
    geometryPointsToMove(_geometryPointsToMove),
    allowChangeLane(false),
    firstGeometryPoint(false),
    operationType(OperationType::GEOMETRY_POINTS) {
}


GNEMoveOperation::GNEMoveOperation(GNEMoveElement* _moveElement,
                                   const GNELane* _lane,
                                   const double _firstPosition,
                                   const bool _allowChangeLane) :
    moveElement(_moveElement),
    firstLane(_lane),
    firstPosition(_firstPosition * _lane->getLengthGeometryFactor()),
    allowChangeLane(_allowChangeLane),
    firstGeometryPoint(false),
    operationType(OperationType::SINGLE_LANE) {
}


GNEMoveOperation::GNEMoveOperation(GNEMoveElement* _moveElement,
                                   const GNELane* _lane,
                                   const double _firstPosition,
                                   const double _lastPosition,
                                   const bool _allowChangeLane,
                                   const OperationType _operationType) :
    moveElement(_moveElement),
    firstLane(_lane),
    firstPosition(_firstPosition * _lane->getLengthGeometryFactor()),
    lastPosition(_lastPosition * _lane->getLengthGeometryFactor()),
    allowChangeLane(_allowChangeLane),
    firstGeometryPoint(false),
    operationType(_operationType) {
}


GNEMoveOperation::GNEMoveOperation(GNEMoveElement* _moveElement,
                                   const GNELane* _firstLane,
                                   const double _firstStartPos,
                                   const GNELane* _lastLane,
                                   const double _lastStartPos,
                                   const bool _allowChangeLane,
                                   const OperationType _operationType) :
    moveElement(_moveElement),
    firstLane(_firstLane),
    firstPosition((_firstStartPos != INVALID_DOUBLE) ? _firstStartPos * _firstLane->getLengthGeometryFactor() : INVALID_DOUBLE),
    lastLane(_lastLane),
    lastPosition((_lastStartPos != INVALID_DOUBLE) ? _lastStartPos * _lastLane->getLengthGeometryFactor() : INVALID_DOUBLE),
    allowChangeLane(_allowChangeLane),
    firstGeometryPoint(false),
    operationType(_operationType) {
}


GNEMoveOperation::~GNEMoveOperation() {}

// ===========================================================================
// GNEMoveOffset method definitions
// ===========================================================================

GNEMoveOffset::GNEMoveOffset() :
    x(0),
    y(0),
    z(0) {
}


GNEMoveOffset::GNEMoveOffset(const double x_, const double y_) :
    x(x_),
    y(y_),
    z(0) {
}


GNEMoveOffset::GNEMoveOffset(const double z_) :
    x(0),
    y(0),
    z(z_) {
}


GNEMoveOffset::~GNEMoveOffset() {}

// ===========================================================================
// GNEMoveResult method definitions
// ===========================================================================

GNEMoveResult::GNEMoveResult(const GNEMoveOperation* moveOperation) :
    operationType(moveOperation->operationType),
    firstLaneOffset(0),
    newFirstLane(nullptr),
    newFirstPos(0),
    lastLaneOffset(0),
    newLastLane(nullptr),
    newLastPos(0) {}


GNEMoveResult::~GNEMoveResult() {}


void
GNEMoveResult::clearLanes() {
    firstLaneOffset = 0;
    newFirstLane = nullptr;
    lastLaneOffset = 0;
    newLastLane = nullptr;
}

// ===========================================================================
// GNEMoveElement method definitions
// ===========================================================================

GNEMoveElement::GNEMoveElement() :
    myMoveElementLateralOffset(0) {
}


GNEMoveOperation*
GNEMoveElement::calculateMoveShapeOperation(const GUIGlObject* obj, const PositionVector originalShape,
        const bool maintainShapeClosed) {
    // get moved geometry points
    const auto geometryPoints = gViewObjectsHandler.getSelectedGeometryPoints(obj);
    // get pos over shape
    const auto posOverShape = gViewObjectsHandler.getSelectedPositionOverShape(obj);
    // declare shape to move
    PositionVector shapeToMove = originalShape;
    const int lastIndex = (int)shapeToMove.size() - 1;
    // check if move existent geometry points or create new
    if (geometryPoints.size() > 0) {
        // move geometry point without creating new geometry point
        if (maintainShapeClosed && ((geometryPoints.front() == 0) || (geometryPoints.front() == lastIndex))) {
            // move first and last point
            return new GNEMoveOperation(this, originalShape, {0, lastIndex}, shapeToMove, {0, lastIndex});
        } else {
            return new GNEMoveOperation(this, originalShape, {geometryPoints.front()}, shapeToMove, {geometryPoints.front()});
        }
    } else if (posOverShape != Position::INVALID) {
        // create new geometry point and keep new index (if we clicked near of shape)
        const int newIndex = shapeToMove.insertAtClosest(posOverShape, true);
        return new GNEMoveOperation(this, originalShape, {shapeToMove.indexOfClosest(posOverShape)}, shapeToMove, {newIndex});
    } else {
        return nullptr;
    }
}


void
GNEMoveElement::moveElement(const GNEViewNet* viewNet, GNEMoveOperation* moveOperation, const GNEMoveOffset& offset) {
    // declare move result
    GNEMoveResult moveResult(moveOperation);
    // set geometry points to move
    moveResult.geometryPointsToMove = moveOperation->geometryPointsToMove;
    // check if we're moving over a lane shape, an entire shape or only certain geometry point
    if (moveOperation->firstLane) {
        // calculate movement over lane
        if (moveOperation->lastLane) {
            // continue depending of operationType
            if ((moveOperation->operationType == GNEMoveOperation::OperationType::MULTIPLE_LANES_MOVE_FIRST) ||
                    (moveOperation->operationType == GNEMoveOperation::OperationType::MULTIPLE_LANES_MOVE_BOTH_FIRST)) {
                // move only first position
                calculateMoveResult(moveResult, viewNet, moveOperation->firstLane, moveOperation->firstPosition, offset,
                                    0, moveOperation->firstLane->getLaneShapeLength());
            } else if ((moveOperation->operationType == GNEMoveOperation::OperationType::MULTIPLE_LANES_MOVE_LAST) ||
                       (moveOperation->operationType == GNEMoveOperation::OperationType::MULTIPLE_LANES_MOVE_BOTH_LAST)) {
                // move only last position
                calculateMoveResult(moveResult, viewNet, moveOperation->lastLane, moveOperation->lastPosition, offset,
                                    0, moveOperation->lastLane->getLaneShapeLength());
            }
        } else {
            if (moveOperation->operationType == GNEMoveOperation::OperationType::SINGLE_LANE) {
                // move first position around the entire lane
                calculateMoveResult(moveResult, viewNet, moveOperation->firstLane, moveOperation->firstPosition, offset,
                                    0, moveOperation->firstLane->getLaneShapeLength());
            } else if (moveOperation->operationType == GNEMoveOperation::OperationType::SINGLE_LANE_MOVE_FIRST) {
                // move first position around [0, lastPosition]
                calculateMoveResult(moveResult, viewNet, moveOperation->firstLane, moveOperation->firstPosition, offset,
                                    0, moveOperation->lastPosition);
            } else if (moveOperation->operationType == GNEMoveOperation::OperationType::SINGLE_LANE_MOVE_LAST) {
                // move first position around [firstPosition, laneLength]
                calculateMoveResult(moveResult, viewNet, moveOperation->firstLane, moveOperation->lastPosition, offset,
                                    moveOperation->firstPosition, moveOperation->firstLane->getLaneShapeLength());
            } else {
                // move both first and last positions
                calculateMoveResult(moveResult, viewNet, moveOperation->firstLane, moveOperation->firstPosition,
                                    moveOperation->lastPosition, offset);
            }
            // calculate new lane
            if (moveOperation->allowChangeLane) {
                calculateNewLaneChange(viewNet, moveOperation->firstLane, moveResult.newFirstLane, moveResult.firstLaneOffset);
            } else {
                moveResult.clearLanes();
            }
        }
    } else if (moveOperation->geometryPointsToMove.size() > 0) {
        // set values in moveResult
        moveResult.shapeToUpdate = moveOperation->shapeToMove;
        // move geometry points
        for (const auto& geometryPointIndex : moveOperation->geometryPointsToMove) {
            if (moveResult.shapeToUpdate[geometryPointIndex] != Position::INVALID) {
                // add offset
                moveResult.shapeToUpdate[geometryPointIndex].add(offset.x, offset.y, offset.z);
                // apply snap to active grid
                moveResult.shapeToUpdate[geometryPointIndex] = viewNet->snapToActiveGrid(moveResult.shapeToUpdate[geometryPointIndex]);
            } else {
                throw ProcessError("trying to move an invalid position");
            }
        }
    } else {
        // set values in moveResult
        moveResult.shapeToUpdate = moveOperation->shapeToMove;
        // move entire shape
        for (auto& geometryPointIndex : moveResult.shapeToUpdate) {
            if (geometryPointIndex != Position::INVALID) {
                // add offset
                geometryPointIndex.add(offset.x, offset.y, offset.z);
                // apply snap to active grid
                geometryPointIndex = viewNet->snapToActiveGrid(geometryPointIndex);
            } else {
                throw ProcessError("trying to move an invalid position");
            }
        }
        // check if we're adjusting width or height
        if ((moveOperation->operationType == GNEMoveOperation::OperationType::WIDTH) ||
                (moveOperation->operationType == GNEMoveOperation::OperationType::HEIGHT) ||
                (moveOperation->operationType == GNEMoveOperation::OperationType::LENGTH)) {
            // calculate extrapolate vector
            moveResult.shapeToUpdate = calculateExtrapolatedVector(moveOperation, moveResult);
        }
    }
    // move shape element
    moveOperation->moveElement->setMoveShape(moveResult);
}


void
GNEMoveElement::commitMove(const GNEViewNet* viewNet, GNEMoveOperation* moveOperation, const GNEMoveOffset& offset, GNEUndoList* undoList) {
    // declare move result
    GNEMoveResult moveResult(moveOperation);
    // check if we're moving over a lane shape, an entire shape or only certain geometry point
    if (moveOperation->firstLane) {
        // calculate original move result
        moveResult.newFirstLane = moveOperation->firstLane;
        moveResult.newFirstPos = moveOperation->firstPosition;
        moveResult.newLastLane = moveOperation->lastLane;
        moveResult.newLastPos = moveOperation->lastPosition;
        // set original positions in element
        moveOperation->moveElement->setMoveShape(moveResult);
        // calculate movement over lane
        if (moveOperation->lastLane) {
            if ((moveOperation->operationType == GNEMoveOperation::OperationType::MULTIPLE_LANES_MOVE_FIRST) ||
                    (moveOperation->operationType == GNEMoveOperation::OperationType::MULTIPLE_LANES_MOVE_BOTH_FIRST)) {
                // move only first position
                calculateMoveResult(moveResult, viewNet, moveOperation->firstLane, moveOperation->firstPosition, offset,
                                    0, moveOperation->firstLane->getLaneShapeLength());
            } else if ((moveOperation->operationType == GNEMoveOperation::OperationType::MULTIPLE_LANES_MOVE_LAST) ||
                       (moveOperation->operationType == GNEMoveOperation::OperationType::MULTIPLE_LANES_MOVE_BOTH_LAST)) {
                // move only two position
                calculateMoveResult(moveResult, viewNet, moveOperation->lastLane, moveOperation->lastPosition, offset,
                                    0, moveOperation->lastLane->getLaneShapeLength());
            }
            // calculate new lane
            if (moveOperation->allowChangeLane) {
                calculateNewLaneChange(viewNet, moveOperation->firstLane, moveResult.newFirstLane, moveResult.firstLaneOffset);
                calculateNewLaneChange(viewNet, moveOperation->lastLane, moveResult.newLastLane, moveResult.lastLaneOffset);
            } else {
                moveResult.clearLanes();
            }
        } else {
            if (moveOperation->operationType == GNEMoveOperation::OperationType::SINGLE_LANE) {
                // move first position around the entire lane
                calculateMoveResult(moveResult, viewNet, moveOperation->firstLane, moveOperation->firstPosition, offset,
                                    0, moveOperation->firstLane->getLaneShapeLength());
            } else if (moveOperation->operationType == GNEMoveOperation::OperationType::SINGLE_LANE_MOVE_FIRST) {
                // move first position around [0, lastPosition]
                calculateMoveResult(moveResult, viewNet, moveOperation->firstLane, moveOperation->firstPosition, offset,
                                    0, moveOperation->lastPosition);
            } else if (moveOperation->operationType == GNEMoveOperation::OperationType::SINGLE_LANE_MOVE_LAST) {
                // move first position around [firstPosition, laneLength]
                calculateMoveResult(moveResult, viewNet, moveOperation->firstLane, moveOperation->lastPosition, offset,
                                    moveOperation->firstPosition, moveOperation->firstLane->getLaneShapeLength());
            } else {
                // move both first and last positions
                calculateMoveResult(moveResult, viewNet, moveOperation->firstLane, moveOperation->firstPosition,
                                    moveOperation->lastPosition, offset);
            }
            // calculate new lane
            if (moveOperation->allowChangeLane) {
                calculateNewLaneChange(viewNet, moveOperation->firstLane, moveResult.newFirstLane, moveResult.firstLaneOffset);
            } else {
                moveResult.clearLanes();
            }
        }
    } else {
        // set original geometry points to move
        moveResult.geometryPointsToMove = moveOperation->originalGeometryPoints;
        // set shapeToUpdate with originalPosOverLanes
        moveResult.shapeToUpdate = moveOperation->originalShape;
        // first restore original geometry geometry
        moveOperation->moveElement->setMoveShape(moveResult);
        // set new geometry points to move
        moveResult.geometryPointsToMove = moveOperation->geometryPointsToMove;
        // set values in moveResult
        moveResult.shapeToUpdate = moveOperation->shapeToMove;
        // check if we're moving an entire shape or only certain geometry point
        if (moveOperation->geometryPointsToMove.size() > 0) {
            // only move certain geometry points
            for (const auto& geometryPointIndex : moveOperation->geometryPointsToMove) {
                if (moveResult.shapeToUpdate[geometryPointIndex] != Position::INVALID) {
                    // add offset
                    moveResult.shapeToUpdate[geometryPointIndex].add(offset.x, offset.y, offset.z);
                    // apply snap to active grid
                    moveResult.shapeToUpdate[geometryPointIndex] = viewNet->snapToActiveGrid(moveResult.shapeToUpdate[geometryPointIndex]);
                } else {
                    throw ProcessError("trying to move an invalid position");
                }
            }
            // remove double points if merge points is enabled (only in commitMove)
            if (viewNet->getViewParent()->getMoveFrame()->getCommonMoveOptions()->getMergeGeometryPoints() && (moveResult.shapeToUpdate.size() > 2)) {
                moveResult.shapeToUpdate.removeDoublePoints(2);
            }
        } else {
            // move entire shape
            for (auto& geometryPointIndex : moveResult.shapeToUpdate) {
                if (geometryPointIndex != Position::INVALID) {
                    // add offset
                    geometryPointIndex.add(offset.x, offset.y, offset.z);
                    // apply snap to active grid
                    geometryPointIndex = viewNet->snapToActiveGrid(geometryPointIndex);
                } else {
                    throw ProcessError("trying to move an invalid position");
                }
            }
            // check if we're adjusting width or height
            if ((moveOperation->operationType == GNEMoveOperation::OperationType::WIDTH) ||
                    (moveOperation->operationType == GNEMoveOperation::OperationType::HEIGHT) ||
                    (moveOperation->operationType == GNEMoveOperation::OperationType::LENGTH)) {
                // calculate extrapolate vector
                moveResult.shapeToUpdate = calculateExtrapolatedVector(moveOperation, moveResult);
            }
        }
    }
    // commit move shape
    moveOperation->moveElement->commitMoveShape(moveResult, undoList);
}


double
GNEMoveElement::calculateLaneOffset(const GNEViewNet* viewNet, const GNELane* lane, const double firstPosition, const double lastPosition,
                                    const GNEMoveOffset& offset, const double extremFrom, const double extremTo) {
    // declare laneOffset
    double laneOffset = 0;
    // calculate central position between two given positions
    const double offsetCentralPosition = (firstPosition + lastPosition) * 0.5;
    // calculate middle length between two given positions
    const double middleLength = std::abs(lastPosition - firstPosition) * 0.5;
    // calculate lane position at offset given by offsetCentralPosition
    Position laneCentralPosition = lane->getLaneShape().positionAtOffset2D(offsetCentralPosition);
    // apply offset to positionAtCentralPosition
    laneCentralPosition.add(offset.x, offset.y, offset.z);
    // snap to grid
    laneCentralPosition = viewNet->snapToActiveGrid(laneCentralPosition);
    // calculate offset over lane using laneCentralPosition
    const double offsetLaneCentralPositionPerpendicular = lane->getLaneShape().nearest_offset_to_point2D(laneCentralPosition);
    // check if offset is within lane shape
    if (offsetLaneCentralPositionPerpendicular == -1) {
        // calculate non-perpendicular offset over lane using laneCentralPosition
        const double offsetLaneCentralPosition = lane->getLaneShape().nearest_offset_to_point2D(laneCentralPosition, false);
        // due laneCentralPosition is out of lane shape, then place positions in extremes
        if (offsetLaneCentralPosition == 0) {
            laneOffset = firstPosition;
        } else {
            laneOffset = lastPosition - lane->getLaneShape().length2D();
        }
    } else {
        // laneCentralPosition is within of lane shapen, then calculate offset using middlelength
        if ((offsetLaneCentralPositionPerpendicular - middleLength) < extremFrom) {
            laneOffset = firstPosition + extremFrom;
        } else if ((offsetLaneCentralPositionPerpendicular + middleLength) > extremTo) {
            laneOffset = lastPosition - extremTo;
        } else {
            laneOffset = (offsetCentralPosition - offsetLaneCentralPositionPerpendicular);
        }
    }
    return laneOffset;
}


void
GNEMoveElement::calculateMoveResult(GNEMoveResult& moveResult, const GNEViewNet* viewNet, const GNELane* lane,
                                    const double pos, const GNEMoveOffset& offset, const double extremFrom, const double extremTo) {
    // get lane offset
    const double laneOffset = calculateLaneOffset(viewNet, lane, pos, pos, offset, extremFrom, extremTo);
    // update moveResult
    moveResult.newFirstPos = (pos - laneOffset) / lane->getLengthGeometryFactor();
    moveResult.newLastPos = 0;
}


void
GNEMoveElement::calculateMoveResult(GNEMoveResult& moveResult, const GNEViewNet* viewNet, const GNELane* lane,
                                    const double firstPos, const double lastPos, const GNEMoveOffset& offset) {
    // get lane offset
    const double laneOffset = calculateLaneOffset(viewNet, lane, firstPos, lastPos, offset, 0, lane->getLaneShape().length2D());
    // update moveResult
    moveResult.newFirstPos = (firstPos - laneOffset) / lane->getLengthGeometryFactor();
    moveResult.newLastPos = (lastPos - laneOffset) / lane->getLengthGeometryFactor();
}


void
GNEMoveElement::calculateMoveResult(GNEMoveResult& moveResult, const GNEViewNet* viewNet, const GNELane* firstLane,
                                    const double firstPos, const GNELane* lastLane, const double lastPos, const GNEMoveOffset& offset) {
    // get lane offset of the first lane
    const double laneOffset = calculateLaneOffset(viewNet, firstLane, firstPos, firstPos, offset, lastLane->getLaneShape().length2D() - firstPos, firstLane->getLaneShape().length2D());
    // update moveResult
    moveResult.newFirstPos = (firstPos - laneOffset) / firstLane->getLengthGeometryFactor();
    moveResult.newLastPos = (lastPos - laneOffset) / firstLane->getLengthGeometryFactor();
}


void
GNEMoveElement::calculateNewLaneChange(const GNEViewNet* viewNet, const GNELane* originalLane, const GNELane*& newLane, double& laneOffset) {
    // get cursor position
    const Position cursorPosition = viewNet->getPositionInformation();
    // iterate over edge lanes
    for (const auto& lane : originalLane->getParentEdge()->getChildLanes()) {
        // avoid moveOperation lane
        if (lane != originalLane) {
            // calculate offset over lane shape
            const double offSet = lane->getLaneShape().nearest_offset_to_point2D(cursorPosition, true);
            // calculate position over lane shape
            const Position posOverLane = lane->getLaneShape().positionAtOffset2D(offSet);
            // check distance
            if (posOverLane.distanceSquaredTo2D(cursorPosition) < 1) {
                // update newlane
                newLane = lane;
                // calculate offset over moveOperation lane
                const double offsetMoveOperationLane = originalLane->getLaneShape().nearest_offset_to_point2D(cursorPosition, true);
                // calculate position over moveOperation lane
                const Position posOverMoveOperationLane = originalLane->getLaneShape().positionAtOffset2D(offsetMoveOperationLane);
                // update moveResult of laneOffset
                laneOffset = posOverLane.distanceTo2D(posOverMoveOperationLane);
                // change sign of  moveResult laneOffset depending of lane index
                if (originalLane->getIndex() < newLane->getIndex()) {
                    laneOffset *= -1;
                }
            }
        }
    }
}


PositionVector
GNEMoveElement::calculateExtrapolatedVector(const GNEMoveOperation* moveOperation, const GNEMoveResult& moveResult) {
    // get original shape half length
    const double halfLength = moveOperation->originalShape.length2D() * -0.5;
    // get original shape and extrapolate
    PositionVector extendedShape = moveOperation->originalShape;
    extendedShape.extrapolate2D(10e5);
    // get geometry point
    const Position geometryPoint = moveOperation->firstGeometryPoint ? moveResult.shapeToUpdate.front() : moveResult.shapeToUpdate.back();
    // calculate offsets to first and last positions
    const double offset = extendedShape.nearest_offset_to_point2D(geometryPoint, false);
    // calculate extrapolate value
    double extrapolateValue = (10e5 - offset);
    // adjust extrapolation
    if (moveOperation->firstGeometryPoint) {
        if (extrapolateValue < halfLength) {
            extrapolateValue = (halfLength - POSITION_EPS);
        }
    } else {
        if (extrapolateValue > halfLength) {
            extrapolateValue = (halfLength - POSITION_EPS);
        }
    }
    // restore shape in in moveResult
    PositionVector extrapolatedShape = moveOperation->shapeToMove;
    // extrapolate
    extrapolatedShape.extrapolate2D(extrapolateValue);
    // check if return reverse
    if (moveOperation->firstGeometryPoint) {
        return extrapolatedShape;
    } else {
        return extrapolatedShape.reverse();
    }
}

/****************************************************************************/
