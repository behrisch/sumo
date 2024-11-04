# -*- coding: utf-8 -*-
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
# Copyright (C) 2009-2024 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    attributesEnum.py
# @author  Pablo Alvarez Lopez
# @date    2022-01-31

# --------------------------------
# GENERAL
# --------------------------------


class editElements:
    overlapped = 5


class selection:
    class lockSelectionNetwork:
        junctions = 1
        edges = 2
        lanes = 3
        connections = 4
        crossings = 5
        walkingAreas = 6
        additionals = 7
        tazs = 8
        wires = 9
        polygons = 10
        pois = 11
        walkableAreas = 12
        obstacles = 13
        selected = 14
        lockAll = 15
        unlockAll = 16


class modes:
    class network:
        grid = '1'
        junctionShape = '2'
        spreadVehicle = '3'
        showDemandElements = '4'
        selectLane = '5'
        showConnections = '6'
        showSubAdditionals = '7'
    # create edge
        chainMode = '5'
        twoWayMode = '6'
    # TLS mode
        applyAllPhases = '5'
    # move mode
        mergingJunction = '5'
        showBubbles = '6'
        moveElevation = '7'
    # select mode
        automaticSelectJunctions = '9'

    class demand:
        grid = '1'
        junctionShape = '2'
        spreadVehicle = '3'
        showShapes = '4'
        showAllTrips = '5'
        showPersonPlans = '6'
        lockPerson = '7'
        showContainerPlans = '8'
        lockContainer = '9'
        showNonInspected = '10'
        showOverlappedRoutes = '11'

    class data:
        junctionShape = '1'
        showAdditionals = '2'
        showShapes = '3'
        showDemandElements = '4'
        TAZRelDrawingMode = '5'
        TAZFill = '6'
        TAZRelOnlyFrom = '7'
        TAZRelOnlyTo = '8'


class frames:
    class route:
        create = 15
        abort = 16
        removeLastInsertedEdge = 17


class fix:
    class demandElements:
        class routes:
            removeStops = 1
            selectInvalidRoutes = 2
            saveInvalidRoutes = 3

# --------------------------------
# NETWORK
# --------------------------------

# junction


class junction:
    class inspect:
        id = 1
        pos = 2
        type = 3
        shape = 5
        radius = 6
        keepClear = 7
        rightOfWay = 8
        fringe = 10
        name = 12
        parameters = 15

    class inspectTLS:
        id = 1
        pos = 2
        type = 3
        shape = 5
        radius = 6
        keepClear = 7
        rightOfWay = 8
        fringe = 10
        name = 12
        tlType = 13
        tlLayout = 15
        tl = 17
        parameters = 20

    class inspectSelection:
        type = 1
        radius = 3
        keepClear = 4
        rightOfWay = 5
        fringe = 7
        name = 9
        parameters = 12

    class inspectSelectionTLS:
        type = 1
        radius = 3
        keepClear = 4
        rightOfWay = 5
        fringe = 7
        name = 9
        tlType = 10
        tlLayout = 12
        tl = 14
        parameters = 17

# edge


class edge:
    class create:
        defaultShort = 3
        disallowPedestrians = 4
        addSidewalk = 5
        addBikelane = 6
        useTemplate = 7
        numLanes = 9
        speed = 10
        allowButton = 10
        allow = 12
        disallow = 13
        spreadType = 14
        priority = 16
        width = 17
        sidewalkWidth = 18
        bikelaneWidth = 19

    class inspect:
        id = 1
        fromEdge = 2
        toEdge = 3
        speed = 4
        priority = 5
        numLanes = 6
        type = 7
        allowButton = 8
        allow = 9
        disallow = 10
        shape = 11
        length = 12
        spreadType = 13
        name = 15
        width = 16
        endOffset = 17
        shapeStart = 18
        shapeEnd = 19
        distance = 20
        stopOffset = 21
        stopOffsetExceptionButton = 22
        stopOffsetException = 23
        parameters = 24

    class inspectSelection:
        speed = 1
        priority = 2
        numLanes = 3
        type = 4
        allowButton = 5
        allow = 6
        disallow = 7
        length = 8
        spreadType = 9
        name = 11
        width = 12
        endOffset = 13
        shapeStart = 14
        shapeEnd = 15
        stopOffset = 16
        stopOffsetExceptionButton = 17
        stopOffsetException = 18
        parameters = 21

    class template:
        create = 30
        copy = 31
        clear = 32

# lane


class lane:
    class create:
        add = 23
        remove = 24
        speed = 25
        allowButton = 25
        allow = 27
        disallow = 28
        width = 29

    class inspect:
        speed = 1
        allowButton = 2
        allow = 3
        disallow = 4
        width = 5
        endOffset = 6
        acceleration = 7
        customShape = 8
        opposite = 9
        changeLeftButton = 10
        changeLeft = 11
        changeRightButton = 12
        changeRight = 13
        type = 14
        stopOffset = 15
        stopOffsetExceptionButton = 16
        stopOffsetException = 17
        parameters = 18

    class inspectSelection:
        speed = 1
        allowButton = 2
        allow = 3
        disallow = 4
        width = 5
        endOffset = 6
        acceleration = 7
        changeLeftButton = 8
        changeLeft = 9
        changeRightButton = 10
        changeRight = 11
        type = 12
        stopOffset = 13
        stopOffsetExceptionButton = 14
        stopOffsetException = 15
        parameters = 18

# connection


class connection:
    class inspect:
        passC = 1
        keepClear = 2
        contPos = 3
        uncontrolled = 4
        visibility = 5
        allowButton = 6
        allow = 7
        disallow = 8
        speed = 9
        length = 10
        customShape = 11
        changeLeftButton = 12
        changeLeft = 13
        changeRightButton = 14
        changeRight = 15
        indirect = 16
        type = 17
        parameters = 20

    class inspectTLS:
        passC = 1
        keepClear = 2
        contPos = 3
        uncontrolled = 4
        visibility = 5
        linkIndex = 6
        linkIndex2 = 7
        allowButton = 8
        allow = 9
        disallow = 10
        speed = 11
        length = 12
        customShape = 13
        changeLeftButton = 14
        changeLeft = 15
        changeRightButton = 16
        changeRight = 17
        indirect = 18
        type = 19
        parameters = 22

    class inspectSelection:
        passC = 1
        keepClear = 2
        contPos = 3
        uncontrolled = 4
        visibility = 5
        allowButton = 6
        allow = 7
        disallow = 8
        speed = 9
        length = 10
        changeLeftButton = 11
        changeLeft = 12
        changeRightButton = 13
        changeRight = 14
        indirect = 15
        type = 16
        parameters = 19

    class inspectSelectionTLS:
        passC = 1
        keepClear = 2
        contPos = 3
        uncontrolled = 4
        visibility = 5
        linkIndex = 6
        linkIndex2 = 7
        allowButton = 8
        allow = 9
        disallow = 10
        speed = 11
        length = 12
        changeLeftButton = 13
        changeLeft = 14
        changeRightButton = 15
        changeRight = 16
        indirect = 17
        type = 18
        parameters = 21

    saveConnections = 4

# crossing


class crossing:
    class create:
        edges = 1
        priority = 2
        width = 3
        button = 10

    class createTLS:
        edges = 1
        width = 2
        button = 10

    class inspect:
        edges = 1
        priority = 2
        width = 3
        customShape = 4
        parameters = 7

    class inspectTLS:
        edges = 1
        priority = 2
        width = 3
        linkIndex = 4
        linkIndex2 = 5
        customShape = 6
        parameters = 9

    class inspectSelection:
        priority = 1
        width = 2
        parameters = 5

    class inspectSelectionTLS:
        priority = 1
        width = 2
        linkIndex = 3
        linkIndex2 = 4
        parameters = 7

    clearEdges = 3
    firstField = 4


# TLS

class TLS:
    class common:
        TLID = 1
        TLType = 2

    class single:

        class attributes:
            offset = 12
            parameters = 14

        class staticPhase:
            numPhases = 5
            dur = 17 + (0 * numPhases) + 0
            state = 17 + (1 * numPhases) + 1
            nxt = 17 + (2 * numPhases) + 2
            name = 17 + (3 * numPhases) + 3
            addButton = 17 + (4 * numPhases) + 1
            deleteButton = 17 + (5 * numPhases) + 2
            moveUp = 17 + (6 * numPhases) + 3
            moveDown = 17 + (7 * numPhases) + 3
            cleanStates = 17 + (8 * numPhases) + 1
            groupSignal = 17 + (8 * numPhases) + 2
            addStates = 17 + (8 * numPhases) + 3
            ungroupSignal = 17 + (8 * numPhases) + 4

        class actuatedPhase:
            numPhases = 5
            dur = 18 + (0 * numPhases) + 0
            minD = 18 + (1 * numPhases) + 1
            maxD = 18 + (2 * numPhases) + 2
            state = 18 + (3 * numPhases) + 3
            earlyEnd = 18 + (4 * numPhases) + 0
            latestEnd = 18 + (5 * numPhases) + 1
            nxt = 18 + (6 * numPhases) + 2
            name = 18 + (7 * numPhases) + 3
            addButton = 18 + (8 * numPhases) + 1
            deleteButton = 18 + (9 * numPhases) + 2
            moveUp = 18 + (10 * numPhases) + 3
            moveDown = 18 + (11 * numPhases) + 4
            cleanStates = 18 + (12 * numPhases) + 1
            groupSignal = 18 + (12 * numPhases) + 2
            addStates = 18 + (12 * numPhases) + 3
            ungroupSignal = 18 + (12 * numPhases) + 4

        class delayBasedPhase:
            numPhases = 5
            dur = 18 + (0 * numPhases) + 0
            minD = 18 + (1 * numPhases) + 1
            maxD = 18 + (2 * numPhases) + 2
            state = 18 + (3 * numPhases) + 3
            nxt = 18 + (4 * numPhases) + 0
            name = 18 + (5 * numPhases) + 1
            addButton = 18 + (6 * numPhases) + 2
            deleteButton = 18 + (7 * numPhases) + 3
            moveUp = 18 + (8 * numPhases) + 0
            moveDown = 18 + (9 * numPhases) + 1
            cleanStates = 18 + (10 * numPhases) + 1
            groupSignal = 18 + (10 * numPhases) + 2
            addStates = 18 + (10 * numPhases) + 3
            ungroupSignal = 18 + (10 * numPhases) + 4

        class NEMAPhase:
            numPhases = 4
            dur = 18 + (0 * numPhases) + 0
            minD = 18 + (1 * numPhases) + 1
            maxD = 18 + (2 * numPhases) + 2
            state = 18 + (3 * numPhases) + 3
            vehext = 18 + (4 * numPhases) + 0
            yellow = 18 + (5 * numPhases) + 1
            red = 18 + (6 * numPhases) + 2
            nxt = 18 + (7 * numPhases) + 3
            name = 18 + (8 * numPhases) + 0
            addButton = 18 + (9 * numPhases) + 1
            deleteButton = 18 + (10 * numPhases) + 2
            moveUp = 18 + (11 * numPhases) + 3
            moveDown = 18 + (12 * numPhases) + 0
            cleanStates = 18 + (13 * numPhases) + 1
            groupSignal = 18 + (13 * numPhases) + 2
            addStates = 18 + (13 * numPhases) + 3
            ungroupSignal = 18 + (13 * numPhases) + 4

    class join:
        class attributes:
            offset = 13
            parameters = 15

        class staticPhase:
            dur = 18 + (0 * 2)
            state = 18 + (1 * 2)
            nxt = 18 + (2 * 2)
            name = 18 + (3 * 2)
            addButton = 18 + (5 * 2)
            deleteButton = 18 + (6 * 2)
            moveUp = 18 + (7 * 2)
            moveDown = 18 + (8 * 2)
            cleanStates = 18 + (8 * 2) + 1
            groupSignal = 18 + (8 * 2) + 2
            addStates = 18 + (8 * 2) + 3
            ungroupSignal = 18 + (8 * 2) + 4

        class actuatedPhase:
            dur = 19 + (0 * 2)
            minD = 19 + (1 * 2)
            maxD = 19 + (2 * 2)
            state = 19 + (3 * 2)
            earlyEnd = 19 + (4 * 2)
            latestEnd = 19 + (5 * 2)
            nxt = 19 + (6 * 2)
            name = 19 + (7 * 2)
            addButton = 19 + (9 * 2)
            deleteButton = 19 + (10 * 2)
            moveUp = 19 + (11 * 2)
            moveDown = 19 + (12 * 2)
            cleanStates = 19 + (12 * 2) + 1
            groupSignal = 19 + (12 * 2) + 2
            addStates = 19 + (12 * 2) + 3
            ungroupSignal = 19 + (12 * 2) + 4

        class delayBasedPhase:
            dur = 19 + (0 * 2)
            minD = 19 + (1 * 2)
            maxD = 19 + (2 * 2)
            state = 19 + (3 * 2)
            nxt = 19 + (4 * 2)
            name = 19 + (5 * 2)
            addButton = 19 + (7 * 2)
            deleteButton = 19 + (8 * 2)
            moveUp = 19 + (9 * 2)
            moveDown = 19 + (10 * 2)
            cleanStates = 19 + (10 * 2) + 1
            groupSignal = 19 + (10 * 2) + 2
            addStates = 19 + (10 * 2) + 3
            ungroupSignal = 19 + (10 * 2) + 4

        class NEMAPhase:
            dur = 19 + (0 * 2)
            minD = 19 + (1 * 2)
            maxD = 19 + (2 * 2)
            state = 19 + (3 * 2)
            vehext = 19 + (4 * 2)
            yellow = 19 + (5 * 2)
            red = 19 + (6 * 2)
            nxt = 19 + (7 * 2)
            name = 19 + (8 * 2)
            addButton = 20 + (9 * 2)
            deleteButton = 20 + (10 * 2)
            moveUp = 20 + (11 * 2)
            moveDown = 20 + (12 * 2)
            cleanStates = 20 + (13 * 2) + 1
            groupSignal = 20 + (13 * 2) + 2
            addStates = 20 + (13 * 2) + 3
            ungroupSignal = 20 + (13 * 2) + 4

    create = 3
    createOverlapped = 4
    copySingle = 9
    copyJoined = 10
    joinTLS = 5
    disjoinTLS = 6
    deleteSingle = 10
    resetPhaseSingle = 11
    resetAllSingle = 12
    deleteJoined = 11
    resetPhaseJoined = 12
    resetAllJoined = 13


# --------------------------------
# ADDITIONALS
# --------------------------------

# additionals

class additionals:
    changeElement = 2

# busStop


class busStop:
    class create:
        id = 5
        name = 6
        friendlyPos = 7
        lines = 8
        personCapacity = 9
        parkingLength = 10
        colorButton = 11
        color = 12
        references = 16
        length = 18

    class inspect:
        id = 1
        lane = 2
        moveLane = 3
        startPos = 4
        endPos = 5
        name = 6
        friendlyPos = 7
        lines = 8
        personCapacity = 9
        parkingLength = 10
        colorButton = 11
        color = 12
        parameters = 15

    class inspectSelection:
        name = 1
        friendlyPos = 2
        lines = 3
        personCapacity = 4
        parkingLength = 5
        colorButton = 6
        color = 7
        parameters = 10

# trainStop


class trainStop:
    class create:
        id = 5
        name = 6
        friendlyPos = 7
        lines = 8
        personCapacity = 9
        parkingLength = 10
        colorButton = 11
        color = 12
        references = 16
        length = 18

    class inspect:
        id = 1
        lane = 2
        moveLane = 3
        startPos = 4
        endPos = 5
        name = 6
        friendlyPos = 7
        lines = 8
        personCapacity = 9
        parkingLength = 10
        colorButton = 11
        color = 12
        parameters = 15

    class inspectSelection:
        name = 1
        friendlyPos = 2
        lines = 3
        personCapacity = 4
        parkingLength = 5
        colorButton = 6
        color = 7
        parameters = 10

# containerStop


class containerStop:
    class create:
        id = 5
        name = 6
        friendlyPos = 7
        lines = 8
        containerCapacity = 9
        parkingLength = 10
        colorButton = 11
        color = 12
        references = 16
        length = 18

    class inspect:
        id = 1
        lane = 2
        moveLane = 3
        startPos = 4
        endPos = 5
        name = 6
        friendlyPos = 7
        lines = 8
        containerCapacity = 9
        parkingLength = 10
        colorButton = 11
        color = 12
        parameters = 15

    class inspectSelection:
        name = 1
        friendlyPos = 2
        lines = 3
        containerCapacity = 4
        parkingLength = 5
        colorButton = 6
        color = 7
        parameters = 10

# chargingStation


class chargingStation:
    class create:
        id = 5
        name = 6
        friendlyPos = 7
        power = 8
        efficiency = 9
        chargeInTransit = 10
        chargeDelay = 11
        chargeType = 12
        waitingTime = 14
        parkingArea = 15
        references = 19
        length = 21

    class inspect:
        id = 1
        lane = 2
        moveLane = 3
        startPos = 4
        endPos = 5
        name = 6
        friendlyPos = 7
        power = 8
        efficiency = 9
        chargeInTransit = 10
        chargeDelay = 11
        chargeType = 12
        waitingTime = 14
        parkingArea = 15
        parameters = 18

    class inspectSelection:
        name = 1
        friendlyPos = 2
        power = 3
        efficiency = 4
        chargeInTrainsit = 5
        chargeDelay = 6
        chargeType = 7
        waitingTime = 9
        parkingArea = 10
        parameters = 13

# access


class access:
    class create:
        length = 5
        friendlyPos = 6
        parent = 9

    class inspect:
        lane = 1
        moveLane = 2
        pos = 3
        length = 4
        friendlyPos = 5

    class inspectSelection:
        length = 1
        friendlyPos = 2

# parkingArea


class parkingArea:
    class create:
        id = 5
        departPos = 6
        name = 7
        acceptedBadges = 8
        roadSideCapacity = 9
        onRoad = 10
        friendlyPos = 11
        width = 12
        length = 13
        angle = 14
        lefthand = 15
        references = 19
        referencesLength = 21

    class inspect:
        id = 1
        lane = 2
        moveLane = 3
        startPos = 4
        endPos = 5
        departPos = 6
        name = 7
        acceptedBadges = 8
        roadSideCapacity = 9
        onRoad = 10
        friendlyPos = 11
        width = 12
        length = 13
        angle = 14
        lefthand = 15
        parameters = 18

    class inspectSelection:
        departPos = 1
        name = 2
        acceptedBadges = 3
        roadSideCapacity = 4
        onRoad = 5
        friendlyPos = 6
        width = 7
        length = 8
        angle = 9
        lefthand = 10
        parameters = 13

# parkingSpace


class parkingSpace:
    class create:
        name = 5
        width = 6
        length = 7
        angle = 8
        slope = 9
        parent = 12

    class inspect:
        pos = 1
        name = 2
        width = 3
        length = 4
        angle = 5
        slope = 6
        parameters = 9
        parent = 13

    class inspectSelection:
        name = 1
        width = 2
        length = 3
        angle = 4
        slope = 5
        parameters = 8
        parent = 11

# E1


class E1:
    class create:
        id = 5
        period = 6
        name = 7
        file = 8
        vTypes = 9
        nextEdges = 10
        detectPersons = 11
        friendlyPos = 13

    class inspect:
        id = 1
        lane = 2
        moveLane = 3
        pos = 4
        period = 5
        name = 6
        file = 7
        vTypes = 8
        nextEdges = 9
        detectPersons = 10
        friendlyPos = 12
        parameters = 15

    class inspectSelection:
        period = 1
        name = 2
        file = 3
        vTypes = 4
        nextEdges = 5
        detectPersons = 6
        friendlyPos = 8
        parameters = 11

# E2


class E2:
    class create:
        id = 5
        length = 6
        period = 7
        tl = 8
        name = 9
        file = 10
        vTypes = 11
        nextEdges = 12
        detectPersons = 13
        timeThreshold = 15
        speedThreshold = 16
        jamThreshold = 17
        friendlyPos = 18
        show = 19

    class inspect:
        id = 1
        lane = 2
        moveLane = 3
        pos = 4
        length = 5
        period = 6
        tl = 7
        name = 8
        file = 9
        vTypes = 10
        nextEdges = 11
        detectPersons = 12
        timeThreshold = 14
        speedThreshold = 15
        jamThreshold = 16
        friendlyPos = 17
        show = 18
        parameters = 21

    class inspectSelection:
        length = 1
        period = 2
        tl = 3
        name = 4
        file = 5
        vTypes = 6
        nextEdges = 7
        detectPersons = 8
        timeThreshold = 10
        speedThreshold = 11
        jamThreshold = 12
        friendlyPos = 13
        show = 14
        parameters = 17

# E2Multilane


class E2Multilane:
    class create:
        id = 5
        period = 6
        tl = 7
        name = 8
        file = 9
        vTypes = 10
        nextEdges = 11
        detectPersons = 12
        timeThreshold = 14
        speedThreshold = 15
        jamThreshold = 16
        friendlyPos = 17
        show = 18

    class inspect:
        id = 1
        lanes = 2
        pos = 3
        endPos = 4
        period = 5
        tl = 6
        name = 7
        file = 8
        vTypes = 9
        nextEdges = 10
        detectPersons = 11
        timeThreshold = 13
        speedThreshold = 14
        jamThreshold = 15
        friendlyPos = 16
        show = 17
        parameters = 20

    class inspectSelection:
        period = 1
        tl = 2
        name = 3
        file = 4
        vTypes = 5
        nextEdges = 6
        detectPersons = 7
        timeThreshold = 9
        speedThreshold = 10
        jamThreshold = 11
        friendlyPos = 12
        show = 13
        parameters = 16

# E3


class E3:
    class create:
        id = 5
        period = 6
        name = 7
        file = 8
        vTypes = 9
        nextEdges = 10
        detectPersons = 11
        openEntry = 13
        timeThreshold = 14
        speedThreshold = 15
        expectArrival = 16

    class inspect:
        id = 1
        pos = 2
        period = 3
        name = 4
        file = 5
        vTypes = 6
        nextEdges = 7
        detectPersons = 8
        openEntry = 10
        timeThreshold = 11
        speedThreshold = 12
        expectArrival = 13
        parameters = 16

    class inspectSelection:
        period = 1
        name = 2
        file = 3
        vTypes = 4
        nextEdges = 5
        detectPersons = 6
        openEntry = 8
        timeThreshold = 9
        speedThreshold = 10
        expectArrival = 11
        parameters = 14

# entryExit


class entryExit:
    class create:
        friendlyPos = 5
        parent = 8

    class inspect:
        lane = 1
        moveLane = 2
        pos = 3
        friendlyPos = 4
        parameters = 7
        parent = 11

    class inspectSelection:
        friendlyPos = 1
        parameters = 4
        parent = 7

# E1Instant


class E1Instant:
    class create:
        id = 5
        name = 6
        file = 7
        vTypes = 8
        nextEdges = 9
        detectPersons = 10
        friendlyPos = 12

    class inspect:
        id = 1
        lane = 2
        moveLane = 3
        pos = 4
        name = 5
        file = 6
        vTypes = 7
        nextEdges = 8
        detectPersons = 9
        friendlyPos = 11
        parameters = 14

    class inspectSelection:
        name = 1
        file = 2
        vTypes = 3
        nextEdges = 4
        detectPersons = 5
        friendlyPos = 7
        parameters = 10

# calibrator


class calibrator:
    class create:
        id = 5
        pos = 6
        frequency = 7
        name = 8
        routeProbe = 9
        output = 10
        jamTreshold = 11
        vTypes = 12
        center = 16

    class inspect:
        id = 1
        edge = 2
        pos = 3
        frequency = 4
        name = 5
        routeProbe = 6
        output = 7
        jamTreshold = 8
        vTypes = 9
        parameters = 12

    class inspectLane:
        id = 1
        lane = 2
        laneMoveUp = 3
        pos = 4
        frequency = 5
        name = 6
        routeProbe = 7
        output = 8
        jamTreshold = 9
        vTypes = 10
        parameters = 13

    class inspectSelection:
        pos = 1
        frequency = 2
        name = 3
        routeProbe = 4
        output = 5
        jamTreshold = 6
        vTypes = 7
        parameters = 10

# routeProbe


class routeProbe:
    class create:
        id = 5
        frequency = 6
        name = 7
        file = 8
        begin = 9
        center = 13

    class inspect:
        id = 1
        edge = 2
        frequency = 3
        name = 4
        file = 5
        begin = 6
        parameters = 9

    class inspectSelection:
        frequency = 1
        name = 2
        file = 3
        begin = 4
        parameters = 7

# vaporizer


class vaporizer:
    class create:
        begin = 5
        end = 6
        name = 7
        center = 11

    class inspect:
        edge = 1
        begin = 2
        end = 3
        name = 4
        parameters = 7

    class inspectSelection:
        begin = 1
        end = 2
        name = 3
        parameters = 6

# --------------------------------
# SHAPE ELEMENTS
# --------------------------------

# Poly


class poly:
    class create:
        id = 5
        colorButton = 6
        color = 7
        fill = 8
        lineWidth = 9
        layer = 10
        type = 11
        name = 12
        imgFile = 13
        reativePath = 14
        angle = 15
        closeShape = 19

    class inspect:
        id = 1
        shape = 2
        colorButton = 3
        color = 4
        fill = 5
        lineWidth = 6
        layer = 7
        type = 8
        name = 9
        imgFile = 10
        reativePath = 11
        angle = 12
        geoshape = 15
        geo = 16
        parameters = 19
        closeShape = 23

    class inspectSelection:
        colorButton = 1
        color = 2
        fill = 3
        lineWidth = 4
        layer = 5
        type = 6
        name = 7
        imgFile = 8
        reativePath = 9
        angle = 10
        geo = 13
        parameters = 16
        closeShape = 19

# POI


class poi:
    class create:
        id = 5
        colorButton = 6
        color = 7
        type = 8
        icon = 9
        name = 11
        layer = 12
        width = 13
        height = 14
        imgFile = 15
        relativePath = 16
        angle = 17

    class inspect:
        id = 1
        pos = 2
        colorButton = 3
        color = 4
        type = 5
        icon = 6
        name = 8
        layer = 9
        width = 10
        height = 11
        imgFile = 12
        relativePath = 13
        angle = 14
        parameters = 17

    class inspectSelection:
        colorButton = 1
        color = 2
        type = 3
        icon = 4
        name = 6
        layer = 7
        width = 8
        height = 9
        imgFile = 10
        relativePath = 11
        angle = 12
        parameters = 15

# POIGEO


class poiGeo:
    class create:
        id = 5
        colorButton = 6
        color = 7
        type = 8
        icon = 9
        name = 11
        layer = 12
        width = 13
        height = 14
        imgFile = 15
        relativePath = 16
        angle = 17
        lonLat = 22

    class inspect:
        id = 1
        lon = 2
        lat = 3
        colorButton = 4
        color = 5
        type = 6
        icon = 7
        name = 9
        layer = 10
        width = 11
        height = 12
        imgFile = 13
        relativePath = 14
        angle = 15
        parameters = 18

    class inspectSelection:
        colorButton = 1
        color = 2
        type = 3
        icon = 4
        name = 6
        layer = 7
        width = 8
        height = 9
        imgFile = 10
        relativePath = 11
        angle = 12
        parameters = 15

# POILane


class poiLane:
    class create:
        id = 5
        friendlyPos = 6
        posLat = 7
        colorButton = 8
        color = 9
        type = 10
        icon = 11
        name = 13
        layer = 14
        width = 15
        height = 16
        imgFile = 17
        relativePath = 18
        angle = 16

    class inspect:
        id = 1
        lane = 2
        moveLane = 3
        pos = 4
        friendlyPos = 5
        posLat = 6
        colorButton = 7
        color = 8
        type = 9
        icon = 10
        name = 12
        layer = 13
        width = 14
        height = 15
        imgFile = 16
        relativePath = 17
        angle = 18
        parameters = 21

    class inspectSelection:
        friendlyPos = 1
        posLat = 2
        colorButton = 3
        color = 4
        type = 5
        icon = 6
        name = 8
        layer = 9
        width = 10
        height = 11
        imgFile = 12
        relativePath = 13
        angle = 14
        parameters = 17

# jps walkableArea


class jpsWalkableArea:
    class create:
        id = 5
        name = 6

    class inspect:
        id = 1
        shape = 2
        name = 3
        parameters = 6

    class inspectSelection:
        name = 1
        parameters = 4

# jps obstacle


class jpsObstacle:
    class create:
        id = 5
        name = 6

    class inspect:
        id = 1
        shape = 2
        name = 3
        parameters = 6

    class inspectSelection:
        name = 1
        parameters = 4

# --------------------------------
# TAZ ELEMENTS
# --------------------------------

# TAZ


class TAZ:
    class create:
        center = 3
        fill = 4
        colorButton = 5
        color = 6
        name = 7
        edgesWithin = 8

    class inspect:
        id = 1
        shape = 2
        center = 3
        fill = 4
        colorButton = 5
        color = 6
        name = 7
        parameters = 10

    class inspectSelection:
        center = 1
        fill = 2
        colorButton = 3
        color = 4
        name = 5
        parameters = 8

# --------------------------------
# DEMAND ELEMENTS
# --------------------------------

# type


class type:

    color = 5

    class buttons:
        create = 2
        delete = 3
        copy = 3
        dialog = 25

    class edit:
        id = 8
        typeDist = 8
        vClass = 9
        colorButton = 11
        color = 12
        length = 13
        minGap = 14
        maxSpeed = 15
        desiredMaxSpeed = 16
        parkingBadges = 17
        accel = 18
        decel = 19
        sigma = 20
        tau = 21

    class editDefault:
        vClass = 7
        colorButton = 9
        color = 10
        length = 11
        minGap = 12
        maxSpeed = 13
        desiredMaxSpeed = 14
        parkingBadges = 15
        accel = 16
        decel = 17
        sigma = 18
        tau = 19

    class editDialog:
        vClass = 1
        id = 3
        colorButton = 4
        color = 5
        length = 6
        minGap = 7
        maxSpeed = 8
        desiredMaxSpeed = 9
        speedFactor = 10
        emissionClass = 11
        width = 13
        height = 14
        imgFileButton = 15
        imgFile = 16
        osgFileButton = 17
        osgFile = 18
        laneChangeModel = 19
        guiShape = 21
        probability = 23
        personCapacity = 24
        containerCapacity = 25
        boardingDuration = 26
        loadingDuration = 27
        latAlignment = 28
        minGapLat = 30
        maxSpeedLat = 31
        actionStepLength = 32
        carriageLength = 33
        locomotiveLength = 34
        carriageGap = 35
        parametersButton = 36
        parameters = 37

# route


class route:
    class create:
        id = 7
        colorButton = 8
        color = 9
        repeat = 10
        cycletime = 11

    class inspect:
        id = 1
        edges = 2
        colorButton = 3
        color = 4
        repeat = 5
        cycletime = 6
        parameters = 9

    class inspectSelection:
        colorButton = 1
        color = 2
        repeat = 3
        cycletime = 4
        parameters = 7

# trip (from-to edges)


class trip:
    class create:
        type = 5
        id = 8
        colorButton = 9
        color = 10
        departLane = 11
        departPos = 12
        departSpeed = 13
        arrivalLane = 14
        arrivalPos = 15
        arrivalSpeed = 16
        line = 17
        personNumber = 18
        containerNumber = 19
        departPosLat = 20
        arrivalPosLat = 21
        insertionChecks = 22
        depart = 23

    class inspect:
        id = 1
        typeButton = 2
        type = 3
        fromEdge = 5
        toEdge = 6
        via = 7
        colorButton = 8
        color = 9
        departLane = 10
        departPos = 11
        departSpeed = 12
        arrivalLane = 13
        arrivalPos = 14
        arrivalSpeed = 15
        line = 16
        personNumber = 17
        containerNumber = 18
        departPosLat = 19
        arrivalPosLat = 20
        insertionChecks = 21
        depart = 22
        parameters = 25

    class inspectSelection:
        type = 1
        colorButton = 3
        color = 4
        departLane = 5
        departPos = 6
        departSpeed = 7
        arrivalLane = 8
        arrivalPos = 9
        arrivalSpeed = 10
        line = 11
        personNumber = 12
        containerNumber = 13
        departPosLat = 14
        arrivalPosLat = 15
        insertionChecks = 16
        depart = 17
        parameters = 20

# trip (from-to junctions)


class tripJunction:
    class create:
        type = 5
        id = 8
        colorButton = 9
        color = 10
        departLane = 11
        departPos = 12
        departSpeed = 13
        arrivalLane = 14
        arrivalPos = 15
        arrivalSpeed = 16
        line = 17
        personNumber = 18
        containerNumber = 19
        departPosLat = 20
        arrivalPosLat = 21
        insertionChecks = 22
        depart = 23

    class inspect:
        id = 1
        typeButton = 2
        type = 3
        fromJunction = 5
        toJunction = 6
        colorButton = 7
        color = 8
        departLane = 9
        departPos = 10
        departSpeed = 11
        arrivalLane = 12
        arrivalPos = 13
        arrivalSpeed = 14
        line = 15
        personNumber = 16
        containerNumber = 17
        departPosLat = 18
        arrivalPosLat = 19
        insertionChecks = 20
        depart = 21
        parameters = 24

    class inspectSelection:
        type = 1
        colorButton = 3
        color = 4
        departLane = 5
        departPos = 6
        departSpeed = 7
        arrivalLane = 8
        arrivalPos = 9
        arrivalSpeed = 10
        line = 11
        personNumber = 12
        containerNumber = 13
        departPosLat = 14
        arrivalPosLat = 15
        insertionChecks = 16
        depart = 17
        parameters = 20

# trip (from-to TAZs)


class tripTAZ:
    class create:
        type = 5
        id = 8
        colorButton = 9
        color = 10
        departLane = 11
        departPos = 12
        departSpeed = 13
        arrivalLane = 14
        arrivalPos = 15
        arrivalSpeed = 16
        line = 17
        personNumber = 18
        containerNumber = 19
        departPosLat = 20
        arrivalPosLat = 21
        insertionChecks = 22
        depart = 23

    class inspect:
        id = 1
        typeButton = 2
        type = 3
        fromTAZ = 5
        toTAZ = 6
        colorButton = 7
        color = 8
        departLane = 9
        departPos = 10
        departSpeed = 11
        arrivalLane = 12
        arrivalPos = 13
        arrivalSpeed = 14
        line = 15
        personNumber = 16
        containerNumber = 17
        departPosLat = 18
        arrivalPosLat = 19
        insertionChecks = 20
        depart = 21
        parameters = 24

    class inspectSelection:
        type = 1
        colorButton = 3
        color = 4
        departLane = 5
        departPos = 6
        departSpeed = 7
        arrivalLane = 8
        arrivalPos = 9
        arrivalSpeed = 10
        line = 11
        personNumber = 12
        containerNumber = 13
        departPosLat = 14
        arrivalPosLat = 15
        insertionChecks = 16
        depart = 17
        parameters = 20

# vehicle (over route)


class vehicle:
    class create:
        type = 5
        id = 8
        colorButton = 9
        color = 10
        departLane = 11
        departPos = 12
        departSpeed = 13
        arrivalLane = 14
        arrivalPos = 15
        arrivalSpeed = 16
        line = 17
        personNumber = 18
        containerNumber = 19
        departPosLat = 20
        arrivalPosLat = 21
        insertionChecks = 22
        depart = 23

    class inspect:
        id = 1
        typeButton = 2
        type = 3
        route = 5
        departEdge = 6
        arrivalEdge = 7
        colorButton = 8
        color = 9
        departLane = 10
        departPos = 11
        departSpeed = 12
        arrivalLane = 13
        arrivalPos = 14
        arrivalSpeed = 15
        line = 16
        personNumber = 17
        containerNumber = 18
        departPosLat = 19
        arrivalPosLat = 20
        insertionChecks = 21
        depart = 22
        parameters = 25

    class inspectSelection:
        type = 1
        colorButton = 3
        color = 4
        departLane = 5
        departPos = 6
        departSpeed = 7
        arrivalLane = 8
        arrivalPos = 9
        arrivalSpeed = 10
        line = 11
        personNumber = 12
        containerNumber = 13
        departPosLat = 14
        arrivalPosLat = 15
        insertionChecks = 16
        depart = 17
        parameters = 20

# vehicle (embedded route)


class vehicleEmbedded:
    class create:
        type = 5
        id = 8
        colorButton = 9
        color = 10
        departLane = 11
        departPos = 12
        departSpeed = 13
        arrivalLane = 14
        arrivalPos = 15
        arrivalSpeed = 16
        line = 17
        personNumber = 18
        containerNumber = 19
        departPosLat = 20
        arrivalPosLat = 21
        insertionChecks = 22
        depart = 23

    class inspect:
        id = 1
        typeButton = 2
        type = 3
        departEdge = 5
        arrivalEdge = 6
        colorButton = 7
        color = 8
        departLane = 9
        departPos = 10
        departSpeed = 11
        arrivalLane = 12
        arrivalPos = 13
        arrivalSpeed = 14
        line = 15
        personNumber = 16
        containerNumber = 17
        departPosLat = 18
        arrivalPosLat = 19
        insertionChecks = 20
        depart = 21
        parameters = 24

    class inspectSelection:
        type = 1
        colorButton = 3
        color = 4
        departLane = 5
        departPos = 6
        departSpeed = 7
        arrivalLane = 8
        arrivalPos = 9
        arrivalSpeed = 10
        line = 11
        personNumber = 12
        containerNumber = 13
        departPosLat = 14
        arrivalPosLat = 15
        insertionChecks = 16
        depart = 17
        parameters = 20

# flow (from-to edges)


class flow:
    class create:
        type = 5
        id = 8
        colorButton = 9
        color = 10
        departLane = 11
        departPos = 12
        departSpeed = 13
        arrivalLane = 14
        arrivalPos = 15
        arrivalSpeed = 16
        line = 17
        personNumber = 18
        containerNumber = 19
        departPosLat = 20
        arrivalPosLat = 21
        insertionChecks = 22
        begin = 23
        terminate = 27
        end = 29
        number = 30
        spacing = 29
        terminateOption = 31
        spacingOption = 32

    class inspect:
        id = 1
        typeButton = 2
        type = 3
        fromEdge = 5
        toEdge = 6
        via = 7
        colorButton = 8
        color = 9
        departLane = 10
        departPos = 11
        departSpeed = 12
        arrivalLane = 13
        arrivalPos = 14
        arrivalSpeed = 15
        line = 16
        personNumber = 17
        containerNumber = 18
        departPosLat = 19
        arrivalPosLat = 20
        insertionChecks = 21
        begin = 22
        terminate = 25
        spacing = 27
        end = 28
        terminateOption = 29
        spacingOption = 30
        parameters = 32

    class inspectSelection:
        type = 1
        colorButton = 3
        color = 4
        departLane = 5
        departPos = 6
        departSpeed = 7
        arrivalLane = 8
        arrivalPos = 9
        arrivalSpeed = 10
        line = 11
        personNumber = 12
        containerNumber = 13
        departPosLat = 14
        arrivalPosLat = 15
        insertionChecks = 16
        begin = 17
        terminate = 20
        spacing = 22
        terminateOption = 24
        spacingOption = 25
        parameters = 25

# flow (from-to junctions)


class flowJunction:
    class create:
        type = 5
        id = 8
        colorButton = 9
        color = 10
        departLane = 11
        departPos = 12
        departSpeed = 13
        arrivalLane = 14
        arrivalPos = 15
        arrivalSpeed = 16
        line = 17
        personNumber = 18
        containerNumber = 19
        departPosLat = 20
        arrivalPosLat = 21
        insertionChecks = 22
        begin = 23
        terminate = 27
        end = 29
        number = 30
        spacing = 29
        terminateOption = 31
        spacingOption = 32

    class inspect:
        id = 1
        typeButton = 2
        type = 3
        fromJunction = 5
        toJunction = 6
        colorButton = 7
        color = 8
        departLane = 9
        departPos = 10
        departSpeed = 11
        arrivalLane = 12
        arrivalPos = 13
        arrivalSpeed = 14
        line = 15
        personNumber = 16
        containerNumber = 17
        departPosLat = 18
        arrivalPosLat = 19
        insertionChecks = 20
        begin = 21
        terminate = 24
        spacing = 26
        end = 27
        terminateOption = 28
        spacingOption = 29
        parameters = 31

    class inspectSelection:
        type = 1
        colorButton = 3
        color = 4
        departLane = 5
        departPos = 6
        departSpeed = 7
        arrivalLane = 8
        arrivalPos = 9
        arrivalSpeed = 10
        line = 11
        personNumber = 12
        containerNumber = 13
        departPosLat = 14
        arrivalPosLat = 15
        insertionChecks = 16
        begin = 17
        terminate = 20
        spacing = 22
        terminateOption = 24
        spacingOption = 25
        parameters = 25

# flow (from-to TAZs)


class flowTAZ:
    class create:
        type = 5
        id = 8
        colorButton = 9
        color = 10
        departLane = 11
        departPos = 12
        departSpeed = 13
        arrivalLane = 14
        arrivalPos = 15
        arrivalSpeed = 16
        line = 17
        personNumber = 18
        containerNumber = 19
        departPosLat = 20
        arrivalPosLat = 21
        insertionChecks = 22
        begin = 23
        terminate = 27
        end = 29
        number = 30
        spacing = 29
        terminateOption = 31
        spacingOption = 32

    class inspect:
        id = 1
        typeButton = 2
        type = 3
        fromTAZ = 5
        toTAZ = 6
        colorButton = 7
        color = 8
        departLane = 9
        departPos = 10
        departSpeed = 11
        arrivalLane = 12
        arrivalPos = 13
        arrivalSpeed = 14
        line = 15
        personNumber = 16
        containerNumber = 17
        departPosLat = 18
        arrivalPosLat = 19
        insertionChecks = 20
        begin = 21
        terminate = 24
        spacing = 26
        end = 27
        terminateOption = 28
        spacingOption = 29
        parameters = 31

    class inspectSelection:
        type = 1
        colorButton = 3
        color = 4
        departLane = 5
        departPos = 6
        departSpeed = 7
        arrivalLane = 8
        arrivalPos = 9
        arrivalSpeed = 10
        line = 11
        personNumber = 12
        containerNumber = 13
        departPosLat = 14
        arrivalPosLat = 15
        insertionChecks = 16
        begin = 17
        terminate = 20
        spacing = 22
        terminateOption = 24
        spacingOption = 25
        parameters = 25

# flow (over route)


class routeFlow:
    class create:
        type = 5
        id = 8
        colorButton = 9
        color = 10
        departLane = 11
        departPos = 12
        departSpeed = 13
        arrivalLane = 14
        arrivalPos = 15
        arrivalSpeed = 16
        line = 17
        personNumber = 18
        containerNumber = 19
        departPosLat = 20
        arrivalPosLat = 21
        insertionChecks = 22
        begin = 23
        terminate = 27
        end = 29
        number = 30
        spacing = 29
        terminateOption = 31
        spacingOption = 32

    class inspect:
        id = 1
        typeButton = 2
        type = 3
        route = 5
        departEdge = 6
        arrivalEdge = 7
        colorButton = 8
        color = 9
        departLane = 10
        departPos = 11
        departSpeed = 12
        arrivalLane = 13
        arrivalPos = 14
        arrivalSpeed = 15
        line = 16
        personNumber = 17
        containerNumber = 18
        departPosLat = 19
        arrivalPosLat = 20
        insertionChecks = 21
        begin = 22
        terminate = 25
        spacing = 27
        end = 28
        terminateOption = 29
        spacingOption = 30
        parameters = 32

    class inspectSelection:
        type = 1
        colorButton = 3
        color = 4
        departLane = 5
        departPos = 6
        departSpeed = 7
        arrivalLane = 8
        arrivalPos = 9
        arrivalSpeed = 10
        line = 11
        personNumber = 12
        containerNumber = 13
        departPosLat = 14
        arrivalPosLat = 15
        insertionChecks = 16
        begin = 17
        terminate = 20
        spacing = 22
        terminateOption = 24
        spacingOption = 25
        parameters = 25

# flow (embedded route)


class flowEmbedded:
    class create:
        type = 5
        id = 8
        colorButton = 9
        color = 10
        departLane = 11
        departPos = 12
        departSpeed = 13
        arrivalLane = 14
        arrivalPos = 15
        arrivalSpeed = 16
        line = 17
        personNumber = 18
        containerNumber = 19
        departPosLat = 20
        arrivalPosLat = 21
        insertionChecks = 22
        begin = 23
        terminate = 27
        end = 29
        number = 30
        spacing = 29
        terminateOption = 31
        spacingOption = 32

    class inspect:
        id = 1
        typeButton = 2
        type = 3
        departEdge = 5
        arrivalEdge = 6
        colorButton = 7
        color = 8
        departLane = 9
        departPos = 10
        departSpeed = 11
        arrivalLane = 12
        arrivalPos = 13
        arrivalSpeed = 14
        line = 15
        personNumber = 16
        containerNumber = 17
        departPosLat = 18
        arrivalPosLat = 19
        insertionChecks = 20
        begin = 21
        terminate = 24
        spacing = 26
        end = 27
        terminateOption = 28
        spacingOption = 29
        parameters = 31

    class inspectSelection:
        type = 1
        colorButton = 3
        color = 4
        departLane = 5
        departPos = 6
        departSpeed = 7
        arrivalLane = 8
        arrivalPos = 9
        arrivalSpeed = 10
        line = 11
        personNumber = 12
        containerNumber = 13
        departPosLat = 14
        arrivalPosLat = 15
        insertionChecks = 16
        begin = 17
        terminate = 20
        spacing = 22
        terminateOption = 24
        spacingOption = 25
        parameters = 25

# stop over bus stop


class stopBusStop:
    class create:
        durationEnable = 8
        duration = 9
        untilEnable = 9
        until = 10
        extensionEnable = 10
        extension = 11
        triggered = 12
        expected = 14
        join = 14
        permitted = 14
        parking = 15
        actType = 17
        tripID = 18
        line = 19
        onDemand = 20
        jump = 21
        split = 22

    class inspect:
        durationEnable = 1
        duration = 2
        untilEnable = 2
        until = 3
        extensionEnable = 3
        extension = 4
        triggered = 5
        expected = 7
        join = 7
        permitted = 7
        parking = 8
        actType = 10
        tripID = 11
        line = 12
        onDemand = 13
        jump = 14
        split = 15

    class inspectSelection:
        durationEnable = 1
        duration = 2
        untilEnable = 3
        until = 4
        extensionEnable = 5
        extension = 6
        triggered = 7
        expected = 9
        join = 9
        permitted = 10
        parking = 11
        actType = 13
        tripID = 14
        line = 15
        onDemand = 16
        jump = 17
        split = 18

# stop over train stop


class stopTrainStop:
    class create:
        durationEnable = 8
        duration = 9
        untilEnable = 9
        until = 10
        extensionEnable = 10
        extension = 11
        triggered = 12
        expected = 14
        join = 14
        permitted = 14
        parking = 15
        actType = 17
        tripID = 18
        line = 19
        onDemand = 20
        jump = 21
        split = 22

    class inspect:
        durationEnable = 1
        duration = 2
        untilEnable = 2
        until = 3
        extensionEnable = 3
        extension = 4
        triggered = 5
        expected = 7
        join = 7
        permitted = 7
        parking = 8
        actType = 10
        tripID = 11
        line = 12
        onDemand = 13
        jump = 14
        split = 15

    class inspectSelection:
        durationEnable = 1
        duration = 2
        untilEnable = 3
        until = 4
        extensionEnable = 5
        extension = 6
        triggered = 7
        expected = 9
        join = 9
        permitted = 10
        parking = 11
        actType = 13
        tripID = 14
        line = 15
        onDemand = 16
        jump = 17
        split = 18

# stop over container stop


class stopContainerStop:
    class create:
        durationEnable = 8
        duration = 9
        untilEnable = 9
        until = 10
        extensionEnable = 10
        extension = 11
        triggered = 12
        expected = 14
        join = 14
        permitted = 14
        parking = 15
        actType = 17
        tripID = 18
        line = 19
        onDemand = 20
        jump = 21
        split = 22

    class inspect:
        durationEnable = 1
        duration = 2
        untilEnable = 2
        until = 3
        extensionEnable = 3
        extension = 4
        triggered = 5
        expected = 7
        join = 7
        permitted = 7
        parking = 8
        actType = 10
        tripID = 11
        line = 12
        onDemand = 13
        jump = 14
        split = 15

    class inspectSelection:
        durationEnable = 1
        duration = 2
        untilEnable = 3
        until = 4
        extensionEnable = 5
        extension = 6
        triggered = 7
        expected = 9
        join = 9
        permitted = 10
        parking = 11
        actType = 13
        tripID = 14
        line = 15
        onDemand = 16
        jump = 17
        split = 18

# stop over charging station


class stopChargingStation:
    class create:
        durationEnable = 8
        duration = 9
        untilEnable = 9
        until = 10
        extensionEnable = 10
        extension = 11
        triggered = 12
        expected = 14
        join = 14
        permitted = 14
        parking = 15
        actType = 17
        tripID = 18
        line = 19
        onDemand = 20
        jump = 21
        split = 22

    class inspect:
        durationEnable = 1
        duration = 2
        untilEnable = 2
        until = 3
        extensionEnable = 3
        extension = 4
        triggered = 5
        expected = 7
        join = 7
        permitted = 7
        parking = 8
        actType = 10
        tripID = 11
        line = 12
        onDemand = 13
        jump = 14
        split = 15

    class inspectSelection:
        durationEnable = 1
        duration = 2
        untilEnable = 3
        until = 4
        extensionEnable = 5
        extension = 6
        triggered = 7
        expected = 9
        join = 9
        permitted = 10
        parking = 11
        actType = 13
        tripID = 14
        line = 15
        onDemand = 16
        jump = 17
        split = 18

# stop over parkings


class stopParkingArea:
    class create:
        durationEnable = 8
        duration = 9
        untilEnable = 9
        until = 10
        extensionEnable = 10
        extension = 11
        triggered = 12
        expected = 14
        join = 14
        permitted = 14
        actType = 15
        tripID = 16
        line = 17
        onDemand = 18
        jump = 19
        split = 20

    class inspect:
        durationEnable = 1
        duration = 2
        untilEnable = 2
        until = 3
        extensionEnable = 3
        extension = 4
        triggered = 5
        expected = 7
        join = 7
        permitted = 7
        actType = 8
        tripID = 9
        line = 10
        onDemand = 11
        jump = 12
        split = 13

    class inspectSelection:
        durationEnable = 1
        duration = 2
        untilEnable = 3
        until = 4
        extensionEnable = 5
        extension = 6
        triggered = 7
        expected = 9
        join = 9
        permitted = 10
        actType = 11
        tripID = 12
        line = 13
        onDemand = 14
        jump = 15
        split = 16

# stop over lane


class stopLane:
    class create:
        friendlyPos = 8
        posLat = 9
        durationEnable = 10
        duration = 11
        untilEnable = 11
        until = 12
        extensionEnable = 12
        extension = 13
        triggered = 14
        expected = 16
        join = 16
        permitted = 16
        parking = 17
        actType = 19
        tripID = 20
        line = 21
        onDemand = 22
        jump = 23
        split = 24

    class inspect:
        lane = 1
        moveLane = 2
        startPos = 3
        endPos = 4
        friendlyPos = 5
        posLat = 6
        durationEnable = 7
        duration = 8
        untilEnable = 8
        until = 9
        extensionEnable = 9
        extension = 10
        triggered = 11
        expected = 13
        join = 13
        permitted = 13
        parking = 14
        actType = 16
        tripID = 17
        line = 18
        onDemand = 19
        jump = 20
        split = 21

    class inspectSelection:
        friendlyPos = 1
        posLat = 2
        durationEnable = 3
        duration = 4
        untilEnable = 5
        until = 6
        extensionEnable = 7
        extension = 8
        triggered = 9
        expected = 11
        join = 11
        permitted = 12
        parking = 13
        actType = 15
        tripID = 16
        line = 17
        onDemand = 18
        jump = 19
        split = 20

# waypoint over bus stops


class waypointBusStop:
    class create:
        durationEnable = 8
        duration = 9
        untilEnable = 9
        until = 10
        extensionEnable = 10
        extension = 11
        permitted = 12
        parking = 13
        actType = 15
        tripID = 16
        line = 17
        speed = 18
        jump = 19
        split = 20

    class inspect:
        durationEnable = 1
        duration = 2
        untilEnable = 2
        until = 3
        extensionEnable = 3
        extension = 4
        permitted = 5
        parking = 6
        actType = 8
        tripID = 9
        line = 10
        speed = 11
        jump = 12
        split = 13

    class inspectSelection:
        durationEnable = 1
        duration = 2
        untilEnable = 3
        until = 4
        extensionEnable = 5
        extension = 6
        permitted = 7
        parking = 8
        actType = 10
        tripID = 11
        line = 12
        speed = 13
        jump = 14
        split = 15

# waypoint over train stops


class waypointTrainStop:
    class create:
        durationEnable = 8
        duration = 9
        untilEnable = 9
        until = 10
        extensionEnable = 10
        extension = 11
        permitted = 12
        parking = 13
        actType = 15
        tripID = 16
        line = 17
        speed = 18
        jump = 19
        split = 20

    class inspect:
        durationEnable = 1
        duration = 2
        untilEnable = 2
        until = 3
        extensionEnable = 3
        extension = 4
        permitted = 5
        parking = 6
        actType = 8
        tripID = 9
        line = 10
        speed = 11
        jump = 12
        split = 13

    class inspectSelection:
        durationEnable = 1
        duration = 2
        untilEnable = 3
        until = 4
        extensionEnable = 5
        extension = 6
        permitted = 7
        parking = 8
        actType = 10
        tripID = 11
        line = 12
        speed = 13
        jump = 14
        split = 15

# waypoint over container stops


class waypointContainerStop:
    class create:
        durationEnable = 8
        duration = 9
        untilEnable = 9
        until = 10
        extensionEnable = 10
        extension = 11
        permitted = 12
        parking = 13
        actType = 15
        tripID = 16
        line = 17
        speed = 18
        jump = 19
        split = 20

    class inspect:
        durationEnable = 1
        duration = 2
        untilEnable = 2
        until = 3
        extensionEnable = 3
        extension = 4
        permitted = 5
        parking = 6
        actType = 8
        tripID = 9
        line = 10
        speed = 11
        jump = 12
        split = 13

    class inspectSelection:
        durationEnable = 1
        duration = 2
        untilEnable = 3
        until = 4
        extensionEnable = 5
        extension = 6
        permitted = 7
        parking = 8
        actType = 10
        tripID = 11
        line = 12
        speed = 13
        jump = 14
        split = 15

# waypoint over charging stations


class waypointChargingStation:
    class create:
        durationEnable = 8
        duration = 9
        untilEnable = 9
        until = 10
        extensionEnable = 10
        extension = 11
        permitted = 12
        parking = 13
        actType = 15
        tripID = 16
        line = 17
        speed = 18
        jump = 19
        split = 20

    class inspect:
        durationEnable = 1
        duration = 2
        untilEnable = 2
        until = 3
        extensionEnable = 3
        extension = 4
        permitted = 5
        parking = 6
        actType = 8
        tripID = 9
        line = 10
        speed = 11
        jump = 12
        split = 13

    class inspectSelection:
        durationEnable = 1
        duration = 2
        untilEnable = 3
        until = 4
        extensionEnable = 5
        extension = 6
        permitted = 7
        parking = 8
        actType = 10
        tripID = 11
        line = 12
        speed = 13
        jump = 14
        split = 15

# waypoint over parkings


class waypointParking:
    class create:
        durationEnable = 8
        duration = 9
        untilEnable = 9
        until = 10
        extensionEnable = 10
        extension = 11
        permitted = 12
        actType = 13
        tripID = 14
        line = 15
        speed = 16
        jump = 17
        split = 18

    class inspect:
        durationEnable = 1
        duration = 2
        untilEnable = 2
        until = 3
        extensionEnable = 3
        extension = 4
        permitted = 5
        actType = 6
        tripID = 7
        line = 8
        speed = 9
        jump = 10
        split = 11

    class inspectSelection:
        durationEnable = 1
        duration = 2
        untilEnable = 3
        until = 4
        extensionEnable = 5
        extension = 6
        permitted = 7
        actType = 8
        tripID = 9
        line = 10
        speed = 11
        jump = 12
        split = 13

# waypoint (over lane)


class waypointLane:
    class create:
        friendlyPos = 8
        posLat = 9
        durationEnable = 10
        duration = 11
        untilEnable = 11
        until = 12
        extensionEnable = 12
        extension = 13
        permitted = 14
        parking = 15
        actType = 17
        tripID = 18
        line = 19
        speed = 20
        jump = 21
        split = 22

    class inspect:
        lane = 1
        moveLane = 2
        startPos = 3
        endPos = 4
        friendlyPos = 5
        posLat = 6
        durationEnable = 7
        duration = 8
        untilEnable = 8
        until = 9
        extensionEnable = 9
        extension = 10
        permitted = 11
        parking = 12
        actType = 14
        tripID = 15
        line = 16
        speed = 17
        jump = 18
        split = 19

    class inspectSelection:
        friendlyPos = 1
        posLat = 2
        durationEnable = 3
        duration = 4
        untilEnable = 5
        until = 6
        extensionEnable = 7
        extension = 8
        permitted = 9
        parking = 10
        actType = 12
        tripID = 13
        line = 14
        speed = 15
        jump = 16
        split = 17

# Person


class person:
    class create:
        id = 8
        colorButton = 9
        color = 10
        departPos = 11
        depart = 12

    class inspect:
        id = 1
        typeButton = 2
        type = 3
        colorButton = 5
        color = 6
        departPos = 7
        depart = 8
        parameters = 11

    class inspectSelection:
        type = 1
        colorButton = 3
        color = 4
        departPos = 5
        depart = 6
        parameters = 9


class personFlow:
    class create:
        id = 8
        colorButton = 9
        color = 10
        departPos = 11
        begin = 12
        terminate = 16
        end = 18
        number = 19
        spacing = 18
        terminateOption = 20
        spacingOption = 21

    class inspect:
        id = 1
        typeButton = 2
        type = 3
        colorButton = 5
        color = 6
        departPos = 7
        begin = 8
        terminate = 11
        spacing = 13
        end = 14
        terminateOption = 15
        spacingOption = 16
        parameters = 18

    class inspectSelection:
        type = 1
        colorButton = 3
        color = 4
        departPos = 5
        begin = 6
        terminate = 9
        spacing = 11
        terminateOption = 13
        spacingOption = 14
        parameters = 16

# PersonPlans


class personTrip:
    class edge:
        class edge:
            class create:
                arrivalPos = 8
                vTypes = 9
                modes = 10
                lines = 11
                walkFactor = 12
                group = 13

            class inspect:
                arrivalPos = 1
                vTypes = 2
                modes = 3
                lines = 4
                walkFactor = 5
                group = 6

            class inspectSelection:
                arrivalPos = 1
                vTypes = 2
                modes = 3
                lines = 4
                walkFactor = 5
                group = 6

        class busStop:
            class create:
                arrivalPos = 8
                vTypes = 9
                modes = 10
                lines = 11
                walkFactor = 12
                group = 13

            class inspect:
                vTypes = 1
                modes = 2
                lines = 3
                walkFactor = 4
                group = 5

            class inspectSelection:
                vTypes = 1
                modes = 2
                lines = 3
                walkFactor = 4
                group = 5

        class junction:
            class create:
                arrivalPos = 8
                vTypes = 9
                modes = 10
                lines = 11
                walkFactor = 12
                group = 13

            class inspect:
                vTypes = 1
                modes = 2
                lines = 3
                walkFactor = 4
                group = 5

            class inspectSelection:
                vTypes = 1
                modes = 2
                lines = 3
                walkFactor = 4
                group = 5

    class busStop:
        class edge:
            class create:
                arrivalPos = 8
                vTypes = 9
                modes = 10
                lines = 11
                walkFactor = 12
                group = 13

            class inspect:
                arrivalPos = 1
                vTypes = 2
                modes = 3
                lines = 4
                walkFactor = 5
                group = 6

            class inspectSelection:
                arrivalPos = 1
                vTypes = 2
                modes = 3
                lines = 4
                walkFactor = 5
                group = 6

        class busStop:
            class create:
                arrivalPos = 8
                vTypes = 9
                modes = 10
                lines = 11
                walkFactor = 12
                group = 13

            class inspect:
                vTypes = 1
                modes = 2
                lines = 3
                walkFactor = 4
                group = 5

            class inspectSelection:
                vTypes = 1
                modes = 2
                lines = 3
                walkFactor = 4
                group = 5

        class junction:
            class create:
                arrivalPos = 8
                vTypes = 9
                modes = 10
                lines = 11
                walkFactor = 12
                group = 13

            class inspect:
                vTypes = 1
                modes = 2
                lines = 3
                walkFactor = 4
                group = 5

            class inspectSelection:
                vTypes = 1
                modes = 2
                lines = 3
                walkFactor = 4
                group = 5

    class junction:
        class edge:
            class create:
                arrivalPos = 8
                vTypes = 9
                modes = 10
                lines = 11
                walkFactor = 12
                group = 13

            class inspect:
                arrivalPos = 1
                vTypes = 2
                modes = 3
                lines = 4
                walkFactor = 5
                group = 6

            class inspectSelection:
                arrivalPos = 1
                vTypes = 2
                modes = 3
                lines = 4
                walkFactor = 5
                group = 6

        class busStop:
            class create:
                arrivalPos = 8
                vTypes = 9
                modes = 10
                lines = 11
                walkFactor = 12
                group = 13

            class inspect:
                vTypes = 1
                modes = 2
                lines = 3
                walkFactor = 4
                group = 5

            class inspectSelection:
                vTypes = 1
                modes = 2
                lines = 3
                walkFactor = 4
                group = 5

        class junction:
            class create:
                arrivalPos = 8
                vTypes = 9
                modes = 10
                lines = 11
                walkFactor = 12
                group = 13

            class inspect:
                vTypes = 1
                modes = 2
                lines = 3
                walkFactor = 4
                group = 5

            class inspectSelection:
                vTypes = 1
                modes = 2
                lines = 3
                walkFactor = 4
                group = 5


class ride:
    class edge:
        class edge:
            class create:
                arrivalPos = 8
                lines = 9
                group = 10

            class inspect:
                arrivalPos = 1
                lines = 2
                group = 3

            class inspectSelection:
                arrivalPos = 1
                lines = 2
                group = 3

        class busStop:
            class create:
                arrivalPos = 8
                lines = 9
                group = 10

            class inspect:
                arrivalPos = 1
                lines = 2
                group = 3

            class inspectSelection:
                arrivalPos = 1
                lines = 2
                group = 3

        class junction:
            class create:
                arrivalPos = 8
                lines = 9
                group = 10

            class inspect:
                arrivalPos = 1
                lines = 2
                group = 3

            class inspectSelection:
                arrivalPos = 1
                lines = 2
                group = 3

    class busStop:
        class edge:
            class create:
                arrivalPos = 8
                lines = 9
                group = 10

            class inspect:
                arrivalPos = 1
                lines = 2
                group = 3

            class inspectSelection:
                arrivalPos = 1
                lines = 2
                group = 3

        class busStop:
            class create:
                arrivalPos = 8
                lines = 9
                group = 10

            class inspect:
                arrivalPos = 1
                lines = 2
                group = 3

            class inspectSelection:
                arrivalPos = 1
                lines = 2
                group = 3

        class junction:
            class create:
                arrivalPos = 8
                lines = 9
                group = 10

            class inspect:
                arrivalPos = 1
                lines = 2
                group = 3

            class inspectSelection:
                arrivalPos = 1
                lines = 2
                group = 3

    class junction:
        class edge:
            class create:
                arrivalPos = 8
                lines = 9
                group = 10

            class inspect:
                arrivalPos = 1
                lines = 2
                group = 3

            class inspectSelection:
                arrivalPos = 1
                lines = 2
                group = 3

        class busStop:
            class create:
                arrivalPos = 8
                lines = 9
                group = 10

            class inspect:
                arrivalPos = 1
                lines = 2
                group = 3

            class inspectSelection:
                arrivalPos = 1
                lines = 2
                group = 3

        class junction:
            class create:
                arrivalPos = 8
                lines = 9
                group = 10

            class inspect:
                arrivalPos = 1
                lines = 2
                group = 3

            class inspectSelection:
                arrivalPos = 1
                lines = 2
                group = 3


class walk:
    class edge:
        class edge:
            class create:
                arrivalPos = 8
                speed = 9
                duration = 10

            class inspect:
                arrivalPos = 1
                speed = 2
                duration = 3

            class inspectSelection:
                arrivalPos = 1
                speed = 2
                duration = 3

        class busStop:
            class create:
                arrivalPos = 8
                speed = 9
                duration = 10

            class inspect:
                arrivalPos = 1
                speed = 2
                duration = 3

            class inspectSelection:
                arrivalPos = 1
                speed = 2
                duration = 3

        class junction:
            class create:
                arrivalPos = 8
                speed = 9
                duration = 10

            class inspect:
                arrivalPos = 1
                speed = 2
                duration = 3

            class inspectSelection:
                arrivalPos = 1
                speed = 2
                duration = 3

    class busStop:
        class edge:
            class create:
                arrivalPos = 8
                speed = 9
                duration = 10

            class inspect:
                arrivalPos = 1
                speed = 2
                duration = 3

            class inspectSelection:
                arrivalPos = 1
                speed = 2
                duration = 3

        class busStop:
            class create:
                arrivalPos = 8
                speed = 9
                duration = 10

            class inspect:
                arrivalPos = 1
                speed = 2
                duration = 3

            class inspectSelection:
                arrivalPos = 1
                speed = 2
                duration = 3

        class junction:
            class create:
                arrivalPos = 8
                speed = 9
                duration = 10

            class inspect:
                arrivalPos = 1
                speed = 2
                duration = 3

            class inspectSelection:
                arrivalPos = 1
                speed = 2
                duration = 3

    class junction:
        class edge:
            class create:
                arrivalPos = 8
                speed = 9
                duration = 10

            class inspect:
                arrivalPos = 1
                speed = 2
                duration = 3

            class inspectSelection:
                arrivalPos = 1
                speed = 2
                duration = 3

        class busStop:
            class create:
                arrivalPos = 8
                speed = 9
                duration = 10

            class inspect:
                arrivalPos = 1
                speed = 2
                duration = 3

            class inspectSelection:
                arrivalPos = 1
                speed = 2
                duration = 3

        class junction:
            class create:
                arrivalPos = 8
                speed = 9
                duration = 10

            class inspect:
                arrivalPos = 1
                speed = 2
                duration = 3

            class inspectSelection:
                arrivalPos = 1
                speed = 2
                duration = 3


class walkEdges:
    class create:
        arrivalPos = 8

    class inspect:
        arrivalPos = 1

    class inspectSelection:
        arrivalPos = 1


class walkRoute:
    class create:
        arrivalPos = 8

    class inspect:
        arrivalPos = 1

    class inspectSelection:
        arrivalPos = 1

# Container


class container:
    class create:
        id = 8
        colorButton = 9
        color = 10
        departPos = 11
        depart = 12

    class inspect:
        id = 1
        typeButton = 2
        type = 3
        colorButton = 5
        color = 6
        departPos = 7
        depart = 8
        parameters = 11

    class inspectSelection:
        type = 1
        colorButton = 3
        color = 4
        departPos = 5
        depart = 6
        parameters = 9


class containerFlow:
    class create:
        id = 8
        colorButton = 9
        color = 10
        departPos = 11
        begin = 12
        terminate = 16
        end = 18
        number = 19
        spacing = 18
        terminateOption = 20
        spacingOption = 21

    class inspect:
        id = 1
        typeButton = 2
        type = 3
        colorButton = 5
        color = 6
        departPos = 7
        begin = 8
        terminate = 11
        spacing = 13
        end = 14
        terminateOption = 15
        spacingOption = 16
        parameters = 18

    class inspectSelection:
        type = 1
        colorButton = 3
        color = 4
        departPos = 5
        begin = 6
        terminate = 9
        spacing = 11
        terminateOption = 13
        spacingOption = 14
        parameters = 16

# PersonPlans


class transport:
    class edge:
        class edge:
            class create:
                arrivalPos = 8
                lines = 9
                group = 10

            class inspect:
                arrivalPos = 1
                lines = 2
                group = 3

            class inspectSelection:
                arrivalPos = 1
                lines = 2
                group = 3

        class busStop:
            class create:
                arrivalPos = 8
                lines = 9
                group = 10

            class inspect:
                lines = 1
                group = 2

            class inspectSelection:
                lines = 1
                group = 2

        class junction:
            class create:
                arrivalPos = 8
                lines = 9
                group = 10

            class inspect:
                lines = 1
                group = 2

            class inspectSelection:
                lines = 1
                group = 2

    class busStop:
        class edge:
            class create:
                arrivalPos = 8
                lines = 9
                group = 10

            class inspect:
                arrivalPos = 1
                lines = 2
                group = 3

            class inspectSelection:
                arrivalPos = 1
                lines = 2
                group = 3

        class busStop:
            class create:
                arrivalPos = 8
                lines = 9
                group = 10

            class inspect:
                lines = 1
                group = 2

            class inspectSelection:
                lines = 1
                group = 2

        class junction:
            class create:
                arrivalPos = 8
                lines = 9
                group = 10

            class inspect:
                lines = 1
                group = 2

            class inspectSelection:
                lines = 1
                group = 2

    class junction:
        class edge:
            class create:
                arrivalPos = 8
                lines = 9
                group = 10

            class inspect:
                arrivalPos = 1
                lines = 2
                group = 3

            class inspectSelection:
                arrivalPos = 1
                lines = 2
                group = 3

        class busStop:
            class create:
                arrivalPos = 8
                lines = 9
                group = 10

            class inspect:
                lines = 1
                group = 2

            class inspectSelection:
                lines = 1
                group = 2

        class junction:
            class create:
                arrivalPos = 8
                lines = 9
                group = 10

            class inspect:
                lines = 1
                group = 2

            class inspectSelection:
                lines = 1
                group = 2


class tranship:
    class edge:
        class edge:
            class create:
                arrivalPos = 8
                speed = 9
                duration = 10

            class inspect:
                arrivalPos = 1
                speed = 2
                duration = 3

            class inspectSelection:
                arrivalPos = 1
                speed = 2
                duration = 3

        class busStop:
            class create:
                arrivalPos = 8
                speed = 9
                duration = 10

            class inspect:
                speed = 1
                duration = 2

            class inspectSelection:
                speed = 1
                duration = 2

        class junction:
            class create:
                arrivalPos = 8
                speed = 9
                duration = 10

            class inspect:
                speed = 1
                duration = 2

            class inspectSelection:
                speed = 1
                duration = 2

    class busStop:
        class edge:
            class create:
                arrivalPos = 8
                speed = 9
                duration = 10

            class inspect:
                arrivalPos = 1
                speed = 2
                duration = 3

            class inspectSelection:
                arrivalPos = 1
                speed = 2
                duration = 3

        class busStop:
            class create:
                arrivalPos = 8
                speed = 9
                duration = 10

            class inspect:
                speed = 1
                duration = 2

            class inspectSelection:
                speed = 1
                duration = 2

        class junction:
            class create:
                speed = 8
                duration = 9

            class inspect:
                speed = 1
                duration = 2

            class inspectSelection:
                speed = 1
                duration = 2

    class junction:
        class edge:
            class create:
                arrivalPos = 8
                speed = 9
                duration = 10

            class inspect:
                arrivalPos = 1
                speed = 2
                duration = 3

            class inspectSelection:
                arrivalPos = 1
                speed = 2
                duration = 3

        class busStop:
            class create:
                arrivalPos = 8
                speed = 9
                duration = 10

            class inspect:
                speed = 1
                duration = 2

            class inspectSelection:
                speed = 1
                duration = 2

        class junction:
            class create:
                arrivalPos = 8
                speed = 9
                duration = 10

            class inspect:
                speed = 1
                duration = 2

            class inspectSelection:
                speed = 1
                duration = 2

# stop plans


class stopPlan:
    class edge:
        class create:
            durationEnable = 8
            duration = 9
            untilEnable = 10
            until = 11
            actType = 11
            friendlyPos = 12

        class inspect:
            endPos = 1
            durationEnable = 2
            duration = 3
            untilEnable = 4
            until = 5
            actType = 5
            friendlyPos = 6

        class inspectSelection:
            durationEnable = 1
            duration = 2
            untilEnable = 3
            until = 4
            actType = 5
            friendlyPos = 6

    class stoppingPlace:
        class create:
            durationEnable = 8
            duration = 9
            untilEnable = 10
            until = 11
            actType = 11

        class inspect:
            durationEnable = 1
            duration = 2
            untilEnable = 3
            until = 4
            actType = 4

        class inspectSelection:
            durationEnable = 1
            duration = 2
            untilEnable = 3
            until = 4
            actType = 4


# --------------------------------
# DATA ELEMENTS
# --------------------------------

# edgeData


class edgeData:
    class create:
        parameters = 12

    class inspect:
        parameters = 3

    class inspectSelection:
        parameters = 3

# edgeRelData


class edgeRelData:
    class create:
        parameters = 12

    class inspect:
        fromEdge = 1
        toEdge = 2
        parameters = 5

    class inspectSelection:
        parameters = 3

# TAZRelData


class TAZRelData:
    class create:
        parameters = 12

    class inspect:
        fromTAZ = 1
        toTAZ = 2
        parameters = 5

    class inspectSelection:
        parameters = 3


# --------------------------------
# Dialogs
# --------------------------------

class dialog:

    class allowVClass:
        allVehicles = 1
        onlyRoadVehicles = 2
        onlyRailVehicles = 3
        disallowAll = 4
        passenger = 5
        private = 6
        taxi = 7
        bus = 8
        coach = 9
        delivery = 10
        truck = 11
        trailer = 12
        emergency = 13
        motorcycle = 14
        moped = 15
        bicycle = 16
        scooter = 17
        pedestrian = 18
        wheelchair = 19
        tram = 20
        rail_electric = 21
        rail_fast = 22
        rail_urban = 23
        rail = 24
        cable_car = 25
        subway = 26
        evehicle = 27
        army = 28
        ship = 29
        authority = 30
        vip = 31
        hov = 32
        container = 33
        aircraft = 34
        drone = 35
        custom1 = 36
        custom2 = 37
        accept = 38
        cancel = 39
        reset = 40
