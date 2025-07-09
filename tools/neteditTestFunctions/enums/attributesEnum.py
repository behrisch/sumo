# -*- coding: utf-8 -*-
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
# Copyright (C) 2009-2025 German Aerospace Center (DLR) and others.
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

class attrs:
    # --------------------------------
    # GENERAL
    # --------------------------------

    class toolbar:
        class file:
            newNetwork = 1
            newWindow = 2
            generateNetwork = 3
            loadNeteditConfig = 4
            loadSumoConfig = 5
            loadNetwork = 6
            openNetconvertConfiguration = 7
            importForeignNetwork = 8
            saveNetwork = 9
            saveNetworkAs = 9
            savePlainXML = 10
            saveJoinedJunctions = 11
            reloadConfig = 12
            reloadNetwork = 13

            class neteditConfig:
                menu = 14
                save = 1
                saveAs = 2

            class sumoConfig:
                menu = 15
                save = 1
                saveAs = 2

            class trafficLights:
                menu = 16
                load = 1
                save = 2
                saveAs = 3
                reload = 4

            class edgeTypes:
                menu = 17
                load = 1
                save = 2
                saveAs = 3
                reload = 4

            class aditionalElements:
                menu = 18
                load = 1
                save = 2
                saveAs = 2
                saveJupedsim = 3
                reload = 4

            class demandElements:
                menu = 19
                load = 1
                save = 2
                saveAs = 2
                reload = 3

            class dataElements:
                menu = 20
                load = 1
                save = 2
                saveAs = 2
                reload = 3

            class meanDataElements:
                menu = 21
                load = 1
                save = 2
                saveAs = 2
                reload = 3

    class editElements:
        overlapped = 5

    class move:
        moveWholePolygon = 5

    class shape:
        size = 200

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
        class changeElement:
            additional = 2
            shape = 2
            vehicle = 2
            route = 2
            person = 2
            container = 2
            meanData = 2
            personPlan = 5
            containerPlan = 5
            stop = 5

        class selection:
            class basic:
                add = 3
                remove = 4
                keep = 5
                replace = 6
                default = 15
                clear = 21
                save = 22
                delete = 23
                invert = 24
                load = 25
                reduce = 26
                invertData = 27

            class networkItem:
                type = 8
                subType = 2
                attribute = 3
                value = 2

        class changeParentElement:
            route = 4
            stop = 2

        class changePlan:
            person = 22
            personFlow = 29
            container = 22
            containerFlow = 29

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
            id = 2
            pos = 3
            type = 4
            shape = 6
            radius = 7
            keepClear = 8
            rightOfWay = 9
            fringe = 11
            name = 13
            parameters = 15
            parametersButton = 16
            frontButton = 18
            helpDialog = 19
            reset = 20

        class inspectTLS:
            id = 2
            pos = 3
            type = 4
            shape = 6
            radius = 7
            keepClear = 8
            rightOfWay = 9
            fringe = 11
            name = 13
            tlType = 14
            tlLayout = 16
            tl = 18
            parameters = 20
            parametersButton = 21
            frontButton = 23
            helpDialog = 24
            reset = 25

        class inspectSelection:
            type = 2
            radius = 3
            keepClear = 4
            rightOfWay = 5
            fringe = 6
            name = 7
            parameters = 9
            parametersButton = 10
            frontButton = 12
            helpDialog = 13
            reset = 14

        class inspectSelectionTLS:
            type = 2
            radius = 3
            keepClear = 4
            rightOfWay = 5
            fringe = 6
            name = 7
            tlType = 8
            tlLayout = 9
            tl = 11
            parameters = 14
            parametersButton = 15
            frontButton = 17
            helpDialog = 18
            reset = 19

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
            allowButton = 11
            allow = 12
            disallow = 13
            spreadType = 14
            priority = 16
            width = 17
            sidewalkWidth = 18
            bikelaneWidth = 19
            parameters = 21
            parametersButton = 22

        class inspect:
            id = 2
            fromEdge = 3
            toEdge = 4
            speed = 5
            priority = 6
            numLanes = 7
            type = 8
            allowButton = 9
            allow = 10
            disallow = 11
            shape = 12
            length = 13
            spreadType = 14
            name = 16
            width = 17
            endOffset = 18
            shapeStart = 19
            shapeEnd = 20
            distance = 21
            stopOffset = 22
            stopOffsetExceptionButton = 23
            stopOffsetException = 24
            parameters = 26
            parametersButton = 27
            frontButton = 29
            helpDialog = 30
            reset = 31

        class inspectSelection:
            speed = 2
            priority = 3
            numLanes = 4
            type = 5
            allowButton = 6
            allow = 7
            disallow = 8
            length = 9
            spreadType = 10
            name = 11
            width = 12
            endOffset = 13
            shapeStart = 14
            shapeEnd = 15
            stopOffset = 16
            stopOffsetExceptionButton = 17
            stopOffsetException = 18
            parameters = 20
            parametersButton = 21
            frontButton = 23
            helpDialog = 24
            reset = 25

        class template:
            create = 32
            copy = 33
            clear = 34

    # lane

    class lane:
        class create:
            add = 24
            remove = 25
            speed = 26
            allowButton = 27
            allow = 28
            disallow = 29
            width = 30
            parameters = 32
            parametersButton = 33

        class inspect:
            speed = 2
            allowButton = 3
            allow = 4
            disallow = 5
            width = 6
            endOffset = 7
            acceleration = 8
            customShape = 9
            opposite = 10
            changeLeftButton = 11
            changeLeft = 12
            changeRightButton = 13
            changeRight = 14
            type = 15
            stopOffset = 16
            stopOffsetExceptionButton = 17
            stopOffsetException = 18
            parameters = 19
            parametersButton = 20
            frontButton = 22
            helpDialog = 23
            reset = 24

        class inspectSelection:
            speed = 2
            allowButton = 3
            allow = 4
            disallow = 5
            width = 6
            endOffset = 7
            acceleration = 8
            changeLeftButton = 9
            changeLeft = 10
            changeRightButton = 11
            changeRight = 12
            type = 13
            stopOffset = 14
            stopOffsetExceptionButton = 15
            stopOffsetException = 16
            parameters = 18
            parametersButton = 19
            frontButton = 21
            helpDialog = 22
            reset = 23

    # connection

    class connection:
        class inspect:
            passC = 2
            keepClear = 3
            contPos = 4
            uncontrolled = 5
            visibility = 6
            allowButton = 7
            allow = 8
            disallow = 9
            speed = 10
            length = 11
            customShape = 12
            changeLeftButton = 13
            changeLeft = 14
            changeRightButton = 15
            changeRight = 16
            indirect = 17
            type = 18
            parameters = 20
            parametersButton = 21
            frontButton = 23
            helpDialog = 24
            reset = 25

        class inspectTLS:
            passC = 2
            keepClear = 3
            contPos = 4
            uncontrolled = 5
            visibility = 6
            linkIndex = 7
            linkIndex2 = 8
            allowButton = 9
            allow = 10
            disallow = 11
            speed = 12
            length = 13
            customShape = 14
            changeLeftButton = 15
            changeLeft = 16
            changeRightButton = 17
            changeRight = 18
            indirect = 19
            type = 20
            parameters = 22
            parametersButton = 23
            frontButton = 25
            helpDialog = 26
            reset = 27

        class inspectSelection:
            passC = 2
            keepClear = 3
            contPos = 4
            uncontrolled = 5
            visibility = 6
            allowButton = 7
            allow = 8
            disallow = 9
            speed = 10
            length = 11
            changeLeftButton = 12
            changeLeft = 13
            changeRightButton = 14
            changeRight = 15
            indirect = 16
            type = 17
            parameters = 19
            parametersButton = 20
            frontButton = 22
            helpDialog = 23
            reset = 24

        class inspectSelectionTLS:
            passC = 2
            keepClear = 3
            contPos = 4
            uncontrolled = 5
            visibility = 6
            linkIndex = 7
            linkIndex2 = 8
            allowButton = 9
            allow = 10
            disallow = 11
            speed = 12
            length = 13
            changeLeftButton = 14
            changeLeft = 15
            changeRightButton = 16
            changeRight = 17
            indirect = 18
            type = 19
            parameters = 21
            parametersButton = 22
            frontButton = 24
            helpDialog = 25
            reset = 26

        saveConnections = 4

    # crossing

    class crossing:
        class create:
            edges = 2
            priority = 3
            width = 4
            button = 10

        class createTLS:
            edges = 2
            width = 3
            button = 9

        class inspect:
            edges = 2
            priority = 3
            width = 4
            customShape = 5
            parameters = 7
            parametersButton = 8
            frontButton = 10
            helpDialog = 11
            reset = 12

        class inspectTLS:
            edges = 2
            priority = 3
            width = 4
            linkIndex = 5
            linkIndex2 = 6
            customShape = 7
            parameters = 9
            parametersButton = 10
            frontButton = 12
            helpDialog = 13
            reset = 14

        class inspectSelection:
            priority = 2
            width = 3
            parameters = 5
            parametersButton = 6
            frontButton = 8
            helpDialog = 9
            reset = 10

        class inspectSelectionTLS:
            priority = 2
            width = 3
            linkIndex = 4
            linkIndex2 = 5
            parameters = 7
            parametersButton = 8
            frontButton = 10
            helpDialog = 11
            reset = 12

        clearEdges = 3
        useSelectedEdges = 3
        firstField = 3

    # TLS

    class TLS:

        class attributes:
            TLID = 2
            TLType = 3
            offset = 13
            parameters = 15

        class phases:

            class static:
                dur = 1
                state = 2
                nxt = 3
                name = 4

            class actuated:
                dur = 1
                minD = 2
                maxD = 3
                state = 4
                earlyEnd = 5
                latestEnd = 6
                nxt = 7
                name = 8

            class delayBased:
                dur = 1
                minD = 2
                maxD = 3
                state = 4
                nxt = 5
                name = 6

            class NEMA:
                dur = 1
                minD = 2
                maxD = 3
                state = 4
                vehext = 5
                yellow = 6
                red = 7
                nxt = 8

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

    # busStop

    class busStop:
        class create:
            id = 5
            friendlyPos = 6
            name = 7
            colorButton = 8
            color = 9
            lines = 10
            personCapacity = 11
            parkingLength = 12
            parameters = 14
            parametersButton = 15
            size = 17
            forceSize = 18
            reference = 19
            additionalFileButton = 21
            additionalFile = 22
            helpDialog = 23
            reset = 24

        class inspect:
            id = 2
            lane = 3
            moveLane = 4
            startPos = 5
            endPos = 6
            friendlyPos = 7
            name = 8
            colorButton = 9
            color = 10
            lines = 11
            personCapacity = 12
            parkingLength = 13
            parameters = 15
            parametersButton = 16
            frontButton = 18
            size = 19
            forceSize = 20
            reference = 21
            additionalFileButton = 23
            additionalFile = 24
            helpDialog = 25
            reset = 26

        class inspectSelection:
            friendlyPos = 2
            name = 3
            colorButton = 4
            color = 5
            lines = 6
            personCapacity = 7
            parkingLength = 8
            parameters = 10
            parametersButton = 11
            frontButton = 13
            size = 14
            forceSize = 15
            reference = 16
            additionalFileButton = 18
            additionalFile = 19
            helpDialog = 20
            reset = 21

    # trainStop

    class trainStop:
        class create:
            id = 5
            friendlyPos = 6
            name = 7
            colorButton = 8
            color = 9
            lines = 10
            personCapacity = 11
            parkingLength = 12
            parameters = 14
            parametersButton = 15
            size = 17
            forceSize = 18
            reference = 19
            additionalFileButton = 21
            additionalFile = 22
            helpDialog = 23
            reset = 24

        class inspect:
            id = 2
            lane = 3
            moveLane = 4
            startPos = 5
            endPos = 6
            friendlyPos = 7
            name = 8
            colorButton = 9
            color = 10
            lines = 11
            personCapacity = 12
            parkingLength = 13
            parameters = 15
            parametersButton = 16
            frontButton = 18
            size = 19
            forceSize = 20
            reference = 21
            additionalFileButton = 23
            additionalFile = 24
            helpDialog = 25
            reset = 26

        class inspectSelection:
            friendlyPos = 2
            name = 3
            colorButton = 4
            color = 5
            lines = 6
            personCapacity = 7
            parkingLength = 8
            parameters = 10
            parametersButton = 11
            frontButton = 13
            size = 14
            forceSize = 15
            reference = 16
            additionalFileButton = 18
            additionalFile = 19
            helpDialog = 20
            reset = 21

    # containerStop

    class containerStop:
        class create:
            id = 5
            friendlyPos = 6
            name = 7
            colorButton = 8
            color = 9
            lines = 10
            containerCapacity = 11
            parkingLength = 12
            parameters = 14
            parametersButton = 15
            size = 17
            forceSize = 18
            reference = 19
            additionalFileButton = 21
            additionalFile = 22
            helpDialog = 23
            reset = 24

        class inspect:
            id = 2
            lane = 3
            moveLane = 4
            startPos = 5
            endPos = 6
            friendlyPos = 7
            name = 8
            colorButton = 9
            color = 10
            lines = 11
            containerCapacity = 12
            parkingLength = 13
            parameters = 15
            parametersButton = 16
            frontButton = 18
            size = 19
            forceSize = 20
            reference = 21
            additionalFileButton = 23
            additionalFile = 24
            helpDialog = 25
            reset = 26

        class inspectSelection:
            friendlyPos = 2
            name = 3
            colorButton = 4
            color = 5
            lines = 6
            containerCapacity = 7
            parkingLength = 8
            parameters = 10
            parametersButton = 11
            frontButton = 13
            size = 14
            forceSize = 15
            reference = 16
            additionalFileButton = 18
            additionalFile = 19
            helpDialog = 20
            reset = 21

    # until here

    # chargingStation

    class chargingStation:
        class create:
            id = 5
            friendlyPos = 6
            name = 7
            power = 8
            efficiency = 9
            chargeInTransit = 10
            chargeDelay = 11
            chargeType = 12
            waitingTime = 14
            parkingArea = 15
            parameters = 17
            parametersButton = 18
            size = 20
            forceSize = 21
            reference = 22
            additionalFileButton = 24
            additionalFile = 25
            helpDialog = 26
            reset = 27

        class inspect:
            id = 2
            lane = 3
            moveLane = 4
            startPos = 5
            endPos = 6
            friendlyPos = 7
            name = 8
            power = 9
            efficiency = 10
            chargeInTransit = 11
            chargeDelay = 12
            chargeType = 13
            waitingTime = 15
            parkingArea = 16
            parameters = 18
            parametersButton = 19
            frontButton = 21
            forceSize = 22
            reference = 23
            additionalFileButton = 25
            additionalFile = 26
            helpDialog = 27
            reset = 28

        class inspectSelection:
            friendlyPos = 2
            name = 3
            power = 4
            efficiency = 5
            chargeInTrainsit = 6
            chargeDelay = 7
            chargeType = 8
            waitingTime = 9
            parkingArea = 10
            parameters = 12
            parametersButton = 13
            frontButton = 15
            forceSize = 16
            reference = 17
            additionalFileButton = 19
            additionalFile = 20
            helpDialog = 21
            reset = 22

    # access

    class access:
        class create:
            friendlyPos = 5
            length = 6
            parameters = 8
            parent = 11
            helpDialog = 13
            reset = 14

        class inspect:
            lane = 2
            moveLane = 3
            pos = 4
            friendlyPos = 5
            length = 6
            parameters = 8
            parametersButton = 9
            frontButton = 11
            reparentButton = 12
            reparent = 13
            helpDialog = 14
            reset = 15

        class inspectSelection:
            friendlyPos = 2
            length = 3
            parameters = 5
            parametersButton = 6
            frontButton = 8
            reparentButton = 9
            reparent = 10
            helpDialog = 11
            reset = 12

    # parkingArea

    class parkingArea:
        class create:
            id = 5
            friendlyPos = 6
            name = 7
            departPos = 8
            acceptedBadges = 9
            roadSideCapacity = 10
            onRoad = 11
            width = 12
            length = 13
            angle = 14
            lefthand = 15
            parameters = 17
            parametersButton = 18
            size = 20
            forceSize = 21
            reference = 22
            additionalFileButton = 24
            additionalFile = 25
            helpDialog = 26
            reset = 27

        class inspect:
            id = 2
            lane = 3
            moveLane = 4
            startPos = 5
            endPos = 6
            friendlyPos = 7
            name = 8
            departPos = 9
            acceptedBadges = 10
            roadSideCapacity = 11
            onRoad = 12
            width = 13
            length = 14
            angle = 15
            lefthand = 16
            parameters = 18
            parametersButton = 19
            frontButton = 21
            size = 22
            forceSize = 23
            reference = 24
            additionalFileButton = 26
            additionalFile = 27
            helpDialog = 28
            reset = 29

        class inspectSelection:
            friendlyPos = 2
            name = 3
            departPos = 4
            acceptedBadges = 5
            roadSideCapacity = 6
            onRoad = 7
            width = 8
            length = 9
            angle = 10
            lefthand = 11
            parameters = 13
            parametersButton = 14
            frontButton = 16
            size = 17
            forceSize = 18
            reference = 19
            additionalFileButton = 21
            additionalFile = 22
            helpDialog = 23
            reset = 24

    # parkingSpace

    class parkingSpace:
        class create:
            name = 5
            width = 6
            length = 7
            angle = 8
            slope = 9
            parameters = 11
            parametersButton = 12
            parent = 14
            helpDialog = 16
            reset = 17

        class inspect:
            pos = 2
            name = 3
            width = 4
            length = 5
            angle = 6
            slope = 7
            parameters = 9
            parametersButton = 10
            frontButton = 12
            reparentButton = 13
            reparentButton = 14
            helpDialog = 15
            reset = 16

        class inspectSelection:
            name = 2
            width = 3
            length = 4
            angle = 5
            slope = 6
            parameters = 8
            parametersButton = 9
            frontButton = 11
            reparentButton = 12
            reparentButton = 13
            helpDialog = 14
            reset = 15

    # E1

    class E1:
        class create:
            id = 5
            friendlyPos = 6
            name = 7
            period = 8
            fileButton = 9
            file = 10
            vTypes = 11
            nextEdges = 12
            detectPersons = 13
            parameters = 16
            parametersButton = 17
            additionalFileButton = 19
            additionalFile = 20
            helpDialog = 21
            reset = 22

        class inspect:
            id = 2
            lane = 3
            moveLane = 4
            pos = 5
            friendlyPos = 6
            name = 7
            period = 8
            fileButton = 9
            file = 10
            vTypes = 11
            nextEdges = 12
            detectPersons = 13
            parameters = 16
            parametersButton = 17
            frontButton = 19
            additionalFileButton = 20
            additionalFile = 21
            helpDialog = 22
            reset = 23

        class inspectSelection:
            friendlyPos = 2
            name = 3
            period = 4
            fileButton = 5
            file = 6
            vTypes = 7
            nextEdges = 8
            detectPersons = 9
            parameters = 11
            parametersButton = 12
            frontButton = 14
            additionalFileButton = 15
            additionalFile = 16
            helpDialog = 17
            reset = 18

    # E2

    class E2:
        class create:
            id = 5
            friendlyPos = 6
            length = 7
            name = 8
            period = 9
            tl = 10
            fileButton = 11
            file = 12
            vTypes = 13
            nextEdges = 14
            detectPersons = 15
            timeThreshold = 17
            speedThreshold = 18
            jamThreshold = 19
            show = 20
            parameters = 22
            parametersButton = 23
            additionalFileButton = 25
            additionalFile = 26
            helpDialog = 27
            reset = 28

        class inspect:
            id = 2
            lane = 3
            moveLane = 4
            pos = 5
            friendlyPos = 6
            length = 7
            name = 8
            period = 9
            tl = 10
            fileButton = 11
            file = 12
            vTypes = 13
            nextEdges = 14
            detectPersons = 15
            timeThreshold = 17
            speedThreshold = 18
            jamThreshold = 19
            show = 20
            parameters = 22
            parametersButton = 23
            frontButton = 25
            additionalFileButton = 26
            additionalFile = 27
            helpDialog = 28
            reset = 29

        class inspectSelection:
            friendlyPos = 2
            length = 3
            name = 4
            period = 5
            tl = 6
            fileButton = 7
            file = 8
            vTypes = 9
            nextEdges = 10
            detectPersons = 12
            timeThreshold = 13
            speedThreshold = 14
            jamThreshold = 15
            show = 16
            parameters = 18
            parametersButton = 19
            frontButton = 21
            additionalFileButton = 22
            additionalFile = 23
            helpDialog = 24
            reset = 25

    # E2Multilane

    class E2Multilane:
        class create:
            id = 5
            friendlyPos = 6
            period = 7
            tl = 8
            name = 9
            fileButton = 10
            file = 11
            vTypes = 12
            nextEdges = 13
            detectPersons = 14
            timeThreshold = 16
            speedThreshold = 17
            jamThreshold = 18
            show = 19
            parameters = 21
            parametersButton = 22
            additionalFileButton = 24
            additionalFile = 25
            helpDialog = 26
            reset = 27

        class inspect:
            id = 2
            lanes = 3
            pos = 4
            endPos = 5
            friendlyPos = 6
            period = 7
            tl = 8
            name = 9
            fileButton = 10
            file = 11
            vTypes = 12
            nextEdges = 13
            detectPersons = 14
            timeThreshold = 16
            speedThreshold = 17
            jamThreshold = 18
            show = 19
            parameters = 21
            parametersButton = 22
            frontButton = 24
            additionalFileButton = 25
            additionalFile = 26
            helpDialog = 27
            reset = 28

        class inspectSelection:
            friendlyPos = 2
            period = 3
            tl = 4
            name = 5
            fileButton = 6
            file = 7
            vTypes = 8
            nextEdges = 9
            detectPersons = 10
            timeThreshold = 12
            speedThreshold = 13
            jamThreshold = 14
            show = 15
            parameters = 17
            parametersButton = 18
            frontButton = 20
            additionalFileButton = 21
            additionalFile = 22
            helpDialog = 23
            reset = 24

    # E3

    class E3:
        class create:
            id = 5
            name = 6
            period = 7
            fileButton = 8
            file = 9
            vTypes = 10
            nextEdges = 11
            detectPersons = 12
            openEntry = 14
            timeThreshold = 15
            speedThreshold = 16
            expectArrival = 17
            parameters = 19
            parametersButton = 20
            additionalFileButton = 22
            additionalFile = 23
            helpDialog = 24
            reset = 25

        class inspect:
            id = 2
            pos = 3
            name = 4
            period = 5
            fileButton = 6
            file = 7
            vTypes = 8
            nextEdges = 9
            detectPersons = 10
            openEntry = 12
            timeThreshold = 13
            speedThreshold = 14
            expectArrival = 15
            parameters = 17
            parametersButton = 18
            frontButton = 20
            additionalFileButton = 21
            additionalFile = 22
            helpDialog = 23
            reset = 24

        class inspectSelection:
            name = 2
            period = 3
            fileButton = 4
            file = 5
            vTypes = 6
            nextEdges = 7
            detectPersons = 8
            openEntry = 10
            timeThreshold = 11
            speedThreshold = 12
            expectArrival = 13
            parameters = 15
            parametersButton = 16
            frontButton = 18
            additionalFileButton = 19
            additionalFile = 20
            helpDialog = 21
            reset = 22

    # entryExit

    class entryExit:
        class create:
            friendlyPos = 5
            parameters = 7
            parametersButton = 8
            parent = 10

        class inspect:
            lane = 2
            moveLane = 3
            pos = 4
            friendlyPos = 5
            parameters = 7
            parametersButton = 8
            frontButton = 10
            additionalFileButton = 11
            additionalFile = 12
            helpDialog = 13
            reset = 14

        class inspectSelection:
            friendlyPos = 2
            parameters = 4
            parametersButton = 5
            frontButton = 7
            additionalFileButton = 8
            additionalFile = 9
            helpDialog = 10
            reset = 11

    # E1Instant

    class E1Instant:
        class create:
            id = 5
            friendlyPos = 6
            name = 7
            fileButton = 8
            file = 9
            vTypes = 10
            nextEdges = 11
            detectPersons = 12
            parameters = 15
            parametersButton = 16
            additionalFileButton = 18
            additionalFile = 19
            helpDialog = 20
            reset = 21

        class inspect:
            id = 2
            lane = 3
            moveLane = 4
            pos = 5
            friendlyPos = 6
            name = 7
            fileButton = 8
            file = 9
            vTypes = 10
            nextEdges = 11
            detectPersons = 12
            parameters = 15
            parametersButton = 16
            frontButton = 18
            additionalFileButton = 19
            additionalFile = 20
            helpDialog = 21
            reset = 22

        class inspectSelection:
            friendlyPos = 2
            name = 3
            fileButton = 4
            file = 5
            vTypes = 6
            nextEdges = 7
            detectPersons = 8
            parameters = 11
            parametersButton = 12
            frontButton = 14
            additionalFileButton = 15
            additionalFile = 16
            helpDialog = 17
            reset = 18

    # calibrator

    class calibrator:
        class create:
            id = 5
            name = 6
            period = 7
            routeProbe = 8
            outputButton = 9
            output = 10
            jamTreshold = 11
            vTypes = 12
            parameters = 14
            parametersButton = 15
            center = 17
            additionalFileButton = 18
            additionalFile = 19
            helpDialog = 20
            reset = 21

        class inspect:
            id = 2
            edge = 3
            pos = 4
            name = 5
            frequency = 6
            routeProbe = 7
            outputButton = 8
            output = 9
            jamTreshold = 10
            vTypes = 11
            parameters = 13
            parametersButton = 14
            frontButton = 16
            calibratorDialog = 17
            additionalFileButton = 18
            additionalFile = 19
            helpDialog = 20
            reset = 21

        class inspectLane:
            id = 2
            lane = 3
            laneMoveUp = 4
            pos = 5
            name = 6
            frequency = 7
            routeProbe = 8
            outputButton = 9
            output = 10
            jamTreshold = 11
            vTypes = 12
            parameters = 14
            parametersButton = 15
            frontButton = 17
            calibratorDialog = 18
            additionalFileButton = 19
            additionalFile = 20
            helpDialog = 21
            reset = 22

        class inspectSelection:
            name = 2
            frequency = 3
            routeProbe = 4
            outputButton = 5
            output = 6
            jamTreshold = 7
            vTypes = 8
            parameters = 10
            parametersButton = 11
            frontButton = 13
            additionalFileButton = 14
            additionalFile = 15
            helpDialog = 16
            reset = 17

    # routeProbe (FALTA CENTER XXXX)

    class routeProbe:
        class create:
            id = 5
            frequency = 6
            name = 7
            file = 8
            begin = 9
            vTypes = 10
            center = 14

        class inspect:
            id = 2
            edge = 3
            frequency = 4
            name = 5
            file = 6
            begin = 7
            vTypes = 8
            parameters = 11

        class inspectSelection:
            frequency = 2
            name = 3
            file = 4
            begin = 5
            vTypes = 6
            parameters = 9

    # vaporizer (FALTA CENTER XXXX)

    class vaporizer:
        class create:
            begin = 5
            end = 6
            name = 7
            parameters = 9
            center = 12

        class inspect:
            edge = 2
            name = 3
            begin = 4
            end = 5
            parameters = 7

        class inspectSelection:
            name = 2
            begin = 3
            end = 4
            parameters = 6

    # --------------------------------
    # SHAPE ELEMENTS
    # --------------------------------

    # Poly

    class poly:
        class create:
            id = 5
            name = 6
            colorButton = 7
            color = 8
            fill = 9
            lineWidth = 10
            layer = 11
            type = 12
            imgFileButton = 13
            imgFile = 14
            angle = 15
            geo = 16
            parameters = 18
            parametersButton = 19
            closeShape = 21
            additionalFileButton = 22
            additionalFile = 23
            helpDialog = 24
            reset = 25

        class inspect:
            id = 2
            shape = 3
            name = 4
            colorButton = 5
            color = 6
            fill = 7
            lineWidth = 8
            layer = 9
            type = 10
            imgFileButton = 11
            imgFile = 12
            angle = 13
            geo = 16
            geoshape = 16
            parameters = 18
            parametersButton = 19
            frontButton = 21
            closeShape = 22
            additionalFileButton = 23
            additionalFile = 24
            helpDialog = 25
            reset = 26

        class inspectSelection:
            name = 2
            colorButton = 3
            color = 4
            fill = 5
            lineWidth = 6
            layer = 7
            type = 8
            imgFileButton = 9
            imgFile = 10
            angle = 12
            geo = 14
            parameters = 16
            parametersButton = 17
            frontButton = 19
            closeShape = 20
            additionalFileButton = 21
            additionalFile = 22
            helpDialog = 23
            reset = 24

    # POI

    class poi:
        class create:
            id = 5
            name = 6
            colorButton = 7
            color = 8
            type = 9
            icon = 10
            layer = 12
            width = 13
            height = 14
            imgFileButton = 15
            imgFile = 16
            angle = 17
            parameters = 19
            parametersButton = 20
            additionalFileButton = 22
            additionalFile = 23
            helpDialog = 24
            reset = 25

        class inspect:
            id = 2
            pos = 3
            name = 4
            colorButton = 5
            color = 6
            type = 7
            icon = 8
            layer = 10
            width = 11
            height = 12
            imgFileButton = 13
            imgFile = 14
            angle = 15
            parameters = 17
            parametersButton = 18
            frontButton = 20
            additionalFileButton = 21
            additionalFile = 22
            helpDialog = 23
            reset = 24

        class inspectSelection:
            name = 2
            colorButton = 3
            color = 4
            type = 5
            icon = 6
            layer = 8
            width = 9
            height = 10
            imgFileButton = 11
            imgFile = 12
            angle = 13
            parameters = 15
            parametersButton = 16
            frontButton = 18
            additionalFileButton = 19
            additionalFile = 20
            helpDialog = 21
            reset = 22

    # POIGEO

    class poiGeo:
        class create:
            id = 5
            name = 6
            colorButton = 7
            color = 8
            type = 9
            icon = 10
            layer = 12
            width = 13
            height = 14
            imgFileButton = 15
            imgFile = 16
            angle = 17
            parameters = 19
            parametersButton = 20
            additionalFileButton = 22
            additionalFile = 23
            helpDialog = 24
            reset = 25
            lonLat = 27
            latLon = 28
            coordinates = 29
            center = 30
            createButton = 31

        class inspect:
            id = 2
            name = 3
            colorButton = 4
            color = 5
            type = 6
            icon = 7
            layer = 9
            width = 10
            height = 11
            imgFileButton = 12
            imgFile = 13
            angle = 14
            lon = 16
            lat = 17
            parameters = 19
            parametersButton = 20
            frontButton = 22
            additionalFileButton = 23
            additionalFile = 24
            helpDialog = 25
            reset = 26

        class inspectSelection:
            name = 2
            colorButton = 3
            color = 4
            type = 5
            icon = 6
            layer = 8
            width = 9
            height = 10
            imgFileButton = 11
            imgFile = 12
            angle = 13
            parameters = 15
            parametersButton = 16
            frontButton = 18
            additionalFileButton = 19
            additionalFile = 20
            helpDialog = 21
            reset = 22

    # POILane

    class poiLane:
        class create:
            id = 5
            friendlyPos = 6
            posLat = 7
            name = 8
            colorButton = 9
            color = 10
            type = 11
            icon = 12
            layer = 14
            width = 15
            height = 16
            imgFileButton = 17
            imgFile = 18
            angle = 19
            parameters = 21
            parametersButton = 22
            additionalFileButton = 24
            additionalFile = 25
            helpDialog = 26
            reset = 27

        class inspect:
            id = 2
            lane = 3
            moveLane = 4
            pos = 5
            friendlyPos = 6
            posLat = 7
            name = 8
            colorButton = 9
            color = 10
            type = 11
            icon = 12
            layer = 14
            width = 15
            height = 16
            imgFileButton = 17
            imgFile = 18
            angle = 19
            parameters = 21
            parametersButton = 22
            frontButton = 24
            additionalFileButton = 25
            additionalFile = 26
            helpDialog = 27
            reset = 28

        class inspectSelection:
            friendlyPos = 2
            posLat = 3
            name = 4
            colorButton = 5
            color = 6
            type = 7
            icon = 8
            layer = 10
            width = 11
            height = 12
            imgFileButton = 13
            imgFile = 14
            angle = 15
            parameters = 17
            parametersButton = 18
            frontButton = 20
            additionalFileButton = 21
            additionalFile = 22
            helpDialog = 23
            reset = 24

    # jps walkableArea

    class jpsWalkableArea:
        class create:
            id = 5
            name = 6
            parameters = 8
            parametersButton = 9
            additionalFileButton = 11
            additionalFile = 12
            helpDialog = 13
            reset = 14

        class inspect:
            id = 2
            shape = 3
            name = 4
            parameters = 6
            parametersButton = 7
            frontButton = 9
            additionalFileButton = 10
            additionalFile = 11
            helpDialog = 12
            reset = 13

        class inspectSelection:
            name = 2
            parameters = 4
            parametersButton = 5
            frontButton = 7
            additionalFileButton = 8
            additionalFile = 9
            helpDialog = 10
            reset = 11

    # jps obstacle

    class jpsObstacle:
        class create:
            id = 5
            name = 6
            parameters = 8
            parametersButton = 9
            additionalFileButton = 11
            additionalFile = 12
            helpDialog = 13
            reset = 14

        class inspect:
            id = 2
            shape = 3
            name = 4
            parameters = 6
            parametersButton = 7
            frontButton = 9
            additionalFileButton = 10
            additionalFile = 11
            helpDialog = 12
            reset = 13

        class inspectSelection:
            name = 2
            parameters = 4
            parametersButton = 5
            frontButton = 7
            additionalFileButton = 8
            additionalFile = 9
            helpDialog = 10
            reset = 11

    # --------------------------------
    # TAZ ELEMENTS
    # --------------------------------

    # TAZ

    class TAZ:
        class create:
            id = 3
            center = 4
            name = 5
            colorButton = 6
            color = 7
            fill = 8
            parameters = 10
            parametersButton = 11
            edgesWithin = 14
            additionalFileButton = 15
            additionalFile = 16
            helpDialog = 17
            reset = 18

        class inspect:
            id = 2
            shape = 3
            center = 4
            name = 5
            colorButton = 6
            color = 7
            fill = 8
            parameters = 10
            parametersButton = 11
            frontButton = 13
            additionalFileButton = 14
            additionalFile = 15
            helpDialog = 16
            reset = 17

        class inspectSelection:
            center = 2
            name = 3
            colorButton = 4
            color = 5
            fill = 6
            parameters = 8
            parametersButton = 9
            frontButton = 11
            additionalFileButton = 12
            additionalFile = 13
            helpDialog = 14
            reset = 15

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
            dialog = 24

        class edit:
            id = 9
            vClass = 10
            colorButton = 12
            color = 13
            length = 14
            minGap = 15
            maxSpeed = 16
            desiredMaxSpeed = 17
            parkingBadges = 18
            accel = 19
            decel = 20
            sigma = 21
            tau = 22
            parameters = 24
            parametersButton = 25
            routeFileButton = 27
            routeFile = 28
            helpDialog = 29
            reset = 30

        class editDefault:
            vClass = 8
            colorButton = 10
            color = 11
            length = 12
            minGap = 13
            maxSpeed = 14
            desiredMaxSpeed = 15
            parkingBadges = 16
            accel = 17
            decel = 18
            sigma = 19
            tau = 20
            parameters = 24
            parametersButton = 25
            routeFileButton = 27
            routeFile = 28
            helpDialog = 29
            reset = 30

        class editDefaultModified:
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
            parameters = 25
            parametersButton = 26
            routeFileButton = 28
            routeFile = 29
            helpDialog = 30
            reset = 31

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
            # carriageLength = 33
            # locomotiveLength = 34
            # carriageGap = 35
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
            parameters = 13
            parametersButton = 14
            routeFileButton = 16
            routeFile = 17
            helpDialog = 18
            reset = 19

        class inspect:
            id = 2
            edges = 3
            colorButton = 4
            color = 5
            repeat = 6
            cycletime = 7
            parameters = 9
            parametersButton = 10
            routeFileButton = 12
            routeFile = 13
            helpDialog = 14
            reset = 15

        class inspectSelection:
            colorButton = 2
            color = 3
            repeat = 4
            cycletime = 5
            parameters = 7
            parametersButton = 8
            routeFileButton = 10
            routeFile = 11
            helpDialog = 12
            reset = 13

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
            parameters = 25
            parametersButton = 26
            routeFileButton = 28
            routeFile = 29
            helpDialog = 30
            reset = 31

        class inspect:
            id = 2
            typeButton = 3
            type = 4
            fromEdge = 6
            toEdge = 7
            via = 8
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
            parameters = 25
            parametersButton = 26
            routeFileButton = 28
            routeFile = 29
            helpDialog = 30
            reset = 31

        class inspectSelection:
            typeButton = 2
            type = 3
            colorButton = 5
            color = 6
            departLane = 7
            departPos = 8
            departSpeed = 9
            arrivalLane = 10
            arrivalPos = 11
            arrivalSpeed = 12
            line = 13
            personNumber = 14
            containerNumber = 15
            departPosLat = 16
            arrivalPosLat = 17
            insertionChecks = 18
            depart = 19
            parameters = 21
            parametersButton = 22
            routeFileButton = 24
            routeFile = 25
            helpDialog = 26
            reset = 27

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
            parameters = 25
            parametersButton = 26
            routeFileButton = 28
            routeFile = 29
            helpDialog = 30
            reset = 31

        class inspect:
            id = 2
            typeButton = 3
            type = 4
            fromJunction = 6
            toJunction = 7
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
            parameters = 24
            parametersButton = 25
            routeFileButton = 27
            routeFile = 28
            helpDialog = 29
            reset = 30

        class inspectSelection:
            typeButton = 2
            type = 3
            colorButton = 5
            color = 6
            departLane = 7
            departPos = 8
            departSpeed = 9
            arrivalLane = 10
            arrivalPos = 11
            arrivalSpeed = 12
            line = 13
            personNumber = 14
            containerNumber = 15
            departPosLat = 16
            arrivalPosLat = 17
            insertionChecks = 18
            depart = 19
            parameters = 21
            parametersButton = 22
            routeFileButton = 24
            routeFile = 25
            helpDialog = 26
            reset = 27

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
            parameters = 25
            parametersButton = 26
            routeFileButton = 28
            routeFile = 29
            helpDialog = 30
            reset = 31

        class inspect:
            id = 2
            typeButton = 3
            type = 4
            fromTAZ = 6
            toTAZ = 7
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
            parameters = 24
            parametersButton = 25
            routeFileButton = 27
            routeFile = 28
            helpDialog = 29
            reset = 30

        class inspectSelection:
            typeButton = 2
            type = 3
            colorButton = 5
            color = 6
            departLane = 7
            departPos = 8
            departSpeed = 9
            arrivalLane = 10
            arrivalPos = 11
            arrivalSpeed = 12
            line = 13
            personNumber = 14
            containerNumber = 15
            departPosLat = 16
            arrivalPosLat = 17
            insertionChecks = 18
            depart = 19
            parameters = 21
            parametersButton = 22
            routeFileButton = 24
            routeFile = 25
            helpDialog = 26
            reset = 27

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
            parameters = 25
            parametersButton = 26
            routeFileButton = 28
            routeFile = 29
            helpDialog = 30
            reset = 31

        class inspect:
            id = 2
            typeButton = 3
            type = 4
            route = 6
            departEdge = 7
            arrivalEdge = 8
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
            parameters = 25
            parametersButton = 26
            routeFileButton = 28
            routeFile = 29
            helpDialog = 30
            reset = 31

        class inspectSelection:
            typeButton = 2
            type = 3
            colorButton = 5
            color = 6
            departLane = 7
            departPos = 8
            departSpeed = 9
            arrivalLane = 10
            arrivalPos = 11
            arrivalSpeed = 12
            line = 13
            personNumber = 14
            containerNumber = 15
            departPosLat = 16
            arrivalPosLat = 17
            insertionChecks = 18
            depart = 19
            parameters = 21
            parametersButton = 22
            routeFileButton = 24
            routeFile = 25
            helpDialog = 26
            reset = 27

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
            parameters = 25
            parametersButton = 26
            routeFileButton = 28
            routeFile = 29
            helpDialog = 30
            reset = 31

        class inspect:
            id = 2
            typeButton = 3
            type = 4
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
            parameters = 24
            parametersButton = 25
            routeFileButton = 27
            routeFile = 28
            helpDialog = 29
            reset = 30

        class inspectSelection:
            typeButton = 2
            type = 3
            colorButton = 5
            color = 6
            departLane = 7
            departPos = 8
            departSpeed = 9
            arrivalLane = 10
            arrivalPos = 11
            arrivalSpeed = 12
            line = 13
            personNumber = 14
            containerNumber = 15
            departPosLat = 16
            arrivalPosLat = 17
            insertionChecks = 18
            depart = 19
            parameters = 21
            parametersButton = 22
            routeFileButton = 24
            routeFile = 25
            helpDialog = 26
            reset = 27

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
            terminate = 25
            spacing = 27
            end = 27
            number = 28
            terminateOption = 29
            spacingOption = 30
            parameters = 32
            parametersButton = 33
            routeFileButton = 35
            routeFile = 36
            helpDialog = 37
            reset = 38

        class inspect:
            id = 2
            typeButton = 3
            type = 4
            fromEdge = 6
            toEdge = 7
            via = 8
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
            terminate = 25
            spacing = 27
            end = 27
            number = 28
            terminateOption = 29
            spacingOption = 30
            parameters = 32
            parametersButton = 33
            routeFileButton = 35
            routeFile = 36
            helpDialog = 37
            reset = 38

        class inspectSelection:
            typeButton = 2
            type = 3
            colorButton = 5
            color = 6
            departLane = 7
            departPos = 8
            departSpeed = 9
            arrivalLane = 10
            arrivalPos = 11
            arrivalSpeed = 12
            line = 13
            personNumber = 14
            containerNumber = 15
            departPosLat = 16
            arrivalPosLat = 17
            insertionChecks = 18
            begin = 19
            terminate = 21
            spacing = 23
            end = 23
            number = 24
            terminateOption = 25
            spacingOption = 26
            parameters = 28
            parametersButton = 29
            routeFileButton = 31
            routeFile = 32
            helpDialog = 33
            reset = 34

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
            terminate = 25
            spacing = 27
            end = 27
            number = 28
            terminateOption = 29
            spacingOption = 30
            parameters = 32
            parametersButton = 33
            routeFileButton = 35
            routeFile = 36
            helpDialog = 37
            reset = 38

        class inspect:
            id = 2
            typeButton = 3
            type = 4
            fromJunction = 6
            toJunction = 7
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
            terminate = 24
            spacing = 26
            end = 26
            number = 27
            terminateOption = 28
            spacingOption = 29
            parameters = 31
            parametersButton = 32
            routeFileButton = 34
            routeFile = 35
            helpDialog = 36
            reset = 37

        class inspectSelection:
            typeButton = 2
            type = 3
            colorButton = 5
            color = 6
            departLane = 7
            departPos = 8
            departSpeed = 9
            arrivalLane = 10
            arrivalPos = 11
            arrivalSpeed = 12
            line = 13
            personNumber = 14
            containerNumber = 15
            departPosLat = 16
            arrivalPosLat = 17
            insertionChecks = 18
            begin = 19
            terminate = 21
            spacing = 23
            end = 23
            number = 24
            terminateOption = 25
            spacingOption = 26
            parameters = 28
            parametersButton = 29
            routeFileButton = 31
            routeFile = 32
            helpDialog = 33
            reset = 34

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
            terminate = 25
            spacing = 27
            end = 27
            number = 28
            terminateOption = 29
            spacingOption = 30
            parameters = 32
            parametersButton = 33
            routeFileButton = 35
            routeFile = 36
            helpDialog = 37
            reset = 38

        class inspect:
            id = 2
            typeButton = 3
            type = 4
            fromTAZ = 6
            toTAZ = 7
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
            terminate = 24
            spacing = 26
            end = 26
            number = 27
            terminateOption = 28
            spacingOption = 29
            parameters = 31
            parametersButton = 32
            routeFileButton = 34
            routeFile = 35
            helpDialog = 36
            reset = 37

        class inspectSelection:
            typeButton = 2
            type = 3
            colorButton = 5
            color = 6
            departLane = 7
            departPos = 8
            departSpeed = 9
            arrivalLane = 10
            arrivalPos = 11
            arrivalSpeed = 12
            line = 13
            personNumber = 14
            containerNumber = 15
            departPosLat = 16
            arrivalPosLat = 17
            insertionChecks = 18
            begin = 19
            terminate = 21
            spacing = 23
            end = 23
            number = 24
            terminateOption = 25
            spacingOption = 26
            parameters = 28
            parametersButton = 29
            routeFileButton = 31
            routeFile = 32
            helpDialog = 33
            reset = 34

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
            terminate = 25
            spacing = 27
            end = 27
            number = 28
            terminateOption = 29
            spacingOption = 30
            parameters = 32
            parametersButton = 33
            routeFileButton = 35
            routeFile = 36
            helpDialog = 37
            reset = 38

        class inspect:
            id = 2
            typeButton = 3
            type = 4
            route = 6
            departEdge = 7
            arrivalEdge = 8
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
            terminate = 25
            spacing = 27
            end = 27
            number = 28
            terminateOption = 29
            spacingOption = 30
            parameters = 32
            parametersButton = 33
            routeFileButton = 35
            routeFile = 36
            helpDialog = 37
            reset = 38

        class inspectSelection:
            typeButton = 2
            type = 3
            colorButton = 5
            color = 6
            departLane = 7
            departPos = 8
            departSpeed = 9
            arrivalLane = 10
            arrivalPos = 11
            arrivalSpeed = 12
            line = 13
            personNumber = 14
            containerNumber = 15
            departPosLat = 16
            arrivalPosLat = 17
            insertionChecks = 18
            begin = 19
            terminate = 21
            spacing = 23
            end = 23
            number = 24
            terminateOption = 25
            spacingOption = 26
            parameters = 28
            parametersButton = 29
            routeFileButton = 31
            routeFile = 32
            helpDialog = 33
            reset = 34

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
            terminate = 25
            spacing = 27
            end = 27
            number = 28
            terminateOption = 29
            spacingOption = 30
            parameters = 32
            parametersButton = 33
            routeFileButton = 35
            routeFile = 36
            helpDialog = 37
            reset = 38

        class inspect:
            id = 2
            typeButton = 3
            type = 4
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
            terminate = 24
            spacing = 26
            end = 26
            number = 27
            terminateOption = 28
            spacingOption = 29
            parameters = 31
            parametersButton = 32
            routeFileButton = 34
            routeFile = 35
            helpDialog = 36
            reset = 37

        class inspectSelection:
            typeButton = 2
            type = 3
            colorButton = 5
            color = 6
            departLane = 7
            departPos = 8
            departSpeed = 9
            arrivalLane = 10
            arrivalPos = 11
            arrivalSpeed = 12
            line = 13
            personNumber = 14
            containerNumber = 15
            departPosLat = 16
            arrivalPosLat = 17
            insertionChecks = 18
            begin = 19
            terminate = 21
            spacing = 23
            end = 23
            number = 24
            terminateOption = 25
            spacingOption = 26
            parameters = 28
            parametersButton = 29
            routeFileButton = 31
            routeFile = 32
            helpDialog = 33
            reset = 34

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
            durationEnable = 2
            duration = 3
            untilEnable = 3
            until = 4
            extensionEnable = 4
            extension = 5
            triggered = 6
            expected = 8
            join = 8
            permitted = 8
            parking = 9
            actType = 11
            tripID = 12
            line = 13
            onDemand = 14
            jump = 15
            split = 16

        class inspectSelection:
            durationEnable = 2
            duration = 3
            untilEnable = 4
            until = 5
            extensionEnable = 6
            extension = 7
            triggered = 8
            expected = 10
            join = 10
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
            durationEnable = 2
            duration = 3
            untilEnable = 3
            until = 4
            extensionEnable = 4
            extension = 5
            triggered = 6
            expected = 8
            join = 8
            permitted = 8
            parking = 9
            actType = 11
            tripID = 12
            line = 13
            onDemand = 14
            jump = 15
            split = 16

        class inspectSelection:
            durationEnable = 2
            duration = 3
            untilEnable = 4
            until = 5
            extensionEnable = 6
            extension = 7
            triggered = 8
            expected = 10
            join = 10
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
            durationEnable = 2
            duration = 3
            untilEnable = 3
            until = 4
            extensionEnable = 4
            extension = 5
            triggered = 6
            expected = 8
            join = 8
            permitted = 8
            parking = 9
            actType = 11
            tripID = 12
            line = 13
            onDemand = 14
            jump = 15
            split = 16

        class inspectSelection:
            durationEnable = 2
            duration = 3
            untilEnable = 4
            until = 5
            extensionEnable = 6
            extension = 7
            triggered = 8
            expected = 10
            join = 10
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
            durationEnable = 2
            duration = 3
            untilEnable = 3
            until = 4
            extensionEnable = 4
            extension = 5
            triggered = 6
            expected = 8
            join = 8
            permitted = 8
            parking = 9
            actType = 11
            tripID = 12
            line = 13
            onDemand = 14
            jump = 15
            split = 16

        class inspectSelection:
            durationEnable = 2
            duration = 3
            untilEnable = 4
            until = 5
            extensionEnable = 6
            extension = 7
            triggered = 8
            expected = 10
            join = 10
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
            durationEnable = 2
            duration = 3
            untilEnable = 3
            until = 4
            extensionEnable = 4
            extension = 5
            triggered = 6
            expected = 8
            join = 8
            permitted = 8
            actType = 9
            tripID = 10
            line = 11
            onDemand = 12
            jump = 13
            split = 14

        class inspectSelection:
            durationEnable = 2
            duration = 3
            untilEnable = 4
            until = 5
            extensionEnable = 6
            extension = 7
            triggered = 8
            expected = 10
            join = 10
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
            lane = 2
            moveLane = 3
            startPos = 4
            endPos = 5
            friendlyPos = 6
            posLat = 7
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

        class inspectSelection:
            friendlyPos = 2
            posLat = 3
            durationEnable = 4
            duration = 5
            untilEnable = 6
            until = 7
            extensionEnable = 8
            extension = 9
            triggered = 10
            expected = 12
            join = 12
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
            durationEnable = 2
            duration = 3
            untilEnable = 3
            until = 4
            extensionEnable = 4
            extension = 5
            permitted = 6
            parking = 7
            actType = 9
            tripID = 10
            line = 11
            speed = 12
            jump = 13
            split = 14

        class inspectSelection:
            durationEnable = 2
            duration = 3
            untilEnable = 4
            until = 5
            extensionEnable = 6
            extension = 7
            permitted = 8
            parking = 9
            actType = 11
            tripID = 12
            line = 13
            speed = 14
            jump = 15
            split = 16

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
            durationEnable = 2
            duration = 3
            untilEnable = 3
            until = 4
            extensionEnable = 4
            extension = 5
            permitted = 6
            parking = 7
            actType = 9
            tripID = 10
            line = 11
            speed = 12
            jump = 13
            split = 14

        class inspectSelection:
            durationEnable = 2
            duration = 3
            untilEnable = 4
            until = 5
            extensionEnable = 6
            extension = 7
            permitted = 8
            parking = 9
            actType = 11
            tripID = 12
            line = 13
            speed = 14
            jump = 15
            split = 16

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
            durationEnable = 2
            duration = 3
            untilEnable = 3
            until = 4
            extensionEnable = 4
            extension = 5
            permitted = 6
            parking = 7
            actType = 9
            tripID = 10
            line = 11
            speed = 12
            jump = 13
            split = 14

        class inspectSelection:
            durationEnable = 2
            duration = 3
            untilEnable = 4
            until = 5
            extensionEnable = 6
            extension = 7
            permitted = 8
            parking = 9
            actType = 11
            tripID = 12
            line = 13
            speed = 14
            jump = 15
            split = 16

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
            durationEnable = 2
            duration = 3
            untilEnable = 3
            until = 4
            extensionEnable = 4
            extension = 5
            permitted = 6
            parking = 7
            actType = 9
            tripID = 10
            line = 11
            speed = 12
            jump = 13
            split = 14

        class inspectSelection:
            durationEnable = 2
            duration = 3
            untilEnable = 4
            until = 5
            extensionEnable = 6
            extension = 7
            permitted = 8
            parking = 9
            actType = 11
            tripID = 12
            line = 13
            speed = 14
            jump = 15
            split = 16

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
            durationEnable = 2
            duration = 3
            untilEnable = 3
            until = 4
            extensionEnable = 4
            extension = 5
            permitted = 6
            actType = 7
            tripID = 8
            line = 9
            speed = 10
            jump = 11
            split = 12

        class inspectSelection:
            durationEnable = 2
            duration = 3
            untilEnable = 4
            until = 5
            extensionEnable = 6
            extension = 7
            permitted = 8
            actType = 9
            tripID = 10
            line = 11
            speed = 12
            jump = 13
            split = 14

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
            lane = 2
            moveLane = 3
            startPos = 4
            endPos = 5
            friendlyPos = 6
            posLat = 7
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

        class inspectSelection:
            friendlyPos = 2
            posLat = 3
            durationEnable = 4
            duration = 5
            untilEnable = 6
            until = 7
            extensionEnable = 8
            extension = 9
            permitted = 10
            parking = 11
            actType = 13
            tripID = 14
            line = 15
            speed = 16
            jump = 17
            split = 18

    # Person

    class person:
        class create:
            id = 8
            colorButton = 9
            color = 10
            departPos = 11
            depart = 12

        class inspect:
            id = 2
            typeButton = 3
            type = 4
            colorButton = 6
            color = 7
            departPos = 8
            depart = 9
            parameters = 11

        class inspectSelection:
            typeButton = 2
            type = 3
            colorButton = 5
            color = 6
            departPos = 7
            depart = 8
            parameters = 10

    class personFlow:
        class create:
            id = 8
            colorButton = 9
            color = 10
            departPos = 11
            begin = 12
            terminate = 14
            end = 16
            number = 17
            spacing = 16
            terminateOption = 18
            spacingOption = 19

        class inspect:
            id = 2
            typeButton = 3
            type = 4
            colorButton = 6
            color = 7
            departPos = 8
            begin = 9
            terminate = 11
            spacing = 13
            end = 14
            terminateOption = 15
            spacingOption = 16
            parameters = 18

        class inspectSelection:
            typeButton = 2
            type = 3
            colorButton = 5
            color = 6
            departPos = 7
            begin = 8
            terminate = 10
            spacing = 12
            terminateOption = 14
            spacingOption = 15
            parameters = 17

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
                    arrivalPos = 2
                    vTypes = 3
                    modes = 4
                    lines = 5
                    walkFactor = 6
                    group = 7

                class inspectSelection:
                    arrivalPos = 2
                    vTypes = 3
                    modes = 4
                    lines = 5
                    walkFactor = 6
                    group = 7

            class busStop:
                class create:
                    arrivalPos = 8
                    vTypes = 9
                    modes = 10
                    lines = 11
                    walkFactor = 12
                    group = 13

                class inspect:
                    vTypes = 2
                    modes = 3
                    lines = 4
                    walkFactor = 5
                    group = 6

                class inspectSelection:
                    vTypes = 2
                    modes = 3
                    lines = 4
                    walkFactor = 5
                    group = 6

            class junction:
                class create:
                    arrivalPos = 8
                    vTypes = 9
                    modes = 10
                    lines = 11
                    walkFactor = 12
                    group = 13

                class inspect:
                    vTypes = 2
                    modes = 3
                    lines = 4
                    walkFactor = 5
                    group = 6

                class inspectSelection:
                    vTypes = 2
                    modes = 3
                    lines = 4
                    walkFactor = 5
                    group = 6

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
                    arrivalPos = 2
                    vTypes = 3
                    modes = 4
                    lines = 5
                    walkFactor = 6
                    group = 7

                class inspectSelection:
                    arrivalPos = 2
                    vTypes = 3
                    modes = 4
                    lines = 5
                    walkFactor = 6
                    group = 7

            class busStop:
                class create:
                    arrivalPos = 8
                    vTypes = 9
                    modes = 10
                    lines = 11
                    walkFactor = 12
                    group = 13

                class inspect:
                    vTypes = 2
                    modes = 3
                    lines = 4
                    walkFactor = 5
                    group = 6

                class inspectSelection:
                    vTypes = 2
                    modes = 3
                    lines = 4
                    walkFactor = 5
                    group = 6

            class junction:
                class create:
                    arrivalPos = 8
                    vTypes = 9
                    modes = 10
                    lines = 11
                    walkFactor = 12
                    group = 13

                class inspect:
                    vTypes = 2
                    modes = 3
                    lines = 4
                    walkFactor = 5
                    group = 6

                class inspectSelection:
                    vTypes = 2
                    modes = 3
                    lines = 4
                    walkFactor = 5
                    group = 6

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
                    arrivalPos = 2
                    vTypes = 3
                    modes = 4
                    lines = 5
                    walkFactor = 6
                    group = 7

                class inspectSelection:
                    arrivalPos = 2
                    vTypes = 3
                    modes = 4
                    lines = 5
                    walkFactor = 6
                    group = 7

            class busStop:
                class create:
                    arrivalPos = 8
                    vTypes = 9
                    modes = 10
                    lines = 11
                    walkFactor = 12
                    group = 13

                class inspect:
                    vTypes = 2
                    modes = 3
                    lines = 4
                    walkFactor = 5
                    group = 6

                class inspectSelection:
                    vTypes = 2
                    modes = 3
                    lines = 4
                    walkFactor = 5
                    group = 6

            class junction:
                class create:
                    arrivalPos = 8
                    vTypes = 9
                    modes = 10
                    lines = 11
                    walkFactor = 12
                    group = 13

                class inspect:
                    vTypes = 2
                    modes = 3
                    lines = 4
                    walkFactor = 5
                    group = 6

                class inspectSelection:
                    vTypes = 2
                    modes = 3
                    lines = 4
                    walkFactor = 5
                    group = 6

    class ride:
        class edge:
            class edge:
                class create:
                    arrivalPos = 8
                    lines = 9
                    group = 10

                class inspect:
                    arrivalPos = 2
                    lines = 3
                    group = 4

                class inspectSelection:
                    arrivalPos = 2
                    lines = 3
                    group = 4

            class busStop:
                class create:
                    arrivalPos = 8
                    lines = 9
                    group = 10

                class inspect:
                    arrivalPos = 2
                    lines = 3
                    group = 4

                class inspectSelection:
                    arrivalPos = 2
                    lines = 3
                    group = 4

            class junction:
                class create:
                    arrivalPos = 8
                    lines = 9
                    group = 10

                class inspect:
                    arrivalPos = 2
                    lines = 3
                    group = 4

                class inspectSelection:
                    arrivalPos = 2
                    lines = 3
                    group = 4

        class busStop:
            class edge:
                class create:
                    arrivalPos = 8
                    lines = 9
                    group = 10

                class inspect:
                    arrivalPos = 2
                    lines = 3
                    group = 4

                class inspectSelection:
                    arrivalPos = 2
                    lines = 3
                    group = 4

            class busStop:
                class create:
                    arrivalPos = 8
                    lines = 9
                    group = 10

                class inspect:
                    arrivalPos = 2
                    lines = 3
                    group = 4

                class inspectSelection:
                    arrivalPos = 2
                    lines = 3
                    group = 4

            class junction:
                class create:
                    arrivalPos = 8
                    lines = 9
                    group = 10

                class inspect:
                    arrivalPos = 2
                    lines = 3
                    group = 4

                class inspectSelection:
                    arrivalPos = 2
                    lines = 3
                    group = 4

        class junction:
            class edge:
                class create:
                    arrivalPos = 8
                    lines = 9
                    group = 10

                class inspect:
                    arrivalPos = 2
                    lines = 3
                    group = 4

                class inspectSelection:
                    arrivalPos = 2
                    lines = 3
                    group = 4

            class busStop:
                class create:
                    arrivalPos = 8
                    lines = 9
                    group = 10

                class inspect:
                    arrivalPos = 2
                    lines = 3
                    group = 4

                class inspectSelection:
                    arrivalPos = 2
                    lines = 3
                    group = 4

            class junction:
                class create:
                    arrivalPos = 8
                    lines = 9
                    group = 10

                class inspect:
                    arrivalPos = 2
                    lines = 3
                    group = 4

                class inspectSelection:
                    arrivalPos = 2
                    lines = 3
                    group = 4

    class walk:
        class edge:
            class edge:
                class create:
                    arrivalPos = 8
                    speed = 9
                    duration = 10

                class inspect:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

                class inspectSelection:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

            class busStop:
                class create:
                    arrivalPos = 8
                    speed = 9
                    duration = 10

                class inspect:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

                class inspectSelection:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

            class junction:
                class create:
                    arrivalPos = 8
                    speed = 9
                    duration = 10

                class inspect:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

                class inspectSelection:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

        class busStop:
            class edge:
                class create:
                    arrivalPos = 8
                    speed = 9
                    duration = 10

                class inspect:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

                class inspectSelection:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

            class busStop:
                class create:
                    arrivalPos = 8
                    speed = 9
                    duration = 10

                class inspect:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

                class inspectSelection:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

            class junction:
                class create:
                    arrivalPos = 8
                    speed = 9
                    duration = 10

                class inspect:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

                class inspectSelection:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

        class junction:
            class edge:
                class create:
                    arrivalPos = 8
                    speed = 9
                    duration = 10

                class inspect:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

                class inspectSelection:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

            class busStop:
                class create:
                    arrivalPos = 8
                    speed = 9
                    duration = 10

                class inspect:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

                class inspectSelection:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

            class junction:
                class create:
                    arrivalPos = 8
                    speed = 9
                    duration = 10

                class inspect:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

                class inspectSelection:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

    class walkEdges:
        class create:
            arrivalPos = 8

        class inspect:
            arrivalPos = 2

        class inspectSelection:
            arrivalPos = 2

    class walkRoute:
        class create:
            arrivalPos = 8

        class inspect:
            arrivalPos = 2

        class inspectSelection:
            arrivalPos = 2

    # Container

    class container:
        class create:
            id = 8
            colorButton = 9
            color = 10
            departPos = 11
            depart = 12

        class inspect:
            id = 2
            typeButton = 3
            type = 4
            colorButton = 6
            color = 7
            departPos = 8
            depart = 9
            parameters = 11

        class inspectSelection:
            typeButton = 2
            type = 3
            colorButton = 5
            color = 6
            departPos = 7
            depart = 8
            parameters = 10

    class containerFlow:
        class create:
            id = 8
            colorButton = 9
            color = 10
            departPos = 11
            begin = 12
            terminate = 14
            end = 16
            number = 17
            spacing = 16
            terminateOption = 18
            spacingOption = 19

        class inspect:
            id = 2
            typeButton = 3
            type = 4
            colorButton = 6
            color = 7
            departPos = 8
            begin = 9
            terminate = 11
            spacing = 13
            end = 14
            terminateOption = 15
            spacingOption = 16
            parameters = 18

        class inspectSelection:
            typeButton = 2
            type = 3
            colorButton = 5
            color = 6
            departPos = 7
            begin = 8
            terminate = 10
            spacing = 12
            terminateOption = 14
            spacingOption = 15
            parameters = 17

    # PersonPlans

    class transport:
        class edge:
            class edge:
                class create:
                    arrivalPos = 8
                    lines = 9
                    group = 10

                class inspect:
                    arrivalPos = 2
                    lines = 3
                    group = 4

                class inspectSelection:
                    arrivalPos = 2
                    lines = 3
                    group = 4

            class busStop:
                class create:
                    arrivalPos = 8
                    lines = 9
                    group = 10

                class inspect:
                    lines = 2
                    group = 3

                class inspectSelection:
                    lines = 2
                    group = 3

            class junction:
                class create:
                    arrivalPos = 8
                    lines = 9
                    group = 10

                class inspect:
                    lines = 2
                    group = 3

                class inspectSelection:
                    lines = 2
                    group = 3

        class busStop:
            class edge:
                class create:
                    arrivalPos = 8
                    lines = 9
                    group = 10

                class inspect:
                    arrivalPos = 2
                    lines = 3
                    group = 4

                class inspectSelection:
                    arrivalPos = 2
                    lines = 3
                    group = 4

            class busStop:
                class create:
                    arrivalPos = 8
                    lines = 9
                    group = 10

                class inspect:
                    lines = 2
                    group = 3

                class inspectSelection:
                    lines = 2
                    group = 3

            class junction:
                class create:
                    arrivalPos = 8
                    lines = 9
                    group = 10

                class inspect:
                    lines = 2
                    group = 3

                class inspectSelection:
                    lines = 2
                    group = 3

        class junction:
            class edge:
                class create:
                    arrivalPos = 8
                    lines = 9
                    group = 10

                class inspect:
                    arrivalPos = 2
                    lines = 3
                    group = 4

                class inspectSelection:
                    arrivalPos = 2
                    lines = 3
                    group = 4

            class busStop:
                class create:
                    arrivalPos = 8
                    lines = 9
                    group = 10

                class inspect:
                    lines = 2
                    group = 3

                class inspectSelection:
                    lines = 2
                    group = 3

            class junction:
                class create:
                    arrivalPos = 8
                    lines = 9
                    group = 10

                class inspect:
                    lines = 2
                    group = 3

                class inspectSelection:
                    lines = 2
                    group = 3

    class tranship:
        class edge:
            class edge:
                class create:
                    arrivalPos = 8
                    speed = 9
                    duration = 10

                class inspect:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

                class inspectSelection:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

            class busStop:
                class create:
                    arrivalPos = 8
                    speed = 9
                    duration = 10

                class inspect:
                    speed = 2
                    duration = 3

                class inspectSelection:
                    speed = 2
                    duration = 3

            class junction:
                class create:
                    arrivalPos = 8
                    speed = 9
                    duration = 10

                class inspect:
                    speed = 2
                    duration = 3

                class inspectSelection:
                    speed = 2
                    duration = 3

        class busStop:
            class edge:
                class create:
                    arrivalPos = 8
                    speed = 9
                    duration = 10

                class inspect:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

                class inspectSelection:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

            class busStop:
                class create:
                    arrivalPos = 8
                    speed = 9
                    duration = 10

                class inspect:
                    speed = 2
                    duration = 3

                class inspectSelection:
                    speed = 2
                    duration = 3

            class junction:
                class create:
                    speed = 8
                    duration = 9

                class inspect:
                    speed = 2
                    duration = 3

                class inspectSelection:
                    speed = 2
                    duration = 3

        class junction:
            class edge:
                class create:
                    arrivalPos = 8
                    speed = 9
                    duration = 10

                class inspect:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

                class inspectSelection:
                    arrivalPos = 2
                    speed = 3
                    duration = 4

            class busStop:
                class create:
                    arrivalPos = 8
                    speed = 9
                    duration = 10

                class inspect:
                    speed = 2
                    duration = 3

                class inspectSelection:
                    speed = 2
                    duration = 3

            class junction:
                class create:
                    arrivalPos = 8
                    speed = 9
                    duration = 10

                class inspect:
                    speed = 2
                    duration = 3

                class inspectSelection:
                    speed = 2
                    duration = 3

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
                endPos = 2
                durationEnable = 3
                duration = 4
                untilEnable = 5
                until = 6
                actType = 6
                friendlyPos = 7

            class inspectSelection:
                durationEnable = 2
                duration = 3
                untilEnable = 4
                until = 5
                actType = 6
                friendlyPos = 7

        class stoppingPlace:
            class create:
                durationEnable = 8
                duration = 9
                untilEnable = 10
                until = 11
                actType = 11

            class inspect:
                durationEnable = 2
                duration = 3
                untilEnable = 4
                until = 5
                actType = 5

            class inspectSelection:
                durationEnable = 2
                duration = 3
                untilEnable = 4
                until = 5
                actType = 5

    # --------------------------------
    # DATA ELEMENTS
    # --------------------------------

    # edgeData

    class edgeData:
        class create:
            parameters = 12

        class inspect:
            parameters = 4

        class inspectSelection:
            parameters = 2

    # edgeRelData

    class edgeRelData:
        class create:
            parameters = 12

        class inspect:
            fromEdge = 2
            toEdge = 3
            parameters = 6

        class inspectSelection:
            parameters = 2

    # TAZRelData

    class TAZRelData:
        class create:
            parameters = 12

        class inspect:
            fromTAZ = 2
            toTAZ = 3
            parameters = 6

        class inspectSelection:
            parameters = 2

    # edge mean data

    class edgeMeanData:

        class inspect:
            id = 7
            fileButton = 8
            file = 9
            period = 10
            begin = 11
            end = 12
            excludeEmpty = 13
            withInternal = 15
            maxTravelTime = 16
            minSamples = 17
            speedThreshold = 18
            vTypes = 19
            trackVehicles = 20
            detectPersons = 21
            writeAttributes = 22
            edges = 23
            edgesFile = 24
            aggregate = 25

    # lane mean data

    class laneMeanData:

        class inspect:
            id = 7
            fileButton = 8
            file = 9
            period = 10
            begin = 11
            end = 12
            excludeEmpty = 13
            withInternal = 15
            maxTravelTime = 16
            minSamples = 17
            speedThreshold = 18
            vTypes = 19
            trackVehicles = 20
            detectPersons = 21
            writeAttributes = 22
            edges = 23
            edgesFile = 24
            aggregate = 25

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
