---
title: ChangeLog
---

## Git Main

### Bugfixes

- sumo
  - Fixed rerouting error on the last route edge with a stop #15552
  - Fixed routing error on departure #15563
  - Fixed invalid warnings regarding inconsistent loaded stop times #15602
  - Fixed blocked vehicles at busStop with attribute parkingLength #15644
  - Fixed train collision at rail_crossing #15658 

- netedit
  - Loaded containers starting from stops are now drawn #15567
  - ESC aborts creation of edgeRel and tazRel datas #15601
  - Fixed invalid TAZ coloring during mouse hovering in create TAZRel mode #15544
  - Simplified creation of laneArea detectors on short lanes #15142
  - Vehicles placed in elevated lanes can now be moved #15367
  - Fixed invalid route shown for non-default departLane #15056
  - Improve geometry of embedded routes #13980 
  - Fixed Crash when editing options before loading a network #13881
  - Fixed text rendering of small elements when zoomed out #15185
  - Selectable area of a POI doesn't now corresponds to custom radius #15532 
  - Fixed crash after undo-redo due to distributions #15642
  - Fixed invalid behavior after pressing ESC key in person plan mode #15193
  - Modified default vType is no longer written after being reset #14985
  - data mode: create interval elements are disabled until a dataset has been selected #13309
  - "Show geometry point indices" option is now working #13969
  - Turnaround connections are now visible for bidi-rail edges #14486
  - Moving a junction now moves it's custom shape #15456
  - Fixed missing/strange attributes while adding routeprobe #15268
  - Can now load trips without from / to #15074 

- netconvert 
  - Fixed invalid sign of geo-coordinate offset in OpenDRIVE input and output #15624 

- TraCI
  - Fixed crash when calling `vehicle.getNextLinks` and `lane.getLinks` at junction type `allway_stop` or `priority_stop` #15603 (regression in 1.21.0)
    
### Enhancements

- sumo
  - The new vType attribute `lcContRight` can be used to configure lane choice at a lane split where all lanes have equal strategic value. #15579
  - Added option **--insertion-checks** to set global defaults for vehicle attribute `insertionChecks` #15149
  - Added option **--pedestrian.striping.jamfactor** to configure the speed of jammed pedestrians (default 0.25) #15610
  - GLOSA Device now looks several phases into the future and can also take queues into account #15614
  - Added new vType attributes `jmAdvance` and `jmExtraGap` to configure the behavior on junctions for crossing and merging streams of traffic #15654
  - Added new insertion behavior `departLane="best_prob"` to increase throughput on multi-lane roads #15661 
  - railways
    - major rewrite of signal logic #7578
    - major improvement in railway simulation speed (simulation time reduced by ~50-75% depending on scenario size) #4379 
    - Fixed various deadlocks #7493, #13262, #15474
    - The new option **--time-to-teleport.railsignal-deadlock** can be used to detect signal based deadlocks #15561    
    - The new optioin **--time-to-teleport.remove-constraint** can be used to resolve detected deadlocks which are caused by a signal constraint by deactivating a responsible constraint #14543
    - The new option **--deadlock-output** can be use to log detected deadlocks and also their resolution
    - Logged deadlocks can be loaded as additional file to prevent them in a subsequent simulation #15569 

- netedit
  - Additional output now writes chargingStation after parkingArea elements #15628
  - Removed "invert edges" from GNECrossingFrame #15129
 
- sumo-gui
  - The value of SUMO_HOME is now shown in the *About Dialog* (also for netedit) #15218
 
- netconvert
  - Added support for zipped shape files #15623

### Miscellaneous

- Added analysis on the effects of attribute `departLane` on [insertion capacity](Simulation/RoadCapacity.md#insertion_capacity_on_a_2-lane_road).


## Version 1.21.0 (10.10.2024)

### Bugfixes

- sumo
  - Fixed unhelpful error message when giving an edge id instead of a lane id for a stop #15158 (regression in 1.11.0)
  - Fixed invalid error when loading a trip directly after a route with `repeat` #14992 (regression in 1.20.0)
  - Persons joining edge via access don't collide with vehicles again #15030 (regression in 1.20.0)
  - personInfo total duration now includes ride waiting time #14887
  - Fixed jump-related bug when there are two stops on the same edge #14883
  - Fixed saving/loading of maximum time value from state #14904
  - Fixed creeping vehicle approach to oncoming pedestrian on shared space #14920
  - Fixed invalid busStop id in warning "aborted waiting at busStop" #14929
  - Fixed emergency braking at pedestrian crossings #14939
  - Fixed emergency braking at pedestrian crossing in subsecond simulation #14928
  - Fixed emergency braking for right-turning traffic at prioritized pedestrian crossing #14952
  - Fixed emergency braking due to unsafe insertion speed #14941
  - Fixed unsafe insertion speed before prioritized pedestrian crossing when using **--step-method.ballistic**. #14951
  - Fixed lane-changing related deadlock around stopped vehicles #14322
  - Fixed crash when parallel routing encounters invalid trips #14963
  - Fixed handling of passed stops after rerouting #14918
  - Fixed crash when using junction-taz in railway simulation #14973
  - Fixed non-deterministic behavior with option **--device.rerouting.threads --weights.random-factor** #10292
  - Electric vehicles now issues a warning on running out of battery power #14980
  - Fixed crash when loading misconfigured NEMA controller #15001
  - Fixed invalid stopping when loading a route with attribute `repeat` and a single stop #13639
  - Fixed invalid density in edgedata-output in sublane simulation #15017
  - Fixed invalid behavior by GLOSA device when there are junctions ahead of a traffic light within **device.glosa.range** #15011
  - Removal of JuPedSim-pedestrians in vanishing area with period < 1 is now working. #14900
  - No more cooperative speed adaptation when setting lcCooperative=-1. #14871
  - Fixed invalid initial blinker for continuous lanechange in lefthand network #15058
  - Now avoiding orthogonal sliding in continous lane change model with attribute `lcMaxSpeedLatStanding="0"` #15052
  - GLOSA device now follows intended 3-phase speed trajectory (decel, constant, accel) instead of (decel, accel) #15067
  - Fixed negative emissions with HBEFA4 model #15079
  - Fixed duplicate 'started' and 'ended' values in vehroute-output #15086
  - Fixed invalid taxi state when receiving a new dispatch while parking #15080
  - Fixed invalid stopping when a route with a single stop uses attribute `repeat` #13639
  - Fixed invalid rerouting one edge with change prohibitions #15098
  - Fixed collision in roundabout in network without internal links #15100
  - Fixed invalid GLOSA behavior when loading multiple programs for a traffic light #15139
  - Fixed crash during portion working #15165
  - Fixed skipped stop when train is joined by a vehicle on another edge #15166
  - Fixed high memory usage in simulation with chargingStations when not using **--chargingstations-output** #15238
  - Fixed bug where simulation does not finish with onDemand stop #15263
  - Fixed queue length estimation for laneAreaDetectors of delay_based traffic lights #15247
  - Fixed invalid laneAreaDetector end position checks #15275
  - Fixed bug where vehicle skips stop after waypoint #15293
  - Fixed emergency braking at zipper with low visibilityDistance #15224
  - Fixed bug where parking rerouter does not reroute occupants #15276
  - Fixed crash when using rerouters on vehicles with stops and looped routes #15290
  - Fixed low insertion speed for `departSpeed="max"` with waypoint #15308
  - Fixed nondeterministic order of constraint trackers in saved state #15406
  - Fixed infinite loop if rerouter interval end < begin #15416
  - Fixed bug where vType-param "device.rerouting.probability" could not be used to prevent rerouting #15288
  - edgeData output now excludes non-driving lanes from laneDensity computation #15383
  - rerouter closingReroute now preserves all existing stops when computing new route #14610
  - Fixed invalid emergency stop message for waypoint at route end #15485
  - Fixed bug where taxi disappears during dispatch #15508
  - Fixed crash when loading state related to rerouting device assignment #15517
  - Now updating via edges when rerouting to an alternative parkingArea #15545
  - StartUpDelay and ActionStepLength can now be used together #14229
  - StartUpDelay now works with small timesteps and IDM #14289
  - Fixed crash when using taxi with pre-booking and ride sharing #15385

- netconvert
  - Fixed crash when guessing ramps #14836 (regression in 1.20.0)
  - Fixed crash when edge removal affects roundabouts defined in OSM. #14930 (regression in 1.20.0)
  - OSM: fixed invalid bus permission on tram line #15301 (regression in 1.20.0)
  - Fixed bad ramp geometry when two ramps overlap #14839
  - Fixed invalid connection with option **--ramps.guess** if the input defines connections #14833
  - Fixed unsafe waiting position within a junction that conflicts with a pedestrian crossing #14946
  - Fixed invalid NEMA controllers in written network #15023
  - Fixed signal precision, signal lane validity and 0-length edges in xodr output. #15078
  - Fixed ugly junction shape #15103
  - Fixed self overlapping crossing outline shape #15111
  - Crossing parameters are now loaded from plain-xml #15135
  - Fixed double-projection of node and crossing shapes when setting option **--proj** #15136, #15137
  - Option **--railway.signal.guess.by-stops** can no longer turn switches into signals #15153
  - Fixed invalid pedestrian infrastructure connections that could cause invalid pedestrian routes #15177, #15183
  - Fixed broken lane shape in opendrive import #15197
  - Matsim import now handles node ids with unusual characters #15196
  - Fixed crash during opendrive import with boundary filter #15243
  - Fixed crash when using tlLayout="alternateOneWay" with crossings #14027
  - Connections from the same edge to the same target edge that cross over each other are now recognized as being in conflict. #15257

- netedit
  - Polygon context menu functions *simplify shape* and *close shape* are working again #14857 (regression in 1.6.0)
  - Custom junction shape polygon now has the correct context menu #14858 (regression in 1.6.0)
  - Fixed crash when attempting to transform person to personFlow #15040 (regression in 1.19.0)
  - Fixed Crash during undo-redo of stops/waypoints over stoppingPlaces #15249 (regression in 1.19.0)
  - Fixed crash when editing person and container types for a selection #15255 (regression in 1.19.0)
  - Fixed junction highlighting on hovering in edge mode #15413 (regression in 1.19.0)
  - Fixed invalid "embededRoute" element in view #15492 (regression in 1.19.0)
  - Vehicle type attribute dialog: guiShape combobox is now working with default vClass #14859  (regression in 1.19.0)
  - Fixed invalid tls link indices #14949 (regression in 1.20.0)
  - Fixed inspecting and selecting elements in data mode #14999 (regression in 1.20.0)
  - Fixed bug where additionals were not saved when saving a NeteditConfig #14739 (regression in 1.20.0)
  - Button "center" in "locate additional" dialog is working again #14986 (regression in 1.20.0)
  - Selecting lanes and edges via context menu is working again #14965 (regression in 1.20.0)
  - Inspected trips show the route again #14964 (regression in 1.20.0)
  - Selection rectangle now considers element locks again. #15010 (regression in 1.20.0)
  - TazRelations can be inspected again #15019 (regression in 1.20.0)
  - Fixed crash when joining edges with crossings #15073 (regression in 1.20.0)
  - Fixed crash when removing junction with crossing in loaded network #15112 (regression in 1.20.0)
  - Additional objects no longer disappear at certain zoom levels #15104 (regression in 1.20.0)
  - Fixed invalid dotted contour around calibrators #15138 (regression in 1.20.0)
  - Fixed click detection to rerouter/vaporizer and stoppingPlace symbols #15280 (regression in 1.20.0)
  - Fixed coloring of selected Persons and containers #15254 (regression in 1.20.0)
  - Fixed crash when creating tranships with different speed #15339 (regression in 1.20.0)
  - Fixed crash related to invalid endOffset #15317 (regression in 1.20.0)
  - Fixed invalid coloring for lane change prohibition #15099 (regression in 1.20.0)
  - Fixed highlighting during POIs and parkingSpace movement #15509, #15512 (regression in 1.20.0)
  - Objects with enabled text rendering (i.e. ID) are always visible at any zoom #15519 (regression in 1.20.0)
  - Fixed crash when using "Replace junction by geometry point" and traffic demand is loaded #14863
  - In inspect mode, the junction contour no longer hides link indices #14948
  - Fixed invalid NEMA controller when changing type from static #15007
  - Fixed invalid junction contour #15002, #14488
  - Fixed crash when loading another network while the traffic light frame has an active traffic light #15004
  - Fixed uncontrolled pedestrian crossings after using 'Clean States' #15047
  - Undo-button becomes instantly active after undoable action #14770
  - Edge context menu function *apply edge template* now applies to the whole selection #14458
  - Rerouters and VSS without edges or lanes are no longer written #15151
  - Hotkey <kbd>BACKSPACE</kbd> now works for taking back a stopping place in person mode #14802
  - Fixed crash when replacing junction by geometry point #15233
  - edgeData attribute `writeAttributes` is now written correctly #15273
  - Fixed error creating POI over lanes #15042
  - Can now parse sumocfg with human readable times #15267
  - Fixed problem when setting stopOffsetException in multiple edges #15297
  - Fixed crash when replacing junction by geometry point in network with person trips #15286
  - Fixed containers creation on top of an existing plan #15344
  - Can now select/inspect/delete person stops clicking over sign #15350
  - Fixed crash recomputing network with volatile recomputing and loaded vehicles #15356
  - Can now create rides with only one edge #15361
  - Fixed clicking Person/container plans over TAZs #15363
  - Fixed crash trying to move persons over TAZ #15365
  - Fixed crash joining junctions with crossings #15328
  - Fixed loss of TAZ edges after recomputing with volatile options #15401
  - Stop attribute `parking` now takes effect when set in *Stop mode* #15439
  - Crossings are now preserved when converting a junction to a roundabout #15231
  - Hotkey ESC now aborts creation of edgeRelation #15228
  - Right click over geometry point now opens edge context menu #15322
  - Fixed invalid "Cursor position in view" values in contextual menu for E2 multilane detectors #15064
  - Fixed crash changing type inspecting multiple containers #14081
  - Persons and container may now use any kind of stopping place #12384
  - Target highlighting in person and container modes now works when mouse is over another person / container #14803
  - Fixed slow-down when loading many polygons #14600
  - Fixed junction merging with active grid #15483
  - Can now load laneAreaDetector defined with endPos #14683
  - Copy template now copies the changeLeft/changeRight attributes #15507
  - Contour of non-filled polygons is now reset after moving #15541
  - Fixed crash when attempting to create a joined NEMA controller #15547
  - Fixed lefthand drawing of additional elements #15566

- sumo-gui
  - Reloading now works if SUMO_HOME is not set #14830 (regression in 1.15.0)
  - Fixed glitch when drawing carriages on edges with custom length. #15075 (regression in 1.17.0)
  - Fixed crash when saving view settings to registry #14909 (regression in 1.19.0)
  - Fixed bad menu spacing and tiny icons when using display scaling on windows (i.e. for high-pixel-count displays) #14882
  - Fixed translation-related crashes where code relied on hard coded english menu entries. #14710
  - Fixed placement of chargingStation and ParkingArea symbol in lefthand networks #14744, #14959, #15110
  - Fixed invalid angle for infrastructure text label in lefthand networks #14743, #14955
  - Fixed invalid scaling of aircraft when zoomed out #14899
  - "Set breakpoint" can no longer cause a crash when no network is loaded #15003
  - Message window "time link" is now working for vehicle-person-collision warning #14931
  - Fixed crash on right-click in "color edge by inclination" mode #14856
  - Fixed invalid lateral placement of wide busStops and containerStops #15089
  - Fixed invalid rotation of articulated vehicles when parked #15168
  - Improved clickable message log in translated GUI #14759
  - Added warning if breakpoint file does not exist #3424
  - Fixed centering on riding person when vehicle exaggeration is active #15397
  - Fixed inconsistent rendering of walkingarea shape depending on zoom #15424

- meso
  - Fixed invalid segment properties when an edge does not allow passenger cars #15020
  - Fixed invalid laneData when using option **--meso-multi-queue** #15093

- TraCI
  - Fixed ignored calls to `vehicle.changeTarget` and `vehicle.setRoute` after calling `vehicle.moveToXY`. #15123 (regression in 1.13.0)
  - Fixed handling of large time input values #14793
  - Fixed crash when trying to call `traci.vehicle.dispatchTaxi` for an undeparted vehicle #15016
  - Fixed invalid error when constructing a new TraCILogic with libsumo #15164
  - traci.simulation.findIntermodalRoute can now be used to find taxi mode routes #15172
  - traci.vehicle.getNextTLS now works for parking vehicles #15181
  - Fixed crash when calling vehicle.getDrivingDistance2D in meso #15404
  - vehicle.getLeader retrieval now works beyond 3000m if subsequent edges have only 1 lane (i.e. for railways) #15418
  - subscribeContext when used without varIDs now uses the default attributes of the target domain. #15430
  - vehicle.getLanePosition is now working for a parked vehicle #15450
  - Setting stop parameter "expected" now works #15486
  - Setting speedFactor now works for walking stages added with appendStage and appendWalkingStage #15518, #15527

- tools
  - osmGet.py: Fixed error downloading data from servers that do not provide gzipped data. #15105 (regression in 1.20.0)
  - sumolib.net.readNet now works with old versions of lxml and pathlib.Path arguments #15422 (regression in 1.20.0)
  - generateParkingAreaRerouters.py : No longer fails if there are more than parkingAreas in the input file #14880
  - traceExporter.py: fixed inconsistent trj-output #14925
  - matsim_importPlans.py: now properly quotes all parameters to ensure that the output is valid XML #14988
  - mapDetectors.py: No longer writes invalid lane pos #15117
  - createScreenshotSequence.py: Now works with the same key frame settings in a row #15352

### Enhancements

- sumo
  - Improved speed of writing xml output by 60% #14994, #8905
  - Added time stamp to E3 warnings #14870
  - personInfo now writes actual stage depart times instead of desired depart #14886
  - added option **--max-num-persons** to delay person insertion above a given threshold. #14884
  - A warning is now given when starting more persons in a vehicle than it's personCapacity permits #14752
  - Persons and containers now support vType attribute `boardingFactor` which gets multiplied with the boarding- or loadingDuration of the vehicle. #14869
  - Loaded polygons can be used to influence [jupedsim agents](Simulation/Pedestrians.md#model_jupedsim) #14325
  - PHEMlight5 parameters can now be configured with [generic parameters](Simulation/GenericParameters.md) #14285
  - Electric vehicles now support [defining variable charge rate](Models/Electric.md#defining_electric_vehicles) #14860
  - The new option **--emission-output.attributes** can now be used to customize the output #12850
  - Jupedsim pedestrian model now waits at pedestrian crossings #15012
  - EIDM model now has a fixed upper boundary to drivererror-intensity to avoid issues for vehicles with high acceleration. #15066
  - GLOSA device now uses speedFactor to set it's target speed when slowing down to improve interaction with carFollowModels #15067
  - Reduced redundant vClass warnings from taxi device #15090
  - Added extra penalty for turnarounds during routing, configurable with option **--weights.turnaround-penalty** #4966
  - Individual speedFactor is now adjusted when switching vehicle type (with calibrators, devices or TraCI) #15076
  - Added GLOSA options to increase or reduce safety. #15152
  - Added vType-attribute `jmStopSignWait` to customize required waiting time at stop sign #15163
  - Actuated traffic lights now support phase attributes `earlyTarget="DEFAULT"` and `finalTarget="DEFAULT"`. This simplifies combining default switching rules (gap based) with custom conditions #14669
  - chargingStation default power is now 22kW (instead of 0) #15144
  - Added warning for unusual distribution definitions #15146
  - fcd-output now contains the vtype for pedestrians #15210
  - Added option **--chargingstations-output.aggregated** to write output that is more compact #15240
  - vType attribute `jmAllwayStopWait` can now be used to customize required waiting time at allwayStop #15428
  - time-to-teleport.disconnected may now be higher than time-to-teleport #15494
  - Stop attribute `index` can now be used to encode stopping on a later edge of a looped route #15503
  - The battery model now records depletion events in tripinfo output #15529

- netedit
  - Junctions and edges now have the virtual attribute `isRoundabout`. This makes it easy to select and find all roundabouts in a network #14865
  - Mode for editing overhead wires now warns about experimental state #14974
  - Using check boxes in submenu now keeps the menu open #15009
  - Loading a single file without options now supports .sumocfg files #15015
  - Add new vClass icons #14646
  - Convert-to-roundabout function now sets roundabout edge properties based on incoming edges #15069
  - Hotkey <kbd>ESC</kbd> + <kbd>SHIFT</kbd> can be used to clear the selection regardless of editing mode #14481
  - Making person/container plan wider on mouse-over #15337
  - Now Showing junctions while creating person and container plans #15345
  - Added detectPersons to InductionLoop attributes #15515

- sumo-gui
  - Traffic light dialog for tracking phases can now scroll to see all links #3862
  - Different color schemes are now supported when calibrating colors to the data (recalibrate rainbow) #12483
  - Improved rendering speed on right-click #15035
  - Reduce width of lane parameter dialog (by adding linebreaks) #15051
  - POI attribute `width` can now be used to configure POI size (in m) #15444

- meso
  - vType attribute `jmTimeGapMinor` is now considered when running with option **--meso-junction-control** #15171
  - depart and arrival lanes are now part of tripinfo output #7680

- netconvert
  - OSM import now supports distances and speeds with units (i.e. feet, knots) #14885
  - OSM import now warns if ways, nodes or relations are out of order #14892
  - OSM import now handles trolleybus routes #14932
  - Shapefile import now respects option **--default.spreadtype** #14984
  - Added option **--geometry.max-angle.fix** for removing sharp angles within edge #15221
  - Added option **--default.connection.cont-pos** to change the default internal junction position (or disable internal junctions) #13642

- marouter
  - Loaded trip attributes (i.e. departLane and departSpeed) are now preserved #15049

- duarouter
  - Added extra penalty for turnarounds during routing, configurable with option **--weights.turnaround-penalty** #4966

- traci
  - The new sumo option **--keep-after-arrival TIME** keeps vehicle objects in memory for a configurable time after their arrival. This makes it possible to retrieve properties of their trip. #9891
  - Fixed missing leader/follower information in lanechange output after forced change. #14912
  - chargingStation attributes can now be modified #12772
  - Added functions `edge.getBidiEdge` and `lane.getBidiLane` #15186
  - Added functions `getMass` and `setMass` to the vehicle, person and vehicletype domains #15258
  - Added person stages now support all stopping places and not only busStops #15281
  - Meso simulation now supports functions `vehicle.getSegmentIndex` and `vehicle.getSegmentID` #14681
  - Meso simulation with option **--meso-lane-queue** now supports `vehicle.getLaneIndex` #15341

- tools
  - plotXMLAttributes.py: can now use special attribute `@FILE` to simplify plotting of multiple files where each file provides one value #14843
  - plotXMLAttributes.py: added option **--robust-parser** which handles files without linebreak after each element #15340
  - gtfs2pt.py: Now warns if no valid GTFS inside the network boundaries has been found #14876
  - addStops2Routes.py: Can now create waypoints by setting option **--speed**. #14895
  - traceExporter.py now supports options **-c** and **-C** for loading and saving configuration files #14934
  - generateRailSignalConstraints.py: Added missing constraint for parking vehicles with 'ended' value. #14609
  - generateRailSignalConstraints.py: Added option **--abort-unordered.keep-actual** which keeps stops after a detected overtaking as valid if they have started/ended values #15065
  - generateRailSignalConstraints.py: Added option **--all-inactive** for setting all constraints as inactive #15312
  - createVehTypeDistribution.py: now automatically writes `speedDev` when only `speedFactor` is defined by the user to avoid a wider distribution than may be expected #15025
  - mapDetectors.py: Option **--max-radius** can now be used to configure maximum mapping radius #15118
  - mapDetectors.py: Can now handle CSV with BOM #15116
  - net2geojson.py: can now optionally include numLanes and speed as properties #15109
  - distributeChargingStations.py: this is a new tool for adding chargingStations to a network with parkingAreas #14667

### Miscellaneous

- configuration file root elements are now application-specific #7700
- Unknown vehicle classes are now ignored by sumo to ensure upwards compatibility when adding classes in the future. Netconvert and duarouter support this behavior by setting option **--ignore-errors** #14993
- Saving config files now handles environment variables as absolute path #15022
- `sumolib.checkBinary` now finds sumo binaries installed via pip #14982
- SUMO game language switch is now working in the downloadable zip #13440
- Fixed broken images in game-cross scenario #15083
- DEFAULT_CONTAINERTYPE now uses vClass `container` #15092
- parkingArea default roadsideCapacity is 1 (instead of 0) if no space definitions are given #15264
- plotting tools can now handle Matplotlib versions suffix like ".post1" #15372
- Fixed invalid characters in documentation command examples #15441
- Fixed inconsistent argument name in libsumo function Vehicle.moveTo #15304
- A warning is now given when trying to use carFollowModel EIDM with actionSteps #15557

## Version 1.20.0 (07.05.2024)

### Bugfixes

- sumo
  - Fixed inconsistent edgeData and laneData emissions. #14209
  - Fixed crash with SSM device and incomplete routes. #14201
  - Fixed invalid handling of jumps after stopping twice in the same spot. #14324
  - Teleported vehicles are no longer moved onto restricted lanes. Issue #14168
  - Speeds imposed by variable speed signs (VSS) or TraCI now take precedence of vClass-restricted speeds (this was said to be fixed in 1.19 but didn't work) #13652
  - Fixed crash when using **--lateral-resolution** with **--no-internal-links** #14460
  - chargingstations-output no longer records charging when battery is full #14473
  - Fixed undefined behavior during opposite direction driving #14475
  - Fixed crash when loading NEMA controller with pedestrian crossings. #14474
  - NEMA controller now warns about missing green phase #14502
  - Fixed bug where person enters the wrong vehicle on looped public transport line #14526
  - Routes with jumps now support attribute `repeat` #14549
  - Fixed crash when a person plan contains walks or personTrips where origin equals destination #14558
  - Fixed invalid insertion after jump #14578
  - Fixed emergency braking in roundabout. #14587
  - traffic light detectors no longer assume having seen a vehicle on step before the simulation started #14590
  - Removed unneeded warning when a person uses a vehicular connection #14619
  - Fixed invalid departSpeed for IDM in subsecond simulation #14621
  - Fixed invalid traffic demand when defining poisson flows with rate below 0.001 #14664
  - Fixed crash when loading person stages without a person (now writes an error) #14654
  - Fixed xsd validation error for router interval with human-readable time #14728
  - HBEFA4 electric vehicles can now recuperate #14764
  - A warning is now given when loading a disconnected walk #14779
  - When simulating a disconnected walk, the walking direction is now correct after passing the disconnected part #14780
  - Fixed invalid pedestrian jam warning #14796
  - Taxi
    - Fixed problem with **--device.taxi.idle-algorithm taxistand** when the destination is unreachable. #14115
    - Taxi reservations from the same busStop are now grouped if the dispatcher permits it #14612
    - Fixed bug where simulation hangs #14821
    - Fixed bug where taxi exits the simulation prematurely #14819
    - Fixed invalid taxi pickup time estimation from busStop #14822
  - Railways
    - Fixed trains getting stuck on reversal due to routing failure. #14332 (also affects duarouter)
    - Inserting vehicle with depart="split" now works on short edges. #14359
    - fixed collision during portion working when train route continues after join. #14350
    - Rail signals now permit train joins where the front is joined to the rear part. #14349
    - Fixed invalid rail signal warnings for looped route with jumps #14554

- netedit
  - Fixed invalid warning when creating poiGeo #14425 (regression in 1.9.0)
  - Lane markings are rendered below the junction bubble again #14417 (regression in 1.12.0)
  - The unsupported attribute "lines" is no longer written for a personTrip #14463 (regression in 1.15.0)
  - Fixed crash on undo #14702 (regression in 1.15.0)
  - Fixed invalid "save" dialog after loading additionals from file. #14464 (regression in 1.16.0)
  - The size and position of the settings dialog are now stored across sessions. #14571 (regression in 1.16.0)
  - Fixed missing connections after adding edge #14391 (regression in 1.19.0)
  - Alt+Hotkeys for menus are working again #14396 (regression in 1.19.0)
  - Fixed superfluous scrollbars in combo boxes #14412 (regression in 1.19.0)
  - Fixed numerical errors in poisson flow rate #14648 (regression in 1.19.0)
  - Fixed crash when loading vTypeDistribution #14733 (regression in 1.19.0)
  - Fixed invalid default lane permissions when writing a `<laneClosingReroute>` #14348
  - Tool plot_trajectories.py is now usable. #14147
  - "copy type" now also copies vehicle class #14444
  - Fixed crash when trying to define ride between busStops #14462
  - Fixed use of python tools involving space in paths #14469
  - Fixed saving of python tool config involving space in paths #14506
  - Fixed invalid state of save-sumoconfig button after changing option #14581
  - Saving traffic light programs to a file now writes all attributes #14674
  - Custom connection permissions can now be reset #14723
  - Fixed bad signal plan after applying 'join junctions' to part of a joined traffic light #14797
  - Fixed person mode bug where it was impossible to define a new person on a short that already had a person on it #14801


- netcovert
  - Fixed crash when importing incomplete OSM data #14766 (regression in 1.15.0)
  - Signal state sequences (green-yellow-green) is no longer generated. #14295
  - Roundabouts defined explicitly in OSM now have correct right-of-way regardless of geometry. #13970
  - Fixed problems related to option **--dlr-navteq-output**. #14071
  - Option **--junctions.minimal-shape** now persists when re-processing the network. #14375
  - Fixed invalid permissions in OSM import of ferry routes. #14362
  - Fixed crash when importing Vissim Network with unusual geometry. #14413
  - Prevented writing an invalid network when aborting with an error #14470
  - NEMA junctions now stays consistent on recompute #14491
  - NEMA junctions now work correctly with option --tls.group-signals #14440
  - generated NEMA controller now also gives green to pedestrians #14501
  - Fixed crash when processing NEMA junctions with pedestrian crossings #14555
  - Fixed broken junction shapes in 3D network. #10645
  - Edge parameters are no longer lost when using option **--geometry.remove**. #14517
  - Fixed invalid route in ptline-output. #14534
  - bike lane default width is now applied to both directions in OSM import #14560
  - Fixed missing bus connection in OSM import. #14507
  - Fixed bug where attribute `allowed` and `disallowed` were not minimal #14632
  - Large circular network structures are no longer misclassified as roundabout. The size threshold can be configured with option **--roundabouts.guess.max-length** #14634
  - Motorway ramp building no longer fails when the default IDs for new junctions and edges are already in the network. #14658
  - Fixed invalid walking area shape #14688, #14731
  - Fixed the root causes for "Unequal lengths of bidi lane" warnings #14699
  - OSM: fixed bug when importing cycle lane #14419
  - Preventing oversized junction join #14769
  - Fixed bad node shape #14777
  - Increased available width on large walkingareas (avoids jamming on shared-space) #14800

- sumo-gui
  - Fixed wrong context menu when clicking on lane in mesosim #14457 (regression in 1.15.0)
  - Fixed positioning of guiShape "scooter". #13691
  - Fixed misleading visualization of single-car vehicle length in draw-rail-carriages mode. #14330
  - Fixed invalid default for edges minSize when loading incomplete gui settings file. #14384
  - Persons are no longer drawn outside the vehicle when drawn as triangle #14433
  - option **--write-license** from configfile is now respected #14494
  - Fixed visible grid in satellite background image tiles. #14573
  - Fixed bug where the legend name was unreadable on black background #14651
  - Fixed chargingStation power text label position/rotation #14740

- meso
  - Fixed bug where taxi fails to pick up person on the current segment. #14500
  - traci functions edge.setMaxSpeed and lane.setMaxSpeed now work correctly when increasing speed #14552,  #14566
  - Fixed invalid error when personTrips require walking before taxi use #14575
  - Fixed invalid error during intermodal routing #14575
  - Fixed bug where vehicles skip ahead after lane speed update #14593

- duarouter
  - Fixed xsd validation error when loading walk or stop with geo-coordinates #14426
  - Fixed invalid intermodal route on looped public transport (also affects routing in sumo) #14343

- od2trips
  - Fixed extremely high running time when loading large TAZ. #14784

- TraCI
  - Fixed error in `trafficlight.setProgramLogic` when loading actuated logic with default minDur and maxDur #14741 (regression in 1.12.0)
  - Fixed missing internal lane length in traci.vehicle.getNextTLS. #14246
  - `vehicle.setStopParameter` now supports "jump" #14441
  - `vehicle.setSpeed` no longer causes stop at wrong position #14459
  - Fixed result of `vehicle.getLeader` for junction leaders #14617
  - Fixed invalid behavior after removing a stop with `vehicle.replaceStop` #14696
  - Fixed invalid traci constant for traffic light type #14736
  - Stops are no longer lost when calling `vehicle.changeTarget` #14789
  - Fixed crash when calling `simulation.loadState` with active vehicle highlights #8511
  - Fixed computation of driving distance along a route #14760
  - Fix for wrong lane occupancy calculation when minGap of vehicles is changed #13024

- Tools
  - osmWebWizard no longer aborts with error if a configured mode has no infrastructure. #14361
  - xml output from edgeDataDiff can now be loaded in netedit and sumo-gui. #14387
  - tileGet.py is able to use mapQuest service again. #14202
  - checkStopOrder.py: Fixed faulty warnings when generating table for multiple locations #14562
  - osmGet.py: Fixed missing building shapes (also affects osmWebWizard.py) #14598
  - Fixed problem locating binary applications on windows. #14676

- Activitygen: Fixed wrong working hour fallback times. #14344

### Enhancements

- sumo
  - Added new vClasses: subway, scooter, aircraft, cable_car, wheelchair, drone, container #12335
  - Access elements support `pos="doors"` to change the algorithm for placing passengers that exit the vehicle. #14336
  - chargingStation now supports attribute "parkingArea". When set, vehicles will only charge after reaching that parkingArea. #13596
  - Persons and containers that continue in a train after [split/join](Simulation/Railways.md#portion_working) no longer incur boarding or loading delay. #14360
  - Added new attribute `departPos="splitFront"` which causes a train with [`depart="split"`](Simulation/Railways.md#portion_working) to be created at the front rather than the rear of the original train. #14358
  - vType attribute `mass` is now used within `carFollowModel="Rail"`. #13055
  - Option **--fcd-output.attributes** now supports the value 'arrivalDelay' #14447
  - Sumo now allows specifying departure and arrival positions in network or geo-coordinates #2182
  - carFollowModel "Rail" now permits loading custom model curves for traction and resistance #14258
  - speedFactorPremature can now make use of stop parameter "flexArrival" if a reference time other than the scheduled arrival is needed. #14503
  - Using jumps together with 'via' is now supported. #14585
  - Option **--weights.tls-penalty** now also applies to tls-controlled pedestrian crossings. #14653
  - Option **--vehroute-output.cost** now applies to routed persons #14655
  - The [battery device](Models/Electric.md) now supports saving and loading state. #14624
  - Stages generated from `<personTrip>` now inherit params from the personTrip. #14513
  - Actuated traffic lights are now more flexible when controlling minor links (reduced warnings of type "... has no controlling detector" warnings) #14692
  - It is now possible to set the vehicle routing mode via `<param key="device.rerouting.mode" value="..."/>` either in the vType or the vehicle/trip/flow. Setting it to value of `8` lets a vehicle ignore rerouter-induced permission changes on insertion and rerouting. The default routing mode for all vehicles can be set with option **--device.rerouting.mode** (even for vehicles that do not have such a device). #13494, #14815
  - A personFlow with `depart="triggered"` can now be distributed over a flow of vehicles #14806
  - vTypes and parkingAreas now support attribute 'parkingBadges' to control access #14482
  - personInfo (in tripinfo-output) now include data that aggregates over all stages of the plan #14823
  - tranship to stopping place other than containerStop can now be defined #15399

- netedit
  - Now sidewalk and bikelane width can be edited in *create edge mode*. #9725
  - Added new netedit option --ignore-missing-inputs. #12395
  - Add color legend in person and container modes. #13139
  - Now netedit checks if repeated routes are valid before saving. #13503
  - Disable stopOffsetException row if stopOffset is 0. #14065
  - Improved rendering speed for large networks. #13894
  - Added red contour to moved elements. #14109
  - The maximum number of rows for a combo box can now be configured in the gui settings (persistent across sessions) #14412
  - Added option for selective hiding of short edges when zoomed out (enabled by default to increase frame rate) #14487
  - POIs and polygons now support "show-cursor position / geo-position" in the context menu #14812

- sumo-gui
  - Hotkey B now sets a breakpoint at the current time. Alt+B ahead of the current time. #10400
  - Train visualization param `locomotiveLength` now supports value *0*, to prevent rendering of a locomotive. #14351
  - The new train visualization param `carriageImages` accepts a comma-separated list of image files to enable distinct images for the different carriages. #14403
  - Reduced exaggerated size of long vehicles (i.e. trains) when zoomed out. Issue #14423
  - Reduced the number of bike markings being rendered to increase realism and reduce clutter. #14545
  - Airways now have their own default color #14813
  - Added more information to stop info in vehicle parameter dialog #15395

- netconvert
  - Edge widths are now imported from OSM. The new option **--ignore-widths** can be used to restore legacy behavior. #4392
  - Added option **-junctions.endpoint-shape** to compute the junction shape based on custom edge endpoints instead of the usual geometry heuristics. #14341
  - Option **--output.street-names** is now enabled by default if **--shapefile.name** is set. #14399
  - OSM import now support importing restricted turn lane information (i.e. turn:bus:lanes) #14476
  - More rail signals are imported from OSM and option **--osm.railsignals** can be used for fine grained control of signal interpretation. #14483
  - Rail signal direction is now imported from OSM #14512
  - Option **--keep-edges.postload** now applies to permission and speed based edge filters. This is useful when these values are updated with patches or public transport processing. #14528
  - Loaded ptLine period can now be patched by loading the same id in **ptline-files** #14586
  - OSM import now exports subways as vClass "subway" and aerialway as "cable_car" #14644
  - Now importing access="no" from OSM #14650
  - Improved now shape computation to allow more slanted stop lines #14778
  - OSM import now distinguishes barrier crossings from no-barrier rail crossings #14786

- polyconvert
  - Option **--osm.merge-relations** now handles "holes" #14599
  - Improved import of waterbodies and harbours #14597

- duarouter
  - Added support for loading ride with geo-coordinates #14427
  - When writing person stops, the stopping place name is now written as a comment. #14521
  - Added option **--weights.tls-penalty** to improve reliability of travel time. #14653

- TraCI / libsumo
  - person-stage attributes `travelTime` now reflects the spent time for the current stage. #11838
  - Function `vehicle.replaceStop(..., teleport=1)` is now usable without enabling teleports (by using a "jump" to move the vehicle) #14438, #14468
  - inductionloop.getIntervalOccupancy, getIntervalMeanSpeed and getIntervalVehicleNumber are now retrievable in meso #7492
  - [TocDevice](ToC_Device.md) openGap parameters can now be set via `vehicle.setParameter` #14582
  - in mesosim, `lane.setMaxSpeed` now only affects a single queue when running with **--meso-lane-queue** #14589
  - Added functions `edge.getFromJunction`, `edge.getToJunction`, `junction.getIncomingEdges`, `junction.getOutgoingEdges` to query the network topology. #14490
  - The routing mode value `ROUTING_MODE_INGNORE_TRANSIENT_PERMISSIONS` can now be used to ignore temporary permission changes from rerouters. #14205
  - Added function `trafficlight.getSpentDuration` #14400
  - `trafficlight.getParameter(tlsID, "typeName")` now retrieves traffic light type #14737

- Tools
  - added [createScreenshotSequence.py](Tools/Misc.md#createscreenshotsequencepy) to help with creating videos from a simulation with scripted view movements. #14060
  - tileGet.py supports downloading rendered tiles from OSM. #14241
  - added `sumolib.net.node.getMaxTLLinkIndex` #14373
  - gtfs2pt.py nows support human-readable times output using option **-H**. #11192
  - [matsim_importPlans.py](Tools/Import/MATSim.md) now supports loading input with coordinates #14422
  - plot_trajectories.py now supports meso fcd when setting option **--meso**. #14592
  - sumolib.net now supports `getVersion` to retrieve the network version #14636
  - osmWebWizard now takes into account traffic lights for intermodal routing to avoid persons missing their ride #14653
  - osmWebWizard now runs with option **--tls.actuated.jam-threshold 30** to improve the capabilities (and efficiency) of traffic lights #14580
  - Added tools/visualization/plotWKT.py for plotting WKT files
  - ptlines2flows.py supports option **--join** to concatenate lines which serve opposite directions. This reduces conflict on single-track rail lines #5320
  - sumolib.shapes.polygon now supports more attributes. #5429
  - Added tool net/buildFullGraph.py to help with building point-to-point aerial mobility #14811
  - Added tool net/patchVClasses.py for downgrading new networks to be used with older SUMO versions and for manipulating permissions in general. #14798


### Miscellaneous

- [Numerical access restrictions](Simulation/VehiclePermissions.md#custom_access_restrictions) for routing are now documented.  #14370
- Fixed inconsistent documentation for jumps #14316
- The "build" directory has been renamed to "build_config" to allow "build" to be used for build outputs.
- [Plexe](https://plexe.car2x.org/) now supports platoon-aware lane change logic #14809, #14395
- Added 6 new configuration files for the [MMPEVEM](Models/MMPEVEM.md) model. #14499
- The battery device now uses param "rotatingMass" instead of "internalMomentOfInertia". The old value has been deprecated. The default value was changed to improve realism. #12513, #13422
- The network now contains outline shapes for pedestrian crossings #11668
- stop-output has slightly reordered attributes to easier see the busStop-id without horizontal scrolling #14542
- extractTest.py now works for netdiff.py tests #14725
- duarouter: adapted Gawron default values to make them consistent with duaIterate.py. #14772


## Version 1.19.0 (07.11.2023)

### Bugfixes

- Simulation
  - Fixed major slowdown on windows due to translation. #13561 (regression in 1.16.0)
  - Fixed failure to join a leading train on subsequent edge. #13539
  - PersonFlows now draw different vTypes from a distribution #13537
  - Fixed invalid route after trying to to join leading train on subsequent edge of diverging route. #13540
  - Edgedata-output now only counts vehicles that start their teleport on an edge in the teleport count of that edge. #13559
  - Fixed collision during sublane-change. #13582, #13585
  - Fixed crash during emergency vehicle simulation. #13598
  - Start and end edge in trips can also be used as vias to create looped routes. #13987
  - bidi edge
    - Fixed avoidable slowdown when approaching a bidi lane at low speed. #13558
    - Fixed deadlock on junction when approaching occupied bidiEdge at traffic light. #13541
    - Fixed lane choice when approaching an edge with a shared median lane. #13402
    - Fixed unsafe insertion on bidiLane. #13566
    - Curved bidi lane now identified on loading. #13571
    - Fixed invalid bidi-leader identification during lane changing. #13572, #13576
    - Fixed invalid collision warning on bidi lane. #13573
    - Fixed failure to change lane on internal bidi lane. #13575
  - Speeds imposed by variable speed signs (VSS) or TraCI limit the edge type speeds (vehicles will respect the edge type speed if lower than VSS one). #13665, #13652
  - Corrected speed calculation for induction loop with length > 0. #13669
  - another fix for leader distance in ACC model. #13681
  - proper normalization when using distributions of vType distributions. #13786
  - fixed rerouting problems with numerical instabilities at the current stop. #13835
  - SSM output
    - fixed wrongly classified collisions. #13918
    - fixed problems with the device after state loading. #13924

- netedit
  - Fixed problem saving person flow attributes. #11022
  - Fixed problem saving containers sorted by begin attribute in netedit. #11022
  - Fixed problem saving edgetypes. #12467
  - Added proper error handling for duplicate edgeRelations. #11329
  - Can now load and modify `<vTypeDistribution>` elements. #12846
  - Now if a network element ID is modified, flag for save children is enabled. #13619
  - Avoid mixing edges and lanes in overlapping module when inspecting edges or lanes. #13093
  - Fixed crash opening undo-redo dialog after setting a simple attribute. #13673
  - Fixed invalid checks setting IDs. #13690
  - Now vType is a combobox when a selection of vehicles is inspected. #12719
  - Fixed error with FXIconComboBox size. #12623
  - Disabled invalid combinations for node 'type' and 'rightOfWay'. #13436
  - Fixed problem moving vehicles. #13824
  - Fixed crash in netedit due to netbuild / connections. #13731
  - "Press F5" message now updates earlier. #13912
  - Fixed vehicle class selection ignoring user input. #13943

- sumo-gui
  - 3D models can be loaded as decals again. #14009 (regression in 1.17)
  - Context menu uses same screen as sumo-gui on Windows. #13674
  - Sidewalks are no longer connected to the ground in 3D view. #13993
  - The time detection for breakpoints now works with different languages and is not triggered by arbitrary numbers. #14003
  - Opening a sumo config in netedit from sumo-gui now works with special characters in paths on Windows. #14005, #13768

- netconvert
  - Fixed unequal length of bidi edge. #13569
  - OSM turn lanes for left hand networks are now in right order. #13549
  - Fixed crash joining junctions. #13581
  - Fixed superfluous connections after joining junction. #13553
  - Fixed problems with UTF8-BOM encoded csv files (e.g. VISSIM). #13932

- Tools
  - routeSampler.py
    - Fixed bias when distributing flow departures over the data interval. #13523
    - Avoid throwing away all routes with weight less than 0.5 in full optimization. #13830
  - plotXMLAttributes.py
    - now works if one of multiple data files contains no data. #13524
    - now uses file name for legend if no id attribute is set. #13534
    - fixed crash when parsing heterogeneous content. #13556
  - traceExporter.py: kepler-JSON now generates correct timestamps. #13522, #13550
  - duaIterate.py: fixed corner case with empty output file after first iteration. #13840
  - mapDetectors.py: now handles multiple lanes with the same distance. #13845
  - fixed index in sumolib.net.node.Node.forbids. #13857
  - osmWebWizard.py: Fixed error when requesting an amount of 0 vehicles for a mode. #13947, #13787
  - convert_detectors2SUMO.py: Syntax error fixed which made the script unusable. #13650
  - gtfs2pt.py: Now writes correct route and vehicle ids with recent pandas versions. #13346

- TraCI / Libsumo
  - traci.vehicletype.setScale is now applied to flows. #13707
  - Vehicles now do opposite lane overtaking for vehicles stopped via TraCI. #13710
  - Fixed error message about unknown edge. #13890
  - Lanearea detector now reports for last interval if output file is not set. #13966
  - Fixed memory leak in libsumo exception code. #13821
  - TRACI_VERSION is now 21 to account for the unit changes in #7277

### Enhancements

- Simulation
  - The access position for a stop can now be "random". #13492
  - Can now use distributions for parameter values like "device.battery.capacity". #13759
  - Renamed some battery parameters and made them available from type and vehicle. #13838, #12297, #13645
    - maximumBatteryCapacity -> device.battery.capacity
    - actualBatteryCapacity -> device.battery.chargeLevel
    - maximumPower -> device.battery.maxPower
    - stoppingTreshold -> device.battery.stoppingThreshold
  - Write a warning if the user uses an internal route (ID starting with a "!") in a different vehicle. #13810

- sumo-gui
  - Added tooltip in scale traffic label. #13545
  - Now current sumoconfig can be loaded directly in netedit (edit / open sumo config in netedit). #13588
  - Charging state of a vehicle can be shown. #13595
  - Only showing the coordinates once if there is no projection defined #13781

- netedit
  - Now supports Human Readable Time (HH:MM:SS). #11022
  - Added support for E3 attribute expectArrival. #13477
  - Now type parameters can be edited in main type frame. #12378
  - Now vehicles and routes can be reversed. #12967
  - Improved locator dialog in netedit. #12654
  - Now trips over TAZs or Junctions can be transformed to flows (and vice versa). #13608
  - Added option "ignore-supermode-question" for disabling question dialog during undo-redo. #13620
  - Add checkbox in Move Frame for forcing draw edge start/end geometry points. #13426
  - Add tooltip in GroupBox expand button. #12236
  - Add locate containers dialog. #13692
  - Added support for vType distributions. #12845
  - Added support for route distributions. #12660
  - Implemented frame for edit vType distributions. #13393
  - Implemented frame for edit route distributions. #13635
  - Added buttons next to lane attribute to shift element between edge's lanes. #13387
  - Added search field in comboBox. #12394, #13735
  - Added drawing reference for creating additionals. #13769
  - Now internal lanes can be drawn. #13790
  - Added JuPedSim elements WalkableArea and Obstacle. #13667, #13687, #13837
  - Add new view "JuPedSim". #13750
  - Support persons over TAZs in netedit. #13784
  - Now in edge frame small edges can be created. #13833
  - Added "icon" attribute for pois. #13718
  - Person plan creation now automatically detects the next approached network element. #13828
  - Improved drawing of walking areas. #13934, #13893

- netconvert
  - Now issues a warning if unknown tls id are passed to option **--tls.join-exclude**. #13466
  - Edge type errors only trigger a warning in plain XML input if no type file is loaded. #13624
  - handling OpenDrive's include tag #13686
  - Exclude signals (traffic light) which are placed on lanes not allowed to drive on in OpenDrive 1.4. #13836

- Tools
  - several improvements to generateParkingAreas.py #13563
  - generateRailSignalConstraints.py: Now handles `depart="split"` for insertionOrderConstraints. #13565
  - Game from downloaded zip no longer requires SUMO_HOME. #13583
  - sumolib can now load selection files. #13589
  - meandata2vss.py: can now create variable speed signs from edge mean data. #13804

- TraCI
  - Rail signal constraints can now bew added via traci. #13853

- Other
  - od2trips now supports pedestrian types #13606
  - Now JuPedSim can be enabled in CMake GUI with a checkbox. #13646
  - Integration of JuPedSim in sumo. #13307, #13491, #13308
  - You can now add current time and PID to all log entries. #13757
  - The old C++ TraCIAPI.cpp and Java's TraaS give now a warning about being deprecated #14026. Please use [libtraci](Libtraci.md).
  - The new SUMO homepage is now https://eclipse.dev/sumo.
  - The repository moved to https://github.com/eclipse-sumo/sumo and the website repo to https://github.com/eclipse-sumo/sumo.website.
  - Added wheels for Python 3.12 and improved on wheel testing in CI

## Version 1.18.0 (29.06.2023)

### Bugfixes

- Simulation
  - Setting param `vTypes` for actuated and delay_based tls detectors is now working again. Issue #13448 (regression in 1.15.0)
  - Fixed invalid emission and consumption outputs for the HBEFA4 emission classes (model was calibrated on km/h instead of m/s). Issue #13250
  - Collisions on shared walkingareas without vehicular road intersection are now detected. Issue #13132
  - A warning is now given if save-state.times are not reached due to a mismatch with step-length and begin time. Issue #13162
  - Fixed crash when loading rail simulation state with step-length > 1. Issue #13161
  - Fixed crash when loading state with vehicles that have triggered departure. Issue #13096
  - Simulation outputs now include non-zero z-data even if the slope is 0. Issue #13171
  - Fixed crash / invalid output if a person has a `<stop>` after accessing a busStop via an access element. Issue #13108
  - Fixed invalid 'started' and 'arrivalDelay' after passing a short waypoint edge at high speed. Issue #13179
  - Fixed crash after rerouting and losing a stop. Issue #13190
  - Fixed invalid TTC computation when both vehicles are extrapolated to stop. Issue #13212
  - A parkingReroute now works even if the parkingArea on the current edge is not included in the alternatives list. Issue #13288
  - Fixed unnecessary emergency braking when cars urgently need to swap lanes. Issue #13295
  - IDM no longer violates right of way due to imprecise stopping at minor link. Issue #13369
  - Fixed error when loading personTrip between identical stops. Issue #13385
  - Fixed invalid error message for disconnected personTrip between stops. Issue #13386
  - Fixed collision on junction due to unsafe lane changing. Issue #13363
  - Fixed error when using option **--replay-rerouting**. Issue #13389
  - Fixed invalid saved state when input contains route distribution within a vehicle. Issue #13449
  - Fixed emergency braking / collision at roundabout. Issue #13292
  - Fixed emergency braking on traffic light junction #13516
  - The simulation now aborts when encountering an invalid stop definition (unless option **--ignore-errors** is set). Issue #12670
  - bidirectional lane use:
    - Fixed invalid collision warning when using bidi lane. Issue #13312
    - Fixed unsafe right-of-way rules at junction with shared median lane. Issue #13316
    - Fixed frontal collision on shared median lane. Issue #13313
    - Fixed deadlock on bidi edge. Issue #13396
    - Fixed blockage on intersection with oncoming leader. Issue #13397
    - Fixed collision and emergency braking. Issue #11642

- netedit
  - Fixed crash when importing OSM data directly. Issue #13297 (regression in 1.16.0)
  - Fixed invalid undo operation after moving a set of network elements. Issue #13421 (regression in 1.16.0)
  - Fixed segfault when closing netedit and no net is loaded #13131 (regression in 1.17.0)
  - Fixed spacing for person mode path info subframe. Issue #13140 (regression in 1.17.0)
  - Loading Data Elements via menu item is working again. Issue #13224 (regression in 1.17.0)
  - Saving plain xml now works when selecting an existing plain-xml file to define the output prefix. Issue #13200 (regression in 1.16.0)
  - netdiff-dialog now permits to change the output prefix. Issue #13130
  - Fixed invalid default values in netgenerate dialog and tool dialogs. Issue #13152
  - Boolean options can now be reset in tool dialogs. Issue #13156
  - Function 'add reverse edge' applied on a selection, no longer adds duplicate reverse edges. Issue #13209
  - Fixed inconsistent ordering of popup functions for additionals. Issue #13261
  - Stops at a `trainStop` can now be defined. Issue #13258
  - Loading a second .sumocfg now properly resets all options. Issue #13123
  - Object ids that contain the characters `!`, `?` or `*` (which are permitted in sumo) can now be loaded (such ids are discouraged since they make it more difficult to list object ids on the command line). Issue #13351
  - A person with a single stop stage is now visible after loading. Issue #13126
  - Creation of busStops and trainStops with the same id is now prevented (since this would give an error when loading the simulation). Issue #13269
  - Person rides between rail stops can now be defined. Issue #13273
  - Fixed crash saving network with stop container plans #13390
  - In the vType editor, the vClass attribute is now updated when switching current type. Issue #13317
  - The "stacked" person indicator is now working for persons that start at a busStop. Issue #12380
  - Z value is no longer applied twice in move mode. Issue #13411
  - Fixed error when saving sumo/netedit configs at path containing spaces. Issue #13373
  - Added missing updates of frames after undo/redo. Issue #13429
  - Python tools now work when the SUMO_HOME path contains spaces. Issue #13458
  - Using netedit to define custom detectors for actuated traffic lights is now working. Issue #13412
  - Resetting the `next` attribute of branching traffic light programs is now working. issue #13409
  - Trigger attributes can no longer be defined for waypoints (the simulation does not allow this). Issue #13499

- sumo-gui
  - The breakpoint-dialog now takes into account the begin time when rounding breakpoints to a reachable step. Issue #13163
  - game mode now switches time display style for all time labels. Issue #13211
  - Fixed crash when switching to a (mismatching) alternative net file. Issue #13215
  - Clicking on a vehicle id which has left the network, no longer moves the view to Position::INVALID. Issue #13251
  - Fixed invalid rendering of spread-bidi lanes for shared median lane network. Issue #13343
  - Fixed bug where vehicle context menu function 'select foes' did not select all relevant junction foes. Issue #13358

- netconvert
  - Fixed inconsistent network file after setting **--default.spreadtype center**. Issue #13127
  - OpenDRIVE export: fixed invalid crosswalk shape. Issue #10432
  - Fixed invalid connection permissions after joining junction. Issue #13217
  - Fixed broken network after setting option **--tls.rebuild**. Issue #13240
  - Fixed classification of turnaround connection on some divided highways. Issue #13238
  - Fixed invalid consistency error for circular public transport line. Issue #13255
  - OSM: fixed incomplete pt-route. Issue #13249
  - OSM: fixed problems with one-way bus and bicycle infrastructure. Issue #12592
  - OSM: fixed crash when using option **--osm.turn-lanes**. Issue #13413
  - User defined node radius is no longer ignored for geometry-like nodes. Issue #13064
  - Fixed incomplete ptline-output for circular route. Issue #13431
  - Fixed minor links in rail taxi network. Issue #13456
  - Fixed unsafe signal plan at joined tls. Issue #13502
  - Fixed overly large junction shapes when edges meet at a sharp angle. Issue #13505
  - bidi edges
    - Option **--railway.topology.repair.minimal** now performs minimal repair with respect to public transport lines. Issue #13248
    - Fixed bug where bidi edge could not be declared. Issue #13225
    - Fixed bug where option **--railway.topology.repair** created superfluous bidi edges. Issue #13247
    - Fixed missing bidi attribute for shared median lane with disconnected outer lanes. Issue #13335
    - Fixed invalid bidi attribute for internal edge. Issue #13344
    - Fixed invalid right of way rules when shared median lane ends. Issue #13345
    - Fixed unsafe right of way rules at internal junction with bidiLane. Issue #13400
    - Fixed asymmetrical bidi attribute. Issue #13510

- duarouter
  - Fixed railway routing failure if the stop is defined on a short buffer edge (also applies to sumo). Issue #13277
  - Fixed invalid route cost for train reversal on long edge (also applies to sumo). Issue #13360
  - Fixed intermodal routing failure for rail taxi (also applies to sumo). Issue #13424
  - Fixed invalid ride in output when starting personTrip on rail edge. Issue #13506
  - Initial taxi waiting time is now included in cost computation when a personTrip starts with a taxi ride. Issue #13513
  - Fixed invalid route output when a personTrip provides alternative vehicle modes. Issue #13512

- activitygen
  - Fixed crash when there are no work positions within a city. Issue #13315
  - Now handling `nan` values in population input. Issue #13323
  - Fixed invalid work position statistics when stat file is lacking city gates. Issue #13381

- TraCI
  - Fixed crash when calling traci.load and running with sumo-gui. Issue #13150 (regression in 1.16.0)
  - Calling `vehicle.insertStop` now preserves the original route edges beyond the inserted stop. Issue #13092
  - Fixed libtraci crashes when working on closed connection. Issue #13285
  - Fixed missing vehicle type in saved state and vehroute-output for vehicle added via TraCI. Issue #13384
  - Added missing error reporting when setting invalid carFollowModel parameters via `traci.vehicle.setParameter`. Issue #13399

- Tools
  - Fixed mouse control in several game scenarios. Issue #13366 (regression in 1.17.0)
  - Fixed error when calling some tools without argument. Issue #13388 (regression in 1.17.0)
  - abstractRail.py: Failure to optimize one region is now recoverable. Issue #13193
  - gridDistricts.py: Networks with non-normalized offsets now create correct taz shapes. Issue #13264
  - Function `sumolib.net.getBoxXY` now returns correct results for large networks. Issue #13320
  - When loading a tool configuration, unknown options now raise an error. Issue #13091
  - generateRailSignalConstraints.py: fixed invalid bidi constraint for trains with multiple diversionary tracks between stops. Issue #13519


### Enhancements

- Simulation
  - Added options **--intermodal-collision.action** and **--intermodal-collision.stoptime** to configure vehicle behavior after colliding with a pedestrian. Issue #13133
  - In the sublane mode, vehicle elevation is now interpolated when lane changing between lanes that have different z-values. Issue #13170
  - Added `device.taxi.idle-algorithm` value `taxistand` which makes idle taxis return to a pre-configured set of locations. Issue #13334
  - The SSM-device now supports the new surrogate safety measure "MDRAC" which is modified maximum deceleration rate to avoid crash with perception/reaction time. Issue #13350
  - statistic-output now include the number of emergency braking events. Issue #10596
  - Vehicles can now be configured to ignore specific vehicles during car-following by using [generic parameters](Simulation/GenericParameters.md) `carFollowMode.ignoreIDs` and `carFollowModel.ignoreTypes`. Issue #13362
  - Stops now support custom parameters #13365
  - Multi-Entry-Exit (E3) detectors now support attribute `expectArrival` to suppress arrival warnings. Issue #13476
  - Vehicles with `depart="split"` now automatically adapt their departPos to the rear of the [train from which they split](Simulation/Railways.md#portion_working). Issue #13480

- sumo-gui
  - Element `parkingArea` now supports placing stopping places to the left of the lane in right-hand networks by setting attribute `lefthand="true"`. Issue #13303
  - Simulation end time is now written into the message window. Issue #13145
  - Live edgeData can now be observed after the simulation ends. Issue #13144
  - vTypes of typed detectors are now shown in the detector parameter dialog. Issue #13447

- netedit
  - Supermodes can now be activated using the modes menu. Issue #13138
  - After generating a network without warnings, the new network will open directly without manually closing the output dialog. Issue #13149
  - Implemented new dialog for setting netedit and sumo options. This dialog supports option help, file selection dialogs and permits dynamic filtering of listed options. Issue #12007
  - Simplified selection of .sumocfg files when calling runSeeds.py tool. Issue #13118
  - Simplified selection of edge IDs when calling python tools. Issue #12852
  - The traffic light join mode now has 'OK / 'cancel' buttons. Issue #13028
  - Now netedit supports trip and flows over TAZs. Issue #13311
  - Improved error messages when loading additional objects. Issue #13507

- netconvert
  - OpenDRIVE import: now supports road objects from connecting roads. Issue #13196
  - OpenDRIVE export: now includes additional road object attributes. Issue #13205
  - Added option **--geometry.remove.max-junction-size FLOAT** to prevent unsmooth road shape when converting large junctions to a geometry point. Issue #13199
  - OSM import: typemap `osmNetconvertRailUsage.typ.xml` now imports service designation. Issue #13213
  - Unused edge types are now excluded from .net.xml. Issue #13228
  - Added option **--railway.signal.guess.by-stops** to add rail_signal nodes to a network that is lacking them. Issue #5143
  - When using option **--tls.rebuild**, the tltype can be changed by setting option **--tls.default-type**. Issue #13267
  - Added option **--junctions.minimal-shape** to enforce small junctions. Issue #13377
  - Added option **--opendrive.signal-groups** to import controller information from OpenDRIVE. Issue #2365
  - Added option **--tls.join-exclude** for more control over automatic tls joining. Issue #13466

- TraCI
  - Added function `simpla.getPlatoonID`. Issue #13029
  - Added functions `lane.getAngle` and `edge.getAngle`. Issue #10901
  - Added function `vehicle.getJunctionFoes`. Issue #12640
  - Using UTF-8 consistently. Issue #7171
  - getAllContextSubscriptionResults has now entries also for objects without results. Issue #6446
  - Vehicles with SSM device now permit retrieval of the minimum PPET using param `device.ssm.minPPET`. Issue #13293
  - Function `traci.vehicle.setStopParameter` and `getStopParameter` now supports optional argument `customParam=True` to work with user defined stop parameters. Issue #13365
  - vehicle and vehicletype domain now support the functions set/getBoardingDuration and set/getImpatience. When impatience is set on a vehicle it affects the dynamic impatience which gets reset on starting to drive. Calling vehicle.getImpatience now returns the dynamic impatience instead of the base value defined in the vType. Issue #10430, #13403

- tools
  - randomTrips.py: added options **--from-stops** and **--to-stops** to define pedestrians, personTrips and person rides from and to stops (i.e. busStop). Issue #13375
  - randomTrips.py: now issues a warning if option **--binomial** is too low for the given period. Issue #13404
  - plotXMLAttributes.py and plot_trajectories.py now show the plot by default. Issue #13158
  - plotXMLAttributes.py: Added option **--join-files** to treat data points from different files as if coming from the same file. Issue #13154
  - plotXMLAttributes.py: Now supports attribute value `@DENSITY` to create density plots. Issue #13182
  - runSeeds.py: shortened generated folder names. Issue #13167
  - runSeeds.py: added option **--no-folders** to keep all written files in the same directory when running with multiple configuration files or applications (folder name becomes file name prefix). Issue #13157
  - routeSampler: Added option **--taz-files** and support for loading origin destination matrices in `tazRelation` format. Issue #6890
  - abstractRail.py: Now handles regions without stops. Issue #13221
  - gtfs2pt.py: can now disable generation of pedestrian access via option **--skip-access**. Issue #12774
  - gtfs2pt: stop names are now included in route-output. Issue #13274
  - ptlines2flows.py: now support configuration files. Issue #13254
  - generateParkingAreas.py: now support option **--lefthand** to create lefthand parking areas. Issue #13305
  - [scheduleStats.py](Tools/Railways.md#schedulestatspy): Now permits analysis of planned and actual travel time between pairs of stops. Issue #13354
  - tileGet.py now supports option **--parallel-jobs** to speed up data retrieval. Issue #13327
  - checkStopOrder.py: can now generate a combined stop table for multiple stop locations. Issue #13259
  - checkStopOrder.py: marks waypoints in stop table output. Issue #13420
  - checkStopOrder.py: added option **--filter-ids** to filter the input by vehicle id. Issue #13488
  - generateRailSignalConstraints.py: now supports [portion working](Simulation/Railways.md#portion_working) (vehicles with `depart="split"`). Issue #13478
  - generateRailSignalConstraints.py: now finds additional bidi conflicts. Issue #12085
  - improved game internationalization. Issue #13082

### Miscellaneous

- splitRouteFiles.py, netcheck.py, circlePolygon.py, plot_csv_bars.py, plot_net_trafficLights.py and plot_csv_timeline now use the **-c** shortcut for loading a configuration. All options that previously used this continue to work with the respective long option name (e.g. --columns). Issue #8409 and #12874
- plotXMLAttributes.py and plot_trajectories.py no longer create output file 'plot.png' by default. Issue #13166
- netconvert: road objects imported from OpenDRIVE are now placed on a higher layer to make them visible above the road. Issue #13197
- netconvert: option **--osm.all-attributes** now defaults to exporting all attributes because the explicit list of attributes (**--osm.extra-attributes**) now defaults to value **all**. Issue #13218
- netconvert: increased default bike path width. Issue #13425
- Netedit demand mode hotkeys changed: **C** now activates container mode and **L** activates person plan mode. Issue #13141
- All strings sent and received by TraCI are now utf8 encoded and interpreted as utf8. Issue #7171
- `traci.simulationStep` has no return value on Python any longer (similar to the other languages), use `traci.simulationStepLegacy` for the old behavior. Issue #13296
- The default file extension for netedit configuration files was changed from `.neteditconfig` to `.netecfg` for consistency with other config extensions. Issue #13100


## Version 1.17.0 (25.04.2023)

### Bugfixes

- Simulation

  - Fixed crash with option **--device.rerouting.threads**. Issue #12711 (regression 1.16.0)
  - Fixed collision at parallel lanes with different lengths. Issue #12590
  - Fixed crash when using option **--collision.action remove** and vehicles collide after lane changing. Issue #12583
  - Traffic light type `delay_based` no longer generates overlapping detectors (which could cause invalid switching decisions). Issue #12615
  - Waypoints with attribute 'triggered' now result in an error rather than undefined behavior. Issue #12665
  - Fixed collision on junction between two conflicting links with internal junction. Issue #12715
  - Fixed invalid error: "Disconnected walk for person". Issue #12744
  - Fixed invalid duration value in summary-output. Issue #13006
  - NEMA Controller now works with uneven yellow time at barrier. Issue #12989
  - A triggered stop is now always aborted when the vehicle is full. Issue #13015
  - Setting attribute expected or expectedContainer now always marks the stop as triggered (for person or container respectively). Issue #13016
  - Persons during access stage are now counted as walking rather than riding. Issue #13019
  - Fixed bug where the simulation does not abort when the last active transportable is riding inside an inactive (triggered) vehicle. Issue #13017
  - Setting phase minDur=0 is now working for `actuated` and `delay_based` traffic lights. Issue #12952, #13089
  - Bike lane detector placement for 'actuated' traffic lights now uses correct bike length. Issue #13109
  - Lane changing
    - Fixed crash related to lane-changing in tight corners. Issue #12618 (regression in 1.13.0)
    - Attribute `lcCooperative` no longer impacts speed adjustments that a vehicle needs for its personal lane change maneuvers. #9473
    - Fixed invalid interpretation of sublane positioning of junction leaders during lane changing. Issue #12580
    - Fixed unsuitable lateral alignment on bidi edge when preparing for a turning movement. Issue #11436
    - vType attribute `lcTurnAlignmentDistance` now works for indirect left turns. Issue #8500
    - Fixed invalid strategic lane computation when lanes on the same edge differ in length. Issue #13021
    - Fixed invalid collision warning at bike-to-road merge with sublane model. Issue #13101
    - Fixed collisions at reduction in lane width when using the sublane model. Issue #13103
  - Railway simulation:
    - Simulation now terminates even when a stop with `triggered="join"` fails. Issue #12668
    - Stop attribute `extension` now works for `triggered="join"`. Issue #12666
    - Fixed unsafe train insertion that could cause a follower train to collide. Issue #12857
    - Fixed emergency braking when reversing on a red signal. Issue #12868
    - Fixed invalid railway routing result. Issue #12872
    - Fixed deadlock on bidirectional track. Issue #12858
    - Train headings now reflect the heading of the locomotive. Issue #12985

- netedit
  - Fixed bug when showing list of newly created vehicle types in type mode. Issue #12625 (regression in 1.15.0)
  - Fixed bug that prevent creation of crossing at priority junctions with speed above 50k/mh. Issue #12609 (regression in 1.16.0)
  - The name of the current network is shown in application title bar again. Issue #12702 (regression in 1.16.0)
  - Fixed bug that caused junction shape to change on repeated computations. Issue #12584
  - Fixed running phase duration for rail crossing in parameter window. Issue #12642
  - Centering on TAZ now works if the TAZ was loaded without a shape. Issue #12687
  - Avoiding unnecessary scrollbars in combobox. Issue #12717
  - Lane positions defined before network computation are now handled if they become invalid after computation. Issue #12727
  - Additional objects are now always included in zoomed-out rectangle selection. Issue #12733
  - Fixed invalid description in undolist. Issue #12838
  - Fixed target of "right click" where a road overlaps a polygon. Issue #12819
  - Fixed wrong message in the message window when attempting to save files without permission. Issue #12992
  - Fixed crash when trying to save demand/additionals into non-writable dir. Issue #12984
  - Deleting geometry points that are in the same spot as a junction bubble now works. Issue #12964
  - Can now load TAZ with param and no shape. Issue #12678
  - Changing sumocfg options now always enables the "save sumocfg" button. Issue #12880
  - Shift-click on geometry point can now be used to make it the custom end point. Issue #12716
  - When activating sidewalks in the create edge frame, they now use the configured default sidewalk width. Issue #12449
  - Fixed crash when unjoining a joint traffic light. Issue #12610
  - Fixed "Invalid reference counter" error after recomputing with volatile options with data elements. Issue #13058
  - Netedit now gives a warning when changing user defined junction type 'rail_crossing' to priority. Issue #9273
  - Fixed bug where person plan elements after a stop were not loaded correctly. Issue #13125

- sumo-gui
  - Fixed invalid carriageLength for tram. Issue #13011 (regression in 1.11.0)
  - Fixed random (rare) crash when having an open detector attribute window. Issue #12595 (regression in 1.16.0)
  - Fixed crash when loading abstract projection in .net.xml. Issue #12762
  - Fixed freeze when selecting reachable lanes while the selection editor is open. Issue #12766
  - Fixed crash when switching to game mode in rail network. issue #12779
  - Fixed invisible lane markings at dense geometry. Issue #12837
  - 3D View (OSG view) fixes:
    - Flight mode navigation is now working on Linux. Issue #12503
    - Fixed loading of 2D viewports. Issue #12638
    - Fixed invalid interpretation of background image attribute. Issue #12671
    - Fixed usage of non-Ascii characters. Issue #12628
    - Preset visualisation scheme in GUI settings file is now used. Issue #12682
    - Now clearing background objects on loading a different simulation. Issue #12751

- netconvert
  - OSM: fixed importing lane access for `psv`. Issue #12457
  - OSM: fixed handling of one-way roads that allow buses in reverse direction. Issue #12592
  - Fixed crash when loading split beyond edge length. Issue #12695
  - Bidi status is no longer lost after symmetrical split. Issue #12698
  - Fixed invalid right-of-way rules involving indirect left turns. Issue #12676
  - Fixed invalid edge ordering for strongly curved edge. Issue #12735
  - Relaxed right of way rules in the context of dedicated lanes. Issue #12720
  - Abstract projection is now resolved when loading .net.xml. Issue #12761
  - Fixed tram connections when using option **--edges.join-tram-dist**. Issue #12767
  - Fixed crash when merging networks. Issue #12824
  - Fixed connection building in intermodal networks that could cause dead ends for passenger traffic. Issue #12978, #13087, #10080

- netgenerate
  - Fixed option category for options that apply to all network types but were placed in the `Random Network` category. Issue #12930

- meso
  - Option **--time-to-teleport.remove** is now working. Issue #12797

- TraCI
  - Fixed failure to perform unsafe lane change. Issue #13007 (regression in 1.11.0)
  - Fixed value returned by `person.getMaxSpeed`. Issue #12786 (regression in 1.15.0)
  - Fixed crash when calling `close()` directly on a traci connection. Issue #12996 (regression in 1.16.0)
  - Fixed wrong edgeId in error message of `simulation.findIntermodalRoute`. Issue #12591
  - Error when loading a state file now indicates possible version problem. Issue #12593
  - Fixed problem when loading JAVA bindings for libsumo/libtraci via JNI. Caution: Windows users must update their code to load dependent libraries explicitly due to JAVA issues that cannot be fixed on the SUMO side. Issue #12605
  - Fixed incomplete cleanup of SSM output with repeated libsumo runs. Issue #12587
  - Function `vehicle.setStopParameter` now takes effect when setting "duration". Issue #12630
  - libtraci.lane.setAllowed has been fixed
  - libtraci.simulation.getCollisions returns meaningful results Issue #12861
  - Function `vehicle.setStopParameter` now works correctly with key 'triggered'. Issue #12664
  - Error messages concerning subscription filters now use the correct command id
  - Fixed bug where user-triggered lane-change were not executed when using the sublane model. Issue #12810, #12944
  - `trafficlight.swapConstraints` now returns constraint parameters for derived constraints. Issue #12935
  - Fixed crash when using `vehicle.moveToXY`. Issue #13053

- tools
  - Fixed invalid error when calling option **--save-template**. Issue #12589
  - Selected python tools now handle 'stdout' and 'stderr' as magic file names (i.e. *gtfs2pt.py* with more tools to follow). Issue #12588
  - Fixed broken routes for public transport from GTFS caused by invalid permissions. Issue #12276
  - tlsCoordinator.py now handles disconnected routes. Issue #11255
  - tlsCycleAdaptation.py: fixed ZeroDivisionError. Issue #12760
  - runSeeds.py: Fixed failure to run multiple configs with a list of seeds. Issue #13116

### Enhancements

- Simulation
  - Vehroute-output now includes the used vehicles for `<driving>` stage. Issue #12520
  - DriverState now optionally affects free flow speed (using param `freeSpeedErrorCoefficient`). Issue #6331
  - Element `<vTypeDistribution>` now supports attribute `probabilities` together with `vTypes` to re-use the same `<Types>` with different probabilities. Issue #12799
  - Some warnings about inconsistent public transport stop times are now avoided when using option **--use-stop-ended** along with stop attribute `ended`. #12825
  - Stop-output now includes optional attribute `usedEnded` to indicate whether a stop was affected by option **--use-stop-ended**. Issue #12863
  - Public transport vehicles may now slow down by a configurable factor when they are ahead of their schedule. (using new vType attribute `speedFactorPremature`). This is based on optional stop attribute `arrival`. If the new option **--use-stop-started** is set, this is instead based on the optional `started` attribute of the stop. Issue #11899
  - Simulating a Two-way-left-turn-lane is now supported. Issues #12924
  - Added option **--personinfo-output** to separate `<personinfo>` elements from `<tripinfo>` elements. Issue #12929
  - vType attribute `startupDelay` is now applied to scheduled stops when using `carFollowModel="Rail"`. Issue #12943
  - Statistics-output now includes performance metrics (i.e. clockTime). Issue #12946
  - The z value is now included in emission-output if the network has elevation data. Issue #13022
  - The time spent while passing a [waypoint](Definition_of_Vehicles%2C_Vehicle_Types%2C_and_Routes.md#waypoints) is no longer counted as 'stoppedTime' in tripinfo-output. Issue #13037
  - Junction model [params](Definition_of_Vehicles%2C_Vehicle_Types%2C_and_Routes.md#transient_parameters) `ignoreIDs` and `ignoreTypes` can now be used to ignore pedestrians. Issue #13057
  - Vehicle attribute `depart` now supports the new value `begin` referring to the start of the simulation. Issue #12270

- netconvert
  - Added options **--shapefile.width** and **--shapefile.length** to allow importing custom widths and lengths from [shape files](Networks/Import/ArcView.md). Issue #12575
  - Option **--osm.extra-attributes** now also applies to all node attributes. Issue #12677
  - OSM: Exceptions for turning restrictions are now imported. Issue #12645
  - OSM: Supporting bus lane restrictions using `psv`. Issue #6767
  - junction attribute 'radius' now overrides option **--junctions.small-radius**. Issue #12816
  - merging two projected plain-xml networks with different offsets is now working. Issue #12841
  - Option **--numerical-ids** now also applies to traffic light IDs. Issue #12886
  - Building networks with Two-way-left-turn-lane is now supported. Issues #12917
  - OpenDRIVE: lane access declarations are now imported. Issue #12804

- netedit
  - Added menu entry for directly calling netgenerate and instantly editing the generated network. Issue #2393
  - Added menu and dialogs for calling python tools without using the command line. Issue #4138
  - When calling netdiff from tools menu, the computed differences may be selectively imported for visualization (i.e. differently colored shapes for added, deleted and modified edges). Issue #2736
  - All objects with a name attribute can now be located by name using the locate-dialog. Issue #12686
  - Inspect mode now allows inspecting individual objects that are part of a selection via ALT+LEFT_CLICK. Issue #12690
  - In data mode, overlapped data elements list is now sorted by interval begin. Issue #11330
  - If a route is selected it will always be shown on top of other overlapping routes. Issue #12582
  - The vehicle type "DEFAULT_RAILTYPE" can now be used for defining trains. Issue #6752
  - The network is automatically recomputed (if needed) when trying to create an E2 multilane detector. Issue #12763
  - Selection files with `node:NODE_ID` are now supported (also in sumo-gui). Issue #13061
  - Delete mode now features a button to toggle all protections. Issue #13073
  - Option dialogs are now resizable. Issue #13099

- sumo-gui
  - Dynamically modified values for `latAlignment` (i.e. when preparing to turn) are now listed in the type-parameter dialog. Issue #12579
  - The attribute value that is used for scaling edge widths can now be drawn as an annotation. Issue #12544
  - Statistic output now includes person teleport count. Issue #12271
  - Option **--alternative-net-file** can now be used to load secondary network shapes. Hotkey <kbd>Ctrl</kbd> + <kbd>K</kbd> can be used to switch between primary and secondary shapes. This is intended to support rail simulations where geographical and abstract shapes are available. Issue #11551
  - Added `.gif`-file support to 3D view. Issue #12672
  - Improved position and scale of vehicle lights in 3D View #12752
  - Add 3D person shapes for 3D View. Issue #11144
  - Background images can now be added via file-dialog. Issue #1627 (also for netedit)
  - Edge context-menu function *select-reachable* now ignores lane direction when selecting vClass "pedestrian". Issue #12801
  - Polygons are moved to the lowest layer upon pressing 'ALT'. Issue #12134

- meso
  - When using option **--meso-lane-queue**, laneData can now be used for lane specific outputs (except trackVehicles=true) #12854

- netgenerate
  - Added option **--spider.attach-length** to create outer extensions to the network. Issue #12928

- TraCI
  - Added function `vehicle.setLateralLanePosition`. Issue #12568
  - Function `vehicle.setStopParameter` now supports "onDemand". Issue #12632
  - Functions [vehicle.getParameter](TraCI/Vehicle_Value_Retrieval.md#supported_device_parameters) and [simulation.getParameter](TraCI/Simulation_Value_Retrieval.md#device_parameter_retrieval) can now retrieve various aggregated trip statistics. Issue #12631
  - TraCIConstants are now available as static values for the Java bindings. Issue #12371
  - Added function `vehicle.getNextLinks` to retrieve all upcoming intersections. Issue #12551
  - Added functions `lane.setChangePermissions` and `lane.getChangePermissions` to dynamically modify 'changeLeft' and 'changeRight' attributes. Issue #12562
  - Simpla can now report platoon-based information and statistics. Issue #12124
  - Added function `traci.executeMove` to only do vehicle movements and allow further interaction before writing the outputs at `simulationStep`. Issue #11091

- tools
  - gtfs2pt.py: multiple improvements to route matching. Issue #12834 (Thanks to Gladys McGan)
  - Visualization tools now support option **--alpha** to set background transparency. Issue #12556
  - Added new tool [generateRerouters.py](Tools/Misc.md#generatererouterspy) to generate rerouters for a given set of closed edges including automatic computation of suitable notification edges. Issue #12510
  - Added new tool [split_at_stops.py](Tools/Net.md#split_at_stopspy) to ensure that each bus/train-stop has its own edge. Issue #12519
  - racing.py: now supports collision sound. Issue #12540
  - randomTrips.py: Added option **--random-factor** to apply random disturbances to edge probabilities (i.e. to break up symmetry in small networks). Issue #12603
  - randomTrips.py: Trips no longer start or end inside a roundabout. To restore the old behavior, option **--allow-roundabouts** can be set. Issue #12619
  - osmWebWizard.py: No longer starts/ends trips on motorways or slip roads (except at the fringe). Issue #12620
  - Improved compatibility between sumolib phase definitions and libsumo phase definitions. Issue #12131
  - Added tool [abstractRail.py](Tools/Net.md#abstractrailpy) to generate an abstract/schematic rail network based on a geodetic rail network. Issue #12662
  - Added tool [stationDistricts.py](Tools/District.md#stationdistrictspy) for segmenting a public transport network based public transport stations. Issue #12662
  - Added tool [attributeCompare.py](Tools/Output.md#attributecomparepy) to compare matching attributes across multiple files. Issue #13026
  - attributeDiff.py: Can now optionally group attributes by one or more id-attributes before comparing. #12794
  - attributeDiff.py: optionally write entries that could not be compared (via new options **-m** and **-M**). Issue #12798
  - attributeStats.py: Now supports option **--human-readable-time** (**-H**) to make attributes with large time values more legible. Issue #12822
  - attributeStats.py: Now supports option **--abs** to include statistics on absolute values. Issue #12899
  - sumolib.miscutils functions `parseTime` and `humandReadableTime` now handle negative values. Issue #12821, #12823
  - tlsCycleAdaptation.py now supports more inputs (i.e. TLS with unused stats). Issue #12777
  - routeSampler.py: Now supports option **--keep-stops** to preserve stops from the route input (*only* when the stops are child element of a route). Issue #12901
  - new research intersection Ingolstadt scenario for the SUMO game
  - Traffic light game now supports showing score for built-in adaptive algorithms. Issue #12915
  - Fixed bug where users could skip yellow phase in traffic light game. Issue #12971
  - netdiff.py: Now writes configuration-style header in diff files. Issue #13036
  - many more python tools now support configuration files
  - plotXMLAttributes.py:
    - can plot by sorting rank with attribute value `@RANK`. Issue #12607
    - can plot by input order with attribute value `@INDEX` (note that this was the behavior of @RANK in 1.16.0). Issue #12607
    - can plot number of occurrences (histogram) with attribute value `@COUNT`. Issue #11971
    - supports binning of data via options **--xbin** and **--ybin** (i.e. for making histograms)
    - supports bar plots via option **--barplot** and **--hbarplot**. Issue #12612, #12833
    - supports box plots via option **-x \@BOX** or **-y \@BOX**. Issue #11994
    - supports clamping of data range via option **--xclamp A:B** and **--yclamp C:D**

- Other
  - Further work on translation. Issue #12611
  - Added "Feedback" dialog in Help menu of netedit and sumo-gui. Issue #12550

### Miscellaneous

- Netedit help menu now links to to netedit shortcuts. Issue #12626
- Removed libsumo / libtraci from the binary windows release to avoid version mismatch. Instead the user must install the bindings for their installed python version via `pip`. Issue #11992
- Stop-output no longer contains attribute `delay` if a stop does not define the `until` attribute. Formerly, the value of *-1* would be written which is ambiguous in conjunction with negative delays caused by TraCI or **--use-stop-ended**. Issue #12883
- Added an installer for the extra version. Issue #8688
- Removed tool `plotXMLAttr.py` since its functionality is covered by `plotXMLAttributes.py` (tool moved to 'purgatory' folder). Issue #11994.
- Netgenerate default parameters where changed to reduce the default network size for spider and random networks. Issue #12927
- Added translation to Italian. Issue #13004
- plot_net_dump.py: no longer supports setting a default color with option **-c**. Instead the long option **--default-color** must be used. The option **-c** is now used for loading of a tool configuration file instead. Issue #13068
- Added [documentation page](Netedit/attribute_help.md) for all xml elements and their attributes. Issue #12658


## Version 1.16.0 (07.02.2023)

### Bugfixes

- Simulation
  - Improved simulation speed in large networks with simulation routing. Issue #12416 (regression in 1.2.0)
  - Fixed invalid blocked state while decelerating and trying to perform lane change. Issue #12108
  - Fixed invalid braking at internal junction. Issue #12000
  - Fixed invalid right of way rules for two conflicting connections with internal junctions. Issue #11988
  - Fixed incomplete vehroute-output when using option **--vehroute-output.sorted** and some cars or vehicle do not finish their journey. Issue #12049
  - stop-output now always contains the correct 'started' values even if other values were part of the input. Issue #12125
  - Fixed error when trying to use laneChangeModel *DK2008* with continuos lane change model. Issue #12144
  - Fixed invalid lane choice in the presence of lane change restrictions. Issue #12118, #12461
  - Fixed crash in public transport scenario with looped routes. Issue #12150
  - Fixed invalid error when loading edgeData with negative electricity consumption. Issue #12172
  - Fixed collision on bidi edge. Issue #12393
  - Fixed invalid switching in branching 'actuated' traffic light programs. Issue #12265
  - Fixed invalid bike lane detector placement for 'actuated' traffic lights. Issue #12266
  - Fixed vehicle angles when using 'lcSigma' with the continuos lane change mode. Issue #12201
  - Fixed inconsistency in waitingTime definition between tripinfo-output and accumulated waitingTime. Issue #12287
  - Fixed invalid lot assignment for onRoad parkingArea. Issue #12330
  - Fixed invalid change to lane with stopped leader. Issue #12113
  - Fixed invalid error when using departSpeed values 'avg', 'last', 'desired' or 'speedLimit' near a minor link or speed reduction. Issue #12398, #12401
  - Vehicles no longer become stuck when using a waypoint with parking=true. Issue #12468
  - The simulation now aborts after an unknown stoppingPlace has been loaded for a vehicle. To obtain the old behavior, option **--ignore-route-errors** may be used. Issue #12487
  - Fixed invalid braking on junction. Issue #12511
  - Bluelight device: probabilistic rescue lane formation no longer depends on step-length. Issue #12516
  - Fixed invalid schema error when reading vehroute-output with human readable times. Issue #12545
  - Traffic light type 'delay_based' now adheres to the specified `maxDur` phase length. Previously phases could be prolonged beyond maxLength in the absence of other traffic. The old behavior can be enabled by setting `<param key="extendMaxDur" value="true"/>`. Issue #12553
  - Routing algorithm CH now uses updated travel times from routing device. Issue #9803
  - intermodal simulation
    - Fixed crash when defining ride without lines. Issue #12167 (regression in 1.11.0)
    - Fixed invalid walking distance output related to lengths of crossings and walkingArea paths. Issue #11983
    - Fixed bug where vehicle with short boardingDuration fails to board passengers after deboarding. Issue #12168
    - Fixed crash when letting persons route between identical junctions. Issue #12242
    - Containers and persons are now unloaded in parallel. Issue #12385
    - Fixed premature simulation end with triggered stop and combined loading/unloading. Issue #12386
    - Fixed unsafe deceleration at pedestrian crossing. Issue #12455
    - Pedestrian no longer walk onto crossings which are fully blocked by traffic. Issue #12507
  - railway fixes
    - Fixed invalid insertion delay on bidirectional track. Issue #12423, #12079
    - Fixed unsafe insertion after parking stop. Issue #12425
    - Fixed deadlock on bidirectional track involving trains that arrive within a block. Issue #12184
  - sublane model fixes:
    - Fixed invalid emergency braking for junction foe. Issue #12202
    - Fixed inconsistent computation for vehicle back position. Issue #12146
    - Fixed unsafe follow speed on multi-lane turning edge. Issue #12204
    - Fixed failure in cooperative speed adaptation at low step length. Issue #12283

- netedit
  - Fixed crash when adding green verge to an edge selection. Issue #12446 (regression in 1.10.0)
  - Fixed bug where stops on looped routes where saved in an invalid order. Issue #12054 (regression in 1.12.0)
  - Right click on elements above a polygon now acts on the top element again. Issue #12111 (regression in 1.14.1)
  - Fixed invalid position of start and end edge geometry points after merging geometry points in move mode. Issue #12178 (regression in 1.14.1)
  - Fixed bug where during creation of new edge, sometimes, the candidate "to" junction wasn't drawn with a magenta dotted contour. Issue #12013 (regression in 1.15.0)
  - Fixed crash when using 'convert to roundabout' from the junction context menu. Issue #12355 (regression in 1.15.0)
  - Fixed invalid selection outline. Issue #12033
  - Fixed bug where right-click object-choice-menu lists the same object twice. Issue #12034
  - Junctions can now be deleted when covered by a walkingarea. Issue #12070
  - Junctions can be merged by moving when the background grid is active. Issue #12080
  - Fixed crash when using option **--prefix**. Issue #12024
  - Fixed bug that permitted invalid combination of stops and and via attribute. Issue #11961
  - The grouping of inspected overlapped elements no longer includes invisible elements. Issue #12126
  - Custom `loadingDuration` and `boardingDuration` are written as s instead of ms (also affects duarouter). Issue #12383
  - Fixed invalid default edge attributes after setting a template. Issue #12392
  - Moving a selected edge with selected junctions now also moves the custom edge end points. Issue #12445
  - Fixed invalid conversion between periodic flow rate and poisson flow rate. Issue #12451
  - Calibrator-flow attributes `begin` and `end` are now loaded correctly. Issue #12470

- sumo-gui
  - Fixed invalid camera position after tracked vehicles exits the simulation. Issue #12137 (regression in 1.13.0)
  - Fixed invalid right-click target when there is a pedestrian crossing on top of a polygon. Issue #12523 (regression in 1.15.0)
  - Pedestrians now follow the exact shape of access lines while in access stage. Issue #12116
  - Aggregated warning summary is now written at simulation end. Issue #12209
  - Fixed invalid objects in right-click disambiguation menu. Issue #12046
  - Fixed initial positions of some dialogs that were too high. Issue #11936
  - Fixed crash on saving gui settings to registry (debug mode only). Issue #11595
  - Pressing Escape key in OSG view no longer makes it unresponsive to further control input. Issue #12313
  - Trailer and Semi-Trailer vehicles are now drawn correctly during opposite-overtaking. Issue #12331
  - Fixed bad default settings when loading a 2D viewport in OSG view. Issue #12348
  - Fixes 3D view boundary coordinates after rotating the view. Issue #11941
  - Fixed crash on quick-reload reloading. Issue #12367
  - Fixed crash when using 'select reachability' (related to negative edge speeds). Issue #12400, #12403
  - Reload hotkey no longer takes effect when running with TraCI. Issue #12431
  - Fixed invalid pedestrian position while passing a short walkingarea. Issue #12456
  - Right-click on vehicles and person no longer fails on very wide lanes. Issue #12505
  - Fixed fluctuating rail width when using edge size-exaggeration at varying zoom levels. Issue #11832

- meso
  - Stopping at pos=0 is now working. Issue #12240
  - Picking up persons and containers with `lines="ANY"` is now working. Issue #12241

- netconvert
  - Fixed xml-validation error when importing MATSim network. Issue #12509 (regression in 1.15.0)
  - Fixed unnecessary dead-end lanes at large intersections. Issue #2472
  - Fixed invalid OpenDRIVE output when writing traffic signals with signal groups. Issue #11980
  - Fixed invalid OpenDRIVE output for lefthand networks. Issue #11995, #12038, #12047
  - Fixed invalid right of way rules when connections from the same edge merge and both have internal junctions. Issue #11988
  - Fixed invalid right of way rules causing mutual conflict at multimodal priority-junction. Issue #5609
  - Fixed bug where **--junctions.join** failed in multimodal networks. Issue #6495
  - Fixed bug where **--junctions.join** joins to much. Issue #10589
  - Fixed invalid dead-end when using option **--osm.turn-lanes**. Issue #12042
  - Fixed invalid plain xml output (after reading an invalid network). Issue #12086
  - Fixed invalid guessed connections. Issue #10771, #10978, #2472, #12181, #12327
  - Fixed missing bidi edges in generated network. Issue #12127
  - OSM import no longer ignores `spreadType` in typemap. Issue #12141
  - Fixed invalid bike lane in OSM import. Issue #12216
  - Fixed invalid bidirectional combined foot/bike path in OSM import. Issue #12214
  - Traffic light building on large intersections now adds extra red time after left-turn phases to ensure safety. Issue #10796
  - Fixed creation of invalid pedestrian crossings. Issue #12234
  - Added missing (guessed) connections when an outgoing edge has additional lanes. Issue #8899
  - Fixed invalid link state at zipper junction. Issue #7676
  - Prohibitions involving edges with underscore in their name are now working. Issue #12419
  - Turnarounds are now correctly added if the innermost lane prohibits passenger traffic. Issue #12447
  - OpenDRIVE-imported networks no longer include guessed turn-arounds at real intersection by default (option **--no-turnarounds false** can be used to replicate the old behavior). Issue #12448
  - Line name from **--ptline-files** is now exported. Issue #12497
  - A `<split>` at pos=0 no longer ignores speed. Issue #12526
  - Distance (kilometrage) is now preserved when adding `<split>` element. Issue #12527
  - Custom edge length is now preserved when adding `<split>` element. Issue #12529

- polyconvert
  - Fixed invalid polygon output for some line based inputs. Issue #12161

- duarouter
  - Fixed crash on unknown landmark edge. Issue #12438
  - Fixed invalid routing result by routing algorithm A* / "astar" (minor difference caused by internal edge lengths). Issue #12463

- TraCI
  - Fixed moveToXY failure at parallel internal junction. Issue #12065
  - Foe lanes for crossings can now be retrieved. Issue #12059
  - `Connection.close()` now closes simulation. Issue #12133
  - Fixed invalid behavior when mixing stops on normal and internal lanes. Issue #11885
  - Function `traci.edge.setAllowed` is now working. Issue #12305
  - Context subscriptions to the simulation domain now always return all requested objects regardless of range argument. Issue #12306
  - `trafficlight.swapConstraints` now preserves params (and swaps params for `bidiPredecessor`). Issue #12326
  - Fixed problems when using libsumo with gui. Issue #12285, #12021
  - Fixed crash on rerouting after insertStop/replaceStop. Issue #12387
  - Signal states are now updated immediately after calling `trafficlight.setProgramLogic`. Issue #12414
  - Cycle length is now updated after call to `traci.trafficlight.setProgramLogic`. Issue #12357

- Tools
  - plot_net_dump_file.py: plotting a single measure is working again. Issue #11975 (regression in 1.15.0)
  - generateTurnRatios.py: fixed unsorted intervals in output when using python2. Issue #12019
  - tlsCycleAdaptation.py: Fixed bug where controllers differed in cycle length when setting option **--unified-cycle**. Issue #12045
  - tlsCycleAdaptation.py: Fixed mismatch between optimal and actual cycle duration. Issue #12068
  - generateRailSignalConstraints.py: Fixed inconsistent bidiPredecessors. Issue #12075
  - generateRailSignalConstraints.py: Added missing insertion constraint after invalidated stop. Issue #12474
  - tracemapper.py: Fixed duplicate consecutive edges in route. Issue #12094
  - cutRoutes.py: No longer writes persons without a plan. Issue #12245
  - routesampler.py: Option **--total-count** is now compatible with **--weighted**. Issue #12284
  - net2geojson.py: Fixed crash when trying to import network without geo projection (now gives an error message instead). Issue #12295
  - gtfs2pt.py: Fixed missing transport modes. Issues #12277
  - gtfs2pt.py: Fixed mapping of bus and tram stops in multimodal networks. Issue #11802, #11849
  - gtfs2pt.py: Output is now usable with duarouter. Issue #12333

- All Applications:
  - Fixed XML-validation error when SUMO_HOME is not set. Issue #12138 (regression in 1.15.0)
  - Fixed crash if gzipped outputfile cannot be opened. Issue #11954

### Enhancements

- Simulation
  - Elements `<ride>` and `<transport>` can now be defined without attribute `lines` and default to a value of `ANY` (taking any eligible vehicle that stops at the destination). Issue #12167
  - Option **--fcd-output.attributes** now supports the attribute `speedLat` for writing lateral speeds. Issue #12213
  - Stops now support attribute `jump="TIME"` to model explicit jumps (teleports) between disconnected locations. Issue #12268
  - The randomness in rescue lane formation can now be configured with bluelight device [parameters](Simulation/Emergency.md#further_parameters). Issue #12437
  - Added option **--pedestrian.striping.reserve-oncoming.max** (default 1.28) to configure an upper limit on stripes to reserve for oncoming pedestrians. Issue #12506

- netconvert
  - The right-of-way rules to take effect when switching a traffic light off, can now be configured as 'allway_stop'. This is the new default for NEMA-type controllers. Issue #12043
  - Improve traffic light programs in networks with separated bicycle paths. Issue #10039
  - OpenDRIVE outputs now supports export of loaded POIs and polygons as road objects. Issue #12060
  - When setting option **--tls.guess**, roads without conflict are excluded from the threshold-heuristic. Issue #6513
  - Improved heuristic for generating rail connections at sharp angles. Issue #12119
  - Warnings of E3 detectors and from the SSM device can now be aggregated. Issue #12149
  - Individual lane widths are now import from OpenStreetMap. Issue #12162
  - Lane divider style can now be exported from OSM to OpenDRIVE. Issue #12158
  - Traffic signs from OSM can now be exported to OpenDRIVE. Issue #12231
  - Pedestrian crossings are now exported to OpenDRIVE. Issue #12229
  - Added option **--osm.crossings** to import pedestrian crossings from OSM. Issue #12238
  - Now separating lanes for bikes and pedestrians according to OSM declarations. Issue #12215
  - Option **--osm.bike-access** now serves to add any additional bike lanes declared in OSM without the need to load another typemap (The typemap may still be used to customize bike lane widths for different categories of lanes). Issue #12228
  - Added option **--tls.rebuild** to rebuild all loaded traffic light plans. Issue #12250
  - Added option **--tls.guess-signals.slack** to identify more controlled intersections by guessing from surrounding *simple* tls nodes. Issue #12249
  - Option **--tls.guess-signals** no longer requires signals on non-passenger edges to interpret an intersection as controlled. Issue #12260
  - Option **--junctions.join** can now join intersections with more than 4 incoming edges. Issue #12261
  - Walkingarea shapes for pure pedestrian intersections now match the junction shape. Issue #12377
  - Added option **--plain-output.lanes** to include all lane attributes in plain-xml output. Issue #12443

- netedit
  - It is now possible to load and save a *.sumocfg* file and also to edit all sumo options (SHIFT-F10). Issue #11896
  - If a .sumocfg is set, it will be used automatically when launching sumo from the menu. Issue #12450
  - Added support for saving and loading *.neteditcfg* files to simplify simulation project management. Issue #12309
  - Added TimeStamp in Undo-Redo list. Issue #11744
  - Now drawing red line between edge geometry points (if the points are not along the edge). Issue #11530
  - Can now disable drawing dotted contours in visualization settings. Issue #11662
  - Now showing a warning dialog if user tries to create a TLS in a junction with all connections set as uncontrolled. Issue #6382
  - Removed dialog-button for permission 'disallow' attribute to avoid confusion. Issue #11940
  - Saved sumocfg now includes version data. Issue #11294
  - Multiple persons or personFlows in the same location are now indicated. Issue #10724
  - Added MeanData mode to create and modify `<edgeData>` and `<laneData>` definitions. Issue #11897
  - Recently used files are now in a sub-menu. Issue #12025
  - Geometry points are now drawn above everything else in move mode. Issue #11725
  - Geometry points now change color in move mode to indicate whether a click would create or merge points. Issue #12177
  - Move mode can now toggle whether closely spaced geometry points shall be automatically removed. Issue #12244
  - Stops now support attribute 'jump'. Issue #12269
  - Crossing mode now ensures that only sensible crossings can be defined. Issue #12366
  - Vehicle inspect mode now allows selecting vehicle type from a drop-down list. Issue #12379
  - Added edge color legend to person and container mode. Issue #11613
  - Added edge color legend when creating e2 detectors along multiple lanes. Issue #11334
  - Moving a single selected edge now moves its whole geometry including endpoints. Issue #12442
  - Parameters for the W99 carFollowModel can now be configured. Issue #12290

- sumo-gui
  - When option **--use-stop-ended** is set, show-route mode now labels the 'ended' time of stops. Issue #11833
  - The view now updates after loading a selection file. Issue #12191
  - Whenever the simulation has tripinfo-devices, the trip based statistics are available in the network parameter dialog. Issue #12207
  - A rainbow color scale can now be defined between upper and lower "hiding" thresholds even when there is no data yet. Issue #11978
  - Aggregated detector values are now listed in their respective parameter dialog. Issue #12031
  - Detector outputs are now flushed at simulation end even while the gui remains open. Issue #12293
  - Added guiShape "aircraft". Issue #12314
  - Added vehicle setting to maintain orientation after reversal. This achieves a more realistic visualisation of reversing trains and (grounded) aircraft. Issue #12140
  - Added settings to show/hide HUD elements in 3D view. Issue #12294
  - Added terrain to 3D view (using flat background color) #12279
  - Added Fly Movement mode in SUMO 3D View for Windows OS (Linux still pending). Issue #11473
  - Color legend decoration now includes the name of the coloring scheme. Issue #11967
  - All dialogs now remember their previous position. Issue #11962
  - Window menu now includes entries for opening a new view. Issue #12417
  - Train insertion that is delayed due to oncoming trains is now indicated in the lane parameter dialog. Issue #12421
  - Added vClass icons in the "Select lanes which allow" and "Select reachable" lists. Issue #12429
  - Scaling edge width by loaded edge data is now supported. Issue #9216

- TraCI
  - Added functions `vehicle.getDeparture` and `vehicle.getDepartDelay`. Issue #3036
  - Added functions to retrieve aggregated inductionLoop detector measures. Issue #12030
  - Added functions to retrieve aggregated lanearea detector measures. Issue #12029
  - Added functions `vehicle.getLoadedIDList` and `vehicle.getTeleportingIDList` to retrieve the corresponding vehicles (some of which could not be retrieved with getIDList). Issue #2338
  - Simpla: The maximum length of a platoon can now be configured. Issue #11426
  - The JAVA bindings now facilitate casting of subscription results. Issue #8930

- Tools
  - osmWebWizard.py: Add checkboxes to select/deselect groups of elements in the road types tab. Issue #10692
  - runSeeds.py: Now forwarding unknown options to application call. Issue #12312
  - runSeeds.py: Now supports setting a list of applications and a list of configurations to run all at once (with results in subfolders). Issue #12311
  - routeSampler.py: Major improvement in sampling speed. Issue #12319
  - routeSampler.py: Full optimization now skips initial sampling for further speed-up. Issue #12307
  - [attributeStats.py](Tools/Output.md#attributestatspy): Permit parsing multiple elements and attributes at once. The new default is to parse all elements and attributes. Issue #12317
  - attributeStats.py now includes count in xml-output. Issue #12337
  - gtfs2pt.py: Now writing short route id and headsign as params. Issue #11819
  - plot_trajectories.py: Now support common visualization options. Issue #11991
  - Many visualization tools can now configure linestyle and marker style. Issue #11985
  - countEdgeUsage.py: Now permits loading named routes. Issue #12010
  - implausibleRoutes.py: Added options **--additional-file**, **--unsorted-input** and **--ignore-errors** which are passed to duarouter when needed. Issue #12090
  - implausibleRoutes.py: Added options to handle unsorted input and additional files when duarouter is being called. Issue #12069
  - tracemapper.py: Added option **--vehicle-class** to guide edge mapping. Issue #12117
  - net2geojson.py: Added option **--boundary** to write polygons instead of center lines. Issue #12296
  - stateReplay.py: Now works on Windows. Issue #12298
  - checkStopOrder.py: now indicating terminal stops and overtaking events in **--stop-table** output. Issue #12471
  - generateRailSignalConstraints.py: Now handling actual times ahead of schedule. Threshold configurable with new option **--premature-threshold**. Issue #12530
  - Added new tool [filterElements.py](Tools/Xml.md#filterelementspy) to filter elements from an xml file (either all instances or filtered by attribute value). Issue #12304
  - Added new tool [attributeDiff.py](Tools/Output.md#attributediffpy) to compute the numerical difference between two xml files with the same structure. Issue #12318
  - Added new tool [fcdDiff.py](Tools/Output.md#fcddiffpy) to compare two fcd-output files (by vehicle id and time). Issue #12233
  - Added new tool [fcdReplay.py](Tools/Visualization.md#visulizing_fcd-data_as_moving_pois) to review a fcd recording as moving POIs (optionally alongside a running simulation). Issue #12433
  - plotXMLAttributes.py
    - can now plot data without assigning ids to the data points. Issue #11969
    - can now plot categorical (non-numerical) data and also a mix of data types. Issue #11970, #11976
    - categorical labels can be sorted and filtered by loading a list of labels with option **--xticks-file** and **--yticks-file**. Issue #12091
    - now supporting additional visualization options (linestyle, markers, grids, ...). Issue #11972
    - plots a marker on a single point if the input data is only one point. Issue #11974
    - Added the possibility to use wildcards with option **--filter-ids**. Issue #11981
    - Attribute options now permit setting a list of attributes. Issue #12015
    - Can now display times as hours by setting **--xtime0** and **--ytime0** (also applies to other plotting tools). Issue #12011
    - csv-output is now directly usable with gnuplot. Issue #12345

- Other:
  - Added option **--language** to select language for warnings and messages of all applications. Issue #11237
  - Added *Language* menu to sumo-gui and netedit to set the interface language persistently. (Can also be set temporarily with **--language**) Issue #12518
  - You can help to complete and improve the translation at https://hosted.weblate.org/projects/eclipse-sumo/ Issue #11859

### Miscellaneous

- Added exemplary plots to many simulation-output documentation pages (with linked example commands to create them).
- Netconvert options for writing polygons are now documented. Issue #12135
- Updated Windows MSVC runtime libraries and Xerces-C to 3.2.4
- Improved pydoc for TraCI functions that modify vType attributes. Issue #11943
- Added all required fmi functions required for FMPy version 0.3.13. Issue #12199
- Removed Netedit options **--SUMOConfig-output, --additionals-output, --demandelements-output, --dataelements-output**. Instead, the corresponding input file path is used for output. Issue #12388
- Some netedit buttons for saving are now grouped in a sub-menu to avoid clutter. Issue #12360
- Test extraction now works for the San Pablo Dam tutorial. Issue #12466
- The netdiff tool accessible via the 'Tools' menu has been temporarily disabled due to stability issues. #12435


## Older Versions

- [Changes to versions 1.12.0, 1.13.0, 1.14.0, 1.14.1 and 1.15.0 (2022 releases)](ChangeLog/Changes_in_2022_releases.md)
- [Changes to versions 1.9.0, 1.9.1, 1.9.2, 1.10.0 and 1.11.0 (2021 releases)](ChangeLog/Changes_in_2021_releases.md)
- [Changes to versions 1.5.0, 1.6.0, 1.7.0 and 1.8.0 (2020 releases)](ChangeLog/Changes_in_2020_releases.md)
- [Changes to versions 1.2.0, 1.3.0, 1.3.1 and 1.4.0 (2019 releases)](ChangeLog/Changes_in_2019_releases.md)
- [Changes to versions 1.0.0, 1.0.1 and 1.1.0 (2018 releases)](ChangeLog/Changes_in_2018_releases.md)
- [Changes to versions 0.29.0, 0.30.0, 0.31.0 and 0.32.0 (2017 releases)](ChangeLog/Changes_in_2017_releases.md)
- [Changes to versions 0.26.0, 0.27.0, 0.27.1 and 0.28.0 (2016 releases)](ChangeLog/Changes_in_2016_releases.md)
- [Changes to versions 0.23.0, 0.24.0 and 0.25.0 (2015 releases)](ChangeLog/Changes_in_2015_releases.md)
- [Changes to versions 0.20.0, 0.21.0 and 0.22.0 (2014 releases)](ChangeLog/Changes_in_2014_releases.md)
- [Changes to versions 0.17.0, 0.17.1, 0.18.0 and 0.19.0 (2013 releases)](ChangeLog/Changes_in_2013_releases.md)
- [Changes to versions 0.14.0, 0.15.0 and 0.16.0 (2012 releases)](ChangeLog/Changes_in_2012_releases.md)
- [Changes to versions 0.12.3, 0.13.0 and 0.13.1 (2011 releases)](ChangeLog/Changes_in_2011_releases.md)
- [Changes to versions 0.12.0, 0.12.1 and 0.12.2 (2010 releases)](ChangeLog/Changes_in_2010_releases.md)
- [Changes to versions 0.10.0, 0.10.1, 0.10.2, 0.10.3, 0.11.0 and 0.11.1 (2009 releases)](ChangeLog/Changes_in_2009_releases.md)
- [Changes in versions 0.9.0 up to 0.9.10](ChangeLog/Changes_from_version_0.9.0_up_to_0.9.10.md)
- [Changes in versions pre0.8 up to 0.8.3](ChangeLog/Changes_from_version_pre0.8_up_to_0.8.3.md)
