---
title: ChangeLog
---

## Git Main

### Bugfixes

- sumo
  - fixed invalid driveWay foes in moving-block mode #17623
  - fixed invalid estimation of pickup-traveltime during taxi dispatch #17631, #17629
  - stopping on a long busStop before reaching the designated spot due to jamming now permits passengers to exit if the vehicle is fully within the busStop #17635
  - Fixed failure to group taxi passengers when the dispatch-period is low #17644

- sumo-gui
  - Fixed crash when drawing persons #17616
  - Fixed rare crash when vehicle parameter dialog is open while a vehicle exits the simulation #17637

- netedit
  - python tool dialogs now permit selecting multiple files #17615, #17619 (regression in 1.25.0)
  - fixed crash when calling python tool and using the 'back' button #17618 (regression in 1.25.0)

  
- netconvert
  - NEMA computation now works for 4-arm junction without right-turns (also affects some signal plans of non-NEMA junctions) #17610  

- TraCI
  - `trafficlight.getSpentDuration` now works correctly after calling `setRedYellowGreenState` #17598

- tools
  - osmWebWizard.py: fixed various platform issues that prevent running. #17503
  - patchRailConflicts.py: no longer declaring rail signals that do not control any links #17588
  - plotXMLAttributes.py: fixed missing labels on barplots with a non-numerical axis #17611


### Enhancements

- sumo
  - Fixed invalid parking positions in network with [length-geometry mismatch](Simulation/Distances.md#length-geometry-mismatch) #17640
  - parkingAreas with `onRoad="true"` are now filled from the downstream end if overtaking is not possible (i.e. on rails) #17627
  - Taxis with idle algorithm `taxistand` now advance in queue when parking with `onRoad="true"` and overtaking is not possible #17632
  - Added parkingArea attribute `reservable` ({{DT_BOOL}}) which avoids multiple vehicles from targeting the same spot when [cruising for parking](Simulation/Rerouter.md#rerouting_to_an_alternative_parking_area) #17628
  - Added taxi param `<param key="device.taxi.swapGroup" value="<GROUP_NAME>"/>` which permits idle taxis to receive dispatch tasks before pickup is complete from other taxis in the same swapGroup if that would reduce time to pickup #17639
  - carFollowModel *ACC* now supports driverstate device (but it is only active when setting vType attribute `applyDriverState="1"`) #17633
  - Option **--vtk-output** now supports writing data at sub-second simulation step-length #17645

- meso
  - edge-type specific meso parameters now support `edgeLength` #17582
  - now supporting battery device in #13843

- tools
  - osmWebWizard.py: now automatically saves osmGet configuration for easier updating of a scenario #17570
  - osmWebWizard.py: now permits selection of public transport modes to import #8628
  - osmWebWizadr.py: now permits to set a 'verbose' checkbox which gets forward to all applications #17573
  - osmGet.py: now attempts to download again after timeout and tries to use proxies (also affects osmWebWizard) #17597
  - plotXMLAttrbutes.py: added option **--join** to configure the separator when joining values or labels #17612
  - generateStationEdges.py: added option **--join-stations** to generate a single access edge for all stops with the same name #17625
  - generateStationEdges.py: added option **--build** to automatically create the extended network #13383



## Version 1.26.0 (29.01.2026)

### Bugfixes

- sumo
  - Fixed deadlock in roundabout #17330 (regression in 1.24.0)
  - Fixed inconsistent computation of attribute flow in edgeData output #17349
  - Fixed invalid braking when trying to avoid junction blockage #17318
  - Fixed incosistent lane-changing inside roundabout while on an internal lane #17336
  - Fixed emergency braking in roundabout #17306
  - Fixed invalid warning after train rerouting #17343
  - Fixed problems when configuring calibrator with period > 1 (period now truncated to 1) #6589, #17371
  - Fixed invalid error about stops when configuring vehicles to ignore temporary permission changes #17380
  - Fixed unsafe sublane-change due to missed neighbor vehicles #17388
  - Fixed invalid expected sublane speeds at lane split with forbidden connections #17390
  - Fixed failure to return from outside lane bounds after lane width change in the sublane model #17383
  - Fixed collisions involving pedestrian crossings #17385
  - Fixed invalid braking after lane change #17408
  - Fixed bug where vehicles block each others lane change #10763
  - Fixed right-of-way violation when entering roundabout #17409
  - Fixed invalid braking at intersection with sublane model #17410
  - Fixed unsafe lane-changing ahead of roundabout #17411
  - Fixed broken vehicle flows where total vehicle count overflows a 32-bit integer #17396
  - A closingReroute with configured permissions no longer affects vehicle with an unaffected vehicle class #17426
  - Fixed missing railsignal vehicle-events on sub-driveways #17442
  - Fixed unsafe train insertion when driveways start and end in the middle of the block #17453
  - Fixed missing driveway foes when a sequence of bidi-edges is interrupted by a unidirectional edge #17367
  - Fixed bug where pedestrian walks into vehicle #17462
  - Fixed unsafe lookahead time when computing crossing conflicts between vehicles and pedestrians (now attribute `jmTimegapMinor` takes effect) #17463
  - vehicles approaching a non-priority crossing no longer yield unless a person steps onto the crossing #17465
  - Fixed infinite traffic from poisson flow at specific random seeds #17468
  - Fixed inconsistency where a route with a single edge and departPos > arrivalPos causes no error on loading but rerouting (now results in a warning) #10246
  - Fixed invalid error when combining option **--device.rerouting.mode** with taz-routing #17490
  - Fixed undefined behavior when computing route cost between taz #17489
  - Fixed missing events when vehicle is inserted on `<instantInductionLoop>` #17510
  - Fixed crash when running rail signal simulation with **--tls.all-off** #17516
  - Fixed bug where vehicle stops despite stop attribute `onDemand` when there is no demand #17523
  - Fixed invalid default stop startPos if endPos is defined as negative #17522
  - Fixed unsafe train insertion when combining rail signals with traffic lights #17546, #17547
  - Fixed invalid vehicle mass for personTrip with car #17551
  - Fixed non-deterministic moving-block simulation #17557
  - Fixed invalid signal state when using WAUT and loading state #17526
  - Fixed unsave signal state if a junction where rail lines cross is defined as rail_signal #17580

- netedit
  - lane selection count now updates when selecting with shift-click #17394 (regression in 1.11.0)
  - In move mode, grabbed and merged geometry points are indicated again (with contour instead of color) #17032 (regression in 1.20.0)
  - Fixed crash in calibrator dialog #17398 (regression in 1.25.0)
  - Merging junctions in move mode is working again #17358 (regression in 1.25.0)
  - Creating calibrator flows is working again #17399 (regression in 1.25.0)
  - No longer saving an invalid network when edges from prohibition elements are deleted #17331
  - selection scaling now work for tazRelations #17382
  - Remove some unsupported combinations of taz/junctions for rides, transport, and tranships #17414
  - Fixed bug where paths in the configuration were absolute when they should have been relative #17446
  - Fixed problem when locking lanes #17514
  - Trying to load waypoints with triggers now issues a warning #17534
  - Fixed visualization of parkingArea with onRoad=true or lefthand=true #17499, #17538

- sumo-gui
  - saving selection to file no longer uses **--output-prefix** #17368
  - fixed crash when tracking a vehicle which already left #17472
  - Fixed invalid positioning of parking spaces #17478
  - Fixed crash when person performs a jump #17506
  - Fixed invalid color when vehicle passes a waypoint and coloring *by speed* is active #17524

- netconvert
  - Fixed crash when removing traffic light crossing via xml input #17515
  - Negative split pos is now relative to custom edge length #17527
  - Fixed invalid handling custom edge length when splitting at an existing node #17528
  - Split at position 0 now sets node type #17533
  - Elements written with **--ptstop-output** are now assigned as trainStop where appropriate #17535
  - Elements written with **--ptstop-output** now preserve their 'lines' attribute #17530
  - Fixed invalid busStop direction in ptline-output #17537
  - Fixed invalid ptlines with option --ptstop-output.no-bidi #17571


- duarouter
  - Fixed crash when loading invalid routes with option **--skip-new-routes** and **--ignore-errors** #17348 (regression in 1.25.0)
  - Option **--ignore-errors** now works when origin or destination are prohibited by option **--restriction-params** #17387
  - Any routes that are repaired with option **--repair** no longer trigger an error (and thus do not require option **--ignore-errors** anymore) #17369
  - Fixed invalid route when two stops on the same edge require looping back #17484
  - Fixed invalid route when departPos > arrivalPos and from=to #17482
  - stop arrival times for flows are now shifted #17504

- TraCI / Libsumo
  - function traci.vehicle.rerouteParkingArea now finds looped route from the current edge #17353
  - libsumo macOS wheels work again #15945

- tools
  - osmGet.py: fixed missing road nodes when using option **--shapes** #17293 (regression in 1.20.0)
  - netdiff.py: fixed crash involving removed `<neigh>` attribute #17345
  - generateRerouters.py: Fixed invalid output when no detours are possible #17361
  - generateRerouters.py: now find notification edges for consecutive closed edges #17360
  - tazRel2POI.py: fixed invalid error on skipped taz #17379
  - countEdgeUsage.py: Fixed invalid count for vehroute-output involving replaced routes #17401
  - sumolib.xml.parse_fast: No longer yields records for element names that start with the same string as the requested element #17403
  - gtfs2pt.py: Fixed bug that was causing invalid stop assignments and large detours #17540
  - driveways2poly.py: fixed crash on rail signals without links #17550
  - remap_additionals.py: fixed bug where wrong edge was picked among parallel edges #17556

- Options **--output-prefix** and **--output-suffix** can now be freely combined #17545

### Enhancements

- sumo
  - Routing with randomized weights (**--weights.random-factor**) is now stable with respect to network changes, vehicle composition and state loading. The random noise for each edge only depends on the random seed and vehicle id (also applies to duarouter). If thew new option **--weights.random-factor.dynamic** is set, the randomness in the simulation also varies over time. #17325
  - The new option **--output-suffix** can be used to modify the names of all output files (similar to **--output-prefix**). The suffix will be inserted right before the file name extension. #17338
  - edgeData output definitions now support attribute `aggregate="taz"` which will aggregated data within each loaded taz definition #11104
  - Added option shortcut **-m** for **--edgedata-files** #17400
  - A warning is now given when loading personTrips with mode "public" and no public transport was loaded #2825
  - Departure on closed edge with option to ignore transient permissions (**--device.rerouting.mode 8**) now delays departure instead of raising an error #17461
  - ChargingStation attribute `totalPower` can now be used to limit the total power when charging multiple vehicles at the same time. #17173
  - Timeloss is now discounted when braking/accelerating for planned stops. Thus, a punctual public transport vehicle does not have any timeLoss #5287
  - Tram simulation
    - Tram simulation now defaults to moving-block mode. This can be configured with the new option **--railsignal.moving-block-default-classes** #17542
    - Train insertion in moving-block mode ignores zipper conflicts to improve operations where when rail signals are sparse #17544
    - Rail signals in moving block mode ignore zipper conflicts if they are beyond 200m (configurable with new option **--railsignal.moving-block.max-dist**) #17542

- netedit
  - Automatically sets sumo option **--junction-taz** if at least one vehicle is configured to start/end at a junction #17405
  - The written sumocfg now tracks network file name changes #17314
  - Configurations with unsupported options can now be loaded with warnings instead of errors #17445
  - Added support for junction attribute 'roundabout' #17271

- duarouter
  - The speedFactor configured in a vehicle, trip or flow is now taken into account when computing costs #17424
  - Added option **--max-traveltime** which lets routing fail if traveling takes too long #17422
  - Rerouters with element `closingReroute` can now be loaded from an **--additional-file** to influence routing #12501
  - Rerouters with element `closingLaneReroute` can now be loaded from an **--additional-file** to influence routing #17428
  - consistency of stops and vias is now checked #17485

- netconvert
  - now keeps more stops from OSM despite minor data errors #17575

- TraCI
  - function traci.simulation.findRoute now supports optional attributes departPos, arrivalPos #17352
  - `traci.vehicle.setSpeedMode` bit 2 (ignoring deceleration constraints) now also applies when validating deceleration for `traci.vehicle.setStop` #17477
  - traci and libsumo python libraries now provide the standard __version__ attribute #17366

- tools
  - attributeCompare.py: Now supports special id-attribute @FILE #17334
  - [visum_convertXMLRoutes.py](Tools/Import/VISUM.md#visum_convertxmlroutespy): new tool for importing VISUM routes file #17347
  - generateRerouters.py: Added option **--closed-edges.input-file** for loading edges to close from a selection file #17359
  - generateRerouters.py: major speed-up #17362
  - tazRel2POI.py: Added option **--reference-taz** to allow filtering relations to those that interact with a specific taz #17375
  - tazRel2POI.py: supports multiple taz input files #17378
  - route2OD.py: supports separating multiple input files with ',' #17377
  - edgesInDistricts.py: now supports geo polygons #17376
  - edgeDataDiff.py: now include mean_abs in output #17404
  - generateDetectors.py: supports generating nextEdges attribute #17509
  - addStops2Routes.py: allows placing stops on vias #14818
  - filterElements.py: Added option **--remove-parent** which filters parent element based on child attributes #17539
  - gtfs2pt.py: now warns about large detours (i.e. implausible  routes) #17567
  - patchRailConflicts.py: new tool for adding tram rail signals

### Miscellaneous

- Fixed invalid meta data in python wheels
- Started Korean Language translation #17420
- add manylinux_2_28 support #16771
- dlr-navteq output no longer defaults to option **--numerical-ids** #17520
- It is no longer possible to end a rail_signal block with a traffic light (this was found to be unsafe in the context of single-track operations). Both types of signaling should only ever be combined in tram simulation which defaults to moving block so no adverse effects are expected #17542
- the default download location for the nightly wheels changed to https://sumo.dlr.de/daily/ciwheels


## Version 1.25.0 (13.11.2025)

### Bugfixes

- sumo
  - Fixed invalid consumption/emissions when loading and unloading persons/containers #17152, #17264 (regression in 1.22.0)
  - Fixed crash in rail simulation after rerouting #16958
  - Rerouters with parkingAreaReroute now take into account `parkingBadges` and `acceptedBadges` #16966
  - Output file paths defined with param keys `device.ssm.file` and `device.toc.file` are now interpreted relative to the file in which they are defined. #16967
  - Fixed bug where taxi dispatch algorithm *routeExtension* takes detours when a ride is shared between 3 or more persons #17059
  - Fixed empty missing vehroute-output when **--vehroute-output.sorted** and **--load-state** are used #16987
  - Fixed cases where vehicles would repeatedly fail to decide between overtaking left or right when the middle lane is blocked. #1312, #17116, #17129
  - Fixed bug where opposite direction overtaking failed in the sublane model due to miscalculating the leader vehicle #17125
  - Reduced lateral jump when an emergency vehicle enters a junction from the wrong turning lane #17115
  - Fixed emergency braking when attribute arrivalLane is set #17130
  - Fixed loss of precision when parsing floating point values from [generic vehicle/vType params](Simulation/GenericParameters.md) #17160
  - Fixed bug where taxi dispatcher creates invalid route #17166
  - Fixed bug where intermodal routing computes disconnected taxi trip #17167
  - Fixed initialization of intermodal taxi routing graph when taxis are defined as a flow #17168
  - Fixed crash when loading roads with a rail_signal #17183, #17251
  - Fixed infinite loop when loading tls program with an actuated red phase that has multiple targets #17186
  - Fixed unsafe vehicle insertion ahead of a junction #17192, #17194
  - Fixed crash on missing attribute in parquet output #17195
  - Fixed unsafe driving on green major link with internal junction #17196
  - Left-turning vehicles no longer slow down for outgoing crossing when in a protected (green major) phase #17194
  - Fixed emergency braking in subsecond simulation involving stranded car #17203
  - Fixed error when setting departSpeed="avg" #17208
  - Setting invalid attributes for edgeData `writeAttributes` now always gives a meaningful error message #17230
  - A vehicle that has persons starting inside of it, no longer causes an error when skipped with option **--begin** #9026
  - Option **--load-state.remove-vehicles** now permits to re-use the removed vehicle-id with a new route #17270
  - Fixed crash when vehicle is inserted on an internal lane with invalid route and option **--ignore-route-errors** #17248
  - Fixed inflated density in lane/edgeData output #16241
  - Fixed emergency braking after teleport when using the EIDM carFollowModel #17317
  - Fixed unsafe train insertion before short signal blocks #17323

- netedit
  - Fixed invalid Id when joining traffic light junctions multiple times #17010 (regression in 1.11.0)
  - Removed invalid elements from additional-mode element selector #17297 (regression in 1.11.0)
  - Fixed missing 'allow="authority' attribute in closingReroute #17299 (regression in 1.11.0)
  - Rectangle selection of edges no longer selects additional objects #17213 (regression in 1.20.0)
  - Hints when moving/creating geometry points are working again #17032 (regression in 1.20.0)
  - Fix invalid contour of rerouter intervals when inspecting #17001 (regression in 1.20.0)
  - Fixed inconsistencies when using 'mark as front element' #17285 (regression in 1.22.0)
  - POI locator sorts IDs again #16963 (regression in 1.22.0)
  - E2 detectors can be moved again #15551 (regression in 1.22.0)
  - Tool tip in inspect mode and create-object mode for mouse-over-attribute are working again #17239 (regression in 1.22.0)
  - Adding members to a vTypeDistribution is working again #16948 (regression in 1.23.0)
  - Turnaround connections are now visible for bidi-rail edges (again) #16956 (regression in 1.23.1)
  - Fixed invalid file contents when saving a vTypeDistribution under a new demand file name (regression in 1.24.0)
  - Inspected trip no longer shows superfluous id when gui setting addName is active #17061
  - Transforming a vehicle with route id into a flow now preserves the route id #17017
  - Disabled "Assign E1 Detector" mode for invalid tlTypes #16949
  - Loading a new network now resets the additional, routes and data files #17038
  - Ensuring all text in dialogs is translated #17045
  - Fixed vehicle id written upside down #17060
  - In demand mode/stop mode, embedded routes are no longer listed as candidate parents for new stops #17062
  - Fixed invalid rotation of dotted contours in routeProbe and vaporizers #16911
  - Fixed freeze when saving empty mean data elements #16812
  - POIs no longer write layer and color attributes with the default value #17128
  - Visual scaling of selected edges is now smooth #16977
  - Fixed invalid position of elements of detectors and stopping places in networks with elevation #17189
  - Recently used list no longer contains relative paths when loading files on the command line #17205
  - Fixed crash when saving additional objects after changing lane count #17218
  - netgenerate dialog no longer fails when the output file path contains spaces #17221
  - Fixed problem when adjusting invalid entity positions on edges with custom length value #17222
  - Added contour to all rerouter edges #17244
  - Data intervals with human-readable times can now be loaded #17268
  - Invalid internal edge ids are no longer shown in tls mode #17249

- netconvert
  - Fixed crash when importing OSM data with public transport relations that reference unknown nodes #16953 (regression in 1.24.0)
  - Fixed invalid roundabout detection at junction cluster #16950 (regression in 1.24.0)
  - Fixed inconsistent behavior when setting option **--default.spreadtype** for OSM import #16952
  - Fixed inconsistent behavior when writing spreadType #16951
  - Fixed unstable right-of-way on re-import (due to rounding of speeds) #16971
  - Sidewalks and bike-lanes are now discounted when computing junctionPriorities #1637
  - Fixed invalid ptstop-output when loading stops and splits #17028
  - Fixed missing bidi-edge after split #17033
  - Fixed bug where intersection rules changed on re-import #17077
  - Fixed invalid stranded-on-red conflict in response matrix #17076
  - Fixed crash when loading network with NaN values #17161 (also applies to netedit)
  - Fixed invalid junction shape (with NaN) #17182
  - OpenDRIVE import: now handling problematic geometries that were causing NaN values #17163
  - Fixed misaligned crossing with custom width #17286
  - VISUM importer now handles missing SURFACEID #17307


- sumo-gui
  - Rerouter notification edges are visible again for rerouters created in netedit (with attribute 'pos') #17298 (regression in 1.22.0)
  - Fixed bug where an unrelated vehicle becomes selected after a selected vehicle has left the simulation #16955
  - Edge and junction locator now works in [alternative-network mode](Simulation/Railways.md#abstract_networks) #17022
  - Loaded color scheme no longer resets on immediate modification #16976
  - Fixed misleading edge parameter dialog flow descriptions #16981 (meso)
  - Fixed crash when running while having network parameter window open #17043 (meso)
  - Start button is disabled when loading simulation aborts with an error #17063
  - Fixed wrong entries / paths for recent config #16906
  - Settings dialog now cancels changes on ESC #17050
  - containerStop: custom container angle now applies to plan item `<stop>` #17089
  - Vehicles on edges that are shorter than their geometrical length are no longer exaggerated lengthwise #17074
  - Fixed crash when deleting last item of coloring/scaling scheme levels #17138 (also applies to netedit)
  - Rerouter ids are now shown #16715
  - JuPedSim obstacles are now shown at their configured z-coordinae in 3D-view #17246
  - Fixed crash after saving gui settings to registry under a new name #17265

- meso
  - edgeData with `withInternal="true"` no longer contains internal edges #17046 (regression in 1.6.0)
  - Fixed error when loading state with high event times #16936
  - Fixed invalid edgeData output with `aggregated="true"` #16982
  - Fixed invalid speed in edgeData when using a [speed calibrator](Simulation/Calibrator.md#calibrating_only_speed) #17228

- duarouter
  - Fixed crash when loading taz and setting option **--persontrip.transfer.walk-taxi allJunctions** #17180
  - Fixed inconsistent default personTrip arrivalPos #17177
  - Fixed bug where intermodal routing failed when restricting taxi arrivals and the destination edge has a stopping place #17178 (also applies to sumo)
  - Fixed invalid error when using option **--bulk-routing** #17267
  - Option **--bulk-routing** now works for railways #17266
  - Option **--ignore-errors** can now filter out loaded routes that violate edge permissions #17294
  - option **--keep-vtype-distributions** ist now working for flows #17305

- TraCI
  - Fixed exaggerated slowDown after the end of the desired slowDown duration #17172 (regression in 1.23.0)

- tools
  - abstractRail.py: fixed failure to use all loaded stops #17023
  - abstractRail.py: fixed bug where option **--horizontal** sometimes didn't work #17025
  - abstractRail.py: now gracefully handles stop input with invalid startPos or endPos #17027
  - abstractRail.py: corrected naming of temporary net when using .net.xml.gz input with option **--split** #17029
  - routeSampler.py: mismatch-output for tazRelations is now also written as tazRelations #17049
  - osmWebWizard.py: no longer fails to start a second instance #16663
  - netdiff.py: Fixed plain-output file names when running diff on .net.xml.gz files #17319

### Enhancements

- sumo
  - Option **--max-num-persons 0** can now be used to run a simulation without persons. #16965
  - Additional files now support element `<preference>` which can be used to [influence routing preference for different vClasses and vTypes](Simulation/Routing.md#routing_by_travel_time_and_routingtype) #9091
  - Added options **--device.rerouting.threshold.factor** and **--device.rerouting.threshold.constant** to avoid routing if the [improvement in travel time is too low](Demand/Automatic_Routing.md#improvement_thresholds) #4635
  - The junction model parameter `jmDriveRedSpeed` now takes effect when emergency vehicles with the bluelight device pass an intersection on red #17120
  - Vehicles with the bluelight device can now pass a junction from the wrong turn lane without first reducing their speed to 0 #17123
  - Vehicles that are driving outside their edge now permit increased lateral acceleration and lateral speed to return to the road #17131
  - Added option **--person-fcd-output** to separate the outputs of vehicles and persons. This also leads to cleanar tabular outputs (csv, parqet) #16814
  - Actuated tls with custom conditions now supports expression `p:TLINDEX` to retrieve the number of pedestrions on approach to a crossing #17229
  - All insertion warnings/errors now report the failure time #17259
  - summary-output now includes the number of discarded vehicles #17282
  - When option **--summary-output.period** is set, summary-output will always include the final simulation step #17283
  - Option **--weights.reversal-penalty** can now be used to configure a penalty for train reversal when routing. A negative value disables reversals #17269
  - edgeData output now includes attribute 'flow' #17284

- netedit
  - Now translating additional tooltips #12652
  - The overwrite-elements dialog can now remember the user choice #17041
  - Gui setting 'show route index' now works for inspected routes and vehicles #17013
  - Gui setting 'show stop info' now work for inspected routes and vehicles #17014
  - tazRelation width can now be scaled by attribute #17136
  - Now using allow/disallow dialog for rerouter closingReroute and closingLaneReroute #5318
  - Data coloring (and scaling) settings can now be saved and loaded #17137
  - Implemented Dialog for a subset of crashes to simplify reporting from end user to developer. #12006
  - Saving files now supports 'apply to all' #17143
  - Settings dialog now shows available POI parameters for the 'show poi text param' feature #17158
  - meanData mode now permits setting edgeData type (i.e. to create edge-emission-output) #15696
  - Embedded routes are now hidden when creating vehicle or flow that references a route #17220
  - POIs can be transformed between network coordinates and geo-coordinates #17118

- sumo-gui
  - Various dialogs can now be closed with ESC #15463
  - edge/lane parameter dialog now shows routingType (which defaults to the edgeType when not set explicitly) #17096
  - Added menu option for opening only the network in netedit when a *.sumocfg* has been loaded in the simulation ( <kbd>Ctrl+Shift+T</kbd>) #17087
  - BusStop and all other stopping places now support `<param key="waitingDepth" value="FLOAT"/>` to customize spacing depth of waiting transportables #17088
  - Gui-settings files now permit [opening a tls tracker on startup](sumo-gui.md#tls_tracking) #17236


- duarouter
  - Added option **--keep-flows** to avoid expanding flows into individual vehicles #2407
  - Additional files now support element `<preference>` which can be used to [influence routing preference for different vClasses and vTypes](Simulation/Routing.md#routing_by_travel_time_and_routingtype) (also applies to other routers and routing in sumo) #9091
  - Option **--weights.reversal-penalty** can now be used to configure a penalty for train reversal when routing. A negative value disables reversals #17269

- netconvert
  - Added option **--output.removed-nodes** which preserves ids of nodes that were removing during simplification withh **--geometry.remove** #16937
  - Added option **--junctions.attach-removed** which can be used to merge networks that were processed with option **--output.removed-nodes** and correctly re-attach at removed junctions #16968
  - Option **--junctions.join-same** now supports setting the matching distance for joining #16969
  - Added new edge attribute `routingType` which can be used to influence routing together with the new `<preference>` element #17094
  - Options **--railway.topology.extend-priority** and **--railway.topology.direction-priority** now set attribute `routingType` #17102
  - Element `<split>` now supports attribute `offset` to customize the lateral offset of newly created lanes #17103
  - Node attribute `roundabout="0"` can now be used to declare that a junction shall not be classified as a roundabout #10677
  - Invalid lane neighbor information with respect to connected junctions is now fixed automatically (with a warning) #17280

- TraCI
  - `vehicle.setSpeedMode` now takes effect for vehicles with the bluelight device #17122

- tools
  - net2geojson.py: now permits exporting of edge AND lane shapes by using options **--edges --lanes** #16774
  - abstractRail.py: Added option **--main-stops** for filtering stops when determining region angle #17024
  - generateDetectors.py: Added option **--edge-probability** to allow randomization per edge #17044
  - instantOutToEdgeCounts.py: new tool to convert induction loop output to edgeData #17048
  - createOvertakingReroutes.py: new tool for building railway rerouters #16448
  - tazRel2POI.py: [new tool for visualizing taz relations](Tools/District.md#tazrel2poipy) #17157
  - randomTrips.py: options **--flows** and **--route-file** now writes flows and runs much faster #17200, #17204


### Miscellaneous

- Option **--junctions.join-same** whas changed from type *BOOL* to type *FLOAT* and now requires a distance argument. The previous behavior can be approximated by setting a value of *0.01* #16969
- sumo now warns when setting option **--threads** with an argument greater than 1 #17057
- Added warnings and errors for different problems with a configuration file #17069
- Command line output under Windows now uses UTF charset #17093
- [Automatic rerouting](Demand/Automatic_Routing.md) will no longer take place if the new route has the same travel time as the old route. #4635
- Vehicles with the bluelight device no longer set prefered lateral alignment to `arbitrary` #17124
- duaIterate.py: option **--binary** was removed #16777
- The attribute order in the FCD output of persons was changed and is now similar to the vehicles (affects only the type attribute).
- The nightly sumogame now loads all 3D objects #3032
- flowrouter.py option **--vclass** now defaults to 'passenger' (to avoid invalid routes in multimodal networks) #17296
- duaIterate.py: option **--gzip** is enabled by default. Option **--no-gzip** may be used for the old behavior. #16777


## Version 1.24.0 (22.07.2025)

### Bugfixes

- sumo
  - Fixed invalid traffic light warnings #16602
  - Fixed overly verbose router output when using stationfinder device #16624
  - Vehicles no longer drive onto forbidden internal lanes when option **--ignore-route-errors** is set #16635
  - Fixed bug where junction collision was not detected #16695
  - Fixed collision on junction due to unsafe lane changing #16643
  - Fixed invalid waitingTime for walking stage in tripinfo-output (value was not accumulating) #16729
  - railway routing now ignores temporary road closings when option **--device.rerouting.mode 8** is set #16799
  - Fixed bug where persons with a personTrip had the wrong arrivalPos when changing from `<ride>` to `<walk>` #16801
  - Fixed invalid emergency stop when vehicle is teleported beyond `arrivalEdge` attribute #16802
  - Fixed bug where parking outflow would be blocked due to numerical issues. #16809
  - Fixed invalid rail signal state when departing before crossed tracks #16819
  - Fixed inconsistencies in emission value output. The option **--emission-output.precision** is now being applied also to [tripinfo](Simulation/Output/TripInfo.md) and [edgedata](Simulation/Output/Lane-_or_Edge-based_Emissions_Measures.md). #16832
  - Fixed routing of pedestrians across multiple traffic light controlled crossings within an intersection #16788
  - Fixed bug where routing ignores option **--device.rerouting.mode 8** when using rerouter and trips #16851
  - Fixed emergency braking on approach to lane end #16779
  - Fixed invalid speed when approaching internal junction with option **--no-internal-links** #16857
  - Fixed invalid computation of follower vehicles when using **--no-internal-links** #16858
  - Fixed prolonged failure to change lane on short edge #1403, #16780
  - Fixed train collisions in *moving block mode* on switches #16855
  - Fixed crash when loading a network with invalid opposite-direction driving data #16904
  - Fixed various bugs that impact replication from loaded state #16765
    - Timing of rerouting events is now preserved when loading state #16772
    - Previously recorded travel speeds are now preserved when loading state #16775
    - State-saving now preserves loading/insertion order for vehicles that depart in the same step #16870
    - Probabilistic device assignment no longer differs when loading state #16784, #16871
    - Behavior no longer differs after loading state when using option **--weights.random-factor** #16876
    - Fixed differing behavior after loading state with option **--meso-overtaking** #16874
    - Queue entry block time is now preserved when saving/loading state #16770
    - edgeData now restores internal state when loading state #16894
    - Option **--save-state.precision** now applies to all values stored in state #16897


- netedit
  - link-direction arrows for spread bidi-rail are drawn in the correct spot #16718 (regression in 1.20.0)
  - bidi-rail connections are drawn large enough for comfortable clicking #16701 (regression in 1.22.0)
  - bidi-rail connections are drawn on the correct side again #16700 (regression in 1.23.0)
  - Fixed Crash transforming flows between TAZs #16859 (regression 1.23.0)
  - Rerouter intervals can be defined again #16796 (regression in 1.23.0)
  - RouteProbe and E3 detectors now properly saves the output filename #16910, #16913 (regression in 1.23.0)
  - Open-network dialog now list net.xml.gz files again #16915 (regression in 1.23.0)
  - Fixed broken functionality when switching language to "Deutsch" #16926 (regression in 1.23.0)
  - Changing connection attribute 'uncontrolled' to `False` and a traffic light, now makes that connection controlled by the traffic light #16705
  - Fixed crash after using tls-mode "reset single" when loaded programs had non-standard programIDs. #16702
  - parkingArea reference in chargingStation is now loaded #16789
  - Fixed bug where saving .sumocfg could overwrite demand when started from sumo-gui with ctrl+t #16711
  - Background images with embedded WGS84 geo-projection data are now correctly imported (UTM as fallback) #16895
  - Fixed crash when trying to handle invalid vehicles during saving #16860

- sumo-gui
  - Fixed bug where the trailer was occasionally not drawn for guiShape truck/semitrailer #16808 (regression in 1.23.0)
  - Fixed bug where articulated trucks (i.e. guiShape=`truck/semitrailer`) had an exaggerated gap when zoomed out #16816
  - carriageImages are now loaded relative to the file in which they are defined #16725
  - game mode rail switches now always switch on the first click #16726
  - Fixed UI freeze when loading edgeData for unknown edges on windows #16742
  - Fixed various parameter window quirks #11033
  - Fixed crash in "About Dialog" if SUMO_HOME is not set #16749
  - Fixed orientation of parking lots to be in line with the documentation #16593
  - Fixed rendering glitches for train carriages #16761
  - Fixed crash when running with options **--device.rerouting.threads** and **--weights.random-factor** #16878
  - Background images with embedded UTM geo-projection data are now imported (UTM as fallback to WGS84) #16895

- mesosim
  - Fixed crash when loading state with different network #16758
  - Fixed crash when loading state with different values of **--meso-lane-queue** #16757
  - Vehroute-output no longer contains invalid route edges when loading state and rerouting #16776
  - Fixed bug where calibrators caused invalid traffic data output when removing vehicles #16821
  - Fixed state-saving related bugs #16770, #16874

- netconvert
  - Fixed invalid right of way at left_before_right junction at specific angles #16793 (regression in 1.23.0)
  - Fixed bug where option **--tls.rebuild** creates invalid signal plan when using custom crossing traffic light indices. #16653
  - Fixed unsafe signal plan when crossings use linkIndex2 #16657
  - Fixed missing yellow phase when crossing re-uses vehicular link index #16658
  - OSM import: fixed low default speed for edge type *highway.service* when used by public transport #16763
  - Fixed bug where small roundabouts were sometimes not detected #16787
  - Fixed inconsistent opposite-driving information in network #16905
  - Fixed invalid connection when defining split with `type="zipper"` #16923

- duarouter
  - Fixed invalid warning about actionStepLength #16899

- TraCI
  - Fixed bug where vehicles would not change on their current lane after modifying permissions #16721

- tools
  - gtfs2pt.py: Fixed bug where option **--repair** did not fix broken **--osm-routes**. #16632 (regression in 1.17.0)
  - netduplicate.py: tool is working again #16931 (regression in 1.17.0)
  - createVehTypeDistribution.py: no longer crashes when output file already exists #16728 (regression in 1.21.0)
  - gtfs2pt.py: Fixed invalid vehicle departure when running with options **--osm-routes --bbox** #16731
  - gtfs2pt.py: Fixed obsolete config header when using **--osm-routes** #16680
  - osmWebWizard.py: Fixed bug where the wizard wouldn't open properly on Linux #16086
  - net2geojson.py: Fixed bug that was causing invalid shapes with option **--boundary** #16295
  - tlsCycleAdaptation.py: Fixed invalid error #14015
  - scaleTimeLine.py: Fixed invalid sorting of output #16744
  - sumolib.net: No longer ignores connection permissions #16633
  - sumolib.xml.toXML: custom indent is now passed on #16734
  - generateRerouters.py: fixed missing character in XML output #16815
  - traceExporter.py: Fixed inconsistent coordinates when setting option **--shift** #16825
  - osmWebWizard.py: Calling build.bat now ensures uncorrelated results between traffic modes and reproduction of the initial traffic #8660

### Enhancements

- sumo
  - Files ending with *.parquet* are now written in [Apache Parquet format](https://en.wikipedia.org/wiki/Apache_Parquet). This can also be activated with option **--output.format parquet**. Further applicable options are **--output.compression**, **--output.column-header**. Feature is in [experimental stage](TabularOutputs.md#limitations) #14694
  - Files ending with *.csv* are now written in [CSV format](https://en.wikipedia.org/wiki/Comma-separated_values). This can also be activated with option **--output.format csv**. Compressed csv is supported with *.csv.gz*. Further applicable options are **--output.column-header**, **--output.column-separator**. Feature is in [experimental stage](TabularOutputs.md#limitations) #16791
  - A warning is now issued for traffic light programs if a link never gets a green phase even when the program has only a single phase. #16652
  - A warning is now issued for traffic light programs if two 'G' links target the same lane in the same phase #16636
  - Added waitingTime to personinfo walk output #16737
  - closingReroute now supports optional attribute `until` for setting the estimated duration of closing. Vehicle may wait at the closed edge if a detour would take longer. #16804
  - Vehicles may now exceed their vType-`maxSpeed` when using `carFollowModel="KraussPS"` and going downhill (up to their `desiredMaxSpeed`) #16805
  - containerStop now supports element `<access>` #16811
  - Vehroute output for persons and containers can now selectively be disabled via param key `"has.vehroute.person-device"`  #16820
  - It is now possible to model [trailers / marshalling](Specification/Logistics.md#trailers_and_rail_cars) by changing vehicle properties upon loading/unloading of containers #8800
  - Custom traffic light switching rules now support function `w:DETID` to retrieve the longest individual waiting time in seconds for vehicles on detector #16841
  - Rerouting trains now always use the current edge as reroute-origin #16852
  - Added options **--default.departspeed** and **--default.departlane** to override the default value when vehicles do not define the respective attributes #16925

- meso
  - `<tlLogic>` with `<param key="meso.tls.control" value="true"/>` is now exempt from options **--meso-tls-penalty** and **--meso-tls-flow-penalty** and runs with full control #16674

- sumo-gui
  - Various improvement to visual placement of rerouter objects #16716
  - In game mode, left-click can now toggle switches (rerouters) #16840
  - The 3D-view can now render vehicles in their 3D-model color #12683
  - The 3D-view now shows POIs and polygons #13991
  - busStop now support `<param key="waitingWidth" value="FLOAT"/>` to customize spacing of waiting transportables #16724
  - ChargingStations that have an associated parkingArea are now drawn at the parking space rather than on the road #16670
  - containerStop now supports attribute `angle` to draw stationed containers at an angle #16813
  - vTypes now support `<param key="scaleVisual" value="FLOAT"/>` to customize individual visual scaling #16849

- netconvert
  - Added options **--prefix.junction** an **--prefix.edge** to set independent prefixes for the respective objects #16900
  - Added option **--kept-ids** to load a selection of objects that shall not be renamed (with **--numerical-ids** or a **--prefix** option) #16902


- duarouter
  - Added option **--repair.max-detour-factor** to give more control over repairing routes. Vehicles will backtrack rather than take large detours. #16746

- tools
  - randomTrips.py now always generates validated trips #12684
  - plotXMLAttributes.py: The options **--xticks-file** and **--yticks-file** now support giving a column for name aliases to group the respective values #16683
  - [plotStops.py](Tools/Railways.md#plotstopspy): New tool to simplify drawing a train schedule diagram along a specified route. #16683
  - generateContinuousRerouters.py: Added option **--stop-file** to add stops at loaded busStops when rerouting #16719
  - mapDetectors.py: Added option **--all-lanes** to place detectors on all lanes of an edge based on a single input coordinate #16751
  - plot_net_dump.py: Added option **--colormap.center** two permit plotting colors with `TwoSlopeNorm` #16778
  - tileGet.py:: Added 'cartodb_' URL-shortcuts and option **--retina** to be used with cartodb URLS #16822
  - net2sel.py: Added new tool to obtain selection file with junctions and edges from .net.xml #16907
  - routeSampler.py: Named route output now always writes the usage count as route "probability" attribute #16919
  - route2poly.py: when using **--standalone** with **--scale-width**, named routes are scaled by attribute probability  #16920
  - route2poly.py: when using option **--scale-width**, routes are sorted by count/probability in descending order. Option **--filter-count** can be used to exlude routes with low count #16921


### Miscellaneous

- sumo-gui: swapped color semantics of stopping place occupancy indicator (red means used and green now means empty) #16668
- Shift-click no longer switches traffic lights or starts tracking vehicles in gaming mode #16703, #16704
- Added railway game #13446
- setting the python root dir now compiles libsumo against the selected python #16755
- It is recommended to use a compiler which supports C++17 (e.g. MSVC 2017 or later, g++ 7 or later)
  - MSVC 2015 is no longer supported (mainly due to updates in SUMOLibraries)
- netconvert: OSM-imported edges with type "highway.service" now have type "highway.service|psv" when used by public transport vehicles. This may require adaptations when using option **--keep-edges.by-type** or **--remove-edges.by-type** and when using edge-type specific [restrictions](Networks/PlainXML.md#vehicle-class_specific_speed_limits) or [meso-settings](Simulation/Meso.md#configuration_by_edge_type)  #16763
- The values of **--default.departspeed** (currently "0") and **--default.departlane** (currently "first") will be changed in a future version so that vehicles depart with higher speed and on all lanes. Make sure to set explicit values if your simulation relies on the old behavior.

## Version 1.23.1 (08.05.2025)

### Bugfixes

- sumo
  - Fixed crash when person jumps between stops #16641
  - Fixed invalid arrivalPos when person jumps between stops #16642
  - Fixed crash when using `overtakeReroute` #16647

- netedit
  - Short connections are visible again when activating 'show connections' #16644 (regression in 1.22.0)
  - Fixed crash setting a negative time threshold in E2 detectors
  - Fixed crash when using new selection mode features #16648
  - Fixed crash when placing two vaporizers on the same edge #16649

- Tools
  - gtfs2pt.py: option **--write-terminals** now works when setting **--osm-routes** #16645
  - gtfs2pt.py: fixed crash when loading **--osm-routes** with smaller network #16646


## Version 1.23.0 (06.05.2025)

### Bugfixes

- sumo
  - Fixed invalid stop position at busStop when a vehicle became blocked while the stop was occupied #16270 (regression in 1.6.0)
  - Fixed crash when using rail signal on shared lane #16219 (regression in 1.22.0)
  - Fixed invalid waitingTime in personinfo output #16164
  - Fixed failure to overtake two parallel stopped leaders #16232
  - Fixed invalid error when departing with fixed speed and large offset from best lane #16244
  - Taxi no longer disappear via teleporting in a jam. They will never teleport while they are on their final drop-off edge. #16268
  - Fixed bug where taxi fails to drive after picking up passenger #16271
  - Verbose output now accounts for discarded persons #16261
  - Verbose output now prints correct number of running persons when setting option **--tripinfo-output.write-unfinished** #16281
  - The number of loaded vehicles is no longer impacted by temporary vehicles used during intermodal routing #16259
  - Fixed oscillating speed during zipper merging #16146
  - Fixed invalid bestLaneOffset when an edge has usable lanes divided by forbidden lanes. This could result in invalid departure errors #16287
  - Fixed crash after lane change with E3 detector using `detectPersons="car"` #16296
  - Ensured simulation terminates when using LC2013 with sublane model #16297
  - Fixed crash when using option **--device.rerouting.threads** with option **--weights.random-factor** and rerouters #16347
  - Fixed collision due to unsafe lane changing ahead of a zipper merge #16305
  - Fixed duplicate collision output when using **--collision-action warn** #16397
  - Fixed misclassification of some frontal-collisions as normal (rear-end) collision #16398
  - Fixed invalid formula for coasting deceleration in case of steep inclines #16309
  - Fixed invalid rail behavior after rerouting #16431
  - Fixed blocked lane changes on junction #16443
  - Fixed deadlock between vehicle and pedestrian on walkingarea #16458
  - The electric vehicle model now honors engine shutoff via params `shutOffStopDuration` and `shutOffAutoDuration` #16341
  - JuPedSim-pedestrians now take into account traffic lights when walking against the direction of the edge. #16313
  - SSM device: fixed unhelpful error message on invalid `device.ssm.exclude-conflict-types` #16511
  - SSM device: fixed problems with indirect left turn #16519
  - Fixed bug where right-turning vehicle ignores bicycle on indirect left turn #16520
  - Option **--ignore-junction-blocker** no longer triggers junction collisions #16525
  - Stops in flows are now correctly handled when saving and loading state #16527
  - Fixed invalid behavior when rerouter closes multiple lanes or edges with different permissions in the same interval #13846
  - Fixed rare crash on loading rail simulation with internal links #16532
  - Fixed insufficient precision when using **--fcd-replay** with JuPedSim #16047
  - Free insertion now takes into account a stop on the insertion edge #16592
  - Insertion after jump no longer skips stop on the insertion edge #16583
  - Fixed treatment of stationfinder device parameter `checkEnergyForRoute` #16380
  - Fixed bug where stationfinder device fails to search for station after failed estimation at low charge #16562
  - Fixed bug where electric vehicles with stationfinder device only recharge once #16597
  - stationFinder/parkingReroute: Fixed invalid estimate of time to reach new stopping place #16603
  - StationFinder: fixed bug where charging only worked once #16597
  - Fixed crash when using calibrator in a simulation beyond step 86400 #16626



- netedit
  - Restored functionality for setting custom geometry point by entering values #16179 (regression in 1.20.0)
  - Improved visibility of internal junction markers on connections #16485 (regression in 1.20.0)
  - Move mode with elevation checkbox active now shows elevation numbers again when zoomed out #16236 (regression in 1.20.0)
  - Fixed crash in tls mode #16608 (regression in 1.20.0)
  - Fixed missing edge colors when defining person walk #16461 (regression in 1.21.0)
  - Fixed overly large endpoint markers in move mode #16266 (regression in 1.22.0)
  - Fixed crash when TL controlled junction overlaps with another uncontrolled one #16483 (regression in 1.22.0)
  - Fixed bug where shift-click to inspect lane only works after moving mouse #16134 (regression in 1.22.0)
  - Fixed invalid connections after using *reset connections* #16127
  - Saving demand that was loaded from a sumocfg in multiple route files is now working #14805
  - Fixed handling of special vType params for visualizing rail carriages #16334
  - Fixed invalid geometry of person plans that end in a stopLane #15355
  - Fixed invalid geometry of person plan from stoppingPlace to stoppingPlace  #15348
  - Fixed crash after deleting edges with opposite-lane information #16500
  - Fixed superfluous default attributes when saving meanData #16427
  - Selection coloring is instantly updated after selecting lanes with button functions (dead end, dead start, ...) in connection mode #16464


- sumo-gui
  - Fixed rendering of rail carriages when scaled by length/geometry #16425
  - Visualization option *scale length with geometry* now works for rail carriages and allows rendering with unscaled length #11576
  - Fixed crash on invalid output file path for calibrators #16545
  - vehicle color param and vehicle text param are now correctly saved in settings #16561

- netconvert
  - Fixed unsafe program transition from 'G' to 'g' #16289 (regression in 1.20.0)
  - Fixed invalid link state when straight connection passes a crossing after internal junction #16167
  - Option **--bikelanes.guess** no longer impacts loaded connection attributes #16196
  - Fixed invalid right of way rules / junction type when shared tram lanes crosses another tram line #16215
  - Fixed bug where stops loaded with option **--ptstop-files** were renamed #16220
  - Fixed failure to map elevation to a network with z-data #16235
  - OpenDRIVE import: Fixed invalid simplification of elevation data on straight roads #16566
  - Fixed invalid connections when using option **--osm.turn-lanes** #13586
  - Fixed invalid bus connections when guessing #16291
  - Fixed invalid right-of-way rules at traffic lights with uncommon phase layouts. #16338
  - Fixed missing signalID params in OpenDRIVE import #16403
  - Fixed crash when using option **--join-tram-dist** #16393
  - Fixed bug where option **--edges.join-tram-dist** didn't join enough #16408
  - Fixed `nan` value in generated network when loading connections with custom length value and length-0 geometry #16441
  - Fixed invalid right of way with respect to left-turns from the oncoming direction at junction type `left_before_right` #16480
  - Fixed invalid internal lane shape when importing OpenDRIVE #16482
  - Fixed invalid ptline-output when running with option **--ptstop-output.no-bidi** #16534
  - Option **--ptline-clean-up** now cleans up more stops #16540
  - Fixed failure to join junctions #16557
  - Fixed bug where a loaded network successively changed it's geometry #16609

- durarouter
  - Fixed invalid route output when loading invalid routes with stops and setting option **--ignore-errors** #16365
  - Improved error message when loading **--astar.landmark-distances** together with taz #16400
  - Fixed crash when loading restriction-params with taz #16514
  - vClass-specific speed restrictions are now working #16580

- TraCI
  - Setting vehicle parameter 'lcContRight' is now working #16147
  - Fixed bug where `traci.vehicle.moveToXY` matched onto the wrong internal lane #15282
  - Fixed bug where `vehicle.slowDown` require and additional simulation step to reach the target speed and `vehicle.setAcceleration` was achieving the wrong acceleration #7361
  - Fixed invalid active person count after person removal #16192
  - `person.removeStage` now works for a taxi-driving stage if the taxi is already en route to pick-up #11620
  - Subscription to `lane.getAngle` is now working #16360
  - Subscriptions to methods with additional parameters now work in libsumo #16383
  - Function `edge.subscribeContext` now correctly collects vehicles at low dist regardless of lane number #16422
  - Function `vehicle.getLeader` no longer contains traffic that crosses the path of the ego vehicle (without ever becoming a leader) #13842
  - libsumo subscriptions no longer ignore begin and end time #16411
  - After `vehicle.setType`, saveState now saves the new type #16185

- Tools
  - `sumolib.net.lane.getClosestLanePosAndDist` now gives correct results when lane length differs from shape length #16269
  - `sumolib.xml` now properly escapes special characters when generating xml #16318
  - gtfs2pt.py: Fixed problem that caused invalid routes to be written #16336
  - gtfs2pt.py: Fixed invalid stop placement on disallowed lane #16352
  - gtfs2pt.py: Now warning about input that provokes negative stop-until times #16322
  - gtfs2pt.py: Fixed exaggerated penalty for alternative departure edge #16543
  - gtfs2pt.py: edge speed is no longer ignored when mapping #16544
  - route2sel.py: Fixed crash when loading flow/trip that references a route id #16395
  - randomTrips.py: Fixed inconsistent behavior of option **--verbose** #11861
  - randomTrips.py: Fixed override of **--edge-permission** when using **--persontrips** #16471
  - randomTrips.py: can now load custom boolean duarouter option from config #16551
  - randomTrips.py: writing .rou.xml.gz is now working #16556
  - scaleRoutes.py: Fixed bug in scaling #16474
  - scaleRoutes.py: Fixed crashes when input exceeds the configured time range #16467
  - scaleRoutes.py: option **--timeline-pair** is now working #16473
  - scaleRoutes.py: Now works with flows defined via `period` #16470
  - route_1htoDay.py: fixed misleading option help text #16466
  - route_1htoDay.py: now supports option **--output-file** to put all vehicles into a single file that can be used with **scaleTimeLine.py** #16468
  - analyzePersonPlans.py: fixed invalid car use classification #16549
  - distributeChargingStations.py: Fixed invalid position of generated parking area when input contains negative positions #16560
  - ptlines2flows.py: Fixed invalid route with **--extend-to-fringe** when pt line is split #16573
  - ptlines2flows.py: No longer writing invalid (disconnected) routes when using a modified input network. Instead disconnected parts are bridged with jumps #16292
  - loading tool config with multiple positional input file arguments now works #16447

### Enhancements

- sumo
  - `departPos=stop` now works for any departLane on the stop edge #16133
  - Pedestrians are now included in full output #16159
  - Added stop attribute `jumpUntil` to set a minimum time for the end of a jump #16153
  - Tram insertion automatically uses moving-block mode when there are no rail signals for tram in the network. The behavior can be configured with option **--railsignal.default-classes** #16208
  - Taxis that transfer passengers at a busStop now register at that stop. This shows up in **--stop-output**. #16263
  - Road slope is now used in HBEFA3 and HBEFA4 computation #16307
  - Rerouting device now supports `<param key="ignoreDest" value="1"/>` to support rerouting to any parkingArea regardless of visibility or occupancy of the current destination. One use case is configuring the [idle-algorithm for finding the next taxi stand](Simulation/Taxi.md#idle_behavior) #16387
  - The **--full-output** now includes vehicle road slope (in degrees) in networks with elevation data #16389
  - Collisions that happen as the direct result of lane-changing are now distinguished as "side"-collisions in errors and **--collision-output** #16396
  - The warning "bus stop too short" no longer occurs if a stop fills the whole length of it's lane or if it's `parkingLength` is set to a sufficiently high value #16391
  - Zipper junctions with an arbitrary number of conflicting connections are now supported #11874
  - Electric vehicle model parameter `constantPowerIntake` is no longer affected by `propulsionEfficiency` or `recuperationEfficiency` #16463
  - vType attribute `jmTimegapMinor` may now be negative to provoke collisions #16478
  - person plan element `<stop>` now supports attribute `jump`. When this is set a person may jump between subsequent stops at different locations, taking the configured time to do so #6325
  - Signal plans where a phase loops back onto itself with no alternative next phase now raise a warning #16487
  - Emergency vehicles may now perform opposite overtaking of queues even when the downstream edge has no opposite edge #16499
  - collision-output now writes network coordinates of the front and rear of the involved vehicles #16509
  - SSM device parameter "write-na" can be used to disable conflict information where all values are `n/a` #16513
  - public transport rides that are created for a personTrip now use any vehicle that stops at the destination by default. The previous behavior of restricting rides to a single line id can be restored with option **--persontrip.ride-public-line**. #12263
  - All carFollowModels now support attributes `speedTable`, `maxAccelProfile` and `desAccelProfile` to model speed-dependant limits on acceleration #3920

- netedit
  - Each object now tracks the file from which it was loaded to facilitate working with projects where multiple route- or additional-files are used #12430
  - Now drawing cyan dotted contour around object with open popup menu #16344
  - In delete mode, hovering over a junction will highlight all edges that would be deleted when deleting the junction #14114
  - Selection of rerouter edges now uses improved highlight and permits toggling of edges #16430
  - Disabled superfluous attribute stopOffsetException when inspecting lanes where stopOffset is 0 #13985
  - Menu-toggle 'show bubbles' is now available in additional editing modes and toggles between showing junction shapes and junction bubbles. #16214

- netconvert
  - Added option **--junctions.join.parallel-threshold DEGREES** to increase user control over joining junctions (with **--junctions.join**) #16140
  - Added option **--osm.annotate-defaults** to document whether speed and lane number were based on OSM data or typemap default values #16094
  - Trams now use safe and efficient zipper merging where possible when no tram rail signals are defined. Option **--railway.signal.permit-unsignalized** can be used to configure other vClasses that are subject to this behavior #16216
  - OSM: newer tagging schemes for on-street parking are now supported #16558
  - The visual configuration 'Show bubbles' make 'show bubbles' more useful #16214

- sumo-gui
  - started work on Japanese translation #16129
  - busStops now support param `emptyColor` to distinguish [virtual stopping places](Simulation/Public_Transport.md#virtual_stops) from normal busStops. #16260
  - Improved visibility of guishape `evehicle` (The large black part now takes on a darker version of the vehicle color) #16262
  - Color settings dialog now features 'recalibrate rainbow' controls for dynamic vehicle coloring schemes #16384

- od2trips
  - Added warning when a taz has no source or sinks #16112

- duarouter
  - public transport rides that are created for a personTrip now use any vehicle that stops at the destination by default. The previous behavior of restricting rides to a single line id can be restored with option **--persontrip.ride-public-line**. #12263

- TraCI
  - `vehicle.setSpeedMode` now supports a 7th bit to control adherence to the road speed limit #3811
  - `traci.start` now supports argument `traceGetters="print"`. When this is set, the generated traceFile will print the outputs of all *get* commands when replayed. #16156
  - Added function `traci.route.delete` #15452
  - `simulation.getMinExpectedNumber` now takes into account pending taxi reservations #16187
  - Addedd function `person.getWalkingDistance` #16197
  - Added `lanearea` functions `getIntervalMeanTimeLoss` and `getLastIntervalMeanTimeLoss` #16311
  - Added function `domainID` to all domains. This can be used as the target domain in `subscribeContext` #16418
  - subscriptions to all complex types should now work #15963 #15962 #15785 #7648

- Tools
  - randomTrips.py: When option **--validate** is set, the generated amount of vehicles is guaranteed (by replacing invalid trips with new valid trips) #8843
  - randomTrips.py: Using value of **--vehicle-class** as default for **--edge-permission** #16471
  - netcheck.py: Added option **--right-of-way** to find selected cases of faulty right-of-way rules (currently only on-ramps are checked) #16036
  - jtcrouter.py: Added option **--additional-files** which are passed to [jtrrouter](jtrrouter.md) #16191
  - csv2xml.py: Added option **--flat** to convert arbitrary csv files without a schema #16204
  - plotXMLAttributes.py: Added option **--xstr** and **--ystr** to force data interpretation as category #16205
  - sumolib: function `xml.parse` is now useful when loading nested elements and not providing argument `element_names`. The new attribute `outputLevel` (default 1) controls which elements to retrieve #16211
  - sumolib: Objects loaded with function `xml.parse` now preserve their attribute order when serialized with `toXML` #16254
  - [remap_additionals.py](Tools/Net.md#remap_additionalspy): convert infrastructure from one network to another network (which may differ in geometry, lanes and edge splits) #16206
  - [remap_renamed.py](Tools/Net.md#remap_renamedpy): convert route files and additional files after renaming network ids (i.e. with **--numerical-ids**) #16252
  - [remap_network.py](Tools/Net.md#remap_networkpy): New tool for finding correspondence between networks based on geometry #16409
  - ptlines2flows.py: now sorts written routes and flows by id #16222
  - ptlines2flows.py: now safely handles missing edges #16293
  - gtfs2pt.py: now supports option **--merged-csv** for loading transit schedule data from a single file and option **--merged-csv-output** for creating such a file from GTFS input. #16310
  - gtfs2pt.py: now supports option **--patched-stops** to customize stop-to-lane assignment for individual stops #10720
  - gtfs2pt.py: now supports option **--stops** for giving a list of candidate stop edges to guide mapping. This can can greatly improve running time #16326
  - gtfs2pt.py: improved running time through caching #15856
  - gtfs2pt.py: now supports option **--bus-parking** to make buses clear the road when stopping #16415
  - gtfs2pt.py: added option **--write-terminals** to include vehicle `<params>`s that describe the known terminal stops of the full route (even if only part of the route is used) #16154
  - net2geojson.py: option **--traffic-lights** can now be used to include the shapes of traffic signals #16419
  - scaleRoutes.py: now uses a non-constant default timeline #16469
  - [generateDetectors.py](Tools/Output.md#generatedetectorspy): new tool for generating detectors on arbitrary network edges #16523
  - osmWebWizard.py: No longer generating superfluous bidi-stops #16533
  - [checkReversals.py](Tools/Railways.md#checkreversalspy): new tool for counting train reversals in route file #16542
  - [mapDetectors.py](Tools/Detector.md#mapdetectorspy): now filters duplicates #16553
  - [mapDetectors.py](Tools/Detector.md#mapdetectorspy): Option **--write-params** can be used to import further data columns #16554
  - edgeDataFromFlow.py: Now supports custom column names with option **--id-column** and **--time-column** and custom interpretation of time values with option **--time-scale** #16555
  - generateTurnRatios.py: added option **--split-types** to create type-specific turn ration attributes #16579
  - analyzePersonPlans.py: now handles rides without line attribute #16617


### Miscellaneous

- In netedit, the default extension of edge data files is now *.xml, instead of *.dat.xml #16257
- Netedit: unified contextual menus for all elements #15314
- The new default emission model is now [HBEFA4](Models/Emissions/HBEFA4-based.md) #15950. Please note that this means a major reduction especially in the values for CO and HC at low speeds.
- mapDetectors.py: no longer searching up to range 1000 by default #16571
- person-ride attribute `lines="ANY"` is no longer written as this is the default value when loading a ride without `line` attribute. #12263
- TraCI
  - the new TraCI version is 22
  - all TraCI functions in the python client should now return tuples instead of lists #15949
    - affected functions are inductionloop.getVehicleData, lane.getLinks, trafficlight.getControlledLinks, trafficlight.getConstraints*, trafficlight.getNemaPhaseCalls, vehicle.getLaneChangeStatePretty, vehicle.getNextLinks
  - the unused function TL_EXTERNAL_STATE has been removed (it had only a server implementation, no known client used it)
  - the functions `simulation.getBusStop*` are deprecated in favor of the corresponding functions in the busstop domain #16433

## Version 1.22.0 (04.02.2025)

### Bugfixes

- sumo
  - Fixed computation of electricity consumption with respect to radialDragCoefficient #14896 (regression in 1.8.0)
  - Fixed platform specific taxi behavior related to pre-booking #15698 (regression in 1.20.0)
  - Fixed crash involving pedestrians at pedestrian crossing #15807 (regression in 1.21.0)
  - Fixed reduced simulation speed of pedestrians (~ factor 5) #15825 (regression in 1.21.0)
  - Fixed bug that caused parkingAreaReroute to assigns an invalid route #15960 (regression in 1.21.0)
  - Fixed rerouting error on the last route edge with a stop #15552
  - Fixed routing error on departure #15563
  - Fixed invalid warnings regarding inconsistent loaded stop times #15602
  - Fixed blocked vehicles at busStop with attribute parkingLength #15644
  - Fixed train collision at rail_crossing #15658
  - Fixed crash when route contains internal edge #15667
  - Fixed invalid error when using jump after stopping twice on the same edge #15635
  - Fixed invalid value of diagnostic param 'caccVehicleMode' when using carFollowModel *CACC* #15682
  - Fixed invalid speedErr computation for carFollowModel *CACC* #15683
  - Actuated traffic light that use the `next` attribute to skip phases now work correctly if the targeted green phase serves no traffic but leads to a subsequent phase which does #15716
  - Fixed invalid approach speed at allway_stop when setting lane-stopOffset #15449
  - Fixed crash when using stop with coordinates and option **--mapmatch.junctions** #15740
  - Fixed emergency braking at `allway_stop` in subsecond simulation #15728
  - Configuring number of doors now takes effect for JuPedSim #15562
  - Fixed collision between vehicles and pedestrians involving lanes of different widths #15770
  - Fixed train collision in network without rail signals #15797
  - Pedestrians no longer walk onto rail_crossing at yellow #15808
  - Fixed collision at pedestrian crossing #15837
  - Fixed bug where a collision between vehicles back and pedestrian on crossing was not detected #15847
  - Fixed unsafe driving at prioritized crossing #15848
  - Fixed invalid lanechanging state ahead of roundabout which could cause lane changing to fail in dense traffic #15854
  - Fixed lanechanging deadlock involving 3 vehicles #15857
  - Stops at chargingStation with attribute parkingArea will now always park #15815
  - Fixed lane-changing deadlock on junction #15887
  - Fixed inconsistent statistics on bike departDelay #13142
  - Fixed bug where vehicles with bluelight device slowed down before entering a rescue lane #12067
  - Fixed invalid right-of-way when turning right-on-red and there is a pedestrian crossing #15939
  - Fixed invalid gap in person movement when a ride without arrivalPos is followed by a stop #8468
  - Vehicles are no longer inserted on used pedestrian crossing #15843
  - Fixed crash when loading state for vehicles with random arrivalPos #13110
  - Fixed inconsistent arrivalPos when loading state #15961
  - Fixed invalid stopping after vehicle teleports beyond stopping place #15972
  - Fixed bug where a vehicle steals another vehicles parking spot #15976
  - Fixed bug where parking egress is blocked after lane change #15757, #10007
  - Fixed missing XML validation for parking badges #16005
  - Fixed undefined behavior when using options **--device.rerouting.threads** with **--weights.random-factor** #15994
  - Option **--edgedata-output** now works when setting option **--begin** #16079

- netedit
  - Fixed crash when moving a big selection #15132 (regression in 1.16.0)
  - Fixed crash when loading a more than 20k vehicles. #15680 (regression in 1.19.0)
  - Fixed crash when creating Calibrator flow #15906 (regression in 1.19.0)
  - Fixed candidate edge coloring in pedestrian mode #15888 (regression in 1.19.0)
  - E2 multilane detectors can be moved again #15551 (regression in 1.20.0)
  - Fixed invalid rectangle selection when zoomed out #15766 (regression in 1.20.0)
  - Traffic light mode shows traffic light icons again when zoomed out #15966 (regression in 1.20.0)
  - Fixed crash when loading network with crossings and overlapping edge geometry #16053 (regression in 1.21.0)
  - Loaded containers starting from stops are now drawn #15567
  - ESC aborts creation of edgeRel and tazRel datas #15601
  - Fixed invalid TAZ coloring during mouse hovering in create TAZRel mode #15544
  - Simplified creation of laneArea detectors on short lanes #15142
  - Vehicles placed in elevated lanes can now be moved #15367
  - Fixed invalid route shown for non-default departLane #15056
  - Improve geometry of embedded routes #13980
  - Fixed Crash when editing options before loading a network #13881
  - Fixed text rendering of small elements when zoomed out #15185
  - Selectable area of a POI now corresponds to custom radius #15532
  - Fixed crash after undo-redo due to distributions #15642
  - Fixed invalid behavior after pressing ESC key in person plan mode #15193
  - Modified default vType is no longer written after being reset #14985
  - data mode: create interval elements are disabled until a dataset has been selected #13309
  - "Show geometry point indices" option is now working #13969
  - Turnaround connections are now visible for bidi-rail edges #14486
  - Moving a junction now moves it's custom shape #15456
  - Fixed missing/strange attributes while adding routeprobe #15268
  - Can now load trips without from / to #15074
  - Fixed slowdown when loading large TAZ #15674
  - Significantly reduced UI freeze when switching to demand mode and many vehicles are loaded #15681
  - Fixed dotted contour of multilane objects (routes, trips, etc.) #15676
  - Fixed crash when undo removes a connection that is being inspected #15724
  - Fixed crash during undo-redo of edited network #15534
  - Fixed "freeze" when deleting a large number of elements while also inspecting a large number of elements #15725
  - Fixed "freeze" when selecting very many elements #15747
  - Fixed crash when loading taz with the same id as a junction-taz #15759
  - Fixed invalid use of taz information when coordinates are defined for a trip #15765
  - Fixed crash when using "split junction and reconnect" #15786
  - Attribute input by pasting with ctrl+V is now working #6018
  - Short edges are now clickable in crossing mode #15795
  - Delete mode now ignores walkingareas #15794
  - Move mode now ignores crossings #15410
  - Fixed invalid file creation when saving .sumocfg #15838, #15270
  - File are no longer written if the objects are unchanged #15271
  - Fixed invalid vType attribute style when inspecting a selection of vehicles #12719
  - Fixed invalid rendering in route creation mode for elevated networks #13992
  - Fixed crash when entering invalid programID in traffic traffic light mode #15491
  - Fixed crash reloading data files #15845
  - Fixed invalid handling of quotation marks in toolcfg #15899
  - Walkingareas no longer intercept clicks in in crossing mode #15916
  - Stop saving sumo/netedit config if a fix element dialog is opened #15918
  - Fixed Invalid behavior after loading demand elements with keep old enabled #15904
  - Dotted contour no longer hides flow label and vehicle stack label #15929
  - Fixed crash editing vClass in VType dialog #16008
  - Selection mode function 'select parents' now selects incoming and outgoing lane of selected connections #15968
  - Fixed invalid weights loading values of TAZ source/sinks #16037
  - Fixed "freezing" when handling very large TAZ #15844
  - Fixed bug when saving toolcfg where the tool uses positional arguments #16075
  - Reloading a demand file now raises warnings rather than errors #16027
  - Deleting one edge no longer removes crossings from other edges #16017
  - Fixed selecting TAZ edges with a small selection #16100
  - Command line options **--new -o OUTPUT_FILE** is now working together. #14989


- sumo-gui
  - Fixed framerate drop when zoomed in very far #15666
  - Fixed missing elements in settings dialog when switching to another viewing scheme while the dialog is open #15637
  - Hotkey ALT no longer has the effect of rendering all POIs and polygons at layer 0. Instead, the layer can be customized in the settings dialog #15558
  - Fixed invalid breakpoints when clicking time stamps on messages in meso #15780
  - Fixed invalid travel time when computing pedestrian reachability in a non-pedestrian network #15792
  - Fixed invalid travel times when computing pedestrian and bicycle reachability #15793
  - Saving and loading of meso edge scaling scheme is now working #15902
  - edgedata-file parsing no longer aborts after encountering a single non-numerical attribute #15903
  - The rainbow colorscheme is now properly stored in gui settings #16080

- netconvert
  - Fixed invalid extra connections from edge where input specifies "no connections" #15944 (regression in 1.2.0)
  - Fixed invalid sign of geo-coordinate offset in OpenDRIVE input and output #15624
  - Fixed bug where right-of-way rules could create deadlock at a traffic light #15150
  - Fixed bug when removing narrow lanes during import #15718
  - No longer generating invalid signal plan when giving invalid argument **--tls.green.time** #15719
  - Fixed invalid linkState for left turns from the major road at junction type `allway_stop` #15737
  - Fixed invalid tlLogic after processing net with **--keep-edges** #15798
  - No longer building bicycle left turns from a straight-only vehicle lane (starting from a left-turn lane instead) #15943
  - Fixed invalid handling of negative stop position in ptstop-files #12183
  - Option **--sidewalks.guess.by-permissions** no longer puts sidewalks onto paths that are shared between pedestrians and bicycles #16060
  - OSM: loading typemap *osmNetconvertPedestrians.typ.xml* no longer puts sidewalks on both sides of oneway streets (i.e. in the middle of a divided road). Legacy behavior can be restored with option **--osm.oneway-reverse-sidewalk** #16063
  - OSM: fixed missing vehicle lane where access=yes and busway:right=lane #16061
  - OSM: fixed missing stops in ptline-output #16101
  - OSM: fixed missing bus direction at oneway street with `oneway:psv=no` #16103

- duarouter
  - Fixed crash when using stop with coordinates and option **--mapmatch.junctions** #15740
  - Fixed invalid use of taz information when coordinates are defined for a trip #15768
  - Fixed invalid route in a network with connection permissions but no other permissions #15925
  - Fixed invalid precision with **--write-costs** #15938
  - Fixed crash when loading flows with particular combination of IDs #16091

- marouter
  - Fixed invalid route involving vClass-restricted connection #15883

- netgenerate
  - No longer hangs when setting option **--rand.connectivity 1** #16089

- meso
  - Fixed gridlocks in roundabouts #14129 (regression in 1.4.0)
  - Fixed invalid queue assignment for turning vehicles #16034 (regression in 1.7.0)
  - Fixed crash when using **--mapmatch.junctions** in a network with internal edges #15741
  - Fixed crash when using **--time-to-teleport.disconnected** #15751
  - Option **--time-to-teleport.disconnected** is now working when connections are missing #15777
  - Fixed crash when rendering vehicle at parkingArea #15956
  - Fixed missing waitingTime in edgeData output #16082


- activitygen
  - fixed crash when attribute is not set #15782

- TraCI
  - Fixed crash when calling `vehicle.getNextLinks` and `lane.getLinks` at junction type `allway_stop` or `priority_stop` #15603 (regression in 1.21.0)
  - Fixed invalid result by `trafficlight.getServedPersonCount` #15715
  - Fixed invalid result by `vehicle.couldChangeLane` #10739
  - Fixd invalid result by `trafficlight.getSpentDuration` after `setProgramLogic` #15753
  - Fixed non-functional libsumo windows wheels #15516
  - setEmissionClass now works with PHEMlight #15761
  - subscribing to complex types now works with the python API #15785
  - Concurrent access to libsumo now works #15967
  - moveToXY now works on parked vehicles #16010
  - When setting option **--keep-after-arrival**, vehicles that were affected by moveToXY while parking are no longer drawn after arrival. #16009
  - Fixed result of `simulaton.convertRoad` when adjacent lanes have different widths #16105
  - Fixed missing collision check between cars and remote-controlled pedestrian #16092

- Tools
  - matsim_importPlans.py: no longer writes unsorted trips with option **-vehicles-only** #15743
  - generateBidiDistricts.py: Option **--radius** now takes effect regardless of edge length #15758
  - countEdgeUsage.py: Fixed misleading warning message #15790
  - sumolib: Fixed invalid result by `net.getShortestPath(..., ignoreDirection=True)` #15789
  - Sumolib: Fixed crash in function `miscutils.getFlowNumber` #15799
  - sumolib: Can now set a new attribute in sumolib.xml element #15991
  - sumolib.xml: Fixed bug where parse_fast retrieves wrong attribute if one attribute is the end-suffix of another attribute #15901
  - randomTrips.py: option **--fringe-factor** now works in lefthand networks #15876
  - randomTrips.py: Options **--random-departpos** and **--random-arrivalpos** now take effect for persons #15946
  - routeSampler.py: fixed crash when loading negative counts #15908
  - gtfs2pt.py: Import now works when optional 'direction_id' is missing #15736
  - Empty strings can now be passed via tool config file #15499
  - tracemapper.py No longer creates routes with the same edge repeated over and over #15625
  - osmWebWizard.py: Dunkirk no longer gets imported with lefthand traffic #16059
  - Fixed saving of toolconfig for jtcrouter.py, plotFlows.py, signal_POIs_from_xodr.py, tripinfoByTAZ.py and tripinfoDiff.py #16085
  - osmGet.py: fixed missing public transport stops (also affects osmWebWizard.py) #16106



### Enhancements

- sumo
  - Added new [stationfinder device](Simulation/Stationfinder.md) which reroutes electric vehicles to a chargingStation depending on it's state of charge #9663, #15871, #15931, #15980
  - Added option **--chargingstations-output.aggregated.write-unfinished** to include still running charging process in charging station output #15677
  - The new vType attribute `lcContRight` can be used to configure lane choice at a lane split where all lanes have equal strategic value. #15579
  - Added option **--insertion-checks** to set global defaults for vehicle attribute `insertionChecks` #15149
  - Added option **--pedestrian.striping.jamfactor** to configure the speed of jammed pedestrians (default 0.25) #15610
  - GLOSA Device now looks several phases into the future and can also take queues into account #15614
  - Added new vType attributes `jmAdvance` and `jmExtraGap` to configure the behavior on junctions for crossing and merging streams of traffic #15654
  - Added new attribute `jmStopLineGapMinor` to set the distance from the stop line at non-prioritized links #15442
  - vType attriubte `jmStopLineGap` now applies to allway_stop #15448
  - Added new vType attribute `lcStrategicLookahead`  for configuring the lookahead distance when computing strategic best lanes #14718
  - Added new vType attribute `lcSpeedGainRemainTime` which controls the minimum time a vehicle can drive on the new lane after a tactical lane change (formerly hard-coded to 20s) #12109
  - Added new insertion behavior `departLane="best_prob"` to increase throughput on multi-lane roads #15661
  - Stationfinder device now supports state saving and loading #15607
  - Traffic lights now supports the special value `offset="begin"` which lets the logic start in cycle-second 0 regardless of simulation begin time #15248
  - Traffic lights of type `actuated` can now use the `next` attribute to switch into fixed-duration phases (and the corresponding lanes will obtain detectors to trigger the switch) #15714
  - Traffic lights of type `actuated` can now react to a pedestrian crossing #1746
  - personTrip now supports geo-coordinates #15739
  - Added option **--mapmatch.taz** which works similar to **--mapmatch.junctions** but uses arbitrary TAZ definitions #15748
  - Added warning if IDM internal stepping is configured too large #15836
  - Battery device now includes info about total energy consumption in tripinfo #15800
  - Pedestrian speeds are now affected by speed limits on crossings and walkingareas (starting with network version 1.20.0) #11527
  - Loaded route files are no logged in **--verbose** mode #13875
  - Option **--pedestrian.jupedsim.wkt** can now be used to export geometry data from JuPedSim #14436
  - railways
    - major rewrite of signal logic #7578
    - major improvement in railway simulation speed (simulation time reduced by ~50-75% depending on scenario size) #4379
    - Fixed various deadlocks #7493, #13262, #15474
    - The signal block information that is written by option **--railsignal-block-output** now includes [information about the foe relationships](Simulation/Railways.md#railsignal-block-output) between driveways / blocks #14991
    - The new option **--railsignal.max-block-length** can be used to customized the threshold for warnings / driveway truncation #15819
    - The new option **--railsignal-vehicle-output** can be used to record entry and exit times of vehicles into signal blocks #14990
    - The new option **--time-to-teleport.railsignal-deadlock** can be used to detect signal based deadlocks #15561
    - The new option **--time-to-teleport.remove-constraint** can be used to resolve detected deadlocks which are caused by a signal constraint by deactivating a responsible constraint #14543
    - The new option **--deadlock-output** can be use to log detected deadlocks and also their method of resolution
    - Logged deadlocks can be loaded as additional file to prevent them in a subsequent simulation #15569
    - A warning is given for unreasonable combinations of **--time-to-teleport** options #15816
  - rerouter support a new radius attribute #15426

- netedit
  - Additional output now writes chargingStation after parkingArea elements #15628
  - Removed "invert edges" from GNECrossingFrame #15129
  - Undo-redo functionality can now optionally be disabled to improve operational speed #15663
  - Undo-redo functionality can now be temporary disabled while loading a file to improve loading speed #15668
  - Improved visibility of short edges #15592
  - The route for inspected vehicles is now highlighted #15930
  - Added 62 python tools to the menu #16076


- sumo-gui
  - The value of SUMO_HOME is now shown in the *About Dialog* (also for netedit) #15218
  - The lane parameter dialog provide information on driveway/foes that prevent train insertion #15823
  - A selection file loaded with **--selection-file** will now cause vehicles, persons and containers to be selected as soon as they are loaded #5427, #14093
  - Improved layering of chargingStation and parkingArea #15826
  - Disabled 'secondary shape' controls if no alternative net is loaded #12653
  - edge color legend now shows the used attribute/key #16026

- netconvert
  - Added support for zipped shape files #15623
  - street-sign-output now sets the sign angle corresponding to road geometry #15671
  - Traffic lights now supports the special value `offset="begin"` which lets the logic start in cycle-second 0 regardless of simulation begin time #15248
  - Actuated pedestrian crossings are now actuated by pedestrians rather than vehicles #7637
  - Pedestrian crossings created by option **--crossings.guess** are now given priority. The old behavior can be obtained by setting option **--crossings.guess.roundabout-priority false** #15833
  - Option **--plain-output-prefix** now also saves a *.netccfg*-file for rebuilding the network from plain-xml files #12998
  - Improved geometry of pedestrian crossings when a footpath crosses a road #15975
  - Removed warning about sharp angles on service roads #16062
  - OSM: busways are now imported #16090

- meso
  - fcd-output can now be configured to include model attributes *segment, queue, entryTime, eventTime* and *blockTime* #15670
  - Vehicle attribute `insertionChecks="none"` can now force insertion #16096


- duarouter
  - The input file for ALT-landmarks can now be defined with geo-coordinates #15855
  - Option **--scale** can now be used for scaling traffic #8353

- polyconvert
  - Added option **--geosjon-files** for official geojson support #16055

- TraCI
  - stationfinder device parameters can now be modified at runtime #15622
  - Added `traci.parkingArea.setAcceptedBadges` and `traci.parkingArea.getAcceptedBadges`  #14807
  - Function `person.appendStage` now supports type `STAGE_TRIP` (`stage.line` is interpreted a `modes` and `stage.intended` as `group`) #15154

- Tools
  - matsim_importPlans.py: Added options **-no-bikes** and **--no-rides** for filtering different modes of traffic. #15738
  - sort_routes.py: Added option **--verbose** #15744
  - osmWebWizard.py: optionally can write output to existing folders #15783
  - emissionsMap: Now supports options **--vtype** and **--additional-files** #15812
  - driveways2poly.py: Added new tool for visualizing train driveways #15027
  - dxf2jupedsim.py: now supports projection 'none'
  - scaleTimeline.py: This is a new tool for modifying the amount of traffic defined in a route file based on a time line #10498
  - generateLandmarks.py: New tool for generating ALT-landmark input along the network rim #15864
  - cutRoutes.py: now writes standard header #15875
  - randomTrips.py: now includes total weight in weight-output file #15878
  - randomTrips.py: Added option **--edge-type-file** for affecting probabilities by edge type #15877
  - randomTrips.py: Added option **--marouter** to write routes which take into account traffic load on the network #15881
  - randomTrips.py: option **--flows** can now be used together with **--pedestrians** or **--persontrips** to create personFlows #12791
  - randomTrips.py: Added option **--poisson** to generate poisson-distributed flows (with option **--flows**) #13178
  - routeStats.py: Added option **--edges-file** for counting the number of times per route that a specific edge (i.e. a counting) location was passed) #15900
  - routeSampler.py: Added option **--verbose.timing** to print wall-clock-time performance statistics #15910
  - routeSampler.py: Major increase in processing speed for long routes #15911
  - routeSampler.py: Added option **--depart-distribution** to distribute departures within the counting data intervals #15909
  - routeSampler.py: now warn about duplicate counting data #15997
  - routeSampler.py: now includes GEH in mismatch-output #16000
  - routeSampler.py: Added option **--geh-scale** to permit custom scaling for GEH value (i.e. to avoid averaging daily counts over 24 hours) #16001
  - routeSampler.py: Options that set attributse to parse (i.e. **--edgedata-attribute**) now support a list of comma separated attributes (values are added) #16020
  - routeSampler.py: Added option **--init-input.remove-overflow** to downsample demand from a given route file where traffic exceeds counts (and keeping as many of the routes as possible) #15971
  - edgeDataDiff.py: Added option **--geh-scale** to permit custom scaling for GEH value and otherwise scaling data to hourly values automatically #16002
  - edgeDataDiff.py: Added option **--attributes** to allow comparing files with differing attribute names #15898
  - xml2csv.py: Added option **--keep-attributes** to limit the attributes exported to csv #15915
  - plotXMLAttributes.py: Added options **--split-x** and **--split-y** for plotting attributes with list values #15934
  - sumolib: Geometry helper functions for rotation at offset is now available #15445
  - duaIterate.py: When loading trips with taz or junction-taz, vehicles may change their depart and arrival edge in each iteration #15983
  - filterDistricts.py: New options **--remove-ids** and **--remove-ids-file** allow filtering out explicit edges #16038
  - countEdgeUsage.py: Now issues a warning when encountering attributes fromJunction or toJunction #15804
  - countEdgeUsage.py: print output on number loaded / filtered routes when option **--verbose** is set #16040
  - countEdgeUsage.py: New option **--subpart.via** allow filtering subparts with gaps (i.e. via edges) #16041
  - netdiff.py: Modified file extension for generated polygons to simplify file filtering in netedit #16042
  - poly2edgedata.py: Added new tool to transform traffic data from polygons (i.e. from geojson) to edgedata (i.e. for routeSampler.py) #16051
  - edgesInDistrict.py: now supports .xml.gz inputs #16066
  - edgeDepartDelay.py: New tool for edge-wise analysis of departDelay from tripinfo-output #16021
  - geoTrip2POI.py: New for visualizing geoTrips #16028
  - duaIterate_routeCosts.py: New tool to analyze route choice in a finished duaIterate run in an interactive manner #15912



### Miscellaneous

- Added analysis on the effects of attribute `departLane` on [insertion capacity](Simulation/RoadCapacity.md#insertion_capacity_on_a_2-lane_road).
- Fixed compatibility issues with Ubuntu 24 affecting geo-projections #15618
- Option **--device.rerouting.railsignal** is now inactive by default #15597
- If a turning movement has more than one lane and may wait within the intersection, the lanes after the *internal junction* are now joined into the same internal edge #14776

## Older Versions

- [Changes to versions 1.20.0 and 1.21.0 (2024 releases)](ChangeLog/Changes_in_2024_releases.md)
- [Changes to versions 1.16.0, 1.17.0, 1.18.0 and 1.19.0 (2023 releases)](ChangeLog/Changes_in_2023_releases.md)
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
