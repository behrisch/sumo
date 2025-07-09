---
title: ChangeLog
---

## Git Main

### Bugfixes

- sumo
  - Fixed invalid traffic light warnings #16602
  - Fixed overly verbose router output when using stationfinder device #16624
  - Vehicles no longer drive onto forbidden internal lanes when option **--ignore-route-errors** is set #16635
  - Fixed bug where junction collision was not detected #16695
  - Fixed collision on junction due to unsafe lane changing #16643
  - Fixed invalid waitingTime for walking stage in tripinfo-output (not accumulating) #16729
  - Previously recorded travel speeds are now preserved when loading state #16775
  - Stochastic assignment of rerouting device is now reproduced when loading state #16784
  - railway routing now ignores temporary road closings when option **--device.rerouting.mode 8** is set #16799
  - Fixed bug where persons with a personTrip had the wrong arrivalPos when changing from `<ride>` to `<walk>` #16801
  - Fixed invalid emergency stop when vehicle is teleported beyond `arrivalEdge` attribute #16802
  - Fixed bug where parking outflow would be blocked due to numerical issues. #16809
  - Fixed invalid rail signal state when departing before crossed tracks #16819
  - Fixed inconsistencies in emission value output. The option **--emission-output.precision** is now being applied also to [tripinfo](Simulation/Output/TripInfo.md) and [edgedata](Simulation/Output/Lane-_or_Edge-based_Emissions_Measures.md). #16832

- netedit
  - link-direction arrows for spread bidi-rail are drawn in the correct spot #16718 (regression in 1.20.0)
  - bidi-rail connections are drawn large enough for comfortable clicking #16701 (regression in 1.22.0)
  - bidi-rail connections are drawn on the correct side again #16700 (regression in 1.23.0)
  - Changing connection attribute 'uncontrolled' to `False` and a traffic light, now makes that connection controlled by the traffic light #16705
  - Fixed crash after using tls-mode "reset single" when loaded programs had non-standard programIDs. #16702
  - parkingArea reference in chargingStation is now loaded #16789

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

- mesosim
  - Fixed crash when loading state with different network #16758
  - Fixed crash when loading state with different values of **--meso-lane-queue** #16757
  - Queue entry block time is now preserved when saving/loading state #16770
  - Vehroute-output no longer contains invalid route edges when loading state and rerouting #16776
  - Fixed bug where calibrators caused invalid traffic data output when removing vehicles #16821

- netconvert
  - invalid right of way at left_before_right junction at specific angles #16793 (regression in 1.23.0)
  - Fixed bug where option **--tls.rebuild** creates invalid signal plan when using custom crossing traffic light indices. #16653
  - Fixed unsafe signal plan when crossings use linkIndex2 #16657
  - Fixed missing yellow phase when crossing re-uses vehicular link index #16658
  - OSM import: fixed low default speed for edge type *highway.service* when used by public transport #16763
  - Fixed bug where small roundabouts where sometimes not detected #16787

- TraCI
  - Fixed bug where vehicles would not change on their current lane after modifying permissions #16721

- tools
  - gtfs2pt.py: Fixed bug where option **--repair** did not fix broken **--osm-routes**. #16632 (regression in 1.17.0)
  - gtfs2pt.py: Fixed invalid vehicle departure when running with options **--osm-routes --bbox** #16731
  - gtfs2pt.py: Fixed obsolete config header when using **--osm-routes** #16680
  - createVehTypeDistribution.py: no longer crashes when output file already exists #16728 (regression in 1.21.0)
  - osmWebWizard.py: Fixed bug where the wizard wouldn't open properly on Linux #16086
  - net2geojson.py: Fixed bug that was causing invalid shapes with option **--boundary** #16295
  - tlsCycleAdaptation.py: Fixed invalid error #14015
  - scaleTimeLine.py: Fixed invalid sorting of output #16744
  - sumolib.net.connection: No longer ignores connection permissions #16633
  - sumolib.xml.toXML: custom indent is now passed on #16734
  - generateRerouters.py: fixed missing character in XML output #16815

### Enhancements

- sumo
  - A warning is now issued for traffic light programs if a link never gets a green phase even when the program has only a single phase. #16652
  - A warning is now issued for traffic light programs if two 'G' links target the same lane in the same phase #16636
  - Added waitingTime to personinfo walk output #16737
  - closingReroute now supports optional attribute `until` for setting the estimated duration of closing. Vehicle may wait at the closed edge if a detour would take longer. #16804
  - Vehicles may now exceed their vType-`maxSpeed` when using `carFollowModel="KraussPS"` and going downhill (up to their `desiredMaxSpeed`) #16805
  - containerStop now supports elemnt `<access>` #16811
  - Vehroute output for persons and containers can now selectively be disalled via param key `"has.vehroute.person-device"`  #16820
  - It is now possible to model [trailers / marshalling](Specification/Logistics.md#trailers_and_rail_cars) by changing vehicle properties upon loading/unloading of containers #8800
  - Experimental support for [CSV and Parquet output](TabularOutputs.md) #14694 #16791


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

- duarouter
  - Added option **--repair.max-detour-factor** to give more control over repairing routes. Vehicles will backtrack rather than take large detours. #16746

- tools
  - plotXMLAttributes.py: The options **--xticks-file** and **--yticks-file** now support giving a column for name aliases to group the respective values #16683
  - [plotStops.py](Tools/Railways.md#plotstopspy): New tool to simplify drawing a train schedule diagram along a specified route. #16683
  - generateContinuousRerouters.py: Added option **--stop-file** to add stops at loaded busStops when rerouting #16719
  - mapDetectors.py: Added option **--all-lanes** to place detectors on all lanes of an edge based on a single input coordinate #16751
  - plot_net_dump.py: Added opion **--colormap.center** two permit plotting colors with `TwoSlopeNorm` #16778



### Miscellaneous

- sumo-gui: swapped color semantics of stopping place occupancy indicator (red means used and green now means empty) #16668
- Shift-click no longer switches traffic lights or starts tracking vehicles in gaming mode #16703, #16704
- Added railway game #13446
- setting the python root dir now compiles libsumo against the selected python #16755
- It is recommended to use a compiler which supports C++17 (e.g. MSVC 2017 or later, g++ 7 or later)
  - MSVC 2015 is no longer supported (mainly due to updates in SUMOLibraries)
- netconvert: OSM-imported edges with type "highway.service" now have type "highway.service|psv" when used by public transport vehicles. This may require adaptations when using option **--keep-edges.by-type** or **--remove-edges.by-type** and when using edge-type specific [restrictions](Networks/PlainXML.md#vehicle-class_specific_speed_limits) or [meso-settings](Simulation/Meso.md#configuration_by_edge_type)  #16763


## Version 1.23.1 (08.05.2025)

### Bugfixes

- sumo
  - Fixed crash when person jumps between stops #16641
  - Fixed invalid arrivalPos when person jumps between stops #16642
  - Fixed crash when using `overtakeReroute` #16647

- netedit
  - Short connections are visible again when activating 'show connections' #16644 (regression in 1.22.0)
  - Fixed crash setting a negative time treshhold in E2 detectors
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
  - The electric vehicle model now honors engine shuttoff via params `shutOffStopDuration` and `shutOffAutoDuration` #16341
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
  - Fixed invalid right of way with respect to left-turns from the oncoming direction at junctiop type `left_before_right` #16480
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
  - Fixed bug where `vehicle.slowDown` require and additional simulation step to reach the taget speed and `vehicle.setAcceleration` was achieving the wrong acceleration #7361
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
  - collision-output now writes network coordindates of the front and rear of the involved vehicles #16509
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
  - Fixed saving of toolconfig for jtcrouter.py,plotFlows.py,signal_POIs_from_xodr.py,tripinfoByTAZ.py and tripinfoDiff.py #16085
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


## Older Versions

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
