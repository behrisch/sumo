<hTitles >SUMO User Documentation<hr style="border: 0.5px solid #338033;"></hTitles>

"**S**imulation of **U**rban **MO**bility" (SUMO) is an open source, highly portable, microscopic and continuous traffic simulation package designed to handle large networks. It allows for intermodal simulation including pedestrians and comes with a large set of tools for scenario creation. It is mainly developed by employees of the [Institute of Transportation Systems](https://www.dlr.de/ts) at the [German Aerospace Center](https://www.dlr.de). SUMO is licensed under the [EPL 2.0](https://www.eclipse.org/legal/epl-2.0/). The source code may also be made available under the following Secondary
Licenses when the conditions for such availability set forth in the EPL 2.0 are satisfied: [GPL2 or later](https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html).

Please use the [mailing lists](Contact.md) for sharing your results or for asking questions. Answers to some common questions may also be found in the [FAQ](FAQ.md).

If You use SUMO, please tell us about your
[Publications](Publications.md).

!!! note
    When citing SUMO in general please use our current reference publication: ["Microscopic Traffic Simulation using SUMO"](https://elib.dlr.de/127994/); Pablo Alvarez Lopez, Michael Behrisch, Laura Bieker-Walz, Jakob Erdmann, Yun-Pang Flötteröd, Robert Hilbrich, Leonhard Lücken, Johannes Rummel, Peter Wagner, and Evamarie Wießner. IEEE Intelligent Transportation Systems Conference (ITSC), 2018.

The content of this Documentation is freely editable. In order to edit a certain page, click on the "Edit on GitHub" button on the upper right corner and submit a Pull Request.
Here is a short help on [editing articles](Editing_Articles.md). You can also [Build](Developer/Documentation_Build.md) the documentation locally, or [Download](https://sumo.dlr.de/sumo_documentation.zip) a copy.

This Documentation is continuously updated and always refers to the latest development version. Documentation for a specific release version of SUMO is included in the download of that version and can be viewed by opening {{SUMO}}/docs/userdoc/index.html.

# Introduction

- [The traffic simulation SUMO](SUMO_at_a_Glance.md)

# Basic Usage

- [Notation in this Documentation](Basics/Notation.md)
- [Needed, basic Computer Skills](Basics/Basic_Computer_Skills.md)
- [Installing SUMO](Installing/index.md)
- [Using SUMO Command Line Applications](Basics/Using_the_Command_Line_Applications.md)
- [Tutorials](Tutorials/index.md)
- [Validating application inputs](XMLValidation.md)
- [Tabular outputs](TabularOutputs.md)

# Network Building

- Introduction to [SUMO Road Networks](Networks/SUMO_Road_Networks.md)
- [Abstract networks generation](Networks/Abstract_Network_Generation.md)
- Importing networks with [netconvert](netconvert.md)
  - [Defining own networks using XML](Networks/PlainXML.md)
  - [Importing non-SUMO networks](Networks/Import.md)
    - [from OpenStreetMap](Networks/Import/OpenStreetMap.md)
      - [3-Click Scenario Generator](Networks/Import/OpenStreetMap.md#3-click_scenario_generation)
    - [from VISUM](Networks/Import/VISUM.md)
    - [from Vissim](Networks/Import/Vissim.md)
    - [from OpenDRIVE](Networks/Import/OpenDRIVE.md)
    - [from MATsim](Networks/Import/MATsim.md)
    - [from ArcView (shapefiles)](Networks/Import/ArcView.md)
    - [from DlrNavTeq](Networks/Import/DlrNavteq.md)
    - [from Robocup Simulation League](Networks/Import/RoboCup.md)
  - [Importing SUMO networks and Patching](Networks/Import/SUMO_Road_Networks.md)
  - [Building networks for motorway simulation](Simulation/Motorways.md#building_a_network_for_motorway_simulation)
  - [Building networks for pedestrian simulation](Simulation/Pedestrians.md#building_a_network_for_pedestrian_simulation)
  - [Further netconvert options](Networks/Further_Options.md)
  - [Exporting non-SUMO networks](Networks/Export.md)
  - [Additional output](Networks/Further_Outputs.md)
- [Creating and modifying networks with netedit](Netedit/index.md)
- [Including elevation data](Networks/Elevation.md)
- [Geo-Coordinates](Geo-Coordinates.md)

# Demand Modelling

- [Introduction to SUMO Demand Modelling](Demand/Introduction_to_demand_modelling_in_SUMO.md)
- [Definition of Vehicles, Vehicle Types, and Routes](Definition_of_Vehicles,_Vehicle_Types,_and_Routes.md)
- [Defining Traffic Demand with netedit](Netedit/elementsDemand.md)
- [Simulation of public transport](Simulation/Public_Transport.md)
- [Simulation of individual persons and trip chains](Specification/Persons.md)
- [Simulation of logistics](Specification/Logistics.md)
- [Shortest or Optimal Path Routing](Demand/Shortest_or_Optimal_Path_Routing.md)
- [Intermodal Routing](IntermodalRouting.md)
- [Routing in the Simulation](Demand/Automatic_Routing.md)
- [Computing Dynamic User Assignment](Demand/Dynamic_User_Assignment.md)
- [Generating pedestrian traffic demand](Simulation/Pedestrians.md#generating_pedestrian_demand)
- [Generate a vehicle type distribution to model the fleet](Tools/Misc.md#createvehtypedistributionpy)

### Data sources for demand generation

- [Importing O/D Matrices](Demand/Importing_O/D_Matrices.md)
  - [Other VISUM Demand Importers](Demand/Further_Ways_to_import_VISUM_Demand_Definitions.md)
  - [Other Vissim Demand Importers](Demand/Further_Ways_to_import_Vissim_Demand_Definitions.md)
- [Routes from Counting Data (road counts, turn counts)](Demand/Routes_from_Observation_Points.md)
- [Routing by Turn Probabilities](Demand/Routing_by_Turn_Probabilities.md)
- [Activity-based Demand Generation](Demand/Activity-based_Demand_Generation.md)
- [Random Trips](Tools/Trip.md#randomtripspy)
- [Multi-modal random traffic](Tools/Import/OSM.md#osmwebwizardpy)
- [GTFS data](Tools/Import/GTFS.md)

# Simulation

- [Basic Definition](Simulation/Basic_Definition.md)
- [Saving and Loading Simulation State](Simulation/SaveAndLoad.md)
- [The SUMO-JuPedSim coupling](jupedsim.md)

## Output
- [Simulation output overview](Simulation/Output/index.md)

## TraCI (On-line Interaction)
- [TraCI overview](TraCI.md) - The **Tra**ffic **C**ontrol **I**nterface
- [Libsumo](Libsumo.md) - Using sumo as a library

## Traffic Management and Other Structures

- [Traffic Lights](Simulation/Traffic_Lights.md)
- [Public Transport](Simulation/Public_Transport.md)
- [Variable Speed Signs](Simulation/Variable_Speed_Signs.md)
- [Rerouter / Alternative Route Signage](Simulation/Rerouter.md)
- [Vaporizer](Simulation/Vaporizer.md) (deprecated, use [Calibrator](Simulation/Calibrator.md) instead)
- [Dynamic calibration of flow and speed and type](Simulation/Calibrator.md)
- [Parking areas](Simulation/ParkingArea.md)
- [Turnarounds](Simulation/Turnarounds.md)

## Traffic Modes

- [Pedestrian simulation](Simulation/Pedestrians.md)
- [Bicycle simulation](Simulation/Bicycles.md)
- [Railway simulation](Simulation/Railways.md)
- [Waterway simulation](Simulation/Waterways.md)

## Additional Features

- [Emissions](Models/Emissions.md)
- [Electric Vehicles](Models/Electric.md)
- [Electric Hybrid Vehicles, overhead lines, power substations](Models/ElectricHybrid.md)
- [Logistics](Specification/Logistics.md)
- [Generic Parameters](Simulation/GenericParameters.md)
- [Shape Visualization](Simulation/Shapes.md)
- [Wireless Device Detection](Simulation/Bluetooth.md)
- [Emergency Vehicles](Simulation/Emergency.md)
- [Simple Platooning (Simpla)](Simpla.md)
- [Demand Responsive Transport (DRT) / Taxis](Simulation/Taxi.md)
- [Green Light Optimal Speed Advisory (GLOSA)](Simulation/GLOSA.md)
- [Stationfinder (autonomous charging)](Simulation/Stationfinder.md)

## Model details

- [Vehicle speed](Simulation/VehicleSpeed.md)
- [Vehicle insertion](Simulation/VehicleInsertion.md)
- [Vehicle permissions (access restrictions)](Simulation/VehiclePermissions.md)
- [Road capacity](Simulation/RoadCapacity.md)
- [Intersection dynamics](Simulation/Intersections.md)
- [Randomness](Simulation/Randomness.md)
- [Routing and Re-routing](Simulation/Routing.md)
- [Sublane Model](Simulation/SublaneModel.md)
- [Opposite Direction Driving](Simulation/OppositeDirectionDriving.md)
- [Safety](Simulation/Safety.md)
- [Mesoscopic model](Simulation/Meso.md)
- [Lengths and Distances](Simulation/Distances.md)
- [Friction](Simulation/Friction.md)

## Common Problems

- [Why Vehicles are teleporting](Simulation/Why_Vehicles_are_teleporting.md)
- [Unexpected jamming](FAQ.md#the_simulation_has_lots_of_jamsdeadlocks_what_can_i_do)
- [Too many turn-arounds](Simulation/Turnarounds.md)
- [Unexpected lane-changing maneuvers?](FAQ.md#why_do_the_vehicles_perform_unexpected_lane-changing_maneuvers)
- [How to get high flows?](FAQ.md#how_do_i_get_high_flowsvehicle_densities)

# Additional Tools

In addition to the [main applications (sumo, sumo-gui, netedit, netconvert, etc.)](SUMO_at_a_Glance.md#included_applications), there are
over 250 additional tools. They cover topics from traffic network
analysis, demand generation, demand modification to output analysis.
Most of them are written in [python](https://www.python.org/). All tools
can be found in the SUMO-distribution under {{SUMO}}/tools.

For an index of all tools see:

- [Tool index](Tools/index.md)

Below are links to some of the most important/used tools:

- [osmWebWizard](Tools/Import/OSM.md#osmwebwizardpy) - create a simple scenario, in just a few clicks and using your web browser
- [Interfacing TraCI from Python](TraCI/Interfacing_TraCI_from_Python.md) - access a running SUMO simulation using Python
- [sumolib](Tools/Sumolib.md) - Python modules for working with SUMO networks and sumo xml files in general
- [Xml Tools](Tools/Xml.md) - tools for converting SUMO outputs to CSV/Spreadsheet, and vice versa
- [traceExporter.py](Tools/TraceExporter.md) - export mobility traces (FCD output) into different "trace file" formats
- [netdiff.py](Tools/Net.md#netdiffpy) - determine the differences between two networks
- [Visualization Tools](Tools/Visualization.md) - visualize a wide range of simulation outputs in a graphical and friendly way

# Theory

- [Traffic simulations in general](Theory/Traffic_Simulations.md)

# Application Manuals

- [sumo](sumo.md)
- [sumo-gui](sumo-gui.md)
- [netconvert](netconvert.md)
- [netedit](Netedit/index.md)
- [netgenerate](netgenerate.md)
- [od2trips](od2trips.md)
- [duarouter](duarouter.md)
- [jtrrouter](jtrrouter.md)
- [dfrouter](dfrouter.md)
- [marouter](marouter.md)
- [polyconvert](polyconvert.md)
- [activitygen](activitygen.md)
- [emissionsMap](Tools/Emissions.md#emissionsmap)
- [emissionsDrivingCycle](Tools/Emissions.md#emissionsdrivingcycle)

# Software Contributions
Some people extended SUMO or built tools to make it more usable. Not all of these extensions are part of the "SUMO core".

- [Included in the distribution](Contributed/index.md#included_in_the_distribution)
- [External](Contributed/index.md#external_extensions)

# Appendices

- [ChangeLog](ChangeLog.md)
- [Glossary](Other/Glossary.md)
- [FAQ](FAQ.md)
- [Known File Extensions](Other/File_Extensions.md)
- [List of all XML-elements and attributes](Netedit/attribute_help.md) supported by [netedit](Netedit/index.md)
