---
title: PHEMlight
---

# PHEM (Passenger Car and Heavy Duty Emission Model)

[PHEM (Passenger Car and Heavy Duty Emission
Model)](https://www.itna.tugraz.at/assets/files/areas/em/Phem_en.pdf)
is an instantaneous vehicle emission model developed by the TU Graz
since 1999. PHEM is based on an extensive European set of vehicle
measurements and covers passenger cars, light duty vehicles and heavy
duty vehicles from city buses up to 40 ton semi-trailers.

Within [COLOMBO](http://web.archive.org/web/20240722054138/https://verkehrsforschung.dlr.de/en/projects/colombo), a simplified version of PHEM,
embedded into SUMO, was developed. The [PHEMlight](https://www.itna.tugraz.at/assets/files/areas/em/PHEMlight_en.pdf) model is the result of
this work. An in-depth description can be found in
[COLOMBO](http://web.archive.org/web/20240722054138/https://verkehrsforschung.dlr.de/en/projects/colombo)'s deliverable
[D4.2](https://web.archive.org/web/20170808122505/https://www.colombo-fp7.eu/results_deliverables.php). The model
received a major update to version 2.0.0.0 in the beginning of 2016
which also used new names for all vehicle classes, most notably all
classes now have an english prefix distinguishing passenger cars (PC_),
light (LCV_) and heavy duty vehicles (HDV_). Since the class ames are
disjoint both models can be used alternatively or even in parallel
although the old classes are considered deprecated.

PHEMlight uses data files that include the parameters of the modelled
emission classes. At the time being, data sets for two emission classes
are included in SUMO: PKW_D_EU4 and PKW_G_EU4 (PC_D_EU4 and
PC_G_EU4 for the new model), resembling passenger vehicles, Euro Norm
4, powered by Diesel (PKW_D_EU4) and gasoline (PKW_G_EU4).

!!! caution
    Only two emission classes are included in the public version. Any further vehicle classes must be [licensed from the TU Graz](https://www.itna.tugraz.at/en/research/areas/em/simulation/phemlight.html).

To use PHEMlight, you can set an additional search path to these input
files using the configuration option **--phemlight-path** {{DT_FILE}} where {{DT_FILE}} is the path. Alternatively,
you may set the environment variable **PHEMLIGHT_PATH**. By default
SUMO is looking into <SUMO_HOME\>/data/emissions/PHEMlight and in the PHEMlight subdir
of your working directory.

In addition, your vehicles must be assigned to one of PHEMlight's
vehicle emission classes, summarised in the following table. To use the
emission classes below prepend the class name with the PHEMlight prefix,
e.g. `emissionClass="PHEMlight/PKW_G_EU4"`.

## Explanations for the 2.0 naming conventions

The name is composed of the vehicle category and size, the technology
and the emission standard, all connected with underscores.

### Vehicle Category and Size Classes

- **PC**  passenger cars

- **LCV**  light commercial vehicles; differentiation into size classes
  according to legislation (N1-I, N1-II, N1-III); size class I RM <
  1305kg; size class II 1305kg < RM <= 1760kg; size class III 1760kg
  < RM <= 3500kg; "RM": reference mass (approx. Vehicle curb
  weight)

- **HDV_RT**  heavy duty vehicles - rigid trucks; size class I 2 axle
  trucks; size class II 3+ axle trucks

- **HDV_TT**  heavy duty vehicles - truck + trailer (incl. articulated
  trucks = tractor + semitrailer)

- **HDV_CB**  heavy duty vehicles - city bus

- **HDV_CO**  heavy duty vehicles - coach

- **MC_2S**  motorcycles - 2 stroke (not included)

- **MC_4S**  motorcycles - 4 stroke (not included)

- **MOP**  moped (not included)

### Technology

- **G**  gasoline engine (conventional powertrain)

- **D**  diesel engine (conventional powertrain)

- **G_HEV**  gasoline engine (parallel hybrid powertrain)

- **D_HEV**  diesel engine (parallel hybrid powertrain)

- **CNG**  Compressed Natural Gas engine

- **BEV**  battery electric vehicles

### Examples

- **PC_G_EU6**  passenger car, gasoline engine, EURO 6

- **PC_BEV**  passenger car, battery electric

- **LCV_III_CNG_EU6**  light commercial vehicle, size class III, CNG engine, Euro 6

- **HDV_RT_I_D_EU4**  rigid truck, size class I, diesel engine, EURO IV

- **HDV_CB_D_EU5**  city bus, diesel engine, Euro V

### Remarks on Hybrid vehicles

The fleet average "HEV" vehicles in PHEM and PHEMlight do NOT match with
single particular vehicles in the market (e.g. Toyota Prius) but have
been defined as follows:

1.  Baseline vehicle setup (e.g. vehicle size) similar than related
    conventional vehicle
2.  Total rated vehicle power of baseline conventional vehicle split to
    combustion engine and to electric motor (EM)
3.  Additional vehicle weight due to EM and battery considered
4.  Application of generic parallel hybrid operation strategy
    (recuperation, boosting, charging etc.)

Also note that plugin hybrids (where the main powertrain is electric)
should be modelled as battery vehicles and need to switch their emission
class (and thus their vehicle type in SUMO) when running conventional.

The following tables show the available classes. Some classes
(especially for motor cycles) were removed in the 2.0 version and some
(especially alternative fuels) are only available there.

## PHEMlight Emission Classes for Light Vehicles running on Diesel or Gasoline

<table style="border-collapse:collapse;border-spacing:0" class="tg"><tr><th colspan="2">Passenger Cars</th><th colspan="6">Light Duty Vehicles</th></tr>
  <tr><td></td><td style="background-color:#34ff34">2.0.0.0</td><td></td><td style="background-color:#34ff34">2.0.0.0</td><td></td><td style="background-color:#34ff34">2.0.0.0</td><td></td><td style="background-color:#34ff34">2.0.0.0</td></tr>
  <tr><td>PKW_D_EU0</td><td>PC_D_EU0</td><td>LNF_D_EU0_I</td><td>LCV_I_D_EU0</td><td>LNF_D_EU0_II</td><td>LCV_II_D_EU0</td><td>LNF_D_EU0_III</td><td>LCV_III_D_EU0</td></tr>
  <tr><td>PKW_D_EU1</td><td>PC_D_EU1</td><td>LNF_D_EU1_I</td><td>LCV_I_D_EU1</td><td>LNF_D_EU1_II</td><td>LCV_II_D_EU1</td><td>LNF_D_EU1_III</td><td>LCV_III_D_EU1</td></tr>
  <tr><td>PKW_D_EU2</td><td>PC_D_EU2</td><td>LNF_D_EU2_I</td><td>LCV_I_D_EU2</td><td>LNF_D_EU2_II</td><td>LCV_II_D_EU2</td><td>LNF_D_EU2_III</td><td>LCV_III_D_EU2</td></tr>
  <tr><td>PKW_D_EU3</td><td>PC_D_EU3</td><td>LNF_D_EU3_I</td><td>LCV_I_D_EU3</td><td>LNF_D_EU3_II</td><td>LCV_II_D_EU3</td><td>LNF_D_EU3_III</td><td>LCV_III_D_EU3</td></tr>
  <tr><td>PKW_D_EU4</td><td>PC_D_EU4</td><td>LNF_D_EU4_I</td><td>LCV_I_D_EU4</td><td>LNF_D_EU4_II</td><td>LCV_II_D_EU4</td><td>LNF_D_EU4_III</td><td>LCV_III_D_EU4</td></tr>
  <tr><td>PKW_D_EU5</td><td>PC_D_EU5</td><td>LNF_D_EU5_I</td><td>LCV_I_D_EU5</td><td>LNF_D_EU5_II</td><td>LCV_II_D_EU5</td><td>LNF_D_EU5_III</td><td>LCV_III_D_EU5</td></tr>
  <tr><td>PKW_D_EU6</td><td>PC_D_EU6</td><td>LNF_D_EU6_I</td><td>LCV_I_D_EU6</td><td>LNF_D_EU6_II</td><td>LCV_II_D_EU6</td><td>LNF_D_EU6_III</td><td>LCV_III_D_EU6</td></tr>
  <tr><td></td><td>PC_D_EU6c</td><td></td><td>LCV_I_D_EU6c</td><td></td><td>LCV_II_D_EU6c</td><td></td><td>LCV_III_D_EU6c</td></tr>
  <tr><td>PKW_G_EU0</td><td>PC_G_EU0</td><td>LNF_G_EU0_I</td><td>LCV_I_G_EU0</td><td>LNF_G_EU0_II</td><td>LCV_II_G_EU0</td><td>LNF_G_EU0_III</td><td>LCV_III_G_EU0</td></tr>
  <tr><td>PKW_G_EU1</td><td>PC_G_EU1</td><td>LNF_G_EU1_I</td><td>LCV_I_G_EU1</td><td>LNF_G_EU1_II</td><td>LCV_II_G_EU1</td><td>LNF_G_EU1_III</td><td>LCV_III_G_EU1</td></tr>
  <tr><td>PKW_G_EU2</td><td>PC_G_EU2</td><td>LNF_G_EU2_I</td><td>LCV_I_G_EU2</td><td>LNF_G_EU2_II</td><td>LCV_II_G_EU2</td><td>LNF_G_EU2_III</td><td>LCV_III_G_EU2</td></tr>
  <tr><td>PKW_G_EU3</td><td>PC_G_EU3</td><td>LNF_G_EU3_I</td><td>LCV_I_G_EU3</td><td>LNF_G_EU3_II</td><td>LCV_II_G_EU3</td><td>LNF_G_EU3_III</td><td>LCV_III_G_EU3</td></tr>
  <tr><td>PKW_G_EU4</td><td>PC_G_EU4</td><td>LNF_G_EU4_I</td><td>LCV_I_G_EU4</td><td>LNF_G_EU4_II</td><td>LCV_II_G_EU4</td><td>LNF_G_EU4_III</td><td>LCV_III_G_EU4</td></tr>
  <tr><td>PKW_G_EU5</td><td>PC_G_EU5</td><td>LNF_G_EU5_I</td><td>LCV_I_G_EU5</td><td>LNF_G_EU5_II</td><td>LCV_II_G_EU5</td><td>LNF_G_EU5_III</td><td>LCV_III_G_EU5</td></tr>
  <tr><td>PKW_G_EU6</td><td>PC_G_EU6</td><td>LNF_G_EU6_I</td><td>LCV_I_G_EU6</td><td>LNF_G_EU6_II</td><td>LCV_II_G_EU6</td><td>LNF_G_EU6_III</td><td>LCV_III_G_EU6</td></tr>
  <tr><td></td><td>PC_G_EU6c</td><td></td><td>LCV_I_G_EU6c</td><td></td><td>LCV_II_G_EU6c</td><td></td><td>LCV_III_G_EU6c</td></tr>
</table>

## PHEMlight Emission Classes for Light Vehicles running on alternative fuels

<table style="border-collapse:collapse;border-spacing:0" class="tg"><tr><th colspan="2">Passenger Cars</th><th colspan="3">Light Duty Vehicles<br></th></tr>
  <tr><td></td><td style="background-color:#34ff34">2.0.0.0</td><td style="background-color:#34ff34">2.0.0.0</td><td style="background-color:#34ff34">2.0.0.0</td><td style="background-color:#34ff34">2.0.0.0</td></tr>
  <tr><td></td><td>PC_BEV</td><td>LCV_I_BEV</td><td>LCV_II_BEV</td><td>LCV_III_BEV</td></tr>
  <tr><td></td><td>PC_CNG_EU5</td><td>LCV_I_CNG_EU5</td><td>LCV_II_CNG_EU5</td><td>LCV_III_CNG_EU5</td></tr>
  <tr><td></td><td>PC_CNG_EU6</td><td>LCV_I_CNG_EU6</td><td>LCV_II_CNG_EU6</td><td>LCV_III_CNG_EU6</td></tr>
  <tr><td></td><td>PC_CNG_EU6c</td><td>LCV_I_CNG_EU6c</td><td>LCV_II_CNG_EU6c</td><td>LCV_III_CNG_EU6c</td></tr>
  <tr><td>H_PKW_D_EU5</td><td></td><td></td><td></td><td></td></tr>
  <tr><td>H_PKW_D_EU6</td><td>PC_D_HEV_EU6</td><td>LCV_I_D_HEV_EU6</td><td>LCV_II_D_HEV_EU6</td><td>LCV_III_D_HEV_EU6</td></tr>
  <tr><td></td><td>PC_D_HEV_EU6c</td><td>LCV_I_D_HEV_EU6c</td><td>LCV_II_D_HEV_EU6c</td><td>LCV_III_D_HEV_EU6c</td></tr>
  <tr><td>H_PKW_G_EU5</td><td></td><td></td><td></td><td></td></tr>
  <tr><td>H_PKW_G_EU6</td><td>PC_G_HEV_EU6</td><td>LCV_I_G_HEV_EU6</td><td>LCV_II_G_HEV_EU6</td><td>LCV_III_G_HEV_EU6</td></tr>
  <tr><td></td><td>PC_G_HEV_EU6c</td><td>LCV_I_G_HEV_EU6c</td><td>LCV_II_G_HEV_EU6c</td><td>LCV_III_G_HEV_EU6c</td></tr>
</table>

## PHEMlight Emission Classes for Motor Cycles

Motor cycles are only available for the old data set (pre 2016)

<table>
  <tr>
    <th>Two-Wheelers</th>
    <th colspan="2">Motor Cycles</th>
  </tr>
  <tr>
    <td>KKR_G_EU0</td>
    <td>MR_G_EU0_2T</td>
    <td>MR_G_EU0_4T</td>
  </tr>
  <tr>
    <td>KKR_G_EU1</td>
    <td>MR_G_EU1_2T</td>
    <td>MR_G_EU1_4T</td>
  </tr>
  <tr>
    <td>KKR_G_EU2</td>
    <td>MR_G_EU2_2T</td>
    <td>MR_G_EU2_4T</td>
  </tr>
  <tr>
    <td>KKR_G_EU3</td>
    <td>MR_G_EU3_2T</td>
    <td>MR_G_EU3_4T</td>
  </tr>
  <tr>
    <td>KKR_G_EU4</td>
    <td>MR_G_EU4_2T</td>
    <td>MR_G_EU4_4T</td>
  </tr>
  <tr>
    <td></td>
    <td>MR_G_EU5_2T</td>
    <td>MR_G_EU5_4T</td>
  </tr>
</table>

## PHEMlight Emission Classes for Heavy Vehicles

<table style="border-collapse:collapse;border-spacing:0" class="tg"><tr><th colspan="2">Tractor/Trailor</th><th colspan="2">Coach</th><th colspan="2">Urban and Inter Urban Buses</th><th colspan="4">Trucks</th></tr>
  <tr><td></td><td style="background-color:#34ff34">2.0.0.0</td><td></td><td style="background-color:#34ff34">2.0.0.0</td><td></td><td style="background-color:#34ff34">2.0.0.0</td><td></td><td style="background-color:#34ff34">2.0.0.0</td><td></td><td style="background-color:#34ff34">2.0.0.0</td></tr>
  <tr><td></td><td></td><td></td><td></td><td></td><td>HDV_CB_BEV</td><td></td><td></td><td></td><td></td></tr>
  <tr><td></td><td></td><td></td><td></td><td></td><td>HDV_CB_CNG_EU6</td><td></td><td></td><td></td><td></td></tr>
  <tr><td>LSZ_D_EU0</td><td>LSZ_D_EU0</td><td>RB_D_EU0</td><td>HDV_CO_D_EU0</td><td>LB_D_EU0</td><td>HDV_CB_D_EU0</td><td>SOLO_LKW_D_EU0_I</td><td>HDV_RT_I_D_EU0</td><td>SOLO_LKW_D_EU0_II</td><td>HDV_RT_II_D_EU0</td></tr>
  <tr><td>LSZ_D_EU1</td><td>HDV_TT_D_EU1</td><td>RB_D_EU1</td><td>HDV_CO_D_EU1</td><td>LB_D_EU1</td><td>HDV_CB_D_EU1</td><td>SOLO_LKW_D_EU1_I</td><td>HDV_RT_I_D_EU1</td><td>SOLO_LKW_D_EU1_II</td><td>HDV_RT_II_D_EU1</td></tr>
  <tr><td>LSZ_D_EU2</td><td>HDV_TT_D_EU2</td><td>RB_D_EU2</td><td>HDV_CO_D_EU2</td><td>LB_D_EU2</td><td>HDV_CB_D_EU2</td><td>SOLO_LKW_D_EU2_I</td><td>HDV_RT_I_D_EU2</td><td>SOLO_LKW_D_EU2_II</td><td>HDV_RT_II_D_EU2</td></tr>
  <tr><td>LSZ_D_EU3</td><td>HDV_TT_D_EU3</td><td>RB_D_EU3</td><td>HDV_CO_D_EU3</td><td>LB_D_EU3</td><td>HDV_CB_D_EU3</td><td>SOLO_LKW_D_EU3_I</td><td>HDV_RT_I_D_EU3</td><td>SOLO_LKW_D_EU3_II</td><td>HDV_RT_II_D_EU3</td></tr>
  <tr><td>LSZ_D_EU4</td><td>HDV_TT_D_EU4</td><td>RB_D_EU4</td><td>HDV_CO_D_EU4</td><td>LB_D_EU4</td><td>HDV_CB_D_EU4</td><td>SOLO_LKW_D_EU4_I</td><td>HDV_RT_I_D_EU4</td><td>SOLO_LKW_D_EU4_II</td><td>HDV_RT_II_D_EU4</td></tr>
  <tr><td>LSZ_D_EU5</td><td>HDV_TT_D_EU5</td><td>RB_D_EU5</td><td>HDV_CO_D_EU5</td><td>LB_D_EU5</td><td>HDV_CB_D_EU5</td><td>SOLO_LKW_D_EU5_I</td><td>HDV_RT_I_D_EU5</td><td>SOLO_LKW_D_EU5_II</td><td>HDV_RT_II_D_EU5</td></tr>
  <tr><td>LSZ_D_EU6</td><td>HDV_TT_D_EU6</td><td>RB_D_EU6</td><td>HDV_CO_D_EU6</td><td>LB_D_EU6</td><td>HDV_CB_D_EU6</td><td>SOLO_LKW_D_EU6_I</td><td>HDV_RT_I_D_EU6</td><td>SOLO_LKW_D_EU6_II</td><td>HDV_RT_II_D_EU6</td></tr>
  <tr><td></td><td></td><td></td><td></td><td></td><td>HDV_CB_D_HEV_EU6</td><td></td><td></td><td></td><td></td></tr>
</table>

## Special Classes

The special class `zero` may be used to specify a vehicle without emissions.

## Further Resources

- [COLOMBO: Deliverable 4.2; Extended Simulation Tool PHEM coupled to
  SUMO with User
  Guide](https://web.archive.org/web/20170808122505/https://www.colombo-fp7.eu/results_deliverables.php); February,
  2014

 <div style="border:1px solid #909090; min-height: 35px;" align="right">
<span style="float: right; margin-top: -5px;"><a href="https://web.archive.org/web/20191005024529/https:/ec.europa.eu/research/fp7/index_en.cfm"><img src="../../images/FP7-small.gif" alt="Seventh Framework Programme"></a>
<a href="http://web.archive.org/web/20240722054138/https://verkehrsforschung.dlr.de/en/projects/colombo"><img src="../../images/COLOMBO-small.png" alt="COLOMBO project"></a></span>
<span style="">This part of SUMO was developed, reworked, or extended within the project
<a href="http://web.archive.org/web/20240722054138/https://verkehrsforschung.dlr.de/en/projects/colombo">"COLOMBO"</a>, co-funded by the European Commission within the <a href="https://web.archive.org/web/20191005024529/https:/ec.europa.eu/research/fp7/index_en.cfm">Seventh Framework Programme</a>.</span></div>
