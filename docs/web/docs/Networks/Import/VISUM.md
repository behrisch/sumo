---
title: VISUM
---

[netconvert](../../netconvert.md) can import native
[VISUM](https://www.ptvgroup.com/en/solutions/products/ptv-visum/)-network files.
Their extension is ".net". If you do not have a file with this
extension, but a ".ver"-file only, you have to generate the ".net"-file
using [VISUM](https://www.ptvgroup.com/en/solutions/products/ptv-visum/) by exporting
it from the loaded version description (".ver"-file).

The option to load a
[VISUM](https://www.ptvgroup.com/en/solutions/products/ptv-visum/) ".net"-file into
[netconvert](../../netconvert.md) in order to convert it into a
SUMO-network is named **--visum-file** {{DT_FILE}} or **--visum** {{DT_FILE}} for short. So, the following call to
[netconvert](../../netconvert.md) imports the road network stored in
"my_visum_net.net" and stores the SUMO-network generated from this
data into "my_sumo_net.net.xml":

```
netconvert --visum my_visum_net.net -o my_sumo_net.net.xml
```

!!! caution
    If your network contains geo-coordinates (lon/lat) you must add option **--proj.utm** in order to project your network into the cartesian plane

# Localization

VISUM network files can use keywords in different languages. By default, [netconvert](../../netconvert.md) attempts to import german keywords.
The option **--visum.language-file {{DT_FILE}}** can be used to switch to a different language.  The following language files are provided in the `<SUMO_HOME>/data/lang` directory:

- visumEN.txt
- visumFR.txt
- visumIT.txt

If you wish to import a file in an unsupported language, please [contact](https://eclipse.dev/sumo/contact/) us and provide an example file. 

# Import Coverage

The following table shows which information is parsed from a given VISUM
network.

!!! caution
    The keywords within a VISUM file are localized. By default netconvert assumes LANGUAGE **DEU**. You can use option **--visum.language-file** {{DT_FILE}} to read a language mapping from DEU into another language. Mapping files for can be found in the `SUMO_HOME/data/lang` folder.

**Information [netconvert](../../netconvert.md) reads from VISUM
networks**

| Table name  | Imported attributes  | Description  |
|---|---|---|
| VSYS  | VSysCode (CODE)<br>VSysMode (TYP)  | Traffic modes  |
| STRECKENTYP  | Nr<br>v0-IV (V0IV)<br>Rang<br>Kap-IV (KAPIV)  | Edge types  |
| KNOTEN  | Nr<br>XKoord<br>YKoord  | Nodes  |
| BEZIRK  | Nr<br>NAME (unused later)<br>Proz_Q<br>Proz_Z<br>XKoord<br>YKoord  | Districts  |
| STRECKE (STRECKEN)  | Nr<br>VonKnot (VonKnotNr)<br>NachKnot (NachKnotNr)<br>Typ (TypNr)<br>Einbahn  | Edges  |
| ANBINDUNG  | BezNr<br>KnotNr<br>Proz<br>t0-IV<br>Typ<br>Richtung  | District connections  |
| ABBIEGEBEZIEHUNG (ABBIEGER)  | VonKnot (VonKnotNr)<br>UeberKnot (UeberKnotNr)<br>NachKnot (NachKnotNr)<br>VSysCode (VSYSSET)  | Edge Connections  |
| STRECKENPOLY  | VonKnot (VonKnotNr)<br>NachKnot (NachKnotNr)<br>INDEX<br>XKoord<br>YKoord  | Edge geometries  |
| FAHRSTREIFEN  | KNOTNR<br>STRNR<br>FSNR<br>RICHTTYP<br>LAENGE  | Lane descriptions  |
| LSA (SIGNALANLAGE)  | Nr<br>Umlaufzeit (UMLZEIT)<br>StdZwischenzeit (STDZWZEIT)<br>PhasenBasiert  | Traffic lights  |
| KNOTENZULSA (SIGNALANLAGEZUKNOTEN)  | KnotNr<br>LsaNr  | Nodes->TLS  |
| LSASIGNALGRUPPE (SIGNALGRUPPE)  | Nr<br>LsaNr<br>GzStart (GRUENANF)<br>GzEnd (GRUENENDE)<br>GELB  | Signal groups  |
| ABBZULSASIGNALGRUPPE<br>(SIGNALGRUPPEZUABBIEGER)   | SGNR (SIGNALGRUPPENNR)<br>LsaNr<br>VonKnot / VONSTRNR<br>NachKnot / NACHSTRNR<br>UeberKnot (UeberKnotNr)<br>LsaNr  | Edge connections->TLS  |
| LSAPHASE (PHASE)  | Nr<br>LsaNr<br>GzStart (GRUENANF)<br>GzEnd (GRUENENDE)  | Signal phases  |
| LSASIGNALGRUPPEZULSAPHASE  | PsNr<br>LsaNr<br>SGNR  | Signal groups->phases  |
| FAHRSTREIFENABBIEGER  | KNOT (KNOTNR)<br>VONSTR (VONSTRNR)<br>NACHSTR (NACHSTRNR)<br>VONFSNR<br>NACHFSNR  | Lane-to-lane descriptions  |


Well, basically that's all for network import, isn't it? Well, actually
not. In the following, the basic possibilities and further advices and
tricks for working with these is described.

!!! note
    "Rang" from "Streckentypen" is used as edge priority information. As streets with a lower "Rang" are normally higher priorised, an edge type's priority is computed as 100-Rang

## Lane Number

[VISUM](https://www.ptvgroup.com/en/solutions/products/ptv-visum/) does not work with
the lane numbers of streets, instead, the streets' capacities are used.
This means that under circumstances the information about the edges' lane
numbers are missing. Still, one can try to obtain the lane number from
the given edges' capacities. An approximation is:

```
LANE_NUMBER = MAXIMUM_FLOW / CAPACITY_NORM
```

The value of CAPACITY_NORM is controlled via the option **--capacity-norm** {{DT_FLOAT}} (default:
1800).

In the case the "*ANZFAHRSTREIFEN*" (lane numbers) field within the
net's "*STRECKEN*" (edges)-table is empty, and also the types do not
hold any information about the according edges' lane numbers, this
conversion is done automatically. Still, we also had
[VISUM](https://www.ptvgroup.com/en/solutions/products/ptv-visum/) nets in our hands,
where a capacity was given, but the lane number field was set to 1 for
all edges. In this case, one has to force
[netconvert](../../netconvert.md) to ignore the lane number attribute
and use the capacity. This is done by using the option
**--visum.recompute-laneno**.

## Dealing with Connectors

[VISUM](https://www.ptvgroup.com/en/solutions/products/ptv-visum/) as a macroscopic
tool, does not regard single vehicles. As one consequence,
[VISUM](https://www.ptvgroup.com/en/solutions/products/ptv-visum/) uses "connections"
through which traffic is fed from the districts into the network.

The effects on using such connectors within a microscopic simulations
should be described at a different
page. Here, we want to show the
possibilities to change the connector attributes using
[netconvert](../../netconvert.md).


## Permissions

A VISUM model declares a list of abstract traffic types and uses these to define access rules for every road. While there are some conventions for typical types (i.e. 'c' for passenger car and 'w' for walk), these types are user-definable. Since version 1.27.0 netconvert uses the following three strategies to interpret permissions (earlier versions only use the first strategy):

- a [pre-defined list](https://github.com/eclipse-sumo/sumo/blob/main/data/typemap/visumNetconvert.typ.xml) of typical traffic type codes
- interpreting the textual description field for each type (i.e. anything with the substring 'bus' in it's description is guessed as abstract vehicle class 'bus')
- loading a custom type file with option **--type-files** that maps traffic type ids to permissions (the [default file](https://github.com/eclipse-sumo/sumo/blob/main/data/typemap/visumNetconvert.typ.xml) can be copied and extended as needed).

# See also

[netconvert](../../netconvert.md) is able to guess some information
which is sometimes missing in imported networks. Below, you may find
links to further information of interest.

- Most [VISUM](https://www.ptvgroup.com/en/solutions/products/ptv-visum/) networks
  do not contain definitions of traffic lights positions; Still,
  [netconvert](../../netconvert.md) is able to [guess tls
  positions](../../netconvert.md#tls_building) and to [guess
  tls programs](../../netconvert.md#tls_building).
- Also, we have not seen a
  [VISUM](https://www.ptvgroup.com/en/solutions/products/ptv-visum/) network where
  on- and off-ramps where available for highways.
  [netconvert](../../netconvert.md) is able to [guess on- and
  off-ramps](../../netconvert.md#ramp_guessing).
- In addition to the network, further descriptions of lane-to-lane or
  edge-to-edge connections may be read.

Other possibilities of [netconvert](../../netconvert.md), such as
projection of geo-coordinates, should not apply when working with
[VISUM](https://www.ptvgroup.com/en/solutions/products/ptv-visum/) networks.

## Importing other data from VISUM

- netconvert can import [public transport stop locations](../Simulation/Public_Transport.md#bus_stops) by setting option **--ptstop-output**
- netconvert can import [Traffic Analysis Zones (TAZ)](../Demand/Importing_O/D_Matrices.md#describing_the_taz) by setting option **--polygon-output**
- [VISUM](https://www.ptvgroup.com/en/solutions/products/ptv-visum/) uses O/D-matrices
as a demand descriptions. There is some further information on
[Demand/Importing O/D
Matrices](../../Demand/Importing_O/D_Matrices.md).
- Abstract network model XML routes (anm_routes.xml) can be imported with [visum_convertXMLRoutes.py](../Tools/Import/VISUM.md#visum_convertxmlroutespy)

# References

- [PTV AG's
  VISUM-page](https://www.ptvgroup.com/en/solutions/products/ptv-visum/)
  (06.09.2011)

# Missing

- Attributes of connectors
- Importing POIs/Shapes
- what is exactly imported (how edge attributes are determined)
- other traffic modes
- Network quality
