---
title: Tabular Output
---

# Generating tabular output

Almost all SUMO tools and applications consume and/or generate XML files
which can be converted to other formats using some of the [XML tools](Tools/Xml.md).
Starting with SUMO 1.24.0 there is experimental support for generating tabular outputs
in CSV or [Parquet](https://parquet.apache.org/) format directly from SUMO applications.

The easiest way to do so is to give your output file a different name:
```
sumo -c my.sumocfg --fcd-output fcd.csv
```
will generate a CSV file instead of an XML file and
```
sumo -c my.sumocfg --fcd-output fcd.parquet
```
will generate Parquet output.

Most of the time the mapping of XML to a tabular format is straight forward. For FCD output for instance the FCD file:
```xml
<fcd-export xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/fcd_file.xsd">
    <timestep time="0.00"/>
    <timestep time="1.00"/>
    <timestep time="2.00"/>
...
    <timestep time="97.00"/>
    <timestep time="98.00"/>
    <timestep time="99.00"/>
    <timestep time="100.00">
        <vehicle id="always_left.0" x="5.10" y="498.35" angle="90.00" type="DEFAULT_VEHTYPE" speed="0.00" pos="5.10" lane="1fi_0" slope="0.00"/>
        <vehicle id="always_right.0" x="501.65" y="5.10" angle="0.00" type="DEFAULT_VEHTYPE" speed="0.00" pos="5.10" lane="3fi_0" slope="0.00"/>
        <vehicle id="horizontal.0" x="994.90" y="501.65" angle="270.00" type="DEFAULT_VEHTYPE" speed="0.00" pos="5.10" lane="2fi_0" slope="0.00"/>
    </timestep>
    <timestep time="101.00">
        <vehicle id="always_left.0" x="7.02" y="498.35" angle="90.00" type="DEFAULT_VEHTYPE" speed="1.92" pos="7.02" lane="1fi_0" slope="0.00"/>
        <vehicle id="always_right.0" x="501.65" y="7.46" angle="0.00" type="DEFAULT_VEHTYPE" speed="2.36" pos="7.46" lane="3fi_0" slope="0.00"/>
        <vehicle id="horizontal.0" x="993.58" y="501.65" angle="270.00" type="DEFAULT_VEHTYPE" speed="1.32" pos="6.42" lane="2fi_0" slope="0.00"/>
    </timestep>
    <timestep time="102.00">
        <vehicle id="always_left.0" x="10.66" y="498.35" angle="90.00" type="DEFAULT_VEHTYPE" speed="3.64" pos="10.66" lane="1fi_0" slope="0.00"/>
        <vehicle id="always_right.0" x="501.65" y="11.19" angle="0.00" type="DEFAULT_VEHTYPE" speed="3.72" pos="11.19" lane="3fi_0" slope="0.00"/>
        <vehicle id="horizontal.0" x="990.38" y="501.65" angle="270.00" type="DEFAULT_VEHTYPE" speed="3.20" pos="9.62" lane="2fi_0" slope="0.00"/>
    </timestep>
...
```

will be mapped to the CSV
```csv
timestep_time;vehicle_id;vehicle_x;vehicle_y;vehicle_angle;vehicle_type;vehicle_speed;vehicle_pos;vehicle_lane;vehicle_edge;vehicle_slope
100.00;always_left.0;5.10;498.35;90.00;DEFAULT_VEHTYPE;0.00;5.10;1fi_0;;0.00
100.00;always_right.0;501.65;5.10;0.00;DEFAULT_VEHTYPE;0.00;5.10;3fi_0;;0.00
100.00;horizontal.0;994.90;501.65;270.00;DEFAULT_VEHTYPE;0.00;5.10;2fi_0;;0.00
101.00;always_left.0;7.02;498.35;90.00;DEFAULT_VEHTYPE;1.92;7.02;1fi_0;;0.00
101.00;always_right.0;501.65;7.46;0.00;DEFAULT_VEHTYPE;2.36;7.46;3fi_0;;0.00
101.00;horizontal.0;993.58;501.65;270.00;DEFAULT_VEHTYPE;1.32;6.42;2fi_0;;0.00
102.00;always_left.0;10.66;498.35;90.00;DEFAULT_VEHTYPE;3.64;10.66;1fi_0;;0.00
102.00;always_right.0;501.65;11.19;0.00;DEFAULT_VEHTYPE;3.72;11.19;3fi_0;;0.00
102.00;horizontal.0;990.38;501.65;270.00;DEFAULT_VEHTYPE;3.20;9.62;2fi_0;;0.00
...
```

That means the structure is completely flattened and all lines which do not have the data of the innermost element (the empty "timestep"s at the beginning)
are omitted. All attributes of the outer elements ("timestep_time" in the example) are repeated in the inner elements.
The column names are derived from the XML element name in conjunction with the attribute name (which is the same format the python tool xml2csv.py uses)
but this can be configured (see below). Please note that the tabular format may contain more columns than the XML ("vehicle_edge" in the example above)
to support different simulation configurations with the same format (for instance meso- and micro-simulation).

!!! caution
    Tabular output is still an experimental feature. Be prepared to loose data especially when getting warnings about unsupported formats.

The motivation for using CSV or Parquet is to get smaller files and better reading times for large outputs such as SUMO's
[FCD output](Simulation/Output/FCDOutput.md). You can expect speedups of a factor up to 50 when reading those files
from Parquet instead of XML using Python's pandas library (see [performance](#performance)).

To further configure the tabular outputs, the following options can be used:
| Option  | Description  |
|---|---|
| **--output.compression** {{DT_STR}} | Defines the standard compression algorithm (currently only for parquet output: 'uncompressed', 'gzip', 'bz2', 'zstd'); *default:* **uncompressed** |
| **--output.format** {{DT_STR}} | Defines the standard output format if not derivable from the file name ('xml', 'csv', 'parquet'); *default:* **xml** |
| **--output.column-header** {{DT_STR}} | How to derive column headers from attribute names ('none', 'tag', 'auto', 'plain'); *default:* **tag** |
| **--output.column-separator** {{DT_STR}} | Separator in CSV output; *default:* **;** |

The **--output.format** option is only being used when the format cannot be derived from the filename so it is mostly useful when writing to stdout or to a socket
or if for any reason you need give a file ending which is not "xml", "csv", "parquet", "xml.gz", or "csv.gz".

For **--output.compression** please note that this will currently only be applied to parquet files and this is an internal compression algorithm applied to every column.
So you will not be able to uncompress a parquet file with gzip compression using a gzip tool on the command line. Furthermore we do not support specifying different
compression methods for different columns yet. Compression support may also depend on the way SUMO (or rather the Arrow / Parquet library it depends on) has been compiled.

By default the column names are derived by connecting the element name with the attribute using "_". This can be changed by setting the option **--output.column-header** to one of the following values:
  - 'none': no header is written in CSV (parquet still uses the default header)
  - 'tag': the default behavior using element name and attribute name
  - 'auto': use only the attribute name unless there is a name clash. If there is a clash use the 'tag' strategy for the clashing column names (except for the first one).
  - 'plain': always use the attribute name only

# Parquet data types

For Parquet every column needs to have a data type. The current mapping is as follows:
  - If the given **--precision** is 2 (the default) or less, 32 bit floats are used for all floating point values, higher precision values trigger the use of 64 bit (double) values.
  - All coordinates in the plane ("x" and "y") use 64 bit floats (double).
  - Time values use 64 bit floats unless **--human-readable-time** is enabled then it uses strings.
  - All integers use 32 bit.
  - Everything else uses UTF8 strings.

# Performance

Here are some initial benchmarks using FCD output for the acosta scenario:

format|write|size | pandas.read | parse_fast_nested
------|-----:|--------: | ---: | ---:
fcd.xml    |12s|357MB | 11s | 8s
fcd.csv    |15s|194MB | 2s
fcd.parquet|13s| 74MB | 0.2s
fcd.xml.gz |19s| 56MB | 7s | 9s
fcd.csv.gz |17s| 47MB | 3s
