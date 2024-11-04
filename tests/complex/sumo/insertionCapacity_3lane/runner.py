#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
# Copyright (C) 2008-2024 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    runner.py
# @author  Jakob Erdmann
# @date    2022-01-04

from __future__ import absolute_import
from __future__ import print_function

import os
import subprocess
import sys

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
from sumolib import checkBinary  # noqa
from sumolib.xml import parse  # noqa


sumoBinary = checkBinary('sumo')


ROUTE_TEMPLATE = """
<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">
    <vType id="type1" sigma="%s" speedDev="%s"/>
    <flow id="flow_0" type="type1" begin="0.00" end="3600.00" number="10800" departSpeed="%s" departLane="%s">
        <route edges="E0"/>
    </flow>
</routes>
"""  # noqa


values = []
for stepLength in [1, 0.1]:
    for extrapolateDepartPos in ['false', 'true']:
        for sigma, speedDev in [(0.5, 0.1)]:
            for departSpeed in ['max', 'avg']:
                for departLane in ['first', 'random', 'free', 'best_prob']:
                    routes = ROUTE_TEMPLATE % (sigma, speedDev, departSpeed, departLane)
                    with open('input_routes.rou.xml', 'w') as rf:
                        rf.write(routes)
                    subprocess.call([sumoBinary,
                                     '-n', 'input_net.net.xml',
                                     '-r', 'input_routes.rou.xml',
                                     '--no-step-log',
                                     '--statistic-output', 'statistic.xml',
                                     '--max-depart-delay', '5',
                                     '-e', '3600',
                                     '--step-length', str(stepLength),
                                     '--extrapolate-departpos', extrapolateDepartPos])

                    inserted = list(parse('statistic.xml', 'vehicles'))[0].inserted
                    values.append((inserted, stepLength, extrapolateDepartPos, sigma, speedDev, departSpeed, departLane))

# write result table
with open('log.txt', 'w') as outf:
    outf.write("inserted stepLength exDP sigma speedDev departSpeed departLane\n")
    for record in values:
        outf.write(" ".join(map(str, record)) + "\n")

# write rank order
values.sort()
with open('log2.txt', 'w') as outf:
    outf.write("stepLength exDP sigma speedDev departSpeed departLane\n")
    for record in values:
        outf.write(" ".join(map(str, record[1:])) + "\n")
