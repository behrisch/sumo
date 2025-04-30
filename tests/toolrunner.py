#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
# Copyright (C) 2008-2025 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    toolrunner.py
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @date    2008-03-29

import os
import subprocess
import sys
if len(sys.argv) < 2:
    sys.exit('required argument <tool> missing')
idx = len(sys.argv) - 1
while idx > 0 and sys.argv[idx][0] == "-":
    idx -= 1
for i, a in enumerate(sys.argv[1:]):
    if a.endswith(".py") or a.endswith(".jar"):
        idx = i + 1
        break
tool = [os.path.join(os.path.dirname(sys.argv[0]), "..", sys.argv[idx])]
del sys.argv[idx]
if tool[0].endswith(".jar"):
    tool = ["java", "-jar"] + tool

if tool[0].endswith(".py"):
    python = os.environ.get('PYTHON', 'python')
    print(sys.executable, python, file=sys.stderr)
    tool = ([python] + tool) if python.endswith("2") else ([python, "-Wd"] + tool)
subprocess.call(tool + sys.argv[1:], env=os.environ, stdout=sys.stdout, stderr=sys.stderr)
