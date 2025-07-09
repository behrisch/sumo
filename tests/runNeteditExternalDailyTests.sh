#!/bin/bash
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

# Bash script for the daily netedit test run.
# sets environment variables respecting SUMO_HOME and starts texttest for all existing netedit daily suites

if test `uname` = "Darwin"; then #macOS specific exports
  export LC_ALL=C
  export LANG=C
fi

pushd `dirname $0`
export TEXTTEST_HOME="$PWD"
if test x"$SUMO_HOME" = x; then
  cd ..
  export SUMO_HOME="$PWD"
fi
popd
export NETEDIT_BINARY="$SUMO_HOME/bin/netedit"

# Monday
if [ "$(date +%u)" -eq 1 ]; then
    for f in \
    "$TEXTTEST_HOME/netedit/testsuite.netedit.external.daily.basic" \
    "$TEXTTEST_HOME/netedit/testsuite.netedit.external.daily.elements.bugs" \
    "$TEXTTEST_HOME/netedit/testsuite.netedit.external.daily.elements.datas" \
    "$TEXTTEST_HOME/netedit/testsuite.netedit.external.daily.elements.shapes" \
    "$TEXTTEST_HOME/netedit/testsuite.netedit.external.daily.elements.tazs"
  do
    base=$(basename "$f")
    texttest -a "${base:10}" "$@" &
  done
fi

# Tuesday
if [ "$(date +%u)" -eq 2 ]; then
  for f in $TEXTTEST_HOME/netedit/testsuite.netedit.external.daily.network.*; do
    base=`basename $f`
    texttest -a ${base:10} "$@" &
  done
fi

# Wednesday
if [ "$(date +%u)" -eq 3 ]; then
  for f in $TEXTTEST_HOME/netedit/testsuite.netedit.external.daily.elements.additionals.*; do
    base=`basename $f`
    texttest -a ${base:10} "$@" &
  done
fi

# Thursday
if [ "$(date +%u)" -eq 4 ]; then
  for f in $TEXTTEST_HOME/netedit/testsuite.netedit.external.daily.elements.demands.*; do
    base=`basename $f`
    texttest -a ${base:10} "$@" &
  done
fi

wait
