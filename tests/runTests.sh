#!/bin/bash
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

# Bash script for the test run.
# Sets environment variables respecting SUMO_HOME and starts texttest.

if test `uname` = "Darwin"; then  # macOS specific exports
  export LC_ALL=C
  export LANG=C
fi

pushd `dirname $0` > /dev/null
export TEXTTEST_HOME="$PWD"
shopt -s nullglob  # expand the pattern to an empty list if no env exists
for i in *env/bin/activate; do
  if test x"$VIRTUAL_ENV" = x; then
    echo "Activating virtual environment $(dirname $(dirname $i))."
    source $i
  else
    echo "Virtual environment $VIRTUAL_ENV already active, ignoring $(dirname $(dirname $i))."
  fi
done
if test x"$SUMO_HOME" = x; then
  cd ..
  export SUMO_HOME="$PWD"
fi
popd > /dev/null
export ACTIVITYGEN_BINARY="$SUMO_HOME/bin/activitygen"
export DFROUTER_BINARY="$SUMO_HOME/bin/dfrouter"
export DUAROUTER_BINARY="$SUMO_HOME/bin/duarouter"
export JTRROUTER_BINARY="$SUMO_HOME/bin/jtrrouter"
export NETCONVERT_BINARY="$SUMO_HOME/bin/netconvert"
export NETEDIT_BINARY="$SUMO_HOME/bin/netedit"
export NETGENERATE_BINARY="$SUMO_HOME/bin/netgenerate"
export OD2TRIPS_BINARY="$SUMO_HOME/bin/od2trips"
export POLYCONVERT_BINARY="$SUMO_HOME/bin/polyconvert"
export SUMO_BINARY="$SUMO_HOME/bin/sumo"
export GUISIM_BINARY="$SUMO_HOME/bin/sumo-gui"
export MAROUTER_BINARY="$SUMO_HOME/bin/marouter"
export PYTHON="python"
export LANG="C"

texttest "$@"
