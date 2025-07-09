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

# @file    build_wheels.sh
# @author  Michael Behrisch
# @author  Robert Hilbrich
# @date    2019

# This script builds the wheels inside a docker container.
# An example call could look like: docker run --rm -v $PWD:/opt/sumo --workdir /opt/sumo manylinux2014_aarch64 tools/build_config/build_wheels.sh
# If we are behind a firewall (HTTPS_PROXY is set) we cannot install from the CentOS repos but we can use pip via the proxy
if test $# -ge 1; then
    export HTTPS_PROXY=$1
fi
if test -z $HTTPS_PROXY; then
    yum install -y epel-release
    yum-config-manager --add-repo=https://download.opensuse.org/repositories/science:/dlr/CentOS_7/
    yum install -y --nogpgcheck ccache libxerces-c-devel proj-devel fox16-devel bzip2-devel gl2ps-devel swig3 eigen3-devel
    yum install -y https://download.postgresql.org/pub/repos/yum/reporpms/EL-7-x86_64/pgdg-redhat-repo-latest.noarch.rpm
    yum install -y geos311-devel
    yum install -y https://packages.apache.org/artifactory/arrow/centos/$(cut -d: -f5 /etc/system-release-cpe | cut -d. -f1)/apache-arrow-release-latest.rpm
    yum install -y arrow-devel parquet-devel # For Apache Parquet
    pipx install -f patchelf==0.16.1.0  # see https://github.com/pypa/manylinux/issues/1421
fi

if test $# -ge 2; then
    pushd ..
    git clone https://github.com/PedestrianDynamics/jupedsim -b $2 --depth 1
    mkdir jupedsim-build jupedsim-install
    cd jupedsim-build
    cmake -DCMAKE_INSTALL_PREFIX=$PWD/../jupedsim-install ../jupedsim
    cmake --build . -j$(nproc)
    cmake --install .
    popd
fi

mkdir -p $HOME/.ccache
echo "hash_dir = false" >> $HOME/.ccache/ccache.conf
echo "base_dir = $PWD/_skbuild/linux-x86_64-3.8" >> $HOME/.ccache/ccache.conf
cp build_config/pyproject.toml .
py=/opt/python/cp312-cp312
$py/bin/python tools/build_config/version.py tools/build_config/setup-sumo.py ./setup.py
$py/bin/python -m build --wheel
mv dist/eclipse_sumo-* `echo dist/eclipse_sumo-* | sed 's/cp312-cp312/py2.py3-none/'`
auditwheel repair dist/eclipse_sumo*.whl
cp -a data tools/libsumo
for py in /opt/python/cp3[1789]*; do
    rm dist/*.whl
    pminor=`echo $py | sed 's,/opt/python/cp3,,;s/-.*//'`
    echo "base_dir = $PWD/_skbuild/linux-x86_64-3.${pminor}" >> $HOME/.ccache/ccache.conf
    $py/bin/python tools/build_config/version.py tools/build_config/setup-sumo.py ./setup.py
    $py/bin/python -m build --wheel
    $py/bin/python tools/build_config/version.py tools/build_config/setup-libsumo.py tools/setup.py
    $py/bin/python -m build --wheel tools -o dist
    auditwheel repair dist/libsumo*.whl
done
# clean up all temporary files except for _skbuild
rm -rf tools/*.egg-info tools/build_config/*.egg-info tools/build tools/libsumo/data pyproject.toml setup.py tools/setup.py data/locale
find tools -name __pycache__ | xargs rm -rf
# the resulting wheels are in wheelhouse but we keep also the dist and the _skbuild dir for inspection
# we make everything writable so that others can clean up
chmod -R a+w _skbuild dist wheelhouse
