#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
# Copyright (C) 2017-2024 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    setup-libsumo.py
# @author  Benjamin Striner
# @author  Michael Behrisch
# @date    2017-01-26


import os
from skbuild import setup
import version

_, SUMO_VERSION = "", "1.21.0"
package_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

setup(
    name='libsumo',
    version=SUMO_VERSION,
    url='https://sumo.dlr.de/docs/Libsumo.html',
    author='DLR and contributors',
    author_email='sumo@dlr.de',
    license='EPL-2.0',
    description="The python version of the libsumo API to communicate with the traffic simulation Eclipse SUMO",
    long_description=open(os.path.join(os.path.dirname(package_dir), 'README.md')).read(),
    long_description_content_type='text/markdown',

    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: Eclipse Public License 2.0 (EPL-2.0)',
        'Programming Language :: Python :: 3',
    ],
    keywords='traffic simulation traci sumo',

    packages=['libsumo'],
    package_dir={'': package_dir},
    package_data={'libsumo': ['*.pyd', '*.so', '*.dylib']},
    install_requires=['traci>='+SUMO_VERSION, 'eclipse-sumo>='+SUMO_VERSION],
)
