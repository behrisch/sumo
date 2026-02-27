#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
# Copyright (C) 2009-2026 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    remap_renamed.py
# @author  Jakob Erdmann
# @date    2025-02-26

from __future__ import print_function
from __future__ import absolute_import
import os
import sys
from collections import defaultdict

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import sumolib  # noqa
from sumolib.xml import parse  # noqa
from sumolib.net import lane2edge, lane2index  # noqa


def get_options(args=None):
    ap = sumolib.options.ArgumentParser(description="Remap routes, infrastructure or selections from one network to another based on the 'origId' attribute")  # noqa
    # ap.add_argument("--orig-net", dest="origNet", required=False, category="input", type=ap.net_file,
    #                 help="SUMO network for loading infrastructure", metavar="FILE")
    ap.add_argument("-n", "--net-file", dest="targetNet", required=True, category="input", type=ap.net_file,
                    help="SUMO network with renamed ids", metavar="FILE")
    gp = ap.add_mutually_exclusive_group(required=True)
    gp.add_argument("-r", "--route-file", dest="routes", category="input", type=ap.route_file,
                    help="File for reading routes", metavar="FILE")
    gp.add_argument("-a", "--additional-file", dest="addfile", category="input", type=ap.route_file,
                    help="File for reading additional elements", metavar="FILE")
    gp.add_argument("-s", "--selection-file", dest="selfile", category="input", type=ap.file,
                    help="File for reading a selection file with edges, lanes or junctions", metavar="FILE")
    ap.add_argument("-o", "--output-file", dest="output", required=True, category="output", type=ap.additional_file,
                    help="File for writing output", metavar="FILE")
    ap.add_argument("-v", "--verbose", action="store_true", dest="verbose",
                    default=False, help="tell me what you are doing")
    options = ap.parse_args()
    return options


MISSING = defaultdict(lambda: 0)
MISSING_JUNCTIONS = defaultdict(lambda: 0)


def get_mapper(lookup_attr, missing):
    def mapper(options, origID):
        newID = getattr(options, lookup_attr).get(origID)
        if not newID:
            missing[origID] += 1
        return newID
    return mapper


remap_edge = get_mapper('lookup', MISSING)
remap_junction = get_mapper('junction_lookup', MISSING_JUNCTIONS)


def remap_lane(options, origID):
    origEdge, index = origID.rsplit("_", 1)
    newEdge = remap_edge(options, origEdge)
    if newEdge:
        return "%s_%s" % (newEdge, index)
    else:
        return None


def remap_edges(options, origIDs):
    newEdges = [remap_edge(options, e) for e in origIDs.split()]
    if None in newEdges:
        return None
    else:
        return ' '.join(newEdges)


def remap_lanes(options, origIDs):
    newLanes = [remap_lane(options, lane) for lane in origIDs.split()]
    if None in newLanes:
        return None
    else:
        return ' '.join(newLanes)


IDS = {
    'edge': remap_edge,
    'attr_from': remap_edge,
    'to': remap_edge,
    'edges': remap_edges,
    'via': remap_edges,
    'fromJunction': remap_junction,
    'toJunction': remap_junction,
    'lane': remap_lane,
    'lanes': remap_lanes,
}


def remap(options, obj):
    success = True
    for attr, mapper in IDS.items():
        if obj.hasAttribute(attr):
            id1 = getattr(obj, attr)
            id2 = mapper(options, id1)
            if id2:
                obj.setAttribute(attr, id2)
            else:
                print("Could not map %s attribute %s '%s'" % (
                    obj.name, attr, id1),
                    file=sys.stderr)
                success = False

    for child in obj.getChildList():
        success &= remap(options, child)
    return success


def addOrigId(lookup, origId, obj):
    if origId is not None:
        if " " in origId:
            print("multi-part origId not supported (%s '%s' origId '%s')" % (
                type(obj), obj.getID(), origId), file=sys.stderr)
        else:
            lookup[origId] = obj.getID()


def build_lookup(net):
    lookup = {}
    for edge in net.getEdges():
        lane = edge.getLanes()[0]
        origId = lane.getParam("origId")
        addOrigId(lookup, origId, edge)
    return lookup


def build_junction_lookup(net):
    lookup = {}
    for node in net.getNodes():
        origId = node.getParam("origId")
        addOrigId(lookup, origId, node)
    return lookup


def mapSelection(options):
    types = (("edge:", lambda x: options.lookup[x]),
             ("junction:", lambda x: options.junction_lookup[x]),
             ("lane:", lambda x: options.lookup[lane2edge(x)] + "_" + str(lane2index(x))))
    with open(options.output, 'w') as fout, open(options.selfile) as infile:
        for line in infile:
            line = line.strip()
            ignored = True
            for prefix, fun in types:
                if line.startswith(prefix):
                    fout.write(prefix + fun(line[len(prefix):]) + '\n')
                    ignored = False
                    break
            if ignored:
                print("Warning: Could not map '%s'" % line, file=sys.stderr)


def main(options):
    if options.verbose:
        print("Reading target-net '%s'" % options.targetNet)
    options.net2 = sumolib.net.readNet(options.targetNet)
    options.lookup = build_lookup(options.net2)
    options.junction_lookup = build_junction_lookup(options.net2)

    if options.selfile:
        mapSelection(options)
    else:
        if options.routes:
            infile = options.routes
            tag = "routes"
        else:
            infile = options.addfile
            tag = "additional"

        with open(options.output, 'w') as fout:
            sumolib.writeXMLHeader(fout, "$Id$", tag, options=options)
            for obj in parse(infile):
                if remap(options, obj):
                    fout.write(obj.toXML(initialIndent=" " * 4, withComments="inline"))
                else:
                    fout.write("    <!--" + obj.toXML()[1:-2] + "-->\n")
            fout.write("</%s>\n" % tag)

    if MISSING:
        print("Could not map %s edges (%s total occurences)" % (
            len(MISSING), sum(MISSING.values())), file=sys.stderr)
        vk = sorted([(v, k) for k, v in MISSING.items()], reverse=True)
        print("Most frequent missing edges:")
        print("\n".join(["%s %s" % (k, v) for (v, k) in vk]))


if __name__ == "__main__":
    main(get_options())
