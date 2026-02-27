#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
# Copyright (C) 2015-2026 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    visum_convertXMLRoutes.py
# @author  Jakob Erdmann
# @date    Nov 17 2025

"""
Parses a VISUM xml-route file and writes a SUMO route file
"""

from __future__ import print_function
from __future__ import absolute_import
import os
import sys
try:
    from functools import cache
except ImportError:
    # python < 3.9
    from functools import lru_cache as cache

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
import sumolib  # noqa
from sumolib.miscutils import openz  # noqa

MSG_CACHE = set()


def get_options(args=None):
    op = sumolib.options.ArgumentParser(description="Import VISUM route file")
    # input
    op.add_argument("-n", "--net-file", category="input", dest="netfile", required=True, type=op.net_file,
                    help="define the SUMO net file (mandatory)")
    op.add_argument("-r", "--route-file", category="input", dest="routefile", required=True, type=op.route_file,
                    help="define the input anm_route.xml file (mandatory)")
    # output
    op.add_argument("-o", "--output-trip-file", category="output", dest="outfile", required=True, type=op.route_file,
                    help="define the output route file")
    # processing
    op.add_argument("--scale", metavar="FLOAT", type=float, default=1,
                    help="Scale volume by the given value (i.e. 24 when volume denotes hourly rather than daily traffic)")  # noqa
    op.add_argument("--vclass", help="Only include routes for the given vclass")
    op.add_argument("-a", "--attributes", default="",
                    help="additional flow attributes.")
    op.add_argument("--trips", action="store_true", default=False, category="processing",
                    help="write unvalidated trips instead of routes")

    options = op.parse_args(args=args)
    return options


def append_no_duplicate(edges, e):
    if edges and edges[-1] == e:
        return
    edges.append(e)


@cache
def repairEdgeEdge(net, prevEdge, edge, vClass):
    if edge in prevEdge.getAllowedOutgoing(vClass):
        return [prevEdge, edge], None
    return net.getFastestPath(prevEdge, edge, vClass=vClass)


@cache
def repairEdgeNode(net, prevEdge, node, vClass):
    bestPath = None
    bestCost = 1e400
    for edge in node.getIncoming():
        path, cost = net.getFastestPath(prevEdge, edge, vClass=vClass)
        if path and cost < bestCost:
            bestPath = path
            bestCost = cost
    return bestPath, bestCost


@cache
def repairNodeNode(net, prevNode, node, vClass):
    bestPath = None
    bestCost = 1e400
    for prevEdge in prevNode.getOutgoing():
        for edge in node.getIncoming():
            path, cost = net.getFastestPath(prevEdge, edge, vClass=vClass)
            if path and cost < bestCost:
                bestPath = path
                bestCost = cost
    return bestPath, bestCost


@cache
def repairNodeEdge(net, prevNode, edge, vClass):
    bestPath = None
    bestCost = 1e400
    for prevEdge in prevNode.getOutgoing():
        path, cost = net.getFastestPath(prevEdge, edge, vClass=vClass)
        if path and cost < bestCost:
            bestPath = path
            bestCost = cost
    return bestPath, bestCost


def getValidNodes(net, nodes):
    """obtain a stream of nodes that exist in the network"""
    for nodeID in nodes:
        if ' ' in nodeID:
            nodeID, id2 = nodeID.split()
            if id2[-1] == 'B':
                continue
            elif id2[-1] == 'A':
                if net.hasNode(nodeID):
                    yield net.getNode(nodeID)
                edgeID = id2[:-1]
                if net.hasEdge(edgeID):
                    yield net.getEdge(edgeID).getToNode()
            else:
                if net.hasNode(nodeID):
                    yield net.getNode(nodeID)
                if net.hasNode(id2):
                    yield net.getNode(id2)
        else:
            if net.hasNode(nodeID):
                yield net.getNode(nodeID)


def getValidEdgesNodes(edgedict, validNodes):
    """obtain stream of edges intermixed with nodes that could not be assigned to edges"""
    singleNodes = []
    for node in validNodes:
        if singleNodes:
            e = edgedict.get((singleNodes[-1], node))
            if e is not None:
                singleNodes.pop()
                yield singleNodes, e
                singleNodes = []
            else:
                singleNodes.append(node)
        else:
            singleNodes.append(node)
    if singleNodes:
        yield singleNodes, None


def msgOnce(msg, key, file):
    if key not in MSG_CACHE:
        print(msg, file=file)
        MSG_CACHE.add(key)


def getAllowedEdge(validEdgesNodes, vClass):
    for singleNodes, edge in validEdgesNodes:
        if edge is not None and edge.allows(vClass):
            return edge
        for n in singleNodes:
            for edge2 in n.getOutgoing():
                if edge2.allows(vClass):
                    return edge2

def getTripEdges(validEdgesNodes, vClass):
    result = [getAllowedEdge(validEdgesNodes, vClass),
              getAllowedEdge(reversed(validEdgesNodes), vClass)]
    if any(e is None for e in result):
        return None
    else:
        return result


def getConnectedEdges(net, validEdgesNodes, vClass, routeID):
    """obtain a sequence of connected edges"""
    result = []
    lastEdge = None
    lastNode = None
    msgSuccess = "Route %s:" % routeID + " Repaired path between %s, length %s, cost %s"
    msgFail = "Route %s:" % routeID + " Found no path between %s"
    for singleNodes, edge in validEdgesNodes:
        for n in singleNodes:
            if lastEdge:
                path, cost = repairEdgeNode(net, lastEdge, n, vClass)
                between = "edge %s and node %s" % (lastEdge.getID(), n.getID())
                if path:
                    if len(path) > 2:
                        msgOnce(msgSuccess % (between, len(path), cost), between, sys.stderr)
                    result += path[1:]
                    lastEdge = path[-1]
                else:
                    print(msgFail % between, file=sys.stderr)
                    return None
            elif lastNode:
                path, cost = repairNodeNode(net, lastNode, n, vClass)
                between = "node %s and node %s" % (lastNode.getID(), n.getID())
                if path:
                    msgOnce(msgSuccess % (between, len(path), cost), between, sys.stderr)
                    result += path
                    lastEdge = path[-1]
                else:
                    print(msgFail % between, file=sys.stderr)
                    return None
            else:
                lastNode = n
        if edge is not None:
            if lastEdge:
                path, cost = repairEdgeEdge(net, lastEdge, edge, vClass)
                between = "edge %s and edge %s" % (lastEdge.getID(), edge.getID())
                if path:
                    if len(path) > 2:
                        msgOnce(msgSuccess % (between, len(path), cost), between, sys.stderr)
                    result += path[1:]
                    lastEdge = path[-1]
                else:
                    print(msgFail % between, file=sys.stderr)
                    return None
            elif lastNode:
                path, cost = repairNodeEdge(net, lastNode, edge, vClass)
                between = "node %s and edge %s" % (lastNode.getID(), edge.getID())
                if path:
                    msgOnce(msgSuccess % (between, len(path), cost), between, sys.stderr)
                    result += path
                    lastEdge = path[-1]
                else:
                    print(msgFail % between, file=sys.stderr)
                    return None
            else:
                result.append(edge)
                lastEdge = edge

    return result


def main(options):
    vTypes = dict()
    nSkipped = 0
    nBroken = 0
    nDisallowed = 0
    nZeroFlows = 0
    nZeroRoutes = 0
    net = sumolib.net.readNet(options.netfile)
    allowed = set()
    if options.vclass:
        allowed.add(None)
        for e in net.getEdges():
            if e.allows(options.vclass):
                allowed.add(e)

    edgedict = {}  # (from,to) -> edge
    for e in net.getEdges():
        edgedict[e.getFromNode(), e.getToNode()] = e

    with openz(options.outfile, 'w') as fout:
        sumolib.writeXMLHeader(fout, "$Id$", "routes", options=options)
        for vtype in sumolib.xml.parse_fast(options.routefile, 'VEHTYPETI',
                                            ['INDEX', 'FROMTIME', 'TOTIME', 'VEHTYPEID']):
            vTypes[vtype.INDEX] = (vtype.VEHTYPEID, vtype.FROMTIME, vtype.TOTIME)
            fout.write('    <vType id="%s"/>\n' % vtype.VEHTYPEID)

        nested = {
            'ITEM': ['NODE'],
            'DEMAND': ['VTI', 'VOLUME']}
        for route in sumolib.xml.parse_fast_structured(options.routefile, 'ROUTE', ['INDEX'], nested):
            nodes = [i.NODE for i in route.ITEM]
            validNodes = list(getValidNodes(net, nodes))
            if len(validNodes) < 2:
                nSkipped += 1
                continue
            validEdgesNodes = list(getValidEdgesNodes(edgedict, validNodes))
            if options.vclass:
                if any([e not in allowed for s, e in validEdgesNodes]):
                    nDisallowed += 1
                    continue
            if options.trips:
                edges = getTripEdges(validEdgesNodes, options.vclass)
            else:
                edges = getConnectedEdges(net, validEdgesNodes, options.vclass, route.INDEX)
            if not edges:
                nBroken += 1
                continue

            totalVolume = 0
            for demand in route.DEMAND:
                totalVolume += float(demand.VOLUME)
            if totalVolume == 0:
                nZeroRoutes += 1
                continue

            edgeIDs = [e.getID() for e in edges]

            if not options.trips:
                fout.write('    <route id="%s" edges="%s"/>\n' % (route.INDEX, ' '.join(edgeIDs)))
            for demand in route.DEMAND:
                flowID = "%s_%s" % (route.INDEX, demand.VTI)
                vtype, begin, end = vTypes[demand.VTI]
                #  assume VOLUME is per day
                rate = float(demand.VOLUME) * options.scale / (3600 * 24)
                if rate > 0:
                    attrs = ""
                    if options.attributes:
                        attrs = " " + options.attributes
                    if options.trips:
                        fromToRoute = ' from="%s" to="%s"' % (edges[0].getID(), edges[-1].getID())
                    else:
                        fromToRoute = ' route="%s"' % route.INDEX
                    fout.write('    <flow id="%s"%s type="%s" begin="%s" end="%s" period="exp(%s)"%s/>\n' % (
                        flowID, fromToRoute, vtype, begin, end, rate, attrs))
                else:
                    nZeroFlows += 1
        fout.write("</routes>\n")

        if nSkipped > 0:
            print("Warning: Skipped %s routes because they were defined with a single node" % nSkipped)
        if nBroken > 0:
            print("Warning: Skipped %s routes because they could not be repaired" % nBroken)
        if nZeroRoutes > 0:
            print("Warning: Skipped %s routes because they have no volume" % nZeroRoutes)
        if nZeroFlows > 0:
            print("Warning: Skipped %s flows because they have no volume" % nZeroFlows)
        if nDisallowed > 0:
            print("Warning: Ignored %s routes because they have edges that are not allowed for %s " % (
                nDisallowed, options.vclass))


if __name__ == "__main__":
    main(get_options())
