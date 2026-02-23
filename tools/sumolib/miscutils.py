# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
# Copyright (C) 2012-2026 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    miscutils.py
# @author  Jakob Erdmann
# @author  Michael Behrisch
# @author  Mirko Barthauer
# @date    2012-05-08

from __future__ import absolute_import
from __future__ import print_function
from __future__ import division
import sys
import time
import os
import math
import colorsys
import socket
import random
import gzip
import codecs
import io
from types import ModuleType, FunctionType
from gc import get_referents
try:
    from urllib.request import urlopen
except ImportError:
    from urllib import urlopen
# needed for backward compatibility
from .statistics import Statistics, geh, uMax, uMin, round  # noqa


_BLACKLIST = type, ModuleType, FunctionType


def get_size(obj):
    """sum size of object & members.
    lifted from https://stackoverflow.com/a/30316760
    """
    if isinstance(obj, (_BLACKLIST)):
        raise TypeError('getsize() does not take argument of type: ' + str(type(obj)))
    seen_ids = set()
    size = 0
    objects = [obj]
    while objects:
        need_referents = []
        for obj in objects:
            if not isinstance(obj, _BLACKLIST) and id(obj) not in seen_ids:
                seen_ids.add(id(obj))
                size += sys.getsizeof(obj)
                need_referents.append(obj)
        objects = get_referents(*need_referents)
    return size


def benchmark(func):
    """
    decorator for timing a function
    """
    def benchmark_wrapper(*args, **kwargs):
        started = time.time()
        now = time.strftime("%a, %d %b %Y %H:%M:%S +0000", time.localtime())
        print('function %s called at %s' % (func.__name__, now))
        sys.stdout.flush()
        result = func(*args, **kwargs)
        print('function %s finished after %f seconds' %
              (func.__name__, time.time() - started))
        sys.stdout.flush()
        return result
    return benchmark_wrapper


class Benchmarker:
    """
    class for benchmarking a function using a "with"-statement.
    Preferable over the "benchmark" function for the following use cases
    - benchmarking a code block that isn't wrapped in a function
    - benchmarking a function only in some calls
    """

    def __init__(self, active, description):
        self.active = active
        self.description = description

    def __enter__(self):
        self.started = time.time()

    def __exit__(self, *args):
        if self.active:
            duration = time.time() - self.started
            print("%s finished after %s" % (self.description, humanReadableTime(duration)))


class working_dir:
    """
    temporarily change working directory using 'with' statement
    """

    def __init__(self, dir):
        self.dir = dir
        self.origdir = os.getcwd()

    def __enter__(self):
        os.chdir(self.dir)

    def __exit__(self, type, value, traceback):
        os.chdir(self.origdir)


class Colorgen:
    DISTINCT = [
        (0.17, 1.0, 0.5),
        (0.0, 0.9, 1.0),
        (0.35, 0.67, 0.71),
        (0.14, 0.9, 1.0),
        (0.56, 1.0, 0.78),
        (0.07, 0.8, 0.96),
        (0.79, 0.83, 0.71),
        (0.5, 0.71, 0.94),
        (0.84, 0.79, 0.94),
        (0.2, 0.76, 0.96),
        (0.0, 0.24, 0.98),
        (0.5, 1.0, 0.5),
        (0.77, 0.25, 1.0),
        (0.09, 0.76, 0.67),
        (0.15, 0.22, 1.0),
        (0.0, 1.0, 0.5),
        (0.38, 0.33, 1.0),
        (0.67, 1.0, 0.5),
    ]

    def __init__(self, hsv, cycleLength=10.67):
        self.hsv = hsv
        self.cycle = [int(random.random() * 256) for x in self.hsv]
        self.cycleOffset = int(round(256 / cycleLength))
        self.distinctIndex = 0

    def get_value(self, opt, index):
        if opt == 'random':
            return random.random()
        if opt == 'cycle':
            # the 255 below is intentional to get all color values when cycling long enough
            self.cycle[index] = (self.cycle[index] + self.cycleOffset) % 255
            return self.cycle[index] / 255.0
        if opt == 'distinct':
            if index == 0:
                self.distinctIndex = (self.distinctIndex + 1) % len(self.DISTINCT)
            return self.DISTINCT[self.distinctIndex][index]
        return float(opt)

    def floatTuple(self):
        """return color as a tuple of floats each in [0,1]"""
        return colorsys.hsv_to_rgb(*[self.get_value(o, i) for i, o in enumerate(self.hsv)])

    def byteTuple(self):
        """return color as a tuple of bytes each in [0,255]"""
        return tuple([int(round(255 * x)) for x in self.floatTuple()])

    def __call__(self):
        """return constant or randomized rgb-color string"""
        return ','.join(map(str, self.byteTuple()))


class priorityDictionary(dict):

    def __init__(self):
        '''Initialize priorityDictionary by creating binary heap
            of pairs (value,key).  Note that changing or removing a dict entry will
            not remove the old pair from the heap until it is found by smallest() or
            until the heap is rebuilt.'''
        self.__heap = []
        dict.__init__(self)

    def smallest(self):
        '''Find smallest item after removing deleted items from heap.'''
        if len(self) == 0:
            raise IndexError("smallest of empty priorityDictionary")
        heap = self.__heap
        while heap[0][1] not in self or self[heap[0][1]] != heap[0][0]:
            lastItem = heap.pop()
            insertionPoint = 0
            while 1:
                smallChild = 2 * insertionPoint + 1
                if smallChild + 1 < len(heap) and \
                        heap[smallChild][0] > heap[smallChild + 1][0]:
                    smallChild += 1
                if smallChild >= len(heap) or lastItem <= heap[smallChild]:
                    heap[insertionPoint] = lastItem
                    break
                heap[insertionPoint] = heap[smallChild]
                insertionPoint = smallChild
        return heap[0][1]

    def __iter__(self):
        '''Create destructive sorted iterator of priorityDictionary.'''
        def iterfn():
            while len(self) > 0:
                x = self.smallest()
                yield x
                del self[x]
        return iterfn()

    def __setitem__(self, key, val):
        '''Change value stored in dictionary and add corresponding
            pair to heap.  Rebuilds the heap if the number of deleted items grows
            too large, to avoid memory leakage.'''
        dict.__setitem__(self, key, val)
        heap = self.__heap
        if len(heap) > 2 * len(self):
            self.__heap = [(v, k) for k, v in self.items()]
            self.__heap.sort()  # builtin sort likely faster than O(n) heapify
        else:
            newPair = (val, key)
            insertionPoint = len(heap)
            heap.append(None)
            while insertionPoint > 0 and val < heap[(insertionPoint - 1) // 2][0]:
                heap[insertionPoint] = heap[(insertionPoint - 1) // 2]
                insertionPoint = (insertionPoint - 1) // 2
            heap[insertionPoint] = newPair

    def setdefault(self, key, val):
        '''Reimplement setdefault to call our customized __setitem__.'''
        if key not in self:
            self[key] = val
        return self[key]

    def update(self, other):
        for key in other.keys():
            self[key] = other[key]


def getFreeSocketPort(numTries=10):
    for _ in range(numTries):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(('', 0))
            p = s.getsockname()[1]
            s.close()
            return p
        except socket.error:
            pass
    return None


def getSocketStream(port, mode='rb'):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("localhost", port))
    s.listen(1)
    conn, _ = s.accept()
    return conn.makefile(mode)


# euclidean distance between two coordinates in the plane
def euclidean(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def humanReadableTime(seconds):
    result = ""
    sign = '-' if seconds < 0 else ''
    seconds = abs(seconds)
    ds = 3600 * 24
    if seconds > ds:
        result = "%s:" % int(seconds / ds)
        seconds = seconds % ds
    result += "%02i:" % int(seconds / 3600)
    seconds = seconds % 3600
    result += "%02i:" % int(seconds / 60)
    seconds = seconds % 60
    if seconds == int(seconds):
        seconds = int(seconds)
    result += "%02i" % seconds
    return sign + result


SPECIAL_TIME_STRINGS = ["triggered", "containerTriggered", "split", "begin"]


def parseTime(t, factor=1):
    try:
        return float(t) * factor
    except ValueError:
        pass
    try:
        # prepended zero is ignored if the date value already contains days
        days, hours, minutes, seconds = ([0] + list(map(float, t.split(':'))))[-4:]
        sign = -1 if t.strip()[0] == '-' else 1
        return (3600 * 24 * days + 3600 * hours + 60 * minutes + seconds) * sign * factor
    except ValueError:
        if t in SPECIAL_TIME_STRINGS:
            # signal special case but don't crash
            return None
        else:
            raise


def parseBool(val):
    # see data/xsd/baseTypes:boolType
    return val in ["true", "True", "x", "1", "yes", "on"]


def getFlowNumber(flow):
    """interpret number of vehicles from a flow parsed by sumolib.xml.parse"""
    if flow.number is not None:
        return int(flow.number)
    if flow.end is not None:
        duration = parseTime(flow.end) - parseTime(flow.begin)
        period = 0
        if flow.period is not None:
            if 'exp' in flow.period:
                # use expected value
                period = 1 / float(flow.period[4:-1])
            else:
                period = float(flow.period)
        for attr in ['perHour', 'vehsPerHour']:
            if flow.hasAttribute(attr):
                period = 3600 / float(flow.getAttribute(attr))
        if period > 0:
            return math.ceil(duration / period)
        else:
            return 1


def intIfPossible(val):
    if int(val) == val:
        return int(val)
    else:
        return val


def openz(fileOrURL, mode="r", **kwargs):
    """
    Opens transparently files, URLs and gzipped files for reading and writing.
    Special file names "stdout" and "stderr" are handled as well.
    Also enforces UTF8 on text output / input and should handle BOMs in input.
    Should be compatible with python 2 and 3.
    """
    encoding = kwargs.get("encoding", "utf8" if "w" in mode else "utf-8-sig")
    try:
        if fileOrURL.startswith("http://") or fileOrURL.startswith("https://"):
            return io.BytesIO(urlopen(fileOrURL).read())
        if fileOrURL == "stdout":
            return sys.stdout
        if fileOrURL == "stderr":
            return sys.stderr
        if fileOrURL.endswith(".gz") and "w" in mode:
            if "b" in mode:
                return gzip.open(fileOrURL, mode="w")
            return gzip.open(fileOrURL, mode="wt", encoding=encoding)
        if kwargs.get("trySocket") and fileOrURL.isdigit():
            return getSocketStream(int(fileOrURL), mode)
        if kwargs.get("tryGZip", True) and "r" in mode:
            with gzip.open(fileOrURL) as fd:
                fd.read(1)
            if "b" in mode:
                return gzip.open(fileOrURL)
            if sys.version_info[0] < 3:
                return codecs.getreader('utf-8')(gzip.open(fileOrURL))
            return gzip.open(fileOrURL, mode="rt", encoding=encoding)
    except OSError as e:
        if kwargs.get("printErrors"):
            print(e, file=sys.stderr)
    except IOError as e:
        if kwargs.get("printErrors"):
            print(e, file=sys.stderr)
    if "b" in mode:
        return io.open(fileOrURL, mode=mode)
    return io.open(fileOrURL, mode=mode, encoding=encoding)


def short_names(filenames, noEmpty):
    if len(filenames) == 1:
        return filenames
    reversedNames = [''.join(reversed(f)) for f in filenames]
    prefix = os.path.commonprefix(filenames)
    suffix = os.path.commonprefix(reversedNames)
    prefixLen = len(prefix)
    suffixLen = len(suffix)
    shortened = [f[prefixLen:-suffixLen] for f in filenames]
    if noEmpty and any([not f for f in shortened]):
        # make longer to avoid empty file names
        base = os.path.basename(prefix)
        shortened = [base + f for f in shortened]
    return shortened
