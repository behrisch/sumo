#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
# Copyright (C) 2010-2025 German Aerospace Center (DLR) and others.
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
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @author  Mirko Barthauer
# @date    2010-01-30

"""
This script runs the gaming GUI for the LNdW traffic light game.
It checks for possible scenarios in the current working directory
and lets the user start them as a game. Furthermore it
saves highscores to local disc and to the central highscore server.
"""
from __future__ import absolute_import
from __future__ import print_function
import os
import subprocess
import sys
import re
import pickle
import glob
import time
try:
    import Tkinter
except ImportError:
    import tkinter as Tkinter
from xml.dom import pulldom
from collections import defaultdict

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
SUMO_HOME = os.environ.get('SUMO_HOME', os.path.join(_THIS_DIR, '..', '..'))
sys.path.append(os.path.join(SUMO_HOME, 'tools'))
import sumolib  # noqa
from sumolib.translation import addLanguageOption, setLanguage, TL  # noqa

_UPLOAD = False if "noupload" in sys.argv else True
_SCOREFILE = "scores.pkl"
REFSCOREFILE = "refscores.pkl"
if _UPLOAD:
    _TIMEOUT = 5
    _SCORESERVER = "sumo.dlr.de"
    _SCORESCRIPT = "/scores.php?game=TLS&"
_DEBUG = True if "debug" in sys.argv else False
_SCORES = 30
BASE = os.path.dirname(sys.argv[0])
_LANGUAGE_CAPTIONS = {}
if not os.path.exists(sumolib.translation.LOCALEDIR) and os.path.exists(os.path.join(_THIS_DIR, 'locale')):
    # monkey patch locale dir for the SUMO game
    sumolib.translation.LOCALEDIR = os.path.join(_THIS_DIR, 'locale')


def updateLocalMessages():
    global _LANGUAGE_CAPTIONS
    _LANGUAGE_CAPTIONS = {'title': TL('Interactive Traffic Light'),
                          'rail': TL('Railway Control'),
                          'rail_demo': TL('Railway Control (Demo)'),
                          'fokr_bs_demo': TL('Research intersection Braunschweig (Demo)'),
                          'fkk_in': TL('Research intersection Ingolstadt'),
                          'cross': TL('Simple Junction'),
                          'cross_demo': TL('Simple Junction (Demo)'),
                          'square': TL('Four Junctions'),
                          'grid6': TL('Six Junctions'),
                          'kuehne': TL('Prof. Kühne'),
                          'bs3d': TL('3D Junction Virtual World'),
                          'bs3Dosm': TL('3D Junction OpenStreetMap'),
                          'highway': TL('Highway Ramp'),
                          'ramp': TL('Combined Highway On and Off Ramp'),
                          'corridor': TL('Corridor'),
                          'A10KW': TL('Highway Ramp A10'),
                          'DRT': TL('Demand Responsive Transport (new)'),
                          'DRT2': TL('DRT - Advanced (new)'),
                          'DRT_demo': TL('DRT - Demo'),
                          'high': TL('Highscore'),
                          'reset': TL('Reset Highscore'),
                          'german': TL('German'),
                          'english': TL('English'),
                          'italian': TL('Italian'),
                          'spanish': TL('Spanish'),
                          'french': TL('French'),
                          'chinese (simplified)': TL('Chinese (simplified)'),
                          'chinese (traditional)': TL('Chinese (traditional)'),
                          'quit': TL("Quit"),
                          'Highscore': TL("Highscore"),
                          'Congratulations': TL("Congratulations!"),
                          'your score': TL('Your Score'),
                          'Continue': TL('Continue'),
                          }


def printDebug(*args):
    if _DEBUG:
        print("DEBUG:", end=" ")
        for message in args:
            print(message, end=" ")
        print()


if _UPLOAD:
    printDebug("import http.client...")
    try:
        import http.client as httplib  # noqa
        import socket  # noqa
        host = socket.gethostbyname(_SCORESERVER)
        s = socket.create_connection((host, 80), 2)
        s.close()
        printDebug("SUCCESS")
    except Exception:
        printDebug("FAILED - disabling upload...")
        _UPLOAD = False


def computeScoreFromWaitingTime(gamename, maxScore=10000):
    totalArrived = 0
    totalWaitingTime = 0
    complete = True
    for s in sumolib.xml.parse(os.path.join(BASE, "output", gamename + ".stats.xml"), ("performance", "vehicleTripStatistics")):
        if s.name == "performance":
            if float(s.end) != parseEndTime(os.path.join(BASE, gamename + ".sumocfg")):
                return 0, 0, False
        else:
            totalWaitingTime = float(s.waitingTime) * float(s.count)
            totalArrived = float(s.count)
    score = maxScore - totalWaitingTime
    return score, totalArrived, complete


def computeScoreFromTimeLoss(gamename, maxScore=10000):
    totalArrived = 0
    timeLoss = None
    departDelay = None
    departDelayWaiting = None
    inserted = None
    running = None
    waiting = None
    completed = False

    for line in open(os.path.join(BASE, "output", gamename + ".log")):
        if "Reason: The final simulation step has been reached." in line:
            completed = True
        m = re.search('Inserted: ([0-9]*)', line)
        if m:
            inserted = float(m.group(1))
        m = re.search('Running: (.*)', line)
        if m:
            running = float(m.group(1))
        m = re.search('Waiting: (.*)', line)
        if m:
            waiting = float(m.group(1))
        m = re.search('TimeLoss: (.*)', line)
        if m:
            timeLoss = float(m.group(1))
        m = re.search('DepartDelay: (.*)', line)
        if m:
            departDelay = float(m.group(1))
        m = re.search('DepartDelayWaiting: (.*)', line)
        if m:
            departDelayWaiting = float(m.group(1))
    if not completed or timeLoss is None:
        return 0, totalArrived, False
    else:
        totalArrived = inserted - running
        if _DEBUG:
            print("timeLoss=%s departDelay=%s departDelayWaiting=%s inserted=%s running=%s waiting=%s" % (
                timeLoss, departDelay, departDelayWaiting, inserted, running, waiting))

        score = maxScore - int(100 * ((timeLoss + departDelay) * inserted +
                                      departDelayWaiting * waiting) / (inserted + waiting))
        return score, totalArrived, True


def computeScoreDRT(gamename):
    rideWaitingTime = 0
    rideDuration = 0
    rideStarted = 0
    rideFinished = 0
    rideCount = 0
    for ride in sumolib.xml.parse(os.path.join(BASE, "output", gamename + ".tripinfos.xml"), 'ride'):
        if float(ride.waitingTime) < 0:
            if _DEBUG:
                print("negative waitingTime")
            ride.waitingTime = 10000
        rideWaitingTime += float(ride.waitingTime)
        if float(ride.duration) >= 0:
            rideDuration += float(ride.duration)
            rideStarted += 1
        if float(ride.arrival) >= 0:
            rideFinished += 1

        rideCount += 1

    if rideCount == 0:
        return 0, 0, False
    else:
        avgWT = rideWaitingTime / rideCount
        avgDur = 0 if rideStarted == 0 else rideDuration / rideStarted
        score = 5000 - int(avgWT + avgDur)
        if _DEBUG:
            print("rideWaitingTime=%s rideDuration=%s persons=%s started=%s finished=%s avgWT=%s avgDur=%s" % (
                rideWaitingTime, rideDuration, rideCount, rideStarted, rideFinished, avgWT, avgDur))
        return score, rideCount, True


def computeScoreRail(gamename):
    expectedMeanWait = 360
    rideCount = 0
    score = 0
    for ride in sumolib.xml.parse(os.path.join(BASE, "output", gamename + ".tripinfos.xml"), 'ride'):
        wt = float(ride.waitingTime)
        if wt < 0:
            if _DEBUG:
                print("negative waitingTime")
            wt = 1000
        factor = min(1, expectedMeanWait / wt)
        if float(ride.arrival) >= 0:
            score += 100 * factor
        elif float(ride.duration) >= 0:
            score += 50 * factor
        rideCount += 1
    if rideCount == 0:
        return 0, 0, False
    else:
        return int(score), rideCount, True


def computeScoreSquare(gamename):
    maxScore = 1000.0
    expectedVehCount = 142
    timeLoss = 0
    tripCount = 0
    arrived = 0
    for trip in sumolib.xml.parse(os.path.join(BASE, "output", gamename + ".tripinfos.xml"), 'tripinfo'):
        timeLoss += float(trip.timeLoss) + float(trip.departDelay)
        tripCount += 1
        if float(trip.arrival) > 0:
            arrived += 1

    if tripCount == 0:
        return 0, 0, False
    else:
        # early-abort score is close to 0, do-nothing timeLoss is ~8000
        earlyEndPenalty = (expectedVehCount - tripCount) * (maxScore / expectedVehCount)
        score = int(1000 - earlyEndPenalty - timeLoss / 10.0)
        if _DEBUG:
            print("tripCount=%s arrived=%s timeLoss=%s avtTimeLoss=%s earlyEndPenalty=%s" % (
                tripCount, arrived, timeLoss, timeLoss / tripCount, earlyEndPenalty))
        return score, arrived, True


_SCORING_FUNCTION = defaultdict(lambda: computeScoreFromWaitingTime)
_SCORING_FUNCTION.update({
    'A10KW': lambda name: computeScoreFromTimeLoss(name, 20000),
    'corridor': lambda name: computeScoreFromWaitingTime(name, 30000),
    'highway': lambda name: computeScoreFromTimeLoss(name, 13000),
    'DRT': computeScoreDRT,
    'DRT2': computeScoreDRT,
    'DRT_demo': computeScoreDRT,
    'square': computeScoreSquare,
    'rail': computeScoreRail,
    'rail_demo': computeScoreRail,
})


def computeScore(gamename):
    return _SCORING_FUNCTION[gamename](gamename)


def parseEndTime(cfg):
    cfg_doc = pulldom.parse(cfg)
    for event, parsenode in cfg_doc:
        if event == pulldom.START_ELEMENT and parsenode.localName == 'end':
            return float(parsenode.getAttribute('value'))


class IMAGE:
    pass


class StartDialog(Tkinter.Frame):

    def __init__(self, parent, lang, langCode):
        Tkinter.Frame.__init__(self, parent)
        # variables for changing language
        self.parent = parent
        self._language_text = lang
        self.buttons = []
        # misc variables
        self.name = ''
        # setup gui
        self.parent.title(self._language_text['title'])
        self.parent.minsize(250, 50)
        self.category = None
        self.highscoreLocation = Tkinter.StringVar(value="local")
        self.high = self.loadHighscore(self.highscoreLocation.get())

        # we use a grid layout with 4 columns
        COL_DLRLOGO, COL_START, COL_HIGH, COL_SUMOLOGO = range(4)
        # there is one column for every config, +2 more columns for control
        # buttons
        configs = sorted(glob.glob(os.path.join(BASE, "*.sumocfg")))
        demos = [cfg for cfg in configs if "demo" in cfg]
        numButtons = len(configs) + 3 - len(demos)
        # button dimensions
        bWidth_start = 40
        bWidth_high = 10

        self.gametime = 0
        self.ret = 0
        haveOSG = "OSG" in subprocess.check_output(sumolib.checkBinary("sumo"), universal_newlines=True)

        # 2 button for each config (start, highscore)

        demo = 0
        row = 0
        for cfg in configs:
            if "bs3" in cfg and not haveOSG:
                continue
            category = self.category_name(cfg)
            # lambda must make a copy of cfg argument
            button = Tkinter.Button(self, width=bWidth_start,
                                    command=lambda cfg=cfg: self.start_cfg(cfg))
            self.addButton(button, category)
            if cfg in demos:
                button.grid(row=numButtons - demo, column=COL_SUMOLOGO)
                demo += 1
                continue

            button.grid(row=row, column=COL_START)

            button = Tkinter.Button(self, width=bWidth_high,
                                    command=lambda cfg=cfg: ScoreDialog(self, [], None, self.category_name(cfg),
                                                                        self._language_text, self.high)
                                    )  # .grid(row=row, column=COL_HIGH)
            self.addButton(button, 'high')
            button.grid(row=row, column=COL_HIGH)
            row += 1

        # some pretty images
        Tkinter.Label(self, image=IMAGE.dlrLogo).grid(row=0, rowspan=numButtons-3, column=COL_DLRLOGO)
        Tkinter.Label(self, image=IMAGE.sumoLogo).grid(row=0, rowspan=numButtons-3, column=COL_SUMOLOGO)

        # control buttons
        if _UPLOAD:
            r1 = Tkinter.Radiobutton(self, text='Local Highscores', value='local', variable=self.highscoreLocation)
            r1.grid(row=numButtons-3, column=COL_DLRLOGO)
            r2 = Tkinter.Radiobutton(self, text='Global Highscores', value='global', variable=self.highscoreLocation)
            r2.grid(row=numButtons-2, column=COL_DLRLOGO)
        button = Tkinter.Button(self, width=bWidth_start, command=self.clear)
        self.addButton(button, 'reset')
        button.grid(row=numButtons - 1, column=COL_DLRLOGO)

        # language selection
        self.langChoices = {
            "de": 'german',
            "en": 'english',
            "es": 'spanish',
            "it": 'italian',
            "fr": 'french',
            "zh": 'chinese (simplified)',
            "zh-Hant": 'chinese (traditional)',
        }
        self._langCode = langCode
        self.langDrop = Tkinter.Listbox(self, height=len(demos), selectmode=Tkinter.SINGLE, width=bWidth_high)
        self.langDrop.bind('<<ListboxSelect>>', self.change_language)
        self.scrollbar = Tkinter.Scrollbar(self, orient=Tkinter.VERTICAL)
        self.scrollbar.config(command=self.langDrop.yview)
        self.langDrop.config(yscrollcommand=self.scrollbar.set)
        self.updateLanguageMenu()
        self.langDrop.grid(row=numButtons - len(demos) + 1, column=COL_HIGH, rowspan=len(demos), sticky="nsew")
        self.scrollbar.grid(row=numButtons - len(demos) + 1, column=COL_SUMOLOGO, rowspan=len(demos), sticky="nsw")

        button = Tkinter.Button(self, width=bWidth_start, command=sys.exit)
        self.addButton(button, 'quit')
        button.grid(row=numButtons, column=COL_START)

        self.grid()
        # The following three commands are needed so the window pops
        # up on top on Windows...
        self.parent.iconify()
        self.parent.update()
        self.parent.deiconify()

    def updateLanguageMenu(self):
        self.langDrop.delete(0, Tkinter.END)
        langChoices = [self._language_text.get(longName, longName) for longName in self.langChoices.values()]
        scrollPos = self.scrollbar.get()
        self.langDrop.insert(0, *langChoices)
        self.langDrop.activate(list(self.langChoices.keys()).index(self._langCode))
        self.langDrop.yview_moveto(scrollPos[0])

    def addButton(self, button, text, key=None):
        button["text"] = self._language_text.get(text, text)
        if key is None:
            key = text
        self.buttons.append((key, button))

    def change_language(self, *args):
        selection = self.langDrop.curselection()
        if len(selection) == 0:
            return
        chosenLang = self.langDrop.get(selection[0])
        for code, longName in self.langChoices.items():
            if self._language_text[longName] == chosenLang:
                self._langCode = code
                break
        setLanguage(self._langCode)
        updateLocalMessages()
        self._language_text = _LANGUAGE_CAPTIONS

        # update language menu
        self.updateLanguageMenu()

        for key, button in self.buttons:
            button["text"] = self._language_text[key]
        self.parent.title(self._language_text['title'])

    def category_name(self, cfg):
        return os.path.basename(cfg)[:-8]

    def loadHighscore(self, location):
        if location == "global":
            printDebug("try to load highscore from scoreserver...")
            try:
                conn = httplib.HTTPConnection(_SCORESERVER, timeout=_TIMEOUT)
                conn.request("GET", _SCORESCRIPT + "top=" + str(_SCORES))
                response = conn.getresponse()
                if response.status == httplib.OK:
                    scores = {}
                    for line in response.read().splitlines():
                        category, values = line.split()
                        scores[category] = _SCORES * [("", "", -1.)]
                        for idx, item in enumerate(values.split(':')):
                            name, game, score = item.split(',')
                            scores[category][idx] = (name, game, int(float(score)))
                    printDebug("SUCCESS")
                    return scores
            except Exception:
                printDebug("FAILED")
        try:
            with open(_SCOREFILE, 'rb') as sf:
                return pickle.load(sf)
        except Exception as e:
            print(e)
            pass
        if os.path.exists(REFSCOREFILE):
            with open(REFSCOREFILE, 'rb') as sf:
                return pickle.load(sf)
        return {}

    def clear(self):
        self.high.clear()
        if os.path.exists(REFSCOREFILE):
            with open(REFSCOREFILE, 'rb') as sf:
                self.high.update(pickle.load(sf))

    def start_cfg(self, cfg):
        # remember which cfg was launched
        self.category = self.category_name(cfg)
        if _DEBUG:
            print("starting", cfg)
        self.gametime = parseEndTime(cfg)
        binary = sumolib.checkBinary("sumo-gui", BASE)
        if binary == "sumo-gui":  # fallback in case SUMO_HOME env is not defined
            binary = sumolib.checkBinary("sumo-gui", os.path.join(SUMO_HOME, "bin"))
        os.makedirs(os.path.join(BASE, "output"), exist_ok=True)
        self.ret = subprocess.call(
            [binary, "-S", "-G", "-Q", "-c", cfg, '-l', 'log',
                '--output-prefix', os.path.join("output", self.category + "."), '--language', self._langCode,
                '--duration-log.statistics', '--statistic-output', 'stats.xml',
                '--tripinfo-output.write-unfinished'], stderr=sys.stderr)

        if _DEBUG:
            print("ended", cfg)
        # ensure files are fully written
        time.sleep(1)

        # compute score
        score, totalArrived, complete = _SCORING_FUNCTION[self.category](self.category)

        # parse switches
        switch = []
        lastProg = {}
        tlsfile = os.path.join(BASE, self.category, "tlsstate.xml")
        if os.path.exists(tlsfile):
            for line in open(tlsfile):
                m = re.search(r'tlsstate time="(\d+(.\d+)?)" id="([^"]*)" programID="([^"]*)"', line)
                if m:
                    tls = m.group(3)
                    program = m.group(4)
                    if tls not in lastProg or lastProg[tls] != program:
                        lastProg[tls] = program
                        switch += [m.group(3), m.group(1)]

        lang = self._language_text
        if _DEBUG:
            print(switch, score, totalArrived, complete)
        if complete:
            ScoreDialog(self, switch, score, self.category, lang, self.high)

        # if ret != 0:
        # quit on error
        #    sys.exit(start.ret)


class ScoreDialog:

    def __init__(self, parent, switch, score, category, lang, high):
        self.root = Tkinter.Toplevel(parent)
        # self.root.transient(parent)
        self.name = None
        self.switch = switch
        self.score = score
        self.category = category
        self.root.title("%s%s" % (lang["Highscore"], ": " + lang[self.category] if self.category in lang else ""))
        self.root.minsize(250, 50)
        self.high = high

        haveHigh = False

        if category not in self.high:
            self.high[category] = _SCORES * [("", "", -1.)]
        if len(self.high[category]) < _SCORES:
            self.high[category] += (_SCORES - len(self.high[category])) * [("", "", -1.)]
        idx = 0
        for n, g, p in self.high[category]:
            if not haveHigh and score is not None and p < score:
                Tkinter.Label(self.root, text=(str(idx + 1) + '. ')).grid(row=idx)
                self.name = Tkinter.Entry(self.root)
                self.name.grid(row=idx, sticky=Tkinter.W, column=1)
                self.scoreLabel = Tkinter.Label(self.root, text="%.2f" % score,
                                                bg="pale green").grid(row=idx, column=2)
                self.idx = idx
                haveHigh = True
                self.root.title(lang["Congratulations"])
                idx += 1
            if p == -1 or idx == _SCORES:
                break
            Tkinter.Label(self.root, text=(str(idx + 1) + '. ')).grid(row=idx)
            Tkinter.Label(self.root, text=n, padx=5).grid(
                row=idx, sticky=Tkinter.W, column=1)
            Tkinter.Label(self.root, text="%.2f" % p).grid(row=idx, column=2)
            idx += 1
        if not haveHigh:
            if score is not None:  # not called from the main menue
                Tkinter.Label(self.root, text=lang['your score'], padx=5,
                              bg="indian red").grid(row=idx, sticky=Tkinter.W, column=1)
                Tkinter.Label(self.root, text="%.2f" % score,
                              bg="indian red").grid(row=idx, column=2)
                idx += 1
        Tkinter.Button(self.root, text=lang["Continue"], command=self.save).grid(
            row=idx, column=2)

        # add QR-code for LNDW
        Tkinter.Label(self.root, image=IMAGE.qrCode).grid(
            row=1, column=3, rowspan=22)

        self.root.grid()
        self.root.bind("<Return>", self.save)
        # self.root.wait_visibility()
        # self.root.grab_set()
        if self.name:
            self.name.focus_set()
        # The following three commands are needed so the window pops
        # up on top on Windows...
        # self.root.iconify()
        # self.root.update()
        # self.root.deiconify()
        # self.root.mainloop()

    def save(self, event=None):
        if self.name and self.name.get():
            name = self.name.get()
            self.high[self.category].insert(self.idx, (name, self.switch, self.score))
            self.high[self.category].pop()
            self.name.destroy()
            self.name = None
            Tkinter.Label(self.root, text=name, padx=5,
                          bg="pale green").grid(row=self.idx, sticky=Tkinter.W, column=1)
            try:
                f = open(_SCOREFILE, 'wb')
                pickle.dump(self.high, f)
                f.close()
            except Exception as e:
                print(e)
                pass

            if _UPLOAD:
                printDebug("try to upload score...")
                try:
                    conn = httplib.HTTPConnection(_SCORESERVER, timeout=_TIMEOUT)
                    conn.request("GET", _SCORESCRIPT + "category=%s&name=%s&instance=%s&points=%s" % (
                        self.category, name, "_".join(self.switch), self.score))
                    if _DEBUG:
                        r1 = conn.getresponse()
                        print(r1.status, r1.reason, r1.read())
                    printDebug("SUCCESS")
                except BaseException:
                    printDebug("FAILED")
        self.quit()

    def quit(self, event=None):
        self.root.destroy()


def main():
    stereoModes = ('ANAGLYPHIC', 'QUAD_BUFFER', 'VERTICAL_SPLIT', 'HORIZONTAL_SPLIT')
    optParser = sumolib.options.ArgumentParser()
    optParser.add_option("-s", "--stereo", metavar="OSG_STEREO_MODE",
                         help=("Defines the stereo mode to use for 3D output; unique prefix of %s" % (
                               ", ".join(stereoModes))))
    optParser.add_option("add", nargs="*", help="additional flags: {debug|noupload}")

    addLanguageOption(optParser)
    options = optParser.parse_args()
    setLanguage(options.language)
    updateLocalMessages()

    if options.stereo:
        for m in stereoModes:
            if m.lower().startswith(options.stereo.lower()):
                os.environ["OSG_STEREO_MODE"] = m
                os.environ["OSG_STEREO"] = "ON"
                break

    if "OSG_FILE_PATH" in os.environ:
        os.environ["OSG_FILE_PATH"] += os.pathsep + \
            os.path.join(os.environ.get("SUMO_HOME", ""), "data", "3D")
    else:
        os.environ["OSG_FILE_PATH"] = os.path.join(
            os.environ.get("SUMO_HOME", ""), "data", "3D")

    root = Tkinter.Tk()
    IMAGE.dlrLogo = Tkinter.PhotoImage(file=os.path.join(BASE, 'dlr.gif'))
    IMAGE.sumoLogo = Tkinter.PhotoImage(file=os.path.join(BASE, 'sumo_logo.gif'))
    IMAGE.qrCode = Tkinter.PhotoImage(file=os.path.join(BASE, 'qr_sumo.dlr.de.gif'))
    StartDialog(root, _LANGUAGE_CAPTIONS, options.language)
    root.mainloop()


if __name__ == "__main__":
    main()
