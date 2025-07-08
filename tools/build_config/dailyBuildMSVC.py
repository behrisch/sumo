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

# @file    dailyBuildMSVC.py
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @author  Laura Bieker
# @date    2008

"""
Does the nightly git pull on the windows server and the visual
studio build. The script is also used for the meso build.
Some paths especially for the names of the texttest output dirs are
hard coded into this script.
"""
from __future__ import absolute_import
from __future__ import print_function
import datetime
import os
import glob
import zipfile
import shutil
import sys

import buildWindowsSUMOWheel
import status
import wix

env = os.environ
if "SUMO_HOME" not in env:
    env["SUMO_HOME"] = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SUMO_HOME = env["SUMO_HOME"]
env["PYTHON"] = "python"
env["SMTP_SERVER"] = "smtprelay.dlr.de"

sys.path += [os.path.join(SUMO_HOME, "tools"), os.path.join(SUMO_HOME, "tests")]
import sumolib  # noqa
import runExtraTests  # noqa

try:
    import matplotlib
    MATPLOTLIB_CACHE = matplotlib.get_cachedir()
except ImportError:
    MATPLOTLIB_CACHE = None

BINARIES = ("activitygen", "emissionsDrivingCycle", "emissionsMap",
            "dfrouter", "duarouter", "jtrrouter", "marouter",
            "netconvert", "netedit", "netgenerate",
            "od2trips", "polyconvert", "sumo", "sumo-gui",
            "TraCITestClient")


def repositoryUpdate(options):
    gitrev = ""
    cwd = os.getcwd()
    for d in options.repositories.split(","):
        os.chdir(os.path.join(options.rootDir, d))
        status.log_subprocess(["git", "clean", "-f", "-x", "-d", "-q"])
        status.log_subprocess(["git", "pull"])
        status.log_subprocess(["git", "submodule", "update"])
        if gitrev == "":
            gitrev = sumolib.version.gitDescribe()
    os.chdir(cwd)
    return gitrev


def runTests(options, env, gitrev, debugSuffix=""):
    if not options.tests:
        return
    prefix = env["FILEPREFIX"] + debugSuffix
    env["SUMO_BATCH_RESULT"] = os.path.join(options.rootDir, prefix + "batch_result")
    env["SUMO_REPORT"] = os.path.join(options.rootDir, prefix + "report")
    env["TEXTTEST_TMP"] = os.path.join(options.rootDir, prefix + "tmp")
    env["TEXTTEST_HOME"] = os.path.join(SUMO_HOME, "tests")
    shutil.rmtree(env["TEXTTEST_TMP"], True)
    if MATPLOTLIB_CACHE:
        shutil.rmtree(MATPLOTLIB_CACHE, True)
    if not os.path.exists(env["SUMO_REPORT"]):
        os.makedirs(env["SUMO_REPORT"])
    for name in BINARIES:
        binary = os.path.join(SUMO_HOME, "bin", name + debugSuffix + ".exe")
        if name == "sumo-gui":
            if os.path.exists(binary):
                env["GUISIM_BINARY"] = binary
        elif os.path.exists(binary):
            env[name.upper() + "_BINARY"] = binary
    # provide more information than just the date:
    fullOpt = ["-b", prefix, "-name", "%sr%s" %
               (datetime.date.today().strftime("%d%b%y"), gitrev)]
    ttBin = "texttest"
    if options.suffix == "extra":
        runExtraTests.run(debugSuffix, fullOpt, True, debugSuffix == "")
    else:
        status.log_subprocess([ttBin] + fullOpt, env)
        status.log_subprocess([ttBin, "-a", "sumo.gui"] + fullOpt, env)
    status.log_subprocess([ttBin, "-b", env["FILEPREFIX"], "-coll"], env)
    shutil.copytree(env["SUMO_REPORT"], os.path.join(options.remoteDir, prefix + "report"), dirs_exist_ok=True)
    status.killall((debugSuffix,), BINARIES)


def generateCMake(generator, platform, checkOptionalLibs, python):
    buildDir = os.path.join(SUMO_HOME, "build", "cmake-build-" + platform)
    cmakeOpt = ["-DCOMPILE_DEFINITIONS=MSVC_TEST_SERVER",
                "-DCHECK_OPTIONAL_LIBS=%s" % checkOptionalLibs]
    if python:
        cmakeOpt += ["-DPYTHON_EXECUTABLE=%s" % python]
    if checkOptionalLibs:
        cmakeOpt += ["-DSUMO_UTILS=True"]
    os.makedirs(buildDir)
    status.printLog("Creating solution for %s." % generator)
    status.log_subprocess(["cmake", "../..", "-G", generator, "-A", platform] + cmakeOpt, cwd=buildDir)
    return buildDir


def main(options, platform="x64"):
    env["FILEPREFIX"] = options.msvc_version + options.suffix + platform
    prefix = os.path.join(options.rootDir, env["FILEPREFIX"])
    makeLog = prefix + "Release.log"
    makeAllLog = prefix + "Debug.log"
    testLog = prefix + "Test.log"
    testDebugLog = prefix + "DebugTest.log"
    statusLog = prefix + "status.log"
    log_handler = status.set_rotating_log(makeLog)

    status.killall(("", "D"), BINARIES)
    status.printLog("Running %s build using python %s." % (options.msvc_version, sys.version))
    gitrev = repositoryUpdate(options)
    generator = "Visual Studio " + ("12 2013" if options.msvc_version == "msvc12" else "16 2019")
    buildDir = generateCMake(generator, platform, options.suffix == "extra", options.python)
    ret = status.log_subprocess(["cmake", "--build", ".", "--config", "Release"], cwd=buildDir)
    status.log_subprocess(["cmake", "--build", ".", "--config", "Release", "--target", "lisum"], cwd=buildDir)
    status.log_subprocess(["cmake", "--build", ".", "--config", "Release", "--target", "userdoc", "examples"],
                          cwd=buildDir)
    status.log_subprocess(["cmake", "--install", "."], cwd=buildDir)
    plat = platform.lower().replace("x", "win")
    if options.msvc_version != "msvc16":
        plat += options.msvc_version
    for d in glob.glob(os.path.join(buildDir, "sumo-*")):
        if os.path.isdir(d):
            installDir = d
    installBase = os.path.basename(installDir)
    binaryZip = os.path.join(buildDir, "sumo-%s%s-%s" % (plat, options.suffix, installBase[5:]))
    if ret == 0:
        try:
            for f in (glob.glob(os.path.join(SUMO_HOME, "*.md")) +
                      [os.path.join(SUMO_HOME, n) for n in ("AUTHORS", "ChangeLog", "CITATION.cff", "LICENSE")]):
                shutil.copy(f, installDir)
            if options.suffix == "extra":
                shutil.copy(os.path.join(SUMO_HOME, "build_config", "wix", "gpl-2.0.txt"),
                            os.path.join(installDir, "LICENSE"))
            for f in glob.glob(os.path.join(SUMO_HOME, "bin", "*.jar")):
                shutil.copy(f, os.path.join(installDir, "bin"))
            for f in glob.glob(os.path.join(SUMO_HOME, "bin", "*-sources.zip")):
                shutil.unpack_archive(f, os.path.join(installDir, "include"))
            if options.suffix == "extra" and os.path.exists(os.path.join(options.remoteDir, "cadyts.jar")):
                shutil.copy(os.path.join(options.remoteDir, "cadyts.jar"), os.path.join(installDir, "bin"))
            shutil.copytree(os.path.join(SUMO_HOME, "docs"), os.path.join(installDir, "docs"),
                            ignore=shutil.ignore_patterns('web'))
            for lib in ("libsumo", "libtraci"):
                shutil.rmtree(os.path.join(installDir, "tools", lib), ignore_errors=True)
            shutil.copy(os.path.join(buildDir, "src", "version.h"), os.path.join(installDir, "include"))
            status.printLog("Creating sumo.zip.")
            shutil.make_archive(binaryZip, 'zip', buildDir, installBase)
            shutil.copy(binaryZip + ".zip", options.remoteDir)
            status.printLog("Creating sumo.msi.")
            if options.suffix == "extra":
                wix.buildMSI(binaryZip + ".zip", binaryZip + ".msi",
                             license_path=os.path.join(SUMO_HOME, "build_config", "wix", "gpl-2.0.rtf"))
            else:
                wix.buildMSI(binaryZip + ".zip", binaryZip + ".msi")
            shutil.copy(binaryZip + ".msi", options.remoteDir)
        except Exception as ziperr:
            status.printLog("Warning: Could not zip to %s.zip (%s)!" % (binaryZip, ziperr))

    gameZip = os.path.join(buildDir, "sumo-game-%s%s-%s.zip" % (plat, options.suffix, installBase[5:]))
    status.printLog("Creating sumo-game.zip.")
    try:
        status.log_subprocess(["cmake", "--build", ".", "--target", "game"], cwd=buildDir)
        shutil.move(os.path.join(buildDir, "sumo-game.zip"), gameZip)
        shutil.copy(gameZip, options.remoteDir)
    except Exception as e:
        status.printLog("Warning: Could not create nightly sumo-game.zip! (%s)" % e)

    if options.suffix == "extra":
        try:
            buildWindowsSUMOWheel.main()
            f = glob.glob(os.path.join(SUMO_HOME, "dist", "eclipse_sumo-*"))[0]
            shutil.copy(f, os.path.join(options.remoteDir, "wheels"))
        except Exception as e:
            status.printLog("Warning: Could not create nightly sumo wheel! (%s)" % e)

    debug_handler = status.set_rotating_log(makeAllLog, log_handler)
    ret = status.log_subprocess(["cmake", "--build", ".", "--config", "Debug"], cwd=buildDir)
    if ret == 0:
        debugZip = os.path.join(buildDir, "sumo-%s%sDebug-%s.zip" % (plat, options.suffix, installBase[5:]))
        status.printLog("Creating sumoDebug.zip.")
        try:
            with zipfile.ZipFile(debugZip, 'w', zipfile.ZIP_DEFLATED) as zipf:
                for ext in ("*D.exe", "*.dll", "*D.pdb"):
                    for f in glob.glob(os.path.join(SUMO_HOME, "bin", ext)):
                        zipf.write(f, os.path.join(installBase, "bin", os.path.basename(f)))
            shutil.copy(debugZip, options.remoteDir)
        except IOError as ziperr:
            status.printLog("Warning: Could not zip to %s (%s)!" % (debugZip, ziperr))

    log_handler = status.set_rotating_log(testLog, debug_handler)
    status.printLog("Running tests.")
    runTests(options, env, gitrev)
    with open(statusLog, 'w') as log:
        status.printStatus(makeLog, makeAllLog, env["SMTP_SERVER"], log, testLog=testLog)
    for f in (makeLog, makeAllLog, statusLog, testLog):
        shutil.copy(f, options.remoteDir)
    if not options.x64only:
        debug_handler = status.set_rotating_log(testDebugLog, log_handler)
        status.printLog("Running debug tests.")
        runTests(options, env, gitrev, "D")
        with open(prefix + "Dstatus.log", 'w') as log:
            status.printStatus(makeAllLog, testDebugLog, env["SMTP_SERVER"], log, testLog=testDebugLog)
        for f in (testDebugLog, log.name):
            shutil.copy(f, options.remoteDir)


if __name__ == "__main__":
    optParser = sumolib.options.ArgumentParser()
    optParser.add_option("-r", "--root-dir", dest="rootDir",
                         default=r"D:\Sumo", help="root for git and log output")
    optParser.add_option("-s", "--suffix", default="", help="suffix to the fileprefix")
    optParser.add_option("-m", "--remote-dir", dest="remoteDir", default="S:\\daily",
                         help="directory to move the results to")
    optParser.add_option("-n", "--no-tests", dest="tests", action="store_false",
                         default=True, help="skip tests")
    optParser.add_option("-x", "--x64only", action="store_true",
                         default=False, help="skip debug build")
    optParser.add_option("-p", "--python", help="path to python interpreter to use")
    optParser.add_option("--msvc-version", default="msvc16",
                         help="Visual Studio version to use (either msvc12 or msvc16)")
    optParser.add_option("-u", "--repositories", default="git",
                         help="repositories to update")
    main(optParser.parse_args())
